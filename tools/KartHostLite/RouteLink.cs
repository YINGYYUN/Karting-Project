using System.IO.Ports;

namespace KartHost;

/// <summary>
/// 路表串口链路（PC 端）。协议与车端 <c>project/code/route_link.c</c> 逐字节对应。
///
/// <para><b>走哪条线</b>：DAP 自带的虚拟串口（车端 <c>UART_0</c>，本机一般显示为 COMx）。
/// 不走 WiFi 的原因写在 <c>route_link.h</c> 里：车端 WiFi 下行是**定长 15 B** 的，
/// 一张表 32 KB 要改下行协议核心 + 自己做分片重传；串口是自己的字节流，一层都没有。</para>
///
/// <para><b>帧</b>：<c>A5 5A | type(1) | len(2,LE) | payload | crc16(2,LE)</c>，
/// crc16 = CRC16-MODBUS（初值 0xFFFF / 反射多项式 0xA001）覆盖 type|len|payload。</para>
///
/// <para><b>槽的纪律</b>：槽 0 是"手推录制"的家，**下发只允许写 1..N-1**（车端协议层也会拒）。
/// 所以灌表不可能覆盖手推的那张表。</para>
///
/// <para>⚠ 本工程从队友 <c>RouteLinkSerial.cs</c> 移植，改了三处：</para>
/// <list type="number">
///   <item>镜像格式改成 **v2**（<c>ver=2</c>、记录 **16 B**、单槽 2048 点 / 32804 B）——
///         队友那份的 <c>VerifyImage()</c> 还停在 v1 的 12 B，用它校验我们的镜像会一律判"不过"；</item>
///   <item>去掉转向阶跃（本工程没有 steer_step 模块）；</item>
///   <item>去掉"固件旧"那句硬编码路径提示，改成指本工程自己的 hex。</item>
/// </list>
/// </summary>
public sealed class RouteLink : IDisposable
{
    // ---- 帧类型（与 route_link.h 对齐，别改数值）----
    public const byte Sync0 = 0xA5, Sync1 = 0x5A;
    public const byte TPing = 0x01, TDirReq = 0x02, TLoadBegin = 0x03, TLoadData = 0x04;
    public const byte TLoadCommit = 0x05, TLoadAbort = 0x06, TSlotSel = 0x07, TSlotLoad = 0x08;
    public const byte TSlotRead = 0x09;
    public const byte TPong = 0x81, TDirData = 0x82, TAck = 0x85, TLoadDone = 0x86, TSlotData = 0x87;

    public const int PayloadMax = 768;
    public const int ChunkMax = 512;

    // ---- 镜像格式（与 ins_route.c 的 RT_MAGIC / RT_VER / rt_rec_t 一致）----
    public const uint ImgMagic = 0x4B525443;   // "CTRK"
    public const ushort ImgVer = 2;            // v2 = 16 B/条
    public const ushort RecSize = 16;
    public const int SlotCapacity = 2048;      // 每槽最多多少条
    public const int SlotMaxBytes = 36 + SlotCapacity * RecSize;   // = 32804

    /// <summary>ACK 的 result 码（与 route_link.h 的 RL_ERR_* 一致，别改数值）</summary>
    public static readonly string[] ResultName =
    {
        "OK", "槽号越界/写了槽0", "尺寸超限", "offset 不连续(丢帧)", "镜像校验不过",
        "没先 LOAD_BEGIN", "正在写 Flash", "状态不允许",
    };

    // ---- 能力位（PONG v2 起）----
    public const uint CapDir = 1u << 0, CapLoad = 1u << 1, CapSlotSel = 1u << 2, CapSlotRead = 1u << 3;

    /// <summary>车端固件不支持时要看哪句 —— 路径写在这里，免得现场翻不到。</summary>
    public const string StaleFirmwareHint =
        "车端固件比上位机旧：重刷 Karting-Project\\project\\iar\\project_config\\Debug_m7_0\\Exe\\"
        + "cyt4bb7_cm_7_0.hex（★ 两个核都要刷），然后断开重连让它重新 PONG 握手。";

    private SerialPort _sp;
    private readonly List<(byte T, byte[] P)> _pending = new();
    private readonly List<byte> _acc = new();
    private readonly byte[] _rxRaw = new byte[4096];

    /// <summary>被丢弃的字节（同步不上 / CRC 不过）。&gt;0 说明线上有噪声。</summary>
    public int BadBytes { get; private set; }

    // ---- PING 之后有效 ----
    public int ProtoVer, SlotCount = 5, SlotPages = 17, RecSizeFromCar = RecSize, MaxBytes = SlotMaxBytes;
    public uint Caps { get; private set; }
    public bool CapsKnown { get; private set; }

    public bool CanSlotRead => !CapsKnown || (Caps & CapSlotRead) != 0;

    public bool IsOpen => _sp != null && _sp.IsOpen;
    public string Describe => _sp == null ? "未打开" : $"{_sp.PortName} @ {_sp.BaudRate}";

    // ================================================================ 打开 / 关闭
    public static string[] ListPorts()
    {
        var names = SerialPort.GetPortNames();
        Array.Sort(names, (a, b) =>
        {
            int x = int.TryParse(a.Replace("COM", ""), out var v) ? v : 0;
            int y = int.TryParse(b.Replace("COM", ""), out var w) ? w : 0;
            return x.CompareTo(y);
        });
        return names;
    }

    public void Open(string port, int baud = 115200)
    {
        Close();
        _sp = new SerialPort(port, baud)
        {
            ReadTimeout = 20,
            WriteTimeout = 1000,
            ReadBufferSize = 8192,
            WriteBufferSize = 8192,
        };
        _sp.Open();
        _pending.Clear();
        _acc.Clear();
        BadBytes = 0;
        CapsKnown = false;
        Caps = 0;
        ProtoVer = 0;
    }

    public void Close()
    {
        try { _sp?.Close(); } catch { /* 拔线时可能抛，忽略 */ }
        _sp?.Dispose();
        _sp = null;
    }

    public void Dispose() => Close();

    // ================================================================ CRC / 组帧 / 拆帧
    /// <summary>
    /// CRC16-MODBUS。<b>这条算法在本工程里有四份实现</b>（车端 rl_crc16 / rt_crc16 /
    /// host_crc16 + 这里）。必须算出同一个值 —— 否则现象是"帧看着发出去了、
    /// 车端永远不回 ACK"，而两边都自认为正确。
    /// </summary>
    public static ushort Crc16(ushort crc, byte[] b, int off, int len)
    {
        for (int i = 0; i < len; i++)
        {
            crc ^= b[off + i];
            for (int k = 0; k < 8; k++)
                crc = (crc & 1) != 0 ? (ushort)((crc >> 1) ^ 0xA001) : (ushort)(crc >> 1);
        }
        return crc;
    }

    public static byte[] Build(byte type, byte[] payload = null)
    {
        int n = payload?.Length ?? 0;
        if (n > PayloadMax) throw new ArgumentException($"载荷 {n} 超过上限 {PayloadMax}");
        var buf = new byte[7 + n];
        buf[0] = Sync0; buf[1] = Sync1; buf[2] = type;
        buf[3] = (byte)(n & 0xFF); buf[4] = (byte)((n >> 8) & 0xFF);
        if (n > 0) Buffer.BlockCopy(payload, 0, buf, 5, n);
        ushort c = Crc16(0xFFFF, buf, 2, 3 + n);
        buf[5 + n] = (byte)(c & 0xFF);
        buf[6 + n] = (byte)(c >> 8);
        return buf;
    }

    public static ushort U16(byte[] b, int o) => (ushort)(b[o] | (b[o + 1] << 8));
    public static uint U32(byte[] b, int o)
        => (uint)(b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24));

    public static void PutU32(byte[] b, int o, uint v)
    {
        b[o] = (byte)v; b[o + 1] = (byte)(v >> 8); b[o + 2] = (byte)(v >> 16); b[o + 3] = (byte)(v >> 24);
    }

    private void Write(byte type, byte[] payload = null)
    {
        var f = Build(type, payload);
        _sp.Write(f, 0, f.Length);
    }

    /// <summary>读一次串口并拆帧，把完整帧塞进 _pending。返回本次新收的帧数。</summary>
    private int Pump()
    {
        int got = 0;
        int avail;
        try { avail = _sp.BytesToRead; } catch { return 0; }
        while (avail > 0)
        {
            int n = Math.Min(avail, _rxRaw.Length);
            try { n = _sp.Read(_rxRaw, 0, n); }
            catch (TimeoutException) { break; }
            if (n <= 0) break;
            got += Feed(_rxRaw, n);
            try { avail = _sp.BytesToRead; } catch { break; }
        }
        return got;
    }

    private int Feed(byte[] data, int len)
    {
        int frames = 0;
        for (int i = 0; i < len; i++) _acc.Add(data[i]);

        while (true)
        {
            int si = -1;
            for (int i = 0; i + 1 < _acc.Count; i++)
                if (_acc[i] == Sync0 && _acc[i + 1] == Sync1) { si = i; break; }

            if (si < 0)
            {
                if (_acc.Count > 1) { BadBytes += _acc.Count - 1; _acc.RemoveRange(0, _acc.Count - 1); }
                break;
            }
            if (si > 0) { BadBytes += si; _acc.RemoveRange(0, si); }
            if (_acc.Count < 5) break;

            int plen = _acc[3] | (_acc[4] << 8);
            if (plen > PayloadMax) { BadBytes++; _acc.RemoveAt(0); continue; }

            int total = 7 + plen;
            if (_acc.Count < total) break;

            var f = _acc.GetRange(0, total).ToArray();
            ushort c = Crc16(0xFFFF, f, 2, 3 + plen);
            if (c != U16(f, 5 + plen)) { BadBytes++; _acc.RemoveAt(0); continue; }

            _acc.RemoveRange(0, total);
            var pl = new byte[plen];
            Buffer.BlockCopy(f, 5, pl, 0, plen);
            _pending.Add((f[2], pl));
            frames++;
        }
        return frames;
    }

    /// <summary>等一帧指定类型。返回 null = 超时。</summary>
    private byte[] Wait(byte type, int timeoutMs)
    {
        long t0 = Environment.TickCount64;
        while (true)
        {
            for (int i = 0; i < _pending.Count; i++)
            {
                if (_pending[i].T == type)
                {
                    var p = _pending[i].P;
                    _pending.RemoveAt(i);
                    return p;
                }
            }
            if (Environment.TickCount64 - t0 >= timeoutMs) return null;
            Pump();
            if (_pending.Count == 0) Thread.Sleep(2);
        }
    }

    private (int Res, int Detail) WaitAck(byte cmd, int timeoutMs)
    {
        var p = Wait(TAck, timeoutMs);
        if (p == null || p.Length < 4) return (-1, 0);
        if (p[0] != cmd) return (-1, 0);
        return (p[1], U16(p, 2));
    }

    private static string AckText(byte cmd, int res, int detail) =>
        res < 0 ? "等 ACK 超时" : $"{(res < ResultName.Length ? ResultName[res] : "?")}（cmd=0x{cmd:X2} detail={detail}）";

    /// <summary>把"已解析但没用"的帧和串口里在途的字节全部倒掉（重发之前必须做）。</summary>
    private void FlushRx()
    {
        _pending.Clear();
        _acc.Clear();
        try
        {
            int a = _sp?.BytesToRead ?? 0;
            if (a > 0)
            {
                var junk = new byte[Math.Min(a, _rxRaw.Length)];
                _sp.Read(junk, 0, junk.Length);
            }
        }
        catch { /* 拔线/超时都忽略：反正马上要重新发请求 */ }
    }

    // ================================================================ 命令
    public string Ping()
    {
        Write(TPing);
        var p = Wait(TPong, 1500)
            ?? throw new Exception("没有 PONG。检查：① 车是否上电 ② 端口是不是 DAP 虚拟串口 "
                + "③ 波特率是否与 DEBUG_UART_BAUDRATE（115200）一致 ④ 串口是否被别的程序占着");

        ProtoVer = p[0];
        SlotCount = p[1];
        SlotPages = p[2];
        RecSizeFromCar = p[3];
        MaxBytes = (int)U32(p, 4);

        CapsKnown = p.Length >= 12;
        Caps = CapsKnown ? U32(p, 8) : (CapDir | CapLoad | CapSlotSel);   // v1 老固件：没有 0x09

        string caps = CapsKnown
            ? (((Caps & CapDir) != 0 ? "DIR " : "") + ((Caps & CapLoad) != 0 ? "LOAD " : "")
             + ((Caps & CapSlotSel) != 0 ? "SEL " : "") + ((Caps & CapSlotRead) != 0 ? "READ " : "")).Trim()
            : "推断(无能力位)";

        return $"PONG：协议 v{ProtoVer}，{SlotCount} 槽 × {SlotPages} 页，记录 {RecSizeFromCar} B，"
             + $"单槽上限 {MaxBytes} B，能力 [{caps}]";
    }

    public sealed class SlotInfo
    {
        public int Slot;
        public bool Valid, Active;
        public int Count, LapMm, DsMm;
        public uint CrcR;
        public string Text => Valid
            ? $"{Slot,2}   {Count,5} 点   {LapMm / 1000.0,8:F2} m   {DsMm,3} mm   0x{CrcR:X4}{(Active ? "   ← 当前" : "")}"
            : $"{Slot,2}   （空）{(Active ? "                      ← 当前" : "")}";
    }

    public List<SlotInfo> Dir(out int active, out int slotCount)
    {
        Write(TDirReq);
        var p = Wait(TDirData, 2000) ?? throw new Exception("没有 DIR_DATA");
        active = p[0];
        slotCount = p[1];
        var list = new List<SlotInfo>(slotCount);
        for (int k = 0; k < slotCount; k++)
        {
            int o = 2 + k * 20;
            list.Add(new SlotInfo
            {
                Valid = p[o] != 0,
                Slot = p[o + 1],
                Count = (int)U32(p, o + 4),
                LapMm = (int)U32(p, o + 8),
                DsMm = (int)U32(p, o + 12),
                CrcR = U32(p, o + 16),
                Active = p[o + 1] == active,
            });
        }
        return list;
    }

    public void SlotSelect(int slot, bool persist, bool load)
    {
        Write(TSlotSel, new byte[] { (byte)slot, (byte)(persist ? 1 : 0), (byte)(load ? 1 : 0) });
        var (res, det) = WaitAck(TSlotSel, 4000);
        if (res != 0) throw new Exception("选槽被拒：" + AckText(TSlotSel, res, det));
    }

    public void SlotLoad(int slot)
    {
        Write(TSlotLoad, new byte[] { (byte)slot });
        var (res, det) = WaitAck(TSlotLoad, 4000);
        if (res != 0) throw new Exception("载入被拒：" + AckText(TSlotLoad, res, det));
    }

    /// <summary>
    /// 下发：PC → 车。<b>分块 + 每块独立重发</b>，落 Flash 之后车端才回 LOAD_DONE。
    /// </summary>
    public string Download(int slot, byte[] image, Action<double, string> progress)
    {
        if (slot <= 0 || slot >= SlotCount)
            throw new Exception($"槽 0 是手推录制的家，下发只允许 1..{SlotCount - 1}");
        if (image == null || image.Length < 48) throw new Exception("镜像太小，不像一张表");
        if (image.Length > MaxBytes) throw new Exception($"镜像 {image.Length} B 超过单槽上限 {MaxBytes} B");

        string bad = VerifyImage(image);
        if (bad != null) throw new Exception("镜像不自洽，没发：" + bad);

        var begin = new byte[5];
        begin[0] = (byte)slot;
        PutU32(begin, 1, (uint)image.Length);
        Write(TLoadBegin, begin);
        var (res, det) = WaitAck(TLoadBegin, 3000);
        if (res != 0) throw new Exception("LOAD_BEGIN 被拒：" + AckText(TLoadBegin, res, det));
        progress?.Invoke(0.02, "已握手");

        int off = 0;
        while (off < image.Length)
        {
            int n = Math.Min(ChunkMax, image.Length - off);
            var pl = new byte[6 + n];
            PutU32(pl, 0, (uint)off);
            pl[4] = (byte)(n & 0xFF); pl[5] = (byte)((n >> 8) & 0xFF);
            Buffer.BlockCopy(image, off, pl, 6, n);

            bool ok = false;
            for (int attempt = 0; attempt < 5 && !ok; attempt++)
            {
                if (attempt > 0) { FlushRx(); Thread.Sleep(10); }
                Write(TLoadData, pl);
                var (r2, d2) = WaitAck(TLoadData, 1500);
                if (r2 == 0) ok = true;
                else if (r2 == 3 || r2 == -1) continue;              // offset 不连续 / 超时 ⇒ 重发同一块
                else throw new Exception("LOAD_DATA 被拒：" + AckText(TLoadData, r2, d2));
            }
            if (!ok) throw new Exception($"offset={off} 重发 5 次仍失败（串口在丢字节 ⇒ 换根线 / 换个 USB 口）");

            off += n;
            progress?.Invoke(0.05 + 0.80 * off / image.Length, $"{off}/{image.Length} B");
        }

        Write(TLoadCommit);
        var (r3, d3) = WaitAck(TLoadCommit, 5000);
        if (r3 != 0) throw new Exception("LOAD_COMMIT 被拒：" + AckText(TLoadCommit, r3, d3));
        progress?.Invoke(0.90, "车端正在写 Flash…");

        var done = Wait(TLoadDone, 25000);
        if (done == null)
            throw new Exception("COMMIT 已 ACK，但没等到 LOAD_DONE（Flash 写超时 / 页写失败）。"
                + "★ 9 成是车端不在【参数页】：分步写盘只在 Param 页的循环里推进"
                + "（route_link_tick 的唯一调用点）。把车端切到主菜单 → Param，再灌一次。");

        progress?.Invoke(1.0, "完成");
        int gotSlot = done[0];
        uint cnt = U32(done, 1), lap = U32(done, 5);
        return $"落盘完成：槽 {gotSlot}，{cnt} 点，{lap / 1000.0:F2} m（已设为当前表）";
    }

    public void Abort()
    {
        Write(TLoadAbort);
        WaitAck(TLoadAbort, 1000);
    }

    /// <summary>
    /// 上传：车 → PC。分块拉整张镜像（车端无状态，流控全在这边）。
    /// 每块独立重试 —— 车端无状态 ⇒ 重问同一个 offset 永远安全。
    /// </summary>
    public byte[] ReadSlot(int slot, Action<double, string> progress)
    {
        if (!CanSlotRead)
            throw new Exception("车端不支持「读槽」(0x09)："
                + (CapsKnown ? $"PONG 的能力位里没有这一项（caps=0x{Caps:X}）。" : "车端回的是 v1 PONG ⇒ 旧固件。")
                + " " + StaleFirmwareHint);

        var acc = new List<byte>(4096);
        int off = 0, expected = -1, retries = 0;

        while (true)
        {
            var pl = new byte[7];
            pl[0] = (byte)slot;
            PutU32(pl, 1, (uint)off);
            pl[5] = (byte)(ChunkMax & 0xFF); pl[6] = (byte)((ChunkMax >> 8) & 0xFF);

            byte[] data = null;
            string lastWhy = "?";

            for (int attempt = 0; attempt < 6; attempt++)
            {
                if (attempt > 0) { FlushRx(); retries++; Thread.Sleep(10); }

                Write(TSlotRead, pl);
                var d = Wait(TSlotData, 1500);

                if (d == null) { lastWhy = "超时"; continue; }
                if (d.Length < 7) { lastWhy = "返回太短"; continue; }

                int gotOff0 = (int)U32(d, 1);
                int gotN0 = U16(d, 5);
                if (gotOff0 != off) { lastWhy = $"车端返回 offset={gotOff0}（请求 {off}）"; continue; }
                if (gotN0 == 0 || gotN0 > d.Length - 7) { lastWhy = "返回长度异常"; continue; }

                data = d;
                break;
            }

            if (data == null)
                throw new Exception($"offset={off} 连续 6 次没读回来（最近一次：{lastWhy}）。"
                    + "串口在长传输里丢字节 ⇒ 换根线 / 换个 USB 口。");

            int n = U16(data, 5);
            acc.AddRange(new ArraySegment<byte>(data, 7, n));
            off += n;

            if (expected < 0 && acc.Count >= 12)
            {
                uint cnt = U32(acc.ToArray(), 8);
                if (cnt > 0 && cnt <= SlotCapacity) expected = 36 + (int)cnt * RecSize;
            }
            double frac = expected > 0 ? Math.Min(1.0, (double)off / expected) : 0.0;
            progress?.Invoke(frac, $"{off}{(expected > 0 ? "/" + expected : "")} B"
                                   + (retries > 0 ? $"   重试 {retries}" : ""));

            if (expected > 0 && off >= expected) break;
            if (off > SlotMaxBytes) throw new Exception("读回来的数据超过单槽上限，中止");
        }

        progress?.Invoke(1.0, "完成");
        return acc.ToArray();
    }

    // ================================================================ 镜像校验（纯函数，可离线自检）
    /// <summary>
    /// 校验一张车端镜像，返回 <c>null</c> = 通过，否则返回原因。
    /// <para>★ 判定与车端 <c>ins_route_load_image()</c> **逐条对应** —— 两边对"什么算一张合法的表"
    /// 必须给出同一个答案，否则会出现"灌进去说成功、车端却说镜像不过"这种自相矛盾。</para>
    /// </summary>
    public static string VerifyImage(byte[] img)
    {
        if (img == null || img.Length < 36) return "太短";

        if (U32(img, 0) != ImgMagic) return "magic 不是 'CTRK'（这不是车端路表镜像）";
        if (U16(img, 4) != ImgVer) return $"版本不是 {ImgVer}（v1 的 12 B 记录已被弃用）";
        if (U16(img, 6) != RecSize) return $"记录长度不是 {RecSize}";
        if (U16(img, 30) != 0 && U16(img, 30) != 1) return "表头 flags 只允许 0/1";

        uint cnt = U32(img, 8), ds = U32(img, 12);
        if (cnt == 0) return "点数为 0";
        if (cnt > SlotCapacity) return $"点数 {cnt} 超过单槽 {SlotCapacity}";
        if (ds == 0) return "点距为 0";

        if (Crc16(0xFFFF, img, 0, 34) != U16(img, 34)) return "表头 CRC 不过";

        int need = 36 + (int)cnt * RecSize;
        if (img.Length < need) return $"记录区不足（需要 {need} B，只有 {img.Length} B）";
        if (Crc16(0xFFFF, img, 36, (int)cnt * RecSize) != U16(img, 32)) return "记录区 CRC 不过";

        return null;
    }

    /// <summary>从镜像里读出点数/点距/一圈长度（显示用）。</summary>
    public static (int Count, int DsMm, int LapMm) ImageInfo(byte[] img)
        => (img.Length >= 36 ? (int)U32(img, 8) : 0,
            img.Length >= 36 ? (int)U32(img, 12) : 0,
            img.Length >= 36 ? (int)U32(img, 16) : 0);
}
