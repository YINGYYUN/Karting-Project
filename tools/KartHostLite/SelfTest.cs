using System.Text;

namespace KartHost;

/// <summary>
/// 无硬件自检 —— 只验证「协议层对不对」，不碰界面、不碰网络。
///
/// <para>用法：<c>KartHostLite.exe --selftest [out.txt]</c>，退出码 = 失败项数。</para>
///
/// <para>为什么值得写：车端 <c>Host_Crtl.c</c> 的帧编码我无法在这里跑（要 IAR + 板子），
/// 所以 PC 侧至少要把「CRC 算得一样、切帧状态机经得起半包/垃圾字节/坏 CRC」这两件事钉死。
/// 现场出现"收不到帧"时，先跑它 —— 能把"协议不一致"和"线/端口/IP 不对"分开。</para>
/// </summary>
internal static class SelfTest
{
    private static int _pass, _fail;
    private static readonly StringBuilder _sb = new();

    public static int Run(string outPath)
    {
        Section("CRC16-MODBUS");
        // 标准校验向量："123456789" → 0x4B37
        var vec = Encoding.ASCII.GetBytes("123456789");
        Check(Proto.Crc16(vec, 0, vec.Length) == 0x4B37, "CRC16(\"123456789\") = 0x4B37",
              $"0x{Proto.Crc16(vec, 0, vec.Length):X4}");

        Section("下行命令帧");
        var cmd = Proto.BuildSetParam(8, 500f, 0x1234);
        Check(cmd.Length == 15, "长度 = 15", cmd.Length.ToString());
        Check(cmd[0] == 0x5A && cmd[1] == 0xA5, "魔数 = 5A A5", $"{cmd[0]:X2} {cmd[1]:X2}");
        Check(cmd[2] == (byte)FrameType.Command, "类型 = Command(0x01)", cmd[2].ToString("X2"));
        Check(Proto.GetU16(cmd, 3) == 8, "LEN = 8", Proto.GetU16(cmd, 3).ToString());
        Check(cmd[5] == (byte)Cmd.SetParam && cmd[6] == 8, "CMD=SET_PARAM / ID=8");
        Check(Math.Abs(Proto.GetF32(cmd, 7) - 500f) < 1e-3, "VALUE = 500.0f");
        Check(Proto.GetU16(cmd, 11) == 0x1234, "TOKEN = 0x1234");
        Check(Proto.GetU16(cmd, 13) == Proto.Crc16(cmd, 2, 11), "CRC 覆盖 TYPE~PAYLOAD");

        Section("上行遥测帧（构造 → 切帧）");
        var tel = BuildTelemetry(seq: 7, ts: 123456, ch: new[] { 1.5f, -2.25f, 100f });
        Check(tel.Length == Proto.UP_OVERHEAD + Proto.TELEM_OVERHEAD + 3 * 4,
              "帧长 = 11 + 10 + 4N", tel.Length.ToString());

        var p = new FrameParser();
        int got = 0; int gotType = -1; int gotCnt = -1; float gotFirst = 0;
        p.OnFrame = (t, b, o) =>
        {
            got++; gotType = t; gotCnt = Proto.GetU16(b, o + 8); gotFirst = Proto.GetF32(b, o + 10);
        };
        p.Feed(tel, tel.Length);
        Check(got == 1, "整帧一次投喂 → 交付 1 帧", got.ToString());
        Check(gotType == (byte)FrameType.Telemetry, "类型 = Telemetry(0x01)");
        Check(gotCnt == 3, "COUNT = 3", gotCnt.ToString());
        Check(Math.Abs(gotFirst - 1.5f) < 1e-6, "ch0 = 1.5", gotFirst.ToString("0.###"));

        Section("切帧纪律");
        // ① 垃圾前缀 + 半个帧头
        var p1 = new FrameParser();
        int n1 = 0; p1.OnFrame = (_, __, ___) => n1++;
        var dirty = new byte[] { 0x11, 0x22, 0xA5, 0x33, 0x44 };
        p1.Feed(dirty, dirty.Length);
        p1.Feed(tel, tel.Length);
        Check(n1 == 1, "垃圾前缀后仍能同步", n1.ToString());

        // ② 逐字节投喂（最狠的半包测试）
        var p2 = new FrameParser();
        int n2 = 0; p2.OnFrame = (_, __, ___) => n2++;
        var one = new byte[1];
        for (int i = 0; i < tel.Length; i++) { one[0] = tel[i]; p2.Feed(one, 1); }
        Check(n2 == 1, "逐字节投喂 → 只交付 1 帧", n2.ToString());
        Check(p2.PendingBytes == 0, "缓冲无残留", p2.PendingBytes.ToString());

        // ③ 一个缓冲里两帧
        var p3 = new FrameParser();
        int n3 = 0; p3.OnFrame = (_, __, ___) => n3++;
        var two = new byte[tel.Length * 2];
        Buffer.BlockCopy(tel, 0, two, 0, tel.Length);
        Buffer.BlockCopy(tel, 0, two, tel.Length, tel.Length);
        p3.Feed(two, two.Length);
        Check(n3 == 2, "两帧连发 → 交付 2 帧", n3.ToString());

        // ④ CRC 被破坏
        var p4 = new FrameParser();
        int n4 = 0; p4.OnFrame = (_, __, ___) => n4++;
        var bad = (byte[])tel.Clone();
        bad[10] ^= 0xFF;                      // 打在载荷上
        p4.Feed(bad, bad.Length);
        Check(n4 == 0 && p4.FramesBadCrc == 1, "坏 CRC → 不交付且计 1 次",
              $"交付{n4} / badcrc{p4.FramesBadCrc}");

        // ⑤ 帧尾错（JustFloat 标记）
        var p5 = new FrameParser();
        int n5 = 0; p5.OnFrame = (_, __, ___) => n5++;
        var badTail = (byte[])tel.Clone();
        badTail[badTail.Length - 1] = 0x00;
        p5.Feed(badTail, badTail.Length);
        Check(n5 == 0, "帧尾不对 → 不交付", n5.ToString());

        Section("参数元数据（73 B 载荷）");
        var meta = BuildParamMeta(8, 0f, 1000f, 500f, 1f, "-", "kp_s", "舵机|比例 Kp");
        Check(meta.Length == 73, "载荷 73 B", meta.Length.ToString());
        Check(Proto.GetStr(meta, 25, 16) == "kp_s", "英文名回读", Proto.GetStr(meta, 25, 16));
        string gl = Proto.GetStr(meta, 41, 32);
        Check(gl == "舵机|比例 Kp" && gl.Contains('|'), "组名|中文标签回读", gl);

        Section("通道表");
        Check(Defaults.ChannelNames.Length == Defaults.ChannelUnits.Length,
              "通道名与单位数量一致",
              $"{Defaults.ChannelNames.Length} vs {Defaults.ChannelUnits.Length}");
        Check(Defaults.ChannelNames.Length == Defaults.ChannelCn.Length,
              "通道名与中文简称数量一致",
              $"{Defaults.ChannelNames.Length} vs {Defaults.ChannelCn.Length}");
        Check(Defaults.ChannelNames.Length == Defaults.ChannelTips.Length,
              "通道名与悬停说明数量一致",
              $"{Defaults.ChannelNames.Length} vs {Defaults.ChannelTips.Length}");
        // 中文简称超宽会被勾选框裁掉（不报错），这里提前拦一次。
        // ⚠ 不能按 .Length 算：「遥控CH1原始」是 7 个字符，但显示宽度只有 4 汉字 + 3 半角 ≈ 5.5 汉字宽，
        //   按字符数会误报。所以换算成"半角单位"（全角算 2），上限 12 = 6 个汉字。
        var longCn = new List<string>();
        for (int i = 0; i < Defaults.ChannelCn.Length; i++)
        {
            int w = 0;
            foreach (char c in Defaults.ChannelCn[i]) w += c < 0x2E80 ? 1 : 2;
            if (w > 12) longCn.Add($"{i}:{Defaults.ChannelCn[i]}({w})");
        }
        Check(longCn.Count == 0, "中文简称显示宽度 ≤ 12 半角单位（≈6 个汉字，否则勾选框会裁字）",
              longCn.Count == 0 ? "ok" : string.Join(" ", longCn));
        Check(Defaults.ChannelNames.Length == 25, "通道数 = 25（与车端 HOST_CH_COUNT 一致）",
              Defaults.ChannelNames.Length.ToString());
        var uniq = new HashSet<string>(Defaults.ChannelNames);
        Check(uniq.Count == Defaults.ChannelNames.Length, "通道名无重复", uniq.Count.ToString());
        bool idxOk = true;
        foreach (int i in Defaults.DefaultShown) if (i < 0 || i >= Defaults.ChannelNames.Length) idxOk = false;
        Check(idxOk, "默认勾选下标都在范围内");

        _sb.Append($"\r\n通过 {_pass} 项，失败 {_fail} 项。\r\n");

        // ★ 本程序是 WinExe，没有控制台 —— 结果必须落到文件里才看得见
        string path = string.IsNullOrEmpty(outPath) ? "selftest.txt" : outPath;
        try { File.WriteAllText(path, _sb.ToString(), Encoding.UTF8); } catch { }
        Console.Write(_sb.ToString());
        return _fail;
    }

    // ---- 构造（与车端 Host_Crtl.c 的组帧逻辑逐字节对应，用于自测）----
    private static byte[] BuildTelemetry(int seq, uint ts, float[] ch)
    {
        int pl = Proto.TELEM_OVERHEAD + ch.Length * 4;
        var f = new byte[Proto.UP_OVERHEAD + pl];
        f[0] = Proto.MAGIC_UP_0; f[1] = Proto.MAGIC_UP_1;
        f[2] = (byte)FrameType.Telemetry;
        Proto.PutU16(f, 3, (ushort)pl);
        Proto.PutU16(f, 5, (ushort)seq);
        Proto.PutU32(f, 7, ts);
        Proto.PutU16(f, 11, 0xFFFF);          // MASK = 全部有效（哨兵）
        Proto.PutU16(f, 13, (ushort)ch.Length);
        for (int i = 0; i < ch.Length; i++) Proto.PutF32(f, 15 + i * 4, ch[i]);
        Proto.PutU16(f, 5 + pl, Proto.Crc16(f, 2, 3 + pl));
        Array.Copy(Proto.TAIL, 0, f, 7 + pl, 4);
        return f;
    }

    private static byte[] BuildParamMeta(byte id, float lo, float hi, float def, float step,
                                         string unit, string name, string grpLabel)
    {
        var b = new byte[73];
        b[0] = id;
        Proto.PutF32(b, 1, lo);
        Proto.PutF32(b, 5, hi);
        Proto.PutF32(b, 9, def);
        Proto.PutF32(b, 13, step);
        Proto.PutStr(b, 17, 8, unit);
        Proto.PutStr(b, 25, 16, name);
        Proto.PutStr(b, 41, 32, grpLabel);
        return b;
    }

    // ---- 断言 ----
    private static void Section(string s)
    {
        _sb.Append($"\r\n[{s}]\r\n");
    }

    private static void Check(bool ok, string what, string actual = null)
    {
        if (ok) { _pass++; _sb.Append($"  PASS  {what}\r\n"); }
        else
        {
            _fail++;
            _sb.Append($"  FAIL  {what}" + (actual != null ? $"   实际: {actual}" : "") + "\r\n");
        }
    }
}
