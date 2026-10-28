using System.Text;

namespace KartHost;

/// <summary>
/// 无硬件自检 —— 只验证「协议层对不对」，不碰界面、不碰网络。
///
/// <para>用法：<c>KartHostLite.exe --selftest [out.txt]</c>，退出码 = 失败项数。</para>
///
/// <para>为什么值得写：车端 <c>Host_Ctrl.c</c> 的帧编码我无法在这里跑（要 IAR + 板子），
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

        Section("倒车段回放（RouteSim：旧 vs 新）");
        {
            var revTab = MakeFwdRevTable(0.1, 15);
            var sim = RouteSim.Run(revTab, 0.8, 0.8);
            Check(sim.OldFullLock > 5, "旧逻辑在换向点附近会指令满舵", sim.OldFullLock.ToString());
            Check(sim.NewFullLock == 0, "新逻辑（认倒车段）全程不满舵", sim.NewFullLock.ToString());
            Check(sim.Rows.Count(z => z.Rev) == 15, "倒车点识别 15 个", sim.Rows.Count(z => z.Rev).ToString());
            Check(sim.TailRev, "表尾状态 = 倒车段（末点是 v=0 收尾点，靠最后 3 点多数判出来）");

            // 零回归：没有倒车段的表，新旧逻辑必须完全一致
            var fw = RouteImage.BuildLine(10.0, 0.05, out _);
            var s2 = RouteSim.Run(fw, 0.8, 0.8);
            Check(s2.OldFullLock == s2.NewFullLock && Math.Abs(s2.OldMaxErr - s2.NewMaxErr) < 1e-9,
                  "没有倒车段的表：新旧指令完全一致", $"{s2.OldFullLock}/{s2.NewFullLock}");
            Check(!s2.TailRev, "表尾状态 = 前进段（不该误判成倒车）");
        }
        Section("路表体检（RouteAudit）");
        {
            var ringImg = RouteImage.BuildRing(5.0, 0.05, out _);
            var a1 = RouteAudit.Run(ringImg, "圆环 R=5");
            Check(Math.Abs(a1.MinRadiusM - 5.0) < 0.35, "圆环 R=5 m ⇒ 最小半径算得 ≈5.0 m",
                  a1.MinRadiusM.ToString("0.000"));
            // 生成器不重复首点 ⇒ 末点离起点正好差一个点距，闭合误差 ≈ 1×ds 是正常的
            Check(a1.CloseErrMValue <= a1.MeanDsM * 1.2, "圆环闭合误差 ≤ 1 个点距",
                  $"{a1.CloseErrMValue:0.000} m（ds {a1.MeanDsM:0.000} m）");
            Check(a1.MaxDYaw < 2.0, "圆环单点航向变化 ≈ 360/628 = 0.57°", a1.MaxDYaw.ToString("0.00"));
            Check(a1.YawGeomDiff < 3.0, "圆环：表内 yaw 与几何方向一致（口径正确）", a1.YawGeomDiff.ToString("0.0"));
            Check(a1.Bad == 0, "圆环：无严重项（打角字段会是警告，因为生成器不写 steer）", a1.Summary);

            var lineImg = RouteImage.BuildLine(10.0, 0.05, out _);
            var a2 = RouteAudit.Run(lineImg, "直线");
            Check(double.IsPositiveInfinity(a2.MinRadiusM), "直线 ⇒ 无曲率（半径 = ∞）", a2.MinRadiusM.ToString());
            Check(a2.CloseErrMValue > 9.0, "直线首尾相距 ≈10 m（不是环）", a2.CloseErrMValue.ToString("0.00"));

            // 人为把一个点的 yaw 打偏 45°：必须被"单点航向变化"抓到
            var jump = (byte[])ringImg.Clone();
            int o = RouteImage.HeadBytes + 100 * RouteImage.RecBytes + 8;
            short yy = BitConverter.ToInt16(jump, o);
            BitConverter.GetBytes((short)(yy + 4500)).CopyTo(jump, o);
            var a3 = RouteAudit.Run(jump, "跳点");
            Check(a3.MaxDYaw > 20.0, "1 个点偏 45° ⇒ 检出航向突变", a3.MaxDYaw.ToString("0.0"));

            // 半径小于最小转弯半径：R=1.0 m 的环应判"不过"
            var tightImg = RouteImage.BuildRing(1.0, 0.05, out _);
            var a4 = RouteAudit.Run(tightImg, "小环 R=1");
            Check(!a4.Ok, "R=1.0 m < L/tan27°≈1.30 m ⇒ 体检不过", a4.Summary);
            Check(a4.MinRadiusM < RouteAudit.SuggestRadiusM, "  ↳ 报告的实测值确实小于建议值",
                  $"{a4.MinRadiusM:0.000} < {RouteAudit.SuggestRadiusM:0.000}");
        }

        Section("台架直给 · SET_TARGET 帧");
        {
            var f1 = Proto.BuildSetTarget(TargetId.Steer, -12.5f, 0x1234);
            Check(f1.Length == 15, "帧长 15 B", f1.Length.ToString());
            Check(f1[5] == (byte)Cmd.SetTarget && f1[5] == 0x09, "cmd = 0x09（0x08 留给读路表）",
                  $"0x{f1[5]:X2}");
            Check(f1[6] == (byte)TargetId.Steer, "id = 0（舵机）", f1[6].ToString());
            Check(Math.Abs(Proto.GetF32(f1, 7) + 12.5f) < 1e-6f, "负角度按 float 原样编码（不能用 uint16 截）",
                  Proto.GetF32(f1, 7).ToString("0.###"));

            var f2 = Proto.BuildSetTarget(TargetId.Wheel, 3.2f, 1);
            Check(f2[6] == (byte)TargetId.Wheel && Math.Abs(Proto.GetF32(f2, 7) - 3.2f) < 1e-6f,
                  "id = 1（两轮 rps），3.2 rps 原样编码");

            // CRC 覆盖 TYPE~PAYLOAD（不含 MAGIC、不含 CRC）
            ushort want = Proto.Crc16(f2, 2, 3 + Proto.DN_PAYLOAD);
            Check(Proto.GetU16(f2, 13) == want, "CRC 覆盖 [2, 13)", $"0x{Proto.GetU16(f2, 13):X4} / 0x{want:X4}");
        }

        Section("参数元数据（73 B 载荷）");
        var meta = BuildParamMeta(8, 0f, 1000f, 500f, 1f, "-", "kp_s", "舵机|比例 Kp");
        Check(meta.Length == 73, "载荷 73 B", meta.Length.ToString());
        Check(Proto.GetStr(meta, 25, 16) == "kp_s", "英文名回读", Proto.GetStr(meta, 25, 16));
        string gl = Proto.GetStr(meta, 41, 32);
        Check(gl == "舵机|比例 Kp" && gl.Contains('|'), "组名|中文标签回读", gl);

        Section("通道表 · 两套档案");
        Check(Defaults.Full41.Count == 41, "全量档案 = 41 路（与车端 HOST_CH_COUNT_MAX 一致）",
              Defaults.Full41.Count.ToString());
        Check(Defaults.Calib18.Count == 18, "标定档案 = 18 路（与车端 HOST_CT_COUNT 一致）",
              Defaults.Calib18.Count.ToString());

        // 两套各 4 张平行表都要长度一致 —— 少一张就整体错位，而 WinForms/GDI 不会报错
        foreach (var prof in new[] { Defaults.Full41, Defaults.Calib18 })
        {
            string tag = prof.Short;
            Check(prof.Names.Length == prof.Units.Length, $"[{tag}] 名字与单位数量一致",
                  $"{prof.Names.Length} vs {prof.Units.Length}");
            Check(prof.Names.Length == prof.Cn.Length, $"[{tag}] 名字与中文简称数量一致",
                  $"{prof.Names.Length} vs {prof.Cn.Length}");
            Check(prof.Names.Length == prof.Tips.Length, $"[{tag}] 名字与悬停说明数量一致",
                  $"{prof.Names.Length} vs {prof.Tips.Length}");

            var dup = prof.Names.GroupBy(s => s).Where(g => g.Count() > 1).Select(g => g.Key).ToArray();
            Check(dup.Length == 0, $"[{tag}] 通道名无重复",
                  dup.Length == 0 ? prof.Names.Length.ToString() : string.Join(",", dup));

            // 中文简称超宽会被勾选框裁掉（不报错），这里提前拦一次。
            // ⚠ 不能按 .Length 算：「CH1方向盘」是 7 个字符，但显示宽度只有 3 半角 + 3 汉字 ≈ 9，
            //   按字符数会误报。所以换算成"半角单位"（全角算 2），上限 12 = 6 个汉字。
            var longCn = new List<string>();
            for (int i = 0; i < prof.Cn.Length; i++)
            {
                int w = 0;
                foreach (char c in prof.Cn[i]) w += c < 0x2E80 ? 1 : 2;
                if (w > 12) longCn.Add($"{i}:{prof.Cn[i]}({w})");
            }
            Check(longCn.Count == 0, $"[{tag}] 中文简称显示宽度 ≤ 12 半角单位（≈6 个汉字，否则勾选框会裁字）",
                  longCn.Count == 0 ? "ok" : string.Join(" ", longCn));

            // 下标类配置也不能越界（越界不会抛，只会显示空白/错位）
            bool idxOk = true;
            foreach (int i in prof.DefaultShown) if (i < 0 || i >= prof.Count) idxOk = false;
            foreach (var (_, chs) in prof.Presets2) foreach (int c in chs) if (c < 0 || c >= prof.Count) idxOk = false;
            foreach (var (_, chs) in prof.Presets4) foreach (int c in chs) if (c < 0 || c >= prof.Count) idxOk = false;
            foreach (var (c, _, _) in prof.ValueBoxes) if (c < 0 || c >= prof.Count) idxOk = false;
            foreach (int c in prof.OdomXy) if (c < 0 || c >= prof.Count) idxOk = false;
            Check(idxOk, $"[{tag}] 默认勾选 / 分图预设 / 实时值 / 轨迹图的下标都在范围内");

            Check(prof.DefaultShown.Length > 0, $"[{tag}] 有默认勾选的通道", prof.DefaultShown.Length.ToString());
        }

        // 轨迹图：全量档必须有 ins_x/ins_y，标定档必须没有（没有就显示 N/A）
        Check(Defaults.Full41.HasOdom && Defaults.Full41.OdomXy[0] == 29 && Defaults.Full41.OdomXy[1] == 30,
              "全量档案的 odom 轨迹用 ins_x(29) / ins_y(30)");
        Check(!Defaults.Calib18.HasOdom, "标定档案没有 ins_x/ins_y ⇒ 轨迹图显示 N/A");

        // 关键下标与车端 motor_link.h 逐项核对（顺序即协议，写这几个最容易错位的）
        Check(Defaults.Full41.Names[0] == "target_l" && Defaults.Full41.Names[2] == "u_l",
              "全量档 0/2 = target_l / u_l");
        Check(Defaults.Full41.Names[15] == "safety" && Defaults.Full41.Names[16] == "steer_tgt",
              "全量档 15/16 = safety / steer_tgt（转向段从 16 起）");
        Check(Defaults.Full41.Names[24] == "rc_frames" && Defaults.Full41.Names[25] == "k_v",
              "全量档 24/25 = rc_frames / k_v");
        Check(Defaults.Full41.Names[40] == "ins_static", "全量档末位 40 = ins_static");
        Check(Defaults.Calib18.Names[6] == "steer_tgt" && Defaults.Calib18.Names[17] == "k_z",
              "标定档 6 / 17 = steer_tgt / k_z");
        Check(Defaults.Calib18.Names[14] == "ol_en" && Defaults.Calib18.Names[15] == "resv",
              "标定档 14/15 = ol_en / resv（车端固定发 0）");

        // u_l / u_r / uff 是**百分数**（车端 ×100），单位表里必须写 %，不能再当 ±1 画
        Check(Defaults.Full41.UnitOf(2) == "%" && Defaults.Full41.UnitOf(5) == "%"
              && Defaults.Full41.UnitOf(6) == "%" && Defaults.Full41.UnitOf(7) == "%",
              "u_l / u_r / uff_l / uff_r 的单位是 %（车端已 ×100）");
        Check(Defaults.Calib18.UnitOf(2) == "%" && Defaults.Calib18.UnitOf(5) == "%",
              "标定档 u_l / u_r 的单位是 %");

        Section("档案选择（按帧头 COUNT）");
        Check(ReferenceEquals(Defaults.ByCount(41), Defaults.Full41), "COUNT = 41 → 全量档案");
        Check(ReferenceEquals(Defaults.ByCount(18), Defaults.Calib18), "COUNT = 18 → 标定档案");
        Check(Defaults.ByCount(30) == null && Defaults.ByCount(0) == null && Defaults.ByCount(40) == null,
              "其它 COUNT → 未知档案（返回 null，调用方必须丢帧）");

        // ==================================================================
        // 路表镜像（串口灌表）—— 与车端 ins_route_load_image() 同一套判定
        // 这一段**不需要串口、不需要车**：镜像格式、CRC、点数/点距、帧组包全在这里钉死。
        // 现场出现"灌不进去"时先跑它 —— 能把"镜像不对"和"线/驱动/波特率不对"分开。
        // ==================================================================
        Section("路表镜像（36 B 头 + N×16 B，v2）");

        var ring = RouteImage.BuildRing(5.0, 0.05, out int ringN);
        Check(RouteLink.VerifyImage(ring) == null, "圆环测试表校验通过", RouteLink.VerifyImage(ring) ?? "ok");
        Check(ring.Length == 36 + ringN * 16, "长度 = 36 + N×16",
              $"{ring.Length} / {36 + ringN * 16}（N={ringN}）");
        Check(ringN == (int)Math.Round(2 * Math.PI * 5.0 / 0.05), "点数 = 周长 / 点距", ringN.ToString());

        var (rc, rds, rlap) = RouteLink.ImageInfo(ring);
        Check(RouteLink.U32(ring, 0) == 0x4B525443, "magic = 'CTRK'");
        Check(RouteLink.U16(ring, 4) == 2, "ver = 2（v1 的 12 B 记录已弃用）");
        Check(RouteLink.U16(ring, 6) == 16, "rec_len = 16");
        Check(rc == ringN && rds == 50 && rlap == (int)Math.Round(2 * Math.PI * 5000.0),
              "表头 count / ds_mm / lap_mm 对得上", $"count={rc} ds={rds} lap={rlap}");
        Check(RouteLink.U16(ring, 34) == RouteLink.Crc16(0xFFFF, ring, 0, 34), "表头 CRC 覆盖 [0,34)");
        Check(RouteLink.U16(ring, 32) == RouteLink.Crc16(0xFFFF, ring, 36, ringN * 16),
              "记录区 CRC 覆盖整段记录");

        // 首点必须在原点：这是"坐标没被平移"的字面证据
        Check(RouteLink.U32(ring, 36) == 0 && RouteLink.U32(ring, 40) == 0,
              "第 0 点在原点 (0, 0)", $"{RouteLink.U32(ring, 36)} {RouteLink.U32(ring, 40)}");

        // 破坏任一处都必须被拦下来 —— 对应车端准入条件的四个分支
        var badCrcH = (byte[])ring.Clone(); badCrcH[10] ^= 0xFF;
        Check(RouteLink.VerifyImage(badCrcH) != null, "坏表头 CRC → 拦下");
        var badCrcR = (byte[])ring.Clone(); badCrcR[36 + 5] ^= 0xFF;
        Check(RouteLink.VerifyImage(badCrcR) != null, "坏记录区 CRC → 拦下");
        var badVer = (byte[])ring.Clone(); badVer[4] = 1;
        Check(RouteLink.VerifyImage(badVer) != null, "ver=1（老格式）→ 拦下");
        var badCnt = (byte[])ring.Clone(); badCnt[8] = 0; badCnt[9] = 0; badCnt[10] = 0; badCnt[11] = 0;
        Check(RouteLink.VerifyImage(badCnt) != null, "点数为 0 → 拦下");
        Check(RouteLink.VerifyImage(new byte[20]) != null, "太短的字节流 → 拦下");

        var line = RouteImage.BuildLine(10.0, 0.05, out int lineN);
        Check(RouteLink.VerifyImage(line) == null && lineN == 200,
              "直线测试表：10 m / 50 mm = 200 点，校验通过", lineN.ToString());

        Section("路表串口帧（与车端 route_link.c 对齐）");
        var pingF = RouteLink.Build(RouteLink.TPing);
        Check(pingF.Length == 7 && pingF[0] == 0xA5 && pingF[1] == 0x5A,
              "PING 帧 = 7 B，同步头 A5 5A", pingF.Length.ToString());
        Check(RouteLink.U16(pingF, 5) == RouteLink.Crc16(0xFFFF, pingF, 2, 3),
              "CRC 覆盖 type|len（不含同步头）");

        var begin = new byte[5];
        begin[0] = 1;
        RouteLink.PutU32(begin, 1, (uint)ring.Length);
        var beginF = RouteLink.Build(RouteLink.TLoadBegin, begin);
        Check(beginF.Length == 12 && RouteLink.U16(beginF, 3) == 5,
              "LOAD_BEGIN 帧 = 12 B（payload 5 B）", beginF.Length.ToString());
        Check(RouteLink.U32(beginF, 6) == (uint)ring.Length, "LOAD_BEGIN 里的 total_bytes 对");

        int chunks = (ring.Length + RouteLink.ChunkMax - 1) / RouteLink.ChunkMax;
        Check(RouteLink.ChunkMax <= RouteLink.PayloadMax - 6, "块大小 + 6 B 头 ≤ 单帧载荷上限",
              $"{RouteLink.ChunkMax} + 6 ≤ {RouteLink.PayloadMax}");
        Check(RouteLink.SlotMaxBytes == 36 + 2048 * 16 && ring.Length <= RouteLink.SlotMaxBytes,
              "单槽上限 32804 B，测试镜像装得下", $"{ring.Length} ≤ {RouteLink.SlotMaxBytes}，{chunks} 块");

        Section("CSV 导出（与 CsvRecorder 同一套表头）");
        try
        {
            var st = new ChannelStore();
            int nc = Math.Min(6, Defaults.Current.Count);
            st.Ensure(nc);

            float[] row = new float[nc];
            for (int i = 0; i < 5; i++)
            {
                for (int c = 0; c < nc; c++) row[c] = c * 10 + i;
                st.Append(row, nc, 1000.0 + i * 0.02, 0xFFFF);
            }

            string hp = System.IO.Path.Combine(System.IO.Path.GetTempPath(), $"kh_csv_{Guid.NewGuid():N}.csv");
            int nrow = CsvExport.Dump(hp, st, new[] { 3, 1 }, out string info);
            string[] lines = File.ReadAllLines(hp);
            try { File.Delete(hp); } catch { }

            string Head(int ch)
            {
                string u = Defaults.ChannelUnit(ch);
                return Defaults.ChannelName(ch) + (string.IsNullOrEmpty(u) ? "" : "[" + u + "]");
            }

            Check(nrow == 5 && lines.Length == 6, "导出行数 = 缓冲点数（5 行 + 1 行表头）", $"{nrow} / {lines.Length}");
            Check(lines[0] == $"t_ms,note,{Head(1)},{Head(3)}",
                  "表头与 CsvRecorder 同格式，且按通道号升序（传进来是 3,1）", lines[0]);
            Check(lines[1].Split(',')[0] == "0", "首行 t_ms = 0（减掉第一点）", lines[1]);
            Check(lines[1].Split(',')[1] == "", "note 列恒为空（快照没有锚点）", lines[1]);

            var empty = new ChannelStore();
            empty.Ensure(4);
            string ep = System.IO.Path.Combine(System.IO.Path.GetTempPath(), $"kh_csv_e_{Guid.NewGuid():N}.csv");
            int n0 = CsvExport.Dump(ep, empty, new[] { 0 }, out string info0);
            Check(n0 == 0 && info0.Contains("没有"), "空缓冲返回 0 并给出原因，不抛异常", info0);
            try { if (File.Exists(ep)) File.Delete(ep); } catch { }
        }
        catch (Exception ex) { Check(false, "CSV 导出不抛异常", ex.Message); }

        _sb.Append($"\r\n通过 {_pass} 项，失败 {_fail} 项。\r\n");
        // ★ 本程序是 WinExe，没有控制台 —— 结果必须落到文件里才看得见
        string path = string.IsNullOrEmpty(outPath) ? "selftest.txt" : outPath;
        try { File.WriteAllText(path, _sb.ToString(), Encoding.UTF8); } catch { }
        Console.Write(_sb.ToString());
        return _fail;
    }

    // ---- 构造（与车端 Host_Ctrl.c 的组帧逻辑逐字节对应，用于自测）----
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

    /// <summary>
    /// 造一张"前进 + 原路倒回"的合成表：沿 +y 走 legPts 点，再沿 −y 走回来。
    /// 直接改 BuildLine 出来的镜像（RouteSim 不校验 CRC，只算几何）。
    /// </summary>
    private static byte[] MakeFwdRevTable(double dsM, int legPts)
    {
        int n = legPts * 2;
        var img = RouteImage.BuildLine(legPts * dsM, dsM, out int baseN);
        if (baseN < n) Array.Resize(ref img, RouteImage.HeadBytes + n * RouteImage.RecBytes);

        for (int i = 0; i < n; i++)
        {
            int o = RouteImage.HeadBytes + i * RouteImage.RecBytes;
            bool rev = i >= legPts;
            double yy = (rev ? (n - i) * dsM : i * dsM);

            BitConverter.GetBytes((int)Math.Round(0.0 * 1000.0)).CopyTo(img, o + 0);
            BitConverter.GetBytes((int)Math.Round(yy * 1000.0)).CopyTo(img, o + 4);
            BitConverter.GetBytes((short)0).CopyTo(img, o + 8);                                  // yaw = 0
            BitConverter.GetBytes((short)(rev ? -20 : 20)).CopyTo(img, o + 10);                  // v = ∓0.20 m/s
            BitConverter.GetBytes((short)0).CopyTo(img, o + 12);                                 // steer
            BitConverter.GetBytes((ushort)(RouteSim.FlagSteer | (rev ? RouteSim.FlagRev : 0))).CopyTo(img, o + 14);
        }

        BitConverter.GetBytes(n).CopyTo(img, 8);
        return img;
    }
}
