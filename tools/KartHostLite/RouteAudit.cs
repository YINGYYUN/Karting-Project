using System.Text;
using System.Linq;

namespace KartHost;

/// <summary>
/// 路表体检：把一张车端路表（.bin 镜像）当成几何对象审一遍，**给出实测值 + 建议值**。
///
/// <para><b>不吃任何 API / 不联网</b>：全部是纯算术（κ、半径、点距、航向差、闭合误差、侧向加速度），
/// 628 点不到 1 ms。这里只给数字结论；要"为什么/怎么改"，把 <see cref="Result.Report"/> 导出后看即可。</para>
///
/// <para><b>为什么这些阈值可信</b>：曲率 κ 用**和车端 <c>route_pb.c</c> 完全相同的窗口**
/// （±<see cref="KWin"/> 个点 = <c>PB_KWIN</c>，弧长 = 逐段弦长累加，转角 = 首尾 yaw 差），
/// 所以"允许最高车速"这个结论就是车上 <c>pb_speed_limit()</c> 会算出来的那个数，不是另算一套。</para>
///
/// <para>⚠ 读回一张表**必查两项**：完整性（CRC，在 <see cref="RouteLink.VerifyImage"/>）与本类体检。
/// 只查 CRC 会放过"CRC 正确但几何不能跑"的表（例如半径小于最小转弯半径、或航向口径错了 90°）。</para>
/// </summary>
public static class RouteAudit
{
    // ---- "建议值"全部写成常量，并标明出处（改车端参数时这里要跟着改）----
    public const double L_M          = 0.66;   // = route_pb.h  PB_L_DEFAULT（有效轴距）
    public const double DeltaMaxDeg  = 27.0;   // = PB_DELTA_MAX（回放自己再夹一次舵角）
    public const double ALatMaxMps2  = 2.00;   // = PB_ALAT_DEFAULT（允许横向加速度）
    public const int    KWin         = 6;      // = PB_KWIN（曲率窗口：单边点数）
    public const double CloseErrM    = 0.30;   // 闭合误差建议上限
    public const double MaxDYawDeg   = 20.0;   // 单点航向变化建议上限（超过基本是跳点/丢点）
    public const double DsSigmaPct   = 0.10;   // 点距标准差建议上限（相对 ds）
    public const double YawGeomDeg   = 30.0;   // 表内 yaw 与几何重建的平均偏差建议上限
    public const double RevMps       = -0.10;  // = PB_REV_EPS（倒车判定）

    /// <summary>车能执行的最小转弯半径 m = L / tan(δmax)。δmax 取 27°，得 ≈ 1.30 m。</summary>
    public static double SuggestRadiusM => L_M / Math.Tan(DeltaMaxDeg * Math.PI / 180.0);

    public sealed class Row
    {
        public string Lv = "✓";        // ✓ / ⚠ / ✗
        public string Name = "";
        public string Found = "";
        public string Suggest = "";
    }

    public sealed class Result
    {
        public bool Ok = true;
        public int Warn, Bad;
        public string Summary = "";
        public string Report = "";
        public readonly List<Row> Rows = new();

        // 数值也留出来，方便 SelfTest / 以后画图
        public int N;
        public double MeanDsM, SdDsM;
        public double MinRadiusM = double.PositiveInfinity;
        public int MinRadiusIdx = -1;
        public double MaxALat, MaxALatSpeed;
        public int MaxALatIdx = -1;
        public double VmaxAllowed = double.PositiveInfinity;
        public double CloseErrMValue;
        public double MaxDYaw, YawGeomDiff;
        public int RevSegs;
        public bool HasSteer;
    }

    // ================================================================ 主入口
    public static Result Run(byte[] img, string title = "路表")
    {
        var r = new Result();

        if (img == null || img.Length < RouteImage.HeadBytes + RouteImage.RecBytes)
        {
            Rows_Add(r, "✗", "镜像太小", $"{img?.Length ?? 0} B", "至少 36 + 16 B");
            return Finish(r, title, 0, 0, 0, (0, 0, 0));
        }

        int count = BitConverter.ToInt32(img, 8);
        int dsMm = BitConverter.ToInt32(img, 12);
        int lapMm = BitConverter.ToInt32(img, 16);
        int oxMm = BitConverter.ToInt32(img, 20);
        int oyMm = BitConverter.ToInt32(img, 24);
        short oyaw = BitConverter.ToInt16(img, 28);

        int n = (img.Length - RouteImage.HeadBytes) / RouteImage.RecBytes;
        if (count > 0 && count < n) n = count;          // 头部计数更小 ⇒ 以它为准（尾部可能是脏数据）
        r.N = n;

        if (n < 3)
        {
            Rows_Add(r, "✗", "点数", n.ToString(), "至少 3 点才能算曲率");
            return Finish(r, title, n, dsMm, lapMm, (0, 0, 0));
        }

        // ---- 取点（mm -> m）----
        var x = new double[n];
        var y = new double[n];
        var yaw = new double[n];
        var v = new double[n];
        var steer = new double[n];

        for (int i = 0; i < n; i++)
        {
            int o = RouteImage.HeadBytes + i * RouteImage.RecBytes;
            x[i] = BitConverter.ToInt32(img, o + 0) / 1000.0;
            y[i] = BitConverter.ToInt32(img, o + 4) / 1000.0;
            yaw[i] = BitConverter.ToInt16(img, o + 8) / 100.0;
            v[i] = BitConverter.ToInt16(img, o + 10) / 100.0;
            steer[i] = BitConverter.ToInt16(img, o + 12) / 100.0;
        }

        // ---- 逐段弦长 ----
        var seg = new double[n - 1];
        double sumSeg = 0;
        for (int i = 0; i < n - 1; i++)
        {
            double dx = x[i + 1] - x[i];
            double dy = y[i + 1] - y[i];
            seg[i] = Math.Sqrt(dx * dx + dy * dy);
            sumSeg += seg[i];
        }

        double mean = sumSeg / seg.Length;
        double var2 = 0;
        for (int i = 0; i < seg.Length; i++) { double d = seg[i] - mean; var2 += d * d; }
        double sd = Math.Sqrt(var2 / Math.Max(1, seg.Length - 1));

        r.MeanDsM = mean;
        r.SdDsM = sd;

        double sdPct = mean > 1e-6 ? sd / mean : 0;
        double sdLimitMm = dsMm > 0 ? dsMm * DsSigmaPct : mean * 1000.0 * DsSigmaPct;
        string sdText = $"点数 {n}，点距 {mean * 1000.0:0.0} mm（σ {sd * 1000.0:0.00} mm = {sdPct * 100.0:0.0}%，"
                      + $"min {seg.Min() * 1000.0:0.0} / max {seg.Max() * 1000.0:0.0} mm）";
        Rows_Add(r, sdPct <= DsSigmaPct ? "✓" : "⚠", "采样点距", sdText,
                 $"σ ≤ {sdLimitMm:0.0} mm（= {DsSigmaPct * 100:0}% × ds）");

        // ---- 曲率 / 半径（与车端 pb_speed_limit 同窗口）----
        double kappaMax = 0;
        for (int i = 0; i < n; i++)
        {
            int i0 = Math.Max(0, i - KWin);
            int i1 = Math.Min(n - 1, i + KWin);
            if (i1 - i0 < 2) continue;

            double arc = 0;
            for (int k = i0; k < i1; k++) arc += seg[k];

            double dyaw = Math.Abs(Wrap180(yaw[i1] - yaw[i0])) * Math.PI / 180.0;
            double kap = arc > 0.02 ? dyaw / arc : (dyaw > 0.017 ? 1.0 / Math.Max(arc, 0.02) : 0.0);

            if (kap > kappaMax)
            {
                kappaMax = kap;
                r.MinRadiusM = 1.0 / kap;
                r.MinRadiusIdx = i;
            }
        }

        // ---- 侧向加速度（按表内速度，仅正车点）----
        for (int i = 0; i < n; i++)
        {
            if (v[i] <= 0.10) continue;
            double kap = LocalKappa(seg, yaw, n, i);
            double a = v[i] * v[i] * kap;
            if (a > r.MaxALat) { r.MaxALat = a; r.MaxALatIdx = i; r.MaxALatSpeed = v[i]; }
        }

        double suggestR = SuggestRadiusM;
        if (double.IsPositiveInfinity(r.MinRadiusM))
        {
            Rows_Add(r, "✓", "最小半径", "∞（全程直道，没有曲率）", $"≥ {suggestR:0.00} m（= L/tan(27°) = {L_M:0.00}/{Math.Tan(DeltaMaxDeg * Math.PI / 180.0):0.000}）");
        }
        else
        {
            string where = $"第 {r.MinRadiusIdx} 点 @ ({x[r.MinRadiusIdx]:0.00}, {y[r.MinRadiusIdx]:0.00}) m";
            string lv = r.MinRadiusM >= suggestR ? "✓" : "✗";
            Rows_Add(r, lv, "最小半径", $"{r.MinRadiusM:0.00} m（{where}）",
                     $"≥ {suggestR:0.00} m（= L/tan(27°) = {L_M:0.00}/{Math.Tan(DeltaMaxDeg * Math.PI / 180.0):0.000}）");
        }

        if (kappaMax > 1e-6)
        {
            r.VmaxAllowed = Math.Sqrt(ALatMaxMps2 / kappaMax);
            string lv = r.VmaxAllowed >= 0.50 ? "✓" : "⚠";
            Rows_Add(r, lv, "允许最高车速", $"{r.VmaxAllowed:0.00} m/s（alat {ALatMaxMps2:0.0} 束住最紧那个弯）",
                     $"巡航 pb_v ≤ {r.VmaxAllowed:0.00} m/s；或 pb_ve=1 让弯道自动降速");
        }

        if (r.MaxALatIdx >= 0)
        {
            string lv = r.MaxALat <= ALatMaxMps2 ? "✓" : "⚠";
            Rows_Add(r, lv, "侧向加速度", $"{r.MaxALat:0.00} m/s²（第 {r.MaxALatIdx} 点，表内速度 {r.MaxALatSpeed:0.00} m/s）",
                     $"≤ {ALatMaxMps2:0.00} m/s²（alat 默认）");
        }
        else
        {
            Rows_Add(r, "✓", "侧向加速度", "无从判断（表内速度全 ≤ 0.10 m/s，速度由 pb_v 给）",
                     $"≤ {ALatMaxMps2:0.00} m/s²（alat 默认）");
        }

        // ---- 闭合误差 ----
        {
            double dx = x[n - 1] - x[0];
            double dy = y[n - 1] - y[0];
            double ce = Math.Sqrt(dx * dx + dy * dy);
            r.CloseErrMValue = ce;
            Rows_Add(r, ce <= CloseErrM ? "✓" : "⚠", "闭合误差", $"{ce:0.000} m", $"≤ {CloseErrM:0.00} m（环线首尾应自己回来）");
        }

        // ---- 单点航向变化 / 航向口径 ----
        {
            double maxd = 0; int maxi = -1;
            for (int i = 1; i < n; i++)
            {
                double d = Math.Abs(Wrap180(yaw[i] - yaw[i - 1]));
                if (d > maxd) { maxd = d; maxi = i; }
            }
            r.MaxDYaw = maxd;
            Rows_Add(r, maxd <= MaxDYawDeg ? "✓" : "✗", "单点航向变化",
                     $"{maxd:0.0}°" + (maxi > 0 ? $"（第 {maxi} 点）" : ""),
                     $"≤ {MaxDYawDeg:0}°（超过通常是跳点/丢点）");

            // 表内 yaw vs 由相邻点几何重建的航向：差值大 ⇒ 表内 yaw 口径不是"车头朝向"
            double sum = 0; int cnt = 0;
            for (int i = 0; i < n - 1; i++)
            {
                if (seg[i] < 1e-3) continue;
                double g = Math.Atan2(-(x[i + 1] - x[i]), (y[i + 1] - y[i])) * 180.0 / Math.PI;

                // ★ 倒车点的 yaw 是"车头朝向"，与行进方向天然差 180° —— 不能算成口径错。
                //   （2026-10-24 用一张含倒车段的实测表发现的：不修正会把 180° 的差摊成 82° 假警报）
                if (v[i] < RevMps) g = Wrap180(g + 180.0);

                sum += Math.Abs(Wrap180(yaw[i] - g));
                cnt++;
            }
            if (cnt > 0)
            {
                double diff = sum / cnt;
                r.YawGeomDiff = diff;
                Rows_Add(r, diff <= YawGeomDeg ? "✓" : "⚠", "航向口径",
                         $"表内 yaw 与「相邻点方向」平均差 {diff:0.0}°",
                         $"≤ {YawGeomDeg:0}°（≈90° 说明 yaw 不是车头朝向，回放的 target_yaw 会整体偏）");
            }
        }

        // ---- 倒车段 ----
        {
            var ranges = new List<string>();
            int i = 0;
            while (i < n)
            {
                if (v[i] >= RevMps) { i++; continue; }
                int j = i;
                while (j + 1 < n && v[j + 1] < RevMps) j++;
                ranges.Add(j > i ? $"{i}-{j}" : i.ToString());
                i = j + 1;
            }
            r.RevSegs = ranges.Count;
            Rows_Add(r, ranges.Count == 0 ? "✓" : "⚠", "倒车段",
                     ranges.Count == 0 ? "无（全表正车）" : $"{ranges.Count} 段（第 {string.Join("、", ranges)} 点）",
                     "正车回放应为「无」；有倒车段则回放会关航向外环、照抄录制打角");
        }

        // ---- 打角字段 ----
        {
            double smin = steer.Min(), smax = steer.Max();
            r.HasSteer = (smax - smin) > 0.5;
            Rows_Add(r, r.HasSteer ? "✓" : "⚠", "打角字段",
                     r.HasSteer ? $"{smin:0.0}° … {smax:0.0}°（变化 {(smax - smin):0.0}°，逐点有效）"
                                : "全表基本相同（≈常量）",
                     "回放若要逐点打角，steer 字段必须有效（flags bit0）");
        }

        return Finish(r, title, n, dsMm, lapMm, (oxMm, oyMm, oyaw));
    }

    // ================================================================ 组装
    private static void Rows_Add(Result r, string lv, string name, string found, string suggest)
    {
        r.Rows.Add(new Row { Lv = lv, Name = name, Found = found, Suggest = suggest });
        if (lv == "⚠") r.Warn++;
        else if (lv == "✗") { r.Bad++; r.Ok = false; }
    }

    private static Result Finish(Result r, string title, int n, int dsMm, int lapMm,
                                 (int ox, int oy, short oyaw) o)
    {
        int okc = r.Rows.Count(x => x.Lv == "✓");
        r.Summary = r.Ok
            ? (r.Warn == 0 ? $"✓ 体检通过（{okc} 项全绿）" : $"✓ 体检通过（{okc} 绿 / {r.Warn} 警告）")
            : $"✗ 体检**不过**（{r.Bad} 严重 / {r.Warn} 警告）";

        var sb = new StringBuilder();
        sb.AppendLine($"路表体检报告 · {title}");
        sb.AppendLine($"生成时间 {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
        sb.AppendLine($"点数 {n}    点距 {(dsMm > 0 ? dsMm + " mm" : "—")}    圈长 {lapMm / 1000.0:0.000} m");
        if (o.ox != 0 || o.oy != 0 || o.oyaw != 0)
        {
            sb.AppendLine($"起点 ({o.ox / 1000.0:0.000}, {o.oy / 1000.0:0.000}) m    起点航向 {o.oyaw / 100.0:0.00}°");
        }
        sb.AppendLine("------------------------------------------------------------------------------");
        foreach (var row in r.Rows)
        {
            // 一项两行：第一行"实测"、第二行"建议"，缩进对齐。
            // 不排版成三列表格是因为中文是双宽字符，按字符数补空格永远对不齐。
            sb.AppendLine($"{row.Lv} {row.Name}：{row.Found}");
            sb.AppendLine($"     建议 {row.Suggest}");
        }
        sb.AppendLine("------------------------------------------------------------------------------");
        sb.AppendLine(r.Summary);
        sb.AppendLine();
        sb.AppendLine("阈值出处：L=0.66 / δmax=27° / alat=2.00 / KWin=6 —— 全部来自 route_pb.h 的默认参数；");
        sb.AppendLine("曲率用与车端 pb_speed_limit() 完全相同的窗口与公式，所以「允许最高车速」就是车上会算出的值。");
        r.Report = sb.ToString();
        return r;
    }

    // ================================================================ 工具
    private static double LocalKappa(double[] seg, double[] yaw, int n, int i)
    {
        int i0 = Math.Max(0, i - KWin);
        int i1 = Math.Min(n - 1, i + KWin);
        if (i1 - i0 < 2) return 0;

        double arc = 0;
        for (int k = i0; k < i1; k++) arc += seg[k];

        double dyaw = Math.Abs(Wrap180(yaw[i1] - yaw[i0])) * Math.PI / 180.0;
        return arc > 0.02 ? dyaw / arc : 0;
    }

    private static double Wrap180(double a)
    {
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }
}
