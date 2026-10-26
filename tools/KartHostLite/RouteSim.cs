using System.Text;

namespace KartHost;

/// <summary>
/// 路表**逐点指令推演**：假设车精确落在表里的每个点上，算出车端"那一拍会下发的速度与舵角"，
/// 和表里录制的打角对比。用来在上车之前回答：**这张表某个点会不会被指令成满舵/反向。**
///
/// <para>⚠ 这里**不是动力学仿真**：没有轮胎/舵机延迟/打滑，也不积分位姿。
/// 它只验证"回放逻辑对每个点下的指令是否合理"，这正是纯跟踪 + 倒车分支出问题的地方
/// （实测那张表在掉头点数：前视点落在车后 0.799 m ⇒ α≈−178° ⇒ δ 指令 −294° 被夹成满舵）。</para>
///
/// <para>公式与车端逐字对应：<c>route_pb.c</c> 的 <c>pb_find_lookahead</c>、
/// <c>α = atan2(bx, by)</c>、<c>target_yaw = yaw − α</c>、<c>δ = −k·(target−meas)</c>、
/// <c>k = 2L/ld</c>、<c>PB_DELTA_MAX</c> 夹取；倒车分支对应 <c>s_rev</c> 那一段。</para>
/// </summary>
public static class RouteSim
{
    // 与车端默认参数对齐（route_pb.h）
    public const double L_M = 0.66;          // PB_L_DEFAULT
    public const double DeltaMax = 27.0;     // PB_DELTA_MAX
    public const double RevEps = 0.10;       // PB_REV_EPS
    public const double RevCreep = 0.15;     // PB_REV_CREEP
    public const double RevVmax = 0.60;      // PB_REV_VMAX
    public const int TailN = 3;              // PB_TAIL_N
    public const double TailV = 0.06;        // PB_TAIL_V
    public const int TailMs = 1500;          // PB_TAIL_MS

    public const int FlagSteer = 0x0001;
    public const int FlagRev = 0x0004;

    public sealed class Row
    {
        public int I;
        public double X, Y, Yaw, VRec, DRec;
        public bool Rev, SteerValid;

        public double VOld, DOld;     // 旧逻辑（不认倒车段：速度恒为 pb_v，转向恒走纯跟踪）
        public double VNew, DNew;     // 新逻辑（本次改动）
    }

    public sealed class Result
    {
        public readonly List<Row> Rows = new();
        public string Text = "";
        public bool TailRev;                      // 表尾是否为倒车段（最后 3 点多数）
        public int OldFullLock, NewFullLock;      // |δ| ≥ DeltaMax-1 的点数
        public int OldWorse10, NewWorse10;        // 与录制打角差 > 10°（正车段内比较）
        public double OldMaxErr, NewMaxErr;
    }

    public static Result Run(byte[] img, double pbV = 0.80, double ld = 0.80)
    {
        var r = new Result();
        if (img == null || img.Length < RouteImage.HeadBytes + RouteImage.RecBytes)
        {
            r.Text = "镜像太小，没法推演";
            return r;
        }

        int count = BitConverter.ToInt32(img, 8);
        int n = (img.Length - RouteImage.HeadBytes) / RouteImage.RecBytes;
        if (count > 0 && count < n) n = count;
        if (n < 3) { r.Text = "点数太少（< 3）"; return r; }

        var x = new double[n];
        var y = new double[n];
        var yaw = new double[n];
        var v = new double[n];
        var st = new double[n];
        var fl = new int[n];

        for (int i = 0; i < n; i++)
        {
            int o = RouteImage.HeadBytes + i * RouteImage.RecBytes;
            x[i] = BitConverter.ToInt32(img, o + 0) / 1000.0;
            y[i] = BitConverter.ToInt32(img, o + 4) / 1000.0;
            yaw[i] = BitConverter.ToInt16(img, o + 8) / 100.0;
            v[i] = BitConverter.ToInt16(img, o + 10) / 100.0;
            st[i] = BitConverter.ToInt16(img, o + 12) / 100.0;
            fl[i] = BitConverter.ToUInt16(img, o + 14);
        }

        double kp = 2.0 * L_M / Math.Max(0.05, ld);
        double pbClamped = Math.Clamp(pbV, -2.0, 2.0);

        for (int i = 0; i < n; i++)
        {
            var row = new Row
            {
                I = i, X = x[i], Y = y[i], Yaw = yaw[i], VRec = v[i], DRec = st[i],
                Rev = ((fl[i] & FlagRev) != 0) || (v[i] < -RevEps),
                SteerValid = (fl[i] & FlagSteer) != 0,
            };

            // ---------------- 旧逻辑：不认倒车段 ----------------
            {
                row.VOld = pbClamped;
                row.DOld = PursueDelta(x, y, yaw, n, i, ld, kp);
            }

            // ---------------- 新逻辑：倒车分支 ----------------
            if (row.Rev)
            {
                double vlim = Math.Min(Math.Abs(pbClamped), RevVmax);
                double vv = v[i];
                if (vv < -vlim) vv = -vlim;
                if (vv > 0) vv = 0;
                row.VNew = vv;

                double d = row.SteerValid ? st[i] : 0.0;
                row.DNew = Math.Clamp(d, -DeltaMax, DeltaMax);
            }
            else
            {
                double vv = pbClamped;

                // 换向点在眼前 ⇒ 爬行
                if (i + 1 < n)
                {
                    bool nextRev = ((fl[i + 1] & FlagRev) != 0) || (v[i + 1] < -RevEps);
                    if (nextRev && vv > RevCreep) vv = RevCreep;
                }

                row.VNew = vv;

                // ★ 到换向点的剩余弧长 < 一个前视 ⇒ 照抄录制打角（别让前视点越进倒车段）
                int rv = -1;
                for (int k = i; k < n; k++)
                {
                    if (((fl[k] & FlagRev) != 0) || (v[k] < -RevEps)) { rv = k; break; }
                }

                double drem = 0;
                if (rv > i)
                {
                    for (int k = i; k < rv; k++)
                    {
                        double ex = x[k + 1] - x[k], ey = y[k + 1] - y[k];
                        drem += Math.Sqrt(ex * ex + ey * ey);
                    }
                }

                row.DNew = (rv >= 0 && drem < ld)
                         ? (row.SteerValid ? Math.Clamp(st[i], -DeltaMax, DeltaMax) : 0.0)
                         : PursueDelta(x, y, yaw, n, i, ld, kp);
            }

            // 统计（只统计"这一拍确实应该照抄录制打角"的点：正车段 + 有 STEER 位）
            if (Math.Abs(row.DOld) >= DeltaMax - 1.0) r.OldFullLock++;
            if (Math.Abs(row.DNew) >= DeltaMax - 1.0) r.NewFullLock++;

            if (!row.Rev && row.SteerValid)
            {
                double eOld = Math.Abs(Wrap180(row.DOld - st[i]));
                double eNew = Math.Abs(Wrap180(row.DNew - st[i]));
                if (eOld > r.OldMaxErr) r.OldMaxErr = eOld;
                if (eNew > r.NewMaxErr) r.NewMaxErr = eNew;
                if (eOld > 10.0) r.OldWorse10++;
                if (eNew > 10.0) r.NewWorse10++;
            }

            r.Rows.Add(row);
        }

        r.TailRev = TailIsRev(v, fl, n);
        r.Text = Build(r, n, pbV, ld, kp);
        return r;
    }

    /// <summary>纯跟踪：沿表往前 ld 米取前视点，算 α 与 δ（与车端同一套公式）。</summary>
    private static double PursueDelta(double[] x, double[] y, double[] yaw, int n, int best, double ld, double kp)
    {
        double acc = 0;
        int j = best;
        for (int i = best; i < n - 1; i++)
        {
            double dx = x[i + 1] - x[i], dy = y[i + 1] - y[i];
            acc += Math.Sqrt(dx * dx + dy * dy);
            if (acc >= ld) { j = i + 1; break; }
            j = n - 1;
        }

        double dxr = x[j] - x[best];
        double dyr = y[j] - y[best];
        double pr = yaw[best] * Math.PI / 180.0;
        double bx = dxr * Math.Cos(pr) + dyr * Math.Sin(pr);
        double by = -dxr * Math.Sin(pr) + dyr * Math.Cos(pr);

        double alpha = Math.Atan2(bx, by);            // rad
        double delta = kp * alpha * 180.0 / Math.PI;  // δ = −k·(target−meas) = k·α
        return Math.Clamp(delta, -DeltaMax, DeltaMax);
    }

    private static string Build(Result r, int n, double pbV, double ld, double kp)
    {
        var sb = new StringBuilder();
        sb.AppendLine($"逐点指令推演（假设车精确落在每个点上；pb_v={pbV:0.00} m/s，ld={ld:0.00} m，k={kp:0.00}）");
        sb.AppendLine("旧 = 不认倒车段（速度恒 pb_v + 纯跟踪）；新 = 认倒车段（表内负速度 + 开环回放打角）");
        sb.AppendLine("──────────────────────────────────────────────────────────────────────────────────────────");
        sb.AppendLine("  i      x       y     yaw   REV │  旧 v/δ         新 v/δ         录制δ   备注");
        sb.AppendLine("──────────────────────────────────────────────────────────────────────────────────────────");

        foreach (var row in r.Rows)
        {
            string note = "";
            if (row.Rev) note = "倒车段";
            if (Math.Abs(row.DNew) >= DeltaMax - 1.0) note += " ⚠新也满舵";
            if (!row.Rev && Math.Abs(row.DOld) >= DeltaMax - 1.0) note += " 旧满舵";

            sb.AppendLine($"{row.I,3} {row.X,7:F3} {row.Y,7:F3} {row.Yaw,6:F1} {(row.Rev ? "REV" : "  - "),4} │ "
                        + $"{row.VOld,6:F2}/{row.DOld,6:F1}  {row.VNew,6:F2}/{row.DNew,6:F1}   {row.DRec,6:F1}   {note}");
        }

        sb.AppendLine("──────────────────────────────────────────────────────────────────────────────────────────");
        sb.AppendLine($"旧逻辑：{r.OldFullLock} 个点被指令成满舵（|δ| ≥ {DeltaMax - 1:0}°）；正车段与录制打角最大差 {r.OldMaxErr:F1}°");
        sb.AppendLine($"新逻辑：{r.NewFullLock} 个点被指令成满舵；正车段与录制打角最大差 {r.NewMaxErr:F1}°");

        int rev = r.Rows.Count(z => z.Rev);
        sb.AppendLine($"倒车段 {rev} 点（占 {100.0 * rev / n:F0}%）：新逻辑下速度 = 表内负值（被 |pb_v| 与 {RevVmax:0.00} 封顶），转向 = 录制打角。");
        sb.AppendLine(r.TailRev
            ? $"★ 表尾状态 = 倒车段 ⇒ 收尾策略：**先刹停再收**（后轮使能+目标 0，舵机保持不回正；|v| ≤ {TailV:0.00} 或 {TailMs} ms 超时后判完成）"
            : "★ 表尾状态 = 前进段 ⇒ 收尾策略：到点即收（维持原行为）");
        sb.AppendLine();
        sb.AppendLine("★ 判读：正车段 δ 与「录制打角」有差是**正常的**（回放走纯跟踪，不是照抄打角）；");
        sb.AppendLine("   但如果正车段出现大面积满舵，说明 pb_v 相对弯太紧、或 ld 太小。");
        sb.AppendLine("   倒车段若「新」不等于「录制δ」且不是夹取造成，说明 STEER 位没置或记录本身有问题。");
        return sb.ToString();
    }

    /// <summary>表尾是否为倒车段：最后 TailN 个点里多数是倒车点（末点可能是 v=0 的收尾点）。</summary>
    private static bool TailIsRev(double[] v, int[] fl, int n)
    {
        int from = Math.Max(0, n - TailN);
        int cnt = 0, revn = 0;
        for (int i = from; i < n; i++)
        {
            cnt++;
            if (((fl[i] & FlagRev) != 0) || (v[i] < -RevEps)) revn++;
        }
        return cnt > 0 && revn * 2 >= cnt;
    }

    private static double Wrap180(double a)
    {
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }
}
