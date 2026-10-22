using System.Drawing.Drawing2D;

namespace KartHost.Ui;

/// <summary>
/// odom 轨迹图（只读）—— 把车端航位推算的 <c>ins_x</c> / <c>ins_y</c> 画成平面轨迹。
///
/// <para><b>坐标约定（与车端一致）</b>：x 向右为正、y 向前为正。屏幕 y 轴朝下，
/// 所以这里画的是 <c>screen_y ∝ -y</c> —— **车头朝上**、左转往左拐。
/// 直接按 +y 画会让整张轨迹上下镜像，"向右转"看着像"向左转"。</para>
///
/// <para><b>只读</b>：不写任何下行命令、不碰参数面板。清空/暂停/等比缩放都只影响本机显示。</para>
///
/// <para><b>标定档案没有这两路</b>（18 路里没有 ins_x/ins_y）⇒ 整块显示 N/A，
/// 而不是画一张空白图 —— 空白图会被误读成"车没动"。</para>
///
/// <para>三条与 <see cref="WavePanel"/> 同源的纪律：重绘由外部固定 30 FPS 驱动、
/// 与接收解耦；点数多于像素时做抽稀；数据从 <see cref="ChannelStore"/> 取快照，不碰接收线程。</para>
/// </summary>
public sealed class TrackPanel : Control
{
    public ChannelStore Store { get; set; }

    /// <summary>{x, y} 两路下标。<c>null</c> 或长度 ≠ 2 = 本档案没有 odom ⇒ 显示 N/A。</summary>
    public int[] OdomXy { get; set; }

    public bool Paused { get; set; }

    /// <summary>等比缩放（1:1）。关掉就两轴各自撑满 —— 看清楚"轨迹被拉长了多少"时有用。</summary>
    public bool Isometric { get; set; } = true;

    public string Title { get; set; } = "odom 轨迹（x 横轴 / -y 纵轴，车头朝上）";

    private bool HasOdom => OdomXy != null && OdomXy.Length == 2;

    private readonly float[] _vx = new float[ChannelStore.Capacity];
    private readonly float[] _vy = new float[ChannelStore.Capacity];
    private readonly double[] _tx = new double[ChannelStore.Capacity];
    private readonly double[] _ty = new double[ChannelStore.Capacity];
    private readonly PointF[] _pts = new PointF[ChannelStore.Capacity + 8];

    private double _right;          // 冻结时刻（暂停用）
    private double _clearT = double.NaN;   // 清空时刻：只画它之后的点

    private const int PadLeft = 52, PadRight = 14, PadTop = 26, PadBottom = 24;

    public TrackPanel()
    {
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer
               | ControlStyles.UserPaint | ControlStyles.ResizeRedraw, true);
        BackColor = Theme.Canvas;
    }

    /// <summary>由外部 30 FPS 定时器调用</summary>
    public void Tick()
    {
        if (!Paused && Store != null && Store.Count > 0) _right = Store.LatestTime();
        Invalidate();
    }

    /// <summary>清空：只丢**显示**，不动 ChannelStore（波形图还要用那份数据）</summary>
    public void Clear()
    {
        _clearT = Store != null ? Store.LatestTime() : double.NaN;
        Invalidate();
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        var g = e.Graphics;
        g.SmoothingMode = SmoothingMode.AntiAlias;
        g.Clear(Theme.Canvas);

        int pw = Width - PadLeft - PadRight, ph = Height - PadTop - PadBottom;
        if (pw < 40 || ph < 40) return;

        using (var frame = new Pen(Theme.Border, 1))
            g.DrawRectangle(frame, PadLeft, PadTop, pw, ph);

        DrawTitle(g);

        if (!HasOdom)
        {
            CenterNote(g, pw, ph, "N/A —— 本档案没有 ins_x / ins_y\n（标定·精简 18 路里没有这两路，切回全量 41 路再看）");
            return;
        }

        if (Store == null) return;

        int n = Store.Snapshot(OdomXy[0], _vx, _tx, ChannelStore.Capacity);
        int m = Store.Snapshot(OdomXy[1], _vy, _ty, ChannelStore.Capacity);
        int cnt = Math.Min(n, m);
        if (cnt <= 0)
        {
            CenterNote(g, pw, ph, "等待数据…（车端进 Host 页 + 打开 Host Link 后推一下车）");
            return;
        }

        double tRight = Paused ? _right : Store.LatestTime();

        // ---- 取窗口内的点，同时算包围盒 ----
        // 注意：纵轴取 -y（屏幕 y 朝下），所以包围盒也按 -y 算
        int first = -1, last = -1;
        double xLo = double.MaxValue, xHi = double.MinValue;
        double yLo = double.MaxValue, yHi = double.MinValue;

        for (int k = 0; k < cnt; k++)
        {
            double t = _tx[k];
            if (!double.IsNaN(_clearT) && t < _clearT) continue;
            if (Paused && t > tRight) break;

            float x = _vx[k], y = _vy[k];
            if (float.IsNaN(x) || float.IsInfinity(x)) continue;
            if (float.IsNaN(y) || float.IsInfinity(y)) continue;

            if (first < 0) first = k;
            last = k;
            if (x < xLo) xLo = x;
            if (x > xHi) xHi = x;
            double sym = -y;                        // 屏幕 y 朝下 ⇒ 画 -y 才是"车头朝上"
            if (sym < yLo) yLo = sym;
            if (sym > yHi) yHi = sym;
        }

        if (first < 0)
        {
            CenterNote(g, pw, ph, "这一刻没有点（刚清空过？推车试试）");
            return;
        }

        double xSpan = xHi - xLo, ySpan = yHi - yLo;
        if (xSpan < 1e-9) { xLo -= 0.5; xHi += 0.5; xSpan = 1.0; }
        if (ySpan < 1e-9) { yLo -= 0.5; yHi += 0.5; ySpan = 1.0; }

        const double padFrac = 0.08;
        double xPad = xSpan * padFrac, yPad = ySpan * padFrac;

        double sx, sy;
        if (Isometric)
        {
            // 等比：一个"米"在 x / y 上占同样像素，否则圆轨迹会变椭圆、直道会歪
            double k = Math.Min(pw / (xSpan + 2 * xPad), ph / (ySpan + 2 * yPad));
            sx = sy = k;
        }
        else
        {
            sx = pw / (xSpan + 2 * xPad);
            sy = ph / (ySpan + 2 * yPad);
        }

        double cx = (xLo + xHi) * 0.5;
        double cy = (yLo + yHi) * 0.5;
        double ox = PadLeft + pw * 0.5;
        double oy = PadTop + ph * 0.5;

        float MapX(double x) => (float)(ox + (x - cx) * sx);
        float MapY(double sY) => (float)(oy - (sY - cy) * sy);   // 屏幕 y 向上 = 值增大

        DrawGrid(g, pw, ph, MapX, MapY, cx, cy, xSpan, ySpan);

        // ---- 折线（单色，6000 点逐段上色代价太大；时间信息靠"末点"和起终点标记给）----
        int np = 0;
        for (int k = first; k <= last && np < _pts.Length; k++)
        {
            if (!double.IsNaN(_clearT) && _tx[k] < _clearT) continue;
            float x = _vx[k], y = _vy[k];
            if (float.IsNaN(x) || float.IsNaN(y)) continue;
            _pts[np++] = new PointF(MapX(x), MapY(-y));
        }
        if (np >= 2)
        {
            using var pen = new Pen(Theme.Route, 1.6f) { LineJoin = LineJoin.Round };
            g.DrawLines(pen, _pts.AsSpan(0, np));
        }

        // 起点（绿）/ 当前位置（黄）
        if (np > 0)
        {
            using (var b = new SolidBrush(Theme.Ok)) g.FillEllipse(b, _pts[0].X - 3, _pts[0].Y - 3, 6, 6);
            using (var b = new SolidBrush(Theme.Nose)) g.FillEllipse(b, _pts[np - 1].X - 4, _pts[np - 1].Y - 4, 8, 8);
        }

        DrawInfo(g, pw, ph, _vx[last], _vy[last], first, last, cnt);
    }

    private void DrawGrid(Graphics g, int pw, int ph,
                          Func<double, float> mapX, Func<double, float> mapY,
                          double cx, double cy,
                          double xSpan, double ySpan)
    {
        using var minor = new Pen(Theme.Grid, 1);
        using var axis = new Pen(Theme.Axis, 1.4f);
        using var f = new Font("Consolas", 7.5f);
        using var b = new SolidBrush(Theme.TextDim);

        // 过原点的两条轴：回答"车现在相对起点在哪一侧"
        float ax = mapX(0);
        if (ax >= PadLeft && ax <= PadLeft + pw) g.DrawLine(axis, ax, PadTop, ax, PadTop + ph);
        float ay = mapY(0);
        if (ay >= PadTop && ay <= PadTop + ph) g.DrawLine(axis, PadLeft, ay, PadLeft + pw, ay);

        // 每 1/4 屏画一条参考线，标注米数
        double xStep = NiceStep(xSpan / 4.0);
        for (double v = Math.Ceiling((cx - xSpan) / xStep) * xStep; v <= cx + xSpan; v += xStep)
        {
            float x = mapX(v);
            if (x < PadLeft || x > PadLeft + pw) continue;
            g.DrawLine(minor, x, PadTop, x, PadTop + ph);
            string s = v.ToString("0.##");
            var sz = g.MeasureString(s, f);
            g.DrawString(s, f, b, x - sz.Width / 2, PadTop + ph + 3);
        }

        double yStep = NiceStep(ySpan / 4.0);
        for (double v = Math.Ceiling((cy - ySpan) / yStep) * yStep; v <= cy + ySpan; v += yStep)
        {
            float y = mapY(v);
            if (y < PadTop || y > PadTop + ph) continue;
            g.DrawLine(minor, PadLeft, y, PadLeft + pw, y);
            string s = v.ToString("0.##");
            var sz = g.MeasureString(s, f);
            g.DrawString(s, f, b, PadLeft - 5 - sz.Width, y - sz.Height / 2);
        }
    }

    private void DrawTitle(Graphics g)
    {
        using var f = new Font("微软雅黑", 9f, FontStyle.Bold);
        using var b = new SolidBrush(Theme.Text);
        g.DrawString(Title, f, b, PadLeft, 6);
    }

    private void DrawInfo(Graphics g, int pw, int ph, float x, float y, int first, int last, int cnt)
    {
        using var f = new Font("Consolas", 8f);
        using var b = new SolidBrush(Theme.TextDim);
        string s = $"x={x:0.000} m   y={y:0.000} m   点数 {last - first + 1}/{cnt}   起点 ●绿  当前 ●黄";
        g.DrawString(s, f, b, PadLeft, PadTop + ph + 8);
    }

    private void CenterNote(Graphics g, int pw, int ph, string msg)
    {
        using var f = new Font("微软雅黑", 9.5f);
        using var b = new SolidBrush(Theme.TextDim);
        var sz = g.MeasureString(msg, f, new SizeF(pw - 12, 0));
        g.DrawString(msg, f, b,
                     PadLeft + (pw - sz.Width) / 2,
                     PadTop + (ph - sz.Height) / 2);
    }

    /// <summary>把任意步长吸附到 1 / 2 / 5 × 10^n（刻度才会是整数）</summary>
    private static double NiceStep(double raw)
    {
        if (raw <= 0 || double.IsNaN(raw) || double.IsInfinity(raw)) return 1;
        double e = Math.Pow(10, Math.Floor(Math.Log10(raw)));
        double m = raw / e;
        double f = m <= 1 ? 1 : m <= 2 ? 2 : m <= 5 ? 5 : 10;
        return f * e;
    }
}
