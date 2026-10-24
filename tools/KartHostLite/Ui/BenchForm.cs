using KartHost.Ui;

namespace KartHost;

/// <summary>
/// ★ 台架直给（2026-10-24）——上位机用滑条直接给车端驱动量，做台架测试/标定。
///
/// 与参数面板的区别：
///   · 参数面板发的是 <see cref="Cmd.SetParam"/>，写的是 `param_cache[]`（**标定量**，可存 Flash）；
///   · 本窗口发的是 <see cref="Cmd.SetTarget"/>，写的是 `Servo_Target_Deg` / 电机 target（**瞬时驱动量**），
///     不进 Flash、不参与「存 Flash」，关窗口就作废。
///
/// 车端生效条件（缺一不可，全在车端 `host_drive_tick()` 里判）：
///   CMD 模式（车端 Host 页按确认键切） + 链路 ON + 心跳未锁存 + 对应使能位 = 1。
/// ⇒ **UP 模式下滑条不会动车**，这是有意的：安全责任落在"切 CMD"那一下。
///
/// 两个刻意的设计：
///   1. 定时器里**只在有变化时**发一帧（滑条拖动会连发，静止时不占带宽）；
///   2. 关闭窗口时发 `SET_ENABLE = 0` + 目标归零 —— 防止"关了窗口车还在按最后那个目标跑"。
/// </summary>
internal sealed class BenchForm : Form
{
    private readonly Func<Cmd, byte, float, bool> _send;   // 交给主窗体的发送通道

    private CheckBox _ckSteer, _ckMotor;
    private TrackBar _tbSteer, _tbWheel;
    private NumericUpDown _numSteer, _numWheel;
    private Label _lblSteer, _lblWheel, _lblHint;
    private System.Windows.Forms.Timer _tick;

    private float _sentSteer = float.NaN, _sentWheel = float.NaN;
    private int _sentEnable = -1;
    private long _framesSent;

    private const float SteerMax = 28.0f;     // = 车端 SERVO_SOFT_LIMIT_DEG
    private const float WheelMax = 3.2f;      // = 车端 MOTOR_CRTL_TARGET_MAX

    public BenchForm(Func<Cmd, byte, float, bool> send)
    {
        _send = send ?? throw new ArgumentNullException(nameof(send));

        Text = "台架直给 · 舵机 / 两轮";
        StartPosition = FormStartPosition.CenterParent;
        FormBorderStyle = FormBorderStyle.FixedDialog;
        MaximizeBox = false;
        MinimizeBox = false;
        ClientSize = new Size(Dpi.Px(560), Dpi.Px(368));
        BackColor = Theme.Bg;
        ForeColor = Theme.Text;
        Font = new Font("Microsoft YaHei UI", 9f);

        int W = ClientSize.Width;

        _lblHint = new Label
        {
            Text = "车端需先在 Host 页切到 CMD 模式；UP 模式下滑条不动车。",
            Location = new Point(Dpi.Px(12), Dpi.Px(12)),
            Size = new Size(W - Dpi.Px(24), Dpi.Px(44)),
            ForeColor = Theme.TextDim,
        };
        Controls.Add(_lblHint);

        int y = Dpi.Px(66);
        int gh = Dpi.Px(96);

        // ---------------- 舵机 ----------------
        var gbS = new GroupBox
        {
            Text = "舵机目标角（±28° 软限位）",
            Location = new Point(Dpi.Px(12), y),
            Size = new Size(W - Dpi.Px(24), gh),
            ForeColor = Theme.Text,
        };

        _ckSteer = new CheckBox
        {
            Text = "使能 STEER",
            AutoSize = true,
            Location = new Point(Dpi.Px(14), Dpi.Px(24)),
        };
        _ckSteer.CheckedChanged += (_, __) => Push();

        _tbSteer = new TrackBar
        {
            Minimum = (int)(-SteerMax * 10), Maximum = (int)(SteerMax * 10),
            TickFrequency = 100, SmallChange = 5, LargeChange = 50,
            Location = new Point(Dpi.Px(12), Dpi.Px(46)),
            Width = Dpi.Px(300), Height = Dpi.Px(40),
            Value = 0,
        };
        _tbSteer.ValueChanged += (_, __) => { _numSteer.Value = _tbSteer.Value / 10m; };

        _numSteer = new NumericUpDown
        {
            DecimalPlaces = 1, Minimum = (decimal)(-SteerMax), Maximum = (decimal)SteerMax,
            Increment = 0.5m, Value = 0m,
            Location = new Point(Dpi.Px(324), Dpi.Px(52)),
            Width = Dpi.Px(80),
        };
        _numSteer.ValueChanged += (_, __) =>
        {
            _tbSteer.Value = (int)Math.Round(_numSteer.Value * 10m);
            Push();
        };

        _lblSteer = new Label
        {
            Text = "0.0°",
            Location = new Point(Dpi.Px(416), Dpi.Px(54)),
            Size = new Size(Dpi.Px(110), Dpi.Px(22)),
            ForeColor = Theme.Accent,
        };

        gbS.Controls.AddRange(new Control[] { _ckSteer, _tbSteer, _numSteer, _lblSteer });
        Controls.Add(gbS);

        // ---------------- 两轮 ----------------
        y += gh + Dpi.Px(8);
        var gbW = new GroupBox
        {
            Text = "两轮转速（同速，不走差速）",
            Location = new Point(Dpi.Px(12), y),
            Size = new Size(W - Dpi.Px(24), gh),
            ForeColor = Theme.Text,
        };

        _ckMotor = new CheckBox
        {
            Text = "使能 MOTOR",
            AutoSize = true,
            Location = new Point(Dpi.Px(14), Dpi.Px(24)),
        };
        _ckMotor.CheckedChanged += (_, __) => Push();

        _tbWheel = new TrackBar
        {
            Minimum = (int)(-WheelMax * 100), Maximum = (int)(WheelMax * 100),
            TickFrequency = 100, SmallChange = 5, LargeChange = 50,
            Location = new Point(Dpi.Px(12), Dpi.Px(46)),
            Width = Dpi.Px(300), Height = Dpi.Px(40),
            Value = 0,
        };
        _tbWheel.ValueChanged += (_, __) => { _numWheel.Value = _tbWheel.Value / 100m; };

        _numWheel = new NumericUpDown
        {
            DecimalPlaces = 2, Minimum = (decimal)(-WheelMax), Maximum = (decimal)WheelMax,
            Increment = 0.05m, Value = 0m,
            Location = new Point(Dpi.Px(324), Dpi.Px(52)),
            Width = Dpi.Px(80),
        };
        _numWheel.ValueChanged += (_, __) =>
        {
            _tbWheel.Value = (int)Math.Round(_numWheel.Value * 100m);
            Push();
        };

        _lblWheel = new Label
        {
            Text = "0.00 rps",
            Location = new Point(Dpi.Px(416), Dpi.Px(54)),
            Size = new Size(Dpi.Px(110), Dpi.Px(22)),
            ForeColor = Theme.Accent,
        };

        gbW.Controls.AddRange(new Control[] { _ckMotor, _tbWheel, _numWheel, _lblWheel });
        Controls.Add(gbW);

        // ---------------- 底排 ----------------
        y += gh + Dpi.Px(12);

        var btnZero = new Button
        {
            Text = "全部归零",
            Location = new Point(Dpi.Px(12), y),
            Width = Dpi.Px(100), Height = Dpi.Px(30),
        };
        btnZero.Click += (_, __) =>
        {
            _numSteer.Value = 0m;
            _numWheel.Value = 0m;
            Push();
        };

        var btnClose = new Button
        {
            Text = "关闭（失能 + 归零）",
            Location = new Point(Dpi.Px(120), y),
            Width = Dpi.Px(190), Height = Dpi.Px(30),
        };
        btnClose.Click += (_, __) => Close();

        Controls.AddRange(new Control[] { btnZero, btnClose });

        _tick = new System.Windows.Forms.Timer { Interval = 50 };
        _tick.Tick += (_, __) => Push();
        _tick.Start();
    }

    /// <summary>只在"有变化"时发；使能位变化立刻发一帧 SET_ENABLE。</summary>
    private void Push()
    {
        int enable = (_ckSteer.Checked ? (int)EnableBits.Steer : 0)
                   | (_ckMotor.Checked ? (int)EnableBits.Motor : 0);

        if (enable != _sentEnable)
        {
            if (_send(Cmd.SetEnable, 0, enable)) { _sentEnable = enable; _framesSent++; }
        }

        float st = (float)_numSteer.Value;
        if (!st.Equals(_sentSteer))
        {
            if (_send(Cmd.SetTarget, (byte)TargetId.Steer, st)) { _sentSteer = st; _framesSent++; }
        }

        float wh = (float)_numWheel.Value;
        if (!wh.Equals(_sentWheel))
        {
            if (_send(Cmd.SetTarget, (byte)TargetId.Wheel, wh)) { _sentWheel = wh; _framesSent++; }
        }

        _lblSteer.Text = st.ToString("0.0") + "°";
        _lblWheel.Text = wh.ToString("0.00") + " rps";
        _lblHint.Text = $"已发 {_framesSent} 帧（只发变化）。车端需在 Host 页切到 CMD 模式；UP 模式不动车。";
    }

    protected override void OnFormClosing(FormClosingEventArgs e)
    {
        _tick?.Stop();

        // ★ 收尾必须"失能 + 归零"：否则关了窗口，车还按最后那个目标跑
        _send(Cmd.SetTarget, (byte)TargetId.Steer, 0f);
        _send(Cmd.SetTarget, (byte)TargetId.Wheel, 0f);
        _send(Cmd.SetEnable, 0, 0f);

        base.OnFormClosing(e);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing) { _tick?.Dispose(); }
        base.Dispose(disposing);
    }
}
