using System.Diagnostics;
using KartHost.Ui;

namespace KartHost;

/// <summary>
/// 轻量上位机主窗 —— 从队友 KartHost 迁移来的「基础功能」部分，**不含**惯导/路表/标定/模拟车端。
///
/// <para><b>只做四件事</b>：收帧 → 画波形 → 调参数 → 录 CSV。</para>
///
/// <para><b>界面骨架照队友那份来</b>（2026-10-17 改版，之前是自造的三栏，观感差很多）：</para>
/// <code>
/// ┌──────────────────────────────────────────────────────────────┐
/// │ ●链路  UDP  端口  [开始接收]  心跳  ...  提示                 │ ← TopBar
/// ├────────────┬─────────────────────────────────┬───────────────┤
/// │ 参数        │ 布局 1×1 2×1 2×2  时间窗 …     │ 实时值/通道    │
/// │ （滑条+数字）│ ┌──────────┬──────────┐        │ ┌────┬────┐   │
/// │            │ │ 波形 1   │ 波形 2   │        │ │格子│格子│   │
/// │            │ ├──────────┼──────────┤        │ └────┴────┘   │
/// │            │ │ 波形 3   │ 波形 4   │        │ ☑ 0 srv_tgt   │
/// │ 存/读 Flash │ └──────────┴──────────┘        │ ☑ 1 srv_ang   │
/// ├────────────┴─────────────────────────────────┴───────────────┤
/// │ 日志（连接 / 参数写入 / CRC / 错误）                          │
/// ├──────────────────────────────────────────────────────────────┤
/// │ 未连接 | 帧率 | 丢包 | 带宽 | CRC 错 | 参数改动                │ ← StatusStrip
/// └──────────────────────────────────────────────────────────────┘
/// </code>
///
/// <para><b>架构要点（照抄队友那条最有价值的决定）</b>：网络线程只往 <see cref="ChannelStore"/> 写，
/// 界面由固定 30 FPS 的定时器取快照重绘 —— **重绘与接收解耦**。
/// 不这么做的典型症状是"车端帧率一变，界面就卡"。</para>
///
/// <para>⚠ 链路拓扑：**车端是 UDP 客户端**，主动发到本机 8086 端口。
/// 所以这里是「绑定本地端口收包」，不是"去连车"。这与队友 TCP 模式方向相反。</para>
/// </summary>
public sealed class MainForm : Form
{
    // ---- 数据与链路 ----
    private readonly Stopwatch _sw = Stopwatch.StartNew();
    private readonly ChannelStore _store = new();
    private readonly CsvRecorder _rec;
    private readonly List<ParamMeta> _tableBuf = new();

    private Link _link;
    private ParamPanel _param;

    // ---- 顶部连接栏 ----
    private LampControl _lampLink;
    private NumericUpDown _numPort;
    private Button _btnConn;
    private CheckBox _ckHb;

    // ---- 波形（1 / 2 / 4 张，跟队友一样用 TableLayoutPanel 拼）----
    private Panel _waveHost;
    private WavePanel[] _waves;
    private int _layout = 4;
    private ComboBox _cbWindow;
    private Button _btnPause;
    private CheckBox _ckAutoY;
    private Button _btnRec;

    // ---- 右侧栏：实时值 + 通道勾选 ----
    private Panel _valHost;
    private readonly Dictionary<int, Label> _valLabels = new();
    private FlowLayoutPanel _chkHost;
    private readonly Dictionary<int, CheckBox> _chkBoxes = new();
    /// <summary>整窗共用一个提示框：每个通道勾选框挂一条中文说明（见 <see cref="Defaults.ChannelTips"/>）</summary>
    private readonly ToolTip _tip = new() { ShowAlways = true, InitialDelay = 350, ReshowDelay = 150, AutoPopDelay = 30000 };

    // ---- 左栏参数脚注 ----
    private Label _lblDirty;
    private Button _btnRefresh, _btnSave, _btnDef;

    // ---- 日志 / 状态栏 ----
    private RichTextBox _log;
    private StatusStrip _status;
    private ToolStripStatusLabel _stLink, _stRate, _stLoss, _stBw, _stCrc, _stDirty;

    private SplitContainer _mainSplit, _rightSplit;

    private System.Windows.Forms.Timer _uiTimer, _statTimer, _tableTimer, _hbTimer;

    private ushort _tok = 1;
    private long _frames, _lost, _crcBad;
    private int _lastSeq = -1;
    private int _warnedCnt = -1;
    private int _frameChCount;
    private long _bytesAtStat;
    private long _framesAtStat;
    private double _tAtStat;

    public MainForm()
    {
        _rec = new CsvRecorder(_store);

        // ★ DPI 三件套，顺序是硬要求：
        //   ① 关自动缩放（默认 Font 模式会跟 Dpi.Px 叠加 → 字号被缩两遍）
        //   ② 建句柄：此时 DeviceDpi 才是这台显示器的真实值
        //   ③ 之后才能用 Dpi.Px 算尺寸
        AutoScaleMode = AutoScaleMode.None;
        CreateHandle();
        Dpi.Init(DeviceDpi);

        Text = $"卡丁快跑 · 轻量上位机 — {Defaults.ProfileName}（{Defaults.ChannelNames.Length} 通道）";

        // ★ 窗口尺寸不许超出工作区（150% 缩放下 1440 逻辑像素 = 2160 物理像素，会顶出版面）
        var wa = Screen.FromControl(this).WorkingArea;
        Width = Math.Max(Dpi.Px(900), Math.Min(Dpi.Px(1440), wa.Width - Dpi.Px(24)));
        Height = Math.Max(Dpi.Px(600), Math.Min(Dpi.Px(860), wa.Height - Dpi.Px(24)));
        MinimumSize = Dpi.Sz(900, 560);
        StartPosition = FormStartPosition.CenterScreen;
        BackColor = Theme.Bg;
        ForeColor = Theme.Text;

        // ★★ 字号一律写 9pt，**不要**乘 Dpi.Scale：点值是物理量，GDI+ 会自己按 DPI 放大。
        //   之前这里写的是 Dpi.Pxf(9f)，150% 下变成 13.5pt —— 字比队友大一倍，
        //   整窗看着"胖一圈"，就是这次改版要修掉的东西。
        Font = new Font("微软雅黑", 9f);

        // ★ Dock 顺序有讲究：WinForms 按 Z 序「最后添加的最先布局」，
        //   所以 Fill 的必须最先 Add，边缘 Dock 的后加，否则 Fill 会吃掉全部空间。
        BuildBody();        // Fill
        BuildTopBar();      // Top
        BuildLog();         // Bottom
        BuildStatusBar();   // Bottom（最后添加 → 最先布局 → 贴最底）

        Theme.Apply(this);
        PaintChannelColors();   // ★ 必须放在 Theme.Apply 之后：它会把 CheckBox 的 ForeColor 刷成 Text

        BuildTimers();
        Log("就绪。车端进 Host 页 → 打开 Host Link → 本机点「开始接收」。");
    }

    // ==================================================================
    // 顶部连接栏
    // ==================================================================
    private void BuildTopBar()
    {
        var bar = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(46), BackColor = Theme.Panel, Padding = Dpi.Pad(8, 6, 8, 6) };

        _lampLink = new LampControl { Caption = "链路", Location = new Point(Dpi.Px(8), Dpi.Px(8)), Size = Dpi.Sz(92, 30) };

        var lbPort = new Label { Text = "本地端口", AutoSize = true, Location = new Point(Dpi.Px(108), Dpi.Px(15)), ForeColor = Theme.TextDim };
        _numPort = new NumericUpDown
        {
            Width = Dpi.Px(72),
            Minimum = 1,
            Maximum = 65535,
            Value = 8086,
            Location = new Point(Dpi.Px(176), Dpi.Px(11)),
            BorderStyle = BorderStyle.FixedSingle,
        };

        _btnConn = new Button { Text = "开始接收", Width = Dpi.Px(96), Height = Dpi.Px(26), Location = new Point(Dpi.Px(258), Dpi.Px(10)) };
        _btnConn.Click += (_, __) => ToggleConnect();

        _ckHb = new CheckBox
        {
            Text = "心跳 100ms",
            AutoSize = true,
            Location = new Point(Dpi.Px(366), Dpi.Px(14)),
            ForeColor = Theme.Text,
            Checked = false,     // ★ 默认关：车端当前不依赖心跳（见 Host_Crtl.h 的 monitor 决定）
        };

        var lbProfile = new Label
        {
            Text = $"通道表：{Defaults.ProfileName}（{Defaults.ChannelNames.Length}）",
            AutoSize = true,
            Location = new Point(Dpi.Px(474), Dpi.Px(15)),
            ForeColor = Theme.TextDim,
        };

        var lbHint = new Label
        {
            // ⚠ AutoSize 标签太长会顶出父容器。整窗 1440 逻辑像素，这里从 664 起 → 最多 ~48 汉字。
            Text = "提示：车端是 UDP 客户端，本机只负责绑定端口收包，不用去“连接”车",
            AutoSize = true,
            Location = new Point(Dpi.Px(664), Dpi.Px(15)),
            ForeColor = Theme.TextDim,
        };

        bar.Controls.AddRange(new Control[] { _lampLink, lbPort, _numPort, _btnConn, _ckHb, lbProfile, lbHint });
        Controls.Add(bar);
    }

    // ==================================================================
    // 底部日志 / 状态栏
    // ==================================================================
    private void BuildLog()
    {
        var host = new Panel { Dock = DockStyle.Bottom, Height = Dpi.Px(108), BackColor = Theme.Panel };
        var head = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(22), BackColor = Theme.PanelAlt };
        head.Controls.Add(new Label
        {
            Text = "  日志（连接 / 参数写入 / CRC / 错误）",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = Theme.TextDim,
        });

        _log = new RichTextBox
        {
            Dock = DockStyle.Fill,
            BackColor = Theme.Bg,
            ForeColor = Theme.TextDim,
            BorderStyle = BorderStyle.None,
            ReadOnly = true,
            WordWrap = false,
            Font = new Font("Consolas", 8.5f),
            ScrollBars = RichTextBoxScrollBars.Vertical,
        };

        host.Controls.Add(_log);    // Fill 先加
        host.Controls.Add(head);    // Top 后加
        Controls.Add(host);
    }

    private void BuildStatusBar()
    {
        _status = new StatusStrip { SizingGrip = false, BackColor = Theme.Panel };
        _stLink = new ToolStripStatusLabel("未连接") { ForeColor = Theme.TextDim };
        _stRate = new ToolStripStatusLabel("帧率 --") { ForeColor = Theme.TextDim };
        _stLoss = new ToolStripStatusLabel("丢包 --") { ForeColor = Theme.TextDim };
        _stBw = new ToolStripStatusLabel("带宽 --") { ForeColor = Theme.TextDim };
        _stCrc = new ToolStripStatusLabel("CRC 错 0") { ForeColor = Theme.TextDim };
        _stDirty = new ToolStripStatusLabel("参数改动 0") { ForeColor = Theme.Warn };

        _status.Items.AddRange(new ToolStripItem[]
        {
            _stLink, new ToolStripSeparator(), _stRate, new ToolStripSeparator(),
            _stLoss, new ToolStripSeparator(), _stBw, new ToolStripSeparator(),
            _stCrc, new ToolStripSeparator(), _stDirty,
        });
        Controls.Add(_status);
    }

    // ==================================================================
    // 主体（左参数 / 中波形 / 右实时值）
    // ==================================================================
    private void BuildBody()
    {
        _mainSplit = new SplitContainer
        {
            Dock = DockStyle.Fill,
            Orientation = Orientation.Vertical,
            SplitterWidth = Dpi.Px(4),
            BackColor = Theme.Border,
        };

        // ---- 左：参数 ----
        var leftHost = new Panel { Dock = DockStyle.Fill, BackColor = Theme.Panel };
        var paramHead = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(28), BackColor = Theme.PanelAlt };
        paramHead.Controls.Add(new Label
        {
            Text = "  参数（滑条实时下发 / 数字框精确输入）",
            Dock = DockStyle.Fill,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = Theme.Text,
        });

        var paramFoot = new Panel { Dock = DockStyle.Bottom, Height = Dpi.Px(62), BackColor = Theme.Panel };
        _lblDirty = new Label
        {
            Text = "改动 0 项未保存",
            Location = new Point(Dpi.Px(10), Dpi.Px(4)),
            Size = Dpi.Sz(280, 18),
            ForeColor = Theme.Warn,
        };

        _btnRefresh = new Button { Text = "刷新参数表", Location = new Point(Dpi.Px(10), Dpi.Px(26)), Width = Dpi.Px(92), Height = Dpi.Px(28) };
        _btnSave = new Button { Text = "存 Flash", Location = new Point(Dpi.Px(108), Dpi.Px(26)), Width = Dpi.Px(80), Height = Dpi.Px(28) };
        _btnDef = new Button { Text = "读 Flash", Location = new Point(Dpi.Px(194), Dpi.Px(26)), Width = Dpi.Px(80), Height = Dpi.Px(28) };

        _btnRefresh.Click += (_, __) => SendCmd(Cmd.GetTable);
        _btnSave.Click += (_, __) =>
        {
            SendCmd(Cmd.SaveParams);
            _param?.ClearDirty();
            UpdateDirtyLabel();
        };
        // ★ 读 Flash = 让车端把 Flash 里存的那套值回灌并重新下发（Cmd.LoadParams），
        //   与「恢复默认」不是一回事：那个要车端擦参数区，语义更重，本工程不做。
        _btnDef.Click += (_, __) => SendCmd(Cmd.LoadParams);

        paramFoot.Controls.AddRange(new Control[] { _lblDirty, _btnRefresh, _btnSave, _btnDef });

        _param = new ParamPanel { Dock = DockStyle.Fill };
        _param.ParamChanged += OnParamChanged;

        leftHost.Controls.Add(_param);        // Fill 先加
        leftHost.Controls.Add(paramFoot);
        leftHost.Controls.Add(paramHead);

        // ---- 右：波形 + 右侧栏 ----
        _rightSplit = new SplitContainer
        {
            Dock = DockStyle.Fill,
            Orientation = Orientation.Vertical,
            SplitterWidth = Dpi.Px(4),
            BackColor = Theme.Border,
        };

        // 波形区
        _waveHost = new Panel { Dock = DockStyle.Fill, BackColor = Theme.Bg };
        var waveBar = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(34), BackColor = Theme.Panel };

        var lbLayout = new Label { Text = "布局", AutoSize = true, Location = new Point(Dpi.Px(8), Dpi.Px(8)), ForeColor = Theme.TextDim };
        var b1 = new Button { Text = "1×1", Location = new Point(Dpi.Px(44), Dpi.Px(4)), Width = Dpi.Px(46), Height = Dpi.Px(26) };
        var b2 = new Button { Text = "2×1", Location = new Point(Dpi.Px(94), Dpi.Px(4)), Width = Dpi.Px(46), Height = Dpi.Px(26) };
        var b4 = new Button { Text = "2×2", Location = new Point(Dpi.Px(144), Dpi.Px(4)), Width = Dpi.Px(46), Height = Dpi.Px(26) };
        b1.Click += (_, __) => ApplyLayout(1);
        b2.Click += (_, __) => ApplyLayout(2);
        b4.Click += (_, __) => ApplyLayout(4);

        var lbWin = new Label { Text = "时间窗", AutoSize = true, Location = new Point(Dpi.Px(206), Dpi.Px(8)), ForeColor = Theme.TextDim };
        _cbWindow = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = Dpi.Px(68), Location = new Point(Dpi.Px(254), Dpi.Px(4)) };
        _cbWindow.Items.AddRange(new object[] { "5 s", "10 s", "15 s", "30 s", "60 s" });
        _cbWindow.SelectedIndex = 2;
        _cbWindow.SelectedIndexChanged += (_, __) =>
        {
            double[] w = { 5, 10, 15, 30, 60 };
            if (_waves == null) return;
            foreach (var wv in _waves) if (wv != null) wv.TimeWindow = w[_cbWindow.SelectedIndex];
        };

        _ckAutoY = new CheckBox { Text = "自动量程", Checked = true, Location = new Point(Dpi.Px(334), Dpi.Px(6)), AutoSize = true, ForeColor = Theme.Text };
        _ckAutoY.CheckedChanged += (_, __) =>
        {
            if (_waves == null) return;
            foreach (var wv in _waves) if (wv != null) wv.AutoScale = _ckAutoY.Checked;
        };

        _btnPause = new Button { Text = "暂停", Location = new Point(Dpi.Px(428), Dpi.Px(4)), Width = Dpi.Px(58), Height = Dpi.Px(26) };
        _btnPause.Click += (_, __) =>
        {
            if (_waves == null || _waves.Length == 0) return;
            bool p = !_waves[0].Paused;
            foreach (var wv in _waves) if (wv != null) wv.Paused = p;
            _btnPause.Text = p ? "继续" : "暂停";
        };

        _btnRec = new Button { Text = "录制 CSV", Location = new Point(Dpi.Px(494), Dpi.Px(4)), Width = Dpi.Px(96), Height = Dpi.Px(26) };
        _btnRec.Click += (_, __) => ToggleRecord();

        waveBar.Controls.AddRange(new Control[] { lbLayout, b1, b2, b4, lbWin, _cbWindow, _ckAutoY, _btnPause, _btnRec });

        _rightSplit.Panel1.Controls.Add(_waveHost);   // Fill 先加
        _rightSplit.Panel1.Controls.Add(waveBar);     // Top 后加

        // ---- 右侧栏：实时值 + 通道勾选 ----
        var sideHost = new Panel { Dock = DockStyle.Fill, BackColor = Theme.Panel, AutoScroll = true };
        var sideHead = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(40), BackColor = Theme.PanelAlt };
        // ⚠ 这里刻意不用 Dock=Top 叠两个 Label：Dock 是按 z-order 逆序布的，
        //   先加的反而被挤到下面，标题/副标题会上下颠倒。用固定位置 + Anchor 最省事。
        sideHead.Controls.Add(new Label
        {
            Text = "  实时值 / 通道显隐",
            Location = new Point(0, 0),
            Size = Dpi.Sz(232, 21),
            Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = Theme.Text,
        });
        // 副标题：回答"这些是不是固定的"。25 路是车端按 HOST_CH_* 顺序发的固定表，
        // 帧里带 COUNT 自描述 —— 通道数变了不会解析失败，但**名字会整体错位**。
        sideHead.Controls.Add(new Label
        {
            Text = "  车→上位机 固定 25 路（下标即协议）",
            Location = new Point(0, Dpi.Px(21)),
            Size = Dpi.Sz(232, 18),
            Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right,
            TextAlign = ContentAlignment.MiddleLeft,
            ForeColor = Theme.TextDim,
            Font = new Font("微软雅黑", 8f),
        });

        _valHost = new Panel { Dock = DockStyle.Top, Height = Dpi.Px(200), BackColor = Theme.Panel };
        BuildValueBoxes(_valHost);

        _chkHost = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,       // 先加 → 最后布局 → 填剩余空间
            FlowDirection = FlowDirection.TopDown,
            WrapContents = false,
            AutoScroll = true,
            BackColor = Theme.Panel,
            Padding = Dpi.Pad(8, 4, 4, 6),
        };
        BuildChannelChecks();

        var chkFoot = new Panel { Dock = DockStyle.Bottom, Height = Dpi.Px(30), BackColor = Theme.Panel };
        var bAll = new Button { Text = "全选", Location = new Point(Dpi.Px(8), Dpi.Px(2)), Width = Dpi.Px(60), Height = Dpi.Px(26) };
        var bNone = new Button { Text = "清空", Location = new Point(Dpi.Px(74), Dpi.Px(2)), Width = Dpi.Px(60), Height = Dpi.Px(26) };
        bAll.Click += (_, __) => SetAllChecked(true);
        bNone.Click += (_, __) => SetAllChecked(false);
        chkFoot.Controls.AddRange(new Control[] { bAll, bNone });

        sideHost.Controls.Add(_chkHost);
        sideHost.Controls.Add(chkFoot);
        sideHost.Controls.Add(_valHost);
        sideHost.Controls.Add(sideHead);
        _rightSplit.Panel2.Controls.Add(sideHost);

        _mainSplit.Panel1.Controls.Add(leftHost);
        _mainSplit.Panel2.Controls.Add(_rightSplit);
        Controls.Add(_mainSplit);       // ★ Fill 必须最先加

        // ★ 分隔条位置不能在构造里设：那时宽度还没定型（会算出 0 或越界，
        //   表现是"参数面板整个消失"）。放到 Load 之后，宽度才是真实值。
        Load += (_, __) => ApplyRuntimeLayout();
        ApplyLayout(4);
    }

    private void ApplyRuntimeLayout()
    {
        try { _mainSplit.SplitterDistance = Dpi.Px(320); } catch { }
        try { _rightSplit.SplitterDistance = Math.Max(Dpi.Px(400), _rightSplit.Width - Dpi.Px(240)); } catch { }
    }

    /// <summary>给 --shot-form 用：把布局推到"Load 之后"的状态（否则量到的是默认分隔位置）</summary>
    internal void ApplyRuntimeLayoutForTest()
    {
        ApplyRuntimeLayout();
        PerformLayout();
    }

    /// <summary>给 --shot-form 用：指定分图数（1 / 2 / 4），检查另一种布局有没有崩</summary>
    internal void SetLayoutForTest(int n)
    {
        if (n == 1 || n == 2 || n == 4) ApplyLayout(n);
        PerformLayout();
    }

    // ==================================================================
    // 波形分图
    // ==================================================================
    /// <summary>
    /// 重排波形图为 <paramref name="n"/> 张（1 / 2 / 4）。
    /// 与队友同一套做法：清掉旧 WavePanel 重建，**Store 共用一份**（数据只有一份，画几遍而已）。
    /// </summary>
    private void ApplyLayout(int n)
    {
        _layout = n;
        _waveHost.SuspendLayout();
        _waveHost.Controls.Clear();
        foreach (var w in _waves ?? Array.Empty<WavePanel>()) w?.Dispose();

        _waves = new WavePanel[n];
        var grid = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = n == 1 ? 1 : 2,
            RowCount = n == 1 ? 1 : n == 2 ? 1 : 2,
            BackColor = Theme.Bg,
            Padding = new Padding(2),
        };
        for (int i = 0; i < grid.ColumnCount; i++) grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f / grid.ColumnCount));
        for (int i = 0; i < grid.RowCount; i++) grid.RowStyles.Add(new RowStyle(SizeType.Percent, 100f / grid.RowCount));

        for (int i = 0; i < n; i++)
        {
            var wp = new WavePanel
            {
                Store = _store,
                Dock = DockStyle.Fill,
                Margin = new Padding(2),
                TimeWindow = _cbWindow.SelectedIndex >= 0 ? new[] { 5.0, 10, 15, 30, 60 }[_cbWindow.SelectedIndex] : 15,
                AutoScale = _ckAutoY?.Checked ?? true,
                Paused = _btnPause?.Text == "继续",
            };
            _waves[i] = wp;
            grid.Controls.Add(wp, grid.ColumnCount == 1 ? 0 : i % 2, grid.RowCount == 1 ? 0 : i / 2);
        }

        _waveHost.Controls.Add(grid);
        _waveHost.ResumeLayout();

        // 把当前勾选分配下去；多图模式下再按预设覆盖分组
        foreach (var kv in _chkBoxes) OnChannelToggled(kv.Key, kv.Value.Checked);
        if (_layout > 1) ApplyPlotPresets();
        UpdateWaveTitles();
        UpdateChannelMatch();
    }

    private void ApplyPlotPresets()
    {
        if (_waves == null || _waves.Length <= 1) return;
        var ps = _waves.Length == 2 ? Defaults.PlotPresets2 : Defaults.PlotPresets4;
        for (int i = 0; i < _waves.Length; i++)
        {
            _waves[i].Shown.Clear();
            if (i < ps.Length)
                foreach (int c in ps[i].Ch) _waves[i].Shown.Add(c);
        }
        UpdateWaveTitles();
    }

    private void UpdateWaveTitles()
    {
        if (_waves == null) return;
        var ps = _waves.Length == 2 ? Defaults.PlotPresets2 : Defaults.PlotPresets4;
        for (int i = 0; i < _waves.Length; i++)
        {
            _waves[i].Title = _waves.Length == 1
                ? "全部勾选通道"
                : (i < ps.Length ? ps[i].Name : "图 " + (i + 1));
        }
    }

    // ==================================================================
    // 右侧栏
    // ==================================================================
    private void BuildValueBoxes(Panel host)
    {
        foreach (var lb in _valLabels.Values) lb.Parent?.Dispose();
        _valLabels.Clear();
        host.Controls.Clear();

        var vb = Defaults.ValueBoxes;
        for (int i = 0; i < vb.Length; i++)
        {
            int col = i % 2, row = i / 2;
            var box = new Panel
            {
                Location = new Point(Dpi.Px(8 + col * 104), Dpi.Px(6 + row * 64)),
                Size = Dpi.Sz(98, 58),
                BackColor = Theme.PanelAlt,
            };
            box.Paint += (s, e) =>
            {
                using var pen = new Pen(Theme.Border);
                e.Graphics.DrawRectangle(pen, 0, 0, ((Panel)s).Width - 1, ((Panel)s).Height - 1);
            };

            box.Controls.Add(new Label
            {
                Text = vb[i].Cap,
                Location = new Point(Dpi.Px(6), Dpi.Px(4)),
                Size = Dpi.Sz(86, 16),
                ForeColor = Theme.TextDim,
            });

            var val = new Label
            {
                Text = "--",
                Location = new Point(Dpi.Px(6), Dpi.Px(20)),
                Size = Dpi.Sz(62, 30),      // ⚠ 别加宽：6+62=68，单位标签从 70 起，加宽就压上去
                Font = new Font("Consolas", 14f, FontStyle.Bold),
                ForeColor = Theme.CurveOf(vb[i].Ch),
            };
            box.Controls.Add(val);

            box.Controls.Add(new Label
            {
                Text = vb[i].Unit,
                Location = new Point(Dpi.Px(70), Dpi.Px(32)),
                AutoSize = true,            // 固定 26px 宽在 150% 下装不下 "rps"（需要 40+）
                ForeColor = Theme.TextDim,
            });

            _valLabels[vb[i].Ch] = val;
            host.Controls.Add(box);
        }
    }

    private void BuildChannelChecks()
    {
        for (int i = 0; i < Defaults.ChannelNames.Length; i++)
        {
            int ch = i;
            var ck = new CheckBox
            {
                // ★ 英文名 + 中文简称（" 5  lr_v      左轮实测转速"）。
                //   宽度 216 是按「9 字符英文 + 6 个汉字 + 勾选标记」估的，
                //   再挤就会裁字（WinForms 不报错，只是看不到后半句）。
                Text = Defaults.ChannelText(i),
                Width = Dpi.Px(216),
                Height = Dpi.Px(20),
                BackColor = Theme.Panel,
                Font = new Font("Consolas", 8.5f),
            };
            _tip.SetToolTip(ck, Defaults.ChannelTip(i));
            ck.CheckedChanged += (_, __) => OnChannelToggled(ch, ck.Checked);
            _chkBoxes[i] = ck;
            _chkHost.Controls.Add(ck);
        }
    }

    /// <summary>
    /// ★ 通道字色必须放在 <see cref="Theme.Apply"/> **之后**再刷一遍 ——
    /// 那个方法会把所有 CheckBox 的 ForeColor 统一成 <see cref="Theme.Text"/>，
    /// 先设就全被覆盖成白色，曲线颜色和图例就对不上了。
    /// </summary>
    private void PaintChannelColors()
    {
        foreach (var kv in _chkBoxes) kv.Value.ForeColor = Theme.CurveOf(kv.Key);
        foreach (int i in Defaults.DefaultShown)
            if (_chkBoxes.TryGetValue(i, out var ck)) ck.Checked = true;

        // ★ 勾默认通道会逐条触发 OnChannelToggled（把通道塞进每一张图），
        //   所以**多图模式下必须在勾完之后再套一次预设** —— 否则分图预设会被这 6 条默认勾选冲掉，
        //   四张图看起来一模一样（每张都画全部默认通道）。
        if (_layout > 1) ApplyPlotPresets();
    }

    private void OnChannelToggled(int ch, bool on)
    {
        if (_waves == null) return;
        foreach (var w in _waves)
        {
            if (w == null) continue;
            if (on) w.Shown.Add(ch); else w.Shown.Remove(ch);
        }
        // 单图模式：勾选就是显隐；多图模式：分组由 ApplyPlotPresets 决定（与队友一致）
    }

    private void SetAllChecked(bool on)
    {
        foreach (var ck in _chkBoxes.Values) ck.Checked = on;
    }

    private void UpdateValueLabels()
    {
        foreach (var kv in _valLabels)
        {
            float v = _store.Latest(kv.Key);
            // ★ 数值框只有 62 逻辑像素（Consolas 14pt Bold），150% 下最多 5 个字符。
            //   所以按"字符数"封顶而不是按量级：|v|≥100 用 "0.0" 会到 "-9999.9"（7 字符）→ 剪尾巴，
            //   因此 ≥1000 用整数、≥10 用一位、其余两位小数。
            kv.Value.Text = float.IsNaN(v) ? "--"
                          : Math.Abs(v) >= 1000 ? v.ToString("0")
                          : Math.Abs(v) >= 10 ? v.ToString("0.0")
                          : v.ToString("0.00");
        }
    }

    private void UpdateDirtyLabel()
    {
        int n = _param?.DirtyCount ?? 0;
        if (_lblDirty != null) _lblDirty.Text = $"改动 {n} 项未保存";
        if (_stDirty != null) _stDirty.Text = $"参数改动 {n}";
    }

    // ==================================================================
    // 定时器
    // ==================================================================
    private void BuildTimers()
    {
        // ★ 固定 30 FPS 重绘，与接收解耦
        _uiTimer = new System.Windows.Forms.Timer { Interval = 33 };
        _uiTimer.Tick += (_, __) =>
        {
            if (_waves != null) foreach (var w in _waves) w?.Tick();
            UpdateValueLabels();
            UpdateDirtyLabel();
            if (_rec.IsRecording) _rec.Tick(_sw.Elapsed.TotalSeconds);
        };
        _uiTimer.Start();

        _statTimer = new System.Windows.Forms.Timer { Interval = 500 };
        _statTimer.Tick += (_, __) => UpdateStats();
        _statTimer.Start();

        // 参数表分批合并：收到第一条后起 300ms 定时器，到点整体重建一次界面
        _tableTimer = new System.Windows.Forms.Timer { Interval = 300 };
        _tableTimer.Tick += (_, __) =>
        {
            _tableTimer.Stop();
            if (_tableBuf.Count == 0) return;
            var list = new List<ParamMeta>(_tableBuf);
            _tableBuf.Clear();
            _param.MergeTable(list);
            Log($"参数表合并：{list.Count} 项（累计 {_param.Params.Count}）");
        };

        // 心跳默认关（车端 monitor 模型不要求保活）；勾上就走 100ms PING
        _hbTimer = new System.Windows.Forms.Timer { Interval = 100 };
        _hbTimer.Tick += (_, __) => { if (_ckHb.Checked) SendCmd(Cmd.Ping); };
        _hbTimer.Start();
    }

    // ==================================================================
    // 链
    // ==================================================================
    private void ToggleConnect()
    {
        if (_link != null && _link.IsOpen) { Disconnect(); return; }

        int port = (int)_numPort.Value;
        try
        {
            _link = new Link();
            _link.FrameReceived += (t, b, o) => OnFrame(t, b, o);
            _link.Log += s => Log(s);
            _link.StateChanged += up => BeginInvoke(new Action(() =>
            {
                _btnConn.Text = up ? "停止接收" : "开始接收";
                SetLamp(up);
                Log(up ? "链路就绪：" + _link.RemoteDesc : "链路已断开");
            }));
            _link.ConnectUdp(port);
            Log($"已绑定本地 UDP {port}，等待车端上报（车端是客户端，主动发到本机）");
        }
        catch (Exception ex)
        {
            _link?.Dispose();
            _link = null;
            Log("绑定失败：" + ex.Message + "（端口被占用？换一个端口，或关掉占用的程序）");
            MessageBox.Show(this, "绑定 UDP 端口失败：\n" + ex.Message, "卡丁快跑");
        }
    }

    private void Disconnect()
    {
        _link?.Close();
        _link = null;
        _btnConn.Text = "开始接收";
        SetLamp(false);
        Log("已停止接收");
    }

    /// <summary>链路指示灯：Off = 未绑定 / Fault = 绑了但还没收到过帧 / On = 正在收</summary>
    private void SetLamp(bool open)
    {
        if (_lampLink == null) return;
        _lampLink.State = !open ? LampControl.LampState.Off
                        : _frames > 0 ? LampControl.LampState.On
                        : LampControl.LampState.Fault;
    }

    private void SendCmd(Cmd c, byte id = 0, float v = 0f)
    {
        if (_link == null || !_link.IsOpen) { Log("未接收，命令未发出：" + c); return; }
        if (!_link.Send(Proto.BuildCommand(c, id, v, _tok++))) Log("发送失败：" + c);
    }

    // ==================================================================
    // 帧分发
    // ==================================================================
    private void OnFrame(byte type, byte[] b, int o)
    {
        switch ((FrameType)type)
        {
            case FrameType.Telemetry: ParseTelemetry(b, o); break;
            case FrameType.ParamMeta: ParseParamMeta(b, o); break;

            case FrameType.ParamValue:
                {
                    byte id = b[o];
                    float v = Proto.GetF32(b, o + 1);
                    BeginInvoke(new Action(() => _param.UpdateFromCar(id, v)));
                    break;
                }

            case FrameType.Ack:
                break;      // 心跳/参数回的 ACK 不刷屏

            case FrameType.Nak:
                {
                    byte cmd = b[o + 2], err = b[o + 3];
                    Log($"车端拒绝命令 0x{cmd:X2}（错误码 {err}：1=编号越界 2=数值非法）");
                    break;
                }

            case FrameType.Event:
                {
                    int n = 0;
                    while (o + 1 + n < b.Length && b[o + 1 + n] != 0) n++;
                    Log("[车] " + System.Text.Encoding.UTF8.GetString(b, o + 1, n));
                    break;
                }
        }
    }

    private void ParseTelemetry(byte[] b, int o)
    {
        // 载荷：SEQ(2) TS(4) MASK(2) COUNT(2) DATA(4N)
        int seq = Proto.GetU16(b, o);
        ushort mask = Proto.GetU16(b, o + 6);
        int cnt = Proto.GetU16(b, o + 8);
        if (cnt <= 0 || cnt > Proto.MAX_CHANNELS) return;

        var ch = new float[cnt];
        for (int i = 0; i < cnt; i++) ch[i] = Proto.GetF32(b, o + 10 + i * 4);

        bool first = _frames == 0;
        _frames++;
        if (_lastSeq >= 0)
        {
            int expect = (_lastSeq + 1) & 0xFFFF;
            if (seq != expect)
            {
                int gap = (seq - expect) & 0xFFFF;
                if (gap < 1000) _lost += gap;
            }
        }
        _lastSeq = seq;
        if (first) BeginInvoke(new Action(() => SetLamp(true)));

        _store.Ensure(cnt);

        // ★ 横轴用 PC 本地时间，不用帧里的 TS —— 否则丢包会让波形压缩/拉伸
        _store.Append(ch, cnt, _sw.Elapsed.TotalSeconds, mask);
        _ = Proto.GetU32(b, o + 2);

        // 帧里通道数与本地名字表不符 ⇒ 通道名整体错位（不越界、不报错、数字还挺像）
        if (cnt != Defaults.ChannelNames.Length && cnt != _warnedCnt)
        {
            _warnedCnt = cnt;
            Log($"⚠⚠ 帧里 {cnt} 通道，本机名字表 {Defaults.ChannelNames.Length} 通道 —— **名字会整体错位**。"
              + "改 Channels.cs 的数组与车端对齐。");
            BeginInvoke(new Action(() =>
            {
                string w = $"⚠ 帧 {cnt} 通道 ≠ 名字表 {Defaults.ChannelNames.Length} 通道，通道名错位";
                if (_waves != null) foreach (var wp in _waves) if (wp != null) wp.WarnText = w;
            }));
        }
        else if (cnt == Defaults.ChannelNames.Length && _warnedCnt >= 0)
        {
            _warnedCnt = -1;
            BeginInvoke(new Action(() =>
            {
                if (_waves != null) foreach (var wp in _waves) if (wp != null) wp.WarnText = null;
            }));
        }

        _frameChCount = cnt;
    }

    /// <summary>通道数对不上时把红条贴回新建的波形上（改一下分图它就没了，必须补）</summary>
    private void UpdateChannelMatch()
    {
        if (_waves == null) return;
        string w = _warnedCnt >= 0
            ? $"⚠ 帧 {_warnedCnt} 通道 ≠ 名字表 {Defaults.ChannelNames.Length} 通道，通道名错位"
            : null;
        foreach (var wp in _waves) if (wp != null) wp.WarnText = w;
    }

    private void ParseParamMeta(byte[] b, int o)
    {
        var p = new ParamMeta
        {
            Id = b[o],
            Lo = Proto.GetF32(b, o + 1),
            Hi = Proto.GetF32(b, o + 5),
            Def = Proto.GetF32(b, o + 9),
            Step = Proto.GetF32(b, o + 13),
            Unit = Proto.GetStr(b, o + 17, 8),
            Name = Proto.GetStr(b, o + 25, 16),
        };
        string grpLabel = Proto.GetStr(b, o + 41, 32);
        int bar = grpLabel.IndexOf('|');
        if (bar > 0) { p.Group = grpLabel.Substring(0, bar); p.Label = grpLabel.Substring(bar + 1); }
        else { p.Group = "其他"; p.Label = grpLabel; }
        p.Value = p.Def;

        _tableBuf.Add(p);
        BeginInvoke(new Action(() => { _tableTimer.Stop(); _tableTimer.Start(); }));
    }

    private void OnParamChanged(ParamMeta p, float v)
    {
        SendCmd(Cmd.SetParam, p.Id, v);
        Log($"下发 {p.Label} = {v:0.###}{p.Unit}（ID {p.Id}）");
    }

    // ==================================================================
    // 录制 / 统计 / 日志
    // ==================================================================
    private void ToggleRecord()
    {
        if (_rec.IsRecording)
        {
            long rows = _rec.Stop();
            _btnRec.Text = "录制 CSV";
            _btnRec.BackColor = Theme.PanelAlt;
            Log($"CSV 已停止：{_rec.Path}（{rows} 行）");
            return;
        }

        var chans = new List<int>();
        for (int i = 0; i < Defaults.ChannelNames.Length; i++)
            if (_chkBoxes.TryGetValue(i, out var ck) && ck.Checked) chans.Add(i);
        if (chans.Count == 0) { Log("没有勾选任何通道，无法录制"); return; }

        using var dlg = new SaveFileDialog
        {
            Filter = "CSV|*.csv",
            FileName = $"kart_{DateTime.Now:yyyyMMdd_HHmmss}.csv",
        };
        if (dlg.ShowDialog(this) != DialogResult.OK) return;

        if (_rec.Start(dlg.FileName, 20, chans, _sw.Elapsed.TotalSeconds, out string err))
        {
            _btnRec.Text = "停止录制";
            _btnRec.BackColor = Theme.Err;
            Log($"CSV 开始：{dlg.FileName}（{chans.Count} 通道 / 20ms）");
        }
        else Log("CSV 启动失败：" + err);
    }

    private void UpdateStats()
    {
        double t = _sw.Elapsed.TotalSeconds;
        double dt = t - _tAtStat;
        long df = _frames - _framesAtStat;
        long bytes = _link?.BytesIn ?? 0;
        long db = bytes - _bytesAtStat;
        _crcBad = _link?.FramesBadCrc ?? 0;

        _framesAtStat = _frames;
        _bytesAtStat = bytes;
        _tAtStat = t;

        double fps = dt > 1e-6 ? df / dt : 0;
        double kbs = dt > 1e-6 ? db / dt / 1024.0 : 0;
        double lossPct = _frames + _lost > 0 ? 100.0 * _lost / (_frames + _lost) : 0;

        _stLink.Text = _link?.IsOpen == true ? _link.RemoteDesc : "未连接";
        _stLink.ForeColor = _link?.IsOpen == true ? Theme.Ok : Theme.TextDim;
        _stRate.Text = $"帧率 {fps:0.0} fps（{_frames} 帧）";
        _stLoss.Text = $"丢包 {_lost}（{lossPct:0.00}%）";
        _stBw.Text = $"带宽 {kbs:0.0} KB/s";
        _stCrc.Text = $"CRC 错 {_crcBad}";
    }

    private void Log(string s)
    {
        if (_log == null || _log.IsDisposed) return;
        if (_log.InvokeRequired) { try { BeginInvoke(new Action(() => Log(s))); } catch { } return; }
        _log.AppendText($"{DateTime.Now:HH:mm:ss}  {s}\r\n");
    }

    // ==================================================================
    // 自查用
    // ==================================================================
    /// <summary>布局自查（--shot-form 用）：把关键尺寸打出来，避免"看着像坏的"和"真坏了"分不清</summary>
    internal string DumpLayout()
    {
        var wp = _waves != null && _waves.Length > 0 ? _waves[0].Size.ToString() : "-";

        // 勾选框文字放不放得下：可用宽 = 控件宽 − 勾选标记（≈18 逻辑像素）
        int maxTxt = 0;
        foreach (var ck in _chkBoxes.Values)
        {
            int w = TextRenderer.MeasureText(ck.Text, ck.Font, new Size(int.MaxValue, int.MaxValue),
                                             TextFormatFlags.NoPadding).Width;
            if (w > maxTxt) maxTxt = w;
        }
        int avail = _chkBoxes.Count > 0
            ? _chkBoxes[0].Width - Dpi.Px(18) - Dpi.Px(4)   // 勾选标记 + 内部间隙
            : 0;

        return $"form={Size} client={ClientSize} scale={Dpi.Scale:0.00} | "
             + $"main: w={_mainSplit.Width} dist={_mainSplit.SplitterDistance} "
             + $"p1={_mainSplit.Panel1.Width} p2={_mainSplit.Panel2.Width} | "
             + $"right: w={_rightSplit.Width} dist={_rightSplit.SplitterDistance} "
             + $"p1={_rightSplit.Panel1.Width} p2={_rightSplit.Panel2.Width} | "
             + $"waves={_waves?.Length ?? 0} wave0={wp} param={_param.Size} "
             + $"valBoxes={_valLabels.Count} chk={_chkBoxes.Count} "
             + $"chkList={_chkHost.Width}x{_chkHost.Height} chkTxt={maxTxt}/{avail}{(maxTxt > avail ? " **裁字**" : "")} "
             + $"btnConn={_btnConn.Width}x{_btnConn.Height} "
             + $"textW={TextRenderer.MeasureText(_btnConn.Text, _btnConn.Font).Width} "
             + $"font={Font.SizeInPoints:0.0}pt";
    }

    /// <summary>
    /// 造一份假数据把界面灌满（--shot-demo 用）。**只用于界面自查**，不参与实际收帧路径。
    /// 理由：波形与参数面板是最容易"写完了却看不出对不对"的两块，没有车的时候也得能看见。
    /// </summary>
    internal void DemoFill()
    {
        var ch = new float[Defaults.ChannelNames.Length];
        _store.Ensure(ch.Length);       // ★ 必须先把环扩到 N，否则 Append 会把 count 夹成 0（什么都不进）
        for (int k = 0; k < 900; k++)
        {
            double t = k * 0.02;
            double sq = Math.Sin(2 * Math.PI * 0.25 * t) > 0 ? 1 : -1;
            ch[0] = (float)(8 * Math.Sin(2 * Math.PI * 0.5 * t));            // 舵机目标
            ch[1] = (float)(8 * Math.Sin(2 * Math.PI * 0.5 * t - 0.18));     // 舵机实际（滞后）
            ch[2] = (float)(1200 * Math.Sin(2 * Math.PI * 0.5 * t - 0.3));
            ch[3] = (float)(2048 + 300 * Math.Sin(t));
            ch[4] = (float)(20 * sq);                                        // L 目标（方波阶跃）
            ch[5] = (float)(20 * sq * (1 - Math.Exp(-t * 3) * 0.6) + 0.5 * Math.Sin(t * 9));
            ch[6] = ch[4] * 0.02f;                                           // L 输出（归一化）
            ch[7] = ch[4] * 0.01f;
            ch[8] = ch[4];                                                   // R 目标
            ch[9] = (float)(ch[4] * 0.94 + 0.4 * Math.Sin(t * 7));
            ch[10] = ch[9] * 0.02f;
            ch[11] = ch[9] * 0.01f;
            ch[12] = (float)(992 + 120 * Math.Sin(2 * Math.PI * 0.5 * t));
            ch[13] = (float)(992 + 200 * Math.Sin(2 * Math.PI * 0.25 * t));
            ch[14] = (float)(8 * Math.Sin(2 * Math.PI * 0.5 * t));
            ch[15] = (float)(sq * 0.6);
            ch[16] = 2; ch[17] = 1;
            ch[18] = 1; ch[19] = (float)(3 + 2 * Math.Abs(Math.Sin(t)));
            ch[20] = 1; ch[21] = 1;
            ch[23] = (float)(30 * Math.Sin(2 * Math.PI * 0.1 * t));
            ch[24] = 3;
            _store.Append(ch, ch.Length, t, 0xFFFF);
        }

        // 参数表按车端 HOST_PARAM_TABLE 的 10 项造（顺序/量程一致，便于核对显示）
        var ps = new (byte Id, string Name, string Group, string Label, float Lo, float Hi, float Def, float Step, string Unit)[]
        {
            (0, "inv_K_l", "后轮L", "前馈 1/K",    0.000f,   0.500f,   0.1548f, 0.001f, "-"),
            (1, "u0_l",    "后轮L", "摩擦截距 u0", 0.000f,   0.300f,   0.0330f, 0.001f, "-"),
            (2, "k_v_l",   "后轮L", "比例 k_v",    0.000f,   2.000f,   0.1640f, 0.005f, "-"),
            (3, "k_z_l",   "后轮L", "积分 k_z",    0.000f,  20.000f,   2.3200f, 0.050f, "-"),
            (4, "inv_K_r", "后轮R", "前馈 1/K",    0.000f,   0.500f,   0.1548f, 0.001f, "-"),
            (5, "u0_r",    "后轮R", "摩擦截距 u0", 0.000f,   0.300f,   0.0330f, 0.001f, "-"),
            (6, "k_v_r",   "后轮R", "比例 k_v",    0.000f,   2.000f,   0.1640f, 0.005f, "-"),
            (7, "k_z_r",   "后轮R", "积分 k_z",    0.000f,  20.000f,   2.3200f, 0.050f, "-"),
            (8, "kp_s",    "舵机",  "比例 Kp",     0.000f, 1000.000f, 500.000f, 1.000f, "-"),
            (9, "u0_s",    "舵机",  "摩擦前馈 U0", 0.000f, 10000.000f,1900.000f,10.000f,"-"),
        };
        var list = new List<ParamMeta>();
        foreach (var p in ps)
        {
            list.Add(new ParamMeta
            {
                Id = p.Id, Name = p.Name, Group = p.Group, Label = p.Label,
                Lo = p.Lo, Hi = p.Hi, Def = p.Def, Step = p.Step, Unit = p.Unit, Value = p.Def,
            });
        }
        _param.MergeTable(list);
        UpdateValueLabels();
    }

    protected override void OnFormClosing(FormClosingEventArgs e)
    {
        try { if (_rec.IsRecording) _rec.Stop(); } catch { }
        _uiTimer?.Stop();
        _statTimer?.Stop();
        _tableTimer?.Stop();
        _hbTimer?.Stop();
        _link?.Dispose();
        base.OnFormClosing(e);
    }
}
