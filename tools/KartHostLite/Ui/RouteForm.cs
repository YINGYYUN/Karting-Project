using KartHost.Ui;

namespace KartHost;

/// <summary>
/// 路表面板（只做"灌表 / 读回 / 选槽"这三件本地动作，不参与驾驶）。
///
/// <para><b>它管的是</b>：把 PC 上的 <c>.bin</c> 镜像通过 <b>DAP 虚拟串口</b>写进车里的某个 Flash 槽，
/// 或者把车上的表读回来。协议见 <see cref="RouteLink"/>（车端 <c>route_link.c</c>）。</para>
///
/// <para><b>它不管的是</b>：WiFi 遥测那条链路（那是 <see cref="MainForm"/> 的事）。
/// 两条链路**互不干扰**，可以同时开 —— 串口灌表时 WiFi 波形照常。</para>
///
/// <para>⚠ <b>槽的纪律</b>：槽 0 是手推录制的家，下拉框里就没有 0。</para>
///
/// <para>⚠ <b>车端写盘靠主循环</b>：COMMIT 之后车端一页一页写 Flash（约几秒）。
/// 车端的落点**只有一个：参数页（主菜单 → Param）** —— 停在别的页面时收不到帧、也推进不了写盘
/// （数据不丢，回到参数页继续；但上位机会先超时）。</para>
/// </summary>
public sealed class RouteForm : Form
{
    private readonly RouteLink _rl = new();
    private byte[] _img;

    // 控件
    private ComboBox _cbPort;
    private NumericUpDown _numBaud;
    private Button _btnOpen, _btnPing, _btnDir;
    private NumericUpDown _numSlot;
    private TextBox _txtPath;
    private Button _btnBrowse, _btnMakeRing, _btnMakeLine;
    private Button _btnLoad, _btnRead, _btnCheck;
    private ProgressBar _bar;
    private Label _lblState, _lblImage;
    private ListBox _lstDir;
    private RichTextBox _log;

    public RouteForm()
    {
        AutoScaleMode = AutoScaleMode.None;
        CreateHandle();
        Dpi.Init(DeviceDpi);

        Text = "路表 · 串口灌表（DAP 虚拟串口）";
        Width = Math.Max(Dpi.Px(720), Math.Min(Dpi.Px(760), Screen.FromControl(this).WorkingArea.Width - Dpi.Px(60)));
        Height = Dpi.Px(520);
        StartPosition = FormStartPosition.CenterParent;
        BackColor = Theme.Bg;
        ForeColor = Theme.Text;
        Font = new Font("微软雅黑", 9f);

        Build();
        Theme.Apply(this);
        RefreshPorts();
        Log("就绪。★ 车端必须停在【参数页】（主菜单 → Param），否则收不到帧、也写不完 Flash。");
        Log("   停好后点「打开串口」→「握手 PING」→ 选镜像 → 灌表。");
    }

    // ---------------------------------------------------------------- 版面
    private void Build()
    {
        int y = 10;
        const int LX = 12, FX = 76;

        // ---- 串口行 ----
        AddLabel("串口", LX, y + 4);
        _cbPort = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Location = new Point(Dpi.Px(FX), Dpi.Px(y)), Width = Dpi.Px(110) };
        Controls.Add(_cbPort);

        AddLabel("波特率", FX + 118, y + 4);
        _numBaud = new NumericUpDown
        {
            Location = new Point(Dpi.Px(FX + 172), Dpi.Px(y)),
            Width = Dpi.Px(80),
            Minimum = 9600, Maximum = 1000000, Increment = 9600, Value = 115200,
            BorderStyle = BorderStyle.FixedSingle,
        };
        Controls.Add(_numBaud);

        _btnOpen = MakeBtn("打开串口", FX + 260, y, 88);
        _btnOpen.Click += (_, __) => ToggleOpen();
        _btnPing = MakeBtn("握手 PING", FX + 354, y, 92);
        _btnPing.Click += (_, __) => DoPing();
        _btnDir = MakeBtn("刷新槽位", FX + 452, y, 88);
        _btnDir.Click += (_, __) => DoDir();

        y += 36;

        // ---- 镜像文件行 ----
        AddLabel("镜像", LX, y + 4);
        _txtPath = new TextBox { Location = new Point(Dpi.Px(FX), Dpi.Px(y)), Width = Dpi.Px(400), BorderStyle = BorderStyle.FixedSingle };
        Controls.Add(_txtPath);
        _btnBrowse = MakeBtn("选择…", FX + 408, y, 70);
        _btnBrowse.Click += (_, __) => BrowseImage();

        y += 34;
        AddLabel("目标槽", LX, y + 4);
        _numSlot = new NumericUpDown
        {
            Location = new Point(Dpi.Px(FX), Dpi.Px(y)),
            Width = Dpi.Px(60),
            Minimum = 1, Maximum = 6, Value = 1,
            BorderStyle = BorderStyle.FixedSingle,
        };
        Controls.Add(_numSlot);
        AddLabel("（槽 0 是手推录制的家，不让写）", FX + 70, y + 4);

        _btnCheck = MakeBtn("校验镜像", FX + 250, y, 88);
        _btnCheck.Click += (_, __) => RecheckImage();
        _btnRead = MakeBtn("从车读回", FX + 344, y, 88);
        _btnRead.Click += (_, __) => DoRead();

        y += 40;

        // ---- 生成测试表 ----
        AddLabel("造测试表", LX, y + 4);
        _btnMakeRing = MakeBtn("圆环（半径 5 m，点距 50 mm）", FX, y, 220);
        _btnMakeRing.Click += (_, __) => MakeTest(ring: true);
        _btnMakeLine = MakeBtn("直线（长 10 m）", FX + 226, y, 120);
        _btnMakeLine.Click += (_, __) => MakeTest(ring: false);

        y += 40;

        // ---- 灌表 ----
        _btnLoad = MakeBtn("▶ 灌进车端 Flash", LX, y, 180);
        _btnLoad.Height = Dpi.Px(32);
        _btnLoad.Font = new Font("微软雅黑", 9.5f, FontStyle.Bold);
        _btnLoad.Click += (_, __) => DoLoad();

        _bar = new ProgressBar { Location = new Point(Dpi.Px(LX + 190), Dpi.Px(y + 4)), Width = Dpi.Px(280), Height = Dpi.Px(24) };
        Controls.Add(_bar);

        _lblState = new Label
        {
            Location = new Point(Dpi.Px(LX + 480), Dpi.Px(y + 6)),
            Size = Dpi.Sz(230, 20),
            ForeColor = Theme.TextDim,
        };
        Controls.Add(_lblState);

        y += 40;

        _lblImage = new Label
        {
            Text = "（没有镜像）",
            Location = new Point(Dpi.Px(LX), Dpi.Px(y)),
            Size = Dpi.Sz(700, 20),
            ForeColor = Theme.TextDim,
        };
        Controls.Add(_lblImage);

        y += 26;

        // ---- 槽位一览 ----
        Controls.Add(new Label
        {
            Text = "车端槽位",
            Location = new Point(Dpi.Px(LX), Dpi.Px(y)),
            Size = Dpi.Sz(80, 18),
            ForeColor = Theme.TextDim,
        });
        y += 20;

        _lstDir = new ListBox
        {
            Location = new Point(Dpi.Px(LX), Dpi.Px(y)),
            Size = Dpi.Sz(700, 110),
            BackColor = Theme.Panel,
            ForeColor = Theme.Text,
            BorderStyle = BorderStyle.FixedSingle,
            Font = new Font("Consolas", 8.5f),
        };
        Controls.Add(_lstDir);

        y += 118;

        // ---- 日志 ----
        _log = new RichTextBox
        {
            Location = new Point(Dpi.Px(LX), Dpi.Px(y)),
            Size = Dpi.Sz(700, 120),
            BackColor = Theme.Bg,
            ForeColor = Theme.TextDim,
            BorderStyle = BorderStyle.None,
            ReadOnly = true,
            WordWrap = false,
            Font = new Font("Consolas", 8.5f),
            Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Bottom,
        };
        Controls.Add(_log);

        _lblState.Text = "未连接";
    }

    private Label AddLabel(string text, int x, int y)
    {
        var lb = new Label { Text = text, Location = new Point(Dpi.Px(x), Dpi.Px(y)), AutoSize = true, ForeColor = Theme.TextDim };
        Controls.Add(lb);
        return lb;
    }

    private Button MakeBtn(string text, int x, int y, int w)
    {
        var b = new Button { Text = text, Location = new Point(Dpi.Px(x), Dpi.Px(y)), Width = Dpi.Px(w), Height = Dpi.Px(26) };
        Controls.Add(b);
        return b;
    }

    // ---------------------------------------------------------------- 动作
    private void RefreshPorts()
    {
        _cbPort.Items.Clear();
        var ports = RouteLink.ListPorts();
        _cbPort.Items.AddRange(ports);
        if (ports.Length > 0) _cbPort.SelectedIndex = 0;
        if (ports.Length == 0) Log("没有找到任何串口。DAP 的虚拟串口要装了驱动、并且板子插上才会出现。");
    }

    private void ToggleOpen()
    {
        if (_rl.IsOpen)
        {
            _rl.Close();
            _btnOpen.Text = "打开串口";
            _lblState.Text = "未连接";
            Log("串口已关闭");
            return;
        }

        if (_cbPort.SelectedItem == null) { Log("先选一个串口"); return; }
        try
        {
            _rl.Open((string)_cbPort.SelectedItem, (int)_numBaud.Value);
            _btnOpen.Text = "关闭串口";
            _lblState.Text = _rl.Describe;
            Log("已打开 " + _rl.Describe);
        }
        catch (Exception ex) { Log("打开失败：" + ex.Message); }
    }

    private void DoPing()
    {
        if (!RequireOpen()) return;
        try { Log(_rl.Ping()); }
        catch (Exception ex) { Log("握手失败：" + ex.Message); }
    }

    private void DoDir()
    {
        if (!RequireOpen()) return;
        try
        {
            var list = _rl.Dir(out int active, out _);
            _lstDir.Items.Clear();
            foreach (var s in list) _lstDir.Items.Add(s.Text);
            _numSlot.Maximum = Math.Max(1, _rl.SlotCount - 1);
            Log($"槽位一览：当前槽 = {active}");
        }
        catch (Exception ex) { Log("读槽位失败：" + ex.Message); }
    }

    private void BrowseImage()
    {
        using var dlg = new OpenFileDialog
        {
            Title = "选择车端路表镜像（route_car.bin）",
            Filter = "车端路表镜像 (*.bin)|*.bin|所有文件 (*.*)|*.*",
        };
        if (dlg.ShowDialog(this) != DialogResult.OK) return;
        _txtPath.Text = dlg.FileName;
        LoadImage(dlg.FileName);
    }

    private void LoadImage(string path)
    {
        _img = File.ReadAllBytes(path);
        _lblImage.Text = RouteImage.Describe(_img);
        _lblImage.ForeColor = RouteLink.VerifyImage(_img) == null ? Theme.Ok : Theme.Err;
        Log("已加载镜像：" + RouteImage.Describe(_img));
    }

    private void RecheckImage()
    {
        if (_img == null) { Log("还没选镜像"); return; }
        string bad = RouteLink.VerifyImage(_img);
        Log(bad == null ? "镜像校验：通过" : "镜像校验：**不过** —— " + bad);
        _lblImage.ForeColor = bad == null ? Theme.Ok : Theme.Err;
    }

    private void MakeTest(bool ring)
    {
        _img = ring ? RouteImage.BuildRing(5.0, 0.05, out _) : RouteImage.BuildLine(10.0, 0.05, out _);
        string def = ring ? "route_ring.bin" : "route_line.bin";
        using var dlg = new SaveFileDialog { Title = "测试镜像存到哪", Filter = "车端路表镜像 (*.bin)|*.bin", FileName = def };
        if (dlg.ShowDialog(this) == DialogResult.OK)
        {
            if (RouteImage.Save(dlg.FileName, _img, out string err))
            {
                _txtPath.Text = dlg.FileName;
                Log($"已生成{(ring ? "圆环" : "直线")}测试镜像 -> {dlg.FileName}");
            }
            else Log("写文件失败：" + err);
        }
        _lblImage.Text = RouteImage.Describe(_img);
        _lblImage.ForeColor = Theme.Ok;
    }

    private void DoLoad()
    {
        if (!RequireOpen()) return;
        if (_img == null && string.IsNullOrWhiteSpace(_txtPath.Text)) { Log("先选一个镜像，或点「造测试表」生成一张"); return; }
        if (_img == null) { try { LoadImage(_txtPath.Text); } catch (Exception ex) { Log("读镜像失败：" + ex.Message); return; } }

        string bad = RouteLink.VerifyImage(_img);
        if (bad != null) { Log("镜像不自洽，没发：" + bad); return; }

        _btnLoad.Enabled = false;
        _bar.Value = 0;
        try
        {
            Log($"开始灌表：槽 {(int)_numSlot.Value}，{RouteImage.Describe(_img)}");
            string msg = _rl.Download((int)_numSlot.Value, _img, (frac, note) =>
            {
                _bar.Value = (int)Math.Round(Math.Clamp(frac, 0, 1) * _bar.Maximum);
                _lblState.Text = note;
                Application.DoEvents();          // 面板上要看得见进度，主窗那边不受影响
            });
            Log("★ " + msg);
            DoDir();
        }
        catch (Exception ex) { Log("灌表失败：" + ex.Message); }
        finally
        {
            _btnLoad.Enabled = true;
            _lblState.Text = _rl.IsOpen ? _rl.Describe : "未连接";
        }
    }

    private void DoRead()
    {
        if (!RequireOpen()) return;
        try
        {
            int slot = (int)_numSlot.Value;
            Log($"从槽 {slot} 读回…");
            var img = _rl.ReadSlot(slot, (frac, note) =>
            {
                _bar.Value = (int)Math.Round(Math.Clamp(frac, 0, 1) * _bar.Maximum);
                _lblState.Text = note;
                Application.DoEvents();
            });
            Log("读回：" + RouteImage.Describe(img));

            using var dlg = new SaveFileDialog
            {
                Title = "读回来的镜像存成文件",
                Filter = "车端路表镜像 (*.bin)|*.bin",
                FileName = $"route_slot{slot}_{DateTime.Now:HHmmss}.bin",
            };
            if (dlg.ShowDialog(this) == DialogResult.OK)
            {
                if (RouteImage.Save(dlg.FileName, img, out string err)) Log("已保存 -> " + dlg.FileName);
                else Log("写文件失败：" + err);
            }
        }
        catch (Exception ex) { Log("读回失败：" + ex.Message); }
        finally { _lblState.Text = _rl.IsOpen ? _rl.Describe : "未连接"; }
    }

    private bool RequireOpen()
    {
        if (_rl.IsOpen) return true;
        Log("串口没打开");
        return false;
    }

    private void Log(string s)
    {
        if (_log == null || _log.IsDisposed) return;
        _log.AppendText($"{DateTime.Now:HH:mm:ss}  {s}\r\n");
    }

    protected override void OnFormClosing(FormClosingEventArgs e)
    {
        try { _rl.Dispose(); } catch { }
        base.OnFormClosing(e);
    }
}
