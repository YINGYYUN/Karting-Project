using KartHost.Ui;

namespace KartHost;

/// <summary>
/// 路表管理器（灌表 / 读回 / 校验 / 预览 / 选槽，不参与驾驶）。
///
/// <para><b>它管的是</b>：把 PC 上的 <c>.bin</c> 镜像通过 <b>DAP 虚拟串口</b>写进车里的某个 Flash 槽，
/// 把车上的表读回来并**当场校验**，以及把某槽设为当前表。协议见 <see cref="RouteLink"/>
/// （车端 <c>route_link.c</c>，0x03~0x09）。</para>
///
/// <para><b>它不管的是</b>：WiFi 遥测那条链路（那是 <see cref="MainForm"/> 的事）。
/// 两条链路**互不干扰**，可以同时开 —— 串口灌表时 WiFi 波形照常。</para>
///
/// <para>⚠ <b>槽的纪律</b>：槽 0 是手推录制的家，下拉框/复制目标里就没有 0。</para>
///
/// <para>⚠ <b>车端写盘靠主循环</b>：COMMIT 之后车端一页一页写 Flash（约几秒）。
/// 车端的落点**只有一个：参数页（主菜单 → Param）** —— 停在别的页面时收不到帧、也推进不了写盘
/// （数据不丢，回到参数页继续；但上位机会先超时）。</para>
///
/// <para>2026-10-24 扩成管理器：槽位从"一段文字"改成**表**，读回后**自动校验并画轨迹**，
/// 新增「设为当前槽」与「槽间复制」。</para>
/// </summary>
public sealed class RouteForm : Form
{
    private readonly RouteLink _rl = new();
    private byte[] _img;            // 本地镜像（准备灌进车的）
    private byte[] _readImg;        // 最近一次从车读回的镜像
    private int    _readSlot = -1;  // _readImg 来自哪个槽

    // 控件
    private ComboBox _cbPort;
    private NumericUpDown _numBaud, _numSlot, _numCopyTo;
    private Button _btnOpen, _btnPing, _btnDir;
    private TextBox _txtPath;
    private Button _btnBrowse, _btnMakeRing, _btnMakeLine;
    private Button _btnLoad, _btnRead, _btnCheck;
    private Button _btnSelActive, _btnCopy, _btnSaveRead, _btnReport;
    private RouteAudit.Result _audit;      // 最近一次体检结果（导出报告用）
    private ProgressBar _bar;
    private Label _lblState, _lblImage, _lblRead;
    private ListView _lvSlots;
    private PlotPanel _plot;
    private RichTextBox _log;

    private List<RouteLink.SlotInfo> _dir = new();
    private int _activeSlot = -1;

    public RouteForm()
    {
        AutoScaleMode = AutoScaleMode.None;
        CreateHandle();
        Dpi.Init(DeviceDpi);

        Text = "路表管理器 · 串口灌表 / 读回校验 / 预览 / 选槽";
        Width = Dpi.Px(940);
        Height = Dpi.Px(720);
        StartPosition = FormStartPosition.CenterParent;
        BackColor = Theme.Bg;
        ForeColor = Theme.Text;
        Font = new Font("微软雅黑", 9f);

        Build();
        Theme.Apply(this);
        RefreshPorts();
        Log("就绪。★ 车端必须停在【参数页】（主菜单 → Param），否则收不到帧、也写不完 Flash。");
        Log("   流程：打开串口 → 握手 PING → 刷新槽位 →（选镜像灌表 / 选中槽读回并校验 / 设为当前 / 槽间复制）。");
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

        _lblState = new Label
        {
            Text = "未连接",
            Location = new Point(Dpi.Px(FX + 548), Dpi.Px(y + 4)),
            Size = Dpi.Sz(240, 20),
            ForeColor = Theme.TextDim,
        };
        Controls.Add(_lblState);

        y += 36;

        // ---- 镜像文件行 ----
        AddLabel("镜像", LX, y + 4);
        _txtPath = new TextBox { Location = new Point(Dpi.Px(FX), Dpi.Px(y)), Width = Dpi.Px(520), BorderStyle = BorderStyle.FixedSingle };
        Controls.Add(_txtPath);
        _btnBrowse = MakeBtn("选择…", FX + 528, y, 70);
        _btnBrowse.Click += (_, __) => BrowseImage();
        _btnCheck = MakeBtn("校验镜像", FX + 606, y, 88);
        _btnCheck.Click += (_, __) => RecheckImage();

        y += 36;

        // ---- 灌表行 ----
        AddLabel("灌到槽", LX, y + 4);
        _numSlot = new NumericUpDown
        {
            Location = new Point(Dpi.Px(FX), Dpi.Px(y)),
            Width = Dpi.Px(56),
            Minimum = 1, Maximum = 5, Value = 1,
            BorderStyle = BorderStyle.FixedSingle,
        };
        Controls.Add(_numSlot);
        AddLabel("（0 = 手推录制的家，不让写）", FX + 62, y + 4);

        _btnLoad = MakeBtn("▶ 灌进车端 Flash", FX + 250, y, 170);
        _btnLoad.Font = new Font("微软雅黑", 9.5f, FontStyle.Bold);
        _btnLoad.Click += (_, __) => DoLoad();

        _bar = new ProgressBar { Location = new Point(Dpi.Px(FX + 428), Dpi.Px(y + 2)), Width = Dpi.Px(200), Height = Dpi.Px(22) };
        Controls.Add(_bar);

        y += 32;
        _lblImage = new Label
        {
            Text = "（没有镜像）",
            Location = new Point(Dpi.Px(LX), Dpi.Px(y)),
            Size = Dpi.Sz(890, 20),
            ForeColor = Theme.TextDim,
        };
        Controls.Add(_lblImage);

        y += 28;

        // ---- 槽位表 ----
        Controls.Add(new Label
        {
            Text = "车端槽位（点一行选中；双击 = 读回）",
            Location = new Point(Dpi.Px(LX), Dpi.Px(y)),
            Size = Dpi.Sz(300, 18),
            ForeColor = Theme.TextDim,
        });

        Controls.Add(new Label
        {
            Text = "轨迹预览（绿 = 起点，红 = 终点）",
            Location = new Point(Dpi.Px(LX + 548), Dpi.Px(y)),
            Size = Dpi.Sz(320, 18),
            ForeColor = Theme.TextDim,
        });

        y += 20;

        _lvSlots = new ListView
        {
            Location = new Point(Dpi.Px(LX), Dpi.Px(y)),
            Size = Dpi.Sz(540, 176),
            View = View.Details,
            FullRowSelect = true,
            GridLines = true,
            MultiSelect = false,
            HideSelection = false,
            BackColor = Theme.Panel,
            ForeColor = Theme.Text,
            BorderStyle = BorderStyle.FixedSingle,
            Font = new Font("Consolas", 8.5f),
        };
        _lvSlots.Columns.Add("槽", Dpi.Px(38), HorizontalAlignment.Center);
        _lvSlots.Columns.Add("有效", Dpi.Px(44), HorizontalAlignment.Center);
        _lvSlots.Columns.Add("点数", Dpi.Px(58), HorizontalAlignment.Right);
        _lvSlots.Columns.Add("里程 m", Dpi.Px(74), HorizontalAlignment.Right);
        _lvSlots.Columns.Add("ds mm", Dpi.Px(62), HorizontalAlignment.Right);
        _lvSlots.Columns.Add("记录 CRC", Dpi.Px(92), HorizontalAlignment.Right);
        _lvSlots.Columns.Add("当前", Dpi.Px(46), HorizontalAlignment.Center);
        _lvSlots.SelectedIndexChanged += (_, __) => OnSlotPicked();
        _lvSlots.DoubleClick += (_, __) => DoRead();
        Controls.Add(_lvSlots);

        _plot = new PlotPanel
        {
            Location = new Point(Dpi.Px(LX + 548), Dpi.Px(y)),
            Size = Dpi.Sz(342, 176),
        };
        Controls.Add(_plot);

        y += 184;

        // ---- 读回 / 选槽 / 复制 ----
        _btnRead = MakeBtn("从选中槽读回并校验", LX, y, 168);
        _btnRead.Click += (_, __) => DoRead();

        _btnSelActive = MakeBtn("设为当前槽", LX + 176, y, 110);
        _btnSelActive.Click += (_, __) => DoSelectActive();

        AddLabel("复制到槽", LX + 296, y + 4);
        _numCopyTo = new NumericUpDown
        {
            Location = new Point(Dpi.Px(LX + 368), Dpi.Px(y)),
            Width = Dpi.Px(52),
            Minimum = 1, Maximum = 5, Value = 2,
            BorderStyle = BorderStyle.FixedSingle,
        };
        Controls.Add(_numCopyTo);
        _btnCopy = MakeBtn("把刚读回的复制过去", LX + 428, y, 168);
        _btnCopy.Click += (_, __) => DoCopy();

        _btnSaveRead = MakeBtn("保存读回的镜像…", LX + 604, y, 140);
        _btnSaveRead.Click += (_, __) => SaveReadImage();

        _btnReport = MakeBtn("导出体检报告…", LX + 752, y, 168);
        _btnReport.Click += (_, __) => ExportReport();
        _btnReport.Enabled = false;

        y += 32;
        _lblRead = new Label
        {
            Text = "（还没读回）",
            Location = new Point(Dpi.Px(LX), Dpi.Px(y)),
            Size = Dpi.Sz(890, 20),
            ForeColor = Theme.TextDim,
        };
        Controls.Add(_lblRead);

        y += 26;

        // ---- 日志 + 最底部的"调试用测试表" ----
        // 注意坐标：ClientSize.Height / Width 已经是**设备像素**，这里一律按设备坐标摆，
        // 不能再过 Dpi.Px()（那会把已经缩放过的高度再乘一次，控件直接被推到窗口外）。
        int logTop = Dpi.Px(y);
        int botRowH = Dpi.Px(28) + Dpi.Px(8);
        int logH = Math.Max(Dpi.Px(120), ClientSize.Height - logTop - botRowH - Dpi.Px(6));

        _log = new RichTextBox
        {
            Location = new Point(Dpi.Px(LX), logTop),
            Size = new Size(ClientSize.Width - Dpi.Px(2 * LX), logH),
            BackColor = Theme.Bg,
            ForeColor = Theme.TextDim,
            BorderStyle = BorderStyle.None,
            ReadOnly = true,
            WordWrap = false,
            Font = new Font("Consolas", 8.5f),
            Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Bottom,
        };
        Controls.Add(_log);

        int byDev = logTop + logH + Dpi.Px(4);           // 底部条的设备 y

        Controls.Add(new Label
        {
            Text = "调试用测试表（正式电子表应由工具/脚本生成；这里只用于自测通路）",
            Location = new Point(Dpi.Px(LX), byDev + Dpi.Px(5)),
            AutoSize = true,
            ForeColor = Theme.TextDim,
            Font = new Font("微软雅黑", 8.5f),
        });

        _btnMakeRing = new Button
        {
            Text = "圆环 5 m",
            Location = new Point(ClientSize.Width - Dpi.Px(206), byDev),
            Size = new Size(Dpi.Px(92), Dpi.Px(24)),
            Font = new Font("微软雅黑", 8.5f),
            ForeColor = Theme.TextDim,
        };
        _btnMakeRing.Click += (_, __) => MakeTest(ring: true);
        Controls.Add(_btnMakeRing);

        _btnMakeLine = new Button
        {
            Text = "直线 10 m",
            Location = new Point(ClientSize.Width - Dpi.Px(108), byDev),
            Size = new Size(Dpi.Px(92), Dpi.Px(24)),
            Font = new Font("微软雅黑", 8.5f),
            ForeColor = Theme.TextDim,
        };
        _btnMakeLine.Click += (_, __) => MakeTest(ring: false);
        Controls.Add(_btnMakeLine);

        UpdateReadButtons();
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

    // ---------------------------------------------------------------- 槽位表
    private void FillSlots()
    {
        _lvSlots.BeginUpdate();
        try
        {
            _lvSlots.Items.Clear();
            foreach (var s in _dir)
            {
                var it = new ListViewItem(s.Slot.ToString()) { Tag = s };
                it.SubItems.Add(s.Valid ? "✓" : "—");
                it.SubItems.Add(s.Valid ? s.Count.ToString() : "");
                it.SubItems.Add(s.Valid ? (s.LapMm / 1000.0).ToString("0.000") : "");
                it.SubItems.Add(s.Valid ? s.DsMm.ToString() : "");
                it.SubItems.Add(s.Valid ? $"0x{s.CrcR:X8}" : "");
                it.SubItems.Add(s.Active ? "★" : "");

                if (!s.Valid) it.ForeColor = Theme.TextDim;
                else if (s.Active) it.ForeColor = Theme.Accent;

                _lvSlots.Items.Add(it);
            }
        }
        finally { _lvSlots.EndUpdate(); }
    }

    private RouteLink.SlotInfo PickedSlot()
    {
        if (_lvSlots.SelectedItems.Count == 0) return null;
        return _lvSlots.SelectedItems[0].Tag as RouteLink.SlotInfo;
    }

    private void OnSlotPicked()
    {
        var s = PickedSlot();
        if (s == null) return;
        _numSlot.Value = Math.Min(_numSlot.Maximum, Math.Max(_numSlot.Minimum, s.Slot));

        // 预览：把"这一槽上一次读回的内容"和当前选中槽对上时才画
        if (s.Valid && _readImg != null && _readSlot == s.Slot) ShowReadImageOnPlot();
    }

    private void UpdateReadButtons()
    {
        bool has = _readImg != null && _readSlot >= 0;
        _btnCopy.Enabled = has;
        _btnSaveRead.Enabled = has;

        var s = PickedSlot();
        bool pickValid = s != null && s.Valid;
        _btnSelActive.Enabled = pickValid;
        _btnRead.Enabled = _rl.IsOpen && s != null && s.Valid;
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
            UpdateReadButtons();
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
        UpdateReadButtons();
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
            _dir = _rl.Dir(out int active, out int slotCount);
            _activeSlot = active;
            FillSlots();

            int maxW = Math.Max(1, _rl.SlotCount - 1);
            _numSlot.Maximum = maxW;
            _numCopyTo.Maximum = maxW;
            Log($"槽位一览：共 {slotCount} 槽，当前槽 = {active}");

            int valid = 0;
            foreach (var s in _dir) if (s.Valid) valid++;
            Log($"  有效表 {valid} 张" + (valid == 0 ? "（空槽：先灌一张，或用「设为当前槽」前先读回确认）" : ""));
        }
        catch (Exception ex) { Log("读槽位失败：" + ex.Message); }
        finally { UpdateReadButtons(); }
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
        string bad = RouteLink.VerifyImage(_img);
        _lblImage.ForeColor = bad == null ? Theme.Ok : Theme.Err;
        if (bad != null) _lblImage.Text += "   ✗ " + bad;
        Log("已加载镜像：" + RouteImage.Describe(_img));

        // 本地镜像也画出来（读回的那张优先，避免混看）
        if (_readImg == null) ShowImageOnPlot(_img, "本地镜像");
    }

    private void RecheckImage()
    {
        if (_img == null) { Log("还没选镜像"); return; }
        string bad = RouteLink.VerifyImage(_img);
        Log(bad == null ? "镜像校验：通过" : "镜像校验：**不过** —— " + bad);
        _lblImage.ForeColor = bad == null ? Theme.Ok : Theme.Err;

        // 灌表**之前**也体检一遍：几何不能跑的表不该先进 Flash
        RunAudit(_img, "本地镜像（灌表前）");
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
        if (_readImg == null) ShowImageOnPlot(_img, "测试镜像");
    }

    private void DoLoad()
    {
        if (!RequireOpen()) return;
        if (_img == null && string.IsNullOrWhiteSpace(_txtPath.Text)) { Log("先选一个镜像（正式表请用工具生成；底部只有调试用的圆环/直线）"); return; }
        if (_img == null) { try { LoadImage(_txtPath.Text); } catch (Exception ex) { Log("读镜像失败：" + ex.Message); return; } }

        string bad = RouteLink.VerifyImage(_img);
        if (bad != null) { Log("镜像不自洽，没发：" + bad); return; }

        _btnLoad.Enabled = false;
        _bar.Value = 0;
        try
        {
            Log($"开始灌表：槽 {(int)_numSlot.Value}，{RouteImage.Describe(_img)}");
            string msg = _rl.Download((int)_numSlot.Value, _img, OnProgress);
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

    /// <summary>读回 + **当场校验** + 画轨迹。这是"防止灌了一张坏表还以为是好的"的核心。</summary>
    private void DoRead()
    {
        if (!RequireOpen()) return;

        var s = PickedSlot();
        if (s == null) { Log("先在表里选一个槽（或双击那一行）"); return; }
        if (!s.Valid) { Log($"槽 {s.Slot} 是空槽，没有可读的表"); return; }

        int slot = s.Slot;
        try
        {
            Log($"从槽 {slot} 读回…（{s.Count} 点 / {(s.LapMm / 1000.0):0.000} m）");
            var img = _rl.ReadSlot(slot, OnProgress);

            string desc = RouteImage.Describe(img);
            string bad = RouteLink.VerifyImage(img);
            var info = RouteLink.ImageInfo(img);

            _readImg = img;
            _readSlot = slot;

            // ---- 体检（几何）----
            RunAudit(img, $"槽 {slot}（读回）");

            if (bad == null)
            {
                _lblRead.Text = $"槽 {slot}：✓ CRC 通过 · {_audit.Summary}";
                _lblRead.ForeColor = _audit.Ok ? Theme.Ok : Theme.Err;
                Log("★ CRC 校验通过：" + desc);
            }
            else
            {
                _lblRead.Text = $"槽 {slot}：✗ CRC 不过（{bad}）";
                _lblRead.ForeColor = Theme.Err;
                Log("✗ CRC 校验**不过**：" + bad + "（这张表别直接复制/回放）");
            }

            // 与槽位表里报的元数据交叉核对 —— 不一致说明读回过程丢块或车端槽头写坏了
            if (info.Count != s.Count || info.DsMm != s.DsMm)
            {
                Log($"⚠ 交叉核对不一致：表里 {s.Count} 点/{s.DsMm} mm，读回 {info.Count} 点/{info.DsMm} mm");
            }
            else
            {
                Log($"  交叉核对一致（{info.Count} 点，{info.DsMm} mm，{(info.LapMm / 1000.0):0.000} m）");
            }

            ShowReadImageOnPlot();
        }
        catch (Exception ex) { Log("读回失败：" + ex.Message); }
        finally
        {
            _lblState.Text = _rl.IsOpen ? _rl.Describe : "未连接";
            UpdateReadButtons();
        }
    }

    /// <summary>把选中槽设为当前表（persist = 写进槽索引，重启后仍是它）。</summary>
    private void DoSelectActive()
    {
        if (!RequireOpen()) return;
        var s = PickedSlot();
        if (s == null) { Log("先在表里选一个槽"); return; }
        if (!s.Valid) { Log($"槽 {s.Slot} 是空槽，不能设为当前"); return; }

        try
        {
            Log($"把槽 {s.Slot} 设为当前表（persist + 立即载入 RAM）…");
            _rl.SlotSelect(s.Slot, persist: true, load: true);
            Log("★ 已切换");
            DoDir();
        }
        catch (Exception ex) { Log("设为当前槽失败：" + ex.Message); }
    }

    /// <summary>槽间复制：把"刚读回的那张"原样灌进另一个槽（依赖读回校验，避免把坏表复制开）。</summary>
    private void DoCopy()
    {
        if (!RequireOpen()) return;
        if (_readImg == null || _readSlot < 0) { Log("先「从选中槽读回并校验」，再复制"); return; }

        int to = (int)_numCopyTo.Value;
        if (to == _readSlot) { Log($"源和目标都是槽 {to}"); return; }
        if (to == 0) { Log("槽 0 是手推录制的家，不让写"); return; }

        string bad = RouteLink.VerifyImage(_readImg);
        if (bad != null)
        {
            Log("源镜像校验不过，拒绝复制：" + bad);
            return;
        }

        if (MessageBox.Show(this,
                $"把槽 {_readSlot} 的这张表（{RouteImage.Describe(_readImg)}）复制到槽 {to}？\r\n\r\n" +
                "会覆盖槽 " + to + " 原有的内容。",
                "槽间复制", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning) != DialogResult.OK) return;

        _bar.Value = 0;
        try
        {
            Log($"复制：槽 {_readSlot} → 槽 {to} …");
            string msg = _rl.Download(to, _readImg, OnProgress);
            Log("★ " + msg);
            DoDir();
        }
        catch (Exception ex) { Log("复制失败：" + ex.Message); }
        finally { _lblState.Text = _rl.IsOpen ? _rl.Describe : "未连接"; }
    }

    /// <summary>跑一次几何体检：日志逐条打"实测 vs 建议"，标题行汇总。</summary>
    private void RunAudit(byte[] img, string title)
    {
        _audit = RouteAudit.Run(img, title);
        _btnReport.Enabled = true;

        Log($"—— 体检：{title} ——");
        foreach (var row in _audit.Rows)
        {
            Log($"   {row.Lv} {row.Name}：{row.Found}");
            Log($"        建议 {row.Suggest}");
        }
        Log("   " + _audit.Summary);
    }

    /// <summary>把最近一次体检导出成 .txt（UTF-8 BOM，记事本/Excel 都能直接开）。</summary>
    private void ExportReport()
    {
        if (_audit == null) { Log("还没有体检结果：先「从选中槽读回并校验」或「校验镜像」"); return; }

        using var dlg = new SaveFileDialog
        {
            Title = "导出路表体检报告",
            Filter = "文本文件 (*.txt)|*.txt|所有文件 (*.*)|*.*",
            FileName = $"route_audit_{DateTime.Now:HHmmss}.txt",
        };
        if (dlg.ShowDialog(this) != DialogResult.OK) return;

        try
        {
            File.WriteAllText(dlg.FileName, _audit.Report, new System.Text.UTF8Encoding(true));
            Log("体检报告已导出 -> " + dlg.FileName);
        }
        catch (Exception ex) { Log("导出失败：" + ex.Message); }
    }
    private void SaveReadImage()
    {
        if (_readImg == null) return;
        using var dlg = new SaveFileDialog
        {
            Title = "读回来的镜像存成文件",
            Filter = "车端路表镜像 (*.bin)|*.bin",
            FileName = $"route_slot{_readSlot}_{DateTime.Now:HHmmss}.bin",
        };
        if (dlg.ShowDialog(this) != DialogResult.OK) return;
        if (RouteImage.Save(dlg.FileName, _readImg, out string err)) Log("已保存 -> " + dlg.FileName);
        else Log("写文件失败：" + err);
    }

    private void OnProgress(double frac, string note)
    {
        _bar.Value = (int)Math.Round(Math.Clamp(frac, 0, 1) * _bar.Maximum);
        _lblState.Text = note;
        Application.DoEvents();          // 面板上要看得见进度，主窗那边不受影响
    }

    private bool RequireOpen()
    {
        if (_rl.IsOpen) return true;
        Log("串口没打开");
        return false;
    }

    // ---------------------------------------------------------------- 预览
    /// <summary>只给 <c>--shot-route</c> 截图自查用：灌一张圆环 + 几行示例槽位，让表格和预览都有东西。</summary>
    internal void ShotLoadDemo()
    {
        _img = RouteImage.BuildRing(5.0, 0.05, out int cnt);
        _txtPath.Text = "（内部测试镜像，未落盘）";
        _lblImage.Text = RouteImage.Describe(_img);
        _lblImage.ForeColor = Theme.Ok;
        ShowImageOnPlot(_img, "圆环测试表");

        _numSlot.Maximum = 5; _numCopyTo.Maximum = 5;
        _dir = new List<RouteLink.SlotInfo>
        {
            new() { Slot = 0, Valid = true,  Active = false, Count = 2048, LapMm = 102000, DsMm = 50,  CrcR = 0x1234ABCDu },
            new() { Slot = 1, Valid = true,  Active = true,  Count = cnt,  LapMm = 31420,  DsMm = 50,  CrcR = 0xDEADBEEFu },
            new() { Slot = 2, Valid = false },
            new() { Slot = 3, Valid = true,  Active = false, Count = 382,  LapMm = 28454,  DsMm = 100, CrcR = 0x5A5AA5A5u },
            new() { Slot = 4, Valid = false },
        };
        FillSlots();
        if (_lvSlots.Items.Count > 1) _lvSlots.Items[1].Selected = true;

        RunAudit(_img, "槽 1（读回，示例）");
        _readImg = _img;
        _readSlot = 1;
        _lblRead.Text = $"槽 1：{RouteImage.Describe(_img)}   ✓ 校验通过（示例）";
        _lblRead.ForeColor = Theme.Ok;
        UpdateReadButtons();
        Log("—— 截图自查：已灌入示例槽位表与圆环预览 ——");
    }

    private void ShowReadImageOnPlot() => ShowImageOnPlot(_readImg, $"槽 {_readSlot}（读回）");

    private void ShowImageOnPlot(byte[] img, string tag)
    {
        var pts = ExtractPoints(img);
        _plot.SetData(pts.x, pts.y, $"{tag}：{pts.x.Length} 点");
    }

    /// <summary>从镜像里取 x/y（mm）。记录布局：x int32 @0, y int32 @4（见 ins_route.h）。</summary>
    private static (float[] x, float[] y) ExtractPoints(byte[] img)
    {
        if (img == null || img.Length < RouteImage.HeadBytes) return (Array.Empty<float>(), Array.Empty<float>());

        int n = (img.Length - RouteImage.HeadBytes) / RouteImage.RecBytes;
        if (n < 0) n = 0;

        var xs = new float[n];
        var ys = new float[n];
        for (int i = 0; i < n; i++)
        {
            int o = RouteImage.HeadBytes + i * RouteImage.RecBytes;
            xs[i] = BitConverter.ToInt32(img, o);
            ys[i] = BitConverter.ToInt32(img, o + 4);
        }
        return (xs, ys);
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

    // ================================================================ 预览面板
    /// <summary>把 x/y(m) 轨迹画成等比例图（自动取景、居中、不拉伸）。</summary>
    private sealed class PlotPanel : Panel
    {
        private float[] _x = Array.Empty<float>();
        private float[] _y = Array.Empty<float>();
        private string _cap = "";

        public PlotPanel()
        {
            SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint
                     | ControlStyles.UserPaint | ControlStyles.ResizeRedraw, true);
            BackColor = Theme.Panel;
            BorderStyle = BorderStyle.FixedSingle;
        }

        public void SetData(float[] x, float[] y, string caption)
        {
            _x = x ?? Array.Empty<float>();
            _y = y ?? Array.Empty<float>();
            _cap = caption ?? "";
            Invalidate();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);

            var g = e.Graphics;
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

            using var bg = new SolidBrush(Theme.Panel);
            g.FillRectangle(bg, ClientRectangle);

            int n = Math.Min(_x.Length, _y.Length);
            if (n < 2)
            {
                using var f0 = new Font("微软雅黑", 8.5f);
                using var b0 = new SolidBrush(Theme.TextDim);
                g.DrawString("（没有可画的轨迹 —— 先读回一个非空槽）", f0, b0, Dpi.Px(8), Dpi.Px(8));
                return;
            }

            float minX = _x[0], maxX = _x[0], minY = _y[0], maxY = _y[0];
            for (int i = 1; i < n; i++)
            {
                if (_x[i] < minX) minX = _x[i];
                if (_x[i] > maxX) maxX = _x[i];
                if (_y[i] < minY) minY = _y[i];
                if (_y[i] > maxY) maxY = _y[i];
            }

            float w = Math.Max(1f, maxX - minX);
            float h = Math.Max(1f, maxY - minY);

            int padL = Dpi.Px(10), padR = Dpi.Px(10), padT = Dpi.Px(26), padB = Dpi.Px(10);
            float availW = Math.Max(10, ClientSize.Width - padL - padR);
            float availH = Math.Max(10, ClientSize.Height - padT - padB);

            float s = Math.Min(availW / w, availH / h);          // 等比：不拉伸
            float offX = padL + (availW - w * s) * 0.5f;
            float offY = padT + (availH - h * s) * 0.5f;

            PointF Map(float px, float py) => new(offX + (px - minX) * s, offY + (maxY - py) * s);   // y 向上

            using var pen = new Pen(Theme.Accent, Dpi.Px(1.6f));
            var prev = Map(_x[0], _y[0]);
            for (int i = 1; i < n; i++)
            {
                var cur = Map(_x[i], _y[i]);
                g.DrawLine(pen, prev, cur);
                prev = cur;
            }

            // 起点绿 / 终点红
            using var bStart = new SolidBrush(Theme.Ok);
            using var bEnd = new SolidBrush(Theme.Err);
            var ps = Map(_x[0], _y[0]);
            var pe = Map(_x[n - 1], _y[n - 1]);
            float r = Dpi.Px(3.5f);
            g.FillEllipse(bStart, ps.X - r, ps.Y - r, r * 2, r * 2);
            g.FillEllipse(bEnd, pe.X - r, pe.Y - r, r * 2, r * 2);

            using var font = new Font("Consolas", 8f);
            using var br = new SolidBrush(Theme.TextDim);
            g.DrawString(_cap, font, br, Dpi.Px(6), Dpi.Px(6));

            string scale = $"范围 {w / 1000.0:0.00}×{h / 1000.0:0.00} m";
            var sz = g.MeasureString(scale, font);
            g.DrawString(scale, font, br, ClientSize.Width - sz.Width - Dpi.Px(6),
                         ClientSize.Height - sz.Height - Dpi.Px(4));   // 放到底右，别和图注打架
        }
    }
}
