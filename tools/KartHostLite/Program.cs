using System.Diagnostics;

namespace KartHost;

internal static class Program
{
    [STAThread]
    private static void Main(string[] args)
    {
        // ★ 无界面模式必须**排在 WinForms 初始化之前**：
        //   Application.EnableVisualStyles() 会建侧激活上下文（comctl32 清单），
        //   在"单文件自包含 + 自解压"的 exe 里这一步可能抛 FileLoadException，
        //   把 --selftest 这种纯协议自检一起带走（2026-10-22 实测：
        //   selftest 还没跑就崩在 Program.Main 第 10 行）。
        //   自检不依赖任何视觉样式，先跑它是零代价的正确顺序。

        // --selftest：无界面跑一遍协议层自检（CRC / 切帧 / 组帧 / 通道表对齐）
        //   没有硬件也能验证；有硬件时直接开界面收帧。
        if (args.Length > 0 && args[0] == "--selftest")
        {
            Environment.ExitCode = SelfTest.Run(args.Length > 1 ? args[1] : null);
            return;
        }

        // DPI 必须在任何窗口创建之前设好（--shot-form 也会建窗口）
        // 控制台输出统一 UTF-8：--audit / --sim 的报告要能直接重定向、也能被脚本读
        try { Console.OutputEncoding = new System.Text.UTF8Encoding(false); } catch { }
        Application.SetHighDpiMode(HighDpiMode.PerMonitorV2);

        // ---- 路表相关（全部无界面，可脚本化）----
        //   --make-route <out.bin> [半径m] [点距m]    生成一张**圆环**测试镜像
        //   --make-line  <out.bin> [长度m] [点距m]    生成一张**直线**测试镜像
        //   --check-route <bin>                       只校验镜像，不发串口
        //   --route-dir  <COMx>                       列出车端 5 个槽里都有什么
        //   --load-route <COMx> <bin> [槽号]           灌表（槽号默认 1；槽 0 禁止写）
        //   --read-route <COMx> <槽号> <out.bin>       把车上的表读回并存成镜像
        if (args.Length > 0 && (args[0] == "--make-route" || args[0] == "--make-line"))
        {
            Environment.ExitCode = MakeRoute(args, ring: args[0] == "--make-route");
            return;
        }
        if (args.Length > 0 && (args[0] == "--check-route" || args[0] == "--route-dir"
                                || args[0] == "--load-route" || args[0] == "--read-route"))
        {
            Environment.ExitCode = RouteCli(args);
            return;
        }

        // --sim <bin> [pb_v] [ld]：逐点指令推演（旧逻辑 vs 新逻辑），不连车、不做动力学仿真
        if (args.Length > 0 && args[0] == "--sim")
        {
            if (args.Length < 2) { Console.WriteLine("用法: --sim <镜像.bin> [pb_v=0.8] [ld=0.8]"); return; }

            byte[] simg = RouteImage.Load(args[1], out string serr);
            if (simg == null) { Console.WriteLine("读镜像失败：" + serr); return; }

            double pv = args.Length > 2 && double.TryParse(args[2], out double a1) ? a1 : 0.80;
            double sld = args.Length > 3 && double.TryParse(args[3], out double a2) ? a2 : 0.80;

            Console.WriteLine(RouteSim.Run(simg, pv, sld).Text);
            return;
        }
        // --audit <bin> [报告.txt]：只体检一张镜像（不连车），报告打到 stdout 或文件
        if (args.Length > 0 && args[0] == "--audit")
        {
            if (args.Length < 2) { Console.WriteLine("用法: --audit <镜像.bin> [报告.txt]"); return; }

            byte[] img = RouteImage.Load(args[1], out string aerr);
            if (img == null) { Console.WriteLine("读镜像失败：" + aerr); return; }

            string crc = RouteLink.VerifyImage(img);
            var audit = RouteAudit.Run(img, System.IO.Path.GetFileName(args[1]));
            string text = (crc == null ? "CRC：通过" : "CRC：不过 —— " + crc) + Environment.NewLine + audit.Report;

            if (args.Length > 2)
            {
                File.WriteAllText(args[2], text, new System.Text.UTF8Encoding(true));
                Console.WriteLine("报告 -> " + args[2]);
            }
            else Console.WriteLine(text);

            Environment.ExitCode = audit.Ok ? 0 : 2;      // 脚本里能直接用退出码判"体检是否通过"
            return;
        }
        // --shot-route <png>：把路表面板离屏画出来自查（不需要串口、不需要车）
        if (args.Length > 0 && args[0] == "--shot-route")
        {
            using var rf = new RouteForm
            {
                ShowInTaskbar = false,
                StartPosition = FormStartPosition.Manual,
                Location = new Point(-4000, -4000),
            };
            rf.Show();
            rf.ShotLoadDemo();          // 灌一张圆环 + 示例槽位，让表格/预览都有内容可看
            for (int i = 0; i < 20; i++) { Application.DoEvents(); System.Threading.Thread.Sleep(20); }
            using var rb = new Bitmap(rf.Width, rf.Height);
            rf.DrawToBitmap(rb, new Rectangle(0, 0, rf.Width, rf.Height));
            rb.Save(args.Length > 1 ? args[1] : "route_form.png", System.Drawing.Imaging.ImageFormat.Png);
            Console.WriteLine("shot -> " + (args.Length > 1 ? args[1] : "route_form.png"));
            return;
        }

        // --shot-bench <png>：把"台架直给"面板离屏画出来自查（不需要链路）
        if (args.Length > 0 && args[0] == "--shot-bench")
        {
            using var bf = new BenchForm((cmd, id, v) => true)
            {
                ShowInTaskbar = false,
                StartPosition = FormStartPosition.Manual,
                Location = new Point(-4000, -4000),
            };
            bf.Show();
            for (int i = 0; i < 20; i++) { Application.DoEvents(); System.Threading.Thread.Sleep(20); }
            using var bb = new Bitmap(bf.Width, bf.Height);
            bf.DrawToBitmap(bb, new Rectangle(0, 0, bf.Width, bf.Height));
            bb.Save(args.Length > 1 ? args[1] : "bench_form.png", System.Drawing.Imaging.ImageFormat.Png);
            Console.WriteLine("shot -> " + (args.Length > 1 ? args[1] : "bench_form.png"));
            return;
        }

        // --shot-form / --shot-demo：离屏截图自查界面布局（不出现在任务栏、不挡人）
        //   --shot-demo 会先灌一份假数据（假遥测 + 假参数表），用来检查波形与参数面板
        //   可选第 3 个参数指定分图数（1 / 2 / 4，默认 4），用来检查另一种布局有没有崩
        //   可选第 4 个参数指定视角：track = odom 轨迹图 / calib18 = 标定 18 路档案
        if (args.Length > 0 && (args[0] == "--shot-form" || args[0] == "--shot-demo"))
        {
            int lay = args.Length > 2 && int.TryParse(args[2], out int l) ? l : 4;
            ShotForm(args.Length > 1 ? args[1] : "form.png", args[0] == "--shot-demo", lay,
                     args.Length > 3 ? args[3] : null);
            return;
        }

        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new MainForm());
    }

    private static void ShotForm(string path, bool demo, int layout = 4, string view = null)
    {
        bool wantCalib = !string.IsNullOrEmpty(view) && view.Contains("calib18", StringComparison.OrdinalIgnoreCase);
        bool wantTrack = !string.IsNullOrEmpty(view) && view.Contains("track", StringComparison.OrdinalIgnoreCase);

        var f = new MainForm { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual, Location = new Point(-4000, -4000) };
        f.Show();
        // ★ 先让 Load 跑完（分隔条位置在 Load 里定），再把布局推一次，
        //   否则量到的 SplitterDistance 还是默认值，出的图会跟真启动时不一样。
        f.ApplyRuntimeLayoutForTest();

        // 档案必须**在灌假数据之前**切好：DemoFill 按 Defaults.Current 造数据
        if (wantCalib) f.SetProfileForTest(Defaults.Calib18);

        f.SetLayoutForTest(layout);
        for (int i = 0; i < 25; i++) { Application.DoEvents(); System.Threading.Thread.Sleep(20); }

        if (demo)
        {
            f.DemoFill();
            for (int i = 0; i < 20; i++) { Application.DoEvents(); System.Threading.Thread.Sleep(20); }
        }

        if (wantTrack)
        {
            f.SetTrackModeForTest(true);
            for (int i = 0; i < 15; i++) { Application.DoEvents(); System.Threading.Thread.Sleep(20); }
        }

        using var bmp = new Bitmap(f.Width, f.Height);
        f.DrawToBitmap(bmp, new Rectangle(0, 0, f.Width, f.Height));
        bmp.Save(path, System.Drawing.Imaging.ImageFormat.Png);

        // 顺带存几张 1:1 局部图 —— 整窗图缩到屏幕宽度后看不清文字，没法判断"是不是真的被裁了"
        string dir = Path.GetDirectoryName(Path.GetFullPath(path)) ?? ".";
        SaveCrop(bmp, Path.Combine(dir, "crop_bar.png"), 0, 0, 1120, 130);
        SaveCrop(bmp, Path.Combine(dir, "crop_wave.png"), 300, 60, Math.Min(900, bmp.Width - 300), 700);
        SaveCrop(bmp, Path.Combine(dir, "crop_param.png"), Math.Max(0, bmp.Width - 620), 60, 620, 760);

        Console.WriteLine($"shot -> {path}");
        Console.WriteLine(f.DumpLayout());
        f.Close();
    }

    private static void SaveCrop(Bitmap src, string path, int x, int y, int w, int h)
    {
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x + w > src.Width) w = src.Width - x;
        if (y + h > src.Height) h = src.Height - y;
        if (w <= 0 || h <= 0) return;
        using var c = src.Clone(new Rectangle(x, y, w, h), src.PixelFormat);
        c.Save(path, System.Drawing.Imaging.ImageFormat.Png);
    }

    // ==================================================================
    // 路表命令行
    // ==================================================================
    private static double ArgD(string[] a, int i, double def)
        => a.Length > i && double.TryParse(a[i], System.Globalization.NumberStyles.Float,
                                           System.Globalization.CultureInfo.InvariantCulture, out double v) ? v : def;

    private static int MakeRoute(string[] a, bool ring)
    {
        string path = a.Length > 1 ? a[1] : (ring ? "route_ring.bin" : "route_line.bin");
        double p1 = ArgD(a, 2, ring ? 5.0 : 10.0);      // 半径 / 长度
        double p2 = ArgD(a, 3, 0.05);                   // 点距

        byte[] img = ring ? RouteImage.BuildRing(p1, p2, out _) : RouteImage.BuildLine(p1, p2, out _);

        string bad = RouteLink.VerifyImage(img);
        if (bad != null) { Console.WriteLine("内部错误：生成的镜像校验不过 —— " + bad); return 2; }

        if (!RouteImage.Save(path, img, out string err)) { Console.WriteLine("写文件失败：" + err); return 3; }

        var (cnt, ds, lap) = RouteLink.ImageInfo(img);
        Console.WriteLine($"已生成 {(ring ? "圆环" : "直线")}测试镜像 -> {Path.GetFullPath(path)}");
        Console.WriteLine($"  {img.Length} B = 36 + {cnt} × 16　点距 {ds} mm　一圈 {lap / 1000.0:F2} m");
        Console.WriteLine("  校验：通过（magic/ver/点数/点距/两个 CRC 都对）");
        return 0;
    }

    private static int RouteCli(string[] a)
    {
        try
        {
            switch (a[0])
            {
                case "--check-route":
                {
                    if (a.Length < 2) { Console.WriteLine("用法：--check-route <bin>"); return 64; }
                    var img = File.ReadAllBytes(a[1]);
                    string bad = RouteLink.VerifyImage(img);
                    var (cnt, ds, lap) = RouteLink.ImageInfo(img);
                    Console.WriteLine($"文件：{Path.GetFullPath(a[1])}");
                    Console.WriteLine($"  {img.Length} B　{cnt} 点　点距 {ds} mm　一圈 {lap / 1000.0:F2} m");
                    Console.WriteLine(bad == null ? "  校验：通过" : "  校验：**不过** —— " + bad);
                    return bad == null ? 0 : 1;
                }

                case "--route-dir":
                {
                    if (a.Length < 2) { Console.WriteLine("用法：--route-dir <COMx>"); return 64; }
                    using var rl = new RouteLink();
                    rl.Open(a[1]);
                    Console.WriteLine(rl.Ping());
                    var list = rl.Dir(out int active, out int n);
                    Console.WriteLine($"当前槽 = {active}，共 {n} 槽：");
                    foreach (var s in list) Console.WriteLine("  " + s.Text);
                    return 0;
                }

                case "--load-route":
                {
                    if (a.Length < 3) { Console.WriteLine("用法：--load-route <COMx> <bin> [槽号]"); return 64; }
                    int slot = a.Length > 3 && int.TryParse(a[3], out int s0) ? s0 : 1;
                    var img = File.ReadAllBytes(a[2]);
                    string bad = RouteLink.VerifyImage(img);
                    if (bad != null) { Console.WriteLine("镜像不自洽，没发：" + bad); return 1; }

                    using var rl = new RouteLink();
                    rl.Open(a[1]);
                    Console.WriteLine(rl.Ping());
                    Console.WriteLine(RouteImage.Describe(img));
                    Console.WriteLine($"开始灌表 -> 槽 {slot} …");
                    string msg = rl.Download(slot, img, (frac, note) =>
                        Console.Write($"\r  {frac * 100,5:F1}%  {note,-40}"));
                    Console.WriteLine();
                    Console.WriteLine(msg);
                    return 0;
                }

                case "--read-route":
                {
                    if (a.Length < 4) { Console.WriteLine("用法：--read-route <COMx> <槽号> <out.bin>"); return 64; }
                    if (!int.TryParse(a[2], out int slot)) { Console.WriteLine("槽号不是数字"); return 64; }
                    using var rl = new RouteLink();
                    rl.Open(a[1]);
                    Console.WriteLine(rl.Ping());
                    var img = rl.ReadSlot(slot, (frac, note) => Console.Write($"\r  {frac * 100,5:F1}%  {note,-40}"));
                    Console.WriteLine();
                    Console.WriteLine(RouteImage.Describe(img));
                    if (!RouteImage.Save(a[3], img, out string err2))
                    {
                        Console.WriteLine("写文件失败：" + err2);
                        return 3;
                    }
                    Console.WriteLine("已保存 -> " + Path.GetFullPath(a[3]));
                    return 0;
                }
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine("失败：" + ex.Message);
            return 1;
        }
        return 64;
    }
}
