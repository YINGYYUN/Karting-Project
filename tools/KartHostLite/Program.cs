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
        Application.SetHighDpiMode(HighDpiMode.PerMonitorV2);

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
}
