using System.Text;

namespace KartHost;

/// <summary>
/// 车端路表镜像（<c>route_car.bin</c>）的生成与读写。
///
/// <para><b>格式（必须与车端 <c>ins_route.c</c> 的 rt_head_t / rt_rec_t 逐字节一致）</b>：</para>
/// <code>
/// 偏移 0   36 B 表头   magic(4)="CTRK" | ver(2)=2 | rec_len(2)=16 | count(4) |
///                      ds_mm(4) | lap_mm(4) | origin_x_mm(4) | origin_y_mm(4) |
///                      origin_yaw_cdeg(2) | flags(2) | crc_r(2) | crc_h(2)
/// 偏移 36  count × 16 B 记录
///                      x_mm(4) | y_mm(4) | yaw_cdeg(2) | v_cmps(2) | steer_cdeg(2) | flags(2)
/// </code>
///
/// <para><b>两个 CRC16-MODBUS</b>：<c>crc_h</c> 覆盖头前 34 B，<c>crc_r</c> 覆盖整个记录区。</para>
///
/// <para><b>坐标约定</b>（与车端 odom 一致）：x 向右为正、y 向前为正、yaw 正 = 左转。
/// 记录的 x/y/yaw 都是**相对录制起点**的量。</para>
///
/// <para>⚠ 本文件生成的是**合成测试表**，只用来验证"灌表链路通不通、车端认不认"。
/// 真正要跑的赛道表得由电子图转出来（形状/点距都要按赛道定）。</para>
/// </summary>
public static class RouteImage
{
    public const int HeadBytes = 36;
    public const int RecBytes = 16;

    /// <summary>
    /// 生成一张**圆环**测试表：从原点出发、逆时针绕一圈回到原点。
    /// <para>为什么用圆环：灌进去之后在轨迹图上应该是一个正圆 —— 一眼就能看出
    /// "点数对、坐标没被缩放、x/y 没互换"。直线表看不出后面这几条。</para>
    /// </summary>
    /// <param name="radiusM">半径 m（建议 5）</param>
    /// <param name="dsM">点距 m（车端默认 0.05）</param>
    /// <param name="count">实际点数</param>
    public static byte[] BuildRing(double radiusM, double dsM, out int count)
    {
        if (radiusM <= 0.5) radiusM = 0.5;
        if (dsM <= 0.005) dsM = 0.005;

        int n = (int)Math.Round(2 * Math.PI * radiusM / dsM);
        if (n < 16) n = 16;
        if (n > RouteLink.SlotCapacity) n = RouteLink.SlotCapacity;
        count = n;

        var img = new byte[HeadBytes + n * RecBytes];

        double dTheta = 2 * Math.PI / n;

        for (int i = 0; i < n; i++)
        {
            double th = i * dTheta;

            // 圆心在 (0, R)，θ=0 时车在原点
            double x = radiusM * Math.Sin(th);
            double y = radiusM * (1.0 - Math.Cos(th));

            // 切线方向（对 θ 求导）：(R cosθ, R sinθ)
            // 航向约定：yaw=0 朝 +y，正 yaw = 左转 ⇒ forward = (-sin yaw, cos yaw)
            double dx = Math.Cos(th), dy = Math.Sin(th);
            double yawDeg = Math.Atan2(-dx, dy) * 180.0 / Math.PI;

            int o = HeadBytes + i * RecBytes;
            PutI32(img, o + 0, (int)Math.Round(x * 1000.0));
            PutI32(img, o + 4, (int)Math.Round(y * 1000.0));
            PutI16(img, o + 8, (short)Math.Round(yawDeg * 100.0));
            PutI16(img, o + 10, 0);                 // v_cmps = 0（回放车速由 pb_v 给）
            PutI16(img, o + 12, 0);                 // steer_cdeg = 0，但下面标了"打角有效"
            PutU16(img, o + 14, 1);                 // flags bit0 = INS_WP_FL_STEER
        }

        int lapMm = (int)Math.Round(2 * Math.PI * radiusM * 1000.0);
        int dsMm = Math.Max(1, (int)Math.Round(dsM * 1000.0));

        return FinishHead(img, n, dsMm, lapMm);
    }

    /// <summary>
    /// 生成一张**直线**测试表（从原点沿 +y 走 <paramref name="lengthM"/> 米）。
    /// 用途：验证"yaw 全 0 时轨迹图应该是一条竖直向上的线"。
    /// </summary>
    public static byte[] BuildLine(double lengthM, double dsM, out int count)
    {
        if (lengthM <= 1.0) lengthM = 1.0;
        if (dsM <= 0.005) dsM = 0.005;

        int n = (int)Math.Round(lengthM / dsM);
        if (n < 8) n = 8;
        if (n > RouteLink.SlotCapacity) n = RouteLink.SlotCapacity;
        count = n;

        var img = new byte[HeadBytes + n * RecBytes];
        for (int i = 0; i < n; i++)
        {
            int o = HeadBytes + i * RecBytes;
            PutI32(img, o + 0, 0);
            PutI32(img, o + 4, (int)Math.Round(i * dsM * 1000.0));
            PutI16(img, o + 8, 0);
            PutI16(img, o + 10, 0);
            PutI16(img, o + 12, 0);
            PutU16(img, o + 14, 1);
        }

        int lapMm = (int)Math.Round(lengthM * 1000.0);
        int dsMm = Math.Max(1, (int)Math.Round(dsM * 1000.0));
        return FinishHead(img, n, dsMm, lapMm);
    }

    private static byte[] FinishHead(byte[] img, int count, int dsMm, int lapMm)
    {
        PutU32(img, 0, RouteLink.ImgMagic);
        PutU16(img, 4, RouteLink.ImgVer);
        PutU16(img, 6, RouteLink.RecSize);
        PutU32(img, 8, (uint)count);
        PutU32(img, 12, (uint)dsMm);
        PutU32(img, 16, (uint)lapMm);
        PutI32(img, 20, 0);          // origin_x_mm
        PutI32(img, 24, 0);          // origin_y_mm
        PutI16(img, 28, 0);          // origin_yaw_cdeg
        PutU16(img, 30, 0);          // flags
        PutU16(img, 32, RouteLink.Crc16(0xFFFF, img, HeadBytes, count * RecBytes));
        PutU16(img, 34, RouteLink.Crc16(0xFFFF, img, 0, 34));
        return img;
    }

    private static void PutU16(byte[] b, int o, ushort v) { b[o] = (byte)v; b[o + 1] = (byte)(v >> 8); }
    private static void PutU32(byte[] b, int o, uint v) => RouteLink.PutU32(b, o, v);
    private static void PutI16(byte[] b, int o, short v) => PutU16(b, o, (ushort)v);
    private static void PutI32(byte[] b, int o, int v) => RouteLink.PutU32(b, o, (uint)v);

    /// <summary>加载一个 .bin 镜像；顺手把不合法的挡在外面（宁可报错，不要灌进去才发现）。</summary>
    public static byte[] Load(string path, out string err)
    {
        err = null;
        try
        {
            var img = File.ReadAllBytes(path);
            err = RouteLink.VerifyImage(img);
            if (err != null) { var bad = img; img = null; return bad; }   // 交回原始字节，让界面显示尺寸
            return img;
        }
        catch (Exception ex) { err = ex.Message; return null; }
    }

    public static bool Save(string path, byte[] img, out string err)
    {
        err = null;
        try
        {
            string dir = Path.GetDirectoryName(Path.GetFullPath(path)) ?? ".";
            Directory.CreateDirectory(dir);
            File.WriteAllBytes(path, img);
            return true;
        }
        catch (Exception ex) { err = ex.Message; return false; }
    }

    /// <summary>给人看的一句话摘要。</summary>
    public static string Describe(byte[] img)
    {
        if (img == null) return "（没有镜像）";
        var (cnt, ds, lap) = RouteLink.ImageInfo(img);
        var sb = new StringBuilder();
        sb.Append($"{img.Length} B（= 36 + {img.Length - HeadBytes}/{RecBytes} × {RecBytes}）");
        if (img.Length >= HeadBytes)
            sb.Append($"　{cnt} 点　点距 {ds} mm　一圈 {lap / 1000.0:F2} m");
        string bad = RouteLink.VerifyImage(img);
        sb.Append(bad == null ? "　✔ 校验通过" : $"　✘ {bad}");
        return sb.ToString();
    }
}
