namespace KartHost;

/// <summary>
/// 一套遥测档案：4 张**平行**表（Names / Units / Cn / Tips）+ 分图预设 + 实时值格子 + 轨迹通道。
///
/// <para><b>下标即协议</b>：第 i 项必须与车端 <c>Host_Ctrl.h</c> 的 <c>HOST_CH_*</c>（全量）
/// 或 <c>HOST_CT_*</c>（标定）一一对应。改顺序 = 改协议，历史 CSV 的列义会整体错位
/// （而且**不报错、不越界，只是名字全挪位**）。</para>
///
/// <para>⚠ 帧里是自描述的（<c>COUNT</c> 字段），所以通道数变化不会让解析失败 ——
/// 只会让名字错位。这正是要按 <c>COUNT</c> **选档案**、并对未知 <c>COUNT</c> 直接丢帧的原因。</para>
/// </summary>
public sealed class ChannelProfile
{
    /// <summary>显示名（窗口标题 / 状态栏）</summary>
    public string Name = "";

    /// <summary>短名，只用于 CSV 文件名 —— 两种列义绝不能混进同一个文件</summary>
    public string Short = "";

    /// <summary>项数（必须 == Names.Length，也 == 车端帧头 COUNT）</summary>
    public int Count => Names.Length;

    public string[] Names = Array.Empty<string>();
    public string[] Units = Array.Empty<string>();

    /// <summary>中文简称 —— **只用于界面显示**（右侧勾选列表）。</summary>
    public string[] Cn = Array.Empty<string>();

    /// <summary>悬停提示 —— 写「通道号 / 英文名 / 单位」+ 取值范围与坑。</summary>
    public string[] Tips = Array.Empty<string>();

    /// <summary>默认勾选画出来的通道</summary>
    public int[] DefaultShown = Array.Empty<int>();

    /// <summary>2×1 分图预设</summary>
    public (string Name, int[] Ch)[] Presets2 = Array.Empty<(string, int[])>();

    /// <summary>2×2 分图预设</summary>
    public (string Name, int[] Ch)[] Presets4 = Array.Empty<(string, int[])>();

    /// <summary>右上「实时值」的 6 个格子</summary>
    public (int Ch, string Cap, string Unit)[] ValueBoxes = Array.Empty<(int, string, string)>();

    /// <summary>
    /// odom 轨迹图用的两路下标：{ x, y }。
    /// **空数组 = 本档案没有这两路**，轨迹图显示 N/A（标定档就是这样）。
    /// </summary>
    public int[] OdomXy = Array.Empty<int>();

    public bool HasOdom => OdomXy.Length == 2;

    public string NameOf(int i) => i >= 0 && i < Names.Length ? Names[i] : ("ch" + i);
    public string UnitOf(int i) => i >= 0 && i < Units.Length ? Units[i] : "";

    /// <summary>右侧勾选列表显示的名字：<c>" 5  v_l        左轮实测转速"</c>（英文名补到 11 字符对齐）</summary>
    public string TextOf(int i)
    {
        string en = NameOf(i);
        string cn = i >= 0 && i < Cn.Length ? Cn[i] : "";
        return $"{i,2}  {en,-11}{cn}";
    }

    public string TipOf(int i) => i >= 0 && i < Tips.Length ? Tips[i] : NameOf(i);
}

/// <summary>
/// 通道名/单位表 —— ★ **全工程唯一一份**，也是「当前用哪套档案」这个状态的唯一落点。
///
/// <para><b>两套档案，与队友 motor_link.h 完全一致</b>（2026-10-22 定稿，不做双档案兼容）：</para>
/// <list type="bullet">
///   <item><see cref="Full41"/> —— 全量 41 路，帧头 COUNT = 41（默认）</item>
///   <item><see cref="Calib18"/> —— 标定精简 18 路，帧头 COUNT = 18（车端 <c>tel_sel = 1</c>）</item>
/// </list>
///
/// <para><b>切换规则</b>：只按帧头 <c>COUNT</c> 自动选（<see cref="ByCount"/>）。
/// 其它值一律视为未知档案 —— 调用方必须**丢弃该帧**并在状态栏提示，
/// 绝不能"猜一个最接近的档案"去套名字（那会把数据标错名，而且不报错）。</para>
///
/// <para>⚠ 顺序即协议，定稿后**不得再重排**。要加通道请先改队友车端那份，再同步这里。</para>
/// </summary>
public static class Defaults
{
    /// <summary>
    /// 当前生效的档案。切换只走 <see cref="Select"/>。
    ///
    /// <para>⚠ **不能**写成 <c>= Full41</c> 的自动属性初始化：C# 的静态字段初始化按**书写顺序**执行，
    /// 而 <see cref="Full41"/> 是写在本类后面的静态字段 —— 那样初始化时它还是 null，
    /// <see cref="Current"/> 会得到一个 null，然后在第一次读 <c>Current.Name</c> 时抛 NullReferenceException
    /// （症状：一启动就崩在窗口构造函数里）。所以这里用延迟初始化。</para>
    /// </summary>
    public static ChannelProfile Current => _cur ??= Full41;

    private static ChannelProfile _cur;

    /// <summary>
    /// 按帧头 COUNT 选档案。返回 <c>null</c> = 未知档案（调用方应丢帧并提示）。
    /// </summary>
    public static ChannelProfile ByCount(int count)
        => count == Full41.Count ? Full41
         : count == Calib18.Count ? Calib18
         : null;

    /// <summary>切到指定档案。返回 true = 真的换了（调用方需要重建界面 / 清数据）。</summary>
    public static bool Select(ChannelProfile p)
    {
        if (p == null || ReferenceEquals(p, Current)) return false;
        _cur = p;
        return true;
    }

    public static string ChannelName(int i) => Current.NameOf(i);
    public static string ChannelUnit(int i) => Current.UnitOf(i);
    public static string ChannelText(int i) => Current.TextOf(i);
    public static string ChannelTip(int i) => Current.TipOf(i);

    // ==================================================================
    // 档案一：全量 41 路（队友 motor_link.h 的 CH_*，下标 0~40）
    // ==================================================================
    public static readonly ChannelProfile Full41 = new()
    {
        Name = "卡丁·全量41",
        Short = "full41",
        Names = new[]
        {
            "target_l",  // 0  目标转速 左轮 rps
            "v_l",       // 1  实测转速 左轮 rps
            "u_l",       // 2  实际输出 左轮 %（车端已 ×100，**不是** ±1）
            "target_r",  // 3
            "v_r",       // 4
            "u_r",       // 5
            "uff_l",     // 6  前馈分量 左轮 %
            "uff_r",     // 7
            "z_l",       // 8  积分状态 左轮 rps*s
            "z_r",       // 9
            "pos_l",     // 10 编码器累计计数（cnt）
            "pos_r",     // 11
            "online",    // 12 链路在线 0/1
            "age_ms",    // 13 下行帧龄 ms（从未收到 = -1）
            "en_bits",   // 14 使能位图 bit1 电机 / bit2 总使能
            "safety",    // 15 保护位图（0 = 正常）
            "steer_tgt", // 16 舵机施加目标角 deg
            "steer_deg", // 17 舵机实际角 deg
            "steer_u",   // 18 舵机 duty（±10000）
            "steer_en",  // 19 转向内环使能 0/1
            "steer_st",  // 20 转向状态位
            "rc_eng",    // 21 接管状态 0~4
            "rc_ch1",    // 22
            "rc_ch2",    // 23
            "rc_frames", // 24 接收机累计帧数
            "k_v",       // 25
            "k_z",       // 26
            "yaw",       // 27 累积航向 deg
            "gz",        // 28 陀螺 Z dps
            "ins_x",     // 29 航位推算 x m（右为正）
            "ins_y",     // 30 航位推算 y m（前为正）
            "ins_dist",  // 31 累计里程 m
            "ins_flags", // 32 惯导状态位
            "rt_cnt",    // 33 路表点数
            "rt_st",     // 34 路表状态位 | (槽号 << 8)
            "pb_elat",   // 35 回放横向误差 m
            "tar_yaw",   // 36 目标航向 deg
            "ht_seg",    // 37 航向测试段号（我方固定 0，非等价）
            "tick_ms",   // 38 车端上电毫秒数
            "ins_frz",   // 39 ZUPT 冻结丢角 deg
            "ins_static",// 40 静止判定 0/1
        },
        Units = new[]
        {
            "rps", "rps", "%", "rps", "rps", "%",
            "%", "%", "rps*s", "rps*s",
            "cnt", "cnt",
            "-", "ms", "-", "-",
            "deg", "deg", "duty", "-", "-",
            "-", "cnt", "cnt", "-",
            "-", "-",
            "deg", "dps", "m", "m", "m", "-",
            "-", "-", "m", "deg", "-", "ms", "deg", "-",
        },
        Cn = new[]
        {
            "左轮目标", "左轮转速", "左轮输出", "右轮目标", "右轮转速", "右轮输出",
            "左轮前馈", "右轮前馈", "左轮积分", "右轮积分",
            "左轮计数", "右轮计数",
            "链路在线", "下行帧龄", "使能位", "保护位",
            "舵机目标", "舵机实际", "舵机PWM", "舵机使能", "舵机状态",
            "遥控状态", "CH1方向盘", "CH2油门", "接收机帧数",
            "速度Kv", "积分Kz",
            "航向角", "陀螺Z", "推算X", "推算Y", "累计里程", "惯导状态",
            "路表点数", "路表状态", "回放横差", "目标航向", "航测段号",
            "车端时基", "冻结丢角", "静止判定",
        },
        Tips = new[]
        {
            /*  0 */ "下标 0  CH_TARGET_L\ntarget_l\n左后轮目标转速（rps）\n斜坡/夹紧之后的实际施加值",
            /*  1 */ "下标 1  CH_V_L\nv_l\n左后轮实测转速（rps）\n由编码器在 5ms 窗口内算出",
            /*  2 */ "下标 2  CH_U_L\nu_l\n左后轮实际输出（**百分数 %**，带符号）\n★ 车端已把归一化 ±1 乘了 100 ⇒ 满占空比是 ±50 而不是 ±0.5",
            /*  3 */ "下标 3  CH_TARGET_R\ntarget_r\n右后轮目标转速（rps）",
            /*  4 */ "下标 4  CH_V_R\nv_r\n右后轮实测转速（rps）",
            /*  5 */ "下标 5  CH_U_R\nu_r\n右后轮实际输出（%），同 u_l",
            /*  6 */ "下标 6  CH_UFF_L\nuff_l\n左后轮前馈分量（%）\n速度环里由目标转速直接算出的那一部分，不是 PI 输出",
            /*  7 */ "下标 7  CH_UFF_R\nuff_r\n右后轮前馈分量（%）",
            /*  8 */ "下标 8  CH_Z_L\nz_l\n左后轮积分状态（rps*s）\n看它是否顶在 ±MOTOR_CTRL_Z_MAX 就知道积分有没有裹死",
            /*  9 */ "下标 9  CH_Z_R\nz_r\n右后轮积分状态（rps*s）",
            /* 10 */ "下标 10 CH_POS_L\npos_l\n左后轮带符号累计编码器计数（cnt）\n标 cpr 用；不是米",
            /* 11 */ "下标 11 CH_POS_R\npos_r\n右后轮累计计数（cnt）",
            /* 12 */ "下标 12 CH_ONLINE\nonline\n1 = 链路活着（最近 600ms 内收到过下行帧）\n这是**车端自己报的**，不是\"本机收到了\"",
            /* 13 */ "下标 13 CH_RX_AGE_MS\nage_ms\n距上次收到下行帧的毫秒数\n从未收到过 = -1；CMD 模式下 >600ms 会锁存刹停",
            /* 14 */ "下标 14 CH_ENABLE_BITS\nen_bits\n使能位图（只发 bit1 电机 / bit2 总使能）\nbit0 舵机不发（队友口径）",
            /* 15 */ "下标 15 CH_SAFETY\nsafety\n保护位图，**0 = 一切正常**\n1 没收到过下行 / 4 失联 / 8 心跳丢失 / 0x20 限幅 / 0x40 遥控接管 / 0x80 遥控急停\n★ 近似映射，与队友的 SAFETY 不等价，bit1/bit4 恒 0",
            /* 16 */ "下标 16 CH_STEER_TARGET\nsteer_tgt\n舵机**实际施加**的目标角（deg）\n软限位 ±28° 与斜坡之后的值，控制器真正追的就是它",
            /* 17 */ "下标 17 CH_STEER_DELTA\nsteer_deg\n舵机实际角（deg），向右为正\n与 steer_tgt 的差就是跟踪误差",
            /* 18 */ "下标 18 CH_STEER_U\nsteer_u\n转向电机输出 duty（±10000）\n不是百分数，别和 u_l/u_r 混",
            /* 19 */ "下标 19 CH_STEER_EN\nsteer_en\n转向内环使能 0/1",
            /* 20 */ "下标 20 CH_STEER_STATUS\nsteer_st\nbit0 自检通过 / bit1 反馈失效 / bit2 顶死或反转\n★ 我方无顶死判据 ⇒ bit2 恒 0（非等价）",
            /* 21 */ "下标 21 CH_RC_ENGAGE\nrc_eng\n遥控接管状态：0 OFF / 1 STANDBY / 2 ON / 3 FAILSAFE / 4 ESTOP\n3 与 4 都锁存，要在遥控页或上位机页人工复位",
            /* 22 */ "下标 22 CH_RC_CH1\nrc_ch1\nCH1 原始计数（方向盘），回中约 992\n做遥控标定时看这个",
            /* 23 */ "下标 23 CH_RC_CH2\nrc_ch2\nCH2 原始计数（油门）",
            /* 24 */ "下标 24 CH_RC_FRAMES\nrc_frames\n接收机累计帧数\n一直不涨 = 接收机没在发 / 帧格式对不上",
            /* 25 */ "下标 25 CH_K_V\nk_v\n本拍实际生效的比例增益（u/rps）\n只有对着\"车上真正生效的 k_v\"才比得动阶跃辨识结果",
            /* 26 */ "下标 26 CH_K_Z\nk_z\n本拍实际生效的积分增益（u/(rps*s)）",
            /* 27 */ "下标 27 CH_INS_YAW\nyaw\n累积航向（deg，可超 ±360）",
            /* 28 */ "下标 28 CH_INS_GZ\ngz\n陀螺 Z 角速度（度/秒，去零偏后）\n静止看它就是残余零偏",
            /* 29 */ "下标 29 CH_INS_X\nins_x\n航位推算 x（m），向右为正\n轨迹图的横轴就在这一路",
            /* 30 */ "下标 30 CH_INS_Y\nins_y\n航位推算 y（m），向前为正\n轨迹图纵轴 = **-y**（screen_y ∝ -y，车头朝上）",
            /* 31 */ "下标 31 CH_INS_DIST\nins_dist\n累计里程（m，标量）\n含倒车，与位置积分分开累加",
            /* 32 */ "下标 32 CH_INS_FLAGS\nins_flags\n惯导状态位（INS_ST_*）\n0x01 READY / 0x02 WARN / 0x10 链路坏 / 0x20 陀螺饱和 / 0x100 静止 …",
            /* 33 */ "下标 33 CH_RT_CNT\nrt_cnt\n当前 RAM 里的路表点数",
            /* 34 */ "下标 34 CH_RT_ST\nrt_st\n路表状态：低 8 位 = 状态位，bit8..11 = 槽号\n1 录制 / 2 FULL / 4 有表 / 8 写盘 / 0x10 写失败 / 0x20 回放 / 0x40 来自 Flash",
            /* 35 */ "下标 35 CH_PB_ELAT\npb_elat\n回放横向误差（m），正 = 车在参考线右侧\n没在回放时它不刷新",
            /* 36 */ "下标 36 CH_TAR_YAW\ntar_yaw\n本拍目标航向（deg）\n与 yaw(27) 成对看就是舵机外环的输入与输出",
            /* 37 */ "下标 37 CH_HT_SEG\nht_seg\n航向测试段号\n★ 我方 head_test 未迁移 ⇒ **固定 0，非等价**",
            /* 38 */ "下标 38 CH_TICK_MS\ntick_ms\n车端上电毫秒数（5ms 时基累加）\n掐 60s 秒表核时基就看它",
            /* 39 */ "下标 39 CH_INS_FRZ\nins_frz\nZUPT 冻结期间被丢弃的转角（deg，累计只读不清）\n转一圈它明显 >0 且与缺口量级相当 ⇒ 就是冻结吃掉的",
            /* 40 */ "下标 40 CH_INS_STATIC\nins_static\n1 = 本拍被判\"静止\"（航向正在被冻结）\n与 ins_frz 配对看",
        },
        DefaultShown = new[] { 0, 1, 3, 4, 16, 17 },
        Presets2 = new[]
        {
            ("后轮转速 / 舵机角", new[] { 0, 1, 3, 4, 16, 17 }),
            ("输出 % / 惯导位置", new[] { 2, 5, 6, 7, 29, 30 }),
        },
        Presets4 = new[]
        {
            ("后轮转速（目标 / 实测）", new[] { 0, 1, 3, 4 }),
            ("控制输出 %（含前馈）", new[] { 2, 5, 6, 7 }),
            ("舵机角（目标 / 实际）", new[] { 16, 17 }),
            ("惯导 x / y / 里程", new[] { 29, 30, 31 }),
        },
        ValueBoxes = new[]
        {
            (0,  "左轮目标", "rps"),
            (1,  "左轮实测", "rps"),
            (3,  "右轮目标", "rps"),
            (4,  "右轮实测", "rps"),
            (17, "舵机实际", "deg"),
            (13, "下行帧龄", "ms"),
        },
        OdomXy = new[] { 29, 30 },
    };

    // ==================================================================
    // 档案二：标定精简 18 路（队友 motor_link.h 的 CT_*，车端 tel_sel = 1）
    //  18 路 = 93 B/帧（全量 41 路 185 B）⇒ 同样的链路能把周期压到 5~10 ms
    // ==================================================================
    public static readonly ChannelProfile Calib18 = new()
    {
        Name = "卡丁·标定18",
        Short = "calib18",
        Names = new[]
        {
            "target_l",  // 0  CT_TARGET_L
            "v_l",       // 1  CT_V_L
            "u_l",       // 2  CT_U_L（%）
            "target_r",  // 3
            "v_r",       // 4
            "u_r",       // 5
            "steer_tgt", // 6  ★ 实际施加的目标角
            "steer_deg", // 7
            "steer_u",   // 8  duty
            "online",    // 9
            "en_bits",   // 10
            "safety",    // 11
            "steer_en",  // 12
            "steer_st",  // 13
            "ol_en",     // 14 ★ 开环模式 —— 我方无开环档，车端固定发 0（非等价）
            "resv",      // 15 预留位，车端固定发 0
            "k_v",       // 16
            "k_z",       // 17
        },
        Units = new[]
        {
            "rps", "rps", "%", "rps", "rps", "%",
            "deg", "deg", "duty",
            "-", "-", "-", "-", "-", "-", "-",
            "-", "-",
        },
        Cn = new[]
        {
            "左轮目标", "左轮转速", "左轮输出", "右轮目标", "右轮转速", "右轮输出",
            "舵机目标", "舵机实际", "舵机PWM",
            "链路在线", "使能位", "保护位", "舵机使能", "舵机状态",
            "开环(无)", "预留位", "速度Kv", "积分Kz",
        },
        Tips = new[]
        {
            /*  0 */ "下标 0  CT_TARGET_L\ntarget_l\n左后轮目标转速（rps）",
            /*  1 */ "下标 1  CT_V_L\nv_l\n左后轮实测转速（rps）",
            /*  2 */ "下标 2  CT_U_L\nu_l\n左电机实际 PWM（%，带符号）\n★ 是百分数，不是 ±1",
            /*  3 */ "下标 3  CT_TARGET_R\ntarget_r\n右后轮目标转速（rps）",
            /*  4 */ "下标 4  CT_V_R\nv_r\n右后轮实测转速（rps）",
            /*  5 */ "下标 5  CT_U_R\nu_r\n右电机实际 PWM（%）",
            /*  6 */ "下标 6  CT_STEER_TGT\nsteer_tgt\n舵机角度环**实际施加**的目标角（deg）\n软限位 + 斜坡之后，不是上位机写的命令",
            /*  7 */ "下标 7  CT_STEER_DELTA\nsteer_deg\n舵机角度环实际角（deg）",
            /*  8 */ "下标 8  CT_STEER_U\nsteer_u\n舵机 PWM（duty，-10000~10000）",
            /*  9 */ "下标 9  CT_ONLINE\nonline\n链路在线 0/1",
            /* 10 */ "下标 10 CT_ENABLE\nen_bits\n使能位图",
            /* 11 */ "下标 11 CT_SAFETY\nsafety\n保护位图（0 = 正常）；0x40 = 遥控接管中\n★ 近似映射，与队友不等价",
            /* 12 */ "下标 12 CT_STEER_EN\nsteer_en\n转向内环使能 0/1",
            /* 13 */ "下标 13 CT_STEER_ST\nsteer_st\nbit0 自检通过 / bit1 反馈失效 / bit2 顶死（我方恒 0）",
            /* 14 */ "下标 14 CT_OL_EN\nol_en\n开环模式 0/1\n★ 我方没有开环档 ⇒ **车端固定发 0**，这一路在本工程不含信息（非等价）",
            /* 15 */ "下标 15 CT_RESV0\nresv\n预留位（扩展用，别占用）\n车端固定发 0",
            /* 16 */ "下标 16 CT_K_V\nk_v\n本拍实际生效的比例增益（u/rps）\n标定档最需要它：阶跃算出来的 k_v 要对着这个比",
            /* 17 */ "下标 17 CT_K_Z\nk_z\n本拍实际生效的积分增益（u/(rps*s)）",
        },
        DefaultShown = new[] { 0, 1, 3, 4, 6, 7 },
        Presets2 = new[]
        {
            ("转速 / 舵机角", new[] { 0, 1, 3, 4, 6, 7 }),
            ("输出 % / 状态", new[] { 2, 5, 9, 11 }),
        },
        Presets4 = new[]
        {
            ("左轮（目标 / 实测）", new[] { 0, 1 }),
            ("右轮（目标 / 实测）", new[] { 3, 4 }),
            ("控制输出 %", new[] { 2, 5 }),
            ("舵机角（目标 / 实际）", new[] { 6, 7 }),
        },
        ValueBoxes = new[]
        {
            (0, "左轮目标", "rps"),
            (1, "左轮实测", "rps"),
            (3, "右轮目标", "rps"),
            (4, "右轮实测", "rps"),
            (7, "舵机实际", "deg"),
            (9, "链路在线", ""),
        },
        // ★ 标定档没有 ins_x / ins_y ⇒ 轨迹图显示 N/A（不是画一张空图让人以为没数据）
        OdomXy = Array.Empty<int>(),
    };
}
