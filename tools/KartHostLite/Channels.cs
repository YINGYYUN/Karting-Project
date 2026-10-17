namespace KartHost;

/// <summary>
/// 通道名/单位表 —— ★ **全工程唯一一份**，也是「要不要裁剪通道」这个决定的唯一落点。
///
/// <para><b>下标即协议</b>：这里的第 i 项必须与车端 <c>Host_Crtl.h</c> 的 <c>HOST_CH_*</c> 一一对应。
/// 改顺序 = 改协议，历史 CSV 的列义会整体错位（而且**不报错、不越界，只是名字全挪位**）。</para>
///
/// <para><b>为什么要独立成一个文件</b>：车端通道表还在讨论要不要精简（25 / 22 / 13）。
/// 把顺序与名字集中在这里，裁剪时只改这一个数组，不用碰波形、参数面板、CSV 任何一处。</para>
///
/// <para>⚠ 帧里是自描述的（<c>COUNT</c> 字段），所以**通道数变化不会让解析失败** ——
/// 只会让名字错位。所以本文件必须跟着车端一起改，改完对齐 <see cref="ChannelNames"/> 的长度。</para>
/// </summary>
public static class Defaults
{
    /// <summary>当前车端遥测档的名字（显示在窗口标题/状态栏，便于现场核对）</summary>
    public const string ProfileName = "卡丁·基础";

    /// <summary>与车端 HOST_CH_* 逐项对应。0~3 舵机 / 4~11 后轮 / 12~17 遥控 / 18~24 状态</summary>
    public static readonly string[] ChannelNames =
    {
        "srv_tgt",   // 0  HOST_CH_SERVO_TGT   舵机施加目标角（软限位+斜坡之后）
        "srv_ang",   // 1  HOST_CH_SERVO_ANG   舵机实际角
        "srv_out",   // 2  HOST_CH_SERVO_OUT   PWM duty（±10000 = ±100%）
        "srv_raw",   // 3  HOST_CH_SERVO_RAW   绝对编码器原始角 0~4095

        "lr_tgt",    // 4  HOST_CH_LR_TGT_APP  后轮L 斜坡后目标
        "lr_v",      // 5  HOST_CH_LR_V        后轮L 实测转速
        "lr_out",    // 6  HOST_CH_LR_OUT      后轮L 归一化输出
        "lr_uff",    // 7  HOST_CH_LR_UFF      后轮L 前馈分量

        "rr_tgt",    // 8  HOST_CH_RR_TGT_APP
        "rr_v",      // 9  HOST_CH_RR_V
        "rr_out",    // 10 HOST_CH_RR_OUT
        "rr_uff",    // 11 HOST_CH_RR_UFF

        "rc_ch1",    // 12 HOST_CH_RC_CH1      遥控 CH1 原始计数（方向盘，回中≈992）
        "rc_ch2",    // 13 HOST_CH_RC_CH2      遥控 CH2 原始计数（油门）
        "rc_steer",  // 14 HOST_CH_RC_STEER    方向盘指令角
        "rc_thr",    // 15 HOST_CH_RC_THR      归一化油门 -1~+1
        "rc_eng",    // 16 HOST_CH_RC_ENGAGE   0 OFF/1 STANDBY/2 ON/3 FAILSAFE/4 ESTOP
        "rc_onl",    // 17 HOST_CH_RC_ONLINE   接收机状态

        "link",      // 18 HOST_CH_HOST_LINK   本链路 0 空闲/掉线 1 进行中
        "age_ms",    // 19 HOST_CH_HOST_AGE_MS 距上次收到下行帧 ms（从未收到 = -1）

        "srv_en",    // 20 HOST_CH_SERVO_EN
        "mot_en",    // 21 HOST_CH_MOTOR_EN
        "srv_flt",   // 22 HOST_CH_SERVO_FAULT
        "yaw_deg",   // 23 HOST_CH_YAW_DEG     来自 CM7_1 共享内存
        "host_en",   // 24 HOST_CH_HOST_EN     上位机下发的使能位图原值
    };

    public static readonly string[] ChannelUnits =
    {
        "°", "°", "duty", "cnt",
        "rps", "rps", "-", "-",
        "rps", "rps", "-", "-",
        "cnt", "cnt", "°", "-", "-", "-",
        "-", "ms",
        "-", "-", "-", "°", "-",
    };

    /// <summary>
    /// 中文简称 —— **只用于界面显示**（右侧勾选列表），CSV 表头与波形图例仍用英文名。
    ///
    /// <para>为什么不让 <see cref="ChannelNames"/> 直接写中文：CSV 列头是给脚本/Excel 认的，
    /// 波形图例宽度也有限；而**协议对应关系**靠的正是英文名与车端 <c>HOST_CH_*</c> 的一一对照，
    /// 混进中文以后 grep 对不上了。</para>
    ///
    /// <para>⚠ 长度上限按 6 个汉字估：勾选框宽 216 逻辑像素，英文名占 9 字符（约 80px）
    /// + 勾选标记 18px + 边距，剩给中文的大约 6 个汉字。超了会被裁掉（不报错）。</para>
    /// </summary>
    public static readonly string[] ChannelCn =
    {
        "舵机目标角",     // 0
        "舵机实际角",     // 1
        "舵机PWM",        // 2
        "编码器原始值",   // 3

        "左轮目标转速",   // 4
        "左轮实测转速",   // 5
        "左轮控制输出",   // 6
        "左轮前馈",       // 7

        "右轮目标转速",   // 8
        "右轮实测转速",   // 9
        "右轮控制输出",   // 10
        "右轮前馈",       // 11

        "CH1方向盘",      // 12
        "CH2油门",        // 13
        "遥控方向角",     // 14
        "遥控油门",       // 15
        "遥控状态机",     // 16
        "接收机在线",     // 17

        "上位机链路",     // 18
        "下行帧龄",       // 19

        "舵机使能",       // 20
        "电机使能",       // 21
        "舵机故障",       // 22
        "偏航角",         // 23
        "上位机使能位",   // 24
    };

    /// <summary>
    /// 悬停提示（鼠标停在勾选框上显示）—— 写「通道号 / 英文名 / 单位」+ 取值范围与坑。
    /// 这里可以写长，反正不占屏幕。
    /// </summary>
    public static readonly string[] ChannelTips =
    {
        /* 0  */ "HOST_CH_SERVO_TGT\n舵机施加目标角（°）\n软限位 ±28° 与斜坡之后的最终目标，控制器真正追的就是它",
        /* 1  */ "HOST_CH_SERVO_ANG\n舵机实际角（°），向右为正\n由绝对编码器换算，跟 srv_tgt 的差就是跟踪误差",
        /* 2  */ "HOST_CH_SERVO_OUT\n舵机 PWM duty（±10000 = ±100%）\n控制器的归一化输出，未乘 PWM 上限",
        /* 3  */ "HOST_CH_SERVO_RAW\n舵机绝对编码器原始计数 0~4095\n回中约 2157；只有排查编码器故障时才需要看",
        /* 4  */ "HOST_CH_LR_TGT_APP\n左后轮目标转速（rps）\n斜坡/夹紧之后的实际施加值",
        /* 5  */ "HOST_CH_LR_V\n左后轮实测转速（rps）\n由编码器在 5ms 窗口内算出",
        /* 6  */ "HOST_CH_LR_OUT\n左后轮归一化控制输出（±1 = ±100% 占空比）\n出口才乘 PWM_DUTY_MAX",
        /* 7  */ "HOST_CH_LR_UFF\n左后轮前馈分量\n速度环里由目标转速直接算出的那一部分，不是 PID 输出",
        /* 8  */ "HOST_CH_RR_TGT_APP\n右后轮目标转速（rps）\n同 lr_tgt",
        /* 9  */ "HOST_CH_RR_V\n右后轮实测转速（rps）",
        /* 10 */ "HOST_CH_RR_OUT\n右后轮归一化控制输出（±1 = ±100%）",
        /* 11 */ "HOST_CH_RR_UFF\n右后轮前馈分量",
        /* 12 */ "HOST_CH_RC_CH1\n遥控 CH1 原始计数（方向盘）\n回中约 992；做遥控标定时看这个",
        /* 13 */ "HOST_CH_RC_CH2\n遥控 CH2 原始计数（油门）",
        /* 14 */ "HOST_CH_RC_STEER\n遥控方向盘换算出的指令角（°）\n已按 ±28° 映射，与 srv_tgt 同量纲",
        /* 15 */ "HOST_CH_RC_THR\n归一化油门 −1 ~ +1\n0 = 中位不动",
        /* 16 */ "HOST_CH_RC_ENGAGE\n遥控状态机：0 OFF / 1 READY / 2 DRIVE / 3 FAILSAFE / 4 ESTOP\n3 与 4 都会锁存刹停，要在遥控页人工复位",
        /* 17 */ "HOST_CH_RC_ONLINE\n接收机在线：1 在线 / 0 失联\n失联时即使上位机在开车也会停车（RC LOST 锁存）",
        /* 18 */ "HOST_CH_HOST_LINK\n上位机链路：1 收发进行中 / 0 空闲\n这是车端自己报的状态，不是\"上位机收到了\"",
        /* 19 */ "HOST_CH_HOST_AGE_MS\n距上次收到上位机下行帧的毫秒数\n从未收到过 = −1；CMD 模式下 >600ms 会锁存刹停",
        /* 20 */ "HOST_CH_SERVO_EN\n舵机使能：1 已使能 / 0 松开",
        /* 21 */ "HOST_CH_MOTOR_EN\n后轮电机使能：1 已使能 / 0 滑行",
        /* 22 */ "HOST_CH_SERVO_FAULT\n舵机故障标志：非 0 = 编码器丢失等故障已锁存\n有两个复位入口（调试页、遥控页）",
        /* 23 */ "HOST_CH_YAW_DEG\n偏航角（°），来自 CM7_1 的 IMU 共享内存\n尚未接温漂/零偏处理，只当参考",
        /* 24 */ "HOST_CH_HOST_EN\n上位机下发的使能位图（原值）\nbit0 舵机 / bit1 电机；只有 CMD 模式才会真正生效",
    };

    public static string ChannelName(int i)
        => i >= 0 && i < ChannelNames.Length ? ChannelNames[i] : ("ch" + i);

    public static string ChannelUnit(int i)
        => i >= 0 && i < ChannelUnits.Length ? ChannelUnits[i] : "";

    /// <summary>右侧勾选列表显示的名字：<c>" 5  lr_v      左轮实测转速"</c>（英文名补到 9 字符对齐）</summary>
    public static string ChannelText(int i)
    {
        string en = ChannelName(i);
        string cn = i >= 0 && i < ChannelCn.Length ? ChannelCn[i] : "";
        return $"{i,2}  {en,-9}{cn}";
    }

    public static string ChannelTip(int i)
        => i >= 0 && i < ChannelTips.Length ? ChannelTips[i] : ChannelName(i);

    /// <summary>默认勾选画出来的通道（速度环左右各一条 + 舵机跟随即可以开跑）</summary>
    public static readonly int[] DefaultShown = { 0, 1, 4, 5, 8, 9 };

    // ==================================================================
    // 分图预设 —— 与队友 KartHost 的 `Defaults.PlotPresets` 同一套用法
    //
    // 分组原则是**同量纲放一张图**：自动量程是按「本图所有通道的极值」算的，
    // 把 ±10000 的 duty 和 ±0.02° 的角度塞一张图，小的那条会压成一条直线。
    // ==================================================================

    /// <summary>2×1 分图（两张图各自管一半通道）</summary>
    public static readonly (string Name, int[] Ch)[] PlotPresets2 =
    {
        ("舵机 / 后轮转速", new[] { 0, 1, 5, 9 }),
        ("输出 / 遥控 / 链路", new[] { 2, 6, 10, 14, 15, 18, 19 }),
    };

    /// <summary>2×2 分图（四张图，每张一个功能块）</summary>
    public static readonly (string Name, int[] Ch)[] PlotPresets4 =
    {
        ("舵机角（目标 / 实际）", new[] { 0, 1 }),
        ("后轮转速（L / R）", new[] { 5, 9 }),
        ("控制输出（舵机 duty / 左右轮）", new[] { 2, 6, 10 }),
        ("遥控指令 / 链路状态", new[] { 14, 15, 18, 19 }),
    };

    /// <summary>
    /// 右上「实时值」的 6 个格子：显示哪几个通道 + 标题 + 单位。
    ///
    /// <para>⚠ 单位字符串尽量短（"°" / "-" / "ms"）：数值标签只有 62 逻辑像素宽，
    /// 单位从 x=70 起，写 "deg" 会顶到 149 反而压在数值上（队友那边是布局体检抓出来的）。</para>
    ///
    /// <para>⚠ 改通道下标要连 <see cref="ChannelNames"/> 一起看 —— 这里写错不会报错，
    /// 只会显示一个"看起来挺合理"的别的量。</para>
    /// </summary>
    public static readonly (int Ch, string Cap, string Unit)[] ValueBoxes =
    {
        (0,  "舵机目标", "°"),
        (1,  "舵机实际", "°"),
        (5,  "后轮 L",   "rps"),
        (9,  "后轮 R",   "rps"),
        (14, "方向盘",   "°"),
        (19, "链路时延", "ms"),
    };
}
