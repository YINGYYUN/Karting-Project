/*******************************************************************************
方向盘舵机角度测量与控制 - Servo_Ctrl.c
本项目数据来源：绝对值角度编码器（ENC_ABS_ANG，CH1 的 5ms 中断采集，与位置环同拍，见 cm7_0_isr.c）
本项目控制对象：Motor 3（MOTOR_SERVO）
*******************************************************************************/

#include "zf_common_headfile.h"

//============================== 对外数据 ==============================
volatile int16 Servo_Ang_Raw            = 0;                // 当前原始读数 0 ~ 4095
volatile float Servo_Angle_Deg          = 0.0f;             // 当前方向盘角度 （度，向右为正）
volatile float Servo_Target_Deg         = 0.0f;             // 目标角度 （目标设定值，度，向右为正）
volatile float Servo_Target_Applied_Deg = 0.0f;             // 实际施加目标 （实际设定值，度，向右为正）
volatile float Servo_Err_Deg            = 0.0f;             // 本拍误差 （度）
volatile float Servo_I_Sum              = 0.0f;             // 积分累加值（deg·拍）
volatile int16 Servo_Out                = 0;                // 本拍实际写入电机输出
volatile float Servo_Ctrl_Kp            = SERVO_KP_DEFAULT; // 位置环比例增益 （参数页/Flash 会覆盖）
volatile float Servo_Ctrl_U0            = SERVO_U0_DEFAULT; // 摩擦截距前馈 （参数页/Flash 会覆盖）
volatile float Servo_Ctrl_Ki            = SERVO_KI_DEFAULT; // 位置环积分增益 （参数页/Flash 会覆盖）
volatile uint8 Servo_Ctrl_Enable        = 0;                // 舵机闭环使能标志位：1使能
volatile uint8 Servo_Enc_Fault          = 0;                // 角度编码器失效锁存标志位：1锁存

//============================== 内部状态 ==============================
static uint8 servo_enabled          = 0;                    // （上一拍）是否为启用状态（用于失能时只清一次输出）
static int8  Servo_U0_Dir           = 0;                    // 摩擦前馈的三态门：+1 / -1 / 0（关）
static uint32 servo_invalid_ms      = 0;                    // 电机在出力、却连续读到无效角度的累计时间

//--------------------------------------
// 函数简介     角度编码器失效保护
// 参数说明     void
// 备注信息     直接在主节拍 Servo_Ctrl_Tick 中调用
// 备注信息     角度编码器数据无效 且 |Servo_Out|大于设定阈值 -> 锁存停机。
// 备注信息     使用Servo_Ctrl_Reset()重置锁存
//--------------------------------------
static void Servo_Ctrl_EncCheck (void)
{
    int16 out = Servo_Out;

    if (Servo_Enc_Fault != 0)
    {
        return;                         // 反馈源失效已锁存，等 Servo_Ctrl_Reset()解除
    }

    /* 方向盘电机出力判定 */
    if ((Servo_Ctrl_Enable == 0) ||
        ((out < SERVO_ENC_LOST_OUT) && (out > -SERVO_ENC_LOST_OUT)))
    {
        // 未使能 或 未足够出力，提前结束判定
        servo_invalid_ms = 0;           // 重置掉线时间
        return;
    }

    /* 角度编码器失效判定 */
    // 利用SPI设备掉线的固定特征值进行判定
    // ENC_ABS_ANG   0 / 4095 交替
    // ENC_ABS_OFF   +1 / -1 交替（驱动的最短路径过零把 ±4095 折叠成 ±1）
    // 换算角度       0 / 3599 交替
    if ((Servo_Ang_Raw <= SERVO_ENC_RAW_INVALID_LOW) ||
        (Servo_Ang_Raw >= SERVO_ENC_RAW_INVALID_HIGH))
    {
        servo_invalid_ms += SERVO_TICK_MS;
    }
    // 数据有效
    else
    {
        servo_invalid_ms = 0;           // 重置掉线时间
    }

    /* 掉线时间到达阈值，停机 */
    if (servo_invalid_ms >= SERVO_ENC_LOST_MS)
    {
        servo_invalid_ms = 0;
        Servo_Enc_Fault  = 1;           // 停机状态锁存
    }
}

//--------------------------------------
// 函数简介     重置状态 （失能 / 清目标 / 误差 / 清计算中间变量 / 输出 / 故障锁存）
// 参数说明     void
// 使用示例     Servo_Ctrl_Reset();
// 备注信息     谢绝定时自动调用！！！
// 备注信息     不清 servo_enabled   （留给 Tick 补硬件PWM归零）
// 备注信息     不清 Servo_Ctrl_Kp/U0（这是参数）
//--------------------------------------
void Servo_Ctrl_Reset (void)
{
    Servo_Ctrl_Enable        = 0;       // 闭环失能
    Servo_Target_Deg         = 0.0f;
    Servo_Target_Applied_Deg = 0.0f;
    Servo_Err_Deg            = 0.0f;
    Servo_Out                = 0;
    Servo_I_Sum              = 0.0f;
    Servo_U0_Dir             = 0;       // 前馈三态门
    Servo_Enc_Fault          = 0;
    servo_invalid_ms         = 0;
}

//--------------------------------------
// 函数简介     转向控制初始化
// 参数说明     void
// 使用示例     Servo_Ctrl_Init();
// 备注信息     上电后调用一次
//--------------------------------------
void Servo_Ctrl_Init (void)
{
    Servo_Ctrl_Reset();
    Motor_Set(MOTOR_SERVO, 0);
}

//--------------------------------------
// 函数简介     5ms 主节拍
// 参数说明     void
// 使用示例     在 PIT 中断里调用 Servo_Ctrl_Tick();
// 备注信息     顺序：角度换算 -> 反馈失效判定 -> 门控 -> 目标处理 -> 位置环 -> 限幅 -> 输出
// 备注信息     角度换算为测量量，始终更新，无关舵机闭环使能
//--------------------------------------
void Servo_Ctrl_Tick (void)
{
    float tar;      // 舵机目标角度（度）
    float err;      // 角度误差（度）
    float abs_err;  // 角度误差绝对值（度）
    float p_out;    // 本拍 Kp 项输出
    float u0_out;   // 本拍摩擦前馈项输出
    float i_out;    // 本拍 Ki 项输出（已限幅到 SERVO_I_MAX）
    float i_add;    // 本拍累加进积分的量（0 = 本拍不累加）
    float out;      // 本拍输出值（极性未修正）
    uint8 block;    // 1 = 越限保护生效，本拍三项都不出力

    /* ---- 角度换算：原始读数 -> 方向盘角度（向右为正）---- */
    Servo_Ang_Raw   = ENC_ABS_ANG;
    Servo_Angle_Deg = SERVO_DIR_SIGN * ((float)ENC_ABS_ANG - SERVO_ENC_MID) / SERVO_CNT_PER_DEG;

    /* ---- 反馈源失效判定：用本拍原始读数 + 上一拍输出，先于门控 ---- */
    Servo_Ctrl_EncCheck();

    /* ---- 目标处理：软限位夹紧 ----
       软限位比物理限位留 SERVO_SOFT_MARGIN_DEG 余量，保证控制器不会主动把方向盘顶向限
       位结构。
       "实际施加目标"始终对外可见（Servo_Target_Applied_Deg），与是否使能无关。 */
    tar = Servo_Target_Deg;
    if (tar >  SERVO_SOFT_LIMIT_DEG) { tar =  SERVO_SOFT_LIMIT_DEG; }
    if (tar < -SERVO_SOFT_LIMIT_DEG) { tar = -SERVO_SOFT_LIMIT_DEG; }
    Servo_Target_Applied_Deg = tar;

    /* ---- 门控不通过：清状态 / 总使能关 / 反馈源失效锁存，在"使能 -> 失能"那一拍清零输出 ---- */
    if ((Servo_Ctrl_Enable == 0) || (Servo_Enc_Fault != 0))
    {
        if (servo_enabled)
        {
            Motor_Set(MOTOR_SERVO, 0);
        }
        Servo_Err_Deg    = 0.0f;
        Servo_Out        = 0;
        Servo_I_Sum      = 0.0f;        // 失能即清积分：重新使能时不能带着上次的积分冲击
        Servo_U0_Dir     = 0;           // 前馈门也复位，重新使能从"关"开始
        servo_enabled    = 0;
        return;
    }

    err = tar - Servo_Angle_Deg;

    /* ---- 越限保护：实测已超出软限位、且误差仍指向限位方向 -> 本拍不出力 ----
       保证即使因外部原因压到限位，电机也不会继续加力去顶限位结构。 */
    block = (((Servo_Angle_Deg >  SERVO_SOFT_LIMIT_DEG) && (err > 0.0f)) ||
             ((Servo_Angle_Deg < -SERVO_SOFT_LIMIT_DEG) && (err < 0.0f))) ? 1 : 0;

    if (block)
    {
        p_out           = 0.0f;
        u0_out          = 0.0f;
        i_add           = 0.0f;
        Servo_U0_Dir    = 0;            // 越限时关闭前馈门，解除后再开
    }
    else
    {
        abs_err = (err >= 0.0f) ? err : -err;

        /* ---- 摩擦前馈三态滞环：> BRK 开 / <= DB 关 / 中间保持上一拍 ----
           单门限（|err| < DB 就撤 U0）在慢变目标时会每穿一次 DB 翻一次，输出跳 2×U0；
           滞环让门在带内"开住"，输出连续。方向每拍重取 sign(err)，不沿用上一拍
           ——峰值 50°/s 时一拍能走 0.25°，沿用旧方向会出现"满前馈往反方向推" */
        {
            int8 s = (err >= 0.0f) ? 1 : -1;        // 本拍方向，0 归"+"侧

            if (abs_err > SERVO_BRK_DEG)                { Servo_U0_Dir = s; }   // 开门
            else if (abs_err <= SERVO_ERR_DEADBAND_DEG) { Servo_U0_Dir = 0; }   // 关门
            else if (0 != Servo_U0_Dir)                 { Servo_U0_Dir = s; }   // 带内门开着，只更新方向
            /* 带内且门是关的 -> 继续关着（这就是滞环的作用） */
        }

        p_out  = (abs_err < SERVO_ERR_DEADBAND_DEG) ? 0.0f : (Servo_Ctrl_Kp * err);
        u0_out = (0 != Servo_U0_Dir) ? (Servo_Ctrl_U0 * (float)Servo_U0_Dir) : 0.0f;

        /* ---- 积分累加门限：只挡纯噪声 ---- */
        i_add = (abs_err < SERVO_I_DEADBAND_DEG) ? 0.0f : err;
    }

    /* ---- 积分项：|err| > SERVO_I_CLR_BAND_DEG 时清零不累加 ---- */
    if ((err > SERVO_I_CLR_BAND_DEG) || (err < -SERVO_I_CLR_BAND_DEG))
    {
        Servo_I_Sum = 0.0f;
    }
    else
    {
        Servo_I_Sum += i_add;
    }

    i_out = Servo_Ctrl_Ki * Servo_I_Sum;
    if (i_out > SERVO_I_MAX)
    {
        i_out       = SERVO_I_MAX;
        Servo_I_Sum = (Servo_Ctrl_Ki > 0.0f) ? ( SERVO_I_MAX / Servo_Ctrl_Ki) : 0.0f;
    }
    else if (i_out < -SERVO_I_MAX)
    {
        i_out       = -SERVO_I_MAX;
        Servo_I_Sum = (Servo_Ctrl_Ki > 0.0f) ? (-SERVO_I_MAX / Servo_Ctrl_Ki) : 0.0f;
    }

    /* ---- 位置环 + 积分 + 摩擦前馈 + 输出限幅 ----
       p_out   = kp × err         ：经典 Kp（死区内这一拍给 0）
       i_out   = ki × Σerr        ：经典 Ki（背隙、齿条回弹、重力分量）
       u0_out  = u0 × 三态门方向   ：克服静摩擦，门由 BRK/DB 滞环决定，方向跟从 err
       三项的"本拍要不要参与"独立判定 */
    out = p_out + i_out + u0_out;

    if (out >  SERVO_U_MAX) { out =  SERVO_U_MAX; }
    if (out < -SERVO_U_MAX) { out = -SERVO_U_MAX; }

    Servo_Err_Deg = err;
    Servo_Out     = (int16)(SERVO_MOTOR_SIGN * out);
    servo_enabled = 1;

    Motor_Set(MOTOR_SERVO, Servo_Out);
}
