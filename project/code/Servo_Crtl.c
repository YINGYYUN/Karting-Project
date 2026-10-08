/*******************************************************************************
转向角测量与控制 - 实现
标定参数与控制器说明见 Servo_Crtl.h
*******************************************************************************/

#include "zf_common_headfile.h"

//============================== 对外数据 ==============================
volatile int16 Servo_Ang_Raw        = 0;                // 当前原始读数 0 ~ 4095
volatile float Servo_Angle_Deg      = 0.0f;             // 当前方向盘角度（度，向右为正）
volatile float Servo_Target_Deg     = 0.0f;             // 目标角度（度，向右为正）
volatile float Servo_Err_Deg        = 0.0f;             // 本拍误差（度）
volatile int16 Servo_Out            = 0;                // 本拍实际写入电机的输出
volatile uint8 Servo_Crtl_Enable    = 0;                // 0 = 输出强制 0 并清状态
volatile float Servo_Crtl_Kp        = SERVO_KP_DEFAULT; // 位置环比例增益（参数页/Flash 会覆盖）
volatile float Servo_Crtl_U0        = SERVO_U0_DEFAULT; // 摩擦截距前馈（参数页/Flash 会覆盖）
volatile uint8 Servo_Enc_Fault      = 0;                // 1 = 反馈源失效已锁存

//============================== 内部状态 ==============================
static float servo_target_applied   = 0.0f;             // 目标斜坡的上一次实际施加目标
static uint8 servo_enabled          = 0;                // 上一拍是否为启用状态（用于失能时只清一次输出）
static uint32 servo_invalid_ms      = 0;                // 在出力、却连续读到无效角度的累计时间

//--------------------------------------
// 函数简介     反馈源失效判定（角度编码器读到无效值）
// 参数说明     void
// 备注信息     前提用"指令侧"的 |Servo_Out|：在出力、反馈却连续读到无效值 -> 锁存停机。
//              必须在门控之前调用；用到的 Servo_Out 是上一拍真正给出去的值
//--------------------------------------
static void Servo_Crtl_EncCheck (void)
{
    float o = (float)Servo_Out;

    if (0 != Servo_Enc_Fault)
    {
        return;                         // 已锁存，等 Servo_Crtl_Reset()
    }

    /* 前提不成立（没使能 / 出力还没到阈值）-> 只清计时，不判 */
    if ((0 == Servo_Crtl_Enable) ||
        ((o < SERVO_ENC_LOST_OUT) && (o > -SERVO_ENC_LOST_OUT)))
    {
        servo_invalid_ms = 0;
        return;
    }

    /* 在出力，看角度读数是不是"掉线特征值" */
    if ((Servo_Ang_Raw <= SERVO_ENC_RAW_INVALID_LOW) ||
        (Servo_Ang_Raw >= SERVO_ENC_RAW_INVALID_HIGH))
    {
        servo_invalid_ms += SERVO_TICK_MS;
    }
    else
    {
        servo_invalid_ms = 0;
    }

    if (servo_invalid_ms >= SERVO_ENC_LOST_MS)
    {
        servo_invalid_ms = 0;
        Servo_Enc_Fault  = 1;
    }
}

//--------------------------------------
// 函数简介     清目标 / 斜坡 / 误差 / 输出
// 参数说明     void
// 使用示例     Servo_Crtl_Reset();
// 备注信息     进/出调试页时调用，避免把上次的设定带进下一次；
//              同时清反馈源失效锁存（这是该故障唯一的复位途径）
//--------------------------------------
void Servo_Crtl_Reset (void)
{
    Servo_Target_Deg = 0.0f;
    Servo_Err_Deg    = 0.0f;
    Servo_Out        = 0;
    servo_target_applied = 0.0f;
    Servo_Enc_Fault  = 0;
    servo_invalid_ms = 0;
}

//--------------------------------------
// 函数简介     转向控制初始化
// 参数说明     void
// 使用示例     Servo_Crtl_Init();
// 备注信息     上电默认不使能，需要上层显式把 Servo_Crtl_Enable 置 1
//--------------------------------------
void Servo_Crtl_Init (void)
{
    Servo_Crtl_Enable = 0;
    Servo_Crtl_Reset();
    Motor_Set(MOTOR_SERVO, 0);
}

//--------------------------------------
// 函数简介     10ms 主节拍
// 参数说明     void
// 使用示例     在 PIT 中断里调用 Servo_Crtl_Tick();
// 备注信息     顺序：角度换算 -> 反馈失效判定 -> 门控 -> 目标处理 -> 位置环 -> 限幅 -> 输出
//              角度换算是测量量，与使能无关，始终更新
//              数据源 ENC_ABS_ANG 由同一中断的编码器采集段刷新，须在它之后调用
//--------------------------------------
void Servo_Crtl_Tick (void)
{
    float r;
    float err;
    float u;

    /* ---- 角度换算：原始读数 -> 方向盘角度（向右为正）----
       放在门控之前：闭环未启用时角度照常更新，供显示与上层使用 */
    Servo_Ang_Raw   = ENC_ABS_ANG;
    Servo_Angle_Deg = SERVO_DIR_SIGN * ((float)ENC_ABS_ANG - SERVO_ENC_MID) / SERVO_CNT_PER_DEG;

    /* ---- 反馈源失效判定：用本拍原始读数 + 上一拍输出，先于门控 ---- */
    Servo_Crtl_EncCheck();

    /* ---- 门控不通过：清状态，并只在"使能 -> 失能"那一拍清零输出 ----
       两种不通过：总使能关 / 反馈源失效锁存。
       失能后不再每拍写电机，否则会和 MOTOR 调试页的手动开环输出互相打架 */
    if ((0 == Servo_Crtl_Enable) || (0 != Servo_Enc_Fault))
    {
        if (servo_enabled)
        {
            Motor_Set(MOTOR_SERVO, 0);
        }
        Servo_Err_Deg    = 0.0f;
        Servo_Out        = 0;
        servo_target_applied = 0.0f;
        servo_enabled    = 0;
        return;
    }

    /* ---- 目标处理：软限位夹紧 + 变化率限制 ----
       软限位比物理限位留 SERVO_SOFT_MARGIN_DEG 余量，保证控制器不会主动把方向盘
       顶向限位结构 */
    // 手动实测限位读数重复性波动约 ±1.2°，余量需要留出裕度
    r = Servo_Target_Deg;
    if (r >  SERVO_SOFT_LIMIT_DEG) { r =  SERVO_SOFT_LIMIT_DEG; }
    if (r < -SERVO_SOFT_LIMIT_DEG) { r = -SERVO_SOFT_LIMIT_DEG; }
    {
        float lim = SERVO_SLEW_DEG_PER_S * SERVO_T_S;
        if (r > servo_target_applied + lim) { r = servo_target_applied + lim; }
        if (r < servo_target_applied - lim) { r = servo_target_applied - lim; }
        servo_target_applied = r;
    }

    err = r - Servo_Angle_Deg;

    /* ---- 越限保护：实测已超出软限位、且误差仍指向限位方向 -> 不再出力 ----
       保证即使因外部原因压到限位，电机也不会继续加力去顶限位结构 */
    if (((Servo_Angle_Deg >  SERVO_SOFT_LIMIT_DEG) && (err > 0.0f)) ||
        ((Servo_Angle_Deg < -SERVO_SOFT_LIMIT_DEG) && (err < 0.0f)))
    {
        err = 0.0f;
    }

    /* ---- 误差死区：抑制静止抖动 ---- */
    if ((err < SERVO_ERR_DEADBAND_DEG) && (err > -SERVO_ERR_DEADBAND_DEG))
    {
        err = 0.0f;
    }

    /* ---- 位置环 + 摩擦前馈 + 输出限幅 ----
       u = kp × err   : 细调
       u0 × sign(err) : 克服静摩擦与减速箱阻力
       err 已经过死区与越限保护，等于 0 时 u0 也不加，静止不会抖 */
    u = Servo_Crtl_Kp * err;
    if      (err > 0.0f) { u +=  Servo_Crtl_U0; }
    else if (err < 0.0f) { u += -Servo_Crtl_U0; }

    if (u >  SERVO_U_MAX) { u =  SERVO_U_MAX; }
    if (u < -SERVO_U_MAX) { u = -SERVO_U_MAX; }

    Servo_Err_Deg = err;
    Servo_Out     = (int16)(SERVO_MOTOR_SIGN * u);
    servo_enabled = 1;

    Motor_Set(MOTOR_SERVO, Servo_Out);
}
