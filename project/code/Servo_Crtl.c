/*******************************************************************************
转向角测量与控制 - Servo_Crtl.c
数据来源：绝对值角度编码器（ENC_ABS_ANG，CH1 的 5ms 中断采集，与位置环同拍，见 cm7_0_isr.c）
控制对象：Motor 3（MOTOR_SERVO）
*******************************************************************************/

#include "zf_common_headfile.h"

//============================== 对外数据 ==============================
volatile int16 Servo_Ang_Raw            = 0;                // 当前原始读数 0 ~ 4095
volatile float Servo_Angle_Deg          = 0.0f;             // 当前方向盘角度（度，向右为正）
volatile float Servo_Target_Deg         = 0.0f;             // 目标角度（设定值，度，向右为正）
volatile float Servo_Target_Applied_Deg = 0.0f;             // 实际施加目标（目标软限位夹紧后，度）
volatile float Servo_Err_Deg            = 0.0f;             // 本拍误差（度）
volatile int16 Servo_Out                = 0;                // 本拍实际写入电机的输出
volatile uint8 Servo_Crtl_Enable        = 0;                // 0 = 输出强制 0 并清状态
volatile float Servo_Crtl_Kp            = SERVO_KP_DEFAULT; // 位置环比例增益（参数页/Flash 会覆盖）
volatile float Servo_Crtl_U0            = SERVO_U0_DEFAULT; // 摩擦截距前馈（参数页/Flash 会覆盖）
volatile float Servo_Crtl_Ki            = SERVO_KI_DEFAULT; // 位置环积分增益（参数页/Flash 会覆盖）
volatile float Servo_I_Sum              = 0.0f;             // 积分累加值（deg·拍），只供调试观察
volatile uint8 Servo_Enc_Fault          = 0;                // 1 = 反馈源失效已锁存

//============================== 内部状态 ==============================
static uint8 servo_enabled          = 0;                    // （上一拍）是否为启用状态（用于失能时只清一次输出）
static int8  Servo_U0_Dir           = 0;                    // 摩擦前馈的三态门：+1 / -1 / 0（关）
static uint32 servo_invalid_ms      = 0;                    // 电机在出力、却连续读到无效角度的累计时间

//--------------------------------------
// 函数简介     反馈源失效判定（角度编码器读出无效值）
// 参数说明     void
// 备注信息     前提用"指令侧"的 |Servo_Out|：在出力、反馈却连续读到无效值 -> 锁存停机。
//              必须在门控之前调用，来尽早停机；用到的 Servo_Out 是上一拍真正给出去的值
//--------------------------------------
static void Servo_Crtl_EncCheck (void)
{
    int16 out = Servo_Out;

    if (Servo_Enc_Fault != 0)
    {
        return;                         // 反馈源失效已锁存，等 Servo_Crtl_Reset()解除
    }

    /* 前提不成立（没使能 / 出力还没到阈值）-> 只清零计时，不判决 */
    if ((Servo_Crtl_Enable == 0) ||
        ((out < SERVO_ENC_LOST_OUT) && (out > -SERVO_ENC_LOST_OUT)))
    {
        servo_invalid_ms = 0;
        return;
    }

    /* 在出力，判定角度编码器是否掉线 */
    // 利用SPI设备掉线的固定特征值进行判定
    if ((Servo_Ang_Raw <= SERVO_ENC_RAW_INVALID_LOW) ||
        (Servo_Ang_Raw >= SERVO_ENC_RAW_INVALID_HIGH))
    {
        servo_invalid_ms += SERVO_TICK_MS;
    }
    // 未掉线
    else
    {
        servo_invalid_ms = 0;
    }

    /* 掉线时间到达阈值，触发停机 */
    if (servo_invalid_ms >= SERVO_ENC_LOST_MS)
    {
        servo_invalid_ms = 0;
        Servo_Enc_Fault  = 1;
    }
}

//--------------------------------------
// 函数简介     重置状态 （失能 / 清目标 / 误差 / 输出 / 故障锁存）
// 参数说明     void
// 使用示例     Servo_Crtl_Reset();
// 备注信息     不清 servo_enabled（有意，留给 Tick 补硬件归零）
//             不清 Servo_Crtl_Kp/U0（归参数页管）
//--------------------------------------
void Servo_Crtl_Reset (void)
{
    Servo_Crtl_Enable        = 0;
    Servo_Target_Deg         = 0.0f;
    Servo_Target_Applied_Deg = 0.0f;
    Servo_Err_Deg            = 0.0f;
    Servo_Out                = 0;
    Servo_I_Sum              = 0.0f;    // 积分累加值一起清，重新使能从 0 开始
    Servo_U0_Dir             = 0;       // 前馈三态门一起清
    Servo_Enc_Fault          = 0;
    servo_invalid_ms         = 0;
}

//--------------------------------------
// 函数简介     转向控制初始化
// 参数说明     void
// 使用示例     Servo_Crtl_Init();
// 备注信息     上电后调用一次
//--------------------------------------
void Servo_Crtl_Init (void)
{
    Servo_Crtl_Reset();
    Motor_Set(MOTOR_SERVO, 0);
}

//--------------------------------------
// 函数简介     5ms 主节拍
// 参数说明     void
// 使用示例     在 PIT 中断里调用 Servo_Crtl_Tick();
// 备注信息     顺序：角度换算 -> 反馈失效判定 -> 门控 -> 目标处理 -> 位置环 -> 限幅 -> 输出
//              角度换算是测量量，始终更新，与舵机闭环使能无关
//              ★ err 在本函数里**始终保持真值**：死区与越限保护只决定"本拍哪一项不参与"，
//                不再把 err 抹成 0（抹 0 会把积分一起杀掉，Servo_Err_Deg 也会变假值）
//              数据源 ENC_ABS_ANG 由本函数所在的同一个中断（CH1, 5ms）在其之前采集，
//              本拍采、本拍用，反馈零滞后 —— 不要再把采集拆到别的通道去
//--------------------------------------
void Servo_Crtl_Tick (void)
{
    float tar_deg;  // 本函数内的舵机目标角度（度）
    float err;      // 角度误差（度），**本拍真值，不再被死区抹成 0**
    float ae;       // |err|
    float u_p;      // 本拍 Kp 项输出
    float u0_term;  // 本拍摩擦前馈项输出
    float i_add;    // 本拍累加进积分的量（0 = 本拍不累加）
    float u;        // 计算中间变量
    float u_i;      // 本拍积分项输出（u，已限幅到 SERVO_I_MAX）
    uint8 block;    // 1 = 越限保护生效，本拍三项都不出力

    /* ---- 角度换算：原始读数 -> 方向盘角度（向右为正）----
       放在门控之前：闭环未启用时角度照常更新，供显示与上层使用 */
    Servo_Ang_Raw   = ENC_ABS_ANG;
    Servo_Angle_Deg = SERVO_DIR_SIGN * ((float)ENC_ABS_ANG - SERVO_ENC_MID) / SERVO_CNT_PER_DEG;

    /* ---- 反馈源失效判定：用本拍原始读数 + 上一拍输出，先于门控 ---- */
    Servo_Crtl_EncCheck();

    /* ---- 目标处理：只做软限位夹紧，不做目标斜坡 ----
       软限位比物理限位留 SERVO_SOFT_MARGIN_DEG 余量，保证控制器不会主动把方向盘顶向限
       位结构。
       夹紧是兜底：各设置入口（调试页按键 / 遥控换算）已按软限位限幅，正常状态下不会真的
       夹到。
       目标阶跃时不限速率：误差一次性给足，Kp·err 直接顶到输出限幅 -> 满 PWM、最快到位。
       代价是没有减速过程，到位附近靠背隙/摩擦自然停住。
       放在门控之前：让"实际施加目标"始终对外可见（Servo_Target_Applied_Deg），与是否
       使能无关。 */
    // 手动实测限位读数重复性波动约 ±1.2°，余量需要留出裕度
    tar_deg = Servo_Target_Deg;
    if (tar_deg >  SERVO_SOFT_LIMIT_DEG) { tar_deg =  SERVO_SOFT_LIMIT_DEG; }
    if (tar_deg < -SERVO_SOFT_LIMIT_DEG) { tar_deg = -SERVO_SOFT_LIMIT_DEG; }
    Servo_Target_Applied_Deg = tar_deg;

    /* ---- 门控不通过：清状态，并只在"使能 -> 失能"那一拍清零输出 ----
       两种不通过：总使能关 / 反馈源失效锁存。
       失能后不再每拍写电机，否则会和 MOTOR 调试页的手动开环输出互相打架 */
    if ((Servo_Crtl_Enable == 0) || (Servo_Enc_Fault != 0))
    {
        if (servo_enabled)
        {
            Motor_Set(MOTOR_SERVO, 0);
        }
        Servo_Err_Deg    = 0.0f;
        Servo_Out        = 0;
        Servo_I_Sum      = 0.0f;    // 失能即清积分：重新使能时不能带着上次的积分冲击
        Servo_U0_Dir     = 0;       // 前馈门也复位，重新使能从"关"开始
        servo_enabled    = 0;
        return;
    }

    err = tar_deg - Servo_Angle_Deg;

    /* ---- 越限保护：实测已超出软限位、且误差仍指向限位方向 -> 本拍不出力 ----
       保证即使因外部原因压到限位，电机也不会继续加力去顶限位结构。
       ★ 这里**不再把 err 抹成 0**：抹 0 会连带杀掉积分（原来的 i_sum += err 吃到的是 0），
         而且 Servo_Err_Deg 显示的是假值。改成"三项都不参与"，err 保持真值。 */
    block = (((Servo_Angle_Deg >  SERVO_SOFT_LIMIT_DEG) && (err > 0.0f)) ||
             ((Servo_Angle_Deg < -SERVO_SOFT_LIMIT_DEG) && (err < 0.0f))) ? 1 : 0;

    if (block)
    {
        u_p      = 0.0f;
        u0_term  = 0.0f;
        i_add    = 0.0f;
        Servo_U0_Dir = 0;               // 越限时把前馈门也关掉，解除后再重新开
    }
    else
    {
        ae = (err >= 0.0f) ? err : -err;

        /* ---- 摩擦前馈三态滞环：> BRK 开 / <= DB 关 / 中间保持上一拍 ----
           单门限（|err| < DB 就撤 U0）在慢变目标时会每穿一次 DB 翻一次，输出跳 2×U0；
           滞环让门在带内"开住"，输出连续。方向每拍重取 sign(err)，不沿用上一拍
           ——峰值 50°/s 时一拍能走 0.25°，沿用旧方向会出现"满前馈往反方向推" */
        {
            int8 s = (err >= 0.0f) ? 1 : -1;        // 本拍方向，0 归"+"侧

            if (ae > SERVO_BRK_DEG)                 { Servo_U0_Dir = s; }   // 开门
            else if (ae <= SERVO_ERR_DEADBAND_DEG)  { Servo_U0_Dir = 0; }   // 关门
            else if (0 != Servo_U0_Dir)             { Servo_U0_Dir = s; }   // 带内门开着，只更新方向
            /* 带内且门是关的 -> 继续关着（这就是滞环的作用） */
        }

        u_p     = (ae < SERVO_ERR_DEADBAND_DEG) ? 0.0f : (Servo_Crtl_Kp * err);
        u0_term = (0 != Servo_U0_Dir) ? (Servo_Crtl_U0 * (float)Servo_U0_Dir) : 0.0f;

        /* ---- 积分累加门限：只挡纯噪声 ----
           门限内的误差不累加，但 err 本身仍在，所以死区里积分不会被"清零"式地停掉，
           小误差才有机会攒成输出、接替 U0 的摩擦补偿职责 */
        i_add = (ae < SERVO_I_DEADBAND_DEG) ? 0.0f : err;
    }

    /* ---- 积分项：|err| > SERVO_I_CLR_BAND_DEG 时直接清零、不累加 ----
       用的是**真实 err**（不再是被死区改过的值），判据因此更准 */
    if ((err > SERVO_I_CLR_BAND_DEG) || (err < -SERVO_I_CLR_BAND_DEG))
    {
        Servo_I_Sum = 0.0f;
    }
    else
    {
        Servo_I_Sum += i_add;
    }

    u_i = Servo_Crtl_Ki * Servo_I_Sum;
    if (u_i > SERVO_I_MAX)
    {
        u_i         = SERVO_I_MAX;
        Servo_I_Sum = (Servo_Crtl_Ki > 0.0f) ? ( SERVO_I_MAX / Servo_Crtl_Ki) : 0.0f;
    }
    else if (u_i < -SERVO_I_MAX)
    {
        u_i         = -SERVO_I_MAX;
        Servo_I_Sum = (Servo_Crtl_Ki > 0.0f) ? (-SERVO_I_MAX / Servo_Crtl_Ki) : 0.0f;
    }

    /* ---- 位置环 + 积分 + 摩擦前馈 + 输出限幅 ----
       u_p     = kp × err         ：经典 Kp（死区内这一拍给 0）
       u_i     = ki × Σerr        ：消除纯 P 留下的稳态误差（背隙、齿条回弹、重力分量）
       u0_term = u0 × 三态门方向  ：克服静摩擦，门由 BRK/DB 滞环决定，方向跟 err
       三项都是"本拍要不要参与"各自判过，err 本身始终保持真值 */
    u = u_p + u_i + u0_term;

    if (u >  SERVO_U_MAX) { u =  SERVO_U_MAX; }
    if (u < -SERVO_U_MAX) { u = -SERVO_U_MAX; }

    Servo_Err_Deg = err;
    Servo_Out     = (int16)(SERVO_MOTOR_SIGN * u);
    servo_enabled = 1;

    Motor_Set(MOTOR_SERVO, Servo_Out);
}
