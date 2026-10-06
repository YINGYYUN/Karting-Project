/*******************************************************************************
后轮电机速度闭环控制 - 实现
控制律与参数出处见 Motor_Crtl.h
*******************************************************************************/


#include "zf_common_headfile.h"


// 1ms 全局毫秒计时（定义在 cm7_0_isr.c 的 pit0_ch10_isr），心跳时基
extern volatile uint32 Sys_Tick_Ms;


/**********************************************************/
/*[S] 实例 [S]--------------------------------------------*/
/**********************************************************/

// 左后轮（参数待标定，见 Motor_Crtl.h 顶部说明）
Motor_Crtl_t Motor_LR_Crtl = {
    .motor_id   = MOTOR_LEFT_REAR,
    .inv_K      = MOTOR_CRTL_INV_K_DEFAULT,
    .u0         = MOTOR_CRTL_U0_DEFAULT,
    .k_v        = MOTOR_CRTL_K_V_DEFAULT,
    .k_z        = MOTOR_CRTL_K_Z_DEFAULT,
    .target_max = MOTOR_CRTL_TARGET_MAX,
    .out_max    = MOTOR_CRTL_OUT_MAX,
};

// 右后轮（参数待标定）
Motor_Crtl_t Motor_RR_Crtl = {
    .motor_id   = MOTOR_RIGHT_REAR,
    .inv_K      = MOTOR_CRTL_INV_K_DEFAULT,
    .u0         = MOTOR_CRTL_U0_DEFAULT,
    .k_v        = MOTOR_CRTL_K_V_DEFAULT,
    .k_z        = MOTOR_CRTL_K_Z_DEFAULT,
    .target_max = MOTOR_CRTL_TARGET_MAX,
    .out_max    = MOTOR_CRTL_OUT_MAX,
};

// 闭环总使能：0 = 两路输出强制 0，并清积分与斜坡状态
volatile uint8  Motor_Crtl_Enable = 0;

// 最近一次心跳的时间戳（由 Motor_Crtl_HB_Feed 刷新）
static volatile uint32  motor_hb_ms = 0;

/**********************************************************/
/*----------------------------------------[E] 实例 [E]----*/
/**********************************************************/


/**********************************************************/
/*[S] 内部函数 [S]----------------------------------------*/
/**********************************************************/

//--------------------------------------
// 函数简介     目标限幅
// 参数说明     p	控制器实例
// 参数说明     r	原始目标
// 返回参数     float	限幅后的目标
// 备注信息     上位机/调试页的量程可能大于电机能力，这里做第二道限幅
//--------------------------------------
static float Motor_Crtl_LimitTarget (Motor_Crtl_t *p, float r)
{
    float m = p->target_max;

    if (m < 0.0f) { m = 0.0f; }
    if (r >  m)   { r =  m; }
    if (r < -m)   { r = -m; }

    return r;
}

//--------------------------------------
// 函数简介     喂心跳（由正在监督电机运行的循环每圈调用）
// 参数说明     void
// 使用示例     放在 MOTOR-PID 调试页的两处 while(1) 里
// 备注信息     这里只记录时间戳；是否"活着"由 Motor_Crtl_HB_Alive() 判定
//--------------------------------------
void Motor_Crtl_HB_Feed (void)
{
    motor_hb_ms = Sys_Tick_Ms;
}

//--------------------------------------
// 函数简介     距上次喂心跳的毫秒数
// 返回参数     uint32	毫秒（调试页显示用）
// 备注信息     无符号相减，天然处理 Sys_Tick_Ms 回绕
//--------------------------------------
uint32 Motor_Crtl_HB_Age_Ms (void)
{
    return (uint32)(Sys_Tick_Ms - motor_hb_ms);
}

// 心跳是否还在（1 = 活着）
static uint8 Motor_Crtl_HB_Alive (void)
{
    return (Motor_Crtl_HB_Age_Ms() <= MOTOR_CRTL_HB_TIMEOUT_MS) ? 1 : 0;
}

//--------------------------------------
// 函数简介     编码器脱落 / 堵转判定（单路，每拍调一次）
// 参数说明     p	控制器实例
// 参数说明     delta	本拍编码器增量（ENC_x_CNT）
// 备注信息     前提用"指令侧"的 |out|：在出力、反馈却连续不动 -> 锁存停机。
//              必须在控制之前调用；用到的 out 是上一拍真正给出去的值
//--------------------------------------
static void Motor_Crtl_EncCheck (Motor_Crtl_t *p, int16 delta)
{
    float o = p->out;

    if (0 != p->enc_fault)
    {
        return;                         // 已锁存，等 Motor_Crtl_Reset()
    }

    /* 前提不成立（没使能 / 心跳丢 / 出力还没到阈值）-> 只清计时，不判 */
    if ((0 == Motor_Crtl_Enable) ||
        (0 == Motor_Crtl_HB_Alive()) ||
        ((o < MOTOR_CRTL_ENC_LOST_OUT) && (o > -MOTOR_CRTL_ENC_LOST_OUT)))
    {
        p->zero_ms = 0;
        return;
    }

    /* 在出力，看反馈有没有动 */
    if (0 == delta) { p->zero_ms += MOTOR_CRTL_TICK_MS; }
    else            { p->zero_ms  = 0; }

    if (p->zero_ms >= MOTOR_CRTL_ENC_LOST_MS)
    {
        p->zero_ms   = 0;
        p->enc_fault = 1;
    }
}

/**********************************************************/
/*----------------------------------------[E] 内部函数 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 接口实现 [S]----------------------------------------*/
/**********************************************************/

//--------------------------------------
// 函数简介     复位：清积分、斜坡状态与目标
// 参数说明     void
// 使用示例     Motor_Crtl_Reset();
// 备注信息     进/出调试页时调用，避免把上次的设定与积分带进下一次；
//              同时清编码器脱落锁存（这是该故障唯一的复位途径）
//--------------------------------------
void Motor_Crtl_Reset (void)
{
    Motor_LR_Crtl.target     = 0.0f;
    Motor_LR_Crtl.target_app = 0.0f;
    Motor_LR_Crtl.z          = 0.0f;
    Motor_LR_Crtl.zero_ms    = 0;
    Motor_LR_Crtl.enc_fault  = 0;

    Motor_RR_Crtl.target     = 0.0f;
    Motor_RR_Crtl.target_app = 0.0f;
    Motor_RR_Crtl.z          = 0.0f;
    Motor_RR_Crtl.zero_ms    = 0;
    Motor_RR_Crtl.enc_fault  = 0;
}

//--------------------------------------
// 函数简介     后轮速度闭环初始化
// 参数说明     void
// 使用示例     Motor_Crtl_Init();
// 备注信息     上电默认不使能，需要上层显式把 Motor_Crtl_Enable 置 1
//--------------------------------------
void Motor_Crtl_Init (void)
{
    Motor_Crtl_Enable = 0;
    Motor_Crtl_Reset();
    Motor_SET_Zero_ALL();
}

//--------------------------------------
// 函数简介     10ms 主节拍
// 参数说明     void
// 使用示例     在 PIT 中断里调用 Motor_Crtl_Tick();
// 备注信息     顺序固定：取反馈 -> 保护判定 -> 控制 -> 写输出
//              保护判定必须在控制之前出结论，控制器用的一定是本拍的结果
//--------------------------------------
void Motor_Crtl_Tick (void)
{
    Motor_LR_Crtl.actual = (float)ENC_LR_CNT;
    Motor_RR_Crtl.actual = (float)ENC_RR_CNT;

    // 编码器脱落/堵转：用本拍增量 + 上一拍输出判定，先于控制
    Motor_Crtl_EncCheck(&Motor_LR_Crtl, ENC_LR_CNT);
    Motor_Crtl_EncCheck(&Motor_RR_Crtl, ENC_RR_CNT);

    Motor_Crtl_Update(&Motor_LR_Crtl);
    Motor_Crtl_Update(&Motor_RR_Crtl);
}

//--------------------------------------
// 函数简介     单路控制计算
// 参数说明     p	控制器实例
// 使用示例     Motor_Crtl_Update(&Motor_LR_Crtl);
// 备注信息     通用函数写法（与 PID 基础函数同一风格），不含任何硬件细节
//--------------------------------------
void Motor_Crtl_Update (Motor_Crtl_t *p)
{
    float r;
    float err;
    float u_ff;
    float z_try;
    float u_raw;
    float u_clamped;

    /* ---- 门控不通过：清掉积分与斜坡状态 ----
       三种不通过：总使能关 / 心跳丢失（监督循环卡死）/ 本路编码器脱落锁存。
       否则"被保护停住"期间误差一直累积，一放开门就是一次猛冲。
       注意：输出只在"使能 -> 失能"的那一拍清零一次，之后不再写 Motor_Set，
             否则会和 MOTOR 调试页的手动开环输出互相打架
             （每拍都写 0 的话，手动给 motor1/2 的 PWM 会被立刻抹掉） */
    if ((0 == Motor_Crtl_Enable) ||
        (0 == Motor_Crtl_HB_Alive()) ||
        (0 != p->enc_fault))
    {
        if (p->enabled) // 确保只置0一次状态
        {
            Motor_Set(p->motor_id, 0);
        }
        p->e          = 0.0f;
        p->z          = 0.0f;
        p->target_app = 0.0f;
        p->u_ff       = 0.0f;
        p->u_cmd      = 0.0f;
        p->out        = 0.0f;
        p->sat        = 0;
        p->enabled    = 0;
        return;
    }

    r = Motor_Crtl_LimitTarget(p, p->target);

    /* ---- 目标斜坡 + 大跳变清积分 ----
       直接写 target 等于灌阶跃：不处理会 1.前馈瞬间打满 2.积分攒一堆 -> 大超调 */
    {
        float d   = r - p->target_app;
        float lim = MOTOR_CRTL_SLEW_DEFAULT * MOTOR_CRTL_T_S;

        if (MOTOR_CRTL_INT_CLR_STEP > 0.0f)
        {
            if ((d > MOTOR_CRTL_INT_CLR_STEP) || (d < -MOTOR_CRTL_INT_CLR_STEP))
            {
                p->z = 0.0f;
            }
        }

        if (lim > 0.0f)
        {
            if      (r > p->target_app + lim) { r = p->target_app + lim; }
            else if (r < p->target_app - lim) { r = p->target_app - lim; }
        }
        p->target_app = r;
    }

    err = r - p->actual;

    /* ---- 前馈：线性项 + 摩擦截距（带符号）----
       目标很小时不加截距，否则目标 0 附近输出会在 ±u0 之间抖 */
    u_ff = r * p->inv_K;
    if      (r >  MOTOR_CRTL_FF_DEADBAND) { u_ff += p->u0; }
    else if (r < -MOTOR_CRTL_FF_DEADBAND) { u_ff -= p->u0; }

    /* ---- 积分试探（含限幅）---- */
    z_try = p->z + err * MOTOR_CRTL_T_S;
    if (MOTOR_CRTL_Z_MAX > 0.0f)
    {
        if (z_try >  MOTOR_CRTL_Z_MAX) { z_try =  MOTOR_CRTL_Z_MAX; }
        if (z_try < -MOTOR_CRTL_Z_MAX) { z_try = -MOTOR_CRTL_Z_MAX; }
    }

    u_raw  = u_ff + p->k_v * err + p->k_z * z_try;
    p->sat = 0;

    /* ---- 条件积分抗饱和：已饱和且误差仍往饱和方向推 -> 冻结积分 ---- */
    if (((u_raw > p->out_max) && (err > 0.0f)) || ((u_raw < -p->out_max) && (err < 0.0f)))
    {
        u_raw  = u_ff + p->k_v * err + p->k_z * p->z;       // 用旧 z 重算
        p->sat = 1;
    }
    else
    {
        p->z = z_try;
    }

    p->u_cmd = u_raw;

    /* ---- 输出限幅 ---- */
    u_clamped = u_raw;
    if (u_clamped >  p->out_max) { u_clamped =  p->out_max; }
    if (u_clamped < -p->out_max) { u_clamped = -p->out_max; }

    p->e       = err;
    p->u_ff    = u_ff;
    p->out     = u_clamped;
    p->enabled = 1;

    Motor_Set(p->motor_id, (int16)u_clamped);
}

/**********************************************************/
/*----------------------------------------[E] 接口实现 [E]*/
/**********************************************************/
