/*******************************************************************************
后轮电机速度闭环控制 - 实现
控制律与参数出处见 Motor_Crtl.h
*******************************************************************************/


#include "zf_common_headfile.h"


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
// 备注信息     进/出调试页时调用，避免把上次的设定与积分带进下一次
//--------------------------------------
void Motor_Crtl_Reset (void)
{
    Motor_LR_Crtl.target     = 0.0f;
    Motor_LR_Crtl.target_app = 0.0f;
    Motor_LR_Crtl.z          = 0.0f;

    Motor_RR_Crtl.target     = 0.0f;
    Motor_RR_Crtl.target_app = 0.0f;
    Motor_RR_Crtl.z          = 0.0f;
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
// 备注信息     顺序固定：门控 -> 取反馈 -> 控制 -> 写输出
//              门控判定必须在控制之前出结论，控制器用的一定是本拍的结果
//--------------------------------------
void Motor_Crtl_Tick (void)
{
    Motor_LR_Crtl.actual = (float)ENC_LR_CNT;
    Motor_RR_Crtl.actual = (float)ENC_RR_CNT;

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

    /* ---- 门控不通过：输出严格 0，并清掉积分与斜坡状态 ----
       否则"被保护停住"期间误差一直累积，一放开门就是一次猛冲 */
    if (0 == Motor_Crtl_Enable)
    {
        p->e          = 0.0f;
        p->z          = 0.0f;
        p->target_app = 0.0f;
        p->u_ff       = 0.0f;
        p->u_cmd      = 0.0f;
        p->out        = 0.0f;
        p->sat        = 0;
        p->enabled    = 0;
        Motor_Set(p->motor_id, 0);
        return;
    }

    r = Motor_Crtl_LimitTarget(p, p->target);

    /* ---- 目标斜坡 + 大跳变清积分 ----
       直接写 target 等于灌阶跃：不处理会 ①前馈瞬间打满 ②积分攒一堆 -> 大超调 */
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
