/*******************************************************************************
后轮电机速度闭环控制 - 实现
控制律与参数出处见 Motor_Ctrl.h
*******************************************************************************/


#include "zf_common_headfile.h"
#include <math.h>


/**********************************************************/
/*[S] 实例 [S]--------------------------------------------*/
/**********************************************************/

// 左后轮（参数待标定，见 Motor_Ctrl.h 顶部说明）
Motor_Ctrl_t Motor_LR_Ctrl = {
    .motor_id   = MOTOR_LEFT_REAR,
    .inv_K      = MOTOR_CTRL_INV_K_DEFAULT,
    .u0         = MOTOR_CTRL_U0_DEFAULT,
    .k_v        = MOTOR_CTRL_K_V_DEFAULT,
    .k_z        = MOTOR_CTRL_K_Z_DEFAULT,
    .target_max = MOTOR_CTRL_TARGET_MAX,
    .out_max    = MOTOR_CTRL_OUT_MAX,
};

// 右后轮（参数待标定）
Motor_Ctrl_t Motor_RR_Ctrl = {
    .motor_id   = MOTOR_RIGHT_REAR,
    .inv_K      = MOTOR_CTRL_INV_K_DEFAULT,
    .u0         = MOTOR_CTRL_U0_DEFAULT,
    .k_v        = MOTOR_CTRL_K_V_DEFAULT,
    .k_z        = MOTOR_CTRL_K_Z_DEFAULT,
    .target_max = MOTOR_CTRL_TARGET_MAX,
    .out_max    = MOTOR_CTRL_OUT_MAX,
};

// 闭环总使能：0 = 两路输出强制 0，并清积分与斜坡状态
volatile uint8  Motor_Ctrl_Enable = 0;

// 最近一次心跳的时间戳（由 Motor_Ctrl_HB_Feed 刷新）
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
static float Motor_Ctrl_LimitTarget (Motor_Ctrl_t *p, float r)
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
// 备注信息     这里只记录时间戳；是否"活着"由 Motor_Ctrl_HB_Alive() 判定
//--------------------------------------
void Motor_Ctrl_HB_Feed (void)
{
    motor_hb_ms = Sys_Tick_Ms;
}

//--------------------------------------
// 函数简介     距上次喂心跳的毫秒数
// 返回参数     uint32	毫秒（调试页显示用）
// 备注信息     无符号相减，天然处理 Sys_Tick_Ms 回绕
//--------------------------------------
uint32 Motor_Ctrl_HB_Age_Ms (void)
{
    return (uint32)(Sys_Tick_Ms - motor_hb_ms);
}

// 心跳是否还在（1 = 活着）
static uint8 Motor_Ctrl_HB_Alive (void)
{
    return (Motor_Ctrl_HB_Age_Ms() <= MOTOR_CTRL_HB_TIMEOUT_MS) ? 1 : 0;
}

//--------------------------------------
// 函数简介     编码器脱落 / 堵转判定（单路，每拍调一次）
// 参数说明     p	控制器实例
// 参数说明     delta	本拍编码器增量窗口均值（ENC_x_CNT_F，浮点原值）
// 备注信息     前提用"指令侧"的 |out|：在出力、反馈却连续不动 -> 锁存停机。
//              必须在控制之前调用；用到的 out 是上一拍真正给出去的值
//              用浮点原值判定：整数版在窗口均值不足 1 计数时会被截断成 0，
//              把"转得很慢"误判成"完全没动"
//--------------------------------------
static void Motor_Ctrl_EncCheck (Motor_Ctrl_t *p, float delta)
{
    float o = p->out;

    if (0 != p->enc_fault)
    {
        /* 软锁存：冷却到点就自动放行一次重试（见 Motor_Ctrl.h 的说明）。
           ★ 只有在"真的重试过 RETRY_MAX 次"之后才变成永久锁存（Reset 才清）。
           重试前输出已经被 Update 清零、积分与斜坡也复位过，所以这是"重新使能一次"，
           不是"带着错误积分继续冲"。 */
        if ((p->retry < MOTOR_CTRL_ENC_RETRY_MAX) &&
            ((uint32)(Sys_Tick_Ms - p->fault_ms) >= MOTOR_CTRL_ENC_RETRY_MS))
        {
            p->retry    ++;
            p->enc_fault = 0;
            p->zero_ms   = 0;
        }
        return;
    }

    /* 前提不成立（没使能 / 心跳丢 / 出力还没到阈值）-> 只清计时，不判 */
    if ((0 == Motor_Ctrl_Enable) ||
        (0 == Motor_Ctrl_HB_Alive()) ||
        ((o < MOTOR_CTRL_ENC_LOST_OUT) && (o > -MOTOR_CTRL_ENC_LOST_OUT)))
    {
        p->zero_ms = 0;
        return;
    }

    /* 在出力，看反馈有没有动（窗口内一个脉冲都没有才会精确为 0.0f） */
    if (0.0f == delta) { p->zero_ms += MOTOR_CTRL_TICK_MS; }
    else               { p->zero_ms  = 0; }

    if (p->zero_ms >= MOTOR_CTRL_ENC_LOST_MS)
    {
        p->zero_ms   = 0;
        p->enc_fault = 1;
        p->fault_ms  = Sys_Tick_Ms;

        // 串口留一行痕迹：这个故障原本是"看不见"的，现场只会觉得"那个轮子慢了"
        printf("[MOT] ENC LOST: motor %u (重试 %u/%u)\r\n",
               (unsigned)p->motor_id, (unsigned)p->retry, (unsigned)MOTOR_CTRL_ENC_RETRY_MAX);
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
// 使用示例     Motor_Ctrl_Reset();
// 备注信息     进/出调试页时调用，避免把上次的设定与积分带进下一次；
//              同时清编码器脱落锁存（这是该故障唯一的复位途径）
//--------------------------------------
void Motor_Ctrl_Reset (void)
{
    Motor_LR_Ctrl.target      = 0.0f;
    Motor_LR_Ctrl.target_prev = 0.0f;
    Motor_LR_Ctrl.target_app  = 0.0f;
    Motor_LR_Ctrl.z           = 0.0f;
    Motor_LR_Ctrl.zero_ms     = 0;
    Motor_LR_Ctrl.enc_fault   = 0;
    Motor_LR_Ctrl.retry       = 0;      // 重试配额也归零：人工复位 = 重新给满配额
    Motor_LR_Ctrl.fault_ms    = 0;

    Motor_RR_Ctrl.target      = 0.0f;
    Motor_RR_Ctrl.target_prev = 0.0f;
    Motor_RR_Ctrl.target_app  = 0.0f;
    Motor_RR_Ctrl.z           = 0.0f;
    Motor_RR_Ctrl.zero_ms     = 0;
    Motor_RR_Ctrl.enc_fault   = 0;
    Motor_RR_Ctrl.retry       = 0;
    Motor_RR_Ctrl.fault_ms    = 0;
}

//--------------------------------------
// 函数简介     后轮速度闭环初始化
// 参数说明     void
// 使用示例     Motor_Ctrl_Init();
// 备注信息     上电默认不使能，需要上层显式把 Motor_Ctrl_Enable 置 1
//--------------------------------------
void Motor_Ctrl_Init (void)
{
    Motor_Ctrl_Enable = 0;
    Motor_Ctrl_Reset();
    Motor_SET_Zero_ALL();
}

//--------------------------------------
// 函数简介     5ms 主节拍
// 参数说明     void
// 使用示例     在 PIT 中断里调用 Motor_Ctrl_Tick();
// 备注信息     顺序固定：取反馈 -> 保护判定 -> 控制 -> 写输出
//              保护判定必须在控制之前出结论，控制器用的一定是本拍的结果
//--------------------------------------
void Motor_Ctrl_Tick (void)
{
    // 编码器计数（每拍增量的窗口均值的浮点原值）-> 转速 rps：
    // 每转 MOTOR_CTRL_CPR 个计数，拍长 MOTOR_CTRL_T_S。
    // 必须取浮点版：低速时整数版会把不足 1 计数的均值截断成 0，反馈恒 0 -> 积分失去回复力
    Motor_LR_Ctrl.actual = ENC_LR_CNT_F / (MOTOR_CTRL_CPR * MOTOR_CTRL_T_S);
    Motor_RR_Ctrl.actual = ENC_RR_CNT_F / (MOTOR_CTRL_CPR * MOTOR_CTRL_T_S);

    // 编码器脱落/堵转：用本拍增量 + 上一拍输出判定，先于控制
    Motor_Ctrl_EncCheck(&Motor_LR_Ctrl, ENC_LR_CNT_F);
    Motor_Ctrl_EncCheck(&Motor_RR_Ctrl, ENC_RR_CNT_F);

    Motor_Ctrl_Update(&Motor_LR_Ctrl);
    Motor_Ctrl_Update(&Motor_RR_Ctrl);
}

//--------------------------------------
// 函数简介     单路控制计算
// 参数说明     p	控制器实例
// 使用示例     Motor_Ctrl_Update(&Motor_LR_Ctrl);
// 备注信息     通用函数写法（与 PID 基础函数同一风格），不含任何硬件细节
//--------------------------------------
void Motor_Ctrl_Update (Motor_Ctrl_t *p)
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
    if ((0 == Motor_Ctrl_Enable) ||
        (0 == Motor_Ctrl_HB_Alive()) ||
        (0 != p->enc_fault))
    {
        if (p->enabled) // 确保只置0一次状态
        {
            Motor_Set(p->motor_id, 0);
        }
        p->e          = 0.0f;
        p->z          = 0.0f;
        p->target_prev = 0.0f;      // 与 target_app 同步归零
        p->target_app = 0.0f;
        p->u_ff       = 0.0f;
        p->u_cmd      = 0.0f;
        p->out        = 0.0f;
        p->sat        = 0;
        p->enabled    = 0;
        return;
    }

    r = Motor_Ctrl_LimitTarget(p, p->target);

    /* ---- 目标斜坡 + 指令大跳变清积分 ----
       直接写 target 等于灌阶跃：不处理会 1.前馈瞬间打满 2.积分攒一堆 -> 大超调
       跳变判据用相邻两拍指令之差，不能用 target_app（斜坡期间会每拍都清） */
    {
        float r_in = r;                                 // 本拍指令（斜坡限幅之前）
        float d    = r_in - p->target_prev;             // 与上一拍指令比
        float lim  = MOTOR_CTRL_SLEW * MOTOR_CTRL_T_S;

        if (MOTOR_CTRL_INT_CLR_STEP > 0.0f)
        {
            if ((d > MOTOR_CTRL_INT_CLR_STEP) || (d < -MOTOR_CTRL_INT_CLR_STEP))
            {
                p->z = 0.0f;
            }
        }

        p->target_prev = r_in;

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
    if      (r >  MOTOR_CTRL_FF_DEADBAND) { u_ff += p->u0; }
    else if (r < -MOTOR_CTRL_FF_DEADBAND) { u_ff -= p->u0; }

    /* ---- 积分试探（含限幅）---- */
    z_try = p->z + err * MOTOR_CTRL_T_S;
    if (MOTOR_CTRL_Z_MAX > 0.0f)
    {
        if (z_try >  MOTOR_CTRL_Z_MAX) { z_try =  MOTOR_CTRL_Z_MAX; }
        if (z_try < -MOTOR_CTRL_Z_MAX) { z_try = -MOTOR_CTRL_Z_MAX; }
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

    // u 是归一化 ±1，只在这一刻换算成 duty 写进驱动
    Motor_Set(p->motor_id, (int16)(u_clamped * (float)PWM_DUTY_MAX));
}

/**********************************************************/
/*----------------------------------------[E] 接口实现 [E]*/
/**********************************************************/

//--------------------------------------
// 函数简介     电子差速分配（阿克曼）
// 参数说明     v_rps       两轮平均目标转速 rps
//              delta_deg   实测方向盘角度（度，向右为正）
//              L_m         有效轴距 m（调用方给：回放传 ROUTE_PB_L 参数）
//              vl / vr     输出：左 / 右轮目标 rps
// 使用示例     Motor_Diff_Alloc(tgt, Servo_Angle_Deg, ROUTE_PB_L, &Motor_LR_Ctrl.target, &Motor_RR_Ctrl.target);
// 备注信息     右转（delta > 0）左轮在外侧 -> 快；前进 / 倒车同一公式。
//              |r| < MOTOR_DIFF_MAX < 1，保证两轮恒与 v 同向。
//              软限位 ±28° 不跨编码器 0 点，直接用原始偏差即可。
//              ★ 轴距由参数传入（原来内用宏 MOTOR_DIFF_L，与回放的 2L/ld 各用一份）
//              ★ 实测角先过一阶低通（α=0.25），与队友 spd_diff_alloc() 一致：
//                编码器有量化噪声，直接进 tan() 会让差速跟着抖
//--------------------------------------
static float s_diff_dm = 0.0f;      // 实测舵角低通状态（度）

void Motor_Diff_Alloc (float v_rps, float delta_deg, float L_m, float *vl, float *vr)
{
    *vl = v_rps;
    *vr = v_rps;

#if MOTOR_DIFF_ENABLE
    {
        float k;            // 曲率 1/m
        float r;            // 单侧相对偏差

        /* 倒车/静止：不做分配，但**让低通跟上**当前角，免得下次前进第一拍跳变
           （队友 spd_diff_alloc() 里也是这个处理） */
        if ((v_rps < MOTOR_DIFF_V_MIN) && (v_rps > -MOTOR_DIFF_V_MIN))
        {
            s_diff_dm = delta_deg;
            return;
        }

        if (L_m < MOTOR_DIFF_L_MIN) { return; }             // 轴距没配好：宁可不差速

        s_diff_dm = (MOTOR_DIFF_LPF * delta_deg) + ((1.0f - MOTOR_DIFF_LPF) * s_diff_dm);

        k = tanf(s_diff_dm * MOTOR_DIFF_DEG2RAD) / L_m;
        r = k * MOTOR_DIFF_W * 0.5f;

        if (r >  MOTOR_DIFF_MAX) { r =  MOTOR_DIFF_MAX; }
        if (r < -MOTOR_DIFF_MAX) { r = -MOTOR_DIFF_MAX; }

        *vl = v_rps * (1.0f + r);
        *vr = v_rps * (1.0f - r);
    }
#endif
}
