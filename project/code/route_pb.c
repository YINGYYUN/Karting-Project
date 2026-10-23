/*******************************************************************************
路表回放：Pure Pursuit 外环
速度取参数 pb_v（0 = 自由轮，手推），转向直接给舵角。
*******************************************************************************/


#include "zf_common_headfile.h"
#include <math.h>
#include <stddef.h>
#include "ins_route.h"
#include "ins_odom.h"
#include "route_pb.h"


#define PB_D2R      (0.01745329252f)
#define PB_R2D      (57.29577951f)


/**********************************************************/
/*[S] 内部状态 [S]----------------------------------------*/
/**********************************************************/

static uint8   s_st   = PB_ST_IDLE;
static uint16  s_idx  = 0u;         // 最近点（单调）
static uint16  s_aim  = 0u;         // 前视点
static uint16  s_n    = 0u;         // 开跑时的点数
static uint8   s_done_armed = 0u;   // 1 = 已走过完成阈值点，才允许判到终点
static float   s_elat = 0.0f;
static float   s_ld   = 0.0f;

static float   s_o0x   = 0.0f;      // 开跑那一刻的 odom（只用相对量）
static float   s_o0y   = 0.0f;
static float   s_o0yaw = 0.0f;

static char    s_why[24] = "";

/**********************************************************/
/*----------------------------------------[E] 内部状态 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 内部函数 [S]----------------------------------------*/
/**********************************************************/

static float pb_wrap180 (float a)
{
    while (a >  180.0f) { a -= 360.0f; }
    while (a < -180.0f) { a += 360.0f; }
    return a;
}

static float pb_dist (const ins_wp_t *a, const ins_wp_t *b)
{
    float dx = b->x - a->x;
    float dy = b->y - a->y;

    return sqrtf(dx * dx + dy * dy);
}

static void pb_set_why (const char *w)
{
    uint8 i = 0u;

    if (NULL == w) { s_why[0] = '\0'; return; }

    while ((i < (uint8)(sizeof(s_why) - 1u)) && ('\0' != w[i]))
    {
        s_why[i] = w[i];
        i ++;
    }
    s_why[i] = '\0';
}

//--------------------------------------
// 函数简介     最近点（从上次索引往前搜固定窗口）
//--------------------------------------
static uint16 pb_find_nearest (float px, float py, const ins_wp_t *p)
{
    uint16 i;
    uint16 best  = s_idx;
    uint16 end   = (uint16)(s_idx + PB_NEAR_WIN);
    float  best_d;

    if (end >= s_n) { end = (uint16)(s_n - 1u); }

    {
        float ex = px - p[s_idx].x;
        float ey = py - p[s_idx].y;

        best_d = ex * ex + ey * ey;
    }

    for (i = (uint16)(s_idx + 1u); i <= end; i ++)
    {
        float ex = px - p[i].x;
        float ey = py - p[i].y;
        float d  = ex * ex + ey * ey;

        if (d < best_d) { best_d = d; best = i; }
    }

    return best;
}

//--------------------------------------
// 函数简介     沿弧长往前找前视点
//--------------------------------------
static uint16 pb_find_lookahead (uint16 nearest, const ins_wp_t *p, float ld)
{
    uint16 i;
    float  acc = 0.0f;

    for (i = nearest; (uint16)(i + 1u) < s_n; i ++)
    {
        acc += pb_dist(&p[i], &p[i + 1u]);
        if (acc >= ld) { return (uint16)(i + 1u); }
    }
    return (uint16)(s_n - 1u);
}

//--------------------------------------
// 函数简介     收尾：回中、让出驾驶
//--------------------------------------
static void pb_finish (uint8 st, const char *why)
{
    s_st = st;
    pb_set_why(why);

    Servo_Target_Deg     = 0.0f;
    Head_Crtl_Set_Kp(HEAD_KP_DEFAULT);      // 收回动态增益，别把回放的 k 留给遥控
    Head_Crtl_Set_Enable(0);
    Motor_Crtl_Enable    = 0;       // 自由轮
    Motor_LR_Crtl.target = 0.0f;
    Motor_RR_Crtl.target = 0.0f;
}

/**********************************************************/
/*----------------------------------------[E] 内部函数 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 接口实现 [S]----------------------------------------*/
/**********************************************************/

void route_pb_init (void)
{
    s_st   = PB_ST_IDLE;
    s_idx  = 0u;
    s_aim  = 0u;
    s_n    = 0u;
    s_elat = 0.0f;
    s_ld   = 0.0f;
    s_why[0] = '\0';
}

//--------------------------------------
// 函数简介     启动回放
// 返回参数     uint8	1 = 已启动；0 = 条件不足
//--------------------------------------
uint8 route_pb_start (void)
{
    ins_odom_snap_t o;
    uint16 n = ins_route_get_count();

    if (n < (uint16)PB_MIN_POINTS)
    {
        pb_set_why("few points");
        return 0u;
    }

    ins_odom_snapshot(&o);

    s_o0x   = o.x;
    s_o0y   = o.y;
    s_o0yaw = o.yaw;

    s_n    = n;
    s_idx  = (n > 3u) ? 3u : 0u;
    s_aim  = s_idx;
    s_elat = 0.0f;
    s_ld   = 0.0f;
    s_done_armed = 0u;              // 新起跑：重新计算"走过没有"
    s_why[0] = '\0';

    // 直接给 δ 的通道先把航向环关掉；走航向通道时由 poll 每拍打开
    if (ROUTE_PB_SMODE >= 0.5f) { Head_Crtl_Set_Enable(0); }

    // 回放要打角：手推录制时舵机是失能的，这里自己打开
    Servo_Crtl_Enable = 1;

    s_st = PB_ST_RUN;
    return 1u;
}

void route_pb_stop (const char *why)
{
    if (PB_ST_RUN == s_st) { pb_finish(PB_ST_ABORT, why); }
}

//--------------------------------------
// 函数简介     主循环每拍：位姿 → 前视点 → 舵角 / 速度
//--------------------------------------
void route_pb_poll (void)
{
    const ins_wp_t *p;
    ins_odom_snap_t o;
    float  px, py, pyaw;
    uint16 best, j;

    if (PB_ST_RUN != s_st) { return; }

    // 遥控接管 / 急停 / 失联：让位
    if ((RC_ENGAGE_ON == RC_Engage) || (RC_ENGAGE_ESTOP == RC_Engage) ||
        (RC_ENGAGE_FAILSAFE == RC_Engage))
    {
        pb_finish(PB_ST_ABORT, "rc takeover");
        return;
    }

    p = ins_route_get_points();
    ins_odom_snapshot(&o);

    // 位姿换到路表系（基准 = 开跑那一刻，不用表头的 origin）
    {
        float dx0 = o.x - s_o0x;
        float dy0 = o.y - s_o0y;
        float c   = cosf(s_o0yaw * PB_D2R);
        float sn  = sinf(s_o0yaw * PB_D2R);

        px   = p[0].x + ( dx0 * c + dy0 * sn);
        py   = p[0].y + (-dx0 * sn + dy0 * c);
        pyaw = pb_wrap180(p[0].yaw + (o.yaw - s_o0yaw));
    }

    best  = pb_find_nearest(px, py, p);
    s_idx = best;

    // 闭合表首尾相接：进度没走过阈值点就不许判终点，否则起跑第一拍就 DONE
    if (0u == s_done_armed)
    {
        uint16 need = (uint16)(s_n / 4u);

        if (need < 4u)                 { need = 4u; }
        if (need > 30u)                { need = 30u; }
        if (need > (uint16)(s_n / 2u)) { need = (uint16)(s_n / 2u); }

        if (best >= need) { s_done_armed = 1u; }
    }

    // 完成 / 中止
    {
        float lx = px - p[s_n - 1u].x;
        float ly = py - p[s_n - 1u].y;

        if ((0u != s_done_armed) && (sqrtf(lx * lx + ly * ly) < PB_FIN_M))
        {
            pb_finish(PB_ST_DONE, "done");
            return;
        }
        if (best >= (uint16)(s_n - 1u))
        {
            pb_finish(PB_ST_ABORT, "off course");
            return;
        }
    }

    // 横向误差（正 = 车在参考线右侧）
    {
        float tyaw = p[best].yaw * PB_D2R;

        s_elat = (px - p[best].x) * cosf(tyaw) + (py - p[best].y) * sinf(tyaw);
    }

    // 速度：只认参数；0 = 自由轮（手推），负值 = 倒车
    {
        float v = ROUTE_PB_V;

        if (v < PB_V_MIN) { v = PB_V_MIN; }
        if (v > PB_V_MAX) { v = PB_V_MAX; }

        if ((v < 0.005f) && (v > -0.005f))      // |v| 太小：自由轮
        {
            Motor_Crtl_Enable    = 0;
            Motor_LR_Crtl.target = 0.0f;
            Motor_RR_Crtl.target = 0.0f;
        }
        else
        {
            float mpr  = (ins_m_per_rev > 1.0e-4f) ? ins_m_per_rev : 0.8193f;
            float vrps = v / mpr;

            Motor_Crtl_Enable = 1;
            Motor_Diff_Alloc(vrps, Servo_Angle_Deg,
                             &Motor_LR_Crtl.target, &Motor_RR_Crtl.target);
        }
    }

    // 前视距离
    s_ld = ROUTE_PB_LD;
    if (s_ld < PB_LD_MIN) { s_ld = PB_LD_MIN; }
    if (s_ld > PB_LD_MAX) { s_ld = PB_LD_MAX; }

    j = pb_find_lookahead(best, p, s_ld);
    s_aim = j;

    // 转向：α 正 = 前视点在右
    {
        float dxr = p[j].x - px;
        float dyr = p[j].y - py;
        float pr  = pyaw * PB_D2R;
        float bx  =  dxr * cosf(pr) + dyr * sinf(pr);
        float by  = -dxr * sinf(pr) + dyr * cosf(pr);
        float alpha = atan2f(bx, by);

        if (ROUTE_PB_SMODE < 0.5f)
        {
            // 航向通道：目标航向 + k = 2L/ld（与 Pure Pursuit 等价）
            float k = 2.0f * PB_L_M / s_ld;

            if (k < PB_K_MIN) { k = PB_K_MIN; }
            if (k > PB_K_MAX) { k = PB_K_MAX; }

            Head_Crtl_Set_Kp(k);
            Head_Crtl_Set_Target(pb_wrap180(o.yaw - alpha * PB_R2D));
            Head_Crtl_Set_Enable(1);
            Head_Crtl_Tick();                   // 回放期间由本函数驱动航向环
        }
        else
        {
            // 直接给 δ
            float kappa = 2.0f * sinf(alpha) / s_ld;
            float delta = atanf(PB_L_M * kappa) * PB_R2D;

            if (delta >  PB_DELTA_MAX) { delta =  PB_DELTA_MAX; }
            if (delta < -PB_DELTA_MAX) { delta = -PB_DELTA_MAX; }

            Head_Crtl_Set_Enable(0);
            Servo_Target_Deg = delta;
        }
    }
}

uint8 route_pb_active (void) { return (PB_ST_RUN == s_st) ? 1u : 0u; }
uint8 route_pb_state  (void) { return s_st; }
float route_pb_elat   (void) { return s_elat; }
float route_pb_ld     (void) { return s_ld; }
uint16 route_pb_aim   (void) { return s_aim; }
uint16 route_pb_index (void) { return s_idx; }

const char *route_pb_reason (void) { return s_why; }

/**********************************************************/
/*----------------------------------------[E] 接口实现 [E]*/
/**********************************************************/
