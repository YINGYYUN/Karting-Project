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
static uint8   s_rev  = 0u;         // 1 = 当前在倒车段（依据 p[best] 的 REV 位 / v 符号）
static uint8   s_tail_rev = 0u;     // 1 = **表尾是倒车段**（起跑时读最后几个点判一次）
static uint8   s_tail_stop = 0u;    // 1 = 正在"倒车收尾刹停"（与倒车→前进过渡共用）
static uint32  s_tail_ms  = 0u;     // 刹停开始时刻
static uint8   s_dir_hold = 0u;     // 1 = 倒车→前进的过渡等待中（等车停住再开外环）
static uint8   s_rev_flipped = 0u;  // 1 = 本次回放已经把点序倒过来了（退出时必须复原）

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

//--------------------------------------
// 函数简介     "这个点是不是倒车点"（**唯一判据**，所有地方都走它）
// 参数说明     q 点
// 返回参数     1 = 倒车点
// 备注信息     ★ 倒序回放（s_rev_flipped = 1）时**永远返回 0**：
//                 点序倒过来之后车是**正着**开回起点的，REV 位只是"这张表当初是倒着录的"
//                 这个出身标记，不再代表新方向上的行驶方向。
//                 不在这里屏蔽的话，整张表都会被当倒车段处理（车会一路倒着开）。
//--------------------------------------
static uint8 pb_point_is_rev (const ins_wp_t *q)
{
    if (0u != s_rev_flipped) { return 0u; }

    return ((0u != (q->flags & (uint16)INS_WP_FL_REV)) || (q->v < -PB_REV_EPS)) ? 1u : 0u;
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
// 函数简介     倒车收尾 / 反向过渡的"先刹停"原语
// 参数说明     v_now 本拍实测车速 m/s（带符号，来自 odom 快照）
// 返回参数     1 = 还在刹（调用方直接 return）/ 0 = 已停住或超时，可以继续
// 备注信息     ★ 为什么需要它：表尾是倒车段时，车到终点**还在往后滑**，
//              此时若直接 pb_finish()（放开电机 = 自由轮、舵机回正到 0），
//              车会继续退一段、方向盘还会突然自己回正 —— 现场看着就是"终点抽一下"。
//              倒车→前进的过渡同理（外环符号是反的）。
//              这里保持电机使能 + 目标 0（速度环主动刹），舵机保持最后角度不回正。
//--------------------------------------
static uint8 pb_tail_stop_wait (float v_now)
{
    if (0u == s_tail_stop)
    {
        s_tail_stop = 1u;
        s_tail_ms   = Sys_Tick_Ms;
    }

    Motor_Crtl_Enable    = 1;
    Motor_LR_Crtl.target = 0.0f;
    Motor_RR_Crtl.target = 0.0f;
    Head_Crtl_Set_Enable(0);

    if (fabsf(v_now) <= PB_TAIL_V)                      { return 0u; }
    if ((uint32)(Sys_Tick_Ms - s_tail_ms) >= PB_TAIL_MS) { return 0u; }

    return 1u;
}

//--------------------------------------
// 函数简介     判定"表尾是不是倒车段"
// 备注信息     ★ 不能只看最后一个点：录制器在停止时会补一个 v=0 的收尾点
//              （实测那张表最后一点 v=0、没有 REV 位），只看末点必然判成"前进"。
//              所以看最后 PB_TAIL_N 个点，多数是倒车点就算倒车段。
//--------------------------------------
static uint8 pb_tail_is_rev (const ins_wp_t *p)
{
    uint16 i    = 0u;
    uint16 from;
    uint8  revn = 0u;
    uint8  cnt  = 0u;

    if (s_n < (uint16)PB_TAIL_N) { from = 0u; }
    else                         { from = (uint16)(s_n - PB_TAIL_N); }

    for (i = from; i < s_n; i ++)
    {
        cnt ++;

        if (0u != pb_point_is_rev(&p[i]))
        {
            revn ++;
        }
    }

    return ((cnt > 0u) && ((uint16)revn * 2u >= (uint16)cnt)) ? 1u : 0u;
}

//--------------------------------------
// 函数简介     往前找第一个倒车点（找不到返回 s_n）
// 备注信息     判据与 poll() 里一致：REV 位优先、v 符号兜底
//--------------------------------------
static uint16 pb_next_rev (uint16 from, const ins_wp_t *p)
{
    uint16 i;
    uint16 end = (uint16)(from + PB_NEAR_WIN);

    if (end >= s_n) { end = (uint16)(s_n - 1u); }

    for (i = from; i <= end; i ++)
    {
        if (0u != pb_point_is_rev(&p[i]))
        {
            return i;
        }
    }

    return s_n;
}

//--------------------------------------
// 函数简介     速度剖面：曲率限速 + 末段收油
// 参数说明     best  当前进度点 / p 点表 / n 点数 / vmax 车速上限（= ROUTE_PB_V）
// 返回参数     float 本拍目标速度 m/s（已夹在 [vmin, vmax]）
// 备注信息     ★ 只在 pb_ve = 1 时调用；= 0 时回放仍走"一个常数速度"的老路径。
//              ★ 每拍现算，不建 s_vlim[] 数组（省 8 KB）：曲率窗口只有 2×PB_KWIN 个点，
//                末段收油最多扫到"从 vmax 降到 creep 所需的那几米"就提前退出。
//--------------------------------------
static float pb_speed_limit (uint16 best, const ins_wp_t *p, uint16 n, float vmax)
{
    float  alat = ROUTE_PB_ALAT;
    float  vmin = ROUTE_PB_VMIN;
    float  v, acc;
    uint16 i0, i1, k;

    if (alat < PB_ALAT_MIN) { alat = PB_ALAT_MIN; }
    if (alat > PB_ALAT_MAX) { alat = PB_ALAT_MAX; }
    if (vmin < PB_VMIN_MIN) { vmin = PB_VMIN_MIN; }
    if (vmin > vmax)        { vmin = vmax; }

    // ---- ① 曲率限速 ----
    // 弧长用**逐段弦长累加**，不能用两端点直线距离：弯道上弦长 < 弧长，会把 κ 算大、速度压过低
    i0 = (best > (uint16)PB_KWIN) ? (uint16)(best - (uint16)PB_KWIN) : 0u;
    i1 = (uint16)(best + (uint16)PB_KWIN);
    if (i1 > (uint16)(n - 1u)) { i1 = (uint16)(n - 1u); }

    acc = 0.0f;
    for (k = i0; k < i1; k ++) { acc += pb_dist(&p[k], &p[k + 1u]); }

    if (acc < 0.02f)
    {
        // 位置几乎没动：转过角度 ⇒ 原地打方向（最急，给地板）；没转 ⇒ 静止段，不压速
        float turn = fabsf(pb_wrap180(p[i1].yaw - p[i0].yaw)) * PB_D2R;

        v = (turn > 0.017f) ? vmin : vmax;
    }
    else
    {
        float kap = fabsf(pb_wrap180(p[i1].yaw - p[i0].yaw)) * PB_D2R / acc;

        v = (kap < 1e-3f) ? vmax : sqrtf(alat / kap);
    }

    // ---- ② 末段收油：v ≤ √(creep² + 2·alat·d) ----
    // 先算"从 vmax 降到 creep 需要多少米"；沿路径走不到这个距离就说明末段收油不生效，
    // 直接退出 —— 这样每拍最多扫十几米，不用每次从 best 走到表尾。
    {
        float need = ((vmax * vmax) - (PB_END_CREEP * PB_END_CREEP)) / (2.0f * alat);
        float d    = 0.0f;
        uint8 far  = 0u;

        for (k = best; (uint16)(k + 1u) < n; k ++)
        {
            d += pb_dist(&p[k], &p[k + 1u]);
            if (d >= need) { far = 1u; break; }
        }

        if (0u == far)
        {
            float vc = sqrtf((PB_END_CREEP * PB_END_CREEP) + (2.0f * alat * d));

            if (v > vc) { v = vc; }
        }
    }

    if (v > vmax) { v = vmax; }
    if (v < vmin) { v = vmin; }

    return v;
}

//--------------------------------------
// 函数简介     收尾：回中、让出驾驶
//--------------------------------------
static void pb_finish (uint8 st, const char *why)
{
    /* ★ 倒序回放的点序是**原地改的**，退出时必须复原，否则：
       ① 下一次起跑会再倒一次（双重翻转 = 回到原序）⇒ 表方向随机；
       ② 上位机读回来的表 / 存档都是反的。
       连倒两次 = 原序（ins_route_reverse_points 幂等）。 */
    if (0u != s_rev_flipped)
    {
        (void)ins_route_reverse_points();
        s_rev_flipped = 0u;
    }

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
    s_rev_flipped = 0u;             // 上电/重新初始化：点序一定是原序
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
// 备注信息     ★ 起跑前会判"表的出身"与 `rt_rply` 是否一致，不一致**拒绝起跑**并写 why：
//                  want=1 而表不是倒车录的  -> "not rev table"
//                  want=0 而表是倒车录的    -> "need RevPlay"
//               这是最危险的组合（以为正着开、其实表是倒的），宁可不跑。
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

    /* ---- 倒车录 / 倒序回放：判表出身 + 原地倒置点序（2026-10-24）----
       用途：车**倒着**沿轨迹录一张表之后，把点序倒过来，车**正着**开回起点。
       同一点的车身朝向完全相同、需要的前轮转角也完全相同，只有遍历顺序相反。
       判据：表里 REV 点过半 ⇒ 这是一张倒车录的表。
       与开关不一致就**不让起跑**，原因写进 s_why（屏幕第 7 项那行显示）。 */
    {
        uint16 nrev  = ins_route_rev_count();
        uint8  isrev = ((uint16)(nrev * 2u) >= n) ? 1u : 0u;
        uint8  want  = (ROUTE_REV_PLAY >= 0.5f) ? 1u : 0u;

        /* 上一次回放留下的倒序状态先复原（正常退出时 pb_finish 已经复原过，
           这里兜底"起跑失败后再次起跑"这条路径） */
        if (0u != s_rev_flipped)
        {
            (void)ins_route_reverse_points();
            s_rev_flipped = 0u;
        }

        if (want != isrev)
        {
            pb_set_why((0u != want) ? "not rev table" : "need RevPlay");
            return 0u;
        }

        if (0u != want)
        {
            if (0u == ins_route_reverse_points())
            {
                pb_set_why("rev flip fail");
                return 0u;
            }

            s_rev_flipped = 1u;
            n = ins_route_get_count();          // 倒置不改点数，这里只是明示
        }
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
    s_rev  = 0u;
    s_tail_stop = 0u;
    s_tail_ms   = 0u;
    s_dir_hold  = 0u;
    s_tail_rev  = pb_tail_is_rev(ins_route_get_points());   // ★ 读表尾状态，决定收尾策略（倒车段 ⇒ 先刹停）
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

        /* ★★ 刚体变换（2026-10-24 修）：位置增量必须和航向用**同一个**旋转角。
           旧代码只把增量转了 −s_o0yaw（等于**假定 p[0].yaw ≡ 0**），航向却叠了 p[0].yaw
           ⇒ 两者差 p[0].yaw，位置与朝向不是一个刚体。正车录的表 p[0].yaw 恒为 0，
           所以这个 bug 一直没露头；一旦**倒序回放**（p[0] 变成原表末点、yaw 非 0），
           横向外环就变成正反馈、车画大弧跑飞（14.pp_for_backroute 的仿真：最大偏离 3.081 m）。
           改法：θ = s_o0yaw − p[0].yaw 后仍用同一表达式 ⇒ 实际旋转 p[0].yaw − s_o0yaw。
           p[0].yaw == 0 时与旧代码逐字节等价 ⇒ 正车表零回归。 */
        float th  = (s_o0yaw - p[0].yaw) * PB_D2R;
        float c   = cosf(th);
        float sn  = sinf(th);

        px   = p[0].x + ( dx0 * c + dy0 * sn);
        py   = p[0].y + (-dx0 * sn + dy0 * c);
        pyaw = pb_wrap180(p[0].yaw + (o.yaw - s_o0yaw));
    }

    best  = pb_find_nearest(px, py, p);
    s_idx = best;

    // ---- 方向：倒车段判定（标志位优先、速度兜底；只认**同一拍**的 p[best]）----
    // 存储结构不用改：REV 位（rec+14 bit2）与有符号的 v（rec+10）本来就在表里，
    // 以前只是没人读。见 route_pb.h 里"倒车段"那段注释。
    {
        uint8  rv = pb_point_is_rev(&p[best]);

        /* 倒车 → 前进的过渡：车还在往后滑时**不能**打开航向环 —— δ = −kp·err 是给
           "车头朝前"推导的，倒着走时符号反，等于正反馈。先刹停再交回闭环。
           这一条与"表尾是倒车"共用同一个刹停原语 pb_tail_stop_wait()。 */
        if ((0u != s_rev) && (0u == rv) && (fabsf(o.v_mps) > PB_TAIL_V))
        {
            s_dir_hold = 1u;
        }

        s_rev = rv;
    }

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
    // ★ 表尾是倒车段（起跑时读最后几个点判出来的 s_tail_rev）⇒ **先刹停再收**：
    //   这时车正在往后滑，直接 pb_finish()（放开电机、舵机回正）会让车继续退、
    //   方向盘还会自己回正。表尾是前进段则维持原来的行为。
    {
        float lx = px - p[s_n - 1u].x;
        float ly = py - p[s_n - 1u].y;

        if ((0u != s_done_armed) && (sqrtf(lx * lx + ly * ly) < PB_FIN_M))
        {
            if ((0u != s_tail_rev) && (0u != pb_tail_stop_wait(o.v_mps))) { return; }

            pb_finish(PB_ST_DONE, "done");
            return;
        }
        if (best >= (uint16)(s_n - 1u))
        {
            if ((0u != s_tail_rev) && (0u != pb_tail_stop_wait(o.v_mps))) { return; }

            pb_finish(PB_ST_ABORT, "off course");
            return;
        }
    }

    // 倒车→前进的过渡等待（刹到 |v| 足够小才放行）
    if (0u != s_dir_hold)
    {
        if (0u != pb_tail_stop_wait(o.v_mps)) { return; }

        s_dir_hold  = 0u;
        s_tail_stop = 0u;               // 用完把这套刹停状态清掉，别影响后面的表尾逻辑
    }

    // 横向误差（正 = 车在参考线右侧）
    // ⚠ 倒车段 p[best].yaw 是车头朝向（与行进方向差 180°），这个量的符号会反；
    //   它只上报遥测、不参与回放决策，所以不为倒车单独修正（否则两处符号会打架）。
    {
        float tyaw = p[best].yaw * PB_D2R;

        s_elat = (px - p[best].x) * cosf(tyaw) + (py - p[best].y) * sinf(tyaw);
    }

    // 速度：正车段只认参数；倒车段用表里录的负值（选项 C：表里优先、参数封顶）
    {
        float v;

        if (0u != s_rev)
        {
            /* 倒车段：v = 表里录的负值，被 |pb_v| 与 PB_REV_VMAX 封顶。
               pb_v = 0（自由轮）时上限也是 0 —— 手推模式本来就不该自己倒车。 */
            float vlim = fabsf(ROUTE_PB_V);

            if (vlim > PB_REV_VMAX) { vlim = PB_REV_VMAX; }

            v = p[best].v;
            if (v < -vlim) { v = -vlim; }
            if (v >  0.0f) { v =  0.0f; }       // 标了倒车却给正值：宁可不走
        }
        else
        {
            v = ROUTE_PB_V;

            if (v < PB_V_MIN) { v = PB_V_MIN; }
            if (v > PB_V_MAX) { v = PB_V_MAX; }

            // 换向点在眼前：留一点爬行速度。实测表里 29→30 只有 4 mm，
            // 不减速的话进度点会在换向段前后反复（而且那一拍速度要反向，冲击很大）。
            if ((uint16)(best + 1u) < s_n)
            {
                if (0u != pb_point_is_rev(&p[best + 1u]))
                {
                    if (v > PB_REV_CREEP) { v = PB_REV_CREEP; }
                }
            }

            // ★ 速度剖面（pb_ve = 1）：pb_v 从"恒定车速"变成"上限"，
            //   曲率限速自动压弯道、末段收油防止冲到表尾。pb_ve = 0 时这一段不执行。
            //   只对**正向**生效（倒车段在上面已经单独处理）。
            if ((ROUTE_PB_VE >= 0.5f) && (v > 0.005f))
            {
                v = pb_speed_limit(best, p, s_n, v);
            }
        }

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
            Motor_Diff_Alloc(vrps, Servo_Angle_Deg, ROUTE_PB_L,
                             &Motor_LR_Crtl.target, &Motor_RR_Crtl.target);
        }
    }

    // ---- 倒车段：关外环 + 开环回放录制打角，然后**跳过**整段纯跟踪 ----
    // 为什么必须跳过：下面 α = atan2(bx,by) 与 target_yaw = o.yaw − α 都按"车头朝前"推导，
    // 倒车时前视点天然落在车后（实测那张表 α≈−178°），继续闭环会把舵机顶到满舵、
    // 而电机是反转的 ⇒ 车辆绕圈跑飞。开环回放录制打角才是对的（那一拍车就是这么走的）。
    if (0u != s_rev)
    {
        float d = 0.0f;

        if (0u != (p[best].flags & (uint16)INS_WP_FL_STEER))
        {
            d = (float)p[best].steer_cdeg * 0.01f;
        }

        if (d >  PB_DELTA_MAX) { d =  PB_DELTA_MAX; }
        if (d < -PB_DELTA_MAX) { d = -PB_DELTA_MAX; }

        s_ld  = 0.0f;                   // 倒车段没有前视距离这个概念
        s_aim = best;

        Head_Crtl_Set_Enable(0);
        Servo_Target_Deg = d;
        return;
    }

    // 前视距离
    s_ld = ROUTE_PB_LD;

    if (s_ld < PB_LD_MIN) { s_ld = PB_LD_MIN; }
    if (s_ld > PB_LD_MAX) { s_ld = PB_LD_MAX; }

    j = pb_find_lookahead(best, p, s_ld);

    /* ★ 前视点不许越进倒车段：越过去的话，纯跟踪会把车头指向"倒回来的那条腿"，
       而那条腿就在车后 ⇒ α≈±180° ⇒ 满舵。实测那张表在 25~29 点就是这样被指令成 −27°，
       可录制打角只有 +13°。
       判据：到换向点的**剩余前进弧长 < 一个前视** ⇒ 这一段是"蹭到换向点"，
       纯跟踪在这个尺度上没有意义，直接照抄表里录的打角（与倒车段同一取舍）。
       没有倒车段的表 ⇒ pb_next_rev() 返回 s_n ⇒ 这段完全不执行，零回归风险。 */
    {
        uint16 rv = pb_next_rev(best, p);

        if (rv < s_n)
        {
            float  drem = 0.0f;
            uint16 k;

            for (k = best; (uint16)(k + 1u) <= rv; k ++)
            {
                drem += pb_dist(&p[k], &p[k + 1u]);
            }

            if (drem < s_ld)
            {
                float d = 0.0f;

                if (0u != (p[best].flags & (uint16)INS_WP_FL_STEER))
                {
                    d = (float)p[best].steer_cdeg * 0.01f;
                }

                if (d >  PB_DELTA_MAX) { d =  PB_DELTA_MAX; }
                if (d < -PB_DELTA_MAX) { d = -PB_DELTA_MAX; }

                s_aim = best;
                Head_Crtl_Set_Enable(0);
                Servo_Target_Deg = d;
                return;
            }
        }
    }

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
            // 实际施加由 5 ms 中断里的 Head_Crtl_Tick() 完成，这里只给目标与增益。
            // pb_kh > 0 时用手填值覆盖（做 A/B 用），见 route_pb.h 的说明。
            float k = (ROUTE_PB_KH > 1.0e-3f) ? ROUTE_PB_KH : (2.0f * ROUTE_PB_L / s_ld);

            if (k < PB_K_MIN) { k = PB_K_MIN; }
            if (k > PB_K_MAX) { k = PB_K_MAX; }

            Head_Crtl_Set_Kp(k);
            Head_Crtl_Set_Target(pb_wrap180(o.yaw - alpha * PB_R2D));
            Head_Crtl_Set_Enable(1);
        }
        else
        {
            // 直接给 δ
            float kappa = 2.0f * sinf(alpha) / s_ld;
            float delta = atanf(ROUTE_PB_L * kappa) * PB_R2D;

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
