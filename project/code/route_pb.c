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
static uint8   s_done_armed = 0u;   // 1 = 已走过完成阈值点（**只给几何通道**用）
static float   s_elat = 0.0f;
static float   s_ld   = 0.0f;
static float   s_tv   = 0.0f;       // 本拍目标速度 m/s（带符号）——前视距离按它算
static float   s_dtrue = 0.0f;      // 本拍到折线的真实距离 m（脱表监测用，elat 只是一维投影）
static float   s_d0   = 0.0f;       // 起跑时的 odom 累计里程 m（跑飞判据用）
static uint16  s_offcnt = 0u;       // 连续脱表拍数
static uint16  s_estop_ticks = 0u;  // 急停去抖计数
static uint8   s_dir_chk = 0u;      // 1 = 首拍方向自检还没做
static uint8   s_rev  = 0u;         // 1 = 当前在倒车段（依据 p[best] 的 REV 位 / v 符号）
static uint8   s_tail_rev = 0u;     // 1 = **表尾是倒车段**（起跑时读最后几个点判一次）
/* 播放副本（= 队友 v15 的 s_play[]）：录制的那份 s_wp[] 一个字都不动。
   2048 × 20 B = 40 KB；换掉"原地翻转 + 退出复原"那套易错逻辑。 */
static ins_wp_t s_play[INS_ROUTE_MAX_POINTS];
static uint8   s_drv_rev = 0u;      // 本腿行驶方向（1 = 倒车）：起跑取 s_dir_tab[0]，每次换腿在确认点更新
static uint8   s_rev_tab = 0u;      // 表出身：1 = 倒车录的表（= v15 的 s_rev_tab）
static uint8   s_rev_ord = 0u;      // 播放顺序：1 = 倒序回放（车正着开回起点，= v15 的 s_rev_ord）
static uint8   s_tail_stop = 0u;    // 1 = 正在"倒车收尾刹停"
static uint32  s_tail_ms  = 0u;     // 刹停开始时刻
static float   s_kp_fwd = 0.0f;     // 起跑时抓的**正向**舵机位置环 Kp（倒车段的基准值）
static uint8   s_kp_bg  = 0u;       // 1 = 当前已切到倒车增益（Kp/PB_REV_KP_DIV）
static uint8   s_rel_pend = 0u;     // 1 = 已收车、等 PB_REL_TICKS 拍再回中（见 pb_release_tick）
static uint16  s_rel_ticks = 0u;    // 释放期已过的拍数
static uint32  s_rel_ms   = 0u;     // 释放期起点（PB_REL_MS 兜底用）

/* ★ 混合表支持（2026-10-28，设计以 15.mix_mode_add 为准）：方向与弧长都做成"逐点表"，
   同一张表里前进段/倒车段混排时按点换向，不再依赖"整表一个方向"。
   RAM：s_dir_tab 2 KB + s_arc_tab 8 KB（cm7_0 SRAM 384 KB，当前 RW ≈ 97 KB） */
static uint8   s_dir_tab[INS_ROUTE_MAX_POINTS];   // 每点行驶方向：1 = 倒车
static float   s_arc_tab[INS_ROUTE_MAX_POINTS];   // 每点累计弧长 m
static float   s_vlim[INS_ROUTE_MAX_POINTS];      // 每点限速 m/s（**幅值**，方向由 s_rev 给）
static float   s_kap[INS_ROUTE_MAX_POINTS];       // 每点曲率 1/m（给曲率变化率限速用）
static uint8   s_mixed = 0u;                      // 1 = 表里既有前进点又有倒车点
static uint8   s_mileage = 0u;                    // 1 = 进度点按里程反查（混合/倒车表）

/* 里程通道的进度基准（= 队友 v15 的 s_leg_start / s_leg_d0）。
   每换一条腿就把基准挪到"当前弧长 + 当前里程"：跨腿累积的轮速标定误差
   （右轮每米多计 4%）与换向打滑都不会带进下一条腿。 */
static uint16  s_leg_start = 0u;                  // 本腿起点的表索引
static float   s_leg_d0    = 0.0f;                // 本腿起点的 odom 累计里程 m
static uint16  s_shift_idx = 0u;                  // 下一条腿的起点（下一个方向切换点），n-1 = 没有了
static uint8   s_shifting  = 0u;                  // 1 = 换向驻留中（目标 0，等车停稳）
static uint8   s_stopped   = 0u;                  // 1 = 驻留期间已停稳
static uint32  s_shift_t0  = 0u;                  // 换向开始时刻 ms（超时用）
static uint32  s_stopped_t0 = 0u;                 // 停稳开始时刻 ms（驻留计时）
static uint8   s_depart    = 0u;                  // 1 = 换腿后的出发限速中（从爬行速度按 abrk 抬起）
static uint32  s_depart_t0 = 0u;                  // 出发时刻 ms

static float   s_o0x   = 0.0f;      // 开跑那一刻的 odom（只用相对量）
static float   s_o0y   = 0.0f;
static float   s_o0yaw = 0.0f;

static char    s_why[24] = "";
static uint8   s_why_code = 0u;     // 收车原因码，见 pb_set_why() 注释里的数值表

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
// 函数简介     该点在**录制时**的行驶方向（1 = 倒车）
// 备注信息     只在 pb_build_play() 建 s_dir_tab 时用；之后一律读 s_dir_tab。
//              倒序回放的方向取反不在这里做，在 pb_build_play() 里整表做一次。
//--------------------------------------
static uint8 pb_rec_dir (const ins_wp_t *q)
{
    return ((0u != (q->flags & (uint16)INS_WP_FL_REV)) || (q->v < -PB_REV_EPS)) ? 1u : 0u;
}

//--------------------------------------
// 函数简介     建**播放副本** s_play[] + 方向表 s_dir_tab[] + 弧长表 s_arc_tab[]
// 参数说明     raw 录制原表 / n 点数 / want 1 = 倒序回放 / isrev 表出身（过半的点标了 REV）
// 备注信息     = 队友 v15 route_pb.c:564-601。
//              ★ 为什么要副本：录制的那份 s_wp[] **一个字都不动** ⇒ 退出 / 中断 / 上位机读表 /
//                存盘 / 再录制，拿到的永远是原序。原来那套"原地翻转 + 退出必须复原"
//                （s_rev_flipped）可以整个删掉。
//              ★ 副本上只做两件事：点序倒置（want）、速度取幅值（方向只由 REV 位承载）。
//              ★ s_dir_tab[i] = 车在 play 第 i 点上的行驶方向。静止点（|v| ≤ 死区）没有方向
//                信息，继承最近一个有效点的方向 —— 否则停车那一段方向会丢，换向点跟着漂。
//              ★ 非倒序时 v15 把方向表左移一位、末点沿用前一点（其 :594-598）。照抄。
//--------------------------------------
static void pb_build_play (const ins_wp_t *raw, uint16 n, uint8 want, uint8 isrev)
{
    uint16 i;
    uint8  rec  = isrev;
    uint16 nfwd = 0u;
    uint16 nrev = 0u;

    /* ---- 1) 方向表：从**原表**建（副本里的 v 会被取成绝对值，方向信息就没了）---- */
    for (i = 0u; i < n; i ++)
    {
        if (fabsf(raw[i].v) > PB_RECORD_DIR_EPS) { rec = pb_rec_dir(&raw[i]); break; }
    }

    for (i = 0u; i < n; i ++)
    {
        if (fabsf(raw[i].v) > PB_RECORD_DIR_EPS) { rec = pb_rec_dir(&raw[i]); }

        s_dir_tab[i] = rec;

        if (0u != rec) { nrev ++; } else { nfwd ++; }
    }

    /* ---- 2) 播放副本 + 弧长 ---- */
    s_arc_tab[0] = 0.0f;

    for (i = 0u; i < n; i ++)
    {
        s_play[i]   = raw[(0u != want) ? (uint16)(n - 1u - i) : i];
        s_play[i].v = fabsf(s_play[i].v);

        if (i > 0u) { s_arc_tab[i] = s_arc_tab[i - 1u] + pb_dist(&s_play[i - 1u], &s_play[i]); }
    }

    /* ---- 3) 方向表换到 play 序 ---- */
    if (0u != want)
    {
        /* 点序倒过来 + 车是反着遍历 ⇒ 每个方向取反 */
        for (i = 0u; i < (uint16)(n / 2u); i ++)
        {
            uint8 t = s_dir_tab[i];

            s_dir_tab[i]                    = s_dir_tab[(uint16)(n - 1u - i)];
            s_dir_tab[(uint16)(n - 1u - i)] = t;
        }
        for (i = 0u; i < n; i ++) { s_dir_tab[i] ^= 1u; }
    }
    else
    {
        for (i = 0u; (uint16)(i + 1u) < n; i ++) { s_dir_tab[i] = s_dir_tab[i + 1u]; }
    }
    s_dir_tab[(uint16)(n - 1u)] = s_dir_tab[(uint16)(n - 2u)];

    /* ---- 4) 通道选择（= v15 :599-601）---- */
    s_mixed   = ((0u != nfwd) && (0u != nrev)) ? 1u : 0u;
    s_drv_rev = s_dir_tab[0];
    s_mileage = ((0u != s_mixed) || (0u != s_drv_rev)) ? 1u : 0u;
}

//--------------------------------------
// 函数简介     点到折线的真实距离（= 队友 v15 route_pb.c:166-186）
// 参数说明     px/py 车位置 / p 点表 / n 点数 / i 进度点
// 返回参数     float 到 i 前后那一段折线的垂直距离 m
// 备注信息     ★ 为什么不用 elat：elat 只是一维横向投影，车沿切线方向跑偏时它≈0，
//                看起来"贴得很好"，实际已经离开路线。脱表监测必须用这个。
//--------------------------------------
static float pb_seg_dist (float px, float py, const ins_wp_t *p, uint16 n, uint16 i)
{
    uint16 i0 = i;
    uint16 i1 = i;
    float ax, ay, bx, by, vx, vy, wx, wy, dd, t;

    if (n < 2u) { return 0.0f; }

    if (i0 > 0u)               { i0 = (uint16)(i - 1u); }
    if (i1 < (uint16)(n - 1u)) { i1 = (uint16)(i + 1u); }

    ax = p[i0].x; ay = p[i0].y;
    bx = p[i1].x; by = p[i1].y;
    vx = bx - ax; vy = by - ay;
    wx = px - ax; wy = py - ay;
    dd = (vx * vx) + (vy * vy);
    t  = (dd > 1.0e-9f) ? ((wx * vx + wy * vy) / dd) : 0.0f;

    if (t < 0.0f) { t = 0.0f; }
    if (t > 1.0f) { t = 1.0f; }

    wx = wx - (t * vx);
    wy = wy - (t * vy);

    return sqrtf((wx * wx) + (wy * wy));
}

//--------------------------------------
// 函数简介     记一条停止原因（屏幕用它显示，why 码给遥测）
// 备注信息     原因码与 v15 route_pb.c:408-432 **同一套数值**，两边对表时能直接比：
//              0 = 本次上电还没停过   1 head test busy  2 pb_en=0   3 RC ON    4 ESTOP
//              5 odom off            6 done: finish    7 end idx   8 runaway 2x
//              9 lost: orbit        10 drv preempted  11 ht busy  15 其它
//              我们没有 head test / pb_en / 驾驶令牌，所以 1/2/9/10/11 暂时不会出现。
//--------------------------------------
static uint8 pb_str_eq (const char *a, const char *b)
{
    while (('\0' != *a) && (*a == *b)) { a ++; b ++; }

    return (uint8)((*a == *b) ? 1u : 0u);
}

static void pb_set_why (const char *w)
{
    uint8 i = 0u;

    if (NULL == w) { s_why[0] = '\0'; s_why_code = 0u; return; }

    while ((i < (uint8)(sizeof(s_why) - 1u)) && ('\0' != w[i]))
    {
        s_why[i] = w[i];
        i ++;
    }
    s_why[i] = '\0';

    if      (pb_str_eq(w, "ESTOP"))              { s_why_code =  4u; }
    else if (pb_str_eq(w, "estop"))              { s_why_code =  4u; }
    else if (pb_str_eq(w, "RC ON"))              { s_why_code =  3u; }
    else if (pb_str_eq(w, "rc takeover"))        { s_why_code =  3u; }
    else if (pb_str_eq(w, "odom off"))           { s_why_code =  5u; }
    else if (pb_str_eq(w, "done"))               { s_why_code =  6u; }
    else if (pb_str_eq(w, "off course"))         { s_why_code =  7u; }
    else if (pb_str_eq(w, "runaway 2x len"))     { s_why_code =  8u; }
    else                                         { s_why_code = 15u; }
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
// 函数简介     本趟已走弧长（里程通道的进度量）
// 参数说明     dist 本拍 odom 累计里程 m
// 返回参数     float 已走弧长 m
// 备注信息     = 队友 v15 route_pb.c:1153 的 `s_arc[s_leg_start] + (dist_abs - s_leg_d0)`。
//              ★ 纯前进表（非混合）时 s_leg_start 恒 0、s_leg_d0 = 起跑里程，
//                与 v15 另一条式子 `dist_abs - s_d0` 完全等价 ⇒ 只用一条式子。
//              ★ 每换一条腿就把基准挪一次，见 poll() 的换向确认段。
//--------------------------------------
static float pb_travel (float dist)
{
    return s_arc_tab[s_leg_start] + (dist - s_leg_d0);
}

//--------------------------------------
// 函数简介     按本趟里程反查进度点（单调前进）
// 参数说明     travel 已走弧长 m
// 返回参数     uint16 进度点索引
// 备注信息     = 队友 v15 route_pb.c:376-387，逐字同构（含 +0.001 的容差）。
//              ★ 为什么几何不行：重叠分支的 x/y 相同但路线位置不同。
//                实测那张混合表用最近点会从 12 跳到 40（弧长 2.68 m），随后整条控制链崩掉。
//--------------------------------------
static uint16 pb_find_mileage (float travel)
{
    uint16 best = s_idx;

    if (travel < 0.0f) { travel = 0.0f; }

    while (((uint16)(best + 1u) < s_n) && (s_arc_tab[best + 1u] <= (travel + 0.001f)))
    {
        best ++;
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

    Motor_Ctrl_Enable    = 1;
    Motor_LR_Ctrl.target = 0.0f;
    Motor_RR_Ctrl.target = 0.0f;
    Head_Ctrl_Set_Enable(0);

    if (fabsf(v_now) <= PB_TAIL_V)                      { return 0u; }
    if ((uint32)(Sys_Tick_Ms - s_tail_ms) >= PB_TAIL_MS) { return 0u; }

    return 1u;
}

//--------------------------------------
// 函数简介     切换舵机位置环增益：倒车段 Kp/PB_REV_KP_DIV，其余用标称值
// 参数说明     rev 1 = 倒车段（含倒车→前进时"还在往后滑"的过渡等待）
// 备注信息     ★ 只在状态真的变了才写 Servo_Ctrl_Kp —— 它是参数页/上位机在用的运行期副本，
//                 每拍无条件写会盖掉同一个 5 ms 周期里刚改的值。
//              ★ 恢复点两处：poll() 里切回前进段、pb_finish()。漏一处就会把 1/3 留给遥控/菜单。
//--------------------------------------
static void pb_steer_gain (uint8 rev)
{
    if (rev == s_kp_bg) { return; }

    if (s_kp_fwd < 1.0f)                    // 起跑没抓到（Kp 被人清成 0）⇒ 不缩，别把舵机做没了
    {
        s_kp_bg = 0u;
        return;
    }

    s_kp_bg = rev;
    Servo_Ctrl_Kp = (0u != rev) ? (s_kp_fwd / PB_REV_KP_DIV) : s_kp_fwd;
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

        if (0u != s_dir_tab[i])
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
        if (0u != s_dir_tab[i])
        {
            return i;
        }
    }

    return s_n;
}

//--------------------------------------
// 函数简介     建速度剖面 s_vlim[] / s_kap[]（= 队友 v15 route_pb.c:188-346）
// 参数说明     n 点数
// 备注信息     ★ 一次建表、每拍只查 s_vlim[best]，不再每拍现算。
//                代价 2 个 2048×4 B = 16 KB；换来的是"反向刹车传播"这种全局算法
//                —— 它要从表尾往前看，每拍现算做不了。
//              ①  曲率限速        v ≤ √(alat/κ)，κ = |Δyaw| / 弧长
//              ①c 曲率变化率限速  δ = atan(L·κ) ⇒ |dδ/dt| ≤ PB_SLEW_RATE_DEG_S
//              ①c'末点锚：最后一个前进点压到 PB_END_CREEP，减速曲线交给 ② 生成
//              ②  反向传刹车      v[i-1] ≤ √(v[i]² + 2·abrk·ds)
//              ③  地板抬回 vmin
//              ④  混合表：每条腿单独向换向点传刹车（跨腿不互传，否则会算出负限速）
//              ★ v15 里还有一段"按 s_vlim<0 判换向点前爬行"（其 :284-287），
//                在"v 恒为非负幅值"的数据契约下**不可达**，故不搬；
//                换向前的爬行由 ④ 和应用段的 cap 承担。
//--------------------------------------
static void pb_build_profile (uint16 n)
{
    const ins_wp_t *p = s_play;
    float alat = ROUTE_PB_ALAT;
    float abrk = PB_ABRK_MPS2;
    float vmax = fabsf(ROUTE_PB_V);
    float vmin = ROUTE_PB_VMIN;
    float l    = ROUTE_PB_L;
    float ds_pt;
    int   win  = (int)PB_KWIN;
    uint16 i;

    if (alat < PB_ALAT_MIN) { alat = PB_ALAT_MIN; }
    if (alat > PB_ALAT_MAX) { alat = PB_ALAT_MAX; }
    if (vmax < 0.10f)       { vmax = 0.10f; }   /* 手推模式面板值 0，剖面仍要建得出来 */
    if (vmax > PB_V_MAX)    { vmax = PB_V_MAX; }
    if (vmin < PB_VMIN_MIN) { vmin = PB_VMIN_MIN; }
    if (vmin > vmax)        { vmin = vmax; }
    if (abrk < 0.05f)       { abrk = 0.05f; }
    if (win < 1)            { win = 1; }
    if (win > 20)           { win = 20; }

    ds_pt = (s_arc_tab[(uint16)(n - 1u)] > 0.05f) ? (s_arc_tab[(uint16)(n - 1u)] / (float)(n - 1u)) : 0.1f;

    /* ---- ① 曲率限速 ---- */
    for (i = 0u; i < n; i ++)
    {
        uint16 i0, i1, k;
        float  arc = 0.0f;
        float  v;

        s_kap[i] = 0.0f;

        i0 = ((int)i - win > 0) ? (uint16)((int)i - win) : 0u;
        i1 = ((int)i + win < (int)n - 1) ? (uint16)((int)i + win) : (uint16)(n - 1u);

        /* 混合表：曲率窗口不许跨过换向点（跨过去等于把两条腿拼成一条，κ 无意义） */
        if (0u != s_mixed)
        {
            i0 = i;
            i1 = i;
            while ((i0 > 0u) && ((uint16)(i - i0) < (uint16)win) &&
                   (s_dir_tab[i0 - 1u] == s_dir_tab[i])) { i0 --; }
            while (((uint16)(i1 + 1u) < n) && ((uint16)(i1 - i) < (uint16)win) &&
                   (s_dir_tab[i1 + 1u] == s_dir_tab[i])) { i1 ++; }
        }

        for (k = i0; k < i1; k ++) { arc += pb_dist(&p[k], &p[k + 1u]); }

        {
            float dyaw = fabsf(pb_wrap180(p[i1].yaw - p[i0].yaw) * PB_D2R);

            if (arc < 0.02f)
            {
                /* 位置几乎没动：转过角度 ⇒ 原地打方向（最急，给地板）；没转 ⇒ 静止段，不压速 */
                v = (dyaw > 0.017f) ? vmin : vmax;
            }
            else
            {
                float kap = dyaw / arc;

                s_kap[i] = kap;
                v = (kap < 1e-3f) ? vmax : sqrtf(alat / kap);
            }
        }

        if (v > vmax) { v = vmax; }
        if (v < vmin) { v = vmin; }

        s_vlim[i] = v;
    }

    /* ---- ①c 曲率变化率限速 ----
       δ = atan(L·κ) ⇒ |dδ/ds| = L·|κ'| / (1 + (Lκ)²)，要求 |dδ/dt| = v·|dδ/ds| ≤ rate
       直道与定圆的 κ'≈0 不受限，只压"边跑边打舵"的 S 段。 */
    {
        const float rate = PB_SLEW_RATE_DEG_S * PB_D2R;     /* rad/s */

        for (i = 1u; (uint16)(i + 1u) < n; i ++)
        {
            float span = s_arc_tab[i + 1u] - s_arc_tab[i - 1u];
            float dk, lk, vcap;

            if (span < 0.001f) { span = 2.0f * ds_pt; }

            dk = (s_kap[i + 1u] - s_kap[i - 1u]) / span;
            if (fabsf(dk) < 1.0e-3f) { continue; }

            lk   = l * s_kap[i];
            vcap = (rate * (1.0f + (lk * lk))) / ((l * fabsf(dk)) + 1.0e-6f);

            if (s_vlim[i] > vcap) { s_vlim[i] = vcap; }
            if (s_vlim[i] < vmin) { s_vlim[i] = vmin; }
        }
    }

    /* ---- ①c' 末点锚：只改最后一个前进点的速度，减速曲线由 ② 反向生成 ---- */
    {
        float creep = PB_END_CREEP;

        if (creep < vmin) { creep = vmin; }
        if (creep > vmax) { creep = vmax; }

        for (i = (uint16)(n - 1u); i > 0u; i --)
        {
            if (s_vlim[i] >= 0.0f)
            {
                if (s_vlim[i] > creep) { s_vlim[i] = creep; }
                break;
            }
        }
    }

    /* ---- ② 反向传刹车：v[i-1] ≤ √(v[i]² + 2·abrk·ds) ----
       这一段就是"前方有弯/有换向点，提前几米开始收油"的来源，
       也是我们原来那句"末段收油是队友反向传刹车的单点简化版"的正式版本。 */
    for (i = (uint16)(n - 1u); i > 0u; i --)
    {
        float ds  = pb_dist(&p[i - 1u], &p[i]);
        float lim = sqrtf((s_vlim[i] * s_vlim[i]) + (2.0f * abrk * ds));

        if (s_vlim[i - 1u] > lim) { s_vlim[i - 1u] = lim; }
    }

    /* ---- ③ 地板抬回 vmin ---- */
    for (i = 0u; i < n; i ++)
    {
        if (s_vlim[i] < vmin) { s_vlim[i] = vmin; }
    }

    /* ---- ④ 混合表：每条腿单独向换向点传刹车（跨腿不互传）---- */
    if (0u != s_mixed)
    {
        for (i = 1u; i < n; i ++)
        {
            uint16 k;
            float  next = 0.0f;

            if (s_dir_tab[i] == s_dir_tab[i - 1u]) { continue; }

            for (k = i; k > 0u; k --)
            {
                float lim = sqrtf((next * next) + (2.0f * abrk * pb_dist(&p[k - 1u], &p[k])));

                if (s_vlim[k - 1u] > lim) { s_vlim[k - 1u] = lim; }
                next = s_vlim[k - 1u];

                if ((k > 1u) && (s_dir_tab[k - 2u] != s_dir_tab[k - 1u])) { break; }
            }
        }
    }
}

//--------------------------------------
// 函数简介     收车回中：连续 PB_REL_TICKS 拍都不在跑，才把舵角目标写 0
// 备注信息     ★ 与队友 v15 route_pb.c:436 的 pb_release_tick 同一思路，但**只写一次**：
//                 v15 从第 6 拍起每拍都写 0，遥控接管后会把 RC 的舵角每圈顶掉一次；
//                 这里达到拍数就清 s_rel_pend，之后不再碰舵角目标。
//              ★ 这条路径由 poll() 在非 RUN 状态下续算，所以 Route 页要配合别抢方向盘，
//                 见 route_pb_releasing()。
//--------------------------------------
static void pb_release_tick (void)
{
    if (0u == s_rel_pend) { return; }

    if (s_rel_ticks < (uint16)PB_REL_TICKS) { s_rel_ticks ++; }

    if (s_rel_ticks >= (uint16)PB_REL_TICKS)
    {
        s_rel_pend       = 0u;
        Servo_Target_Deg = 0.0f;        // 回中（不对齐实测角：对齐等于把满舵保持住）
    }
}

//--------------------------------------
// 函数简介     收尾：让出驾驶（舵角回中延后到释放期，见 pb_release_tick）
//--------------------------------------
static void pb_finish (uint8 st, const char *why)
{
    /* ★ 不再需要"复原点序"：倒序回放改的是私有副本 s_play[]，录制表 s_wp[] 从没被动过。
       原实现要在这里再倒一次，漏掉就会让表停在倒序（下次起跑双重翻转 / 上位机读回反表）。 */

    s_st = st;
    pb_set_why(why);

    /* 收车顺序：航向外环 → 速度目标 → 舵角回中（回中放到释放期，见 pb_release_tick）。
       先关外环再清速度：外环还开着时清速度，车一边减速一边被外环继续打角。 */
    Head_Ctrl_Set_Kp(HEAD_KP_DEFAULT);      // 收回动态增益，别把回放的 k 留给遥控
    Head_Ctrl_Set_Enable(0);
    pb_steer_gain(0u);                      // 收回倒车段的 1/3 内环增益（同上，不留给遥控）
    Motor_Ctrl_Enable    = 0;       // 自由轮
    Motor_LR_Ctrl.target = 0.0f;
    Motor_RR_Ctrl.target = 0.0f;

    s_rel_ticks = 0u;
    s_rel_ms    = (uint32)Sys_Tick_Ms;
    s_rel_pend  = 1u;
    pb_release_tick();             // 收车这一拍先记一拍，回中在拍数够了之后才写
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
    s_kp_fwd = 0.0f;                // 下次起跑重新抓标称内环 Kp
    s_kp_bg  = 0u;
    s_rel_pend  = 0u;
    s_rel_ticks = 0u;
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

    /* 录制中不许起跑（= v15 route_pb.c:507），与 ins_route_start() 的回放闸门配成一对锁 */
    if (0u != ins_route_is_running()) { pb_set_why("recording"); return 0u; }

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

        /* 点序不再原地倒：私有副本在 pb_build_play() 里按 s_rev_ord 生成，
           录制表 s_wp[] 从起跑到收车一个字都没动过 */

        /* 混合表（前进段 + 倒车段混排）：两种方向都有 ⇒ 只允许"按表序跑"。
           rply = 1 要求整表只有一个方向，对混合表没有意义 ⇒ 直接拒跑并说明原因。 */
        {
            const ins_wp_t *raw = ins_route_get_points();
            uint16 i;
            uint8  nf = 0u, nr = 0u;

            for (i = 0u; i < n; i ++)
            {
                if (fabsf(raw[i].v) <= PB_REV_EPS) { continue; }
                if (0u != pb_rec_dir(&raw[i])) { nr = 1u; } else { nf = 1u; }
            }

            if ((0u != nf) && (0u != nr))
            {
                if (0u != want) { pb_set_why("mix table: rply=0"); return 0u; }
            }
            else if (want != isrev)
            {
                pb_set_why((0u != want) ? "not rev table" : "need RevPlay");
                return 0u;
            }
        }

        s_rev_tab = isrev;

        /* ★ 点序不再原地倒：pb_build_play() 在私有副本上做，录制表一个字不动 */
        s_rev_ord = want;
    }

    ins_odom_snapshot(&o);

    s_o0x   = o.x;
    s_o0y   = o.y;
    s_o0yaw = o.yaw;
    s_d0    = o.dist_abs;               // 跑飞判据的里程基准
    s_offcnt     = 0u;
    s_estop_ticks = 0u;
    s_dir_chk    = 1u;                  // 首拍做一次方向自检

    s_n    = n;
    s_idx  = (n > 3u) ? 3u : 0u;
    s_aim  = s_idx;
    s_elat = 0.0f;
    s_ld   = 0.0f;
    s_rev  = 0u;
    s_tail_stop = 0u;
    s_tail_ms   = 0u;
    pb_build_play(ins_route_get_points(), n, s_rev_ord, s_rev_tab);   // ★ 副本 + 方向 + 弧长
    pb_build_profile(n);                                     // ★ 速度剖面（曲率/速率/反向刹车）
    s_leg_start = 0u;                                       // 里程通道：本腿基准 = 表头
    s_leg_d0    = o.dist_abs;
    s_shifting  = 0u;
    s_stopped   = 0u;
    s_depart    = 0u;

    /* 下一个方向切换点 = 表里**第一个**方向变化处（= v15 route_pb.c:602-604）。
       注意与"最后一个切换点"不是一回事：换腿后要在这里重算，见 poll() 的换向确认段。 */
    {
        uint16 i;

        s_shift_idx = (uint16)(s_n - 1u);
        for (i = 1u; (uint16)(i + 1u) < s_n; i ++)
        {
            if (s_dir_tab[i] != s_dir_tab[i - 1u]) { s_shift_idx = i; break; }
        }
    }
    s_tail_rev  = pb_tail_is_rev(s_play);                   // ★ 读表尾状态，决定收尾策略（倒车段 ⇒ 先刹停）
    s_done_armed = 0u;              // 新起跑：重新计算"走过没有"
    pb_set_why(NULL);               // 原因与 why 码一起清（NULL ⇒ 码 0 = 本次上电还没停过）

    // 直接给 δ 的通道先把航向环关掉；走航向通道时由 poll 每拍打开
    if (ROUTE_PB_SMODE >= 0.5f) { Head_Ctrl_Set_Enable(0); }

    /* 内环增益基准：每次起跑重抓一次 Servo_Ctrl_Kp（参数页/上位机可能在两次回放之间改过），
       并保证起跑那一拍是正向值。倒车段由 poll() 逐点切到 1/3。 */
    pb_steer_gain(0u);                      // 兜底：上一次若没收干净，先复原
    s_kp_fwd = (Servo_Ctrl_Kp > 1.0f) ? Servo_Ctrl_Kp : SERVO_KP_DEFAULT;

    s_rel_pend  = 0u;                       // 上一趟没走完的"收车回中"作废：正在跑就由 poll 管舵角
    s_rel_ticks = 0u;

    // 回放要打角：手推录制时舵机是失能的，这里自己打开
    Servo_Ctrl_Enable = 1;

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

    if (PB_ST_RUN != s_st) { pb_release_tick(); return; }

    /* ---- 安全门（= v15 route_pb.c:1028-1066）----
       顺序：遥控/急停 → odom。急停要**去抖**：安全层当拍就硬停机了，
       回放这边慢 PB_ESTOP_TICKS 拍再判中止，避免单拍抖一下就收车。 */
    if ((RC_ENGAGE_ON == RC_Engage) || (RC_ENGAGE_ESTOP == RC_Engage) ||
        (RC_ENGAGE_FAILSAFE == RC_Engage))
    {
        if (RC_ENGAGE_ESTOP == RC_Engage)
        {
            if (s_estop_ticks < (uint16)PB_ESTOP_TICKS) { s_estop_ticks ++; }

            if (s_estop_ticks >= (uint16)PB_ESTOP_TICKS)
            {
                pb_finish(PB_ST_ABORT, "ESTOP");
            }

            /* ★ 去抖期间也必须立刻退出：v15 有安全层做授权仲裁，路线层写目标会被否掉；
               我们没有那一层（谁后写谁生效），这里若继续往下跑，就会在急停的这 4 拍里
               把 Remote_Ctrl 刚清 0 的速度/舵角目标重新写回去。 */
            return;
        }
        else
        {
            pb_finish(PB_ST_ABORT, (RC_ENGAGE_ON == RC_Engage) ? "RC ON" : "no RC & no link");
            return;
        }
    }
    else
    {
        s_estop_ticks = 0u;
    }

    if (ins_odom_enable < 0.5f)
    {
        pb_finish(PB_ST_ABORT, "odom off");
        return;
    }

    p = s_play;
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

    /* ---- 进度点：正向按几何，倒车/混合按本趟里程反查（= v15 route_pb.c:1150-1160）---- */
    if (0u != s_mileage)
    {
        best = pb_find_mileage(pb_travel(o.dist_abs));
    }
    else
    {
        best = pb_find_nearest(px, py, p);
    }

    /* ---- 换向：先停车驻留，确认停稳才换腿（= v15 route_pb.c:1161-1214）----
       为什么必须停：① 车还在往前滑就反号，等于让电机在高速下反向；
                    ② 换腿后的里程基准必须在"车真的停住"那一刻取，否则刹车过冲会把下一段推快。
       驻留期间保持旧腿的舵角、速度目标给 0（速度环主动刹，不撤使能），
       所以这期间**不清** s_rev，下面按 s_shift_idx-1 取方向仍是旧腿。 */
    if ((0u != s_mixed) && (s_shift_idx < (uint16)(s_n - 1u)))
    {
        uint32 now = (uint32)Sys_Tick_Ms;

        if ((0u == s_shifting) &&
            (pb_travel(o.dist_abs) >= (s_arc_tab[s_shift_idx] - PB_SHIFT_TRIG_M)))
        {
            s_shifting = 1u;
            s_stopped  = 0u;
            s_shift_t0 = now;
        }

        if (0u != s_shifting)
        {
            float mpr = (ins_m_per_rev > 1.0e-4f) ? ins_m_per_rev : 0.8193f;

            /* 驻留期：目标给 0，**不动使能位**（= v15 只写 spd_target、不碰 coast）。
               使能是速度段每拍决定的：跑着的时候本来就是 1 ⇒ 这里等价于主动刹；
               手推模式（pb_v = 0）是 0 ⇒ 保持自由轮，不跟操作员抢车。 */
            Motor_LR_Ctrl.target = 0.0f;
            Motor_RR_Ctrl.target = 0.0f;

            /* 停稳判据要三路都满足：odom 车速 + 左右轮实测转速（= v15 :1176-1178） */
            if ((fabsf(o.v_mps) <= PB_SHIFT_STOP_MPS) &&
                (fabsf(Motor_LR_Ctrl.actual * mpr) <= PB_SHIFT_STOP_MPS) &&
                (fabsf(Motor_RR_Ctrl.actual * mpr) <= PB_SHIFT_STOP_MPS))
            {
                if (0u == s_stopped) { s_stopped = 1u; s_stopped_t0 = now; }
            }
            else
            {
                s_stopped = 0u;
            }

            if ((0u != s_stopped) &&
                ((uint32)(now - s_stopped_t0) >= (uint32)PB_SHIFT_HOLD_MS))
            {
                uint16 i;

                /* 换腿确认：进度点、里程基准、下一个切换点一起更新 */
                s_idx       = best = s_shift_idx;
                s_leg_start = s_shift_idx;
                s_leg_d0    = o.dist_abs;
                s_drv_rev   = s_dir_tab[s_leg_start];   /* 新腿的方向（= v15 :1190）*/
                s_shift_idx = (uint16)(s_n - 1u);
                for (i = (uint16)(s_leg_start + 1u); (uint16)(i + 1u) < s_n; i ++)
                {
                    if (s_dir_tab[i] != s_dir_tab[i - 1u]) { s_shift_idx = i; break; }
                }
                s_shifting  = 0u;
                s_stopped   = 0u;
                s_depart    = 1u;           /* 出发：从 PB_SHIFT_CREEP 按 abrk 抬起 */
                s_depart_t0 = now;
                s_tail_stop = 0u;           /* 让表尾刹停逻辑重新计时 */
            }
            else
            {
                if ((uint32)(now - s_shift_t0) >= (uint32)PB_SHIFT_TIMEOUT_MS)
                {
                    pb_finish(PB_ST_ABORT, "shift timeout");
                    return;
                }

                s_idx = (uint16)(s_shift_idx - 1u);   /* 驻留期：进度停在旧腿末点 */
                return;
            }
        }
        else if (best >= s_shift_idx)
        {
            best = (uint16)(s_shift_idx - 1u);        /* 没到触发点前不许越进下一条腿 */
        }
    }

    s_idx = best;

    // ---- 方向：倒车段判定（标志位优先、速度兜底；只认**同一拍**的 p[best]）----
    // 存储结构不用改：REV 位（rec+14 bit2）与有符号的 v（rec+10）本来就在表里，
    // 以前只是没人读。见 route_pb.h 里"倒车段"那段注释。
    // ★ 换向不再需要单独的"往后滑过渡"等待：上面的换向驻留已经把车刹停了，
    //   方向切换只可能发生在 |v| ≤ PB_SHIFT_STOP_MPS 的那一拍。
    s_rev = s_dir_tab[best];

    /* ---- 首拍方向自检（= v15 route_pb.c:1542-1552）----
       表切向与车头差 > 120° ⇒ 方向开关与表不自洽（比如把车摆反了），拒绝起跑。
       两侧都是**车头朝向**（表的 yaw 列记的就是录制时的车头），正常情况这个差≈0。 */
    if (0u != s_dir_chk)
    {
        s_dir_chk = 0u;

        if (fabsf(pb_wrap180(p[best].yaw - pyaw)) > 120.0f)
        {
            pb_finish(PB_ST_ABORT, "dir mismatch");
            return;
        }
    }

    /* 内环增益跟着方向走：倒车段降到 1/3，见 pb_steer_gain() */
    pb_steer_gain((0u != s_rev) ? 1u : 0u);

    // 闭合表首尾相接：进度没走过阈值点就不许判终点，否则起跑第一拍就 DONE
    // 门限照 v15 取 3/4 表长（原来取 n/4 且封顶 30）：终点圈通常压在表尾，
    // 但只要表的首尾在空间上接近（闭环/来回），松门限就可能在起点附近提前判完成。
    if (0u == s_done_armed)
    {
        uint16 need = (uint16)((s_n * 3u) / 4u);

        if (need < 4u)             { need = 4u; }
        if (need > (uint16)(s_n - 1u)) { need = (uint16)(s_n - 1u); }

        if (best >= need) { s_done_armed = 1u; }
    }

    // 完成 / 中止
    // ★ 两条判据通道（= 队友 v15 route_pb.c:1255）：
    //   里程通道（混合/倒车表）：剩余弧长 < PB_FIN_M；
    //   几何通道（纯前进表）：离末点 < PB_FIN_M。
    //   ★ 纯前进表**不能**改成弧长判：末段点距可能大于 PB_FIN_M，弧长判要等 best 跳到末点才成立，
    //     而 best = 末点那一拍会先撞下面"off course"分支 ⇒ 正常跑完反而报脱线。
    //   ★ 混合表**不能**用几何判：两条腿在空间上贴得近时，车还在第一条腿就已经落在末点 0.15 m 内
    //     （那张 81 点的表就是这样被提前判 DONE 的）。
    // ★ 表尾是倒车段（起跑时读最后几个点判出来的 s_tail_rev）⇒ **先刹停再收**：
    //   这时车正在往后滑，直接 pb_finish()（放开电机、舵机回正）会让车继续退、
    //   方向盘还会自己回正。表尾是前进段则维持原来的行为。
    {
        float lx = px - p[s_n - 1u].x;
        float ly = py - p[s_n - 1u].y;
        float ld2 = sqrtf(lx * lx + ly * ly);                      // 离末点的直线距离 m
        float fin = (PB_FIN_M > 0.02f) ? PB_FIN_M : 0.02f;
        float travel = pb_travel(o.dist_abs);                      // 本趟已走弧长 m
        float len    = s_arc_tab[(uint16)(s_n - 1u)];              // 表总弧长 m
        float remain = len - travel;                               // 剩余弧长 m
        uint8 done;

        if (remain < 0.0f) { remain = 0.0f; }

        /* 跑飞：已走超过 2 倍表长还没到末点（= v15 :1237-1245） */
        if ((len > 0.5f) && ((o.dist_abs - s_d0) > ((2.0f * len) + 2.0f)) &&
            (s_idx < (uint16)(s_n - 1u)))
        {
            pb_finish(PB_ST_ABORT, "runaway 2x len");
            return;
        }

        if (0u != s_mileage)
        {
            /* 混合表：下一条腿还没换过去就不许判完成（s_shift_idx 已无待换的腿才放行） */
            done = (((0u == s_mixed) || (s_shift_idx == (uint16)(s_n - 1u))) &&
                    (remain < fin)) ? 1u : 0u;
        }
        else
        {
            done = ((0u != s_done_armed) && (ld2 < fin)) ? 1u : 0u;
        }

        if (0u != done)
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

    // 横向误差（正 = 车在参考线右侧）
    // ⚠ 倒车段 p[best].yaw 是车头朝向（与行进方向差 180°），这个量的符号会反；
    //   它只上报遥测、不参与回放决策，所以不为倒车单独修正（否则两处符号会打架）。
    {
        float tyaw = p[best].yaw * PB_D2R;

        s_elat = (px - p[best].x) * cosf(tyaw) + (py - p[best].y) * sinf(tyaw);
    }

    /* 脱表监测（= v15 :1290-1315）：只计数 + 告警，**不停车**。
       elat 沿切线跑偏时≈0，所以真实距离必须用点到折线的距离。 */
    s_dtrue = pb_seg_dist(px, py, p, s_n, best);

    if (s_dtrue > PB_OFF_DIST)
    {
        if (s_offcnt < 0xFFFFu) { s_offcnt ++; }
    }
    else
    {
        s_offcnt = 0u;
    }

    // ---- 目标速度：查**预建剖面**在本段的限速（= v15 route_pb.c:1317-1347）----
    // pb_v 从此是"车速上限"（剖面里当 vmax 用），不再是恒定车速；
    // pb_ve 这个开关不再起作用（剖面恒建），保留只是为了不动参数 ID。
    {
        float v    = 0.0f;
        float vmax = fabsf(ROUTE_PB_V);
        float abrk = PB_ABRK_MPS2;
        uint8 pwr  = (vmax >= 0.005f) ? 1u : 0u;    /* 0 = 手推（自由轮），剖面不参与 */

        if (0u != pwr)
        {
            if (vmax > PB_V_MAX) { vmax = PB_V_MAX; }
            if (abrk < 0.05f)    { abrk = 0.05f; }

            v = s_vlim[best];

            /* 刚换腿那一点：用下一点的限速，别被"末点锚"的 creep 卡住（= v15 :1322 要求 s_depart） */
            if ((0u != s_mixed) && (0u != s_depart) && (best == s_leg_start) &&
                ((uint16)(best + 1u) < s_n))
            {
                v = s_vlim[best + 1u];
            }

            /* 换向点在眼前：按 abrk 收尾，并给一个爬行地板（低于它就不走了） */
            if ((0u != s_mixed) && (s_shift_idx < (uint16)(s_n - 1u)))
            {
                float remain = s_arc_tab[s_shift_idx] - pb_travel(o.dist_abs);
                float cap    = sqrtf(2.0f * abrk * ((remain > 0.0f) ? remain : 0.0f));

                if (cap < PB_SHIFT_CREEP) { cap = PB_SHIFT_CREEP; }
                if (v > cap) { v = cap; }
            }

            /* 出发斜坡：换腿后从爬行速度按 abrk 抬起来，不许一步跳回巡航 */
            if (0u != s_depart)
            {
                float elapsed = (float)(uint32)(Sys_Tick_Ms - s_depart_t0) * 0.001f;
                float cap     = PB_SHIFT_CREEP + (abrk * elapsed);

                if (v > cap) { v = cap; } else { s_depart = 0u; }
            }

            if (0u != s_drv_rev) { v = -v; }         /* 倒车 = 整条剖面翻号（= v15 :1344） */
            if (v >  vmax) { v =  vmax; }
            if (v < -vmax) { v = -vmax; }

            /* ★ 我方保留的安全钳：v15 没有单独的倒车限速，倒车段只被 ±pb_vmax 封顶
               （一条 1.5 m/s 的倒车是允许的）。27° 满舵下这个速度不可接受，
               所以仍用统一过的 PB_REV_VMAX 再夹一次。要去掉就删这一行。 */
            if (v < -PB_REV_VMAX) { v = -PB_REV_VMAX; }
        }

        s_tv = v;                               /* 供前视距离用（= v15 的 s_tv） */

        if ((0u == pwr) || ((v < 0.005f) && (v > -0.005f)))     // 手推 或 |v| 太小：自由轮
        {
            Motor_Ctrl_Enable    = 0;
            Motor_LR_Ctrl.target = 0.0f;
            Motor_RR_Ctrl.target = 0.0f;
        }
        else
        {
            float mpr  = (ins_m_per_rev > 1.0e-4f) ? ins_m_per_rev : 0.8193f;
            float vrps = v / mpr;

            Motor_Ctrl_Enable = 1;
            Motor_Diff_Alloc(vrps, Servo_Angle_Deg, ROUTE_PB_L,
                             &Motor_LR_Ctrl.target, &Motor_RR_Ctrl.target);
        }
    }

    // ---- 倒车段：关外环 + 开环回放录制打角，然后**跳过**整段纯跟踪 ----
    // 为什么必须跳过：下面 α = atan2(bx,by) 与 target_yaw = o.yaw − α 都按"车头朝前"推导，
    // 倒车时前视点天然落在车后（实测那张表 α≈−178°），继续闭环会把舵机顶到满舵、
    // 而电机是反转的 ⇒ 车辆绕圈跑飞。开环回放录制打角才是对的（那一拍车就是这么走的）。
    if (0u != s_rev)
    {
        float d = 0.0f;

        /* 该点没有打角记录（flags bit0 = 0）时只用航向 P，不当作"直行"（= v15 :1483） */
        if (0u != (p[best].flags & (uint16)INS_WP_FL_STEER))
        {
            d = (float)p[best].steer_cdeg * 0.01f;
        }

        /* ---- 倒车航向 P 纠偏（= v15 route_pb.c:1462-1485）----
           err = 表在该点的车头朝向 − 实测车头朝向（都在表系，左正）；
           ★ 纠偏项符号与前进段航向外环**相反**：倒车时前轮在后，转向对航向的作用反向。
             所以这里是 "+k·err"，而前进段是 "−k·err"。
           实测朝向用 pyaw（= p[0].yaw + 起跑后转过的角），它已经是表系，
           不需要 v15 那样再单独对齐一次 IMU 零位。 */
        {
            float corr = PB_REV_HEAD_KP * pb_wrap180(p[best].yaw - pyaw);

            if (corr >  PB_REV_CORR_MAX) { corr =  PB_REV_CORR_MAX; }
            if (corr < -PB_REV_CORR_MAX) { corr = -PB_REV_CORR_MAX; }

            d += corr;
        }

        if (d >  PB_DELTA_MAX) { d =  PB_DELTA_MAX; }
        if (d < -PB_DELTA_MAX) { d = -PB_DELTA_MAX; }

        s_ld  = 0.0f;                   // 倒车段没有前视距离这个概念
        s_aim = best;

        Head_Ctrl_Set_Enable(0);
        Servo_Target_Deg = d;
        return;
    }

    // 前视距离 ld = a + b·|v_tgt|（= v15 route_pb.c:1383-1388）
    // 用**目标**速度不用实测：剖面已按几何算好该多快，跟实测会被打滑带偏
    s_ld = PB_LD_A + (PB_LD_B * fabsf(s_tv));

    if (s_ld < PB_LD_MIN) { s_ld = PB_LD_MIN; }
    if (s_ld > PB_LD_MAX) { s_ld = PB_LD_MAX; }
    if (s_ld < 0.05f)     { s_ld = 0.05f; }

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
                Head_Ctrl_Set_Enable(0);
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
            // 实际施加由 5 ms 中断里的 Head_Ctrl_Tick() 完成，这里只给目标与增益。
            // pb_kh > 0 时用手填值覆盖（做 A/B 用），见 route_pb.h 的说明。
            float k = (ROUTE_PB_KH > 1.0e-3f) ? ROUTE_PB_KH : (2.0f * ROUTE_PB_L / s_ld);

            if (k < PB_K_MIN) { k = PB_K_MIN; }
            if (k > PB_K_MAX) { k = PB_K_MAX; }

            Head_Ctrl_Set_Kp(k);
            Head_Ctrl_Set_Target(pb_wrap180(o.yaw - alpha * PB_R2D));
            Head_Ctrl_Set_Enable(1);
        }
        else
        {
            // 直接给 δ
            float kappa = 2.0f * sinf(alpha) / s_ld;
            float delta = atanf(ROUTE_PB_L * kappa) * PB_R2D;

            if (delta >  PB_DELTA_MAX) { delta =  PB_DELTA_MAX; }
            if (delta < -PB_DELTA_MAX) { delta = -PB_DELTA_MAX; }

            Head_Ctrl_Set_Enable(0);
            Servo_Target_Deg = delta;
        }
    }
}

uint8 route_pb_active (void) { return (PB_ST_RUN == s_st) ? 1u : 0u; }
uint8 route_pb_releasing (void)
{
    if (0u == s_rel_pend) { return 0u; }

    /* poll() 停了（切页/换模式）拍数就不再推进 ⇒ 用时间收口，别让 Route 页一直让着方向盘 */
    return ((uint32)(Sys_Tick_Ms - s_rel_ms) < (uint32)PB_REL_MS) ? 1u : 0u;
}
uint8 route_pb_state  (void) { return s_st; }
float route_pb_elat   (void) { return s_elat; }
float route_pb_ld     (void) { return s_ld; }
uint16 route_pb_aim   (void) { return s_aim; }
uint16 route_pb_index (void) { return s_idx; }

const char *route_pb_reason (void) { return s_why; }
uint8 route_pb_why_code (void) { return s_why_code; }
float route_pb_off_dist (void) { return s_dtrue; }
uint16 route_pb_off_ticks (void) { return s_offcnt; }

/**********************************************************/
/*----------------------------------------[E] 接口实现 [E]*/
/**********************************************************/
