/*********************************************************************************************************************
 * 路表录制与 Flash 存取实现
 *
 * ins_ = INS（Inertial Navigation System，惯导）子系统前缀，本文件属"惯导 + 路表"系列
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "ins_odom.h"
#include "ins_imu.h"
#include "ins_route.h"
#include "route_pb.h"       /* route_pb_active()：回放中不许改写 s_wp[]（= 队友 v15 ins_route.c:11） */

#define DEG2RAD_F   (0.017453292519943295f)

static ins_wp_t          s_wp[INS_ROUTE_MAX_POINTS];
static ins_route_head_t  s_head;

static uint8   s_running   = 0;
static uint8   s_flags     = 0;
static float   s_dist_acc  = 0.0f;      /* 距上一个已存点的累计距离 */
static float   s_dist_last = 0.0f;      /* 上一次采样的 dist_abs */
static float   s_dist_at_start = 0.0f;  /* 起点处的 dist_abs */

static uint16  s_key_cnt   = 0u;
static uint16  s_steer_cnt = 0u;
static uint8   s_rec_reverse = 0u;      /* 当前录制段的行驶方向（REV 位用），带滞回 */

volatile float ins_route_ds = INS_ROUTE_DS_DEFAULT;

//--------------------------------------
// 函数简介     初始化（清缓冲 + 夹点距）
//--------------------------------------
void ins_route_init (void)
{
    if (ins_route_ds < INS_ROUTE_DS_MIN) { ins_route_ds = INS_ROUTE_DS_MIN; }
    if (ins_route_ds > INS_ROUTE_DS_MAX) { ins_route_ds = INS_ROUTE_DS_MAX; }

    s_head.origin_x   = 0.0f;
    s_head.origin_y   = 0.0f;
    s_head.origin_yaw = 0.0f;
    s_head.count      = 0;
    s_head.version    = 1;
    s_head.total_dist = 0.0f;

    s_running   = 0;
    s_flags     = 0;
    s_dist_acc  = 0.0f;
    s_dist_last = 0.0f;
    s_key_cnt   = 0u;
    s_steer_cnt = 0u;
}

//--------------------------------------
// 函数简介     只清缓冲（起点不动）
//--------------------------------------
void ins_route_clear (void)
{
    /* 互斥闸门（= 队友 v15 ins_route.c:62）：录制中 / 回放中 / 写盘中都不许清表，
       否则回放的点序会被人从背后换掉 */
    if (route_pb_active() || ins_route_flash_busy() || s_running) { return; }

    s_head.count      = 0;
    s_head.total_dist = 0.0f;
    s_dist_acc  = 0.0f;
    s_flags    &= (uint8)~INS_ROUTE_ST_FULL;
    s_key_cnt   = 0u;
    s_steer_cnt = 0u;
}

//--------------------------------------
// 函数简介     标记最后一个已录点（关键点）
// 返回参数     uint8	1 = 标上了；0 = 没在录 / 点太少
//--------------------------------------
uint8 ins_route_mark_key (void)
{
    uint16 last;

    if (0 == s_running)     { return 0u; }
    if (s_head.count < 2u)  { return 0u; }

    last = (uint16)(s_head.count - 1u);
    if (0u == (s_wp[last].flags & (uint16)INS_WP_FL_KEY))
    {
        s_wp[last].flags |= (uint16)INS_WP_FL_KEY;
        s_key_cnt ++;
    }
    return 1u;
}

uint16 ins_route_key_count   (void) { return s_key_cnt;   }
uint16 ins_route_steer_count (void) { return s_steer_cnt; }

//--------------------------------------
// 函数简介     世界位姿换算到录制起点车体系
//--------------------------------------
static void route_to_local (const ins_odom_snap_t *o, uint16 idx, float v_mps)
{
    float  dx = o->x - s_head.origin_x;
    float  dy = o->y - s_head.origin_y;
    float  a0 = s_head.origin_yaw * DEG2RAD_F;
    float  c  = cosf(a0);
    float  s  = sinf(a0);
    float  d;
    uint16 fl = 0u;

    s_wp[idx].x   =  c * dx + s * dy;
    s_wp[idx].y   = -s * dx + c * dy;
    s_wp[idx].yaw =  o->yaw - s_head.origin_yaw;
    /* ★ 表里只存速度**幅值**，方向只由 REV 位承载（= 队友 v15 ins_route.c:115）。
       以前按 rt_rrec 开关决定存不存负号，回放又拿 p[best].v 的符号当倒车速度 ——
       方向有两个来源，剖面一旦要"整条翻号"就会打架。
       旧表里 v 仍可能是负的：回放判据是 (REV 位 || v < 0)，历史表照样能读。 */
    s_wp[idx].v   =  fabsf(v_mps);

    /* 打角列：记**实测**方向盘角，不是指令角也不是遥控角。
       ★ 为什么不用指令值：手推录制时舵机失能，指令列（Servo_Target_Applied_Deg）恒为 0，
         但 STEER 位照置 —— 录出来的倒车段回放时一律打 0°，而回放拿的正是这一列。
       ★ 为什么不用遥控指令角：RC 是指令，实测角才是车轮真正经历的转角。
       ★ 实测角始终有效（Servo_Ctrl.c:131 无条件更新，与使能无关）；
         失效时用 Servo_Enc_Fault 拦，避免把 0/4095 的假读数写成打角。
       ★ 代价（可算，不是"可能有点滞后"）：内环稳态 实测 = 指令 + b，b = 静差。
         记指令 ⇒ 回放喂同一个环，物理角 = C+b，误差只出现一次、轨迹可复现；
         记实测 ⇒ 回放喂 M=C+b，物理角 = C+2b，误差被吃两次。
         实测 b = −1.08°（steer_tgt=0 时 steer_deg=−1.08）⇒ 满舵 27° 的转弯半径
         1.296 m 变 1.356 m，偏大 4.6%。**静差修掉后这一项自动归零。** */
    d = (0u == Servo_Enc_Fault) ? Servo_Angle_Deg : 999.0f;
    if ((d > -40.0f) && (d < 40.0f))
    {
        s_wp[idx].steer_cdeg = (int16)(d * 100.0f);
        fl |= (uint16)INS_WP_FL_STEER;
        s_steer_cnt ++;
    }
    else
    {
        s_wp[idx].steer_cdeg = 0;
    }

    /* 方向带滞回（= 队友 v15 ins_route.c:130-132）：静止点没有方向信息，
       继承上一段的方向。瞬时判据会让停车那一段丢方向，换向点位置跟着漂。 */
    if (v_mps < -0.02f)      { s_rec_reverse = 1u; }
    else if (v_mps > 0.02f)  { s_rec_reverse = 0u; }
    if (0u != s_rec_reverse) { fl |= (uint16)INS_WP_FL_REV; }

    s_wp[idx].flags = fl;
}

//--------------------------------------
// 函数简介     开始录制（重新捕获起点）
//--------------------------------------
void ins_route_start (void)
{
    ins_odom_snap_t o;

    /* 互斥闸门（= 队友 v15 ins_route.c:142）：回放中 / 写盘中不许起录 */
    if (route_pb_active() || ins_route_flash_busy() || s_running) { return; }

    ins_odom_snapshot(&o);

    s_head.origin_x   = o.x;
    s_head.origin_y   = o.y;
    s_head.origin_yaw = o.yaw;
    s_head.count      = 0;
    s_head.total_dist = 0.0f;
    s_head.version    = 1;

    s_dist_acc      = 0.0f;
    s_dist_last     = o.dist_abs;
    s_dist_at_start = o.dist_abs;
    s_flags         = INS_ROUTE_ST_RUNNING;

    s_key_cnt   = 0u;
    s_steer_cnt = 0u;

    /* 起点方向预设（= v15 ins_route.c:144-146 按 rt_route_mode）：
       用"倒车录表"开关当初始方向，车一动就由实测速度接管。
       不预设的话，倒着起步的头几点会先被标成前进。 */
    s_rec_reverse = (ROUTE_REV_REC >= 0.5f) ? 1u : 0u;

    route_to_local(&o, 0, 0.0f);
    s_head.count = 1;

    ins_imu_zupt_frz_reset();

    s_running = 1;
}

//--------------------------------------
// 函数简介     停止录制
// 备注信息     ★ 收笔补一点：表尾 = 车真正停住那一刻的位置与航向（= v15 ins_route.c:178-200）。
//   为什么需要：采样按**里程**触发（每走 ins_route_ds 落一点），按下停止时若距上一点
//   还不到一个点距，最后那一段就没有点；而终点往往正是打着大角的地方。
//   判据只有**距离 ≥20 mm**；打角不再另设判据（v15 也没有）。
//   v 强制写 0：末点是"停住"，不是"还在跑"；带当拍速度会让回放在表尾继续给目标速度。
//   20 mm 同时挡住"完全重合的点"——零长度线段会让最近点搜索选中重复点、前视向量退化。
//
//   与 v15 的差异只剩一处：ROUTE_END_PT 开关门控（v15 恒执行）。删参数 ID 会动到参数表
//   契约，所以保留开关，默认 1 与 v15 行为一致。
//--------------------------------------
void ins_route_stop (void)
{
    if (0 == s_running) { return; }             /* 没在录：直接返回，重复调用必须无副作用 */

    if ((ROUTE_END_PT >= 0.5f) && (s_head.count > 0u) &&
        (s_head.count < INS_ROUTE_MAX_POINTS))
    {
        ins_odom_snap_t o;
        uint16 idx = s_head.count;              /* 候选点写在第 count 槽，count 先不加 */
        uint16 sc  = s_steer_cnt;
        float  dx, dy;

        ins_odom_snapshot(&o);
        route_to_local(&o, idx, 0.0f);           /* ★ v = 0：末点是停住那一拍 */

        dx = s_wp[idx].x - s_wp[idx - 1u].x;
        dy = s_wp[idx].y - s_wp[idx - 1u].y;

        if ((dx * dx + dy * dy) >= (0.020f * 0.020f))
        {
            s_head.total_dist = o.dist_abs - s_dist_at_start;
            s_head.count      = (uint16)(idx + 1u);

            if (s_head.count >= INS_ROUTE_MAX_POINTS)
            {
                s_flags |= INS_ROUTE_ST_FULL;
            }
        }
        else
        {
            s_steer_cnt = sc;                   /* 没采用：把打角计数回滚 */
        }
    }

    s_running = 0;
    s_flags  &= (uint8)~INS_ROUTE_ST_RUNNING;
}

//--------------------------------------
// 函数简介     主循环每拍：按里程采样
//--------------------------------------
void ins_route_poll (void)
{
    ins_odom_snap_t o;
    float ds;
    uint8 append;

    if (!s_running || (s_head.count >= INS_ROUTE_MAX_POINTS))
    {
        if (s_running && (s_head.count >= INS_ROUTE_MAX_POINTS))
        {
            s_flags |= INS_ROUTE_ST_FULL;
            s_running = 0;
            s_flags &= (uint8)~INS_ROUTE_ST_RUNNING;
        }
        return;
    }

    ins_odom_snapshot(&o);

    /* 本拍位移用 dist_abs 的差值：标量、单调增 */
    ds = o.dist_abs - s_dist_last;
    if (ds < 0.0f) { ds = 0.0f; }
    s_dist_last = o.dist_abs;
    s_dist_acc += ds;

    s_head.total_dist = o.dist_abs - s_dist_at_start;

    append = (s_dist_acc >= ins_route_ds) ? 1u : 0u;

    if (append)
    {
        route_to_local(&o, s_head.count, o.v_mps);

        s_head.count ++;

        /* 记一点后立即清零，**不留欠账**（= v15 ins_route.c:252-255）。
           留欠账时，车静止或蠕行会每拍都满足阈值，写出一串位置相同的重复点；
           零长度线段会让回放的最近点搜索选中重复点、前视向量退化成 0。
           表长不靠点距累加：回放侧 total_dist 由 s_arc 的弦长和给出。 */
        s_dist_acc = 0.0f;

        if (s_head.count >= INS_ROUTE_MAX_POINTS)
        {
            s_flags |= INS_ROUTE_ST_FULL;
            s_running = 0;
            s_flags &= (uint8)~INS_ROUTE_ST_RUNNING;
        }
    }
}

uint8  ins_route_is_running (void)      { return s_running; }
uint8  ins_route_flags      (void)      { return s_flags; }
uint16 ins_route_get_count  (void)      { return s_head.count; }
float  ins_route_get_total_dist (void)  { return s_head.total_dist; }

const ins_wp_t         *ins_route_get_points (void) { return s_wp; }
const ins_route_head_t *ins_route_get_head   (void) { return &s_head; }

/*===================================================================================================================
 *  Flash 存取
 *=================================================================================================================*/
#define RT_MAGIC            (0x4B525443u)       /* "CTRK" */
#define RT_VER              (2u)                /* v2 = 16 B/条 */

typedef struct
{
    int32   x_mm;           /* 相对录制起点，mm（向右） */
    int32   y_mm;           /* 相对录制起点，mm（向前） */
    int16   yaw_cdeg;       /* 相对录制起点航向，0.01° */
    int16   v_cmps;         /* 车速，cm/s */
    int16   steer_cdeg;     /* 实际施加的舵角，0.01° */
    uint16  flags;          /* INS_WP_FL_* */
} rt_rec_t;

typedef struct
{
    uint32  magic;
    uint16  ver;
    uint16  rec_len;
    uint32  count;
    uint32  ds_mm;
    uint32  lap_mm;
    int32   origin_x_mm;
    int32   origin_y_mm;
    int16   origin_yaw_cdeg;
    uint16  flags;
    uint16  crc_r;          /* 记录区 CRC16 */
    uint16  crc_h;          /* 表头 CRC16，覆盖 [0, 34) */
} rt_head_t;

/* 字段按 4/2 字节天然对齐，不加 packed；布局变了这两条守卫会报错 */
typedef char rt_head_size_check[(sizeof(rt_head_t) == 36u) ? 1 : -1];
typedef char rt_head_off_check [(offsetof(rt_head_t, crc_h) == 34u) ? 1 : -1];
typedef char rt_rec_size_check [(sizeof(rt_rec_t) == INS_RT_REC_SIZE) ? 1 : -1];

static uint8   s_fl_busy   = 0u;
static uint8   s_fl_rc     = 0u;        /* 0 未做 / 1 成功 / 2 写失败 / 3 表空 / 4 槽号错 */
static uint8   s_fl_loaded = 0u;
static uint16  s_fl_page   = 0u;
static uint16  s_fl_pages  = 0u;
static uint16  s_fl_crc    = 0u;
static uint16  s_fl_done   = 0u;

static uint8   s_fl_slot   = 0u;
static uint32  s_fl_base   = 0u;
static uint8   s_fl_last_slot = 0u;
static uint8   s_active_slot  = 0u;

static uint32  s_fl_count  = 0u;
static uint32  s_fl_ds_mm  = 0u;
static uint32  s_fl_lap_mm = 0u;
static int32   s_fl_ox_mm  = 0;
static int32   s_fl_oy_mm  = 0;
static int16   s_fl_oyaw   = 0;
static uint16  s_fl_flags  = 0u;

/* 槽索引页：active_slot + 每槽摘要 + CRC */
#define RT_IDX_MAGIC    (0x58495243u)   /* "CRIX" */
#define RT_IDX_VER      (1u)

typedef struct
{
    uint32  magic;
    uint16  ver;
    uint16  slot_count;
    uint8   active_slot;
    uint8   rsv[3];
    struct
    {
        uint8   valid;
        uint8   slot;
        uint16  tag;
        uint32  count;
        uint32  lap_mm;
        uint32  ds_mm;
        uint32  crc_r;
    } s[INS_RT_SLOT_COUNT];
    uint16  crc;
} rt_idx_t;

typedef char rt_idx_off_check[(offsetof(rt_idx_t, crc) == 12u + 20u * INS_RT_SLOT_COUNT) ? 1 : -1];
typedef char rt_idx_size_check[(sizeof(rt_idx_t) <= 2048u) ? 1 : -1];

//--------------------------------------
// 函数简介     CRC16-MODBUS
//--------------------------------------
static uint16 rt_crc16 (uint16 crc, const uint8 *p, uint32 n)
{
    uint32 i;
    uint8  j;

    for (i = 0; i < n; i ++)
    {
        crc ^= (uint16)p[i];
        for (j = 0; j < 8u; j ++)
        {
            if (0u != (crc & 1u)) { crc = (uint16)((crc >> 1) ^ 0xA001u); }
            else                  { crc = (uint16)(crc >> 1); }
        }
    }
    return crc;
}

/* 浮点转整数：半 LSB 处远离 0 取整，正负对称（= v15 rt_round_i32）。
   C 的截断对负数偏向 +inf，反复"落盘 → 载入"会朝一个方向累积漂移。 */
static int32 rt_round_i32 (float v)
{
    return (int32)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

static int16 rt_wrap_cdeg (float yaw_deg)
{
    float y = yaw_deg;

    while (y >  180.0f) { y -= 360.0f; }
    while (y < -180.0f) { y += 360.0f; }

    return (int16)rt_round_i32(y * 100.0f);
}

static float rt_clampf (float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static void rt_rec_pack (const ins_wp_t *wp, rt_rec_t *r)
{
    r->x_mm       = rt_round_i32(wp->x * 1000.0f);
    r->y_mm       = rt_round_i32(wp->y * 1000.0f);
    r->yaw_cdeg   = rt_wrap_cdeg(wp->yaw);
    r->v_cmps     = (int16)rt_round_i32(rt_clampf(wp->v * 100.0f, -32000.0f, 32000.0f));
    r->steer_cdeg = wp->steer_cdeg;
    r->flags      = wp->flags;
}

//--------------------------------------
// 函数简介     读槽索引（无效时返回全空）
//--------------------------------------
static void rt_idx_load (rt_idx_t *d)
{
    rt_idx_t r;

    memset(d, 0, sizeof(*d));
    d->magic      = RT_IDX_MAGIC;
    d->ver        = (uint16)RT_IDX_VER;
    d->slot_count = (uint16)INS_RT_SLOT_COUNT;

    flash_read_page_to_buffer(0u, INS_RT_INDEX_PAGE, (uint32)FLASH_PAGE_LENGTH);
    memcpy(&r, (const void *)flash_union_buffer, sizeof(r));

    if ((RT_IDX_MAGIC == r.magic) &&
        ((uint16)RT_IDX_VER == r.ver) &&
        ((uint16)INS_RT_SLOT_COUNT == r.slot_count) &&
        (rt_crc16(0xFFFFu, (const uint8 *)&r, (uint32)offsetof(rt_idx_t, crc)) == r.crc))
    {
        *d = r;
    }
    else
    {
        uint8 k;

        for (k = 0u; k < (uint8)INS_RT_SLOT_COUNT; k ++) { d->s[k].slot = k; }
    }

    if (d->active_slot >= (uint8)INS_RT_SLOT_COUNT) { d->active_slot = 0u; }
}

//--------------------------------------
// 函数简介     整页重写槽索引
//--------------------------------------
static void rt_idx_store (const rt_idx_t *src)
{
    rt_idx_t w = *src;

    w.magic      = RT_IDX_MAGIC;
    w.ver        = (uint16)RT_IDX_VER;
    w.slot_count = (uint16)INS_RT_SLOT_COUNT;
    w.crc        = rt_crc16(0xFFFFu, (const uint8 *)&w, (uint32)offsetof(rt_idx_t, crc));

    memset((void *)flash_union_buffer, 0xFF, 2048u);
    memcpy((void *)flash_union_buffer, &w, sizeof(w));

    flash_erase_page(0u, INS_RT_INDEX_PAGE);
    (void)flash_write_page_from_buffer(0u, INS_RT_INDEX_PAGE, (uint32)FLASH_PAGE_LENGTH);
}

//--------------------------------------
// 函数简介     武装一次分步写盘（只快照，不擦写）
//--------------------------------------
static void flash_arm (uint8 slot, uint32 count, uint32 ds_mm, uint32 lap_mm,
                       int32 ox, int32 oy, int16 oyaw, uint16 flags)
{
    if (slot >= (uint8)INS_RT_SLOT_COUNT) { s_fl_busy = 0u; s_fl_rc = 4u; return; }
    if (0u == count)                      { s_fl_busy = 0u; s_fl_rc = 3u; return; }
    if (count > INS_RT_FLASH_CAPACITY)    { count = INS_RT_FLASH_CAPACITY; }

    flash_init();

    s_fl_slot  = slot;
    s_fl_base  = INS_RT_SLOT_BASE(slot);
    s_fl_pages = (uint16)((count + (INS_RT_REC_PER_PAGE - 1u)) / INS_RT_REC_PER_PAGE);
    s_fl_page  = 0u;
    s_fl_crc   = 0xFFFFu;
    s_fl_done  = 0u;
    s_fl_rc    = 0u;
    s_fl_busy  = 1u;

    s_fl_count  = count;
    s_fl_ds_mm  = ds_mm;
    s_fl_lap_mm = lap_mm;
    s_fl_ox_mm  = ox;
    s_fl_oy_mm  = oy;
    s_fl_oyaw   = oyaw;
    s_fl_flags  = flags;
}

//--------------------------------------
// 函数简介     把当前 RAM 里的表写进指定槽（分步）
//--------------------------------------
void ins_route_flash_save_slot (uint8 slot)
{
    /* 互斥闸门：录制中（表还在长）/ 回放中（点序可能被原地翻过）/ 写盘中
       —— 这三种状态下存出去的都不是"这张表" */
    if (route_pb_active() || s_running || ins_route_flash_busy()) { return; }

    flash_arm(slot, (uint32)s_head.count,
              (uint32)(ins_route_ds * 1000.0f + 0.5f),
              (uint32)(s_head.total_dist * 1000.0f + 0.5f),
              (int32)(s_head.origin_x * 1000.0f),
              (int32)(s_head.origin_y * 1000.0f),
              rt_wrap_cdeg(s_head.origin_yaw),
              (uint16)((0u != (s_flags & INS_ROUTE_ST_FULL)) ? 1u : 0u));
}

//--------------------------------------
// 函数简介     分步写盘：每次调用最多写一页
//--------------------------------------
void ins_route_flash_poll (void)
{
    uint8 *pg = (uint8 *)flash_union_buffer;

    if (0u == s_fl_busy) { return; }

    /* --- 记录页 --- */
    if (s_fl_page < s_fl_pages)
    {
        uint16 k;

        memset(pg, 0xFF, 2048u);

        for (k = 0u; k < (uint16)INS_RT_REC_PER_PAGE; k ++)
        {
            uint32 idx = (uint32)s_fl_page * INS_RT_REC_PER_PAGE + k;
            rt_rec_t r;

            if (idx >= s_fl_count) { break; }

            rt_rec_pack(&s_wp[idx], &r);
            memcpy(pg + (uint32)k * INS_RT_REC_SIZE, &r, INS_RT_REC_SIZE);
        }

        /* 只对真正写进去的字节算 CRC */
        {
            uint32 used = (uint32)s_fl_page * INS_RT_REC_PER_PAGE;
            uint32 left = (s_fl_count > used) ? (s_fl_count - used) : 0u;

            if (left > INS_RT_REC_PER_PAGE) { left = INS_RT_REC_PER_PAGE; }
            s_fl_crc = rt_crc16(s_fl_crc, pg, left * INS_RT_REC_SIZE);
        }

        flash_erase_page(0u, (uint32)(s_fl_base + 1u + (uint32)s_fl_page));
        (void)flash_write_page_from_buffer(0u, (uint32)(s_fl_base + 1u + (uint32)s_fl_page),
                                           (uint32)FLASH_PAGE_LENGTH);

        s_fl_page ++;
        s_fl_done ++;

        /* 首记录页读回校验（先抄期望值，读回会覆盖同一个缓冲） */
        if (1u == s_fl_page)
        {
            uint8 exp[16];

            memcpy(exp, pg, sizeof(exp));

            flash_read_page_to_buffer(0u, (uint32)(s_fl_base + 1u), (uint32)FLASH_PAGE_LENGTH);
            if (0 != memcmp(exp, (const void *)flash_union_buffer, sizeof(exp)))
            {
                s_fl_busy = 0u;
                s_fl_rc   = 2u;
                return;
            }
        }
        return;
    }

    /* --- 最后写表头页 + 更新槽索引 --- */
    {
        rt_head_t h;

        memset(&h, 0, sizeof(h));
        h.magic           = RT_MAGIC;
        h.ver             = (uint16)RT_VER;
        h.rec_len         = (uint16)INS_RT_REC_SIZE;
        h.count           = s_fl_count;
        h.ds_mm           = s_fl_ds_mm;
        h.lap_mm          = s_fl_lap_mm;
        h.origin_x_mm     = s_fl_ox_mm;
        h.origin_y_mm     = s_fl_oy_mm;
        h.origin_yaw_cdeg = s_fl_oyaw;
        h.flags           = s_fl_flags;
        h.crc_r           = s_fl_crc;
        h.crc_h           = rt_crc16(0xFFFFu, (const uint8 *)&h, 34u);

        memset((void *)flash_union_buffer, 0xFF, 2048u);
        memcpy((void *)flash_union_buffer, &h, sizeof(h));

        flash_erase_page(0u, s_fl_base);
        (void)flash_write_page_from_buffer(0u, s_fl_base, (uint32)FLASH_PAGE_LENGTH);

        {
            rt_head_t rb;

            flash_read_page_to_buffer(0u, s_fl_base, (uint32)FLASH_PAGE_LENGTH);
            memcpy(&rb, (const void *)flash_union_buffer, sizeof(rb));
            if (0 != memcmp(&rb, &h, sizeof(h)))
            {
                s_fl_busy = 0u;
                s_fl_rc   = 2u;
                s_fl_done ++;
                return;
            }
        }

        {
            rt_idx_t d;
            uint8    k = s_fl_slot;

            rt_idx_load(&d);
            d.s[k].valid  = 1u;
            d.s[k].slot   = k;
            d.s[k].count  = h.count;
            d.s[k].lap_mm = h.lap_mm;
            d.s[k].ds_mm  = h.ds_mm;
            d.s[k].crc_r  = h.crc_r;
            d.active_slot = k;
            rt_idx_store(&d);
            s_active_slot = k;
        }
    }

    s_fl_last_slot = s_fl_slot;
    s_fl_busy      = 0u;
    s_fl_rc        = 1u;
    s_fl_done ++;
    s_fl_loaded    = 1u;
}

uint8 ins_route_flash_busy   (void) { return s_fl_busy; }
uint8 ins_route_flash_last_rc(void) { return s_fl_rc; }
uint8 ins_route_flash_loaded (void) { return s_fl_loaded; }
uint8 ins_route_flash_last_slot (void) { return s_fl_last_slot; }

uint8 ins_route_flash_progress (void)
{
    uint16 total = (uint16)(s_fl_pages + 1u);

    if (0u == total)          { return 0u; }
    if (s_fl_done >= total)   { return 100u; }
    return (uint8)((s_fl_done * 100u) / total);
}

//--------------------------------------
// 函数简介     从指定槽载入到 RAM
// 返回参数     uint8	1 = 成功；0 = 无表 / 校验不过
//--------------------------------------
uint8 ins_route_flash_load_slot (uint8 slot)
{
    rt_head_t h;

    /* 互斥闸门（= 队友 v15 ins_route.c:786）：回放中载入会把 s_wp[] 换成另一张表，
       而回放正按自己的进度点在里面取值 */
    if (route_pb_active() || s_running || ins_route_flash_busy()) { return 0u; }
    uint32    n;
    uint32    base;

    s_fl_loaded = 0u;

    if (slot >= (uint8)INS_RT_SLOT_COUNT) { return 0u; }
    base = INS_RT_SLOT_BASE(slot);

    flash_init();
    flash_read_page_to_buffer(0u, base, (uint32)FLASH_PAGE_LENGTH);
    memcpy(&h, (const void *)flash_union_buffer, sizeof(h));

    if (RT_MAGIC != h.magic)                                  { return 0u; }
    if ((uint16)RT_VER != h.ver)                              { return 0u; }
    if ((uint16)INS_RT_REC_SIZE != h.rec_len)                 { return 0u; }
    if (0u == h.count)                                        { return 0u; }
    if (h.count > INS_RT_FLASH_CAPACITY)                      { return 0u; }
    if (h.count > (uint32)INS_ROUTE_MAX_POINTS)               { return 0u; }
    if (rt_crc16(0xFFFFu, (const uint8 *)&h, 34u) != h.crc_h) { return 0u; }

    n = h.count;

    {
        uint16 crc_r = 0xFFFFu;
        uint32 done  = 0u;

        while (done < n)
        {
            uint32 page = done / INS_RT_REC_PER_PAGE;
            uint32 left = n - done;
            uint32 k;

            if (left > INS_RT_REC_PER_PAGE) { left = INS_RT_REC_PER_PAGE; }

            flash_read_page_to_buffer(0u, (uint32)(base + 1u + page),
                                      (uint32)FLASH_PAGE_LENGTH);

            crc_r = rt_crc16(crc_r, (const uint8 *)flash_union_buffer,
                             left * INS_RT_REC_SIZE);

            for (k = 0u; k < left; k ++)
            {
                rt_rec_t r;

                memcpy(&r, (const uint8 *)flash_union_buffer + k * INS_RT_REC_SIZE,
                       INS_RT_REC_SIZE);

                s_wp[done + k].x          = (float)r.x_mm / 1000.0f;
                s_wp[done + k].y          = (float)r.y_mm / 1000.0f;
                s_wp[done + k].yaw        = (float)r.yaw_cdeg / 100.0f;
                s_wp[done + k].v          = (float)r.v_cmps / 100.0f;
                s_wp[done + k].steer_cdeg = r.steer_cdeg;
                s_wp[done + k].flags      = r.flags;
            }

            done += left;
        }

        if (crc_r != h.crc_r) { return 0u; }
    }

    s_head.origin_x   = (float)h.origin_x_mm / 1000.0f;
    s_head.origin_y   = (float)h.origin_y_mm / 1000.0f;
    s_head.origin_yaw = (float)h.origin_yaw_cdeg / 100.0f;
    s_head.count      = (uint16)n;
    s_head.version    = 1u;
    s_head.total_dist = (float)h.lap_mm / 1000.0f;

    s_running = 0u;
    s_flags   = 0u;
    if (0u != (h.flags & 1u)) { s_flags |= INS_ROUTE_ST_FULL; }

    {
        uint32 i;

        s_steer_cnt = 0u;
        s_key_cnt   = 0u;
        for (i = 0u; i < n; i ++)
        {
            if (0u != (s_wp[i].flags & (uint16)INS_WP_FL_STEER)) { s_steer_cnt ++; }
            if (0u != (s_wp[i].flags & (uint16)INS_WP_FL_KEY))   { s_key_cnt   ++; }
        }
    }

    s_fl_last_slot = slot;
    s_fl_loaded    = 1u;
    return 1u;
}

uint8 ins_route_flash_load (void)
{
    return ins_route_flash_load_slot(s_active_slot);
}

//--------------------------------------
// 函数简介     从索引读回 active 槽
//--------------------------------------
void ins_route_slot_init (void)
{
    rt_idx_t d;

    flash_init();
    rt_idx_load(&d);
    s_active_slot = d.active_slot;
}

uint8 ins_route_slot_active (void) { return s_active_slot; }

//--------------------------------------
// 函数简介     选槽；persist = 1 时写索引页
// 返回参数     uint8	0 = 成功
//--------------------------------------
uint8 ins_route_slot_set_active (uint8 slot, uint8 persist)
{
    if (slot >= (uint8)INS_RT_SLOT_COUNT) { return 1u; }

    /* 互斥闸门（= 队友 v15 ins_route.c:954）：回放/录制/写盘期间换槽会让
       "当前槽"与 RAM 里的表不一致 */
    if (route_pb_active() || s_running || ins_route_flash_busy()) { return 1u; }

    s_active_slot = slot;

    if (0u != persist)
    {
        rt_idx_t d;

        flash_init();
        rt_idx_load(&d);
        d.active_slot = slot;
        rt_idx_store(&d);
    }
    return 0u;
}

//--------------------------------------
// 函数简介     读槽摘要；索引没有就现读该槽头页
// 返回参数     uint8	1 = 该槽有可用表
//--------------------------------------
uint8 ins_route_slot_info (uint8 slot, ins_route_slot_info_t *out)
{
    rt_idx_t  d;
    rt_head_t h;

    if ((NULL == out) || (slot >= (uint8)INS_RT_SLOT_COUNT)) { return 0u; }

    memset(out, 0, sizeof(*out));
    out->valid = 0u;
    out->slot  = slot;

    flash_init();

    rt_idx_load(&d);
    if (0u != d.s[slot].valid)
    {
        out->valid  = 1u;
        out->tag    = d.s[slot].tag;
        out->count  = d.s[slot].count;
        out->lap_mm = d.s[slot].lap_mm;
        out->ds_mm  = d.s[slot].ds_mm;
        out->crc_r  = d.s[slot].crc_r;
        return 1u;
    }

    flash_read_page_to_buffer(0u, INS_RT_SLOT_BASE(slot), (uint32)FLASH_PAGE_LENGTH);
    memcpy(&h, (const void *)flash_union_buffer, sizeof(h));

    if ((RT_MAGIC != h.magic) || ((uint16)RT_VER != h.ver) ||
        ((uint16)INS_RT_REC_SIZE != h.rec_len) || (0u == h.count) ||
        (h.count > INS_RT_FLASH_CAPACITY) ||
        (rt_crc16(0xFFFFu, (const uint8 *)&h, 34u) != h.crc_h))
    {
        return 0u;
    }

    out->valid  = 1u;
    out->count  = h.count;
    out->lap_mm = h.lap_mm;
    out->ds_mm  = h.ds_mm;
    out->crc_r  = h.crc_r;
    return 1u;
}

/*===================================================================================================================
 *  镜像级接口（灌表 / 上传用）
 *
 *  与 ins_route_flash_load*() 读的是**同一套字节格式**（36 B 头 + N×16 B 记录，两个 CRC16），
 *  区别只是来源：那两条从 Flash 分页读，这三条从一块连续内存读 / 按偏移往外拷。
 *=================================================================================================================*/

//--------------------------------------
// 函数简介     把一整块"车端镜像"校验并展开进 RAM
// 参数说明     img	36 B 头 + N×16 B 记录，小端
// 参数说明     len	字节数
// 返回参数     uint8	1 = 通过；0 = 不过（期间不修改 RAM）
// 使用示例     ins_route_load_image(buf, n);
// 备注信息     ★ 准入条件与 ins_route_flash_load_slot() 逐条相同 —— 两个入口必须对
//              "什么算一张合法的表"给同一个答案，否则会出现"灌进去说成功、载入却说无效"。
//              ★ 按手工偏移读，不 memcpy 到 rt_head_t：不依赖 packed。
//              ★ 只写 s_wp[] / s_head，不碰 Flash、不碰 s_fl_busy；
//                写盘由 ins_route_slot_load_image() → ins_route_flash_save_slot() 接手。
//--------------------------------------
uint8 ins_route_load_image (const uint8 *img, uint32 len)
{
    uint32 magic, count, ds_mm, lap_mm, need, i;
    uint16 ver, rec_len, crc_r, crc_h, flags;
    int32  ox, oy;
    int16  oyaw;

    if ((NULL == img) || (len < 36u)) { return 0u; }

    /* 互斥闸门（= 队友 v15 ins_route.c:860）：回放对点序是原地重排的，
       此时灌进来一张表，回放会读到别人写了一半的 s_wp[] */
    if (route_pb_active() || s_running || ins_route_flash_busy()) { return 0u; }

    magic   = (uint32)img[0]  | ((uint32)img[1]  << 8) | ((uint32)img[2]  << 16) | ((uint32)img[3]  << 24);
    ver     = (uint16)((uint16)img[4]  | ((uint16)img[5]  << 8));
    rec_len = (uint16)((uint16)img[6]  | ((uint16)img[7]  << 8));
    count   = (uint32)img[8]  | ((uint32)img[9]  << 8) | ((uint32)img[10] << 16) | ((uint32)img[11] << 24);
    ds_mm   = (uint32)img[12] | ((uint32)img[13] << 8) | ((uint32)img[14] << 16) | ((uint32)img[15] << 24);
    lap_mm  = (uint32)img[16] | ((uint32)img[17] << 8) | ((uint32)img[18] << 16) | ((uint32)img[19] << 24);
    ox      = (int32)((uint32)img[20] | ((uint32)img[21] << 8) | ((uint32)img[22] << 16) | ((uint32)img[23] << 24));
    oy      = (int32)((uint32)img[24] | ((uint32)img[25] << 8) | ((uint32)img[26] << 16) | ((uint32)img[27] << 24));
    oyaw    = (int16)((uint16)img[28] | ((uint16)img[29] << 8));
    flags   = (uint16)((uint16)img[30] | ((uint16)img[31] << 8));
    crc_r   = (uint16)((uint16)img[32] | ((uint16)img[33] << 8));
    crc_h   = (uint16)((uint16)img[34] | ((uint16)img[35] << 8));

    /* ---- 与 flash_load 完全一致的准入条件 ---- */
    if (RT_MAGIC != magic)                        { return 0u; }
    if ((uint16)RT_VER != ver)                    { return 0u; }
    if ((uint16)INS_RT_REC_SIZE != rec_len)       { return 0u; }
    if (0u == count)                              { return 0u; }
    if (count > (uint32)INS_ROUTE_MAX_POINTS)     { return 0u; }
    if (0u == ds_mm)                              { return 0u; }
    if (rt_crc16(0xFFFFu, img, 34u) != crc_h)     { return 0u; }

    need = 36u + count * (uint32)INS_RT_REC_SIZE;
    if (len < need)                               { return 0u; }
    if (rt_crc16(0xFFFFu, img + 36u, count * (uint32)INS_RT_REC_SIZE) != crc_r) { return 0u; }

    /* ---- 展开：mm → m、0.01° → °、cm/s → m/s ---- */
    for (i = 0u; i < count; i ++)
    {
        const uint8 *p = img + 36u + i * (uint32)INS_RT_REC_SIZE;

        int32  x_mm     = (int32)((uint32)p[0] | ((uint32)p[1] << 8) | ((uint32)p[2] << 16) | ((uint32)p[3] << 24));
        int32  y_mm     = (int32)((uint32)p[4] | ((uint32)p[5] << 8) | ((uint32)p[6] << 16) | ((uint32)p[7] << 24));
        int16  yaw_cdeg = (int16)((uint16)p[8]  | ((uint16)p[9]  << 8));
        int16  v_cmps   = (int16)((uint16)p[10] | ((uint16)p[11] << 8));
        int16  st_cdeg  = (int16)((uint16)p[12] | ((uint16)p[13] << 8));
        uint16 fl       = (uint16)((uint16)p[14] | ((uint16)p[15] << 8));

        s_wp[i].x          = (float)x_mm / 1000.0f;
        s_wp[i].y          = (float)y_mm / 1000.0f;
        s_wp[i].yaw        = (float)yaw_cdeg / 100.0f;
        s_wp[i].v          = (float)v_cmps / 100.0f;
        s_wp[i].steer_cdeg = st_cdeg;
        s_wp[i].flags      = fl;
    }

    /* 统计"这份表带不带打角/关键点"（菜单与遥测要看）*/
    s_steer_cnt = 0u;
    s_key_cnt   = 0u;
    for (i = 0u; i < count; i ++)
    {
        if (0u != (s_wp[i].flags & (uint16)INS_WP_FL_STEER)) { s_steer_cnt ++; }
        if (0u != (s_wp[i].flags & (uint16)INS_WP_FL_KEY))   { s_key_cnt   ++; }
    }

    s_head.origin_x   = (float)ox / 1000.0f;
    s_head.origin_y   = (float)oy / 1000.0f;
    s_head.origin_yaw = (float)oyaw / 100.0f;
    s_head.count      = (uint16)count;
    s_head.version    = 1u;
    s_head.total_dist = (float)lap_mm / 1000.0f;

    s_running = 0u;
    s_flags   = 0u;
    if (0u != (flags & 1u)) { s_flags |= INS_ROUTE_ST_FULL; }

    s_fl_loaded = 0u;               /* ★ 不是从 Flash 来的：上位机可以提示"这是刚下发的表" */

    return 1u;
}

//--------------------------------------
// 函数简介     从内存镜像写进指定槽：先校验+展开进 RAM，再武装分步写盘
// 参数说明     slot	槽号（灌表只允许 1..INS_RT_SLOT_COUNT-1，槽 0 是手推录制的家）
// 参数说明     img / len	镜像
// 返回参数     uint8	0 = 已武装（等 poll 写完）/ 1 = 槽越界 / 2 = 镜像不过 / 3 = 无法武装
// 使用示例     ins_route_slot_load_image(1, buf, n);
// 备注信息     ★ **不在这里等写盘完成**：擦一页是毫秒级，一口气写 17 页会把 5 ms 控制节拍顶穿。
//              真正的写由主循环的 ins_route_flash_poll() 一页一页推进。
//              ★ 写完 ins_route_flash_save_slot() 会顺手把该槽设为 active ——
//                所以灌完不用再选槽，菜单里直接就是这张新表。
//--------------------------------------
uint8 ins_route_slot_load_image (uint8 slot, const uint8 *img, uint32 len)
{
    if (slot >= (uint8)INS_RT_SLOT_COUNT)     { return 1u; }
    if (0u == ins_route_load_image(img, len)) { return 2u; }

    ins_route_flash_save_slot(slot);
    if (0u == s_fl_busy)                      { return 3u; }
    return 0u;
}

//--------------------------------------
// 函数简介     按"镜像偏移"读一块（车 → PC 上传用）
// 参数说明     slot	槽号 / off	镜像内偏移 / dst	目的地 / n	最多读多少字节
// 返回参数     uint16	实际拷贝字节数；0 = 失败（槽空 / 格式不对 / 参数错）
// 使用示例     ins_route_slot_read_bytes(1, off, buf, 512);
// 备注信息     ★ 镜像是**逻辑连续**的（36 B 头 + N×16 B 记录），物理上却是 2 KB 分页，
//              所以偏移要先换算成"页 + 页内偏移"。
//              ★ 一次调用里只在**页号变化时重读那一页**（flash_read_page_to_buffer 是 2 KB
//                阻塞读，512 B 的块最多碰 2 页）。
//              ⚠ 不做跨调用的页缓存：flash_union_buffer 是全局共享的，别的模块
//                （Param_Storage / 分步写盘）随时会把它盖掉，跨调用缓存就是自找脏读。
//--------------------------------------
uint16 ins_route_slot_read_bytes (uint8 slot, uint32 off, uint8 *dst, uint16 n)
{
    rt_head_t h;
    uint32    base, img_len, pos;
    uint32    last_page = 0xFFFFFFFFu;
    uint16    done = 0u, copy;

    if ((NULL == dst) || (slot >= (uint8)INS_RT_SLOT_COUNT) || (0u == n)) { return 0u; }

    flash_init();
    base = INS_RT_SLOT_BASE(slot);

    flash_read_page_to_buffer(0u, base, (uint32)FLASH_PAGE_LENGTH);
    last_page = base;
    memcpy(&h, (const void *)flash_union_buffer, sizeof(h));

    if ((RT_MAGIC != h.magic) || ((uint16)RT_VER != h.ver) ||
        ((uint16)INS_RT_REC_SIZE != h.rec_len) || (0u == h.count) ||
        (h.count > INS_RT_FLASH_CAPACITY))
    {
        return 0u;
    }

    img_len = 36u + (uint32)h.count * (uint32)INS_RT_REC_SIZE;
    if (off >= img_len)              { return 0u; }
    if ((off + (uint32)n) > img_len) { n = (uint16)(img_len - off); }

    pos = off;

    /* --- 头 36 B（就在刚才那一页里）--- */
    if (pos < 36u)
    {
        uint32 take = 36u - pos;

        if (take > (uint32)n) { take = (uint32)n; }
        memcpy(dst, (const uint8 *)flash_union_buffer + pos, take);
        done = (uint16)take;
        pos += take;
    }

    /* --- 记录区 --- */
    while (done < n)
    {
        uint32 rec      = (pos - 36u) / (uint32)INS_RT_REC_SIZE;
        uint32 in_rec   = (pos - 36u) % (uint32)INS_RT_REC_SIZE;
        uint32 page     = rec / (uint32)INS_RT_REC_PER_PAGE;
        uint32 in_page  = (rec % (uint32)INS_RT_REC_PER_PAGE) * (uint32)INS_RT_REC_SIZE + in_rec;

        /* 本页还剩多少字节，必须**从 pos 算**（含记录内偏移 in_rec），不能从"本条记录的开头"算：
           漏掉 in_rec 的话，每个页边界都会多拷几个字节（读到页内 0xFF 填充）、
           再跳掉下一页开头几个字节。症状是镜像长度对、每帧 CRC 也过，只有记录区 CRC 不对。 */
        uint32 till_end = (uint32)INS_RT_REC_PER_PAGE * (uint32)INS_RT_REC_SIZE - in_page;

        if (last_page != (base + 1u + page))
        {
            flash_read_page_to_buffer(0u, base + 1u + page, (uint32)FLASH_PAGE_LENGTH);
            last_page = base + 1u + page;
        }

        copy = (uint16)((uint32)n - (uint32)done);
        if ((uint32)copy > till_end) { copy = (uint16)till_end; }

        memcpy(dst + done, (const uint8 *)flash_union_buffer + in_page, copy);
        done += copy;
        pos  += copy;
    }

    return done;
}

/**********************************************************/
/* ---- 倒车录 / 倒序回放支持（2026-10-24）---------------------------------------------
 *
 * 来历：队友 14.pp_for_backroute 的 ins_route_reverse_points() / ins_route_rev_count()。
 *
 * 这张表怎么来的：车**倒着**沿一条路开一遍录表（REV 位照打、速度存 |v|）。
 * 它怎么用：回放时把点序**倒过来**，车**正着**开回起点。
 * 为什么成立：同一点的车身朝向、以及通过该点所需的前轮转角完全相同，
 *             只有"遍历顺序"相反 —— 所以不需要重算几何，只要把点序翻过来。
 */
/**********************************************************/

//--------------------------------------
// 函数简介     数本表打了 REV 位的点数（"表出身"判据）
// 返回参数     REV 点个数
// 使用示例     isrev = (ins_route_rev_count() * 2u >= n)
// 备注信息     REV 位是**录制时 v < -0.02 m/s** 打的（ins_route.c 的录制分支）
//--------------------------------------
uint16 ins_route_rev_count (void)
{
    uint16 i;
    uint16 c = 0u;

    for (i = 0u; i < s_head.count; i ++)
    {
        if (0u != (s_wp[i].flags & (uint16)INS_WP_FL_REV)) { c ++; }
    }

    return c;
}

//--------------------------------------
// 函数简介     【已废弃】原"把点序原地倒过来"——现在恒返回 0，不动点表
// 返回参数     恒 0
// 备注信息     ★ 为什么废弃：倒序回放改由回放侧在**私有副本**上做（route_pb.c 的 pb_build_play），
//                录制表 s_wp[] 全程不改。原地翻转需要"退出时再倒一次复原"，
//                漏一次表就停在倒序（下次起跑双重翻转 / 上位机读回反表），已删。
//              ★ 保留这个符号只是为了让旧调用点继续编译（= 队友 v15 ins_route.c:293-296
//                同样留了个恒返回 0 的旧 ABI 桩）。新代码不要用它。
//--------------------------------------
uint8 ins_route_reverse_points (void)
{
    return 0u;
}