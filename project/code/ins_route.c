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
    /* ★ 倒车录表（rt_rrec = 1）：v 存成 |v|。
       这张表此后只用于"正车倒序回放"，新方向上车是往前开的 ⇒ 速度必须全正。
       下面 REV 位仍然用**带符号**的 v_mps 判 —— 那是"表的出身"标记，不能丢。 */
    s_wp[idx].v   =  (ROUTE_REV_REC >= 0.5f) ? fabsf(v_mps) : v_mps;

    /* 打角列：记当拍实际施加的舵角（不是实测角，实测角带随动滞后） */
    d = Servo_Target_Applied_Deg;
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

    if (v_mps < -0.02f) { fl |= (uint16)INS_WP_FL_REV; }

    s_wp[idx].flags = fl;
}

//--------------------------------------
// 函数简介     开始录制（重新捕获起点）
//--------------------------------------
void ins_route_start (void)
{
    ins_odom_snap_t o;

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

    route_to_local(&o, 0, 0.0f);
    s_head.count = 1;

    ins_imu_zupt_frz_reset();

    s_running = 1;
}

//--------------------------------------
// 函数简介     停止录制
// 备注信息     ★ 参数 ROUTE_END_PT = 1（默认）时，把"当前这一拍"强制补成路表最后一点。
//
//   为什么需要：采样是按**里程**触发的（每走 ins_route_ds 才落一点），
//   所以按下停止那一刻如果距上一个点还不到一个点距，最后那一段就整段没有点。
//   而终点往往正是**打着大角**的地方（收车、过弯尾）—— 少这一点，回放跑到末尾时
//   最后一段既没有位置也没有正确的打角，会直接冲出赛道。
//
//   什么时候不补：位置没动且打角没变（说明上一点已经就是终点状态）。
//   补点的阈值取得很小（2 mm / 1°），只用来挡"完全重复的点"——
//   零长度线段会让回放的最近点搜索选中重复点、前视向量退化成 0。
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
        int    dsteer;

        ins_odom_snapshot(&o);
        route_to_local(&o, idx, o.v_mps);        /* 展开候选点（会顺带累加 s_steer_cnt） */

        dx     = s_wp[idx].x - s_wp[idx - 1u].x;
        dy     = s_wp[idx].y - s_wp[idx - 1u].y;
        dsteer = (int)s_wp[idx].steer_cdeg - (int)s_wp[idx - 1u].steer_cdeg;
        if (dsteer < 0) { dsteer = -dsteer; }

        if (((dx * dx + dy * dy) > (0.002f * 0.002f)) || (dsteer > 100))
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

        /* 扣阈值而不是清零：清零会把这一圈多走的丢掉，点距会整体偏大 */
        s_dist_acc -= ins_route_ds;
        if (s_dist_acc < 0.0f) { s_dist_acc = 0.0f; }

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

static int16 rt_wrap_cdeg (float yaw_deg)
{
    float y = yaw_deg;

    while (y >  180.0f) { y -= 360.0f; }
    while (y < -180.0f) { y += 360.0f; }

    return (int16)(y * 100.0f);
}

static float rt_clampf (float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static void rt_rec_pack (const ins_wp_t *wp, rt_rec_t *r)
{
    r->x_mm       = (int32)(wp->x * 1000.0f);
    r->y_mm       = (int32)(wp->y * 1000.0f);
    r->yaw_cdeg   = rt_wrap_cdeg(wp->yaw);
    r->v_cmps     = (int16)rt_clampf(wp->v * 100.0f, -32000.0f, 32000.0f);
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
// 函数简介     把点序**原地**倒过来，并把负速度取正
// 返回参数     1 = 成功；0 = 录制中 / 表太短（这两种情况不动点表）
// 使用示例     ins_route_reverse_points();
// 备注信息     ★ 幂等：连调两次 = 原序 —— 退出回放时用它复原（见 route_pb.c 的 pb_finish）
//              ★ 录制中拒绝：录制器按 idx 递增写点，倒一次就全乱
//              ★ yaw 不动：它记的是"录制时车头朝向"，倒序后仍然正确
//                （新方向上前轮转角相同、朝向相同，只有遍历顺序变了）
//--------------------------------------
uint8 ins_route_reverse_points (void)
{
    uint16 i;
    uint16 j;
    ins_wp_t t;

    if (0 != s_running)     { return 0u; }       // 录制中不许倒
    if (s_head.count < 2u)  { return 0u; }

    for (i = 0u, j = (uint16)(s_head.count - 1u); i < j; i ++, j --)
    {
        t       = s_wp[i];
        s_wp[i] = s_wp[j];
        s_wp[j] = t;
    }

    /* 倒置后新方向是"正着开" ⇒ 速度必须全正。
       注意：录制时若开了 rt_rrec，v 本来就已是 |v|；没开的话是带符号的负数，
       这里统一取绝对值，两种来源都能用。 */
    for (i = 0u; i < s_head.count; i ++)
    {
        if (s_wp[i].v < 0.0f) { s_wp[i].v = -s_wp[i].v; }
    }

    return 1u;
}