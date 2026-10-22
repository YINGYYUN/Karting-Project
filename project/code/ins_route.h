/*********************************************************************************************************************
 * 路表录制（米制路线图）与 Flash 多槽存取
 *
 * ins_ = INS（Inertial Navigation System，惯导）子系统前缀，本文件属"惯导 + 路表"系列
 *
 * 只存相对量：x/y/yaw 都是"相对录制起点车体系"（x 向右、y 向前），yaw 是相对增量。
 * 采样只按里程（点距 ins_route_ds），不按转角。
 ********************************************************************************************************************/

#ifndef _INS_ROUTE_H
#define _INS_ROUTE_H

#include "zf_common_headfile.h"
#include "flash_map.h"

/*===================================================================================================================
 *  一、配置
 *=================================================================================================================*/
#define INS_ROUTE_MAX_POINTS    (INS_RT_SLOT_CAPACITY)      /* = 2048 */
#define INS_ROUTE_DS_DEFAULT    (0.05f)                     /* 米，单槽 102 m */
#define INS_ROUTE_DS_MIN        (0.02f)
#define INS_ROUTE_DS_MAX        (0.20f)

/* 运行时点距（米）；录制中改它只影响之后的点 */
extern volatile float   ins_route_ds;

/*===================================================================================================================
 *  二、数据结构
 *=================================================================================================================*/
typedef struct
{
    float   x;              /* 米，相对录制起点：x 向右 */
    float   y;              /* 米，               y 向前 */
    float   yaw;            /* 度，相对录制起点 */
    float   v;              /* m/s，录制瞬间车速 */
    int16   steer_cdeg;     /* 当拍实际施加的舵角，0.01° */
    uint16  flags;          /* INS_WP_FL_* */
} ins_wp_t;

/* 逐点标志；bit0 = 打角有效（打角为 0 也可能是有效值，不能用 0 判断） */
#define INS_WP_FL_STEER         (0x0001u)
#define INS_WP_FL_KEY           (0x0002u)
#define INS_WP_FL_REV           (0x0004u)
#define INS_WP_FL_TRAJ          (0x0008u)

typedef struct
{
    float   origin_x;       /* 录制起点的 odom 坐标，米 */
    float   origin_y;
    float   origin_yaw;     /* 录制起点的航向，度 */
    uint16  count;          /* 已录点数 */
    uint16  version;        /* 格式版本 */
    float   total_dist;     /* 录制期间总里程，米 */
} ins_route_head_t;

/* 录制状态位 */
#define INS_ROUTE_ST_RUNNING    (0x01u)
#define INS_ROUTE_ST_FULL       (0x02u)     /* 录满（点距太大 / 赛道太长） */

/*===================================================================================================================
 *  三、录制接口
 *=================================================================================================================*/
void    ins_route_init      (void);
void    ins_route_start     (void);         /* 重新捕获起点并清空缓冲 */
void    ins_route_stop      (void);
void    ins_route_poll      (void);         /* 主循环每圈一次 */
void    ins_route_clear     (void);         /* 只清缓冲 */

uint8   ins_route_is_running(void);
uint8   ins_route_flags     (void);
uint16  ins_route_get_count (void);
float   ins_route_get_total_dist (void);

uint8   ins_route_mark_key  (void);         /* 标记最后一个已录点；1 = 标上了 */
uint16  ins_route_key_count (void);
uint16  ins_route_steer_count (void);

const ins_wp_t          *ins_route_get_points (void);
const ins_route_head_t  *ins_route_get_head   (void);

/*===================================================================================================================
 *  四、Flash 多槽（页号见 flash_map.h）
 *
 *  每槽镜像 = 36 B 表头 + N × 16 B 记录，两处 CRC16。
 *  写盘是分步的：save_slot 只武装，flash_poll 每圈写一页，避免阻塞顶穿控制节拍。
 *=================================================================================================================*/
#define INS_RT_SLOT_COUNT       (FM_SLOT_COUNT)
#define INS_RT_SLOT_PAGES       (FM_SLOT_PAGES)
#define INS_RT_FLASH_REC_PAGES  (FM_SLOT_REC_PAGES)
#define INS_RT_REC_SIZE         (FM_REC_SIZE)
#define INS_RT_REC_PER_PAGE     (FM_REC_PER_PAGE)
#define INS_RT_SLOT_CAPACITY    (FM_SLOT_CAPACITY)
#define INS_RT_FLASH_CAPACITY   (INS_RT_SLOT_CAPACITY)
#define INS_RT_INDEX_PAGE       (FM_SLOT_INDEX_PAGE)
#define INS_RT_SLOT_BASE(k)     (FM_SLOT_BASE_PAGE(k))

uint8   ins_route_flash_load        (void);     /* 载入当前 active 槽；1 = 成功 */
uint8   ins_route_flash_load_slot   (uint8 slot);

void    ins_route_slot_init         (void);     /* 从索引读回 active 槽 */
uint8   ins_route_slot_active       (void);
uint8   ins_route_slot_set_active   (uint8 slot, uint8 persist);
void    ins_route_flash_save_slot    (uint8 slot);  /* 武装分步写盘 */

void    ins_route_flash_poll        (void);     /* 主循环每圈一次 */
uint8   ins_route_flash_busy        (void);
uint8   ins_route_flash_last_rc     (void);     /* 0 未做 / 1 成功 / 2 写失败 / 3 表空 / 4 槽号错 */
uint8   ins_route_flash_progress    (void);     /* 0~100 */
uint8   ins_route_flash_last_slot   (void);
uint8   ins_route_flash_loaded      (void);     /* 1 = 当前 RAM 里的表来自 Flash */

typedef struct
{
    uint8   valid;
    uint8   slot;
    uint16  tag;
    uint32  count;
    uint32  lap_mm;
    uint32  ds_mm;
    uint32  crc_r;
} ins_route_slot_info_t;
uint8   ins_route_slot_info         (uint8 slot, ins_route_slot_info_t *out);

#endif
