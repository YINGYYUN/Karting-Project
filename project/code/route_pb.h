/*********************************************************************************************************************
 * 路表回放（Pure Pursuit 外环）
 *
 * 速度不参考录制值：由参数 pb_v 给定，0 = 不驱动后轮（自由轮，手推看转向）。
 * 转向直接给舵角 δ = atan(L·2sinα/ld)，回放期间航向环必须关闭。
 ********************************************************************************************************************/

#ifndef _ROUTE_PB_H
#define _ROUTE_PB_H

#include "zf_common_headfile.h"

/* 回放参数（Route 参数页，持久化在 Flash 页 0 末尾） */
#define PB_V_DEFAULT        (0.00f)     /* 巡航车速 m/s；0 = 自由轮，手推 */
#define PB_V_MIN            (0.00f)
#define PB_V_MAX            (2.00f)

#define PB_LD_DEFAULT       (0.80f)     /* 前视距离 m */
#define PB_LD_MIN           (0.20f)
#define PB_LD_MAX           (2.00f)

/* 有效轴距 m：满舵转圈实测值，物理常量，不做参数 */
#define PB_L_M              (0.66f)

/* 转向通道：0 = 航向外环（k 自动取 2L/ld）/ 1 = 直接给 δ */
#define PB_SMODE_DEFAULT    (0.0f)
#define PB_K_MIN            (0.30f)
#define PB_K_MAX            (8.00f)

/* 判据 */
#define PB_MIN_POINTS       (10u)       /* 少于这么多点不许启动 */
#define PB_NEAR_WIN         (40u)       /* 最近点前向搜索窗（点） */
#define PB_FIN_M            (0.50f)     /* 离末点多近算跑完 m */
#define PB_DELTA_MAX        (27.0f)     /* 回放自己再夹一次舵角 deg */

/* 状态 */
#define PB_ST_IDLE          (0u)
#define PB_ST_RUN           (1u)
#define PB_ST_DONE          (2u)        /* 跑到终点 */
#define PB_ST_ABORT         (3u)        /* 中止（脱线 / 人工 / 遥控接管） */

void        route_pb_init   (void);
uint8       route_pb_start  (void);     /* 1 = 已启动；0 = 条件不足，原因见 route_pb_reason() */
void        route_pb_stop   (const char *why);
void        route_pb_poll   (void);     /* 主循环每圈一次 */

uint8       route_pb_active (void);
uint8       route_pb_state  (void);
float       route_pb_elat   (void);     /* 横向误差 m，正 = 车在参考线右侧 */
float       route_pb_ld     (void);
uint16      route_pb_aim    (void);
uint16      route_pb_index  (void);
const char *route_pb_reason (void);

#endif
