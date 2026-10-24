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
#define PB_V_MIN            (-2.00f)    /* 负值 = 倒车（后轮反转，同 |v| 同速） */
#define PB_V_MAX            (2.00f)

#define PB_LD_DEFAULT       (0.80f)     /* 前视距离 m */
#define PB_LD_MIN           (0.20f)
#define PB_LD_MAX           (2.00f)

/* 有效轴距 m：满舵转圈实测值。
   ★ 2026-10-23 起从"编译期常量"提成运行期参数（Route 参数页的 L），与回放参数一起持久化：
     换车 / 改转向结构 / 重新量轴距都不用重编译。这里的 L_M 只是默认值。 */
#define PB_L_DEFAULT        (0.66f)
#define PB_L_MIN            (0.40f)
#define PB_L_MAX            (1.20f)

/* 转向通道：0 = 航向外环（k 自动取 2L/ld）/ 1 = 直接给 δ */
#define PB_SMODE_DEFAULT    (0.0f)
#define PB_K_MIN            (0.30f)
#define PB_K_MAX            (8.00f)

/* ---------------------------------------------------------------------------
 * 速度剖面：曲率限速 + 末段收油（2026-10-24）
 *
 * pb_ve = 1 时，ROUTE_PB_V 从"恒定车速"变成"车速上限"，实际目标由下面两条决定：
 *   ① 曲率限速  v ≤ √(alat / κ)，κ = 窗口内航向变化 / 弧长（弧长用逐段弦长累加）
 *   ② 末段收油  v ≤ √(creep² + 2·alat·d)，d = 沿路径到末点的弧长
 * pb_ve = 0 时**完全走老路径**（一个常数速度），所以它默认是 0。
 *
 * ⚠ ② 是队友"反向传刹车"的**单点简化版**：他们从每个低速点（弯心、末点）反向传播
 *   刹车曲线，我们只从末点往前收。差别是"出弯后不会提前加速"，但末段不会冲出去。
 * ------------------------------------------------------------------------- */
#define PB_VE_DEFAULT       (0.0f)      /* 0 = 关（行为不变）/ 1 = 开 */
#define PB_ALAT_DEFAULT     (2.00f)     /* 允许横向加速度 m/s^2 */
#define PB_ALAT_MIN         (0.20f)
#define PB_ALAT_MAX         (10.0f)
#define PB_VMIN_DEFAULT     (0.30f)     /* 曲率限速的地板 m/s（低于电机死区会一走一停）*/
#define PB_VMIN_MIN         (0.00f)
#define PB_VMIN_MAX         (1.00f)

#define PB_KWIN             (6u)        /* 曲率窗口（单边点距数）：太大⇒把弯当直道，太小⇒抖 */
#define PB_END_CREEP        (0.50f)     /* 末段收油的终点速度 m/s */

/* 航向环增益的手工覆盖：> 0 就直接用它，不再按 2L/ld 现算。
   为什么留着这个口子：2L/ld 是"纯跟踪的等价增益"，**前视一变它就得跟着变**——
   一旦把它写死成常数，这个环就变成了"另一个环"，特征频率随前视漂移（现场表现是"谁开谁振"）。
   默认 0 = 自动。只在做 A/B 对比时才动它。 */
#define PB_KH_DEFAULT       (0.0f)
#define PB_KH_MIN           (0.0f)
#define PB_KH_MAX           (8.0f)

/* 判据 */
#define PB_MIN_POINTS       (10u)       /* 少于这么多点不许启动 */
#define PB_NEAR_WIN         (40u)       /* 最近点前向搜索窗（点） */
#define PB_FIN_M            (0.15f)     /* 离末点多近算跑完 m（= 队友 pb_fin）*/
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
