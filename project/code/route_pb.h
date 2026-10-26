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

/* ---- 倒车段（2026-10-24）--------------------------------------------------------
 * 表里**本来就有**这些东西，回放以前一个都没读（存储结构不用改）：
 *   rec+10  v_cmps      int16  有符号，< 0 就是倒车（这张实测表是 −22…−35 cm/s）
 *   rec+12  steer_cdeg  int16  当拍实际施加的舵角
 *   rec+14  flags       bit2 = INS_WP_FL_REV（录制侧 v < −0.02 m/s 时打上）
 *
 * 倒车段的三条纪律（缺一条就撞/跑飞）：
 *   ① 速度 = 表里录的负值（再被｜pb_v｜和 PB_REV_VMAX 封顶）；
 *   ② 转向**关航向外环、开环回放录制打角** —— 纯跟踪的 α 与 δ=−kp·err 都按"车头朝前"
 *      推导，倒车时前后关系整体差 180°，继续闭环就是正反馈；
 *   ③ 换向点前留一点爬行速度，否则进度点会卡在那几毫米的换向段上过不去。
 * 判据用"标志位优先、速度兜底"：录制侧打位门限是 −0.02，回放侧用 −0.10 更稳，
 * 两个判据都以**同一拍**的 p[best] 为准，不需要任何新字段。 */
#define PB_REV_EPS          (0.10f)     /* 兜底判据：v < −0.10 m/s 视为倒车 */
#define PB_REV_CREEP        (0.15f)     /* 倒车点之前那一点的爬行速度 m/s */
#define PB_REV_VMAX         (0.60f)     /* 倒车速度上限 m/s（表里的负值不会超过它） */

/* ---- 表尾状态 → 收尾策略（2026-10-24）---------------------------------------------
 * 车端能读到表尾是"倒车"还是"前进"，那就该按它决定怎么收：
 *   表尾 = 倒车段 ⇒ **先刹停再收**（车到终点还在往后滑，直接放开电机会继续退、
 *                    舵机回正会让方向盘突然自己回正）；
 *   表尾 = 前进段 ⇒ 维持原来的行为（到点即收）。
 * 判据看最后 PB_TAIL_N 个点的多数，而不是只看末点 —— 录制器停止时会补一个
 * v=0 的收尾点（实测表最后一点就没有 REV 位），只看末点必然判错。 */
#define PB_TAIL_N           (3u)        /* 判表尾状态看最后几个点 */
#define PB_TAIL_V           (0.06f)     /* 刹停判据：|v| 小于它就算停住 m/s */
#define PB_TAIL_MS          (1500u)     /* 刹停最长等待 ms，超时也放行（别卡死在终点） */

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
