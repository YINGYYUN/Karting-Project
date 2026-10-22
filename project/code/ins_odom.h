/*********************************************************************************************************************
 * 航位推算（IMU 航向 × 编码器里程 → 米制位姿）
 *
 * ins_ = INS（Inertial Navigation System，惯导）子系统前缀，本文件属"惯导 + 路表"系列
 *
 *   yaw ← ins_imu_get_yaw()                 （累积航向，可超 ±360）
 *   ds  ← 左右后轮单拍原始增量各换算成米，取平均（带符号，倒车为负）
 *   x   += -sin(yaw) · ds                   （x = 右）
 *   y   +=  cos(yaw) · ds                   （y = 前，即录制起点朝向）
 *
 * 数据源 = ENC_LR_DLT / ENC_RR_DLT（未滤波的单拍整数增量）。不要换成 ENC_*_CNT_F：
 * 那是滑动平均后的值，滞后 (窗口-1)/2 拍，里程跟不上航向会让轨迹晚转。
 *
 * 坐标约定：yaw = 0 车头朝 +Y；yaw 增大（左转）转向 -X。上位机渲染 screen_x ∝ x、screen_y ∝ -y，
 *           改这里必须同步改上位机。
 ********************************************************************************************************************/

#ifndef _INS_ODOM_H
#define _INS_ODOM_H

#include "zf_common_headfile.h"

/* 每转多少米 = 后轮周长。实测 2.815 m / 6383.5 cnt（cpr 1858）⇒ 0.8193 */
#define INS_ODOM_MPM_DEFAULT    (0.8193f)

/* 采样周期，与 CH11 一致 */
#define INS_ODOM_T_S            (0.005f)

/* 连续多少拍两轮都没计数算静止（10 拍 = 50 ms） */
#define INS_ODOM_STILL_TICKS    (10)

/* 比例尺：只影响"米"这个单位，尺度错了轨迹形状与航向仍对 */
extern volatile float  ins_m_per_rev;

/* 0 = 冻结积分（标定/推车时用），只跟踪基准 */
extern volatile float  ins_odom_enable;

/* 打滑判据：左右计数差超过该百分比且航向几乎没变 ⇒ 置标志（只报警不改数据） */
extern volatile float  ins_slip_pct;

/* 状态位 */
#define ODOM_ST_ACTIVE      (0x01u)     /* 正在积分                 */
#define ODOM_ST_STILL       (0x02u)     /* 连续 N 拍两轮都没动       */
#define ODOM_ST_SLIP        (0x04u)     /* 本拍判定打滑             */

/* 一致位姿快照：x/y/yaw/dist 必须一次取完，分三次 get 会被 5ms 中断插到半路 */
typedef struct
{
    float x;            /* m，向右                     */
    float y;            /* m，向前                     */
    float yaw;          /* deg，相对起点，累积可超 ±360 */
    float dist_abs;     /* 累计路程（标量）m            */
    float v_mps;        /* 本拍车速 m/s（带符号）        */
    uint8 flags;        /* ODOM_ST_*                   */
} ins_odom_snap_t;

void    ins_odom_init       (void);
void    ins_odom_update     (void);         /* CH11 5ms，必须在编码器与 IMU 更新之后 */
void    ins_odom_reset      (void);         /* 位置/里程清零；要连航向一起归零请另调 ins_imu_zero_yaw() */
void    ins_odom_set_origin (float x, float y, float yaw_deg);

float   ins_odom_get_x      (void);
float   ins_odom_get_y      (void);
float   ins_odom_get_yaw    (void);
float   ins_odom_get_dist   (void);         /* 累计路程（标量）m */
float   ins_odom_get_v_mps  (void);         /* 本拍车速 m/s（带符号） */

uint8   ins_odom_flags      (void);
void    ins_odom_snapshot   (ins_odom_snap_t *out);

#endif
