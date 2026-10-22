#ifndef _HEAD_CRTL_H
#define _HEAD_CRTL_H

#include "zf_common_headfile.h"

/*********************************************************************************************************************
 * 航向外环：目标航向 -> 舵角
 *
 *   δ = -sgn(v) · K · wrap180(target_yaw - yaw)
 *
 *   前进：δ = -K·err；倒车：δ = +K·rev·err（倒车时 yaw 对 δ 的响应反号）。
 *
 *   本层只管"给定目标航向，把车拉过去"，不关心目标从哪来（遥控点动 / 路表 / 纯跟踪）。
 *   目标航向用累积值（可超 ±360°），误差内部折到 ±180°。
 *   使能时本层独占 Servo_Target_Deg，上层不要再写它。
 ********************************************************************************************************************/

#define HEAD_KP_DEFAULT             (2.20f)     /* 比例增益 deg(舵角)/deg(航向误差) */
#define HEAD_KP_REV_SCALE           (0.60f)     /* 倒车增益折扣（倒车是不稳定方向） */

// 实际航向不经过本层，全项目统一读 ins_imu_get_yaw()

// 每拍调用：刷新误差；使能时把 δ 写进 Servo_Target_Deg
void    Head_Crtl_Tick         (void);
// 1 = 接管 Servo_Target_Deg。0->1 那一拍会把目标对齐当前航向（防甩头），上层随后可再 Set_Target 覆盖
void    Head_Crtl_Set_Enable   (uint8 en);
// 1 = 按倒车算（增益取反并打折）
void    Head_Crtl_Set_Reverse  (uint8 rev);
// 上层写目标航向（累积值，度）
void    Head_Crtl_Set_Target   (float yaw_deg);
// 设置航向环增益（回放按 2L/ld 覆盖；超出 0.3~8 会被夹）
void    Head_Crtl_Set_Kp       (float kp);
// 目标对齐当前航向
void    Head_Crtl_Sync         (void);

uint8   Head_Crtl_Enabled      (void);
float   Head_Crtl_Get_Target   (void);
float   Head_Crtl_Get_Err      (void);

#endif
