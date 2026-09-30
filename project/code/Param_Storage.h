/*******************************************************************************
参数 Flash 存储
将可调参数持久化到 Flash，上电自动加载并同步到实际应用参数。
CYT4BB 适配：存储位置改为 Work Flash 第 95 页（sector 参数恒为 0）。
宏定义映射 — 每个参数单独编号，不强制 Kp/Ki/Kd 整齐排列。
*******************************************************************************/


#ifndef __PARAM_STORAGE_H__
#define __PARAM_STORAGE_H__


#include "zf_common_headfile.h"

// Flash 存储位置（CYT4BB Work Flash：sector 恒为 0，页范围 0-95）
#define PARAM_FLASH_SECTION         (0)
#define PARAM_FLASH_PAGE            (95)

// 参数总数（4 个电机 × 3 个参数）
#define PARAM_COUNT                 (12)

// 参数缓冲区(Flash 读写的唯一载体)
extern float param_cache[PARAM_COUNT];

// 参数索引定义
// Motor_1_PID (0-2)  电机接口 1 速度环
#define MOTOR_1_KP_IDX             0
#define MOTOR_1_KI_IDX             1
#define MOTOR_1_KD_IDX             2

// Motor_2_PID (3-5)  电机接口 2 速度环
#define MOTOR_2_KP_IDX             3
#define MOTOR_2_KI_IDX             4
#define MOTOR_2_KD_IDX             5

// Motor_3_PID (6-8)  电机接口 3 速度环
#define MOTOR_3_KP_IDX             6
#define MOTOR_3_KI_IDX             7
#define MOTOR_3_KD_IDX             8

// Motor_4_PID (9-11) 电机接口 4 速度环
#define MOTOR_4_KP_IDX             9
#define MOTOR_4_KI_IDX             10
#define MOTOR_4_KD_IDX             11

// 便捷访问宏
#define MOTOR_1_KP                 param_cache[MOTOR_1_KP_IDX]
#define MOTOR_1_KI                 param_cache[MOTOR_1_KI_IDX]
#define MOTOR_1_KD                 param_cache[MOTOR_1_KD_IDX]

#define MOTOR_2_KP                 param_cache[MOTOR_2_KP_IDX]
#define MOTOR_2_KI                 param_cache[MOTOR_2_KI_IDX]
#define MOTOR_2_KD                 param_cache[MOTOR_2_KD_IDX]

#define MOTOR_3_KP                 param_cache[MOTOR_3_KP_IDX]
#define MOTOR_3_KI                 param_cache[MOTOR_3_KI_IDX]
#define MOTOR_3_KD                 param_cache[MOTOR_3_KD_IDX]

#define MOTOR_4_KP                 param_cache[MOTOR_4_KP_IDX]
#define MOTOR_4_KI                 param_cache[MOTOR_4_KI_IDX]
#define MOTOR_4_KD                 param_cache[MOTOR_4_KD_IDX]


void    Param_Init          (void);     // 初始化(加载或设默认值)
void    Param_Save          (void);     // 保存当前参数到 Flash
void    Param_Erase         (void);     // 擦除 Flash(下次启动恢复默认)
void    Flash_SyncTo_Param  (void);     // 将缓存区的值推送到实际参数


#endif
