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

// 参数总数（2 个后轮电机 × 4 个参数 + 舵机 × 2 个参数）
#define PARAM_COUNT                 (10)

// 一次性迁移开关：默认值改动后，烧录一次时置 1，把 Flash 里的旧参数擦掉，
// 让新默认值真正生效；确认参数已按预期加载后改回 0 再烧一次（否则每次上电都恢复默认）
#define PARAM_FORCE_RESET           (0)

// 参数缓冲区(Flash 读写的唯一载体)
extern float param_cache[PARAM_COUNT];

// 参数索引定义
// 左后轮速度闭环 (0-3)
#define MOTOR_LR_INV_K_IDX          0
#define MOTOR_LR_U0_IDX             1
#define MOTOR_LR_KV_IDX             2
#define MOTOR_LR_KZ_IDX             3

// 右后轮速度闭环 (4-7)
#define MOTOR_RR_INV_K_IDX          4
#define MOTOR_RR_U0_IDX             5
#define MOTOR_RR_KV_IDX             6
#define MOTOR_RR_KZ_IDX             7

// 舵机 转向位置环 (8-9)
#define SERVO_KP_IDX                8
#define SERVO_U0_IDX                9

// 便捷访问宏
#define MOTOR_LR_INV_K              param_cache[MOTOR_LR_INV_K_IDX]
#define MOTOR_LR_U0                 param_cache[MOTOR_LR_U0_IDX]
#define MOTOR_LR_KV                 param_cache[MOTOR_LR_KV_IDX]
#define MOTOR_LR_KZ                 param_cache[MOTOR_LR_KZ_IDX]

#define MOTOR_RR_INV_K              param_cache[MOTOR_RR_INV_K_IDX]
#define MOTOR_RR_U0                 param_cache[MOTOR_RR_U0_IDX]
#define MOTOR_RR_KV                 param_cache[MOTOR_RR_KV_IDX]
#define MOTOR_RR_KZ                 param_cache[MOTOR_RR_KZ_IDX]

#define SERVO_KP                    param_cache[SERVO_KP_IDX]
#define SERVO_U0                    param_cache[SERVO_U0_IDX]


void    Param_Init          (void);     // 初始化(加载或设默认值)
void    Param_Save          (void);     // 保存当前参数到 Flash
void    Param_Erase         (void);     // 擦除 Flash(下次启动恢复默认)
void    Flash_SyncTo_Param  (void);     // 将缓存区的值推送到实际参数


#endif
