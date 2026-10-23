/*******************************************************************************
参数 Flash 存储
将可调参数持久化到 Flash，上电自动加载并同步到实际应用参数。
CYT4BB 适配：存储位置为 Work Flash 第 0 页（sector 参数恒为 0）。
宏定义映射 — 每个参数单独编号，不强制 Kp/Ki/Kd 整齐排列。
*******************************************************************************/


#ifndef __PARAM_STORAGE_H__
#define __PARAM_STORAGE_H__


#include "zf_common_headfile.h"

// Flash 存储位置（CYT4BB Work Flash：sector 恒为 0，页范围 0-95）
// 页分配约定：第 0 页存放参数；第 1-95 页预留给路表等其他数据
#define PARAM_FLASH_SECTION         (0)
#define PARAM_FLASH_PAGE            (0)

// 参数总数（2 个后轮电机 × 4 + 舵机 × 3 + 路表 × 4 + 遥测档案 × 1）
#define PARAM_COUNT                 (16)

// 上电强制重置falsh参数开关
// 1：上电强制重写参数页flash
#define PARAM_FORCE_RESET           (1)

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

// 舵机 转向位置环 (8-10)
// ★ ki 是后加的，**必须放在末尾**：前面 10 个索引不能变，否则 Flash 里已有的数据会整体错位
//   （旧的 inv_K 会被当成 u0 读进来）。
#define SERVO_KP_IDX                8
#define SERVO_U0_IDX                9
#define SERVO_KI_IDX                10

// 路表 (11-14)：录制点距 / 回放车速 / 前视距离 / 转向通道
#define ROUTE_DS_IDX                11
#define PB_V_IDX                    12
#define PB_LD_IDX                   13
#define PB_SMODE_IDX                14

// ★ 遥测档案选择 (15)：0 = 全量 41 路 / 1 = 标定精简 18 路（2026-10-22 新增）
//   同样是"只准往后加"：它必须排在最后，否则 Flash 里已有的 15 项会整体错位。
//   语义与消费者见 Host_Crtl.h 的 HOST_TEL_SEL_*；本参数**不经过** Flash_SyncTo_Param
//   （没有对应的控制器副本），由 Host_Crtl.c 每帧直接读 param_cache[TEL_SEL_IDX]。
#define TEL_SEL_IDX                 15

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
#define SERVO_KI                    param_cache[SERVO_KI_IDX]

#define ROUTE_DS                    param_cache[ROUTE_DS_IDX]
#define ROUTE_PB_V                  param_cache[PB_V_IDX]
#define ROUTE_PB_LD                 param_cache[PB_LD_IDX]
#define ROUTE_PB_SMODE              param_cache[PB_SMODE_IDX]

// 遥测档案选择：0.0 = 全量 41 路 / 1.0 = 标定精简 18 路（默认 0）
#define TEL_SEL_DEFAULT             (0.0f)
#define TEL_SEL                     param_cache[TEL_SEL_IDX]

void    Param_Init          (void);     // 初始化(加载或设默认值)
void    Param_Save          (void);     // 保存当前参数到 Flash —— **没改动就不写**（见 Param_Is_Dirty）
void    Param_Erase         (void);     // 擦除 Flash(下次启动恢复默认)
void    Flash_SyncTo_Param  (void);     // 将缓存区的值推送到实际参数

// 缓存区是否与 Flash 里的不一致（= 有未保存的改动）。0 = 一致。
// 实现：把 Flash 那一页读出来跟 param_cache 按位比 —— 不维护任何额外状态。
// 存在的理由：一次保存 = 擦掉一个 2 KB 扇区，而参数页"根层按 BACK 退出"是无条件调
// Param_Save() 的；不判一下，进去看一眼就白擦一次。
uint8   Param_Is_Dirty      (void);

#endif
