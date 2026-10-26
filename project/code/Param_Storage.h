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

// 参数总数（2 个后轮电机 × 4 + 舵机 × 3 + 路表 × 13 + 遥测档案 × 1）
#define PARAM_COUNT                 (24)

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

// ★ 录制终点补点 (16)：1 = 停止录制时把"当前这一拍"强制补进路表，0 = 关
//   为什么需要：采样是按里程（ins_route_ds）触发的 ⇒ 按下停止那一刻若还没走过一个点距，
//   最后那一段就没有点，而**终点往往正打着大角**（收车/过弯尾）—— 回放跑到末尾会少一截、
//   且最后一点的方向与实际不一致，很危险。开着它，停止时一定会落下带当前打角的一点。
#define ROUTE_END_PT_IDX            16

// ★ 有效轴距 L (17)：回放的 Pure Pursuit 与航向增益 k = 2L/ld 都要它
//   2026-10-23 从 route_pb.h 的编译期常量 PB_L_M 提成运行期参数。
#define ROUTE_PB_L_IDX              17

// ★ 速度剖面 (18-20)：曲率限速 + 末段收油
//   pb_ve = 1 时，pb_v 从"恒定车速"变成"车速上限"，实际目标由下面两条决定：
//     ① 曲率限速：v ≤ √(alat / κ)，κ 由前后 kwin 个点的航向变化 / 弧长算出；
//     ② 末段收油：越接近末点，上限按 √(creep² + 2·alat·d) 收紧（d = 沿路径到末点的弧长）。
//   pb_ve = 0 时行为与以前**完全一样**（一个常数速度），所以它默认是 0。
#define PB_VE_IDX                   18
#define PB_ALAT_IDX                 19
#define PB_VMIN_IDX                 20

// ★ 航向增益手工覆盖 (21)：0 = 自动按 2L/ld（默认），> 0 = 直接用这个值做 A/B
#define PB_KH_IDX                   21

// ★★ 倒车录 / 倒序回放 (22, 23) —— 2026-10-24
//   用途：让车**倒着**沿一条路录一张表，再**正着**开回去（同一点的朝向与所需前轮转角完全复用，
//   只有遍历顺序相反）。这是队友 14.pp_for_backroute 那套做法，我们按同一语义实现。
//     rt_rrec = 1：录制时把 v 存成 |v|（REV 位照打）—— 这张表此后只用于"正车倒序回放"
//     rt_rply = 1：起跑时把点序原地倒过来（车正着开、沿表反向走）
//   两者必须与"表的出身"一致：过半点是 REV ⇒ 倒车录的表。不一致时 route_pb_start() 拒绝起跑，
//   原因写进 s_why（屏幕上看得到），避免"以为在正着开、其实表是正的"这种最危险的组合。
#define RT_RREC_IDX                 22
#define RT_RPLY_IDX                 23

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

// 录制终点补点：1.0 = 停止录制时强制补最后一个点（默认开），0.0 = 关
#define ROUTE_END_PT_DEFAULT        (1.0f)
#define ROUTE_END_PT                param_cache[ROUTE_END_PT_IDX]

// 有效轴距 m（默认值来自 route_pb.h 的 PB_L_DEFAULT，量程也用它那对 MIN/MAX）
#define ROUTE_PB_L                  param_cache[ROUTE_PB_L_IDX]

// 速度剖面：使能 / 横向加速度 / 地板
#define ROUTE_PB_VE                 param_cache[PB_VE_IDX]
#define ROUTE_PB_ALAT               param_cache[PB_ALAT_IDX]
#define ROUTE_PB_VMIN               param_cache[PB_VMIN_IDX]

// 航向增益手工覆盖：0 = 自动（2L/ld）
#define ROUTE_PB_KH                 param_cache[PB_KH_IDX]

// 倒车录表 / 倒序回放（0/1）
#define ROUTE_REV_REC               param_cache[RT_RREC_IDX]
#define ROUTE_REV_PLAY              param_cache[RT_RPLY_IDX]

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
