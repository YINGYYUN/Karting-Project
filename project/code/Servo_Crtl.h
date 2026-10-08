/*******************************************************************************
转向角测量与控制 - Servo_Crtl
数据来源：绝对值角度编码器（ENC_ABS_ANG，10ms 中断统一采集，见 cm7_0_isr.c）
标定依据：串口接收数据（角度编码器）.txt 中 15 次满幅摆动的统计
控制对象：Motor 3（MOTOR_SERVO），DIR + PWM 驱动
控制律  ：位置环 u = kp × (目标角 - 实测角)，含软限位、目标斜坡、误差死区
*******************************************************************************/
#ifndef _servo_crtl_h_
#define _servo_crtl_h_

#include "zf_common_typedef.h"

//============================== 标定参数（由实测统计得到） ==============================
// 实测（15 次满幅摆动分别打到左、右限位）：
//     右限位读数  均值 3659.3（最小 3600 / 最大 3698）
//     左限位读数  均值  654.5（最小  595 / 最大  719）
//     静止读数    2072
// 行程 = 3659.3 - 654.5 = 3004.8 计数，按方向盘"左右各 30°"折算
#define SERVO_ENC_MID           (2157.0f)       // 0° 对应的原始读数 = 行程中点 (3659.3+654.5)/2
#define SERVO_CNT_PER_DEG       (50.08f)        // 每 1° 方向盘对应的编码器计数 = 3004.8 / 60
                                                // 分辨率：1 计数 ≈ 0.020°

//============================== 角度极性 ==============================
// 约定：方向盘"向右"为角度正值
// 取 +1：读数增大方向 = 向右
// 取 -1：读数增大方向 = 向左（方向取反）
#define SERVO_DIR_SIGN          (-1)

//============================== 机械限位 ==============================
// 限位读数的重复性波动约 ±1°（右 98 计数、左 124 计数），来源是手动推力差异与限位处柔性，
// 因此不宜用"限位读数"当作角度基准，标定只依赖上面的中点与比例。
#define SERVO_LIMIT_DEG         (30.0f)         // 物理限位：方向盘左右各 30°
#define SERVO_ENC_LEFT_AVG      (654.5f)        // 左限位均值读数
#define SERVO_ENC_RIGHT_AVG     (3659.3f)       // 右限位均值读数
#define SERVO_ENC_LEFT_MIN      (595)           // 左限位实测极值（留档）
#define SERVO_ENC_RIGHT_MAX     (3698)          // 右限位实测极值（留档）

// 软限位：比物理限位留 SERVO_SOFT_MARGIN_DEG 余量，让控制器永远不会主动把方向盘推向限位结构。
// 余量需大于上述波动量（±1.2°），余量需要留出裕度
#define SERVO_SOFT_MARGIN_DEG   (2.0f)
#define SERVO_SOFT_LIMIT_DEG    (SERVO_LIMIT_DEG - SERVO_SOFT_MARGIN_DEG)   // 28.0°

//============================== 控制参数 ==============================
#define SERVO_T_S               (0.01f)         // 控制周期（秒），与 10ms 节拍一致
#define SERVO_KP_DEFAULT        (0.0f)          // 位置环比例增益的"上电默认值"
                                                // 实际生效值放在 Servo_Crtl_Kp，由参数页/Flash 提供
#define SERVO_U0_DEFAULT        (1300.0f)       // 摩擦截距前馈的"上电默认值"
                                                // 实测（整车落地、不前后运动）启动阈值约 1300 （注：摩擦似乎也和当前方向盘角度有关）
                                                // 实际生效值放在 Servo_Crtl_U0，由参数页/Flash 提供
#define SERVO_U_MAX             (6000.0f)       // 输出限幅（与 Motor_Set 刻度一致：±10000 = ±100%）
#define SERVO_ERR_DEADBAND_DEG  (0.2f)          // 误差死区，抑制静止抖动
#define SERVO_SLEW_DEG_PER_S    (120.0f)         // 目标变化率上限（度/秒），防止目标阶跃被全额灌进环路

//============================== 反馈源失效保护 ==============================
// 角度编码器掉线 / 悬空时的特征：SPI 读回全 0 或全 1，原始读数在 0 / 4095 交替。
// 正常行程是 595~3698，两端留余量后不会误判。
#define SERVO_ENC_RAW_INVALID_LOW   (3)         // 原始读数 <= 它视为无效
#define SERVO_ENC_RAW_INVALID_HIGH  (4092)      // 原始读数 >= 它视为无效

// 前提用"指令侧"的 |Servo_Out|：反馈一失效，误差就不再收敛、输出会顶到限幅，
// 很快超过 1800（实测启动阈值约 1300~1600），所以这是个可靠的"确实在出力"判据。
#define SERVO_ENC_LOST_OUT      (1800.0f)       // |Servo_Out| 超过它才算"在出力"
#define SERVO_ENC_LOST_MS       (200)           // 连续多少 ms 读到无效值就停机（锁存，Reset 才清）
#define SERVO_TICK_MS           (10)            // 节拍周期 ms，与 PIT 一致

// 输出极性：Motor 3 的 PWM 为正时方向盘向左转，而角度约定"向右为正"，
// 故控制器算出的 u 需取反后送电机。实测方向相反时改成 +1。
#define SERVO_MOTOR_SIGN        (-1)

//============================== 对外数据 ==============================
extern volatile int16 Servo_Ang_Raw;            // 当前原始读数 0 ~ 4095
extern volatile float Servo_Angle_Deg;          // 当前方向盘角度（度，向右为正）
extern volatile float Servo_Target_Deg;         // 目标角度（度，向右为正）
extern volatile float Servo_Err_Deg;            // 本拍误差（度）
extern volatile int16 Servo_Out;                // 本拍实际写入电机的输出
extern volatile uint8 Servo_Crtl_Enable;        // 0 = 输出强制 0 并清状态
extern volatile float Servo_Crtl_Kp;            // 位置环比例增益（上电默认 SERVO_KP_DEFAULT，参数页可改）
extern volatile float Servo_Crtl_U0;            // 摩擦截距前馈（上电默认 SERVO_U0_DEFAULT，参数页可改）
extern volatile uint8 Servo_Enc_Fault;          // 1 = 反馈源失效已锁存（Servo_Crtl_Reset 才清）

//============================== 接口 ==============================
void Servo_Crtl_Init  (void);                   // 初始化（清状态、不使能）
void Servo_Crtl_Reset (void);                   // 清目标 / 斜坡 / 误差 / 输出（目标归 0）
void Servo_Crtl_Tick  (void);                   // 10ms 调用：换算 + 位置环 + 输出

#endif
