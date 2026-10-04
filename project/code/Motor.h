/*******************************************************************************
电机相关
*******************************************************************************/


#ifndef __MOTOR_H__
#define __MOTOR_H__


#include "zf_common_headfile.h"


/**********************************************************/
/*[S] 电机驱动 [S]----------------------------------------*/
/**********************************************************/

// 电机驱动(DRV8701)引脚配置	
#define MOTOR_1_DIR_PIN             P05_1
#define MOTOR_1_PWM_CHANNEL         TCPWM_CH09_P05_0
#define MOTOR_2_DIR_PIN             P05_3
#define MOTOR_2_PWM_CHANNEL         TCPWM_CH11_P05_2
#define MOTOR_3_DIR_PIN             P10_3
#define MOTOR_3_PWM_CHANNEL         TCPWM_CH30_P10_2
#define MOTOR_4_DIR_PIN             P09_1
#define MOTOR_4_PWM_CHANNEL         TCPWM_CH24_P09_0

// 电机调用二次宏定义
// 体现在 Motor_Set() 函数的调用替代中
#define MOTOR_LEFT_REAR             1               // 左后轮
#define MOTOR_RIGHT_REAR            2               // 右后轮
#define MOTOR_SERVO                 3               // “舵机”
// 通道 4 预留，暂不启用

// PWM 频率 10kHz
#define MOTOR_PWM_FREQ              ( 10000 )

// 电机驱动引脚初始化
void    Motor_init                  (void);
// 设置duty,范围-10000~10000
void    Motor_Set                   (uint8 motor, int16 duty);
// 电机速度归零
void    Motor_SET_Zero_ALL          (void);
/**********************************************************/
/*----------------------------------------[E] 电机驱动 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 编码器 [S]------------------------------------------*/
/**********************************************************/

// 左后轮 正交编码器
#define ENCODER_LEFT_REAR           TC_CH20_ENCODER
#define ENC_LR_P_CH1                TC_CH20_ENCODER_CH1_P08_1
#define ENC_LR_P_CH2                TC_CH20_ENCODER_CH2_P08_2

// 右后轮 正交编码器
#define ENCODER_RIGHT_REAR          TC_CH07_ENCODER
#define ENC_RR_P_CH1                TC_CH07_ENCODER_CH1_P07_6
#define ENC_RR_P_CH2                TC_CH07_ENCODER_CH2_P07_7

// 编码器调用二次宏定义
#define ENC_LR_GET()                (-encoder_get_count(ENCODER_LEFT_REAR))
#define ENC_LR_CLEAR()              encoder_clear_count(ENCODER_LEFT_REAR)

#define ENC_RR_GET()                ( encoder_get_count(ENCODER_RIGHT_REAR))
#define ENC_RR_CLEAR()              encoder_clear_count(ENCODER_RIGHT_REAR)

// 编码器计数值全局变量 （单周期增量，不存储累加值）
extern int16 ENC_LR_CNT;            // 左后轮 正交编码器
extern int16 ENC_RR_CNT;            // 右后轮 正交编码器

// 编码器累加值全局变量 （累计值）
// 更多在于调试性质的观测
extern int32 ENC_LR_SUM;            // 左后轮 正交编码器
extern int32 ENC_RR_SUM;            // 右后轮 正交编码器

// 磁编码器（MENC15A，转向减速箱高速侧）对外快照，由 10ms 中断统一采集刷新
extern uint16 ENC_MAG_ANG;          // 编码器轴 绝对角 0~32767（单圈）
extern int16  ENC_MAG_OFF;          // 相对上一次的角度增量（带符号）
extern int16  ENC_MAG_SPD;          // 转速原始值
extern int16  ENC_MAG_REV;          // AREV 圈数（编码器轴每转一整圈 ±1）

// 角度编码器（360° 绝对式，转向柱侧）对外快照，由 10ms 中断统一采集刷新
extern int16  ENC_ABS_ANG;          // 原始角 0~4095（4096 对应一圈）
extern int16  ENC_ABS_OFF;          // 相对上一次的增量

// 编码器相关数据重置（只清软件侧数据，不碰 SPI）
void ENC_All_Clear(void);
/**********************************************************/
/*------------------------------------------[E] 编码器 [E]*/
/**********************************************************/


#endif
