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

// PWM 频率 10kHz
#define MOTOR_PWM_FREQ              ( 10000 )

// 电机驱动引脚初始化
void    Motor_init                  (void);
// 设置duty,范围-10000~10000
void    Motor_Set                   (uint8 motor, int16 duty);
// 电机速度归零
void    Motor_SET_Zero_ALL              (void);
/**********************************************************/
/*----------------------------------------[E] 电机驱动 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 编码器 [S]------------------------------------------*/
/**********************************************************/

// 编码器(正交)通道定义二次宏定义
// 1 
#define ENCODER_1 				    TC_CH20_ENCODER
#define ENC_1_P_CH1				    TC_CH20_ENCODER_CH1_P08_1
#define ENC_1_P_CH2				    TC_CH20_ENCODER_CH2_P08_2

// 2 
#define ENCODER_2 				    TC_CH07_ENCODER
#define ENC_2_P_CH1				    TC_CH07_ENCODER_CH1_P07_6
#define ENC_2_P_CH2				    TC_CH07_ENCODER_CH2_P07_7

// 3 
#define ENCODER_3 				    TC_CH58_ENCODER
#define ENC_3_P_CH1				    TC_CH58_ENCODER_CH1_P17_3
#define ENC_3_P_CH2				    TC_CH58_ENCODER_CH2_P17_4

// 4 
#define ENCODER_4 				    TC_CH27_ENCODER
#define ENC_4_P_CH1				    TC_CH27_ENCODER_CH1_P19_2
#define ENC_4_P_CH2				    TC_CH27_ENCODER_CH2_P19_3

// 编码器(正交)调用二次宏定义
#define ENC_1_GET()                (-encoder_get_count(ENCODER_1))
#define ENC_1_CLEAR()              encoder_clear_count(ENCODER_1)

#define ENC_2_GET()                ( encoder_get_count(ENCODER_2))
#define ENC_2_CLEAR()              encoder_clear_count(ENCODER_2)

#define ENC_3_GET()                (-encoder_get_count(ENCODER_3))
#define ENC_3_CLEAR()              encoder_clear_count(ENCODER_3)

#define ENC_4_GET()                ( encoder_get_count(ENCODER_4))
#define ENC_4_CLEAR()              encoder_clear_count(ENCODER_4)

// 编码器计数值全局变量 （单周期增量，不存储累加值）
extern int16 ENC_1_CNT;
extern int16 ENC_2_CNT;
extern int16 ENC_3_CNT;
extern int16 ENC_4_CNT;

// 编码器累加值全局变量 （累计值）
// 更多在于调试性质的观测
extern int32 ENC_1_SUM;
extern int32 ENC_2_SUM;
extern int32 ENC_3_SUM;
extern int32 ENC_4_SUM;

// 编码器相关数据重置
void ENC_All_Clear(void);
/**********************************************************/
/*------------------------------------------[E] 编码器 [E]*/
/**********************************************************/


#endif
