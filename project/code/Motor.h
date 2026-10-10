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
#define MOTOR_1_DIR_PIN             P09_1
#define MOTOR_1_PWM_CHANNEL         TCPWM_CH24_P09_0	
#define MOTOR_2_DIR_PIN             P10_3
#define MOTOR_2_PWM_CHANNEL         TCPWM_CH30_P10_2
#define MOTOR_3_DIR_PIN             P05_1
#define MOTOR_3_PWM_CHANNEL         TCPWM_CH09_P05_0
#define MOTOR_4_DIR_PIN             P05_3
#define MOTOR_4_PWM_CHANNEL         TCPWM_CH11_P05_2

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

// ---- 方向编码器（脉冲 + 方向） ----
// 方向由逐飞驱动给出（encoder_get_count 的符号即 DIR 电平），软件只做极性对齐与滑动平均

// 左后轮：P17.3(脉冲) / P17.4(方向)
#define ENCODER_LEFT_REAR           TC_CH58_ENCODER
#define ENC_LR_P_CH1                TC_CH58_ENCODER_CH1_P17_3
#define ENC_LR_P_CH2                TC_CH58_ENCODER_CH2_P17_4

// 右后轮：P19.2(脉冲) / P19.3(方向)
#define ENCODER_RIGHT_REAR          TC_CH27_ENCODER
#define ENC_RR_P_CH1                TC_CH27_ENCODER_CH1_P19_2
#define ENC_RR_P_CH2                TC_CH27_ENCODER_CH2_P19_3

// 方向编码器数据处理参数
#define ENC_WIN_MAX                 (16)    // 滑动平均窗口上限（缓冲按它开）
#define ENC_WIN_DEFAULT             (4)     // 滑动平均窗口默认值（1 = 不滤波）

// 方向编码器极性
// 手把左后轮朝车前进方向转，若 ENC_LR_CNT 为负则把 ENC_LR_INVERT 置 1；右后轮同理
#define ENC_LR_INVERT               (0)
#define ENC_RR_INVERT               (1)

// 方向编码器一路的读取状态（由 ENC_Read_One 内部维护）
typedef struct
{
    encoder_index_enum      idx;            // 定时器通道
    encoder_channel1_enum   pulse_pin;      // 脉冲脚
    encoder_channel2_enum   dir_pin;        // 方向脚（交给驱动配置，软件不读它）

    int16   win_buf[ENC_WIN_MAX];           // 滑动平均缓冲
    int32   win_sum;                        // 缓冲内计数之和
    uint8   win_idx;                        // 缓冲写指针（>= 窗口长度 表示需要整体预填）
} Enc_Read_t;

extern Enc_Read_t   Enc_LR_Read;            // 左后轮
extern Enc_Read_t   Enc_RR_Read;            // 右后轮

// 编码器计数值全局变量 （单周期增量，已含方向符号与滑动平均）
extern int16 ENC_LR_CNT;                    // 左后轮
extern int16 ENC_RR_CNT;                    // 右后轮

// 编码器累加值全局变量 （累计值）
// 更多在于调试性质的观测
extern int32 ENC_LR_SUM;                    // 左后轮
extern int32 ENC_RR_SUM;                    // 右后轮

// ---- 磁编码器 （SPI通信）----

// 磁编码器（MENC15A，转向减速箱高速侧）对外快照，由 10ms 中断统一采集刷新
extern uint16 ENC_MAG_ANG;                  // 编码器轴 绝对角 0~32767（单圈）
extern int16  ENC_MAG_OFF;                  // 相对上一次的角度增量（带符号）
extern int16  ENC_MAG_SPD;                  // 转速原始值
extern int16  ENC_MAG_REV;                  // AREV 圈数（编码器轴每转一整圈 ±1）

// 磁编码器硬编码使能
// 0关 1开
// 关闭将一同取消相关引脚的初始化
#define ENC_MAG_ENABLE                      1
void ENC_MAG_Init(void);
// ---- 角度编码器（SPI通信） ----

// 角度编码器（360° 绝对式，转向柱侧）对外快照，由 10ms 中断统一采集刷新
extern int16  ENC_ABS_ANG;                  // 原始角 0~4095（4096 对应一圈）
extern int16  ENC_ABS_OFF;                  // 相对上一次的增量

// 方向编码器读取初始化（两路一起：配置引脚 + 预读初值），由 Motor_init() 调用
void ENC_Read_Init(void);
// 单路 10ms 读取更新：取驱动读数 -> 清计数 -> 极性对齐 -> 滑动平均 -> 累加
void ENC_LR_Update(void);
void ENC_RR_Update(void);

// 编码器相关数据重置（只清软件侧数据，不碰 SPI）
void ENC_All_Clear(void);
/**********************************************************/
/*------------------------------------------[E] 编码器 [E]*/
/**********************************************************/


#endif
