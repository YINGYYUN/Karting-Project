/*******************************************************************************
电机相关
*******************************************************************************/


#include "zf_common_headfile.h"


/**********************************************************/
/*[S] 电机驱动 [S]----------------------------------------*/
/**********************************************************/

// 电机驱动(DRV8701)引脚初始化
void Motor_init (void)
{
    // DIR
    gpio_init(MOTOR_1_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_2_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_3_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_4_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    // PWM
    pwm_init(MOTOR_1_PWM_CHANNEL, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_2_PWM_CHANNEL, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_3_PWM_CHANNEL, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_4_PWM_CHANNEL, MOTOR_PWM_FREQ, 0);
    // ENC 
    // 正交编码器 左后轮 / 右后轮
	encoder_quad_init(ENCODER_LEFT_REAR,  ENC_LR_P_CH1, ENC_LR_P_CH2);
	encoder_quad_init(ENCODER_RIGHT_REAR, ENC_RR_P_CH1, ENC_RR_P_CH2);
}

//----------------------------------------------------------
// 函数简介     设置电机速度
// 使用示例     Motor_Set(1, 2500);
// 参数说明     motor范围：1-4
// 参数说明     duty范围：-10000~10000（注：duty本意是没有负数的）
//----------------------------------------------------------
void Motor_Set (uint8 motor, int16 duty)
{
    gpio_pin_enum    dir_pin;// 方向引脚
    pwm_channel_enum pwm_ch;// PWM引脚

    switch (motor)
    {
        case 1:
        {
            dir_pin = MOTOR_1_DIR_PIN;
            pwm_ch  = MOTOR_1_PWM_CHANNEL;
            duty = -duty;
            break;
        }
        case 2:
        {
            dir_pin = MOTOR_2_DIR_PIN;
            pwm_ch  = MOTOR_2_PWM_CHANNEL;
            duty = -duty;
            break;
        }
        case 3:
        {
            dir_pin = MOTOR_3_DIR_PIN;
            pwm_ch  = MOTOR_3_PWM_CHANNEL;
            break;
        }
        case 4:
        {
            dir_pin = MOTOR_4_DIR_PIN;
            pwm_ch  = MOTOR_4_PWM_CHANNEL;
            break;
        }
        default:
            break;
    }


    if(duty > 0)
    {
        gpio_high(dir_pin);
    }
    else
    {
        gpio_low(dir_pin);
        duty = -duty;
    }


    if(duty > PWM_DUTY_MAX) // 限幅
    {
        duty = PWM_DUTY_MAX;
    }

    pwm_set_duty(pwm_ch, (uint16)duty);
}


// 电机速度归零
void Motor_SET_Zero_ALL(void)
{
    Motor_Set(1,0);
    Motor_Set(2,0);
    Motor_Set(3,0);
    Motor_Set(4,0);
}
/**********************************************************/
/*----------------------------------------[E] 电机驱动 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 编码器 [S]------------------------------------------*/
/**********************************************************/

//============================ 正交编码器 （左右后轮，10ms 周期增量） ============================
// 编码器计数值全局变量 （单周期增量，不存储累加值）
int16 ENC_LR_CNT = 0;               // 左后轮 正交编码器
int16 ENC_RR_CNT = 0;               // 右后轮 正交编码器

// 编码器累加值全局变量 （累计值）
// 更多在于调试性质的观测
int32 ENC_LR_SUM = 0;               // 左后轮 正交编码器
int32 ENC_RR_SUM = 0;               // 右后轮 正交编码器

//============================ 磁编码器 （MENC15A，转向减速箱高速侧） ============================
// 这 4 个是本工程对外的唯一数据源，全部由 10ms 中断统一采集刷新（见 cm7_0_isr.c）
// 注意：不要再从其它地方直接调 zf_device_menc15a.c 的读取函数，
//       否则会和中断里的读交错，破坏其内部"上次位置"基准（OFF 会算出错值）
uint16 ENC_MAG_ANG = 0;             // 编码器轴 绝对角 0~32767（单圈；32768 对应一圈）
int16  ENC_MAG_OFF = 0;             // 相对上一次的角度增量（带符号）
int16  ENC_MAG_SPD = 0;             // 转速原始值（含偶发错读，消费方自行判断）
int16  ENC_MAG_REV = 0;             // AREV 圈数（编码器轴每转一整圈 ±1，掉电清零）

//============================ 角度编码器 （360° 绝对式，转向柱侧） ============================
// 同样由 10ms 中断统一采集刷新
int16  ENC_ABS_ANG = 0;             // 原始角 0~4095（4096 对应一圈）
int16  ENC_ABS_OFF = 0;             // 相对上一次的增量（超过半圈按最短路径）

//============================ 传感器掉线 / 悬空时的数据特征 ============================
// SPI 主机自行产生时钟，从机不在时传输仍会完成，读回全 0 或全 1；不会超时，也不会卡死。
// 两个传感器解码后的表现不同：
//
//   角度编码器（12 位，取 16bit 数据的 >>4）
//       ENC_ABS_ANG   0 / 4095 交替
//       ENC_ABS_OFF   +1 / -1 交替（驱动的最短路径过零把 ±4095 折叠成 ±1）
//       换算角度      0 / 3599 交替
//
//   磁编码器（15 位角度、15 位速度、9 位圈数）
//       ENC_MAG_ANG   0 / 32767 交替
//       ENC_MAG_SPD   0 / -2 交替（0x7FFF 解为 32767，减 32768 得 -1，乘 2.36 约为 -2）
//       ENC_MAG_REV   0 / -1 交替（9 位全 1 解码为 -1）
//       掉线时只有 ENC_MAG_ANG 能反映出来，SPD / REV 的解码结果与正常小值无法区分。
//
//   可用的掉线判据：连续 N 次原始读数只落在全 0 / 全 1 两种极端值。

// 编码器相关数据统一重置
// 只清"软件侧"的数据，不碰 SPI：磁编码器 / 角度编码器的快照会在下一个 10ms 中断里被重新刷新
void ENC_All_Clear(void)
{
    ENC_LR_CNT = 0; ENC_RR_CNT = 0;
    ENC_LR_SUM = 0; ENC_RR_SUM = 0;
    ENC_LR_CLEAR(); ENC_RR_CLEAR();

    ENC_MAG_ANG = 0; ENC_MAG_OFF = 0; ENC_MAG_SPD = 0; ENC_MAG_REV = 0;
    ENC_ABS_ANG = 0; ENC_ABS_OFF = 0;
}
/**********************************************************/
/*------------------------------------------[E] 编码器 [E]*/
/**********************************************************/
