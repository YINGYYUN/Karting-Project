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
    // ENC 只保留两路（左后轮 / 右后轮）
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

// 编码器计数值全局变量 （单周期增量，不存储累加值）
int16 ENC_LR_CNT = 0;
int16 ENC_RR_CNT = 0;

// 编码器累加值全局变量 （累计值）
// 更多在于调试性质的观测
int32 ENC_LR_SUM = 0;
int32 ENC_RR_SUM = 0;

// 编码器相关数据重置
void ENC_All_Clear(void)
{
    ENC_LR_CNT = 0; ENC_RR_CNT = 0;
    ENC_LR_SUM = 0; ENC_RR_SUM = 0;
    ENC_LR_CLEAR(); ENC_RR_CLEAR();
}
/**********************************************************/
/*------------------------------------------[E] 编码器 [E]*/
/**********************************************************/
