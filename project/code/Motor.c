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
    // ENC：方向编码器（脉冲 + 方向）左后轮 / 右后轮
    ENC_Read_Init();
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
            duty = duty;
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

//============================ 方向编码器 （左右后轮，10ms 周期增量） ============================
// 读取方式：脉冲脚计数、方向脚读电平 + 毛刺门限 + 方向去抖 + 滑动平均。

// 编码器计数值全局变量 （单周期增量，已含方向符号与滑动平均）
int16 ENC_LR_CNT = 0;               // 左后轮
int16 ENC_RR_CNT = 0;               // 右后轮

// 编码器累加值全局变量 （累计值）
// 更多在于调试性质的观测
int32 ENC_LR_SUM = 0;               // 左后轮
int32 ENC_RR_SUM = 0;               // 右后轮

// 两路读取状态（引脚在这里落地）
Enc_Read_t Enc_LR_Read =
{
    .idx       = ENCODER_LEFT_REAR,
    .pulse_pin = ENC_LR_P_CH1,
    .dir_pin   = ENC_LR_P_CH2,
    .dir_gpio  = ENC_LR_DIR_GPIO,
};

Enc_Read_t Enc_RR_Read =
{
    .idx       = ENCODER_RIGHT_REAR,
    .pulse_pin = ENC_RR_P_CH1,
    .dir_pin   = ENC_RR_P_CH2,
    .dir_gpio  = ENC_RR_DIR_GPIO,
};

//--------------------------------------
// 函数简介     一路读取状态复位
// 参数说明     e	该路读取状态
// 备注信息     清硬件计数并重新预读方向；滑动平均用哨兵值表示"下次整体预填"
//--------------------------------------
static void ENC_Read_Reset (Enc_Read_t *e)
{
    uint8 i;

    encoder_clear_count(e->idx);

    e->raw_last = 0;
    e->dir_last = (uint8)gpio_get_level(e->dir_gpio);
    e->dir_pend = e->dir_last;
    e->dir_cnt  = 0;

    for (i = 0; i < ENC_WIN_MAX; i++)
    {
        e->win_buf[i] = 0;
    }
    e->win_sum = 0;
    e->win_idx = ENC_WIN_MAX;
}

//--------------------------------------
// 函数简介     一路方向编码器读取
// 参数说明     e	该路读取状态
// 参数说明     invert	测速取反（0/1）
// 返回参数     int16	本拍计数增量（已含方向符号与滑动平均）
// 备注信息     逐飞 encoder_dir_init 把 TCPWM 配成"每条脉冲 +1"的单向计数器，
//              方向是靠 encoder_get_count() 读 DIR 电平、给整个计数值取反得到的。
//              因此只要 DIR 在两次采样之间变过，"本次读数 - 上次读数"就是错的：
//              这里先把库加的符号还原成原始计数，再用去抖后的方向重新定符号。
//--------------------------------------
static int16 ENC_Read_One (Enc_Read_t *e, uint8 invert)
{
    int16 c;                // 库读数（±计数，符号由库读到的方向电平决定）
    int16 raw;              // 还原出的原始计数（未定符号）
    int16 d_raw;            // 原始计数的单拍增量
    int16 delta;            // 定符号后的本拍增量
    uint8 d;                // 本次读到的方向电平
    uint8 n;                // 实际使用的窗口长度
    uint8 i;

    c = encoder_get_count(e->idx);
    d = (uint8)gpio_get_level(e->dir_gpio);

    // 1) 还原原始计数：库的符号只取决于方向电平，按本次读到的电平反推即可
    raw = (d != 0) ? c : (int16)(-c);

    // 2) 原始计数的单拍增量 + 毛刺门限
    d_raw = (int16)(raw - e->raw_last);
    if ((d_raw > ENC_DLT_GLITCH_MAX) || (d_raw < -ENC_DLT_GLITCH_MAX))
    {
        d_raw = 0;
    }
    e->raw_last = raw;

    // 3) 方向去抖：新电平要连续 ENC_DIR_DEBOUNCE 拍一致才认换向，认之前沿用旧方向
    //    （换向必经过 0 速，晚认一两拍几乎不损失精度；好处是一次毛刺不会被判成两次换向）
    if (d == e->dir_last)
    {
        e->dir_pend = d;
        e->dir_cnt  = 0;
    }
    else
    {
        if (d == e->dir_pend)
        {
            if (e->dir_cnt < 250) { e->dir_cnt++; }
        }
        else
        {
            e->dir_pend = d;
            e->dir_cnt  = 1;
        }
        if (e->dir_cnt >= ENC_DIR_DEBOUNCE)
        {
            e->dir_last = d;
            e->dir_cnt  = 0;
        }
    }

    // 4) 用去抖后的方向定符号；再用 invert 做一次测速极性对齐
    delta = (e->dir_last != 0) ? d_raw : (int16)(-d_raw);
    if (invert)
    {
        delta = (int16)(-delta);
    }

    // 5) 滑动平均（窗口 ENC_WIN_DEFAULT）
    n = ENC_WIN_DEFAULT;
    if (n < 1u)          { n = 1u; }
    if (n > ENC_WIN_MAX) { n = ENC_WIN_MAX; }

    if (e->win_idx >= n)            // 首次 / 窗口变化：用本拍值把窗口整体预填，避免前几拍虚低
    {
        for (i = 0; i < ENC_WIN_MAX; i++)
        {
            e->win_buf[i] = delta;
        }
        e->win_sum = (int32)delta * (int32)n;
        e->win_idx = 0;
    }
    else
    {
        e->win_sum += (int32)delta - (int32)e->win_buf[e->win_idx];
        e->win_buf[e->win_idx] = delta;
        e->win_idx++;
        if (e->win_idx >= n) { e->win_idx = 0; }
    }

    return (int16)(e->win_sum / (int32)n);
}

//--------------------------------------
// 函数简介     方向编码器读取初始化
// 使用示例     在 Motor_init() 里调用一次即可
// 备注信息     两路一起：配置引脚（脉冲计数 + 方向 GPIO）+ 预读初值
//--------------------------------------
void ENC_Read_Init (void)
{
    encoder_dir_init(Enc_LR_Read.idx, Enc_LR_Read.pulse_pin, Enc_LR_Read.dir_pin);
    encoder_dir_init(Enc_RR_Read.idx, Enc_RR_Read.pulse_pin, Enc_RR_Read.dir_pin);

    ENC_Read_Reset(&Enc_LR_Read);
    ENC_Read_Reset(&Enc_RR_Read);

    ENC_LR_CNT = 0; ENC_RR_CNT = 0;
    ENC_LR_SUM = 0; ENC_RR_SUM = 0;
}

//--------------------------------------
// 函数简介     左后轮 10ms 读取更新
// 使用示例     在 PIT 中断里调用，内部已累加进 ENC_LR_SUM
//--------------------------------------
void ENC_LR_Update (void)
{
    ENC_LR_CNT = ENC_Read_One(&Enc_LR_Read, ENC_LR_INVERT);
    ENC_LR_SUM += ENC_LR_CNT;
}

//--------------------------------------
// 函数简介     右后轮 10ms 读取更新
//--------------------------------------
void ENC_RR_Update (void)
{
    ENC_RR_CNT = ENC_Read_One(&Enc_RR_Read, ENC_RR_INVERT);
    ENC_RR_SUM += ENC_RR_CNT;
}

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
    ENC_Read_Reset(&Enc_LR_Read);
    ENC_Read_Reset(&Enc_RR_Read);

    ENC_LR_CNT = 0; ENC_RR_CNT = 0;
    ENC_LR_SUM = 0; ENC_RR_SUM = 0;

    ENC_MAG_ANG = 0; ENC_MAG_OFF = 0; ENC_MAG_SPD = 0; ENC_MAG_REV = 0;
    ENC_ABS_ANG = 0; ENC_ABS_OFF = 0;
}
/**********************************************************/
/*------------------------------------------[E] 编码器 [E]*/
/**********************************************************/
