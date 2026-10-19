/*******************************************************************************
调试
*******************************************************************************/


#include "zf_common_headfile.h"
#include "ins_imu.h"

/**********************************************************/
/*[S] 界面样式 [S]----------------------------------------*/
/**********************************************************/

// [二级界面]Debug模式界面
void Debug_Page_Menu_UI(void)
{
    ips200_show_string(8  ,0  , "[Debug]");
    ips200_show_string(0  ,16 , "==============================");
    ips200_printf(10 ,32 , "WIFI-SPI  %d", wifi_spi_inited);
    ips200_show_string(10 ,48 , "MOTOR");
    ips200_show_string(10 ,64 , "MOTOR-PID");
    ips200_show_string(10 ,80 , "IMU");
    ips200_show_string(10 ,96 , "MAG-ENC");
    ips200_show_string(10 ,112, "ABS-ENC");
    ips200_show_string(10 ,128, "SERVO");
    ips200_show_string(10 ,144, "UART-RC");
}

// [三级界面]电机调试界面
void Debug_MOTOR_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-MOTOR");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "PWM 01:###");
    ips200_show_string(10 ,48 , "PWM 02:###");
    ips200_show_string(10 ,64 , "PWM 03:###");
    ips200_show_string(10 ,80 , "PWM 04:###");
    // 空行
    ips200_show_string(10 ,112, "ENC: ");
    ips200_show_string(10 ,128, "LR:###        RR:###");
    // 空行
    ips200_show_string(10 ,176, "SUM:");
    ips200_show_string(10 ,192, "LR:###");
    ips200_show_string(10 ,208, "RR:###");
}

// [三级界面]Motor_PID调试界面   
// 速度环
void Debug_Motor_PID_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-MOTOR-PID");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "TAR LR:###    ENC:###");
    ips200_show_string(10 ,48 , "TAR RR:###    ENC:###");
    // 心跳（距上次喂心跳的 ms）/ 编码器脱落锁存标志
    ips200_show_string(10 ,80 , "HB ms:#####");
    ips200_show_string(10 ,96 , "FLT LR:#  RR:#");
    ips200_show_string(10 ,112, "PWM LR:###");
    ips200_show_string(10 ,128, "PWM RR:###");
    // 空行
    ips200_show_string(10 ,192, "SUM:");
    ips200_show_string(10 ,208, "LR:###");
    ips200_show_string(10 ,224, "RR:###");
}

// [三级界面]IMU调试界面   
// IMU (实际解算交给另一核)
void Debug_IMU_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-IMU");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "Roll :###");
    ips200_show_string(10 ,48 , "Y a w:###");
    ips200_show_string(10 ,64 , "Pitch:###");
    // 空行
    ips200_show_string(10 ,96 , "Gyro Calib");
    ips200_show_string(10 ,112, "Yaw Reset");
}

// [三级界面]磁编码器调试界面
// MENC15A 15位磁编码器（硬件 SPI3/SCB6, P3.0/3.1/3.2, CS P3.3）
void Debug_MENC15A_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-MAG-ENC");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "ABS:#####  OFF:#####");
    ips200_show_string(10 ,48 , "SPD:#####");
}

// [三级界面]绝对值角度编码器调试界面
// 逐飞 360° 绝对式角度传感器（硬件 SPI4/SCB5, P7.0/7.1/7.2, CS P7.3）
// ANG 为原始值 0~4095 对应 0~360°，DEG 为换算后的 0.1° 单位值
void Debug_ABS_ENCODER_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-ABS-ENC");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "ANG:#####  OFF:#####");
    ips200_show_string(10 ,48 , "DEG:#####");
}

// [三级界面]SERVO调试界面
// 转向角测量 + 位置环调试（上/下键 = 目标角 ±1°，确认键 = 切换闭环使能）
void Debug_SERVO_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-SERVO");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "ANG:#####  DEG:+##.#");
    ips200_show_string(10 ,48 , "TAR:+##.#  ERR:+##.#");
    ips200_show_string(10 ,64 , "ENABLE:0   OUT:#####");
    ips200_show_string(10 ,80 , "FLT:#");
}

// [三级界面]UART接收机调试界面
// 逐飞 UART 接收机（SBUS 帧，UART_4 = SCB2，RX = P14.0），CH1~CH6 量程 0~2047（中位约 1024）
void Debug_UART_RC_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-UART-RC");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "CH1:####");
    ips200_show_string(10 ,48 , "CH2:####");
    ips200_show_string(10 ,64 , "CH3:####");
    ips200_show_string(10 ,80 , "CH4:####");
    ips200_show_string(10 ,96 , "CH5:####");
    ips200_show_string(10 ,112, "CH6:####");
    ips200_show_string(10 ,144, "ST:#   FRM:#####");
}

// ★ WiFi 选择连接界面已**统一**到 Host_Crtl（唯一实现 = `Host_Crtl_WiFi_Select()`）——
//   本文件原来的 `Debug_WiFi_SPI_UI()` + `Debug_WiFi_SSID[]/Debug_WiFi_PASS[]`
//   （第二份界面 + 第二张候选表）已删除。
//   候选表 / 密码 / 目标 IP 现在只有 `Host_Crtl.h` 一份（`HOST_WIFI_LIST` / `HOST_TARGET_*`），
//   改那里两个入口同时生效。**不要再在这里复制一份。**

/**********************************************************/
/*----------------------------------------[E] 界面样式 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 菜单逻辑 [S]----------------------------------------*/
/**********************************************************/

// 相关函数提前声明
// ※ 原 `Debug_WiFi_SPI()` 已删除：WiFi 界面统一走 `Host_Crtl_WiFi_Select()`（见调试页第 1 项）
int Debug_Motor         (void);
int Debug_Motor_PID     (void);
int Debug_IMU           (void);
int Debug_MENC15A       (void);
int Debug_ABS_ENCODER   (void);
int Debug_SERVO         (void);
int Debug_UART_RC       (void);

// [二级界面]Debug模式界面
int Debug_Page_Menu(void)
{
    // Debug模式选项 标志位
    uint8_t Debug_Page_flag = 1;

    Debug_Page_Menu_UI();
    ips200_show_string(0  ,32 , ">");

    // 重置计时参考值
    Time_Count1 = 0;
    Time_Count2 = 0;

    while(1)
    {
        // 存储确认键被按下时Debug_Page_flag的值的临时变量，默认为无效值0
		uint8_t Debug_Page_flag_temp = 0;
		// 上/下按键是否被按下过
		uint8_t key_pressed = 0;


        /* 按键处理*/
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            key_pressed = 1;
            Debug_Page_flag --;
            if (Debug_Page_flag < 1)Debug_Page_flag = 8;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN); 
            key_pressed = 1;
            Debug_Page_flag ++;
            if (Debug_Page_flag > 8)Debug_Page_flag = 1;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            Debug_Page_flag_temp = Debug_Page_flag;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))    
        {
			key_clear_state(KEY_BACK);
            // 返回上一级界面
            return 0;   
        }


        /* 模式跳转*/
        if (Debug_Page_flag_temp == 1)
        {
            ips200_clear();
            // ★ WiFi 选择连接界面统一在 Host_Crtl（候选表 / 密码 / 目标 IP 只有那一份配置）
            Host_Crtl_WiFi_Select();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI();
            key_pressed = 1;
        }
        else if (Debug_Page_flag_temp == 2)
        {
            ips200_clear();
            Debug_Motor();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI();
            key_pressed = 1;
        }
        else if (Debug_Page_flag_temp == 3)
        {
            ips200_clear();
            Debug_Motor_PID();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI();
            key_pressed = 1;
        }
        else if (Debug_Page_flag_temp == 4)
        {
            ips200_clear();
            Debug_IMU();

            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI();
            key_pressed = 1;
        }
        else if (Debug_Page_flag_temp == 5)
        {
            ips200_clear();
            Debug_MENC15A();

            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI();
            key_pressed = 1;
        }
        else if (Debug_Page_flag_temp == 6)
        {
            ips200_clear();
            Debug_ABS_ENCODER();

            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI();
            key_pressed = 1;
        }
        else if (Debug_Page_flag_temp == 7)
        {
            ips200_clear();
            Debug_SERVO();

            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI();
            key_pressed = 1;
        }
        else if (Debug_Page_flag_temp == 8)
        {
            ips200_clear();
            Debug_UART_RC();

            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI();
            key_pressed = 1;
        }

        
        /* 显示更新*/
        if (key_pressed)
        {
			// 清理光标
			ips200_show_string(0  ,32 , " ");
			ips200_show_string(0  ,48 , " ");
			ips200_show_string(0  ,64 , " ");
			ips200_show_string(0  ,80 , " ");
			ips200_show_string(0  ,96 , " ");
			ips200_show_string(0  ,112, " ");
			ips200_show_string(0  ,128, " ");
			ips200_show_string(0  ,144, " ");
			// 显示光标
			ips200_show_string(0  ,16 + 16*Debug_Page_flag , ">");
        }
    }
}
/**********************************************************/
/*----------------------------------------[E] 菜单逻辑 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 调试逻辑 [S]----------------------------------------*/
/**********************************************************/

//  #   #  #####  #####  #####  
//  #   #    #    #        #    
//  # # #    #    #####    #    
//  ## ##    #    #        #    
//  #   #  #####  #      #####  
//
// ※ WiFi SPI 初始化界面已**统一**到 Host_Crtl：本页第 1 项现在直接调用 `Host_Crtl_WiFi_Select()`，
//   原来这里那份独立界面 + 独立候选表已删除（两份配置要各自维护，是历史包袱）。
//   要改 WiFi 界面 / 候选表请去 `Host_Crtl.c` / `Host_Crtl.h`。

//	#   #   ###   #####   ###   ####   
//  ## ##  #   #    #    #   #  #   #  
//  # # #  #   #    #    #   #  ####   
//  #   #  #   #    #    #   #  #  #   
//  #   #   ###     #     ###   #   #  
//
// [三级界面]电机调试
int Debug_Motor (void)
{
    // 电机驱动相关,为方便调用元素数量为5
    int16_t pwm[5] = {0};
    Motor_SET_Zero_ALL();

    // 电机调试界面光标 标志位
    // 完整的命名为Debug_Motor_flag，此处进行简化
    uint8_t Debug_M_f = 1;

    Debug_MOTOR_UI();
    ips200_show_string(0 ,32 , ">");
    ips200_printf(66 ,32 , "%d   ", pwm[1]);
    ips200_printf(66 ,48 , "%d   ", pwm[2]);
    ips200_printf(66 ,64 , "%d   ", pwm[3]);
    ips200_printf(66 ,80 , "%d   ", pwm[4]);

    // 重置计时参考值
    Time_Count1 = 0;
    Time_Count2 = 0;

    ENC_All_Clear();

    while(1)
    {
        // 存储确认键被按下时Debug_M_f的值的临时变量，默认为无效值0
        uint8_t Debug_M_f_temp = 0;
        // 上/下按键是否被按下过
        uint8_t key_pressed = 0;

        /* 按键处理 */
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            key_pressed = 1;
            Debug_M_f --;
            if (Debug_M_f < 1){Debug_M_f = 4;}
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            key_pressed = 1;
            Debug_M_f ++;
            if (Debug_M_f > 4){Debug_M_f = 1;}      
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            Debug_M_f_temp = Debug_M_f;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);

            Motor_SET_Zero_ALL();
            // 返回上一级界面
            return 0;
        }
            
            
        /* 参数设置 */
        if (1 <=Debug_M_f_temp && Debug_M_f_temp <= 4)
        {
            ips200_show_string(0 ,16 + 16*Debug_M_f_temp , "=");
            
            // 电机手动设置
            while(1)
            {
                /* 按键解析 */
                if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
                {
                    key_clear_state(KEY_UP);
                    pwm[Debug_M_f] += 100;
                    if (pwm[Debug_M_f] > 10000)pwm[Debug_M_f] = 10000;
                    Motor_Set(Debug_M_f, pwm[Debug_M_f]);
                    ips200_printf(66 ,16 + 16*Debug_M_f, "%d   ", pwm[Debug_M_f]);
                }
                else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
                {
                    key_clear_state(KEY_DOWN);
                    pwm[Debug_M_f] -= 100;
                    if (pwm[Debug_M_f] < -10000)pwm[Debug_M_f] = -10000;
                    Motor_Set(Debug_M_f, pwm[Debug_M_f]);
                    ips200_printf(66 ,16 + 16*Debug_M_f, "%d   ", pwm[Debug_M_f]);
                }
                else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM) || 
                         KEY_SHORT_PRESS == key_get_state(KEY_BACK))
                {
                    key_clear_state(KEY_CONFIRM);
                    key_clear_state(KEY_BACK);
					ips200_show_string(0 ,16 + 16*Debug_M_f_temp , ">");
                    
                    break;  // 退出修改模式
                }


                /* 显示更新 */
                if (Time_Count1 >= 10)// 10ms * 10 显示周期
                {
                    Time_Count1 = 0;
                    
                    ips200_printf(34 ,128, "%d   ", ENC_LR_CNT);
                    ips200_printf(146,128, "%d   ", ENC_RR_CNT);
                    ips200_printf(34 ,192, "%d     ", ENC_LR_SUM);
                    ips200_printf(34 ,208, "%d     ", ENC_RR_SUM);
                }
            }
        }
        

        /* 显示更新 */
        if (Time_Count1 >= 10)// 10ms * 10 显示周期
        {
            Time_Count1 = 0;
            
            ips200_printf(34 ,128, "%d   ", ENC_LR_CNT);
            ips200_printf(146,128, "%d   ", ENC_RR_CNT);
            ips200_printf(34 ,192, "%d     ", ENC_LR_SUM);
            ips200_printf(34 ,208, "%d     ", ENC_RR_SUM);
        }


        /* 光标更新 */
        if (key_pressed)
        {
            // 清理光标
            ips200_show_string(0 ,32 , " ");
            ips200_show_string(0 ,48 , " ");
            ips200_show_string(0 ,64 , " ");
            ips200_show_string(0 ,80 , " ");
            // 显示光标
            ips200_show_string(0 ,16 + 16*Debug_M_f  , ">");
        }
    }
}

//	#   #   ###   #####   ###   ####          ####   #####  ####   
//  ## ##  #   #    #    #   #  #   #         #   #    #    #   #  
//  # # #  #   #    #    #   #  ####    ###   ####     #    #   #  
//  #   #  #   #    #    #   #  #  #          #        #    #   #  
//  #   #   ###     #     ###   #   #         #      #####  ####   
//
// [三级界面]电机调试
int Debug_Motor_PID (void)
{
    // 目标值相关，为方便调用元素数量为3（只用 [1] [2]）
    // ★ 单位：0.01 rps（与 Motor_Crtl 同口径），写入 target 时 ×0.01
    int16_t enc_tar[3] = {0};
    // 重置闭环状态
    Motor_Crtl_Reset();
    // 电机速度重置
    Motor_SET_Zero_ALL();

    Motor_Crtl_Enable = 1;

	Debug_Motor_PID_UI();
	ips200_show_string(0 ,32 , ">");
    ips200_printf(66 ,32 , "%d  ", enc_tar[1]);
    ips200_printf(66 ,48 , "%d  ", enc_tar[2]);
    
    // 电机调试界面光标 标志位
    // 正常的命名为Debug_Motor_PID_flag，此处进行简化
    uint8_t Debug_M_P_f = 1;

    // 参考计时值重置
    Time_Count1 = 0;
    Time_Count2 = 0;

    ENC_All_Clear();

    while(1)
    {

        // 存储确认键被按下时Debug_M_f的值的临时变量，默认为无效值0
        uint8_t Debug_M_P_f_temp = 0;
        // 上/下按键是否被按下过
        uint8_t key_pressed = 0;

        Motor_Crtl_HB_Feed();       // 心跳：MOTOR-PID 主页循环还活着

        /* 按键处理 */
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            key_pressed = 1;
            Debug_M_P_f --;
            if (Debug_M_P_f < 1){Debug_M_P_f = 2;}
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            key_pressed = 1;
            Debug_M_P_f ++;
            if (Debug_M_P_f > 2){Debug_M_P_f = 1;}      
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            Debug_M_P_f_temp = Debug_M_P_f;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);

            Motor_Crtl_Enable = 0;
            // 重置闭环状态
            Motor_Crtl_Reset();
            // 电机速度重置
            Motor_SET_Zero_ALL();
            // 返回上一级界面
            return 0;
        }

        
        /* 参数设置 */
        if (1 <=Debug_M_P_f_temp && Debug_M_P_f_temp <= 2)
        {
            ips200_show_string(0 ,16 + 16*Debug_M_P_f_temp , "=");
            
            // 电机手动设置
            while(1)
            {
                Motor_Crtl_HB_Feed();   // 心跳：目标编辑循环还活着

                /* 按键解析 */
                if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
                {
                    key_clear_state(KEY_UP);
                    enc_tar[Debug_M_P_f] += 2;      // 步进 2 = 0.02 rps
                    if (enc_tar[Debug_M_P_f] > 300)enc_tar[Debug_M_P_f] = 300;   // 上限 3.00 rps
                    Motor_LR_Crtl.target = enc_tar[1] * 0.01f;
                    Motor_RR_Crtl.target = enc_tar[2] * 0.01f;
                    ips200_printf(66 ,16 + 16*Debug_M_P_f, "%d  ", enc_tar[Debug_M_P_f]);
                }
                else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
                {
                    key_clear_state(KEY_DOWN);
                    enc_tar[Debug_M_P_f] -= 2;      // 步进 2 = 0.02 rps
                    if (enc_tar[Debug_M_P_f] < -300)enc_tar[Debug_M_P_f] = -300; // 下限 -3.00 rps
                    Motor_LR_Crtl.target = enc_tar[1] * 0.01f;
                    Motor_RR_Crtl.target = enc_tar[2] * 0.01f;
                    ips200_printf(66 ,16 + 16*Debug_M_P_f, "%d  ", enc_tar[Debug_M_P_f]);
                }
                else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM) || 
                        KEY_SHORT_PRESS == key_get_state(KEY_BACK))
                {
                    key_clear_state(KEY_CONFIRM);
                    key_clear_state(KEY_BACK);
                    ips200_show_string(0 ,16 + 16*Debug_M_P_f_temp , ">");
                    
                    break;  // 退出修改模式
                }


                /* 数据显示 */
                if (Time_Count1 >= 5)// 10ms * 5 周期
                {
                    Time_Count1 = 0;

                    ips200_printf(154,32, "%d    ", (int16_t)(Motor_LR_Crtl.actual * 100.0f));   // 0.01 rps
                    ips200_printf(154,48, "%d    ", (int16_t)(Motor_RR_Crtl.actual * 100.0f));
                    ips200_printf(66 ,112, "%d    ", (int16_t)(Motor_LR_Crtl.out * 10000.0f));   // 等效 duty
                    ips200_printf(66 ,128, "%d    ", (int16_t)(Motor_RR_Crtl.out * 10000.0f));
                    ips200_printf(58 ,80 , "%d    ", (int)Motor_Crtl_HB_Age_Ms());
                    ips200_printf(66 ,96 , "%d ", Motor_LR_Crtl.enc_fault);
                    ips200_printf(114,96 , "%d ", Motor_RR_Crtl.enc_fault);
                }

                if (Time_Count2 >= 1)// 10ms * 1 周期
                {
                    Time_Count2 = 0;

                    if(wifi_spi_inited) 
                    {
                        char buf[64];
                        sprintf(buf, "%d,%d,%d\n", (int16_t)(Motor_RR_Crtl.actual * 100.0f), (int16_t)(Motor_RR_Crtl.target * 100.0f), (int16_t)(Motor_RR_Crtl.out * 10000.0f));
                        wifi_spi_send_buffer((uint8_t *)buf, (uint32)strlen(buf));
                    }
                }


            }
        }
        

        /* 数据显示 */
        if (Time_Count1 >= 5)// 10ms * 5 周期
        {
            Time_Count1 = 0;

            ips200_printf(154,32, "%d    ", (int16_t)(Motor_LR_Crtl.actual * 100.0f));   // 0.01 rps
            ips200_printf(154,48, "%d    ", (int16_t)(Motor_RR_Crtl.actual * 100.0f));
            ips200_printf(66 ,112, "%d    ", (int16_t)(Motor_LR_Crtl.out * 10000.0f));   // 等效 duty
            ips200_printf(66 ,128, "%d    ", (int16_t)(Motor_RR_Crtl.out * 10000.0f));
            ips200_printf(58 ,80 , "%d    ", (int)Motor_Crtl_HB_Age_Ms());
            ips200_printf(66 ,96 , "%d ", Motor_LR_Crtl.enc_fault);
            ips200_printf(114,96 , "%d ", Motor_RR_Crtl.enc_fault);
        }

        if (Time_Count2 >= 1)// 10ms * 1 周期
        {
            Time_Count2 = 0;

            if(wifi_spi_inited) 
            {
                char buf[64];
                sprintf(buf, "%d,%d,%d\n", (int16_t)(Motor_RR_Crtl.actual * 100.0f), (int16_t)(Motor_RR_Crtl.target * 100.0f), (int16_t)(Motor_RR_Crtl.out * 10000.0f));
                wifi_spi_send_buffer((uint8_t *)buf, (uint32)strlen(buf));
            }
        }
        
        
        /* 光标更新 */
        if (key_pressed)
        {
            // 清理光标
            ips200_show_string(0 ,32 , " ");
            ips200_show_string(0 ,48 , " ");
            ips200_show_string(0 ,64 , " ");
            ips200_show_string(0 ,80 , " ");
            // 显示光标
            ips200_show_string(0 ,16 + 16*Debug_M_P_f  , ">");
        }
    }
}

//  #####  #   #  #   #  
//    #    ## ##  #   #  
//    #    # # #  #   #  
//    #    #   #  #   #  
//  #####  #   #   ###   
//
// [三级界面]IMU模块
int Debug_IMU (void)
{
    Debug_IMU_UI();
    ips200_show_string(0 ,96 , ">");

    // IMU调试界面光标 标志位
    uint8_t Debug_IMU_f = 1;
    // ★ 2026-10-19：`gyro_cal_flag` 那套"发申请 -> 等 CM7_1 回状态"的三段式已删除。
    //   解算在本核，标定是同步调用，不再需要跨核状态机。

    // 参考计时值重置
    Time_Count1 = 0;
    Time_Count2 = 0;

    while(1)
    {
        // 存储确认键被按下时Debug_IMU_f的值的临时变量，默认为无效值0
        uint8_t Debug_IMU_f_temp = 0;
        // 上/下按键是否被按下过
        uint8_t key_pressed = 0;

        /* 按键处理 */
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            key_pressed = 1;
            Debug_IMU_f--;
            if(Debug_IMU_f < 1){Debug_IMU_f = 2;}
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            key_pressed = 1;
            Debug_IMU_f++;
            if(Debug_IMU_f > 2){Debug_IMU_f = 1;}
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);

            Debug_IMU_f_temp = Debug_IMU_f;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);

            // 返回上一级界面
            return 0;
        }


        /* 触发命令（★ 2026-10-19：解算在本核，直接调用，不再走共享内存 -> CM7_1） */
        if (Debug_IMU_f_temp == 1)
        {
            /* 陀螺零偏重标：**阻塞约 5 秒**（1000 拍 × 5ms），失败内部自动重试，最多 25 秒。
               ★ 车必须静止；一旦单轴极差超限就判"被碰过"并重来。
               ★ 期间 ins_imu.c 的 s_calib_busy 会挡住 5ms 中断里的 ins_imu_update()，
                 两者不会同时访问 SPI_2。 */
            ips200_show_string(130 ,96 , "WAIT");
            if (0 == ins_imu_calib_gyro()) { ips200_show_string(130 ,96 , "Done"); }
            else                           { ips200_show_string(130 ,96 , "Fail"); }
        }
        else if (Debug_IMU_f_temp == 2)
        {
            ins_imu_zero_yaw();                  // 累积航向归零（不动四元数姿态）
            ips200_show_string(130 ,96 , "Done");
        }


        /* 显示 yaw（★ 直接取本核 ins_imu 的输出，不再经 Yaw_Receive 定点中转） */
        if (Time_Count1 >= 10)  // 10ms * 10显示周期
        {
            Time_Count1 = 0;
            float yaw_now = ins_imu_get_yaw();
            int16 yaw_deg = (int16)yaw_now;                             // 截断取整（与原整数除法口径一致）
            int16 yaw_fra = (int16)((yaw_now - (float)yaw_deg) * 100.0f);
            if(yaw_fra < 0) yaw_fra = -yaw_fra;
            ips200_printf(58, 48, "%d.%02d ", yaw_deg, yaw_fra);
        }


        /* 光标更新 */
        if (key_pressed)
        {
            ips200_show_string(0 ,96 , " ");
            ips200_show_string(0 ,112, " ");
            ips200_show_string(0 ,80 + 16*Debug_IMU_f , ">");
        }
    }
}

//  #   #   ###    ###        #####  #   #  #####  
//  ## ##  #   #  #           #      ##  #  #      
//  # # #  #####  #   #  ###  #####  # # #  #      
//  #   #  #   #  #   #       #      #  ##  #      
//  #   #  #   #   ####       #####  #   #  #####  
//
// [三级界面]磁编码器调试
int Debug_MENC15A(void)
{
    Debug_MENC15A_UI();

    // 参考计时值重置
    Time_Count1 = 0;
    Time_Count2 = 0;

    int32_t menc15a_sum = 0;

    // 上次积分时刻（用于按真实时间间隔积分，避免循环体耗时被算进周期）
    uint32 menc15a_last_ms = Sys_Tick_Ms;

    while(1)
    {
        if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);
            // 返回上一级界面
            return 0;
        }

        /* 数据读取 + WiFi 发送（约 20ms 周期，按真实时间间隔积分） */
        uint32 now_ms  = Sys_Tick_Ms;
        uint16 dt_real = (uint16)(now_ms - menc15a_last_ms);
        if (dt_real >= 20)
        {
            menc15a_last_ms = now_ms;

            // 单次积分时长限幅保护：
            // 若循环被意外阻塞（长时间 LCD 刷新、WiFi 卡顿等），间隔会被拉得很长，
            // 而积分是"当前速度 × 间隔"，等于把整段时长都按这一个瞬时速度计入，误差很大。
            // 故单次最多按 50ms 计，把单次误差限制住。
            uint16 dt_ms = (dt_real > 50) ? 50 : dt_real;

            // 数据全部由 5ms 中断统一采集（见 cm7_0_isr.c），这里只读快照，不再直接调驱动
            // 速度积分：角速度(rad/s) × 真实间隔(ms) = 毫弧度(mrad)，一圈 = 2π×1000 ≈ 6283 mrad
            // 注意：必须用真实 dt，不能用固定 20ms —— 循环体（SPI 读 + WiFi 发送）耗时会让实际周期变长
            int16 spd = ENC_MAG_SPD; 
            // 中断里已做过"偶发错读原位重读"；这里再兜底一道：仍异常就丢弃
            if (spd > 15000 || spd < -15000) spd = 0;
            // ±10 rad/s 为死区，速度原始值的噪声基本被去除（折算到输出轴仅 0.05 rad/s，不影响真实转向）
            if (spd >= -10 && spd <= 10) spd = 0; 
            menc15a_sum += (int32_t)spd * (int32_t)dt_ms;

            // AREV 圈数原始值（编码器轴每转一整圈 ±1，9 位有符号）
            int16 arev_now = ENC_MAG_REV;

            // 输出轴角度（编码器在电机轴侧，三级齿轮减速比 (67/12)×(50/8)×(44/8) = 191.927）
            // menc15a_sum 为编码器轴毫弧度(mrad)，换算到输出轴毫度(0.001°)：
            // out_mdeg = sum × 360000 / (191.927 × 2π×1000) ≈ sum × 298529 / 1000000
            int32_t out_mdeg = (int32_t)((int64_t)menc15a_sum * 298529 / 1000000);

            if(wifi_spi_inited)
            {
                char buf[80];
                sprintf(buf, "%d,%d,%d,%d,%d,%d,%d\n", (int)ENC_MAG_ANG, (int)ENC_MAG_OFF, (int)ENC_MAG_SPD, menc15a_sum, out_mdeg, (int)arev_now, (int)dt_ms);
                wifi_spi_send_buffer((uint8_t *)buf, (uint32)strlen(buf));
            }
        }

        /* 屏幕显示更新（100ms 周期） */
        if (Time_Count1 >= 10)  // 10ms * 10 显示周期
        {
            Time_Count1 = 0;

            ips200_printf(42 ,32 , "%d    ", (int)ENC_MAG_ANG);
            ips200_printf(130,32 , "%d    ", (int)ENC_MAG_OFF);
            ips200_printf(42 ,48 , "%d    ", (int)ENC_MAG_SPD);
        }
    }
}


//   ###   ####   #####       #####  #   #  #####  
//  #   #  #   #  #           #      ##  #  #      
//  #####  ####   #####  ###  #####  # # #  #      
//  #   #  #   #      #       #      #  ##  #      
//  #   #  ####   #####       #####  #   #  #####  
//
// [三级界面]绝对值角度编码器调试
int Debug_ABS_ENCODER(void)
{
    Debug_ABS_ENCODER_UI();

    // 参考计时值重置
    Time_Count1 = 0;
    Time_Count2 = 0;

    int16   abs_enc_ang = 0;            // 当前绝对角度 原始值 0 ~ 4095
    int16   abs_enc_off = 0;            // 相对上一次读取位置的偏移（超过半圈按最短路径）
    int32_t abs_enc_deg = 0;            // 换算后的角度 单位 0.1°（0 ~ 3600）

    // 数据由 5ms 中断统一采集（见 cm7_0_isr.c），这里只读快照，不再直接调驱动

    while(1)
    {
        if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);
            // 返回上一级界面
            return 0;
        }

        /* 数据读取 + WiFi 发送（约 20ms 周期） */
        if (Time_Count2 >= 2)   // 10ms * 2
        {
            Time_Count2 = 0;

            // 读快照（5ms 中断已刷新）
            abs_enc_ang = ENC_ABS_ANG;
            abs_enc_off = ENC_ABS_OFF;
            // 换算成 0.1° 单位：4096 计数 = 360.0° → ang × 3600 / 4096
            abs_enc_deg = (int32_t)abs_enc_ang * 3600 / 4096;

            if(wifi_spi_inited)
            {
                char buf[48];
                sprintf(buf, "%d,%d,%d\n", (int)abs_enc_ang, (int)abs_enc_off, (int)abs_enc_deg);
                wifi_spi_send_buffer((uint8_t *)buf, (uint32)strlen(buf));
            }
        }

        /* 屏幕显示更新（100ms 周期） */
        if (Time_Count1 >= 10)  // 10ms * 10 显示周期
        {
            Time_Count1 = 0;

            ips200_printf(42 ,32 , "%d    ", (int)abs_enc_ang);
            ips200_printf(130,32 , "%d    ", (int)abs_enc_off);
            ips200_printf(42 ,48 , "%d    ", (int)abs_enc_deg);
        }
    }
}

// 屏幕输出：带符号的角度（单位 0.1°）
// 分符号输出，避免依赖 %s / %f 的格式支持
static void Debug_SERVO_ShowDeg (uint16 x, uint16 y, int32_t deg10)
{
    int32_t v = (deg10 >= 0) ? deg10 : -deg10;

    if (deg10 < 0)
    {
        ips200_printf(x, y, "-%d.%d    ", (int)(v / 10), (int)(v % 10));
    }
    else
    {
        ips200_printf(x, y, "+%d.%d    ", (int)(v / 10), (int)(v % 10));
    }
}

//  #####  #####  ####   #   #   ###   
//  #      #      #   #  #   #  #   #  
//  #####  #####  ####   #   #  #   #  
//      #  #      #  #    # #   #   #  
//  #####  #####  #   #    #     ###   
//
// [三级界面]舵机闭环调试界面
int Debug_SERVO(void)
{
    Debug_SERVO_UI();

    // 参考计时值重置
    Time_Count1 = 0;
    Time_Count2 = 0;

    int32_t servo_deg10 = 0;            // 方向盘角度 单位 0.1°
    int32_t servo_tar10 = 0;            // 目标角度   单位 0.1°
    int32_t servo_err10 = 0;            // 误差       单位 0.1°

    // 进入本页：目标清 0（临时量），使能一并清（Servo_Crtl_Reset 内部已置 Enable=0）
    // 此时输出为 0，进页面本身不会产生任何动作
    Servo_Crtl_Reset();

    while(1)
    {
        /* 按键处理 */
        if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);
            Servo_Crtl_Enable = 0;      // 退出固定置 0（输出由 Tick 里的门控清零）
            // 返回上一级界面
            return 0;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            // 步进上限直接取【软】限位：本页设的目标永远落在控制器真正能执行的范围里，
            // 于是 Servo_Target_Deg 与施加目标一致，屏幕/上发不会被"夹紧前的命令"污染
            Servo_Target_Deg += 3.0f;
            if (Servo_Target_Deg > SERVO_SOFT_LIMIT_DEG) { Servo_Target_Deg = SERVO_SOFT_LIMIT_DEG; }
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            Servo_Target_Deg -= 3.0f;
            if (Servo_Target_Deg < -SERVO_SOFT_LIMIT_DEG) { Servo_Target_Deg = -SERVO_SOFT_LIMIT_DEG; }
        }
        // 确认键直接绑定调试界面的使能
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            Servo_Crtl_Enable = (0 == Servo_Crtl_Enable) ? 1 : 0;   // 切换闭环使能
        }

        /* 数据读取 + WiFi 发送（约 20ms 周期） */
        if (Time_Count2 >= 2)   // 10ms * 2
        {
            Time_Count2 = 0;

            // 读快照（5ms 中断已刷新）
            servo_deg10 = (int32_t)(Servo_Angle_Deg * 10.0f);
            servo_tar10 = (int32_t)(Servo_Target_Deg * 10.0f);
            servo_err10 = (int32_t)(Servo_Err_Deg * 10.0f);

            if(wifi_spi_inited)
            {
                char buf[48];
                // 第三列用施加目标(Servo_Target_Applied_Deg，软限位夹紧后)，而不是屏幕上的 raw
                // servo_tar10：否则命令超软限位时，外部拿 目标-实际 反推会得到偏大的假误差
                sprintf(buf, "%d,%d,%d\n", (int)Servo_Ang_Raw, (int)servo_deg10,
                        (int)(int32_t)(Servo_Target_Applied_Deg * 10.0f));
                wifi_spi_send_buffer((uint8_t *)buf, (uint32)strlen(buf));
            }
        }

        /* 屏幕显示更新（100ms 周期） */
        if (Time_Count1 >= 10)  // 10ms * 10 显示周期
        {
            Time_Count1 = 0;

            ips200_printf(42 ,32 , "%d    ", (int)Servo_Ang_Raw);
            Debug_SERVO_ShowDeg(130, 32, servo_deg10);
            Debug_SERVO_ShowDeg(42 , 48, servo_tar10);
            Debug_SERVO_ShowDeg(130, 48, servo_err10);
            ips200_printf(66 ,64 , "%d ", (int)Servo_Crtl_Enable);
            ips200_printf(130,64 , "%d    ", (int)Servo_Out);
            ips200_printf(42 ,80 , "%d ", (int)Servo_Enc_Fault);
        }
    }
}

//  
//  
//  
//  
//  
//
// [三级界面]UART 接收机调试 （对应HotRc遥控器）
int Debug_UART_RC(void)
{
    Debug_UART_RC_UI();

    // 参考计时值重置
    Time_Count1 = 0;
    Time_Count2 = 0;

    uint16 rc_ch[6]  = {0};             // CH1~CH6 快照，仅用于本调试界面的显示
    uint8  rc_state  = 0;               // 遥控器状态 1=正常 0=失控
    uint32 rc_frames = 0;               // 累计收到的完整帧数

    while(1)
    {
        if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);
            // 返回上一级界面
            return 0;
        }

        /* 数据读取 + WiFi 发送（约 20ms 周期） */
        if (Time_Count2 >= 2)   // 10ms * 2
        {
            Time_Count2 = 0;

            // 读快照（10ms 中断里解析好的结果）
            rc_ch[0] = uart_receiver.channel[0];
            rc_ch[1] = uart_receiver.channel[1];
            rc_ch[2] = uart_receiver.channel[2];
            rc_ch[3] = uart_receiver.channel[3];
            rc_ch[4] = uart_receiver.channel[4];
            rc_ch[5] = uart_receiver.channel[5];
            rc_state = uart_receiver.state;

            // 收到一帧就把标志清掉并累加：这样 FRAMES 能反映"链路是否在持续收帧"
            if (uart_receiver.finsh_flag)
            {
                uart_receiver.finsh_flag = 0;
                rc_frames ++;
            }

            if(wifi_spi_inited)
            {
                char buf[64];
                sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d\n",
                        (int)rc_ch[0], (int)rc_ch[1], (int)rc_ch[2],
                        (int)rc_ch[3], (int)rc_ch[4], (int)rc_ch[5],
                        (int)rc_state, (int)rc_frames);
                wifi_spi_send_buffer((uint8_t *)buf, (uint32)strlen(buf));
            }
        }

        /* 屏幕显示更新（100ms 周期） */
        if (Time_Count1 >= 10)  // 10ms * 10 显示周期
        {
            Time_Count1 = 0;

            ips200_printf(42 ,32 , "%d    ", (int)rc_ch[0]);
            ips200_printf(42 ,48 , "%d    ", (int)rc_ch[1]);
            ips200_printf(42 ,64 , "%d    ", (int)rc_ch[2]);
            ips200_printf(42 ,80 , "%d    ", (int)rc_ch[3]);
            ips200_printf(42 ,96 , "%d    ", (int)rc_ch[4]);
            ips200_printf(42 ,112, "%d    ", (int)rc_ch[5]);
            ips200_printf(34 ,144, "%d ",   (int)rc_state);
            ips200_printf(98 ,144, "%d    ", (int)rc_frames);
        }
    }
}

/**********************************************************/
/*----------------------------------------[E] 调试逻辑 [E]*/
/**********************************************************/
