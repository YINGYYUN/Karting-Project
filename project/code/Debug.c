/*******************************************************************************
调试
*******************************************************************************/


#include "zf_common_headfile.h"
#include "IMU_Analysis.h"

// 从 CM7_1 收到的 yaw（定点，0.01°/LSB），定义在 main_cm7_0.c
extern volatile int16 Yaw_Receive;
extern volatile uint8 wifi_spi_inited;
// 全局毫秒计时（定义在 cm7_0_isr.c），用于按真实时间积分
extern volatile uint32 Sys_Tick_Ms;

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
    ips200_show_string(10 ,96 , "MENC15A");
    ips200_show_string(10 ,112, "ABS-ENC");
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
    // 空行
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
    ips200_show_string(8  ,0  , "[DEBUG]-MENC15A");
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

/**********************************************************/
/*----------------------------------------[E] 界面样式 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 菜单逻辑 [S]----------------------------------------*/
/**********************************************************/

// 相关函数提前声明
int Debug_WiFi_SPI      (void);
int Debug_Motor         (void);
int Debug_Motor_PID     (void);
int Debug_IMU           (void);
int Debug_MENC15A       (void);
int Debug_ABS_ENCODER   (void);

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
            if (Debug_Page_flag < 1)Debug_Page_flag = 6;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN); 
            key_pressed = 1;
            Debug_Page_flag ++;
            if (Debug_Page_flag > 6)Debug_Page_flag = 1;
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
            Debug_WiFi_SPI();
            
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
// [三级界面]WiFi SPI初始化（手动触发 WiFi SPI初始化 + 连接）
int Debug_WiFi_SPI(void)
{
    ips200_show_string(8 ,0  , "[DEBUG]-WIFI-SPI");
    ips200_show_string(0 ,16 , "==============================");
    ips200_show_string(10,32 , "Press Confirm to Init");

    if (wifi_spi_inited)
    {
        ips200_show_string(10, 96, "Init done!");
    }
    

    while(1)
    {
        if(KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);

            // 触发 WiFi 初始化 + 连 UDP（目标：电脑 192.168.50.247:8086）
            uint8 r1 = wifi_spi_init("ASC_Circuit_IoT", "1145141919810");
            uint8 r2 = wifi_spi_socket_connect("UDP", "192.168.50.247", "8086", "6666");

            if(r1 == 0 && r2 == 0) wifi_spi_inited = 1;
            ips200_show_string(10, 64, r1 ? "Wifi fail" : "Wifi ok");
            ips200_show_string(10, 80, r2 ? "Sock fail" : "Sock ok");
            ips200_show_string(10, 96, "Init done!");
        }
        else if(KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);
            return 0;
        }
    }
}

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
                /* 按键解析 */
                if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
                {
                    key_clear_state(KEY_UP);
                    enc_tar[Debug_M_P_f] += 20;
                    if (enc_tar[Debug_M_P_f] > 800)enc_tar[Debug_M_P_f] = 800;
                    Motor_LR_Crtl.target = enc_tar[1];
                    Motor_RR_Crtl.target = enc_tar[2];
                    ips200_printf(66 ,16 + 16*Debug_M_P_f, "%d  ", enc_tar[Debug_M_P_f]);
                }
                else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
                {
                    key_clear_state(KEY_DOWN);
                    enc_tar[Debug_M_P_f] -= 20;
                    if (enc_tar[Debug_M_P_f] < -800)enc_tar[Debug_M_P_f] = -800;
                    Motor_LR_Crtl.target = enc_tar[1];
                    Motor_RR_Crtl.target = enc_tar[2];
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

                    ips200_printf(154,32, "%d    ", (int16_t)Motor_LR_Crtl.actual);
                    ips200_printf(154,48, "%d    ", (int16_t)Motor_RR_Crtl.actual);
                    // ips200_printf(66 ,112, "%d   ", (int16_t)Motor_LR_Crtl.out);
                    // ips200_printf(66 ,128, "%d   ", (int16_t)Motor_RR_Crtl.out);
                }

                if (Time_Count2 >= 1)// 10ms * 1 周期
                {
                    Time_Count2 = 0;

                    if(wifi_spi_inited) 
                    {
                        char buf[64];
                        sprintf(buf, "%d,%d,%d\n", (int16_t)Motor_LR_Crtl.actual, (int16_t)Motor_LR_Crtl.target, (int16_t)Motor_LR_Crtl.out);
                        wifi_spi_send_buffer((uint8_t *)buf, (uint32)strlen(buf));
                    }
                }


            }
        }
        

        /* 数据显示 */
        if (Time_Count1 >= 5)// 10ms * 5 周期
        {
            Time_Count1 = 0;

            ips200_printf(154,32, "%d    ", (int16_t)Motor_LR_Crtl.actual);
            ips200_printf(154,48, "%d    ", (int16_t)Motor_RR_Crtl.actual);
            // ips200_printf(66 ,112, "%d   ", (int16_t)Motor_LR_Crtl.out);
            // ips200_printf(66 ,128, "%d   ", (int16_t)Motor_RR_Crtl.out);
        }

        if (Time_Count2 >= 1)// 10ms * 1 周期
        {
            Time_Count2 = 0;

            if(wifi_spi_inited) 
            {
                char buf[64];
                sprintf(buf, "%d,%d,%d\n", (int16_t)Motor_LR_Crtl.actual, (int16_t)Motor_LR_Crtl.target, (int16_t)Motor_LR_Crtl.out);
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
    // IMU调试界面 陀螺仪校准状态 标志位
    // 直接用于推进菜单显示
    uint8_t gyro_cal_flag= 0;
    // 0 空闲
    // 1 发送校准申请
    // 2 校准正在进行
    // 3 确认校准完成

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


        /* 触发命令 */
        if (Debug_IMU_f_temp == 1 && SHARED_IMU_ADDR->cmd_gyro_calib == 0)
        {
            // 发送（陀螺仪零漂）校准指令
            SHARED_IMU_ADDR->cmd_gyro_calib = 1;
            SHARED_IMU_WRITE_SYNC();
            ips200_show_string(130 ,96 , "Send");

            // 发送校准申请
            gyro_cal_flag = 1;
        }
        else if (Debug_IMU_f_temp == 2 && gyro_cal_flag == 0)
        {
            SHARED_IMU_ADDR->cmd_reset = 1;   // 命令 CM7_1 Yaw 归零
            SHARED_IMU_WRITE_SYNC();
        }

        // 核心1 指示（陀螺仪零漂）校准开始
        if (gyro_cal_flag == 1 && SHARED_IMU_ADDR->cmd_gyro_calib == 2)
        {
            // 校准正在进行
            gyro_cal_flag = 2;
            ips200_show_string(130 ,96 , "ing~");
        }
        // 核心1 指示（陀螺仪零漂）校准完成
        else if (gyro_cal_flag == 2 && SHARED_IMU_ADDR->cmd_gyro_calib == 3)
        {
            // 确认校准完成
            gyro_cal_flag = 3;
            ips200_show_string(130 ,96 , "Done");

            // 状态机重置
            SHARED_IMU_ADDR->cmd_gyro_calib = 0;
            SHARED_IMU_WRITE_SYNC();
            gyro_cal_flag = 0;
        }


        /* 显示 yaw + 光流 */
        if (Time_Count1 >= 10)  // 10ms * 10显示周期
        {
            Time_Count1 = 0;
            int16 yaw_deg = Yaw_Receive / 100;
            int16 yaw_fra = Yaw_Receive % 100;
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

//  #####  #   #  #####   ###   #   #    #   #  
//    #    #   #    #    #   #  #   #    #   #  
//    #    # # #    #    #   #  #   #    #   #  
//    #    #   #    #    #   #  #   #    #   #  
//  #####  #   #  #####   ###   #   #     ###   
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

            // 数据全部由 10ms 中断统一采集（见 cm7_0_isr.c），这里只读快照，不再直接调驱动
            // 速度积分：角速度(rad/s) × 真实间隔(ms) = 毫弧度(mrad)，一圈 = 2π×1000 ≈ 6283 mrad
            // 注意：必须用真实 dt，不能用固定 20ms —— 循环体（SPI 读 + WiFi 发送）耗时会让实际周期变长
            int16 spd = ENC_MAG_SPD; 
            // 中断里已做过"偶发错读原位重读"；这里再兜底一道：仍异常就丢弃
            if (spd > 15000 || spd < -15000) spd = 0;
            // ±10 rad/s 为死区，速度原始值的噪声基本被去除（折算到输出轴仅 0.05 rad/s，不影响真实转向）
            if (spd >= -10 && spd <= 10) spd = 0; 
            menc15a_sum += (int32_t)spd * (int32_t)dt_ms;

            // AREV 圈数原始值（编码器轴每转一整圈 ±1，9 位有符号）
            // 需要转了几圈时，直接用首尾两个 arev_now 相减即可
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

            // ips200_printf(42 ,32 , "%d    ", (int)ENC_MAG_ANG);
            // ips200_printf(130,32 , "%d    ", (int)ENC_MAG_OFF);
            // ips200_printf(42 ,48 , "%d    ", (int)ENC_MAG_SPD);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// [三级界面]绝对值角度编码器调试
// 逐飞 360° 绝对式角度传感器（硬件 SPI4/SCB5, P7.0/7.1/7.2, CS P7.3）
// 该传感器是绝对式，上电即可读到当前角度，不需要积分、也不存在累积漂移
//-------------------------------------------------------------------------------------------------------------------
int Debug_ABS_ENCODER(void)
{
    Debug_ABS_ENCODER_UI();

    // 参考计时值重置
    Time_Count1 = 0;
    Time_Count2 = 0;

    int16   abs_enc_ang = 0;            // 当前绝对角度 原始值 0 ~ 4095
    int16   abs_enc_off = 0;            // 相对上一次读取位置的偏移（超过半圈按最短路径）
    int32_t abs_enc_deg = 0;            // 换算后的角度 单位 0.1°（0 ~ 3600）

    // 数据由 10ms 中断统一采集（见 cm7_0_isr.c），这里只读快照，不再直接调驱动

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

            // 读快照（10ms 中断已刷新）
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

/**********************************************************/
/*----------------------------------------[E] 调试逻辑 [E]*/
/**********************************************************/
