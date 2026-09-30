/*******************************************************************************
调试
*******************************************************************************/


#include "zf_common_headfile.h"
#include "IMU_Analysis.h"

// 从 CM7_1 收到的 yaw（定点，0.01°/LSB），定义在 main_cm7_0.c
extern volatile int16 Yaw_Receive;
extern volatile uint8 wifi_spi_inited;

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
    ips200_show_string(10 ,128, "01:###        02:###");
    ips200_show_string(10 ,144, "03:###        04:###");
    // 空行
    ips200_show_string(10 ,176, "SUM:");
    ips200_show_string(10 ,192, "01:###");
    ips200_show_string(10 ,208, "02:###");
    ips200_show_string(10 ,224, "03:###");
    ips200_show_string(10 ,240, "04:###");
}

// [三级界面]Motor_PID调试界面   
// 速度环
void Debug_Motor_PID_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-MOTOR-PID");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "TAR 01:###    ENC:###");
    ips200_show_string(10 ,48 , "TAR 02:###    ENC:###");
    ips200_show_string(10 ,64 , "TAR 03:###    ENC:###");
    ips200_show_string(10 ,80 , "TAR 04:###    ENC:###");
    // 空行
    ips200_show_string(10 ,112, "PWM 01:###");
    ips200_show_string(10 ,128, "PWM 02:###");
    ips200_show_string(10 ,144, "PWM 03:###");
    ips200_show_string(10 ,160, "PWM 04:###");
    // 空行
    ips200_show_string(10 ,192, "SUM:");
    ips200_show_string(10 ,208, "01:###");
    ips200_show_string(10 ,224, "02:###");
    ips200_show_string(10 ,240, "03:###");
    ips200_show_string(10 ,256, "04:###");
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
                    
                    ips200_printf(34 ,128, "%d   ", ENC_1_CNT);
                    ips200_printf(146,128, "%d   ", ENC_2_CNT);
                    ips200_printf(34 ,144, "%d   ", ENC_3_CNT);
                    ips200_printf(146,144, "%d   ", ENC_4_CNT);
                    ips200_printf(34 ,192, "%d     ", ENC_1_SUM);
                    ips200_printf(34 ,208, "%d     ", ENC_2_SUM);
                    ips200_printf(34 ,224, "%d     ", ENC_3_SUM);
                    ips200_printf(34 ,240, "%d     ", ENC_4_SUM);
                }
            }
        }
        

        /* 显示更新 */
        if (Time_Count1 >= 10)// 10ms * 10 显示周期
        {
            Time_Count1 = 0;
            
            ips200_printf(34 ,128, "%d   ", ENC_1_CNT);
            ips200_printf(146,128, "%d   ", ENC_2_CNT);
            ips200_printf(34 ,144, "%d   ", ENC_3_CNT);
            ips200_printf(146,144, "%d   ", ENC_4_CNT);
            ips200_printf(34 ,192, "%d     ", ENC_1_SUM);
            ips200_printf(34 ,208, "%d     ", ENC_2_SUM);
            ips200_printf(34 ,224, "%d     ", ENC_3_SUM);
            ips200_printf(34 ,240, "%d     ", ENC_4_SUM);
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
    // PID期望值相关,为方便调用元素数量为5
    int16_t enc_tar[5] = {0};
    // 重置PID中间量
    PID_ALL_Init();
    // 电机速度重置
    Motor_SET_Zero_ALL();

    Speed_PID_Crtl_Enable = 1;

	Debug_Motor_PID_UI();
	ips200_show_string(0 ,32 , ">");
    ips200_printf(66 ,32 , "%d  ", enc_tar[1]);
    ips200_printf(66 ,48 , "%d  ", enc_tar[2]);
    ips200_printf(66 ,64 , "%d  ", enc_tar[3]);
    ips200_printf(66 ,80 , "%d  ", enc_tar[4]);
    
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
            if (Debug_M_P_f < 1){Debug_M_P_f = 4;}
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            key_pressed = 1;
            Debug_M_P_f ++;
            if (Debug_M_P_f > 4){Debug_M_P_f = 1;}      
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            Debug_M_P_f_temp = Debug_M_P_f;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);

            Speed_PID_Crtl_Enable = 0;
            // 重置PID中间量
            PID_ALL_Init();
            // 电机速度重置
            Motor_SET_Zero_ALL();
            // 返回上一级界面
            return 0;
        }

        
        /* 参数设置 */
        if (1 <=Debug_M_P_f_temp && Debug_M_P_f_temp <= 4)
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
                    Motor_1_PID.Target = enc_tar[1];
                    Motor_2_PID.Target = enc_tar[2];
                    Motor_3_PID.Target = enc_tar[3];
                    Motor_4_PID.Target = enc_tar[4];
                    ips200_printf(66 ,16 + 16*Debug_M_P_f, "%d  ", enc_tar[Debug_M_P_f]);
                }
                else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
                {
                    key_clear_state(KEY_DOWN);
                    enc_tar[Debug_M_P_f] -= 20;
                    if (enc_tar[Debug_M_P_f] < -800)enc_tar[Debug_M_P_f] = -800;
                    Motor_1_PID.Target = enc_tar[1];
                    Motor_2_PID.Target = enc_tar[2];
                    Motor_3_PID.Target = enc_tar[3];
                    Motor_4_PID.Target = enc_tar[4];
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

                    ips200_printf(154,32, "%d    ", (int16_t)Motor_1_PID.Actual);
                    ips200_printf(154,48, "%d    ", (int16_t)Motor_2_PID.Actual);
                    ips200_printf(154,64, "%d    ", (int16_t)Motor_3_PID.Actual);
                    ips200_printf(154,80, "%d    ", (int16_t)Motor_4_PID.Actual);
                    // ips200_printf(66 ,112, "%d   ", (int16_t)Motor_1_PID.Out);
                    // ips200_printf(66 ,128, "%d   ", (int16_t)Motor_2_PID.Out);
                    // ips200_printf(66 ,144, "%d   ", (int16_t)Motor_3_PID.Out);
                    // ips200_printf(66 ,160, "%d   ", (int16_t)Motor_4_PID.Out);
                    // ips200_printf(34 ,208, "%d     ", ENC_1_SUM);
                    // ips200_printf(34 ,224, "%d     ", ENC_2_SUM);
                    // ips200_printf(34 ,240, "%d     ", ENC_3_SUM);
                    // ips200_printf(34 ,256, "%d     ", ENC_4_SUM);
                }

                if (Time_Count2 >= 1)// 10ms * 1 周期
                {
                    Time_Count2 = 0;

                    if(wifi_spi_inited) 
                    {
                        char buf[64];
                        sprintf(buf, "%d,%d,%d\n", (int16_t)Motor_1_PID.Actual, (int16_t)Motor_1_PID.Target, (int16_t)Motor_1_PID.Out);
                        wifi_spi_send_buffer((uint8_t *)buf, (uint32)strlen(buf));
                    }
                }


            }
        }
        

        /* 数据显示 */
        if (Time_Count1 >= 5)// 10ms * 5 周期
        {
            Time_Count1 = 0;

            ips200_printf(154,32, "%d    ", (int16_t)Motor_1_PID.Actual);
            ips200_printf(154,48, "%d    ", (int16_t)Motor_2_PID.Actual);
            ips200_printf(154,64, "%d    ", (int16_t)Motor_3_PID.Actual);
            ips200_printf(154,80, "%d    ", (int16_t)Motor_4_PID.Actual);
            // ips200_printf(66 ,112, "%d   ", (int16_t)Motor_1_PID.Out);
            // ips200_printf(66 ,128, "%d   ", (int16_t)Motor_2_PID.Out);
            // ips200_printf(66 ,144, "%d   ", (int16_t)Motor_3_PID.Out);
            // ips200_printf(66 ,160, "%d   ", (int16_t)Motor_4_PID.Out);
            // ips200_printf(34 ,208, "%d     ", ENC_1_SUM);
            // ips200_printf(34 ,224, "%d     ", ENC_2_SUM);
            // ips200_printf(34 ,240, "%d     ", ENC_3_SUM);
            // ips200_printf(34 ,256, "%d     ", ENC_4_SUM);
        }

        if (Time_Count2 >= 1)// 10ms * 1 周期
        {
            Time_Count2 = 0;

            if(wifi_spi_inited) 
            {
                char buf[64];
                sprintf(buf, "%d,%d,%d\n", (int16_t)Motor_1_PID.Actual, (int16_t)Motor_1_PID.Target, (int16_t)Motor_1_PID.Out);
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

/**********************************************************/
/*----------------------------------------[E] 调试逻辑 [E]*/
/**********************************************************/
