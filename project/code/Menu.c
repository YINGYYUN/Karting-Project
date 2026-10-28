/*******************************************************************************
主菜单
*******************************************************************************/


#include "zf_common_headfile.h"

//----------------------------------------------------------
// 函数简介     外设初始化
// 使用示例     Peripheral_Init();
// 备注信息     只在"main.c"中调用一次
//----------------------------------------------------------
void Peripheral_Init(void)
{
    // IPS200 初始化 (SPI 接口串口两寸屏)
	ips200_init(IPS200_TYPE_SPI);
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);       // 字，背景
    ips200_clear();                                     // 清屏为黑色
	
	// 按键初始化(10ms扫描周期)
    key_init(10);

    Motor_init();
    Motor_Ctrl_Init();          // 后轮电机速度闭环变量重置（上电默认不使能）

    ENC_MAG_Init();             // 磁编码器初始化（硬件 SPI3/SCB6, P3.0/3.1/3.2/3.3）
    absolute_encoder_init();    // 绝对值角度编码器初始化（硬件 SPI4/SCB5, P7.0/7.1/7.2, CS P7.3）
    Servo_Ctrl_Init();          // 舵机位置环变量重置（上电默认不使能）

    uart_receiver_init();       // uart接收机初始化(P14.0 P14.1)

    Param_Init();

    Route_Ctrl_Init();          // 路表：读槽索引 + 载入 active 槽（阻塞，必须在 PIT 打开前）

    route_link_init();          // 串口灌表：清收发状态 + 放开日志门闩（收字节在 uart0_isr，
                                //   解析/写盘在 route_link_tick()，调用点在参数页的循环里）

    Host_Ctrl_Init();           // 上位机对接：只重置链路状态（不连 WiFi，避免卡住上电）
                                // 调用纪律（静默 / 链路来源）见 Host_Ctrl.h 文件头
}


/**********************************************************/
/*[S] 界面样式 [S]----------------------------------------*/
/**********************************************************/

// [一级界面]主菜单界面
void Menu_UI(void)
{
    ips200_show_string(8  ,0  , "[Menu]");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "Process");
    ips200_show_string(10 ,48 , "Debug");
    ips200_show_string(10 ,64 , "Param");
    ips200_show_string(10 ,80 , "Remote_Ctrl");
    ips200_show_string(10 ,96 , "Host_Ctrl");
    ips200_show_string(10 ,112, "Route_Ctrl");
}
/**********************************************************/
/*----------------------------------------[E] 界面样式 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 菜单逻辑 [S]----------------------------------------*/
/**********************************************************/

// 菜单选项 标志位
uint8_t menu_flag = 1;

void Menu_Show(void)
{
    ips200_clear();
    Menu_UI();
    ips200_show_string(0  ,32 , ">");

    while(1)
    {
        // 存储确认键被按下时menuflag的值的临时变量，默认为无效值0
		uint8_t menu_flag_temp = 0;
		// 上/下按键是否被按下过
		uint8_t key_pressed = 0;


        /* 按键处理 */
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            key_pressed = 1;
            menu_flag --;
            if (menu_flag < 1)menu_flag = 6;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN); 
            key_pressed = 1;
            menu_flag ++;
            if (menu_flag > 6)menu_flag = 1;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            menu_flag_temp = menu_flag;
        }
        key_clear_state(KEY_BACK); // 仅消费标志位


        /* 模式跳转 */
        if (menu_flag_temp == 1)
        {
            ips200_clear();

            // 从Process界面返回，显示主菜单界面
            ips200_clear();
            Menu_UI();
            key_pressed = 1;
        }
        else if (menu_flag_temp == 2)
        {
            ips200_clear();
            Debug_Page_Menu();
            
            // 从Debug界面返回，显示主菜单界面
            ips200_clear();
            Menu_UI();
            key_pressed = 1;
        }
        else if (menu_flag_temp == 3)
        {
            ips200_clear();
            Param_Page_Menu();
            
            // 从Param界面返回，显示主菜单界面
            ips200_clear();
            Menu_UI();
            key_pressed = 1;
        }
        else if (menu_flag_temp == 4)
        {
            ips200_clear();
            Remote_Ctrl_Page();

            // 从遥控界面返回，显示主菜单界面
            ips200_clear();
            Menu_UI();
            key_pressed = 1;
        }
        else if (menu_flag_temp == 5)
        {
            ips200_clear();
            Host_Ctrl_Page();

            // 从上位机界面返回，显示主菜单界面
            ips200_clear();
            Menu_UI();
            key_pressed = 1;
        }
        else if (menu_flag_temp == 6)
        {
            ips200_clear();
            Route_Ctrl_Page();

            // 从路表界面返回，显示主菜单界面
            ips200_clear();
            Menu_UI();
            key_pressed = 1;
        }


        /* 光标更新 */
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
			ips200_show_string(0  ,16 + 16*menu_flag , ">");
        }
    }
}
/**********************************************************/
/*----------------------------------------[E] 菜单逻辑 [E]*/
/**********************************************************/
