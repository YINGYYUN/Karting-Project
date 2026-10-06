/*******************************************************************************
参数更改菜单（CYT4BB 适配：四键方案 + 4 电机 PID）
参数不在本文件定义，而在对应的模块文件中
参数缓存区(param_cache) → Flash 持久化, 通过 Param_Storage 宏访问
*******************************************************************************/


#include "zf_common_headfile.h"
#include "common_menu.h"
#include "Param_Storage.h"


// 构建参数页面
static void param_page_init(void)
{
    menu_reset();       // 清空旧菜单

    // 左后轮 速度闭环
    // 量程按"队友参数换算到本工程单位"后的量级放大：
    //   inv_K≈83、k_v≈88、k_z≈1249、u0≈330（cpr=1858 换算），另留标定余量
    Menu_Item *folder_lr = DynamicCreate_Menu_Folder(&head, "Motor_LR");
    DynamicCreate_Menu_LimitNumber(folder_lr, "invK", &MOTOR_LR_INV_K, float_Box, 0, 1000);
    DynamicCreate_Menu_LimitNumber(folder_lr, "u0",   &MOTOR_LR_U0,    float_Box, 0, 1000);
    DynamicCreate_Menu_LimitNumber(folder_lr, "kv",   &MOTOR_LR_KV,    float_Box, 0, 1000);
    DynamicCreate_Menu_LimitNumber(folder_lr, "kz",   &MOTOR_LR_KZ,    float_Box, 0, 10000);

    // 右后轮 速度闭环
    Menu_Item *folder_rr = DynamicCreate_Menu_Folder(&head, "Motor_RR");
    DynamicCreate_Menu_LimitNumber(folder_rr, "invK", &MOTOR_RR_INV_K, float_Box, 0, 1000);
    DynamicCreate_Menu_LimitNumber(folder_rr, "u0",   &MOTOR_RR_U0,    float_Box, 0, 1000);
    DynamicCreate_Menu_LimitNumber(folder_rr, "kv",   &MOTOR_RR_KV,    float_Box, 0, 1000);
    DynamicCreate_Menu_LimitNumber(folder_rr, "kz",   &MOTOR_RR_KZ,    float_Box, 0, 10000);

    // 舵机 转向位置环
    Menu_Item *folder_sv = DynamicCreate_Menu_Folder(&head, "Servo");
    DynamicCreate_Menu_LimitNumber(folder_sv, "kp", &SERVO_KP, float_Box, 0, 1000);
    DynamicCreate_Menu_LimitNumber(folder_sv, "u0", &SERVO_U0, float_Box, 0, 3000);

    key = head.first_son;   // 光标移到第一项
}


// 参数设置页面主入口
int Param_Page_Menu(void)
{
    param_page_init();          // 构建菜单
    menu_show_all();            // 清屏 + 刷新

    while(1)
    {
        // 短按: 上键
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            key_up_btn();
            menu_show();
        }
        // 短按: 下键
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            key_down_btn();
            menu_show();
        }
        // 短按: 确认键 → 进入文件夹 / 选中参数
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            key_enter_btn();
            menu_show();
        }
        // 短按: 返回键
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);

            // 根层 且 未选中 → 保存并退出
            if (key->father->father == NULL && key->select == false)
            {
                Param_Save();           // 持久化到 Flash
                Flash_SyncTo_Param();   // 同步到使用中的参数(比如PID)
                return 0;
            }
            else
            {
                // 选中状态 → 步进切换；子文件夹 → 回退上一级
                key_quit_btn();
                menu_show();
            }
        }
    }
}
