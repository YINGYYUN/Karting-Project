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

    // 电机接口 1 PID
    Menu_Item *folder_m1 = DynamicCreate_Menu_Folder(&head, "Motor_1_PID");
    DynamicCreate_Menu_LimitNumber(folder_m1, "KP", &MOTOR_1_KP, float_Box, 0, 70);
    DynamicCreate_Menu_LimitNumber(folder_m1, "KI", &MOTOR_1_KI, float_Box, 0, 70);
    DynamicCreate_Menu_LimitNumber(folder_m1, "KD", &MOTOR_1_KD, float_Box, 0, 70);

    // 电机接口 2 PID
    Menu_Item *folder_m2 = DynamicCreate_Menu_Folder(&head, "Motor_2_PID");
    DynamicCreate_Menu_LimitNumber(folder_m2, "KP", &MOTOR_2_KP, float_Box, 0, 70);
    DynamicCreate_Menu_LimitNumber(folder_m2, "KI", &MOTOR_2_KI, float_Box, 0, 70);
    DynamicCreate_Menu_LimitNumber(folder_m2, "KD", &MOTOR_2_KD, float_Box, 0, 70);

    // 电机接口 3 PID
    Menu_Item *folder_m3 = DynamicCreate_Menu_Folder(&head, "Motor_3_PID");
    DynamicCreate_Menu_LimitNumber(folder_m3, "KP", &MOTOR_3_KP, float_Box, 0, 70);
    DynamicCreate_Menu_LimitNumber(folder_m3, "KI", &MOTOR_3_KI, float_Box, 0, 70);
    DynamicCreate_Menu_LimitNumber(folder_m3, "KD", &MOTOR_3_KD, float_Box, 0, 70);

    // 电机接口 4 PID
    Menu_Item *folder_m4 = DynamicCreate_Menu_Folder(&head, "Motor_4_PID");
    DynamicCreate_Menu_LimitNumber(folder_m4, "KP", &MOTOR_4_KP, float_Box, 0, 70);
    DynamicCreate_Menu_LimitNumber(folder_m4, "KI", &MOTOR_4_KI, float_Box, 0, 70);
    DynamicCreate_Menu_LimitNumber(folder_m4, "KD", &MOTOR_4_KD, float_Box, 0, 70);

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
