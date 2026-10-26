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
    Menu_Item *folder_lr = DynamicCreate_Menu_Folder(&head, "Motor_LR");
    DynamicCreate_Menu_LimitNumber(folder_lr, "invK", &MOTOR_LR_INV_K, float_04_Box, 0, 1);
    DynamicCreate_Menu_LimitNumber(folder_lr, "u0",   &MOTOR_LR_U0,    float_03_Box, 0, 0.5);
    DynamicCreate_Menu_LimitNumber(folder_lr, "kv",   &MOTOR_LR_KV,    float_03_Box, 0, 2);
    DynamicCreate_Menu_LimitNumber(folder_lr, "kz",   &MOTOR_LR_KZ,    float_03_Box, 0, 20);

    // 右后轮 速度闭环
    Menu_Item *folder_rr = DynamicCreate_Menu_Folder(&head, "Motor_RR");
    DynamicCreate_Menu_LimitNumber(folder_rr, "invK", &MOTOR_RR_INV_K, float_04_Box, 0, 1);
    DynamicCreate_Menu_LimitNumber(folder_rr, "u0",   &MOTOR_RR_U0,    float_03_Box, 0, 0.5);
    DynamicCreate_Menu_LimitNumber(folder_rr, "kv",   &MOTOR_RR_KV,    float_03_Box, 0, 2);
    DynamicCreate_Menu_LimitNumber(folder_rr, "kz",   &MOTOR_RR_KZ,    float_03_Box, 0, 20);

    // 舵机 转向位置环
    Menu_Item *folder_sv = DynamicCreate_Menu_Folder(&head, "Servo");
    DynamicCreate_Menu_LimitNumber(folder_sv, "kp", &SERVO_KP, float_01_Box, 0, 1000);
    DynamicCreate_Menu_LimitNumber(folder_sv, "u0", &SERVO_U0, float_01_Box, 0, 3000);
    DynamicCreate_Menu_LimitNumber(folder_sv, "ki", &SERVO_KI, float_01_Box, 0, 1000);

    // 路表
    // ★ 2026-10-24：`sm / L / ept / ve / alat / vmin / kh` 七项**从板载菜单移出**。
    //   它们仍然是参数（param_cache 里位置不变、Flash 照旧存、上位机滑条照旧能改），
    //   只是不在车上的参数页里显示了 —— 板上只留"跑之前一定会动"的这几项。
    //   ⚠ 不要为了"少几项"去改它们的 IDX：那样会错位整个参数表（存档与上位机都跟着废）。
    Menu_Item *folder_rt = DynamicCreate_Menu_Folder(&head, "Route");
    DynamicCreate_Menu_LimitNumber(folder_rt, "ds", &ROUTE_DS, float_03_Box,
                                   INS_ROUTE_DS_MIN, INS_ROUTE_DS_MAX);
    DynamicCreate_Menu_LimitNumber(folder_rt, "v",  &ROUTE_PB_V,  float_02_Box, PB_V_MIN,  PB_V_MAX);
    DynamicCreate_Menu_LimitNumber(folder_rt, "ld", &ROUTE_PB_LD, float_02_Box, PB_LD_MIN, PB_LD_MAX);
    // 倒车录 / 倒序回放（两项必须与"表的出身"一致，否则回放拒绝起跑并在屏幕显示原因）
    DynamicCreate_Menu_LimitNumber(folder_rt, "rrec", &ROUTE_REV_REC,  float_01_Box, 0, 1);            // 录制时 v 存 |v|
    DynamicCreate_Menu_LimitNumber(folder_rt, "rply", &ROUTE_REV_PLAY, float_01_Box, 0, 1);            // 起跑时倒点序

    // 遥测（顶层文件夹）：档案选择 0 = 全量 41 路 / 1 = 标定精简 18 路
    // ★ 切换后**无需重启**：Host_Crtl.c 每帧按 tel_sel 现算通道数与表，
    //   帧头 COUNT 随之变化，上位机按 COUNT 自动切档案。
    Menu_Item *folder_tel = DynamicCreate_Menu_Folder(&head, "Tele");
    DynamicCreate_Menu_LimitNumber(folder_tel, "sel", &TEL_SEL, float_01_Box, 0, 1);

    key = head.first_son;   // 光标移到第一项
}


// 参数页公共循环（进入前菜单已建好）
static int param_page_loop(void)
{
    menu_show_all();            // 清屏 + 刷新

    while(1)
    {
        route_link_tick();          // 串口灌表：收帧 + 推进分步写盘（全工程唯一调用点）
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


// 参数设置页面主入口
int Param_Page_Menu(void)
{
    param_page_init();
    return param_page_loop();
}
