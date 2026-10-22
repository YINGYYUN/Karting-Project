/*******************************************************************************
路表页：模式（录制 / 回放）+ 状态机 + 遥控接管
录制是监听者，不参与驾驶权；CH6 = 1 时把驾驶权交给遥控。
*******************************************************************************/


#include "zf_common_headfile.h"


#define ROUTE_MIN_SAVE_POINTS   (20u)       // 少于这么多点不落盘

// 模式
#define ROUTE_MODE_REC          (0)
#define ROUTE_MODE_PLAY         (1)

// 状态
#define ROUTE_ST_IDLE           (0)         // 空闲
#define ROUTE_ST_RUN            (1)         // 启动中 / 运行中
#define ROUTE_ST_DONE           (2)         // 正常结束（录满 / 跑完）
#define ROUTE_ST_STOP           (3)         // 人工停止
#define ROUTE_ST_FAULT          (4)         // 异常


/**********************************************************/
/*[S] 内部状态 [S]----------------------------------------*/
/**********************************************************/

static uint8        route_mode  = ROUTE_MODE_REC;
static uint8        route_state = ROUTE_ST_IDLE;
static uint8        route_cursor = 3;       // 0 = MODE / 1 = SLOT / 2 = PARAM / 3 = RUN
static const char  *route_msg   = "";
static uint8        route_save_prev = 0;    // 上一圈是否在写盘（抓 1->0 沿）

/**********************************************************/
/*----------------------------------------[E] 内部状态 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 界面样式 [S]----------------------------------------*/
/**********************************************************/

static const char *Route_Mode_Text (void)
{
    return (ROUTE_MODE_PLAY == route_mode) ? "PLAY" : "REC ";
}

static const char *Route_State_Text (void)
{
    if (ROUTE_ST_RUN   == route_state) { return "RUN  "; }
    if (ROUTE_ST_DONE  == route_state) { return "DONE "; }
    if (ROUTE_ST_STOP  == route_state) { return "STOP "; }
    if (ROUTE_ST_FAULT == route_state) { return "FAULT"; }
    return "IDLE ";
}

static void Route_Crtl_UI (void)
{
    ips200_show_string(8  ,0  , "[ROUTE]");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "MODE :");
    ips200_show_string(10 ,48 , "SLOT :");
    ips200_show_string(10 ,64 , "PARAM:");
    ips200_show_string(66 ,64 , "OPEN");
    ips200_show_string(10 ,80 , "RUN  :");
    ips200_show_string(10 ,96 , "STATE:");
    ips200_show_string(10 ,112, "DIST :");
    ips200_show_string(10 ,128, "TBL  :");
    ips200_show_string(10 ,144, "SAV  :");
    ips200_show_string(10 ,160, "RC:0 ENG:0 K5:0 K6:0");
    ips200_show_string(10 ,176, "CTRL :");
    ips200_show_string(10 ,192, "ELAT :");
    ips200_show_string(10 ,208, "MSG  :");
    ips200_show_string(10 ,240, "CONFIRM: SELECT");
    ips200_show_string(10 ,256, "UP/DN  : MOVE");
    ips200_show_string(10 ,272, "BACK   : EXIT");
}

// 界面缩写说明：
//   MODE   模式：REC 录制 / PLAY 回放
//   SLOT   Flash 槽号 0~4；选中即载入，录制固定写槽 0
//   PARAM  路表参数页入口（当前只有 ds：录制采样点距）
//   RUN    光标停在这行时，CONFIRM = 启动/停止
//   STATE  状态：IDLE 空闲 / RUN 运行 / DONE 正常结束 / STOP 人工停止 / FAULT 异常
//   DIST   当前表的里程，cm
//   TBL    RAM 里的路表点数
//   SAV    上次写盘结果：---- 未写 / OK / FAIL / EMPTY / BUSY nn%
//   RC     接收机在线 0/1
//   ENG    遥控状态机：0 OFF / 1 STANDBY / 2 ON / 3 FAILSAFE / 4 ESTOP
//   K5     CH5 急停
//   K6     CH6 把驾驶权交给遥控
//   CTRL   当前驾驶权：STANDBY / REMOTE / RC LOST / STOP
//   ELAT   回放横向误差 cm（正 = 车在参考线右侧）
//   MSG    状态机给出的结果或异常原因

/**********************************************************/
/*----------------------------------------[E] 界面样式 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 接口实现 [S]----------------------------------------*/
/**********************************************************/

//--------------------------------------
// 函数简介     空闲态：舵机回中，后轮自由轮
// 备注信息     后轮不使能，方便手推；不经过 Remote_Crtl_Apply()，避免它按 RC_Online 锁 FAILSAFE
//--------------------------------------
static void Route_Drive_Idle (void)
{
    Servo_Crtl_Enable    = 1;
    Servo_Target_Deg     = 0.0f;
    Motor_Crtl_Enable    = 0;
    Motor_LR_Crtl.target = 0.0f;
    Motor_RR_Crtl.target = 0.0f;
    Head_Crtl_Set_Enable(0);
}

//--------------------------------------
// 函数简介     停止录制并按点数决定是否落盘
//--------------------------------------
static void Route_Rec_Stop (void)
{
    ins_route_stop();

    if (ins_route_get_count() >= (uint16)ROUTE_MIN_SAVE_POINTS)
    {
        ins_route_flash_save_slot(FM_SLOT_MANUAL);
        route_state = ROUTE_ST_STOP;
        route_msg   = "SAVED";
    }
    else
    {
        route_state = ROUTE_ST_STOP;
        route_msg   = "TOO FEW POINTS";
    }
}

//--------------------------------------
// 函数简介     上电初始化（载入 active 槽）
//--------------------------------------
void Route_Crtl_Init (void)
{
    ins_route_init();
    ins_route_slot_init();
    (void)ins_route_flash_load();
}

//--------------------------------------
// 函数简介     页面主循环
//--------------------------------------
int Route_Crtl_Page (void)
{
    Route_Crtl_UI();

    route_state  = ROUTE_ST_IDLE;
    route_cursor = 3;
    route_msg    = "";

    // 进页面：双闭环待命（使能、目标 0），遥控要 CH6 = 1 才接管
    RC_Engage = RC_ENGAGE_STANDBY;

    Time_Count1 = 0;
    Time_Count2 = 0;

    while(1)
    {
        if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);

            if (ins_route_flash_busy())         // 写盘没完不能走
            {
                continue;
            }
            else if (ROUTE_ST_RUN == route_state)
            {
                if (ROUTE_MODE_REC == route_mode) { Route_Rec_Stop(); }
                else
                {
                    route_pb_stop("back");
                    route_state = ROUTE_ST_STOP;
                    route_msg   = "STOPPED";
                }
            }
            else
            {
                Remote_Crtl_Release();
                return 0;
            }
        }

        Motor_Crtl_HB_Feed();               // 本页会使能后轮闭环

        Remote_Crtl_Update();

        // 光标：运行中锁在 RUN 行
        if (ROUTE_ST_RUN == route_state)
        {
            route_cursor = 3;
        }
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            if (ROUTE_ST_RUN != route_state)
            {
                route_cursor = (0 == route_cursor) ? 3 : (uint8)(route_cursor - 1);
            }
        }
        if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            if (ROUTE_ST_RUN != route_state)
            {
                route_cursor = (3 == route_cursor) ? 0 : (uint8)(route_cursor + 1);
            }
        }

        // CH5 优先：接收机在线时一直是硬急停（锁存）
        if (1 == RC_Sw5)
        {
            RC_Engage = RC_ENGAGE_ESTOP;
            Remote_Crtl_Apply();

            if (ROUTE_ST_RUN == route_state)        // 急停即结束本次运行
            {
                if (ROUTE_MODE_REC == route_mode) { Route_Rec_Stop(); route_msg = "ESTOP"; }
                else
                {
                    route_pb_stop("estop");
                    route_state = ROUTE_ST_STOP;
                    route_msg   = "ESTOP";
                }
            }
        }
        // CH6：显式把驾驶权交给遥控；回 0 收回待命
        else if (1 == RC_Sw6)
        {
            if (0 != route_pb_active())             // 遥控接管：回放立即让位
            {
                route_pb_stop("rc takeover");
                route_state = ROUTE_ST_STOP;
                route_msg   = "RC TAKEOVER";
            }

            if ((RC_ENGAGE_STANDBY == RC_Engage) || (RC_ENGAGE_OFF == RC_Engage))
            {
                RC_Engage = RC_ENGAGE_ON;
            }

            if (RC_ENGAGE_ON == RC_Engage) { Remote_Crtl_Apply(); }
            else if ((RC_ENGAGE_STANDBY == RC_Engage) || (RC_ENGAGE_OFF == RC_Engage))
            {
                if (0 == route_pb_active()) { Route_Drive_Idle(); }
            }
        }
        else
        {
            if (RC_ENGAGE_ON == RC_Engage) { RC_Engage = RC_ENGAGE_STANDBY; }

            if ((RC_ENGAGE_STANDBY == RC_Engage) || (RC_ENGAGE_OFF == RC_Engage))
            {
                if (0 == route_pb_active()) { Route_Drive_Idle(); }
            }
            // ESTOP / FAILSAFE：保持锁存，目标已清 0
        }

        // 确认键：急停锁存时用来恢复；否则是开始/停止
        if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);

            if ((RC_ENGAGE_ESTOP == RC_Engage) || (RC_ENGAGE_FAILSAFE == RC_Engage))
            {
                if (0 == RC_Sw5) { RC_Engage = RC_ENGAGE_STANDBY; }
            }
            else if (ROUTE_ST_RUN == route_state)
            {
                if (ROUTE_MODE_REC == route_mode) { Route_Rec_Stop(); }
                else
                {
                    route_pb_stop("stopped");
                    route_state = ROUTE_ST_STOP;
                    route_msg   = "STOPPED";
                }
            }
            else if (0 == route_cursor)             // MODE：切模式
            {
                route_mode = (ROUTE_MODE_REC == route_mode) ? ROUTE_MODE_PLAY : ROUTE_MODE_REC;
            }
            else if (1 == route_cursor)             // SLOT：切到下一个槽并载入
            {
                if (ins_route_flash_busy())
                {
                    route_state = ROUTE_ST_FAULT;
                    route_msg   = "FLASH BUSY";
                }
                else
                {
                    uint8 s = (uint8)((ins_route_slot_active() + 1u) % (uint8)INS_RT_SLOT_COUNT);

                    (void)ins_route_slot_set_active(s, 1u);

                    if (0 != ins_route_flash_load_slot(s))
                    {
                        route_msg = "LOADED";
                    }
                    else
                    {
                        ins_route_clear();
                        route_msg = "NO TABLE";
                    }
                    route_state = ROUTE_ST_IDLE;
                }
            }
            else if (2 == route_cursor)             // PARAM：打开路表参数页
            {
                if (ins_route_flash_busy())
                {
                    route_state = ROUTE_ST_FAULT;
                    route_msg   = "FLASH BUSY";
                }
                else if (RC_ENGAGE_ON == RC_Engage)
                {
                    route_state = ROUTE_ST_FAULT;
                    route_msg   = "RELEASE CH6";
                }
                else
                {
                    Param_Page_Route();

                    ips200_clear();
                    Route_Crtl_UI();
                    Time_Count1 = 0;
                    route_msg   = "PARAM";
                }
            }
            else                                    // RUN：启动
            {
                if (ROUTE_MODE_REC == route_mode)
                {
                    if (ins_route_flash_busy())
                    {
                        route_state = ROUTE_ST_FAULT;
                        route_msg   = "FLASH BUSY";
                    }
                    else
                    {
                        ins_route_start();
                        route_state = ROUTE_ST_RUN;
                        route_msg   = "RECORDING";
                    }
                }
                else
                {
                    if (0 == route_pb_start())
                    {
                        route_state = ROUTE_ST_FAULT;
                        route_msg   = route_pb_reason();
                    }
                    else
                    {
                        route_state = ROUTE_ST_RUN;
                        route_msg   = "PLAYING";
                    }
                }
            }
        }

        // 录制采样 + 分步写盘 + 回放，每圈各推进一步
        ins_route_poll();
        ins_route_flash_poll();

        if (ROUTE_MODE_PLAY == route_mode) { route_pb_poll(); }

        // 录制被内部停掉（录满）→ 正常结束并落盘
        if ((ROUTE_ST_RUN == route_state) && (ROUTE_MODE_REC == route_mode) &&
            (0 == ins_route_is_running()))
        {
            if (ins_route_get_count() >= (uint16)ROUTE_MIN_SAVE_POINTS)
            {
                ins_route_flash_save_slot(FM_SLOT_MANUAL);
                route_msg = "FULL, SAVED";
            }
            else
            {
                route_msg = "TOO FEW POINTS";
            }
            route_state = ROUTE_ST_DONE;
        }

        // 回放自己结束（跑完 / 脱线）→ 映射到页面状态
        if ((ROUTE_ST_RUN == route_state) && (ROUTE_MODE_PLAY == route_mode) &&
            (0 == route_pb_active()))
        {
            route_state = (PB_ST_DONE == route_pb_state()) ? ROUTE_ST_DONE : ROUTE_ST_STOP;
            route_msg   = route_pb_reason();
        }

        // 写盘结束那一圈：失败就报异常
        {
            uint8 busy_now = ins_route_flash_busy();

            if ((0 != route_save_prev) && (0 == busy_now) &&
                (2 == ins_route_flash_last_rc()))
            {
                route_state = ROUTE_ST_FAULT;
                route_msg   = "SAVE FAIL";
            }
            route_save_prev = busy_now;
        }

        /* 屏幕显示更新（100ms） */
        if (Time_Count1 >= 10)
        {
            Time_Count1 = 0;

            // 光标
            ips200_show_string(0 ,32 , " ");
            ips200_show_string(0 ,48 , " ");
            ips200_show_string(0 ,64 , " ");
            ips200_show_string(0 ,80 , " ");
            ips200_show_string(0 ,(uint16)(32u + 16u * route_cursor), ">");

            ips200_show_string(66 ,32 , "    ");
            ips200_show_string(66 ,32 , Route_Mode_Text());
            ips200_printf(66 ,48 , "%d    ", (int)ins_route_slot_active());
            ips200_show_string(66 ,80 , (ROUTE_ST_RUN == route_state) ? "STOP " : "START");
            ips200_show_string(66 ,96 , Route_State_Text());

            ips200_printf(66 ,112, "%d    ", (int)(ins_route_get_total_dist() * 100.0f + 0.5f));
            ips200_printf(66 ,128, "%d    ", (int)ins_route_get_count());

            if (0 != ins_route_flash_busy())
            {
                ips200_printf(66 ,144, "BUSY %d%%    ", (int)ins_route_flash_progress());
            }
            else if (1 == ins_route_flash_last_rc()) { ips200_show_string(66 ,144, "OK      "); }
            else if (2 == ins_route_flash_last_rc()) { ips200_show_string(66 ,144, "FAIL    "); }
            else if (3 == ins_route_flash_last_rc()) { ips200_show_string(66 ,144, "EMPTY   "); }
            else                                     { ips200_show_string(66 ,144, "----    "); }

            ips200_printf(34 ,160, "%d ", (int)RC_Online);
            ips200_printf(82 ,160, "%d ", (int)RC_Engage);
            ips200_printf(122,160, "%d ", (int)RC_Sw5);
            ips200_printf(162,160, "%d ", (int)RC_Sw6);

            if (RC_ENGAGE_ESTOP == RC_Engage)         { ips200_show_string(66 ,176, "STOP      "); }
            else if (RC_ENGAGE_FAILSAFE == RC_Engage) { ips200_show_string(66 ,176, "RC LOST   "); }
            else if (RC_ENGAGE_ON == RC_Engage)       { ips200_show_string(66 ,176, "REMOTE    "); }
            else                                      { ips200_show_string(66 ,176, "STANDBY   "); }

            ips200_printf(66 ,192, "%d    ", (int)(route_pb_elat() * 100.0f));

            ips200_show_string(66 ,208, "                    ");
            ips200_show_string(66 ,208, route_msg);
        }
    }
}

/**********************************************************/
/*----------------------------------------[E] 接口实现 [E]*/
/**********************************************************/
