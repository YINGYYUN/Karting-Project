/*******************************************************************************
遥控（目前对应 HotRc 手持枪式遥控器）
换算规则、标定值与通道分配见 Remote_Ctrl.h
*******************************************************************************/


#include "zf_common_headfile.h"
#include "ins_imu.h"


/**********************************************************/
/*[S] 对外数据 [S]----------------------------------------*/
/**********************************************************/

volatile uint16 RC_Raw_Ch1  = 0;
volatile uint16 RC_Raw_Ch2  = 0;
volatile float  RC_Steer_Deg = 0.0f;
volatile float  RC_Throttle  = 0.0f;
volatile uint8  RC_Sw3       = 0;
volatile int8   RC_Sw4       = 0;
volatile uint8  RC_Sw5       = 0;
volatile uint8  RC_Sw6       = 0;
volatile uint8  RC_Online    = 0;
volatile uint8  RC_Engage    = RC_ENGAGE_OFF;

// ★★ 累计接收机帧数（导出给遥测通道 HOST_CH_RC_FRAMES）。
//    只由 Remote_Ctrl_Update() 消费 finsh_flag 时自增 —— 遥控页不再自己数一遍，
//    否则两个消费者抢同一个 finsh_flag：谁先清谁得，另一个永远数不到（症状是
//    "遥控页显示的帧数不涨，但遥测通道在涨"，或者反过来）。
volatile uint32 RC_Frames   = 0;

/**********************************************************/
/*----------------------------------------[E] 对外数据 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 方向盘信号整形 [S]----------------------------------*/
/**********************************************************/

static uint8  s_ch_bad    = 0;          // 通道掉线位图：bit0 CH1 / bit1 CH2
static uint16 RC_Steer_Mid = RC_STEER_MID;  // 方向盘静止位（开机学一次，之后慢跟踪）
static uint16 RC_Thr_Mid   = RC_THR_MID;    // 油门静止位
static uint8  s_mid_locked = 0;         // 0 = 还在开机捕获
static uint8  s_mid_cnt    = 0;         // 已看到的有效读数次数
static uint32 s_mid_ms     = 0;         // 上次慢跟踪时刻

static uint8  s_prev_on    = 0;         // 上一拍是否处在"遥控接管"，用来抓 ON 的上升沿

//--------------------------------------
// 函数简介     中位慢跟踪：每次最多挪 RC_MID_TRACK_STEP 计数
// 备注信息     明显不在中位（手推着杆）就一个计数都不许动，否则推杆会把中位学跑
//--------------------------------------
static void RC_Mid_Step (uint16 *mid, uint16 raw)
{
    int d = (int)raw - (int)(*mid);

    if ((d > RC_MID_TRACK_BAND) || (d < -RC_MID_TRACK_BAND)) { return; }

    if (d >= RC_MID_TRACK_STEP)       { *mid = (uint16)(*mid + RC_MID_TRACK_STEP); }
    else if (d <= -RC_MID_TRACK_STEP) { *mid = (uint16)(*mid - RC_MID_TRACK_STEP); }
}

//--------------------------------------
// 函数简介     中位学习（每圈调用）
// 备注信息     写死 992 时，静止位只要偏 20 计数就超出 ±8 死区，静止目标恒偏，
//              表现为方向盘自己抽/车自己慢慢动。开机取一次，之后 1 计数/100ms 慢跟踪。
//              ON 期间不许学：司机稳按一个小油门会被当成"中位漂移"学掉
//--------------------------------------
static void RC_Mid_Learn (void)
{
    int d1, d2;

    if (0 == s_mid_locked)
    {
        // 开机捕获：拿到 3 次有效读数后取一次；偏太多（上电时手按着杆）就不认，保持标称值
        if ((0 == RC_Online) ||
            (RC_Raw_Ch1 < RC_CH_VALID_MIN) || (RC_Raw_Ch2 < RC_CH_VALID_MIN)) { return; }

        if (++s_mid_cnt < 3) { return; }

        d1 = (int)RC_Raw_Ch1 - RC_STEER_MID;
        d2 = (int)RC_Raw_Ch2 - RC_THR_MID;

        if ((d1 <= RC_MID_CAP_BAND) && (d1 >= -RC_MID_CAP_BAND)) { RC_Steer_Mid = RC_Raw_Ch1; }
        if ((d2 <= RC_MID_CAP_BAND) && (d2 >= -RC_MID_CAP_BAND)) { RC_Thr_Mid   = RC_Raw_Ch2; }

        s_mid_locked = 1;
        s_mid_ms     = Sys_Tick_Ms;
        return;
    }

    if ((RC_ENGAGE_ON != RC_Engage) && ((uint32)(Sys_Tick_Ms - s_mid_ms) >= RC_MID_TRACK_MS))
    {
        s_mid_ms = Sys_Tick_Ms;
        RC_Mid_Step(&RC_Steer_Mid, RC_Raw_Ch1);
        RC_Mid_Step(&RC_Thr_Mid,   RC_Raw_Ch2);
    }
}

/**********************************************************/
/*----------------------------------------[E] 信号整形 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 内部函数 [S]----------------------------------------*/
/**********************************************************/

//--------------------------------------
// 函数简介     原始值 -> 归一化（-1.0 ~ +1.0）
// 参数说明     raw	当前原始通道值
// 参数说明     mid	中位原始值
// 参数说明     min	一端满幅原始值
// 参数说明     max	另一端满幅原始值
// 返回参数     float	-1.0 ~ +1.0
// 备注信息     中位两侧量程不对称（比如方向盘 776 : 824），所以两侧分别归一化，
//              这样两端都能吃满 ±1.0，不会有一边提前截断
//--------------------------------------
static float RC_Norm (uint16 raw, int mid, int min, int max)
{
    float t;

    if ((int)raw >= mid)
    {
        if (max <= mid) { return 0.0f; }
        t = (float)((int)raw - mid) / (float)(max - mid);
    }
    else
    {
        if (mid <= min) { return 0.0f; }
        t = (float)((int)raw - mid) / (float)(mid - min);
    }

    if (t >  1.0f) { t =  1.0f; }
    if (t < -1.0f) { t = -1.0f; }

    return t;
}

//--------------------------------------
// 函数简介     两态开关判定
// 参数说明     v	原始通道值
// 返回参数     uint8	0 = 低档，1 = 高档
// 备注信息     以三档的中值（992）为界
//--------------------------------------
static uint8 RC_Pick2 (uint16 v)
{
    return ((int)v > RC_SW_VAL_MID) ? 1 : 0;
}

//--------------------------------------
// 函数简介     三档开关判定
// 参数说明     v	原始通道值
// 返回参数     int8	-1 = 低档，0 = 中档，+1 = 高档
// 备注信息     取离三个标定值最近的那个
//--------------------------------------
static int8 RC_Pick3 (uint16 v)
{
    int x      = (int)v;
    int d_low  = (x > RC_SW_VAL_LOW)  ? (x - RC_SW_VAL_LOW)  : (RC_SW_VAL_LOW  - x);
    int d_mid  = (x > RC_SW_VAL_MID)  ? (x - RC_SW_VAL_MID)  : (RC_SW_VAL_MID  - x);
    int d_high = (x > RC_SW_VAL_HIGH) ? (x - RC_SW_VAL_HIGH) : (RC_SW_VAL_HIGH - x);

    if ((d_low <= d_mid) && (d_low <= d_high))  { return -1; }
    if ((d_high <= d_mid) && (d_high <= d_low)) { return +1; }

    return 0;
}

/**********************************************************/
/*[S] 舵角来源 [S]----------------------------------------*/
/**********************************************************/

// 遥控态：CH1 直通舵角（Servo_Target_Deg = RC_Steer_Deg），航向环不接管。
// 航向环（Head_Ctrl）只由路表回放（route_pb）启用。

/**********************************************************/
/*----------------------------------------[E] 舵角来源 [E]*/
/**********************************************************/

//--------------------------------------
// 函数简介     油门 -> 后轮速度目标
// 返回参数     float	rps（前进为正，与 Motor_Ctrl 同口径）
// 备注信息     前进 / 后退各自限幅，两个上限都由宏给
//--------------------------------------
static float RC_ThrottleToTarget (void)
{
    if (RC_Throttle > 0.0f) { return RC_Throttle * RC_THR_TARGET_FWD; }
    if (RC_Throttle < 0.0f) { return RC_Throttle * (-RC_THR_TARGET_REV); }

    return 0.0f;
}

/**********************************************************/
/*----------------------------------------[E] 内部函数 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 接口实现 [S]----------------------------------------*/
/**********************************************************/

//--------------------------------------
// 函数简介     从接收机快照换算一次
// 参数说明     void
// 使用示例     Remote_Ctrl_Update();
// 备注信息     只读 uart_receiver（10ms 中断里解析好的快照），不碰驱动
//              ★★ 本函数是 `uart_receiver.finsh_flag` **唯一**的消费者（2026-10-22 统一）：
//                以前这个累加写在遥控页的 20ms 分支里，于是
//                ① 遥测通道（Host 页）拿不到这个数，② 两个页面各数一遍会互相饿死。
//                现在无论 Host 页还是遥控页，只要在调 Update 就在计，RC_Frames 始终是真值。
//--------------------------------------
void Remote_Ctrl_Update (void)
{
    int thr;

    // 收到一帧就把标志清掉并累加：RC_Frames 能反映"接收机是否在持续发帧"
    if (uart_receiver.finsh_flag)
    {
        uart_receiver.finsh_flag = 0;
        RC_Frames ++;
    }

    RC_Raw_Ch1 = uart_receiver.channel[RC_CH_STEER];
    RC_Raw_Ch2 = uart_receiver.channel[RC_CH_THROTTLE];
    RC_Online  = uart_receiver.state;

    RC_Mid_Learn();

    // 通道有效性：接收机取不到某通道时 channel[] 是 0，而 0 在量程 192~1792 之外，
    // 归一化会算出 -1.24、夹到 -1 ⇒ 满舵 / 满倒车。raw 过低一律按"回中"处理
    s_ch_bad = 0;
    if (RC_Raw_Ch1 < RC_CH_VALID_MIN) { s_ch_bad = (uint8)(s_ch_bad | 0x01); }
    if (RC_Raw_Ch2 < RC_CH_VALID_MIN) { s_ch_bad = (uint8)(s_ch_bad | 0x02); }

    // 方向盘：死区 + 掉线都按回中；量程跟着学到的中位一起平移（span 不变，满杆仍是 ±1）
    if ((0 != (s_ch_bad & 0x01)) ||
        ((RC_Raw_Ch1 <= (RC_Steer_Mid + RC_STEER_DEADBAND)) &&
         (RC_Raw_Ch1 >= (RC_Steer_Mid - RC_STEER_DEADBAND))))
    {
        RC_Steer_Deg = 0.0f;
    }
    else
    {
        RC_Steer_Deg = (float)RC_STEER_DIR_SIGN
                     * RC_Norm(RC_Raw_Ch1, RC_Steer_Mid,
                               (int)RC_Steer_Mid - (RC_STEER_MID - RC_STEER_MIN),
                               (int)RC_Steer_Mid + (RC_STEER_MAX - RC_STEER_MID))
                     * RC_STEER_LIMIT_DEG;
    }

    // 油门：死区 + 掉线都按回中（掉线时绝不许变成满倒车）；量程同样跟着中位平移
    thr = (int)RC_Raw_Ch2 - (int)RC_Thr_Mid;
    if ((0 != (s_ch_bad & 0x02)) ||
        ((thr <= RC_THR_DEADBAND) && (thr >= -RC_THR_DEADBAND)))
    {
        RC_Throttle = 0.0f;
    }
    else
    {
        RC_Throttle = RC_Norm(RC_Raw_Ch2, RC_Thr_Mid,
                              (int)RC_Thr_Mid - (RC_THR_MID - RC_THR_MIN),
                              (int)RC_Thr_Mid + (RC_THR_MAX - RC_THR_MID));
    }

    // 开关
    RC_Sw3 = RC_Pick2(uart_receiver.channel[2]);        // CH3 两态（空置，仅显示）
    RC_Sw4 = RC_Pick3(uart_receiver.channel[3]);        // CH4 三档（空置，仅显示）
    RC_Sw5 = RC_Pick2(uart_receiver.channel[4]);        // CH5 两态
    RC_Sw6 = RC_Pick2(uart_receiver.channel[5]);        // CH6 两态
}

//--------------------------------------
// 函数简介     跑一次介入状态机并写目标
// 参数说明     void
// 使用示例     Remote_Ctrl_Apply();
// 备注信息     调用前必须先 Remote_Ctrl_Update()；
//              四种状态的处理见 Remote_Ctrl.h 里 RC_ENGAGE_* 的说明
//--------------------------------------
void Remote_Ctrl_Apply (void)
{
    float tgt = RC_ThrottleToTarget();

    // 急停：CH5 拨到高位即触发，锁存。松开 CH5 也不会自动退出（要按确认键才恢复）
    if (1 == RC_Sw5)
    {
        RC_Engage = RC_ENGAGE_ESTOP;
    }
    // 接收机报失控：掉进"失控挂起"（锁存，不自动恢复）
    // 急停优先：已锁在 ESTOP 时不被 FAILSAFE 顶掉
    else if ((0 == RC_Online) && (RC_ENGAGE_ESTOP != RC_Engage))
    {
        RC_Engage = RC_ENGAGE_FAILSAFE;
    }

    // 遥控态永远不接管航向环（本函数与 route_pb 分属不同页面，不会同拍跑）
    Head_Ctrl_Set_Enable(0);

    if (RC_ENGAGE_ON != RC_Engage) { s_prev_on = 0; }

    switch (RC_Engage)
    {
        case RC_ENGAGE_ON:              // 遥控接管：双闭环 + CH1 直通舵角
        {
            /* ★ 每次重新接管都清一次"编码器脱落"锁存。
               那个锁存原本只有上电或调试页能解：一次瞬时丢脉冲（起步打滑、插头抖、
               被顶住）就会让**某一路永久停机**，而那个轮子只是被拖着走、编码器照样报数
               ⇒ 现场看到的是"左侧永远比右侧快"，且与打角方向无关。
               真有故障时 200 ms 内会重新锁上，保护不会丢（见 Motor_Ctrl.h 的说明）。 */
            if (0 == s_prev_on)
            {
                Motor_Ctrl_Reset();
                s_prev_on = 1;
            }

            Servo_Ctrl_Enable    = 1;
            Motor_Ctrl_Enable    = 1;

            Servo_Target_Deg     = RC_Steer_Deg;

            // 电子差速：用实测方向盘角分配（目标角有滞后，会让后轮提前分速）
            Motor_Diff_Alloc(tgt, Servo_Angle_Deg, ROUTE_PB_L,
                             &Motor_LR_Ctrl.target, &Motor_RR_Ctrl.target);
            break;
        }

        case RC_ENGAGE_STANDBY:         // 待命：双闭环使能，但目标 0（遥控还没接管）
            Servo_Ctrl_Enable    = 1;
            Motor_Ctrl_Enable    = 1;
            Servo_Target_Deg     = 0.0f;
            Motor_LR_Ctrl.target = 0.0f;
            Motor_RR_Ctrl.target = 0.0f;
            break;

        case RC_ENGAGE_ESTOP:           // 急停：后轮保持使能、目标 0（主动刹停）
            Servo_Ctrl_Enable    = 1;   // 舵机仍使能，保住转向
            Motor_Ctrl_Enable    = 1;
            Motor_LR_Ctrl.target = 0.0f;
            Motor_RR_Ctrl.target = 0.0f;
            // 故意不写 Servo_Target_Deg：保持急停前的转向，
            // 避免车在减速过程中突然自己回正（那会更危险）
            break;

        case RC_ENGAGE_FAILSAFE:        // 失控挂起：舵机失能；后轮保持使能、目标 0
            Servo_Ctrl_Enable    = 0;
            Servo_Target_Deg     = 0.0f;
            Motor_Ctrl_Enable    = 1;
            Motor_LR_Ctrl.target = 0.0f;
            Motor_RR_Ctrl.target = 0.0f;
            break;

        default:                        // OFF：全部失能、目标 0
            Servo_Ctrl_Enable    = 0;
            Servo_Target_Deg     = 0.0f;
            Motor_Ctrl_Enable    = 0;
            Motor_LR_Ctrl.target = 0.0f;
            Motor_RR_Ctrl.target = 0.0f;
            break;
    }
}

//--------------------------------------
// 函数简介     解除介入
// 参数说明     void
// 使用示例     离开遥控页面时调用
// 备注信息     不依赖下一拍：直接把输出侧的使能与目标收干净
//--------------------------------------
void Remote_Ctrl_Release (void)
{
    RC_Engage = RC_ENGAGE_OFF;

    Head_Ctrl_Set_Enable(0);

    Servo_Ctrl_Enable    = 0;
    Servo_Target_Deg     = 0.0f;
    Motor_Ctrl_Enable    = 0;
    Motor_LR_Ctrl.target = 0.0f;
    Motor_RR_Ctrl.target = 0.0f;
}

/**********************************************************/
/*----------------------------------------[E] 接口实现 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 页面 [S]--------------------------------------------*/
/**********************************************************/

//-------------------------------------------------------------------------------------------------------------------
// [一级界面]遥控页面
// 数据源：Remote_Ctrl_Update() 换算出来的指令量（原始值来自 10ms 中断的 uart_receiver 快照）
// WiFi 上行：CH1,CH2,DEG10,THR,K3,S4,K5,K6,ST,EN,TGT,FRAMES （20ms 一行）
//   DEG10 = 方向盘指令角，单位 0.1°；THR = 油门千分比（-1000~+1000）
//   ST = 接收机状态；TGT = 后轮目标(0.01 rps)
//   EN = 介入状态：0 全部失能 / 1 待命(闭环使能、目标 0) / 2 遥控接管 / 3 失控挂起 / 4 急停
//   FRAMES = 累计收帧数（一直不涨 -> 接收机没在发 / 帧格式对不上）
//-------------------------------------------------------------------------------------------------------------------

static void Remote_Ctrl_UI (void)
{
    ips200_show_string(8  ,0  , "[REMOTE]");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "CH1:####      CH2:####");
    ips200_show_string(10 ,48 , "DEG:+##.#   THR:+####");
    ips200_show_string(10 ,64 , "K3:# K5:# K6:#  S4:#");
    ips200_show_string(10 ,80 , "ST:#  FRM:#####");
    ips200_show_string(10 ,96 , "EN:#  TGT:+####");
    ips200_show_string(10 ,112, "YAW:         H:#");
    ips200_show_string(10 ,128, "SRV:         E:#");
    ips200_show_string(10 ,144, "MOT F: L:#  R:#");   // 0 = 正常，1 = 该路编码器脱落已停机
    ips200_show_string(10 ,176, "CONFIRM: RC ON/OFF");
}

// 屏幕输出：带符号的整数（分符号输出，避免依赖 %+d 的格式支持）
static void Remote_Ctrl_ShowInt (uint16 x, uint16 y, int32_t v)
{
    if (v < 0)
    {
        ips200_printf(x, y, "-%d    ", (int)(-v));
    }
    else
    {
        ips200_printf(x, y, "+%d    ", (int)v);
    }
}

// 屏幕输出：带符号的角度，单位 0.1°
static void Remote_Ctrl_ShowDeg (uint16 x, uint16 y, int32_t deg10)
{
    int32_t v = (deg10 >= 0) ? deg10 : -deg10;

    if (deg10 < 0)
    {
        ips200_printf(x, y, "-%d.%d   ", (int)(v / 10), (int)(v % 10));
    }
    else
    {
        ips200_printf(x, y, "+%d.%d   ", (int)(v / 10), (int)(v % 10));
    }
}

int Remote_Ctrl_Page (void)
{
    Remote_Ctrl_UI();

    // 重置舵机状态，包括强制失能舵机闭环一次
    Servo_Ctrl_Reset();

    // 进页面即让双闭环"待命"：两路使能、目标 0。
    // 车不会动，但方向盘会被驱动回 0°；按下确认键后遥控才接管目标
    RC_Engage = RC_ENGAGE_STANDBY;

    // 参考计时值重置
    Time_Count1 = 0;
    Time_Count2 = 0;

    // ★ 累计帧数改用全局 RC_Frames（见 Remote_Ctrl.h）：累加动作已挪进 Remote_Ctrl_Update()，
    //   本页不再自己数 —— 否则两处抢同一个 finsh_flag，必然有一处永远不涨。

    while(1)
    {
        if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);
            Remote_Ctrl_Release();      // 退出即解除：双闭环失能、目标清零
            // 返回主菜单
            return 0;
        }

        // 心跳：本页面在监督后轮闭环，必须每圈喂一次。
        // 漏喂 -> Motor_Ctrl 的心跳门会一直判定"监督循环已死"，电机输出恒为 0。
        Motor_Ctrl_HB_Feed();

        // 先算遥控指令量（纯算术，开销可忽略）
        Remote_Ctrl_Update();

        // 确认键：只有接收机状态正常（ST = 1）才允许让遥控接管 / 收回
        // （遥控收回后回到 STANDBY：闭环仍然使能，只是目标回到 0）
        if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            // 急停还挂在 CH5 上时不允许恢复：必须先松开 CH5，再按确认键
            if (RC_Online && (0 == RC_Sw5))
            {
                RC_Engage = (RC_ENGAGE_ON == RC_Engage) ? RC_ENGAGE_STANDBY : RC_ENGAGE_ON;
            }
        }

        // 跑介入状态机并写目标
        Remote_Ctrl_Apply();

        /* WiFi 发送（约 20ms 周期） */
        if (Time_Count2 >= 2)   // 10ms * 2
        {
            Time_Count2 = 0;

            // FRAMES 的累加已在 Remote_Ctrl_Update() 里统一做掉（消费 finsh_flag），
            // 本页只负责显示 —— 不要再在这里数一遍。

            // 打印所有遥控器数据的演示
            // if(wifi_spi_inited)
            // {
            //     char buf[96];
            //     sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            //             (int)RC_Raw_Ch1, (int)RC_Raw_Ch2,
            //             (int)(RC_Steer_Deg * 10.0f), (int)(RC_Throttle * 1000.0f),
            //             (int)RC_Sw3, (int)RC_Sw4, (int)RC_Sw5, (int)RC_Sw6,
            //             (int)RC_Online, (int)RC_Engage,
            //             (int)(Motor_LR_Ctrl.target * 100.0f), (int)RC_Frames);
            //     wifi_spi_send_buffer((uint8_t *)buf, (uint32)strlen(buf));
            // }

            // 舵机/航向数据上发（每 20ms 一行，6 列）：
            //   施加目标角, 实际角, PWM 输出, 实测航向, 舵机误差, 航向环接管
            // 角度单位 0.01°（×100）：编码器 1 计数 ≈ 0.02°，×100 不丢分辨率，
            //   且与后轮通道（actual/target ×100、out ×10000）的 ×100 口径一致，便于同图对比。
            // ★ 第一列用 Servo_Target_Applied_Deg（软限位夹紧后实际施加的目标），不要用 Servo_Target_Deg：
            //   后者是用户设定值，命令超过软限位(±28°)时它仍显示 ±30°，拿 目标-实际 反推误差会
            //   看到约 2° 的假误差（实际早已进死区）。
            // PWM 直接取 Servo_Out 整数值 —— 它就是 Motor_Set 的刻度（±SERVO_U_MAX = ±7500），
            //   与后轮 out→duty 的 ×10000 不同，这里无需再缩放。
            if (wifi_spi_inited)
            {
                char buf[96];
                sprintf(buf, "%d,%d,%d,%d,%d,%d\n",
                        (int)(Servo_Target_Applied_Deg * 100.0f),   // 施加目标角 0.01°
                        (int)(Servo_Angle_Deg      * 100.0f),   // 实际角 0.01°
                        (int)Servo_Out,                         // PWM 输出 duty
                        (int)(ins_imu_get_yaw() * 100.0f),       // 实测航向 0.01°
                        (int)(Servo_Err_Deg * 100.0f),          // 舵机误差 0.01°
                        (int)Head_Ctrl_Enabled());              // 航向环接管（回放时 1）
                wifi_spi_send_buffer((uint8_t *)buf, (uint32)strlen(buf));
            }
        }

        /* 屏幕显示更新（100ms 周期） */
        if (Time_Count1 >= 10)  // 10ms * 10 显示周期
        {
            Time_Count1 = 0;

            ips200_printf(42 ,32 , "%d    ", (int)RC_Raw_Ch1);
            ips200_printf(154,32 , "%d    ", (int)RC_Raw_Ch2);
            Remote_Ctrl_ShowDeg(42 ,48 , (int32_t)(RC_Steer_Deg * 10.0f));
            Remote_Ctrl_ShowInt(138,48 , (int32_t)(RC_Throttle * 1000.0f));
            ips200_printf(34 ,64 , "%d ", (int)RC_Sw3);
            ips200_printf(74 ,64 , "%d ", (int)RC_Sw5);
            ips200_printf(114,64 , "%d ", (int)RC_Sw6);
            Remote_Ctrl_ShowInt(162,64 , (int32_t)RC_Sw4);
            ips200_printf(34 ,80 , "%d ", (int)RC_Online);
            ips200_printf(90 ,80 , "%d    ", (int)RC_Frames);
            ips200_printf(34 ,96 , "%d ", (int)RC_Engage);
            Remote_Ctrl_ShowInt(90 ,96 , (int32_t)(Motor_LR_Ctrl.target * 100.0f));   // 0.01 rps
            Remote_Ctrl_ShowDeg(42 ,112, (int32_t)(ins_imu_get_yaw() * 10.0f));       // 0.1 deg
            Remote_Ctrl_ShowInt(130,112, (int32_t)Head_Ctrl_Enabled());                // 1 = 航向环接管（回放）
            Remote_Ctrl_ShowDeg(42 ,128, (int32_t)(Servo_Angle_Deg * 10.0f));          // 实测舵角 0.1°
            Remote_Ctrl_ShowDeg(130,128, (int32_t)(Servo_Err_Deg * 10.0f));            // 舵机误差 0.1°
            ips200_printf(82 ,144, "%d ", (int)Motor_LR_Ctrl.enc_fault);               // 左后轮编码器停机
            ips200_printf(122,144, "%d ", (int)Motor_RR_Ctrl.enc_fault);               // 右后轮编码器停机
        }
    }
}

/**********************************************************/
/*----------------------------------------[E] 页面 [E]----*/
/**********************************************************/
