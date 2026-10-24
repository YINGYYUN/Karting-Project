/*******************************************************************************
上位机对接（PC 端 = 队友的 KartHost）

数据来源/去向：逐飞 WiFi-SPI 模块上的 UDP 链路
               上发遥测帧、接收命令帧；帧协议与队友车端 motor_link.c 逐字节对齐
本模块的职责：把实时数据按帧上发、把上位机下发的命令解析并执行，并提供一个主菜单页面

★★ 两条调用纪律（改本模块、或想给别的页面加链路之前先读这里 —— 都是"故意的"，不是漏写）

   ① 静默：Host_Crtl_Task() 的调用点**只有 Host_Crtl_Page()** 一处。
      主菜单 Menu_Show() 的 while(1) 里**故意不挂** —— 待机时整车静默，一个字节都不发。
      别看到"主菜单没调用"就顺手补上。其它页面想有波形，在该页 while(1) 里加一行即可。
      ★ 2026-10-17 起有第二道门：Task 内部判 Host_Link，关着就 return。
        两道门是冗余的（不进页面 → 没人调；进了没开 Link → 内部拦），但都留着：
        页面那道管"待机静默"，Link 那道管"页内暂停"。
      退出 Host 页即静默，但**链路本身仍连着**（wifi_spi_inited 保持 1），重进不用重连
      （WiFi + Socket 重连要十几秒，不能每次进页面重连）。

   ② 链路来源：UDP 链路可以在两处开起来 —— 调试页与 Host 页自己。
      · 调试页：主菜单 → Debug → 第 1 项 WIFI-SPI。★ 该菜单项现在**直接调用本模块的
        `Host_Crtl_WiFi_Select()`**（原来 Debug.c 里那份独立的界面/候选表已删除）。
        调试页主菜单第 2 行的 "WIFI-SPI %d" 就是"连上没有"的指示灯。
      · Host 页：**进门自动检查**，未连就先弹同一个选择界面。
      两处跑的都是 `wifi_spi_init` + `wifi_spi_socket_connect`，都成功才置 `wifi_spi_inited = 1`
      —— 共用同一个标志，**谁先连上，另一个直接复用**，不会重复 init。
      ⚠ `WIFI_SPI_AUTO_CONNECT = 0` → 每次上电都得手按一次。
      ★ Host 页的连接时机（**进门先连**）：进入 Host 页时若 `wifi_spi_inited == 0`，
        立刻弹出 WiFi 候选界面（标题 `[WIFI]-SPI`，上下键选热点 / 确认键连 / **BACK 回主菜单**）；
        连上后才进 Host 页主界面；**没连上就不进**（BACK 取消 = 直接退回主菜单）。
        已连则直接进页面，不弹任何界面（此时会顺带把参数表补发一次）。
      ⚠ 选择界面本身**不阻塞** —— 只有按确认键选定热点后才跑 init（最长十几秒）。
      ★★ 候选表 `HOST_WIFI_LIST` 与目标 IP/端口 `HOST_TARGET_*` 现在都是**全工程唯一一份**，
        两个入口共用 → 换热点 / 换上位机只改本头文件。
      ⚠ 调试页的数据页在 `wifi_spi_inited` 为真时会往**同一条 socket** 发**文本 CSV** ——
        两种格式混流会让上位机的帧解析器持续计入丢包/CRC 错。用帧协议时别停在那些页面。

★★ 分层（这是与 Remote_Crtl 最关键的区别，不要照抄它的形状）
   链路层 —— Host_Crtl_Task()：收命令 + 发遥测，含限频与"模块忙就跳过"，不含任何界面代码。
             ★ 受 Host_Link 门控：Link = OFF 时进函数第一句就 return，**上行下行一起停**。
   页面层 —— Host_Crtl_Page()：三个开关 + 链路体检 + 进门自动连接 + 遥控/上位机的仲裁。
   界面层 —— Host_Crtl_WiFi_Select()：**全工程统一的** WiFi 选择连接界面（Debug 页也调它）。

★★ 页面上的三个开关 + 两个遥控开关（2026-10-17 晚定稿，取代同日的"总闸"模型）
   **两条控制通路是并行独立的**，不是一棵树的上下级：

       Host Link   ── 只管 WiFi 链路：关 = 上位机收 + 发全停，**遥控一概不受影响**
         └ Host Mode  ── 依附于 Link：只能在 Link = OFF 时改；Link = ON 期间不许改
       Remote Crtl ── 只管"采不采信接收机"：独立于 Link，与 WiFi 毫无关系
                      "UP + Link OFF + 遥控开车"是**合法组合**（纯本地跑车，不上发）

   Host Link  每次进页面都回 **OFF**（进页面 = 一次上电式复位）。Link 从 OFF→ON 的那一刻
              立刻补发参数表与 EVENT，上位机的参数面板靠它长出来。
              ★ 关 Link **不收回遥控**：只在遥控未接管时才清驱动（遥控在跑的时候清驱动，
                车会因 Motor_Crtl_Enable 1→0→1 顿一下）。
              ⚠ "与接收机建立通信"是接收机自己做的（UART 一直在收）。Remote = OFF 的
                准确含义是**不采信**接收机数据、不接管，而不是把接收机断电。
   Host Mode  两档，见下面的 HOST_MODE_*。
              ★★ Link = ON 期间**禁止改 Mode** —— 改模式等于换安全责任归属，
                 必须是"先停车、再换模式"的动作，不能运行中切。
   Remote Crtl 开的是"遥控接管"（RC_ENGAGE_ON）。屏幕值不止 ON/OFF：
                 OFF     未接管（不采信接收机）
                 READY   采信了，但驾驶权不在这里（CMD 模式 + CH6 = 0，上位机在开）
                 DRIVE   采信了，且驾驶权在这里（正在开车）
                 RC LOST 接收机失联（RC_ENGAGE_FAILSAFE，锁存）
                 ESTOP   CH5 急停（锁存）
              在 RC LOST / ESTOP 上按确认 = 尝试恢复：要求 `RC_Online && !RC_Sw5` 才放行；
              软件无法主动重连接收机（RC_Online 是 uart_receiver.state 的硬件自报），
              只能等它自己重新同步。

★★★ 驾驶权仲裁（这是安全模型，不是实现细节，别改成"最后写者赢"）
   `host_auth_resolve()` 每圈算一次"谁在写驱动侧"，三种结果：

       STOP    遥控侧锁存（ESTOP / RC LOST）→ 停车优先于一切
       REMOTE  遥控开：UP 模式下直接给；CMD 模式下要 **CH6 = 1** 才给
       HOST    CMD 模式且 Link = ON（UP 模式没有 HOST，因为上位机在 UP 下不许开车）
       NONE    没人开（UP + Remote OFF / CMD + Link OFF + CH6 = 0）→ 驱动侧清空

       CH5 = 1 一律停车（STOP），**与 Host Mode、Host Link 全都无关** —— 前提只是
       Remote = ON 且接收机在线。这是 CMD 模式下唯一的物理急停通道，不能省。

   Host_Crtl_Page() 每圈只做两件事：
       ① 算 auth；② auth 变化时先清驱动侧，再交给对应的一方
       ```
       if (auth != prev) { if (REMOTE != auth) host_actuator_clear(); prev = auth; }
       if (STOP == auth || REMOTE == auth) { Remote_Crtl_Apply(); }
       else                                { host_drive_tick();   }
       ```
   ★ 为什么"变化时先清"是必须的：清目标这件事只写在 Apply 的 OFF 分支里，而一旦
     交回上位机 / 变成 NONE，本页就不再调 Apply —— 不清就会出现
     "遥控让出驾驶权了，车还按最后那个遥控目标在跑"。
   ★ 为什么 CMD 要 CH6 而不是"遥控一开就抢"：CMD 模式把安全责任交给上位机，
     人工接管必须是一次**显式动作**（拨杆），不能靠"谁最后写"。
   ★ 因此 Remote_Crtl_Apply() 不能在 RC_ENGAGE_OFF 时被调 —— 它在 OFF 分支会清掉目标，
     那样每圈都会把上位机刚写的目标抹掉，表现为"上位机指令完全无效"。
   ★ CMD + Remote DRIVE 时**心跳超时不停车**（auth 已经是 REMOTE，host_drive_tick 不被调用）。
     人在开车，PC 掉线与他无关 —— 与队友 spd_safety 里 s_local 豁免上位机门同源。
   ⚠ 反过来，**Remote = ON 但接收机失联时，即使车是上位机在开也会停车**（RC LOST 锁存）。
     理由：这时候 CH5 已经不可信，等于安全员失去了急停手段。要"上位机独跑"就把 Remote 关掉
     —— 那条边界现在很清晰：Remote = ON 就等于"安全员在岗"，他掉线就停车。

★★ 心跳超时之后（只在 CMD 模式、且遥控未接管时判定）
   锁存 Host_Lost = 1 并**不会自动恢复**：上位机重新发帧也不解除，必须人工在板子上按确认键。
   锁存期间驱动侧 = **闭环刹停**：后轮保持使能、目标 0（速度环主动出力减速）；
   舵机保持失联前的转向（故意不回正 —— 减速过程中方向盘突然自己回正更危险）。
   这一组与 RC_ENGAGE_ESTOP 完全同口径。

★ 与队友的有意差异（不要"补齐"）
   ① 参数表：队友 34 项，含 PWM 极性 / 编码器取反这类**装机时定一次**的量；
      本工程的习惯是"板载菜单测出来、代码里写死" → 参数表只留 `param_cache[]` 那 10 项
      （真正需要在线整肃的增益）。X-Macro 格式照抄，**要加参数只加一行**。
   ② 遥测档案：**已与队友完全对齐**（2026-10-22）—— 全量 41 路 / 标定 18 路，
      下标、顺序、单位、位义都照 motor_link.h，运行期由参数 `tel_sel` 切换。
      ⚠ 也就是说"裁剪到 25 路"那个阶段结束了：本模块的通道表就是队友的表，
      要加通道请先改队友那份（顺序即协议），再同步上位机 Channels.cs。
   ③ CMD 模式**只到"远程使能/失能"**：下行还没有"设目标"的命令，所以上位机目前还开不了车。
      骨架已齐（Mode + Link + 心跳锁存 + 刹停），将来要开车只在 host_drive_tick() 里
      多写两行目标赋值即可，**帧类型码 0x10~0x12 留给路表，别占用**。
   ④ 队友的失联阈值 `lost_ms` 是参数表 ID 14（100~3000ms 在线可调）；本工程写死
      HOST_ALIVE_MS。理由是遥控器本身还有一层 FAILSAFE，上位机这层不需要现场整定。
*******************************************************************************/


#ifndef __HOST_CRTL_H__
#define __HOST_CRTL_H__


#include "zf_common_headfile.h"


/**********************************************************/
/*[S] 链路配置 [S]----------------------------------------*/
/**********************************************************/

// 换网络/换电脑只改这里 —— 这是**全工程唯一一份**链路配置（2026-10-16 统一后）：
//   调试页原来那份 `Debug_WiFi_SSID[]/Debug_WiFi_PASS[]` 与硬编码 IP 已随界面统一而删除，
//   调试页的 WIFI-SPI 现在直接调用本模块的 `Host_Crtl_WiFi_Select()`。改完这一处，两个入口同时生效。
#define HOST_TARGET_IP          "192.168.50.247"    // 上位机 IP
#define HOST_TARGET_PORT        "8086"              // 上位机端口
#define HOST_LOCAL_PORT         "6666"              // 本机端口，一般不用改

// ★ WiFi 候选表：**进入 Host 页时若链路还没建立**，就在屏幕上列出这几条让用户选。
//   格式 X(热点名, 密码)；无密码的热点写 NULL。
//   ★★ 这是**全工程唯一一张候选表**：调试页的 WIFI-SPI 也走同一张界面，改这里两边同时生效。
#define HOST_WIFI_LIST(X)                               \
    X("ASC_Circuit_IoT", "1145141919810")               \
    X("YUNYYING",        "12345678")

// TCP / UDP 二选一。**改这里，上位机左上角的下拉框要选同一个**，否则收不到数据。
// UDP：丢帧不拖累主循环、上位机可随时开关；TCP：模块负责重连、每帧都到。
#define HOST_SOCK_TYPE          "UDP"

// 遥测周期 ms（20 -> 50Hz）。落地做阶跃标定时调小它 —— 阶跃响应的时间分辨率就是这个值。
#define HOST_TX_PERIOD_MS       (20)
// 收命令周期 ms（10 -> 100Hz）
#define HOST_RX_PERIOD_MS       (10)

// "链路活着"的判据：距上次收到下行帧不超过它，就算在线（见 HOST_ST_* 的说明）
#define HOST_ALIVE_MS           (600)

// 页面状态区的刷新节拍（单位 = PIT_CH0 的 10ms 拍）。30 = 300ms。
// 链路状态是给人看的，100ms 刷一次对眼睛是负担、还白占 CPU 带宽；300ms 足够。
// ★ 改它还不会影响收发 —— 收发节拍由上面的 HOST_TX/RX_PERIOD_MS 单独管。
#define HOST_UI_REFRESH_TICKS   (30)

// ★★ 上位机链路开关（Host Link）。关着的时候 Host_Crtl_Task() 第一句就返回：
//    上行不发、下行不收 —— **不是"只停上行"**，是这条 WiFi 链路整体停摆（屏幕上的 IDLE 就是这个意思）。
//    ★★ 它**只管 WiFi**：不碰遥控（遥控走 UART）。关 Link 时若遥控正在接管，
//       驱动侧一个字节都不清 —— 真正"全收"只有退出页面（Host_Crtl_Release）才做。
//    每次进页面都复位成 OFF；退出页面也回 OFF。
#define HOST_LINK_OFF           (0)
#define HOST_LINK_ON            (1)

// ★★ 工作模式（Host Mode）。**只在 Link = ON 之后才生效**，且 Link = ON 期间不允许改。
//
//    UP  仅上行 —— "不依赖上位机也能工作"，不是"关闭接收"：
//          · 遥测照发（不管上位机开没开，UDP 发出去就算完，不等确认）
//          · 下行照收：SET_PARAM / SAVE / LOAD / GET_TABLE **全部正常执行** —— 调参要用！
//          · **SET_ENABLE 只记录位图、回 ACK，不碰车上的使能与目标**
//          · 不判心跳，HOST_ALIVE_MS 在本模式下没有后果
//          · 可以开 Remote Crtl —— 这是"人开车 + 上位机看帧协议波形"的组合，
//            也是本页最常用的形态（遥控页那套文本 CSV 与帧协议混流，看不了波形）
//          · ★ 本模式下 **Remote = ON 就直接开车**，不需要 CH6（只有一条驾驶通路，
//            不需要"移交"这个动作）
//
//    CMD 上位机指令 —— 车听上位机的：
//          · 按 SET_ENABLE 的位图真正写 Servo_Crtl_Enable / Motor_Crtl_Enable
//          · 判心跳：HOST_ALIVE_MS 内没有合法下行帧 -> 锁存 Host_Lost -> 闭环刹停
//          · 驾驶权默认在上位机；★ **CH6 = 1 才把驾驶权交给遥控**（拨杆 = 显式移交），
//            CH6 回 0 立刻交回上位机（上位机若已锁存/没开 Link，则没人开 → 停车）
//          · 遥控驾驶期间心跳门不生效（auth = REMOTE 时根本不走 host_drive_tick）
//          · ⚠ 单按 BACK 退出页面时是 `Motor_Crtl_Enable = 0`（**滑行**，不是刹停），
//            与遥控页一致。车在跑就先关 Link 让它刹停，再退出
#define HOST_MODE_UP            (0)
#define HOST_MODE_CMD           (1)

/**********************************************************/
/*----------------------------------------[E] 链路配置 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 遥测通道表 [S]--------------------------------------*/
/**********************************************************/

// ★★ 2026-10-22：通道表**整套换成队友 motor_link.h 的那一套**（全量 41 路 + 标定 18 路），
//    不再做"双档案兼容"—— 历史 30 列 CSV 作废是已知代价（见文件头"有意差异"②）。
//    权威定义：zjc_kart_cyt4bb7-main/.../route_pp_wifi/code/motor_link.h 第 152~266 行。
//
// ★ 下标即协议：必须与上位机「档案」下拉框里那一档逐项对齐，**改顺序 = 改协议**。
//   只准往后加，老下标一个不动（否则历史 CSV 的列义会整体错位）。
//   单位一律用物理量本身的单位（deg / rps / % / m），不做定点缩放 ——
//   帧里是 float32，缩放只会平白引入"口径"这一层要记的东西。
//
// ★★ 运行期档案切换：实际发送数在 host_ch_count 里（41 或 18），由参数 `tel_sel` 每帧选：
//      tel_sel = 0 -> 全量 41 路（下表 HOST_CH_*）
//      tel_sel = 1 -> 标定 18 路（下表 HOST_CT_*，就是全量档里挑出的 18 项）
//    帧头 COUNT 随之变化，上位机按 COUNT 自动切档案。切换后**无需重启**。
#define HOST_CH_COUNT_MAX       (41)    // 全量档案通道数（缓冲按它开）

/*===================================================================================================================
 *  ★ 队友下标 → 我方来源 对照表（41 行，逐行对齐 motor_link.h 的 CH_* 与 motor_link.c 的口径）
 *
 *  下标  队友宏             单位     我方来源 / 换算
 *  ---- ------------------ -------- ------------------------------------------------------------------------------
 *    0   CH_TARGET_L        rps      Motor_LR_Crtl.target_app
 *    1   CH_V_L             rps      Motor_LR_Crtl.actual
 *    2   CH_U_L             %        Motor_LR_Crtl.out    * 100.0f      ← 我方 out 是归一化 ±1，**必须 ×100**
 *    3   CH_TARGET_R        rps      Motor_RR_Crtl.target_app
 *    4   CH_V_R             rps      Motor_RR_Crtl.actual
 *    5   CH_U_R             %        Motor_RR_Crtl.out    * 100.0f
 *    6   CH_UFF_L           %        Motor_LR_Crtl.u_ff   * 100.0f
 *    7   CH_UFF_R           %        Motor_RR_Crtl.u_ff   * 100.0f
 *    8   CH_Z_L             rps*s    Motor_LR_Crtl.z
 *    9   CH_Z_R             rps*s    Motor_RR_Crtl.z
 *   10   CH_POS_L           cnt      ENC_LR_POS                    （Motor.c 的带符号累计计数）
 *   11   CH_POS_R           cnt      ENC_RR_POS
 *   12   CH_ONLINE          0/1      (HOST_ST_ACTIVE == Host_Crtl_State()) ? 1 : 0
 *   13   CH_RX_AGE_MS       ms       距上次收到下行帧 ms；从未收到 = -1
 *   14   CH_ENABLE_BITS     位图     (float)(host_enable & 0x0006)  → bit1 电机 / bit2 总使能
 *   15   CH_SAFETY          位图     合成值，见下面 HOST_SAFETY_*（★ 近似，非等价，见注释）
 *   16   CH_STEER_TARGET    deg      Servo_Target_Applied_Deg      （施加值，不是用户设定值）
 *   17   CH_STEER_DELTA     deg      Servo_Angle_Deg
 *   18   CH_STEER_U         duty     Servo_Out                     （±10000 刻度）
 *   19   CH_STEER_EN        0/1      Servo_Crtl_Enable
 *   20   CH_STEER_STATUS    位图     bit0 = !Servo_Enc_Fault / bit1 = Servo_Enc_Fault / bit2 = 0（无顶死判据）
 *   21   CH_RC_ENGAGE       0~4      RC_Engage
 *   22   CH_RC_CH1          cnt      RC_Raw_Ch1
 *   23   CH_RC_CH2          cnt      RC_Raw_Ch2
 *   24   CH_RC_FRAMES       计数     RC_Frames                     （Remote_Crtl.c 的全局量，见 Remote_Crtl.h）
 *   25   CH_K_V             -        Motor_LR_Crtl.k_v
 *   26   CH_K_Z             -        Motor_LR_Crtl.k_z
 *   27   CH_INS_YAW         deg      ins_imu_get_yaw()
 *   28   CH_INS_GZ          dps      ins_imu_get_gyro_z_dps()
 *   29   CH_INS_X           m        ins_odom_get_x()               （右为正）
 *   30   CH_INS_Y           m        ins_odom_get_y()               （前为正）
 *   31   CH_INS_DIST        m        ins_odom_get_dist()            （累计，含倒车）
 *   32   CH_INS_FLAGS       位图     ins_imu_flags()                （INS_ST_* 惯导状态位）
 *   33   CH_RT_CNT          -        ins_route_get_count()
 *   34   CH_RT_ST           位图     RT_ST 合成，见下面 HOST_RT_ST_*（低 8 位状态位 + bit8..11 槽号）
 *   35   CH_PB_ELAT         m        route_pb_elat()
 *   36   CH_TAR_YAW         deg      Head_Crtl_Get_Target()
 *   37   CH_HT_SEG          -        0                             ★ 无此模块（head_test 未迁移），固定 0，非等价
 *   38   CH_TICK_MS         ms       Sys_Tick_Ms
 *   39   CH_INS_FRZ         deg      ins_imu_zupt_frz_deg()
 *   40   CH_INS_STATIC      0/1      (ins_imu_flags() & INS_ST_STATIC) ? 1 : 0
 *=================================================================================================================*/
#define HOST_CH_TARGET_L        (0)     // 目标转速 左轮 rps
#define HOST_CH_V_L             (1)     // 实测转速 左轮 rps
#define HOST_CH_U_L             (2)     // 实际输出 左轮 %（带符号；我方 out×100）
#define HOST_CH_TARGET_R        (3)     // 目标转速 右轮 rps
#define HOST_CH_V_R             (4)     // 实测转速 右轮 rps
#define HOST_CH_U_R             (5)     // 实际输出 右轮 %
#define HOST_CH_UFF_L           (6)     // 前馈分量 左轮 %
#define HOST_CH_UFF_R           (7)     // 前馈分量 右轮 %
#define HOST_CH_Z_L             (8)     // 积分状态 左轮 rps*s
#define HOST_CH_Z_R             (9)     // 积分状态 右轮 rps*s
#define HOST_CH_POS_L           (10)    // 带符号累计计数 左轮 cnt
#define HOST_CH_POS_R           (11)    // 带符号累计计数 右轮 cnt
#define HOST_CH_ONLINE          (12)    // 1 = 链路活着
#define HOST_CH_RX_AGE_MS       (13)    // 距上次收到下行帧 ms（从未收到 = -1）
#define HOST_CH_EN_BITS         (14)    // 使能位图 bit1=电机 bit2=总使能
#define HOST_CH_SAFETY          (15)    // 保护状态位图（0 = 一切正常）；★ 近似映射，见 HOST_SAFETY_*

// --- 转向(16~20) ---
#define HOST_CH_STEER_TGT       (16)    // ★ 实际施加的目标转角 deg（软限位夹紧 + 斜坡之后）
#define HOST_CH_STEER_DEG       (17)    // 实测转角 deg（向右为正）
#define HOST_CH_STEER_U         (18)    // 转向电机输出 duty（±10000）
#define HOST_CH_STEER_EN        (19)    // 转向内环使能 0/1
#define HOST_CH_STEER_ST        (20)    // 转向状态 bit0 自检通过 / bit1 反馈失效 / bit2 顶死（我方位2恒 0）

// --- 遥控(21~24) ---
#define HOST_CH_RC_ENG          (21)    // 接管状态 0 OFF/1 STANDBY/2 ON/3 FAILSAFE/4 ESTOP
#define HOST_CH_RC_CH1          (22)    // CH1 原始值（方向盘，回中≈992）
#define HOST_CH_RC_CH2          (23)    // CH2 原始值（油门）
#define HOST_CH_RC_FRAMES       (24)    // 累计接收机帧数（不涨 = 接收机没在发）

// --- 增益回读(25~26) ---
#define HOST_CH_K_V             (25)    // 本拍实际生效的比例增益 k_v（取左轮，两轮同源整定）
#define HOST_CH_K_Z             (26)    // 本拍实际生效的积分增益 k_z

// --- 惯导(27~32) ---
#define HOST_CH_YAW             (27)    // 累积航向 deg（可超 ±360）
#define HOST_CH_GZ              (28)    // 陀螺 Z 角速度 dps（去零偏后）
#define HOST_CH_INS_X           (29)    // 航位推算 x m（向右为正）
#define HOST_CH_INS_Y           (30)    // 航位推算 y m（向前为正）
#define HOST_CH_INS_DIST        (31)    // 累计里程 m（只累加绝对值，倒车也增）
#define HOST_CH_INS_FLAGS       (32)    // 惯导状态位图（INS_ST_*）

// --- 路表/回放(33~35) ---
#define HOST_CH_RT_CNT          (33)    // 路表点数（当前 RAM 里的表）
#define HOST_CH_RT_ST           (34)    // 路表状态位 | (槽号 << 8)
#define HOST_CH_PB_ELAT         (35)    // 回放横向误差 m（正 = 车在参考线右侧）

// --- 航向测试(36~37) ---
#define HOST_CH_TAR_YAW         (36)    // ★ 本拍目标航向 deg（航向外环的输入）
#define HOST_CH_HT_SEG          (37)    // 航向测试段号 —— ★ 我方无此模块（head_test 未迁移），固定 0，非等价

// --- 诊断三件套(38~40) ---
#define HOST_CH_TICK_MS         (38)    // 车端上电毫秒数（Sys_Tick_Ms）
#define HOST_CH_INS_FRZ         (39)    // ZUPT 冻结期间被丢弃的转角 deg（累计，只读不清）
#define HOST_CH_INS_STATIC      (40)    // 1 = 本拍被判"静止"（航向正在被冻结）

/*===================================================================================================================
 *  ★ SAFETY 位图（HOST_CH_SAFETY）—— **近似映射，非等价**
 *
 *  队友的 spd_safety 有一套完整判据；本工程只做"把已有的量映射成同名的位"，
 *  **不新增任何安全判据**。因此下面几位的含义只是"我方有依据的近似"：
 *      0x0001 上电以来没收到过下行帧      !host_ever_rx
 *      0x0002 ★ 固定 0 —— 我方无对应判据（原义与队友不同，宁可不置）
 *      0x0004 失联（CMD 模式心跳超时锁存） Host_Lost
 *      0x0008 主循环心跳丢失              Motor_Crtl_HB_Age_Ms() > MOTOR_CRTL_HB_TIMEOUT_MS
 *      0x0010 ★ 固定 0 —— 我方无对应判据
 *      0x0020 本拍被限幅                  Motor_LR_Crtl.sat || Motor_RR_Crtl.sat
 *      0x0040 遥控接管中                  RC_ENGAGE_ON == RC_Engage
 *      0x0080 遥控急停                    RC_ENGAGE_ESTOP == RC_Engage
 *  其余位一律 0。上位机只按"0 = 一切正常"使用，逐位含义请对照本表，**不要当成队友那套**。
 *=================================================================================================================*/
#define HOST_SAFETY_NEVER_RX    (0x0001)    // 上电以来没收到过下行帧
#define HOST_SAFETY_RESV_0002   (0x0002)    // ★ 固定 0（我方无对应判据）
#define HOST_SAFETY_LOST        (0x0004)    // 失联（Host_Lost 锁存）
#define HOST_SAFETY_HB_LOST     (0x0008)    // 主循环心跳丢失
#define HOST_SAFETY_RESV_0010   (0x0010)    // ★ 固定 0（我方无对应判据）
#define HOST_SAFETY_SAT         (0x0020)    // 本拍被限幅
#define HOST_SAFETY_RC_ON       (0x0040)    // 遥控接管中
#define HOST_SAFETY_RC_ESTOP    (0x0080)    // 遥控急停

/*===================================================================================================================
 *  ★ 路表状态位（HOST_CH_RT_ST）—— 低 8 位状态位 + bit8..11 槽号
 *      0x0001 录制中 / 0x0002 FULL / 0x0004 有表 / 0x0008 写盘中 /
 *      0x0010 写失败 / 0x0020 回放中 / 0x0040 表来自 Flash / 0x0080 固定 0
 *      bit8..11 = (slot & 0x0F)
 *=================================================================================================================*/
#define HOST_RT_ST_RUNNING      (0x0001)
#define HOST_RT_ST_FULL         (0x0002)
#define HOST_RT_ST_HAS          (0x0004)
#define HOST_RT_ST_SAVING       (0x0008)
#define HOST_RT_ST_SAVE_FAIL    (0x0010)
#define HOST_RT_ST_PLAYBACK     (0x0020)
#define HOST_RT_ST_FROM_FLASH   (0x0040)
#define HOST_RT_ST_RESV_0080    (0x0080)    // 固定 0

/*===================================================================================================================
 *  ★ 标定精简档案（18 路）—— 下标严格照队友 motor_link.h 的 CT_*
 *
 *  只在"实车落地标定"时用，由参数 `tel_sel = 1` 切进来：18 路 = 93 B/帧，
 *  同样的链路能把周期压到 5~10 ms（全量 41 路是 185 B）。
 *
 *  ⚠ 下标就是协议：上位机的「标定·精简」档案必须与这张表逐项对齐。
 *  ⚠ 来源与全量档**完全一致**，只是挑出这 18 项（Host_Crtl.c 的 host_calib_from_full 表）。
 *  ⚠ 预留位(15) 先留着：以后要加"电流 / 母线电压 / 舵机温度"这类就往后放，不要插队。
 *=================================================================================================================*/
#define HOST_CT_COUNT           (18)

#define HOST_CT_TARGET_L        (0)     // 速度环目标转速 左轮 rps
#define HOST_CT_V_L             (1)     // 速度环实际转速 左轮 rps
#define HOST_CT_U_L             (2)     // 左电机实际 PWM %（带符号）
#define HOST_CT_TARGET_R        (3)     // 速度环目标转速 右轮 rps
#define HOST_CT_V_R             (4)     // 速度环实际转速 右轮 rps
#define HOST_CT_U_R             (5)     // 右电机实际 PWM %
#define HOST_CT_STEER_TGT       (6)     // ★ 舵机角度环实际施加的目标角 deg（不是上位机写的命令）
#define HOST_CT_STEER_DELTA     (7)     // 舵机角度环实际角 deg
#define HOST_CT_STEER_U         (8)     // 舵机 PWM（duty，-10000~10000）
#define HOST_CT_ONLINE          (9)     // 链路在线 0/1
#define HOST_CT_ENABLE          (10)    // 使能位图
#define HOST_CT_SAFETY          (11)    // 保护位图（0x40 = 本地(遥控)接管中）
#define HOST_CT_STEER_EN        (12)    // 转向内环使能 0/1
#define HOST_CT_STEER_ST        (13)    // 转向状态：bit0 自检通过 / bit1 反馈失效 / bit2 顶死
#define HOST_CT_OL_EN           (14)    // 开环模式 0/1 —— ★ 我方无开环档，固定发 0（非等价）
#define HOST_CT_RESV0           (15)    // 预留位（扩展用，别占用）—— 固定发 0
#define HOST_CT_K_V             (16)    // 本拍实际生效的比例增益 k_v
#define HOST_CT_K_Z             (17)    // 本拍实际生效的积分增益 k_z

// `tel_sel` 的取值（参数表 ID 15，见下面 HOST_PARAM_TABLE 与 Param_Storage.h 的 TEL_SEL_IDX）
#define HOST_TEL_SEL_FULL       (0.0f)  // 0 = 全量 41 路
#define HOST_TEL_SEL_CALIB      (1.0f)  // 1 = 标定精简 18 路
#define HOST_TEL_SEL_THRESH     (0.5f)  // ≥ 它就算"标定档"（参数是连续浮点，先二值化再用）

/**********************************************************/
/*----------------------------------------[E] 遥测通道表 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 参数表 [S]（X-Macro：一份定义，两处展开）------------*/
/**********************************************************/

// 上位机通过 GET_TABLE 拉取元数据后**自动长出滑条** —— 加一个参数只要加一行，
// 上位机不用改代码、不用重编译。
//
// 字段: ID, 英文名, "组名|中文标签", 下限, 上限, 默认值, 单位, 滑条步长, 变量地址
//
// ⚠ ID 一旦定下就**不要插队**（新参数用新 ID）—— 否则上位机本地存档里保存的值会错位。
// ⚠ 滑条步长只供上位机交互用；车端收到有限浮点值后**只做量程钳位，不按步长取整**。
//    （队友 2026-10-14 才修掉那个"接收端又按步长取整一次"的 bug ——
//      后果是"文件名/滑条上的数"不等于"车上真正生效的数"，历史实验数据被污染。）
// ⚠ 上限/下限/默认值必须与 Param_Storage.c 的 DEFAULT_PARAMS 对得上，
//    否则上位机面板上显示的"默认值"是假的。
//
// ★ 来源是 `param_cache[]`（见 Param_Storage.h），**ID 就用缓存的索引**：
//   SET_PARAM 写完之后调 Flash_SyncTo_Param() 推到实际控制器 —— 那是本工程既定的唯一同步点。
//   所以这里**不要**直接指向 Motor_LR_Crtl.k_v 之类的运行期副本，两处默认值会打架。
//
// ★ 最后一项 tel_sel 是**遥测档案选择**（0 = 全量 41 路 / 1 = 标定 18 路）：
//   它不经过 Flash_SyncTo_Param（没有对应的控制器副本），由 Host_Crtl.c 每帧直接读 param_cache。
//   上位机拖这个滑条就能让车端切档案，切完 COUNT 立刻变，无需重启。
#define HOST_PARAM_TABLE(X)                                                                                                   \
    X( 0, "inv_K_l", "后轮L|前馈 1/K",    0.000f,     0.500f,    0.1548f, "-",    0.001f, &param_cache[MOTOR_LR_INV_K_IDX])      \
    X( 1, "u0_l",    "后轮L|摩擦截距 u0", 0.000f,     0.300f,    0.057f,  "-",    0.001f, &param_cache[MOTOR_LR_U0_IDX])         \
    X( 2, "k_v_l",   "后轮L|比例 k_v",    0.000f,     2.000f,    0.164f,  "-",    0.005f, &param_cache[MOTOR_LR_KV_IDX])         \
    X( 3, "k_z_l",   "后轮L|积分 k_z",    0.000f,    20.000f,    0.512f,  "-",    0.010f, &param_cache[MOTOR_LR_KZ_IDX])         \
    X( 4, "inv_K_r", "后轮R|前馈 1/K",    0.000f,     0.500f,    0.1548f, "-",    0.001f, &param_cache[MOTOR_RR_INV_K_IDX])      \
    X( 5, "u0_r",    "后轮R|摩擦截距 u0", 0.000f,     0.300f,    0.057f,  "-",    0.001f, &param_cache[MOTOR_RR_U0_IDX])         \
    X( 6, "k_v_r",   "后轮R|比例 k_v",    0.000f,     2.000f,    0.164f,  "-",    0.005f, &param_cache[MOTOR_RR_KV_IDX])         \
    X( 7, "k_z_r",   "后轮R|积分 k_z",    0.000f,    20.000f,    0.512f,  "-",    0.010f, &param_cache[MOTOR_RR_KZ_IDX])         \
    X( 8, "kp_s",    "舵机|比例 Kp",      0.000f,  1000.000f,  400.000f,  "-",    1.000f, &param_cache[SERVO_KP_IDX])            \
    X( 9, "u0_s",    "舵机|摩擦前馈 U0",  0.000f, 10000.000f, 1700.000f,  "-",   10.000f, &param_cache[SERVO_U0_IDX])       \
    X(10, "ki_s",    "舵机|积分 Ki",      0.000f,  1000.000f,   25.000f,  "-",    1.000f, &param_cache[SERVO_KI_IDX])            \
    X(11, "rt_ds",   "路表|采样点距",      0.020f,    0.200f,    0.050f,  "m",    0.010f, &param_cache[ROUTE_DS_IDX])            \
    X(12, "pb_v",    "回放|车速",          0.000f,    2.000f,    0.000f,  "m/s",  0.010f, &param_cache[PB_V_IDX])               \
    X(13, "pb_ld",   "回放|前视距离",      0.200f,    2.000f,    0.800f,  "m",    0.010f, &param_cache[PB_LD_IDX])             \
    X(14, "pb_smode","回放|转向通道0航向1直给", 0.000f,   1.000f,    0.000f, "-",    1.000f, &param_cache[PB_SMODE_IDX])        \
    X(15, "tel_sel", "遥测|档案 0全量41 1标定18", 0.000f, 1.000f,   0.000f, "-",    1.000f, &param_cache[TEL_SEL_IDX])         \
    X(16, "rt_ept",  "路表|停止时补终点1开0关",  0.000f,   1.000f,   1.000f, "-",    1.000f, &param_cache[ROUTE_END_PT_IDX])    \
    X(17, "pb_L",    "回放|有效轴距",      0.400f,    1.200f,    0.660f,  "m",    0.010f, &param_cache[ROUTE_PB_L_IDX])       \
    X(18, "pb_ve",   "回放|速度剖面0关1开", 0.000f,   1.000f,    0.000f,  "-",    1.000f, &param_cache[PB_VE_IDX])           \
    X(19, "pb_alat", "回放|弯道横向加速度", 0.200f,  10.000f,    2.000f,  "m/s2", 0.100f, &param_cache[PB_ALAT_IDX])         \
    X(20, "pb_vmin", "回放|限速地板",      0.000f,   1.000f,    0.300f,  "m/s",  0.050f, &param_cache[PB_VMIN_IDX])         \
    X(21, "pb_kh",   "回放|手工k 0自动2L/ld", 0.000f, 8.000f,   0.000f,  "-",    0.050f, &param_cache[PB_KH_IDX])

/**********************************************************/
/*----------------------------------------[E] 参数表 [E]--*/
/**********************************************************/


/**********************************************************/
/*[S] 连接状态 [S]----------------------------------------*/
/**********************************************************/

// ★ 判据只用"收到下行帧"这一个信号 —— 最权威：
//   模块把包收下了没有、发出去没有，车端都看不见；只有真收到上位机的帧才能证明"对面在"。
//   ⚠ 因此上位机必须定期发点什么。KartHost 会发 PING，本模块回 ACK。
//     若上位机只收不发，本模块会一直停在 OFFLINE（这不算 bug，是判据本身的含义）。
// ★★ 2026-10-17 起这个三态**只给遥测通道 HOST_CH_ONLINE 用**，屏幕的 LINK 行不再用它：
//   屏幕要区分"UP 模式在发"和"CMD 模式心跳正常"，还得显示锁存，见 Host_Crtl.c 的 host_link_text()。
//   两者故意不同源：遥测通道的语义必须稳定（上位机与历史 CSV 都依赖它），
//   而屏幕上给人看的是"现在处于哪个阶段"。
#define HOST_ST_IDLE            (0)     // 空闲：链路未建立（还没连 WiFi / Socket）
#define HOST_ST_ACTIVE          (1)     // 进行中：链路已建，且最近 HOST_ALIVE_MS 内收到过上位机的帧
#define HOST_ST_OFFLINE         (2)     // 掉线：链路已建，但收不到上位机的帧（从未收到 / 已超时）

// 错误码（Host_Crtl_Connect 的返回值，也是 Host_Crtl_Err 的内容）
#define HOST_OK                 (0)     // 成功
#define HOST_ERR_WIFI           (1)     // WiFi 连接失败
#define HOST_ERR_SOCKET         (2)     // Socket 建链失败
#define HOST_ERR_NOT_INIT       (3)     // 还没连过（上电默认）；Host_Crtl_Connect 返回它 = 用户取消了连接

// SET_ENABLE 的使能位（与上位机 EnableBits 一致）
#define HOST_EN_STEER           (0x0001)    // bit0 舵机
#define HOST_EN_MOTOR           (0x0002)    // bit1 后轮
#define HOST_EN_MASTER          (0x0004)    // bit2 总使能（本工程暂未使用）
#define HOST_EN_TELEMETRY       (0x0008)    // bit3 遥测开关（本工程暂未使用，收到即忽略）

/**********************************************************/
/*----------------------------------------[E] 连接状态 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 页面状态 [S]----------------------------------------*/
/**********************************************************/

// ★ 三个开关的实际值。**写它们请走 Host_Crtl_Set_Mode / Set_Link / Lost_Clear**，
//   直接赋值会绕过"Link = ON 不许改 Mode""关 Link 要清驱动"这些约束，
//   表现是"切了模式但车还在按老模式跑"这种极难查的问题。
//   （读它们随便读 —— 与 RC_Engage 同口径，都是 volatile 的对外量）
extern volatile uint8 Host_Mode;        // HOST_MODE_UP / HOST_MODE_CMD
extern volatile uint8 Host_Link;        // HOST_LINK_OFF / HOST_LINK_ON
extern volatile uint8 Host_Lost;        // 1 = CMD 模式心跳超时已锁存（等人工复位）

/**********************************************************/
/*----------------------------------------[E] 页面状态 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 接口 [S]--------------------------------------------*/
/**********************************************************/

// 上电调用一次（Menu.c 的 Peripheral_Init）。只重置状态，**不阻塞**
//（连 WiFi 最长十几秒，绝不能放在这里）
void    Host_Crtl_Init       (void);

// 连接：**已连则复用**（在调试页连过就直接用，不再重复 init）；**未连则弹 WiFi 选择界面**
// （三级界面，[WIFI]-SPI，上下键选热点、确认键连、BACK 取消）。
// ★ 会阻塞（未连时 WiFi 最长约 10s、Socket 更长）。调用者**只有 Host_Crtl_Page()**：
//   **进门自动调一次**（仅当未连）。页内已无重连键，不要在其它循环里无脑调它。
// 返回 HOST_OK / HOST_ERR_WIFI / HOST_ERR_SOCKET / HOST_ERR_NOT_INIT（用户在选择界面取消）
uint8   Host_Crtl_Connect    (void);

// ★★ 全工程统一的 WiFi 选择连接界面（三级界面，[WIFI]-SPI）：上下键选热点、确认键连、BACK 取消。
//   两个入口共用它 —— ① `Host_Crtl_Page()` 进门未连时自动弹 ② **调试页第 1 项 WIFI-SPI**
//   （`Debug.c` 里原来那份独立的界面与候选表已删除，那个菜单项现在直接调本函数）。
//   第三个入口：Route 页的 HOST 项（选上就连 + 开上行；选 SKIP 就关上行）。
//   ★ 会阻塞（WiFi 最长约 10s、Socket 更长），只能在按键处调用。
//   ★ 连上后**直接返回**且不清屏 —— 调用者必须自己把界面整屏重画（`ips200_clear()` + UI）。
//   ★ 列表末尾固定有一项 SKIP = 不连接；已连时选它同时 `Host_Crtl_Set_Link(OFF)` 关上行。
//   返回 HOST_OK / HOST_ERR_NOT_INIT（BACK 取消，或选了 SKIP）
uint8   Host_Crtl_WiFi_Select (void);

// ★★ 链路层：收命令 + 发遥测。自带限频与"模块忙就跳过"，可安全放在任意 while(1) 里。
// ★ 两道门：`wifi_spi_inited`（链路没建）与 `Host_Link`（链路没开）—— 任一不满足就是
//   纯空操作，**上行下行一起停**（不是只停上行）。遥控不受它影响。
// ★ 调用点只有 Host_Crtl_Page() 一处 —— 主菜单待机时整车静默（链路仍连着，重进免重连）。
void    Host_Crtl_Task       (void);

// ★★ 上位机链路开关（只管 WiFi）。OFF -> 停收停发 + 丢使能位图 + 清心跳锁存；
//    **不收回遥控**，且只在遥控未接管时才清驱动侧。
//    ON  -> 立刻补发参数表与 EVENT（上位机的参数面板靠它长出来），并给心跳门 HOST_ALIVE_MS 宽限。
//    ★ 幂等：重复开/关都不报错，关的时候总是会执行链路侧的清理（不因为"已经是 OFF"就跳过）。
void    Host_Crtl_Set_Link   (uint8 on);

// ★★ 工作模式切换。**Link = ON 时调用无效**（直接 return，不报错）——理由见文件头。
//    切到 UP 时会把使能位图作废（CMD 的使能不该跟着带进 UP），并撤销 Host_Lost 锁存；
//    驱动侧只在遥控未接管时才清。所以可在 Link = OFF + 遥控开车时安全地预设模式。
//    ⚠ UP -> CMD 时若 CH6 = 0，遥控会立刻让出驾驶权（车停下或交给上位机）—— 要接续先拨 CH6。
void    Host_Crtl_Set_Mode   (uint8 mode);

// ★★ 人工复位心跳锁存（锁存态的确认键调它）。**不保证能开走**：
//    复位后下一圈如果心跳仍然超时，会立刻重新锁存 —— 这是正确行为，不是 bug。
void    Host_Crtl_Lost_Clear (void);

// 整页退出：Link 归零、模式回 UP、锁存与遥控全清、驱动侧失能（与 Remote_Crtl_Release 同口径）
void    Host_Crtl_Release    (void);

// 主菜单里的上位机页面。★ 进入时若链路未建立，会先弹 WiFi 选择界面（"进门先连"）——
//   在那里按 BACK 取消 = 直接退回主菜单，**不会**进入本页
int     Host_Crtl_Page       (void);

// 状态查询（遥测通道 HOST_CH_ONLINE 用它；**屏幕的 LINK 行不用**，理由见 HOST_ST_* 段）
uint8   Host_Crtl_State      (void);        // 取 HOST_ST_IDLE / ACTIVE / OFFLINE
uint8   Host_Crtl_Err        (void);        // 最近一次 Connect 的错误码
uint32  Host_Crtl_Tx_Count   (void);        // 已成功交给模块的遥测帧数
uint32  Host_Crtl_Rx_Count   (void);        // 已解析通过的命令帧数
uint32  Host_Crtl_Crc_Err    (void);        // CRC 校验失败次数（很大 = 线序/波特率/对面协议不对）

/**********************************************************/
/*----------------------------------------[E] 接口 [E]----*/
/**********************************************************/


#endif
