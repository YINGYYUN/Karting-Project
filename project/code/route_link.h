/*********************************************************************************************************************
 * 路表 —— 串口"灌表"链路（PC → 车）
 *
 * 做什么
 *   把 PC 做好的**路表镜像**（36 B 头 + N×16 B 记录，格式见 ins_route.h）通过调试串口写进车里
 *   某个 Flash 槽并选为当前表；反向也能把车上的槽读回 PC（0x09）。PC 端见 KartHostLite 的 RouteLink.cs。
 *   为什么不用 WiFi：下行帧是**定长 15 B** 的（HOST_DN_FRAME_LEN），一张表 32 KB 要改下行协议
 *   再自己做分片重传；串口是自己的字节流，一层都没有。
 *
 * 走哪根线
 *   DEBUG_UART_INDEX（UART_0，P00_1/P00_0，即 DAP 的虚拟串口），波特率 115200。
 *   ⚠ 与 printf 共用一根线：文本字节插进帧中间，PC 就解不出那一帧。
 *     ⇒ 传输期间把日志静音（zf_common_debug.c 的 debug_uart_mute），COMMIT/ABORT/超时再放开。
 *   ⚠ UART_0 的**接收**由本模块独家消费（uart0_isr 调 route_link_uart_irq），
 *     不再走 debug 环形缓冲 —— 两个消费者抢 uart_query_byte() 必然丢字节。
 *
 * 帧格式（小端）
 *   A5 5A | type(1) | len(2) | payload[len] | crc16(2)
 *   crc16 = CRC16-MODBUS（初值 0xFFFF / 反射多项式 0xA001），覆盖 type|len|payload。
 *   同步头用 A5 5A：日志全是 ASCII，撞不出 0xA5 ⇒ 静音前后都不会误同步。
 *
 * 命令表
 *   0x01 PING        无                             → 0x81 PONG
 *   0x02 DIR_REQ     无                             → 0x82 DIR_DATA
 *   0x03 LOAD_BEGIN  slot(1) total_bytes(4)         → 0x85 ACK
 *   0x04 LOAD_DATA   offset(4) len(2) bytes[len]    → 0x85 ACK
 *   0x05 LOAD_COMMIT 无                             → 0x85 ACK；写完再发 0x86 LOAD_DONE
 *   0x06 LOAD_ABORT  无                             → 0x85 ACK
 *   0x07 SLOT_SEL    slot(1) persist(1) load(1)     → 0x85 ACK
 *   0x08 SLOT_LOAD   slot(1)                        → 0x85 ACK
 *   0x09 SLOT_READ   slot(1) offset(4) n(2)         → 0x87 SLOT_DATA（拉模式，车端无状态）
 *   各载荷的字段顺序见 route_link.c 的 rl_reply_pong / rl_reply_dir / rl_load_* / rl_slot_*。
 *
 * ★ 槽的纪律：只允许写 1..INS_RT_SLOT_COUNT-1。槽 0 是"手推录制"的家，协议层就拒。
 *
 * 线程约定
 *   route_link_uart_irq()  只在 UART_0 接收中断里调（只搬字节，不解析）
 *   route_link_poll()      只在主循环调（会写 Flash、会阻塞发串口）
 ********************************************************************************************************************/

#ifndef _ROUTE_LINK_H_
#define _ROUTE_LINK_H_

#include "zf_common_headfile.h"

/*===================================================================================================================
 *  协议常量（PC 端 RouteLink.cs 必须与此逐一对应）
 *=================================================================================================================*/
#define RL_SYNC0                (0xA5u)
#define RL_SYNC1                (0x5Au)

#define RL_T_PING               (0x01u)
#define RL_T_DIR_REQ            (0x02u)
#define RL_T_LOAD_BEGIN         (0x03u)
#define RL_T_LOAD_DATA          (0x04u)
#define RL_T_LOAD_COMMIT        (0x05u)
#define RL_T_LOAD_ABORT         (0x06u)
#define RL_T_SLOT_SEL           (0x07u)
#define RL_T_SLOT_LOAD          (0x08u)
#define RL_T_SLOT_READ          (0x09u)     /* slot(1) offset(4) n(2) → 0x87 SLOT_DATA */

#define RL_T_PONG               (0x81u)
#define RL_T_DIR_DATA           (0x82u)
#define RL_T_ACK                (0x85u)
#define RL_T_LOAD_DONE          (0x86u)
#define RL_T_SLOT_DATA          (0x87u)     /* slot(1) offset(4) n(2) bytes[n]（上传用）*/

#define RL_PROTO_VER            (2u)        /* 2 = PONG 末尾带 caps(4)；1 = 只有前 8 B */

/*===================================================================================================================
 *  能力位（PONG 的 caps 字段，u32 小端）
 *
 *  给命令表加一条命令，最容易出的问题不是写错，而是"上位机发了、车端不认识" —— 车端只会回一个
 *  笼统的 RL_ERR_STATE（状态不允许），现场看不出是"固件旧"。有了 caps，上位机在 PING 那一刻
 *  就知道车端会什么，缺哪项能直接点名。
 *  ⚠ 加新命令必须同步在这里加一位，否则又退化成"状态不允许"。
 *=================================================================================================================*/
#define RL_CAP_DIR              (1u << 0)   /* 0x02/0x82  槽位一览              */
#define RL_CAP_LOAD             (1u << 1)   /* 0x03..0x06 灌表（PC→车）          */
#define RL_CAP_SLOT_SEL         (1u << 2)   /* 0x07/0x08  选当前槽 / 载入 RAM    */
#define RL_CAP_SLOT_READ        (1u << 3)   /* 0x09/0x87  读槽（车→PC）          */
#define RL_CAP_STEP             (1u << 4)   /* 保留位（本工程未实现，不上报）     */
#define RL_CAP_ALL              (RL_CAP_DIR | RL_CAP_LOAD | RL_CAP_SLOT_SEL | RL_CAP_SLOT_READ)
#define RL_PAYLOAD_MAX          (768u)      /* 单帧 payload 上限（含 LOAD_DATA 的 6 B 头）*/
#define RL_CHUNK_MAX            (512u)      /* 一帧最多搬多少"镜像字节"                    */
#define RL_RX_RING_SZ           (4096u)
#define RL_LOAD_TIMEOUT_MS      (3000u)     /* 传了一半没下文就自动放弃                    */

/* ACK 的 result 码（PC 侧的判据，别随便改数值）*/
#define RL_OK                   (0u)
#define RL_ERR_SLOT             (1u)        /* 槽号越界 / 试图写槽 0                        */
#define RL_ERR_TOO_BIG          (2u)        /* total_bytes 超过单槽上限                     */
#define RL_ERR_OFFSET           (3u)        /* LOAD_DATA 的 offset 不连续（丢帧了）          */
#define RL_ERR_IMAGE            (4u)        /* 镜像 CRC/magic/布局不过                       */
#define RL_ERR_NOSTART          (5u)        /* 没先 LOAD_BEGIN 就发 DATA                    */
#define RL_ERR_BUSY             (6u)        /* 正在写 Flash，先别来                          */
#define RL_ERR_STATE            (7u)        /* 状态不允许（例如没在传输却 COMMIT）           */

/*===================================================================================================================
 *  接口
 *=================================================================================================================*/
void    route_link_init      (void);    /* 上电一次：清状态 + 放开日志门闩                    */

/* UART_0 接收中断里调：把库缓冲里的字节全部搬进本模块环形缓冲（只搬，不解析）。
   ⚠ 它是 UART_0 接收字节的唯一消费者，不要再同时调 debug_interrupr_handler()。*/
void    route_link_uart_irq  (void);

/* 只投一个字节进环形缓冲（route_link_uart_irq 的内部动作，一般不用直接调）。*/
void    route_link_rx_byte   (uint8 b);

/* 主循环每圈调一次（幂等）：拆帧 + 回 ACK。 */
void    route_link_poll      (void);

/* ★ 本工程的"主循环钩子" = route_link_poll() + ins_route_flash_poll()（推进分步写盘）。
   唯一调用点是 Param_Page.c 的 param_page_loop()（参数页的 while(1)）。
   由此定下一条约定：**灌表时车端必须停在参数页（主菜单 → Param）**；
   停在别的页面时收不到帧、也推进不了写盘（数据不丢，回到参数页继续）。
   详细理由见 route_link.c 里这个函数的注释。*/
void    route_link_tick      (void);

/* 诊断 */
uint8   route_link_active    (void);    /* 1 = 正在收 / 正在写盘（此刻日志被静音）*/
uint32  route_link_rx_frames (void);    /* 收到并 CRC 通过的帧数（累计）*/
uint32  route_link_rx_bad    (void);    /* CRC 不过 / 丢弃的字节数*/
uint32  route_link_last_ms   (void);    /* 最近一次收到合法帧的时刻（Sys_Tick_Ms）*/

#endif
