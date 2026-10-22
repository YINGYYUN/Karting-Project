/*******************************************************************************
上位机对接（PC 端 = 队友的 KartHost）— 链路层 + 页面 + WiFi 选择界面
协议、帧格式、解析纪律与队友车端 motor_link.c 逐字节对齐；
分层说明、调用纪律、链路来源见 Host_Crtl.h
*******************************************************************************/


#include "zf_common_headfile.h"
#include "ins_imu.h"                    // 遥测里的 yaw 直接取模块输出（不再经 Yaw_Receive 中转）
#include <float.h>


/**********************************************************/
/*[S] 参数表元数据 [S]（由 Host_Crtl.h 的 X-Macro 展开）--*/
/**********************************************************/

typedef struct
{
    uint8           id;
    const char     *name;       // 英文名
    const char     *label;      // "组名|中文标签"（上位机按 | 拆分折叠）
    float           lo;
    float           hi;
    float           def;
    const char     *unit;
    float           step;       // 滑条步长（只供上位机交互，车端不按它取整）
    volatile float *ptr;        // 参数被中断和主循环共享，必须 volatile
} Host_Param_t;

#define HOST_DEF_META(id, name, label, lo, hi, def, unit, step, ptr) \
    { (uint8)(id), name, label, lo, hi, def, unit, step, ptr },

static const Host_Param_t host_param_meta[] = { HOST_PARAM_TABLE(HOST_DEF_META) };
#undef HOST_DEF_META

#define HOST_PARAM_COUNT    ((uint8)(sizeof(host_param_meta) / sizeof(host_param_meta[0])))

/**********************************************************/
/*----------------------------------------[E] 参数表元数据 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 协议常量 [S]----------------------------------------*/
/**********************************************************/

// 上行 MAGIC = A5 5A，下行 MAGIC = 5A A5（反序）——这样环回时不会把自己的下行帧当成上行遥测解析
#define HOST_MAGIC_UP_0         (0xA5)
#define HOST_MAGIC_UP_1         (0x5A)
#define HOST_MAGIC_DN_0         (0x5A)
#define HOST_MAGIC_DN_1         (0xA5)

// 帧类型
#define HOST_TYPE_TELEMETRY     (0x01)      // 车 → PC
#define HOST_TYPE_PARAM_META    (0x02)
#define HOST_TYPE_PARAM_VALUE   (0x03)
#define HOST_TYPE_ACK           (0x04)
#define HOST_TYPE_NAK           (0x05)
#define HOST_TYPE_EVENT         (0x06)
#define HOST_TYPE_COMMAND       (0x01)      // PC → 车（与上行 TYPE 值相同，靠 MAGIC 方向区分）

// 下行命令码（与上位机 Cmd 枚举一致）
#define HOST_CMD_PING           (0x01)
#define HOST_CMD_GET_PARAM      (0x02)
#define HOST_CMD_SET_PARAM      (0x03)
#define HOST_CMD_SET_ENABLE     (0x04)
#define HOST_CMD_SAVE           (0x05)
#define HOST_CMD_LOAD           (0x06)
#define HOST_CMD_GET_TABLE      (0x07)

// 下行帧固定长度：MAGIC(2) + TYPE(1) + LEN(2) + CMD(1) + ID(1) + VALUE(4) + TOKEN(2) + CRC(2) = 15
#define HOST_DN_FRAME_LEN       (15)
#define HOST_DN_PAYLOAD         (8)         // 下行 LEN 恒定 8

// CRC16-MODBUS 覆盖范围：从 TYPE 起、到 PAYLOAD 结束（不含 MAGIC，也不含 CRC 自己）
#define HOST_CRC_SPAN           (3)         // TYPE(1) + LEN(2)

/**********************************************************/
/*----------------------------------------[E] 协议常量 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 对外数据 [S]----------------------------------------*/
/**********************************************************/

// 三个开关的实体（声明在 Host_Crtl.h）。**只由 Host_Crtl_Set_Mode / Set_Link / Lost_Clear 写**，
// 别在页面里直接赋值 —— 那会绕过"Link = ON 不许改 Mode""关 Link 要清驱动"这些约束。
volatile uint8 Host_Mode = HOST_MODE_UP;
volatile uint8 Host_Link = HOST_LINK_OFF;
volatile uint8 Host_Lost = 0;

/**********************************************************/
/*----------------------------------------[E] 对外数据 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 内部状态 [S]----------------------------------------*/
/**********************************************************/

// Sys_Tick_Ms 是 1ms 全局计时（PIT_CH10），永不复位 → 用它算周期，
// 所有差值都写成 (uint32)(now - last)，天然抗回绕，不需要清零。
static uint32   host_tx_ms      = 0;        // 上次发遥测的时刻
static uint32   host_rx_ms      = 0;        // 上次尝试收命令的时刻
static uint32   host_alive_ms   = 0;        // 最近一次收到下行帧的时刻
static uint8    host_ever_rx    = 0;        // 1 = 上电以来至少收到过一帧下行
static uint16   host_tx_seq     = 0;        // 遥测序号（上位机按它算丢包）
static uint16   host_enable     = 0;        // 上位机下发的使能位图
static uint32   host_tx_count   = 0;        // 已成功交给模块的遥测帧数
static uint32   host_rx_count   = 0;        // 已解析通过的命令帧数
static uint32   host_crc_err    = 0;        // CRC 校验失败次数
static uint8    host_err        = HOST_ERR_NOT_INIT;

// "这一圈谁在写驱动侧"（取值见下方"驾驶权仲裁"段的 HOST_AUTH_*）。
// 只在本文件内用 —— 通道表已冻结，不往上位机送。放在这里是因为 Host_Crtl_Release 也要清它。
#define HOST_AUTH_NONE          (0)         // 没人开：UP + 遥控关 / CMD + Link 关 + CH6 = 0
#define HOST_AUTH_HOST          (1)         // 上位机在开（CMD + Link = ON）
#define HOST_AUTH_REMOTE        (2)         // 遥控在开（UP 直接给，CMD 要 CH6 = 1）
#define HOST_AUTH_STOP          (3)         // 遥控侧锁存：ESTOP / RC LOST，停车优先于一切
static uint8    Host_Auth       = HOST_AUTH_NONE;

// 库函数要 char*（不能直接传字符串字面量，否则 IAR 会报 const 转换）
static char     host_sock_type[]   = HOST_SOCK_TYPE;
static char     host_target_ip[]   = HOST_TARGET_IP;
static char     host_target_port[] = HOST_TARGET_PORT;
static char     host_local_port[]  = HOST_LOCAL_PORT;

// WiFi 候选表（由 Host_Crtl.h 的 HOST_WIFI_LIST 展开；那边的"第二份"警告要先读）
typedef struct
{
    char *ssid;
    char *pwd;
} Host_Wifi_t;

#define HOST_WIFI_ENTRY(ssid, pwd)  { ssid, pwd },
static Host_Wifi_t host_wifi_list[] = { HOST_WIFI_LIST(HOST_WIFI_ENTRY) };
#undef HOST_WIFI_ENTRY

#define HOST_WIFI_COUNT     ((uint8)(sizeof(host_wifi_list) / sizeof(host_wifi_list[0])))

// 发送缓冲。全量档整帧 = 11 + 10 + 41*4 = 185 字节，留 32 字节余量（缓冲按 41 路开，
// 标定档 18 路时只用前 93+11 字节 —— 不另开第二条缓冲，省得两处长度各错一次）。
// 单线程（只在页面/主菜单的 while(1) 里）使用，不重入 —— 所以用 static 省栈。
#define HOST_TEL_BUF_SIZE       (32 + HOST_CH_COUNT_MAX * 4)
#define HOST_FRM_BUF_SIZE       (128)       // 其它帧：参数表元数据最大 11 + 73 = 84
static uint8    host_telbuf[HOST_TEL_BUF_SIZE];
static uint8    host_frmbuf[HOST_FRM_BUF_SIZE];
static uint8    host_rxbuf[256];
static float    host_ch[HOST_CH_COUNT_MAX];

// ★★ 本帧实际要发的通道数（运行期变量，不再是编译期常量）：
//    41 = 全量档 / 18 = 标定档，每帧由参数 `tel_sel` 现算（见 host_tel_pick_count）。
//    帧头 COUNT 也跟着它变 —— 上位机按 COUNT 自动切档案，所以这个变量**必须**和
//    实际写进帧里的循环上界同源，不能一处 41 一处 18。
static uint16   host_ch_count = HOST_CH_COUNT_MAX;

// ★ 标定档 18 路 → 全量档 41 路的下标映射（下标严格照 teammate motor_link.h 的 CT_*）。
//   0xFF = 我方无对应物，固定发 0（**非等价**，不要拿别的量去凑）。
//   ⚠ 这张表和 Host_Crtl.h 的 HOST_CT_* 必须逐项对齐；顺序即协议。
static const uint8 host_calib_from_full[HOST_CT_COUNT] =
{
    HOST_CH_TARGET_L,       // CT_TARGET_L     0
    HOST_CH_V_L,            // CT_V_L          1
    HOST_CH_U_L,            // CT_U_L          2
    HOST_CH_TARGET_R,       // CT_TARGET_R     3
    HOST_CH_V_R,            // CT_V_R          4
    HOST_CH_U_R,            // CT_U_R          5
    HOST_CH_STEER_TGT,      // CT_STEER_TGT    6
    HOST_CH_STEER_DEG,      // CT_STEER_DELTA  7
    HOST_CH_STEER_U,        // CT_STEER_U      8
    HOST_CH_ONLINE,         // CT_ONLINE       9
    HOST_CH_EN_BITS,        // CT_ENABLE      10
    HOST_CH_SAFETY,         // CT_SAFETY      11
    HOST_CH_STEER_EN,       // CT_STEER_EN    12
    HOST_CH_STEER_ST,       // CT_STEER_ST    13
    0xFF,                   // CT_OL_EN       14 —— ★ 我方没有开环档，固定 0（非等价）
    0xFF,                   // CT_RESV0       15 —— 预留位，固定 0
    HOST_CH_K_V,            // CT_K_V         16
    HOST_CH_K_Z,            // CT_K_Z         17
};

// 收到的字节先攒进累加器再解析。上限 = 一次最多读 256 字节 + 不足一帧的尾巴(14)，
// 288 留足余量。解析完把尾巴搬到开头，所以它永远是"很小"的。
#define HOST_RXACC_SIZE         (288)
static uint8    host_rxacc[HOST_RXACC_SIZE];
static uint16   host_rxacc_len = 0;

/**********************************************************/
/*----------------------------------------[E] 内部状态 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 小工具 [S]（CRC16 与小端读写）----------------------*/
/**********************************************************/

//--------------------------------------
// 函数简介     CRC16-MODBUS（多项式 0xA001，初值 0xFFFF，移位异或不查表）
// 参数说明     buf 起始地址
// 参数说明     len 字节数
// 返回参数     uint16 校验值
// 使用示例     host_crc16(buf + 2, 3 + plen);
// 备注信息     与上位机 Proto.Crc16 逐位一致，不要"优化"成查表（表会占 Flash，这里不慢）
//--------------------------------------
static uint16 host_crc16 (const uint8 *buf, uint16 len)
{
    uint16 crc = 0xFFFF;
    uint8  i, j;

    for (i = 0; i < len; i ++)
    {
        crc ^= buf[i];
        for (j = 0; j < 8; j ++)
        {
            if (crc & 0x01) { crc = (uint16)((crc >> 1) ^ 0xA001); }
            else            { crc = (uint16)(crc >> 1); }
        }
    }

    return crc;
}

static uint16 host_put_u16 (uint8 *b, uint16 o, uint16 v)
{
    b[o]     = (uint8)(v & 0xFF);
    b[o + 1] = (uint8)((v >> 8) & 0xFF);
    return (uint16)(o + 2);
}

static uint16 host_put_u32 (uint8 *b, uint16 o, uint32 v)
{
    b[o]     = (uint8)(v & 0xFF);
    b[o + 1] = (uint8)((v >> 8) & 0xFF);
    b[o + 2] = (uint8)((v >> 16) & 0xFF);
    b[o + 3] = (uint8)((v >> 24) & 0xFF);
    return (uint16)(o + 4);
}

// Cortex-M7 是小端，float 直接拷内存 —— 这就是"零格式化开销"的关键（不要改成 sprintf）
static uint16 host_put_f32 (uint8 *b, uint16 o, float v)
{
    memcpy(b + o, &v, 4);
    return (uint16)(o + 4);
}

static uint16 host_get_u16 (const uint8 *b, uint16 o)
{
    return (uint16)((uint16)b[o] | ((uint16)b[o + 1] << 8));
}

static uint32 host_get_u32 (const uint8 *b, uint16 o)
{
    return ((uint32)b[o]) | (((uint32)b[o + 1]) << 8)
         | (((uint32)b[o + 2]) << 16) | (((uint32)b[o + 3]) << 24);
}

static float host_get_f32 (const uint8 *b, uint16 o)
{
    float v;

    memcpy(&v, b + o, 4);
    return v;
}

/**********************************************************/
/*----------------------------------------[E] 小工具 [E]--*/
/**********************************************************/


/**********************************************************/
/*[S] 上行发送 [S]----------------------------------------*/
/**********************************************************/

//--------------------------------------
// 函数简介     发送一个通用上行帧（TYPE + LEN + PAYLOAD + CRC + TAIL）
// 参数说明     type 帧类型
// 参数说明     payload 载荷（可为 NULL）
// 参数说明     plen 载荷长度
// 返回参数     uint8 1 = 这一帧真的交给模块了；0 = 模块忙，本帧被跳过
// 使用示例     host_send_frame(HOST_TYPE_ACK, pl, 4);
// 备注信息     ★ 返回 0 是有意义的：参数表分批发送靠它做"发不出去就重试"。
//              驱动里的 wait_idle(OTHER_TIME_OUT) 最长约 1s，初始化要靠它所以保留默认；
//              运行期就靠这个 INT 判断避免被它卡住。
//--------------------------------------
static uint8 host_send_frame (uint8 type, const uint8 *payload, uint16 plen)
{
    uint16 n = 0;

    if (0 == gpio_get_level(WIFI_SPI_INT_PIN)) { return 0; }

    host_frmbuf[n ++] = HOST_MAGIC_UP_0;
    host_frmbuf[n ++] = HOST_MAGIC_UP_1;
    host_frmbuf[n ++] = type;
    n = host_put_u16(host_frmbuf, n, plen);

    if ((plen > 0) && (NULL != payload))
    {
        memcpy(host_frmbuf + n, payload, plen);
        n = (uint16)(n + plen);
    }

    n = host_put_u16(host_frmbuf, n, host_crc16(host_frmbuf + 2, (uint16)(HOST_CRC_SPAN + plen)));

    // 帧尾 00 00 80 7F = 小端的 1.0f，与 VOFA+ JustFloat 一致 ——
    // 必要时可以先用 VOFA+ 验证链路通不通，再上 KartHost
    host_frmbuf[n ++] = 0x00;
    host_frmbuf[n ++] = 0x00;
    host_frmbuf[n ++] = 0x80;
    host_frmbuf[n ++] = 0x7F;

    if (0 != wifi_spi_send_buffer(host_frmbuf, n)) { return 0; }    // 没全部发出去

    return 1;
}

// ACK/NAK 载荷：TOKEN(2) + CMD(1) + 结果/错误码(1)
// ★ TOKEN 必须原样带回 —— 多条命令连发时，上位机靠它知道哪条生效、哪条被拒
static void host_send_ack (uint8 cmd, uint16 token)
{
    uint8 pl[4];

    pl[0] = (uint8)(token & 0xFF);
    pl[1] = (uint8)((token >> 8) & 0xFF);
    pl[2] = cmd;
    pl[3] = 0;

    host_send_frame(HOST_TYPE_ACK, pl, 4);
}

static void host_send_nak (uint8 cmd, uint16 token, uint8 err)
{
    uint8 pl[4];

    pl[0] = (uint8)(token & 0xFF);
    pl[1] = (uint8)((token >> 8) & 0xFF);
    pl[2] = cmd;
    pl[3] = err;

    host_send_frame(HOST_TYPE_NAK, pl, 4);
}

// 参数值回读：ID(1) + VALUE(4) = 5
static uint8 host_send_param_value (uint8 id, float v)
{
    uint8  pl[5];
    uint16 n = 0;

    pl[n ++] = id;
    n = host_put_f32(pl, n, v);

    return host_send_frame(HOST_TYPE_PARAM_VALUE, pl, 5);
}

static void host_send_event (const char *text)
{
    uint8 pl[64];
    uint8 len;

    pl[0] = 1;                                  // level = 1(信息)
    len = (uint8)strlen(text);
    if (len > 62) { len = 62; }
    memcpy(pl + 1, text, len);

    host_send_frame(HOST_TYPE_EVENT, pl, (uint16)(1 + len));
}

/**********************************************************/
/*----------------------------------------[E] 上行发送 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 参数表分批发送 [S]----------------------------------*/
/**********************************************************/

// ★ 为什么必须分批：host_send_frame() 遇到"模块忙"会静默跳过，
//   一口气连发 10 项一定会漏几帧，上位机于是拿到一张"混血参数表"——
//   到货的项用新名字，没到货的项还留着上位机本地的旧名字，而两边 ID 含义完全不同，
//   用户照着面板拖滑条就把值写进了不相干的参数里。
//   改成：每 5ms 发一项，发不出去就不推进 id、下一轮重试同一项。最多 10*5 = 50ms，
//   而上位机本来就是"收到第一帧后 300ms 统一重建界面"，体感没有差别。
#define HOST_PARAM_TX_GAP_MS    (5u)

static uint8    host_pt_pend = 0;       // 1 = 有一张参数表等着发
static uint8    host_pt_id   = 0;       // 下一个要发的参数下标
static uint32   host_pt_ms   = 0;       // 上次尝试发送的时刻

static void host_param_table_start (void)
{
    host_pt_id   = 0;
    host_pt_pend = 1;                   // 只"武装"序列，真正的发送交给 host_param_table_tick
}

// 由 Host_Crtl_Task() 每圈调用（自带 5ms 限频）
static void host_param_table_tick (void)
{
    uint8  pl[73];                      // ID(1)+LO(4)+HI(4)+DEF(4)+STEP(4)+UNIT(8)+NAME(16)+LABEL(32)
    uint16 n;

    if (0 == host_pt_pend) { return; }
    if ((uint32)(Sys_Tick_Ms - host_pt_ms) < HOST_PARAM_TX_GAP_MS) { return; }
    host_pt_ms = Sys_Tick_Ms;

    memset(pl, 0, sizeof(pl));
    n = 0;
    pl[n ++] = host_param_meta[host_pt_id].id;
    n = host_put_f32(pl, n, host_param_meta[host_pt_id].lo);
    n = host_put_f32(pl, n, host_param_meta[host_pt_id].hi);
    n = host_put_f32(pl, n, host_param_meta[host_pt_id].def);
    n = host_put_f32(pl, n, host_param_meta[host_pt_id].step);

    // 定长字符串：只写 cap-1 字节，末字节留 0（memset 已经补过 0）
    strncpy((char *)(pl + 17), host_param_meta[host_pt_id].unit,  7);
    strncpy((char *)(pl + 25), host_param_meta[host_pt_id].name, 15);
    strncpy((char *)(pl + 41), host_param_meta[host_pt_id].label, 31);

    if (0 == host_send_frame(HOST_TYPE_PARAM_META, pl, 73))
    {
        return;                         // 模块忙 -> 本项没发出去，下一轮重试同一个 id
    }

    // ★ 紧接着把【车端当前的数值】也报一份：META 帧里只有 lo/hi/def/step、没有"当前值"，
    //   上位机的 MergeTable 只合并名字/量程、**不更新数值** —— 少了这一帧，
    //   面板上显示的就只是它自己的本地值，用户分不清这个数是车上的还是自己上次留下的。
    if (0 == host_send_param_value(host_pt_id, *(host_param_meta[host_pt_id].ptr)))
    {
        return;                         // 值没发出去就不推进 id，下一轮连 META 一起重发
    }

    host_pt_id ++;
    if (host_pt_id >= (uint8)HOST_PARAM_COUNT) { host_pt_pend = 0; }
}

/**********************************************************/
/*----------------------------------------[E] 参数表分批发送 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 遥测 [S]（组帧 + 发送）-----------------------------*/
/**********************************************************/

//--------------------------------------
// 函数简介     合成 SAFETY 位图（HOST_CH_SAFETY）
// 参数说明     void
// 返回参数     uint16 位图（0 = 一切正常）
// 使用示例     内部调用（host_fill_channels）
// 备注信息     ★★ **只做映射，不新增任何安全判据**。逐位含义见 Host_Crtl.h 的 HOST_SAFETY_*，
//              那里写清了哪几位是"我方有依据的近似"、哪几位恒 0（我方无对应判据）。
//              所以这个值和队友的 SAFETY **不等价**，上位机只当"0 = 正常"用。
//              0x0002 / 0x0010 故意不置 —— 宁可不报，也不要拿不相干的量去凑一个假报警。
//--------------------------------------
static uint16 host_safety_bits (void)
{
    uint16 b = 0;

    if (0 == host_ever_rx)                              { b |= HOST_SAFETY_NEVER_RX; }
    if (0 != Host_Lost)                                 { b |= HOST_SAFETY_LOST; }
    if (Motor_Crtl_HB_Age_Ms() > MOTOR_CRTL_HB_TIMEOUT_MS) { b |= HOST_SAFETY_HB_LOST; }
    if ((0 != Motor_LR_Crtl.sat) || (0 != Motor_RR_Crtl.sat)) { b |= HOST_SAFETY_SAT; }
    if (RC_ENGAGE_ON == RC_Engage)                      { b |= HOST_SAFETY_RC_ON; }
    if (RC_ENGAGE_ESTOP == RC_Engage)                   { b |= HOST_SAFETY_RC_ESTOP; }

    return b;
}

//--------------------------------------
// 函数简介     合成路表状态位（HOST_CH_RT_ST）
// 参数说明     void
// 返回参数     uint16 低 8 位 = 状态位，bit8..11 = 当前槽号
// 使用示例     内部调用（host_fill_channels）
// 备注信息     定义见 Host_Crtl.h 的 HOST_RT_ST_*。bit7 恒 0。
//              队友把这个值当"位图 + 槽号"混合体用，上位机按整数显示即可。
//--------------------------------------
static uint16 host_route_status_bits (void)
{
    uint16 b = 0;
    uint8  f = ins_route_flags();

    if (0 != (f & INS_ROUTE_ST_RUNNING))  { b |= HOST_RT_ST_RUNNING; }
    if (0 != (f & INS_ROUTE_ST_FULL))     { b |= HOST_RT_ST_FULL; }
    if (ins_route_get_count() > 0)        { b |= HOST_RT_ST_HAS; }
    if (0 != ins_route_flash_busy())      { b |= HOST_RT_ST_SAVING; }
    if (2 == ins_route_flash_last_rc())   { b |= HOST_RT_ST_SAVE_FAIL; }      // 2 = 写失败
    if (0 != route_pb_active())           { b |= HOST_RT_ST_PLAYBACK; }
    if (0 != ins_route_flash_loaded())    { b |= HOST_RT_ST_FROM_FLASH; }

    b |= (uint16)(((uint16)ins_route_slot_active() & 0x000Fu) << 8);          // 槽号放 bit8..11

    return b;
}

//--------------------------------------
// 函数简介     把各模块的实时量灌进通道数组（全量 41 路）
// 参数说明     void
// 使用示例     内部调用
// 备注信息     直接从模块全局量取值，不需要 main 每圈来填 —— 少一处漏填的机会。
//              ★ 下标/单位/位义与队友 motor_link.h **逐项对齐**，
//                对照表写在 Host_Crtl.h 的"队友下标 → 我方来源"注释里（41 行）。
//              ★ 单位口径：
//                  · rps / deg / m 直接用物理量，不做缩放；
//                  · **电机 out / u_ff 我方是归一化 ±1，队友是 %，所以这里必须 ×100** ——
//                    漏了这个乘，上位机看到的输出只有 ±0.5 而不是 ±50，波形像是"没出力"；
//                  · 位置是编码器累计计数（cnt），不是米。
//--------------------------------------
static void host_fill_channels (void)
{
    // ---- 后轮速度环（0~11）----
    host_ch[HOST_CH_TARGET_L]   = Motor_LR_Crtl.target_app;     // 斜坡之后的实际目标，不是 target
    host_ch[HOST_CH_V_L]        = Motor_LR_Crtl.actual;
    host_ch[HOST_CH_U_L]        = Motor_LR_Crtl.out * 100.0f;   // ★ 归一化 → %
    host_ch[HOST_CH_TARGET_R]   = Motor_RR_Crtl.target_app;
    host_ch[HOST_CH_V_R]        = Motor_RR_Crtl.actual;
    host_ch[HOST_CH_U_R]        = Motor_RR_Crtl.out * 100.0f;   // ★ 归一化 → %
    host_ch[HOST_CH_UFF_L]      = Motor_LR_Crtl.u_ff * 100.0f;  // ★ 归一化 → %
    host_ch[HOST_CH_UFF_R]      = Motor_RR_Crtl.u_ff * 100.0f;  // ★ 归一化 → %
    host_ch[HOST_CH_Z_L]        = Motor_LR_Crtl.z;
    host_ch[HOST_CH_Z_R]        = Motor_RR_Crtl.z;
    host_ch[HOST_CH_POS_L]      = (float)ENC_LR_POS;
    host_ch[HOST_CH_POS_R]      = (float)ENC_RR_POS;

    // ---- 链路（12~15）----
    host_ch[HOST_CH_ONLINE]     = (HOST_ST_ACTIVE == Host_Crtl_State()) ? 1.0f : 0.0f;
    host_ch[HOST_CH_RX_AGE_MS]  = (0 == host_ever_rx)
                                ? -1.0f                                   // 从未收到过 = -1
                                : (float)(uint32)(Sys_Tick_Ms - host_alive_ms);
    host_ch[HOST_CH_EN_BITS]    = (float)(host_enable & 0x0006);         // bit1 电机 / bit2 总使能
    host_ch[HOST_CH_SAFETY]     = (float)host_safety_bits();             // ★ 近似映射，见函数注释

    // ---- 转向（16~20）----
    // ★ 用"施加值"而不是 Servo_Target_Deg：后者是用户设定值，命令超过软限位时它仍显示 ±30°，
    //   拿 目标-实际 反推误差会看到约 2° 的假误差（实际早已进死区）
    host_ch[HOST_CH_STEER_TGT]  = Servo_Target_Applied_Deg;
    host_ch[HOST_CH_STEER_DEG]  = Servo_Angle_Deg;
    host_ch[HOST_CH_STEER_U]    = (float)Servo_Out;
    host_ch[HOST_CH_STEER_EN]   = (float)Servo_Crtl_Enable;
    // bit0 = 自检通过 / bit1 = 反馈失效 / bit2 = 顶死或反转（我方无顶死判据，恒 0 —— 非等价）
    host_ch[HOST_CH_STEER_ST]   = (0 != Servo_Enc_Fault)
                                ? (float)0x0002
                                : (float)0x0001;

    // ---- 遥控（21~24）----
    host_ch[HOST_CH_RC_ENG]     = (float)RC_Engage;
    host_ch[HOST_CH_RC_CH1]     = (float)RC_Raw_Ch1;
    host_ch[HOST_CH_RC_CH2]     = (float)RC_Raw_Ch2;
    host_ch[HOST_CH_RC_FRAMES]  = (float)RC_Frames;                      // ★ 全局量，见 Remote_Crtl.h

    // ---- 增益回读（25~26）：两轮同源整定，取左轮 ----
    host_ch[HOST_CH_K_V]        = Motor_LR_Crtl.k_v;
    host_ch[HOST_CH_K_Z]        = Motor_LR_Crtl.k_z;

    // ---- 惯导（27~32）----
    host_ch[HOST_CH_YAW]        = ins_imu_get_yaw();                     // 累积航向 deg（可超 ±360）
    host_ch[HOST_CH_GZ]         = ins_imu_get_gyro_z_dps();
    host_ch[HOST_CH_INS_X]      = ins_odom_get_x();
    host_ch[HOST_CH_INS_Y]      = ins_odom_get_y();
    host_ch[HOST_CH_INS_DIST]   = ins_odom_get_dist();
    host_ch[HOST_CH_INS_FLAGS]  = (float)ins_imu_flags();                // INS_ST_* 惯导状态位

    // ---- 路表/回放（33~35）----
    host_ch[HOST_CH_RT_CNT]     = (float)ins_route_get_count();
    host_ch[HOST_CH_RT_ST]      = (float)host_route_status_bits();
    host_ch[HOST_CH_PB_ELAT]    = route_pb_elat();

    // ---- 航向测试（36~37）----
    host_ch[HOST_CH_TAR_YAW]    = Head_Crtl_Get_Target();
    host_ch[HOST_CH_HT_SEG]     = 0.0f;      // ★ 非等价：我方无 head_test 模块（未迁移），固定 0

    // ---- 诊断三件套（38~40）----
    host_ch[HOST_CH_TICK_MS]    = (float)Sys_Tick_Ms;                    // 车端上电毫秒数
    host_ch[HOST_CH_INS_FRZ]    = ins_imu_zupt_frz_deg();
    host_ch[HOST_CH_INS_STATIC] = (0 != (ins_imu_flags() & INS_ST_STATIC)) ? 1.0f : 0.0f;
}

//--------------------------------------
// 函数简介     本帧该发几路（读参数 tel_sel）
// 参数说明     void
// 返回参数     uint16 HOST_CH_COUNT_MAX(41) 或 HOST_CT_COUNT(18)
// 使用示例     内部调用（host_send_telemetry 组帧之前）
// 备注信息     ★ 每帧现算，所以**切换档案不用重启**：上位机把 tel_sel 从 0 拖到 1，
//              下一帧的 COUNT 就变成 18，上位机按 COUNT 自动换档案。
//              ★ 参数是连续浮点（滑条 0~1），这里先二值化再用，避免 0.5 这种中间值
//                让 COUNT 变成"既不是 41 也不是 18"的第三个值 —— 上位机会直接把帧丢掉。
//--------------------------------------
static uint16 host_tel_pick_count (void)
{
    return (TEL_SEL >= HOST_TEL_SEL_THRESH) ? (uint16)HOST_CT_COUNT
                                            : (uint16)HOST_CH_COUNT_MAX;
}

//--------------------------------------
// 函数简介     取本帧第 i 路的数值（按当前档案解引用）
// 参数说明     i 当前档案内的下标（0..host_ch_count-1）
// 返回参数     float 通道值
// 使用示例     内部调用（host_send_telemetry 的填充循环）
// 备注信息     ★ 标定档里"我方无对应物"的那两路（ol_en / resv）固定返回 0，
//                不是在 host_ch[] 里随便找个位置顶替 —— 那是造假数据。
//--------------------------------------
static float host_tel_value (uint16 i)
{
    uint8 src;

    if (HOST_CT_COUNT == host_ch_count)
    {
        src = host_calib_from_full[i];
        return (0xFF == src) ? 0.0f : host_ch[src];
    }

    return host_ch[i];
}

//--------------------------------------
// 函数简介     组一帧遥测并发出去
// 参数说明     void
// 使用示例     内部调用
// 备注信息     PAYLOAD = SEQ(2) + TS(4) + MASK(2) + COUNT(2) + float32×COUNT
//              ★ 帧格式、MAGIC、CRC、4 字节尾一律不动（与队友 motor_link.c 逐字节一致）；
//                变的只有 COUNT 与数据段长度（41 或 18）。
//--------------------------------------
static void host_send_telemetry (void)
{
    uint16 n  = 0;
    uint16 pl;
    uint16 i;

    host_fill_channels();                       // 全量 41 路一律填满（标定档只是挑着发）

    host_ch_count = host_tel_pick_count();      // ★ 每帧现算档案，切完不用重启
    pl = (uint16)(10 + host_ch_count * 4);      // SEQ+TS+MASK+CNT+DATA

    host_telbuf[n ++] = HOST_MAGIC_UP_0;
    host_telbuf[n ++] = HOST_MAGIC_UP_1;
    host_telbuf[n ++] = HOST_TYPE_TELEMETRY;
    n = host_put_u16(host_telbuf, n, pl);
    n = host_put_u16(host_telbuf, n, host_tx_seq ++);
    n = host_put_u32(host_telbuf, n, Sys_Tick_Ms);

    // VALID_MASK：0xFFFF = **全部有效**（哨兵值，不是逐位图）。
    // ⚠ 字段只有 16 位而通道有 41 个，按位根本表达不下 —— 所以约定"全 1 = 全部有效"，
    //   上位机 ChannelStore.IsValid() 也按这条解释。真要做逐通道有效性必须先扩成 u32（改协议）。
    n = host_put_u16(host_telbuf, n, 0xFFFF);
    n = host_put_u16(host_telbuf, n, host_ch_count);

    for (i = 0; i < host_ch_count; i ++)
    {
        n = host_put_f32(host_telbuf, n, host_tel_value(i));
    }

    n = host_put_u16(host_telbuf, n, host_crc16(host_telbuf + 2, (uint16)(HOST_CRC_SPAN + pl)));

    host_telbuf[n ++] = 0x00;
    host_telbuf[n ++] = 0x00;
    host_telbuf[n ++] = 0x80;
    host_telbuf[n ++] = 0x7F;

    // ★ 模块忙就整帧丢掉：遥测可以丢，主循环不能被拖住（详见 host_send_frame 的说明）
    if (0 == gpio_get_level(WIFI_SPI_INT_PIN)) { return; }

    if (0 == wifi_spi_send_buffer(host_telbuf, n))
    {
        host_tx_count ++;
    }
}

/**********************************************************/
/*----------------------------------------[E] 遥测 [E]----*/
/**********************************************************/


/**********************************************************/
/*[S] 下行解析 [S]----------------------------------------*/
/**********************************************************/

//--------------------------------------
// 函数简介     执行一条下行命令，并回 ACK/NAK
// 参数说明     cmd 命令码 / pid 参数编号 / raw VALUE 的原始 32 位 / val VALUE 按 float 解释
// 参数说明     token 回执令牌（必须原样带回）
// 返回参数     void
// 使用示例     内部调用
// 备注信息     VALUE 一个字段装两种东西：参数下发时是 float，使能位图时按**数值**解释
//--------------------------------------
static void host_dispatch (uint8 cmd, uint8 pid, uint32 raw, float val, uint16 token)
{
    uint8 i;
    float v;

    (void)raw;                          // 只有 SET_ENABLE 需要数值解释，见下面的说明

    host_alive_ms = Sys_Tick_Ms;        // ★ 这就是"链路活着"的唯一判据来源
    host_ever_rx  = 1;
    host_rx_count ++;

    switch (cmd)
    {
        case HOST_CMD_PING:
            host_send_ack(cmd, token);
            break;

        case HOST_CMD_GET_PARAM:
            if (pid < HOST_PARAM_COUNT)
            {
                host_send_param_value(pid, *(host_param_meta[pid].ptr));
            }
            host_send_ack(cmd, token);
            break;

        case HOST_CMD_SET_PARAM:
            if (pid >= HOST_PARAM_COUNT)
            {
                host_send_nak(cmd, token, 1);       // 错误码 1：参数编号越界
                break;
            }
            // 取反写法：NaN / Inf 参与 > 和 < 比较都为假，用旧写法会漏过去
            if (!((val >= -FLT_MAX) && (val <= FLT_MAX)))
            {
                host_send_nak(cmd, token, 2);       // 错误码 2：数值非法
                break;
            }

            v = val;
            if (v < host_param_meta[pid].lo) { v = host_param_meta[pid].lo; }   // ★ 车端必须钳位
            if (v > host_param_meta[pid].hi) { v = host_param_meta[pid].hi; }

            // 滑条步长只是上位机的交互刻度，**不是接收端的量化器** ——
            // 这里若再按 step 取整一次，"滑条上的数"就不等于"车上生效的数"了
            *(host_param_meta[pid].ptr) = v;

            // ★ 推到实际控制器。这是本工程既定的唯一同步点
            //   （param_cache[] → Motor_LR/RR_Crtl 与 Servo_Crtl_Kp/U0）
            Flash_SyncTo_Param();

            host_send_param_value(pid, v);          // 回读，上位机据此确认
            host_send_ack(cmd, token);
            break;

        case HOST_CMD_SET_ENABLE:
            if (!((val >= 0.0f) && (val <= 65535.0f)))
            {
                host_send_nak(cmd, token, 2);
                break;
            }
            // ★ VALUE 按"数值"解释（上位机发 6.0f，这里就该得到 6），**不能**写成 (uint16)raw ——
            //   raw 是 IEEE754 位模式，6.0f 的字节是 00 00 C0 40，截 16 位会变成 0x0000，
            //   后果是"使能怎么都传不下来"且完全不报错。
            //
            // ★★ 这里**只记录位图**，真正写驱动在 host_drive_tick() 里做。不要在这里补
            //    `Servo_Crtl_Enable = ...` —— 页面级的仲裁（遥控优先 / UP 模式不动车 /
            //    心跳锁存刹停）全在那边，两处都写会让那三道门在"刚收到 SET_ENABLE"的那一圈失效，
            //    表现是 UP 模式或锁存态下电机仍被上位机使能。
            host_enable = (uint16)(val + 0.5f);

            host_send_ack(cmd, token);
            break;

        case HOST_CMD_GET_TABLE:
            host_param_table_start();
            host_send_ack(cmd, token);
            break;

        case HOST_CMD_SAVE:
            Param_Save();                           // 写 Flash（Param_Storage.c）
            host_send_ack(cmd, token);
            break;

        case HOST_CMD_LOAD:
            // 语义：**重新从 Flash 载入**（丢弃未保存的改动）。
            // 不做"恢复出厂默认"：DEFAULT_PARAMS 是 Param_Storage.c 里的 static const，不对外暴露，
            // 而本工程恢复默认的既定做法是"擦 Flash 后重启"（Param_Erase + PARAM_FORCE_RESET）。
            Param_Init();                           // 内含 flash 读取 + Flash_SyncTo_Param

            for (i = 0; i < HOST_PARAM_COUNT; i ++)
            {
                host_send_param_value(i, *(host_param_meta[i].ptr));    // 逐项回读
            }
            host_send_ack(cmd, token);
            break;

        default:
            host_send_ack(cmd, token);
            break;
    }
}

//--------------------------------------
// 函数简介     把新收到的字节追加进累加器，并解析出所有完整命令帧
// 参数说明     buf 本次收到的数据 / len 字节数
// 返回参数     void
// 使用示例     内部调用
// 备注信息     ★★ 两条纪律都在这里，改之前先读懂：
//              ① CRC 失败只前移 1 字节，**不能跳整帧** —— 帧头 5A A5 完全可能是数据里的
//                 巧合字节，按"帧头是假的就把 15 字节全丢掉"处理，会把后面真正的帧一起吞掉，
//                 症状是"偶尔丢一帧、查不出来"。
//              ② 解析完必须把**不足一帧的尾巴搬到开头留着** —— 这次收到的最后一个字节
//                 可能就是 0x5A，0xA5 在下一个包里；丢掉它这一帧就永远找不回来了。
//--------------------------------------
static void host_rx_feed (const uint8 *buf, uint16 len)
{
    uint16 i;
    uint16 room;

    if (0 == len) { return; }

    room = (uint16)(HOST_RXACC_SIZE - host_rxacc_len);
    if (len > room) { len = room; }         // 正常到不了这里：每轮都把累加器清到 < 15 字节

    memcpy(host_rxacc + host_rxacc_len, buf, len);
    host_rxacc_len = (uint16)(host_rxacc_len + len);

    i = 0;
    while ((uint16)(i + HOST_DN_FRAME_LEN) <= host_rxacc_len)
    {
        if ((HOST_MAGIC_DN_0 != host_rxacc[i]) || (HOST_MAGIC_DN_1 != host_rxacc[i + 1])) { i ++; continue; }
        if (HOST_TYPE_COMMAND != host_rxacc[i + 2])                                       { i ++; continue; }
        if (HOST_DN_PAYLOAD   != host_get_u16(host_rxacc, (uint16)(i + 3)))                { i ++; continue; }

        if (host_crc16(host_rxacc + i + 2, (uint16)(HOST_CRC_SPAN + HOST_DN_PAYLOAD))
            != host_get_u16(host_rxacc, (uint16)(i + 13)))
        {
            host_crc_err ++;
            i ++;                           // ★ 纪律①：只前移 1 字节
            continue;
        }

        host_dispatch(host_rxacc[i + 5],                                      // CMD
                      host_rxacc[i + 6],                                      // ID
                      host_get_u32(host_rxacc, (uint16)(i + 7)),              // VALUE 原始 32 位
                      host_get_f32(host_rxacc, (uint16)(i + 7)),              // VALUE 按 float 解释
                      host_get_u16(host_rxacc, (uint16)(i + 11)));            // TOKEN

        i = (uint16)(i + HOST_DN_FRAME_LEN);
    }

    if (i > 0)                              // ★ 纪律②：留下不足一帧的尾巴
    {
        memmove(host_rxacc, host_rxacc + i, (uint32)(host_rxacc_len - i));
        host_rxacc_len = (uint16)(host_rxacc_len - i);
    }
}

/**********************************************************/
/*----------------------------------------[E] 下行解析 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] WiFi 选择与连接 [S]---------------------------------*/
/**********************************************************/

// ★★ 这是**全工程唯一的 WiFi 选择连接界面**（2026-10-16 统一）：
//   两个入口共用它 —— ① Host 页进门（未连时自动弹）② 调试页第 1 项 WIFI-SPI
//   （`Debug.c` 里原来的第二份界面/候选表已删除，那个菜单项改为直接调本函数）。
//   统一后**候选表与目标 IP/端口只剩 `Host_Crtl.h` 一份**
//   （`HOST_WIFI_LIST` / `HOST_TARGET_*`），换热点或换上位机只改那个头文件。
//   两边最终都置同一个 `wifi_spi_inited`，所以谁先连上，另一个都能直接复用（不再重复 init）。

// [三级界面]WiFi 候选列表
static void host_wifi_ui (void)
{
    uint8 i;

    ips200_show_string(8  , 0  , "[WIFI]-SPI");
    ips200_show_string(0  , 16 , "==============================");

    for (i = 0; i < HOST_WIFI_COUNT; i ++)
    {
        ips200_show_string(10, (uint16)(48 + 16 * i), host_wifi_list[i].ssid);
    }

    ips200_show_string(10, 128, "CONFIRM: connect");
    ips200_show_string(10, 144, "BACK: cancel");
}

//--------------------------------------
// 函数简介     用候选表第 idx 项发起 WiFi 初始化 + 建 UDP socket
// 参数说明     idx 候选下标（0 起）
// 返回参数     HOST_OK / HOST_ERR_WIFI / HOST_ERR_SOCKET
// 使用示例     内部调用（Host_Crtl_WiFi_Select）
// 备注信息     ★ 会阻塞（WiFi 最长约 10s、Socket 更长）。**两步都成功才置 wifi_spi_inited**
//--------------------------------------
static uint8 host_wifi_open (uint8 idx)
{
    if (0 != wifi_spi_init(host_wifi_list[idx].ssid, host_wifi_list[idx].pwd))
    {
        host_err = HOST_ERR_WIFI;
        return HOST_ERR_WIFI;
    }

    if (0 != wifi_spi_socket_connect(host_sock_type, host_target_ip, host_target_port, host_local_port))
    {
        host_err = HOST_ERR_SOCKET;
        return HOST_ERR_SOCKET;
    }

    // 与调试页共用的"WiFi 已可用"标志：置上之后两边都能用这条链路
    wifi_spi_inited = 1;
    host_err = HOST_OK;

    return HOST_OK;
}

//--------------------------------------
// 函数简介     ★★ 全工程统一的 WiFi 选择连接界面（Host 页进门 / 调试页 WIFI-SPI 共用）
// 参数说明     void
// 返回参数     HOST_OK = 已连上；HOST_ERR_NOT_INIT = 用户按 BACK 取消
// 使用示例     Host_Crtl_Connect() 内部；Debug.c 的调试页第 1 项
// 备注信息     ★ 阻塞界面：WiFi 最长十几秒，只能在按键处调用。
//              连上**直接返回**（本界面会把整屏清掉，调用者随后自己重画）；
//              失败则留在本界面，换一个热点或再试；BACK 取消
//              ★ 未连时弹本界面；已连时（只可能来自调试页）也允许进来看一眼并换网络
//--------------------------------------
uint8 Host_Crtl_WiFi_Select (void)
{
    uint8 sel         = 0;
    uint8 i;
    uint8 key_pressed = 1;              // 首圈画一次光标
    uint8 ret;

    ips200_clear();
    host_wifi_ui();

    // 已连时给个提示（Host 页在已连时不会弹本界面，这条只可能出现在调试页入口）
    if (0 != wifi_spi_inited)
    {
        ips200_show_string(10, 176, "link already up   ");
    }

    while (1)
    {
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            key_pressed = 1;
            sel = (0 == sel) ? (uint8)(HOST_WIFI_COUNT - 1) : (uint8)(sel - 1);
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            key_pressed = 1;
            sel ++;
            if (sel >= HOST_WIFI_COUNT) { sel = 0; }
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);

            ips200_show_string(10, 176, "connecting...     ");
            ret = host_wifi_open(sel);

            if (HOST_OK == ret) { return HOST_OK; }         // 连上 -> 回 Host 页

            // 失败：留在本界面。是哪一步失败直接显示在这里
            // （Host 页那一格 ERR 已按用户要求撤掉，所以错误信息只在本界面出现）
            ips200_show_string(10, 176, (HOST_ERR_WIFI == ret) ? "Wifi FAIL         " : "Sock FAIL         ");
            key_pressed = 1;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);
            return HOST_ERR_NOT_INIT;                       // 用户取消
        }

        /* 光标更新：先把所有候选行的光标位清空再画，避免残影 */
        if (key_pressed)
        {
            key_pressed = 0;
            for (i = 0; i < HOST_WIFI_COUNT; i ++)
            {
                ips200_show_string(0, (uint16)(48 + 16 * i), " ");
            }
            ips200_show_string(0, (uint16)(48 + 16 * sel), ">");
        }
    }
}

/**********************************************************/
/*----------------------------------------[E] WiFi 选择与连接 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 接口实现 [S]----------------------------------------*/
/**********************************************************/

//--------------------------------------
// 函数简介     清链路运行状态（计数器、时间戳、接收缓冲）
// 参数说明     void
// 使用示例     内部调用 / Host_Crtl_Connect 开头
// 备注信息     不碰 wifi_spi_inited（那是"WiFi 可用"的标志，归连接流程管）
//--------------------------------------
static void host_state_clear (void)
{
    host_tx_ms      = 0;
    host_rx_ms      = 0;
    host_alive_ms   = 0;
    host_ever_rx    = 0;
    host_tx_seq     = 0;
    host_tx_count   = 0;
    host_rx_count   = 0;
    host_crc_err    = 0;
    host_rxacc_len  = 0;
    host_pt_pend    = 0;
    host_pt_id      = 0;
    host_pt_ms      = 0;
}

//--------------------------------------
// 函数简介     上电初始化
// 参数说明     void
// 使用示例     Peripheral_Init() 里调用一次
// 备注信息     ★ 不阻塞、不连 WiFi（连 WiFi 最长十几秒，放这里会把上电流程卡住）。
//              连接动作由页面上的确认键触发（Host_Crtl_Connect）
//--------------------------------------
void Host_Crtl_Init (void)
{
    host_state_clear();
    host_enable = 0;
    host_err    = HOST_ERR_NOT_INIT;

    // 三个开关回上电默认：仅上行、链路关、遥控关、无锁存
    Host_Mode = HOST_MODE_UP;
    Host_Link = HOST_LINK_OFF;
    Host_Lost = 0;

    // 如果 WiFi 已经由调试页连上了，本模块直接复用那条链路（wifi_spi_inited 是共用标志），
    // 不需要再连一次 —— Task 的效率判据也是它
    if (0 != wifi_spi_inited) { host_err = HOST_OK; }
}

//--------------------------------------
// 函数简介     连接：★ 已连则复用，未连则弹 WiFi 选择界面
// 参数说明     void
// 返回参数     HOST_OK / HOST_ERR_WIFI / HOST_ERR_SOCKET / HOST_ERR_NOT_INIT(用户取消)
// 使用示例     Host 页的确认键触发
// 备注信息     ★ 会阻塞（未连时 WiFi 最长约 10s、Socket 更长），只能在页面里按键调用。
//              ★ 已连（wifi_spi_inited != 0，例如在调试页连过）时**不再重复 init** ——
//                重复 init 会打断已建的 socket，且白等十几秒
//--------------------------------------
uint8 Host_Crtl_Connect (void)
{
    uint8 ret;

    host_state_clear();

    if (0 == wifi_spi_inited)
    {
        // 未连 -> 弹统一的 WiFi 候选界面；连上返回 HOST_OK，失败/取消带回对应错误码
        ret = Host_Crtl_WiFi_Select();
        if (HOST_OK != ret)
        {
            // HOST_ERR_NOT_INIT 在这里的语义 = 用户取消了（原义"还没连过"，正好对上）
            host_err = ret;
            return ret;
        }
    }
    else
    {
        // 调试页已经连过 -> 直接复用那条链路
        host_err = HOST_OK;
    }

    // ★ 参数表与 EVENT 已挪到 Host_Crtl_Set_Link(ON)：进页面时 Link 还是 OFF，
    //   这里发出去就违反了"链路关着一个字节都不发"。开 Link 时补发，上位机照样能长滑条。

    return HOST_OK;
}

//--------------------------------------
// 函数简介     ★★ 链路层：收命令 + 发遥测
// 参数说明     void
// 使用示例     Host_Crtl_Task();   // 放在 Host_Crtl_Page() 的循环里
// 备注信息     自带限频与"模块忙就跳过"，可安全放在任意 while(1) 里。
//              ★ 两道门：① wifi_spi_inited 为 0（链路没建）② Host_Link = OFF（链路没开）
//                任一不满足就是纯空操作，**上行下行一起停**。
//              ★ 调用点只有 Host_Crtl_Page() 一处 —— 主菜单待机时整车"静默"，一个字节都不发。
//                这是有意为之：待机状态下不该有 RF 活动，也不该让链路状态机空转
//--------------------------------------
void Host_Crtl_Task (void)
{
    uint16 n;

    if (0 == wifi_spi_inited) { return; }   // 链路没建立 -> 什么都不做

    // ★★ 链路开关（只管 WiFi）。注意是"上行下行一起停"，不是只停上行；
    //    Link = OFF 时还要收帧是没意义的（收到的 SET_PARAM 也没人看波形验证），
    //    而且会让"静默"这条纪律变成空话。
    if (HOST_LINK_ON != Host_Link) { return; }

    /* 收命令（限频） */
    if ((uint32)(Sys_Tick_Ms - host_rx_ms) >= HOST_RX_PERIOD_MS)
    {
        host_rx_ms = Sys_Tick_Ms;

        /* ★ 先看 INT 引脚：模块忙就跳过这一轮读取。
         *   否则 wifi_spi_read_buffer 内部的 wait_idle(OTHER_TIME_OUT) 最长会把循环卡住 1 秒
         *   —— 少收一轮命令没关系，控制循环不能被拖住。 */
        if (0 != gpio_get_level(WIFI_SPI_INT_PIN))
        {
            n = (uint16)wifi_spi_read_buffer(host_rxbuf, (uint32)sizeof(host_rxbuf));
            if (n > 0)
            {
                host_rx_feed(host_rxbuf, n);
            }
        }
    }

    /* 发遥测（限频） */
    if ((uint32)(Sys_Tick_Ms - host_tx_ms) >= HOST_TX_PERIOD_MS)
    {
        host_tx_ms = Sys_Tick_Ms;
        host_send_telemetry();
    }

    /* 参数表分批发送：必须每圈都调 —— 它自带 5ms 限频，
       不能挂在上面的限频分支里，否则会跟遥测的节拍互相饿死 */
    host_param_table_tick();
}

//--------------------------------------
// 函数简介     驱动侧清零（内部）
// 参数说明     void
// 使用示例     遥控收回、关链路、切模式时调用（都只在遥控未接管时才允许清）
// 备注信息     只清"本模块可能写过的那些量"，不碰 Servo_Crtl_Kp / 参数缓存。
//              ★ Motor_Crtl_Enable 由 1 变 0 的那一拍 Motor_Crtl 会补发一次 Motor_Set(...,0)，
//                之后不再写 —— 所以反复调用是安全的，不会每拍冲 PWM
//--------------------------------------
static void host_actuator_clear (void)
{
    Servo_Crtl_Enable    = 0;
    Servo_Target_Deg     = 0.0f;
    Motor_Crtl_Enable    = 0;
    Motor_LR_Crtl.target = 0.0f;
    Motor_RR_Crtl.target = 0.0f;
}

//--------------------------------------
// 函数简介     驱动侧清零 + 丢掉上位机的使能位图
// 参数说明     void
// 使用示例     Host_Crtl_Release()（退出整页）—— **唯一的调用点**
// 备注信息     ★ 与 host_actuator_clear() 的区别就是最后那个 host_enable：
//              出入页面要"全收"，位图也该一并作废；
//              而"遥控收回 / 关链路 / 切模式"只该收驱动侧，**不能**顺手把上位机的位图清掉 ——
//              那样 CMD 模式下遥控一松手，上位机就得重发 SET_ENABLE 才能继续。
//              （关链路与切模式确实要作废位图，但那是直接写 host_enable = 0，不走本函数，
//                因为那两处可能正被遥控接管，驱动侧一个字节都不能碰）
//--------------------------------------
static void host_drive_clear (void)
{
    host_enable = 0;
    host_actuator_clear();
}

//--------------------------------------
// 函数简介     ★★ 上位机链路开关（只管 WiFi）
// 参数说明     on	HOST_LINK_ON / HOST_LINK_OFF
// 使用示例     Host_Crtl_Set_Link(HOST_LINK_ON);
// 备注信息     关：停止收/发 + 丢弃上位机的使能位图 + 清心跳锁存。
//                 ★★ **不收回遥控**（2026-10-17 语义收窄）：遥控走 UART 接收机，
//                    与本开关没有任何关系。只有当"遥控没在接管"时才顺手把驱动侧收干净；
//                    遥控正在跑的时候去清驱动，车会顿一下（Motor_Crtl_Enable 1→0→1）。
//             开：补发参数表与 EVENT（上位机的参数面板靠它长出来 —— 进页面时 Link 还是 OFF，
//                 所以这件事不能放在 Connect 里），并给心跳门一个 HOST_ALIVE_MS 的宽限
//             退出整个页面才"全收"（含遥控与驱动），见 Host_Crtl_Release()
//--------------------------------------
void Host_Crtl_Set_Link (uint8 on)
{
    if (0 == on)
    {
        Host_Link = HOST_LINK_OFF;
        Host_Lost = 0;

        // 位图必须丢：链路的来路断了，旧位图不该还留着生效
        host_enable = 0;

        // 驱动只在"上位机自己掌权"时清；遥控接管期间一个字节都不能碰
        if (RC_ENGAGE_OFF == RC_Engage)
        {
            host_actuator_clear();
        }
        return;
    }

    if (HOST_LINK_ON == Host_Link) { return; }   // 已经开着，别重复发参数表

    Host_Link = HOST_LINK_ON;

    // 把限频起点往回拨一个周期 -> 下一圈立刻发，不用等 20ms
    host_tx_ms = Sys_Tick_Ms - HOST_TX_PERIOD_MS;
    host_rx_ms = Sys_Tick_Ms - HOST_RX_PERIOD_MS;

    // ★ 心跳门重新起算，给 HOST_ALIVE_MS 宽限。
    //   不这么做的话，CMD 模式一开 Link 就立刻锁存（上位机还没发包），用户会以为板子坏了
    host_alive_ms = Sys_Tick_Ms;

    if (0 != wifi_spi_inited)           // 防御：链路没建就发不出去，别白写 SPI
    {
        host_param_table_start();       // 上位机据此长滑条
        host_send_event("host link on");
    }
}

//--------------------------------------
// 函数简介     ★★ 工作模式切换
// 参数说明     mode	HOST_MODE_UP / HOST_MODE_CMD
// 使用示例     Host_Crtl_Set_Mode(HOST_MODE_CMD);
// 备注信息     ★★ Link = ON 时**直接返回、不报错**：改模式等于换安全责任归属，
//                 必须是"先停车、再换模式"的动作，不能运行中切。
//              ★ 切模式会把驱动侧清一次 —— CMD 的使能（host_enable 位图）不该跟着带进 UP，
//                 否则会出现"切回 UP 了车还在被上位机的旧位图使能"。
//              ★ Host_Lost 一并清：换模式即撤销锁存（换到 UP 本来就不判心跳了）。
//              ⚠ UP -> CMD 时如果 CH6 还是 0，遥控会立刻让出驾驶权（auth 从 REMOTE 变
//                 NONE/HOST，车停下或交给上位机）。这是"切模式 = 换驾驶员"的必然结果，
//                 不是 bug；要无缝接续就先拨 CH6 = 1 再切。
//--------------------------------------
void Host_Crtl_Set_Mode (uint8 mode)
{
    if (HOST_LINK_ON == Host_Link) { return; }

    mode = (HOST_MODE_CMD == mode) ? HOST_MODE_CMD : HOST_MODE_UP;   // 收窄到合法值
    if (mode == Host_Mode) { return; }

    Host_Mode = mode;
    Host_Lost = 0;

    // ★ 位图一定作废：CMD 的使能不该跟着带进 UP（否则"切回 UP 了车还按旧位图使能"）
    host_enable = 0;

    // ★ 驱动只在遥控未接管时才清 —— Mode 能在 Link = OFF 时改，而那时遥控完全可能正在跑，
    //   无条件清驱动会让车顿一下（与 Set_Link(OFF) 同一个理由）
    if (RC_ENGAGE_OFF == RC_Engage)
    {
        host_actuator_clear();
    }
}

//--------------------------------------
// 函数简介     人工复位心跳锁存
// 参数说明     void
// 使用示例     锁存态的确认键调用
// 备注信息     ★ 故意**不**重置 host_alive_ms：复位不等于"对面还活着"。
//                 上位机确实没在发的话，复位后下一圈会立刻重新锁存 —— 这是正确行为，不是 bug。
//                 想真正解除，要么让上位机发帧，要么把 Mode 切回 UP / 关掉 Link。
//--------------------------------------
void Host_Crtl_Lost_Clear (void)
{
    Host_Lost = 0;
}

//--------------------------------------
// 函数简介     整页退出
// 参数说明     void
// 使用示例     离开上位机页面时调用
// 备注信息     ★ 退出整个页面是**唯一**的"全收"动作（2026-10-17 语义收窄后）：
//              Set_Link(OFF) 只管 WiFi，不再代劳收回遥控；遥控与驱动在这里显式收干净。
//              "每次进页面都是仅上行 + 链路关 + 遥控关"就靠这一句保证。
//              ⚠ 单按 BACK 退出时是 `Motor_Crtl_Enable = 0`（**滑行**，不是刹停），
//                与 Remote_Crtl_Release 同口径。车在跑请先关 Link（刹停）再退出
//--------------------------------------
void Host_Crtl_Release (void)
{
    Host_Crtl_Set_Link(HOST_LINK_OFF);      // 关 WiFi：停收发 + 丢位图 + 清锁存（不碰遥控）

    // ★ Link 不再管遥控了，所以这里必须显式收回 —— 否则"遥控开着退出页面"会留下接管态
    RC_Engage = RC_ENGAGE_OFF;
    Host_Auth = HOST_AUTH_NONE;             // 驾驶权归零，下次进页面重新解析
    host_drive_clear();                     // 驱动侧全清（含位图，兜底）

    Host_Mode = HOST_MODE_UP;
}

/**********************************************************/
/*------------------------------------[S] 驾驶权仲裁 [S]--*/
/**********************************************************/

//--------------------------------------
// 函数简介     遥控侧安全监视（CH5 急停 / 接收机失联）
// 参数说明     void
// 使用示例     Host_Crtl_Page() 每圈在仲裁之前调一次
// 备注信息     ★★ 这两条判据原先只写在 Remote_Crtl_Apply() 里，而 Apply 只在"遥控掌权"
//                 时才被调用 —— 后果是 CMD 模式（上位机开车）下拨 CH5 **完全无效**，
//                 等于把唯一的物理急停通道废掉了。提到这里之后：
//                    · 只要 Remote = ON，CH5 在任何 Mode / 任何 Link 状态下都能停车
//                    · Remote = OFF 时仍然一概不采信（这是明确要保留的边界）
//              ★ 锁存语义与遥控页一致：ESTOP / FAILSAFE 都不自动恢复，要人工按确认键。
//              ⚠ 与 Remote_Crtl_Apply() 里的同名判据**有意重复**：那个函数是遥控页在用的，
//                 不能改成"只由本函数判定"，否则遥控页单独跑时就没有急停了。
//--------------------------------------
static void host_rc_watch (void)
{
    if (RC_ENGAGE_OFF == RC_Engage) { return; }     // 没接管 -> 接收机数据一概不采信

    if (0 == RC_Online)
    {
        // 失联优先于急停：数据本身就不可信时，屏幕该说"失联"而不是"急停"
        if (RC_ENGAGE_ESTOP != RC_Engage) { RC_Engage = RC_ENGAGE_FAILSAFE; }
        return;
    }

    if (1 == RC_Sw5) { RC_Engage = RC_ENGAGE_ESTOP; }   // CH5：任何 Mode / Link 下都停车
}

//--------------------------------------
// 函数简介     驾驶权解析（纯函数，不写任何驱动）
// 参数说明     void
// 返回参数     HOST_AUTH_NONE / HOST / REMOTE / STOP
// 使用示例     Host_Crtl_Page() 每圈算一次，结果同时用于仲裁与屏幕
// 备注信息     ★ 优先级：遥控锁存（STOP） > 遥控接管 > 上位机 > 没人
//              ★ UP 模式下**不存在 HOST**：UP 的定义就是"上位机不开车"，
//                所以 UP + Remote OFF = 没人开（驱动侧清空），不是"上位机在开"。
//              ★ CMD + Link = OFF 返回 NONE 而不是 HOST：链路没开通，上位机发不出
//                任何目标，报"上位机在开"会骗人（屏幕显示 HOST 却不走车最难受）。
//--------------------------------------
static uint8 host_auth_resolve (void)
{
    if ((RC_ENGAGE_ESTOP == RC_Engage) || (RC_ENGAGE_FAILSAFE == RC_Engage))
    {
        return HOST_AUTH_STOP;
    }

    if (RC_ENGAGE_ON == RC_Engage)
    {
        if (HOST_MODE_UP == Host_Mode) { return HOST_AUTH_REMOTE; }     // UP：开了就开
        if (1 == RC_Sw6)               { return HOST_AUTH_REMOTE; }     // CMD：CH6 显式移交
    }

    if (HOST_MODE_CMD == Host_Mode)
    {
        return (HOST_LINK_ON == Host_Link) ? HOST_AUTH_HOST : HOST_AUTH_NONE;
    }

    return HOST_AUTH_NONE;                  // UP + 遥控未接管
}

//--------------------------------------
// 函数简介     ★★ CMD 模式的驱动结算（内部，只由 Host_Crtl_Page 的仲裁调用）
// 参数说明     void
// 使用示例     Host_Crtl_Page() 里：auth = HOST / NONE 时才调它
// 备注信息     ★ 为什么不在 SET_ENABLE 的命令处理里直接写驱动：那只是"某一帧到达时"的一拍，
//                 而遥控抢权、模式切换、心跳超时都发生在其它时刻。全部收到这里结算，
//                 驱动侧就只有一个写入口，不会互相打架。
//              ★ UP 模式直接返回、不碰驱动侧：从 CMD 切过来时 Set_Mode 已经清过一次，
//                 这里每圈重复清只是白费。
//              ★ 锁存后**不会自动恢复**：上位机重新发帧也不解除，要人工按确认键。
//--------------------------------------
static void host_drive_tick (void)
{
    if (HOST_MODE_CMD != Host_Mode) { return; }

    // ★★ Link = OFF 时**不参与结算**：上位机链路没开通，`host_alive_ms` 的时钟也不走
    //    （它只在 Set_Link(ON) 与收到下行帧时刷新）。此时若继续判心跳，600ms 后必然假锁存，
    //    表现为"关着链路、屏幕 State 显示 IDLE，驱动却被锁存刹停接管"，而且确认键会被
    //    "优先复位锁存"吃掉（想开 Link 却只清了锁存）。
    //    链路关着时驱动侧的状态由 Set_Link(OFF) / Set_Mode 负责收干净，不需要这里补。
    if (HOST_LINK_ON != Host_Link) { return; }

    /* 心跳门：CMD 模式的前提是"上位机还在" */
    if (0 == Host_Lost)
    {
        if ((uint32)(Sys_Tick_Ms - host_alive_ms) > HOST_ALIVE_MS)
        {
            Host_Lost = 1;                  // 锁存，等人工复位
        }
    }

    if (0 != Host_Lost)
    {
        /* 闭环刹停：后轮保持使能、目标 0（速度环主动出力减速）；
           舵机保持使能 + 保持失联前的转向 —— 故意不写 Servo_Target_Deg，
           减速过程中方向盘突然自己回正更危险（与 RC_ENGAGE_ESTOP 同一取舍）。
           目标 0 到实际停住由 Motor_Crtl 的斜坡（MOTOR_CRTL_SLEW）兜着，不会猛刹。 */
        Servo_Crtl_Enable    = 1;
        Motor_Crtl_Enable    = 1;
        Motor_LR_Crtl.target = 0.0f;
        Motor_RR_Crtl.target = 0.0f;
        return;
    }

    /* 正常：上位机的使能位图说了算（SET_ENABLE 只写 host_enable，写驱动在这里） */
    Servo_Crtl_Enable = (0 != (host_enable & HOST_EN_STEER)) ? 1 : 0;
    Motor_Crtl_Enable = (0 != (host_enable & HOST_EN_MOTOR)) ? 1 : 0;
}

//--------------------------------------
// 函数简介     查询连接状态
// 参数说明     void
// 返回参数     uint8 HOST_ST_IDLE / HOST_ST_ACTIVE / HOST_ST_OFFLINE
// 使用示例     Host_Crtl_State();
// 备注信息     判据只用"是否收到下行帧"——见 Host_Crtl.h 里 HOST_ST_* 的说明
//--------------------------------------
uint8 Host_Crtl_State (void)
{
    if (0 == wifi_spi_inited) { return HOST_ST_IDLE; }
    if (0 == host_ever_rx)    { return HOST_ST_OFFLINE; }   // 链路建了，但对面一直没响（上位机没连上）

    if ((uint32)(Sys_Tick_Ms - host_alive_ms) > HOST_ALIVE_MS) { return HOST_ST_OFFLINE; }

    return HOST_ST_ACTIVE;
}

uint8 Host_Crtl_Err (void)
{
    return host_err;
}

uint32 Host_Crtl_Tx_Count (void)
{
    return host_tx_count;
}

uint32 Host_Crtl_Rx_Count (void)
{
    return host_rx_count;
}

uint32 Host_Crtl_Crc_Err (void)
{
    return host_crc_err;
}

/**********************************************************/
/*----------------------------------------[E] 接口实现 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 页面 [S]--------------------------------------------*/
/**********************************************************/

//-------------------------------------------------------------------------------------------------------------------
// [一级界面]上位机页面
//
// 屏幕分三段（从上到下）：
//   ① 三个开关（上下键选行、确认键切换；光标是 x=2 的 ">"）
//        Host Mode:  UP / CMD    —— 仅上行 / 上位机指令（★ Link = ON 时按不动，见文件头）
//        Host Link:  OFF / ON    —— **只管 WiFi**：关 = 上位机收/发全停，遥控不受影响
//        Remote Crtl: OFF / READY / DRIVE / ESTOP / RC LOST
//                                 —— 是否采信接收机（走 UART，与 Link 无关）
//                                    ★ DRIVE = 驾驶权在遥控；READY = 通信正常但驾驶权
//                                      在别处（CMD + CH6 = 0，上位机在开）。两者分开显示，
//                                      否则"CMD 下遥控开着"会被误读成"遥控在开车"
//        ★ 驾驶权规则：UP 直接给遥控；CMD 默认上位机，**CH6 = 1** 才交给遥控。
//          CH6 是物理拨杆，本页不给它单独一行 —— 拨一下看 Remote 那行 OFF/READY → DRIVE
//          就是全部反馈。
//        ★ CH5 = 1 一律停车（ESTOP 锁存），**与 Mode、Link 全都无关**，前提只是 Remote = ON。
//          这是 CMD 模式下唯一的物理急停通道。
//   ② 链路体检（下半三行只讲**上位机这条 WiFi 链路**；遥控的状态已经在上面的开关行显示）
//        State  IDLE   = 链路关着（Link: OFF）
//              TX ON  = UP 模式在发遥测。★ 本模式**不判心跳**，"我在发"就是全部含义，
//                       所以不能沿用 ACTIVE/OFFLINE —— 那会让人对着正常工作的链路查故障
//              ACTIVE = CMD 模式，心跳正常
//              LOST   = CMD 模式，心跳超时已锁存（必须人工按确认键复位）
//        AGE   距上次收到下行帧的毫秒数；**UP 模式显示 ---**（本模式下它没有后果）
//        CRC   CRC 校验失败次数（一直涨 = 对面协议不对 / 线序有问题）
//   ③ 退出
//        BACK: MENU  —— 退出本页 = **唯一"全收"**：链路关 + 遥控收回 + 模式回 UP + 驱动失能
//
// ★ 按键（2026-10-17 起本页不再"只有 BACK 一个键"）：
//     UP / DOWN   移动光标（三行环回）
//     CONFIRM     切换光标所在行的值；**锁存态下优先做"复位锁存"**（不切开关）
//     BACK        退出本页
//
// ★ 屏幕内容只减不增（2026-10-17 用户要求）：TX/RX 收发帧数、ERR 错误码都已从屏幕撤掉 ——
//   接口仍保留（Host_Crtl_Tx_Count / Rx_Count / Err），想用直接调，屏幕上不占行。
//   唯一"新增"的是 y=128 那行锁存提示，且只在异常态出现。
// ★ 刷新节拍 = HOST_UI_REFRESH_TICKS（默认 300ms）；按键时立刻整屏重画，不等节拍。
// ★ 连接时机 = **进入页面**（"进门先连"，用户设计）：
//   链路已建（`wifi_spi_inited != 0`，例如调试页连过）-> 直接进页面，不弹任何界面
//   链路未建 -> 立刻弹 [HOST]-WIFI 三级界面选热点；连上后回到本页面
//   ★ 弹出来的是**选择界面**，本身不阻塞 —— 只有按下确认键选定热点才真正跑 init（最长十几秒）
//   ★ 从那个界面回来后必须整屏重画（它会 ips200_clear），见 host_page_connect()
//   BACK 取消 = **回主菜单**（本页的入口就是那个界面，取消了自然不进来），不算失败
//-------------------------------------------------------------------------------------------------------------------

// ★★ 屏幕尺寸是 240 宽 × 320 高（竖屏，IPS200_DEFAULT_DISPLAY_DIR = IPS200_PORTAIT_180）
//    8x16 字体每字符 8px，所以**从 x=10 起一行最多写 28 个字符**（10 + 8*28 - 1 = 233），
//    从 x=0 起最多 30 个字符（0 + 8*30 - 1 = 239，正好到边）。
//    ⚠ 超了不会"画到下一行"，而是在 `ips200_set_region()`（zf_device_ips200.c:331）
//      的 `zf_assert(x2 < ips200_width_max)` 上炸断言、整车停在异常里 ——
//      而且报错行号指向的是逐飞库，不是这里。2026-10-17 就踩过：某行"补空格擦残影"
//      补成了 30 字符、起点又 x=10，一直画到 249 → 进页面即断言。
//    → 定宽擦残影时，先算 (240 - x) / 8，别凭感觉补
//
// 版面：三行开关 y=32/48/64，键 x=10..105，值统一从 x=114 起、定宽 7 字符（114+56-1=169）
//      状态区 y=80/96/112，标签 x=10，值统一从 x=66 起（"State:" 最长 6 字符 -> 到 x=57）
#define HOST_UI_ROW_Y0      (32)        // 第一行开关的 y
#define HOST_UI_ROW_DY      (16)        // 行距（8x16 字体）
#define HOST_UI_VAL_X       (114)       // 三行开关的"值"起始列
#define HOST_UI_CUR_X       (2)         // 光标列（"文字"从 x=10 开始，中间留 8px）
#define HOST_UI_STA_X       (66)        // 状态区（State/AGE/CRC）的值起始列
#define HOST_UI_UNIT_X      (122)       // AGE 行的 "ms" 单位列（AGE 定宽 6 字符 -> 到 x=113）

static void Host_Crtl_UI (void)
{
    ips200_show_string(8  ,0  , "[HOST]");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "Host Mode:  ");       // 补到 12 字符 -> 值对齐到 x=114
    ips200_show_string(10 ,48 , "Host Link:  ");       // 补到 12 字符 -> 同上
    ips200_show_string(10 ,64 , "Remote Crtl:");
    ips200_show_string(10 ,80 , "State:");             // 原叫 "LINK:" —— 与开关 "Host Link" 撞名，
                                                       // 一个管"通不通"一个管"现在什么状态"，分开叫
    ips200_show_string(10 ,96 , "AGE:");
    ips200_show_string(HOST_UI_UNIT_X ,96 , "ms");
    ips200_show_string(10 ,112, "CRC:");
    ips200_show_string(10 ,144, "BACK: MENU");
}

// 三行开关的值文字，一律定宽 7 字符 ——
// 定宽才不会因为新旧字符串长度不同在屏幕上留残影（"RC LOST" 比 "OFF" 长四格）
static const char *host_mode_text (uint8 m)
{
    return (HOST_MODE_CMD == m) ? "CMD    " : "UP     ";
}

static const char *host_rc_text (void)
{
    if (RC_ENGAGE_ESTOP    == RC_Engage) { return "ESTOP  "; }
    if (RC_ENGAGE_FAILSAFE == RC_Engage) { return "RC LOST"; }   // 失控挂起：接收机没信号（锁存）
    if (RC_ENGAGE_OFF      == RC_Engage) { return "OFF    "; }

    // ★ ON 分两种，必须分开显示 —— 否则 CMD + CH6 = 0 时会让人以为遥控正在开车，
    //   实际方向盘还在上位机手里
    //     DRIVE = 采信了且驾驶权在这里（UP，或 CMD 且 CH6 = 1）
    //     READY = 采信了但驾驶权不在（CMD 且 CH6 = 0，上位机在开）
    return (HOST_AUTH_REMOTE == Host_Auth) ? "DRIVE  " : "READY  ";
}

// LINK 的四种文字。★ 与遥测通道 HOST_CH_ONLINE 不同源，理由见 Host_Crtl.h 的 HOST_ST_* 段
static const char *host_link_text (void)
{
    if (HOST_LINK_OFF == Host_Link)  { return "IDLE   "; }
    if (HOST_MODE_UP   == Host_Mode) { return "TX ON  "; }
    if (0 != Host_Lost)              { return "LOST   "; }

    return "ACTIVE ";
}

// 画三行开关的值（只画值，不画标签 —— 标签在 Host_Crtl_UI 里一次画完）
static void host_ui_switch (void)
{
    ips200_show_string(HOST_UI_VAL_X, 32, host_mode_text(Host_Mode));
    ips200_show_string(HOST_UI_VAL_X, 48, (HOST_LINK_ON == Host_Link) ? "ON     " : "OFF    ");
    ips200_show_string(HOST_UI_VAL_X, 64, host_rc_text());
}

// 画光标：三行全擦再画当前行。
// 比"记住上次位置只擦旧行"多两次写，但不会因为某次漏擦留残影，换个键盘布局也不容易写错
static void host_ui_cursor (uint8 row)
{
    uint16 y;

    if (row > 2) { row = 2; }
    y = (uint16)(HOST_UI_ROW_Y0 + HOST_UI_ROW_DY * row);

    ips200_show_string(HOST_UI_CUR_X, 32, " ");
    ips200_show_string(HOST_UI_CUR_X, 48, " ");
    ips200_show_string(HOST_UI_CUR_X, 64, " ");
    ips200_show_string(HOST_UI_CUR_X, y , ">");
}

// 屏幕输出：右对齐定宽的十进制。
// 不用 "%5d" 之类的宽度格式（IPS200 的 printf 是裁剪版，宽度不一定支持），
// 也顺便保证"新值比旧值短"时右侧被空格覆盖、不留残影。
static void Host_ShowU (uint16 x, uint16 y, uint32 v, uint8 width)
{
    char  s[12];
    uint8 i;

    if (width > 10) { width = 10; }
    for (i = 0; i < width; i ++) { s[i] = ' '; }
    s[width] = 0;

    i = width;
    do
    {
        i --;
        s[i] = (char)('0' + (uint8)(v % 10u));
        v /= 10u;
    } while ((0 != v) && (0 != i));

    ips200_show_string(x, y, s);
}

//--------------------------------------
// 函数简介     画状态区：LINK / AGE / CRC + 锁存提示
// 参数说明     void
// 使用示例     Host_Crtl_Page() 的 300ms 节拍、以及每次按键之后
// 备注信息     ★ UP 模式的 AGE 画 "---" 不画数字：本模式不判心跳，`host_alive_ms` 只在
//              收到帧时刷新，画出来就是个一直在涨的数 —— 会被误读成"链路快断了"。
//              ★ CMD 模式的 AGE 也**不是延迟曲线**：上位机 PING 每 100ms 一次，
//              正常值就是 0→100 的锯齿。一路涨过 HOST_ALIVE_MS 才是真掉线
//              （与队友那个"k_z 列其实是毫秒计数器"的坑同源，录 CSV 时别当延迟解释）。
//--------------------------------------
static void host_ui_status (void)
{
    uint32 age;

    ips200_show_string(HOST_UI_STA_X, 80, host_link_text());

    if (HOST_MODE_UP == Host_Mode)
    {
        ips200_show_string(HOST_UI_STA_X, 96, "  --- ");    // 6 字符，与 Host_ShowU 的定宽一致
    }
    else
    {
        age = (0 == host_ever_rx) ? 999999u : (uint32)(Sys_Tick_Ms - host_alive_ms);
        Host_ShowU(HOST_UI_STA_X, 96, age, 6);
    }

    Host_ShowU(HOST_UI_STA_X, 112, Host_Crtl_Crc_Err(), 6);

    // 锁存提示：只在异常态出现，正常态清空（这就是"屏幕只减不增"里唯一那处新增）
    if (0 != Host_Lost)
    {
        ips200_show_string(10, 128, "LOST! CONFIRM: RESET");
    }
    else
    {
        ips200_show_string(10, 128, "                    ");
    }
}

//--------------------------------------
// 函数简介     页面内的连接流程：走 Host_Crtl_Connect，然后整屏重画页面
// 参数说明     void
// 返回参数     0 = 链路已就绪，留在 Host 页继续
//              1 = 用户在 WiFi 选择界面按了 BACK，**调用者应退出本页回主菜单**
// 使用示例     Host_Crtl_Page() 进门时链路未建立 —— **唯一的调用点**（页内没有重连键）
// 备注信息     ★ 成功回来时必须整屏重画：WiFi 选择界面自己 ips200_clear 过，
//              不重画的话屏幕上会留着那张热点列表（取消时不重画 —— 马上要退回主菜单，
//              菜单那边会 ips200_clear）
//              ★ "link up / link FAIL" 那两句提示已删：LINK 行现在会明确显示 IDLE / TX ON，
//                再叠一句同义文字只是占地方。仍然保留 "connecting..."，因为连 WiFi 的
//                那十几秒里屏幕本来就是空的，没这句会让人以为死机
//--------------------------------------
static uint8 host_page_connect (void)
{
    uint8 ret;

    // 定宽 28 字符 = x=10 起一行能写满的上限（见本文件 UI 段开头的屏幕尺寸说明）。
    // ⚠ 这里原先是 30 字符，x=10 起会画到 x=249，超 240 宽 → ips200 断言，进页面即崩
    ips200_show_string(10, 128, "connecting...               ");

    ret = Host_Crtl_Connect();

    if (HOST_ERR_NOT_INIT == ret)
    {
        // 用户在 WiFi 选择界面按了 BACK —— 取消进入，交回调用者退出本页
        return 1;
    }

    ips200_clear();
    Host_Crtl_UI();
    host_ui_switch();
    host_ui_status();
    host_ui_cursor(0);

    Time_Count1 = HOST_UI_REFRESH_TICKS;    // 直接顶到阈值 -> 下一圈立刻刷状态区，不用再等 300ms

    return 0;
}

int Host_Crtl_Page (void)
{
    uint8 row   = 0;            // 光标行 0..2（Mode / Link / Remote）
    uint8 dirty = 1;            // 置位 = 下一圈整屏重画（按键后立刻生效，不等 300ms）
    uint8 auth;                 // 本圈的驾驶权（HOST_AUTH_*）

    // ★ 进门先复位：链路关、模式回 UP、遥控关、锁存清、驱动侧失能（Release 一次做全）。
    //   这样"每次进来都是仅上行 + 默认关闭"是必然结果，不依赖上一页留下什么
    Host_Crtl_Release();

    Host_Crtl_UI();
    host_ui_switch();
    host_ui_status();
    host_ui_cursor(row);

    // 参考计时值重置
    Time_Count1 = 0;
    Time_Count2 = 0;

    // ★ 进入页面即检查链路（"进门先连"）：
    //   已建立（例如调试页连过）-> 直接进页面，不弹任何界面
    //   未建立 -> 弹 WiFi 选择界面（上下键选热点 / 确认键连 / BACK 回主菜单）
    //   ★ 弹出来的是选择界面本身，不阻塞 —— 只有按确认键选定热点才真正跑 init（最长十几秒）
    if (0 == wifi_spi_inited)
    {
        if (0 != host_page_connect())
        {
            return 0;               // 用户在 WiFi 界面按了 BACK -> 回主菜单
        }
    }

    while(1)
    {
        if (KEY_SHORT_PRESS == key_get_state(KEY_BACK))
        {
            key_clear_state(KEY_BACK);
            Host_Crtl_Release();        // 退出即全关：链路归零、遥控收回、驱动侧失能
            return 0;
        }

        // 心跳：本页面在监督后轮闭环，必须每圈喂一次。
        // 漏喂 -> Motor_Crtl 的心跳门会一直判定"监督循环已死"，电机输出恒为 0。
        Motor_Crtl_HB_Feed();

        /* ---------- 按键 ---------- */
        if (KEY_SHORT_PRESS == key_get_state(KEY_UP))
        {
            key_clear_state(KEY_UP);
            row   = (0 == row) ? 2 : (uint8)(row - 1);
            dirty = 1;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_DOWN))
        {
            key_clear_state(KEY_DOWN);
            row   = (2 == row) ? 0 : (uint8)(row + 1);
            dirty = 1;
        }
        else if (KEY_SHORT_PRESS == key_get_state(KEY_CONFIRM))
        {
            key_clear_state(KEY_CONFIRM);
            dirty = 1;

            if (0 != Host_Lost)
            {
                // ★ 锁存态下确认键 = 复位，**不去切开关**：
                //   否则会出现"想复位、结果把模式改了"——锁存时人的第一反应是找复位
                Host_Crtl_Lost_Clear();
            }
            else if (0 == row)          // Host Mode
            {
                // Link = ON 时 Set_Mode 内部直接返回（切不动），屏幕值不变即是全部反馈
                Host_Crtl_Set_Mode((HOST_MODE_CMD == Host_Mode) ? HOST_MODE_UP : HOST_MODE_CMD);
            }
            else if (1 == row)          // Host Link
            {
                Host_Crtl_Set_Link((HOST_LINK_ON == Host_Link) ? HOST_LINK_OFF : HOST_LINK_ON);
            }
            else                        // Remote Crtl
            {
                // ★★ 不再要求 Link = ON（2026-10-17 语义收窄）：遥控走 UART 接收机，
                //    与 WiFi 链路无关 —— "UP + Link OFF + 遥控开车"是合法组合（纯本地跑车，不上发）。
                // ★ 仍然沿用遥控页的两条纪律：接收机在线、CH5 不在急停位才允许切换。
                //    少这两条，按一下确认就能把急停解掉 —— 非常危险。
                // ★ 失控挂起（RC LOST）下按确认 = **尝试重连**：要求 RC_Online 已恢复才放行，
                //    信号没回来时按了不会有任何反应，屏幕仍显示 RC LOST —— 那就是"重连失败"的反馈。
                //    软件无法主动重连接收机（RC_Online 是 uart_receiver.state 的硬件自报状态），
                //    只能等它自己重新同步，所以这里只负责"允许它回到接管态"。
                if (RC_Online && (0 == RC_Sw5))
                {
                    uint8 was_on = (RC_ENGAGE_ON == RC_Engage) ? 1 : 0;

                    RC_Engage = was_on ? RC_ENGAGE_OFF : RC_ENGAGE_ON;

                    // ★ 遥控收回的这一刻必须把驱动侧收干净：RC_ENGAGE_OFF 之后本页就不再调
                    //   Remote_Crtl_Apply()（它只在 OFF 分支才清目标），而 UP 模式下
                    //   host_drive_tick() 又直接返回 —— 不收就会出现
                    //   "遥控关了、车还按最后那个遥控目标在跑"
                    // ★ 只在"它真的在开车"时才清：CMD + CH6 = 0 时驾驶权在上位机，
                    //   遥控只是 READY，这时清驱动会平白打断上位机一拍输出
                    // ★ 用 host_actuator_clear 而不是 host_drive_clear：
                    //   遥控松手不该顺带作废上位机的使能位图（见两个函数的注释）
                    if ((0 != was_on) && (HOST_AUTH_REMOTE == Host_Auth))
                    {
                        host_actuator_clear();
                    }
                }
            }
        }

        /* ---------- 驱动侧仲裁：STOP > 遥控 > 上位机（见文件头"驾驶权仲裁"） ---------- */
        Remote_Crtl_Update();       // 遥测通道要用这些量，必须每圈跑（纯算术，开销可忽略）
        host_rc_watch();            // CH5 急停 / 接收机失联：任何 Mode、任何 Link 都生效

        auth = host_auth_resolve();

        // ★ 驾驶权换手时先把驱动侧收干净：交出的那一方（尤其是遥控）不会再来清目标，
        //   不收就会"遥控让出驾驶权、车还按最后那个遥控目标跑"
        // ★ 只在"遥控 -> 上位机/无人"这一种换手上清：
        //     · 遥控 -> STOP 不用清，Apply 的 ESTOP/FAILSAFE 分支本来就会以目标 0 覆盖
        //     · 其它方向也不用清，接手的那个分支第一句就把四个量全写出来了
        //   多清一次会让 Servo/Motor_Enable 在同一圈里 1 -> 0 -> 1，撞上 ISR 就多写一拍 0
        if ((HOST_AUTH_REMOTE == Host_Auth) && (HOST_AUTH_REMOTE != auth) && (HOST_AUTH_STOP != auth))
        {
            host_actuator_clear();
        }
        Host_Auth = auth;

        if ((HOST_AUTH_REMOTE == auth) || (HOST_AUTH_STOP == auth))
        {
            Remote_Crtl_Apply();    // 遥控掌权 / 锁存停车：驱动侧全由遥控侧写
        }
        else
        {
            host_drive_tick();      // 上位机掌权（含 CMD 心跳门与锁存刹停）；NONE 时内部不动
        }

        /* ---------- 链路层（Link 门控在内部） ---------- */
        Host_Crtl_Task();

        /* ---------- 屏幕 ---------- */
        if (0 != dirty)
        {
            dirty       = 0;
            Time_Count1 = 0;
            host_ui_switch();
            host_ui_status();
            host_ui_cursor(row);
        }
        else if (Time_Count1 >= HOST_UI_REFRESH_TICKS)
        {
            Time_Count1 = 0;
            host_ui_switch();       // 值会被状态机改（心跳锁存、接收机失控…），一起重画最省心
            host_ui_status();
        }
    }
}

/**********************************************************/
/*----------------------------------------[E] 页面 [E]----*/
/**********************************************************/
