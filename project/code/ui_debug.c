/*********************************************************************************************************************
 * 板载调试页：诊断仪表，6 个视图（两级：先选视图，再进详情）
 *
 *  id 标题       内容
 *   0 MOTOR      后轮：实测/目标/斜坡后目标/累计计数/单拍增量/输出占空比/开环/使能门/M_EN
 *   1 IMU        yaw/姿态/陀螺 z/链路与器件 ID/零偏/位姿 x y/里程标志；K3 = yaw 归零
 *   2 SERVO      编码器链（整帧/滤波后/未滤波/减中位/度）+ 位置环（施加目标/指令目标/误差/输出）
 *   3 UART-RC    接收机 6 通道原始值/中位/归一化/ENGAGE 与在线/摇杆异常位图
 *   4 SYS        安全层三态/驱动令牌/链路状态/路表槽/参数加载/回放/跑班/日志落盘
 *   5 WIFI       热点与目标选择 + 连接状态；进入本页会关掉速度环与舵机内环
 *
 * 显示顺序见 S_ORDER[]（WIFI 第一），与 id 无关，加视图不必改 id。
 * 绘制策略：进页或切视图时整页重画（含标签）；数值行每 200 ms 只重画**内容变了**的行
 * （s_vcache 比较），避免每拍全量重写把 200 ms 节拍拖长。
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <stdio.h>
#include <string.h>
#include "ui_core.h"
#include "ui_debug.h"
#include "spd_enc.h"
#include "spd_ctrl.h"
#include "spd_safety.h"
#include "steer_ctrl.h"
#include "steer_abs.h"
#include "ins_imu.h"
#include "ins_odom.h"
#include "remote_ctrl.h"
#include "proto.h"
#include "ins_route.h"
#include "route_app.h"
#include "app_params.h"
#include "motor_link.h"
#include "spd_bench.h"
#include "bench_log.h"
#include "route_pb.h"

/* ---- 版式：标签 0 列起，值 8 列起；行 0 标题、行 16 提示 ----
   值列从 6 移到 8，是为了容纳 ctl_tar / app_tar / sum_cnt 这些 7 字符的标签。
   8 + 22 = 30 字符 = 240 px，正好占满屏宽。 */
#define DBG_ROW_VAL0    (1u)
#define DBG_COL_VAL     (8u)
#define DBG_ROW_HINT    (16u)

#define DBG_VIEWS       (6u)
#define DBG_MAX_ROWS    (11u)   /* 单视图最多标签行；留一位给 NULL */

/* 视图 id。**显示顺序**由 S_ORDER 决定，所以 id 只要求稳定、不要求有序。
   SERVO 复用了原 ABS-ENC 的 id 2（两者已合并），这样 dbg_value() 里
   其它 case 的序号一个都不用动。 */
#define DBG_VIEW_SERVO  (2u)
#define DBG_VIEW_WIFI   (5u)

/* 两级：先选视图，再进详情（= 本项目旧版的使用方式：列表选 + 确认进入）*/
#define DBG_PAGE_LIST   (0u)
#define DBG_PAGE_DETAIL (1u)
#define DBG_REFRESH_MS  (200u)

static const char *const S_TITLE[DBG_VIEWS] =
{
    "MOTOR", "IMU", "SERVO", "UART-RC", "SYSTEM", "WIFI"
};

/* 视图选择列表的**显示顺序**：WIFI 第一，SERVO 紧跟 MOTOR */
static const uint8 S_ORDER[DBG_VIEWS] =
{
    DBG_VIEW_WIFI, 0u, DBG_VIEW_SERVO, 1u, 3u, 4u
};

/* 每个视图的标签；NULL 结尾。行号 = 下标 + DBG_ROW_VAL0
   WIFI 视图版式特殊（候选列表 + 明细），不走这张表，故留空 */
static const char *const S_LABEL[DBG_VIEWS][12] =
{
    /* 0 MOTOR —— 只放后轮；使能位（M_EN）已移到 SYSTEM，进本页会自动关环 */
    { "rps",     "ctl_tar", "app_tar", "sum_cnt", "cnt",
      "duty",    "ol",      "en",      NULL },
    /* 1 IMU   */
    { "yaw",  "att",  "gz",  "link", "bias", "odom", "flags", NULL },
    /* 2 SERVO —— 编码器链（5 行） + 位置环（4 行），原 ABS-ENC 已并入 */
    { "frame",   "raw",   "live",    "ctr",   "deg",
      "app_tar", "ctl_tar", "err",   "duty",  NULL },
    /* 3 RC    */
    { "ch12", "ch34", "ch56", "mid",  "norm", "eng",  "bad",  NULL },
    /* 4 SYSTEM —— 使能位集中放这里（MOTOR/SERVO 进页会关环，那两个页面上看会误导） */
    { "safe", "M_EN", "S_EN", "drv",  "link", "slot",
      "parm", "pb",   "bench", "store", NULL },
    /* 5 WIFI  */
    { NULL },
};

static uint8  s_view = 0u;       /* 详情页当前视图 */
static uint8  s_sel  = 0u;       /* 列表页光标 */
static uint8  s_page = 0u;       /* DBG_PAGE_* */
static uint32 s_last = 0u;

/* 值行缓存：记录每行上一次真正画出去的字符串。
   IPS200 写一行 20+ 字符约 1000+ 像素，SPI 慢时一次刷新要几十毫秒；
   不比较就重写会让 200 ms 节拍被拖长，所以"没变就不写"。 */
#define DBG_CACHE_LEN   (24u)
static char   s_vcache[DBG_MAX_ROWS][DBG_CACHE_LEN];

static void dbg_f2 (char *b, uint8 n, float v) { ui_ftoa(b, n, v, 2u); }
static void dbg_f1 (char *b, uint8 n, float v) { ui_ftoa(b, n, v, 1u); }

/* 把某视图某行的"值"写进 buf；返回 0 = 该行没有值（不该画） */
static uint8 dbg_value (uint8 view, uint8 row, char *buf, uint8 size)
{
    switch (view)
    {
        /* ---------------- MOTOR ---------------- */
        case 0u:
            switch (row)
            {
                case 0u:  /* 实测 rps */
                {
                    char a[12], b[12];
                    dbg_f2(a, sizeof(a), enc_l.rps);
                    dbg_f2(b, sizeof(b), enc_r.rps);
                    snprintf(buf, size, "L%s R%s", a, b);
                    return 1u;
                }
                case 1u:  /* 目标 rps（上位机/遥控下发）*/
                {
                    char a[12], b[12];
                    dbg_f2(a, sizeof(a), spd_target_l);
                    dbg_f2(b, sizeof(b), spd_target_r);
                    snprintf(buf, size, "L%s R%s", a, b);
                    return 1u;
                }
                case 2u:  /* 斜坡之后真正施加的目标 */
                {
                    char a[12], b[12];
                    dbg_f2(a, sizeof(a), ctl_l.r_app);
                    dbg_f2(b, sizeof(b), ctl_r.r_app);
                    snprintf(buf, size, "L%s R%s", a, b);
                    return 1u;
                }
                case 3u:  /* 累计计数（标定 cpr 用）*/
                    snprintf(buf, size, "L%d R%d", (int)enc_l.pos, (int)enc_r.pos);
                    return 1u;
                case 4u:  /* 本拍计数增量（滤毛刺后）*/
                    snprintf(buf, size, "L%d R%d", (int)enc_l.delta, (int)enc_r.delta);
                    return 1u;
                case 5u:  /* 控制器输出（归一化 ×100 = %）*/
                {
                    char a[12], b[12];
                    dbg_f1(a, sizeof(a), ctl_l.u * 100.0f);
                    dbg_f1(b, sizeof(b), ctl_r.u * 100.0f);
                    snprintf(buf, size, "L%s R%s%%", a, b);
                    return 1u;
                }
                case 6u:  /* 开环：绕过速度环直接给占空比 */
                {
                    char a[12], b[12];
                    dbg_f1(a, sizeof(a), spd_ol_duty_l);
                    dbg_f1(b, sizeof(b), spd_ol_duty_r);
                    snprintf(buf, size, "en%u L%s R%s",
                             (unsigned)(spd_ol_en > 0.5f), a, b);
                    return 1u;
                }
                case 7u:  /* 本拍是否允许出力 / 是否被夹 */
                    snprintf(buf, size, "L%u R%u sat%u%u",
                             (unsigned)ctl_l.enabled, (unsigned)ctl_r.enabled,
                             (unsigned)ctl_l.sat, (unsigned)ctl_r.sat);
                    return 1u;
                default: return 0u;
            }

        /* ---------------- IMU ---------------- */
        case 1u:
            switch (row)
            {
                case 0u:
                    dbg_f2(buf, size, ins_imu_get_yaw());
                    return 1u;
                case 1u:
                {
                    char a[12], b[12];
                    dbg_f2(a, sizeof(a), ins_imu_get_pitch());
                    dbg_f2(b, sizeof(b), ins_imu_get_roll());
                    snprintf(buf, size, "P%s R%s", a, b);
                    return 1u;
                }
                case 2u:
                    dbg_f2(buf, size, ins_imu_get_gyro_z_dps());
                    return 1u;
                case 3u:
                {
                    uint8 id0f = 0u, id00 = 0u, idv = 0u;
                    ins_imu_dev_id_get(&id0f, &id00, &idv);
                    snprintf(buf, size, "ok%u id%02X/%02X",
                             (unsigned)ins_imu_link_ok(), (unsigned)id0f, (unsigned)id00);
                    return 1u;
                }
                case 4u:
                {
                    float bx = 0.0f, by = 0.0f, bz = 0.0f;
                    char a[12], b[12], c[12];
                    ins_diag_get_bias(&bx, &by, &bz);
                    dbg_f1(a, sizeof(a), bx);
                    dbg_f1(b, sizeof(b), by);
                    dbg_f1(c, sizeof(c), bz);
                    snprintf(buf, size, "%s/%s/%s", a, b, c);
                    return 1u;
                }
                case 5u:
                {
                    char a[12], b[12];
                    dbg_f2(a, sizeof(a), ins_odom_get_x());
                    dbg_f2(b, sizeof(b), ins_odom_get_y());
                    snprintf(buf, size, "x%s y%s", a, b);
                    return 1u;
                }
                case 6u:
                {
                    char a[12];
                    dbg_f2(a, sizeof(a), ins_odom_get_dist());
                    snprintf(buf, size, "fl%02X d%s", (unsigned)ins_odom_flags(), a);
                    return 1u;
                }
                default: return 0u;
            }

        /* ---------------- SERVO（原 ABS-ENC 已并入）---------------- */
        case DBG_VIEW_SERVO:
            switch (row)
            {
                /* 前 5 行：编码器链，顺序就是数据流：帧 -> 滤波后 -> 未滤波 -> 减中位 -> 度
                   bits[15:4] 才是 12 位角度（详见 steer_abs.h 的量纲说明），bits[3:0] 驱动未解析 */
                case 0u: snprintf(buf, size, "%04X", (unsigned)steer_abs_get_frame());     return 1u;
                case 1u: snprintf(buf, size, "%u", (unsigned)steer_abs_get_raw());         return 1u;
                case 2u: snprintf(buf, size, "%u", (unsigned)steer_abs_get_raw_live());    return 1u;
                case 3u: snprintf(buf, size, "%d", (int)steer_abs_get_center_delta());     return 1u;
                case 4u: dbg_f2(buf, size, steer_abs_get_deg());                           return 1u;

                /* 后 4 行：位置环。注意 meas 就是上面那行 deg（steer_ctrl.c:221 同一来源），故不重复显示 */
                case 5u: dbg_f1(buf, size, steer.applied_delta); return 1u;   /* 实际施加目标（软限位+斜坡后）*/
                case 6u: dbg_f1(buf, size, steer.target_delta);  return 1u;   /* 指令目标 */
                case 7u: dbg_f1(buf, size, steer.err);           return 1u;   /* 本拍误差 */
                case 8u:  /* 输出占空比：output_duty 是 -10000~10000 的 duty，换成 % */
                {
                    char a[12];
                    dbg_f1(a, sizeof(a), (float)steer.output_duty * 0.01f);
                    snprintf(buf, size, "%s%%", a);
                    return 1u;
                }
                default: return 0u;
            }

        /* ---------------- UART-RC ---------------- */
        case 3u:
            switch (row)
            {
                case 0u:
                    snprintf(buf, size, "%u / %u", (unsigned)RC_Raw_Ch1, (unsigned)RC_Raw_Ch2);
                    return 1u;
                case 1u:
                    snprintf(buf, size, "%u / %u", (unsigned)RC_Raw_Ch3, (unsigned)RC_Raw_Ch4);
                    return 1u;
                case 2u:
                    snprintf(buf, size, "%u / %u", (unsigned)RC_Raw_Ch5, (unsigned)RC_Raw_Ch6);
                    return 1u;
                case 3u:
                    snprintf(buf, size, "T%u S%u", (unsigned)RC_Thr_Mid, (unsigned)RC_Steer_Mid);
                    return 1u;
                case 4u:
                {
                    char a[12], b[12];
                    dbg_f2(a, sizeof(a), RC_Throttle);
                    dbg_f2(b, sizeof(b), RC_Steer_Deg);
                    snprintf(buf, size, "%s / %s", a, b);
                    return 1u;
                }
                case 5u:
                    snprintf(buf, size, "e%u on%u", (unsigned)RC_Engage, (unsigned)RC_Online);
                    return 1u;
                case 6u:
                    snprintf(buf, size, "%02X", (unsigned)RC_Stick_Bad);
                    return 1u;
                default: return 0u;
            }

        /* ---------------- SYSTEM ---------------- */
        case 4u:
            switch (row)
            {
                case 0u:
                    snprintf(buf, size, "al%u loc%u est%u",
                             (unsigned)spd_safety_allow(0u),
                             (unsigned)spd_safety_local(),
                             (unsigned)spd_safety_estop());
                    return 1u;
                case 1u:  /* M_EN 后轮总使能：生效结论 / 本地接管 / 上位机使能位图
                             allow = 五道门的总结论（0 = 任一门不过）
                             loc   = 本地接管（遥控/回放），此时上位机三道门不参与
                             h     = 上位机下发的位图：bit1(0x0002) 电机 + bit2(0x0004) 总使能 */
                    snprintf(buf, size, "allow%u loc%u h%04X",
                             (unsigned)spd_safety_allow(0u),
                             (unsigned)spd_safety_local(),
                             (unsigned)motor_link_enable_bits());
                    return 1u;
                case 2u:  /* S_EN 舵机：内环 / 外环 / 目标归属 / 故障位 */
                    snprintf(buf, size, "in%u hd%u own%u f%02X",
                             (unsigned)steer.angle_enable,
                             (unsigned)steer.head_enable,
                             (unsigned)route_pb_owns_target(),
                             (unsigned)steer_ctrl_fault());
                    return 1u;
                case 3u:  /* 驱动令牌持有者: 0 无 / 1 遥控 / 2 上位机 / 3 菜单 */
                    snprintf(buf, size, "%u", (unsigned)proto_drv_owner());
                    return 1u;
                case 4u:
                    snprintf(buf, size, "err%u on%u rx%u",
                             (unsigned)motor_link_init_err(),
                             (unsigned)motor_link_online(),
                             (unsigned)motor_link_ever_rx());
                    return 1u;
                case 5u:
                    snprintf(buf, size, "S%u %upt %s",
                             (unsigned)ins_route_slot_active(),
                             (unsigned)route_app_count(),
                             (0u != ins_route_is_running()) ? "REC" :
                             ((0u != ins_route_flash_busy()) ? "SAV" : "ok"));
                    return 1u;
                case 6u:
                    snprintf(buf, size, "ld%u e%u",
                             (unsigned)app_param_loaded(),
                             (unsigned)app_param_last_err());
                    return 1u;
                case 7u:
                    /* 回放：使能位 / 状态码 / 进度索引 */
                    snprintf(buf, size, "en%u st%u i%u",
                             (unsigned)(pb_en > 0.5f),
                             (unsigned)route_pb_state(),
                             (unsigned)route_pb_index());
                    return 1u;
                case 8u:
                    /* 跑班：状态 / 段号 / 日志条数 */
                    snprintf(buf, size, "st%u seg%u n%u",
                             (unsigned)spd_bench_state(),
                             (unsigned)spd_bench_seg(),
                             (unsigned)bench_log_count());
                    return 1u;
                case 9u:
                    /* 日志落盘 / 参数加载 */
                    snprintf(buf, size, "sd%u ld%u e%u",
                             (unsigned)bench_log_stored(),
                             (unsigned)app_param_loaded(),
                             (unsigned)app_param_last_err());
                    return 1u;
                default: return 0u;
            }

        default: return 0u;
    }
}

/* ================= WIFI 视图（版式特殊，不走 S_LABEL 表）=================
   row0  标题
   row1~4  4 个候选，光标 "> " 在选中那行
   row5  空行
   row6  SSID:
   row7  IP  :
   row8  PORT: 8086 UDP
   row9  st  : --
   row16 提示
   进本视图即关后轮速度环与舵机内环（配置链路时车必须完全不动）。 */
#define DBG_WIFI_ROW0   (1u)
#define DBG_WIFI_DET    (6u)

static uint8 s_wifi_sel = 0u;       /* 候选光标 */
static uint8 s_wifi_st  = 0u;       /* 0 未连 / 1 连接中 / 2 成功 / 3 失败 */

/* ---- 进详情页关环 / 退出恢复 ----
   ⚠ 舵机不能用 steer_ctrl_set_angle_enable(0) 直接关：steer_ctrl_update() 每 5 ms 会把
     steer.angle_enable 同步回 steer_en_cmd（steer_ctrl.c:210-218），下一拍就又使能了。
     真要关必须写 steer_en_cmd = 0。
   ⚠ 后轮 coast 没有任何人会自动清（只有 route_pb 的少数路径清），所以离开页面必须恢复，
     否则车会一直自由轮。进出各存一次原值，退出时还原。 */
static uint8 s_loops_saved = 0u;        /* 1 = 已经关过环，退出要恢复 */
static uint8 s_coast_saved = 0u;
static float s_steer_en_saved = 0.0f;

static void dbg_loops_off (void)
{
    if (0u != s_loops_saved) { return; }

    s_coast_saved    = spd_ctrl_get_coast();
    s_steer_en_saved = steer_en_cmd;

    spd_ctrl_set_coast(1u);             /* 后轮自由轮：输出钉 0，测速/里程照常 */
    spd_target_l = 0.0f;
    spd_target_r = 0.0f;

    steer_en_cmd = 0.0f;                /* 这个才是真正生效的舵机开关 */
    steer_ctrl_set_angle_enable(0u);    /* 立即生效，不等下一拍 */

    s_loops_saved = 1u;
}

/* 退出详情页：把进页时保存的状态还原（回放设的 coast 也不会被清掉）*/
static void dbg_loops_restore (void)
{
    if (0u == s_loops_saved) { return; }

    spd_ctrl_set_coast(s_coast_saved);
    steer_en_cmd = s_steer_en_saved;

    s_loops_saved = 0u;
}

static void dbg_wifi_det (void)
{
    const motor_ap_t *ap = &motor_link_ap_list()[s_wifi_sel];
    char line[40];

    snprintf(line, sizeof(line), "SSID: %s", ap->ssid);
    ui_puts_row(DBG_WIFI_DET + 0u, 0u, line);
    snprintf(line, sizeof(line), "IP  : %s", ap->ip);
    ui_puts_row(DBG_WIFI_DET + 1u, 0u, line);
    ui_puts_row(DBG_WIFI_DET + 2u, 0u, "PORT: 8086 UDP");
}

static void dbg_wifi_st (void)
{
    static char s_cache[40] = "";       /* 与上一帧比，没变就不写（省 SPI）*/
    char line[40];

    switch (s_wifi_st)
    {
        case 1u:  snprintf(line, sizeof(line), "st  : CONNECT...            "); break;
        case 2u:  snprintf(line, sizeof(line), "st  : ok  online%u ever%u   ",
                           (unsigned)motor_link_online(), (unsigned)motor_link_ever_rx()); break;
        case 3u:  snprintf(line, sizeof(line), "st  : fail %u               ",
                           (unsigned)motor_link_init_err()); break;
        default:  snprintf(line, sizeof(line), "st  : -- (not connected)    "); break;
    }

    if (0 != strcmp(line, s_cache))
    {
        memcpy(s_cache, line, sizeof(s_cache));
        s_cache[sizeof(s_cache) - 1u] = '\0';
        ui_puts_row(DBG_WIFI_DET + 3u, 0u, line);
    }
}

static void dbg_wifi_draw (void)
{
    uint8 i;
    char  line[40];

    ui_clear();

    snprintf(line, sizeof(line), "DEBUG %u/%u  WIFI%s", (unsigned)(s_sel + 1u),
             (unsigned)DBG_VIEWS, (0u != s_loops_saved) ? " LOOP-OFF" : "");
    ui_puts_row(0u, 0u, line);

    for (i = 0u; i < (uint8)MOTOR_AP_COUNT; i ++)
    {
        snprintf(line, sizeof(line), "%s%s", (i == s_wifi_sel) ? "> " : "  ",
                 motor_link_ap_list()[i].label);
        ui_puts_row((uint8)(DBG_WIFI_ROW0 + i), 0u, line);
    }

    dbg_wifi_det();
    dbg_wifi_st();

    ui_puts_row(DBG_ROW_HINT, 0u, "K1/K2=Pick K3=Conn K4=Bk");
}

/* 候选光标移动：只改两行 */
static void dbg_wifi_cursor (uint8 old)
{
    char line[40];

    snprintf(line, sizeof(line), "  %s", motor_link_ap_list()[old].label);
    ui_puts_row((uint8)(DBG_WIFI_ROW0 + old), 0u, line);

    snprintf(line, sizeof(line), "> %s", motor_link_ap_list()[s_wifi_sel].label);
    ui_puts_row((uint8)(DBG_WIFI_ROW0 + s_wifi_sel), 0u, line);

    dbg_wifi_det();
}

/* 画某视图的标签 + 标题 + 提示（整页重画时用） */
static void dbg_draw_static (void)
{
    uint8 r;
    char  line[40];

    ui_clear();

    /* 整屏已清空：缓存必须跟着失效，否则第一次刷新会以为"没变"而不画 */
    memset(s_vcache, 0, sizeof(s_vcache));

    /* 标题带上环状态：这三个页面进来就把闭环关了，屏上必须看得见 */
    snprintf(line, sizeof(line), "DEBUG %u/%u  %s%s", (unsigned)(s_sel + 1u),
             (unsigned)DBG_VIEWS, S_TITLE[s_view],
             (0u != s_loops_saved) ? "  LOOP-OFF" : "");
    ui_puts_row(0u, 0u, line);

    for (r = 0u; r < DBG_MAX_ROWS; r ++)
    {
        if (NULL == S_LABEL[s_view][r]) { break; }
        ui_puts_row((uint8)(DBG_ROW_VAL0 + r), 0u, S_LABEL[s_view][r]);
    }

    /* 详情页：返回键回列表；IMU 视图多一个动作键（yaw 归零）*/
    ui_puts_row(DBG_ROW_HINT, 0u, (1u == s_view) ? "K3=yaw0 K4=Bk" : "K4=Bk");
}

/* 只重画值列：内容没变的行直接跳过（见 s_vcache 的说明） */
static void dbg_draw_values (void)
{
    uint8 r;
    char  buf[40];
    char  out[DBG_CACHE_LEN + 8u];

    for (r = 0u; r < DBG_MAX_ROWS; r ++)
    {
        uint8 n_new;
        uint8 n_old;
        uint8 n_out;
        uint8 i;

        if (NULL == S_LABEL[s_view][r]) { break; }

        buf[0] = '\0';
        if (0u == dbg_value(s_view, r, buf, sizeof(buf))) { continue; }

        n_new = (uint8)strlen(buf);
        n_old = (uint8)strlen(s_vcache[r]);

        if ((n_new == n_old) && (0 == strcmp(buf, s_vcache[r]))) { continue; }  /* 没变 */

        /* 新串比旧串短时补空格，避免旧字符残留；一次写入，不做"先擦后写" */
        n_out = (n_old > n_new) ? n_old : n_new;
        if (n_out > (DBG_CACHE_LEN - 1u)) { n_out = (DBG_CACHE_LEN - 1u); }

        memcpy(out, buf, n_new);
        for (i = n_new; i < n_out; i ++) { out[i] = ' '; }
        out[n_out] = '\0';

        ui_puts_row((uint8)(DBG_ROW_VAL0 + r), DBG_COL_VAL, out);

        memcpy(s_vcache[r], buf, (n_new < DBG_CACHE_LEN) ? (n_new + 1u) : DBG_CACHE_LEN);
        s_vcache[r][DBG_CACHE_LEN - 1u] = '\0';
    }
}

/* ---- 视图选择列表（第一级）---- */
static void dbg_draw_choose (void)
{
    uint8 i;
    char  line[40];

    ui_clear();

    ui_puts_row(0u, 0u, "DEBUG  select view");

    for (i = 0u; i < DBG_VIEWS; i ++)
    {
        snprintf(line, sizeof(line), "%s%s", (i == s_sel) ? "> " : "  ", S_TITLE[S_ORDER[i]]);
        ui_puts_row((uint8)(DBG_ROW_VAL0 + i), 0u, line);
    }

    ui_puts_row(DBG_ROW_HINT, 0u, "K1/K2=Pick K3=OK K4=Bk");
}

void ui_debug_enter (void)
{
    s_page    = DBG_PAGE_LIST;
    s_sel     = 0u;                     /* 光标停在第一项（WIFI）*/
    s_view    = S_ORDER[0];
    s_wifi_sel = 0u;
    s_wifi_st  = (0u == motor_link_init_err()) ? 2u : 0u;   /* 已连过就显示 ok */
    s_last    = motor_link_now_ms();

    dbg_draw_choose();
}

void ui_debug_move (int8 dir)
{
    uint8 old;

    if (0 == dir) { return; }

    if (DBG_PAGE_LIST == s_page)
    {
        old   = s_sel;
        /* dir>0 = 上移键 ⇒ 光标往**上一个**条目走（同主菜单的约定）*/
        s_sel = (uint8)((s_sel + DBG_VIEWS + ((dir > 0) ? -1 : 1)) % DBG_VIEWS);

        /* 只挪光标，不整屏重画 */
        {
            char line[40];

            snprintf(line, sizeof(line), "  %s", S_TITLE[S_ORDER[old]]);
            ui_puts_row((uint8)(DBG_ROW_VAL0 + old), 0u, line);

            snprintf(line, sizeof(line), "> %s", S_TITLE[S_ORDER[s_sel]]);
            ui_puts_row((uint8)(DBG_ROW_VAL0 + s_sel), 0u, line);
        }
        return;
    }

    /* 详情页里只有 WIFI 视图吃上下键：移动候选光标 */
    if (DBG_VIEW_WIFI == s_view)
    {
        old        = s_wifi_sel;
        s_wifi_sel = (uint8)((s_wifi_sel + (uint8)MOTOR_AP_COUNT +
                              ((dir > 0) ? -1 : 1)) % (uint8)MOTOR_AP_COUNT);
        dbg_wifi_cursor(old);
    }
}

void ui_debug_action (void)
{
    if (DBG_PAGE_LIST == s_page)
    {
        /* 确认 = 进入选中的视图 */
        s_view = S_ORDER[s_sel];
        s_page = DBG_PAGE_DETAIL;
        s_last = motor_link_now_ms();

        /* ★ 三个"要动机械"的页面进来先关环，退出时由 ui_debug_back() 恢复：
             MOTOR   —— 要手推车/开环测试，后轮不能自己出力
             SERVO   —— 要手推方向盘核对编码器读数，舵机不能自己出力
             WIFI    —— 配置链路期间车必须完全不动
           IMU / UART-RC / SYSTEM 是纯观察量，不关环。 */
        if ((0u == s_view) || (DBG_VIEW_SERVO == s_view) || (DBG_VIEW_WIFI == s_view))
        {
            dbg_loops_off();
        }

        if (DBG_VIEW_WIFI == s_view)
        {
            s_wifi_st = (0u == motor_link_init_err()) ? 2u : 0u;
            dbg_wifi_draw();
        }
        else
        {
            dbg_draw_static();
            dbg_draw_values();
        }
        return;
    }

    if (DBG_VIEW_WIFI == s_view)
    {
        /* 连接选中候选：阻塞（WiFi 最长约 10 s，Socket 更久），屏上先给 CONNECT */
        s_wifi_st = 1u;
        dbg_wifi_st();

        s_wifi_st = (0u == motor_link_connect(s_wifi_sel)) ? 2u : 3u;
        dbg_wifi_st();
        return;
    }

    /* 其它详情页里只有 IMU 视图有动作：把当前点设成 yaw = 0。
       ⚠ ins_odom_set_origin() 会同时把 x/y 设成传入值并清左右编码器累计；
         这里传当前位置，所以只有 yaw 归零、位置连续。 */
    if (1u == s_view)
    {
        ins_odom_set_origin(ins_odom_get_x(), ins_odom_get_y(), 0.0f);
        memset(s_vcache, 0, sizeof(s_vcache));      /* 强制下一帧全部重画 */
        dbg_draw_values();
    }
}

uint8 ui_debug_back (void)
{
    if (DBG_PAGE_DETAIL == s_page)
    {
        /* 详情 → 回列表：先把进页时关掉的环还原，再整屏重画（两级页布局不同）*/
        dbg_loops_restore();
        s_page = DBG_PAGE_LIST;
        dbg_draw_choose();
        return 1u;
    }

    return 0u;      /* 已在列表：让调用方回主菜单 */
}

void ui_debug_tick (void)
{
    uint32 now = motor_link_now_ms();

    if (DBG_PAGE_DETAIL != s_page) { return; }      /* 列表页没有实时量 */
    if ((uint32)(now - s_last) < (uint32)DBG_REFRESH_MS) { return; }
    s_last = now;

    if (DBG_VIEW_WIFI == s_view) { dbg_wifi_st(); return; }

    dbg_draw_values();
}
