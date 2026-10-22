/*********************************************************************************************************************
 * 航位推算实现。坐标约定与比例尺说明见 ins_odom.h
 *
 * ins_ = INS（Inertial Navigation System，惯导）子系统前缀，本文件属"惯导 + 路表"系列
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include "ins_odom.h"
#include "ins_imu.h"
#include "Motor.h"

#define DEG2RAD_F   (0.017453292519943295f)

/* 临界区：直接存取 PRIMASK。
   ⚠ 不要用逐飞的 interrupt_global_disable()/interrupt_global_enable(pm) —— 那是自增计数器，
     pm 非 0 时 enable 不会真的开中断，会把中断永久关死且零报错。 */
static uint32 ins_crit_enter (void) { uint32 pm = __get_PRIMASK(); __disable_irq(); return pm; }
static void   ins_crit_exit  (uint32 pm) { __set_PRIMASK(pm); }

volatile float ins_m_per_rev   = INS_ODOM_MPM_DEFAULT;
volatile float ins_odom_enable = 1.0f;
volatile float ins_slip_pct    = 30.0f;

static volatile float s_x        = 0.0f;
static volatile float s_y        = 0.0f;
static volatile float s_yaw      = 0.0f;
static volatile float s_dist_abs = 0.0f;
static volatile float s_v_mps    = 0.0f;
static volatile uint8 s_flags    = 0;

static uint16 s_still_cnt = 0;

//--------------------------------------
// 函数简介     初始化（位姿清零）
//--------------------------------------
void ins_odom_init (void)
{
    s_x         = 0.0f;
    s_y         = 0.0f;
    s_yaw       = ins_imu_get_yaw();
    s_dist_abs  = 0.0f;
    s_v_mps     = 0.0f;
    s_flags     = ODOM_ST_ACTIVE;
    s_still_cnt = 0;
}

//--------------------------------------
// 函数简介     位置与里程清零（航向不动）
// 备注信息     编码器用单拍增量积分，没有累计基准要清；窗口里残留的几拍会在下一拍补进来（<1 cm）
//--------------------------------------
void ins_odom_reset (void)
{
    uint32 pm = ins_crit_enter();

    s_x         = 0.0f;
    s_y         = 0.0f;
    s_dist_abs  = 0.0f;
    s_v_mps     = 0.0f;
    s_flags     = ODOM_ST_ACTIVE;
    s_still_cnt = 0;

    ins_crit_exit(pm);
}

//--------------------------------------
// 函数简介     把当前点设成指定位姿
//--------------------------------------
void ins_odom_set_origin (float x, float y, float yaw_deg)
{
    uint32 pm = ins_crit_enter();

    s_x   = x;
    s_y   = y;
    s_yaw = yaw_deg;

    ins_crit_exit(pm);
}

//--------------------------------------
// 函数简介     5ms 每拍：吃本拍编码器增量 + yaw → (x, y)
// 备注信息     必须在 ENC_LR/RR_Update 与 ins_imu_update 之后调用
//              用 ENC_*_DLT（单拍原始增量），不用滑动平均后的 _CNT_F
//--------------------------------------
void ins_odom_update (void)
{
    float yaw_deg = ins_imu_get_yaw();
    int16 dl      = ENC_LR_DLT;                 // 本拍原始增量（已含方向与极性，未经滑动平均）
    int16 dr      = ENC_RR_DLT;
    float dl_m    = ((float)dl / MOTOR_CRTL_CPR) * ins_m_per_rev;
    float dr_m    = ((float)dr / MOTOR_CRTL_CPR) * ins_m_per_rev;
    float ds      = (dl_m + dr_m) * 0.5f;       // 取两后轮中部
    uint8 flags   = 0;

    // 静止判据：只打状态位，不冻结积分（编码器里程不像纯惯导那样漂）
    if ((0 == dl) && (0 == dr))
    {
        if (s_still_cnt < INS_ODOM_STILL_TICKS) { s_still_cnt ++; }
    }
    else
    {
        s_still_cnt = 0;
    }
    if (s_still_cnt >= INS_ODOM_STILL_TICKS) { flags |= ODOM_ST_STILL; }

    // 打滑判据：左右计数差很大、航向却几乎没变 ⇒ 大概率一侧打滑。只报警
    {
        int32 al = (dl < 0) ? -(int32)dl : (int32)dl;
        int32 ar = (dr < 0) ? -(int32)dr : (int32)dr;
        int32 mx = (al > ar) ? al : ar;

        if (mx > 5)                             // 计数太小不算，避免噪声
        {
            float diff_pct = (float)((al > ar) ? (al - ar) : (ar - al)) * 100.0f / (float)mx;
            float dyaw     = fabsf(yaw_deg - s_yaw);

            if (dyaw > 180.0f) { dyaw = 360.0f - dyaw; }

            if ((diff_pct > ins_slip_pct) && (dyaw < 1.0f))
            {
                flags |= ODOM_ST_SLIP;
            }
        }
    }

    // 积分：ds 带符号（倒车为负），总路程另算
    if (ins_odom_enable > 0.5f)
    {
        float rad = yaw_deg * DEG2RAD_F;

        s_x += -sinf(rad) * ds;
        s_y +=  cosf(rad) * ds;
        s_dist_abs += fabsf(ds);
    }

    s_yaw   = yaw_deg;
    s_v_mps = ds / INS_ODOM_T_S;
    s_flags = (uint8)(flags | ODOM_ST_ACTIVE);
}

//--------------------------------------
// 函数简介     查询
//--------------------------------------
float ins_odom_get_x     (void) { return s_x; }
float ins_odom_get_y     (void) { return s_y; }
float ins_odom_get_yaw   (void) { return s_yaw; }
float ins_odom_get_dist  (void) { return s_dist_abs; }
float ins_odom_get_v_mps (void) { return s_v_mps; }
uint8 ins_odom_flags     (void) { return s_flags; }

//--------------------------------------
// 函数简介     原子取位姿快照
//--------------------------------------
void ins_odom_snapshot (ins_odom_snap_t *out)
{
    uint32 pm;

    if (NULL == out) { return; }

    pm = ins_crit_enter();
    out->x        = s_x;
    out->y        = s_y;
    out->yaw      = s_yaw;
    out->dist_abs = s_dist_abs;
    out->v_mps    = s_v_mps;
    out->flags    = s_flags;
    ins_crit_exit(pm);
}
