/*******************************************************************************
参数 Flash 存储 — 实现（CYT4BB 适配）
*******************************************************************************/


#include "Param_Storage.h"


// 注意：Motor_LR_Ctrl / Motor_RR_Ctrl 已在 Motor_Ctrl.h 中声明

// 默认参数值
static const float DEFAULT_PARAMS[PARAM_COUNT] = {
    // 左后轮：inv_K, u0, k_v, k_z（口径 v=rps、u=归一化占空比）
    0.1548f, 0.057f, 0.164f, 0.512f,
    // 右后轮：inv_K, u0, k_v, k_z
    0.1548f, 0.057f, 0.164f, 0.512f,
    // 舵机位置环：kp, u0, ki
    400.0f, 1700.0f, 25.0f,
    // 路表：录制采样点距 m
    INS_ROUTE_DS_DEFAULT,
    // 回放：车速 m/s、前视距离 m、转向通道 0=航向 / 1=直给 δ
    PB_V_DEFAULT, PB_LD_DEFAULT, PB_SMODE_DEFAULT,
    // 遥测档案选择：0 = 全量 41 路（上电默认） / 1 = 标定精简 18 路
    TEL_SEL_DEFAULT,
    // 录制终点补点：1 = 停止时强制补最后一个点（默认开）
    ROUTE_END_PT_DEFAULT,
    // 有效轴距 m（回放的 2L/ld 与 δ=atan(L·κ) 都用它）
    PB_L_DEFAULT,
    // 速度剖面：使能（0 = 关，行为与以前完全一样）/ 允许横向加速度 / 限速地板
    PB_VE_DEFAULT, PB_ALAT_DEFAULT, PB_VMIN_DEFAULT,
    // 航向增益手工覆盖：0 = 自动按 2L/ld
    PB_KH_DEFAULT,
    // 倒车录表 / 倒序回放（默认都关：普通表行为完全不受影响）
    0.0f, 0.0f,
};

// 参数缓存区(菜单直接修改此数组, Flash 读写也通过此数组)
float param_cache[PARAM_COUNT];


/**********************************************************/
/*[S] 内部函数 [S]----------------------------------------*/
/**********************************************************/

static void load_default(void)
{
    for (uint8_t i = 0; i < PARAM_COUNT; i++)
        param_cache[i] = DEFAULT_PARAMS[i];
}

static void copy_cache_to_flash_buffer(void)
{
    for (uint8_t i = 0; i < PARAM_COUNT; i++)
        flash_union_buffer[i].float_type = param_cache[i];
}

static void copy_flash_buffer_to_cache(void)
{
    for (uint8_t i = 0; i < PARAM_COUNT; i++)
        param_cache[i] = flash_union_buffer[i].float_type;
}

// 校验参数是否合法(异常值无效)
// 采样式校验：只拿左右后轮的 inv_K 当采样目标。
static uint8_t param_cache_is_valid(void)
{
    // 用"取反的区间判断"而不是 > || <：NaN 参与 > 和 < 比较都为假，
    // 这样 NaN / Inf / 全 0xFF 的空白页都会被判为非法（旧写法会漏过去，之后算出 NaN 输出）
    // 取 ±5 是宽松兜底
    if (!((MOTOR_LR_INV_K > -5.0f) && (MOTOR_LR_INV_K < 5.0f)))
    {
        return 0;
    }

    if (!((MOTOR_RR_INV_K > -5.0f) && (MOTOR_RR_INV_K < 5.0f)))
    {
        return 0;
    }

    return 1;
}

//--------------------------------------
// 函数简介     末尾新增参数的缺省修复
// 备注信息     旧参数页没有这几项，读出来是空白值；只补它们，不动前面的参数
//--------------------------------------
static void fix_appended_params (void)
{
    uint8 fix = 0;

    if (!((ROUTE_DS >= INS_ROUTE_DS_MIN) && (ROUTE_DS <= INS_ROUTE_DS_MAX)))
    {
        ROUTE_DS = INS_ROUTE_DS_DEFAULT;
        fix = 1;
    }

    if (!((ROUTE_PB_V >= PB_V_MIN) && (ROUTE_PB_V <= PB_V_MAX)))
    {
        ROUTE_PB_V = PB_V_DEFAULT;
        fix = 1;
    }

    if (!((ROUTE_PB_LD >= PB_LD_MIN) && (ROUTE_PB_LD <= PB_LD_MAX)))
    {
        ROUTE_PB_LD = PB_LD_DEFAULT;
        fix = 1;
    }

    if (!((ROUTE_PB_SMODE >= 0.0f) && (ROUTE_PB_SMODE <= 1.0f)))
    {
        ROUTE_PB_SMODE = PB_SMODE_DEFAULT;
        fix = 1;
    }

    // 遥测档案选择：旧参数页的 index 15 是空白区（0xFF），读出来是 NaN/巨值
    // ★ 取反写法（!区间）能连 NaN 一起拦住 —— 与 param_cache_is_valid 同源
    if (!((TEL_SEL >= 0.0f) && (TEL_SEL <= 1.0f)))
    {
        TEL_SEL = TEL_SEL_DEFAULT;
        fix = 1;
    }

    // 录制终点补点：0/1
    if (!((ROUTE_END_PT >= 0.0f) && (ROUTE_END_PT <= 1.0f)))
    {
        ROUTE_END_PT = ROUTE_END_PT_DEFAULT;
        fix = 1;
    }

    // 有效轴距：空白页读出来是 NaN/巨值；量程用 route_pb.h 的那一对
    if (!((ROUTE_PB_L >= PB_L_MIN) && (ROUTE_PB_L <= PB_L_MAX)))
    {
        ROUTE_PB_L = PB_L_DEFAULT;
        fix = 1;
    }

    // 速度剖面使能：0/1
    if (!((ROUTE_PB_VE >= 0.0f) && (ROUTE_PB_VE <= 1.0f)))
    {
        ROUTE_PB_VE = PB_VE_DEFAULT;
        fix = 1;
    }

    // 横向加速度：太小限速会把车压成爬行，太大等于没有限速
    if (!((ROUTE_PB_ALAT >= PB_ALAT_MIN) && (ROUTE_PB_ALAT <= PB_ALAT_MAX)))
    {
        ROUTE_PB_ALAT = PB_ALAT_DEFAULT;
        fix = 1;
    }

    // 限速地板：0 也是合法的（= 弯里可以压到 0），所以只夹上限
    if (!((ROUTE_PB_VMIN >= PB_VMIN_MIN) && (ROUTE_PB_VMIN <= PB_VMIN_MAX)))
    {
        ROUTE_PB_VMIN = PB_VMIN_DEFAULT;
        fix = 1;
    }

    // 航向增益手工覆盖：0 = 自动，合法范围 0~PB_KH_MAX
    if (!((ROUTE_PB_KH >= PB_KH_MIN) && (ROUTE_PB_KH <= PB_KH_MAX)))
    {
        ROUTE_PB_KH = PB_KH_DEFAULT;
        fix = 1;
    }

    // 倒车录 / 倒序回放：只接受 0 或 1（存成浮点，其它值一律归 0）
    if (!((ROUTE_REV_REC >= -0.5f) && (ROUTE_REV_REC <= 1.5f)))
    {
        ROUTE_REV_REC = 0.0f;
        fix = 1;
    }
    if (!((ROUTE_REV_PLAY >= -0.5f) && (ROUTE_REV_PLAY <= 1.5f)))
    {
        ROUTE_REV_PLAY = 0.0f;
        fix = 1;
    }

    if (0 != fix)
    {
        copy_cache_to_flash_buffer();
        flash_write_page_from_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, PARAM_COUNT);
    }
}/**********************************************************/
/*----------------------------------------[E] 内部函数 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 外部函数 [S]----------------------------------------*/
/**********************************************************/

// 初始化参数系统(上电调用一次)
void Param_Init(void)
{
    flash_init();       // CYT4BB Flash 使用前必须先初始化

#if PARAM_FORCE_RESET
    // 一次性：擦掉 Flash 里的旧参数，让下面的新默认值生效（由 PARAM_FORCE_RESET 控制）
    flash_erase_page(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE);
#endif

    if (flash_check(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE))
    {
        // Flash 有数据 → 读取到缓冲区
        flash_read_page_to_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, PARAM_COUNT);
        copy_flash_buffer_to_cache();

        // 校验失败 → 回退默认值并修复 Flash
        if (!param_cache_is_valid())
        {
            load_default();
            copy_cache_to_flash_buffer();
            flash_write_page_from_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, PARAM_COUNT);
        }
        else
        {
            fix_appended_params();
        }
    }
    else
    {
        // 首次使用 → 写入默认值
        load_default();
        copy_cache_to_flash_buffer();
        flash_write_page_from_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, PARAM_COUNT);
    }

    // 将参数推送到实际应用参数
    Flash_SyncTo_Param();
}

// 缓存区是否与 Flash 里的不一致（= 有未保存的改动）
//
// 判据就是"把 Flash 里那一页读出来，跟 cache 按位比" —— 不额外维护任何状态，
// 所以不管是菜单步进改的还是上位机 SET_PARAM 改的，都自动算进来。
// ⚠ 按位比，不用浮点 != ：NaN != NaN 恒真，一个 NaN 就会"永远有改动"、每次白擦一页。
uint8 Param_Is_Dirty(void)
{
    flash_read_page_to_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, PARAM_COUNT);
    return (0 != memcmp(flash_union_buffer, param_cache, sizeof(float) * PARAM_COUNT)) ? 1u : 0u;
}

// 保存参数到 Flash(退出参数页面时调用)
//
// ★ 2026-10-23：**没改动就一个字节都不写**。
//   原来是无条件写，而参数页"根层按 BACK 退出"就是无条件调它 —— 进去看一眼、
//   什么都没动，也会擦掉一个 2 KB 扇区（Work Flash 的擦除粒度就是 2 KB）。
void Param_Save(void)
{
    if (0 == Param_Is_Dirty()) { return; }

    copy_cache_to_flash_buffer();
    flash_write_page_from_buffer(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, PARAM_COUNT);
}

// 擦除 Flash(下次启动恢复默认)
void Param_Erase(void)
{
    flash_erase_page(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE);
}

// 将缓存区值同步到实际应用参数
void Flash_SyncTo_Param(void)
{
    Motor_LR_Ctrl.inv_K = MOTOR_LR_INV_K;
    Motor_LR_Ctrl.u0    = MOTOR_LR_U0;
    Motor_LR_Ctrl.k_v   = MOTOR_LR_KV;
    Motor_LR_Ctrl.k_z   = MOTOR_LR_KZ;

    Motor_RR_Ctrl.inv_K = MOTOR_RR_INV_K;
    Motor_RR_Ctrl.u0    = MOTOR_RR_U0;
    Motor_RR_Ctrl.k_v   = MOTOR_RR_KV;
    Motor_RR_Ctrl.k_z   = MOTOR_RR_KZ;

    Servo_Ctrl_Kp = SERVO_KP;
    Servo_Ctrl_U0 = SERVO_U0;
    Servo_Ctrl_Ki = SERVO_KI;

    ins_route_ds = ROUTE_DS;
}
/**********************************************************/
/*----------------------------------------[E] 外部函数 [E]*/
/**********************************************************/
