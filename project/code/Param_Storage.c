/*******************************************************************************
参数 Flash 存储 — 实现（CYT4BB 适配）
*******************************************************************************/


#include "Param_Storage.h"


// 注意：Motor_LR_Crtl / Motor_RR_Crtl 已在 Motor_Crtl.h 中声明

// 默认参数值
static const float DEFAULT_PARAMS[PARAM_COUNT] = {
    // 左后轮：inv_K, u0, k_v, k_z
    //   inv_K=83.3, u0=330, k_v=88.3, k_z=1249
    83.3f, 330.0f, 88.3f, 1249.0f,
    // 右后轮：inv_K, u0, k_v, k_z
    //   inv_K=83.3, u0=330, k_v=88.3, k_z=1249
    83.3f, 330.0f, 88.3f, 1249.0f,
    // 舵机位置环：kp, u0
    300.0f, 1600.0f,
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
static uint8_t param_cache_is_valid(void)
{
    for (uint8_t i = 0; i < PARAM_COUNT; i++)
    {
        // 范围检查（Flash 空白/损坏时读出的浮点会远超此范围）
        // 用"取反的区间判断"而不是 > || <，这样 NaN / Inf 也会被判为非法
        // （NaN 参与 > 和 < 比较都为假，用旧写法会漏过去，之后参与运算会算出 NaN 输出）
        if (!((param_cache[i] > -10000.0f) && (param_cache[i] < 10000.0f)))
            return 0;
        // 采样性质，后续的其他参数不必都参与检查
    }
    return 1;
}
/**********************************************************/
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

// 保存参数到 Flash(退出参数页面时调用)
void Param_Save(void)
{
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
    Motor_LR_Crtl.inv_K = MOTOR_LR_INV_K;
    Motor_LR_Crtl.u0    = MOTOR_LR_U0;
    Motor_LR_Crtl.k_v   = MOTOR_LR_KV;
    Motor_LR_Crtl.k_z   = MOTOR_LR_KZ;

    Motor_RR_Crtl.inv_K = MOTOR_RR_INV_K;
    Motor_RR_Crtl.u0    = MOTOR_RR_U0;
    Motor_RR_Crtl.k_v   = MOTOR_RR_KV;
    Motor_RR_Crtl.k_z   = MOTOR_RR_KZ;

    Servo_Crtl_Kp = SERVO_KP;
    Servo_Crtl_U0 = SERVO_U0;
}
/**********************************************************/
/*----------------------------------------[E] 外部函数 [E]*/
/**********************************************************/
