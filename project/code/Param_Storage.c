/*******************************************************************************
参数 Flash 存储 — 实现（CYT4BB 适配）
*******************************************************************************/


#include "Param_Storage.h"


// 4 个电机的增量式 PID（定义于 PID.c）
extern PID_INC_t Motor_1_PID;           // 电机接口 1
extern PID_INC_t Motor_2_PID;           // 电机接口 2
extern PID_INC_t Motor_3_PID;           // 电机接口 3
extern PID_INC_t Motor_4_PID;           // 电机接口 4


// 默认参数值(首次使用或恢复出厂设置时使用)
static const float DEFAULT_PARAMS[PARAM_COUNT] = {
    // Motor_1_PID
    4.0f, 0.6f, 1.0f,       // KP, KI, KD
    // Motor_2_PID
    4.0f, 0.6f, 1.0f,       // KP, KI, KD
    // Motor_3_PID
    4.0f, 0.6f, 1.0f,       // KP, KI, KD
    // Motor_4_PID
    4.0f, 0.6f, 1.0f,       // KP, KI, KD
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
    uint8_t kp_nonzero = 0;

    for (uint8_t i = 0; i < PARAM_COUNT; i++)
    {
        // 范围检查
        if (param_cache[i] > 10000.0f || param_cache[i] < -10000.0f)
            return 0;

        // 任意一个 Kp 非零即认为有效
        // 只是采样检验有效性，后续参数不必要参与该判定
        if (i == MOTOR_1_KP_IDX || i == MOTOR_2_KP_IDX ||
            i == MOTOR_3_KP_IDX || i == MOTOR_4_KP_IDX)
        {
            if (param_cache[i] > 0.0001f || param_cache[i] < -0.0001f)
                kp_nonzero = 1;
        }
    }

    return kp_nonzero;
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
    Motor_1_PID.Kp = MOTOR_1_KP;
    Motor_1_PID.Ki = MOTOR_1_KI;
    Motor_1_PID.Kd = MOTOR_1_KD;

    Motor_2_PID.Kp = MOTOR_2_KP;
    Motor_2_PID.Ki = MOTOR_2_KI;
    Motor_2_PID.Kd = MOTOR_2_KD;

    Motor_3_PID.Kp = MOTOR_3_KP;
    Motor_3_PID.Ki = MOTOR_3_KI;
    Motor_3_PID.Kd = MOTOR_3_KD;

    Motor_4_PID.Kp = MOTOR_4_KP;
    Motor_4_PID.Ki = MOTOR_4_KI;
    Motor_4_PID.Kd = MOTOR_4_KD;
}
/**********************************************************/
/*----------------------------------------[E] 外部函数 [E]*/
/**********************************************************/
