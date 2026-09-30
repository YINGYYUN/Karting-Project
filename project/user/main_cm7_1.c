/*********************************************************************************************************************
* CYT4BB Opensource Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_1
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "IMU_Analysis.h"


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化

    // 外设初始化
    // 当DEFINE_IMU_ANALYSIS_MODE == 0时，IMU初始化和解算函数将不调用实际内容
    // 0 - 初始化成功; 1 - 初始化失败; 4 - imu被禁用 
    uint8 imu_ret = IMU_init();    // IMU963RA 初始化（硬件 SPI2, P15.x）
    printf("[CM7_1] imu_init ret=%d\n", imu_ret);
    pit_ms_init(PIT_CH2, 10);           // 10ms 定时，IMU 解算（pit0_ch2_isr）

    while(true)
    {
        // 处理 CM7_0 通过共享内存发来的命令（先失效 cache 读最新值）
        SHARED_IMU_READ_SYNC();
        if(SHARED_IMU_ADDR->cmd_gyro_calib == 1)
        {
            IMU_Gyro_Calib_Start(&gyro_cal);    // 零漂校准开始（车需静止）
            // 指示（陀螺仪零漂）校准开始
            SHARED_IMU_ADDR->cmd_gyro_calib = 2;
            SHARED_IMU_WRITE_SYNC();
        }

        if(IMU_D_and_A_Enable && IMU_Gyro_Calib_Check(&gyro_cal) == GYRO_CALIB_STATE_RUNNING)
        {
            // 如果校准正在进行，将消耗IMU_D_and_A_Enable标志位，并阻止解算
            // 下面这行置零只是指示这个标志位在这里被置零了
            IMU_D_and_A_Enable = 0;
        }
        else if (IMU_Gyro_Calib_Check(&gyro_cal) == GYRO_CALIB_STATE_DONE && SHARED_IMU_ADDR->cmd_gyro_calib != 0)
        {
            // 指示（陀螺仪零漂）校准完成
            SHARED_IMU_ADDR->cmd_gyro_calib = 3;
            SHARED_IMU_WRITE_SYNC();
        }

        if(SHARED_IMU_ADDR->cmd_reset == 1)
        {
            IMU_Reset_Data();                   // Yaw 归零
            SHARED_IMU_ADDR->cmd_reset = 0;
        }

        // IMU 解算（10ms）
        if(IMU_D_and_A_Enable)
        {
            IMU_D_and_A_Enable = 0;
            IMU_Update_Analysis();              // 解算 Yaw_Result
            SHARED_IMU_ADDR->yaw = (int16)(Yaw_Result * 100.0f);  // 定点：0.01°/LSB
            SHARED_IMU_WRITE_SYNC();            // 写回 cache，让 CM7_0 可见
        }
    }
}

