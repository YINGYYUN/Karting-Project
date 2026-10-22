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
* 文件名称          main_cm7_0
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
#include "ins_imu.h"

// WiFi 是否已初始化（未初始化时，依赖 WiFi 的功能不启用）
volatile uint8 wifi_spi_inited = 0;


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	    // 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化

    Peripheral_Init();				    // 初始化外设，自行配置

    /* ---- IMU 姿态解算（IMU963RA / SPI_2，本核独占）----
       ★★ 这一步**会阻塞 5 秒**（1000 个 5ms 采样求零偏），失败最多重试 5 次 = 25 秒。
          所以必须放在"屏幕已经能显示"之后，让现场知道是在标定、不是死机。
          标定期间要求车辆静止：单轴极差超过 INS_CALIB_MAX_SPAN_LSB 会被判"被碰过"并重来。
       ★ 失败不影响开车：ins_imu_update() 会走"链路坏"分支，一步都不碰 SPI_2，
         只是 yaw 恒为 0（转向环也用不到它）。*/
    ips200_show_string(8, 120, "IMU GYRO CALIB...");
    ips200_show_string(8, 136, "PLEASE KEEP STILL");
    {
        uint8 imu_ret = ins_imu_init();         // 0 = 成功 / 1 = 器件初始化失败 / 2 = 零偏超限
        if (0 == imu_ret) { ips200_show_string(8, 152, "IMU OK              "); }
        else              { ips200_show_string(8, 152, "IMU FAIL, SEE UART  "); }
    }

    // 位姿基准：把起点设成 (0,0)，航向取当前 IMU 值（IMU 失败时就是 0）
    ins_odom_init();

    pit_ms_init(PIT_CH0, 10);
    pit_ms_init(PIT_CH1, 5);            // 5ms：转向（角度编码器采集 + 舵机位置环，同中断）
    pit_ms_init(PIT_CH11, 5);           // 5ms：后轮速度环 + IMU 姿态解算（见 pit0_ch11_isr）
    pit_ms_init(PIT_CH10, 1);           // 1ms 定时，全局毫秒计时（编码器速度积分用）

    while(true)
    {
        Menu_Show();
    }
}
