# Karting-Project 项目记忆

## 技术栈
- 芯片：Infineon TRAVEO T2G **CYT4BB7**（双核 ARM Cortex-M7 @ 250MHz）
- 厂商库：**逐飞科技 (SeekFree) CYT4BB Opensource Library**，GPL3.0，基于官方 SDK（libraries/sdk/tviibh4m）
- IDE：**IAR Embedded Workbench 9.40.1**（工程在 project/iar/project_config，含 Debug_m7_0 / Debug_m7_1 两个工程）
- 语言：**纯 C**（无 .cpp），头文件 IMU_Analysis.h 注释误写 .cpp，实际是 C

## 目录约定
- libraries/zf_common（公共层）、zf_driver（外设驱动）、zf_device（外接设备驱动：IMU/屏幕/无线/GPS/摄像头等）
- project/code：用户自建代码（Menu/Motor/PID/IMU_Analysis/Param_Storage/Debug），新增代码放此目录、勿建子文件夹
- project/user：main_cm7_0.c / main_cm7_1.c + 两个核的 isr 文件

## 架构要点
- 双核分工：CM7_0 跑菜单+电机速度环（编码器/PIT_CH1）；CM7_1 跑 IMU963RA 解算（PIT_CH2）
- 双核通信：共享内存结构体 `shared_imu_t` 在 0x280B8000，宏 SHARED_IMU_ADDR；cache 同步用 SCB_CleanDCache_by_Addr / SCB_InvalidateDCache_by_Addr（写前 clean、读前 invalidate）
- 当前状态：仅骨架模板，Peripheral_Init 只初始化 IPS200/key/motor/param，缺少实际行车控制闭环

## IMU（IMU963RA）硬件与开关设计
- 硬件 SPI2，引脚全在 P15 口：SPI2_CLK=P15_2、SPI2_MOSI=P15_1、SPI2_MISO=P15_0、CS=P15_3（逐飞 CYT4BB7 学习板 IMU 插座）
- `imu963ra_init()` 自检 WHO_AM_I(0x0F) 期望 0x6B，失败返回 1；当前 main_cm7_1.c 把返回值存 imu_ret 但**未使用**
- 引脚被另一模块占用时（IMU 取下），绝对不能再调用 imu963ra_init（spi_init 会重配 P15 引脚、干扰另一模块）→ IMU 开关必须**编译期宏**为主，运行期 WHO_AM_I 自检为辅
- IMU 任务由 cm7_1 的 PIT_CH2 ISR 置 IMU_D_and_A_Enable=1、main_cm7_1 主循环消费；禁用 IMU 即不初始化 PIT_CH2

## 我的能力边界（代码编写）
- 可做：写/改 project/code 应用层、按 zf_driver API 加外设、扩双核 IPC、加 ISR、移植 zf_device 已支持的设备
- 不能做：本机无法用 IAR 编译/烧录（需付费 IDE + 实体板+调试器），只能写代码+逻辑核对，不能跑 build/硬件测试
