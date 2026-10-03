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
* 文件名称          zf_device_absolute_encoder
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version (TC264)
* 2023-04-25       pudding            补充部分注释说明 (TC264)
* 2026-09-30       --                 CYT4BB 平台移植（依据 TC264 开源库同名驱动）
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   SCLK               查看 zf_device_absolute_encoder.h 中 ABSOLUTE_ENCODER_SCLK_PIN 宏定义
*                   MOSI               查看 zf_device_absolute_encoder.h 中 ABSOLUTE_ENCODER_MOSI_PIN 宏定义
*                   MISO               查看 zf_device_absolute_encoder.h 中 ABSOLUTE_ENCODER_MISO_PIN 宏定义
*                   CS                 查看 zf_device_absolute_encoder.h 中 ABSOLUTE_ENCODER_CS_PIN   宏定义
*                   VCC                3.3V电源
*                   GND                电源地
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_absolute_encoder_h_
#define _zf_device_absolute_encoder_h_

#include "zf_common_typedef.h"

//=================================================定义 角度传感器 参数配置================================================
#define ABSOLUTE_ENCODER_USE_SOFT_SPI       (0)                                 // 默认使用硬件 SPI 方式驱动，改为 1 则使用软件 SPI
#if ABSOLUTE_ENCODER_USE_SOFT_SPI
//====================================================软件 SPI 驱动====================================================
#define ABSOLUTE_ENCODER_SOFT_SPI_DELAY     (1)                                 // 软件 SPI 延时参数 数值越大 SPI 通信速率越慢
#define ABSOLUTE_ENCODER_SCLK_PIN           (P07_2)                             // 软件 SPI SCK  引脚
#define ABSOLUTE_ENCODER_MOSI_PIN           (P07_1)                             // 软件 SPI MOSI 引脚
#define ABSOLUTE_ENCODER_MISO_PIN           (P07_0)                             // 软件 SPI MISO 引脚
//====================================================软件 SPI 驱动====================================================
#else
//====================================================硬件 SPI 驱动====================================================
#define ABSOLUTE_ENCODER_SPI_SPEED          (8 * 1000 * 1000)                   // 硬件 SPI 速率 通信异常时可适当降低
#define ABSOLUTE_ENCODER_SPI                (SPI_4           )                  // 硬件 SPI 号 (SCB5)
#define ABSOLUTE_ENCODER_SCLK_PIN           (SPI4_CLK_P07_2  )                  // 硬件 SPI SCK  引脚  P7.2
#define ABSOLUTE_ENCODER_MOSI_PIN           (SPI4_MOSI_P07_1 )                  // 硬件 SPI MOSI 引脚  P7.1
#define ABSOLUTE_ENCODER_MISO_PIN           (SPI4_MISO_P07_0 )                  // 硬件 SPI MISO 引脚  P7.0
//====================================================硬件 SPI 驱动====================================================
#endif

#define ABSOLUTE_ENCODER_CS_PIN             (P07_3)                             // CS 引脚定义  P7.3
#define ABSOLUTE_ENCODER_CSN(x)             ((x) ? (gpio_high(ABSOLUTE_ENCODER_CS_PIN)) : (gpio_low(ABSOLUTE_ENCODER_CS_PIN)))

#define ABSOLUTE_ENCODER_TIMEOUT_COUNT      (100)                               // 自检超时时间
#define ABSOLUTE_ENCODER_DEFAULT_ZERO       (0)                                 // 默认零点位置 取值范围 0 ~ 4095

#define ABSOLUTE_ENCODER_SPI_W              (0x80)                              // 写标志
#define ABSOLUTE_ENCODER_SPI_R              (0x40)                              // 读标志

#define ABSOLUTE_ENCODER_ZERO_L_REG         (0x00)                              // 零点位置 低字节寄存器
#define ABSOLUTE_ENCODER_ZERO_H_REG         (0x01)                              // 零点位置 高字节寄存器
#define ABSOLUTE_ENCODER_DIR_REG            (0x09)                              // 旋转方向寄存器
//=================================================定义 角度传感器 参数配置================================================


//=================================================声明 角度传感器 外接驱动================================================
int16   absolute_encoder_get_location       (void);                             // 返回值：返回读取的当前位置 12 位分辨率 0 ~ 4095
int16   absolute_encoder_get_offset         (void);                             // 返回值：返回相对上一次位置的偏移值 带上限功能
uint8   absolute_encoder_init               (void);                             // 返回值：初始化状态 0-成功 1-失败
//=================================================声明 角度传感器 外接驱动================================================
#endif
