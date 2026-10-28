# 卡丁快跑 · Karting-Project

CYT4BB7（Infineon TRAVEO™ T2G，CM7 双核）卡丁车固件，以及配套的轻量上位机。

- **固件** —— IAR EW 9.2，双核工程 `cyt4bb7_cm_7_0.ewp`（控制）与 `cyt4bb7_cm_7_1.ewp`（惯导）
- **上位机** —— .NET 9 WinForms，UDP 收遥测、在线整定参数、录 CSV
- **实物** —— 后轮双电机速度闭环 + 舵机位置闭环；遥控接收机与上位机两条并列的控制通路

## 目录

| 路径 | 内容 |
|---|---|
| `project/user/` | 入口与中断：`main_cm7_0.c` / `main_cm7_1.c` / `cm7_*_isr.c` |
| `project/code/` | 业务代码（见下表） |
| `project/iar/` | IAR 工程与调试配置 |
| `libraries/` | 逐飞 CYT4BB7 开源库 + Infineon SDK |
| `tools/KartHostLite/` | 轻量上位机，见其 [README](tools/KartHostLite/README.md) |

`project/code/` 里几个主要文件：

| 文件 | 作用 |
|---|---|
| `Motor_Ctrl.c` | 后轮速度闭环（斜坡 + 前馈 + PID），左右独立 |
| `Servo_Ctrl.c` | 舵机位置闭环（绝对编码器反馈，软限位 ±28°） |
| `Remote_Ctrl.c` | 遥控接收机解析、模式状态机（OFF/READY/DRIVE/ESTOP/FAILSAFE） |
| `Host_Ctrl.c` | 上位机链路：帧协议、遥测上报、在线调参、驾驶权仲裁 |
| `Param_Storage.c` | 参数存 Flash（逐飞第 0 页），上电自动回灌 |
| `Menu.c` / `Param_Page.c` / `Debug.c` | IPS200 三级界面：主菜单 / 参数页 / 调试与数据页 |
| `IMU_Analysis.c` | CM7_1 侧惯导分析，结果经共享内存给 CM7_0 |
| `common_menu.c` / `PID.c` | 通用菜单控件与 PID 实现 |

## 上位机怎么跑

`tools/KartHostLite/` 下有 `build.bat` / `run.bat` / `publish.bat`（需要 .NET 9 SDK）。
`publish.bat` 产出 `dist/KartHostLite.exe` —— 自包含单文件，双击即用，不用装运行时。

没有硬件也能验证协议层：

```bat
KartHostLite.exe --selftest          :: 30 项断言，退出码 = 失败项数
KartHostLite.exe --shot-demo out.png 4   :: 灌假数据离屏截图，检查界面
```

## 与队友的合作

本项目由我与队友 **zjc** 合作完成，他的仓库：
**[hz303/zjc_kart_cyt4bb7](https://github.com/hz303/zjc_kart_cyt4bb7)**。

## 说明

- `libraries/` 是逐飞与 Infineon 的第三方库，各自带 LICENSE，版权归原方。
- 固件里的 WiFi 热点、目标 IP 是为了台架调试写死的现场配置
  （`project/code/Host_Ctrl.h` 的 `HOST_WIFI_LIST` / `HOST_TARGET_IP` / `HOST_TARGET_PORT`），
  换场地或换电脑时需要改这里，并让上位机连同一个 IP/端口。
- 两个核心必须同时编译：改完 `project/code/` 要编 CM7_0，改完 `IMU_Analysis` 要编 CM7_1。
  新加的 `.c` 文件**必须登记进 `.ewp`**，否则不参与编译而且不会报错。
