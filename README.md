# robomaster_heavy_robot

英雄机器人云台控制代码（STM32F407 + M3508/C620 + 铅丝杆 pitch 轴）。

## 目录

| 目录 | 内容 |
|---|---|
| `balance_application/` | 应用层：云台、射击、IMU、USB、遥控、控制算法 |
| `balance_bsp/` | 板级支持：CAN、串口、IMU 驱动、滤波 |
| `Core/` | CubeMX 生成的 HAL 初始化 |
| `Drivers/`、`Middlewares/` | STM32 HAL / CMSIS / FreeRTOS |
| `USB_DEVICE/` | USB CDC 虚拟串口 |
| `MDK-ARM/` | Keil MDK 工程（`gimbal.uvprojx`） |
| `docs/` | 协议核对、问题分析、命名规范 |
| `backup_naming_*/`、`patch_backup_*/` | 重构前的文件备份 |

## 文档

- [Pitch 轴 C620 协议核对与问题分析](docs/pitch_3508_C620_协议核对与问题分析.md)
- [命名与符号规范](docs/命名与符号规范.md)

## 构建

用 Keil MDK 打开 `MDK-ARM/gimbal.uvprojx`，选择目标 `25_hero_gimbal` 编译。
