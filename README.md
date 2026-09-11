# Q-SmartCar

TC264D 双核 Q 型独轮车工程，包含三轴平衡、视觉循迹、车身菜单和 Windows 上位机。

> 核对日期：2026-09-07；基线为当前工作区源码，HEAD 为 `2a03d23`。本文说明已实现行为，不代表当前固件已通过 TASKING 构建或实车验证。编译默认值来自 `code/board_config.h`，上电实际参数以 Flash 加载后的 `g_param` 和 MCU Schema 为准。

## 文档入口

| 文档 | 内容 |
|---|---|
| [控制调试指南](控制调试指南.md) | 当前 PID、混控、模式、安全退出和调试顺序 |
| [视觉调试指南](视觉调试指南.md) | 整帧逐行提边、IPM、中线、元素与诊断 |
| [菜单指南](菜单指南.md) | 当前页面、动作、参数与保存范围 |
| [参数参考](参数参考.md) | 从源码提取的全部 72 项参数、默认值、范围和菜单步长 |
| [调参命令](调参命令.md) | UART2 命令与遥测入口 |
| [上位机说明](tools/unicycle_station/README.md) | 安装、Mock、波形、参数、任务、日志和回放 |
| [源码核对记录](tools/unicycle_station/docs/source-audit.md) | 已修正文档差异及尚存的实现边界 |

项目 skill 位于 `.agents/skills/develop-tc264-unicycle/`，开发时以当前源码和具体接口文档为准。

2026-09-11 补充：在线调参和 Run Test 波形见 [实时调参说明](tools/unicycle_station/docs/live-tuning.md)；实车原地振荡记录见 [Pitch 分析](tools/unicycle_station/docs/pitch-analysis-20260911/分析与调参建议.md) 与 [Roll 分析](tools/unicycle_station/docs/roll-analysis-20260911/分析与调参建议.md)。构建辅助工具用法见 [ADS 工具说明](tools/ads/README.md)。

## 当前能力

| 模块 | 源码状态 |
|---|---|
| IMU660RB / Mahony | 初始化、静止标定、姿态收敛及链路/发散保护 |
| 三轴控制 | Roll 回收→角度→角速度；Pitch 速度→角度→角速度；Yaw 航向→角速度 |
| 调试入口 | A/B/C Jog、单轴 Rate/Angle/Speed Test、Balance、Remote、1m Test |
| 正式 Run | 视觉转向、分段速度规划、压弯、Yaw 共模动量降权、失帧/丢线/斑马线退出 |
| 视觉 | 188×120 整帧、大津二值化、逐行提边、来源标记、半宽表、中线拟合、IPM |
| 元素 | 斑马线、十字、左右环岛、坡道；没有路障实现 |
| 通信 | cfg v1 / task1；att、run、stat、par、rsp、task、taskevt |
| 参数 | 72 项运行参数、按名称及类型键保存、CRC 与逐字回读校验 |

PID 默认值已经不是全零；例如 Roll rate 为 `(-18, 0, -7.5)`，Pitch rate 为 `(20, 0.5, 0)`，Yaw rate 为 `(50, 0, 0)`。Reset 会恢复这些非零默认值及默认零点、IPM 矩阵。源码默认参数不是适用于其他车辆的通用整定值。

## 双核与实时调度

| 执行位置 | 职责 |
|---|---|
| CPU0 前台 | 按顺序执行 `vofa_cmd_poll → menu_run → control_ipm_flush → vofa_poll` |
| CPU0 1 ms 中断 | 电机链路、陀螺仪/角速度、运行状态、安全闸、视觉结果接收、控制输出、定长遥测快照、有界 UART 搬运 |
| CPU0 5 ms | 姿态、角度/Yaw 外环、C 编码器采样、向 CPU1 发布参数和运动反馈 |
| CPU0 10 ms | 按键扫描 |
| CPU0 20 ms | Pitch 速度环、Roll 回收/动量权限、方向控制和速度斜坡 |
| CPU1 前台 | 摄像头初始化及整帧图像、IPM、元素、结果和屏幕快照发布 |
| CPU1 中断 / DMA | UART1、VSYNC、DMA5 完成；PCLK 直接触发 DMA |

CPU1 不驱动电机。跨核通信使用 `vision_core.c` 的序号锁及 `__dsync()`；图像显示遵守 request/read/release 所有权协议。CPU0 状态在 `cpu0_dsram`，CPU1 图像状态在 `cpu1_dsram`，信箱在 `vision_shared`，部分 CPU1 算法在 `cpu1_psram`。

不调用 `debug_init()`，UART0 不初始化；`ips200_init()` 将断言/日志输出接到屏幕。

## 硬件接线（当前源码配置）

| 部件 | 接口 |
|---|---|
| IMU660RB | SPI0：SCK P20_11、MOSI P20_14、MISO P20_12、CS P20_13 |
| A/B 动量轮 | CYT2BL3，UART3 460800：TX P15_6、RX P15_7 |
| C 行进轮 | DRV8701E：DIR P21_4、PWM P21_5，17 kHz |
| C 编码器 | TIM2：脉冲 P33_7、方向 P33_6；每 5 ms 采样 |
| MT9V03X | D0–D7 P00_0–P00_7；VSYNC P02_0、PCLK P02_1；UART1 P02_2/P02_3 |
| IPS200 | SCK P15_3、MOSI P15_5、CS P15_2、RST P15_1、DC P15_0、BL P15_4 |
| 按键 | 上 P20_6、下 P20_7、确认 P11_2、返回 P11_3 |
| 无线 UART2 | 115200：MCU TX P10_5、RX P10_6；模块 RTS 接 P10_2，高电平暂停上行 |

Roll 为左右倾斜，A/B 差动；Pitch 为前后倾斜，C 轮驱动；Yaw 为水平转向，A/B 同向。混控先保 Roll，再把剩余 A/B 余量给 Yaw。

原 `推荐IO分配.txt`、`尽量不要使用的引脚.txt` 当前已删除。接线应核对 `board_config.h`、`user/isr_config.h` 和实际设备驱动头文件；此表只列当前占用，不是其他引脚均可使用的保证。

## 运行与停止

主菜单为 `Params / Image / Run / Run Test`。`Run Test` 内有 `Balance / Remote / Back`，参数页另有 Lean、Run、Odometry。

- Balance：三轴原地平衡，目标速度 0，启动时锁存航向，视觉不参与。
- Run：要求 STOP、摄像头 READY、新鲜且有效的视觉结果、IPM 有效，以及全部 Balance 安全条件；按键两次确认。
- Remote：由车身两次确认启动后，才接受 `speed:turn,speed`；没有 cfg 远程发车命令。
- 1m Test：使用三轴平衡直行验证里程，完成后仍保持平衡。
- Run 的人工结束、视觉超时、持续丢线和斑马线结束可退回 Balance；`run_active=0` 不等于电机停转。
- 基础运行状态非 STOP 时返回键、串口 `stop` 或控制安全故障走完全停机路径，清输出并锁 A/B 软件刹车。Jog 编辑参数时第一次返回仅退出编辑，需再返回离页才停止，详见菜单指南。

## 开发与验证

应用模块在 `code/`，入口/中断在 `user/`，第三方驱动在 `libraries/`，链接配置为 `Lcf_Tasking_Tricore_Tc.lsl`。ADS/TASKING 的 Build Project 是权威构建，`Debug/`、`Release/` 为生成目录，不能手改生成 makefile。上位机源码在 `tools/unicycle_station/`，应保持在 ADS 源码扫描排除项内。

推荐联调顺序：静止标定与轴向 → 架空电机/编码器 → 单轴 Test → 零点与 Balance → 里程标定 → 图像/IPM/元素 → 受约束的低速 Run。历史 PC 测试和发行包记录见 [验证记录](tools/unicycle_station/docs/validation.md)，不能代替本次目标构建或台架证据。
## 转弯饱和专项评估（2026-09-09）

当前源码的压弯链路、饱和原因假设、实车对照方法及优化优先级见 [转弯饱和与压弯评估](转弯饱和与压弯评估.md)。本次将 Lean Kp 上限调整为 100、Lean Max 上限调整为 10°；默认值仍为 2.5 / 5°，Roll 合成目标仍限 ±10°。该专项评估当时尚无实车日志，扩大范围不代表已解决倒车。
