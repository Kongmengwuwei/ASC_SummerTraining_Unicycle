# Q-SmartCar

TC264D 双核 Q 型独轮车调试工程，主要用于姿态、三轴串级、电机、参数和视觉调试。

## 当前状态

| 模块 | 状态 |
|---|---|
| IMU660RB、Mahony、三轴映射 | 代码已实现，轴向已实车确认 |
| IPS200 菜单、无线串口波形/调参、Flash 参数 | 代码已实现 |
| A/B/C 电机点动与单轴 Test | 代码已实现，PID 默认值全为 0 |
| CPU1 大津法、八邻域、边线、中线 | 已接入 |
| 斑马线、十字、环岛、坡道、路障 | 已接入 CPU1，结果只用于调试 |
| 视觉到 Yaw/速度闭环 | 未接入 |
| 主菜单 `Run` | 占位，不会启动电机 |


Flash 里有值时，会覆盖默认参数。

## 硬件

| 部件 | 接口 |
|---|---|
| IMU660RB | SPI0：SCK `P20_11`，MOSI `P20_14`，MISO `P20_12`，CS `P20_13` |
| A/B 动量轮 | CYT2BL3 UART3 @460800：TX `P15_6`，RX `P15_7` |
| C 行进轮 | DIR `P21_4`，PWM `P21_5`，编码器 `P33_7/P33_6` |
| MT9V03X | D0-D7 `P00_0~P00_7`，VSYNC `P02_0`，PCLK `P02_1`，UART1 `P02_2/P02_3` |
| IPS200 | SCK `P15_3`，MOSI `P15_5`，CS `P15_2`，RST `P15_1`，DC `P15_0`，BL `P15_4` |
| 按键 | 上 `P20_6`，下 `P20_7`，确认 `P11_2`，返回 `P11_3` |
| 无线转串口 | UART2 @115200：TX `P10_5`，RX `P10_6`，RTS `P10_2` |

```text
Roll   左右倾斜   A/B 差动
Pitch  前后倾斜   C 轮
Yaw    水平转向   A/B 同向
```

## 双核与调度

| | CPU0 | CPU1 |
|---|---|---|
| 主循环 | 菜单、IPS200、无线串口 | 摄像头取帧和整帧视觉 |
| 中断 | 1 ms 控制、UART2、UART3 | VSYNC、DMA 完成、UART1 |

工程不调 `debug_init()`，UART0 从不初始化。`ips200_init()` 会把 `zf_assert` / `zf_log` 的输出接管到屏幕上。
| 电机 | 可以访问 | 禁止访问 |

PCLK 直接触发 DMA，不进入 CPU。控制和视觉通过序号锁信箱交换数值；屏幕图像使用 request/read/release 快照。链接默认数据主机是 CPU0，CPU1 图像数据显式放入 `cpu1_dsram`。

```text
1 ms    陀螺仪、角速度环
5 ms    姿态、角度环、Yaw 外环、C 轮编码器
10 ms   按键扫描
20 ms   Pitch 速度环、Roll 回收环
每帧    视觉与元素处理
```

控制顺序：Roll 回收→角度→角速度，Pitch 速度→角度→角速度，Yaw 航向→角速度。A/B 先保 Roll，Yaw 用剩余余量。


## 菜单

```text
MENU
├─ Params
│  ├─ Attitude
│  ├─ Roll
│  ├─ Pitch
│  ├─ Yaw
│  ├─ Camera
│  ├─ Element
│  ├─ Motor
│  ├─ Zero
│  ├─ Save
│  ├─ Reset
│  └─ Back
├─ Image
└─ Run
```
`Run` 目前只是占位。

各页的动作行：

| 页 | 动作行 |
|---|---|
| Roll / Pitch / Yaw | `Rate` / `Angle` / `Speed` 分环 Test |
| Motor | 六个架空点动 |
| Zero | `Capture Zero` 抓当前姿态角当机械零点 |
| Camera / Element | 只看实时数据，自动开 `trk` 波形 |

- `Rate` 只开角速度环。
- `Angle` 再开角度环。
- `Speed` 再开最外环；Yaw 没有 `Speed`。
- 三轴 Test 需要 IMU、标定和姿态收敛正常；Roll/Yaw 还要 CYT2BL3 在线。
- Motor 点动无固定时长，再按一次同一行、按返回键或驱动掉线才停。

## 无线串口与参数

波形和调参走 UART2 @115200 的无线转串口模块，使用 VOFA+ FireWater 文本帧。
模块连接：RX→`P10_5`、TX→`P10_6`、RTS→`P10_2`。

| tag | 什么时候出 |
|---|---|
| `att` | Attitude 页按 `Test: ON` |
| `roll` / `pit` / `yaw` | 对应轴页启动分环 Test，由 `control_test_start()` 按轴给 |
| `mot` | Motor 页点动，由 `control_jog_start()` 给 |
| `trk` | 进 Camera 或 Element 页 |

其余页面一律 `off`。

下行命令只保留这一种格式：

```text
<参数名> <值>\r\n          例如  r_rate_kp 12.5
```

回 `ack:1.000` 或 `ack:0.000`。

- 只接受 `s_tune_tbl` 里的 24 个 PID 名字，其余参数用菜单或按键改。
- 非法格式、NaN、Inf 会被拒绝；越界值会钳位。
- 发车或点动期间拒绝改参数；测试期间只放行当前轴、当前最高启用环之内的 PID。
- `Params → Reset` 会清掉参数页并恢复默认值，需要二次确认。

## 视觉与元素

CPU1 当前流程：

```text
188×120 原图
→ 底部居中 180×80 ROI
→ 大津二值化
→ 八邻域提取左右边线
→ 标准赛宽补中线
→ 元素检测与补线
→ 最终偏差和元素结果
→ 发布给 CPU0
```

Image 页用于看屏幕，可切灰度、二值和边线图；`trk` 波形在 `Params → Camera` 和 `Params → Element` 页启动。

元素结果只做调试显示，当前不会直接改电机输出。阈值和状态机仍需实车标定。

## 安全与下一步

- 上电默认 STOP，A/B 软件刹车锁定。
- IMU、姿态或驱动异常会阻止测试；保护角越界会停机。
- 返回键可停止当前 Test 或点动。
- `Run` 未实现前，不要自由落地测试。

建议顺序：

```text
IMU → 电机/编码器方向 → Roll → Pitch → Yaw
→ 机械零点 → 普通视觉 → 元素识别
→ 最后实现 Run、视觉转向和失联保护
```

详细步骤见 [控制调试指南.md](控制调试指南.md) 和 [视觉调试指南.md](视觉调试指南.md)。
