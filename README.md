# Q-SmartCar

TC264D 双核 Q 型独轮车调试工程。当前用于姿态、三轴串级、电机、参数和视觉分阶段调试。

## 当前状态

| 模块 | 状态 |
|---|---|
| IMU660RB、Mahony、三轴映射 | 代码已实现，轴向已实车确认 |
| IPS200 菜单、UART0 波形/调参、Flash 参数 | 代码已实现 |
| A/B/C 电机点动与单轴 Test/Wave | 代码已实现，PID 默认值全为 0 |
| CPU1 大津法、八邻域、边线、中线 | 已接入 |
| 斑马线、十字、环岛、坡道、路障 | 已接入 CPU1，结果只用于调试 |
| 视觉到 Yaw/速度闭环 | 未接入 |
| 主菜单 `Run` | 占位，不会启动电机 |


有效 Flash 参数会覆盖全零默认 PID。

## 硬件

| 部件 | 接口 |
|---|---|
| IMU660RB | SPI0：SCK `P20_11`，MOSI `P20_14`，MISO `P20_12`，CS `P20_13` |
| A/B 动量轮 | CYT2BL3 UART3 @460800：TX `P15_6`，RX `P15_7` |
| C 行进轮 | PWM `P21_3`，DIR `P21_2`，编码器 `P33_7/P33_6` |
| MT9V03X | D0-D7 `P00_0~P00_7`，VSYNC `P02_0`，PCLK `P02_1`，UART1 `P02_2/P02_3` |
| IPS200 | SCK `P15_3`，MOSI `P15_5`，CS `P15_2`，RST `P15_1`，DC `P15_0`，BL `P15_4` |
| 按键 | 上 `P20_6`，下 `P20_7`，确认 `P11_2`，返回 `P11_3` |
| UART0 | 烧录器虚拟串口 `P14_0/P14_1`，115200 |

```text
Roll   左右倾斜   A/B 差动
Pitch  前后倾斜   C 轮
Yaw    水平转向   A/B 同向
```

## 双核与调度

| | CPU0 | CPU1 |
|---|---|---|
| 主循环 | 菜单、IPS200、UART0 | 摄像头取帧和整帧视觉 |
| 中断 | 1 ms 控制、UART0、UART3 | VSYNC、DMA 完成、UART1 |
| 电机 | 可以访问 | 禁止访问 |

PCLK 直接触发 DMA，不进入 CPU。控制和视觉通过序号锁信箱交换数值；屏幕图像使用 request/read/release 快照。链接默认数据主机是 CPU0，CPU1 图像数据显式放入 `cpu1_dsram`。

```text
1 ms    陀螺仪、三轴角速度环
5 ms    C轮编码器采样、Mahony、角度环、Yaw外环、跨核参数
10 ms   按键扫描
20 ms   Pitch速度环、Roll飞轮回收环
每帧    大津法、八邻域、中线、元素状态机
```

控制结构：

```text
Roll   飞轮回收 → 角度 → 角速度
Pitch  速度     → 角度 → 角速度
Yaw              航向 → 角速度
```

Roll/Pitch 角速度环为增量式 PID，其余环为位置式 PID。A/B 混控优先保证 Roll，Yaw 只使用剩余输出余量。


## 菜单

```text
MENU
├─ Params
│  ├─ Attitude
│  ├─ Roll
│  ├─ Pitch
│  ├─ Yaw
│  ├─ Camera
│  ├─ Motor
│  ├─ Zero
│  ├─ Save
│  └─ Back
├─ Image
└─ Run
```
`Run` 当前只显示 `RUN NOT ENABLED`。

Test/Wave：

- `Rate`：只开角速度环。
- `Angle`：开角速度环和角度环。
- `Speed`：再开最外环；Yaw 没有 Speed。
- 三轴 Test/Wave 要求 IMU 初始化、静止标定和姿态收敛有效；Roll/Yaw 还要求 CYT2BL3 在线。Camera Test 只检查摄像头状态。
- Motor Jog 持续 1500 ms，结束后自动停机并关闭 `mot` 波形。

## UART0 与参数

UART0 固定 115200，使用 VOFA+ FireWater 文本帧。波形模式：

```text
off  imu  att  roll  pit  yaw  trk  mot  dash
```

常用命令：

```text
axis roll|pitch|yaw|next
ring rate|angle|vel|next
kp|ki|kd <value>
set <name> <value>
get <name>
list
save
wave <mode>
ping
```

非法格式、NaN 和 Inf 会被拒绝；有限越界值会钳位到参数允许范围，方向参数会归一为 `+1/-1`。测试运行时不能切轴、切环或保存，只能修改当前已启用串级内的 PID。

参数存放在 DFlash 扇区 0 第 11 页。v9 记录带 CRC32、范围和方向校验，并在写入后回读确认；合法 v8 参数仍可加载，下一次 Save 写为 v9。

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

Image 页按原比例显示为 320×142，可切换灰度、二值、二值加边线。图像显示在 IPS200。

元素结果包括 `active_elem`、`speed_scale` 和 `stop_request`。当前 CPU0 只保存并发送这些调试量，不修改目标速度、目标航向或电机状态。元素阈值和状态机仍需逐项实车标定。

## 安全与下一步

- 上电默认 STOP，A/B 软件刹车锁定。
- IMU、姿态或驱动异常会阻止测试；保护角越界会停机。
- 返回键用于停止当前 Test/Wave 或点动。
- Run 未实现前，不允许用单轴 Test/Wave 自由落地。

建议顺序：

```text
IMU → 电机/编码器方向 → Roll → Pitch → Yaw
→ 机械零点 → 普通视觉 → 元素识别
→ 最后实现 Run、视觉转向和失联保护
```

详细步骤见 [控制调试指南.md](控制调试指南.md) 和 [视觉调试指南.md](视觉调试指南.md)。
