# 当前源码核对与文档边界

核对日期：2026-09-07；基线为 HEAD `2a03d23` 的当前工作区源程序。本次仅更新项目文档及项目 skill，不更改固件、上位机程序、默认参数或历史验证产物。开始时工作区已有两份引脚文本删除，保持原状。

## 权威入口

| 内容 | 源码 |
|---|---|
| 默认值 / 类型 / 范围 / Flash | code/board_config.h、param.c、param.h |
| 控制模式 / PID / 退出 | code/control.c、control.h、pid.c |
| 页面 / 编辑门槛 / 分页保存 | code/menu.c |
| 视觉 / IPM / 元素 | code/image.c、perspective.c、element.c、vision_core.c |
| UART2 cfg、遥测与任务 | code/vofa.c、vofa.h、vofa_task.inc |
| 核归属、调度和内存 | user/cpu0_main.c、cpu1_main.c、isr.c、isr_config.h、Lcf_Tasking_Tricore_Tc.lsl |
| PC 参数和解析 | app/protocols/firewater.py、services/requests.py、ui/parameters.py、profiles/tc264_unicycle/ |

根目录的 [README](../../../README.md)、[控制指南](../../../控制调试指南.md)、[视觉指南](../../../视觉调试指南.md)、[菜单指南](../../../菜单指南.md)、[参数参考](../../../参数参考.md)、[命令说明](../../../调参命令.md) 和项目 skill 已按这些实现同步。代码注释自身也可能落后，冲突时以实际分支、赋值和调用关系为准。

## 本次修正的旧说明

| 旧说明 | 当前实现 |
|---|---|
| PID 全零默认；Reset 清零增益 | 非零 PID 默认；Reset 恢复 board_config 中的 PID、零点及 IPM 默认值 |
| Roll 内环增量式，必须先调 Ki | Roll rate 是 pid_loc_calc_limited；Pitch rate 才是增量式 |
| r_rcy_tau / r_rcy_limit / r_angle_limit 可调 | 三项均已移除；回收直接使用 A−B RPM，合成 Roll 目标限 ±10° |
| 180×80 ROI、八邻域、固定 7 行平均误差 | 188×120 整帧、逐行提边、来源标记、可调前瞻及 IPM/拟合 |
| 无 Run / 视觉只供诊断 | 正式 Run 已连接速度、转向、元素停车与压弯 |
| Camera 8 项、Element 14 项、Zero 有 Capture Zero | Camera 2 项+IPM；Element 8 项；Zero 4 项且无抓零动作 |
| 默认全部元素关闭、存在路障 | 默认 Cross=1，其余 0；只有斑马线/十字/左右环/坡道 |
| 元素限速是倍率、超时以帧数计 | 绝对 m/s 限速；超时/屏蔽用 10 ms 常量及 uptime；确认帧仍按帧数 |
| Remote 速度输入除以 30 | 当前 REMOTE_SPEED_INPUT_DIVISOR=40，超出 ±1.50 m/s 拒绝，有效速度原始输入实际为 ±60 |
| 旧 PID 名称直写 / ack / roll/pit/yaw/mot/trk/bal | cfg v1，att/run/stat/par/rsp，加 task/taskevt |
| Run 停止代表三电机锁停 | run_hold_balance 保留平衡；公共 control_stop 才完全锁停 |
| 主菜单直接 Balance | 主菜单 Params/Image/Run/Run Test，后者内有 Balance/Remote |
| Camera/Element 显示 G/B/E/M 与 US/MAX | 当前无这些面板；无分段耗时遥测，不能引用旧性能数值 |

## 已发现但本次未改代码的边界

1. **Roll 菜单权限索引残留。** s_roll_items 现为 9 项，索引 6 已是 r_rcy_kp，但 group_param_edit_allowed() 仍把索引 3～6 视作 Angle。结果为 Angle Test 下 Speed Kp 可编辑，实际回收环没有启用。cfg_pid_mask() 按 3 项一组正确限制；不要把此菜单现象写成回收已参与控制。
2. **菜单与 cfg 写入保护不同。** cfg 在 Balance/Run/Remote 中除 PID 外开放明确的速度、转向和压弯白名单（见 live-tuning.md），其余非 PID 运行写入仍锁定；菜单主要限制 Motor/Zero/Odometry 和单轴 Test 的环。其他参数页可编辑范围较宽，Jog 使用 START_STOP，因此仅看 start_flag 的菜单限制也不能等同完整运行保护。
3. **保存门槛不同。** 菜单 Save/IPM 自动保存没有 cfg 的 500 ms 稳定轮速与 100 ms RX 静默门槛。param_save()/param_save_names() 提供 CRC/回读，但不等于所有调用入口都实施了相同静止判据。
4. **注释与实际退出不同。** menu_run() 附近旧注释只提斑马线保留平衡；control.c 的 manual/lost/vision 路径也会 run_hold_balance。状态显示标志不能替代 start_flag/运行标志和实际输出判定。
5. **有效性不止一个标志。** vision_core 发布 g_direction_valid；my_image.Track_Valid 是中间中线状态。task 分类依据 CPU0 已接收状态，且超时判断为年龄 >100 ms；Run 退出采用 ≥100 ms，两者边界相差一拍。
6. **遥测丢弃计数并非全覆盖。** tx_push 队列溢出和部分带宽门槛会累加 tx_drop；task 周期帧因预留空间不足而跳过、普通波形格式化超长返回等路径不一定累计。taskevt 的 event_drop 只计事件环溢出，不能把这些计数当成全部无线丢包数。

7. **Jog 编辑态返回不是立即停机。** motor_jog_action 不设置 s_test_on，Jog 的 start_flag 仍为 START_STOP；进入参数编辑后第一次返回只退出编辑，第二次返回触发 set_page/stop_local_test 才停止。确认任一 Jog 动作或完整 stop 也可停止。
8. **Remote 有两层范围检查。** 文本层允许 ±90，控制层要求 speed_raw/40 在 ±1.50 m/s 内，越界拒绝，不自动钳位。

以上是当前源码行为与需要关注的实现差异，不表示本次已经修复。文档不应将这些差异隐藏成统一规则。

## PC 与固件验证记录

当前上位机验证见 [validation.md](validation.md)；历史版本的验证记录保存在 Git 提交 `d3d9c31` 及之前的历史中，不能当作当前固件或安装包的新验证结果。

cfg/task 源码及 PC 主机测试不代替当前 TASKING 编译、1 ms ISR 时序、实际 UART2 吞吐、Flash 停顿和车辆行为。上位机 Profile、Mock、示例预设与现有 EXE 是独立快照；车端实际参数须经 Schema 读取，电脑上有某个 ELF 或 EXE 不能证明车辆已运行相应固件。
## 2026-09-09 转弯压弯专项

完整分析见 [转弯饱和与压弯评估](../../../转弯饱和与压弯评估.md)。当时源码已确认压弯依赖航向跟踪误差且受原始视觉偏差门控；回收输出可能抵消压弯；共模降权不能代表单轮转速余量；Yaw 裁剪未直接抑制上游航向目标累积。动态加速度造成姿态误差属于待实测假设。没有实车日志，不把上述问题认定为本次倒车的已证实原因。

本次仅把 Lean Kp 范围扩大为 0～100、Lean Max 扩大为 0～10°，默认 2.5/5°及 Roll 合成 ±10°不变。主机回归 5 项通过；ADS headless 构建被 Project Booster 的图形工作台依赖阻断，未取得目标构建成功记录，未烧录。

## 2026-09-11 压弯算法更新

旧视觉偏差门控与航向误差压弯已替换为持续转弯率方向项＋`v*r/g` 速度项，含速度滤波/淡入、连续转弯率死区及可调斜率；新三项参数不复用旧 Flash 增益，其他有效旧键保留，总数 74。MCU、菜单、在线白名单和 Profile 同步。25 项针对性主机回归及 ADS/TASKING 构建通过（0 errors；既有 W515 警告），未烧录、未实车验证。完整当前行为和证据见 [压弯控制说明](../../../压弯控制说明.md)。

## 2026-09-12 Pitch 角速度坐标修正

Pitch 全串级及单轴测试改用 `gyro_y*cos(roll)-gyro_z*sin(roll)`，原始机体系角速度、Mahony、CPU1 反馈及其他控制参数保持原义。角度更新时缓存三角函数，陀螺仍以 1 ms 更新；初始化和串级复位清除缓存有效标志。

8 项主机回归通过：实际 C 投影及两个控制入口覆盖左右倾斜、不同 Pitch/Yaw 运动组合、直立旧响应、持续转弯零 Pitch 漂移输入、缓存更新/复位及输出限幅，同时回归原压弯、Flash 迁移和协议。ADS/TASKING Debug 构建通过，0 errors，32.5 s；仅现有 `vofa_task.inc:35` W515。输入 SHA-256：`c5e88cb2fad9af266cb5059e2fadd79fe33f535b0b4f9c5cd83a795f2294a553`。最新 ELF/HEX/MAP 路径及哈希见 [.ads/build/latest.json](../../../.ads/build/latest.json)。未烧录、未进行修正后的实车验证，也未测量新增路径在车端的最坏执行时间。

## 2026-09-12 视觉方向误差范围扩大

共用 `DIRECTION_CAMERA_LIMIT` 由 5000 调至 15000（除以 345 后约 ±43.48 pixel），同时覆盖图像发布、CPU0 接收和方向计算；未改运行时增益、参数数量或 Flash 格式。当前控制/视觉指南、通道说明和 skill 项目参考已同步；历史日志保留原 14.493 pixel 限幅证据。

新增真实 C 函数回归覆盖正负大偏差、小偏差、部分有效行归一化、回退/HOLD/失效、CPU0 限幅、Yaw 动量权限、斜率及航向超前限制。连同 Pitch 坐标、压弯和协议/ISR 检查，共 9 项通过。ADS/TASKING Debug 构建 28.1 秒通过，0 errors，仍有 `code/vofa_task.inc:35` 既有 W515 警告。输入 SHA-256：`7264fe19e3259febcbc94e59e01bc5aab2719655570ca54919f10381d4544c5d`。skill 校验通过。未烧录或进行实车验证；副作用及对照步骤见 [控制调试指南](../../../控制调试指南.md)。
