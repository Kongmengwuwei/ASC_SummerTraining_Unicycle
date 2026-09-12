# 独轮车通道

源码核对：2026-09-07。当前固件标识 tc264-cfg1-task1，配置协议版本仍为 1。

`att:` 为 Roll、Pitch、连续 Yaw，单位 °。3D 默认显示映射使用这三路；在实车上逐轴校验，不能根据模型猜 IMU 安装方向。

| run CH | Profile 通道 | 含义 / 单位 |
|---:|---|---|
|0|uptime_ms|MCU uptime / ms|
|1|roll|Roll 估计角（IMU 融合）/ °|
|2|roll_target|Roll 目标角 / °|
|3|roll_rate|Roll 角速度 / °/s|
|4|recovery_feedback|回收反馈 A−B / RPM|
|5|recovery_output|回收输出 / °|
|6|roll_output|Roll 混控前 / duty|
|7|lean_offset|压弯动态零点 / °|
|8|yaw_rate_target|视觉目标 Yaw 角速度 / °/s|
|9|yaw_rate_command|斜率限制后用于积分航向目标的命令；非 Yaw 内环设定值 / °/s|
|10|yaw_rate_actual|机体系 gyro_z；压弯时不等同精确地面航向变化率 / °/s|
|11|yaw_output_raw|Yaw 混控前 / duty|
|12|yaw_output_applied|Yaw 实际分配 / duty|
|13|flywheel_common_rpm|A/B 共模 / RPM|
|14|direction_offset|方向像素误差（滤波后），当前限约 ±43.48 / pixel|
|15|lateral_error|BEV 横向误差 / 赛道半宽归一化|
|16|heading_error|赛道航向误差 / °|
|17|curvature|BEV 有符号归一化曲率|
|18|speed_plan|规划速度 / m/s|
|19|speed_ramp|斜坡速度 / m/s|
|20|speed_actual|C 轮实测速度 / m/s|
|21|momentum_scale|Yaw 动量权限 / 0–1|
|22|vision_quality|视觉质量 / 0–1|
|23|vision_age_ms|视觉帧年龄 / ms|
|24|state_flags|无符号状态位|

状态位：bit0 Run、bit1 Balance、bit2 Track Valid、bit3 本次 Run 视觉帧已应用、bit4 IPM、bit5 HOLD、bit6 斑马线停车请求、bit8–11 元素、bit12–14 停车原因。具体配置集中在 Profile 的 status_bits 中。

PC 派生通道包含 roll_error、yaw_clip、speed_error、vision_timeout 和 telemetry_lost，不增加 MCU 负担。设置页可以修改受限算术表达式，禁止任意代码执行。

时间戳以无符号 32 位差值处理自然回绕。反向跳变识别为复位或乱序，长间断不直接累加为丢帧。20 ms 预期间隔下缺口仅用于估计丢失遥测数，无法区分无线丢包、MCU TX 主动丢帧或采样配置变化，不能据此断言 1 ms 控制器停顿。

## 配置状态与任务通道

stat 为 15 个整数字段：运行基础模式、Run/Test/Jog 标志、IMU/驱动/摄像头/姿态状态、停止原因、参数修订与收发计数、PID 权限位图。完整顺序和位定义集中在 [protocol.md](protocol.md)，不能由某个波形 tag 推断模式或写权限。

task1 的 task 为 13 字段：uptime_ms、state、element、phase、track_mode、run_active、vision_age_ms、quality、transition_seq、event_drop、yaw_deg、speed_mps、vision_frame_seq；hello 后约 5 Hz。taskevt 为 sequence、uptime_ms、previous_state、current_state、element、phase、run_active 共 7 字段。道路状态枚举与轨迹使用方式见 [debug-workflow.md](debug-workflow.md)。

task 的 yaw/speed 来自同一 CPU0 快照；run 不包含完整 Pitch/Yaw 姿态，辅助 att 用于显示。task 道路分类不是固件新增的控制输入，PC 轨迹不是绝对定位。

## 参数快照与波形的区别

当前 g_param_table 有 74 项，编译默认 PID 非全零。参数的当前值由 Schema/get/set 应答和 revision 同步；run 的 25 通道不附带 24 个 PID 增益。历史日志中必须结合参数应答/试验快照解释调参过程。旧 roll/pit/yaw/mot/trk/bal 通道不存在，不能复用旧编号。

tx_drop、task.event_drop 和按 uptime 估计的遥测缺口计数覆盖不同路径；task 周期帧的带宽跳过也可能不增加 tx_drop。它们均不是可靠的无线丢包率测量。