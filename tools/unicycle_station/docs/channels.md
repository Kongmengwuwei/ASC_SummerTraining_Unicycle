# 独轮车通道

`att:` 为 Roll、Pitch、连续 Yaw，单位 °。3D 默认显示映射使用这三路；在实车上逐轴校验，不能根据模型猜 IMU 安装方向。

| run CH | Profile 通道 | 含义 / 单位 |
|---:|---|---|
|0|uptime_ms|MCU uptime / ms|
|1|roll|Roll 实际角 / °|
|2|roll_target|Roll 目标角 / °|
|3|roll_rate|Roll 角速度 / °/s|
|4|recovery_feedback|回收反馈 A−B / RPM|
|5|recovery_output|回收输出 / °|
|6|roll_output|Roll 混控前 / duty|
|7|lean_offset|压弯动态零点 / °|
|8|yaw_rate_target|视觉目标 Yaw 角速度 / °/s|
|9|yaw_rate_command|斜率限制后的 Yaw 命令 / °/s|
|10|yaw_rate_actual|Yaw 实际角速度 / °/s|
|11|yaw_output_raw|Yaw 混控前 / duty|
|12|yaw_output_applied|Yaw 实际分配 / duty|
|13|flywheel_common_rpm|A/B 共模 / RPM|
|14|direction_offset|方向像素误差 / pixel|
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
