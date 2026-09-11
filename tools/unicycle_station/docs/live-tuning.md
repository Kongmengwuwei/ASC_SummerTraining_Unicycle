# 0.5.1 · Run Test 波形与运行中调参

更新上位机并将当前车端源码按原流程构建、烧录。程序入口仍是 `start_app.bat` 或 `dist/EmbeddedStation/EmbeddedStation.exe`。

## 一边看波形，一边调参数

- 车身进入 Run Test → Balance 或 Remote 后，完成上位机握手即可接收完整诊断帧。单轴 Test 同样有波形，不必启动正式 Run。实时波形页勾选通道，必要时点击“恢复实时显示”。
- “调参工作台”保留 Roll/Pitch/Yaw，并新增“速度 / 转向”和“压弯”，参数与相应波形同屏显示。
- 可勾选“只看当前可调”筛选参数。修改“待应用值”后，点击“应用所选”或“应用当前组”，核对差异后写入 RAM。MCU 当前值以车端应答为准；不会每输入一个数字就发送，也不会自动保存 Flash。
- 运行中可写不等于所有参数都会立即影响当前模式。Run 速度规划和视觉转向参数在正式 Run 中使用；Remote 行驶速度仍由遥控页输入决定。压弯参数在相关压弯逻辑启用时生效。

## 当前权限

Balance、Run 和 Remote 允许三轴 PID，以及以下 16 项在线控制参数：

- 速度：run_speed_straight / curve / cross / ring / ramp / lost。
- 加减速：run_accel_mps2、run_decel_mps2。
- 转向：direction_pixel_kp、direction_heading_kp、direction_curve_kff、direction_rate_kd。
- 压弯：lean_turn_kp、lean_speed_kp、lean_slew_dps、lean_max_angle。2026-09-11 替换了旧增益，公式与 Flash 迁移见 [压弯控制说明](../../../压弯控制说明.md)。

单轴 Test 只开放该测试实际参与的 PID 环路；Jog 全部锁定。电机方向、机械零点、保护阈值、IPM 等参数和 Flash 保存保持停车限制。断线、状态过期或模式变化时重新锁定不允许的项，车端每笔写入也会重新检查权限与范围。

“run:”只是诊断帧标签，真实 Run 状态位仍决定任务与轨迹记录，不会因 Run Test 出现波形就开始累计 Run 轨迹。姿态、速度、PID 输出可按实际激活环路观察；视觉规划等未启用模块的数值不能当成当前控制指令。

0.5.1 在线调参功能当时只进行了 PC/Mock 与主机 C 契约测试。2026-09-11 新压弯已完成 ADS 构建和相关主机回归，未烧录或实车验证，记录见压弯控制说明。
