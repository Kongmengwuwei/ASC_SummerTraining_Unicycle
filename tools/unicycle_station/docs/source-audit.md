# 当前源码审计与文档差异

审计基线：开发开始时仅发现用户已删除的四张 `run_logs/` 图片。它们保持原状态，不恢复、不覆盖。上位机代码与依赖位于项目总目录的 `tools/unicycle_station/`。ADS `.project/.cproject` 仅增加 tools 排除，防止 Python 环境和主机测试 C 文件被当作固件源文件。

## 已核对文件与结论

| 文件 | 核对结果 |
|---|---|
| code/vofa.c、vofa.h | 原上行为 att/run，原下行为 speed/stop；没有旧文档中的 s_tune_tbl/ack 通用调参实现。新增 cfg/stat，保留合法完整行的旧命令。 |
| code/param.c、param.h | g_param_table 共 72 项，含范围与类型；按名称哈希存 Flash；已有 CRC 与逐字回读验证。本次仅开放 param_count()，复用保存实现。 |
| code/control.c、control.h | 已有正式 Run、视觉速度/转向、HOLD、斑马线/丢线/视觉保护。Roll rate 当前使用位置式，Pitch rate 为增量式，Yaw rate 为位置式。未修改控制算法、增益、极性或电机输出。 |
| code/menu.c | 车身菜单保留二次确认；主菜单有 Run 和 Run Test，Run Test 含 Balance/Remote。菜单参数组实际为当前表中的项；若历史文字列出 r_rcy_tau/r_rcy_limit 等，不能据此发送不存在的参数。 |
| user/cpu0_main.c | 前台先 vofa_cmd_poll，然后 menu_run/IPM 保存/vofa_poll。配置、Schema 与 Flash 保持在该前台调用链。 |
| user/cpu1_main.c | CPU1 摄像头/视觉循环，没有新增通信或电机责任。 |
| user/isr.c、isr_config.h | CPU0 1ms 控制和 UART2/UART3；CPU1 相机/DMA/UART1。未改变中断优先级和核归属。 |
| README、调参命令、控制调试指南、菜单指南、project-map | 多处残留旧阶段信息；以当前代码及本次 protocol/channels 文档为准。旧整定指南已明确标记历史适用范围，不能把旧算法增益照搬到当前固件。 |

## 重点差异

- “尚未实现 Run/视觉到控制”与当前源码冲突，已纠正当前能力说明。
- 旧 `<参数名> <值>`、`ack:`、roll/pit/yaw/mot/trk/bal 文本帧已不对应当前 vofa 实现。当前参数命令为 cfg v1；原始 att/run 通道定义继续保留。
- Roll 角速度环当前调用 `pid_loc_calc_limited()`，旧“先给增量 ki”笔记不能当本固件直接调参步骤。
- 当前元素实现有斑马线、十字、环岛、坡道；旧 README 的路障不应宣称已实现。
- CH24 bit1 为 Balance 仍闭环；正式 Run 结束后可能保持 Balance，不能仅凭 run_active=0 允许 Flash。
- 新增 stat 末尾 PID 写权限位图，让 PC 按 MCU 当前闭环状态锁定参数；没有以“看到某条曲线”推断可写权限。

## 固件安全影响

新增处理只放在现有 vofa.c 翻译单元，避免依赖手改 ADS 生成清单。既有 control_stop、安全闸、模式互斥、CPU1 信箱和 1ms 控制子速率未修改。

配置值先完整解析，拒绝 NaN/Inf 和不符合类型的值，再做短临界区的有限写入。没有在 ISR 中加格式化、Flash、分配或字符串解析；UART TX 循环改为明确最多 16 次。

超长/非 ASCII 控制字符/截断行整体作废，取消旧的无换行前缀重同步；对依赖“不发结束符”的旧无线发送端，这是有意的兼容性收紧。发送端应发送完整 CRLF 行。

Flash 仍是前台阻塞硬件操作，但仅在静止判据满足后执行。新增保存不改变现有菜单或 IPM 自动保存路径；这些既有路径的硬件停顿特性仍需实测。状态和 Schema 会增加 UART2 带宽占用，整帧丢弃计数和遥测缺口可用于观察。

用户明确要求暂不进行 ADS 构建测试。因此没有声称本次固件已完成 TASKING 编译链接，也不应使用旧 hex 推断成功。源码主机契约测试不代替目标 MCU 构建、时序测量或台架验证。
