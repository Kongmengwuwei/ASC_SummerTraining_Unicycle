# UART2 配置协议 v1

源码核对：2026-09-07，code/vofa.c、vofa_task.inc、param.c、board_config.h。

默认链路为 115200、8N1、ASCII、PC 无流控；MCU 模块自身的 P10_2 RTS 仍控制上行 FIFO 搬运。端口和 PC 串口配置可修改并保存。

## 帧与兼容性

保留 `att:roll,pitch,yaw`、25 通道 `run:`、`speed:turn,speed` 和完整行 `stop`。下行必须以 CR、LF 或 CRLF 结束。应用默认 CRLF。旧固件不回应 cfg 时仅监看及停车，参数保持锁定。

单条下行不含结束符最多 **63 字节**。超长、串口环溢出、1 秒未结束的残行均作废，丢弃到下一结束符。旧实现的“遇到新 speed/stop 前缀就提交残行”已移除，避免截断或粘连命令被执行。合法的完整旧命令保持兼容。Remote 的 turn 和 speed 输入范围均为 −90～90，speed ÷40 转为 m/s 后必须在 ±1.50 m/s 内，否则拒绝（有效 speed 原始输入实际为 ±60）；仅车身启动 Remote 后接受，超过 1000 ms 无有效命令归零速度但保持平衡。

| 请求 | 成功应答 |
|---|---|
| `cfg:hello,1` | `rsp:1,ok,hello,1,tc264-cfg1-task1,63` |
| `cfg:status,2` | `stat:…`，然后 `rsp:2,ok,status` |
| `cfg:schema,3` | 多条 `par:3,…`，最后 `rsp:3,ok,schema,72`（数量以当前表为准） |
| `cfg:get,4,r_rate_kp` | `rsp:4,ok,get,r_rate_kp,实际值` |
| `cfg:set,5,r_rate_kp,12.5` | `rsp:5,ok,set,r_rate_kp,实际值,APPLIED` |
| `cfg:save,6,Roll` | `rsp:6,ok,save,Roll,VERIFIED` |
| `cfg:save,7,all` | `rsp:7,ok,save,all,VERIFIED` |

seq 为 1–65535；PC 同一时刻仅保留一个在途配置请求。读取超时可重试一次；set/save 不自动重试，以免不确定应答造成重复写入。Schema 等待上限 15 秒，必须收到结尾数量与完整参数表匹配后才开放编辑。迟到应答不能匹配新的请求。

参数记录：

```text
par:<seq>,<name>,<float|int>,<value>,<min>,<max>,<group>,<step>,<flags>
```

flags：bit0=PID 运行期候选，bit1=危险参数，bit2=可持久化，bit3=只读。bit0 还必须与当前 stat 的 pid_write_mask 相交，不表示所有模式均可写。参数的类型与范围以 `g_param_table` 为准，显示名称、说明、步长等由 Profile 补充。未知参数自动显示在未分类组。

越界有限值被钳位后返回 `CLAMPED` 与实际生效值；NaN、Inf、非整数 int、非法极性（不是 ±1）被拒绝。RAM 写入在极短临界区中完成并更新 revision，PID 在下一控制拍刷新。不会自动保存 Flash。

错误应答 `rsp:<seq>,err,<code>,<message>`。当前会产生 `UNKNOWN_COMMAND`、`UNKNOWN_PARAM`、`INVALID_VALUE`、`OUT_OF_RANGE`、`RUNNING_LOCKED`、`UNSAFE_PARAM`、`SAVE_BLOCKED`、`FLASH_FAILED`、`BUSY`、`BAD_FORMAT`；PC 对不兼容握手报告 `VERSION_MISMATCH`。无法安全解析 seq 的损坏行只累计接收错误，不伪造 seq。

## 明确状态

10 Hz 状态帧有 **15 项**；相对需求中的 14 项示例，末尾追加了当前真正可写的 PID 位图：

```text
stat:uptime,start_mode,run_active,test_mode,jog_mode,imu_state,bldc_state,cam_state,att_state,run_stop,test_status,param_revision,rx_error,tx_drop,pid_write_mask
```

- start_mode：0 STOP、1 DRIVE_ONLY、2 BALANCE；正式 Run 另看 run_active。
- test_mode：0 无 Test，否则 `1 + axis*3 + ring`，axis=Roll/Pitch/Yaw 的 0/1/2，ring=Rate/Angle/外环 的 0/1/2。
- jog_mode：0 无 Jog，1/2/3 为 A/B/C。
- imu_state：bit0 初始化通过、bit1 静止标定通过、bit2 链路正常。
- bldc_state、cam_state：1 正常、0 未就绪或异常。
- att_state：bit0 已收敛，bit1 曾发散。
- run_stop：0 无、1 斑马线、2 丢线、3 视觉过期、4 人工、5 安全保护。
- test_status：与 `control_test_status_t` 一致，见固件头文件。
- pid_write_mask：bit0–8 Roll rate/angle/rcy 的 kp/ki/kd；bit9–17 Pitch rate/angle/vel；bit18–23 Yaw rate/angle。

PC 状态超过 700 ms 未更新即锁定参数。STOP 必须同时满足 start_mode、run_active、test_mode、jog_mode 为 0；Run 的正常终点可能仍处于 Balance，不能当作完全 STOP。

## 停车与 Flash

PC 停车使用独立线程，不进入普通请求队列。串口发送临界区仅允许当前一次有界写入完成，清理系统待发缓冲后发送 `\r\nstop\r\n`，前置结束符隔离可能的半条配置命令。Mock 直接发送 `stop\r\n`。已发送和 MCU 确认 STOP 是两种不同状态。PC 随后发送带序号的 status 请求，只有该请求成功且状态为 STOP 才确认停车，积压的旧周期状态帧不会单独触发确认。此无线命令不替代车身返回键或现场物理断电。

MCU 在 CPU0 前台解析完整 stop，取消尚未执行的 Schema/保存操作并调用原有 `control_stop()`。没有新增任何发车入口。

Flash 保存先排队，待下行已消费完且静默 100 ms 后执行；要求 STOP、无 Test/Jog/Remote/Run，A/B 链路正常且绝对回读转速均 ≤50 RPM、C 轮当前速度计数为 0，并连续满足至少 500 ms。存在 IPM 待保存时拒绝。该判据是现有传感器分辨率下的静止判断，不能证明车体绝对静止。

实际写入复用 `param_save()` / `param_save_names()`，它们已实现 CRC 与逐字回读比对。当前非空保存组为 Roll/Pitch/Yaw/Lean/Run/Camera/Element/Motor/Zero/Odometry/IPM，另支持 all；Other 是分类回退值，当前没有成员，保存会返回 BAD_FORMAT。组名大小写按源码精确匹配。PC 危险参数必须停车、解锁高级模式、预览新旧值并二次确认。

## 实时与带宽边界

格式化、Schema 遍历、参数解析、Flash 均在 CPU0 前台。1 ms 中断的 UART RX 最多搬 32 字节，TX 最多 16 字节。保留现有快照、双核职责和电机安全闸。

tx_push 装不下时整帧丢弃并累计 tx_drop；部分发送前跳过路径不累计（例如 task 预留空间不足、普通波形格式化超长），所以 tx_drop 不是全部遥测损失数。遥测为应答预留 384 字节；Schema 每次前台最多发一个参数，间隔至少 30 ms。hello 后提供额外 10 Hz 姿态（当原 att 模式未启用），便于 Run 时看到 Pitch/Yaw。

115200 的理论有效载荷约 11520 B/s。25 通道文本帧长度随数值变化，不能保证任何数值下持续 50 Hz 加完整配置流都无丢帧；PC 将遥测缺口与控制停顿区分。准确单向链路时延无法由未同步 PC/MCU 时钟直接求得，界面提供接收年龄、遥测间隔和应答耗时，不冒充绝对链路时延。


## task1 诊断扩展（cfg v1 保持兼容）

新 hello 固件标识为 `tc264-cfg1-task1`，不增加下行命令。完成 hello 后，独立于普通 VOFA 模式输出：

```text
task:uptime_ms,state,element,phase,track_mode,run_active,vision_age_ms,quality,transition_seq,event_drop,yaw_deg,speed_mps,vision_frame_seq
taskevt:sequence,uptime_ms,previous_state,current_state,element,phase,run_active
```

`task` 固定 13 项、5 Hz；`taskevt` 固定 7 项，单调 uint32 序号，只在道路、环岛阶段或 Run 标志改变时生成，最多 50 条/秒出队。时间和事件计数可按 uint32 回绕。初始状态 previous=0 表示首次观测；相同道路状态的事件表示阶段或 Run 改变。字段定义和诊断阈值见 [调试增强说明](debug-workflow.md)。

实现位于 `code/vofa_task.inc`，由 vofa.c 包含，无需新增 ADS 翻译单元。CPU0 的 vofa_snapshot 在现有 1 ms ISR 内执行固定次数诊断比较和定长快照/队列写入，无字符串格式化和动态分配。CPU0 前台 vofa_poll 取快照并格式化，状态帧为配置应答预留发送空间。16 项 SPSC 事件环使用发布/获取内存屏障；队列满增加丢弃计数并保留原有事件顺序。约 5 Hz 的同步速度/航向供上位机估算，不参与控制。

task 诊断中的视觉过期判断为 age >100 ms，Run 停帧退出为 age ≥100 ms，边界相差一拍。普通 att/run/stat 字段与 cfg 写入/停车权限保持原约定。旧 cfg v1 固件没有 task 帧时，上位机仍开放正常的兼容功能，任务页面提示等待数据。

## Remote 只读确认（0.5.0）

请求：`cfg:remote,<seq>`，无参数；应答：`rsp:<seq>,ok,remote,1,<active>,<steer_deg>,<speed_mps>,<age_ms>,<seen>`。`1` 是该扩展版本，active/seen 为 0/1，age_ms 是最后一条合法遥控命令的年龄（0–65535）。该接口不会启动 Remote；现有 hello 标识、stat 15 字段及其余命令保持兼容。

PC 使用新鲜 Remote 查询和 stat 双重确认再允许现有 speed 指令。speed 的第一项是相对当前航向角，第二项是速度原始输入（m/s ×40）；正负转向遵循 STEER_DIR。超时回零与操作步骤见 [遥控说明](experience-remote.md)。
