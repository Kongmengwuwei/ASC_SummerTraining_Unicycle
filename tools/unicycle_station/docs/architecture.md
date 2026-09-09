# 架构与扩展

源码核对：2026-09-07。当前固件标识 tc264-cfg1-task1，配置协议版本仍为 1。

```text
串口 / Mock → device-io 线程：增量解析 + 请求状态机 → 固定容量 DataStore
                        └→ 有界日志队列 → session-writer 线程 → raw.txt / frames.jsonl
GUI 25 Hz ← 快照 / 环形缓存
GUI 停车按钮 → 独立 priority-stop 线程 → 有界串口写锁 → stop
历史会话 → ReplayDataSource → 相同解析器 / DataStore / 图表
```

`core/models.py` 定义 Transport、ProtocolParser、DataSource、DeviceProfile、VisualizationPlugin 接口。实际实现为 SerialDataSource、MockDataSource、ReplayDataSource、FireWaterParser。主窗口只负责组合页面及工作区，不负责串口读写与协议解析。

配置、模型、通道、参数说明和默认曲线均放在 `app/profiles/<device>/profile.json`。替换 Profile 会关闭现有数据源并重建页面、清空缓存和参数；通用传感器没有独轮车姿态、参数和调参工作台。未知数值 tag 可自动发现，单设备最多 256 个通道、每通道 6500 个样本，最多 8 个图表、每图最多 32 条曲线。错误、原始预览和事件也有容量上限。

新增普通数值设备优先添加 Profile；新协议通过 `plugins/registry.py` 显式注册。Profile JSON 不允许导入任意模块。VisualizationPlugin 可返回 Qt Widget，并实现状态保存恢复；内置姿态组件提供相同接口。TCP/UDP/CAN/HID/Bluetooth/图像流仅预留接口，本次不包含实现或多设备调度。

参数由 MCU Schema 实时生成；Profile 可补充中文名、默认值、单位、描述、步长、小数位、条件显示/使能和枚举。MCU 当前只产生 float/int，通用模型和控件兼容 bool/enum/string；FireWater v1 的 set 仍是数值协议，字符串写入需要对应设备协议插件扩展。

工作区、Profile、参数预设、连接预设和会话元数据分别使用带 `kind` 和 `schema_version=1` 的 JSON。未知版本明确拒绝，避免静默套用不兼容设置。当前仅有 v1，尚无旧版本迁移需求。

图表时间轴为 PC 单调接收时间，原始日志另保留 MCU uptime。回放使用磁盘顺序读取和每 5 秒一个索引，避免将完整会话载入内存；事件索引最多 10000 项。快速拖动后重置图表缓存，从目标位置继续播放；不将播放空隙补成真实样本。

参考实现接口：[Qt QOpenGLWidget](https://doc.qt.io/qtforpython-6/PySide6/QtOpenGLWidgets/QOpenGLWidget.html)、[pyqtgraph PlotDataItem](https://pyqtgraph.readthedocs.io/en/latest/api_reference/graphicsItems/plotdataitem.html)。

## task1 与调试增强模块

- 车端 code/vofa_task.inc 由 vofa.c 包含，复用 CPU0 1 ms 快照调用；只观察 CPU0 已接收的视觉与控制状态。前台格式化 task/taskevt，不增加独立控制分支或 CPU1 电机责任。
- app/services/task_state.py 管理道路状态/事件与相对轨迹；app/ui/task_page.py 展示。使用同步的 MCU 时间、航向和轮速积分，数据缺口分段，不能当作真实赛道地图。
- app/services/experiments.py 和 app/ui/experiments_page.py 处理试验片段、参数快照、对比和故障截取；app/services/background.py 承担相应后台工作。档案和日志不进入车端控制 ISR。
- app/plotting/measurement.py 与 scope.py 提供测量及联动波形；方向开关只影响 PC 显示/估算，不改车端零点或极性。

当前文档导航以 0.2 的十页面结构为准；旧初版八页面截图/测试记录保留在 validation.md 的历史部分。

## 与固件参数保持一致

g_param_table 和车端 Schema 决定名称、类型、范围、当前值及权限；Profile/示例/Mock 是独立文件快照。默认值及菜单步长的源码索引见 [参数参考](../../../参数参考.md)。scripts/generate_profiles.py 会重写 Profile 和部分示例，应仅在需要更新这些文件时执行，并审查差异。

改参数时应同步 board_config、param_load_defaults、menu 分组与保存列表、CPU1 反馈、cfg_group/权限及 PC 元数据。当前 direction_rate_kd 对应结构字段 direction_balance_kd，lean_roll_kp 对应 direction_roll_kp，不能简单假设协议名等于结构成员名。