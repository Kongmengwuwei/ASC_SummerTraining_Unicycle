# Embedded Station · 独轮车上位机

整理日期：2026-09-09。车端参数与模式以当前源码及 MCU Schema 为准，见 [参数参考](../../参数参考.md) 和 [源码核对](docs/source-audit.md)。

程序全部位于独轮车项目总文件夹的 `tools/unicycle_station/` 下。Windows 桌面应用，Python 3.11+、PySide6、pyqtgraph、pyserial；首个设备配置为 TC264D Q 型独轮车。

## 启动

**最新版 0.5.1**：双击本目录的 `start_app.bat`，或打开 `dist/EmbeddedStation/EmbeddedStation.exe`。无需安装 Python；复制到其他电脑时须复制整个 `EmbeddedStation` 文件夹。

本目录只保留这一份发行程序。历史源码与验证记录保存在 Git；安装包不纳入 Git，需要旧版时从对应提交重新构建。最新改动见 [版本说明](docs/live-tuning.md)，验证结果见 [验证记录](docs/validation.md)。

快捷键、折叠和排序见 [交互说明](docs/interaction-workflow.md)，Run 记录、名称自定义及总览显隐见 [个性化说明](docs/personalization-workflow.md)，任务轨迹等见 [调试说明](docs/debug-workflow.md)。串口连接问题见 [连接排查](docs/handshake-diagnosis.md)。

开发时可双击 `start_windows.bat`，然后点 **Mock 演示**。Mock 会自动循环模拟运行与停车状态，仅作用于软件演示，点击立即停车后保持模拟 STOP。无需车辆即可查看姿态、波形、74 项参数的模拟数据、记录和回放。Mock/Profile 的默认值与示例预设是快照，不是车端读回值；zero_pid 只是全零导入演示，不代表固件当前默认值。首次启动缺少依赖时会在本目录 `.venv` 中安装。

也可在本目录执行：

```powershell
.\.venv\Scripts\python.exe run_station.py --mock
```

连接实车前先打开“串口设置”，选择端口、波特率、8N1、编码、结束符和流控等。默认 115200、ASCII、CRLF、无 PC 硬件流控。可保存连接预设，最近连接和工作区自动保存。勾选“启动时连接上次设备”后才会自动连接实车。

红色 **立即停车** 始终保留在顶部，空格键也可发送。提示“已发送”表示串口写入完成；只有停车后发送的带序号状态查询成功且确认 STOP 后才提示停止。应用没有远程发车、Balance/Test/Jog 启动操作，发车仍由车身按键确认。

## 操作

- **遥控驾驶**：车身进入 Remote 后，启用本次遥控，按住 W/A/S/D、方向键或方向按钮控制前进、后退、转向；松手回零，失焦或切页自动释放。车端需更新状态查询接口，详见 [遥控与界面说明](docs/experience-remote.md)。
- **总览**：姿态、运行模式、链路、视觉、速度与参数 revision。
- **实时波形**：左侧搜索与勾选通道，选择目标图表后勾选即显示；“显示姿态”用于快速检查，“恢复实时显示”恢复跟随和自动纵轴。可添加/删除/停靠/浮动图表、添加设备预设。同一通道可显示在多个图表。右键图表进一步调整坐标，支持暂停、跟随、5–120 秒窗口、十字线、样式、别名、统计、CSV/PNG 导出。统计下方悬停可查看全部已选通道。
- **3D 姿态**：非对称长方体、车头箭头、彩色坐标轴、地面网格；鼠标旋转/缩放/平移。默认 ZYX 旋转，轴映射 JSON 可配置。先逐轴转动车体校验映射；显示归零只作用于画面。超过 700 ms 无姿态输入显示 STALE，停止运动。
- **任务与轨迹**：显示道路状态、进入/退出和环岛阶段，提供轮速与航向积分的相对轨迹；缺失数据明确分段。
- **试验与对比**：自动运行片段、参数快照、A/B 波形与参数差异、故障前后日志截取。
- **参数调节**：先等待完整 Schema，同步后按组搜索。当前 MCU 值与待应用值分列。单项/组应用会预览差异并逐项等待确认，失败立即中止。支持收藏、回退、微调、JSON 预设。未知新增参数显示在未分类组。
- **Flash**：仅 MCU 确认 STOP 时开放按钮，MCU 还会检查轮速稳定和待处理操作。保存只在回读验证应答后标记成功。危险参数需停车、高级模式解锁和二次确认。
- **调参工作台**：Roll/Pitch/Yaw、速度/转向及压弯参数与相关波形同屏显示。支持 Run Test 波形及运行中在线调参，见 [操作说明](docs/live-tuning.md)。当前固件没有独立 Pitch 输出完整遥测，页面只展示确实存在的姿态/速度，不伪造缺失通道。
- **日志与回放**：默认连接即开始记录，保证车身发车前已有数据。运行中请求停止记录会延迟到停车后至少 2 秒；主动断开或关闭应用会结束会话。打开 `frames.jsonl`，播放/暂停/倍速/拖动，双击事件跳转，导出区间。图表暂停不停止后台记录。
- **通信诊断**：原始行、解析错误、参数应答和故障记录。未知行不会使程序崩溃。
- **设置**：日志目录、自动记录、Mock 异常注入、派生通道。工作区菜单可新建、打开、保存、另存、导入/导出、恢复默认布局和切换设备 Profile。

会话默认保存在用户文档目录 `EmbeddedStation/sessions/`；可在设置页改为本项目的 `sessions/`。每次会话含原始字节 `raw.txt`、逐行可恢复的 `frames.jsonl`、`session.json` 元数据。**原始 run 行会保留**。

## 测试与打包

```powershell
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m app.main --smoke
.\.venv\Scripts\python.exe scripts/endurance.py
```

`--smoke` 自动打开 Mock 并逐页截图到 `artifacts/screenshots/`；endurance 默认进行 30 分钟真实时间收数、记录和内存测试，结果在 `artifacts/endurance.json`。

双击 `build_windows.bat`：先安装项目依赖、运行测试，再以 PyInstaller 生成 `dist/EmbeddedStation/EmbeddedStation.exe`。分发时复制整个 `dist/EmbeddedStation` 文件夹，不能只取其中 exe。`requirements-lock.txt` 记录本次 Windows / Python 3.13 验证所用版本。

测试截图、构建日志和 Mock 测试会话统一放在 Git 忽略的 `artifacts/` 下，可在测试结束后删除；实际采集会话仍使用设置页配置的日志目录。

## 首次实车联调

1. 固定车体于台架，确保现场可以立即按返回键或断电。不要从上位机改初始 PID 或极性来“试启动”。
2. 在 ADS/TASKING 中构建本次修改源码并核对新产物，按既有烧录流程操作。PC 测试通过不代表固件构建或车辆稳定性已验证。
3. 连接 UART2，仅检查 hello、stat、Schema 与车身菜单的实际参数一致性。
4. 在停车状态验证立即停车的发送与 MCU 状态确认。逐轴手动检查 3D 映射。
5. 先以一个普通非危险参数验证 RAM 实际值、回退、Flash 保存和重新上电读取；最后才由人工在台架上操作车身测试入口。

详细协议见 [protocol.md](docs/protocol.md)，通道见 [channels.md](docs/channels.md)，架构见 [architecture.md](docs/architecture.md)，验证与限制见 [validation.md](docs/validation.md)。示例位于 `examples/`。
