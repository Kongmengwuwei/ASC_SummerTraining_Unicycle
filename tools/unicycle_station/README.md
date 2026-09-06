# Embedded Station · 独轮车上位机

程序全部位于独轮车项目总文件夹的 `tools/unicycle_station/` 下。Windows 桌面应用，Python 3.11+、PySide6、pyqtgraph、pyserial；首个设备配置为 TC264D Q 型独轮车。

## 启动

本机已生成 Windows 发行包，可直接打开 `dist/EmbeddedStation/EmbeddedStation.exe`，无需另外安装 Python。首次使用点 **Mock 演示**。复制到其他电脑时请复制整个 `EmbeddedStation` 文件夹。

双击 `start_windows.bat`，然后点 **Mock 演示**。Mock 会自动循环模拟运行与停车状态，仅作用于软件演示，点击立即停车后保持模拟 STOP。无需车辆即可查看姿态、波形、72 项固件参数、记录和回放。首次启动缺少依赖时会在本目录 `.venv` 中安装。

也可在本目录执行：

```powershell
.\.venv\Scripts\python.exe run_station.py --mock
```

连接实车前先打开“串口设置”，选择端口、波特率、8N1、编码、结束符和流控等。默认 115200、ASCII、CRLF、无 PC 硬件流控。可保存连接预设，最近连接和工作区自动保存。勾选“启动时连接上次设备”后才会自动连接实车。

红色 **立即停车** 始终保留在顶部，空格键也可发送。提示“已发送”表示串口写入完成；只有停车后发送的带序号状态查询成功且确认 STOP 后才提示停止。应用没有远程发车、Balance/Test/Jog 启动操作，发车仍由车身按键确认。

## 操作

- **总览**：姿态、运行模式、链路、视觉、速度与参数 revision。
- **实时波形**：左侧搜索与勾选通道，选择目标图表后“选择应用到图表”。可添加/删除/停靠/浮动图表、添加设备预设。同一通道可显示在多个图表。右键图表进一步调整坐标，支持暂停、跟随、5–120 秒窗口、十字线、样式、别名、统计、CSV/PNG 导出。统计下方悬停可查看全部已选通道。
- **3D 姿态**：非对称长方体、车头箭头、彩色坐标轴、地面网格；鼠标旋转/缩放/平移。默认 ZYX 旋转，轴映射 JSON 可配置。先逐轴转动车体校验映射；显示归零只作用于画面。超过 700 ms 无姿态输入显示 STALE，停止运动。
- **参数调节**：先等待完整 Schema，同步后按组搜索。当前 MCU 值与待应用值分列。单项/组应用会预览差异并逐项等待确认，失败立即中止。支持收藏、回退、微调、JSON 预设。未知新增参数显示在未分类组。
- **Flash**：仅 MCU 确认 STOP 时开放按钮，MCU 还会检查轮速稳定和待处理操作。保存只在回读验证应答后标记成功。危险参数需停车、高级模式解锁和二次确认。
- **调参工作台**：Roll/Pitch/Yaw 的 Schema 参数与相关数值曲线。当前固件没有独立 Pitch 输出完整遥测，页面只展示确实存在的姿态/速度，不伪造缺失通道。
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

`--smoke` 自动打开 Mock 并逐页截图到 `docs/screenshots/`；endurance 默认进行 30 分钟真实时间收数、记录和内存测试，结果在 `docs/endurance.json`。

双击 `build_windows.bat`：先安装项目依赖、运行测试，再以 PyInstaller 生成 `dist/EmbeddedStation/EmbeddedStation.exe`。分发时复制整个 `dist/EmbeddedStation` 文件夹，不能只取其中 exe。`requirements-lock.txt` 记录本次 Windows / Python 3.13 验证所用版本。

## 首次实车联调

1. 固定车体于台架，确保现场可以立即按返回键或断电。不要从上位机改初始 PID 或极性来“试启动”。
2. 在 ADS/TASKING 中构建本次修改源码并核对新产物，按既有烧录流程操作。PC 测试通过不代表固件构建或车辆稳定性已验证。
3. 连接 UART2，仅检查 hello、stat、Schema 与车身菜单的实际参数一致性。
4. 在停车状态验证立即停车的发送与 MCU 状态确认。逐轴手动检查 3D 映射。
5. 先以一个普通非危险参数验证 RAM 实际值、回退、Flash 保存和重新上电读取；最后才由人工在台架上操作车身测试入口。

详细协议见 [protocol.md](docs/protocol.md)，通道见 [channels.md](docs/channels.md)，架构见 [architecture.md](docs/architecture.md)，验证与限制见 [validation.md](docs/validation.md)。示例位于 `examples/`。
