# ADS：specs.src duplicate action

这是 2026-09-06 构建故障的处理记录，2026-09-07 核对文档时保留。下述备份和清理属于当时操作，本次没有再次移动生成目录或启动 ADS；当前是否编译成功需另查对应构建记录。

## 原因

此前在工程内部创建了临时 headless 工作区 `.ads/station-build`。ADS 扫描到了其中的 Eclipse 编译器探测文件 `specs.c` 和 `specs.cpp`，两者被生成成同名目标 `specs.src`，使 TASKING amk 报 F126。该错误发生在构建规则解析阶段。

`.gitignore` 只控制 Git，不能阻止 ADS 扫描文件。

## 本次修复

- `.cproject` 的源码入口排除 `.ads`、`.metadata` 和 `tools`。
- `.project` 的资源过滤规则排除同名目录，防止元数据和上位机依赖再次被扫描。
- 已确认临时工作区未占用，删除 `.ads/station-build` 和仅用于该次尝试的 `station-headless.ini`。
- 旧 `Debug` 整体备份到 `.ads/build-backups/Debug-20260906-195311`，其中的旧 HEX/MAP 等保留，没有手工改写生成 makefile。
- `.ads/winIDEAWorkspaces` 保留。未修改控制源码，未启动 ADS/TASKING 构建。

## 在已经打开的 ADS 中继续

关闭工程（Close Project）后重新打开，再刷新（F5），使 ADS 重新读取磁盘上的 `.project/.cproject`。随后执行 Build Project，ADS 将重新生成 Debug 目录及构建规则。

如果工程仍显示旧过滤设置，退出并重新打开 ADS 后再构建。不要把备份目录里的旧 HEX 当作本次编译结果。

后续临时 Eclipse 工作区应建在固件工程目录之外；不要将工程内部的 `.ads` 或 `.metadata` 用作 `-data` 工作区。

验证范围：工程 XML 可解析、排除配置和备份/清理结果已检查。是否完整编译链接通过仍须以用户接下来的 ADS 构建结果为准。
