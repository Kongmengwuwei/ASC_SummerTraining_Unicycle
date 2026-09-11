# ADS 构建辅助工具

`build.py` 在工程外创建源码快照和临时 Eclipse 工作区，通过真实 ADS 图形工作台编译 Debug；不修改安装目录，不把 `.ads`、`tools` 或生成文件当作固件源码。`BuildStartup.java` 是工作台启动桥接源码。

需要 Windows、Python 和已安装且有可用 TASKING 许可的 ADS。默认安装位置来自脚本中的 `DEFAULT_ADS`；其他电脑通过 `ADS_HOME` 或 `--ads` 指定。

从仓库根目录手动执行：

```powershell
python tools/ads/build.py --ads "D:\AURIX Development Stdio\AURIX-Studio-1.10.10"
python tools/ads/build.py --clean
```

`--project` 可指定另一份工程，`--timeout` 设置等待秒数。结果索引在工程 `.ads/build/latest.json`；完整日志和 ELF/HEX/MAP 路径由索引给出。成功判定同时检查工作台结果、编译控制台、产物和源码快照一致性，旧产物存在本身不算构建成功。

## 显式集成自检

```powershell
python tools/ads/test_build.py
```

这会真正启动两次 ADS：先在临时工程注入预期编译错误，再移除错误验证恢复。仅在需要且允许 ADS 构建时手动执行；导入该文件或普通 pytest 收集不会启动它。临时工程及日志保留用于排障。

本次仓库整理只检查 Python 语法、无构建的导入行为和固件/上位机主机回归，未执行 ADS 构建、自检或烧录。历史构建记录不代表当前工作区已通过目标验证。
