# 项目交接本 (PROGRESS.md)

> **最高规则**：本项目是 UAV-GPR 无人机探地雷达系统（地面站 Qt GUI + 空中端黑匣子采集服务）。
> 每一棒 AI 在修改代码前**必须**先通读本文件；完成任务后**必须**更新本文件并与代码一同提交推送。

---

- **当前版本**：v8.0.0-round8
- **最后更新时间**：2026-10-15
- **所属仓库**：uav-gpr / UVA_GPR_system (Private)

---

## 1. 项目架构与核心职责

- **空中端（Air Station）**：
  - `uav_gpr_air_service.py`：常驻后台采集服务（黑匣子），驱动 LibreVNA 与 GNSS 进行回波数据采集与本地落盘（disk outbox）。计算棒上只运行它。
  - `air_core.py`：空中端核心零件（采集 worker、AcquisitionService、GNSS、校准接线、处理流水线、RemoteSender 等），**无界面**，只依赖 `PySide6.QtCore`。黑匣子服务 `import air_core as air`。
  - `uav_gpr_qt.py`：**仅兼容别名**（`sys.modules` 指向 `air_core`，同一个模块对象），给旧脚本用；新代码一律 `import air_core`。空中端旧 GUI（MainWindow 等）已于 2026-10-15 退役删除。
  - `run_air_station.bat`：改为以本机模式启动黑匣子服务（`--server 127.0.0.1:8765`），配合本机地面站做桌面联调。
  - `air_gnss.py`：空中端 GNSS 驱动与串口解析。
- **地面端（Ground Station）**：
  - `uav_gpr_ground_station.py`：PySide6 / Qt 界面，提供 B-scan 实时成像、设备控制、校准向导等。
  - `ground_gnss.py`、`calibration_service.py`：地面差分定位与 OSL/SOLT 标定校准核心逻辑。
- **通信核心不变量**：
  - 空地采用带内 TCP 通信与滑动窗口重传机制。
  - 地面 `ReceiverThread` 严格串行单槽，**严禁向空中端随意引入未定义的消息 type**，新增控制指令一律走 command + action。

---

## 2. 怎么把项目跑起来与验证（环境说明书）

- **Python 运行时**：必须使用配套的 Python 3.12 虚拟环境：
  - 可执行文件路径：`.venv312\Scripts\python.exe`
- **全量测试回归命令（至关重要）**：
  ```powershell
  & .venv312\Scripts\python.exe docs\run_tests_batched.py
  ```
- **核心测试避坑指南（必须遵守）**：
  1. **严禁**使用单进程 `python -m unittest discover` 跑全量：Qt 6.11.2 下 WebEngine 在单进程连续运行第 72 个用例左右会抛出 `CalledOnValidBrowserThread` 崩溃，这是 Qt 引擎机制，必须走 `run_tests_batched.py` 的独立子进程分批执行。
  2. **判绿依据**：测试通过与否以子进程输出的 `^OK` 为准。部分 Qt 进程在 teardown 时可能以 0xC0000005 退出，但只要测试输出全 OK 即代表通过，切勿仅凭进程退出码误报回归。
  3. **测试规模基线**：重构前 42 模块 1191 项；2026-10-15 退役空中端旧 GUI 后删除约 84 项纯旧窗口用例、新增 `tests/test_air_core_headless.py` 7 项守卫，现为 41 模块约 1100 项（以 Windows 实跑为准）。

---

## 3. 已完成（前8轮成果，不要推翻）

- [x] Round 1~5：实时流水线性能优化、常驻采集服务、磁盘发件箱（disk outbox）、UI3 界面重构、滑动窗口重传。
- [x] Round 6：B-scan 显存与显示窗口扩展（64 -> 256）及空中端远程连接体验优化。
- [x] Round 7：空中端黑匣子无头服务（headless service）与计算棒远程部署。
- [x] Round 8：标定校准与背景采集修复（OSL 激活会话状态隔离、v2 校准文件传输适配、P1 重采状态机重置，42 模块 1191 项全绿）。

- [x] 2026-10-15 架构技术债第 1 步（分支 `refactor/air-core-retire-air-gui`）：空中端旧 GUI（MainWindow、BScanWidget、AirSettingsDialog、`_apply_trace_to_ui`、`_ServiceCalibrationCapturerProxy`、`main`）删除；核心原样改名 `air_core.py`（7207 → 约 4700 行，逻辑零改动）；黑匣子服务/air_gnss/field_test 改 import `air_core`；黑匣子 import 时不再加载 QtWidgets/QtGui/WebEngine/pyqtgraph。
  - 测试迁移规则：依赖空中端窗口的用例删除（其行为由黑匣子服务测试 round7/round8 覆盖）；混在其中的核心或地面端断言已拆出保留。
  - 已发现的既有差异（未改，待决定）：旧 GUI 在“校准会话进行中”会拒绝 `set_params`，黑匣子服务 `_cmd_set_params` 只检查“采集运行中”。
  - `tools/build_round8_portable.py` 是第 8 轮冻结哈希的历史打包脚本，未改；下一次发版的打包脚本需把 `air_core.py` 和 `uav_gpr_qt.py`（别名）一起放进两个包，并更新地面包说明里“run_air_station.bat 启动带画图界面的空中程序”那段。
  - Linux 沙箱上 `test_round1_third_fixes.test_concurrent_stop_and_put_race` 在 main 上也会偶发失败（计时竞争），非本次引入。

---

## 4. 下一步任务建议

0. 架构技术债第 2 步：拆分 `air_core.py`（按采集/GNSS/校准/发送分文件，注意测试大量 `patch.object(air, ...)`，拆分时要保证被 patch 的名字仍从同一模块查找）；第 3 步：拆分 `uav_gpr_ground_station.py`；补 CI。

1. 保持当前架构稳定性，新增需求先编写测试用例或在对应模块添加独立单元测试。
2. 任何涉及空地通信的改动，务必运行 `docs\run_tests_batched.py` 验证 1191 项测试全部通过。
