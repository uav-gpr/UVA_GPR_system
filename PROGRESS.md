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
  - `uav_gpr_air_service.py`：常驻后台采集服务，驱动 LibreVNA 与 GNSS 进行回波数据采集与本地落盘（disk outbox）。
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
  3. **测试规模基线**：当前基线为 43 个模块共 1195 项测试全绿（2026-10-15 新增色阶滑块测试 4 项），以及 30 项独立黑匣子端到端 E2E 测试。

---

## 3. 已完成（前8轮成果，不要推翻）

- [x] Round 1~5：实时流水线性能优化、常驻采集服务、磁盘发件箱（disk outbox）、UI3 界面重构、滑动窗口重传。
- [x] Round 6：B-scan 显存与显示窗口扩展（64 -> 256）及空中端远程连接体验优化。
- [x] Round 7：空中端黑匣子无头服务（headless service）与计算棒远程部署。
- [x] Round 8：标定校准与背景采集修复（OSL 激活会话状态隔离、v2 校准文件传输适配、P1 重采状态机重置，42 模块 1191 项全绿）。
- [x] 2026-10-15 小功能：地面站 B-scan 右侧新增竖向「色阶」滑块（`bscan_contrast_slider`，1–100%，默认 100% = 原自动色阶行为），按百分比收窄 colorbar 显示范围以调整颜色深浅；S11/S22 各按自身 max|x| 缩放；拖动只调 `ImageItem.setLevels`，不重算矩阵；「重置」为 QToolButton（主窗口 QPushButton 仍只有开始/停止/暂停/继续）。`bscan_area` 改为 QGridLayout（第 0 列 HH/VV/历史滑块，第 1 列色阶滑块），行序与原契约一致。新增 `tests/test_bscan_contrast_slider.py`（4 项），测试基线变为 43 模块 1195 项。

---

## 4. 下一步任务建议

1. 保持当前架构稳定性，新增需求先编写测试用例或在对应模块添加独立单元测试。
2. 任何涉及空地通信的改动，务必运行 `docs\run_tests_batched.py` 验证 1191 项测试全部通过。
