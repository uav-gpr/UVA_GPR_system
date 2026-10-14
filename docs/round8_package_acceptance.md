# 第8轮 成品包独立复验报告（t8 · 打包后解压复验）

**执行角色**: verifier（独立复验，与打包者 t7 分离）
**日期**: 2026-10-14
**复验对象**: t7 交付的两个最终 ZIP（从头独立解压，非 staging 复用）
**结论**: **两个 ZIP 均通过独立复验，可交付用户。** 双进程真实拓扑（地面包 runtime ↔ 空中包 runtime over TCP）完成三故障全链路、v2 导入/导出/重启恢复、重采/错误恢复与真实按钮点击；全部数值/状态/哈希证据见下。真机精度边界见 §7。

---

## 1. ZIP 与解压（从头、独立）

| ZIP | SHA-256（复验实测） | t7 记录值 | 一致 |
|---|---|---|---|
| 地面站便携包_20261013_校准修复.zip (292,245,488 B) | `E9C93D68635310B7B27789EF3832370FBDAE0667C18F864D61A7E91AA811D0F2` | 同 | ✓ |
| 空中端黑匣子便携包_20261013_校准修复.zip (292,180,158 B) | `CA5D6841A4C9E8AE7A0350EA514CA9B2E51C81BBCB0B4161DE4EE8FBAA739827` | 同 | ✓ |

解压（Expand-Archive 全新目录，验证结束后复核 ZIP 哈希**不变**）：
- `dist\releases\20261013_calibration_fix\verification_tmp\ground\地面站便携包\`
- `dist\releases\20261013_calibration_fix\verification_tmp\air\空中端黑匣子便携包\`

两包均含 `runtime\`（嵌入式 python312 + python.exe/pythonw.exe）、`site-packages\`（PySide6/numpy/serial/matplotlib 等 45 项）、`librevna\`、应用源码、`RELEASE_MANIFEST.txt` 与启动器。**包内无 logs/gpr_runs/outbox/remote_runs/__pycache__ 等测试数据或运行残渣（解压后即时核查：CLEAN）**。

---

## 2. 运行时隔离证明（无开发 venv/PYTHONPATH 偷渡）

命令（清除 `PYTHONPATH` 环境变量后，以各包 `runtime\python.exe` 执行探针）：

```powershell
Remove-Item Env:PYTHONPATH -ErrorAction SilentlyContinue
& "<pkg>\runtime\python.exe" probe.py   # 打印 sys.path / PYTHONPATH / uav_gpr_qt.__file__
```

实测（两包一致）：
- `PYTHONPATH_env=None`（嵌入式 `python312._pth` 生效，忽略外部 PYTHONPATH）；
- `sys.path` 仅含：`runtime\python312.zip`、`runtime\`、`site-packages\`、包根目录——**全部位于 verification_tmp 解压目录内**；
- `uav_gpr_qt.__file__ = …\verification_tmp\ground\地面站便携包\uav_gpr_qt.py`（air 同理）→ `__file__` 实际来自解压包，`under_verification_tmp=True`。
- 启动器与真实启动方式一致：`run_ground_station.bat`/`run_air_service_console.bat` 均为 `runtime\python(exw).exe <pkg>\<script>`，无其他路径来源。

---

## 3. 双进程真实拓扑 E2E（成品包 runtime ↔ 成品包 runtime over TCP）

**拓扑**：地面进程 = 地面包 runtime + 解压包 `uav_gpr_ground_station.py`（真实窗口/ReceiverThread/CalibrationWizard/真实 Qt 按钮，offscreen）；空中进程 = 空中包 runtime + 解压包 `uav_gpr_air_service.py main`（`--config` 指向**临时模拟配置**：`device_source=simulated`、`autostart_acquisition=false`、`ground_port=51777`、`output_dir`/日志均指向验证临时区）。驱动脚本 `verification_tmp\package_e2e_driver.py`（标准 unittest，Phase A 4 用例 + Phase B 1 用例，复用 t11 E2E 逻辑）。

### 3.1 Phase A（三故障正向主链 + 按钮 + 错误恢复）——`Ran 4 tests, OK`（exit 0，11.5s）

| 步骤（全部真实按钮/真实 wire） | 实证 |
|---|---|
| `begin_button.click()` → OSL 六步（`capture_button.click()` × 6，每步等待空中真实 `step_done`）→ `finish_button.click()` → `activate_button.click()` | 六步全完成；激活后 `_cal_id=None`、`begin_button`/`mode_combo` 解锁（**故障一修复**）；`_active_calibration` 非空 |
| 模式切背景 → 再 `begin/capture/finish/activate` | 背景会话真实完成，OSL+背景双生效展示 |
| `receiver.send_command("start", {"single": True})`（真实采集命令入口） | 地面收到真实 trace NPZ：201 点、`display_S11` 与 `raw_S11` 全有限且**显著可区分（校正确实生效）**、`acquisition.calibration_id` 非空 |
| P1_Open 完成 → `recapture_button.click()`（真实重采按钮）| 重采完成、下一步指针 = **P1_Short**（**故障三修复**） |
| 在途重复 `cal_capture`（真实 wire） | 空中真实拒绝（ack ok=false 且按请求回带 `request_id=pkg_dup_001`）；本会话目录 `capture_P1_Short_*.npz` 恰为 2 条（**无重复执行**）；行/指针无残留（GND-09 实证） |
| **真实 `import_button.click()`** + 测试侧 QFileDialog 桩选择真实临时 v2（`pkg_user_v2.json`，内容=原件副本） | 文件框触发；空中校验/落盘/激活；地面 active label `P1+P2`/`201点`（**故障二修复**） |
| **真实 `export_button.click()`** + 保存框桩 | 导出文件与导入文件**逐字段等价** |
| **真实 `query_button.click()`** | 导入身份/端口/点数如实呈现 |

### 3.2 空中硬重启 + Phase B（恢复）——`Ran 1 tests, OK`（exit 0，2.7s）

- 编排器 `Stop-Process -Force` 硬杀空中进程 → 以同一 `--config` 重新启动解压包空中服务 → 空中日志：**`自启动恢复生效校准: pkg_user_v2.json`**（v8.0.0-round8，pid 记录在案）。
- 地面**新进程**（重新绑定同端口）等空中真实重连（5s→30s 退避）→ **真实 `query_button.click()`** → 导入身份 `pkg_user_v2.json`/`["P1","P2"]`/`201` 如实呈现 → 真实 `start` → 收到**校准身份匹配（`pkg_user_v2`）**的真实 trace → display 与 RAW 可区分（重启后校正仍生效）→ 真实 `stop` + get_status 确认回 idle_waiting。

---

## 4. 版本/清单/纯净性核验

- `RELEASE_MANIFEST.txt`（两包）：版本标识 `8.0.0-round8`；质量门禁记录 "Round 8 全量回归通过 (42 模块 1191 测试 PASS, E2E 30 测试 PASS)"——与 t11 归档一致。
- **包内核心文件 SHA-256 与 t11 冻结值逐字一致**（实测）：`uav_gpr_ground_station.py=46E00701…`、`uav_gpr_qt.py=EEF2BE69…`（两包同名同哈希）、`uav_gpr_air_service.py=F10F6163…`、`calibration_service.py=47FD0D4C…`、`librevna/librevna_usb_s11.py=F023E805…`、`solt_2port_20260807_202342.json=14276D50…`。
- 空中包 `air_config.json` 原始内容核验：`device_source="usb"`、`gnss_baud=9600`、`autostart_acquisition=false`、`calibration_path=""`——**发布默认纯净**（USB 真机、不自动采集、无预置校准）。本次复验的模拟配置写在验证临时区，未触碰包内生产配置。
- 运行期产物（`logs\air_service.log` 等）仅产生于 verification_tmp 解压目录（与真实部署行为一致），ZIP 本体哈希复核不变。

---

## 5. 升级/MIR 保护核验（只读审查，未执行）

`升级_替换程序.bat`（空中包内）关键机制逐条核验：
- **非 purge 同步**：`robocopy /E`（**无 `/MIR`**，绝不删除目标文件）；
- **保留用户配置**：`/XF air_config.json`；
- **保留数据与持久化校准目录**：`/XD logs gpr_runs remote_runs _calibration calibrations __pycache__`——**MIR 不保护的新持久化校准目录（`_calibration/`）被显式排除**；
- 升级前完整备份到时间戳目录，失败即止且不改目标；
- 定向结束仅旧目录路径下的 `pythonw` 进程。
`安装到计算棒.bat`（计划任务安装器）为**交付给用户的可选安装器**，本轮**未执行**（禁安装/自启要求）；其内容经只读审查不含对本机开发环境的任何引用。

---

## 6. 完整性复核（未改任何资产）

- 真实校准原件 `solt_2port_20260807_202342.json`：`14276D50…D82CB5`（与 t11 一致，未变）。
- 旧包目录 `dist\地面站便携包`/`dist\空中端黑匣子便携包`：完好未动（mtime 2026-10-12/14/15，早于本轮全部工作）。
- 开发冻结源码四个文件哈希复测：与 t11 逐字一致。
- 两个最终 ZIP：复验结束后哈希**复核不变**。

---

## 7. 边界（诚实声明）

- **已验证**：成品包 runtime/依赖/启动器完整性、隔离性、双进程真实拓扑下的三故障正向链、v2 导入/导出/重启恢复、重采与关键错误恢复、真实 Qt 按钮（import/export/query，点击实例而非 handler 直调）、版本/清单/哈希一致性、发布默认纯净、升级脚本保护机制（只读审查）。
- **模拟验证边界**：采集数据来自包内模拟设备（协议线一致、物理场景模型），**不能证明真实 VNA/标准件测量精度、真实 USB 时序、真实 GNSS/串口**。真机复核清单（t4 报告 §6.2/使用说明）随包交付，现场按清单执行。
- **未做**：未运行安装器/升级脚本/自启任务；未触碰用户开发目录与真实原件；未修改任何包内文件。
- 运行期产物（air 日志、runs、验证驱动）全部位于 `verification_tmp\` 验证临时区与 `work\` 目录，不影响交付 ZIP。

---

## 8. 交付判定

两个 ZIP 通过全部独立复验项，**可交付用户**。建议交付物：两个 ZIP + `docs/round8_release_notes.md` + 真机复核清单；包内 `使用说明*.txt` 与 `RELEASE_MANIFEST.txt` 已随包自带。
