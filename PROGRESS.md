# 项目交接本 (PROGRESS.md)

> **最高规则**：本项目是 UAV-GPR 无人机探地雷达系统（地面站 Qt GUI + 空中端黑匣子采集服务）。
> 每一棒 AI 在修改代码前**必须**先通读本文件；完成任务后**必须**更新本文件并与代码一同提交推送。

---

- **当前版本**：v8.0.0-round8
- **最后更新时间**：2026-11-12
- **所属仓库**：uav-gpr / UVA_GPR_system (Public)

---

## 1. 项目架构与核心职责

- **空中端（Air Station）**：
  - `uav_gpr_air_service.py`：常驻后台采集服务（黑匣子），驱动 LibreVNA 与 GNSS 进行回波数据采集与本地落盘（disk outbox）。计算棒上只运行它。
  - `air_core.py`：空中端核心零件（采集 worker、AcquisitionService、GNSS、校准接线、处理流水线、RemoteSender 等），**无界面**，只依赖 `PySide6.QtCore`。黑匣子服务 `import air_core as air`。
  - `uav_gpr_qt.py`：**仅兼容别名**（`sys.modules` 指向 `air_core`，同一个模块对象），给旧脚本用；新代码一律 `import air_core`。空中端旧 GUI（MainWindow 等）已于 2026-10-15 退役删除。
  - `run_air_station.bat`：改为以本机模式启动黑匣子服务（`--server 127.0.0.1:8765`），配合本机地面站做桌面联调。
  - `air_gnss.py`、`gnss_diagnostics.py`：空中端常驻 GNSS 驱动、串口解析、固定解分类/筛选策略与可选滚动 JSONL 诊断日志。
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
  3. **测试规模基线**：重构前 42 模块 1191 项；2026-10-15 退役空中端旧 GUI 后删除约 84 项纯旧窗口用例、新增 `tests/test_air_core_headless.py` 7 项守卫，再加校准失效、单通道、色阶滑块、整次采集合并、GNSS 专项与校准交互修复等新测试，当前 Windows 分批全量回归基线为 **47 模块 1206 项**（含 30 项黑匣子端到端 E2E 测试）。

---

## 3. 已完成（前8轮成果，不要推翻）

- [x] Round 1~5：实时流水线性能优化、常驻采集服务、磁盘发件箱（disk outbox）、UI3 界面重构、滑动窗口重传。
- [x] Round 6：B-scan 显存与显示窗口扩展（64 -> 256）及空中端远程连接体验优化。
- [x] Round 7：空中端黑匣子无头服务（headless service）与计算棒远程部署。
- [x] Round 8：标定校准与背景采集修复（OSL 激活会话状态隔离、v2 校准文件传输适配、P1 重采状态机重置，42 模块 1191 项全绿）。
- [x] 2026-10-15 小功能：地面站 B-scan 右侧新增竖向「色阶」滑块（`bscan_contrast_slider`，1–100%，默认 100% = 原自动色阶行为），按百分比收窄 colorbar 显示范围以调整颜色深浅；S11/S22 各按自身 max|x| 缩放；拖动只调 `ImageItem.setLevels`，不重算矩阵；「重置」为 QToolButton（主窗口 QPushButton 仍只有开始/停止/暂停/继续）。`bscan_area` 改为 QGridLayout（第 0 列 HH/VV/历史滑块，第 1 列色阶滑块），行序与原契约一致。新增 `tests/test_bscan_contrast_slider.py`（4 项），测试基线变为 43 模块 1195 项。

- [x] 2026-10-15 架构技术债第 1 步（分支 `refactor/air-core-retire-air-gui`）：空中端旧 GUI（MainWindow、BScanWidget、AirSettingsDialog、`_apply_trace_to_ui`、`_ServiceCalibrationCapturerProxy`、`main`）删除；核心原样改名 `air_core.py`（7207 → 约 4700 行，逻辑零改动）；黑匣子服务/air_gnss/field_test 改 import `air_core`；黑匣子 import 时不再加载 QtWidgets/QtGui/WebEngine/pyqtgraph。
  - 测试迁移规则：依赖空中端窗口的用例删除（其行为由黑匣子服务测试 round7/round8 覆盖）；混在其中的核心或地面端断言已拆出保留。
  - 已发现的既有差异：旧 GUI 在“校准会话进行中”会拒绝 `set_params`，黑匣子服务 `_cmd_set_params` 只检查“采集运行中”。→ 已在下一条（校准失效机制）补上。
  - `tools/build_round8_portable.py` 是第 8 轮冻结哈希的历史打包脚本，未改；下一次发版的打包脚本需把 `air_core.py` 和 `uav_gpr_qt.py`（别名）一起放进两个包，并更新地面包说明里“run_air_station.bat 启动带画图界面的空中程序”那段。
  - Linux 沙箱上 `test_round1_third_fixes.test_concurrent_stop_and_put_race` 在 main 上也会偶发失败（计时竞争），非本次引入。
- [x] 2026-10-15 校准失效机制（分支 `feature/calibration-stale-on-param-change`，叠在上一条分支之上）：用户确认规则——校准完成后改扫频参数要提醒重新校准。
  - 新模块 `calibration_guard.py`（纯 Python）：比较当前扫频与生效校准自身的频率轴（OSL P1/P2、Through、背景各自频点）。
  - **频率范围或频点数不一致 → 失效**：黑匣子采集时停用这份校准（数据 `calibration_id` 为空 = 未校准），日志写明原因；OSL 失效时背景一并失效；背景单独不一致只停用背景。生效校准本身不删除，参数改回原样自动恢复。
  - **只改发射功率/中频带宽 → 只提醒**，校准照常使用（参照校准当时记下的扫频参数：会话快照 `config_snapshot`、导入/重启恢复读文件 `sweep_config`，没有就不提醒）。
  - 校准会话进行中（planned/in_progress/solved）黑匣子拒绝 `set_params`。
  - 回执与状态：`set_params`、`start/single`、`get_status` 结果带 `calibration_fit`（level=ok/warning/stale、message、reasons、warnings）。没有新增消息 type。
  - 地面站：操作栏右侧新增 `calibration_fit_label`（失效红底“校准已失效，请重新校准”，提醒橙底；正常隐藏），**不进状态栏**（状态栏冻结 3 个标签）；`set_params` 被拒时采集状态标签显示原因；打开校准对话框时把校准频率范围/频点数预填为当前采集设置。
  - 注意：导入的校准文件若频率轴与当前扫频不同，导入后会立刻显示“失效”并停用（以前是插值后静默使用）。
  - 测试：`tests/test_calibration_guard.py`（22 项）。
- [x] 2026-10-15 单通道采集 + VV 背景扣除（分支 `feature/single-channel-vv-background`，叠在校准失效分支之上）。
  - **通道选择**：`set_params` 新增 `channels`（`dual` / `S11` / `S22`），地面设置中心“采集控制”页新增“采集通道”下拉（`acquisition_channels_input`）。黑匣子把所选通道经 `AcquisitionTaskSpec.channels` → `ContinuousAcquisitionWorker` → `ProcessingConsumer`，只处理/保存/发送所选通道（NPZ 与遥测 `channels` 声明只含该通道）。legacy 引擎选单通道时拒绝启动。
  - **第一阶段限制（重要）**：雷达仍用已在真机验证的双反射扫法 `0x1241`，只是丢弃没选的那一路，所以**扫描速度不变**，省的是处理、存储和链路带宽。真正只扫一个端口（提速）需要只采 S11 用 `0x1240`（已验证但连续采集后端未支持）、只采 S22 需新组合（推算为 `0x1208`，**未验证**，`ALLOWED_STAGES_BITMAPS` 禁止擅自扩展），必须先真机验证。
  - **地面显示**：解码接受显式声明的单通道 payload；没采的那幅 B-scan 补零列（两幅道数保持同步），标题注明“（本次未采集）”，清空 B-scan 后恢复。HH/VV 两幅图与布局冻结约定不变。
  - **单端口校准**：`build_osl_plan(..., ports=P1|P2|both)`，`cal_begin` 带 `ports`；校准对话框新增“OSL（只校 P1 / HH，三步）”“OSL（只校 P2 / VV，三步）”，打开对话框时按采集通道自动选。解算/激活/质量回报/文件导入（允许只有 P2 的文件）都支持缺一个端口。
  - **VV 背景扣除**：空采解算同时保留 S22 行（`background_s22`），激活时用 P2 校正；采集时背景合成 `(2, F)`，`process_trace_pipeline` 各通道只减自己的背景（仍遵守 R-C-19 不交叉套用）。旧的单行背景语义（只扣 HH）保留兼容。背景导出新增 `background_s22_real/imag`。
  - 校准匹配回报新增通道覆盖提醒：已有校准但所选通道缺对应端口校准或缺 VV 背景时给橙色提醒。
  - 测试：`tests/test_single_channel_and_vv_background.py`（21 项，含模拟器真跑一次只采 S22 落盘）。
- [x] 2026-10-15 单通道真正提速（同一分支 `feature/single-channel-vv-background` 第二个提交）。
  - 只采 HH：雷达改用只让端口 1 发射的扫法 `0x1240`（真机 ACK 验证过），连续采集后端 `PersistentLibreVnaBackend` 现接受 `0x1240`，按扫法选数据点解析器（`parse_s11_point`），没测的 S22 填 0 占位，上层按所选通道丢弃。
  - 只采 VV：新增 `S22_STAGES_BITMAP = 0x1208`（Stages=1 阶段，端口 2 在 stage0 发射，端口 1 指到不存在的 stage1；与 0x1240 对称推导），**未经真机验证**，不进 `ALLOWED_STAGES_BITMAPS`，只有 `SweepSettings(allow_experimental=True)` 才接受；解析 `parse_s22_point`（stage0 Port-2 / stage0 参考）。由 `air_config.json` 的 `experimental_s22_fast_sweep`（默认 false）控制；关闭时只采 VV 仍用 `0x1241` 再丢掉 HH。
  - `air_core.stages_bitmap_for_channels()` 统一决定扫法；`AcquisitionService.open_and_start(cfg, stages_bitmap)`，扫法变化会触发 reconfigure（冻结配置字节含 bitmap）。启动日志打印“采集通道 / 雷达扫法”。
  - 校准（OSL / 空采）仍用双通道 `0x1241` 采；端口 1 的接收路径在两种扫法下相同，按理误差项一致，但需真机对比确认。
  - 模拟器按 SWEEP_SETTINGS 里的 bitmap 出三接收机单端口包，单阶段节奏减半。
  - 删除旧用例 `test_librevna_backend.test_single_channel_0x1240_rejected`（行为已改）。新增测试 7 项（共 28 项）。
  - **真机验证清单**：①只采 HH 实测速度≈双通道一半耗时、HH 数据与双通道 HH 一致；②打开开关后只采 VV：确认设备 ACK `0x1208`、每点 3 个接收机、VV 数据与双通道 VV 一致。

- [x] 2026-10-16 Windows 实测反馈修复（分支 `fix/hide-uncollected-channel`）。
  - 单通道采集时主界面**隐藏**没采的那幅 B-scan（设置中心选了单通道即隐藏；收到的单通道数据也会触发），留下的那幅占满原来两幅的高度（该容器宽高比上限临时减半，隐藏行 row stretch=0）；容器仍在 `bscan_area` 布局里，只是 `setVisible(False)`，切回双通道或清空 B-scan 恢复。没采通道仍在内存里补零以保持道数同步，但不再显示。
  - 色阶滑块面板加最小宽度 44 px、固定宽度策略，防止被 B-scan 挤没。用户反馈“看不到滑块”的主要原因是测的是 PR #4 分支（不含 PR #1），main 上已有滑块。
  - 新增测试 3 项（`test_single_channel_and_vv_background.py` 共 31 项）。

- [x] 2026-10-22 整次采集合并 + 地面站历史采集回看（分支 `feature/run-merge-offline-viewer`）。
  - 新增 `run_archive.py`（纯 numpy）：把 `remote_runs/<run_id>/trace_*.npz`（地面命名）或 `gpr_runs/<run_id>/trace_NNNNN_<时间戳>.npz`（空中命名）合并成 `<run_id>_merged.npz`（format=`uav_gpr_run_merged_v1`）。按编号排序去重，缺号/重复/损坏/参数不一致逐条记账并跳过；先写 `.tmp` 再原子改名；原始单道文件一律保留。
  - 离线重处理从 `display_*`（空中端已做完校准+背景扣除的频域数据）重做 BPF→IFFT→(可选)Dewow，再按需 Flat Filter；与 `air_core.make_time_trace` 同函数同参数，测试锁定“同参数重算 = 实时结果”。老格式无 `display_*` 时退回已存时域 trace。
  - 新增 `offline_viewer.py`：地面站菜单「B-scan → 历史采集回看……」打开的独立非模态窗口（打开文件夹/合并文件、整条测线 B-scan、Dewow/Flat Filter、色阶/色图、合并保存、导出处理结果）。主界面按钮与菜单栏顶层不变。
  - 新增 `tools/merge_runs.py` + `merge_runs.bat`：双击合并 `remote_runs` 下全部采集，或把采集文件夹拖上去；默认跳过 `_c`/`_idle`/`_` 开头文件夹。
  - 已知限制：trace 里只有 `calibration_id`，校准系数与空采背景不随数据走，离线暂不能“换背景”；数千道合并约数秒、文件约百 MB 量级。
  - 测试：`tests/test_run_archive.py`（11 项）。

- [x] 2026-11-12 空中端 GNSS 断流抢 GIL、多行 FIFO、GGA/RMC 审计与静默安全探测修复（分支 `my-modifications`）。
  - **`read_gga` 空转抢 GIL 修复**（`air_core.py`）：串口半断开立返 `b""`、持续噪声/非 NMEA 字节或校验被拒行时施加 `READ_LOOP_MIN_INTERVAL_S = 0.01` 让出 CPU，消除单核忙转拖慢连续雷达采集（实测约 0.7 → ~9 Hz 恢复）；消费已入队完整行时零额外延迟。
  - **256 行有界 FIFO**（`air_core.py`）：`LINE_FIFO_CAPACITY = 256` + `fifo_overflows` 溢出计数，同块多条 NMEA 与跨块半句严格按到达顺序（含 `quality` 0→1→4→5）逐条返回不丢行；关闭/重置时同步清空残留。
  - **GGA/RMC 双报文审计与历元去重**（`air_core.py`）：两类报文统一解析记入 `sentence_audit`，同历元去重并记录有效性/坐标冲突（`epoch_conflicts`）；保持 GGA 优先、禁止旧 RMC 覆盖较新 GGA，支持 `23:59:59 ↔ 00:00:00` 跨午夜与冷启动授时跳变，持续 `quality=0` GGA 不压制后续较新有效 RMC 兜底。
  - **停用静默改速写配置**（`air_core.py`、`tests/test_air_gnss.py`）：默认波特率统一为 `9600`；静默超时期间停用切 `115200` 及下发 `com com1/com3 9600` + `saveconfig` 写 flash，仅保留同速率物理口输出探测。
  - **诊断日志与解类型分类**（新增 `gnss_diagnostics.py`、`air_gnss.py`）：新增默认关闭的可选滚动 JSONL `GnssLogWriter`（记录原始字节、报文审计、指令/应答、连接与发布快照，容忍重连、轮转与写盘异常，黑匣子入口零改动）；新增 `solution_type`（仅有效 GGA `fix_quality==4` 判 `rtk_fixed`、`5` 判 `rtk_float`）与默认 `off` 的 `fix_policy`（仅影响 `fix_eligible` 元数据，既有 `snapshot()`、NPZ 10 字段 `GgaFix` 与地面显示 100% 兼容）；`snapshot()` 区分 `configured_port/baud` 与实连 `connected_port/baud`。
  - 测试：新增 `tests/test_gnss_specialist.py`（20 项，含连续雷达防饿死实测与 `20261106_151256_0009` 125 历元历史回放）。

- [x] 2026-11-12 标定校准三项现场问题端到端修复（分支 `my-modifications`）。
  - **问题 1（导入校准文件后未真正应用到采集数据）**：`uav_gpr_air_service.py` 新增 `_align_sweep_to_calibration`，在 `cal_import_file(activate=True)`、自启动恢复与 `cal_activate` 时将空中端射频扫频网格（`start_mhz`/`stop_mhz`/`points` 及合法 `ifbw_hz`/`power_dbm`/`dwell_us`）自动对齐生效校准（保持 Dewow/IFFT 处理参数不变），并通过回执 `sweep` + `calibration_fit` 同步地面 `acq_settings` 与向导输入框，消除此前因扫频网格不一致被 `calibration_guard` 静默判 `stale` 停用校准的问题（导入后若用户再手动 `set_params` 改频率/点数仍照常判 `stale`）；导入同时自动取消未激活的空闲会话、支持仅含 P2 的 VV 单端口文件及含空格/括号的 Windows 副本文件名。
  - **问题 2（校准中途点窗口 `×` / `Esc` 关闭未点“取消会话”导致空中端卡死拒采）**：`CalibrationWizard` 补齐 `closeEvent` 与 `reject` 自动发送 `cal_cancel`（含 `_cancelled_cal_ids` 防在途 `cal_begin` 迟到 `planned` 复活、断线待发补偿、幂等防重发，已激活校准绝不误取消）；空中端允许校准自身 `ServiceStepCapturer` 在途时接收同会话 `cal_cancel`，经 `_BlackboxCalibrationCapturerProxy.cancel_all` + `CalibrationSession.capture` 取消守卫立即唤醒中止在途采样且不伪发 `step_done`/`failed`，关闭窗口后可立即 `start`/`single`/`set_params`。
  - **问题 3（导入校准文件后点击“查询生效状态”卡在查询中/查不到状态）**：地面端 `on_command_result("cal_query_active")` 与 `cal_status` 回程新增 `_format_active_query_summary` 刷新 `status_label`、`result_label` 与 `active_cal_label`（含 ID、文件名、端口、频点数、频段、Through/背景及 `calibration_fit` 状态，不再停留在“正在查询…”）；空中端统一导入 `cal_id` 口径，并修复 `_command_calibration_status` 在无活动 `_calibration_session` 时漏报已导入/自启动恢复校准的问题。无新增消息 `type`，主窗口 4 个业务按钮与状态栏 3 个标签冻结契约保持不变。
  - 测试：新增 `tests/test_calibration_ux_fixes.py`（12 项真实 TCP 双端 E2E + 回归用例，含 `1016dual.json` 与 `solt_2port_20260807_202342.json` 数值级校正核对）；全量 `docs\run_tests_batched.py` 47 模块 1206 项通过。

- [x] 2026-11-12 重打发布包 `20261112`（含上述 GNSS 与校准修复，空中端 + 地面站）。
  - **打包清单缺模块缺陷（上机即崩）**：`air_gnss.py` 顶部为裸 `from gnss_diagnostics import (...)`，`uav_gpr_air_service.py` 又直接 `from air_gnss import AirGnssMonitor`，而 `build_release_98f8530.py` 的 `AIR_PY_FILES`/`GROUND_PY_FILES` 均未包含新增的 `gnss_diagnostics.py`。已在隔离副本实测：缺该文件时 `import uav_gpr_air_service` 抛 `ModuleNotFoundError`，包一启动即退出。地面站包同样受影响（`run_air_station.bat` 本机联调会拉起黑匣子服务）。
  - **修复方式**：新增 `dist/build_tools/build_release_20261112.py`（`dist/` 属 `.gitignore`，与历史构建脚本同处不入库），复用 `build_release_98f8530.py` 组装函数与 `601a2ad` 的清单/自检/冒烟逻辑，仅把 `gnss_diagnostics.py` 补进两个包清单（空中 15→16 项、地面 21→22 项），**未修改任何历史构建脚本**，`98f8530`/`601a2ad` 两个旧包仍可原样复现。
  - **产物**：`dist/发布包_20261112/` 下三个包——`空中端黑匣子便携包_20261112.zip`（278.6 MB，6627 文件）、`空中端黑匣子便携包_20261112_slim.zip`（27.8 MB，654 文件）、`地面站便携包_20261112.zip`（278.8 MB，6633 文件）。旧包 `dist/发布包_601a2ad/` 未被触碰。
  - **验证**（全部为实跑，非推断）：① 包内 runtime 自检 `AIR_SELFTEST_OK`/`GND_SELFTEST_OK` 通过；② 追加自检 `AIR_GNSS_OK`/`GND_AIR_IMPORT_OK`，黑匣子服务在包内可导入且 `gnss_diagnostics` 在位；③ 地面站真启动冒烟（offscreen 8s）窗口存活、无导入/语法错误；④ 三个包的 `RELEASE_MANIFEST.txt` 共 104 条 SHA-256 条目与磁盘文件逐一比对全通过（空中 33/瘦身 32/地面 39），清单头部幂等重建无重复；⑤ 三包 `air_core.py`/`air_gnss.py`/`gnss_diagnostics.py`/`uav_gpr_air_service.py` 等与源码逐字节一致；⑥ 三个 zip 解压 CRC 全部完好，关键修复标记在位；⑦ 瘦身版解压到干净目录后用**包内自带 python** 端到端验证：`READ_LOOP_MIN_INTERVAL_S=0.01`、`LINE_FIFO_CAPACITY=256`、默认波特率 9600、`fix_policy=off`、生产配置 `192.168.144.20/usb/9600/calibration_path=""`、`load_air_config` 无 issue。
  - **配置红线**：工作区 `air_config.json` 为开发调试值（`127.0.0.1` + `simulated`，未提交），三个包内配置一律由构建脚本重新生成为现场值，已在构建中加断言拦截。
  - **未验证项（需现场联调）**：三包均未带真雷达 + 真 GNSS 接收机验证；`_align_sweep_to_calibration` 路径上未见"采集中禁止导入校准"的运行态拦截，建议上机前试一次"采集进行中导入校准"。

---

## 4. 下一步任务建议

0. 历史采集回看后续：让空中端把当次校准系数和 HH/VV 空采背景随数据发到地面并写进合并文件，离线即可“换背景/重新校准”；可选“停止采集后自动合并”。
0. 架构技术债第 2 步：拆分 `air_core.py`（按采集/GNSS/校准/发送分文件，注意测试大量 `patch.object(air, ...)`，拆分时要保证被 patch 的名字仍从同一模块查找）；第 3 步：拆分 `uav_gpr_ground_station.py`；补 CI。

1. 保持当前架构稳定性，新增需求先编写测试用例或在对应模块添加独立单元测试。
2. 任何涉及空地通信的改动，务必运行 `docs\run_tests_batched.py` 验证 1191 项测试全部通过。
