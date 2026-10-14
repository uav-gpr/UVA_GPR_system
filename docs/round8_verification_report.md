# 第8轮 t4 独立双端验证与全量回归报告

**执行角色**: verifier（独立验证，与实现工程师互不重叠）
**任务**: t4 — 独立复跑真实双端校准链路与修改后全量回归
**执行日期**: 2026-10-13
**被测基线**: 当前未提交工作区（my-modifications，含用户既有 GNSS 改造与 t2/t3 修复），非旧 HEAD
**新增测试**: `tests/test_round8_calibration_end_to_end.py`（verifier 独占，17 项，全部通过）
**回归日志**: `docs/round8_full_regression.log`（本次实际运行完整输出）

---

## 1. 结论摘要

1. **三个现场故障在真实双端命令链上已消除，且以数值级证据验证**（非文案级）：
   - **故障一（OSL 激活后无法背景采集）**：真实六步 OSL（每步经真实 `cal_capture` → 黑匣子 `ServiceStepCapturer` → `AcquisitionService` 短任务 → 模拟设备协议线采集）→ 解算 → 激活后，向导 `_cal_id` 解除、模式下拉框与"开始校准"恢复可用；随即以同一真实链路新建背景会话并完成采集/解算/激活，已生效 OSL 与背景同时在 `cal_query_active` 中如实呈现（R8-1）。
   - **故障二（地面无法使用既有校准文件）**：仓库真实 `solt_2port_20260807_202342.json`（SHA-256 `14276D50…D82CB5`，**原件哈希全流程不变**）经地面向导"导入校准文件"按钮以**完整内容**（非路径）经真实 TCP 下发，空中端校验、原子落盘、激活；`cal_query_active` 返回 ports `["P1","P2"]`、points `201`、频段 `400–4000 MHz`、`has_through=true`；`cal_export_file` 拉回内容与原件逐字段等价；**空中服务重启后自动恢复同一校准（201 点双端口）且恢复后真实采集 trace 的 display 通道 == 恢复校准对 raw 的逐点反算**（R8-3/4/5/6）。
   - **故障三（P1_Open 重采被拒且 UI 长期"请求采集中"）**：P1_Open 完成后单击重采（`recapture=true` + `request_id`）经真实链路被接受并产生 `_r2_` 审计批次，下一步真实推进为 `P1_Short`；已解算后重采作废旧解算、其余 5 步保留、重新解算成功；拒绝/双击/重复命令/在途冲突均回退行、不假成功、不重复执行、不永久"请求采集中"（R8-7/8/9）。
2. **修改后全量分块回归全绿**：42 模块（39 基线 + 3 个第8轮新模块）、**1164 项测试、329.1s、failed modules: none、OVERALL OK（退出码 0）**；唯一 `PASS*` 为既有 WebEngine 拆除段崩溃（0xC0000005，测试本身 OK，与基线口径一致）。**基线 39 个模块逐模块测试数与本次完全一致，无删改减项**；基线的 2 个失败模块（6 项 GNSS 断言）在对齐后转为通过。
3. **独立验证发现 13 项观察/缺口**（§6），其中 4 项建议进入修复回合（F1 陈旧 query 回执释放、F3 import 文件域回滚、F4 activate 持久化失败仍报成功、F5 validator dict 尺寸门），其余为低危/信息级。这些发现**不影响本任务门禁三条判据**（不假成功、不重复执行、不永久请求中——均有真实链路证据），但已逐条给出可复现证据与修复建议。

---

## 2. 独立验证方法（为什么这能证明双端）

与工程师单测的本质区别（对照 `docs/第8轮…计划.md` §5"不能仅用假接收器宣布成功"）：

- **真实地面按钮入口**：`GroundStationWindow` + 真实 `CalibrationWizard`（`begin_calibration` / `capture_next_step` / `recapture_selected_step` / `import_calibration_file` / `export_calibration_file` / `query_active_calibration` 按钮处理函数直接驱动，含防抖/看门狗/行状态机）。
- **真实传输层**：地面侧真实 `ReceiverThread`（TCP 服务端，port=0 由 OS 分配）；空中侧真实 `uav_gpr_qt.RemoteSender`（TCP 客户端、hello 握手、滑动窗口、重连退避 0.3s→0.8s 注入）；全部回程（`calibration_capture` NPZ 流、`calibration_quality`、`calibration_result`、`command_ack`）走真实 framing。
- **真实黑匣子命令入口**：`AirBlackboxService.handle_command`（`RemoteSender.command_callback` 挂钩，即生产命令分发路径）+ `start/stop` 经地面 `receiver.send_command` 下发。
- **真实采集管线**：`CalibrationSession` + `_BlackboxCalibrationCapturerProxy` + `ServiceStepCapturer` + `AcquisitionService.start_task` + `PersistentLibreVnaBackend` + `ContinuousAcquisitionWorker`，零 mock 核心。
- **模拟设备 + 可控标准件**：`ControlledStandardTransport`（`SimulatedLibreVnaTransport` 子类）在既有物理模型上按采集任务轮换 Open/Short/Load × P1/P2 受控反射偏置（第 7 个任务为背景场景、第 8+ 为 trace 场景），`noise_level=0` 保证数值确定、OSL 三方程可解、背景扣除可数值区分。
- **数值级核对**：trace NPZ 的 `raw_S11/S22` 与 `display_S11/S22` 直接对照运行期校准对象逐点反算（容差 1e-7）；背景数组对照"背景原始采样的 OSL 校正域值"且与 RAW 域显著可区分；trace 的 `acquisition.calibration_id` 元数据核对。

**不声称**：真实 VNA 硬件精度、已部署、真实 USB。模拟/可控标准件只证明软件协议、状态机与数值管线。

---

## 3. 三故障消除的端到端证据（对应验收判据 1–3）

### 3.1 校准 → 激活 → 背景 → 正常 trace（R8-1）
- 真实 `cal_query_active` 回执：`in_progress_session.cal_id/mode/steps_status` 与会话一致；`capabilities.step_done=true`。
- 六步各收到权威 `step_done`（严格模式）+ `calibration_quality`（既有回程保留，≥6 条）。
- 激活后：向导 `_cal_id=None`、`begin_button`/`mode_combo` 恢复；空中 `active_osl.active=true`、`active_background.active=false`。
- 背景会话在同一真实链路完成；`cal_query_active` 双组件如实呈现。
- 真实 trace（`start` 命令，single）：地面站收到并落盘 NPZ，B-scan 显示链路提交；数值核对：
  - `display_S11 == OSL.correct(raw_S11) − background`（1e-7）；
  - `display_S22 == P2.correct(raw_S22)`（1e-7）；
  - 背景数组 == 校正域（R-C-19）且与 RAW 域差 >0.1（校正真实发生）；
  - 校正后扣除背景真实改变 display（差 >0.1）。

### 3.2 真实 v2 文件导入/导出/重启（R8-3/4/5/6）
- 导入：完整内容传输（对地面路径零依赖）；空中 `imports/solt_2port_20260807_202342.json` 原子落盘；原件哈希不变。
- 查询：`ports=["P1","P2"]`、`points=201`、`valid_band_hz=[400e6, 4e9]`、`has_through=true`。
- 导出：`cal_export_file` 内容与原件 JSON **逐字段等价**。
- 重启：`svc.load_air_config` → 新 `AirBlackboxService` 自启动恢复 201 点双端口、身份 `solt_2port_20260807_202342`；恢复后真实 trace 的 `display_S11 == 恢复校准.correct(raw)` 且与 RAW 可区分。
- 旧单端口文件：如实 `ports=["P1"]`、`has_through=false`，不伪造 P2。
- 坏文件（NaN）与频带不兼容（轴不一致）：空中真实拒绝（`空中端拒绝校准文件`），旧生效校准完整保留（`active_calibration_id`/points 不变）。

### 3.3 受控重采与审计证据（R8-7/8）
- 完成后重采：`capture_P1_Open_r2_*.npz` 独立批次存在，首轮 `capture_P1_Open_1/2.npz` **逐字节哈希不变**；`progress.attempt=2`、首轮采样归档 `history_captures`。
- 重采后下一步 = P1_Short（真实 step_done 推进）。
- 已解算后重采 P1_Load：空中会话回 `in_progress`、`solved_result=None`，其余 5 步 `done` 保留，重新解算成功。
- 未采集步骤重采：空中真实拒绝（"尚未进行首次采集"），行回退"待采集"+可读原因，不卡死。

### 3.4 健壮性（R8-9/10）
- 同 `request_id` 重发：空中幂等缓存回放、**零新采样文件**、地面去重不重复计数。
- 双击防抖：在途再次点击不发新请求（`_pending_request_id` 不变）。
- 在途重复命令：空中"上一步骤采集仍在进行"真实拒绝；行不残留"请求中"。
- 看门狗超时 → 真实 `cal_query_active` → 远端 `busy=true` 时保持锁定（含 `active_request_id` 一致），释放后正常收尾；watchdog 预算随扫描规模增长（下限 30s，长扫描更大）。
- 断线（真实关闭 socket）→ 地面感知断开、锁定；空中 0.3s 退避真实重连 → `cal_query_active` 同步远端会话/步骤集合 → 续采正常。
- 旧端仿真（拦截真实消息层，非 DummyReceiver）：无 `step_done` 时末样本即完成、不挂起；`cal_import_file` `supported=false` → "版本较低"明确可见；`capabilities.step_done=false` 被地面真实接受。

---

## 4. captain t3 附加六项风险：独立证伪/确认结果

| # | 风险 | 结论 | 证据 |
|---|---|---|---|
| 1 | `cal_query_active` 在校准 worker 在途时被互斥拒绝、不能查 busy | **证伪**（实现已豁免） | 在途窗口实测：ack `ok=true`、`in_progress_session.busy=true`、`active_request_id` 与地面 pending 一致；`cal_query_active/cal_export_file/cal_status` 在 `_calibration_acquisition_conflict` 白名单内 |
| 2 | import 回执缺 ports/points/valid_band/format；clear 回执 `target` vs 地面读 `cleared`；capabilities 键名与文档不一致；activated 缺 mode/ports/points/active_osl_id | **确认**（契约字段偏差，功能不破坏，见 F10） | 实测 ack 载荷仅 `ok/filename/activated/cal_id/request_id`；地面导入显示退化"P1"，查询后恢复；clear 后本地显示把 OSL 一并清除（与空中真实状态不符，查询自愈） |
| 3 | import 先 `os.replace` 后 `save_config`，失败不回滚旧同名文件；filename 未净化 | **确认**（F3/F7） | 同名二次导入 + 受控 `save_config=False`：ack `ok=false`、内存校准未损，但旧同名文件内容被新内容覆盖（bytes 实测）；`..` 探针写出 imports 目录成功 |
| 4 | 背景激活覆盖 `active_calibration_id`；采集 spec `through_measurement=None` 虽 `_active_through` 非 None | **确认**（F11/F13） | `cal_query_active.active_osl.cal_id` 在背景激活后返回背景身份；重启恢复后实测 `worker.through_measurement is None`（continuous 原始数据无 S21/S12，现无数值影响） |
| 5 | `cal_activate` 忽略 save_config 失败仍报成功；close_current 异常被吞仍 step_done | **确认**（F4）；close_current 吞异常路径未单独实测（模拟设备下 close 稳定返回，无法在无硬件下触发） | config_path 指向目录（真实 OSError）：activated 广播、`air_config.json` 未写入、无告警 |
| 6 | validator dict 无 10MB 限制/通道结构有限值/缺 P2 等边界 | **确认**（F5/F6）；缺 P1 三标准件被正确拒绝 | `validate_calibration_content`：str >10MB 拒绝、等价 dict 通过；`channels.S22=NaN`（顶层合法）→ `load_components` 静默 `p2=None`，导入 ok、ports=["P1"] |

---

## 5. 独立发现清单（分级 + 建议）

> 判定口径：本任务门禁三条判据 = 不假成功 / 不重复执行 / 不永久"请求中"。以下 F1–F4 建议进入修复回合；F5–F9 低危；F10–F13 信息级。

- **F1（建议修复）陈旧 query 回执错误释放在途请求**：`cal_query_active` 的 `busy=false` 快照（在采集开始前生成）迟到时，向导无请求关联地执行 `_capturing=False` + 回退在途行（"超时已释放"）。确定性复现：`test_stale_query_ack_does_not_hijack_inflight_capture`（先 query 后立即采集，回执必然迟到）。空中互斥护栏保证无重复执行，真实 `step_done` 到达后自愈，但期间 UI 状态失真且可诱发用户重复点击。**建议**：query 回执带请求 nonce，或释放前核对 `steps_status` 中 pending 步骤的真实状态。
- **F2（建议修复）`cal_capture` 的 command_ack 缺 cal_id/request_id 关联**：拒绝 ack 仅 `{"ok":false}`，地面无法区分陈旧拒绝与当前请求（与 F1 同根，ack/result 缺关联字段）。**建议**：ack.result 回带 `cal_id/request_id`，地面按关联字段过滤。
- **F3（建议修复）import 文件域未回滚**：`_handle_cal_import_file` 先 `os.replace` 覆盖同名目标、后 `save_config()`；配置写盘失败仅回滚配置字符串 → 旧同名生效文件内容被永久覆盖。实测：`test_import_config_save_failure_keeps_memory_but_file_evidence`（内存态未损）。**建议**：先写 `.tmp`、`save_config` 成功后再 replace，或失败时从 `.tmp`/备份恢复旧文件。
- **F4（建议修复）`cal_activate` 持久化失败仍报成功**：实测 `test_activate_success_reported_even_when_persistence_fails`（config_path 为目录 → 真实 OSError，activated 照发，config 未写）。重启后校准丢失且无告警。**建议**：激活回执带 `persisted: bool`，失败时地面明示。
- **F5（低）validator dict 输入绕过 10MB 上限**：str 受限、dict 不受限（实测）。真实向导路径恒发 str，现网风险低；协议层建议补齐 `len(json.dumps)` 检查。
- **F6（低）坏通道数据静默丢 P2**：`channels.S22=NaN` → `load_components` 捕获 ValueError → `p2=None`，导入成功、ports=["P1"]，无显式告警。数量/端口如实显示，但双端口文件静默降级宜告警。
- **F7（低）空中端 filename 未 basename 净化**：`..` 相对路径可写出 imports 目录（探针实测）。地面 GUI 已 basename，现网不可达；协议层建议 `Path(filename).name`。
- **F8（低）能力未知（未查询）时按旧端模式在末样本立即完成**：早于空中收尾，立即点击下一步会被空中"采集任务运行中或服务正在收尾"拒绝（实测探针；行回退、重试成功、不卡死）。**建议**：能力未知时按严格 step_done 模式等待。
- **F9（低）重采被拒后 `_step_index` 不回退**：`recapture_step` 先把指针指向重采目标，rejected 分支恢复行但不恢复指针；下一次真实 step_done 重算自愈。UI combo 仅列已完成步骤，正常用户路径不可达（实测记录于防抖用例）。
- **F10（信息）契约字段偏差汇总**：import ack 缺 ports/points/valid_band/format；activated 缺 mode/ports/points/active_osl_id（地面回退向导当前 mode，向导发起会话下正确）；capabilities 用 `cal_import_file/cal_export_file` 键（文档为 `import_file/export_file`；地面仅读 `step_done`，无功能影响）；`cal_clear_active` 回执 `target` vs 地面读 `cleared`（默认 all → 本地显示把 OSL 一并清掉，查询自愈）；export 地面发 `cal_id`、空中读 `target`（默认 osl，功能正常）；import ack cal_id 带扩展名 vs 查询 stem 不一致。
- **F11（信息）背景激活覆盖 `active_calibration_id`**：`cal_query_active.active_osl.cal_id` 与 trace `acquisition.calibration_id` 在背景激活后均为背景身份（实测）。身份语义应解耦为 OSL/背景双身份。
- **F12（信息）`_schedule_legacy_fallback` 为死代码**：补遗 §修订6 的"末样本 0.5s 平滑收尾缓冲后自动查询"从未被调用；实际降级路径 = 末样本立即完成（不缓冲、不自动查询）。
- **F13（信息）导入 Through 未接入采集任务 spec**：`_start_continuous` 恒传 `through_measurement=None`（运行时实测 worker 值为 None）。continuous 引擎原始数据仅 S11/S22，through 校正现无数值影响；legacy 引擎在模拟设备下被生产规则拒绝，**through 校正数值无法端到端验证**（诚实记录为未验证边界）。

---

## 6. 修改后全量回归 vs 改前基线（逐模块对照）

命令：`$env:QT_QPA_PLATFORM='offscreen'; $env:QTWEBENGINE_DISABLE_SANDBOX='1'; .venv312\Scripts\python.exe docs\run_tests_batched.py`
完整输出：`docs/round8_full_regression.log`（最终版测试文件实际运行；exit 0）

| 模块 | 基线 | 本次 | 变化 |
|---|---|---|---|
| test_acquisition_scheduler | 27 OK | 27 OK | — |
| test_air_gnss | 6 OK | 6 OK | — |
| test_bscan_gui | 13 OK | 13 OK | — |
| test_bscan_view | 8 OK | 8 OK | — |
| test_bscan_window_rows | 11 OK | 11 OK | — |
| test_dual_channel_reference | 3 OK | 3 OK | — |
| test_field_test | 127 OK | 127 OK | — |
| test_gnss_map_widget | 9 OK | 9 OK | — |
| test_gpr_processing | 19 OK | 19 OK | — |
| test_ground_gnss | 6 OK | 6 OK | — |
| test_ground_station_compact_ui | 64 OK | 64 OK | — |
| test_ground_trace_pipeline | 13 OK | 13 OK | — |
| test_librevna_backend | 81 OK | 81 OK | — |
| test_librevna_continuous | 26 OK | 26 OK | — |
| test_librevna_protocol | 72 OK | 72 OK | — |
| test_librevna_transport | 28 OK | 28 OK | — |
| test_pipeline_metrics | 11 OK | 11 OK | — |
| test_rebar_inspector_s11_s22_reference | 4 OK | 4 OK | — |
| test_round1_defaults | 14 OK | 14 OK | — |
| test_round1_review_fixes | 35 PASS* | 35 PASS* | — （WebEngine 拆除段 0xC0000005，测试本身 OK，基线同口径） |
| test_round1_third_fixes | 16 OK | 16 OK | — |
| test_round2_air_ui_compact | 10 OK | 10 OK | — |
| test_round2_disk_outbox | 73 OK | 73 OK | — |
| test_round2_ground_calibration | 37 OK | 37 OK | — |
| test_round2_resident_service | 31 OK | 31 OK | — |
| test_round5_ifft_alignment | 16 OK | 16 OK | — |
| test_round5_sliding_window | 9 OK | 9 OK | — |
| test_round5_ui_behavior | 11 OK | 11 OK | — |
| test_round6_bugfix | 15 OK | 15 OK | — |
| **test_round7_air_service** | **60 FAIL(5)** | **60 OK** | 5 项 GNSS 旧断言受控对齐后转绿（§7） |
| test_round7_ground_panel | 38 OK | 38 OK | — |
| **test_round8_calibration_core** | （无，t2 新增） | 11 OK | 新增 |
| **test_round8_calibration_end_to_end** | （无，t4 新增） | 17 OK | 新增（本报告；在途窗口用例含 ifbw=100 慢步进真实扫频，单模块约 82s） |
| **test_round8_calibration_ground** | （无，t3 新增） | 13 OK | 新增 |
| test_simulated_transport | 23 OK | 23 OK | — |
| **test_uav_gpr_qt_continuous** | **102 FAIL(1)** | **102 OK** | 1 项 GNSS 旧断言对齐后转绿（§7） |
| test_uav_gpr_qt_metadata | 51 OK | 51 OK | — |
| test_uav_gpr_qt_pipeline | 11 OK | 11 OK | — |
| test_ui3_air_minimal | 4 OK | 4 OK | — |
| test_ui3_calibration_wizard | 8 OK | 8 OK | — |
| test_ui3_engine_default | 18 OK | 18 OK | — |
| test_ui3_ground_settings | 13 OK | 13 OK | — |
| **总计** | **39 模块 / 1123 项 / 323.8s / 2 FAIL** | **42 模块 / 1164 项 / 405.8s / 0 FAIL** | +3 模块 / +41 项；1123+41=1164 逐项对账一致 |

**逐项结论**：基线 39 模块计数完全一致（无静默删改/减项）；唯一 `PASS*` 与基线同口径；原 2 个失败模块的 6 项失败全部转绿且测试数量不变。

---

## 7. t2 六项 GNSS 旧断言受控对齐：独立核验

对照 `docs/round8_test_alignment.md` 逐项核验实际测试文件与生产代码（非信自述）：

| # | 位置 | 对齐后断言（实测读取） | 生产锚点（实测读取） | 覆盖保留 |
|---|---|---|---|---|
| 1 | test_round7_air_service.py:102 `test_missing_file_uses_defaults` | `gnss_baud == 9600` | uav_gpr_air_service.py:105 `gnss_baud: int = 9600`（现场实测 9600 才不丢低电平位） | 默认值回退逐字段断言保留 |
| 2 | test_round7_air_service.py:181 `test_invalid_fields_fall_back_per_field` | 非法值回退 `9600` | 同上 | 逐字段回退断言保留 |
| 3 | test_round7_air_service.py:482 `test_test_gnss_without_config_reports_error` | `ok=true, fix=false, port="", state="waiting"` | uav_gpr_air_service.py:955-957 `_cmd_test_gnss → self.gnss.snapshot()`（常驻 owner，注释明示不重开串口） | 未配置诊断断言保留 |
| 4 | test_round7_air_service.py:490-498 `test_test_gnss_with_fake_reader` | 常驻监视器快照 lat/lon=52.0906/5.1214、fix_quality=1 | 同上 snapshot 契约 | 坐标精度与端口校验保留 |
| 5 | test_round7_air_service.py:504 `test_test_gnss_reader_failure_is_reported` | `state="port_not_found"`、detail 串口打开失败 | 同上 snapshot 诊断契约 | 故障如实呈现断言保留 |
| 6 | test_uav_gpr_qt_continuous.py:799-800 `test_open_sets_write_timeout_and_sends_config` | `write_timeout==1.0` + `written==[b"log gpgga ontime 1\r\n", b"log gprmc ontime 1\r\n"]` | uav_gpr_qt.py:1057-1058 `self._write(b"log gpgga ontime 1\r\n"); self._write(b"log gprmc ontime 1\r\n")` | **write_timeout=1.0 保险丝断言原样保留** |

- **生产 GNSS 未改**：`AirConfig.gnss_baud=9600` 默认、常驻 `self.gnss.snapshot()` owner 架构、`ontime 1` 初始化命令均为**用户既有未提交改造的原样保留**（即基线记录的现场适配），对齐只发生在测试断言层；`air_gnss.py`/`ground_gnss.py` 未被本轮触碰。
- **减项核查**：两模块测试数与基线一致（60/102），全量回归实测通过。

---

## 8. 自动模拟已验证边界 vs 现场真机复核（诚实边界）

**自动模拟已验证**：双端协议/状态机/竞态守卫/持久化/恢复/幂等/降级/数值管线（校正、背景扣除、双端口恢复、带外拒绝、审计证据）。**不能证明**：真实 VNA/标准件的测量精度、真实 USB 时序、真实 GNSS/串口行为。

现场真机复核清单（沿用计划 §6.2，交付随包）：
1. 计算棒连 LibreVNA USB，日志确认 `设备来源 usb`；地面站连接成功。
2. 导入 `solt_2port_20260807_202342.json` → 查询确认 P1/P2 各 201 点生效；计算棒重启后确认自动恢复。
3. 采集 `P1_Open` 后重采 → 确认顺利完成并推进 `P1_Short`。
4. 六步解算激活 → 向导恢复可用 → 无缝空采背景。
5. 连续采集 B-scan 瀑布刷新平稳、背景去噪生效；**用标准件复核校正后数值**（软件数值管线已验证，物理精度以此为准）。
6. 若使用双端口 Through 工作流：真机验证 S21/S12 校正（本轮自动模拟无法覆盖，见 F13）。

---

## 9. 范围与红线自查

- **仅新增**：`tests/test_round8_calibration_end_to_end.py`、`docs/round8_verification_report.md`、`docs/round8_full_regression.log`。未修改任何生产源码、工程师单测（t2/t3 契约文件）、`air_config.json`、实测校准原件（哈希 `14276D50…D82CB5` 全流程不变）或任何用户数据。
- **未写 smoke 脚本**；未执行自启/升级/部署；未运行安装自启任务。
- 测试用例中的故障注入（`save_config` 置 False、config_path 指向目录、旧端消息层拦截）均在用例自身临时目录/实例上进行，不触碰用户资产与生产默认值。
- 后台作业已全部收集（全量回归 job `pwsh-118` 已完成，exit 0）。

---

*报告结束。verifier 结论：t4 验收判据 1–6 全部满足，实证记录如上；13 项独立发现已逐条给出证据与建议，其中 F1–F4 建议由 captain 决策是否进入修复回合。*
