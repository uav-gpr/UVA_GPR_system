# 第8轮 验证口径校正与正式回归转换说明（docs/round8_verification_corrections.md）

**执行角色**: verifier（t10）
**日期**: 2026-10-13
**关联**: t4 验证报告（docs/round8_verification_report.md）、t5 审查（needs_revision，9 项缺陷）、t12 修复（repair-round-2，已完成）、t13 复审（进行中）、t11 最终验证（待跑）
**本文件只改验证资料与独立测试；不改生产、不改 t4 原终态。**

---

## 1. 验证口径校正（对 t4 报告的两点修正）

### 1.1 六条契约验收不缩水：正向主链已通过 ≠ 异常验收已通过

t4 报告结论把 6 条契约验收归并为"3 条判据（不假成功/不重复执行/不永久请求中）均满足"。此口径不完整，特此校正：

| # | 契约验收（原文口径，不缩水） | t4 实测状态 | 本文件口径 |
|---|---|---|---|
| 1 | 新增可重复真实双端集成测试；校准→激活→背景→trace 全链路+数值核对 | **通过** | 正向主链，维持通过 |
| 2 | 真实 v2 原件哈希不变/导入导出/重启恢复/旧单端口明示/坏文件保留 | **通过**（坏文件部分见 §3 F6） | 正向主链，维持通过 |
| 3 | P1_Open 重采→P1_Short/已解算重采/计数一致/旧解算失效/审计证据 | **通过** | 正向主链，维持通过 |
| 4 | 双击/重复命令/收尾未完/拒绝/发送失败/超时/断线重连/迟到或跨会话消息等不假成功、不重复执行、不永久请求中，旧端能力不支持明确可见 | **部分通过**：F1 陈旧回执释放在途请求、F2 ack 缺请求关联、F8 未知能力末样本解锁、F9 重采被拒指针劫持均实测成立——临时自愈不判达标 | **异常验收未通过**（§3 正式回归当前红） |
| 5 | 分块全量回归与基线对照、完整记录、无删改减项 | **通过**（1164/42 模块/0 FAIL，逐模块对账一致） | 维持通过 |
| 6 | 只改独立测试和报告范围；边界声明 | **通过** | 维持通过 |

结论口径（代替 t4 报告的概括句）：**"正向主链（契约 1/2/3/5/6）已通过；契约 4 的异常分支验收未通过，F1/F2/F8/F9 及 F10 显示/关联等缺口已由 t10 转换为期望正确行为的正式回归（当前红），待修复后在 t11 全绿。"** 6 条契约不缩成 3 条；任何"step_done 后自愈/查询后显示自愈"只作恢复证据，不作达标依据。

### 1.2 全量日志口径校正（329.1s / 405.8s 与编码）

- **编码**：`docs/round8_full_regression.log` 由 PowerShell `Tee-Object` 生成，实为 **UTF-16LE（BOM FF FE）**，故被文本读取器判为 binary。原文件字节未动（不在本任务范围）；已无损转存为 **`docs/round8_full_regression_utf8.log`**（UTF-8 无 BOM，2768 字节，45 行，内容与原文件逐行一致）。
- **耗时矛盾**：329.1s 与 405.8s 是两次**真实**全量运行：329.1s 为中间版（t4 开发中，end_to_end 模块 38.4s）；**405.8s 为归档口径**（最终版测试文件，end_to_end 模块 82.5s——在途窗口用例采用 ifbw=100 慢步进真实扫频）。以归档文件 `round8_full_regression.log`/`_utf8.log` 为准：**42 模块 / 1164 项 / 405.8s / failed modules: none / OVERALL: OK**。两行归档摘要：
  `BATCHED TOTAL: Ran 1164 tests in 405.8s over 42 modules; failed modules: none`
  `shutdown-crash (environment drift, tests OK): test_round1_review_fixes (exit 3221225477)`

---

## 2. 诊断断言 → 正式产品回归的转换清单

t4 部分用例以"断言旧 bug 现状"的方式记录证据（其绿是诊断通过，不是正确行为通过）。t10 将其全部转换为**期望正确产品行为**的正式回归：场景/方法/覆盖保留，不删/skip/expectedFailure。**修复前现状可红**（当前红/绿见 §3）；全部走真实 wire（真实按钮/ReceiverThread/RemoteSender/黑匣子命令入口/真实采集线程+模拟设备），不以 handle_command 直调返回替代双端证明（仅校验器纯函数边界与"自己的实例桩"故障注入除外，均已注明）。

| 缺口 | 转换后正式回归（方法名） | 期望正确行为（断言要点） | 当前 |
|---|---|---|---|
| F1 陈旧 query 回执释放 | `test_stale_query_ack_does_not_hijack_inflight_capture` | 陈旧 busy=false 回执迟到时 `_capturing`/行/pending 保持锁定，真实 step_done 才收尾（恢复判据保留） | **红**（被 t12 回归阻塞，见 §3.1；缺陷本体仍存在） |
| F2 ack 缺请求关联 | `test_inflight_duplicate_rejection_keeps_current_request_locked` | 在途期间重复命令的真实拒绝不得解锁当前请求/清除 request_id/回退在途行 | **红**（同上阻塞） |
| F1-cross 跨会话 | `test_cross_session_...`（正式断言替换观察） | 跨会话陈旧拒绝不得解锁/回退 B 会话在途请求 | **红**（同上阻塞） |
| F3 import 文件域回滚 | `test_import_config_save_failure_preserves_old_file_bytes_and_memory` | 同名导入+save_config 失败：旧文件字节/配置/内存三不变，无半套残留 | **绿**（t12 已修，实证通过） |
| F3b filename 净化 | `test_air_import_filename_cannot_escape_imports_dir` | `..` 相对路径必须被净化/拒绝，不得写出 imports 目录（探针限用例临时树） | **绿**（t12 已修，实证通过） |
| F4 activate 假成功 | `test_activate_persistence_failure_must_not_broadcast_activated` | 持久化失败（真实 OSError）不得 ok=true 且广播 activated | **绿**（t12 已修，实证通过） |
| F5 validator dict 尺寸 | `test_validator_enforces_10mb_for_dict_and_string_alike` | dict 与 str 同受 10MB 上限 | **红**（实测 dict 绕过，err 为空） |
| F6 双端口坏通道 | `test_corrupt_dual_port_channel_data_must_be_rejected` | 声明双端口文件含 channels.S22=NaN 必须整笔拒绝，不得静默降级单端口 | **绿**（t12 已修，实证通过） |
| F8 未知能力末样本解锁 | `test_unknown_capability_must_not_unlock_on_last_sample` | 能力未知（未查询）时末样本不得完成步骤；step_done 为准；旧端降级须显式协商（既有旧端用例保留） | **红**（实测末样本即完成） |
| F9 重采被拒指针 | `test_pending_step_recapture_rejection_restores_pointer` | 重采被真实拒绝后指针恢复首个未完成步骤 | **红**（被 t12 回归阻塞） |
| F9 用户路径 | `test_recapture_rejected_then_normal_next_step_is_p1_short` | 完成 P1_Open→单击重采遇真实拒绝（真实在途忙碌）→普通下一步仍 P1_Short，不回 P1_Open 循环 | **红**（同上阻塞） |
| F10a import ack 字段 | `test_import_ack_and_query_report_consistent_complete_fields` | ack 携带 cal_id/filename/format/ports/points/valid_band_hz 且与 query 一致 | **绿**（t12 已修，实证通过） |
| F10b 仅清背景不伪清 OSL | `test_clear_background_only_must_not_clear_osl_display` | clear ack 带 cleared 字段；地面仅清背景显示，OSL 保持 | **红**（实测 OSL 显示被伪清） |
| F10c 并发导出关联 | `test_export_double_click_writes_each_target_correctly` | 连点两次导出：两个目标文件各自真实落盘且内容等价 | **红**（实测首个目标文件未落盘） |
| F10d activated 字段 | `test_activated_broadcast_carries_contract_fields` | activated 广播携带 mode/ports/points/valid_band_hz/active_osl_id | **红**（实测缺 mode/ports/points 等） |
| F11 背景覆盖 OSL 身份 | `test_osl_activate_background_and_calibrated_trace`（正式断言替换证据断言） | 背景激活后 query.active_osl.cal_id 仍为 OSL 身份；trace 元数据 calibration_id=OSL 身份 ≠ 背景身份 | **红**（被 t12 回归阻塞；t12 称已修 AIR-01，待 t11 复验） |
| F13 Through 接入 spec | `test_real_v2_import_export_query_restart_and_correction`（正式断言替换证据断言） | 采集 worker.through_measurement 必须为导入的 Through（非 None）；真机数值校正诚实注明不可无硬件验证 | **红**（被 t12 回归阻塞；t12 称已修 AIR-02，待 t11 复验） |
| close_current 注入 | `test_close_current_failure_must_not_fake_step_done` | close_current 注入失败（自己的实例桩）时严禁照常广播权威 step_done | **绿**（t12 已修，实证通过） |

**方法计数**：模块从 t4 的 17 项扩展为 **26 项**（+9：validator 拆分 +1、原防抖组合用例拆分 +2、F8 +1、F10 四项 +4、close_current 注入 +1；原 filename 诊断方法被 F3b 正式方法原位替换，场景保留）。既有绿用例（正向主链）全部保留，无删除/跳过/expectedFailure。

---

## 3. 当前实际红/绿快照（post-t12，真实归档）

命令：`$env:QT_QPA_PLATFORM='offscreen'; $env:QTWEBENGINE_DISABLE_SANDBOX='1'; .venv312\Scripts\python.exe docs\run_tests_batched.py --only round8_calibration_end_to_end`
（等价直跑：`.venv312\Scripts\python.exe -m unittest tests.test_round8_calibration_end_to_end`）

**结果：Ran 26 tests，FAILED (failures=18)——8 绿 / 18 红。**

### 3.1 【新阻断缺陷·t12 引入，已报 t13 复核】cal_query_active 对 fresh 服务抛 AttributeError

- **症状**：服务启动后未做 import/activate 时，任何 `cal_query_active` 返回
  `result={"error": "'AirBlackboxService' object has no attribute '_active_osl_id'", "ok": false}`
  → 地面能力协商失败（`_air_supports_step_done` 无法建立）→ 12 个以 `_begin_osl` 为前置的用例在能力握手处被阻塞（含正向主链全链路用例）。
- **定位**：`uav_gpr_air_service.py` 约第 1504 行 `active_osl_cal_id = self._active_osl_id or ...`；该属性仅在 import（L1392）/clear（L1592）/activate（L2003）路径赋值，`__init__` 未初始化。
- **最小复现**：fresh `AirBlackboxService` 实例直接 `handle_command("cal_query_active", {})`。
- **建议**：`__init__` 初始化 `self._active_osl_id = None`。**注意**：此缺陷掩盖了下表中"阻塞"用例的 F 缺陷证据，修复后这些用例应转为"红在 F 缺口本体"。

### 3.2 红（18）分类

| 组 | 用例 | 症状摘要 |
|---|---|---|
| 被 §3.1 阻塞（12） | disconnect_reconnect、osl_activate_background、real_v2_import_export、inflight_duplicate、pending_step_recapture、recapture_p1_open_continue、recapture_rejected_then、cross_session、same_request_id_replay、stale_query、watchdog_budget、watchdog_timeout | 统一症状：`cal_query_active 回执未建立 step_done 能力：last_ack={...error: '_active_osl_id'...}`；其中 F11/F13/F1/F2/F9 的缺口本体断言在其后，修复 §3.1 后即可见红 |
| F5（1） | validator_enforces_10mb_for_dict_and_string_alike | `dict 输入必须与 str 同受 10MB 上限约束（实测通过，尺寸门被绕过）` |
| F8（1） | unknown_capability_must_not_unlock_on_last_sample | `能力未知时末样本不得直接解锁在途请求（实测被解锁）` |
| F10b（1） | clear_background_only_must_not_clear_osl_display | clear ack 无 `cleared` 字段；地面 OSL 显示被伪清 |
| F10c（1） | export_double_click_writes_each_target_correctly | `第一次导出的目标文件必须真实落盘（实测被第二次请求的目标路径抢占）` |
| F10d（1） | activated_broadcast_carries_contract_fields | `'mode' not found in {'status': 'activated', 'profile_summary': ...}` |
| 旧端降级连带（1） | old_air_end_without_step_done... | 旧端能力查询同样被 §3.1 污染（caps 协商失败） |

### 3.3 绿（8）——t12 空中侧修复已被正式回归实证

import 文件域回滚（F3）、filename 净化（F3b）、activate 持久化失败不广播假 activated（F4）、双端口坏通道拒绝（F6）、import ack 完整字段（F10a）、close_current 注入不伪报 step_done、旧单端口范围明示+坏文件保留、import 真实 OSError 路径。**这些绿是"期望正确行为"的绿（非诊断绿）**，可作为 t13 复审的修复证据；t11 最终验证将整模块复跑。

---

## 4. 交接说明（t5/t6 审查与自动修复、t11 最终验证）

1. 本模块（`tests/test_round8_calibration_end_to_end.py`）现在同时承载：正向主链绿回归（不得回退）+ 异常契约红回归（修复目标）。**修复回合的验收 = 全模块绿**，任何"以恢复/自愈判达标"的断言都已被替换为契约断言。
2. 修复优先级建议：先修 §3.1（一行初始化，解除 12 用例阻塞）→ 再修 F5（validator dict 尺寸）→ F8/F10b/c/d（地面向导末样本解锁、clear ack `cleared` 字段+显示一致性、export 请求关联、activated 契约字段）→ 复跑本模块至全绿（t11）。
3. `docs/round8_full_regression_utf8.log` 为 t4 全量归档的无损 UTF-8 转存（原 UTF-16LE 文件字节未动）；329.1s/405.8s 口径见 §1.2。t11 全绿后应以同一分块命令重新归档。
4. 本任务未修改任何生产源码；`uav_gpr_air_service.py`/`calibration_service.py` 的变更来自 t12 修复回合（air-engineer），其间的 `_active_osl_id` 回归由本模块正式回归捕获并已报 t13。
