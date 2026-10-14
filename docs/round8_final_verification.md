# 第8轮 最终独立验证报告（t11 · 打包前同源验证）

**执行角色**: verifier（独立验证）
**日期**: 2026-10-14
**验证对象**: 全部审查/修复回合（t12/t14 空中三轮、t16/t18 地面两轮、t20 队长收尾）通过后的冻结源码
**前置输入**: docs/round8_captain_closure.md、docs/round8_captain_closure_review.md（t21 pass）、docs/round8_e2e_fixture_corrections.md（t22）、docs/round8_verification_corrections.md（t10）

---

## 1. 结论

**最终冻结源码在真实双端命令链与全量回归上全部实证通过：30 项独立 E2E 全绿（含 2 项新增 GND-13/14 真实 wire 场景）+ 42 模块 1191 项分块全量 0 FAIL（OVERALL OK，exit 0）。** 三故障正向主链、t10 升级的 13 项契约回归、队长收尾 GND-12/13/14、原子性/身份/Through 接线/请求关联全部有数值/状态/审计证据。**打包门槛（t7）可以放行**；真实 VNA/标准件物理精度仍以现场复核清单为准（§6）。

---

## 2. 验证源与冻结哈希（同源打包依据）

| 文件 | SHA-256 | 备注 |
|---|---|---|
| tests/test_round8_calibration_end_to_end.py | `65AE5396E7105931878B0C5B41C7EA3DAFBBD69BC2B08DDED68216DACAFFEECA` | 本轮最终 E2E（30 方法；t22 版 C45B…之后新增 GND13/14 两方法，按 t22 说明未拘泥旧哈希） |
| uav_gpr_ground_station.py | `46E00701F073059C53C2A6527AF39F90A164847DA3E9A044323B74B64583F559` | = t20 冻结值（逐字一致） |
| uav_gpr_qt.py | `EEF2BE69B83AD4034F4D55705CF619663CDC48479CF1987FE7B79F9352A7E7E3` | = t20 冻结值 |
| uav_gpr_air_service.py | `F10F616328D17D1092DC60E7296E117EA5D8199493651959D2304F4CBC0850AA` | = t20 冻结值 |
| calibration_service.py | `47FD0D4CEAA70302DC11C1F8AB30F50515CD875738EA395345F525ADE14F8060` | = t20 冻结值 |
| tests/test_round8_calibration_ground.py | `ADD3108DB5E69846F03F567A5CD71A23D58449B7BC810F8E39DC99B38355D390` | = t20 冻结值 |
| librevna/librevna_usb_s11.py | `F023E8051A83A24FF932D5F03FB38310C8692D14F01EC0D7BD082F1423CBA4CD` | 未改动 |
| air_gnss.py / ground_gnss.py | `A6C52F9B…` / `BD980AD2…` | 未改动（GNSS 既有行为保持） |
| solt_2port_20260807_202342.json | `14276D5048ED53D2C87C79D8A71F235B3D228BE0F0F6FA86BE42FD20C0D82CB5` | 真实校准原件全流程不变（E2E 内亦有断言） |

py_compile（tests/test_round8_calibration_end_to_end.py）：exit 0。

---

## 3. 实跑命令与结果（精确）

| # | 命令 | 结果 |
|---|---|---|
| 1 | `& .venv312\Scripts\python.exe docs\run_tests_batched.py --only round8_calibration_end_to_end` | `[01/1] OK test_round8_calibration_end_to_end: Ran 30 tests in 154.562s` → `BATCHED TOTAL: Ran 30 tests in 155.4s over 1 modules; failed modules: none` → `OVERALL: OK` |
| 2 | `& .venv312\Scripts\python.exe docs\run_tests_batched.py` | 42 模块全部行见 `docs/round8_final_regression_utf8.log`（UTF-8，read 工具已验证可读）：`BATCHED TOTAL: Ran 1191 tests in 449.4s over 42 modules; failed modules: none` → `shutdown-crash (environment drift, tests OK): test_round1_review_fixes (exit 3221225477)` → `OVERALL: OK`（exit 0） |
| 3 | `& .venv312\Scripts\python.exe -m unittest tests.test_round8_calibration_end_to_end -v`（前置于归档的等价直跑） | `Ran 30 tests in 155.428s — OK` |

无 NO-SUMMARY、无超时；唯一 `PASS*` 为 test_round1_review_fixes 的 Qt WebEngine 拆除段异常（0xC0000005，35 项测试本身 OK），按基线既有口径**独立记账**，不计为通过掩盖。

---

## 4. 基线与新增覆盖透明对账（无删/skip/expectedFailure）

- **基线 39 模块 / 1123 项**：逐模块测试数与 `docs/round8_baseline.log` 完全一致（27/6/13/8/11/3/127/9/19/6/64/13/81/26/72/28/11/4/14/35/16/10/73/37/31/16/9/11/15/60/38/23/102/51/11/4/8/18/13），**零删改减项**。
- **新增覆盖 +68**：round8_calibration_core 11（t2）+ round8_calibration_ground **27**（t3 的 21 + t20 队长收尾 6，未删方法）+ round8_calibration_end_to_end **30**（t4 17 → t10 26 → t22 28 → **t11 30**，仅增不减）。
- **对账**：1123 + 68 = **1191** 与归档逐字一致。E2E 文件经 grep 核实**无** skip/expectedFailure/xfail。

---

## 5. 缺口闭合证据映射（13 项旧缺口 + 队长收尾 3 项）

| 缺口 | 闭合证据（E2E 方法 → 实测要点） |
|---|---|
| F1 陈旧 query 回执释放 | `test_stale_query_ack_does_not_hijack_inflight_capture`：step_done 暂存桩期间不解锁、真实 TCP 释放后权威解锁；配合 GND13 用例（陈旧 token 回执零副作用） |
| F2 ack 缺请求关联 | `test_inflight_duplicate_rejection_keeps_current_request_locked`：在途重复命令拒绝后 `_capturing`/request_id/行保持（GND-09 修复实证） |
| F3 同名导入覆盖旧文件 | `test_import_config_save_failure_preserves_old_file_bytes_and_memory`：save_config 失败整笔失败、旧文件字节/配置/内存三不变、无半套残留（AIR-05/06/12 实证） |
| F3b filename 逃逸 | `test_air_import_filename_cannot_escape_imports_dir`：`..` 路径被净化/拒绝，无逃逸文件 |
| F4 activate 假成功 | `test_activate_persistence_failure_must_not_broadcast_activated`：持久化失败（真实 OSError）不 ok=true 不广播假 activated（AIR-13 回滚实证） |
| F5 dict 绕 10MB | `test_validator_enforces_10mb_for_dict_and_string_alike`：dict 与 str 同限（AIR-11 UTF-8 字节门控实证） |
| F6 坏双端口静默降级 | `test_corrupt_dual_port_channel_data_must_be_rejected`：channels.S22=NaN 整笔拒绝、内存不生效（AIR-08 实证） |
| F8 未知能力末样本解锁 | `test_unknown_capability_must_not_unlock_on_last_sample`：自己实例发送层桩暂存权威 step_done → 暂存期间不解锁 → 真实 TCP 释放后权威解锁（GND-12 实证） |
| F9 重采被拒指针/下一步 | `test_pending_step_recapture_rejection_restores_pointer` + `test_recapture_rejected_then_normal_next_step_is_p1_short`：拒绝后指针恢复 P1_Open/普通下一步仍 P1_Short（GND-04/09 实证） |
| F10a import ack 字段 | `test_import_ack_and_query_report_consistent_complete_fields`：ack 带 format/ports/points/valid_band_hz 且与 query 一致（AIR-04 实证） |
| F10b 仅清背景伪清 OSL | `test_clear_background_only_must_not_clear_osl_display`：ack 带 cleared、地面仅清背景、空中 OSL 保持（AIR-03+GND-06 实证） |
| F10c 并发导出串单 | `test_export_double_click_writes_each_target_correctly` + `test_export_replay_and_unknown_ack_have_zero_side_effects`：两目标各自落盘；重放/未知 ack 保存框 0 次、字节不变、合法 pending 不被消耗（GND-03/11/14 实证） |
| F10d activated 字段 | `test_activated_broadcast_carries_contract_fields`：mode/ports/points/valid_band_hz/active_osl_id 齐全（AIR-13 实证） |
| F11 背景覆盖 OSL 身份 | `test_osl_activate_background_and_calibrated_trace`：背景激活后 query.active_osl.cal_id 仍为 OSL 身份、trace 元数据 calibration_id=OSL 身份 ≠ 背景身份（AIR-01 实证） |
| F13 Through 未接入 spec | `test_real_v2_import_export_query_restart_and_correction`：有界连续采集窗口捕获运行 worker，`through_measurement` 为导入 Through；重启恢复后 display==恢复校准逐点反算（AIR-02 实证；S21/S12 真机数值校正诚实注明不可无硬件验证） |
| close_current 假收尾 | `test_close_current_failure_must_not_fake_step_done`：注入失败后无 step_done 假广播（AIR-09 实证） |
| **GND-12** 能力未知假完成 | `test_unknown_capability_must_not_unlock_on_last_sample`（同 F8；None 保持锁定等权威收尾） |
| **GND-13** 旧查询回执跨界 | `test_stale_watchdog_reply_cannot_unlock_new_capture`（**本轮新增**）：P1_Open 在途触发真实看门狗查询（token/上下文=`(cal_id,req_open,P1_Open)`）→ 自己实例命令回调桩延迟该查询 → P1_Open 权威 step_done（token 生命周期失效断言）→ 启动 P1_Short 新请求 → 释放延迟查询（真实 handler 快照 + 生产窄关联 request_id 回带，经 enqueue_calibration 真实 TCP）→ 断言新采集/请求/行/新 token 不变、无拒绝痕迹 → 当前新看门狗查询（token2）获真实回执保持等待 → P1_Short 真实完成推进 P1_Load |
| **GND-14** 导出回执白名单 | `test_export_replay_and_unknown_ack_have_zero_side_effects`（**本轮新增**）：两次真实导出消耗各自 pending ID → 原样重放旧 ack 帧 + 未知 ID 真实格式 ack（真实内容）→ 保存框 0 次调用、既有文件字节不变、无副作用文件 → 合法导出仍正常 → 合法无预设路径经真实按钮+测试侧保存框桩完整落盘 |

---

## 6. 边界声明（诚实边界）

- **已验证**：双端协议/状态机/竞态守卫/持久化/恢复/幂等/能力协商/降级/数值管线（OSL 校正、背景扣除、双端口恢复、带外拒绝、审计证据、Through 接线），全部在真实 TCP wire + 真实按钮/线程/管线上以模拟设备与可控标准件完成。
- **不可验证（需真机）**：真实 VNA/标准件测量精度、真实 USB 时序、真实 GNSS/串口、Through 对 S21/S12 的实际数值校正。现场复核清单见 t4 报告 §6.2，随包交付。
- **未做**：不改生产、不改用户数据、不部署、不注册自启任务、不打包（t7/t8 职责）。

---

## 7. 后台作业

- pwsh-118（t4 全量，已完成）、pwsh-130（t4 最终全量，已完成）、pwsh-272（队长原样 E2E，队长已收集）、**pwsh-290（本轮最终全量，已完成，exit 0）**——全部已收集，无遗留运行作业。
