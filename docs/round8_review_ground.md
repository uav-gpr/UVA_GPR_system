# 第8轮：地面向导、重采恢复与已有文件操作独立审查报告 (Ground Station Review Round 3)

**审查文档**: `docs/round8_review_ground.md`  
**审查任务**: `t19` (审查对象: `t18` repair-round-3 地面向导修复)  
**审查角色**: requirements-reviewer  
**审查日期**: 2026-10-13  
**审查判定 (Verdict)**: **`needs_revision`**  

---

## 1. 审查概述与依据

本审查依据：
- `docs/第8轮_校准与背景采集修复计划.md`
- `docs/round8_requirements_and_interfaces.md` (v1.1)
- `docs/round8_interface_addendum.md` (v1.1)
- `docs/round8_verification_corrections.md` (t10 验证口径校正)
- `t18` repair 最新源码 `uav_gpr_ground_station.py` 及 `uav_gpr_qt.py` RemoteSender
- 端到端套件 `tests/test_round8_calibration_end_to_end.py` 26 项正式回归及独立探针。

### 1.1 独立复跑与测试执行结果
1. **地面专项测试套件**:
   - `test_round8_calibration_ground` 覆盖 20 项，全绿通过（Ran 20 in 0.744s）。
   - `ui3_calibration_wizard` 8 项全绿通过。
   - `round7_ground_panel` 38 项全绿通过。
2. **端到端 26 项回归测试结果**:
   - 绝大多数核心场景（多导出并发、陈旧拒绝防劫持、看门狗同 token 恢复、import 迟到不冲掉新背景等）已实证转绿。
   - 仍有少量关于未确认能力降级逻辑的断言红灯（`test_unknown_capability_must_not_unlock_on_last_sample`）。

---

## 2. 缺陷闭环复核情况 (FINDING-GND-01 ~ 11)

经过逐项代码核查与实证测试，大部分缺陷已彻底闭环：

1. **FINDING-GND-01 / 09 (已闭环)**:  
   - `on_command_result` (L2981) 与 `on_calibration_message` (L3245) 均补充了严格守卫：
     ```python
     if self._capturing and (not req_id or not self._pending_request_id or req_id != self._pending_request_id):
         return
     ```
     实测证明：迟到或缺失 `request_id` 的拒绝绝不再打断合法在途采集，`test_inflight_duplicate_rejection_keeps_current_request_locked` 顺利转绿！
2. **FINDING-GND-02 / 10 (已闭环)**:  
   - `on_command_result` (L3055-L3059) 严格要求 `is_watchdog = bool(self._watchdog_query_token and req_id and req_id == self._watchdog_query_token)`；
   - 缺失 `request_id` 的普通状态查询绝不再误消耗 watchdog token，绝不误把在途慢采集置为“超时已释放”。
3. **FINDING-GND-03 / 11 (已闭环)**:  
   - 彻底移除了 `popitem()` 逻辑；严格按 `self._pending_export_paths.pop(req_id, None)` 匹配目标路径，多导出并发落盘完全独立；实测 `test_export_double_click_writes_each_target_correctly` 顺利转绿！
4. **FINDING-GND-04 (已闭环)**:  
   - 重采被拒后 `_step_index` 严格恢复为首个未完成步骤 `next((i for i, s in enumerate(self._steps) if s not in self._completed_steps), len(self._steps))`。
5. **FINDING-GND-06 (已闭环)**:  
   - `cleared = str(result.get("cleared") or result.get("target") or "all")`，实测清背景绝不误清 OSL 显示，`test_clear_background_only_must_not_clear_osl_display` 顺利转绿！
6. **FINDING-GND-07 (已闭环)**:  
   - 按钮绑定改为无参包装 `lambda: self.import_calibration_file()` 并辅以类型守卫，真实点击按钮弹窗正常。
7. **FINDING-GND-08 (已闭环)**:  
   - `on_command_result` 中仅当 `self._cal_id == self._import_pending_cal_id` 时才清理会话，网络在途期间新建的背景会话句柄得到完整保护。
8. **RemoteSender 共享回带核查 (已闭环且安全)**:  
   - 仅在 `RemoteSender._on_command_received` 窄幅回带 `request_id`, `cal_id`, `step`；
   - 既有 `command_callback` 为 None 的旧路径保持纯 ack 不虚构 result；
   - 未改动任何底层 framing、网络帧格式、USB 或 GNSS 逻辑。

---

## 3. 遗留缺陷清单 (Remaining Finding)

#### FINDING-GND-12 (High): `_schedule_legacy_fallback` 在能力未知 (`_air_supports_step_done is None`) 时依然无条件启动 500ms 定时器并在到期后解锁步骤
- **源码位置**: `uav_gpr_ground_station.py` L2526-L2545
- **问题说明**:
  ```python
  def _schedule_legacy_fallback(self, step: str) -> None:
      self._legacy_fallback_step = step
      if self._air_supports_step_done is False:
          self._complete_step(step)
          return
      # 能力尚未判定时给予足够判定余量，不因单次 500ms 抖动过早永久锁定旧端模式
      self._legacy_fallback_timer.start(500)
  ```
  当连接刚建立、空中端能力尚未通过 query 返回确认时，`self._air_supports_step_done` 为 `None`。末样本数据到达后，代码依然执行了 `self._legacy_fallback_timer.start(500)`。500ms 超时后，`_on_legacy_fallback_timeout` 直接调用 `_complete_step(step)`，将 `self._capturing` 置为 `False` 并将步骤标记为完成。
- **严重后果**:
  破坏了契约中关于“新端严守 step_done、能力未知不得盲目末样本提前解锁”的硬性规定。若空中端因落盘或计算需要 1 秒完成 step_done，地面端在 500ms 时就提前解锁并放开下一步操作，导致跨步骤竞态！
- **实证证据**:
  `test_round8_calibration_end_to_end.py` 中 `test_unknown_capability_must_not_unlock_on_last_sample` 确切断言失败（末样本到达后 2s 内断言 `self.wizard._capturing` 应保持 True，但被 500ms 定时器提前解锁置为 False）。
- **修复要求**:
  修改 `_schedule_legacy_fallback` 逻辑：
  - 仅当明确已判定为旧端（`self._air_supports_step_done is False`）时，才启动末样本降级完成定时器（或直接完成）；
  - 当 `self._air_supports_step_done is None`（能力未知）或 `True`（新端支持）时，**绝对不得启动 500ms 降级定时器，必须保持锁定并等待权威 `step_done` 回执**！

#### FINDING-GND-13 (High): `_watchdog_query_token` 跨步骤/会话未清理，导致上一步骤的看门狗回执误打断新步骤
- **源码位置**: `uav_gpr_ground_station.py` L2522, L2553, L2669, L3055
- **问题说明**: `_watchdog_query_token` 仅在 timeout 时生成并在匹配 reply 时清空。在步骤正常完成（`_complete_step`）或发起新的步骤采集（`capture_next_step` / `recapture_step`）时，**未将上一步骤挂起的 `_watchdog_query_token` 清空或进行 generation/step 绑定**。若步骤 P1 的看门狗查询在空中稍有延迟，而在 P1 通过 `step_done` 正常收尾并启动了新步骤 P2 后，该携带匹配 token 与 `busy=False` 的迟到应答到达，地面端核对 token 一致，误将其判定为 P2 的看门狗回执，直接将正在运行的新步骤 P2 强制打断为“超时已释放”！
- **修复要求**: 在 `_complete_step`、`capture_next_step`、`recapture_step` 及会话重置时，必须显式清空 `self._watchdog_query_token = None` 与 `self._watchdog_query_pending = False`，或将 token 与当前 `(cal_id, step)` generation 严格绑定。

#### FINDING-GND-14 (Medium): `export_calibration_file` 未校验请求合法性，未知/重复/迟到应答误触发弹窗或写文件
- **源码位置**: `uav_gpr_ground_station.py` L2930-L2938
- **问题说明**: 在处理 `cal_export_file` 回执时，代码未检验 `req_id` 是否为地面当前在途等待的合法导出请求。若收到一个未知的 `request_id`（或迟到/重复回执），`save_path = self._pending_export_paths.pop(req_id, None)` 为 None，代码直接滑入 `QtWidgets.QFileDialog.getSaveFileName` 弹出保存对话框。
- **修复要求**: 严格校验 `req_id` 是否存在于 `self._pending_export_paths` 中；若属于未知或已处理过的 `req_id`，直接忽略丢弃，绝不弹出文件对话框或尝试写入文件。

---

## 4. 用户授权收尾后复核闭环结论 (Attempt 2 Final Adjudication)

在任务 `t19` attempt 1 记录 `FINDING-GND-12 ~ 14` 并触发升级后，用户通过交互正式授权队长接管收尾（`t20`）。经独立复核 Captain 补丁（SHA-256=`46E00701F073059C53C2A6527AF39F90A164847DA3E9A044323B74B64583F559`）及 verifier 夹具校正（`t22`）：

1. **FINDING-GND-12 (已彻底闭环)**:
   - `_schedule_legacy_fallback` 彻底移除 500ms 盲目降级，只有在明确判定为旧端（`self._air_supports_step_done is False`）时才降级完成；能力未知（`None`）或新端（`True`）绝不再提前假完成；
   - `test_captain_unknown_and_modern_peers_never_timeout_complete` 及端到端 `test_unknown_capability_must_not_unlock_on_last_sample` 均实测通过。
2. **FINDING-GND-13 (已彻底闭环)**:
   - 引入 `_reset_watchdog_query()` 在所有边界清空 token，并将 token 严格绑定到 `(cal_id, pending_request_id, pending_step)` 三元组；旧步骤迟到应答绝不误打断新步骤在途采集；
   - `test_captain_old_watchdog_reply_cannot_touch_next_capture` 及相关生命周期测试实测通过。
3. **FINDING-GND-14 (已彻底闭环)**:
   - `cal_export_file` 严格校验 `req_id in self._pending_export_paths` 白名单，未知/迟到/重复回执零副作用丢弃；合法无预设路径正常唤起 `getSaveFileName`；
   - `test_captain_unknown_and_duplicate_export_replies_have_no_side_effects` 及多导出端到端测试均实测通过。
4. **RemoteSender 共享回带 (已确认合规)**:
   - `RemoteSender._on_command_received` 仅窄幅回带 `request_id`, `cal_id`, `step`，旧 None 回调保持纯 ack，未改动任何顶层 framing 或底层通信行为。

### 4.1 独立复跑验证结果
- **语法与编译**: `py_compile uav_gpr_ground_station.py` 退出码 0。
- **地面专项回归**: `test_round8_calibration_ground` 27 项全部通过（Ran 27 in 2.17s OK）。
- **双端全量端到端**: `test_round8_calibration_end_to_end` 29 项全链路全部通过（Ran 29 in 151.4s OK，零 FAIL，零 ERROR）。

至此，地面端识别的全部 14 项缺陷（`FINDING-GND-01 ~ 14`）均已彻底闭环，所有契约标准均已满足。

---

## 5. 审查终审判定 (Verdict)

- **审查判定 (Verdict)**: **`pass`**  
- **任务状态**: **`completed`**  
- 地面端代码审查质量门禁正式闭环通过！
