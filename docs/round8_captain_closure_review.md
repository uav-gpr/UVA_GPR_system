# 第8轮：用户授权队长收尾独立复审报告 (Review Round 4 - Final Closure)

**审查文档**: `docs/round8_captain_closure_review.md`  
**审查任务**: `t21` (审查对象: `t20` 用户授权队长收尾：修复地面剩余3项异步边界)  
**审查角色**: requirements-reviewer  
**审查日期**: 2026-10-14  
**审查判定 (Verdict)**: **`pass`**  

---

## 1. 审查概述与依据

在第8轮地面端自动质量循环达到 3 轮上限后，用户通过明确交互选择“授权队长接管收尾”。队长在 `t20` 中针对第三轮审查报告 [docs/round8_review_ground.md](docs/round8_review_ground.md) 确认的最后 3 项关键异步缺陷（`GND-12`, `GND-13`, `GND-14`）实施了收尾修复。同时，verifier 在 `t22` 中对端到端时序夹具完成了精确校正，并扩充了真实按钮实例测试。

本审查对 `t20` 最新补丁源码、新增的 6 项地面专用回归测试、`t22` 的 28 项端到端全链路测试及全套通信不变量进行严格的独立验证与裁决。

---

## 2. 核心缺陷 (GND-12, GND-13, GND-14) 闭环核查

### 2.1 GND-12 闭环核查：能力未知/新端绝不通过 500ms 计时假完成
- **源码审查 (`uav_gpr_ground_station.py` L2541-L2562)**:
  - `_schedule_legacy_fallback` 彻底重构：
    ```python
    def _schedule_legacy_fallback(self, step: str) -> None:
        self._cancel_legacy_fallback()
        self._legacy_fallback_step = step
        if self._air_supports_step_done is False:
            self._complete_step(step)
    ```
  - 当能力未知（`_air_supports_step_done is None`）或明确为新端（`True`）时，**绝对不再启动 500ms 降级定时器**，杜绝了单方面末样本提前假结束。
  - `_on_legacy_fallback_timeout` 增加了多重硬性守卫：必须满足 `_air_supports_step_done is False` 且 `_legacy_fallback_step == self._pending_step` 且 `_capturing` 仍为真，防止排队中的旧定时器跨状态污染。
- **实测证据**:
  - `tests/test_round8_calibration_ground.py` 中的 `test_captain_unknown_and_modern_peers_never_timeout_complete` 运行通过；
  - `tests/test_round8_calibration_end_to_end.py` 中的 `test_unknown_capability_must_not_unlock_on_last_sample` 运行通过。

---

### 2.2 GND-13 闭环核查：看门狗 Token 严格绑定当前会话/请求/步骤，跨边界自动失效
- **源码审查 (`uav_gpr_ground_station.py` L2497-L2504, L2533-L2538, L3035-L3055)**:
  - 引入了 `_reset_watchdog_query()`，在 `_start_capture_watchdog`、`_stop_capture_watchdog`、`_complete_step` 以及会话重置时均强制调用，清除任何在途未决的 token。
  - 看门狗触发查询时，将 token 与三元组上下文绑定：
    ```python
    self._watchdog_query_context = (
        self._cal_id, self._pending_request_id, self._pending_step
    )
    ```
  - `on_command_result` 中处理 `cal_query_active` 时，不仅严格校验 `req_id == self._watchdog_query_token`，还强制比对 `self._watchdog_query_context == (self._cal_id, self._pending_request_id, self._pending_step)`。
  - 迟到的旧步骤应答（即使 token 偶然相同或为空），只要与当前正在运行的步骤上下文不匹配，一律被丢弃，绝不会强行把新步骤打断为“超时已释放”。
- **实测证据**:
  - `test_captain_old_watchdog_reply_cannot_touch_next_capture` 运行通过（旧 P1 应答无法打断随后的 P1_Short 步骤）；
  - `test_captain_query_context_retires_on_recapture_and_session_boundaries` 运行通过。

---

### 2.3 GND-14 闭环核查：Pending 导出 ID 白名单机制，杜绝窃取路径与误弹窗
- **源码审查 (`uav_gpr_ground_station.py` L2921-L2935)**:
  - 彻底删除了错误的 `popitem()` 盲猜逻辑。
  - 处理 `cal_export_file` 时，只有 `req_id` 存在且包含在 `self._pending_export_paths` 中时才会被消费：
    ```python
    if not req_id or req_id not in self._pending_export_paths:
        return
    save_path = self._pending_export_paths.pop(req_id)
    ```
  - 未知 ID、重复回执或已消耗回执被直接丢弃，绝不引发任何弹窗、写文件或状态篡改等副作用。
  - 合法未指定路径（`save_path is None`）正常唤起 `QFileDialog.getSaveFileName`，指定路径的并发多导出各写各的目标文件，互不干扰。
- **实测证据**:
  - `test_captain_unknown_and_duplicate_export_replies_have_no_side_effects` 运行通过；
  - `test_captain_known_export_without_path_uses_only_its_chooser` 运行通过；
  - `test_export_double_click_writes_each_target_correctly` 在双端真实链路上运行通过。

---

## 3. 既有功能与通信不变量保持

1. **真实 Qt 按钮点击全链验证**:
   - `import_button` 与 `export_button` 采用无参 lambda 绑定并辅以类型守卫，真实点击事件与文件选择框弹窗逻辑完全正常，已由专项单测与端到端用例全面覆盖。
2. **协议与帧格式不变量**:
   - `RemoteSender` 仅在 `command_ack` 入口窄幅回带 `request_id`, `cal_id`, `step` 关联键，旧 `None` 回调保持纯 ack，未改动任何顶层 `type`、数据帧结构、出站锁、run_id 词汇表或 GNSS 串口逻辑。
3. **工作区合规性**:
   - 修改范围严格限制在 `uav_gpr_ground_station.py`、`tests/test_round8_calibration_ground.py` 及相关文档内；
   - 用户未提交的 GNSS 改造代码及实测校准文件完好无损，未删除任何测试用例，未引入任何 skip 或 expectedFailure 假绿手段。

---

## 4. 独立复跑验证命令与记录

在隔离环境中执行契约规定的两组验证命令：

1. **编译检查**:
   ```powershell
   .venv312\Scripts\python.exe -m py_compile uav_gpr_ground_station.py tests\test_round8_calibration_ground.py
   ```
   - 退出码: `0`（无语法或编译错误）。

2. **地面与 UI 模块分块回归**:
   ```powershell
   .venv312\Scripts\python.exe docs\run_tests_batched.py --only round8_calibration_ground ui3_calibration_wizard round7_ground_panel
   ```
   - 结果: 3 个模块共 73 项测试全部通过（Ran 73 in 5.7s, failed modules: none, exit code: 0）。

3. **双端全流程端到端回归**:
   ```powershell
   .venv312\Scripts\python.exe docs\run_tests_batched.py --only round8_calibration_end_to_end
   ```
   - 结果: 28 项全链路测试全部通过（Ran 28 tests in 133.1s over 1 modules; failed modules: none, exit code: 0）。

---

## 5. 审查结论与判定

经过第四轮严密复查与实证测试，用户授权队长实施的收尾补丁确实彻底闭合了 `GND-12`, `GND-13`, `GND-14` 全部遗留缺陷，修复逻辑精简严谨，边界守卫完备，全链路测试实证全绿。

- **审查判定 (Verdict)**: **`pass`**  
- **任务状态**: **`completed`**  
- 地面端修复全部闭环，正式移交 verifier（`t11`）执行最终同源全量回归与便携打包！
