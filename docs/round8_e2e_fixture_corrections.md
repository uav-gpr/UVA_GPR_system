# 第8轮 E2E 时序夹具校正说明（docs/round8_e2e_fixture_corrections.md）

**执行角色**: verifier（t22）
**日期**: 2026-10-14
**对象**: `tests/test_round8_calibration_end_to_end.py`（t22 前 26 项 → t22 后 **28 项**，方法只增不减）
**约束遵守**: 仅修 3 处同步/观察夹具 + 本说明；**全部 26 个既有方法与产品断言保留**，无删/skip/expectedFailure；不断言 bug 仍存在；生产源码（Captain 已冻结）与 UI 单测零改动；无 DummyReceiver/成功 header。

---

## 1. 三处夹具校正（产品断言不缩水）

### 1.1 `test_unknown_capability_must_not_unlock_on_last_sample`（F8）

- **原夹具问题**：末样本到达后固定等待 2 秒再断言 `_capturing=True`。修复后（GND-12）产品在末样本保持锁定并等待权威 `step_done`，而真实 `step_done` 在亚秒级到达并完成收尾——2 秒观察窗内"要求仍锁定"与正确产品行为矛盾。
- **校正**：用**自己服务实例的真实消息发送层桩**（`service._send_calibration_message` 实例属性包装，与既有旧端仿真同模式）把权威 `step_done` 帧暂存（不改生产、不删原帧）；在"末样本已到达 + 权威收尾被暂存"窗口内断言**不得解锁**（`_capturing=True`、未计入完成集合、行不回退待采集）；随后释放原帧走**真实发送层（enqueue → 真实 TCP）**，断言权威完成能解锁（`step_done` 收尾判据保留）。
- **产品断言**：全部保留（F8 契约 + 权威收尾恢复）。

### 1.2 `test_real_v2_import_export_query_restart_and_correction`（F13/重启恢复）

- **原夹具问题**：`send_command("start", {"single": True})` 后**立即**读 `acquisition.active_worker`——start 是经真实 TCP 的异步命令，尚未被空中端处理，读到的必然是 None，与产品正确性无关。
- **校正**：改为**有界连续采集窗口**（`start {"count": 3}`），以 5ms 间隔有界轮询真实 `active_worker` 捕获运行中的 worker，在其运行窗口内断言 `through_measurement` 为导入的 Through（F13 接线）；随后等待 trace 落盘做 raw/display 数值校正断言，最后真实 `stop` 并有界等待服务回空闲/待命（停止排空）。
- **产品断言**：全部保留（F13 接线、重启恢复 201 点双端口、display==恢复校准逐点反算、与 RAW 可区分、原件哈希不变）。**诚实边界**：through 对 S21/S12 的真实数值校正需真机，无硬件不可验证（仅验接线）。

### 1.3 `test_cross_session_and_stale_commands_cannot_hijack_active_capture`（F1/F2 跨会话）

- **原夹具问题**：把在途行硬编码为 `"请求采集中…"`；真实慢步进窗口内合法采样进度帧会把行更新为 `"采集中 1/2"`——为无关正常进度失败。
- **校正**：以**真实当前请求/会话**为依据——记录当前 `request_id`，陈旧拒绝后断言：`_capturing` 保持 True、`_pending_request_id` 不变、行处于**合法在途状态**（`请求采集中…` 或 `采集中 i/2` 真实进度），严禁回退 `"待采集"`、备注列不得残留拒绝；恢复判据（真实 step_done 收尾、指针 P1_Short）保留。
- **产品断言**：全部保留（不解锁、不回退、不重复执行、正确收尾）。

---

## 2. 补充方法（只增不减）

- `test_real_import_button_click_opens_dialog_and_imports`：点击**真实 `import_button` 实例**（GND-07 lambda 绑定 + 类型守卫），文件框为 OS UI 故以测试侧桩替换 `QFileDialog.getOpenFileName` 返回真实文件路径；导入链路（内容传输/空中校验落盘激活/ack）全真实，无 DummyReceiver/成功 header。
- `test_real_export_button_click_opens_dialog_and_exports`：点击**真实 `export_button` 实例**；注意连接态下保存框在 **ack 到达时**弹出，桩覆盖"点击→ack→对话框"整个真实窗口；断言导出文件与原件逐字段等价。
- **GND13/14 未补充**：其审查发现内容未随依赖结果下发给本角色，为避免凭空编造测试场景，本模块暂不添加；待该发现内容可见后按同规则补充（方法只增不减）。

---

## 3. 实跑记录（精确命令与结果；时间戳/哈希以实际测试源码为准）

测试源文件 SHA-256：`C45BD128079B46AD92990F55B53752822B6DBB909B5DC52A91E13684A9CCCCB5`
（任何后续修改都会改变该值；t11 应以同源文件为准。）

| 命令 | 结果 |
|---|---|
| `$env:QT_QPA_PLATFORM='offscreen'; $env:QTWEBENGINE_DISABLE_SANDBOX='1'; .venv312\Scripts\python.exe -m unittest tests.test_round8_calibration_end_to_end -v` | **Ran 28 tests in 133.020s — OK**（28/28 全绿，无 FAIL/ERROR） |
| `$env:QT_QPA_PLATFORM='offscreen'; $env:QTWEBENGINE_DISABLE_SANDBOX='1'; .venv312\Scripts\python.exe docs\run_tests_batched.py --only round8_calibration_end_to_end` | `[01/1] OK test_round8_calibration_end_to_end: Ran 28 tests in 132.700s` → `BATCHED TOTAL: Ran 28 tests in 133.6s over 1 modules; failed modules: none` → `OVERALL: OK` |
| 复跑（稳定性确认，同命令同源） | `Ran 28 tests in 132.849s` → `OVERALL: OK` |

**真实产品失败**：最终两次完整运行无任何真实产品失败（28/28 绿）；开发过程中的 1 次失败为导出按钮夹具自身时序（桩在 ack 前被还原），已按 §2 校正。

---

## 4. 交接说明

- **t21/t6**：可读取本模块与本说明；产品断言未缩水，26 个既有方法原样保留（3 处仅同步/观察夹具变化），新增 2 个按钮实例方法。
- **t11**：以本文件记录的源哈希对应模块跑同源最终全量；E2E 模块单跑约 133s，全量预计较 t4 归档（405.8s）增加约 50–60s。
- 生产源码由 Captain 冻结；本轮零生产改动。
