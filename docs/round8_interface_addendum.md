# 第8轮：双端接口契约与需求规范补遗说明 (Addendum v1.1)

**文档标识**: `docs/round8_interface_addendum.md`  
**关联主规范**: `docs/round8_requirements_and_interfaces.md` (升级至 v1.1)  
**编写角色**: requirements-reviewer  
**编写日期**: 2026-10-13  
**状态**: 定稿生效，供 `air-engineer`、`ground-engineer` 及全体验证角色共同遵循。

---

## 1. 补遗背景与总体说明

在任务 t1 定稿初版规范后，结合双端实际通信层实现、共享核心代码结构以及真实扫频物理约束，发现初版存在若干术语不严谨、边界约束不充分及测试契约命名偏差。本补遗针对性更正 10 项核心问题，给出确切唯一的字段格式、状态结构与竞态防护机制，确保双端工程师在并行开发时具备完全一致的代码与测试依据。

---

## 2. 十项核心修订要点与技术细节

### 修订 1：通信消息类型与回程通道修正 (§3.1)
- **修订原因**: 初版文档将同步命令应答混称 `ack`，并漏记了 `type="calibration_quality"`，造成回程类型被误认为受限。
- **确切契约**:
  1. **命令同步回执**: 统一使用底层 `RemoteSender` 实际生成的 `type="command_ack"`，载荷必含 `action: str`, `supported: bool`, `result: dict`。
  2. **异步回程保留**: 完整保留既有的 `type="calibration_quality"`（质量指标推送）、`type="calibration_capture"`（NPZ 样本流）与 `type="calibration_result"`（状态机事件广播）。严禁废弃或拦截 `calibration_quality`。

---

### 修订 2：共享 API 名称更正与测试路径规范 (§4.2, §5.2)
- **修订原因**: 初版文档将 `CalibrationSession.capture(...)` 误写为示例名 `capture_step(...)`，且测试文件名称未遵循任务分配的契约路径。
- **确切契约**:
  1. **核心 API 命名**: `CalibrationSession` 的标准步骤采集方法名称为 `capture(step_name: str, *, recapture: bool = False, on_capture: Callable = None) -> dict`。
  2. **契约测试文件命名**: 必须严格遵循任务分工所划定的测试路径：
     - 空中与核心逻辑测试: `tests/test_round8_calibration_core.py` (t2 契约)
     - 地面端与向导测试: `tests/test_round8_calibration_ground.py` (t3 契约，ground-engineer 所有)
     - 双端联合全流程端到端测试: `tests/test_round8_calibration_end_to_end.py` (t4 契约，verifier 独占；两位工程师不得修改)
     严禁要求或引入未经授权的其他命名测试文件。

---

### 修订 3：`step_done` 语义、USB 句柄持久与采集槽竞态约束 (§3.1, §4.1)
- **修订原因**: 需明确 `step_done` 并不等同于硬件销毁；必须防范执行线程 join 自身及迟到 finally 覆盖新 slot 的竞态。
- **确切契约**:
  1. **`step_done` 状态定义**: 表示单步采集 worker 任务顺利执行完毕，采集槽释放并进入 `idle_waiting` 可接单状态。
  2. **USB 硬件连接保持**: 底层常驻 USB 连接句柄保持打开（keep-alive），**绝对不得为了步骤收尾而重置或销毁 USB 硬件**。
  3. **线程与槽位释放安全约束**:
     - 采集工作线程**严禁尝试 join 自身**。
     - 槽位释放必须在锁保护下进行。
     - 在 worker 的 `finally` 清理块中，必须校验当前采集槽中的 `(cal_id, step, request_id)` 与当前任务一致；若槽已被新任务占用，迟到的 `finally` 绝对不得将槽重置为 idle，杜绝跨步骤冲突。

---

### 修订 4：动态看门狗预算、防假死与超时自愈 (§3.3)
- **修订原因**: 初版固定的 30s 超时时间在合法的高次数（如 `repeats=256`）或极窄 IFBW 慢扫频场景下会发生严重误判；超时后直接放开按钮会导致重复并发。
- **确切契约**:
  1. **动态看门狗机制**:
     - 采用滑动窗口重置：每次地面端收到 `calibration_capture` 进度通知时，重置看门狗定时器。
     - 静默无进度超时门限设定为：
       $$\text{Timeout} = \max\left(30.0,\; \text{预估单次扫频耗时} \times 3.0\right)$$
  2. **超时/断线安全恢复流程**:
     - 看门狗触发或 TCP 断线重连后，地面端**严禁盲目释放向导按钮让用户重试**。
     - 地面端必须主动发送 `cal_query_active`，核查 `in_progress_session.busy` 以及正在执行的 `active_request_id`。
     - 若空中端仍处于 `busy=true`，界面保持锁定并提示“空中端仍在执行步骤采集，正在同步进度…”，杜绝向正在工作的空中端并发灌入新采集请求。

---

### 修订 5：`request_id` 幂等性、有限缓存与防重复执行 (§3.3)
- **修订原因**: 防止重复点击或网络重发导致步骤被无意中重复采集。
- **确切契约**:
  1. 空中端建立大小为 128 项的有限 FIFO/LRU 请求结果缓存 `_request_id_cache`。
  2. 记录 `request_id` 及其执行阶段（`in_progress` / `completed` 及对应返回载荷）。
  3. 命中缓存的处理规则：
     - 若目标 `request_id` 正在采集执行中：立即返回 `{"ok": false, "error": "该请求正在执行中，请勿重复发起", "duplicate": true}`。
     - 若目标 `request_id` 已经完成：直接回放历史回执 `result`（附带 `"cached": true`），**绝对不得重新拉起底层采集 worker**。

---

### 修订 6：`cal_query_active` 完整结构、能力协商与旧端降级 (§3.2, §3.3)
- **修订原因**: 需对外暴露会话每一步的实际状态与空中端能力；同时防止新地面端连接旧空中端时因死等 `step_done` 而永久挂起。
- **确切契约**:
  1. **`cal_query_active` 完整字段结构**:
     ```json
     {
       "ok": true,
       "active_osl": {
         "active": true,
         "cal_id": "imported_solt_2port_20260807_202342",
         "filename": "solt_2port_20260807_202342.json",
         "valid_band_hz": [400000000.0, 4000000000.0],
         "ports": ["P1", "P2"],
         "format": "uav_gpr_solt_calibration_v2",
         "points": 201
       },
       "active_background": {
         "active": false,
         "cal_id": null,
         "valid_band_hz": null,
         "points": 0
       },
       "in_progress_session": {
         "active": true,
         "cal_id": "cal_20261013_180000",
         "mode": "osl",
         "busy": false,
         "current_step": "P1_Short",
         "active_request_id": null,
         "steps_status": {
           "P1_Open": "done",
           "P1_Short": "pending",
           "P1_Load": "pending",
           "P2_Open": "pending",
           "P2_Short": "pending",
           "P2_Load": "pending"
         },
         "steps_done": 1,
         "total_steps": 6
       },
       "capabilities": {
         "recapture": true,
         "import_file": true,
         "export_file": true,
         "step_done": true
       }
     }
     ```
  2. **旧端能力协商与平滑降级**:
     - 旧端空中服务收到带 `recapture=true` 的 `cal_capture` 不等同于 `unknown action`，但若旧端因缺乏参数支持拒绝，地面端需给出降级说明。
     - 若空中端 `capabilities.step_done == false`（或握手判定为旧空中端），地面端在接收到末样本（`index >= of`）后，等待 0.5s 平滑收尾缓冲后自动将步骤置为已完成并调用状态查询，**绝不因缺少 `step_done` 导致旧端正常流程挂起**。

---

### 修订 7：补齐 `cal_export_file` 契约与配置原子落盘回滚规则 (§3.2)
- **修订原因**: 初版遗漏了导出校准文件的完整参数；且导入时未定义 `air_config.json` 落盘失败时的回滚行为。
- **确切契约**:
  1. **`cal_export_file` 动作规范**:
     - `action`: `"cal_export_file"`
     - `params`: `{"cal_id": str (可选，默认当前生效), "format": "uav_gpr_solt_calibration_v2", "request_id": str}`
     - 同步回执 `result`: `{"ok": true, "cal_id": "...", "filename": "...", "format": "...", "content": str, "size_bytes": int, "request_id": "..."}`。
     - 单次传输上限约束: 文件内容字符串最大 **10 MB**。
  2. **`cal_import_file` 配置落盘失败强制回滚**:
     - 校验合格后，写入文件并更新 `air_config.json` 的 `active_calibration_path`。
     - **若 `save_config()` 发生任何异常或写盘失败，整笔导入操作必须返回 `ok=false` 失败！**
     - 必须回滚并删除新写入的校准文件；**绝对不得污染空中端原有的同名生效文件，也绝对不得破坏内存中现存的有效校准对象**。

---

### 修订 8：OSL 变更对背景的彻底失效规则 (§4.1)
- **修订原因**: 初版仅在“频段不一致”时才失效背景。但实际上背景去除严格依赖于当前 OSL 的误差项；改变 OSL 意味着校正空间已发生根本改变。
- **确切契约**:
  - **原则**: 只要发生**任何** OSL 身份或校正域变更（包括重新激活新 OSL、导入并生效新校准文件、或清除 OSL），**现有已生效背景一律立即彻底失效并清空**。
  - 地面端向导与状态栏必须明确提示：“OSL 校准已变更，先前背景已失效，请重新采集背景”，严禁跨 OSL 校正域混用旧背景。

---

### 修订 9：`cal_clear_active` 目标边界与消除歧义 (§3.2)
- **修订原因**: 初版初稿正文 L261 描述可能被误解为清除背景时也会清空 `air_config.json` 中的 `active_calibration_path`。
- **确切契约**:
  - `target == "background"`: **仅清除内存中的背景数据 (`_background`, `active_background_id`)，严禁触动 `air_config.json` 中的 `active_calibration_path`！**
  - `target == "osl"`: 清除内存中 OSL 组件，将 `air_config.json` 中的 `active_calibration_path` 置为空串并持久化，同时联动清除现有背景。
  - `target == "all"`: 清除全部生效组件，持久化路径置空并写盘。
  - 界面端无需额外增加复杂清理按钮，仅在向导内部生命周期及必要管理中遵循该精准边界。

---

### 修订 10：测试策略更新与 GNSS 6 项断言受控对齐 (§2.2, §5.2)
- **修订原因**: 基线测试中 2 个模块 6 项失败系用户未提交的 GNSS 架构改造引起，Captain 已受控 amend t2 允许断言对齐，必须明确生产代码不可修改的红线与最终全绿标准。
- **确切契约**:
  1. **生产代码保护红线**:
     - 严禁为了测试全绿而修改 `uav_gpr_air_service.py` 中的 `gnss_baud = 9600` 生产默认值。
     - 严禁修改常驻 `self.gnss.snapshot()` 所有者架构与初始化命令。
  2. **受控测试对齐与记账**:
     - 在 `tests/test_round7_air_service.py` 与 `tests/test_uav_gpr_qt_continuous.py` 中，将过期的旧断言（如期望 115200、旧一次性测试器逻辑、`log gprmc`）对齐到当前生产既有行为。
     - 在 `docs/round8_test_alignment.md` 中透明记录这 6 项断言修改的理由与代码位置。
  3. **验收全绿标准**:
     - 最终回归必须达到基线 39 个模块及全部新增模块实证全绿（零 FAIL、零 ERROR，已知 Windows WebEngine 进程销毁异常单独记账），**不采用永久豁免策略**。
  4. **客观严谨的物理表述**:
     - 软件状态完整与物理测量精度严格区分；文档不凭空声称用户真机校准精度完好，实际物理精度交付现场真机复核清单判定。

---

## 3. 字段与消息汇总速查表 (Cheat Sheet for Engineers)

| 消息类型 (`type`) | 关键动作/状态 (`action` / `status`) | 关键字段要求 | 备注 |
|---|---|---|---|
| `command` | `cal_capture` | `cal_id`, `step`, `recapture: bool`, `request_id: str` | 地面端发起采集/重采 |
| `command_ack` | 应答任何 command | `action`, `supported: bool`, `result: dict` | 同步回执，带 ok 与数据 |
| `calibration_capture` | 每条 sweep 原始数据 | `cal_id`, `step`, `index`, `of`, `channel`, `payload` (npz) | 既有异步频域数据流 |
| `calibration_quality` | 步骤质量评估 | `cal_id`, `step`, `repeat_rms`, `max_abs_err` | 既有步骤质量指标回程 |
| `calibration_result` | `step_done` | `cal_id`, `step`, `request_id`, `next_step`, `steps_done` | 单步采集结束、采集槽释放 |
| `calibration_result` | `activated` | `cal_id`, `mode`, `valid_band_hz`, `active_osl_id` | 校准激活事件 |
| `command` | `cal_import_file` | `filename`, `content` (max 10MB), `activate: bool`, `request_id` | 下发并生效校准文件 |
| `command` | `cal_export_file` | `cal_id` (可选), `format`, `request_id` | 拉取空中端生效校准文件 |
| `command` | `cal_query_active` | `{}` | 查询生效组件、会话与能力 |
| `command` | `cal_clear_active` | `target: "all" \| "osl" \| "background"` | 精准清除目标校准组件 |

---

*（本补遗自发布之日起生效，作为第8轮所有实现与测试验收的权威裁决依据。）*
