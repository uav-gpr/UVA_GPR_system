# UAV-GPR 第 2 轮：需求与接口契约

日期：2026-09-04　作者：engA-resident-service（t1）
状态：**本文档是第 2 轮三条工作流的唯一需求依据**。实现（t3/t5/t7）、评审（t4/t6/t8）、终验（t9）与终报（t11）中出现任何理解分歧，以本文档为准；本文档未覆盖的新需求必须先补充本文档再实现。

---

## 0. 阅读方式与编号约定

- 条款编号：`R-A-nn`（工作流 A：常驻采集服务）、`R-B-nn`（工作流 B：磁盘 outbox）、`R-C-nn`（工作流 C：空中端 UI 精简 + 地面校准）、`R-X-nn`（跨工作流约束）。
- 每条条款都必须可判定：标注验证手段为 `[自动]`（存在对应单测/基准脚本）、`[人工]`（读代码/读日志核查）之一。**不允许出现无法验证的愿望式表述。**
- `[冻结]` 标记：该行为是第 1 轮已经评审通过、第 2 轮任何人不得改变的语义（对应既有回归测试）。
- 需求来源：全部取自第 1 轮三份报告中**明确写出的"留给第 2 轮"条目**（完成报告 §11 磁盘 outbox 未实现声明、§17 已知限制 1/2、§18 第 2 轮建议；复审修复报告 §2 一 RemoteSender 真实排空；复审第二次定向修复报告 §2 五 failed_item 终态），未另起新需求。

## 1. 基线与铁律

基线：分支 `my-modifications`，HEAD `9027366`（t0 存档）；全量测试实证基线 **732** 条 OK（`.venv312\Scripts\python.exe`，Python 3.12.14，`QT_QPA_PLATFORM=offscreen` + `QTWEBENGINE_DISABLE_SANDBOX=1`，分块运行器）。注：第 1 轮报告写的"716"是第二次修复报告时的陈旧叙事数，**732 = t0 归档树实证**（captain 在 t0 worktree 分块跑 732/22 模块 OK + reviewer AST 加载模拟 732 + engA 批跑 732 三方互证）。

| 编号 | 铁律 | 验证 |
|---|---|---|
| R-X-01 | 禁止任何 `git commit / push / reset / checkout / restore / clean / stash / branch` 写操作。修改只停留在工作区。 | `[人工]` verifier 用 `git log`（HEAD 仍为 9027366）与 `git reflog` 核查 |
| R-X-02 | 禁止修改 `uav-gpr-telemetry-v1` framing 的既有字段与既有消息语义：trace header（`type/protocol/run_id/trace_index/timestamp/payload_format/payload_bytes/sha256/config/processing/gnss/channels`，及 acquisition 存在时追加的 `acquisition/acquired_at_utc/flight`）、`ack_trace`（`type/protocol/run_id/trace_index/path`，duplicate 时 `duplicate: true`）、`hello`/`ack`、`command`/`command_ack`、`acquisition_status`（state 取值沿用第 1 轮 `stopping/stopped/drain_failed` + 本轮补齐发送方的 `acquiring` + 新增 `outbox_alert`；未知 state 一律落"采集状态：未知状态"分支 `[冻结]`）。允许**新增空中→地面**消息 `type`，走"旧接收端忽略"先例（`ReceiverThread.handle_client` else 分支 → `Ignored message`）。**但严禁新增"地面→空中"的消息 `type`**：空中 `RemoteSender._run`（`uav_gpr_qt.py:1586`）与 `_wait_for_trace_ack`（`:1564`）对等 ack 期间收到的非 `command` 帧一律 `raise ConnectionError` → 踢下线 + 无限重连（§3.7 同类死锁）；地面→空中新语义只能用 `type="command"` 承载新 `action` 值（详见 §5.4）。 | `[自动]` framing 既有断言全绿（`test_uav_gpr_qt_pipeline` / `test_ground_trace_pipeline` / `test_round1_*`）+ `[人工]` diff 审查 |
| R-X-03 | `reference_code/`、`dual_channel_reference/`、`calibration_reference/`、`数据处理函数/` 与第 1 轮三份报告只读，不修改、不移动。 | `[人工]` git status 中无这些路径的改动 |
| R-X-04 | 绝不 `terminate()` / `kill` 活线程；所有线程收尾一律"请求停止 + 有界等待 + 未退出则报告并保持"。 | `[自动]` 既有 `CloseOrderTests` / `test_no_thread_residual_after_close` 保持全绿 `[冻结]`；新增代码 rg 不得出现 `.terminate(` |
| R-X-05 | 计数不谎报：任何 `selected/processed/saved/sent/acked/committed/outstanding/pending_files` 计数必须来自运行对象的真实统计；失败路径绝不报告成功终态。 | `[自动]` 第 1 轮全部对账测试 `[冻结]` + 第 2 轮新增条款各自的对账测试 |
| R-X-06 | 新增生产代码统一落盘方式：临时文件 + `os.replace` 原子改名（需要原子性的场合）；崩溃/断电后遗留的 `*.tmp` 一律按未完成处理。 | `[自动]` R-B-03/R-C-15 对应测试 |

## 2. 三条工作流总览与共享文件写入次序

| 工作流 | 负责人 | 任务卡 | 触碰文件（写入权） |
|---|---|---|---|
| A 常驻采集服务 | engA | t3/t4 | `uav_gpr_qt.py`、`librevna/librevna_continuous.py`、`librevna/librevna_transport.py`、`tests/test_round2_resident_service.py` |
| B 磁盘 outbox | engB | t5/t6 | **新增** `disk_outbox.py`、`uav_gpr_qt.py`（仅 RemoteSender/接线区域）、`tests/test_round2_disk_outbox.py` |
| C UI 精简 + 地面校准 | engC | t7/t8 | `uav_gpr_qt.py`（仅 MainWindow/calibration_service 接线）、`uav_gpr_ground_station.py`、**新增** `calibration_service.py`、`tests/test_round2_air_ui_compact.py`、`tests/test_round2_ground_calibration.py` |

**`uav_gpr_qt.py` 串行改动次序（互不覆盖的硬规定）**：

1. **第 1 步（t3，engA）**：常驻服务改造。完成后 `ContinuousAcquisitionWorker` 对外只保留服务命令入口（§3），worker 构造签名变化**只影响 uav_gpr_qt.py 内部**（MainWindow 与测试）；`RemoteSender` 公开方法保持**向后兼容签名**（`stop/outstanding_count/start` 不变；`enqueue_trace/enqueue_status` 只允许新增**带默认值的关键字参数**——t3 加 `run_id=None`（§3.7），旧调用点与既有测试逐字可用 `[冻结]`）。
2. **第 2 步（t5，engB）**：在 t3 成果之上加 `disk_outbox.py` 与 `RemoteSender` 接线。允许修改 `RemoteSender.__init__`（新增关键字参数，带默认值保持旧调用兼容）与 `_run`/`_note_*` 内部；**不允许**改动 t3 的服务/会话层代码与 §3 的命令入口。
3. **第 3 步（t7，engC）**：UI 精简 + 地面校准。允许修改 `MainWindow`、把 `MainWindow` 内的校准方法**搬家**到 `calibration_service.py`（见 R-C-09 迁移映射），以及在 `GroundStationWindow` 新增校准入口；对服务层与 outbox 只允许**接线级**修改（调用既有公开 API，不改其内部）。

依据任务卡依赖：t3 →（t5）→（t7），评审门 t4/t6/t8 串行把关。

---

## 3. 工作流 A：常驻采集服务（engA / t3）

### 3.0 需求来源与目标

第 1 轮完成报告 §18："建议第2轮：常驻采集服务（停止后 SET_IDLE 保持 USB）时把'受控停止'从 worker 事件升级为服务命令"；参考文档《钢筋仪项目_S11_S22_采集与校准参考》§1/§3.1/§6-1（"采集必须采用持久 USB 会话"）。第 1 轮虽然实现了 `PersistentLibreVnaBackend`（open/configure 各一次 + pause/resume），但 `ContinuousAcquisitionWorker.run()` 的 finally 仍走 `_shutdown → _release_backend`，**每个任务结束都释放 USB**；下一个任务重新 open+configure，握手/claim 重建开销重复发生，且真机上 USB 重开是最高风险动作。

**目标**：任务（task）与会话（USB session）解耦。一个进程内的 USB 会话常驻：首次启动任务时 open/configure；任务停止时只发 `SET_IDLE` 进入"待命"（设备停流、USB 保持 claim、不 release_interface/dispose_resources）；只有程序关闭（或 FAULTED）才真正释放。所有停止路径——操作员按钮、地面端 `command(stop)`、处理队列硬上限受控停止——统一为一个服务命令入口的同一语义。

**非目标**：不改 legacy `AcquisitionWorker`（feature flag `UAV_GPR_ACQUISITION_MODE` 默认 legacy 的路径保持逐条 open/close 行为不变 `[冻结]`）；不实现磁盘 outbox（B）；不动地面端；不做真机验证。

> **第4轮更新（2026-09，采集速度修复）**：本条 `[冻结]` 的**对象**是 legacy worker
> 的逐帧 open/close 行为本身——它仍然不变（代码原样保留，作为调试退路）。
> 但其中"**默认 legacy**"这一句已被第4轮凭据取代：真机实测 legacy 约 2.75 s/帧，
> 故 `DEFAULT_ACQUISITION_MODE` 改为 `"continuous"`（`uav_gpr_qt.py:253`），
> legacy 只能显式选（env=legacy 或界面"采集引擎"下拉）。
> 详见 `docs/实时链路与UI_第4轮采集速度修复报告.md`。

### 3.1 服务状态机（契约）

新增类 `AcquisitionService`，落在 `uav_gpr_qt.py`，**纯 Python（非 QObject）+ `threading` 原语，线程安全**，可从 Qt 主线程、worker USB 线程、`RemoteSender` 命令回调线程（经 MainWindow queued 信号转发后）安全调用。

```
CLOSED（无 USB 会话）
  └─ start_task ─→ 打开会话 ─→ (未 configure 过时 backend=OPEN_IDLE)
                                    │ configure / reconfigure
ACQUIRING（任务运行，设备连续流）
  ├─ 任务停止命令 / count 到达 / 队列硬上限受控停止
  │     → STOPPING（USB 线程在完整 sweep 边界停选 → backend.pause()=SET_IDLE
  │        → 设备停流；消费者排空处理队列）
  │     → 排空全部成功 → IDLE_WAITING（USB 保持 claim，backend=PAUSED 或 OPEN_IDLE）
  │     → 任一失败     → FAULTED_RELEASED（按第1轮 fail-closed 释放后 → CLOSED）
  ├─ IDLE_WAITING ─ start_task（配置未变）→ resume（冻结配置）→ ACQUIRING
  ├─ IDLE_WAITING ─ start_task（配置已变）→ reconfigure（新 SWEEP_SETTINGS）→ ACQUIRING
  ├─ 任意态 ─ shutdown(有界超时) → 释放 USB → CLOSED（唯一正常释放路径）
  └─ 设备/协议不可确定（LibreVna*Error、transport.read 断开、pending 溢出、
        NACK、put 超时 failed_item、排空失败）→ FAULTED → 立即 fail-closed 释放
        （复用第1轮 _release_backend 重试循环语义）→ CLOSED
```

`AcquisitionService.state` 取值（字符串，与上表一一对应，日志/UI/测试共用）：
`"closed" | "idle_waiting" | "acquiring" | "stopping" | "shutting_down" | "faulted"`。

**与 backend 状态机的映射**（`librevna/librevna_continuous.BackendState` 不改既有语义）：服务 `IDLE_WAITING` ⇔ backend `PAUSED` 或 `OPEN_IDLE`；`ACQUIRING` ⇔ `STREAMING`；`FAULTED` ⇔ backend `FAULTED`。

### 3.2 服务命令接口（统一入口，契约签名）

```python
class AcquisitionService:
    def __init__(
        self,
        *,
        backend_factory: Callable[[], PersistentLibreVnaBackend] | None = None,
        backend_settings: PersistentBackendSettings | None = None,
        status_callback: Callable[[str], None] | None = None,
    ) -> None: ...
    # —— 统一命令入口（MainWindow、地面端 command、内部受控停止都走这里）——
    def start_task(self, spec: "AcquisitionTaskSpec") -> "ContinuousAcquisitionWorker | None": ...
    def request_stop(self, reason: str | None = None) -> bool: ...
    def request_pause(self) -> bool: ...
    def request_resume(self) -> bool: ...
    def shutdown(self, timeout_ms: int = 10_000) -> bool: ...
    # —— 只读观测 ——
    @property
    def state(self) -> str: ...
    @property
    def backend(self) -> PersistentLibreVnaBackend | None: ...   # 会话对象（含 open_count 等统计）
    @property
    def active_worker(self) -> "ContinuousAcquisitionWorker | None": ...
    @property
    def last_stop_reason(self) -> str | None: ...                 # 最近一次停止命令的 reason（含受控停止）
    def statistics(self) -> dict[str, object]: ...                # JSON 可序列化：state/open_count/configure_count/
                                                                  # trace_index/task_seq/pending_stop_reason…
```

`AcquisitionTaskSpec` 为 frozen dataclass，聚合现在散落的 worker 构造参数（`cfg/interval_s/count/out_dir/gnss_port/baud/calibration*/background*/autosave/remote_sender/queue_maxsize`），并新增三个字段：

- `run_id: str` —— 本任务的发送身份，由 `service.new_run_id()` 签发（格式与规则见 §3.7）；worker 全部遥测消息（trace/acquiring/stopping/stopped/drain_failed）显式携带它。
- `task_seq: int` —— 服务内单调任务序号（从 1 起）。写入 `acquisition` metadata **新 key `"task_seq"`**（只在 NPZ/JSONL 的 acquisition 嵌套 dict 内增加，地面解析器对未知 key 原样透传；**不进 telemetry framing 顶层字段**，R-X-02 边界内）。
- `status_reporting: bool = True` —— `False` 时该任务**不发送** `acquisition_status` 消息（供工作流 C 的校准短任务使用，§5.4）；`True` 保持第 1 轮全部状态消息语义 `[冻结]`。

`request_stop/request_pause/request_resume` 可从任意线程调用；返回 False 仅表示"当前状态不接受该命令"（如无活动任务时 request_pause），**绝不**用返回值伪装停止成功。停止的实际完成通过 `state` 迁移与 `acquisition_status` 消息观测。

### 3.3 逐条验收条款

| 编号 | 条款 | 验证 |
|---|---|---|
| R-A-01 | 会话常驻：首次 `start_task` 惰性 `open()`+`configure()`；任务停止后 backend 进入 PAUSED/OPEN_IDLE 并保持，**不**调用 `close()`、**不** release_interface/dispose_resources（fake transport 断言 `transport.is_open == True`、`close_count == 0`）。 | `[自动]` |
| R-A-02 | 连续两次完整 start/stop（同一服务、配置不变）：`backend.statistics.open_count == 1` 且 `configure_count == 1`；第二次任务经 `resume()`（冻结配置）恢复，测试直接断言 open_count/configure_count。 | `[自动]` |
| R-A-03 | 停止时发送 `SET_IDLE`（即 `backend.pause()`：完整 sweep 边界 + 清边界残留，`_discard_stale_sweep_data` 语义与第 1 轮 pause 一致），fake transport 可观测到停止时序中存在 SET_IDLE 包且其后再无 VNA_DATAPOINT 读取交付。 | `[自动]` |
| R-A-04 | 配置变化的再启动：`start_task` 携带与冻结配置**不同**的 `SweepSettings` 字段（start/stop/points/ifbw/power/dwell 任一）时，走新后端方法 `reconfigure(settings)`（§3.4）：open_count 不变、configure_count+1、新 assembler、边界数据清空、trace_index 与会话计数连续。配置相同则禁止 reconfigure，走 resume。判定依据是 `SweepSettings.encode()` 字节逐字比较（杜绝浮点近似漏判）。 | `[自动]` |
| R-A-05 | 受控停止服务命令化：`LosslessProcessingQueue.on_hard_limit` 回调路径改为 `service.request_stop(reason=...)`；MainWindow 停止按钮、地面端 `command(stop)`（`handle_remote_command`）、队列硬上限三个调用方得到**完全相同**的语义（同一事件、同一 reason 记录、同一状态迁移、同一状态消息序列）。worker 私有 `_stop` 事件删除/降级为 service 停止事件的别名，不再存在绕过服务入口的停止路径（rg 检查生产代码不再出现 `worker.request_stop(` 的独立调用点——`AcquisitionService` 内部对 worker 循环的唤醒除外）。 | `[自动]` + `[人工]` |
| R-A-06 | 停止超时不伪装：`IDLE_WAITING` 迁移必须等 backend.pause() 成功 + 消费者排空完成。若 USB 线程在**有界等待**（`service.wait_idle(timeout_ms)`，默认由调用方给）内未达到 `idle_waiting`，`request_stop`/`shutdown` 不得报告成功：`state` 停留在 `"stopping"`，`last_stop_reason`/statistics 可观测卡点（worker 存活、backend 状态、队列深度、consumer.drain_state），并明确日志。绝不 terminate（R-X-04）。 | `[自动]` |
| R-A-07 | 只有 `shutdown()`（程序关闭）真正释放 USB：closeEvent 调 `service.shutdown(bounded)`；内部复用第 1 轮 `_release_backend` 的 fail-closed 重试语义（资源未释放前不结束、每 5s 报状态、绝不 FAULTED-仍打开 视作成功）；超时/未释放 → closeEvent `event.ignore()`，窗口保留，再次关闭可重试；成功关闭后服务 `state=="closed"`、`transport.is_open==False`、无残留线程。 | `[自动]` |
| R-A-08 | stop→start 边界数据不跨任务拼接：进入 IDLE_WAITING 与再次启动时，`PacketStream` 与 pending 的清理规则与第 1 轮 pause/resume 一致（pause：SET_IDLE+`_discard_stale_sweep_data`；resume：再清一次后发 SWEEP_SETTINGS，ACK 共批 datapoint 进 pending 供第一次 acquire_one 消费，不丢不拼）；assembler 当前未完成 sweep 在边界丢弃（`reset_current` 语义），**统计不重置**（dropped/incomplete 等跨任务累计）。任务级 `FixedIntervalSweepSelector` 每任务 `reset()`（新任务新时间线，与第 1 轮 worker 行为一致 `[冻结]`）。 | `[自动]` |
| R-A-09 | FAULTED 例外于"保持 USB"：backend 进入 FAULTED（断开/协议错/pending 溢出等既有 fail-closed 源）时，服务必须立即释放（重试到 `is_open==False`）并回 `CLOSED`；下一次 `start_task` 重新 open（open_count+1，允许且必须如实统计）；`terminal_state`/`drain_failed` 上报路径与第 1 轮完全一致 `[冻结]`。 | `[自动]` |
| R-A-10 | 任务身份与编号（修订版，详见 §3.7）：遥测 `trace_index` **任务内**从 1 连续（沿用 `_selected_count+1` 语义 `[冻结]`）；发送**身份**（run_id）随采集任务轮换，保证不变量 INV：同一 run_id 生命周期内 `(run_id, trace_index)` 唯一对应一个 payload。backend 的 `trace_index`（设备道号）继续会话内累计，仅作诊断经 `session_statistics` 透传，与遥测编号解耦。 | `[自动]` R-A-16 |
| R-A-19b | **补传模式**（队长裁定 P2；规则全文见 §3.7 末"R-A-19b"段）：地面按 **run_id 完整字典序**比较基线（**禁止只比 seq**——空中进程重启后 seq 回 1 会把新任务误判为补传）；旧 run 重放 → `_accept_trace` 照常落盘+ACK，但 `begin_run`/基线/状态标签/display FIFO 全部不动；补传接收按 run 计数并在 run 结束时记一条汇总日志（不静默吞显示）。落点 t7（地面文件域），t5/t6 不要求地面改码。 | `[自动]` |
| R-A-11 | 任务开始状态消息：任务实际进入 STREAMING（configure/resume/reconfigure 成功）后、消费第一条 sweep 之前，向地面端发送 `enqueue_status("acquiring")`（该消息**类型与词汇第 1 轮地面端已实现处理分支**（`_on_acquisition_status` 的 `acquiring` → `_begin_task`+显示"采集中"），空中端只是从未发送——本轮补上发送方，零 framing 变更）；`status_reporting=False` 的校准短任务除外（§5.4）。`stopping/stopped/drain_failed` 三态序列与时序不变 `[冻结]`（StopReconcileTests 等全绿）。 | `[自动]` |
| R-A-12 | 兼容性护栏：`ContinuousAcquisitionWorker(service=None)` 的独立构造路径（既有 732 条基线测试全部走此路径）行为与第 1 轮**逐字一致**（自 open、自 close、自行号）；服务仅以新增关键字参数注入（`service=`），不改既有位置参数次序。`backend=` 注入既有测试点（fake backend）保持可用。 | `[自动]` 全量回归 |
| R-A-13 | 手动 USB 互斥升级：判据改为**并集**——`_reject_manual_usb_op` 在"`self._acquisition_running()`（worker 在跑，第 1 轮判据 `[冻结]`）**或** 服务持有 USB（`service.state != "closed"`，含 `idle_waiting` 待命态：接口仍被 claim，`acquire_sparameters`/`open_device` 自开自关必然撞 claim）"时一律拒绝，日志注明当前服务状态。MainWindow 手动 LibreVNA 路径（`test_vna`/`test_sweep_settings`/`capture_calibration_standard`/`capture_background`）继续走该函数。GNSS 串口不受影响（不同设备）。**必须用并集而非替换**：既有 `test_uav_gpr_qt_continuous.py:1460/:1479` 两条测试用 `window.worker = ...` 直接赋值绕过服务——若把判据"改成"服务状态，运行期不再拒绝 → `m.call_count==0` 断言失败（回归）；并集令二者全绿。新增测试专测服务路径：任务停止进入 `idle_waiting` 后手动入口**仍被拒绝**（这是常驻的定义性语义：待命期不恢复、`shutdown()` 后才恢复），并在 t3 报告点名该行为变化。 | `[自动]` |
| R-A-14 | 地面端停止对账不受影响：服务化后最终仍发 `stopped`+`drain_state` 且时序同第 1 轮（stopping→…→stopped），地面端 `_reconcile_stop_state` 六条件与停止提交屏障语义 `[冻结]`（对应测试不改而全绿）。 | `[自动]` 全量回归 |

### 3.4 后端层配套改动（`librevna/librevna_continuous.py`，最小增量）

1. **新增** `PersistentLibreVnaBackend.reconfigure(settings: SweepSettings) -> None`：仅允许 `PAUSED → STREAMING`；命令序列与 `configure()` 相同（SET_IDLE → 清 stream/pending → SWEEP_SETTINGS，失败 FAULTED）；成功后新建 `ContinuousSweepAssembler(settings.points)`、更新冻结配置、`configure_count += 1`、assembler 派生计数**折叠累计不缩小**（旧 assembler 统计并入快照，新计数在其上累加）；`trace_index/open_count/connection_generation` 连续。既有 `configure()` 从 PAUSED/STREAMING 的拒绝语义**保持不变** `[冻结]`（`_state_error` 既有分支不动）。
2. 其余既有状态机、统计、超时、pending 语义一律不改（第 1 轮 46 条后端契约测试 `[冻结]`）。
3. `librevna/librevna_transport.py` 预期**零改动**；若 t3 实测发现常驻必须触碰 transport（不预期），改动必须保持 `LibreVnaTransportProtocol`（open/read/write/close/is_open）接口不变，并在 t3 报告中单列说明。

### 3.5 涉及文件与函数签名清单（A）

| 文件 | 变更 | 关键签名/成员 |
|---|---|---|
| `uav_gpr_qt.py` | 修改 | 新增 `AcquisitionService`、`AcquisitionTaskSpec`（§3.2）；`ContinuousAcquisitionWorker.__init__(..., service: AcquisitionService | None = None)`；`run()` 内 open/configure 段改为 `service.open_or_ensure()/resume/reconfigure` 选择，finally 的 `_shutdown` 在服务模式下不释放 backend（改 `service.end_task(...)`）；`MainWindow.__init__` 持有 `self.acquisition_service`；`start_acquisition/_start_continuous_acquisition/stop_acquisition/pause_acquisition/resume_acquisition/handle_remote_command/closeEvent/_reject_manual_usb_op/_set_task_controls_locked` 改为服务入口 |
| `librevna/librevna_continuous.py` | 修改 | 新增 `reconfigure()`；其余不动 |
| `librevna/librevna_transport.py` | 预期不动 | （若动，见 §3.4-3 约束） |
| `tests/test_round2_resident_service.py` | 新增 | R-A-01…R-A-20 对应测试（fake transport/fake backend 注入，沿用 `test_uav_gpr_qt_continuous` 的 `_make_backend/_make_worker` 模式） |

### 3.6 第 1 轮既有语义保持清单（A 禁止回归项）

`selected==processed==saved` 对账与 `terminal_state` 五源综合判定 `[冻结]`；`failed_item` fail-closed 绝不伪装 drained `[冻结]`；`RemoteSender.stop(drain=True)` 超时返回 False 且不动线程/数据 `[冻结]`；acquisition_status `stopping/stopped/drain_failed` 序列与地面端显示 `[冻结]`；`LosslessProcessingQueue` put 三分语义与高水位跨阈值告警 `[冻结]`；closeEvent "USB 未释放不发 finished_run" 与绝不 terminate `[冻结]`；既有 732 条基线测试不改断言而全绿（如需改任何既有测试，必须在 t3 输出中逐条说明理由，评审人按"掩盖缺陷"审查）。

**例外条款（第 1 轮报告 §11/§17 已声明的语义变化）**：`tests/test_uav_gpr_qt_continuous.py::test_start_immediate_stop_closes_backend` 断言的"stop 后必须释放 USB"是独立 worker 路径的既有语义，与 R-A-01 的服务路径互不冲突——服务模式（`service=` 注入）"任务停止不释放"由新测试覆盖；独立路径（`service=None`）释放行为与全部既有断言**一字不改**。

### 3.7 不变量 INV：run_id 内 `(run_id, trace_index)` ↔ payload 唯一（t3/t5/t7 共同硬约束）

**不变量**：同一 run_id 的生命周期内，每个 `(run_id, trace_index)` 唯一对应一个 payload。地面端对"同 run 同编号不同 sha256"执行**冲突拒绝且不 ACK**（`uav_gpr_ground_station.py:759-805`，`CrossRestartDedupTests::test_restart_different_payload_rejected` `[冻结]`，任何工作流不得推翻）。发送端后果：`_wait_for_trace_ack` 等不到匹配 ack → `ConnectionError` → pending 无限重连重发同字节 → `outstanding` 永不归零 → `stop(drain=True)` 永远 False → **永久死锁**（不是丢一条）。取证脚本（真实 `ReceiverThread`，四场景 A冲突死锁/B幂等吸收/C身份轮换可行/D地面重启跨重启判重，退出码判定）：`docs/round2_repro_runid_collision.py`。第 1 轮侥幸成立仅因"每任务结束进程即退出/秒级时间戳恰好不同"；第 2 轮 sender 跨任务常驻必然触发。

**契约决策：采用方案 (a)"run_id 随采集任务轮换"（engB 建议）。实现形态 = 通道常驻（单实例、单 TCP 连接，沿用第 1 轮）+ 身份由消息携带（消息在**生产时刻**绑定 producing 任务的 run_id）+ 身份变化时在同一条连接上带内重播 hello。否决 (a-i) 无协调地原地改共享实例的 `self.run_id`；否决 (b) 服务级全局连续编号；并明确否决"每任务一个新 sender 实例/新连接"的方案变体（理由见下"地面单客户约束"）。**

- **地面单客户约束（读码事实，`uav_gpr_ground_station.py:635-683`）**：`ReceiverThread.run()` 的 accept 循环**串行**调用 `handle_client(client)`——旧连接不退出就不会 accept 下一条连接（新 connect 只滞留在 listen backlog，其 hello 得不到 ack）。因此"每任务一个实例/一条新连接"会抢占该唯一客户槽：退役实例断线重试期间可无限期霸占槽位，任务 2 的通道被饿死（相对第 1 轮的可用性回退）。**同一条约束对 t5 生效**：outbox 补传不得另开并行连接与实时通道抢槽，必须复用常驻 sender 的 FIFO（§4.4）。
- **(a-i) 被否决的具体竞态**：`RemoteSender` 在多处**发送时刻**读 `self.run_id`（`_connect` 的 hello、`_handle_command` 的 command_ack）。若在任务 2 启动时原地改写，而任务 1 的终态 `stopped` header 已在队（入队时按旧身份定型）或仍有在途 pending 消息，则会出现"任务 1 的终态以任务 2 的 run_id 抵达地面"→ 地面 run 匹配通过 → 为新任务错误建立 `_final_stop_state` 与停止提交屏障（飞行中闪现"正在停止"，正是第 1 轮复审点名的"计数说谎"类暗病）。身份由消息携带后，身份在生产时刻定型，任何后续改写都不可能造成错配。
- **(b) 被否决**：任务内 1..N 编号是第 1 轮大量 `[冻结]` 测试的既有断言（count=5 → trace_index 1..5）；全局计数还把"身份"耦合进"USB 会话"——outbox 重放要求磁盘条目携带**原始身份**（§4.6），全局计数器的续传点无法仅由磁盘推出；同身份内编号空洞也给地面端与事后分析制造无谓复杂度。
- **trace_index 冲突面归零的原理**：新任务=新 run_id ⇒ 地面内存 key 与磁盘 run 目录天然隔离 ⇒ 同编号不同 payload 不再可能互撞。编号本身保持任务内从 1 连续（`_selected_count+1` `[冻结]` 语义不动）。

**任务身份制（契约）**：

1. **run_id 格式（按来源分两式，互不碰撞）**：
   - **采集任务身份**：`YYYYmmdd_HHMMSS_%04d`（**`seq` 固定四位零填充**，与 §4.1"目录名字典序==时间序"的扫描排序前提同源；若实现成裸 `<seq>`，同秒内 `..._9` > `..._10` 会让 outbox 重放顺序倒挂——engB 指出的缺陷，此处写死）。`seq` = 服务内**采集**任务单调序号（从 1 起，进程内不重复）；同一秒连开两任务不冲突。
   - **待机通道身份**：`YYYYmmdd_HHMMSS_idle`（不匹配采集式正则，永不发送 trace；因此地面 R-A-19 的 `begin_run` 触发判据必须用"匹配 `^\d{8}_\d{6}_\d{4}$` 且 != 当前镜像身份"，而非"seq>0"字符串比较）。
   - **校准任务身份**（§5.4：校准以 `service.start_task` 运行，故消耗一个 seq）：`YYYYmmdd_HHMMSS_%04d_c`（同一 `new_run_id()` 入口加 kind 后缀）。当前编排下校准任务不发送 trace、`status_reporting=False`，该身份只存在于 spec/metadata，不上线；校准回程消息沿用当前连接身份（§5.4），所以地面词汇表实际只会见到采集任务式与 idle 式。
   - **签发唯一入口**：`AcquisitionService.new_run_id(kind: str = "task") -> str`（`kind ∈ {"task","idle","calibration"}`）；MainWindow/worker/校准/outbox 一律向服务取号，禁止自行拼接（现状散点 `uav_gpr_qt.py:3401` 的 `datetime.now().strftime("%Y%m%d_%H%M%S")` 收编进服务）。
2. **通道常驻**：`RemoteSender` 实例与 TCP 连接的生命周期沿用第 1 轮（跨任务存活；断线保留 pending 重连重发 `[冻结]`；`stop(drain=True)` 超时返回 False 且不动数据 `[冻结]`；`connect_remote_sender()` 语义与空闲期"地面可发命令"可用性 `[冻结]`）。`self.run_id` 继续存在，语义收敛为"**当前连接身份**"（hello 与 command_ack 用），不再是消息身份的唯一来源。
3. **消息携带身份**：`enqueue_trace(...)` 与 `enqueue_status(state, ...)` 新增关键字参数 `run_id: str | None = None`——`None` 时取 `self.run_id`（**第 1 轮全部调用点与既有断言逐字兼容 `[冻结]`**，如 `test_round1_defaults.py:257`）；非 `None` 时该消息 header 的 `run_id` 即为传入值。worker / outbox / 校准 **必须**显式传入本任务 run_id（由 `AcquisitionTaskSpec.run_id` 下发）。
4. **带内身份切换（同一条连接）**：`_run` 取出一条消息后，若 `header["run_id"] != self.run_id`：先在同一 socket 发送 `hello{run_id: 新}` 并等待其 `ack`（复用既有握手帧，零 framing 变更），再令 `self.run_id = 新`，然后发送该消息。切换由发送线程**单点串行**执行 ⇒ 同一 run_id 的全部消息在 FIFO 中必然连续（生产方按任务绑定身份，同任务消息之间不可能混入别的身份）；地面看到的顺序恒为 `hello(run₂) → acquiring(run₂) → traces(run₂) → stopping/stopped(run₂)`。**实现雷区（必须处理）**：等待 hello 的 `ack` 期间地面可能并发下发 `command`——切换等待循环必须像 `_wait_for_trace_ack` 一样分派 `command` 后继续等（否则命令帧会把切换踢成 `ConnectionError`，复现 §3.7 要消灭的死锁类缺陷）；未知 type 在切换等待中按既有先例 raise（不得静默吞）。
5. **acquiring 必发链（不变量 INV 的生效前提，engB 补充；两端各一票保障，缺一不可）**：空中侧——hello 成功（含带内切换）且 backend 进入 STREAMING 后、第一条 sweep 处理前，必须以该任务 run_id 入队 `acquisition_status("acquiring")`（R-A-11）；地面侧——收到**新 run_id** 的 hello 或 acquiring **任一**即主动完成 run 切换（R-A-19，落 t7）。
   - **地面 run_id 词汇表按来源分域（写死，防"忽略迟到停止"误伤新任务）**：四类身份（采集任务式 / 校准式 / idle 式 / 地面本地式）的**唯一定义与处置见本节末"地面身份词汇表"表格**——R-A-19、R-A-19b、R-A-20 三条共用同一 `classify_run_id()` 与同一字典序比较函数，**任何一处都不得再写第二套正则**（engB 指出：原文 R-A-19 用 `(_c)?` 而 R-A-20 用 `\d+`，两条判据对校准 id 结论相反）。`_on_acquisition_status` 的"迟到"护栏：① run_id == 当前基线 → 正常处理；② == 基线镜像的空中 run_id（切换时记录 `_mirrored_air_run_id`）→ 正常处理；③ 分类**不是** `air_task`（idle / 校准 / 本地占位 `start-<epoch>`、`single-<epoch>`）→ **不适用**"忽略迟到"，按当前 run 处理；④ 分类为 `air_task` 且 ≠ 当前镜像身份 → 判为旧 run 迟到消息 → 忽略（`test_late_old_run_id_stopped_ignored` `[冻结]`）。此分域消灭一个真实竞态：地面点"开始"（本地基线）与随后 `acquiring(run₂)` 到达顺序不定——run₂ 按 ③/④→R-A-19① 完成切换，新任务不会被误判"迟到"而永停在"正在停止"。
   - 若只有空中侧必发而地面不切：新 run 的 `stopped` 被"忽略迟到停止"吞掉 → 地面永停在"正在停止"（比死锁更隐蔽的状态失联）；若只有地面切换而 acquiring 丢失：新 run trace 仍能正确保存（hello 已切换），但显示基线不完整。两条路径各有测试（R-A-16/R-A-19）。
6. **地面既有机制零改动承接**：`_accept_trace` 按 `(run_id, index)` key 隔离 + 磁盘按 run 分目录 + `ReceiverThread.begin_run` 清内存表 + `_on_acquisition_status` run 切换分支（`:1542-1550`）；迟到 stopped/drain_failed 忽略分支（`test_late_old_run_id_stopped_ignored` `[冻结]`）在新机制下仍是期望行为——旧 run 消息不得影响当前 run 对账。
7. **残余窗口（如实声明）**：任务 1 仍有 trace 处于 pending 重试时任务 2 启动，重连后的带内 hello(run₂) 会先完成切换；此后旧 pending 仍以 run₁ 身份发送（header 已定型）、地面按 run₁ 正常对账/判重——这是期望行为，禁止"顺手改进"为按当前连接身份重发。
8. **legacy 模式例外（读码事实）**：`MainWindow.start_acquisition` 非连续模式创建的 `AcquisitionWorker`（`uav_gpr_qt.py:3301`）不接收 `remote_sender`——第 1 轮 legacy 路径本就不做遥测。常驻服务与任务身份制**只作用于 continuous 路径**；legacy 路径逐字保持第 1 轮 `[冻结]`。`run_id` 唯一签发口是 `service.new_run_id(kind)`；现状散点 `uav_gpr_qt.py:3401`（秒级时间戳）在 t3 收编（通道创建时取 idle 身份，任务启动经带内 hello 切到任务身份），生产代码不得存在第二处 run_id 拼接。
9. **地面身份词汇表（唯一定义；R-A-19 / R-A-19b / R-A-20 三条必须引用本表，禁止各自再写正则——engB 指出的"两处判据对同一 id 结论相反"缺陷由此根除）**：

| 类别 | 单一正则 | 是否可能上线发 trace | 地面处置 |
|---|---|---|---|
| `air_task` 采集任务式 | `^\d{8}_\d{6}_\d{4}$` | 是 | hello/acquiring 触发切换（R-A-19①，须先过 R-A-19b 的"不早于基线"判据）；trace 正常落盘+ACK+显示 |
| `calibration` 校准式 | `^\d{8}_\d{6}_\d{4}_c$` | **本轮不上线**（§5.4：校准 `status_reporting=False`，回程用当前连接身份） | 一旦出现：按 **R-A-19b 补传模式**处置（trace 照常落盘+ACK 保数据完整，绝不 begin_run / 绝不显示）+ 记一条"意外的校准身份上线"日志。**禁止**"忽略且不回 ACK"（＝§3.7 死锁复活） |
| `idle` 待机式 | `^\d{8}_\d{6}_idle$` | 否（永不发 trace） | 不触发切换、不改基线、不建任务（只作连接身份承载命令通道） |
| `local` 地面本地式 | 其余一切（含 `start-<epoch>`/`single-<epoch>`） | — | 分类非 `air_task` → R-A-20 规则③，永不判为"迟到" |

   地面必须实现为**单一分类函数 + 单一比较函数**：`classify_run_id(run_id) -> str`（返回四类之一）与 `is_not_older_than(run_x, baseline)`（**完整 run_id 字典序**，因定长时间戳+`%04d` 等价于时间序；基线空视为满足）。R-A-19（是否切换）、R-A-19b（是否进补传模式）、R-A-20（是否忽略迟到状态）三处一律调这两个函数；**不允许**任何一处再写第二套正则或改用 seq 数值比较（有测试锁定三处行为一致）。

**验收条款（R-A-15 … R-A-19）**：

| 编号 | 条款 | 验证 |
|---|---|---|
| R-A-15 | 任务身份制：每采集任务由服务签发唯一 run_id（格式/seq 单调/同秒不重复直接断言；散点生成收编，rg 生产代码不再出现第二处 run_id 拼接）；消息携带身份（`run_id=` 显式传入路径）；`_run` 在身份变化处**带内重播 hello 并等到 ack** 后才发该消息；同一 run_id 的消息在 FIFO 中连续；`run_id=None` 默认路径与第 1 轮逐字兼容 `[冻结]`。 | `[自动]` + `[人工]` |
| R-A-16 | 死锁回归（engB 复现固化进 `tests/test_round2_resident_service.py`，不依赖仓库外脚本）：真实 `ReceiverThread` + 常驻服务**同一条客户连接**上连续两任务：任务 2 首条 trace 被地面**保存并 ACK**（全程 `Trace 冲突拒绝` 日志计数==0）；两任务终态各自 `stop(drain=True)` 为 True、`outstanding` 归零；两任务 `(run_id, trace_index)` 集合不相交；任务 1 终态 stopped 携带 run_id₁、任务 2 携带 run_id₂（地面日志可验证归属）；hello 已切换但 acquiring 人为丢失（直连 socket 注入场景）时，新 run trace 仍保存+ACK。 | `[自动]` |
| R-A-17 | 命令通道归属：全程只有一条地面客户连接在工作（不得出现第二连接饿死客户槽——地面 accept 串行是读码事实）；带内 hello 切换不新建连接；`command_ack` 携带发送时刻的连接身份；地面命令对活动任务生效；`handle_remote_command` 若因身份/状态不符拒绝执行，必须记一条含原因与当前身份的日志（不静默丢弃）。 | `[自动]` |
| R-A-18 | 断线不阻塞新任务：任务 1 终态消息滞留（地面不可达、pending 重试）时启动任务 2 —— 任务 2 的 acquiring/trace 排在同一 FIFO 之后，链路恢复后按序全部送达（跨两任务 `selected==acked` 对账成立；旧 run 的 pending 以 run₁ 原字节送达、地面按 run₁ 幂等对账），任务 2 本地采集/保存全程不受影响。 | `[自动]` |
| R-A-19 | 地面 run 切换双触发（**落 t7/engC**：t3 的文件域不含地面端；t3 只保证空中侧必发）：① 收到 hello 且 `classify_run_id(run_id)=="air_task"`（§3.7 第 9 点词汇表，**唯一判据来源**）且 != 当前镜像身份 **且 `is_not_older_than(run_id, 基线)`（R-A-19b）** → `receiver.begin_run(run_id)` 清内存去重表（磁盘判重路径不受影响 `[冻结]`）+ 窗口按既有 acquiring 路径建立任务基线（hello 触发的切换在窗口侧合成等价的 `acquisition_status{state:"acquiring", run_id}` 处理，不新增第二套基线逻辑；显示"采集中"；idle / calibration / local 类 id 一律不触发切换）；② 真 `acquiring` 与当前 run 一致 → **幂等**（不重复 `_begin_task`、不清 `_final_stop_state`、不重置已建基线）；③ **同** run_id 的重连 hello **不得**重置窗口任务基线（防链路瞬断丢停止对账状态）。测试覆盖：只有 hello 无 acquiring 的新任务 → 首条 trace 保存+ACK、显示"采集中"、冲突拒绝计数 0；hello 丢失只有 acquiring → 同样完成切换；idle hello → 不切换；**旧 run hello（早于基线）→ 不切换，走 R-A-19b 补传模式**。 | `[自动]` |
| R-A-19b | **补传模式**（队长裁定 P2；规则全文见 §3.7 末"R-A-19b"段）：地面按 **run_id 完整字典序**与基线比较（**禁止只比 seq**——空中进程重启后 seq 回到 1，会把更晚的新任务误判为补传）；`run_x < 基线` 的重放 → `_accept_trace` 照常落盘+ACK（duplicate 吸收不变），但 `begin_run`/基线/状态标签/display FIFO 全部不动；补传接收按 run 计数并在 run 结束时记一条汇总日志（不静默吞显示）。落点 t7（地面文件域），t5/t6 不要求地面改码。 | `[自动]` |
| R-A-20 | run_id 词汇表分域（地面侧；判据**唯一来源 = §3.7 第 9 点身份词汇表**，本条不再自带正则）：`_on_acquisition_status` 的"忽略迟到停止"护栏必须先调 `classify_run_id()`——分类**不是** `air_task`（地面本地占位 `start-<ts>`/`single-<ts>`、idle 式、校准式）**不得**被判为迟到；分类为 `air_task` 且不等于当前镜像身份才忽略。测试覆盖：地面点"开始"后先收到 run₂ 的 acquiring（基线为本地占位）→ 必须完成切换并显示"采集中"，随后 run₂ 的 stopped 必须被正常对账（不被吞）；`test_late_old_run_id_stopped_ignored` 既有场景（真旧 run 迟到）仍通过 `[冻结]`。 | `[自动]` |

**R-A-19b（队长裁定 P2：补传模式 = 身份信息推断，禁止"补传标记"方案）**：

地面维护"当前基线身份"（`_mirrored_air_run_id`，无基线=空）。收到 hello/acquiring(run_x) 时：

- **比较键必须是完整 run_id 的字典序，不得只比 seq**。定长时间戳 + `%04d`（P1 裁定）保证"字典序 == 时间序 == 数值序"；而 `seq` 在**空中端进程重启后回到 1**——只比 seq 会把"重启后的新任务"（时间更晚、seq 更小）误判为补传，新任务从此不上屏（比第 1 轮"停止后又来一条"更难查）。
- `run_x ≥ 基线`（或基线为空）→ 走 R-A-19 正常路径（`begin_run` + 建基线 + 显示"采集中"）。
- `run_x < 基线` → 进入**补传模式**：
  1. run_x 的 trace 帧**仍走 `_accept_trace` 完整落盘 + ACK**（含 duplicate 吸收）——数据完整性优先，且 outbox 依赖 ACK 才能划除（R-B-02），不 ACK 会直接把 §3.7 死锁复活；
  2. **不** `begin_run`、**不**切基线、**不**改采集状态标签（不显示"采集中"）、**不** `_note_emitted()` / 不 emit `trace_received`（因此不进 decoder 与 display FIFO）——避免污染当前任务的显示窗与停止对账基线（`_task_baseline`/`_stop_commit_barrier`）；
  3. run_x 的 `acquisition_status(stopped/drain_failed)` 仍按 R-A-20 规则忽略；
  4. **可观测性不得静默**：补传模式期间按 run 计数 `replay_ingested`，每 run 结束时在运行日志记一条"补传模式接收 run_x N 条（仅落盘，不显示）"——否则"吞显示"本身就成了新暗病。
- 退出条件：收到 `run_id ≥ 基线` 的 hello/acquiring → 正常切换并退出补传模式。
- 地面刚重启（基线为空）时重放按正常当前-run 处理（与取证脚本场景 D 一致；随后新任务的 acquiring 自然切走，可接受）。
- **落点**：`uav_gpr_ground_station.py` 属 engC 文件域 → 实现归 **t7**，与 R-A-19 同处；**t5/t6 不要求地面改码**（t5 的"重放不重复显示"验收由既有 duplicate 吸收达成，补传模式是 t7 之上的额外显示洁净性保证）。

**R-A-10（修订定稿）**：遥测 trace_index 任务内从 1 连续 `[冻结]`；发送身份按本节任务身份制轮换。**撤销**本文档早期草稿的"服务级全局单调编号"（即方案 b），以本节为准。
**R-A-09（补充）**：FAULTED 任务的终态消息以该任务 run_id 发出后，**通道保持常驻**（不关闭连接、不释放客户槽）；backend 按 R-A-09 释放并回 CLOSED，下一次 `start_task` 重新 open（open_count+1 如实统计）。

---

## 4. 工作流 B：磁盘 outbox 断点补传（engB / t5）

### 4.0 需求来源与目标

第 1 轮完成报告 §11 明确声明未实现磁盘 outbox，且列出了**未覆盖故障范围**：①进程崩溃/被杀；②正常关闭时网络长时间断开（30s 排空超时后用户强退）；③系统断电。§17-1 将其列为已知限制。理由段原文（§11）："磁盘 outbox 需要原子写、ACK 回执持久化、重启扫描与 run_id/trace_index 续传协议，属于独立的可靠性工程，建议第2轮与常驻采集服务一起实现（见 §18）"。

**目标**：待发送遥测 trace 先原子落盘、收到匹配 ACK 后划除；进程重启后扫描未确认条目，按原始身份 `(run_id, trace_index, sha256)` **字节级**续传；覆盖上述三类故障，且全部重发被地面端第 1 轮去重机制吸收（不重复保存、不重复显示、无脏文件）。

**非目标**：不改地面端任何代码；不改 telemetry framing 既有字段（§3.7 的带内 hello 复用既有消息类型，不算新增）；不做滑动窗口 ACK（p95 目标仍是第 1 轮口径）；不改采集服务与 UI 布局。

### 4.1 落盘目录与文件编码（engB 主张①，采纳）

- **目录**：`<out_dir>/_outbox/<run_id>/<trace_index:06d>.frame`。`out_dir` 即空中端采集输出目录（默认 `gpr_runs`）。选点理由与约束：
  - outbox 与**已确认无需再发的业务数据**（NPZ/CSV）分离；`_outbox/` 前缀不会与 run 数据目录冲突——run 目录名是 `safe_name(run_id)`，而本契约 run_id 格式 `YYYYmmdd_HHMMSS_%04d`（§3.7，seq 四位零填充）保证目录名以数字开头，`"_outbox"` 永不与之相撞；
  - 扫描排序：目录名定长时间戳+定长 seq ⇒ **字典序 == 时间序**（第 1 轮"秒级时间戳变长后缀"的教训固化进格式）；
  - `.frame` 文件存在本身 = "未确认"标记；目录内文件数即该 run 未确认条数。
- **内容 = 原始 wire 帧字节**：`pack("!I",len(hb)) + hb + pack("!Q",len(pb)) + pb`，与 `send_remote_frame` 写到 socket 的字节**逐字节一致**（header JSON 用与线上完全相同的 `json.dumps(..., separators=(",",":"))` 定型；禁止落盘后重新序列化）。重放即"把文件字节 sendall"，天然满足 R-X-02 逐字节不变。
- **命名唯一性**：`(run_id, trace_index)` 已被"目录+文件名"完整编码；文件名零填充 6 位与地面 `trace_%05d.npz` 无耦合（仅 outbox 内部排序用）。

### 4.2 ACK 回执持久化（engB 主张②前半，采纳"划除即持久化"）

- **划除**：`_run` 收到匹配 `ack_trace`（header `trace_index` 相等，含 `duplicate: true`）→ 删除对应 `.frame` 文件 → `_note_completed()`。删除失败（被占用等）→ **不**减 outstanding、记录 `ack_removal_failures` 计数并告警（下轮扫描时地面幂等 duplicate ACK 会再次尝试划除）。
- **无需独立 ledger**：帧文件的"存在/不存在"就是持久化确认状态（存在=未确认；删除=已确认）。划除前崩溃 → 重启重发同字节 → 地面 duplicate ACK（取证 `docs/round2_repro_runid_collision.py` 场景 B/D：内存表命中与跨重启磁盘比 sha 两条路径均 duplicate ACK、不重复落盘、不重复写 metadata）。
- **崩溃窗口诚实声明**：`sendall 成功→收到 ACK→删除文件` 之间任一点崩溃，结果都只是"重发+被吸收"，无数据丢失、无重复副作用。

### 4.3 落盘时序与故障语义（fail-closed）

1. **先落盘后入队**：`RemoteSender.enqueue_trace` 的组装顺序改为 `trace_npz_bytes` → header 定型 → **原子写盘**（`<name>.tmp` → flush+fsync → `os.replace`）→ `_note_enqueued()` → `queue.put`。入队/发送逻辑与时机不变 `[冻结]`。
2. **写盘失败 = 该条不进 outbox 保护，但不得静默**：捕获 OSError（磁盘满/权限）→ 条目**仍入内存队列尝试发送**（不因写盘失败丢实时链路）→ 计数 `outbox_write_failures += 1` + `status_callback("Outbox 写盘失败: ...")` + `acquisition_status("outbox_alert", kind="write_failed", trace_index=...)`（新增消息走 R-X-02 先例）；程序崩溃时该条确实会丢——这正是"绝不伪装成功"的口径：写盘失败条数在统计与终报中可见。
3. **损坏文件 fail-closed**：扫描时文件缺失/截断（帧解析失败）、header sha256 与 payload 实算不符、header 缺 `run_id/trace_index` → 移入 `<out_dir>/_outbox/_corrupt/<原名>.corrupt`（**不删除**，保留取证），逐条记录并计入 `outbox_corrupt_files`；绝不猜测身份重发（防制造地面冲突拒绝死锁）。
4. **outbox 统计**：`pending_files / bytes / write_failures / corrupt_files / replayed / replay_duplicate_acked`（后者=收到 `duplicate:true` 而划除的条数，证明重发被吸收，不是丢数）。

### 4.4 恢复扫描与续传（engB 主张②，采纳"sender 启动期扫描 + 未完成条目计入 outstanding"）

- **触发点**：`RemoteSender` **构造/start 时同步扫描** outbox（`scan_pending() -> list[OutboxEntry]`，按 `(run_id, trace_index)` 排序），恢复列表进入既有 `_run` 主循环的 `pending` 变量位（与断线 pending 重发同一状态机，天然 FIFO 有序）。**否决独立恢复线程/独立阶段**：单 `_run` 串行发送保证任意时刻只有一帧在途，双线程必然引入乱序与双份 outstanding 记账（第 1 轮复审点名的"计数说谎"温床）；也否决另开 TCP 连接（地面单客户槽，§3.7）。
- **身份重播**：重放条目**不**新建连接、**不**自行握手——它作为普通帧进入常驻 sender 的 FIFO，由 §3.7-4 的**带内身份切换**机制在其首帧前以该条目的 `run_id` 重播 hello。禁止给重放条目改写/重新签发 run_id 或 trace_index（重放身份 == 原始发送身份，三元组 `(run_id, trace_index, sha256)` 与文件字节同源）。地面重启后重连同理（hello 携带待重放条目的 run_id，R-A-19 路径①完成 `begin_run`）。
- **计数衔接（写死，engB 主张②）**：恢复条目在入恢复列表时即 `_note_enqueued()`（**完成前必须计入 outstanding**，与实时消息同一把锁、同一个计数器）；收到匹配 ack_trace 后 `_note_completed()` + 划除。效果：`stop(drain=True)`/closeEvent 的真实排空语义自动覆盖补传（`MainWindowCloseDrainTests` 与复审报告 §2 一的语义 `[冻结]` 不得被绕过）；地面未启动时启动程序 → outstanding>0、sender 按既有 2s 限速重连重试，绝不伪装排空。
- **trace_index 续传规则**（任务卡原文"重启扫描 run_id/trace_index 续传"）：重放严格使用磁盘条目内身份；新进程的新任务由 §3.7 身份制签发**新 run_id**、trace_index 从 1 起——重放与新生成集合不相交，无冲突面。
- **地面重启兼容**：取证脚本场景 D 已证：地面新实例收同 `(run_id,trace_index)` 同 sha → 磁盘比 sha duplicate ACK、不重复落盘——本条款为 t9 "崩溃恢复基准"的必过项。

### 4.5 与常驻服务（t3）的衔接

- worker/消费者不感知 outbox：`ProcessingConsumer → RemoteSender.enqueue_trace` 调用点不变；持久化发生在 sender 内部（payload 已含全部 metadata，header 可 JSON 落盘）。
- 服务 `shutdown()` 前不再新增 trace 入队（停止收敛），closeEvent 排空含恢复条目（R-A-07 + §4.4）。
- legacy `AcquisitionWorker` 路径不接 outbox（保持第 1 轮行为 `[冻结]`）。

### 4.6 逐条验收条款（B）

| 编号 | 条款 | 验证 |
|---|---|---|
| R-B-01 | 每条待发 trace 先原子写盘（`.tmp`+`os.replace`，flush 后 rename），后入队发送；帧文件字节与 socket 线上字节逐字节一致（测试：捕获 sendall 字节 == 文件内容）。 | `[自动]` |
| R-B-02 | 收到匹配 ack_trace（含 duplicate）后划除文件；未 ACK 条目在"进程任意时刻被杀"后重启时全部被发现并重发；`ack_removal_failures/outbox_write_failures/outbox_corrupt_files` 计数可观测。 | `[自动]` |
| R-B-03 | **崩溃恢复基准（engB 主张③口径，写死）**：测试用 `subprocess` 起**新 python 进程**加载同一 outbox 目录重放（同进程内新建对象不算崩溃恢复）；场景=发送 K 条中 `os._exit(9)` 硬杀 → 新进程扫描续传 → 地面最终"新保存+duplicate"总数==K、无冲突拒绝、无 metadata 重复行、无 `trace_00000` 类脏文件；t9 另跑全链路版（模拟断发 N 条→重启→地面最终计数与无 outbox 基线一致、无重复显示）。 | `[自动]` |
| R-B-04 | 重放不引入新身份：条目 `(run_id, trace_index, sha256)` 与落盘时一致（重放帧字节 == 原文件字节断言）；与 §3.7 身份制兼容（新任务新 run_id，重放只发旧 run_id 帧）。 | `[自动]` |
| R-B-05 | 恢复条目计入 outstanding，排空/关闭语义覆盖 outbox（`stop(drain=True)` 在有恢复条目未 ACK 时返回 False 且数据无损；恢复完成后可再次成功）。 | `[自动]` |
| R-B-06 | 损坏/半文件（截断帧、sha 不符、缺身份字段）→ `_corrupt` 隔离 + 告警 + 不重发不删除；磁盘满/写失败 → 实时链路不中断 + `outbox_alert` 状态消息 + 计数如实。 | `[自动]` |
| R-B-07 | 地面端零改动吸收全部重发（既有 `CrossRestartDedupTests`/`ReceiverThread` 测试一字不改全绿 `[冻结]`）。 | `[自动]` 全量回归 |
| R-B-08 | 写盘开销有**真实测量数字**：outbox 专项计时（每条 `outbox_write` 阶段进 AIR_METRICS，全链路基准报告 p50/p95/max；500 条基准端到端 p95 仍 ≤200ms `[冻结]` 预算内），禁止口头声明。 | `[自动]` bench 输出 |
| R-B-09 | framing 零变更：`.frame` 内容、hello、ack_trace、新增消息仅 `acquisition_status` 的 `outbox_alert` 一类（旧地面端落入现有"未知 state → 采集状态：未知状态"分支，不崩溃不拒收——既有 `_on_acquisition_status` else 分支 `[冻结]`）。 | `[自动]` + `[人工]` diff |
| R-B-10 | `disk_outbox.py` 为纯 Python 模块（不依赖 Qt），FakeTransport/临时目录可完全单测；`RemoteSender.__init__` 新增 `outbox_dir: Path | None = None`（None=禁用持久化，全部既有 sender 测试逐字兼容 `[冻结]`）。 | `[自动]` 全量回归 |

### 4.7 涉及文件与函数签名清单（B）

| 文件 | 变更 | 关键签名 |
|---|---|---|
| `disk_outbox.py` | **新增** | `class DiskOutbox: __init__(self, root: Path)`；`append(frame_bytes, run_id, trace_index) -> Path`（原子写）；`scan_pending() -> list[OutboxEntry]`（排序+校验+隔离损坏）；`remove(entry)` / `mark_acked(entry)`；`statistics() -> dict`；`OutboxEntry(path, run_id, trace_index, sha256, frame: bytes)` |
| `uav_gpr_qt.py` | 修改（仅 RemoteSender 接线区域） | `RemoteSender.__init__(..., outbox_dir=None)`；`enqueue_trace` 写盘前置；`_run` 启动扫描注入 pending 链；ack 后划除；`outbox_alert` 状态消息 |
| `tests/test_round2_disk_outbox.py` | 新增 | R-B-01…R-B-10 对应测试（含 subprocess 崩溃注入） |

---

## 5. 工作流 C：空中端 UI 精简 + 地面校准功能（engC / t7）

### 5.0 需求来源与目标

- **空中端 UI 精简**：第 1 轮完成报告 §15 原文"空中端 UI：除 interval spin 默认值 1.0→0.0 外无变化（本轮禁止大规模重做，遵守）"——即第 1 轮把空中端 UI 精简明确留后；本轮团队目标③正式立项。现状：`MainWindow` 两个 Tab（`_connection_page` "连接/参数/校准" + `_acquisition_page` "数据采集/B-scan"）承载 40+ 控件，飞行中操作员需在同一屏区分"能碰/不能碰"。
- **地面校准功能**：依据 `docs/钢筋仪项目_S11_S22_采集与校准参考.md` §4（六步双反射 OSL：P1 OSL 采 S11 + P2 OSL 采 S22，每标准件默认 16 条复数平均，范围 1–256）、§5（背景扣除必须在 RAW/OSL 域一致，禁止把单通道背景广播到 S11/S22）、§6-5（"接入六步双 OSL"）、§8（"必须用真实 Open/Short/Load 完成验收，不能只凭合成单元测试"）。现状：空中端 `MainWindow` 已有本地校准采集/生成/保存全套；**地面站完全没有校准入口**。

**目标**：
1. 空中端主界面只保留操作员飞行中真正需要的要素（连接 / 开始 / 停止 / 节奏 / 状态 / B-scan / 日志），深度设置移入独立**非模态**对话框，任务运行中危险项禁用（防误触）。
2. 地面站新增校准功能：在地面站**发起并查看** OSL / 空采参考采集编排（人在地面、设备在机上），空中端执行、地面端展示；全部逻辑在 fake 后端 + offscreen Qt 下可自动化测试。

**非目标**：不改第 1 轮显示/排空/去重/B-scan/GNSS/无损队列任何语义（全量回归为准）；不改 telemetry framing 既有字段；不改 B-scan 数学与处理管线算法；不做真机校准质量验收（列入未验证清单）；不改 `librevna/`（校准采集复用 t3 的**服务持有的 backend**，不引入新的 USB 出口）。

### 5.1 空中端 UI 精简：保留 / 移入对话框清单（契约）

主界面（`MainWindow` 中央区域）**保留**：

| 要素 | 现有控件 | 说明 |
|---|---|---|
| 地面链路 | `remote_enable`/`remote_host`/`remote_port`/`remote_connect` | 可折叠为"链路"分组，控件与语义不变 `[冻结]` |
| GNSS 串口 | `port_combo`/`baud_combo`/刷新/`test_gnss_btn` | GNSS 串口与 LibreVNA 无关，任务运行中仍可操作（R-A-13 只锁 LibreVNA） |
| 任务命令 | `start_btn`/`single_btn`/`stop_btn` | 停止永远可用 `[冻结]` |
| 采集节奏 | `interval`（默认 0.0 每个完整 Sweep `[冻结]`）、`count`、`continuous_enabled`、暂停/继续 | |
| 状态区 | `continuous_status_label`、`last_info`、**新增服务状态行**（§3 state / open_count / "USB 待命中"提示） | |
| 主视图 | `bscan_views`（S11 上 / S22 下 `[冻结]`）、`bscan_window_slider`+`bscan_window_label`（浏览旧窗口不抢回 `[冻结]`）、`bscan_filter_status` | B-scan 不得移除或降级 |
| 日志 | `log_box`（与 `device_log` 可合并为一区） | 内容契约不变 `[冻结]` |

移入 `AirSettingsDialog`（非模态，主界面"设置…"打开；控件本身**只换宿主不删不改名**）：

| 对话框分组 | 控件 |
|---|---|
| VNA 基础参数 | `start_mhz`/`stop_mhz`/`points`/`ifbw`/`power`/`dwell` |
| 处理参数 | `ifft_points`/`time_window`/`dewow_window`/`dewow_enabled` |
| 校准（空中端本地） | `cal_path`/`cal_mode`/`cal_lamps`/`cal_step_buttons`/生成/保存校准入口 |
| 显示 | `bscan_cmap`/`bscan_aspect`/`flat_window_traces`/`flat_filter_enabled` |
| 输出与保存 | `output_dir`+`choose_output`/`autosave` |
| 背景扣除 | `capture_bg_btn`/清除背景/`bg_label` |
| 诊断 | `test_vna_btn`/`test_sweep_btn`/链路性能诊断 |

**禁止删除或改语义**：`stop_btn`、`start_btn`、`interval`（含默认 0.0）、`count`、`autosave`、`continuous_enabled`、暂停/继续、`bscan_window_slider`、`gnss_map`（可折叠但不得删除）、`clear_bscan`、`remote_*` 全链、关于/菜单既有 action（`AboutMenuTests` `[冻结]`）。`QTabWidget` 可被替换，但上表控件的属性名必须仍可从 `MainWindow` 解析（既有测试定位点，如 `test_bscan_gui` 用 `window.bscan_window_slider`）。

**防误触（飞行中禁用）**：既有 `_set_task_controls_locked(True)` 清单 `[冻结]` 继续生效并扩展为"**凡访问 LibreVNA 或改变冻结任务配置的控件，在任务运行中一律禁用，无论其宿主是主界面还是已打开的对话框**"（不得只在构造时判定；`R-A-13` 的判据"服务持有 USB"是同一把尺）。"设置…"按钮本身保持可点（允许只读查看参数）。

### 5.2 地面校准功能：入口、流程步骤与可测边界

**入口**：`GroundStationWindow` 菜单"任务"新增"校准……"（objectName `action_calibration`）→ `CalibrationDialog`（非模态）。对话框只做编排与展示，不直接访问硬件（硬件在机上）。

**编排流程（`calibration_service.CalibrationSession` 状态机）**：

```
地面 CalibrationDialog ──command(action="cal_*")──▶ 空中端执行 ──calibration_*──▶ 地面展示
 1 select_mode    OSL（六步双反射）| 空采参考（背景）
 2 plan           steps=[P1_Open,P1_Short,P1_Load,P2_Open,P2_Short,P2_Load]
                  repeats（默认 16，范围 1–256）+ SweepConfig 快照（频/IFBW/功率/dwell）
 3 cal_begin      {cal_id, mode, steps, repeats, config} → 空中端校验服务空闲（R-C-14）后建会话
 4 cal_capture    逐 step：提示操作员接标准件 → 空中端采集 repeats 条完整 sweep
                  → 复数平均 np.mean(captures, axis=0)
 5 回程           calibration_capture {cal_id, step, index, of, channel, freq_hz, real, imag}
                  calibration_quality {cal_id, step, repeat_rms, max_abs_err, sample_count}
 6 cal_finish     空中端解算两套 one-port OSL(D,T,M) → 按既有格式落盘 →
                  calibration_result {cal_id, path, format, profile_summary, valid_band_hz}
 7 cal_activate   {cal_id} → 空中端启用为当前会话校准（后续采集立即生效）
 x cal_cancel / 地面断开 / 空中 FAULTED → 会话作废，绝不激活半套校准
```

**地面展示**：步骤进度表（待采集 / 采集中 n/of / 已完成 / 失败+原因）、质量指标（重复 RMS、最大绝对误差、样本数）、结果摘要（有效频段、点数、P1/P2 profile 身份、文件路径）。

**不依赖真机的可测边界（写死，杜绝"只能真机验收"）**：
1. 硬件出口全部经注入点：`CalibrationSession(capturer=...)`，capturer 是**服务持有的后端抽象**（见 §5.4 USB 归属条款：禁止 `acquire_sparameters` 式自开自关），测试注入 fake capturer 产生确定性 S11/S22 序列（沿用 `test_librevna_backend.py` fake transport、`test_uav_gpr_qt_continuous.py::_make_backend` 模式）；
2. 数学层为纯函数（复数平均 / 三项 OSL 求解 / 质量指标 / 带内插值），用"已知 D,T,M + 理想标准件正演生成的测量值"验证**自洽还原**（rtol 1e-9），并交叉验证 P1/P2 两套不得交换共用；
3. 协议层用真实 `ReceiverThread` + loopback 验证 `cal_*` 命令与 `calibration_*` 回程（含旧端忽略分支）；
4. UI 层 offscreen 断言控件状态/文本/禁用项。
5. **明确未验证**：真实 Open/Short/Load、匹配终端、现场线缆效应下的校准质量（§7）。

### 5.3 校准数据落盘格式（对齐参考文档，不发明新格式）

- 校准文件：**复用** `uav_gpr_qt.write_calibration_file` 的 `format="uav_gpr_solt_calibration_v2"` JSON（`created_at/method/s_parameter/sweep_config/measurements[]`）。扩展只允许两种形态：`measurements` 中多一条 `type`，或顶层新增**可选** key；`read_calibration_measurements` 必须能读回（往返一致），旧文件仍可读（`tests/test_uav_gpr_qt_metadata.py` 兼容断言 `[冻结]`）。六步 key 沿用既有常量 `CALIBRATION_STEPS`/`P1_OSL_KEYS`（`P1_Open…P2_Load`）。
- 每条 capture 原始复数：写 NPZ，key 沿用既有约定（`frequency_hz`、`<channel>_real`/`<channel>_imag`），**只**写在 `out_dir/_calibration/<cal_id>/` 下，且**文件名一律 `capture_<step>_<i>.npz`，禁止使用 `trace_*` 前缀**。理由（读码事实）：`docs/bench_round1_fullchain.py:221` 用 `air_out.rglob("trace_*.npz")` **递归**统计 `local_saved`——只要文件名撞上 `trace_`，即使放在子目录也会被计入，第 1 轮 `[冻结]` 的 500:500 对账口径即被污染（不是"放对目录就安全"）。
- 空采参考（背景）：沿用 `background_frequency_hz + background`，域约束见 R-C-19（OSL 之后、BPF/IFFT 之前的 RAW/OSL 域；频率轴不一致整条拒绝；禁止单通道背景广播到双通道）。
- 校准激活后的身份必须进入后续 trace 的 `acquisition` metadata **新可选 key**（如 `"calibration_id"`）——只进嵌套 metadata dict，不进 framing 顶层（R-X-02）。

### 5.4 协议约束（C，含一处必须避开的既有雷区）

- **地面 → 空中**：一律走既有 `type="command"` + **新 `action` 值**（`cal_begin/cal_capture/cal_finish/cal_activate/cal_cancel`）。**严禁新增地面→空中的消息 `type`**：`RemoteSender._run`（`uav_gpr_qt.py:1586`）与 `_wait_for_trace_ack`（`:1564`）对非 `command` 的入站帧一律 `raise ConnectionError` → 等 trace ack 期间收到未知 type 会把空中端踢下线并无路重试 → §3.7 同类死锁。旧空中端收到未知 action 的既有行为（回 `command_ack` + `handle_remote_command` 记 "Unknown remote command"）作为向后兼容先例沿用。
- **空中 → 地面**：新增 `type="calibration_capture" / "calibration_quality" / "calibration_result"`，走既有 framing（4B hdr len + hdr JSON + 8B payload len + payload）。旧地面端 `handle_client` else 分支 → `"Ignored message: <type>"`，连接不断、trace 接收不受影响 `[冻结]` 行为，必须有测试（R-C-13）。
- 校准回程的身份与通道（**不得制造第二个身份切换源**）：回程 header 携带自有 `cal_id` 字段用于关联，`run_id` **沿用当前连接身份**（待机 seq=0 或当前任务身份），因此**不触发 §3.7-4 的带内 hello 切换**、不触发地面 `begin_run`（否则校准消息会把地面 run 基线切到一个无 trace 的 id 上，制造出与 §3.7 同类的隐性状态失联）。发送语义与状态消息同规则（新增 `RemoteSender.enqueue_calibration(header, payload)`：`send_remote_frame` 成功即 `_note_completed`，不等 `ack_trace`；入队时同样计入 outstanding，保证关闭前排空、不静默丢弃）。地面按 `type` 分派到校准展示分支，绝不进入 `_accept_trace` / display FIFO / `_seen_traces`（R-C-17）。
- 校准短任务默认 `status_reporting=False`（§3.2）：**不发** `acquiring/stopping/stopped`，避免地面 `_final_stop_state`/任务基线被校准会话毒化（下一个采集任务的停止对账必须干净）。校准进度由 `calibration_*` 消息自身承载。
- 校准期间 USB 归属（**与 R-A-13 互斥模型一致：禁止第二条自开自关的 USB 路径**）：`cal_begin/cal_capture` 只在服务 `state ∈ {"idle_waiting","closed"}` 时执行（R-C-14）。实现方式：空中端用 `AcquisitionTaskSpec(status_reporting=False)` 经**同一个 `service.start_task`** 跑"校准采集任务"——服务照常持有 backend（closed→open/configure、idle→resume/reconfigure），校准编排驱动"取 repeats 条完整 sweep 后 `request_stop()`"。因此 `CalibrationSession(capturer=...)` 注入的是**服务/后端抽象**（测试注入 fake capturer 产生确定性 S11/S22），**不是** `librevna_usb_s11.acquire_sparameters`（后者每次自行 open/configure/close，服务持有会话时必然撞 claim——那正是 R-A-13 要拒绝的路径）。校准 config 与采集 config 不同时由 §3.4 `reconfigure` 承接（open_count 不变、configure_count+1 如实统计）。采集态收到 `cal_*` → 拒绝 + `command_ack` + 可读原因回传地面（不静默）；校准任务运行中收到采集 `start` 同样由服务单入口拒绝（同一判据、同一函数，禁止两套实现）。

### 5.5 逐条验收条款（C）

| 编号 | 条款 | 验证 |
|---|---|---|
| R-C-01 | 精简后主界面控件清单与 §5.1"保留"表逐一对应且 objectName 可解析；"移入"表控件全部存在于 `AirSettingsDialog` 且 objectName 不变；**无任何控件被删除**（前后清单对比）。 | `[自动]` + `[人工]` |
| R-C-02 | 第 1 轮既有功能契约零回归：`test_bscan_gui`/`test_ground_station_compact_ui`/`test_uav_gpr_qt_continuous`/`test_uav_gpr_qt_metadata`/`test_uav_gpr_qt_pipeline` 等既有断言一字不改全绿 `[冻结]`。 | `[自动]` 全量回归 |
| R-C-03 | 飞行中防误触：任务运行时打开 `AirSettingsDialog`，其中访问 LibreVNA / 改变冻结配置的控件逐项 `isEnabled()==False`；`stop_btn`、B-scan 滑块、GNSS 串口控件仍可操作；断言逐项列出（非"应该没问题"）。 | `[自动]` |
| R-C-04 | 对话框非模态：`AirSettingsDialog`/`CalibrationDialog` 用 `show()`+`setModal(False)`（生产代码 rg 不得出现二者 `exec()`）；打开时主界面仍可点击。 | `[自动]` + `[人工]` |
| R-C-05 | 校准会话状态机完整：`planned → in_progress → step_done(每步) → solved → activated`，失败任一点 → `failed` 携带原因；`cancelled` 独立终态；**绝不**从 `failed/cancelled` 迁移到 `activated`。 | `[自动]` |
| R-C-06 | 每步 repeats 条完整 sweep 的复数平均为 `np.mean(captures, axis=0)`（实部虚部同次平均）；repeats 默认 16、合法范围 1–256；0/负数/bool/非 int/NaN → 拒绝且不建会话。 | `[自动]` |
| R-C-07 | OSL 数学自洽：正演"已知 D/T/M + 理想标准件"生成的 m → 解算还原（rtol 1e-9）；P1/P2 两套 D/T/M 交换后结果必须不同（证明未共用）；频率轴不一致/点数不一致 → 拒绝；带内插值允许、带外外推拒绝。 | `[自动]` |
| R-C-08 | 质量指标真实计算并回传：逐条校正后的重复 RMS 与最大绝对误差 + `sample_count`；样本不足（<2）时字段为 `None` 并明确"样本不足"，不伪造 0。 | `[自动]` |
| R-C-09 | 落盘格式与 `uav_gpr_solt_calibration_v2` 兼容：write→read 往返一致、旧文件仍可读；六步 key 沿用 `CALIBRATION_STEPS`；不发明不兼容新格式（对齐参考文档 §4）。 | `[自动]` |
| R-C-10 | 空中端校准逻辑**搬家**到 `calibration_service.py`（纯 Python，不 import Qt），`MainWindow` 仅接线；既有 USB 出口（`acquire_sparameters`/`open_device`）仍只在空中端；搬家后既有测试全绿 `[冻结]`。迁移映射：`capture_calibration_standard`→`CalibrationSession.capture(step)`、`build_session_calibration`→`CalibrationSession.solve()`、`save_session_calibration`→`write_calibration_file()` 复用、`capture_background`→`build_reference_background()`、`active_calibration_components`→`CalibrationSession.active()`。 | `[自动]` 全量回归 + `[人工]` |
| R-C-11 | 地面入口可发现（`action_calibration`）；`CalibrationDialog` 展示步骤进度/质量/结果摘要；未连接空中端时明确显示"未连接"并使"开始校准"禁用（不静默失败）。 | `[自动]` |
| R-C-12 | 校准命令只用 `type="command"`+新 action；生产代码 rg 证明**未新增地面→空中 type**；`command_ack` 与旧端"Unknown remote command"忽略分支各有测试。 | `[自动]` + `[人工]` |
| R-C-13 | 旧地面端吸收新 type：真实 `ReceiverThread` 收到 `calibration_*` → `Ignored message`、连接不断、其后 trace 正常接收+ACK+显示。 | `[自动]` |
| R-C-14 | 校准与采集互斥（与 R-A-13 同一判据、同一函数，禁止两套实现）：采集态 `cal_*` 一律拒绝并回传原因；空闲态才可开始；校准会话进行中不得启动采集任务（服务 `start_task` 亦拒绝，理由写入 `last_stop_reason` 同款可观测字段）。 | `[自动]` |
| R-C-15 | 校准产物只落 `out_dir/_calibration/<cal_id>/`，原子写（`.tmp`+`os.replace`，R-X-06）；`_calibration` 不落入 run 目录；全链路基准 `local_saved` 口径不变 `[冻结]`。 | `[自动]` |
| R-C-16 | `cal_activate` 后新校准立即作用于后续采集（`active_calibration_components()` 返回新 profile，且有 trace 级证据）；地面端可查询当前生效校准摘要（`profile_summary`/`calibration_id` 进 acquisition metadata）。 | `[自动]` |
| R-C-17 | 校准消息不占 trace_index 序列、不进 display FIFO、不进 `_seen_traces` 去重表（地面 `_accept_trace` 不在其路径上）；有明确断言。 | `[自动]` |
| R-C-18 | 取消/断线：`cal_cancel`、地面断开、空中 FAULTED → 会话作废、已收 capture 文件保留但标记 incomplete、**不激活**半套校准；重连后携带旧 `cal_id` 的命令被拒绝（防跨会话错配），并记录一条日志。 | `[自动]` |
| R-C-19 | 背景域校验：背景与待减数据同域（RAW/OSL）、同频率轴逐元素一致；不匹配 → 该条 trace 拒绝并计数上报，绝不静默用错误背景，绝不把单通道背景广播到双通道。 | `[自动]` |
| R-C-20 | **承接 R-A-19**：新 run_id 的 hello 或 acquiring 任一即完成地面 run 切换（`begin_run`+任务基线+"采集中"）；同 run 的 acquiring 幂等；同 run 重连 hello 不重置基线。 | `[自动]` |
| R-C-21 | 精简后空中端仍能完成一次完整采集（fake 后端 offscreen）：连接→开始→停止→B-scan 出图→日志可读，即"精简没砍掉功能"。 | `[自动]` |

### 5.6 涉及文件与函数签名清单（C）

| 文件 | 变更 | 关键签名 |
|---|---|---|
| `uav_gpr_qt.py` | 修改（MainWindow/接线区域） | 新增 `AirSettingsDialog`（收纳 §5.1 控件、非模态）；`MainWindow._build_ui` 重排为操作员界面 + `service_state_label`；校准方法改为委托 `calibration_service`；`handle_remote_command` 增 `cal_*` 分派（含状态门 R-C-14） |
| `uav_gpr_ground_station.py` | 修改 | `ReceiverThread.handle_client` 增 `calibration_*` 分支 + hello 触发 run 切换（R-A-19/R-C-20）；`GroundStationWindow` 新增 `action_calibration`、`CalibrationDialog`、校准回程展示；`_on_acquisition_status` 既有分支语义不改 |
| `calibration_service.py` | **新增** | `CalibrationStep`（frozen dataclass：name/repeats/config_snapshot）；`CalibrationSession`（状态机 + `capture/solve/activate/cancel/statistics`）；`average_complex_captures(captures: Sequence[np.ndarray]) -> np.ndarray`；`solve_one_port_osl(freq_hz, m_open, m_short, m_load, ideal=(1,-1,0)) -> OSLTerms`；`quality_metrics(captures, terms) -> dict[str, float | None]`；`build_reference_background(...)`；写盘复用 `write_calibration_file` |
| `tests/test_round2_air_ui_compact.py` | 新增 | R-C-01…R-C-04、R-C-21 |
| `tests/test_round2_ground_calibration.py` | 新增 | R-C-05…R-C-20（fake 后端 + offscreen + 真实 ReceiverThread） |

---

## 6. 职责边界与共享文件冲突规避

### 6.1 三人职责边界（谁的缺陷谁负责，跨边界只允许"接线级"修改）

| 领域 | owner | 边界说明 |
|---|---|---|
| `PersistentLibreVnaBackend` / `reconfigure` / USB 生命周期 / `AcquisitionService` / `ContinuousAcquisitionWorker` / `LosslessProcessingQueue` | **engA** | engB/engC 需要新行为时向 engA 的公开入口提要求，不得直接改内部 |
| `RemoteSender`（含 `_run`/`outstanding`/hello/ack 处理）+ `disk_outbox.py` | **engB**（t5 起） | engA 的 t3 只允许改 `enqueue_trace/enqueue_status` 的**身份参数**与 hello 带内切换（§3.7），不得引入持久化逻辑；engC 只能调用公开方法 |
| `MainWindow` UI 布局 / `AirSettingsDialog` / `calibration_service.py` | **engC** | engA 的 t3 会改 `MainWindow` 的服务接线与"设置…"按钮宿主之外的控件归属；**次序保证**：t3 完成后 engC 才动 `MainWindow`（依赖 t6→t7） |
| 地面端 `uav_gpr_ground_station.py`（含 `ReceiverThread`、`GroundStationWindow`） | **engC**（t7） | engA/engB **不得**改地面端（t3/t5 文件域已排除）；R-A-19 的地面侧因此落在 engC 任务里 |
| `ground_trace_pipeline.py` / `pipeline_metrics.py` / `bscan_view.py` / `gnss_map_widget.py` | 本轮无人可改 | 三份任务卡的 outOfScope 一致 |
| 全链路基准 `docs/bench_round1_fullchain.py` | verifier（t9）只读运行 | 计数口径变化须由 owner 在实现报告中声明，verifier 不擅自改脚本 |

### 6.2 `uav_gpr_qt.py` 冲突热区与串行保证

`uav_gpr_qt.py` 三人都会碰，热区与归属：

| 热区 | 归属次序 | 规则 |
|---|---|---|
| `RemoteSender.__init__/_run/enqueue_*` | engA(§3.7 身份) → engB(outbox) → engC(只调用) | 后手不得回退前手语义；每步新增关键字参数必须带默认值保持旧调用兼容 |
| `ContinuousAcquisitionWorker` | engA → engB/engC 只读 | 服务化后 worker 不再自行 open/close/run_id 生成 |
| `MainWindow._build_ui` / 控件归属 | engA(服务接线) → engC(精简) | engA **不得**重排布局；engC 重排时保留全部 objectName |
| `handle_remote_command` | engA(run_id 护栏) → engC(`cal_*` 分派) | 同一函数两处顺序修改，后手基于前手 diff |

串行由任务卡依赖保证：`t1 → t3 → t5(→t6) → t7`，评审门 `t4/t6/t8` 逐段把关；**禁止**并行改 `uav_gpr_qt.py`（engB 的 t5 依赖 t3，engC 的 t7 依赖 t5+t6+t3）。若某评审返回 needs_revision 触发修复，修复任务仍属原 owner，下游依赖由系统重接到修复门。

### 6.3 交接点自检清单（后手开工前必读前手成果）

- engB 开工前：读 §3.7（身份制/带内 hello/单客户槽）+ engA t3 输出中的 `RemoteSender` changedPaths；确认 `run_id=None` 默认路径未被破坏。
- engC 开工前：读 §3（服务状态词汇 `state`）+ §3.7 + §4.4（outbox 计入 outstanding）+ t5 报告中新增/改动的 sender 行为；R-A-19/R-C-20 与 `_mirrored_air_run_id` 由 engC 落地。
- verifier：§7 的每条款对应测试文件 + `docs/round2_repro_runid_collision.py`（EXIT=0）+ 全链路基准退出码。

## 7. 测试、验收与未验证边界

### 7.1 测试环境固定命令（第2轮全量口径 = 分块运行器）

两个环境变量必须同时设置：`QT_QPA_PLATFORM=offscreen` + `QTWEBENGINE_DISABLE_SANDBOX=1`。

```powershell
& 'D:\博士任务\UVA_GPR_system\.venv312\Scripts\python.exe' -m py_compile uav_gpr_qt.py librevna/librevna_continuous.py librevna/librevna_transport.py
# 全量口径（唯一）：每模块独立子进程 + 以输出 ^OK 文本判定
& '...\.venv312\Scripts\python.exe' docs\run_tests_batched.py
# 专项与基准（单模块，可继续用 -m unittest）
& '...\.venv312\Scripts\python.exe' -m unittest tests.test_round2_resident_service
& '...\.venv312\Scripts\python.exe' -m unittest tests.test_round2_disk_outbox
& '...\.venv312\Scripts\python.exe' -m unittest tests.test_round2_air_ui_compact tests.test_round2_ground_calibration
& '...\.venv312\Scripts\python.exe' docs\bench_round1_fullchain.py
& '...\.venv312\Scripts\python.exe' docs\round2_repro_runid_collision.py
```

**全量口径说明（第2轮环境勘误，captain 定稿）**：本机 Qt 6.11.2 下，单进程 `python -m unittest discover -s tests` 会在约第 72 个用例被 QtWebEngine 击杀（FATAL `er.CalledOnValidBrowserThread` / `Must be called on Chrome_UIThread`）；t0=9027366 worktree 同环境复现同样崩溃 ⇒ **环境漂移，非第2轮回归**。因此全量统一改用 `docs/run_tests_batched.py`（每模块独立子进程）。判定规则：模块以 `^OK` 文本为准，**不看** returncode —— `test_round1_defaults`、`test_round1_review_fixes` 等含 QWebEngineView 的模块会在解释器拆除阶段以 `0xC0000005` 退出（tests 本身全 OK），这类记为 `PASS*`/shutdown-crash 注记**豁免**；只有真 FAIL/ERROR/超时才算不通过。`RUN 总数`由各模块 `Ran N tests` 求和。

基线：**732** 条 OK（t0 归档树实证；"716"为第 1 轮第二次修复报告时的陈旧叙事数）；基准 500:500 全等、端到端 p95 17.2ms、Qt 提交 p95 1.28ms、exit 0。第 2 轮结束时的**最低**要求：`docs\run_tests_batched.py` OVERALL OK（零真 FAIL/超时，`PASS*` 豁免除外）且总数 ≥ 732 + 三套专项新增（t3 实测 763 = 732 + 31）；基准退出码 0 且计数全等；`round2_repro_runid_collision.py` 退出码 0。

### 7.2 验收数字纪律

任何写入 t3/t5/t7 报告与 t11 终报的性能/对账数字必须逐字来自真实运行输出（`bench_round1_fullchain.py` / 专项计时），并附命令与退出码；禁止转述、估算或占位。写盘/补传类开销必须给出 p50/p95/max。

### 7.3 明确未验证（真机相关，第 2 轮不得声称已验证）

沿用第 1 轮口径并追加：

1. LibreVNA 真机连续采集与常驻会话（USB 长时间保持 claim、`SET_IDLE` 待命后重发的真实设备行为、`reconfigure` 的真实 ACK 行为）；
2. 真机"停止→待命→再启动"是否触发设备固件侧异常/热插拔重枚举；
3. 串口 GNSS 真实数据流；
4. 跨机无线链路端到端延迟（基准为同机同进程 `perf_counter` 时钟；跨机需两端时钟同步）；
5. 真实 GPU 屏幕绘制帧率（offscreen 无 GPU 合成）；
6. **新增**：outbox 在真实断电/磁盘错误下的表现（测试用 subprocess 硬杀 + 临时目录模拟，非真实掉电）；
7. **新增**：真机 Open/Short/Load 六步物理校准质量（参考文档 §8 明确要求现场验收；本轮只到合成自洽 + fake 后端编排）。

## 8. 本文档对任务卡 acceptance 的自检（t1）

| acceptance 条目 | 覆盖位置 |
|---|---|
| 文档存在且覆盖三条工作流各自的目标/非目标/涉及文件与函数签名 | §3.0/§3.5、§4.0/§4.7、§5.0/§5.6、§2 表 |
| 常驻采集服务的状态机与命令接口（stop 后保持 USB、SET_IDLE、受控停止升级为服务命令的语义与调用方） | §3.1（状态机）、§3.2（`AcquisitionService` 签名）、§3.3 R-A-01/03/05（保持 USB / SET_IDLE / 三调用方同一语义） |
| 磁盘 outbox 的原子写格式、ACK 回执持久化字段、重启扫描与 run_id/trace_index 续传规则、与 RemoteSender outstanding 计数衔接点 | §4.1（格式/目录/文件编码）、§4.2（持久化=帧文件存在性 + 划除）、§4.4（扫描时机 + 身份原样重放）、§4.4 计数衔接（写死计入 outstanding）、§4.6 R-B-05 |
| 空中端 UI 精简的保留/移除控件清单，地面校准的入口、流程步骤与不依赖真机的可测试边界 | §5.1（两张清单 + 禁止删除项）、§5.2（入口 + 8 步流程 + 边界 1–5） |
| 逐条标注哪些要求受 `uav-gpr-telemetry-v1` framing 不可变约束（只允许新增消息类型且旧接收端忽略） | R-X-02（总纲）、§3.7-4（带内 hello 复用既有 type）、§4.6 R-B-09、§5.4（四条协议约束 + 雷区说明）、各条款 `[冻结]` 标记 |
| engA/engB/engC 职责边界与共享文件 `uav_gpr_qt.py` 的串行改动次序 | §2（写入权表 + 三步次序）、§6.1（边界表）、§6.2（热区 + 串行保证）、§6.3（交接自检） |
| 每条需求可被自动化测试或人工核查判定，不含愿望式表述 | 全部条款带 `[自动]`/`[人工]` 标注；§7.1 给出固定命令；§7.2 数字纪律 |

补充说明（非 acceptance 要求，但属实现约束的固化）：本文档在评审 engB 提出的 `run_id` 死锁取证（`docs/round2_repro_runid_collision.py`）基础上，把方案 (a) 具体化为"通道常驻 + 消息携带身份 + 带内 hello 切换 + 地面双触发 run 切换"，并给出否决项与原因（§3.7）；t3/t5/t7 的实现评审一律以 §3.7 为准，不接受"每任务新建连接"或"全局连续编号"变体。



