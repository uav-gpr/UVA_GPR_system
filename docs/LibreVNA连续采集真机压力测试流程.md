# LibreVNA 连续采集真机地面压力测试流程（阶段 5）

| 项目 | 内容 |
| --- | --- |
| 目标项目 | `E:\UVA_GPR_system` |
| 对应路线图 | `docs/LibreVNA连续采集功能开发路线图.md` §15.6、§16 阶段 5 |
| 测试工具 | `librevna/field_test.py`（`probe` / `bench` / `longrun` / `failures`） |
| 前置要求 | 阶段 1～4 已通过软件层验收；**真实 LibreVNA 通过 USB 连接** |
| 编制日期 | 2026-08-15 |

本文档供现场人员对**真实 LibreVNA 硬件**执行阶段 5 压力测试。无硬件时工具会
干净报错并返回非零退出码；开发机可用 `--simulate` 验证工具本身，但**模拟跑通
不等于真机通过**。

## 0. 术语：四种结果必须区分

| 结果 | 含义 | 判定 |
| --- | --- | --- |
| 命令运行成功 | 进程正常返回、JSON 可解析 | 退出码 `0` |
| 软件模拟通过 | `--simulate` 下不变量与清理全部通过 | `rc=0` 且 `acceptance_eligible=false` |
| 具备真机验收资格 | 真机 + 全部不变量 + 清理 + 时长 + 基线门槛 | `rc=0` 且 `acceptance_eligible=true` |
| 阶段 5 真机通过 | 全部现场项（含人工拔插）逐一完成并记录 | 汇总到完成报告 |

**关键规则**：

- `rc=0` 只表示"命令自身全部必需不变量 + 清理通过"。
- 完整性失败、零有效数据、保存/网络数量不一致、清理失败、线程残留、close 抛错
  或 close 后仍打开，一律返回非零（`2`）并在 JSON `failure_reasons` 写明原因。
- `--simulate` 永不验收合格；短时间 smoke 即使 `rc=0` 也是 `acceptance_eligible=false`。
- 每条报告 JSON 都含 `passed`、`failure_reasons`、`cleanup`、`acceptance_eligible`
  （failures 顶层同样有 cleanup）。
- `acceptance_eligible` 只是单条命令具备资格，**不代表阶段 5 整体通过**：人工
  USB 拔插、GNSS 串口、长稳与真实链路仍须单独完成。

## 1. 环境与前置检查

- Python：项目 `.venv312/Scripts/python.exe`（含 PyUSB 与 libusb DLL）。
- USB：LibreVNA 通过 USB 连接（`VID=0x1209 PID=0x4121`），驱动可用。
- **USB 独占**：测试期间不要同时运行 GUI 采集或 LibreVNA-GUI。
- 工作目录：`E:\UVA_GPR_system`；记录被测设备固件/硬件版本（`probe` 打印）。

## 1.1 互斥执行模式（强制）

probe/bench/longrun/failures 必须**显式二选一**指定执行模式，缺一或冲突即拒绝
（返回非零，且不构造任何 transport、不枚举 USB）：

- `--simulate`：软件模拟（`StreamingSimulatedTransport`），永不连接真实设备；
- `--hardware`：**用户显式确认连接、控制真实 LibreVNA**；`--hardware` 不会通过默认
  值隐式开启——缺失 `--simulate` 不再隐式视为真机。

```powershell
# 软件模拟（显式）
.venv312\Scripts\python.exe -m librevna.field_test probe --simulate
# 真机（显式确认）
.venv312\Scripts\python.exe -m librevna.field_test probe --hardware
```

## 2. 设备确认（§15.6 第 1～3 项）

```powershell
.venv312\Scripts\python.exe -m librevna.field_test probe --hardware
.venv312\Scripts\python.exe -m librevna.field_test probe --hardware --points 101 --sample 20
```

probe 的 `rc=0` 需同时满足（任一失败都会在 `failure_reasons` 列明并返回非零）：

- `sample_complete == sample`；
- `records_parsed` 足够且 `full_cycles_observed >= sample`；
- `point_sequence_strict_0_to_n_minus_1 == true`（容忍 configure 后未同步前缀）；
- `s11_s22_same_datapoint == true`（**禁止空集合**：`records_parsed` 为 0 必须失败）；
- `every_sweep_shape_and_finite == true`；
- integrity 的 `duplicate/jump/out_of_range/invalid/dropped/incomplete` 全为 0；
- backend 与 transport 已安全关闭（`cleanup.ok == true`）且
  `cleanup.release_within_2_5s == true`。

若 `error_type == "LibreVnaDeviceNotFoundError"`：设备未识别，检查 USB 与驱动，
**不要**把该结果当作验收通过。

## 3. 分层基准（§15.6 第 4 项）

### 3.1 纯 USB 基线（同步直接 backend，非验收）

```powershell
.venv312\Scripts\python.exe -m librevna.field_test bench --hardware --layers usb --seconds 60 --json-out bench_usb.json
```

- `topology` 为 `synchronous component microbenchmark`；
- `processed == 0`、`saved == 0`、`enqueued == 0`；
- `acceptance_eligible=false`——这是**基线数据**，不是验收结果；
- 保存该 JSON 供第 3.3 节使用（基线时长需 >= 60s）。

### 3.2 生产线程拓扑基准（默认验收路径）

正式验收命令必须显式使用**正数 `--interval`**（如 `--interval 1`），保证 60 秒/
30 分钟运行产生足够 schedule_error 样本以满足 §14.5 门槛；`interval=0`（逐道选择）
无 schedule 样本，`schedule_applicable=false`，**不能获得正式验收资格**。

```powershell
.venv312\Scripts\python.exe -m librevna.field_test bench --hardware --layers usb,proc --seconds 60 --interval 1 --json-out bench_proc.json
.venv312\Scripts\python.exe -m librevna.field_test bench --hardware --layers usb,proc,disk --seconds 60 --interval 1 --json-out bench_disk.json
.venv312\Scripts\python.exe -m librevna.field_test bench --hardware --layers usb,proc,disk,net --seconds 60 --interval 1 --baseline-json bench_usb.json --json-out bench_net.json
```

任何含 `proc` 的层默认走**生产线程拓扑**
（`ContinuousAcquisitionWorker` + `BoundedQueue` + `ProcessingConsumer`）：
USB 生产线程只 acquire/select/put；处理、保存与遥测压缩在 consumer 线程；
RemoteSender 独立线程发送。`--mode sync` 为显式同步组件微基准，`acceptance_eligible`
恒 false，**不能**作为生产架构性能或验收结果。

`rc=0` 需同时满足（fail-closed，任一不满足即失败）：

- `open_count==1`、`configure_count==1`、`reconnect_count==0`；
- `processed==selected`、`saved==selected`、`enqueued==selected`；
- **backend integrity**：`dropped/incomplete/duplicate/jump/out_of_range/invalid/
  sweep_timeouts/pending_overflows` 全为 0，`pending_depth_current==0`；
- **processing_queue**：`overflows==0`、`dropped==0`、`depth==0`（正常收尾后）；
- **scheduler**：`missed_slots_total==0`；
- **网络**（net 层）：`received==unique==enqueued`、
  `duplicates/missing/out_of_order==0`、sender 队列最终排空；
- worker/consumer 的 `errors`/`fatal_error` 为空（`errors` 不含 `worker.failed`）；
- `cleanup.ok==true`（每类资源独立字段，见第 4 节）。

### 3.3 90% 基线比较（§14 第 6 项）

正式验收命令必须同时包含正数 `--interval` 与匹配的 `--baseline-json`：

```powershell
.venv312\Scripts\python.exe -m librevna.field_test bench --hardware --layers usb,proc,disk,net --seconds 60 --interval 1 --baseline-json bench_usb.json
```

**基线必须来自同一设备与等价采集配置**（同 firmware/protocol/hardware、同
points/start/stop/ifbw/power/dwell/sweep_timeout），禁止使用不匹配的历史基线。
基线 JSON 只有**同时**满足以下条件才可用于正式 90% 比较：
JSON 可解析、`passed==true`、`failure_reasons` 为空、`cleanup.ok==true`、
`mode` 与当前运行一致（硬件运行禁止用 simulate 基线）、`layers==["usb"]`、
`topology` 是同步纯 USB 基线、`processed/saved/enqueued==0`、`sweeps_per_s` 有限
正数、基线时长 >= 60s、采集参数一致、真机时设备 firmware/protocol/hardware 标识一致。

无基线 / 无效基线 / 参数不一致：该命令**仅用于冒烟**（`acceptance_eligible` 必然为
false，不得作为正式验收证据）；`baseline_90_percent_met` 标记 `"未评估"`、性能门不
适用。低于 90% 时 `passed=false`、rc=2。

> **baseline 数值 schema**：baseline JSON 的计数、时长、速率及配置数值必须使用标准
> JSON 数字；bool、数字字符串、NaN、Infinity 和 -Infinity 均视为损坏 baseline，
> 不参与验收（`baseline_90_percent_met` 标记"未评估（基线无法读取或包含非标准非
> 有限数值）"，`acceptance_eligible=false`）。`sweeps_per_s`/`elapsed_s` 须有限实数、
> `processed/saved/enqueued` 须精确整数 0、`config.points/ifbw_hz` 须正整数、
> `config.dwell_us/count` 须非负整数、`config.start_mhz/stop_mhz/power_dbm` 须有限
> 实数、`config.sweep_timeout_s` 须 None 或有限正实数。

> **baseline 容器 schema**：baseline 不仅数值类型必须正确，顶层 `layers`、`config`、
> `device`、`cleanup` 等容器结构也必须与工具生成报告一致；`layers` 须严格为
> `["usb"]`（list 类型），损坏或类型错误的嵌套结构只会标记为"未评估"，不参与验收，
> 也不得导致测试工具崩溃。

### 3.4 验收资格使用实际时长 + 调度误差门槛

`acceptance_eligible` 使用**实际 `elapsed_s`**，不是命令请求的 `--seconds/--duration`：

- `--seconds 60 --count 1` 快速完成（elapsed 约 0.1s）→ `acceptance_eligible=false`；
- longrun 提前结束同样不得获得资格；
- bench 真机需实际 >= 60s，longrun 真机需实际 >= 1800s。

依赖**有效纯 USB 基线**执行 roadmap §14.5 门槛：由基线 `sweeps_per_s` 换算完整
sweep 周期（`1000/rate` ms），要求 `schedule_error_ms.p95` 不超过该周期；缺少样本、
p95 非有限或超限时 `schedule_error_within_sweep_period=false` → 不得获得验收资格
（报告字段 `schedule_error_within_sweep_period`/`sweep_period_ms`/`schedule_error_p95_ms`）。

**任一必需门不适用即 `acceptance_eligible=false`**：

- `interval=0`（逐道选择）无 schedule_error 样本：`schedule_applicable=false`、不得
  获得正式验收资格；正式验收必须使用正数 `--interval`（§3.2）。
- 未提供 `--baseline-json`：性能门（90% + schedule 门槛）不适用，`acceptance_eligible`
  必然为 false（仅冒烟）。

### 3.5 严格 JSON 与人工阶段门

- 报告使用严格序列化（`allow_nan=False`，无 `default=str` 静默转换）：callable、
  任意不支持对象、NaN/Inf 都会导致 `passed=false`、`acceptance_eligible=false`、
  rc=2；若报告无法序列化，输出一个最小、严格合法的诊断 JSON 并列出非法字段路径
  与类型。
- `--json-out` 写入失败也返回非零并明确诊断。
- **RSS/队列趋势（内存泄漏、pending 增长）属于人工阶段门**：工具如实输出
  `resources.rss_mb` 与队列深度序列，但**不**自动判定"无泄漏/稳定"；该判定由审查者
  结合运行时间曲线人工完成。工具的自动 fail-closed 只覆盖 §3.2 列出的可判定不变量。

## 4. 清理与线程生命周期（fail-closed）

`cleanup` 为每类实际存在的资源设置独立字段：

- `backend_closed`、`producer_exited`（模拟 producer）、`sender_exited`、
  `receiver_exited`、`feeder_exited`（GNSS）、`monitor_exited`、`controller_exited`、
  `worker_exited`（QThread）、`consumer_exited`（QThread）；
- `cleanup.ok` 必须是**所有实际存在资源均已安全退出**；
- `cleanup.clean`：清理操作（close/request_stop/join/wait/is_alive/状态探测）是否有
  异常；任何异常置 false 并留下含角色、操作、异常类型、消息的 failure_reason；
- **`cleanup.clean_shutdown`（= ok and clean）是硬性通过门**：只要
  `clean_shutdown != true`，则 `passed=false`、`acceptance_eligible=false`、rc=2、
  failure_reasons 非空，基线不得用于正式比较；
- `stop_to_usb_release_s` 与 `release_within_2_5s`（roadmap §14.8）。

`stop_to_usb_release_s >= 2.5` 时（路线图要求**严格小于** 2.5s，2.5s 本身不达标）：
`release_within_2_5s=false`、明确 failure_reason、`passed=false`、`acceptance_eligible=false`。
没有取得生产 stop→release 测量时也不得获得验收资格。

不退出即失败：sender.stop 返回 False、receiver/feeder/monitor/controller 残留、
worker/consumer QThread `isRunning()` 为 True（从真实 `worker.consumer` 读取，
不使用 `threading.enumerate()` 冒充 Qt QThread 检查）、模拟 producer 未退出，都导致
`cleanup.ok=false` 且 rc 非零。不使用 `QThread.terminate()`。返回时
`threading.enumerate()` 不得再有 `field-test-*` 或 `uav-gpr-remote-sender` Python 线程。

清理操作（receiver.close()/is_alive()、线程 join/request_stop）抛错必须结构化
fail-closed 进入 `failure_reasons`，不得逃出报告流程：close() 抛错但线程已退出时
`receiver_exited` 可为 true 但记 failure_reason（passed=false、rc=2）；close() 抛错
且线程仍活跃时 `receiver_exited=false`、`cleanup.ok=false`。

worker 因有限 count / 错误 / 提前结束时，harness 立即 stop 并 join controller。

## 5. 稳定性测试（§15.6 第 5 项）

正式验收的 longrun 必须是**完整命令**：同时包含正数 `--interval`（如 `--interval 1`）
与匹配当前设备/配置/模式的 `--baseline-json bench_usb.json`。短时冒烟命令可保留为
smoke（标记为**仅用于冒烟**：`acceptance_eligible` 必然为 false，不得作为正式验收证据）。

```powershell
# 冒烟（仅用于冒烟，不具备正式验收资格）
.venv312\Scripts\python.exe -m librevna.field_test longrun --hardware --duration 60 --sample-every 10 --json-out short.json

# 至少 30 分钟稳定性（正式验收：含正 interval + 匹配基线）
.venv312\Scripts\python.exe -m librevna.field_test longrun --hardware --duration 1800 --interval 1 --baseline-json bench_usb.json --sample-every 60 --json-out stable_30min.json

# 1～2 小时压力（正式验收：含正 interval + 匹配基线）
.venv312\Scripts\python.exe -m librevna.field_test longrun --hardware --duration 7200 --interval 1 --baseline-json bench_usb.json --sample-every 300 --json-out stress_2h.json
```

`longrun` 恒为生产线程拓扑，`command` 恒为 `"longrun"`。验收时长门槛：**bench
真机 >= 60s；longrun 真机 >= 1800s**。60s 的 longrun 只是 smoke，`acceptance_eligible`
恒 false。观察点：`dropped`/`sweep_timeouts` 不增长、`pending_depth_peak` 与
`processing_queue.depth_peak` 稳定、`gnss.age_ms` 在 `--gnss-max-age` 内、
`resources.rss_mb` 不线性上涨、`stop_to_usb_release_s < 2.5s`（RSS/队列趋势为人工
阶段门）。

## 6. 失败场景（§15.6 第 6～8 项，自动化部分）

```powershell
.venv312\Scripts\python.exe -m librevna.field_test failures --hardware
```

- `pause_resume`：3 轮 pause/resume，passed 由实际条件计算（open/configure==1、
  未 reopen/reconfigure、trace_index 连续、取得预期道数、integrity 全 0、清理成功）。
- `gnss_cache_no_fix_stale`：**只**覆盖 `GnssFixCache` 无 Fix/陈旧语义与 metadata
  正确性；`serial_disconnect_covered=false`——不等于 GNSS 串口断开测试。
- `network_drop`：无监听端端口使 sender 连接失败；确认
  `sender_connect_failure_observed=true`；真实处理 + `enqueue_trace()` + 本地临时
  目录保存；记录 acquired/selected/processed/saved/enqueued 与远程队列深度；
  `network_did_not_stop_usb` 由事件与计数计算；结束后 sender 退出、backend 释放。
  已知风险：RemoteSender 发送失败重连时可能重发已确认 trace（本轮不改）。

failures 顶层也有 `cleanup`：汇总资源型子场景（pause_resume、network_drop），
GNSS 缓存场景标记无资源。每个场景先初始化 result 再执行，异常不会丢失原始
failure/cleanup。

### 6.1 USB 真机拔插（§15.6 第 6 项，现场人员执行）

1. 启动 30 分钟 `longrun`；运行约 5 分钟后现场人员**物理拔出** USB 线。
2. 观察：backend 进入 FAULTED / `disconnects` 计数，**不得**把断开前后数据拼成
   同一道；插回后按当前策略（fail-closed，需 close 后重新 open；自动重连未实现）。
3. 记录拔插时间点与日志，人工核对 `connection_generation` 边界。

### 6.2 GNSS 串口断开/重连（现场未验证项）

工具只覆盖缓存语义。真机验证需在运行中拔插 GNSS 串口，观察 `GnssReaderThread`
退避重开、`gnss_valid=false`、`gnss_age_ms` 递增而 `flight_height_agl_m` 不伪造。
**无硬件不执行**。

### 6.3 网络断开

关闭地面站/回环接收端，确认空中端本地继续采集保存；真机上对拍地面站显示。

## 7. 记录清单（路线图 §14 全部指标）

每次测试保存 `--json-out`；至少记录设备 firmware/protocol/硬件版本与配置；
`sweeps_per_s`/`selected_per_s` 与各 completed/dropped/incomplete/duplicate/jump/
out_of_range/invalid；`missed_slots_total`、`actual_interval_s` 与
`schedule_error_ms`（mean/p95/max）；`pending_depth_current/peak/overflows`；
`processing_queue` 深度/峰值/溢出/丢弃；`gnss.age_ms`/`valid`；timings（生产路径
`acquire_s=None`——生产 API 不暴露逐道 acquire，如实为 None，不伪造）；资源
CPU/RSS；open/configure/reconnect 次数；`cleanup.stop_to_usb_release_s`。

## 8. 非零退出码的完整条件

| 场景 | 退出码 |
| --- | --- |
| CLI 参数非法（sample/seconds/duration/points/rate/频率/interval/count/age 等） | `1` |
| 设备未找到 / open 失败（真机无硬件） | `2` |
| probe 任一不变量失败 | `2` |
| bench/longrun 完整性、零有效数据、保存/网络数量不一致、integrity 非零、队列溢出/未排空、missed_slots 非零、worker failed | `2` |
| 清理失败：close 抛错或仍打开、任何线程残留、release 达 2.5s（严格 < 不达标） | `2` |
| 90% 基线低于门槛（提供了有效 `--baseline-json`） | `2` |
| 全部不变量 + 清理通过 | `0` |

## 9. 冻结代码缺陷（已修复）

`ContinuousAcquisitionWorker.statistics()` 曾把 bound method
`processing_queue.statistics` 放进 `session_statistics`，导致 autosave/遥测 JSON
序列化崩溃。该缺陷已在 `uav_gpr_qt.py` 修复（改为调用 `.statistics()` 取得快照，
`statistics()` 与 `_emit_statistics` 均无 callable，可被 `json.dumps()` 直接序列化），
并有 4 项生产回归测试（processing_queue 是 dict、递归无 callable、json.dumps 通过、
真实 worker→consumer→save_trace 无需 workaround 保存、遥测 metadata 可序列化）。
field_test 不再有任何 sanitizer/workaround；若再次出现不可序列化对象，consumer
会崩溃 → `worker.failed` → 生产 benchmark 明确失败。

## 10. 回退

阶段 5 交付物为 `librevna/field_test.py`、`tests/test_field_test.py`、流程文档，以及
`uav_gpr_qt.py` 中 `statistics()`/`_emit_statistics()` 的序列化修复（该修复是正向
行为保持，`statistics()` 返回内容不变，只是不再含 bound method）。删除工具文件即可
移除诊断；主程序采集默认仍为 legacy。

## 11. 无硬件时的最终结论

阶段 5 软件工具继续收口；**真实设备曾被一次越权 probe 意外访问**（详见完成报告
披露），但 ≥30 分钟长稳、USB 拔插、真实 GNSS/TCP、1–2 小时压力及人工资源判定
尚未获授权或尚未执行，因此**阶段 5 正式现场验收未完成**。不得把该次 probe 当作
正式阶段验收证据，也不得声称正式验收通过。
