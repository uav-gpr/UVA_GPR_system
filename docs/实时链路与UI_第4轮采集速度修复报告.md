# 采集速度修复（第4轮）：默认高速连续引擎 + 引擎开关可见化

| 项目 | 内容 |
| --- | --- |
| 缺陷 | 真机实测 **约 2.75 秒/帧**，控制台每帧打印一次 `Connected` |
| 目标 | 默认且**可见**地走高速连续采集，让用户本人真机自测速度 |
| 改动范围 | `uav_gpr_qt.py`（空中端默认值 + 连接栏新增引擎行）、`tests/test_uav_gpr_qt_continuous.py`（契约更新）、`tests/test_ui3_engine_default.py`（新增） |
| 协议 | **零改动**：未新增/修改任何 framing 字段、消息 type 或既有字段类型 |
| git | 未执行任何 commit / push / reset / checkout / clean |

## 1. 根因（captain 已逐行核实，本轮不重挖）

| # | 位置 | 事实 |
| --- | --- | --- |
| ① | `uav_gpr_ground_station.py:1000-1005` | 地面站 `command_params` 只发 `count/interval_s/single`，不含引擎选择 |
| ② | `uav_gpr_qt.py` `start_acquisition` | 依 `continuous_enabled` 勾选框二选一：勾上走常驻高速连续采集，否则走 legacy `AcquisitionWorker` |
| ③ | `DEFAULT_ACQUISITION_MODE`（原 `"legacy"`） | 勾选框默认值取环境变量 `UAV_GPR_ACQUISITION_MODE`，未设即 legacy |
| ④ | 第3轮界面精简 | `addWidget(self.continuous_enabled)` 命中 0 —— 操作栏被删后勾选框**失去布局宿主**，界面上已无法打开它 |
| ⑤ | `uav_gpr_qt.py` legacy worker 每帧 `acquire_sparameters` | `librevna_usb_s11.py:362 open_device` … `:393-398` 释放关闭 → 2.75 s/帧 + 每帧一行 `Connected` |

## 2. 改动清单

### 2.1 默认锁高速连续引擎（③ 的修复）

- `DEFAULT_ACQUISITION_MODE`：`"legacy"` → **`"continuous"`**（`uav_gpr_qt.py:253`）。
- `acquisition_mode()`：缺省 → continuous；**非法值也回退 continuous**（不再静默掉进慢引擎）。
- legacy 退路保留且**只能显式选**：`UAV_GPR_ACQUISITION_MODE=legacy` 或界面下拉。
- 新增常量：`ENGINE_MODE_CONTINUOUS/LEGACY`、`ENGINE_MODE_LABEL_*`。

### 2.2 引擎开关可见化（④ 的修复）

- 连接栏 `_connection_bar()` **第 3 行**新增：`采集引擎:` + 下拉 + 既有 `continuous_enabled` 勾选框 + 只读回显标签。
  - 勾选框从此**有布局宿主**（第3轮精简正是丢了宿主），且默认打勾。
  - 下拉与勾选框**双向同步**（`_on_engine_mode_combo_changed` / `_on_continuous_checkbox_toggled`），同一语义两个入口，绝不各说各话。
  - 回显标签写明：`当前引擎：连续 · 高速（USB 只开一次） · 默认`。
- **不复活** `operator_bar`、**不新增**主界面本地开始/停止按钮 —— 第3轮"远程命令驱动采集"契约不回退。
- 任务运行中下拉/勾选框随既有 `_set_task_controls_locked` 一起锁定（不得中途换引擎）。

### 2.3 引擎状态可见化（防止"新代码没生效"被误判）

- 启动横幅：`采集引擎: continuous（高速常驻，USB 只开一次）｜出厂默认`。
- 每次启动采集写明本轮引擎：`本任务引擎: continuous…` / `本任务引擎: legacy（…真机约 2.75 s/帧…）`。
- **状态区「连续模式」行两态文案（t3 修复后，两态都写明）**：
  - **continuous 态**（默认引擎，任务未运行）：`连续模式: 高速连续采集（默认，未运行）`；
    一轮任务收尾后（USB 待命、下次启动即用同一高速路径）：`连续模式: 高速连续采集（待命，下次启动即用）`。
    —— 两处都**不含**「未启用」（旧文案写"未启用"与同屏引擎行"当前引擎：连续 · 高速 · 默认"自相矛盾，
    是评审 F1 点名的状态不一致）。
  - **legacy 态**（显式选了慢引擎）：`连续模式: 未启用（本轮 legacy 引擎，每帧开关一次 USB，较慢）；下次要高速请把"采集引擎"切回"高速连续采集（推荐，默认）"`；
    启动时 env=legacy（UI 未运行）则为 `连续模式: 未启用（当前 legacy 引擎，每帧开关一次 USB，较慢）`。
  - 四条文案都在 `uav_gpr_qt.py` 顶部以常量定义（`CONTINUOUS_STATUS_READY/_STANDBY/_LEGACY/_LEGACY_RUNTIME`），
    由 `test_continuous_status_label_has_no_disabled_ambiguity` 逐条断言（含"continuous 态不得含未启用"、
    "只有 legacy 态才允许未启用"）。

### 2.4 未改动（刻意）

- framing / 消息 type / `command_params` 字段：**零改动**（地面站无需知道引擎，空中端默认即是高速引擎）。
- legacy 引擎代码：**完整保留**，作为调试退路。

### 2.5 本轮一并写入的账目内文档（t3 补齐披露）

- `docs/round2_requirements_and_interfaces.md`：给第2轮 §3 非目标里"feature flag
  `UAV_GPR_ACQUISITION_MODE` 默认 legacy `[冻结]`"一句加**取代注记**——注记明确
  `[冻结]` 的对象是 legacy worker 的逐帧 open/close 行为本身（仍不变），被取代的只是
  "默认 legacy"这一句；第2轮历史正文**未被改写**。
- `docs/实时链路与UI_第4轮采集速度修复报告.md`（本文件）：第4轮修复报告本体。
- 这两条在 t1 首次交付时**未申报进 changedPaths**，t3 修复轮如实补上（评审 F2）。

## 3. 既有测试的透明更新（0 静默删除）

`tests/test_uav_gpr_qt_continuous.py::FeatureFlagTests`：旧契约是"默认 / 非法值 → legacy"，
新契约是"默认 / 非法值 → continuous；legacy 只能显式选"。**断言意图逐条保留，只反转默认值**：

| 旧测试名 | 新测试名 | 变化 |
| --- | --- | --- |
| `test_default_mode_is_legacy` | `test_default_mode_is_continuous` | 契约反转（默认值本身改了）＋ 补断言 `DEFAULT_ACQUISITION_MODE == "continuous"` |
| `test_invalid_env_value_falls_back_to_legacy` | `test_invalid_env_value_falls_back_to_continuous` | 回退目标 = 当前默认 |
| `test_env_var_selects_continuous` | 同名保留 | 不变 |
| —— | `test_env_var_selects_legacy`（新增） | 补上 legacy **显式**退路的证明 |
| —— | `test_default_window_starts_with_continuous_engine`（新增） | 端到端默认值证明（跨模块与 `test_ui3_engine_default` 有少量重叠，刻意保留：契约模块要能独立判红） |

模块 docstring 里"feature flag 默认回退 legacy"一句也同步改为"默认 continuous ⋯ legacy 需显式选择"（同一处契约描述，非断言）。

无任何断言被删除；`test_unchecked_flag_routes_to_legacy_worker` 等显式 `setChecked(False)`
的既有测试**不需要改**（语义未变：显式取消勾选仍走 legacy）。

## 4. 新增测试

`tests/test_ui3_engine_default.py`（17 条）把根因 ①～⑤ 各自钉成断言：

- ①：地面站 `command_params()` 仍只有 `count/interval_s/single`，源码与动作字典均无 `"engine"` 键——引擎选择**不靠协议**（这是"不改地面端"的证据）。
- ②/③：默认值 = continuous；缺省窗口勾选框已勾选；默认启动真的路由到 `ContinuousAcquisitionWorker`；legacy 仅在显式取消勾选时可用。
- **验收核心** `test_default_start_never_calls_acquire_sparameters`：用真实 `MainWindow` + `_FlowTransport`（协议线假 LibreVNA）跑完默认路径的完整采集，`patch acquire_sparameters`（side_effect=AssertionError）计数必须为 0，且 `AcquisitionService` 真的进入 acquiring、3 条 trace 真的出图、USB 经持久后端打开——**非文字描述、非替身**。
- ④：勾选框必须在 centralWidget 子控件集合里且可见、且**确实在某个 layout 的 item 列表中被 addWidget**；引擎行文案含"高速连续采集"；`operator_bar`/`diagnostics_group` 不复活、主界面仍无本地任务按钮；下拉 ↔ 勾选框双向同步；任务中锁定；两种引擎状态下状态区文案各自可验证（不撒谎）。
- ⑤：源码级证据 —— legacy worker 仍在调 `acquire_sparameters`；continuous 走 `AcquisitionService.start_task` 常驻服务。

## 5. 验证

- `docs/run_tests_batched.py` 全量：**951 tests / 31 modules，failed modules: none，OVERALL: OK**（exit 0）。
  - 基线 931 / 30 modules → 951 / 31 modules：新增模块 `test_ui3_engine_default`（t1 17 条 + t3 状态文案断言 1 条 = 18 条）+ `FeatureFlagTests` 契约更新净增 2 条。
  - 唯一的 `PASS*` 是 `test_round1_review_fixes`（Qt 6.11.2 WebEngine 退出期 0xC0000005，环境既有漂移；测试层已打印 OK，runner 按 captain 裁决策略记为 PASS）。
- 契约 verify 三条命令逐条执行：`py_compile` exit 0；受影响模块 4 个 `Ran 128 tests ... OK`；全量 batched OVERALL OK。
- 原始日志：`full_regression_r4.log`。

## 6. 用户侧交付

真机自测指引（大白话 ≤1 页）：`docs/第3轮_真机高速采集自测指引.md`（路径沿用任务卡声明的交付物名）—— 怎么看引擎行、
三个"已走高速"的观察点（控制台不再刷 `Connected` —— 已核实该行只由
`librevna_usb_s11.py:320/367` 的 legacy `acquire_s11`/`acquire_sparameters` 打印，
持久传输层 `librevna_transport.py` 全无 print；每帧 ≤40 ms 而非 2–3 秒；
状态区 acquiring + 引擎行文案）、异常时发回哪三样（两张截图 + 最后 20 行日志 + 每帧秒数）、
怎么故意回退 legacy。
