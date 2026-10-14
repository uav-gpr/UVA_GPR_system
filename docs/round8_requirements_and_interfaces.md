# 第8轮：校准、背景采集与校准文件复用需求与双端接口规范

**文档版本**: v1.1 (包含补遗与架构修订)  
**编写角色**: requirements-reviewer  
**编写日期**: 2026-10-13  
**状态**: 需求契约与双端接口终版已定稿，包含 10 项修订补遗，双端工程师依据同一规范并行工作。

---

## 目录

1. [现场三个故障的源码根因分析与软硬件边界判定](#1-现场三个故障的源码根因分析与软硬件边界判定)
2. [改前基线测试结果与测试对齐策略](#2-改前基线测试结果与测试对齐策略)
3. [双端共用协议契约 (保持 framing 与 type 不变)](#3-双端共用协议契约-保持-framing-与-type-不变)
4. [核心功能需求规格与状态机设计](#4-核心功能需求规格与状态机设计)
5. [验收场景 R8-1 至 R8-11 覆盖矩阵与文件范围分工](#5-验收场景-r8-1-至-r8-11-覆盖矩阵与文件范围分工)
6. [验证方案：无硬件自动化验证与现场真机复核清单](#6-验证方案无硬件自动化验证与现场真机复核清单)

---

## 1. 现场三个故障的源码根因分析与软硬件边界判定

### 1.1 故障一：OSL 激活后无法继续背景采集
- **现象**: 现场向导完成六步 OSL 采集、解算并点击“激活校准”后，界面停留在已激活状态，但“开始校准”按钮永久禁用，无法切换为背景采集模式；用户被困在该界面。
- **源码根因分析**:
  1. **地面端状态未解绑 (`uav_gpr_ground_station.py` L2348-L2355, L2555-L2558)**:
     地面向导以 `self._cal_id is not None` 作为 `session_active` 的唯一判定依据。在收到空中端 `calibration_result(status="activated")` 时，仅更新了文本标签，**未清空 `self._cal_id`**，也未重置 `self._steps` 与 `self._solved`。导致 `self.begin_button` 永久 `setEnabled(False)`，扫描配置输入框也永久处于锁定状态。
  2. **双端将“会话生命周期”与“生效校准组件”混为一谈**:
     地面端把 `_cal_id` 同时用作“当前向导操作句柄”与“生效校准身份”。一旦校准激活，向导会话实际上已经完成（进入终态），但因缺少“结案关会话、保留生效记录”的状态转移，无法启动下一个背景会话。
  3. **空中端生效记录单一覆盖风险 (`uav_gpr_air_service.py` L1474-L1505)**:
     空中端在激活时仅维护一个单一的 `self.active_calibration_id` 字符串。一旦用户后续执行背景校准并激活，背景的 `cal_id` 会覆盖 OSL 的 `active_calibration_id`；同时向导状态查询 `_command_calibration_status` 仅读取 `self._calibration_session.active()`，当新的背景会话覆盖 `_calibration_session` 后，地面端将无法再查询到先前生效的 OSL 频带与指标。
- **软硬件边界判定**:
  这是纯粹的**双端软件状态机与生命周期设计缺陷**，与硬件射频或天线物理链路无关。

---

### 1.2 故障二：地面无法使用既有校准文件
- **现象**: 地面站无法直接选用用户历史实测的 SOLT 校准文件（如 `solt_2port_20260807_202342.json`）并使其在空中端常驻生效；空中黑匣子重启后无法恢复校准。
- **源码根因分析**:
  1. **缺少文件内容传输与双端协同通道**:
     地面端与空中端通过局域网通信，地面主机的绝对路径（如 `D:\...`）在空中计算棒上不存在。系统缺少在既有 `type="command"` 协议框架下将校准文件内容推送到空中端、空中端校验并落盘的动作协议。
  2. **空中端缺少持久化恢复与原子落盘机制**:
     空中端 `air_config.json` 缺少 `active_calibration_path` 字段；空中端重启初始化时，未从配置文件加载并校验既有校准文件，无法实现开机自动恢复。
  3. **格式兼容与双端口/单端口适配缺陷 (`librevna/librevna_usb_s11.py` L75-L95, `calibration_service.py` L187-L245)**:
     `librevna_usb_s11.py` 的 `OSLCalibration.from_file` 仅支持旧版带 `Open` / `Short` / `Load` 单端口标记的文件。当传入新版 `uav_gpr_solt_calibration_v2`（包含 `P1_Open`, `P1_Short`, `P1_Load`, `P2_Open` 等键）时，会抛出 `ValueError("Calibration file is missing measurements: Open, Short, Load")`。缺少对 v2 标准件和双端口（P1/P2/Through）的统一适配与回退逻辑。
- **软硬件边界判定**:
  仓库中的历史校准文件结构完整、包含 201 频点 S 参数数据。无法使用系**软件协议缺失、路径不可达、格式解析未兼容及持久化未串联**导致。不能凭空声称用户真机校准精度依然完好（物理精度需在真机由标准件复核证明），但数据文件的软件语法与数值结构是完整合法的，必须严格保留。

---

### 1.3 故障三：P1_Open 重采被拒且 UI 长期显示“请求采集中”
- **现象**: 现场截图显示，用户在第一步采集完成后点击“重采当前步骤”，空中端立即拒绝，提示步骤必须按序进行；地面表格该行永久停留在“请求采集中…”，向导无法继续也无法退出。
- **源码根因分析**:
  1. **会话层完全未实现重采语义 (`calibration_service.py` L867-L879)**:
     `CalibrationSession.capture(step_name)` 内部硬编码顺序校验：
     ```python
     next_step = self.next_pending_step()
     if next_step.name != step_name:
         raise RuntimeError(f"下一待采步骤为 {next_step.name}，收到 {step_name!r}（步骤必须按序进行）")
     ```
     当 `P1_Open` 首次完成后，`next_pending_step` 已推进为 `P1_Short`。此时再次请求 `P1_Open`，直接抛出 `RuntimeError`，导致空中端向地面回传 `status="rejected"`。
  2. **地面向导收到拒绝后未复位行状态 (`uav_gpr_ground_station.py` L2561-L2567)**:
     地面端在 `recapture_selected` 中将表格行置为“请求采集中…”，但收到 `status="rejected"` 时，仅设置了状态栏文本，**未调用 `_update_step_row` 恢复原状态**，也未纠正 `_step_index`，造成 UI 永久卡死在“请求采集中…”。
  3. **缺少步骤收尾确认 (`step_done`) 导致竞态**:
     空中端向地面连续推送每条 sweep 的 `calibration_capture` 消息。地面端仅凭末样本 `index >= of` 就单方面判定步骤完成，而此时空中端后台线程与底层 USB worker 尚未完成收尾释放并回到 `idle_waiting` 槽。若用户立即发起下一步或重采，极易触发跨线程冲突。
  4. **缺少请求关联 ID (`request_id`) 与防重复点击控制**:
     地面重采按钮未做防抖，未携带请求唯一标识，迟到的回执容易与新请求产生串扰。
- **软硬件边界判定**:
  这是典型的**状态机协议缺陷、异步时序竞态与异常分支 UI 回退遗漏**，非硬件故障。

---

## 2. 改前基线测试结果与测试对齐策略

### 2.1 改前测试基线 (详见 docs/round8_baseline.log)
执行命令:
```powershell
$env:QT_QPA_PLATFORM='offscreen'; $env:QTWEBENGINE_DISABLE_SANDBOX='1'; .venv312\Scripts\python.exe docs\run_tests_batched.py
```
- **统计**: 39 个模块、1123 项测试，耗时 323.8 秒。
- **状态**: 36 个 OK，1 个 PASS*（`test_round1_review_fixes` 退出码 3221225477 / 0xC0000005，Qt WebEngine 拆除段 Chromium 线程已知漂移），2 个失败模块共 6 项失败。
- **6 项失败根因**: 用户此前未提交的 GNSS 模块架构改造（`AirConfig.gnss_baud` 默认改为 9600、`_cmd_test_gnss` 采用常驻 `self.gnss.snapshot()`、串口初始化仅发 `log gpgga`）。

### 2.2 测试对齐策略 (修订补充)
- **严禁修改生产代码**: 绝对禁止修改 GNSS 生产默认值（保持 9600）、禁止修改 GNSS 常驻 owner 或初始化指令来迎合旧测试。
- **受控对齐测试断言**: Captain 已受控 amend t2 契约，允许在 `tests/test_round7_air_service.py` 与 `tests/test_uav_gpr_qt_continuous.py` 中将这 6 项旧断言对齐到当前生产既有行为，并在 `docs/round8_test_alignment.md` 中逐项记录对齐依据。
- **全量实证全绿目标**: 最终全量测试必须实证达到所有 39 模块无断言失败（拆除段崩溃单独标注），不永久豁免或掩盖这 6 项测试。

---

## 3. 双端共用协议契约 (保持 framing 与 type 不变)

### 3.1 消息通道与类型划分 (修订更正)
- **地面 → 空中**: 统一使用既有 `type="command"`，载荷包含 `{"action": str, "params": dict, "client_id": str}`。
- **空中 → 地面命令同步应答**: 统一使用既有 `type="command_ack"`，包含 `{"action": str, "supported": bool, "result": dict}`。
- **空中 → 地面异步校准回程**:
  1. `type="calibration_capture"`: 单条 sweep 频域原始数据回程（NPZ 载荷，用于实时绘图与进度显示）。
  2. `type="calibration_quality"`: 既有步骤质量指标回程（各步 RMS/最大误差，**文档明确保留，严禁禁止或废除**）。
  3. `type="calibration_result"`: 会话状态机关键事件广播（`planned`, `step_done`, `solved`, `activated`, `rejected`, `failed`, `cancelled`）。

---

### 3.2 动作扩展与新增契约规范

#### (1) `cal_capture` (采集/重采步骤)
- **Action**: `"cal_capture"`
- **Params**:
  ```json
  {
    "cal_id": "cal_20261013_180000",
    "step": "P1_Open",
    "recapture": true,
    "request_id": "req_cap_20261013_180001_1234"
  }
  ```
- **字段规范**:
  - `cal_id` (str, 必填): 当前会话 ID。
  - `step` (str, 必填): 步骤名称（OSL 6 步或 `Background`）。
  - `recapture` (bool, 可选, 默认 `false`): 为 `true` 时显式声明重采，允许覆盖已完成步骤。
  - `request_id` (str, 必填/建议必选): 请求唯一令牌，空中端用于幂等去重。
- **同步应答 (`type="command_ack"`)**:
  - 成功接收: `result={"ok": true, "action": "cal_capture", "cal_id": "...", "step": "...", "recapture": bool, "request_id": "..."}`
  - 非法状态/拒绝: `result={"ok": false, "error": "具体原因", "request_id": "..."}`
- **异步收尾回执 (`type="calibration_result"`, status="step_done")**:
  ```json
  {
    "cal_id": "cal_20261013_180000",
    "status": "step_done",
    "step": "P1_Open",
    "request_id": "req_cap_20261013_180001_1234",
    "next_step": "P1_Short",
    "steps_done": 1,
    "total_steps": 6
  }
  ```
  - **核心语义约束**:
    1. `step_done` 标志着后台采集任务结束、采集槽释放并回到 `idle_waiting` 可继续状态。
    2. **常驻 USB 句柄保持连接**，绝不销毁或断开 USB 硬件。
    3. **竞态防范**: 采集执行线程**绝对不能 join 自身**；采集槽释放与身份检查必须在锁保护下进行；旧任务迟到的 `finally` 必须校验当前槽内的 `cal_id`/`step`/`request_id` 匹配，绝对不得误覆盖新分配的采集槽。

#### (2) `cal_import_file` (下发并生效已有校准文件)
- **Action**: `"cal_import_file"`
- **Params**:
  ```json
  {
    "filename": "solt_2port_20260807_202342.json",
    "content": "{\"format\": \"uav_gpr_solt_calibration_v2\", ...}",
    "activate": true,
    "request_id": "req_imp_20261013_180002"
  }
  ```
- **大小上限**: `content` 字符串最大限制为 **10 MB**，防止恶意或异常大文件导致空中端 OOM。
- **校验与原子性保障 (修订更正)**:
  1. 语法与数值严格校验: 频点数 $\ge 2$、严格单调递增、所有复数值为有限浮点（无 NaN/Inf）。
  2. 原子文件落盘: 先写临时文件 `.tmp`，校验无误后 `replace` 替换。
  3. 配置持久化与失败回滚: 将路径写入 `air_config.json` 的 `active_calibration_path`。**若 `save_config()` 写盘失败，整笔导入操作视为失败！** 此时必须回滚/删除临时落盘文件，**绝对不得破坏原有同名生效文件，也绝对不得破坏内存中现存的已生效校准状态**。
- **同步应答 (`command_ack`)**:
  - 成功: `result={"ok": true, "cal_id": "imported_...", "format": "...", "ports": ["P1", "P2"], "points": 201, "valid_band_hz": [400000000.0, 4000000000.0], "activated": true, "request_id": "..."}`
  - 失败: `result={"ok": false, "error": "...", "request_id": "..."}`

#### (3) `cal_export_file` (地面拉取空中端校准文件内容 - 补充定稿)
- **Action**: `"cal_export_file"`
- **Params**:
  ```json
  {
    "cal_id": "imported_solt_2port_20260807_202342",
    "format": "uav_gpr_solt_calibration_v2",
    "request_id": "req_exp_20261013_180003"
  }
  ```
  *(注: `cal_id` 为空时，默认导出当前空中端已生效的 OSL 校准)*
- **大小上限**: 单次导出文件上限 10 MB。
- **同步应答 (`command_ack`)**:
  - 成功:
    ```json
    {
      "ok": true,
      "cal_id": "imported_solt_2port_20260807_202342",
      "filename": "solt_2port_20260807_202342.json",
      "format": "uav_gpr_solt_calibration_v2",
      "content": "{\"format\": \"uav_gpr_solt_calibration_v2\", ...}",
      "size_bytes": 1048576,
      "request_id": "req_exp_20261013_180003"
    }
    ```
  - 失败: `result={"ok": false, "error": "未找到指定校准文件或无生效校准", "request_id": "..."}`

#### (4) `cal_query_active` (查询完整生效与会话状态 - 补充定稿)
- **Action**: `"cal_query_active"`
- **Params**: `{}`
- **同步应答 (`command_ack`) 结构**:
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

#### (5) `cal_clear_active` (清除生效校准 - 消除歧义修订)
- **Action**: `"cal_clear_active"`
- **Params**: `{"target": "all" | "osl" | "background"}`
- **语义与持久化边界**:
  1. `target == "background"`: **仅清除内存中的 `_background` 与 `active_background_id`，绝对不得清除 `air_config.json` 中的 `active_calibration_path`！**
  2. `target == "osl"`: 清除内存中 OSL 组件，同时清除 `air_config.json` 的 `active_calibration_path` 并执行 `save_config()`；**同时自动使现有背景失效**！
  3. `target == "all"`: 同时清除 OSL 与背景，清空持久化路径并 `save_config()`。

---

### 3.3 看门狗、幂等去重与新旧端兼容降级

#### 1. 动态看门狗预算机制 (取代固定 30s)
- **问题**: 在高重复次数（如 `repeats=256`）或窄 IFBW 慢扫频场景下，固定 30s 极易产生误判。
- **规则**:
  - 地面端采用**动态滑动看门狗**：每收到一条 `calibration_capture` 进度消息，看门狗计时器重置。
  - 静默无进度超时上限计算：
    $$\text{Timeout} = \max\left(30.0,\; \text{预估单次扫频耗时} \times 3.0\right)$$
  - 超时或断线恢复流程：超时触发后，地面端**严禁直接置空状态并允许用户重新点击**。必须先发送 `cal_query_active` 查询空中端 `in_progress_session.busy` 与当前执行中的 `active_request_id`；若空中端依然处于 busy，保持界面等待锁定，绝不重复发起。

#### 2. 请求幂等与防重缓存 (`request_id`)
- 空中端维护一个有限容量的 LRU/FIFO 请求结果缓存队列（容量 128 条）。
- 记录 `request_id` 及其执行状态（`in_progress` / `completed` / `result`）。
- 若收到相同 `request_id`：
  - 正在执行中：返回 `{"ok": false, "error": "请求正在执行中", "duplicate": true}`。
  - 已完成：直接回放上次的执行回执，**绝对不得二次重新触发底层采集 worker**。

#### 3. 新端与旧端兼容与平滑降级
- **旧端能力判定**:
  - 旧端空中服务收到带 `recapture=true` 的 `cal_capture` 时，并不会报 `unknown action`，但旧服务可能会因内部未识别参数而拒绝；
  - 旧端空中服务不会发送 `step_done`，只会在采集完成发送末样本（`index >= of`）。
- **地面端兼容降级规则**:
  - 若 `cal_query_active` 显示空中端 `capabilities.step_done == false`（或握手判定为旧空中端）：
    地面端在收到最后一条样本数据（`index >= of`）后，启动 0.5s 平滑收尾缓冲，随后主动调用状态查询以完成步骤，**严禁因苦等 `step_done` 而导致旧端流程永久挂起**。

---

## 4. 核心功能需求规格与状态机设计

### 4.1 会话管理与生效状态严格解耦
- **生命周期推进规则**:
  1. OSL 会话执行 `cal_activate` 成功后，该会话生命周期正式**结束**。
  2. 空中端将解算结果注入 `ActiveCalibrationState`，将 `self._calibration_session` 置为 `None`。
  3. 地面端向导收到 `activated` 后，清空 `self._cal_id`，将当前生效 OSL 信息永久保存在向导及主界面的“当前校准”状态栏中；向导状态机回到空闲，`begin_button` 恢复可用。
  4. 用户随后在模式下拉框选择“空采参考（背景）”，点击“开始校准”，向导正常创建新的 `cal_begin(mode="background")` 会话，直接在既有 OSL 校准域下采集背景，无需重启任何一端。
  5. **校准域变更背景失效原则 (修订更正)**:
     **只要 OSL 身份或校正域发生任何变更（新 OSL 解算激活、导入新校准文件、或清除 OSL），旧背景必须立即彻底失效并清除**，不限于频段是否相同。地面端明确提示“OSL 校准已变更，先前背景已失效”。

### 4.2 步骤受控重采机制 (`recapture`)
- **API 规范**: 统一调用 `CalibrationSession.capture(step_name, recapture=True, ...)`。
- **状态转移**:
  - 若对已完成步骤发起 `recapture=True`：清除该步骤历史样本均值，重新启动 worker 采集；
  - 若先前已解算（`solved`）：重采成功后旧 `solved_result` 自动置为 `None`，向导状态回退至 `in_progress`，必须重新解算后方可激活；
  - 拒绝与失败处理：收到 `rejected` 或 `failed`，地面向导表格行立即恢复为原有真实状态，严禁停留在“请求采集中…”。

### 4.3 格式统一适配与持久化恢复
- `uav_gpr_solt_calibration_v2` 与旧单端口文件由 `read_calibration_measurements` 统一解析为 `CalibrationMeasurement`；
- `OSLCalibration.from_file` 兼容读取包含 `P1_Open` 键的字典或新版格式；
- 启动时自动从 `air_config.json` 的 `active_calibration_path` 安全恢复；文件缺失或损坏时仅打日志告警，置生效为 None，杜绝未捕获崩溃。

---

## 5. 验收场景 R8-1 至 R8-11 覆盖矩阵与文件范围分工

### 5.1 验收场景矩阵
| 编号 | 场景名称 | 验收标准与判定准则 |
|---|---|---|
| **R8-1** | 六步 OSL → 解算 → 激活 → 背景 → 正常采集 | 双端不重启；OSL 激活后向导解除锁定可直接切背景建会话；P1/P2 校准与背景协同生效。 |
| **R8-2** | 六步 OSL 完成但未激活 | 状态诚实提示“未启用/需激活或取消”，向导退出或查询时不误报校准生效。 |
| **R8-3** | 读取仓库真实 v2 校准文件 | 原件保持只读未修改；导入后解析两端口各 201 点，频段 400-4000 MHz 正确；空中端保存并实际生效。 |
| **R8-4** | 旧 Open/Short/Load 文件兼容 | 解析生成 P1/S11 校准；明确显示单端口范围，不伪造成双端口，P2 行为安全回退。 |
| **R8-5** | 地面导入 → 空中重启 → 状态恢复与采集 | 空中端模拟重启后自动重新装载校准；地面端 `cal_query_active` 查到同一身份；采集数值校正生效。 |
| **R8-6** | 坏文件/缺步骤/频率不一致/NaN/Inf/越界 | 空中端原子拒绝；返回可读错误原因；当前已生效校准完整保留，无破坏、无半套覆盖。 |
| **R8-7** | P1_Open 首次完成后重采再继续 | 允许重采；成功替换数据，下一步正常指向 P1_Short；不再出现顺序拒绝死锁。 |
| **R8-8** | 后续步骤完成或已解算后再重采 P1_Open | 其它有效步骤数据保留；旧解算结果作废；重新解算采用新数据。 |
| **R8-9** | 防抖/收尾竞态/拒绝/超时/迟到回执 | UI 无无限“请求采集中”、无重复下发；收到 `step_done` 后放开操作；超时动态可控。 |
| **R8-10** | 新旧两端组合与既有通信回归 | 旧端对接不崩溃并平滑降级；系统 run_id、hello 握手、ACK、补传及 GNSS 既有行为不被破坏。 |
| **R8-11** | 最终双端便携包解压独立复验 | 从生成 ZIP 独立解压，使用各包内置 Python runtime 验证依赖与启动；验证 `__file__` 不指向源码目录。 |

---

### 5.2 工程师职责与代码修改范围分工 (严格遵循合约路径)

Captain 路径/所有权勘误：以已批准任务和 t2 的受控 amendment 为最终准绳，下面示意图不扩大任何成员范围。**准确的新增测试路径与所有权**如下（覆盖示意图的旧路径/归属）：

```text
t2 / air-engineer: tests/test_round8_calibration_core.py
t3 / ground-engineer: tests/test_round8_calibration_ground.py
t4 / verifier: tests/test_round8_calibration_end_to_end.py
```

端到端测试由 verifier 独占；air-engineer 和 ground-engineer 只读协作，均不得写该文件。t2 另外仅获准对齐六项已核实的旧 GNSS 测试断言并写说明，不修改 GNSS 生产行为。

```
+-------------------------------------------------------------+
|                     requirements-reviewer                   |
|  - 输出需求规范 docs/round8_requirements_and_interfaces.md   |
|  - 输出补遗说明 docs/round8_interface_addendum.md           |
|  - 输出基线日志 docs/round8_baseline.log                    |
|  - 后续执行独立代码审查；严禁触碰任何生产源码与现有测试     |
+-------------------------------------------------------------+
                              |
             +----------------+----------------+
             |                                 |
+----------------------------+   +----------------------------+
|        air-engineer        |   |      ground-engineer       |
| [空中端与共享核心实现范围]  |   |    [地面端与向导实现范围]   |
|                            |   |                            |
| 1. calibration_service.py: |   | 1. uav_gpr_ground_station  |
|    - session.capture(...)  |   |    .py:                    |
|      支持 recapture=True   |   |    - CalibrationWizard 改版|
|    - 统一读 v2/旧校准数据  |   |    - 激活后释放 _cal_id     |
| 2. librevna/librevna_usb_  |   |    - 导入/导出校准文件入口 |
|    s11.py:                 |   |    - 增加 request_id 与防抖|
|    - from_file 兼容 v2 字典|   |    - 动态看门狗与超时自愈  |
| 3. uav_gpr_air_service.py: |   |    - 接收 step_done / 降级 |
|    - cal_capture 支持重采  |   | 2. 测试契约路径:           |
|    - 释放采集槽发 step_done|   |    - tests/test_round8_    |
|    - 实现 cal_import_file  |   |      calibration_ground.py |
|    - 实现 cal_export_file  |   |                            |
|    - 实现 cal_query_active |   | 严禁修改:                   |
|    - 实现 cal_clear_active |   |   uav_gpr_air_service.py   |
|    - air_config.json 持久化|   |   calibration_service.py   |
| 4. 测试契约路径:           |   |   librevna/*               |
|    - tests/test_round8_    |   |                            |
|      calibration_core.py   |   +----------------------------+
|    - tests/test_round8_    |
|      calibration_end_to_end.py（verifier独占，空中只读协作） |
|    - tests/test_round7_air |
|      _service.py (断言对齐)|
|    - tests/test_uav_gpr_qt |
|      _continuous.py (对齐) |
|    - docs/round8_test_     |
|      alignment.md (对齐记账|
|                            |
| 严禁修改:                   |
|   uav_gpr_ground_station.py|
+----------------------------+
```

---

## 6. 验证方案：无硬件自动化验证与现场真机复核清单

### 6.1 无硬件自动化测试方案
1. **测试基座与执行方式**:
   全量回归在虚拟环境中执行：
   ```powershell
   & .venv312\Scripts\python.exe docs\run_tests_batched.py
   ```
2. **测试全绿标准**:
   在 `t2` 完成 6 项 GNSS 旧断言受控对齐后，全量 39 个模块必须达到实证全绿（除 Windows WebEngine 进程退出拆除段崩溃单独记账外，零 FAIL、零 ERROR）。

### 6.2 现场真机复核清单
- [ ] **1. 物理连接与真机初始化**: 计算棒连入 LibreVNA USB，确认日志输出 `设备来源 usb`；地面站连接成功。
- [ ] **2. 双端口 SOLT 文件导入验证**: 导入 `solt_2port_20260807_202342.json`，查询确认生效；计算棒重启后确认校准自动恢复。
- [ ] **3. 步骤采集与 P1_Open 现场重采**: 采集 `P1_Open` 后下拉选择重采，确认顺利完成并正常推进到 `P1_Short`。
- [ ] **4. OSL 激活与背景连贯采集**: 六步解算激活，向导按钮恢复，无缝切换并完成空采背景参考。
- [ ] **5. 实际雷达探地采集**: 连续采集模式下 B-scan 瀑布图刷新平稳，背景去噪生效。
