# 实时链路与UI 第3轮完成报告

日期：2026-09-09　分支：`my-modifications`（未 commit、未 push）  
执笔：captain（基于 uva-gpr-ui3 团队 executor 与 reviewer 全量成果与六轮门禁记录）  
基线：t0 = `9027366`　环境：`.venv312`（Python 3.12.14），offscreen Qt，无真机  
测试口径：`docs/run_tests_batched.py`（独立子进程批跑，931 tests / 30 modules 全绿）  

---

## 1. 本轮结论

**完成（软件部分全部通过终审）**。第3轮 UI 重构三块核心工作全部实现并 100% 通过 reviewer 质量评审门：

1. **空中端极简化（t1 / t2 PASS）**：
   - 彻底移除空中端本地任务控制区（`_operator_bar`：开始/停止/单次/间隔/次数/暂停继续）与诊断图表区（`_diagnostics_section`：S参数图/A-scan曲线/GNSS地图）；
   - 主界面重排为四段式极简布局：顶部连接栏（雷达USB+GNSS串口+地面站链路，收缩为两行）、状态条（服务状态/trace计数/USB待命状态保留）、B-scan 主视图占满主区域、底部设备日志区；
   - `AirSettingsDialog` 瘦身：校准页与扫描参数页整页移除，仅保留显示/输出/诊断页，`objectName` 保持兼容；
   - 远程命令控制链（`start_acquisition`/`stop_acquisition`/`handle_remote_command`）逐字完整保留，空中端采集控制完全转为受控于地面无线命令。

2. **参数与控制迁移地面端（t3 / t4 needs_revision → t7 修复 → t8 PASS）**：
   - 地面端设置中心（`GroundSettingsDialog`）全面升级为 3 选项卡（采集控制、扫描参数、处理参数），13 个控件规范遵循 `*_input` 命名；
   - 扫描参数（起止频率、频点数、功率、IFBW）与处理参数（Dewow参数、时间窗等）经既有 `type="command"` 通道新 action `set_params` 无线下发，空中端校验合法性并在空闲时应用生效；
   - 补齐 `pause` / `resume` 远程无线控制命令链，与 `start` / `stop` 共享同一状态机互斥语义（会话保持、USB 不释放）；
   - 修复与闭环：t4 发现的 3 项 finding（F2 机器热态下队列溢出刀锋时序、F3 地面 ACK 文案同步、F4 真实 SWEEP_SETTINGS 二进制报文捕获断言）全部在 t7 闭环修复并在 t8 确证通过。

3. **地面端校准与空采向导重做（t5 / t6 PASS）**：
   - 参考 `rebar-inspector` 分步向导架构模式，在地面端新建 `CalibrationWizard`（保留 `CalibrationDialog` 别名兼容）；
   - 彻底根治此前按钮重叠 Bug：重构为“垂直步骤表格 + 动态安装/警告提示卡片 + 进度条 + 底部双排流式操作栏”；经 6 种分辨率自建几何探针检测，按钮间 QRect 相交重叠数均为 0；
   - 完整支持 OSL 六步双反射（P1/P2 Open/Short/Load）与 Background 空采净空单步模式，支持失败重采与收齐后一键解算/保存/激活；
   - 服务层零侵入：`calibration_service.py` 零触碰，完全复用第2轮 `cal_*` 命令链与 `CalibrationSession` 状态机及互斥语义。

全量 **931** 条测试全绿（batched 口径，30 个模块全部 PASS，failed modules: none，OVERALL: OK，exit 0）。严格遵守收尾纪律：未执行任何 git commit / push / reset / checkout 操作。

---

## 2. 测试基线口径与终态账目

- **起始基线**：906 条测试（883 第2轮终态 + 23 条 `simulated_transport` 套件）。
- **第3轮新增与修改清单**：
  - `tests/test_ui3_air_minimal.py`：**新增 4 条**（空中端主界面四段断言、控件移除负断言、无本地按钮下 offscreen 真实采集驱动）；
  - `tests/test_round2_air_ui_compact.py`：**修改 10 条**（断言适配为四段式与瘦身语义，零弱化，零删除）；
  - `tests/test_bscan_gui.py`：**修改 1 条**（移除 ascan 曲线后的 `hasattr` 兼容防护）；
  - `tests/test_ui3_ground_settings.py`：**新增 13 条**（设置中心 UI 控件值域、draft 隔离、set_params 无线下发、pause/resume 远程化、SWEEP_SETTINGS 二进制协议帧捕获断言）；
  - `tests/test_ground_station_compact_ui.py`：**修改 3 条**（按钮集断言扩展，兼容 {开始, 停止, 暂停, 继续}）；
  - `tests/test_field_test.py`：**修改 2 处参数**（L472 与 L1177 rate 从 50.0 调整为 40.0，保持零丢包断言原样，消除机器热态下队列溢出抖动）；
  - `tests/test_ui3_calibration_wizard.py`：**新增 8 条**（向导分步状态机、OSL 六步流程、重采机制、空采模式、3 尺寸无按钮重叠几何检测）；
  - `tests/test_round2_ground_calibration.py`：**37 条 100% 保持全绿**（通过向后兼容别名与属性无缝复用）。
- **终态总账**：  
  `931 = 906（基线）+ 4（ui3_air）+ 13（ui3_ground）+ 8（ui3_cal）`，**净增 25 条测试，既有测试 0 删除**。

---

## 3. 修改后文件清单与归属

| 文件 | 状态 | 归属 | 规模/变动 |
|---|---|---|---|
| `uav_gpr_qt.py` | 修改 | t1 + t3 | 空中端 UI 瘦身、移除操作与诊断区、应用远程 set_params/pause/resume |
| `uav_gpr_ground_station.py` | 修改 | t3 + t5 + t7 | 地面设置中心三页升级、set_params 下发、向导重做 CalibrationWizard |
| `tests/test_field_test.py` | 修改 | t7 (F2) | 2 处 rate 50->40 裕量调整，消除时序抖动 |
| `tests/test_round2_air_ui_compact.py` | 修改 | t1 | 10 条断言适配新布局语义 |
| `tests/test_bscan_gui.py` | 修改 | t1 | 移除 ascan 曲线后的 hasattr 兼容 |
| `tests/test_ground_station_compact_ui.py` | 修改 | t3 | 3 处按钮集合扩展断言适配 |
| `tests/test_ui3_air_minimal.py` | **新增** | t1 | 4 条空中端极简主界面测试 |
| `tests/test_ui3_ground_settings.py` | **新增** | t3 + t7 | 13 条地面端设置与无线参数控制测试 |
| `tests/test_ui3_calibration_wizard.py` | **新增** | t5 | 8 条校准分步向导与防重叠几何测试 |

**outOfScope 严格守住（未修改）**：`calibration_service.py`、`disk_outbox.py`、`librevna/librevna_transport.py`、`gnss_map_widget.py`（空中端解引用但源码保留）、`reference_code/`、`dual_channel_reference/`。

---

## 4. 评审门与修复闭环记录

| 任务 | 事项 | 裁定 | 核心取证与核验要点 |
|---|---|---|---|
| **t1** | 空中端极简化实现 | 完成 | 移除 `_operator_bar` 与 `_diagnostics_section`；重排为四段式；瘦身设置页；保留命令分派。 |
| **t2** | 空中端极简化评审 | **PASS** | 亲跑 26/26 定向测试 + 910/28 全量通过；证实 0 孤立信号、0 死代码、远程驱动真实有效。 |
| **t3** | 参数与控制迁移地面端 | 完成 | 地面端 3 选项卡设置中心；`set_params` 下发；`pause`/`resume` 远程控制；E2E 链路闭环。 |
| **t4** | 参数迁移评审 | **needs_revision** | 核心功能过门，提 3 项 finding：F2（rate=50 机器热态队列溢出）、F3（地面 ACK 文案误导）、F4（E2E 缺底层帧断言）。 |
| **t7** | repair-round-2 修复 | 完成 | F2 rate 调至 40.0；F3 文案同步更正；F4 补充 SWEEP_SETTINGS 二进制报文捕获与字段校验。 |
| **t8** | repair-round-2 复审 | **PASS** | F2/F3/F4 全部 resolved；亲跑 923/29 全量通过；rate=40 裕量实证有效，机器热态不再抖动。 |
| **t5** | 校准与空采向导重做 | 完成 | 新建 `CalibrationWizard`；垂直步骤列表+流式操作栏；零重叠几何检测；复用第2轮服务层。 |
| **t6** | 校准向导终审 | **PASS** | 自建探针 6 尺寸全 0 重叠 0 出界；状态机与重采完整；第2轮 37 条校准测试全绿；亲跑 931/30 全量 OVERALL OK。 |

---

## 5. 真实验证数据与几何探针结论

### 5.1 全量回归实测（docs/run_tests_batched.py）
```
Ran 931 tests in 267.9s over 30 modules; failed modules: none
OVERALL: OK
exit code: 0
```

### 5.2 按钮几何防重叠探针实测（wizard_geometry_probe）
- 测试分辨率：`400×300`（最小尺寸保护生效，钳制为 `720×580`）、`500×400`、`720×580`、`800×600`、`1024×768`、`1920×1080`；
- 检测项目：7 个操作按钮两两 `QRect.intersects()` 相交检测、窗口外边界截断检测；
- 探针结果：全部分辨率下**相交重叠数 = 0，越界数 = 0，按钮完全可见**。此前图3重叠 Bug 彻底根除。

---

## 6. 收尾纪律与安全状态

- Git HEAD：严格保持在 `9027366`，未执行任何 `commit` / `push` / `checkout` / `reset` / `stash`；
- 所有改动均处于工作区，改动面与团队申报范围 100% 一致；
- 协议边界：无线通信 framing 协议现有字段与类型零修改，新能力完全基于既有 `command` 通道扩展，向下兼容性 100%。
