# 第8轮：空中与共享校准修复独立审查报告 (Review Round 3 - Final)

**审查文档**: `docs/round8_review_air.md`  
**审查任务**: `t15` (审查对象: `t14` repair-round-3 空中/共享校准修复)  
**审查角色**: requirements-reviewer  
**审查日期**: 2026-10-13  
**审查判定 (Verdict)**: **`pass`**  

---

## 1. 审查概述与依据

本审查依据：
- `docs/第8轮_校准与背景采集修复计划.md`
- `docs/round8_requirements_and_interfaces.md` (v1.1)
- `docs/round8_interface_addendum.md` (v1.1)
- `docs/round8_verification_corrections.md` (t10 验证口径校正)
- `t14` repair 最新代码实现 (`calibration_service.py`, `uav_gpr_air_service.py`, `librevna/librevna_usb_s11.py`, `uav_gpr_qt.py`)
- 真实探针执行、专项单测及分块回归测试结果。

### 1.1 独立复跑与测试执行结果
1. **核心与空中模块单测执行**:
   ```powershell
   & .venv312\Scripts\python.exe docs\run_tests_batched.py --only round8_calibration_core round7_air_service round2_ground_calibration
   ```
   - 结果统计: 3 个模块共 108 项测试全部通过（Ran 108 tests in 10.1s, failed modules: none, exit code: 0）。
2. **端到端测试归因核查**:
   - `test_round8_calibration_end_to_end` 中涉及空中端的 18 项用例（含修复后转换的正式回归：Through 注入、OSL 与背景身份隔离、`activated` 广播完整字段、字典 10MB 门控拦截、文件域回滚等）全部实证通过。
   - 剩余 8 项失败均准确归因于地面端（`FINDING-GND-01 ~ 06`），正在 `t16` 中由 ground-engineer 修复，不影响空中端自身的终态裁决。

---

## 2. 缺陷清单 (FINDING-AIR-01 ~ 13) 完整闭环复核

经过逐项代码核查与独立测试探针实证，全部 13 项发现项均已彻底修复并闭环：

### 2.1 第一轮缺陷复核 (FINDING-AIR-01 ~ 09)
1. **FINDING-AIR-01 (已闭环)**:  
   - 源码在 `_handle_cal_query_active` 中通过 `_active_osl_id` 独立管理 OSL 身份，激活背景时保留 OSL ID，实测证明 OSL ID 不再被背景覆盖。
2. **FINDING-AIR-02 (已闭环)**:  
   - `_start_continuous` (L2127) 与 `_start_legacy` (L2192) 已正确将 `through_measurement=self._active_through` 注入 `AcquisitionTaskSpec`，双端口 Through 校准校正链路完整贯通。
3. **FINDING-AIR-03 (已闭环)**:  
   - `_handle_cal_clear_active` (L1609) 返回字典已补齐 `"cleared": target`，实测清背景不会再误清地面 OSL。
4. **FINDING-AIR-04 (已闭环)**:  
   - `_handle_cal_import_file` (L1414-L1425) 返回字典已补齐 `ports`, `points`, `valid_band_hz`, `format`, `has_through`，地面端摘要显示完整。
5. **FINDING-AIR-05 (已闭环，结合 AIR-12 彻底解决)**:  
   - 文件名通过 `Path(filename).name` 安全清洗且严格限制合法字符；写盘前对既有文件做完整字节备份，写盘失败自动还原。
6. **FINDING-AIR-06 (已闭环，结合 AIR-13 彻底解决)**:  
   - `_command_calibration_activate` 增加了 `save_config()` 返回值校验，落盘失败向客户端权威报告 `status="failed"`，绝不冒充 `activated`。
7. **FINDING-AIR-07 (已闭环，结合 AIR-11 彻底解决)**:  
   - 频点数校验已强制要求 `len(step_freqs) >= 2`，杜绝单频点内插异常。
8. **FINDING-AIR-08 (已闭环)**:  
   - `librevna_usb_s11.py` L217-L226 仅在完全无 P2 时降级单端口；若存在残缺 P2 标准件严格抛出 `ValueError`，杜绝静默降级。
9. **FINDING-AIR-09 (已闭环)**:  
   - 采集驱动线程 `close_current` 异常被捕获时记录详细日志且权威报告 `status="failed"`，杜绝假收尾。

### 2.2 第二轮缺陷复核 (FINDING-AIR-10 ~ 13)
10. **FINDING-AIR-10 (已闭环)**:  
    - `AirBlackboxService.__init__` (L664) 显式声明并初始化 `self._active_osl_id: str | None = None`。
    - 实证验证：全新服务实例启动后直接调用 `handle_command('cal_query_active', {})`，成功返回 `ok=True, supported=True`，不再抛出任何 AttributeError，彻底解除了握手阻塞。
11. **FINDING-AIR-11 (已闭环)**:  
    - `calibration_service.py` L208-L213 补充了序列化 UTF-8 字节长度门控：
      ```python
      serialized_len = len(json.dumps(data, ensure_ascii=False).encode("utf-8"))
      if serialized_len > MAX_CALIBRATION_FILE_BYTES:
          return False, f"校准内容超过最大限制 ({MAX_CALIBRATION_FILE_BYTES // (1024 * 1024)} MB)", {}
      ```
    - 实证验证：传入携带 15MB 文本的合法点数字典，校验器权威拦截并返回“校准内容超过最大限制”，10MB 门控严密闭环。
12. **FINDING-AIR-12 (已闭环)**:  
    - `_handle_cal_import_file` 调整原子事务顺序：先通过备份与 `os.replace` 将目标文件安全落盘就位，再调用 `save_config()` 更新路径；若 `save_config()` 失败，恢复原配置路径并用备份还原原文件字节（若原文件不存在则物理删除），杜绝留下悬空配置。
13. **FINDING-AIR-13 (已闭环)**:  
    - `CalibrationSession` 引入 `rollback_activation()`（L1245-L1250）；在 `_command_calibration_activate` 中，若 `save_config()` 失败，调用 `session.rollback_activation()` 严格回滚会话状态至 `STATE_SOLVED`，允许用户排障后重新尝试激活，会话不再被不可逆废弃。
    - 此外，`status="activated"` 广播消息已完整补齐 `mode`, `ports`, `points`, `valid_band_hz`, `active_osl_id` 等契约字段。

---

## 3. 审查结论与判定

空中黑匣子服务与共享校准核心模块（`calibration_service.py`, `uav_gpr_air_service.py`, `librevna/librevna_usb_s11.py`, `uav_gpr_qt.py`）所有发现项与质量契约已全部核实闭环。逻辑状态机、协议字段、原子事务与数值有效性均达到预期，未引入新的回归缺陷。

- **审查判定 (Verdict)**: **`pass`**  
- **任务状态**: **`completed`**
