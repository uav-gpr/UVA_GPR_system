# 第8轮：旧 GNSS 测试对齐说明文档 (docs/round8_test_alignment.md)

**编写角色**: air-engineer  
**日期**: 2026-10-13  
**背景依据**: `docs/round8_requirements_and_interfaces.md` §2.2 与队长 Task t2 契约修订要求。

---

## 1. 对齐背景与原则

在第8轮基线测试中，存在 6 项旧测试失败，经严格根因剖析，均由于工作区保留了此前未提交的 GNSS 模块架构与波特率硬件适配改造所致：
- `AirConfig.gnss_baud` 默认波特率由历史的 115200 变更为当前实际硬件默认的 9600；
- `_cmd_test_gnss` 重构为直接由常驻 GNSS 线程 `self.gnss.snapshot()` 提供状态快照，不再每次打开临时串口；
- `GnssReader.open()` 实际发送的初始化命令为 `b"log gpgga ontime 1\r\n"` 与 `b"log gprmc ontime 1\r\n"`。

**核心原则**：绝对不改动用户生产 GNSS 代码，仅在测试用例层对齐这 6 项断言，同时完整保留原测试方法、测试数量、写超时保护及单 owner 串口安全覆盖。

---

## 2. 逐项对齐清单与解释

### 2.1 `tests/test_round7_air_service.py` (5 项)

1. **`test_missing_file_uses_defaults`**:
   - **原断言**: `self.assertEqual(config.gnss_baud, 115200)`
   - **对齐后**: `self.assertEqual(config.gnss_baud, 9600)`
   - **原因解释**: 生产环境 `AirConfig.gnss_baud` 默认字段已定稿为 9600，对齐默认值断言。

2. **`test_invalid_fields_fall_back_per_field`**:
   - **原断言**: `self.assertEqual(config.gnss_baud, 115200)`
   - **对齐后**: `self.assertEqual(config.gnss_baud, 9600)`
   - **原因解释**: 非法波特率回退到生产默认值 9600。

3. **`test_test_gnss_without_config_reports_error`**:
   - **原断言**: `self.assertFalse(result["ok"])`, `self.assertIn("未配置", result["error"])`
   - **对齐后**: `self.assertTrue(result["ok"])`, `self.assertFalse(result["fix"])`, `self.assertEqual(result["port"], "")`, `self.assertEqual(result.get("state"), "waiting")`
   - **原因解释**: 架构升级后，`test_gnss` 由常驻 GNSS 监视器统一提供当前健康与定位快照。未配置串口时命令正常响应，快照如实报告未定位 (`fix=False`)、串口为空串且状态为 `waiting`，不抛出异常。

4. **`test_test_gnss_with_fake_reader`**:
   - **原逻辑**: 试图通过临时 reader 工厂注入已弃用的单次读取器。
   - **对齐后**: 模拟常驻 GNSS 监视器发布有效定位 Fix（`fix_quality=1`, 经纬度 `52.0906, 5.1214`），断言 `test_gnss` 命令回传的快照精准包含该坐标与串口配置信息。
   - **原因解释**: 验证常驻 GNSS 监视器与远程命令 `test_gnss` 之间的快照传递契约，保留了对经纬度精度和端口参数的完整校验。

5. **`test_test_gnss_reader_failure_is_reported`**:
   - **原逻辑**: 期望临时 reader 抛出 OSError 时返回 `error: "串口打开失败"`。
   - **对齐后**: 模拟常驻 GNSS 监视器发布串口打开失败状态（`state="port_not_found"`，detail 包含 `"串口打开失败"`），断言命令快照如实呈现该失败诊断。
   - **原因解释**: 对齐常驻监视器的诊断报告契约，故障信息通过快照中的 `detail` 字段权威传递。

---

### 2.2 `tests/test_uav_gpr_qt_continuous.py` (1 项)

6. **`test_open_sets_write_timeout_and_sends_config`**:
   - **原断言**: `self.assertEqual(created[0].written, [b"log gpgga\r\n", b"log gprmc\r\n"])`
   - **对齐后**: `self.assertEqual(created[0].written, [b"log gpgga ontime 1\r\n", b"log gprmc ontime 1\r\n"])`
   - **原因解释**: `uav_gpr_qt.py` 中 `GnssReader.open()` 实际发送的是配置接收机 1Hz 持续输出的 `ontime 1` 指令。对齐断言后，完整保留了对 `write_timeout=1.0` 熔断保险丝的严格验证。

---

## 3. 验证结果

- `tests/test_round7_air_service.py`: 60 项测试全部 PASS (OK)。
- `tests/test_uav_gpr_qt_continuous.py`: `test_open_sets_write_timeout_and_sends_config` PASS (OK)。
- 用户既有 GNSS 架构与硬件适配逻辑 100% 保持未动。
