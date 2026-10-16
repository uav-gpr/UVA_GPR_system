"""第7轮 t1：空中端黑匣子服务测试（docs/第7轮_计划_空中端黑匣子与计算棒部署.md §4）。

覆盖契约验收点：
- 配置三层（默认值 / air_config.json / 命令行覆盖）+ 损坏容错（绝不崩）；
- 重连退避 5s→30s（经 RemoteSender reconnect_backoff 注入点；GUI 默认 2s 零变化）；
- 断线自动重连 + 失联采集继续 + DiskOutbox 本地保存 + 恢复自动补传（真实
  simulated 设备 + 真实 RemoteSender + 真实磁盘 outbox，零 mock 核心）；
- 磁盘水位：触发线/目标线、优先删已送达 run、删到未送达数据显式报警 + 逐条记账、
  active run 保护、探测可注入；
- 七个新 action 分发 + 设置类回写 air_config.json + 未知 action 回执"不支持"不崩；
- 远程校准 cal_* 全词表（会话建立/互斥/顺序校验/OSL 全链/背景真实链路/取消与关闭收尾）；
- autostart_acquisition=false 等地面命令 / true 开机自动采集。

全部无窗口（offscreen），无真机；透明记账断言到逐条事件。
"""

from __future__ import annotations

import json
import os
import shutil
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
import unittest.mock
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from PySide6 import QtCore  # noqa: E402

try:
    import uav_gpr_air_service as svc
except Exception as exc:  # noqa: BLE001 - 依赖缺失时整体跳过
    svc = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

try:
    import air_core as air
except Exception as exc:  # noqa: BLE001
    air = None
    AIR_IMPORT_ERROR = exc
else:
    AIR_IMPORT_ERROR = None

AIR_SERVICE_SKIP = f"uav_gpr_air_service unavailable: {IMPORT_ERROR}" if svc is None else ""
AIR_SKIP = f"uav_gpr_qt unavailable: {AIR_IMPORT_ERROR}" if air is None else ""

Gb = 1024 ** 3


def wait_until(predicate, timeout_s: float, interval_s: float = 0.02) -> bool:
    """有界轮询（绝不无限等待）。"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return predicate()


def _free_dead_port() -> int:
    """占用一个"绑定但不监听"的端口：connect 立即被拒（模拟地面端不在）。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("127.0.0.1", 0))
    port = int(sock.getsockname()[1])
    globals()["_ROUND7_DEAD_PORT_SOCKET"] = sock  # 保活占用，进程结束才释放
    return port


class _TmpDirTestCase(unittest.TestCase):
    """每个测试独立临时目录（测试自清理，零静默残留）。"""

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="round7-air-"))
        self.addCleanup(self._cleanup_tmp)

    def _cleanup_tmp(self) -> None:
        shutil.rmtree(self.tmp, ignore_errors=True)


# =========================================================================
# 配置三层：默认值 → air_config.json → 命令行
# =========================================================================


@unittest.skipIf(svc is None, AIR_SERVICE_SKIP)
class AirConfigLayerTests(_TmpDirTestCase):
    def test_missing_file_uses_defaults(self) -> None:
        config, issues = svc.load_air_config(self.tmp / "nope.json")
        self.assertEqual(config.ground_host, "192.168.144.20")
        self.assertEqual(config.ground_port, 8765)
        self.assertEqual(config.gnss_port, "")  # 空 = 不用 GNSS，绝不自动选中
        self.assertEqual(config.gnss_baud, 9600)
        self.assertEqual(config.engine, "continuous")
        self.assertEqual(config.device_source, "usb")
        self.assertEqual(config.output_dir, "gpr_runs")
        self.assertFalse(config.autostart_acquisition)
        self.assertEqual(config.disk_min_free_gb, 20.0)
        self.assertEqual(config.disk_target_free_gb, 40.0)
        # 透明记账：文件缺失必须留下说明（绝不静默回退）
        self.assertEqual(len(issues), 1)
        self.assertIn("不存在", issues[0])
        # path=None（无配置文件）= 纯默认、零 issue
        config_none, issues_none = svc.load_air_config(None)
        self.assertEqual(config_none, svc.AirConfig())
        self.assertEqual(issues_none, [])

    def test_valid_file_loaded(self) -> None:
        path = self.tmp / "air_config.json"
        path.write_text(
            json.dumps(
                {
                    "ground_host": "10.1.2.3",
                    "ground_port": 9000,
                    "gnss_port": "COM7",
                    "gnss_baud": 9600,
                    "engine": "legacy",
                    "device_source": "simulated",
                    "autostart_acquisition": True,
                    "disk_min_free_gb": 5,
                    "disk_target_free_gb": 8,
                }
            ),
            encoding="utf-8",
        )
        config, issues = svc.load_air_config(path)
        self.assertEqual(issues, [])
        self.assertEqual(config.ground_host, "10.1.2.3")
        self.assertEqual(config.ground_port, 9000)
        self.assertEqual(config.gnss_port, "COM7")
        self.assertEqual(config.gnss_baud, 9600)
        self.assertEqual(config.engine, "legacy")
        self.assertEqual(config.device_source, "simulated")
        self.assertTrue(config.autostart_acquisition)
        self.assertEqual(config.disk_min_free_gb, 5.0)
        self.assertEqual(config.disk_target_free_gb, 8.0)

    def test_corrupt_json_falls_back_to_defaults(self) -> None:
        path = self.tmp / "air_config.json"
        path.write_text("{ definitely not json", encoding="utf-8")
        config, issues = svc.load_air_config(path)
        self.assertEqual(config, svc.AirConfig())
        self.assertTrue(any("损坏" in i for i in issues), issues)

    def test_non_object_json_falls_back(self) -> None:
        path = self.tmp / "air_config.json"
        path.write_text("[1, 2, 3]", encoding="utf-8")
        config, issues = svc.load_air_config(path)
        self.assertEqual(config, svc.AirConfig())
        self.assertTrue(any("JSON 对象" in i for i in issues), issues)

    def test_invalid_fields_fall_back_per_field(self) -> None:
        path = self.tmp / "air_config.json"
        path.write_text(
            json.dumps(
                {
                    "ground_port": 0,  # 非法 → 默认 8765
                    "engine": "turbo",  # 非法 → 默认 continuous
                    "device_source": "radio",  # 非法 → 默认 usb
                    "gnss_baud": -1,  # 非法 → 默认 115200
                    "autostart_acquisition": "yes",  # 非法（非 bool）→ 默认 false
                    "ground_host": "",  # 非法（空）→ 默认
                    "disk_min_free_gb": "很多",  # 非法 → 默认 20
                }
            ),
            encoding="utf-8",
        )
        config, issues = svc.load_air_config(path)
        self.assertEqual(config.ground_port, 8765)
        self.assertEqual(config.engine, "continuous")
        self.assertEqual(config.device_source, "usb")
        self.assertEqual(config.gnss_baud, 9600)
        self.assertFalse(config.autostart_acquisition)
        self.assertEqual(config.ground_host, "192.168.144.20")
        self.assertEqual(config.disk_min_free_gb, 20.0)
        self.assertEqual(len(issues), 7)

    def test_target_below_min_falls_back(self) -> None:
        path = self.tmp / "air_config.json"
        path.write_text(
            json.dumps({"disk_min_free_gb": 30, "disk_target_free_gb": 10}), encoding="utf-8"
        )
        config, issues = svc.load_air_config(path)
        self.assertEqual(config.disk_min_free_gb, 30.0)
        self.assertEqual(config.disk_target_free_gb, 40.0)  # 目标线回默认
        self.assertTrue(any("disk_target_free_gb" in i for i in issues), issues)

    def test_unknown_fields_ignored_with_note(self) -> None:
        path = self.tmp / "air_config.json"
        path.write_text(json.dumps({"ground_port": 9001, "wizard": True}), encoding="utf-8")
        config, issues = svc.load_air_config(path)
        self.assertEqual(config.ground_port, 9001)
        self.assertTrue(any("wizard" in i for i in issues), issues)

    def test_cli_overrides_config_overrides_default(self) -> None:
        # 配置文件层：文件值覆盖默认值。
        path = self.tmp / "air_config.json"
        path.write_text(
            json.dumps({"ground_host": "10.9.9.9", "ground_port": 9002, "engine": "legacy"}),
            encoding="utf-8",
        )
        config, _ = svc.load_air_config(path)
        self.assertEqual(config.ground_host, "10.9.9.9")
        # 命令行层：CLI 覆盖文件值；未覆盖字段保持文件值。
        args = svc.build_arg_parser().parse_args(
            ["--server", "172.16.0.1:1234", "--autostart", "true", "--gnss-port", "COM9"]
        )
        config = svc.apply_cli_overrides(config, args)
        self.assertEqual(config.ground_host, "172.16.0.1")
        self.assertEqual(config.ground_port, 1234)
        self.assertTrue(config.autostart_acquisition)
        self.assertEqual(config.gnss_port, "COM9")
        self.assertEqual(config.engine, "legacy")  # CLI 未提 → 配置层值保持

    def test_cli_bad_server_format_raises(self) -> None:
        config = svc.AirConfig()
        args = svc.build_arg_parser().parse_args(["--server", "no-port-here"])
        with self.assertRaises(ValueError):
            svc.apply_cli_overrides(config, args)

    def test_cli_gnss_port_empty_disables(self) -> None:
        config = svc.AirConfig(gnss_port="COM7")
        args = svc.build_arg_parser().parse_args(["--gnss-port", ""])
        config = svc.apply_cli_overrides(config, args)
        self.assertEqual(config.gnss_port, "")  # 空 = 显式禁用 GNSS


# =========================================================================
# uav_gpr_qt.py 最小增量回归：重连退避注入点 + command_ack 结果回执挂钩
# =========================================================================


@unittest.skipIf(air is None, AIR_SKIP)
class RemoteSenderHookTests(unittest.TestCase):
    def test_default_reconnect_delay_is_round1_2s(self) -> None:
        sender = air.RemoteSender("127.0.0.1", 1, "run", lambda m: None)
        self.assertEqual(sender._reconnect_delay_s(), 2.0)  # GUI 现状逐字不变

    def test_injected_backoff_used(self) -> None:
        sequence = iter([5.0, 10.0, 30.0])
        sender = air.RemoteSender(
            "127.0.0.1", 1, "run", lambda m: None, reconnect_backoff=lambda: next(sequence)
        )
        self.assertEqual(sender._reconnect_delay_s(), 5.0)
        self.assertEqual(sender._reconnect_delay_s(), 10.0)
        self.assertEqual(sender._reconnect_delay_s(), 30.0)

    def test_bad_backoff_falls_back_to_2s(self) -> None:
        sender = air.RemoteSender(
            "127.0.0.1", 1, "run", lambda m: None, reconnect_backoff=lambda: -3.0
        )
        self.assertEqual(sender._reconnect_delay_s(), 0.0)  # 负值按 0 处理（不崩）
        sender2 = air.RemoteSender(
            "127.0.0.1", 1, "run", lambda m: None, reconnect_backoff=lambda: 1 / 0
        )
        self.assertEqual(sender2._reconnect_delay_s(), 2.0)  # 抛错回退现状

    def test_backoff_sequence_5s_to_30s(self) -> None:
        backoff = svc.AirReconnectBackoff()
        delays = [round(backoff(), 3) for _ in range(6)]
        self.assertEqual(delays, [5.0, 10.0, 20.0, 30.0, 30.0, 30.0])
        backoff.reset()
        self.assertEqual(backoff(), 5.0)

    def _capture_ack(self, sender, header) -> dict:
        captured: list[dict] = []

        def fake_send(sock, frame, payload=b""):
            captured.append(frame)

        with unittest.mock.patch.object(air, "send_remote_frame", side_effect=fake_send):
            sender._handle_command(header)
        self.assertEqual(len(captured), 1)
        return captured[0]

    def test_ack_without_callback_has_no_new_fields(self) -> None:
        sender = air.RemoteSender("127.0.0.1", 1, "conn", lambda m: None)
        sender._sock = object()  # 仅供发送判定；send 被 mock
        frame = self._capture_ack(sender, {"type": "command", "action": "start", "params": {}})
        self.assertEqual(
            sorted(frame.keys()),
            ["action", "protocol", "run_id", "timestamp", "type"],
        )

    def test_ack_result_hook_known_action(self) -> None:
        sender = air.RemoteSender(
            "127.0.0.1", 1, "conn", lambda m: None,
            command_callback=lambda action, params: {"ok": True, "engine": "legacy", "supported": True},
        )
        sender._sock = object()
        frame = self._capture_ack(sender, {"type": "command", "action": "set_engine", "params": {}})
        self.assertTrue(frame["supported"])
        self.assertEqual(frame["result"], {"ok": True, "engine": "legacy"})

    def test_ack_unknown_action_supported_false(self) -> None:
        sender = air.RemoteSender(
            "127.0.0.1", 1, "conn", lambda m: None,
            command_callback=lambda action, params: {"supported": False},
        )
        sender._sock = object()
        frame = self._capture_ack(sender, {"type": "command", "action": "dance", "params": {}})
        self.assertFalse(frame["supported"])
        self.assertNotIn("result", frame)

    def test_ack_callback_exception_fallback(self) -> None:
        def boom(action, params):
            raise RuntimeError("注入故障")

        sender = air.RemoteSender("127.0.0.1", 1, "conn", lambda m: None, command_callback=boom)
        sender._sock = object()
        frame = self._capture_ack(sender, {"type": "command", "action": "start", "params": {}})
        self.assertFalse(frame["supported"])
        self.assertFalse(frame["result"]["ok"])
        self.assertIn("注入故障", frame["result"]["error"])


# =========================================================================
# 命令分发：七个新 action + 设置回写 + 未知 action 兜底
# =========================================================================


def _dead_port() -> int:
    global _ROUND7_DEAD_PORT
    try:
        return _ROUND7_DEAD_PORT  # type: ignore[name-defined]
    except NameError:
        _ROUND7_DEAD_PORT = _free_dead_port()
        return _ROUND7_DEAD_PORT


@unittest.skipIf(svc is None or air is None, AIR_SERVICE_SKIP or AIR_SKIP)
class CommandDispatchTests(_TmpDirTestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])

    def setUp(self) -> None:
        super().setUp()
        self.config_path = self.tmp / "air_config.json"
        config = svc.AirConfig(
            ground_host="127.0.0.1",
            ground_port=_dead_port(),
            device_source="simulated",
            output_dir=str(self.tmp / "runs"),
        )
        self.logs: list[str] = []
        self.service = svc.AirBlackboxService(
            config,
            program_dir=self.tmp,
            config_path=self.config_path,
            log=self.logs.append,
            reconnect_initial_s=1.0,
            reconnect_max_s=2.0,
        )
        self.addCleanup(self._teardown_service)
        self.addCleanup(air.set_default_device_source, "usb")  # 测试结束恢复进程默认

    def _teardown_service(self) -> None:
        try:
            self.service.acquisition.shutdown(3000)
        except Exception:  # noqa: BLE001
            pass

    # ---- get_status ----

    def test_get_status_contract_fields(self) -> None:
        result = self.service.handle_command("get_status", {})
        self.assertTrue(result["supported"])
        payload = {k: v for k, v in result.items() if k != "supported"}
        for key in (
            "connected", "vna_usb", "run_id", "acquiring", "backlog",
            "disk_free_gb", "program_version", "active_config",
        ):
            self.assertIn(key, payload, key)
        # 与 t3 面板对齐的超集字段
        for key in ("online", "reconnecting", "version", "config", "disk_cleanup"):
            self.assertIn(key, payload, key)
        self.assertEqual(payload["version"], svc.PROGRAM_VERSION)
        self.assertIn("engine", payload["config"])
        self.assertIn("ground_host", payload["config"])
        self.assertFalse(payload["acquiring"])  # autostart=false 等命令

    # ---- 未知 / 暂不支持 action 兜底 ----

    def test_unknown_action_reports_unsupported(self) -> None:
        result = self.service.handle_command("dance_with_the_radar", {})
        self.assertIs(result["supported"], False)
        self.assertEqual([k for k in result if k != "supported"], [])
        # 不崩、服务状态不受影响
        self.assertEqual(self.service.acquisition.state, "closed")

    def test_calibration_actions_supported(self) -> None:
        # 契约变更：黑匣子已移植远程校准编排，cal_* 不再回 supported=false。
        # cal_begin 缺 cal_id → 计划层拒绝（ok=false 的 ack + calibration_result），
        # 但 action 本身是受支持词表（supported=true）。
        result = self.service.handle_command("cal_begin", {})
        self.assertTrue(result["supported"])
        self.assertFalse(result["ok"])
        result = self.service.handle_command("cal_status", {"cal_id": "query"})
        self.assertTrue(result["supported"])
        self.assertTrue(result["ok"])

    # ---- 设置类回写 air_config.json ----

    def test_set_engine_writes_config(self) -> None:
        result = self.service.handle_command("set_engine", {"engine": "legacy"})
        self.assertTrue(result["ok"])
        self.assertEqual(result["engine"], "legacy")
        on_disk = json.loads(self.config_path.read_text(encoding="utf-8"))
        self.assertEqual(on_disk["engine"], "legacy")

    def test_set_engine_rejects_illegal_and_keeps_file(self) -> None:
        result = self.service.handle_command("set_engine", {"engine": "warp"})
        self.assertFalse(result["ok"])
        self.assertIn("error", result)
        self.assertFalse(self.config_path.exists())  # 拒绝时绝不回写

    def test_set_device_source_writes_config_and_takes_effect(self) -> None:
        result = self.service.handle_command("set_device_source", {"device_source": "usb"})
        self.assertTrue(result["ok"])
        self.assertEqual(air.get_default_device_source(), "usb")
        on_disk = json.loads(self.config_path.read_text(encoding="utf-8"))
        self.assertEqual(on_disk["device_source"], "usb")

    def test_set_device_source_rejects_illegal(self) -> None:
        result = self.service.handle_command("set_device_source", {"device_source": "bluetooth"})
        self.assertFalse(result["ok"])
        self.assertEqual(air.get_default_device_source(), "simulated")  # 保持原值

    def test_set_gnss_port_writes_config(self) -> None:
        result = self.service.handle_command("set_gnss_port", {"port": "COM5", "baud": 9600})
        self.assertTrue(result["ok"])
        self.assertEqual(result["port"], "COM5")
        self.assertEqual(result["baud"], 9600)
        on_disk = json.loads(self.config_path.read_text(encoding="utf-8"))
        self.assertEqual(on_disk["gnss_port"], "COM5")
        self.assertEqual(on_disk["gnss_baud"], 9600)

    def test_set_gnss_port_empty_disables(self) -> None:
        self.service.handle_command("set_gnss_port", {"port": "COM5", "baud": 9600})
        result = self.service.handle_command("set_gnss_port", {"port": "", "baud": 115200})
        self.assertTrue(result["ok"])
        on_disk = json.loads(self.config_path.read_text(encoding="utf-8"))
        self.assertEqual(on_disk["gnss_port"], "")  # 空 = 不用 GNSS

    def test_set_gnss_port_rejects_bad_baud(self) -> None:
        result = self.service.handle_command("set_gnss_port", {"port": "COM5", "baud": 0})
        self.assertFalse(result["ok"])

    # ---- list_serial_ports / test_gnss ----

    def test_list_serial_ports(self) -> None:
        fake_ports = [
            SimpleNamespace(device="COM3", description="GNSS Receiver"),
            SimpleNamespace(device="COM7", description="USB Serial"),
        ]
        import serial.tools.list_ports as list_ports_module

        with unittest.mock.patch.object(list_ports_module, "comports", return_value=fake_ports):
            result = self.service.handle_command("list_serial_ports", {})
        self.assertTrue(result["ok"])
        self.assertEqual(
            result["ports"],
            [{"port": "COM3", "description": "GNSS Receiver"},
             {"port": "COM7", "description": "USB Serial"}],
        )

    def test_test_gnss_without_config_reports_error(self) -> None:
        result = self.service.handle_command("test_gnss", {})
        self.assertTrue(result["ok"])
        self.assertFalse(result["fix"])
        self.assertEqual(result["port"], "")
        self.assertEqual(result.get("state"), "waiting")

    def test_test_gnss_with_fake_reader(self) -> None:
        self.service.handle_command("set_gnss_port", {"port": "COM5", "baud": 115200})
        # 常驻 GNSS 监视器持有串口并通过 snapshot 提供快照
        self.service.gnss._port = "COM5"
        fix = air.GgaFix(
            raw="$GPGGA,...", utc="ure",
            latitude=52.0906, longitude=5.1214, fix_quality=1, satellites=9,
            serial_open=True, nmea_received=True,
        )
        self.service.gnss._publish(fix)
        result = self.service.handle_command("test_gnss", {})
        self.assertTrue(result["ok"])
        self.assertTrue(result["fix"])
        self.assertAlmostEqual(result["lat"], 52.0906)
        self.assertAlmostEqual(result["lon"], 5.1214)
        self.assertEqual(result["port"], "COM5")
        self.assertEqual(result["baud"], 115200)

    def test_test_gnss_reader_failure_is_reported(self) -> None:
        self.service.handle_command("set_gnss_port", {"port": "COMX", "baud": 115200})
        err_fix = air.GgaFix(serial_open=False, state="port_not_found")
        self.service.gnss._publish(err_fix, "串口打开失败")
        result = self.service.handle_command("test_gnss", {})
        self.assertTrue(result["ok"])
        self.assertFalse(result["fix"])
        self.assertIn("串口打开失败", result.get("detail", ""))

    # ---- test_vna ----

    def test_test_vna_session_backend(self) -> None:
        fake_backend = SimpleNamespace(
            is_open=True,
            device_info=SimpleNamespace(
                firmware="1.6.5", protocol=14, max_freq_hz=6_000_000_000,
                summary="LibreVNA FW 1.6.5",
            ),
        )
        self.service.acquisition._backend = fake_backend
        result = self.service.handle_command("test_vna", {})
        self.assertTrue(result["ok"])
        self.assertTrue(result["online"])
        self.assertEqual(result["firmware"], "1.6.5")
        self.assertEqual(result["protocol"], 14)
        self.assertEqual(result["max_freq_hz"], 6_000_000_000)
        self.assertEqual(result["source"], "session")

    def test_test_vna_simulated_reports_not_usb(self) -> None:
        result = self.service.handle_command("test_vna", {})
        self.assertFalse(result["ok"])
        self.assertEqual(result["source"], "simulated")

    def test_test_vna_probe_path(self) -> None:
        self.service.config.device_source = "usb"
        air.set_default_device_source("usb")
        self.service._usb_probe = lambda: {
            "ok": True, "online": True, "firmware": "1.6.5",
            "protocol": 14, "max_freq_hz": 6_000_000_000, "source": "probe",
        }
        result = self.service.handle_command("test_vna", {})
        self.assertTrue(result["ok"])
        self.assertEqual(result["source"], "probe")

    def test_vna_present_simulated_is_false(self) -> None:
        self.assertFalse(self.service._vna_present())

    # ---- set_params（无界面等价校验）----

    def test_set_params_updates_headless_sweep(self) -> None:
        result = self.service.handle_command(
            "set_params",
            {"start_mhz": 500, "stop_mhz": 3000, "points": 101, "ifbw_hz": 5000,
             "power_dbm": -10, "interval_s": 0.2, "count": 7},
        )
        self.assertTrue(result["ok"])
        self.assertEqual(self.service._sweep.start_mhz, 500)
        self.assertEqual(self.service._sweep.points, 101)
        self.assertEqual(self.service._interval_s, 0.2)
        self.assertEqual(self.service._count, 7)

    def test_set_params_rejects_bad_ranges(self) -> None:
        for params in (
            {"start_mhz": 3000, "stop_mhz": 500},
            {"points": 1},
            {"ifbw_hz": 1},
            {"power_dbm": 10},
            {"dwell_us": 99999},
            {"ifft_points": 100},
            {"time_window_ns": 0.5},
            {"dewow_window_ns": 0},
        ):
            result = self.service.handle_command("set_params", params)
            self.assertFalse(result["ok"], params)

    def test_set_params_rejects_while_acquiring(self) -> None:
        self.service._active_run_id = "x"
        with unittest.mock.patch.object(
            self.service, "_is_acquiring", return_value=True
        ):
            result = self.service.handle_command("set_params", {"points": 101})
        self.assertFalse(result["ok"])

    # ---- start 兜底（模拟器 + legacy = 与 GUI 同规则拒绝）----

    def test_start_rejects_simulated_legacy_combo(self) -> None:
        self.service.handle_command("set_engine", {"engine": "legacy"})
        result = self.service.handle_command("start", {})
        self.assertTrue(result["supported"])
        self.assertFalse(result["started"])

    def test_start_rejects_bad_sweep(self) -> None:
        self.service._sweep = air.SweepConfig(start_mhz=4000, stop_mhz=400)
        result = self.service.handle_command("start", {})
        self.assertFalse(result["started"])


# =========================================================================
# 磁盘水位：触发/目标线、优先级、报警记账、active run 保护
# =========================================================================


@unittest.skipIf(svc is None, AIR_SERVICE_SKIP)
class DiskWatermarkTests(_TmpDirTestCase):
    RUN_OLD = "20260901_000000_0001"
    RUN_MID = "20260902_000000_0002"
    RUN_NEW = "20260903_000000_0003"

    def _make_run(self, out_dir: Path, run_id: str, *, files: int = 2, size: int = 100) -> None:
        run_dir = out_dir / run_id
        run_dir.mkdir(parents=True)
        for i in range(files):
            (run_dir / f"trace_{i:05d}.npz").write_bytes(b"x" * size)

    def _make_outbox_frame(self, out_dir: Path, run_id: str, index: int = 0) -> Path:
        frame = out_dir / "_outbox" / run_id / f"{index:06d}.frame"
        frame.parent.mkdir(parents=True, exist_ok=True)
        frame.write_bytes(b"f" * 50)
        return frame

    def _manager(self, out_dir: Path, free_bytes_box: dict, **kwargs) -> svc.DiskWatermarkManager:
        # 动态探测：注入基线 + 已删除字节（模拟真实"删除→剩余增加"，探测可注入契约）。
        initial_bytes = sum(p.stat().st_size for p in out_dir.rglob("*") if p.is_file())
        free_bytes_box.setdefault("initial_bytes", initial_bytes)

        def probe() -> int:
            current = sum(p.stat().st_size for p in out_dir.rglob("*") if p.is_file())
            return int(free_bytes_box["free"]) + int(free_bytes_box["initial_bytes"] - current)

        return svc.DiskWatermarkManager(
            out_dir,
            min_free_gb=kwargs.pop("min_free_gb", 20.0),
            target_free_gb=kwargs.pop("target_free_gb", 40.0),
            free_space_bytes=probe,
            on_event=lambda message: self.events.append(message),
            **kwargs,
        )

    def setUp(self) -> None:
        super().setUp()
        self.events: list[str] = []
        self.out_dir = self.tmp / "runs"
        self.out_dir.mkdir()
        self.free_box = {"free": 100 * Gb}

    def test_no_trigger_above_min(self) -> None:
        for run in (self.RUN_OLD, self.RUN_NEW):
            self._make_run(self.out_dir, run)
        manager = self._manager(self.out_dir, self.free_box)
        state = manager.check()
        self.assertFalse(state["triggered"])
        self.assertEqual(list(self.out_dir.iterdir()), [self.out_dir / self.RUN_OLD, self.out_dir / self.RUN_NEW])
        self.assertEqual(state["runs_deleted"], 0)

    def test_deletes_delivered_runs_oldest_first_to_target(self) -> None:
        for run in (self.RUN_OLD, self.RUN_MID, self.RUN_NEW):
            self._make_run(self.out_dir, run, size=Gb // 10)  # 每个 run 2×0.1GB ≈ 0.2GB
        # 剩余 94.75GB < 触发线 95GB → 触发；删最老的 OLD（+0.2GB → 94.95 ≥ 目标 94.9）即停。
        self.free_box["free"] = int(94.75 * Gb)
        manager = self._manager(
            self.out_dir, self.free_box, min_free_gb=95.0, target_free_gb=94.9
        )
        state = manager.check()
        self.assertTrue(state["triggered"])
        self.assertTrue((self.out_dir / self.RUN_MID).exists())
        self.assertTrue((self.out_dir / self.RUN_NEW).exists())
        self.assertFalse((self.out_dir / self.RUN_OLD).exists())
        self.assertEqual(state["runs_deleted"], 1)
        self.assertEqual(state["files_deleted"], 2)
        self.assertFalse(state["undelivered_alarm"])

    def test_delivered_before_undelivered(self) -> None:
        self._make_run(self.out_dir, self.RUN_OLD, size=Gb // 10)
        self._make_run(self.out_dir, self.RUN_NEW, size=Gb // 10)
        self._make_outbox_frame(self.out_dir, self.RUN_NEW)  # NEW 未送达
        self.free_box["free"] = int(94.75 * Gb)
        manager = self._manager(self.out_dir, self.free_box, min_free_gb=95.0, target_free_gb=94.9)
        state = manager.check()
        # 只需删一个：必须删已送达的 OLD，不碰未送达的 NEW
        self.assertFalse((self.out_dir / self.RUN_OLD).exists())
        self.assertTrue((self.out_dir / self.RUN_NEW).exists())
        self.assertTrue((self.out_dir / "_outbox" / self.RUN_NEW / "000000.frame").exists())
        self.assertFalse(state["undelivered_alarm"])

    def test_undelivered_deleted_only_with_alarm_and_accounting(self) -> None:
        self._make_run(self.out_dir, self.RUN_OLD, files=1, size=10)  # 未送达（有帧）
        self._make_outbox_frame(self.out_dir, self.RUN_OLD)
        self.free_box["free"] = 1 * Gb
        manager = self._manager(self.out_dir, self.free_box, min_free_gb=20.0, target_free_gb=40.0)
        state = manager.check()
        self.assertFalse((self.out_dir / self.RUN_OLD).exists())
        self.assertTrue(state["undelivered_alarm"])
        self.assertIn(self.RUN_OLD, state["undelivered_runs_deleted"])
        # 报警必须显式出现在事件流（绝不静默）
        self.assertTrue(any("DISK-ALARM" in e and self.RUN_OLD in e for e in self.events), self.events)
        # 逐条记账：每文件一条 run_file 事件
        run_file_events = [e for e in self.events if "记账 run_file" in e]
        self.assertEqual(len(run_file_events), 1)
        self.assertIn("trace_00000.npz", run_file_events[0])
        self.assertEqual(state["files_deleted"], 1)
        self.assertEqual(state["bytes_freed"], 10)
        self.assertTrue(len(state["recent_events"]) >= 1)

    def test_active_run_never_deleted(self) -> None:
        self._make_run(self.out_dir, self.RUN_OLD, size=10)
        self.free_box["free"] = 0  # 无论如何都清不回目标 → 只剩这一个候选
        manager = self._manager(
            self.out_dir, self.free_box,
            min_free_gb=20.0, target_free_gb=40.0,
            active_run_id=lambda: self.RUN_OLD,
        )
        state = manager.check()
        self.assertTrue((self.out_dir / self.RUN_OLD).exists())  # 采集中 run 绝不删
        self.assertEqual(state["runs_deleted"], 0)

    def test_loose_legacy_files_cleaned_after_run_dirs(self) -> None:
        self._make_run(self.out_dir, self.RUN_OLD, size=Gb // 10)
        loose = self.out_dir / "trace_00001_20250101_000000_000.npz"
        loose.write_bytes(b"L" * (Gb // 10))
        self.free_box["free"] = int(94.75 * Gb)
        manager = self._manager(self.out_dir, self.free_box, min_free_gb=95.0, target_free_gb=94.9)
        manager.check()
        self.assertFalse((self.out_dir / self.RUN_OLD).exists())  # 先删 run 目录
        self.assertTrue(loose.exists())  # 目标已达，散落文件保留
        self.free_box["free"] = 1 * Gb
        manager.check()
        self.assertFalse(loose.exists())  # 仍不足 → 清散落历史文件
        self.assertTrue(any("legacy_file" in e for e in self.events), self.events)

    def test_non_run_entries_untouched(self) -> None:
        keep1 = self.out_dir / "_outbox"
        keep2 = self.out_dir / "_calibration"
        keep3 = self.out_dir / "notes.txt"
        for path in (keep1, keep2):
            path.mkdir()
        keep3.write_text("keep")
        self.free_box["free"] = 1 * Gb
        manager = self._manager(self.out_dir, self.free_box, min_free_gb=20.0, target_free_gb=40.0)
        manager.check()
        self.assertTrue(keep1.exists() and keep2.exists() and keep3.exists())

    def test_calibration_run_dirs_are_candidates(self) -> None:
        cal_run = "20260901_000000_0001_c"
        self._make_run(self.out_dir, cal_run, size=Gb // 10)
        self.free_box["free"] = int(94.75 * Gb)
        manager = self._manager(self.out_dir, self.free_box, min_free_gb=95.0, target_free_gb=94.9)
        state = manager.check()
        self.assertFalse((self.out_dir / cal_run).exists())
        self.assertEqual(state["runs_deleted"], 1)

    def test_outbox_scan_failure_never_deletes_that_run(self) -> None:
        self._make_run(self.out_dir, self.RUN_OLD, size=Gb // 10)
        outbox_run = self.out_dir / "_outbox" / self.RUN_OLD
        outbox_run.mkdir(parents=True)
        self.free_box["free"] = 1 * Gb
        manager = self._manager(self.out_dir, self.free_box, min_free_gb=20.0, target_free_gb=40.0)
        with unittest.mock.patch.object(
            manager, "pending_frame_count", return_value=-1  # 探测故障 → 未知
        ):
            state = manager.check()
        self.assertTrue((self.out_dir / self.RUN_OLD).exists())  # 宁可不删
        self.assertEqual(state["runs_deleted"], 0)


# =========================================================================
# autostart：false 等命令 / true 开机自动采集（真实 simulated 链路）
# =========================================================================


@unittest.skipIf(svc is None or air is None, AIR_SERVICE_SKIP or AIR_SKIP)
class AutostartTests(_TmpDirTestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])

    def _make_config(self, autostart: bool) -> svc.AirConfig:
        return svc.AirConfig(
            ground_host="127.0.0.1",
            ground_port=_dead_port(),
            device_source="simulated",
            engine="continuous",
            output_dir=str(self.tmp / "runs"),
            autostart_acquisition=autostart,
            disk_min_free_gb=0.001,  # 永不触发（测试盘剩余远大于 1MB）
            disk_target_free_gb=0.002,
        )

    def test_autostart_false_waits_for_ground_command(self) -> None:
        service = svc.AirBlackboxService(
            self._make_config(False), program_dir=self.tmp,
            log=lambda m: None, reconnect_initial_s=5.0, reconnect_max_s=10.0,
        )
        self.addCleanup(service.acquisition.shutdown, 3000)
        service.start()
        self.assertFalse(service._is_acquiring())
        self.assertEqual(service.acquisition.state, air.SERVICE_STATE_CLOSED)
        self.assertEqual(service._active_run_id, "")  # 无任务，等地面端命令（D11）

    def test_autostart_true_starts_acquisition_and_stop_command_stops(self) -> None:
        logs: list[str] = []
        service = svc.AirBlackboxService(
            self._make_config(True), program_dir=self.tmp,
            log=logs.append, reconnect_initial_s=5.0, reconnect_max_s=10.0,
        )
        self.addCleanup(service.acquisition.shutdown, 3000)
        service._interval_s = 0.05  # 压节奏：autostart 默认 interval=0 全速跑，测试降速
        service.start()
        self.assertTrue(
            wait_until(lambda: service.acquisition.state == air.SERVICE_STATE_ACQUIRING, 15.0),
            f"autostart=true 未自动开始采集（state={service.acquisition.state}）\n"
            + "\n".join(logs[-12:]),
        )
        self.assertNotEqual(service._active_run_id, "")
        # 等会话真正进入扫频（backend trace_index 增长）再停：启动窗口内下发 stop
        # 会走既有 R-A-09 fail-closed 语义（terminal unknown → 释放 → closed），
        # 那是既有服务的正确行为，不是本契约的验收路径。
        self.assertTrue(
            wait_until(
                lambda: int(service.acquisition.statistics().get("backend_trace_index", 0)) >= 1,
                15.0,
            ),
            "会话未进入扫频状态\n" + "\n".join(logs[-12:]),
        )
        result = service.handle_command("stop", {})
        self.assertTrue(result["ok"])
        self.assertTrue(
            service.acquisition.wait_idle(15000),
            f"停止未回待命（state={service.acquisition.state}）\n" + "\n".join(logs[-12:]),
        )
        # finished_run 回调清 active_run_id 与 wait_idle 唤醒之间存在微小窗口：有界等待。
        self.assertTrue(
            wait_until(lambda: service._active_run_id == "", 5.0),
            f"结束后 run_id 未清空: {service._active_run_id}",
        )


# =========================================================================
# 断线 → 采集继续 + DiskOutbox 本地保存 → 恢复自动补传（真实链路集成）
# =========================================================================


class _MiniGround:
    """最小地面端：accept 循环 + hello/ack、trace/ack_trace（既有协议线）。"""

    def __init__(self) -> None:
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind(("127.0.0.1", 0))
        self._listener.listen(4)
        self.port = int(self._listener.getsockname()[1])
        self._listener.settimeout(0.2)
        self.received: list[tuple[object, object]] = []
        self.statuses: list[dict] = []
        self._conns: list[socket.socket] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _addr = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.settimeout(0.2)
            self._conns.append(conn)
            threading.Thread(target=self._client, args=(conn,), daemon=True).start()

    @staticmethod
    def _recv_exact(conn: socket.socket, size: int) -> bytes:
        buf = b""
        while len(buf) < size:
            chunk = conn.recv(size - len(buf))
            if not chunk:
                raise ConnectionError("socket closed")
            buf += chunk
        return buf

    def _client(self, conn: socket.socket) -> None:
        try:
            while not self._stop.is_set():
                try:
                    (hlen,) = struct.unpack("!I", self._recv_exact(conn, 4))
                    header = json.loads(self._recv_exact(conn, hlen).decode("utf-8"))
                    (plen,) = struct.unpack("!Q", self._recv_exact(conn, 8))
                    if plen:
                        self._recv_exact(conn, plen)
                except (socket.timeout, OSError, ValueError):
                    return
                mtype = header.get("type")
                if mtype == "hello":
                    self._send(conn, {"type": "ack", "protocol": "uav-gpr-telemetry-v1", "run_id": header.get("run_id")})
                elif mtype == "trace":
                    self.received.append((header.get("run_id"), header.get("trace_index")))
                    self._send(conn, {"type": "ack_trace", "protocol": "uav-gpr-telemetry-v1", "trace_index": header.get("trace_index")})
                elif mtype == "acquisition_status":
                    self.statuses.append(header)
                # heartbeat / 其他：忽略
        finally:
            try:
                conn.close()
            except OSError:
                pass

    @staticmethod
    def _send(conn: socket.socket, header: dict) -> None:
        data = json.dumps(header, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        conn.sendall(struct.pack("!I", len(data)) + data + struct.pack("!Q", 0))

    def drop_clients(self) -> None:
        """模拟断链：只断已建立的连接，监听保持（恢复后可重连）。"""
        for conn in list(self._conns):
            try:
                conn.close()
            except OSError:
                pass
        self._conns.clear()

    def stop(self) -> None:
        self._stop.set()
        self.drop_clients()
        try:
            self._listener.close()
        except OSError:
            pass
        self._thread.join(timeout=2.0)


@unittest.skipIf(svc is None or air is None, AIR_SERVICE_SKIP or AIR_SKIP)
class DisconnectedBacklogIntegrationTests(_TmpDirTestCase):
    """黑匣子接线下的完整失联剧本：断线→继续采→落盘→重连→补传清零。

    核心路径零 mock：真实 ContinuousAcquisitionWorker（simulated 设备）、
    真实 ProcessingConsumer、真实 RemoteSender、真实 DiskOutbox。
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])

    def test_disconnect_reconnect_replay(self) -> None:
        ground = _MiniGround()
        self.addCleanup(ground.stop)
        config = svc.AirConfig(
            ground_host="127.0.0.1",
            ground_port=ground.port,
            device_source="simulated",
            engine="continuous",
            output_dir=str(self.tmp / "runs"),
            autostart_acquisition=False,
            disk_min_free_gb=0.001,
            disk_target_free_gb=0.002,
        )
        logs: list[str] = []
        service = svc.AirBlackboxService(
            config,
            program_dir=self.tmp,
            config_path=self.tmp / "air_config.json",
            log=logs.append,
            reconnect_initial_s=0.3,
            reconnect_max_s=1.0,
        )
        self.addCleanup(service.shutdown)
        # 压节奏：interval 50ms（autostart 路径默认 interval=0，测试降速防内存膨胀）。
        service._interval_s = 0.05
        service.start()
        self.assertTrue(
            wait_until(lambda: service.connected, 10.0),
            "黑匣子未自动连上地面端",
        )
        # 配置扫频（5 点，interval 20ms）并启动采集（等价地面端 start 命令）
        self.assertTrue(self.service_start(service))
        run_id = service._active_run_id
        self.assertNotEqual(run_id, "")
        self.assertTrue(
            wait_until(lambda: len(ground.received) >= 3, 30.0),
            f"断线前未收到足够 trace：{len(ground.received)}",
        )
        # ---- 断链：只断已建连接（监听保持）----
        ground.drop_clients()
        self.assertTrue(
            wait_until(lambda: service.reconnecting, 15.0),
            "断链后未进入重连状态",
        )
        # 失联期间采集继续 → outbox 本地保存（未确认帧 ≥1）
        self.assertTrue(
            wait_until(lambda: service.outbox_pending_count() >= 1, 15.0),
            "失联期间 outbox 无未确认帧（采集/落盘断档）",
        )
        pending_during_outage = service.outbox_pending_count()
        # ---- 自动重连（退避 0.3s 起）----
        self.assertTrue(wait_until(lambda: service.connected, 20.0), "未自动重连")
        # ---- 恢复自动补传：outbox 清零、地面端收到补传 ----
        self.assertTrue(
            wait_until(lambda: service.outbox_pending_count() == 0, 60.0),
            f"补传未清零（outbox 还有 {service.outbox_pending_count()} 帧）",
        )
        self.assertGreater(len(ground.received), 3)
        # 失联期间落盘的帧确实被补传到了地面端（不是被丢弃）
        replayed_runs = {run for run, _idx in ground.received}
        self.assertIn(run_id, replayed_runs)
        # ---- 停止采集并回到待命 ----
        result = service.handle_command("stop", {})
        self.assertTrue(result["ok"])
        self.assertTrue(service.acquisition.wait_idle(15000))
        self.assertGreaterEqual(pending_during_outage, 1)

    def service_start(self, service) -> bool:
        self.assertTrue(
            service.handle_command("set_params", {"points": 5, "interval_s": 0.02})["ok"]
        )
        result = service.handle_command("start", {"count": 0})
        return bool(result.get("started"))


def _fake_osl_capturer(n_points: int = 9):
    """确定性 fake capturer：已知 D/T/M 正演 → 各标准件测量 (2, F)。

    与 tests/test_round2_ground_calibration.py 同数学模型（自包含拷贝，
    避免跨测试模块 import 地面站 GUI）：解算可还原，质量指标真实。
    """
    import numpy as np

    frequency = np.linspace(400e6, 4000e6, n_points)

    def _terms(seed: float):
        index = np.arange(n_points, dtype=float) + seed
        return (
            0.02 * np.exp(1j * 0.31 * index),
            0.8 * np.exp(-1j * 0.17 * index),
            0.05 * np.exp(1j * 0.9 * index),
        )

    def _forward(d: complex, t: complex, m: complex, gamma: complex) -> complex:
        return d + gamma * t / (1.0 - gamma * m)

    table: dict[str, np.ndarray] = {}
    for tag, terms in (("P1", _terms(0.0)), ("P2", _terms(5.0))):
        d, t, m = terms
        table[f"{tag}_Open"] = np.array([_forward(d[i], t[i], m[i], 1.0 + 0j) for i in range(n_points)])
        table[f"{tag}_Short"] = np.array([_forward(d[i], t[i], m[i], -1.0 + 0j) for i in range(n_points)])
        table[f"{tag}_Load"] = np.array([_forward(d[i], t[i], m[i], 0.0 + 0j) for i in range(n_points)])

    class _FakeCapturer:
        def capture_one(self, step, index):
            values = table[step.name]
            s11 = values if step.channel == "S11" else np.full(n_points, 0.3 + 0.1j)
            s22 = values if step.channel == "S22" else np.full(n_points, 0.2 - 0.1j)
            return frequency, np.vstack([s11, s22]).astype(np.complex128)

    return _FakeCapturer(), frequency


@unittest.skipIf(svc is None or air is None, AIR_SERVICE_SKIP or AIR_SKIP)
class BlackboxCalibrationTests(_TmpDirTestCase):
    """黑匣子远程校准编排：cal_* 全词表可用（回归"请求采集中"卡死）。

    - 回程帧经实例级包装记录（不断言真实 socket）；
    - OSL 全链用 fake ServiceStepCapturer（确定性、无 USB，patch 点与
      test_round2_ground_calibration.py 相同：air.ServiceStepCapturer）；
    - 背景模式跑一条真实 simulated 链路，证明生产 capturer 无界面可用。
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])

    def setUp(self) -> None:
        super().setUp()
        config = svc.AirConfig(
            ground_host="127.0.0.1",
            ground_port=_dead_port(),
            device_source="simulated",
            output_dir=str(self.tmp / "runs"),
        )
        self.logs: list[str] = []
        self.service = svc.AirBlackboxService(
            config,
            program_dir=self.tmp,
            config_path=self.tmp / "air_config.json",
            log=self.logs.append,
            reconnect_initial_s=1.0,
            reconnect_max_s=2.0,
        )
        self.addCleanup(self._teardown_service)
        self.addCleanup(air.set_default_device_source, "usb")
        # 回程记录：只看 header/payload，不经过真实 socket。
        self.frames: list[tuple[dict, bytes]] = []
        sender = self.service.sender
        original = sender.enqueue_calibration

        def _record(header, payload=b""):
            self.frames.append((dict(header), bytes(payload)))
            return original(header, payload)

        sender.enqueue_calibration = _record  # type: ignore[method-assign]

    def _teardown_service(self) -> None:
        try:
            thread = self.service._calibration_thread
            if thread is not None and thread.is_alive():
                thread.join(30.0)
        except Exception:  # noqa: BLE001
            pass
        try:
            self.service.acquisition.shutdown(3000)
        except Exception:  # noqa: BLE001
            pass

    def _patch_step_capturer(self, capturer) -> None:
        class _FakeStepCapturer:
            def __init__(self, **kwargs):
                self.kwargs = kwargs
                self.closed = False

            def capture_one(self, step, index):
                return capturer.capture_one(step, index)

            def close(self):
                self.closed = True

        patcher = unittest.mock.patch.object(air, "ServiceStepCapturer", _FakeStepCapturer)
        patcher.start()
        self.addCleanup(patcher.stop)

    def _begin(self, cal_id: str, mode: str = "osl", repeats: int = 2, points: int = 9):
        return self.service.handle_command(
            "cal_begin",
            {
                "cal_id": cal_id,
                "mode": mode,
                "repeats": repeats,
                "config": {
                    "start_mhz": 400.0,
                    "stop_mhz": 4000.0,
                    "points": points,
                    "ifbw_hz": 10000,
                    "power_dbm": -20.0,
                    "dwell_us": 0,
                },
            },
        )

    def _results(self, status: str | None = None) -> list[dict]:
        headers = [h for h, _p in self.frames if h.get("type") == "calibration_result"]
        if status is not None:
            headers = [h for h in headers if h.get("status") == status]
        return headers

    def _captures(self) -> list[tuple[dict, bytes]]:
        return [(h, p) for h, p in self.frames if h.get("type") == "calibration_capture"]

    def _capture_step_and_wait(self, cal_id: str, step: str, timeout_s: float = 30.0) -> None:
        result = self.service.handle_command("cal_capture", {"cal_id": cal_id, "step": step})
        self.assertTrue(result["ok"], f"cal_capture({step}) 未受理: {result}")
        self.assertTrue(
            wait_until(lambda: self.service._calibration_thread is None, timeout_s),
            f"步骤 {step} 采集线程未在 {timeout_s}s 内结束",
        )

    # ---- 会话建立与互斥 ----

    def test_begin_plans_session_and_reports_planned(self) -> None:
        result = self._begin("cal_blackbox_001")
        self.assertTrue(result["supported"])
        self.assertTrue(result["ok"])
        session = self.service._calibration_session
        self.assertIsNotNone(session)
        assert session is not None
        self.assertEqual(session.state, "planned")
        self.assertEqual(
            session.capture_dir, self.service.output_root / "_calibration" / "cal_blackbox_001"
        )
        planned = self._results("planned")
        self.assertEqual(len(planned), 1)
        self.assertEqual(planned[0].get("mode"), "osl")
        self.assertEqual(len(planned[0].get("steps")), 6)

    def test_begin_rejected_while_acquiring(self) -> None:
        worker = unittest.mock.Mock()
        worker.isRunning.return_value = True
        self.service._legacy_worker = worker
        self.addCleanup(setattr, self.service, "_legacy_worker", None)
        result = self._begin("cal_rejected")
        self.assertTrue(result["supported"])
        self.assertFalse(result["ok"])
        rejected = self._results("rejected")
        self.assertEqual(len(rejected), 1)
        self.assertIn("采集任务运行中", str(rejected[0].get("reason")))
        self.assertIsNone(self.service._calibration_session)

    def test_start_rejected_while_session_open(self) -> None:
        self._begin("cal_blocks_start")
        # R-C-14 采集方向：会话进行中拒绝启动（不建 worker）。
        self.assertFalse(self.service.start_acquisition())
        self.assertIsNone(self.service._legacy_worker)
        self.assertTrue(
            any("校准会话进行中" in line for line in self.logs), f"日志尾={self.logs[-3:]}"
        )
        result = self.service.handle_command(
            "cal_cancel", {"cal_id": "cal_blocks_start", "reason": "测试取消"}
        )
        self.assertTrue(result["ok"])
        self.assertIsNone(self.service._calibration_session)
        cancelled = self._results("cancelled")
        self.assertEqual(len(cancelled), 1)

    def test_capture_wrong_step_rejected(self) -> None:
        self._begin("cal_order")
        result = self.service.handle_command(
            "cal_capture", {"cal_id": "cal_order", "step": "P1_Short"}
        )
        self.assertFalse(result["ok"])
        rejected = self._results("rejected")
        self.assertEqual(len(rejected), 1)
        self.assertIn("P1_Open", str(rejected[0].get("reason")))
        self.assertIsNone(self.service._calibration_thread)

    # ---- OSL 全链（fake capturer，确定性）----

    def test_full_osl_loop_with_fake_capturer(self) -> None:
        capturer, _frequency = _fake_osl_capturer(9)
        self._patch_step_capturer(capturer)
        cal_id = "cal_blackbox_osl"
        self.assertTrue(self._begin(cal_id)["ok"])
        for step in air.OSL_STEP_NAMES:
            self._capture_step_and_wait(cal_id, step)
        # 逐 sweep 回程：6 步 × 2 条，index/of 递增。
        captures = self._captures()
        self.assertEqual(len(captures), 12)
        first, last = captures[0][0], captures[-1][0]
        self.assertEqual((first.get("index"), first.get("of")), (1, 2))
        self.assertEqual((last.get("index"), last.get("of")), (2, 2))
        self.assertTrue(all(len(payload) > 0 for _h, payload in captures))
        # 解算：质量回程 ×6 + solved + 落盘。
        self.assertTrue(self.service.handle_command("cal_finish", {"cal_id": cal_id})["ok"])
        session = self.service._calibration_session
        assert session is not None
        self.assertEqual(session.state, "solved")
        quality = [h for h, _p in self.frames if h.get("type") == "calibration_quality"]
        self.assertEqual(len(quality), 6)
        solved = self._results("solved")
        self.assertEqual(len(solved), 1)
        self.assertEqual(solved[0].get("format"), "uav_gpr_solt_calibration_v2")
        cal_dir = self.service.output_root / "_calibration" / cal_id
        self.assertEqual(len(list(cal_dir.glob("capture_*.npz"))), 12)
        self.assertTrue((cal_dir / f"{cal_id}.json").exists())
        # 激活：生效组件写回服务自身状态（R-C-16 无界面版）。
        self.assertTrue(self.service.handle_command("cal_activate", {"cal_id": cal_id})["ok"])
        self.assertEqual(self.service.active_calibration_id, cal_id)
        self.assertIsNotNone(self.service._session_calibration)
        self.assertIsNotNone(self.service._session_calibration_p2)
        self.assertEqual(len(self.service._session_calibration.points), 9)
        self.assertEqual(len(self._results("activated")), 1)
        # 地面可查询当前生效校准摘要。
        self.frames.clear()
        self.assertTrue(
            self.service.handle_command("cal_status", {"cal_id": "query"})["ok"]
        )
        active = self._results("active")
        self.assertEqual(len(active), 1)
        self.assertEqual(active[0].get("calibration_id"), cal_id)

    # ---- 背景模式（真实 simulated 链路，生产 capturer）----

    def test_background_loop_with_real_simulated_device(self) -> None:
        cal_id = "cal_blackbox_bg"
        self.assertTrue(self._begin(cal_id, mode="background", repeats=1, points=5)["ok"])
        self._capture_step_and_wait(cal_id, air.BACKGROUND_STEP_NAME, timeout_s=90.0)
        captures = self._captures()
        self.assertEqual(len(captures), 1)
        header, payload = captures[0]
        self.assertEqual(header.get("step"), air.BACKGROUND_STEP_NAME)
        self.assertEqual((header.get("index"), header.get("of")), (1, 1))
        self.assertGreater(len(payload), 0)
        self.assertTrue(self.service.handle_command("cal_finish", {"cal_id": cal_id})["ok"])
        self.assertEqual(len(self._results("solved")), 1)
        self.assertTrue(self.service.handle_command("cal_activate", {"cal_id": cal_id})["ok"])
        self.assertEqual(self.service.active_calibration_id, cal_id)
        self.assertIsNotNone(self.service._background)
        self.assertEqual(len(self.service._background), 5)
        self.assertIsNotNone(self.service._background_frequency_hz)

    # ---- 收尾 ----

    def test_shutdown_cancels_open_session(self) -> None:
        self._begin("cal_shutdown")
        self.service.shutdown()
        self.assertIsNone(self.service._calibration_session)
        marker = self.service.output_root / "_calibration" / "cal_shutdown" / "incomplete.json"
        self.assertTrue(marker.exists())


if __name__ == "__main__":
    unittest.main()
