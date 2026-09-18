"""阶段 4 验收测试：acquisition/flight metadata 与地面兼容（roadmap §9.3、§10、§11）。

覆盖：
- save_trace / trace_npz_bytes 只增 ``acquisition``/``flight`` 字段，保留全部旧
  NPZ key 与 CSV 列；
- RemoteSender.enqueue_trace header 增加字段，保持 uav-gpr-telemetry-v1 framing
  与 SHA-256/ACK 语义；未提供 metadata 时 header/NPZ 与旧行为一致；
- ProcessingConsumer 组装 acquisition（含首末点/中点时间、设备 sweep 序号、
  connection_generation、GNSS age、会话统计）与 flight（gnss_valid / MSL 海拔 /
  AGL=未知），无效/陈旧 Fix 绝不伪装为有效坐标；
- ContinuousAcquisitionWorker 端到端填充 SelectedSweep 新字段；
- GroundStationWindow 显示新字段，旧 payload（缺 acquisition/flight）仍可显示。

全部使用 FakeTransport，无真实硬件 / 串口 / 网络。
"""

from __future__ import annotations

import hashlib
import io
import json
import os
import socket
import struct
import tempfile
import threading
import time
import unittest
import unittest.mock
import weakref
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    import uav_gpr_qt as air
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover - environment dependent
    air = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

from PySide6 import QtCore, QtGui, QtWidgets

from librevna.librevna_continuous import PersistentBackendSettings, PersistentLibreVnaBackend
from librevna.librevna_errors import LibreVnaTimeoutError
from librevna.librevna_protocol import (
    ACK,
    DEVICE_INFO,
    REQUEST_DEVICE_INFO,
    SET_IDLE,
    SWEEP_SETTINGS,
    VNA_DATAPOINT,
    encode_packet,
)


def _shutdown_window(window, timeout_ms: int = 2000) -> None:
    """fail-closed 窗口清理：stop 后检查 wait 返回值、断言 receiver 已退出，再关闭/删除。

    禁止忽略 wait() 返回值；deleteLater 前必须确认 receiver.isRunning() 为 False；
    异常路径也由调用方 finally 调用本函数完成清理。
    """
    receiver = window.receiver
    receiver.stop()
    if not receiver.wait(timeout_ms):
        raise AssertionError("receiver 未在期限内退出（wait 返回 False）")
    if receiver.isRunning():
        raise AssertionError("receiver 仍运行（QThread 残留）")
    window.close()
    window.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    app = QtWidgets.QApplication.instance()
    if app is not None:
        app.processEvents()


def _device_info_payload(num_ports: int = 2) -> bytes:
    return struct.pack(
        "<HBBBBcQQIIHhhIIBQBH",
        14, 1, 2, 3, 5, b"A",
        100_000_000, 6_000_000_000, 1, 1_000_000, 10_001,
        -3000, 1000, 1, 1_000_000, 200, 12_000_000_000, num_ports, 65_535,
    )


def _dual_receivers(s11: complex, s22: complex) -> list[tuple[int, complex]]:
    return [
        (0x01, s11),
        (0x02, complex(9.0, 9.0)),
        (0x13, complex(1.0, 0.0)),
        (0x21, complex(7.0, 7.0)),
        (0x22, s22),
        (0x33, complex(1.0, 0.0)),
    ]


def _dual_payload(
    receivers: list[tuple[int, complex]],
    *,
    freq_hz: int = 500_000_000,
    point_number: int = 0,
) -> bytes:
    n = len(receivers)
    payload = struct.pack("<QhH", int(freq_hz), -1000, int(point_number))
    payload += struct.pack("<" + "f" * n, *[v.real for _, v in receivers])
    payload += struct.pack("<" + "f" * n, *[v.imag for _, v in receivers])
    payload += bytes(d for d, _ in receivers)
    return payload


def _vna_point_packet(freq_hz: float, point_number: int, s11: complex, s22: complex) -> bytes:
    return encode_packet(
        VNA_DATAPOINT,
        _dual_payload(_dual_receivers(s11, s22), freq_hz=freq_hz, point_number=point_number),
    )


class _RepeatingSweepTransport:
    """按需连续生成完整 sweep 的 fake transport（对齐 test_librevna_backend.py 字节布局）。"""

    def __init__(self, n_points: int = 5, *, max_sweeps: int | None = None) -> None:
        self.n_points = n_points
        self.max_sweeps = max_sweeps
        self._point = 0
        self.sweeps_generated = 0
        self.open_count = 0
        self.close_count = 0
        self.opened = False
        self.writes: list[bytes] = []
        self._reply = "data"

    def open(self) -> None:
        self.open_count += 1
        self.opened = True

    def close(self) -> None:
        self.close_count += 1
        self.opened = False

    @property
    def is_open(self) -> bool:
        return self.opened

    def read(self, max_length: int, timeout_ms: int) -> bytes:
        if not self.opened:
            raise LibreVnaTimeoutError("not open")
        if self._reply == "device_info":
            self._reply = "data"
            return encode_packet(DEVICE_INFO, _device_info_payload())
        if self._reply == "ack":
            self._reply = "data"
            return encode_packet(ACK)
        if self.max_sweeps is not None and self.sweeps_generated >= self.max_sweeps:
            raise LibreVnaTimeoutError("exhausted")
        point = self._point
        self._point += 1
        if self._point >= self.n_points:
            self._point = 0
            self.sweeps_generated += 1
        return _vna_point_packet(
            500_000_000 + point * 10_000_000,
            point,
            complex(0.5, -0.2),
            complex(0.3, 0.1),
        )

    def write(self, data: bytes) -> None:
        if not self.opened:
            raise LibreVnaTimeoutError("not open")
        self.writes.append(bytes(data))
        if len(data) >= 4 and data[0] == 0x5A:
            packet_type = data[3]
            if packet_type == REQUEST_DEVICE_INFO:
                self._reply = "device_info"
            elif packet_type in (SET_IDLE, SWEEP_SETTINGS):
                self._reply = "ack"
            else:
                self._reply = "data"


# ----------------------------------------------------------------------
# metadata 构造 helpers
# ----------------------------------------------------------------------

_MIDPOINT = 100.0


def _cfg() -> air.SweepConfig:
    return air.SweepConfig(
        start_mhz=400,
        stop_mhz=4000,
        points=5,
        ifbw_hz=10000,
        ifft_points=8,
        time_window_ns=8.0,
    )


def _item(trace_index: int = 3) -> air.SelectedSweep:
    freq = np.array([400e6, 1300e6, 2200e6, 3100e6, 4000e6], dtype=float)
    raw = np.array(
        [
            [0.5 + 0.1j, 0.4 + 0.1j, 0.3 + 0.1j, 0.2 + 0.1j, 0.1 + 0.1j],
            [0.3 + 0.1j, 0.3 + 0.05j, 0.2 + 0.05j, 0.1 + 0.05j, 0.05 + 0.05j],
        ],
        dtype=np.complex128,
    )
    return air.SelectedSweep(
        trace_index=trace_index,
        device_sweep_index=trace_index + 98,
        frequency_hz=freq,
        raw=raw,
        cfg=_cfg(),
        midpoint_monotonic=_MIDPOINT,
        selected_at_utc="2026-08-14T00:00:01.000+00:00",
        missed_slots=0,
        actual_interval_s=1.01,
        schedule_error_ms=10.0,
        acquired_at_utc="2026-08-14T00:00:00.500+00:00",
        sweep_started_utc="2026-08-14T00:00:00.100+00:00",
        sweep_completed_utc="2026-08-14T00:00:00.900+00:00",
        scheduled_at_utc="2026-08-14T00:00:00.000+00:00",
        connection_generation=2,
        target_interval_s=1.0,
        session_statistics={
            "mode": "continuous",
            "backend": {"completed_sweeps": 120},
            "scheduler": {"selected_sweeps": 3},
        },
    )


def _dual_payload_parts(item: air.SelectedSweep):
    raw = air.dual_channel_raw(item.raw)
    display, time_ns, traces = air.process_trace_pipeline(item.frequency_hz, raw, item.cfg)
    return raw, display, time_ns, traces


def _valid_fix() -> air.GgaFix:
    return air.GgaFix(fix_quality=1, latitude=30.5, longitude=114.3, satellites=8, altitude_m=12.5)


def _acquisition_expected(item: air.SelectedSweep) -> dict[str, object]:
    return {
        "acquired_at_utc": item.acquired_at_utc,
        "sweep_started_utc": item.sweep_started_utc,
        "sweep_completed_utc": item.sweep_completed_utc,
        "scheduled_at_utc": item.scheduled_at_utc,
        "selected_at_utc": item.selected_at_utc,
        "device_sweep_index": item.device_sweep_index,
        "selected_trace_index": item.trace_index,
        "connection_generation": item.connection_generation,
        "target_interval_s": item.target_interval_s,
        "actual_interval_s": item.actual_interval_s,
        "schedule_error_ms": item.schedule_error_ms,
        "session_statistics": item.session_statistics,
    }


class _FakeSender:
    """记录 enqueue_trace 调用的假 RemoteSender。"""

    def __init__(self) -> None:
        self.calls: list[tuple[tuple, dict]] = []

    def enqueue_trace(self, *args, **kwargs) -> None:
        self.calls.append((args, kwargs))


class _RecordingQueueConsumer(QtCore.QThread):
    """录制选中 sweep 的消费者（worker 端到端测试）。"""

    trace_ready = QtCore.Signal(int, object, object, object, object, object, object, object, str)
    status = QtCore.Signal(str)
    failed = QtCore.Signal(str)
    fatal_error = QtCore.Signal(str)

    def __init__(self, queue, _gnss_cache) -> None:
        super().__init__()
        self._q = queue
        self._stop = threading.Event()
        self.items: list[air.SelectedSweep] = []

    def request_stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        while not self._stop.is_set():
            item = self._q.get(timeout=0.02)
            if item is None:
                continue
            self.items.append(item)
        while True:
            item = self._q.get(timeout=0.01)
            if item is None:
                break
            self.items.append(item)


def _wait_until(condition, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.01)
    return False


# ----------------------------------------------------------------------
# save_trace / trace_npz_bytes / RemoteSender
# ----------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class SaveTraceMetadataTests(unittest.TestCase):
    """save_trace 只增 acquisition/flight 字段；旧 NPZ key 与 CSV 列保持。"""

    def setUp(self) -> None:
        self.out_dir = Path(tempfile.mkdtemp())
        self.item = _item()
        self.raw, self.display, self.time_ns, self.traces = _dual_payload_parts(self.item)
        self.acquisition = _acquisition_expected(self.item)
        self.acquisition["gnss_age_ms"] = 2000.0
        self.flight = {
            "gnss_valid": True,
            "gnss_altitude_msl_m": 12.5,
            "flight_height_agl_m": None,
        }

    def test_new_payload_adds_acquisition_and_flight_and_keeps_old_keys(self) -> None:
        csv_path = air.save_trace(
            self.out_dir,
            self.item.trace_index,
            self.item.cfg,
            self.item.frequency_hz,
            self.raw,
            self.display,
            self.time_ns,
            self.traces,
            _valid_fix(),
            channels=("S11", "S22"),
            acquisition=self.acquisition,
            flight=self.flight,
        )
        npz_path = csv_path.with_suffix(".npz")
        with np.load(npz_path, allow_pickle=False) as data:
            keys = set(data.files)
            # 旧 key 全部保留。
            for expected in (
                "config",
                "processing",
                "frequency_hz",
                "time_ns",
                "trace",
                "gnss",
                "channels",
                "raw_S11_real",
                "raw_S11_imag",
                "display_S11_real",
                "display_S11_imag",
                "trace_S11",
                "raw_S22_real",
                "raw_S22_imag",
                "display_S22_real",
                "display_S22_imag",
                "trace_S22",
            ):
                self.assertIn(expected, keys, f"缺少旧 NPZ key: {expected}")
            self.assertIn("acquisition", keys)
            self.assertIn("flight", keys)
            self.assertIn("trace_S11_complex", keys)
            self.assertIn("trace_S22_complex", keys)
            self.assertEqual(data["trace_S11_complex"].dtype, np.complex128)
            self.assertEqual(data["trace_S22_complex"].dtype, np.complex128)
            np.testing.assert_allclose(data["trace_S11"], data["trace_S11_complex"].real)
            np.testing.assert_allclose(data["trace_S22"], data["trace_S22_complex"].real)
            acquisition = json.loads(str(data["acquisition"].item()))
            self.assertEqual(
                {k: acquisition[k] for k in self.acquisition},
                self.acquisition,
            )
            self.assertTrue(acquisition["saved_at_utc"], "saved_at_utc 必须非空")
            flight = json.loads(str(data["flight"].item()))
            self.assertEqual(flight, self.flight)

    def test_run_metadata_jsonl_includes_acquisition_and_flight(self) -> None:
        csv_path = air.save_trace(
            self.out_dir,
            self.item.trace_index,
            self.item.cfg,
            self.item.frequency_hz,
            self.raw,
            self.display,
            self.time_ns,
            self.traces,
            _valid_fix(),
            channels=("S11", "S22"),
            acquisition=self.acquisition,
            flight=self.flight,
        )
        line = (self.out_dir / "run_metadata.jsonl").read_text(encoding="utf-8").strip().splitlines()[-1]
        meta = json.loads(line)
        self.assertIn("acquisition", meta)
        self.assertIn("flight", meta)
        self.assertEqual(meta["acquisition"]["selected_trace_index"], self.item.trace_index)
        self.assertEqual(meta["flight"], self.flight)
        self.assertTrue(meta["acquisition"]["saved_at_utc"])

    def test_csv_columns_unchanged(self) -> None:
        csv_path = air.save_trace(
            self.out_dir,
            self.item.trace_index,
            self.item.cfg,
            self.item.frequency_hz,
            self.raw,
            self.display,
            self.time_ns,
            self.traces,
            _valid_fix(),
            channels=("S11", "S22"),
            acquisition=self.acquisition,
            flight=self.flight,
        )
        header = csv_path.read_text(encoding="utf-8-sig").splitlines()[0]
        self.assertTrue(header.startswith("frequency_hz,frequency_mhz"))
        self.assertIn("s11_real_raw", header)
        self.assertIn("s22_real_display", header)
        # acquisition/flight 绝不引入 CSV 列。
        self.assertNotIn("acquisition", header)
        self.assertNotIn("flight", header)

    def test_without_metadata_produces_old_style_payload(self) -> None:
        csv_path = air.save_trace(
            self.out_dir,
            self.item.trace_index,
            self.item.cfg,
            self.item.frequency_hz,
            self.raw,
            self.display,
            self.time_ns,
            self.traces,
            _valid_fix(),
            channels=("S11", "S22"),
        )
        npz_path = csv_path.with_suffix(".npz")
        with np.load(npz_path, allow_pickle=False) as data:
            self.assertNotIn("acquisition", data.files)
            self.assertNotIn("flight", data.files)
        meta_line = (self.out_dir / "run_metadata.jsonl").read_text(encoding="utf-8").strip()
        self.assertNotIn("acquisition", meta_line)


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class TraceNpzBytesMetadataTests(unittest.TestCase):
    """trace_npz_bytes 只增字段；不提供 metadata 时字节级保持旧结构。"""

    def setUp(self) -> None:
        self.item = _item()
        self.raw, self.display, self.time_ns, self.traces = _dual_payload_parts(self.item)
        self.acquisition = _acquisition_expected(self.item)
        self.acquisition["gnss_age_ms"] = None
        self.flight = {
            "gnss_valid": False,
            "gnss_altitude_msl_m": None,
            "flight_height_agl_m": None,
        }

    def test_new_payload_contains_acquisition_and_flight(self) -> None:
        finalized = {**self.acquisition, "saved_at_utc": "2026-08-14T01:00:00.000+00:00"}
        payload = air.trace_npz_bytes(
            self.item.trace_index,
            self.item.cfg,
            self.item.frequency_hz,
            self.raw,
            self.display,
            self.time_ns,
            self.traces,
            _valid_fix(),
            channels=("S11", "S22"),
            acquisition=finalized,
            flight=self.flight,
        )
        with np.load(io.BytesIO(payload), allow_pickle=False) as data:
            self.assertIn("acquisition", data.files)
            self.assertIn("flight", data.files)
            acquisition = json.loads(str(data["acquisition"].item()))
            self.assertEqual(acquisition["device_sweep_index"], self.item.device_sweep_index)
            # 逐字写入：调用方预设的 saved_at_utc 不被覆盖。
            self.assertEqual(acquisition["saved_at_utc"], finalized["saved_at_utc"])
            self.assertEqual(json.loads(str(data["flight"].item())), self.flight)

    def test_trace_npz_bytes_does_not_overwrite_preset_saved_at(self) -> None:
        finalized = {**self.acquisition, "saved_at_utc": "2026-08-14T09:00:00.000+00:00"}
        payload = air.trace_npz_bytes(
            self.item.trace_index,
            self.item.cfg,
            self.item.frequency_hz,
            self.raw,
            self.display,
            self.time_ns,
            self.traces,
            _valid_fix(),
            channels=("S11", "S22"),
            acquisition=finalized,
        )
        with np.load(io.BytesIO(payload), allow_pickle=False) as data:
            parsed = json.loads(str(data["acquisition"].item()))
        self.assertEqual(parsed, finalized)

    def test_without_metadata_has_no_new_keys(self) -> None:
        payload = air.trace_npz_bytes(
            self.item.trace_index,
            self.item.cfg,
            self.item.frequency_hz,
            self.raw,
            self.display,
            self.time_ns,
            self.traces,
            _valid_fix(),
            channels=("S11", "S22"),
        )
        with np.load(io.BytesIO(payload), allow_pickle=False) as data:
            self.assertNotIn("acquisition", data.files)
            self.assertNotIn("flight", data.files)


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class RemoteSenderMetadataTests(unittest.TestCase):
    """enqueue_trace 保持 uav-gpr-telemetry-v1 framing；metadata 只增 header 字段。"""

    def setUp(self) -> None:
        self.item = _item()
        self.raw, self.display, self.time_ns, self.traces = _dual_payload_parts(self.item)
        self.acquisition = _acquisition_expected(self.item)
        self.acquisition["gnss_age_ms"] = 2000.0
        self.flight = {
            "gnss_valid": True,
            "gnss_altitude_msl_m": 12.5,
            "flight_height_agl_m": None,
        }
        self.sender = air.RemoteSender(
            "127.0.0.1", 0, "meta-test", status_callback=lambda _s: None
        )

    def test_header_includes_metadata_and_keeps_framing_fields(self) -> None:
        self.sender.enqueue_trace(
            self.item.trace_index,
            self.item.cfg,
            self.item.frequency_hz,
            self.raw,
            self.display,
            self.time_ns,
            self.traces,
            _valid_fix(),
            channels=("S11", "S22"),
            acquisition=self.acquisition,
            flight=self.flight,
        )
        header, payload = self.sender.queue.get_nowait()
        self.assertEqual(header["type"], "trace")
        self.assertEqual(header["protocol"], "uav-gpr-telemetry-v1")
        self.assertEqual(header["trace_index"], self.item.trace_index)
        self.assertEqual(header["payload_format"], "npz")
        self.assertEqual(header["payload_bytes"], len(payload))
        self.assertEqual(header["sha256"], hashlib.sha256(payload).hexdigest())
        self.assertEqual(header["acquisition"]["selected_trace_index"], self.item.trace_index)
        self.assertEqual(header["acquisition"]["gnss_age_ms"], 2000.0)
        self.assertTrue(header["acquisition"]["saved_at_utc"])
        # 顶层 acquired_at_utc 与 acquisition 内一致。
        self.assertEqual(header["acquired_at_utc"], header["acquisition"]["acquired_at_utc"])
        self.assertEqual(header["acquired_at_utc"], self.item.acquired_at_utc)
        self.assertEqual(header["flight"], self.flight)
        # NPZ payload 内部 acquisition 与 header 逐字段完全一致。
        with np.load(io.BytesIO(payload), allow_pickle=False) as data:
            self.assertIn("acquisition", data.files)
            self.assertIn("flight", data.files)
            parsed = json.loads(str(data["acquisition"].item()))
        self.assertEqual(header["acquisition"], parsed)
        self.assertEqual(parsed["saved_at_utc"], header["acquisition"]["saved_at_utc"])

    def test_header_and_payload_acquisition_identical_under_mocked_time(self) -> None:
        """即使 utc helper 每次返回不同值，同一遥测对象的 header 与 payload 仍一致。"""
        values = ["2026-08-14T01:00:00.001+00:00", "2026-08-14T01:00:00.002+00:00"]
        with unittest.mock.patch.object(air, "utc_iso_now", side_effect=lambda: values.pop(0)):
            self.sender.enqueue_trace(
                self.item.trace_index,
                self.item.cfg,
                self.item.frequency_hz,
                self.raw,
                self.display,
                self.time_ns,
                self.traces,
                _valid_fix(),
                channels=("S11", "S22"),
                acquisition=self.acquisition,
                flight=self.flight,
            )
        header, payload = self.sender.queue.get_nowait()
        with np.load(io.BytesIO(payload), allow_pickle=False) as data:
            parsed = json.loads(str(data["acquisition"].item()))
        self.assertEqual(header["acquisition"], parsed)
        self.assertEqual(header["acquired_at_utc"], header["acquisition"]["acquired_at_utc"])
        # enqueue_trace 只生成一次 saved_at_utc，取首值，header 与 NPZ 共享。
        self.assertEqual(parsed["saved_at_utc"], "2026-08-14T01:00:00.001+00:00")

    def test_without_metadata_header_has_no_new_fields(self) -> None:
        self.sender.enqueue_trace(
            self.item.trace_index,
            self.item.cfg,
            self.item.frequency_hz,
            self.raw,
            self.display,
            self.time_ns,
            self.traces,
            _valid_fix(),
            channels=("S11", "S22"),
        )
        header, payload = self.sender.queue.get_nowait()
        self.assertNotIn("acquisition", header)
        self.assertNotIn("flight", header)
        self.assertNotIn("acquired_at_utc", header)
        self.assertEqual(header["payload_bytes"], len(payload))
        self.assertEqual(header["sha256"], hashlib.sha256(payload).hexdigest())


# ----------------------------------------------------------------------
# ProcessingConsumer 元数据组装
# ----------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class ProcessingConsumerMetadataTests(unittest.TestCase):
    """消费者按 sweep 中点匹配最近未过期 Fix，组装完整 acquisition/flight。"""

    def _consumer(self, cache: air.GnssFixCache, sender: _FakeSender | None = None):
        return air.ProcessingConsumer(
            queue=air.BoundedQueue(8),
            gnss_cache=cache,
            cfg=_cfg(),
            out_dir=Path(tempfile.mkdtemp()),
            autosave=False,
            calibration_p1=None,
            calibration_p2=None,
            through_measurement=None,
            background_frequency_hz=None,
            background=None,
            remote_sender=sender,
        )

    def test_valid_fix_populates_full_metadata(self) -> None:
        cache = air.GnssFixCache(clock=lambda: 98.0)
        self.assertTrue(cache.update(_valid_fix()))
        sender = _FakeSender()
        consumer = self._consumer(cache, sender)
        consumer._process_one(_item())
        self.assertEqual(consumer.processed_count, 1)
        args, kwargs = sender.calls[0]
        self.assertEqual(kwargs["acquisition"]["acquired_at_utc"], _item().acquired_at_utc)
        self.assertEqual(kwargs["acquisition"]["sweep_started_utc"], _item().sweep_started_utc)
        self.assertEqual(kwargs["acquisition"]["sweep_completed_utc"], _item().sweep_completed_utc)
        self.assertEqual(kwargs["acquisition"]["scheduled_at_utc"], _item().scheduled_at_utc)
        self.assertEqual(kwargs["acquisition"]["selected_at_utc"], _item().selected_at_utc)
        self.assertEqual(kwargs["acquisition"]["device_sweep_index"], 101)
        self.assertEqual(kwargs["acquisition"]["selected_trace_index"], 3)
        self.assertEqual(kwargs["acquisition"]["connection_generation"], 2)
        self.assertEqual(kwargs["acquisition"]["target_interval_s"], 1.0)
        self.assertEqual(kwargs["acquisition"]["actual_interval_s"], 1.01)
        self.assertEqual(kwargs["acquisition"]["schedule_error_ms"], 10.0)
        self.assertEqual(kwargs["acquisition"]["gnss_age_ms"], 2000.0)
        self.assertEqual(
            kwargs["acquisition"]["session_statistics"]["backend"]["completed_sweeps"], 120
        )
        self.assertEqual(
            kwargs["flight"], {"gnss_valid": True, "gnss_altitude_msl_m": 12.5, "flight_height_agl_m": None}
        )

    def test_no_fix_sets_gnss_valid_false_and_empty_altitudes(self) -> None:
        cache = air.GnssFixCache(clock=lambda: 98.0)  # 空缓存，从未 update
        sender = _FakeSender()
        consumer = self._consumer(cache, sender)
        consumer._process_one(_item())
        args, kwargs = sender.calls[0]
        self.assertIsNone(kwargs["acquisition"]["gnss_age_ms"])
        self.assertEqual(
            kwargs["flight"],
            {"gnss_valid": False, "gnss_altitude_msl_m": None, "flight_height_agl_m": None},
        )
        # 不把无效 Fix 当作有效坐标传给保存/UI。
        fix = args[7]
        self.assertEqual(fix.latitude, None)
        self.assertEqual(fix.fix_quality, 0)

    def test_stale_fix_keeps_real_age_and_sets_invalid(self) -> None:
        cache = air.GnssFixCache(clock=lambda: 50.0, max_age_s=5.0)
        self.assertTrue(cache.update(_valid_fix()))
        sender = _FakeSender()
        consumer = self._consumer(cache, sender)
        consumer._process_one(_item())  # 中点 100.0，Fix age = 50s > 5s
        args, kwargs = sender.calls[0]
        self.assertAlmostEqual(kwargs["acquisition"]["gnss_age_ms"], 50000.0)
        self.assertFalse(kwargs["flight"]["gnss_valid"])
        self.assertIsNone(kwargs["flight"]["gnss_altitude_msl_m"])


# ----------------------------------------------------------------------
# ContinuousAcquisitionWorker 端到端
# ----------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class WorkerMetadataEndToEndTests(unittest.TestCase):
    """worker 组装 SelectedSweep 新字段（首末点/中点/计划时刻、generation、会话统计）。"""

    def _run_worker(self, interval_s: float) -> list[air.SelectedSweep]:
        backend = PersistentLibreVnaBackend(
            transport=_RepeatingSweepTransport(5),
            settings=PersistentBackendSettings(),
        )
        out_dir = Path(tempfile.mkdtemp())
        consumer_holder: list[_RecordingQueueConsumer] = []

        def factory(queue, cache):
            consumer = _RecordingQueueConsumer(queue, cache)
            consumer_holder.append(consumer)
            return consumer

        worker = air.ContinuousAcquisitionWorker(
            cfg=_cfg(),
            interval_s=interval_s,
            out_dir=out_dir,
            count=2,
            gnss_port="",
            baud=115200,
            calibration_path="",
            session_calibration=None,
            session_calibration_p2=None,
            through_measurement=None,
            background_frequency_hz=None,
            background=None,
            autosave=False,
            remote_sender=None,
            backend=backend,
            consumer_factory=factory,
        )
        finished = threading.Event()
        worker.finished_run.connect(finished.set, QtCore.Qt.ConnectionType.DirectConnection)
        worker.start()
        self.assertTrue(finished.wait(10.0), "worker 应在 10s 内结束")
        worker.wait(2000)
        return consumer_holder[0].items

    def test_timed_selection_populates_metadata(self) -> None:
        items = self._run_worker(interval_s=0.0)
        self.assertEqual(len(items), 2)
        for item in items:
            self.assertTrue(item.acquired_at_utc)
            self.assertTrue(item.sweep_started_utc)
            self.assertTrue(item.sweep_completed_utc)
            self.assertLessEqual(item.sweep_started_utc, item.acquired_at_utc)
            self.assertLessEqual(item.acquired_at_utc, item.sweep_completed_utc)
            # interval=0 无固定计划时刻。
            self.assertIsNone(item.scheduled_at_utc)
            self.assertEqual(item.target_interval_s, 0.0)
            self.assertEqual(item.connection_generation, 1)
            self.assertIn("backend", item.session_statistics)
            self.assertIn("scheduler", item.session_statistics)
        self.assertEqual([item.trace_index for item in items], [1, 2])
        self.assertEqual([item.device_sweep_index for item in items], [0, 1])

    def test_interval_selection_records_scheduled_deadline(self) -> None:
        items = self._run_worker(interval_s=1.0)
        self.assertEqual(len(items), 2)
        # 第一条立即选择：scheduled_at 约等于选中时刻（非 None）。
        self.assertIsNotNone(items[0].scheduled_at_utc)
        self.assertEqual(items[0].target_interval_s, 1.0)
        self.assertTrue(items[0].session_statistics)

    def test_worker_times_are_true_utc(self) -> None:
        items = self._run_worker(interval_s=1.0)
        self.assertTrue(items)
        for item in items:
            for field in (
                "acquired_at_utc",
                "sweep_started_utc",
                "sweep_completed_utc",
                "selected_at_utc",
            ):
                parsed = datetime.fromisoformat(getattr(item, field))
                self.assertEqual(parsed.utcoffset(), timedelta(0))
            if item.scheduled_at_utc is not None:
                parsed = datetime.fromisoformat(item.scheduled_at_utc)
                self.assertEqual(parsed.utcoffset(), timedelta(0))


# ----------------------------------------------------------------------
# 地面端兼容
# ----------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class GroundStationCompatibilityTests(unittest.TestCase):
    """旧 payload（缺 acquisition/flight）与新增字段均可在 GroundStationWindow 显示。"""

    @classmethod
    def setUpClass(cls) -> None:
        from PySide6 import QtWidgets

        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def _window(self):
        try:
            import uav_gpr_ground_station as gs
        except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
            self.skipTest(f"uav_gpr_ground_station unavailable: {exc}")
        window = gs.GroundStationWindow("127.0.0.1", 0, Path(tempfile.mkdtemp()), start_receiver=False)
        window.receiver.stop()
        return window, gs

    def _header(self, trace_index: int = 1) -> dict[str, object]:
        return {
            "type": "trace",
            "protocol": "uav-gpr-telemetry-v1",
            "run_id": "compat",
            "trace_index": trace_index,
            "timestamp": "2026-08-14T00:00:00",
        }

    def _payload(self, *, with_metadata: bool):
        item = _item(trace_index=1)
        raw, display, time_ns, traces = _dual_payload_parts(item)
        acquisition = _acquisition_expected(item)
        acquisition["gnss_age_ms"] = 2000.0
        flight = {"gnss_valid": True, "gnss_altitude_msl_m": 12.5, "flight_height_agl_m": None}
        if with_metadata:
            return air.trace_npz_bytes(
                1, item.cfg, item.frequency_hz, raw, display, time_ns, traces, _valid_fix(),
                channels=("S11", "S22"), acquisition=acquisition, flight=flight,
            )
        return air.trace_npz_bytes(
            1, item.cfg, item.frequency_hz, raw, display, time_ns, traces, _valid_fix(),
            channels=("S11", "S22"),
        )

    def test_old_payload_still_displays_without_new_fields(self) -> None:
        window, gs = self._window()
        logs: list[str] = []
        try:
            window.log = lambda text: logs.append(text)
            payload = self._payload(with_metadata=False)
            window.on_trace(self._header(), payload, "old_trace.npz")
            self.assertEqual(len(window.bscan_rows["S11"]), 1, "旧 payload 仍应显示 HH")
            self.assertEqual(len(window.bscan_rows["S22"]), 1, "旧 payload 仍应显示 VV")
            self.assertFalse(any("Trace已忽略" in text for text in logs), f"不应忽略: {logs}")
            self.assertEqual(window.acquisition_label.text(), "采集：-")
        finally:
            _shutdown_window(window)

    def test_new_payload_displays_metadata_fields(self) -> None:
        window, gs = self._window()
        logs: list[str] = []
        try:
            window.log = lambda text: logs.append(text)
            payload = self._payload(with_metadata=True)
            window.on_trace(self._header(), payload, "new_trace.npz")
            self.assertEqual(len(window.bscan_rows["S11"]), 1)
            self.assertFalse(any("Trace已忽略" in text for text in logs), f"不应忽略: {logs}")
            label = window.acquisition_label.text()
            self.assertIn("采集时间", label)
            self.assertIn("设备Sweep编号：99", label)
            self.assertIn("连接代数：2", label)
            self.assertIn("GNSS时延：2000 ms", label)
            self.assertIn("GNSS有效", label)
            self.assertIn("GNSS海拔：12.5 m", label)
            self.assertIn("对地高度：未知", label)
        finally:
            _shutdown_window(window)


# ----------------------------------------------------------------------
# UTC 语义
# ----------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class UtcSemanticsTests(unittest.TestCase):
    """新 acquisition 的 ``*_utc`` 字段必须为真 UTC（+00:00），不能是本地偏移。"""

    @staticmethod
    def _assert_true_utc(iso: str) -> None:
        parsed = datetime.fromisoformat(iso)
        assert parsed.utcoffset() == timedelta(0)

    def test_utc_helpers_produce_true_utc(self) -> None:
        self._assert_true_utc(air.utc_iso_now())
        self._assert_true_utc(air.utc_iso_from_epoch(1_750_000_000.5))

    def test_save_trace_acquisition_fields_are_true_utc(self) -> None:
        item = _item()
        raw, display, time_ns, traces = _dual_payload_parts(item)
        acquisition = _acquisition_expected(item)
        acquisition["gnss_age_ms"] = 2000.0
        flight = {"gnss_valid": True, "gnss_altitude_msl_m": 12.5, "flight_height_agl_m": None}
        out_dir = Path(tempfile.mkdtemp())
        csv_path = air.save_trace(
            out_dir, item.trace_index, item.cfg, item.frequency_hz,
            raw, display, time_ns, traces, _valid_fix(),
            channels=("S11", "S22"), acquisition=acquisition, flight=flight,
        )
        with np.load(csv_path.with_suffix(".npz"), allow_pickle=False) as data:
            acq = json.loads(str(data["acquisition"].item()))
        for field in (
            "acquired_at_utc",
            "sweep_started_utc",
            "sweep_completed_utc",
            "scheduled_at_utc",
            "selected_at_utc",
            "saved_at_utc",
        ):
            self._assert_true_utc(acq[field])

    def test_remote_header_acquisition_times_are_true_utc(self) -> None:
        item = _item()
        raw, display, time_ns, traces = _dual_payload_parts(item)
        acquisition = _acquisition_expected(item)
        acquisition["gnss_age_ms"] = 2000.0
        flight = {"gnss_valid": True, "gnss_altitude_msl_m": 12.5, "flight_height_agl_m": None}
        sender = air.RemoteSender("127.0.0.1", 0, "utc-test", status_callback=lambda _s: None)
        sender.enqueue_trace(
            item.trace_index, item.cfg, item.frequency_hz, raw, display, time_ns, traces,
            _valid_fix(), channels=("S11", "S22"), acquisition=acquisition, flight=flight,
        )
        header, _payload = sender.queue.get_nowait()
        for field in ("acquired_at_utc", "saved_at_utc"):
            self._assert_true_utc(header["acquisition"][field])
        self._assert_true_utc(header["acquired_at_utc"])


# ----------------------------------------------------------------------
# channels/header 一致性校验
# ----------------------------------------------------------------------


def _npz_with_channel_declaration(channels_value: object) -> bytes:
    """构造带显式 channels 声明的双通道 NPZ（供一致性/冲突测试）。"""
    item = _item()
    _raw, _display, time_ns, traces = _dual_payload_parts(item)
    buffer = io.BytesIO()
    np.savez_compressed(
        buffer,
        index=np.array(1, dtype=np.int64),
        frequency_hz=item.frequency_hz,
        time_ns=time_ns,
        channels=json.dumps(channels_value, ensure_ascii=False),
        trace_S11=traces["S11"],
        trace_S22=traces["S22"],
    )
    return buffer.getvalue()


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class ChannelsConsistencyTests(unittest.TestCase):
    """新格式 channels 声明冲突/损坏必须 fail-closed 原子拒绝，绝不静默回退 legacy。"""

    @classmethod
    def setUpClass(cls) -> None:
        from PySide6 import QtWidgets

        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def _window(self):
        try:
            import uav_gpr_ground_station as gs
        except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
            self.skipTest(f"uav_gpr_ground_station unavailable: {exc}")
        window = gs.GroundStationWindow("127.0.0.1", 0, Path(tempfile.mkdtemp()), start_receiver=False)
        window.receiver.stop()
        return window

    def _run(self, payload: bytes, header_channels: object = None):
        window = self._window()
        logs: list[str] = []
        header: dict[str, object] = {
            "type": "trace",
            "protocol": "uav-gpr-telemetry-v1",
            "run_id": "channels",
            "trace_index": 1,
            "timestamp": "2026-08-14T00:00:00",
        }
        if header_channels is not None:
            header["channels"] = header_channels
        try:
            window.log = lambda text: logs.append(text)
            window.on_trace(header, payload, "conflict.npz")
        finally:
            _shutdown_window(window)
        return window, logs

    def test_npz_header_channels_conflict_rejected_atomically(self) -> None:
        payload = _npz_with_channel_declaration(["S11", "S22"])
        window, logs = self._run(payload, header_channels=["S11"])
        self.assertTrue(any("Trace已忽略" in text for text in logs), f"应拒绝: {logs}")
        self.assertEqual(len(window.bscan_rows["S11"]), 0, "冲突不得修改 B-scan 历史")
        self.assertEqual(len(window.bscan_rows["S22"]), 0)

    def test_duplicate_channels_rejected(self) -> None:
        payload = _npz_with_channel_declaration(["S11", "S11"])
        window, logs = self._run(payload)
        self.assertTrue(any("Trace已忽略" in text for text in logs))
        self.assertEqual(len(window.bscan_rows["S11"]), 0)

    def test_invalid_channels_type_rejected(self) -> None:
        # channels 声明不是字符串列表（此处为单个字符串），解析失败。
        payload = _npz_with_channel_declaration("S11")
        window, logs = self._run(payload)
        self.assertTrue(any("Trace已忽略" in text for text in logs))
        self.assertEqual(len(window.bscan_rows["S11"]), 0)

    def test_declared_channels_mismatch_actual_fields_rejected(self) -> None:
        # 声明只有 S11，但 payload 实际含 trace_S11+trace_S22。
        payload = _npz_with_channel_declaration(["S11"])
        window, logs = self._run(payload)
        self.assertTrue(any("Trace已忽略" in text for text in logs))
        self.assertEqual(len(window.bscan_rows["S11"]), 0)
        self.assertEqual(len(window.bscan_rows["S22"]), 0)

    def test_valid_dual_channel_payload_still_accepted(self) -> None:
        item = _item()
        raw, display, time_ns, traces = _dual_payload_parts(item)
        payload = air.trace_npz_bytes(
            1, item.cfg, item.frequency_hz, raw, display, time_ns, traces,
            _valid_fix(), channels=("S11", "S22"),
        )
        window, logs = self._run(payload, header_channels=["S11", "S22"])
        self.assertFalse(any("Trace已忽略" in text for text in logs), f"不应拒绝: {logs}")
        self.assertEqual(len(window.bscan_rows["S11"]), 1)
        self.assertEqual(len(window.bscan_rows["S22"]), 1)


# ----------------------------------------------------------------------
# 真实旧 NPZ legacy 兼容
# ----------------------------------------------------------------------


_MISSING = object()


def _legacy_old_npz(
    *,
    channels_value: object = _MISSING,
    time_ns: object = _MISSING,
    trace: object = _MISSING,
    display_names: tuple[str, ...] = ("S11", "S21", "S12", "S22"),
    half_display: tuple[str, ...] = (),
) -> bytes:
    """合成复现真实旧 schema：trace + display_Sxx，无 processing / trace_Sxx。

    默认不含 channels key；传 ``channels_value`` 时写入 JSON 编码的 channels 声明。
    ``half_display`` 指定只写 ``display_{name}_real``、不写 imag 的通道（制造半份
    数据）。``time_ns``/``trace`` 可覆盖以构造非法时间轴/长度。
    """
    freq = np.array([400e6, 1300e6, 2200e6, 3100e6, 4000e6], dtype=float)
    default_time = np.arange(8, dtype=float) * 0.5
    time_values = default_time if time_ns is _MISSING else time_ns
    trace_values = np.sin(default_time) if trace is _MISSING else trace
    buffer = io.BytesIO()
    payload: dict[str, object] = {
        "config": json.dumps(
            {"start_mhz": 400.0, "stop_mhz": 4000.0, "points": 5}, ensure_ascii=False
        ),
        "frequency_hz": freq,
        "time_ns": time_values,
        "trace": trace_values,
        "gnss": json.dumps(
            {"raw": "", "utc": "", "latitude": None, "longitude": None,
             "fix_quality": 0, "satellites": 0, "altitude_m": None},
            ensure_ascii=False,
        ),
    }
    if channels_value is not _MISSING:
        payload["channels"] = json.dumps(channels_value, ensure_ascii=False)
    for name in display_names:
        value = complex(0.5, 0.1)
        payload[f"raw_{name}_real"] = np.full(freq.size, value.real, dtype=float)
        payload[f"raw_{name}_imag"] = np.full(freq.size, value.imag, dtype=float)
        payload[f"display_{name}_real"] = np.full(freq.size, value.real, dtype=float)
        if name not in half_display:
            payload[f"display_{name}_imag"] = np.full(freq.size, value.imag, dtype=float)
    np.savez_compressed(buffer, **payload)
    return buffer.getvalue()


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class LegacyOldNpzTests(unittest.TestCase):
    """真实旧格式（无 channels/trace_Sxx，仅 trace）按 legacy trace-only 接受；显式损坏原子拒绝。"""

    @classmethod
    def setUpClass(cls) -> None:
        from PySide6 import QtWidgets

        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def _window(self):
        try:
            import uav_gpr_ground_station as gs
        except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
            self.skipTest(f"uav_gpr_ground_station unavailable: {exc}")
        window = gs.GroundStationWindow("127.0.0.1", 0, Path(tempfile.mkdtemp()), start_receiver=False)
        window.receiver.stop()
        return window

    def _cleanup(self, window) -> None:
        _shutdown_window(window)

    def _header(self, trace_index: int = 1, channels: object = _MISSING) -> dict[str, object]:
        header: dict[str, object] = {
            "type": "trace",
            "protocol": "uav-gpr-telemetry-v1",
            "run_id": "legacy",
            "trace_index": trace_index,
            "timestamp": "2026-08-14T00:00:00",
        }
        if channels is not _MISSING:
            header["channels"] = channels
        return header

    def _dual_state(self, window) -> None:
        """向窗口加载合法 S11/S22 新 payload，建立双通道显示状态。"""
        item = _item()
        raw, display, time_ns, traces = _dual_payload_parts(item)
        payload = air.trace_npz_bytes(
            1, item.cfg, item.frequency_hz, raw, display, time_ns, traces,
            _valid_fix(), channels=("S11", "S22"),
        )
        window.on_trace(self._header(), payload, "dual.npz")

    def _state_snapshot(self, window) -> dict[str, object]:
        return {
            "bscan_s11": [np.asarray(r).copy() for r in window.bscan_rows["S11"]],
            "bscan_s22": [np.asarray(r).copy() for r in window.bscan_rows["S22"]],
            "bscan_time_ns": (
                None if window.bscan_time_ns is None else np.asarray(window.bscan_time_ns).copy()
            ),
            "bscan_sig": window.bscan_processing_signature,
            "bscan_window_start": window.bscan_window_start,
            "acq_label": window.acquisition_label.text(),
            "legacy_status": window.legacy_status.text(),
            "dewow_status": window.dewow_status.text(),
            "trace_status": window.trace_status_label.text(),
        }

    def _assert_state_equal(self, window, before: dict[str, object]) -> None:
        after = self._state_snapshot(window)
        for key in before:
            b = before[key]
            a = after[key]
            if isinstance(b, np.ndarray):
                self.assertIsNotNone(a, f"{key} 不应被清空")
                np.testing.assert_array_equal(a, b, err_msg=f"{key} 应保持不变")
            elif isinstance(b, list):
                self.assertEqual(len(a), len(b), f"{key} 数量不应变化")
                for ba, bb in zip(a, b):
                    np.testing.assert_array_equal(
                        np.asarray(ba), np.asarray(bb), err_msg=f"{key} 内容不应变化"
                    )
            else:
                self.assertEqual(a, b, f"{key} 不应变化")

    def _assert_rejected(
        self,
        window,
        logs: list[str],
        payload: bytes,
        header: dict[str, object],
    ) -> None:
        before = self._state_snapshot(window)
        window.log = lambda text: logs.append(text)
        window.on_trace(header, payload, "bad_legacy.npz")
        self.assertTrue(any("Trace已忽略" in text for text in logs), f"应拒绝: {logs}")
        self._assert_state_equal(window, before)

    # ---- 接受路径 ----

    def test_legacy_trace_only_accepted_without_faking_s22(self) -> None:
        window = self._window()
        logs: list[str] = []
        try:
            window.log = lambda text: logs.append(text)
            window.on_trace(self._header(), _legacy_old_npz(), "legacy_trace.npz")
            self.assertFalse(any("Trace已忽略" in text for text in logs), f"不得忽略: {logs}")
            # 双通道 B-scan 历史不被破坏（保持为空），legacy 不伪造 S22。
            self.assertEqual(len(window.bscan_rows["S11"]), 0)
            self.assertEqual(len(window.bscan_rows["S22"]), 0)
            # 详细状态明确 legacy/S22 unavailable 警告（中文）。
            self.assertIn("旧格式NPZ仅包含S11", window.legacy_status.text())
            self.assertIn("没有S22", window.legacy_status.text())
            self.assertIn("未加入双通道B-scan", window.legacy_status.text())
            # 缺新增 metadata → 采集：-。
            self.assertEqual(window.acquisition_label.text(), "采集：-")
            # 合法 display_S11/S22 通过校验但不创建任何图形状态（无 sparam/ascan 曲线）。
            self.assertFalse(hasattr(window, "sparam_curves"))
            self.assertFalse(hasattr(window, "ascan_curves"))
        finally:
            self._cleanup(window)

    # ---- 显式 channels 损坏拒绝（不得静默回退 legacy）----

    def test_trace_only_with_header_channels_declaration_rejected(self) -> None:
        window = self._window()
        logs: list[str] = []
        try:
            self._dual_state(window)
            self._assert_rejected(
                window,
                logs,
                _legacy_old_npz(),
                self._header(channels=["S11", "S21", "S12", "S22"]),
            )
        finally:
            self._cleanup(window)

    def test_trace_only_with_npz_channels_declaration_rejected(self) -> None:
        window = self._window()
        logs: list[str] = []
        try:
            self._dual_state(window)
            self._assert_rejected(
                window,
                logs,
                _legacy_old_npz(channels_value=["S11", "S21", "S12", "S22"]),
                self._header(),
            )
        finally:
            self._cleanup(window)

    def test_channels_none_rejected(self) -> None:
        window = self._window()
        logs: list[str] = []
        try:
            self._dual_state(window)
            self._assert_rejected(window, logs, _legacy_old_npz(channels_value=None), self._header())
        finally:
            self._cleanup(window)

    def test_channels_empty_string_rejected(self) -> None:
        window = self._window()
        logs: list[str] = []
        try:
            self._dual_state(window)
            self._assert_rejected(window, logs, _legacy_old_npz(channels_value=""), self._header())
        finally:
            self._cleanup(window)

    def test_channels_empty_list_rejected(self) -> None:
        window = self._window()
        logs: list[str] = []
        try:
            self._dual_state(window)
            self._assert_rejected(window, logs, _legacy_old_npz(channels_value=[]), self._header())
        finally:
            self._cleanup(window)

    # ---- legacy time_ns / trace 严格验证 ----

    def test_bad_time_or_trace_rejected_atomically(self) -> None:
        bad_time_nan = np.array([0.0, np.nan, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5])
        bad_time_inf = np.array([0.0, np.inf, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5])
        bad_trace_nan = np.array([0.0, np.nan, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5])
        cases: list[tuple[str, object, object]] = [
            ("time_ns NaN", bad_time_nan, _MISSING),
            ("time_ns Inf", bad_time_inf, _MISSING),
            ("time_ns 2D", np.zeros((2, 4)), _MISSING),
            ("time_ns empty", np.zeros((0,)), _MISSING),
            ("trace length mismatch", _MISSING, np.zeros(10)),
            ("trace NaN", _MISSING, bad_trace_nan),
        ]
        for label, bad_time, bad_trace in cases:
            with self.subTest(label=label):
                window = self._window()
                logs: list[str] = []
                try:
                    self._dual_state(window)
                    self._assert_rejected(
                        window,
                        logs,
                        _legacy_old_npz(
                            time_ns=_MISSING if bad_time is _MISSING else bad_time,
                            trace=_MISSING if bad_trace is _MISSING else bad_trace,
                        ),
                        self._header(),
                    )
                finally:
                    self._cleanup(window)

    # ---- display 半份 / 缺失 ----

    def test_legacy_half_display_rejected(self) -> None:
        window = self._window()
        logs: list[str] = []
        try:
            self._dual_state(window)
            self._assert_rejected(
                window, logs, _legacy_old_npz(half_display=("S11",)), self._header()
            )
        finally:
            self._cleanup(window)

    def test_legacy_missing_display_accepted_no_stale_display_state(self) -> None:
        window = self._window()
        logs: list[str] = []
        try:
            window.log = lambda text: logs.append(text)
            self._dual_state(window)
            before_s11 = len(window.bscan_rows["S11"])
            before_s22 = len(window.bscan_rows["S22"])
            # legacy 文件只带 display_S11，无 display_S22 → 允许；不会产生陈旧
            # S-parameter/A-scan 显示状态（该功能已移除）。
            window.on_trace(
                self._header(), _legacy_old_npz(display_names=("S11",)), "legacy.npz"
            )
            self.assertFalse(any("Trace已忽略" in text for text in logs), f"不得忽略: {logs}")
            self.assertIn("旧格式NPZ仅包含S11", window.legacy_status.text())
            # B-scan 历史保持不变（legacy 不追加双通道）。
            self.assertEqual(len(window.bscan_rows["S11"]), before_s11)
            self.assertEqual(len(window.bscan_rows["S22"]), before_s22)
            # 不再存在 sparam/ascan 显示对象。
            self.assertFalse(hasattr(window, "sparam_curves"))
            self.assertFalse(hasattr(window, "ascan_curves"))
        finally:
            self._cleanup(window)

    # ---- 状态切换：双通道 → legacy ----

    def test_switch_from_dual_channel_to_legacy_keeps_bscan_history(self) -> None:
        window = self._window()
        logs: list[str] = []
        try:
            window.log = lambda text: logs.append(text)
            self._dual_state(window)
            self.assertEqual(len(window.bscan_rows["S11"]), 1)
            self.assertEqual(len(window.bscan_rows["S22"]), 1)
            before_s11 = np.asarray(window.bscan_rows["S11"][0]).copy()
            before_s22 = np.asarray(window.bscan_rows["S22"][0]).copy()
            before_bscan_time = np.asarray(window.bscan_time_ns).copy()

            window.on_trace(self._header(), _legacy_old_npz(), "legacy.npz")
            self.assertFalse(any("Trace已忽略" in text for text in logs), f"不应忽略: {logs}")
            # HH/VV B-scan 历史数量与内容保持原状（legacy 不追加、不清空双通道）。
            self.assertEqual(len(window.bscan_rows["S11"]), 1)
            self.assertEqual(len(window.bscan_rows["S22"]), 1)
            np.testing.assert_array_equal(window.bscan_rows["S11"][0], before_s11)
            np.testing.assert_array_equal(window.bscan_rows["S22"][0], before_s22)
            np.testing.assert_array_equal(window.bscan_time_ns, before_bscan_time)
            # legacy 状态文字正确（中文）。
            self.assertIn("旧格式NPZ仅包含S11", window.legacy_status.text())
            self.assertIn("没有S22", window.legacy_status.text())
            self.assertIn("未加入双通道B-scan", window.legacy_status.text())
            self.assertEqual(window.acquisition_label.text(), "采集：-")
        finally:
            self._cleanup(window)


# ----------------------------------------------------------------------
# 会话完整性统计显示
# ----------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class SessionStatsDisplayTests(unittest.TestCase):
    """地面端显示精简完整性统计；缺失/非法数值绝不显示成可信统计。"""

    @classmethod
    def setUpClass(cls) -> None:
        from PySide6 import QtWidgets

        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def _show(self, session_statistics: dict[str, object]) -> str:
        try:
            import uav_gpr_ground_station as gs
        except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
            self.skipTest(f"uav_gpr_ground_station unavailable: {exc}")
        window = gs.GroundStationWindow("127.0.0.1", 0, Path(tempfile.mkdtemp()), start_receiver=False)
        window.receiver.stop()
        item = _item()
        raw, display, time_ns, traces = _dual_payload_parts(item)
        acquisition = _acquisition_expected(item)
        acquisition["gnss_age_ms"] = 2000.0
        acquisition["session_statistics"] = session_statistics
        flight = {"gnss_valid": True, "gnss_altitude_msl_m": 12.5, "flight_height_agl_m": None}
        payload = air.trace_npz_bytes(
            1, item.cfg, item.frequency_hz, raw, display, time_ns, traces,
            _valid_fix(), channels=("S11", "S22"),
            acquisition=acquisition, flight=flight,
        )
        header: dict[str, object] = {
            "type": "trace",
            "protocol": "uav-gpr-telemetry-v1",
            "run_id": "stats",
            "trace_index": 1,
            "timestamp": "2026-08-14T00:00:00",
        }
        try:
            window.on_trace(header, payload, "stats.npz")
            label = window.acquisition_label.text()
        finally:
            _shutdown_window(window)
        return label

    def test_integrity_stats_shown_when_present(self) -> None:
        label = self._show(
            {
                "mode": "continuous",
                "backend": {
                    "completed_sweeps": 120,
                    "dropped_sweeps": 3,
                    "incomplete_sweeps": 1,
                    "sweep_timeouts": 2,
                    "pending_overflows": 0,
                },
                "processing_queue": {"overflows": 4, "dropped": 4},
            }
        )
        self.assertIn("完整Sweep=120", label)
        self.assertIn("丢弃Sweep=3", label)
        self.assertIn("不完整Sweep=1", label)
        self.assertIn("Sweep超时=2", label)
        self.assertIn("Pending溢出=0", label)
        self.assertIn("队列溢出=4", label)
        self.assertIn("队列丢弃=4", label)
        # 显示的是中文标签，不再暴露英文内部 key。
        self.assertNotIn("completed_sweeps", label)
        self.assertNotIn("queue_overflows", label)

    def test_invalid_stats_not_shown_as_trusted(self) -> None:
        label = self._show(
            {
                "mode": "continuous",
                "backend": {
                    "completed_sweeps": 120,
                    "dropped_sweeps": float("nan"),
                    "incomplete_sweeps": float("inf"),
                },
                "processing_queue": {"overflows": "many", "dropped": None},
            }
        )
        self.assertIn("完整Sweep=120", label)
        self.assertNotIn("丢弃Sweep", label)
        self.assertNotIn("不完整Sweep", label)
        self.assertNotIn("队列溢出", label)
        self.assertNotIn("队列丢弃", label)

    def test_missing_session_stats_still_shows_other_fields(self) -> None:
        label = self._show({})
        self.assertIn("采集时间", label)
        self.assertIn("对地高度：未知", label)
        self.assertNotIn("完整Sweep", label)


# ----------------------------------------------------------------------
# receiver 透明透传
# ----------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class ReceiverPassthroughTests(unittest.TestCase):
    """uav_gpr_receiver 不改：新 header 字段原样落盘、payload 字节不变、framing/ACK 不变。"""

    def _trace(self):
        item = _item()
        raw, display, time_ns, traces = _dual_payload_parts(item)
        acquisition = _acquisition_expected(item)
        acquisition["gnss_age_ms"] = 2000.0
        flight = {"gnss_valid": True, "gnss_altitude_msl_m": 12.5, "flight_height_agl_m": None}
        sender = air.RemoteSender(
            "127.0.0.1", 0, "receiver-test", status_callback=lambda _s: None
        )
        sender.enqueue_trace(
            item.trace_index, item.cfg, item.frequency_hz, raw, display, time_ns, traces,
            _valid_fix(), channels=("S11", "S22"), acquisition=acquisition, flight=flight,
        )
        return sender.queue.get_nowait()

    def test_receiver_save_trace_preserves_header_and_payload(self) -> None:
        try:
            import uav_gpr_receiver as receiver_mod
        except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
            self.skipTest(f"uav_gpr_receiver unavailable: {exc}")
        header, payload = self._trace()
        out_dir = Path(tempfile.mkdtemp())
        receiver = receiver_mod.TraceReceiver("127.0.0.1", 0, out_dir)
        path = receiver.save_trace(header, payload)
        self.assertEqual(path.read_bytes(), payload, "NPZ payload 字节必须保持不变")
        line = (path.parent / "run_metadata.jsonl").read_text(encoding="utf-8").splitlines()[0]
        meta = json.loads(line)
        self.assertEqual(meta["acquired_at_utc"], header["acquired_at_utc"])
        self.assertEqual(meta["acquisition"], header["acquisition"])
        self.assertEqual(meta["flight"], header["flight"])
        self.assertEqual(meta["receiver_sha256"], header["sha256"])

    def test_receiver_full_round_trip_framing_and_ack(self) -> None:
        try:
            import uav_gpr_receiver as receiver_mod
        except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
            self.skipTest(f"uav_gpr_receiver unavailable: {exc}")
        header, payload = self._trace()
        out_dir = Path(tempfile.mkdtemp())
        receiver = receiver_mod.TraceReceiver("127.0.0.1", 0, out_dir)
        server_sock, client_sock = socket.socketpair()
        errors: list[Exception] = []

        def serve() -> None:
            try:
                receiver.handle_client(server_sock)
            except Exception as exc:  # 连接关闭属预期
                errors.append(exc)

        thread = threading.Thread(target=serve, daemon=True)
        thread.start()
        try:
            receiver_mod.send_frame(
                client_sock,
                {
                    "type": "hello",
                    "protocol": "uav-gpr-telemetry-v1",
                    "run_id": "receiver-test",
                    "timestamp": "2026-08-14T00:00:00",
                },
            )
            ack, _ = receiver_mod.recv_frame(client_sock)
            self.assertEqual(ack["type"], "ack")
            receiver_mod.send_frame(client_sock, header, payload)
            ack_trace, _ = receiver_mod.recv_frame(client_sock)
            self.assertEqual(ack_trace["type"], "ack_trace")
            self.assertEqual(ack_trace["protocol"], "uav-gpr-telemetry-v1")
            self.assertEqual(ack_trace["trace_index"], header["trace_index"])
            saved = (
                out_dir
                / "receiver-test"
                / f"trace_{int(header['trace_index']):05d}.npz"
            )
            self.assertTrue(saved.exists())
            self.assertEqual(saved.read_bytes(), payload)
        finally:
            client_sock.close()
            server_sock.close()
            thread.join(5.0)


# ----------------------------------------------------------------------
# ReceiverThread 生命周期竞态回归（修正轮 13 独立任务）
# ----------------------------------------------------------------------

#: 记录本类创建过的 ReceiverThread（弱引用），供残留检查。
_LIFECYCLE_RECEIVERS: "weakref.WeakSet" = weakref.WeakSet()


class _StubReceiver(QtCore.QThread):
    """可编程 wait 结果的接收线程替身（closeEvent fail-closed 分支测试用）。"""

    status = QtCore.Signal(str)
    trace_received = QtCore.Signal(dict, bytes, str)
    command_status = QtCore.Signal(str)

    def __init__(self, wait_result: bool = True) -> None:
        super().__init__()
        self._wait_result = wait_result
        self.stop_calls = 0

    def start(self) -> None:
        pass  # 不启动真实线程

    def stop(self) -> None:
        self.stop_calls += 1

    def wait(self, ms: int) -> bool:
        return self._wait_result

    def isRunning(self) -> bool:
        return not self._wait_result


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class ReceiverThreadLifecycleTests(unittest.TestCase):
    """GroundStation ReceiverThread 生命周期竞态回归（stop 唤醒 accept/recv、closeEvent
    fail-closed、重复 stop 幂等、无残留）。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def _make_receiver(self) -> "object":
        try:
            import uav_gpr_ground_station as gs
        except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
            self.skipTest(f"uav_gpr_ground_station unavailable: {exc}")
        receiver = gs.ReceiverThread("127.0.0.1", 0, Path(tempfile.mkdtemp()))
        _LIFECYCLE_RECEIVERS.add(receiver)
        # 创建后立即注册清理：任何前置断言/异常都会让 addCleanup 执行 stop+wait。
        self.addCleanup(self._cleanup_receiver, receiver)
        return receiver

    def _make_client_socket(self, server_addr) -> "socket.socket":
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5.0)
        # socket 创建后立即注册关闭（connect/accept-client 失败也执行）。
        self.addCleanup(self._cleanup_socket, client)
        client.connect(server_addr)
        return client

    def _cleanup_receiver(self, receiver) -> None:
        """统一清理：stop -> 有界 wait -> 断言已退出。幂等（视 stop 是否已调用）。"""
        receiver.stop()
        if not receiver.wait(3000):
            self.fail("receiver 未在 3s 内退出（wait 返回 False）")
        if receiver.isRunning():
            self.fail("receiver 仍运行（QThread 残留）")

    def _cleanup_socket(self, sock: "socket.socket") -> None:
        try:
            sock.close()
        except OSError:
            pass

    def _cleanup_window(self, window) -> None:
        window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        self.app.processEvents()

    def _wait_listening(self, receiver, timeout_s: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if receiver._server is not None:
                return True
            time.sleep(0.01)
        return False

    def test_stop_before_run(self) -> None:
        receiver = self._make_receiver()  # addCleanup 已注册
        receiver.stop()  # stop 发生在线程真正进入 run() 之前
        receiver.start()
        self.assertTrue(receiver.wait(3000), "stop 后线程应在 3s 内退出")
        self.assertFalse(receiver.isRunning())

    def test_stop_during_accept(self) -> None:
        receiver = self._make_receiver()
        receiver.start()
        self.assertTrue(self._wait_listening(receiver), "线程未进入 accept 阻塞")
        receiver.stop()  # accept 阻塞期间 stop 应唤醒
        self.assertTrue(receiver.wait(3000), "accept 阻塞期间 stop 应唤醒线程")
        self.assertFalse(receiver.isRunning())

    def test_stop_during_client_recv(self) -> None:
        receiver = self._make_receiver()
        receiver.start()
        self.assertTrue(self._wait_listening(receiver))
        server_addr = receiver._server.getsockname()
        # client 创建后立即注册关闭；connect/前置断言失败也会执行 addCleanup(_cleanup_socket)。
        client = self._make_client_socket(server_addr)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if receiver._client is not None:
                break
            time.sleep(0.01)
        self.assertIsNotNone(receiver._client, "client 应被 accept")
        # 不发送数据：recv 阻塞；stop 应关闭 client 唤醒 recv。
        receiver.stop()
        self.assertTrue(receiver.wait(3000), "client recv 阻塞期间 stop 应唤醒线程")
        self.assertFalse(receiver.isRunning())
        # 清理由 addCleanup 完成（_cleanup_socket(client) + _cleanup_receiver(receiver)）。

    def test_repeated_stop_idempotent(self) -> None:
        receiver = self._make_receiver()
        receiver.start()
        self.assertTrue(self._wait_listening(receiver))
        receiver.stop()
        receiver.stop()
        receiver.stop()  # 重复 stop 不抛异常
        self.assertTrue(receiver.wait(3000))
        self.assertFalse(receiver.isRunning())

    def _stub_window(self, wait_result: bool) -> "object":
        import uav_gpr_ground_station as gs
        stub = _StubReceiver(wait_result=wait_result)
        window = gs.GroundStationWindow(
            "127.0.0.1", 0, Path(tempfile.mkdtemp()), receiver=stub, start_receiver=False
        )
        self.addCleanup(self._cleanup_window, window)
        return window

    def test_close_event_ignores_when_receiver_not_exited(self) -> None:
        window = self._stub_window(wait_result=False)  # addCleanup 清理 window
        event = QtGui.QCloseEvent()
        window.closeEvent(event)
        self.assertFalse(event.isAccepted(), "receiver 未退出时 closeEvent 必须 ignore")
        self.assertEqual(window.receiver.stop_calls, 1)

    def test_close_event_accepts_after_receiver_exits(self) -> None:
        window = self._stub_window(wait_result=True)
        event = QtGui.QCloseEvent()
        window.closeEvent(event)
        self.assertTrue(event.isAccepted(), "receiver 已退出时 closeEvent 应接受关闭")

    def test_second_close_event_accepts_after_exit(self) -> None:
        window = self._stub_window(wait_result=False)
        event1 = QtGui.QCloseEvent()
        window.closeEvent(event1)
        self.assertFalse(event1.isAccepted(), "第一次关闭应被 ignore")
        window.receiver._wait_result = True  # 线程最终退出
        event2 = QtGui.QCloseEvent()
        window.closeEvent(event2)
        self.assertTrue(event2.isAccepted(), "第二次关闭应接受")

    def test_failure_path_cleanup_runs(self) -> None:
        """嵌套 TestCase/TestResult 回归：intentional failure 后真实 addCleanup 调度
        `_cleanup_receiver`（而非外层手动调用）。

        内部 case 在等待 listening 前自报失败；unittest 会在失败后执行其 addCleanup，
        断言 receiver 被 stop/wait 且 isRunning() 为 False。
        """
        import uav_gpr_ground_station as gs
        receiver = gs.ReceiverThread("127.0.0.1", 0, Path(tempfile.mkdtemp()))
        _LIFECYCLE_RECEIVERS.add(receiver)
        result = {}

        class _Inner(unittest.TestCase):
            def runTest(self) -> None:
                # 在内部 case 上注册真实 addCleanup（不是外层手动调用）。
                self.addCleanup(self._do_cleanup, receiver)
                receiver.start()
                # 前置失败：不等 listening 即触发 intentional failure；
                # addCleanup 必须在失败后仍执行。
                self.fail("intentional failure")

            def _do_cleanup(self, recv) -> None:
                recv.stop()
                result["stop_called"] = True
                result["wait_ok"] = bool(recv.wait(3000))
                result["no_longer_running"] = not recv.isRunning()

        suite = unittest.defaultTestLoader.loadTestsFromTestCase(_Inner)
        test_result = unittest.TestResult()
        suite.run(test_result)
        # intentional failure 必须被嵌套记录一次，不污染外层套件。
        self.assertEqual(len(test_result.failures), 1, "应恰好记录一个 expected failure")
        self.assertEqual(len(test_result.errors), 0)
        self.assertTrue(result.get("stop_called"), "addCleanup 必须调用 stop")
        self.assertTrue(result.get("wait_ok"), "addCleanup 的 wait 必须返回 True")
        self.assertTrue(result.get("no_longer_running"), "cleaned receiver 不得仍运行")
        self.assertFalse(receiver.isRunning())

    def test_no_residual_after_early_failure(self) -> None:
        """嵌套 TestCase/TestResult 回归：intentional exception 后 client socket 因
        addCleanup(_cleanup_socket) 被关闭（而非外层手动调用）。"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        result = {}

        class _Inner(unittest.TestCase):
            def runTest(self) -> None:
                # socket 创建后立即在内部 case 注册 addCleanup。
                self.addCleanup(self._do_cleanup, sock)
                # connect 前主动抛异常；addCleanup 必须在异常传播后仍执行。
                raise RuntimeError("intentional error")

            def _do_cleanup(self, s) -> None:
                try:
                    s.close()
                except OSError:
                    pass

        suite = unittest.defaultTestLoader.loadTestsFromTestCase(_Inner)
        test_result = unittest.TestResult()
        suite.run(test_result)
        # 恰好记录一个 expected error（异常被内部嵌套捕获，不污染外层）。
        self.assertEqual(len(test_result.errors), 1, "应恰好记录一个 expected error")
        self.assertEqual(len(test_result.failures), 0)
        self.assertEqual(sock.fileno(), -1, "addCleanup 必须关闭 socket")
        sock.close()  # 防御性：若未关闭则在此兜底，避免 fd 泄漏

    def test_zzz_no_residual_running_receivers(self) -> None:
        for receiver in list(_LIFECYCLE_RECEIVERS):
            if receiver is not None:
                self.assertFalse(receiver.isRunning(), "残留 ReceiverThread 仍在运行")


if __name__ == "__main__":
    unittest.main()
