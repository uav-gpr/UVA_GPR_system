"""阶段 3 连续采集组件测试：GNSS 缓存、有界队列、线程生命周期、慢消费者。

覆盖 roadmap §15.4（GNSS 最近 Fix / 过期 Fix / 无 Fix）与 §15.5（start 后立即
stop、重复启停、pause/resume 无残留线程、慢消费者不阻塞 USB、closeEvent 等待
线程退出）。全部使用 FakeTransport，无真实硬件 / 串口 / 网络。

验收点（roadmap §16 阶段 3）：
- USB worker 无保存间隔 sleep / GNSS 阻塞 / NPZ 压缩 / 网络等待（通过结构断言
  与行为测试验证）；
- 慢消费者不阻止 USB 持续排流；
- start 后立即 stop、重复启停、pause/resume 均无残留线程；
- feature flag **默认 continuous**（第4轮采集速度修复：真机 legacy ≈2.75 s/帧），
  legacy 需显式选择（env=legacy 或界面“采集引擎”下拉）。
"""

from __future__ import annotations

import io
import json
import os
import struct
import tempfile
import threading
import time
import unittest
import unittest.mock
from pathlib import Path

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    import air_core as air
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover - environment dependent
    air = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

from PySide6 import QtCore, QtGui

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


# ----------------------------------------------------------------------
# Fake transport / packet helpers（对齐 test_librevna_backend.py 的字节布局）
# ----------------------------------------------------------------------


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


def _dual_payload(receivers: list[tuple[int, complex]], *, freq_hz: int = 500_000_000, point_number: int = 0) -> bytes:
    n = len(receivers)
    payload = struct.pack("<QhH", int(freq_hz), -1000, int(point_number))
    payload += struct.pack("<" + "f" * n, *[v.real for _, v in receivers])
    payload += struct.pack("<" + "f" * n, *[v.imag for _, v in receivers])
    payload += bytes(d for d, _ in receivers)
    return payload


def _vna_point_packet(freq_hz: float, point_number: int, s11: complex, s22: complex) -> bytes:
    return encode_packet(VNA_DATAPOINT, _dual_payload(_dual_receivers(s11, s22), freq_hz=freq_hz, point_number=point_number))


class _RepeatingSweepTransport:
    """按需连续生成完整 sweep 的 fake transport（open/configure 回复 ACK）。

    每次 read 返回一个 VNADatapoint，点号 0..n_points-1 循环，模拟真机连续扫频。
    """

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


class _StallTransport(_RepeatingSweepTransport):
    """发送前 ``stall_at_point`` 个点后停止，制造单次 sweep 超时（非致命）。"""

    def __init__(self, n_points: int = 5, *, stall_at_point: int = 2) -> None:
        super().__init__(n_points)
        self.stall_at_point = stall_at_point
        self._sent = 0

    def read(self, max_length: int, timeout_ms: int) -> bytes:
        if not self.opened:
            raise LibreVnaTimeoutError("not open")
        if self._reply == "device_info":
            self._reply = "data"
            return encode_packet(DEVICE_INFO, _device_info_payload())
        if self._reply == "ack":
            self._reply = "data"
            return encode_packet(ACK)
        if self._sent >= self.stall_at_point:
            raise LibreVnaTimeoutError("stalled")
        point = self._sent
        self._sent += 1
        return _vna_point_packet(
            500_000_000 + point * 10_000_000,
            point,
            complex(0.5, -0.2),
            complex(0.3, 0.1),
        )


class _TurtleConsumer(QtCore.QThread):
    """慢消费者：每条 sleep delay_s，验证生产者不被其阻塞。"""

    trace_ready = QtCore.Signal(int, object, object, object, object, object, object, object, str)
    status = QtCore.Signal(str)
    failed = QtCore.Signal(str)
    fatal_error = QtCore.Signal(str)

    def __init__(self, queue: air.BoundedQueue, gnss_cache: air.GnssFixCache, delay_s: float = 0.05) -> None:
        super().__init__()
        self._q = queue
        self._delay_s = delay_s
        self._stop = threading.Event()
        self.processed_count = 0
        self.drain_state = "running"

    def request_stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        while not self._stop.is_set():
            item = self._q.get(timeout=0.02)
            if item is None:
                continue
            time.sleep(self._delay_s)
            self.processed_count += 1
        self.drain_state = "draining"
        while True:
            item = self._q.get(timeout=0.01)
            if item is None:
                break
            time.sleep(self._delay_s)
            self.processed_count += 1
        self.drain_state = "drained"


class _FakeSer:
    is_open = True


class _FakeGnssReader:
    """替代 GnssReader 的假串口读取器（连续返回同一有效 Fix）。"""

    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.closed = False
        self.ser = None

    def open(self) -> None:
        self.ser = _FakeSer()

    def close(self) -> None:
        self.closed = True
        self.ser = None

    def read_gga(self, timeout_s: float = 2.0) -> air.GgaFix:
        return air.GgaFix(fix_quality=1, latitude=30.5, longitude=114.3, satellites=8)


# ----------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------


def _make_backend(n_points: int = 5, transport: _RepeatingSweepTransport | None = None, **settings_kwargs):
    transport = transport if transport is not None else _RepeatingSweepTransport(n_points)
    backend = PersistentLibreVnaBackend(
        transport=transport,
        settings=PersistentBackendSettings(**settings_kwargs) if settings_kwargs else None,
    )
    return backend, transport


def _make_worker(backend, *, interval_s=0.0, count=0, gnss_port="", autosave=False, **kwargs) -> "air.ContinuousAcquisitionWorker":
    cfg = air.SweepConfig(
        start_mhz=400,
        stop_mhz=4000,
        points=5,
        ifbw_hz=10000,
        ifft_points=8,
        time_window_ns=8.0,
    )
    return air.ContinuousAcquisitionWorker(
        cfg=cfg,
        interval_s=interval_s,
        out_dir=Path(tempfile.mkdtemp()),
        count=count,
        gnss_port=gnss_port,
        baud=115200,
        calibration_path="",
        session_calibration=None,
        session_calibration_p2=None,
        through_measurement=None,
        background_frequency_hz=None,
        background=None,
        autosave=autosave,
        remote_sender=None,
        backend=backend,
        **kwargs,
    )


def _wait_until(predicate, timeout_s: float, interval_s: float = 0.01) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return predicate()


DIRECT = QtCore.Qt.ConnectionType.DirectConnection


class _FakeClock:
    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, delta: float) -> None:
        self.t += delta


def _make_selected_sweep(trace_index: int = 1, midpoint: float = 100.0) -> air.SelectedSweep:
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
        device_sweep_index=trace_index - 1,
        frequency_hz=freq,
        raw=raw,
        cfg=air.SweepConfig(
            start_mhz=400, stop_mhz=4000, points=5, ifbw_hz=10000,
            ifft_points=8, time_window_ns=8.0,
        ),
        midpoint_monotonic=midpoint,
        selected_at_utc="2026-08-14T00:00:00",
        missed_slots=0,
        actual_interval_s=None,
        schedule_error_ms=None,
    )


def _fake_s_params_points(n: int = 4) -> list[tuple[float, dict[str, complex]]]:
    return [
        (
            400e6 + i * 300e6,
            {
                "S11": complex(0.5 - 0.1 * i, 0.05),
                "S21": complex(0.2, 0.0),
                "S12": complex(0.2, 0.0),
                "S22": complex(0.4 - 0.1 * i, 0.05),
            },
        )
        for i in range(n)
    ]


class _FakeRemoteSender:
    """记录 enqueue_trace 调用的假 RemoteSender（供恰好一次 / 线程位置断言）。"""

    def __init__(self, *, raise_on_enqueue: bool = False) -> None:
        self.calls: list[tuple[tuple, dict, int]] = []
        self.raise_on_enqueue = raise_on_enqueue

    def enqueue_trace(self, *args, **kwargs) -> None:
        if self.raise_on_enqueue:
            raise RuntimeError("network down")
        self.calls.append((args, kwargs, threading.get_ident()))

    @property
    def enqueue_count(self) -> int:
        return len(self.calls)


class _StubbornConsumer(QtCore.QThread):
    """顽固消费者：忽略 request_stop，直到测试放行才退出（区域三）。"""

    trace_ready = QtCore.Signal(int, object, object, object, object, object, object, object, str)
    status = QtCore.Signal(str)
    failed = QtCore.Signal(str)
    fatal_error = QtCore.Signal(str)

    def __init__(self, _queue, _gnss_cache) -> None:
        super().__init__()
        self._release = threading.Event()

    def request_stop(self) -> None:
        pass  # 顽固：忽略 stop

    def run(self) -> None:
        self._release.wait()  # 阻塞直到测试放行


class _StubbornGnssThread(QtCore.QThread):
    """顽固 GNSS 线程：忽略 request_stop，直到测试放行才退出（区域三）。"""

    status = QtCore.Signal(str)

    def __init__(self, _port, _baud, _cache) -> None:
        super().__init__()
        self._release = threading.Event()

    def request_stop(self) -> None:
        pass  # 顽固：忽略 stop

    def run(self) -> None:
        self._release.wait()


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class GnssFixCacheTests(unittest.TestCase):
    class _Clock:
        def __init__(self, t: float = 100.0) -> None:
            self.t = t

        def __call__(self) -> float:
            return self.t

    def _cache(self, *, max_age_s: float = 5.0, clock: "_Clock | None" = None):
        clock = clock or self._Clock()
        cache = air.GnssFixCache(max_age_s=max_age_s, clock=clock, utc_now=lambda: "UTC")
        return cache, clock

    def test_update_caches_valid_fix(self) -> None:
        cache, clock = self._cache()
        fix = air.GgaFix(latitude=30.5, longitude=114.3, fix_quality=1, satellites=8)
        self.assertTrue(cache.update(fix))
        self.assertEqual(cache.update_count, 1)
        lookup = cache.latest(as_of_monotonic=clock.t)
        self.assertTrue(lookup.valid)
        self.assertIs(lookup.fix, fix)
        self.assertEqual(lookup.age_ms, 0.0)

    def test_update_skips_invalid_fix(self) -> None:
        cache, _clock = self._cache()
        self.assertFalse(cache.update(air.GgaFix()))  # lat/lon None
        self.assertEqual(cache.update_count, 0)
        self.assertEqual(cache.invalid_updates_skipped, 1)
        self.assertFalse(cache.latest().valid)

    def test_latest_uses_newest_fix_at_or_before_as_of(self) -> None:
        cache, clock = self._cache()
        fix1 = air.GgaFix(fix_quality=1, latitude=30.5, longitude=114.3)
        clock.t = 100.0
        cache.update(fix1)
        fix2 = air.GgaFix(fix_quality=1, latitude=30.6, longitude=114.4)
        clock.t = 101.0
        cache.update(fix2)
        # as_of 在两条 Fix 之间 -> 取 fix1。
        lookup = cache.latest(as_of_monotonic=100.5)
        self.assertIs(lookup.fix, fix1)
        self.assertAlmostEqual(lookup.age_ms, 500.0)
        # as_of 之后 -> 取 fix2。
        lookup = cache.latest(as_of_monotonic=101.2)
        self.assertIs(lookup.fix, fix2)

    def test_stale_fix_invalid_with_age(self) -> None:
        cache, clock = self._cache(max_age_s=5.0)
        cache.update(air.GgaFix(fix_quality=1, latitude=30.5, longitude=114.3))
        clock.t = 108.0
        lookup = cache.latest(as_of_monotonic=107.0)
        self.assertFalse(lookup.valid)
        self.assertIsNotNone(lookup.fix)
        self.assertAlmostEqual(lookup.age_ms, 7000.0)

    def test_no_fix_invalid(self) -> None:
        cache, _clock = self._cache()
        lookup = cache.latest(as_of_monotonic=100.0)
        self.assertFalse(lookup.valid)
        self.assertIsNone(lookup.fix)
        self.assertIsNone(lookup.age_ms)

    def test_latest_before_first_fix_invalid(self) -> None:
        cache, clock = self._cache()
        cache.update(air.GgaFix(latitude=30.5, longitude=114.3))
        lookup = cache.latest(as_of_monotonic=99.0)
        self.assertFalse(lookup.valid)
        self.assertIsNone(lookup.fix)

    def test_timestamped_fix_carries_received_utc(self) -> None:
        cache, clock = self._cache()
        clock.t = 200.0
        cache.update(air.GgaFix(fix_quality=1, latitude=30.5, longitude=114.3))
        entry = cache._fixes[-1]  # noqa: SLF001 - 测试直接检查内部条目
        self.assertEqual(entry.received_monotonic, 200.0)
        self.assertEqual(entry.received_utc, "UTC")

    def test_fix_or_empty_returns_empty_when_missing(self) -> None:
        cache, _clock = self._cache()
        empty = cache.latest().fix_or_empty()
        self.assertIsNone(empty.latitude)
        self.assertIsNone(empty.longitude)

    def test_statistics_snapshot(self) -> None:
        cache, _clock = self._cache()
        cache.update(air.GgaFix(fix_quality=1, latitude=30.5, longitude=114.3))
        stats = cache.statistics()
        self.assertEqual(stats["size"], 1)
        self.assertEqual(stats["update_count"], 1)
        self.assertIsNotNone(stats["newest_age_s"])

    def test_invalid_parameters_rejected(self) -> None:
        with self.assertRaises(ValueError):
            air.GnssFixCache(max_age_s=0.0)
        with self.assertRaises(ValueError):
            air.GnssFixCache(max_age_s=-1.0)
        with self.assertRaises(TypeError):
            air.GnssFixCache(max_age_s=True)
        with self.assertRaises(ValueError):
            air.GnssFixCache(max_entries=0)


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class BoundedQueueTests(unittest.TestCase):
    def test_put_never_blocks_when_full_drop_incoming(self) -> None:
        q = air.BoundedQueue(2, drop_policy="drop_incoming")
        self.assertTrue(q.put("a"))
        self.assertTrue(q.put("b"))
        start = time.monotonic()
        self.assertFalse(q.put("c"))  # 满队列立即返回 False，不阻塞。
        self.assertLess(time.monotonic() - start, 0.1)
        stats = q.statistics()
        self.assertEqual(stats["depth"], 2)
        self.assertEqual(stats["overflows"], 1)
        self.assertEqual(stats["dropped"], 1)
        self.assertEqual(stats["depth_peak"], 2)

    def test_drop_oldest_replaces_oldest(self) -> None:
        q = air.BoundedQueue(2, drop_policy="drop_oldest")
        q.put("a")
        q.put("b")
        self.assertTrue(q.put("c"))
        self.assertEqual(q.get(), "b")
        self.assertEqual(q.get(), "c")
        self.assertIsNone(q.get(timeout=0.01))
        stats = q.statistics()
        self.assertEqual(stats["overflows"], 1)
        self.assertEqual(stats["dropped"], 1)

    def test_get_timeout_returns_none(self) -> None:
        q = air.BoundedQueue(1)
        start = time.monotonic()
        self.assertIsNone(q.get(timeout=0.05))
        self.assertGreaterEqual(time.monotonic() - start, 0.04)

    def test_put_get_roundtrip_and_depth_peak(self) -> None:
        q = air.BoundedQueue(3)
        for i in range(3):
            self.assertTrue(q.put(i))
        self.assertEqual(q.statistics()["depth_peak"], 3)
        self.assertEqual([q.get() for _ in range(3)], [0, 1, 2])

    def test_validation_rejects_bad_args(self) -> None:
        with self.assertRaises(ValueError):
            air.BoundedQueue(0)
        with self.assertRaises(TypeError):
            air.BoundedQueue(2.5)
        with self.assertRaises(ValueError):
            air.BoundedQueue(2, drop_policy="nope")


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class ContinuousWorkerLifecycleTests(unittest.TestCase):
    def test_start_immediate_stop_closes_backend(self) -> None:
        backend, transport = _make_backend(5)
        worker = _make_worker(backend)
        worker.start()
        worker.stop()
        self.assertTrue(worker.wait(5000), "worker 应在 5s 内退出")
        self.assertFalse(worker.isRunning())
        # stop 可能早于 open：无论是否曾打开，结束时 USB 必须已释放。
        self.assertFalse(backend.is_open, "stop 后必须释放 USB")
        if transport.open_count >= 1:
            self.assertEqual(transport.close_count, 1, "打开过的传输必须被 close 一次")

    def test_repeated_start_stop_no_residual_threads(self) -> None:
        for _ in range(3):
            backend, transport = _make_backend(5)
            worker = _make_worker(backend, count=3)
            worker.start()
            self.assertTrue(worker.wait(5000))
            self.assertFalse(worker.isRunning())
            self.assertFalse(backend.is_open)
            self.assertIsNotNone(worker.consumer)
            self.assertFalse(worker.consumer.isRunning(), "消费者线程必须已退出")
            self.assertEqual(transport.close_count, 1)

    def test_count_limits_selected_and_consumer_drains(self) -> None:
        backend, _transport = _make_backend(5)
        worker = _make_worker(backend, count=5)
        worker.start()
        self.assertTrue(worker.wait(5000))
        self.assertFalse(worker.isRunning())
        self.assertEqual(worker.scheduler.selected_sweeps, 5)
        self.assertEqual(backend.statistics.completed_sweeps, 5)
        # 停止后消费者排空已选数据：队列无残留、全部处理。
        self.assertEqual(worker.processing_queue.statistics()["depth"], 0)
        consumer = worker.consumer
        self.assertIsNotNone(consumer)
        self.assertEqual(consumer.processed_count, 5)

    def test_interval_zero_selects_every_complete_sweep(self) -> None:
        backend, _transport = _make_backend(5)
        worker = _make_worker(backend, interval_s=0.0, count=4)
        worker.start()
        self.assertTrue(worker.wait(5000))
        self.assertEqual(worker.scheduler.selected_sweeps, 4)
        self.assertEqual(worker.scheduler.statistics["skipped_sweeps"], 0)
        self.assertEqual(backend.statistics.completed_sweeps, 4)

    def test_interval_positive_first_immediate_then_paced(self) -> None:
        backend, _transport = _make_backend(5)
        worker = _make_worker(backend, interval_s=0.2, count=2)
        worker.start()
        self.assertTrue(_wait_until(lambda: worker.scheduler.selected_sweeps >= 2, 3.0))
        # 第一条立即选择（无实际间隔）；第二条应接近 0.2s 的实际间隔。
        interval_stats = worker.scheduler.statistics["actual_interval_s"]
        self.assertEqual(interval_stats["count"], 1)
        self.assertGreaterEqual(interval_stats["mean"], 0.12)
        self.assertLessEqual(interval_stats["mean"], 0.45)
        worker.stop()
        self.assertTrue(worker.wait(5000))

    def test_single_sweep_timeout_is_not_fatal(self) -> None:
        transport = _StallTransport(5, stall_at_point=2)
        backend = PersistentLibreVnaBackend(
            transport=transport,
            settings=PersistentBackendSettings(sweep_timeout_s=0.1),
        )
        worker = _make_worker(backend)
        worker.start()
        time.sleep(0.35)  # 约 3 个 0.1s 超时周期
        self.assertTrue(worker.isRunning(), "单次 sweep 超时不应杀死 worker")
        self.assertEqual(backend.state.value, "streaming", "超时后保持 STREAMING 重同步")
        self.assertGreaterEqual(backend.statistics.sweep_timeouts, 2)
        worker.stop()
        self.assertTrue(worker.wait(5000))
        self.assertFalse(backend.is_open)

    def test_pause_resume_no_residual_threads(self) -> None:
        backend, _transport = _make_backend(5)
        worker = _make_worker(backend)
        worker.start()
        self.assertTrue(_wait_until(lambda: backend.statistics.completed_sweeps >= 1, 3.0))
        worker.request_pause()
        self.assertTrue(
            _wait_until(lambda: worker.is_paused_at_boundary(), 3.0),
            "worker 应在 sweep 边界进入暂停",
        )
        self.assertEqual(backend.state.value, "paused")
        worker.request_resume()
        self.assertTrue(
            _wait_until(lambda: backend.state.value == "streaming", 3.0),
            "resume 后应回到 STREAMING",
        )
        worker.stop()
        self.assertTrue(worker.wait(5000))
        self.assertFalse(worker.isRunning())
        self.assertFalse(backend.is_open)
        consumer = worker.consumer
        self.assertIsNotNone(consumer)
        self.assertFalse(consumer.isRunning())

    def test_open_configure_once_only(self) -> None:
        backend, transport = _make_backend(5)
        worker = _make_worker(backend, count=3)
        worker.start()
        self.assertTrue(worker.wait(5000))
        self.assertEqual(transport.open_count, 1)
        sweep_settings_writes = [
            w for w in transport.writes if len(w) >= 4 and w[0] == 0x5A and w[3] == SWEEP_SETTINGS
        ]
        self.assertEqual(len(sweep_settings_writes), 1)


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class SlowConsumerTests(unittest.TestCase):
    def test_slow_consumer_lossless_no_drops(self) -> None:
        """第1轮检查点C：慢消费者不丢 Trace——队列无损积压，达到硬上限时
        受控停止新采集，已选数据全部排空（processed == selected）。"""
        backend, _transport = _make_backend(5)
        worker = _make_worker(
            backend,
            interval_s=0.0,
            processing_queue_maxsize=2,
            consumer_factory=lambda queue, cache: _TurtleConsumer(queue, cache, delay_s=0.05),
        )
        worker.start()
        self.assertTrue(_wait_until(lambda: backend.statistics.completed_sweeps >= 1, 3.0))
        time.sleep(0.3)
        worker.stop()
        self.assertTrue(worker.wait(10000))
        # 无损：没有任何丢弃。
        queue_stats = worker.processing_queue.statistics()
        self.assertNotIn("dropped", queue_stats, "生产队列不得再使用丢弃策略")
        self.assertFalse(worker.processing_queue.stop_requested.is_set() and worker._controlled_stop_reason is None)
        # 对账：处理数 == 选择数（全部排空，不丢不重）。
        consumer = worker.consumer
        self.assertIsNotNone(consumer)
        self.assertEqual(consumer.processed_count, worker._selected_count)
        self.assertEqual(consumer.drain_state, "drained")


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class GnssReaderThreadTests(unittest.TestCase):
    def test_gnss_thread_reads_and_caches_fixes(self) -> None:
        cache = air.GnssFixCache()
        thread = air.GnssReaderThread("COM9", 115200, cache, poll_interval_s=0.01)
        with unittest.mock.patch.object(air, "GnssReader", _FakeGnssReader):
            thread.start()
            self.assertTrue(_wait_until(lambda: cache.update_count >= 1, 3.0))
            thread.request_stop()
            self.assertTrue(thread.wait(3000))
        self.assertFalse(thread.isRunning())
        self.assertTrue(cache.update_count >= 1)
        lookup = cache.latest()
        self.assertIsNotNone(lookup.fix)
        self.assertAlmostEqual(lookup.fix.latitude, 30.5)

    def test_gnss_open_failure_does_not_raise(self) -> None:
        class _FailingReader(_FakeGnssReader):
            def open(self) -> None:
                raise RuntimeError("serial unavailable")

        cache = air.GnssFixCache()
        thread = air.GnssReaderThread("COM9", 115200, cache, poll_interval_s=0.01)
        with unittest.mock.patch.object(air, "GnssReader", _FailingReader):
            thread.start()
            time.sleep(0.2)  # 初次打开失败应进入重试循环而非退出
            self.assertTrue(thread.isRunning(), "初次 open 失败应重试而不退出")
            thread.request_stop()
            self.assertTrue(thread.wait(3000))
        self.assertFalse(thread.isRunning())
        self.assertEqual(cache.update_count, 0)


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class GnssReaderWriteTimeoutTests(unittest.TestCase):
    """GNSS open 的写保险丝（2026-09-30 现场挂死回归）。

    现场：GNSS 串口 open() 内的 write() 无界阻塞 → GnssReaderThread 卡死在
    第一次 open → stop 后无限 join → worker 永不结束 → 新任务被
    "采集已经在运行"拒绝。修复=write_timeout=1.0 + 写失败关串口重抛。
    """

    class _WriteBlockedSer:
        """模拟驱动 TX 永不排空：write 必抛写超时（过去会永久阻塞）。"""

        def __init__(self, *args, **kwargs) -> None:
            self.is_open = True
            self.closed = False
            self.write_timeout = kwargs.get("write_timeout")

        def write(self, _data: bytes) -> int:
            raise air.serial.SerialTimeoutException("simulated: write blocked forever")

        def close(self) -> None:
            self.closed = True
            self.is_open = False

    class _RecordingSer:
        def __init__(self, *args, **kwargs) -> None:
            self.is_open = True
            self.write_timeout = kwargs.get("write_timeout")
            self.written: list[bytes] = []

        def write(self, data: bytes) -> int:
            self.written.append(data)
            return len(data)

        def close(self) -> None:
            self.is_open = False

    def test_open_write_blocked_closes_port_and_raises(self) -> None:
        reader = air.GnssReader("COM9", 115200)
        created: list[GnssReaderWriteTimeoutTests._WriteBlockedSer] = []

        def _factory(*args, **kwargs):
            ser = GnssReaderWriteTimeoutTests._WriteBlockedSer(*args, **kwargs)
            created.append(ser)
            return ser

        with unittest.mock.patch.object(air.serial, "Serial", _factory):
            with self.assertRaises(air.serial.SerialTimeoutException):
                reader.open()
        self.assertIsNone(reader.ser, "写保险丝触发后必须释放串口句柄")
        self.assertTrue(created and created[0].closed, "打开失败路径必须先 close 串口")

    def test_open_sets_write_timeout_and_sends_config(self) -> None:
        reader = air.GnssReader("COM9", 115200)
        created: list[GnssReaderWriteTimeoutTests._RecordingSer] = []

        def _factory(*args, **kwargs):
            ser = GnssReaderWriteTimeoutTests._RecordingSer(*args, **kwargs)
            created.append(ser)
            return ser

        with unittest.mock.patch.object(air.serial, "Serial", _factory):
            reader.open()
        self.assertIsNotNone(reader.ser)
        self.assertEqual(created[0].write_timeout, 1.0, "open 必须携带 write_timeout=1.0 保险丝")
        self.assertEqual(created[0].written, [b"log gpgga ontime 1\r\n", b"log gprmc ontime 1\r\n"])


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class GnssPortSelectionTests(unittest.TestCase):
    """GNSS 串口选择契约（2026-09-30 现场回归：无 GNSS 硬件）。

    此前 refresh_ports 自动选中扫描到的串口（优先 COM3），无 GNSS 硬件时
    选中的是别的设备，任务一启动就拉起 GNSS 线程并被其卡死。新契约：
    首项"不使用 GNSS"（data=""）且**绝不自动选中**任何串口；selected_port()
    返回空串 ⇒ worker 不启动 GNSS 线程（既有守卫）。
    """

    def test_worker_without_gnss_port_starts_no_gnss_thread(self) -> None:
        """selected_port()==""（不使用 GNSS）⇒ worker.gnss_thread 为 None。"""
        backend, _transport = _make_backend(5)
        worker = _make_worker(backend, count=1, gnss_port="")
        worker.start()
        self.assertIsNone(worker.gnss_thread, "gnss_port 为空时绝不能创建 GNSS 线程")
        self.assertTrue(worker.wait(5000))
        self.assertFalse(worker.isRunning())


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class FeatureFlagTests(unittest.TestCase):
    """采集引擎 feature flag 契约（第4轮：**默认改为高速连续**）。

    第4轮透明更新（真机 2.75 s/帧 修复，团队任务卡 t1）：
    - 旧契约：缺省 / 非法值 → legacy；显式 env 才 continuous。
      （原 test 名为 test_default_mode_is_legacy / test_invalid_env_value_falls_back_to_legacy）
    - 新契约：缺省 → **continuous**（高速常驻引擎，USB 只开一次）；非法值同样
      回退 continuous；legacy 只能**显式**选（env=legacy 或界面下拉）。
      断言意图（默认值是谁 / env 能否覆盖 / 非法值回退到默认）逐条保留，
      只是默认值本身变了——不是删除断言。
    """

    def test_default_mode_is_continuous(self) -> None:
        """缺省引擎 = 高速连续采集（原 test_default_mode_is_legacy 的契约反转）。"""
        self.assertEqual(air.DEFAULT_ACQUISITION_MODE, "continuous")
        self.assertEqual(air.acquisition_mode(), "continuous")
        self.assertTrue(air.is_continuous_enabled())

    def test_env_var_selects_legacy(self) -> None:
        """legacy 退路仍可用：显式 env=legacy 明确切回慢引擎（调试用）。"""
        with unittest.mock.patch.dict(os.environ, {"UAV_GPR_ACQUISITION_MODE": "legacy"}):
            self.assertEqual(air.acquisition_mode(), "legacy")
            self.assertFalse(air.is_continuous_enabled())

    def test_env_var_selects_continuous(self) -> None:
        with unittest.mock.patch.dict(os.environ, {"UAV_GPR_ACQUISITION_MODE": "continuous"}):
            self.assertTrue(air.is_continuous_enabled())
            self.assertEqual(air.acquisition_mode(), "continuous")

    def test_invalid_env_value_falls_back_to_continuous(self) -> None:
        """非法值回退到**当前默认**（continuous）——绝不静默掉进慢引擎。"""
        with unittest.mock.patch.dict(os.environ, {"UAV_GPR_ACQUISITION_MODE": "bogus"}):
            self.assertEqual(air.acquisition_mode(), "continuous")
            self.assertTrue(air.is_continuous_enabled())


class UsbWorkerStructuralTests(unittest.TestCase):
    """结构断言：USB worker 循环中不得出现保存间隔 sleep / GNSS / 压缩 / 网络。"""

    FORBIDDEN_SOURCE_FRAGMENTS = (
        "sleep(",  # 保存间隔 sleep
        "save_trace(",
        "trace_npz_bytes(",
        "enqueue_trace(",
        "read_gga(",
        "GnssFixCache().update",
    )

    def _worker_run_source(self) -> str:
        if air is None:
            self.skipTest("uav_gpr_qt dependencies unavailable")
        return inspect_source_lines(air, "ContinuousAcquisitionWorker.run")

    def test_usb_worker_has_no_save_interval_sleep_or_gnss_or_io(self) -> None:
        source = self._worker_run_source()
        for fragment in self.FORBIDDEN_SOURCE_FRAGMENTS:
            self.assertNotIn(
                fragment,
                source,
                f"USB worker 循环中禁止出现 {fragment!r}（应只负责组装与入队）",
            )

    def test_usb_worker_uses_scheduler_and_bounded_queue(self) -> None:
        source = self._worker_run_source()
        self.assertIn("should_select", source)
        self.assertIn("processing_queue.put", source)

    def test_events_used_for_stop_pause_resume(self) -> None:
        if air is None:
            self.skipTest("uav_gpr_qt dependencies unavailable")
        init_source = inspect_source_lines(air, "ContinuousAcquisitionWorker.__init__")
        self.assertIn("threading.Event()", init_source)


def inspect_source_lines(module, qualified_name: str) -> str:
    import inspect

    current: object = module
    for part in qualified_name.split("."):
        current = getattr(current, part)
    return inspect.getsource(current)


class _FakeWorker(QtCore.QObject):
    """可注入的假采集 worker：记录构造参数，start() 不启动真实线程。"""

    trace_ready = QtCore.Signal(int, object, object, object, object, object, object, object, str)
    status = QtCore.Signal(str)
    failed = QtCore.Signal(str)
    finished_run = QtCore.Signal()
    paused = QtCore.Signal()
    resumed = QtCore.Signal()
    statistics_updated = QtCore.Signal(dict)

    def __init__(self, **kwargs) -> None:
        super().__init__()
        self.kwargs = kwargs
        self.started = False

    def start(self) -> None:
        self.started = True

    def isRunning(self) -> bool:
        return False


class _FakeStoppableSender:
    """可控 stop() 返回值的假 RemoteSender（closeEvent 回归测试用）。"""

    def __init__(self, exited: bool = False) -> None:
        self.exited = exited
        self.stop_calls: list[int] = []

    def stop(self, timeout_ms: int = 3000, *, drain: bool = True) -> bool:
        self.stop_calls.append(timeout_ms)
        return self.exited


class _ControllableWorker(QtCore.QThread):
    """closeEvent 测试用 worker：stop() 后退出，或阻塞直至测试放行。"""

    def __init__(self, *, obey_stop: bool = True) -> None:
        super().__init__()
        self._stop = threading.Event()
        self._release = threading.Event()
        self._obey_stop = obey_stop
        self.stop_called = 0

    def stop(self) -> None:
        self.stop_called += 1
        if self._obey_stop:
            self._stop.set()

    def run(self) -> None:
        if self._obey_stop:
            self._stop.wait()
        else:
            self._release.wait()  # 忽略 stop，模拟卡死的 worker


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class GnssValidityTests(unittest.TestCase):
    """区域一：is_valid_gnss_fix 有效性判定。"""

    def test_accepts_valid_fix(self) -> None:
        fix = air.GgaFix(fix_quality=1, latitude=30.5, longitude=114.3)
        self.assertTrue(air.is_valid_gnss_fix(fix))

    def test_rejects_fix_quality_zero_with_coords(self) -> None:
        fix = air.GgaFix(fix_quality=0, latitude=30.5, longitude=114.3)
        self.assertFalse(air.is_valid_gnss_fix(fix))

    def test_rejects_nan_or_inf_coords(self) -> None:
        self.assertFalse(
            air.is_valid_gnss_fix(air.GgaFix(fix_quality=1, latitude=float("nan"), longitude=114.3))
        )
        self.assertFalse(
            air.is_valid_gnss_fix(air.GgaFix(fix_quality=1, latitude=30.5, longitude=float("inf")))
        )

    def test_rejects_out_of_range_coords(self) -> None:
        self.assertFalse(air.is_valid_gnss_fix(air.GgaFix(fix_quality=1, latitude=91.0, longitude=114.3)))
        self.assertFalse(air.is_valid_gnss_fix(air.GgaFix(fix_quality=1, latitude=-90.1, longitude=0.0)))
        self.assertFalse(air.is_valid_gnss_fix(air.GgaFix(fix_quality=1, latitude=30.5, longitude=181.0)))

    def test_rejects_bool_and_string_coords(self) -> None:
        self.assertFalse(air.is_valid_gnss_fix(air.GgaFix(fix_quality=1, latitude=True, longitude=114.3)))
        self.assertFalse(air.is_valid_gnss_fix(air.GgaFix(fix_quality=1, latitude="30.5", longitude=114.3)))

    def test_rejects_missing_coords(self) -> None:
        self.assertFalse(air.is_valid_gnss_fix(air.GgaFix(fix_quality=1)))

    def test_rejects_bool_fix_quality(self) -> None:
        self.assertFalse(air.is_valid_gnss_fix(air.GgaFix(fix_quality=True, latitude=30.5, longitude=114.3)))

    def test_rejects_non_gga_fix(self) -> None:
        self.assertFalse(air.is_valid_gnss_fix(object()))
        self.assertFalse(air.is_valid_gnss_fix(None))


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class GnssFixCacheRejectTests(unittest.TestCase):
    """区域一：GnssFixCache.update 拒绝无效 Fix。"""

    def _cache(self):
        return air.GnssFixCache(clock=_FakeClock())

    def test_update_rejects_quality_zero(self) -> None:
        cache = self._cache()
        fix = air.GgaFix(fix_quality=0, latitude=30.5, longitude=114.3)
        self.assertFalse(cache.update(fix))
        self.assertEqual(cache.update_count, 0)
        self.assertEqual(cache.invalid_updates_skipped, 1)
        self.assertFalse(cache.latest().valid)

    def test_update_rejects_nan_coords(self) -> None:
        cache = self._cache()
        self.assertFalse(cache.update(air.GgaFix(fix_quality=1, latitude=float("nan"), longitude=114.3)))
        self.assertEqual(cache.update_count, 0)
        self.assertEqual(cache.invalid_updates_skipped, 1)

    def test_update_rejects_out_of_range_coords(self) -> None:
        cache = self._cache()
        self.assertFalse(cache.update(air.GgaFix(fix_quality=1, latitude=91.0, longitude=0.0)))
        self.assertEqual(cache.invalid_updates_skipped, 1)

    def test_update_accepts_valid_fix(self) -> None:
        cache = self._cache()
        self.assertTrue(cache.update(air.GgaFix(fix_quality=1, latitude=30.5, longitude=114.3)))
        self.assertEqual(cache.update_count, 1)
        self.assertEqual(cache.invalid_updates_skipped, 0)


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class ProcessingConsumerFixValidityTests(unittest.TestCase):
    """区域一：ProcessingConsumer._process_one 只在 lookup.valid 时传递 Fix。"""

    def _consumer(self, cache, remote=None, *, autosave=True):
        return air.ProcessingConsumer(
            queue=air.BoundedQueue(4),
            gnss_cache=cache,
            cfg=_make_selected_sweep().cfg,
            out_dir=Path(tempfile.mkdtemp()),
            autosave=autosave,
            calibration_p1=None,
            calibration_p2=None,
            through_measurement=None,
            background_frequency_hz=None,
            background=None,
            remote_sender=remote,
        )

    def test_stale_fix_passes_empty_fix_to_save_and_remote(self) -> None:
        clock = _FakeClock(100.0)
        cache = air.GnssFixCache(max_age_s=5.0, clock=clock)
        cache.update(air.GgaFix(fix_quality=1, latitude=30.5, longitude=114.3, altitude_m=50.0))
        clock.t = 110.0  # age 10s > 5s -> stale
        remote = _FakeRemoteSender()
        consumer = self._consumer(cache, remote, autosave=True)
        saved: list[tuple] = []

        def fake_save_trace(*args, **kwargs):
            saved.append((args, kwargs))
            return Path("trace_dummy.csv")

        with unittest.mock.patch.object(air, "save_trace", side_effect=fake_save_trace):
            consumer._process_one(_make_selected_sweep(1, midpoint=109.0))
        self.assertEqual(len(saved), 1)
        save_fix = saved[0][0][8]  # save_trace 第 9 个位置参数是 fix
        self.assertIsInstance(save_fix, air.GgaFix)
        self.assertEqual(save_fix.fix_quality, 0, "stale Fix 不得带有效质量")
        self.assertIsNone(save_fix.latitude, "stale Fix 不得带陈旧坐标")
        self.assertIsNone(save_fix.longitude)
        # 遥测收到的也是空 Fix。
        self.assertEqual(remote.enqueue_count, 1)
        remote_fix = remote.calls[0][0][7]  # enqueue_trace 第 8 个位置参数是 fix
        self.assertEqual(remote_fix.fix_quality, 0)
        self.assertIsNone(remote_fix.latitude)
        # age 保留给统计，不伪造为 0。
        self.assertIsNotNone(consumer.last_gnss_age_ms)
        self.assertGreaterEqual(consumer.last_gnss_age_ms, 1000.0)

    def test_valid_fresh_fix_passed_through(self) -> None:
        clock = _FakeClock(100.0)
        cache = air.GnssFixCache(max_age_s=5.0, clock=clock)
        fix = air.GgaFix(fix_quality=1, latitude=30.5, longitude=114.3)
        cache.update(fix)
        clock.t = 100.5  # age 0.5s < 5s -> valid
        remote = _FakeRemoteSender()
        consumer = self._consumer(cache, remote, autosave=False)
        captured: list[tuple] = []
        consumer.trace_ready.connect(lambda *args: captured.append(args), DIRECT)
        consumer._process_one(_make_selected_sweep(1, midpoint=100.4))
        self.assertEqual(len(captured), 1)
        trace_fix = captured[0][6]  # trace_ready 第 7 个参数是 fix
        self.assertIs(trace_fix, fix)
        self.assertEqual(remote.enqueue_count, 1)
        self.assertIs(remote.calls[0][0][7], fix)


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class GnssReconnectTests(unittest.TestCase):
    """区域二：GnssReaderThread 初次失败重试 / 读取失败重连 / 可停止。"""

    def test_first_two_opens_fail_third_succeeds(self) -> None:
        class _Flaky(_FakeGnssReader):
            attempts = 0

            def open(self):
                type(self).attempts += 1
                if type(self).attempts <= 2:
                    raise RuntimeError("port busy")
                self.ser = _FakeSer()

        cache = air.GnssFixCache()
        thread = air.GnssReaderThread("COM9", 115200, cache, poll_interval_s=0.01)
        with unittest.mock.patch.object(air, "GnssReader", _Flaky):
            thread.start()
            self.assertTrue(_wait_until(lambda: cache.update_count >= 1, 6.0))
            thread.request_stop()
            self.assertTrue(thread.wait(3000))
        self.assertFalse(thread.isRunning())
        self.assertGreaterEqual(cache.update_count, 1)

    def test_permanent_open_failure_stops_on_request(self) -> None:
        class _AlwaysFail(_FakeGnssReader):
            def open(self):
                raise RuntimeError("always down")

        cache = air.GnssFixCache()
        thread = air.GnssReaderThread("COM9", 115200, cache, poll_interval_s=0.01)
        with unittest.mock.patch.object(air, "GnssReader", _AlwaysFail):
            thread.start()
            time.sleep(0.3)  # 处于退避重试中
            self.assertTrue(thread.isRunning())
            thread.request_stop()
            self.assertTrue(thread.wait(3000), "永久失败 + request_stop 应能及时退出")
        self.assertFalse(thread.isRunning())
        self.assertEqual(cache.update_count, 0)

    def test_read_failure_reopens_and_recovers(self) -> None:
        class _ReadFailsOnce(_FakeGnssReader):
            reads = 0

            def read_gga(self, timeout_s=2.0):
                type(self).reads += 1
                if type(self).reads == 1:
                    raise RuntimeError("read timeout")
                return super().read_gga(timeout_s=timeout_s)

        cache = air.GnssFixCache()
        thread = air.GnssReaderThread("COM9", 115200, cache, poll_interval_s=0.01)
        with unittest.mock.patch.object(air, "GnssReader", _ReadFailsOnce):
            thread.start()
            self.assertTrue(_wait_until(lambda: cache.update_count >= 1, 6.0))
            thread.request_stop()
            self.assertTrue(thread.wait(3000))
        self.assertGreaterEqual(cache.update_count, 1)

    def test_quality_zero_read_not_cached(self) -> None:
        class _NoFix(_FakeGnssReader):
            def read_gga(self, timeout_s=2.0):
                return air.GgaFix(fix_quality=0, latitude=30.5, longitude=114.3)

        cache = air.GnssFixCache()
        thread = air.GnssReaderThread("COM9", 115200, cache, poll_interval_s=0.01)
        with unittest.mock.patch.object(air, "GnssReader", _NoFix):
            thread.start()
            self.assertTrue(_wait_until(lambda: cache.invalid_updates_skipped >= 1, 6.0))
            thread.request_stop()
            self.assertTrue(thread.wait(3000))
        self.assertEqual(cache.update_count, 0)
        self.assertGreaterEqual(cache.invalid_updates_skipped, 1)


class _CloseFailsTransport(_RepeatingSweepTransport):
    """close() 抛错但确实释放（opened=False）：backend.close 失败需明确上报。"""

    def close(self) -> None:
        super().close()
        raise LibreVnaTimeoutError("close failed")


class _CloseKeepsOpenTransport(_RepeatingSweepTransport):
    """close() 抛错且保持 opened=True，直到 ``closable=True`` 才真正释放。

    模拟"释放失败且资源仍被 USB 持有"：阶段 2 backend 进入 FAULTED 仍 is_open。
    """

    def __init__(self, n_points: int = 5) -> None:
        super().__init__(n_points)
        self.close_calls = 0
        self.closable = False

    def close(self) -> None:
        self.close_calls += 1
        if not self.closable:
            raise LibreVnaTimeoutError("transport refuses to close")
        self.opened = False


class _SilentCloseKeepsOpenTransport(_RepeatingSweepTransport):
    """close() 正常返回但仍保持 opened=True，直到 ``closable=True`` 才释放。

    使用阶段 2 已支持的"静默关闭失败"语义（close 返回但 transport 仍报告打开）。
    """

    def __init__(self, n_points: int = 5) -> None:
        super().__init__(n_points)
        self.close_calls = 0
        self.closable = False

    def close(self) -> None:
        self.close_calls += 1
        if not self.closable:
            return  # 静默失败：正常返回但 opened 仍 True
        self.opened = False


class _CloseReleaseThenRaiseTransport(_RepeatingSweepTransport):
    """close() 先释放资源（opened=False）再抛错：资源已释放但报告清理错误。"""

    def close(self) -> None:
        super().close()  # opened=False, close_count += 1
        raise LibreVnaTimeoutError("close reported after release")


class _RecordingConsumer(QtCore.QThread):
    """录制 request_stop 后立即退出的消费者（shutdown 顺序测试用）。"""

    trace_ready = QtCore.Signal(int, object, object, object, object, object, object, object, str)
    status = QtCore.Signal(str)
    failed = QtCore.Signal(str)
    fatal_error = QtCore.Signal(str)

    def __init__(self, _queue, _gnss_cache, events: list[str]) -> None:
        super().__init__()
        self.events = events

    def request_stop(self) -> None:
        self.events.append("consumer.request_stop")

    def run(self) -> None:
        pass  # 立即退出


class _RecordingGnssThread(QtCore.QThread):
    """录制 request_stop 后立即退出的 GNSS 线程（shutdown 顺序测试用）。"""

    status = QtCore.Signal(str)

    def __init__(self, _port, _baud, _cache, events: list[str]) -> None:
        super().__init__()
        self.events = events

    def request_stop(self) -> None:
        self.events.append("gnss.request_stop")

    def run(self) -> None:
        pass  # 立即退出


class _OrderRecordingTransport(_RepeatingSweepTransport):
    """首次 close() 抛错并保持打开，记录事件顺序；再次 close() 成功释放。"""

    def __init__(self, n_points: int = 5, events: list[str] | None = None) -> None:
        super().__init__(n_points)
        self.events = events
        self.close_attempts = 0

    def close(self) -> None:
        self.events.append("backend.close")
        self.close_attempts += 1
        if self.close_attempts == 1:
            raise LibreVnaTimeoutError("first close fails")
        self.opened = False


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class StubbornThreadTests(unittest.TestCase):
    """区域三：子线程顽固时不发 finished_run、backend 先释放、释放后 worker 才结束。"""

    def test_stubborn_consumer_worker_does_not_finish_until_released(self) -> None:
        backend, _transport = _make_backend(5)
        finished = threading.Event()
        worker = _make_worker(
            backend,
            count=0,
            interval_s=0.0,
            consumer_factory=lambda queue, cache: _StubbornConsumer(queue, cache),
        )
        worker.finished_run.connect(finished.set, DIRECT)
        worker.start()
        self.assertTrue(_wait_until(lambda: backend.is_open, 3.0))
        worker.stop()
        # 超过有界 3s wait 后 worker 仍不得结束 / 发 finished_run。
        self.assertFalse(worker.wait(3500), "stubborn consumer 存在时 worker 不应在 3.5s 内结束")
        self.assertTrue(worker.isRunning())
        self.assertFalse(finished.is_set())
        self.assertFalse(backend.is_open, "backend 必须先释放（USB）")
        # 释放 stubborn consumer 后 worker 才结束，无残留。
        consumer = worker.consumer
        self.assertIsNotNone(consumer)
        consumer._release.set()  # noqa: SLF001
        self.assertTrue(worker.wait(5000), "consumer 释放后 worker 应结束")
        self.assertFalse(worker.isRunning())
        self.assertFalse(consumer.isRunning())
        self.assertTrue(finished.is_set())

    def test_stubborn_gnss_worker_does_not_finish_until_released(self) -> None:
        backend, _transport = _make_backend(5)
        finished = threading.Event()
        worker = _make_worker(backend, count=0, interval_s=0.0, gnss_port="COM9")
        worker.finished_run.connect(finished.set, DIRECT)
        with unittest.mock.patch.object(air, "GnssReaderThread", _StubbornGnssThread):
            worker.start()
            self.assertTrue(_wait_until(lambda: backend.is_open, 3.0))
            worker.stop()
            self.assertFalse(worker.wait(3500))
            self.assertTrue(worker.isRunning())
            self.assertFalse(finished.is_set())
            self.assertFalse(backend.is_open)
            gnss = worker.gnss_thread
            self.assertIsNotNone(gnss)
            gnss._release.set()  # noqa: SLF001
            self.assertTrue(worker.wait(5000))
            self.assertFalse(gnss.isRunning())
            self.assertFalse(worker.isRunning())
            self.assertTrue(finished.is_set())

    def test_stubborn_gnss_thread_abandoned_after_bounded_wait(self) -> None:
        """收尾总闸（2026-09-30 现场回归）：GNSS 线程永不退出时，worker 在
        有界等待后遗弃它并正常收尾——任务可结束、下一任务可启动；绝不
        terminate，遗弃线程寄存保活防 QThread 提前析构。"""
        backend, _transport = _make_backend(5)
        finished = threading.Event()
        statuses: list[str] = []
        worker = _make_worker(backend, count=0, interval_s=0.0, gnss_port="COM9")
        worker.finished_run.connect(finished.set, DIRECT)
        worker.status.connect(statuses.append, DIRECT)
        gnss = None
        try:
            with (
                unittest.mock.patch.object(
                    air.ContinuousAcquisitionWorker, "GNSS_SHUTDOWN_FIRST_WAIT_S", 0.2
                ),
                unittest.mock.patch.object(
                    air.ContinuousAcquisitionWorker, "GNSS_SHUTDOWN_EXTRA_WAIT_S", 0.3
                ),
                unittest.mock.patch.object(air, "GnssReaderThread", _StubbornGnssThread),
            ):
                worker.start()
                self.assertTrue(_wait_until(lambda: backend.is_open, 3.0))
                worker.stop()
                self.assertTrue(
                    worker.wait(8000),
                    "GNSS 顽固不退时 worker 应在总闸触发后正常结束（不得无限 join）",
                )
                self.assertFalse(worker.isRunning())
                self.assertTrue(
                    finished.is_set(),
                    "遗弃 GNSS 后必须发 finished_run（否则下一任务被'采集已经在运行'拒绝）",
                )
                gnss = worker.gnss_thread
                self.assertIsNotNone(gnss)
                self.assertTrue(gnss.isRunning(), "遗弃≠terminate：线程仍在运行，等待自然退出")
                self.assertIn(gnss, air._ABANDONED_GNSS_THREADS, "遗弃线程必须寄存保活")  # noqa: SLF001
                self.assertTrue(
                    any("放弃等待并继续收尾" in s for s in statuses),
                    "总闸触发必须有明确日志（现场可诊断）",
                )
        finally:
            if gnss is not None:
                gnss._release.set()  # noqa: SLF001
                gnss.wait(3000)
        self.assertFalse(gnss.isRunning())

    def test_backend_close_failure_reported(self) -> None:
        backend = PersistentLibreVnaBackend(transport=_CloseFailsTransport(5))
        failed_messages: list[str] = []
        worker = _make_worker(backend, count=1)
        worker.failed.connect(failed_messages.append, DIRECT)
        worker.start()
        self.assertTrue(worker.wait(5000))
        self.assertFalse(worker.isRunning())
        self.assertFalse(backend.is_open)
        self.assertTrue(
            any("释放 LibreVNA 失败" in message for message in failed_messages),
            "backend.close() 失败必须明确上报，不得伪装成功",
        )


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class RealConsumerTests(unittest.TestCase):
    """区域八：真实 ProcessingConsumer 处理入口。"""

    def _consumer(self, remote=None, *, autosave=False, out_dir=None, **kwargs):
        return air.ProcessingConsumer(
            queue=air.BoundedQueue(4),
            gnss_cache=air.GnssFixCache(),
            cfg=_make_selected_sweep().cfg,
            out_dir=out_dir or Path(tempfile.mkdtemp()),
            autosave=autosave,
            calibration_p1=None,
            calibration_p2=None,
            through_measurement=None,
            background_frequency_hz=None,
            background=None,
            remote_sender=remote,
            **kwargs,
        )

    def test_real_pipeline_processes_dual_channel_input(self) -> None:
        item = _make_selected_sweep(1, midpoint=100.0)
        raw = air.dual_channel_raw(item.raw)
        self.assertEqual(sorted(raw.keys()), ["S11", "S22"])
        display, time_ns, traces = air.process_trace_pipeline(item.frequency_hz, raw, item.cfg)
        self.assertEqual(sorted(traces.keys()), ["S11", "S22"])
        self.assertIsNotNone(time_ns)
        self.assertEqual(display["S11"].shape, item.frequency_hz.shape)

    def test_autosave_success_writes_dual_channel_files(self) -> None:
        out_dir = Path(tempfile.mkdtemp())
        consumer = self._consumer(autosave=True, out_dir=out_dir)
        consumer._process_one(_make_selected_sweep(1, midpoint=100.0))
        csvs = list(out_dir.glob("trace_00001_*.csv"))
        self.assertEqual(len(csvs), 1)
        with np.load(csvs[0].with_suffix(".npz"), allow_pickle=False) as archive:
            channels = json.loads(str(archive["channels"].item()))
            self.assertEqual(channels, ["S11", "S22"])
            self.assertNotIn("raw_S21_real", archive.files, "不得生成假 S21 字段")
            self.assertNotIn("trace_S12", archive.files)

    def test_autosave_failure_emits_fatal_and_stops_producer(self) -> None:
        backend, _transport = _make_backend(5)
        finished = threading.Event()
        failed_messages: list[str] = []
        worker = _make_worker(
            backend,
            count=0,
            interval_s=0.0,
            autosave=True,
            processing_queue_maxsize=64,
        )
        worker.finished_run.connect(finished.set, DIRECT)
        worker.failed.connect(failed_messages.append, DIRECT)
        with unittest.mock.patch.object(air, "save_trace", side_effect=RuntimeError("disk full")):
            worker.start()
            # 最坏路径是设计内且有界的：消费者 fatal 退出后，若生产者在
            # _stop 被观察到之前已填满队列，LosslessProcessingQueue.put 会
            # "先触发受控停止再有界阻塞等待消费者腾位"（默认 max_wait_s=30，
            # 第 1 轮 fail-closed 语义：绝不静默丢 Trace）。因此等待上限必须
            # 覆盖该设计边界，不能只给 5s（否则调度时机决定是否偶发）。
            self.assertTrue(
                finished.wait(40.0), "autosave 失败应触发 fatal 并停止 producer"
            )
        # finished_run 以 DIRECT 连接在 run() 的 finally **内部**发出：emit 时
        # QThread 尚未标记退出。直接断言 isRunning()==False 会与"emit 返回 →
        # run() 返回 → 线程退出"赛跑（间歇性假失败）。语义不变：仍要求线程
        # 真正退出，只是先做有界等待（与本文件 :578/:1199 既有写法一致）。
        self.assertTrue(worker.wait(5000), "worker 线程应在 5s 内退出")
        self.assertFalse(worker.isRunning())
        self.assertTrue(any("处理线程异常" in m or "disk full" in m for m in failed_messages))
        self.assertFalse(backend.is_open)

    def test_remote_enqueued_exactly_once_in_consumer_thread(self) -> None:
        remote = _FakeRemoteSender()

        class _IdentConsumer(air.ProcessingConsumer):
            def run(self):
                self.thread_ident = threading.get_ident()
                super().run()

        consumer = _IdentConsumer(
            queue=air.BoundedQueue(4),
            gnss_cache=air.GnssFixCache(),
            cfg=_make_selected_sweep().cfg,
            out_dir=Path(tempfile.mkdtemp()),
            autosave=False,
            calibration_p1=None,
            calibration_p2=None,
            through_measurement=None,
            background_frequency_hz=None,
            background=None,
            remote_sender=remote,
        )
        main_ident = threading.main_thread().ident
        consumer.start()
        for i in range(3):
            consumer._queue.put(_make_selected_sweep(i + 1, midpoint=100.0 + i))
        self.assertTrue(_wait_until(lambda: remote.enqueue_count >= 3, 5.0))
        consumer.request_stop()
        self.assertTrue(consumer.wait(3000))
        self.assertEqual(remote.enqueue_count, 3, "每条恰好 enqueue 一次")
        for (_args, _kwargs, ident) in remote.calls:
            self.assertEqual(ident, consumer.thread_ident, "enqueue 必须在消费者线程")
            self.assertNotEqual(ident, main_ident, "enqueue 不得在 Qt 主线程")

    def test_stop_drain_success(self) -> None:
        consumer = self._consumer()
        consumer.start()
        for i in range(3):
            consumer._queue.put(_make_selected_sweep(i + 1, midpoint=100.0 + i))
        consumer.request_stop()
        self.assertTrue(consumer.wait(3000))
        self.assertEqual(consumer.processed_count, 3)
        self.assertEqual(consumer._queue.statistics()["depth"], 0)

    def test_stop_drain_timeout_leaves_residual(self) -> None:
        consumer = self._consumer(drain_after_stop_s=0.1)
        original = consumer._process_one

        def slow(item):
            time.sleep(0.1)
            original(item)

        consumer._process_one = slow
        consumer.start()
        for i in range(5):
            consumer._queue.put(_make_selected_sweep(i + 1, midpoint=100.0 + i))
        consumer.request_stop()
        self.assertTrue(consumer.wait(3000))
        self.assertLess(consumer.processed_count, 5, "drain 超时应留下未处理项")

    def test_stop_drain_processing_failure_reported(self) -> None:
        consumer = self._consumer(drain_after_stop_s=0.5)
        failures: list[str] = []
        consumer.failed.connect(failures.append, DIRECT)
        consumer._queue.put(_make_selected_sweep(1, midpoint=100.0))

        def boom(_item):
            raise RuntimeError("processing boom")

        consumer._process_one = boom
        consumer._drain_remaining()  # 直接调用 drain：处理失败必须经 failed 上报
        self.assertEqual(len(failures), 1)
        self.assertIn("processing boom", failures[0])

    def test_trace_index_survives_queue_pressure(self) -> None:
        """第1轮检查点C：慢消费者下不丢 Trace；trace_index 与 scheduler 口径
        一致，全部选中 sweep 最终处理完成。"""
        backend, _transport = _make_backend(5)
        worker = _make_worker(
            backend,
            count=0,
            interval_s=0.0,
            processing_queue_maxsize=8,
            consumer_factory=lambda queue, cache: _TurtleConsumer(queue, cache, delay_s=0.05),
        )
        worker.start()
        self.assertTrue(
            _wait_until(lambda: worker._selected_count >= 3, 5.0),
            "即使消费者慢，也应选中并（无损）入队多条 Trace",
        )
        worker.stop()
        self.assertTrue(worker.wait(15000))
        self.assertFalse(worker.isRunning())
        # trace_index（_selected_count）与 scheduler selected 口径一致，且单调递增。
        self.assertEqual(worker._selected_count, worker.scheduler.selected_sweeps)
        self.assertGreater(worker._selected_count, 1)
        consumer = worker.consumer
        self.assertIsNotNone(consumer)
        self.assertEqual(consumer.processed_count, worker._selected_count)
        self.assertEqual(consumer.drain_state, "drained")


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class PauseStatsTests(unittest.TestCase):
    """区域六：pause/resume 保留会话累计统计，暂停时长不计入 actual_interval。"""

    def test_pause_preserves_session_statistics(self) -> None:
        backend, _transport = _make_backend(5)
        worker = _make_worker(backend, interval_s=0.05, count=0)
        worker.start()
        self.assertTrue(_wait_until(lambda: worker.scheduler.selected_sweeps >= 3, 5.0))
        selected_before = worker.scheduler.selected_sweeps
        skipped_before = worker.scheduler.statistics["skipped_sweeps"]
        max_interval_before = worker.scheduler.statistics["actual_interval_s"]["max"]
        worker.request_pause()
        self.assertTrue(_wait_until(lambda: worker.is_paused_at_boundary(), 3.0))
        time.sleep(0.3)  # 模拟暂停一段时间
        worker.request_resume()
        self.assertTrue(_wait_until(lambda: backend.state.value == "streaming", 3.0))
        # 恢复后第一条立即选择，selected 累计为 selected_before + 1。
        self.assertTrue(
            _wait_until(lambda: worker.scheduler.selected_sweeps == selected_before + 1, 3.0)
        )
        stats = worker.scheduler.statistics
        self.assertEqual(stats["selected_sweeps"], selected_before + 1)
        self.assertGreaterEqual(stats["skipped_sweeps"], skipped_before, "skipped 不得倒退")
        self.assertEqual(worker._selected_count, stats["selected_sweeps"], "worker 与 scheduler 口径一致")
        # 暂停时长不得计入 actual_interval：恢复后的第一条 actual=None，后续 max 仍接近 interval。
        self.assertLessEqual(stats["actual_interval_s"]["max"], max_interval_before + 0.1)
        worker.stop()
        self.assertTrue(worker.wait(5000))
        self.assertFalse(backend.is_open)



@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class ShutdownFailClosedTests(unittest.TestCase):
    """本轮：shutdown 的 fail-closed 语义（close 失败且资源仍打开时不结束 worker）。"""

    def test_close_raises_and_keeps_open_worker_stays_until_released(self) -> None:
        transport = _CloseKeepsOpenTransport(5)
        backend = PersistentLibreVnaBackend(transport=transport)
        finished = threading.Event()
        failed_messages: list[str] = []
        worker = _make_worker(backend, count=1)
        worker.finished_run.connect(finished.set, DIRECT)
        worker.failed.connect(failed_messages.append, DIRECT)
        worker.start()
        self.assertTrue(_wait_until(lambda: backend.state.value == "faulted", 5.0))
        time.sleep(0.3)  # 让 worker 停留在 close 重试
        self.assertTrue(worker.isRunning(), "close 失败且资源仍打开时 worker 必须保持运行")
        self.assertEqual(backend.state.value, "faulted")
        self.assertTrue(backend.is_open, "FAULTED 但仍打开的 backend 不得当作关闭成功")
        self.assertFalse(finished.is_set(), "不得提前发 finished_run")
        self.assertTrue(
            any("释放 LibreVNA 失败" in message for message in failed_messages),
            "close 失败必须明确上报",
        )
        self.assertGreaterEqual(transport.close_calls, 1)
        # close 以有限频率自动重试（非忙循环）。
        self.assertTrue(
            _wait_until(lambda: transport.close_calls >= 2, 3.0), "close 必须被自动重试"
        )
        # 允许 transport 关闭后 worker 才结束，最终无残留。
        transport.closable = True
        self.assertTrue(worker.wait(5000))
        self.assertFalse(worker.isRunning())
        self.assertFalse(backend.is_open)
        self.assertTrue(finished.is_set())
        consumer = worker.consumer
        self.assertIsNotNone(consumer)
        self.assertFalse(consumer.isRunning(), "无残留 consumer")
        self.assertIsNone(worker.gnss_thread, "本测试未启用 GNSS 线程")

    def test_silent_close_keeps_open_worker_stays_until_released(self) -> None:
        transport = _SilentCloseKeepsOpenTransport(5)
        backend = PersistentLibreVnaBackend(transport=transport)
        finished = threading.Event()
        worker = _make_worker(backend, count=1)
        worker.finished_run.connect(finished.set, DIRECT)
        worker.start()
        self.assertTrue(_wait_until(lambda: backend.state.value == "faulted", 5.0))
        time.sleep(0.3)
        self.assertTrue(worker.isRunning(), "静默 close 失败也必须保持 worker 运行")
        self.assertTrue(backend.is_open)
        self.assertFalse(finished.is_set())
        transport.closable = True
        self.assertTrue(worker.wait(5000))
        self.assertFalse(worker.isRunning())
        self.assertFalse(backend.is_open)
        self.assertTrue(finished.is_set())

    def test_close_releases_then_raises_worker_finishes(self) -> None:
        backend = PersistentLibreVnaBackend(transport=_CloseReleaseThenRaiseTransport(5))
        finished = threading.Event()
        failed_messages: list[str] = []
        worker = _make_worker(backend, count=1)
        worker.finished_run.connect(finished.set, DIRECT)
        worker.failed.connect(failed_messages.append, DIRECT)
        worker.start()
        self.assertTrue(worker.wait(5000))
        self.assertFalse(worker.isRunning())
        self.assertFalse(backend.is_open, "资源已实际释放")
        self.assertTrue(finished.is_set(), "close 抛错但资源已释放时仍可完成退出")
        self.assertTrue(
            any("释放 LibreVNA 失败" in message for message in failed_messages),
            "close 清理错误必须上报",
        )
        self.assertLessEqual(backend.transport.close_count, 2, "资源已释放，不得无限重试")

    def test_shutdown_orders_request_stop_before_backend_close(self) -> None:
        events: list[str] = []
        transport = _OrderRecordingTransport(5, events)
        backend = PersistentLibreVnaBackend(transport=transport)
        worker = _make_worker(
            backend,
            count=0,
            interval_s=0.0,
            gnss_port="COM9",
            consumer_factory=lambda queue, cache: _RecordingConsumer(queue, cache, events),
        )
        with unittest.mock.patch.object(
            air, "GnssReaderThread", lambda port, baud, cache: _RecordingGnssThread(port, baud, cache, events)
        ):
            worker.start()
            self.assertTrue(_wait_until(lambda: backend.is_open, 3.0))
            worker.stop()
            self.assertTrue(worker.wait(5000))
        self.assertFalse(worker.isRunning())
        self.assertFalse(backend.is_open)
        # 顺序：consumer/gnss 的 request_stop 必须先于可能阻塞/失败的 backend.close。
        consumer_index = events.index("consumer.request_stop")
        gnss_index = events.index("gnss.request_stop")
        close_index = events.index("backend.close")
        self.assertLess(consumer_index, close_index, "consumer.request_stop 必须先于 backend.close")
        self.assertLess(gnss_index, close_index, "gnss.request_stop 必须先于 backend.close")
        # close 被重试且最终成功：事件序列包含两次 backend.close。
        self.assertEqual(events.count("backend.close"), 2)


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class GroundStationDualChannelTests(unittest.TestCase):
    """地面端交叉验证：连续双通道 payload 可被 GroundStationWindow.on_trace 显示。"""

    @classmethod
    def setUpClass(cls) -> None:
        from PySide6 import QtWidgets

        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_dual_channel_payload_displays_in_ground_station(self) -> None:
        try:
            import uav_gpr_ground_station as gs
        except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
            self.skipTest(f"uav_gpr_ground_station unavailable: {exc}")
        # 生成真实双通道 payload（ProcessingConsumer 使用的同一保存边界）。
        item = _make_selected_sweep(1, midpoint=100.0)
        raw = air.dual_channel_raw(item.raw)
        display, time_ns, traces = air.process_trace_pipeline(item.frequency_hz, raw, item.cfg)
        self.assertEqual(sorted(traces.keys()), ["S11", "S22"])
        fix = air.GgaFix(fix_quality=1, latitude=30.5, longitude=114.3)
        payload = air.trace_npz_bytes(
            1, item.cfg, item.frequency_hz, raw, display, time_ns, traces, fix,
            channels=("S11", "S22"),
        )
        out_dir = Path(tempfile.mkdtemp())
        window = gs.GroundStationWindow("127.0.0.1", 0, out_dir)
        logs: list[str] = []
        try:
            window.receiver.stop()  # 不启动真实网络监听
            window.log = lambda text: logs.append(text)
            before_s11 = len(window.bscan_rows["S11"])
            before_s22 = len(window.bscan_rows["S22"])
            header = {
                "type": "trace",
                "protocol": "uav-gpr-telemetry-v1",
                "run_id": "cross-check",
                "trace_index": 1,
                "timestamp": "2026-08-14T00:00:00",
            }
            window.on_trace(header, payload, "remote_trace.npz")
            self.assertEqual(len(window.bscan_rows["S11"]), before_s11 + 1, "HH 应增加一道")
            self.assertEqual(len(window.bscan_rows["S22"]), before_s22 + 1, "VV 应增加一道")
            self.assertFalse(
                any("Trace ignored" in text for text in logs),
                f"地面端不得忽略真实双通道 payload: {logs}",
            )
        finally:
            window.receiver.stop()
            window.receiver.wait(2000)
            window.close()
            window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
            self.app.processEvents()



@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class RemoteSenderStopTests(unittest.TestCase):
    """RemoteSender.stop() 有界 join：退避中快速退出、幂等、未 start 安全、不再回调。"""

    def test_stop_during_retry_backoff_exits_quickly(self) -> None:
        calls: list[str] = []
        sender = air.RemoteSender("127.0.0.1", 1, "run-test", calls.append)
        with unittest.mock.patch.object(sender, "_connect", side_effect=ConnectionError("no ground")):
            sender.start()
            self.assertTrue(_wait_until(lambda: any("retrying" in c for c in calls), 3.0))
            start = time.monotonic()
            exited = sender.stop(3000)
            elapsed = time.monotonic() - start
        self.assertTrue(exited, "stop 应确认线程已退出")
        self.assertFalse(sender._thread.is_alive(), "stop 后不得残留 sender 线程")
        self.assertLess(elapsed, 1.5, f"不应等待完整 2 秒退避，实际 {elapsed:.2f}s")

    def test_stop_is_idempotent(self) -> None:
        sender = air.RemoteSender("127.0.0.1", 1, "run-test", lambda message: None)
        with unittest.mock.patch.object(sender, "_connect", side_effect=ConnectionError("no ground")):
            sender.start()
            self.assertTrue(sender.stop(3000))
        self.assertTrue(sender.stop(3000), "重复 stop 幂等且不抛错")
        self.assertFalse(sender._thread.is_alive())

    def test_stop_without_start_is_safe(self) -> None:
        sender = air.RemoteSender("127.0.0.1", 1, "run-test", lambda message: None)
        self.assertTrue(sender.stop(500), "未启动的 sender stop 不得抛错，应视为已退出")
        self.assertFalse(sender._thread.is_alive())

    def test_stop_does_not_call_status_callback_after_exit(self) -> None:
        calls: list[str] = []
        sender = air.RemoteSender("127.0.0.1", 1, "run-test", calls.append)
        with unittest.mock.patch.object(sender, "_connect", side_effect=ConnectionError("no ground")):
            sender.start()
            self.assertTrue(_wait_until(lambda: any("retrying" in c for c in calls), 3.0))
            before = len(calls)
            self.assertTrue(sender.stop(3000))
            time.sleep(0.3)  # 给潜在的异步回调留一点时间
        self.assertEqual(len(calls), before, "stop 后不得再调用已销毁窗口的 status callback")



@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class PythonThreadResidualTests(unittest.TestCase):
    """只负责检查 Python ``threading.Thread`` 残留（如 uav-gpr-remote-sender）。

    Qt ``QThread``（ContinuousAcquisitionWorker / ProcessingConsumer /
    GnssReaderThread）是原生线程，**不出现在** ``threading.enumerate()`` 中；
    它们由各持有对象的测试在清理末尾断言 ``isRunning() is False`` 验证，并经
    独立子进程干净退出（无 QThread destroyed / access violation）辅助确认。
    """

    def test_no_residual_remote_sender_python_thread(self) -> None:
        residual = [
            t.name
            for t in threading.enumerate()
            if t.name == "uav-gpr-remote-sender" and t.is_alive()
        ]
        self.assertEqual(residual, [], f"残留 uav-gpr-remote-sender 活线程: {residual}")


def _contains_callable(obj) -> bool:
    """递归检查对象是否含 callable（bound method 等），用于序列化回归。"""
    if callable(obj):
        return True
    if isinstance(obj, dict):
        return any(_contains_callable(v) for v in obj.values())
    if isinstance(obj, (list, tuple)):
        return any(_contains_callable(v) for v in obj)
    return False


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class WorkerStatisticsSerializationTests(unittest.TestCase):
    """阶段 5 发现的冻结缺陷回归：worker.statistics() 必须可 JSON 序列化。

    缺陷：statistics() 把 bound method ``processing_queue.statistics`` 放进
    session_statistics，autosave/遥测 JSON 序列化抛
    "Object of type method is not JSON serializable"。本类验证修复后：
    processing_queue 是普通 dict、无 callable、json.dumps 通过、真实
    worker→consumer→save_trace 路径无需 workaround 即可保存、遥测 metadata 可序列化。
    """

    def test_worker_statistics_processing_queue_is_dict(self) -> None:
        backend, _transport = _make_backend(5)
        worker = _make_worker(backend, count=3)
        worker.start()
        try:
            self.assertTrue(worker.wait(5000))
            stats = worker.statistics()
            self.assertIsInstance(stats["processing_queue"], dict)
            self.assertIn("depth", stats["processing_queue"])
            self.assertIn("high_water_events", stats["processing_queue"])
        finally:
            worker.wait(1000)

    def test_worker_statistics_no_callables_and_json_serializable(self) -> None:
        backend, _transport = _make_backend(5)
        worker = _make_worker(backend, count=3)
        worker.start()
        try:
            self.assertTrue(worker.wait(5000))
            stats = worker.statistics()
            self.assertFalse(_contains_callable(stats), "statistics() 不得含 callable")
            json.dumps(stats)  # 不得抛 TypeError
        finally:
            worker.wait(1000)

    def test_worker_autosave_path_saves_without_workaround(self) -> None:
        backend, _transport = _make_backend(5)
        worker = _make_worker(backend, count=3, autosave=True)
        worker.start()
        try:
            self.assertTrue(worker.wait(5000))
            out = Path(worker.out_dir)
            csvs = sorted(out.glob("trace_*.csv"))
            npzs = sorted(out.glob("trace_*.npz"))
            self.assertGreaterEqual(len(csvs), 1, "autosave 应产出 CSV")
            self.assertGreaterEqual(len(npzs), 1, "autosave 应产出 NPZ")
            self.assertEqual(len(csvs), len(npzs), "CSV 与 NPZ 应成对")
            # NPZ 内 acquisition metadata 必须可解析、可 JSON 序列化。
            data = np.load(npzs[0], allow_pickle=False)
            raw = data["acquisition"]
            raw = raw.decode("utf-8") if isinstance(raw, bytes) else raw
            acquisition = json.loads(str(raw))
            session = acquisition["session_statistics"]
            self.assertIsInstance(session["processing_queue"], dict)
            json.dumps(acquisition)
        finally:
            worker.wait(1000)

    def test_telemetry_metadata_serializable(self) -> None:
        backend, _transport = _make_backend(5)
        worker = _make_worker(backend, count=1)
        worker.start()
        try:
            self.assertTrue(worker.wait(5000))
            session = worker.statistics()
            json.dumps(session)
            item = air.SelectedSweep(
                trace_index=1,
                device_sweep_index=0,
                frequency_hz=np.array([1.0e9, 2.0e9]),
                raw=np.zeros((2, 2), dtype=np.complex128),
                cfg=worker.cfg,
                midpoint_monotonic=time.monotonic(),
                selected_at_utc="t",
                missed_slots=0,
                actual_interval_s=None,
                schedule_error_ms=None,
                acquired_at_utc="t",
                sweep_started_utc="t",
                sweep_completed_utc="t",
                scheduled_at_utc=None,
                connection_generation=1,
                target_interval_s=0.0,
                session_statistics=session,
            )
            lookup = air.GnssFixLookup(None, None, False)
            acquisition = air.ProcessingConsumer._acquisition_metadata(item, lookup)
            flight = air.ProcessingConsumer._flight_metadata(lookup, air.GgaFix())
            json.dumps(acquisition)
            json.dumps(flight)
            self.assertIsInstance(acquisition["session_statistics"]["processing_queue"], dict)
        finally:
            worker.wait(1000)


if __name__ == "__main__":
    unittest.main()