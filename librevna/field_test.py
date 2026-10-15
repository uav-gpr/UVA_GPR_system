"""Headless field-test harness for the continuous LibreVNA backend (roadmap §16 阶段5).

Runs the phase-1..4 continuous-acquisition components end-to-end either against
a real LibreVNA over USB (explicit ``--hardware``) or against a simulated
streaming transport (explicit ``--simulate``).  **执行模式必须显式二选一**：
``--simulate`` 或 ``--hardware``，缺一或冲突即拒绝且不构造任何 transport、不枚举
USB；不存在"默认真机"。Subcommands:

``probe``
    Confirm device / firmware / protocol, ``0x1241`` SWEEP_SETTINGS ACK,
    same-packet S11/S22 and the strict ``0..N-1`` point loop on a sample of
    sweeps (roadmap §15.6 items 1-3).

``bench``
    Layered benchmark (roadmap §15.6 item 4): ``usb`` -> ``usb,proc`` ->
    ``usb,proc,disk`` -> ``usb,proc,disk,net``.  ``usb`` alone is a direct
    synchronous pure-USB baseline.  Any layer including ``proc`` runs the
    **production threaded topology** by default
    (``ContinuousAcquisitionWorker`` + ``BoundedQueue`` + ``ProcessingConsumer``),
    so the USB producer thread only acquire/select/put while processing, saving
    and telemetry compression happen in the consumer thread.  ``--mode sync``
    opts into an explicitly-labelled synchronous component microbenchmark that
    is **not** an acceptance result.

``longrun``
    Stability run (roadmap §15.6 item 5): production threaded mode only, with
    periodic sampling and a final summary.  Field procedure asks for >= 30 min,
    then 1-2 h when possible.

``failures``
    Failure-mode scenarios (roadmap §15.6 items 6-8): pause/resume (never
    reopen), ``gnss_cache_no_fix_stale`` (GnssFixCache + metadata semantics only
    -- **not** a serial-port disconnect test), and a dropped network link while
    local acquisition and saving continue.

Exit-code contract (machine-decidable):

- Every report JSON contains ``passed``, ``failure_reasons`` and ``cleanup``.
- ``rc == 0`` only when every required invariant passes **and** cleanup succeeds
  (backend released, sender/receiver/feeder/monitor/producer threads exited).
- Integrity failures, zero valid data, save/network count mismatches, cleanup
  failures and thread residuals all return non-zero with explicit
  ``failure_reasons``.
- ``acceptance_eligible`` is True only for real-hardware runs (``--simulate`` is
  never eligible) that pass every invariant; ``longrun`` additionally requires
  duration >= 1800 s.  A short smoke run may return ``rc=0`` with
  ``acceptance_eligible=false``.

Only the continuous backend is exercised.  The legacy backend and the
``continuous-not-default`` feature flag (``UAV_GPR_ACQUISITION_MODE``) are not
touched.  Real USB hot-unplug cannot be performed by this tool and must be
executed by field personnel; the simulation only exercises the software
resynchronisation behaviour.  阶段 5 正式现场验收未完成：真实设备曾被一次越权
probe 意外访问，但 ≥30 分钟长稳、USB 拔插、真实 GNSS/TCP、1–2 小时压力及人工
资源判定尚未获授权或尚未执行。
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import numbers
import os
import random
import socket
import struct
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from librevna.librevna_continuous import (
    BackendState,
    PersistentBackendSettings,
    PersistentLibreVnaBackend,
)
from librevna.librevna_errors import (
    LibreVnaDisconnectedError,
    LibreVnaError,
    LibreVnaStateError,
    LibreVnaSweepError,
    LibreVnaTimeoutError,
)
from librevna.librevna_protocol import (
    ACK,
    DEVICE_INFO,
    NACK,
    REQUEST_DEVICE_INFO,
    SET_IDLE,
    S11_S22_STAGES_BITMAP,
    SWEEP_SETTINGS,
    VNA_DATAPOINT,
    PacketStream,
    SweepSettings,
    decode_device_info,
    encode_packet,
    parse_s11_s22_point,
)
from librevna.librevna_transport import LibreVnaUsbTransport

import air_core as U  # production pipeline functions (imports PyQt5, headless-safe)

DEFAULT_POINTS = 201
DEFAULT_START_MHZ = 400.0
DEFAULT_STOP_MHZ = 4000.0
DEFAULT_IFBW_HZ = 10_000
DEFAULT_RATE_SWEEPS_PER_SEC = 50.0
DEFAULT_GNSS_MAX_AGE_S = 5.0
#: roadmap §14.8 Stop 到 USB 释放目标。
USB_RELEASE_TARGET_S = 2.5
#: longrun 验收最短时长（roadmap §15.6.5：至少 30 分钟）。
LONGRUN_ACCEPTANCE_MIN_S = 1800.0
#: bench 验收最短时长（非 smoke）。
BENCH_ACCEPTANCE_MIN_S = 60.0
#: 90% 比较的纯 USB 基线最短时长（roadmap §14.6 同参数基线）。
BASELINE_MIN_S = 60.0
#: 生产 run 的 worker 最长等待余量。
WORKER_WAIT_GRACE_S = 40.0
#: stop 后等待 USB 释放的硬上限。
RELEASE_POLL_TIMEOUT_S = 30.0


# =====================================================================
# 统计与进程资源（无第三方依赖，固定低频采样）
# =====================================================================


def _percentile(sorted_values: list[float], fraction: float) -> Optional[float]:
    if not sorted_values:
        return None
    index = fraction * (len(sorted_values) - 1)
    low = int(math.floor(index))
    high = int(math.ceil(index))
    if low == high:
        return sorted_values[low]
    weight = index - low
    return sorted_values[low] * (1.0 - weight) + sorted_values[high] * weight


def _sample_stats(values: list[float]) -> dict[str, object]:
    if not values:
        return {"count": 0, "min": None, "mean": None, "p95": None, "max": None}
    sorted_values = sorted(values)
    return {
        "count": len(sorted_values),
        "min": sorted_values[0],
        "mean": sum(sorted_values) / len(sorted_values),
        "p95": _percentile(sorted_values, 0.95),
        "max": sorted_values[-1],
    }


def _mono() -> float:
    """高分辨率 monotonic 秒（Windows 上亚毫秒精度）。"""
    return time.perf_counter_ns() / 1_000_000_000.0


_WIN_PSAPI_CACHE: Optional[tuple] = None


def _process_rss_mb() -> float:
    """当前进程工作集（MB）。Windows 用 psapi（句柄缓存，避免每道重复加载 DLL）。"""
    global _WIN_PSAPI_CACHE
    if sys.platform.startswith("win"):
        if _WIN_PSAPI_CACHE is None:
            import ctypes

            class _PMC(ctypes.Structure):  # PROCESS_MEMORY_COUNTERS（完整布局）
                _fields_ = [
                    ("cb", ctypes.c_ulong),
                    ("PageFaultCount", ctypes.c_ulong),
                    ("PeakWorkingSetSize", ctypes.c_size_t),
                    ("WorkingSetSize", ctypes.c_size_t),
                    ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                    ("PagefileUsage", ctypes.c_size_t),
                    ("PeakPagefileUsage", ctypes.c_size_t),
                ]

            psapi = ctypes.WinDLL("psapi", use_last_error=True)
            kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
            psapi.GetProcessMemoryInfo.argtypes = [
                ctypes.c_void_p,
                ctypes.c_void_p,
                ctypes.c_ulong,
            ]
            psapi.GetProcessMemoryInfo.restype = ctypes.c_int
            kernel32.GetCurrentProcess.restype = ctypes.c_void_p
            _WIN_PSAPI_CACHE = (_PMC, psapi, kernel32)
        _PMC, psapi, kernel32 = _WIN_PSAPI_CACHE
        counters = _PMC()
        counters.cb = ctypes.sizeof(_PMC)
        handle = kernel32.GetCurrentProcess()
        if not psapi.GetProcessMemoryInfo(
            handle, ctypes.byref(counters), ctypes.sizeof(counters)
        ):
            raise OSError(ctypes.get_last_error(), "GetProcessMemoryInfo failed")
        return counters.WorkingSetSize / (1024.0 * 1024.0)
    import resource

    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


class _RunMonitor(threading.Thread):
    """固定低频（默认 0.5s）采样 CPU%/RSS 与队列深度，不逐道调用 psapi。

    采样线程异常一律吞掉并继续；``request_stop()`` 必须让线程尽快退出。
    """

    def __init__(self, interval_s: float = 0.5) -> None:
        super().__init__(name="field-test-monitor", daemon=True)
        self.interval_s = interval_s
        self._stop_event = threading.Event()
        self._cpu_samples: list[float] = []
        self._rss_samples: list[float] = []
        self.sender_qsize_samples: list[int] = []
        self.processing_depth_samples: list[int] = []
        self.sender = None
        self.processing_queue = None
        self._last_cpu: Optional[float] = None
        self._last_wall: Optional[float] = None

    def request_stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        while not self._stop_event.wait(self.interval_s):
            self._sample()

    def _sample(self) -> None:
        try:
            times = os.times()
            now = time.monotonic()
            cpu_total = times.user + times.system
            if self._last_cpu is not None and self._last_wall is not None:
                dt_wall = now - self._last_wall
                dt_cpu = cpu_total - self._last_cpu
                if dt_wall > 0.0:
                    self._cpu_samples.append(min(100.0 * dt_cpu / dt_wall, 100.0))
            self._last_cpu = cpu_total
            self._last_wall = now
        except Exception:  # noqa: BLE001 - 采样不得抛出
            self._last_cpu = None
        try:
            self._rss_samples.append(_process_rss_mb())
        except Exception:  # noqa: BLE001
            pass
        if self.sender is not None:
            try:
                self.sender_qsize_samples.append(self.sender.queue.qsize())
            except Exception:  # noqa: BLE001
                pass
        if self.processing_queue is not None:
            try:
                stats = self.processing_queue.statistics()
                self.processing_depth_samples.append(int(stats["depth"]))
            except Exception:  # noqa: BLE001
                pass

    def result(self) -> dict[str, object]:
        sender_depth = self.sender_qsize_samples
        proc_depth = self.processing_depth_samples
        return {
            "cpu_percent": _sample_stats(self._cpu_samples),
            "rss_mb": _sample_stats(self._rss_samples),
            "sender_queue_depth": {
                "samples": len(sender_depth),
                "current": sender_depth[-1] if sender_depth else None,
                "peak": max(sender_depth) if sender_depth else 0,
            },
            "processing_queue_depth": {
                "samples": len(proc_depth),
                "current": proc_depth[-1] if proc_depth else None,
                "peak": max(proc_depth) if proc_depth else 0,
            },
        }


# =====================================================================
# 模拟传输（真实 backend 全链路，无需硬件）
# =====================================================================


def _device_info_payload() -> bytes:
    """与 tests/test_librevna_backend.py 相同的合法 DeviceInfo（固件 1.2.3）。"""
    return struct.pack(
        "<HBBBBcQQIIHhhIIBQBH",
        14, 1, 2, 3, 5, b"A",
        100_000_000, 6_000_000_000, 1, 1_000_000, 10_001,
        -3000, 1000, 1, 1_000_000, 200, 12_000_000_000, 2, 65_535,
    )


def _point_payload(freq_hz: float, point_number: int, s11: complex, s22: complex) -> bytes:
    """6-receiver 双端口点（真机 descriptor 顺序），S11/S22 同包。"""
    receivers = [
        (0x01, s11),
        (0x02, complex(9.0, 9.0)),
        (0x13, complex(1.0, 0.0)),
        (0x21, complex(7.0, 7.0)),
        (0x22, s22),
        (0x33, complex(1.0, 0.0)),
    ]
    n = len(receivers)
    payload = struct.pack("<QhH", int(freq_hz), -1000, int(point_number))
    payload += struct.pack("<" + "f" * n, *[v.real for _, v in receivers])
    payload += struct.pack("<" + "f" * n, *[v.imag for _, v in receivers])
    payload += bytes(d for d, _ in receivers)
    return payload


class StreamingSimulatedTransport:
    """以目标 sweep 速率持续产生严格 ``0..N-1`` VNADatapoint 流的模拟设备。

    实现 :class:`LibreVnaTransportProtocol`（``is_open`` 为 **property**、
    ``open()``/``close()`` 幂等且只维护一个 producer 线程）。命令响应
    （DEVICE_INFO / ACK / NACK）与连续扫频数据都经同一内部缓冲与条件变量，
    backend 的 PacketStream / pending / 超时逻辑被真实驱动。

    构造参数严格校验：points 必须为 >=2 的 int（拒绝 bool）；start_hz/stop_hz
    必须为有限正数且 start < stop；sweeps_per_sec 必须为有限正数。
    """

    def __init__(
        self,
        *,
        points: int = DEFAULT_POINTS,
        start_hz: float = DEFAULT_START_MHZ * 1e6,
        stop_hz: float = DEFAULT_STOP_MHZ * 1e6,
        sweeps_per_sec: float = DEFAULT_RATE_SWEEPS_PER_SEC,
        seed: int = 7,
    ) -> None:
        if isinstance(points, bool) or not isinstance(points, int) or points < 2:
            raise ValueError(f"points 必须是 >=2 的 int（bool 被拒绝），实际 {points!r}")
        for name, value in (("start_hz", start_hz), ("stop_hz", stop_hz)):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{name} 必须是实数，实际 {type(value).__name__}")
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} 必须是有限正数，实际 {value!r}")
        if not float(start_hz) < float(stop_hz):
            raise ValueError(f"start_hz 必须 < stop_hz，实际 {start_hz} >= {stop_hz}")
        if isinstance(sweeps_per_sec, bool) or not isinstance(sweeps_per_sec, (int, float)):
            raise TypeError("sweeps_per_sec 必须是实数，实际 {type(sweeps_per_sec).__name__}")
        if not math.isfinite(float(sweeps_per_sec)) or float(sweeps_per_sec) <= 0.0:
            raise ValueError(f"sweeps_per_sec 必须是有限正数，实际 {sweeps_per_sec!r}")
        self._points = points
        self._freqs = np.linspace(start_hz, stop_hz, points)
        self._rate = float(sweeps_per_sec)
        # 每个点的时间 = 每道时长 / 点数 = (1/rate) / points。
        self._point_period_s = 1.0 / max(self._rate, 1e-6) / points
        self._rng = random.Random(seed)
        self._open = False
        self._streaming = False
        self._disconnected = False
        self._next_command_nack = False
        self._buf = bytearray()
        self._cond = threading.Condition()
        self._stop_producer = threading.Event()
        self._producer: Optional[threading.Thread] = None
        self._sweep_no = 0
        self._point_no = 0
        self._writes: list[int] = []
        self.produced_points = 0
        self.open_count = 0
        self.close_count = 0

    # ---- 生命周期（幂等）----

    def open(self) -> None:
        """幂等：重复 open 不创建第二个 producer。"""
        self._open = True
        self._disconnected = False
        self._stop_producer.clear()
        self.open_count += 1
        if self._producer is None or not self._producer.is_alive():
            self._producer = threading.Thread(
                target=self._produce, name="field-test-sim-usb", daemon=True
            )
            self._producer.start()

    def close(self) -> None:
        """幂等：确认 producer 已退出，不丢失旧 producer 引用。"""
        self._open = False
        self._streaming = False
        self._stop_producer.set()
        with self._cond:
            self._cond.notify_all()
        thread = self._producer
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self.close_count += 1

    @property
    def is_open(self) -> bool:
        return self._open and not self._disconnected

    def producer_alive(self) -> bool:
        thread = self._producer
        return thread is not None and thread.is_alive()

    # ---- 命令 / 数据 ----

    def write(self, data: bytes) -> None:
        if not self._open:
            raise LibreVnaError("simulated transport not open")
        packet_type = self._decode_packet_type(data)
        self._writes.append(packet_type)
        with self._cond:
            if packet_type == REQUEST_DEVICE_INFO:
                self._buf.extend(encode_packet(DEVICE_INFO, _device_info_payload()))
            elif packet_type == SWEEP_SETTINGS:
                if self._next_command_nack:
                    self._next_command_nack = False
                    self._buf.extend(encode_packet(NACK))
                else:
                    # 真机收到新的 SWEEP_SETTINGS（含 resume 重发）后从点 0 重新
                    # 开始扫频；清掉暂停前残留的点、重置点号，避免 resume 后
                    # 首个非零点被 assembler 计为 dropped。
                    self._buf.clear()
                    self._point_no = 0
                    self._buf.extend(encode_packet(ACK))
                    self._streaming = True
            elif packet_type == SET_IDLE:
                if self._next_command_nack:
                    self._next_command_nack = False
                    self._buf.extend(encode_packet(NACK))
                else:
                    self._buf.extend(encode_packet(ACK))
                    self._streaming = False
            else:
                self._buf.extend(encode_packet(NACK))
            self._cond.notify_all()

    def read(self, max_length: int, timeout_ms: int) -> bytes:
        deadline = time.monotonic() + timeout_ms / 1000.0
        with self._cond:
            while not self._buf and time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._cond.wait(min(remaining, 0.05))
            if self._disconnected:
                raise LibreVnaDisconnectedError("simulated USB unplugged")
            if not self._buf:
                raise LibreVnaTimeoutError(f"simulated read timeout after {timeout_ms} ms")
            chunk = bytes(self._buf[:max_length])
            del self._buf[:max_length]
            return chunk

    # ---- 故障注入（供 failures 场景）----

    def inject_nack_once(self) -> None:
        self._next_command_nack = True

    def inject_disconnect(self) -> None:
        with self._cond:
            self._disconnected = True
            self._streaming = False
            self._buf.clear()
            self._cond.notify_all()

    def inject_stall(self, seconds: float) -> None:
        """暂停产出指定时长（用于触发单次 sweep 超时后重同步）。"""
        with self._cond:
            self._streaming = False
            self._cond.notify_all()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._stop_producer.wait(0.05)
            if self._stop_producer.is_set():
                return
        with self._cond:
            if self._open:
                self._streaming = True
                self._cond.notify_all()

    # ---- 内部 ----

    def _decode_packet_type(self, data: bytes) -> int:
        if data and data[0] == 0x5A and len(data) >= 4:
            return data[3]
        return -1

    def _produce(self) -> None:
        """按目标速率产出严格 0..N-1 循环的点流。

        Windows 系统计时器粒度约 15.6ms，逐点睡眠无法精确节流。改为按
        ``tick_s`` 批量产点并自校正：每 tick 产出覆盖该时长的点数，再睡一个
        tick，使**平均**速率接近目标（抖动约一个 tick）。点间隔 >= tick 的
        慢速率则逐点节流。
        """
        point_period = self._point_period_s
        tick_s = 0.015
        if point_period < tick_s:
            batch = max(1, int(round(tick_s / point_period)))
            sleep_s = tick_s
        else:
            batch = 1
            sleep_s = point_period
        next_tick = time.perf_counter() + sleep_s
        while not self._stop_producer.is_set():
            with self._cond:
                streaming = self._streaming
                if streaming:
                    for _ in range(batch):
                        packet = encode_packet(VNA_DATAPOINT, self._next_point())
                        self._buf.extend(packet)
                    self._cond.notify_all()
            if not streaming:
                self._stop_producer.wait(0.01)
                next_tick = time.perf_counter() + sleep_s
                continue
            delay = next_tick - time.perf_counter()
            while delay > 0.0 and not self._stop_producer.is_set():
                self._stop_producer.wait(min(delay, 0.01))
                delay = next_tick - time.perf_counter()
            next_tick += sleep_s

    def _next_point(self) -> bytes:
        freq = float(self._freqs[self._point_no])
        point = self._point_no
        base = complex(0.5 + 0.001 * point, -0.2)
        s11 = base + complex(self._rng.uniform(-0.001, 0.001), self._rng.uniform(-0.001, 0.001))
        s22 = complex(0.3, 0.1 - 0.001 * point) + complex(
            self._rng.uniform(-0.001, 0.001), self._rng.uniform(-0.001, 0.001)
        )
        self._point_no += 1
        self.produced_points += 1
        if self._point_no >= self._points:
            self._point_no = 0
            self._sweep_no += 1
        return _point_payload(freq, point, s11, s22)


class TappingTransport:
    """包装任何 transport，从 read() 字节流中记录前若干 VNADatapoint 解析结果。

    用于 ``probe``：不依赖 backend 内部状态，独立验证真实/模拟设备的
    S11/S22 同包解析与逐点序号。``is_open`` 为 property，委托内层。
    """

    def __init__(
        self,
        inner,
        *,
        max_records: int,
        on_point: Callable[[tuple[int, float, complex, complex]], None],
    ) -> None:
        self._inner = inner
        self._max_records = max_records
        self._on_point = on_point
        self._stream = PacketStream()
        self._count = 0

    def open(self) -> None:
        self._inner.open()

    def close(self) -> None:
        self._inner.close()

    @property
    def is_open(self) -> bool:
        return self._inner.is_open

    def write(self, data: bytes) -> None:
        self._inner.write(data)

    def read(self, max_length: int, timeout_ms: int) -> bytes:
        data = self._inner.read(max_length, timeout_ms)
        if self._count < self._max_records:
            for packet in self._stream.feed(data):
                if packet.packet_type == VNA_DATAPOINT:
                    try:
                        parsed = parse_s11_s22_point(packet.payload)
                    except struct.error:
                        parsed = None
                    if parsed is not None:
                        self._on_point(parsed)
                        self._count += 1
                        if self._count >= self._max_records:
                            break
        return data


# =====================================================================
# 模拟 GNSS 馈送
# =====================================================================


class SimGnssFeeder(threading.Thread):
    """按周期向 GnssFixCache 馈送 Fix，模拟三种现场 GNSS 状态。"""

    def __init__(
        self,
        cache: U.GnssFixCache,
        mode: str,
        *,
        period_s: float = 1.0,
        max_age_s: float = DEFAULT_GNSS_MAX_AGE_S,
    ) -> None:
        super().__init__(name="field-test-sim-gnss", daemon=True)
        if mode not in ("valid", "no_fix", "stale", "none"):
            raise ValueError(f"gnss mode 必须是 valid/no_fix/stale/none，实际得到 {mode!r}")
        self._cache = cache
        self._mode = mode
        self._period_s = period_s
        self._max_age_s = max_age_s
        self._stop_event = threading.Event()
        self.updates_attempted = 0
        self.updates_accepted = 0

    def request_stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        if self._mode == "none":
            return
        first = True
        while not self._stop_event.wait(self._period_s):
            self.updates_attempted += 1
            if self._mode == "valid" or (self._mode == "stale" and first):
                fix = U.GgaFix(
                    raw="valid",
                    utc="00:00:00",
                    latitude=30.123456,
                    longitude=120.654321,
                    fix_quality=1,
                    satellites=10,
                    altitude_m=100.5,
                )
            else:
                fix = U.GgaFix(
                    raw="", utc="", latitude=None, longitude=None,
                    fix_quality=0, satellites=0, altitude_m=None,
                )
            first = False
            if self._cache.update(fix):
                self.updates_accepted += 1


# =====================================================================
# 网络回环接收端
# =====================================================================


class LoopbackReceiver(threading.Thread):
    """本地回环接收端：hello->ack、trace->ack_trace，统计收到的 trace。"""

    def __init__(self, host: str = "127.0.0.1") -> None:
        super().__init__(name="field-test-loopback-rx", daemon=True)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, 0))
        self._sock.listen(1)
        self.port = self._sock.getsockname()[1]
        self._stop_event = threading.Event()
        self.trace_indexes: list[int] = []
        self._conn: Optional[socket.socket] = None

    def request_stop(self) -> None:
        self._stop_event.set()

    @property
    def received_traces(self) -> int:
        return len(self.trace_indexes)

    def trace_stats(self) -> dict[str, object]:
        indexes = self.trace_indexes
        if not indexes:
            return {
                "received": 0,
                "unique": 0,
                "duplicates": 0,
                "missing": 0,
                "out_of_order": 0,
                "max_index": None,
            }
        unique = len(set(indexes))
        min_index = min(indexes)
        max_index = max(indexes)
        present = set(indexes)
        missing = sum(1 for i in range(min_index, max_index + 1) if i not in present)
        out_of_order = sum(1 for a, b in zip(indexes, indexes[1:]) if b < a)
        return {
            "received": len(indexes),
            "unique": unique,
            "duplicates": len(indexes) - unique,
            "missing": missing,
            "out_of_order": out_of_order,
            "max_index": max_index,
        }

    def run(self) -> None:
        self._sock.settimeout(0.2)
        conn: Optional[socket.socket] = None
        try:
            while not self._stop_event.is_set():
                try:
                    conn, _addr = self._sock.accept()
                    break
                except socket.timeout:
                    continue
            if conn is None:
                return
            self._conn = conn
            conn.settimeout(0.2)
            while not self._stop_event.is_set():
                try:
                    header, _payload = U.recv_remote_frame(conn)
                except socket.timeout:
                    continue
                except OSError:
                    break
                msg_type = header.get("type")
                if msg_type == "hello":
                    U.send_remote_frame(
                        conn,
                        {
                            "type": "ack",
                            "protocol": "uav-gpr-telemetry-v1",
                            "run_id": header.get("run_id"),
                            "timestamp": U.utc_timestamp(),
                        },
                    )
                elif msg_type == "trace":
                    index = header.get("trace_index")
                    if isinstance(index, int):
                        self._record_trace(index)
                    U.send_remote_frame(
                        conn,
                        {
                            "type": "ack_trace",
                            "protocol": "uav-gpr-telemetry-v1",
                            "trace_index": index,
                            "timestamp": U.utc_timestamp(),
                        },
                    )
                elif msg_type == "command":
                    U.send_remote_frame(
                        conn,
                        {
                            "type": "command_ack",
                            "protocol": "uav-gpr-telemetry-v1",
                            "action": header.get("action"),
                            "timestamp": U.utc_timestamp(),
                        },
                    )
        finally:
            if conn is not None:
                try:
                    conn.close()
                except OSError:
                    pass

    def _record_trace(self, index: int) -> None:
        """记录收到的 trace index（供故障注入子类覆写）。"""
        self.trace_indexes.append(index)

    def close(self) -> None:
        self.request_stop()
        try:
            self._sock.close()
        except OSError:
            pass
        self.join(timeout=2.0)


def _safe_probe(
    failures: list[str], description: str, read: Callable[[], bool], default: bool = True
) -> tuple[bool, bool]:
    """状态读取抛错时 fail-closed：按"未安全退出"（default=True 即视为仍活跃）处理。

    返回 ``(value, ok)``：ok=False 表示读取抛错（原始异常类型+消息已写入 failures）。
    """
    try:
        return bool(read()), True
    except Exception as exc:  # noqa: BLE001 - 状态探测抛错不得逃出
        failures.append(
            f"{description} 读取抛错（按未安全退出处理）: {type(exc).__name__}: {exc}"
        )
        return default, False


def _safe_stop_call(
    failures: list[str], role: str, operation: str, fn: Callable, default: object = None
) -> tuple[object, bool]:
    """清理操作（request_stop/join/wait 等）调用；抛错追加结构化诊断，绝不逃出。

    返回 ``(value, ok)``；failure 至少含 角色.操作() 抛错: 类型: 消息。
    """
    try:
        return fn(), True
    except Exception as exc:  # noqa: BLE001 - 清理调用抛错不得逃出
        failures.append(f"{role}.{operation}() 抛错: {type(exc).__name__}: {exc}")
        return default, False


def _stop_thread(
    thread: Optional[threading.Thread], name: str, failures: list[str]
) -> tuple[bool, bool]:
    """有界等待线程退出；返回 ``(exited, ok)``。未启动视为已退出。

    ident/join/is_alive 抛错都按未退出处理并写入结构化诊断（ok=False）。
    """
    if thread is None:
        return True, True
    try:
        ident = thread.ident
    except Exception as exc:  # noqa: BLE001 - 状态探测抛错不得逃出
        failures.append(f"{name}.ident 读取抛错（按未安全退出处理）: {type(exc).__name__}: {exc}")
        return False, False
    if ident is None:
        return True, True
    try:
        thread.join(timeout=2.0)
    except Exception as exc:  # noqa: BLE001 - 清理异常不得逃出
        failures.append(f"{name}.join() 抛错: {type(exc).__name__}: {exc}")
        return False, False
    try:
        alive = bool(thread.is_alive())
    except Exception as exc:  # noqa: BLE001 - 状态探测抛错不得逃出
        failures.append(f"{name}.is_alive() 读取抛错（按未安全退出处理）: {type(exc).__name__}: {exc}")
        return False, False
    return not alive, True


# =====================================================================
# 配置与 CLI 校验
# =====================================================================


@dataclass
class RunOptions:
    layers: tuple[str, ...] = ("usb", "proc", "disk", "net")
    interval_s: float = 0.0
    count: int = 0
    duration_s: float = 10.0
    points: int = DEFAULT_POINTS
    start_mhz: float = DEFAULT_START_MHZ
    stop_mhz: float = DEFAULT_STOP_MHZ
    ifbw_hz: int = DEFAULT_IFBW_HZ
    power_dbm: float = -20.0
    dwell_us: int = 0
    ifft_points: int = 2048
    time_window_ns: float = 40.0
    dewow_window_ns: float = 4.0
    dewow_enabled: bool = False
    simulate: bool = False
    hardware: bool = False
    rate: float = DEFAULT_RATE_SWEEPS_PER_SEC
    gnss_mode: str = "valid"
    gnss_max_age_s: float = DEFAULT_GNSS_MAX_AGE_S
    sweep_timeout_s: Optional[float] = None
    out_dir: Optional[Path] = None
    sample_every_s: Optional[float] = None
    json_out: Optional[Path] = None
    mode: str = "production"
    queue_maxsize: int = 16
    queue_drop_policy: str = "drop_incoming"
    monitor_interval_s: float = 0.5
    baseline_json: Optional[Path] = None


def _check_positive_real(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是有限正实数（bool 被拒绝），实际 {type(value).__name__}")
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} 必须是有限正实数，实际 {value!r}")
    return number


def _check_nonneg_real(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是有限非负实数（bool 被拒绝），实际 {type(value).__name__}")
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{name} 必须是有限非负实数，实际 {value!r}")
    return number


def _check_nonneg_int(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} 必须是 int（bool 被拒绝），实际 {type(value).__name__}")
    if value < 0:
        raise ValueError(f"{name} 必须 >= 0，实际 {value!r}")
    return value


def _check_positive_int(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} 必须是 int（bool 被拒绝），实际 {type(value).__name__}")
    if value <= 0:
        raise ValueError(f"{name} 必须 > 0，实际 {value!r}")
    return value


def _check_freq_mhz(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是有限实数（bool 被拒绝），实际 {type(value).__name__}")
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} 必须是有限正实数，实际 {value!r}")
    return number


def _options_from_args(args: argparse.Namespace, default_duration: float) -> RunOptions:
    exec_mode = _resolve_mode(args)  # 显式确认门：simulate/hardware，缺一或冲突即拒绝。
    layers = tuple(s.strip() for s in args.layers.split(",") if s.strip())
    for layer in layers:
        if layer not in ("usb", "proc", "disk", "net"):
            raise SystemExit(f"未知 layer: {layer!r}（usb/proc/disk/net）")
    if "usb" not in layers:
        raise SystemExit("layers 必须至少包含 usb（测量采集基础）")
    if ("disk" in layers or "net" in layers) and "proc" not in layers:
        raise SystemExit(
            "layers 是累积式的：disk/net 依赖 proc 的处理输出，"
            "必须写 usb,proc,disk（或 usb,proc,disk,net）"
        )
    mode = getattr(args, "mode", "production")
    if mode not in ("production", "sync"):
        raise SystemExit(f"未知 mode: {mode!r}（production/sync）")
    try:
        opts = _build_run_options(args, default_duration, layers, mode)
    except (ValueError, TypeError) as exc:
        raise SystemExit(str(exc)) from exc
    opts.simulate = exec_mode == "simulate"
    opts.hardware = exec_mode == "hardware"
    if not opts.start_mhz < opts.stop_mhz:
        raise SystemExit(f"start_mhz 必须 < stop_mhz，实际 {opts.start_mhz} >= {opts.stop_mhz}")
    if opts.queue_drop_policy not in ("drop_incoming", "drop_oldest"):
        raise SystemExit(
            f"queue_drop_policy 必须是 drop_incoming/drop_oldest，实际 {opts.queue_drop_policy!r}"
        )
    if opts.gnss_mode not in ("valid", "no_fix", "stale", "none"):
        raise SystemExit(f"gnss 必须是 valid/no_fix/stale/none，实际 {opts.gnss_mode!r}")
    if opts.layers == ("usb",) and mode == "production":
        # 纯 USB 没有消费者线程拓扑，只能是直接 backend 基准。
        opts.mode = "sync"
    return opts


def _build_run_options(
    args: argparse.Namespace, default_duration: float, layers: tuple[str, ...], mode: str
) -> RunOptions:
    return RunOptions(
        layers=layers,
        interval_s=_check_nonneg_real("interval", args.interval),
        count=_check_nonneg_int("count", args.count),
        duration_s=_check_positive_real("seconds/duration", default_duration),
        points=_check_positive_int("points", args.points),
        start_mhz=_check_freq_mhz("start_mhz", args.start_mhz),
        stop_mhz=_check_freq_mhz("stop_mhz", args.stop_mhz),
        ifbw_hz=_check_positive_int("ifbw", args.ifbw),
        power_dbm=float(args.power),
        dwell_us=_check_nonneg_int("dwell", args.dwell),
        ifft_points=_check_positive_int("ifft_points", args.ifft_points),
        time_window_ns=_check_positive_real("time_window_ns", args.time_window_ns),
        dewow_window_ns=_check_positive_real("dewow_window_ns", args.dewow_window_ns),
        simulate=bool(args.simulate),
        rate=_check_positive_real("rate", args.rate),
        gnss_mode=args.gnss,
        gnss_max_age_s=_check_positive_real("gnss_max_age", args.gnss_max_age),
        sweep_timeout_s=(
            _check_positive_real("sweep_timeout", args.sweep_timeout)
            if args.sweep_timeout is not None
            else None
        ),
        out_dir=Path(args.out) if args.out else None,
        sample_every_s=(
            _check_positive_real("sample_every", args.sample_every)
            if getattr(args, "sample_every", None) is not None
            else None
        ),
        json_out=Path(args.json_out) if args.json_out else None,
        mode=mode,
        queue_maxsize=_check_positive_int("queue_maxsize", getattr(args, "queue_maxsize", 16)),
        queue_drop_policy=getattr(args, "queue_drop_policy", "drop_incoming"),
        monitor_interval_s=_check_positive_real(
            "monitor_interval", getattr(args, "monitor_interval", 0.5)
        ),
        baseline_json=Path(args.baseline_json) if getattr(args, "baseline_json", None) else None,
    )
    return opts


def _sweep_config(opts: RunOptions) -> U.SweepConfig:
    return U.SweepConfig(
        start_mhz=opts.start_mhz,
        stop_mhz=opts.stop_mhz,
        points=opts.points,
        ifbw_hz=opts.ifbw_hz,
        power_dbm=opts.power_dbm,
        dwell_us=opts.dwell_us,
        ifft_points=opts.ifft_points,
        time_window_ns=opts.time_window_ns,
        dewow_window_ns=opts.dewow_window_ns,
        dewow_enabled=opts.dewow_enabled,
    )


def _sweep_settings(opts: RunOptions) -> SweepSettings:
    cfg = _sweep_config(opts)
    return SweepSettings(
        start_hz=int(cfg.start_mhz * 1e6),
        stop_hz=int(cfg.stop_mhz * 1e6),
        points=cfg.points,
        ifbw_hz=cfg.ifbw_hz,
        power_dbm=cfg.power_dbm,
        dwell_us=cfg.dwell_us,
        stages_bitmap=S11_S22_STAGES_BITMAP,
    )


def _device_dict(backend: PersistentLibreVnaBackend) -> Optional[dict[str, object]]:
    info = backend.device_info
    if info is None:
        return None
    return {
        "protocol": info.protocol,
        "firmware": info.firmware,
        "hardware_version": info.hardware_version,
        "hardware_revision": info.hardware_revision,
        "min_freq_hz": info.min_freq_hz,
        "max_freq_hz": info.max_freq_hz,
        "min_ifbw_hz": info.min_ifbw_hz,
        "max_ifbw_hz": info.max_ifbw_hz,
        "max_points": info.max_points,
        "num_ports": info.num_ports,
    }


def _build_backend(opts: RunOptions) -> tuple[PersistentLibreVnaBackend, object]:
    """构造 backend（真机或模拟）+ 底层 transport（供故障注入/诊断）。

    **在实例化任何 transport 之前**严格校验：simulate/hardware 必须是 Python bool
    且恰好一个为 True（XOR）；两者都 True / 都 False / 类型非法均 ValueError。
    只有 ``hardware=True`` 才实例化 ``LibreVnaUsbTransport``；``simulate=True`` 只
    能构造模拟 transport。绝不隐式连接真机。
    """
    simulate = getattr(opts, "simulate", False)
    hardware = getattr(opts, "hardware", False)
    if type(simulate) is not bool or type(hardware) is not bool:
        raise ValueError("simulate/hardware 必须是布尔值")
    if simulate and hardware:
        raise ValueError("simulate 与 hardware 互斥，只能一个为 True")
    if not simulate and not hardware:
        raise ValueError(
            "未显式指定执行模式：必须 --simulate（软件模拟）或 --hardware（连接真实 LibreVNA）"
        )
    settings = PersistentBackendSettings(sweep_timeout_s=opts.sweep_timeout_s)
    if simulate:
        transport = StreamingSimulatedTransport(
            points=opts.points,
            start_hz=opts.start_mhz * 1e6,
            stop_hz=opts.stop_mhz * 1e6,
            sweeps_per_sec=opts.rate,
        )
        return PersistentLibreVnaBackend(transport=transport, settings=settings), transport
    transport = LibreVnaUsbTransport()  # 仅 hardware=True 可达。
    return PersistentLibreVnaBackend(transport=transport, settings=settings), transport


def _resolve_mode(args: argparse.Namespace) -> str:
    """解析互斥执行模式：simulate / hardware；缺一或冲突时拒绝（SystemExit）。

    simulate/hardware 必须是 Python bool（拒绝字符串/整数/其他真值对象）；在构造
    transport/backend 之前调用；不得因"缺少 --simulate"而隐式连接真机。
    """
    simulate = getattr(args, "simulate", False)
    hardware = getattr(args, "hardware", False)
    if type(simulate) is not bool or type(hardware) is not bool:
        raise SystemExit("simulate/hardware 必须是布尔值（--simulate/--hardware）")
    if simulate and hardware:
        raise SystemExit("--simulate 与 --hardware 互斥，只能二选一")
    if not simulate and not hardware:
        raise SystemExit(
            "必须显式指定执行模式：--simulate（软件模拟）或 --hardware（连接真实 LibreVNA）"
        )
    return "simulate" if simulate else "hardware"


# =====================================================================
# 清理跟踪（fail-closed：先清理，后定报告与退出码）
# =====================================================================


@dataclass
class CleanupResult:
    """每类实际存在的资源独立的退出状态；``ok`` 需全部为 True。

    ``clean``：清理操作（close/request_stop/join/is_alive/probe/stop）是否有异常；
    任何异常都置 False。``clean_shutdown``（报告字段）= ``ok and clean``。
    """

    backend_closed: bool = False
    producer_exited: bool = True
    sender_exited: bool = True
    receiver_exited: bool = True
    feeder_exited: bool = True
    monitor_exited: bool = True
    controller_exited: bool = True
    worker_exited: bool = True
    consumer_exited: bool = True
    clean: bool = True
    stop_to_usb_release_s: Optional[float] = None
    release_within_2_5s: Optional[bool] = None
    details: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(
            (
                self.backend_closed,
                self.producer_exited,
                self.sender_exited,
                self.receiver_exited,
                self.feeder_exited,
                self.monitor_exited,
                self.controller_exited,
                self.worker_exited,
                self.consumer_exited,
            )
        )

    @property
    def clean_shutdown(self) -> bool:
        """无清理异常且全部资源已退出。"""
        return self.ok and self.clean

    def to_dict(self) -> dict[str, object]:
        return {
            "ok": self.ok,
            "clean": self.clean,
            "clean_shutdown": self.clean_shutdown,
            "backend_closed": self.backend_closed,
            "producer_exited": self.producer_exited,
            "sender_exited": self.sender_exited,
            "receiver_exited": self.receiver_exited,
            "feeder_exited": self.feeder_exited,
            "monitor_exited": self.monitor_exited,
            "controller_exited": self.controller_exited,
            "worker_exited": self.worker_exited,
            "consumer_exited": self.consumer_exited,
            "stop_to_usb_release_s": self.stop_to_usb_release_s,
            "release_within_2_5s": self.release_within_2_5s,
            "details": list(self.details),
        }


def _release_within_target(release_s: float) -> bool:
    """roadmap §14.8：stop->USB 释放必须**严格小于** 2.5s（2.5s 本身不达标）。"""
    return release_s < USB_RELEASE_TARGET_S


class _CleanupTracker:
    """收集每类资源的清理结果与失败原因；调用方必须在最终报告前调用 finish()。"""

    def __init__(self, failures: list[str]) -> None:
        self._failures = failures
        self.result = CleanupResult()

    def close_backend(self, backend: PersistentLibreVnaBackend, *, stop_mono: Optional[float] = None) -> None:
        """尽力 close 并核对：抛错或仍打开都记为失败（fail-closed）。

        未打开（is_open 已为 False）的 backend 视为已安全关闭。stop_mono 提供时
        测量 **stop_mono → 实际释放完成** 的完整时长（roadmap §14.8），否则测量
        close() 调用自身耗时。release 达到或超过 2.5s 目标记 failure。is_open 读取
        抛错按"仍打开"处理。
        """
        is_open, _ok = _safe_probe(self._failures, "backend.is_open", lambda: backend.is_open, default=True)
        if not _ok:
            self.result.clean = False
        self.result.backend_closed = not is_open
        if not is_open:
            self.result.details.append("backend 未打开（无需关闭）")
            return
        start = stop_mono if stop_mono is not None else _mono()
        error: Optional[Exception] = None
        try:
            backend.close()
        except Exception as exc:  # noqa: BLE001 - 清理失败必须报告
            error = exc
            self.result.clean = False
        is_open_after, _ok = _safe_probe(
            self._failures, "backend.is_open", lambda: backend.is_open, default=True
        )
        if not _ok:
            self.result.clean = False
        released_at = _mono() if not is_open_after else None
        self.result.backend_closed = not is_open_after
        if released_at is not None:
            self.result.stop_to_usb_release_s = released_at - start
            self.result.release_within_2_5s = _release_within_target(
                self.result.stop_to_usb_release_s
            )
        if self.result.release_within_2_5s is False:
            self._failures.append(
                f"stop_to_usb_release_s（{self.result.stop_to_usb_release_s:.3f}s）达到或超过 "
                f"{USB_RELEASE_TARGET_S:g}s 目标（要求严格小于）"
            )
        if error is not None:
            released = not is_open_after
            self._failures.append(
                f"backend.close() 抛错（{type(error).__name__}: {error}）"
                + ("" if released else "；且 is_open 仍为 True（资源未释放）")
            )
            self.result.details.append(f"backend.close() 抛错: {type(error).__name__}: {error}")
        elif not is_open_after:
            self.result.details.append("backend 已安全关闭")
        else:
            self._failures.append("backend.close() 正常返回但 backend.is_open 仍为 True（资源未释放）")
            self.result.details.append("backend.close() 返回但 is_open 仍为 True")

    def verify_backend_closed(self, backend: PersistentLibreVnaBackend) -> None:
        """生产 run：backend 由 worker 的 _shutdown 释放，此处仅核对。"""
        is_open, _ok = _safe_probe(self._failures, "backend.is_open", lambda: backend.is_open, default=True)
        if not _ok:
            self.result.clean = False
        self.result.backend_closed = not is_open
        if is_open:
            self._failures.append("worker 结束后 backend.is_open 仍为 True（USB 未释放）")
            self.result.details.append("worker 结束后 backend 仍报告打开")

    def set_release_measurement(self, stop_mono: float, released_mono: float) -> None:
        self.result.stop_to_usb_release_s = released_mono - stop_mono
        self.result.release_within_2_5s = _release_within_target(
            self.result.stop_to_usb_release_s
        )
        if not self.result.release_within_2_5s:
            self._failures.append(
                f"stop_to_usb_release_s（{self.result.stop_to_usb_release_s:.3f}s）达到或超过 "
                f"{USB_RELEASE_TARGET_S:g}s 目标（要求严格小于）"
            )

    def stop_sender(self, sender: Optional[U.RemoteSender]) -> None:
        if sender is None:
            self.result.sender_exited = True
            return
        try:
            # 第1轮复审语义：先尝试真实排空（等待全部 ACK/状态消息完成）。
            # 网络断开等场景排空必然失败——清理阶段随后强制停止（明确放弃
            # 未发送内容；drain 失败本身是断网场景的预期结果，不记入
            # failure_reasons，只作为状态信息）。
            stopped = sender.stop(timeout_ms=3000, drain=True)
            if not stopped:
                stopped = sender.stop(timeout_ms=3000, drain=False)
        except Exception as exc:  # noqa: BLE001 - sender.stop 抛错结构化 + 标记 unclean
            self.result.sender_exited = False
            self.result.clean = False
            self._failures.append(f"sender.stop() 抛错: {type(exc).__name__}: {exc}")
            return
        self.result.sender_exited = bool(stopped)
        if not stopped:
            self._failures.append("RemoteSender 未在 3s 内退出（sender 线程残留）")

    def stop_receiver(self, receiver: Optional["LoopbackReceiver"]) -> None:
        """receiver.close()/is_alive() 异常必须结构化 fail-closed，不得逃出报告流程。

        - close() 抛错但线程已退出：receiver_exited 可为 true，但记 failure_reason
          （passed=false、rc=2）；
        - close() 抛错且线程仍活跃：receiver_exited=false、cleanup.ok=false。
        """
        if receiver is None:
            self.result.receiver_exited = True
            return
        close_error: Optional[Exception] = None
        try:
            receiver.close()
        except Exception as exc:  # noqa: BLE001 - 清理异常必须结构化
            close_error = exc
            self.result.clean = False
        alive, _ok = _safe_probe(
            self._failures, "receiver.is_alive()", lambda: receiver.is_alive(), default=True
        )
        if not _ok:
            self.result.clean = False
        self.result.receiver_exited = not alive
        if close_error is not None:
            self._failures.append(
                f"receiver.close() 抛错（{type(close_error).__name__}: {close_error}）"
            )
            self.result.details.append(
                f"receiver.close() 抛错: {type(close_error).__name__}: {close_error}"
            )
        if not self.result.receiver_exited:
            self._failures.append("receiver 未在期限内退出（线程残留）")

    def stop_thread(self, thread: Optional[threading.Thread], role: str, *, request_stop: Optional[Callable[[], None]] = None) -> None:
        """role ∈ feeder/monitor/controller；每类资源独立字段，互不复用。

        request_stop/join/ident/is_alive 抛错均结构化 fail-closed 并标记 unclean。
        """
        if thread is None:
            return
        if request_stop is not None:
            _value, _ok = _safe_stop_call(self._failures, role, "request_stop", request_stop)
            if not _ok:
                self.result.clean = False
        exited, _ok = _stop_thread(thread, role, self._failures)
        if not _ok:
            self.result.clean = False
        if role == "feeder":
            self.result.feeder_exited = exited
        elif role == "monitor":
            self.result.monitor_exited = exited
        elif role == "controller":
            self.result.controller_exited = exited
        if not exited:
            self._failures.append(f"{role} 未在期限内退出（线程残留）")

    def check_producer(self, transport: object) -> None:
        """只设置 producer_exited，绝不复用/覆盖 monitor 等字段。producer_alive 读取抛错按未退出。"""
        try:
            alive = bool(getattr(transport, "producer_alive", lambda: False)())
        except Exception as exc:  # noqa: BLE001 - 状态探测抛错不得逃出
            alive = True
            self.result.clean = False
            self._failures.append(
                f"transport.producer_alive() 抛错（按未安全退出处理）: {type(exc).__name__}: {exc}"
            )
        self.result.producer_exited = not alive
        if alive:
            self._failures.append("模拟 producer 线程未退出（线程残留）")

    def set_worker_exited(self, exited: bool) -> None:
        self.result.worker_exited = exited
        if not exited:
            self._failures.append("worker QThread 仍运行（线程残留）")

    def set_consumer_exited(self, exited: bool) -> None:
        self.result.consumer_exited = exited
        if not exited:
            self._failures.append("consumer QThread 仍运行（线程残留）")

    def finish(self) -> CleanupResult:
        if not self.result.ok and self.result.details:
            self._failures.append("清理不完整：" + "; ".join(self.result.details))
        return self.result


def _safe_repr(value: object) -> str:
    """安全 repr：超大整数/异常值的 str() 可能触发 Python 整数位数限制，兜底为类型名。"""
    try:
        return repr(value)
    except (ValueError, OverflowError):
        return f"<{type(value).__name__}（过大无法表示）>"


def _is_finite_real_not_bool(value: object) -> bool:
    """有限实数（int/float/numbers.Real），明确拒绝 bool、NaN、Inf、字符串数字。

    对任意 JSON 可表示值只返回 bool，不抛异常：超大整数（如 10**10000）使
    ``float()`` 抛 OverflowError，一并捕获。
    """
    if type(value) is bool:
        return False
    try:
        if not isinstance(value, numbers.Real):
            return False
        return math.isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def _is_finite_positive_real_not_bool(value: object) -> bool:
    return _is_finite_real_not_bool(value) and float(value) > 0.0


def _is_exact_nonneg_int(value: object) -> bool:
    """精确 Python int 且非负；拒绝 bool/float/字符串/NumPy 类型。"""
    return type(value) is int and value >= 0


def _is_exact_pos_int(value: object) -> bool:
    """精确 Python int 且为正；拒绝 bool/float/字符串/NumPy 类型。"""
    return type(value) is int and value > 0


def _strict_json_loads(text: str) -> object:
    """拒绝 NaN/Infinity/-Infinity 的严格 JSON 解析（parse_constant 抛 ValueError）。"""

    def _reject(name: str) -> float:
        raise ValueError(f"非标准 JSON 常量 {name}")

    return json.loads(text, parse_constant=_reject)


def _failed_cleanup(details: "list[str] | str") -> dict[str, object]:
    """统一失败 cleanup schema：ok/clean/clean_shutdown 均为精确 bool False。

    所有最终失败 JSON（缺失 cleanup 证据、严格序列化失败、json_out 写入失败、
    子场景抛异常、其他最小诊断报告）都使用该结构；禁止任何最终 JSON 出现只含 ok、
    缺少 clean/clean_shutdown 的 cleanup。
    """
    if isinstance(details, str):
        details = [details]
    return {
        "ok": False,
        "clean": False,
        "clean_shutdown": False,
        "details": [str(d) for d in details],
    }


def _cleanup_dict_is_strictly_clean(value: object) -> bool:
    """序列化 cleanup 字典必须三字段均为精确 bool True（ok/clean/clean_shutdown）。

    拒绝缺失、非 dict、非 bool、False/None/0/1/"true"、三字段自相矛盾。CleanupResult
    对象路径改用属性 ``clean_shutdown is True``。
    """
    if not isinstance(value, dict):
        return False
    for key in ("ok", "clean", "clean_shutdown"):
        if key not in value:
            return False
        if type(value[key]) is not bool:
            return False
        if value[key] is not True:
            return False
    return True


def _finalize_report(
    report: dict[str, object],
    args: argparse.Namespace,
    command: str,
    failures: list[str],
    cleanup: Optional[CleanupResult] = None,
    *,
    acceptance_eligible: Optional[bool] = None,
) -> int:
    """把 failure_reasons/cleanup/passed 写入报告并返回退出码（passed ? 0 : 2）。

    原子性（§1）：先组装并严格序列化报告；如指定 ``json_out`` 先尝试写入；**全部
    成功后才**向 stdout 输出成功报告。json_out 写入失败时 stdout 只输出一个最终
    失败 JSON（passed=false、acceptance_eligible=false、failure_reasons 含 json_out
    写入失败、rc=2），**绝不先输出 passed=true 报告**；stderr 只输出辅助信息且不与
    stdout 的最终结论冲突。

    使用严格序列化（``allow_nan=False``，无 ``default=str`` 静默转换）：callable、
    任意不支持对象、NaN/Inf 都导致 rc=2；报告无法序列化时输出最小、严格合法的诊断
    JSON 并列出非法字段路径与类型。
    """
    report["command"] = command
    report["failure_reasons"] = list(failures)
    # 确定 clean_shutdown 硬门（与 eligibility 来源无关）。每份最终报告都必须有
    # cleanup 证据：cleanup 对象 或 report 自带的严格 cleanup 字典 + 顶层字段。
    if cleanup is not None:
        real_cleanup = cleanup.to_dict()
        report["clean_shutdown"] = cleanup.clean_shutdown
        cleanup_gate = cleanup.clean_shutdown is True
        if not cleanup_gate:
            # 失败路径统一 cleanup 三字段全 false；真实资源结果保留在独立字段，
            # 不参与最终 clean_shutdown 判定。
            report["resource_cleanup_before_report_failure"] = real_cleanup
            report["cleanup"] = _failed_cleanup(
                "clean_shutdown 为 false（资源未全部安全退出或清理有异常）"
            )
        else:
            report["cleanup"] = real_cleanup
    else:
        cleanup_dict = report.get("cleanup")
        top_cs = report.get("clean_shutdown")
        cleanup_gate = (
            _cleanup_dict_is_strictly_clean(cleanup_dict)
            and type(top_cs) is bool
            and top_cs is True
        )
        if not cleanup_gate:
            # 补全/规范化 cleanup 诊断字典：三字段必须均为精确 bool true 且顶层
            # clean_shutdown=true，否则一律 clean_shutdown=false。
            report["cleanup"] = _failed_cleanup(
                "cleanup 证据缺失或不一致（需 ok/clean/clean_shutdown 均为 true 且顶层 clean_shutdown=true）"
            )
            report["clean_shutdown"] = False
    if not cleanup_gate:
        # clean_shutdown 是硬性通过门：不成立必须追加明确失败原因。
        report["failure_reasons"].append(
            "clean_shutdown 为 false（缺少或不一致的 cleanup 证据）"
        )
    # eligibility 归一化（严格 is True）：形参 > report 预置 > False；非法类型视为
    # false 并追加 failure（禁止 bool("false") 把字符串当 True）。
    if acceptance_eligible is not None:
        if type(acceptance_eligible) is bool:
            requested_eligible = acceptance_eligible
        else:
            report["failure_reasons"].append(
                "acceptance_eligible 形参类型非法（必须 bool，视为 false）"
            )
            requested_eligible = False
    elif "acceptance_eligible" in report:
        if type(report["acceptance_eligible"]) is bool:
            requested_eligible = report["acceptance_eligible"]
        else:
            report["failure_reasons"].append(
                "acceptance_eligible 类型非法（必须 bool，视为 false）"
            )
            requested_eligible = False
    else:
        requested_eligible = False
    report["passed"] = not report["failure_reasons"]
    report["acceptance_eligible"] = requested_eligible and cleanup_gate
    # 1) 严格序列化（失败 -> 只输出诊断 JSON）。
    try:
        text = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    except (TypeError, ValueError) as exc:
        invalid = _find_invalid_paths(report)
        failure_text, rc = _serialization_failure_json(command, str(exc), invalid)
        print(failure_text)
        _best_effort_json_out(failure_text, args)
        return rc
    # 2) 如指定 json_out，先写入；失败 -> stdout 只输出失败 JSON。
    json_out = getattr(args, "json_out", None)
    if json_out:
        try:
            Path(json_out).write_text(text, encoding="utf-8")
        except OSError as exc:
            saved_cleanup = report.get("cleanup")
            failure_report: dict[str, object] = {
                "command": command,
                "passed": False,
                "acceptance_eligible": False,
                "clean_shutdown": False,
                "failure_reasons": [f"json_out 写入失败: {exc}"],
                "cleanup": _failed_cleanup([f"json_out 写入失败: {exc}"]),
            }
            if isinstance(saved_cleanup, dict):
                # 保留写入失败前的真实资源清理结果，但不参与最终 clean_shutdown 判定。
                failure_report["resource_cleanup_before_report_failure"] = saved_cleanup
            failure_text = json.dumps(
                failure_report, ensure_ascii=False, indent=2, allow_nan=False
            )
            print(failure_text)
            print(f"json_out 写入失败: {exc}", file=sys.stderr)
            return 2
    # 3) 全部成功：stdout 输出成功/最终报告。
    print(text)
    return 0 if report["passed"] else 2


def _best_effort_json_out(text: str, args: argparse.Namespace) -> None:
    """诊断 JSON 尽力写入 json_out；失败只追加 stderr 辅助信息，不改变退出码。"""
    json_out = getattr(args, "json_out", None)
    if not json_out:
        return
    try:
        Path(json_out).write_text(text, encoding="utf-8")
    except OSError as exc:  # noqa: BLE001 - 辅助写入失败不影响结论
        print(f"json_out 诊断写入失败: {exc}", file=sys.stderr)


def _find_invalid_paths(obj: object, path: str = "$", _seen: Optional[set] = None) -> list[str]:
    """定位无法严格 JSON 序列化的节点（路径:类型），返回可序列化的字符串列表。

    安全处理：非法/NaN 字典键、循环引用（基于路径的 object-id 集合，绝不递归爆炸）、
    callable、任意不支持对象、嵌套 list/tuple/dict。诊断逻辑自身不得抛
    RecursionError/TypeError。
    """
    if _seen is None:
        _seen = set()
    if isinstance(obj, (dict, list, tuple)):
        oid = id(obj)
        if oid in _seen:
            return [f"{path}:circular({type(obj).__name__})"]
        _seen.add(oid)
        try:
            invalid: list[str] = []
            if isinstance(obj, dict):
                for key, value in obj.items():
                    if not _is_valid_json_key(key):
                        invalid.append(f"{path}.<key:{type(key).__name__}>")
                    invalid.extend(_find_invalid_paths(value, f"{path}.{_safe_key_label(key)}", _seen))
                return invalid
            for index, value in enumerate(obj):
                invalid.extend(_find_invalid_paths(value, f"{path}[{index}]", _seen))
            return invalid
        finally:
            _seen.discard(oid)  # 兄弟分支复用同一对象不误报循环
    try:
        json.dumps(obj, allow_nan=False)
    except (TypeError, ValueError):
        return [f"{path}:{type(obj).__name__}"]
    return []


def _is_valid_json_key(key: object) -> bool:
    """JSON 对象键必须是 str/int/float/bool/None，且 float 有限。"""
    if isinstance(key, (str, int, bool)) or key is None:
        return True
    if isinstance(key, float):
        return math.isfinite(key)
    return False


def _safe_key_label(key: object) -> str:
    try:
        label = str(key)
    except Exception:  # noqa: BLE001 - 键标签失败用类型名
        label = f"<{type(key).__name__}>"
    return label.replace(".", "_").replace("]", "_").replace("[", "_")


def _serialization_failure_json(
    command: str, reason: str, invalid: list[str]
) -> tuple[str, int]:
    """构造最小、严格合法的诊断 JSON（可被 allow_nan=False 序列化）。"""
    diagnostic = {
        "command": command,
        "passed": False,
        "acceptance_eligible": False,
        "clean_shutdown": False,
        "failure_reasons": [f"报告无法严格序列化: {reason}", f"非法字段: {invalid}"],
        "cleanup": _failed_cleanup(["报告序列化失败"]),
    }
    return json.dumps(diagnostic, ensure_ascii=False, indent=2, allow_nan=False), 2


# =====================================================================
# probe（fail-closed）
# =====================================================================


def cmd_probe(args: argparse.Namespace) -> int:
    exec_mode = _resolve_mode(args)  # 显式确认门：缺一或冲突即 SystemExit，且不构造 transport。
    try:
        sample = _check_positive_int("sample", args.sample)
        points = _check_positive_int("points", args.points)
        start_mhz = _check_freq_mhz("start_mhz", args.start_mhz)
        stop_mhz = _check_freq_mhz("stop_mhz", args.stop_mhz)
        rate = _check_positive_real("rate", args.rate)
        sweep_timeout_s = (
            _check_positive_real("sweep_timeout", args.sweep_timeout)
            if args.sweep_timeout is not None
            else None
        )
    except (ValueError, TypeError) as exc:
        raise SystemExit(str(exc)) from exc
    if not start_mhz < stop_mhz:
        raise SystemExit(f"start_mhz 必须 < stop_mhz，实际 {start_mhz} >= {stop_mhz}")
    opts = RunOptions(
        points=points,
        start_mhz=start_mhz,
        stop_mhz=stop_mhz,
        simulate=exec_mode == "simulate",
        hardware=exec_mode == "hardware",
        rate=rate,
        sweep_timeout_s=sweep_timeout_s,
    )
    failures: list[str] = []
    records: list[tuple[int, float, complex, complex]] = []
    _backend, raw_transport = _build_backend(opts)
    # 多留一道余量：真机在 configure 后可能处于某道中途，前几个点可能不是 0。
    tap = TappingTransport(
        raw_transport,
        max_records=(sample + 1) * opts.points,
        on_point=records.append,
    )
    backend = PersistentLibreVnaBackend(
        transport=tap, settings=PersistentBackendSettings(sweep_timeout_s=args.sweep_timeout)
    )
    tracker = _CleanupTracker(failures)
    report: dict[str, object] = {"command": "probe", "mode": "simulate" if opts.simulate else "hardware"}
    sweeps: list[object] = []
    try:
        backend.open()
        report["device"] = _device_dict(backend)
        backend.configure(_sweep_settings(opts))
        report["sweep_settings_stages_bitmap"] = hex(S11_S22_STAGES_BITMAP)
        for _ in range(sample):
            sweeps.append(backend.acquire_one())
    except Exception as exc:  # noqa: BLE001 - 普通运行异常结构化（不吞 KeyboardInterrupt/SystemExit）
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            raise
        failures.append(f"probe 执行失败: {type(exc).__name__}: {exc}")
    finally:
        # fail-closed：清理与状态必须在最终报告之前确定。
        tracker.close_backend(backend)
        tracker.check_producer(raw_transport)
        cleanup = tracker.finish()

    stats = backend.statistics
    report["sample"] = sample
    report["sample_complete"] = len(sweeps)
    report["points_per_sweep"] = opts.points
    report["integrity"] = {
        "duplicate_points": stats.duplicate_points,
        "jump_points": stats.jump_points,
        "out_of_range_points": stats.out_of_range_points,
        "invalid_points": stats.invalid_points,
        "completed_sweeps": stats.completed_sweeps,
        "dropped_sweeps": stats.dropped_sweeps,
        "incomplete_sweeps": stats.incomplete_sweeps,
        "sweep_timeouts": stats.sweep_timeouts,
        "pending_overflows": stats.pending_overflows,
    }

    # 逐点验证：容忍 configure 后未同步前缀，从第一个点 0 开始独立验证完整循环。
    seq = [r[0] for r in records]
    first_zero = next((i for i, p in enumerate(seq) if p == 0), None)
    full_cycles_ok = 0
    full_cycles_total = 0
    if first_zero is not None:
        tail = seq[first_zero:]
        full_cycles_total = len(tail) // opts.points
        for cycle in range(full_cycles_total):
            window = tail[cycle * opts.points : (cycle + 1) * opts.points]
            if window == list(range(opts.points)):
                full_cycles_ok += 1
    both_channels = bool(records) and all(
        r[2] is not None and r[3] is not None for r in records
    )
    sweep_shape_ok = all(
        sweep.frequency_hz.size == opts.points
        and sweep.raw.shape == (2, opts.points)
        and bool(np.all(np.isfinite(sweep.frequency_hz)))
        and bool(np.all(np.isfinite(sweep.raw.real)))
        and bool(np.all(np.isfinite(sweep.raw.imag)))
        for sweep in sweeps
    )
    report["point_sequence_strict_0_to_n_minus_1"] = bool(
        len(sweeps) == sample
        and full_cycles_total >= sample
        and full_cycles_ok == full_cycles_total
    )
    report["full_cycles_observed"] = full_cycles_ok
    report["unsynchronised_prefix_points"] = first_zero
    report["records_parsed"] = len(records)
    report["s11_s22_same_datapoint"] = both_channels
    report["every_sweep_shape_and_finite"] = bool(sweep_shape_ok)

    # 不变量校验（任一失败 -> failure_reason -> 非零）。
    if len(sweeps) != sample:
        failures.append(f"sample_complete != sample（{len(sweeps)} != {sample}）")
    if len(records) < sample * opts.points:
        failures.append(
            f"records_parsed 不足（{len(records)} < {sample}*{opts.points}）；"
            "无法验证完整循环"
        )
    if full_cycles_observed := full_cycles_ok:
        if full_cycles_ok < sample:
            failures.append(f"full_cycles_observed（{full_cycles_ok}）< sample（{sample}）")
    if not report["point_sequence_strict_0_to_n_minus_1"]:
        failures.append("点序不是严格的 0..N-1 循环")
    if not both_channels:
        failures.append("并非每个 VNADatapoint 同时给出有效的 S11 与 S22（含 records 为空）")
    if not sweep_shape_ok:
        failures.append("并非每条 sweep 形状为 (2, points) 且全部有限")
    for name in ("duplicate_points", "jump_points", "out_of_range_points", "invalid_points",
                 "dropped_sweeps", "incomplete_sweeps"):
        if report["integrity"][name] != 0:
            failures.append(f"integrity.{name} 非零（{report['integrity'][name]}）")

    acceptance = (
        bool(opts.simulate) is False
        and not failures
        and cleanup.clean_shutdown
        and cleanup.release_within_2_5s is True
    )
    return _finalize_report(
        report, args, "probe", failures, cleanup, acceptance_eligible=acceptance
    )


# =====================================================================
# 同步组件微基准（显式命名，非生产拓扑，不可作验收）
# =====================================================================


def _backend_integrity_reasons(stats) -> list[str]:
    """roadmap §14/§15 backend integrity：正常运行全部必须为 0。"""
    reasons: list[str] = []
    for name in (
        "dropped_sweeps",
        "incomplete_sweeps",
        "duplicate_points",
        "jump_points",
        "out_of_range_points",
        "invalid_points",
        "sweep_timeouts",
        "pending_overflows",
    ):
        value = getattr(stats, name)
        if value != 0:
            reasons.append(f"backend integrity.{name} 非零（{value}）")
    if stats.pending_depth_current != 0:
        reasons.append(f"backend pending_depth_current 非零（{stats.pending_depth_current}）")
    return reasons


def _queue_integrity_reasons(queue_stats: Optional[dict[str, object]]) -> list[str]:
    """生产 BoundedQueue：正常收尾后 overflows/dropped/depth 必须为 0。"""
    reasons: list[str] = []
    if queue_stats is None:
        return reasons
    for name in ("overflows", "dropped", "depth"):
        value = queue_stats.get(name, 0) or 0
        if value != 0:
            reasons.append(f"processing_queue.{name} 非零（{value}）")
    return reasons


def _scheduler_integrity_reasons(scheduler_stats: dict[str, object]) -> list[str]:
    reasons: list[str] = []
    if (scheduler_stats.get("missed_slots_total", 0) or 0) != 0:
        reasons.append(
            f"missed_slots_total 非零（{scheduler_stats['missed_slots_total']}）"
        )
    return reasons


def _net_integrity_reasons(trace_stats: Optional[dict[str, object]], enqueued: int) -> list[str]:
    """网络完整性：received/unique 必须 == enqueued，且无重复/缺失/乱序。"""
    reasons: list[str] = []
    if trace_stats is None:
        return reasons
    if trace_stats["received"] != enqueued:
        reasons.append(f"net.received（{trace_stats['received']}）!= enqueued（{enqueued}）")
    if trace_stats["unique"] != enqueued:
        reasons.append(f"net.unique（{trace_stats['unique']}）!= enqueued（{enqueued}）")
    for name in ("duplicates", "missing", "out_of_order"):
        if trace_stats[name] != 0:
            reasons.append(f"net.trace_stats.{name} 非零（{trace_stats[name]}）")
    return reasons


def run_layered_sync(
    opts: RunOptions,
    on_periodic: Optional[Callable[[dict[str, object]], None]] = None,
) -> tuple[dict[str, object], list[str], CleanupResult]:
    """Synchronous component microbenchmark：同一线程内 acquire+process+save+send。

    只测量各组件在本进程内的相对耗时，**不**复现生产线程解耦；任何含 proc 的
    默认验收路径必须用 :func:`run_layered_production`。topology 固定标记为
    ``synchronous component microbenchmark``，acceptance_eligible 恒 False。
    """
    failures: list[str] = []
    out_dir = opts.out_dir
    cleanup_out = False
    if "disk" in opts.layers and out_dir is None:
        out_dir = Path(tempfile.mkdtemp(prefix="field_test_save_"))
        cleanup_out = True
    backend, transport = _build_backend(opts)
    cfg = _sweep_config(opts)
    scheduler = U.FixedIntervalSweepSelector(opts.interval_s)
    gnss_cache = U.GnssFixCache(max_age_s=opts.gnss_max_age_s)
    gnss_feeder: Optional[SimGnssFeeder] = None
    receiver: Optional[LoopbackReceiver] = None
    sender: Optional[U.RemoteSender] = None
    monitor = _RunMonitor(opts.monitor_interval_s)
    proc_times: list[float] = []
    save_times: list[float] = []
    enqueue_times: list[float] = []
    acquire_times: list[float] = []
    all_sweeps = 0
    selected = 0
    processed = 0
    saved = 0
    enqueued = 0
    start_mono = time.monotonic()
    start_wall = time.time()
    last_sample = start_mono
    errors: list[str] = []
    tracker = _CleanupTracker(failures)

    try:
        backend.open()
        backend.configure(_sweep_settings(opts))
        if opts.gnss_mode != "none":
            gnss_feeder = SimGnssFeeder(gnss_cache, opts.gnss_mode, max_age_s=opts.gnss_max_age_s)
            gnss_feeder.start()
        if "net" in opts.layers:
            receiver = LoopbackReceiver()
            receiver.start()
            sender = U.RemoteSender(
                "127.0.0.1",
                receiver.port,
                run_id="field-test",
                status_callback=lambda msg: None,
            )
            sender.start()
        monitor.sender = sender
        monitor.start()
        while time.monotonic() - start_mono < opts.duration_s:
            if opts.count > 0 and selected >= opts.count:
                break
            acq_start = time.monotonic()
            acq_start_hires = _mono()
            acq_start_wall = time.time()
            try:
                sweep = backend.acquire_one()
            except LibreVnaSweepError as exc:
                errors.append(f"sweep timeout: {exc}")
                continue
            acq_cost = _mono() - acq_start_hires
            acq_end_wall = time.time()
            acquire_times.append(acq_cost)
            all_sweeps += 1
            midpoint = acq_start + acq_cost / 2.0
            result = scheduler.should_select(midpoint)
            if not result.select:
                continue
            selected += 1
            item = U.SelectedSweep(
                trace_index=selected,
                device_sweep_index=sweep.trace_index,
                frequency_hz=sweep.frequency_hz,
                raw=sweep.raw,
                cfg=cfg,
                midpoint_monotonic=midpoint,
                selected_at_utc=U.utc_iso_now(),
                missed_slots=result.missed_slots,
                actual_interval_s=result.actual_interval_s,
                schedule_error_ms=result.schedule_error_ms,
                acquired_at_utc=U.utc_iso_from_epoch((acq_start_wall + acq_end_wall) / 2.0),
                sweep_started_utc=U.utc_iso_from_epoch(acq_start_wall),
                sweep_completed_utc=U.utc_iso_from_epoch(acq_end_wall),
                scheduled_at_utc=(
                    U.utc_iso_from_epoch(
                        (result.next_due_s - opts.interval_s) + (start_wall - start_mono)
                    )
                    if result.next_due_s is not None and opts.interval_s > 0
                    else None
                ),
                connection_generation=backend.statistics.connection_generation,
                target_interval_s=opts.interval_s,
                session_statistics={"mode": "continuous", "scheduler": scheduler.statistics},
            )
            lookup = gnss_cache.latest(midpoint)
            fix = lookup.fix if (lookup.valid and lookup.fix is not None) else U.GgaFix()
            acquisition = U.ProcessingConsumer._acquisition_metadata(item, lookup)
            flight = U.ProcessingConsumer._flight_metadata(lookup, fix)
            raw_sparams = U.dual_channel_raw(sweep.raw)
            if "proc" in opts.layers:
                t0 = _mono()
                display, time_ns, trace_by_channel = U.process_trace_pipeline(
                    sweep.frequency_hz,
                    raw_sparams,
                    cfg,
                    calibration_p1=None,
                    calibration_p2=None,
                    through_measurement=None,
                    background_frequency_hz=None,
                    background=None,
                )
                proc_times.append(_mono() - t0)
                processed += 1  # 只有实际执行 proc 才计数
            else:
                display = None
                time_ns = None
                trace_by_channel = None
            if "disk" in opts.layers:
                t0 = _mono()
                U.save_trace(
                    out_dir,
                    selected,
                    cfg,
                    sweep.frequency_hz,
                    raw_sparams,
                    display,
                    time_ns,
                    trace_by_channel,
                    fix,
                    channels=("S11", "S22"),
                    acquisition=acquisition,
                    flight=flight,
                )
                save_times.append(_mono() - t0)
                saved += 1
            if "net" in opts.layers:
                t0 = _mono()
                sender.enqueue_trace(
                    selected,
                    cfg,
                    sweep.frequency_hz,
                    raw_sparams,
                    display,
                    time_ns,
                    trace_by_channel,
                    fix,
                    channels=("S11", "S22"),
                    acquisition=acquisition,
                    flight=flight,
                )
                enqueue_times.append(_mono() - t0)
                enqueued += 1
            if (
                opts.sample_every_s is not None
                and time.monotonic() - last_sample >= opts.sample_every_s
            ):
                last_sample = time.monotonic()
                on_periodic(_sync_snapshot(
                    backend, scheduler, gnss_cache, opts, transport, all_sweeps, selected,
                    processed, saved, enqueued, acquire_times, proc_times, save_times,
                    enqueue_times, monitor, errors, receiver, start_mono, start_wall,
                ))
        # 循环结束后（duration 到达），等待 sender 送达全部已入队 trace（有界）。
        if sender is not None and receiver is not None:
            drain_deadline = time.monotonic() + 5.0
            while time.monotonic() < drain_deadline and receiver.received_traces < enqueued:
                time.sleep(0.05)
    except Exception as exc:  # noqa: BLE001 - 普通运行异常结构化（不吞 KeyboardInterrupt/SystemExit）
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            raise
        failures.append(f"sync microbench 执行失败: {type(exc).__name__}: {exc}")
    finally:
        # fail-closed：先清理，后定报告与退出码。
        tracker.stop_thread(
            gnss_feeder, "feeder", request_stop=gnss_feeder.request_stop if gnss_feeder else None
        )
        tracker.stop_thread(monitor, "monitor", request_stop=monitor.request_stop)
        tracker.stop_sender(sender)
        tracker.stop_receiver(receiver)
        tracker.close_backend(backend)  # 未打开时按已安全关闭处理。
        tracker.check_producer(transport)
        if cleanup_out and out_dir is not None:
            _rmtree(out_dir)

    elapsed = time.monotonic() - start_mono
    backend_stats = backend.statistics
    scheduler_stats = scheduler.statistics
    if not all_sweeps:
        failures.append("零有效 sweep（all_complete_sweeps == 0）")
    if not selected:
        failures.append("零选中 sweep（selected_sweeps == 0）")
    if backend_stats.open_count != 1:
        failures.append(f"open_count != 1（{backend_stats.open_count}）")
    if backend_stats.configure_count != 1:
        failures.append(f"configure_count != 1（{backend_stats.configure_count}）")
    if max(0, backend_stats.connection_generation - 1) != 0:
        failures.append(f"reconnect_count != 0（{max(0, backend_stats.connection_generation - 1)}）")
    if "proc" in opts.layers and processed != selected:
        failures.append(f"processed（{processed}）!= selected（{selected}）")
    if "disk" in opts.layers and saved != selected:
        failures.append(f"saved（{saved}）!= selected（{selected}）")
    if "net" in opts.layers and enqueued != selected:
        failures.append(f"enqueued（{enqueued}）!= selected（{selected}）")
    failures.extend(_backend_integrity_reasons(backend_stats))
    failures.extend(_scheduler_integrity_reasons(scheduler_stats))
    if "net" in opts.layers:
        if receiver is not None:
            failures.extend(_net_integrity_reasons(receiver.trace_stats(), enqueued))
    report = _sync_snapshot(
        backend, scheduler, gnss_cache, opts, transport, all_sweeps, selected, processed,
        saved, enqueued, acquire_times, proc_times, save_times, enqueue_times, monitor,
        errors, receiver, start_mono, start_wall,
    )
    report["runtime_s"] = elapsed
    report["topology"] = "synchronous component microbenchmark"
    report["acceptance_eligible"] = False
    cleanup = tracker.finish()
    return report, failures, cleanup


def _sync_snapshot(
    backend, scheduler, gnss_cache, opts, transport, all_sweeps, selected, processed, saved,
    enqueued, acquire_times, proc_times, save_times, enqueue_times, monitor, errors, receiver,
    start_mono, start_wall,
) -> dict[str, object]:
    backend_stats = backend.statistics
    elapsed = time.monotonic() - start_mono
    scheduler_stats = scheduler.statistics
    gnss_latest = gnss_cache.latest()
    return {
        "command": "bench",
        "mode": "simulate" if opts.simulate else "hardware",
        "device": _device_dict(backend),
        "config": {
            "start_mhz": opts.start_mhz,
            "stop_mhz": opts.stop_mhz,
            "points": opts.points,
            "ifbw_hz": opts.ifbw_hz,
            "power_dbm": opts.power_dbm,
            "dwell_us": opts.dwell_us,
            "interval_s": opts.interval_s,
            "count": opts.count,
            "sweep_timeout_s": opts.sweep_timeout_s,
        },
        "layers": list(opts.layers),
        "simulate": opts.simulate,
        "sim_rate_sweeps_per_sec": opts.rate if opts.simulate else None,
        "elapsed_s": elapsed,
        "all_complete_sweeps": all_sweeps,
        "selected_sweeps": selected,
        "processed": processed,
        "saved": saved,
        "enqueued": enqueued,
        "sweeps_per_s": (all_sweeps / elapsed) if elapsed > 0 else 0.0,
        "selected_per_s": (selected / elapsed) if elapsed > 0 else 0.0,
        "backend": backend_stats.to_dict(),
        "open_count": backend_stats.open_count,
        "configure_count": backend_stats.configure_count,
        "connection_generation": backend_stats.connection_generation,
        "reconnect_count": max(0, backend_stats.connection_generation - 1),
        "scheduler": scheduler_stats,
        "actual_interval_s": scheduler_stats.get("actual_interval_s"),
        "schedule_error_ms": scheduler_stats.get("schedule_error_ms"),
        "processing_queue": None,  # 同步微基准无生产队列（不伪造指标）。
        "gnss": {
            "mode": opts.gnss_mode,
            "age_ms": gnss_latest.age_ms,
            "valid": gnss_latest.valid,
            "cache": gnss_cache.statistics(),
        },
        "timings": {
            "acquire_s": _sample_stats(acquire_times),
            "process_s": _sample_stats(proc_times),
            "save_s": _sample_stats(save_times),
            "enqueue_s": _sample_stats(enqueue_times),
        },
        "net": {
            "enqueued": enqueued,
            "received": receiver.received_traces if receiver is not None else None,
            "trace_stats": receiver.trace_stats() if receiver is not None else None,
        },
        "resources": monitor.result(),
        "sim_transport": (
            {
                "produced_points": getattr(transport, "produced_points", None),
                "writes": list(getattr(transport, "_writes", [])),
            }
            if opts.simulate
            else {}
        ),
        "errors": list(errors),
    }


# =====================================================================
# 生产线程拓扑 benchmark（默认验收路径）
# =====================================================================


class _TimedConsumer(U.ProcessingConsumer):
    """ProcessingConsumer 计时子类（consumer_factory 注入，不改冻结代码）。

    只负责计时，**不得**修改 ``item.session_statistics`` 或掩盖产品缺陷。冻结代码
    的 ``worker.statistics()`` 序列化缺陷已在 ``uav_gpr_qt.py`` 修复；若再次出现
    不可序列化对象，consumer 会崩溃 -> worker.failed -> 生产 benchmark 明确失败
    （由 errors 检查转为 failure_reason）。
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.trace_latencies: list[float] = []

    def _process_one(self, item) -> None:
        t0 = _mono()
        super()._process_one(item)
        self.trace_latencies.append(_mono() - t0)


class _ProductionController(threading.Thread):
    """到时 request_stop，测量 stop -> backend 释放 的时长，并周期采样状态。"""

    def __init__(
        self,
        worker,
        backend,
        duration_s: float,
        *,
        sample_every_s: Optional[float] = None,
        snapshot_fn: Optional[Callable[[], dict[str, object]]] = None,
        on_periodic: Optional[Callable[[dict[str, object]], None]] = None,
    ) -> None:
        super().__init__(name="field-test-controller", daemon=True)
        self._worker = worker
        self._backend = backend
        self._duration_s = duration_s
        self._sample_every_s = sample_every_s
        self._snapshot_fn = snapshot_fn
        self._on_periodic = on_periodic
        self._stop_event = threading.Event()
        self.stop_requested_at: Optional[float] = None
        self.backend_released_at: Optional[float] = None
        self.backend_release_timeout = False
        #: run() 内 request_stop / 释放探测 抛错的结构化诊断（不崩溃线程）。
        self.control_error: Optional[str] = None

    def request_stop(self) -> None:
        """请求 controller 立即退出（worker 提前结束时由主线程调用）。"""
        self._stop_event.set()

    def run(self) -> None:
        start = time.monotonic()
        last = start
        deadline = start + self._duration_s
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now >= deadline:
                break
            if (
                self._sample_every_s
                and self._on_periodic
                and self._snapshot_fn is not None
                and now - last >= self._sample_every_s
            ):
                last = now
                try:
                    self._on_periodic(self._snapshot_fn())
                except Exception:  # noqa: BLE001 - 周期采样不得中断
                    pass
            self._stop_event.wait(0.1)
        if self._stop_event.is_set():
            # 主线程提前叫停（worker 已结束）：不再 request_stop worker，直接退出。
            return
        self.stop_requested_at = _mono()
        try:
            self._worker.request_stop()
        except Exception as exc:  # noqa: BLE001 - stop 失败记录诊断，不崩溃线程
            self.control_error = (
                f"worker.request_stop() 抛错: {type(exc).__name__}: {exc}"
            )
            return
        deadline = self.stop_requested_at + RELEASE_POLL_TIMEOUT_S
        while _mono() < deadline:
            try:
                released = not self._backend.is_open
            except Exception as exc:  # noqa: BLE001 - 释放探测抛错记录诊断
                self.control_error = (
                    f"backend.is_open 读取抛错（按未释放处理）: {type(exc).__name__}: {exc}"
                )
                return
            if released:
                self.backend_released_at = _mono()
                return
            time.sleep(0.01)
        self.backend_release_timeout = True


@contextlib.contextmanager
def _timing_wrappers(proc_times: list, save_times: list, enqueue_times: list, enqueued_counter: list, sender):
    """包装生产模块函数以采集逐层耗时（consumer 线程内调用，运行后恢复）。

    ``sender.enqueue_trace`` 用实例属性包装（self 已绑定）；process/save 用模块
    全局包装（consumer 在 uav_gpr_qt 命名空间按名解析）。
    """
    real_process = U.process_trace_pipeline
    real_save = U.save_trace
    real_enqueue = None if sender is None else getattr(sender, "enqueue_trace", None)

    def wrap_process(frequency_hz, raw_sparams, cfg, **kwargs):
        t0 = _mono()
        result = real_process(frequency_hz, raw_sparams, cfg, **kwargs)
        proc_times.append(_mono() - t0)
        return result

    def wrap_save(*args, **kwargs):
        t0 = _mono()
        result = real_save(*args, **kwargs)
        save_times.append(_mono() - t0)
        return result

    def wrap_enqueue(*args, **kwargs):
        t0 = _mono()
        result = real_enqueue(*args, **kwargs)
        enqueue_times.append(_mono() - t0)
        enqueued_counter[0] += 1
        return result

    U.process_trace_pipeline = wrap_process
    U.save_trace = wrap_save
    if sender is not None and real_enqueue is not None:
        sender.enqueue_trace = wrap_enqueue  # 实例属性：无描述符自绑定
    try:
        yield
    finally:
        U.process_trace_pipeline = real_process
        U.save_trace = real_save
        if sender is not None and real_enqueue is not None:
            sender.enqueue_trace = real_enqueue


def run_layered_production(
    opts: RunOptions,
    on_periodic: Optional[Callable[[dict[str, object]], None]] = None,
    *,
    consumer_factory: Optional[Callable] = None,
    receiver_factory: Optional[Callable] = None,
    purpose: str = "bench",
) -> tuple[dict[str, object], list[str], CleanupResult]:
    """生产线程拓扑：ContinuousAcquisitionWorker + BoundedQueue + ProcessingConsumer。

    USB 生产线程只 acquire/select/put；处理、保存与遥测压缩在 consumer 线程；
    RemoteSender 独立线程发送。慢消费者不会阻塞 USB 排流（BoundedQueue 丢弃策略）。
    输出真实的 processing_queue / sender 队列 / receiver 统计。

    ``purpose`` 为 ``bench``/``longrun``，决定 acceptance 时长门槛
    （bench >= 60s，longrun >= 1800s）。backend integrity、队列、网络、调度器
    与线程清理任一不满足都 fail-closed。
    """
    if purpose not in ("bench", "longrun"):
        raise ValueError(f"purpose 必须是 bench/longrun，实际 {purpose!r}")
    failures: list[str] = []
    base_dir = opts.out_dir
    cleanup_out = False
    if "disk" in opts.layers and base_dir is None:
        base_dir = Path(tempfile.mkdtemp(prefix="field_test_save_"))
        cleanup_out = True
    # 唯一 run 子目录：避免覆盖/误计目标目录中已有的 trace 文件。
    run_dir = (base_dir / f"run_{os.getpid()}_{time.time_ns()}") if base_dir is not None else None
    if run_dir is not None:
        run_dir.mkdir(parents=True, exist_ok=True)
    out_dir = run_dir
    backend, transport = _build_backend(opts)
    scheduler = U.FixedIntervalSweepSelector(opts.interval_s)
    gnss_cache = U.GnssFixCache(max_age_s=opts.gnss_max_age_s)
    gnss_feeder: Optional[SimGnssFeeder] = None
    receiver: Optional[LoopbackReceiver] = None
    sender: Optional[U.RemoteSender] = None
    monitor = _RunMonitor(opts.monitor_interval_s)
    proc_times: list[float] = []
    save_times: list[float] = []
    enqueue_times: list[float] = []
    enqueued_counter: list[int] = [0]
    errors: list[str] = []
    status_log: list[str] = []
    tracker = _CleanupTracker(failures)
    worker: Optional[U.ContinuousAcquisitionWorker] = None
    controller: Optional[_ProductionController] = None
    consumer: Optional[U.ProcessingConsumer] = None
    start_mono = time.monotonic()
    saved_before_cleanup = 0
    try:
        if opts.gnss_mode != "none":
            gnss_feeder = SimGnssFeeder(gnss_cache, opts.gnss_mode, max_age_s=opts.gnss_max_age_s)
            gnss_feeder.start()
        if "net" in opts.layers:
            receiver = (receiver_factory() if receiver_factory is not None else LoopbackReceiver())
            receiver.start()
            sender = U.RemoteSender(
                "127.0.0.1",
                receiver.port,
                run_id="field-test",
                status_callback=lambda msg: None,
            )
            sender.start()

        def default_factory(queue, cache):
            return _TimedConsumer(
                queue=queue,
                gnss_cache=cache,
                cfg=_sweep_config(opts),
                out_dir=out_dir,
                autosave=("disk" in opts.layers),
                calibration_p1=None,
                calibration_p2=None,
                through_measurement=None,
                background_frequency_hz=None,
                background=None,
                remote_sender=sender,
            )

        factory = consumer_factory if consumer_factory is not None else default_factory
        worker = U.ContinuousAcquisitionWorker(
            cfg=_sweep_config(opts),
            interval_s=opts.interval_s,
            out_dir=out_dir,
            count=opts.count,
            gnss_port="",
            baud=115200,
            calibration_path="",
            session_calibration=None,
            session_calibration_p2=None,
            through_measurement=None,
            background_frequency_hz=None,
            background=None,
            autosave=("disk" in opts.layers),
            remote_sender=sender,
            backend=backend,
            scheduler=scheduler,
            gnss_cache=gnss_cache,
            consumer_factory=factory,
            processing_queue_maxsize=opts.queue_maxsize,
            queue_drop_policy=opts.queue_drop_policy,
            backend_settings=PersistentBackendSettings(sweep_timeout_s=opts.sweep_timeout_s),
        )
        monitor.sender = sender
        monitor.processing_queue = worker.processing_queue
        # 用 DirectConnection 捕获 worker/consumer 的诊断消息（无事件循环时 queued 信号不会投递）。
        worker.failed.connect(
            lambda msg: errors.append(f"worker.failed: {msg}"),
            U.QtCore.Qt.ConnectionType.DirectConnection,
        )
        worker.status.connect(
            lambda msg: status_log.append(str(msg)),
            U.QtCore.Qt.ConnectionType.DirectConnection,
        )
        monitor.start()

        def periodic_snapshot() -> dict[str, object]:
            elapsed = time.monotonic() - start_mono
            backend_stats = backend.statistics
            consumed = worker.consumer.processed_count if worker.consumer is not None else 0
            return {
                "elapsed_s": elapsed,
                "sweeps_per_s": (
                    backend_stats.completed_sweeps / elapsed if elapsed > 0 else 0.0
                ),
                "selected_sweeps": scheduler.statistics.get("selected_sweeps", 0),
                "processed": consumed,
                "saved": (_count_saved_pairs(out_dir) if "disk" in opts.layers else 0),
                "net": {"received": receiver.received_traces if receiver is not None else None},
                "backend": backend_stats.to_dict(),
                "gnss": {"age_ms": gnss_cache.latest().age_ms},
            }

        controller = _ProductionController(
            worker,
            backend,
            opts.duration_s,
            sample_every_s=opts.sample_every_s,
            snapshot_fn=periodic_snapshot,
            on_periodic=on_periodic,
        )
        controller.start()
        with _timing_wrappers(proc_times, save_times, enqueue_times, enqueued_counter, sender):
            worker.start()
            _wait_value, _ok = _safe_stop_call(
                failures,
                "worker",
                "wait",
                lambda: worker.wait(int((opts.duration_s + WORKER_WAIT_GRACE_S) * 1000)),
                default=False,
            )
            if not _ok:
                tracker.result.clean = False
            finished = bool(_wait_value)
        # worker 已结束（有限 count / 错误 / 超时）：立即停 controller 并确认退出。
        _v, _ok = _safe_stop_call(failures, "controller", "request_stop", controller.request_stop)
        if not _ok:
            tracker.result.clean = False
        _v, _ok = _safe_stop_call(failures, "controller", "join", lambda: controller.join(timeout=5.0))
        if not _ok:
            tracker.result.clean = False
        if controller.control_error:
            failures.append(controller.control_error)
            tracker.result.clean = False
        # worker 结束后，等待 sender 把已入队 trace 全部送达（有界），再进入清理，
        # 避免末尾入队项因 stop 未排空导致 received != enqueued 的竞态。
        if sender is not None and receiver is not None:
            drain_deadline = time.monotonic() + 5.0
            while (
                time.monotonic() < drain_deadline
                and receiver.received_traces < enqueued_counter[0]
            ):
                time.sleep(0.05)
        if "disk" in opts.layers:
            saved_before_cleanup = _count_saved_pairs(out_dir)
        if not finished:
            failures.append(f"worker 未在 {opts.duration_s + WORKER_WAIT_GRACE_S:g}s 内退出")
        controller_alive, _ok = _safe_probe(
            failures, "controller.is_alive()", lambda: controller.is_alive(), default=True
        )
        if not _ok:
            tracker.result.clean = False
        if controller_alive:
            failures.append("controller 未在 5s 内退出（线程残留）")
        if controller.backend_release_timeout:
            failures.append(f"stop 后 {RELEASE_POLL_TIMEOUT_S:g}s 内 USB 未释放")
        elif controller.stop_requested_at is not None and controller.backend_released_at is not None:
            tracker.set_release_measurement(
                controller.stop_requested_at, controller.backend_released_at
            )
        elif controller.stop_requested_at is None:
            tracker.result.details.append("controller 未请求 stop（时长异常）")
    except Exception as exc:  # noqa: BLE001 - 普通运行异常结构化（不吞 KeyboardInterrupt/SystemExit）
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            raise
        failures.append(f"production run 执行失败: {type(exc).__name__}: {exc}")
    finally:
        # fail-closed：先清理，后定报告与退出码。所有操作异常结构化诊断，绝不静默吞掉。
        worker_running = False
        if worker is not None:
            worker_running, _ok = _safe_probe(
                failures, "worker.isRunning()", lambda: worker.isRunning(), default=True
            )
            if not _ok:
                tracker.result.clean = False
        if worker is not None and worker_running:
            _v, _ok = _safe_stop_call(failures, "worker", "request_stop", worker.request_stop)
            if not _ok:
                tracker.result.clean = False
            _v, _ok = _safe_stop_call(failures, "worker", "wait", lambda: worker.wait(5000))
            if not _ok:
                tracker.result.clean = False
        if worker is None:
            tracker.set_worker_exited(True)
        else:
            worker_alive, _ok = _safe_probe(
                failures, "worker.isRunning()", lambda: worker.isRunning(), default=True
            )
            if not _ok:
                tracker.result.clean = False
            tracker.set_worker_exited(not worker_alive)
        # 读取**真实** worker.consumer 并检查 isRunning()（不得用局部 None 判断）。
        # 安全处理 None / 属性抛错：无法确定时按未安全退出处理。
        try:
            real_consumer = worker.consumer if worker is not None else None
        except Exception as exc:  # noqa: BLE001 - 检查失败按未退出处理
            tracker.result.clean = False
            failures.append(f"worker.consumer 读取抛错: {type(exc).__name__}: {exc}")
            real_consumer = None
            consumer_alive = True
        else:
            if real_consumer is None:
                consumer_alive = False
            else:
                consumer_alive, _ok = _safe_probe(
                    failures, "consumer.isRunning()", lambda: real_consumer.isRunning(), default=True
                )
                if not _ok:
                    tracker.result.clean = False
        tracker.set_consumer_exited(not consumer_alive)
        tracker.stop_sender(sender)
        tracker.stop_receiver(receiver)
        tracker.stop_thread(
            gnss_feeder, "feeder", request_stop=gnss_feeder.request_stop if gnss_feeder else None
        )
        tracker.stop_thread(monitor, "monitor", request_stop=monitor.request_stop)
        if controller is not None:
            _v, _ok = _safe_stop_call(failures, "controller", "request_stop", controller.request_stop)
            if not _ok:
                tracker.result.clean = False
            tracker.stop_thread(controller, "controller")
        tracker.verify_backend_closed(backend)
        tracker.check_producer(transport)
        if cleanup_out and base_dir is not None:
            _rmtree(base_dir)

    backend_stats = backend.statistics
    elapsed = time.monotonic() - start_mono
    scheduler_stats = scheduler.statistics
    gnss_latest = gnss_cache.latest()
    queue_stats = (
        worker.processing_queue.statistics() if worker is not None else {}
    )
    consumer = worker.consumer if worker is not None else None
    processed = consumer.processed_count if consumer is not None else 0
    trace_latencies = consumer.trace_latencies if isinstance(consumer, _TimedConsumer) else []
    saved_files = saved_before_cleanup if "disk" in opts.layers else 0
    enqueued = enqueued_counter[0]
    net_stats = {
        "enqueued": enqueued,
        "received": receiver.received_traces if receiver is not None else None,
        "trace_stats": receiver.trace_stats() if receiver is not None else None,
        "sender_queue_depth": {
            "current": sender.queue.qsize() if sender is not None else None,
            "peak": monitor.result()["sender_queue_depth"]["peak"] if sender is not None else None,
        },
    }
    report: dict[str, object] = {
        "command": "bench",
        "mode": "simulate" if opts.simulate else "hardware",
        "device": _device_dict(backend),
        "config": {
            "start_mhz": opts.start_mhz,
            "stop_mhz": opts.stop_mhz,
            "points": opts.points,
            "ifbw_hz": opts.ifbw_hz,
            "power_dbm": opts.power_dbm,
            "dwell_us": opts.dwell_us,
            "interval_s": opts.interval_s,
            "count": opts.count,
            "sweep_timeout_s": opts.sweep_timeout_s,
        },
        "layers": list(opts.layers),
        "simulate": opts.simulate,
        "sim_rate_sweeps_per_sec": opts.rate if opts.simulate else None,
        "topology": "production (ContinuousAcquisitionWorker + BoundedQueue + ProcessingConsumer)",
        "elapsed_s": elapsed,
        "all_complete_sweeps": backend_stats.completed_sweeps,
        "selected_sweeps": scheduler_stats.get("selected_sweeps", 0),
        "skipped_sweeps": scheduler_stats.get("skipped_sweeps", 0),
        "missed_slots_total": scheduler_stats.get("missed_slots_total", 0),
        "processed": processed,
        "saved": saved_files,
        "enqueued": enqueued,
        "sweeps_per_s": (backend_stats.completed_sweeps / elapsed) if elapsed > 0 else 0.0,
        "selected_per_s": (
            (scheduler_stats.get("selected_sweeps", 0) / elapsed) if elapsed > 0 else 0.0
        ),
        "backend": backend_stats.to_dict(),
        "open_count": backend_stats.open_count,
        "configure_count": backend_stats.configure_count,
        "connection_generation": backend_stats.connection_generation,
        "reconnect_count": max(0, backend_stats.connection_generation - 1),
        "scheduler": scheduler_stats,
        "actual_interval_s": scheduler_stats.get("actual_interval_s"),
        "schedule_error_ms": scheduler_stats.get("schedule_error_ms"),
        "processing_queue": queue_stats,
        "gnss": {
            "mode": opts.gnss_mode,
            "age_ms": gnss_latest.age_ms,
            "valid": gnss_latest.valid,
            "cache": gnss_cache.statistics(),
        },
        "timings": {
            # 生产 API 不暴露逐道 acquire 时间（worker 内部测量），如实为 None。
            "acquire_s": None,
            "consumer_trace_latency_s": _sample_stats(trace_latencies),
            "process_s": _sample_stats(proc_times),
            "save_s": _sample_stats(save_times),
            "enqueue_s": _sample_stats(enqueue_times),
        },
        "net": net_stats,
        "resources": monitor.result(),
        "queue_drop_policy": opts.queue_drop_policy,
        "errors": list(errors),
        "status_log": status_log,
    }

    # 不变量校验（fail-closed）。
    if not report["all_complete_sweeps"]:
        failures.append("零有效 sweep（all_complete_sweeps == 0）")
    if not report["selected_sweeps"]:
        failures.append("零选中 sweep（selected_sweeps == 0）")
    if report["open_count"] != 1:
        failures.append(f"open_count != 1（{report['open_count']}）")
    if report["configure_count"] != 1:
        failures.append(f"configure_count != 1（{report['configure_count']}）")
    if report["reconnect_count"] != 0:
        failures.append(f"reconnect_count != 0（{report['reconnect_count']}）")
    if "proc" in opts.layers and processed != report["selected_sweeps"]:
        failures.append(f"processed（{processed}）!= selected（{report['selected_sweeps']}）")
    if "disk" in opts.layers and saved_files != report["selected_sweeps"]:
        failures.append(f"saved（{saved_files}）!= selected（{report['selected_sweeps']}）")
    if "net" in opts.layers:
        if enqueued != report["selected_sweeps"]:
            failures.append(f"enqueued（{enqueued}）!= selected（{report['selected_sweeps']}）")
        failures.extend(_net_integrity_reasons(receiver.trace_stats() if receiver is not None else None, enqueued))
        if sender is not None and sender.queue.qsize() != 0:
            failures.append(f"sender 队列未排空（qsize={sender.queue.qsize()}）")
    failures.extend(_backend_integrity_reasons(backend_stats))
    failures.extend(_queue_integrity_reasons(queue_stats))
    failures.extend(_scheduler_integrity_reasons(scheduler_stats))
    if any("worker.failed" in e for e in errors):
        failures.append("worker/consumer 出现 failed/fatal_error（errors 含 worker.failed）")
    cleanup = tracker.finish()

    _apply_baseline_check(report, opts, failures)
    release_ok = cleanup.release_within_2_5s is True
    # §14.5 schedule_error p95 <= 一个 sweep 周期（需有效纯 USB 基线）。
    if opts.interval_s == 0:
        # interval=0 逐道选择，无 schedule_error 样本：正式验收必须用正数 --interval。
        schedule_ok = False
        sweep_period_ms: Optional[float] = None
        schedule_p95_ms: Optional[float] = None
        report["schedule_applicable"] = False
        report["schedule_reason"] = "interval=0 无 schedule_error 样本；正式验收需 --interval > 0"
    else:
        report["schedule_applicable"] = True
        baseline_for_period = (
            {"sweeps_per_s": report["baseline_sweeps_per_s"]}
            if report.get("baseline_sweeps_per_s") is not None
            else None
        )
        schedule_ok, sweep_period_ms, schedule_p95_ms = _schedule_error_within_sweep_period(
            scheduler_stats, baseline_for_period
        )
    report["schedule_error_within_sweep_period"] = schedule_ok
    report["sweep_period_ms"] = sweep_period_ms
    report["schedule_error_p95_ms"] = schedule_p95_ms
    acceptance = _acceptance_eligible(
        mode="simulate" if opts.simulate else "hardware",
        elapsed_s=float(report["elapsed_s"]),
        purpose=purpose,
        failures=failures,
        cleanup=cleanup,
        release_ok=release_ok,
        baseline_met=report.get("baseline_90_percent_met") is True,
        schedule_ok=schedule_ok,
    )
    report["acceptance_eligible"] = acceptance
    report["failure_reasons"] = list(failures)
    report["cleanup"] = cleanup.to_dict() if cleanup else {}
    report["clean_shutdown"] = cleanup.clean_shutdown if cleanup else False
    report["passed"] = not report["failure_reasons"]
    return report, failures, cleanup


def _count_saved_pairs(out_dir: Path) -> int:
    """统计本次运行生成的完整 CSV+NPZ 对（同 stem），不数历史 CSV。"""
    try:
        return sum(
            1
            for csv in out_dir.glob("trace_*.csv")
            if csv.with_suffix(".npz").exists()
        )
    except OSError:
        return 0


def _baseline_invalid_reason(baseline: object, opts: RunOptions, report: dict[str, object]) -> Optional[str]:
    """严格基线校验（§6）：对任意 JSON 值都是 total function（只返回 None 或错误字符串，
    不抛 AttributeError/TypeError/KeyError/OverflowError，不依赖调用方保证嵌套结构）。

    顶层与 nested cleanup schema 同时校验；layers/config/device 容器先验类型再访问。
    """
    if not isinstance(baseline, dict):
        return "基线不是 JSON 对象"
    if baseline.get("command") != "bench":
        return "基线 command != bench"
    if type(baseline.get("passed")) is not bool or baseline.get("passed") is not True:
        return "基线 passed 不是精确 bool true"
    if baseline.get("failure_reasons") != []:
        return "基线 failure_reasons 必须严格等于空 list"
    top_cs = baseline.get("clean_shutdown")
    if type(top_cs) is not bool or top_cs is not True:
        return "基线顶层 clean_shutdown 不是精确 bool true"
    cleanup = baseline.get("cleanup")
    if not _cleanup_dict_is_strictly_clean(cleanup):
        return "基线 cleanup 三字段（ok/clean/clean_shutdown）必须均为精确 bool true"
    if top_cs != cleanup["clean_shutdown"]:
        return "基线顶层 clean_shutdown 与 nested cleanup.clean_shutdown 不一致"
    if baseline.get("mode") != ("simulate" if opts.simulate else "hardware"):
        return "基线 mode 与当前运行不一致"
    # layers：必须 type is list 且严格 == ["usb"]（禁止 list() 隐式转换）。
    layers = baseline.get("layers")
    if type(layers) is not list or layers != ["usb"]:
        return "基线 layers 必须严格为 ['usb']（list 类型）"
    if baseline.get("topology") != "synchronous component microbenchmark":
        return "基线 topology 不是同步纯 USB 基线"
    # 计数整数字段：必须精确 Python int 0（拒绝 False/True/0.0/"0"/负数）。
    for key in ("processed", "saved", "enqueued"):
        if not (type(baseline.get(key)) is int and baseline.get(key) == 0):
            return f"基线 {key} 必须精确整数 0"
    # sweeps_per_s：有限正实数，非 bool/NaN/Inf/字符串。
    rate = baseline.get("sweeps_per_s")
    if not _is_finite_positive_real_not_bool(rate):
        return "基线 sweeps_per_s 必须有限正实数（非 bool/NaN/Inf/字符串）"
    # elapsed_s：有限实数 >= 最小时长。
    elapsed = baseline.get("elapsed_s")
    if not _is_finite_real_not_bool(elapsed) or float(elapsed) < BASELINE_MIN_S:
        return f"基线时长不足 {BASELINE_MIN_S:g}s（{_safe_repr(elapsed)}）"
    # config：先验 type is dict 再访问。
    bcfg = baseline.get("config")
    if type(bcfg) is not dict:
        return "基线 config 不是对象"
    for key in ("points", "ifbw_hz"):
        if not _is_exact_pos_int(bcfg.get(key)):
            return f"基线 config.{key} 必须精确正整数"
    for key in ("dwell_us", "count"):
        if not _is_exact_nonneg_int(bcfg.get(key)):
            return f"基线 config.{key} 必须精确非负整数"
    for key in ("start_mhz", "stop_mhz", "power_dbm"):
        if not _is_finite_real_not_bool(bcfg.get(key)):
            return f"基线 config.{key} 必须有限实数"
    st = bcfg.get("sweep_timeout_s")
    if st is not None and not _is_finite_positive_real_not_bool(st):
        return "基线 config.sweep_timeout_s 必须 None 或有限正实数"
    # 当前运行 report.config：防御损坏。
    cfg = report.get("config") if isinstance(report, dict) else None
    if type(cfg) is not dict:
        return "报告 config 不是对象（无法与基线比较参数）"
    for key in ("points", "start_mhz", "stop_mhz", "ifbw_hz", "power_dbm", "dwell_us",
                "sweep_timeout_s"):
        if bcfg.get(key) != cfg.get(key):
            return f"基线参数不一致: {key}"
    if baseline.get("mode") == "hardware":
        bdev = baseline.get("device")
        dev = report.get("device") if isinstance(report, dict) else None
        if type(bdev) is not dict:
            return "基线 device 不是对象"
        if type(dev) is not dict:
            return "报告 device 不是对象（无法与基线比较设备标识）"
        # 两侧都必须显式包含三个字段（缺失或显式 None 都拒绝，防 .get() 绕过）。
        for key in ("firmware", "protocol", "hardware_version"):
            if key not in bdev or bdev[key] is None:
                return f"基线 device 缺少或为 None: {key}"
            if key not in dev or dev[key] is None:
                return f"报告 device 缺少或为 None: {key}"
        # 按 LibreVnaDeviceInfo schema 严格类型校验后再比较值。
        for side, device in (("基线", bdev), ("报告", dev)):
            if type(device["firmware"]) is not str:
                return f"{side} device.firmware 必须精确为 str"
            if type(device["protocol"]) is not int:
                return f"{side} device.protocol 必须精确为 int（拒绝 bool）"
            if type(device["hardware_version"]) is not int:
                return f"{side} device.hardware_version 必须精确为 int（拒绝 bool）"
        for key in ("firmware", "protocol", "hardware_version"):
            if bdev[key] != dev[key]:
                return f"基线设备标识不一致: {key}"
    return None


def _baseline_ratio_met(current: float, baseline_rate: float) -> tuple[float, bool]:
    """roadmap §14.6 90% 门槛的确定性判定：ratio >= 0.9 才达标。"""
    ratio = current / baseline_rate if baseline_rate > 0.0 else 0.0
    return ratio, ratio >= 0.9


def _apply_baseline_check(report: dict[str, object], opts: RunOptions, failures: list[str]) -> None:
    """90% 基线比较（roadmap §14.6）。无基线 / 无效基线 / 参数不一致一律标"未评估"，
    不判通过也不判失败（smoke 可 passed=true），但会阻止 acceptance eligibility。
    """
    if opts.baseline_json is None:
        report["baseline_90_percent_met"] = "未评估（未提供 --baseline-json）"
        return
    try:
        baseline = _strict_json_loads(Path(opts.baseline_json).read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001 - 无效基线只标未评估，不伪造达标
        report["baseline_90_percent_met"] = (
            f"未评估（基线无法读取或包含非标准非有限数值: {type(exc).__name__}）"
        )
        return
    # 最外层防御：即使 _baseline_invalid_reason 遗漏，外部基线也不得让 bench/longrun
    # 崩溃。普通 Exception -> 未评估；不捕获 KeyboardInterrupt/SystemExit。
    try:
        reason = _baseline_invalid_reason(baseline, opts, report)
        if reason is not None:
            report["baseline_90_percent_met"] = f"未评估（{reason}）"
            return
        baseline_rate = float(baseline["sweeps_per_s"])
        current = report.get("sweeps_per_s", 0.0) or 0.0
        ratio, met = _baseline_ratio_met(current, baseline_rate)
        report["baseline_90_percent_met"] = met
        report["baseline_90_percent_ratio"] = ratio
        # 供 §14.5 schedule_error p95 <= sweep 周期 门槛使用。
        report["baseline_sweeps_per_s"] = baseline_rate
        if not met:
            failures.append(
                f"有效 sweep 速率低于同参数纯 USB 基线的 90%（{current:.3f} / "
                f"{baseline_rate:.3f} = {ratio:.3f}）"
            )
    except (KeyboardInterrupt, SystemExit):
        raise
    except Exception as exc:  # noqa: BLE001 - 最后防线，不逃逸
        report["baseline_90_percent_met"] = (
            f"未评估（baseline schema 校验异常: {type(exc).__name__}）"
        )
        report.pop("baseline_sweeps_per_s", None)


def _acceptance_eligible(
    *,
    mode: str,
    elapsed_s: float,
    purpose: str,
    failures: list[str],
    cleanup: CleanupResult,
    release_ok: bool,
    baseline_met: bool,
    schedule_ok: bool,
) -> bool:
    """单条命令是否具备真机验收资格。

    - 使用**实际 elapsed_s**，不是命令请求的 duration_s（--seconds 60 --count 1
      快速完成不得获得资格；longrun 提前结束同样不得获得）；
    - bench>=60s、longrun>=1800s；simulate 永不；
    - 依赖有效纯 USB 基线的 schedule_error p95 <= 一个 sweep 周期门槛
      （schedule_ok）。
    """
    if mode != "hardware":
        return False
    if failures:
        return False
    if not cleanup.clean_shutdown:
        return False
    if not release_ok:
        return False
    if not baseline_met:
        return False
    if not schedule_ok:
        return False
    min_s = LONGRUN_ACCEPTANCE_MIN_S if purpose == "longrun" else BENCH_ACCEPTANCE_MIN_S
    return elapsed_s >= min_s


def _schedule_error_within_sweep_period(
    scheduler_stats: dict[str, object], baseline: object
) -> tuple[bool, Optional[float], Optional[float]]:
    """roadmap §14.5：schedule_error_ms.p95 不得超过一个完整 sweep 周期。

    返回 (ok, sweep_period_ms, p95_ms)。基线无效/缺少样本/p95 非有限/超限 -> ok=False。
    """
    sweep_period_ms: Optional[float] = None
    p95_ms: Optional[float] = None
    if not isinstance(baseline, dict):
        return False, None, None
    baseline_rate = baseline.get("sweeps_per_s")
    if not _is_finite_positive_real_not_bool(baseline_rate):
        return False, None, None
    sweep_period_ms = 1000.0 / float(baseline_rate)
    # 容器先验类型：scheduler_stats / schedule_error_ms 都必须是 dict，否则 fail-closed。
    if not isinstance(scheduler_stats, dict):
        return False, sweep_period_ms, None
    error_stats = scheduler_stats.get("schedule_error_ms")
    if not isinstance(error_stats, dict):
        return False, sweep_period_ms, None
    p95_ms = error_stats.get("p95")
    # p95：有限非负实数，非 bool/NaN/Inf/字符串。
    if not _is_finite_real_not_bool(p95_ms) or float(p95_ms) < 0.0:
        return False, sweep_period_ms, p95_ms
    # count：精确 int、非 bool、>= 2。
    count = error_stats.get("count")
    if type(count) is not int or count < 2:
        return False, sweep_period_ms, float(p95_ms)
    return float(p95_ms) <= sweep_period_ms, sweep_period_ms, float(p95_ms)


# =====================================================================
# CLI 命令
# =====================================================================


def cmd_bench(args: argparse.Namespace) -> int:
    opts = _options_from_args(args, default_duration=args.seconds)
    if opts.mode == "production":
        report, failures, cleanup = run_layered_production(
            opts, _periodic_printer(opts), purpose="bench"
        )
    else:
        report, failures, cleanup = run_layered_sync(opts, _periodic_printer(opts))
    return _finalize_report(report, args, "bench", failures, cleanup)


def cmd_longrun(args: argparse.Namespace) -> int:
    opts = _options_from_args(args, default_duration=args.duration)
    opts.mode = "production"  # longrun 恒为生产线程拓扑验收路径。
    opts.sample_every_s = args.sample_every
    report, failures, cleanup = run_layered_production(
        opts, _periodic_printer(opts), purpose="longrun"
    )
    return _finalize_report(report, args, "longrun", failures, cleanup)


def _periodic_printer(opts: RunOptions) -> Callable[[dict[str, object]], None]:
    def printer(snapshot: dict[str, object]) -> None:
        backend = snapshot.get("backend", {})
        net = snapshot.get("net", {})
        print(
            f"[t={snapshot.get('elapsed_s', 0.0):7.1f}s] "
            f"sweeps/s={snapshot.get('sweeps_per_s', 0.0):.1f} "
            f"selected={snapshot.get('selected_sweeps', 0)} "
            f"processed={snapshot.get('processed', 0)} "
            f"saved={snapshot.get('saved', 0)} "
            f"sent={net.get('received') if isinstance(net, dict) else None} "
            f"dropped={backend.get('dropped_sweeps', 0) if isinstance(backend, dict) else 0} "
            f"pending_peak={backend.get('pending_depth_peak', 0) if isinstance(backend, dict) else 0} "
            f"gnss_age_ms={snapshot.get('gnss', {}).get('age_ms')}",
            flush=True,
        )

    return printer


# =====================================================================
# failures 场景
# =====================================================================


def cmd_failures(args: argparse.Namespace) -> int:
    exec_mode = _resolve_mode(args)  # 显式确认门：缺一或冲突即 SystemExit。
    try:
        opts = RunOptions(
            points=_check_positive_int("points", args.points),
            start_mhz=_check_freq_mhz("start_mhz", args.start_mhz),
            stop_mhz=_check_freq_mhz("stop_mhz", args.stop_mhz),
            simulate=exec_mode == "simulate",
            hardware=exec_mode == "hardware",
            rate=_check_positive_real("rate", args.rate),
            sweep_timeout_s=(
                _check_positive_real("sweep_timeout", args.sweep_timeout)
                if args.sweep_timeout is not None
                else None
            ),
        )
    except (ValueError, TypeError) as exc:
        raise SystemExit(str(exc)) from exc
    if not opts.start_mhz < opts.stop_mhz:
        raise SystemExit(f"start_mhz 必须 < stop_mhz，实际 {opts.start_mhz} >= {opts.stop_mhz}")
    results: dict[str, object] = {"command": "failures", "mode": "simulate" if opts.simulate else "hardware"}
    overall_failures: list[str] = []
    for name, fn in (
        ("pause_resume", _scenario_pause_resume),
        ("gnss_cache_no_fix_stale", _scenario_gnss_cache_no_fix_stale),
        ("network_drop", _scenario_network_drop),
    ):
        try:
            result = fn(opts)
        except Exception as exc:  # noqa: BLE001 - 场景失败记录并继续（保留原始信息）
            result = {
                "passed": False,
                "failure_reasons": [f"{type(exc).__name__}: {exc}"],
                "cleanup": _failed_cleanup([f"场景执行抛错: {exc}"]),
            }
        results[name] = result

    # 子场景 schema 校验（不信任子场景自报的 passed/clean_shutdown）。
    def _require_passed(result: object, name: str) -> None:
        if not isinstance(result, dict):
            overall_failures.append(f"{name} 场景结果不是 dict")
            return
        if type(result.get("passed")) is not bool or result.get("passed") is not True:
            overall_failures.append(f"{name} 场景 passed 不是精确 bool True")
            return
        if result.get("failure_reasons") != []:
            overall_failures.append(f"{name} 场景 failure_reasons 非空")
            return
        if name in ("pause_resume", "network_drop") and not _cleanup_dict_is_strictly_clean(
            result.get("cleanup")
        ):
            overall_failures.append(
                f"{name} 场景 cleanup 三字段（ok/clean/clean_shutdown）未全部为精确 bool true"
            )

    for name in ("pause_resume", "gnss_cache_no_fix_stale", "network_drop"):
        _require_passed(results.get(name), name)

    # 顶层 cleanup：两个资源子场景都必须通过严格三字段校验。
    resource_cleanups = []
    for name in ("pause_resume", "network_drop"):
        scenario = results.get(name)
        resource_cleanups.append(scenario.get("cleanup") if isinstance(scenario, dict) else None)
    top_clean_shutdown = len(resource_cleanups) == 2 and all(
        _cleanup_dict_is_strictly_clean(c) for c in resource_cleanups
    )
    results["cleanup"] = {
        "ok": top_clean_shutdown,
        "clean": top_clean_shutdown,
        "clean_shutdown": top_clean_shutdown,
        "resource_scenarios": (
            ["pause_resume", "network_drop"] if top_clean_shutdown else []
        ),
        "gnss_cache_scenario_has_no_resources": True,
    }
    if not top_clean_shutdown:
        overall_failures.append(
            "failures 资源子场景未全部满足 cleanup 三字段 true（缺失/损坏/仅 ok=true 均失败）"
        )
    results["failure_reasons"] = overall_failures
    results["passed"] = not overall_failures
    results["clean_shutdown"] = top_clean_shutdown and not overall_failures
    acceptance = (
        bool(opts.simulate) is False and not overall_failures and top_clean_shutdown
    )
    results["acceptance_eligible"] = acceptance
    return _finalize_report(results, args, "failures", overall_failures)


def _scenario_pause_resume(opts: RunOptions) -> dict[str, object]:
    """多轮 pause/resume：passed 完全由实际条件计算，禁止硬编码。"""
    failures: list[str] = []
    cycles = 3
    sweeps_per_side = 5
    expected_count = cycles * sweeps_per_side * 2
    backend, _transport = _build_backend(opts)
    tracker = _CleanupTracker(failures)
    indices: list[int] = []
    # 执行前预初始化 result，避免异常路径 UnboundLocalError 丢失原始 failure/cleanup。
    result: dict[str, object] = {
        "cycles": cycles,
        "open_count": 0,
        "configure_count": 0,
        "trace_index_continuous": False,
        "sweeps_acquired": 0,
        "expected_sweeps": expected_count,
        "integrity": {},
    }
    try:
        backend.open()
        backend.configure(_sweep_settings(opts))
        first_open = backend.statistics.open_count
        first_configure = backend.statistics.configure_count
        for cycle in range(cycles):
            for _ in range(sweeps_per_side):
                indices.append(backend.acquire_one().trace_index)
            backend.pause()
            if backend.state is not BackendState.PAUSED:
                failures.append(f"pause 后状态不是 PAUSED（{backend.state.value}）")
            try:
                backend.acquire_one()
                failures.append("PAUSED 下 acquire_one 未被拒绝")
            except LibreVnaStateError:
                pass
            backend.resume()
            if backend.state is not BackendState.STREAMING:
                failures.append(f"resume 后状态不是 STREAMING（{backend.state.value}）")
            for _ in range(sweeps_per_side):
                indices.append(backend.acquire_one().trace_index)
        stats = backend.statistics
        if stats.open_count != 1:
            failures.append(f"open_count != 1（{stats.open_count}）")
        if stats.configure_count != 1:
            failures.append(f"configure_count != 1（{stats.configure_count}）")
        if stats.open_count != first_open:
            failures.append("发生了 reopen")
        if stats.configure_count != first_configure:
            failures.append("发生了 reconfigure")
        if indices != list(range(expected_count)):
            failures.append(f"trace_index 不连续（共 {len(indices)} 条，预期 {expected_count}）")
        for name in ("dropped_sweeps", "incomplete_sweeps", "duplicate_points", "jump_points",
                     "out_of_range_points", "invalid_points", "sweep_timeouts", "pending_overflows"):
            if getattr(stats, name) != 0:
                failures.append(f"integrity.{name} 非零（{getattr(stats, name)}）")
        result.update({
            "cycles": cycles,
            "open_count": stats.open_count,
            "configure_count": stats.configure_count,
            "trace_index_continuous": indices == list(range(expected_count)),
            "sweeps_acquired": len(indices),
            "expected_sweeps": expected_count,
            "integrity": {
                "completed": stats.completed_sweeps,
                "dropped": stats.dropped_sweeps,
                "incomplete": stats.incomplete_sweeps,
                "duplicate": stats.duplicate_points,
                "jump": stats.jump_points,
                "out_of_range": stats.out_of_range_points,
                "invalid": stats.invalid_points,
                "timeouts": stats.sweep_timeouts,
                "overflows": stats.pending_overflows,
            },
        })
    except (LibreVnaError, OSError) as exc:
        failures.append(f"pause_resume 执行失败: {type(exc).__name__}: {exc}")
    finally:
        tracker.close_backend(backend)
        tracker.check_producer(_transport)
        cleanup = tracker.finish()
    result["failure_reasons"] = list(failures)
    result["cleanup"] = cleanup.to_dict()
    result["passed"] = not failures and cleanup.clean_shutdown
    return result


def _scenario_gnss_cache_no_fix_stale(opts: RunOptions) -> dict[str, object]:
    """GnssFixCache 无 Fix / 陈旧 的缓存与 metadata 语义（纯软件，不接串口）。

    该场景**不**声称覆盖 GNSS 串口断开/重连——那是现场未验证项。名称
    ``gnss_cache_no_fix_stale`` 与覆盖范围一致。
    """
    import datetime

    fake_clock = [1000.0]

    def clock() -> float:
        return fake_clock[0]

    def utc_now() -> str:
        return datetime.datetime.fromtimestamp(0, datetime.timezone.utc).isoformat(timespec="milliseconds")

    cache = U.GnssFixCache(max_age_s=5.0, clock=clock, utc_now=utc_now)
    valid_fix = U.GgaFix(
        raw="r", utc="00:00:00", latitude=30.0, longitude=120.0,
        fix_quality=1, satellites=9, altitude_m=100.0,
    )
    no_fix = U.GgaFix(raw="", utc="", latitude=None, longitude=None, fix_quality=0, satellites=0, altitude_m=None)

    accepted = cache.update(no_fix)
    lookup_none = cache.latest(fake_clock[0])
    accepted_valid = cache.update(valid_fix)
    fake_clock[0] = 1002.0
    lookup_fresh = cache.latest(fake_clock[0])
    fake_clock[0] = 1007.0
    lookup_stale = cache.latest(fake_clock[0])

    item = U.SelectedSweep(
        trace_index=1, device_sweep_index=1, frequency_hz=np.zeros(2),
        raw=np.zeros((2, 2), dtype=np.complex128), cfg=_sweep_config(opts),
        midpoint_monotonic=fake_clock[0], selected_at_utc=utc_now(), missed_slots=0,
        actual_interval_s=None, schedule_error_ms=None, acquired_at_utc=utc_now(),
        sweep_started_utc=utc_now(), sweep_completed_utc=utc_now(), scheduled_at_utc=None,
        connection_generation=1, target_interval_s=0.0,
        session_statistics={"mode": "continuous"},
    )
    meta_stale = U.ProcessingConsumer._acquisition_metadata(item, lookup_stale)
    flight_stale = U.ProcessingConsumer._flight_metadata(lookup_stale, U.GgaFix())
    failures: list[str] = []
    checks = {
        "invalid_fix_rejected": not accepted,
        "valid_fix_accepted": accepted_valid,
        "fresh_valid": lookup_fresh.valid,
        "fresh_age_ms": lookup_fresh.age_ms,
        "stale_valid": lookup_stale.valid,
        "stale_age_ms": lookup_stale.age_ms,
        "metadata_gnss_valid_on_stale": flight_stale.get("gnss_valid"),
        "metadata_gnss_age_ms_on_stale": meta_stale.get("gnss_age_ms"),
        "cache_invalid_skipped": cache.invalid_updates_skipped,
    }
    if accepted:
        failures.append("无效 Fix 未被拒绝")
    if not accepted_valid:
        failures.append("有效 Fix 未被接受")
    if not lookup_fresh.valid or lookup_fresh.age_ms != 2000.0:
        failures.append(f"新鲜 Fix 判定错误（valid={lookup_fresh.valid} age={lookup_fresh.age_ms}）")
    if lookup_stale.valid or lookup_stale.age_ms != 7000.0:
        failures.append(f"陈旧 Fix 判定错误（valid={lookup_stale.valid} age={lookup_stale.age_ms}）")
    if flight_stale.get("gnss_valid") is not False:
        failures.append("陈旧 Fix 的 flight.gnss_valid 不为 False")
    if meta_stale.get("gnss_age_ms") != 7000.0:
        failures.append(f"陈旧 Fix 的 metadata.gnss_age_ms 错误（{meta_stale.get('gnss_age_ms')}）")
    return {
        "checks": checks,
        "scope": "GnssFixCache 无 Fix/陈旧语义（不含 GNSS 串口断开/重连）",
        "serial_disconnect_covered": False,
        "failure_reasons": failures,
        "passed": not failures,
    }


def _scenario_network_drop(opts: RunOptions) -> dict[str, object]:
    """断网期间生产路径：确认连接失败、真实处理/保存/enqueue、本地继续。

    用没有监听端的端口让 RemoteSender 一直重试；采集与保存不因发送失败而停止。
    passed 由事件与计数计算，network_did_not_stop_usb 不是常量。
    """
    failures: list[str] = []
    out_dir = Path(tempfile.mkdtemp(prefix="field_test_netdrop_"))
    backend, transport = _build_backend(opts)
    tracker = _CleanupTracker(failures)
    # 找一个没有监听者的端口。
    dead = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    dead.bind(("127.0.0.1", 0))
    dead_port = dead.getsockname()[1]
    dead.close()
    sender_status: list[str] = []
    sender: Optional[U.RemoteSender] = None
    gnss_cache = U.GnssFixCache(max_age_s=5.0)
    scheduler = U.FixedIntervalSweepSelector(0.0)
    acquired = 0
    selected = 0
    saved = 0
    enqueued = 0
    queue_peak = 0
    # 执行前预初始化 result，避免异常路径 UnboundLocalError 丢失原始 failure/cleanup。
    result: dict[str, object] = {
        "acquired_while_network_down": 0,
        "selected_while_network_down": 0,
        "processed": 0,
        "saved_while_network_down": 0,
        "enqueued_while_network_down": 0,
        "remote_queue_depth_current": None,
        "remote_queue_depth_peak": 0,
        "backend_open_count": 0,
        "backend_configure_count": 0,
        "sender_connect_failure_observed": False,
        "network_did_not_stop_usb": False,
        "sender_never_connected": None,
        "note_duplicate_send_risk": (
            "RemoteSender 在发送失败重连时可能重发已确认 trace（已知产品风险，本轮不改）"
        ),
    }
    try:
        sender = U.RemoteSender(
            "127.0.0.1", dead_port, run_id="field-test", status_callback=sender_status.append
        )
        sender.start()
        backend.open()
        backend.configure(_sweep_settings(opts))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            sweep = backend.acquire_one()
            acquired += 1
            select_result = scheduler.should_select()
            if not select_result.select:
                continue
            selected += 1
            lookup = gnss_cache.latest()
            fix = lookup.fix if (lookup.valid and lookup.fix is not None) else U.GgaFix()
            raw_sparams = U.dual_channel_raw(sweep.raw)
            display, time_ns, trace_by_channel = U.process_trace_pipeline(
                sweep.frequency_hz, raw_sparams, _sweep_config(opts),
                calibration_p1=None, calibration_p2=None, through_measurement=None,
                background_frequency_hz=None, background=None,
            )
            U.save_trace(
                out_dir, selected, _sweep_config(opts), sweep.frequency_hz, raw_sparams,
                display, time_ns, trace_by_channel, fix,
                channels=("S11", "S22"),
            )
            saved += 1
            sender.enqueue_trace(
                selected, _sweep_config(opts), sweep.frequency_hz, raw_sparams,
                display, time_ns, trace_by_channel, fix,
                channels=("S11", "S22"),
            )
            enqueued += 1
            queue_peak = max(queue_peak, sender.queue.qsize())
        # 等待并确认 sender 已发生连接失败（status 回调里出现 retrying）。
        retry_seen = any("retrying" in msg for msg in sender_status)
        if not retry_seen:
            failures.append("未观察到 RemoteSender 连接失败/重试事件")
        stats = backend.statistics
        network_did_not_stop_usb = bool(
            acquired > 0 and selected > 0 and saved == selected and stats.completed_sweeps > 0
        )
        if not network_did_not_stop_usb:
            failures.append("网络断开期间采集/处理/保存未正常继续")
        if saved != selected:
            failures.append(f"saved（{saved}）!= selected（{selected}）")
        if enqueued == 0:
            failures.append("enqueued == 0（未真正调用 enqueue_trace）")
        result.update({
            "acquired_while_network_down": acquired,
            "selected_while_network_down": selected,
            "processed": selected,
            "saved_while_network_down": saved,
            "enqueued_while_network_down": enqueued,
            "remote_queue_depth_current": sender.queue.qsize(),
            "remote_queue_depth_peak": queue_peak,
            "backend_open_count": stats.open_count,
            "backend_configure_count": stats.configure_count,
            "sender_connect_failure_observed": retry_seen,
            "network_did_not_stop_usb": network_did_not_stop_usb,
            "sender_never_connected": sender._sock is None,
        })
    except (LibreVnaError, OSError) as exc:
        failures.append(f"network_drop 执行失败: {type(exc).__name__}: {exc}")
    finally:
        tracker.stop_sender(sender)
        tracker.close_backend(backend)
        tracker.check_producer(transport)
        _rmtree(out_dir)
        cleanup = tracker.finish()
    result["failure_reasons"] = list(failures)
    result["cleanup"] = cleanup.to_dict()
    result["passed"] = not failures and cleanup.clean_shutdown
    return result


def _rmtree(path: Path) -> None:
    import shutil

    shutil.rmtree(path, ignore_errors=True)


# =====================================================================
# CLI
# =====================================================================


def _add_common_parser(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--points", type=int, default=DEFAULT_POINTS, help="每道频点数（默认 201）")
    parser.add_argument("--start-mhz", type=float, default=DEFAULT_START_MHZ)
    parser.add_argument("--stop-mhz", type=float, default=DEFAULT_STOP_MHZ)
    parser.add_argument("--ifbw", type=int, default=DEFAULT_IFBW_HZ, help="IFBW Hz（默认 10000）")
    parser.add_argument("--power", type=float, default=-20.0, help="功率 dBm（默认 -20）")
    parser.add_argument("--dwell", type=int, default=0, help="dwell µs（默认 0）")
    parser.add_argument("--sweep-timeout", type=float, default=None, help="单道超时秒（默认按配置推导）")
    parser.add_argument("--simulate", action="store_true", help="无硬件：用模拟传输跑真实 backend 全链路")
    parser.add_argument("--hardware", action="store_true",
                        help="显式确认连接、控制真实 LibreVNA（与 --simulate 互斥，二者必选其一）")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_SWEEPS_PER_SEC, help="模拟 sweep/s（默认 200）")
    parser.add_argument("--json-out", type=str, default=None, help="把完整指标 JSON 写入该文件")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)

    p_probe = sub.add_parser("probe", help="确认设备/固件/0x1241 ACK/S11S22 同包/0..N-1")
    _add_common_parser(p_probe)
    p_probe.add_argument("--sample", type=int, default=10, help="抽查的道数（默认 10）")
    p_probe.set_defaults(func=cmd_probe)

    p_bench = sub.add_parser("bench", help="分层基准：usb[,proc[,disk[,net]]]")
    _add_common_parser(p_bench)
    p_bench.add_argument("--layers", type=str, default="usb,proc,disk,net")
    p_bench.add_argument("--seconds", type=float, default=10.0)
    p_bench.add_argument("--interval", type=float, default=0.0, help="应用层保存间隔秒（0=逐道）")
    p_bench.add_argument("--count", type=int, default=0, help="最多选择道数（0=不限）")
    p_bench.add_argument("--mode", type=str, default="production", choices=["production", "sync"],
                         help="production=线程拓扑（默认）；sync=同步组件微基准（非验收）")
    p_bench.add_argument("--queue-maxsize", type=int, default=16)
    p_bench.add_argument("--queue-drop-policy", type=str, default="drop_incoming",
                         choices=["drop_incoming", "drop_oldest"])
    p_bench.add_argument("--baseline-json", type=str, default=None,
                         help="同参数纯 USB 基线 JSON（用于 §14.6 的 90% 比较）")
    p_bench.add_argument("--monitor-interval", type=float, default=0.5)
    p_bench.add_argument("--gnss", type=str, default="valid", choices=["valid", "no_fix", "stale", "none"])
    p_bench.add_argument("--gnss-max-age", type=float, default=DEFAULT_GNSS_MAX_AGE_S)
    p_bench.add_argument("--ifft-points", type=int, default=2048)
    p_bench.add_argument("--time-window-ns", type=float, default=40.0)
    p_bench.add_argument("--dewow-window-ns", type=float, default=4.0)
    p_bench.add_argument("--out", type=str, default=None, help="保存目录（disk 层，默认临时目录）")
    p_bench.set_defaults(func=cmd_bench)

    p_long = sub.add_parser("longrun", help="稳定性运行（真机 >=30 分钟，生产线程拓扑）")
    _add_common_parser(p_long)
    p_long.add_argument("--layers", type=str, default="usb,proc,disk")
    p_long.add_argument("--duration", type=float, default=1800.0, help="总时长秒（默认 1800 = 30 分钟）")
    p_long.add_argument("--sample-every", type=float, default=60.0, help="周期性指标采样间隔秒")
    p_long.add_argument("--interval", type=float, default=0.0)
    p_long.add_argument("--count", type=int, default=0)
    p_long.add_argument("--queue-maxsize", type=int, default=16)
    p_long.add_argument("--queue-drop-policy", type=str, default="drop_incoming",
                         choices=["drop_incoming", "drop_oldest"])
    p_long.add_argument("--baseline-json", type=str, default=None)
    p_long.add_argument("--monitor-interval", type=float, default=0.5)
    p_long.add_argument("--gnss", type=str, default="valid", choices=["valid", "no_fix", "stale", "none"])
    p_long.add_argument("--gnss-max-age", type=float, default=DEFAULT_GNSS_MAX_AGE_S)
    p_long.add_argument("--ifft-points", type=int, default=2048)
    p_long.add_argument("--time-window-ns", type=float, default=40.0)
    p_long.add_argument("--dewow-window-ns", type=float, default=4.0)
    p_long.add_argument("--out", type=str, default=None)
    p_long.set_defaults(func=cmd_longrun)

    p_fail = sub.add_parser("failures", help="pause/resume、GNSS 缓存、网络断开场景")
    _add_common_parser(p_fail)
    p_fail.set_defaults(func=cmd_failures)

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
