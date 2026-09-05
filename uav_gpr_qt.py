#!/usr/bin/env python3
"""Qt control software for UAV-GPR acquisition with GNSS and LibreVNA."""

from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import os
import queue
import socket
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

import numpy as np
import pyqtgraph as pg
import serial
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PySide6 import QtCore, QtGui, QtWidgets
from serial.tools import list_ports

from bscan_view import (
    BSCAN_WINDOW_TRACES,
    bscan_start_after_append,
    bscan_processing_signature,
    bscan_time_axes_compatible,
    bscan_window_bounds,
    bscan_window_max_start,
    select_bscan_window,
    validate_bscan_trace,
)
from gpr_processing import apply_frequency_bpf, flat_reflection_filter, frequency_bpf_weights, frequency_to_time_trace
from gnss_map_widget import GnssMapWidget

sys.path.insert(0, str(Path(__file__).resolve().parent / "librevna"))
from librevna_usb_probe import PacketStream, open_device  # noqa: E402
from librevna_usb_s11 import (  # noqa: E402
    OSLCalibration,
    OSLPoint,
    acquire_s11,
    acquire_sparameters,
    compute_osl_point,
    request_device_info,
)

# 阶段 3 连续采集：时间选择器与持久后端（PyUSB 惰性加载，不在此触发硬件导入）。
sys.path.insert(0, str(Path(__file__).resolve().parent))
from acquisition_scheduler import FixedIntervalSweepSelector  # noqa: E402
from pipeline_metrics import PipelineMetrics  # noqa: E402

#: 空中端全局性能统计（第1轮复审：真实阶段接入生产路径）。默认 None=禁用；
#: 由 MainWindow/基准脚本注入。所有记录点吞异常，绝不影响采集。
AIR_METRICS: PipelineMetrics | None = None


def set_air_metrics(metrics: PipelineMetrics | None) -> None:
    """注入/清除空中端性能统计（None=禁用）。"""
    global AIR_METRICS
    AIR_METRICS = metrics


def _record_stage(stage: str, duration_s: float) -> None:
    """安全记录空中端阶段耗时（未注入/异常都不得影响采集）。"""
    if AIR_METRICS is None:
        return
    try:
        AIR_METRICS.record_stage(stage, duration_s)
    except Exception:  # noqa: BLE001
        pass
from librevna.librevna_continuous import (  # noqa: E402
    PersistentBackendSettings,
    PersistentLibreVnaBackend,
)
from librevna.librevna_errors import LibreVnaSweepError  # noqa: E402
from librevna.librevna_protocol import (  # noqa: E402
    S11_S22_STAGES_BITMAP,
    SweepSettings,
)


PROCESSING_BPF_RULE = "f0=fmin; f1=fmin+(fmax-fmin)/4; f2=fmax-(fmax-fmin)/4; f3=fmax; sin^2 tapers"


@dataclass
class SweepConfig:
    start_mhz: float = 400.0
    stop_mhz: float = 4000.0
    points: int = 201
    ifbw_hz: int = 10000
    power_dbm: float = -20.0
    dwell_us: int = 0
    ifft_points: int = 2048
    time_window_ns: float = 40.0
    dewow_window_ns: float = 4.0
    dewow_enabled: bool = False
    bpf_rule: str = PROCESSING_BPF_RULE


@dataclass
class GgaFix:
    raw: str = ""
    utc: str = ""
    latitude: float | None = None
    longitude: float | None = None
    fix_quality: int = 0
    satellites: int = 0
    altitude_m: float | None = None


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def utc_iso_now() -> str:
    """真正 UTC 的 ISO 8601（+00:00 偏移），供 acquisition ``*_utc`` 字段使用。

    与 :func:`utc_timestamp`（本地偏移）区分：遥测 header 的 ``timestamp`` 保持
    既有兼容行为，新增的 acquisition 时间字段统一为真 UTC。
    """
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def utc_iso_from_epoch(epoch_seconds: float) -> str:
    """把 Unix epoch 秒转为真正 UTC 的 ISO 8601（+00:00 偏移）。

    供 acquisition 的 sweep 首末点/计划时刻（来自 :func:`time.time` 墙钟）格式
    化为 UTC；满足 ``datetime.fromisoformat(...).utcoffset() == timedelta(0)``。
    """
    return datetime.fromtimestamp(epoch_seconds, timezone.utc).isoformat(timespec="milliseconds")


# =====================================================================
# 阶段 3：legacy/continuous feature flag 与纯 Python 并发组件
# =====================================================================

#: 采集模式环境变量（"legacy" 或 "continuous"），缺省 / 非法值回退 legacy。
ACQUISITION_MODE_ENV = "UAV_GPR_ACQUISITION_MODE"
DEFAULT_ACQUISITION_MODE = "legacy"


def acquisition_mode() -> str:
    """读取当前采集模式（环境变量 UAV_GPR_ACQUISITION_MODE，默认 legacy）。"""
    mode = os.environ.get(ACQUISITION_MODE_ENV, DEFAULT_ACQUISITION_MODE).strip().lower()
    return mode if mode in ("legacy", "continuous") else DEFAULT_ACQUISITION_MODE


def is_continuous_enabled() -> bool:
    return acquisition_mode() == "continuous"


def is_valid_gnss_fix(fix: GgaFix) -> bool:
    """统一 GNSS Fix 有效性校验（roadmap §10 数据真实性，区域一）。

    必须**全部**满足才算有效，否则 :class:`GnssFixCache` 拒绝写入：

    1. fix 是 :class:`GgaFix`；
    2. ``fix_quality > 0``（0 = 无定位，不算有效 Fix）；
    3. latitude / longitude 存在；
    4. 经纬度是有限实数（拒绝 bool / 字符串 / NaN / Inf）；
    5. latitude ∈ [-90, 90]、longitude ∈ [-180, 180]。
    """
    if not isinstance(fix, GgaFix):
        return False
    if isinstance(fix.fix_quality, bool) or not isinstance(fix.fix_quality, int):
        return False
    if fix.fix_quality <= 0:
        return False
    latitude, longitude = fix.latitude, fix.longitude
    if isinstance(latitude, bool) or not isinstance(latitude, (int, float)):
        return False
    if isinstance(longitude, bool) or not isinstance(longitude, (int, float)):
        return False
    lat, lon = float(latitude), float(longitude)
    if not math.isfinite(lat) or not math.isfinite(lon):
        return False
    return -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0


@dataclass
class TimestampedFix:
    """GNSS Fix + 主机接收时间（monotonic 与 UTC），供时间匹配。"""

    fix: GgaFix
    received_monotonic: float
    received_utc: str


@dataclass
class GnssFixLookup:
    """:meth:`GnssFixCache.latest` 的结果：Fix + 相对给定时刻的年龄与有效性。"""

    fix: GgaFix | None
    age_ms: float | None
    valid: bool

    def fix_or_empty(self) -> GgaFix:
        return self.fix if self.fix is not None else GgaFix()


class GnssFixCache:
    """线程安全、带时间戳的最近 GNSS Fix 缓存（roadmap §10）。

    GNSS 线程对每条有效 Fix 调用 :meth:`update`；消费者按 sweep 采集中点的
    host monotonic 时间查询"该时刻生效的最新 Fix"（received_monotonic <=
    as_of）。只有存在且年龄不超过 ``max_age_s`` 的 Fix 才算 ``valid``；否则
    ``valid=False`` 并保留实际 age，绝不把过期/缺失坐标伪装成有效数据。
    """

    def __init__(
        self,
        max_age_s: float = 5.0,
        *,
        clock: Callable[[], float] | None = None,
        utc_now: Callable[[], str] | None = None,
        max_entries: int = 128,
    ) -> None:
        if isinstance(max_age_s, bool) or not isinstance(max_age_s, (int, float)):
            raise TypeError("max_age_s 必须是有限正数（bool 被拒绝）")
        max_age = float(max_age_s)
        if not math.isfinite(max_age) or max_age <= 0.0:
            raise ValueError(f"max_age_s 必须是有限正数，实际得到 {max_age_s!r}")
        if isinstance(max_entries, bool) or not isinstance(max_entries, int) or max_entries <= 0:
            raise ValueError(f"max_entries 必须是正整数，实际得到 {max_entries!r}")
        self.max_age_s = max_age
        self._clock = clock if clock is not None else time.monotonic
        self._utc_now = utc_now if utc_now is not None else utc_timestamp
        self._lock = threading.Lock()
        self._fixes: deque[TimestampedFix] = deque(maxlen=max_entries)
        self.update_count = 0
        self.invalid_updates_skipped = 0
        self.lookup_count = 0

    def update(self, fix: GgaFix) -> bool:
        """缓存一条 Fix；不满足 :func:`is_valid_gnss_fix` 的 Fix 被拒绝。

        拒绝时返回 False、不写入缓存、``invalid_updates_skipped += 1``。
        """
        if not is_valid_gnss_fix(fix):
            with self._lock:
                self.invalid_updates_skipped += 1
            return False
        with self._lock:
            self._fixes.append(
                TimestampedFix(
                    fix=fix,
                    received_monotonic=self._clock(),
                    received_utc=self._utc_now(),
                )
            )
            self.update_count += 1
            return True

    def latest(self, as_of_monotonic: float | None = None) -> GnssFixLookup:
        """返回给定时刻最新生效的 Fix（received <= as_of）及其年龄。

        无 Fix 或 Fix 年龄超过 ``max_age_s`` 时 ``valid=False`` 并给出实际 age。
        """
        as_of = self._clock() if as_of_monotonic is None else float(as_of_monotonic)
        if not math.isfinite(as_of):
            raise ValueError(f"as_of_monotonic 必须是有限值：{as_of_monotonic!r}")
        with self._lock:
            self.lookup_count += 1
            newest: TimestampedFix | None = None
            for entry in reversed(self._fixes):
                if entry.received_monotonic <= as_of:
                    newest = entry
                    break
            if newest is None:
                return GnssFixLookup(None, None, False)
            age_s = as_of - newest.received_monotonic
            valid = age_s >= 0.0 and age_s <= self.max_age_s
            return GnssFixLookup(newest.fix, age_s * 1000.0, valid)

    def statistics(self) -> dict[str, object]:
        with self._lock:
            newest = self._fixes[-1] if self._fixes else None
            return {
                "size": len(self._fixes),
                "update_count": self.update_count,
                "invalid_updates_skipped": self.invalid_updates_skipped,
                "lookup_count": self.lookup_count,
                "newest_age_s": (
                    self._clock() - newest.received_monotonic if newest is not None else None
                ),
            }


class BoundedQueue:
    """有界 FIFO 队列，生产者 :meth:`put` 永不阻塞（roadmap §14、§18）。

    队列满时按 ``drop_policy`` 处理并记录 overflows / dropped 统计：

    - ``drop_incoming``：丢弃新来的 item（默认）；
    - ``drop_oldest``：丢弃最旧的 item 后接受新 item。

    消费者 :meth:`get` 超时返回 None，配合 stop Event 实现有界退出，保证慢
    消费者不会阻止 USB 持续排流。
    """

    def __init__(self, maxsize: int, *, drop_policy: str = "drop_incoming") -> None:
        if isinstance(maxsize, bool) or not isinstance(maxsize, int):
            raise TypeError(
                f"maxsize 必须是 int（bool 被拒绝），实际得到 {type(maxsize).__name__}"
            )
        if maxsize <= 0:
            raise ValueError(f"maxsize 必须是正整数，实际得到 {maxsize!r}")
        if drop_policy not in ("drop_incoming", "drop_oldest"):
            raise ValueError(f"drop_policy 必须是 drop_incoming 或 drop_oldest，实际得到 {drop_policy!r}")
        self.maxsize = maxsize
        self.drop_policy = drop_policy
        self._q: queue.Queue[object] = queue.Queue(maxsize)
        self._lock = threading.Lock()
        self.total_put = 0
        self.total_get = 0
        self.overflows = 0
        self.dropped = 0
        self.depth_peak = 0

    def put(self, item: object) -> bool:
        """尝试入队；队列满时按策略处理。返回最终是否入队成功。"""
        with self._lock:
            if self._q.qsize() < self.maxsize:
                self._q.put_nowait(item)
                self.total_put += 1
                depth = self._q.qsize()
                if depth > self.depth_peak:
                    self.depth_peak = depth
                return True
            self.overflows += 1
            if self.drop_policy == "drop_incoming":
                self.dropped += 1
                return False
            try:
                self._q.get_nowait()
            except queue.Empty:
                self.dropped += 1
                return False
            self.dropped += 1
            self._q.put_nowait(item)
            self.total_put += 1
            return True

    def get(self, timeout: float | None = None) -> object | None:
        try:
            item = self._q.get(timeout=timeout)
        except queue.Empty:
            return None
        with self._lock:
            self.total_get += 1
        return item

    def empty(self) -> bool:
        return self._q.empty()

    def statistics(self) -> dict[str, object]:
        with self._lock:
            return {
                "maxsize": self.maxsize,
                "depth": self._q.qsize(),
                "depth_peak": self.depth_peak,
                "total_put": self.total_put,
                "total_get": self.total_get,
                "overflows": self.overflows,
                "dropped": self.dropped,
            }


class LosslessProcessingQueue:
    """生产处理队列的无损模式（第1轮检查点C）。

    与 :class:`BoundedQueue` 的丢弃策略相对：

    - FIFO，``put`` 恒定成功（达到安全硬上限时阻塞等待消费者腾位；
      正常运行下 USB 线程绝不阻塞——处理是流水线，阻塞仅出现在长期
      过载触发受控停止路径上）；
    - 不允许 drop_incoming / drop_oldest；
    - 记录 depth / peak / oldest_age_ms / high_water_events；
    - 达到高水位立即置 ``high_water`` 事件（供生产者告警）；
    - 达到硬上限后先接收当前 Trace，然后置 ``stop_requested``——生产者在
      完整 sweep 边界受控停止新采集，随后消费者把已有数据全部排空
      （告警 + 受控停止 + 完整排空，绝不静默丢失）。

    注意：生产速度长期高于消费速度时，有限内存无法无限保证持续采集；
    本策略选择"告警 + 受控停止新采集 + 完整排空"，而不是静默丢弃。
    """

    def __init__(
        self,
        maxsize: int,
        *,
        high_watermark: float = 0.75,
        on_high_water=None,
        on_hard_limit=None,
    ) -> None:
        if isinstance(maxsize, bool) or not isinstance(maxsize, int):
            raise TypeError(f"maxsize 必须是 int，实际得到 {type(maxsize).__name__}")
        if maxsize <= 0:
            raise ValueError(f"maxsize 必须是正整数，实际得到 {maxsize!r}")
        if not 0.0 < float(high_watermark) < 1.0:
            raise ValueError("high_watermark 必须在 (0, 1) 区间内")
        self.maxsize = maxsize
        self.high_watermark = float(high_watermark)
        self._hw_threshold = max(1, int(maxsize * self.high_watermark))
        self._on_high_water = on_high_water
        self._on_hard_limit = on_hard_limit
        self._q: queue.Queue[object] = queue.Queue(maxsize=maxsize)
        self._enqueue_times: deque[float] = deque()
        self._lock = threading.Lock()
        self.total_put = 0
        self.total_get = 0
        self.depth_peak = 0
        self.high_water_events = 0
        self.hard_limit_events = 0
        #: 受控停止请求后仍到达的 put 尝试数（调用方不得计入 selected）。
        self.rejected_after_stop = 0
        #: put 超时未能入队的 item（fail-closed 证据，绝不静默消失）。
        self.failed_item: object | None = None
        self.stop_requested = threading.Event()

    def put(self, item: object, *, max_wait_s: float = 30.0) -> bool:
        """入队（保证成功；硬上限满时先触发受控停止再阻塞等待消费者腾位）。

        行为（第1轮复审修复）：

        1. 队列已满且尚未触发硬上限：先触发 hard-limit 回调与
           ``stop_requested``（生产者受控停止新选择），然后**有界阻塞**等待
           消费者腾位——本条触发 Trace 必须被处理，绝不丢弃；
        2. 已触发受控停止后再次 put（生产者尚未到边界）：立即返回 False，
           调用方（worker）不得把该 sweep 计入 selected（先判后选）；
        3. 阻塞超过 ``max_wait_s``（消费者完全卡死）：返回 False，item 保存
           在 ``failed_item``（可追踪，绝不静默消失），调用方必须 fail-closed
           （fatal/failed 上报），不得伪装 drained。

        返回 True 表示 item 已入队（计入 total_put）。
        """
        # 硬上限已满：先接收当前 Trace（有界阻塞等待消费者），同时立即
        # 触发 hard-limit 回调与 stop_requested——生产者在完整 sweep 边界
        # 受控停止新采集。先触发再阻塞，保证不无限等待慢消费者。
        if self._q.qsize() >= self.maxsize and not self.stop_requested.is_set():
            self.hard_limit_events += 1
            self.stop_requested.set()
            if self._on_hard_limit is not None:
                try:
                    self._on_hard_limit(self._q.qsize())
                except Exception:  # noqa: BLE001
                    pass
        elif self._q.qsize() >= self.maxsize and self.stop_requested.is_set():
            # 受控停止已请求且队列仍满：后续 sweep 不再入队，调用方不得
            # 计入 selected。绝不阻塞（不卡 USB 线程）。
            self.rejected_after_stop += 1
            return False
        # 第三次定向修复：先把 enqueued_at_perf 打到 item 上，再真正入队。
        # 关键：必须在 q.put 之前完成赋值（Python 引用是同一个对象，但 put
        # 之前消费者无法通过 get() 取到），且不能因为入队成功/失败而影响
        # ——失败（超时）时消费者永远不会看到它，残留字段无影响。
        enqueued_at = time.perf_counter()
        try:
            setattr(item, "enqueued_at_perf", enqueued_at)
        except Exception:  # noqa: BLE001 - 不支持 setattr 的 item 跳过（极少见）
            pass
        try:
            self._q.put(item, timeout=max(0.1, max_wait_s))
        except queue.Full:
            # 消费者在 max_wait_s 内完全未腾位：fail-closed。item 不从内存
            # 消失——保存在 failed_item 供诊断/上报。
            with self._lock:
                self.failed_item = item
            return False
        with self._lock:
            self.total_put += 1
            self._enqueue_times.append(time.perf_counter())
            depth = self._q.qsize()
            prev_depth = depth - 1
            if depth > self.depth_peak:
                self.depth_peak = depth
        # 高水位告警只在**跨越**阈值时触发（低于→高于一次），不得每次
        # put 都刷屏。硬上限只由"队列已满时尝试 put"（put 前预检查）触发；
        # 填满队列的那条 put 本身正常入队（它是被处理的 Trace，不是触发者）。
        if depth >= self._hw_threshold > prev_depth:
            self.high_water_events += 1
            if self._on_high_water is not None:
                try:
                    self._on_high_water(depth)
                except Exception:  # noqa: BLE001
                    pass
        return True

    def get(self, timeout: float | None = None) -> object | None:
        try:
            item = self._q.get(timeout=timeout)
        except queue.Empty:
            return None
        with self._lock:
            self.total_get += 1
            if self._enqueue_times:
                self._enqueue_times.popleft()
        return item

    def empty(self) -> bool:
        return self._q.empty()

    def statistics(self) -> dict[str, object]:
        with self._lock:
            depth = self._q.qsize()
            oldest = (
                (time.perf_counter() - self._enqueue_times[0]) * 1000.0
                if self._enqueue_times
                else None
            )
            return {
                "maxsize": self.maxsize,
                "depth": depth,
                "depth_peak": self.depth_peak,
                "total_put": self.total_put,
                "total_get": self.total_get,
                "high_water_events": self.high_water_events,
                "hard_limit_events": self.hard_limit_events,
                "oldest_age_ms": oldest,
                "stop_requested": self.stop_requested.is_set(),
                "rejected_after_stop": self.rejected_after_stop,
                "failed_item": self.failed_item is not None,
            }


@dataclass
class SelectedSweep:
    """USB 线程选中的一条完整 sweep，交给处理/保存消费者。

    ``raw`` 形状 (2, N)：``raw[0]`` = S11、``raw[1]`` = S22（双反射后端，
    roadmap 决策门：仅双通道）。``midpoint_monotonic`` 用于按采集中点匹配
    GNSS Fix；调度字段（missed_slots / actual_interval_s / schedule_error_ms）
    来自 :class:`FixedIntervalSweepSelector`。
    """

    trace_index: int
    device_sweep_index: int
    frequency_hz: np.ndarray
    raw: np.ndarray
    cfg: SweepConfig
    midpoint_monotonic: float
    selected_at_utc: str
    missed_slots: int
    actual_interval_s: float | None
    schedule_error_ms: float | None
    # 阶段4 acquisition metadata（worker 组装，consumer 落盘/发送）。
    acquired_at_utc: str = ""
    sweep_started_utc: str = ""
    sweep_completed_utc: str = ""
    scheduled_at_utc: str | None = None
    connection_generation: int = 0
    target_interval_s: float = 0.0
    session_statistics: dict[str, object] = field(default_factory=dict)
    #: 入处理队列的 perf_counter 时刻（默认 0 表示未知——如测试直接构造；
    # processing_started 阶段统计用：入队到 Consumer 开始处理的真实等待）。
    enqueued_at_perf: float = 0.0


def dual_channel_raw(raw: np.ndarray) -> dict[str, np.ndarray]:
    """双反射后端原始数据 (2, N) → S11/S22 双通道 dict（决策门 B）。

    只包含后端真实返回的 S11 / S22，**不**生成 S21/S12 假字段；通道可用性由
    调用方显式传递 ``channels=("S11", "S22")`` 标记。
    """
    array = np.asarray(raw)
    if array.ndim != 2 or array.shape[0] != 2:
        raise ValueError(f"raw 必须是 (2, N) 双反射数组，实际形状 {array.shape}")
    return {
        "S11": np.asarray(array[0], dtype=np.complex128),
        "S22": np.asarray(array[1], dtype=np.complex128),
    }


@dataclass
class CalibrationMeasurement:
    frequency_hz: list[float]
    real: list[float]
    imag: list[float]
    channels: dict[str, dict[str, list[float]]] | None = None

    @classmethod
    def from_points(cls, points: list[tuple[float, complex]]) -> "CalibrationMeasurement":
        return cls(
            frequency_hz=[float(frequency) for frequency, _ in points],
            real=[float(s11.real) for _, s11 in points],
            imag=[float(s11.imag) for _, s11 in points],
        )

    @classmethod
    def from_sparameter_points(cls, points: list[tuple[float, dict[str, complex]]], primary: str = "S11") -> "CalibrationMeasurement":
        frequency_hz = [float(frequency) for frequency, _ in points]
        channels: dict[str, dict[str, list[float]]] = {}
        for name in ("S11", "S21", "S12", "S22"):
            values = [sparams[name] for _frequency, sparams in points if name in sparams]
            if len(values) == len(points):
                channels[name] = {
                    "real": [float(value.real) for value in values],
                    "imag": [float(value.imag) for value in values],
                }
        primary_values = [
            sparams.get(primary) or sparams.get("S11") or next(iter(sparams.values()))
            for _frequency, sparams in points
        ]
        return cls(
            frequency_hz=frequency_hz,
            real=[float(value.real) for value in primary_values],
            imag=[float(value.imag) for value in primary_values],
            channels=channels,
        )

    def as_complex_by_frequency(self, channel: str | None = None) -> dict[float, complex]:
        if channel and self.channels and channel in self.channels:
            real_values = self.channels[channel]["real"]
            imag_values = self.channels[channel]["imag"]
            return {
                float(frequency): complex(float(real), float(imag))
                for frequency, real, imag in zip(self.frequency_hz, real_values, imag_values)
            }
        return {
            float(frequency): complex(float(real), float(imag))
            for frequency, real, imag in zip(self.frequency_hz, self.real, self.imag)
        }


CALIBRATION_STEPS = [
    ("P1_Open", "P1 Open"),
    ("P1_Short", "P1 Short"),
    ("P1_Load", "P1 Load"),
    ("P2_Open", "P2 Open"),
    ("P2_Short", "P2 Short"),
    ("P2_Load", "P2 Load"),
    ("Through", "Through"),
]
CALIBRATION_STEP_LABELS = dict(CALIBRATION_STEPS)
P1_OSL_KEYS = ("P1_Open", "P1_Short", "P1_Load")
LEGACY_OSL_MAP = {"P1_Open": "Open", "P1_Short": "Short", "P1_Load": "Load"}
S_PARAMETERS = ("S11", "S21", "S12", "S22")
UI_CHANNELS = ("S11", "S22")
CHANNEL_LABELS = {"S11": "HH (S11)", "S22": "VV (S22)"}
CHANNEL_COLORS = {"S11": "#0077b6", "S22": "#6d597a"}


class BScanWidget(QtWidgets.QWidget):
    def __init__(self, title: str) -> None:
        super().__init__()
        self.title = title
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)
        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.title_label.setStyleSheet("font-weight: 600;")
        self.figure = Figure(figsize=(4.0, 1.85), dpi=100)
        self.figure.subplots_adjust(left=0.12, right=0.90, bottom=0.24, top=0.95)
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumHeight(220)
        self.canvas.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel("Trace")
        self.ax.set_ylabel("Time (ns)")
        self.image_artist = None
        self.colorbar = None
        self.cmap = "gray"
        self.aspect_ratio = 2.0
        layout.addWidget(self.title_label)
        layout.addWidget(self.canvas, 1)
        self._draw_empty()

    def set_aspect_ratio(self, width_to_height: float) -> None:
        self.aspect_ratio = min(max(float(width_to_height), 0.5), 6.0)
        self.ax.set_box_aspect(1.0 / self.aspect_ratio)
        self.canvas.draw_idle()

    def set_colormap(self, name: str) -> None:
        self.cmap = "gray" if name == "gray" else "jet"
        if self.image_artist is not None:
            self.image_artist.set_cmap(self.cmap)
            self.canvas.draw_idle()

    def set_data(
        self,
        image: np.ndarray,
        time_ns: np.ndarray | None,
        start_trace: int = 1,
    ) -> None:
        self.figure.clear()
        self.figure.subplots_adjust(left=0.12, right=0.90, bottom=0.24, top=0.95)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel("Trace")
        self.ax.set_ylabel("Time (ns)")
        self.ax.set_box_aspect(1.0 / self.aspect_ratio)
        trace_count = image.shape[1]
        x0 = float(start_trace) - 0.5
        x1 = x0 + trace_count
        if time_ns is not None and len(time_ns) > 1:
            extent = [x0, x1, float(time_ns[-1]), float(time_ns[0])]
        else:
            extent = [x0, x1, image.shape[0], 0]
        finite = image[np.isfinite(image)]
        if finite.size:
            vmin, vmax = np.percentile(finite, [2, 98])
            if math.isclose(vmin, vmax):
                vmin, vmax = float(finite.min()), float(finite.max())
        else:
            vmin, vmax = 0.0, 1.0
        self.image_artist = self.ax.imshow(
            image,
            cmap=self.cmap,
            aspect="auto",
            interpolation="nearest",
            origin="upper",
            extent=extent,
            vmin=vmin,
            vmax=vmax,
        )
        self.colorbar = self.figure.colorbar(self.image_artist, ax=self.ax, fraction=0.046, pad=0.02)
        self.colorbar.ax.tick_params(labelsize=8)
        self.canvas.draw_idle()

    def clear(self) -> None:
        self._draw_empty()

    def _draw_empty(self) -> None:
        self.figure.clear()
        self.figure.subplots_adjust(left=0.12, right=0.96, bottom=0.24, top=0.95)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel("Trace")
        self.ax.set_ylabel("Time (ns)")
        self.ax.set_box_aspect(1.0 / self.aspect_ratio)
        self.ax.set_xlim(0.5, BSCAN_WINDOW_TRACES + 0.5)
        self.ax.set_ylim(40, 0)
        self.ax.grid(True, alpha=0.2)
        self.image_artist = None
        self.colorbar = None
        self.canvas.draw_idle()


def osl_measurements_for_port(measurements: dict[str, CalibrationMeasurement], port: int) -> dict[str, CalibrationMeasurement]:
    if port == 1:
        mapping = {"P1_Open": "Open", "P1_Short": "Short", "P1_Load": "Load"}
    elif port == 2:
        mapping = {"P2_Open": "Open", "P2_Short": "Short", "P2_Load": "Load"}
    else:
        raise ValueError(f"Unsupported calibration port: {port}")
    selected: dict[str, CalibrationMeasurement] = {}
    for key, legacy_key in mapping.items():
        measurement = measurements.get(key) or measurements.get(legacy_key)
        if measurement:
            selected[legacy_key] = measurement
    return selected


def s11_osl_measurements(measurements: dict[str, CalibrationMeasurement]) -> dict[str, CalibrationMeasurement]:
    return osl_measurements_for_port(measurements, 1)


def nmea_coord(value: str, hemi: str) -> float | None:
    if not value or not hemi:
        return None
    dot = value.find(".")
    deg_len = dot - 2 if dot >= 0 else len(value) - 2
    deg = float(value[:deg_len])
    minutes = float(value[deg_len:])
    signed = deg + minutes / 60.0
    return -signed if hemi in {"S", "W"} else signed


def parse_gga(line: str) -> GgaFix:
    parts = line.strip().split(",")
    if len(parts) < 10 or not parts[0].endswith("GGA"):
        return GgaFix(raw=line)
    return GgaFix(
        raw=line,
        utc=parts[1],
        latitude=nmea_coord(parts[2], parts[3]),
        longitude=nmea_coord(parts[4], parts[5]),
        fix_quality=int(parts[6] or 0),
        satellites=int(parts[7] or 0),
        altitude_m=float(parts[9]) if parts[9] else None,
    )


def parse_rmc(line: str) -> GgaFix:
    parts = line.strip().split(",")
    if len(parts) < 10 or not parts[0].endswith("RMC"):
        return GgaFix(raw=line)
    valid = parts[2] == "A"
    return GgaFix(
        raw=line,
        utc=parts[1],
        latitude=nmea_coord(parts[3], parts[4]),
        longitude=nmea_coord(parts[5], parts[6]),
        fix_quality=1 if valid else 0,
        satellites=0,
        altitude_m=None,
    )


def calibration_from_measurements(
    measurements: dict[str, CalibrationMeasurement],
    filename: Path | None = None,
    port: int = 1,
) -> OSLCalibration:
    osl_measurements = osl_measurements_for_port(measurements, port)
    missing = [name for name in ("Open", "Short", "Load") if name not in osl_measurements]
    if missing:
        raise ValueError(f"P{port} 一端口校准缺少标准件: {', '.join(missing)}")

    channel = "S11" if port == 1 else "S22"
    measured = {name: measurement.as_complex_by_frequency(channel) for name, measurement in osl_measurements.items()}
    common_freqs = sorted(set(measured["Open"]) & set(measured["Short"]) & set(measured["Load"]))
    if not common_freqs:
        raise ValueError("Open/Short/Load 的频率点不一致，不能生成校准")

    points: list[OSLPoint] = []
    for frequency in common_freqs:
        points.append(
            compute_osl_point(
                frequency,
                open_measured=measured["Open"][frequency],
                short_measured=measured["Short"][frequency],
                load_measured=measured["Load"][frequency],
                open_actual=complex(1.0, 0.0),
                short_actual=complex(-1.0, 0.0),
                load_actual=complex(0.0, 0.0),
            )
        )
    return OSLCalibration(points, filename or Path("session_osl_calibration.json"))


def calibration_points_payload(measurement: CalibrationMeasurement) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for idx, (frequency, real, imag) in enumerate(zip(measurement.frequency_hz, measurement.real, measurement.imag)):
        channels = {
            channel: {
                "real": values["real"][idx],
                "imag": values["imag"][idx],
            }
            for channel, values in (measurement.channels or {}).items()
            if idx < len(values["real"]) and idx < len(values["imag"])
        }
        rows.append({"frequency": frequency, "real": real, "imag": imag, "channels": channels})
    return rows


def write_calibration_file(path: Path, measurements: dict[str, CalibrationMeasurement], cfg: SweepConfig) -> None:
    payload = {
        "format": "uav_gpr_solt_calibration_v2",
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "method": "SOLT/SLOT; one-port S11 uses P1 Open/Short/Load, two-port workflow records P1/P2 OSL plus Through",
        "s_parameter": "S11 now; S11/S21/S12/S22 ready for two-port extension",
        "sweep_config": asdict(cfg),
        "measurements": [
            {
                "type": name,
                "data": {"points": calibration_points_payload(measurement)},
            }
            for name, measurement in measurements.items()
        ],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def read_calibration_measurements(path: Path) -> dict[str, CalibrationMeasurement]:
    data = json.loads(path.read_text(encoding="utf-8"))
    measurements: dict[str, CalibrationMeasurement] = {}
    for measurement in data.get("measurements", []):
        name = measurement.get("type")
        raw_points = measurement.get("data", {}).get("points", [])
        if name in set(CALIBRATION_STEP_LABELS) | {"Open", "Short", "Load"}:
            channels: dict[str, dict[str, list[float]]] = {}
            for point in raw_points:
                for channel, value in point.get("channels", {}).items():
                    channels.setdefault(channel, {"real": [], "imag": []})
                    channels[channel]["real"].append(float(value["real"]))
                    channels[channel]["imag"].append(float(value["imag"]))
            measurements[name] = CalibrationMeasurement(
                frequency_hz=[float(point["frequency"]) for point in raw_points],
                real=[float(point["real"]) for point in raw_points],
                imag=[float(point["imag"]) for point in raw_points],
                channels=channels or None,
            )
    return measurements


class GnssReader:
    def __init__(self, port: str, baud: int = 115200) -> None:
        self.port = port
        self.baud = baud
        self.ser: serial.Serial | None = None

    def open(self) -> None:
        if self.ser and self.ser.is_open:
            return
        self.ser = serial.Serial(self.port, self.baud, timeout=0.8)
        self.ser.write(b"log gpgga\r\n")
        self.ser.write(b"log gprmc\r\n")

    def close(self) -> None:
        if self.ser:
            self.ser.close()
        self.ser = None

    def read_gga(self, timeout_s: float = 2.0) -> GgaFix:
        self.open()
        assert self.ser is not None
        deadline = time.monotonic() + timeout_s
        last = ""
        while time.monotonic() < deadline:
            line = self.ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            last = line
            if line.startswith(("$GPGGA", "$GNGGA", "$BDGGA")):
                fix = parse_gga(line)
                if fix.latitude is not None and fix.longitude is not None:
                    return fix
            if line.startswith(("$GPRMC", "$GNRMC", "$BDRMC")):
                fix = parse_rmc(line)
                if fix.latitude is not None and fix.longitude is not None:
                    return fix
        return GgaFix(raw=last)


def db20(values: np.ndarray) -> np.ndarray:
    return 20.0 * np.log10(np.maximum(np.abs(values), 1e-15))


def normalize_trace_for_display(trace: np.ndarray) -> np.ndarray:
    peak = float(np.max(np.abs(trace))) if trace.size else 0.0
    if peak <= 1e-15:
        return trace.copy()
    return trace / peak


def robust_db_range(values: list[np.ndarray], default: tuple[float, float]) -> tuple[float, float]:
    if not values:
        return default
    merged = np.concatenate([np.asarray(v, dtype=float).ravel() for v in values if len(v)])
    merged = merged[np.isfinite(merged)]
    if merged.size == 0:
        return default
    low, high = np.percentile(merged, [5, 95])
    if math.isclose(float(low), float(high), abs_tol=1e-6):
        low, high = float(merged.min()), float(merged.max())
    pad = max((high - low) * 0.15, 3.0)
    return float(low - pad), float(high + pad)


def sparameter_arrays(points: list[tuple[float, dict[str, complex]]]) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    frequency_hz = np.array([frequency for frequency, _sparams in points], dtype=float)
    arrays = {
        name: np.array([sparams[name] for _frequency, sparams in points], dtype=np.complex128)
        for name in S_PARAMETERS
    }
    return frequency_hz, arrays


def interpolate_measurement_channel(
    measurement: CalibrationMeasurement,
    channel: str,
    frequency_hz: np.ndarray,
) -> np.ndarray:
    measured = measurement.as_complex_by_frequency(channel)
    cal_freq = np.array(sorted(measured), dtype=float)
    cal_values = np.array([measured[float(f)] for f in cal_freq], dtype=np.complex128)
    real = np.interp(frequency_hz, cal_freq, cal_values.real)
    imag = np.interp(frequency_hz, cal_freq, cal_values.imag)
    return real + 1j * imag


def apply_reflection_and_through_calibration(
    frequency_hz: np.ndarray,
    raw_sparams: dict[str, np.ndarray],
    calibration_p1: OSLCalibration | None,
    calibration_p2: OSLCalibration | None,
    through: CalibrationMeasurement | None,
) -> dict[str, np.ndarray]:
    display = {name: values.copy() for name, values in raw_sparams.items()}
    if calibration_p1:
        display["S11"] = np.array([calibration_p1.correct(f, s) for f, s in zip(frequency_hz, raw_sparams["S11"])], dtype=np.complex128)
    if calibration_p2:
        display["S22"] = np.array([calibration_p2.correct(f, s) for f, s in zip(frequency_hz, raw_sparams["S22"])], dtype=np.complex128)
    if through:
        for channel in ("S21", "S12"):
            if channel not in raw_sparams:
                continue
            ref = interpolate_measurement_channel(through, channel, frequency_hz)
            display[channel] = np.divide(raw_sparams[channel], ref, out=raw_sparams[channel].copy(), where=np.abs(ref) > 1e-15)
    return display


def bpf_frequency_points(frequency_hz: np.ndarray) -> list[float]:
    """Return the four actual-frequency BPF breakpoints used for this trace."""
    frequency = np.asarray(frequency_hz, dtype=np.float64)
    frequency_bpf_weights(frequency)
    f0 = float(frequency[0])
    f3 = float(frequency[-1])
    span = f3 - f0
    return [f0, f0 + span / 4.0, f3 - span / 4.0, f3]


def processing_metadata(frequency_hz: np.ndarray, cfg: SweepConfig) -> dict[str, object]:
    return {
        "bpf": {
            "rule": cfg.bpf_rule,
            "frequency_hz": bpf_frequency_points(frequency_hz),
        },
        "dewow": {
            "enabled": bool(cfg.dewow_enabled),
            "window_ns": float(cfg.dewow_window_ns),
        },
        "dewow_enabled": bool(cfg.dewow_enabled),
        "dewow_window_ns": float(cfg.dewow_window_ns),
        "ifft_points": int(cfg.ifft_points),
    }


def make_time_trace(
    frequency_hz: np.ndarray,
    complex_spectrum: np.ndarray,
    cfg: SweepConfig,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply frequency BPF, then IFFT and Dewow using the actual frequency grid."""
    filtered_spectrum = apply_frequency_bpf(frequency_hz, complex_spectrum)
    return frequency_to_time_trace(
        frequency_hz,
        filtered_spectrum,
        ifft_points=cfg.ifft_points,
        time_window_ns=cfg.time_window_ns,
        dewow_window_ns=cfg.dewow_window_ns,
        apply_dewow=cfg.dewow_enabled,
    )


def process_trace_pipeline(
    frequency_hz: np.ndarray,
    raw_sparams: dict[str, np.ndarray],
    cfg: SweepConfig,
    calibration_p1: OSLCalibration | None = None,
    calibration_p2: OSLCalibration | None = None,
    through_measurement: CalibrationMeasurement | None = None,
    background: np.ndarray | None = None,
    background_frequency_hz: np.ndarray | None = None,
) -> tuple[dict[str, np.ndarray], np.ndarray, dict[str, np.ndarray]]:
    """Run calibration/background/BPF/IFFT/Dewow for all four S-parameters."""
    scan_frequency = np.asarray(frequency_hz, dtype=np.float64)
    if (background_frequency_hz is None) != (background is None):
        raise ValueError("background_frequency_hz and background must be provided together")
    if background is not None and background_frequency_hz is not None:
        background_frequency = np.asarray(background_frequency_hz, dtype=np.float64)
        if background_frequency.shape != scan_frequency.shape:
            raise ValueError(
                "background frequency axis shape does not match the current scan frequency axis"
            )
        if not np.all(np.isfinite(background_frequency)):
            raise ValueError("background frequency axis must contain only finite values")
        if not np.array_equal(scan_frequency, background_frequency):
            raise ValueError("background frequency axis does not match the current scan frequency axis")
        background_values = np.asarray(background, dtype=np.complex128)
        if background_values.shape != scan_frequency.shape:
            raise ValueError("background trace length does not match the current scan frequency axis")
        if not np.all(np.isfinite(background_values.real)) or not np.all(np.isfinite(background_values.imag)):
            raise ValueError("background trace must contain only finite values")
    else:
        background_values = None

    display = apply_reflection_and_through_calibration(
        frequency_hz,
        raw_sparams,
        calibration_p1,
        calibration_p2,
        through_measurement,
    )
    if background_values is not None:
        display["S11"] = display["S11"] - background_values

    # 处理 raw_sparams 中真实存在的通道（legacy 四通道 / continuous 双通道）。
    channels = tuple(name for name in S_PARAMETERS if name in raw_sparams)
    if not channels:
        raise ValueError("raw_sparams 必须包含至少一个 S-parameter 通道")

    trace_by_channel: dict[str, np.ndarray] = {}
    time_ns: np.ndarray | None = None
    for name in channels:
        channel_time_ns, channel_trace = make_time_trace(frequency_hz, display[name], cfg)
        if time_ns is None:
            time_ns = channel_time_ns
        elif not np.array_equal(time_ns, channel_time_ns):
            raise ValueError("all S-parameter channels must produce the same time axis")
        trace_by_channel[name] = channel_trace
    assert time_ns is not None
    return display, time_ns, trace_by_channel


def validate_trace_by_channel(
    time_ns: np.ndarray,
    trace_by_channel: dict[str, np.ndarray],
    *,
    channels: Sequence[str] | None = None,
) -> dict[str, np.ndarray]:
    channel_names = tuple(channels) if channels is not None else S_PARAMETERS
    time_values = np.asarray(time_ns)
    if time_values.ndim != 1:
        raise ValueError("time_ns must be one-dimensional")
    missing = [name for name in channel_names if name not in trace_by_channel]
    if missing:
        raise ValueError(f"trace_by_channel is missing channels: {', '.join(missing)}")

    validated: dict[str, np.ndarray] = {}
    for name in channel_names:
        values = np.asarray(trace_by_channel[name])
        if values.ndim != 1 or values.size != time_values.size:
            raise ValueError(
                f"trace_{name} length must equal time_ns length ({time_values.size})"
            )
        validated[name] = values
    return validated


def save_trace(
    out_dir: Path,
    index: int,
    cfg: SweepConfig,
    frequency_hz: np.ndarray,
    raw_sparams: dict[str, np.ndarray],
    display_sparams: dict[str, np.ndarray],
    time_ns: np.ndarray,
    trace_by_channel: dict[str, np.ndarray],
    fix: GgaFix,
    *,
    channels: Sequence[str] | None = None,
    acquisition: dict[str, object] | None = None,
    flight: dict[str, object] | None = None,
) -> Path:
    channel_names = tuple(channels) if channels is not None else S_PARAMETERS
    validated_traces = validate_trace_by_channel(time_ns, trace_by_channel, channels=channel_names)
    # 阶段4：只增加字段，绝不删除既有 key。保存时刻由本函数统一记录一次，
    # NPZ 与 run_metadata.jsonl 使用同一 saved_at_utc。
    final_acquisition = (
        {**acquisition, "saved_at_utc": utc_iso_now()} if acquisition is not None else None
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    csv_path = out_dir / f"trace_{index:05d}_{stamp}.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        header = ["frequency_hz", "frequency_mhz"]
        for name in channel_names:
            header += [
                f"{name.lower()}_real_raw",
                f"{name.lower()}_imag_raw",
                f"{name.lower()}_db_raw",
                f"{name.lower()}_real_display",
                f"{name.lower()}_imag_display",
                f"{name.lower()}_db_display",
            ]
        writer.writerow(header)
        for idx, f in enumerate(frequency_hz):
            row: list[float] = [float(f), float(f / 1e6)]
            for name in channel_names:
                raw = raw_sparams[name][idx]
                disp = display_sparams[name][idx]
                row += [raw.real, raw.imag, db20(np.array([raw]))[0], disp.real, disp.imag, db20(np.array([disp]))[0]]
            writer.writerow(row)

    npz_path = csv_path.with_suffix(".npz")
    npz_payload = {
        "config": json.dumps(asdict(cfg), ensure_ascii=False),
        "processing": json.dumps(processing_metadata(frequency_hz, cfg), ensure_ascii=False),
        "frequency_hz": frequency_hz,
        "time_ns": time_ns,
        "trace": validated_traces[channel_names[0]],
        "gnss": json.dumps(asdict(fix), ensure_ascii=False),
        "channels": json.dumps(list(channel_names), ensure_ascii=False),
    }
    if final_acquisition is not None:
        npz_payload["acquisition"] = json.dumps(final_acquisition, ensure_ascii=False)
    if flight is not None:
        npz_payload["flight"] = json.dumps(flight, ensure_ascii=False)
    for name in channel_names:
        npz_payload[f"raw_{name}_real"] = raw_sparams[name].real
        npz_payload[f"raw_{name}_imag"] = raw_sparams[name].imag
        npz_payload[f"display_{name}_real"] = display_sparams[name].real
        npz_payload[f"display_{name}_imag"] = display_sparams[name].imag
        npz_payload[f"trace_{name}"] = validated_traces[name]
    np.savez_compressed(
        npz_path,
        **npz_payload,
    )

    meta = {
        "index": index,
        "timestamp": stamp,
        "frequency_range_hz": [float(frequency_hz[0]), float(frequency_hz[-1])],
        "s_parameter": list(channel_names),
        "channels": list(channel_names),
        "config": asdict(cfg),
        "processing": processing_metadata(frequency_hz, cfg),
        "gnss": asdict(fix),
        "csv": csv_path.name,
        "npz": npz_path.name,
    }
    if final_acquisition is not None:
        meta["acquisition"] = final_acquisition
    if flight is not None:
        meta["flight"] = flight
    meta_path = out_dir / "run_metadata.jsonl"
    try:
        with meta_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(meta, ensure_ascii=False) + "\n")
    except PermissionError:
        sidecar_path = csv_path.with_suffix(".metadata.json")
        sidecar_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    return csv_path


def trace_npz_bytes(
    index: int,
    cfg: SweepConfig,
    frequency_hz: np.ndarray,
    raw_sparams: dict[str, np.ndarray],
    display_sparams: dict[str, np.ndarray],
    time_ns: np.ndarray,
    trace_by_channel: dict[str, np.ndarray],
    fix: GgaFix,
    *,
    channels: Sequence[str] | None = None,
    acquisition: dict[str, object] | None = None,
    flight: dict[str, object] | None = None,
) -> bytes:
    channel_names = tuple(channels) if channels is not None else S_PARAMETERS
    validated_traces = validate_trace_by_channel(time_ns, trace_by_channel, channels=channel_names)
    payload: dict[str, object] = {
        "index": np.array(index, dtype=np.int64),
        "config": json.dumps(asdict(cfg), ensure_ascii=False),
        "processing": json.dumps(processing_metadata(frequency_hz, cfg), ensure_ascii=False),
        "frequency_hz": np.asarray(frequency_hz),
        "time_ns": np.asarray(time_ns),
        "trace": validated_traces[channel_names[0]],
        "gnss": json.dumps(asdict(fix), ensure_ascii=False),
        "channels": json.dumps(list(channel_names), ensure_ascii=False),
    }
    if acquisition is not None:
        # 调用方负责最终化 acquisition（含 saved_at_utc）；这里逐字写入，
        # 绝不覆盖已确定的 saved_at_utc，保证 header 与 NPZ 内部一致。
        payload["acquisition"] = json.dumps(acquisition, ensure_ascii=False)
    if flight is not None:
        payload["flight"] = json.dumps(flight, ensure_ascii=False)
    for name in channel_names:
        payload[f"raw_{name}_real"] = raw_sparams[name].real
        payload[f"raw_{name}_imag"] = raw_sparams[name].imag
        payload[f"display_{name}_real"] = display_sparams[name].real
        payload[f"display_{name}_imag"] = display_sparams[name].imag
        payload[f"trace_{name}"] = np.asarray(validated_traces[name], dtype=float)
    buffer = io.BytesIO()
    np.savez_compressed(buffer, **payload)
    return buffer.getvalue()


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("socket closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def send_remote_frame(sock: socket.socket, header: dict[str, object], payload: bytes = b"") -> None:
    header_bytes = json.dumps(header, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    sock.sendall(struct.pack("!I", len(header_bytes)))
    sock.sendall(header_bytes)
    sock.sendall(struct.pack("!Q", len(payload)))
    if payload:
        sock.sendall(payload)


def recv_remote_header(sock: socket.socket) -> dict[str, object]:
    header_len = struct.unpack("!I", recv_exact(sock, 4))[0]
    header = json.loads(recv_exact(sock, header_len).decode("utf-8"))
    payload_len = struct.unpack("!Q", recv_exact(sock, 8))[0]
    if payload_len:
        raise ValueError(f"unexpected ack payload: {payload_len} bytes")
    return header


def recv_remote_frame(sock: socket.socket) -> tuple[dict[str, object], bytes]:
    header_len = struct.unpack("!I", recv_exact(sock, 4))[0]
    header = json.loads(recv_exact(sock, header_len).decode("utf-8"))
    payload_len = struct.unpack("!Q", recv_exact(sock, 8))[0]
    payload = recv_exact(sock, payload_len) if payload_len else b""
    return header, payload


class _QueuedRemoteFrame:
    """RemoteSender 内部队列的包装（第三次定向修复）。

    携带 ``enqueued_at_perf`` 让 ``remote_send_started`` 阶段测量真正
    的"入队→开始 sendall"等待时间，而不仅仅是 ``queue.get`` → sendall
    的几微秒 Python 开销。
    """

    __slots__ = ("header", "payload", "enqueued_at_perf")

    def __init__(self, header: dict[str, object], payload: bytes, enqueued_at_perf: float) -> None:
        self.header = header
        self.payload = payload
        self.enqueued_at_perf = enqueued_at_perf

    def __iter__(self):
        # 兼容历史测试 ``header, payload = sender.queue.get_nowait()``。
        # 生产代码路径走 .header/.payload 显式字段，不依赖 __iter__。
        return iter((self.header, self.payload))


class RemoteSender:
    def __init__(self, host: str, port: int, run_id: str, status_callback, command_callback=None) -> None:
        self.host = host
        self.port = port
        self.run_id = run_id
        self.status_callback = status_callback
        self.command_callback = command_callback
        # 第三次定向修复：queue 元素改用 _QueuedRemoteFrame 包装，携带
        # 入队时间戳供 remote_send_started 阶段使用。
        self.queue: queue.Queue[_QueuedRemoteFrame | None] = queue.Queue()
        self._stop = threading.Event()
        self._busy = threading.Event()
        self._thread = threading.Thread(target=self._run, name="uav-gpr-remote-sender", daemon=True)
        self._started = False
        self._sock: socket.socket | None = None
        # ---- 真实排空语义（第1轮复审修复）----
        # 未完成消息计数：入队 +1；trace 收到匹配 ack_trace、状态消息成功
        # send 完成各 -1。覆盖三种"未完成"形态：仍在 queue 中、已取出正在
        # 发送（in-flight）、发送失败进入 pending 重试状态。queue.empty()
        # 不能作为排空完成的唯一判断依据。
        self._outstanding = 0
        self._outstanding_lock = threading.Lock()
        #: 已取出、发送失败正在重试的消息（诊断/drain 判断用；由 _run 维护）。
        self._pending_item: _QueuedRemoteFrame | None = None

    def _note_enqueued(self) -> None:
        with self._outstanding_lock:
            self._outstanding += 1

    def _note_completed(self) -> None:
        with self._outstanding_lock:
            self._outstanding = max(0, self._outstanding - 1)

    def outstanding_count(self) -> int:
        """未完成消息数（queue + in-flight + pending 重试）。"""
        with self._outstanding_lock:
            return self._outstanding

    def start(self) -> None:
        self._started = True
        self._thread.start()

    def stop(self, timeout_ms: int = 3000, *, drain: bool = True) -> bool:
        """停止发送线程（drain=True 时等待全部消息真正完成）。

        ``drain=True``（默认）：等待所有 trace 消息收到匹配 ack_trace、所有
        状态消息成功发送，且已取出的 pending 重试消息也计入未完成数。
        - 等待成功 → 正常停止（置 stop、哨兵、关 socket、有界 join）；
        - 等待超时 → **返回 False**，保持线程存活、socket 重连能力、pending
          与队列内容完全不动（不置 _stop、不插退出哨兵、不关线程）；
          用户/上层可再次调用 stop 继续尝试排空；
        - ``drain=False`` 立即强制停止，明确放弃未发送内容（仅强制关闭场景）。

        返回线程是否已退出；绝不在 sender 自身线程 join 自己；未启动过的
        sender 调用安全（drain 语义对未启动线程为：无未完成即成功）。
        """
        current = threading.current_thread()
        if drain and self._started and self._thread is not current:
            deadline = time.monotonic() + timeout_ms / 1000.0
            while time.monotonic() < deadline:
                if self.outstanding_count() == 0:
                    break
                time.sleep(0.02)
            if self.outstanding_count() != 0:
                # 排空失败：不设置 _stop、不插哨兵、不关闭任何资源。
                # 线程继续按原节奏重试；再次调用 stop 可继续排空。
                return False
        if drain and not self._started:
            # 未启动：无可发送内容，视为排空完成。
            if self.outstanding_count() != 0:
                return False
        self._stop.set()
        self.queue.put(None)
        self._close()
        thread = self._thread
        if (
            self._started
            and thread is not None
            and thread is not current
            and thread.is_alive()
        ):
            thread.join(timeout_ms / 1000.0)
        return not (self._started and thread is not None and thread.is_alive())

    def enqueue_trace(
        self,
        index: int,
        cfg: SweepConfig,
        frequency_hz: np.ndarray,
        raw_sparams: dict[str, np.ndarray],
        display_sparams: dict[str, np.ndarray],
        time_ns: np.ndarray,
        trace_by_channel: dict[str, np.ndarray],
        fix: GgaFix,
        *,
        channels: Sequence[str] | None = None,
        acquisition: dict[str, object] | None = None,
        flight: dict[str, object] | None = None,
    ) -> None:
        # remote_enqueued：payload 组装 + 成功入队的真实耗时（第二次定向
        # 修复：不再用固定 0.0 冒充耗时）。
        _enqueue_t0 = time.perf_counter()
        channel_names = tuple(channels) if channels is not None else S_PARAMETERS
        # 阶段4 修正：同一遥测 trace 只生成一次 saved_at_utc，得到最终
        # final_acquisition；header["acquisition"]、NPZ 内部 acquisition 与顶层
        # acquired_at_utc 全部取自同一个 final_acquisition，逐字段完全一致。
        final_acquisition = (
            {**acquisition, "saved_at_utc": utc_iso_now()} if acquisition is not None else None
        )
        payload = trace_npz_bytes(
            index,
            cfg,
            frequency_hz,
            raw_sparams,
            display_sparams,
            time_ns,
            trace_by_channel,
            fix,
            channels=channel_names,
            acquisition=final_acquisition,
            flight=flight,
        )
        header = {
            "type": "trace",
            "protocol": "uav-gpr-telemetry-v1",
            "run_id": self.run_id,
            "trace_index": index,
            "timestamp": utc_timestamp(),
            "payload_format": "npz",
            "payload_bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
            "config": asdict(cfg),
            "processing": processing_metadata(frequency_hz, cfg),
            "gnss": asdict(fix),
            "channels": list(channel_names),
        }
        # 阶段4：只增加遥测 header 字段，保持 uav-gpr-telemetry-v1 framing。
        # legacy 调用（无 acquisition）不新增任何字段，保持旧 schema。
        if final_acquisition is not None:
            header["acquisition"] = final_acquisition
            header["acquired_at_utc"] = final_acquisition["acquired_at_utc"]
        if flight is not None:
            header["flight"] = flight
        self._note_enqueued()
        _record_stage("remote_enqueued", time.perf_counter() - _enqueue_t0)
        self.queue.put(_QueuedRemoteFrame(header, payload, time.perf_counter()))

    def enqueue_status(self, state: str, *, reason: str | None = None, **fields: object) -> None:
        """发送 acquisition_status 状态消息（第1轮检查点D，向后兼容）。

        在既有 uav-gpr-telemetry-v1 framing 上新增 ``type="acquisition_status"``
        空载荷消息；旧接收端不认识时会走"忽略未知消息"分支。不改变 trace
        framing、SHA-256 或 ack_trace 语义。
        """
        header: dict[str, object] = {
            "type": "acquisition_status",
            "protocol": "uav-gpr-telemetry-v1",
            "run_id": self.run_id,
            "state": state,
            "timestamp": utc_timestamp(),
        }
        if reason is not None:
            header["reason"] = reason
        for key, value in fields.items():
            if value is not None:
                header[key] = value
        self._note_enqueued()
        self.queue.put(_QueuedRemoteFrame(header, b"", time.perf_counter()))

    def _connect(self) -> socket.socket:
        sock = socket.create_connection((self.host, self.port), timeout=5.0)
        sock.settimeout(0.5)
        send_remote_frame(
            sock,
            {
                "type": "hello",
                "protocol": "uav-gpr-telemetry-v1",
                "run_id": self.run_id,
                "timestamp": utc_timestamp(),
                "supports_commands": True,
            },
        )
        ack, _payload = self._read_frame(sock)
        if ack.get("type") != "ack":
            raise ConnectionError(f"unexpected hello ack: {ack}")
        self.status_callback(f"Remote connected: {self.host}:{self.port}")
        return sock

    def _close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _read_frame(self, sock: socket.socket) -> tuple[dict[str, object], bytes]:
        while not self._stop.is_set():
            try:
                return recv_remote_frame(sock)
            except socket.timeout:
                continue
        raise ConnectionError("remote sender stopped")

    def _handle_command(self, header: dict[str, object]) -> None:
        action = str(header.get("action") or "")
        params = header.get("params") if isinstance(header.get("params"), dict) else {}
        if self.command_callback is not None:
            self.command_callback(action, params)
        if self._sock is not None:
            send_remote_frame(
                self._sock,
                {
                    "type": "command_ack",
                    "protocol": "uav-gpr-telemetry-v1",
                    "run_id": self.run_id,
                    "action": action,
                    "timestamp": utc_timestamp(),
                },
            )

    def _wait_for_trace_ack(self, sock: socket.socket, trace_index: object) -> None:
        while not self._stop.is_set():
            header, _payload = self._read_frame(sock)
            msg_type = header.get("type")
            if msg_type == "ack_trace" and header.get("trace_index") == trace_index:
                self._note_completed()
                return
            if msg_type == "command":
                self._handle_command(header)
                continue
            raise ConnectionError(f"unexpected trace ack: {header}")

    def _run(self) -> None:
        # pending：已从 queue 取出但发送失败、正在重试的消息。它**保留**在
        # 未完成计数中（发送成功/收到 ACK 才减），drain 不得因 queue 空而
        # 误判完成。
        pending: _QueuedRemoteFrame | None = None
        self._pending_item = None
        while not self._stop.is_set():
            try:
                if self._sock is None:
                    self._sock = self._connect()
                try:
                    item = pending or self.queue.get(timeout=0.2)
                except queue.Empty:
                    try:
                        header, _payload = recv_remote_frame(self._sock)
                    except socket.timeout:
                        continue
                    if header.get("type") == "command":
                        self._handle_command(header)
                        continue
                    raise ConnectionError(f"unexpected remote frame: {header}")
                pending = None
                self._pending_item = None
                if item is None:
                    break
                # 第三次定向修复：从 enqueued_at_perf 开始测量真实入队等待，
                # 不再被几微秒的 Python queue.get 耗时冒充。
                _enqueued_at = item.enqueued_at_perf
                header, payload = item.header, item.payload
                self._busy.set()
                try:
                    _send_t0 = time.perf_counter()
                    is_status = header.get("type") == "acquisition_status"
                    if not is_status:
                        # remote_send_started 仅记录 trace 消息（避免
                        # acquisition_status 污染阶段统计）；状态消息如需
                        # 监控请使用独立指标。
                        _record_stage("remote_send_started", _send_t0 - _enqueued_at)
                    send_remote_frame(self._sock, header, payload)
                    if is_status:
                        # 状态消息不需要 ack_trace（旧/新地面端都只忽略或记录）。
                        # send_remote_frame 无异常返回即视为成功发送。
                        self._note_completed()
                        self.status_callback(f"Remote sent status: {header.get('state')}")
                        continue
                    self._wait_for_trace_ack(self._sock, header.get("trace_index"))
                    _record_stage("remote_ack_received", time.perf_counter() - _send_t0)
                    self.status_callback(f"Remote sent trace {header['trace_index']}")
                finally:
                    self._busy.clear()
                    self._pending_item = pending
            except queue.Empty:
                continue
            except Exception as exc:
                if self._stop.is_set():
                    # 停止中：直接退出，不再发送 "retrying" 状态，也不阻塞等待。
                    # 强制停止路径（drain=False）：放弃 pending（调用方明确要求）。
                    return
                if pending is None and "item" in locals() and item is not None:
                    pending = item
                self._pending_item = pending
                self._close()
                self.status_callback(f"Remote send retrying: {exc}")
                # 用 Event.wait 限速，stop 可立即唤醒（不等完整 2 秒）。
                self._stop.wait(2.0)


class AcquisitionWorker(QtCore.QThread):
    trace_ready = QtCore.Signal(int, object, object, object, object, object, object, object, str)
    status = QtCore.Signal(str)
    failed = QtCore.Signal(str)
    finished_run = QtCore.Signal()

    def __init__(
        self,
        cfg: SweepConfig,
        gnss_port: str,
        baud: int,
        out_dir: Path,
        count: int,
        interval_s: float,
        calibration_path: str,
        session_calibration: OSLCalibration | None,
        session_calibration_p2: OSLCalibration | None,
        through_measurement: CalibrationMeasurement | None,
        background_frequency_hz: np.ndarray | None,
        background: np.ndarray | None,
        autosave: bool,
    ) -> None:
        super().__init__()
        self.cfg = cfg
        self.gnss_port = gnss_port
        self.baud = baud
        self.out_dir = out_dir
        self.count = count
        self.interval_s = interval_s
        self.calibration_path = calibration_path
        self.session_calibration = session_calibration
        self.session_calibration_p2 = session_calibration_p2
        self.through_measurement = through_measurement
        self.background_frequency_hz = background_frequency_hz
        self.background = background
        self.autosave = autosave
        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def run(self) -> None:
        gnss = GnssReader(self.gnss_port, self.baud) if self.gnss_port else None
        calibration_p1 = self.session_calibration or (OSLCalibration.from_file(self.calibration_path) if self.calibration_path else None)
        calibration_p2 = self.session_calibration_p2
        index = 0
        try:
            while not self._stop and (self.count <= 0 or index < self.count):
                index += 1
                self.status.emit(f"Acquiring trace {index}...")
                points = acquire_sparameters(
                    int(self.cfg.start_mhz * 1e6),
                    int(self.cfg.stop_mhz * 1e6),
                    self.cfg.points,
                    self.cfg.ifbw_hz,
                    self.cfg.power_dbm,
                    self.cfg.dwell_us,
                )
                freq, raw = sparameter_arrays(points)
                display, time_ns, trace_by_channel = process_trace_pipeline(
                    freq,
                    raw,
                    self.cfg,
                    calibration_p1=calibration_p1,
                    calibration_p2=calibration_p2,
                    through_measurement=self.through_measurement,
                    background_frequency_hz=self.background_frequency_hz,
                    background=self.background,
                )
                fix = gnss.read_gga() if gnss else GgaFix()
                path_text = ""
                if self.autosave:
                    path_text = str(save_trace(self.out_dir, index, self.cfg, freq, raw, display, time_ns, trace_by_channel, fix))
                self.trace_ready.emit(index, freq, raw, display, time_ns, trace_by_channel, fix, self.cfg, path_text)
                if self.count > 0 and index >= self.count:
                    break
                slept = 0.0
                while not self._stop and slept < self.interval_s:
                    time.sleep(min(0.1, self.interval_s - slept))
                    slept += 0.1
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            if gnss:
                gnss.close()
            self.finished_run.emit()


class GnssReaderThread(QtCore.QThread):
    """独立 GNSS 连续读取线程（roadmap §10、区域二）。

    持续读取 NMEA，把有效 Fix 写入 :class:`GnssFixCache`（有效性由缓存统一
    判定）。初次 open 失败不退出，进入可停止的重试循环；读取失败后 close、
    退避等待、重新 open。所有等待使用 ``threading.Event.wait()``，Stop 能及时
    唤醒，不允许忙循环；GNSS 失败绝不影响 USB 数据流。
    """

    status = QtCore.Signal(str)

    #: open 失败退避下限 / 上限。
    INITIAL_BACKOFF_S = 0.5
    MAX_BACKOFF_S = 5.0
    #: 读取失败后重新 open 前的退避。
    READ_FAILURE_BACKOFF_S = 1.0

    def __init__(self, port: str, baud: int, cache: GnssFixCache, *, poll_interval_s: float = 0.1) -> None:
        super().__init__()
        self.port = port
        self.baud = baud
        self.cache = cache
        self._stop = threading.Event()
        self._poll_interval_s = float(poll_interval_s)

    def request_stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        reader: GnssReader | None = None
        backoff_s = self.INITIAL_BACKOFF_S
        while not self._stop.is_set():
            # 初次打开 / 读取失败后的重连：可停止的退避重试。
            if reader is None or reader.ser is None or not reader.ser.is_open:
                try:
                    reader = GnssReader(self.port, self.baud)
                    reader.open()
                except Exception as exc:
                    self.status.emit(f"GNSS 打开失败（重试中，不影响 USB）: {exc}")
                    try:
                        if reader is not None:
                            reader.close()
                    except Exception:
                        pass
                    if self._stop.wait(backoff_s):
                        break
                    backoff_s = min(backoff_s * 2.0, self.MAX_BACKOFF_S)
                    continue
                backoff_s = self.INITIAL_BACKOFF_S
                self.status.emit(f"GNSS 重新连接成功: {self.port}")
            assert reader is not None
            # 打开成功：连续读取。
            try:
                fix = reader.read_gga(timeout_s=0.5)
            except Exception as exc:
                self.status.emit(f"GNSS 读取错误（将重连，不影响 USB）: {exc}")
                try:
                    reader.close()
                except Exception:
                    pass
                reader = None
                if self._stop.wait(self.READ_FAILURE_BACKOFF_S):
                    break
                continue
            if fix.latitude is not None and fix.longitude is not None:
                self.cache.update(fix)
            elif not self._stop.wait(self._poll_interval_s):
                continue
        # 收尾：确保串口关闭。
        if reader is not None:
            try:
                reader.close()
            except Exception:
                pass


class ProcessingConsumer(QtCore.QThread):
    """处理/保存/发送消费者线程（roadmap §5 PROC、§3 线程解耦）。

    从有界队列取选中的 sweep，执行校准/BPF/IFFT/Dewow 处理、CSV/NPZ 保存与
    远程发送；不接触 USB / 串口。本地处理/保存失败按 roadmap §13 默认终止
    任务（发出 ``fatal_error``，USB 线程收到后停止）。
    """

    trace_ready = QtCore.Signal(int, object, object, object, object, object, object, object, str)
    status = QtCore.Signal(str)
    failed = QtCore.Signal(str)
    fatal_error = QtCore.Signal(str)

    def __init__(
        self,
        *,
        queue: BoundedQueue,
        gnss_cache: GnssFixCache,
        cfg: SweepConfig,
        out_dir: Path,
        autosave: bool,
        calibration_p1: OSLCalibration | None,
        calibration_p2: OSLCalibration | None,
        through_measurement: CalibrationMeasurement | None,
        background_frequency_hz: np.ndarray | None,
        background: np.ndarray | None,
        remote_sender: RemoteSender | None = None,
        drain_after_stop_s: float = 2.0,
    ) -> None:
        super().__init__()
        self._queue = queue
        self._gnss_cache = gnss_cache
        self.cfg = cfg
        self.out_dir = out_dir
        self.autosave = autosave
        self.calibration_p1 = calibration_p1
        self.calibration_p2 = calibration_p2
        self.through_measurement = through_measurement
        self.background_frequency_hz = background_frequency_hz
        self.background = background
        self.remote_sender = remote_sender
        self._drain_after_stop_s = float(drain_after_stop_s)  # 兼容保留（不再使用固定时限）
        self._stop = threading.Event()
        self.processed_count = 0
        #: 第1轮检查点D：停止排空状态机 running → draining → drained / drain_failed。
        self.drain_state: str = "running"
        self.drain_failures: list[str] = []
        #: 最近一条已处理 sweep 的 GNSS Fix 年龄（供统计；不伪造为 0）。
        self.last_gnss_age_ms: float | None = None

    def request_stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        try:
            while not self._stop.is_set():
                item = self._queue.get(timeout=0.05)
                if item is None:
                    continue
                self._process_one(item)
            # 第1轮检查点D：停止标志表示"不再等待未来数据"；队列中已被选中的
            # sweep 必须全部处理完（删除固定 2 秒放弃语义）。生产者先停，
            # 队列不再有新入队，排空到队列为空即完成。
            self.drain_state = "draining"
            self._drain_remaining()
            self.drain_state = (
                "drained" if not self.drain_failures else "drain_failed"
            )
        except Exception as exc:
            self.drain_state = "drain_failed"
            self.drain_failures.append(f"处理线程异常: {exc}")
            self.fatal_error.emit(f"处理线程异常: {exc}")

    def _process_one(self, item: SelectedSweep) -> None:
        # processing_started：SelectedSweep 入处理队列到 Consumer 开始处理的
        # 真实等待时间（第二次定向修复：不再用 0.0 冒充；enqueued_at_perf==0
        # 表示测试直接构造、无法测量——跳过记录，绝不伪造）。
        enqueued_at = getattr(item, "enqueued_at_perf", 0.0)
        if enqueued_at > 0.0:
            _record_stage("processing_started", time.perf_counter() - enqueued_at)
        _processing_t0 = time.perf_counter()
        # 连续采集为 S11/S22 双通道 schema（决策门 B）：只处理真实存在的通道。
        channels = ("S11", "S22")
        lookup = self._gnss_cache.latest(item.midpoint_monotonic)
        self.last_gnss_age_ms = lookup.age_ms
        # 只有 lookup.valid 时才把缓存 Fix 当作当前有效定位传给保存/UI/遥测；
        # 否则传递空 GgaFix，绝不把陈旧/无质量坐标伪装成有效数据（roadmap §10）。
        fix = lookup.fix if (lookup.valid and lookup.fix is not None) else GgaFix()
        acquisition = self._acquisition_metadata(item, lookup)
        flight = self._flight_metadata(lookup, fix)
        raw_sparams = dual_channel_raw(item.raw)
        display, time_ns, trace_by_channel = process_trace_pipeline(
            item.frequency_hz,
            raw_sparams,
            self.cfg,
            calibration_p1=self.calibration_p1,
            calibration_p2=self.calibration_p2,
            through_measurement=self.through_measurement,
            background_frequency_hz=self.background_frequency_hz,
            background=self.background,
        )
        path_text = ""
        if self.autosave:
            _save_t0 = time.perf_counter()
            path_text = str(
                save_trace(
                    self.out_dir,
                    item.trace_index,
                    self.cfg,
                    item.frequency_hz,
                    raw_sparams,
                    display,
                    time_ns,
                    trace_by_channel,
                    fix,
                    channels=channels,
                    acquisition=acquisition,
                    flight=flight,
                )
            )
            _record_stage("local_save_completed", time.perf_counter() - _save_t0)
        self.trace_ready.emit(
            item.trace_index,
            item.frequency_hz,
            raw_sparams,
            display,
            time_ns,
            trace_by_channel,
            fix,
            self.cfg,
            path_text,
        )
        if self.remote_sender is not None:
            self.remote_sender.enqueue_trace(
                item.trace_index,
                self.cfg,
                item.frequency_hz,
                raw_sparams,
                display,
                time_ns,
                trace_by_channel,
                fix,
                channels=channels,
                acquisition=acquisition,
                flight=flight,
            )
        self.processed_count += 1
        _record_stage("processing_completed", time.perf_counter() - _processing_t0)

    @staticmethod
    def _acquisition_metadata(item: SelectedSweep, lookup: GnssFixLookup) -> dict[str, object]:
        """组装 roadmap §9.3 的 acquisition metadata（保存时刻由落盘/发送边界补记）。"""
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
            "gnss_age_ms": lookup.age_ms,
            "session_statistics": item.session_statistics,
        }

    @staticmethod
    def _flight_metadata(lookup: GnssFixLookup, fix: GgaFix) -> dict[str, object]:
        """高度语义：GNSS MSL 海拔 vs 对地 AGL。无真实 AGL 源时保持 None。"""
        return {
            "gnss_valid": bool(lookup.valid),
            "gnss_altitude_msl_m": fix.altitude_m if lookup.valid else None,
            "flight_height_agl_m": None,
        }

    def _drain_remaining(self) -> None:
        """停止后处理已被选中的全部残留项（无固定时限，排空到队列空为止）。

        排空期间持续更新 remaining 数量；任一处理/保存失败明确记录
        （``drain_failures``），不假装排空完成（``drain_state=drain_failed``）。
        """
        while True:
            item = self._queue.get(timeout=0.1)
            if item is None:
                # 生产者已停止且队列空（get 超时）：排空完成。
                break
            try:
                self._process_one(item)
            except Exception as exc:
                self.drain_failures.append(f"trace #{getattr(item, 'trace_index', '?')}: {exc}")
                self.failed.emit(f"停止后处理残留 trace 失败: {exc}")


class ContinuousAcquisitionWorker(QtCore.QThread):
    """连续采集 USB 生产者线程（roadmap §5 USB、§3 线程解耦）。

    只负责：open/configure 一次 :class:`PersistentLibreVnaBackend`、持续
    ``acquire_one()`` 组装完整 sweep、用 :class:`FixedIntervalSweepSelector`
    按固定 monotonic deadline 抽样、把选中的 sweep 非阻塞放入有界处理队列。
    **不**在此线程执行保存间隔 sleep、GNSS 读取、NPZ 压缩、文件保存或网络
    等待。

    GNSS 连续读取（:class:`GnssReaderThread`）与处理/保存/发送
    （:class:`ProcessingConsumer`）由本线程启动、协同，并在 finally 中按有界
    时间收尾。stop/pause/resume 全部使用 :class:`threading.Event`，不依赖
    queued slot，绝不 terminate 活线程。
    """

    trace_ready = QtCore.Signal(int, object, object, object, object, object, object, object, str)
    status = QtCore.Signal(str)
    failed = QtCore.Signal(str)
    finished_run = QtCore.Signal()
    paused = QtCore.Signal()
    resumed = QtCore.Signal()
    statistics_updated = QtCore.Signal(dict)

    def __init__(
        self,
        cfg: SweepConfig,
        interval_s: float,
        out_dir: Path,
        count: int,
        gnss_port: str,
        baud: int,
        calibration_path: str,
        session_calibration: OSLCalibration | None,
        session_calibration_p2: OSLCalibration | None,
        through_measurement: CalibrationMeasurement | None,
        background_frequency_hz: np.ndarray | None,
        background: np.ndarray | None,
        autosave: bool,
        remote_sender: RemoteSender | None = None,
        *,
        backend: PersistentLibreVnaBackend | None = None,
        scheduler: FixedIntervalSweepSelector | None = None,
        gnss_cache: GnssFixCache | None = None,
        consumer_factory: Callable[[BoundedQueue, GnssFixCache], ProcessingConsumer] | None = None,
        processing_queue_maxsize: int = 512,
        queue_drop_policy: str | None = None,
        backend_settings: PersistentBackendSettings | None = None,
        statistics_interval_s: float = 0.5,
    ) -> None:
        super().__init__()
        self.cfg = cfg
        self.interval_s = float(interval_s)
        self.out_dir = out_dir
        self.count = count
        self.gnss_port = gnss_port
        self.baud = baud
        self.calibration_path = calibration_path
        self.session_calibration = session_calibration
        self.session_calibration_p2 = session_calibration_p2
        self.through_measurement = through_measurement
        self.background_frequency_hz = background_frequency_hz
        self.background = background
        self.autosave = autosave
        self.remote_sender = remote_sender
        self._backend = backend
        self._scheduler = scheduler if scheduler is not None else FixedIntervalSweepSelector(interval_s)
        self._gnss_cache = gnss_cache if gnss_cache is not None else GnssFixCache()
        self._consumer_factory = consumer_factory
        # 第1轮检查点C：生产处理队列默认无损（LosslessProcessingQueue）。
        # 显式传入 queue_drop_policy 的旧测试仍获得 BoundedQueue 丢弃策略。
        self._processing_queue: LosslessProcessingQueue | BoundedQueue
        if queue_drop_policy is None:
            self._processing_queue = LosslessProcessingQueue(
                processing_queue_maxsize,
                on_high_water=self._on_queue_high_water,
                on_hard_limit=self._on_queue_hard_limit,
            )
        else:
            self._processing_queue = BoundedQueue(
                processing_queue_maxsize, drop_policy=queue_drop_policy
            )
        self._controlled_stop_reason: str | None = None
        self._backend_settings = backend_settings
        self._statistics_interval_s = float(statistics_interval_s)
        self._stop = threading.Event()
        self._pause_requested = threading.Event()
        self._resume_requested = threading.Event()
        self._paused_at_boundary = threading.Event()
        self._last_statistics_emit = 0.0
        self._run_started_monotonic: float | None = None
        self._consumer: ProcessingConsumer | None = None
        self._gnss_thread: GnssReaderThread | None = None
        #: 会话级已选 trace 计数（独立于 scheduler；pause/reset 不清零，供
        #: trace_index 与有限 count 判断使用）。
        self._selected_count = 0
        #: 综合终态（第二次定向修复）：worker 运行期错误（含 failed_item 路径）。
        self._terminal_error: str | None = None
        #: 最终终态：drained / failed / unknown（停止后由 finally 综合判定）。
        self.terminal_state: str = "unknown"

    # ---- 外部控制（线程安全，可从任意线程调用）----

    def stop(self) -> None:
        self.request_stop()

    def request_stop(self) -> None:
        self._stop.set()
        self._resume_requested.set()  # 唤醒暂停等待

    def request_pause(self) -> None:
        """请求暂停（下一完整 acquire_one 边界生效）。"""
        self._pause_requested.set()

    def request_resume(self) -> None:
        """请求继续（唤醒暂停等待，重建 schedule 后继续）。"""
        self._resume_requested.set()

    def is_paused_at_boundary(self) -> bool:
        """工作线程是否已停在暂停边界（等待 resume/stop 唤醒）。"""
        return self._paused_at_boundary.is_set()

    def _on_queue_high_water(self, depth: int) -> None:
        """处理队列高水位：立即告警（不停止采集）。"""
        self.status.emit(f"处理队列高水位告警: depth={depth}（消费者过慢，继续无损积压）")

    def _on_queue_hard_limit(self, depth: int) -> None:
        """处理队列硬上限：先接收当前 Trace，然后请求受控停止新采集。

        注意：生产速度长期高于消费速度时，有限内存无法无限保证持续采集；
        这里选择"告警 + 受控停止新采集 + 完整排空"，绝不静默丢失。
        """
        self._controlled_stop_reason = f"处理队列达到硬上限 depth={depth}（受控停止新采集，已有数据将全部排空）"
        self.status.emit(self._controlled_stop_reason)
        self.request_stop()

    # ---- 只读访问（测试 / 诊断 / UI）----

    @property
    def scheduler(self) -> FixedIntervalSweepSelector:
        return self._scheduler

    @property
    def gnss_cache(self) -> GnssFixCache:
        return self._gnss_cache

    @property
    def processing_queue(self) -> BoundedQueue:
        return self._processing_queue

    @property
    def backend(self) -> PersistentLibreVnaBackend | None:
        return self._backend

    @property
    def consumer(self) -> ProcessingConsumer | None:
        """当前消费者线程（run() 启动后有效；测试/诊断用）。"""
        return self._consumer

    @property
    def gnss_thread(self) -> GnssReaderThread | None:
        """当前 GNSS 线程（配置了 gnss_port 且启动后有效；测试/诊断用）。"""
        return self._gnss_thread

    def statistics(self) -> dict[str, object]:
        """会话统计快照；返回值为纯 JSON 可序列化 dict（无 callable）。

        ``processing_queue`` 必须调用 ``statistics()`` 取得快照，否则 bound method
        进入 ``session_statistics`` 后会在 autosave/遥测 JSON 序列化时抛
        "Object of type method is not JSON serializable"（阶段 5 发现的阻塞缺陷）。
        """
        backend_stats: dict[str, int] = {}
        backend = self._backend
        if backend is not None:
            try:
                backend_stats = backend.statistics.to_dict()
            except Exception:  # noqa: BLE001 - 只读诊断不得抛出
                backend_stats = {}
        try:
            queue_stats = self._processing_queue.statistics()
        except Exception:  # noqa: BLE001 - 只读诊断不得抛出
            queue_stats = {}
        return {
            "mode": "continuous",
            "scheduler": self._scheduler.statistics,
            "processing_queue": queue_stats,
            "gnss_age_ms": self._gnss_cache.latest().age_ms,
            "backend": backend_stats,
        }

    # ---- QThread.run ----

    def run(self) -> None:
        phase = "启动连续采集"
        backend = self._backend
        gnss_thread: GnssReaderThread | None = None
        consumer: ProcessingConsumer | None = None
        try:
            if backend is None:
                backend = PersistentLibreVnaBackend(settings=self._backend_settings)
            self._backend = backend
            if self.gnss_port:
                gnss_thread = GnssReaderThread(self.gnss_port, self.baud, self._gnss_cache)
                self._gnss_thread = gnss_thread
                gnss_thread.status.connect(self.status)
                gnss_thread.start()
            consumer = self._create_consumer()
            self._consumer = consumer
            consumer.trace_ready.connect(self.trace_ready)
            consumer.status.connect(self.status)
            consumer.failed.connect(self.failed)
            # fatal 用 DirectConnection：在处理线程内立即 request_stop，避免
            # 依赖主线程事件循环投递 queued 信号而延误停机。
            consumer.fatal_error.connect(
                self._on_consumer_fatal,
                QtCore.Qt.ConnectionType.DirectConnection,
            )
            consumer.start()

            # start 后立即 stop：open/configure 前即可观察，不覆盖已设请求。
            if self._stop.is_set():
                return
            phase = "打开采集后端"
            backend.open()
            if self._stop.is_set():
                return
            phase = "配置扫频参数"
            backend.configure(self._sweep_settings())
            self._scheduler.reset()
            self._run_started_monotonic = time.monotonic()
            self.status.emit("连续采集已启动 (continuous)")
            while not self._stop.is_set():
                # 暂停检查放在每次 acquire_one 前：pause 只在完整 sweep 边界生效。
                if self._pause_requested.is_set():
                    if not self._pause_wait_for_wakeup(backend):
                        return
                if self._stop.is_set():
                    break
                if self.count > 0 and self._selected_count >= self.count:
                    break
                started_monotonic = time.monotonic()
                started_wall = time.time()
                try:
                    sweep = backend.acquire_one()
                except LibreVnaSweepError as exc:
                    # 单次 sweep 超时：丢 partial、计数、继续重同步（roadmap §13）。
                    self.status.emit(f"sweep 超时（继续）: {exc}")
                    self._emit_statistics(force=True)
                    continue
                completed_monotonic = time.monotonic()
                completed_wall = time.time()
                _record_stage("sweep_completed", completed_monotonic - started_monotonic)
                midpoint = (started_monotonic + completed_monotonic) / 2.0
                # 第1轮复审修复：受控停止已请求时，在 should_select **之前**
                # 阻止后续 sweep 被选择（scheduler 不计数 → selected 口径
                # 一致；不计 selected、不入队、不再 acquire → 不卡 USB 线程）。
                if (
                    isinstance(self._processing_queue, LosslessProcessingQueue)
                    and self._processing_queue.stop_requested.is_set()
                ):
                    continue
                result = self._scheduler.should_select(midpoint)
                self._emit_statistics()
                if not result.select:
                    continue
                next_index = self._selected_count + 1
                # 阶段4：CompleteSweep 不携带逐点主机接收时间，用 acquire_one
                # 调用起止的墙钟近似首末点时间；规范采集时间取中点（roadmap §9.3）。
                offset = (
                    (started_wall - started_monotonic) + (completed_wall - completed_monotonic)
                ) / 2.0
                scheduled_wall = None
                if result.next_due_s is not None and self.interval_s > 0:
                    scheduled_wall = (result.next_due_s - self.interval_s) + offset
                item = SelectedSweep(
                    trace_index=next_index,
                    device_sweep_index=sweep.trace_index,
                    frequency_hz=sweep.frequency_hz,
                    raw=sweep.raw,
                    cfg=self.cfg,
                    midpoint_monotonic=midpoint,
                    selected_at_utc=utc_iso_now(),
                    missed_slots=result.missed_slots,
                    actual_interval_s=result.actual_interval_s,
                    schedule_error_ms=result.schedule_error_ms,
                    acquired_at_utc=utc_iso_from_epoch((started_wall + completed_wall) / 2.0),
                    sweep_started_utc=utc_iso_from_epoch(started_wall),
                    sweep_completed_utc=utc_iso_from_epoch(completed_wall),
                    scheduled_at_utc=(
                        utc_iso_from_epoch(scheduled_wall) if scheduled_wall is not None else None
                    ),
                    connection_generation=backend.statistics.connection_generation,
                    target_interval_s=self.interval_s,
                    session_statistics=self.statistics(),
                )
                # 第1轮复审修复：put 失败只有两种语义——(a) 受控停止已请求且
                # 队列满（sweep 未入队：不计 selected，受控停止）；(b) put 超时
                # （消费者卡死：fail-closed fatal，绝不伪装 drained，绝不静默
                # 丢 Trace——item 保存在队列 failed_item 中可追踪）。
                if not self._processing_queue.put(item):
                    if isinstance(self._processing_queue, LosslessProcessingQueue) and self._processing_queue.failed_item is item:
                        self._terminal_error = (
                            f"处理队列入队失败（消费者长时间未消费，fail-closed）: "
                            f"trace #{item.trace_index}（已保存待诊断，绝不静默丢弃）"
                        )
                        self.failed.emit(self._terminal_error)
                        self.request_stop()
                        break
                    # (a) 受控停止路径：sweep 未入队也不计 selected。
                    continue
                # put 成功才计入 selected（保证 selected == 入队 == 可处理）。
                item.enqueued_at_perf = time.perf_counter()  # 真实入队时刻。
                self._selected_count += 1
        except Exception as exc:
            self._terminal_error = f"{phase}失败：{exc}"
            self.failed.emit(self._terminal_error)
        finally:
            # 第1轮检查点D：先向地面端报告"正在停止（排空中）"；最终 stopped /
            # drain_failed 只有在队列归零/排空完成后才报告。
            if self.remote_sender is not None:
                try:
                    queue_stats = self._processing_queue.statistics()
                except Exception:  # noqa: BLE001
                    queue_stats = {}
                self.remote_sender.enqueue_status(
                    "stopping",
                    processing_queue_depth=queue_stats.get("depth", 0),
                    selected_count=self._selected_count,
                    reason=self._controlled_stop_reason,
                )
            self._shutdown(consumer, gnss_thread, backend)
            # ---- 综合终态判定（第二次定向修复）----
            # 不得只依据 consumer.drain_state；至少综合：
            # processing_queue.failed_item / worker terminal error /
            # consumer.drain_state / consumer.drain_failures / selected-processed 对账。
            queue_failed_item = getattr(self._processing_queue, "failed_item", None)
            drain_state = getattr(consumer, "drain_state", None) if consumer is not None else None
            drain_failures = list(getattr(consumer, "drain_failures", [])) if consumer is not None else []
            processed_count = int(getattr(consumer, "processed_count", 0) or 0) if consumer is not None else 0
            misaccounted = processed_count != self._selected_count
            failure_details: list[str] = []
            if queue_failed_item is not None:
                failure_details.append(
                    f"处理队列failed_item: trace #{getattr(queue_failed_item, 'trace_index', '?')}"
                )
            if self._terminal_error:
                failure_details.append(f"worker错误: {self._terminal_error}")
            if drain_failures:
                failure_details.append(f"排空失败 {len(drain_failures)} 条: {drain_failures[0]}")
            if misaccounted:
                failure_details.append(
                    f"对账失败: selected={self._selected_count} processed={processed_count}"
                )
            terminal_failed = bool(
                queue_failed_item is not None
                or self._terminal_error
                or drain_state == "drain_failed"
                or bool(drain_failures)
                or misaccounted
            )
            self.terminal_state = "failed" if terminal_failed else (
                "drained" if drain_state == "drained" else "unknown"
            )
            # 失败必须明确上报（不得伪装正常完成）。
            if terminal_failed:
                self.failed.emit(
                    f"任务最终状态: drain_failed/failed——{'；'.join(failure_details) or '未知原因'}"
                )
            # ---- 最终状态消息：失败任务绝不发 stopped+drained ----
            if self.remote_sender is not None:
                if terminal_failed:
                    self.remote_sender.enqueue_status(
                        "drain_failed",
                        drain_state="failed" if queue_failed_item is not None or self._terminal_error else "drain_failed",
                        processing_queue_depth=len(drain_failures),
                        selected_count=self._selected_count,
                        processed_count=processed_count,
                        reason=self._controlled_stop_reason or ("；".join(failure_details) or None),
                        failure_details=failure_details or None,
                    )
                else:
                    self.remote_sender.enqueue_status(
                        "stopped",
                        drain_state=drain_state,
                        processing_queue_depth=0,
                        selected_count=self._selected_count,
                        processed_count=processed_count,
                        reason=self._controlled_stop_reason,
                    )
            self.finished_run.emit()

    def _create_consumer(self) -> ProcessingConsumer:
        if self._consumer_factory is not None:
            return self._consumer_factory(self._processing_queue, self._gnss_cache)
        calibration_p1 = self.session_calibration or (
            OSLCalibration.from_file(self.calibration_path) if self.calibration_path else None
        )
        return ProcessingConsumer(
            queue=self._processing_queue,
            gnss_cache=self._gnss_cache,
            cfg=self.cfg,
            out_dir=self.out_dir,
            autosave=self.autosave,
            calibration_p1=calibration_p1,
            calibration_p2=self.session_calibration_p2,
            through_measurement=self.through_measurement,
            background_frequency_hz=self.background_frequency_hz,
            background=self.background,
            remote_sender=self.remote_sender,
        )

    def _sweep_settings(self) -> SweepSettings:
        cfg = self.cfg
        return SweepSettings(
            start_hz=int(cfg.start_mhz * 1e6),
            stop_hz=int(cfg.stop_mhz * 1e6),
            points=cfg.points,
            ifbw_hz=cfg.ifbw_hz,
            power_dbm=cfg.power_dbm,
            dwell_us=cfg.dwell_us,
            stages_bitmap=S11_S22_STAGES_BITMAP,
        )

    def _pause_wait_for_wakeup(self, backend: PersistentLibreVnaBackend) -> bool:
        """在完整 acquire_one 边界暂停：backend.pause() + 等待 resume/stop。

        返回 True 表示收到 resume 且后端已恢复（调用方继续采集）；False 表示
        等待期间收到 stop（调用方退出，**不**调用 backend.resume）。
        """
        if self._stop.is_set():
            return False
        backend.pause()
        self._paused_at_boundary.set()
        # 暂停是无意的时钟中断：恢复后重新建立固定周期（下一道立即选择），但
        # **保留**会话累计统计（selected/skipped/missed 与统计样本不清零）。
        self._scheduler.restart_timeline()
        self.paused.emit()
        try:
            while not self._resume_requested.is_set():
                if self._stop.wait(timeout=0.02):
                    return False
        finally:
            self._clear_pause_events()
        if self._stop.is_set():
            return False
        backend.resume()
        self.resumed.emit()
        return True

    def _clear_pause_events(self) -> None:
        self._pause_requested.clear()
        self._resume_requested.clear()
        self._paused_at_boundary.clear()

    def _on_consumer_fatal(self, message: str) -> None:
        self._terminal_error = message
        self.failed.emit(message)
        self.request_stop()

    def _emit_statistics(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._last_statistics_emit < self._statistics_interval_s:
            return
        self._last_statistics_emit = now
        backend = self._backend
        backend_stats: dict[str, int] = {}
        if backend is not None:
            try:
                backend_stats = backend.statistics.to_dict()
            except Exception:  # noqa: BLE001 - 只读诊断不得抛出
                backend_stats = {}
        elapsed = (now - self._run_started_monotonic) if self._run_started_monotonic else 0.0
        completed = int(backend_stats.get("completed_sweeps", 0))
        sweep_rate_hz = (completed / elapsed) if elapsed > 0.0 else 0.0
        try:
            queue_stats = self._processing_queue.statistics()
        except Exception:  # noqa: BLE001 - 只读诊断不得抛出
            queue_stats = {}
        self.statistics_updated.emit(
            {
                "mode": "continuous",
                "sweep_rate_hz": round(sweep_rate_hz, 3),
                "backend": backend_stats,
                "scheduler": self._scheduler.statistics,
                "processing_queue": queue_stats,
                "gnss_age_ms": self._gnss_cache.latest().age_ms,
                "count_target": self.count,
            }
        )

    def _release_backend(self, backend: PersistentLibreVnaBackend) -> None:
        """fail-closed 释放 USB：重试 backend.close() 直到 backend.is_open=False。

        - close() 抛错但 is_open=False：资源已实际释放，保留错误上报后返回；
        - close() 抛错且 is_open=True：上报（首次 + 每 5s 周期状态），按 1s
          有限频率继续重试，**绝不**把 FAULTED 但仍打开的 backend 当作关闭成功；
        - close() 正常返回但仍报告 is_open=True：同样继续重试。

        重试等待用 ``time.sleep(1.0)`` 固定间隔（非忙循环）；stop 请求**不会**
        提前退出本循环——资源未释放前不得结束 worker / 发 finished_run。
        """
        first_report = True
        last_status = 0.0
        while True:
            try:
                backend.close()
            except Exception as exc:  # noqa: BLE001 - 释放失败需上报并重试
                if first_report:
                    self.failed.emit(f"释放 LibreVNA 失败（将自动重试）: {exc}")
                    first_report = False
                if not backend.is_open:
                    # close 抛错但资源已实际释放：保留上述错误上报后完成退出。
                    return
            else:
                if not backend.is_open:
                    return
                if first_report:
                    self.failed.emit("释放 LibreVNA 失败: close() 已返回但仍报告打开")
                    first_report = False
            now = time.monotonic()
            if now - last_status >= 5.0:
                self.status.emit("仍在等待释放 LibreVNA（资源仍被持有，将自动重试）...")
                last_status = now
            time.sleep(1.0)

    def _shutdown(
        self,
        consumer: ProcessingConsumer | None,
        gnss_thread: GnssReaderThread | None,
        backend: PersistentLibreVnaBackend | None,
    ) -> None:
        """fail-closed 收尾：先通知子线程停止，再重试释放 USB，最后 join 子线程。

        顺序（本轮修正）：

        1. 先对 consumer / GNSS 发出非阻塞 ``request_stop()``——只是极短的内存
           操作，不等、不阻塞，也不延迟 USB 释放；
        2. 随即 :meth:`_release_backend` 重试 ``backend.close()`` 直到
           ``backend.is_open=False``（fail-closed：资源仍被持有绝不结束）；
        3. USB 已释放后再完成子线程 join（有界，超时明确报告哪个线程尚未退出
           并继续等待其退出）。

        任何情况下都不发 finished_run，除非 backend 已释放（不存在或 is_open
        明确为 False）**且**全部子线程已退出；绝不 terminate 活线程；MainWindow
        在 worker 未安全释放前不会把 worker 清为 None、也不会解锁手动 USB 控件。
        """
        if consumer is not None:
            consumer.request_stop()
        if gnss_thread is not None:
            gnss_thread.request_stop()
        if backend is not None:
            self._release_backend(backend)
        if consumer is not None:
            if not consumer.wait(3000):
                self.status.emit("处理线程未在 3s 内退出，正在等待其结束...")
                consumer.wait()
        if gnss_thread is not None:
            if not gnss_thread.wait(3000):
                self.status.emit("GNSS 线程未在 3s 内退出，正在等待其结束...")
                gnss_thread.wait()


def _apply_trace_to_ui(window, index, freq, display, time_ns, trace_by_channel, fix, cfg) -> str:
    """共享的 UI 更新逻辑（S-param / A-scan / B-scan / GNSS map）。返回 fix_text。

    接收 window-like 对象（MainWindow 或测试中的 SimpleNamespace），使 legacy
    与 continuous 两条 trace 通道复用同一 UI 更新。
    """
    window.last_display_sparams = display
    window.last_time_ns = time_ns
    for name in UI_CHANNELS:
        window.sparam_curves[name].setData(freq, db20(display[name]))
        window.ascan_curves[name].setData(time_ns, normalize_trace_for_display(trace_by_channel[name]))
    processing_signature = bscan_processing_signature(
        cfg.dewow_enabled,
        cfg.dewow_window_ns,
    )
    window._append_bscan_trace(time_ns, trace_by_channel, processing_signature)
    window.update_sparam_ranges(freq, display)
    window.update_ascan_range(time_ns)
    window.refresh_bscan_views()
    fix_text = f"fix={fix.fix_quality}, sats={fix.satellites}, lat={fix.latitude}, lon={fix.longitude}"
    window.last_info.setText(f"#{index} {fix_text}")
    window.gnss_map.update_fix(
        latitude=fix.latitude,
        longitude=fix.longitude,
        fix_quality=fix.fix_quality,
        satellites=fix.satellites,
        altitude_m=fix.altitude_m,
        utc=fix.utc,
    )
    return fix_text


class MainWindow(QtWidgets.QMainWindow):
    remote_status = QtCore.Signal(str)
    remote_command = QtCore.Signal(str, object)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("UAV-GPR Control System")
        self.resize(1380, 860)
        # 第1轮复审：空中端性能统计接入生产路径（默认注入；诊断日志可查看）。
        if AIR_METRICS is None:
            set_air_metrics(PipelineMetrics())
        self.worker: AcquisitionWorker | ContinuousAcquisitionWorker | None = None
        self.remote_sender: RemoteSender | None = None
        self.remote_run_id = ""
        self.background: np.ndarray | None = None
        self.background_frequency_hz: np.ndarray | None = None
        self.cal_measurements: dict[str, CalibrationMeasurement] = {}
        self.session_calibration: OSLCalibration | None = None
        self.session_calibration_p2: OSLCalibration | None = None
        self.through_measurement: CalibrationMeasurement | None = None
        self.bscan: dict[str, list[np.ndarray]] = {name: [] for name in UI_CHANNELS}
        self.bscan_time_ns: np.ndarray | None = None
        self.bscan_processing_signature: tuple[bool, float | None] | None = None
        self.bscan_filter_status_override: str | None = None
        self.bscan_window_start = 1
        self.bscan_follow_latest = True
        self.last_display_sparams: dict[str, np.ndarray] | None = None
        self.last_time_ns: np.ndarray | None = None
        self.remote_status.connect(self.log)
        self.remote_command.connect(self.handle_remote_command)
        self._closing = False
        # 延迟自动连接 RemoteSender：由 MainWindow 管理的单次 QTimer（parented），
        # closeEvent 中先停止，窗口关闭后不得再创建 RemoteSender。
        self._remote_connect_timer = QtCore.QTimer(self)
        self._remote_connect_timer.setSingleShot(True)
        self._remote_connect_timer.timeout.connect(self.connect_remote_sender)
        self._remote_connect_timer.start(1200)
        self._build_ui()
        self.refresh_ports()

    def _build_ui(self) -> None:
        tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(tabs)
        tabs.addTab(self._connection_page(), "连接/参数/校准")
        tabs.addTab(self._acquisition_page(), "数据采集/B-scan")
        self.statusBar().showMessage("Ready")

    def _spin(self, value: float, min_v: float, max_v: float, decimals: int = 0) -> QtWidgets.QDoubleSpinBox:
        box = QtWidgets.QDoubleSpinBox()
        box.setRange(min_v, max_v)
        box.setDecimals(decimals)
        box.setValue(value)
        box.setKeyboardTracking(False)
        return box

    def _connection_page(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(page)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)

        dev_group = QtWidgets.QGroupBox("设备连接")
        dev = QtWidgets.QGridLayout(dev_group)
        self.port_combo = QtWidgets.QComboBox()
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["115200", "9600", "38400", "57600"])
        refresh = QtWidgets.QPushButton("刷新串口")
        refresh.clicked.connect(self.refresh_ports)
        test_gnss = QtWidgets.QPushButton("测试 GNSS")
        test_gnss.clicked.connect(self.test_gnss)
        self.test_gnss_btn = test_gnss
        self.test_vna_btn = QtWidgets.QPushButton("测试 LibreVNA USB")
        self.test_vna_btn.clicked.connect(self.test_vna)
        self.device_log = QtWidgets.QPlainTextEdit()
        self.device_log.setReadOnly(True)
        self.device_log.setMaximumBlockCount(200)
        self.device_log.setMinimumHeight(180)
        dev.addWidget(QtWidgets.QLabel("GNSS 串口"), 0, 0)
        dev.addWidget(self.port_combo, 0, 1)
        dev.addWidget(refresh, 0, 2)
        dev.addWidget(QtWidgets.QLabel("波特率"), 1, 0)
        dev.addWidget(self.baud_combo, 1, 1)
        dev.addWidget(test_gnss, 2, 0, 1, 2)
        dev.addWidget(self.test_vna_btn, 2, 2)
        dev.addWidget(self.device_log, 3, 0, 1, 3)
        dev.setColumnStretch(1, 1)

        sweep_group = QtWidgets.QGroupBox("VNA 基础参数")
        sweep = QtWidgets.QGridLayout(sweep_group)
        self.start_mhz = self._spin(400, 0.001, 6000, 3)
        self.stop_mhz = self._spin(4000, 0.001, 6000, 3)
        self.points = QtWidgets.QSpinBox()
        self.points.setRange(2, 4501)
        self.points.setValue(201)
        self.ifbw = QtWidgets.QSpinBox()
        self.ifbw.setRange(6, 50000)
        self.ifbw.setValue(10000)
        self.power = self._spin(-20, -40, 0, 1)
        self.dwell = QtWidgets.QSpinBox()
        self.dwell.setRange(0, 10239)
        self.ifft_points = QtWidgets.QSpinBox()
        self.ifft_points.setRange(256, 65536)
        self.ifft_points.setValue(2048)
        self.time_window = self._spin(40, 1, 1000, 1)
        self.dewow_window = self._spin(4, 0.001, 1000, 3)
        self.dewow_enabled = QtWidgets.QCheckBox("启用 Dewow")
        self.dewow_enabled.setChecked(False)
        self.dewow_window.setEnabled(False)
        self.dewow_enabled.toggled.connect(self.dewow_window.setEnabled)
        fields = [
            ("Start MHz", self.start_mhz), ("Stop MHz", self.stop_mhz),
            ("Points", self.points), ("IFBW Hz", self.ifbw),
            ("Power dBm", self.power), ("Dwell us", self.dwell),
            ("IFFT points", self.ifft_points), ("Time window ns", self.time_window),
            ("Dewow window ns", self.dewow_window),
        ]
        for index, (label, widget) in enumerate(fields):
            row = index // 2
            col = (index % 2) * 2
            sweep.addWidget(QtWidgets.QLabel(label), row, col)
            sweep.addWidget(widget, row, col + 1)
        sweep.addWidget(self.dewow_enabled, 5, 0, 1, 2)
        self.test_sweep_btn = QtWidgets.QPushButton("按当前设置测试扫频")
        self.test_sweep_btn.clicked.connect(self.test_sweep_settings)
        sweep.addWidget(self.test_sweep_btn, 6, 0, 1, 4)
        sweep.setColumnStretch(1, 1)
        sweep.setColumnStretch(3, 1)

        cal_group = QtWidgets.QGroupBox("校准")
        cal = QtWidgets.QGridLayout(cal_group)
        self.cal_path = QtWidgets.QLineEdit()
        browse_cal = QtWidgets.QPushButton("加载校准文件")
        browse_cal.clicked.connect(self.choose_cal)
        self.cal_mode = QtWidgets.QComboBox()
        self.cal_mode.addItems(["一端口校准 P1 OSL（三项）", "两端口校准 SOLT/SLOT（七项）"])
        self.cal_mode.currentIndexChanged.connect(lambda: self.update_calibration_status())
        self.cal_lamps: dict[str, QtWidgets.QLabel] = {}
        build_cal = QtWidgets.QPushButton("生成并启用 SOLT/SLOT 校准")
        build_cal.clicked.connect(self.build_session_calibration)
        save_cal = QtWidgets.QPushButton("保存当前校准")
        save_cal.clicked.connect(self.save_session_calibration)
        self.capture_bg_btn = QtWidgets.QPushButton("采集背景扣除")
        self.capture_bg_btn.clicked.connect(self.capture_background)
        clear_bg = QtWidgets.QPushButton("清除背景")
        clear_bg.clicked.connect(self.clear_background)
        self.bg_label = QtWidgets.QLabel("未采集背景")
        self.cal_status = QtWidgets.QLabel("未启用校准")
        self.cal_status.setWordWrap(True)
        cal.addWidget(QtWidgets.QLabel("LibreVNA 校准文件"), 0, 0)
        cal.addWidget(self.cal_path, 0, 1, 1, 2)
        cal.addWidget(browse_cal, 0, 3)
        cal.addWidget(QtWidgets.QLabel("校准模式"), 1, 0)
        cal.addWidget(self.cal_mode, 1, 1, 1, 3)

        self.cal_step_buttons: dict[str, QtWidgets.QPushButton] = {}
        for index, (key, label) in enumerate(CALIBRATION_STEPS, start=1):
            row = 1 + index
            lamp = QtWidgets.QLabel()
            lamp.setFixedSize(16, 16)
            lamp.setToolTip(f"{label} 未采集")
            self.cal_lamps[key] = lamp
            button = QtWidgets.QPushButton(f"{index} 采集 {label}")
            button.clicked.connect(lambda _checked=False, step_key=key: self.capture_calibration_standard(step_key))
            self.cal_step_buttons[key] = button
            cal.addWidget(lamp, row, 0, QtCore.Qt.AlignmentFlag.AlignCenter)
            cal.addWidget(button, row, 1, 1, 3)

        cal.addWidget(build_cal, 9, 0, 1, 2)
        cal.addWidget(save_cal, 9, 2, 1, 2)
        cal.addWidget(self.capture_bg_btn, 10, 0, 1, 2)
        cal.addWidget(clear_bg, 10, 2, 1, 2)
        cal.addWidget(self.cal_status, 11, 0, 1, 4)
        cal.addWidget(self.bg_label, 12, 0, 1, 4)
        cal.setColumnStretch(1, 1)
        self.update_calibration_lamps()

        layout.addWidget(dev_group, 0, 0)
        layout.addWidget(sweep_group, 1, 0)
        layout.addWidget(cal_group, 0, 1, 2, 1)
        layout.setColumnStretch(0, 3)
        layout.setColumnStretch(1, 2)
        layout.setRowStretch(0, 1)
        layout.setRowStretch(1, 0)
        return page

    def _acquisition_page(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(page)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)
        controls = QtWidgets.QGroupBox("采集控制")
        ctl = QtWidgets.QGridLayout(controls)
        self.output_dir = QtWidgets.QLineEdit(str(Path.cwd() / "gpr_runs"))
        browse_out = QtWidgets.QPushButton("输出目录")
        browse_out.clicked.connect(self.choose_output)
        # 第1轮检查点C：默认"无采集时间间隔"（interval_s=0 = 每个完整Sweep）。
        self.interval = self._spin(0.0, 0.0, 3600.0, 2)
        self.count = QtWidgets.QSpinBox()
        self.count.setRange(0, 1000000)
        self.count.setValue(0)
        self.autosave = QtWidgets.QCheckBox("自动保存 CSV/NPZ")
        self.autosave.setChecked(True)
        self.remote_enable = QtWidgets.QCheckBox("Remote send")
        self.remote_enable.setChecked(True)
        self.remote_host = QtWidgets.QLineEdit("127.0.0.1")
        self.remote_host.setMaximumWidth(140)
        self.remote_port = QtWidgets.QSpinBox()
        self.remote_port.setRange(1, 65535)
        self.remote_port.setValue(8765)
        self.remote_port.setMaximumWidth(90)
        self.remote_connect = QtWidgets.QPushButton("Connect")
        self.remote_connect.clicked.connect(self.connect_remote_sender)
        self.bscan_cmap = QtWidgets.QComboBox()
        self.bscan_cmap.addItems(["gray", "jet"])
        self.bscan_cmap.currentTextChanged.connect(lambda _text: self.refresh_bscan_views())
        self.bscan_aspect = QtWidgets.QDoubleSpinBox()
        self.bscan_aspect.setRange(0.5, 6.0)
        self.bscan_aspect.setDecimals(2)
        self.bscan_aspect.setSingleStep(0.1)
        self.bscan_aspect.setValue(2.0)
        self.flat_window_traces = QtWidgets.QSpinBox()
        self.flat_window_traces.setRange(3, 10001)
        self.flat_window_traces.setSingleStep(2)
        self.flat_window_traces.setValue(31)
        self.flat_filter_enabled = QtWidgets.QCheckBox("启用 Flat Filter")
        self.flat_filter_enabled.setChecked(False)
        self.flat_window_traces.setEnabled(False)
        self.flat_filter_enabled.toggled.connect(self.update_flat_filter_enabled)
        self.bscan_window_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.bscan_window_slider.setRange(1, 1)
        self.bscan_window_slider.setValue(1)
        self.bscan_window_slider.valueChanged.connect(self.on_bscan_window_changed)
        self.bscan_window_label = QtWidgets.QLabel("显示 1–0 / 共 0 道")
        self.bscan_filter_status = QtWidgets.QLabel("Flat Filter: 无数据")
        self.start_btn = QtWidgets.QPushButton("开始采集")
        self.start_btn.clicked.connect(self.start_acquisition)
        self.single_btn = QtWidgets.QPushButton("采集一次")
        self.single_btn.clicked.connect(lambda: self.start_acquisition(single=True))
        self.stop_btn = QtWidgets.QPushButton("停止")
        self.stop_btn.clicked.connect(self.stop_acquisition)
        clear = QtWidgets.QPushButton("清空 B-scan")
        clear.clicked.connect(self.clear_bscan)
        self.last_info = QtWidgets.QLabel("No data")
        ctl.addWidget(QtWidgets.QLabel("输出目录"), 0, 0)
        ctl.addWidget(self.output_dir, 0, 1, 1, 6)
        ctl.addWidget(browse_out, 0, 7)
        ctl.addWidget(QtWidgets.QLabel("间隔 s"), 1, 0)
        ctl.addWidget(self.interval, 1, 1)
        ctl.addWidget(QtWidgets.QLabel("次数(0=连续)"), 1, 2)
        ctl.addWidget(self.count, 1, 3)
        ctl.addWidget(self.autosave, 1, 4)
        ctl.addWidget(self.single_btn, 1, 5)
        ctl.addWidget(self.start_btn, 1, 6)
        ctl.addWidget(self.stop_btn, 1, 7)
        ctl.addWidget(QtWidgets.QLabel("B-scan 色图"), 2, 0)
        ctl.addWidget(self.bscan_cmap, 2, 1)
        ctl.addWidget(QtWidgets.QLabel("B-scan W:H"), 2, 2)
        ctl.addWidget(self.bscan_aspect, 2, 3)
        ctl.addWidget(QtWidgets.QLabel("Flat window"), 2, 4)
        ctl.addWidget(self.flat_window_traces, 2, 5)
        ctl.addWidget(self.flat_filter_enabled, 2, 6)
        ctl.addWidget(clear, 2, 7)
        ctl.addWidget(self.remote_enable, 3, 0)
        ctl.addWidget(QtWidgets.QLabel("Remote IP"), 3, 1)
        ctl.addWidget(self.remote_host, 3, 2)
        ctl.addWidget(QtWidgets.QLabel("Port"), 3, 3)
        ctl.addWidget(self.remote_port, 3, 4)
        ctl.addWidget(self.remote_connect, 3, 5)
        ctl.addWidget(self.bscan_window_slider, 4, 0, 1, 5)
        ctl.addWidget(self.bscan_window_label, 4, 5, 1, 3)
        ctl.addWidget(self.bscan_filter_status, 5, 0, 1, 8)
        ctl.addWidget(self.last_info, 6, 0, 1, 8)
        self.continuous_enabled = QtWidgets.QCheckBox("连续采集 (continuous)")
        self.continuous_enabled.setChecked(is_continuous_enabled())
        pause_btn = QtWidgets.QPushButton("暂停")
        pause_btn.clicked.connect(self.pause_acquisition)
        resume_btn = QtWidgets.QPushButton("继续")
        resume_btn.clicked.connect(self.resume_acquisition)
        self.continuous_status_label = QtWidgets.QLabel("连续模式: 未启用")
        ctl.addWidget(self.continuous_enabled, 7, 0, 1, 2)
        ctl.addWidget(pause_btn, 7, 2)
        ctl.addWidget(resume_btn, 7, 3)
        ctl.addWidget(self.continuous_status_label, 8, 0, 1, 8)
        ctl.setColumnStretch(1, 1)
        ctl.setColumnStretch(6, 1)

        self.sparam_plot = pg.PlotWidget(title="S-parameter magnitude")
        self.sparam_plot.setLabel("bottom", "Frequency", units="Hz")
        self.sparam_plot.setLabel("left", "Magnitude", units="dB")
        self.sparam_plot.showGrid(x=True, y=True, alpha=0.25)
        self.sparam_plot.setMouseEnabled(x=False, y=False)
        self.sparam_plot.setMenuEnabled(False)
        self.sparam_plot.getPlotItem().hideButtons()
        self.sparam_legend = self.sparam_plot.getPlotItem().addLegend(offset=(10, 10))
        self.sparam_curves: dict[str, pg.PlotDataItem] = {}
        for name in UI_CHANNELS:
            curve = pg.PlotDataItem(pen=pg.mkPen(CHANNEL_COLORS[name], width=2), name=CHANNEL_LABELS[name])
            self.sparam_curves[name] = curve
            self.sparam_plot.addItem(curve)
            self.sparam_legend.addItem(curve, CHANNEL_LABELS[name])
        self.sparam_plot.setXRange(400e6, 4000e6, padding=0)
        self.sparam_plot.setYRange(-60, 10, padding=0)

        self.ascan_plot = pg.PlotWidget(title="A-scan")
        self.ascan_plot.setLabel("bottom", "Time", units="ns")
        self.ascan_plot.setLabel("left", "Normalized amplitude")
        self.ascan_plot.showGrid(x=True, y=True, alpha=0.25)
        self.ascan_plot.setMouseEnabled(x=False, y=False)
        self.ascan_plot.setMenuEnabled(False)
        self.ascan_plot.getPlotItem().hideButtons()
        self.ascan_plot.addLegend(offset=(10, 10))
        self.ascan_plot.setMinimumHeight(125)
        self.ascan_curves: dict[str, pg.PlotDataItem] = {}
        for name in UI_CHANNELS:
            self.ascan_curves[name] = self.ascan_plot.plot(
                pen=pg.mkPen(CHANNEL_COLORS[name], width=2),
                name=CHANNEL_LABELS[name],
            )
        self.ascan_plot.setXRange(0, 40, padding=0)
        self.ascan_plot.setYRange(-1.05, 1.05, padding=0)

        bscan_panel = QtWidgets.QWidget()
        # B-scans are the primary acquisition view.  Leave just enough space
        # for S-parameter/A-scan diagnostics above them at the default size.
        bscan_panel.setMinimumHeight(315)
        bscan_grid = QtWidgets.QGridLayout(bscan_panel)
        bscan_grid.setContentsMargins(0, 0, 0, 0)
        bscan_grid.setSpacing(8)
        self.bscan_views: dict[str, BScanWidget] = {}
        for index, name in enumerate(UI_CHANNELS):
            view = BScanWidget(f"{CHANNEL_LABELS[name]} B-scan")
            view.setMinimumHeight(295)
            view.set_colormap(self.bscan_cmap.currentText())
            self.bscan_views[name] = view
            bscan_grid.addWidget(view, index // 2, index % 2)
        self.bscan_aspect.valueChanged.connect(self.update_bscan_aspect_ratio)
        self.flat_window_traces.valueChanged.connect(lambda _value: self.refresh_bscan_views())
        self.update_flat_filter_enabled(self.flat_filter_enabled.isChecked())
        self.update_bscan_aspect_ratio(self.bscan_aspect.value())

        plot_area = QtWidgets.QWidget()
        plot_area.setMinimumHeight(300)
        plot_area.setMaximumHeight(330)
        plot_area_layout = QtWidgets.QHBoxLayout(plot_area)
        plot_area_layout.setContentsMargins(0, 0, 0, 0)
        plot_area_layout.setSpacing(8)
        left_plots = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_plots)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(8)
        left_layout.addWidget(self.sparam_plot, 1)
        left_layout.addWidget(self.ascan_plot, 1)
        self.gnss_map = GnssMapWidget()
        plot_area_layout.addWidget(left_plots, 3)
        plot_area_layout.addWidget(self.gnss_map, 2)

        layout.addWidget(controls, 0, 0)
        layout.addWidget(plot_area, 1, 0)
        layout.addWidget(bscan_panel, 2, 0)
        layout.setRowStretch(1, 1)
        layout.setRowStretch(2, 4)
        layout.setColumnStretch(0, 1)
        return page

    def config(self) -> SweepConfig:
        return SweepConfig(
            start_mhz=self.start_mhz.value(),
            stop_mhz=self.stop_mhz.value(),
            points=self.points.value(),
            ifbw_hz=self.ifbw.value(),
            power_dbm=self.power.value(),
            dwell_us=self.dwell.value(),
            ifft_points=self.ifft_points.value(),
            time_window_ns=self.time_window.value(),
            dewow_enabled=self.dewow_enabled.isChecked(),
            dewow_window_ns=self.dewow_window.value(),
        )

    def update_sparam_ranges(self, frequency_hz: np.ndarray, display: dict[str, np.ndarray]) -> None:
        if frequency_hz.size:
            self.sparam_plot.setXRange(float(frequency_hz[0]), float(frequency_hz[-1]), padding=0)
        values = [db20(display[name]) for name in UI_CHANNELS if name in display]
        self.sparam_plot.setYRange(*robust_db_range(values, default=(-60.0, 10.0)), padding=0)

    def update_ascan_range(self, time_ns: np.ndarray) -> None:
        if time_ns.size:
            self.ascan_plot.setXRange(float(time_ns[0]), float(time_ns[-1]), padding=0)
        self.ascan_plot.setYRange(-1.05, 1.05, padding=0)

    def log(self, text: str) -> None:
        self.device_log.appendPlainText(f"[{datetime.now().strftime('%H:%M:%S')}] {text}")
        self.statusBar().showMessage(text)

    def refresh_ports(self) -> None:
        self.port_combo.clear()
        ports = list(list_ports.comports())
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}", port.device)
        com3 = self.port_combo.findData("COM3")
        if com3 >= 0:
            self.port_combo.setCurrentIndex(com3)
        self.log(f"发现 {len(ports)} 个串口")

    def selected_port(self) -> str:
        return self.port_combo.currentData() or ""

    def test_gnss(self) -> None:
        try:
            reader = GnssReader(self.selected_port(), int(self.baud_combo.currentText()))
            fix = reader.read_gga()
            reader.close()
            self.log(f"GNSS: {fix.raw}")
            self.log(f"fix={fix.fix_quality}, sats={fix.satellites}, lat={fix.latitude}, lon={fix.longitude}")
            if hasattr(self, "gnss_map"):
                self.gnss_map.update_fix(
                    latitude=fix.latitude,
                    longitude=fix.longitude,
                    fix_quality=fix.fix_quality,
                    satellites=fix.satellites,
                    altitude_m=fix.altitude_m,
                    utc=fix.utc,
                )
        except Exception as exc:
            self.log(f"GNSS 测试失败: {exc}")

    def test_vna(self) -> None:
        if self._reject_manual_usb_op("测试 LibreVNA USB"):
            return
        try:
            dev, interface = open_device()
            try:
                info = request_device_info(dev, PacketStream())
            finally:
                import usb.util
                usb.util.release_interface(dev, interface)
                usb.util.dispose_resources(dev)
            self.log(f"LibreVNA: FW {info['firmware']}, max {info['max_freq_hz'] / 1e9:g} GHz, points {info['max_points']}")
        except Exception as exc:
            self.log(f"LibreVNA 测试失败: {exc}")

    def test_sweep_settings(self) -> None:
        if self._reject_manual_usb_op("按当前设置测试扫频"):
            return
        try:
            cfg = self.config()
            if cfg.stop_mhz <= cfg.start_mhz:
                self.log("Stop MHz 必须大于 Start MHz")
                return
            points = acquire_sparameters(
                int(cfg.start_mhz * 1e6),
                int(cfg.stop_mhz * 1e6),
                cfg.points,
                cfg.ifbw_hz,
                cfg.power_dbm,
                cfg.dwell_us,
            )
            freq, raw = sparameter_arrays(points)
            calibration_p1, calibration_p2, through_measurement = self.active_calibration_components()
            display, time_ns, trace_by_channel = process_trace_pipeline(
                freq,
                raw,
                cfg,
                calibration_p1=calibration_p1,
                calibration_p2=calibration_p2,
                through_measurement=through_measurement,
                background_frequency_hz=self.background_frequency_hz,
                background=self.background,
            )
            for name in UI_CHANNELS:
                self.sparam_curves[name].setData(freq, db20(display[name]))
                self.ascan_curves[name].setData(time_ns, normalize_trace_for_display(trace_by_channel[name]))
            self.update_sparam_ranges(freq, display)
            self.update_ascan_range(time_ns)
            self.log(
                f"当前参数扫频成功: {cfg.start_mhz:g}-{cfg.stop_mhz:g} MHz, "
                f"{cfg.points} 点, IFBW {cfg.ifbw_hz} Hz, power {cfg.power_dbm:g} dBm, "
                "通道 S11/S21/S12/S22"
            )
        except Exception as exc:
            self.log(f"参数测试扫频失败: {exc}")

    def choose_cal(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "选择校准文件",
            str(Path.cwd()),
            "Calibration (*.json *.cal);;All files (*)",
        )
        if path:
            self.cal_path.setText(path)
            try:
                measurements = read_calibration_measurements(Path(path))
                if measurements:
                    for new_key, legacy_key in LEGACY_OSL_MAP.items():
                        if legacy_key in measurements and new_key not in measurements:
                            measurements[new_key] = measurements[legacy_key]
                    self.cal_measurements = measurements
                    self.session_calibration = calibration_from_measurements(measurements, Path(path))
                    try:
                        self.session_calibration_p2 = calibration_from_measurements(measurements, Path(path), port=2)
                    except Exception:
                        self.session_calibration_p2 = None
                    self.through_measurement = measurements.get("Through")
                    self.update_calibration_status(f"已加载并启用 {Path(path).name}")
                else:
                    self.session_calibration = None
                    self.session_calibration_p2 = None
                    self.through_measurement = None
                    self.update_calibration_status(f"将使用外部校准文件 {Path(path).name}")
            except Exception:
                self.session_calibration = None
                self.session_calibration_p2 = None
                self.through_measurement = None
                self.update_calibration_status(f"将使用外部校准文件 {Path(path).name}")

    def choose_output(self) -> None:
        path = QtWidgets.QFileDialog.getExistingDirectory(self, "选择输出目录", self.output_dir.text())
        if path:
            self.output_dir.setText(path)

    def update_calibration_status(self, message: str | None = None) -> None:
        self.update_calibration_lamps()
        done = ", ".join(CALIBRATION_STEP_LABELS[name] for name, _label in CALIBRATION_STEPS if name in self.cal_measurements)
        required = list(P1_OSL_KEYS) if self.cal_mode.currentIndex() == 0 else [key for key, _label in CALIBRATION_STEPS]
        ready = sum(1 for key in required if key in self.cal_measurements)
        prefix = f"已采集: {done or '无'}; 当前模式 {ready}/{len(required)}"
        if self.session_calibration:
            prefix += f"; 已启用 {len(self.session_calibration.points)} 点校准"
        self.cal_status.setText(message or prefix)
        self.log(message or prefix)

    def update_calibration_lamps(self) -> None:
        for key, label in CALIBRATION_STEPS:
            lamp = self.cal_lamps.get(key)
            if not lamp:
                continue
            captured = key in self.cal_measurements
            color = "#22c55e" if captured else "#9ca3af"
            border = "#166534" if captured else "#6b7280"
            lamp.setStyleSheet(f"border-radius: 8px; background: {color}; border: 1px solid {border};")
            lamp.setToolTip(f"{label} {'已采集' if captured else '未采集'}")

    def capture_calibration_standard(self, standard: str) -> None:
        if self._reject_manual_usb_op(f"采集校准标准 {standard}"):
            return
        try:
            cfg = self.config()
            if cfg.stop_mhz <= cfg.start_mhz:
                self.log("Stop MHz 必须大于 Start MHz")
                return
            points = acquire_sparameters(
                int(cfg.start_mhz * 1e6),
                int(cfg.stop_mhz * 1e6),
                cfg.points,
                cfg.ifbw_hz,
                cfg.power_dbm,
                cfg.dwell_us,
            )
            primary = "S11"
            if standard.startswith("P2_"):
                primary = "S22"
            elif standard == "Through":
                primary = "S21"
            self.cal_measurements[standard] = CalibrationMeasurement.from_sparameter_points(points, primary=primary)
            self.session_calibration = None
            self.session_calibration_p2 = None
            self.through_measurement = self.cal_measurements.get("Through")
            label = CALIBRATION_STEP_LABELS.get(standard, standard)
            self.update_calibration_status(f"{label} 已采集: {len(points)} 点，已记录 S11/S21/S12/S22")
        except Exception as exc:
            label = CALIBRATION_STEP_LABELS.get(standard, standard)
            self.log(f"{label} 校准采集失败: {exc}")

    def build_session_calibration(self) -> None:
        try:
            if self.cal_mode.currentIndex() == 1:
                missing_two_port = [CALIBRATION_STEP_LABELS[key] for key, _label in CALIBRATION_STEPS if key not in self.cal_measurements]
                if missing_two_port:
                    self.log(f"两端口校准还缺少: {', '.join(missing_two_port)}")
                    return
            self.session_calibration = calibration_from_measurements(self.cal_measurements)
            try:
                self.session_calibration_p2 = calibration_from_measurements(self.cal_measurements, port=2)
            except Exception:
                self.session_calibration_p2 = None
            self.through_measurement = self.cal_measurements.get("Through")
            self.cal_path.clear()
            message = "P1 S11 一端口 OSL 校准已生成并启用"
            if self.session_calibration_p2:
                message += "；P2 S22 一端口 OSL 校准已生成并启用"
            if self.cal_mode.currentIndex() == 1:
                message += "；七项两端口数据已记录，采集文件保存 S11/S21/S12/S22"
            self.update_calibration_status(message)
        except Exception as exc:
            self.log(f"生成校准失败: {exc}")

    def save_session_calibration(self) -> None:
        try:
            required = list(P1_OSL_KEYS) if self.cal_mode.currentIndex() == 0 else [key for key, _label in CALIBRATION_STEPS]
            missing = [CALIBRATION_STEP_LABELS[key] for key in required if key not in self.cal_measurements]
            if missing:
                self.log(f"需要先采集: {', '.join(missing)}")
                return
            default = Path.cwd() / "calibration" / f"solt_2port_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self,
                "保存校准文件",
                str(default),
                "Calibration JSON (*.json);;All files (*)",
            )
            if not path:
                return
            write_calibration_file(Path(path), self.cal_measurements, self.config())
            self.cal_path.setText(path)
            self.session_calibration = calibration_from_measurements(self.cal_measurements, Path(path))
            try:
                self.session_calibration_p2 = calibration_from_measurements(self.cal_measurements, Path(path), port=2)
            except Exception:
                self.session_calibration_p2 = None
            self.through_measurement = self.cal_measurements.get("Through")
            self.update_calibration_status(f"校准已保存并启用: {Path(path).name}")
        except Exception as exc:
            self.log(f"保存校准失败: {exc}")

    def active_calibration_components(
        self,
    ) -> tuple[OSLCalibration | None, OSLCalibration | None, CalibrationMeasurement | None]:
        calibration_p1 = self.session_calibration
        calibration_p2 = self.session_calibration_p2
        if calibration_p1 is None and self.cal_path.text().strip():
            calibration_p1 = OSLCalibration.from_file(self.cal_path.text().strip())
        return calibration_p1, calibration_p2, self.through_measurement

    def apply_active_calibration(self, frequency_hz: np.ndarray, raw_sparams: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        calibration_p1, calibration_p2, through_measurement = self.active_calibration_components()
        return apply_reflection_and_through_calibration(
            frequency_hz,
            raw_sparams,
            calibration_p1,
            calibration_p2,
            through_measurement,
        )

    def capture_background(self) -> None:
        if self._reject_manual_usb_op("采集背景扣除"):
            return
        try:
            cfg = self.config()
            points = acquire_sparameters(int(cfg.start_mhz * 1e6), int(cfg.stop_mhz * 1e6), cfg.points, cfg.ifbw_hz, cfg.power_dbm, cfg.dwell_us)
            freq, raw = sparameter_arrays(points)
            self.background = self.apply_active_calibration(freq, raw)["S11"]
            self.background_frequency_hz = np.asarray(freq, dtype=np.float64).copy()
            self.bg_label.setText(f"已采集 {len(self.background)} 点背景")
            self.log("背景扣除数据已采集")
        except Exception as exc:
            self.log(f"背景采集失败: {exc}")

    def clear_background(self) -> None:
        self.background = None
        self.background_frequency_hz = None
        self.bg_label.setText("未采集背景")
        self.log("已清除背景")

    def _acquisition_running(self) -> bool:
        """任一采集 worker 仍在运行（legacy 或 continuous）。"""
        return self.worker is not None and self.worker.isRunning()

    def _reject_manual_usb_op(self, action: str) -> bool:
        """任务运行期间拒绝手动访问 LibreVNA 的操作；返回 True 表示已拒绝。

        方法入口必须做运行状态检查（不能只依赖按钮 disabled）；拒绝时记录明确
        日志且绝不触发 USB 调用（roadmap §3 互斥）。
        """
        if self._acquisition_running():
            self.log(f"{action}: 采集任务运行中，禁止手动访问 LibreVNA")
            return True
        return False

    def _set_task_controls_locked(self, locked: bool) -> None:
        """任务运行期间锁定会访问 LibreVNA / 改变冻结任务配置的控件。

        停止选择新数据后仍可用的安全控件（Stop、B-scan 浏览等）不在此列表。
        """
        for widget in (
            self.start_mhz, self.stop_mhz, self.points, self.ifbw, self.power,
            self.dwell, self.ifft_points, self.time_window, self.dewow_window,
            self.dewow_enabled, self.interval, self.count, self.autosave,
            self.continuous_enabled, self.cal_path, self.cal_mode,
            self.test_gnss_btn, self.test_vna_btn, self.test_sweep_btn,
            self.capture_bg_btn, self.start_btn, self.single_btn,
        ):
            widget.setEnabled(not locked)
        # Stop / B-scan 浏览等安全控件保持可用（不在上述列表）。
        for button in self.cal_step_buttons.values():
            button.setEnabled(not locked)

    def start_acquisition(self, single: bool = False) -> None:
        # 只要 self.worker 存在（无论是否仍在运行）就不启动另一个任务，避免旧
        # worker 的 finished_run queued slot 尚未处理时清除新 worker。
        if self.worker is not None:
            if self.worker.isRunning():
                self.log("采集已经在运行")
            else:
                self.log("上一任务正在完成清理，请稍候")
            return
        cfg = self.config()
        if cfg.stop_mhz <= cfg.start_mhz:
            self.log("Stop MHz 必须大于 Start MHz")
            return
        if self.remote_enable.isChecked():
            self.connect_remote_sender()
        if self.continuous_enabled.isChecked():
            self._start_continuous_acquisition(cfg, single)
            return
        self.worker = AcquisitionWorker(
            cfg=cfg,
            gnss_port=self.selected_port(),
            baud=int(self.baud_combo.currentText()),
            out_dir=Path(self.output_dir.text()),
            count=1 if single else self.count.value(),
            interval_s=self.interval.value(),
            calibration_path=self.cal_path.text().strip(),
            session_calibration=self.session_calibration,
            session_calibration_p2=self.session_calibration_p2,
            through_measurement=self.through_measurement,
            background_frequency_hz=self.background_frequency_hz,
            background=self.background,
            autosave=self.autosave.isChecked(),
        )
        self.worker.trace_ready.connect(self.on_trace)
        self.worker.status.connect(self.log)
        self.worker.failed.connect(lambda msg: self.log(f"采集失败: {msg}"))
        self.worker.finished_run.connect(self.on_worker_finished)
        self.worker.start()
        self._set_task_controls_locked(True)

    def _start_continuous_acquisition(self, cfg: SweepConfig, single: bool) -> None:
        self.worker = ContinuousAcquisitionWorker(
            cfg=cfg,
            interval_s=self.interval.value(),
            out_dir=Path(self.output_dir.text()),
            count=1 if single else self.count.value(),
            gnss_port=self.selected_port(),
            baud=int(self.baud_combo.currentText()),
            calibration_path=self.cal_path.text().strip(),
            session_calibration=self.session_calibration,
            session_calibration_p2=self.session_calibration_p2,
            through_measurement=self.through_measurement,
            background_frequency_hz=self.background_frequency_hz,
            background=self.background,
            autosave=self.autosave.isChecked(),
            remote_sender=self.remote_sender,
        )
        self.worker.trace_ready.connect(self.on_continuous_trace)
        self.worker.status.connect(self.log)
        self.worker.failed.connect(lambda msg: self.log(f"连续采集失败: {msg}"))
        self.worker.finished_run.connect(self.on_worker_finished)
        self.worker.paused.connect(lambda: self.log("连续采集已暂停"))
        self.worker.resumed.connect(lambda: self.log("连续采集已恢复"))
        self.worker.statistics_updated.connect(self.on_continuous_statistics)
        self.worker.start()
        self._set_task_controls_locked(True)
        self.continuous_status_label.setText("连续模式: 运行中...")

    def pause_acquisition(self) -> None:
        worker = self.worker
        if isinstance(worker, ContinuousAcquisitionWorker) and worker.isRunning():
            worker.request_pause()
            self.log("请求暂停连续采集...")
        else:
            self.log("当前采集不支持暂停")

    def resume_acquisition(self) -> None:
        worker = self.worker
        if isinstance(worker, ContinuousAcquisitionWorker) and worker.isRunning():
            worker.request_resume()
            self.log("请求继续连续采集...")
        else:
            self.log("当前采集不支持继续")

    def on_continuous_statistics(self, stats: dict) -> None:
        scheduler = stats.get("scheduler", {})
        backend = stats.get("backend", {})
        queue_stats = stats.get("processing_queue", {})
        intervals = scheduler.get("actual_interval_s", {}) or {}
        mean_interval = intervals.get("mean")
        interval_text = f"{mean_interval:.3f}s" if mean_interval is not None else "-"
        gnss_age = stats.get("gnss_age_ms")
        age_text = f"{gnss_age:.0f}ms" if gnss_age is not None else "-"
        self.continuous_status_label.setText(
            "连续模式: "
            f"rate={stats.get('sweep_rate_hz', 0.0):.2f}Hz | "
            f"device={backend.get('completed_sweeps', 0)} "
            f"drop={backend.get('dropped_sweeps', 0)} | "
            f"selected={scheduler.get('selected_sweeps', 0)} "
            f"missed={scheduler.get('missed_slots_total', 0)} | "
            f"interval={interval_text} | "
            f"q={queue_stats.get('depth', 0)}/{queue_stats.get('maxsize', 0)} "
            f"overflow={queue_stats.get('overflows', 0)} | "
            f"gnss age={age_text}"
        )

    def connect_remote_sender(self) -> None:
        if self._closing:
            return
        if not hasattr(self, "remote_enable") or not self.remote_enable.isChecked():
            return
        if self.remote_sender is not None:
            self.log("Remote sender already running")
            return
        host = self.remote_host.text().strip()
        if not host:
            self.log("Remote IP is empty")
            return
        self.remote_run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.remote_sender = RemoteSender(
            host,
            int(self.remote_port.value()),
            self.remote_run_id,
            self.remote_status.emit,
            self.remote_command.emit,
        )
        self.remote_sender.start()
        self.log(f"Remote sender connecting: {host}:{self.remote_port.value()}")

    def handle_remote_command(self, action: str, params: object) -> None:
        self.log(f"Remote command: {action}")
        if isinstance(params, dict):
            if "count" in params:
                self.count.setValue(max(0, min(int(params["count"]), self.count.maximum())))
            if "interval_s" in params:
                self.interval.setValue(float(params["interval_s"]))
        if action == "start":
            single = bool(params.get("single", False)) if isinstance(params, dict) else False
            self.start_acquisition(single=single)
        elif action == "stop":
            self.stop_acquisition()
        elif action == "single":
            self.start_acquisition(single=True)
        else:
            self.log(f"Unknown remote command: {action}")

    def stop_acquisition(self) -> None:
        if self.worker:
            self.worker.stop()
            self.log("正在停止采集...")

    def on_worker_finished(self) -> None:
        # 只接受当前 self.worker 的 finished_run；旧 worker / stale sender 的完成
        # 信号不得清除或解锁一个较新的 worker。
        sender = self.sender()
        if sender is not None and sender is not self.worker:
            self.log("忽略旧任务的完成信号")
            return
        self.log("采集结束")
        self.worker = None
        self._set_task_controls_locked(False)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._closing = True
        if self._remote_connect_timer.isActive():
            self._remote_connect_timer.stop()
        # 先停采集线程并等待有界退出（USB 在 worker finally 中释放）；绝不
        # terminate 活线程。超时则忽略关闭事件，由用户稍后重试。
        worker = self.worker
        if worker is not None and worker.isRunning():
            worker.stop()
            if not worker.wait(5000):
                self.log("采集线程未在 5s 内退出，请稍后重试关闭")
                event.ignore()
                return
        if self.remote_sender is not None:
            sender = self.remote_sender
            # 第1轮检查点D：正常关闭先排空发送队列（最多 30s），避免丢已采集
            # Trace；超时仍保留引用，等待其自行退出。
            if not sender.stop(30000, drain=True):
                self.log("Remote sender 仍在退出（含发送队列排空等待），保留引用；请稍后重试关闭")
                event.ignore()
                return
            self.remote_sender = None
        self.worker = None
        super().closeEvent(event)

    def clear_bscan(self) -> None:
        self.bscan = {name: [] for name in UI_CHANNELS}
        self.bscan_time_ns = None
        self.bscan_processing_signature = None
        self.bscan_filter_status_override = None
        self.bscan_window_start = 1
        self.bscan_follow_latest = True
        self.bscan_window_slider.blockSignals(True)
        self.bscan_window_slider.setRange(1, 1)
        self.bscan_window_slider.setValue(1)
        self.bscan_window_slider.blockSignals(False)
        self.bscan_window_label.setText("显示 1–0 / 共 0 道")
        self.bscan_filter_status.setText(
            "Flat Filter: disabled" if not self.flat_filter_enabled.isChecked() else "Flat Filter: 无数据"
        )
        for view in self.bscan_views.values():
            view.clear()
        self.log("B-scan 已清空")

    def _append_bscan_trace(
        self,
        time_ns: np.ndarray,
        trace_by_channel: dict[str, np.ndarray],
        processing_signature: tuple[bool, float | None] | None = None,
    ) -> bool:
        try:
            candidate_time, candidate_traces = validate_bscan_trace(time_ns, trace_by_channel)
        except (TypeError, ValueError) as exc:
            self.bscan_filter_status_override = f"B-scan rejected: {exc}; clear B-scan to restart"
            return False
        if self.bscan_time_ns is not None and not bscan_time_axes_compatible(self.bscan_time_ns, candidate_time):
            self.bscan_filter_status_override = (
                "B-scan rejected: time axis changed; clear B-scan before using the new time axis"
            )
            return False
        if (
            self.bscan_processing_signature is not None
            and processing_signature is not None
            and self.bscan_processing_signature != processing_signature
        ):
            self.bscan_filter_status_override = (
                "B-scan rejected: Dewow setting changed; clear B-scan before continuing"
            )
            return False

        total_before = self.bscan_trace_count()
        was_latest = self.bscan_follow_latest or self.bscan_window_start >= bscan_window_max_start(total_before)
        if self.bscan_time_ns is None:
            self.bscan_time_ns = candidate_time
        if self.bscan_processing_signature is None and processing_signature is not None:
            self.bscan_processing_signature = processing_signature
        for name in UI_CHANNELS:
            self.bscan[name].append(candidate_traces[name])
        total_after = self.bscan_trace_count()
        self.bscan_window_start = bscan_start_after_append(
            self.bscan_window_start,
            total_before,
            total_after,
            was_latest,
        )
        self.bscan_follow_latest = was_latest
        self.bscan_filter_status_override = None
        return True

    def bscan_trace_count(self) -> int:
        return max((len(self.bscan.get(name, [])) for name in UI_CHANNELS), default=0)

    def _update_bscan_window_controls(self) -> None:
        total = self.bscan_trace_count()
        maximum = bscan_window_max_start(total)
        self.bscan_window_start = min(max(self.bscan_window_start, 1), maximum)
        start, end = bscan_window_bounds(total, self.bscan_window_start)
        self.bscan_window_slider.blockSignals(True)
        self.bscan_window_slider.setRange(1, maximum)
        self.bscan_window_slider.setValue(self.bscan_window_start)
        self.bscan_window_slider.blockSignals(False)
        self.bscan_window_label.setText(f"显示 {start}–{end} / 共 {total} 道")

    def on_bscan_window_changed(self, value: int) -> None:
        total = self.bscan_trace_count()
        maximum = bscan_window_max_start(total)
        self.bscan_window_start = min(max(int(value), 1), maximum)
        self.bscan_follow_latest = self.bscan_window_start >= maximum
        self.refresh_bscan_views()

    def update_bscan_aspect_ratio(self, value: float) -> None:
        for view in getattr(self, "bscan_views", {}).values():
            view.set_aspect_ratio(value)

    def update_flat_filter_enabled(self, enabled: bool) -> None:
        self.flat_window_traces.setEnabled(bool(enabled))
        self.refresh_bscan_views()

    def flat_filter_window(self) -> int:
        value = max(3, int(self.flat_window_traces.value()))
        if value % 2 == 0:
            value -= 1
            self.flat_window_traces.blockSignals(True)
            self.flat_window_traces.setValue(value)
            self.flat_window_traces.blockSignals(False)
        return value

    def _bscan_matrix_for_display(self, name: str) -> np.ndarray | None:
        rows = self.bscan.get(name, [])
        if not rows:
            return None
        matrix = np.column_stack([np.asarray(row, dtype=float) for row in rows])
        flat_window = self.flat_filter_window()
        if self.flat_filter_enabled.isChecked() and matrix.shape[1] >= flat_window:
            matrix = flat_reflection_filter(matrix, window_size=flat_window)
        return matrix

    def refresh_bscan_views(self) -> None:
        self._update_bscan_window_controls()
        cmap = self.bscan_cmap.currentText() if hasattr(self, "bscan_cmap") else "gray"
        total = self.bscan_trace_count()
        flat_window = self.flat_filter_window()
        if self.bscan_filter_status_override is not None:
            self.bscan_filter_status.setText(self.bscan_filter_status_override)
        elif not self.flat_filter_enabled.isChecked():
            self.bscan_filter_status.setText("Flat Filter: disabled")
        elif total == 0:
            self.bscan_filter_status.setText("Flat Filter: 无数据")
        elif total < flat_window:
            self.bscan_filter_status.setText(
                f"Flat Filter: waiting for {flat_window} traces; showing unfiltered data"
            )
        else:
            self.bscan_filter_status.setText(f"Flat Filter: active, window {flat_window} traces")
        for name in UI_CHANNELS:
            view = self.bscan_views.get(name)
            if view is None:
                continue
            matrix = self._bscan_matrix_for_display(name)
            if matrix is None:
                continue
            selected = select_bscan_window(matrix, self.bscan_window_start)
            view.set_colormap(cmap)
            view.set_data(selected.matrix, self.bscan_time_ns, selected.start_trace)

    def on_trace(self, index, freq, raw, display, time_ns, trace_by_channel, fix, cfg, path_text) -> None:
        fix_text = _apply_trace_to_ui(self, index, freq, display, time_ns, trace_by_channel, fix, cfg)
        if self.remote_sender is not None:
            self.remote_sender.enqueue_trace(index, cfg, freq, raw, display, time_ns, trace_by_channel, fix)
        self.log(f"Trace {index} complete. {fix_text}" + (f" saved: {Path(path_text).name}" if path_text else ""))

    def on_continuous_trace(self, index, freq, raw, display, time_ns, trace_by_channel, fix, cfg, path_text) -> None:
        # 连续模式：远程发送已由 ProcessingConsumer 完成，这里只更新 UI，避免双发。
        fix_text = _apply_trace_to_ui(self, index, freq, display, time_ns, trace_by_channel, fix, cfg)
        self.log(f"Trace {index} complete. {fix_text}" + (f" saved: {Path(path_text).name}" if path_text else ""))


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True, background="w", foreground="k")
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
