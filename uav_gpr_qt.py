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
import select
import socket
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field, replace
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
from disk_outbox import (  # noqa: E402
    RUN_ID_CLASS_AIR_TASK,
    DiskOutbox,
    OutboxFrameError,
    OutboxWriteError,
    classify_run_id,
    frame_bytes,
)
from pipeline_metrics import PipelineMetrics  # noqa: E402
# 第2轮工作流 C（R-C-09/R-C-10）：校准数据层与编排搬家到 calibration_service.py
# （纯 Python，不 import Qt）；此处 re-export 保持 ``air.*`` 既有引用、既有测试
# 定位点与 MainWindow 接线完全兼容。
from calibration_service import (  # noqa: E402
    BACKGROUND_STEP_NAME,
    CALIBRATION_MODE_BACKGROUND,
    CALIBRATION_MODE_OSL,
    CALIBRATION_STEPS,
    CALIBRATION_STEP_LABELS,
    CalibrationMeasurement,
    CalibrationPlan,
    CalibrationSession,
    CalibrationStep,
    LEGACY_OSL_MAP,
    OSL_STEP_CHANNEL,
    OSL_STEP_NAMES,
    OSLTerms,
    P1_OSL_KEYS,
    P2_OSL_KEYS,
    REPEATS_DEFAULT,
    REPEATS_MAX,
    REPEATS_MIN,
    build_background_plan,
    build_osl_plan,
    build_reference_background,
    calibration_from_measurements,
    calibration_points_payload,
    osl_measurements_for_port,
    read_calibration_measurements,
    s11_osl_measurements,
    solve_one_port_osl,
    terms_to_osl_calibration,
    validate_repeats,
    write_calibration_file,
)

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
    BackendState,
    PersistentBackendSettings,
    PersistentLibreVnaBackend,
)
from librevna.librevna_errors import LibreVnaSweepError  # noqa: E402
from librevna.librevna_protocol import (  # noqa: E402
    S11_S22_STAGES_BITMAP,
    SweepSettings,
)
from librevna.simulated_transport import (  # noqa: E402
    SimulatedLibreVnaTransport,
)
from librevna.librevna_transport import LibreVnaTransportProtocol  # noqa: E402

#: 设备来源（空中端“设置”对话框）：USB 真机（默认）或 模拟器。模拟通道是
#: USB **旁路**（不替换）：在既有协议线上注入假 LibreVNA transport，worker/
#: 服务/流水线/地面站全部零适配；默认仍走真机（transport=None）。
DEVICE_SOURCE_USB = "usb"
DEVICE_SOURCE_SIMULATED = "simulated"
DEVICE_SOURCE_LABELS: dict[str, str] = {
    DEVICE_SOURCE_USB: "USB 真机 (LibreVNA)",
    DEVICE_SOURCE_SIMULATED: "模拟器（无硬件演示）",
}

#: 进程级默认设备来源（默认 USB）。被“未显式注入 backend/factory”的两处
#: 后端装配回退点（AcquisitionService._ensure_backend 与
#: ContinuousAcquisitionWorker.run 的独立 worker 自建路径）读取，使 UI 开关
#: 对两处装配点同时生效；MainWindow 的 UI 下拉提交时同步。默认 USB 时回退
#: 分支与第 2 轮逐字一致（既有测试不触碰该全局 → 行为零变化）。
_default_device_source = DEVICE_SOURCE_USB


def get_default_device_source() -> str:
    """当前进程级设备来源（USB 真机 / 模拟器）。"""
    return _default_device_source


def set_default_device_source(source: str) -> str:
    """设置进程级设备来源（供后端装配回退点读取）。非法值抛 ValueError。"""
    global _default_device_source
    if source not in DEVICE_SOURCE_LABELS:
        raise ValueError(f"未知设备来源: {source!r}")
    _default_device_source = source
    return source


class _LazyStageSink:
    """把 DiskOutbox 的阶段计时转发到**当时的** AIR_METRICS（第2轮 R-B-08）。

    DiskOutbox 在构造期持有 metrics 引用，而 AIR_METRICS 是运行期注入的全局；
    用这层薄适配器保证 outbox_write 耗时进入真实阶段统计（未注入则自动 no-op），
    同时让 disk_outbox.py 保持不 import 本模块（R-B-10 纯 Python）。
    """

    __slots__ = ()

    def record_stage(self, stage: str, duration_s: float) -> None:
        _record_stage(stage, duration_s)


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

#: 采集模式环境变量（"legacy" 或 "continuous"），缺省 / 非法值回退 continuous。
ACQUISITION_MODE_ENV = "UAV_GPR_ACQUISITION_MODE"
#: 出厂默认采集引擎：**高速连续采集**（常驻后端，USB 只开一次）。
#:
#: 变更依据（真机实测）：legacy worker 每帧经 ``acquire_sparameters`` 走
#: ``open_device`` → 采集 → 关闭释放（librevna_usb_s11.py:362…398），真机实测
#: 约 2.75 s/帧，且控制台每帧打印一次 ``Connected``；continuous 走常驻服务 +
#: 持久后端（USB 打开一次、configure 一次），因此默认必须是 continuous。
#:
#: legacy 引擎**代码保留**（调试退路），只是不再做默认：
#:   - 显式设 ``UAV_GPR_ACQUISITION_MODE=legacy``；或
#:   - 在主界面“采集引擎”下拉里选“兼容/调试（legacy，慢）”。
#: 非法值回退到默认 continuous（不再回退 legacy）。
DEFAULT_ACQUISITION_MODE = "continuous"


def acquisition_mode() -> str:
    """读取当前采集模式（环境变量 UAV_GPR_ACQUISITION_MODE，默认 continuous）。"""
    mode = os.environ.get(ACQUISITION_MODE_ENV, DEFAULT_ACQUISITION_MODE).strip().lower()
    return mode if mode in ("legacy", "continuous") else DEFAULT_ACQUISITION_MODE


def is_continuous_enabled() -> bool:
    return acquisition_mode() == "continuous"


#: 主界面“采集引擎”下拉的两个条目（文案 / 引擎标识一一对应，顺序 = 默认在选择框首位）。
ENGINE_MODE_CONTINUOUS = "continuous"
ENGINE_MODE_LEGACY = "legacy"
ENGINE_MODE_LABEL_CONTINUOUS = "高速连续采集（推荐，默认）"
ENGINE_MODE_LABEL_LEGACY = "兼容/调试 legacy（每帧开关 USB，慢）"

#: 状态区“连续模式”行两态文案（第4轮修复 t3）：文案必须与真实分支一致，
#: 且 continuous 态不得出现「未启用」歧义（默认引擎就是连续）。
#:
#: - ``_READY``：空闲/窗口刚起来时（引擎 = 连续，任务未运行）；
#: - ``_STANDBY``：一轮任务收尾后（USB 待命，下次启动即用同一高速路径）；
#: - ``_LEGACY``：显式选了 legacy 引擎时（写明为什么慢）。
CONTINUOUS_STATUS_READY = "连续模式: 高速连续采集（默认，未运行）"
CONTINUOUS_STATUS_STANDBY = "连续模式: 高速连续采集（待命，下次启动即用）"
CONTINUOUS_STATUS_LEGACY = "连续模式: 未启用（当前 legacy 引擎，每帧开关一次 USB，较慢）"
CONTINUOUS_STATUS_LEGACY_RUNTIME = (
    "连续模式: 未启用（本轮 legacy 引擎，每帧开关一次 USB，较慢）；"
    "下次要高速请把“采集引擎”切回“高速连续采集（推荐，默认）”"
)


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
    #: 第2轮常驻服务：任务序号（0 = 独立 worker 路径，不写入 metadata，
    #: 保持第 1 轮 acquisition 字段集合逐字不变）。
    task_seq: int = 0
    #: 第2轮 R-C-16：本条 trace 生效的会话校准身份（空串 = 未启用/独立 worker
    #: 路径，不写入 metadata，保持第 1 轮 acquisition 键集合逐字不变）。
    calibration_id: str = ""
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


# 第2轮 R-C-09：CalibrationMeasurement 与 CALIBRATION_STEPS/P1_OSL_KEYS/
# LEGACY_OSL_MAP 等校准数据层已搬家到 calibration_service.py（模块顶部
# re-export，``air.CalibrationMeasurement`` 等引用不变）。
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


# 第2轮 R-C-09：osl_measurements_for_port / s11_osl_measurements 已搬家到
# calibration_service.py（模块顶部 re-export）。
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


# 第2轮 R-C-09/R-C-10：calibration_from_measurements / calibration_points_payload /
# write_calibration_file / read_calibration_measurements 已搬家到 calibration_service.py
# （模块顶部 re-export，air.* 引用与既有测试逐字兼容）。


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
    arr = np.asarray(trace)
    if np.iscomplexobj(arr):
        arr = arr.real
    peak = float(np.max(np.abs(arr))) if arr.size else 0.0
    if peak <= 1e-15:
        return arr.copy()
    return arr / peak


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
    first_trace = validated_traces[channel_names[0]]
    npz_payload = {
        "config": json.dumps(asdict(cfg), ensure_ascii=False),
        "processing": json.dumps(processing_metadata(frequency_hz, cfg), ensure_ascii=False),
        "frequency_hz": frequency_hz,
        "time_ns": time_ns,
        "trace": first_trace.real.astype(np.float64, copy=False) if np.iscomplexobj(first_trace) else np.asarray(first_trace, dtype=float),
        "gnss": json.dumps(asdict(fix), ensure_ascii=False),
        "channels": json.dumps(list(channel_names), ensure_ascii=False),
    }
    if final_acquisition is not None:
        npz_payload["acquisition"] = json.dumps(final_acquisition, ensure_ascii=False)
    if flight is not None:
        npz_payload["flight"] = json.dumps(flight, ensure_ascii=False)
    for name in channel_names:
        raw_val = validated_traces[name]
        npz_payload[f"raw_{name}_real"] = raw_sparams[name].real
        npz_payload[f"raw_{name}_imag"] = raw_sparams[name].imag
        npz_payload[f"display_{name}_real"] = display_sparams[name].real
        npz_payload[f"display_{name}_imag"] = display_sparams[name].imag
        npz_payload[f"trace_{name}"] = raw_val.real.astype(np.float64, copy=False) if np.iscomplexobj(raw_val) else np.asarray(raw_val, dtype=float)
        npz_payload[f"trace_{name}_complex"] = np.asarray(raw_val, dtype=np.complex128)
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
    first_trace = validated_traces[channel_names[0]]
    payload: dict[str, object] = {
        "index": np.array(index, dtype=np.int64),
        "config": json.dumps(asdict(cfg), ensure_ascii=False),
        "processing": json.dumps(processing_metadata(frequency_hz, cfg), ensure_ascii=False),
        "frequency_hz": np.asarray(frequency_hz),
        "time_ns": np.asarray(time_ns),
        "trace": first_trace.real.astype(np.float64, copy=False) if np.iscomplexobj(first_trace) else np.asarray(first_trace, dtype=float),
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
        raw_val = validated_traces[name]
        payload[f"raw_{name}_real"] = raw_sparams[name].real
        payload[f"raw_{name}_imag"] = raw_sparams[name].imag
        payload[f"display_{name}_real"] = display_sparams[name].real
        payload[f"display_{name}_imag"] = display_sparams[name].imag
        payload[f"trace_{name}"] = raw_val.real.astype(np.float64, copy=False) if np.iscomplexobj(raw_val) else np.asarray(raw_val, dtype=float)
        payload[f"trace_{name}_complex"] = np.asarray(raw_val, dtype=np.complex128)
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

    第2轮 t5（R-B-01/02）：新增两个可选字段——``outbox_path`` 为该消息对应的
    磁盘 outbox 帧文件（None=未持久化/状态消息），``replay`` 标记该条来自
    重启恢复扫描（用于 outbox 重放计数）。两者都有默认值，历史构造点
    （含 tests/test_round1_third_fixes.py:473 直接构造）逐字兼容。
    """

    __slots__ = ("header", "payload", "enqueued_at_perf", "outbox_path", "replay")

    def __init__(
        self,
        header: dict[str, object],
        payload: bytes,
        enqueued_at_perf: float,
        outbox_path: Path | None = None,
        replay: bool = False,
    ) -> None:
        self.header = header
        self.payload = payload
        self.enqueued_at_perf = enqueued_at_perf
        self.outbox_path = outbox_path
        self.replay = replay

    def __iter__(self):
        # 兼容历史测试 ``header, payload = sender.queue.get_nowait()``。
        # 生产代码路径走 .header/.payload 显式字段，不依赖 __iter__。
        return iter((self.header, self.payload))


class RemoteSender:
    def __init__(
        self,
        host: str,
        port: int,
        run_id: str,
        status_callback,
        command_callback=None,
        outbox_dir: Path | None = None,
        *,
        window_size: int = 16,
        ack_timeout: float = 5.0,
    ) -> None:
        self.host = host
        self.port = port
        self.run_id = run_id
        self.status_callback = status_callback
        self.command_callback = command_callback
        # ---- 第5轮：滑动窗口流控（替换逐条停等）----
        self.window_size = max(1, int(window_size))
        self.ack_timeout = max(0.5, float(ack_timeout))
        # ---- 第2轮 t5：磁盘 outbox（R-B-01…R-B-10）----
        # outbox_dir=None ⇒ 完全禁用持久化，第1轮全部 sender 测试逐字兼容。
        #: 磁盘 outbox 实例（None=禁用）。构造本身不建目录（磁盘目录在首次
        #: append 时才创建），避免 MainWindow 默认输出路径下产生垃圾目录。
        self.outbox: DiskOutbox | None = (
            DiskOutbox(Path(outbox_dir), metrics=_LazyStageSink()) if outbox_dir is not None else None
        )
        #: 待写盘失败后需要补发的告警是否已发过（每 run 一次即可，不刷屏）。
        self._outbox_alerted_runs: set[str] = set()
        #: §4.5 legacy/第1轮兼容路径显式 skip 的条数（与 write_failures 严格分账：
        #: 前者是"本就不该受保护"，后者是"该保护却没保住"）。
        self.outbox_skipped = 0
        #: start() 同步扫描出的恢复条目（未启动时为空列表）。
        self._recovery_items: list[_QueuedRemoteFrame] = []
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
        # ---- 第5轮：滑动窗口在途跟踪与积压探针 ----
        self._inflight: dict[int, _QueuedRemoteFrame] = {}
        self._inflight_sent_at: dict[int, float] = {}
        self._inflight_lock = threading.Lock()
        #: 探针观测值：运行期间同时在途未确认 trace 达到的最大值
        self.max_inflight_seen = 0

    def inflight_count(self) -> int:
        """当前在途（已发送、等待 ack_trace）未确认 trace 数量。"""
        with self._inflight_lock:
            return len(self._inflight)

    @property
    def inflight(self) -> dict[int, _QueuedRemoteFrame]:
        """只读副本：当前在途 trace 字典（trace_index -> item）。"""
        with self._inflight_lock:
            return dict(self._inflight)

    def backlog_counts(self) -> dict[str, int]:
        """发送端运行对象实时计数：在途、队列中、未确认总数（来自真实运行对象非估算）。"""
        inflight = self.inflight_count()
        with self._outstanding_lock:
            outstanding = self._outstanding
        queued = max(0, outstanding - inflight)
        pending_box = self.outbox.pending_count() if self.outbox is not None else 0
        return {
            "inflight": inflight,
            "queued": queued,
            "outstanding": outstanding,
            "unacknowledged": outstanding,
            "outbox_pending": pending_box,
            "stopping_remaining": outstanding,
        }

    def backlog_text(self, state: str = "") -> str:
        """格式化积压文案（满足验收条目4）。"""
        counts = self.backlog_counts()
        inflight = counts["inflight"]
        outstanding = counts["outstanding"]
        if state in ("stopping", "paused", "draining") or (state.startswith("stop") and outstanding > 0):
            return f"排空中 · 还差 {outstanding} 条"
        if outstanding == 0:
            return "空闲/已停止"
        return f"在途 {inflight} / 未确认 {outstanding}"

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
        # 第2轮 §4.4：恢复扫描在**本线程启动前同步完成**。刻意不放 _run 开头：
        # 否则 start() 与扫描之间存在窗口，此间调用 stop(drain=True) 会看到
        # outstanding==0 而"排空成功"，把补传条目漏在保护之外（closeEvent
        # 的排空语义必须覆盖恢复条目，R-B-05）。
        self._recovery_items = self._recover_outbox_items()
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
        run_id: str | None = None,
    ) -> None:
        # remote_enqueued：payload 组装 + 成功入队的真实耗时（第二次定向
        # 修复：不再用固定 0.0 冒充耗时）。
        # 第2轮 §3.7-3：run_id=None 时取连接身份（第 1 轮行为逐字兼容）；显式
        # 传入时该消息的生产身份即为该值（常驻服务下"身份由消息携带"）。
        _enqueue_t0 = time.perf_counter()
        message_run_id = run_id if run_id is not None else self.run_id
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
            "run_id": message_run_id,
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
        # ---- 第2轮 R-B-01：先原子落盘，后入队（顺序刻意如此）----
        # 只有落盘成功的条目才受崩溃保护；落盘失败仍入队走实时链路，但必须
        # 显式告警计数（§4.3-2 "绝不伪装成功"）。
        outbox_path = self._outbox_persist(header, payload, explicit_run_id=run_id is not None)
        self._note_enqueued()
        _record_stage("remote_enqueued", time.perf_counter() - _enqueue_t0)
        self.queue.put(
            _QueuedRemoteFrame(header, payload, time.perf_counter(), outbox_path=outbox_path)
        )

    def _outbox_persist(
        self,
        header: dict[str, object],
        payload: bytes,
        *,
        explicit_run_id: bool,
    ) -> Path | None:
        """R-B-01/§4.5：把待发 trace 原子落盘，返回帧文件路径（None=未受保护）。

        判据用"**调用方是否显式携带任务身份**"，不是字符串格式：continuous
        worker/校准按 §3.7-3 必须显式传 ``run_id=``，而 legacy
        ``AcquisitionWorker.on_trace`` 走第1轮原路径不传（此时 header 里是
        *连接* 身份——可能是带内切换后残留的旧任务身份，落盘会与旧 run 同编号
        真数据撞车，复活 §3.7 死锁）。

        三种"不落盘"语义完全不同，**只有第 (3) 种计失败并发告警**：

        1. outbox 未启用（``outbox_dir=None``）⇒ 静默跳过，第1轮行为；
        2. ``explicit_run_id is False``（legacy/第1轮兼容语义，§4.5 明写
           "legacy 路径不接 outbox"）⇒ 契约要求的 skip：不写盘、不计
           ``write_failures``、不发 ``outbox_alert``（算了就是统计为一条本不
           该受保护的链路撒谎）；只在身份确为非任务式时计一次独立的
           ``outbox_skipped`` 供诊断，与失败严格分账；
        3. 真写盘失败（磁盘满/权限/IO/身份非法）⇒ 计 ``write_failures`` +
           每 run 一次 ``outbox_alert``，条目仍入队继续实时发送。
        """
        if self.outbox is None:
            return None
        message_run_id = header.get("run_id")
        if not explicit_run_id:
            self.outbox_skipped += 1
            return None  # (2) 契约 skip：legacy/第1轮兼容路径不接 outbox。
        if classify_run_id(message_run_id) != RUN_ID_CLASS_AIR_TASK:
            self.outbox_skipped += 1
            try:
                self.status_callback(
                    f"Outbox 跳过（非采集任务身份）: run={message_run_id!r} "
                    f"trace={header.get('trace_index')}"
                )
            except Exception:  # noqa: BLE001
                pass
            return None
        try:
            return self.outbox.append_message(header, payload)
        except OutboxFrameError as exc:
            # 帧/身份非法（DiskOutbox 计 rejections）与磁盘故障（计
            # write_failures）分账且措辞不得混用——日志与计数器互相打脸正是
            # 第1轮复审点名的"计数说谎"同类（t5 实测曾把拒绝报成写盘失败）。
            self._note_outbox_failure(
                message_run_id, header.get("trace_index"), exc, kind="rejected"
            )
            return None
        except OutboxWriteError as exc:
            self._note_outbox_failure(
                message_run_id, header.get("trace_index"), exc, kind="write_failed"
            )
            return None

    def _note_outbox_failure(
        self, run_id: object, trace_index: object, exc: Exception, *, kind: str
    ) -> None:
        """未受保护条目的可见化（§4.3-2 / R-B-06）：日志 + 每 run 每类一条告警。"""
        key = f"{run_id}:{kind}"
        label = "Outbox 拒绝落盘" if kind == "rejected" else "Outbox 写盘失败"
        try:
            self.status_callback(f"{label} (run={run_id} trace={trace_index}): {exc}")
        except Exception:  # noqa: BLE001 - 告警通道故障不得影响发送
            pass
        if key in self._outbox_alerted_runs:
            return  # 每 run 每类只补发一条状态消息，避免磁盘故障时刷屏。
        self._outbox_alerted_runs.add(key)
        try:
            self.enqueue_status(
                "outbox_alert",
                kind=kind,
                reason=str(exc),
                trace_index=trace_index,
                run_id=run_id if isinstance(run_id, str) else None,
            )
        except Exception as exc2:  # noqa: BLE001 - 告警本身失败绝不再递归
            try:
                self.status_callback(f"Outbox 告警消息入队失败: {exc2}")
            except Exception:  # noqa: BLE001
                pass

    def enqueue_status(
        self,
        state: str,
        *,
        reason: str | None = None,
        run_id: str | None = None,
        **fields: object,
    ) -> None:
        """发送 acquisition_status 状态消息（第1轮检查点D，向后兼容）。

        在既有 uav-gpr-telemetry-v1 framing 上新增 ``type="acquisition_status"``
        空载荷消息；旧接收端不认识时会走"忽略未知消息"分支。不改变 trace
        framing、SHA-256 或 ack_trace 语义。

        第2轮 §3.7-3：``run_id=None`` 取当前连接身份（第 1 轮行为不变）；显式
        传入时消息身份在生产时刻定型（常驻服务的任务身份制）。
        """
        header: dict[str, object] = {
            "type": "acquisition_status",
            "protocol": "uav-gpr-telemetry-v1",
            "run_id": run_id if run_id is not None else self.run_id,
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

    def enqueue_calibration(self, header: dict[str, object], payload: bytes = b"") -> None:
        """发送校准回程消息（§5.4：``type="calibration_*"``，走既有 framing）。

        与状态消息同规则（R-X-02 允许的空中→地面新增消息类型，旧地面端落入
        ``handle_client`` 的"忽略未知消息"分支）：入队即计 outstanding（保证
        关闭前排空、不静默丢弃）；``send_remote_frame`` 成功即视为完成（**不等**
        ``ack_trace``）。header 的 ``run_id`` 沿用**当前连接身份**（待机 seq=0
        或当前任务身份）——校准回程绝不触发带内身份切换、绝不触发地面
        ``begin_run``（§5.4：不得制造第二个身份切换源）。缺省字段在此补齐，
        调用方只给业务字段。
        """
        frame = dict(header)
        frame.setdefault("type", "calibration_status")
        frame.setdefault("protocol", "uav-gpr-telemetry-v1")
        frame.setdefault("run_id", self.run_id)
        frame.setdefault("timestamp", utc_timestamp())
        self._note_enqueued()
        self.queue.put(_QueuedRemoteFrame(frame, payload, time.perf_counter()))

    def _switch_connection_identity(self, new_run_id: str) -> None:
        """同一条连接上带内重播 hello（§3.7-4）。

        地面在等 ack 期间可能并发下发 command：等待循环必须像
        :meth:`_wait_for_trace_ack` 一样先分派 command 再继续等，否则命令帧会把
        切换踢成 ConnectionError（=§3.7 要消灭的死锁类缺陷）。未知 type 按既有
        先例 raise（不得静默吞）。
        """
        sock = self._sock
        if sock is None:
            raise ConnectionError("带内身份切换时 socket 未建立")
        send_remote_frame(
            sock,
            {
                "type": "hello",
                "protocol": "uav-gpr-telemetry-v1",
                "run_id": new_run_id,
                "timestamp": utc_timestamp(),
                "supports_commands": True,
            },
        )
        self.status_callback(f"Remote identity switch -> {new_run_id}")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                header, _payload = recv_remote_frame(sock)
            except socket.timeout:
                if self._stop.is_set():
                    raise ConnectionError("身份切换等待期间停止")
                continue
            msg_type = header.get("type")
            if msg_type == "ack":
                self.run_id = new_run_id
                return
            if msg_type == "command":
                self._handle_command(header)
                continue
            raise ConnectionError(f"unexpected ack for identity switch: {header}")
        raise ConnectionError(f"身份切换等待 ack 超时: {new_run_id}")

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

    def _ack_single_trace(
        self,
        trace_index: object,
        item: _QueuedRemoteFrame | None,
        header: dict[str, object],
    ) -> bool:
        """处理单条 trace 确认（划除 outbox，更新完成计数）。返回是否划除/确认成功。"""
        if item is not None and item.outbox_path is not None and self.outbox is not None:
            if item.replay:
                self.outbox.note_replayed(duplicate_acked=bool(header.get("duplicate")))
            try:
                removed = self.outbox.remove(item.outbox_path)
            except Exception:  # noqa: BLE001
                removed = False
            if not removed:
                try:
                    self.status_callback(
                        f"Outbox ACK 后划除失败（保持未完成，重启后重试）"
                        f" trace={trace_index}: {item.outbox_path.name}"
                    )
                except Exception:  # noqa: BLE001
                    pass
                return False  # fail-closed：持久化未划除不减 outstanding
        self._note_completed()
        return True

    def _wait_for_trace_ack(
        self,
        sock: socket.socket,
        trace_index: object,
        item: _QueuedRemoteFrame | None = None,
    ) -> None:
        """等待匹配 ack_trace（兼容保留：主循环已用滑动窗口，当前无调用点，
        仅为单条同步等待语义与既有测试兼容而保留）。"""
        deadline = time.monotonic() + self.ack_timeout
        while not self._stop.is_set():
            if time.monotonic() > deadline:
                raise TimeoutError(f"等待 ack_trace 超时 trace={trace_index}")
            header, _payload = self._read_frame(sock)
            msg_type = header.get("type")
            if msg_type == "ack_trace":
                ack_idx = header.get("trace_index")
                if ack_idx == trace_index:
                    self._ack_single_trace(trace_index, item, header)
                    return
                # 乱序 ACK：先处理该条
                with self._inflight_lock:
                    other_item = self._inflight.pop(ack_idx, None)
                    self._inflight_sent_at.pop(ack_idx, None)
                if other_item is not None:
                    self._ack_single_trace(ack_idx, other_item, header)
                continue
            if msg_type == "command":
                self._handle_command(header)
                continue
            raise ConnectionError(f"unexpected trace ack: {header}")

    def _drain_incoming_frames(self, sock: socket.socket, timeout: float = 0.0) -> None:
        """从 socket 批量读出已到达的 ack_trace 与 command。

        第5轮 F-1：窗口有空位且发送队列仍有待发工作时，以**零超时**轮询——
        已到达的 ack 批量消化（保住批量 pipelining），socket 暂无数据立即
        返回（不把剩余超时等满，window=1 形态不再有每条 +~100ms 尾延迟）。
        窗口仍满或无待发工作时保持原语义：按剩余超时等待 ack。
        """
        deadline = time.monotonic() + timeout if timeout > 0 else 0.0
        while not self._stop.is_set():
            with self._inflight_lock:
                has_space = len(self._inflight) < self.window_size
            has_work = (
                bool(self._recovery_items)
                or self._pending_item is not None
                or not self.queue.empty()
            )
            if timeout > 0:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return
                cur_timeout = 0.0 if (has_space and has_work) else remaining
            else:
                cur_timeout = 0.0
            try:
                r, _, _ = select.select([sock], [], [], cur_timeout)
            except (OSError, ValueError):
                raise ConnectionError("socket select failed")
            if not r:
                break
            header, _payload = recv_remote_frame(sock)
            msg_type = header.get("type")
            if msg_type == "ack_trace":
                ack_idx = header.get("trace_index")
                with self._inflight_lock:
                    item = self._inflight.pop(ack_idx, None)
                    sent_t0 = self._inflight_sent_at.pop(ack_idx, None)
                if sent_t0 is not None:
                    _record_stage("remote_ack_received", time.perf_counter() - sent_t0)
                if item is not None:
                    if self._ack_single_trace(ack_idx, item, header):
                        self.status_callback(f"Remote sent trace {ack_idx}")
                continue
            if msg_type == "command":
                self._handle_command(header)
                continue
            raise ConnectionError(f"unexpected remote frame: {header}")

    def _check_inflight_timeout(self) -> None:
        """检查在途未确认 trace 是否超过 ack_timeout。"""
        now = time.perf_counter()
        with self._inflight_lock:
            for idx, sent_t0 in list(self._inflight_sent_at.items()):
                if now - sent_t0 > self.ack_timeout:
                    raise TimeoutError(f"ack_trace 超时 ({self.ack_timeout}s): trace {idx}")

    def _recover_outbox_items(self) -> list[_QueuedRemoteFrame]:
        """§4.4：构造/start 时同步扫描磁盘 outbox，生成恢复条目列表。

        - 按 ``(run_id, trace_index)`` 排序（``%04d`` ⇒ 字典序==时间序）；
        - 每条**入列表即计入 outstanding**（与实时消息同一把锁、同一个计数器）
          ⇒ ``stop(drain=True)``/closeEvent 的真实排空语义自动覆盖补传
          （R-B-05，第1轮"不得伪装排空"语义不许被绕过）；
        - 复用 :class:`_QueuedRemoteFrame` 且标 ``replay=True``：它随后进入
          既有 ``_run`` 的 pending 状态机，**不新建连接、不另起线程**
          （地面 accept 串行、双连接互相饿死，§3.7）。
        """
        if self.outbox is None:
            return []
        recovered: list[_QueuedRemoteFrame] = []
        try:
            entries = self.outbox.scan_pending()
        except Exception as exc:  # noqa: BLE001 - 扫描故障必须可见，但不得杀死启动
            try:
                self.status_callback(f"Outbox 扫描失败（本次不补传）: {exc}")
            except Exception:  # noqa: BLE001
                pass
            return recovered
        for entry in entries:
            item = _QueuedRemoteFrame(
                entry.header,
                entry.payload,
                time.perf_counter(),
                outbox_path=entry.path,
                replay=True,
            )
            self._note_enqueued()  # 完成前必须计入 outstanding（§4.4 写死）。
            recovered.append(item)
        if recovered:
            try:
                self.status_callback(
                    "Outbox 恢复待补传 "
                    + str(len(recovered))
                    + " 条（run="
                    + ",".join(sorted({e.run_id for e in entries}))
                    + "）"
                )
            except Exception:  # noqa: BLE001
                pass
        return recovered

    def _run(self) -> None:
        pending: list[_QueuedRemoteFrame] = []
        self._pending_item = None
        # ---- 第2轮 §4.4：start() 已同步扫描出的恢复列表在本循环 pending 位串行
        # 消费：无独立恢复线程、无第二条 TCP 连接、无双份 outstanding 记账。
        recovery: list[_QueuedRemoteFrame] = self._recovery_items
        while not self._stop.is_set():
            try:
                if self._sock is None:
                    self._sock = self._connect()
                sock = self._sock

                # 1. 检查并读出已到达的 ACK 与 command
                with self._inflight_lock:
                    num_inflight = len(self._inflight)
                has_work = bool(pending or recovery or not self.queue.empty())

                if num_inflight >= self.window_size:
                    self._drain_incoming_frames(sock, timeout=0.1)
                elif num_inflight > 0 and not has_work:
                    self._drain_incoming_frames(sock, timeout=0.05)
                else:
                    self._drain_incoming_frames(sock, timeout=0.0)

                # 检查在途 trace 是否超时
                self._check_inflight_timeout()

                # 2. 窗口若满，不能发送新帧，继续等待 ACK
                with self._inflight_lock:
                    if len(self._inflight) >= self.window_size:
                        continue

                # 3. 获取下一条待发消息
                item = None
                if pending:
                    item = pending.pop(0)
                    self._pending_item = pending[0] if pending else None
                elif recovery:
                    item = recovery.pop(0)
                else:
                    with self._inflight_lock:
                        cur_inflight = len(self._inflight)
                    q_timeout = 0.1 if cur_inflight == 0 else 0.01
                    try:
                        item = self.queue.get(timeout=q_timeout)
                    except queue.Empty:
                        continue

                if item is None:
                    # 退出哨兵
                    break

                header, payload = item.header, item.payload
                message_run_id = header.get("run_id")

                # ---- 带内身份切换：跨 run_id 发送前先排空旧 run 的在途 trace ----
                if (
                    isinstance(message_run_id, str)
                    and message_run_id
                    and message_run_id != self.run_id
                ):
                    while not self._stop.is_set():
                        with self._inflight_lock:
                            if not self._inflight:
                                break
                        self._drain_incoming_frames(sock, timeout=0.1)
                        self._check_inflight_timeout()
                    self._switch_connection_identity(message_run_id)

                self._busy.set()
                try:
                    _send_t0 = time.perf_counter()
                    message_type = str(header.get("type") or "")
                    is_status = (
                        message_type == "acquisition_status"
                        or message_type.startswith("calibration_")
                    )
                    if not is_status:
                        _record_stage("remote_send_started", _send_t0 - item.enqueued_at_perf)
                    send_remote_frame(sock, header, payload)
                    if is_status:
                        self._note_completed()
                        if message_type.startswith("calibration_"):
                            self.status_callback(
                                f"Remote sent calibration: {header.get('cal_id')} {message_type}"
                            )
                        else:
                            self.status_callback(f"Remote sent status: {header.get('state')}")
                        continue
                    # Trace 帧：记录到在途字典，滑动窗口继续前行
                    trace_idx = header.get("trace_index")
                    with self._inflight_lock:
                        self._inflight[trace_idx] = item
                        self._inflight_sent_at[trace_idx] = _send_t0
                        curr_len = len(self._inflight)
                        if curr_len > self.max_inflight_seen:
                            self.max_inflight_seen = curr_len
                finally:
                    self._busy.clear()

            except queue.Empty:
                continue
            except Exception as exc:
                if self._stop.is_set():
                    return
                with self._inflight_lock:
                    unacked = list(self._inflight.values())
                    self._inflight.clear()
                    self._inflight_sent_at.clear()
                if "item" in locals() and item is not None and item.header.get("type") == "trace":
                    if item not in unacked:
                        unacked.append(item)
                pending = unacked + pending
                self._pending_item = pending[0] if pending else None
                self._close()
                self.status_callback(f"Remote send retrying: {exc}")
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
        run_id: str | None = None,
    ) -> None:
        super().__init__()
        self._queue = queue
        self._gnss_cache = gnss_cache
        self.cfg = cfg
        self.out_dir = out_dir
        self.autosave = autosave
        #: 第2轮任务身份制（§3.7）：本消费者产出 trace 携带的身份；
        #: None = 用连接身份（第 1 轮行为逐字兼容）。
        self.run_id = run_id
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
                run_id=self.run_id,
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
            # 第2轮常驻服务：task_seq 只进 acquisition 嵌套 metadata（不进
            # telemetry framing 顶层，R-X-02）；task_seq==0（独立 worker 路径）
            # 时不写该字段，保持第 1 轮 acquisition 键集合逐字不变。
            **(
                {"task_seq": int(item.task_seq)}
                if getattr(item, "task_seq", 0)
                else {}
            ),
            # 第2轮 R-C-16：calibration_id 同样只进 acquisition 嵌套 metadata；
            # 空串（未启用校准 / 独立 worker 路径）不写字段。
            **(
                {"calibration_id": str(item.calibration_id)}
                if getattr(item, "calibration_id", "")
                else {}
            ),
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


#: 第2轮：常驻采集服务的状态词汇（docs/round2_requirements_and_interfaces.md §3.1）。
SERVICE_STATE_CLOSED = "closed"
SERVICE_STATE_IDLE = "idle_waiting"
SERVICE_STATE_ACQUIRING = "acquiring"
SERVICE_STATE_STOPPING = "stopping"
SERVICE_STATE_SHUTTING_DOWN = "shutting_down"
SERVICE_STATE_FAULTED = "faulted"

#: run_id 后缀类型（§3.7 身份词汇表）。
RUN_ID_KIND_TASK = "task"
RUN_ID_KIND_IDLE = "idle"
RUN_ID_KIND_CALIBRATION = "calibration"


def _run_id_now(seq: int, kind: str) -> str:
    """签发 run_id：`YYYYmmdd_HHMMSS_%04d`（seq 四位零填充，§3.7-1 字典序==时间序）。"""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if kind == RUN_ID_KIND_IDLE:
        return f"{stamp}_idle"
    if kind == RUN_ID_KIND_CALIBRATION:
        return f"{stamp}_{seq:04d}_c"
    return f"{stamp}_{seq:04d}"


class CalibrationCaptureConsumer(QtCore.QThread):
    """校准采集任务的消费者（§5.4）：把选中 sweep 交给会话，绝不保存/发送。

    与 :class:`ProcessingConsumer` 保持相同的对外形状（信号、
    ``drain_state``/``drain_failures``/``processed_count``、``request_stop``），
    因此 :class:`ContinuousAcquisitionWorker` 的对账（processed==selected）、
    排空与 fail-closed 语义按第 1 轮契约原样工作（R-C-15：校准任务不落
    trace、不发遥测、不写 ``trace_*`` 文件）。Qt 依赖类只能落在
    ``uav_gpr_qt.py`` 接线区——``calibration_service.py`` 保持纯 Python。
    """

    trace_ready = QtCore.Signal(int, object, object, object, object, object, object, object, str)
    status = QtCore.Signal(str)
    failed = QtCore.Signal(str)
    fatal_error = QtCore.Signal(str)

    def __init__(
        self,
        *,
        queue: BoundedQueue,
        on_sweep: Callable[[SelectedSweep], None],
    ) -> None:
        super().__init__()
        self._queue = queue
        self._on_sweep = on_sweep
        self._stop = threading.Event()
        self.processed_count = 0
        self.drain_state: str = "running"
        self.drain_failures: list[str] = []

    def request_stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        try:
            while not self._stop.is_set():
                item = self._queue.get(timeout=0.05)
                if item is None:
                    continue
                self._process_one(item)
            # 与 ProcessingConsumer 相同的排空语义：停止后已选中的 sweep 全部
            # 交付（无固定时限），失败明确记录，绝不伪装 drained。
            self.drain_state = "draining"
            while True:
                item = self._queue.get(timeout=0.1)
                if item is None:
                    break
                self._process_one(item)
            self.drain_state = "drained" if not self.drain_failures else "drain_failed"
        except Exception as exc:  # noqa: BLE001 - 与 ProcessingConsumer 一致 fail-closed
            self.drain_state = "drain_failed"
            self.drain_failures.append(f"处理线程异常: {exc}")
            self.fatal_error.emit(f"校准消费线程异常: {exc}")

    def _process_one(self, item: SelectedSweep) -> None:
        try:
            self._on_sweep(item)
        except Exception as exc:  # noqa: BLE001 - 单条失败计入 drain_failures
            self.drain_failures.append(
                f"trace #{getattr(item, 'trace_index', '?')}: {exc}"
            )
            self.failed.emit(f"校准 sweep 交付失败: {exc}")
            return
        self.processed_count += 1


def _calibration_sweep_timeout_s(cfg: SweepConfig) -> float:
    """单条 sweep 的有界等待（对齐参考文档 §3.1 保守估算 + 10s 余量）。"""
    expected_s = cfg.points * 2 / max(float(cfg.ifbw_hz), 1.0)
    return max(10.0, expected_s * 5.0) + 10.0


class ServiceStepCapturer:
    """以服务短任务承载**一个校准步骤**的 repeats 条 sweep 采集（§5.4）。

    - 首次 :meth:`capture_one` 时经 ``AcquisitionTaskSpec(status_reporting=
      False)`` + **同一个** ``service.start_task`` 启动校准采集任务
      （closed→open/configure、idle_waiting→resume/reconfigure；服务照常持有
      backend）；``count=step.repeats`` 让 worker 取满 repeats 条后自行受控
      停止，编排侧再补一次 ``service.request_stop``（合同原文"取 repeats 条
      完整 sweep 后 request_stop()"）。
    - 经 worker 既有 ``consumer_factory`` 注入点挂
      :class:`CalibrationCaptureConsumer`，每条选中 sweep 进入内部队列，
      ``capture_one`` 逐条取走 —— 这就是 R-C-14 的"服务/后端抽象" capturer，
      **不是** ``acquire_sparameters``（后者自开自关必然撞 claim）。
    - 绝不保存 trace/绝不发送遥测（autosave=False、remote_sender=None）。
    """

    def __init__(
        self,
        *,
        service: AcquisitionService,
        cfg: SweepConfig,
        cal_id: str,
        run_id: str,
        step: CalibrationStep,
        out_dir: Path,
        sweep_timeout_s: float | None = None,
    ) -> None:
        self._service = service
        self._cfg = cfg
        self._cal_id = cal_id
        self._run_id = run_id
        self._step = step
        self._out_dir = out_dir
        self._sweep_timeout_s = float(sweep_timeout_s) if sweep_timeout_s is not None else _calibration_sweep_timeout_s(cfg)
        self._queue: queue.Queue[SelectedSweep] = queue.Queue()
        self._started = False
        self._closed = False
        self._start_error: str | None = None

    def _ensure_task(self) -> None:
        if self._started or self._closed:
            return
        spec = AcquisitionTaskSpec(
            cfg=self._cfg,
            interval_s=0.0,
            out_dir=self._out_dir,
            count=int(self._step.repeats),
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
            run_id=self._run_id,
            status_reporting=False,
        )

        def _consumer_factory(pq: BoundedQueue, _cache: GnssFixCache) -> CalibrationCaptureConsumer:
            return CalibrationCaptureConsumer(
                queue=pq,
                on_sweep=self._queue.put,
            )

        def _factory(task_spec: AcquisitionTaskSpec) -> ContinuousAcquisitionWorker:
            worker = ContinuousAcquisitionWorker(
                cfg=task_spec.cfg,
                interval_s=task_spec.interval_s,
                out_dir=task_spec.out_dir,
                count=task_spec.count,
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
                service=self._service,
                task_spec=task_spec,
                consumer_factory=_consumer_factory,
            )
            return worker

        worker = self._service.start_task(spec, _factory)
        if worker is None:
            raise RuntimeError(
                f"校准采集任务被服务拒绝（state={self._service.state}，"
                f"last_stop_reason={self._service.last_stop_reason}）"
            )
        self._started = True

    def capture_one(self, step: CalibrationStep, index: int) -> tuple[np.ndarray, np.ndarray]:
        """CalibrationSession capturer 协议：取下一条完整 sweep。"""
        self._ensure_task()
        try:
            item = self._queue.get(timeout=self._sweep_timeout_s)
        except queue.Empty as exc:
            raise RuntimeError(
                f"校准 sweep 等待超时（>{self._sweep_timeout_s:.1f}s，step={step.name} #{index}）"
            ) from exc
        return item.frequency_hz, item.raw

    def close(self) -> None:
        """步骤结束（或失败）后的有界收尾：请求受控停止并等待任务退出。"""
        if self._closed:
            return
        self._closed = True
        if not self._started:
            return
        try:
            self._service.request_stop(f"校准步骤 {self._step.name} 采集完成")
        except Exception:  # noqa: BLE001 - 收尾不得抛出
            pass
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            worker = self._service.active_worker
            if worker is None or not worker.isRunning():
                return
            time.sleep(0.05)


#: §5.4：地面校准命令 = 既有 ``type="command"`` + 新 action 值（严禁新增
#: 地面→空中消息 type）。``cal_status`` 供地面查询当前生效校准摘要（R-C-16）。
_CALIBRATION_COMMAND_ACTIONS = (
    "cal_begin",
    "cal_capture",
    "cal_finish",
    "cal_activate",
    "cal_cancel",
    "cal_status",
)


def safe_cal_dir_name(cal_id: object) -> str:
    """cal_id → 安全目录/文件名（与地面 ``safe_name`` 同规则）。"""
    text = str(cal_id or "cal")
    return "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in text)


def _calibration_capture_npz_bytes(frequency_hz: np.ndarray, raw: np.ndarray) -> bytes:
    """单条校准 capture 的 NPZ 载荷（key 沿用既有 ``<channel>_real/_imag`` 约定）。"""
    buffer = io.BytesIO()
    np.savez(
        buffer,
        frequency_hz=np.asarray(frequency_hz, dtype=np.float64),
        S11_real=np.asarray(raw[0].real, dtype=np.float64),
        S11_imag=np.asarray(raw[0].imag, dtype=np.float64),
        S22_real=np.asarray(raw[1].real, dtype=np.float64),
        S22_imag=np.asarray(raw[1].imag, dtype=np.float64),
    )
    return buffer.getvalue()


class _ServiceCalibrationCapturerProxy:
    """会话级 capturer：按步骤惰性创建 :class:`ServiceStepCapturer`。

    :class:`CalibrationSession` 只见 ``capture_one(step, index)`` 接口；
    本代理把"每步骤一个服务短任务"的编排细节封装在接线层，并在
    ``close_current(step)`` 时做有界收尾（受控停止 + 等待退出）。
    """

    def __init__(self, window: "MainWindow", cal_id: str) -> None:
        self._window = window
        self._cal_id = cal_id
        self._step_capturers: dict[str, ServiceStepCapturer] = {}

    def _capturer_for(self, step: CalibrationStep) -> ServiceStepCapturer:
        capturer = self._step_capturers.get(step.name)
        if capturer is None:
            cfg = self._window._calibration_config_from_snapshot(
                getattr(self._window._calibration_session, "config_snapshot", {}) or {}
                if self._window._calibration_session is not None
                else {}
            )
            capturer = ServiceStepCapturer(
                service=self._window.acquisition_service,
                cfg=cfg,
                cal_id=self._cal_id,
                run_id=self._window.acquisition_service.new_run_id(RUN_ID_KIND_CALIBRATION),
                step=step,
                out_dir=(
                    self._window._calibration_session.capture_dir
                    if self._window._calibration_session is not None and self._window._calibration_session.capture_dir is not None
                    else Path(self._window.output_dir.text()) / "_calibration" / safe_cal_dir_name(self._cal_id)
                ),
            )
            self._step_capturers[step.name] = capturer
        return capturer

    def capture_one(self, step: CalibrationStep, index: int) -> tuple[np.ndarray, np.ndarray]:
        return self._capturer_for(step).capture_one(step, index)

    def close_current(self, step_name: str) -> None:
        capturer = self._step_capturers.pop(step_name, None)
        if capturer is not None:
            capturer.close()


@dataclass(frozen=True)
class AcquisitionTaskSpec:
    """一次采集任务的完整参数快照（§3.2）。"""

    cfg: SweepConfig
    interval_s: float
    out_dir: Path
    count: int
    gnss_port: str
    baud: int
    calibration_path: str
    session_calibration: OSLCalibration | None
    session_calibration_p2: OSLCalibration | None
    through_measurement: CalibrationMeasurement | None
    background_frequency_hz: np.ndarray | None
    background: np.ndarray | None
    autosave: bool
    remote_sender: RemoteSender | None = None
    run_id: str = ""
    task_seq: int = 0
    status_reporting: bool = True
    processing_queue_maxsize: int = 512
    queue_drop_policy: str | None = None
    #: 第2轮 R-C-16：本任务生效的会话校准身份（空串 = 未启用）。由 MainWindow
    #: 在 cal_activate 之后填充；经 SelectedSweep 透传进每条 trace 的**嵌套**
    #: acquisition metadata（不进 telemetry framing 顶层，R-X-02）。
    calibration_id: str = ""


class AcquisitionService:
    """常驻采集服务：任务之间保持 USB 会话，停止只下发 SET_IDLE（第2轮 R-A 系列）。

    纯 Python + threading 原语（非 QObject），全部公开方法线程安全。状态机
    （§3.1）::

        closed ─start_task→ acquiring ─request_stop→ stopping ─排空完成→ idle_waiting
        idle_waiting ─start_task(配置同)→ resume → acquiring
        idle_waiting ─start_task(配置变)→ reconfigure → acquiring
        acquiring/idle_waiting ─shutdown(仅程序关闭)→ releasing → closed
        任一态设备/协议不可确定 → faulted →（fail-closed 释放）→ closed

    "受控停止"不再依赖 worker 私有事件——队列硬上限、操作员停止按钮、地面端
    stop 命令全部汇聚到 :meth:`request_stop`（同一事件、同一 reason 记录、
    同一状态迁移与状态消息序列）。
    """

    def __init__(
        self,
        *,
        backend_factory: Callable[[], PersistentLibreVnaBackend] | None = None,
        backend_settings: PersistentBackendSettings | None = None,
        status_callback: Callable[[str], None] | None = None,
    ) -> None:
        self._backend_factory = backend_factory
        self._backend_settings = backend_settings
        self._status_callback = status_callback
        self._lock = threading.RLock()
        self._state = SERVICE_STATE_CLOSED
        self._backend: PersistentLibreVnaBackend | None = None
        self._worker: ContinuousAcquisitionWorker | None = None
        #: 会话冻结的扫频配置字节（SweepSettings.encode()，配置变更判定依据）。
        self._frozen_config_bytes: bytes | None = None
        self._frozen_cfg: SweepConfig | None = None
        self._pending_stop_reason: str | None = None
        self._last_stop_reason: str | None = None
        #: 停止/待命边界的唤醒事件（worker 与 wait_idle 共用）。
        self._idle_reached = threading.Event()
        self._idle_reached.set()
        self._shutdown_called = False
        self._task_seq = 0
        self._release_failures = 0
        self._session_open_count = 0

    # ---- 只读观测 ----

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def backend(self) -> PersistentLibreVnaBackend | None:
        with self._lock:
            return self._backend

    @property
    def active_worker(self) -> ContinuousAcquisitionWorker | None:
        with self._lock:
            return self._worker

    @property
    def last_stop_reason(self) -> str | None:
        with self._lock:
            return self._last_stop_reason

    @property
    def holds_usb(self) -> bool:
        """服务是否持有 USB（R-A-13 手动互斥判据）。"""
        with self._lock:
            return self._state != SERVICE_STATE_CLOSED and self._backend is not None

    @property
    def task_seq(self) -> int:
        with self._lock:
            return self._task_seq

    def new_run_id(self, kind: str = RUN_ID_KIND_TASK) -> str:
        """身份签发唯一入口（§3.7-1；idle 不消耗 seq）。"""
        if kind not in (RUN_ID_KIND_TASK, RUN_ID_KIND_IDLE, RUN_ID_KIND_CALIBRATION):
            raise ValueError(f"未知 run_id kind: {kind!r}")
        with self._lock:
            if kind == RUN_ID_KIND_TASK:
                self._task_seq += 1
                seq = self._task_seq
            elif kind == RUN_ID_KIND_CALIBRATION:
                self._task_seq += 1
                seq = self._task_seq
            else:
                seq = 0
        return _run_id_now(seq, kind)

    def statistics(self) -> dict[str, object]:
        """JSON 可序列化服务快照（诊断/测试用；绝不抛出）。"""
        with self._lock:
            backend_stats: dict[str, int] = {}
            if self._backend is not None:
                try:
                    backend_stats = self._backend.statistics.to_dict()
                except Exception:  # noqa: BLE001
                    backend_stats = {}
            return {
                "state": self._state,
                "task_seq": self._task_seq,
                "holds_usb": self._state != SERVICE_STATE_CLOSED and self._backend is not None,
                "backend_open_count": backend_stats.get("open_count", 0),
                "backend_configure_count": backend_stats.get("configure_count", 0),
                "backend_trace_index": backend_stats.get("trace_index", 0),
                "frozen_config": self._frozen_cfg is not None,
                "pending_stop_reason": self._pending_stop_reason,
                "last_stop_reason": self._last_stop_reason,
                "release_failures": self._release_failures,
                "worker_running": bool(self._worker is not None and self._worker.isRunning()),
            }

    def _notify(self, text: str) -> None:
        callback = self._status_callback
        if callback is None:
            return
        try:
            callback(text)
        except Exception:  # noqa: BLE001 - 诊断不得影响服务
            pass

    # ---- 会话生命周期（由 worker 线程调用/服务命令入口调用）----

    def _settings_of(self, cfg: SweepConfig) -> SweepSettings:
        return SweepSettings(
            start_hz=int(cfg.start_mhz * 1e6),
            stop_hz=int(cfg.stop_mhz * 1e6),
            points=cfg.points,
            ifbw_hz=cfg.ifbw_hz,
            power_dbm=cfg.power_dbm,
            dwell_us=cfg.dwell_us,
            stages_bitmap=S11_S22_STAGES_BITMAP,
        )

    def _ensure_backend(self) -> PersistentLibreVnaBackend:
        """返回常驻 backend；不存在/已释放时新建（调用方须持锁）。

        设备来源装配点①：无显式 backend_factory 时按进程级设备来源回退
        （默认 USB 与第 2 轮逐字一致；模拟时注入假 LibreVNA transport）。
        """
        backend = self._backend
        if backend is None or not backend.is_open:
            if self._backend_factory is not None:
                backend = self._backend_factory()
            elif get_default_device_source() == DEVICE_SOURCE_SIMULATED:
                backend = PersistentLibreVnaBackend(
                    transport=SimulatedLibreVnaTransport(),
                    settings=self._backend_settings,
                )
            else:
                backend = PersistentLibreVnaBackend(settings=self._backend_settings)
            self._backend = backend
            self._frozen_config_bytes = None
            self._frozen_cfg = None
        return backend

    def open_and_start(
        self, cfg: SweepConfig
    ) -> tuple[PersistentLibreVnaBackend, str]:
        """worker 线程启动会话入口。

        返回 ``(backend, path)``，path ∈ {"opened_configured", "resumed",
        "reconfigured", "reopened"}：
        - closed/无会话：open + configure（计 1 次 open、1 次 configure）
        - 待命 + 配置字节相同：resume（不 reopen、不重 configure）
        - 待命 + 配置字节不同：reconfigure（configure_count+1，open_count 不变）
        """
        settings = self._settings_of(cfg)
        with self._lock:
            if self._state in (SERVICE_STATE_SHUTTING_DOWN, SERVICE_STATE_STOPPING):
                raise RuntimeError(f"服务状态 {self._state} 不允许启动新任务")
            backend = self._ensure_backend()
            previous = self._state
            if previous == SERVICE_STATE_CLOSED or self._frozen_config_bytes is None:
                backend.open()
                backend.configure(settings)
                self._frozen_config_bytes = settings.encode()
                self._frozen_cfg = cfg
                self._state = SERVICE_STATE_ACQUIRING
                self._idle_reached.clear()
                self._session_open_count = backend.statistics.open_count
                return backend, "opened_configured"
            if backend.state is BackendState.FAULTED:
                # 故障会话只允许释放后重开（R-A-09）。
                self._release_backend_locked(backend)
                backend = self._ensure_backend()
                backend.open()
                backend.configure(settings)
                self._frozen_config_bytes = settings.encode()
                self._frozen_cfg = cfg
                self._state = SERVICE_STATE_ACQUIRING
                self._idle_reached.clear()
                return backend, "reopened"
            if self._frozen_config_bytes == settings.encode():
                if backend.state is BackendState.PAUSED:
                    backend.resume()
                elif backend.state is BackendState.OPEN_IDLE:
                    backend.configure(settings)
                self._state = SERVICE_STATE_ACQUIRING
                self._idle_reached.clear()
                return backend, "resumed"
            if backend.state is BackendState.STREAMING:
                backend.pause()
            backend.reconfigure(settings)
            self._frozen_config_bytes = settings.encode()
            self._frozen_cfg = cfg
            self._state = SERVICE_STATE_ACQUIRING
            self._idle_reached.clear()
            return backend, "reconfigured"

    def pause_stream(
        self, backend: PersistentLibreVnaBackend, *, block: bool = False
    ) -> str:
        """停止采集后立刻下发 SET_IDLE（R-A-03）：只停流，**不**释放 USB。

        返回 paused / idle / faulted_released / released / stale / shutting_down。
        待命态（IDLE_WAITING）由 :meth:`finish_task` 在消费者排空完成后才登记
        （R-A-06：pause 成功 + 排空完成 才算进入待命）。
        """
        fault_reason: str | None = None
        with self._lock:
            if self._state == SERVICE_STATE_SHUTTING_DOWN:
                return "shutting_down"
            if backend is not self._backend:
                return "stale"
            if self._shutdown_called:
                return "released"
            try:
                if backend.state is BackendState.STREAMING:
                    backend.pause()
                elif backend.state is BackendState.OPEN_IDLE:
                    pass  # 从未 configure：已经空闲，保持打开。
                # PAUSED：已在待命态，无需再 pause（pause 只接受 STREAMING）。
                if backend.state is BackendState.FAULTED:
                    fault_reason = "后端处于 FAULTED"
                elif backend.state is BackendState.OPEN_IDLE:
                    # 从未 configure：本就没有扫频流可停，措辞不得声称"已下发
                    # SET_IDLE"（第2轮 t4 评审 LOW 观察：纯文案不准）。
                    self._notify("后端从未 configure，无扫频流需停止（USB 保持连接）")
                    return "paused"
                else:
                    self._notify("已下发 SET_IDLE 停止扫频流（USB 保持连接，等待排空）")
                    return "paused"
            except Exception as exc:  # noqa: BLE001 - 停流失败按故障处理
                self._notify(f"下发 SET_IDLE 失败（按故障处理）: {exc}")
                fault_reason = f"待命失败: {exc}"
        # 故障释放在锁**外**执行：block 模式的重试睡眠绝不持锁
        # （否则 UI 线程的 statistics()/request_* 会被冻结）。
        return self._fault_and_release_locked(backend, fault_reason or "未知故障", block=block)

    def finish_task(
        self, worker: ContinuousAcquisitionWorker, *, terminal_state: str
    ) -> str:
        """worker 排空/收尾完成后的登记入口：登记待命或故障释放（R-A-06/R-A-09）。

        ``terminal_state``：drained / failed / unknown。只有 drained 才进入
        IDLE_WAITING（USB 仍持有）；failed 走 fail-closed 释放（R-A-09）。
        """
        fault_backend: PersistentLibreVnaBackend | None = None
        fault_reason = ""
        with self._lock:
            if self._worker is worker:
                self._worker = None
            backend = self._backend
            if self._state == SERVICE_STATE_SHUTTING_DOWN:
                self._idle_reached.set()
                return "shutting_down"
            if self._state in (SERVICE_STATE_FAULTED, SERVICE_STATE_CLOSED):
                self._idle_reached.set()
                return self._state
            if terminal_state != "drained":
                reason = self._pending_stop_reason or "任务未干净排空"
                self._pending_stop_reason = None
                if backend is not None and backend.is_open:
                    # R-A-09：失败任务的会话必须释放（锁外阻塞重试到真正释放）。
                    fault_backend, fault_reason = backend, f"任务失败释放 USB: {reason}"
                else:
                    self._state = SERVICE_STATE_CLOSED
                    self._last_stop_reason = reason
                    self._idle_reached.set()
                    return SERVICE_STATE_CLOSED
            elif backend is None or not backend.is_open:
                # 会话已不在（独立 worker 路径/测试直接释放）：回到 CLOSED。
                self._state = SERVICE_STATE_CLOSED
                self._last_stop_reason = self._pending_stop_reason
                self._pending_stop_reason = None
                self._idle_reached.set()
                return SERVICE_STATE_CLOSED
            elif backend.state is BackendState.FAULTED:
                fault_backend, fault_reason = backend, "后端处于 FAULTED"
            else:
                self._state = SERVICE_STATE_IDLE
                self._last_stop_reason = self._pending_stop_reason
                self._pending_stop_reason = None
                self._idle_reached.set()
                self._notify("任务已排空，进入 USB 待命（SET_IDLE 保持连接，不释放设备）")
                return SERVICE_STATE_IDLE
        # 锁外执行 fail-closed 释放（阻塞重试不持锁 → 不冻结 UI/命令入口）。
        if fault_backend is not None:
            return self._fault_and_release_locked(fault_backend, fault_reason, block=True)
        return self.state

    def note_backend_faulted(self, backend: PersistentLibreVnaBackend, reason: str) -> None:
        """worker 侧检测到 backend FAULTED/释放失败时的登记入口。"""
        with self._lock:
            if backend is not self._backend:
                return
            self._fault_and_release_locked(backend, reason)

    def _fault_and_release_locked(
        self, backend: PersistentLibreVnaBackend, reason: str, *, block: bool = False
    ) -> str:
        """故障处置：标记 FAULTED 并 fail-closed 释放（R-A-09）。

        `block=True`（worker 线程调用）时在本线程重试直到资源真正释放——保持第 1
        轮"资源未释放前不得结束 worker / 不发 finished_run"语义；睡眠发生在锁**外**，
        绝不因释放重试卡死 UI 线程或命令入口（UI 线程用默认 block=False）。
        """
        with self._lock:
            self._state = SERVICE_STATE_FAULTED
            self._last_stop_reason = reason
            self._pending_stop_reason = None
            self._notify(f"采集服务故障，fail-closed 释放 USB: {reason}")
            released = self._release_backend_locked(backend)
            self._idle_reached.set()
        if released:
            with self._lock:
                self._state = SERVICE_STATE_CLOSED
            return "faulted_released"
        if not block:
            return "faulted"
        last_status = time.monotonic()
        while True:
            time.sleep(1.0)  # 锁外等待
            with self._lock:
                released = self._release_backend_locked(backend)
            if released:
                with self._lock:
                    self._state = SERVICE_STATE_CLOSED
                self._notify("故障会话最终已释放，服务回到 CLOSED")
                return "faulted_released"
            now = time.monotonic()
            if now - last_status >= 5.0:
                last_status = now
                self._notify("仍在等待释放 LibreVNA（资源仍被持有，将自动重试）...")

    def _release_backend_locked(self, backend: PersistentLibreVnaBackend) -> bool:
        """fail-closed 释放（不阻塞等待循环：单次尝试 + 状态如实在册）。

        返回是否真正释放（is_open False）。调用方持锁。
        """
        try:
            backend.close()
        except Exception as exc:  # noqa: BLE001 - 释放失败必须可见
            self._release_failures += 1
            if backend.is_open:
                self._notify(f"释放 LibreVNA 失败（资源仍被持有，待重试）: {exc}")
                return False
            self._notify(f"释放 LibreVNA 抛错但资源已释放: {exc}")
        if backend.is_open:
            self._release_failures += 1
            self._notify("释放 LibreVNA 失败: close() 已返回但仍报告打开（待重试）")
            return False
        self._frozen_config_bytes = None
        self._frozen_cfg = None
        return True

    def release_idle_session(self) -> bool:
        """释放**待命**会话并回到 CLOSED（“设备来源”切换专用）。

        传输层无法在已打开的后端上热更换，切换设备来源必须先释放待命会话，
        让下一次任务按当前设备来源新建后端：

        - 仅接受 IDLE_WAITING（无任务运行）；其他状态返回 False；
        - 释放成功 → 状态回 CLOSED、backend 清空；
        - 释放失败保持 IDLE（资源仍被持有），返回 False、不伪装成功。
        - 不触碰 ``_shutdown_called``——与程序关闭的 :meth:`shutdown` 区分，
          切换后服务仍可正常启动新任务。
        """
        with self._lock:
            if self._state != SERVICE_STATE_IDLE:
                return False
            backend = self._backend
            if backend is None or not backend.is_open:
                self._backend = None
                self._frozen_config_bytes = None
                self._frozen_cfg = None
                self._state = SERVICE_STATE_CLOSED
                self._idle_reached.set()
                self._notify("待命会话已释放，服务回到 CLOSED")
                return True
            released = self._release_backend_locked(backend)
            if not released:
                self._notify(
                    "设备来源切换：待命会话释放未成功，服务保持 IDLE_WAITING（可重试）"
                )
                return False
            self._backend = None
            self._state = SERVICE_STATE_CLOSED
            self._idle_reached.set()
            self._notify("待命会话已释放，服务回到 CLOSED（下次任务按当前设备来源新建）")
            return True

    def retry_release(self) -> bool:
        """重试释放残留 backend（测试/shutdown 循环用）。"""
        with self._lock:
            backend = self._backend
            if backend is None or not backend.is_open:
                self._backend = None
                if self._state in (SERVICE_STATE_FAULTED, SERVICE_STATE_CLOSED):
                    self._state = SERVICE_STATE_CLOSED
                return True
            ok = self._release_backend_locked(backend)
            if ok:
                self._backend = None
                self._state = SERVICE_STATE_CLOSED
                self._notify("残留 USB 已释放，服务回到 CLOSED")
            return ok

    # ---- 统一命令入口 ----

    @staticmethod
    def _command(worker, primary: str, fallback: str | None = "stop") -> None:
        """对 worker 下命令（优先 request_*，退回 stop）。

        生产 worker 三个方法都有；测试注入的类替换品（unittest.mock.patch 整个
        ContinuousAcquisitionWorker 类）可能只实现 stop() —— 绝不因此抛
        AttributeError 让命令入口静默失效。
        """
        method = getattr(worker, primary, None)
        if callable(method):
            method()
            return
        if fallback:
            method = getattr(worker, fallback, None)
            if callable(method):
                method()

    def request_stop(self, reason: str | None = None) -> bool:
        """停止当前任务（操作员按钮 / 地面命令 / 队列硬上限受控停止共用入口）。

        返回 False 仅表示"当前无可停任务或状态不接受"；停止是否**完成**必须
        通过 :attr:`state` / :meth:`wait_idle` / acquisition_status 消息观测，
        绝不用返回值伪装成功。
        """
        with self._lock:
            worker = self._worker
            if (
                self._state not in (SERVICE_STATE_ACQUIRING, SERVICE_STATE_STOPPING)
                or worker is None
            ):
                if self._state == SERVICE_STATE_IDLE:
                    self._notify(f"request_stop({reason}): 已处于待命，无需停止")
                return False
            self._pending_stop_reason = reason or "操作员/外部停止命令"
            self._state = SERVICE_STATE_STOPPING
        self._command(worker, "request_stop")
        return True

    def request_pause(self) -> bool:
        with self._lock:
            worker = self._worker
            if self._state != SERVICE_STATE_ACQUIRING or worker is None:
                return False
        self._command(worker, "request_pause", fallback=None)
        return True

    def request_resume(self) -> bool:
        with self._lock:
            worker = self._worker
            if self._state != SERVICE_STATE_ACQUIRING or worker is None:
                return False
        worker.request_resume()
        return True

    def wait_idle(self, timeout_ms: int = 5000) -> bool:
        """有界等待进入**待命**（IDLE_WAITING）。

        只有真正 `idle_waiting` 才算成功；`closed`（故障释放/已关闭）**不是**
        待命，返回 False——绝不把"资源已释放"伪装成"已进入待命"（R-A-06）。
        """
        deadline = time.monotonic() + max(0, timeout_ms) / 1000.0
        while True:
            with self._lock:
                state = self._state
            if state == SERVICE_STATE_IDLE:
                return True
            if state in (SERVICE_STATE_FAULTED, SERVICE_STATE_CLOSED):
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._idle_reached.wait(timeout=min(0.05, remaining))

    def wait_worker_exit(self, timeout_ms: int = 5000) -> bool:
        deadline = time.monotonic() + max(0, timeout_ms) / 1000.0
        while True:
            with self._lock:
                worker = self._worker
            if worker is None or not worker.isRunning():
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            time.sleep(min(0.05, remaining))

    def start_task(
        self,
        spec: AcquisitionTaskSpec,
        worker_factory: Callable[[AcquisitionTaskSpec], ContinuousAcquisitionWorker],
    ) -> ContinuousAcquisitionWorker | None:
        """统一启动入口：签发身份 → 工厂建 worker → 登记 → 启动。

        UI 侧以工厂注入具体 worker 构造（测试可 patch 类）；服务负责签发
        run_id/task_seq 并迁移到 acquiring。校验类失败返回 None 并记录原因；
        状态不允许（正在关闭/已有任务）返回 None 且**不改状态**（不伪装启动）。
        """
        with self._lock:
            if self._shutdown_called or self._state == SERVICE_STATE_SHUTTING_DOWN:
                self._notify("start_task 被拒绝: 服务正在关闭")
                return None
            if self._worker is not None and self._worker.isRunning():
                self._notify("start_task 被拒绝: 采集任务已在运行")
                return None
            if self._state in (SERVICE_STATE_ACQUIRING, SERVICE_STATE_STOPPING):
                self._notify(f"start_task 被拒绝: 服务状态 {self._state}")
                return None
            if spec.cfg.stop_mhz <= spec.cfg.start_mhz:
                self._notify("start_task 被拒绝: Stop MHz 必须大于 Start MHz")
                return None
            run_id = spec.run_id or self.new_run_id(RUN_ID_KIND_TASK)
            task_spec = replace(spec, run_id=run_id, task_seq=self._task_seq)
            previous_state = self._state
            self._state = SERVICE_STATE_ACQUIRING
            self._idle_reached.clear()
        try:
            worker = worker_factory(task_spec)
        except Exception as exc:  # noqa: BLE001 - 启动失败必须如实回退状态
            with self._lock:
                self._worker = None
                self._state = previous_state
                self._notify(f"start_task 失败: {exc}")
            raise
        with self._lock:
            self._worker = worker
        worker.start()
        return worker

    def attach_task(self, worker: ContinuousAcquisitionWorker) -> None:
        """登记已在别处构造的服务 worker（不走 worker_factory 的 UI 路径用）。"""
        with self._lock:
            self._worker = worker
            if self._state == SERVICE_STATE_CLOSED:
                self._state = SERVICE_STATE_ACQUIRING
                self._idle_reached.clear()

    def note_worker_finished(self, worker: ContinuousAcquisitionWorker) -> None:
        """UI 收到 finished_run 时的兜底登记（真实 worker 已由 finish_task 处理）。

        真实服务模式 worker 的 finally 会先调 finish_task 并清掉本服务引用，
        这里就变成 no-op；而测试注入的假 worker（不执行 run()）通过本方法把
        服务从 acquiring 收回，避免状态永久悬挂（不伪装：只在确实没有会话时
        回到 idle/closed）。
        """
        with self._lock:
            if self._worker is not worker:
                return
            self._worker = None
            if self._state in (SERVICE_STATE_ACQUIRING, SERVICE_STATE_STOPPING):
                backend_open = self._backend is not None and self._backend.is_open
                self._state = SERVICE_STATE_IDLE if backend_open else SERVICE_STATE_CLOSED
                self._idle_reached.set()

    def shutdown(self, timeout_ms: int = 10_000) -> bool:
        """程序关闭专用：停止任务 → 有界等待退出 → 真正释放 USB → CLOSED。

        返回 False 表示有界时间内未能干净收尾（资源仍被持有/线程未退出）；
        此时保持状态可观测并可重试，绝不 terminate、绝不伪装成功。
        """
        with self._lock:
            if self._state == SERVICE_STATE_CLOSED and self._backend is None:
                self._shutdown_called = True
                return True
            self._shutdown_called = True
            self._state = SERVICE_STATE_SHUTTING_DOWN
            worker = self._worker
        self._idle_reached.clear()
        if worker is not None:
            self._pending_stop_reason = "程序关闭"
            # 真实 worker 用 request_stop；测试注入的替身可能只实现 stop()
            # （duck-typing，绝不 terminate/kill，R-X-04）。
            request_stop = getattr(worker, "request_stop", None)
            if callable(request_stop):
                request_stop()
            else:
                stop = getattr(worker, "stop", None)
                if callable(stop):
                    stop()
        if worker is not None and not self.wait_worker_exit(timeout_ms):
            self._notify("采集线程未在时限内退出，关闭保持可重试（不 terminate）")
            return False
        backend = self._backend
        if backend is None or not backend.is_open:
            with self._lock:
                self._backend = None
                self._worker = None
                self._frozen_config_bytes = None
                self._frozen_cfg = None
                self._state = SERVICE_STATE_CLOSED
            self._idle_reached.set()
            return True
        # 有界重试释放（每 1s 一次，共享同一 timeout_ms 预算）。
        deadline = time.monotonic() + max(0, timeout_ms) / 1000.0
        while True:
            with self._lock:
                released = self._release_backend_locked(backend)
            if released:
                break
            if time.monotonic() >= deadline:
                self._notify("USB 资源仍被持有，关闭未完成（不 terminate，可重试）")
                return False
            time.sleep(0.2)
        with self._lock:
            self._backend = None
            self._worker = None
            self._state = SERVICE_STATE_CLOSED
        self._idle_reached.set()
        self._notify("采集服务已关闭，USB 已释放")
        return True


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
        service: AcquisitionService | None = None,
        task_spec: AcquisitionTaskSpec | None = None,
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
        #: 第2轮常驻服务：service=None 时保持第 1 轮独立 worker 行为（自 open、
        #: 自 close、run_id 取 sender 身份）；注入 service 时会话由服务持有、
        #: 任务停止只 SET_IDLE 待命，身份由 spec 下发（§3.1/§3.7）。
        self._service = service
        self._task_spec = task_spec
        self._task_seq = int(getattr(task_spec, "task_seq", 0) or 0)
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

        第2轮 R-A-05：受控停止已升级为**服务命令**——注入 service 时走
        ``service.request_stop(reason)``（与操作员按钮、地面端 stop 完全同一
        入口、同一 reason 记录、同一状态迁移），worker 私有事件只是服务停止
        命令落到本任务线程的唤醒别名。
        """
        reason = f"处理队列达到硬上限 depth={depth}（受控停止新采集，已有数据将全部排空）"
        self._controlled_stop_reason = reason
        self.status.emit(reason)
        service = self._service
        if service is not None:
            service.request_stop(reason)
        else:
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
            if backend is None and self._service is None:
                # 设备来源装配点②（独立 worker 自建后端）：无显式 backend 时
                # 按进程级设备来源回退（默认 USB 与第 2 轮逐字一致）。
                if get_default_device_source() == DEVICE_SOURCE_SIMULATED:
                    backend = PersistentLibreVnaBackend(
                        transport=SimulatedLibreVnaTransport(),
                        settings=self._backend_settings,
                    )
                else:
                    backend = PersistentLibreVnaBackend(settings=self._backend_settings)
                self._backend = backend
            elif backend is None:
                # 服务模式：会话对象由 AcquisitionService 持有（open_and_start
                # 返回时写入 self._backend）；此处不得自建第二个 backend。
                pass
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
            if self._service is not None:
                # 第2轮常驻服务：会话由服务持有。closed→open+configure；
                # idle_waiting 且配置字节相同→resume（冻结配置，不重新 open）；
                # 配置字节不同→reconfigure（configure_count+1、open_count 不变）。
                phase = "启动常驻采集会话"
                backend, session_path = self._service.open_and_start(self.cfg)
                self._backend = backend
                self.status.emit(
                    {
                        "opened_configured": "常驻服务：新会话已 open/configure",
                        "resumed": "常驻服务：复用 USB 会话，resume 恢复扫频",
                        "reconfigured": "常驻服务：配置变更，reconfigure 重发 SWEEP_SETTINGS",
                        "reopened": "常驻服务：上一会话故障，fail-closed 释放后重新 open",
                    }.get(session_path, f"常驻服务：会话启动路径 {session_path}")
                )
                if self._stop.is_set():
                    return
                self._emit_acquiring_status()
            else:
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
                    # 第2轮常驻服务：任务序号（0=独立 worker，不写 metadata）。
                    task_seq=self._task_seq,
                    # 第2轮 R-C-16：校准身份透传（空串不写 metadata 字段）。
                    calibration_id=self._calibration_id(),
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
            # ---- 第2轮常驻服务：停止边界 SET_IDLE 停流（R-A-03/R-A-06）----
            # 生产循环退出（完整 sweep 边界）后、消费者排空**之前**下发
            # SET_IDLE：设备立即停流但 USB 保持 claim；随后由 finally 尾部的
            # finish_task 在排空完成后才登记 IDLE_WAITING / 或按 R-A-09
            # fail-closed 释放。
            service = self._service_session()
            if service is not None and self._backend is not None:
                # worker 线程内：故障释放必须在本线程重试到真正释放（R-A-09 与
                # 第 1 轮"资源未释放前不得结束 worker/不发 finished_run"一致）。
                service.pause_stream(self._backend, block=True)
            # 第1轮检查点D：先向地面端报告"正在停止（排空中）"；最终 stopped /
            # drain_failed 只有在队列归零/排空完成后才报告。
            if self.remote_sender is not None and self._reports_status():
                try:
                    queue_stats = self._processing_queue.statistics()
                except Exception:  # noqa: BLE001
                    queue_stats = {}
                self.remote_sender.enqueue_status(
                    "stopping",
                    processing_queue_depth=queue_stats.get("depth", 0),
                    selected_count=self._selected_count,
                    reason=self._controlled_stop_reason,
                    run_id=self._message_run_id(),
                    remaining=(
                        self.remote_sender.outstanding_count()
                        if hasattr(self.remote_sender, "outstanding_count")
                        else 0
                    ),
                    inflight=(
                        self.remote_sender.inflight_count()
                        if hasattr(self.remote_sender, "inflight_count")
                        else 0
                    ),
                )
            self._shutdown(
                consumer,
                gnss_thread,
                backend,
                keep_backend_open=(service is not None),
            )
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
            if self.remote_sender is not None and self._reports_status():
                if terminal_failed:
                    self.remote_sender.enqueue_status(
                        "drain_failed",
                        drain_state="failed" if queue_failed_item is not None or self._terminal_error else "drain_failed",
                        processing_queue_depth=len(drain_failures),
                        selected_count=self._selected_count,
                        processed_count=processed_count,
                        reason=self._controlled_stop_reason or ("；".join(failure_details) or None),
                        failure_details=failure_details or None,
                        run_id=self._message_run_id(),
                    )
                else:
                    self.remote_sender.enqueue_status(
                        "stopped",
                        drain_state=drain_state,
                        processing_queue_depth=0,
                        selected_count=self._selected_count,
                        processed_count=processed_count,
                        reason=self._controlled_stop_reason,
                        run_id=self._message_run_id(),
                    )
            # ---- 第2轮：向服务登记终态（排空完成才决定 待命 / 故障释放）----
            # drained → IDLE_WAITING（USB 保持 claim，R-A-01）；
            # failed/unknown → R-A-09 fail-closed 释放（绝不带着故障待命）。
            # 即使本任务从未拿到服务会话（start 前即 stop），也必须登记，
            # 否则服务会停留在 acquiring。
            if self._service is not None:
                self._service.finish_task(self, terminal_state=self.terminal_state)
            self.finished_run.emit()

    # ---- 第2轮：任务身份与状态上报口径（§3.7）----

    def _service_session(self) -> AcquisitionService | None:
        """本 worker 是否运行在**服务持有**的会话上（决定停止时是否释放 USB）。

        仅当注入 service 且当前 backend 就是服务持有的那个会话对象时返回服务；
        测试注入独立 backend 的 worker（即使带 service）返回 None——保持第 1 轮
        "自 open 的 backend 在任务结束时必须释放"语义（R-A-12）。
        """
        service = self._service
        if service is None:
            return None
        if self._backend is None or self._backend is not service.backend:
            return None
        return service

    def _message_run_id(self) -> str | None:
        """本任务消息身份；独立 worker（无 spec）返回 None → sender 用连接身份。"""
        run_id = getattr(self._task_spec, "run_id", "") or ""
        return run_id or None

    def _calibration_id(self) -> str:
        """本任务生效的会话校准身份（R-C-16 透传缝隙；无 spec/未启用 → 空串）。"""
        return str(getattr(self._task_spec, "calibration_id", "") or "")

    def _reports_status(self) -> bool:
        """status_reporting=False 的校准短任务完全不发 acquisition_status（§5.4）。"""
        return bool(getattr(self._task_spec, "status_reporting", True))

    def _emit_acquiring_status(self) -> None:
        """acquiring 必发链（R-A-11）：进入 STREAMING 后、第一条 sweep 处理前。"""
        sender = self.remote_sender
        if sender is None or not self._reports_status():
            return
        sender.enqueue_status(
            "acquiring",
            selected_count=0,
            task_seq=self._task_seq or None,
            run_id=self._message_run_id(),
            remaining=(
                sender.outstanding_count()
                if hasattr(sender, "outstanding_count")
                else 0
            ),
            inflight=(
                sender.inflight_count()
                if hasattr(sender, "inflight_count")
                else 0
            ),
        )

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
            # 第2轮 §3.7：任务身份由 spec 下发；独立 worker 为 None（连接身份）。
            run_id=self._message_run_id(),
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
        keep_backend_open: bool = False,
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

        ``keep_backend_open=True``（第2轮常驻服务模式）：本方法**不**释放 USB
        ——SET_IDLE 停流已在停止边界完成，"排空后是待命还是释放"由调用方在
        综合终态判定后交 :meth:`AcquisitionService.finish_task` 决定（R-A-01/
        R-A-06/R-A-09）；子线程 join 语义与绝不 terminate 完全不变。
        """
        if consumer is not None:
            consumer.request_stop()
        if gnss_thread is not None:
            gnss_thread.request_stop()
        if backend is not None and not keep_backend_open:
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
    if hasattr(window, "sparam_curves"):
        for name in UI_CHANNELS:
            if name in window.sparam_curves:
                window.sparam_curves[name].setData(freq, db20(display[name]))
    if hasattr(window, "ascan_curves"):
        for name in UI_CHANNELS:
            if name in window.ascan_curves:
                window.ascan_curves[name].setData(time_ns, normalize_trace_for_display(trace_by_channel[name]))
    processing_signature = bscan_processing_signature(
        cfg.dewow_enabled,
        cfg.dewow_window_ns,
    )
    window._append_bscan_trace(time_ns, trace_by_channel, processing_signature)
    if hasattr(window, "update_sparam_ranges"):
        window.update_sparam_ranges(freq, display)
    if hasattr(window, "update_ascan_range"):
        window.update_ascan_range(time_ns)
    window.refresh_bscan_views()
    fix_text = f"fix={fix.fix_quality}, sats={fix.satellites}, lat={fix.latitude}, lon={fix.longitude}"
    window.last_info.setText(f"#{index} {fix_text}")
    if hasattr(window, "gnss_map"):
        window.gnss_map.update_fix(
            latitude=fix.latitude,
            longitude=fix.longitude,
            fix_quality=fix.fix_quality,
            satellites=fix.satellites,
            altitude_m=fix.altitude_m,
            utc=fix.utc,
        )
    return fix_text


class AirSettingsDialog(QtWidgets.QDialog):
    """空中端深度设置对话框（非模态，R-C-04）。

    第3轮瘦身：校准页与 VNA 扫描参数页移除（职能移交地面端），
    保留显示/输出/诊断相关页。
    """

    def __init__(self, window: "MainWindow") -> None:
        super().__init__(window)
        self._window = window
        self.setObjectName("air_settings_dialog")
        self.setWindowTitle("空中端设置")
        # R-C-04：非模态——show() 打开、主界面保持可点；生产代码不得 exec()。
        self.setModal(False)
        tabs = QtWidgets.QTabWidget()
        tabs.setObjectName("air_settings_tabs")
        tabs.addTab(self._display_page(), "显示 / 输出 / 诊断")
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(tabs)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Close)
        buttons.setObjectName("air_settings_buttons")
        buttons.rejected.connect(self.hide)
        layout.addWidget(buttons)

    def _display_page(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(page)
        display_group = QtWidgets.QGroupBox("显示")
        display_group.setObjectName("settings_display_group")
        display = QtWidgets.QGridLayout(display_group)
        display.addWidget(QtWidgets.QLabel("B-scan 色图"), 0, 0)
        display.addWidget(self._window.bscan_cmap, 0, 1)
        display.addWidget(QtWidgets.QLabel("B-scan W:H"), 1, 0)
        display.addWidget(self._window.bscan_aspect, 1, 1)
        display.addWidget(QtWidgets.QLabel("Flat window"), 2, 0)
        display.addWidget(self._window.flat_window_traces, 2, 1)
        display.addWidget(self._window.flat_filter_enabled, 3, 0, 1, 2)
        output_group = QtWidgets.QGroupBox("输出与保存")
        output_group.setObjectName("settings_output_group")
        output = QtWidgets.QGridLayout(output_group)
        output.addWidget(QtWidgets.QLabel("输出目录"), 0, 0)
        output.addWidget(self._window.output_dir, 0, 1)
        output.addWidget(self._window.browse_out, 0, 2)
        output.addWidget(self._window.autosave, 1, 0, 1, 3)
        output.setColumnStretch(1, 1)
        diag_group = QtWidgets.QGroupBox("诊断")
        diag_group.setObjectName("settings_diagnostics_group")
        diag = QtWidgets.QGridLayout(diag_group)
        diag.addWidget(QtWidgets.QLabel("设备来源"), 0, 0)
        diag.addWidget(self._window.device_source_combo, 0, 1)
        metrics_btn = QtWidgets.QPushButton("链路性能诊断")
        metrics_btn.setObjectName("air_pipeline_metrics_button")
        metrics_btn.clicked.connect(self._window.log_air_pipeline_metrics)
        diag.addWidget(metrics_btn, 1, 0, 1, 2)
        layout.addWidget(display_group, 0, 0)
        layout.addWidget(output_group, 1, 0)
        layout.addWidget(diag_group, 2, 0)
        return page


class MainWindow(QtWidgets.QMainWindow):
    remote_status = QtCore.Signal(str)
    remote_command = QtCore.Signal(str, object)
    #: 常驻服务状态回调（可能来自 worker/sender 线程）→ queued 回主线程写日志。
    service_status = QtCore.Signal(str)

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
        #: 第2轮常驻采集服务：进程内唯一会话持有者（R-A-01/R-A-05/R-A-07）。
        #: status_callback 用 lambda 延迟取 self.log，避免构造顺序依赖。
        # status_callback 可能被 worker/sender 线程调用，绝不能跨线程直接碰 Qt
        # 控件：统一经 service_status 信号（queued）投递回主线程再写日志。
        #: 当前设备来源（USB 真机 / 模拟器），由“设置”对话框下拉切换；纯 str
        #: 属性，worker/服务线程只读（backend_factory 在工作线程回调），绝不跨
        #: 线程访问 Qt 控件。
        self._device_source = DEVICE_SOURCE_USB
        self.acquisition_service = AcquisitionService(
            backend_factory=self._make_acquisition_backend,
            status_callback=self.service_status.emit,
        )
        self.service_status.connect(self.log)
        self.background: np.ndarray | None = None
        self.background_frequency_hz: np.ndarray | None = None
        self.cal_measurements: dict[str, CalibrationMeasurement] = {}
        self.session_calibration: OSLCalibration | None = None
        self.session_calibration_p2: OSLCalibration | None = None
        self.through_measurement: CalibrationMeasurement | None = None
        # 第2轮工作流 C（§5.2/§5.4）：地面校准编排会话（同一 service 单入口，
        # R-C-14 互斥）；capture_dir 按 cal_id 落 out_dir/_calibration/<cal_id>/。
        self._calibration_session: CalibrationSession | None = None
        self._calibration_lock = threading.RLock()
        self._calibration_thread: threading.Thread | None = None
        #: R-C-16：cal_activate 后的生效校准身份（进 acquisition metadata 的
        #: 透传缝隙见 AcquisitionTaskSpec 注释）。
        self.active_calibration_id: str | None = None
        self._active_calibration_path: Path | None = None
        self.bscan: dict[str, list[np.ndarray]] = {name: [] for name in UI_CHANNELS}
        self.bscan_complex: dict[str, list[np.ndarray]] = {name: [] for name in UI_CHANNELS}
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
        """第3轮精简主界面：连接栏 + 状态条 + B-scan 全屏 + 日志四段。"""
        central = QtWidgets.QWidget()
        central.setObjectName("main_central")
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(8)
        self._create_settings_widgets()
        outer.addWidget(self._connection_bar())
        outer.addWidget(self._status_strip())
        outer.addWidget(self._bscan_section(), 1)
        outer.addWidget(self._log_section())
        # 第2/3轮：深度设置对话框（非模态，R-C-04；生产代码不得 exec()）。
        self.air_settings_dialog = AirSettingsDialog(self)
        self.statusBar().showMessage("Ready")
        # §5.1 状态区新增服务状态行：1s 轮询服务统计（只读展示，§3 state/
        # open_count/"USB 待命中"提示）。
        self._service_state_timer = QtCore.QTimer(self)
        self._service_state_timer.setInterval(1000)
        self._service_state_timer.timeout.connect(self._refresh_service_state_label)
        self._service_state_timer.start()
        self._refresh_service_state_label()
        # 第4轮：采集引擎可见化——启动时把引擎选择行刷成与默认一致的文案。
        self.refresh_engine_mode_label()
        # 启动横幅：真机自测时第一眼就能确认当前引擎（选到 legacy 会每帧刷 Connected）。
        default_now = DEFAULT_ACQUISITION_MODE == ENGINE_MODE_CONTINUOUS
        self.log(
            "采集引擎: "
            + (
                "continuous（高速常驻，USB 只开一次）"
                if is_continuous_enabled()
                else "legacy（每帧开关 USB，较慢）"
            )
            + ("｜出厂默认" if is_continuous_enabled() == default_now else "｜非默认")
        )

    def _spin(self, value: float, min_v: float, max_v: float, decimals: int = 0) -> QtWidgets.QDoubleSpinBox:
        box = QtWidgets.QDoubleSpinBox()
        box.setRange(min_v, max_v)
        box.setDecimals(decimals)
        box.setValue(value)
        box.setKeyboardTracking(False)
        return box

    def _create_settings_widgets(self) -> None:
        """构造全部控件（自旧 _connection_page/_acquisition_page 迁移）。

        属性名 / objectName / 信号连接与第 1 轮逐字一致；宿主改为按 §5.1
        保留清单分组。此处只构造不布局，布局见各 section/dialog。
        """
        # ---- GNSS 串口（任务运行中仍可操作，R-A-13 只锁 LibreVNA）----
        self.port_combo = QtWidgets.QComboBox()
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["115200", "9600", "38400", "57600"])
        self.refresh_btn = QtWidgets.QPushButton("刷新串口")
        self.refresh_btn.setObjectName("refresh_ports_button")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        test_gnss = QtWidgets.QPushButton("测试 GNSS")
        test_gnss.clicked.connect(self.test_gnss)
        self.test_gnss_btn = test_gnss
        self.test_vna_btn = QtWidgets.QPushButton("测试 LibreVNA USB")
        self.test_vna_btn.setObjectName("test_vna_button")
        self.test_vna_btn.clicked.connect(self.test_vna)
        self.device_log = QtWidgets.QPlainTextEdit()
        self.device_log.setObjectName("device_log")
        self.device_log.setReadOnly(True)
        self.device_log.setMaximumBlockCount(200)
        # ---- VNA 基础参数 ----
        self.start_mhz = self._spin(400, 0.001, 6000, 3)
        self.stop_mhz = self._spin(4000, 0.001, 6000, 3)
        self.points = QtWidgets.QSpinBox()
        self.points.setObjectName("points_input")
        self.points.setRange(2, 4501)
        self.points.setValue(201)
        self.ifbw = QtWidgets.QSpinBox()
        self.ifbw.setObjectName("ifbw_input")
        self.ifbw.setRange(6, 50000)
        self.ifbw.setValue(10000)
        self.power = self._spin(-20, -40, 0, 1)
        self.dwell = QtWidgets.QSpinBox()
        self.dwell.setObjectName("dwell_input")
        self.dwell.setRange(0, 10239)
        # ---- 设备来源（模拟器演示 / 无硬件测试；只影响连续采集后端新建）----
        self.device_source_combo = QtWidgets.QComboBox()
        self.device_source_combo.setObjectName("device_source_input")
        for _source in (DEVICE_SOURCE_USB, DEVICE_SOURCE_SIMULATED):
            self.device_source_combo.addItem(DEVICE_SOURCE_LABELS[_source], _source)
        self.device_source_combo.currentIndexChanged.connect(self._on_device_source_changed)
        # ---- 处理参数 ----
        self.ifft_points = QtWidgets.QSpinBox()
        self.ifft_points.setObjectName("ifft_points_input")
        self.ifft_points.setRange(256, 65536)
        self.ifft_points.setValue(2048)
        self.time_window = self._spin(40, 1, 1000, 1)
        self.dewow_window = self._spin(4, 0.001, 1000, 3)
        self.dewow_enabled = QtWidgets.QCheckBox("启用 Dewow")
        self.dewow_enabled.setObjectName("dewow_enabled_input")
        self.dewow_enabled.setChecked(False)
        self.dewow_window.setEnabled(False)
        self.dewow_enabled.toggled.connect(self.dewow_window.setEnabled)
        self.test_sweep_btn = QtWidgets.QPushButton("按当前设置测试扫频")
        self.test_sweep_btn.setObjectName("test_sweep_button")
        self.test_sweep_btn.clicked.connect(self.test_sweep_settings)
        # ---- 校准（空中端本地；硬件出口仍只在空中端，R-C-10）----
        self.cal_path = QtWidgets.QLineEdit()
        self.cal_path.setObjectName("cal_path_input")
        self.browse_cal = QtWidgets.QPushButton("加载校准文件")
        self.browse_cal.setObjectName("browse_cal_button")
        self.browse_cal.clicked.connect(self.choose_cal)
        self.cal_mode = QtWidgets.QComboBox()
        self.cal_mode.setObjectName("cal_mode_input")
        self.cal_mode.addItems(["一端口校准 P1 OSL（三项）", "两端口校准 SOLT/SLOT（七项）"])
        self.cal_mode.currentIndexChanged.connect(lambda: self.update_calibration_status())
        self.cal_lamps: dict[str, QtWidgets.QLabel] = {}
        self.build_cal = QtWidgets.QPushButton("生成并启用 SOLT/SLOT 校准")
        self.build_cal.setObjectName("build_cal_button")
        self.build_cal.clicked.connect(self.build_session_calibration)
        self.save_cal = QtWidgets.QPushButton("保存当前校准")
        self.save_cal.setObjectName("save_cal_button")
        self.save_cal.clicked.connect(self.save_session_calibration)
        self.capture_bg_btn = QtWidgets.QPushButton("采集背景扣除")
        self.capture_bg_btn.setObjectName("capture_background_button")
        self.capture_bg_btn.clicked.connect(self.capture_background)
        self.clear_bg = QtWidgets.QPushButton("清除背景")
        self.clear_bg.setObjectName("clear_background_button")
        self.clear_bg.clicked.connect(self.clear_background)
        self.bg_label = QtWidgets.QLabel("未采集背景")
        self.bg_label.setObjectName("bg_label")
        self.cal_status = QtWidgets.QLabel("未启用校准")
        self.cal_status.setObjectName("cal_status_label")
        self.cal_status.setWordWrap(True)
        self.cal_step_buttons: dict[str, QtWidgets.QPushButton] = {}
        for key, label in CALIBRATION_STEPS:
            lamp = QtWidgets.QLabel()
            lamp.setFixedSize(16, 16)
            lamp.setToolTip(f"{label} 未采集")
            self.cal_lamps[key] = lamp
            button = QtWidgets.QPushButton(f"采集 {label}")
            button.setObjectName(f"cal_step_{key}")
            button.clicked.connect(lambda _checked=False, step_key=key: self.capture_calibration_standard(step_key))
            self.cal_step_buttons[key] = button
        # ---- 显示 ----
        self.bscan_cmap = QtWidgets.QComboBox()
        self.bscan_cmap.setObjectName("bscan_cmap_input")
        self.bscan_cmap.addItems(["gray", "jet"])
        self.bscan_cmap.currentTextChanged.connect(lambda _text: self.refresh_bscan_views())
        self.bscan_aspect = QtWidgets.QDoubleSpinBox()
        self.bscan_aspect.setObjectName("bscan_aspect_input")
        self.bscan_aspect.setRange(0.5, 6.0)
        self.bscan_aspect.setDecimals(2)
        self.bscan_aspect.setSingleStep(0.1)
        self.bscan_aspect.setValue(2.0)
        self.flat_window_traces = QtWidgets.QSpinBox()
        self.flat_window_traces.setObjectName("flat_window_input")
        self.flat_window_traces.setRange(3, 10001)
        self.flat_window_traces.setSingleStep(2)
        self.flat_window_traces.setValue(31)
        self.flat_filter_enabled = QtWidgets.QCheckBox("启用 Flat Filter")
        self.flat_filter_enabled.setObjectName("flat_filter_input")
        self.flat_filter_enabled.setChecked(False)
        self.flat_window_traces.setEnabled(False)
        self.flat_filter_enabled.toggled.connect(self.update_flat_filter_enabled)
        # ---- 输出与保存 ----
        self.output_dir = QtWidgets.QLineEdit(str(Path.cwd() / "gpr_runs"))
        self.output_dir.setObjectName("output_dir_input")
        self.browse_out = QtWidgets.QPushButton("输出目录")
        self.browse_out.setObjectName("browse_output_button")
        self.browse_out.clicked.connect(self.choose_output)
        self.autosave = QtWidgets.QCheckBox("自动保存 CSV/NPZ")
        self.autosave.setObjectName("autosave_input")
        self.autosave.setChecked(True)
        # ---- 任务命令与节奏（保留清单）----
        # 第1轮检查点C：默认"无采集时间间隔"（interval_s=0 = 每个完整Sweep）[冻结]。
        self.interval = self._spin(0.0, 0.0, 3600.0, 2)
        self.count = QtWidgets.QSpinBox()
        self.count.setObjectName("count_input")
        self.count.setRange(0, 1000000)
        self.count.setValue(0)
        self.start_btn = QtWidgets.QPushButton("开始采集")
        self.start_btn.setObjectName("start_button")
        self.start_btn.setMinimumHeight(32)
        self.start_btn.clicked.connect(self.start_acquisition)
        self.single_btn = QtWidgets.QPushButton("采集一次")
        self.single_btn.setObjectName("single_button")
        self.single_btn.setMinimumHeight(32)
        self.single_btn.clicked.connect(lambda: self.start_acquisition(single=True))
        self.stop_btn = QtWidgets.QPushButton("停止")
        self.stop_btn.setObjectName("stop_button")
        self.stop_btn.setMinimumHeight(32)
        self.stop_btn.clicked.connect(self.stop_acquisition)
        self.continuous_enabled = QtWidgets.QCheckBox("连续采集 (continuous)")
        self.continuous_enabled.setObjectName("continuous_enabled_input")
        self.continuous_enabled.setChecked(is_continuous_enabled())
        self.continuous_enabled.setToolTip(
            "采集引擎开关（默认勾选 = 高速连续采集）：勾选走常驻服务 + 持久 USB 会话；"
            "取消勾选走兼容/调试用的 legacy 逐帧引擎（每帧开关一次 USB，慢）"
        )
        # 第4轮（采集速度修复）：引擎开关必须有可见宿主——第3轮界面精简删掉操作栏后
        # 该勾选框失去布局宿主（addWidget 命中 0），用户界面上已无法打开它，导致真机
        # 一直走 legacy 逐帧引擎（约 2.75 s/帧）。这里把它挂进连接栏第 3 行，
        # 与“采集引擎”下拉互相同步（同一语义两个入口，勾选框保持既有契约）。
        self.engine_mode_combo = QtWidgets.QComboBox()
        self.engine_mode_combo.setObjectName("engine_mode_combo")
        self.engine_mode_combo.addItem(ENGINE_MODE_LABEL_CONTINUOUS, ENGINE_MODE_CONTINUOUS)
        self.engine_mode_combo.addItem(ENGINE_MODE_LABEL_LEGACY, ENGINE_MODE_LEGACY)
        self.engine_mode_combo.setToolTip("采集引擎（默认：高速连续采集）")
        self.engine_mode_combo.setCurrentIndex(
            0 if is_continuous_enabled() else 1
        )
        self.engine_mode_combo.currentIndexChanged.connect(self._on_engine_mode_combo_changed)
        self.engine_mode_label = QtWidgets.QLabel("")
        self.engine_mode_label.setObjectName("engine_mode_label")
        # 勾选框与下拉互相同步（同一语义两个入口；测试以勾选框为准，
        # 操作员既可以点下拉也可以点勾选框）。
        self.continuous_enabled.toggled.connect(self._on_continuous_checkbox_toggled)
        self.pause_btn = QtWidgets.QPushButton("暂停")
        self.pause_btn.setObjectName("pause_button")
        self.pause_btn.clicked.connect(self.pause_acquisition)
        self.resume_btn = QtWidgets.QPushButton("继续")
        self.resume_btn.setObjectName("resume_button")
        self.resume_btn.clicked.connect(self.resume_acquisition)
        self.clear_bscan_btn = QtWidgets.QPushButton("清空 B-scan")
        self.clear_bscan_btn.setObjectName("clear_bscan_button")
        self.clear_bscan_btn.clicked.connect(self.clear_bscan)
        # ---- 地面链路（保留清单：remote_* 全链）----
        self.remote_enable = QtWidgets.QCheckBox("Remote send")
        self.remote_enable.setObjectName("remote_enable_input")
        self.remote_enable.setChecked(True)
        self.remote_host = QtWidgets.QLineEdit("127.0.0.1")
        self.remote_host.setObjectName("remote_host_input")
        self.remote_host.setMaximumWidth(140)
        self.remote_port = QtWidgets.QSpinBox()
        self.remote_port.setObjectName("remote_port_input")
        self.remote_port.setRange(1, 65535)
        self.remote_port.setValue(8765)
        self.remote_port.setMaximumWidth(90)
        self.remote_connect = QtWidgets.QPushButton("Connect")
        self.remote_connect.setObjectName("remote_connect_button")
        self.remote_connect.clicked.connect(self.connect_remote_sender)
        self.air_settings_button = QtWidgets.QPushButton("设置…")
        self.air_settings_button.setObjectName("air_settings_button")
        self.air_settings_button.setMinimumHeight(32)
        self.air_settings_button.clicked.connect(self.show_air_settings)
        # ---- 状态区（保留清单 + 新增服务状态行）----
        # 第4轮修复（t3）：两态文案都必须与真实状态一致，且 continuous 态**不得**
        # 出现「未启用」歧义——默认引擎就是连续，写成"未启用"会让操作员以为没生效
        # （与连接栏引擎行"当前引擎：连续 · 高速 · 默认"自相矛盾）。
        self.continuous_status_label = QtWidgets.QLabel(
            CONTINUOUS_STATUS_READY
            if is_continuous_enabled()
            else CONTINUOUS_STATUS_LEGACY
        )
        self.continuous_status_label.setObjectName("continuous_status_label")
        self.last_info = QtWidgets.QLabel("No data")
        self.last_info.setObjectName("last_info_label")
        self.service_state_label = QtWidgets.QLabel("服务状态：closed")
        self.service_state_label.setObjectName("service_state_label")
        self.backlog_status_label = QtWidgets.QLabel("链路积压：空闲/已停止")
        self.backlog_status_label.setObjectName("backlog_status_label")
        # ---- B-scan 主视图（不得移除或降级 [冻结]）----
        self.bscan_window_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.bscan_window_slider.setObjectName("bscan_window_slider")
        self.bscan_window_slider.setRange(1, 1)
        self.bscan_window_slider.setValue(1)
        self.bscan_window_slider.valueChanged.connect(self.on_bscan_window_changed)
        self.bscan_window_label = QtWidgets.QLabel("显示 1–0 / 共 0 道")
        self.bscan_window_label.setObjectName("bscan_window_label")
        self.bscan_window_label.setMinimumWidth(140)
        self.bscan_filter_status = QtWidgets.QLabel("Flat Filter: 无数据")
        self.bscan_filter_status.setObjectName("bscan_filter_status")
        self.bscan_views: dict[str, BScanWidget] = {}
        for name in UI_CHANNELS:
            view = BScanWidget(f"{CHANNEL_LABELS[name]} B-scan")
            view.setMinimumHeight(220)
            view.set_colormap(self.bscan_cmap.currentText())
            self.bscan_views[name] = view
        self.bscan_aspect.valueChanged.connect(self.update_bscan_aspect_ratio)
        self.flat_window_traces.valueChanged.connect(lambda _value: self.refresh_bscan_views())
        self.update_flat_filter_enabled(self.flat_filter_enabled.isChecked())
        self.update_bscan_aspect_ratio(self.bscan_aspect.value())
    def _connection_bar(self) -> QtWidgets.QWidget:
        bar = QtWidgets.QGroupBox("连接")
        bar.setObjectName("connection_bar")
        grid = QtWidgets.QGridLayout(bar)
        grid.setContentsMargins(8, 4, 8, 4)
        grid.setSpacing(6)

        # 行 1: 雷达 USB + GNSS 串口
        grid.addWidget(QtWidgets.QLabel("雷达 USB:"), 0, 0)
        grid.addWidget(self.test_vna_btn, 0, 1)
        grid.addWidget(QtWidgets.QLabel("GNSS 串口:"), 0, 2)
        grid.addWidget(self.port_combo, 0, 3)
        grid.addWidget(QtWidgets.QLabel("波特率:"), 0, 4)
        grid.addWidget(self.baud_combo, 0, 5)
        grid.addWidget(self.refresh_btn, 0, 6)
        grid.addWidget(self.test_gnss_btn, 0, 7)

        # 行 2: 地面站链路 + 设置按钮
        grid.addWidget(self.remote_enable, 1, 0, 1, 2)
        grid.addWidget(QtWidgets.QLabel("地面站 IP:"), 1, 2)
        grid.addWidget(self.remote_host, 1, 3)
        grid.addWidget(QtWidgets.QLabel("端口:"), 1, 4)
        grid.addWidget(self.remote_port, 1, 5)
        grid.addWidget(self.remote_connect, 1, 6)
        grid.addWidget(self.air_settings_button, 1, 7)

        # 行 3: 采集引擎（第4轮采集速度修复：默认高速连续 + 可见可切换）。
        # 置于连接栏（而非已删除的操作栏）——不复活 operator_bar，也不引入
        # 本地开始/停止按钮（第3轮契约：远程命令驱动采集）。
        grid.addWidget(QtWidgets.QLabel("采集引擎:"), 2, 0)
        grid.addWidget(self.engine_mode_combo, 2, 1, 1, 2)
        grid.addWidget(self.continuous_enabled, 2, 3, 1, 2)
        grid.addWidget(self.engine_mode_label, 2, 5, 1, 3)

        grid.setColumnStretch(3, 1)
        return bar

    def _status_strip(self) -> QtWidgets.QWidget:
        group = QtWidgets.QGroupBox("状态")
        group.setObjectName("status_group")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(2)
        for label in (self.service_state_label, self.continuous_status_label, self.backlog_status_label, self.last_info):
            label.setWordWrap(True)
            layout.addWidget(label)
        return group

    def _bscan_section(self) -> QtWidgets.QWidget:
        section = QtWidgets.QWidget()
        section.setObjectName("bscan_section")
        layout = QtWidgets.QVBoxLayout(section)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        bscan_grid = QtWidgets.QGridLayout()
        bscan_grid.setContentsMargins(0, 0, 0, 0)
        bscan_grid.setSpacing(8)
        for index, (name, view) in enumerate(self.bscan_views.items()):
            bscan_grid.addWidget(view, index // 2, index % 2)
        layout.addLayout(bscan_grid, 1)
        slider_row = QtWidgets.QHBoxLayout()
        slider_row.addWidget(self.bscan_window_slider, 1)
        slider_row.addWidget(self.bscan_window_label)
        slider_row.addWidget(self.clear_bscan_btn)
        layout.addLayout(slider_row)
        layout.addWidget(self.bscan_filter_status)
        return section

    def _log_section(self) -> QtWidgets.QWidget:
        self.device_log.setMinimumHeight(110)
        self.device_log.setMaximumHeight(190)
        return self.device_log

    def show_air_settings(self) -> None:
        """打开深度设置对话框（非模态；任务运行中也可只读查看，R-C-04）。"""
        self.air_settings_dialog.show()
        self.air_settings_dialog.raise_()
        self.air_settings_dialog.activateWindow()

    def log_air_pipeline_metrics(self) -> None:
        """把空中端链路性能统计写入日志（诊断入口，不影响主界面）。"""
        if AIR_METRICS is None:
            self.log("链路性能诊断: 未启用性能统计")
            return
        try:
            import json as _json

            self.log(f"链路性能诊断: {_json.dumps(AIR_METRICS.snapshot(), ensure_ascii=False)}")
        except Exception as exc:  # noqa: BLE001 - 诊断不得抛出
            self.log(f"链路性能诊断失败: {exc}")

    def _refresh_service_state_label(self) -> None:
        """服务状态行（§5.1 状态区新增）：state / open_count / 待命提示。"""
        try:
            stats = self.acquisition_service.statistics()
        except Exception:  # noqa: BLE001 - 只读展示不得抛出
            return
        state = str(stats.get("state", "unknown"))
        open_count = stats.get("backend_open_count", 0)
        configure_count = stats.get("backend_configure_count", 0)
        hints = {
            SERVICE_STATE_IDLE: "（USB 待命中：任务已停止，设备保持连接）",
            SERVICE_STATE_ACQUIRING: "（采集中）",
            SERVICE_STATE_STOPPING: "（正在停止并排空）",
            SERVICE_STATE_FAULTED: "（故障，USB 已/将释放）",
        }
        self.service_state_label.setText(
            f"服务状态：{state} | open={open_count} configure={configure_count} {hints.get(state, '')}"
        )
        if hasattr(self, "backlog_status_label"):
            self.backlog_status_label.setText(f"链路积压：{self.backlog_text(state=state)}")

    def backlog_counts(self) -> dict[str, int]:
        """获取当前链路积压计数的快照字典。"""
        sender = getattr(self, "remote_sender", None)
        if sender is not None and hasattr(sender, "backlog_counts"):
            return sender.backlog_counts()
        return {"inflight": 0, "unacknowledged": 0, "outbox_pending": 0, "stopping_remaining": 0}

    def backlog_text(self, state: str = "") -> str:
        """获取当前链路积压的展示文案。"""
        sender = getattr(self, "remote_sender", None)
        if sender is not None and hasattr(sender, "backlog_text"):
            return sender.backlog_text(state=state)
        return "空闲/已停止"

    def _engine_mode_from_combo(self) -> str:
        """当前下拉选中的引擎标识（continuous / legacy）。"""
        data = self.engine_mode_combo.currentData()
        return str(data) if data in (ENGINE_MODE_CONTINUOUS, ENGINE_MODE_LEGACY) else DEFAULT_ACQUISITION_MODE

    def _on_engine_mode_combo_changed(self, _index: int) -> None:
        """下拉切引擎 → 同步勾选框（勾选框仍是 start_acquisition 的判据）。"""
        want_continuous = self._engine_mode_from_combo() == ENGINE_MODE_CONTINUOUS
        if self.continuous_enabled.isChecked() != want_continuous:
            self.continuous_enabled.setChecked(want_continuous)
        self.refresh_engine_mode_label()
        self.log(
            "采集引擎已切换为: "
            + ("高速连续采集（下次启动采集生效）" if want_continuous else "legacy 兼容/调试引擎（下次启动采集生效，较慢）")
        )

    def _on_continuous_checkbox_toggled(self, checked: bool) -> None:
        """勾选框切换 → 同步下拉（两个入口同一语义，绝不各说各话）。"""
        target = ENGINE_MODE_CONTINUOUS if checked else ENGINE_MODE_LEGACY
        index = self.engine_mode_combo.findData(target)
        if index >= 0 and index != self.engine_mode_combo.currentIndex():
            self.engine_mode_combo.setCurrentIndex(index)
        self.refresh_engine_mode_label()

    def refresh_engine_mode_label(self) -> None:
        """引擎行只读回显：当前引擎 + 是否高速 + 当前是不是默认值。"""
        continuous = self.continuous_enabled.isChecked()
        default_note = (
            "默认"
            if continuous == (DEFAULT_ACQUISITION_MODE == ENGINE_MODE_CONTINUOUS)
            else "非默认（调试用）"
        )
        speed = "高速（USB 只开一次）" if continuous else "慢（每帧开关一次 USB）"
        self.engine_mode_label.setText(f"当前引擎：{'连续' if continuous else 'legacy'} · {speed} · {default_note}")

    def _make_acquisition_backend(self) -> PersistentLibreVnaBackend:
        """按当前“设备来源”构造连续采集后端（AcquisitionService 工厂注入）。

        - USB 真机：``transport=None`` → 后端内部默认 :class:`LibreVnaUsbTransport`
          （与第 2 轮语义逐字一致，真机仍为默认）；
        - 模拟器：注入 :class:`SimulatedLibreVnaTransport`（协议线假 LibreVNA，
          生成冰层场景 VNADatapoint 流）——后端/worker/服务/流水线/地面站
          全部走既有代码路径，零适配。
        """
        if self._device_source == DEVICE_SOURCE_SIMULATED:
            transport: LibreVnaTransportProtocol | None = SimulatedLibreVnaTransport()
        else:
            transport = None
        return PersistentLibreVnaBackend(transport=transport)

    def device_source(self) -> str:
        """当前设备来源（USB 真机 / 模拟器）。"""
        return self._device_source

    def _on_device_source_changed(self, index: int) -> None:
        """设备来源切换（USB 真机 ↔ 模拟器）。

        语义（尽量简单、不伪装）：
        - 采集任务运行中（worker 在跑 / 服务 acquiring/stopping）→ 拒绝并回退；
        - 服务待命（IDLE_WAITING，传输仍持有）→ 先释放待命会话再切换——
          传输层无法热更换，切换后下一次任务按新来源新建后端；
        - 待命释放失败 → 保持原来源并回退（资源仍被持有，可重试）。
        """
        combo = self.device_source_combo
        chosen = combo.itemData(index)
        previous = self._device_source
        if chosen == previous:
            return
        if self._acquisition_running():
            self._revert_device_source_combo(combo, previous)
            self.log("设备来源切换被拒绝：采集任务运行中（停止后才能切换）")
            return
        service = self.acquisition_service
        if service.state == SERVICE_STATE_IDLE:
            if not service.release_idle_session():
                self.log(
                    f"设备来源切换失败：待命会话释放未成功"
                    f"（state={service.state}），保持 {DEVICE_SOURCE_LABELS[previous]}"
                )
                self._revert_device_source_combo(combo, previous)
                return
        self._device_source = chosen
        # 同步进程级默认来源：未显式注入 backend/factory 的两处装配回退点
        # （服务默认后端 / 独立 worker 自建后端）也服从本次切换。
        set_default_device_source(chosen)
        self.log(
            f"设备来源已切换: {DEVICE_SOURCE_LABELS[previous]}"
            f" → {DEVICE_SOURCE_LABELS[chosen]}"
        )

    @staticmethod
    def _revert_device_source_combo(combo: QtWidgets.QComboBox, source: str) -> None:
        """把设备来源下拉回退到指定值（不触发切换逻辑）。"""
        index = combo.findData(source)
        if index < 0:
            return
        combo.blockSignals(True)
        combo.setCurrentIndex(index)
        combo.blockSignals(False)

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
        if hasattr(self, "sparam_plot"):
            if frequency_hz.size:
                self.sparam_plot.setXRange(float(frequency_hz[0]), float(frequency_hz[-1]), padding=0)
            values = [db20(display[name]) for name in UI_CHANNELS if name in display]
            self.sparam_plot.setYRange(*robust_db_range(values, default=(-60.0, 10.0)), padding=0)

    def update_ascan_range(self, time_ns: np.ndarray) -> None:
        if hasattr(self, "ascan_plot"):
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
        if self._require_real_device("测试 LibreVNA USB"):
            return
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
        if self._require_real_device("按当前设置测试扫频"):
            return
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
            if hasattr(self, "sparam_curves"):
                for name in UI_CHANNELS:
                    if name in self.sparam_curves:
                        self.sparam_curves[name].setData(freq, db20(display[name]))
                    if hasattr(self, "ascan_curves") and name in self.ascan_curves:
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
                    # 本地加载文件后，远程校准身份不再描述当前生效校准。
                    self.active_calibration_id = None
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
        if self._require_real_device(f"采集校准标准 {standard}"):
            return
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
            self.active_calibration_id = None
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
        if self._require_real_device("采集背景扣除"):
            return
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
        """任一采集 worker 仍在运行（legacy 或 continuous），或常驻服务正持有
        活动任务（acquiring/stopping）。第2轮：控件锁定判据与任务命令作用域。"""
        if self.worker is not None and self.worker.isRunning():
            return True
        service = self.acquisition_service
        return service.state in (SERVICE_STATE_ACQUIRING, SERVICE_STATE_STOPPING)

    def _holds_usb_session(self) -> bool:
        """R-A-13 手动 USB 互斥判据（并集）：任务在跑 **或** 服务持有会话。

        服务 `idle_waiting` 时接口仍被 claim，`acquire_sparameters` /
        `open_device` 自开自关必然与之冲突，因此待命期同样拒绝；只有
        `shutdown()` 真正释放后才恢复（R-A-01/R-A-07）。既有两条
        `test_manual_usb_entries_blocked_while_*` 用 `window.worker` 直接赋值
        绕过服务，其"worker 运行期拒绝 / 结束后恢复"断言不受本判据影响。
        """
        return self._acquisition_running() or self.acquisition_service.holds_usb

    def _reject_manual_usb_op(self, action: str) -> bool:
        """任务运行期间拒绝手动访问 LibreVNA 的操作；返回 True 表示已拒绝。

        方法入口必须做运行状态检查（不能只依赖按钮 disabled）；拒绝时记录明确
        日志且绝不触发 USB 调用（roadmap §3 互斥）。
        """
        if self._holds_usb_session():
            state = self.acquisition_service.state
            self.log(f"{action}: 采集任务运行中或服务持有 USB 会话（state={state}），禁止手动访问 LibreVNA")
            return True
        return False

    def _require_real_device(self, action: str) -> bool:
        """设备来源=模拟器时拒绝只支持真机的操作；返回 True 表示已拒绝。

        模拟通道是连续采集协议线旁路：手动单扫 / 校准 / USB 测试等 legacy
        （open_device / acquire_sparameters）入口不提供模拟适配，明确提示，
        绝不静默走向真机。
        """
        if self._device_source == DEVICE_SOURCE_SIMULATED:
            self.log(
                f"{action}: 需要 USB 真机（当前设备来源=模拟器）。"
                "模拟器覆盖连续采集链路，请改用“连续采集 (continuous)”或切回 USB 真机"
            )
            return True
        return False

    def _set_task_controls_locked(self, locked: bool) -> None:
        """任务运行期间锁定会访问 LibreVNA / 改变冻结任务配置的控件。

        停止选择新数据后仍可用的安全控件（Stop、B-scan 浏览等）不在此列表。
        第2轮 §5.1 防误触扩展：无论控件宿主是主界面还是已打开的
        :class:`AirSettingsDialog`，锁定一律生效（同一批控件实例）；
        "设置…"按钮 / Stop / B-scan 滑块 / GNSS 串口选择不在清单内。
        """
        for widget in (
            self.start_mhz, self.stop_mhz, self.points, self.ifbw, self.power,
            self.dwell, self.ifft_points, self.time_window, self.dewow_window,
            self.dewow_enabled, self.interval, self.count, self.autosave,
            self.continuous_enabled, self.cal_path, self.cal_mode,
            self.test_gnss_btn, self.test_vna_btn, self.test_sweep_btn,
            self.device_source_combo,
            self.capture_bg_btn, self.start_btn, self.single_btn,
            # 第2轮 §5.1 扩展：改变校准/输出配置的入口随任务一起锁定。
            self.browse_cal, self.build_cal, self.save_cal,
            self.clear_bg, self.browse_out,
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
        # 第2轮 R-C-14：校准会话与采集互斥（同一判据、同一函数；服务 start_task
        # 的自有拒绝互为纵深）。
        conflict = self._calibration_acquisition_conflict("启动采集")
        if conflict:
            self.log(conflict)
            return
        cfg = self.config()
        if cfg.stop_mhz <= cfg.start_mhz:
            self.log("Stop MHz 必须大于 Start MHz")
            return
        if (
            self._device_source == DEVICE_SOURCE_SIMULATED
            and not self.continuous_enabled.isChecked()
        ):
            # 模拟器走连续采集协议线假设备；legacy 单次采集路径（open_device/
            # acquire_sparameters）保持只接真机，不在此做第二条模拟适配。
            self.log(
                "模拟器（无硬件演示）只支持连续采集："
                "请勾选“连续采集 (continuous)”后重试"
            )
            return
        # 第5轮 Q2=A：开始新采集时自动清空 B-scan（新任务从干净画布开始；
        # 停止后不清，历史图可回看）。clear_bscan 同时复位滑块/计数/Flat 状态。
        self.clear_bscan()
        if self.remote_enable.isChecked():
            self.connect_remote_sender()
        if self.continuous_enabled.isChecked():
            self.log("本任务引擎: continuous（高速常驻，USB 只开一次）")
            self._start_continuous_acquisition(cfg, single)
            return
        # 第4轮：走到这里说明引擎被显式切成了 legacy（默认不会）——写明"为什么慢"，
        # 免得真机自测时把 2.75 s/帧 的旧引擎当成新代码没生效。
        self.log("本任务引擎: legacy（每帧 acquire_sparameters 开关一次 USB，真机约 2.75 s/帧；如需高速请切回连续采集）")
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
        self.log(f"连续采集启动: 设备来源 {DEVICE_SOURCE_LABELS[self._device_source]}")
        # 第2轮常驻服务：任务经 service.start_task 统一入口创建（R-A-05）。
        # UI 侧仍以工厂构造 worker，保留 ContinuousAcquisitionWorker 的类级
        # patch 语义（既有测试用 unittest.mock.patch.object(air, ...) 替换）。
        spec = AcquisitionTaskSpec(
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
            **(
                # R-C-16：激活校准后任务 spec 携带 calibration_id（经 engA 类域
                # 的 SelectedSweep→acquisition metadata 透传缝隙进入每条 trace
                # 的嵌套 metadata，不进 framing 顶层）。缝隙落地前字段不存在
                # 时不传——绝不伪造 metadata key。
                {"calibration_id": self.active_calibration_id or ""}
                if "calibration_id" in AcquisitionTaskSpec.__dataclass_fields__
                else {}
            ),
        )

        def _factory(task_spec: AcquisitionTaskSpec) -> ContinuousAcquisitionWorker:
            worker = ContinuousAcquisitionWorker(
                cfg=task_spec.cfg,
                interval_s=task_spec.interval_s,
                out_dir=task_spec.out_dir,
                count=task_spec.count,
                gnss_port=task_spec.gnss_port,
                baud=task_spec.baud,
                calibration_path=task_spec.calibration_path,
                session_calibration=task_spec.session_calibration,
                session_calibration_p2=task_spec.session_calibration_p2,
                through_measurement=task_spec.through_measurement,
                background_frequency_hz=task_spec.background_frequency_hz,
                background=task_spec.background,
                autosave=task_spec.autosave,
                remote_sender=task_spec.remote_sender,
                service=self.acquisition_service,
                task_spec=task_spec,
            )
            worker.trace_ready.connect(self.on_continuous_trace)
            worker.status.connect(self.log)
            worker.failed.connect(lambda msg: self.log(f"连续采集失败: {msg}"))
            worker.finished_run.connect(self.on_worker_finished)
            worker.paused.connect(self._on_continuous_paused)
            worker.resumed.connect(lambda: self.log("连续采集已恢复"))
            worker.statistics_updated.connect(self.on_continuous_statistics)
            return worker

        worker = self.acquisition_service.start_task(spec, _factory)
        if worker is None:
            self.log("常驻服务拒绝启动采集任务（详见服务状态日志）")
            return
        self.worker = worker
        self._set_task_controls_locked(True)
        self.continuous_status_label.setText("连续模式: 运行中...")

    def _on_continuous_paused(self) -> None:
        """暂停在扫频边界生效后：日志 + 向地面发 paused 状态（带真实积压计数）。

        第5轮 Q1=B：地面据此显示"已暂停 · 还差 K 条"并随逐条到达递减；
        恢复采集不另发状态（command_ack resume 已驱动地面 UI，积压计数
        由逐条递减自然归零）。
        """
        self.log("连续采集已暂停")
        sender = getattr(self, "remote_sender", None)
        if sender is None or not hasattr(sender, "enqueue_status"):
            return
        sender.enqueue_status(
            "paused",
            remaining=(
                sender.outstanding_count()
                if hasattr(sender, "outstanding_count")
                else 0
            ),
            inflight=(
                sender.inflight_count()
                if hasattr(sender, "inflight_count")
                else 0
            ),
            run_id=self._message_run_id(),
        )

    def pause_acquisition(self) -> None:
        # 第2轮：暂停命令统一走服务入口（与地面命令同一语义、同一作用域）。
        if self.acquisition_service.request_pause():
            self.log("请求暂停连续采集...")
            return
        worker = self.worker
        if isinstance(worker, ContinuousAcquisitionWorker) and worker.isRunning():
            worker.request_pause()
            self.log("请求暂停连续采集...")
        else:
            self.log("当前采集不支持暂停")

    def resume_acquisition(self) -> None:
        if self.acquisition_service.request_resume():
            self.log("请求继续连续采集...")
            return
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
        # 第2轮 §3.7：连接（通道）身份统一由服务签发（idle 式，永不携带 trace；
        # 任务消息各自携带自己的 run_id）。生产代码不得再有第二处 run_id 拼接。
        self.remote_run_id = self.acquisition_service.new_run_id(RUN_ID_KIND_IDLE)
        self.remote_sender = RemoteSender(
            host,
            int(self.remote_port.value()),
            self.remote_run_id,
            self.remote_status.emit,
            self.remote_command.emit,
            # 第2轮 t5：磁盘 outbox 落在采集输出目录下（DiskOutbox 只在首次
            # append 时建目录，构造本身不产生任何目录，避免污染 cwd/gpr_runs）。
            outbox_dir=Path(self.output_dir.text()),
        )
        self.remote_sender.start()
        self.log(f"Remote sender connecting: {host}:{self.remote_port.value()}")

    def _is_acquisition_running(self) -> bool:
        worker_running = self.worker is not None and self.worker.isRunning()
        service_busy = self.acquisition_service.state in (
            SERVICE_STATE_ACQUIRING,
            SERVICE_STATE_STOPPING,
            SERVICE_STATE_SHUTTING_DOWN,
        )
        return worker_running or service_busy

    def _handle_set_params_command(self, params: dict[str, object]) -> bool:
        if self._is_acquisition_running():
            self.log("set_params 拒绝: 采集运行中不可修改配置")
            return False
        with self._calibration_lock:
            session = self._calibration_session
            if session is not None and session.state in (
                CalibrationSession.STATE_PLANNED,
                CalibrationSession.STATE_IN_PROGRESS,
                CalibrationSession.STATE_SOLVED,
            ):
                self.log("set_params 拒绝: 校准会话进行中不可修改配置")
                return False

        try:
            start = float(params.get("start_mhz", self.start_mhz.value()))
            stop = float(params.get("stop_mhz", self.stop_mhz.value()))
            if stop <= start:
                self.log(f"set_params 拒绝: 终止频率 ({stop} MHz) 必须大于起始频率 ({start} MHz)")
                return False

            if "points" in params:
                pts = int(params["points"])
                if pts < 2 or pts > 4501:
                    self.log(f"set_params 拒绝: 频点数 ({pts}) 超出范围 [2, 4501]")
                    return False

            ifbw_val = params.get("ifbw_hz", params.get("ifbw"))
            if ifbw_val is not None:
                ifbw = int(ifbw_val)
                if ifbw < 6 or ifbw > 50000:
                    self.log(f"set_params 拒绝: 中频带宽 ({ifbw} Hz) 超出范围 [6, 50000]")
                    return False

            pwr_val = params.get("power_dbm", params.get("power"))
            if pwr_val is not None:
                pwr = float(pwr_val)
                if pwr < -40.0 or pwr > 0.0:
                    self.log(f"set_params 拒绝: 发射功率 ({pwr} dBm) 超出范围 [-40, 0]")
                    return False

            dwell_val = params.get("dwell_us", params.get("dwell"))
            if dwell_val is not None:
                dwell = int(dwell_val)
                if dwell < 0 or dwell > 10239:
                    self.log(f"set_params 拒绝: 驻留时间 ({dwell} us) 超出范围 [0, 10239]")
                    return False

            if "ifft_points" in params:
                ifft = int(params["ifft_points"])
                if ifft < 256 or ifft > 65536:
                    self.log(f"set_params 拒绝: IFFT 点数 ({ifft}) 超出范围 [256, 65536]")
                    return False

            tw_val = params.get("time_window_ns", params.get("time_window"))
            if tw_val is not None:
                tw = float(tw_val)
                if tw < 1.0 or tw > 1000.0:
                    self.log(f"set_params 拒绝: 时间窗 ({tw} ns) 超出范围 [1, 1000]")
                    return False

            dw_val = params.get("dewow_window_ns", params.get("dewow_window"))
            if dw_val is not None:
                dw = float(dw_val)
                if dw <= 0.0 or dw > 1000.0:
                    self.log(f"set_params 拒绝: Dewow 窗长 ({dw} ns) 必须在 (0, 1000] 范围内")
                    return False
        except (ValueError, TypeError) as exc:
            self.log(f"set_params 拒绝: 参数类型解析错误: {exc}")
            return False

        if "start_mhz" in params:
            self.start_mhz.setValue(float(params["start_mhz"]))
        if "stop_mhz" in params:
            self.stop_mhz.setValue(float(params["stop_mhz"]))
        if "points" in params:
            self.points.setValue(int(params["points"]))
        if ifbw_val is not None:
            self.ifbw.setValue(int(ifbw_val))
        if pwr_val is not None:
            self.power.setValue(float(pwr_val))
        if dwell_val is not None:
            self.dwell.setValue(int(dwell_val))
        if "ifft_points" in params:
            self.ifft_points.setValue(int(params["ifft_points"]))
        if tw_val is not None:
            self.time_window.setValue(float(tw_val))
        if "dewow_enabled" in params:
            self.dewow_enabled.setChecked(bool(params["dewow_enabled"]))
        if dw_val is not None:
            self.dewow_window.setValue(float(dw_val))

        cfg = self.config()
        self.log(
            f"远程参数更新成功: {cfg.start_mhz:g}-{cfg.stop_mhz:g} MHz, "
            f"{cfg.points} 点, IFBW={cfg.ifbw_hz} Hz, 功率={cfg.power_dbm:g} dBm"
        )
        return True

    def handle_remote_command(self, action: str, params: object) -> None:
        self.log(f"Remote command: {action}")
        if isinstance(params, dict):
            if "count" in params:
                self.count.setValue(max(0, min(int(params["count"]), self.count.maximum())))
            if "interval_s" in params:
                self.interval.setValue(float(params["interval_s"]))
        # 第2轮 §5.4：地面校准命令走既有 type="command" + 新 action 值
        # （严禁新增地面→空中消息 type——空中对未知入站 type 会踢下线）。
        if action in _CALIBRATION_COMMAND_ACTIONS:
            self._handle_calibration_command(action, params if isinstance(params, dict) else {})
            return
        if action == "start":
            single = bool(params.get("single", False)) if isinstance(params, dict) else False
            self.start_acquisition(single=single)
        elif action == "stop":
            self.stop_acquisition()
        elif action == "single":
            self.start_acquisition(single=True)
        elif action == "pause":
            self.pause_acquisition()
        elif action == "resume":
            self.resume_acquisition()
        elif action == "set_params":
            self._handle_set_params_command(params if isinstance(params, dict) else {})
        else:
            self.log(f"Unknown remote command: {action}")

    # ---- 第2轮工作流 C：地面校准编排（§5.2/§5.4，R-C-05…R-C-19）----

    def _calibration_acquisition_conflict(self, action: str, cal_id: str = "") -> str | None:
        """校准/采集互斥判据（R-C-14：唯一实现，start 与 cal_* 两方向共用）。

        返回拒绝原因；None = 允许。规则：

        - 采集任务运行中（worker 在跑或服务 acquiring/stopping/shutting_down）
          → 采集与 cal_* **全部**拒绝；
        - 校准会话进行中（planned/in_progress/solved）→ **采集方向**拒绝
          （cal_id 缺省 = 采集调用）；**cal_* 方向**仅当 cal_id 与当前会话
          不匹配时拒绝（同会话的 cal_capture/cal_finish/cal_activate/
          cal_cancel 是编排本体，必须放行）。

        与服务单入口 ``start_task`` 的自有拒绝互为纵深，绝不绕过。
        """
        state = self.acquisition_service.state
        worker_running = self.worker is not None and self.worker.isRunning()
        if worker_running or state in (
            SERVICE_STATE_ACQUIRING,
            SERVICE_STATE_STOPPING,
            SERVICE_STATE_SHUTTING_DOWN,
        ):
            return f"{action} 被拒绝: 采集任务运行中或服务正在收尾（state={state}）"
        session = self._calibration_session
        session_open = (
            session is not None
            and session.state
            in (
                CalibrationSession.STATE_PLANNED,
                CalibrationSession.STATE_IN_PROGRESS,
                CalibrationSession.STATE_SOLVED,
            )
        )
        if not session_open:
            return None
        if cal_id and session.cal_id == cal_id:
            # 同一会话的编排命令（cal_capture/cal_finish/cal_activate/cal_cancel）。
            return None
        return (
            f"{action} 被拒绝: 校准会话进行中"
            f"（cal_id={session.cal_id}, state={session.state}）"
        )

    def _send_calibration_message(self, message_type: str, cal_id: str, payload: bytes = b"", **fields: object) -> None:
        """校准回程（§5.4）：run_id 沿用当前连接身份，不触发带内身份切换。"""
        sender = self.remote_sender
        if sender is None:
            self.log(f"校准回程无法发送（未连接地面端）: {message_type} cal_id={cal_id}")
            return
        header: dict[str, object] = {"type": message_type, "cal_id": cal_id}
        for key, value in fields.items():
            if value is not None:
                header[key] = value
        try:
            sender.enqueue_calibration(header, payload)
        except Exception as exc:  # noqa: BLE001 - 回程失败必须可见，不影响采集
            self.log(f"校准回程入队失败: {message_type} cal_id={cal_id}: {exc}")

    def _handle_calibration_command(self, action: str, params: dict[str, object]) -> None:
        """地面校准命令入口（cal_begin/cal_capture/cal_finish/cal_activate/cal_cancel）。

        全部动作先过 :meth:`_calibration_acquisition_conflict`（R-C-14）；
        拒绝时回 ``command_ack``（RemoteSender 已发）+ ``calibration_result``
        携带可读原因（不静默）。
        """
        cal_id = str(params.get("cal_id") or "")
        conflict = self._calibration_acquisition_conflict(action, cal_id=cal_id)
        if conflict:
            self.log(conflict)
            self._send_calibration_message(
                "calibration_result", cal_id, status="rejected", reason=conflict
            )
            return
        if action == "cal_begin":
            self._begin_calibration_session(params)
        elif action == "cal_capture":
            self._command_calibration_capture(cal_id, str(params.get("step") or ""))
        elif action == "cal_finish":
            self._command_calibration_finish(cal_id)
        elif action == "cal_activate":
            self._command_calibration_activate(cal_id)
        elif action == "cal_cancel":
            self._command_calibration_cancel(cal_id, str(params.get("reason") or "地面请求取消"))
        elif action == "cal_status":
            self._command_calibration_status(cal_id)

    def _calibration_config_from_snapshot(self, snapshot: dict[str, object]) -> SweepConfig:
        """由地面 cal_begin 的 SweepConfig 快照构建扫频配置（缺省回退当前 UI 值）。"""

        def _number(key: str, fallback: float) -> float:
            value = snapshot.get(key, fallback)
            try:
                return float(value)  # type: ignore[arg-type]
            except (TypeError, ValueError):
                return float(fallback)

        return SweepConfig(
            start_mhz=_number("start_mhz", self.start_mhz.value()),
            stop_mhz=_number("stop_mhz", self.stop_mhz.value()),
            points=int(_number("points", self.points.value())),
            ifbw_hz=int(_number("ifbw_hz", self.ifbw.value())),
            power_dbm=_number("power_dbm", self.power.value()),
            dwell_us=int(_number("dwell_us", self.dwell.value())),
            ifft_points=self.ifft_points.value(),
            time_window_ns=self.time_window.value(),
            dewow_enabled=self.dewow_enabled.isChecked(),
            dewow_window_ns=self.dewow_window.value(),
        )

    def _begin_calibration_session(self, params: dict[str, object]) -> None:
        cal_id = str(params.get("cal_id") or "").strip()
        mode = str(params.get("mode") or CALIBRATION_MODE_OSL)
        if not cal_id:
            self._send_calibration_message(
                "calibration_result", "", status="rejected", reason="cal_begin 缺少 cal_id"
            )
            return
        with self._calibration_lock:
            existing = self._calibration_session
            if (
                existing is not None
                and existing.state
                in (
                    CalibrationSession.STATE_PLANNED,
                    CalibrationSession.STATE_IN_PROGRESS,
                    CalibrationSession.STATE_SOLVED,
                )
            ):
                self._send_calibration_message(
                    "calibration_result",
                    cal_id,
                    status="rejected",
                    reason=f"已有校准会话进行中（cal_id={existing.cal_id}）",
                )
                return
            try:
                repeats = validate_repeats(params.get("repeats", REPEATS_DEFAULT))
                if mode == CALIBRATION_MODE_OSL:
                    plan = build_osl_plan(cal_id, repeats, {})
                elif mode == CALIBRATION_MODE_BACKGROUND:
                    plan = build_background_plan(cal_id, repeats, {})
                else:
                    raise ValueError(f"未知校准模式: {mode!r}")
                cfg = self._calibration_config_from_snapshot(
                    params.get("config") if isinstance(params.get("config"), dict) else {}
                )
                if cfg.stop_mhz <= cfg.start_mhz:
                    raise ValueError("Stop MHz 必须大于 Start MHz")
                capture_dir = Path(self.output_dir.text()) / "_calibration" / safe_cal_dir_name(cal_id)
                session = CalibrationSession(plan, _ServiceCalibrationCapturerProxy(self, cal_id), capture_dir=capture_dir)
                session.config_snapshot = {
                    "start_mhz": cfg.start_mhz,
                    "stop_mhz": cfg.stop_mhz,
                    "points": cfg.points,
                    "ifbw_hz": cfg.ifbw_hz,
                    "power_dbm": cfg.power_dbm,
                    "dwell_us": cfg.dwell_us,
                }
            except Exception as exc:  # noqa: BLE001 - 计划非法：拒绝并回传原因
                self.log(f"cal_begin 计划非法: {exc}")
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected", reason=str(exc)
                )
                return
            self._calibration_session = session
        self.log(f"校准会话已建立: cal_id={cal_id} mode={mode} repeats={repeats}")
        self._send_calibration_message(
            "calibration_result",
            cal_id,
            status="planned",
            mode=mode,
            steps=[step.name for step in plan.steps],
            repeats=repeats,
        )

    def _command_calibration_capture(self, cal_id: str, step_name: str) -> None:
        with self._calibration_lock:
            session = self._calibration_session
            if session is None or session.cal_id != cal_id:
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected",
                    reason="无进行中的校准会话或 cal_id 不匹配（防跨会话错配）",
                )
                return
            if session.state not in (CalibrationSession.STATE_PLANNED, CalibrationSession.STATE_IN_PROGRESS):
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected",
                    reason=f"会话状态 {session.state} 不允许采集",
                )
                return
            pending = session.next_pending_step()
            if pending is None or pending.name != step_name:
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected",
                    reason=f"下一待采步骤为 {pending.name if pending else '无'}，收到 {step_name!r}",
                )
                return
            if self._calibration_thread is not None and self._calibration_thread.is_alive():
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected",
                    reason="上一步骤采集仍在进行",
                )
                return
            thread = threading.Thread(
                target=self._calibration_capture_driver,
                args=(session, step_name),
                name=f"uav-gpr-cal-capture-{step_name}",
                daemon=True,
            )
            self._calibration_thread = thread
        self.log(f"校准步骤开始采集: cal_id={cal_id} step={step_name}")
        thread.start()

    def _calibration_capture_driver(self, session: CalibrationSession, step_name: str) -> None:
        """校准采集驱动线程：执行一个步骤的 repeats 条采集（会话状态机内）。"""
        try:
            step = session.plan.step(step_name)

            def _on_capture(frequency: np.ndarray, raw: np.ndarray, index: int, of: int) -> None:
                # 每条完整 sweep 一条回程（NPZ 载荷），进度实时可见（R-C-11）。
                payload = _calibration_capture_npz_bytes(frequency, raw)
                self._send_calibration_message(
                    "calibration_capture",
                    session.cal_id,
                    payload,
                    step=step_name,
                    index=index,
                    of=of,
                    channel=step.channel,
                    points=int(frequency.size),
                    payload_format="npz",
                )

            session.capture(step_name, on_capture=_on_capture)
            self.log(f"校准步骤采集完成: cal_id={session.cal_id} step={step_name}")
        except Exception as exc:  # noqa: BLE001 - 步骤失败：会话已判 failed
            self.log(f"校准步骤采集失败: cal_id={session.cal_id} step={step_name}: {exc}")
            self._send_calibration_message(
                "calibration_result",
                session.cal_id,
                status="failed",
                step=step_name,
                reason=str(exc),
            )
        finally:
            # 有界收尾：停止本步骤的服务短任务（worker count=repeats 已自停，
            # 这里补一次统一入口 request_stop，等待退出，绝不 terminate）。
            proxy = session.capturer
            close = getattr(proxy, "close_current", None)
            if callable(close):
                close(step_name)
            with self._calibration_lock:
                self._calibration_thread = None

    def _command_calibration_finish(self, cal_id: str) -> None:
        with self._calibration_lock:
            session = self._calibration_session
            if session is None or session.cal_id != cal_id:
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected",
                    reason="无进行中的校准会话或 cal_id 不匹配",
                )
                return
            if self._calibration_thread is not None and self._calibration_thread.is_alive():
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected",
                    reason="校准采集仍在进行，不能解算",
                )
                return
            try:
                result = session.solve()
            except Exception as exc:  # noqa: BLE001 - 解算失败：会话已判 failed
                self.log(f"校准解算失败: cal_id={cal_id}: {exc}")
                self._send_calibration_message(
                    "calibration_result", cal_id, status="failed", reason=str(exc)
                )
                return
        self.log(f"校准解算完成: cal_id={cal_id}")
        # 每步回传校正后质量指标（R-C-08：逐条校正后的重复 RMS/最大绝对误差）。
        self._send_step_quality(session, result)
        # 解算结果落盘（复用 uav_gpr_solt_calibration_v2 格式，R-C-09）。
        path: Path | None = None
        if session.plan.mode == CALIBRATION_MODE_OSL:
            measurements = self._measurements_from_result(result)
            cfg = self._calibration_config_from_snapshot(getattr(session, "config_snapshot", {}) or {})
            path = session.capture_dir / f"{safe_cal_dir_name(cal_id)}.json" if session.capture_dir else None
            if path is not None:
                try:
                    write_calibration_file(path, measurements, cfg)
                except Exception as exc:  # noqa: BLE001
                    self.log(f"校准文件写盘失败: {exc}")
                    path = None
        summary = json.dumps(result.get("profile_summary", {}), ensure_ascii=False)
        band = result.get("valid_band_hz") or (None, None)
        self._send_calibration_message(
            "calibration_result",
            cal_id,
            status="solved",
            path=str(path) if path else "",
            format="uav_gpr_solt_calibration_v2",
            profile_summary=summary,
            valid_band_hz=f"{band[0]:g}-{band[1]:g}" if band[0] is not None else "",
        )

    def _measurements_from_result(self, result: dict[str, object]) -> dict[str, CalibrationMeasurement]:
        """解算结果 → 六步标准件均值（写 uav_gpr_solt_calibration_v2 用）。"""
        session = self._calibration_session
        assert session is not None
        measurements: dict[str, CalibrationMeasurement] = {}
        means = session.step_means()
        for name, mean in means.items():
            frequency = session.step_frequency(name)
            points = [
                (float(f), {"S11": complex(mean[0][index]), "S22": complex(mean[1][index])})
                for index, f in enumerate(frequency)
            ]
            measurements[name] = CalibrationMeasurement.from_sparameter_points(
                points, primary=OSL_STEP_CHANNEL[name]
            )
        return measurements

    def _send_step_quality(self, session: CalibrationSession, result: dict[str, object]) -> None:
        if session.plan.mode == CALIBRATION_MODE_BACKGROUND:
            progress = session.step_progress(BACKGROUND_STEP_NAME)
            captures = np.asarray(progress.captures, dtype=np.complex128)
            sample_count = int(captures.shape[0]) if captures.ndim == 3 else 0
            if sample_count >= 2 and captures.ndim == 3:
                mean = np.mean(captures, axis=0)
                errors = np.abs(captures - mean[np.newaxis, ...])
                repeat_rms = float(np.sqrt(np.mean(np.square(errors))))
                max_abs = float(np.max(errors))
            else:
                repeat_rms = None
                max_abs = None
            self._send_calibration_message(
                "calibration_quality",
                session.cal_id,
                step=BACKGROUND_STEP_NAME,
                repeat_rms=repeat_rms,
                max_abs_err=max_abs,
                sample_count=sample_count,
                note=None if sample_count >= 2 else "样本不足",
            )
            return
        p1: OSLTerms = result["p1_terms"]  # type: ignore[assignment]
        p2: OSLTerms = result["p2_terms"]  # type: ignore[assignment]
        for step_name in OSL_STEP_NAMES:
            port_terms = p1 if step_name.startswith("P1_") else p2
            label = step_name.split("_", 1)[1].lower()
            metrics = port_terms.quality_metrics
            self._send_calibration_message(
                "calibration_quality",
                session.cal_id,
                step=step_name,
                repeat_rms=metrics.get(f"{label}_repeat_rms_abs_error"),
                max_abs_err=metrics.get(f"{label}_repeat_max_abs_error"),
                fit_max_abs_error=metrics.get(f"{label}_fit_max_abs_error"),
                sample_count=metrics.get(f"{label}_sample_count"),
                note="样本不足" if metrics.get(f"{label}_insufficient_samples") else None,
            )

    def _command_calibration_activate(self, cal_id: str) -> None:
        with self._calibration_lock:
            session = self._calibration_session
            if session is None or session.cal_id != cal_id:
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected",
                    reason="无进行中的校准会话或 cal_id 不匹配（防跨会话错配）",
                )
                return
            try:
                activation = session.activate()
            except Exception as exc:  # noqa: BLE001 - 激活失败：原因回传
                self.log(f"校准激活失败: cal_id={cal_id}: {exc}")
                self._send_calibration_message(
                    "calibration_result", cal_id, status="failed", reason=str(exc)
                )
                return
            # ---- 接线：生效组件写回 MainWindow（R-C-16）----
            cal_path = self._active_calibration_path
            if session.plan.mode == CALIBRATION_MODE_OSL:
                p1_terms: OSLTerms = activation["p1_terms"]  # type: ignore[assignment]
                p2_terms: OSLTerms = activation["p2_terms"]  # type: ignore[assignment]
                self.session_calibration = terms_to_osl_calibration(p1_terms, cal_path)
                self.session_calibration_p2 = terms_to_osl_calibration(p2_terms, cal_path)
                self.cal_measurements = dict(self._measurements_from_result(session.solved_result() or {}))
                self.update_calibration_status(
                    f"远程校准已激活: cal_id={cal_id}（P1/P2 双反射 OSL，{len(p1_terms.frequency_hz)} 点）"
                )
            else:
                frequency = np.asarray(activation["background_frequency_hz"], dtype=np.float64)
                raw_mean = np.asarray(activation["background"], dtype=np.complex128)
                # R-C-19 域约束：背景保存为"当前生效 OSL 域"的 S11（与既有
                # capture_background 语义一致），绝不把单通道背景广播到双通道。
                calibration_p1, _p2, _through = self.active_calibration_components()
                if calibration_p1 is not None:
                    background = np.array(
                        [calibration_p1.correct(float(f), complex(v)) for f, v in zip(frequency, raw_mean)],
                        dtype=np.complex128,
                    )
                else:
                    background = raw_mean.copy()
                self.background = background
                self.background_frequency_hz = np.array(frequency, dtype=np.float64, copy=True)
                self.bg_label.setText(f"远程背景已激活: {len(background)} 点（cal_id={cal_id}）")
                self.log("远程空采背景已激活")
            self.active_calibration_id = cal_id
            summary = json.dumps(
                (session.solved_result() or {}).get("profile_summary", {}), ensure_ascii=False
            )
        self.log(f"校准已激活: cal_id={cal_id}")
        self._send_calibration_message(
            "calibration_result",
            cal_id,
            status="activated",
            profile_summary=summary,
            calibration_id=cal_id,
        )

    def _command_calibration_cancel(self, cal_id: str, reason: str) -> None:
        with self._calibration_lock:
            session = self._calibration_session
            if session is None or (cal_id and session.cal_id != cal_id):
                # R-C-18：重连后携带旧 cal_id 的命令被拒绝并记录日志。
                self.log(f"cal_cancel 拒绝: 无匹配会话（cal_id={cal_id!r}）")
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected",
                    reason="无匹配的校准会话",
                )
                return
            try:
                session.cancel(reason)
            except Exception as exc:  # noqa: BLE001
                self.log(f"校准取消失败: {exc}")
                return
            self._calibration_session = None
        self.log(f"校准会话已取消: cal_id={session.cal_id}（{reason}）；已收 capture 文件保留")
        self._send_calibration_message(
            "calibration_result", session.cal_id, status="cancelled", reason=reason
        )

    def _command_calibration_status(self, cal_id: str) -> None:
        """R-C-16：地面查询当前生效校准摘要。"""
        with self._calibration_lock:
            session = self._calibration_session
            active = session.active() if session is not None else None
        payload_fields: dict[str, object] = {
            "status": "active" if active else "none",
            "calibration_id": self.active_calibration_id or "",
        }
        if active:
            payload_fields["valid_band_hz"] = "{:g}-{:g}".format(*active["valid_band_hz"])
            payload_fields["mode"] = str(active.get("mode", ""))
        self._send_calibration_message("calibration_result", cal_id or "query", **payload_fields)

    def stop_acquisition(self) -> None:
        # 第2轮 R-A-05：操作员停止与地面端 stop 命令、队列硬上限受控停止共用
        # 服务统一入口（同一 reason 记录、同一状态迁移、同一状态消息序列）。
        if self.acquisition_service.request_stop("操作员停止按钮（空中端）"):
            self.log("正在停止采集...")
            return
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
        # 服务侧兜底登记：真实 worker 的 finally 已 finish_task（此处 no-op）；
        # 测试注入的替身不执行 run()/不继承本类，因此按 sender 身份匹配登记，
        # 避免服务永久停在 acquiring（用 duck-typing，不用 isinstance）。
        if sender is not None:
            self.acquisition_service.note_worker_finished(sender)
        self.worker = None
        self._set_task_controls_locked(False)
        # 第4轮可见化（t3 收紧了 continuous 态文案）：任务结束后立刻在状态区写明
        # 本轮引擎与下一次的行为——continuous 态写"待命，下次启动即用"（USB 保持
        # 待命、路径不变），**不再写"未启用"**（默认引擎就是连续，写"未启用"自相矛盾）；
        # legacy 态写明为什么慢（逐帧开关 USB，真机约 2.75 s/帧）。
        if self.continuous_enabled.isChecked():
            self.continuous_status_label.setText(CONTINUOUS_STATUS_STANDBY)
        else:
            self.continuous_status_label.setText(CONTINUOUS_STATUS_LEGACY_RUNTIME)
        self.refresh_engine_mode_label()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._closing = True
        if self._remote_connect_timer.isActive():
            self._remote_connect_timer.stop()
        # 第2轮 R-C-18：程序关闭时进行中的校准会话作废（capture 文件保留并
        # 标记 incomplete，绝不激活半套校准）。
        with self._calibration_lock:
            session = self._calibration_session
            if session is not None and session.state not in (
                CalibrationSession.STATE_FAILED,
                CalibrationSession.STATE_CANCELLED,
            ):
                try:
                    session.cancel("程序关闭")
                except Exception:  # noqa: BLE001 - 取消失败不阻止关闭流程
                    pass
                self._calibration_session = None
        # 先停采集线程并等待有界退出（USB 在 worker finally 中释放）；绝不
        # terminate 活线程。超时则忽略关闭事件，由用户稍后重试。
        worker = self.worker
        if worker is not None and worker.isRunning():
            # R-A-05：服务持有的任务必须经统一命令入口停止；只有非服务 worker
            # （legacy / 测试直接赋值）才退回 worker.stop()。
            if not self.acquisition_service.request_stop("程序关闭"):
                worker.stop()
            if not worker.wait(5000):
                self.log("采集线程未在 5s 内退出，请稍后重试关闭")
                event.ignore()
                return
        # 第2轮 R-A-07：只有程序关闭才真正释放 USB（服务有界释放，fail-closed
        # 不伪装成功：未释放则 ignore 并保留窗口，可重试关闭）。
        if not self.acquisition_service.shutdown(10000):
            self.log("USB 资源或采集线程尚未干净收尾，请稍后重试关闭")
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
        self.bscan_complex = {name: [] for name in UI_CHANNELS}
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
        traces_real = {
            name: (np.asarray(trace).real if np.iscomplexobj(trace) else trace)
            for name, trace in trace_by_channel.items()
        }
        try:
            candidate_time, candidate_traces = validate_bscan_trace(time_ns, traces_real)
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
        if not hasattr(self, "bscan_complex"):
            self.bscan_complex = {name: [] for name in UI_CHANNELS}
        for name in UI_CHANNELS:
            self.bscan[name].append(candidate_traces[name])
            self.bscan_complex[name].append(np.asarray(trace_by_channel[name], dtype=np.complex128))
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
        rows_c = getattr(self, "bscan_complex", {}).get(name, [])
        if rows_c and len(rows_c) == len(self.bscan.get(name, [])):
            matrix_c = np.column_stack([np.asarray(row, dtype=np.complex128) for row in rows_c])
            flat_window = self.flat_filter_window()
            if self.flat_filter_enabled.isChecked() and matrix_c.shape[1] >= flat_window:
                matrix_c = flat_reflection_filter(matrix_c, window_size=flat_window)
            return matrix_c.real
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
