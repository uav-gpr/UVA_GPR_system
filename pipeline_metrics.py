"""轻量级流水线性能统计（第1轮检查点A，roadmap 实时链路改造）。

纯 Python、无 Qt 依赖，空中端与地面端均可使用。记录各阶段耗时样本与
队列深度，用于定位"采集→地面显示"端到端延迟。

设计约束：

- 同一进程内耗时用 :func:`time.perf_counter`；跨进程端到端时间用 UTC
  （依赖两端系统时钟同步，调用方需自行注明）。
- 样本保存于有限环形缓冲（默认 4096），不会无限增长。
- 所有公共方法线程安全且绝不抛出采集异常：统计失败不得影响采集线程。
- 队列深度/峰值通过 :class:`QueueMetrics` 或直接 ``record_queue`` 记录。

用法::

    metrics = PipelineMetrics()
    metrics.record_stage("processing_completed", 0.0123)   # 秒
    metrics.record_queue("processing_queue", depth=3, peak=9)
    print(metrics.stage_report("processing_completed"))
    print(metrics.snapshot())
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Final

#: 默认每个阶段保留的最大延迟样本数（有限样本窗口）。
DEFAULT_MAX_SAMPLES: Final[int] = 4096

_KNOWN_STAGES: Final[tuple[str, ...]] = (
    "sweep_completed",
    "processing_started",
    "processing_completed",
    "local_save_completed",
    "remote_enqueued",
    "remote_send_started",
    "remote_ack_received",
    "ground_received",
    "ground_saved",
    "ground_decoded",
    "ground_display_enqueued",
    "ground_displayed",
)


def _percentile(sorted_values: list[float], fraction: float) -> float:
    """已排序样本的 R-7 线性插值百分位（len>=1）。"""
    index = fraction * (len(sorted_values) - 1)
    low = int(math.floor(index))
    high = int(math.ceil(index))
    if low == high:
        return sorted_values[low]
    weight = index - low
    return sorted_values[low] * (1.0 - weight) + sorted_values[high] * weight


@dataclass
class QueueMetrics:
    """单个队列的深度/峰值/积压年龄统计（线程安全由持有方锁保护）。"""

    depth: int = 0
    peak: int = 0
    total_put: int = 0
    total_get: int = 0
    high_water_events: int = 0
    oldest_age_ms: float | None = None

    def as_dict(self) -> dict[str, object]:
        return {
            "depth": self.depth,
            "peak": self.peak,
            "total_put": self.total_put,
            "total_get": self.total_get,
            "high_water_events": self.high_water_events,
            "oldest_age_ms": self.oldest_age_ms,
        }


class _StageStats:
    """单个阶段的有限样本统计（内部，锁外不得访问）。"""

    __slots__ = ("samples", "max_samples")

    def __init__(self, max_samples: int) -> None:
        self.samples: list[float] = []
        self.max_samples = max_samples

    def add(self, value_s: float) -> None:
        self.samples.append(value_s)
        if len(self.samples) > self.max_samples:
            # 环形缓冲语义：丢最旧样本，保持窗口有限。
            del self.samples[0 : len(self.samples) - self.max_samples]

    def report(self) -> dict[str, float | int | None]:
        # latest 必须是**真正最后加入**的样本（插入序），而非排序后的最大值
        # （复审修复：旧实现 latest==max，掩盖了时序信息）。
        latest = self.samples[-1] if self.samples else None
        values = sorted(self.samples)
        if not values:
            return {"count": 0, "latest": None, "mean": None, "p50": None, "p95": None, "max": None}
        return {
            "count": len(values),
            "latest": latest,
            "mean": sum(values) / len(values),
            "p50": _percentile(values, 0.50),
            "p95": _percentile(values, 0.95),
            "max": values[-1],
        }


class PipelineMetrics:
    """线程安全的阶段耗时/队列深度统计。

    所有方法吞掉自身异常（除参数类型错误在开发期尽早暴露的
    ``record_stage`` 非有限值拒绝），保证统计故障不影响采集线程。
    """

    def __init__(self, *, max_samples: int = DEFAULT_MAX_SAMPLES, clock=None) -> None:
        if max_samples <= 0:
            raise ValueError("max_samples 必须为正整数")
        self._max_samples = int(max_samples)
        self._clock = clock if clock is not None else time.perf_counter
        self._lock = threading.Lock()
        self._stages: dict[str, _StageStats] = {}
        self._queues: dict[str, QueueMetrics] = {}

    # ---- 阶段耗时 ----

    def record_stage(self, stage: str, duration_s: float) -> bool:
        """记录一次阶段耗时（秒）。非有限/负值被安全拒绝并返回 False。

        返回值仅用于测试/诊断；调用方（采集线程）无需检查。
        """
        try:
            if not isinstance(stage, str) or not stage:
                return False
            value = float(duration_s)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(value) or value < 0.0:
            return False
        try:
            with self._lock:
                stats = self._stages.get(stage)
                if stats is None:
                    stats = _StageStats(self._max_samples)
                    self._stages[stage] = stats
                stats.add(value)
        except Exception:  # noqa: BLE001 - 统计异常不得影响采集线程
            return False
        return True

    def measure(self, stage: str):
        """上下文管理器：with 块耗时自动记录（异常也记录，便于诊断）。"""
        return _StageMeasure(self, stage)

    def stage_report(self, stage: str) -> dict[str, float | int | None]:
        """单阶段统计快照（count/latest/mean/p50/p95/max）。"""
        with self._lock:
            stats = self._stages.get(stage)
            report = stats.report() if stats is not None else None
        if report is None:
            return {"count": 0, "latest": None, "mean": None, "p50": None, "p95": None, "max": None}
        return report

    # ---- 队列深度 ----

    def record_queue(
        self,
        name: str,
        *,
        depth: int,
        peak: int | None = None,
        oldest_age_ms: float | None = None,
        high_water: bool = False,
    ) -> None:
        """更新队列深度快照（非 put 计数：本方法只记录观测到的深度）。

        ``peak`` 缺省时按历史峰值自增；``total_put``/``total_get`` 属于真实
        队列自身的计数，不由快照推断（复审修复：旧实现把"记录一次快照"
        错误计为 total_put）。真实 put/get 请由队列本身统计并传入。
        """
        try:
            with self._lock:
                metrics = self._queues.get(name)
                if metrics is None:
                    metrics = QueueMetrics()
                    self._queues[name] = metrics
                metrics.depth = int(depth)
                if peak is None:
                    peak = max(metrics.peak, int(depth))
                metrics.peak = max(metrics.peak, int(peak))
                metrics.oldest_age_ms = (
                    float(oldest_age_ms) if oldest_age_ms is not None else None
                )
                if high_water:
                    metrics.high_water_events += 1
        except Exception:  # noqa: BLE001
            pass

    def queue_report(self, name: str) -> dict[str, object]:
        with self._lock:
            metrics = self._queues.get(name)
            return dict(metrics.as_dict()) if metrics is not None else QueueMetrics().as_dict()

    # ---- 汇总 ----

    def snapshot(self) -> dict[str, object]:
        """全部统计的 JSON 可序列化快照（诊断入口使用）。"""
        with self._lock:
            stages = {name: stats.report() for name, stats in sorted(self._stages.items())}
            queues = {name: dict(m.as_dict()) for name, m in sorted(self._queues.items())}
        return {
            "stages": stages,
            "queues": queues,
            "known_stages": list(_KNOWN_STAGES),
            "max_samples": self._max_samples,
        }

    def reset(self) -> None:
        with self._lock:
            self._stages.clear()
            self._queues.clear()


class _StageMeasure:
    """:meth:`PipelineMetrics.measure` 的上下文管理器。"""

    __slots__ = ("_metrics", "_stage", "_start")

    def __init__(self, metrics: PipelineMetrics, stage: str) -> None:
        self._metrics = metrics
        self._stage = stage
        self._start = 0.0

    def __enter__(self) -> "_StageMeasure":
        self._start = self._metrics._clock()
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        try:
            self._metrics.record_stage(self._stage, self._metrics._clock() - self._start)
        except Exception:  # noqa: BLE001
            pass
        return False  # 不吞业务异常


__all__ = [
    "DEFAULT_MAX_SAMPLES",
    "PipelineMetrics",
    "QueueMetrics",
]
