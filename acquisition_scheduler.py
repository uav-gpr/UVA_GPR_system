"""应用层固定间隔时间选择（roadmap §9.2、§14）。

纯 Python，不依赖 Qt、PyUSB 或硬件。实现
:class:`FixedIntervalSweepSelector`：USB 工作线程持续产出完整 sweep，
应用层按固定 monotonic deadline 选择需要处理/保存/发送的 sweep。

规则（roadmap §9.2，逐字保持一致）：

- ``interval == 0``：选择每条有效 sweep（不抽样）。
- ``interval > 0``：第一条有效 sweep 立即选择；
  ``next_due = first_sweep_time + interval``；后续选择第一条
  ``acquisition_time >= next_due`` 的完整 sweep。
- ``next_due`` 按固定周期递增，**禁止**用 ``now + interval`` 造成累计漂移。
- 一次跨过多个 deadline 时只选择当前一条 sweep，并把错过的槽位计入
  ``missed_slots_total``（不重复保存同一 sweep）。

统计（roadmap §14）：selected / skipped / missed 槽位、实际采集间隔
mean/min/max、调度误差 mean/min/max。时钟可通过 ``clock`` 注入便于测试，
默认使用 :func:`time.monotonic`。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Final, Optional

Clock: Final = Callable[[], float]


def _check_interval(interval_s: object) -> float:
    """校验 interval_s：有限非负（0 表示逐 sweep），拒绝 bool/NaN/Inf/负。"""
    if isinstance(interval_s, bool) or not isinstance(interval_s, (int, float)):
        raise TypeError(
            f"interval_s 必须是有限非负数（bool 被拒绝），实际得到 {type(interval_s).__name__}"
        )
    value = float(interval_s)
    if not math.isfinite(value):
        raise ValueError(f"interval_s 必须是有限值（不允许 NaN/Infinity）：{interval_s!r}")
    if value < 0.0:
        raise ValueError(f"interval_s 必须 >= 0（0 表示逐 sweep 选择）：{interval_s!r}")
    return value


def _percentile(sorted_values: list[float], fraction: float) -> float:
    """已排序样本的近似百分位（R-7 线性插值，len>=1）。"""
    if not sorted_values:
        raise ValueError("空样本无法计算百分位")
    index = fraction * (len(sorted_values) - 1)
    low = int(math.floor(index))
    high = int(math.ceil(index))
    if low == high:
        return sorted_values[low]
    weight = index - low
    return sorted_values[low] * (1.0 - weight) + sorted_values[high] * weight


@dataclass(frozen=True)
class SelectionResult:
    """一次 :meth:`FixedIntervalSweepSelector.should_select` 的结果。"""

    select: bool
    #: 本次选择跨过的、未被覆盖的时间槽数量（未被选择即 0）。
    missed_slots: int = 0
    #: 与上一条已选择 sweep 的实际间隔（第一条为 None）。
    actual_interval_s: Optional[float] = None
    #: 本次选择相对其对应 deadline 的调度误差（毫秒；interval=0/第一条为 None）。
    schedule_error_ms: Optional[float] = None
    #: 当前（更新后）的下一个 deadline（monotonic；interval=0 为 None）。
    next_due_s: Optional[float] = None


class FixedIntervalSweepSelector:
    """固定 monotonic deadline 的 sweep 时间选择器。

    用法：USB 线程对每条完整 sweep 调用一次 :meth:`should_select`
    （参数为该 sweep 的采集中点 host monotonic 时间）；返回
    :class:`SelectionResult`，``select=True`` 表示应处理/保存/发送该 sweep。

    ``reset()`` 清空时间表与统计；暂停/恢复边界可调用它以让恢复后的第一条
    sweep 立即重新开始新的固定周期。
    """

    #: 保留最近至多这么多条实际间隔/调度误差样本用于 p95 统计。
    _MAX_SAMPLES: Final[int] = 8192

    def __init__(self, interval_s: float, *, clock: Optional[Clock] = None) -> None:
        self.interval_s = _check_interval(interval_s)
        self._clock = clock if clock is not None else time.monotonic
        if not callable(self._clock):
            raise TypeError("clock 必须是可调用对象")
        # 时间表状态。
        self._first_time_s: Optional[float] = None
        self._next_due_s: Optional[float] = None
        self._prev_selected_s: Optional[float] = None
        # 累计统计。
        self.total_sweeps_seen: int = 0
        self.selected_sweeps: int = 0
        self.skipped_sweeps: int = 0
        self.missed_slots_total: int = 0
        self._interval_samples: list[float] = []
        self._error_samples: list[float] = []

    # ---- 时间选择 ----

    def should_select(self, sweep_time_s: Optional[float] = None) -> SelectionResult:
        """对一条完整 sweep 做选择判定。

        ``sweep_time_s`` 为该 sweep 的采集中点 host monotonic 时间；省略时使用
        ``self._clock()``。返回 :class:`SelectionResult`。跨过多个 deadline 时
        只选择当前这一条并累计 ``missed_slots_total``。
        """
        time_now = self._clock() if sweep_time_s is None else float(sweep_time_s)
        if not math.isfinite(time_now):
            raise ValueError(f"sweep_time_s 必须是有限值：{sweep_time_s!r}")
        self.total_sweeps_seen += 1
        if self.interval_s == 0.0:
            return self._select_every(time_now)
        return self._select_timed(time_now)

    def reset(self) -> None:
        """完整重置（新任务开始）：清时间表与全部统计。"""
        self._first_time_s = None
        self._next_due_s = None
        self._prev_selected_s = None
        self.total_sweeps_seen = 0
        self.selected_sweeps = 0
        self.skipped_sweeps = 0
        self.missed_slots_total = 0
        self._interval_samples = []
        self._error_samples = []

    def restart_timeline(self) -> None:
        """只重启时间基准（pause/resume 边界），**保留**会话累计统计。

        清 ``first_time`` / ``next_due`` / ``prev_selected``：恢复后第一条立即
        选择并重新建立固定周期。**不**清 selected / skipped / missed，不丢弃
        已收集的统计样本，暂停时长不会计入下一次 actual_interval（因为
        ``prev_selected`` 被清空，恢复后的第一条 actual_interval 为 None）。
        """
        self._first_time_s = None
        self._next_due_s = None
        self._prev_selected_s = None

    # ---- 统计快照 ----

    @property
    def statistics(self) -> dict[str, object]:
        """JSON 可序列化统计快照（roadmap §14：实际间隔与调度误差 mean/p95/min/max）。"""
        intervals = sorted(self._interval_samples)
        errors = sorted(self._error_samples)
        return {
            "interval_s": self.interval_s,
            "total_sweeps_seen": self.total_sweeps_seen,
            "selected_sweeps": self.selected_sweeps,
            "skipped_sweeps": self.skipped_sweeps,
            "missed_slots_total": self.missed_slots_total,
            "actual_interval_s": self._sample_stats(intervals),
            "schedule_error_ms": self._sample_stats(errors),
        }

    # ---- 内部 ----

    def _select_every(self, time_now: float) -> SelectionResult:
        """interval=0：选择每条完整 sweep，并记录相邻已选 sweep 的实际间隔。"""
        actual_interval = None
        if self._prev_selected_s is not None:
            actual_interval = time_now - self._prev_selected_s
            self._record_interval(actual_interval)
        self._prev_selected_s = time_now
        self.selected_sweeps += 1
        return SelectionResult(
            select=True,
            missed_slots=0,
            actual_interval_s=actual_interval,
            schedule_error_ms=None,
            next_due_s=None,
        )

    def _select_timed(self, time_now: float) -> SelectionResult:
        """interval>0：固定 deadline 抽样，跨多个 deadline 只选一条并计 missed。"""
        if self._first_time_s is None:
            # 第一条有效 sweep 立即选择；next_due 从此刻按固定周期递增。
            self._first_time_s = time_now
            self._next_due_s = time_now + self.interval_s
            self._prev_selected_s = time_now
            self.selected_sweeps += 1
            return SelectionResult(
                select=True,
                missed_slots=0,
                actual_interval_s=None,
                schedule_error_ms=None,
                next_due_s=self._next_due_s,
            )
        assert self._next_due_s is not None
        if time_now < self._next_due_s:
            self.skipped_sweeps += 1
            return SelectionResult(
                select=False,
                missed_slots=0,
                actual_interval_s=None,
                schedule_error_ms=None,
                next_due_s=self._next_due_s,
            )
        # 跨过 deadline：把 next_due 按固定周期推进到 time_now 之后，统计跨过数。
        crossed = 0
        while self._next_due_s <= time_now:
            self._next_due_s += self.interval_s
            crossed += 1
        # 本次选择只覆盖最后一个跨过的 deadline；前面 crossed-1 个槽位错过。
        missed = crossed - 1
        self.missed_slots_total += missed
        actual_interval = time_now - self._prev_selected_s if self._prev_selected_s is not None else None
        deadline = self._next_due_s - self.interval_s
        schedule_error_s = time_now - deadline
        if actual_interval is not None:
            self._record_interval(actual_interval)
        self._record_error(schedule_error_s * 1000.0)
        self._prev_selected_s = time_now
        self.selected_sweeps += 1
        return SelectionResult(
            select=True,
            missed_slots=missed,
            actual_interval_s=actual_interval,
            schedule_error_ms=schedule_error_s * 1000.0,
            next_due_s=self._next_due_s,
        )

    def _record_interval(self, value: float) -> None:
        self._interval_samples.append(value)
        if len(self._interval_samples) > self._MAX_SAMPLES:
            self._interval_samples.pop(0)

    def _record_error(self, value_ms: float) -> None:
        self._error_samples.append(value_ms)
        if len(self._error_samples) > self._MAX_SAMPLES:
            self._error_samples.pop(0)

    @staticmethod
    def _sample_stats(sorted_values: list[float]) -> dict[str, Optional[float]]:
        if not sorted_values:
            return {"count": 0, "min": None, "mean": None, "p95": None, "max": None}
        return {
            "count": len(sorted_values),
            "min": sorted_values[0],
            "mean": sum(sorted_values) / len(sorted_values),
            "p95": _percentile(sorted_values, 0.95),
            "max": sorted_values[-1],
        }


__all__ = [
    "FixedIntervalSweepSelector",
    "SelectionResult",
]
