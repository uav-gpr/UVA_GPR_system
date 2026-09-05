"""FixedIntervalSweepSelector 单元测试（roadmap §15.4：定时选择）。

纯 Python，不依赖 Qt / PyUSB / 硬件。使用注入时钟验证：

- interval=0 选择每条完整 sweep；
- interval>0 第一条立即选择；
- 固定 monotonic deadline，无 ``now + interval`` 累计漂移；
- 跨过多个 deadline 只选择当前一条并统计 missed slots；
- 实际间隔与调度误差统计（mean/min/p95/max）；
- reset / 参数校验。
"""

from __future__ import annotations

import unittest

from acquisition_scheduler import FixedIntervalSweepSelector, SelectionResult


class FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def advance(self, delta: float) -> None:
        self.t += delta


def _selector(interval_s: float, clock: FakeClock) -> FixedIntervalSweepSelector:
    return FixedIntervalSweepSelector(interval_s, clock=clock)


class IntervalValidationTests(unittest.TestCase):
    def test_interval_zero_accepted(self) -> None:
        sel = _selector(0.0, FakeClock())
        self.assertEqual(sel.interval_s, 0.0)

    def test_interval_positive_accepted(self) -> None:
        self.assertEqual(_selector(0.5, FakeClock()).interval_s, 0.5)

    def test_negative_interval_rejected(self) -> None:
        with self.assertRaises(ValueError):
            _selector(-0.1, FakeClock())

    def test_nan_interval_rejected(self) -> None:
        with self.assertRaises(ValueError):
            _selector(float("nan"), FakeClock())

    def test_inf_interval_rejected(self) -> None:
        with self.assertRaises(ValueError):
            _selector(float("inf"), FakeClock())

    def test_bool_interval_rejected(self) -> None:
        with self.assertRaises(TypeError):
            _selector(True, FakeClock())

    def test_string_interval_rejected(self) -> None:
        with self.assertRaises(TypeError):
            _selector("1.0", FakeClock())

    def test_nonfinite_sweep_time_rejected(self) -> None:
        clock = FakeClock()
        sel = _selector(1.0, clock)
        with self.assertRaises(ValueError):
            sel.should_select(float("nan"))


class IntervalZeroTests(unittest.TestCase):
    def test_selects_every_sweep(self) -> None:
        clock = FakeClock()
        sel = _selector(0.0, clock)
        results: list[SelectionResult] = []
        for delta in (0.1, 0.2, 0.3):
            clock.advance(delta)
            results.append(sel.should_select())
        self.assertTrue(all(r.select for r in results))
        self.assertEqual(sel.selected_sweeps, 3)
        self.assertEqual(sel.skipped_sweeps, 0)
        self.assertEqual(sel.missed_slots_total, 0)
        self.assertEqual(sel.total_sweeps_seen, 3)

    def test_actual_interval_between_consecutive_sweeps(self) -> None:
        clock = FakeClock()
        sel = _selector(0.0, clock)
        clock.advance(0.1)
        first = sel.should_select()
        self.assertIsNone(first.actual_interval_s)
        clock.advance(0.5)
        second = sel.should_select()
        self.assertIsNotNone(second.actual_interval_s)
        self.assertAlmostEqual(second.actual_interval_s, 0.5)

    def test_statistics_interval_samples(self) -> None:
        clock = FakeClock()
        sel = _selector(0.0, clock)
        for delta in (0.1, 0.2, 0.1):
            clock.advance(delta)
            sel.should_select()
        stats = sel.statistics
        intervals = stats["actual_interval_s"]
        self.assertEqual(intervals["count"], 2)
        self.assertAlmostEqual(intervals["min"], 0.1)
        self.assertAlmostEqual(intervals["max"], 0.2)
        self.assertAlmostEqual(intervals["mean"], 0.15)


class FirstSweepImmediateTests(unittest.TestCase):
    def test_first_sweep_selected_immediately(self) -> None:
        clock = FakeClock()
        clock.t = 100.0
        sel = _selector(5.0, clock)
        result = sel.should_select()
        self.assertTrue(result.select)
        self.assertEqual(result.missed_slots, 0)
        self.assertIsNone(result.actual_interval_s)
        self.assertIsNotNone(result.next_due_s)
        self.assertAlmostEqual(result.next_due_s, 105.0)

    def test_second_sweep_before_deadline_skipped(self) -> None:
        clock = FakeClock()
        clock.t = 100.0
        sel = _selector(5.0, clock)
        sel.should_select()  # first selected immediately
        clock.advance(2.0)   # 102 < 105
        result = sel.should_select()
        self.assertFalse(result.select)
        self.assertEqual(result.missed_slots, 0)
        self.assertEqual(sel.skipped_sweeps, 1)
        self.assertEqual(sel.selected_sweeps, 1)

    def test_second_sweep_at_deadline_selected_no_miss(self) -> None:
        clock = FakeClock()
        clock.t = 100.0
        sel = _selector(5.0, clock)
        sel.should_select()
        clock.advance(5.0)   # exactly 105
        result = sel.should_select()
        self.assertTrue(result.select)
        self.assertEqual(result.missed_slots, 0)
        self.assertAlmostEqual(result.actual_interval_s, 5.0)
        self.assertAlmostEqual(result.schedule_error_ms, 0.0, places=6)


class FixedDeadlineNoDriftTests(unittest.TestCase):
    def test_deadlines_fixed_no_cumulative_drift(self) -> None:
        """即便处理较慢导致每次选择都迟一点，deadline 也不随 now 漂移。"""
        clock = FakeClock()
        clock.t = 0.0
        sel = _selector(1.0, clock)
        sel.should_select()  # t=0, next_due=1
        due: list[float] = []
        for advance in (1.1, 1.1, 1.1):
            clock.advance(advance)
            result = sel.should_select()
            self.assertTrue(result.select)
            due.append(result.next_due_s)
        # deadline 恒为整数：2, 3, 4，而不是 1.1, 2.2, 3.3。
        self.assertAlmostEqual(due[0], 2.0)
        self.assertAlmostEqual(due[1], 3.0)
        self.assertAlmostEqual(due[2], 4.0)
        self.assertEqual(sel.missed_slots_total, 0)
        # 每次实际间隔反映真实时间差，不受漂移影响。
        stats = sel.statistics
        self.assertAlmostEqual(stats["actual_interval_s"]["min"], 1.1)
        self.assertAlmostEqual(stats["actual_interval_s"]["max"], 1.1)
        self.assertAlmostEqual(stats["actual_interval_s"]["mean"], 1.1)


class MissedSlotsTests(unittest.TestCase):
    def test_crossing_multiple_deadlines_selects_once_and_counts_missed(self) -> None:
        clock = FakeClock()
        clock.t = 0.0
        sel = _selector(1.0, clock)
        sel.should_select()  # t=0
        clock.advance(3.5)   # 跨过 t=1,2,3 共三个 deadline
        result = sel.should_select()
        self.assertTrue(result.select)
        self.assertEqual(result.missed_slots, 2)
        self.assertEqual(sel.missed_slots_total, 2)
        self.assertEqual(sel.selected_sweeps, 2)
        self.assertAlmostEqual(result.schedule_error_ms, 500.0)  # 3.5 vs deadline 3.0

    def test_no_double_count_on_subsequent_regular_sweep(self) -> None:
        clock = FakeClock()
        clock.t = 0.0
        sel = _selector(1.0, clock)
        sel.should_select()  # t=0
        clock.advance(3.5)
        sel.should_select()  # missed=2
        clock.advance(0.6)   # t=4.1, deadline=4.0, 正常跨 1 个
        result = sel.should_select()
        self.assertTrue(result.select)
        self.assertEqual(result.missed_slots, 0)
        self.assertEqual(sel.missed_slots_total, 2)
        self.assertAlmostEqual(result.schedule_error_ms, 100.0)

    def test_skipped_sweeps_never_selected_again(self) -> None:
        clock = FakeClock()
        clock.t = 0.0
        sel = _selector(1.0, clock)
        sel.should_select()
        clock.advance(0.5)
        self.assertFalse(sel.should_select().select)  # skipped
        clock.advance(0.5)  # t=1.0, 到达 deadline
        result = sel.should_select()
        self.assertTrue(result.select)
        self.assertEqual(result.missed_slots, 0)
        self.assertEqual(sel.skipped_sweeps, 1)
        self.assertEqual(sel.selected_sweeps, 2)


class ScheduleErrorTests(unittest.TestCase):
    def test_schedule_error_positive_when_late(self) -> None:
        clock = FakeClock()
        clock.t = 0.0
        sel = _selector(1.0, clock)
        sel.should_select()
        clock.advance(1.2)
        result = sel.should_select()
        self.assertAlmostEqual(result.schedule_error_ms, 200.0)

    def test_schedule_error_zero_on_time(self) -> None:
        clock = FakeClock()
        clock.t = 0.0
        sel = _selector(2.0, clock)
        sel.should_select()
        clock.advance(2.0)
        self.assertAlmostEqual(sel.should_select().schedule_error_ms, 0.0, places=6)

    def test_error_statistics(self) -> None:
        clock = FakeClock()
        clock.t = 0.0
        sel = _selector(1.0, clock)
        sel.should_select()
        clock.advance(1.0)
        sel.should_select()  # error 0
        clock.advance(1.5)
        sel.should_select()  # error 500
        stats = sel.statistics["schedule_error_ms"]
        self.assertEqual(stats["count"], 2)
        self.assertAlmostEqual(stats["min"], 0.0)
        self.assertAlmostEqual(stats["max"], 500.0)
        self.assertAlmostEqual(stats["mean"], 250.0)


class ResetTests(unittest.TestCase):
    def test_reset_restarts_schedule_and_stats(self) -> None:
        clock = FakeClock()
        clock.t = 0.0
        sel = _selector(1.0, clock)
        sel.should_select()
        clock.advance(0.5)
        sel.should_select()
        self.assertEqual(sel.total_sweeps_seen, 2)
        sel.reset()
        self.assertEqual(sel.total_sweeps_seen, 0)
        self.assertEqual(sel.selected_sweeps, 0)
        self.assertEqual(sel.missed_slots_total, 0)
        # 重置后第一条再次立即选择，重新建立 next_due。
        clock.advance(3.0)
        result = sel.should_select()
        self.assertTrue(result.select)
        self.assertEqual(result.missed_slots, 0)
        self.assertIsNone(result.actual_interval_s)
        self.assertAlmostEqual(result.next_due_s, clock.t + 1.0)


class RestartTimelineTests(unittest.TestCase):
    """区域六：restart_timeline 只重启时间基准，保留会话累计统计。"""

    def test_restart_preserves_selected_skipped_and_missed(self) -> None:
        clock = FakeClock()
        clock.t = 0.0
        sel = _selector(1.0, clock)
        sel.should_select()  # 第一条立即选择
        clock.advance(0.5)
        self.assertFalse(sel.should_select().select)  # skip
        clock.advance(1.0)
        self.assertTrue(sel.should_select().select)  # select（t=1.5）
        self.assertEqual(sel.selected_sweeps, 2)
        self.assertEqual(sel.skipped_sweeps, 1)
        self.assertEqual(sel.missed_slots_total, 0)
        sel.restart_timeline()
        # 会话累计统计保留。
        self.assertEqual(sel.selected_sweeps, 2)
        self.assertEqual(sel.skipped_sweeps, 1)
        self.assertEqual(sel.missed_slots_total, 0)
        self.assertIsNone(sel._first_time_s)
        self.assertIsNone(sel._next_due_s)
        self.assertIsNone(sel._prev_selected_s)

    def test_restart_then_first_sweep_selected_immediately(self) -> None:
        clock = FakeClock()
        clock.t = 0.0
        sel = _selector(2.0, clock)
        sel.should_select()  # select at t=0
        sel.restart_timeline()
        clock.advance(100.0)  # 暂停 100s
        result = sel.should_select()
        self.assertTrue(result.select, "restart 后第一条立即选择")
        self.assertEqual(result.missed_slots, 0, "暂停不计 missed")
        self.assertIsNone(result.actual_interval_s, "暂停时长不计入实际间隔")
        self.assertAlmostEqual(result.next_due_s, clock.t + 2.0)
        self.assertEqual(sel.selected_sweeps, 2)

    def test_restart_does_not_discard_interval_samples(self) -> None:
        clock = FakeClock()
        clock.t = 0.0
        sel = _selector(1.0, clock)
        sel.should_select()
        clock.advance(1.0)
        sel.should_select()  # actual=1.0，样本 1 条
        self.assertEqual(sel.statistics["actual_interval_s"]["count"], 1)
        sel.restart_timeline()
        self.assertEqual(sel.statistics["actual_interval_s"]["count"], 1, "样本不得丢弃")
        clock.advance(50.0)  # 暂停
        sel.should_select()  # actual=None（prev 清空），不新增样本
        self.assertEqual(sel.statistics["actual_interval_s"]["count"], 1)
        clock.advance(1.0)
        sel.should_select()  # actual=1.0（从恢复点起）
        self.assertEqual(sel.statistics["actual_interval_s"]["count"], 2)

    def test_reset_vs_restart_are_distinct(self) -> None:
        clock = FakeClock()
        clock.t = 0.0
        sel = _selector(1.0, clock)
        sel.should_select()
        sel.reset()
        self.assertEqual(sel.selected_sweeps, 0)
        sel.should_select()
        sel.restart_timeline()
        self.assertEqual(sel.selected_sweeps, 1, "restart 保留累计，reset 清零")


class DefaultClockTests(unittest.TestCase):
    def test_default_clock_is_monotonic(self) -> None:
        import time

        sel = FixedIntervalSweepSelector(0.0)
        t0 = sel._clock()
        self.assertTrue(sel._clock() >= t0)
        self.assertIs(sel._clock, time.monotonic)


if __name__ == "__main__":
    unittest.main()
