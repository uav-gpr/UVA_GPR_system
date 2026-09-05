"""第1轮检查点B：B-scan 窗口纯函数与 Flat Filter 窗口化测试（无 Qt）。"""

from __future__ import annotations

import unittest

import numpy as np

from bscan_view import (
    BSCAN_WINDOW_TRACES,
    apply_flat_filter_windowed,
    bscan_window_from_rows,
    flat_filter_window_bounds,
)
from gpr_processing import flat_reflection_filter


def _rows(n_traces: int, n_samples: int, seed: int = 7) -> list[np.ndarray]:
    rng = np.random.default_rng(seed)
    return [rng.standard_normal(n_samples) for _ in range(n_traces)]


class BScanWindowFromRowsTests(unittest.TestCase):
    def test_various_trace_counts(self) -> None:
        # 传入极大 start_trace：自动收敛到 max_start（follow-latest 窗口语义）。
        for n_traces, expected_start in ((0, 1), (1, 1), (63, 1), (64, 1), (65, 2), (100, 37), (1000, 937)):
            rows = _rows(n_traces, 5)
            window = bscan_window_from_rows(rows, 10**9, n_samples=5)
            self.assertEqual(window.total_traces, n_traces)
            self.assertEqual(window.matrix.shape, (5, BSCAN_WINDOW_TRACES))
            self.assertEqual(window.start_trace, expected_start)
            count = min(n_traces, BSCAN_WINDOW_TRACES)
            if count:
                np.testing.assert_allclose(
                    window.matrix[:, :count], np.column_stack(rows[-count:])
                )
            if count < BSCAN_WINDOW_TRACES:
                self.assertTrue(np.all(np.isnan(window.matrix[:, count:])), f"n={n_traces} 应 NaN 填充")

    def test_arbitrary_time_samples(self) -> None:
        for n_samples in (1, 3, 2048, 5000):
            rows = _rows(70, n_samples)
            window = bscan_window_from_rows(rows, 1)
            self.assertEqual(window.matrix.shape, (n_samples, BSCAN_WINDOW_TRACES))

    def test_explicit_n_samples(self) -> None:
        rows = _rows(10, 8)
        window = bscan_window_from_rows(rows, 1, n_samples=8)
        self.assertEqual(window.matrix.shape, (8, BSCAN_WINDOW_TRACES))
        # 与行长度不一致的显式 n_samples 必须拒绝。
        with self.assertRaises(ValueError):
            bscan_window_from_rows(rows, 1, n_samples=4)

    def test_s11_s22_same_window(self) -> None:
        rows_a = _rows(100, 16, seed=1)
        rows_b = _rows(100, 16, seed=2)
        wa = bscan_window_from_rows(rows_a, 5)
        wb = bscan_window_from_rows(rows_b, 5)
        self.assertEqual(wa.start_trace, wb.start_trace)
        self.assertEqual(wa.end_trace, wb.end_trace)

    def test_does_not_mutate_history(self) -> None:
        rows = _rows(10, 4)
        original = [row.copy() for row in rows]
        bscan_window_from_rows(rows, 1)
        for row, old in zip(rows, original):
            np.testing.assert_array_equal(row, old)

    def test_cost_not_linear_in_total_history(self) -> None:
        """1000 道历史的窗口构造不应近似按总道数线性增长。"""
        import time

        small = _rows(70, 512)
        large = _rows(1000, 512)
        t0 = time.perf_counter()
        for _ in range(50):
            bscan_window_from_rows(small, 1)
        t_small = time.perf_counter() - t0
        t0 = time.perf_counter()
        for _ in range(50):
            bscan_window_from_rows(large, 1)
        t_large = time.perf_counter() - t0
        # 14 倍数据量下窗口构造时间应基本持平（允许有限常数差异）。
        self.assertLess(t_large, max(t_small * 4.0, t_small + 0.2))


class FlatFilterWindowedTests(unittest.TestCase):
    def test_bounds_clamped(self) -> None:
        # 可见窗口 1..64 + half=15 邻域 → [1, 79]。
        self.assertEqual(flat_filter_window_bounds(1000, 1, 31), (1, 79))
        # 937..1000 可见 → [922, 1000]。
        self.assertEqual(flat_filter_window_bounds(1000, 937, 31), (922, 1000))
        self.assertEqual(flat_filter_window_bounds(50, 1, 31), (1, 50))

    def test_elementwise_equal_to_full_history_filter(self) -> None:
        """窗口化滤波与全历史滤波逐元素一致（可见 64 道范围）。"""
        for total, start, window in ((70, 1, 31), (200, 50, 31), (1000, 937, 31), (100, 10, 95)):
            rows = _rows(total, 8, seed=3)
            full = np.column_stack(rows)
            filtered_full = flat_reflection_filter(full, window_size=window)
            windowed, applied = apply_flat_filter_windowed(rows, start, window)
            self.assertTrue(applied, f"total={total} window={window} 应执行滤波")
            vis_start, vis_end = start, min(total, start + BSCAN_WINDOW_TRACES - 1)
            expected = filtered_full[:, vis_start - 1 : vis_end]
            got = windowed[:, : vis_end - vis_start + 1]
            np.testing.assert_allclose(got, expected, rtol=1e-12, atol=1e-12)

    def test_insufficient_history_returns_unfiltered(self) -> None:
        rows = _rows(20, 8)
        window, applied = apply_flat_filter_windowed(rows, 1, 31)
        self.assertFalse(applied)
        expected = bscan_window_from_rows(rows, 1).matrix
        np.testing.assert_array_equal(window, expected)

    def test_even_window_rejected(self) -> None:
        rows = _rows(70, 8)
        with self.assertRaises(ValueError):
            apply_flat_filter_windowed(rows, 1, 30)

    def test_follow_latest_window_semantics(self) -> None:
        rows = _rows(65, 4)
        window = bscan_window_from_rows(rows, 2)
        self.assertEqual((window.start_trace, window.end_trace), (2, 65))
        window = bscan_window_from_rows(rows, 100)  # 越界收敛到最大起点
        self.assertEqual((window.start_trace, window.end_trace), (2, 65))


if __name__ == "__main__":
    unittest.main()
