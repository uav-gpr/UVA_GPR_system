import unittest

import numpy as np

from bscan_view import (
    BSCAN_WINDOW_TRACES,
    bscan_processing_signature,
    bscan_start_after_append,
    bscan_time_axes_compatible,
    bscan_window_bounds,
    bscan_window_max_start,
    select_bscan_window,
    validate_bscan_trace,
)
from gpr_processing import flat_reflection_filter


class BScanWindowTests(unittest.TestCase):
    def test_processing_signature_ignores_disabled_window_and_rejects_invalid_state(self) -> None:
        self.assertEqual(bscan_processing_signature(False, 1000.0), (False, None))
        self.assertEqual(bscan_processing_signature(True, 4.0), (True, 4.0))
        with self.assertRaises(TypeError):
            bscan_processing_signature(1, 4.0)
        with self.assertRaises(ValueError):
            bscan_processing_signature(True, np.inf)

    def test_window_ranges_for_requested_trace_counts(self) -> None:
        # 扩容 64→256（任务 t1）：窗口契约断言随 BSCAN_WINDOW_TRACES 同步更新，
        # 64 窗时代的 65→2 / 100→37 / 130→67 边界值改为不足一窗（→1），
        # 并补充 256/257 精确边界与 300/1000 大历史断言（0 删除，纯换算）。
        expected_max = {
            0: 1,
            1: 1,
            63: 1,
            64: 1,
            65: 1,
            100: 1,
            130: 1,
            256: 1,
            257: 2,
            300: 45,
            1000: 745,
        }
        for total, maximum in expected_max.items():
            self.assertEqual(bscan_window_max_start(total), maximum)
        self.assertEqual(bscan_window_bounds(65, 1), (1, 65))
        # 65 道不足一窗：请求起点 2 收敛到 max_start=1（64 窗时代为 (2,65)）。
        self.assertEqual(bscan_window_bounds(65, 2), (1, 65))
        self.assertEqual(bscan_window_bounds(256, 1), (1, 256))
        self.assertEqual(bscan_window_bounds(257, 1), (1, 256))
        self.assertEqual(bscan_window_bounds(257, 2), (2, 257))
        self.assertEqual(bscan_window_bounds(300, 45), (45, 300))
        # 100/130 道不足一窗：原起点 37/67 收敛到 max_start=1（64 窗时代保留）。
        self.assertEqual(bscan_window_bounds(100, 37), (1, 100))
        self.assertEqual(bscan_window_bounds(130, 67), (1, 130))

    def test_short_scan_is_padded_without_stretching_trace_positions(self) -> None:
        matrix = np.arange(2 * 63, dtype=float).reshape(2, 63)
        window = select_bscan_window(matrix)
        self.assertEqual(window.matrix.shape, (2, BSCAN_WINDOW_TRACES))
        np.testing.assert_array_equal(window.matrix[:, :63], matrix)
        # 扩容 64→256：63 道之后到窗口右缘（列 63..255）全部 NaN 填充。
        self.assertTrue(np.isnan(window.matrix[:, 63:]).all())
        self.assertEqual((window.start_trace, window.end_trace), (1, 63))

    def test_history_and_latest_following(self) -> None:
        # 扩容 64→256（任务 t1）：
        # 不足一窗（65 < 256）时追加不产生窗口滑动（max_start 保持 1）。
        self.assertEqual(bscan_start_after_append(1, 64, 65, True), 1)
        self.assertEqual(bscan_start_after_append(1, 64, 65, False), 1)
        # 跨过 256 道边界后 follow-latest 追随到 max_start(after)=2。
        self.assertEqual(bscan_start_after_append(1, 256, 257, True), 2)
        self.assertEqual(bscan_start_after_append(1, 256, 257, False), 2)
        # 历史足够长时 follow-latest 跳到新 max_start；非 follow 保持原起点。
        self.assertEqual(bscan_start_after_append(2, 257, 300, True), 45)
        self.assertEqual(bscan_start_after_append(1, 257, 300, False), 1)

    def test_same_start_gives_aligned_absolute_trace_slice_for_hh_and_vv(self) -> None:
        # 扩容 64→256（任务 t1）：100 道在 256 窗下不足一窗（起点会被收敛到 1），
        # 改用 300 道历史（max_start=45）保持"同起点 → HH/VV 绝对道对齐"意图。
        hh = np.arange(3 * 300, dtype=float).reshape(3, 300)
        vv = hh + 10000.0
        hh_window = select_bscan_window(hh, 37)
        vv_window = select_bscan_window(vv, 37)
        self.assertEqual(hh_window.start_trace, vv_window.start_trace)
        self.assertEqual(hh_window.end_trace, vv_window.end_trace)
        np.testing.assert_array_equal(hh_window.matrix[:, 0], hh[:, 36])
        np.testing.assert_array_equal(vv_window.matrix[:, 0], vv[:, 36])
        np.testing.assert_array_equal(vv_window.matrix - hh_window.matrix, 10000.0)

    def test_flat_filter_operates_along_trace_axis(self) -> None:
        data = np.array(
            [
                [1.0, 1.0, 4.0, 1.0, 1.0],
                [2.0, 2.0, 5.0, 2.0, 2.0],
                [3.0, 3.0, 6.0, 3.0, 3.0],
            ]
        )
        filtered = flat_reflection_filter(data, window_size=3)
        self.assertEqual(filtered.shape, data.shape)
        self.assertGreater(abs(filtered[0, 2]), abs(filtered[0, 0]))
        np.testing.assert_allclose(filtered[:, 0], 0.0)

    def test_window_indices_require_real_integer_types(self) -> None:
        for value in (True, 1.5, "1"):
            with self.assertRaises(TypeError):
                bscan_window_max_start(value)
            with self.assertRaises(TypeError):
                bscan_window_bounds(10, value)
        self.assertEqual(bscan_window_max_start(np.int64(257)), 2)
        # 65 道不足一窗：起点 2 收敛到 1；257 道时起点 2 合法保留。
        self.assertEqual(bscan_window_bounds(np.int64(65), np.int64(2)), (1, 65))
        self.assertEqual(bscan_window_bounds(np.int64(257), np.int64(2)), (2, 257))

    def test_trace_validation_and_time_axis_compatibility(self) -> None:
        time_ns = np.array([0.0, 2.0, 4.0, 6.0])
        traces = {"S11": np.ones(4), "S22": np.full(4, 2.0)}
        validated_time, validated_traces = validate_bscan_trace(time_ns, traces)
        np.testing.assert_array_equal(validated_time, time_ns)
        np.testing.assert_array_equal(validated_traces["S22"], traces["S22"])
        self.assertTrue(bscan_time_axes_compatible(time_ns, time_ns + 1e-10))
        self.assertFalse(bscan_time_axes_compatible(time_ns, time_ns + 1e-3))
        for invalid_time in (
            np.array([]),
            np.array([[0.0, 2.0]]),
            np.array([0.0, np.nan, 4.0, 6.0]),
            np.array([0.0, 2.0, np.inf, 6.0]),
        ):
            with self.assertRaises(ValueError):
                validate_bscan_trace(invalid_time, traces)
        for invalid_traces in (
            {"S11": np.ones(4), "S22": np.ones(3)},
            {"S11": np.ones(4), "S22": np.array([[1.0, 2.0, 3.0, 4.0]])},
            {"S11": np.ones(4), "S22": np.array([1.0, np.nan, 3.0, 4.0])},
            {"S11": np.ones(4)},
        ):
            with self.assertRaises(ValueError):
                validate_bscan_trace(time_ns, invalid_traces)


if __name__ == "__main__":
    unittest.main()
