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
        expected_max = {
            0: 1,
            1: 1,
            63: 1,
            64: 1,
            65: 2,
            100: 37,
            130: 67,
        }
        for total, maximum in expected_max.items():
            self.assertEqual(bscan_window_max_start(total), maximum)
        self.assertEqual(bscan_window_bounds(65, 1), (1, 64))
        self.assertEqual(bscan_window_bounds(65, 2), (2, 65))
        self.assertEqual(bscan_window_bounds(100, 37), (37, 100))
        self.assertEqual(bscan_window_bounds(130, 67), (67, 130))

    def test_short_scan_is_padded_without_stretching_trace_positions(self) -> None:
        matrix = np.arange(2 * 63, dtype=float).reshape(2, 63)
        window = select_bscan_window(matrix)
        self.assertEqual(window.matrix.shape, (2, BSCAN_WINDOW_TRACES))
        np.testing.assert_array_equal(window.matrix[:, :63], matrix)
        self.assertTrue(np.isnan(window.matrix[:, 63]).all())
        self.assertEqual((window.start_trace, window.end_trace), (1, 63))

    def test_history_and_latest_following(self) -> None:
        self.assertEqual(bscan_start_after_append(1, 64, 65, True), 2)
        self.assertEqual(bscan_start_after_append(1, 64, 65, False), 2)
        self.assertEqual(bscan_start_after_append(2, 65, 100, True), 37)
        self.assertEqual(bscan_start_after_append(1, 65, 100, False), 1)

    def test_same_start_gives_aligned_absolute_trace_slice_for_hh_and_vv(self) -> None:
        hh = np.arange(3 * 100, dtype=float).reshape(3, 100)
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
        self.assertEqual(bscan_window_max_start(np.int64(65)), 2)
        self.assertEqual(bscan_window_bounds(np.int64(65), np.int64(2)), (2, 65))

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
