"""Strict S11/S22 sweep-assembler unit tests (no hardware, pure Python).

Covers the roadmap section 15.2 assembler test matrix:

- complete ``0..N-1``;
- initial non-zero points;
- dropped / duplicate / jump / regression / out-of-range points;
- a new point 0 interrupting a partial sweep;
- two sweeps arriving consecutively (equivalent to one read spanning two sweeps);
- non-increasing and non-finite frequencies;
- no joining across pause / timeout / reconnect boundaries (``reset_current``);
- statistics counter semantics.

Expected counters are asserted exactly so the strict discard rules cannot
silently drift.
"""

from __future__ import annotations

import unittest

import numpy as np

from librevna.librevna_continuous import (
    CompleteSweep,
    ContinuousSweepAssembler,
    InvalidPoint,
    S11S22Point,
)


def _point(
    number: int,
    frequency_hz: float,
    s11: complex = 0.5 + 0.1j,
    s22: complex = -0.3 + 0.2j,
) -> S11S22Point:
    return S11S22Point(number, frequency_hz, s11, s22)


def _run(assembler: ContinuousSweepAssembler, numbers: list[int]) -> list[CompleteSweep]:
    """Feed one point per given point number (linearly increasing frequency)."""
    results: list[CompleteSweep] = []
    for number in numbers:
        sweep = assembler.feed(_point(number, float(number) + 1.0))
        if sweep is not None:
            results.append(sweep)
    return results


class AssemblerConstructionTests(unittest.TestCase):
    def test_rejects_bool_or_small_n_points(self) -> None:
        for bad in (True, 0, 1, -3):
            with self.assertRaises(ValueError):
                ContinuousSweepAssembler(bad)  # type: ignore[arg-type]

    def test_rejects_non_int_n_points(self) -> None:
        with self.assertRaises(ValueError):
            ContinuousSweepAssembler(3.5)  # type: ignore[arg-type]

    def test_initial_statistics_are_zero(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        self.assertEqual(
            assembler.statistics,
            {
                "completed_sweeps": 0,
                "dropped_sweeps": 0,
                "incomplete_sweeps": 0,
                "duplicate_points": 0,
                "jump_points": 0,
                "out_of_range_points": 0,
                "invalid_points": 0,
            },
        )

    def test_feed_rejects_wrong_type(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        with self.assertRaises(TypeError):
            assembler.feed(InvalidPoint())  # type: ignore[arg-type]
        with self.assertRaises(TypeError):
            assembler.feed(object())  # type: ignore[arg-type]

    def test_statistics_are_a_fresh_snapshot(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        first = assembler.statistics
        first["completed_sweeps"] = 999
        self.assertEqual(assembler.statistics["completed_sweeps"], 0)


class AssemblerSequenceTests(unittest.TestCase):
    def test_complete_0_to_n_minus_1_returns_sweep(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        self.assertIsNone(assembler.feed(_point(0, 1.0)))
        self.assertIsNone(assembler.feed(_point(1, 2.0)))
        sweep = assembler.feed(_point(2, 3.0))
        self.assertIsNotNone(sweep)
        assert sweep is not None
        np.testing.assert_array_equal(sweep.frequency_hz, [1.0, 2.0, 3.0])
        np.testing.assert_allclose(sweep.s11, [0.5 + 0.1j] * 3)
        np.testing.assert_allclose(sweep.s22, [-0.3 + 0.2j] * 3)
        self.assertEqual(sweep.raw.shape, (2, 3))
        self.assertEqual(sweep.trace_index, 0)
        self.assertEqual(assembler.statistics["completed_sweeps"], 1)
        self.assertEqual(assembler.statistics["dropped_sweeps"], 0)

    def test_sweep_frequency_uses_device_actual_axis(self) -> None:
        # point frequencies are whatever the device returned; the axis is kept
        # as-is (not re-derived from point number).
        assembler = ContinuousSweepAssembler(3)
        assembler.feed(_point(0, 1.25e9))
        assembler.feed(_point(1, 2.50e9))
        sweep = assembler.feed(_point(2, 5.00e9))
        assert sweep is not None
        np.testing.assert_array_equal(sweep.frequency_hz, [1.25e9, 2.5e9, 5.0e9])

    def test_initial_non_zero_points_are_ignored_then_resync(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        # not active: non-zero points ignored (one unsynchronised drop counted)
        self.assertIsNone(assembler.feed(_point(1, 2.0)))
        self.assertIsNone(assembler.feed(_point(2, 3.0)))
        self.assertEqual(assembler.statistics["dropped_sweeps"], 1)
        # point 0 resynchronises and the sweep completes
        results = _run(assembler, [0, 1, 2])
        self.assertEqual(len(results), 1)
        self.assertEqual(assembler.statistics["completed_sweeps"], 1)
        self.assertEqual(assembler.statistics["dropped_sweeps"], 1)

    def test_dropped_point_discards_sweep_and_resyncs(self) -> None:
        assembler = ContinuousSweepAssembler(4)
        _run(assembler, [0, 1, 3])  # 3 jumps over missing 2 -> discard
        self.assertEqual(assembler.statistics["incomplete_sweeps"], 1)
        self.assertEqual(assembler.statistics["dropped_sweeps"], 1)
        self.assertEqual(assembler.statistics["jump_points"], 1)
        results = _run(assembler, [0, 1, 2, 3])
        self.assertEqual(len(results), 1)
        self.assertEqual(assembler.statistics["completed_sweeps"], 1)

    def test_duplicate_point_discards_sweep(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        _run(assembler, [0, 1, 1, 2])  # duplicate 1 -> discard, then stray 2 ignored
        self.assertEqual(assembler.statistics["duplicate_points"], 1)
        self.assertEqual(assembler.statistics["dropped_sweeps"], 1)
        self.assertEqual(assembler.statistics["incomplete_sweeps"], 1)
        results = _run(assembler, [0, 1, 2])
        self.assertEqual(len(results), 1)
        self.assertEqual(assembler.statistics["completed_sweeps"], 1)

    def test_backward_regression_discards_sweep(self) -> None:
        assembler = ContinuousSweepAssembler(4)
        _run(assembler, [0, 1, 2, 1])  # 1 < expected 3 -> duplicate, discard
        self.assertEqual(assembler.statistics["duplicate_points"], 1)
        self.assertEqual(assembler.statistics["jump_points"], 0)
        self.assertEqual(assembler.statistics["dropped_sweeps"], 1)
        self.assertEqual(assembler.statistics["incomplete_sweeps"], 1)

    def test_forward_jump_discards_sweep(self) -> None:
        assembler = ContinuousSweepAssembler(4)
        _run(assembler, [0, 1, 3])  # 3 > expected 2 -> jump, discard
        self.assertEqual(assembler.statistics["jump_points"], 1)
        self.assertEqual(assembler.statistics["duplicate_points"], 0)
        self.assertEqual(assembler.statistics["dropped_sweeps"], 1)
        self.assertEqual(assembler.statistics["incomplete_sweeps"], 1)

    def test_out_of_range_point_discards_sweep(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        _run(assembler, [0, 1, 5])  # 5 >= n_points -> out of range, discard
        self.assertEqual(assembler.statistics["out_of_range_points"], 1)
        self.assertEqual(assembler.statistics["dropped_sweeps"], 1)
        self.assertEqual(assembler.statistics["incomplete_sweeps"], 1)
        # negative point number is also out of range
        assembler2 = ContinuousSweepAssembler(3)
        sweeps = _run(assembler2, [-1, 0, 1, 2])
        self.assertEqual(assembler2.statistics["out_of_range_points"], 1)
        self.assertEqual(len(sweeps), 1)

    def test_new_point0_interrupts_partial_sweep(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        # 0,1 then a fresh 0: first sweep is dropped/incomplete
        _run(assembler, [0, 1, 0, 1, 2])
        self.assertEqual(assembler.statistics["dropped_sweeps"], 1)
        self.assertEqual(assembler.statistics["incomplete_sweeps"], 1)
        self.assertEqual(assembler.statistics["completed_sweeps"], 1)

    def test_two_consecutive_sweeps_return_both(self) -> None:
        # equivalent to one read spanning two sweeps at the assembler level
        assembler = ContinuousSweepAssembler(3)
        results = _run(assembler, [0, 1, 2, 0, 1, 2])
        self.assertEqual(len(results), 2)
        np.testing.assert_array_equal(results[0].frequency_hz, [1.0, 2.0, 3.0])
        np.testing.assert_array_equal(results[1].frequency_hz, [1.0, 2.0, 3.0])
        self.assertEqual(results[0].statistics["completed_sweeps"], 1)
        self.assertEqual(results[1].statistics["completed_sweeps"], 2)
        self.assertEqual(assembler.statistics["dropped_sweeps"], 0)

    def test_non_increasing_frequency_discards_sweep_at_completion(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        assembler.feed(_point(0, 1.0e9))
        assembler.feed(_point(1, 2.0e9))
        sweep = assembler.feed(_point(2, 1.5e9))  # not increasing -> discard
        self.assertIsNone(sweep)
        self.assertEqual(assembler.statistics["invalid_points"], 1)
        self.assertEqual(assembler.statistics["dropped_sweeps"], 1)
        self.assertEqual(assembler.statistics["incomplete_sweeps"], 1)
        self.assertEqual(assembler.statistics["completed_sweeps"], 0)

    def test_non_finite_point_value_discards_sweep(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        assembler.feed(_point(0, 1.0))
        assembler.feed(_point(1, 2.0))
        # s11 = inf -> invalid point -> current sweep discarded
        self.assertIsNone(assembler.feed(_point(2, 3.0, s11=complex(float("inf"), 0.0))))
        self.assertEqual(assembler.statistics["invalid_points"], 1)
        self.assertEqual(assembler.statistics["dropped_sweeps"], 1)
        self.assertEqual(assembler.statistics["incomplete_sweeps"], 1)
        # NaN frequency also invalid
        self.assertIsNone(assembler.feed(_point(0, float("nan"))))
        self.assertEqual(assembler.statistics["invalid_points"], 2)

    def test_bool_point_number_is_invalid(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        self.assertIsNone(assembler.feed(_point(True, 1.0)))  # type: ignore[arg-type]
        self.assertEqual(assembler.statistics["invalid_points"], 1)

    def test_reset_current_clears_partial_without_joining(self) -> None:
        # pause / timeout / reconnect boundary: partial state cleared; a later
        # sweep must not be joined with the abandoned points.
        assembler = ContinuousSweepAssembler(3)
        assembler.feed(_point(0, 1.0))
        assembler.feed(_point(1, 2.0))
        assembler.reset_current()
        self.assertEqual(assembler.statistics["dropped_sweeps"], 0)
        results = _run(assembler, [0, 1, 2])
        self.assertEqual(len(results), 1)
        np.testing.assert_array_equal(results[0].frequency_hz, [1.0, 2.0, 3.0])
        self.assertEqual(assembler.statistics["completed_sweeps"], 1)
        self.assertEqual(assembler.statistics["incomplete_sweeps"], 0)

    def test_mark_invalid_preserves_point_number_and_counts(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        assembler.mark_invalid(7)
        self.assertEqual(assembler.statistics["invalid_points"], 1)
        # not active: the reference semantics count one unsynchronised drop
        self.assertEqual(assembler.statistics["dropped_sweeps"], 1)

    def test_mark_invalid_discards_an_active_sweep(self) -> None:
        assembler = ContinuousSweepAssembler(3)
        assembler.feed(_point(0, 1.0))
        assembler.mark_invalid(7)
        self.assertEqual(assembler.statistics["invalid_points"], 1)
        self.assertEqual(assembler.statistics["dropped_sweeps"], 1)
        self.assertEqual(assembler.statistics["incomplete_sweeps"], 1)
        # a later sweep is not joined with the invalidated partial
        results = _run(assembler, [0, 1, 2])
        self.assertEqual(len(results), 1)
        np.testing.assert_array_equal(results[0].frequency_hz, [1.0, 2.0, 3.0])

    def test_statistics_accumulate_across_many_sweeps(self) -> None:
        assembler = ContinuousSweepAssembler(4)
        # bad stream then a clean stream
        _run(assembler, [0, 2, 0, 1, 2, 3, 0, 1, 1, 0, 1, 2, 3])
        self.assertEqual(assembler.statistics["completed_sweeps"], 2)
        self.assertEqual(assembler.statistics["dropped_sweeps"], 2)
        self.assertEqual(assembler.statistics["incomplete_sweeps"], 2)
        self.assertEqual(assembler.statistics["duplicate_points"], 1)
        self.assertEqual(assembler.statistics["jump_points"], 1)


class CompleteSweepTests(unittest.TestCase):
    def test_validates_frequency_axis(self) -> None:
        with self.assertRaises(ValueError):
            CompleteSweep([1.0, 2.0, 1.0], np.zeros((2, 3), dtype=complex))
        with self.assertRaises(ValueError):
            CompleteSweep([1.0, float("nan"), 3.0], np.zeros((2, 3), dtype=complex))
        with self.assertRaises(ValueError):
            CompleteSweep([1.0], np.zeros((2, 1), dtype=complex))

    def test_validates_raw_shape_and_finite(self) -> None:
        with self.assertRaises(ValueError):
            CompleteSweep([1.0, 2.0, 3.0], np.zeros((2, 2), dtype=complex))
        with self.assertRaises(ValueError):
            CompleteSweep([1.0, 2.0, 3.0], np.zeros((3, 3), dtype=complex))
        bad = np.zeros((2, 3), dtype=complex)
        bad[0, 0] = complex(float("inf"), 0.0)
        with self.assertRaises(ValueError):
            CompleteSweep([1.0, 2.0, 3.0], bad)

    def test_s11_s22_properties_and_readonly(self) -> None:
        raw = np.array([[1 + 0j, 2 + 0j, 3 + 0j], [4 + 0j, 5 + 0j, 6 + 0j]])
        sweep = CompleteSweep([1.0, 2.0, 3.0], raw)
        np.testing.assert_array_equal(sweep.s11, raw[0])
        np.testing.assert_array_equal(sweep.s22, raw[1])
        with self.assertRaises(ValueError):
            sweep.frequency_hz[0] = 99.0  # type: ignore[index]
        with self.assertRaises(ValueError):
            sweep.raw[0, 0] = 99.0  # type: ignore[index]

    def test_statistics_is_copied_not_shared(self) -> None:
        original = {"completed_sweeps": 1}
        sweep = CompleteSweep(
            [1.0, 2.0, 3.0], np.zeros((2, 3), dtype=complex), statistics=original
        )
        original["completed_sweeps"] = 999
        self.assertEqual(sweep.statistics["completed_sweeps"], 1)


if __name__ == "__main__":
    unittest.main()
