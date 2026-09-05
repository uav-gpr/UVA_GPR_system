from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from dual_channel_reference import (
    DualChannelCalibration,
    DualChannelSweep,
    SweepSettings,
    acquire_dual_channel,
)


def measured_reflection(
    actual: np.ndarray,
    directivity: np.ndarray,
    tracking: np.ndarray,
    source_match: np.ndarray,
) -> np.ndarray:
    return directivity + actual * tracking / (1.0 - actual * source_match)


class DualChannelReferenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.frequency = np.linspace(400e6, 800e6, 5)
        self.settings = SweepSettings(
            start_hz=400_000_000,
            stop_hz=800_000_000,
            points=5,
        )
        self.d1 = np.full(5, 0.02 + 0.01j)
        self.t1 = np.full(5, 0.91 - 0.04j)
        self.m1 = np.full(5, 0.05 + 0.02j)
        self.d2 = np.full(5, -0.01 + 0.015j)
        self.t2 = np.full(5, 0.88 + 0.03j)
        self.m2 = np.full(5, 0.04 - 0.01j)
        self.through_s21 = np.full(5, 0.75 - 0.08j)
        self.through_s12 = np.full(5, 0.72 + 0.05j)

    def sweep(self, s11, s22, s21=None, s12=None) -> DualChannelSweep:
        return DualChannelSweep(
            self.frequency,
            {
                "S11": np.asarray(s11, dtype=np.complex128),
                "S21": self.through_s21 if s21 is None else np.asarray(s21),
                "S12": self.through_s12 if s12 is None else np.asarray(s12),
                "S22": np.asarray(s22, dtype=np.complex128),
            },
            settings=self.settings,
        )

    def calibration(self) -> DualChannelCalibration:
        open_actual = np.ones(5, dtype=np.complex128)
        short_actual = -open_actual
        load_actual = np.zeros(5, dtype=np.complex128)
        return DualChannelCalibration.from_captures(
            p1_open=self.sweep(
                measured_reflection(open_actual, self.d1, self.t1, self.m1),
                np.zeros(5),
            ),
            p1_short=self.sweep(
                measured_reflection(short_actual, self.d1, self.t1, self.m1),
                np.zeros(5),
            ),
            p1_load=self.sweep(
                measured_reflection(load_actual, self.d1, self.t1, self.m1),
                np.zeros(5),
            ),
            p2_open=self.sweep(
                np.zeros(5),
                measured_reflection(open_actual, self.d2, self.t2, self.m2),
            ),
            p2_short=self.sweep(
                np.zeros(5),
                measured_reflection(short_actual, self.d2, self.t2, self.m2),
            ),
            p2_load=self.sweep(
                np.zeros(5),
                measured_reflection(load_actual, self.d2, self.t2, self.m2),
            ),
            through=self.sweep(
                np.zeros(5),
                np.zeros(5),
                self.through_s21,
                self.through_s12,
            ),
        )

    def test_seven_capture_calibration_recovers_all_channels(self) -> None:
        calibration = self.calibration()
        actual_s11 = np.linspace(0.1, 0.3, 5) + 0.05j
        actual_s22 = np.linspace(-0.2, 0.2, 5) - 0.03j
        actual_s21 = np.linspace(0.2, 0.6, 5) + 0.02j
        actual_s12 = np.linspace(0.3, 0.5, 5) - 0.04j
        raw = self.sweep(
            measured_reflection(actual_s11, self.d1, self.t1, self.m1),
            measured_reflection(actual_s22, self.d2, self.t2, self.m2),
            actual_s21 * self.through_s21,
            actual_s12 * self.through_s12,
        )
        corrected = calibration.apply(raw)
        np.testing.assert_allclose(corrected.sparams["S11"], actual_s11, atol=1e-12)
        np.testing.assert_allclose(corrected.sparams["S22"], actual_s22, atol=1e-12)
        np.testing.assert_allclose(corrected.sparams["S21"], actual_s21, atol=1e-12)
        np.testing.assert_allclose(corrected.sparams["S12"], actual_s12, atol=1e-12)

    def test_sweep_and_calibration_round_trip(self) -> None:
        calibration = self.calibration()
        raw = self.sweep(np.ones(5) * 0.1, np.ones(5) * 0.2)
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            sweep_path = raw.save_npz(directory_path / "raw.npz")
            calibration_path = calibration.save_json(directory_path / "calibration.json")
            loaded_sweep = DualChannelSweep.load_npz(sweep_path)
            loaded_calibration = DualChannelCalibration.load_json(calibration_path)
            np.testing.assert_array_equal(loaded_sweep.frequency_hz, raw.frequency_hz)
            np.testing.assert_allclose(
                loaded_calibration.port1.directivity,
                calibration.port1.directivity,
            )

    def test_acquisition_accepts_an_injected_backend(self) -> None:
        def backend(start_hz, stop_hz, points, ifbw_hz, power_dbm, dwell_us):
            self.assertEqual((start_hz, stop_hz, points), (400_000_000, 800_000_000, 5))
            return [
                (
                    float(frequency),
                    {name: complex(index + 1, channel) for channel, name in enumerate(("S11", "S21", "S12", "S22"))},
                )
                for index, frequency in enumerate(self.frequency)
            ]

        sweep = acquire_dual_channel(self.settings, backend=backend)
        self.assertEqual(sweep.frequency_hz.size, 5)
        self.assertEqual(set(sweep.sparams), {"S11", "S21", "S12", "S22"})


if __name__ == "__main__":
    unittest.main()
