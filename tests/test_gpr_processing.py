import unittest
from unittest.mock import patch

import numpy as np

from gpr_processing import (
    apply_frequency_bpf,
    dewow,
    flat_reflection_filter,
    frequency_bpf_weights,
    frequency_to_time_trace,
)


class FrequencyBpfTests(unittest.TestCase):
    def test_boundaries_and_smooth_regions(self) -> None:
        frequency = np.linspace(0.0, 2.0, 9)
        weights = frequency_bpf_weights(frequency)
        self.assertAlmostEqual(weights[0], 0.0)
        self.assertAlmostEqual(weights[2], 1.0)
        self.assertAlmostEqual(weights[6], 1.0)
        self.assertAlmostEqual(weights[-1], 0.0)
        self.assertAlmostEqual(weights[1], 0.5)
        self.assertAlmostEqual(weights[7], 0.5)
        self.assertTrue(np.all((weights >= 0.0) & (weights <= 1.0)))

    def test_complex_multiplication_keeps_shape_and_complex_dtype(self) -> None:
        frequency = np.linspace(0.0, 2.0, 9)
        spectrum = np.arange(18, dtype=np.float64).reshape(9, 2).astype(np.complex128)
        result = apply_frequency_bpf(frequency, spectrum)
        expected = spectrum * frequency_bpf_weights(frequency)[:, None]
        self.assertEqual(result.shape, spectrum.shape)
        self.assertTrue(np.iscomplexobj(result))
        np.testing.assert_allclose(result, expected)

    def test_nonfinite_complex_spectrum_is_rejected_for_1d_and_2d(self) -> None:
        frequency = np.linspace(0.0, 2.0, 4)
        for invalid in (np.nan, np.inf, -np.inf):
            one_dimensional = np.ones(4, dtype=np.complex128)
            one_dimensional[1] = complex(invalid, 0.0)
            with self.subTest(shape="1d", value=invalid):
                with self.assertRaisesRegex(ValueError, "finite"):
                    apply_frequency_bpf(frequency, one_dimensional)

            two_dimensional = np.ones((4, 2), dtype=np.complex128)
            two_dimensional[1, 1] = complex(0.0, invalid)
            with self.subTest(shape="2d", value=invalid):
                with self.assertRaisesRegex(ValueError, "finite"):
                    apply_frequency_bpf(frequency, two_dimensional)

    def test_invalid_frequency_axis(self) -> None:
        with self.assertRaises(ValueError):
            frequency_bpf_weights([0.0])
        with self.assertRaises(ValueError):
            frequency_bpf_weights([0.0, 1.0, 1.0])
        with self.assertRaises(ValueError):
            frequency_bpf_weights([0.0, np.nan, 2.0])
        with self.assertRaises(ValueError):
            apply_frequency_bpf([0.0, 1.0], np.ones(3, dtype=np.complex128))


class FrequencyToTimeTests(unittest.TestCase):
    def test_ifft_does_not_demean_and_dewow_sees_full_trace(self) -> None:
        frequency = np.arange(4, dtype=np.float64) * 100e6 + 1e9
        spectrum = np.array([2.0 + 0.0j, 1.0 + 0.5j, 0.5 + 0.0j, 0.25 - 0.5j])
        seen = {}

        def identity_dewow(data, dt_s, window_ns):
            seen["length"] = data.size
            seen["dt_s"] = dt_s
            seen["window_ns"] = window_ns
            return data

        with patch("gpr_processing.dewow", side_effect=identity_dewow):
            time_ns, trace = frequency_to_time_trace(
                frequency,
                spectrum,
                ifft_points=8,
                time_window_ns=2.0,
                dewow_window_ns=3.0,
            )

        padded = np.zeros(8, dtype=np.complex128)
        padded[:4] = spectrum
        expected = np.fft.ifft(padded).real
        self.assertEqual(seen["length"], 8)
        np.testing.assert_allclose(trace, expected[: time_ns.size])

    def test_ifft_length_and_time_axis_use_actual_frequency_spacing(self) -> None:
        frequency = np.arange(4, dtype=np.float64) * 100e6 + 1e9
        spectrum = np.ones(4, dtype=np.complex128)
        time_ns, trace = frequency_to_time_trace(
            frequency,
            spectrum,
            ifft_points=8,
            time_window_ns=6.0,
            dewow_window_ns=2.5,
        )
        expected_dt_ns = 1.0 / (8 * 100e6) * 1e9
        self.assertEqual(trace.size, 5)
        self.assertEqual(time_ns.size, trace.size)
        self.assertAlmostEqual(time_ns[0], 0.0)
        self.assertAlmostEqual(time_ns[1] - time_ns[0], expected_dt_ns)
        self.assertLessEqual(time_ns[-1], 6.0)

    def test_ifft_can_skip_dewow_without_validating_dewow_window(self) -> None:
        frequency = np.arange(4, dtype=np.float64) * 100e6 + 1e9
        spectrum = np.ones(4, dtype=np.complex128)
        with patch("gpr_processing.dewow") as mocked_dewow:
            time_ns, trace = frequency_to_time_trace(
                frequency,
                spectrum,
                ifft_points=8,
                time_window_ns=6.0,
                dewow_window_ns=1e9,
                apply_dewow=False,
            )
        mocked_dewow.assert_not_called()
        padded = np.zeros(8, dtype=np.complex128)
        padded[:4] = spectrum
        expected = np.fft.ifft(padded).real
        np.testing.assert_allclose(trace, expected[: time_ns.size])

    def test_apply_dewow_must_be_bool(self) -> None:
        frequency = np.arange(4, dtype=np.float64) * 100e6 + 1e9
        spectrum = np.ones(4, dtype=np.complex128)
        for value in (1, 0, "true", np.bool_(True)):
            with self.subTest(value=value):
                with self.assertRaises(TypeError):
                    frequency_to_time_trace(frequency, spectrum, 8, 6.0, apply_dewow=value)

    def test_invalid_ifft_window_frequency_and_shapes(self) -> None:
        frequency = np.arange(4, dtype=np.float64) * 100e6 + 1e9
        spectrum = np.ones(4, dtype=np.complex128)
        with self.assertRaises(ValueError):
            frequency_to_time_trace(frequency, spectrum, 1, 6.0)
        with self.assertRaises(ValueError):
            frequency_to_time_trace(frequency, spectrum, 8, 0.0)
        with self.assertRaises(ValueError):
            frequency_to_time_trace(frequency, spectrum[:3], 8, 6.0)
        with self.assertRaises(ValueError):
            frequency_to_time_trace([1.0, 2.0, 4.0, 5.0], spectrum, 8, 6.0)


class DewowTests(unittest.TestCase):
    def test_constant_and_slow_drift_are_suppressed(self) -> None:
        constant = np.full(51, 3.0)
        np.testing.assert_allclose(dewow(constant, 1e-9, 5.0), 0.0, atol=1e-12)

        slow_drift = np.linspace(-1.0, 1.0, 101)
        filtered = dewow(slow_drift, 1e-9, 11.0)
        self.assertEqual(filtered.shape, slow_drift.shape)
        self.assertLess(float(np.max(np.abs(filtered))), 0.1)

    def test_nonfinite_signal_is_rejected_for_1d_and_2d(self) -> None:
        for invalid in (np.nan, np.inf, -np.inf):
            one_dimensional = np.ones(11, dtype=np.float64)
            one_dimensional[3] = invalid
            with self.subTest(shape="1d", value=invalid):
                with self.assertRaisesRegex(ValueError, "finite"):
                    dewow(one_dimensional, 1e-9, 5.0)

            two_dimensional = np.ones((3, 11), dtype=np.float64)
            two_dimensional[1, 3] = invalid
            with self.subTest(shape="2d", value=invalid):
                with self.assertRaisesRegex(ValueError, "finite"):
                    dewow(two_dimensional, 1e-9, 5.0)

    def test_validation_and_time_axis(self) -> None:
        data = np.ones((3, 11), dtype=np.float64)
        np.testing.assert_allclose(dewow(data, 1e-9, 5.0, time_axis=1), 0.0, atol=1e-12)
        with self.assertRaises(ValueError):
            dewow(np.ones((2, 2, 2)), 1e-9, 5.0)
        with self.assertRaises(ValueError):
            dewow(data, 0.0, 5.0)
        with self.assertRaises(ValueError):
            dewow(data, 1e-9, 0.0)
        with self.assertRaises(ValueError):
            dewow(data, 1e-9, 1.0)


class FlatReflectionFilterTests(unittest.TestCase):
    def test_shape_axis_direction_and_background_removal(self) -> None:
        data = np.array(
            [
                [10.0, 10.0, 10.0, 10.0, 10.0],
                [0.0, 1.0, 2.0, 3.0, 4.0],
            ]
        )
        filtered = flat_reflection_filter(data, window_size=3)
        self.assertEqual(filtered.shape, data.shape)
        self.assertEqual(filtered.dtype, np.float64)
        np.testing.assert_allclose(filtered[0], 0.0)
        np.testing.assert_allclose(filtered[1], [-1 / 3, 0.0, 0.0, 0.0, 1 / 3])

    def test_nonfinite_signal_is_rejected(self) -> None:
        for invalid in (np.nan, np.inf, -np.inf):
            data = np.ones((3, 5), dtype=np.float64)
            data[1, 2] = invalid
            with self.subTest(value=invalid):
                with self.assertRaisesRegex(ValueError, "finite"):
                    flat_reflection_filter(data, window_size=3)

    def test_validation_and_single_ascan_rejection(self) -> None:
        data = np.ones((3, 5), dtype=np.float64)
        with self.assertRaises(ValueError):
            flat_reflection_filter(np.ones(5), 3)
        with self.assertRaises(TypeError):
            flat_reflection_filter(data, 3.0)
        with self.assertRaises(ValueError):
            flat_reflection_filter(data, 2)
        with self.assertRaises(ValueError):
            flat_reflection_filter(data, 7)


if __name__ == "__main__":
    unittest.main()
