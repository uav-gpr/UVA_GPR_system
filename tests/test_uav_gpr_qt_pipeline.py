import io
import inspect
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

try:
    import air_core as air
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover - environment dependent
    air = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class UavGprPipelineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.frequency_hz = np.array([100e6, 200e6, 300e6, 400e6], dtype=float)
        self.spectrum = np.array([1.0 + 2.0j, 2.0 + 0.5j, -1.0 + 1.0j, 0.5 - 0.25j])
        self.raw = {
            name: self.spectrum.copy() * (index + 1)
            for index, name in enumerate(air.S_PARAMETERS)
        }
        self.cfg = air.SweepConfig(ifft_points=8, time_window_ns=8.0, dewow_window_ns=4.0)

    def test_pipeline_orders_calibration_background_bpf_and_ifft(self) -> None:
        events: list[tuple[str, np.ndarray | None]] = []

        def fake_calibration(*args):
            events.append(("calibration", None))
            return {
                name: np.full(self.frequency_hz.size, 4.0 + 0.0j)
                for name in air.S_PARAMETERS
            }

        def fake_bpf(_frequency, spectrum):
            events.append(("bpf", np.asarray(spectrum).copy()))
            return np.asarray(spectrum) * 0.5

        def fake_ifft(_frequency, spectrum, **_kwargs):
            events.append(("ifft", np.asarray(spectrum).copy()))
            return np.arange(3, dtype=float), np.zeros(3, dtype=float)

        with patch.object(air, "apply_reflection_and_through_calibration", side_effect=fake_calibration), \
                patch.object(air, "apply_frequency_bpf", side_effect=fake_bpf), \
                patch.object(air, "frequency_to_time_trace", side_effect=fake_ifft):
            display, time_ns, traces = air.process_trace_pipeline(
                self.frequency_hz,
                self.raw,
                self.cfg,
                background_frequency_hz=self.frequency_hz,
                background=np.ones(self.frequency_hz.size, dtype=np.complex128),
            )

        self.assertEqual(events[0][0], "calibration")
        self.assertEqual(events[1][0], "bpf")
        self.assertEqual(events[2][0], "ifft")
        np.testing.assert_allclose(events[1][1], 3.0)
        np.testing.assert_allclose(events[2][1], 1.5)
        self.assertEqual(set(display), set(air.S_PARAMETERS))
        self.assertEqual(set(traces), set(air.S_PARAMETERS))
        np.testing.assert_array_equal(time_ns, np.arange(3, dtype=float))

    def test_background_frequency_axis_mismatch_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "background frequency axis does not match"):
            air.process_trace_pipeline(
                self.frequency_hz,
                self.raw,
                self.cfg,
                background_frequency_hz=self.frequency_hz + 1.0,
                background=np.ones(self.frequency_hz.size, dtype=np.complex128),
            )
        with self.assertRaisesRegex(ValueError, "background frequency axis shape"):
            air.process_trace_pipeline(
                self.frequency_hz,
                self.raw,
                self.cfg,
                background_frequency_hz=self.frequency_hz[:-1],
                background=np.ones(self.frequency_hz.size - 1, dtype=np.complex128),
            )
        with self.assertRaisesRegex(ValueError, "background trace length"):
            air.process_trace_pipeline(
                self.frequency_hz,
                self.raw,
                self.cfg,
                background_frequency_hz=self.frequency_hz,
                background=np.ones(self.frequency_hz.size - 1, dtype=np.complex128),
            )

    def test_pipeline_uses_bpf_spectrum_without_implicit_mean(self) -> None:
        with patch("gpr_processing.dewow", side_effect=lambda data, **_kwargs: data):
            _display, time_ns, traces = air.process_trace_pipeline(
                self.frequency_hz,
                self.raw,
                self.cfg,
            )

        filtered = air.apply_frequency_bpf(self.frequency_hz, self.spectrum)
        padded = np.zeros(self.cfg.ifft_points, dtype=np.complex128)
        padded[: filtered.size] = filtered
        expected_trace = np.fft.ifft(padded).real
        expected_time = np.arange(self.cfg.ifft_points) / (self.cfg.ifft_points * 100e6) * 1e9
        mask = expected_time <= self.cfg.time_window_ns
        np.testing.assert_allclose(time_ns, expected_time[mask])
        np.testing.assert_allclose(traces["S11"], expected_trace[mask])

        old_demeaned = np.zeros(self.cfg.ifft_points, dtype=np.complex128)
        old_demeaned[: self.spectrum.size] = (self.spectrum - self.spectrum.mean()) * air.frequency_bpf_weights(self.frequency_hz)
        self.assertFalse(np.allclose(traces["S11"], np.fft.ifft(old_demeaned).real[mask]))

    def test_pipeline_supports_dewow_on_and_off_independently(self) -> None:
        for enabled, expected_calls in ((False, 0), (True, len(air.S_PARAMETERS))):
            cfg = air.SweepConfig(
                ifft_points=8,
                time_window_ns=8.0,
                dewow_enabled=enabled,
                dewow_window_ns=4.0,
            )
            with self.subTest(dewow_enabled=enabled):
                with patch("gpr_processing.dewow", side_effect=lambda data, **_kwargs: data) as mocked:
                    air.process_trace_pipeline(self.frequency_hz, self.raw, cfg)
                self.assertEqual(mocked.call_count, expected_calls)

    def test_pipeline_preserves_trace_source_for_each_dewow_setting(self) -> None:
        for dewow_enabled in (False, True):
            cfg = air.SweepConfig(
                ifft_points=8,
                time_window_ns=8.0,
                dewow_enabled=dewow_enabled,
                dewow_window_ns=4.0,
            )
            with self.subTest(dewow_enabled=dewow_enabled):
                with patch("gpr_processing.dewow", side_effect=lambda data, **_kwargs: data):
                    _display, _time_ns, traces = air.process_trace_pipeline(
                        self.frequency_hz,
                        self.raw,
                        cfg,
                    )
                source_before = traces["S11"].copy()
                self.assertEqual(traces["S11"].shape, source_before.shape)
                np.testing.assert_array_equal(traces["S11"], source_before)

    def test_make_time_trace_passes_explicit_dewow_flag(self) -> None:
        seen: list[dict[str, object]] = []

        def fake_ifft(_frequency, _spectrum, **kwargs):
            seen.append(kwargs)
            return np.arange(2, dtype=float), np.zeros(2, dtype=float)

        with patch.object(air, "frequency_to_time_trace", side_effect=fake_ifft):
            air.make_time_trace(
                self.frequency_hz,
                self.spectrum,
                air.SweepConfig(dewow_enabled=False),
            )
            air.make_time_trace(
                self.frequency_hz,
                self.spectrum,
                air.SweepConfig(dewow_enabled=True),
            )

        self.assertEqual([kwargs["apply_dewow"] for kwargs in seen], [False, True])

    def test_four_channel_npz_and_telemetry_fields_are_preserved(self) -> None:
        display = {name: values * (0.5 + 0.0j) for name, values in self.raw.items()}
        time_ns = np.arange(3, dtype=float)
        traces = {name: np.arange(3, dtype=float) + index for index, name in enumerate(air.S_PARAMETERS)}
        fix = air.GgaFix(utc="120000", latitude=31.2, longitude=121.5, fix_quality=1, satellites=8, altitude_m=12.5)

        payload = air.trace_npz_bytes(7, self.cfg, self.frequency_hz, self.raw, display, time_ns, traces, fix)
        with np.load(io.BytesIO(payload), allow_pickle=False) as archive:
            for name in air.S_PARAMETERS:
                self.assertIn(f"raw_{name}_real", archive)
                self.assertIn(f"raw_{name}_imag", archive)
                self.assertIn(f"display_{name}_real", archive)
                self.assertIn(f"display_{name}_imag", archive)
                self.assertIn(f"trace_{name}", archive)
            config = json.loads(str(archive["config"].item()))
            self.assertEqual(config["bpf_rule"], air.PROCESSING_BPF_RULE)
            self.assertFalse(config["dewow_enabled"])
            self.assertEqual(config["dewow_window_ns"], 4.0)
            self.assertEqual(config["ifft_points"], 8)
            processing = json.loads(str(archive["processing"].item()))
            self.assertEqual(len(processing["bpf"]["frequency_hz"]), 4)
            self.assertEqual(processing["ifft_points"], 8)
            self.assertFalse(processing["dewow_enabled"])
            self.assertFalse(processing["dewow"]["enabled"])
            self.assertEqual(processing["dewow_window_ns"], 4.0)

        sender = air.RemoteSender("127.0.0.1", 1, "run-test", lambda _message: None)
        sender.enqueue_trace(7, self.cfg, self.frequency_hz, self.raw, display, time_ns, traces, fix)
        header, remote_payload = sender.queue.get_nowait()
        self.assertEqual(header["protocol"], "uav-gpr-telemetry-v1")
        self.assertEqual(header["payload_format"], "npz")
        self.assertEqual(header["payload_bytes"], len(remote_payload))
        self.assertEqual(header["processing"]["ifft_points"], 8)
        self.assertFalse(header["processing"]["dewow_enabled"])
        self.assertFalse(header["processing"]["dewow"]["enabled"])
        self.assertEqual(remote_payload, payload)

    def test_saved_npz_and_common_processing_entry_are_compatible(self) -> None:
        display = {name: values.copy() for name, values in self.raw.items()}
        traces = {name: np.arange(3, dtype=float) + index for index, name in enumerate(air.S_PARAMETERS)}
        fix = air.GgaFix(fix_quality=1)
        with tempfile.TemporaryDirectory() as directory:
            csv_path = air.save_trace(
                Path(directory),
                1,
                self.cfg,
                self.frequency_hz,
                self.raw,
                display,
                np.arange(3, dtype=float),
                traces,
                fix,
            )
            with np.load(csv_path.with_suffix(".npz"), allow_pickle=False) as archive:
                for name in air.S_PARAMETERS:
                    self.assertIn(f"raw_{name}_real", archive)
                    self.assertIn(f"display_{name}_imag", archive)
                    self.assertIn(f"trace_{name}", archive)
                self.assertIn("trace", archive)
                self.assertIn("processing", archive)
                np.testing.assert_array_equal(archive["trace"], archive["trace_S11"])

        self.assertEqual(air.UI_CHANNELS, ("S11", "S22"))
        self.assertEqual(air.SweepConfig().dewow_window_ns, 4.0)
        # 2026-10-15：空中端旧 GUI 已退役，源码检查只剩采集 worker 一处入口。
        source = inspect.getsource(air.AcquisitionWorker.run)
        self.assertGreaterEqual(source.count("process_trace_pipeline"), 1)
        self.assertIn("trace_by_channel, fix", source)

    def test_save_trace_rejects_missing_or_mismatched_channels(self) -> None:
        display = {name: values.copy() for name, values in self.raw.items()}
        fix = air.GgaFix(fix_quality=1)
        time_ns = np.arange(3, dtype=float)
        traces = {name: np.arange(3, dtype=float) for name in air.S_PARAMETERS}
        with tempfile.TemporaryDirectory() as directory:
            missing = dict(traces)
            del missing["S12"]
            with self.assertRaisesRegex(ValueError, "missing channels"):
                air.save_trace(Path(directory), 1, self.cfg, self.frequency_hz, self.raw, display, time_ns, missing, fix)

            wrong_length = dict(traces)
            wrong_length["S22"] = np.arange(2, dtype=float)
            with self.assertRaisesRegex(ValueError, "trace_S22 length"):
                air.save_trace(Path(directory), 2, self.cfg, self.frequency_hz, self.raw, display, time_ns, wrong_length, fix)


    def test_dewow_window_is_checked_against_available_ifft_time(self) -> None:
        cfg = air.SweepConfig(
            ifft_points=8,
            time_window_ns=8.0,
            dewow_enabled=True,
            dewow_window_ns=1000.0,
        )
        with self.assertRaises(ValueError):
            air.process_trace_pipeline(self.frequency_hz, self.raw, cfg)


if __name__ == "__main__":
    unittest.main()
