"""单通道采集（只采 HH/S11 或只采 VV/S22）+ VV 背景扣除（2026-10-15）。

第一阶段做法：雷达仍按已在硬件上验证过的双反射扫法（0x1241）扫，黑匣子只保留
用户选中的通道去处理、保存、发送；地面站没采的那幅 B-scan 补零并在标题注明
“本次未采集”。真正只扫一个端口（提速）需要新的扫频 stages 组合，必须先在真机
验证，本轮不做。
"""

from __future__ import annotations

import io
import json
import os
import shutil
import tempfile
import time
import unittest
import unittest.mock
from pathlib import Path
from types import SimpleNamespace

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

import calibration_service as cal  # noqa: E402

try:
    from PySide6 import QtCore, QtWidgets

    import air_core as air
    import uav_gpr_air_service as svc
except Exception as exc:  # pragma: no cover - environment dependent
    air = None
    svc = None
    SKIP = f"air stack unavailable: {exc}"
else:
    SKIP = ""


def _freq(n: int = 9) -> np.ndarray:
    return np.linspace(400e6, 4000e6, n)


def _known_terms(n: int, phase: float):
    f = np.arange(n)
    d = 0.05 * np.exp(1j * (0.1 * f + phase))
    t = 0.9 * np.exp(1j * (0.2 * f + phase))
    m = 0.1 * np.exp(1j * (0.05 * f + phase))
    return d, t, m


def _forward(terms, gamma):
    d, t, m = terms
    return d + t * gamma / (1 - m * gamma)


class _OslCapturer:
    """P1 用 S11 行、P2 用 S22 行的确定性正演；另一行塞常数。"""

    def __init__(self, n: int = 9) -> None:
        self.n = n
        self.frequency = _freq(n)
        self.p1 = _known_terms(n, 0.0)
        self.p2 = _known_terms(n, 0.7)
        self.calls: list[str] = []

    def capture_one(self, step, index):
        self.calls.append(step.name)
        gamma = {"Open": 1.0 + 0j, "Short": -1.0 + 0j, "Load": 0j}[step.name.split("_", 1)[1]]
        s11 = _forward(self.p1, gamma) if step.name.startswith("P1") else np.full(self.n, 0.3 + 0.1j)
        s22 = _forward(self.p2, gamma) if step.name.startswith("P2") else np.full(self.n, 0.2 - 0.1j)
        return self.frequency, np.vstack([s11, s22]).astype(np.complex128)


class CalibrationPortTests(unittest.TestCase):
    def test_normalize_osl_ports(self) -> None:
        self.assertEqual(cal.normalize_osl_ports(None), ("P1", "P2"))
        self.assertEqual(cal.normalize_osl_ports("both"), ("P1", "P2"))
        self.assertEqual(cal.normalize_osl_ports("p1"), ("P1",))
        self.assertEqual(cal.normalize_osl_ports(["P2"]), ("P2",))
        with self.assertRaises(ValueError):
            cal.normalize_osl_ports("P3")

    def test_p1_only_plan_has_three_steps_and_solves(self) -> None:
        plan = cal.build_osl_plan("cal_p1", 2, {}, ports="P1")
        self.assertEqual([s.name for s in plan.steps], ["P1_Open", "P1_Short", "P1_Load"])
        capturer = _OslCapturer()
        session = cal.CalibrationSession(plan, capturer)
        for step in plan.steps:
            session.capture(step.name)
        result = session.solve()
        self.assertIsNotNone(result["p1_terms"])
        self.assertIsNone(result["p2_terms"])
        np.testing.assert_allclose(result["p1_terms"].directivity, capturer.p1[0], atol=1e-9)
        self.assertEqual(result["profile_summary"]["ports"], ["P1"])
        self.assertNotIn("p2", result["profile_summary"])
        activation = session.activate()
        self.assertIsNone(activation["p2_terms"])
        self.assertFalse(any(name.startswith("P2") for name in capturer.calls), "P2 不应被采")

    def test_p2_only_plan_solves_from_s22_row(self) -> None:
        plan = cal.build_osl_plan("cal_p2", 1, {}, ports="P2")
        capturer = _OslCapturer()
        session = cal.CalibrationSession(plan, capturer)
        for step in plan.steps:
            session.capture(step.name)
        result = session.solve()
        self.assertIsNone(result["p1_terms"])
        np.testing.assert_allclose(result["p2_terms"].source_match, capturer.p2[2], atol=1e-9)

    def test_background_keeps_s22_row(self) -> None:
        frequency = _freq(7)
        s11 = np.linspace(0.2, 0.5, 7) + 0.1j
        s22 = np.linspace(0.1, 0.3, 7) - 0.2j

        class _Bg:
            def capture_one(self, step, index):
                return frequency, np.vstack([s11, s22]).astype(np.complex128)

        session = cal.CalibrationSession(cal.build_background_plan("bg", 2, {}), _Bg())
        session.capture(cal.BACKGROUND_STEP_NAME)
        result = session.solve()
        np.testing.assert_allclose(result["background"], s11)
        np.testing.assert_allclose(result["background_s22"], s22)
        activation = session.activate()
        np.testing.assert_allclose(activation["background_s22"], s22)

    def test_p2_only_file_validates_and_loads(self) -> None:
        def _points(values):
            return [
                {"frequency": float(f), "real": float(v.real), "imag": float(v.imag)}
                for f, v in zip(_freq(5), values)
            ]

        terms = _known_terms(5, 0.3)
        data = {
            "format": "uav_gpr_solt_calibration_v2",
            "measurements": [
                {"type": f"P2_{name}", "data": {"points": _points(_forward(terms, gamma))}}
                for name, gamma in (("Open", 1.0 + 0j), ("Short", -1.0 + 0j), ("Load", 0j))
            ],
        }
        ok, error, _ = cal.validate_calibration_content(data)
        self.assertTrue(ok, error)
        from librevna.librevna_usb_s11 import OSLCalibration

        p1, p2, _through = OSLCalibration.load_components(data)
        self.assertIsNone(p1)
        self.assertIsNotNone(p2)

    def test_partial_p1_still_rejected(self) -> None:
        data = {
            "measurements": [
                {"type": "P1_Open", "data": {"points": [{"frequency": 1e9, "real": 1, "imag": 0}, {"frequency": 2e9, "real": 1, "imag": 0}]}},
            ]
        }
        ok, _error, _ = cal.validate_calibration_content(data)
        self.assertFalse(ok)


@unittest.skipIf(air is None, SKIP)
class PipelineChannelTests(unittest.TestCase):
    def _cfg(self):
        return air.SweepConfig(start_mhz=400, stop_mhz=4000, points=9)

    def test_normalize_channel_mode(self) -> None:
        self.assertEqual(air.normalize_channel_mode("dual"), "dual")
        self.assertEqual(air.normalize_channel_mode("hh"), "S11")
        self.assertEqual(air.normalize_channel_mode("s22"), "S22")
        self.assertEqual(air.normalize_channel_mode(["S11", "S22"]), "dual")
        self.assertEqual(air.normalize_channel_mode(["S22"]), "S22")
        for bad in ("S21", "", ["S22", "S11"], None):
            with self.assertRaises(ValueError):
                air.normalize_channel_mode(bad)

    def test_two_row_background_subtracts_each_channel(self) -> None:
        f = _freq()
        raw = {"S11": np.full(9, 0.5 + 0.1j), "S22": np.full(9, 0.4 - 0.2j)}
        bg = np.vstack([np.full(9, 0.1 + 0j), np.full(9, 0.3 + 0j)])
        display, _t, _tr = air.process_trace_pipeline(
            f, raw, self._cfg(), background=bg, background_frequency_hz=f
        )
        np.testing.assert_allclose(display["S11"], raw["S11"] - 0.1)
        np.testing.assert_allclose(display["S22"], raw["S22"] - 0.3)

    def test_one_row_background_only_touches_s11(self) -> None:
        f = _freq()
        raw = {"S11": np.full(9, 0.5 + 0.1j), "S22": np.full(9, 0.4 - 0.2j)}
        display, _t, _tr = air.process_trace_pipeline(
            f, raw, self._cfg(), background=np.full(9, 0.1 + 0j), background_frequency_hz=f
        )
        np.testing.assert_allclose(display["S22"], raw["S22"])

    def test_s22_only_with_p1_calibration_and_two_row_background(self) -> None:
        f = _freq()
        raw = air.select_channels(
            {"S11": np.full(9, 0.5 + 0.1j), "S22": np.full(9, 0.4 - 0.2j)}, ("S22",)
        )
        p1 = cal.terms_to_osl_calibration(
            cal.OSLTerms(f, np.zeros(9), np.ones(9), np.zeros(9), "S11"), None
        )
        bg = np.vstack([np.full(9, 0.1 + 0j), np.full(9, 0.3 + 0j)])
        display, _t, traces = air.process_trace_pipeline(
            f, raw, self._cfg(), calibration_p1=p1, background=bg, background_frequency_hz=f
        )
        self.assertEqual(set(display), {"S22"})
        self.assertEqual(set(traces), {"S22"})
        np.testing.assert_allclose(display["S22"], raw["S22"] - 0.3)

    def test_bad_background_shape_rejected(self) -> None:
        f = _freq()
        raw = {"S11": np.ones(9, dtype=complex), "S22": np.ones(9, dtype=complex)}
        with self.assertRaises(ValueError):
            air.process_trace_pipeline(
                f, raw, self._cfg(), background=np.ones((3, 9), dtype=complex), background_frequency_hz=f
            )

    def test_single_channel_npz_roundtrip_through_ground_decoder(self) -> None:
        import ground_trace_pipeline as gtp

        f = _freq()
        cfg = self._cfg()
        raw = {"S22": np.full(9, 0.4 - 0.2j)}
        display, time_ns, traces = air.process_trace_pipeline(f, raw, cfg)
        payload = air.trace_npz_bytes(
            1, cfg, f, raw, display, time_ns, traces, air.GgaFix(), channels=("S22",)
        )
        decoded = gtp.decode_ground_trace_payload({"channels": ["S22"], "trace_index": 1}, payload, "x.npz")
        self.assertIsNone(decoded.trace_s11)
        self.assertIsNotNone(decoded.trace_s22)
        self.assertEqual(decoded.channels, ("S22",))
        # 声明双通道但只有一路 → 仍按原规则拒绝
        with self.assertRaises(ValueError):
            gtp.decode_ground_trace_payload({"channels": ["S11", "S22"]}, payload, "x.npz")


@unittest.skipIf(svc is None, SKIP)
class AirServiceChannelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="single_ch_"))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        config = svc.AirConfig(
            ground_host="127.0.0.1", ground_port=1, device_source="simulated", output_dir=str(self.tmp / "runs")
        )
        self.logs: list[str] = []
        self.service = svc.AirBlackboxService(
            config,
            program_dir=self.tmp,
            config_path=self.tmp / "air_config.json",
            log=self.logs.append,
            reconnect_initial_s=1.0,
            reconnect_max_s=2.0,
        )
        self.addCleanup(self._teardown)
        self.addCleanup(air.set_default_device_source, "usb")

    def _teardown(self) -> None:
        try:
            self.service.acquisition.shutdown(3000)
        except Exception:  # noqa: BLE001
            pass

    def test_set_params_channels_roundtrip(self) -> None:
        result = self.service.handle_command("set_params", {"channels": "S22"})
        self.assertTrue(result["ok"])
        self.assertEqual(result["channels"], "S22")
        status = self.service.handle_command("get_status", {})
        self.assertEqual(status["config"]["channels"], "S22")
        bad = self.service.handle_command("set_params", {"channels": "S21"})
        self.assertFalse(bad["ok"])
        self.assertEqual(self.service._channel_mode, "S22", "非法值不得改动")

    def test_task_spec_carries_selected_channels(self) -> None:
        self.service.handle_command("set_params", {"channels": "S11"})
        captured = {}

        def _fake_start_task(spec, _factory):
            captured["spec"] = spec
            return None

        with unittest.mock.patch.object(self.service.acquisition, "start_task", _fake_start_task):
            self.service.start_acquisition(single=True)
        self.assertEqual(captured["spec"].channels, ("S11",))

    def test_legacy_engine_rejects_single_channel(self) -> None:
        self.service.handle_command("set_params", {"channels": "S22"})
        self.service.config.engine = "legacy"
        with unittest.mock.patch.object(air, "get_default_device_source", return_value="usb"):
            self.assertFalse(self.service.start_acquisition(single=True))
        self.assertTrue(any("不支持单通道" in line for line in self.logs))

    def test_effective_background_combines_vv_row(self) -> None:
        f = _freq()
        self.service._background = np.full(9, 0.1 + 0j)
        self.service._background_s22 = np.full(9, 0.3 + 0j)
        self.service._background_frequency_hz = f
        self.service._sweep = air.SweepConfig(start_mhz=400, stop_mhz=4000, points=9)
        effective = self.service._effective_calibration_components()
        self.assertEqual(np.asarray(effective["background"]).shape, (2, 9))
        np.testing.assert_allclose(effective["background"][1], 0.3)

    def test_vv_selected_without_vv_background_warns(self) -> None:
        f = _freq()
        self.service._sweep = air.SweepConfig(start_mhz=400, stop_mhz=4000, points=9)
        self.service._background = np.full(9, 0.1 + 0j)
        self.service._background_frequency_hz = f
        result = self.service.handle_command("set_params", {"channels": "S22"})
        self.assertEqual(result["calibration_fit"]["level"], "warning")
        self.assertIn("VV", result["calibration_fit"]["message"])

    def test_simulated_single_channel_run_saves_only_selected_channel(self) -> None:
        self.service.handle_command("set_params", {"channels": "S22", "points": 51, "count": 1})
        started = self.service.start_acquisition(single=True)
        if not started:
            self.skipTest("模拟采集在此环境无法启动")
        deadline = time.monotonic() + 30
        npz_files: list[Path] = []
        while time.monotonic() < deadline:
            QtCore.QCoreApplication.processEvents()
            npz_files = list((self.tmp / "runs").rglob("trace_*.npz"))
            if npz_files and not self.service._is_acquiring():
                break
            time.sleep(0.1)
        self.assertTrue(npz_files, "应有落盘 trace")
        with np.load(npz_files[0], allow_pickle=False) as data:
            self.assertEqual(json.loads(str(data["channels"])), ["S22"])
            self.assertIn("trace_S22", data.files)
            self.assertNotIn("trace_S11", data.files)


@unittest.skipIf(svc is None, SKIP)
class GroundSingleChannelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        import uav_gpr_ground_station as ground

        self.ground = ground
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.tmp.name), start_receiver=False)

    def tearDown(self) -> None:
        self.window.close()
        self.window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        self.app.processEvents()

    def test_settings_dialog_channels_reach_sweep_params(self) -> None:
        dialog = self.ground.AcquisitionSettingsDialog(self.window)
        index = dialog.channels_input.findData("S11")
        dialog.channels_input.setCurrentIndex(index)
        settings = dialog.draft_settings()
        self.assertEqual(settings.channels, "S11")
        self.assertEqual(settings.sweep_params()["channels"], "S11")
        dialog.load_from(self.ground.AcquisitionUiSettings(channels="S22"))
        self.assertEqual(dialog.channels_input.currentData(), "S22")
        dialog.deleteLater()

    def test_single_channel_trace_fills_other_plot_with_zeros_and_marks_title(self) -> None:
        f = _freq()
        cfg = air.SweepConfig(start_mhz=400, stop_mhz=4000, points=9)
        raw = {"S11": np.full(9, 0.4 - 0.2j)}
        display, time_ns, traces = air.process_trace_pipeline(f, raw, cfg)
        payload = air.trace_npz_bytes(1, cfg, f, raw, display, time_ns, traces, air.GgaFix(), channels=("S11",))
        self.window.on_trace({"channels": ["S11"], "trace_index": 1}, payload, "t1.npz")
        self.assertEqual(len(self.window.bscan_rows["S11"]), 1)
        self.assertEqual(len(self.window.bscan_rows["S22"]), 1)
        self.assertFalse(np.any(self.window.bscan_rows["S22"][0]))
        title = self.window.bscan_plots["S22"].getPlotItem().titleLabel.text
        self.assertIn("本次未采集", title)
        self.assertNotIn("本次未采集", self.window.bscan_plots["S11"].getPlotItem().titleLabel.text)
        self.window.clear_bscan()
        self.assertNotIn("本次未采集", self.window.bscan_plots["S22"].getPlotItem().titleLabel.text)

    def test_calibration_dialog_follows_channel_choice(self) -> None:
        from dataclasses import replace

        self.window.acq_settings = replace(self.window.acq_settings, channels="S22")
        self.window.open_calibration_dialog()
        dialog = self.window.calibration_dialog
        self.assertEqual(dialog.mode_combo.currentData(), "osl_p2")
        sent = []
        dialog._connected = True
        with unittest.mock.patch.object(dialog, "_send", lambda action, params: sent.append((action, params))):
            dialog.begin_calibration()
        self.assertEqual(sent[0][1]["mode"], "osl")
        self.assertEqual(sent[0][1]["ports"], "P2")
        self.assertEqual(dialog._steps, ["P2_Open", "P2_Short", "P2_Load"])
        dialog.hide()


if __name__ == "__main__":
    unittest.main()
