"""校准完成后改扫频参数：提醒/停用校准（2026-10-15）。

规则（用户确认）：
- 改频率范围或频点数 → 校准失效：黑匣子停用、数据按未校准、地面站红色提示重新校准；
  参数改回原样 → 自动恢复。
- 只改功率或中频带宽 → 只提醒，校准照常使用。
- 校准进行中 → 不许改扫频参数。
"""

from __future__ import annotations

import os
import shutil
import tempfile
import unittest
import unittest.mock
from pathlib import Path
from types import SimpleNamespace

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

import calibration_guard as guard  # noqa: E402


def _sweep(start=100.0, stop=3000.0, points=401, power=-10.0, ifbw=1000):
    return SimpleNamespace(start_mhz=start, stop_mhz=stop, points=points, power_dbm=power, ifbw_hz=ifbw)


def _fake_osl(start=100.0, stop=3000.0, points=401):
    freqs = np.linspace(start * 1e6, stop * 1e6, points)
    return SimpleNamespace(points=[SimpleNamespace(frequency=float(f)) for f in freqs])


class CalibrationGuardPureTests(unittest.TestCase):
    def test_same_grid_is_ok(self) -> None:
        fit = guard.evaluate_calibration_fit(_sweep(), osl_grids=[guard.grid_from_osl(_fake_osl())])
        self.assertEqual(fit.level, "ok")
        self.assertEqual(fit.message(), "")

    def test_no_calibration_is_ok(self) -> None:
        fit = guard.evaluate_calibration_fit(_sweep(points=11), osl_grids=[None], reference_sweep={"power_dbm": 0})
        self.assertEqual(fit.level, "ok")

    def test_points_change_is_stale(self) -> None:
        fit = guard.evaluate_calibration_fit(_sweep(points=801), osl_grids=[guard.grid_from_osl(_fake_osl())])
        self.assertTrue(fit.osl_stale)
        self.assertEqual(fit.level, "stale")
        self.assertIn("请重新校准", fit.message())
        self.assertIn("401 点", fit.message())
        self.assertIn("801 点", fit.message())

    def test_range_change_is_stale(self) -> None:
        for sweep in (_sweep(start=200.0), _sweep(stop=2500.0)):
            fit = guard.evaluate_calibration_fit(sweep, osl_grids=[guard.grid_from_osl(_fake_osl())])
            self.assertTrue(fit.osl_stale, sweep)

    def test_tiny_frequency_quantization_is_tolerated(self) -> None:
        osl = _fake_osl()
        osl.points[0].frequency += 300.0  # VNA 实际频点 300 Hz 量化误差
        fit = guard.evaluate_calibration_fit(_sweep(), osl_grids=[guard.grid_from_osl(osl)])
        self.assertEqual(fit.level, "ok")

    def test_power_or_ifbw_change_only_warns(self) -> None:
        ref = {"start_mhz": 100.0, "stop_mhz": 3000.0, "points": 401, "power_dbm": -10.0, "ifbw_hz": 1000}
        fit = guard.evaluate_calibration_fit(
            _sweep(power=-5.0, ifbw=500), osl_grids=[guard.grid_from_osl(_fake_osl())], reference_sweep=ref
        )
        self.assertFalse(fit.stale)
        self.assertEqual(fit.level, "warning")
        self.assertEqual(len(fit.warnings), 2)
        self.assertIn("校准仍在使用", fit.message())

    def test_background_mismatch_only_disables_background(self) -> None:
        fit = guard.evaluate_calibration_fit(
            _sweep(),
            osl_grids=[guard.grid_from_osl(_fake_osl())],
            background_grid=guard.grid_from_frequencies(np.linspace(100e6, 3000e6, 201)),
        )
        self.assertFalse(fit.osl_stale)
        self.assertTrue(fit.background_stale)

    def test_osl_stale_also_disables_background(self) -> None:
        fit = guard.evaluate_calibration_fit(
            _sweep(points=801),
            osl_grids=[guard.grid_from_osl(_fake_osl())],
            background_grid=guard.grid_from_frequencies(np.linspace(100e6, 3000e6, 801)),
        )
        self.assertTrue(fit.osl_stale)
        self.assertTrue(fit.background_stale)

    def test_to_dict_is_json_friendly(self) -> None:
        import json

        fit = guard.evaluate_calibration_fit(_sweep(points=801), osl_grids=[guard.grid_from_osl(_fake_osl())])
        data = json.loads(json.dumps(fit.to_dict(), ensure_ascii=False))
        self.assertEqual(data["level"], "stale")
        self.assertEqual(data["calibration_grid"]["points"], 401)


try:
    from PySide6 import QtCore, QtWidgets

    import air_core as air
    import uav_gpr_air_service as svc
except Exception as exc:  # pragma: no cover - environment dependent
    svc = None
    SERVICE_SKIP = f"air service unavailable: {exc}"
else:
    SERVICE_SKIP = ""


@unittest.skipIf(svc is None, SERVICE_SKIP)
class AirServiceCalibrationFitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="calguard_"))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        config = svc.AirConfig(
            ground_host="127.0.0.1",
            ground_port=1,
            device_source="simulated",
            output_dir=str(self.tmp / "runs"),
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
        # 生效校准：按当前扫频的频率轴做的 OSL（P1/P2）
        sweep = self.service._sweep
        self.osl = _fake_osl(sweep.start_mhz, sweep.stop_mhz, sweep.points)
        self.service._session_calibration = self.osl
        self.service._session_calibration_p2 = self.osl
        self.service.active_calibration_id = "CAL-1"
        self.service._active_osl_id = "CAL-1"
        self.service._calibration_sweep_ref = {
            "start_mhz": sweep.start_mhz,
            "stop_mhz": sweep.stop_mhz,
            "points": sweep.points,
            "power_dbm": sweep.power_dbm,
            "ifbw_hz": sweep.ifbw_hz,
        }
        self.original = sweep

    def _teardown(self) -> None:
        try:
            self.service.acquisition.shutdown(3000)
        except Exception:  # noqa: BLE001
            pass

    def test_unchanged_params_keep_calibration(self) -> None:
        result = self.service.handle_command("set_params", {"count": 3})
        self.assertTrue(result["ok"])
        self.assertEqual(result["calibration_fit"]["level"], "ok")
        effective = self.service._effective_calibration_components()
        self.assertIs(effective["session_calibration"], self.osl)
        self.assertEqual(effective["calibration_id"], "CAL-1")

    def test_points_change_disables_calibration_and_reports_stale(self) -> None:
        result = self.service.handle_command("set_params", {"points": self.original.points + 100})
        self.assertTrue(result["ok"])
        fit = result["calibration_fit"]
        self.assertEqual(fit["level"], "stale")
        self.assertIn("请重新校准", fit["message"])
        effective = self.service._effective_calibration_components()
        self.assertIsNone(effective["session_calibration"])
        self.assertIsNone(effective["session_calibration_p2"])
        self.assertEqual(effective["calibration_id"], "", "失效后数据必须标成未校准")
        # 状态查询同样带出失效
        status = self.service.handle_command("get_status", {})
        self.assertEqual(status["calibration_fit"]["level"], "stale")
        # 生效校准本身没被删掉：改回原参数自动恢复
        self.assertEqual(self.service.active_calibration_id, "CAL-1")
        back = self.service.handle_command("set_params", {"points": self.original.points})
        self.assertEqual(back["calibration_fit"]["level"], "ok")
        effective = self.service._effective_calibration_components()
        self.assertIs(effective["session_calibration"], self.osl)
        self.assertEqual(effective["calibration_id"], "CAL-1")

    def test_range_change_is_stale(self) -> None:
        result = self.service.handle_command("set_params", {"start_mhz": self.original.start_mhz + 50})
        self.assertEqual(result["calibration_fit"]["level"], "stale")

    def test_power_change_only_warns_and_keeps_calibration(self) -> None:
        new_power = self.original.power_dbm - 5.0
        result = self.service.handle_command("set_params", {"power_dbm": new_power})
        self.assertTrue(result["ok"])
        self.assertEqual(result["calibration_fit"]["level"], "warning")
        effective = self.service._effective_calibration_components()
        self.assertIs(effective["session_calibration"], self.osl)
        self.assertEqual(effective["calibration_id"], "CAL-1")

    def test_set_params_rejected_while_calibration_in_progress(self) -> None:
        self.service._calibration_session = SimpleNamespace(state=air.CalibrationSession.STATE_IN_PROGRESS)
        result = self.service.handle_command("set_params", {"points": 101})
        self.assertFalse(result["ok"])
        self.assertIn("校准进行中", result["error"])
        self.assertEqual(self.service._sweep.points, self.original.points, "被拒时参数不得改动")

    def test_set_params_allowed_after_calibration_finished(self) -> None:
        self.service._calibration_session = SimpleNamespace(state=air.CalibrationSession.STATE_ACTIVATED)
        result = self.service.handle_command("set_params", {"count": 2})
        self.assertTrue(result["ok"])

    def test_clear_active_drops_reference(self) -> None:
        self.service.handle_command("cal_clear_active", {"target": "all"})
        self.assertIsNone(self.service._calibration_sweep_ref)
        result = self.service.handle_command("set_params", {"points": 101})
        self.assertEqual(result["calibration_fit"]["level"], "ok", "没有生效校准时不提醒")

    def test_start_task_spec_uses_uncalibrated_when_stale(self) -> None:
        self.service.handle_command("set_params", {"points": self.original.points + 100})
        captured: dict[str, object] = {}

        def _fake_start_task(spec, _factory):
            captured["spec"] = spec
            return None

        with unittest.mock.patch.object(self.service.acquisition, "start_task", _fake_start_task):
            self.service.start_acquisition(single=True)
        spec = captured["spec"]
        self.assertIsNone(spec.session_calibration)
        self.assertEqual(spec.calibration_id, "")
        self.assertTrue(any("请重新校准" in line for line in self.logs))


@unittest.skipIf(svc is None, SERVICE_SKIP)
class GroundCalibrationFitDisplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        import uav_gpr_ground_station as ground

        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.tmp.name), start_receiver=False)

    def tearDown(self) -> None:
        self.window.close()
        self.window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        self.app.processEvents()

    def _ack(self, action: str, result: dict) -> None:
        self.window._on_command_result({"type": "command_ack", "action": action, "result": result, "supported": True})

    def test_label_hidden_by_default(self) -> None:
        self.assertTrue(self.window.calibration_fit_label.isHidden())

    def test_stale_shows_red_recalibrate_hint_then_clears(self) -> None:
        self._ack(
            "set_params",
            {"ok": True, "calibration_fit": {"level": "stale", "message": "校准已失效，请重新校准：OSL 校准是 …"}},
        )
        label = self.window.calibration_fit_label
        self.assertFalse(label.isHidden())
        self.assertIn("请重新校准", label.text())
        self.assertIn("#c0392b", label.styleSheet())
        self._ack("get_status", {"ok": True, "calibration_fit": {"level": "ok", "message": ""}})
        self.assertTrue(label.isHidden())

    def test_warning_shows_orange_hint(self) -> None:
        self._ack(
            "set_params",
            {"ok": True, "calibration_fit": {"level": "warning", "message": "校准提醒：发射功率…"}},
        )
        label = self.window.calibration_fit_label
        self.assertFalse(label.isHidden())
        self.assertIn("#f5b041", label.styleSheet())

    def test_rejected_set_params_is_visible(self) -> None:
        self._ack("set_params", {"ok": False, "error": "校准进行中不可修改扫频参数（请先完成或取消校准）"})
        self.assertIn("被空中端拒绝", self.window.acquisition_status_label.text())
        self.assertIn("校准进行中", self.window.acquisition_status_label.text())

    def test_calibration_dialog_prefills_current_acquisition_grid(self) -> None:
        from dataclasses import replace

        self.window.acq_settings = replace(self.window.acq_settings, start_mhz=500.0, stop_mhz=3500.0, points=301)
        self.window.open_calibration_dialog()
        dialog = self.window.calibration_dialog
        self.assertEqual(dialog.cal_start.value(), 500.0)
        self.assertEqual(dialog.cal_stop.value(), 3500.0)
        self.assertEqual(dialog.cal_points.value(), 301)
        dialog.hide()


if __name__ == "__main__":
    unittest.main()
