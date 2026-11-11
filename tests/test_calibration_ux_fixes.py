#!/usr/bin/env python3
"""Regression tests for the 3 user-reported Calibration UX/pipeline bugs.

Covers end-to-end (real TCP ReceiverThread <-> RemoteSender <-> AirBlackboxService
<-> CalibrationSession / AcquisitionService) and unit paths for:

1. Bug 1 — Imported calibration file is genuinely applied:
   - Importing real user schemas (`1016dual.json` and `solt_2port_20260807_202342.json`)
     automatically aligns Air `_sweep` RF grid (`start_mhz`/`stop_mhz`/`points`/RF params)
     and syncs Ground `acq_settings` + wizard spinboxes so `calibration_guard` reports
     `level == "ok"` instead of silently disabling calibration as `stale`.
   - Acquired traces after import are numerically corrected by P1/P2 OSL (`display_S11` /
     `display_S22` match `OSLCalibration.correct`) and carry `calibration_id`.
   - Subsequent user `set_params` changing frequency/points still triggers `stale` per
     `calibration_guard` contracts.
   - Importing while an unactivated calibration session is open cleanly cancels the open
     session, activates the imported file, and unblocks radar acquisition.
   - Importing P2-only VV files and Windows copy filenames (`1016dual (1) - 副本.json`)
     succeeds without `AttributeError` or false filename rejection.
   - Startup calibration restore (`AirBlackboxService.__init__`) and custom-grid session
     activation (`cal_activate`) align `_sweep` so restored/activated calibration applies.

2. Bug 2 — Closing CalibrationWizard via window `×` / `Esc` / `reject` without clicking
   "取消会话" automatically cancels unactivated calibration sessions:
   - Closing via `QCloseEvent` (`wizard.close()`) or `Esc` (`wizard.reject()`) during
     `planned`, `in_progress`, or unactivated `solved` sends `cal_cancel` once (no
     double-cancel), clears Air `_calibration_session`, and allows immediate `start` /
     `single` / `set_params`.
   - Closing after `activated` (or after `cal_import_file`) does NOT send `cal_cancel`
     and preserves the active calibration.
   - Closing while `cal_begin` is still in flight cancels cleanly and ignores late
     `planned` frames on Ground so no ghost session resurrects.
   - Closing mid-capture while a slow `ServiceStepCapturer` sweep is running aborts the
     capture task promptly, suppresses bogus `step_done`/`failed`, and allows immediate
     radar acquisition.

3. Bug 3 — Clicking "查询生效状态" (`cal_query_active` / `cal_status`) after importing a
   calibration file reliably finds and displays the active calibration status:
   - `wizard.status_label` transitions from `"正在查询空中端当前生效状态…"` to
     `"查询完成：OSL 已生效…"` (never stuck), and `result_label` + `active_cal_label`
     show ID, filename, ports, points, frequency band, Through/background status, and
     `calibration_fit` match state.
   - `cal_id` is consistent between `cal_import_file` ack and `cal_query_active`.
   - Fresh Ground wizard discovers pre-existing/restored Air calibration on query.
   - `cal_status` reports `status="active"` for imported/restored calibrations even when
     `_calibration_session is None`, and Ground handles `cal_status` replies cleanly.
"""

from __future__ import annotations

import copy
import json
import os
import shutil
import tempfile
import time
import unittest
from dataclasses import replace as dataclass_replace
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

import air_core as air
import uav_gpr_air_service as svc
import uav_gpr_ground_station as ground
from librevna.librevna_continuous import PersistentBackendSettings, PersistentLibreVnaBackend
from librevna.librevna_usb_s11 import OSLCalibration
from librevna.simulated_transport import SimulatedLibreVnaTransport

REPO_ROOT = Path(__file__).resolve().parent.parent
MAIN_WORKSPACE_FALLBACK = Path(r"D:\博士任务\UVA_GPR_system")


def wait_until(predicate, timeout_s: float = 15.0, interval_s: float = 0.03, app=None) -> bool:
    deadline = time.monotonic() + timeout_s
    while True:
        if bool(predicate()):
            return True
        if time.monotonic() >= deadline:
            return bool(predicate())
        if app is not None:
            app.processEvents()
        time.sleep(interval_s)


def npz_complex(npz, prefix: str) -> np.ndarray:
    return np.asarray(npz[f"{prefix}_real"], dtype=np.float64) + 1j * np.asarray(
        npz[f"{prefix}_imag"], dtype=np.float64
    )


def _build_synthetic_user_cal(
    *,
    start_mhz: float = 400.0,
    stop_mhz: float = 4000.0,
    points: int = 201,
    include_p1: bool = True,
    include_p2: bool = True,
    include_through: bool = False,
    dewow_enabled: bool = False,
) -> dict[str, object]:
    freqs = np.linspace(start_mhz * 1e6, stop_mhz * 1e6, points)
    phase = np.linspace(0.0, 1.4, points)

    def _pts(
        s11_vec: np.ndarray,
        s22_vec: np.ndarray,
        *,
        primary: str = "S11",
        extra_through: bool = False,
    ) -> list[dict[str, object]]:
        rows: list[dict[str, object]] = []
        for idx, f in enumerate(freqs):
            sp: dict[str, list[float]] = {
                "S11": [float(s11_vec[idx].real), float(s11_vec[idx].imag)],
                "S22": [float(s22_vec[idx].real), float(s22_vec[idx].imag)],
            }
            if extra_through:
                t_val = 0.72 * np.exp(-1j * phase[idx])
                sp["S21"] = [float(t_val.real), float(t_val.imag)]
                sp["S12"] = [float(t_val.real), float(t_val.imag)]
            prim_val = sp.get(primary, sp["S11"])
            rows.append(
                {
                    "frequency": float(f),
                    "real": float(prim_val[0]),
                    "imag": float(prim_val[1]),
                    "sparameters": sp,
                }
            )
        return rows

    open_1 = 0.88 * np.exp(-1j * 0.25 * phase)
    short_1 = -0.89 * np.exp(-1j * 0.25 * phase)
    load_1 = 0.03 * np.exp(1j * 0.1 * phase)

    open_2 = 0.86 * np.exp(-1j * 0.30 * phase)
    short_2 = -0.87 * np.exp(-1j * 0.30 * phase)
    load_2 = 0.025 * np.exp(-1j * 0.12 * phase)
    idle = np.full(points, 0.01 + 0.005j, dtype=np.complex128)

    measurements: list[dict[str, object]] = []
    if include_p1:
        for name, vec in (("P1_Open", open_1), ("P1_Short", short_1), ("P1_Load", load_1)):
            measurements.append(
                {
                    "type": name,
                    "data": {"primary_parameter": "S11", "points": _pts(vec, idle, primary="S11")},
                }
            )
    if include_p2:
        for name, vec in (("P2_Open", open_2), ("P2_Short", short_2), ("P2_Load", load_2)):
            measurements.append(
                {
                    "type": name,
                    "data": {"primary_parameter": "S22", "points": _pts(idle, vec, primary="S22")},
                }
            )
    if include_through:
        measurements.append(
            {
                "type": "Through",
                "data": {
                    "primary_parameter": "S21",
                    "points": _pts(idle, idle, primary="S21", extra_through=True),
                },
            }
        )
    return {
        "format": "uav_gpr_solt_calibration_v2",
        "saved_at": "2026-10-16T20:28:18",
        "sweep_config": {
            "start_mhz": float(start_mhz),
            "stop_mhz": float(stop_mhz),
            "points": int(points),
            "ifbw_hz": 10000,
            "power_dbm": -20.0,
            "dwell_us": 0,
            "ifft_points": 2048,
            "time_window_ns": 40.0,
            "dewow_window_ns": 4.0,
            "dewow_enabled": bool(dewow_enabled),
            "bpf_rule": "actual_frequency_quarters",
        },
        "measurements": measurements,
    }


def _resolve_or_materialize_user_file(tmp_dir: Path, filename: str, *, include_through: bool, dewow: bool) -> Path:
    """Prefer real user reference JSONs (`1016dual.json` / `solt_2port_*.json`); fallback to identical schema."""
    for base in (REPO_ROOT, MAIN_WORKSPACE_FALLBACK):
        candidate = base / filename
        if candidate.exists():
            dst = tmp_dir / filename
            if dst.resolve() != candidate.resolve():
                shutil.copyfile(candidate, dst)
            return dst
    dst = tmp_dir / filename
    payload = _build_synthetic_user_cal(include_through=include_through, dewow_enabled=dewow)
    dst.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return dst


class _DeterministicTransport(SimulatedLibreVnaTransport):
    """Deterministic LibreVNA transport with distinct standard offsets per task."""

    OFFSETS = (
        complex(0.90, -0.03),
        complex(-0.88, 0.04),
        complex(0.03, 0.02),
        complex(0.91, 0.05),
        complex(-0.90, -0.02),
        complex(0.02, -0.03),
        complex(0.42, -0.18),
    )

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.task_switches = 0

    def _apply_sweep_settings(self, payload: bytes) -> None:
        self.task_switches += 1
        super()._apply_sweep_settings(payload)

    def _next_point_value(self, reflectors, freq_hz: int) -> complex:
        idx = min(max(0, self.task_switches - 1), len(self.OFFSETS) - 1)
        return super()._next_point_value(reflectors, freq_hz) + self.OFFSETS[idx]


class _FrameRecorder(QtCore.QObject):
    def __init__(self, window: ground.GroundStationWindow) -> None:
        super().__init__()
        self.cal_frames: list[dict] = []
        self.cmd_acks: list[dict] = []
        window.receiver.calibration_message.connect(lambda h: self.cal_frames.append(dict(h)))
        window.receiver.command_result.connect(lambda h: self.cmd_acks.append(dict(h)))


class _CalibrationUxE2EBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="cal_ux_fix_"))
        self.addCleanup(self._cleanup)
        air.set_default_device_source("simulated")
        self.ground_out = self.tmp / "ground_out"
        self.window = ground.GroundStationWindow(
            host="127.0.0.1",
            port=0,
            out_dir=self.ground_out,
            start_receiver=True,
        )
        self.wizard = self.window.calibration_dialog
        self.assertTrue(
            wait_until(lambda: self.window.receiver.bound_port is not None, 10, app=self.app),
            "Ground ReceiverThread did not bind port in time",
        )
        self.transport = _DeterministicTransport(pace=True, noise_level=0.0)
        self.air_logs: list[str] = []
        self.extra_services: list[svc.AirBlackboxService] = []
        self.service = self._make_service(self.tmp / "air_config.json", start_sender=True)
        self.assertTrue(
            wait_until(
                lambda: self.service.connected and self.window._remote_connected,
                15,
                app=self.app,
            ),
            "AirBlackboxService failed to complete TCP handshake with GroundStationWindow",
        )
        self.recorder = _FrameRecorder(self.window)

    def _backend_factory(self) -> PersistentLibreVnaBackend:
        return PersistentLibreVnaBackend(
            transport=self.transport,
            settings=PersistentBackendSettings(sweep_timeout_s=60.0),
        )

    def _make_service(
        self,
        config_path: Path,
        *,
        start_sender: bool,
        config: svc.AirConfig | None = None,
        output_dir: Path | None = None,
    ) -> svc.AirBlackboxService:
        cfg = config or svc.AirConfig(
            ground_host="127.0.0.1",
            ground_port=int(self.window.receiver.bound_port),
            device_source="simulated",
            gnss_port="",
            output_dir=str(output_dir or (self.tmp / "runs")),
        )
        instance = svc.AirBlackboxService(
            cfg,
            program_dir=self.tmp,
            config_path=config_path,
            log=self.air_logs.append,
            backend_factory=self._backend_factory,
            reconnect_initial_s=0.25,
            reconnect_max_s=0.6,
        )
        self.extra_services.append(instance)
        if start_sender:
            instance.sender.start()
        return instance

    def _cleanup(self) -> None:
        for instance in reversed(getattr(self, "extra_services", [])):
            try:
                instance.shutdown()
            except Exception:  # noqa: BLE001
                pass
        window = getattr(self, "window", None)
        if window is not None:
            for _ in range(30):
                stats = window.display_fifo.statistics()
                if stats.get("depth", 0) == 0 and stats.get("unresolved", 0) == 0:
                    break
                self.app.processEvents()
                time.sleep(0.02)
            window.close()
            window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
            self.app.processEvents()
        air.set_default_device_source("usb")
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _acquire_single_trace_npz(self) -> tuple[Path, dict[str, object]]:
        existing = set(self.ground_out.rglob("trace_*.npz"))
        self.window.send_single()
        found: list[Path] = []

        def _has_new_trace() -> bool:
            now_files = [p for p in self.ground_out.rglob("trace_*.npz") if p not in existing]
            if now_files:
                found[:] = sorted(now_files)
                return True
            return False

        self.assertTrue(
            wait_until(_has_new_trace, 20, app=self.app),
            f"Single radar trace was not received on Ground (air_logs={self.air_logs[-8:]})",
        )
        self.assertTrue(
            wait_until(lambda: not self.service._is_acquiring(), 15, app=self.app),
            "Air acquisition did not return to idle after single trace",
        )
        trace_path = found[-1]
        meta_file = trace_path.parent / "run_metadata.jsonl"
        meta_lines = [
            json.loads(line)
            for line in meta_file.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        return trace_path, meta_lines[-1]


class TestBug1ImportedCalibrationApplies(_CalibrationUxE2EBase):
    """Bug 1: 校准文件导入上去以后必须真正生效并应用到采集数据。"""

    def test_import_1016dual_aligns_mismatched_sweep_and_applies_to_acquired_trace(self) -> None:
        cal_file = _resolve_or_materialize_user_file(
            self.tmp, "1016dual.json", include_through=False, dewow=False
        )
        # 先把地面与空中扫频改成与 1016dual.json (400-4000 MHz, 201 点) 不同的参数，
        # 复现现场“之前改过扫频参数 -> 导入校准文件 -> calibration_guard 判 stale 静默不生效”。
        self.window.apply_acquisition_settings(
            dataclass_replace(
                self.window.acq_settings,
                start_mhz=600.0,
                stop_mhz=3200.0,
                points=151,
            )
        )
        ack_count_before = len(self.recorder.cmd_acks)
        self.window.send_set_params()
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("action") == "set_params" and (h.get("result") or {}).get("ok")
                    for h in self.recorder.cmd_acks[ack_count_before:]
                ),
                10,
                app=self.app,
            )
        )
        self.assertEqual(self.service._sweep.points, 151)

        # 导入 1016dual.json
        imp_before = len(self.recorder.cmd_acks)
        self.assertTrue(self.wizard.import_calibration_file(str(cal_file)))
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("action") == "cal_import_file" and (h.get("result") or {}).get("ok")
                    for h in self.recorder.cmd_acks[imp_before:]
                ),
                15,
                app=self.app,
            ),
            "地面未收到 1016dual.json 导入成功回执",
        )

        # 1. 空中端射频扫频参数自动对齐到 400-4000 MHz / 201 点，calibration_fit 为 ok
        self.assertAlmostEqual(self.service._sweep.start_mhz, 400.0)
        self.assertAlmostEqual(self.service._sweep.stop_mhz, 4000.0)
        self.assertEqual(self.service._sweep.points, 201)
        fit = self.service._calibration_fit()
        self.assertEqual(fit.level, "ok", f"导入后校准不应 stale: {fit.to_dict()}")
        self.assertFalse(fit.stale)
        self.assertTrue(self.window.calibration_fit_label.isHidden())

        # 2. 地面端本地 acq_settings 与向导输入框同步对齐，防止后续打开设置中心误覆盖
        self.assertAlmostEqual(self.window.acq_settings.start_mhz, 400.0)
        self.assertAlmostEqual(self.window.acq_settings.stop_mhz, 4000.0)
        self.assertEqual(self.window.acq_settings.points, 201)
        self.assertEqual(self.wizard.cal_points.value(), 201)

        # 3. 真实发起单次雷达采集，数值级验证 P1(S11) 与 P2(S22) 校准真正作用到 trace！
        trace_path, meta = self._acquire_single_trace_npz()
        acq_meta = meta.get("acquisition") or {}
        self.assertEqual(acq_meta.get("calibration_id"), "1016dual")

        p1_ref, p2_ref, _ = OSLCalibration.load_components(cal_file)
        self.assertIsNotNone(p1_ref)
        self.assertIsNotNone(p2_ref)
        with np.load(trace_path) as npz:
            freq_hz = np.asarray(npz["frequency_hz"], dtype=np.float64)
            raw_s11 = npz_complex(npz, "raw_S11")
            disp_s11 = npz_complex(npz, "display_S11")
            raw_s22 = npz_complex(npz, "raw_S22")
            disp_s22 = npz_complex(npz, "display_S22")

        expected_s11 = np.array(
            [p1_ref.correct(float(f), complex(v)) for f, v in zip(freq_hz, raw_s11)],
            dtype=np.complex128,
        )
        expected_s22 = np.array(
            [p2_ref.correct(float(f), complex(v)) for f, v in zip(freq_hz, raw_s22)],
            dtype=np.complex128,
        )
        self.assertFalse(np.allclose(disp_s11, raw_s11), "display_S11 等于原始未校准数据，校准未生效！")
        self.assertFalse(np.allclose(disp_s22, raw_s22), "display_S22 等于原始未校准数据，校准未生效！")
        np.testing.assert_allclose(disp_s11, expected_s11, rtol=1e-10, atol=1e-12)
        np.testing.assert_allclose(disp_s22, expected_s22, rtol=1e-10, atol=1e-12)

        # 4. 守护契约不退化：导入之后若用户再手动把频点数改成 301，仍须判 stale 并红色提醒
        stale_before = len(self.recorder.cmd_acks)
        self.window.send_set_params({**self.window.sweep_params(), "points": 301})
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("action") == "set_params"
                    and ((h.get("result") or {}).get("calibration_fit") or {}).get("level") == "stale"
                    for h in self.recorder.cmd_acks[stale_before:]
                ),
                10,
                app=self.app,
            )
        )
        self.assertFalse(self.window.calibration_fit_label.isHidden())
        self.assertIn("请重新校准", self.window.calibration_fit_label.text())

    def test_import_solt_2port_applies_osl_and_through_without_flipping_dewow(self) -> None:
        cal_file = _resolve_or_materialize_user_file(
            self.tmp, "solt_2port_20260807_202342.json", include_through=True, dewow=True
        )
        # 先设成不匹配网格，且地面 Dewow 关闭
        self.window.apply_acquisition_settings(
            dataclass_replace(
                self.window.acq_settings,
                start_mhz=500.0,
                stop_mhz=2500.0,
                points=81,
                dewow_enabled=False,
            )
        )
        self.window.send_set_params()
        self.assertTrue(wait_until(lambda: self.service._sweep.points == 81, 10, app=self.app))

        imp_before = len(self.recorder.cmd_acks)
        self.assertTrue(self.wizard.import_calibration_file(str(cal_file)))
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("action") == "cal_import_file" and (h.get("result") or {}).get("ok")
                    for h in self.recorder.cmd_acks[imp_before:]
                ),
                15,
                app=self.app,
            )
        )
        self.assertEqual(self.service._sweep.points, 201)
        self.assertAlmostEqual(self.service._sweep.start_mhz, 400.0)
        self.assertAlmostEqual(self.service._sweep.stop_mhz, 4000.0)
        # 校准文件里的 dewow_enabled=true 不应偷改当前地面/空中信号处理开关
        self.assertFalse(self.service._sweep.dewow_enabled)
        self.assertFalse(self.window.acq_settings.dewow_enabled)
        self.assertIsNotNone(self.service._active_through)
        self.assertEqual(self.service._calibration_fit().level, "ok")

        trace_path, meta = self._acquire_single_trace_npz()
        self.assertEqual(
            (meta.get("acquisition") or {}).get("calibration_id"),
            "solt_2port_20260807_202342",
        )
        p1_ref, p2_ref, _ = OSLCalibration.load_components(cal_file)
        with np.load(trace_path) as npz:
            freq_hz = np.asarray(npz["frequency_hz"], dtype=np.float64)
            raw_s11 = npz_complex(npz, "raw_S11")
            disp_s11 = npz_complex(npz, "display_S11")
            raw_s22 = npz_complex(npz, "raw_S22")
            disp_s22 = npz_complex(npz, "display_S22")
        np.testing.assert_allclose(
            disp_s11,
            [p1_ref.correct(float(f), complex(v)) for f, v in zip(freq_hz, raw_s11)],
            rtol=1e-10,
            atol=1e-12,
        )
        np.testing.assert_allclose(
            disp_s22,
            [p2_ref.correct(float(f), complex(v)) for f, v in zip(freq_hz, raw_s22)],
            rtol=1e-10,
            atol=1e-12,
        )

    def test_import_during_open_unactivated_session_supersedes_session_and_acquires(self) -> None:
        cal_file = _resolve_or_materialize_user_file(
            self.tmp, "1016dual.json", include_through=False, dewow=False
        )
        self.wizard.repeats.setValue(1)
        self.wizard.begin_calibration()
        open_cal_id = self.wizard._cal_id
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("cal_id") == open_cal_id and h.get("status") == "planned"
                    for h in self.recorder.cal_frames
                ),
                10,
                app=self.app,
            )
        )
        self.wizard.capture_next_step()
        self.assertTrue(
            wait_until(
                lambda: "P1_Open" in self.wizard._completed_steps and not self.wizard._capturing,
                15,
                app=self.app,
            )
        )
        self.assertIsNotNone(self.service._calibration_session)

        # 未点取消会话，直接导入 1016dual.json：不得被旧会话互斥拒绝，且导入后立即允许雷达采集
        imp_before = len(self.recorder.cmd_acks)
        self.assertTrue(self.wizard.import_calibration_file(str(cal_file)))
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("action") == "cal_import_file" and (h.get("result") or {}).get("ok")
                    for h in self.recorder.cmd_acks[imp_before:]
                ),
                15,
                app=self.app,
            )
        )
        self.assertIsNone(self.service._calibration_session)
        self.assertIsNone(self.wizard._cal_id)
        self.assertEqual(self.service.active_calibration_id, "1016dual")

        trace_path, meta = self._acquire_single_trace_npz()
        self.assertTrue(trace_path.exists())
        self.assertEqual((meta.get("acquisition") or {}).get("calibration_id"), "1016dual")

    def test_import_p2_only_and_windows_copy_filename_succeed_and_apply(self) -> None:
        p2_only_payload = _build_synthetic_user_cal(
            start_mhz=500.0,
            stop_mhz=2500.0,
            points=51,
            include_p1=False,
            include_p2=True,
            include_through=False,
        )
        win_copy_file = self.tmp / "1016dual (1) - 副本.json"
        win_copy_file.write_text(
            json.dumps(p2_only_payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )

        # 切换到只采 S22 (VV)
        self.window.apply_acquisition_settings(
            dataclass_replace(self.window.acq_settings, channels="S22")
        )
        self.window.send_set_params()
        self.assertTrue(wait_until(lambda: self.service._channel_mode == "S22", 10, app=self.app))

        imp_before = len(self.recorder.cmd_acks)
        self.assertTrue(self.wizard.import_calibration_file(str(win_copy_file)))
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("action") == "cal_import_file" and (h.get("result") or {}).get("ok")
                    for h in self.recorder.cmd_acks[imp_before:]
                ),
                15,
                app=self.app,
            ),
            f"带空格/括号的 P2 单端口校准文件导入失败: {self.wizard.status_label.text()}",
        )
        self.assertIsNone(self.service._session_calibration)
        self.assertIsNotNone(self.service._session_calibration_p2)
        self.assertEqual(self.service._sweep.points, 51)
        self.assertEqual(self.service._calibration_fit().level, "ok")
        self.assertIn("P2", self.wizard.active_cal_label.text())

        trace_path, _meta = self._acquire_single_trace_npz()
        _, p2_ref, _ = OSLCalibration.load_components(win_copy_file)
        with np.load(trace_path) as npz:
            freq_hz = np.asarray(npz["frequency_hz"], dtype=np.float64)
            raw_s22 = npz_complex(npz, "raw_S22")
            disp_s22 = npz_complex(npz, "display_S22")
        np.testing.assert_allclose(
            disp_s22,
            [p2_ref.correct(float(f), complex(v)) for f, v in zip(freq_hz, raw_s22)],
            rtol=1e-10,
            atol=1e-12,
        )

    def test_startup_restore_custom_grid_and_session_activate_custom_grid_apply(self) -> None:
        # Part A: 自启动恢复非默认网格 (550-2750 MHz, 41 点) 校准文件，_sweep 自动对齐
        custom_cal_path = self.tmp / "restored_custom_41pts.json"
        custom_payload = _build_synthetic_user_cal(start_mhz=550.0, stop_mhz=2750.0, points=41)
        custom_cal_path.write_text(json.dumps(custom_payload, ensure_ascii=False), encoding="utf-8")

        cfg_path = self.tmp / "restore_air_config.json"
        restored_cfg = svc.AirConfig(
            ground_host="127.0.0.1",
            ground_port=1,
            device_source="simulated",
            output_dir=str(self.tmp / "restore_runs"),
            calibration_path=str(custom_cal_path),
        )
        restored_service = self._make_service(cfg_path, start_sender=False, config=restored_cfg)
        self.assertAlmostEqual(restored_service._sweep.start_mhz, 550.0)
        self.assertAlmostEqual(restored_service._sweep.stop_mhz, 2750.0)
        self.assertEqual(restored_service._sweep.points, 41)
        self.assertEqual(restored_service._calibration_fit().level, "ok")
        eff = restored_service._effective_calibration_components()
        self.assertIsNotNone(eff["session_calibration"])
        self.assertEqual(eff["calibration_id"], "restored_custom_41pts")

        # Part B: 在校准向导中自定义 650-2650 MHz / 31 点做分步校准并点击激活，双端扫频自动同步
        self.wizard.cal_start.setValue(650.0)
        self.wizard.cal_stop.setValue(2650.0)
        self.wizard.cal_points.setValue(31)
        self.wizard.repeats.setValue(1)
        idx = self.wizard.mode_combo.findData("osl_p1")
        self.wizard.mode_combo.setCurrentIndex(idx)
        self.window.apply_acquisition_settings(
            dataclass_replace(self.window.acq_settings, channels="S11")
        )
        self.window.send_set_params()
        self.assertTrue(wait_until(lambda: self.service._channel_mode == "S11", 10, app=self.app))

        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("cal_id") == cal_id and h.get("status") == "planned"
                    for h in self.recorder.cal_frames
                ),
                10,
                app=self.app,
            )
        )
        for step_name in ("P1_Open", "P1_Short", "P1_Load"):
            self.wizard.capture_next_step()
            self.assertTrue(
                wait_until(
                    lambda s=step_name: s in self.wizard._completed_steps and not self.wizard._capturing,
                    15,
                    app=self.app,
                )
            )
        self.wizard.finish_calibration()
        self.assertTrue(wait_until(lambda: self.wizard._solved, 10, app=self.app))
        self.wizard.activate_calibration()
        self.assertTrue(
            wait_until(lambda: self.wizard._cal_id is None and self.service._active_osl_id == cal_id, 10, app=self.app)
        )
        self.assertAlmostEqual(self.service._sweep.start_mhz, 650.0)
        self.assertAlmostEqual(self.service._sweep.stop_mhz, 2650.0)
        self.assertEqual(self.service._sweep.points, 31)
        self.assertEqual(self.service._calibration_fit().level, "ok")
        self.assertEqual(self.window.acq_settings.points, 31)


class TestBug2DialogCloseAutoCancelsSession(_CalibrationUxE2EBase):
    """Bug 2: 校准一半点窗口叉号（×）/ Esc 关闭未点取消会话，空中端绝不能卡住拒采。"""

    def test_closing_wizard_via_window_x_mid_session_cancels_air_and_allows_acquisition(self) -> None:
        self.window.open_calibration_dialog()
        self.assertTrue(self.wizard.isVisible())
        self.wizard.repeats.setValue(1)
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id
        self.assertIsNotNone(cal_id)
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("cal_id") == cal_id and h.get("status") == "planned"
                    for h in self.recorder.cal_frames
                ),
                10,
                app=self.app,
            )
        )
        # 采集第 1 步（校准到一半：in_progress）
        self.wizard.capture_next_step()
        self.assertTrue(
            wait_until(
                lambda: "P1_Open" in self.wizard._completed_steps and not self.wizard._capturing,
                15,
                app=self.app,
            )
        )
        self.assertIsNotNone(self.service._calibration_session)

        # 模拟用户直接点击右上角叉号关闭窗口（触发真实 QCloseEvent，不点“取消会话”）
        cancel_acks_before = sum(
            1 for h in self.recorder.cmd_acks if h.get("action") == "cal_cancel"
        )
        close_event = QtGui.QCloseEvent()
        QtWidgets.QApplication.sendEvent(self.wizard, close_event)
        self.wizard.hide()

        self.assertIsNone(self.wizard._cal_id, "关闭窗口后向导本地未激活会话必须立即解除")
        self.assertTrue(
            wait_until(
                lambda: self.service._calibration_session is None
                and sum(1 for h in self.recorder.cmd_acks if h.get("action") == "cal_cancel")
                == cancel_acks_before + 1,
                10,
                app=self.app,
            ),
            "关闭校准窗口后空中端未自动收到并执行 cal_cancel",
        )

        # 再次调用 close() 不得重复发第二次 cal_cancel
        self.wizard.close()
        self.app.processEvents()
        time.sleep(0.15)
        self.app.processEvents()
        self.assertEqual(
            sum(1 for h in self.recorder.cmd_acks if h.get("action") == "cal_cancel"),
            cancel_acks_before + 1,
            "重复关闭窗口不得重复发送 cal_cancel",
        )

        # 立即修改参数 + 立即启动雷达采集：均须畅通无阻！
        set_before = len(self.recorder.cmd_acks)
        self.window.send_set_params()
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("action") == "set_params" and (h.get("result") or {}).get("ok")
                    for h in self.recorder.cmd_acks[set_before:]
                ),
                10,
                app=self.app,
            ),
            "关闭校准窗口后 set_params 仍被卡住拒绝",
        )
        trace_path, _ = self._acquire_single_trace_npz()
        self.assertTrue(trace_path.exists())

    def test_closing_via_esc_reject_in_planned_and_solved_cancels_but_activated_keeps_calibration(self) -> None:
        # Case A: planned 阶段按 Esc (reject)
        self.window.open_calibration_dialog()
        self.wizard.repeats.setValue(1)
        self.wizard.begin_calibration()
        cal_a = self.wizard._cal_id
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("cal_id") == cal_a and h.get("status") == "planned"
                    for h in self.recorder.cal_frames
                ),
                10,
                app=self.app,
            )
        )
        self.wizard.reject()
        self.assertTrue(
            wait_until(lambda: self.service._calibration_session is None, 10, app=self.app),
            "Esc (reject) 关闭未自动取消 planned 会话",
        )

        # Case B: solved 但未点击“激活校准”阶段按 Esc (reject) -> 自动取消，不卡采集
        self.window.open_calibration_dialog()
        idx_bg = self.wizard.mode_combo.findData("background")
        self.wizard.mode_combo.setCurrentIndex(idx_bg)
        self.wizard.begin_calibration()
        cal_b = self.wizard._cal_id
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("cal_id") == cal_b and h.get("status") == "planned"
                    for h in self.recorder.cal_frames
                ),
                10,
                app=self.app,
            )
        )
        self.wizard.capture_next_step()
        self.assertTrue(
            wait_until(
                lambda: "Background" in self.wizard._completed_steps and not self.wizard._capturing,
                15,
                app=self.app,
            )
        )
        self.wizard.finish_calibration()
        self.assertTrue(wait_until(lambda: self.wizard._solved, 10, app=self.app))
        self.assertEqual(self.service._calibration_session.state, air.CalibrationSession.STATE_SOLVED)

        self.wizard.reject()
        self.assertTrue(
            wait_until(lambda: self.service._calibration_session is None, 10, app=self.app),
            "solved 未激活时按 Esc 关闭必须自动取消空中端会话",
        )

        # Case C: 已激活后关闭窗口 -> 绝不发 cal_cancel，已生效校准完好保留
        cal_file = _resolve_or_materialize_user_file(
            self.tmp, "1016dual.json", include_through=False, dewow=False
        )
        self.window.open_calibration_dialog()
        imp_before = len(self.recorder.cmd_acks)
        self.assertTrue(self.wizard.import_calibration_file(str(cal_file)))
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("action") == "cal_import_file" and (h.get("result") or {}).get("ok")
                    for h in self.recorder.cmd_acks[imp_before:]
                ),
                15,
                app=self.app,
            )
        )
        cancels_before = sum(1 for h in self.recorder.cmd_acks if h.get("action") == "cal_cancel")
        self.wizard.close()
        self.app.processEvents()
        time.sleep(0.15)
        self.app.processEvents()
        self.assertEqual(
            sum(1 for h in self.recorder.cmd_acks if h.get("action") == "cal_cancel"),
            cancels_before,
            "已激活校准后关闭窗口绝不得发送 cal_cancel",
        )
        self.assertEqual(self.service.active_calibration_id, "1016dual")

    def test_closing_wizard_while_cal_begin_in_flight_prevents_ghost_session(self) -> None:
        self.window.open_calibration_dialog()
        # 同一个事件循环 tick 内连续点击“开始校准”并立刻点叉号关闭（cal_begin 尚在途）
        self.wizard.begin_calibration()
        inflight_cal_id = self.wizard._cal_id
        self.assertIsNotNone(inflight_cal_id)
        self.wizard.close()
        self.assertIsNone(self.wizard._cal_id)

        # 等待空中端依次处理完 cal_begin 和紧随其后的 cal_cancel
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("cal_id") == inflight_cal_id and h.get("status") == "cancelled"
                    for h in self.recorder.cal_frames
                ),
                10,
                app=self.app,
            ),
            "在途 cal_begin 紧跟窗口关闭未完成空中端取消闭环",
        )
        self.app.processEvents()
        self.assertIsNone(self.service._calibration_session)
        # 迟到的 planned 帧不得把向导复活回有会话态
        self.assertIsNone(self.wizard._cal_id)
        self.assertTrue(self.wizard.begin_button.isEnabled())

        trace_path, _ = self._acquire_single_trace_npz()
        self.assertTrue(trace_path.exists())

    def test_closing_wizard_mid_active_slow_capture_aborts_worker_and_allows_immediate_start(self) -> None:
        self.window.open_calibration_dialog()
        # 慢步进采集（ifbw=100 + points=51 + repeats=10：单 sweep ~1s，全采完需 >10s），
        # 验证采集进行中点叉号立即在当前单次 sweep 边界中止，无需等完剩余 9 次采样！
        self.wizard.cal_ifbw.setValue(100)
        self.wizard.cal_points.setValue(51)
        self.wizard.repeats.setValue(10)
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("cal_id") == cal_id and h.get("status") == "planned"
                    for h in self.recorder.cal_frames
                ),
                10,
                app=self.app,
            )
        )
        self.wizard.capture_next_step()
        self.assertTrue(self.wizard._capturing)
        self.assertTrue(
            self.wizard.cancel_button.isEnabled(),
            "采集进行中也必须允许点击取消会话按钮",
        )
        # 等空中端真实进入 _calibration_thread 且至少采回 1 条样本
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("type") == "calibration_capture" and h.get("cal_id") == cal_id
                    for h in self.recorder.cal_frames
                ),
                20,
                app=self.app,
            ),
            "慢步进校准采集未进入在途状态",
        )

        t0 = time.monotonic()
        self.wizard.close()
        self.assertTrue(
            wait_until(
                lambda: self.service._calibration_session is None
                and self.service._calibration_thread is None
                and not self.service._is_acquiring(),
                8.0,
                app=self.app,
            ),
            "在途慢采集中关闭窗口未在时限内中止空中端采集任务",
        )
        elapsed = time.monotonic() - t0
        self.assertLess(elapsed, 6.0, "取消在途采集未跳过剩余采样轮次")

        # 不得发出伪造的 step_done 或 failed
        bogus = [
            h
            for h in self.recorder.cal_frames
            if h.get("cal_id") == cal_id
            and h.get("type") == "calibration_result"
            and h.get("status") in ("step_done", "failed")
        ]
        self.assertEqual(bogus, [], f"已取消的在途步骤不得再发 step_done/failed: {bogus}")

        # 立即发起正常雷达单次采集成功
        trace_path, _ = self._acquire_single_trace_npz()
        self.assertTrue(trace_path.exists())


class TestBug3QueryStatusAfterImport(_CalibrationUxE2EBase):
    """Bug 3: 导入校准文件后点击“查询生效状态”，必须查到并清晰展示生效校准详情。"""

    def test_query_button_after_importing_1016dual_and_solt_2port_updates_status_and_details(self) -> None:
        for filename, expected_id, expect_through in (
            ("1016dual.json", "1016dual", False),
            ("solt_2port_20260807_202342.json", "solt_2port_20260807_202342", True),
        ):
            with self.subTest(filename=filename):
                cal_file = _resolve_or_materialize_user_file(
                    self.tmp, filename, include_through=expect_through, dewow=expect_through
                )
                imp_before = len(self.recorder.cmd_acks)
                self.assertTrue(self.wizard.import_calibration_file(str(cal_file)))
                self.assertTrue(
                    wait_until(
                        lambda: any(
                            h.get("action") == "cal_import_file" and (h.get("result") or {}).get("ok")
                            for h in self.recorder.cmd_acks[imp_before:]
                        ),
                        15,
                        app=self.app,
                    )
                )
                imp_ack = [
                    h["result"]
                    for h in self.recorder.cmd_acks[imp_before:]
                    if h.get("action") == "cal_import_file" and (h.get("result") or {}).get("ok")
                ][-1]
                self.assertEqual(imp_ack.get("cal_id"), expected_id)

                # 真实点击“查询生效状态”按钮
                query_before = len(self.recorder.cmd_acks)
                self.wizard.query_button.click()
                self.assertEqual(self.wizard.status_label.text(), "正在查询空中端当前生效状态…")

                self.assertTrue(
                    wait_until(
                        lambda: any(
                            h.get("action") == "cal_query_active" and (h.get("result") or {}).get("ok")
                            for h in self.recorder.cmd_acks[query_before:]
                        )
                        and "正在查询" not in self.wizard.status_label.text(),
                        10,
                        app=self.app,
                    ),
                    f"点击查询生效状态后 status_label 一直卡在查询中: {self.wizard.status_label.text()!r}",
                )

                status_text = self.wizard.status_label.text()
                result_text = self.wizard.result_label.text()
                active_text = self.wizard.active_cal_label.text()

                self.assertIn("查询完成", status_text)
                self.assertIn("OSL 已生效", status_text)
                self.assertIn(expected_id, status_text)
                self.assertIn(filename, status_text)
                self.assertIn("P1+P2", status_text)
                self.assertIn("201点", status_text)
                self.assertIn("400.0-4000.0 MHz", status_text)
                self.assertIn("与当前扫频参数一致", status_text)
                if expect_through:
                    self.assertIn("含Through", status_text)
                self.assertIn("生效状态详情", result_text)
                self.assertIn(filename, active_text)
                self.assertEqual(
                    (self.wizard._active_calibration or {}).get("cal_id"),
                    imp_ack.get("cal_id"),
                )

    def test_query_button_discovers_preexisting_air_calibration_on_fresh_ground_state(self) -> None:
        # 空中端预先导入生效 1016dual.json，模拟地面端重开/刚连上尚未有本地缓存
        cal_file = _resolve_or_materialize_user_file(
            self.tmp, "1016dual.json", include_through=False, dewow=False
        )
        res = self.service.handle_command(
            "cal_import_file",
            {
                "filename": "1016dual.json",
                "content": cal_file.read_text(encoding="utf-8"),
                "activate": True,
                "request_id": "pre_imp_01",
            },
        )
        self.assertTrue(res["ok"])
        self.assertIsNone(self.wizard._active_calibration)
        self.assertIn("OSL[未启用]", self.wizard.active_cal_label.text())

        # 用户点击“查询生效状态”：地面必须完整发现并展示 1016dual.json
        q_before = len(self.recorder.cmd_acks)
        self.wizard.query_button.click()
        self.assertTrue(
            wait_until(
                lambda: len(self.recorder.cmd_acks) > q_before
                and "查询完成" in self.wizard.status_label.text(),
                10,
                app=self.app,
            )
        )
        self.assertIsNotNone(self.wizard._active_calibration)
        self.assertIn("1016dual.json", self.wizard.active_cal_label.text())
        self.assertIn("P1+P2", self.wizard.active_cal_label.text())
        self.assertIn("1016dual", self.wizard.status_label.text())
        self.assertIn("1016dual.json", self.wizard.result_label.text())

    def test_cal_status_command_and_legacy_fallback_report_imported_calibration(self) -> None:
        # 未导入前 cal_status 返回 status="none"
        none_res = self.service.handle_command("cal_status", {"cal_id": "query"})
        self.assertTrue(none_res["ok"])
        self.assertEqual(none_res["status"], "none")

        # 导入 1016dual.json（此时 _calibration_session 为 None！）后调用 cal_status
        cal_file = _resolve_or_materialize_user_file(
            self.tmp, "1016dual.json", include_through=False, dewow=False
        )
        self.service.handle_command(
            "cal_import_file",
            {
                "filename": "1016dual.json",
                "content": cal_file.read_text(encoding="utf-8"),
                "activate": True,
            },
        )
        self.assertIsNone(self.service._calibration_session)

        cal_frames_before = len(self.recorder.cal_frames)
        self.window.receiver.send_command("cal_status", {"cal_id": "query"})
        self.assertTrue(
            wait_until(
                lambda: any(
                    h.get("type") == "calibration_result" and h.get("status") == "active"
                    for h in self.recorder.cal_frames[cal_frames_before:]
                ),
                10,
                app=self.app,
            ),
            "导入校准文件后 cal_status 仍错误返回 none 而非 active",
        )
        self.assertIn("OSL 已生效", self.wizard.status_label.text())
        self.assertIn("1016dual", self.wizard.status_label.text())
        self.assertIn("P1+P2", self.wizard.active_cal_label.text())


if __name__ == "__main__":
    unittest.main()
