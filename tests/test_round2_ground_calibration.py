"""第2轮工作流 C 地面校准 + 地面身份/补传契约测试（R-C-05…R-C-20）。

验收依据：docs/round2_requirements_and_interfaces.md §5.2/§5.4/§5.5 与
§3.7-9（地面身份词汇表）、R-A-19/R-A-19b/R-A-20。纯数学层无 Qt；编排层
fake 后端 + offscreen Qt；协议层真实 ReceiverThread + loopback socket。
明确未验证（§7.3-7）：真实 Open/Short/Load 物理校准质量。
"""

from __future__ import annotations

import io
import json
import os
import socket
import struct
import tempfile
import threading
import time
import unittest
import unittest.mock
from pathlib import Path

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

import calibration_service as cal

try:
    import uav_gpr_ground_station as ground
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
    ground = None
    GROUND_IMPORT_ERROR = exc
else:
    GROUND_IMPORT_ERROR = None

try:
    import air_core as air
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
    air = None
    AIR_IMPORT_ERROR = exc
else:
    AIR_IMPORT_ERROR = None

from PySide6 import QtCore, QtWidgets

PROTOCOL = "uav-gpr-telemetry-v1"


def _wait_until(predicate, timeout_s: float, interval_s: float = 0.01) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return predicate()


# ----------------------------------------------------------------------
# 正演模型：已知 D/T/M + 理想标准件 → 测量值（R-C-07 自洽验证的基准）
# ----------------------------------------------------------------------


def _forward(d: complex, t: complex, m: complex, gamma: complex) -> complex:
    """误差模型反演：m = D + ΓT / (1 − ΓM)（与校正公式互逆）。"""
    return d + gamma * t / (1.0 - gamma * m)


def _known_terms(n_points: int, seed: float):
    index = np.arange(n_points, dtype=float) + seed
    directivity = 0.02 * np.exp(1j * 0.31 * index)
    tracking = 0.8 * np.exp(-1j * 0.17 * index)
    source_match = 0.05 * np.exp(1j * 0.9 * index)
    return directivity, tracking, source_match


def _frequency(n_points: int) -> np.ndarray:
    return np.linspace(400e6, 4000e6, n_points)


def _standard_measurements(terms, n_points: int) -> dict[str, np.ndarray]:
    d, t, m = terms
    return {
        "open": np.array([_forward(d[i], t[i], m[i], 1.0 + 0j) for i in range(n_points)]),
        "short": np.array([_forward(d[i], t[i], m[i], -1.0 + 0j) for i in range(n_points)]),
        "load": np.array([_forward(d[i], t[i], m[i], 0.0 + 0j) for i in range(n_points)]),
    }


# ----------------------------------------------------------------------
# 纯数学层（R-C-06/07/08/09/19）
# ----------------------------------------------------------------------


class CalibrationMathTests(unittest.TestCase):
    def test_solve_recovers_known_dtm(self) -> None:
        """R-C-07：已知 D/T/M 正演 → 解算还原（rtol 1e-9）。"""
        n = 33
        frequency = _frequency(n)
        terms = _known_terms(n, 0.0)
        measured = _standard_measurements(terms, n)
        solved = cal.solve_one_port_osl(
            frequency, measured["open"], measured["short"], measured["load"],
            s_parameter="S11",
        )
        d, t, m = terms
        np.testing.assert_allclose(solved.directivity, d, rtol=1e-9)
        np.testing.assert_allclose(solved.reflection_tracking, t, rtol=1e-9)
        np.testing.assert_allclose(solved.source_match, m, rtol=1e-9)

    def test_p1_p2_profiles_never_share_terms(self) -> None:
        """R-C-07：P1/P2 两套 D/T/M 交换后结果必须不同（证明未共用）。"""
        n = 17
        frequency = _frequency(n)
        terms_p1 = _known_terms(n, 0.0)
        terms_p2 = _known_terms(n, 5.0)
        m1 = _standard_measurements(terms_p1, n)
        m2 = _standard_measurements(terms_p2, n)
        solved_p1 = cal.solve_one_port_osl(frequency, m1["open"], m1["short"], m1["load"], s_parameter="S11")
        solved_swapped = cal.solve_one_port_osl(frequency, m2["open"], m2["short"], m2["load"], s_parameter="S11")
        self.assertGreater(
            float(np.max(np.abs(solved_p1.directivity - solved_swapped.directivity))), 1e-3
        )

    def test_frequency_axis_mismatch_rejected(self) -> None:
        frequency = _frequency(8)
        with self.assertRaises(ValueError):
            cal.solve_one_port_osl(
                frequency,
                np.ones(9, dtype=complex),  # 频率点数不一致
                np.ones(8, dtype=complex),
                np.zeros(8, dtype=complex),
                s_parameter="S11",
            )

    def test_in_band_interpolation_allowed_and_extrapolation_rejected(self) -> None:
        """R-C-07：带内插值允许、带外外推拒绝。"""
        n = 9
        terms = cal.solve_one_port_osl(
            _frequency(n),
            np.full(n, 0.5 + 0.1j), np.full(n, -0.5 + 0.1j), np.zeros(n, dtype=complex),
            s_parameter="S11",
        )
        in_band = np.array([terms.frequency_hz[2], terms.frequency_hz[5]])
        d, t, s = cal.interpolate_terms_in_band(in_band, terms)
        self.assertEqual(d.shape, (2,))
        out_of_band = np.array([terms.frequency_hz[0] - 1.0, terms.frequency_hz[-1]])
        with self.assertRaises(ValueError):
            cal.interpolate_terms_in_band(out_of_band, terms)

    def test_average_is_plain_complex_mean(self) -> None:
        """R-C-06：复数平均 = np.mean(captures, axis=0)（实部虚部同次平均）。"""
        captures = [np.array([1 + 2j, 3 - 4j]), np.array([1 - 2j, 3 + 4j])]
        mean = cal.average_complex_captures(captures)
        np.testing.assert_allclose(mean, np.mean(np.stack(captures), axis=0))
        np.testing.assert_allclose(mean.real, np.mean([c.real for c in captures], axis=0))
        with self.assertRaises(ValueError):
            cal.average_complex_captures([np.ones(3, dtype=complex), np.ones(4, dtype=complex)])
        with self.assertRaises(ValueError):
            cal.average_complex_captures([np.array([np.nan + 0j])])

    def test_validate_repeats_range(self) -> None:
        """R-C-06：repeats 默认 16、范围 1–256；0/负/bool/非 int/NaN 拒绝。"""
        self.assertEqual(cal.validate_repeats(16), 16)
        self.assertEqual(cal.validate_repeats(1), 1)
        self.assertEqual(cal.validate_repeats(256), 256)
        for bad in (0, -1, True, False, 2.5, "16", None, 257):
            with self.assertRaises(ValueError):
                cal.validate_repeats(bad)

    def test_quality_metrics_real_and_insufficient_samples_not_fabricated(self) -> None:
        """R-C-08：重复 RMS/最大绝对误差真实计算；样本 <2 → None + 明确标注。"""
        n = 11
        frequency = _frequency(n)
        terms_known = _known_terms(n, 0.0)
        measured = _standard_measurements(terms_known, n)
        rng = np.random.default_rng(7)
        noise = (rng.standard_normal((4, n)) + 1j * rng.standard_normal((4, n))) * 1e-3
        open_captures = measured["open"][np.newaxis, :] + noise
        short_captures = np.repeat(measured["short"][np.newaxis, :], 4, axis=0)
        load_captures = np.repeat(measured["load"][np.newaxis, :], 4, axis=0)
        solved = cal.solve_one_port_osl(
            frequency, open_captures, short_captures, load_captures, s_parameter="S11"
        )
        metrics = solved.quality_metrics
        self.assertEqual(metrics["open_sample_count"], 4)
        self.assertIsInstance(metrics["open_repeat_rms_abs_error"], float)
        self.assertGreater(metrics["open_repeat_rms_abs_error"], 0.0)
        self.assertGreaterEqual(metrics["open_repeat_max_abs_error"], metrics["open_repeat_rms_abs_error"])
        # 单样本：重复性字段必须为 None 并标注"样本不足"，绝不伪造 0。
        solved_single = cal.solve_one_port_osl(
            frequency, measured["open"], measured["short"], measured["load"], s_parameter="S11"
        )
        self.assertIsNone(solved_single.quality_metrics["open_repeat_rms_abs_error"])
        self.assertIsNone(solved_single.quality_metrics["open_repeat_max_abs_error"])
        self.assertTrue(solved_single.quality_metrics["open_insufficient_samples"])

    def test_calibration_file_roundtrip_and_legacy_readable(self) -> None:
        """R-C-09：v2 格式 write→read 往返一致；旧文件仍可读；不发明新格式。"""
        n = 5
        frequency = _frequency(n)
        measurements = {}
        for name in ("P1_Open", "P1_Short", "P1_Load"):
            values = np.linspace(0.1, 0.9, n) * (1 + 0.1j)
            points = [(float(f), {"S11": complex(v), "S22": complex(0.2 * v)}) for f, v in zip(frequency, values)]
            measurements[name] = cal.CalibrationMeasurement.from_sparameter_points(points, primary="S11")
        cfg = air.SweepConfig() if air is not None else None
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "cal.json"
            cal.write_calibration_file(path, measurements, cfg)
            payload = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(payload["format"], "uav_gpr_solt_calibration_v2")
            loaded = cal.read_calibration_measurements(path)
            for name, measurement in measurements.items():
                self.assertIn(name, loaded)
                np.testing.assert_allclose(loaded[name].frequency_hz, measurement.frequency_hz)
                np.testing.assert_allclose(loaded[name].real, measurement.real)
                np.testing.assert_allclose(loaded[name].imag, measurement.imag)
                np.testing.assert_allclose(
                    loaded[name].channels["S22"]["real"], measurement.channels["S22"]["real"]
                )
            # 旧格式（legacy key Open/Short/Load）仍可读。
            legacy = {
                "format": "uav_gpr_solt_calibration_v2",
                "measurements": [
                    {"type": "Open", "data": {"points": [{"frequency": 1e9, "real": 0.9, "imag": 0.0, "channels": {}}]}},
                    {"type": "Short", "data": {"points": [{"frequency": 1e9, "real": -0.9, "imag": 0.0, "channels": {}}]}},
                    {"type": "Load", "data": {"points": [{"frequency": 1e9, "real": 0.01, "imag": 0.0, "channels": {}}]}},
                ],
            }
            legacy_path = Path(tmp) / "legacy.json"
            legacy_path.write_text(json.dumps(legacy), encoding="utf-8")
            legacy_loaded = cal.read_calibration_measurements(legacy_path)
            self.assertEqual(set(legacy_loaded), {"Open", "Short", "Load"})

    def test_build_reference_background_domain(self) -> None:
        """R-C-19：背景 = RAW/OSL 域校正后 S11（绝不广播到双通道）。"""
        n = 7
        frequency = _frequency(n)
        terms_known = _known_terms(n, 0.0)
        measured = _standard_measurements(terms_known, n)
        solved = cal.solve_one_port_osl(
            frequency, measured["open"], measured["short"], measured["load"], s_parameter="S11"
        )
        raw_s11 = np.linspace(0.2, 0.4, n) * np.exp(1j * np.linspace(0.1, 0.9, n))
        background, frequency_copy = cal.build_reference_background(frequency, raw_s11, solved)
        self.assertEqual(background.shape, (n,))
        np.testing.assert_allclose(frequency_copy, frequency)
        corrected = cal.apply_error_model(raw_s11, solved.directivity, solved.reflection_tracking, solved.source_match, frequency)
        np.testing.assert_allclose(background, corrected, rtol=1e-9)
        raw_background, _ = cal.build_reference_background(frequency, raw_s11, None)
        np.testing.assert_allclose(raw_background, raw_s11)
        with self.assertRaises(ValueError):
            cal.build_reference_background(frequency, np.ones(n + 1, dtype=complex), None)


# ----------------------------------------------------------------------
# 校准会话状态机（R-C-05/06/15/18）
# ----------------------------------------------------------------------


def _fake_capturer_for_session(n_points: int = 9):
    """确定性 fake capturer：按步骤返回该标准件的正演测量 (2, F)。"""
    frequency = _frequency(n_points)
    terms_p1 = _known_terms(n_points, 0.0)
    terms_p2 = _known_terms(n_points, 5.0)
    m_p1 = _standard_measurements(terms_p1, n_points)
    m_p2 = _standard_measurements(terms_p2, n_points)
    standard_by_step = {
        "P1_Open": m_p1["open"], "P1_Short": m_p1["short"], "P1_Load": m_p1["load"],
        "P2_Open": m_p2["open"], "P2_Short": m_p2["short"], "P2_Load": m_p2["load"],
    }

    class _FakeCapturer:
        def __init__(self) -> None:
            self.calls: list[tuple[str, int]] = []
            self.fail_on: str | None = None

        def capture_one(self, step, index):
            self.calls.append((step.name, index))
            if self.fail_on == step.name and index >= 2:
                raise RuntimeError("注入：设备 sweep 超时")
            if step.name == cal.BACKGROUND_STEP_NAME:
                # 空采参考步：S11 = 已知场景的开路测量（确定性即可）。
                s11 = standard_by_step["P1_Open"]
                s22 = standard_by_step["P2_Open"]
                return frequency, np.vstack([s11, s22]).astype(np.complex128)
            s11 = standard_by_step[step.name] if step.channel == "S11" else np.full(n_points, 0.3 + 0.1j)
            s22 = standard_by_step[step.name] if step.channel == "S22" else np.full(n_points, 0.2 - 0.1j)
            return frequency, np.vstack([s11, s22]).astype(np.complex128)

    return _FakeCapturer(), frequency, terms_p1, terms_p2


class CalibrationSessionTests(unittest.TestCase):
    def _session(self, repeats: int = 3, capture_dir=None):
        capturer, frequency, terms_p1, terms_p2 = _fake_capturer_for_session()
        plan = cal.build_osl_plan("20260904_120000_0001_c", repeats, {})
        return cal.CalibrationSession(plan, capturer, capture_dir=capture_dir), capturer, frequency, terms_p1, terms_p2

    def test_happy_path_state_machine(self) -> None:
        """R-C-05：planned → in_progress → 逐步 done → solved → activated。"""
        session, _capturer, _frequency, terms_p1, terms_p2 = self._session(repeats=2)
        self.assertEqual(session.state, "planned")
        for index, step in enumerate(cal.OSL_STEP_NAMES):
            session.capture(step)
            self.assertEqual(session.state, "in_progress")
            progress = session.step_progress(step)
            self.assertEqual(progress.state, "done")
            self.assertEqual(progress.captured, 2)
        result = session.solve()
        self.assertEqual(session.state, "solved")
        np.testing.assert_allclose(result["p1_terms"].directivity, terms_p1[0], rtol=1e-9)
        np.testing.assert_allclose(result["p2_terms"].directivity, terms_p2[0], rtol=1e-9)
        self.assertEqual(result["p1_terms"].s_parameter, "S11")
        self.assertEqual(result["p2_terms"].s_parameter, "S22")
        activation = session.activate()
        self.assertEqual(session.state, "activated")
        self.assertIn("p1_terms", activation)
        active = session.active()
        self.assertIsNotNone(active)
        self.assertEqual(active["cal_id"], session.cal_id)

    def test_capture_step_order_enforced(self) -> None:
        session, _c, _f, _t1, _t2 = self._session()
        with self.assertRaises(RuntimeError):
            session.capture("P1_Short")  # 下一待采是 P1_Open
        session.capture("P1_Open")
        with self.assertRaises(RuntimeError):
            session.capture("P1_Open")  # 重复采集同一步拒绝

    def test_capture_failure_marks_failed_and_blocks_everything(self) -> None:
        """R-C-05：任一点失败 → failed（终态），绝不迁移到 activated。"""
        session, capturer, _f, _t1, _t2 = self._session()
        capturer.fail_on = "P1_Short"
        session.capture("P1_Open")
        with self.assertRaises(RuntimeError):
            session.capture("P1_Short")
        self.assertEqual(session.state, "failed")
        self.assertIn("P1_Short", session.failure_reason)
        with self.assertRaises(RuntimeError):
            session.capture("P1_Load")
        with self.assertRaises(RuntimeError):
            session.solve()
        with self.assertRaises(RuntimeError):
            session.activate()

    def test_cancel_is_terminal_and_keeps_capture_files(self) -> None:
        """R-C-18：取消 → 会话作废、capture 文件保留并写 incomplete 标记。"""
        with tempfile.TemporaryDirectory() as tmp:
            capture_dir = Path(tmp) / "_calibration" / "cal_x"
            session, _c, _f, _t1, _t2 = self._session(capture_dir=capture_dir)
            session.capture("P1_Open")
            files_before = sorted(p.name for p in capture_dir.glob("capture_*.npz"))
            self.assertEqual(len(files_before), 3)
            session.cancel("地面请求取消")
            self.assertEqual(session.state, "cancelled")
            self.assertEqual(files_before, sorted(p.name for p in capture_dir.glob("capture_*.npz")))
            marker = capture_dir / "incomplete.json"
            self.assertTrue(marker.exists())
            self.assertIn("取消", json.loads(marker.read_text(encoding="utf-8"))["reason"])
            with self.assertRaises(RuntimeError):
                session.capture("P1_Short")
            with self.assertRaises(RuntimeError):
                session.activate()

    def test_frequency_grid_change_between_captures_rejected(self) -> None:
        frequency = _frequency(5)
        other = _frequency(6)

        class _ShiftingCapturer:
            def __init__(self) -> None:
                self.count = 0

            def capture_one(self, step, index):
                self.count += 1
                axis = frequency if self.count <= 2 else other
                return axis, np.ones((2, axis.size), dtype=complex)

        plan = cal.build_osl_plan("20260904_120000_0002_c", 4, {})
        session = cal.CalibrationSession(plan, _ShiftingCapturer())
        with self.assertRaises(ValueError):
            session.capture("P1_Open")
        self.assertEqual(session.state, "failed")

    def test_capture_npz_naming_and_keys(self) -> None:
        """R-C-15：capture_ 前缀（禁 trace_）、key 沿用既有约定、无 .tmp 残留。"""
        with tempfile.TemporaryDirectory() as tmp:
            capture_dir = Path(tmp) / "cal_npz"
            session, _c, _f, _t1, _t2 = self._session(capture_dir=capture_dir)
            session.capture("P1_Open")
            files = sorted(capture_dir.glob("*"))
            for path in files:
                self.assertTrue(path.name.startswith("capture_"), f"非法文件名: {path.name}")
                self.assertFalse(path.name.startswith("trace_"))
            with np.load(capture_dir / "capture_P1_Open_1.npz") as npz:
                for key in ("frequency_hz", "S11_real", "S11_imag", "S22_real", "S22_imag"):
                    self.assertIn(key, npz.files)
            self.assertEqual(list(capture_dir.glob("*.tmp")), [])

    def test_repeats_out_of_range_plan_rejected(self) -> None:
        with self.assertRaises(ValueError):
            cal.build_osl_plan("cal_bad", 0, {})
        with self.assertRaises(ValueError):
            cal.build_osl_plan("cal_bad", 300, {})
        with self.assertRaises(ValueError):
            cal.build_osl_plan("cal_bad", True, {})

    def test_background_mode_plan_and_solve(self) -> None:
        """§5.2 空采参考模式：单步 Background → solve → activate 背景生效。"""
        frequency = _frequency(7)
        raw = np.linspace(0.2, 0.5, 7) * np.exp(1j * np.linspace(0.2, 1.1, 7))

        class _BackgroundCapturer:
            def capture_one(self, step, index):
                return frequency, np.vstack([raw, raw * 0.5]).astype(np.complex128)

        plan = cal.build_background_plan("20260904_120000_0003_c", 3, {})
        session = cal.CalibrationSession(plan, _BackgroundCapturer())
        session.capture(cal.BACKGROUND_STEP_NAME)
        result = session.solve()
        self.assertEqual(session.state, "solved")
        np.testing.assert_allclose(result["background"], np.mean(np.stack([raw] * 3), axis=0))
        activation = session.activate()
        np.testing.assert_allclose(activation["background"], result["background"])
        self.assertEqual(session.active()["mode"], "background")


# ----------------------------------------------------------------------
# 地面身份词汇表 / 补传模式（真实 ReceiverThread + loopback socket）
# ----------------------------------------------------------------------


def _trace_payload(trace_index: int = 1) -> tuple[dict, bytes]:
    rng = np.random.default_rng(trace_index)
    buffer = io.BytesIO()
    np.savez_compressed(
        buffer,
        frequency_hz=np.linspace(400e6, 4000e6, 8),
        time_ns=np.linspace(0, 8, 16),
        trace_S11=rng.standard_normal(16),
        trace_S22=rng.standard_normal(16),
    )
    payload = buffer.getvalue()
    header = {
        "type": "trace", "protocol": PROTOCOL,
        "run_id": "20260906_101112_0001", "trace_index": trace_index,
        "sha256": __import__("hashlib").sha256(payload).hexdigest(),
    }
    return header, payload


@unittest.skipIf(ground is None, f"ground dependencies unavailable: {GROUND_IMPORT_ERROR}")
class GroundIdentityProtocolTests(unittest.TestCase):
    """R-A-19/R-A-19b/R-A-20/R-C-13/R-C-17：真实 ReceiverThread 行为。"""

    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.receiver = ground.ReceiverThread("127.0.0.1", 0, Path(self.tmp.name))
        self.status_log: list[str] = []
        self.run_switches: list[str] = []
        self.trace_headers: list[dict] = []
        self.receiver.status.connect(
            self.status_log.append, type=QtCore.Qt.ConnectionType.DirectConnection
        )
        self.receiver.run_switched.connect(
            self.run_switches.append, type=QtCore.Qt.ConnectionType.DirectConnection
        )
        self.receiver.trace_received.connect(
            lambda header, _payload, _path: self.trace_headers.append(header),
            type=QtCore.Qt.ConnectionType.DirectConnection,
        )
        self.receiver.start()
        self.assertTrue(_wait_until(lambda: self.receiver.bound_port is not None, 5.0))
        self.client = socket.create_connection(("127.0.0.1", self.receiver.bound_port), timeout=5)
        self.client.settimeout(5)

    def tearDown(self) -> None:
        try:
            self.receiver.stop()
            self.receiver.wait(3000)
        finally:
            try:
                self.client.close()
            except OSError:
                pass
            self.tmp.cleanup()

    def _send_frame(self, header: dict, payload: bytes = b"") -> None:
        ground.send_frame(self.client, header, payload)

    def _recv_frame(self):
        return ground.recv_frame(self.client)

    def _hello(self, run_id: str) -> None:
        self._send_frame({"type": "hello", "protocol": PROTOCOL, "run_id": run_id})
        header, _payload = self._recv_frame()
        self.assertEqual(header.get("type"), "ack")

    def test_new_air_task_hello_triggers_switch_and_trace_display(self) -> None:
        """R-A-19①/R-C-20：新 run 的 hello → 切换 + trace 正常落盘+ACK+显示。"""
        self._hello("20260906_101112_0002")
        self.assertTrue(_wait_until(
            lambda: self.receiver._mirrored_air_run_id == "20260906_101112_0002", 3.0
        ))
        # 确定性等待（根因消除，非重试掩盖）：接收线程内"镜像赋值 → run_switched
        # emit（DirectConnection 同步追加）"存在顺序窗口，主线程在镜像就绪后
        # 立即断言会偶发读到尚未追加的列表（engA 5 模块串跑首跑观察到 1 次；
        # 修复前 20 连跑采样未复现，见 t12 取证）。事件到达等待不改变被测语义
        # 与断言强度（仍断言恰一次切换）。
        self.assertTrue(_wait_until(lambda: self.run_switches == ["20260906_101112_0002"], 3.0))
        header, payload = _trace_payload(1)
        header = {**header, "run_id": "20260906_101112_0002"}  # 与 hello 的任务身份一致
        self._send_frame(header, payload)
        ack_header, _ack_payload = self._recv_frame()
        self.assertEqual(ack_header.get("type"), "ack_trace")
        self.assertEqual(ack_header.get("trace_index"), 1)
        self.assertTrue(_wait_until(lambda: len(self.trace_headers) == 1, 3.0))
        self.assertTrue((Path(self.tmp.name) / "20260906_101112_0002" / "trace_00001.npz").exists())

    def test_idle_hello_does_not_switch(self) -> None:
        """R-A-19①：idle 式 hello 只作连接身份，不切换不改基线。"""
        self._hello("20260906_101112_idle")
        time.sleep(0.2)
        self.assertEqual(self.receiver._mirrored_air_run_id, "")
        self.assertEqual(self.run_switches, [])
        self.assertIsNone(self.receiver._replay_run_id)

    def test_same_run_hello_is_idempotent(self) -> None:
        """R-A-19③：同 run 重连 hello 不重置内存去重表/任务基线。"""
        self._hello("20260906_101112_0002")
        self.assertTrue(_wait_until(
            lambda: self.receiver._mirrored_air_run_id == "20260906_101112_0002", 3.0
        ))
        header, payload = _trace_payload(1)
        self._send_frame(header, payload)
        self._recv_frame()
        self._hello("20260906_101112_0002")  # 同 run 重连
        # 同 run 重连不得二次切换（同上：事件到达等待消除顺序窗口）。
        self.assertTrue(_wait_until(lambda: self.run_switches == ["20260906_101112_0002"], 3.0))
        self.assertEqual(len(self.run_switches), 1)  # 不二次切换
        # 去重表未被清空：duplicate 直接被内存表吸收。
        self._send_frame(header, payload)
        dup_header, _p = self._recv_frame()
        self.assertEqual(dup_header.get("duplicate"), True)
        self.assertEqual(len(self.trace_headers), 1)

    def test_older_run_hello_enters_replay_mode(self) -> None:
        """R-A-19b：早于基线的 run → 补传模式（落盘+ACK，不显示），退出有汇总。"""
        self._hello("20260906_101112_0002")  # 建立基线
        self.assertTrue(_wait_until(lambda: self.receiver._mirrored_air_run_id != "", 3.0))
        self._hello("20260906_101112_0001")  # 更早的 run → 补传模式
        self.assertTrue(_wait_until(lambda: self.receiver._replay_run_id is not None, 3.0))
        # 基线未动（事件到达等待，同上消除顺序窗口）。
        self.assertTrue(_wait_until(lambda: self.run_switches == ["20260906_101112_0002"], 3.0))
        self.assertEqual(len(self.run_switches), 1)
        header, payload = _trace_payload(7)
        header = {**header, "run_id": "20260906_101112_0001"}
        self._send_frame(header, payload)
        ack_header, _p = self._recv_frame()
        self.assertEqual(ack_header.get("type"), "ack_trace")  # 照常 ACK（outbox 依赖）
        self.assertTrue(_wait_until(
            lambda: (Path(self.tmp.name) / "20260906_101112_0001" / "trace_00007.npz").exists(), 3.0
        ))
        time.sleep(0.2)
        self.assertEqual(self.trace_headers, [], "补传帧不得进入显示路径")
        # 更新的 run hello → 退出补传模式并记汇总日志。
        self._hello("20260906_101112_0003")
        self.assertTrue(_wait_until(
            lambda: any("补传模式结束" in line for line in self.status_log), 3.0
        ))
        self.assertIsNone(self.receiver._replay_run_id)

    def test_replay_run_stopped_logs_summary(self) -> None:
        """R-A-19b：补传 run 的 stopped 到达即记汇总（可观测性不静默）。"""
        self._hello("20260906_101112_0002")
        self.assertTrue(_wait_until(lambda: self.receiver._mirrored_air_run_id != "", 3.0))
        self._hello("20260906_101112_0001")
        self.assertTrue(_wait_until(lambda: self.receiver._replay_run_id is not None, 3.0))
        self._send_frame({
            "type": "acquisition_status", "protocol": PROTOCOL,
            "state": "stopped", "drain_state": "drained",
            "run_id": "20260906_101112_0001",
        })
        self.assertTrue(_wait_until(
            lambda: any("补传模式结束" in line for line in self.status_log), 3.0
        ))

    def test_calibration_messages_never_touch_trace_path(self) -> None:
        """R-C-13/R-C-17：calibration_* 不占 trace 序列/不进去重表；后续 trace 正常。"""
        cal_events: list[dict] = []
        self.receiver.calibration_message.connect(
            cal_events.append, type=QtCore.Qt.ConnectionType.DirectConnection
        )
        self._hello("20260906_101112_0002")
        self.assertTrue(_wait_until(lambda: self.receiver._mirrored_air_run_id != "", 3.0))
        self._send_frame({
            "type": "calibration_quality", "protocol": PROTOCOL,
            "cal_id": "cal_x", "step": "P1_Open", "repeat_rms": 0.01,
            "max_abs_err": 0.02, "sample_count": 4,
        })
        self.assertTrue(_wait_until(lambda: len(cal_events) == 1, 3.0))
        # 未知类型仍走旧端忽略分支（R-C-13 的旧端吸收先例）。
        self._send_frame({"type": "telemetry_note", "protocol": PROTOCOL})
        self.assertTrue(_wait_until(
            lambda: any("Ignored message: telemetry_note" in line for line in self.status_log), 3.0
        ))
        # 其后 trace 正常接收+ACK+显示（连接不断）。
        header, payload = _trace_payload(1)
        header = {**header, "run_id": "20260906_101112_0002"}
        self._send_frame(header, payload)
        ack_header, _p = self._recv_frame()
        self.assertEqual(ack_header.get("type"), "ack_trace")
        self.assertTrue(_wait_until(lambda: len(self.trace_headers) == 1, 3.0))
        # 校准消息未产生任何 run 目录/去重记录（不占 trace 序列）。
        self.assertEqual([p.name for p in Path(self.tmp.name).iterdir()], ["20260906_101112_0002"])

    def test_identity_functions_single_source(self) -> None:
        """§3.7-9：地面 classify 与 outbox 同源（禁止第二套正则）+ 字典序比较。"""
        import disk_outbox

        self.assertIs(ground.classify_run_id, disk_outbox.classify_run_id)
        self.assertEqual(
            ground.classify_run_id("20260906_101112_0002"), "air_task"
        )
        self.assertEqual(
            ground.classify_run_id("20260906_101112_0002_c"), "calibration"
        )
        self.assertEqual(ground.classify_run_id("20260906_101112_idle"), "idle")
        self.assertEqual(ground.classify_run_id("start-1757000000"), "local")
        # 完整字典序：空基线满足；更早 False；相等/更晚 True。
        self.assertTrue(ground.is_not_older_than("20260906_101112_0001", ""))
        self.assertFalse(ground.is_not_older_than("20260906_101112_0001", "20260906_101112_0002"))
        self.assertTrue(ground.is_not_older_than("20260906_101112_0002", "20260906_101112_0002"))
        self.assertTrue(ground.is_not_older_than("20260906_101112_0003", "20260906_101112_0002"))
        # seq 回 1 的新任务（时间更晚）不被误判为补传（R-A-19b 禁 seq 比较）。
        self.assertTrue(ground.is_not_older_than("20260907_090000_0001", "20260906_101112_0009"))


@unittest.skipIf(ground is None, f"ground dependencies unavailable: {GROUND_IMPORT_ERROR}")
class GroundWindowStatusGuardTests(unittest.TestCase):
    """R-A-20：地面迟到护栏与 run 切换（窗口侧，offscreen）。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.tmp.name))

    def tearDown(self) -> None:
        self.window.close()
        self.window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        self.app.processEvents()
        self.tmp.cleanup()

    def test_local_placeholder_baseline_then_air_run_switch(self) -> None:
        """R-A-20：点开始（本地占位）→ run₂ acquiring 完成切换 → run₂ stopped 正常对账。"""
        self.window._begin_task(run_id="start-1757000000")
        self.window.acquisition_status_label.setText("采集状态：正在发送启动命令")
        self.window._on_acquisition_status({
            "type": "acquisition_status", "state": "acquiring",
            "run_id": "20260906_101112_0002",
        })
        self.assertEqual(self.window._task_run_id, "20260906_101112_0002")
        self.assertEqual(self.window._mirrored_air_run_id, "20260906_101112_0002")
        self.assertIn("采集中", self.window.acquisition_status_label.text())
        # run₂ 自己的 stopped 必须被正常对账（不被吞）。
        self.window._on_acquisition_status({
            "type": "acquisition_status", "state": "stopped",
            "drain_state": "drained", "run_id": "20260906_101112_0002",
            "selected_count": 0, "processed_count": 0,
        })
        self.assertIn("已停止（数据已全部排空）", self.window.acquisition_status_label.text())

    def test_acquiring_older_than_baseline_ignored(self) -> None:
        """R-A-19b：早于基线的 acquiring 不得重建任务基线。"""
        self.window._accept_air_run("20260906_101112_0009")
        baseline = self.window._task_run_id
        self.window._on_acquisition_status({
            "type": "acquisition_status", "state": "acquiring",
            "run_id": "20260906_101112_0002",
        })
        self.assertEqual(self.window._task_run_id, baseline)
        self.assertEqual(self.window._mirrored_air_run_id, "20260906_101112_0009")

    def test_same_run_acquiring_idempotent(self) -> None:
        """R-C-20：同 run 的 acquiring 幂等（不清停止对账状态）。"""
        self.window._accept_air_run("20260906_101112_0002")
        self.window._final_stop_state = {"marker": True}
        self.window._on_acquisition_status({
            "type": "acquisition_status", "state": "acquiring",
            "run_id": "20260906_101112_0002",
        })
        self.assertEqual(self.window._final_stop_state, {"marker": True})
        self.assertIn("采集中", self.window.acquisition_status_label.text())

    def test_late_old_air_run_stopped_still_ignored(self) -> None:
        """第1轮冻结语义保持：真旧 run 迟到 stopped 被忽略。"""
        self.window._accept_air_run("20260906_101112_0009")
        self.window._final_stop_state = None
        self.window._on_acquisition_status({
            "type": "acquisition_status", "state": "stopped",
            "drain_state": "drained", "run_id": "20260906_101112_0002",
        })
        self.assertIsNone(self.window._final_stop_state)

    def test_hello_switch_synthesizes_acquiring_path(self) -> None:
        """R-C-20①：hello 触发的切换经合成 acquiring 建立基线（hello 丢失 acquiring 场景）。"""
        self.window._on_hello_run_switched("20260906_101112_0005")
        self.assertEqual(self.window._task_run_id, "20260906_101112_0005")
        self.assertEqual(self.window._mirrored_air_run_id, "20260906_101112_0005")
        self.assertIn("采集中", self.window.acquisition_status_label.text())


class GroundCalibrationDialogTests(unittest.TestCase):
    """R-C-11：地面站校准对话框（纯地面端）。

    2026-10-15 重构：原 AirCalibrationCommandTests 以空中端旧 GUI MainWindow 驱动
    cal_* 命令链；旧 GUI 已退役，空中端 cal_* 命令链改由黑匣子服务测试覆盖
    （test_round7_air_service / test_round8_calibration_core /
    test_round8_calibration_end_to_end）。这里只保留与空中端 GUI 无关的两条
    地面对话框用例。
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def test_ground_dialog_disabled_when_not_connected(self) -> None:
        """R-C-11：未连接时"开始校准"禁用并明确显示未连接。"""
        ground_window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.tmp.name))
        try:
            dialog = ground_window.calibration_dialog
            self.assertFalse(dialog.begin_button.isEnabled())
            self.assertIn("未连接", dialog.status_label.text())
            dialog.set_connected(True)
            self.assertTrue(dialog.begin_button.isEnabled())
        finally:
            ground_window.close()
            ground_window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
            self.app.processEvents()

    def test_ground_dialog_updates_from_calibration_messages(self) -> None:
        """R-C-11：步骤进度/质量/结果摘要随回程更新。"""
        ground_window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.tmp.name))
        try:
            dialog = ground_window.calibration_dialog
            dialog.set_connected(True)
            dialog.begin_calibration()
            self.assertTrue(dialog._cal_id)
            ground_window._on_calibration_message({
                "type": "calibration_result", "cal_id": dialog._cal_id,
                "status": "planned", "mode": "osl",
                "steps": list(cal.OSL_STEP_NAMES), "repeats": 2,
            })
            self.assertEqual(dialog.steps_table.rowCount(), 6)
            ground_window._on_calibration_message({
                "type": "calibration_capture", "cal_id": dialog._cal_id,
                "step": "P1_Open", "index": 1, "of": 2, "channel": "S11",
            })
            self.assertIn("采集中 1/2", dialog.steps_table.item(0, 1).text())
            ground_window._on_calibration_message({
                "type": "calibration_capture", "cal_id": dialog._cal_id,
                "step": "P1_Open", "index": 2, "of": 2, "channel": "S11",
            })
            self.assertEqual(dialog.steps_table.item(0, 1).text(), "已完成")
            ground_window._on_calibration_message({
                "type": "calibration_quality", "cal_id": dialog._cal_id,
                "step": "P1_Open", "repeat_rms": 0.01, "max_abs_err": 0.02,
                "sample_count": 2,
            })
            self.assertIn("重复RMS", dialog.quality_label.text())
            # 旧 cal_id 的回程不得污染当前展示（R-C-18）。
            dialog._cal_id = "cal_current"
            ground_window._on_calibration_message({
                "type": "calibration_result", "cal_id": "cal_old",
                "status": "failed", "reason": "x",
            })
            self.assertNotIn("failed", dialog.status_label.text())
        finally:
            ground_window.close()
            ground_window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
            self.app.processEvents()


if __name__ == "__main__":
    unittest.main()
