"""第3轮：模拟采集通道（协议线假 LibreVNA + 空中端“设备来源”开关）。

覆盖（对照任务卡 acceptance 逐条取证）：
- 模拟 transport 协议兼容：DeviceInfo/SET_IDLE/SWEEP_SETTINGS/ACK 握手、
  连续 VNADatapoint（0..N-1 严格循环、S11/S22 同包、频率严格递增、reference
  非零）——真实 strict assembler 零修改组装出完整 sweep（0 dropped/invalid）；
- 冰层场景数据模型：表面/浅层/深层三反射器（时延 0 / 1 ns / 2.5 ns、幅度递减）
  + 固定 seed 复高斯噪声；同 seed 两次完整采集 bit 级一致；open() 重置会话；
- 默认扫频节奏对标真机（201 点 / IFBW 10 kHz ≈ 25 道/秒，pace 可关）；
- 模拟路径零 PyUSB/libusb（干净解释器子进程探针）；
- AcquisitionService + 真实 worker 端到端（服务复用/resume/reconfigure/
  待命释放、NPZ/CSV 落盘）；
- MainWindow 设备来源开关：默认 USB、运行中禁用/拒绝切换、往返切换在下次
  连接生效、模拟模式全流程出图。

全部 offscreen Qt + 模拟设备，无真实硬件 / USB。
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import tempfile
import textwrap
import time
import unittest
import unittest.mock
from pathlib import Path

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

try:
    import uav_gpr_qt as air
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover - env dependent
    air = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

from PySide6 import QtCore, QtWidgets  # noqa: E402

from librevna.librevna_continuous import (  # noqa: E402
    BackendState,
    PersistentLibreVnaBackend,
)
from librevna.librevna_errors import LibreVnaTimeoutError  # noqa: E402
from librevna.librevna_protocol import S11_S22_STAGES_BITMAP  # noqa: E402
from librevna.simulated_transport import (  # noqa: E402
    DEEP_DELAY_S,
    DEFAULT_ICE_SCENE_S11,
    SHALLOW_DELAY_S,
    SURFACE_DELAY_S,
    Reflector,
    SimulatedLibreVnaTransport,
    reflector_response,
)
from librevna.librevna_transport import LibreVnaUsbTransport  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent


def _wait_until(predicate, timeout_s: float, interval_s: float = 0.01) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return predicate()


def _wait_until_qt(
    predicate, timeout_s: float, app: QtWidgets.QApplication, interval_s: float = 0.01
) -> bool:
    """带 Qt 事件泵的有界等待（跨线程 queued 信号必须 processEvents 才投递）。"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        app.processEvents()
        if predicate():
            return True
        time.sleep(interval_s)
    app.processEvents()
    return predicate()


def _sweep_settings(points: int = 5, start_mhz: float = 400.0, stop_mhz: float = 4000.0):
    return air.SweepSettings(
        start_hz=int(start_mhz * 1e6),
        stop_hz=int(stop_mhz * 1e6),
        points=points,
        ifbw_hz=10000,
        stages_bitmap=S11_S22_STAGES_BITMAP,
    )


def _configured_backend(transport, points: int = 5) -> PersistentLibreVnaBackend:
    backend = PersistentLibreVnaBackend(transport=transport)
    backend.open()
    backend.configure(_sweep_settings(points=points))
    return backend


def _sweep_config(points: int = 5, ifbw_hz: int = 10000):
    return air.SweepConfig(
        start_mhz=400.0, stop_mhz=4000.0, points=points, ifbw_hz=ifbw_hz
    )


def _make_spec(out_dir: Path, *, count: int = 2, points: int = 5, autosave: bool = False):
    return air.AcquisitionTaskSpec(
        cfg=_sweep_config(points=points),
        interval_s=0.0,
        out_dir=out_dir,
        count=count,
        gnss_port="",
        baud=115200,
        calibration_path="",
        session_calibration=None,
        session_calibration_p2=None,
        through_measurement=None,
        background_frequency_hz=None,
        background=None,
        autosave=autosave,
    )


def _worker_factory(service, **extra):
    def build(task_spec: air.AcquisitionTaskSpec) -> air.ContinuousAcquisitionWorker:
        return air.ContinuousAcquisitionWorker(
            cfg=task_spec.cfg,
            interval_s=task_spec.interval_s,
            out_dir=task_spec.out_dir,
            count=task_spec.count,
            gnss_port=task_spec.gnss_port,
            baud=task_spec.baud,
            calibration_path=task_spec.calibration_path,
            session_calibration=task_spec.session_calibration,
            session_calibration_p2=task_spec.session_calibration_p2,
            through_measurement=task_spec.through_measurement,
            background_frequency_hz=task_spec.background_frequency_hz,
            background=task_spec.background,
            autosave=task_spec.autosave,
            remote_sender=task_spec.remote_sender,
            service=service,
            task_spec=task_spec,
            **extra,
        )

    return build


def _expected_freq_hz(points: int, start_mhz: float, stop_mhz: float) -> np.ndarray:
    return np.linspace(start_mhz * 1e6, stop_mhz * 1e6, points, dtype=np.float64)


# ---------------------------------------------------------------------
# 模拟 transport：协议兼容与物理模型
# ---------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt unavailable: {IMPORT_ERROR}")
class SimulatedTransportProtocolTests(unittest.TestCase):
    """假 LibreVNA 的协议线行为（真实 PersistentLibreVnaBackend 直连）。"""

    def test_backend_open_configure_acquire_end_to_end(self) -> None:
        transport = SimulatedLibreVnaTransport(pace=False)
        backend = _configured_backend(transport)
        first = backend.acquire_one()
        second = backend.acquire_one()
        self.assertEqual(first.trace_index, 0)
        self.assertEqual(second.trace_index, 1)
        self.assertEqual(first.frequency_hz.shape, (5,))
        self.assertEqual(first.raw.shape, (2, 5))  # S11 / S22 双通道
        np.testing.assert_allclose(first.frequency_hz, _expected_freq_hz(5, 400, 4000))
        self.assertTrue(np.all(np.isfinite(first.raw)))
        # 物理合理性：反射响应幅度 <=1（无源反射 + 双通道差异）
        self.assertLessEqual(float(np.abs(first.raw[0]).max()), 1.0)
        self.assertNotEqual(first.raw[0, 0], first.raw[1, 0])
        # strict assembler 零修改即组装成功：两道完整、无丢弃/无效/乱序。
        stats = backend.statistics
        self.assertEqual(stats.completed_sweeps, 2)
        self.assertEqual(stats.dropped_sweeps, 0)
        self.assertEqual(stats.invalid_points, 0)
        self.assertEqual(stats.duplicate_points, 0)
        self.assertEqual(stats.jump_points, 0)
        self.assertEqual(stats.out_of_range_points, 0)
        backend.close()
        self.assertFalse(backend.is_open)

    def test_device_info_advertises_dual_port_and_simulated_hw(self) -> None:
        transport = SimulatedLibreVnaTransport()
        backend = PersistentLibreVnaBackend(transport=transport)
        backend.open()
        info = backend.device_info
        self.assertIsNotNone(info)
        assert info is not None
        self.assertEqual(info.num_ports, 2)
        self.assertEqual(info.min_freq_hz, 100_000)
        self.assertEqual(info.max_freq_hz, 6_000_000_000)
        self.assertGreaterEqual(info.max_points, 10_001)
        self.assertEqual(info.hardware_revision, "S")
        backend.close()

    def test_set_idle_stops_stream_and_resume_restarts(self) -> None:
        transport = SimulatedLibreVnaTransport(pace=False)
        backend = _configured_backend(transport)
        backend.acquire_one()
        backend.pause()
        self.assertEqual(backend.state, BackendState.PAUSED)
        self.assertFalse(transport.streaming)
        backend.resume()
        self.assertEqual(backend.state, BackendState.STREAMING)
        resumed = backend.acquire_one()
        self.assertEqual(resumed.trace_index, 1)
        self.assertEqual(int(resumed.frequency_hz[0]), 400_000_000)
        backend.close()

    def test_sweep_settings_drive_frequency_axis(self) -> None:
        transport = SimulatedLibreVnaTransport(pace=False)
        backend = PersistentLibreVnaBackend(transport=transport)
        backend.open()
        backend.configure(
            _sweep_settings(points=11, start_mhz=800.0, stop_mhz=2600.0)
        )
        sweep = backend.acquire_one()
        expected = _expected_freq_hz(11, 800, 2600)
        np.testing.assert_allclose(sweep.frequency_hz, expected, rtol=0, atol=1)
        backend.close()

    def test_default_pacing_targets_real_device_rate(self) -> None:
        """默认节奏对标真机：201 点 / IFBW 10 kHz ≈ 25 道/秒（可在 8–60 抖动带）。"""
        paced = SimulatedLibreVnaTransport(pace=True)
        backend = PersistentLibreVnaBackend(transport=paced)
        backend.open()
        backend.configure(_sweep_settings(points=201, start_mhz=400.0, stop_mhz=4000.0))
        started = time.monotonic()
        backend.acquire_one()
        paced_sweep_s = time.monotonic() - started
        backend.close()
        self.assertGreaterEqual(paced_sweep_s, 0.02, "pace=True 必须产生真实时长节奏")
        self.assertLessEqual(paced_sweep_s, 0.5, f"201 点一道不应过慢: {paced_sweep_s:.3f}s")

        fast = SimulatedLibreVnaTransport(pace=False)
        backend2 = PersistentLibreVnaBackend(transport=fast)
        backend2.open()
        backend2.configure(_sweep_settings(points=201, start_mhz=400.0, stop_mhz=4000.0))
        started = time.monotonic()
        backend2.acquire_one()
        fast_sweep_s = time.monotonic() - started
        backend2.close()
        self.assertLess(fast_sweep_s, paced_sweep_s / 5.0, "pace=False 必须显著加速")

    def test_deterministic_session_and_noise_respects_seed(self) -> None:
        def _first_sweep(seed: int) -> np.ndarray:
            transport = SimulatedLibreVnaTransport(pace=False, seed=seed)
            backend = _configured_backend(transport)
            raw = np.array(backend.acquire_one().raw, dtype=np.complex128, copy=True)
            backend.close()
            return raw

        same_a = _first_sweep(7)
        same_b = _first_sweep(7)
        np.testing.assert_array_equal(same_a, same_b, "同 seed 会话必须逐位一致")
        other = _first_sweep(8)
        self.assertFalse(np.array_equal(same_a, other), "不同 seed 数据应不同")

    def test_open_restarts_session_state(self) -> None:
        transport = SimulatedLibreVnaTransport(pace=False)
        backend = _configured_backend(transport, points=5)
        backend.acquire_one()
        backend.close()
        self.assertFalse(transport.is_open)
        backend.open()
        backend.configure(_sweep_settings(points=5))
        sweep = backend.acquire_one()
        self.assertEqual(int(sweep.frequency_hz[0]), 400_000_000)
        self.assertEqual(sweep.trace_index, 0, "重新 open 后 trace_index 从 0 开始")
        backend.close()

    def test_reflector_model_math_and_ifft_peak(self) -> None:
        # H(f) = A·exp(-j2πfτ)：幅度守恒、相位 = -2πfτ。
        delay = 1.0e-9
        amplitude = 2.0 + 3.0j
        freq = 1.5e9
        value = reflector_response(freq, (Reflector(delay, amplitude),))
        self.assertAlmostEqual(abs(value), abs(amplitude), places=12)
        ratio = value / amplitude
        expected_phase = -2.0 * math.pi * freq * delay
        actual = math.atan2(ratio.imag, ratio.real)
        phase_diff = (actual - expected_phase) % (2.0 * math.pi)
        if phase_diff > math.pi:
            phase_diff -= 2.0 * math.pi
        self.assertAlmostEqual(phase_diff, 0.0, places=9)

        # 频带采样 IFFT：峰值落在 k ≈ N·Δf·τ。
        n_points = 1001
        freqs = np.linspace(400e6, 4000e6, n_points)
        resp = np.asarray(
            [
                reflector_response(float(f), (Reflector(delay, 1.0 + 0.0j),))
                for f in freqs
            ],
            dtype=np.complex128,
        )
        envelope = np.abs(np.fft.ifft(resp))
        delta_f = freqs[1] - freqs[0]
        expected_index = int(round(n_points * delta_f * delay))
        self.assertEqual(np.argmax(envelope), expected_index)

    def test_default_scene_three_reflectors_decreasing_amplitude(self) -> None:
        """冰层场景：三反射器（表面/浅层/深层，0 / 1 ns / 2.5 ns，幅度递减）。"""
        delays = [r.delay_s for r in DEFAULT_ICE_SCENE_S11]
        self.assertEqual(
            delays, [SURFACE_DELAY_S, SHALLOW_DELAY_S, DEEP_DELAY_S]
        )
        self.assertEqual(delays[1], 1.0e-9)
        self.assertEqual(delays[2], 2.5e-9)
        magnitudes = [abs(r.amplitude) for r in DEFAULT_ICE_SCENE_S11]
        self.assertEqual(magnitudes, sorted(magnitudes, reverse=True), "幅度必须递减")
        self.assertGreater(magnitudes[0], magnitudes[-1])

    def test_read_timeout_when_idle(self) -> None:
        transport = SimulatedLibreVnaTransport()
        transport.open()
        with self.assertRaises(LibreVnaTimeoutError):
            transport.read(512, 1)  # 未配置扫频 = 设备空闲
        transport.close()


# ---------------------------------------------------------------------
# 模拟路径零 PyUSB/libusb（无真机环境完整运行）
# ---------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt unavailable: {IMPORT_ERROR}")
class SimulatedNoHardwareTests(unittest.TestCase):
    """干净解释器探针：模拟装配分支全程零 PyUSB/libusb import/调用。"""

    def test_simulated_protocol_path_never_imports_pyusb(self) -> None:
        script = textwrap.dedent(
            """
            import sys
            sys.path.insert(0, {root!r})
            from librevna.librevna_continuous import PersistentLibreVnaBackend
            from librevna.librevna_protocol import SweepSettings, S11_S22_STAGES_BITMAP
            from librevna.simulated_transport import SimulatedLibreVnaTransport

            backend = PersistentLibreVnaBackend(
                transport=SimulatedLibreVnaTransport(pace=False)
            )
            backend.open()
            backend.configure(
                SweepSettings(
                    start_hz=400_000_000, stop_hz=4_000_000_000, points=5,
                    ifbw_hz=10000, stages_bitmap=S11_S22_STAGES_BITMAP,
                )
            )
            sweep = backend.acquire_one()
            assert sweep.raw.shape == (2, 5), sweep.raw.shape
            backend.close()

            touched = [
                m for m in sys.modules
                if m == "usb" or m.startswith("usb.") or m.startswith("libusb")
            ]
            if touched:
                raise SystemExit("模拟路径触碰了 PyUSB/libusb: %r" % (touched,))
            print("OK-NO-USB")
            """
        ).format(root=str(ROOT))
        proc = subprocess.run(
            [sys.executable, "-c", script],
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=120,
        )
        self.assertEqual(proc.returncode, 0, f"stderr={proc.stderr}\nstdout={proc.stdout}")
        self.assertIn("OK-NO-USB", proc.stdout)

    def test_usb_default_assembly_branches_are_byte_identical_to_round2(self) -> None:
        """USB 默认装配分支源码与第 2 轮逐字一致（两处回退均为同一行构造）。"""
        source = Path(air.__file__).read_text(encoding="utf-8")
        legacy_line = "backend = PersistentLibreVnaBackend(settings=self._backend_settings)"
        self.assertEqual(
            source.count(legacy_line),
            2,
            "USB 默认装配分支必须保留两处与第 2 轮逐字相同的构造行",
        )


# ---------------------------------------------------------------------
# 常驻服务 + 真实 worker：模拟后端端到端（USB 旁路、零适配）
# ---------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt unavailable: {IMPORT_ERROR}")
class SimulatedServiceIntegrationTests(unittest.TestCase):
    """AcquisitionService + ContinuousAcquisitionWorker 在模拟通道上跑真实任务。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="sim-device-"))
        air.set_default_device_source(air.DEVICE_SOURCE_USB)

    def tearDown(self) -> None:
        if hasattr(self, "service"):
            self.service.shutdown(8000)
        air.set_default_device_source(air.DEVICE_SOURCE_USB)

    def _make_service(self):
        holder: dict[str, object] = {}

        def factory() -> PersistentLibreVnaBackend:
            transport = SimulatedLibreVnaTransport(pace=False)
            holder["transport"] = transport
            return PersistentLibreVnaBackend(transport=transport)

        service = air.AcquisitionService(backend_factory=factory)
        self.holder = holder
        return service

    def test_service_run_reuses_simulated_session_across_tasks(self) -> None:
        service = self._make_service()
        self.service = service

        worker = service.start_task(
            _make_spec(self.tmp, count=2), _worker_factory(service)
        )
        self.assertIsNotNone(worker)
        assert worker is not None
        self.assertTrue(worker.wait(15000), "首个模拟任务未在时限内完成")
        transport = self.holder["transport"]
        self.assertIsInstance(transport, SimulatedLibreVnaTransport)
        backend = service.backend
        self.assertIsNotNone(backend)
        assert backend is not None
        self.assertIs(backend.transport, transport)
        stats = service.statistics()
        self.assertEqual(stats["state"], "idle_waiting")
        self.assertGreaterEqual(stats["backend_trace_index"], 2)
        self.assertEqual(stats["backend_open_count"], 1)
        self.assertTrue(transport.is_open, "待命期模拟会话保持打开（R-A-01 语义）")

        # 第二任务：同配置 → resume（不重新 open、不新建 transport）。
        worker2 = service.start_task(
            _make_spec(self.tmp, count=2), _worker_factory(service)
        )
        self.assertIsNotNone(worker2)
        assert worker2 is not None
        self.assertTrue(worker2.wait(15000), "复用会话的模拟任务未完成")
        stats2 = service.statistics()
        self.assertEqual(stats2["backend_open_count"], 1, "resume 不得重新 open")
        self.assertIs(service.backend, backend, "resume 复用同一后端")
        self.assertEqual(stats2["state"], "idle_waiting")

        # 配置变化 → reconfigure（同一后端、同一 transport）。
        worker3 = service.start_task(
            _make_spec(self.tmp, count=1, points=8), _worker_factory(service)
        )
        assert worker3 is not None
        self.assertTrue(worker3.wait(15000), "reconfigure 的模拟任务未完成")
        stats3 = service.statistics()
        self.assertEqual(stats3["backend_open_count"], 1, "reconfigure 不得重新 open")
        self.assertEqual(stats3["state"], "idle_waiting")

    def test_simulated_task_autosaves_npz_csv(self) -> None:
        """模拟模式端到端落盘：自动保存 NPZ/CSV 与 run_metadata 正常。"""
        service = self._make_service()
        self.service = service
        worker = service.start_task(
            _make_spec(self.tmp, count=1, autosave=True), _worker_factory(service)
        )
        assert worker is not None
        self.assertTrue(worker.wait(20000), "模拟落盘任务未在时限内完成")
        self.assertTrue(
            _wait_until(lambda: service.state == "idle_waiting", 10.0),
            f"落盘任务未进入待命（state={service.state}）",
        )
        csvs = list(self.tmp.glob("trace_*.csv"))
        npzs = list(self.tmp.glob("trace_*.npz"))
        metadata = list(self.tmp.glob("run_metadata.jsonl"))
        self.assertTrue(csvs, "模拟数据未落盘 CSV")
        self.assertTrue(npzs, "模拟数据未落盘 NPZ")
        self.assertTrue(metadata, "模拟数据未写 run_metadata.jsonl")

    def test_release_idle_session_switches_to_closed(self) -> None:
        service = self._make_service()
        self.service = service
        worker = service.start_task(
            _make_spec(self.tmp, count=1), _worker_factory(service)
        )
        assert worker is not None
        self.assertTrue(worker.wait(15000))
        self.assertEqual(service.state, "idle_waiting")
        backend = service.backend
        self.assertIsNotNone(backend)
        assert backend is not None
        self.assertTrue(backend.is_open)

        self.assertTrue(service.release_idle_session(), "待命态应可释放")
        self.assertEqual(service.state, "closed")
        self.assertIsNone(service.backend)
        self.assertFalse(backend.is_open, "释放后模拟后端必须关闭")
        # 非待命状态拒绝释放（不伪装成功）。
        self.assertFalse(service.release_idle_session())

    def test_worker_without_service_on_simulated_backend(self) -> None:
        """独立 worker（service=None）注入模拟后端照常工作（第1轮语义不变）。"""
        transport = SimulatedLibreVnaTransport(pace=False)
        backend = PersistentLibreVnaBackend(transport=transport)
        worker = air.ContinuousAcquisitionWorker(
            cfg=_sweep_config(points=5),
            interval_s=0.0,
            out_dir=self.tmp,
            count=2,
            gnss_port="",
            baud=115200,
            calibration_path="",
            session_calibration=None,
            session_calibration_p2=None,
            through_measurement=None,
            background_frequency_hz=None,
            background=None,
            autosave=False,
            backend=backend,
        )
        worker.start()
        self.assertTrue(worker.wait(15000))
        self.assertFalse(backend.is_open, "独立 worker 结束必须释放（第1轮语义）")
        self.assertGreaterEqual(backend.statistics.trace_index, 2)

    def test_worker_self_created_backend_follows_default_source(self) -> None:
        """设备来源装配点②：独立 worker 自建后端服从进程级设备来源。"""
        air.set_default_device_source(air.DEVICE_SOURCE_SIMULATED)
        try:
            worker = air.ContinuousAcquisitionWorker(
                cfg=_sweep_config(points=5),
                interval_s=0.0,
                out_dir=self.tmp,
                count=1,
                gnss_port="",
                baud=115200,
                calibration_path="",
                session_calibration=None,
                session_calibration_p2=None,
                through_measurement=None,
                background_frequency_hz=None,
                background=None,
                autosave=False,
            )
            worker.start()
            self.assertTrue(worker.wait(20000), "自建模拟后端任务未完成")
            self.assertEqual(worker.terminal_state, "drained")
        finally:
            air.set_default_device_source(air.DEVICE_SOURCE_USB)
        backend = worker.backend
        self.assertIsNotNone(backend, "独立 worker 应自建并保留 backend 引用")
        self.assertIsInstance(backend.transport, SimulatedLibreVnaTransport)
        self.assertFalse(backend.is_open, "独立 worker 结束必须释放后端（第1轮语义）")


# ---------------------------------------------------------------------
# MainWindow：设备来源开关
# ---------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt unavailable: {IMPORT_ERROR}")
class AirDeviceSourceSwitchTests(unittest.TestCase):
    """空中端“设置”对话框里的设备来源开关（USB 真机 / 模拟器）。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        air.set_default_device_source(air.DEVICE_SOURCE_USB)
        self.window = air.MainWindow()
        self.window.remote_enable.setChecked(False)
        self.window.continuous_enabled.setChecked(True)

    def tearDown(self) -> None:
        worker = self.window.worker
        if worker is not None and getattr(worker, "isRunning", lambda: False)():
            worker.stop()
            worker.wait(5000)
        self.window.close()
        self.window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        self.app.processEvents()
        air.set_default_device_source(air.DEVICE_SOURCE_USB)

    def test_device_source_combo_present_in_settings_dialog_default_usb(self) -> None:
        combo = self.window.device_source_combo
        self.assertEqual(combo.objectName(), "device_source_input")
        self.assertEqual(combo.currentData(), air.DEVICE_SOURCE_USB)
        self.assertEqual(self.window.device_source(), air.DEVICE_SOURCE_USB)
        self.assertEqual(air.get_default_device_source(), air.DEVICE_SOURCE_USB)
        # 宿主在非模态“空中端设置”对话框中。
        descendants = set(self.window.air_settings_dialog.findChildren(QtWidgets.QWidget))
        self.assertIn(combo, descendants, "设备来源开关必须挂在设置对话框内")
        # 两个选项：USB 真机 / 模拟器。
        self.assertEqual(
            set(combo.itemData(i) for i in range(combo.count())),
            {air.DEVICE_SOURCE_USB, air.DEVICE_SOURCE_SIMULATED},
        )

    def test_backend_factory_switches_transport_by_source(self) -> None:
        # 默认 USB：transport=None → 后端默认真机 USB transport。
        usb_backend = self.window._make_acquisition_backend()
        self.assertIsInstance(usb_backend.transport, LibreVnaUsbTransport)
        usb_backend.close()

        sim_index = self.window.device_source_combo.findData(air.DEVICE_SOURCE_SIMULATED)
        self.window.device_source_combo.setCurrentIndex(sim_index)
        self.assertEqual(self.window.device_source(), air.DEVICE_SOURCE_SIMULATED)
        self.assertEqual(air.get_default_device_source(), air.DEVICE_SOURCE_SIMULATED)
        sim_backend = self.window._make_acquisition_backend()
        self.assertIsInstance(sim_backend.transport, SimulatedLibreVnaTransport)
        sim_backend.close()

    def test_source_round_trip_applies_to_next_connection(self) -> None:
        """设置往返：USB→模拟→USB 后来源复位，下次连接（工厂）回到真机。"""
        combo = self.window.device_source_combo
        combo.setCurrentIndex(combo.findData(air.DEVICE_SOURCE_SIMULATED))
        self.assertEqual(self.window.device_source(), air.DEVICE_SOURCE_SIMULATED)
        sim_backend = self.window._make_acquisition_backend()
        self.assertIsInstance(sim_backend.transport, SimulatedLibreVnaTransport)
        sim_backend.close()

        combo.setCurrentIndex(combo.findData(air.DEVICE_SOURCE_USB))
        self.assertEqual(self.window.device_source(), air.DEVICE_SOURCE_USB)
        self.assertEqual(air.get_default_device_source(), air.DEVICE_SOURCE_USB)
        # 下次连接（新建后端）按最终选择走真机（与改动前逐字一致）。
        usb_backend = self.window._make_acquisition_backend()
        self.assertIsInstance(usb_backend.transport, LibreVnaUsbTransport)
        usb_backend.close()
        log_text = self.window.device_log.toPlainText()
        self.assertIn("设备来源已切换", log_text)

    def test_switch_rejected_while_task_running(self) -> None:
        combo = self.window.device_source_combo
        combo.setCurrentIndex(combo.findData(air.DEVICE_SOURCE_SIMULATED))
        self.assertEqual(self.window.device_source(), air.DEVICE_SOURCE_SIMULATED)
        # 任务运行中：切换回 USB 被拒绝并回退。
        self.window.worker = unittest.mock.Mock()
        self.window.worker.isRunning.return_value = True
        combo.setCurrentIndex(combo.findData(air.DEVICE_SOURCE_USB))
        self.assertEqual(
            self.window.device_source(), air.DEVICE_SOURCE_SIMULATED,
            "任务运行中设备来源必须保持原值",
        )
        self.assertEqual(combo.currentData(), air.DEVICE_SOURCE_SIMULATED)
        self.window.worker = None

    def test_source_combo_locked_during_task(self) -> None:
        self.window._set_task_controls_locked(True)
        try:
            self.assertFalse(self.window.device_source_combo.isEnabled())
        finally:
            self.window._set_task_controls_locked(False)

    def test_full_ui_flow_on_simulated_device_then_switch_back(self) -> None:
        """设置→模拟器→开始→B-scan 出图→停止待命→切回 USB 释放会话。"""
        out_dir = Path(tempfile.mkdtemp(prefix="sim-ui-"))
        self.window.output_dir.setText(str(out_dir))
        self.window.points.setValue(11)
        self.window.start_mhz.setValue(400.0)
        self.window.stop_mhz.setValue(600.0)
        self.window.count.setValue(2)

        combo = self.window.device_source_combo
        combo.setCurrentIndex(combo.findData(air.DEVICE_SOURCE_SIMULATED))
        self.assertEqual(self.window.device_source(), air.DEVICE_SOURCE_SIMULATED)

        self.window.start_acquisition()
        worker = self.window.worker
        self.assertIsNotNone(worker)
        assert worker is not None
        self.assertTrue(
            _wait_until_qt(
                lambda: self.window.bscan_trace_count() >= 2, 30.0, self.app
            ),
            f"模拟通道 2 条 trace 未出图（当前 {self.window.bscan_trace_count()} 道；"
            f"日志尾={self.window.device_log.toPlainText()[-300:]!r}）",
        )
        service = self.window.acquisition_service
        # count 达到后 worker 排空退出、服务登记待命（等待真正进入 idle）。
        self.assertTrue(
            _wait_until(lambda: not worker.isRunning(), 20.0),
            "模拟任务 worker 未在时限内退出",
        )
        self.assertTrue(
            _wait_until(lambda: service.state == air.SERVICE_STATE_IDLE, 10.0),
            f"count 达到后未进入待命（state={service.state}）",
        )
        backend = service.backend
        self.assertIsNotNone(backend)
        assert backend is not None
        self.assertIsInstance(backend.transport, SimulatedLibreVnaTransport)
        self.assertGreaterEqual(backend.statistics.trace_index, 2)
        log_text = self.window.device_log.toPlainText()
        self.assertIn("模拟器", log_text, "日志应记录设备来源=模拟器")
        # 待命期切换回 USB：先释放模拟会话再切换（传输层不可热更换）。
        combo.setCurrentIndex(combo.findData(air.DEVICE_SOURCE_USB))
        self.assertEqual(self.window.device_source(), air.DEVICE_SOURCE_USB)
        self.assertEqual(service.state, "closed")
        self.assertIsNone(service.backend)
        self.assertFalse(backend.is_open, "切换来源后旧模拟会话必须已释放")


if __name__ == "__main__":
    unittest.main()
