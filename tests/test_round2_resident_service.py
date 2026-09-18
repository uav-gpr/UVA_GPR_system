"""第2轮 t3：常驻采集服务（docs/round2_requirements_and_interfaces.md §3）。

覆盖条款：R-A-01…R-A-09（会话复用 / SET_IDLE / reconfigure / 边界 / 故障）、
R-A-10…R-A-18（任务身份制 + engB 死锁回归固化）、R-A-13、R-A-07。
全部使用 fake transport / 真实 ReceiverThread，无真机；offscreen Qt。
蓝本参考：docs/round2_repro_runid_collision.py（场景 A 冲突死锁 / B 幂等吸收）。
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
import unittest.mock
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

try:
    import uav_gpr_qt as air
except Exception as exc:  # noqa: BLE001 - 依赖缺失时整体跳过
    air = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

import uav_gpr_ground_station as ground
from librevna.librevna_continuous import BackendState, PersistentLibreVnaBackend
from librevna.librevna_errors import LibreVnaTimeoutError
from librevna.librevna_protocol import (
    ACK,
    DEVICE_INFO,
    REQUEST_DEVICE_INFO,
    SET_IDLE,
    SWEEP_SETTINGS,
    VNA_DATAPOINT,
    encode_packet,
)

DIRECT = QtCore.Qt.ConnectionType.DirectConnection


# ---------------------------------------------------------------------
# fake 设备：记录命令序列与 open/close/read 次数
# ---------------------------------------------------------------------


def _device_info_payload(num_ports: int = 2) -> bytes:
    """合法 DeviceInfo payload（与既有测试同构：num_ports=2、max_points=65535）。"""
    return struct.pack(
        "<HBBBBcQQIIHhhIIBQBH",
        14, 1, 2, 3, 5, b"A",
        100_000_000, 6_000_000_000, 1, 1_000_000, 10_001,
        -3000, 1000, 1, 1_000_000, 200, 12_000_000_000, num_ports, 65_535,
    )


def _point_packet(freq_hz: int, point: int, s11: complex, s22: complex) -> bytes:
    """一个 VNADatapoint（protocol 14 双阶段布局，与既有测试同构）。"""
    receivers = [
        (0x01, s11),
        (0x02, complex(9.0, 9.0)),
        (0x13, complex(1.0, 0.0)),
        (0x21, complex(7.0, 7.0)),
        (0x22, s22),
        (0x33, complex(1.0, 0.0)),
    ]
    n = len(receivers)
    payload = struct.pack("<QhH", int(freq_hz), -1000, int(point))
    payload += struct.pack("<" + "f" * n, *[v.real for _, v in receivers])
    payload += struct.pack("<" + "f" * n, *[v.imag for _, v in receivers])
    payload += bytes(d for d, _ in receivers)
    return encode_packet(VNA_DATAPOINT, payload)


class RecordingTransport:
    """连续扫频 fake 设备。"""

    def __init__(self, n_points: int = 5, *, max_sweeps: int | None = None) -> None:
        self.n_points = n_points
        self.max_sweeps = max_sweeps
        self.open_count = 0
        self.close_count = 0
        self.read_count = 0
        self.commands: list[str] = []
        self.point = 0
        self.sweeps_generated = 0
        self.opened = False
        self._reply: str | None = None

    def open(self) -> None:
        self.open_count += 1
        self.opened = True

    def close(self) -> None:
        self.close_count += 1
        self.opened = False

    @property
    def is_open(self) -> bool:
        return self.opened

    def write(self, data: bytes) -> None:
        if not self.opened:
            raise LibreVnaTimeoutError("not open")
        packet_type = data[3]
        if packet_type == REQUEST_DEVICE_INFO:
            self._reply = "device_info"
            self.commands.append("device_info")
        elif packet_type == SET_IDLE:
            self._reply = "ack"
            self.commands.append("set_idle")
        elif packet_type == SWEEP_SETTINGS:
            self._reply = "ack"
            self.commands.append("sweep_settings")
        else:
            self.commands.append(f"type:{packet_type}")

    def read(self, max_length: int, timeout_ms: int) -> bytes:
        if not self.opened:
            raise LibreVnaTimeoutError("not open")
        self.read_count += 1
        if self._reply == "device_info":
            self._reply = None
            return encode_packet(DEVICE_INFO, _device_info_payload())
        if self._reply == "ack":
            self._reply = None
            return encode_packet(ACK)
        if self.max_sweeps is not None and self.sweeps_generated >= self.max_sweeps:
            raise LibreVnaTimeoutError("exhausted")
        point = self.point
        self.point += 1
        if self.point >= self.n_points:
            self.point = 0
            self.sweeps_generated += 1
        return _point_packet(
            500_000_000 + point * 10_000_000, point, complex(0.5, -0.2), complex(0.3, 0.1)
        )


def _wait_until(predicate, timeout_s: float, interval_s: float = 0.01) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return predicate()


def _sweep_config(points: int = 5, start_mhz: float = 400.0):
    return air.SweepConfig(
        points=points, start_mhz=start_mhz, stop_mhz=4000.0, ifbw_hz=10000
    )


def _make_spec(out_dir: Path, *, count: int = 3, cfg=None, **overrides):
    kwargs = dict(
        cfg=cfg or _sweep_config(),
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
        autosave=False,
    )
    kwargs.update(overrides)
    return air.AcquisitionTaskSpec(**kwargs)


def _make_service(transport) -> "air.AcquisitionService":
    return air.AcquisitionService(
        backend_factory=lambda: PersistentLibreVnaBackend(transport=transport)
    )


def _worker_factory(service, **extra):
    def build(task_spec):
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


class _CloseFailsTransport(RecordingTransport):
    """close() 抛错且仍报告打开的 fake（释放失败路径）。"""

    def __init__(self, n_points: int = 5) -> None:
        super().__init__(n_points)
        self.closable = False

    def close(self) -> None:
        self.close_count += 1
        if not self.closable:
            raise RuntimeError("注入：release_interface 失败")
        self.opened = False


def _configured_backend(transport) -> PersistentLibreVnaBackend:
    backend = PersistentLibreVnaBackend(transport=transport)
    backend.open()
    backend.configure(
        air.SweepSettings(
            start_hz=400_000_000,
            stop_hz=4_000_000_000,
            points=5,
            ifbw_hz=10000,
            stages_bitmap=0x1241,
        )
    )
    return backend


class _NoOpWorker:
    """服务登记用最小 worker 替身（不启动线程）。"""

    def __init__(self) -> None:
        self.stop_called = 0

    def isRunning(self) -> bool:
        return False

    def request_stop(self) -> None:
        self.stop_called += 1

    def wait(self, _ms: int) -> bool:
        return True


# ---------------------------------------------------------------------
# R-A-01/02/03/04/08/09/12：常驻会话
# ---------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt unavailable: {IMPORT_ERROR}")
class ResidentSessionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="round2-resident-"))
        self.transport = RecordingTransport(5)
        self.service = _make_service(self.transport)

    def tearDown(self) -> None:
        self.service.shutdown(8000)

    def _finish(self, worker) -> None:
        self.assertTrue(worker.wait(10000), "worker 未在 10s 内退出")

    def test_two_tasks_reuse_single_usb_session(self) -> None:
        """R-A-01/R-A-02：连续两任务仅首次 open/configure；停止不释放 USB。"""
        w1 = self.service.start_task(
            _make_spec(self.tmp, count=3), _worker_factory(self.service)
        )
        self.assertIsNotNone(w1)
        self._finish(w1)
        self.assertEqual(w1.terminal_state, "drained")
        self.assertEqual(
            self.service.state, "idle_waiting", f"实际 {self.service.statistics()}"
        )
        self.assertTrue(self.transport.is_open, "任务停止后 USB 不得释放")
        self.assertEqual(self.transport.close_count, 0, "任务停止不得调用 transport.close")
        backend = self.service.backend
        self.assertEqual(backend.statistics.open_count, 1)
        self.assertEqual(backend.statistics.configure_count, 1)
        self.assertEqual(backend.state, BackendState.PAUSED, "停止后必须停在 SET_IDLE 待命态")

        w2 = self.service.start_task(
            _make_spec(self.tmp, count=3), _worker_factory(self.service)
        )
        self._finish(w2)
        self.assertEqual(w2.terminal_state, "drained")
        self.assertEqual(self.transport.open_count, 1, "第二个任务不得重新 open")
        self.assertEqual(backend.statistics.open_count, 1)
        self.assertEqual(backend.statistics.configure_count, 1, "配置相同不得重复 configure")
        self.assertEqual(backend.statistics.completed_sweeps, 6)

    def test_stop_issues_set_idle_and_leaves_no_stream(self) -> None:
        """R-A-03：停止边界下发 SET_IDLE；此后不再有扫频数据流读取。"""
        w1 = self.service.start_task(
            _make_spec(self.tmp, count=3), _worker_factory(self.service)
        )
        self._finish(w1)
        self.assertEqual(self.transport.commands.count("sweep_settings"), 1)
        self.assertGreaterEqual(
            self.transport.commands.count("set_idle"), 2,
            f"open 与停止边界都必须有 SET_IDLE: {self.transport.commands}",
        )
        after_reads = self.transport.read_count
        time.sleep(0.2)
        self.assertEqual(self.transport.read_count, after_reads, "待命期不得残留扫频流读取")
        w2 = self.service.start_task(
            _make_spec(self.tmp, count=2), _worker_factory(self.service)
        )
        self._finish(w2)
        self.assertEqual(self.transport.open_count, 1, "resume 路径不得重新 open")

    def test_config_change_uses_reconfigure_not_reopen(self) -> None:
        """R-A-04：配置字节变化 → reconfigure（open 不变、configure+1、统计连续）。"""
        w1 = self.service.start_task(
            _make_spec(self.tmp, count=2, cfg=_sweep_config(5, 400.0)),
            _worker_factory(self.service),
        )
        self._finish(w1)
        backend = self.service.backend
        self.assertEqual(backend.statistics.completed_sweeps, 2)

        w2 = self.service.start_task(
            _make_spec(self.tmp, count=3, cfg=_sweep_config(5, 500.0)),
            _worker_factory(self.service),
        )
        self._finish(w2)
        self.assertEqual(self.transport.open_count, 1, "reconfigure 不得 reopen")
        self.assertEqual(backend.statistics.open_count, 1)
        self.assertEqual(backend.statistics.configure_count, 2, "reconfigure 必须计一次 configure")
        self.assertEqual(backend.statistics.completed_sweeps, 5, "诊断统计必须跨 reconfigure 折叠累计")
        self.assertEqual(backend.statistics.trace_index, 5, "trace_index 会话内连续")
        self.assertEqual(backend.config.start_hz, 500_000_000, "冻结配置必须更新为新配置")

    def test_boundaries_do_not_splice_across_sessions(self) -> None:
        """R-A-08：stop→start 边界丢弃未完成 sweep，绝不跨会话拼接。"""
        w1 = self.service.start_task(
            _make_spec(self.tmp, count=2), _worker_factory(self.service)
        )
        self._finish(w1)
        backend = self.service.backend
        self.assertEqual(backend.statistics.pending_depth_current, 0, "边界必须清空 pending")
        before = backend.statistics.completed_sweeps
        w2 = self.service.start_task(
            _make_spec(self.tmp, count=1), _worker_factory(self.service)
        )
        self._finish(w2)
        self.assertEqual(backend.statistics.completed_sweeps, before + 1, "新任务只能贡献整道")
        self.assertEqual(
            backend.statistics.jump_points + backend.statistics.duplicate_points, 0,
            "跨会话拼接必然表现为跳号/重复点",
        )

    def test_pause_failure_releases_and_does_not_fake_idle(self) -> None:
        """R-A-09/R-A-06：待命失败 → fail-closed 释放，绝不伪装待命成功。"""
        backend = _configured_backend(self.transport)
        service = air.AcquisitionService(backend_factory=lambda: backend)
        service.attach_task(_NoOpWorker())
        service._backend = backend  # noqa: SLF001 - 模拟 open_and_start 已持有该会话
        self.assertEqual(service.state, "acquiring")

        def _boom():
            raise LibreVnaTimeoutError("注入：SET_IDLE 无 ACK")

        backend.pause = _boom  # type: ignore[assignment]
        result = service.pause_stream(backend)
        self.assertIn(result, ("faulted_released", "faulted"), f"未如实报告故障: {result}")
        self.assertFalse(backend.is_open, "故障会话必须释放，绝不带着半故障待命")
        self.assertEqual(service.state, "closed")
        self.assertFalse(service.wait_idle(200), "故障后不得报告已进入待命")

    def test_pause_stream_rejects_foreign_backend(self) -> None:
        """服务只认自己持有的会话：外来 backend 必须拒绝（防双会话误用）。"""
        mine = _configured_backend(self.transport)
        other = PersistentLibreVnaBackend(transport=RecordingTransport(5))
        other.open()
        service = air.AcquisitionService(backend_factory=lambda: mine)
        service.attach_task(_NoOpWorker())
        service._backend = mine  # noqa: SLF001
        self.assertEqual(service.pause_stream(other), "stale")
        self.assertTrue(mine.is_open, "拒绝外来会话不得副作用自己的会话")
        service.shutdown(3000)
        other.close()

    def test_idle_only_after_drain(self) -> None:
        """R-A-06：SET_IDLE 成功但排空未完成时，状态不得提前变 idle_waiting。"""
        backend = _configured_backend(self.transport)
        service = air.AcquisitionService(backend_factory=lambda: backend)
        service.attach_task(_NoOpWorker())
        service._backend = backend  # noqa: SLF001
        paused = service.pause_stream(backend)
        self.assertEqual(paused, "paused")
        self.assertEqual(service.state, "acquiring", "排空前不得登记待命（不伪装）")
        self.assertTrue(backend.is_open, "pause 成功但任务未结束：USB 仍持有")
        self.assertFalse(service.wait_idle(150))
        # 排空完成登记后才进入待命。
        service.finish_task(_NoOpWorker(), terminal_state="drained")

    def test_failed_task_releases_instead_of_idle(self) -> None:
        """R-A-09（控制流锁定）：terminal!=drained 绝不登记待命，必须释放。

        本测试抓回的是本次实现中真实出现的缺陷：finish_task 分支曾写成
        if/if/elif/else，失败路径设好 fault 标记后仍会落入 else 被登记为
        IDLE_WAITING（带着坏会话"待命"）。
        """
        backend = _configured_backend(self.transport)
        service = air.AcquisitionService(backend_factory=lambda: backend)
        service._backend = backend  # noqa: SLF001 - 模拟服务持有该会话
        worker = _NoOpWorker()
        service.attach_task(worker)
        result = service.finish_task(worker, terminal_state="failed")  # type: ignore[arg-type]
        self.assertNotEqual(result, "idle_waiting", "失败任务绝不允许登记待命")
        self.assertNotEqual(service.state, "idle_waiting")
        self.assertEqual(service.state, "closed", f"失败必须释放并回 closed: {result}")
        self.assertFalse(backend.is_open, "失败任务后 USB 必须真正释放")
        self.assertGreaterEqual(self.transport.close_count, 1)

    def test_unknown_terminal_also_releases(self) -> None:
        """R-A-09/R-X-05：unknown 终态同样不得伪装成待命。"""
        backend = _configured_backend(self.transport)
        service = air.AcquisitionService(backend_factory=lambda: backend)
        service._backend = backend  # noqa: SLF001
        worker = _NoOpWorker()
        service.attach_task(worker)
        service.finish_task(worker, terminal_state="unknown")  # type: ignore[arg-type]
        self.assertEqual(service.state, "closed")
        self.assertFalse(backend.is_open)

    def test_task_seq_carried_by_spec_and_only_in_metadata(self) -> None:
        """R-A-01 配套：服务签发 task_seq；它只进 acquisition 嵌套 metadata。"""
        sender = RecordingSender()
        worker = self.service.start_task(
            _make_spec(self.tmp, count=1, remote_sender=sender),
            _worker_factory(self.service),
        )
        self.assertTrue(worker.wait(10000))
        self.assertEqual(
            worker._task_spec.task_seq, 1,  # noqa: SLF001 - 断言服务签发值
            f"task_seq 应为 1，实际 {worker._task_spec.task_seq}",
        )
        self.assertTrue(sender.traces, "应有 trace 发出")

        freq = np.array([400e6, 1300e6, 2200e6, 3100e6, 4000e6], dtype=float)
        raw = np.zeros((2, 5), dtype=np.complex128)
        base = dict(
            trace_index=1, device_sweep_index=0, frequency_hz=freq, raw=raw,
            cfg=_sweep_config(), midpoint_monotonic=1.0, selected_at_utc="t",
            missed_slots=0, actual_interval_s=None, schedule_error_ms=None,
        )
        service_item = air.SelectedSweep(**base, task_seq=3)
        metadata = air.ProcessingConsumer._acquisition_metadata(
            service_item, air.GnssFixLookup(None, None, False)
        )
        self.assertEqual(metadata["task_seq"], 3)
        legacy_metadata = air.ProcessingConsumer._acquisition_metadata(
            air.SelectedSweep(**base), air.GnssFixLookup(None, None, False)
        )
        self.assertNotIn(
            "task_seq", legacy_metadata,
            "独立 worker（task_seq=0）路径的 acquisition 键集合必须与第 1 轮一致",
        )

    def test_worker_without_service_keeps_round1_release(self) -> None:
        """R-A-12：service=None 独立 worker 路径行为不变（自 open、自 close）。"""
        transport = RecordingTransport(5)
        backend = PersistentLibreVnaBackend(transport=transport)
        worker = air.ContinuousAcquisitionWorker(
            cfg=_sweep_config(),
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
        self.assertTrue(worker.wait(10000))
        self.assertFalse(backend.is_open, "独立 worker 任务结束必须释放 USB（第1轮语义）")
        self.assertEqual(transport.close_count, 1)
        self.assertEqual(transport.open_count, 1)


# ---------------------------------------------------------------------
# R-A-05 / R-A-06 / R-A-07：统一命令入口、超时不伪装、关闭才释放
# ---------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt unavailable: {IMPORT_ERROR}")
class ServiceCommandTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="round2-cmd-"))
        self.transport = RecordingTransport(5)
        self.service = _make_service(self.transport)

    def tearDown(self) -> None:
        self.service.shutdown(8000)

    def test_hard_limit_controlled_stop_is_a_service_command(self) -> None:
        """R-A-05：队列硬上限的受控停止已升级为服务命令（原因登记在服务上）。"""
        holder: dict[str, object] = {}

        class _BlockingConsumer(QtCore.QThread):
            trace_ready = QtCore.Signal(int, object, object, object, object, object, object, object, str)
            status = QtCore.Signal(str)
            failed = QtCore.Signal(str)
            fatal_error = QtCore.Signal(str)

            def __init__(self, queue, cache) -> None:
                super().__init__()
                self._q = queue
                self._stop = threading.Event()
                self.processed_count = 0
                self.drain_state = "running"
                self.drain_failures: list[str] = []
                self.started_consuming = threading.Event()
                self.release = threading.Event()

            def request_stop(self) -> None:
                self._stop.set()
                self.release.set()

            def run(self) -> None:
                # 先让队列填满（阻塞消费者），再排空。
                while not self._stop.is_set():
                    item = self._q.get(timeout=0.02)
                    if item is not None:
                        self.started_consuming.set()
                        self.release.wait(15)
                        self.processed_count += 1
                self.drain_state = "draining"
                while True:
                    item = self._q.get(timeout=0.05)
                    if item is None:
                        break
                    self.processed_count += 1
                self.drain_state = "drained"

        def factory(queue, cache):
            consumer = _BlockingConsumer(queue, cache)
            holder["consumer"] = consumer
            return consumer

        worker = self.service.start_task(
            _make_spec(self.tmp, count=0),
            _worker_factory(self.service, consumer_factory=factory,
                            processing_queue_maxsize=2),
        )
        self.assertIsNotNone(worker)
        self.assertTrue(
            _wait_until(lambda: "consumer" in holder, 5.0), "消费者未创建"
        )
        consumer = holder["consumer"]
        self.assertTrue(
            _wait_until(lambda: worker.processing_queue.stop_requested.is_set(), 10.0),
            "硬上限未触发受控停止",
        )
        self.assertTrue(
            _wait_until(lambda: self.service.state in ("stopping", "idle_waiting"), 10.0),
            f"受控停止未进入服务状态机: {self.service.statistics()}",
        )
        consumer.release.set()
        self.assertTrue(worker.wait(15000))
        reason = self.service.last_stop_reason or ""
        self.assertIn("硬上限", reason, f"服务未登记受控停止原因: {reason!r}")
        self.assertEqual(self.service.state, "idle_waiting")

    def test_request_stop_rejected_when_no_task(self) -> None:
        """R-A-05：空闲/待命时 request_stop 返回 False（不接受命令），不伪装。"""
        self.assertFalse(self.service.request_stop("无任务"))
        self.assertEqual(self.service.state, "closed")

    def test_stop_timeout_does_not_fake_idle(self) -> None:
        """R-A-06：有界等待内未进入待命 → 不伪装成功，且卡点可观测。"""
        self.service.attach_task(_NoOpWorker())
        self.service._state = "acquiring"  # noqa: SLF001 - 直接构造停止场景
        self.assertTrue(self.service.request_stop("操作员"))
        self.assertEqual(self.service.state, "stopping")
        self.assertFalse(self.service.wait_idle(200), "未进入待命时不得报告成功")
        stats = self.service.statistics()
        self.assertEqual(stats["state"], "stopping")
        self.assertEqual(stats["pending_stop_reason"], "操作员")
        # 收尾后正常登记待命。
        self.service.finish_task(_NoOpWorker(), terminal_state="drained")

    def test_shutdown_is_the_only_release_path(self) -> None:
        """R-A-07：只有 shutdown 真正释放；未启动时幂等。"""
        backend = _configured_backend(self.transport)
        self.service._backend = backend  # noqa: SLF001
        self.service._state = "idle_waiting"  # noqa: SLF001
        self.assertTrue(backend.is_open)
        self.assertTrue(self.service.shutdown(5000))
        self.assertEqual(self.service.state, "closed")
        self.assertFalse(self.transport.is_open)
        self.assertEqual(self.transport.close_count, 1)
        self.assertTrue(self.service.shutdown(1000), "重复 shutdown 必须幂等")

    def test_release_failure_reports_and_allows_retry(self) -> None:
        """R-A-07/R-X-04：释放失败不伪装成功、可重试、绝不 terminate。"""

        transport = _CloseFailsTransport(5)
        backend = _configured_backend(transport)
        service = air.AcquisitionService(backend_factory=lambda: backend)
        service._backend = backend  # noqa: SLF001
        service._state = "idle_waiting"  # noqa: SLF001
        transport.closable = False
        self.assertFalse(service.shutdown(400), "资源未释放时不得报告关闭成功")
        self.assertEqual(service.state, "shutting_down")
        self.assertGreaterEqual(service.statistics()["release_failures"], 1)
        transport.closable = True
        self.assertTrue(service.shutdown(3000))
        self.assertEqual(service.state, "closed")
        self.assertFalse(transport.is_open)


# ---------------------------------------------------------------------
# R-A-10 / R-A-11 / R-A-15：任务身份制
# ---------------------------------------------------------------------


class RecordingSender:
    """记录 enqueue_trace / enqueue_status 调用（含携带身份）的 sender 替身。"""

    def __init__(self, run_id: str = "conn-idle") -> None:
        self.run_id = run_id
        self.traces: list[tuple[int, str | None]] = []
        self.statuses: list[tuple[str, str | None]] = []

    def enqueue_trace(self, index, *args, **kwargs) -> None:
        self.traces.append((int(index), kwargs.get("run_id")))

    def enqueue_status(self, state, *, reason=None, run_id=None, **fields) -> None:
        self.statuses.append((state, run_id))

    def outstanding_count(self) -> int:
        return 0

    def stop(self, timeout_ms: int = 0, *, drain: bool = True) -> bool:
        return True


@unittest.skipIf(air is None, f"uav_gpr_qt unavailable: {IMPORT_ERROR}")
class TaskIdentityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="round2-identity-"))
        self.transport = RecordingTransport(5)
        self.service = _make_service(self.transport)

    def tearDown(self) -> None:
        self.service.shutdown(8000)

    def test_run_id_format_seq_monotonic_and_unique(self) -> None:
        """R-A-15：`%04d` 填充、seq 单调、同秒不重复、idle 身份不消耗 seq。"""
        idle_id = self.service.new_run_id("idle")
        self.assertRegex(idle_id, r"^\d{8}_\d{6}_idle$")
        a = self.service.new_run_id("task")
        b = self.service.new_run_id("task")
        self.assertRegex(a, r"^\d{8}_\d{6}_\d{4}$", "seq 必须四位零填充（§3.7-1）")
        self.assertNotEqual(a, b, "同一秒内两个任务身份不得相同")
        seq_a = int(a.rsplit("_", 1)[1])
        seq_b = int(b.rsplit("_", 1)[1])
        self.assertEqual(seq_b, seq_a + 1, "seq 必须单调 +1")
        self.assertRegex(self.service.new_run_id("calibration"), r"^\d{8}_\d{6}_\d{4}_c$")
        # 字典序 == 数值序（outbox 扫描排序前提）。
        self.assertLess(a, b)

    def test_only_one_run_id_issuing_point(self) -> None:
        """R-A-15：生产代码不得出现第二处 run_id 拼接（散点收编）。"""
        import inspect

        source = inspect.getsource(sys.modules["uav_gpr_qt"])
        occurrences = [
            line.strip()
            for line in source.splitlines()
            if 'strftime("%Y%m%d_%H%M%S")' in line
        ]
        self.assertEqual(len(occurrences), 1, f"run_id 生成点必须唯一: {occurrences}")

    def test_task_identity_bound_and_carried_by_all_messages(self) -> None:
        """R-A-10/R-A-11：trace 与全部状态消息携带本任务身份；acquiring 必发且第一。"""
        sender = RecordingSender()
        worker = self.service.start_task(
            _make_spec(self.tmp, count=2, remote_sender=sender),
            _worker_factory(self.service),
        )
        self.assertTrue(worker.wait(10000))
        run_id = worker._task_spec.run_id  # noqa: SLF001
        self.assertRegex(run_id, r"^\d{8}_\d{6}_\d{4}$")
        self.assertEqual(
            [state for state, _ in sender.statuses][:1], ["acquiring"],
            f"acquiring 必须是本任务第一条状态消息: {sender.statuses}",
        )
        self.assertEqual(sender.statuses[0][1], run_id)
        self.assertTrue(sender.traces, "任务应产生遥测 trace")
        for _index, trace_run_id in sender.traces:
            self.assertEqual(trace_run_id, run_id, "trace 必须携带本任务身份（§3.7-3）")
        final_state, final_run = sender.statuses[-1]
        self.assertEqual(final_state, "stopped")
        self.assertEqual(final_run, run_id, "终态必须携带产生任务的 run_id")
        # 任务内编号仍从 1 连续（第 1 轮语义不变）。
        self.assertEqual([i for i, _ in sender.traces], [1, 2])

    def test_two_tasks_same_connection_disjoint_identities(self) -> None:
        """R-A-16（身份部分）：同一常驻 sender 上两任务身份互不相交。"""
        sender = RecordingSender()
        w1 = self.service.start_task(
            _make_spec(self.tmp, count=2, remote_sender=sender), _worker_factory(self.service)
        )
        self.assertTrue(w1.wait(10000))
        w2 = self.service.start_task(
            _make_spec(self.tmp, count=2, remote_sender=sender), _worker_factory(self.service)
        )
        self.assertTrue(w2.wait(10000))
        id1 = w1._task_spec.run_id  # noqa: SLF001
        id2 = w2._task_spec.run_id  # noqa: SLF001
        self.assertNotEqual(id1, id2)
        pairs1 = {idx for idx, rid in sender.traces if rid == id1}
        pairs2 = {idx for idx, rid in sender.traces if rid == id2}
        self.assertEqual(sorted(pairs1), [1, 2])
        self.assertEqual(sorted(pairs2), [1, 2])
        self.assertNotEqual(id1, id2, "(run_id, trace_index) 身份集合必须不相交")
        # 第 1 轮停止序列 acquiring→stopping→stopped 必须逐任务保持（[冻结]）。
        states = [s for s, _ in sender.statuses]
        self.assertEqual(
            states,
            ["acquiring", "stopping", "stopped", "acquiring", "stopping", "stopped"],
        )
        self.assertEqual(
            [rid for _, rid in sender.statuses], [id1, id1, id1, id2, id2, id2],
            "同一任务的全部状态消息必须携带同一身份，且任务间顺序串行",
        )

    def test_calibration_task_is_status_silent(self) -> None:
        """§5.4/R-A-11：status_reporting=False 的校准短任务完全不发状态消息。"""
        sender = RecordingSender()
        worker = self.service.start_task(
            _make_spec(self.tmp, count=2, remote_sender=sender, status_reporting=False),
            _worker_factory(self.service),
        )
        self.assertTrue(worker.wait(10000))
        self.assertEqual(sender.statuses, [], "校准短任务不得发送 acquisition_status")
        self.assertTrue(sender.traces, "trace 仍携带身份发送")

    def test_default_run_id_path_is_round1_compatible(self) -> None:
        """R-A-15 [冻结]：run_id=None 时消息取连接身份（第 1 轮行为逐字不变）。"""
        server = _MiniGround()
        try:
            sender = air.RemoteSender(
                "127.0.0.1", server.port, "conn-run", lambda m: None
            )
            sender.start()
            self.assertTrue(_wait_until(lambda: sender._sock is not None, 5.0))
            sender.enqueue_status("stopping", processing_queue_depth=0)
            self.assertTrue(_wait_until(lambda: len(server.statuses) == 1, 5.0))
            self.assertEqual(server.statuses[0]["run_id"], "conn-run")
            self.assertTrue(sender.stop(2000, drain=False))
        finally:
            server.stop()

    def test_in_band_hello_switch_reuses_same_connection(self) -> None:
        """R-A-15/R-A-17：身份变化 → 同连接带内重播 hello，不新建连接。"""
        server = _MiniGround()
        try:
            sender = air.RemoteSender("127.0.0.1", server.port, "id-a", lambda m: None)
            sender.start()
            self.assertTrue(_wait_until(lambda: sender._sock is not None, 5.0))
            self.assertEqual(server.connects, 1)
            sender.enqueue_status("acquiring", run_id="id-a")
            self.assertTrue(_wait_until(lambda: server.hello_ids == ["id-a"], 5.0))
            sender.enqueue_status("acquiring", run_id="id-b")
            self.assertTrue(
                _wait_until(lambda: server.hello_ids == ["id-a", "id-b"], 5.0),
                f"未在带内重播 hello: {server.hello_ids}",
            )
            self.assertEqual(server.connects, 1, "身份切换不得新建连接（地面单客户槽）")
            self.assertEqual(sender.run_id, "id-b", "连接身份必须跟进")
            self.assertEqual(
                [s["state"] for s in server.statuses], ["acquiring", "acquiring"]
            )
            self.assertTrue(sender.stop(2000, drain=False))
        finally:
            server.stop()

    def test_identity_switch_survives_concurrent_command(self) -> None:
        """§3.7-4 雷区：等 hello ack 期间到达的 command 必须被分派而不是踢断。"""
        server = _MiniGround(command_before_ack=True)
        actions: list[str] = []
        try:
            sender = air.RemoteSender(
                "127.0.0.1", server.port, "run-a", lambda m: None,
                command_callback=lambda action, params: actions.append(action),
            )
            sender.start()
            self.assertTrue(_wait_until(lambda: sender._sock is not None, 5.0))
            sender.enqueue_status("acquiring", run_id="run-b")
            self.assertTrue(
                _wait_until(lambda: sender.run_id == "run-b", 10.0),
                "command 帧不得打断身份切换",
            )
            self.assertIn("stop", actions, "切换等待期间的 command 必须被分派")
            self.assertEqual(server.connects, 1)
            self.assertTrue(sender.stop(2000, drain=False))
        finally:
            server.stop()


# ---------------------------------------------------------------------
# R-A-16：engB 死锁复现固化（真实 ReceiverThread，同一条客户连接）
# ---------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt unavailable: {IMPORT_ERROR}")
class RunIdCollisionRegressionTests(unittest.TestCase):
    """docs/round2_repro_runid_collision.py 场景 A 的 unittest 固化（零 mock）。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def _send_trace(self, sock: socket.socket, run_id: str, index: int, marker: str):
        payload = f"NPZ-PAYLOAD::{marker}".encode("utf-8")
        header = {
            "type": "trace",
            "protocol": ground.PROTOCOL,
            "run_id": run_id,
            "trace_index": index,
            "timestamp": "t",
            "payload_format": "npz",
            "payload_bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
        }
        raw = json.dumps(header, separators=(",", ":")).encode("utf-8")
        sock.sendall(struct.pack("!I", len(raw)) + raw + struct.pack("!Q", len(payload)) + payload)
        sock.settimeout(3.0)
        try:
            return _recv_header(sock)
        except Exception:  # noqa: BLE001 - 超时=地面未 ACK
            return None

    def _start_receiver(self, out_dir: Path) -> ground.ReceiverThread:
        receiver = ground.ReceiverThread("127.0.0.1", 0, out_dir)
        receiver.start()
        self.assertTrue(_wait_until(lambda: receiver.bound_port, 5.0))
        return receiver

    def test_two_task_identities_over_one_ground_connection(self) -> None:
        """任务 2 首条 trace（同编号不同 payload、新身份）必须被保存并 ACK。"""
        out_dir = Path(tempfile.mkdtemp(prefix="round2-ground-"))
        receiver = self._start_receiver(out_dir)
        status_events: list[dict] = []
        receiver.acquisition_status.connect(status_events.append, DIRECT)
        conflicts: list[str] = []
        receiver.status.connect(lambda text: conflicts.append(text) if "冲突拒绝" in text else None, DIRECT)
        sock = socket.create_connection(("127.0.0.1", int(receiver.bound_port)), timeout=5)
        try:
            sock.sendall(_frame({"type": "hello", "protocol": ground.PROTOCOL,
                                 "run_id": "run-1", "timestamp": "t"}))
            self.assertEqual(_recv_header(sock).get("type"), "ack")
            ack1 = self._send_trace(sock, "run-1", 1, "task-1")
            self.assertEqual(ack1.get("type"), "ack_trace", f"任务1 未被 ACK: {ack1}")
            ack2 = self._send_trace(sock, "run-2", 1, "task-2")
            self.assertIsNotNone(ack2, "任务2 首条 trace 无 ACK = §3.7 死锁回归")
            self.assertEqual(
                ack2.get("type"), "ack_trace",
                f"任务2 必须被保存并 ACK（冲突拒绝即死锁链）: {ack2}",
            )
            self.assertNotIn(True, [ack2.get("duplicate")], "新身份首条不得是 duplicate")
            self.assertEqual(conflicts, [], f"不得出现冲突拒绝: {conflicts}")
            self.assertEqual(len(list((out_dir / "run-1").glob("trace_*.npz"))), 1)
            self.assertEqual(len(list((out_dir / "run-2").glob("trace_*.npz"))), 1)
            self.assertFalse(list(out_dir.rglob("trace_00000.npz")), "不得产生脏文件")
        finally:
            sock.close()
            receiver.stop()
            self.assertTrue(receiver.wait(3000))

    def test_same_identity_different_payload_still_rejected(self) -> None:
        """[冻结] 反面对照：同 run 同编号不同 payload 仍拒绝且不 ACK（未削弱去重）。"""
        out_dir = Path(tempfile.mkdtemp(prefix="round2-ground2-"))
        receiver = self._start_receiver(out_dir)
        sock = socket.create_connection(("127.0.0.1", int(receiver.bound_port)), timeout=5)
        try:
            sock.sendall(_frame({"type": "hello", "protocol": ground.PROTOCOL,
                                 "run_id": "same-run", "timestamp": "t"}))
            self.assertEqual(_recv_header(sock).get("type"), "ack")
            first = self._send_trace(sock, "same-run", 1, "a")
            self.assertEqual(first.get("type"), "ack_trace")
            conflict = self._send_trace(sock, "same-run", 1, "b")
            self.assertIsNone(conflict, f"同身份不同 payload 不得收到 ACK: {conflict}")
        finally:
            sock.close()
            receiver.stop()
            self.assertTrue(receiver.wait(3000))

    def test_sender_drain_across_two_task_identities(self) -> None:
        """R-A-16/R-A-18：两任务消息在同一 sender 上排空、outstanding 归零。"""
        out_dir = Path(tempfile.mkdtemp(prefix="round2-drain-"))
        receiver = self._start_receiver(out_dir)
        sender = air.RemoteSender(
            "127.0.0.1", int(receiver.bound_port), "idle-conn", lambda m: None
        )
        sender.start()
        cfg = _sweep_config()
        freq = np.linspace(400e6, 4000e6, 5)
        rng = np.random.default_rng(7)
        raw = {"S11": rng.standard_normal(5) + 1j * rng.standard_normal(5),
               "S22": rng.standard_normal(5) + 1j * rng.standard_normal(5)}
        time_ns = np.linspace(0, 8, 16)
        traces = {"S11": rng.standard_normal(16), "S22": rng.standard_normal(16)}
        try:
            for run_id in ("20260101_000000_0001", "20260101_000000_0002"):
                for index in (1, 2):
                    sender.enqueue_trace(
                        index, cfg, freq, raw, dict(raw), time_ns, traces, air.GgaFix(),
                        channels=("S11", "S22"), run_id=run_id,
                    )
            self.assertTrue(sender.stop(20000, drain=True), "两任务消息必须可排空")
            self.assertEqual(sender.outstanding_count(), 0)
            for run_id in ("20260101_000000_0001", "20260101_000000_0002"):
                self.assertEqual(len(list((out_dir / run_id).glob("trace_*.npz"))), 2)
        finally:
            receiver.stop()
            self.assertTrue(receiver.wait(3000))


# ---------------------------------------------------------------------
# R-A-13 / R-A-07：手动 USB 互斥并集 + 关闭释放
# ---------------------------------------------------------------------


@unittest.skipIf(air is None, f"uav_gpr_qt unavailable: {IMPORT_ERROR}")
class ManualUsbMutexAndCloseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="round2-mutex-"))
        self.window = air.MainWindow()
        self.window.remote_enable.setChecked(False)
        self.window.continuous_enabled.setChecked(True)

    def tearDown(self) -> None:
        worker = self.window.worker
        if worker is not None and getattr(worker, "isRunning", lambda: False)():
            worker.stop()
            worker.wait(5000)
        if not self.window.acquisition_service.shutdown(5000):
            self.window.acquisition_service.retry_release()
        self.window.close()
        self.window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        self.app.processEvents()

    def _hold_idle_session(self) -> RecordingTransport:
        transport = RecordingTransport(5)
        backend = _configured_backend(transport)
        backend.pause()
        service = self.window.acquisition_service
        service._backend = backend  # noqa: SLF001 - 模拟真实任务停止后的待命
        service._state = "idle_waiting"  # noqa: SLF001
        return transport

    def test_manual_usb_rejected_while_service_idle_waiting(self) -> None:
        """R-A-13：任务停止进入待命后，手动 USB 入口仍必须被拒绝。"""
        transport = self._hold_idle_session()
        with unittest.mock.patch.object(air, "acquire_sparameters") as mocked:
            self.window.test_sweep_settings()
            self.window.capture_background()
            self.window.capture_calibration_standard("P1_Open")
            self.assertEqual(mocked.call_count, 0, "待命期不得自开第二条 USB 会话")
        self.assertTrue(transport.is_open)
        self.assertIn("禁止手动访问 LibreVNA", self.window.device_log.toPlainText())

    def test_manual_usb_recovers_after_shutdown(self) -> None:
        """R-A-13：只有 shutdown() 释放后手动入口才恢复（常驻定义性语义）。"""
        self._hold_idle_session()
        self.assertTrue(self.window.acquisition_service.shutdown(5000))
        with unittest.mock.patch.object(air, "acquire_sparameters", return_value=[]) as mocked:
            self.window.test_sweep_settings()
            self.assertEqual(mocked.call_count, 1, "释放后手动入口应恢复")

    def test_close_event_releases_usb_and_leaves_no_threads(self) -> None:
        """R-A-07：closeEvent 经服务真正释放 USB，无残留线程。"""
        transport = self._hold_idle_session()
        before = {t.ident for t in threading.enumerate()}
        event = QtGui.QCloseEvent()
        self.window.closeEvent(event)
        self.assertTrue(event.isAccepted(), "服务干净收尾后必须接受关闭")
        self.assertFalse(transport.is_open, "只有程序关闭才真正释放 USB")
        self.assertEqual(self.window.acquisition_service.state, "closed")
        alive = {t.ident for t in threading.enumerate() if t.is_alive()} - before
        self.assertEqual(alive, set(), f"残留线程: {alive}")

    def test_close_event_ignored_when_release_blocked(self) -> None:
        """R-A-07：资源未释放 → closeEvent ignore，第二次可成功。"""
        transport = _CloseFailsTransport(5)
        backend = _configured_backend(transport)
        service = self.window.acquisition_service
        service._backend = backend  # noqa: SLF001
        service._state = "idle_waiting"  # noqa: SLF001
        event = QtGui.QCloseEvent()
        self.window.closeEvent(event)
        self.assertFalse(event.isAccepted(), "未释放时不得伪装关闭成功")
        transport.closable = True
        event2 = QtGui.QCloseEvent()
        self.window.closeEvent(event2)
        self.assertTrue(event2.isAccepted())
        self.assertFalse(transport.is_open)


# ---------------------------------------------------------------------
# 极简地面替身（hello/ack + 可注入"ack 前先发 command"）
# ---------------------------------------------------------------------


class _MiniGround:
    def __init__(self, *, command_before_ack: bool = False) -> None:
        self.command_before_ack = command_before_ack
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(4)
        self.port = self.server.getsockname()[1]
        self.connects = 0
        self.hello_ids: list[str] = []
        self.statuses: list[dict] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        self.server.settimeout(0.2)
        injected = False
        while not self._stop.is_set():
            try:
                conn, _ = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.connects += 1
            conn.settimeout(0.2)
            try:
                while not self._stop.is_set():
                    try:
                        header = _recv_header(conn)
                    except (socket.timeout, ConnectionError, OSError):
                        break
                    if header is None:
                        break
                    kind = header.get("type")
                    if kind == "hello":
                        if self.command_before_ack and not injected and len(self.hello_ids) == 1:
                            injected = True
                            conn.sendall(_frame({"type": "command", "protocol": ground.PROTOCOL,
                                                 "action": "stop", "params": {}}))
                        self.hello_ids.append(str(header.get("run_id")))
                        conn.sendall(_frame({"type": "ack", "protocol": ground.PROTOCOL}))
                    elif kind == "acquisition_status":
                        self.statuses.append(dict(header))
                    elif kind == "trace":
                        conn.sendall(_frame({"type": "ack_trace", "protocol": ground.PROTOCOL,
                                             "trace_index": header.get("trace_index")}))
            finally:
                try:
                    conn.close()
                except OSError:
                    pass

    def stop(self) -> None:
        self._stop.set()
        try:
            self.server.close()
        except OSError:
            pass
        self._thread.join(2000)


def _frame(header: dict) -> bytes:
    raw = json.dumps(header, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    return struct.pack("!I", len(raw)) + raw + struct.pack("!Q", 0)


def _recv_header(sock: socket.socket) -> dict | None:
    head = _recv_exact(sock, 4)
    if head is None:
        return None
    (length,) = struct.unpack("!I", head)
    raw = _recv_exact(sock, length)
    if raw is None:
        return None
    header = json.loads(raw.decode("utf-8"))
    (payload_len,) = struct.unpack("!Q", _recv_exact(sock, 8) or b"")
    if payload_len:
        _recv_exact(sock, payload_len)
    return header


def _recv_exact(sock: socket.socket, size: int) -> bytes | None:
    chunks: list[bytes] = []
    remaining = size
    while remaining > 0:
        try:
            chunk = sock.recv(remaining)
        except socket.timeout:
            return None
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


if __name__ == "__main__":
    unittest.main()

