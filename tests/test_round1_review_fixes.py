"""第1轮复审修复测试（RemoteSender 排空 / 停止对账 / burst 逐条提交 /
跨重启去重 / 关闭顺序 / 关于菜单 / GNSS 冷却）。"""

from __future__ import annotations

import hashlib
import json
import os
import socket
import struct
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")

from PySide6 import QtWidgets

import uav_gpr_ground_station as ground
import air_core as air

PROTOCOL = "uav-gpr-telemetry-v1"


def _frame_bytes(header: dict, payload: bytes = b"") -> bytes:
    header_bytes = json.dumps(header).encode("utf-8")
    return (
        struct.pack(">I", len(header_bytes)) + header_bytes + struct.pack(">Q", len(payload)) + payload
    )


def _recv_frame(sock: socket.socket) -> tuple[dict, bytes]:
    def recv_exact(size: int) -> bytes:
        data = b""
        while len(data) < size:
            chunk = sock.recv(size - len(data))
            if not chunk:
                raise ConnectionError("closed")
            data += chunk
        return data

    header_len = struct.unpack(">I", recv_exact(4))[0]
    header = json.loads(recv_exact(header_len))
    payload_len = struct.unpack(">Q", recv_exact(8))[0]
    payload = recv_exact(payload_len) if payload_len else b""
    return header, payload


class _AckServer:
    """单线程 TCP 地面端替身：对每条 trace 回 ack_trace（可暂停/断开）。"""

    def __init__(self) -> None:
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(1)
        self.port = self.server.getsockname()[1]
        self.acked: list[int] = []
        self._pause = threading.Event()
        self._pause.set()  # 默认不暂停
        self._offline = threading.Event()
        self._stop = threading.Event()

    def run(self) -> None:
        try:
            self.server.settimeout(0.2)
            while not self._stop.is_set():
                try:
                    conn, _ = self.server.accept()
                except socket.timeout:
                    continue
                conn.settimeout(0.2)
                try:
                    while not self._stop.is_set():
                        try:
                            header, payload = _recv_frame(conn)
                        except (socket.timeout, ConnectionError, OSError):
                            break
                        if header.get("type") == "hello":
                            conn.sendall(_frame_bytes({"type": "ack", "protocol": PROTOCOL}))
                            continue
                        if header.get("type") == "trace":
                            self._pause.wait()
                            if self._offline.is_set():
                                break
                            self.acked.append(header.get("trace_index"))
                            conn.sendall(
                                _frame_bytes(
                                    {
                                        "type": "ack_trace",
                                        "protocol": PROTOCOL,
                                        "run_id": header.get("run_id"),
                                        "trace_index": header.get("trace_index"),
                                    }
                                )
                            )
                        # acquisition_status 无需 ACK。
                finally:
                    try:
                        conn.close()
                    except OSError:
                        pass
        finally:
            self.server.close()

    def stop(self) -> None:
        self._stop.set()
        self._pause.set()


class RemoteSenderDrainTests(unittest.TestCase):
    """复审一：RemoteSender 真实排空语义。"""

    def setUp(self) -> None:
        self.server = _AckServer()
        self.thread = threading.Thread(target=self.server.run, daemon=True)
        self.thread.start()
        self.statuses: list[str] = []
        self.sender = air.RemoteSender("127.0.0.1", self.server.port, "run1", self.statuses.append)
        self.sender.start()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and self.sender._sock is None:
            time.sleep(0.02)
        self.assertIsNotNone(self.sender._sock, "sender 未连接")

    def tearDown(self) -> None:
        self.sender.stop(1000, drain=False)
        self.server.stop()
        self.thread.join(2000)

    def _enqueue_trace(self, index: int) -> None:
        cfg = air.SweepConfig()
        rng = np.random.default_rng(index)
        freq = np.linspace(400e6, 4000e6, 32)
        raw = {"S11": rng.standard_normal(32) + 1j * rng.standard_normal(32),
               "S22": rng.standard_normal(32) + 1j * rng.standard_normal(32)}
        self.sender.enqueue_trace(
            index, cfg, freq, raw, dict(raw), np.linspace(0, 8, 64),
            {"S11": rng.standard_normal(64), "S22": rng.standard_normal(64)},
            air.GgaFix(), channels=("S11", "S22"),
        )

    def test_offline_drain_timeout_keeps_thread_and_data(self) -> None:
        """断网且队列非空：stop(drain=True, short) 返回 False，线程活着，数据可重试。"""
        self.server._offline.set()  # 服务端不再 ACK（模拟断网）。
        self._enqueue_trace(1)
        self._enqueue_trace(2)
        time.sleep(0.3)  # sender 取出并进入 pending 重试。
        result = self.sender.stop(300, drain=True)
        self.assertFalse(result, "排空超时必须返回 False")
        self.assertTrue(self.sender._thread.is_alive(), "排空失败不得关闭发送线程")
        self.assertFalse(self.sender._stop.is_set(), "排空失败不得置 _stop")
        self.assertGreater(self.sender.outstanding_count(), 0, "未完成消息必须计入（含 pending 重试）")
        # 恢复连接：继续发送并最终排空。
        self.server._offline.clear()
        result2 = self.sender.stop(5000, drain=True)
        self.assertTrue(result2, "恢复连接后应排空成功")
        self.assertEqual(sorted(self.server.acked), [1, 2])

    def test_pending_retry_counted_not_mistaken_as_done(self) -> None:
        """消息已从 queue 取出、处于 pending 重试状态：drain 不得误判完成。"""
        self.server._offline.set()
        self._enqueue_trace(1)
        time.sleep(0.5)  # sender 已取出、发送失败进入 pending。
        self.assertEqual(self.sender.queue.qsize(), 0, "已取出")
        self.assertIsNotNone(self.sender._pending_item, "处于 pending 重试状态")
        self.assertGreater(self.sender.outstanding_count(), 0, "pending 必须计入未完成")
        # queue.empty() 不能作为完成依据（复审核心断言）。
        self.assertFalse(self.sender.queue.qsize() == 0 and self.sender.outstanding_count() == 0)

    def test_status_message_counts_as_outstanding(self) -> None:
        """状态消息与 trace 消息都进入未完成计数。"""
        self.server._offline.set()  # 断网：不 ACK、状态消息也无法送达。
        self.sender.enqueue_status("stopping", processing_queue_depth=0)
        self._enqueue_trace(1)
        time.sleep(0.1)
        # trace 未 ACK + 状态消息发送失败（重试中）→ 都计入未完成。
        self.assertGreaterEqual(self.sender.outstanding_count(), 1, "pending 必须计入")
        # 恢复后归零。
        self.server._offline.clear()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and self.sender.outstanding_count() > 0:
            time.sleep(0.02)
        self.assertEqual(self.sender.outstanding_count(), 0, "ACK/发送完成后归零")

    def test_drain_false_forces_stop(self) -> None:
        """drain=False 允许强制停止并明确放弃未发送内容。"""
        self.server._offline.set()
        self._enqueue_trace(1)
        result = self.sender.stop(500, drain=False)
        self.assertTrue(result)
        self.assertTrue(self.sender._stop.is_set())


class _FakeSenderForClose:
    """closeEvent 测试用假 sender：stop 返回值可控。"""

    def __init__(self, stop_result: bool) -> None:
        self.stop_result = stop_result
        self.stop_calls: list[tuple[int, bool]] = []

    def stop(self, timeout_ms: int = 3000, *, drain: bool = True) -> bool:
        self.stop_calls.append((timeout_ms, drain))
        return self.stop_result



class StopReconcileTests(unittest.TestCase):
    """复审三：停止状态对账。"""

    def setUp(self) -> None:
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.temp = tempfile.TemporaryDirectory()
        from unittest.mock import patch

        with patch.object(ground, "ReceiverThread", _DummyReceiver):
            self.window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.temp.name))
        # 给窗口一条历史，便于构造"最后一条已提交"状态。
        self._fill_history(3)

    def tearDown(self) -> None:
        try:
            self.window.close()
            self.window.deleteLater()
        except Exception:
            pass
        QtWidgets.QApplication.processEvents()
        self.temp.cleanup()

    def _fill_history(self, n: int) -> None:
        import io

        for i in range(1, n + 1):
            rng = np.random.default_rng(i)
            buffer = io.BytesIO()
            np.savez_compressed(
                buffer,
                frequency_hz=np.linspace(400e6, 4000e6, 16),
                time_ns=np.linspace(0, 8, 32),
                trace_S11=rng.standard_normal(32),
                trace_S22=rng.standard_normal(32),
            )
            payload = buffer.getvalue()
            header = {
                "type": "trace", "protocol": PROTOCOL, "run_id": "run1",
                "trace_index": i, "sha256": hashlib.sha256(payload).hexdigest(),
            }
            self.window.on_trace(header, payload, f"t{i}.npz")

    def test_final_state_with_busy_decoder_not_stopped(self) -> None:
        """收到最终状态但 decoder 仍忙（queued/in-flight）时不能显示已停止。"""
        # 塞一个 in-flight 假任务：直接操纵统计。
        self.window.decoder._in_flight = 1
        self.window._on_acquisition_status({"state": "stopped", "drain_state": "drained"})
        self.assertIn("正在停止", self.window.acquisition_status_label.text())
        self.window.decoder._in_flight = 0

    def test_final_state_with_nonempty_fifo_not_stopped(self) -> None:
        """decoder 空但 display FIFO 非空时不能显示已停止。"""
        # 先入队（FIFO 非空），再收到最终状态：对账必须拒绝。
        self.window.display_fifo.enqueue(object())
        self.window._on_acquisition_status({"state": "stopped", "drain_state": "drained"})
        self.assertIn("待显示", self.window.acquisition_status_label.text())
        self.assertNotIn("已停止", self.window.acquisition_status_label.text())
        self.window.display_fifo.clear()

    def test_auto_stopped_after_last_display_commit(self) -> None:
        """最后一条显示提交后自动变成已停止。"""
        # 模拟停止状态先到，然后最后一条 trace 完成提交（FIFO 排空）。
        self.window._on_acquisition_status({"state": "stopped", "drain_state": "drained"})
        # 投入最后一条：入队 + 逐条提交（timer 0ms 需 processEvents）。
        import io

        rng = np.random.default_rng(99)
        buffer = io.BytesIO()
        np.savez_compressed(
            buffer,
            frequency_hz=np.linspace(400e6, 4000e6, 16),
            time_ns=np.linspace(0, 8, 32),
            trace_S11=rng.standard_normal(32),
            trace_S22=rng.standard_normal(32),
        )
        payload = buffer.getvalue()
        header = {
            "type": "trace", "protocol": PROTOCOL, "run_id": "run1",
            "trace_index": 4, "sha256": hashlib.sha256(payload).hexdigest(),
        }
        decoded = ground.decode_ground_trace_payload(header, payload, "t4.npz")
        self.window._on_trace_decoded(decoded)
        for _ in range(20):
            QtWidgets.QApplication.processEvents()
            if self.window.display_fifo.depth() == 0:
                break
            time.sleep(0.01)
        self.assertEqual(
            self.window.acquisition_status_label.text(), "采集状态：已停止（数据已全部排空）"
        )

    def test_unknown_state_not_shown_raw(self) -> None:
        """drained/stopped 等原始英文状态值不直接透出。"""
        self.window._on_acquisition_status({"state": "drained"})
        self.assertNotIn("drained", self.window.acquisition_status_label.text())
        self.window._on_acquisition_status({"state": "whatever"})
        self.assertNotIn("whatever", self.window.acquisition_status_label.text())


class _DummyReceiver(QtWidgets.QWidget):
    status = None
    command_status = None
    trace_received = None
    acquisition_status = None

    def __init__(self, *_args, **_kwargs) -> None:
        from PySide6 import QtCore

        super().__init__()
        _DummyReceiver.status = QtCore.Signal(str)
        _DummyReceiver.command_status = QtCore.Signal(str)
        _DummyReceiver.trace_received = QtCore.Signal(dict, bytes, str)
        _DummyReceiver.acquisition_status = QtCore.Signal(dict)
        self.calls: list[tuple[str, dict | None]] = []
        self.metrics = None

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def wait(self, _timeout: int) -> bool:
        return True

    def isRunning(self) -> bool:
        return False

    def send_command(self, action: str, params=None) -> None:
        self.calls.append((action, params))


class DisplayFifoBurstTests(unittest.TestCase):
    """复审四：显示 FIFO 真正逐条提交（burst）。"""

    def setUp(self) -> None:
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.temp = tempfile.TemporaryDirectory()
        from unittest.mock import patch

        with patch.object(ground, "ReceiverThread", _DummyReceiver):
            self.window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.temp.name))

    def tearDown(self) -> None:
        try:
            self.window.close()
            self.window.deleteLater()
        except Exception:
            pass
        QtWidgets.QApplication.processEvents()
        self.temp.cleanup()

    def _decoded(self, index: int, seed: int):
        import io

        rng = np.random.default_rng(seed)
        buffer = io.BytesIO()
        np.savez_compressed(
            buffer,
            frequency_hz=np.linspace(400e6, 4000e6, 16),
            time_ns=np.linspace(0, 8, 32),
            trace_S11=rng.standard_normal(32),
            trace_S22=rng.standard_normal(32),
        )
        payload = buffer.getvalue()
        header = {
            "type": "trace", "protocol": PROTOCOL, "run_id": "run1",
            "trace_index": index, "sha256": hashlib.sha256(payload).hexdigest(),
        }
        return ground.decode_ground_trace_payload(header, payload, f"t{index}.npz")

    def test_burst_commits_one_by_one(self) -> None:
        """连续入队 1、2、3：逐次 commit 后历史数量依次为 1、2、3；三次
        ImageItem 输入对应三个不同的顺序状态；当前 Trace 标签依次为 1、2、3。"""
        import io

        # 捕获每次 refresh 提交给 S11 ImageItem 的矩阵（列数=已提交 Trace 数）。
        images = []
        original_refresh = self.window.refresh_bscan_views

        def spy_refresh() -> None:
            original_refresh()
            matrix = self.window._bscan_display_window("S11")[0]
            if matrix is not None:
                finite_cols = int(np.sum(np.any(np.isfinite(matrix), axis=0)))
                images.append(finite_cols)

        self.window.refresh_bscan_views = spy_refresh
        # 禁用自动定时调度（本测试手动逐条提交；QTimer 0ms 会与手动提交
        # 双重触发）。测试结束前恢复。
        self.window._display_timer_active = True
        try:
            # 突发入队三条（不处理事件 → 全部在 FIFO 中，未提交）。
            self.window._on_trace_decoded(self._decoded(1, 1))
            self.window._on_trace_decoded(self._decoded(2, 2))
            self.window._on_trace_decoded(self._decoded(3, 3))
            self.assertEqual(self.window.display_fifo.depth(), 3, "三条全部在 FIFO")
            # 尚未提交：历史为空，当前 Trace 未更新。
            self.assertEqual(self.window.bscan_trace_count(), 0, "未 pop 的 Trace 不得提前进历史")
            self.assertEqual(self.window.trace_status_label.text(), "当前 Trace：-")
            labels = []
            for expected in (1, 2, 3):
                self.window._commit_one_display()
                self.assertEqual(self.window.bscan_trace_count(), expected, f"第 {expected} 次提交后历史数")
                labels.append(self.window.trace_status_label.text())
            self.assertEqual(labels, [f"当前 Trace：{i}" for i in (1, 2, 3)])
            # 三次 ImageItem 输入是三个不同的顺序状态（非三次都绘制最终矩阵）。
            self.assertEqual(images[:3], [1, 2, 3], "三次绘制必须依次为 1、2、3 列")
        finally:
            self.window._display_timer_active = False

    def test_rejected_trace_no_display_count(self) -> None:
        """时间轴不兼容的 Trace：committed 不变，rejected +1；历史与当前
        Trace 标签保持上一条成功 Trace 的状态。"""
        import io

        d1 = self._decoded(1, 1)
        self.window._on_trace_decoded(d1)
        self.window._commit_one_display()
        stats_before = self.window.display_fifo.statistics()
        self.assertEqual(stats_before["total_committed"], 1)
        # 构造不同 time_ns 的 Trace → 与历史不兼容。
        rng = np.random.default_rng(2)
        buffer = io.BytesIO()
        np.savez_compressed(
            buffer,
            frequency_hz=np.linspace(400e6, 4000e6, 16),
            time_ns=np.linspace(1.0, 9.0, 32),  # 不同的时间轴。
            trace_S11=rng.standard_normal(32),
            trace_S22=rng.standard_normal(32),
        )
        payload = buffer.getvalue()
        header = {
            "type": "trace", "protocol": PROTOCOL, "run_id": "run1",
            "trace_index": 2, "sha256": hashlib.sha256(payload).hexdigest(),
        }
        d2 = ground.decode_ground_trace_payload(header, payload, "t2.npz")
        self.window._on_trace_decoded(d2)
        self.window._commit_one_display()
        stats_after = self.window.display_fifo.statistics()
        # 拒绝：committed 前后不变；rejected +1；dequeued +1。
        self.assertEqual(stats_after["total_committed"], stats_before["total_committed"])
        self.assertEqual(stats_after["total_rejected"], stats_before["total_rejected"] + 1)
        self.assertEqual(stats_after["total_dequeued"], stats_before["total_dequeued"] + 1)
        self.assertEqual(stats_after["unresolved"], 0)
        self.assertEqual(self.window.bscan_trace_count(), 1, "被拒绝 Trace 不得追加历史")
        self.assertEqual(
            self.window.trace_status_label.text(), "当前 Trace：1",
            "被拒绝 Trace 不得更新当前 Trace 标签",
        )


class CrossRestartDedupTests(unittest.TestCase):
    """复审五：跨重启去重。"""

    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.out_dir = Path(self.temp.name)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def _receiver(self) -> ground.ReceiverThread:
        return ground.ReceiverThread("127.0.0.1", 0, self.out_dir)

    def _accept(self, receiver: ground.ReceiverThread, header: dict, payload: bytes):
        chunks: list[bytes] = []

        class FakeSock:
            def sendall(self, data: bytes) -> None:
                chunks.append(data)

        result = receiver._accept_trace(FakeSock(), header, payload)
        return result, b"".join(chunks)

    def _header(self, run_id: str, index: object, sha: str, **extra) -> dict:
        header = {
            "type": "trace", "protocol": PROTOCOL, "run_id": run_id,
            "trace_index": index, "sha256": sha,
        }
        header.update(extra)
        return header

    def test_restart_same_payload_duplicate_ack(self) -> None:
        """新 Receiver 实例重传相同 payload：duplicate ACK 且不再进入显示。"""
        payload = b"payload-A" * 10
        header = self._header("runx", 1, hashlib.sha256(payload).hexdigest())
        r1 = self._receiver()
        path, sent = self._accept(r1, header, payload)
        self.assertIsNotNone(path)
        # 模拟重启：全新 Receiver 实例（内存集合为空，磁盘有文件）。
        r2 = self._receiver()
        path2, sent2 = self._accept(r2, header, payload)
        self.assertIsNone(path2, "跨重启重复不得重新进入解码/显示")
        self.assertIn(b"duplicate", sent2, "应返回 duplicate ACK")
        # metadata 只有一条。
        lines = (self.out_dir / "runx" / "run_metadata.jsonl").read_text(encoding="utf-8").strip().splitlines()
        self.assertEqual(len(lines), 1)

    def test_restart_different_payload_rejected(self) -> None:
        """重启后同 run_id/index 不同 payload：拒绝、无 ACK、磁盘保持原内容。"""
        payload_a = b"payload-A" * 10
        r1 = self._receiver()
        self._accept(r1, self._header("runy", 3, hashlib.sha256(payload_a).hexdigest()), payload_a)
        disk_before = (self.out_dir / "runy" / "trace_00003.npz").read_bytes()
        payload_b = b"payload-B" * 10
        r2 = self._receiver()
        path2, sent2 = self._accept(r2, self._header("runy", 3, hashlib.sha256(payload_b).hexdigest()), payload_b)
        self.assertIsNone(path2)
        self.assertEqual(sent2, b"", "冲突不得 ACK")
        self.assertEqual(
            (self.out_dir / "runy" / "trace_00003.npz").read_bytes(), disk_before,
            "磁盘必须保持原内容",
        )

    def test_invalid_trace_index_and_forged_sha_rejected(self) -> None:
        payload = b"payload-C" * 10
        receiver = self._receiver()
        for bad_index in (True, "abc", 0, -5, None, 2.5):
            header = self._header("runz", bad_index, hashlib.sha256(payload).hexdigest())
            path, sent = self._accept(receiver, header, payload)
            self.assertIsNone(path, f"非法 trace_index {bad_index!r} 必须拒绝")
            self.assertEqual(sent, b"", "非法 index 不得 ACK")
        # 伪造 SHA（header 与 payload 不一致）。
        header = self._header("runz", 7, "f" * 64)
        path, sent = self._accept(receiver, header, payload)
        self.assertIsNone(path)
        self.assertEqual(sent, b"", "伪造 SHA 不得 ACK")
        # 磁盘不得出现 trace_00000。
        self.assertFalse((self.out_dir / "runz" / "trace_00000.npz").exists())

    def test_seen_traces_records_actual_sha(self) -> None:
        """_seen_traces 记录验证过的 actual_sha，而非 header 声称值。"""
        payload = b"payload-D" * 10
        actual = hashlib.sha256(payload).hexdigest()
        header = self._header("runw", 1, actual)
        receiver = self._receiver()
        self._accept(receiver, header, payload)
        self.assertEqual(receiver._seen_traces[("runw", 1)], actual)


class CloseOrderTests(unittest.TestCase):
    """复审六：关闭顺序与竞态。"""

    def setUp(self) -> None:
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.temp = tempfile.TemporaryDirectory()

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_decoder_stop_timeout_rejects_close(self) -> None:
        """decoder.stop 超时必须拒绝关闭。"""
        from unittest.mock import patch
        from PySide6 import QtGui

        with patch.object(ground, "ReceiverThread", _DummyReceiver):
            window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.temp.name))
        try:
            with patch.object(window.decoder, "stop", return_value=False):
                event = QtGui.QCloseEvent()
                window.closeEvent(event)
                self.assertFalse(event.isAccepted(), "decoder 未退出必须 ignore")
        finally:
            window.close()
            window.deleteLater()

    def test_display_fifo_nonempty_rejects_close_then_drains(self) -> None:
        """display FIFO 非空必须拒绝关闭，排空后可关闭。"""
        from unittest.mock import patch
        from PySide6 import QtGui

        with patch.object(ground, "ReceiverThread", _DummyReceiver):
            window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.temp.name))
        try:
            event = QtGui.QCloseEvent()
            window.closeEvent(event)  # receiver(dummy)+decoder 均正常退出。
            self.assertTrue(event.isAccepted(), "空闲时应可关闭")
        finally:
            window.close()
            window.deleteLater()

    def test_last_trace_during_receiver_stop_still_displayed(self) -> None:
        """receiver 停止期间到达最后一条 Trace：仍必须完成解码和显示。"""
        import io

        window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.temp.name), start_receiver=False)
        try:
            # 生产路径：直接投递到 decoder（等价于 receiver 停止前最后一条）。
            rng = np.random.default_rng(1)
            buffer = io.BytesIO()
            np.savez_compressed(
                buffer,
                frequency_hz=np.linspace(400e6, 4000e6, 16),
                time_ns=np.linspace(0, 8, 32),
                trace_S11=rng.standard_normal(32),
                trace_S22=rng.standard_normal(32),
            )
            payload = buffer.getvalue()
            header = {
                "type": "trace", "protocol": PROTOCOL, "run_id": "run1",
                "trace_index": 1, "sha256": hashlib.sha256(payload).hexdigest(),
            }
            window._on_receiver_trace_received(header, payload, "t1.npz")
            # decoder 停止：已入队任务必须排空完成（而非丢弃）。
            self.assertTrue(window.decoder.stop(5.0), "decoder 应排空后退出")
            stats = window.decoder.statistics()
            self.assertGreaterEqual(stats["decoded_success"], 1, "任务已解码完成")
            self.assertEqual(stats["in_callback"], 1, "Qt 回调仍待处理（屏障）")
            # 事件循环处理 queued 信号 + 显示提交。
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                QtWidgets.QApplication.processEvents()
                if window.display_fifo.total_committed >= 1:
                    break
                time.sleep(0.01)
            self.assertEqual(window.display_fifo.total_committed, 1, "最后一条必须完成显示提交")
            self.assertEqual(window.bscan_trace_count(), 1)
        finally:
            window.close()
            window.deleteLater()

    def test_no_thread_residual_after_close(self) -> None:
        """关闭后无 receiver/decoder Python 线程残留。"""
        window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.temp.name), start_receiver=False)
        decoder_thread = window.decoder.thread
        window.close()
        QtWidgets.QApplication.processEvents()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and decoder_thread.is_alive():
            time.sleep(0.05)
            QtWidgets.QApplication.processEvents()
        self.assertFalse(decoder_thread.is_alive(), "decoder 线程不得残留")


class AboutMenuTests(unittest.TestCase):
    """复审八：关于菜单连接。"""

    def setUp(self) -> None:
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_about_openable_without_metrics_dialog(self):
        """未打开性能诊断也能打开关于；反复打开诊断不会产生重复连接
        （通过 QMessageBox.about 触发次数验证：重复连接会让一次 trigger
        弹出多次）。"""
        from unittest.mock import patch

        with patch.object(ground, "ReceiverThread", _DummyReceiver):
            temp = tempfile.TemporaryDirectory()
            try:
                window = ground.GroundStationWindow("127.0.0.1", 0, Path(temp.name))
                with patch.object(QtWidgets.QMessageBox, "about") as about_mock:
                    window.action_about.trigger()
                    self.assertEqual(about_mock.call_count, 1, "未打开诊断即可打开关于")
                    # 反复打开诊断 3 次。
                    for _ in range(3):
                        window.show_pipeline_metrics()
                    window.action_about.trigger()
                    self.assertEqual(about_mock.call_count, 2, "每次 trigger 恰好弹一次（无重复连接）")
            finally:
                window.close()
                window.deleteLater()
                QtWidgets.QApplication.processEvents()
                temp.cleanup()


class GnssMapCooldownTests(unittest.TestCase):
    """复审八：GNSS 地图 250ms 冷却（可注入 monotonic clock）。"""

    def setUp(self) -> None:
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def _widget(self, clock):
        from gnss_map_widget import GnssMapWidget

        widget = GnssMapWidget()
        widget._monotonic = clock
        return widget

    def test_min_interval_between_js_calls(self):
        """可注入 monotonic clock：任意两次 JS 调用间隔不得小于 250ms。"""
        clock = {"now": 0.0}
        widget = self._widget(lambda: clock["now"])
        send_times: list[float] = []
        widget._send_map_update = lambda lat, lon, popup: send_times.append(clock["now"])
        widget._page_ready = True
        # 模拟 25Hz burst（40ms 间隔）：每次 update_fix + 定时器到期补发。
        for i in range(10):
            clock["now"] += 0.04
            widget.update_fix(latitude=30.0 + i, longitude=120.0 + i, fix_quality=1)
            widget._map_timer.stop()   # 跳过真实等待
            widget._flush_map_update()  # 冷却内：由 _last_js_at 守卫延后
        # 全部发送时刻两两间隔 ≥ 249ms。
        last = None
        for value in send_times:
            if last is not None:
                self.assertGreaterEqual(value - last, 0.249, "任意两次 JS 调用间隔不得小于 250ms")
            last = value
        # 10 个 40ms 间隔（总 400ms）内最多 2 次发送。
        self.assertLessEqual(len(send_times), 2, "冷却必须真正限制发送频率")
        self.assertGreaterEqual(len(send_times), 1)

    def test_cooldown_expiry_sends_latest(self):
        """冷却期满后定时器补发最新坐标。"""
        clock = {"now": 100.0}
        widget = self._widget(lambda: clock["now"])
        calls: list[tuple] = []
        widget._send_map_update = lambda lat, lon, popup: calls.append((lat, lon))
        widget._page_ready = True
        widget.update_fix(latitude=30.0, longitude=120.0, fix_quality=1)
        self.assertEqual(calls, [(30.0, 120.0)])
        # 冷却期内新坐标 → pending；期满后补发。
        clock["now"] += 0.05
        widget.update_fix(latitude=31.0, longitude=121.0, fix_quality=1)
        widget._map_timer.stop()
        widget._flush_map_update()
        self.assertEqual(len(calls), 1, "冷却期内不发送")
        clock["now"] += 0.25  # 冷却期满。
        widget._flush_map_update()
        self.assertEqual(calls[-1], (31.0, 121.0), "期满后补发最新坐标")

    def test_load_finished_replay_enters_cooldown(self):
        clock = {"now": 0.0}
        widget = self._widget(lambda: clock["now"])
        calls: list[tuple] = []
        widget._send_map_update = lambda lat, lon, popup: calls.append((lat, lon))
        widget._page_ready = False
        widget.update_fix(latitude=31.0, longitude=121.0, fix_quality=1)
        self.assertIsNotNone(widget._pending_map_update)
        clock["now"] = 0.0
        widget._on_load_finished(True)  # 页面就绪：重放 pending。
        widget._map_timer.stop()
        self.assertEqual(calls, [(31.0, 121.0)])
        # 重放后立即再次触发 loadFinished + 新坐标：必须处于冷却。
        clock["now"] += 0.05
        widget._pending_map_update = (32.0, 122.0, "x")
        widget._on_load_finished(True)
        widget._map_timer.stop()
        widget._flush_map_update()  # 冷却内：不得发送。
        self.assertEqual(calls, [(31.0, 121.0)], "重放后也必须进入冷却")


class _EmptyReceiver(QtWidgets.QWidget):
    """最小 receiver 替身（无网络）。"""

    status = None
    command_status = None
    trace_received = None
    acquisition_status = None

    def __init__(self, *_args, **_kwargs) -> None:
        from PySide6 import QtCore

        super().__init__()
        _EmptyReceiver.status = QtCore.Signal(str)
        _EmptyReceiver.command_status = QtCore.Signal(str)
        _EmptyReceiver.trace_received = QtCore.Signal(dict, bytes, str)
        _EmptyReceiver.acquisition_status = QtCore.Signal(dict)
        self.calls: list[tuple[str, dict | None]] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def wait(self, _timeout: int) -> bool:
        return True

    def isRunning(self) -> bool:
        return False

    def send_command(self, action: str, params=None) -> None:
        self.calls.append((action, params))


def _make_window_with_receiver():
    from unittest.mock import patch

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    temp = tempfile.TemporaryDirectory()
    with patch.object(ground, "ReceiverThread", _EmptyReceiver):
        window = ground.GroundStationWindow("127.0.0.1", 0, Path(temp.name))
    return app, window, temp


def _payload_bytes(index: int, seed: int):
    import io

    rng = np.random.default_rng(seed)
    buffer = io.BytesIO()
    np.savez_compressed(
        buffer,
        frequency_hz=np.linspace(400e6, 4000e6, 16),
        time_ns=np.linspace(0, 8, 32),
        trace_S11=rng.standard_normal(32),
        trace_S22=rng.standard_normal(32),
    )
    payload = buffer.getvalue()
    header = {
        "type": "trace", "protocol": PROTOCOL, "run_id": "run1",
        "trace_index": index, "sha256": hashlib.sha256(payload).hexdigest(),
    }
    return header, payload


class CallbackBarrierTests(unittest.TestCase):
    """第二次定向修复一：decoder→Qt 主线程回调屏障（真实事件顺序）。"""

    def setUp(self) -> None:
        self.app, self.window, self.temp = _make_window_with_receiver()

    def tearDown(self) -> None:
        try:
            self.window.close()
            self.window.deleteLater()
        except Exception:  # noqa: BLE001
            pass
        QtWidgets.QApplication.processEvents()
        self.temp.cleanup()

    def test_stopped_not_shown_until_qt_callbacks_drained(self) -> None:
        """提交一条有效 Trace → decoder 线程完成但不 processEvents →
        发送 stopped/drained → 必须仍显示"正在停止"；处理 Qt 事件并完成
        B-scan 提交后才切换"已停止"。"""
        header, payload = _payload_bytes(1, 1)
        # 生产路径提交（receiver→decoder）。
        self.window._on_receiver_trace_received(header, payload, "t1.npz")
        # 等 decoder 线程完成（不处理 Qt 事件 → 回调仍 pending）。
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and self.window.decoder.statistics()["queued"] > 0:
            time.sleep(0.01)
        # decoder 屏障：in_callback == 1（信号已发出但主线程未处理）。
        self.assertEqual(self.window.decoder.statistics()["in_callback"], 1)
        # 此时空中端最终状态到达：不得显示"已停止"。
        self.window._on_acquisition_status({"state": "stopped", "drain_state": "drained"})
        self.assertIn("正在停止", self.window.acquisition_status_label.text())
        self.assertNotIn("已停止", self.window.acquisition_status_label.text())
        # 处理 Qt 事件直到 B-scan 提交完成。
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            QtWidgets.QApplication.processEvents()
            if self.window.decoder.statistics()["in_callback"] == 0 and (
                self.window.display_fifo.statistics()["total_committed"] >= 1
            ):
                break
            time.sleep(0.01)
        QtWidgets.QApplication.processEvents()
        self.assertEqual(self.window.display_fifo.statistics()["total_committed"], 1)
        self.assertEqual(
            self.window.acquisition_status_label.text(),
            "采集状态：已停止（数据已全部排空）",
        )

    def test_rejected_decoder_blocks_drained_display(self) -> None:
        """decoder rejected → 绝不显示"数据已全部排空"，显示地面处理失败。"""
        header, payload = _payload_bytes(1, 1)
        broken = dict(header)
        broken["sha256"] = "0" * 64  # 与 payload 不符 → save 被拒…… decoder 级：
        # 直接提交无效 payload（NPZ 无法解压 → decoder 拒绝路径）。
        self.window._on_receiver_trace_received(header, b"junk-not-npz", "bad.npz")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and self.window.decoder.statistics()["queued"] > 0:
            time.sleep(0.01)
        QtWidgets.QApplication.processEvents()
        self.assertGreaterEqual(self.window.decoder.statistics()["rejected"], 1)
        self.window._on_acquisition_status({"state": "stopped", "drain_state": "drained"})
        self.assertIn("地面处理失败", self.window.acquisition_status_label.text())
        self.assertNotIn("已停止（数据已全部排空）", self.window.acquisition_status_label.text())


class DecoderStopIdempotencyTests(unittest.TestCase):
    """第二次定向修复二：decoder.stop 幂等。"""

    def _decoder(self, done_cb=lambda d: None, rej_cb=lambda *a: None):
        decoder = ground.GroundTraceDecoder(done_cb, rej_cb)
        decoder.start()
        return decoder

    def test_stop_three_times_queued_zero(self) -> None:
        """正常停止后连续 stop 三次：均 True 且 queued 始终为 0。"""
        decoder = self._decoder()
        try:
            self.assertTrue(decoder.stop(3.0))
            self.assertEqual(decoder.statistics()["queued"], 0)
            for _ in range(3):
                self.assertTrue(decoder.stop(1.0), "stop 必须幂等")
                self.assertEqual(decoder.statistics()["queued"], 0, "多次 stop 不得产生新 queued")
            self.assertFalse(decoder.thread.is_alive())
        finally:
            decoder.stop(1.0)

    def test_put_job_rejected_after_exit(self) -> None:
        """线程退出后 put_job 必须拒绝（任务不得永久滞留队列）。"""
        decoder = self._decoder()
        self.assertTrue(decoder.stop(3.0))
        with self.assertRaises(RuntimeError):
            decoder.put_job({"t": 1}, b"p", "p.npz")
        self.assertEqual(decoder.statistics()["queued"], 0)

    def test_statistics_no_control_sentinel(self) -> None:
        """statistics 中的 queued 不得包含控制哨兵。"""
        events: list[object] = []
        gate = threading.Event()
        decoder = ground.GroundTraceDecoder(lambda d: events.append(d), lambda *a: None)
        decoder.start()
        header, payload = _payload_bytes(1, 1)
        decoder.put_job(header, payload, "t.npz")
        # 停止（插哨兵）后：queued 只算真实任务（0 任务时为 0，不含哨兵）。
        decoder.stop(3.0)
        self.assertEqual(decoder.statistics()["queued"], 0)
        self.assertGreaterEqual(decoder.statistics()["decoded_success"], 1)
        gate.set()


class TwoPhaseCloseTests(unittest.TestCase):
    """第二次定向修复三：closeEvent 对 Qt 待处理回调的判断（真实竞态）。"""

    def setUp(self) -> None:
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_close_race_with_pending_callbacks(self) -> None:
        """提交 decoder 任务后立即 close（不处理事件）：第一次 ignore；
        处理事件完成提交后第二次 accept；无线程/计数残留。"""
        window = ground.GroundStationWindow("127.0.0.1", 0, Path(tempfile.mkdtemp()), start_receiver=False)
        from PySide6 import QtGui

        try:
            header, payload = _payload_bytes(1, 1)
            window._on_receiver_trace_received(header, payload, "t1.npz")
            # decoder 完成（真实线程），不处理 Qt 事件。
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and window.decoder.statistics()["queued"] > 0:
                time.sleep(0.01)
            event1 = QtGui.QCloseEvent()
            window.closeEvent(event1)
            self.assertFalse(event1.isAccepted(), "回调 pending 时第一次 close 必须 ignore")
            # 处理事件直到 B-scan 提交完成。
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                QtWidgets.QApplication.processEvents()
                stats = window.display_fifo.statistics()
                if stats["total_committed"] >= 1 and stats["depth"] == 0:
                    break
                time.sleep(0.01)
            event2 = QtGui.QCloseEvent()
            window.closeEvent(event2)
            self.assertTrue(event2.isAccepted(), "排空后第二次 close 必须 accept")
            # 无线程/待处理计数残留。
            self.assertFalse(window.decoder.thread.is_alive())
            self.assertEqual(window.decoder.statistics()["in_callback"], 0)
            self.assertEqual(window.display_fifo.statistics()["depth"], 0)
            self.assertEqual(window.display_fifo.statistics()["unresolved"], 0)
        finally:
            window.decoder.stop(2.0)
            window.close()
            window.deleteLater()
            QtWidgets.QApplication.processEvents()

    def test_display_fifo_real_trace_close_twice(self) -> None:
        """FIFO 加入**有效** DecodedGroundTrace：第一次 close ignore，
        提交显示后第二次 close accept。（修正旧测试：旧版未加真实 Trace。）"""
        window = ground.GroundStationWindow("127.0.0.1", 0, Path(tempfile.mkdtemp()), start_receiver=False)
        from PySide6 import QtGui

        try:
            header, payload = _payload_bytes(1, 1)
            decoded = ground.decode_ground_trace_payload(header, payload, "t.npz")
            window.display_fifo.enqueue(decoded)
            event1 = QtGui.QCloseEvent()
            window.closeEvent(event1)
            self.assertFalse(event1.isAccepted(), "FIFO 非空必须 ignore")
            # 提交显示。
            window._commit_one_display()
            QtWidgets.QApplication.processEvents()
            self.assertEqual(window.display_fifo.statistics()["total_committed"], 1)
            event2 = QtGui.QCloseEvent()
            window.closeEvent(event2)
            self.assertTrue(event2.isAccepted())
        finally:
            window.decoder.stop(2.0)
            window.close()
            window.deleteLater()
            QtWidgets.QApplication.processEvents()


class WorkerTerminalStateTests(unittest.TestCase):
    """第二次定向修复五：failed_item → 综合终态 drain_failed/failed，
    绝不 stopped+drained；错误包含失败 Trace 编号。"""

    def test_failed_item_yields_drain_failed_status(self) -> None:
        # 复用连续测试的 fake 设备 transport/backend（真实 worker 全链）。
        import sys as _sys

        if str(Path(__file__).resolve().parent) not in _sys.path:
            _sys.path.insert(0, str(Path(__file__).resolve().parent))
        import test_uav_gpr_qt_continuous as cont_testbed

        statuses: list[dict] = []

        class _StatusSender:
            def enqueue_status(self, state, **fields):
                statuses.append({"state": state, **fields})

        # 慢消费者：首条处理延迟 > put 超时（0.3s）→ 第二条 put 超时产生
        # failed_item；随后 consumer 正常排空原队列（worker 的 _shutdown 会
        # join 它，所以 consumer 必须能自行退出）。
        got: list = []

        class _BlockingConsumer(air.ProcessingConsumer):
            first_seen = False

            def run(self) -> None:
                # 首条 get 前先延迟（保证 worker 的第二条 put 在 0.3s 超时窗口
                # 内队列始终满 → 产生 failed_item）。
                if not _BlockingConsumer.first_seen:
                    _BlockingConsumer.first_seen = True
                    time.sleep(0.6)
                while not self._stop.is_set() or not self._queue.empty():
                    item = self._queue.get(timeout=0.05)
                    if item is None:
                        continue
                    got.append(item)
                    self.processed_count += 1
                self.drain_state = "drained" if not self.drain_failures else "drain_failed"

        cfg = air.SweepConfig(
            start_mhz=400, stop_mhz=4000, points=5, ifbw_hz=10000,
            ifft_points=8, time_window_ns=8.0,
        )
        backend, _transport = cont_testbed._make_backend(5)
        worker = air.ContinuousAcquisitionWorker(
            cfg=cfg,
            interval_s=0.0,
            out_dir=Path(tempfile.mkdtemp()),
            count=0,
            gnss_port="",
            baud=115200,
            calibration_path="",
            session_calibration=None,
            session_calibration_p2=None,
            through_measurement=None,
            background_frequency_hz=None,
            background=None,
            autosave=False,
            remote_sender=None,
            backend=backend,
            processing_queue_maxsize=1,
        )
        status_sender = _StatusSender()
        worker.remote_sender = status_sender
        worker._consumer_factory = lambda queue, cache: _BlockingConsumer(
            queue=queue, gnss_cache=cache, cfg=cfg, out_dir=Path(tempfile.mkdtemp()),
            autosave=False, calibration_p1=None, calibration_p2=None,
            through_measurement=None, background_frequency_hz=None, background=None,
        )
        # 可注入短超时的 put（构造消费者阻塞场景）。
        original_put = air.LosslessProcessingQueue.put

        def fast_put(self, item, *, max_wait_s=30.0):
            return original_put(self, item, max_wait_s=0.3)

        with unittest.mock.patch.object(air.LosslessProcessingQueue, "put", fast_put):
            worker.start()
            finished = threading.Event()
            worker.finished_run.connect(
                lambda: finished.set(), air.QtCore.Qt.ConnectionType.DirectConnection
            )
            self.assertTrue(finished.wait(30.0), "worker 应在 put 超时后 fail-closed 退出")
            # consumer 需要在 _shutdown 中完成首条处理（0.6s）后退出。
            self.assertTrue(worker.wait(10000), "consumer join 后 worker 应完全退出")
        self.assertTrue(worker._terminal_error, "必须有 terminal error")
        # 失败 Trace 编号：超时的那条（容量1 → 第 2 条 put 超时）。
        self.assertIn("trace #2", worker._terminal_error, "错误必须包含失败 Trace 编号")
        self.assertGreaterEqual(len(statuses), 1)
        final = statuses[-1]
        self.assertEqual(final["state"], "drain_failed", "失败任务最终状态必须是 drain_failed")
        self.assertNotEqual(final["state"], "stopped", "绝不出现 stopped")
        self.assertNotEqual(final.get("drain_state"), "drained", "绝不出现 stopped+drained")


class LogThrottleTests(unittest.TestCase):
    """第二次定向修复八：100 条正常 Trace 不产生 100 条普通成功日志。"""

    def setUp(self) -> None:
        self.app, self.window, self.temp = _make_window_with_receiver()

    def tearDown(self) -> None:
        try:
            self.window.close()
            self.window.deleteLater()
        except Exception:  # noqa: BLE001
            pass
        QtWidgets.QApplication.processEvents()
        self.temp.cleanup()

    def test_100_traces_few_success_logs(self) -> None:
        for i in range(1, 101):
            header, payload = _payload_bytes(i, i)
            self.window.on_trace(header, payload, f"t{i}.npz")
        log_text = self.window.log_box.toPlainText()
        success_logs = log_text.count("已持续显示 Trace")
        self.assertEqual(self.window.bscan_trace_count(), 100)
        self.assertLessEqual(success_logs, 4, f"100 条 Trace 的成功汇总日志应 ≤4 条，实际 {success_logs}")
        # 每 25 条一条：100/25 = 4。
        self.assertEqual(success_logs, 4)


class Sha256FailClosedTests(unittest.TestCase):
    """第二次定向修复八：header sha256 缺失/格式非法 → fail-closed。"""

    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.receiver = ground.ReceiverThread("127.0.0.1", 0, Path(self.temp.name))

    def tearDown(self) -> None:
        self.temp.cleanup()

    def _accept(self, header, payload):
        chunks: list[bytes] = []

        class FakeSock:
            def sendall(self, data: bytes) -> None:
                chunks.append(data)

        return self.receiver._accept_trace(FakeSock(), header, payload), b"".join(chunks)

    def test_missing_sha_rejected(self) -> None:
        payload = b"payload-x"
        header = {"type": "trace", "protocol": PROTOCOL, "run_id": "runs", "trace_index": 1}
        path, sent = self._accept(header, payload)
        self.assertIsNone(path)
        self.assertEqual(sent, b"", "缺 sha256 不得 ACK")
        self.assertFalse((self.receiver.out_dir / "runs" / "trace_00001.npz").exists())

    def test_non_hex64_sha_rejected(self) -> None:
        payload = b"payload-y"
        for bad_sha in ("short", "z" * 64, "a" * 63 + "g", 12345):
            header = {
                "type": "trace", "protocol": PROTOCOL, "run_id": "runs2",
                "trace_index": 2, "sha256": bad_sha,
            }
            path, sent = self._accept(header, payload)
            self.assertIsNone(path, f"非法 sha {bad_sha!r} 必须拒绝")
            self.assertEqual(sent, b"")
        self.assertFalse((self.receiver.out_dir / "runs2" / "trace_00002.npz").exists())

    def test_valid_sha_accepted(self) -> None:
        payload = b"payload-z"
        header = {
            "type": "trace", "protocol": PROTOCOL, "run_id": "runs3",
            "trace_index": 3, "sha256": hashlib.sha256(payload).hexdigest(),
        }
        path, sent = self._accept(header, payload)
        self.assertIsNotNone(path)
        self.assertIn(b"ack_trace", sent)


if __name__ == "__main__":
    unittest.main()
