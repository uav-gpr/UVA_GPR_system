"""第1轮检查点B/D：地面解析线程、显示 FIFO、接收去重、GNSS 限频测试。

解码线程测试不依赖 Qt 事件循环（GroundTraceDecoder 使用普通线程）；
去重测试直接驱动 ReceiverThread._accept_trace；GNSS 限频测试使用
offscreen QApplication。
"""

from __future__ import annotations

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
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")

from PySide6 import QtCore

import uav_gpr_ground_station as ground
from ground_trace_pipeline import (
    GroundDisplayFIFO,
    GroundTraceDecoder,
    decode_ground_trace_payload,
)

PROTOCOL = "uav-gpr-telemetry-v1"


def _frame(header: dict, payload: bytes = b"") -> bytes:
    import json

    header_bytes = json.dumps(header).encode("utf-8")
    return struct.pack(">I", len(header_bytes)) + header_bytes + struct.pack(">I", len(payload)) + payload


def _make_header(run_id: str, index: int, sha: str) -> dict:
    return {
        "type": "trace",
        "protocol": PROTOCOL,
        "run_id": run_id,
        "trace_index": index,
        "sha256": sha,
    }


def _sha(payload: bytes) -> str:
    import hashlib

    return hashlib.sha256(payload).hexdigest()


class GroundDisplayFifoTests(unittest.TestCase):
    def test_fifo_order_no_merge_no_overwrite(self) -> None:
        fifo = GroundDisplayFIFO()
        for i in range(100):
            fifo.enqueue(i)
        self.assertEqual(fifo.depth(), 100)
        self.assertEqual(fifo.peak, 100)
        for i in range(100):
            self.assertEqual(fifo.pop(), i)
        self.assertIsNone(fifo.pop())

    def test_depth_and_oldest_age(self) -> None:
        fifo = GroundDisplayFIFO()
        self.assertIsNone(fifo.oldest_age_ms())
        fifo.enqueue("a")
        time.sleep(0.05)
        fifo.enqueue("b")
        self.assertEqual(fifo.depth(), 2)
        self.assertGreaterEqual(fifo.oldest_age_ms(), 40.0)
        fifo.pop()
        self.assertLess(fifo.oldest_age_ms(), 40.0)

    def test_statistics_and_clear(self) -> None:
        fifo = GroundDisplayFIFO()
        fifo.enqueue(1)
        fifo.enqueue(2)
        stats = fifo.statistics()
        self.assertEqual(stats["depth"], 2)
        self.assertEqual(stats["total_enqueued"], 2)
        # pop 只代表 dequeued，不代表 committed。
        fifo.pop()
        self.assertEqual(fifo.statistics()["total_dequeued"], 1)
        self.assertEqual(fifo.statistics()["total_committed"], 0)
        self.assertEqual(fifo.statistics()["unresolved"], 1)
        fifo.mark_committed()
        self.assertEqual(fifo.statistics()["total_committed"], 1)
        self.assertEqual(fifo.statistics()["unresolved"], 0)
        fifo.clear()
        self.assertEqual(fifo.statistics()["depth"], 0)

    def test_mark_rejected_and_failed_not_committed(self) -> None:
        fifo = GroundDisplayFIFO()
        fifo.enqueue(1)
        fifo.enqueue(2)
        fifo.enqueue(3)
        fifo.pop()
        fifo.mark_committed()
        fifo.pop()
        fifo.mark_rejected()
        fifo.pop()
        fifo.mark_failed()
        stats = fifo.statistics()
        self.assertEqual(stats["total_dequeued"], 3)
        self.assertEqual(stats["total_committed"], 1)
        self.assertEqual(stats["total_rejected"], 1)
        self.assertEqual(stats["total_failed"], 1)
        self.assertEqual(stats["unresolved"], 0)


class GroundTraceDecoderTests(unittest.TestCase):
    def _payload(self, index: int = 1, time_ns=None, trace_s11=None, trace_s22=None) -> bytes:
        import io
        import json

        rng = np.random.default_rng(index)
        if time_ns is None:
            time_ns = np.linspace(0.0, 8.0, 32)
        buffer = io.BytesIO()
        np.savez_compressed(
            buffer,
            frequency_hz=np.linspace(400e6, 4000e6, 201),
            time_ns=time_ns,
            trace_S11=trace_s11 if trace_s11 is not None else rng.standard_normal(32),
            trace_S22=trace_s22 if trace_s22 is not None else rng.standard_normal(32),
            channels=json.dumps(["S11", "S22"]),
        )
        return buffer.getvalue()

    def test_new_dual_channel_payload(self) -> None:
        decoded = decode_ground_trace_payload(
            _make_header("r1", 1, "sha"), self._payload(), "p.npz"
        )
        self.assertEqual(decoded.trace_index, 1)
        self.assertEqual(decoded.channels, ("S11", "S22"))
        self.assertIsNotNone(decoded.trace_s11)
        self.assertIsNotNone(decoded.trace_s22)
        self.assertIsNone(decoded.legacy_trace)

    def test_invalid_payload_rejected(self) -> None:
        with self.assertRaises(ValueError):
            decode_ground_trace_payload(_make_header("r1", 1, "sha"), b"not-npz", "p.npz")
        # 通道声明冲突：声明双通道，实际只有 S11。
        import io
        import json

        buffer = io.BytesIO()
        np.savez_compressed(
            buffer,
            frequency_hz=np.linspace(400e6, 4000e6, 201),
            time_ns=np.linspace(0, 8, 32),
            trace_S11=np.zeros(32),
            channels=json.dumps(["S11", "S22"]),
        )
        with self.assertRaises(ValueError):
            decode_ground_trace_payload(
                _make_header("r1", 1, "sha"), buffer.getvalue(), "p.npz"
            )

    def test_decoding_not_on_main_thread(self) -> None:
        seen_threads: list[str] = []
        done = threading.Event()
        decoder = GroundTraceDecoder(
            lambda decoded: (seen_threads.append(threading.current_thread().name), done.set()),
            lambda *_: None,
        )
        decoder.start()
        try:
            decoder.put_job(_make_header("r1", 1, "sha"), self._payload(), "p.npz")
            self.assertTrue(done.wait(5.0))
            self.assertNotEqual(threading.current_thread().name, seen_threads[0])
        finally:
            self.assertTrue(decoder.stop(2.0))

    def test_rejection_callback(self) -> None:
        rejected: list[object] = []
        done = threading.Event()
        decoder = GroundTraceDecoder(lambda d: None, lambda h, p, e: (rejected.append(e), done.set()))
        decoder.start()
        try:
            decoder.put_job(_make_header("r1", 1, "sha"), b"junk", "p.npz")
            self.assertTrue(done.wait(5.0))
            self.assertIsInstance(rejected[0], Exception)
        finally:
            self.assertTrue(decoder.stop(2.0))

    def test_time_axis_conflict_detected_on_main_thread_append(self) -> None:
        """时间轴/Dewow 冲突属于与历史相关的校验，仍由主线程追加阶段执行。"""
        # 这里仅验证 decode 本身允许不同 time_ns 通过（与历史比较在窗口层）。
        decoded = decode_ground_trace_payload(
            _make_header("r1", 1, "sha"),
            self._payload(time_ns=np.linspace(1.0, 9.0, 32)),
            "p.npz",
        )
        np.testing.assert_allclose(decoded.time_ns, np.linspace(1.0, 9.0, 32))


class ReceiverDedupTests(unittest.TestCase):
    """接收端幂等去重（run_id + trace_index + sha256）。"""

    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.receiver = ground.ReceiverThread("127.0.0.1", 0, Path(self.temp.name))

    def tearDown(self) -> None:
        self.temp.cleanup()

    def _accept(self, header, payload):
        # _accept_trace 需要 socket 发送 ACK；用 fake socket 拼接完整帧。
        chunks: list[bytes] = []

        class FakeSock:
            def sendall(self, data: bytes) -> None:
                chunks.append(data)

        result = self.receiver._accept_trace(FakeSock(), header, payload)
        return result, b"".join(chunks)

    def test_first_accept_duplicate_conflict(self) -> None:
        payload = b"payload-1"
        header = _make_header("run1", 1, _sha(payload))
        path, sent = self._accept(header, payload)
        self.assertIsNotNone(path)
        self.assertTrue(path.exists())
        self.assertIn(b"ack_trace", sent)
        # 完全相同重传：不重复保存（同路径），返回 None（不重新显示）。
        path2, sent2 = self._accept(header, payload)
        self.assertIsNone(path2)
        self.assertIn(b"ack_trace", sent2)
        self.assertIn(b"duplicate", sent2)
        # 同编号不同 sha：冲突拒绝，不 ACK。
        bad = _make_header("run1", 1, "b" * 64)
        path3, sent3 = self._accept(bad, b"other")
        self.assertIsNone(path3)
        self.assertEqual(sent3, b"", "冲突不得 ACK")

    def test_metadata_not_duplicated(self) -> None:
        payload = b"px"
        header = _make_header("run2", 5, _sha(payload))
        path, _ = self._accept(header, payload)
        self._accept(header, payload)
        lines = (path.parent / "run_metadata.jsonl").read_text(encoding="utf-8").strip().splitlines()
        self.assertEqual(len(lines), 1, "重复重传不得再次写 metadata")


class GnssMapThrottleTests(unittest.TestCase):
    def setUp(self) -> None:
        from PySide6 import QtWidgets

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_unchanged_coordinates_skip_javascript(self) -> None:
        from gnss_map_widget import GnssMapWidget, MAP_UPDATE_INTERVAL_MS

        widget = GnssMapWidget()
        calls: list[tuple] = []
        js_times: list[float] = []
        clock = {"now": 0.0}
        widget._monotonic = lambda: clock["now"]
        widget._send_map_update = lambda lat, lon, popup: calls.append((lat, lon))
        widget._page_ready = True
        widget.update_fix(latitude=31.0, longitude=121.0, fix_quality=1, satellites=8)
        widget._map_timer.stop()
        self.assertEqual(len(calls), 1)
        # 坐标不变：不调度 JavaScript。
        widget.update_fix(latitude=31.0, longitude=121.0, fix_quality=1, satellites=8)
        widget._map_timer.stop()
        self.assertEqual(len(calls), 1, "坐标不变不得重复调用 runJavaScript")
        # 冷却期内的新坐标：立即 flush 不得发送（间隔 < 250ms）。
        clock["now"] = 0.05
        widget.update_fix(latitude=31.5, longitude=121.5, fix_quality=1, satellites=8)
        widget._map_timer.stop()
        widget._flush_map_update()
        self.assertEqual(len(calls), 1, "冷却期内不得发送（间隔 <250ms）")
        # 冷却期满后补发最新坐标。
        clock["now"] = MAP_UPDATE_INTERVAL_MS / 1000.0 + 0.01
        widget._flush_map_update()
        self.assertEqual(calls[-1], (31.5, 121.5))
        # 再次新坐标 + 冷却期满 → 最终新坐标显示。
        clock["now"] += MAP_UPDATE_INTERVAL_MS / 1000.0 + 0.01
        widget.update_fix(latitude=32.0, longitude=122.0, fix_quality=1, satellites=8)
        widget._map_timer.stop()
        widget._flush_map_update()
        self.assertEqual(calls[-1], (32.0, 122.0))

    def test_invalid_fix_keeps_pending(self) -> None:
        from gnss_map_widget import GnssMapWidget

        widget = GnssMapWidget()
        widget._page_ready = False
        widget.update_fix(latitude=31.0, longitude=121.0, fix_quality=1)
        self.assertIsNotNone(widget._pending_map_update, "页面未就绪时保留最新待更新位置")


if __name__ == "__main__":
    unittest.main()
