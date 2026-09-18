"""第5轮 t5：B-scan 自动清空 + 暂停/停止排空可见化（Q1=B / Q2=A）。

覆盖：
1. 空中端：start_acquisition 触发 clear_bscan（新任务干净画布）；停止后历史保留。
2. 地面端：send_start / send_single 触发 clear_bscan；send_stop 不清。
3. 暂停排空可见化：paused 状态消息驱动"已暂停（排空中 · 还差 K 条）"，
   逐条递减真实计数，归零切"已暂停（待命）"；resume ack 清除暂停标记。
4. 停止排空文案与真实计数一致（不谎报）。
"""

from __future__ import annotations

import json
import socket
import struct
import tempfile
import threading
import time
import unittest
import unittest.mock
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
from PySide6 import QtWidgets

import uav_gpr_qt as air
from uav_gpr_ground_station import GroundStationWindow

_PROTOCOL = "uav-gpr-telemetry-v1"


def _send_wire_frame(sock: socket.socket, header: dict) -> None:
    """最小 wire 帧（与 send_remote_frame 同格式）：4B 头长 + 头 + 8B 载荷长。"""
    hb = bytes(json.dumps(header, ensure_ascii=False, separators=(",", ":")), "utf-8")
    sock.sendall(struct.pack("!I", len(hb)))
    sock.sendall(hb)
    sock.sendall(struct.pack("!Q", 0))


def _app() -> QtWidgets.QApplication:
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _fake_trace(time_end_ns: float = 40.0, points: int = 32) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    time_ns = np.linspace(0.0, time_end_ns, points)
    traces = {
        "S11": (np.sin(time_ns * 0.5)).astype(np.float64),
        "S22": (np.cos(time_ns * 0.5)).astype(np.float64),
        "S12": np.zeros(points, dtype=np.float64),
        "S21": np.zeros(points, dtype=np.float64),
    }
    return time_ns, traces


class AirStationAutoClearTests(unittest.TestCase):
    """空中端：开始新采集清空、停止保留。"""

    def setUp(self) -> None:
        self.app = _app()
        self.win = air.MainWindow()

    def test_start_acquisition_clears_bscan_history(self) -> None:
        time_ns, traces = _fake_trace()
        self.assertTrue(
            self.win._append_bscan_trace(time_ns, traces),
            "前置：注入历史道必须成功",
        )
        self.assertGreater(len(self.win.bscan["S11"]), 0)

        # 断言 clear_bscan 会被 start_acquisition 调用（spy 不替换行为）。
        calls: list[bool] = []
        original_clear = self.win.clear_bscan

        def spy_clear() -> None:
            calls.append(True)
            original_clear()

        self.win.clear_bscan = spy_clear  # type: ignore[method-assign]

        # 走 legacy 分支：worker 用哨兵替换（不真开 USB）。
        self.win.continuous_enabled.setChecked(False)
        self.win.remote_enable.setChecked(False)
        sentinel = MagicMock()
        sentinel.isRunning = MagicMock(return_value=False)
        sentinel.trace_ready = MagicMock()
        sentinel.status = MagicMock()
        sentinel.failed = MagicMock()
        sentinel.finished_run = MagicMock()
        with unittest.mock.patch.object(air, "AcquisitionWorker", return_value=sentinel):
            self.win.start_acquisition(single=True)

        self.assertTrue(calls, "start_acquisition 必须调用 clear_bscan（Q2=A）")
        self.assertEqual(len(self.win.bscan["S11"]), 0, "开始新采集后画布必须为空")
        self.assertEqual(self.win.bscan_time_ns, None)

    def test_stop_keeps_bscan_history_for_review(self) -> None:
        time_ns, traces = _fake_trace()
        self.win._append_bscan_trace(time_ns, traces)
        before = len(self.win.bscan["S11"])
        self.assertGreater(before, 0)
        # 停止路径（Q2=A 语义：停止后可回看，不清空）。
        self.win.stop_acquisition()
        self.assertEqual(len(self.win.bscan["S11"]), before, "停止后历史必须保留")


class GroundStationAutoClearTests(unittest.TestCase):
    """地面端：send_start/send_single 清空、send_stop 保留。"""

    def setUp(self) -> None:
        self.app = _app()
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        receiver = MagicMock()
        receiver.send_command = MagicMock()
        receiver.metrics = None
        self.win = GroundStationWindow(
            "127.0.0.1",
            0,
            Path(self._tmp.name),
            receiver=receiver,
            start_receiver=False,
        )

    def _seed_history(self) -> int:
        time_ns, traces = _fake_trace()
        ok = self.win._append_bscan_trace(time_ns, traces)
        assert ok, "前置：注入历史道必须成功"
        return len(self.win.bscan_rows["S11"])

    def test_send_start_clears_bscan(self) -> None:
        before = self._seed_history()
        self.assertGreater(before, 0)
        self.win.send_start()
        self.assertEqual(len(self.win.bscan_rows["S11"]), 0, "开始新采集必须清空（Q2=A）")
        self.assertEqual(self.win.bscan_time_ns, None)

    def test_send_single_clears_bscan(self) -> None:
        self._seed_history()
        self.win.send_single()
        self.assertEqual(len(self.win.bscan_rows["S11"]), 0, "单次采集同样是新任务，必须清空")

    def test_send_stop_keeps_bscan_history(self) -> None:
        before = self._seed_history()
        self.win.send_stop()
        self.assertEqual(len(self.win.bscan_rows["S11"]), before, "停止后历史必须保留（Q2=A）")


class DrainVisibilityTests(unittest.TestCase):
    """暂停/停止排空计数：来自真实字段，不谎报。"""

    def setUp(self) -> None:
        self.app = _app()
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        receiver = MagicMock()
        receiver.send_command = MagicMock()
        receiver.metrics = None
        self.win = GroundStationWindow(
            "127.0.0.1",
            0,
            Path(self._tmp.name),
            receiver=receiver,
            start_receiver=False,
        )

    def test_paused_status_shows_remaining_and_counts_down(self) -> None:
        # 建立任务基线（真实流程：send_start/_begin_task 后基线=当前任务）。
        self.win._task_run_id = "task-test"
        self.win._on_acquisition_status(
            {"state": "paused", "run_id": "task-test", "remaining": 37, "inflight": 3}
        )
        self.assertTrue(self.win._air_paused)
        self.assertIn("已暂停", self.win.acquisition_status_label.text())
        # 第5轮 F-A：K 口径与 stopping 统一 = remaining（37），不叠加 inflight。
        self.assertIn("还差 37 条", self.win.acquisition_status_label.text())
        self.assertIn("已暂停 · 还差 37 条", self.win.backlog_label.text())

        # 逐条递减（与 _on_trace_received 的计数路径同语义）
        self.win._air_unacknowledged = 0
        self.win._air_inflight = 0
        self._decrement_like_trace_receipt(self.win)
        self.assertIn("已暂停（待命）", self.win.acquisition_status_label.text())
        self.assertIn("已暂停（待命）", self.win.backlog_label.text())

        # resume ack 清除暂停标记
        self.win._on_command_status("Remote accepted: resume")
        self.assertFalse(self.win._air_paused)

    @staticmethod
    def _decrement_like_trace_receipt(win) -> None:
        """复用 _on_trace_received 的计数递减语义（镜像实现，保持一致）。"""
        if getattr(win, "_air_unacknowledged", 0) > 0:
            win._air_unacknowledged = max(0, win._air_unacknowledged - 1)
        if getattr(win, "_air_inflight", 0) > 0:
            win._air_inflight = max(0, win._air_inflight - 1)
        if hasattr(win, "backlog_label"):
            if getattr(win, "_air_paused", False):
                curr_state = "paused"
            elif "正在停止" in win.acquisition_status_label.text():
                curr_state = "stopping"
            else:
                curr_state = ""
            win.backlog_label.setText(win.backlog_text(state=curr_state))
        if getattr(win, "_air_paused", False):
            pending = int(getattr(win, "_air_unacknowledged", 0)) + int(
                getattr(win, "_air_inflight", 0)
            )
            if pending > 0:
                win.acquisition_status_label.setText(
                    f"采集状态：已暂停（排空中 · 还差 {pending} 条）"
                )
            else:
                win.acquisition_status_label.setText("采集状态：已暂停（待命）")
        elif "正在停止（排空中" in win.acquisition_status_label.text():
            rem = getattr(win, "_air_unacknowledged", 0)
            if rem > 0:
                win.acquisition_status_label.setText(
                    f"采集状态：正在停止（排空中 · 还差 {rem} 条）"
                )
            else:
                win.acquisition_status_label.setText("采集状态：正在停止（排空中）")

    def test_stopping_status_shows_remaining(self) -> None:
        self.win._task_run_id = "task-test"
        self.win._on_acquisition_status(
            {"state": "stopping", "run_id": "task-test", "remaining": 12, "inflight": 2}
        )
        self.assertIn("还差 12 条", self.win.acquisition_status_label.text())
        self.assertIn("排空中 · 还差 12 条", self.win.backlog_label.text())

    def test_zero_remaining_does_not_claim_count(self) -> None:
        self.win._on_acquisition_status(
            {"state": "stopping", "run_id": "task-test", "remaining": 0, "inflight": 0}
        )
        self.assertNotIn("还差", self.win.acquisition_status_label.text())
        self.assertNotIn("还差", self.win.backlog_label.text())

    def test_backlog_text_paused_unit_semantics(self) -> None:
        self.win._air_paused = True
        self.win._air_unacknowledged = 5
        self.win._air_inflight = 2
        # 第5轮 F-A：K=remaining（不叠加 inflight）。
        self.assertEqual(self.win.backlog_text(state="paused"), "已暂停 · 还差 5 条")
        self.win._air_unacknowledged = 0
        self.win._air_inflight = 0
        self.assertEqual(self.win.backlog_text(state="paused"), "已暂停（待命）")

    def test_late_paused_does_not_stick_and_acquiring_clears(self) -> None:
        """第5轮 F-B 回归（评审探针序列）：迟到 paused 不得粘染新任务。

        序列（复刻 t6 评审 probe_t6_ui.py 探针3 的生产复现路径）：
        旧 run paused 迟到 → 必须 ignore（标记不置位、标签不变）→
        新 run acquiring 接受 → 清标记 → 当前 run paused → 正常显示。
        """
        # 新任务基线已建立
        self.win._task_run_id = "task-0002"
        # 1. 旧 run 的 paused 迟到到达：必须整体忽略
        self.win._on_acquisition_status(
            {"state": "paused", "run_id": "task-0001", "remaining": 9, "inflight": 1}
        )
        self.assertFalse(
            self.win._air_paused, "迟到 paused 不得置粘性标记（F-B）"
        )
        self.assertNotIn("已暂停", self.win.acquisition_status_label.text())
        # 2. 新 run acquiring 接受：清粘性标记
        self.win._on_acquisition_status({"state": "acquiring", "run_id": "task-0002"})
        self.assertFalse(self.win._air_paused)
        # 3. 当前 run 的 paused：正常置位并显示真实计数
        self.win._on_acquisition_status(
            {"state": "paused", "run_id": "task-0002", "remaining": 5, "inflight": 2}
        )
        self.assertTrue(self.win._air_paused)
        self.assertIn("还差 5 条", self.win.acquisition_status_label.text())


class SlidingWindowDrainF1Tests(unittest.TestCase):
    """第5轮 F-1 回归：满窗等待中消化 ack 后，窗口有空位且有排队工作 → 立即返回。

    旧实现把剩余 select 超时等满（window=1 形态每条固定 +~100ms 尾延迟）。
    （放本文件的原因：t5 inScope 未含 test_round5_sliding_window.py，
    而该回归测试属 t5 交付。）
    """

    def test_drain_returns_immediately_when_space_and_work(self) -> None:
        sender = air.RemoteSender("127.0.0.1", 0, "run-f1", lambda s: None, window_size=1)
        client, peer = socket.socketpair()
        self.addCleanup(client.close)
        self.addCleanup(peer.close)

        header = {
            "type": "trace",
            "protocol": _PROTOCOL,
            "run_id": "run-f1",
            "trace_index": 1,
        }
        item = air._QueuedRemoteFrame(header, b"", time.perf_counter())
        with sender._inflight_lock:
            sender._inflight[1] = item
            sender._inflight_sent_at[1] = time.perf_counter()
        sender._outstanding = 1
        # 排队工作：队列里还有下一条（has_work=True 触发零超时轮询路径）。
        sender.queue.put(
            air._QueuedRemoteFrame(dict(header, trace_index=2), b"", time.perf_counter())
        )

        def _peer_send_ack() -> None:
            _send_wire_frame(
                peer,
                {
                    "type": "ack_trace",
                    "protocol": _PROTOCOL,
                    "run_id": "run-f1",
                    "trace_index": 1,
                },
            )

        threading.Thread(target=_peer_send_ack, daemon=True).start()

        t0 = time.perf_counter()
        sender._drain_incoming_frames(client, timeout=5.0)
        elapsed = time.perf_counter() - t0
        self.assertLess(
            elapsed,
            2.0,
            f"F-1：消化 ack 且窗口有空位+有排队工作时应立即返回，实测 {elapsed:.2f}s（旧实现等满 5s）",
        )
        self.assertEqual(sender.inflight_count(), 0)
        self.assertEqual(sender.outstanding_count(), 0)


if __name__ == "__main__":
    unittest.main()
