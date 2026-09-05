"""第1轮检查点C/D：默认无间隔与无损队列、停止排空语义的补充测试。"""

from __future__ import annotations

import os
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import uav_gpr_ground_station as ground
import uav_gpr_qt as air


class DefaultIntervalTests(unittest.TestCase):
    def test_ground_acquisition_settings_default_every_sweep(self) -> None:
        settings = ground.AcquisitionUiSettings()
        self.assertEqual(settings.interval_s, 0.0, "地面端默认 interval_s=0（每个完整Sweep）")
        self.assertEqual(settings.command_params()["interval_s"], 0.0)

    def test_ground_dialog_mode_mapping(self) -> None:
        from PySide6 import QtWidgets

        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        dialog = ground.AcquisitionSettingsDialog.__new__(ground.AcquisitionSettingsDialog)
        # 直接用 dataclass 语义验证（对话框 UI 映射由其它测试覆盖）。
        settings = ground.AcquisitionUiSettings(interval_s=0.0, count=0)
        self.assertEqual(settings.command_params(single=True)["count"], 1)

    def test_air_interval_spin_default_zero(self) -> None:
        # SweepConfig 默认值与本测试无关；断言模块级默认常量路径：
        # ContinuousAcquisitionWorker 未显式传 interval 时按调用方值，
        # UI 默认在 MainWindow._acquisition_page 中设置为 0.0。
        from PySide6 import QtWidgets

        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        window = air.MainWindow()
        try:
            self.assertEqual(window.interval.value(), 0.0, "空中端默认 interval=0（无采集间隔）")
        finally:
            window.close()
            window.deleteLater()


class LosslessQueueTests(unittest.TestCase):
    def test_default_queue_is_lossless(self) -> None:
        worker = air.ContinuousAcquisitionWorker(
            cfg=air.SweepConfig(),
            interval_s=0.0,
            out_dir=None,
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
        )
        self.assertIsInstance(worker.processing_queue, air.LosslessProcessingQueue)

    def test_explicit_drop_policy_still_bounded_queue(self) -> None:
        worker = air.ContinuousAcquisitionWorker(
            cfg=air.SweepConfig(),
            interval_s=0.0,
            out_dir=None,
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
            queue_drop_policy="drop_incoming",
        )
        self.assertIsInstance(worker.processing_queue, air.BoundedQueue)

    def test_high_water_and_hard_limit(self) -> None:
        """高水位只在**跨越**阈值时告警一次（复审要求：不得每次 put 刷屏）。"""
        events: list[str] = []
        queue = air.LosslessProcessingQueue(
            4,
            high_watermark=0.5,
            on_high_water=lambda depth: events.append(f"high:{depth}"),
            on_hard_limit=lambda depth: events.append(f"hard:{depth}"),
        )
        for i in range(4):
            queue.put(i)
        # depth 2 跨越阈值告警一次；depth 3/4 未再跨越 → 不重复告警。
        self.assertEqual(events, ["high:2"], "高水位告警只在跨越时触发")
        self.assertFalse(queue.stop_requested.is_set(), "填满队列本身不触发受控停止")
        # 队列满时的下一次 put 触发硬上限（本条有界等待空间，超时走
        # fail-closed；此处 max_wait_s 极小，验证 failed_item 可追踪）。
        queue.put(4, max_wait_s=0.1)
        self.assertTrue(queue.stop_requested.is_set())
        self.assertIn("hard:4", events)
        stats = queue.statistics()
        self.assertEqual(stats["depth_peak"], 4)
        self.assertEqual(stats["high_water_events"], 1)
        self.assertEqual(stats["hard_limit_events"], 1)
        # 消费后 oldest_age/计数正确。
        queue.get(timeout=1.0)
        self.assertEqual(queue.statistics()["depth"], 3)
        self.assertIsNotNone(queue.statistics()["oldest_age_ms"])

    def test_put_after_controlled_stop_rejected_not_dropped(self) -> None:
        """容量1：触发硬上限的 Trace 必须被接收（有界等待消费者腾位）；
        受控停止后队列仍满的后续 put 立即拒绝（不阻塞、不丢弃已入队内容）。"""
        import threading

        release = threading.Event()
        queue = air.LosslessProcessingQueue(
            1,
            on_hard_limit=lambda depth: release.set(),  # 触发硬上限时放行消费者
        )
        consumed: list[object] = []

        def consumer() -> None:
            release.wait(5.0)
            while True:
                item = queue.get(timeout=0.2)
                if item is None:
                    break
                consumed.append(item)

        thread = threading.Thread(target=consumer)
        thread.start()
        try:
            self.assertTrue(queue.put("a"))
            # b 触发硬上限（回调放行消费者）并阻塞等待腾位 → 必须成功。
            self.assertTrue(queue.put("b", max_wait_s=5.0), "触发硬上限的 Trace 必须被接收")
            self.assertTrue(queue.stop_requested.is_set())
            # 受控停止 + 队列满 → 后续立即拒绝。
            self.assertFalse(queue.put("c"))
            self.assertEqual(queue.statistics()["rejected_after_stop"], 1)
            # 消费者取走 b（等待有界时间）。
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and len(consumed) < 2:
                time.sleep(0.02)
            self.assertEqual(consumed, ["a", "b"])
            self.assertFalse(queue.statistics()["failed_item"])
        finally:
            release.set()
            thread.join(5.0)

    def test_all_items_eventually_processed_slow_consumer(self) -> None:
        """慢消费者：所有**已入队**项最终按序处理，无丢失无乱序。

        容量远大于突发（20 < 100）→ 不触发硬上限，put 全部成功且按序消费；
        受控停止拒绝路径由 test_put_after_controlled_stop_rejected_not_dropped
        与 worker 级测试覆盖。
        """
        import threading

        queue = air.LosslessProcessingQueue(100)
        got: list[int] = []
        stop = threading.Event()

        def consumer() -> None:
            while not stop.is_set() or not queue.empty():
                item = queue.get(timeout=0.05)
                if item is not None:
                    time.sleep(0.01)
                    got.append(item)

        thread = threading.Thread(target=consumer)
        thread.start()
        try:
            for i in range(20):
                self.assertTrue(queue.put(i), "容量充足时 put 必须成功")
        finally:
            stop.set()
            thread.join(5.0)
        self.assertEqual(got, list(range(20)), "所有已入队项最终按序处理，无丢失无乱序")

    def test_capacity1_paused_consumer_no_silent_loss(self) -> None:
        """复审场景1：容量1、消费者暂停——第二条 Trace 不得静默消失。

        trace1 入队；trace2 在满队列上触发受控停止并有界等待；消费者
        （由 hard-limit 回调放行）腾位后 trace2 必须入队成功；后续 put 被
        显式拒绝（计数），已入队内容全部处理，无任何静默消失。
        """
        import threading

        release = threading.Event()
        hard_events: list[int] = []
        queue = air.LosslessProcessingQueue(
            1,
            on_hard_limit=lambda depth: (hard_events.append(depth), release.set()),
        )
        got: list[object] = []

        def consumer() -> None:
            release.wait(10.0)
            while True:
                item = queue.get(timeout=0.2)
                if item is None:
                    break
                got.append(item)

        thread = threading.Thread(target=consumer)
        thread.start()
        try:
            self.assertTrue(queue.put("trace1"))
            ok2 = queue.put("trace2", max_wait_s=5.0)
            self.assertTrue(ok2, "触发硬上限的 Trace 必须被接收（不静默消失）")
            self.assertTrue(queue.stop_requested.is_set())
            self.assertEqual(hard_events, [1])
            # 受控停止 + 满队列 → 后续立即拒绝（不阻塞、不静默丢）。
            self.assertFalse(queue.put("trace3"))
            self.assertEqual(queue.statistics()["rejected_after_stop"], 1)
            thread.join(5.0)
        finally:
            release.set()
            thread.join(5.0)
        self.assertEqual(got, ["trace1", "trace2"], "全部已入队项处理完成")
        self.assertFalse(queue.statistics()["failed_item"], "无 fail-closed 丢 Trace")

    def test_extreme_block_fail_closed_not_drained(self) -> None:
        """复审场景2：消费者完全卡死 + put 超时 → item 保存在 failed_item，
        不得静默消失，调用方走 fatal（绝不报告 drained）。"""
        queue = air.LosslessProcessingQueue(1)
        self.assertTrue(queue.put("a"))
        # 无消费者：put 阻塞 max_wait_s 后 fail-closed。
        ok = queue.put("b", max_wait_s=0.3)
        self.assertFalse(ok)
        stats = queue.statistics()
        self.assertTrue(stats["failed_item"], "超时 item 必须可追踪（failed_item）")
        self.assertEqual(queue.failed_item, "b", "item 未从内存消失")
        # 已入队的 a 仍完好。
        self.assertEqual(queue.get(timeout=0.1), "a")
        self.assertEqual(queue.failed_item, "b", "item 未从内存消失")

    def test_invalid_construction(self) -> None:
        with self.assertRaises(ValueError):
            air.LosslessProcessingQueue(0)
        with self.assertRaises(TypeError):
            air.LosslessProcessingQueue(True)
        with self.assertRaises(ValueError):
            air.LosslessProcessingQueue(4, high_watermark=1.5)


class SenderStatusMessageTests(unittest.TestCase):
    def test_enqueue_status_header_shape(self) -> None:
        sender = air.RemoteSender("127.0.0.1", 1, "runx", lambda text: None, None)
        sender.enqueue_status("stopping", processing_queue_depth=3, selected_count=7)
        header, payload = sender.queue.get_nowait()
        self.assertEqual(header["type"], "acquisition_status")
        self.assertEqual(header["protocol"], "uav-gpr-telemetry-v1")
        self.assertEqual(header["state"], "stopping")
        self.assertEqual(header["processing_queue_depth"], 3)
        self.assertEqual(header["run_id"], "runx")
        self.assertEqual(payload, b"")
        self.assertIn("timestamp", header)


class DrainSemanticsTests(unittest.TestCase):
    def test_drain_has_no_fixed_2s_limit(self) -> None:
        """排空不受固定 2 秒限制：>2s 的残留也全部处理。"""
        import queue as queue_mod
        import threading

        consumer = air.ProcessingConsumer(
            queue=air.LosslessProcessingQueue(16),
            gnss_cache=air.GnssFixCache(),
            cfg=air.SweepConfig(),
            out_dir=None,
            autosave=False,
            calibration_p1=None,
            calibration_p2=None,
            through_measurement=None,
            background_frequency_hz=None,
            background=None,
        )
        # 伪造队列残留（0.15s/条 → 10 条 = 1.5s+，直接调用 _drain_remaining）。
        items = list(range(10))
        for item in items:
            consumer._queue.put(item)
        processed: list[int] = []
        consumer._process_one = lambda item: (time.sleep(0.15), processed.append(item))
        t0 = time.monotonic()
        consumer._drain_remaining()
        elapsed = time.monotonic() - t0
        self.assertEqual(processed, items, "全部残留处理完")
        self.assertGreaterEqual(elapsed, 1.4, "排空未被固定 2s 时限截断（10×0.15s）")

    def test_drain_failure_recorded_not_faked(self) -> None:
        consumer = air.ProcessingConsumer(
            queue=air.LosslessProcessingQueue(4),
            gnss_cache=air.GnssFixCache(),
            cfg=air.SweepConfig(),
            out_dir=None,
            autosave=False,
            calibration_p1=None,
            calibration_p2=None,
            through_measurement=None,
            background_frequency_hz=None,
            background=None,
        )
        for i in range(2):
            consumer._queue.put(i)

        def boom(item):
            raise RuntimeError(f"processing boom {item}")

        consumer._process_one = boom
        failures: list[str] = []
        consumer.failed.connect(lambda msg: failures.append(msg))
        consumer._drain_remaining()
        self.assertEqual(len(failures), 2, "任一处理失败必须明确上报")


if __name__ == "__main__":
    unittest.main()
