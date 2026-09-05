"""第1轮检查点A：pipeline_metrics 单元测试（纯 Python，无 Qt 依赖）。"""

from __future__ import annotations

import threading
import unittest

from pipeline_metrics import PipelineMetrics, QueueMetrics


class PipelineMetricsTests(unittest.TestCase):
    def test_empty_report(self) -> None:
        metrics = PipelineMetrics()
        report = metrics.stage_report("ground_displayed")
        self.assertEqual(report["count"], 0)
        self.assertIsNone(report["p95"])
        self.assertIsNone(report["max"])

    def test_percentiles_and_order_stats(self) -> None:
        metrics = PipelineMetrics()
        # 1..100 秒样本：p50 ≈ 50.5，p95 ≈ 95.05，max=100。
        for value in range(1, 101):
            metrics.record_stage("stage", float(value))
        report = metrics.stage_report("stage")
        self.assertEqual(report["count"], 100)
        self.assertEqual(report["max"], 100.0)
        self.assertEqual(report["latest"], 100.0)
        self.assertAlmostEqual(report["p50"], 50.5, places=2)
        self.assertAlmostEqual(report["p95"], 95.05, places=2)
        self.assertAlmostEqual(report["mean"], 50.5, places=2)

    def test_limited_sample_window(self) -> None:
        metrics = PipelineMetrics(max_samples=10)
        for value in range(100):
            metrics.record_stage("stage", float(value))
        report = metrics.stage_report("stage")
        self.assertEqual(report["count"], 10, "样本必须有上限")
        # 保留最新的 10 个样本（90..99）。
        self.assertEqual(report["max"], 99.0)
        self.assertEqual(report["p50"], 94.5)

    def test_invalid_values_rejected(self) -> None:
        metrics = PipelineMetrics()
        self.assertFalse(metrics.record_stage("stage", float("nan")))
        self.assertFalse(metrics.record_stage("stage", float("inf")))
        self.assertFalse(metrics.record_stage("stage", -1.0))
        self.assertFalse(metrics.record_stage("stage", "oops"))
        self.assertFalse(metrics.record_stage("", 0.1))
        self.assertEqual(metrics.stage_report("stage")["count"], 0)

    def test_thread_safety(self) -> None:
        metrics = PipelineMetrics(max_samples=4096)
        errors: list[BaseException] = []

        def worker() -> None:
            try:
                for i in range(500):
                    metrics.record_stage("concurrent", 0.001)
                    metrics.record_queue("q", depth=i % 5)
                    metrics.stage_report("concurrent")
                    metrics.snapshot()
            except BaseException as exc:  # noqa: BLE001
                errors.append(exc)

        threads = [threading.Thread(target=worker) for _ in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(errors, [])
        self.assertEqual(metrics.stage_report("concurrent")["count"], 4000)

    def test_measure_context_records_duration(self) -> None:
        metrics = PipelineMetrics()
        with metrics.measure("ctx"):
            pass
        report = metrics.stage_report("ctx")
        self.assertEqual(report["count"], 1)
        self.assertGreaterEqual(report["latest"], 0.0)
        # 异常不吞、但也记录。
        with self.assertRaises(RuntimeError):
            with metrics.measure("ctx_err"):
                raise RuntimeError("boom")
        self.assertEqual(metrics.stage_report("ctx_err")["count"], 1)

    def test_queue_metrics(self) -> None:
        metrics = PipelineMetrics()
        metrics.record_queue("processing_queue", depth=3)
        metrics.record_queue("processing_queue", depth=9, oldest_age_ms=12.5, high_water=True)
        metrics.record_queue("processing_queue", depth=1)
        report = metrics.queue_report("processing_queue")
        self.assertEqual(report["depth"], 1)
        self.assertEqual(report["peak"], 9)
        self.assertEqual(report["high_water_events"], 1)
        self.assertEqual(report["oldest_age_ms"], None)
        # 复审修复：record_queue 是快照，不得把"记录一次"计为 total_put。
        self.assertEqual(report["total_put"], 0)
        self.assertEqual(metrics.queue_report("missing")["depth"], 0)

    def test_latest_is_last_sample_not_max(self) -> None:
        """latest 必须是真正最后加入的样本（复审修复）。"""
        metrics = PipelineMetrics()
        # 降序加入：最后一条是 1.0（最小），latest 不得是 max。
        for value in (5.0, 3.0, 9.0, 1.0):
            metrics.record_stage("stage", value)
        report = metrics.stage_report("stage")
        self.assertEqual(report["latest"], 1.0)
        self.assertEqual(report["max"], 9.0)
        # 有限窗口下 latest 仍是最后保留的样本。
        metrics2 = PipelineMetrics(max_samples=2)
        for value in (5.0, 3.0, 9.0, 1.0):
            metrics2.record_stage("stage", value)
        report2 = metrics2.stage_report("stage")
        self.assertEqual(report2["latest"], 1.0)
        self.assertEqual(report2["count"], 2)

    def test_snapshot_jsonable(self) -> None:
        import json

        metrics = PipelineMetrics()
        metrics.record_stage("sweep_completed", 0.04)
        metrics.record_queue("display_fifo", depth=2, peak=5)
        snapshot = metrics.snapshot()
        json.dumps(snapshot)  # 不得抛出

    def test_reset_and_invalid_max_samples(self) -> None:
        metrics = PipelineMetrics()
        metrics.record_stage("x", 1.0)
        metrics.reset()
        self.assertEqual(metrics.stage_report("x")["count"], 0)
        with self.assertRaises(ValueError):
            PipelineMetrics(max_samples=0)


class QueueMetricsTests(unittest.TestCase):
    def test_defaults(self) -> None:
        data = QueueMetrics().as_dict()
        self.assertEqual(data["depth"], 0)
        self.assertEqual(data["peak"], 0)
        self.assertIsNone(data["oldest_age_ms"])


if __name__ == "__main__":
    unittest.main()
