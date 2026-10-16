"""Regression tests for the headless field-test harness (roadmap §16 阶段5).

``librevna/field_test.py`` drives the real continuous-acquisition components
against a simulated streaming transport (``--simulate``) or, only with explicit
``--hardware`` confirmation, a real LibreVNA.  执行模式必须显式二选一；本套测试
全部使用模拟/fake/mock，绝不访问真实 USB。These tests assert the
machine-decidable exit-code contract:

- report JSON carries ``passed`` / ``failure_reasons`` / ``cleanup`` and
  ``rc`` is consistent with them (``rc==0`` only when every invariant and all
  cleanup succeed);
- probe fails closed on any integrity / sequence / channel / shape violation,
  on empty records (``all([])`` must not pass), and on close failure;
- simulated transport honours the ``LibreVnaTransportProtocol`` contract
  (``is_open`` property, idempotent open/close, single producer thread) with
  strict parameter validation;
- production threaded benchmark reports real ``processing_queue`` metrics and
  proves a slow consumer does not stop the USB producer;
- failures scenarios compute ``passed`` from actual conditions (pause_resume
  integrity, network_drop real enqueue+save, GNSS cache scope is honest);
- invalid CLI values and zero-data runs return non-zero.

Nothing here modifies the frozen phase-1..4 modules or the legacy backend.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import socket
import threading
import time
import unittest
from unittest import mock

import numpy as np

import air_core as U

from librevna.librevna_continuous import BackendState, PersistentBackendSettings, PersistentLibreVnaBackend
from librevna.librevna_errors import LibreVnaDeviceNotFoundError, LibreVnaError

import librevna.field_test as ft


# =====================================================================
# 测试辅助
# =====================================================================


def _run_cmd(fn, args) -> tuple[int, dict[str, object]]:
    """执行 cmd_*，捕获其 JSON 输出并返回 (rc, report)。

    stdout 可能含周期 ``[t=...]`` 状态行，取最后一个 JSON 文档（最终报告）。
    """
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = fn(args)
    text = buf.getvalue()
    # 周期行都是 [t=...]；最终报告的顶层 { 独占行首（内部嵌套 { 都有缩进）。
    last_brace_line = text.rfind("\n{")
    start = last_brace_line + 1 if last_brace_line != -1 else 0
    if start >= len(text) or text[start] != "{":
        raise AssertionError(f"命令输出中没有 JSON 报告: {text[:200]!r}")
    report = json.loads(text[start:])
    return rc, report


def _probe_args(**kw) -> argparse.Namespace:
    base = dict(points=201, start_mhz=400.0, stop_mhz=4000.0, simulate=True,
                hardware=False, rate=50.0, sweep_timeout=None, sample=10, json_out=None)
    base.update(kw)
    return argparse.Namespace(**base)


def _bench_args(**kw) -> argparse.Namespace:
    base = dict(layers="usb,proc,disk,net", seconds=3.0, interval=0.0, count=0,
                points=201, start_mhz=400.0, stop_mhz=4000.0, ifbw=10000,
                power=-20.0, dwell=0, ifft_points=2048, time_window_ns=40.0,
                dewow_window_ns=4.0, gnss="valid", gnss_max_age=5.0,
                sweep_timeout=None, out=None, json_out=None, mode="production",
                queue_maxsize=16, queue_drop_policy="drop_incoming",
                monitor_interval=0.5, baseline_json=None, simulate=True,
                hardware=False, rate=50.0)
    base.update(kw)
    return argparse.Namespace(**base)


def _longrun_args(**kw) -> argparse.Namespace:
    base = dict(layers="usb,proc,disk", duration=3.0, sample_every=1.5,
                interval=0.0, count=0, points=201, start_mhz=400.0,
                stop_mhz=4000.0, ifbw=10000, power=-20.0, dwell=0,
                ifft_points=2048, time_window_ns=40.0, dewow_window_ns=4.0,
                gnss="valid", gnss_max_age=5.0, sweep_timeout=None, out=None,
                json_out=None, queue_maxsize=16, queue_drop_policy="drop_incoming",
                monitor_interval=0.5, baseline_json=None, simulate=True,
                hardware=False, rate=50.0)
    base.update(kw)
    return argparse.Namespace(**base)


def _failures_args(**kw) -> argparse.Namespace:
    base = dict(points=201, start_mhz=400.0, stop_mhz=4000.0, simulate=True,
                hardware=False, rate=50.0, sweep_timeout=None, json_out=None)
    base.update(kw)
    return argparse.Namespace(**base)


def _opts(**kw) -> ft.RunOptions:
    values = dict(layers=("usb", "proc", "disk"), interval_s=0.0, count=0,
                  duration_s=3.0, points=201, start_mhz=400.0, stop_mhz=4000.0,
                  ifbw_hz=10000, power_dbm=-20.0, dwell_us=0, ifft_points=2048,
                  time_window_ns=40.0, dewow_window_ns=4.0, dewow_enabled=False,
                  simulate=True, hardware=False, rate=50.0, gnss_mode="valid", gnss_max_age_s=5.0,
                  sweep_timeout_s=None, out_dir=None, sample_every_s=None,
                  json_out=None, mode="production", queue_maxsize=16,
                  queue_drop_policy="drop_incoming", monitor_interval_s=0.5,
                  baseline_json=None)
    values.update(kw)
    return ft.RunOptions(**values)


def _make_valid_baseline(mode: str = "simulate", **overrides) -> dict[str, object]:
    """合法的纯 USB 基线 dict（供基线校验测试）。"""
    base: dict[str, object] = {
        "command": "bench",
        "mode": mode,
        "passed": True,
        "failure_reasons": [],
        "cleanup": {"ok": True, "clean": True, "clean_shutdown": True},
        "clean_shutdown": True,
        "layers": ["usb"],
        "topology": "synchronous component microbenchmark",
        "processed": 0,
        "saved": 0,
        "enqueued": 0,
        "sweeps_per_s": 50.0,
        "elapsed_s": 60.0,
        "config": {
            "points": 201, "start_mhz": 400.0, "stop_mhz": 4000.0,
            "ifbw_hz": 10000, "power_dbm": -20.0, "dwell_us": 0,
            "sweep_timeout_s": None, "interval_s": 0.0, "count": 0,
        },
        "device": {"firmware": "1.2.3", "protocol": 14, "hardware_version": 5},
    }
    base.update(overrides)
    return base


def _make_report_like(mode: str = "hardware", **overrides) -> dict[str, object]:
    """与当前运行报告同构的最小 dict（供基线校验测试）。"""
    base: dict[str, object] = {
        "mode": mode,
        "config": {
            "points": 201, "start_mhz": 400.0, "stop_mhz": 4000.0,
            "ifbw_hz": 10000, "power_dbm": -20.0, "dwell_us": 0,
            "sweep_timeout_s": None, "interval_s": 0.0, "count": 0,
        },
        "device": {"firmware": "1.2.3", "protocol": 14, "hardware_version": 5},
        "sweeps_per_s": 50.0,
    }
    base.update(overrides)
    return base


def _assert_failure_report_schema(report: dict[str, object]) -> None:
    """公共失败报告 schema 断言：passed/eligible/clean_shutdown 均 False、cleanup 三字段均 False、reasons 非空。"""
    assert report.get("passed") is False, f"passed 必须 False: {report.get('passed')}"
    assert report.get("acceptance_eligible") is False, f"eligible 必须 False: {report.get('acceptance_eligible')}"
    assert report.get("clean_shutdown") is False, f"clean_shutdown 必须 False: {report.get('clean_shutdown')}"
    cleanup = report.get("cleanup")
    assert isinstance(cleanup, dict), f"cleanup 必须 dict: {type(cleanup).__name__}"
    for key in ("ok", "clean", "clean_shutdown"):
        value = cleanup.get(key)
        assert type(value) is bool and value is False, f"cleanup.{key} 必须精确 bool False: {value!r}"
    reasons = report.get("failure_reasons")
    assert isinstance(reasons, list) and reasons, f"failure_reasons 必须非空 list: {reasons!r}"


class _FaultyReceiver(ft.LoopbackReceiver):
    """网络故障注入接收端：重复/缺失/乱序/重复替代缺失。仍正确 ACK（sender 继续）。"""

    def __init__(self, fault: str, **kwargs) -> None:
        super().__init__(**kwargs)
        self._fault = fault
        self._buf: Optional[int] = None

    def _record_trace(self, index: int) -> None:
        if self._fault == "duplicate":
            self.trace_indexes.append(index)
            self.trace_indexes.append(index)
        elif self._fault == "missing":
            if index == 2:
                return
            super()._record_trace(index)
        elif self._fault == "out_of_order":
            if self._buf is None:
                self._buf = index
            else:
                self.trace_indexes.append(index)  # 先记新的 -> 乱序
                self.trace_indexes.append(self._buf)
                self._buf = None
        elif self._fault == "dup_replacing_missing":
            if index == 1:
                self.trace_indexes.append(index)
                self.trace_indexes.append(index)  # 重复
                return
            if index == 2:
                return  # 缺失
            super()._record_trace(index)
        else:
            super()._record_trace(index)


class _BadSeqTransport(ft.StreamingSimulatedTransport):
    """第一道注入一个重复点 2（0,1,2,2,3...），后续道干净。"""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._dup_done = False

    def _next_point(self) -> bytes:
        if not self._dup_done and self._point_no == 2:
            self._dup_done = True
            return ft._point_payload(float(self._freqs[2]), 2,
                                     complex(0.5, -0.2), complex(0.3, 0.1))
        return super()._next_point()


class _NeverStreamsTransport(ft.StreamingSimulatedTransport):
    """命令正常应答，但从不产点（records 为空 / 零数据）。"""

    def _produce(self) -> None:
        self._stop_producer.wait()


class _FailingCloseTransport(ft.StreamingSimulatedTransport):
    """close 抛错；make_closable() 前 is_open 保持 True。

    close() 先停掉本地 producer（避免跨测试线程残留），再模拟"设备仍占端口"
    （``_open`` 保持 True 并抛错）。
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._closable = False

    def make_closable(self) -> None:
        self._closable = True

    @property
    def is_open(self) -> bool:
        if not self._closable and self._open:
            return True
        return super().is_open

    def close(self) -> None:
        if not self._closable:
            # 本地 producer 必须清理（模拟连接已断），但设备仍占端口：_open 保持 True。
            self._stop_producer.set()
            with self._cond:
                self._cond.notify_all()
            thread = self._producer
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=1.0)
            raise LibreVnaError("simulated close failure")
        super().close()


def _inject_build_backend(transport):
    """patch _build_backend 返回用给定 transport 构造的 backend。"""
    def factory(opts):
        backend = PersistentLibreVnaBackend(
            transport=transport,
            settings=PersistentBackendSettings(sweep_timeout_s=opts.sweep_timeout_s),
        )
        return backend, transport
    return mock.patch.object(ft, "_build_backend", side_effect=factory)


# =====================================================================
# 模拟传输契约
# =====================================================================


class SimulatedTransportContractTests(unittest.TestCase):
    def test_is_open_is_property_not_method(self) -> None:
        # backend 以属性访问 transport.is_open；方法恒真会导致 close() 误判。
        self.assertIsInstance(ft.StreamingSimulatedTransport.is_open, property)
        self.assertIsInstance(ft.TappingTransport.is_open, property)

    def test_open_idempotent_single_producer(self) -> None:
        transport = ft.StreamingSimulatedTransport(points=201, sweeps_per_sec=50)
        try:
            transport.open()
            transport.open()
            transport.open()
            producers = [t for t in threading.enumerate() if t.name == "field-test-sim-usb"]
            self.assertEqual(len(producers), 1, "重复 open 不得创建第二个 producer")
            self.assertEqual(transport.open_count, 3)
        finally:
            transport.close()

    def test_close_idempotent_no_residual_producer(self) -> None:
        transport = ft.StreamingSimulatedTransport(points=201, sweeps_per_sec=50)
        transport.open()
        transport.close()
        transport.close()
        transport.close()
        self.assertFalse(transport.is_open)
        self.assertFalse(transport.producer_alive())
        self.assertEqual(transport.close_count, 3)
        residual = [t for t in threading.enumerate() if t.name == "field-test-sim-usb"]
        self.assertEqual(residual, [], "close 后不得残留 field-test-sim-usb 线程")

    def test_strict_parameter_validation(self) -> None:
        bad = [
            {"points": 1}, {"points": 0}, {"points": 201.5}, {"points": True},
            {"points": False},
            {"start_hz": 400e6, "stop_hz": 400e6},
            {"start_hz": -1.0}, {"start_hz": float("nan")},
            {"sweeps_per_sec": 0}, {"sweeps_per_sec": -5},
            {"sweeps_per_sec": float("nan")}, {"sweeps_per_sec": float("inf")},
        ]
        for kwargs in bad:
            with self.subTest(kwargs=kwargs):
                with self.assertRaises((ValueError, TypeError)):
                    ft.StreamingSimulatedTransport(**kwargs)

    def test_streaming_produces_strict_0_to_n_minus_1(self) -> None:
        transport = ft.StreamingSimulatedTransport(points=101, sweeps_per_sec=300)
        transport.open()
        transport._streaming = True
        try:
            collected: list[int] = []
            start = time.time()
            while len(collected) < 101 * 3:
                if time.time() - start > 2.0:
                    break
                with transport._cond:
                    data = bytes(transport._buf)
                    transport._buf.clear()
                collected.extend(_extract_point_numbers(data))
            self.assertGreaterEqual(len(collected), 101 * 2)
            self.assertEqual(collected[:101], list(range(101)))
            self.assertEqual(collected[101:202], list(range(101)))
        finally:
            transport.close()


def _extract_point_numbers(data: bytes) -> list[int]:
    import struct
    numbers: list[int] = []
    offset = 0
    while offset + 4 <= len(data):
        if data[offset] != 0x5A:
            offset += 1
            continue
        length = struct.unpack_from("<H", data, offset + 1)[0]
        if length < 8 or offset + length > len(data):
            break
        if data[offset + 3] == ft.VNA_DATAPOINT:
            payload = data[offset + 4 : offset + length - 4]
            if len(payload) >= 12:
                numbers.append(struct.unpack_from("<hH", payload, 8)[1])
        offset += length
    return numbers


# =====================================================================
# probe
# =====================================================================


class ProbeTests(unittest.TestCase):
    def test_probe_simulate_passes_all_acceptance_fields(self) -> None:
        rc, report = _run_cmd(ft.cmd_probe, _probe_args())
        self.assertEqual(rc, 0)
        self.assertTrue(report["passed"])
        self.assertEqual(report["failure_reasons"], [])
        self.assertEqual(report["sample_complete"], report["sample"])
        self.assertTrue(report["point_sequence_strict_0_to_n_minus_1"])
        self.assertGreaterEqual(report["full_cycles_observed"], report["sample"])
        self.assertTrue(report["s11_s22_same_datapoint"])
        self.assertGreater(report["records_parsed"], 0)
        self.assertTrue(report["every_sweep_shape_and_finite"])
        self.assertEqual(report["integrity"]["duplicate_points"], 0)
        self.assertEqual(report["integrity"]["dropped_sweeps"], 0)
        self.assertFalse(report["acceptance_eligible"], "simulate 永不验收合格")
        self.assertTrue(report["cleanup"]["ok"])

    def test_probe_bad_sequence_returns_nonzero(self) -> None:
        transport = _BadSeqTransport(points=201, sweeps_per_sec=50)
        with _inject_build_backend(transport):
            rc, report = _run_cmd(ft.cmd_probe, _probe_args(sample=5))
        self.assertNotEqual(rc, 0)
        self.assertFalse(report["passed"])
        joined = " ".join(report["failure_reasons"])
        self.assertTrue(
            any(k in joined for k in ("integrity", "点序", "duplicate"))
            or report["integrity"]["duplicate_points"] != 0
        )

    def test_probe_empty_records_does_not_pass_via_all(self) -> None:
        # all([]) 为 True：空 records 绝不能通过 s11_s22_same_datapoint。
        transport = _NeverStreamsTransport(points=201, sweeps_per_sec=50)
        with _inject_build_backend(transport):
            rc, report = _run_cmd(ft.cmd_probe, _probe_args(sample=3))
        self.assertNotEqual(rc, 0)
        self.assertFalse(report["s11_s22_same_datapoint"])
        self.assertEqual(report["records_parsed"], 0)
        self.assertFalse(report["passed"])

    def test_probe_close_throws_still_open_fails(self) -> None:
        transport = _FailingCloseTransport(points=201, sweeps_per_sec=50)
        with _inject_build_backend(transport):
            rc, report = _run_cmd(ft.cmd_probe, _probe_args(sample=3))
        self.assertNotEqual(rc, 0)
        self.assertFalse(report["passed"])
        _assert_failure_report_schema(report)
        # 真实资源状态保留在独立字段（不参与最终 clean_shutdown 判定）。
        resource = report.get("resource_cleanup_before_report_failure", {})
        self.assertFalse(resource.get("backend_closed"))
        self.assertTrue(
            any("close" in r for r in report["failure_reasons"]),
            "close 失败必须在 failure_reasons 中体现",
        )

    def test_probe_close_throws_after_release_reports_diagnostic(self) -> None:
        transport = _FailingCloseTransport(points=201, sweeps_per_sec=50)
        transport.make_closable()
        # close 正常可释放 -> 干净通过。
        with _inject_build_backend(transport):
            rc, report = _run_cmd(ft.cmd_probe, _probe_args(sample=3))
        self.assertEqual(rc, 0)
        self.assertTrue(report["cleanup"]["ok"])

    def test_probe_device_not_found_returns_2(self) -> None:
        args = _probe_args(simulate=False, hardware=True)
        with mock.patch.object(
            PersistentLibreVnaBackend,
            "open",
            side_effect=LibreVnaDeviceNotFoundError("no LibreVNA"),
        ):
            rc, report = _run_cmd(ft.cmd_probe, args)
        self.assertEqual(rc, 2)
        self.assertFalse(report["passed"])


# =====================================================================
# 分层 bench（同步微基准 + 生产线程拓扑）
# =====================================================================


class LayeredBenchTests(unittest.TestCase):
    def test_pure_usb_processed_is_zero(self) -> None:
        rc, report = _run_cmd(ft.cmd_bench, _bench_args(layers="usb", seconds=3.0))
        self.assertEqual(rc, 0)
        self.assertEqual(report["processed"], 0, "纯 usb 层 processed 必须为 0")
        self.assertEqual(report["saved"], 0)
        self.assertEqual(report["enqueued"], 0)
        self.assertGreater(report["all_complete_sweeps"], 0)
        self.assertEqual(report["topology"], "synchronous component microbenchmark")
        self.assertFalse(report["acceptance_eligible"])
        self.assertIsNone(report["processing_queue"], "同步微基准不得伪造队列指标")

    def test_production_full_layers_clean_invariants(self) -> None:
        # rate=40：热状态余量（第3轮裁定延续；实测整机发热时 50/s 余量仅 4%，必然抖破）
        rc, report = _run_cmd(
            ft.cmd_bench, _bench_args(layers="usb,proc,disk,net", seconds=4.0, rate=40.0)
        )
        self.assertEqual(rc, 0)
        self.assertTrue(report["passed"])
        self.assertEqual(report["open_count"], 1)
        self.assertEqual(report["configure_count"], 1)
        self.assertEqual(report["processed"], report["selected_sweeps"])
        self.assertEqual(report["saved"], report["selected_sweeps"])
        self.assertEqual(report["enqueued"], report["selected_sweeps"])
        self.assertEqual(report["net"]["received"], report["enqueued"])
        self.assertEqual(report["net"]["trace_stats"]["duplicates"], 0)
        self.assertEqual(report["net"]["trace_stats"]["missing"], 0)
        self.assertIsNotNone(report["processing_queue"])
        self.assertGreaterEqual(report["processing_queue"]["total_put"], 1)
        self.assertLessEqual(report["processing_queue"]["overflows"], 0)
        self.assertTrue(report["cleanup"]["ok"])
        self.assertEqual(report["failure_reasons"], [])
        # JSON passed 与 rc 一致。
        self.assertEqual(report["passed"], rc == 0)

    def test_sync_microbench_label_and_processing_queue_none(self) -> None:
        rc, report = _run_cmd(
            ft.cmd_bench,
            _bench_args(layers="usb,proc,disk", mode="sync", seconds=3.0),
        )
        self.assertEqual(rc, 0)
        self.assertEqual(report["topology"], "synchronous component microbenchmark")
        self.assertIsNone(report["processing_queue"])
        self.assertFalse(report["acceptance_eligible"])
        self.assertGreater(report["processed"], 0)

    def test_longrun_command_is_longrun_and_production(self) -> None:
        rc, report = _run_cmd(ft.cmd_longrun, _longrun_args(duration=3.0, sample_every=1.5))
        self.assertEqual(rc, 0)
        self.assertEqual(report["command"], "longrun")
        self.assertIn("ContinuousAcquisitionWorker", report["topology"])
        self.assertFalse(report["acceptance_eligible"])  # 时长不足 + simulate
        self.assertTrue(report["cleanup"]["ok"])

    def test_zero_sweep_returns_nonzero(self) -> None:
        transport = _NeverStreamsTransport(points=201, sweeps_per_sec=50)
        with _inject_build_backend(transport):
            rc, report = _run_cmd(ft.cmd_bench, _bench_args(layers="usb", seconds=2.0))
        self.assertNotEqual(rc, 0)
        self.assertEqual(report["all_complete_sweeps"], 0)
        self.assertTrue(any("零" in r or "selected" in r or "sweep" in r for r in report["failure_reasons"]))

    def test_slow_consumer_usb_continues_and_queue_stats_accurate(self) -> None:
        opts = _opts(layers=("usb", "proc"), duration_s=3.0, rate=150.0,
                     queue_maxsize=4)
        calls = {"n": 0}

        class _SlowConsumer(ft._TimedConsumer):
            def _process_one(self, item):
                calls["n"] += 1
                time.sleep(0.05)
                super()._process_one(item)

        def factory(queue, cache):
            return _SlowConsumer(
                queue=queue, gnss_cache=cache, cfg=ft._sweep_config(opts),
                out_dir=opts.out_dir, autosave=False, calibration_p1=None,
                calibration_p2=None, through_measurement=None,
                background_frequency_hz=None, background=None, remote_sender=None,
            )

        report, failures, cleanup = ft.run_layered_production(opts, consumer_factory=factory)
        queue = report["processing_queue"]
        self.assertGreater(report["all_complete_sweeps"], 0, "慢消费者不得停止 USB 排流")
        self.assertGreater(queue["overflows"], 0, "慢消费者应触发队列溢出")
        self.assertGreater(queue["dropped"], 0)
        self.assertLess(report["processed"], report["selected_sweeps"],
                        "慢消费者应导致 processed < selected")
        self.assertEqual(queue["depth"], 0, "停止后队列应排空")
        self.assertEqual(report["open_count"], 1)
        # 溢出 run 必须是预期失败结果：不能 passed=true，且 failure_reason 准确。
        self.assertFalse(report["passed"], "溢出 run 不得报告 passed=true")
        self.assertTrue(
            any("processing_queue" in r for r in report["failure_reasons"]),
            f"缺少 processing_queue 溢出 failure_reason: {report['failure_reasons']}",
        )

    def test_baseline_ratio_boundaries_deterministic(self) -> None:
        """90% 门槛确定性单元测试：不依赖两个墙钟性能运行恰好达标。"""
        ratio, met = ft._baseline_ratio_met(0.89 * 50.0, 50.0)
        self.assertAlmostEqual(ratio, 0.89)
        self.assertFalse(met, "0.89 必须低于 90% 门槛")
        ratio, met = ft._baseline_ratio_met(0.90 * 50.0, 50.0)
        self.assertAlmostEqual(ratio, 0.90)
        self.assertTrue(met, "0.90 必须达标（>= 90%）")
        ratio, met = ft._baseline_ratio_met(0.91 * 50.0, 50.0)
        self.assertTrue(met, "0.91 必须达标")
        ratio, met = ft._baseline_ratio_met(0.0, 50.0)
        self.assertFalse(met)

    def test_baseline_90_percent_met_and_missing_reported(self) -> None:
        # 无基线 -> 未评估（不自动通过，且不具备验收资格）。
        rc, report = _run_cmd(ft.cmd_bench, _bench_args(layers="usb,proc,disk", seconds=3.0))
        self.assertEqual(rc, 0, f"无基线应 smoke 通过: {report['failure_reasons']}")
        self.assertIsInstance(report["baseline_90_percent_met"], str)
        self.assertIn("未评估", str(report["baseline_90_percent_met"]))
        self.assertFalse(report["acceptance_eligible"], "无基线不得具备验收资格")

        # 有基线（固定 rate=30，明显低于模拟生产能力 ~46-50）：稳定达标，无机器负载抖动。
        import tempfile
        from pathlib import Path
        baseline = _make_valid_baseline(mode="simulate", sweeps_per_s=30.0)
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as fh:
            json.dump(baseline, fh)
            baseline_path = fh.name
        try:
            rc, report = _run_cmd(
                ft.cmd_bench,
                _bench_args(layers="usb,proc,disk", seconds=3.0, baseline_json=baseline_path),
            )
            self.assertEqual(rc, 0, f"90% 集成必须通过: {report['failure_reasons']}")
            self.assertIs(
                report["baseline_90_percent_met"], True, f"{report['failure_reasons']}"
            )
            self.assertGreaterEqual(
                report["baseline_90_percent_ratio"], 0.9, f"{report['failure_reasons']}"
            )
        finally:
            Path(baseline_path).unlink(missing_ok=True)


# =====================================================================
# failures 场景
# =====================================================================


class FailuresTests(unittest.TestCase):
    def test_all_failure_scenarios_pass(self) -> None:
        rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        self.assertEqual(rc, 0)
        self.assertTrue(report["passed"])
        self.assertEqual(report["failure_reasons"], [])
        self.assertTrue(report["pause_resume"]["passed"])
        self.assertTrue(report["gnss_cache_no_fix_stale"]["passed"])
        self.assertTrue(report["network_drop"]["passed"])

    def test_gnss_scenario_scope_is_honest(self) -> None:
        _rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        gnss = report["gnss_cache_no_fix_stale"]
        self.assertIs(gnss["serial_disconnect_covered"], False,
                      "不得声称缓存 fake-clock 测试覆盖串口断开")
        self.assertEqual(gnss["scope"], "GnssFixCache 无 Fix/陈旧语义（不含 GNSS 串口断开/重连）")

    def test_network_drop_really_enqueues_and_saves(self) -> None:
        _rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        net = report["network_drop"]
        self.assertTrue(net["passed"])
        self.assertGreater(net["saved_while_network_down"], 0, "断网期间必须本地保存")
        self.assertGreater(net["enqueued_while_network_down"], 0, "必须真正调用 enqueue_trace")
        self.assertEqual(net["saved_while_network_down"], net["selected_while_network_down"])
        self.assertIs(net["sender_connect_failure_observed"], True, "必须观察到连接失败事件")
        self.assertIs(net["network_did_not_stop_usb"], True)
        self.assertIs(net["sender_never_connected"], True)
        self.assertTrue(net["cleanup"]["ok"])
        self.assertTrue(net["cleanup"]["sender_exited"], "断网后 sender 必须退出")

    def test_pause_resume_passed_false_on_integrity_violation(self) -> None:
        transport = _BadSeqTransport(points=201, sweeps_per_sec=50)
        opts = _opts()
        with _inject_build_backend(transport):
            result = ft._scenario_pause_resume(opts)
        self.assertFalse(result["passed"])
        self.assertTrue(
            any("integrity" in r or "trace_index" in r for r in result["failure_reasons"])
        )


# =====================================================================
# CLI 校验 / 退出码
# =====================================================================


class CliValidationTests(unittest.TestCase):
    def test_invalid_values_rejected(self) -> None:
        cases: list[tuple[str, argparse.Namespace]] = [
            ("bench", _bench_args(seconds=0)),
            ("bench", _bench_args(seconds=-5)),
            ("bench", _bench_args(seconds=float("nan"))),
            ("bench", _bench_args(seconds=float("inf"))),
            ("bench", _bench_args(points=0)),
            ("bench", _bench_args(points=-3)),
            ("bench", _bench_args(rate=0)),
            ("bench", _bench_args(rate=float("nan"))),
            ("bench", _bench_args(rate=-1)),
            ("bench", _bench_args(start_mhz=5000, stop_mhz=4000)),
            ("bench", _bench_args(interval=-1)),
            ("bench", _bench_args(count=-1)),
            ("bench", _bench_args(gnss_max_age=0)),
            ("probe", _probe_args(sample=0)),
            ("probe", _probe_args(sample=-2)),
            ("longrun", _longrun_args(duration=0)),
            ("longrun", _longrun_args(duration=2, sample_every=0)),
        ]
        fn_by_name = {"bench": ft.cmd_bench, "probe": ft.cmd_probe, "longrun": ft.cmd_longrun}
        for name, args in cases:
            with self.subTest(name=name, args=vars(args)):
                with self.assertRaises(SystemExit):
                    fn_by_name[name](args)

    def test_invalid_cumulative_layers_rejected(self) -> None:
        with self.assertRaises(SystemExit):
            ft.cmd_bench(_bench_args(layers="usb,disk"))

    def test_bench_device_not_found_returns_2(self) -> None:
        args = _bench_args(layers="usb", simulate=False, hardware=True)
        with mock.patch.object(
            PersistentLibreVnaBackend,
            "open",
            side_effect=LibreVnaDeviceNotFoundError("no LibreVNA"),
        ):
            rc, report = _run_cmd(ft.cmd_bench, args)
        self.assertEqual(rc, 2)
        self.assertFalse(report["passed"])
        self.assertTrue(report["failure_reasons"])
        self.assertTrue(report["cleanup"]["ok"], "open 失败时未打开的 backend 应干净释放")

    def test_production_mode_processing_queue_metrics_present(self) -> None:
        rc, report = _run_cmd(ft.cmd_bench, _bench_args(layers="usb,proc", seconds=3.0))
        self.assertEqual(rc, 0)
        self.assertIn("depth_peak", report["processing_queue"])
        self.assertIn("overflows", report["processing_queue"])
        self.assertIn("dropped", report["processing_queue"])
        self.assertGreaterEqual(report["processing_queue"]["total_put"], 1)


class CleanupTrackerTests(unittest.TestCase):
    def test_stubborn_thread_causes_failure(self) -> None:
        class _Stubborn(threading.Thread):
            def __init__(self) -> None:
                super().__init__(daemon=True)
                self._stop_event = threading.Event()

            def run(self):
                while not self._stop_event.wait(0.05):
                    pass

        thread = _Stubborn()
        thread.start()
        try:
            failures: list[str] = []
            tracker = ft._CleanupTracker(failures)
            tracker.stop_thread(thread, "feeder", request_stop=lambda: None)
            self.assertFalse(tracker.result.feeder_exited)
            self.assertTrue(any("残留" in r for r in failures))
        finally:
            thread._stop_event.set()
            thread.join(timeout=1.0)

    def test_sender_stop_false_causes_failure(self) -> None:
        class _StuckSender:
            def stop(self, timeout_ms=3000):
                return False

        failures: list[str] = []
        tracker = ft._CleanupTracker(failures)
        tracker.stop_sender(_StuckSender())
        self.assertFalse(tracker.result.sender_exited)
        self.assertTrue(any("sender" in r for r in failures))

    def test_close_error_and_still_open_fails(self) -> None:
        class _BadBackend:
            is_open = True

            def close(self):
                raise LibreVnaError("boom")

        failures: list[str] = []
        tracker = ft._CleanupTracker(failures)
        tracker.close_backend(_BadBackend())
        self.assertFalse(tracker.result.backend_closed)
        self.assertFalse(tracker.finish().ok)


class ReportConsistencyTests(unittest.TestCase):
    def test_rc_and_passed_and_failure_reasons_consistent(self) -> None:
        # 成功路径。
        rc, report = _run_cmd(ft.cmd_bench, _bench_args(layers="usb", seconds=2.0))
        self.assertEqual(report["passed"], rc == 0)
        self.assertEqual(len(report["failure_reasons"]) == 0, rc == 0)
        # 失败路径（坏序列 transport -> 零 sweep）。
        transport = _NeverStreamsTransport(points=201, sweeps_per_sec=50)
        with _inject_build_backend(transport):
            rc2, report2 = _run_cmd(ft.cmd_bench, _bench_args(layers="usb", seconds=2.0))
        self.assertNotEqual(rc2, 0)
        self.assertEqual(report2["passed"], rc2 == 0)
        self.assertTrue(report2["failure_reasons"])


class ProductionIntegrityTests(unittest.TestCase):
    """§3/§5/§6/§7/§8 fail-closed 关键测试。"""

    def test_production_integrity_error_fails(self) -> None:
        transport = _BadSeqTransport(points=201, sweeps_per_sec=50)
        with _inject_build_backend(transport):
            rc, report = _run_cmd(ft.cmd_bench, _bench_args(layers="usb,proc", seconds=3.0))
        self.assertNotEqual(rc, 0)
        self.assertFalse(report["passed"])
        self.assertGreater(report["all_complete_sweeps"], 0, "USB 应继续排流")
        self.assertTrue(
            any("integrity" in r for r in report["failure_reasons"]),
            f"integrity 非零必须出现在 failure_reasons: {report['failure_reasons']}",
        )

    def test_count_completion_stops_controller(self) -> None:
        opts = _opts(layers=("usb", "proc"), duration_s=60.0, count=1, rate=50.0)
        start = time.time()
        report, failures, cleanup = ft.run_layered_production(opts, purpose="bench")
        wall = time.time() - start
        self.assertLess(wall, 5.0, "count=1 应快速完成并同步停止 controller")
        self.assertEqual(report["selected_sweeps"], 1)
        self.assertTrue(cleanup.controller_exited, failures)
        self.assertTrue(cleanup.ok, failures)
        # 名义 --seconds 60 但实际快速完成：elapsed_s < 60，不得获得验收资格。
        self.assertLess(report["elapsed_s"], 60.0)
        self.assertFalse(report["acceptance_eligible"], "快速完成不得凭名义时长获得资格")

    def test_receiver_residual_fails_cleanup(self) -> None:
        class _StuckReceiver:
            port = 0
            received_traces = 0

            def close(self):
                pass

            def is_alive(self):
                return True

            def trace_stats(self):
                return {"received": 0, "unique": 0, "duplicates": 0,
                        "missing": 0, "out_of_order": 0, "max_index": None}

        failures: list[str] = []
        tracker = ft._CleanupTracker(failures)
        tracker.stop_receiver(_StuckReceiver())
        self.assertFalse(tracker.result.receiver_exited)
        self.assertFalse(tracker.finish().ok)
        self.assertTrue(any("receiver" in r for r in failures))

    def test_monitor_failure_not_overwritten_by_producer_check(self) -> None:
        class _StuckThread(threading.Thread):
            def __init__(self) -> None:
                super().__init__(daemon=True)
                self._stop_event = threading.Event()

            def run(self):
                while not self._stop_event.wait(0.05):
                    pass

        thread = _StuckThread()
        thread.start()
        try:
            failures: list[str] = []
            tracker = ft._CleanupTracker(failures)
            tracker.stop_thread(thread, "monitor", request_stop=lambda: None)
            self.assertFalse(tracker.result.monitor_exited)

            class _CleanTransport:
                def producer_alive(self):
                    return False

            tracker.check_producer(_CleanTransport())
            self.assertTrue(tracker.result.producer_exited)
            self.assertFalse(
                tracker.result.monitor_exited,
                "monitor 失败不得被 check_producer 覆盖",
            )
            self.assertFalse(tracker.finish().ok)
        finally:
            thread._stop_event.set()
            thread.join(timeout=1.0)

    def test_controller_residual_fails_cleanup(self) -> None:
        class _StuckThread(threading.Thread):
            def __init__(self) -> None:
                super().__init__(daemon=True)
                self._stop_event = threading.Event()

            def run(self):
                while not self._stop_event.wait(0.05):
                    pass

        thread = _StuckThread()
        thread.start()
        try:
            failures: list[str] = []
            tracker = ft._CleanupTracker(failures)
            tracker.stop_thread(thread, "controller")
            self.assertFalse(tracker.result.controller_exited)
            self.assertFalse(tracker.finish().ok)
        finally:
            thread._stop_event.set()
            thread.join(timeout=1.0)

    def test_release_over_2_5_seconds_fails(self) -> None:
        class _SlowCloseBackend:
            def __init__(self) -> None:
                self.is_open = True

            def close(self):
                time.sleep(2.51)
                self.is_open = False

        failures: list[str] = []
        tracker = ft._CleanupTracker(failures)
        tracker.close_backend(_SlowCloseBackend())
        self.assertFalse(tracker.result.release_within_2_5s)
        # 资源已退出（backend_closed True），但 2.5s 目标未达标 -> failure_reason。
        self.assertTrue(tracker.result.backend_closed)
        self.assertTrue(any("2.5" in r for r in failures), f"缺少 2.5s failure: {failures}")
        # failure_reason 存在即 passed=false（报告层由 _finalize_report 判定）。
        self.assertTrue(failures)

    def test_release_boundary_2_49_and_2_51(self) -> None:
        # roadmap §14.8：stop->USB 释放**严格小于** 2.5s；2.5s 本身不达标。
        self.assertTrue(ft._release_within_target(2.49))
        self.assertFalse(ft._release_within_target(2.5))
        self.assertFalse(ft._release_within_target(2.51))

    def test_longrun_uses_1800_second_threshold(self) -> None:
        ok_cleanup = ft.CleanupResult(backend_closed=True)
        kwargs = dict(mode="hardware", failures=[], cleanup=ok_cleanup,
                      release_ok=True, baseline_met=True, schedule_ok=True)
        self.assertTrue(ft._acceptance_eligible(elapsed_s=60.0, purpose="bench", **kwargs))
        self.assertFalse(ft._acceptance_eligible(elapsed_s=60.0, purpose="longrun", **kwargs),
                         "60s longrun 只是 smoke")
        self.assertFalse(ft._acceptance_eligible(elapsed_s=1799.0, purpose="longrun", **kwargs))
        self.assertTrue(ft._acceptance_eligible(elapsed_s=1800.0, purpose="longrun", **kwargs))
        sim_kwargs = {**kwargs, "mode": "simulate"}
        self.assertFalse(
            ft._acceptance_eligible(elapsed_s=1800.0, purpose="longrun", **sim_kwargs),
            "simulate 永不具备验收资格",
        )
        # schedule_ok=False（p95 超一个 sweep 周期 / 缺样本）不得获得资格。
        self.assertFalse(
            ft._acceptance_eligible(elapsed_s=1800.0, purpose="longrun",
                                    **{**kwargs, "schedule_ok": False}),
        )

    def test_missing_baseline_blocks_acceptance_eligibility(self) -> None:
        rc, report = _run_cmd(ft.cmd_bench, _bench_args(layers="usb,proc", seconds=3.0))
        self.assertEqual(rc, 0)
        self.assertIsInstance(report["baseline_90_percent_met"], str)
        self.assertFalse(report["acceptance_eligible"], "无基线不得具备验收资格")
        ok_cleanup = ft.CleanupResult(backend_closed=True)
        self.assertFalse(
            ft._acceptance_eligible(mode="hardware", elapsed_s=1800.0, purpose="longrun",
                                    failures=[], cleanup=ok_cleanup,
                                    release_ok=True, baseline_met=False, schedule_ok=True),
        )

    def test_simulated_or_failed_baseline_rejected(self) -> None:
        opts = _opts(simulate=False, hardware=True)
        report = _make_report_like(mode="hardware")
        reason = ft._baseline_invalid_reason(_make_valid_baseline(mode="simulate"), opts, report)
        self.assertIn("mode", reason, "simulate 基线不得用于硬件运行")
        reason = ft._baseline_invalid_reason(_make_valid_baseline(passed=False), opts, report)
        self.assertIsNotNone(reason)

    def test_non_usb_baseline_rejected(self) -> None:
        opts = _opts()
        report = _make_report_like()
        baseline = _make_valid_baseline(layers=["usb", "proc"])
        reason = ft._baseline_invalid_reason(baseline, opts, report)
        self.assertIn("layers", reason)

    def test_network_equal_count_with_duplicate_and_missing_fails(self) -> None:
        opts = _opts(layers=("usb", "proc", "net"), duration_s=3.0, rate=50.0)
        report, failures, _cleanup = ft.run_layered_production(
            opts, receiver_factory=lambda: _FaultyReceiver("dup_replacing_missing"), purpose="bench"
        )
        self.assertFalse(report["passed"])
        self.assertEqual(report["net"]["received"], report["enqueued"],
                         "received==enqueued 仍必须因 dup/missing 失败")
        self.assertGreater(report["net"]["trace_stats"]["duplicates"], 0)
        self.assertGreater(report["net"]["trace_stats"]["missing"], 0)
        joined = " ".join(failures)
        self.assertTrue(any(k in joined for k in ("unique", "duplicates", "missing")))

    def test_network_fault_variants_all_fail(self) -> None:
        for fault in ("duplicate", "missing", "out_of_order"):
            with self.subTest(fault=fault):
                opts = _opts(layers=("usb", "proc", "net"), duration_s=3.0, rate=50.0)
                report, failures, _cleanup = ft.run_layered_production(
                    opts, receiver_factory=lambda f=fault: _FaultyReceiver(f), purpose="bench"
                )
                self.assertFalse(report["passed"], f"{fault} 必须失败")
                self.assertTrue(report["failure_reasons"])

    def test_failures_report_has_top_level_cleanup(self) -> None:
        rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        self.assertEqual(rc, 0)
        for key in ("passed", "failure_reasons", "cleanup", "acceptance_eligible"):
            self.assertIn(key, report)
        self.assertTrue(report["cleanup"]["ok"])
        self.assertIn("resource_scenarios", report["cleanup"])

    def test_probe_invalid_points_and_rate_are_cleanly_rejected(self) -> None:
        with self.assertRaises(SystemExit):
            ft.cmd_probe(_probe_args(points=0))
        with self.assertRaises(SystemExit):
            ft.cmd_probe(_probe_args(rate=0))
        with self.assertRaises(SystemExit):
            ft.cmd_probe(_probe_args(rate=float("nan")))
        with self.assertRaises(SystemExit):
            ft.cmd_probe(_probe_args(start_mhz=5000, stop_mhz=4000))

    def test_worker_statistics_is_json_serializable_without_workaround(self) -> None:
        rc, report = _run_cmd(ft.cmd_bench, _bench_args(layers="usb,proc", seconds=3.0))
        self.assertEqual(rc, 0)
        self.assertNotIn("frozen_worker_statistics_defect_workaround", report,
                         "冻结缺陷修复后不得再有 workaround 兼容字段")
        self.assertNotIn("frozen_defect_note", report)
        self.assertFalse(hasattr(ft._TimedConsumer, "workaround_frozen_worker_statistics"))

    def test_existing_output_is_not_overwritten_or_miscounted(self) -> None:
        import shutil
        import tempfile
        from pathlib import Path
        base = Path(tempfile.mkdtemp(prefix="field_test_existing_"))
        try:
            stale_csv = base / "trace_00000_old.csv"
            stale_npz = base / "trace_00000_old.npz"
            stale_csv.write_text("stale", encoding="utf-8")
            stale_npz.write_bytes(b"stale")
            opts = _opts(layers=("usb", "proc", "disk"), duration_s=3.0, out_dir=base, rate=50.0)
            report, failures, cleanup = ft.run_layered_production(opts, purpose="bench")
            self.assertTrue(report["passed"], failures)
            self.assertEqual(report["saved"], report["selected_sweeps"])
            self.assertEqual(stale_csv.read_text(encoding="utf-8"), "stale", "旧 CSV 被覆盖")
            self.assertTrue(stale_npz.exists())
            subdirs = [p for p in base.iterdir() if p.is_dir()]
            self.assertTrue(subdirs, "本次运行应写入唯一 run 子目录")
        finally:
            shutil.rmtree(base, ignore_errors=True)

    def test_consumer_qthread_exit_state_reflects_real_object(self) -> None:
        """正常生产 run：cleanup.consumer_exited 来自真实 worker.consumer.isRunning()。"""
        opts = _opts(layers=("usb", "proc"), duration_s=2.0, rate=50.0)
        report, failures, cleanup = ft.run_layered_production(opts, purpose="bench")
        self.assertTrue(report["passed"], failures)
        self.assertTrue(cleanup.consumer_exited, failures)
        self.assertTrue(cleanup.worker_exited, failures)

    def test_consumer_residual_fails_cleanup(self) -> None:
        """残留 consumer（isRunning()==True）必须使 cleanup.ok=false、passed=false。"""

        class _StuckConsumer(ft._TimedConsumer):
            def run(self):
                time.sleep(0.01)  # 线程实际很快结束

            def isRunning(self):
                return True  # 声称仍运行

        opts = _opts(layers=("usb", "proc"), duration_s=2.0, count=1, rate=50.0)

        def factory(queue, cache):
            return _StuckConsumer(
                queue=queue, gnss_cache=cache, cfg=ft._sweep_config(opts),
                out_dir=opts.out_dir, autosave=False, calibration_p1=None,
                calibration_p2=None, through_measurement=None,
                background_frequency_hz=None, background=None, remote_sender=None,
            )

        report, failures, cleanup = ft.run_layered_production(
            opts, consumer_factory=factory, purpose="bench"
        )
        self.assertFalse(cleanup.consumer_exited, "残留 consumer 必须被检测为未退出")
        self.assertFalse(cleanup.ok, "残留 consumer 必须使 cleanup.ok=false")
        self.assertFalse(report["passed"], "残留 consumer 必须使 passed=false")
        self.assertTrue(
            any("consumer" in r for r in report["failure_reasons"]),
            f"缺少 consumer 残留 failure_reason: {report['failure_reasons']}",
        )

    def test_receiver_close_throws_after_release_fails(self) -> None:
        """close() 抛错但线程已退出：receiver_exited 可为 true，但必须有 failure_reason。"""

        class _CloseAfterReleaseReceiver(ft.LoopbackReceiver):
            def close(self):
                super().close()  # 正常停止并 join
                raise RuntimeError("simulated close-after-release failure")

        opts = _opts(layers=("usb", "proc", "net"), duration_s=2.0, rate=50.0)
        report, failures, cleanup = ft.run_layered_production(
            opts, receiver_factory=lambda: _CloseAfterReleaseReceiver(), purpose="bench"
        )
        # 不产生 Traceback：run_layered_production 正常返回报告。
        self.assertFalse(report["passed"])
        self.assertTrue(cleanup.receiver_exited, "线程已退出 receiver_exited 可为 true")
        self.assertTrue(
            any("close" in r for r in report["failure_reasons"]),
            f"close 抛错必须进入 failure_reasons: {report['failure_reasons']}",
        )

    def test_receiver_close_throws_still_alive_fails(self) -> None:
        """close() 抛错且线程仍活跃：receiver_exited=false、cleanup.ok=false、passed=false。"""

        class _FakeStuckReceiver:
            received_traces = 0

            def __init__(self) -> None:
                dead = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                dead.bind(("127.0.0.1", 0))
                self.port = dead.getsockname()[1]
                dead.close()

            def start(self):
                pass

            def close(self):
                raise RuntimeError("simulated stuck close failure")

            def is_alive(self):
                return True

            def trace_stats(self):
                return {"received": 0, "unique": 0, "duplicates": 0,
                        "missing": 0, "out_of_order": 0, "max_index": None}

        opts = _opts(layers=("usb", "proc", "net"), duration_s=2.0, rate=50.0)
        report, failures, cleanup = ft.run_layered_production(
            opts, receiver_factory=lambda: _FakeStuckReceiver(), purpose="bench"
        )
        self.assertFalse(report["passed"])
        self.assertFalse(cleanup.receiver_exited)
        self.assertFalse(cleanup.ok, "receiver 仍活跃必须使 cleanup.ok=false")
        self.assertTrue(
            any("close" in r for r in report["failure_reasons"]),
            f"close 抛错必须进入 failure_reasons: {report['failure_reasons']}",
        )

    def test_strict_json_rejects_callable_object_nan_inf(self) -> None:
        import io as _io
        cases = [
            {"bad": lambda: None},
            {"bad": object()},
            {"bad": float("nan")},
            {"bad": float("inf")},
        ]
        for payload in cases:
            with self.subTest(payload=type(payload["bad"]).__name__):
                report = {
                    "command": "bench", "passed": True, "acceptance_eligible": True,
                    "failure_reasons": [], "cleanup": {"ok": True},
                    **payload,
                }
                buf = _io.StringIO()
                args = argparse.Namespace(json_out=None)
                with contextlib.redirect_stdout(buf):
                    rc = ft._finalize_report(report, args, "bench", [], None,
                                             acceptance_eligible=True)
                self.assertEqual(rc, 2, f"非法对象必须 rc=2: {payload}")
                out = json.loads(buf.getvalue())  # 必须是严格合法 JSON
                self.assertFalse(out["passed"])
                self.assertFalse(out["acceptance_eligible"])
                joined = " ".join(out["failure_reasons"])
                self.assertTrue(any(k in joined for k in ("序列化", "非法字段")),
                                f"诊断必须列非法字段: {out['failure_reasons']}")

    def test_json_out_write_failure_returns_nonzero(self) -> None:
        from pathlib import Path as _Path
        cleanup = ft.CleanupResult(backend_closed=True)  # 有效 CleanupResult（clean_shutdown True）
        report: dict[str, object] = {"command": "bench", "passed": True,
                                     "acceptance_eligible": True, "failure_reasons": []}
        args = argparse.Namespace(json_out="unused.json")
        stdout_buf = io.StringIO()
        stderr_buf = io.StringIO()
        with mock.patch.object(_Path, "write_text", side_effect=OSError("disk full")):
            with contextlib.redirect_stdout(stdout_buf), contextlib.redirect_stderr(stderr_buf):
                rc = ft._finalize_report(report, args, "bench", [], cleanup,
                                         acceptance_eligible=True)
        self.assertEqual(rc, 2, "json_out 写入失败必须返回非零")
        # stdout 只含一个 JSON 文档：json.loads 对尾随数据会抛 Extra data。
        stdout_text = stdout_buf.getvalue().strip()
        out = json.loads(stdout_text)
        _assert_failure_report_schema(out)
        self.assertTrue(any("json_out" in r for r in out["failure_reasons"]))
        # 顶层与 nested cleanup 全部为 false（矛盾字段修复）。
        self.assertIs(out["clean_shutdown"], False)
        self.assertIs(out["cleanup"]["ok"], False)
        self.assertIs(out["cleanup"]["clean"], False)
        self.assertIs(out["cleanup"]["clean_shutdown"], False)
        # 写入失败前真实资源结果可保留在独立字段，但不使最终 cleanup 三字段变 true。
        if "resource_cleanup_before_report_failure" in out:
            self.assertIs(out["cleanup"]["ok"], False)
            self.assertIs(out["cleanup"]["clean"], False)
            self.assertIs(out["cleanup"]["clean_shutdown"], False)
        # stdout 不包含先前的成功报告。
        self.assertNotIn('"passed": true', stdout_text)
        # stderr 只含辅助信息（纯文本），不与 stdout 的最终结论冲突。
        self.assertIn("json_out 写入失败", stderr_buf.getvalue())
        self.assertNotIn('"passed": true', stderr_buf.getvalue())

    def test_no_python_field_test_threads_alive_at_return(self) -> None:
        """只声称检查 Python threading.Thread（Qt QThread 不出现在 enumerate()）。

        Qt QThread（worker/consumer）的真实退出由 cleanup.worker_exited /
        cleanup.consumer_exited（来自 isRunning() 检查）验证，见
        test_consumer_qthread_exit_state_reflects_real_object。
        """
        # rate=40：热状态余量（第3轮裁定延续）
        opts = _opts(layers=("usb", "proc", "disk", "net"), duration_s=2.0, rate=40.0)
        report, failures, cleanup = ft.run_layered_production(opts, purpose="bench")
        self.assertTrue(report["passed"], failures)
        self.assertTrue(cleanup.ok, failures)
        time.sleep(0.3)
        residual = [
            t.name
            for t in threading.enumerate()
            if t.name.startswith("field-test") or t.name == "uav-gpr-remote-sender"
        ]
        self.assertEqual(residual, [], f"返回时残留 Python 线程: {residual}")


class FailClosedStateProbeTests(unittest.TestCase):
    """§3：清理状态探测抛错必须 fail-closed 返回结构化结果，不得逃出。"""

    def test_safe_probe_is_alive_throws(self) -> None:
        failures: list[str] = []

        def throw():
            raise RuntimeError("boom")

        result, ok = ft._safe_probe(failures, "receiver.is_alive()", throw, default=True)
        self.assertTrue(result, "is_alive 抛错按未安全退出（True=仍活跃）处理")
        self.assertFalse(ok, "读取抛错 ok 必须为 False")
        self.assertTrue(any("receiver.is_alive()" in f for f in failures))
        self.assertFalse(any("Traceback" in f for f in failures))

    def test_check_producer_producer_alive_throws(self) -> None:
        class _BadTransport:
            def producer_alive(self):
                raise RuntimeError("boom")

        failures: list[str] = []
        tracker = ft._CleanupTracker(failures)
        tracker.check_producer(_BadTransport())  # 不得抛出
        self.assertFalse(tracker.result.producer_exited)
        self.assertFalse(tracker.result.clean, "producer_alive 抛错必须标记 unclean")
        self.assertTrue(any("producer_alive" in f for f in failures))

    def test_safe_probe_worker_and_consumer_isrunning_throws(self) -> None:
        def throw():
            raise RuntimeError("boom")

        failures: list[str] = []
        result, ok = ft._safe_probe(failures, "worker.isRunning()", throw, default=True)
        self.assertTrue(result)
        self.assertFalse(ok)
        self.assertTrue(any("worker.isRunning()" in f for f in failures))
        failures.clear()
        result, ok = ft._safe_probe(failures, "consumer.isRunning()", throw, default=True)
        self.assertTrue(result)
        self.assertFalse(ok)
        self.assertTrue(any("consumer.isRunning()" in f for f in failures))

    def test_stop_thread_ident_throws_returns_false(self) -> None:
        class _FakeThread:
            @property
            def ident(self):
                raise RuntimeError("boom")

        failures: list[str] = []
        exited, ok = ft._stop_thread(_FakeThread(), "x", failures)
        self.assertFalse(exited)
        self.assertFalse(ok)
        self.assertTrue(any("x.ident" in f for f in failures))


class StrictJsonDiagnosticsTests(unittest.TestCase):
    """§2：_find_invalid_paths 安全处理非法键/循环引用，诊断自身不抛。"""

    def _assert_fails_closed(self, payload: object) -> dict[str, object]:
        report = {
            "command": "bench", "passed": True, "acceptance_eligible": True,
            "failure_reasons": [], "cleanup": {"ok": True}, "payload": payload,
        }
        buf = io.StringIO()
        args = argparse.Namespace(json_out=None)
        with contextlib.redirect_stdout(buf):
            rc = ft._finalize_report(report, args, "bench", [], None, acceptance_eligible=True)
        self.assertEqual(rc, 2)
        out = json.loads(buf.getvalue())  # 必须是严格合法 JSON，无 Traceback
        self.assertFalse(out["passed"])
        self.assertFalse(out["acceptance_eligible"])
        self.assertTrue(out["failure_reasons"])
        return out

    def test_bad_dict_keys_and_cycles_all_fail_closed(self) -> None:
        self_ref: dict[str, object] = {}
        self_ref["self"] = self_ref
        nested: dict[str, object] = {"a": {"b": []}}
        nested["a"]["b"].append(nested)  # 嵌套循环容器
        cases = [
            {(1, 2): "bad"},
            {float("nan"): "bad"},
            self_ref,
            nested,
            {"fn": lambda: None},
            object(),
        ]
        for payload in cases:
            with self.subTest(payload=type(payload).__name__):
                out = self._assert_fails_closed(payload)
                joined = " ".join(out["failure_reasons"])
                self.assertTrue(any(k in joined for k in ("序列化", "非法字段")),
                                f"诊断必须列非法字段: {out['failure_reasons']}")
                # _find_invalid_paths 自身不得抛 RecursionError/TypeError。
                paths = ft._find_invalid_paths(payload)
                self.assertIsInstance(paths, list)

    def test_shared_reference_not_false_positive_circular(self) -> None:
        shared: list[object] = [1, 2]
        payload = {"a": shared, "b": shared}
        paths = ft._find_invalid_paths(payload)
        self.assertEqual([p for p in paths if "circular" in p], [],
                         "兄弟分支共享对象不得误报为循环引用")


class CleanupStructuredDiagnosticsTests(unittest.TestCase):
    """§二/§三：清理操作抛错结构化 fail-closed，不逃逸、不静默、有完整诊断。"""

    def _throwing_first(self, orig):
        state = {"n": 0}

        def wrapper(*args, **kwargs):
            state["n"] += 1
            if state["n"] == 1:
                raise RuntimeError("boom")
            return orig(*args, **kwargs)

        return wrapper

    def test_worker_request_stop_throws_fails_closed(self) -> None:
        # count=0 + 短 bound：worker 持续运行，controller 到点调 request_stop（第 1 次抛错）
        # -> control_error；finally 的第 2 次 request_stop 委托原实现才真正停 worker。
        opts = _opts(layers=("usb", "proc"), duration_s=1.0, count=0, rate=50.0)
        with mock.patch.object(
            U.ContinuousAcquisitionWorker,
            "request_stop",
            self._throwing_first(U.ContinuousAcquisitionWorker.request_stop),
        ), mock.patch.object(ft, "WORKER_WAIT_GRACE_S", 1.0):
            report, failures, cleanup = ft.run_layered_production(opts, purpose="bench")
        # 异常不逃逸：正常返回报告。
        self.assertTrue(
            any("worker.request_stop() 抛错: RuntimeError: boom" in f for f in failures),
            f"必须含角色+操作+类型+消息: {failures}",
        )
        self.assertFalse(report["passed"])
        self.assertFalse(report["acceptance_eligible"])
        self.assertFalse(report["clean_shutdown"])
        self.assertFalse(cleanup.clean, "request_stop 抛错必须标记 unclean")
        json.dumps(report, allow_nan=False)

    def test_controller_request_stop_throws_fails_closed(self) -> None:
        opts = _opts(layers=("usb", "proc"), duration_s=60.0, count=1, rate=50.0)
        with mock.patch.object(
            ft._ProductionController,
            "request_stop",
            self._throwing_first(ft._ProductionController.request_stop),
        ):
            report, failures, cleanup = ft.run_layered_production(opts, purpose="bench")
        self.assertTrue(
            any("controller.request_stop() 抛错: RuntimeError: boom" in f for f in failures),
            f"缺少 controller.request_stop 诊断: {failures}",
        )
        self.assertFalse(report["passed"])
        self.assertFalse(report["acceptance_eligible"])
        self.assertFalse(report["clean_shutdown"])

    def test_controller_join_throws_fails_closed(self) -> None:
        opts = _opts(layers=("usb", "proc"), duration_s=60.0, count=1, rate=50.0)
        with mock.patch.object(
            ft._ProductionController,
            "join",
            self._throwing_first(ft._ProductionController.join),
        ):
            report, failures, cleanup = ft.run_layered_production(opts, purpose="bench")
        self.assertTrue(
            any("controller.join() 抛错: RuntimeError: boom" in f for f in failures),
            f"缺少 controller.join 诊断: {failures}",
        )
        self.assertFalse(report["passed"])
        self.assertFalse(report["clean_shutdown"])

    def test_controller_is_alive_throws_fails_closed(self) -> None:
        opts = _opts(layers=("usb", "proc"), duration_s=60.0, count=1, rate=50.0)
        with mock.patch.object(
            ft._ProductionController,
            "is_alive",
            self._throwing_first(ft._ProductionController.is_alive),
        ):
            report, failures, cleanup = ft.run_layered_production(opts, purpose="bench")
        self.assertTrue(
            any("controller.is_alive()" in f for f in failures),
            f"缺少 controller.is_alive 诊断: {failures}",
        )
        self.assertFalse(report["passed"])
        self.assertFalse(report["clean_shutdown"])

    def test_interval_zero_schedule_not_applicable(self) -> None:
        opts = _opts(layers=("usb", "proc"), duration_s=2.0, interval_s=0.0, rate=50.0)
        report, failures, cleanup = ft.run_layered_production(opts, purpose="bench")
        self.assertTrue(report["passed"], failures)
        self.assertFalse(report["schedule_applicable"])
        self.assertTrue(report["schedule_reason"])
        self.assertFalse(report["acceptance_eligible"])

    def test_positive_interval_and_baseline_can_achieve_eligibility(self) -> None:
        """确定性：_acceptance_eligible 各门满足时 eligibility=true（纯函数，无墙钟依赖）。"""
        ok_cleanup = ft.CleanupResult(backend_closed=True)
        kwargs = dict(mode="hardware", elapsed_s=1800.0, purpose="longrun",
                      failures=[], cleanup=ok_cleanup, release_ok=True,
                      baseline_met=True, schedule_ok=True)
        self.assertTrue(ft._acceptance_eligible(**kwargs))
        self.assertFalse(ft._acceptance_eligible(**{**kwargs, "schedule_ok": False}))
        self.assertFalse(ft._acceptance_eligible(**{**kwargs, "baseline_met": False}))
        self.assertFalse(ft._acceptance_eligible(**{**kwargs, "release_ok": False}))

    def test_schedule_helper_boundaries(self) -> None:
        # 缺失样本（count=0）
        ok, period, p95 = ft._schedule_error_within_sweep_period(
            {"schedule_error_ms": {"count": 0, "p95": None}}, {"sweeps_per_s": 10.0}
        )
        self.assertFalse(ok)
        # NaN p95
        ok, _p, _m = ft._schedule_error_within_sweep_period(
            {"schedule_error_ms": {"count": 5, "p95": float("nan")}}, {"sweeps_per_s": 10.0}
        )
        self.assertFalse(ok)
        # Inf p95
        ok, _p, _m = ft._schedule_error_within_sweep_period(
            {"schedule_error_ms": {"count": 5, "p95": float("inf")}}, {"sweeps_per_s": 10.0}
        )
        self.assertFalse(ok)
        # 无基线 -> 无法换算周期
        ok, _p, _m = ft._schedule_error_within_sweep_period(
            {"schedule_error_ms": {"count": 5, "p95": 10.0}}, None
        )
        self.assertFalse(ok)
        # rate=100 -> 周期 10ms；边界：p95 == 10ms -> 达标
        ok, period, _m = ft._schedule_error_within_sweep_period(
            {"schedule_error_ms": {"count": 5, "p95": 10.0}}, {"sweeps_per_s": 100.0}
        )
        self.assertEqual(period, 10.0)
        self.assertTrue(ok, "恰好等于 sweep 周期应达标")
        # 小于边界 -> 达标
        ok, _p, _m = ft._schedule_error_within_sweep_period(
            {"schedule_error_ms": {"count": 5, "p95": 9.99}}, {"sweeps_per_s": 100.0}
        )
        self.assertTrue(ok)
        # 大于边界 -> 不达标
        ok, _p, _m = ft._schedule_error_within_sweep_period(
            {"schedule_error_ms": {"count": 5, "p95": 10.01}}, {"sweeps_per_s": 100.0}
        )
        self.assertFalse(ok)


class HardwareGateAndCleanShutdownTests(unittest.TestCase):
    """§一/§二/§四：显式执行模式门 + clean_shutdown 硬性通过门。"""

    def test_cli_requires_exec_mode_neither_rejected_no_backend(self) -> None:
        args = _probe_args(simulate=False, hardware=False)
        with mock.patch.object(ft, "_build_backend") as mb:
            with self.assertRaises(SystemExit):
                ft.cmd_probe(args)
            mb.assert_not_called()
        with mock.patch.object(ft, "_build_backend") as mb:
            with self.assertRaises(SystemExit):
                ft.cmd_bench(_bench_args(simulate=False, hardware=False))
            mb.assert_not_called()

    def test_cli_requires_exec_mode_both_rejected(self) -> None:
        args = _probe_args(simulate=True, hardware=True)
        with mock.patch.object(ft, "_build_backend") as mb:
            with self.assertRaises(SystemExit):
                ft.cmd_probe(args)
            mb.assert_not_called()

    def test_cli_simulate_runs(self) -> None:
        rc, report = _run_cmd(ft.cmd_probe, _probe_args())  # simulate=True, hardware=False
        self.assertEqual(rc, 0)

    def test_cli_hardware_with_fake_backend_runs(self) -> None:
        # hardware=True：用 mock backend.open 抛错模拟"设备不可用"，不访问真实 USB。
        args = _probe_args(simulate=False, hardware=True)
        with mock.patch.object(
            PersistentLibreVnaBackend, "open",
            side_effect=LibreVnaDeviceNotFoundError("fake no device"),
        ):
            rc, report = _run_cmd(ft.cmd_probe, args)
        self.assertEqual(rc, 2)
        self.assertFalse(report["passed"])

    def test_real_mode_not_implicit_by_missing_simulate(self) -> None:
        # 缺少执行模式（无 --simulate 也无 --hardware）必须拒绝，绝不隐式连接真机。
        for fn, args in (
            (ft.cmd_probe, _probe_args(simulate=False, hardware=False)),
            (ft.cmd_bench, _bench_args(simulate=False, hardware=False)),
            (ft.cmd_longrun, _longrun_args(simulate=False, hardware=False)),
            (ft.cmd_failures, _failures_args(simulate=False, hardware=False)),
        ):
            with self.subTest(fn=fn.__name__):
                with mock.patch.object(ft, "_build_backend") as mb:
                    with self.assertRaises(SystemExit):
                        fn(args)
                    mb.assert_not_called()

    def test_cleanup_ok_true_clean_false_not_eligible(self) -> None:
        cleanup = ft.CleanupResult(backend_closed=True, clean=False)
        self.assertTrue(cleanup.ok)
        self.assertFalse(cleanup.clean_shutdown)
        self.assertFalse(
            ft._acceptance_eligible(
                mode="hardware", elapsed_s=1800.0, purpose="longrun", failures=[],
                cleanup=cleanup, release_ok=True, baseline_met=True, schedule_ok=True,
            )
        )

    def test_finalize_clean_shutdown_false_forces_failure(self) -> None:
        cleanup = ft.CleanupResult(backend_closed=True, clean=False)
        report: dict[str, object] = {"command": "bench"}
        buf = io.StringIO()
        args = argparse.Namespace(json_out=None)
        with contextlib.redirect_stdout(buf):
            rc = ft._finalize_report(report, args, "bench", [], cleanup,
                                     acceptance_eligible=True)
        self.assertEqual(rc, 2)
        out = json.loads(buf.getvalue())
        _assert_failure_report_schema(out)

    def test_baseline_ok_true_clean_false_invalid(self) -> None:
        baseline = _make_valid_baseline(
            cleanup={"ok": True, "clean": False, "clean_shutdown": False}
        )
        reason = ft._baseline_invalid_reason(baseline, _opts(), _make_report_like())
        self.assertIsNotNone(reason)
        self.assertIn("clean_shutdown", reason)

    def test_failures_top_clean_shutdown_false(self) -> None:
        fake_result = {
            "passed": False,
            "failure_reasons": ["boom"],
            "cleanup": {"ok": True, "clean": False, "clean_shutdown": False,
                        "details": ["sub-scenario unclean"]},
        }
        with mock.patch.object(ft, "_scenario_pause_resume", return_value=fake_result), \
             mock.patch.object(ft, "_scenario_network_drop", return_value=fake_result):
            rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        self.assertEqual(rc, 2)
        self.assertFalse(report["clean_shutdown"])
        self.assertFalse(report["passed"])
        self.assertFalse(report["acceptance_eligible"])
        self.assertTrue(report["failure_reasons"])

    def test_sender_stop_throws_clean_false(self) -> None:
        class _ThrowingSender:
            def stop(self, timeout_ms=3000, *, drain=True):
                raise RuntimeError("boom")

        failures: list[str] = []
        tracker = ft._CleanupTracker(failures)
        tracker.stop_sender(_ThrowingSender())
        self.assertFalse(tracker.result.clean)
        self.assertFalse(tracker.result.sender_exited)
        self.assertTrue(any("sender.stop() 抛错: RuntimeError: boom" in f for f in failures))

    def test_worker_wait_first_throws_then_cleanup(self) -> None:
        opts = _opts(layers=("usb", "proc"), duration_s=60.0, count=1, rate=50.0)
        state = {"n": 0}
        orig = U.ContinuousAcquisitionWorker.wait

        def wrapper(self, *a, **k):
            state["n"] += 1
            if state["n"] == 1:
                raise RuntimeError("wait boom")
            return orig(self, *a, **k)

        with mock.patch.object(U.ContinuousAcquisitionWorker, "wait", wrapper):
            report, failures, cleanup = ft.run_layered_production(opts, purpose="bench")
        # 异常不逃逸；报告严格 JSON。
        json.dumps(report, allow_nan=False)
        self.assertFalse(report["passed"])
        self.assertFalse(report["clean_shutdown"])
        self.assertFalse(report["acceptance_eligible"])
        self.assertTrue(
            any("worker.wait() 抛错: RuntimeError: wait boom" in f for f in failures),
            f"必须含 worker.wait 诊断: {failures}",
        )
        time.sleep(0.3)
        residual = [
            t.name
            for t in threading.enumerate()
            if t.name.startswith("field-test") or t.name == "uav-gpr-remote-sender"
        ]
        self.assertEqual(residual, [], f"worker.wait 异常后残留线程: {residual}")


    def test_finalize_pre_set_eligible_bypass_closed(self) -> None:
        """调用方已预置 report["acceptance_eligible"]=True 且不传形参时，仍须过 clean_shutdown 硬门。"""
        cleanup = ft.CleanupResult(backend_closed=True, clean=False)  # ok=True, clean_shutdown=False
        report: dict[str, object] = {"command": "bench", "acceptance_eligible": True}
        buf = io.StringIO()
        args = argparse.Namespace(json_out=None)
        with contextlib.redirect_stdout(buf):
            rc = ft._finalize_report(report, args, "bench", [], cleanup)  # 不传 acceptance_eligible
        self.assertEqual(rc, 2)
        out = json.loads(buf.getvalue())
        self.assertFalse(out["passed"])
        self.assertFalse(out["clean_shutdown"])
        self.assertFalse(out["acceptance_eligible"], "预置 eligibility=true 也必须被硬门强制为 false")
        self.assertTrue(out["failure_reasons"])
        self.assertFalse(out["passed"] is True and out["clean_shutdown"] is False and out["acceptance_eligible"] is True)

    def test_build_backend_strict_defense_no_real_transport(self) -> None:
        """内部 API 防御：非 bool 或非 XOR 模式一律 ValueError，且 LibreVnaUsbTransport 不被构造。"""
        cases = [
            dict(simulate=True, hardware=True),
            dict(simulate=False, hardware=False),
            dict(simulate=False, hardware="false"),
            dict(simulate=False, hardware=1),
            dict(simulate="true", hardware=False),
        ]
        for kw in cases:
            with self.subTest(**kw):
                opts = ft.RunOptions(**kw)
                with mock.patch.object(ft, "LibreVnaUsbTransport") as mtu:
                    with self.assertRaises(ValueError):
                        ft._build_backend(opts)
                    mtu.assert_not_called()

    def test_build_backend_simulate_true_only_sim_transport(self) -> None:
        opts = ft.RunOptions(simulate=True, hardware=False, points=201,
                             start_mhz=400.0, stop_mhz=4000.0, rate=50.0)
        with mock.patch.object(ft, "LibreVnaUsbTransport") as mtu, \
             mock.patch.object(ft, "StreamingSimulatedTransport") as mst:
            ft._build_backend(opts)
            mtu.assert_not_called()
            mst.assert_called_once()

    def test_failures_aggregation_ok_only_cleanup_fails(self) -> None:
        fake_ok_only = {"passed": True, "failure_reasons": [], "cleanup": {"ok": True}}
        with mock.patch.object(ft, "_scenario_pause_resume", return_value=fake_ok_only), \
             mock.patch.object(ft, "_scenario_network_drop", return_value=fake_ok_only):
            rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        self.assertEqual(rc, 2, "仅 ok=true 的子场景不得通过聚合门")
        self.assertFalse(report["clean_shutdown"])
        self.assertFalse(report["passed"])

    def test_failures_aggregation_missing_cleanup_fails(self) -> None:
        fake_missing = {"passed": True, "failure_reasons": []}  # 无 cleanup
        with mock.patch.object(ft, "_scenario_pause_resume", return_value=fake_missing), \
             mock.patch.object(ft, "_scenario_network_drop", return_value=fake_missing):
            rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        self.assertEqual(rc, 2)
        self.assertFalse(report["clean_shutdown"])

    def test_failures_aggregation_clean_shutdown_truthy_rejected(self) -> None:
        for bad in (1, "true"):
            with self.subTest(bad=bad):
                fake = {"passed": True, "failure_reasons": [],
                        "cleanup": {"ok": True, "clean": True, "clean_shutdown": bad}}
                with mock.patch.object(ft, "_scenario_pause_resume", return_value=fake), \
                     mock.patch.object(ft, "_scenario_network_drop", return_value=fake):
                    rc, report = _run_cmd(ft.cmd_failures, _failures_args())
                self.assertEqual(rc, 2, f"clean_shutdown={bad!r} 必须失败")
                self.assertFalse(report["clean_shutdown"])

    def test_failures_aggregation_both_clean_shutdown_true_passes(self) -> None:
        good = {"passed": True, "failure_reasons": [],
                "cleanup": {"ok": True, "clean": True, "clean_shutdown": True, "details": []}}
        with mock.patch.object(ft, "_scenario_pause_resume", return_value=good), \
             mock.patch.object(ft, "_scenario_network_drop", return_value=good):
            rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        self.assertEqual(rc, 0, f"两个资源场景 clean_shutdown=True 应通过聚合门: {report['failure_reasons']}")
        self.assertTrue(report["clean_shutdown"])


class CleanupEvidenceSchemaTests(unittest.TestCase):
    """§一/§二/§三/§四：缺失 cleanup 证据 fail-closed、三字段严格校验、子场景 schema、eligibility 类型。"""

    def test_cleanup_dict_strict_helper(self) -> None:
        self.assertTrue(ft._cleanup_dict_is_strictly_clean(
            {"ok": True, "clean": True, "clean_shutdown": True}))
        cases = [
            None,
            "true",
            {"ok": True, "clean": True},  # 缺 clean_shutdown
            {"ok": True, "clean": True, "clean_shutdown": False},
            {"ok": True, "clean": False, "clean_shutdown": True},
            {"ok": False, "clean": True, "clean_shutdown": True},
            {"ok": 1, "clean": True, "clean_shutdown": True},
            {"ok": "true", "clean": True, "clean_shutdown": True},
            {"ok": None, "clean": True, "clean_shutdown": True},
        ]
        for value in cases:
            with self.subTest(value=value):
                self.assertFalse(ft._cleanup_dict_is_strictly_clean(value))

    def _finalize(self, report, cleanup=None, eligible_param=None) -> tuple[int, dict[str, object]]:
        buf = io.StringIO()
        args = argparse.Namespace(json_out=None)
        with contextlib.redirect_stdout(buf):
            rc = ft._finalize_report(report, args, "bench", [], cleanup,
                                     acceptance_eligible=eligible_param)
        return rc, json.loads(buf.getvalue())

    def test_finalize_missing_cleanup_evidence_fails_closed(self) -> None:
        report: dict[str, object] = {"command": "bench", "acceptance_eligible": True}
        rc, out = self._finalize(report, cleanup=None)
        self.assertEqual(rc, 2)
        _assert_failure_report_schema(out)
        self.assertIsInstance(out["cleanup"], dict)
        self.assertFalse(out["cleanup"]["clean_shutdown"])

    def test_finalize_top_clean_shutdown_without_cleanup_dict_fails(self) -> None:
        report = {"command": "bench", "clean_shutdown": True, "acceptance_eligible": True}
        rc, out = self._finalize(report, cleanup=None)
        self.assertEqual(rc, 2)
        self.assertFalse(out["clean_shutdown"])
        self.assertFalse(out["passed"])

    def test_finalize_cleanup_dict_without_top_clean_shutdown_fails(self) -> None:
        report = {"command": "bench",
                  "cleanup": {"ok": True, "clean": True, "clean_shutdown": True}}
        rc, out = self._finalize(report, cleanup=None)
        self.assertEqual(rc, 2)
        self.assertFalse(out["clean_shutdown"])
        self.assertFalse(out["passed"])

    def test_finalize_cleanup_only_ok_true_fails(self) -> None:
        report = {"command": "bench", "cleanup": {"ok": True}}
        rc, out = self._finalize(report, cleanup=None)
        self.assertEqual(rc, 2)
        self.assertFalse(out["clean_shutdown"])

    def test_finalize_strict_cleanup_and_top_clean_shutdown_passes(self) -> None:
        report = {
            "command": "bench",
            "cleanup": {"ok": True, "clean": True, "clean_shutdown": True},
            "clean_shutdown": True,
            "acceptance_eligible": True,
        }
        rc, out = self._finalize(report, cleanup=None)
        self.assertEqual(rc, 0)
        self.assertTrue(out["clean_shutdown"])
        self.assertTrue(out["acceptance_eligible"])
        self.assertTrue(out["passed"])

    def test_finalize_eligibility_non_bool_false(self) -> None:
        for bad in ("false", 1, None):
            with self.subTest(bad=bad):
                report = {
                    "command": "bench",
                    "cleanup": {"ok": True, "clean": True, "clean_shutdown": True},
                    "clean_shutdown": True,
                    "acceptance_eligible": bad,
                }
                rc, out = self._finalize(report, cleanup=None)
                self.assertEqual(rc, 2, f"eligibility={bad!r} 必须失败")
                self.assertFalse(out["acceptance_eligible"])
                self.assertTrue(any("acceptance_eligible" in r for r in out["failure_reasons"]))

    def test_failures_schema_cleanup_contradictory_fails(self) -> None:
        bad_cleanup_cases = [
            {"ok": False, "clean": False, "clean_shutdown": True},
            {"ok": True, "clean": False, "clean_shutdown": True},
            {"ok": True, "clean": True, "clean_shutdown": 1},
            {"ok": True, "clean": True, "clean_shutdown": "true"},
        ]
        for cleanup in bad_cleanup_cases:
            with self.subTest(cleanup=cleanup):
                fake = {"passed": True, "failure_reasons": [], "cleanup": cleanup}
                with mock.patch.object(ft, "_scenario_pause_resume", return_value=fake), \
                     mock.patch.object(ft, "_scenario_network_drop", return_value=fake), \
                     mock.patch.object(ft, "_scenario_gnss_cache_no_fix_stale",
                                       return_value={"passed": True, "failure_reasons": []}):
                    rc, report = _run_cmd(ft.cmd_failures, _failures_args())
                self.assertEqual(rc, 2)
                self.assertFalse(report["clean_shutdown"])
                self.assertTrue(any("cleanup" in r for r in report["failure_reasons"]))

    def test_failures_schema_passed_string_and_reasons_fail(self) -> None:
        good = {"passed": True, "failure_reasons": [],
                "cleanup": {"ok": True, "clean": True, "clean_shutdown": True}}
        # passed="true"（字符串）
        bad_passed = {"passed": "true", "failure_reasons": [],
                      "cleanup": {"ok": True, "clean": True, "clean_shutdown": True}}
        with mock.patch.object(ft, "_scenario_pause_resume", return_value=bad_passed), \
             mock.patch.object(ft, "_scenario_network_drop", return_value=good), \
             mock.patch.object(ft, "_scenario_gnss_cache_no_fix_stale",
                               return_value={"passed": True, "failure_reasons": []}):
            rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        self.assertEqual(rc, 2)
        self.assertTrue(any("pause_resume" in r and "passed" in r for r in report["failure_reasons"]))
        # passed=True 但 failure_reasons 非空
        bad_reasons = {"passed": True, "failure_reasons": ["boom"],
                       "cleanup": {"ok": True, "clean": True, "clean_shutdown": True}}
        with mock.patch.object(ft, "_scenario_pause_resume", return_value=bad_reasons), \
             mock.patch.object(ft, "_scenario_network_drop", return_value=good), \
             mock.patch.object(ft, "_scenario_gnss_cache_no_fix_stale",
                               return_value={"passed": True, "failure_reasons": []}):
            rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        self.assertEqual(rc, 2)
        self.assertTrue(any("pause_resume" in r and "failure_reasons" in r for r in report["failure_reasons"]))

    def test_failures_schema_result_non_dict_fails(self) -> None:
        good = {"passed": True, "failure_reasons": [],
                "cleanup": {"ok": True, "clean": True, "clean_shutdown": True}}
        with mock.patch.object(ft, "_scenario_pause_resume", return_value="not-a-dict"), \
             mock.patch.object(ft, "_scenario_network_drop", return_value=good), \
             mock.patch.object(ft, "_scenario_gnss_cache_no_fix_stale",
                               return_value={"passed": True, "failure_reasons": []}):
            rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        self.assertEqual(rc, 2)
        self.assertTrue(any("pause_resume" in r and "dict" in r for r in report["failure_reasons"]))

    def test_failures_schema_all_correct_passes(self) -> None:
        good = {"passed": True, "failure_reasons": [],
                "cleanup": {"ok": True, "clean": True, "clean_shutdown": True, "details": []}}
        with mock.patch.object(ft, "_scenario_pause_resume", return_value=good), \
             mock.patch.object(ft, "_scenario_network_drop", return_value=good), \
             mock.patch.object(ft, "_scenario_gnss_cache_no_fix_stale",
                               return_value={"passed": True, "failure_reasons": []}):
            rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        self.assertEqual(rc, 0, f"全 schema 正确应通过: {report['failure_reasons']}")
        self.assertTrue(report["clean_shutdown"])
        self.assertTrue(report["passed"])


class FailureSchemaConsistencyTests(unittest.TestCase):
    """§一/§三/§四/§五：统一失败 cleanup 三字段、序列化失败 schema、baseline 顶层 schema。"""

    def _finalize_raw(self, report, cleanup=None, eligible_param=None):
        buf = io.StringIO()
        args = argparse.Namespace(json_out=None)
        with contextlib.redirect_stdout(buf):
            rc = ft._finalize_report(report, args, "bench", [], cleanup,
                                     acceptance_eligible=eligible_param)
        return rc, json.loads(buf.getvalue())

    def test_serialization_failure_schema_all_cases(self) -> None:
        self_ref: dict[str, object] = {}
        self_ref["self"] = self_ref
        cases = [
            {"bad": object()},
            {"bad": lambda: None},
            {"bad": float("nan")},
            {"bad": float("inf")},
            {"bad": self_ref},
            {"bad": {(1, 2): "tuple-key"}},
            {"bad": {float("nan"): "nan-key"}},
        ]
        for payload in cases:
            with self.subTest(payload=type(payload["bad"]).__name__):
                report = {"command": "bench", **payload}
                rc, out = self._finalize_raw(report, cleanup=None)
                self.assertEqual(rc, 2)
                _assert_failure_report_schema(out)
                self.assertIsInstance(out["cleanup"]["details"], list)
                json.dumps(out, allow_nan=False)  # 可再次严格序列化
                self.assertFalse(out["cleanup"]["ok"])
                self.assertFalse(out["cleanup"]["clean"])
                self.assertFalse(out["cleanup"]["clean_shutdown"])

    def test_baseline_top_schema_rejections(self) -> None:
        opts = _opts()
        report = _make_report_like()
        cases = [
            (_make_valid_baseline(clean_shutdown=None), "clean_shutdown=None"),
            (_make_valid_baseline(clean_shutdown=False), "clean_shutdown=False"),
            (_make_valid_baseline(clean_shutdown=0), "clean_shutdown=0"),
            (_make_valid_baseline(clean_shutdown=1), "clean_shutdown=1"),
            (_make_valid_baseline(clean_shutdown="true"), "clean_shutdown=true-str"),
            # nested cleanup 全 true 但顶层 false
            (_make_valid_baseline(cleanup={"ok": True, "clean": True, "clean_shutdown": True},
                                  clean_shutdown=False), "nested true + top false"),
            # failure_reasons 非严格空 list
            (_make_valid_baseline(failure_reasons=""), "reasons=''"),
            (_make_valid_baseline(failure_reasons=None), "reasons=None"),
            (_make_valid_baseline(failure_reasons=()), "reasons=()"),
            (_make_valid_baseline(failure_reasons={}), "reasons={}"),
            # command
            (dict(_make_valid_baseline(), **{"command": "longrun"}), "command=longrun"),
            (dict(_make_valid_baseline(), **{"command": None}), "command=None"),
            # passed 非精确 bool true
            (_make_valid_baseline(passed=1), "passed=1"),
            (_make_valid_baseline(passed="true"), "passed=true-str"),
            # 顶层 clean_shutdown 缺失
            ({k: v for k, v in _make_valid_baseline().items() if k != "clean_shutdown"},
             "clean_shutdown missing"),
        ]
        for baseline, label in cases:
            with self.subTest(label=label):
                reason = ft._baseline_invalid_reason(baseline, opts, report)
                self.assertIsNotNone(reason, f"{label} 必须被拒绝")

    def test_baseline_top_schema_valid_passes(self) -> None:
        baseline = _make_valid_baseline(mode="simulate", sweeps_per_s=30.0)
        reason = ft._baseline_invalid_reason(baseline, _opts(), _make_report_like())
        self.assertIsNone(reason)

    def test_failures_exception_fallback_cleanup_schema(self) -> None:
        # 场景抛异常 -> fallback cleanup 三字段全 false 且可再次严格序列化。
        def _boom(opts):
            raise RuntimeError("scenario boom")

        with mock.patch.object(ft, "_scenario_pause_resume", side_effect=_boom), \
             mock.patch.object(ft, "_scenario_network_drop", side_effect=_boom), \
             mock.patch.object(ft, "_scenario_gnss_cache_no_fix_stale",
                               return_value={"passed": True, "failure_reasons": []}):
            rc, report = _run_cmd(ft.cmd_failures, _failures_args())
        self.assertEqual(rc, 2)
        self.assertFalse(report["passed"])
        self.assertFalse(report["clean_shutdown"])
        self.assertIsInstance(report["cleanup"], dict)
        for key in ("ok", "clean", "clean_shutdown"):
            self.assertIs(type(report["cleanup"][key]), bool)
            self.assertIs(report["cleanup"][key], False)
        json.dumps(report, allow_nan=False)


class BaselineNumericSchemaTests(unittest.TestCase):
    """§一/§二/§三/§四：baseline 数值 schema 严格校验。"""

    def _reject(self, baseline, label) -> None:
        reason = ft._baseline_invalid_reason(baseline, _opts(), _make_report_like())
        self.assertIsNotNone(reason, f"{label} 必须被拒绝")
        return reason

    def test_baseline_sweeps_per_s_numeric_rejection(self) -> None:
        for bad in (True, False, float("nan"), float("inf"), float("-inf"),
                    "1.0", 0, -5):
            with self.subTest(bad=bad):
                self._reject(_make_valid_baseline(sweeps_per_s=bad), f"rate={bad!r}")

    def test_baseline_elapsed_s_numeric_rejection(self) -> None:
        for bad in (True, False, float("nan"), float("inf"), float("-inf"),
                    "60", 10.0):
            with self.subTest(bad=bad):
                self._reject(_make_valid_baseline(elapsed_s=bad), f"elapsed={bad!r}")

    def test_baseline_count_fields_exact_int(self) -> None:
        for key in ("processed", "saved", "enqueued"):
            for bad in (False, 0.0, "0"):
                with self.subTest(key=key, bad=bad):
                    self._reject(_make_valid_baseline(**{key: bad}), f"{key}={bad!r}")
        # 精确整数 0 通过。
        reason = ft._baseline_invalid_reason(
            _make_valid_baseline(processed=0, saved=0, enqueued=0),
            _opts(), _make_report_like(),
        )
        self.assertIsNone(reason)

    def test_baseline_config_int_fields_rejection(self) -> None:
        cfg_ok = _make_valid_baseline()["config"]
        for key in ("points", "ifbw_hz"):
            for bad in (True, False, 201.0, "201", -201):
                cfg = dict(cfg_ok, **{key: bad})
                with self.subTest(key=key, bad=bad):
                    self._reject(_make_valid_baseline(config=cfg), f"{key}={bad!r}")
        for key in ("dwell_us", "count"):
            for bad in (True, False, 1.0, "1", -1):
                cfg = dict(cfg_ok, **{key: bad})
                with self.subTest(key=key, bad=bad):
                    self._reject(_make_valid_baseline(config=cfg), f"{key}={bad!r}")
        # config 实数：bool/字符串/非有限拒绝。
        for key in ("start_mhz", "stop_mhz", "power_dbm"):
            for bad in (True, "400", float("nan"), float("inf")):
                cfg = dict(cfg_ok, **{key: bad})
                with self.subTest(key=key, bad=bad):
                    self._reject(_make_valid_baseline(config=cfg), f"{key}={bad!r}")

    def test_baseline_json_file_nonfinite_constant(self) -> None:
        import tempfile
        from pathlib import Path
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as fh:
            json.dump({"sweeps_per_s": float("nan")}, fh)  # allow_nan 默认 -> 写 NaN
            path = fh.name
        try:
            opts = _opts(baseline_json=Path(path))
            report = _make_report_like()
            failures: list[str] = []
            ft._apply_baseline_check(report, opts, failures)  # 不得抛未处理异常
            self.assertIn("未评估", str(report["baseline_90_percent_met"]))
            self.assertIn("非标准", str(report["baseline_90_percent_met"]))
            # baseline 未达标（未评估），不会用于 eligibility。
            self.assertIsNot(report["baseline_90_percent_met"], True)
        finally:
            Path(path).unlink(missing_ok=True)

    def test_schedule_helper_numeric_rejection(self) -> None:
        base = {"schedule_error_ms": {"count": 5, "p95": 10.0}}
        for label, rate in (("rate=True", True), ("rate=False", False),
                            ("rate=nan", float("nan")), ("rate='50'", "50"), ("rate=0", 0)):
            with self.subTest(label=label):
                ok, _p, _m = ft._schedule_error_within_sweep_period(base, {"sweeps_per_s": rate})
                self.assertFalse(ok, label)
        for label, p95 in (("p95=True", True), ("p95=False", False),
                           ("p95=nan", float("nan")), ("p95=inf", float("inf")),
                           ("p95='5'", "5"), ("p95=-1", -1)):
            with self.subTest(label=label):
                ok, _p, _m = ft._schedule_error_within_sweep_period(
                    {"schedule_error_ms": {"count": 5, "p95": p95}}, {"sweeps_per_s": 100.0}
                )
                self.assertFalse(ok, label)
        for label, count in (("count=True", True), ("count=False", False),
                             ("count=2.0", 2.0), ("count='5'", "5"), ("count=1", 1)):
            with self.subTest(label=label):
                ok, _p, _m = ft._schedule_error_within_sweep_period(
                    {"schedule_error_ms": {"count": count, "p95": 10.0}},
                    {"sweeps_per_s": 100.0},
                )
                self.assertFalse(ok, label)
        # 合法值通过。
        ok, period, _m = ft._schedule_error_within_sweep_period(
            {"schedule_error_ms": {"count": 5, "p95": 10.0}}, {"sweeps_per_s": 100.0}
        )
        self.assertTrue(ok)
        self.assertEqual(period, 10.0)

    def test_baseline_numeric_valid_tool_generated_passes(self) -> None:
        baseline = _make_valid_baseline(mode="simulate", sweeps_per_s=30.0, elapsed_s=60.0)
        reason = ft._baseline_invalid_reason(baseline, _opts(), _make_report_like())
        self.assertIsNone(reason)


class BaselineContainerSchemaTests(unittest.TestCase):
    """§一/§二/§四：baseline 容器 schema 严格校验 + _baseline_invalid_reason total function。"""

    def _reject(self, baseline, report=None, simulate=True, label="") -> Optional[str]:
        opts = _opts(simulate=simulate, hardware=not simulate)
        report = report if report is not None else _make_report_like(mode="simulate" if simulate else "hardware")
        reason = ft._baseline_invalid_reason(baseline, opts, report)
        self.assertIsNotNone(reason, f"{label} 必须被拒绝")
        return reason

    def test_config_container_rejection(self) -> None:
        for bad in ("bad", [], 1, None):
            with self.subTest(bad=bad):
                self._reject(_make_valid_baseline(config=bad), label=f"config={bad!r}")

    def test_layers_container_rejection(self) -> None:
        for bad in ("usb", 1, {"usb": True}, None, [], ["usb", "proc"], ("usb",)):
            with self.subTest(bad=bad):
                self._reject(_make_valid_baseline(layers=bad), label=f"layers={bad!r}")

    def test_hardware_device_container_rejection(self) -> None:
        for bad in ("bad", [], None):
            with self.subTest(bad=bad):
                self._reject(
                    _make_valid_baseline(mode="hardware", device=bad),
                    simulate=False, label=f"device={bad!r}",
                )
        # 缺 firmware/protocol/hardware_version
        for key in ("firmware", "protocol", "hardware_version"):
            dev = {"firmware": "1.2.3", "protocol": 14, "hardware_version": 5}
            del dev[key]
            with self.subTest(missing=key):
                self._reject(
                    _make_valid_baseline(mode="hardware", device=dev),
                    simulate=False, label=f"device missing {key}",
                )
        # 类型与当前 device 不一致（bool vs int 绕过）
        mismatched = _make_valid_baseline(mode="hardware", device={
            "firmware": "1.2.3", "protocol": True, "hardware_version": 5,
        })
        self._reject(mismatched, simulate=False, label="protocol bool vs int")

    def test_report_config_device_broken(self) -> None:
        baseline = _make_valid_baseline(mode="simulate")
        # report.config 损坏 -> 明确错误，不抛异常。
        reason = ft._baseline_invalid_reason(
            baseline, _opts(), {"mode": "simulate", "config": "bad", "device": None}
        )
        self.assertIsNotNone(reason)
        self.assertIn("config", reason)
        # report.device 损坏在 hardware 模式 -> 明确错误，不抛异常。
        hw = _make_valid_baseline(mode="hardware")
        reason = ft._baseline_invalid_reason(
            hw, _opts(simulate=False, hardware=True),
            {"mode": "hardware", "config": _make_report_like()["config"], "device": []},
        )
        self.assertIsNotNone(reason)
        self.assertIn("device", reason)

    def test_apply_baseline_check_broken_structures_no_crash(self) -> None:
        import tempfile
        from pathlib import Path
        broken_cases = [
            dict(_make_valid_baseline(), **{"config": "bad"}),
            dict(_make_valid_baseline(), **{"layers": "usb"}),
            dict(_make_valid_baseline(), **{"cleanup": {"ok": True, "clean": True}}),
            dict(_make_valid_baseline(), **{"sweeps_per_s": float("nan")}),
            dict(_make_valid_baseline(), **{"elapsed_s": "60"}),
        ]
        for broken in broken_cases:
            with self.subTest(broken=broken):
                with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as fh:
                    json.dump(broken, fh)
                    path = fh.name
                try:
                    opts = _opts(baseline_json=Path(path))
                    report = _make_report_like()
                    failures: list[str] = []
                    ft._apply_baseline_check(report, opts, failures)  # 不抛异常
                    self.assertIn("未评估", str(report["baseline_90_percent_met"]))
                    self.assertNotIn("baseline_sweeps_per_s", report)
                    self.assertIsNot(report["baseline_90_percent_met"], True)
                finally:
                    Path(path).unlink(missing_ok=True)

    def test_valid_baseline_container_passes(self) -> None:
        baseline = _make_valid_baseline(mode="simulate", sweeps_per_s=30.0)
        reason = ft._baseline_invalid_reason(baseline, _opts(), _make_report_like())
        self.assertIsNone(reason)

    def test_schedule_helper_container_fail_closed(self) -> None:
        for label, sched in (("None", None), ("str", "bad"), ("list", [])):
            with self.subTest(label=label):
                ok, _p, _m = ft._schedule_error_within_sweep_period(sched, {"sweeps_per_s": 100.0})
                self.assertFalse(ok, label)
        for label, err in (("str", "bad"), ("list", [])):
            with self.subTest(label=label):
                ok, _p, _m = ft._schedule_error_within_sweep_period(
                    {"schedule_error_ms": err}, {"sweeps_per_s": 100.0}
                )
                self.assertFalse(ok, label)
        ok, _p, _m = ft._schedule_error_within_sweep_period(
            {"schedule_error_ms": {"count": 5, "p95": 10.0}}, "bad"
        )
        self.assertFalse(ok)


class HugeJsonNumberTests(unittest.TestCase):
    """§一：超大 JSON 整数（10**10000）不得抛 OverflowError，一律 fail-closed。"""

    def _reason(self, baseline, simulate=True):
        opts = _opts(simulate=simulate, hardware=not simulate)
        return ft._baseline_invalid_reason(
            baseline, opts, _make_report_like(mode="simulate" if simulate else "hardware"))

    def test_huge_sweeps_per_s_rejected(self) -> None:
        reason = self._reason(_make_valid_baseline(sweeps_per_s=10**10000))
        self.assertIsNotNone(reason)
        self.assertIn("sweeps_per_s", reason)

    def test_huge_elapsed_s_rejected(self) -> None:
        reason = self._reason(_make_valid_baseline(elapsed_s=10**10000))
        self.assertIsNotNone(reason)

    def test_huge_config_fields_rejected(self) -> None:
        for key in ("start_mhz", "stop_mhz", "power_dbm", "sweep_timeout_s"):
            with self.subTest(key=key):
                cfg = dict(_make_valid_baseline()["config"])
                cfg[key] = 10**10000
                reason = self._reason(_make_valid_baseline(config=cfg))
                self.assertIsNotNone(reason, f"config.{key} 超大整数必须拒绝")
                self.assertIn(key, reason)

    def test_huge_int_direct_helper_no_escape(self) -> None:
        self.assertFalse(ft._is_finite_real_not_bool(10**10000))
        self.assertFalse(ft._is_finite_positive_real_not_bool(10**10000))

    def test_schedule_helper_huge_rate_and_p95(self) -> None:
        ok, _p, _m = ft._schedule_error_within_sweep_period(
            {"schedule_error_ms": {"count": 5, "p95": 10.0}}, {"sweeps_per_s": 10**10000})
        self.assertFalse(ok)
        ok, _p, _m = ft._schedule_error_within_sweep_period(
            {"schedule_error_ms": {"count": 5, "p95": 10**10000}}, {"sweeps_per_s": 100.0})
        self.assertFalse(ok)

    def test_apply_baseline_check_huge_int_no_crash(self) -> None:
        import sys
        import tempfile
        from pathlib import Path
        has_limit = (
            hasattr(sys, "get_int_max_str_digits") and hasattr(sys, "set_int_max_str_digits")
        )
        before = sys.get_int_max_str_digits() if has_limit else None
        path: Optional[str] = None
        try:
            if has_limit:
                # 仅在生成/读取超大整数 JSON 所需的最小范围内置 0（Python 3.11+）。
                sys.set_int_max_str_digits(0)
            broken = _make_valid_baseline(sweeps_per_s=10**10000)
            with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as fh:
                json.dump(broken, fh)
                path = fh.name
            opts = _opts(baseline_json=Path(path))
            report = _make_report_like()
            failures: list[str] = []
            ft._apply_baseline_check(report, opts, failures)  # 不抛异常
            self.assertIn("未评估", str(report["baseline_90_percent_met"]))
            self.assertNotIn("baseline_sweeps_per_s", report)
        finally:
            # 无条件恢复 digit limit（json.dump/读取/断言/校验抛错也要恢复），并删除临时文件。
            if has_limit and before is not None:
                sys.set_int_max_str_digits(before)
            if path is not None:
                Path(path).unlink(missing_ok=True)
        # 退出临时范围后 digit limit 必须等于进入前的值（Python 3.11+）。
        if has_limit and before is not None:
            self.assertEqual(sys.get_int_max_str_digits(), before)

    def test_huge_int_test_leaves_digit_limit_unchanged(self) -> None:
        """回归：运行超大整数测试后全局 int_max_str_digits 必须未变化（不依赖测试顺序）。"""
        import sys
        if not hasattr(sys, "get_int_max_str_digits"):
            self.skipTest("Python < 3.11 无 int_max_str_digits API")
        before = sys.get_int_max_str_digits()
        # 直接运行会临时置 0 的测试方法；无论其结果如何，都必须恢复。
        self.test_apply_baseline_check_huge_int_no_crash()
        self.assertEqual(sys.get_int_max_str_digits(), before)


class DeviceSchemaTests(unittest.TestCase):
    """§二：hardware device 缺字段绕过修复与类型严格校验。"""

    def _hardware_reason(self, baseline, report) -> Optional[str]:
        return ft._baseline_invalid_reason(
            baseline, _opts(simulate=False, hardware=True), report)

    def test_report_device_missing_fields_rejected(self) -> None:
        for key in ("firmware", "protocol", "hardware_version"):
            with self.subTest(missing=key):
                rdev = {"firmware": "1.2.3", "protocol": 14, "hardware_version": 5}
                del rdev[key]
                report = _make_report_like(mode="hardware")
                report["device"] = rdev
                reason = self._hardware_reason(_make_valid_baseline(mode="hardware"), report)
                self.assertIsNotNone(reason, f"report.device 缺 {key} 必须拒绝")
                self.assertIn(key, reason)

    def test_baseline_none_and_report_missing_bypass_closed(self) -> None:
        # 旧代码用 .get()：baseline 字段 None 与 report 缺字段（.get 返回 None）相等。
        bdev = {"firmware": None, "protocol": 14, "hardware_version": 5}
        rdev = {"protocol": 14, "hardware_version": 5}  # 缺 firmware
        report = _make_report_like(mode="hardware")
        report["device"] = rdev
        reason = self._hardware_reason(_make_valid_baseline(mode="hardware", device=bdev), report)
        self.assertIsNotNone(reason)
        self.assertIn("firmware", reason)

    def test_device_illegal_types_rejected(self) -> None:
        report = _make_report_like(mode="hardware")
        bad_devices = [
            {"firmware": 123, "protocol": 14, "hardware_version": 5},
            {"firmware": "1.2.3", "protocol": True, "hardware_version": 5},
            {"firmware": "1.2.3", "protocol": "14", "hardware_version": 5},
            {"firmware": "1.2.3", "protocol": 14, "hardware_version": True},
            {"firmware": "1.2.3", "protocol": 14, "hardware_version": "5"},
            {"firmware": "1.2.3", "protocol": None, "hardware_version": 5},
        ]
        for bdev in bad_devices:
            with self.subTest(bdev=bdev):
                reason = self._hardware_reason(
                    _make_valid_baseline(mode="hardware", device=bdev), report)
                self.assertIsNotNone(reason, f"非法 device 必须拒绝: {bdev}")

    def test_valid_hardware_device_passes(self) -> None:
        reason = self._hardware_reason(
            _make_valid_baseline(mode="hardware"), _make_report_like(mode="hardware"))
        self.assertIsNone(reason)


class ZzzResidualThreadTests(unittest.TestCase):
    """测试套件末尾确认无 harness/生产线程残留（按名称运行顺序最后执行）。"""

    def test_no_field_test_threads_residual(self) -> None:
        names = [t.name for t in threading.enumerate()]
        leftover = [
            n for n in names
            if n.startswith("field-test") or n == "uav-gpr-remote-sender"
        ]
        self.assertEqual(leftover, [], f"存在残留线程: {leftover}")


if __name__ == "__main__":
    unittest.main()
