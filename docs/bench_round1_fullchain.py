"""第1轮复审：真实软件全链路基准（fake 设备 → 空中端 → 地面端显示）。

链路（全部为真实生产代码，仅 USB 层用 fake transport 模拟 25Hz 设备）：

  fake backend（设备按 25Hz 产生完整 sweep，201 频点 S11+S22）
    → ContinuousAcquisitionWorker（USB 生产者线程 + 无损队列）
    → ProcessingConsumer（校准/BPF/IFFT/Dewow 处理、本地保存）
    → RemoteSender（TCP 逐条 ACK）
    → ReceiverThread（地面接收/去重/落盘/ACK）
    → GroundTraceDecoder（后台解码线程）
    → GroundDisplayFIFO（显式 FIFO）
    → Qt 主线程逐条提交（校验+追加+setImage）

计数全部来自真实运行对象（scheduler/队列/磁盘/信号统计），绝不用常量
冒充。端到端计时点在**成功完成 setImage 提交后**（display FIFO pop 的
对象携带 air 侧 sweep 完成时刻）。

运行：QT_QPA_PLATFORM=offscreen python docs/bench_round1_fullchain.py
"""

from __future__ import annotations

import os
import sys
import tempfile
import time
from collections import deque
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tests"))

import numpy as np
from PySide6 import QtWidgets

import pyqtgraph as pg

import uav_gpr_qt as air
from pipeline_metrics import PipelineMetrics
import uav_gpr_ground_station as ground

# 复用测试基建：fake transport/backend（按需产生完整 sweep）。
import test_uav_gpr_qt_continuous as testbed  # noqa: E402

N_FREQ = 201          # 设备每 sweep 频点数
N_TIME_TARGET = 2048  # IFFT 时域点（SweepConfig.ifft_points）
N_TRACES = 500
RATE_HZ = 25.0
SWEEP_PERIOD = 1.0 / RATE_HZ


class _RateLimitedTransport(testbed._RepeatingSweepTransport):
    """25Hz 限速的 fake 设备：每条完整 sweep 至少间隔 SWEEP_PERIOD。

    read() 在点 0 前阻塞到下一个 25Hz tick（模拟真实扫频时长）。
    """

    def __init__(self, n_points: int) -> None:
        super().__init__(n_points)
        self._next_sweep_at = time.monotonic()

    def read(self, max_length: int, timeout_ms: int) -> bytes:
        if self._point == 0 and not self._open_sweep:
            # 新 sweep 开始前按 25Hz 节流。
            now = time.monotonic()
            if now < self._next_sweep_at:
                time.sleep(self._next_sweep_at - now)
            self._next_sweep_at += SWEEP_PERIOD
            self._open_sweep = True
        try:
            return super().read(max_length, timeout_ms)
        finally:
            if self._point == 0:
                self._open_sweep = False

    _open_sweep = False


def main() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pg.setConfigOptions(antialias=True, background="w", foreground="k")

    air_temp = tempfile.TemporaryDirectory()
    ground_temp = tempfile.TemporaryDirectory()
    air_out = Path(air_temp.name)
    ground_out = Path(ground_temp.name)

    # ---- 性能统计（真实阶段接入：空中端 + 地面端共用同一实例）----
    metrics = PipelineMetrics(max_samples=8192)
    air.set_air_metrics(metrics)
    window = ground.GroundStationWindow("127.0.0.1", 0, ground_out, start_receiver=True)
    # 替换窗口/接收器/解码器的统计为基准实例（统一口径）。
    window.metrics = metrics
    window.receiver.metrics = metrics
    ground.set_ground_decoder_metrics(metrics)

    # 端到端计时：sweep 完成时刻（worker 线程记录）→ setImage 提交完成。
    sweep_done_at: dict[int, float] = {}
    display_done_at: dict[int, float] = {}
    # 提交完成后（setImage 成功）记录该 Trace 的完成时刻；trace_index 在
    # 生产链路严格连续（1..N），按提交顺序对应 FIFO committed 计数。
    state = {"committed": 0}

    def instrumented_commit() -> None:
        depth_before = window.display_fifo.depth()
        alive = window._process_one_display_item()
        if alive and depth_before > 0:
            state["committed"] += 1
            display_done_at[state["committed"]] = time.perf_counter()
        if window.display_fifo.depth() > 0:
            from PySide6 import QtCore

            QtCore.QTimer.singleShot(0, instrumented_commit)
        else:
            window._display_timer_active = False
            window._reconcile_stop_state()

    window._commit_one_display = instrumented_commit  # type: ignore[method-assign]

    # sweep 完成时刻：包装 backend.acquire_one。
    transport = _RateLimitedTransport(N_FREQ)
    backend = testbed.PersistentLibreVnaBackend(transport=transport)

    original_acquire = backend.acquire_one

    def instrumented_acquire():
        sweep = original_acquire()
        sweep_done_at[len(sweep_done_at) + 1] = time.perf_counter()
        return sweep

    backend.acquire_one = instrumented_acquire  # type: ignore[method-assign]

    # ---- 地面端接收端口就绪 ----
    deadline = time.monotonic() + 5.0
    port = None
    while time.monotonic() < deadline:
        app.processEvents()
        port = window.receiver.bound_port
        if port:
            break
    assert port, "receiver 未完成端口绑定"

    # ---- 空中端：sender + worker + consumer ----
    statuses: list[str] = []
    sender = air.RemoteSender("127.0.0.1", port, "benchrun", statuses.append, None)
    sender.start()
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and sender._sock is None:
        app.processEvents()
    assert sender._sock is not None, "sender 未连接"

    cfg = air.SweepConfig(
        start_mhz=400,
        stop_mhz=4000,
        points=N_FREQ,
        ifbw_hz=10000,
        ifft_points=N_TIME_TARGET,
        time_window_ns=60.0,
    )
    worker = air.ContinuousAcquisitionWorker(
        cfg=cfg,
        interval_s=0.0,  # 每个完整 Sweep（默认无间隔模式）
        out_dir=air_out,
        count=N_TRACES,
        gnss_port="",
        baud=115200,
        calibration_path="",
        session_calibration=None,
        session_calibration_p2=None,
        through_measurement=None,
        background_frequency_hz=None,
        background=None,
        autosave=True,  # 本地保存计入对账
        remote_sender=sender,
        backend=backend,
    )
    finished = False

    import threading

    done_event = threading.Event()
    worker.finished_run.connect(
        lambda: done_event.set(), air.QtCore.Qt.ConnectionType.DirectConnection
    )
    worker.start()

    start_wall = time.monotonic()
    # 主循环：泵 Qt 事件（解码回调 + 显示提交）。
    while not done_event.is_set() or window.display_fifo.depth() > 0:
        app.processEvents()
        if time.monotonic() - start_wall > 120:
            print("TIMEOUT: 全链路未在 120s 内完成", file=sys.stderr)
            break
    # 等待 sender 排空（全部 ACK）：stop 返回值必须检查（复审修复）。
    sender_drained = sender.stop(10000, drain=True)
    if not sender_drained:
        print(f"FAIL: sender.stop(drain=True) 未能排空（outstanding={sender.outstanding_count()}）",
              file=sys.stderr)

    # 等待地面端屏障归零（复审：decoder queued/in_flight/in_callback=0、
    # FIFO depth=0、停止屏障完成）。
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        app.processEvents()
        stats = window.decoder.statistics()
        fifo = window.display_fifo.statistics()
        if (
            stats["queued"] == 0 and stats["in_flight"] == 0 and stats["in_callback"] == 0
            and fifo["depth"] == 0 and fifo["unresolved"] == 0
        ):
            break
        time.sleep(0.01)
    app.processEvents()

    # ---- 真实计数（读取运行对象，不用常量）----
    scheduler_stats = worker.scheduler.statistics
    consumer = worker.consumer
    local_saved = len(list(air_out.rglob("trace_*.npz")))
    ground_saved = len(list(ground_out.rglob("trace_*.npz")))
    # ACK 精确计数：地面 receiver 的 _seen_traces（每条首次保存一条）。
    remote_acked = len(window.receiver._seen_traces)
    decoder_stats = window.decoder.statistics()
    decoded = decoder_stats["decoded_success"]  # 真正解码成功数（复审：非 completed）
    decoded_rejected = decoder_stats["rejected"]
    fifo_final = window.display_fifo.statistics()
    displayed = fifo_final["total_committed"]  # 真正成功 setImage 的提交计数
    display_rejected = fifo_final["total_rejected"]
    display_failed = fifo_final["total_failed"]

    # ---- 延迟统计 ----
    e2e = []
    for index, t_done in display_done_at.items():
        t_sweep = sweep_done_at.get(index)
        if t_sweep is not None:
            e2e.append((t_done - t_sweep) * 1000.0)

    def stats(values: list[float]) -> dict:
        if not values:
            return {"count": 0, "p50_ms": None, "p95_ms": None, "max_ms": None}
        ordered = sorted(values)
        return {
            "count": len(ordered),
            "p50_ms": round(float(np.percentile(ordered, 50)), 2),
            "p95_ms": round(float(np.percentile(ordered, 95)), 2),
            "max_ms": round(float(max(ordered)), 2),
        }

    stage_stats = metrics._stages.get("ground_displayed")
    qt_commit = [v * 1000.0 for v in stage_stats.samples] if stage_stats else []
    e2e_stat = stats(e2e)
    qt_stat = stats(qt_commit)
    report = {
        "配置": {
            "频点": N_FREQ,
            "时域点": N_TIME_TARGET,
            "通道": "S11+S22",
            "速率": f"{RATE_HZ} sweep/s",
            "条数": N_TRACES,
            "模式": "每个完整Sweep（interval_s=0）",
        },
        "真实计数": {
            "selected(scheduler)": scheduler_stats["selected_sweeps"],
            "selected(worker入队)": worker._selected_count,
            "processed(consumer)": consumer.processed_count if consumer else None,
            "local_saved(磁盘NPZ)": local_saved,
            "remote_acked(地面保存数)": remote_acked,
            "ground_saved(磁盘NPZ)": ground_saved,
            "decoded_success(decoder)": decoded,
            "decoded_rejected(decoder)": decoded_rejected,
            "displayed(FIFO committed，真正setImage成功)": displayed,
            "display_rejected": display_rejected,
            "display_failed": display_failed,
            "sender_outstanding(应=0)": sender.outstanding_count(),
            "sender_drained": sender_drained,
        },
        "端到端_sweep完成→setImage提交_ms": e2e_stat,
        "Qt主线程提交_ms": qt_stat,
        "阶段统计": {
            name: metrics.stage_report(name)
            for name in (
                "sweep_completed", "processing_started", "processing_completed",
                "local_save_completed", "remote_enqueued", "remote_send_started",
                "remote_ack_received", "ground_received", "ground_saved",
                "ground_decoded", "ground_display_enqueued", "ground_displayed",
            )
            if metrics.stage_report(name)["count"]
        },
        "队列": {
            "processing_queue": worker.processing_queue.statistics(),
            "display_fifo": fifo_final,
            "decoder": decoder_stats,
        },
        "总耗时_s": round(time.monotonic() - start_wall, 1),
    }
    import json

    print(json.dumps(report, ensure_ascii=False, indent=2))

    # ---- 验收判定（复审：失败条件必须实际值可见、非零 exit code）----
    failures: list[str] = []
    counters = report["真实计数"]
    for name, expected_count in (
        ("selected(scheduler)", N_TRACES),
        ("selected(worker入队)", N_TRACES),
        ("processed(consumer)", N_TRACES),
        ("local_saved(磁盘NPZ)", N_TRACES),
        ("remote_acked(地面保存数)", N_TRACES),
        ("ground_saved(磁盘NPZ)", N_TRACES),
        ("decoded_success(decoder)", N_TRACES),
        ("displayed(FIFO committed，真正setImage成功)", N_TRACES),
    ):
        actual = counters[name]
        if actual != expected_count:
            failures.append(f"{name}={actual} != {expected_count}")
    if decoded_rejected != 0:
        failures.append(f"decoded_rejected={decoded_rejected} != 0")
    if display_rejected != 0:
        failures.append(f"display_rejected={display_rejected} != 0")
    if display_failed != 0:
        failures.append(f"display_failed={display_failed} != 0")
    if counters["sender_outstanding(应=0)"] != 0:
        failures.append(f"sender_outstanding={counters['sender_outstanding(应=0)']} != 0")
    if not sender_drained:
        failures.append("sender.stop(drain=True) 未排空")
    if worker.terminal_state != "drained":
        failures.append(f"worker.terminal_state={worker.terminal_state} != drained")
    qt_p95 = qt_stat.get("p95_ms")
    if qt_p95 is None or qt_p95 > 20.0:
        failures.append(f"Qt提交p95={qt_p95}ms > 20ms")
    e2e_p95 = e2e_stat.get("p95_ms")
    if e2e_p95 is None or e2e_p95 > 200.0:
        failures.append(f"端到端p95={e2e_p95}ms > 200ms")

    if failures:
        print("\n=== 验收失败（实际值） ===", file=sys.stderr)
        for item in failures:
            print(f"FAIL: {item}", file=sys.stderr)
        exit_code = 1
    else:
        print("\n=== 验收通过：全部计数 500/500，Qt p95 与端到端 p95 达标 ===")
        exit_code = 0

    # ---- 第三次定向修复：所有阶段样本数、underflow、callback_consumed 必须可见 ----
    extra_failures: list[str] = []
    ds = decoder_stats
    expected = N_TRACES
    if ds["submitted"] != expected:
        extra_failures.append(f"decoder.submitted={ds['submitted']} != {expected}")
    if ds["decoded_success"] != expected:
        extra_failures.append(f"decoder.decoded_success={ds['decoded_success']} != {expected}")
    if ds["callback_consumed"] != expected:
        extra_failures.append(
            f"decoder.callback_consumed={ds['callback_consumed']} != {expected}"
        )
    if ds["queued"] != 0 or ds["in_flight"] != 0 or ds["in_callback"] != 0:
        extra_failures.append(
            "decoder 未归零: queued={}, in_flight={}, in_callback={}".format(
                ds["queued"], ds["in_flight"], ds["in_callback"]
            )
        )
    if ds["callback_underflow"] != 0:
        extra_failures.append(
            f"decoder.callback_underflow={ds['callback_underflow']} != 0"
        )
    if ds["in_flight_underflow"] != 0:
        extra_failures.append(
            f"decoder.in_flight_underflow={ds['in_flight_underflow']} != 0"
        )
    # FIFO 计数（来自真实 fifo_final）
    if fifo_final["total_enqueued"] != expected:
        extra_failures.append(
            f"fifo.total_enqueued={fifo_final['total_enqueued']} != {expected}"
        )
    if fifo_final["total_dequeued"] != expected:
        extra_failures.append(
            f"fifo.total_dequeued={fifo_final['total_dequeued']} != {expected}"
        )
    if fifo_final["total_committed"] != expected:
        extra_failures.append(
            f"fifo.total_committed={fifo_final['total_committed']} != {expected}"
        )
    if fifo_final["total_rejected"] != 0 or fifo_final["total_failed"] != 0:
        extra_failures.append(
            f"fifo rejected/failed={fifo_final['total_rejected']}/{fifo_final['total_failed']} != 0/0"
        )
    if fifo_final["unresolved"] != 0:
        extra_failures.append(f"fifo.unresolved={fifo_final['unresolved']} != 0")
    if fifo_final["depth"] != 0:
        extra_failures.append(f"fifo.depth={fifo_final['depth']} != 0")

    # 阶段样本数
    def _sample_count(name: str) -> int:
        rep = metrics.stage_report(name)
        return int(rep.get("count", 0))

    if _sample_count("processing_started") < expected:
        extra_failures.append(
            f"processing_started samples={_sample_count('processing_started')} < {expected}"
        )
    # remote_send_started 必须恰好 == N_TRACES（acquisition_status 不应混入）
    if _sample_count("remote_send_started") != expected:
        extra_failures.append(
            f"remote_send_started samples={_sample_count('remote_send_started')} != {expected}（必须排除 status 消息）"
        )
    # ground_display_enqueued 必须 >= expected（跨线程真实延迟）
    if _sample_count("ground_display_enqueued") < expected:
        extra_failures.append(
            f"ground_display_enqueued samples={_sample_count('ground_display_enqueued')} < {expected}"
        )
    if _sample_count("ground_display_enqueue_cost") < expected:
        extra_failures.append(
            f"ground_display_enqueue_cost samples={_sample_count('ground_display_enqueue_cost')} < {expected}"
        )

    # 任务级 delta（基线起）必须等于 N_TRACES，无 carry-over
    # 重新设置基线后再次扫描——本轮没有"中途停止再开始"，基线 = 0；总累计应
    # 等于 N_TRACES。
    if extra_failures:
        print("\n=== 第三次定向修复断言失败 ===", file=sys.stderr)
        for item in extra_failures:
            print(f"FAIL-3R: {item}", file=sys.stderr)
        exit_code = 1

    # 清理。
    worker.wait(5000)
    window.decoder.stop(2000)
    window.close()
    app.processEvents()
    air_temp.cleanup()
    ground_temp.cleanup()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
