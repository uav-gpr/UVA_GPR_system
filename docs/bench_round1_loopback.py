"""第1轮性能验收：同机 loopback 端到端基准（无头 offscreen）。

链路：RemoteSender(空中端发送线程) → TCP localhost → ReceiverThread(地面)
→ 后台解码线程 → 显示 FIFO → Qt 主线程逐条提交（QTimer 0ms）。

配置：201 频点 / 2048 时域点 / S11+S22 双通道 / 25 Trace/s × 20 秒
（共 500 条）。

统计：
- 数量对账：sent == accepted(新增保存) == decoded == displayed；
- ground_decoded / ground_displayed 阶段 p50/p95/max（进程内 perf_counter）；
- 端到端（air enqueue → ground 显示提交）p50/p95/max：同进程同一时钟，
  真实无线跨机场景需两端系统时钟同步后才能复用该口径；
- 显示 FIFO 深度/峰值。

运行：QT_QPA_PLATFORM=offscreen python docs/bench_round1_loopback.py
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from PySide6 import QtWidgets

import uav_gpr_ground_station as ground
import uav_gpr_qt as air


N_FREQ = 201
N_TIME = 2048
N_TRACES = 500
RATE_HZ = 25.0


def make_payload(index: int) -> tuple[bytes, dict]:
    rng = np.random.default_rng(index)
    time_ns = np.linspace(0.0, 60.0, N_TIME)
    raw = {
        "S11": rng.standard_normal(N_FREQ) + 1j * rng.standard_normal(N_FREQ),
        "S22": rng.standard_normal(N_FREQ) + 1j * rng.standard_normal(N_FREQ),
    }
    display = {k: v for k, v in raw.items()}
    traces = {
        "S11": rng.standard_normal(N_TIME),
        "S22": rng.standard_normal(N_TIME),
    }
    cfg = air.SweepConfig()
    fix = air.GgaFix()
    sender = air.RemoteSender("127.0.0.1", 0, "benchrun", lambda text: None, None)
    # 只借用 payload 组装逻辑，不启动线程。
    # enqueue_trace 会把 (header, payload) 放进 queue；这里直接读队列。
    sender.enqueue_trace(
        index,
        cfg,
        np.linspace(400e6, 4000e6, N_FREQ),
        raw,
        display,
        time_ns,
        traces,
        fix,
        channels=("S11", "S22"),
    )
    header, payload = sender.queue.get_nowait()
    sender.stop(drain=False)
    return payload, header


def main() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    import pyqtgraph as pg

    pg.setConfigOptions(antialias=True, background="w", foreground="k")

    temp = tempfile.TemporaryDirectory()
    out_dir = Path(temp.name)

    # ---- 地面端 ----
    window = ground.GroundStationWindow("127.0.0.1", 0, out_dir, start_receiver=True)
    deadline = time.monotonic() + 5.0
    port = None
    while time.monotonic() < deadline:
        app.processEvents()
        port = window.receiver.bound_port
        if port:
            break
    assert port, "receiver 未完成端口绑定"
    assert window.receiver.isRunning(), "receiver 未启动"

    # 端到端计时：air 入队时刻（同进程，时钟一致）。
    air_times: dict[object, float] = {}
    e2e_latencies: list[float] = []

    # 包装 pop：拿到 trace_index 计算 e2e。
    original_pop = window.display_fifo.pop

    def instrumented_pop():
        item = original_pop()
        if item is not None:
            index = getattr(item, "trace_index", None)
            enqueued = air_times.get(index)
            if enqueued is not None:
                e2e_latencies.append((time.perf_counter() - enqueued) * 1000.0)
        return item

    window.display_fifo.pop = instrumented_pop  # type: ignore[method-assign]

    # ---- 空中端 sender ----
    sender = air.RemoteSender("127.0.0.1", port, "benchrun", lambda text: None, None)
    sender.start()
    deadline = time.monotonic() + 10.0
    while sender._sock is None and time.monotonic() < deadline:
        app.processEvents()
    assert sender._sock is not None, "sender 未连接"

    start = time.monotonic()
    period = 1.0 / RATE_HZ
    for index in range(1, N_TRACES + 1):
        target = start + (index - 1) * period
        payload, header = make_payload(index)
        # 重用 enqueue_trace 会重新打包；直接把预生成的 header/payload 入队。
        now = time.monotonic()
        if now < target:
            time.sleep(target - now)
        air_times[header.get("trace_index")] = time.perf_counter()
        sender.queue.put((header, payload))
        # 泵事件循环让地面端持续解码/提交。
        app.processEvents()

    # 等待全部显示提交完成。
    t_end = time.monotonic() + 60.0
    while time.monotonic() < t_end:
        app.processEvents()
        if (
            window.display_fifo.total_committed >= N_TRACES
            and window.display_fifo.depth() == 0
        ):
            break
        time.sleep(0.005)

    elapsed = time.monotonic() - start

    decoded_report = window.metrics.stage_report("ground_decoded")
    displayed_report = window.metrics.stage_report("ground_displayed")
    fifo_stats = window.display_fifo.statistics()
    receiver = window.receiver

    def stats(values: list[float]) -> dict:
        if not values:
            return {"count": 0}
        ordered = sorted(values)
        return {
            "count": len(ordered),
            "p50_ms": round(float(np.percentile(ordered, 50)), 3),
            "p95_ms": round(float(np.percentile(ordered, 95)), 3),
            "max_ms": round(float(max(ordered)), 3),
        }

    report = {
        "配置": {
            "频点": N_FREQ,
            "时域点": N_TIME,
            "通道": "S11+S22",
            "目标速率": f"{RATE_HZ} Trace/s",
            "条数": N_TRACES,
            "总耗时_s": round(elapsed, 2),
        },
        "数量对账": {
            "sent(air enqueued)": N_TRACES,
            "ground_accepted(新增保存)": len(receiver._seen_traces),
            "ground_decoded": window.display_fifo.total_enqueued,
            "ground_displayed": window.display_fifo.total_committed,
        },
        "ground_decoded耗时_ms": stats(
            _metric_samples(window, "ground_decoded")
        ),
        "ground_displayed(Qt提交)_ms": stats(
            _metric_samples(window, "ground_displayed")
        ),
        "端到端_air入队→地面显示_ms": stats(e2e_latencies),
        "显示FIFO": fifo_stats,
        "metrics_raw_decoded": decoded_report,
        "metrics_raw_displayed": displayed_report,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))

    sender.stop(5000, drain=False)
    window.decoder.stop(2000)
    window.close()
    temp.cleanup()
    app.processEvents()


def _metric_samples(window, stage: str) -> list[float]:
    """从 PipelineMetrics 有限样本窗口提取原始秒样本（×1000 转 ms）。"""
    stats_obj = window.metrics._stages.get(stage)
    return [value * 1000.0 for value in stats_obj.samples] if stats_obj else []


if __name__ == "__main__":
    main()
