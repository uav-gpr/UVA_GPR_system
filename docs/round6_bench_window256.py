"""B-scan 显示窗口扩容 64→256 性能基准与功能契约实测（任务 t1 证据脚本）。

同机同进程对比 64 档（运行时回拨 BSCAN_WINDOW_TRACES=64）与 256 档：

  1) 纯 NumPy 共享路径（bscan_view）：
     - bscan_window_from_rows（Flat Filter 关）；
     - apply_flat_filter_windowed（Flat Filter 开，窗口 31）；
  2) offscreen 真实 GUI：GroundStationWindow.refresh_bscan_views
     （HH+VV 双通道、flat 开/关），计时含 processEvents 渲染；
  3) 300 道历史功能契约断言：显示窗=最后 256 道、滑块范围、标签、
     NaN 填充、setRect/xRange。

预算：25 道/秒 → 每道刷新 p95 ≤ 40ms；超预算必须如实报告，不得静默接受。

运行（仓库根目录）：
    $env:QT_QPA_PLATFORM='offscreen'; $env:QTWEBENGINE_DISABLE_SANDBOX='1'
    .venv312\\Scripts\\python.exe docs\\bench_window256.py

输出 JSON（stdout）。本脚本只读生产代码 + 运行时回拨常量，不修改任何文件。
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import time
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

import bscan_view

N_SAMPLES = 1475      # 用户实际典型显示矩阵行数（1475×64 → 1475×256）
N_HISTORY = 1000      # 历史总道数（超过 256，取末窗语义）
REPEAT_PURE = 50
REPEAT_GUI = 30
FLAT_WINDOW = 31


def _stats(samples_ms: list[float]) -> dict[str, float]:
    arr = np.asarray(samples_ms, dtype=float)
    return {
        "p50_ms": round(float(np.percentile(arr, 50)), 3),
        "p95_ms": round(float(np.percentile(arr, 95)), 3),
        "max_ms": round(float(arr.max()), 3),
    }


def _timed(func, repeat: int) -> dict[str, float]:
    best = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        func()
        best.append((time.perf_counter() - t0) * 1000.0)
    return _stats(best)


def make_rows(n_traces: int, n_samples: int, seed: int = 42) -> list[np.ndarray]:
    rng = np.random.default_rng(seed)
    return [rng.standard_normal(n_samples) for _ in range(n_traces)]


def bench_pure(width: int) -> dict[str, object]:
    """纯 NumPy 共享路径：末窗（follow-latest）构造与窗口化滤波。"""
    bscan_view.BSCAN_WINDOW_TRACES = width
    from gpr_processing import flat_reflection_filter

    rows = make_rows(N_HISTORY, N_SAMPLES)
    start = bscan_view.bscan_window_max_start(N_HISTORY)
    results: dict[str, object] = {
        "window": width,
        "last_window_start": start,
        "flat_off_bscan_window_from_rows_ms": _timed(
            lambda: bscan_view.bscan_window_from_rows(rows, start), REPEAT_PURE
        ),
        "flat_on_apply_flat_filter_windowed_ms": _timed(
            lambda: bscan_view.apply_flat_filter_windowed(
                rows, start, FLAT_WINDOW, filter_func=flat_reflection_filter
            ),
            REPEAT_PURE,
        ),
    }
    return results


def bench_pure_worst_flat(width: int, flat_window: int, start_override: int | None = None) -> dict[str, object]:
    """纯 NumPy 最坏邻域用例：subset 可达 256 + (flat_window-1) 列。"""
    bscan_view.BSCAN_WINDOW_TRACES = width
    from gpr_processing import flat_reflection_filter

    rows = make_rows(N_HISTORY, N_SAMPLES)
    start = start_override or bscan_view.bscan_window_max_start(N_HISTORY)
    subset_lo, subset_hi = bscan_view.flat_filter_window_bounds(N_HISTORY, start, flat_window)
    return {
        "window": width,
        "flat_window": flat_window,
        "start": start,
        "subset_cols": subset_hi - subset_lo + 1,
        "flat_on_apply_flat_filter_windowed_ms": _timed(
            lambda: bscan_view.apply_flat_filter_windowed(
                rows, start, flat_window, filter_func=flat_reflection_filter
            ),
            REPEAT_PURE,
        ),
    }


def _make_gui():
    from PySide6 import QtCore, QtWidgets
    import uav_gpr_ground_station as ground

    class DummyReceiver(QtCore.QObject):
        status = QtCore.Signal(str)
        command_status = QtCore.Signal(str)
        trace_received = QtCore.Signal(dict, bytes, str)

        def __init__(self, *_args) -> None:
            super().__init__()

        def start(self) -> None:
            pass

        def stop(self) -> None:
            pass

        def wait(self, _timeout: int) -> bool:
            return True

        def send_command(self, _action: str, _params=None) -> None:
            pass

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    temp = tempfile.TemporaryDirectory()
    with mock.patch.object(ground, "ReceiverThread", DummyReceiver):
        window = ground.GroundStationWindow("127.0.0.1", 0, Path(temp.name))
    return app, window, temp, ground


def bench_gui(width: int, flat_enabled: bool, flat_window: int = FLAT_WINDOW) -> dict[str, object]:
    """offscreen 真实 GUI 刷新：refresh_bscan_views + processEvents。"""
    app, window, temp, ground = _make_gui()
    bscan_view.BSCAN_WINDOW_TRACES = width
    ground.BSCAN_WINDOW_TRACES = width  # setRect/xRange 引用 ground 命名空间副本
    try:
        rows = make_rows(N_HISTORY, N_SAMPLES)
        window.bscan_rows = {"S11": rows, "S22": [r + 1.0 for r in rows]}
        window.bscan_rows_complex = {
            "S11": [r.astype(np.complex128) for r in rows],
            "S22": [(r + 1.0).astype(np.complex128) for r in rows],
        }
        window.bscan_time_ns = np.linspace(0.0, 40.0, N_SAMPLES)
        window.apply_bscan_settings(ground.BscanUiSettings(
            colormap="gray",
            aspect_ratio=2.0,
            flat_filter_enabled=flat_enabled,
            flat_window_traces=flat_window,
        ))

        def one_refresh() -> None:
            window.refresh_bscan_views()
            app.processEvents()

        return {
            "window": width,
            "flat_enabled": flat_enabled,
            "flat_window": flat_window,
            "history": N_HISTORY,
            "n_samples": N_SAMPLES,
            "refresh_plus_paint_ms": _timed(one_refresh, REPEAT_GUI),
        }
    finally:
        window.close()
        app.processEvents()
        temp.cleanup()
        bscan_view.BSCAN_WINDOW_TRACES = 256
        ground.BSCAN_WINDOW_TRACES = 256


def functional_checks_300() -> dict[str, object]:
    """offscreen 功能契约：300 道历史 → 显示窗=最后 256 道（扩容验收③）。

    300 道通过生产路径 ``_append_bscan_trace`` 逐道追加（follow-latest 语义），
    不直接注入 rows——直接注入会保留"历史视图"起点（窗口停在 1..256 也是
    正确行为，但不是流式采集时的用户可见行为）。
    """
    app, window, temp, _ground = _make_gui()
    checks: list[dict[str, object]] = []

    def check(name: str, ok: bool, detail: str) -> None:
        checks.append({"check": name, "ok": bool(ok), "detail": detail})

    try:
        rows = make_rows(300, N_SAMPLES, seed=7)
        time_ns = np.linspace(0.0, 40.0, N_SAMPLES)
        traces = {"S11": rows[0], "S22": rows[0] + 1.0}
        for index, row in enumerate(rows):
            traces = {"S11": row, "S22": row + 1.0}
            ok_append = window._append_bscan_trace(time_ns, traces)
            if not ok_append:
                check(f"第{index + 1}道追加成功", False, "_append_bscan_trace 返回 False")
                break
        window.refresh_bscan_views()
        app.processEvents()

        matrix, _levels = window._bscan_display_window("S11")
        finite_cols = int(np.sum(np.any(np.isfinite(matrix), axis=0)))
        check("300道逐道追加_历史=300", window.bscan_trace_count() == 300,
              f"count={window.bscan_trace_count()}")
        check("300道_显示窗起点=45(follow-latest)", window.bscan_window_start == 45,
              f"start={window.bscan_window_start}")
        check("300道_滑块最大=45", window.bscan_window_slider.maximum() == 45,
              f"slider_max={window.bscan_window_slider.maximum()}")
        check("300道_标签=显示 45–300 / 共 300 道",
              window.bscan_window_label.text() == "显示 45–300 / 共 300 道",
              f"label={window.bscan_window_label.text()!r}")
        check("300道_图像宽=256", window.bscan_images["S11"].width() == 256,
              f"width={window.bscan_images['S11'].width()}")
        check("300道_窗口矩阵形状=[1475,256]", matrix.shape == (N_SAMPLES, 256),
              f"shape={matrix.shape}")
        check("300道_256列全部有限(无NaN)", finite_cols == 256, f"finite_cols={finite_cols}")
        check("300道_首列=第45道", np.array_equal(matrix[:, 0], rows[44]), "matrix[:,0]==rows[44]")
        check("300道_末列=第300道", np.array_equal(matrix[:, 255], rows[299]),
              "matrix[:,255]==rows[299]")
        xr = window.bscan_plots["S11"].getViewBox().viewRange()[0]
        check("300道_xRange=[44.5,300.5]",
              abs(xr[0] - 44.5) < 1e-9 and abs(xr[1] - 300.5) < 1e-9, f"xRange={xr}")

        # 不足一窗：100 道 → 右侧 NaN 填充（列 100..255）。
        window.clear_bscan()
        rows100 = rows[:100]
        for row in rows100:
            window._append_bscan_trace(time_ns, {"S11": row, "S22": row + 1.0})
        window.refresh_bscan_views()
        app.processEvents()
        matrix100, _ = window._bscan_display_window("S11")
        finite100 = int(np.sum(np.any(np.isfinite(matrix100), axis=0)))
        check("100道_有限列=100", finite100 == 100, f"finite_cols={finite100}")
        check("100道_列100..255全NaN", bool(np.all(np.isnan(matrix100[:, 100:]))),
              f"nan_cols={matrix100.shape[1] - finite100}")
        check("100道_图像宽=256", window.bscan_images["S11"].width() == 256,
              f"width={window.bscan_images['S11'].width()}")
        check("100道_标签=显示 1–100 / 共 100 道",
              window.bscan_window_label.text() == "显示 1–100 / 共 100 道",
              f"label={window.bscan_window_label.text()!r}")

        # 回到 300 道历史，拖滑块到合法中段起点 20 与最大起点 45。
        window.clear_bscan()
        for row in rows:
            window._append_bscan_trace(time_ns, {"S11": row, "S22": row + 1.0})
        window.refresh_bscan_views()
        app.processEvents()
        window.bscan_window_slider.setValue(20)
        app.processEvents()
        matrix20, _ = window._bscan_display_window("S11")
        finite20 = int(np.sum(np.any(np.isfinite(matrix20), axis=0)))
        check("拖到20_标签=显示 20–275 / 共 300 道",
              window.bscan_window_label.text() == "显示 20–275 / 共 300 道",
              f"label={window.bscan_window_label.text()!r}")
        check("拖到20_256列全部有限", finite20 == 256, f"finite_cols={finite20}")
        check("拖到20_首列=第20道", np.array_equal(matrix20[:, 0], rows[19]),
              "matrix20[:,0]==rows[19]")
        check("拖到20_末列=第275道", np.array_equal(matrix20[:, 255], rows[274]),
              "matrix20[:,255]==rows[274]")
        window.bscan_window_slider.setValue(45)
        app.processEvents()
        matrix45, _ = window._bscan_display_window("S11")
        check("拖到45_标签=显示 45–300 / 共 300 道",
              window.bscan_window_label.text() == "显示 45–300 / 共 300 道",
              f"label={window.bscan_window_label.text()!r}")
        check("拖到45_首列=第45道", np.array_equal(matrix45[:, 0], rows[44]),
              "matrix45[:,0]==rows[44]")
        # 越界起点 50（> max_start=45）→ 收敛到 45（不越界）。
        window.bscan_window_slider.setValue(50)
        app.processEvents()
        check("越界50_收敛到45",
              window.bscan_window_start == 45
              and window.bscan_window_label.text() == "显示 45–300 / 共 300 道",
              f"start={window.bscan_window_start}, label={window.bscan_window_label.text()!r}")
    finally:
        window.close()
        app.processEvents()
        temp.cleanup()

    all_ok = all(item["ok"] for item in checks)
    return {"all_ok": all_ok, "checks": checks}


def main() -> None:
    results: dict[str, object] = {}
    results["配置"] = {
        "n_samples": N_SAMPLES,
        "n_history": N_HISTORY,
        "flat_window": FLAT_WINDOW,
        "repeat_pure": REPEAT_PURE,
        "repeat_gui": REPEAT_GUI,
        "预算_ms_per_refresh": 40.0,
    }
    # 1) 纯 NumPy：先 64（运行时回拨）再 256（还原）。
    results["纯NumPy_64档"] = bench_pure(64)
    results["纯NumPy_256档"] = bench_pure(256)
    # 1b) 纯 NumPy 最坏邻域：flat 窗 127 → 末窗 subset 319 列（边缘截断）、
    #     中段窗口 subset 382 列（目标书 ≈382 列口径的最大邻域）。
    results["纯NumPy_256档_flat窗口127_末窗"] = bench_pure_worst_flat(256, 127)
    results["纯NumPy_256档_flat窗口127_中段382列"] = bench_pure_worst_flat(256, 127, 400)
    # 2) offscreen GUI：64 与 256 各测 flat 开/关；256 加测最坏 flat 窗 127。
    results["GUI_64档_flat关"] = bench_gui(64, False)
    results["GUI_64档_flat开"] = bench_gui(64, True)
    results["GUI_256档_flat关"] = bench_gui(256, False)
    results["GUI_256档_flat开"] = bench_gui(256, True)
    results["GUI_256档_flat开_窗口127"] = bench_gui(256, True, 127)
    # 3) 300 道历史功能契约。
    results["功能契约_300道"] = functional_checks_300()
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
