"""第1轮检查点A/B：B-scan 显示路径基准（无头、纯 NumPy，无 GPU 绘制）。

对比：
- 旧路径：np.column_stack(全部历史) → 全历史 Flat Filter → 截取 64 道；
- 新路径：bscan_window_from_rows / apply_flat_filter_windowed（窗口化）。

配置覆盖用户当前典型参数：201 频点 / 2048 时域点 / 64 道窗口 /
1000 道完整历史 / Flat Filter 开与关。

运行：QT_QPA_PLATFORM=offscreen python docs/bench_round1_baseline.py
输出为 JSON（stdout），供完成报告引用。
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from bscan_view import (
    BSCAN_WINDOW_TRACES,
    apply_flat_filter_windowed,
    bscan_window_from_rows,
    select_bscan_window,
)
from gpr_processing import flat_reflection_filter


def make_rows(n_traces: int, n_samples: int) -> list[np.ndarray]:
    rng = np.random.default_rng(42)
    return [rng.standard_normal(n_samples) for _ in range(n_traces)]


def time_call(func, repeat: int = 20):
    best = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        func()
        best.append((time.perf_counter() - t0) * 1000.0)
    return {
        "p50_ms": round(float(np.percentile(best, 50)), 3),
        "p95_ms": round(float(np.percentile(best, 95)), 3),
        "max_ms": round(float(max(best)), 3),
    }


def main() -> None:
    results: dict[str, object] = {}
    for n_samples, label in ((2048, "2048时域点"), (64, "小规模64点")):
        for total in (64, 100, 1000):
            rows = make_rows(total, n_samples)
            # 旧路径：全历史矩阵 + 全历史滤波 + 截取。
            def old_path(rows=rows):
                matrix = np.column_stack([np.asarray(row, dtype=float) for row in rows])
                if matrix.shape[1] >= 31:
                    matrix = flat_reflection_filter(matrix, window_size=31)
                return select_bscan_window(matrix, 1)

            # 新路径：只构造可见窗口（+滤波邻域）。
            def new_path_flat(rows=rows):
                return apply_flat_filter_windowed(rows, 1, 31)

            def new_path_noflat(rows=rows):
                return bscan_window_from_rows(rows, 1)

            key = f"{label}_历史{total}道"
            results[key] = {
                "旧路径_flat开_ms": time_call(old_path),
                "新路径_flat开_ms": time_call(new_path_flat),
                "旧路径_flat关_ms": time_call(old_path),
                "新路径_flat关_ms": time_call(new_path_noflat),
            }
    results["窗口宽度"] = BSCAN_WINDOW_TRACES
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
