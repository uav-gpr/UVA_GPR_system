# -*- coding: utf-8 -*-
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import time
import tempfile
import shutil
import numpy as np

import uav_gpr_qt as air

def run_bench():
    points_count = 201
    n_traces = 100
    cfg = air.SweepConfig(
        start_mhz=400.0, stop_mhz=4000.0, points=points_count,
        ifbw_hz=10000, power_dbm=0.0, dwell_us=10.0,
        time_window_ns=40.0, dewow_window_ns=4.0, ifft_points=2048,
    )
    freq = np.linspace(400e6, 4000e6, points_count, dtype=np.float64)
    raw = {"S11": np.random.randn(points_count) + 1j * np.random.randn(points_count), "S22": np.random.randn(points_count) + 1j * np.random.randn(points_count)}
    disp = {"S11": raw["S11"].copy(), "S22": raw["S22"].copy()}
    time_ns = np.linspace(0.0, 40.0, 1475, dtype=np.float64)
    traces = {"S11": np.random.randn(1475).astype(np.complex128), "S22": np.random.randn(1475).astype(np.complex128)}
    fix = air.GgaFix(raw="$GNGGA,...", utc="123456.00", latitude=39.9, longitude=116.4, fix_quality=1, satellites=12, altitude_m=50.0)

    # 1. 仅 CSV 单独耗时
    # 2. 仅 NPZ + JSONL 耗时
    # 3. CSV + NPZ + JSONL 耗时
    
    tmp = Path(tempfile.mkdtemp(prefix="bench_isolated_"))
    try:
        # A: 仅 NPZ + JSON
        dur_npz_only = []
        for i in range(1, n_traces + 1):
            t0 = time.perf_counter()
            air.save_trace(tmp, i, cfg, freq, raw, disp, time_ns, traces, fix, channels=("S11", "S22"), save_csv=False)
            dur_npz_only.append((time.perf_counter() - t0) * 1000.0)
            
        # 清理
        for f in tmp.iterdir(): f.unlink()
        
        # B: 完整保存 (含 CSV)
        dur_full = []
        for i in range(1, n_traces + 1):
            t0 = time.perf_counter()
            air.save_trace(tmp, i, cfg, freq, raw, disp, time_ns, traces, fix, channels=("S11", "S22"), save_csv=True)
            dur_full.append((time.perf_counter() - t0) * 1000.0)
            
        csv_overhead = np.array(dur_full) - np.array(dur_npz_only)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print("RESULT_START")
    print(f"NPZ_JSON_AVG={np.mean(dur_npz_only):.2f}")
    print(f"NPZ_JSON_P50={np.median(dur_npz_only):.2f}")
    print(f"NPZ_JSON_P95={np.percentile(dur_npz_only, 95):.2f}")
    print(f"FULL_AVG={np.mean(dur_full):.2f}")
    print(f"FULL_P50={np.median(dur_full):.2f}")
    print(f"FULL_P95={np.percentile(dur_full, 95):.2f}")
    print(f"CSV_ALONE_ESTIMATE={np.mean(dur_full) - np.mean(dur_npz_only):.2f}")
    print("RESULT_END")

if __name__ == '__main__':
    run_bench()