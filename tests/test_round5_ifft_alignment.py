"""第5轮专项测试：时域算法对齐钢筋仪（ifft, dewow, flat_reflection）。

覆盖要求：
1. _next_power_of_two 与 _compute_ifft_grid 网格参数校验（与钢筋仪对照逐字一致）；
2. DC 补零与相位斜坡 exp(2j*pi*offset*t) 修正频偏；
3. 复数 dewow 线性独立性（real/imag 分部等价）；
4. 复数 flat_reflection_filter 线性独立性（real/imag 分部等价）；
5. frequency_to_time_trace 全程复数链与截取点数一致性；
6. 双通道 S11 / S22 统一时域网格与时间轴；
7. NPZ 格式向后兼容性（旧 key 保持 float64，只增 trace_*_complex）；
8. UI 绝无 abs / 包络控制项，送显严格取 .real；
9. 真实飞行测试数据（remote_runs）兼容性验证。
"""

from __future__ import annotations

import math
import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from gpr_processing import (
    _compute_ifft_grid,
    _next_power_of_two,
    apply_frequency_bpf,
    dewow,
    flat_reflection_filter,
    frequency_to_time_trace,
)
import air_core as air
import uav_gpr_ground_station as gs


class TestNextPowerOfTwo(unittest.TestCase):
    def test_powers_of_two(self) -> None:
        self.assertEqual(_next_power_of_two(1), 1)
        self.assertEqual(_next_power_of_two(2), 2)
        self.assertEqual(_next_power_of_two(3), 4)
        self.assertEqual(_next_power_of_two(4), 4)
        self.assertEqual(_next_power_of_two(7), 8)
        self.assertEqual(_next_power_of_two(223), 256)
        self.assertEqual(_next_power_of_two(256), 256)
        self.assertEqual(_next_power_of_two(257), 512)

    def test_rejects_non_positive(self) -> None:
        with self.assertRaises(ValueError):
            _next_power_of_two(0)
        with self.assertRaises(ValueError):
            _next_power_of_two(-5)


class TestIfftGridComputation(unittest.TestCase):
    def setUp(self) -> None:
        # UAV-GPR 标准扫频配置：400MHz - 4GHz, 201 频点
        self.freq = np.linspace(400e6, 4000e6, 201)

    def test_uav_gpr_standard_grid_values(self) -> None:
        df, fft_size, first_bin, offset, time_s = _compute_ifft_grid(self.freq, oversampling=16)
        self.assertAlmostEqual(df, 18e6)
        self.assertEqual(first_bin, 22)
        self.assertAlmostEqual(offset, 4e6)
        # padded_len = 22 + 201 = 223 -> next_pow2(223) = 256 -> 256 * 16 = 4096
        self.assertEqual(fft_size, 4096)
        expected_dt = 1.0 / (4096 * 18e6)
        self.assertAlmostEqual(time_s[1] - time_s[0], expected_dt)
        self.assertEqual(time_s.size, 4096)
        self.assertAlmostEqual(time_s[0], 0.0)

    def test_grid_rejects_negative_frequency(self) -> None:
        with self.assertRaises(ValueError):
            _compute_ifft_grid(np.array([-100e6, 0.0, 100e6]))

    def test_grid_rejects_non_uniform_frequency(self) -> None:
        with self.assertRaises(ValueError):
            _compute_ifft_grid(np.array([100e6, 200e6, 400e6]))


class TestComplexDewowAndFlatFilter(unittest.TestCase):
    def test_complex_dewow_equals_independent_real_imag(self) -> None:
        rng = np.random.default_rng(42)
        real_part = rng.standard_normal(200)
        imag_part = rng.standard_normal(200)
        complex_signal = real_part + 1j * imag_part

        dt_s = 0.05e-9
        window_ns = 2.0

        filtered_complex = dewow(complex_signal, dt_s=dt_s, window_ns=window_ns)
        filtered_real = dewow(real_part, dt_s=dt_s, window_ns=window_ns)
        filtered_imag = dewow(imag_part, dt_s=dt_s, window_ns=window_ns)

        self.assertTrue(np.iscomplexobj(filtered_complex))
        np.testing.assert_allclose(filtered_complex.real, filtered_real, atol=1e-12)
        np.testing.assert_allclose(filtered_complex.imag, filtered_imag, atol=1e-12)

    def test_complex_flat_filter_equals_independent_real_imag(self) -> None:
        rng = np.random.default_rng(42)
        real_part = rng.standard_normal((100, 30))
        imag_part = rng.standard_normal((100, 30))
        complex_bscan = real_part + 1j * imag_part

        filtered_complex = flat_reflection_filter(complex_bscan, window_size=5)
        filtered_real = flat_reflection_filter(real_part, window_size=5)
        filtered_imag = flat_reflection_filter(imag_part, window_size=5)

        self.assertTrue(np.iscomplexobj(filtered_complex))
        np.testing.assert_allclose(filtered_complex.real, filtered_real, atol=1e-12)
        np.testing.assert_allclose(filtered_complex.imag, filtered_imag, atol=1e-12)


class TestFrequencyToTimeTraceAlignment(unittest.TestCase):
    def setUp(self) -> None:
        self.freq = np.linspace(400e6, 4000e6, 201)
        rng = np.random.default_rng(123)
        self.spec = rng.standard_normal(201) + 1j * rng.standard_normal(201)

    def test_aligned_mode_returns_complex_with_rebar_inspector_grid(self) -> None:
        t_ns, trace_c = frequency_to_time_trace(
            self.freq,
            self.spec,
            time_window_ns=40.0,
            apply_dewow=False,
            aligned=True,
            return_complex=True,
            oversampling=16,
        )
        self.assertTrue(np.iscomplexobj(trace_c))
        # 40 ns / dt_s = 40e-9 / (1 / (4096 * 18e6)) = 40e-9 * 73728e6 = 2949.12 -> round is 2949
        # oversampling=16 显式传入：与钢筋仪参照同参数对照（生产默认为 8，见下一条测试）。
        self.assertEqual(trace_c.size, 2949)
        self.assertEqual(t_ns.size, 2949)
        self.assertAlmostEqual(t_ns[0], 0.0)
        self.assertLessEqual(t_ns[-1], 40.0)

    def test_production_default_oversampling_8_restores_legacy_sample_count(self) -> None:
        # 生产默认 oversampling=8：padded_len=223 -> next_pow2=256 -> 256*8=2048，
        # dt 与重构前完全一致（27.13ps），40ns 窗口恢复重构前的每道采样数（1475），
        # 保住现场基准 50 道/秒的吞吐余量（物理不变：bin 对齐+相位校正仍在）。
        t_ns, trace_c = frequency_to_time_trace(
            self.freq,
            self.spec,
            time_window_ns=40.0,
            apply_dewow=False,
            aligned=True,
            return_complex=True,
        )
        self.assertEqual(trace_c.size, 1475)
        self.assertEqual(t_ns.size, 1475)
        expected_dt_ns = 1.0 / (2048 * 18e6) * 1e9
        self.assertAlmostEqual(t_ns[1] - t_ns[0], expected_dt_ns, places=6)

    def test_phase_ramp_restores_carrier_offset(self) -> None:
        # 单频载波测试：在起始频率 400 MHz 处设置冲击响应
        df = 18e6
        first_bin = 22
        offset = 400e6 - first_bin * df  # 4 MHz
        dt_s = 1.0 / (4096 * df)
        t_ns, trace_c = frequency_to_time_trace(
            self.freq,
            np.array([1.0 + 0j] + [0.0j] * 200),
            time_window_ns=10.0,
            apply_dewow=False,
            aligned=True,
            return_complex=True,
        )
        t_s = t_ns * 1e-9
        # 频点在 f0 = 400 MHz，IFFT 放置在 bin 22 (396 MHz)，经相位斜坡 exp(2j*pi*4MHz*t) 还原后
        # 其复指数频率应为 396MHz + 4MHz = 400 MHz
        expected_phase = 2 * np.pi * 400e6 * t_s
        actual_phase = np.unwrap(np.angle(trace_c))
        # 验证相位斜率（角频率 2*pi*f0）
        phase_slope = np.polyfit(t_s, actual_phase, 1)[0]
        self.assertAlmostEqual(phase_slope, 2 * np.pi * 400e6, delta=1e5)

    def test_dual_channel_s11_s22_identical_grid(self) -> None:
        cfg = air.SweepConfig(start_mhz=400, stop_mhz=4000, points=201, time_window_ns=40.0)
        raw_sparams = {
            "S11": self.spec,
            "S22": self.spec * 0.8,
        }
        display, time_ns, traces = air.process_trace_pipeline(self.freq, raw_sparams, cfg)
        self.assertIn("S11", traces)
        self.assertIn("S22", traces)
        self.assertEqual(traces["S11"].shape, traces["S22"].shape)
        self.assertEqual(traces["S11"].shape, time_ns.shape)
        self.assertTrue(np.iscomplexobj(traces["S11"]))
        self.assertTrue(np.iscomplexobj(traces["S22"]))


class TestNpzBackwardCompatibilityAndComplexKeys(unittest.TestCase):
    def setUp(self) -> None:
        self.out_dir = Path(tempfile.mkdtemp())
        self.freq = np.linspace(400e6, 4000e6, 201)
        rng = np.random.default_rng(77)
        self.spec11 = rng.standard_normal(201) + 1j * rng.standard_normal(201)
        self.spec22 = rng.standard_normal(201) + 1j * rng.standard_normal(201)
        self.cfg = air.SweepConfig(start_mhz=400, stop_mhz=4000, points=201, time_window_ns=40.0)
        self.raw = {"S11": self.spec11, "S22": self.spec22}
        self.display = {"S11": self.spec11, "S22": self.spec22}
        _, self.time_ns, self.traces = air.process_trace_pipeline(self.freq, self.raw, self.cfg)

    def test_save_trace_writes_float_and_complex_keys(self) -> None:
        csv_path = air.save_trace(
            self.out_dir,
            1,
            self.cfg,
            self.freq,
            self.raw,
            self.display,
            self.time_ns,
            self.traces,
            air.GgaFix(),
            channels=("S11", "S22"),
        )
        npz_path = csv_path.with_suffix(".npz")
        with np.load(npz_path, allow_pickle=False) as data:
            # 1. 旧 key 全部存在且为 float
            self.assertIn("trace_S11", data.files)
            self.assertIn("trace_S22", data.files)
            self.assertIn("trace", data.files)
            self.assertEqual(data["trace_S11"].dtype, np.float64)
            self.assertEqual(data["trace_S22"].dtype, np.float64)
            self.assertEqual(data["trace"].dtype, np.float64)

            # 2. 新增 complex key
            self.assertIn("trace_S11_complex", data.files)
            self.assertIn("trace_S22_complex", data.files)
            self.assertEqual(data["trace_S11_complex"].dtype, np.complex128)
            self.assertEqual(data["trace_S22_complex"].dtype, np.complex128)

            # 3. 语义严格对齐：旧 key == 新 key 的实部
            np.testing.assert_allclose(data["trace_S11"], data["trace_S11_complex"].real)
            np.testing.assert_allclose(data["trace_S22"], data["trace_S22_complex"].real)
            np.testing.assert_allclose(data["trace"], data["trace_S11_complex"].real)

    def test_trace_npz_bytes_writes_float_and_complex_keys(self) -> None:
        payload = air.trace_npz_bytes(
            1,
            self.cfg,
            self.freq,
            self.raw,
            self.display,
            self.time_ns,
            self.traces,
            air.GgaFix(),
            channels=("S11", "S22"),
        )
        import io
        with np.load(io.BytesIO(payload), allow_pickle=False) as data:
            self.assertIn("trace_S11_complex", data.files)
            self.assertIn("trace_S22_complex", data.files)
            self.assertEqual(data["trace_S11_complex"].dtype, np.complex128)
            self.assertEqual(data["trace_S11"].dtype, np.float64)
            np.testing.assert_allclose(data["trace_S11"], data["trace_S11_complex"].real)


class TestNoAbsControlsAndRealDisplay(unittest.TestCase):
    def test_no_abs_or_envelope_controls(self) -> None:
        from PySide6 import QtWidgets
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        # 2026-10-15：空中端旧 GUI 已退役，只检查地面站窗口。
        gs_win = gs.GroundStationWindow("127.0.0.1", 0, Path(tempfile.mkdtemp()), start_receiver=False)

        try:
            # 严格禁止添加任何 abs/包络切换控件
            forbidden_tokens = ["envelope", "包络", "amplitude", "振幅"]
            for target_win, name in [(gs_win, "GroundStationWindow")]:
                all_widgets = target_win.findChildren(QtWidgets.QWidget)
                for w in all_widgets:
                    obj_name = w.objectName().lower()
                    # 避免 "tabs" 中的 "abs" 误报
                    tokens = [t for t in obj_name.split("_") if t]
                    self.assertNotIn("abs", tokens, f"{name} 存在 abs 控件: {w.objectName()}")
                    for word in forbidden_tokens:
                        self.assertNotIn(
                            word,
                            obj_name,
                            f"{name} 存在违禁控件/属性: {w.objectName()} 含 '{word}'",
                        )
                    if hasattr(w, "text") and callable(w.text):
                        text = w.text().lower()
                        self.assertNotIn("abs", text.split())
                        for word in forbidden_tokens:
                            self.assertNotIn(
                                word,
                                text,
                                f"{name} 文本含违禁词: {text} 含 '{word}'",
                            )
        finally:
            gs_win.close()
            app.processEvents()


class TestRealFlightDatasetCompatibility(unittest.TestCase):
    def test_load_and_process_real_npz(self) -> None:
        run_dir = Path("remote_runs/20260915_102919_0001")
        if not run_dir.exists():
            self.skipTest("真实飞行数据未找到")
        npz_files = sorted(run_dir.glob("trace_*.npz"))
        if not npz_files:
            self.skipTest("真实飞行 NPZ 为空")

        with np.load(npz_files[0], allow_pickle=True) as d:
            freq_hz = np.asarray(d["frequency_hz"], dtype=np.float64)
            s11_raw = d["display_S11_real"] + 1j * d["display_S11_imag"]
            s22_raw = d["display_S22_real"] + 1j * d["display_S22_imag"]

        cfg = air.SweepConfig(start_mhz=400, stop_mhz=4000, points=201, time_window_ns=40.0)
        display, time_ns, traces = air.process_trace_pipeline(
            freq_hz,
            {"S11": s11_raw, "S22": s22_raw},
            cfg,
        )

        # 生产默认 oversampling=8：每道 1475 点（与重构前吞吐余量一致），全程复数
        self.assertEqual(time_ns.size, 1475)
        self.assertEqual(traces["S11"].size, 1475)
        self.assertEqual(traces["S22"].size, 1475)
        self.assertTrue(np.all(np.isfinite(traces["S11"])))
        self.assertTrue(np.all(np.isfinite(traces["S22"])))


if __name__ == "__main__":
    unittest.main()
