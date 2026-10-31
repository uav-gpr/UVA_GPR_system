"""整次采集合并 + 离线重处理（run_archive / tools/merge_runs / offline_viewer）测试。

覆盖：
1. 地面命名 trace_00001.npz 与空中命名 trace_00001_<时间戳>.npz 都能合并；
2. 缺号、重复编号、损坏文件、参数不一致文件：排序去重、记账、跳过，不让合并失败；
3. 合并文件原子写入、可读回、数组与原始逐道完全一致；原始文件保留；
4. 离线用采集时同参数重算 = 空中端实时结果（Dewow 开/关两种）；
5. spectrum_to_trace 与 air_core.make_time_trace 逐点一致；
6. Flat Filter 与 gpr_processing.flat_reflection_filter 一致；
7. 老格式（无 display_*）退回已存时域 trace；
8. load_run：合并文件比单道新时读合并文件，否则现场组装；
9. merge_runs 工具默认跳过 _c / _idle / _ 开头文件夹；
10. 回看窗口：打开、处理、导出、合并；地面主窗口 B-scan 菜单有入口。
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import run_archive as ra  # noqa: E402
from gpr_processing import flat_reflection_filter  # noqa: E402

try:
    import air_core as air  # noqa: E402
    AIR_ERROR = ""
except Exception as exc:  # pragma: no cover - 环境缺依赖时跳过
    air = None
    AIR_ERROR = str(exc)


def _make_trace_bytes(index: int, cfg, freq, rng, channels=("S11", "S22")) -> bytes:
    raw = {
        name: rng.normal(size=freq.size) + 1j * rng.normal(size=freq.size)
        for name in channels
    }
    display, time_ns, traces = air.process_trace_pipeline(freq, raw, cfg)
    fix = air.GgaFix(latitude=30.0 + index * 1e-5, longitude=114.0, fix_quality=4, satellites=20, altitude_m=12.5)
    return air.trace_npz_bytes(
        index, cfg, freq, raw, display, time_ns, traces, fix,
        channels=channels, acquisition={"run_id": "r", "trace_index": index},
    )


@unittest.skipIf(air is None, f"air_core unavailable: {AIR_ERROR}")
class RunArchiveTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.run_dir = self.root / "20261022_101500_0001"
        self.run_dir.mkdir()
        self.rng = np.random.default_rng(7)
        self.cfg = air.SweepConfig(start_mhz=400.0, stop_mhz=4000.0, points=201, dewow_enabled=True, dewow_window_ns=4.0)
        self.freq = np.linspace(400e6, 4000e6, 201)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def _write(self, name: str, index: int, **kwargs) -> Path:
        path = self.run_dir / name
        path.write_bytes(_make_trace_bytes(index, kwargs.get("cfg", self.cfg), kwargs.get("freq", self.freq), self.rng))
        return path

    def _standard_run(self) -> None:
        # 1,2,4,5 + 空中命名的 6；缺 3；重复 2；一个损坏；一个频率不一致。
        for i in (1, 2, 4, 5):
            self._write(f"trace_{i:05d}.npz", i)
        self._write("trace_00006_20261022_101501_123.npz", 6)
        self._write("trace_00002_20261022_101502_000.npz", 2)  # 重复编号
        (self.run_dir / "trace_00007.npz").write_bytes(b"not a zip")
        cfg2 = air.SweepConfig(start_mhz=400.0, stop_mhz=3000.0, points=151, dewow_enabled=True)
        self._write("trace_00008.npz", 8, cfg=cfg2, freq=np.linspace(400e6, 3000e6, 151))
        (self.run_dir / "run_metadata.jsonl").write_text("{}\n", encoding="utf-8")

    def test_merge_handles_gaps_duplicates_corrupt_and_mismatch(self) -> None:
        self._standard_run()
        before = sorted(p.name for p in self.run_dir.iterdir())
        report = ra.merge_run(self.run_dir)
        self.assertEqual(report.n_traces, 5)
        self.assertEqual(report.channels, ("S11", "S22"))
        self.assertEqual(report.missing_trace_indices, [3, 7, 8])
        self.assertEqual(report.duplicate_trace_indices, [2])
        reasons = {item["file"]: item["reason"] for item in report.skipped_files}
        self.assertIn("trace_00007.npz", reasons)
        self.assertIn("频率点不一致", reasons["trace_00008.npz"])
        self.assertIn("trace_00002_20261022_101502_000.npz", reasons)
        self.assertTrue(report.output_path.exists())
        self.assertEqual(report.output_path.name, "20261022_101500_0001_merged.npz")
        self.assertFalse(any(p.name.endswith(".tmp") for p in self.run_dir.iterdir()))
        after = sorted(p.name for p in self.run_dir.iterdir())
        self.assertEqual(sorted(set(after) - set(before)), [report.output_path.name])
        self.assertIn("缺号 3, 7–8", report.summary_text())

        merged = ra.load_merged(report.output_path)
        np.testing.assert_array_equal(merged.trace_index, [1, 2, 4, 5, 6])
        with np.load(self.run_dir / "trace_00004.npz") as data:
            np.testing.assert_allclose(merged.trace["S22"][2], data["trace_S22_complex"])
            np.testing.assert_allclose(merged.display["S11"][2].real, data["display_S11_real"])
            np.testing.assert_allclose(merged.raw["S11"][2].imag, data["raw_S11_imag"])
        with np.load(report.output_path) as data:
            self.assertEqual(str(data["format"]), ra.MERGED_FORMAT)
            np.testing.assert_allclose(data["gnss_latitude"][0], 30.0 + 1e-5)
            self.assertEqual(int(data["gnss_fix_quality"][0]), 4)
            self.assertEqual(data["trace_S11_complex"].shape[0], 5)

    def test_offline_same_params_reproduces_live_traces(self) -> None:
        for dewow_enabled in (True, False):
            with self.subTest(dewow=dewow_enabled):
                for p in self.run_dir.iterdir():
                    p.unlink()
                self.cfg = air.SweepConfig(dewow_enabled=dewow_enabled, dewow_window_ns=4.0)
                for i in (1, 2, 3):
                    self._write(f"trace_{i:05d}.npz", i)
                run = ra.load_run(self.run_dir)
                result = ra.reprocess(run, ra.ProcessingParams(dewow_enabled=dewow_enabled, dewow_window_ns=4.0))
                self.assertEqual(result.source_kind, "display")
                np.testing.assert_allclose(result.time_ns, run.time_ns)
                for name in run.channels:
                    np.testing.assert_allclose(result.matrices[name], run.trace[name].T, rtol=1e-12, atol=1e-15)

    def test_spectrum_to_trace_matches_air_make_time_trace(self) -> None:
        spec = self.rng.normal(size=self.freq.size) + 1j * self.rng.normal(size=self.freq.size)
        for enabled in (True, False):
            cfg = air.SweepConfig(dewow_enabled=enabled, dewow_window_ns=3.0, time_window_ns=50.0)
            t_air, x_air = air.make_time_trace(self.freq, spec, cfg)
            t_off, x_off = ra.spectrum_to_trace(
                self.freq, spec, ifft_points=cfg.ifft_points, time_window_ns=cfg.time_window_ns,
                dewow_enabled=enabled, dewow_window_ns=cfg.dewow_window_ns,
            )
            np.testing.assert_array_equal(t_air, t_off)
            np.testing.assert_array_equal(x_air, x_off)

    def test_flat_filter_matches_gpr_processing(self) -> None:
        for i in range(1, 8):
            self._write(f"trace_{i:05d}.npz", i)
        run = ra.load_run(self.run_dir)
        base = ra.reprocess(run, ra.ProcessingParams(dewow_enabled=True))
        flat = ra.reprocess(run, ra.ProcessingParams(dewow_enabled=True, flat_enabled=True, flat_window_traces=5))
        np.testing.assert_allclose(flat.matrices["S11"], flat_reflection_filter(base.matrices["S11"], window_size=5))
        short = ra.reprocess(run, ra.ProcessingParams(flat_enabled=True, flat_window_traces=31))
        self.assertTrue(any("少于 Flat Filter 窗口" in n for n in short.notes))

    def test_legacy_file_without_display_falls_back(self) -> None:
        time_ns = np.linspace(0, 40, 50)
        for i in (1, 2, 3):
            np.savez_compressed(
                self.run_dir / f"trace_{i:05d}.npz",
                frequency_hz=self.freq, time_ns=time_ns, trace=self.rng.normal(size=50),
            )
        run = ra.load_run(self.run_dir)
        self.assertEqual(run.channels, ("S11",))
        result = ra.reprocess(run, ra.ProcessingParams(dewow_enabled=True, dewow_window_ns=4.0))
        self.assertEqual(result.source_kind, "stored_trace")
        self.assertEqual(result.matrices["S11"].shape, (50, 3))
        self.assertTrue(result.notes)

    def test_load_run_prefers_fresh_merged_file(self) -> None:
        for i in (1, 2):
            self._write(f"trace_{i:05d}.npz", i)
        report = ra.merge_run(self.run_dir)
        run = ra.load_run(self.run_dir)
        self.assertEqual(Path(run.source), report.output_path)
        # 合并后又补进来一道（比如断线补发）→ 现场重新组装。
        self._write("trace_00003.npz", 3)
        newer = time.time() + 5
        os.utime(self.run_dir / "trace_00003.npz", (newer, newer))
        run = ra.load_run(self.run_dir)
        self.assertEqual(Path(run.source), self.run_dir)
        self.assertEqual(run.n_traces, 3)
        # 直接给单道文件也能打开整次采集。
        self.assertEqual(ra.load_run(self.run_dir / "trace_00001.npz").n_traces, 3)

    def test_errors_are_chinese_and_safe(self) -> None:
        with self.assertRaises(ra.RunArchiveError):
            ra.build_run_from_folder(self.root / "nope")
        with self.assertRaises(ra.RunArchiveError):
            ra.build_run_from_folder(self.run_dir)  # 空文件夹
        (self.run_dir / "trace_00001.npz").write_bytes(b"bad")
        with self.assertRaises(ra.RunArchiveError):
            ra.merge_run(self.run_dir)
        self.assertFalse(ra.merged_path_for(self.run_dir).exists())

    def test_merge_tool_skips_calibration_and_idle_runs(self) -> None:
        sys.path.insert(0, str(ROOT / "tools"))
        import merge_runs  # noqa: PLC0415

        for i in (1, 2):
            self._write(f"trace_{i:05d}.npz", i)
        for name in ("20261022_100000_0001_c", "20261022_100000_idle", "_outbox"):
            d = self.root / name
            d.mkdir()
            (d / "trace_00001.npz").write_bytes(self.run_dir.joinpath("trace_00001.npz").read_bytes())
        found = merge_runs.find_run_dirs(self.root)
        self.assertEqual([p.name for p in found], [self.run_dir.name])
        self.assertEqual(len(merge_runs.find_run_dirs(self.root, include_all=True)), 4)
        self.assertEqual(merge_runs.main([str(self.root)]), 0)
        self.assertTrue(ra.merged_path_for(self.run_dir).exists())

    def test_save_processed(self) -> None:
        for i in (1, 2, 3):
            self._write(f"trace_{i:05d}.npz", i)
        run = ra.load_run(self.run_dir)
        result = ra.reprocess(run, ra.ProcessingParams(dewow_enabled=False))
        out = ra.save_processed(run, result, self.root / "out.npz")
        with np.load(out) as data:
            self.assertEqual(str(data["format"]), ra.PROCESSED_FORMAT)
            self.assertEqual(json.loads(str(data["processing_params"]))["dewow_enabled"], False)
            self.assertEqual(data["bscan_S22"].shape, result.matrices["S22"].shape)


try:
    from PySide6 import QtCore, QtWidgets  # noqa: E402
    import offline_viewer  # noqa: E402
    QT_ERROR = ""
except Exception as exc:  # pragma: no cover
    offline_viewer = None
    QT_ERROR = str(exc)


@unittest.skipIf(air is None or offline_viewer is None, f"Qt/air unavailable: {AIR_ERROR or QT_ERROR}")
class OfflineViewerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.temp = tempfile.TemporaryDirectory()
        self.run_dir = Path(self.temp.name) / "20261022_101500_0001"
        self.run_dir.mkdir()
        rng = np.random.default_rng(3)
        cfg = air.SweepConfig(dewow_enabled=False)
        freq = np.linspace(400e6, 4000e6, 201)
        for i in (1, 2, 3, 5):
            (self.run_dir / f"trace_{i:05d}.npz").write_bytes(_make_trace_bytes(i, cfg, freq, rng))

    def tearDown(self) -> None:
        self.app.processEvents()
        self.temp.cleanup()

    def test_open_process_export_merge(self) -> None:
        viewer = offline_viewer.OfflineRunViewer(default_dir=Path(self.temp.name))
        try:
            self.assertFalse(viewer.apply_button.isEnabled())
            self.assertTrue(viewer.open_path(self.run_dir))
            self.assertEqual(set(viewer.images), {"S11", "S22"})
            self.assertIn("4 道", viewer.info_label.text())
            self.assertIn("缺号 4", viewer.info_label.text())
            matrix = viewer.display_matrix("S11")
            self.assertEqual(matrix.shape[1], 4)
            self.assertFalse(viewer.dewow_check.isChecked())  # 默认 = 采集时参数

            viewer.flat_check.setChecked(True)
            viewer.flat_window.setValue(3)
            self.assertTrue(viewer.apply_processing())
            self.assertTrue(viewer.result.params.flat_enabled)
            viewer.contrast_slider.setValue(40)
            levels = viewer.images["S11"].levels
            self.assertAlmostEqual(float(levels[1]), viewer._max_abs["S11"] * 0.4)

            out = Path(self.temp.name) / "processed.npz"
            self.assertTrue(viewer.export_to(out))
            self.assertTrue(out.exists())
            self.assertTrue(viewer.merge_current())
            self.assertTrue(ra.merged_path_for(self.run_dir).exists())
            self.assertIn("缺号 4", viewer.status_label.text())

            # 打开合并文件：不能再“合并保存”。
            self.assertTrue(viewer.open_path(ra.merged_path_for(self.run_dir)))
            self.assertFalse(viewer.merge_button.isEnabled())
            self.assertFalse(viewer.open_path(Path(self.temp.name) / "nope"))
            self.assertIn("打开失败", viewer.status_label.text())
        finally:
            viewer.close()
            viewer.deleteLater()

    def test_ground_menu_entry_opens_viewer(self) -> None:
        try:
            import uav_gpr_ground_station as ground  # noqa: PLC0415
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"ground unavailable: {exc}")

        class _DummyReceiver(QtCore.QObject):
            status = QtCore.Signal(str)
            command_status = QtCore.Signal(str)
            trace_received = QtCore.Signal(dict, bytes, str)
            _emitted_underflow_errors = 0

            def __init__(self, *_args) -> None:
                super().__init__()

            def start(self) -> None:
                pass

            def stop(self) -> None:
                pass

            def wait(self, _timeout: int) -> bool:
                return True

            def send_command(self, action, params=None) -> None:
                pass

            def begin_run(self, run_id) -> None:
                pass

            def note_signal_consumed(self) -> None:
                pass

        with patch.object(ground, "ReceiverThread", _DummyReceiver):
            window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.temp.name))
        try:
            self.assertEqual([a.text() for a in window.menuBar().actions()], ["任务", "B-scan", "视图", "帮助"])
            self.assertIn(window.action_open_history, window.menu_bscan.actions())
            self.assertEqual(window.action_open_history.objectName(), "action_open_history")
            window.action_open_history.trigger()
            viewer = window.history_viewer
            self.assertTrue(viewer.isVisible())
            window.action_open_history.trigger()
            self.assertIs(window.history_viewer, viewer)  # 只开一个
            # 主界面任务按钮仍然只有 开始/停止/暂停/继续。
            texts = sorted(b.text() for b in window.centralWidget().findChildren(QtWidgets.QPushButton))
            self.assertNotIn("应用处理", texts)
        finally:
            window.close()
            window.deleteLater()


if __name__ == "__main__":
    unittest.main()
