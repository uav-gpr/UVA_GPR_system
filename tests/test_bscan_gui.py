import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

try:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")
    os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
    from PySide6 import QtCore, QtWidgets
    import uav_gpr_ground_station as ground
    import air_core as air
    import gpr_processing
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover - environment dependent
    QtCore = None
    QtWidgets = None
    ground = None
    air = None
    gpr_processing = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


@unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
class GroundBScanGuiTests(unittest.TestCase):
    def test_ground_bscan_is_inverted_and_uses_fixed_window(self) -> None:
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
        try:
            with patch.object(ground, "ReceiverThread", DummyReceiver):
                window = ground.GroundStationWindow("127.0.0.1", 0, Path(temp.name))
            try:
                self.assertTrue(all(plot.getPlotItem().getViewBox().yInverted() for plot in window.bscan_plots.values()))
                self.assertTrue(all(container.hasHeightForWidth() for container in window.bscan_containers.values()))
                old_time = np.array([0.0, 2.0, 4.0, 6.0])
                old_traces = {name: np.arange(4, dtype=float) + index for index, name in enumerate(ground.UI_CHANNELS)}
                self.assertTrue(window._append_bscan_trace(old_time, old_traces))
                old_history = {name: [row.copy() for row in rows] for name, rows in window.bscan_rows.items()}
                self.assertFalse(window._append_bscan_trace(np.array([0.0, 2.0, 4.0, 6.0, 8.0]), {
                    name: np.arange(5, dtype=float) for name in ground.UI_CHANNELS
                }))
                window.refresh_bscan_views()
                np.testing.assert_array_equal(window.bscan_time_ns, old_time)
                self.assertEqual(len(window.bscan_rows["S11"]), len(window.bscan_rows["S22"]))
                np.testing.assert_array_equal(window.bscan_rows["S11"][0], old_history["S11"][0])
                self.assertIn("请清空B-scan", window.bscan_filter_status.text())
                window.clear_bscan()
                new_time = np.arange(5, dtype=float)
                self.assertTrue(window._append_bscan_trace(new_time, {
                    name: np.ones(5) for name in ground.UI_CHANNELS
                }))
                np.testing.assert_array_equal(window.bscan_time_ns, new_time)
                window.clear_bscan()
                # 扩容 64→256（任务 t1）：喂 257 道历史保持滑块 max=2 的原断言意图。
                window.bscan_rows = {
                    name: [np.full(4, float(index)) for index in range(257)]
                    for name in ground.UI_CHANNELS
                }
                window.refresh_bscan_views()
                self.assertEqual(window.bscan_window_slider.maximum(), 2)
                self.assertEqual(window.bscan_images["S11"].width(), 256)
                self.assertEqual(window.bscan_images["S11"].height(), 4)
                self.assertEqual(window.bscan_images["S11"].width(), window.bscan_images["S22"].width())
                window.bscan_window_slider.setValue(2)
                self.assertIn("2", window.bscan_window_label.text())
            finally:
                window.close()
                app.processEvents()
        finally:
            temp.cleanup()

    def test_300_trace_history_shows_last_256(self) -> None:
        """扩容 64→256（任务 t1）锁定测试：300 道历史走生产追加路径
        （_append_bscan_trace），显示窗必须是最后 256 道；滑块范围、
        NaN 填充、setRect/xRange 全部跟随 BSCAN_WINDOW_TRACES=256。"""
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
        try:
            with patch.object(ground, "ReceiverThread", DummyReceiver):
                window = ground.GroundStationWindow("127.0.0.1", 0, Path(temp.name))
            try:
                rows = [np.arange(8, dtype=float) + index for index in range(300)]
                time_ns = np.linspace(0.0, 40.0, 8)
                for row in rows:
                    self.assertTrue(window._append_bscan_trace(
                        time_ns, {"S11": row, "S22": row + 1.0}
                    ))
                window.refresh_bscan_views()
                app.processEvents()
                # follow-latest：显示窗收敛到最后 256 道（起点 45）。
                self.assertEqual(window.bscan_trace_count(), 300)
                self.assertEqual(window.bscan_window_start, 45)
                self.assertEqual(window.bscan_window_slider.maximum(), 45)
                self.assertEqual(
                    window.bscan_window_label.text(), "显示 45–300 / 共 300 道"
                )
                self.assertEqual(window.bscan_images["S11"].width(), 256)
                self.assertEqual(window.bscan_images["S11"].height(), 8)
                matrix, _levels = window._bscan_display_window("S11")
                self.assertEqual(matrix.shape, (8, 256))
                self.assertEqual(int(np.sum(np.any(np.isfinite(matrix), axis=0))), 256)
                np.testing.assert_array_equal(matrix[:, 0], rows[44])
                np.testing.assert_array_equal(matrix[:, 255], rows[299])
                x_range = window.bscan_plots["S11"].getViewBox().viewRange()[0]
                self.assertAlmostEqual(x_range[0], 44.5)
                self.assertAlmostEqual(x_range[1], 300.5)
                # 不足一窗：100 道 → 右侧列 100..255 全 NaN 填充。
                window.clear_bscan()
                for row in rows[:100]:
                    self.assertTrue(window._append_bscan_trace(
                        time_ns, {"S11": row, "S22": row + 1.0}
                    ))
                window.refresh_bscan_views()
                matrix100, _ = window._bscan_display_window("S11")
                self.assertEqual(window.bscan_images["S11"].width(), 256)
                self.assertTrue(np.all(np.isnan(matrix100[:, 100:])))
                np.testing.assert_array_equal(matrix100[:, 0], rows[0])
                np.testing.assert_array_equal(matrix100[:, 99], rows[99])
                # 拖滑块到合法中段起点 20 → 显示 20–275。
                window.clear_bscan()
                for row in rows:
                    window._append_bscan_trace(time_ns, {"S11": row, "S22": row + 1.0})
                window.refresh_bscan_views()
                window.bscan_window_slider.setValue(20)
                app.processEvents()
                self.assertEqual(
                    window.bscan_window_label.text(), "显示 20–275 / 共 300 道"
                )
                matrix20, _ = window._bscan_display_window("S11")
                np.testing.assert_array_equal(matrix20[:, 0], rows[19])
                np.testing.assert_array_equal(matrix20[:, 255], rows[274])
            finally:
                window.close()
                app.processEvents()
        finally:
            temp.cleanup()


    def test_ground_flat_filter_is_optional_display_only(self) -> None:
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

        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        temp = tempfile.TemporaryDirectory()
        try:
            with patch.object(ground, "ReceiverThread", DummyReceiver):
                window = ground.GroundStationWindow("127.0.0.1", 0, Path(temp.name))
            try:
                self.assertFalse(window.bscan_settings.flat_filter_enabled)
                dialog = ground.BscanSettingsDialog(window)
                dialog.load_from(window.bscan_settings)
                self.assertFalse(dialog.flat_window.isEnabled())
                saved_window = dialog.flat_window.value()
                window.bscan_rows = {
                    name: [np.full(4, float(index)) for index in range(40)]
                    for name in ground.UI_CHANNELS
                }
                window.bscan_time_ns = np.arange(4, dtype=float)
                old_rows = {name: [row.copy() for row in rows] for name, rows in window.bscan_rows.items()}
                old_time = window.bscan_time_ns.copy()
                with patch.object(ground, "flat_reflection_filter", wraps=ground.flat_reflection_filter) as mocked:
                    window.refresh_bscan_views()
                    self.assertEqual(mocked.call_count, 0)
                    self.assertIn("已禁用", window.bscan_filter_status.text())
                    window.apply_bscan_settings(ground.BscanUiSettings(
                        colormap="gray",
                        aspect_ratio=window.bscan_settings.aspect_ratio,
                        flat_filter_enabled=True,
                        flat_window_traces=saved_window,
                    ))
                    dialog.load_from(window.bscan_settings)
                    self.assertTrue(dialog.flat_window.isEnabled())
                    self.assertEqual(mocked.call_count, 2)
                    window.apply_bscan_settings(ground.BscanUiSettings(
                        colormap="gray",
                        aspect_ratio=window.bscan_settings.aspect_ratio,
                        flat_filter_enabled=False,
                        flat_window_traces=saved_window,
                    ))
                    dialog.load_from(window.bscan_settings)
                    self.assertFalse(dialog.flat_window.isEnabled())
                    self.assertEqual(dialog.flat_window.value(), saved_window)
                    self.assertEqual(mocked.call_count, 2)
                self.assertEqual(window.bscan_trace_count(), 40)
                np.testing.assert_array_equal(window.bscan_time_ns, old_time)
                for name in ground.UI_CHANNELS:
                    np.testing.assert_array_equal(window.bscan_rows[name][0], old_rows[name][0])
            finally:
                window.close()
                app.processEvents()
        finally:
            temp.cleanup()

    def test_ground_rejects_changed_dewow_signature_and_clear_restarts(self) -> None:
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

        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        temp = tempfile.TemporaryDirectory()
        try:
            with patch.object(ground, "ReceiverThread", DummyReceiver):
                window = ground.GroundStationWindow("127.0.0.1", 0, Path(temp.name))
            try:
                time_ns = np.arange(4, dtype=float)
                traces = {name: np.ones(4) for name in ground.UI_CHANNELS}
                self.assertTrue(window._append_bscan_trace(time_ns, traces, (False, None)))
                self.assertFalse(window._append_bscan_trace(time_ns, traces, (True, 4.0)))
                window.refresh_bscan_views()
                self.assertIn("Dewow设置已变化", window.bscan_filter_status.text())
                self.assertEqual(window.bscan_trace_count(), 1)
                window.clear_bscan()
                self.assertTrue(window._append_bscan_trace(time_ns, traces, (True, 4.0)))
            finally:
                window.close()
                app.processEvents()
        finally:
            temp.cleanup()


    @unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
    def test_ground_reads_new_and_legacy_dewow_metadata(self) -> None:
        class Data:
            def __init__(self, values: dict[str, object]) -> None:
                self.files = tuple(values)
                self.values = values

            def __getitem__(self, key: str) -> object:
                return self.values[key]

        enabled_key = "dew" + "ow_enabled"
        window_key = "dew" + "ow_window_ns"
        disabled = Data({"processing": {enabled_key: False}})
        signature, state = ground.dewow_settings_from_sources(disabled, {})
        self.assertEqual(signature, (False, None))
        self.assertEqual(state, "disabled")

        enabled = Data({"processing": {
            enabled_key: True,
            window_key: 4.0,
        }})
        signature, state = ground.dewow_settings_from_sources(enabled, {})
        self.assertEqual(signature, (True, 4.0))
        self.assertEqual(state, "enabled")

        legacy = Data({})
        signature, state = ground.dewow_settings_from_sources(legacy, {})
        self.assertEqual(signature, (True, 4.0))
        self.assertEqual(state, "legacy")

    @unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
    def test_ground_dewow_metadata_is_strict_and_source_ordered(self) -> None:
        class Data:
            def __init__(self, values: dict[str, object]) -> None:
                self.files = tuple(values)
                self.values = values

            def __getitem__(self, key: str) -> object:
                return self.values[key]

        enabled_key = "dew" + "ow_enabled"
        window_key = "dew" + "ow_window_ns"
        nested_key = "dew" + "ow"

        for invalid_window in (-1, 0, np.nan, np.inf, -np.inf, "invalid", None):
            with self.subTest(invalid_window=invalid_window):
                source = Data({"processing": {
                    enabled_key: True,
                    window_key: invalid_window,
                }})
                with self.assertRaisesRegex(ValueError, "window"):
                    ground.dewow_settings_from_sources(source, {})

        for invalid_enabled in ("maybe", {"value": True}, np.nan, np.inf, -np.inf):
            with self.subTest(invalid_enabled=invalid_enabled):
                source = Data({"processing": {enabled_key: invalid_enabled}})
                with self.assertRaisesRegex(ValueError, "enabled"):
                    ground.dewow_settings_from_sources(source, {})

        for invalid_window in (None, "invalid", np.nan, np.inf, -np.inf, 0, -1):
            with self.subTest(disabled_window=invalid_window):
                source = Data({"processing": {
                    enabled_key: "off",
                    window_key: invalid_window,
                }})
                self.assertEqual(
                    ground.dewow_settings_from_sources(source, {}),
                    ((False, None), "disabled"),
                )

        with self.assertRaisesRegex(ValueError, "object"):
            ground.dewow_settings_from_sources(
                Data({"processing": {nested_key: "not-an-object"}}),
                {},
            )

        # Nested fields are authoritative over the compatibility fields.
        nested = Data({"processing": {
            nested_key: {"enabled": "yes", "window_ns": np.float64(6.0)},
            enabled_key: "false",
            window_key: 2.0,
        }})
        self.assertEqual(
            ground.dewow_settings_from_sources(nested, {}),
            ((True, 6.0), "enabled"),
        )
        nested_disabled = Data({"processing": {
            nested_key: {"enabled": np.bool_(False), "window_ns": "invalid"},
            enabled_key: True,
            window_key: 2.0,
        }})
        self.assertEqual(
            ground.dewow_settings_from_sources(nested_disabled, {}),
            ((False, None), "disabled"),
        )
        top_level_bytes = Data({"processing": {
            enabled_key: b"true",
            window_key: np.bytes_(b"4.5"),
        }})
        self.assertEqual(
            ground.dewow_settings_from_sources(top_level_bytes, {}),
            ((True, 4.5), "enabled"),
        )

        # The real old sender recorded only the window in each possible
        # source.  The value is authoritative and is not replaced by 4.0 ns.
        legacy_sources = (
            (Data({"processing": {window_key: 4.0, "ifft_points": 2048}}), {}),
            (Data({"config": {window_key: 6.0}}), {}),
            (Data({}), {"processing": {window_key: 6.0}}),
            (Data({}), {"config": {window_key: 6.0}}),
        )
        for source, header in legacy_sources:
            with self.subTest(legacy_source=(source.files, header)):
                self.assertEqual(
                    ground.dewow_settings_from_sources(source, header),
                    ((True, 6.0 if "config" in source.files or header else 4.0), "legacy"),
                )
        for invalid_window in (0, -1, np.nan, np.inf, -np.inf, "invalid", None):
            with self.subTest(legacy_invalid_window=invalid_window):
                with self.assertRaisesRegex(ValueError, "legacy window"):
                    ground.dewow_settings_from_sources(
                        Data({"processing": {window_key: invalid_window}}),
                        {"config": {window_key: 6.0}},
                    )

        # A source with no Dewow fields is skipped in the documented order.
        self.assertEqual(
            ground.dewow_settings_from_sources(
                Data({
                    "processing": {"other": 1},
                    "config": {enabled_key: False},
                }),
                {"processing": {enabled_key: True}, "config": {enabled_key: True}},
            ),
            ((False, None), "disabled"),
        )
        self.assertEqual(
            ground.dewow_settings_from_sources(
                Data({"processing": {"other": 1}, "config": {"other": 2}}),
                {"processing": {enabled_key: False}, "config": {enabled_key: True}},
            ),
            ((False, None), "disabled"),
        )
        self.assertEqual(
            ground.dewow_settings_from_sources(
                Data({"processing": {"other": 1}, "config": {"other": 2}}),
                {"processing": {"other": 3}, "config": {enabled_key: False}},
            ),
            ((False, None), "disabled"),
        )
        self.assertEqual(
            ground.dewow_settings_from_sources(
                Data({"processing": {"other": 1}, "config": {"other": 2}}),
                {"processing": {"other": 3}, "config": {"other": 4}},
            ),
            ((True, 4.0), "legacy"),
        )

        # Invalid high-priority metadata must not be hidden by lower sources.
        with self.assertRaisesRegex(ValueError, "window"):
            ground.dewow_settings_from_sources(
                Data({
                    "processing": {enabled_key: True, window_key: -1},
                    "config": {enabled_key: False},
                }),
                {"processing": {enabled_key: False}, "config": {enabled_key: False}},
            )

    @unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
    def test_ground_invalid_dewow_payload_is_rejected_atomically(self) -> None:
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

        def make_payload(processing: dict[str, object], offset: float) -> bytes:
            output = io.BytesIO()
            np.savez_compressed(
                output,
                frequency_hz=np.array([1.0, 2.0, 3.0]),
                time_ns=np.arange(4, dtype=float),
                display_S11_real=np.array([1.0, 2.0, 3.0]) + offset,
                display_S11_imag=np.zeros(3),
                display_S22_real=np.array([2.0, 3.0, 4.0]) + offset,
                display_S22_imag=np.zeros(3),
                trace_S11=np.arange(4, dtype=float) + offset,
                trace_S22=np.arange(4, dtype=float) + 1.0 + offset,
                processing=json.dumps(processing),
            )
            return output.getvalue()

        def snapshot(window) -> dict[str, object]:
            return {
                "bscan_s11": [np.asarray(r).copy() for r in window.bscan_rows["S11"]],
                "bscan_s22": [np.asarray(r).copy() for r in window.bscan_rows["S22"]],
                "bscan_time": None if window.bscan_time_ns is None else np.asarray(window.bscan_time_ns).copy(),
                "bscan_sig": window.bscan_processing_signature,
                "bscan_window_start": window.bscan_window_start,
                "acq": window.acquisition_label.text(),
                "legacy": window.legacy_status.text(),
                "dewow": window.dewow_status.text(),
                "filter": window.bscan_filter_status.text(),
                "trace_status": window.trace_status_label.text(),
            }

        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        temp = tempfile.TemporaryDirectory()
        try:
            with patch.object(ground, "ReceiverThread", DummyReceiver):
                window = ground.GroundStationWindow("127.0.0.1", 0, Path(temp.name))
            try:
                window.on_trace(
                    {"trace_index": 1},
                    make_payload({"dew" + "ow_enabled": False}, 0.0),
                    "baseline.npz",
                )
                before = snapshot(window)

                invalid_payload = make_payload(
                    {"dew" + "ow_enabled": True, "dew" + "ow_window_ns": -1},
                    100.0,
                )
                window.on_trace(
                    {"processing": {"dew" + "ow_enabled": False}},
                    invalid_payload,
                    "invalid.npz",
                )

                after = snapshot(window)
                for key in before:
                    b = before[key]
                    a = after[key]
                    if isinstance(b, np.ndarray):
                        np.testing.assert_array_equal(a, b, err_msg=f"{key} 应保持不变")
                    elif isinstance(b, list):
                        self.assertEqual(len(a), len(b), f"{key} 数量不应变化")
                        for ba, bb in zip(a, b):
                            np.testing.assert_array_equal(np.asarray(ba), np.asarray(bb), err_msg=f"{key} 内容不应变化")
                    else:
                        self.assertEqual(a, b, f"{key} 不应变化")
                self.assertIn("Trace已忽略", window.log_box.toPlainText())
            finally:
                window.close()
                app.processEvents()
        finally:
            temp.cleanup()

    @unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
    def test_ground_accepts_real_legacy_window_only_payload(self) -> None:
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

        output = io.BytesIO()
        np.savez_compressed(
            output,
            frequency_hz=np.array([1.0, 2.0, 3.0]),
            time_ns=np.arange(4, dtype=float),
            trace_S11=np.arange(4, dtype=float),
            trace_S22=np.arange(4, dtype=float) + 1.0,
            processing=json.dumps({
                "bpf": {"fmin": 1.0, "fmax": 3.0},
                "dew" + "ow_window_ns": 6.0,
                "ifft_points": 2048,
            }),
            config=json.dumps({"dew" + "ow_window_ns": 6.0}),
        )

        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        temp = tempfile.TemporaryDirectory()
        try:
            with patch.object(ground, "ReceiverThread", DummyReceiver):
                window = ground.GroundStationWindow("127.0.0.1", 0, Path(temp.name))
            try:
                with patch("gpr_processing.dewow") as mocked_dewow:
                    window.on_trace({"trace_index": 7}, output.getvalue(), "legacy.npz")
                mocked_dewow.assert_not_called()
                self.assertEqual(window.dewow_status.text(), "Dewow：旧格式默认启用")
                self.assertEqual(window.bscan_processing_signature, (True, 6.0))
                self.assertEqual(window.bscan_trace_count(), 1)
                self.assertEqual(len(window.bscan_rows["S11"]), len(window.bscan_rows["S22"]))
                np.testing.assert_array_equal(window.bscan_time_ns, np.arange(4, dtype=float))
            finally:
                window.close()
                app.processEvents()
        finally:
            temp.cleanup()


if __name__ == "__main__":
    unittest.main()
