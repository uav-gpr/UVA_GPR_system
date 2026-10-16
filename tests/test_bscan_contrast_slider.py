"""地面站 B-scan 色阶（colorbar 显示范围）滑块测试。"""

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
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover - environment dependent
    QtCore = None
    QtWidgets = None
    ground = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


class BscanContrastLevelsTests(unittest.TestCase):
    @unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
    def test_levels_scale_and_clamp(self) -> None:
        f = ground.bscan_contrast_levels
        self.assertEqual(f(4.0, 100), (-4.0, 4.0))
        self.assertEqual(f(4.0, 25), (-1.0, 1.0))
        self.assertEqual(f(4.0, 0), f(4.0, ground.BSCAN_CONTRAST_MIN_PERCENT))
        self.assertEqual(f(4.0, 500), (-4.0, 4.0))
        self.assertEqual(f(0.0, 100), (-1.0, 1.0))
        self.assertEqual(f(float("nan"), 50), (-0.5, 0.5))
        self.assertEqual(f(2.0, float("nan")), (-2.0, 2.0))


@unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
class GroundBscanContrastSliderTests(unittest.TestCase):
    def _make_window(self, temp_dir: str):
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

        QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with patch.object(ground, "ReceiverThread", DummyReceiver):
            return ground.GroundStationWindow("127.0.0.1", 0, Path(temp_dir))

    def _fill(self, window) -> None:
        time_ns = np.linspace(0.0, 10.0, 8)
        for i in range(5):
            s11 = np.sin(time_ns + i) * 2.0  # max|x| ≈ 2
            s22 = np.cos(time_ns + i) * 8.0  # max|x| ≈ 8
            self.assertTrue(window._append_bscan_trace(time_ns, {"S11": s11, "S22": s22}))
        window.refresh_bscan_views()

    def test_default_is_100_percent_and_unchanged_levels(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            window = self._make_window(temp)
            try:
                slider = window.findChild(QtWidgets.QSlider, "bscan_contrast_slider")
                self.assertIsNotNone(slider)
                self.assertEqual(slider.orientation(), QtCore.Qt.Orientation.Vertical)
                self.assertEqual(slider.value(), 100)
                self.assertEqual((slider.minimum(), slider.maximum()), (1, 100))
                self._fill(window)
                for name in ground.UI_CHANNELS:
                    matrix, levels = window._bscan_display_window(name)
                    finite = matrix[np.isfinite(matrix)]
                    max_abs = float(np.max(np.abs(finite)))
                    self.assertAlmostEqual(levels[0], -max_abs)
                    self.assertAlmostEqual(levels[1], max_abs)
            finally:
                window.close()

    def test_slider_scales_image_levels_per_channel(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            window = self._make_window(temp)
            try:
                self._fill(window)
                full = {n: tuple(window.bscan_images[n].getLevels()) for n in ground.UI_CHANNELS}
                window.bscan_contrast_slider.setValue(25)
                self.assertEqual(window.bscan_contrast_percent, 25)
                self.assertEqual(window.bscan_contrast_label.text(), "25%")
                for name in ground.UI_CHANNELS:
                    lo, hi = window.bscan_images[name].getLevels()
                    self.assertAlmostEqual(hi, full[name][1] * 0.25)
                    self.assertAlmostEqual(lo, full[name][0] * 0.25)
                # 新数据到来后刷新仍保持当前百分比
                window.refresh_bscan_views()
                _m, levels = window._bscan_display_window("S22")
                self.assertAlmostEqual(levels[1], full["S22"][1] * 0.25)
                window.bscan_contrast_reset_button.click()
                self.assertEqual(window.bscan_contrast_slider.value(), 100)
                for name in ground.UI_CHANNELS:
                    lo, hi = window.bscan_images[name].getLevels()
                    self.assertAlmostEqual(hi, full[name][1])
            finally:
                window.close()

    def test_slider_before_data_does_not_crash(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            window = self._make_window(temp)
            try:
                window.bscan_contrast_slider.setValue(10)
                self.assertEqual(window.bscan_contrast_percent, 10)
                self._fill(window)
                _m, levels = window._bscan_display_window("S11")
                matrix = _m[np.isfinite(_m)]
                self.assertAlmostEqual(levels[1], float(np.max(np.abs(matrix))) * 0.10)
            finally:
                window.close()


if __name__ == "__main__":
    unittest.main()
