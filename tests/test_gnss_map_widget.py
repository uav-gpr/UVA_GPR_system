import json
import os
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

try:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")
    os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
    from PySide6 import QtCore, QtWidgets
    import gnss_map_widget as gnss
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover - environment dependent
    QtCore = None
    QtWidgets = None
    gnss = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


if QtWidgets is not None:
    class FakePage:
        def __init__(self) -> None:
            self.runJavaScript = MagicMock()


    class FakeWebView(QtWidgets.QWidget):
        loadFinished = QtCore.Signal(bool)

        def __init__(self) -> None:
            super().__init__()
            self.page_object = FakePage()
            self.html = ""

        def setHtml(self, html: str) -> None:
            self.html = html

        def page(self) -> "FakePage":
            return self.page_object
else:  # pragma: no cover - environment dependent
    FakeWebView = None


@unittest.skipIf(gnss is None, f"GNSS GUI dependencies unavailable: {IMPORT_ERROR}")
class GnssMapWidgetTests(unittest.TestCase):
    def setUp(self) -> None:
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.web_patch = patch.object(gnss, "QWebEngineView", FakeWebView)
        self.web_patch.start()
        self.widget = gnss.GnssMapWidget()

    def tearDown(self) -> None:
        self.widget.close()
        self.web_patch.stop()
        self.app.processEvents()

    def test_fix_quality_zero_does_not_call_javascript(self) -> None:
        self.widget.web.loadFinished.emit(True)
        self.widget.update_fix(latitude=30.0, longitude=120.0, fix_quality=0, satellites=8)

        self.widget.web.page_object.runJavaScript.assert_not_called()
        self.assertIn("no valid fix", self.widget.status.text())
        self.assertIn("#fff3cd", self.widget.status.styleSheet())

    def test_valid_fix_calls_javascript_after_page_is_ready(self) -> None:
        self.widget.web.loadFinished.emit(True)
        self.widget.update_fix(
            latitude=30.12345678,
            longitude=120.87654321,
            fix_quality=1,
            satellites=9,
            altitude_m=12.5,
            utc="120000",
        )

        page = self.widget.web.page_object
        page.runJavaScript.assert_called_once()
        script = page.runJavaScript.call_args.args[0]
        self.assertIn("window.updateGnss(30.123456780000, 120.876543210000, ", script)
        self.assertIn("#d1fae5", self.widget.status.styleSheet())

    def test_valid_fix_is_cached_until_successful_load_then_replayed(self) -> None:
        self.widget.update_fix(latitude=30.0, longitude=120.0, fix_quality=1, utc="first")
        self.widget.update_fix(latitude=31.0, longitude=121.0, fix_quality=1, utc="latest")
        page = self.widget.web.page_object
        page.runJavaScript.assert_not_called()
        self.assertIsNotNone(self.widget._pending_map_update)

        self.widget.web.loadFinished.emit(True)

        page.runJavaScript.assert_called_once()
        script = page.runJavaScript.call_args.args[0]
        self.assertIn("31.000000000000, 121.000000000000", script)
        self.assertIsNone(self.widget._pending_map_update)

    def test_failed_load_does_not_send_and_later_success_replays_cache(self) -> None:
        self.widget.update_fix(latitude=30.0, longitude=120.0, fix_quality=1)
        page = self.widget.web.page_object

        self.widget.web.loadFinished.emit(False)
        page.runJavaScript.assert_not_called()
        self.assertFalse(self.widget._page_ready)
        self.assertIsNotNone(self.widget._pending_map_update)

        self.widget.web.loadFinished.emit(True)

        page.runJavaScript.assert_called_once()
        self.assertIsNone(self.widget._pending_map_update)

    def test_numpy_string_and_bytes_inputs_are_supported(self) -> None:
        self.widget.web.loadFinished.emit(True)
        self.widget.update_fix(
            latitude=np.float64(30.0),
            longitude=b"120.0",
            fix_quality=np.int64(1),
            satellites="8",
            altitude_m=b"12.5",
            utc=b"120001",
        )

        self.widget.web.page_object.runJavaScript.assert_called_once()
        self.assertIn("fix=1, sats=8", self.widget.status.text())

    def test_nonfinite_integer_fields_are_safe_and_invalid_fix_is_not_queued(self) -> None:
        self.widget.web.loadFinished.emit(True)
        page = self.widget.web.page_object
        for value in (np.nan, np.inf, -np.inf):
            self.widget.update_fix(latitude=30.0, longitude=120.0, fix_quality=value, satellites=value)

        page.runJavaScript.assert_not_called()
        self.assertIn("fix=0, sats=0", self.widget.status.text())

    def test_invalid_or_out_of_range_coordinates_do_not_call_javascript(self) -> None:
        self.widget.web.loadFinished.emit(True)
        page = self.widget.web.page_object
        for latitude, longitude in (
            (None, 120.0),
            (np.nan, 120.0),
            (91.0, 120.0),
            (-91.0, 120.0),
            (30.0, 181.0),
            (30.0, -181.0),
        ):
            self.widget.update_fix(latitude=latitude, longitude=longitude, fix_quality=1)

        page.runJavaScript.assert_not_called()

    def test_popup_json_escapes_quotes_and_backslashes(self) -> None:
        self.widget.web.loadFinished.emit(True)
        utc = '12"\\00'
        self.widget.update_fix(latitude=30.0, longitude=120.0, fix_quality=1, utc=utc)

        script = self.widget.web.page_object.runJavaScript.call_args.args[0]
        prefix = "window.updateGnss(30.000000000000, 120.000000000000, "
        self.assertTrue(script.startswith(prefix))
        popup = json.loads(script[len(prefix):-2])
        self.assertIn(utc, popup)


class GnssIntegerConversionTests(unittest.TestCase):
    @unittest.skipIf(gnss is None, f"GNSS GUI dependencies unavailable: {IMPORT_ERROR}")
    def test_int_or_default_handles_finite_and_nonfinite_values(self) -> None:
        self.assertEqual(gnss._int_or_default(4), 4)
        self.assertEqual(gnss._int_or_default(np.int64(5)), 5)
        self.assertEqual(gnss._int_or_default("6"), 6)
        self.assertEqual(gnss._int_or_default(b"7"), 7)
        for value in (None, "invalid", np.nan, np.inf, -np.inf):
            self.assertEqual(gnss._int_or_default(value, default=9), 9)


if __name__ == "__main__":
    unittest.main()
