"""第3轮空中端极简化契约测试（t1 验收基准）。

覆盖：
1. 主界面只剩四段：顶部连接栏（雷达USB+GNSS串口+地面站链路）、状态条、B-scan 全屏主视图、底部日志；
   _operator_bar 与 _diagnostics_section（S参数图/A-scan/GNSS地图）彻底移除；
2. AirSettingsDialog 瘦身：校准页与 VNA 扫描参数页移除，仅保留显示/输出/诊断页，objectName 为 air_settings_dialog；
3. 无本地按钮下远程命令 start/stop 仍能正常驱动采集（offscreen 真实 MainWindow）；
4. gnss_map_widget 空中端解引用，源文件完整保留供地面端使用。
"""

from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

try:
    import uav_gpr_qt as air
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
    air = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

from PySide6 import QtCore, QtWidgets

from librevna.librevna_continuous import PersistentLibreVnaBackend
from tests.test_round2_air_ui_compact import _FlowTransport, _wait_until, _wait_until_qt


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class AirMinimalUiTests(unittest.TestCase):
    """第3轮空中端极简 UI 结构与远程驱动测试。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.window = air.MainWindow()
        self.window.remote_enable.setChecked(False)

    def tearDown(self) -> None:
        worker = self.window.worker
        if worker is not None and getattr(worker, "isRunning", lambda: False)():
            worker.stop()
            worker.wait(5000)
        self.window.close()
        self.window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        self.app.processEvents()

    def test_main_window_four_sections_minimal_structure(self) -> None:
        """空中端主界面只剩四段：连接栏、状态条、B-scan主视图、日志区。"""
        central = self.window.centralWidget()
        self.assertIsNotNone(central)

        # 1. 顶部连接栏在
        conn_bar = self.window.findChild(QtWidgets.QWidget, "connection_bar")
        self.assertIsNotNone(conn_bar, "顶部连接栏必须存在且 objectName 为 connection_bar")
        # 包含雷达 USB、GNSS 串口、地面站链路、设置按钮
        self.assertEqual(self.window.test_vna_btn.objectName(), "test_vna_button")
        self.assertEqual(self.window.refresh_btn.objectName(), "refresh_ports_button")
        self.assertEqual(self.window.remote_enable.objectName(), "remote_enable_input")
        self.assertEqual(self.window.remote_host.objectName(), "remote_host_input")
        self.assertEqual(self.window.remote_port.objectName(), "remote_port_input")
        self.assertEqual(self.window.remote_connect.objectName(), "remote_connect_button")
        self.assertEqual(self.window.air_settings_button.objectName(), "air_settings_button")
        self.assertIn(self.window.test_vna_btn, conn_bar.findChildren(QtWidgets.QWidget))
        self.assertIn(self.window.port_combo, conn_bar.findChildren(QtWidgets.QWidget))
        self.assertIn(self.window.remote_connect, conn_bar.findChildren(QtWidgets.QWidget))
        self.assertIn(self.window.air_settings_button, conn_bar.findChildren(QtWidgets.QWidget))

        # 2. 状态条在
        status_group = self.window.findChild(QtWidgets.QWidget, "status_group")
        self.assertIsNotNone(status_group)
        self.assertEqual(self.window.service_state_label.objectName(), "service_state_label")
        self.assertEqual(self.window.continuous_status_label.objectName(), "continuous_status_label")
        self.assertEqual(self.window.last_info.objectName(), "last_info_label")

        # 3. B-scan 主视图在
        bscan_section = self.window.findChild(QtWidgets.QWidget, "bscan_section")
        self.assertIsNotNone(bscan_section)
        self.assertEqual(set(self.window.bscan_views.keys()), {"S11", "S22"})
        self.assertEqual(self.window.bscan_window_slider.objectName(), "bscan_window_slider")

        # 4. 日志区在
        self.assertEqual(self.window.device_log.objectName(), "device_log")

        # 移除断言：操作控制栏与诊断图表区彻底拿掉
        self.assertIsNone(self.window.findChild(QtWidgets.QWidget, "operator_bar"))
        self.assertIsNone(self.window.findChild(QtWidgets.QWidget, "diagnostics_group"))
        self.assertFalse(hasattr(self.window, "_operator_bar"))
        self.assertFalse(hasattr(self.window, "_diagnostics_section"))
        self.assertFalse(hasattr(self.window, "sparam_plot"))
        self.assertFalse(hasattr(self.window, "ascan_plot"))
        self.assertFalse(hasattr(self.window, "gnss_map"))

        # 主界面 centralWidget 不含任何本地任务开始/停止按钮
        central_buttons = set(central.findChildren(QtWidgets.QPushButton))
        self.assertNotIn(self.window.start_btn, central_buttons)
        self.assertNotIn(self.window.stop_btn, central_buttons)
        self.assertNotIn(self.window.single_btn, central_buttons)
        self.assertNotIn(self.window.pause_btn, central_buttons)
        self.assertNotIn(self.window.resume_btn, central_buttons)

    def test_air_settings_dialog_tabs_inventory(self) -> None:
        """AirSettingsDialog 瘦身：校准页/VNA扫描参数页移除，保留显示/输出/诊断。"""
        dialog = self.window.air_settings_dialog
        self.assertIsNotNone(dialog)
        self.assertEqual(dialog.objectName(), "air_settings_dialog")

        tabs = dialog.findChild(QtWidgets.QTabWidget, "air_settings_tabs")
        self.assertIsNotNone(tabs)
        self.assertEqual(tabs.count(), 1)
        self.assertEqual(tabs.tabText(0), "显示 / 输出 / 诊断")

        # 移除的分组
        self.assertIsNone(dialog.findChild(QtWidgets.QGroupBox, "settings_vna_group"))
        self.assertIsNone(dialog.findChild(QtWidgets.QGroupBox, "settings_processing_group"))
        self.assertIsNone(dialog.findChild(QtWidgets.QGroupBox, "settings_calibration_group"))
        self.assertIsNone(dialog.findChild(QtWidgets.QGroupBox, "settings_background_group"))

        # 保留的分组
        self.assertIsNotNone(dialog.findChild(QtWidgets.QGroupBox, "settings_display_group"))
        self.assertIsNotNone(dialog.findChild(QtWidgets.QGroupBox, "settings_output_group"))
        self.assertIsNotNone(dialog.findChild(QtWidgets.QGroupBox, "settings_diagnostics_group"))

        # 控件归属：设备来源开关挂在设置对话框内
        descendants = set(dialog.findChildren(QtWidgets.QWidget))
        self.assertIn(self.window.device_source_combo, descendants)
        self.assertIn(self.window.bscan_cmap, descendants)
        self.assertIn(self.window.output_dir, descendants)

    def test_remote_commands_drive_acquisition_without_local_buttons(self) -> None:
        """无本地按钮下，远程命令链路（start/stop/single）仍完整驱动采集。"""
        tmp = tempfile.TemporaryDirectory()
        try:
            self.window.output_dir.setText(tmp.name)
            self.window.continuous_enabled.setChecked(True)
            self.window.points.setValue(5)
            self.window.start_mhz.setValue(400.0)
            self.window.stop_mhz.setValue(440.0)
            transport = _FlowTransport(5)
            service = air.AcquisitionService(
                backend_factory=lambda: PersistentLibreVnaBackend(transport=transport)
            )
            self.window.acquisition_service = service

            # 1. 远程 start 命令
            self.window.handle_remote_command("start", {"count": 3, "interval_s": 0.0})
            self.assertIsNotNone(self.window.worker)
            self.assertTrue(
                _wait_until_qt(lambda: self.window.bscan_trace_count() >= 3, 10.0, self.app),
                f"远程 start 未产生 trace（当前 {self.window.bscan_trace_count()} 道；日志尾={self.window.device_log.toPlainText()[-200:]!r}）",
            )
            self.assertIn("Trace", self.window.device_log.toPlainText())

            # 2. 远程 stop 命令
            self.window.handle_remote_command("stop", {})
            self.assertTrue(
                _wait_until_qt(
                    lambda: self.window.acquisition_service.state == "idle_waiting"
                    and self.window.worker is None,
                    15.0,
                    self.app,
                ),
                f"远程 stop 后未进入待命或 worker 未清理（当前 state={self.window.acquisition_service.state}, worker={self.window.worker}）",
            )
            self.assertTrue(transport.is_open, "停止后 USB 设备应保持待命连接")

            # 3. 远程 single 命令
            self.window.handle_remote_command("single", {})
            # 第5轮 Q2=A 契约变更：single 是新采集，start_acquisition 会先自动
            # 清空 B-scan（旧预期"累计 >=4"已失效）。清空后 single 恰出 1 道。
            self.assertTrue(
                _wait_until_qt(lambda: self.window.bscan_trace_count() == 1, 10.0, self.app),
                f"远程 single 后画布应恰为 1 道（当前 {self.window.bscan_trace_count()} 道）",
            )
            self.assertTrue(
                _wait_until_qt(
                    lambda: self.window.acquisition_service.state == "idle_waiting"
                    and self.window.worker is None,
                    15.0,
                    self.app,
                ),
                f"远程 single 后未回到待命（当前 state={self.window.acquisition_service.state}）",
            )
        finally:
            tmp.cleanup()

    def test_gnss_map_widget_dereferenced_in_air(self) -> None:
        """gnss_map_widget 在空中端完全解引用，但文件本身保留供地面端使用。"""
        self.assertFalse(hasattr(air, "GnssMapWidget"), "uav_gpr_qt 不应导出 GnssMapWidget")
        code = Path("uav_gpr_qt.py").read_text(encoding="utf-8")
        self.assertNotIn("from gnss_map_widget import GnssMapWidget", code)
        self.assertTrue(Path("gnss_map_widget.py").is_file(), "gnss_map_widget.py 文件必须保留")


if __name__ == "__main__":
    unittest.main()
