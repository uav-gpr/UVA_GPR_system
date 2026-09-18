"""第2轮工作流 C 空中端 UI 精简契约测试（R-C-01…R-C-04、R-C-21）。

验收依据：docs/round2_requirements_and_interfaces.md §5.1（保留/移入清单），
§5.5 R-C-01…R-C-04、R-C-21。全部 offscreen Qt + fake 后端，无真实硬件。
"""

from __future__ import annotations

import os
import struct
import tempfile
import time
import unittest
import unittest.mock
from pathlib import Path

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

try:
    import uav_gpr_qt as air
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover - environment dependent
    air = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

from PySide6 import QtCore, QtWidgets

from librevna.librevna_continuous import PersistentLibreVnaBackend
from librevna.librevna_errors import LibreVnaTimeoutError
from librevna.librevna_protocol import (
    ACK,
    DEVICE_INFO,
    REQUEST_DEVICE_INFO,
    SET_IDLE,
    SWEEP_SETTINGS,
    VNA_DATAPOINT,
    encode_packet,
)


# ----------------------------------------------------------------------
# Fake transport（与 test_round2_resident_service 同构的最小设备替身）
# ----------------------------------------------------------------------


def _device_info_payload(num_ports: int = 2) -> bytes:
    return struct.pack(
        "<HBBBBcQQIIHhhIIBQBH",
        14, 1, 2, 3, 5, b"A",
        100_000_000, 6_000_000_000, 1, 1_000_000, 10_001,
        -3000, 1000, 1, 1_000_000, 200, 12_000_000_000, num_ports, 65_535,
    )


def _point_packet(freq_hz: int, point: int, s11: complex, s22: complex) -> bytes:
    receivers = [
        (0x01, s11),
        (0x02, complex(9.0, 9.0)),
        (0x13, complex(1.0, 0.0)),
        (0x21, complex(7.0, 7.0)),
        (0x22, s22),
        (0x33, complex(1.0, 0.0)),
    ]
    n = len(receivers)
    payload = struct.pack("<QhH", int(freq_hz), -1000, int(point))
    payload += struct.pack("<" + "f" * n, *[v.real for _, v in receivers])
    payload += struct.pack("<" + "f" * n, *[v.imag for _, v in receivers])
    payload += bytes(d for d, _ in receivers)
    return encode_packet(VNA_DATAPOINT, payload)


class _FlowTransport:
    """连续扫频 fake 设备（R-C-21 全流程用）。"""

    def __init__(self, n_points: int = 5) -> None:
        self.n_points = n_points
        self.open_count = 0
        self.close_count = 0
        self.opened = False
        self.point = 0
        self._reply: str | None = None

    def open(self) -> None:
        self.open_count += 1
        self.opened = True

    def close(self) -> None:
        self.close_count += 1
        self.opened = False

    @property
    def is_open(self) -> bool:
        return self.opened

    def write(self, data: bytes) -> None:
        if not self.opened:
            raise LibreVnaTimeoutError("not open")
        packet_type = data[3]
        if packet_type == REQUEST_DEVICE_INFO:
            self._reply = "device_info"
        elif packet_type in (SET_IDLE, SWEEP_SETTINGS):
            self._reply = "ack"
        else:  # pragma: no cover - 防御
            self._reply = None

    def read(self, max_length: int, timeout_ms: int) -> bytes:
        if not self.opened:
            raise LibreVnaTimeoutError("not open")
        if self._reply == "device_info":
            self._reply = None
            return encode_packet(DEVICE_INFO, _device_info_payload())
        if self._reply == "ack":
            self._reply = None
            return encode_packet(ACK)
        point = self.point
        self.point = (self.point + 1) % self.n_points
        return _point_packet(
            400_000_000 + point * 10_000_000, point, complex(0.5, -0.2), complex(0.3, 0.1)
        )


def _wait_until(predicate, timeout_s: float, interval_s: float = 0.01) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return predicate()


def _wait_until_qt(predicate, timeout_s: float, app: QtWidgets.QApplication, interval_s: float = 0.01) -> bool:
    """带 Qt 事件泵的有界等待（跨线程 queued 信号必须 processEvents 才投递）。"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        app.processEvents()
        if predicate():
            return True
        time.sleep(interval_s)
    app.processEvents()
    return predicate()


# 第3轮空中端极简主界面保留清单与移入清单（AirSettingsDialog瘦身版）。
MAIN_WINDOW_KEEP_ATTRS = (
    # 地面链路
    "remote_enable", "remote_host", "remote_port", "remote_connect",
    # GNSS 串口
    "port_combo", "baud_combo", "refresh_btn", "test_gnss_btn",
    # 雷达 USB
    "test_vna_btn",
    # 设置按钮
    "air_settings_button",
    # 状态区
    "continuous_status_label", "last_info", "service_state_label",
    # 主视图（B-scan 不得移除或降级 [冻结]）
    "bscan_views", "bscan_window_slider", "bscan_window_label", "bscan_filter_status",
    # 日志
    "device_log",
)

DIALOG_MOVED_ATTRS = (
    # 显示
    "bscan_cmap", "bscan_aspect", "flat_window_traces", "flat_filter_enabled",
    # 输出与保存
    "output_dir", "browse_out", "autosave",
    # 诊断/设备来源
    "device_source_combo",
)

#: R-C-03 / 第3轮：任务运行中必须逐项禁用的控件（访问 LibreVNA / 改变冻结配置）。
FLIGHT_LOCKED_ATTRS = (
    "test_vna_btn", "device_source_combo", "autosave", "browse_out",
)

#: R-C-03 / 第3轮：任务运行中必须保持可操作的控件。
FLIGHT_SAFE_ATTRS = (
    "bscan_window_slider", "port_combo", "baud_combo", "refresh_btn",
    "air_settings_button",
)


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class AirUiCompactContractTests(unittest.TestCase):
    """R-C-01/R-C-03/R-C-04：保留/移入清单、防误触、非模态。"""

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
        self.app_process_events()

    def app_process_events(self) -> None:
        self.app.processEvents()

    def test_kept_controls_present_on_main_window(self) -> None:
        """R-C-01 / 第3轮：保留清单逐一可解析（连接栏/状态/B-scan/日志四段）。"""
        for attr in MAIN_WINDOW_KEEP_ATTRS:
            self.assertTrue(hasattr(self.window, attr), f"主界面缺少保留控件: {attr}")
        self.assertEqual(set(self.window.bscan_views), {"S11", "S22"})
        self.assertIsNotNone(self.window.findChild(QtWidgets.QWidget, "connection_bar"))
        self.assertIsNone(self.window.findChild(QtWidgets.QWidget, "operator_bar"))
        self.assertIsNone(self.window.findChild(QtWidgets.QWidget, "diagnostics_group"))
        self.assertFalse(hasattr(self.window, "gnss_map"))
        self.assertEqual(self.window.service_state_label.objectName(), "service_state_label")
        self.assertEqual(self.window.bscan_window_slider.objectName(), "bscan_window_slider")
        self.assertEqual(self.window.device_log.objectName(), "device_log")
        self.assertEqual(self.window.test_vna_btn.objectName(), "test_vna_button")
        self.assertEqual(self.window.refresh_btn.objectName(), "refresh_ports_button")
        self.assertEqual(self.window.remote_connect.objectName(), "remote_connect_button")
        self.assertEqual(self.window.air_settings_button.objectName(), "air_settings_button")

    def test_moved_controls_hosted_inside_air_settings_dialog(self) -> None:
        """R-C-01 / 第3轮：移入表控件全部存在于 AirSettingsDialog（瘦身版）。"""
        dialog = self.window.air_settings_dialog
        self.assertIsNotNone(dialog)
        descendants = set(dialog.findChildren(QtWidgets.QWidget))
        for attr in DIALOG_MOVED_ATTRS:
            widget = getattr(self.window, attr)
            self.assertIsNotNone(widget)
            if isinstance(widget, QtWidgets.QWidget):
                self.assertIn(
                    widget, descendants,
                    f"移入控件未挂到 AirSettingsDialog: {attr}",
                )
        # 分组 objectName 可定位（显示/输出/诊断三组，VNA与校准组已移除）。
        for group_name in (
            "settings_display_group", "settings_output_group", "settings_diagnostics_group",
        ):
            found = dialog.findChild(QtWidgets.QGroupBox, group_name)
            self.assertIsNotNone(found, f"对话框缺少分组: {group_name}")
        for removed_group in (
            "settings_vna_group", "settings_processing_group",
            "settings_calibration_group", "settings_background_group",
        ):
            self.assertIsNone(dialog.findChild(QtWidgets.QGroupBox, removed_group), f"对话框不应残留分组: {removed_group}")

    def test_no_control_deleted_full_inventory(self) -> None:
        """R-C-01 / 第3轮：保留+移入全部控件无一删除，内部配置属性完整。"""
        for attr in MAIN_WINDOW_KEEP_ATTRS + DIALOG_MOVED_ATTRS:
            self.assertTrue(hasattr(self.window, attr), f"控件被删除: {attr}")
        # 远程命令链路与后端依赖的内部配置属性仍保留在 window 上
        for internal_attr in (
            "start_mhz", "stop_mhz", "points", "ifbw", "power", "dwell",
            "ifft_points", "time_window", "dewow_window", "dewow_enabled",
            "interval", "count", "continuous_enabled", "start_btn", "stop_btn", "single_btn",
        ):
            self.assertTrue(hasattr(self.window, internal_attr), f"内部配置属性缺失: {internal_attr}")

    def test_flight_lock_disables_every_dangerous_widget(self) -> None:
        """R-C-03 / 第3轮：任务运行中危险控件逐项 isEnabled()==False（连接栏/设置对话框语义）。"""
        self.window._set_task_controls_locked(True)
        try:
            for attr in FLIGHT_LOCKED_ATTRS:
                widget = getattr(self.window, attr)
                self.assertFalse(
                    widget.isEnabled(),
                    f"任务运行中仍可误触: {attr}",
                )
        finally:
            self.window._set_task_controls_locked(False)

    def test_flight_lock_keeps_safe_controls_operable(self) -> None:
        """R-C-03 / 第3轮：B-scan 滑块/GNSS 串口/设置按钮保持可操作。"""
        self.window._set_task_controls_locked(True)
        try:
            for attr in FLIGHT_SAFE_ATTRS:
                widget = getattr(self.window, attr)
                self.assertTrue(widget.isEnabled(), f"安全控件被误锁: {attr}")
        finally:
            self.window._set_task_controls_locked(False)
        # 解锁后危险控件恢复。
        for attr in FLIGHT_LOCKED_ATTRS:
            self.assertTrue(getattr(self.window, attr).isEnabled())

    def test_settings_dialog_is_non_modal(self) -> None:
        """R-C-04：AirSettingsDialog 非模态（show() + setModal(False)）。"""
        dialog = self.window.air_settings_dialog
        self.assertFalse(dialog.isModal())
        dialog.show()
        self.assertTrue(dialog.isVisible())
        self.assertTrue(self.window.isEnabled(), "非模态对话框打开时主界面必须仍可点击")
        dialog.hide()

    def test_production_code_never_execs_the_dialogs(self) -> None:
        """R-C-04：生产代码不得对两个对话框调用 exec()（源码级检查）。"""
        for source_file in ("uav_gpr_qt.py", "uav_gpr_ground_station.py"):
            text = Path(source_file).read_text(encoding="utf-8")
            self.assertNotIn("air_settings_dialog.exec", text)
            self.assertNotIn("calibration_dialog.exec", text)
            self.assertNotIn("settings_dialog.exec(", text)

    def test_service_state_label_reflects_service_state(self) -> None:
        """§5.1 状态区新增服务状态行：state/open_count/待命提示。"""
        service = self.window.acquisition_service
        service._state = "idle_waiting"  # noqa: SLF001 - 直接注入只读展示状态
        service._backend = None  # noqa: SLF001
        self.window._refresh_service_state_label()
        text = self.window.service_state_label.text()
        self.assertIn("idle_waiting", text)
        self.assertIn("USB 待命中", text)
        service._state = "closed"  # noqa: SLF001
        self.window._refresh_service_state_label()
        self.assertIn("closed", self.window.service_state_label.text())

    def test_calibration_conflict_function_is_single_source(self) -> None:
        """R-C-14：互斥判据唯一函数存在且双向可判。"""
        reason = self.window._calibration_acquisition_conflict("启动采集")
        self.assertIsNone(reason)  # 空闲 + 无会话 → 允许
        self.window.worker = unittest.mock.Mock()
        self.window.worker.isRunning.return_value = True
        reason = self.window._calibration_acquisition_conflict("cal_begin")
        self.assertIsNotNone(reason)
        self.assertIn("采集任务运行中", reason)
        self.window.worker = None


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class AirUiFullFlowTests(unittest.TestCase):
    """R-C-21：精简后空中端仍能完成一次完整采集（fake 后端 offscreen）。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.window = air.MainWindow()
        self.window.remote_enable.setChecked(False)
        self.window.continuous_enabled.setChecked(True)
        self.window.count.setValue(3)
        self.window.output_dir.setText(self.tmp.name)
        # 扫频配置对齐 fake 设备（5 点、400–440 MHz，与 _FlowTransport 逐点频率一致）。
        self.window.points.setValue(5)
        self.window.start_mhz.setValue(400.0)
        self.window.stop_mhz.setValue(440.0)
        self.transport = _FlowTransport(5)
        service = air.AcquisitionService(
            backend_factory=lambda: PersistentLibreVnaBackend(transport=self.transport)
        )
        self.window.acquisition_service = service

    def tearDown(self) -> None:
        worker = self.window.worker
        if worker is not None and getattr(worker, "isRunning", lambda: False)():
            worker.stop()
            worker.wait(5000)
        self.window.close()
        self.window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        self.app.processEvents()
        self.tmp.cleanup()

    def test_simplified_ui_completes_full_acquisition_flow(self) -> None:
        """连接→开始→B-scan 出图→停止→待命（精简没砍掉功能）。"""
        # 精简界面：开始按钮可点（连接=remote_enable 关闭即视为本地演练）。
        self.assertTrue(self.window.start_btn.isEnabled())
        self.window.start_acquisition()
        self.assertIsNotNone(self.window.worker)
        self.assertTrue(
            _wait_until_qt(
                lambda: self.window.bscan_trace_count() >= 3, 10.0, self.app
            ),
            f"3 条 trace 未在时限内出图（当前 {self.window.bscan_trace_count()} 道；"
            f"日志尾={self.window.device_log.toPlainText()[-200:]!r}）",
        )
        # B-scan 视图确实收到数据（S11/S22 两通道）。
        for name in ("S11", "S22"):
            self.assertEqual(len(self.window.bscan[name]), 3)
        # 日志可读（device_log 收到 Trace 完成记录）。
        log_text = self.window.device_log.toPlainText()
        self.assertIn("Trace", log_text)
        # 停止（统一服务入口）→ 排空完成 → USB 待命保持。
        self.window.stop_acquisition()
        self.assertTrue(
            _wait_until(
                lambda: self.window.acquisition_service.state == "idle_waiting", 15.0
            ),
            f"停止后未进入待命（state={self.window.acquisition_service.state}）",
        )
        self.assertTrue(self.transport.is_open, "R-A-01：待命期 USB 必须保持打开")
        self.assertEqual(self.transport.close_count, 0, "待命期不得释放 USB")
        # 服务状态行展示待命提示（§5.1 状态区）。
        self.window._refresh_service_state_label()
        self.assertIn("USB 待命中", self.window.service_state_label.text())


if __name__ == "__main__":
    unittest.main()
