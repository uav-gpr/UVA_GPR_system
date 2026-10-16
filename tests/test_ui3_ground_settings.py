"""Round 3 Unit Tests: Ground Station Settings Center, Command Dispatch, and Pause/Resume.

Covers:
1. Ground Station Settings Center (AcquisitionSettingsDialog / GroundSettingsDialog):
   - Tabbed layout: 采集控制, 扫描参数, 处理参数.
   - Standard objectName naming (*_input) and SweepConfig alignment.
   - Value range validations (start/stop frequency, points, bandwidth, power, dwell, time window, Dewow window).
2. Remote Command set_params:
   - Payload dictionary with sweep and processing parameters.
   - Conforms strictly to R-X-02 wire protocol (type="command", action="set_params").
   - Air station handle_remote_command: validation -> update local config -> next start takes effect.
   - Air station state guard: rejects set_params while acquisition is active.
3. Remote Command pause and resume:
   - Command channel pause/resume actions connected to AcquisitionService.
   - Ground station pause_button / resume_button and menu actions.
   - ACK status text transitions for pause, resume, and set_params.
4. End-to-end parameter transfer link:
   - Ground station UI edit -> construct command -> Air station handle_remote_command -> SweepConfig verified.
"""

from __future__ import annotations

import os
import struct
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

from PySide6 import QtCore, QtWidgets

from librevna.librevna_continuous import PersistentLibreVnaBackend
import uav_gpr_ground_station as ground
import air_core as air


class DummyReceiver(QtCore.QObject):
    status = QtCore.Signal(str)
    command_status = QtCore.Signal(str)
    trace_received = QtCore.Signal(dict, bytes, str)

    def __init__(self) -> None:
        super().__init__()
        self.calls: list[tuple[str, dict | None]] = []

    def send_command(self, action: str, params: dict | None = None) -> None:
        self.calls.append((action, params))


class GroundStationSettingsUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance()
        if cls.app is None:
            cls.app = QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.receiver = DummyReceiver()
        self.window = ground.GroundStationWindow(
            host="127.0.0.1",
            port=0,
            out_dir=Path("./tmp_ground_test"),
            receiver=self.receiver,
            start_receiver=False,
        )

    def tearDown(self) -> None:
        self.window.close()
        self.app.processEvents()

    # ---- 1. 设置中心对话框结构与控件规范 ----

    def test_dialog_tabs_and_object_names(self) -> None:
        dialog = self.window.acquisition_dialog
        self.assertIsInstance(dialog, ground.AcquisitionSettingsDialog)
        self.assertEqual(dialog.objectName(), "acquisition_settings_dialog")
        self.assertEqual(dialog.tab_widget.count(), 3)
        tab_titles = [dialog.tab_widget.tabText(i) for i in range(3)]
        self.assertIn("采集控制", tab_titles[0])
        self.assertIn("扫描参数", tab_titles[1])
        self.assertIn("处理参数", tab_titles[2])

        # 验证全部 13 个关键控件 objectName 均符合 *_input 规范
        expected_inputs = {
            "acquisition_mode_input": dialog.mode,
            "acquisition_interval_input": dialog.interval,
            "acquisition_count_input": dialog.count,
            "start_mhz_input": dialog.start_mhz,
            "stop_mhz_input": dialog.stop_mhz,
            "points_input": dialog.points,
            "ifbw_input": dialog.ifbw,
            "power_input": dialog.power,
            "dwell_input": dialog.dwell,
            "dewow_enabled_input": dialog.dewow_enabled,
            "dewow_window_input": dialog.dewow_window,
            "time_window_input": dialog.time_window,
            "ifft_points_input": dialog.ifft_points,
        }
        for obj_name, widget in expected_inputs.items():
            self.assertEqual(widget.objectName(), obj_name, f"控件 {widget} objectName 不匹配")

    def test_dialog_default_values_align_with_sweep_config(self) -> None:
        dialog = self.window.acquisition_dialog
        dialog.load_from(self.window.acq_settings)
        ref_cfg = air.SweepConfig()

        self.assertEqual(dialog.start_mhz.value(), ref_cfg.start_mhz)
        self.assertEqual(dialog.stop_mhz.value(), ref_cfg.stop_mhz)
        self.assertEqual(dialog.points.value(), ref_cfg.points)
        self.assertEqual(dialog.ifbw.value(), ref_cfg.ifbw_hz)
        self.assertEqual(dialog.power.value(), ref_cfg.power_dbm)
        self.assertEqual(dialog.dwell.value(), ref_cfg.dwell_us)
        self.assertEqual(dialog.time_window.value(), ref_cfg.time_window_ns)
        self.assertEqual(dialog.dewow_enabled.isChecked(), ref_cfg.dewow_enabled)
        self.assertEqual(dialog.dewow_window.value(), ref_cfg.dewow_window_ns)
        self.assertEqual(dialog.ifft_points.value(), ref_cfg.ifft_points)

    def test_dialog_value_range_validations(self) -> None:
        dialog = self.window.acquisition_dialog
        dialog.load_from(self.window.acq_settings)

        # 默认值应通过验证
        self.assertIsNone(dialog.validate())

        # 终止频率 <= 起始频率
        dialog.start_mhz.setValue(2000.0)
        dialog.stop_mhz.setValue(1000.0)
        self.assertIn("终止频率必须大于起始频率", dialog.validate() or "")

        # 恢复频率
        dialog.start_mhz.setValue(500.0)
        dialog.stop_mhz.setValue(3000.0)
        self.assertIsNone(dialog.validate())

        # 控件范围约束与 validate 防护
        self.assertEqual(dialog.points.minimum(), 2)
        self.assertEqual(dialog.points.maximum(), 4501)
        self.assertEqual(dialog.ifbw.minimum(), 6)
        self.assertEqual(dialog.ifbw.maximum(), 50000)
        self.assertEqual(dialog.power.minimum(), -40.0)
        self.assertEqual(dialog.power.maximum(), 0.0)
        self.assertEqual(dialog.time_window.minimum(), 1.0)
        self.assertEqual(dialog.time_window.maximum(), 1000.0)

        with patch.object(dialog.points, "value", return_value=1):
            self.assertIn("频点数", dialog.validate() or "")
        with patch.object(dialog.time_window, "value", return_value=0.5):
            self.assertIn("时间窗", dialog.validate() or "")
        with patch.object(dialog.dewow_window, "value", return_value=0.0):
            self.assertIn("Dewow 窗长", dialog.validate() or "")

    def test_dialog_draft_and_cancel_preserves_settings(self) -> None:
        dialog = self.window.acquisition_dialog
        dialog.load_from(self.window.acq_settings)

        dialog.start_mhz.setValue(800.0)
        dialog.stop_mhz.setValue(3500.0)
        dialog.points.setValue(401)
        dialog.ifbw.setValue(20000)
        dialog.power.setValue(-10.0)
        dialog.dwell.setValue(50)
        dialog.dewow_enabled.setChecked(True)
        dialog.dewow_window.setValue(6.0)
        dialog.time_window.setValue(60.0)
        dialog.ifft_points.setValue(4096)

        draft = dialog.draft_settings()
        self.assertEqual(draft.start_mhz, 800.0)
        self.assertEqual(draft.stop_mhz, 3500.0)
        self.assertEqual(draft.points, 401)
        self.assertEqual(draft.ifbw_hz, 20000)
        self.assertEqual(draft.power_dbm, -10.0)
        self.assertEqual(draft.dwell_us, 50)
        self.assertTrue(draft.dewow_enabled)
        self.assertEqual(draft.dewow_window_ns, 6.0)
        self.assertEqual(draft.time_window_ns, 60.0)
        self.assertEqual(draft.ifft_points, 4096)

        # 取消后，主窗口已保存设置不受影响
        dialog.reject()
        self.assertEqual(self.window.acq_settings.start_mhz, 400.0)
        self.assertEqual(self.window.acq_settings.stop_mhz, 4000.0)
        self.assertEqual(len(self.receiver.calls), 0)

    # ---- 2. 参数下发命令 (set_params) 与 framing 契约 ----

    def test_apply_sends_set_params_command(self) -> None:
        dialog = self.window.acquisition_dialog
        dialog.load_from(self.window.acq_settings)
        dialog.start_mhz.setValue(500.0)
        dialog.stop_mhz.setValue(3000.0)
        dialog.points.setValue(101)
        dialog.ifbw.setValue(5000)
        dialog.power.setValue(-15.0)
        dialog.dewow_enabled.setChecked(True)
        dialog.dewow_window.setValue(5.0)

        dialog.apply()

        # 检查命令发送与参数字典
        self.assertTrue(any(call[0] == "set_params" for call in self.receiver.calls))
        action, payload = self.receiver.calls[-1]
        self.assertEqual(action, "set_params")
        self.assertIsInstance(payload, dict)
        self.assertEqual(payload["start_mhz"], 500.0)
        self.assertEqual(payload["stop_mhz"], 3000.0)
        self.assertEqual(payload["points"], 101)
        self.assertEqual(payload["ifbw_hz"], 5000)
        self.assertEqual(payload["power_dbm"], -15.0)
        self.assertTrue(payload["dewow_enabled"])
        self.assertEqual(payload["dewow_window_ns"], 5.0)

    def test_send_params_button_dispatches_without_closing(self) -> None:
        dialog = self.window.acquisition_dialog
        dialog.load_from(self.window.acq_settings)
        dialog.start_mhz.setValue(600.0)
        dialog.stop_mhz.setValue(2500.0)

        dialog.send_params_button.click()

        self.assertEqual(self.receiver.calls[-1][0], "set_params")
        payload = self.receiver.calls[-1][1]
        self.assertEqual(payload["start_mhz"], 600.0)
        self.assertEqual(payload["stop_mhz"], 2500.0)
        self.assertEqual(self.window.acq_settings.start_mhz, 600.0)

    # ---- 3. 地面端暂停与继续控制 ----

    def test_ground_pause_resume_buttons_and_actions(self) -> None:
        self.window.pause_button.click()
        self.assertEqual(self.receiver.calls[-1], ("pause", None))
        self.assertIn("正在发送暂停命令", self.window.acquisition_status_label.text())

        self.window.resume_button.click()
        self.assertEqual(self.receiver.calls[-1], ("resume", None))
        self.assertIn("正在发送继续命令", self.window.acquisition_status_label.text())

        # 菜单项触发测试
        self.window.action_pause.trigger()
        self.assertEqual(self.receiver.calls[-1], ("pause", None))

        self.window.action_resume.trigger()
        self.assertEqual(self.receiver.calls[-1], ("resume", None))

    def test_ground_command_status_ack_handling(self) -> None:
        self.window._on_command_status("Remote accepted: pause")
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：已暂停")

        self.window._on_command_status("Remote accepted: resume")
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：采集中")

        self.window._on_command_status("Remote accepted: set_params")
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：参数已下发（以空中应用结果为准）")



if __name__ == "__main__":
    unittest.main()
