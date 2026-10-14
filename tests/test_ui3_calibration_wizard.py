"""Unit and integration tests for UAV-GPR Ground Station CalibrationWizard.

Covers:
- Wizard state machine transitions and step installation prompts
- Command dispatching (cal_begin, cal_capture, cal_finish, cal_activate, cal_cancel)
- Step-by-step progress tracking and quality/result updates
- Recapture flow (selective step recapture)
- Six-step OSL full workflow and single-step Background workflow
- Layout anti-overlap verification (ensuring no button collisions)
"""

from __future__ import annotations

import os
import unittest
from pathlib import Path
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

from PySide6 import QtCore, QtWidgets

import uav_gpr_ground_station as ground


class DummyReceiver(QtCore.QObject):
    status = QtCore.Signal(str)
    command_status = QtCore.Signal(str)
    trace_received = QtCore.Signal(dict, bytes, str)

    def __init__(self) -> None:
        super().__init__()
        self.calls: list[tuple[str, dict | None]] = []

    def send_command(self, action: str, params: dict | None = None) -> None:
        self.calls.append((action, params))


class CalibrationWizardUiTests(unittest.TestCase):
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
            out_dir=Path("./tmp_ground_cal_test"),
            receiver=self.receiver,
            start_receiver=False,
        )
        self.wizard = self.window.calibration_dialog

    def tearDown(self) -> None:
        self.window.close()
        self.window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        self.app.processEvents()

    # ---- 1. 结构与对象命名规范测试 ----

    def test_wizard_object_names_and_structure(self) -> None:
        self.assertEqual(self.wizard.objectName(), "calibration_dialog")
        self.assertIsInstance(self.wizard, ground.CalibrationWizard)
        self.assertIs(ground.CalibrationDialog, ground.CalibrationWizard)

        # 检查关键控件存在性及规范命名
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QGroupBox, "calibration_plan_group"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QComboBox, "calibration_mode_input"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QSpinBox, "calibration_repeats_input"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QDoubleSpinBox, "calibration_start_mhz_input"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QDoubleSpinBox, "calibration_stop_mhz_input"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QSpinBox, "calibration_points_input"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QSpinBox, "calibration_ifbw_input"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QDoubleSpinBox, "calibration_power_input"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QSpinBox, "calibration_dwell_input"))

        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QTableWidget, "calibration_steps_table"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QProgressBar, "calibration_progress_bar"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QLabel, "calibration_instruction_label"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QLabel, "calibration_status_label"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QLabel, "calibration_quality_label"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QLabel, "calibration_result_label"))

        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QPushButton, "calibration_begin_button"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QPushButton, "calibration_capture_button"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QComboBox, "calibration_recapture_combo"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QPushButton, "calibration_recapture_button"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QPushButton, "calibration_finish_button"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QPushButton, "calibration_activate_button"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QPushButton, "calibration_cancel_button"))
        self.assertIsNotNone(self.wizard.findChild(QtWidgets.QPushButton, "calibration_close_button"))

    # ---- 2. 连接状态与开始按钮门控 ----

    def test_connection_state_gate(self) -> None:
        # 初始未连接状态
        self.wizard.set_connected(False)
        self.assertFalse(self.wizard.begin_button.isEnabled())
        self.assertIn("未连接", self.wizard.status_label.text())

        # 连接就绪
        self.wizard.set_connected(True)
        self.assertTrue(self.wizard.begin_button.isEnabled())
        self.assertIn("已连接", self.wizard.status_label.text())

    # ---- 3. 命令派发格式与参数校验 (cal_begin / cal_capture / ...) ----

    def test_begin_calibration_dispatches_cal_begin(self) -> None:
        self.wizard.set_connected(True)
        self.wizard.cal_start.setValue(500.0)
        self.wizard.cal_stop.setValue(3000.0)
        self.wizard.cal_points.setValue(101)
        self.wizard.cal_ifbw.setValue(12000)
        self.wizard.cal_power.setValue(-15.0)
        self.wizard.cal_dwell.setValue(20)
        self.wizard.repeats.setValue(8)

        self.wizard.begin_calibration()
        self.assertEqual(len(self.receiver.calls), 1)
        action, params = self.receiver.calls[0]
        self.assertEqual(action, "cal_begin")
        self.assertIsNotNone(params)
        self.assertTrue(params["cal_id"].startswith("cal_"))
        self.assertEqual(params["mode"], "osl")
        self.assertEqual(params["repeats"], 8)
        cfg = params["config"]
        self.assertEqual(cfg["start_mhz"], 500.0)
        self.assertEqual(cfg["stop_mhz"], 3000.0)
        self.assertEqual(cfg["points"], 101)
        self.assertEqual(cfg["ifbw_hz"], 12000)
        self.assertEqual(cfg["power_dbm"], -15.0)
        self.assertEqual(cfg["dwell_us"], 20)

        # 检查步骤表初始化
        self.assertEqual(self.wizard.steps_table.rowCount(), 6)
        self.assertEqual(self.wizard.steps_table.item(0, 0).text(), "P1_Open")
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "待采集")
        self.assertIn("Port 1", self.wizard.instruction_label.text())

    def test_capture_and_cancel_commands(self) -> None:
        self.wizard.set_connected(True)
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id

        # 点击采集当前步骤（第8轮协议升级：cal_capture 增加显式 recapture 标志与 request_id 令牌）
        self.wizard.capture_next_step()
        self.assertEqual(self.receiver.calls[-1][0], "cal_capture")
        call_params = self.receiver.calls[-1][1]
        self.assertEqual(call_params.get("cal_id"), cal_id)
        self.assertEqual(call_params.get("step"), "P1_Open")
        self.assertFalse(call_params.get("recapture"))
        self.assertTrue(bool(call_params.get("request_id")))
        self.assertFalse(self.wizard.capture_button.isEnabled())

        # 取消会话
        self.wizard.cancel_session()
        self.assertEqual(self.receiver.calls[-1][0], "cal_cancel")
        self.assertEqual(self.receiver.calls[-1][1]["cal_id"], cal_id)

    # ---- 4. 六步 OSL 完整流程与状态机推进 ----

    def test_full_osl_workflow_to_activation(self) -> None:
        self.wizard.set_connected(True)
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id

        # 模拟空中端返回 planned
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "planned",
            "mode": "osl",
            "steps": list(ground.CalibrationWizard.OSL_STEPS),
            "repeats": 4,
        })
        self.assertEqual(self.wizard.steps_table.rowCount(), 6)
        self.assertTrue(self.wizard.capture_button.isEnabled())

        # 依次完成 6 步采集
        for idx, step_name in enumerate(ground.CalibrationWizard.OSL_STEPS):
            self.assertEqual(self.wizard._step_index, idx)
            self.assertIn(step_name, self.wizard.instruction_label.text())

            # 模拟收到单步数据流
            self.window._on_calibration_message({
                "type": "calibration_capture",
                "cal_id": cal_id,
                "step": step_name,
                "index": 2,
                "of": 4,
            })
            self.assertIn("采集中 2/4", self.wizard.steps_table.item(idx, 1).text())

            # 模拟收到单步完成与质量指标
            self.window._on_calibration_message({
                "type": "calibration_capture",
                "cal_id": cal_id,
                "step": step_name,
                "index": 4,
                "of": 4,
            })
            self.window._on_calibration_message({
                "type": "calibration_quality",
                "cal_id": cal_id,
                "step": step_name,
                "repeat_rms": 0.005,
                "max_abs_err": 0.012,
                "sample_count": 4,
                "note": "优",
            })
            # 第8轮协议升级：单步硬件收尾完成回传 step_done
            self.window._on_calibration_message({
                "type": "calibration_result",
                "cal_id": cal_id,
                "status": "step_done",
                "step": step_name,
                "steps_done": idx + 1,
                "total_steps": 6,
            })
            self.assertEqual(self.wizard.steps_table.item(idx, 1).text(), "已完成")
            self.assertIn("重复RMS=5.000e-03", self.wizard.steps_table.item(idx, 3).text())

        # 6 步收齐后，finish_button 变为可用
        self.assertTrue(self.wizard.finish_button.isEnabled())
        self.assertFalse(self.wizard.activate_button.isEnabled())

        # 点击解算
        self.wizard.finish_calibration()
        self.assertEqual(self.receiver.calls[-1][0], "cal_finish")

        # 模拟解算完成回程
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "solved",
            "path": "/data/cal_osl.json",
            "valid_band_hz": [400000000, 4000000000],
            "profile_summary": "OSL 双端口已解算",
        })
        self.assertTrue(self.wizard.activate_button.isEnabled())
        self.assertIn("解算完成", self.wizard.result_label.text())

        # 点击激活
        self.wizard.activate_calibration()
        self.assertEqual(self.receiver.calls[-1][0], "cal_activate")

        # 模拟激活完成
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "activated",
        })
        self.assertIn("校准已激活", self.wizard.status_label.text())

    # ---- 5. 失败重采机制 (Recapture Flow) ----

    def test_recapture_step_flow(self) -> None:
        self.wizard.set_connected(True)
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id

        # 完成第一步 P1_Open
        self.window._on_calibration_message({
            "type": "calibration_capture",
            "cal_id": cal_id,
            "step": "P1_Open",
            "index": 2,
            "of": 2,
        })
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "step_done",
            "step": "P1_Open",
            "steps_done": 1,
            "total_steps": 6,
        })
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "已完成")
        self.assertEqual(self.wizard._step_index, 1)

        # 从重采下拉框选择 P1_Open 进行重采
        self.wizard.recapture_combo.setCurrentText("P1_Open")
        self.assertTrue(self.wizard.recapture_button.isEnabled())
        self.wizard.recapture_selected_step()

        # 验证发送 cal_capture(P1_Open)（第8轮协议升级：显式重采设置 recapture=True 与 request_id）
        self.assertEqual(self.receiver.calls[-1][0], "cal_capture")
        recap_params = self.receiver.calls[-1][1]
        self.assertEqual(recap_params.get("cal_id"), cal_id)
        self.assertEqual(recap_params.get("step"), "P1_Open")
        self.assertTrue(recap_params.get("recapture"))
        self.assertTrue(bool(recap_params.get("request_id")))
        self.assertEqual(self.wizard._step_index, 0)
        self.assertIn("请求重采中", self.wizard.steps_table.item(0, 1).text())

        # 重采完成后状态重新变为已完成
        self.window._on_calibration_message({
            "type": "calibration_capture",
            "cal_id": cal_id,
            "step": "P1_Open",
            "index": 2,
            "of": 2,
        })
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "step_done",
            "step": "P1_Open",
            "steps_done": 1,
            "total_steps": 6,
        })
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "已完成")

    # ---- 6. 空采背景参考模式 (Background Mode) ----

    def test_background_mode_workflow(self) -> None:
        self.wizard.set_connected(True)
        self.wizard.mode_combo.setCurrentIndex(1)  # 选择空采参考
        self.assertIn("空采", self.wizard.instruction_label.text())

        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id

        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "planned",
            "mode": "background",
            "steps": ["Background"],
            "repeats": 10,
        })
        self.assertEqual(self.wizard.steps_table.rowCount(), 1)
        self.assertEqual(self.wizard.steps_table.item(0, 0).text(), "Background")

        # 采集单步
        self.wizard.capture_next_step()
        bg_params = self.receiver.calls[-1][1]
        self.assertEqual(bg_params.get("cal_id"), cal_id)
        self.assertEqual(bg_params.get("step"), "Background")
        self.assertFalse(bg_params.get("recapture"))
        self.assertTrue(bool(bg_params.get("request_id")))

        self.window._on_calibration_message({
            "type": "calibration_capture",
            "cal_id": cal_id,
            "step": "Background",
            "index": 10,
            "of": 10,
        })
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "step_done",
            "step": "Background",
            "steps_done": 1,
            "total_steps": 1,
        })
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "已完成")
        self.assertTrue(self.wizard.finish_button.isEnabled())

    # ---- 7. 布局与抗挤压无重叠校验 ----

    def test_layout_no_button_overlap(self) -> None:
        self.wizard.show()
        for width, height in ((720, 580), (800, 600), (1024, 768)):
            self.wizard.resize(width, height)
            self.app.processEvents()

            buttons = self.wizard.findChildren(QtWidgets.QPushButton)
            self.assertGreaterEqual(len(buttons), 7)

            for i in range(len(buttons)):
                for j in range(i + 1, len(buttons)):
                    b1, b2 = buttons[i], buttons[j]
                    p1 = b1.mapTo(self.wizard, QtCore.QPoint(0, 0))
                    p2 = b2.mapTo(self.wizard, QtCore.QPoint(0, 0))
                    r1 = QtCore.QRect(p1, b1.size())
                    r2 = QtCore.QRect(p2, b2.size())
                    self.assertFalse(
                        r1.intersects(r2),
                        f"按钮 {b1.objectName()} 与 {b2.objectName()} 在尺寸 {width}x{height} 下发生几何重叠！",
                    )


if __name__ == "__main__":
    unittest.main()
