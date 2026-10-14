"""Unit and integration tests for Round 8 Calibration Ground Station fixes.

Covers R8-1 ~ R8-10 acceptance criteria:
- R8-1: OSL activation unlocks wizard mode and begin_button for background, preserves active OSL.
- R8-2: Solved but unactivated OSL remains honestly unactivated.
- R8-3: Full v2 calibration file import with complete content transmission and confirmation.
- R8-4: Legacy single-port calibration file import handling and display fallback.
- R8-5: Full calibration file export (cal_export_file) writing valid JSON locally.
- R8-6: Bad/oversized file rejection and remote rejection handling without fake success.
- R8-7: Recapture step execution on single click, correct state machine advance and tracking.
- R8-8: Recapturing after solved invalidates old solve while preserving other completed steps.
- R8-9: Non-residual request state on rejection, request_id idempotency, watchdog and remote busy gating.
- R8-10: Disconnect/reconnect handling and OSL replacement invalidating background while clear background preserves OSL.
"""

from __future__ import annotations

import json
import os
import shutil
import tempfile
import unittest
import unittest.mock
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")

from PySide6 import QtCore, QtWidgets, QtTest

import uav_gpr_ground_station as ground
import uav_gpr_qt as air


class DummyReceiver(QtCore.QObject):
    status = QtCore.Signal(str)
    command_status = QtCore.Signal(str)
    trace_received = QtCore.Signal(dict, bytes, str)

    def __init__(self) -> None:
        super().__init__()
        self.calls: list[tuple[str, dict | None]] = []

    def send_command(self, action: str, params: dict | None = None) -> None:
        self.calls.append((action, params))


class Round8CalibrationGroundTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance()
        if cls.app is None:
            cls.app = QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp_dir = tempfile.mkdtemp(prefix="test_r8_ground_")
        self.receiver = DummyReceiver()
        self.window = ground.GroundStationWindow(
            host="127.0.0.1",
            port=0,
            out_dir=Path(self.tmp_dir),
            receiver=self.receiver,
            start_receiver=False,
        )
        self.wizard = self.window.calibration_dialog
        self.wizard.set_connected(True)

    def tearDown(self) -> None:
        self.window.close()
        self.window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        self.app.processEvents()
        shutil.rmtree(self.tmp_dir, ignore_errors=True)

    # -------------------------------------------------------------------------
    # R8-1: 六步 OSL -> 解算 -> 激活 -> 模式解锁 -> 新建背景并保留 OSL
    # -------------------------------------------------------------------------
    def test_r8_1_osl_to_background_workflow(self) -> None:
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id
        self.assertIsNotNone(cal_id)
        self.assertEqual(self.wizard._mode, "osl")

        # 模拟空中端计划就绪
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "planned",
            "mode": "osl",
            "steps": list(self.wizard.OSL_STEPS),
            "repeats": 4,
        })
        self.assertEqual(self.wizard.steps_table.rowCount(), 6)

        # 依次采集 6 步，模拟收到 step_done
        for idx, step_name in enumerate(self.wizard.OSL_STEPS):
            self.assertEqual(self.wizard.current_step_name(), step_name)
            self.wizard.capture_next_step()
            self.assertTrue(self.wizard._capturing)
            self.assertIn("请求采集中", self.wizard.steps_table.item(idx, 1).text())

            # 模拟样本流
            self.window._on_calibration_message({
                "type": "calibration_capture",
                "cal_id": cal_id,
                "step": step_name,
                "index": 4,
                "of": 4,
            })
            # 模拟空中端收尾完成 step_done
            req_id = self.receiver.calls[-1][1].get("request_id")
            self.window._on_calibration_message({
                "type": "calibration_result",
                "cal_id": cal_id,
                "status": "step_done",
                "step": step_name,
                "request_id": req_id,
                "steps_done": idx + 1,
                "total_steps": 6,
            })
            self.assertFalse(self.wizard._capturing)
            self.assertEqual(self.wizard.steps_table.item(idx, 1).text(), "已完成")
            self.assertIn(step_name, self.wizard._completed_steps)

        self.assertEqual(self.wizard._steps_done, 6)
        self.assertTrue(self.wizard.finish_button.isEnabled())

        # 解算
        self.wizard.finish_calibration()
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "solved",
            "path": "_calibration/osl_solved.json",
            "valid_band_hz": [400000000.0, 4000000000.0],
            "profile_summary": "P1+P2 SOLT 双端口求解完成",
        })
        self.assertTrue(self.wizard._solved)
        self.assertTrue(self.wizard.activate_button.isEnabled())

        # 激活
        self.wizard.activate_calibration()
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "activated",
            "mode": "osl",
            "valid_band_hz": [400000000.0, 4000000000.0],
            "ports": ["P1", "P2"],
            "points": 201,
            "active_osl_id": cal_id,
        })

        # 核心断言：会话解除，但生效 OSL 记录并展示！
        self.assertIsNone(self.wizard._cal_id, "激活后临时会话 cal_id 必须解除")
        self.assertTrue(self.wizard.begin_button.isEnabled(), "激活后开始按钮必须解锁可用")
        self.assertTrue(self.wizard.mode_combo.isEnabled(), "激活后模式下拉框必须解锁可用")
        self.assertIsNotNone(self.wizard._active_calibration)
        self.assertIn("P1+P2", self.wizard.active_cal_label.text())
        self.assertIn("400.0-4000.0 MHz", self.wizard.active_cal_label.text())

        # 切换到空采参考（背景），点击开始校准
        self.wizard.mode_combo.setCurrentIndex(1)
        self.assertEqual(self.wizard.mode_combo.currentData(), "background")
        self.wizard.begin_calibration()
        bg_cal_id = self.wizard._cal_id
        self.assertIsNotNone(bg_cal_id)
        self.assertNotEqual(bg_cal_id, cal_id)
        self.assertEqual(self.wizard._mode, "background")

        # 验证开始背景采集并未抹掉已生效的 OSL
        self.assertIsNotNone(self.wizard._active_calibration)
        self.assertIn("P1+P2", self.wizard.active_cal_label.text())

        # 模拟背景采集并激活
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": bg_cal_id,
            "status": "planned",
            "mode": "background",
            "steps": ["Background"],
            "repeats": 4,
        })
        self.wizard.capture_next_step()
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": bg_cal_id,
            "status": "step_done",
            "step": "Background",
            "steps_done": 1,
            "total_steps": 1,
        })
        self.wizard.finish_calibration()
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": bg_cal_id,
            "status": "solved",
            "path": "_calibration/bg.npz",
        })
        self.wizard.activate_calibration()
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": bg_cal_id,
            "status": "activated",
            "mode": "background",
            "points": 201,
        })

        # 验证 OSL 与背景同时处于生效展示状态
        self.assertIsNotNone(self.wizard._active_calibration)
        self.assertIsNotNone(self.wizard._active_background)
        self.assertIn("OSL[P1+P2", self.wizard.active_cal_label.text())
        self.assertIn("背景[201点]", self.wizard.active_cal_label.text())

    # -------------------------------------------------------------------------
    # R8-2: 六步完成已解算但未激活状态诚实展示
    # -------------------------------------------------------------------------
    def test_r8_2_osl_solved_unactivated_honest_state(self) -> None:
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id

        # 模拟步骤完成并求解
        for step in self.wizard.OSL_STEPS:
            self.wizard._completed_steps.add(step)
        self.wizard._steps_done = 6
        self.wizard.finish_calibration()
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "solved",
            "path": "_calibration/solved_unactivated.json",
            "valid_band_hz": [400000000.0, 4000000000.0],
        })

        # 未点击激活前：生效状态仍为未启用
        self.assertTrue(self.wizard._solved)
        self.assertIsNone(self.wizard._active_calibration)
        self.assertIn("未启用", self.wizard.active_cal_label.text())
        self.assertIn("解算完成", self.wizard.result_label.text())
        self.assertIn("请点击“激活校准”使其生效", self.wizard.status_label.text())

    # -------------------------------------------------------------------------
    # R8-3 & R8-4: 校准文件加载、完整内容传输与双端确认
    # -------------------------------------------------------------------------
    def test_r8_3_import_v2_calibration_file(self) -> None:
        # 准备合法的 v2 校准文件
        v2_path = os.path.join(self.tmp_dir, "test_v2_cal.json")
        v2_payload = {
            "format": "uav_gpr_solt_calibration_v2",
            "version": "2.0",
            "created_at": "2026-10-13T18:00:00",
            "sweep_config": {
                "start_hz": 400000000.0,
                "stop_hz": 4000000000.0,
                "points": 201,
            },
            "measurements": {
                "P1_Open": {"s11_re": [0.1] * 201, "s11_im": [0.0] * 201},
                "P1_Short": {"s11_re": [-0.9] * 201, "s11_im": [0.0] * 201},
                "P1_Load": {"s11_re": [0.0] * 201, "s11_im": [0.0] * 201},
                "P2_Open": {"s22_re": [0.1] * 201, "s22_im": [0.0] * 201},
                "P2_Short": {"s22_re": [-0.9] * 201, "s22_im": [0.0] * 201},
                "P2_Load": {"s22_re": [0.0] * 201, "s22_im": [0.0] * 201},
            },
        }
        with open(v2_path, "w", encoding="utf-8") as f:
            json.dump(v2_payload, f)

        # 执行导入
        ok = self.wizard.import_calibration_file(v2_path)
        self.assertTrue(ok)

        # 检查发出的命令不是本地路径，而是完整内容！
        last_action, last_params = self.receiver.calls[-1]
        self.assertEqual(last_action, "cal_import_file")
        self.assertEqual(last_params.get("filename"), "test_v2_cal.json")
        self.assertTrue(last_params.get("activate"))
        self.assertTrue(bool(last_params.get("request_id")))
        sent_content = json.loads(last_params.get("content"))
        self.assertEqual(sent_content.get("format"), "uav_gpr_solt_calibration_v2")

        # 模拟空中端 command_ack 回执
        req_id = last_params.get("request_id")
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_import_file",
            "supported": true if hasattr(self, "_true") else True,
            "result": {
                "ok": True,
                "cal_id": "cal_imp_20261013_180000",
                "status": "activated",
                "valid_band_hz": [400000000.0, 4000000000.0],
                "ports": ["P1", "P2"],
                "format": "uav_gpr_solt_calibration_v2",
                "points": 201,
                "filename": "test_v2_cal.json",
                "request_id": req_id,
            },
        })

        # 验证地面端状态更新
        self.assertIsNotNone(self.wizard._active_calibration)
        self.assertIn("P1+P2", self.wizard.active_cal_label.text())
        self.assertIn("201点", self.wizard.active_cal_label.text())
        self.assertIn("test_v2_cal.json", self.wizard.active_cal_label.text())
        self.assertIn("导入并激活成功", self.wizard.status_label.text())
        self.assertIsNone(self.wizard._cal_id, "导入后会话空闲")
        self.assertTrue(self.wizard.begin_button.isEnabled())

    def test_r8_4_import_legacy_single_port_file(self) -> None:
        p1_path = os.path.join(self.tmp_dir, "legacy_p1.json")
        p1_payload = {
            "Open": {"s11_re": [0.1] * 101, "s11_im": [0.0] * 101},
            "Short": {"s11_re": [-0.9] * 101, "s11_im": [0.0] * 101},
            "Load": {"s11_re": [0.0] * 101, "s11_im": [0.0] * 101},
        }
        with open(p1_path, "w", encoding="utf-8") as f:
            json.dump(p1_payload, f)

        ok = self.wizard.import_calibration_file(p1_path)
        self.assertTrue(ok)

        # 模拟空中端返回单端口结果
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_import_file",
            "result": {
                "ok": True,
                "cal_id": "cal_imp_legacy_p1",
                "status": "activated",
                "valid_band_hz": [500000000.0, 2000000000.0],
                "ports": ["P1"],
                "points": 101,
                "filename": "legacy_p1.json",
            },
        })

        self.assertIn("P1", self.wizard.active_cal_label.text())
        self.assertNotIn("P2", self.wizard.active_cal_label.text(), "旧单端口不能伪造成 P2")

    # -------------------------------------------------------------------------
    # R8-5: 完整校准文件导出 (cal_export_file) 与本地写盘
    # -------------------------------------------------------------------------
    def test_r8_5_export_calibration_file(self) -> None:
        export_target = os.path.join(self.tmp_dir, "exported_cal.json")
        ok = self.wizard.export_calibration_file(export_target)
        self.assertTrue(ok)

        last_action, last_params = self.receiver.calls[-1]
        self.assertEqual(last_action, "cal_export_file")
        self.assertEqual(last_params.get("format"), "uav_gpr_solt_calibration_v2")
        self.assertTrue(bool(last_params.get("request_id")))

        # 模拟空中端回传完整 JSON 内容
        full_content = {
            "format": "uav_gpr_solt_calibration_v2",
            "points": 201,
            "created_at": "2026-10-13T18:05:00",
            "terms": {"p1": [1.0, 2.0]},
        }
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_export_file",
            "result": {
                "ok": True,
                "cal_id": "cal_export_001",
                "filename": "exported_cal.json",
                "content": json.dumps(full_content),
                "format": "uav_gpr_solt_calibration_v2",
                "request_id": last_params.get("request_id"),
            },
        })

        # 验证本地文件被写入且内容完好
        self.assertTrue(os.path.exists(export_target))
        with open(export_target, "r", encoding="utf-8") as f:
            saved_data = json.load(f)
        self.assertEqual(saved_data.get("format"), "uav_gpr_solt_calibration_v2")
        self.assertEqual(saved_data.get("points"), 201)
        self.assertIn("已完整导出至本地", self.wizard.status_label.text())

    # -------------------------------------------------------------------------
    # R8-6: 坏文件/不支持拒绝不假装成功
    # -------------------------------------------------------------------------
    def test_r8_6_bad_file_and_unsupported_rejection(self) -> None:
        # 本地非 JSON 文件
        bad_json_path = os.path.join(self.tmp_dir, "corrupt.json")
        with open(bad_json_path, "w", encoding="utf-8") as f:
            f.write("NOT_A_JSON{{{")

        count_before = len(self.receiver.calls)
        ok = self.wizard.import_calibration_file(bad_json_path)
        self.assertFalse(ok)
        self.assertEqual(len(self.receiver.calls), count_before, "损坏文件不得向空中端发送")
        self.assertIn("读取失败", self.wizard.status_label.text())

        # 缺少 SOLT 测量的 JSON 文件
        missing_meas_path = os.path.join(self.tmp_dir, "missing.json")
        with open(missing_meas_path, "w", encoding="utf-8") as f:
            json.dump({"dummy": 123}, f)
        ok = self.wizard.import_calibration_file(missing_meas_path)
        self.assertFalse(ok)
        self.assertIn("缺少必要的 SOLT 标准件", self.wizard.status_label.text())

        # 模拟空中端明确拒绝 (ok=False)
        valid_path = os.path.join(self.tmp_dir, "dummy_valid.json")
        with open(valid_path, "w", encoding="utf-8") as f:
            json.dump({"format": "uav_gpr_solt_calibration_v2"}, f)
        self.wizard.import_calibration_file(valid_path)
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_import_file",
            "result": {
                "ok": False,
                "error": "频点数量不匹配: 期望 201，实际 101",
            },
        })
        self.assertIn("频点数量不匹配", self.wizard.status_label.text())
        self.assertIsNone(self.wizard._active_calibration, "被拒绝的导入绝对不可生效")

        # 模拟旧空中端不支持 (supported=False)
        self.wizard.import_calibration_file(valid_path)
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_import_file",
            "supported": False,
            "result": {"ok": False, "error": "unknown action: cal_import_file"},
        })
        self.assertIn("空中端版本较低", self.wizard.status_label.text())
        self.assertIsNone(self.wizard._active_calibration)

    # -------------------------------------------------------------------------
    # R8-7 & R8-8: 单击受控重采与求解后重采状态回退
    # -------------------------------------------------------------------------
    def test_r8_7_single_click_recapture_flow(self) -> None:
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id

        # 采集完成 P1_Open
        self.wizard.capture_next_step()
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "step_done",
            "step": "P1_Open",
            "steps_done": 1,
            "total_steps": 6,
        })
        self.assertIn("P1_Open", self.wizard._completed_steps)
        self.assertEqual(self.wizard.recapture_combo.count(), 1)
        self.assertEqual(self.wizard.recapture_combo.currentText(), "P1_Open")

        # 单击“重采该步骤”
        count_before = len(self.receiver.calls)
        self.wizard.recapture_button.click()

        # 核心断言：单次点击直接发出契约重采命令，无需用户再点下一步！
        self.assertEqual(len(self.receiver.calls), count_before + 1)
        action, params = self.receiver.calls[-1]
        self.assertEqual(action, "cal_capture")
        self.assertEqual(params.get("step"), "P1_Open")
        self.assertTrue(params.get("recapture"), "重采必须携带 recapture=True 契约字段")
        self.assertTrue(bool(params.get("request_id")))

        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "请求重采中…")
        self.assertTrue(self.wizard._capturing)

        # 模拟重采成功回程
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "step_done",
            "step": "P1_Open",
            "request_id": params.get("request_id"),
            "steps_done": 1,
            "total_steps": 6,
        })
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "已完成")
        self.assertFalse(self.wizard._capturing)
        # 步骤指示继续指向首个未完成的步骤 P1_Short
        self.assertEqual(self.wizard.current_step_name(), "P1_Short")

    def test_r8_8_recapture_after_solved_invalidates_solution(self) -> None:
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id

        # 全部 6 步已完成
        for s in self.wizard.OSL_STEPS:
            self.wizard._completed_steps.add(s)
        self.wizard._steps_done = 6
        self.wizard._refresh_step_highlight()

        # 解算完成
        self.wizard.finish_calibration()
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "solved",
            "path": "_calibration/osl_old.json",
        })
        self.assertTrue(self.wizard._solved)
        self.assertTrue(self.wizard.activate_button.isEnabled())

        # 用户选择 P1_Short 发起重采
        self.wizard.recapture_combo.setCurrentText("P1_Short")
        self.wizard.recapture_button.click()

        # 核心断言：旧解算结果作废，激活按钮禁用，但其它 5 步数据完好保留！
        self.assertFalse(self.wizard._solved, "重采发起时旧解算结果必须作废")
        self.assertFalse(self.wizard.activate_button.isEnabled())
        self.assertIn("P1_Open", self.wizard._completed_steps)
        self.assertIn("P2_Load", self.wizard._completed_steps)

        # 模拟 P1_Short 重采完成
        recap_req_id = self.receiver.calls[-1][1].get("request_id")
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "step_done",
            "step": "P1_Short",
            "request_id": recap_req_id,
            "steps_done": 6,
            "total_steps": 6,
        })
        # 全部步骤再次齐全，指引引导解算
        self.assertEqual(self.wizard._steps_done, 6)
        self.assertIn("解算并生成", self.wizard.instruction_label.text())
        self.assertTrue(self.wizard.finish_button.isEnabled())

    # -------------------------------------------------------------------------
    # R8-9: 拒绝回退、幂等、Watchdog 与 remote busy
    # -------------------------------------------------------------------------
    def test_r8_9_rejection_restores_state_without_hanging(self) -> None:
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id

        # 首次采集 P1_Open 发生拒绝
        self.wizard.capture_next_step()
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "请求采集中…")
        req_id = self.receiver.calls[-1][1].get("request_id")

        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "rejected",
            "step": "P1_Open",
            "request_id": req_id,
            "reason": "VNA 端口未就绪",
        })

        # 状态行恢复为“待采集”，绝不残留“请求采集中…”
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "待采集")
        self.assertIn("拒绝: VNA 端口未就绪", self.wizard.steps_table.item(0, 3).text())
        self.assertFalse(self.wizard._capturing)
        self.assertTrue(self.wizard.capture_button.isEnabled())

        # 已完成步骤重采被拒绝，状态行恢复为“已完成”
        self.wizard._completed_steps.add("P1_Open")
        self.wizard._update_step_row("P1_Open", "已完成", "")
        self.wizard.recapture_step("P1_Open")
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "请求重采中…")

        recap_req_id = self.receiver.calls[-1][1].get("request_id")
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "rejected",
            "step": "P1_Open",
            "request_id": recap_req_id,
            "reason": "设备忙碌中",
        })
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "已完成")
        self.assertIn("重采拒绝: 设备忙碌中", self.wizard.steps_table.item(0, 3).text())
        self.assertFalse(self.wizard._capturing)

    def test_r8_9_request_id_idempotency(self) -> None:
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id
        self.wizard.capture_next_step()
        req_id = self.receiver.calls[-1][1].get("request_id")

        # 首次收到 step_done
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "step_done",
            "step": "P1_Open",
            "request_id": req_id,
            "steps_done": 1,
            "total_steps": 6,
        })
        self.assertEqual(self.wizard._steps_done, 1)

        # 模拟重复收到的迟到包（相同的 request_id）
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "step_done",
            "step": "P1_Open",
            "request_id": req_id,
            "steps_done": 1,
            "total_steps": 6,
        })
        self.assertEqual(self.wizard._steps_done, 1, "重复 request_id 绝不二次计数或推进")

    def test_r8_9_watchdog_timeout_queries_busy_remote(self) -> None:
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id
        self.wizard.capture_next_step()
        self.assertTrue(self.wizard._capturing)

        # 触发 watchdog 超时
        self.wizard._on_capture_watchdog_timeout()
        self.assertIn("查询远程实际状态", self.wizard.status_label.text())

        # 验证发出 cal_query_active
        self.assertEqual(self.receiver.calls[-1][0], "cal_query_active")

        # A real reply carries the query ID and current session/capture ID.
        busy_token = self.wizard._watchdog_query_token
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_query_active",
            "result": {
                "ok": True,
                "request_id": busy_token,
                "in_progress_session": {
                    "cal_id": cal_id,
                    "active_request_id": self.wizard._pending_request_id,
                    "active": True,
                    "busy": True,
                    "current_step": "P1_Open",
                },
            },
        })
        # 核心断言：空中端仍忙碌时，地面端保持锁定状态，绝不误放开操作导致踩踏！
        self.assertTrue(self.wizard._capturing)
        self.assertFalse(self.wizard.capture_button.isEnabled())
        self.assertIn("空中端仍在忙碌执行", self.wizard.status_label.text())

        # The busy reply is consumed; a later snapshot answers a NEW query.
        self.assertIsNone(self.wizard._watchdog_query_token)
        self.wizard._on_capture_watchdog_timeout()
        wd_token = self.wizard._watchdog_query_token
        self.assertNotEqual(wd_token, busy_token)
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_query_active",
            "request_id": wd_token,
            "result": {
                "ok": True,
                "request_id": wd_token,
                "in_progress_session": {
                    "cal_id": cal_id,
                    "active": True,
                    "busy": False,
                    "steps_status": {"P1_Open": "done"},
                },
            },
        })
        self.assertFalse(self.wizard._capturing)
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "已完成")

    # -------------------------------------------------------------------------
    # R8-10: 掉线重连与 OSL 变更失效背景
    # -------------------------------------------------------------------------
    def test_r8_10_disconnect_and_reconnect(self) -> None:
        self.wizard.begin_calibration()
        self.wizard.capture_next_step()
        self.assertTrue(self.wizard._capturing)

        # 掉线
        self.wizard.set_connected(False)
        self.assertFalse(self.wizard._connected)
        self.assertFalse(self.wizard.capture_button.isEnabled())
        self.assertFalse(self.wizard.begin_button.isEnabled())
        self.assertIn("未连接", self.wizard.status_label.text())
        # 悬挂步骤恢复
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "待采集")

        # 重新连接
        self.wizard.set_connected(True)
        self.assertTrue(self.wizard._connected)
        self.assertIn("已连接", self.wizard.status_label.text())

    def test_r8_10_osl_replacement_invalidates_background(self) -> None:
        # 设置已有生效 OSL 和背景
        self.wizard._active_calibration = {
            "ports": ["P1", "P2"],
            "points": 201,
            "valid_band_hz": [4e8, 4e9],
            "cal_id": "osl_old",
        }
        self.wizard._active_background = {
            "points": 201,
            "cal_id": "bg_old",
        }
        self.wizard._update_active_calibration_display()
        self.assertIn("背景[201点]", self.wizard.active_cal_label.text())

        # 激活一套新 OSL
        self.wizard.begin_calibration()
        new_cal_id = self.wizard._cal_id
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": new_cal_id,
            "status": "activated",
            "mode": "osl",
            "ports": ["P1", "P2"],
            "points": 201,
            "valid_band_hz": [4e8, 4e9],
        })

        # 核心断言：旧背景必须失效，并给出明确提示
        self.assertIsNone(self.wizard._active_background, "OSL 变更时先前背景必须自动失效")
        self.assertIn("先前背景已失效", self.wizard.status_label.text())
        self.assertIn("背景[未启用]", self.wizard.active_cal_label.text())
        self.assertIsNotNone(self.wizard._active_calibration, "新 OSL 必须生效")

        # 恢复背景后，测试清除背景操作 (cal_clear_active target=background)
        self.wizard._active_background = {"points": 201}
        self.wizard._update_active_calibration_display()
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_clear_active",
            "result": {"ok": True, "cleared": "background"},
        })
        self.assertIsNone(self.wizard._active_background)
        self.assertIsNotNone(self.wizard._active_calibration, "清除背景绝不应影响生效 OSL")

    # -------------------------------------------------------------------------
    # FINDING-GND-01 ~ FINDING-GND-07 专项回归
    # -------------------------------------------------------------------------

    def test_r8_finding_gnd_01_mismatched_rejection_ignored(self) -> None:
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id
        self.wizard.capture_next_step()
        self.assertTrue(self.wizard._capturing)
        real_req_id = self.wizard._pending_request_id

        # 模拟迟到或不相符的拒绝回执到达
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "rejected",
            "step": "P1_Open",
            "request_id": "stale_unmatched_req_999",
            "reason": "迟到错误",
        })
        # 必须不打断当前在途采集，不篡改在途状态行
        self.assertTrue(self.wizard._capturing, "不相符的迟到拒绝绝不得打断在途采集")
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "请求采集中…")

        # 真实匹配的拒绝到达
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "rejected",
            "step": "P1_Open",
            "request_id": real_req_id,
            "reason": "真实端口错误",
        })
        self.assertFalse(self.wizard._capturing)
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "待采集")

    def test_r8_finding_gnd_02_stale_query_does_not_reset_capturing(self) -> None:
        self.wizard.begin_calibration()
        self.wizard.capture_next_step()
        self.assertTrue(self.wizard._capturing)

        # 模拟普通查询回执到达（busy=False）
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_query_active",
            "result": {
                "ok": True,
                "in_progress_session": {
                    "active": True,
                    "busy": False,
                },
            },
        })
        # 普通陈旧查询绝不得重置在途采集状态
        self.assertTrue(self.wizard._capturing, "陈旧查询回执绝不得强行重置 _capturing")
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "请求采集中…")

        # 触发看门狗超时后的查询才能在非忙时解锁
        self.wizard._on_capture_watchdog_timeout()
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_query_active",
            "result": {
                "ok": True,
                "request_id": self.wizard._watchdog_query_token,
                "in_progress_session": {
                    "active": True,
                    "busy": False,
                },
            },
        })
        self.assertFalse(self.wizard._capturing)
        self.assertEqual(self.wizard.steps_table.item(0, 1).text(), "待采集")

    def test_r8_finding_gnd_03_concurrent_exports_mapped_by_request_id(self) -> None:
        path1 = os.path.join(self.tmp_dir, "exp_file_1.json")
        path2 = os.path.join(self.tmp_dir, "exp_file_2.json")

        self.wizard.export_calibration_file(path1)
        req1 = self.receiver.calls[-1][1]["request_id"]

        self.wizard.export_calibration_file(path2)
        req2 = self.receiver.calls[-1][1]["request_id"]

        self.assertIn(req1, self.wizard._pending_export_paths)
        self.assertIn(req2, self.wizard._pending_export_paths)

        # 先返回第 2 个文件的回执
        content2 = json.dumps({"format": "uav_gpr_solt_calibration_v2", "file_num": 2})
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_export_file",
            "result": {
                "ok": True,
                "request_id": req2,
                "content": content2,
                "filename": "exp_file_2.json",
            },
        })

        # 再返回第 1 个文件的回执
        content1 = json.dumps({"format": "uav_gpr_solt_calibration_v2", "file_num": 1})
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_export_file",
            "result": {
                "ok": True,
                "request_id": req1,
                "content": content1,
                "filename": "exp_file_1.json",
            },
        })

        # 验证两个文件各自落盘，无串单无覆盖
        self.assertTrue(os.path.exists(path1))
        self.assertTrue(os.path.exists(path2))
        with open(path1, "r", encoding="utf-8") as f:
            data1 = json.load(f)
        with open(path2, "r", encoding="utf-8") as f:
            data2 = json.load(f)
        self.assertEqual(data1["file_num"], 1)
        self.assertEqual(data2["file_num"], 2)

    def test_r8_finding_gnd_04_recapture_rejection_restores_first_uncompleted(self) -> None:
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id

        # 完成 P1_Open 与 P1_Short
        self.wizard._completed_steps.add("P1_Open")
        self.wizard._update_step_row("P1_Open", "已完成", "")
        self.wizard._completed_steps.add("P1_Short")
        self.wizard._update_step_row("P1_Short", "已完成", "")
        self.wizard._steps_done = 2
        self.wizard._step_index = 2  # 指向 P1_Load
        self.wizard._refresh_step_highlight()
        self.wizard._set_session_controls()

        # 用户选择重采 P1_Open
        self.wizard.recapture_combo.setCurrentText("P1_Open")
        self.assertTrue(self.wizard.recapture_button.isEnabled())
        self.wizard.recapture_button.click()
        self.assertEqual(self.wizard._step_index, 0)
        recap_req_id = self.receiver.calls[-1][1]["request_id"]

        # 空中端拒绝本次重采
        self.window._on_calibration_message({
            "type": "calibration_result",
            "cal_id": cal_id,
            "status": "rejected",
            "step": "P1_Open",
            "request_id": recap_req_id,
            "reason": "校准件未接触",
        })

        # 核心断言：_step_index 必须重新纠正为首个未完成步骤 P1_Load (索引 2)
        self.assertEqual(self.wizard._step_index, 2, "重采被拒必须恢复首个未完成步骤索引")
        self.assertEqual(self.wizard.current_step_name(), "P1_Load")
        self.assertIn("P1_Load", self.wizard.instruction_label.text())

    def test_r8_finding_gnd_05_legacy_fallback_margin(self) -> None:
        self.wizard.begin_calibration()
        self.assertIsNone(self.wizard._air_supports_step_done)

        # 模拟末样本到达并触发单次退避超时
        self.wizard._schedule_legacy_fallback("P1_Open")
        self.wizard._on_legacy_fallback_timeout()

        # 单次超时不永久锁定为 False
        self.assertNotEqual(self.wizard._air_supports_step_done, False, "单次抖动超时不得过早锁定旧端模式")

    def test_r8_finding_gnd_06_clear_active_target_fallback(self) -> None:
        self.wizard._active_calibration = {"cal_id": "osl_1", "ports": ["P1"]}
        self.wizard._active_background = {"points": 100}

        # 空中端回传 target 字段而非 cleared 字段
        self.window._on_command_result({
            "type": "command_ack",
            "action": "cal_clear_active",
            "result": {"ok": True, "target": "background"},
        })
        self.assertIsNone(self.wizard._active_background)
        self.assertIsNotNone(self.wizard._active_calibration, "只清背景绝不得误清 OSL")

    def test_r8_finding_gnd_07_real_button_clicks_with_qfiledialog(self) -> None:
        # 准备一个合法的测试文件
        v2_path = os.path.join(self.tmp_dir, "btn_test_v2.json")
        with open(v2_path, "w", encoding="utf-8") as f:
            json.dump({"format": "uav_gpr_solt_calibration_v2", "measurements": {"Open": {}}}, f)

        # 真实模拟用户点击 import_button 按钮，patch 文件选择框
        original_get_open = QtWidgets.QFileDialog.getOpenFileName
        try:
            QtWidgets.QFileDialog.getOpenFileName = lambda *args, **kwargs: (v2_path, "JSON (*.json)")
            count_before = len(self.receiver.calls)
            self.wizard.import_button.click()
            self.assertEqual(len(self.receiver.calls), count_before + 1)
            self.assertEqual(self.receiver.calls[-1][0], "cal_import_file")
        finally:
            QtWidgets.QFileDialog.getOpenFileName = original_get_open

        # 真实模拟用户点击 export_button 按钮，patch 保存文件框
        export_path = os.path.join(self.tmp_dir, "btn_test_export.json")
        original_get_save = QtWidgets.QFileDialog.getSaveFileName
        try:
            QtWidgets.QFileDialog.getSaveFileName = lambda *args, **kwargs: (export_path, "JSON (*.json)")
            count_before = len(self.receiver.calls)
            self.wizard.export_button.click()
            self.assertEqual(len(self.receiver.calls), count_before + 1)
            self.assertEqual(self.receiver.calls[-1][0], "cal_export_file")
        finally:
            QtWidgets.QFileDialog.getSaveFileName = original_get_save

    def test_r8_remote_sender_ack_correlation(self) -> None:
        """验证 RemoteSender command_ack 窄关联回带与地面解析一致（成功/拒绝/异常/None）。"""
        captured: list[dict] = []

        def fake_send(sock, frame, payload=b""):
            captured.append(frame)

        # 1. 成功回调
        sender_ok = air.RemoteSender(
            "127.0.0.1", 1, "run_ack", lambda m: None,
            command_callback=lambda action, params: {"ok": True, "status": "accepted"}
        )
        sender_ok._sock = object()
        with unittest.mock.patch.object(air, "send_remote_frame", side_effect=fake_send):
            sender_ok._handle_command({
                "type": "command", "action": "cal_capture",
                "params": {"cal_id": "cal_99", "step": "P1_Open", "request_id": "req_ack_01"}
            })
        self.assertEqual(len(captured), 1)
        ack_ok = captured[-1]
        self.assertEqual(ack_ok["action"], "cal_capture")
        self.assertEqual(ack_ok["request_id"], "req_ack_01")
        self.assertEqual(ack_ok["cal_id"], "cal_99")
        self.assertEqual(ack_ok["step"], "P1_Open")
        self.assertEqual(ack_ok["result"]["request_id"], "req_ack_01")

        # 2. 拒绝回调（与地面校验结合）
        captured.clear()
        sender_rej = air.RemoteSender(
            "127.0.0.1", 1, "run_ack", lambda m: None,
            command_callback=lambda action, params: {"ok": False, "error": "device busy"}
        )
        sender_rej._sock = object()
        with unittest.mock.patch.object(air, "send_remote_frame", side_effect=fake_send):
            sender_rej._handle_command({
                "type": "command", "action": "cal_capture",
                "params": {"cal_id": "cal_99", "step": "P1_Open", "request_id": "req_ack_02"}
            })
        ack_rej = captured[-1]
        self.assertEqual(ack_rej["request_id"], "req_ack_02")
        self.assertEqual(ack_rej["result"]["ok"], False)
        self.assertEqual(ack_rej["result"]["request_id"], "req_ack_02")

        # 地面端消费该回执
        self.wizard.begin_calibration()
        self.wizard.capture_next_step()
        self.wizard._pending_request_id = "req_ack_02"
        self.window._on_command_result(ack_rej)
        self.assertFalse(self.wizard._capturing)
        self.assertIn("device busy", self.wizard.status_label.text())

        # 3. 异常回调
        captured.clear()
        def fail_callback(a, p):
            raise RuntimeError("hardware exception")
        sender_err = air.RemoteSender("127.0.0.1", 1, "run_ack", lambda m: None, command_callback=fail_callback)
        sender_err._sock = object()
        with unittest.mock.patch.object(air, "send_remote_frame", side_effect=fake_send):
            sender_err._handle_command({
                "type": "command", "action": "cal_capture",
                "params": {"cal_id": "cal_99", "step": "P1_Open", "request_id": "req_ack_03"}
            })
        ack_err = captured[-1]
        self.assertFalse(ack_err["supported"])
        self.assertFalse(ack_err["result"]["ok"])
        self.assertEqual(ack_err["request_id"], "req_ack_03")

        # 4. None 回调（旧 GUI / 纯 ack）
        captured.clear()
        sender_none = air.RemoteSender("127.0.0.1", 1, "run_ack", lambda m: None, command_callback=lambda a, p: None)
        sender_none._sock = object()
        with unittest.mock.patch.object(air, "send_remote_frame", side_effect=fake_send):
            sender_none._handle_command({
                "type": "command", "action": "cal_capture",
                "params": {"cal_id": "cal_99", "step": "P1_Open", "request_id": "req_ack_04"}
            })
        ack_none = captured[-1]
        self.assertNotIn("result", ack_none)
        self.assertNotIn("supported", ack_none)
        self.assertEqual(ack_none["request_id"], "req_ack_04")

    # Captain-authorized closure: GND-12/13/14. These are UI state tests;
    # the independent E2E module remains owned by verifier and is not changed.
    def test_captain_unknown_and_modern_peers_never_timeout_complete(self) -> None:
        self.wizard.begin_calibration()
        self.wizard.capture_next_step()
        req_id = self.wizard._pending_request_id
        for capability in (None, True):
            with self.subTest(capability=capability):
                self.wizard._air_supports_step_done = capability
                self.window._on_calibration_message({
                    "type": "calibration_capture", "cal_id": self.wizard._cal_id,
                    "step": "P1_Open", "index": 16, "of": 16,
                    "request_id": req_id,
                })
                # The receive path already waits; the fallback helper and an
                # already-queued timeout must obey the same capability guard.
                self.wizard._schedule_legacy_fallback("P1_Open")
                QtTest.QTest.qWait(650)
                # Also exercise an already-queued timeout after cancellation.
                self.wizard._on_legacy_fallback_timeout()
                self.assertTrue(self.wizard._capturing)
                self.assertEqual(self.wizard._pending_request_id, req_id)
                self.assertNotIn("P1_Open", self.wizard._completed_steps)
                self.assertFalse(self.wizard.capture_button.isEnabled())
                self.assertFalse(self.wizard._legacy_fallback_timer.isActive())

    def test_captain_confirmed_legacy_peer_completes_and_retires_query(self) -> None:
        self.wizard.begin_calibration()
        self.wizard._air_supports_step_done = False
        self.wizard.capture_next_step()
        self.wizard._on_capture_watchdog_timeout()
        self.assertTrue(self.wizard._watchdog_query_token)
        self.window._on_calibration_message({
            "type": "calibration_capture", "cal_id": self.wizard._cal_id,
            "step": "P1_Open", "index": 16, "of": 16,
            "request_id": self.wizard._pending_request_id,
        })
        self.assertFalse(self.wizard._capturing)
        self.assertIn("P1_Open", self.wizard._completed_steps)
        self.assertEqual(self.wizard.current_step_name(), "P1_Short")
        self.assertIsNone(self.wizard._watchdog_query_token)
        self.assertFalse(self.wizard._watchdog_query_pending)

    def test_captain_old_watchdog_reply_cannot_touch_next_capture(self) -> None:
        self.wizard.begin_calibration()
        self.wizard._air_supports_step_done = True
        cal_id = self.wizard._cal_id
        self.wizard.capture_next_step()
        first_req = self.wizard._pending_request_id
        self.wizard._on_capture_watchdog_timeout()
        old_token = self.wizard._watchdog_query_token
        self.window._on_calibration_message({
            "type": "calibration_result", "status": "step_done",
            "cal_id": cal_id, "step": "P1_Open", "request_id": first_req,
        })
        self.assertIsNone(self.wizard._watchdog_query_token)
        self.wizard.capture_next_step()
        second_req = self.wizard._pending_request_id
        self.wizard._on_capture_watchdog_timeout()
        current_token = self.wizard._watchdog_query_token
        row_before = self.wizard.steps_table.item(1, 1).text()
        self.window._on_command_result({
            "type": "command_ack", "action": "cal_query_active",
            "result": {
                "ok": True, "request_id": old_token,
                "in_progress_session": {
                    "cal_id": cal_id, "active": True, "busy": False,
                    "steps_status": {"P1_Open": "done", "P1_Short": "pending"},
                },
            },
        })
        self.assertTrue(self.wizard._capturing)
        self.assertEqual(self.wizard._pending_request_id, second_req)
        self.assertEqual(self.wizard._watchdog_query_token, current_token)
        self.assertTrue(self.wizard._watchdog_query_pending)
        self.assertEqual(self.wizard.steps_table.item(1, 1).text(), row_before)
        # A response to the CURRENT query still recovers an ended task.
        self.window._on_command_result({
            "type": "command_ack", "action": "cal_query_active",
            "result": {
                "ok": True, "request_id": current_token,
                "in_progress_session": {
                    "cal_id": cal_id, "active": True, "busy": False,
                    "steps_status": {"P1_Open": "done", "P1_Short": "done"},
                },
            },
        })
        self.assertFalse(self.wizard._capturing)
        self.assertIsNone(self.wizard._watchdog_query_token)
        self.assertFalse(self.wizard._watchdog_query_pending)
        self.assertEqual(self.wizard.current_step_name(), "P1_Load")

    def test_captain_query_context_retires_on_recapture_and_session_boundaries(self) -> None:
        self.wizard.begin_calibration()
        self.wizard.capture_next_step()
        self.wizard._on_capture_watchdog_timeout()
        self.wizard._complete_step("P1_Open")
        # Simulate an obsolete pending query at the boundary of a new request.
        self.wizard._watchdog_query_token = "obsolete"
        self.wizard._watchdog_query_pending = True
        self.wizard.recapture_step("P1_Open")
        self.assertIsNone(self.wizard._watchdog_query_token)
        self.assertFalse(self.wizard._watchdog_query_pending)
        self.wizard._on_capture_watchdog_timeout()
        self.wizard.set_connected(False)
        self.assertIsNone(self.wizard._watchdog_query_token)
        self.assertFalse(self.wizard._watchdog_query_pending)
        self.wizard.set_connected(True)
        self.wizard.cancel_session()
        self.wizard._watchdog_query_token = "obsolete_session"
        self.wizard._watchdog_query_pending = True
        self.wizard.begin_calibration()
        self.assertIsNone(self.wizard._watchdog_query_token)
        self.assertFalse(self.wizard._watchdog_query_pending)

    def test_captain_unknown_and_duplicate_export_replies_have_no_side_effects(self) -> None:
        target = os.path.join(self.tmp_dir, "known_export.json")
        unwanted = os.path.join(self.tmp_dir, "unsolicited_export.json")
        content = json.dumps({"format": "uav_gpr_solt_calibration_v2", "marker": "known"})
        self.wizard.export_calibration_file(target)
        req_id = self.receiver.calls[-1][1]["request_id"]
        def response(request_id, body=content):
            return {"type": "command_ack", "action": "cal_export_file", "result": {
                "ok": True, "request_id": request_id, "content": body, "filename": "cal.json",
            }}
        # Real Qt was initialized offscreen; only the application branch is
        # switched so the patched chooser proves it must NOT be invoked.
        with unittest.mock.patch.dict(os.environ, {"QT_QPA_PLATFORM": "windows"}), \
                unittest.mock.patch.object(QtWidgets.QFileDialog, "getSaveFileName",
                                           return_value=(unwanted, "JSON (*.json)")) as chooser:
            self.window._on_command_result(response("unknown"))
            chooser.assert_not_called()
            self.assertIn(req_id, self.wizard._pending_export_paths)
            self.assertFalse(os.path.exists(unwanted))
            self.window._on_command_result(response(req_id))
            with open(target, "r", encoding="utf-8") as f:
                first_contents = f.read()
            self.window._on_command_result(response(req_id, json.dumps({"marker": "duplicate"})))
            self.window._on_command_result(response("unknown_after_completion"))
            chooser.assert_not_called()
            with open(target, "r", encoding="utf-8") as f:
                self.assertEqual(f.read(), first_contents)
            self.assertFalse(os.path.exists(unwanted))

    def test_captain_known_export_without_path_uses_only_its_chooser(self) -> None:
        chosen = os.path.join(self.tmp_dir, "chosen.json")
        explicit = os.path.join(self.tmp_dir, "explicit.json")
        self.wizard.export_calibration_file()
        req_a = self.receiver.calls[-1][1]["request_id"]
        self.wizard.export_calibration_file(explicit)
        req_b = self.receiver.calls[-1][1]["request_id"]
        def response(request_id, marker):
            return {"type": "command_ack", "action": "cal_export_file", "result": {
                "ok": True, "request_id": request_id,
                "content": json.dumps({"format": "uav_gpr_solt_calibration_v2", "marker": marker}),
                "filename": "cal.json",
            }}
        with unittest.mock.patch.object(QtWidgets.QFileDialog, "getSaveFileName",
                                       return_value=(chosen, "JSON (*.json)")) as chooser:
            self.window._on_command_result(response(req_b, "B"))
            chooser.assert_not_called()
            self.assertIn(req_a, self.wizard._pending_export_paths)
            self.window._on_command_result(response(req_a, "A"))
            chooser.assert_called_once()
            self.window._on_command_result(response(req_a, "late"))
            chooser.assert_called_once()
        with open(chosen, "r", encoding="utf-8") as f:
            self.assertEqual(json.load(f)["marker"], "A")
        with open(explicit, "r", encoding="utf-8") as f:
            self.assertEqual(json.load(f)["marker"], "B")


if __name__ == "__main__":
    unittest.main()
