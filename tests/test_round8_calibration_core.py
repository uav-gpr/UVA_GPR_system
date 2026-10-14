#!/usr/bin/env python3
"""第8轮核心校准与背景测试套件 (tests/test_round8_calibration_core.py)。

覆盖验收矩阵 R8-1 ~ R8-11 中空中黑匣子与共享校准核心契约：
1. 受控选择重采 (recapture)、步进保留、旧解算失效、原始采样证据保留与防重采守卫；
2. 权威可继续回执 (step_done)、服务收尾、request_id 幂等、并发与状态机守卫；
3. OSL 激活后背景采集、解耦状态管理、新 OSL 使旧背景失效、单通道不向 S22 广播；
4. 真实 v2 (solt_2port_20260807_202342.json) 与旧 OSL 导入/导出、双端口/Through 独立恢复；
5. 文件校验、原子保存、持久化回写与服务自启动重启恢复、异常保留旧校准；
6. 带外频率拒绝与清除校准定向控制 (clear_active)。
"""

import json
import math
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np

import calibration_service as cal
import uav_gpr_air_service as airsvc
from librevna.librevna_usb_s11 import OSLCalibration, OSLPoint


class _MockStepCapturer:
    """提供可预测复数测试数据的 Capturer Proxy。"""

    def __init__(self, n_points: int = 5, freq_min: float = 400e6, freq_max: float = 4000e6) -> None:
        self.frequencies = np.linspace(freq_min, freq_max, n_points, dtype=np.float64)
        self.n_points = n_points
        self.call_count: dict[str, int] = {}
        self.closed_steps: list[str] = []

    def capture_one(self, step: cal.CalibrationStep, index: int) -> tuple[np.ndarray, np.ndarray]:
        self.call_count[step.name] = self.call_count.get(step.name, 0) + 1
        offset = self.call_count[step.name] * 0.01
        if "Open" in step.name:
            v1, v2 = 0.95 + offset, 0.95 + offset
        elif "Short" in step.name:
            v1, v2 = -0.95 + offset, -0.95 + offset
        elif "Load" in step.name:
            v1, v2 = 0.01 + offset, 0.01 + offset
        else:
            v1, v2 = 0.2 + offset, 0.2 + offset
        s11 = np.ones(self.n_points, dtype=np.complex128) * complex(v1, 0.02)
        s22 = np.ones(self.n_points, dtype=np.complex128) * complex(v2, 0.02)
        raw = np.stack([s11, s22], axis=0)
        return self.frequencies, raw

    def close_current(self, step_name: str) -> None:
        self.closed_steps.append(step_name)


class Round8CalibrationCoreTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp_dir = tempfile.mkdtemp(prefix="test_round8_cal_")
        self.tmp_path = Path(self.tmp_dir)

    def tearDown(self) -> None:
        shutil.rmtree(self.tmp_dir, ignore_errors=True)

    # -------------------------------------------------------------------------
    # 1. 显式受控重采机制 (Acceptance 1: R8-1, R8-2)
    # -------------------------------------------------------------------------

    def test_recapture_p1_open_and_continue(self) -> None:
        """显式重采 P1_Open 成功且可继续 P1_Short；目标均值只来自最新采样，历史证据保留。"""
        plan = cal.build_osl_plan("test_cal_recapture", 2, {})
        capturer = _MockStepCapturer(n_points=5)
        session = cal.CalibrationSession(plan, capturer, capture_dir=self.tmp_path)

        # 1. 首次采集 P1_Open
        res1 = session.capture("P1_Open", recapture=False)
        self.assertEqual(res1["step"], "P1_Open")
        self.assertEqual(session.step_progress("P1_Open").state, "done")
        self.assertEqual(session.step_progress("P1_Open").attempt, 1)
        first_mean = session.step_progress("P1_Open").mean.copy()
        self.assertEqual(len(session.step_progress("P1_Open").captures), 2)

        # 检查首轮落盘文件
        f1 = self.tmp_path / "capture_P1_Open_1.npz"
        f2 = self.tmp_path / "capture_P1_Open_2.npz"
        self.assertTrue(f1.exists())
        self.assertTrue(f2.exists())

        # 2. 正常情况下下一待采步为 P1_Short
        self.assertEqual(session.next_pending_step().name, "P1_Short")

        # 若普通采集 P1_Open (recapture=False)，应被拒绝（已完成）
        with self.assertRaises(RuntimeError) as ctx:
            session.capture("P1_Open", recapture=False)
        self.assertIn("已完成", str(ctx.exception))

        # 3. 显式重采 P1_Open (recapture=True)
        res_re = session.capture("P1_Open", recapture=True)
        self.assertEqual(res_re["step"], "P1_Open")
        progress = session.step_progress("P1_Open")
        self.assertEqual(progress.state, "done")
        self.assertEqual(progress.attempt, 2)
        # 目标均值只来自最新有效采样，与首轮均值不同
        self.assertFalse(np.allclose(progress.mean, first_mean))
        # 历次原始证据归档保留，最新 captures 仅有 2 条
        self.assertEqual(len(progress.captures), 2)
        self.assertEqual(len(progress.history_captures), 1)
        self.assertEqual(len(progress.history_captures[0]), 2)

        # 重采批次落盘包含 _r2_ 文件，且首轮文件绝对未被覆盖
        rf1 = self.tmp_path / "capture_P1_Open_r2_1.npz"
        rf2 = self.tmp_path / "capture_P1_Open_r2_2.npz"
        self.assertTrue(rf1.exists())
        self.assertTrue(rf2.exists())
        self.assertTrue(f1.exists(), "首轮原始采样证据必须保留，严禁静默覆盖删除")
        self.assertTrue(f2.exists())

        # 4. 重采后仍可继续 P1_Short
        self.assertEqual(session.next_pending_step().name, "P1_Short")
        res_short = session.capture("P1_Short", recapture=False)
        self.assertEqual(res_short["step"], "P1_Short")
        self.assertEqual(session.step_progress("P1_Short").state, "done")

    def test_recapture_after_solved_invalidates_solution(self) -> None:
        """已解算 (solved) 后重采某步：旧解算失效，状态回退到 in_progress，后续步骤保留。"""
        plan = cal.build_osl_plan("test_cal_solved_re", 1, {})
        capturer = _MockStepCapturer(n_points=5)
        session = cal.CalibrationSession(plan, capturer, capture_dir=self.tmp_path)

        # 采集所有 6 个步骤
        for step in plan.steps:
            session.capture(step.name, recapture=False)

        # 解算
        solved = session.solve()
        self.assertEqual(session.state, cal.CalibrationSession.STATE_SOLVED)
        self.assertIsNotNone(session.solved_result())

        # 在 solved 状态下未采集步骤重采抛错，普通采集抛错
        with self.assertRaises(RuntimeError):
            session.capture("P1_Open", recapture=False)

        # 显式重采 P1_Load
        session.capture("P1_Load", recapture=True)
        # 状态回退到 in_progress，旧解算清空
        self.assertEqual(session.state, cal.CalibrationSession.STATE_IN_PROGRESS)
        self.assertIsNone(session.solved_result())

        # 其余步骤仍为 done 状态
        for step in plan.steps:
            self.assertEqual(session.step_progress(step.name).state, "done")

        # 重新 solve 成功
        new_solved = session.solve()
        self.assertEqual(session.state, cal.CalibrationSession.STATE_SOLVED)
        self.assertIsNotNone(new_solved)

    def test_recapture_guards_and_terminal_states(self) -> None:
        """已激活/失败/取消状态严守卫：不可进行重采或跳步重采。"""
        plan = cal.build_osl_plan("test_cal_guards", 1, {})
        capturer = _MockStepCapturer(n_points=5)
        session = cal.CalibrationSession(plan, capturer, capture_dir=self.tmp_path)

        # 未进行首次采样的后续步骤不能直接重采
        with self.assertRaises(RuntimeError) as ctx:
            session.capture("P1_Short", recapture=True)
        self.assertIn("尚未进行首次采集", str(ctx.exception))

        # 完成并激活
        for step in plan.steps:
            session.capture(step.name)
        session.solve()
        session.activate()
        self.assertEqual(session.state, cal.CalibrationSession.STATE_ACTIVATED)

        # 已激活不可再重采
        with self.assertRaises(RuntimeError) as ctx:
            session.capture("P1_Open", recapture=True)
        self.assertIn("已激活", str(ctx.exception))

        # 取消会话守卫
        session2 = cal.CalibrationSession(plan, capturer, capture_dir=self.tmp_path / "sub2")
        session2.cancel("测试取消")
        with self.assertRaises(RuntimeError):
            session2.capture("P1_Open", recapture=True)

    # -------------------------------------------------------------------------
    # 2. 权威可继续回执与命令幂等收尾 (Acceptance 2: R8-9)
    # -------------------------------------------------------------------------

    def test_air_service_cal_capture_step_done_and_idempotence(self) -> None:
        """黑匣子 cal_capture 触发短任务并在收尾后发送权威 step_done；同一 request_id 幂等防重采。"""
        cfg = airsvc.AirConfig(output_dir=str(self.tmp_path / "runs"))
        service = airsvc.AirBlackboxService(cfg, program_dir=self.tmp_path)
        cal_id = "cal_bb_test_01"

        # 发起 cal_begin
        begin_res = service.handle_command("cal_begin", {"cal_id": cal_id, "repeats": 1, "mode": "osl"})
        self.assertTrue(begin_res["ok"])

        # 模拟采集 capturer
        fake_capturer = _MockStepCapturer(n_points=5)
        service._calibration_session.capturer = fake_capturer

        # 发起 P1_Open 采集，带 request_id
        req_id = "req_capture_open_001"
        res_cap = service.handle_command(
            "cal_capture", {"cal_id": cal_id, "step": "P1_Open", "recapture": False, "request_id": req_id}
        )
        self.assertTrue(res_cap["ok"])

        # 等待后台线程执行并收尾
        if service._calibration_thread is not None:
            service._calibration_thread.join(timeout=3.0)

        # 验证步骤完成
        progress = service._calibration_session.step_progress("P1_Open")
        self.assertEqual(progress.state, "done")
        self.assertIn("P1_Open", fake_capturer.closed_steps, "短任务 close_current 必须被调用以确保 worker idle")
        self.assertIsNone(service._calibration_thread)

        # 重复发送同一 request_id 的请求：应命中缓存直接返回，不触发二次采集
        init_calls = fake_capturer.call_count.get("P1_Open", 0)
        res_dup = service.handle_command(
            "cal_capture", {"cal_id": cal_id, "step": "P1_Open", "recapture": True, "request_id": req_id}
        )
        self.assertTrue(res_dup["ok"])
        self.assertEqual(fake_capturer.call_count.get("P1_Open", 0), init_calls, "同一 request_id 严禁重复采集")

        # 发送错误 cal_id 请求
        res_err = service.handle_command(
            "cal_capture", {"cal_id": "wrong_cal_id", "step": "P1_Short", "request_id": "req_err"}
        )
        self.assertFalse(res_err["ok"])

    # -------------------------------------------------------------------------
    # 3. OSL 激活后背景采集与校正域解耦 (Acceptance 3: R8-3)
    # -------------------------------------------------------------------------

    def test_background_after_osl_activated_and_invalidation(self) -> None:
        """OSL 激活后可新建并完成背景采集；单通道不广播；新 OSL 使旧背景失效。"""
        cfg = airsvc.AirConfig(output_dir=str(self.tmp_path / "runs"))
        service = airsvc.AirBlackboxService(cfg, program_dir=self.tmp_path)

        # 1. 设置一个生效的 OSL 校准
        pts = [
            OSLPoint(1e9, complex(0.1, 0.1), complex(0.9, 0.0), complex(0.05, 0.05)),
            OSLPoint(2e9, complex(0.1, 0.1), complex(0.9, 0.0), complex(0.05, 0.05)),
        ]
        service._session_calibration = OSLCalibration(pts, self.tmp_path / "fake_osl.json")
        service.active_calibration_id = "osl_active_01"

        # 2. 在已有生效 OSL 的情况下，发起 background 模式的 cal_begin
        bg_cal_id = "cal_bg_01"
        begin_bg = service.handle_command("cal_begin", {"cal_id": bg_cal_id, "mode": "background", "repeats": 1})
        self.assertTrue(begin_bg["ok"], "OSL 激活后必须允许创建背景校准会话")
        self.assertEqual(service._calibration_session.plan.mode, "background")

        # 模拟采集 background
        fake_capturer = _MockStepCapturer(n_points=2, freq_min=1e9, freq_max=2e9)
        service._calibration_session.capturer = fake_capturer

        service.handle_command("cal_capture", {"cal_id": bg_cal_id, "step": "Background"})
        if service._calibration_thread is not None:
            service._calibration_thread.join(timeout=3.0)

        # finish & activate background
        fin_res = service.handle_command("cal_finish", {"cal_id": bg_cal_id})
        self.assertTrue(fin_res["ok"])
        act_res = service.handle_command("cal_activate", {"cal_id": bg_cal_id})
        self.assertTrue(act_res["ok"])

        # 验证背景已激活且为 S11 域 1D 数组（不广播至 S22）
        self.assertIsNotNone(service._background)
        self.assertEqual(service._background.ndim, 1)
        self.assertEqual(len(service._background), 2)
        self.assertEqual(service._background_cal_id, bg_cal_id)
        # OSL 依然生效保持
        self.assertIsNotNone(service._session_calibration)

        # 3. 激活新的 OSL 校准，旧背景必须显式失效
        new_osl_cal_id = "osl_active_02"
        service.handle_command("cal_begin", {"cal_id": new_osl_cal_id, "mode": "osl", "repeats": 1})
        service._calibration_session.capturer = fake_capturer
        for step in service._calibration_session.plan.steps:
            service._calibration_session.capture(step.name)
        service.handle_command("cal_finish", {"cal_id": new_osl_cal_id})
        service.handle_command("cal_activate", {"cal_id": new_osl_cal_id})

        self.assertEqual(service.active_calibration_id, new_osl_cal_id)
        self.assertIsNone(service._background, "新 OSL 激活后，旧背景必须被显式置为失效 None")
        self.assertIsNone(service._background_cal_id)

    # -------------------------------------------------------------------------
    # 4. 校准文件内容导入/导出与双端口/Through (Acceptance 4: R8-4, R8-5)
    # -------------------------------------------------------------------------

    def test_import_real_v2_solt_file_and_export(self) -> None:
        """真实仓库 solt_2port_20260807_202342.json 导入生效、双端口与 Through 恢复、完整导出。"""
        repo_v2_path = Path("solt_2port_20260807_202342.json")
        self.assertTrue(repo_v2_path.exists(), "仓库真实 v2 文件必须存在")
        content_str = repo_v2_path.read_text(encoding="utf-8")

        cfg = airsvc.AirConfig(output_dir=str(self.tmp_path / "runs"))
        service = airsvc.AirBlackboxService(cfg, program_dir=self.tmp_path)

        imp_res = service.handle_command(
            "cal_import_file",
            {
                "filename": "solt_2port_20260807_202342.json",
                "content": content_str,
                "activate": True,
                "request_id": "req_imp_real_v2",
            },
        )
        self.assertTrue(imp_res["ok"])
        self.assertTrue(imp_res["activated"])

        # 验证双端口均成功恢复
        self.assertIsNotNone(service._session_calibration, "P1 S11 OSL 必须加载")
        self.assertIsNotNone(service._session_calibration_p2, "P2 S22 OSL 必须加载")
        self.assertIsNotNone(service._active_through, "Through 测量必须加载")
        self.assertEqual(len(service._session_calibration.points), 201)
        self.assertEqual(len(service._session_calibration_p2.points), 201)

        # 查询状态
        query_res = service.handle_command("cal_query_active", {})
        self.assertTrue(query_res["ok"])
        self.assertTrue(query_res["active_osl"]["active"])
        self.assertEqual(query_res["active_osl"]["ports"], ["P1", "P2"])
        self.assertTrue(query_res["active_osl"]["has_through"])
        self.assertTrue(query_res["capabilities"]["recapture"])
        self.assertTrue(query_res["capabilities"]["cal_import_file"])

        # 完整内容导出
        exp_res = service.handle_command("cal_export_file", {"target": "osl"})
        self.assertTrue(exp_res["ok"])
        self.assertIn("content", exp_res)
        exported_data = json.loads(exp_res["content"])
        self.assertEqual(exported_data["format"], "uav_gpr_solt_calibration_v2")

    def test_import_legacy_osl_format_and_single_port_capabilities(self) -> None:
        """旧版 Open/Short/Load 单端口格式导入：P1 生效，明确不伪造 P2/Through。"""
        legacy_data = {
            "format": "uav_gpr_solt_calibration_v2",
            "measurements": [
                {
                    "type": "Open",
                    "data": {
                        "points": [
                            {"frequency": 1e9, "real": 0.9, "imag": 0.0},
                            {"frequency": 2e9, "real": 0.8, "imag": -0.1},
                        ]
                    },
                },
                {
                    "type": "Short",
                    "data": {
                        "points": [
                            {"frequency": 1e9, "real": -0.9, "imag": 0.0},
                            {"frequency": 2e9, "real": -0.8, "imag": 0.1},
                        ]
                    },
                },
                {
                    "type": "Load",
                    "data": {
                        "points": [
                            {"frequency": 1e9, "real": 0.01, "imag": 0.0},
                            {"frequency": 2e9, "real": 0.01, "imag": 0.0},
                        ]
                    },
                },
            ],
        }

        cfg = airsvc.AirConfig(output_dir=str(self.tmp_path / "runs"))
        service = airsvc.AirBlackboxService(cfg, program_dir=self.tmp_path)

        imp_res = service.handle_command(
            "cal_import_file",
            {"filename": "legacy_osl.json", "content": legacy_data, "activate": True},
        )
        self.assertTrue(imp_res["ok"])
        self.assertIsNotNone(service._session_calibration)
        self.assertIsNone(service._session_calibration_p2, "单端口校准严禁伪造 P2")
        self.assertIsNone(service._active_through, "无 Through 数据不可虚构")

        query_res = service.handle_command("cal_query_active", {})
        self.assertEqual(query_res["active_osl"]["ports"], ["P1"])
        self.assertFalse(query_res["active_osl"]["has_through"])

    # -------------------------------------------------------------------------
    # 5. 文件校验、原子落盘与异常保留旧校准 (Acceptance 5: R8-4, R8-8)
    # -------------------------------------------------------------------------

    def test_corrupt_import_preserves_existing_active_calibration(self) -> None:
        """坏文件/缺步骤/轴不一致/非有限值/超限等导入失败，绝对保留旧可用校准且不破坏文件。"""
        cfg = airsvc.AirConfig(output_dir=str(self.tmp_path / "runs"))
        service = airsvc.AirBlackboxService(cfg, program_dir=self.tmp_path)

        # 先导入一个合法的校准作为既有可用校准
        repo_v2_path = Path("solt_2port_20260807_202342.json")
        service.handle_command(
            "cal_import_file",
            {"filename": "valid_baseline.json", "content": repo_v2_path.read_text("utf-8"), "activate": True},
        )
        original_p1 = service._session_calibration
        original_path = service._active_calibration_path
        self.assertIsNotNone(original_p1)

        bad_cases = [
            ("bad_json", "{ this is definitely not valid json"),
            ("missing_steps", {"measurements": [{"type": "P1_Open", "data": {"points": []}}]}),
            (
                "axis_mismatch",
                {
                    "measurements": [
                        {"type": "P1_Open", "data": {"points": [{"frequency": 1e9, "real": 1, "imag": 0}]}},
                        {"type": "P1_Short", "data": {"points": [{"frequency": 1.5e9, "real": -1, "imag": 0}]}},
                        {"type": "P1_Load", "data": {"points": [{"frequency": 1e9, "real": 0, "imag": 0}]}},
                    ]
                },
            ),
            (
                "non_finite_nan",
                {
                    "measurements": [
                        {"type": "P1_Open", "data": {"points": [{"frequency": 1e9, "real": float("nan"), "imag": 0}]}},
                        {"type": "P1_Short", "data": {"points": [{"frequency": 1e9, "real": -1, "imag": 0}]}},
                        {"type": "P1_Load", "data": {"points": [{"frequency": 1e9, "real": 0, "imag": 0}]}},
                    ]
                },
            ),
        ]

        for case_name, bad_content in bad_cases:
            res = service.handle_command(
                "cal_import_file",
                {"filename": f"bad_{case_name}.json", "content": bad_content, "activate": True},
            )
            self.assertFalse(res["ok"], f"坏用例 {case_name} 必须被拒绝")
            self.assertIn("error", res)
            # 必须完好保留原有校准
            self.assertIs(service._session_calibration, original_p1)
            self.assertEqual(service._active_calibration_path, original_path)

    def test_persistence_and_restart_recovery(self) -> None:
        """持久化引用至 air_config.json 并在服务重新实例化时自启动恢复生效。"""
        cfg_path = self.tmp_path / "air_config.json"
        cfg = airsvc.AirConfig(output_dir=str(self.tmp_path / "runs"))
        service = airsvc.AirBlackboxService(cfg, config_path=cfg_path, program_dir=self.tmp_path)

        # 导入并激活
        repo_v2_path = Path("solt_2port_20260807_202342.json")
        service.handle_command(
            "cal_import_file",
            {"filename": "persisted_osl.json", "content": repo_v2_path.read_text("utf-8"), "activate": True},
        )
        saved_cal_path = service.config.calibration_path
        self.assertTrue(saved_cal_path)

        # 检查 air_config.json 落盘内容
        self.assertTrue(cfg_path.exists())
        config_disk = json.loads(cfg_path.read_text("utf-8"))
        self.assertEqual(config_disk["calibration_path"], saved_cal_path)

        # 模拟服务重启 (实例化新服务对象)
        new_cfg, _ = airsvc.load_air_config(cfg_path)
        recovered_service = airsvc.AirBlackboxService(new_cfg, config_path=cfg_path, program_dir=self.tmp_path)

        # 验证新服务自启动即恢复生效校准
        self.assertIsNotNone(recovered_service._session_calibration)
        self.assertEqual(len(recovered_service._session_calibration.points), 201)
        self.assertEqual(recovered_service.active_calibration_id, "persisted_osl")

    # -------------------------------------------------------------------------
    # 6. 带外频率拒绝与清除校准定向控制 (Acceptance 6, 7)
    # -------------------------------------------------------------------------

    def test_out_of_band_frequency_rejected(self) -> None:
        """带外频率申请校正时，严禁非法外推，必须抛出 ValueError。"""
        from uav_gpr_qt import apply_reflection_and_through_calibration

        pts = [
            OSLPoint(1e9, complex(0.1, 0.1), complex(0.9, 0.0), complex(0.05, 0.05)),
            OSLPoint(2e9, complex(0.1, 0.1), complex(0.9, 0.0), complex(0.05, 0.05)),
        ]
        cal_p1 = OSLCalibration(pts, Path("cal.json"))

        # 带内正常
        in_band_f = np.array([1.2e9, 1.8e9])
        raw_sparams = {"S11": np.array([0.5 + 0.1j, 0.5 + 0.1j])}
        res = apply_reflection_and_through_calibration(in_band_f, raw_sparams, cal_p1, None, None)
        self.assertIn("S11", res)

        # 带外（低于 1GHz）拒绝
        low_f = np.array([800e6, 1.5e9])
        with self.assertRaises(ValueError) as ctx:
            apply_reflection_and_through_calibration(low_f, raw_sparams, cal_p1, None, None)
        self.assertIn("禁止带外外推", str(ctx.exception))

        # 带外（高于 2GHz）拒绝
        high_f = np.array([1.5e9, 2.5e9])
        with self.assertRaises(ValueError) as ctx:
            apply_reflection_and_through_calibration(high_f, raw_sparams, cal_p1, None, None)
        self.assertIn("禁止带外外推", str(ctx.exception))

    def test_clear_active_controls(self) -> None:
        """cal_clear_active: clear background 不得清除 OSL 持久路径；clear all 两者皆清。"""
        cfg_path = self.tmp_path / "air_config.json"
        cfg = airsvc.AirConfig(output_dir=str(self.tmp_path / "runs"))
        service = airsvc.AirBlackboxService(cfg, config_path=cfg_path, program_dir=self.tmp_path)

        repo_v2_path = Path("solt_2port_20260807_202342.json")
        service.handle_command(
            "cal_import_file",
            {"filename": "persisted_osl.json", "content": repo_v2_path.read_text("utf-8"), "activate": True},
        )
        service._background = np.ones(10, dtype=np.complex128)
        service._background_frequency_hz = np.linspace(1e9, 2e9, 10)
        service._background_cal_id = "bg_01"

        # 仅清除背景
        clr_bg_res = service.handle_command("cal_clear_active", {"target": "background"})
        self.assertTrue(clr_bg_res["ok"])
        self.assertIsNone(service._background)
        self.assertIsNotNone(service._session_calibration, "清除背景不得损坏生效 OSL")
        self.assertTrue(service.config.calibration_path, "清除背景不得清空 OSL 持久化路径")

        # 清除全部
        clr_all_res = service.handle_command("cal_clear_active", {"target": "all"})
        self.assertTrue(clr_all_res["ok"])
        self.assertIsNone(service._session_calibration)
        self.assertEqual(service.config.calibration_path, "")


if __name__ == "__main__":
    unittest.main()
