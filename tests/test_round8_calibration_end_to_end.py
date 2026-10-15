#!/usr/bin/env python3
"""第8轮 t4 独立双端端到端回归（tests/test_round8_calibration_end_to_end.py）。

验证角色（verifier）独占文件。与工程师单测的本质区别：

- **真实双端命令链**：地面向导真实按钮入口（begin/capture/recapture/import/
  export/query）→ 真实 ``ReceiverThread``（TCP 服务端）→ 真实 ``RemoteSender``
  （TCP 客户端，uav_gpr_qt 既有实现）→ 真实 ``AirBlackboxService.handle_command``
  黑匣子命令入口 → 真实 ``CalibrationSession`` + ``ServiceStepCapturer`` → 真实
  ``AcquisitionService``/``PersistentLibreVnaBackend``/``ContinuousAcquisitionWorker``
  采集管线。全程 loopback TCP + 真实 framing，**绝不**用 DummyReceiver/注入成功
  header 宣布双端成功。
- **模拟设备 + 可控标准件**：``ControlledStandardTransport`` 在真实 USB 协议线
  模拟器（``SimulatedLibreVnaTransport``）物理模型上叠加"按采集任务轮换标准件"
  的受控反射偏置（Open/Short/Load 六步各不同、背景与后续 trace 场景不同），
  使 OSL 解算可执行且数值可核对；``noise_level=0`` 保证数值确定性。
- **数值级核对**：校正/背景数值直接核对（NPZ raw/display 通道 vs 运行期校准
  对象反算），校准身份核对（acquisition metadata / cal_query_active），不是只看
  文案。噪声为 0 的模拟数据不证明真实 VNA 精度（现场真机复核清单另行交付）。

覆盖映射（验收矩阵 R8-1/3/4/5/6/7/8/9/10 + captain t3 附加静态风险独立证伪）：
1. OSL 六步（真实采集）→ 解算 → 激活 → 模式解锁 → 背景会话 → 真实 trace，
   数值核对校正域/背景扣除/端口/校准身份（R8-1、风险4）。
2. 仓库真实 v2 文件完整内容导入/导出往返 → 空中重启恢复 → 恢复后数值校正；
   旧单端口范围明示；坏文件拒绝保留旧校准；原件哈希不变（R8-3/4/5/6）。
3. P1_Open 完成后受控重采 → P1_Short 续采；已解算后重采作废旧解算；原始采样
   审计证据保留（R8-7/8）。
4. 同 request_id 重发不重复采集；跨会话/迟到命令不得劫持在途采集；watchdog
   超时查询远端 busy 且可恢复；长扫描看门狗预算；双击防抖与拒绝回退（R8-9）。
5. 断线重连后会话/步骤/能力同步；旧空中端（无 step_done/不支持导入）平滑降级
   且"不支持"明确可见（R8-10）。
6. 黑匣子命令入口边界：import 配置写盘失败回滚、activate 持久化失败仍报成功、
   validator dict 无 10MB 限制/坏通道静默丢 P2、空中端 filename 未净化
   （captain 风险 3/5/6 独立实测）。

红线遵守：不改生产代码；不改工程师单测；不写 smoke 脚本；不部署/不自启。
"""

from __future__ import annotations

import json
import os
import shutil
import tempfile
import time
import unittest
import uuid
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")

import numpy as np

import calibration_service as cal
import uav_gpr_air_service as svc
import uav_gpr_ground_station as ground
import air_core as air
from librevna.librevna_continuous import PersistentBackendSettings, PersistentLibreVnaBackend
from librevna.simulated_transport import (
    SimulatedLibreVnaTransport,
)
from PySide6 import QtCore, QtWidgets

REPO_ROOT = Path(__file__).resolve().parent.parent
REAL_V2_FILE = REPO_ROOT / "solt_2port_20260807_202342.json"


def wait_until(predicate, timeout_s: float = 20.0, interval_s: float = 0.05, app=None) -> bool:
    """有界轮询；Qt 测试里每轮 processEvents 驱动 queued 信号投递。"""
    deadline = time.monotonic() + timeout_s
    while True:
        ok = bool(predicate())
        if ok:
            return True
        if time.monotonic() >= deadline:
            return bool(predicate())
        if app is not None:
            app.processEvents()
        time.sleep(interval_s)


def sha256_of(path: Path) -> str:
    import hashlib

    return hashlib.sha256(path.read_bytes()).hexdigest()


def npz_complex(npz, prefix: str) -> np.ndarray:
    return np.asarray(npz[f"{prefix}_real"], dtype=np.float64) + 1j * np.asarray(
        npz[f"{prefix}_imag"], dtype=np.float64
    )


class ControlledStandardTransport(SimulatedLibreVnaTransport):
    """在模拟设备物理模型上叠加"按采集任务轮换的可控标准件"偏置。

    每次 ``SWEEP_SETTINGS``（= 一次采集任务 configure/resume）递增任务序号；
    任务序号 1..6 对应 OSL 六步（Open/Short/Load × P1/P2 各给不同反射偏置，
    保证 OSL 三方程可解），7 为背景场景，≥8 为雷达 trace 场景（与背景不同，
    使背景扣除可数值核对）。``noise_level=0`` 保证数值确定性。确定性"采集
    在途"窗口用 ifbw=100 + repeats=2 的真实设备步进节奏（第 1 条样本后仍有
    约 8s 第 2 条步进）建立，不阻塞设备读取。
    """

    STANDARD_TABLE = (
        complex(0.90, -0.03),  # task1 P1_Open
        complex(-0.88, 0.04),  # task2 P1_Short
        complex(0.03, 0.02),  # task3 P1_Load
        complex(0.92, 0.05),  # task4 P2_Open
        complex(-0.91, -0.02),  # task5 P2_Short
        complex(0.02, -0.03),  # task6 P2_Load
        complex(0.31, 0.12),  # task7 Background
        complex(0.55, -0.20),  # task8+ 雷达 trace（与背景可区分）
    )

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.task_switches = 0

    def _apply_sweep_settings(self, payload: bytes) -> None:
        self.task_switches += 1
        super()._apply_sweep_settings(payload)

    def _standard_offset(self) -> complex:
        idx = max(0, self.task_switches - 1)
        table = self.STANDARD_TABLE
        return table[idx] if idx < len(table) - 1 else table[-1]

    def _next_point_value(self, reflectors, freq_hz: int) -> complex:
        return super()._next_point_value(reflectors, freq_hz) + self._standard_offset()


class _MessageCollector(QtCore.QObject):
    """测试观测器：只计数/记录真实接收线程已收到的帧，不改任何生产状态。"""

    def __init__(self, window: "ground.GroundStationWindow") -> None:
        super().__init__()
        self.calibration_messages: list[dict] = []
        self.command_results: list[dict] = []
        window.receiver.calibration_message.connect(self._on_cal)
        window.receiver.command_result.connect(self._on_cmd)

    def _on_cal(self, header: dict) -> None:
        self.calibration_messages.append(dict(header))

    def _on_cmd(self, header: dict) -> None:
        self.command_results.append(dict(header))

    def count_cal(self, **fields: object) -> int:
        def _match(header: dict) -> bool:
            return all(header.get(k) == v for k, v in fields.items())

        return sum(1 for h in self.calibration_messages if _match(h))

    def has_cmd(self, **fields: object) -> bool:
        def _match(header: dict) -> bool:
            for key, value in fields.items():
                if key == "result":
                    result = header.get("result")
                    if not isinstance(result, dict) or not all(
                        result.get(k) == v for k, v in value.items()
                    ):
                        return False
                elif header.get(key) != value:
                    return False
            return True

        return any(_match(h) for h in self.command_results)

    def count_cmd(self, **fields: object) -> int:
        def _match(header: dict) -> bool:
            for key, value in fields.items():
                if key == "result":
                    result = header.get("result")
                    if not isinstance(result, dict) or not all(
                        result.get(k) == v for k, v in value.items()
                    ):
                        return False
                elif header.get(key) != value:
                    return False
            return True

        return sum(1 for h in self.command_results if _match(h))


class _E2ECase(unittest.TestCase):
    """真实双端基座：地面窗口（真实监听）+ 空中黑匣子（真实发送/命令入口）。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance()
        if cls.app is None:
            cls.app = QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="r8_e2e_"))
        self.addCleanup(self._cleanup)
        air.set_default_device_source("simulated")
        self.ground_out = self.tmp / "ground_out"
        self.window = ground.GroundStationWindow(
            host="127.0.0.1",
            port=0,
            out_dir=self.ground_out,
            start_receiver=True,
        )
        self.wizard = self.window.calibration_dialog
        self.assertTrue(
            wait_until(lambda: self.window.receiver.bound_port is not None, 10, app=self.app),
            "地面 ReceiverThread 未在时限内绑定端口",
        )
        # pace=True：模拟设备按 2/ifbw 逐点步进。默认 ifbw=10000 时每道仅
        # ~0.08s（不影响常规用例速度）；在途窗口用例将 ifbw 降到 100 获得
        # ~8s/步的确定性真实扫频时长。
        self.transport = ControlledStandardTransport(pace=True, noise_level=0.0)
        self.air_logs: list[str] = []
        self.service = self._make_service(
            self.tmp / "air_config.json",
            backend_factory=self._backend_factory,
            start_sender=True,
        )
        self.assertTrue(
            wait_until(
                lambda: self.service.connected and self.window._remote_connected,
                15,
                app=self.app,
            ),
            "空中端未在时限内完成对地面端真实 TCP 连接/握手",
        )
        self.assertTrue(self.wizard.begin_button.isEnabled(), "连接后向导应可用")
        self.collector = _MessageCollector(self.window)

    def _backend_factory(self) -> PersistentLibreVnaBackend:
        """注入点：真实 PersistentLibreVnaBackend + 受控标准件模拟传输。

        sweep_timeout_s 放宽到 120s，使慢步进在途窗口（ifbw=100 约 8s/步）
        远短于设备扫频超时，绝不把测试注入故障混入被测行为。
        """
        return PersistentLibreVnaBackend(
            transport=self.transport,
            settings=PersistentBackendSettings(sweep_timeout_s=120.0),
        )

    def _make_service(
        self,
        config_path: Path,
        *,
        backend_factory,
        start_sender: bool,
        output_dir: Path | None = None,
        config: "svc.AirConfig | None" = None,
    ) -> "svc.AirBlackboxService":
        cfg = config or svc.AirConfig(
            ground_host="127.0.0.1",
            ground_port=int(self.window.receiver.bound_port),
            device_source="simulated",
            gnss_port="",
            output_dir=str(output_dir or (self.tmp / "runs")),
        )
        service = svc.AirBlackboxService(
            cfg,
            program_dir=self.tmp,
            config_path=config_path,
            log=self.air_logs.append,
            backend_factory=backend_factory,
            reconnect_initial_s=0.3,
            reconnect_max_s=0.8,
        )
        self.addCleanup(self._shutdown_service, service)
        if start_sender:
            service.sender.start()
        return service

    @staticmethod
    def _shutdown_service(service: "svc.AirBlackboxService") -> None:
        try:
            service.shutdown()
        except Exception:  # noqa: BLE001 - 收尾失败不阻断用例清理
            pass

    def _cleanup(self) -> None:
        for service in getattr(self, "_extra_services", []):
            self._shutdown_service(service)
        window = getattr(self, "window", None)
        if window is not None:
            for _ in range(50):
                stats = window.display_fifo.statistics()
                if stats.get("depth", 0) == 0 and stats.get("unresolved", 0) == 0:
                    break
                self.app.processEvents()
                time.sleep(0.02)
            window.close()
            window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
            self.app.processEvents()
        air.set_default_device_source("usb")
        shutil.rmtree(self.tmp, ignore_errors=True)

    # ---- 地面向导驱动助手（真实按钮入口）----

    def _query_capabilities(self) -> None:
        """真实查询按钮：建立新端严格 step_done 模式（补遗修订6 能力协商）。

        必须等待"本次查询"的回执真实到达并被向导处理（计数基线）：若只等
        sticky 状态，陈旧回执会在采集在途期间才到达（实测 F-stale 缺口）。
        """
        query_acks_baseline = self.collector.count_cmd(action="cal_query_active")
        self.wizard.query_active_calibration()
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cmd(action="cal_query_active")
                > query_acks_baseline,
                15,
                app=self.app,
            ),
            "新空中端能力 step_done 应为 True（真实 cal_query_active 回执）",
        )
        self.assertTrue(
            self.wizard._air_supports_step_done is True,
            f"cal_query_active 回执未建立 step_done 能力："
            f"last_ack={self.collector.command_results[-1] if self.collector.command_results else None} "
            f"state={self.wizard._air_supports_step_done!r}",
        )

    def _begin_osl(
        self,
        *,
        query_capabilities: bool = True,
        repeats: int = 2,
        ifbw: int = 10000,
    ) -> str:
        # repeats/ifbw 必须在 begin 前设置（会话计划在 cal_begin 冻结）。
        # ifbw=100 + repeats=1 时模拟设备按 2/ifbw 逐点步进，单步约 8s，
        # 提供确定性的真实"采集在途"窗口（供并发命令/看门狗/陈旧回执用例）。
        self.wizard.repeats.setValue(repeats)
        self.wizard.cal_ifbw.setValue(ifbw)
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id
        self.assertIsNotNone(cal_id)
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cal(type="calibration_result", status="planned",
                                                 cal_id=cal_id) >= 1,
                15,
                app=self.app,
            ),
            "空中端未回 planned（真实命令链故障）",
        )
        self.assertEqual(self.wizard.steps_table.rowCount(), 6)
        if query_capabilities:
            self._query_capabilities()
        return str(cal_id)

    def _begin_background(self) -> str:
        self.wizard.mode_combo.setCurrentIndex(1)
        self.assertEqual(self.wizard.mode_combo.currentData(), "background")
        self.wizard.repeats.setValue(2)
        self.wizard.begin_calibration()
        cal_id = self.wizard._cal_id
        self.assertIsNotNone(cal_id)
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cal(type="calibration_result", status="planned",
                                                 cal_id=cal_id) >= 1,
                15,
                app=self.app,
            ),
            "空中端未回背景 planned",
        )
        self.assertEqual(self.wizard.steps_table.rowCount(), 1)
        self._query_capabilities()
        return str(cal_id)

    def _capture_and_wait_done(self, expected_step: str) -> str:
        """点击"采集下一步"，等待真实采集完成（严格 step_done 路径或末样本）。"""
        self.wizard.capture_next_step()
        if self.wizard.current_step_name() != expected_step:
            self.fail(
                f"指针错位: 期望 {expected_step} 实际 {self.wizard.current_step_name()} "
                f"completed={sorted(self.wizard._completed_steps)} steps_done={self.wizard._steps_done} "
                f"capturing={self.wizard._capturing} "
                f"pending={self.wizard._pending_step}/{self.wizard._pending_request_id} "
                f"msgs=["
                + ", ".join(
                    json.dumps({k: v for k, v in m.items() if k in ('type', 'status', 'step', 'request_id', 'reason')}, ensure_ascii=False)
                    for m in self.collector.calibration_messages[-8:]
                )
                + f"] air_logs={self.air_logs[-10:]} "
                f"window_log={self.window.log_box.toPlainText()[-1200:]!r}"
            )
        self.assertTrue(self.wizard._capturing)
        request_id = str(self.wizard._pending_request_id or "")
        self.assertTrue(request_id, "采集请求必须携带 request_id")
        ok = wait_until(
            lambda: (not self.wizard._capturing)
            and self.wizard.steps_table.item(
                self.wizard._steps.index(expected_step), 1
            ).text()
            == "已完成",
            60,
            app=self.app,
        )
        if not ok:
            self.fail(
                f"步骤 {expected_step} 未在时限内真实完成（行文本="
                f"{self.wizard.steps_table.item(self.wizard._steps.index(expected_step), 1).text()}）"
            )
        if expected_step not in self.wizard._completed_steps:
            self.fail(
                f"步骤 {expected_step} 行显示已完成但未进入真实完成集合: "
                f"completed={sorted(self.wizard._completed_steps)} steps_done={self.wizard._steps_done} "
                f"pending={self.wizard._pending_step}/{self.wizard._pending_request_id} "
                f"capturing={self.wizard._capturing} msgs=["
                + ", ".join(
                    json.dumps({k: v for k, v in m.items() if k in ('type', 'status', 'step', 'request_id', 'reason')}, ensure_ascii=False)
                    for m in self.collector.calibration_messages[-8:]
                )
                + f"] air_logs={self.air_logs[-10:]} "
                f"window_log={self.window.log_box.toPlainText()[-1200:]!r}"
            )
        if self.wizard.current_step_name() != self._next_expected_after(expected_step):
            self.fail(
                f"完成 {expected_step} 后指针未推进到首个未完成步骤: "
                f"{self.wizard.current_step_name()} completed={sorted(self.wizard._completed_steps)} "
                f"steps_done={self.wizard._steps_done} msgs=["
                + ", ".join(
                    json.dumps({k: v for k, v in m.items() if k in ('type', 'status', 'step', 'request_id', 'reason')}, ensure_ascii=False)
                    for m in self.collector.calibration_messages[-8:]
                )
                + f"] air_logs={self.air_logs[-8:]} "
                f"window_log={self.window.log_box.toPlainText()[-1500:]!r}"
            )
        return request_id

    def _next_expected_after(self, expected_step: str) -> str | None:
        steps = self.wizard._steps
        remaining = [s for s in steps if s not in self.wizard._completed_steps]
        return remaining[0] if remaining else None

    def _capture_all_osl_steps(self) -> None:
        for step in self.wizard.OSL_STEPS:
            self._capture_and_wait_done(step)
        self.assertEqual(self.wizard._steps_done, 6)

    def _row_text(self, step: str, column: int = 1) -> str:
        row = self.wizard._steps.index(step)
        item = self.wizard.steps_table.item(row, column)
        return item.text() if item is not None else ""

    def _finish_and_solve(self, cal_id: str) -> None:
        solved_baseline = self.collector.count_cal(
            type="calibration_result", status="solved", cal_id=cal_id
        )
        self.wizard.finish_calibration()
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cal(type="calibration_result", status="solved",
                                                 cal_id=cal_id) > solved_baseline,
                30,
                app=self.app,
            ),
            f"空中端未回 solved（诊断: completed={sorted(self.wizard._completed_steps)} "
            f"steps_done={self.wizard._steps_done} capturing={self.wizard._capturing} "
            f"cal_id={self.wizard._cal_id} air_session={self.service._calibration_session.state if self.service._calibration_session else None} "
            f"msgs=["
            + ", ".join(
                json.dumps({k: v for k, v in m.items() if k in ('type', 'status', 'step', 'reason')}, ensure_ascii=False)
                for m in self.collector.calibration_messages[-8:]
            )
            + f"] air_logs={self.air_logs[-8:]})"
        )
        self.assertTrue(self.wizard._solved)
        self.assertTrue(self.wizard.activate_button.isEnabled())

    def _activate(self, cal_id: str) -> None:
        activated_baseline = self.collector.count_cal(
            type="calibration_result", status="activated", cal_id=cal_id
        )
        self.wizard.activate_calibration()
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cal(type="calibration_result", status="activated",
                                                 cal_id=cal_id) > activated_baseline,
                30,
                app=self.app,
            ),
            "空中端未回 activated",
        )

    def _wait_air_session_quiet(self) -> None:
        """等待空中端采集服务回到空闲（校准短任务收尾、设备释放待命）。"""
        self.assertTrue(
            wait_until(
                lambda: self.service.acquisition.state
                in ("closed", "idle_waiting", "paused")
                and (
                    self.service._calibration_thread is None
                    or not self.service._calibration_thread.is_alive()
                ),
                30,
            ),
            f"空中端服务未回空闲: {self.service.acquisition.statistics()}",
        )


class TestR8_1_FullChain(_E2ECase):
    """R8-1：六步 OSL（真实采集）→ 激活 → 背景 → 真实 trace，数值核对。"""

    def test_osl_activate_background_and_calibrated_trace(self) -> None:
        cal_id = self._begin_osl()

        # 计划后先查询一次：能力/会话结构来自真实 cal_query_active 回执。
        # 等待"本次查询"的回执真实到达并被处理（计数基线），避免陈旧回执
        # 在采集在途期间才到达（实测见 test_stale_query_ack_does_not_hijack）。
        query_acks_baseline = self.collector.count_cmd(action="cal_query_active")
        self.wizard.query_active_calibration()
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cmd(action="cal_query_active")
                > query_acks_baseline,
                15,
                app=self.app,
            ),
            "cal_query_active 未获回执",
        )
        self.assertTrue(self.wizard._air_supports_step_done, "新空中端能力 step_done 应为 True")
        query_cmds = [h for h in self.collector.command_results if h.get("action") == "cal_query_active"]
        result = query_cmds[-1].get("result") or {}
        self.assertTrue(result.get("ok"))
        self.assertEqual(result["in_progress_session"]["cal_id"], cal_id)
        self.assertEqual(result["in_progress_session"]["mode"], "osl")
        self.assertEqual(
            result["in_progress_session"]["steps_status"],
            {name: "pending" for name in self.wizard.OSL_STEPS},
        )

        # 严格 step_done 路径逐真实采集六步
        self._capture_all_osl_steps()
        self.assertEqual(
            self.collector.count_cal(type="calibration_result", status="step_done"),
            6,
            "六步应各有一条权威 step_done",
        )

        # 解算 → 激活：向导解锁且 OSL 身份保留（故障一修复证明）
        self._finish_and_solve(cal_id)
        # 既有 calibration_quality 回程必须保留（补遗修订1），解算时按步回传
        self.assertGreaterEqual(
            self.collector.count_cal(type="calibration_quality"),
            6,
            "六步 OSL 解算后应各有一条 calibration_quality",
        )
        self._activate(cal_id)
        self.assertIsNone(self.wizard._cal_id, "激活后临时会话必须解除")
        self.assertTrue(self.wizard.begin_button.isEnabled())
        self.assertTrue(self.wizard.mode_combo.isEnabled())
        self.assertIsNotNone(self.wizard._active_calibration)
        self.assertIn("OSL[", self.wizard.active_cal_label.text())

        # 空中端：会话激活后可以马上查到生效 OSL
        q = self.service.handle_command("cal_query_active", {})
        self.assertTrue(q["ok"])
        self.assertTrue(q["active_osl"]["active"])
        self.assertEqual(q["active_osl"]["cal_id"], cal_id)
        self.assertFalse(q["active_background"]["active"])

        # OSL 激活后无缝新建背景会话（同一真实链路，双端不重启）
        bg_cal_id = self._begin_background()
        self.assertNotEqual(bg_cal_id, cal_id)
        self._capture_and_wait_done("Background")
        self._finish_and_solve(bg_cal_id)
        self._activate(bg_cal_id)
        self.assertIsNotNone(self.wizard._active_background)
        self.assertIn("背景[", self.wizard.active_cal_label.text())

        q2 = self.service.handle_command("cal_query_active", {})
        self.assertTrue(q2["active_background"]["active"])
        self.assertEqual(q2["active_background"]["cal_id"], bg_cal_id)
        # F11 正式回归（t10）：背景激活严禁覆盖 OSL 身份——OSL 仍然生效，
        # active_osl.cal_id 必须保持 OSL 会话身份。
        # 修复前现状 = 被覆盖为背景身份（本断言红，修复后转绿）。
        self.assertEqual(
            q2["active_osl"].get("cal_id"),
            cal_id,
            "F11 契约：背景激活后 active_osl.cal_id 必须仍是 OSL 身份 "
            f"（实测={q2['active_osl'].get('cal_id')!r}，被背景身份覆盖）",
        )
        self.assertTrue(q2["active_osl"]["active"], "背景激活不得使 OSL 失效")

        # ---- 真实雷达 trace（单道）经既有 start 命令入口 ----
        self._osl_cal_id_for_trace = cal_id
        self._bg_cal_id_for_trace = bg_cal_id
        self.window.receiver.send_command("start", {"single": True})
        trace_path = self._wait_ground_trace()
        self._verify_trace_numerics(trace_path, expect_background=True)

        # 采集任务与校准互斥/收尾：服务回空闲
        self._wait_air_session_quiet()

    def _wait_ground_trace(self) -> Path:
        def _trace_file():
            candidates = sorted(self.ground_out.rglob("trace_*.npz"))
            return candidates[0] if candidates else None

        self.assertTrue(
            wait_until(lambda: _trace_file() is not None, 45, app=self.app),
            "地面站未在时限内收到真实 trace（真实发送/接收链路故障）",
        )
        trace_path = _trace_file()
        # 显示链路：解码提交后 B-scan 行出现（UI 消费证明）
        wait_until(
            lambda: len(self.window.bscan_rows.get("S11", [])) >= 1, 20, app=self.app
        )
        return trace_path

    def _verify_trace_numerics(self, trace_path: Path, *, expect_background: bool) -> None:
        npz = np.load(trace_path)
        self.assertIn("acquisition", npz.files, "trace 必须带 acquisition 元数据")
        acquisition = json.loads(str(npz["acquisition"]))
        freq = np.asarray(npz["frequency_hz"], dtype=np.float64)
        raw_s11 = npz_complex(npz, "raw_S11")
        raw_s22 = npz_complex(npz, "raw_S22")
        disp_s11 = npz_complex(npz, "display_S11")
        disp_s22 = npz_complex(npz, "display_S22")

        cal_p1 = self.service._session_calibration
        cal_p2 = self.service._session_calibration_p2
        self.assertIsNotNone(cal_p1, "P1 校准必须真实生效")
        self.assertIsNotNone(cal_p2, "P2 校准必须真实生效（双端口 201 点）")
        self.assertEqual(len(cal_p1.points), 201)
        self.assertEqual(len(cal_p2.points), 201)

        expected_s11 = np.array([cal_p1.correct(f, v) for f, v in zip(freq, raw_s11)])
        if expect_background:
            background = self.service._background
            self.assertIsNotNone(background, "背景必须真实生效")
            expected_s11 = expected_s11 - background
        self.assertTrue(
            np.allclose(disp_s11, expected_s11, atol=1e-7, rtol=1e-7),
            f"S11 display 未按 OSL 校准（+背景扣除）真实生效: max diff="
            f"{np.max(np.abs(disp_s11 - expected_s11)):.3e}",
        )
        expected_s22 = np.array([cal_p2.correct(f, v) for f, v in zip(freq, raw_s22)])
        if expect_background:
            # 2026-10-15 起 VV 也扣自己的背景（P2 校正域），绝不套用 HH 背景。
            background_s22 = self.service._background_s22
            self.assertIsNotNone(background_s22, "VV 背景必须真实生效")
            expected_s22 = expected_s22 - background_s22
        self.assertTrue(
            np.allclose(disp_s22, expected_s22, atol=1e-7, rtol=1e-7),
            "S22 display 未按 P2 校准（+VV 背景扣除）真实生效",
        )

        if expect_background:
            # 背景存储域核对：等于"背景原始采样的 OSL 校正域值"，绝不等于 RAW
            bg_dir = self.service.output_root / "_calibration"
            bg_files = sorted(bg_dir.rglob("capture_Background_*.npz"))
            self.assertTrue(bg_files, "背景会话的原始采样证据应存在")
            raws = []
            bg_freq = None
            for path in bg_files:
                data = np.load(path)
                bg_freq = np.asarray(data["frequency_hz"], dtype=np.float64)
                raws.append(npz_complex(data, "S11"))
            raw_bg_mean = np.mean(np.stack(raws, axis=0), axis=0)
            expected_bg = np.array([cal_p1.correct(f, v) for f, v in zip(bg_freq, raw_bg_mean)])
            self.assertTrue(
                np.allclose(self.service._background, expected_bg, atol=1e-7, rtol=1e-7),
                "背景数组未按 R-C-19 存储为 OSL 校正域",
            )
            self.assertGreater(
                float(np.max(np.abs(self.service._background - raw_bg_mean))),
                0.1,
                "背景校正域数值与 RAW 域必须可区分（校正真实发生）",
            )
            # 校正后背景扣减必须真实改变 display（非恒等）
            only_calibrated = np.array([cal_p1.correct(f, v) for f, v in zip(freq, raw_s11)])
            self.assertGreater(
                float(np.max(np.abs(only_calibrated - disp_s11))),
                0.1,
                "背景扣除必须真实改变 S11 display 数值",
            )
            # F11 正式回归（t10）：trace 元数据 calibration_id 必须引用真实生效
            # 的 OSL 校准身份（实际校正域），严禁被背景身份覆盖。
            # 修复前现状 = 背景身份（本断言红，修复后转绿）。
            observed_cal_id = acquisition.get("calibration_id")
            self.assertEqual(
                observed_cal_id,
                self.service.active_calibration_id,
                "trace 元数据 calibration_id 必须与空中端 active_calibration_id 一致",
            )
            self.assertEqual(
                observed_cal_id,
                self._osl_cal_id_for_trace,
                "F11 契约：trace 元数据 calibration_id 必须是 OSL 身份（实际校正域）"
                f"（实测={observed_cal_id!r}，被背景身份覆盖）",
            )
            self.assertNotEqual(
                observed_cal_id,
                self._bg_cal_id_for_trace,
                "F11 契约：trace 元数据 calibration_id 不得是背景身份",
            )


class TestR8_3_5_ImportExportRestart(_E2ECase):
    """R8-3/4/5/6：真实 v2 文件导入/导出/重启恢复/旧单端口/坏文件保留。"""

    def test_real_v2_import_export_query_restart_and_correction(self) -> None:
        self.assertTrue(REAL_V2_FILE.exists())
        repo_sha_before = sha256_of(REAL_V2_FILE)
        repo_json = json.loads(REAL_V2_FILE.read_text(encoding="utf-8"))

        # 真实向导按钮入口导入完整内容（非路径）
        self.assertTrue(self.wizard.import_calibration_file(str(REAL_V2_FILE)))
        self.assertTrue(
            wait_until(
                lambda: self.service._session_calibration is not None
                and self.service._active_calibration_path is not None,
                30,
            ),
            "空中端未真实落盘并生效导入文件",
        )
        self.assertTrue(
            wait_until(
                lambda: self.wizard._active_calibration is not None,
                15,
                app=self.app,
            ),
            "地面未收到导入 command_ack",
        )

        # 查询真实生效身份/端口/点数/频段（补遗 §修订6 结构）
        self.wizard.query_active_calibration()
        self.assertTrue(
            wait_until(
                lambda: (self.wizard._active_calibration or {}).get("ports") == ["P1", "P2"],
                15,
                app=self.app,
            ),
            "查询后地面未同步到双端口生效信息",
        )
        self.assertIn("P1+P2", self.wizard.active_cal_label.text())
        self.assertIn("201点", self.wizard.active_cal_label.text())
        self.assertIn("400.0-4000.0 MHz", self.wizard.active_cal_label.text())

        q = self.service.handle_command("cal_query_active", {})
        self.assertTrue(q["ok"])
        self.assertEqual(q["active_osl"]["ports"], ["P1", "P2"])
        self.assertEqual(q["active_osl"]["points"], 201)
        self.assertEqual(
            q["active_osl"]["valid_band_hz"], [400000000.0, 4000000000.0]
        )
        self.assertTrue(q["active_osl"]["has_through"], "真实 v2 文件的 Through 必须恢复")
        self.assertIsNotNone(self.service._session_calibration_p2)
        # captain 风险4 实测：导入的 Through 是否传入采集任务 spec
        self.assertIsNotNone(self.service._active_through, "内存 Through 已加载")

        # 完整内容导出 → 本地文件可重读且与原件等价
        export_target = self.tmp / "exported_roundtrip.json"
        self.assertTrue(self.wizard.export_calibration_file(str(export_target)))
        self.assertTrue(
            wait_until(lambda: export_target.exists(), 20, app=self.app),
            "导出未真实落盘",
        )
        exported = json.loads(export_target.read_text(encoding="utf-8"))
        self.assertEqual(exported.get("format"), "uav_gpr_solt_calibration_v2")
        self.assertEqual(exported, repo_json, "导出内容必须与原件逐字段等价（完整内容，非路径）")

        # 空中端重启：旧服务真实关闭（断开发送线程），新服务实例从
        # air_config.json 自启动恢复同一校准并重新连接地面端
        cal_path = Path(self.service._active_calibration_path)
        self._shutdown_service(self.service)
        restored_cfg, _issues = svc.load_air_config(self.tmp / "air_config.json")
        self.assertEqual(
            Path(restored_cfg.calibration_path), cal_path, "持久化引用必须落盘"
        )
        service2 = self._make_service(
            self.tmp / "air_config.json",
            backend_factory=lambda: PersistentLibreVnaBackend(
                transport=ControlledStandardTransport(pace=False, noise_level=0.0),
                settings=PersistentBackendSettings(sweep_timeout_s=120.0),
            ),
            start_sender=True,
            config=restored_cfg,
        )
        self.assertTrue(
            wait_until(
                lambda: service2.connected and self.window._remote_connected,
                30,
                app=self.app,
            ),
            "重启后的空中端未重新接入地面端",
        )
        self.assertIsNotNone(service2._session_calibration, "重启后必须自动恢复生效校准")
        self.assertEqual(len(service2._session_calibration.points), 201)
        self.assertEqual(
            service2.active_calibration_id,
            "solt_2port_20260807_202342",
            "恢复身份必须来自持久化文件名",
        )
        q2 = service2.handle_command("cal_query_active", {})
        self.assertTrue(q2["ok"])
        self.assertTrue(q2["active_osl"]["active"])
        self.assertEqual(q2["active_osl"]["ports"], ["P1", "P2"])

        # 重启后数值校正真实生效：有界连续采集窗口（count=3）——start 为真实
        # 异步命令（经真实 TCP），必须等任务真实启动后再捕获运行中的 worker。
        self.window.receiver.send_command("start", {"count": 3})
        self.assertTrue(
            wait_until(
                lambda: service2.acquisition.active_worker is not None,
                15,
                interval_s=0.005,
            ),
            "采集任务未在时限内启动（异步 start 命令未被空中端处理）",
        )
        running_worker = service2.acquisition.active_worker
        # F13 正式回归（t10）：导入的 Through 必须真实进入采集任务 spec
        # （接线可无真机验证；through 对 S21/S12 的真实数值校正需真机，诚实注明）。
        self.assertIsNotNone(running_worker, "采集任务应处于运行窗口")
        self.assertIsNotNone(
            getattr(running_worker, "through_measurement", None),
            "F13 契约：service._active_through 非 None 时采集任务 spec 的 "
            "through_measurement 必须接入（实测=None，导入 Through 未接线）",
        )
        trace_path = self._wait_any_trace(self.ground_out)
        npz = np.load(trace_path)
        freq = np.asarray(npz["frequency_hz"], dtype=np.float64)
        raw_s11 = npz_complex(npz, "raw_S11")
        disp_s11 = npz_complex(npz, "display_S11")
        expected = np.array(
            [service2._session_calibration.correct(f, v) for f, v in zip(freq, raw_s11)]
        )
        self.assertTrue(
            np.allclose(disp_s11, expected, atol=1e-7, rtol=1e-7),
            "重启恢复后校正未真实生效",
        )
        self.assertFalse(
            np.allclose(disp_s11, raw_s11, atol=1e-3),
            "校正后数值必须与 RAW 可区分",
        )
        # 停止并排空有界窗口（stop 为真实异步命令，等待服务回空闲/待命）
        self.window.receiver.send_command("stop", {})
        self.assertTrue(
            wait_until(
                lambda: service2.acquisition.active_worker is None
                and service2.acquisition.state
                in ("idle_waiting", "closed", "paused"),
                30,
                interval_s=0.02,
            ),
            f"采集未在时限内停止排空（state={service2.acquisition.state}）",
        )
        self.assertEqual(sha256_of(REAL_V2_FILE), repo_sha_before, "仓库原件哈希不得变化")

    def _wait_any_trace(self, out_dir: Path) -> Path:
        def _find():
            candidates = sorted(out_dir.rglob("trace_*.npz"))
            return candidates[0] if candidates else None

        self.assertTrue(
            wait_until(lambda: _find() is not None, 45, app=self.app),
            "未在时限内收到 trace",
        )
        return _find()

    def test_real_import_button_click_opens_dialog_and_imports(self) -> None:
        """GND-07/GND-11 真实按钮实例边界（t22 补充）：点击真实
        import_button（lambda 绑定 + 类型守卫）必须弹出文件框并走完整真实
        导入链路。文件框为 OS UI，测试以桩替换 QFileDialog 返回真实文件
        路径（非 DummyReceiver/非成功 header，导入链路全真实）。
        """
        original_dialog = QtWidgets.QFileDialog.getOpenFileName
        dialog_calls: list[tuple] = []

        def stub_dialog(*args: object, **kwargs: object) -> tuple[str, str]:
            dialog_calls.append(args)
            return (str(REAL_V2_FILE), "JSON 校准文件 (*.json)")

        QtWidgets.QFileDialog.getOpenFileName = staticmethod(stub_dialog)  # type: ignore[assignment]
        try:
            self.wizard.import_button.click()
        finally:
            QtWidgets.QFileDialog.getOpenFileName = original_dialog  # type: ignore[assignment]
        self.assertTrue(
            dialog_calls,
            "真实 import_button 点击必须触发文件框（GND-07 lambda 绑定）",
        )
        self.assertTrue(
            wait_until(lambda: self.service._session_calibration is not None, 30),
            "真实按钮触发的导入未在空中端生效",
        )
        self.assertTrue(
            wait_until(
                lambda: self.wizard._active_calibration is not None, 15, app=self.app
            ),
            "真实按钮触发的导入 ack 未到达地面",
        )
        q = self.service.handle_command("cal_query_active", {})
        self.assertTrue(q["active_osl"]["active"])
        self.assertEqual(q["active_osl"]["ports"], ["P1", "P2"])

    def test_real_export_button_click_opens_dialog_and_exports(self) -> None:
        """GND-07/GND-11 真实按钮实例边界（t22 补充）：点击真实
        export_button 必须弹出保存框并把生效校准完整落盘（导出链路全真实）。
        """
        self.assertTrue(self.wizard.import_calibration_file(str(REAL_V2_FILE)))
        self.assertTrue(
            wait_until(lambda: self.service._session_calibration is not None, 30)
        )
        export_target = self.tmp / "button_export.json"
        original_dialog = QtWidgets.QFileDialog.getSaveFileName
        dialog_calls: list[tuple] = []

        def stub_dialog(*args: object, **kwargs: object) -> tuple[str, str]:
            dialog_calls.append(args)
            return (str(export_target), "JSON 校准文件 (*.json)")

        # 注意：连接态下 export_button 点击只发请求；保存框在 ack 到达时弹出。
        # 桩必须覆盖"点击→ack→对话框"整个真实窗口。
        QtWidgets.QFileDialog.getSaveFileName = staticmethod(stub_dialog)  # type: ignore[assignment]
        try:
            self.wizard.export_button.click()
            self.assertTrue(
                wait_until(lambda: export_target.exists(), 20, app=self.app),
                "真实按钮触发的导出未落盘",
            )
        finally:
            QtWidgets.QFileDialog.getSaveFileName = original_dialog  # type: ignore[assignment]
        self.assertTrue(
            dialog_calls,
            "真实 export_button 的 ack 处理必须弹出保存框（GND-07/GND-11）",
        )
        exported = json.loads(export_target.read_text(encoding="utf-8"))
        self.assertEqual(
            exported,
            json.loads(REAL_V2_FILE.read_text(encoding="utf-8")),
            "按钮导出内容必须与原件逐字段等价",
        )

    def test_export_replay_and_unknown_ack_have_zero_side_effects(self) -> None:
        """GND-14 正式回归（t11）：真实导出消耗 pending ID 后，重放旧 ID 或
        发送未知 ID 的真实格式 ack 必须零副作用（保存框 0 次调用、既有文件
        字节不变、合法 pending 请求不被消耗）；合法无预设路径的导出经真实
        按钮 + 测试侧 QFileDialog 桩正常落盘。

        重放/未知帧均为真实格式与真实内容，仅经测试实例发送层重放
        （enqueue_calibration → 真实 TCP），无 DummyReceiver/成功 header。
        """
        self.assertTrue(self.wizard.import_calibration_file(str(REAL_V2_FILE)))
        self.assertTrue(
            wait_until(lambda: self.service._session_calibration is not None, 30)
        )
        repo_json = json.loads(REAL_V2_FILE.read_text(encoding="utf-8"))

        # 两次真实导出（各自消耗自己的 pending ID）
        path1 = self.tmp / "gnd14_a.json"
        path2 = self.tmp / "gnd14_b.json"
        self.assertTrue(self.wizard.export_calibration_file(str(path1)))
        self.assertTrue(
            wait_until(lambda: path1.exists(), 20, app=self.app), "第一次导出未落盘"
        )
        self.assertTrue(self.wizard.export_calibration_file(str(path2)))
        self.assertTrue(
            wait_until(lambda: path2.exists(), 20, app=self.app), "第二次导出未落盘"
        )
        bytes1 = path1.read_bytes()
        bytes2 = path2.read_bytes()
        self.assertEqual(json.loads(bytes1.decode("utf-8")), repo_json)
        self.assertEqual(json.loads(bytes2.decode("utf-8")), repo_json)
        # 此时两个 pending ID 均已被真实 ack 消耗
        self.assertEqual(
            dict(self.wizard._pending_export_paths),
            {},
            "前置：两次真实导出后 pending 白名单应为空",
        )

        # 捕获第一条真实 ack 帧用于原样重放
        ack1 = [
            h for h in self.collector.command_results
            if h.get("action") == "cal_export_file" and (h.get("result") or {}).get("ok")
        ][0]
        self.assertTrue(ack1.get("result", {}).get("request_id"))

        # 测试侧保存框计数桩（GND-14：重放/未知回执严禁弹框）
        dialog_calls: list[tuple] = []
        original_dialog = QtWidgets.QFileDialog.getSaveFileName

        def counting_dialog(*args: object, **kwargs: object) -> tuple[str, str]:
            dialog_calls.append(args)
            return (str(self.tmp / "should_not_exist.json"), "JSON 校准文件 (*.json)")

        QtWidgets.QFileDialog.getSaveFileName = staticmethod(counting_dialog)  # type: ignore[assignment]
        try:
            # 重放旧 ID 的真实 ack 帧（原样重放，内容真实）
            self.service.sender.enqueue_calibration(dict(ack1))
            # 未知 ID 的真实格式 ack（内容为真实原件字符串）
            self.service.sender.enqueue_calibration({
                "type": "command_ack",
                "protocol": "uav-gpr-telemetry-v1",
                "action": "cal_export_file",
                "result": {
                    "ok": True,
                    "cal_id": "cal_unknown_probe",
                    "filename": "unknown_probe.json",
                    "content": json.dumps(repo_json, ensure_ascii=False),
                    "format": "uav_gpr_solt_calibration_v2",
                    "request_id": "req_unknown_replay_999",
                    "supported": True,
                },
            })
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                self.app.processEvents()
                time.sleep(0.05)
            # GND-14 契约①：保存框 0 次调用
            self.assertEqual(
                dialog_calls,
                [],
                f"GND-14 契约：重放/未知导出回执严禁弹出保存框（实测 {len(dialog_calls)} 次）",
            )
            # GND-14 契约②：既有文件字节不变、无未知副作用文件
            self.assertEqual(path1.read_bytes(), bytes1)
            self.assertEqual(path2.read_bytes(), bytes2)
            self.assertFalse(
                (self.tmp / "should_not_exist.json").exists(),
                "重放/未知回执不得写出任何文件",
            )
        finally:
            QtWidgets.QFileDialog.getSaveFileName = original_dialog  # type: ignore[assignment]
        # GND-14 契约③：合法 pending 请求未被消耗——合法导出仍正常工作
        path3 = self.tmp / "gnd14_c.json"
        self.assertTrue(self.wizard.export_calibration_file(str(path3)))
        self.assertTrue(
            wait_until(lambda: path3.exists(), 20, app=self.app),
            "重放/未知回执后合法导出必须仍正常工作",
        )
        self.assertEqual(json.loads(path3.read_text(encoding="utf-8")), repo_json)

        # GND-14 契约④：合法无预设路径 → 真实按钮 + 测试侧保存框桩正常落盘
        button_target = self.tmp / "gnd14_button.json"
        original_button_dialog = QtWidgets.QFileDialog.getSaveFileName

        def button_dialog(*args: object, **kwargs: object) -> tuple[str, str]:
            dialog_calls.append(args)
            return (str(button_target), "JSON 校准文件 (*.json)")

        QtWidgets.QFileDialog.getSaveFileName = staticmethod(button_dialog)  # type: ignore[assignment]
        try:
            self.wizard.export_button.click()
            self.assertTrue(
                wait_until(lambda: button_target.exists(), 20, app=self.app),
                "合法无预设路径导出（真实按钮+保存框）未落盘",
            )
        finally:
            QtWidgets.QFileDialog.getSaveFileName = original_button_dialog  # type: ignore[assignment]
        self.assertEqual(
            json.loads(button_target.read_text(encoding="utf-8")),
            repo_json,
            "合法无预设路径导出的内容必须完整",
        )
        self.assertGreaterEqual(
            len(dialog_calls),
            1,
            "合法无预设路径导出必须恰好唤起保存框",
        )

    def test_legacy_single_port_and_bad_file_preserves_active(self) -> None:
        # 先导入真实 v2（生效基线）
        self.assertTrue(self.wizard.import_calibration_file(str(REAL_V2_FILE)))
        self.assertTrue(
            wait_until(lambda: self.service._session_calibration is not None, 30),
            "v2 导入未生效",
        )
        baseline_id = self.service.active_calibration_id

        # 旧版单端口 Open/Short/Load 文件（101 点，500-2000 MHz）
        legacy_path = self.tmp / "legacy_p1.json"
        freqs = np.linspace(500e6, 2000e6, 101)
        legacy_payload = {
            "format": "uav_gpr_solt_calibration_v2",
            "measurements": [
                {
                    "type": std,
                    "data": {
                        "points": [
                            {
                                "frequency": float(f),
                                "real": val[0],
                                "imag": val[1],
                            }
                            for f in freqs
                        ]
                    },
                }
                for std, val in (
                    ("Open", (0.90, -0.03)),
                    ("Short", (-0.88, 0.04)),
                    ("Load", (0.03, 0.02)),
                )
            ],
        }
        legacy_path.write_text(json.dumps(legacy_payload), encoding="utf-8")
        self.assertTrue(self.wizard.import_calibration_file(str(legacy_path)))
        self.assertTrue(
            wait_until(
                lambda: self.service.active_calibration_id == "legacy_p1", 30
            ),
            "旧单端口文件未生效",
        )
        self.wizard.query_active_calibration()
        self.assertTrue(
            wait_until(
                lambda: (self.wizard._active_calibration or {}).get("ports") == ["P1"],
                15,
                app=self.app,
            ),
            "旧单端口范围必须如实显示为 P1",
        )
        self.assertNotIn("P2", self.wizard.active_cal_label.text(), "不得伪造 P2")
        q = self.service.handle_command("cal_query_active", {})
        self.assertEqual(q["active_osl"]["ports"], ["P1"])
        self.assertFalse(q["active_osl"]["has_through"])

        # 坏文件（NaN）导入被原子拒绝，旧校准完整保留
        bad_path = self.tmp / "bad_nan.json"
        bad_payload = {
            "format": "uav_gpr_solt_calibration_v2",
            "measurements": [
                {
                    "type": "P1_Open",
                    "data": {"points": [{"frequency": 1e9, "real": float("nan"), "imag": 0.0}]},
                },
                {
                    "type": "P1_Short",
                    "data": {"points": [{"frequency": 1e9, "real": -1.0, "imag": 0.0}]},
                },
                {
                    "type": "P1_Load",
                    "data": {"points": [{"frequency": 1e9, "real": 0.0, "imag": 0.0}]},
                },
            ],
        }
        bad_path.write_text(json.dumps(bad_payload), encoding="utf-8")
        self.assertTrue(self.wizard.import_calibration_file(str(bad_path)))
        self.assertTrue(
            wait_until(
                lambda: "导入失败" in self.wizard.status_label.text(), 15, app=self.app
            ),
            f"坏文件拒绝原因未在地面可见（实际: {self.wizard.status_label.text()}）",
        )
        self.assertIn(
            "空中端拒绝校准文件", self.wizard.status_label.text(),
            "拒绝原因必须来自空中端真实校验",
        )
        self.assertEqual(
            self.service.active_calibration_id, "legacy_p1", "坏文件不得破坏旧生效校准"
        )
        q2 = self.service.handle_command("cal_query_active", {})
        self.assertTrue(q2["active_osl"]["active"])
        self.assertEqual(q2["active_osl"]["points"], 101)
        self.assertNotEqual(q2["active_osl"]["cal_id"], baseline_id)
        # 频带不兼容（频率轴不一致/非单调）同样拒绝且保留
        bad_axis_path = self.tmp / "bad_axis.json"
        bad_axis = {
            "format": "uav_gpr_solt_calibration_v2",
            "measurements": [
                {
                    "type": "Open",
                    "data": {"points": [
                        {"frequency": 5e8, "real": 0.9, "imag": 0.0},
                        {"frequency": 4e8, "real": 0.8, "imag": 0.1},
                    ]},
                },
                {
                    "type": "Short",
                    "data": {"points": [
                        {"frequency": 5e8, "real": -0.9, "imag": 0.0},
                        {"frequency": 2e9, "real": -0.8, "imag": 0.1},
                    ]},
                },
                {
                    "type": "Load",
                    "data": {"points": [
                        {"frequency": 5e8, "real": 0.0, "imag": 0.0},
                        {"frequency": 2e9, "real": 0.0, "imag": 0.0},
                    ]},
                },
            ],
        }
        bad_axis_path.write_text(json.dumps(bad_axis), encoding="utf-8")
        self.assertTrue(self.wizard.import_calibration_file(str(bad_axis_path)))
        self.assertTrue(
            wait_until(
                lambda: "空中端拒绝校准文件" in self.wizard.status_label.text(), 15, app=self.app
            ),
            f"频带不兼容拒绝未可见（实际: {self.wizard.status_label.text()}）",
        )
        self.assertEqual(self.service.active_calibration_id, "legacy_p1")


class TestR8_7_8_Recapture(_E2ECase):
    """R8-7/8：受控重采、续采、已解算后重采、审计证据与计数一致性。"""

    def test_recapture_p1_open_continue_after_solve_and_audit(self) -> None:
        cal_id = self._begin_osl()

        self._capture_and_wait_done("P1_Open")
        capture_dir = self.service.output_root / "_calibration" / air.safe_cal_dir_name(cal_id)
        first_files = sorted(capture_dir.glob("capture_P1_Open_1.npz")) + sorted(
            capture_dir.glob("capture_P1_Open_2.npz")
        )
        self.assertEqual(len(first_files), 2, "repeats=2 首轮应有两条原始采样")
        first_hashes = {p.name: sha256_of(p) for p in first_files}

        # 单击重采：recapture=True + request_id（真实链路）
        self.assertEqual(self.wizard.recapture_combo.currentText(), "P1_Open")
        self.wizard.recapture_selected_step()
        self.assertTrue(self.wizard._capturing)
        self.assertTrue(
            wait_until(
                lambda: self._row_text("P1_Open") == "已完成" and not self.wizard._capturing,
                60,
                app=self.app,
            ),
            f"重采未完成，行文本={self._row_text('P1_Open')}",
        )
        # 下一步仍是 P1_Short（故障三修复证明）
        self.assertEqual(self.wizard.current_step_name(), "P1_Short")

        # 审计证据：r2 批次存在且首轮文件逐字节未变
        r2_files = sorted(capture_dir.glob("capture_P1_Open_r2_*.npz"))
        self.assertEqual(len(r2_files), 2, "重采应有独立 r2 审计文件")
        for path in first_files:
            self.assertEqual(
                sha256_of(path), first_hashes[path.name], "首轮原始采样证据严禁覆盖"
            )

        # 重采后均值必须来自新批次（数值可区分：新偏置 +0.90-0.03j）
        session = self.service._calibration_session
        progress = session.step_progress("P1_Open")
        self.assertEqual(progress.attempt, 2)
        self.assertEqual(len(progress.history_captures), 1, "首轮采样应归档为历史证据")
        self.assertEqual(len(progress.captures), 2, "当前批次只含最新采样")

        # 续采 P1_Short 正常
        self._capture_and_wait_done("P1_Short")
        self.assertEqual(self.wizard._steps_done, 2)

        # 完成全部 → 解算 → 已解算状态重采 P1_Load：旧解算作废、重新解算用新数据
        for step in ("P1_Load", "P2_Open", "P2_Short", "P2_Load"):
            self._capture_and_wait_done(step)
        self._finish_and_solve(cal_id)
        old_solved = self.service._calibration_session.solved_result()
        self.assertIsNotNone(old_solved)

        self.wizard.recapture_combo.setCurrentText("P1_Load")
        self.wizard.recapture_selected_step()
        self.assertFalse(self.wizard._solved, "重采发起即作废地面旧解算状态")
        self.assertFalse(self.wizard.activate_button.isEnabled())
        self.assertTrue(
            wait_until(
                lambda: self._row_text("P1_Load") == "已完成" and not self.wizard._capturing,
                60,
                app=self.app,
            ),
            "已解算后重采未完成",
        )
        session = self.service._calibration_session
        self.assertEqual(session.state, cal.CalibrationSession.STATE_IN_PROGRESS)
        self.assertIsNone(session.solved_result(), "空中端旧解算必须作废")
        self.assertEqual(session.step_progress("P1_Load").attempt, 2)
        # 其他已完成步骤保留
        for step in ("P1_Open", "P1_Short", "P2_Open", "P2_Short", "P2_Load"):
            self.assertEqual(session.step_progress(step).state, "done")
        # 重新解算成功
        self._finish_and_solve(cal_id)
        new_solved = self.service._calibration_session.solved_result()
        self.assertIsNotNone(new_solved)
        p1_load_mean_new = session.step_progress("P1_Load").mean
        self.assertIsNotNone(p1_load_mean_new)

    def test_inflight_duplicate_rejection_keeps_current_request_locked(self) -> None:
        """F2 正式回归（t10）：在途期间到达的重复/陈旧 cal_capture 拒绝
        （ack 与 calibration_result 均缺 cal_id/request_id 关联字段）不得解锁
        当前在途请求——行保持"请求采集中/采集中"、_capturing 保持 True、
        pending request 不变，直到真实 step_done 收尾。

        修复前现状 = 陈旧拒绝到达后地面立即解锁并回退在途行
        （本用例红，修复后转绿）。
        """
        cal_id = self._begin_osl(repeats=2, ifbw=100)
        self.wizard.capture_next_step()
        req_id_first = self.wizard._pending_request_id
        self.assertEqual(self.wizard.current_step_name(), "P1_Open")
        captures_baseline = self.collector.count_cal(
            type="calibration_capture", cal_id=cal_id
        )
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cal(
                    type="calibration_capture", cal_id=cal_id
                )
                > captures_baseline,
                30,
                app=self.app,
            ),
            "采集在途窗口未建立（首条 sweep 未到达）",
        )
        # 双击防抖：在途期间再次点击"采集下一步"必须被按钮守卫拦截（不重复发送）
        self.wizard.capture_next_step()
        self.assertEqual(
            self.wizard._pending_request_id, req_id_first, "双击不得发起新请求"
        )
        # 在途期间同步骤重复命令（新 request_id）：空中真实拒绝
        self.window.receiver.send_command(
            "cal_capture",
            {
                "cal_id": cal_id,
                "step": "P1_Open",
                "recapture": True,
                "request_id": "req_dup_inflight_001",
            },
        )
        rejected_baseline = self.collector.count_cal(
            type="calibration_result", status="rejected", cal_id=cal_id
        )
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cal(
                    type="calibration_result", status="rejected", cal_id=cal_id
                )
                > rejected_baseline,
                15,
                app=self.app,
            ),
            "在途重复采集命令应被空中真实拒绝",
        )
        # F2 契约：陈旧拒绝不得解锁当前请求（修复前现状=立即解锁并回退行，红）
        self.assertTrue(
            self.wizard._capturing,
            "F2 契约：陈旧/重复命令的拒绝不得解锁当前在途请求（实测被解锁）",
        )
        self.assertEqual(
            self.wizard._pending_request_id,
            req_id_first,
            "F2 契约：当前在途请求的 request_id 不得被陈旧拒绝清除",
        )
        self.assertIn(
            self._row_text("P1_Open"),
            ("请求采集中…", "采集中 1/2"),
            f"F2 契约：在途行不得被陈旧拒绝回退（实测={self._row_text('P1_Open')!r}）",
        )
        # 真实 step_done 收尾后状态一致：完成、指针 P1_Short
        self.assertTrue(
            wait_until(
                lambda: self._row_text("P1_Open") == "已完成"
                and not self.wizard._capturing
                and "P1_Open" in self.wizard._completed_steps,
                90,
                app=self.app,
            ),
            "慢采集应真实完成（step_done 收尾）",
        )
        self.assertEqual(self.wizard.current_step_name(), "P1_Short")
        self._capture_and_wait_done("P1_Short")
        self.assertEqual(self.wizard._steps_done, 2)

    def test_pending_step_recapture_rejection_restores_pointer(self) -> None:
        """F9（同根）正式回归（t10）：重采被真实拒绝后，rejected 分支必须把
        下一步指针恢复到首个未完成步骤，而非停留在重采目标。

        修复前现状 = recapture_step 先改 _step_index、拒绝后不回退
        （本用例红，修复后转绿）。
        """
        cal_id = self._begin_osl(repeats=2, ifbw=100)
        self.wizard.recapture_combo.addItem("P1_Short", "P1_Short")
        self.wizard.recapture_combo.setCurrentText("P1_Short")
        self.wizard.recapture_step("P1_Short")
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cal(
                    type="calibration_result", status="rejected", cal_id=cal_id
                )
                >= 1,
                15,
                app=self.app,
            ),
            "未采集步骤的重采应被空中真实拒绝",
        )
        self.assertFalse(self.wizard._capturing)
        self.assertEqual(self._row_text("P1_Short"), "待采集")
        self.assertIn("拒绝", self._row_text("P1_Short", 3))
        # F9 契约：拒绝后指针必须回到首个未完成步骤 P1_Open
        self.assertEqual(
            self.wizard.current_step_name(),
            "P1_Open",
            "F9 契约：重采被拒后下一步指针必须恢复首个未完成步骤"
            f"（实测={self.wizard.current_step_name()!r}，停留在重采目标）",
        )
        # 恢复后普通下一步必须能正常发送并完成 P1_Open
        self._capture_and_wait_done("P1_Open")
        self.assertEqual(self.wizard.current_step_name(), "P1_Short")

    def test_recapture_rejected_then_normal_next_step_is_p1_short(self) -> None:
        """F9 正式回归（t10，用户路径）：正常完成 P1_Open 后单击"重采该步骤"
        遇到空中真实拒绝（采集线程在途），随后普通"采集下一步"必须仍是
        P1_Short——不得回到 P1_Open 形成拒绝循环。

        修复前现状 = 指针被重采目标劫持回 P1_Open（本用例红，修复后转绿）。
        忙碌源为真实在途采集（真实链路命令），拒绝来自空中真实互斥。
        """
        cal_id = self._begin_osl(repeats=2, ifbw=100)
        self._capture_and_wait_done("P1_Open")
        self.assertEqual(self.wizard.current_step_name(), "P1_Short")

        # 制造真实忙碌：下发一条真实 P1_Short 采集（空中真实线程在途）
        self.window.receiver.send_command(
            "cal_capture",
            {"cal_id": cal_id, "step": "P1_Short", "recapture": False,
             "request_id": "req_busy_maker_001"},
        )
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cal(
                    type="calibration_capture", cal_id=cal_id
                )
                >= 1,
                30,
                app=self.app,
            ),
            "忙碌采集未启动",
        )
        # 用户单击"重采该步骤"（P1_Open 已完成，在 combo 中）：空中真实拒绝
        self.assertEqual(self.wizard.recapture_combo.currentText(), "P1_Open")
        self.wizard.recapture_selected_step()
        rejected_baseline = self.collector.count_cal(
            type="calibration_result", status="rejected", cal_id=cal_id
        )
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cal(
                    type="calibration_result", status="rejected", cal_id=cal_id
                )
                > rejected_baseline,
                15,
                app=self.app,
            ),
            "在途期间的重采必须被空中真实拒绝",
        )
        # F9 契约：拒绝后普通下一步必须仍是 P1_Short（首个未完成步骤）
        self.assertEqual(
            self.wizard.current_step_name(),
            "P1_Short",
            "F9 契约：重采被拒后普通下一步必须仍是 P1_Short"
            f"（实测={self.wizard.current_step_name()!r}，被劫持回 P1_Open）",
        )
        # 忙碌采集真实收尾后（其 step_done 属于当前会话，地面如实入集合），
        # 普通下一步继续推进且不形成拒绝循环
        self.assertTrue(
            wait_until(
                lambda: self._row_text("P1_Short") == "已完成"
                and "P1_Short" in self.wizard._completed_steps,
                90,
                app=self.app,
            ),
            "忙碌采集应真实完成并同步地面",
        )
        self.assertNotEqual(
            self.wizard.current_step_name(),
            "P1_Open",
            "F9 契约：收尾后普通下一步不得回到 P1_Open 形成拒绝循环",
        )


class TestR8_9_Robustness(_E2ECase):
    """R8-9：幂等重放、跨会话隔离、watchdog busy 查询与预算。"""

    def test_same_request_id_replay_does_not_recapture(self) -> None:
        cal_id = self._begin_osl()
        req_id = self._capture_and_wait_done("P1_Open")
        self.assertTrue(req_id)
        capture_dir = self.service.output_root / "_calibration" / air.safe_cal_dir_name(cal_id)
        files_before = sorted(p.name for p in capture_dir.glob("capture_P1_Open_*.npz"))
        steps_done_before = self.wizard._steps_done

        # 相同 request_id 重发：空中回放缓存回执，绝不二次采集
        self.window.receiver.send_command(
            "cal_capture",
            {"cal_id": cal_id, "step": "P1_Open", "recapture": True, "request_id": req_id},
        )
        self.assertTrue(
            wait_until(lambda: self.service._recent_requests.get(req_id) is not None, 10),
            "完成的 request_id 必须在空中幂等缓存中",
        )
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(0.05)
        files_after = sorted(p.name for p in capture_dir.glob("capture_P1_Open_*.npz"))
        self.assertEqual(files_after, files_before, "幂等重放严禁产生新采样文件")
        self.assertEqual(self.wizard._steps_done, steps_done_before, "重放不得重复计数")
        self.assertEqual(self.wizard.current_step_name(), "P1_Short")

    def test_cross_session_and_stale_commands_cannot_hijack_active_capture(self) -> None:
        cal_id_a = self._begin_osl()
        self._capture_all_osl_steps()
        self._finish_and_solve(cal_id_a)
        self._activate(cal_id_a)

        # 新会话 B（真实链路，repeats=2 + ifbw=100 提供确定性在途窗口）
        cal_id_b = self._begin_osl(repeats=2, ifbw=100)
        self.assertNotEqual(cal_id_a, cal_id_b)

        # 会话空闲时携带 A 的旧 cal_id 采集命令：空中拒绝（防跨会话错配）
        rejected_acks_baseline = self.collector.count_cmd(action="cal_capture", result={"ok": False})
        self.window.receiver.send_command(
            "cal_capture",
            {"cal_id": cal_id_a, "step": "P1_Short", "recapture": False,
             "request_id": "stale_req_idle"},
        )
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cmd(action="cal_capture", result={"ok": False})
                > rejected_acks_baseline,
                15,
                app=self.app,
            ),
            "跨会话采集命令必须被拒绝",
        )
        self.assertFalse(self.wizard._capturing)

        # 在途期间迟到异 cal_id 命令：实测是否劫持在途请求（B 会话慢步进在途窗口）
        try:
            self.wizard.capture_next_step()
            self.assertEqual(self.wizard.current_step_name(), "P1_Open")
            current_request_id = self.wizard._pending_request_id
            self.assertTrue(current_request_id, "当前在途请求应有 request_id")
            captures_baseline_b = self.collector.count_cal(
                type="calibration_capture", cal_id=cal_id_b
            )
            self.assertTrue(
                wait_until(
                    lambda: self.collector.count_cal(
                        type="calibration_capture", cal_id=cal_id_b
                    )
                    > captures_baseline_b,
                    30,
                    app=self.app,
                ),
                "采集在途窗口未建立",
            )
            stale_rejected_baseline = self.collector.count_cal(
                type="calibration_result", status="rejected", cal_id=cal_id_a
            )
            self.window.receiver.send_command(
                "cal_capture",
                {"cal_id": cal_id_a, "step": "P1_Open", "recapture": True,
                 "request_id": "stale_req_inflight"},
            )
            self.assertTrue(
                wait_until(
                    lambda: self.collector.count_cal(
                        type="calibration_result", status="rejected", cal_id=cal_id_a
                    )
                    > stale_rejected_baseline,
                    15,
                    app=self.app,
                ),
                "跨会话迟到命令应被空中拒绝（calibration_result 带 A 的 cal_id）",
            )
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                self.app.processEvents()
                time.sleep(0.05)
            stale_ack_observed = (
                self.collector.count_cmd(action="cal_capture", result={"ok": False})
                > rejected_acks_baseline
            )
            # F1/F2 正式回归（t10，t22 夹具校正）：跨会话陈旧命令的拒绝不得
            # 解锁或回退 B 会话的在途请求。在途行以"真实当前请求"为依据：
            # 请求未变 + 合法在途状态（请求采集中… 或 真实采样进度 采集中 i/2）
            # 均属未解锁；严禁回退为"待采集"或在备注列残留拒绝。
            row_after_stale = self._row_text("P1_Open")
            legitimate_inflight = (
                row_after_stale == "请求采集中…"
                or row_after_stale.startswith("采集中")
            )
            self.assertTrue(
                self.wizard._capturing,
                "F1/F2 契约：跨会话陈旧拒绝不得解锁 B 会话在途请求"
                "（实测 _capturing 被置 False）",
            )
            self.assertEqual(
                self.wizard._pending_request_id,
                current_request_id,
                "F1/F2 契约：当前在途请求的 request_id 不得被陈旧拒绝替换"
                f"（实测={self.wizard._pending_request_id!r}，stale_ack={stale_ack_observed}）",
            )
            self.assertTrue(
                legitimate_inflight,
                "F1/F2 契约：B 会话在途行必须保持当前请求的合法在途状态"
                f"（实测={row_after_stale!r}；合法=请求采集中…或真实采样进度）",
            )
            self.assertNotEqual(
                row_after_stale,
                "待采集",
                "F1/F2 契约：在途行不得被陈旧拒绝回退为待采集",
            )
            self.assertNotIn(
                "拒绝",
                self._row_text("P1_Open", 3),
                "F1/F2 契约：陈旧拒绝不得在当前请求的备注列留下拒绝痕迹"
                f"（实测={self._row_text('P1_Open', 3)!r}）",
            )
        finally:
            self.wizard.repeats.setValue(2)
        # 真实 step_done 仍按 B 会话推进，跨会话命令未破坏新请求
        self.assertTrue(
            wait_until(
                lambda: self._row_text("P1_Open") == "已完成"
                and not self.wizard._capturing
                and "P1_Open" in self.wizard._completed_steps,
                90,
                app=self.app,
            ),
            f"B 会话慢采集未真实完成（行={self._row_text('P1_Open')}）",
        )
        self.assertEqual(self.wizard.current_step_name(), "P1_Short")

    def test_watchdog_timeout_queries_busy_remote_and_recovers(self) -> None:
        cal_id = self._begin_osl(repeats=2, ifbw=100)
        query_acks_baseline = self.collector.count_cmd(action="cal_query_active")

        # 第 1 条样本到达后仍有约 8s 真实扫频（第 2 条步进），在途窗口
        try:
            self.wizard.capture_next_step()
            self.assertTrue(self.wizard._capturing)
            captures_baseline = self.collector.count_cal(
                type="calibration_capture", cal_id=cal_id
            )
            self.assertTrue(
                wait_until(
                    lambda: self.collector.count_cal(
                        type="calibration_capture", cal_id=cal_id
                    )
                    > captures_baseline,
                    30,
                    app=self.app,
                ),
                "采集在途窗口未建立",
            )
            # 看门狗超时处理（真实发送 cal_query_active）：远端 busy 必须可见
            self.wizard._on_capture_watchdog_timeout()
            self.assertTrue(
                wait_until(
                    lambda: self.collector.count_cmd(action="cal_query_active")
                    > query_acks_baseline,
                    15,
                    app=self.app,
                ),
                "watchdog 超时必须真实查询远端状态",
            )
            # captain 风险1 实测：校准 worker 在途时 cal_query_active 不得被互斥拒绝
            q = self.service.handle_command("cal_query_active", {})
            self.assertTrue(
                q["in_progress_session"]["busy"],
                f"实测采集在途时 busy=False（诊断: service={self.service.acquisition.statistics()} "
                f"thread={self.service._calibration_thread} logs={self.air_logs[-6:]}）",
            )
            self.assertEqual(q["in_progress_session"]["active_request_id"],
                             self.wizard._pending_request_id)
            self.assertIn("忙碌", self.wizard.status_label.text())
            self.assertTrue(self.wizard._capturing, "远端 busy 时保持锁定，绝不放开重试")
        finally:
            self.wizard.repeats.setValue(2)
        # 完成后 step_done 正常收尾
        self.assertTrue(
            wait_until(
                lambda: self._row_text("P1_Open") == "已完成"
                and not self.wizard._capturing
                and "P1_Open" in self.wizard._completed_steps,
                90,
                app=self.app,
            ),
            "慢采集应真实完成（step_done 收尾）",
        )
        self.assertFalse(self.wizard._capturing)

        # 第二个采集在途时再次超时：busy → 释放 → step_done 恢复（可诊断可恢复）
        try:
            self.wizard.capture_next_step()
            self.assertEqual(self.wizard.current_step_name(), "P1_Short")
            captures_baseline2 = self.collector.count_cal(
                type="calibration_capture", cal_id=cal_id
            )
            self.assertTrue(
                wait_until(
                    lambda: self.collector.count_cal(
                        type="calibration_capture", cal_id=cal_id
                    )
                    > captures_baseline2,
                    30,
                    app=self.app,
                ),
                "第二采集在途窗口未建立",
            )
            self.wizard._on_capture_watchdog_timeout()
            self.assertTrue(
                wait_until(lambda: "忙碌" in self.wizard.status_label.text(), 15, app=self.app)
            )
        finally:
            self.wizard.repeats.setValue(2)
        self.assertTrue(
            wait_until(
                lambda: self._row_text("P1_Short") == "已完成"
                and not self.wizard._capturing
                and "P1_Short" in self.wizard._completed_steps,
                90,
                app=self.app,
            ),
            "watchdog 查询后释放的采集必须正常收尾",
        )
        self.assertFalse(self.wizard._capturing)
        self.assertEqual(self.wizard._steps_done, 2)

    def test_stale_query_ack_does_not_hijack_inflight_capture(self) -> None:
        """F1 正式回归（t10）：陈旧 query 回执（busy=False 快照）迟到于在途
        采集时，严禁释放在途请求——行保持"请求采集中/采集中"、_capturing
        保持 True、pending request 不变，直到真实 step_done 收尾。

        构造（真实链路，确定性）：先发 cal_query_active（空中在采集开始前
        处理 → 回执快照 busy=False），随后立即发起 repeats=2 + ifbw=100 的
        慢步进真实采集——该"陈旧回执"必然在采集在途期间到达地面。

        修复前现状 = 陈旧回执到达即错误释放（_capturing=False、行被回退为
        "超时已释放"）（本用例红，修复后转绿：保留恢复判据）。
        """
        cal_id = self._begin_osl(repeats=2, ifbw=100)
        # 查询先于采集被空中处理：回执必然携带 busy=False 快照
        self.window.receiver.send_command("cal_query_active", {})
        try:
            self.wizard.capture_next_step()
            self.assertEqual(self.wizard.current_step_name(), "P1_Open")
            captures_baseline = self.collector.count_cal(
                type="calibration_capture", cal_id=cal_id
            )
            self.assertTrue(
                wait_until(
                    lambda: self.collector.count_cal(
                        type="calibration_capture", cal_id=cal_id
                    )
                    > captures_baseline,
                    30,
                    app=self.app,
                ),
                "采集在途窗口未建立",
            )
            # 给陈旧回执留出真实到达并被处理的时间
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                self.app.processEvents()
                time.sleep(0.05)
            # F1 契约：陈旧回执不得释放在途请求（修复前现状=被释放，红）
            self.assertTrue(
                self.wizard._capturing,
                "F1 契约：陈旧 cal_query_active 回执不得释放在途采集"
                "（实测 _capturing 被置 False）",
            )
            self.assertEqual(
                self.wizard._pending_step,
                "P1_Open",
                "F1 契约：在途请求的 pending 步骤不得被陈旧回执清除"
                f"（实测={self.wizard._pending_step!r}）",
            )
            self.assertIn(
                self._row_text("P1_Open"),
                ("请求采集中…", "采集中 1/2"),
                f"F1 契约：在途行不得被陈旧回执回退（实测={self._row_text('P1_Open')!r}）",
            )
        finally:
            self.wizard.repeats.setValue(2)
        # 恢复判据：真实 step_done 后状态一致——不假成功、不永久请求中
        self.assertTrue(
            wait_until(
                lambda: self._row_text("P1_Open") == "已完成"
                and not self.wizard._capturing
                and "P1_Open" in self.wizard._completed_steps,
                90,
                app=self.app,
            ),
            "陈旧回执干扰后真实完成未恢复",
        )
        self.assertEqual(self.wizard.current_step_name(), "P1_Short")
        # 无重复执行：P1_Open 首轮 attempt=1 的采样文件必须唯一（repeats=1 → 1 条；
        # 重复执行会产生 r2 批次或重复首轮文件）
        capture_dir = self.service.output_root / "_calibration" / air.safe_cal_dir_name(cal_id)
        self.assertEqual(
            len(list(capture_dir.glob("capture_P1_Open_1.npz"))),
            1,
            "陈旧回执不得引发重复采集（首轮文件必须唯一）",
        )
        self.assertEqual(
            len(list(capture_dir.glob("capture_P1_Open_r2_*.npz"))),
            0,
            "陈旧回执不得触发重采批次",
        )

    def test_stale_watchdog_reply_cannot_unlock_new_capture(self) -> None:
        """GND-13 正式回归（t11）：上一步的 watchdog 查询回执在新步骤开始后
        到达，绝不得解锁新请求、消耗新 token 或改写新步骤行；当前新查询仍能
        正常恢复。

        构造（真实链路，确定性）：P1_Open 在途时触发看门狗查询（token/上下文
        真实下发），用**自己服务实例的命令回调桩**把该查询延迟到 P1_Open
        权威 step_done 之后、P1_Short 启动之前再由真实 handler 计算回执
        （真实快照 + 生产窄关联的 request_id 回带），经真实发送层
        （enqueue → 真实 TCP）重放。帧内容真实，仅投递时机受控。
        """
        cal_id = self._begin_osl(repeats=2, ifbw=100)
        self.wizard.capture_next_step()
        req_open = self.wizard._pending_request_id
        self.assertEqual(self.wizard.current_step_name(), "P1_Open")
        captures_baseline = self.collector.count_cal(
            type="calibration_capture", cal_id=cal_id
        )
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cal(
                    type="calibration_capture", cal_id=cal_id
                )
                > captures_baseline,
                60,
                app=self.app,
            ),
            "采集在途窗口未建立",
        )
        # 在途窗口内触发看门狗查询：真实 token/上下文下发
        self.wizard._on_capture_watchdog_timeout()
        token1 = self.wizard._watchdog_query_token
        self.assertTrue(token1, "看门狗查询必须签发真实 token")
        self.assertEqual(
            self.wizard._watchdog_query_context,
            (cal_id, req_open, "P1_Open"),
            "看门狗查询上下文必须绑定当前会话/请求/步骤",
        )
        # 自己服务实例的命令回调桩：延迟 cal_query_active 到指定时刻再由真实
        # handler 计算（期间仅回真实最小 ack，无 result，地面无害忽略）。
        original_handle = self.service.handle_command
        deferred: list[tuple[str, dict]] = []
        defer_state = {"deferring": True}

        def deferring_handle(action: str, params: object) -> dict:
            if defer_state["deferring"] and action == "cal_query_active":
                deferred.append((action, dict(params) if isinstance(params, dict) else {}))
                return {"supported": True}
            return original_handle(action, params)

        self.service.handle_command = deferring_handle  # type: ignore[method-assign]
        try:
            # P1_Open 权威收尾（真实 step_done 走真实 TCP；defer 只拦查询）
            self.assertTrue(
                wait_until(
                    lambda: self._row_text("P1_Open") == "已完成"
                    and not self.wizard._capturing
                    and "P1_Open" in self.wizard._completed_steps,
                    90,
                    app=self.app,
                ),
                "P1_Open 未真实完成",
            )
            self.assertIsNone(
                self.wizard._watchdog_query_token,
                "step_done 收尾必须失效旧查询 token（GND-13 生命周期边界）",
            )
            # 启动新步骤 P1_Short（真实采集在途）
            self.wizard.capture_next_step()
            new_request_id = self.wizard._pending_request_id
            self.assertEqual(self.wizard.current_step_name(), "P1_Short")
            self.assertNotEqual(new_request_id, req_open)
            # 释放：真实 handler 计算当前快照，经真实发送层重放
            # （request_id 回带与生产 RemoteSender 窄关联一致）
            defer_state["deferring"] = False
            self.service.handle_command = original_handle  # type: ignore[method-assign]
            for action, params in deferred:
                real_result = dict(original_handle(action, params))
                real_result.setdefault("supported", True)
                real_result["request_id"] = params.get("request_id")
                self.service.sender.enqueue_calibration({
                    "type": "command_ack",
                    "protocol": "uav-gpr-telemetry-v1",
                    "action": action,
                    "result": real_result,
                })
            self.assertTrue(
                wait_until(
                    lambda: self.collector.count_cmd(action="cal_query_active") >= 1,
                    15,
                    app=self.app,
                ),
                "重放回执未到达地面",
            )
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                self.app.processEvents()
                time.sleep(0.05)
            # GND-13 契约：陈旧回执不得解锁新请求/消耗新 token/改写新步骤行
            self.assertTrue(
                self.wizard._capturing,
                "GND-13 契约：上一步查询回执不得解锁新步骤在途采集",
            )
            self.assertEqual(
                self.wizard._pending_request_id,
                new_request_id,
                "GND-13 契约：新步骤请求不得被陈旧回执清除",
            )
            self.assertIsNone(
                self.wizard._watchdog_query_token,
                "GND-13 契约：陈旧回执不得消耗/复活新步骤的查询 token",
            )
            self.assertTrue(
                self._row_text("P1_Short") == "请求采集中…"
                or self._row_text("P1_Short").startswith("采集中"),
                f"GND-13 契约：新步骤行必须保持合法在途状态"
                f"（实测={self._row_text('P1_Short')!r}）",
            )
            self.assertNotIn("拒绝", self._row_text("P1_Short", 3))
            # 当前新查询仍能恢复：真实触发看门狗查询（不再延迟），busy=true
            # 保持等待；随后真实 step_done 完成收尾
            self.wizard._on_capture_watchdog_timeout()
            token2 = self.wizard._watchdog_query_token
            self.assertTrue(token2 and token2 != token1, "新步骤看门狗查询必须签发新 token")
            self.assertTrue(
                wait_until(
                    lambda: "忙碌" in self.wizard.status_label.text(), 15, app=self.app
                ),
                f"当前新查询未获真实回执（token2={token2}）",
            )
            self.assertTrue(self.wizard._capturing, "当前查询 busy 期间保持锁定")
        finally:
            defer_state["deferring"] = False
            self.service.handle_command = original_handle  # type: ignore[method-assign]
        self.assertTrue(
            wait_until(
                lambda: self._row_text("P1_Short") == "已完成"
                and not self.wizard._capturing
                and "P1_Short" in self.wizard._completed_steps,
                90,
                app=self.app,
            ),
            "P1_Short 未真实完成",
        )
        self.assertEqual(self.wizard.current_step_name(), "P1_Load")

    def test_watchdog_budget_scales_for_long_scans(self) -> None:
        cal_id = self._begin_osl()
        # 默认参数（201 点 / IFBW 10k / repeats 2）→ 预算下限 30s
        self.wizard.capture_next_step()
        interval_default = self.wizard._capture_watchdog_timer.interval()
        self.assertGreaterEqual(interval_default, 30_000)
        self.assertTrue(
            wait_until(lambda: self._row_text("P1_Open") == "已完成", 60, app=self.app)
        )
        # 合法长扫描（repeats=256）：预算必须随扫频耗时放大，而非固定误判
        self.wizard.repeats.setValue(256)
        self.wizard._start_capture_watchdog()
        interval_long = self.wizard._capture_watchdog_timer.interval()
        self.assertGreater(interval_long, 30_000, "长扫描预算必须大于 30s 下限")
        self.assertGreater(
            interval_long,
            interval_default,
            "预算必须随扫描规模增长（动态滑动看门狗）",
        )
        self.wizard.cancel_session()


class TestR8_10_DisconnectLegacy(_E2ECase):
    """R8-10：断线重连同步；旧空中端（无 step_done/不支持导入）降级可见。"""

    def test_disconnect_reconnect_resync_session_without_hang(self) -> None:
        cal_id = self._begin_osl()
        self._capture_and_wait_done("P1_Open")

        # 物理断链：关闭地面侧已连接 socket（真实 TCP 断开）
        with self.window.receiver._sock_lock:
            client = self.window.receiver._client
        self.assertIsNotNone(client)
        client.close()
        self.assertTrue(
            wait_until(lambda: not self.window._remote_connected, 15, app=self.app),
            "地面未在时限内感知断线",
        )
        self.assertFalse(self.wizard.begin_button.isEnabled())

        # 空中端自动重连（注入短退避 0.3s→0.8s）
        self.assertTrue(
            wait_until(lambda: self.window._remote_connected, 30, app=self.app),
            "空中端未在时限内真实重连",
        )
        # 会话仍在空中进行中 → 向导保持会话锁定（不得盲目解锁重发），
        # 但采集控制恢复可用
        self.assertTrue(self.wizard._connected)
        self.assertIsNotNone(self.wizard._cal_id, "会话身份不得因断线丢失")

        # 重连后先同步远端真实会话/步骤/能力（而非盲目解锁重发）。
        # 计数基线等待"本次查询"回执被处理，避免陈旧回执在途释放。
        query_acks_baseline = self.collector.count_cmd(action="cal_query_active")
        self.wizard.query_active_calibration()
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cmd(action="cal_query_active")
                > query_acks_baseline,
                15,
                app=self.app,
            ),
            "重连查询未获回执",
        )
        self.assertTrue(
            wait_until(
                lambda: "P1_Open" in self.wizard._completed_steps, 15, app=self.app
            ),
            "重连后步骤集合必须从远端真实状态同步（P1_Open 应恢复已完成）",
        )
        self.assertEqual(self.wizard.current_step_name(), "P1_Short")
        q = self.service.handle_command("cal_query_active", {})
        self.assertTrue(q["in_progress_session"]["active"])
        self.assertEqual(q["in_progress_session"]["cal_id"], cal_id)

        # 同步后续采 P1_Short 正常完成（会话未因断线损坏）
        self._capture_and_wait_done("P1_Short")
        self.assertEqual(self.wizard._steps_done, 2)

    def test_unknown_capability_must_not_unlock_on_last_sample(self) -> None:
        """F8 正式回归（t10，t22 夹具校正）：能力未知（本会话从未查询）时，
        末样本到达不得直接解锁步骤——必须以新端权威收尾（step_done）为准；
        旧端可见降级属于显式能力协商结果，不属于"未知即旧端"。

        t22 夹具校正：末样本后真实 step_done 会立即到达，直接断言
        "_capturing 仍为 True"会与正确产品行为矛盾。现用**自己服务实例的
        真实消息发送层桩**暂存 step_done（不改生产、不删原帧），证明
        "末样本到达且权威收尾被暂存"期间不得解锁；随后释放原帧走真实 TCP，
        证明权威完成能解锁（恢复判据保留）。
        """
        cal_id = self._begin_osl(query_capabilities=False, repeats=2, ifbw=100)
        self.assertIsNone(
            self.wizard._air_supports_step_done,
            "前置：本会话不得有任何能力查询",
        )
        original_send = self.service._send_calibration_message
        held_frames: list[tuple[str, str, bytes, dict]] = []
        hold_state = {"holding": True}

        def holding_send(message_type: str, hold_cal_id: str,
                         payload: bytes = b"", **fields: object) -> None:
            if (hold_state["holding"] and message_type == "calibration_result"
                    and fields.get("status") == "step_done"):
                # 暂存权威收尾帧（自己的测试实例桩；稍后按原载荷经真实发送层放行）
                held_frames.append((message_type, hold_cal_id, payload, dict(fields)))
                return
            original_send(message_type, hold_cal_id, payload, **fields)

        self.service._send_calibration_message = holding_send  # type: ignore[method-assign]
        try:
            self.wizard.capture_next_step()
            self.assertTrue(self.wizard._capturing)
            # 等待末样本（repeats=2 → 第 2 条 calibration_capture）真实到达
            captures_baseline = self.collector.count_cal(
                type="calibration_capture", cal_id=cal_id
            )
            self.assertTrue(
                wait_until(
                    lambda: self.collector.count_cal(
                        type="calibration_capture", cal_id=cal_id
                    )
                    >= captures_baseline + 2,
                    60,
                    app=self.app,
                ),
                "末样本未到达",
            )
            # 暂存窗口：给任何"过早解锁"留出真实发生的时间（末样本已到、
            # 权威 step_done 被暂存未放行）
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                self.app.processEvents()
                time.sleep(0.05)
            self.assertTrue(
                held_frames,
                "前置：权威 step_done 应已被暂存（未收到说明采集未真实完成）",
            )
            # F8 契约：权威收尾被暂存期间严禁解锁（修复前现状=末样本即完成，红）
            self.assertTrue(
                self.wizard._capturing,
                "F8 契约：step_done 暂存期间不得解锁在途请求（实测被解锁）",
            )
            self.assertNotIn(
                "P1_Open",
                self.wizard._completed_steps,
                "F8 契约：step_done 暂存期间不得计入真实完成集合",
            )
            self.assertNotEqual(
                self._row_text("P1_Open"),
                "待采集",
                "F8 契约：暂存期间在途行不得被回退为待采集",
            )
        finally:
            hold_state["holding"] = False
            self.service._send_calibration_message = original_send  # type: ignore[method-assign]
        # 释放原帧：经真实发送层（enqueue → 真实 TCP → 地面处理）权威收尾
        for held_type, held_cal_id, held_payload, held_fields in held_frames:
            original_send(held_type, held_cal_id, held_payload, **held_fields)
        self.assertTrue(
            wait_until(
                lambda: self._row_text("P1_Open") == "已完成"
                and not self.wizard._capturing
                and "P1_Open" in self.wizard._completed_steps,
                60,
                app=self.app,
            ),
            "step_done 权威收尾未到达",
        )
        self.assertEqual(self.wizard.current_step_name(), "P1_Short")

    def test_old_air_end_without_step_done_completes_and_reports_unsupported(self) -> None:
        # 旧空中端仿真：拦截真实消息层（保持真实 TCP/采集管线/回执路径）
        original_send = self.service._send_calibration_message
        suppressed: list[dict] = []

        def old_send(message_type: str, cal_id: str, payload: bytes = b"", **fields: object) -> None:
            if message_type == "calibration_result" and fields.get("status") == "step_done":
                suppressed.append(dict(fields))
                return  # 旧端不发 step_done
            original_send(message_type, cal_id, payload, **fields)

        self.service._send_calibration_message = old_send  # type: ignore[method-assign]

        original_handle = self.service.handle_command

        def old_handle(action: str, params: object) -> dict:
            if action == "cal_import_file":
                return {"supported": False}
            result = original_handle(action, params)
            if action == "cal_query_active" and isinstance(result, dict) and result.get("ok"):
                caps = result.get("capabilities")
                if isinstance(caps, dict):
                    caps["step_done"] = False
            return result

        self.service.sender.command_callback = old_handle  # type: ignore[assignment]

        cal_id = self._begin_osl(query_capabilities=False)
        # 旧端能力查询：step_done=False 被地面真实接受
        self.wizard.query_active_calibration()
        self.assertTrue(
            wait_until(lambda: self.wizard._air_supports_step_done is False, 15, app=self.app),
            "旧端能力必须让地面切换为降级模式",
        )

        # 采集：末样本（index>=of）到达即完成，绝不苦等 step_done 挂起
        self.wizard.capture_next_step()
        self.assertTrue(
            wait_until(
                lambda: self._row_text("P1_Open") == "已完成" and not self.wizard._capturing,
                60,
                app=self.app,
            ),
            f"旧端降级未完成（行={self._row_text('P1_Open')}）",
        )
        # 空中端驱动的 step_done 被仿真层截获（证明新端会发、旧端不发也不挂起）
        self.assertTrue(
            wait_until(lambda: len(suppressed) >= 1, 15),
            "空中端应尝试发送 step_done（被旧端仿真层截获）",
        )
        self.assertEqual(len(suppressed), 1)
        # 末样本到达即完成且无"请求采集中"残留（旧端平滑降级契约）
        self.assertFalse(self.wizard._capturing)
        self.assertNotIn("请求", self._row_text("P1_Open"))

        # 第二步同样平滑完成
        self.wizard.capture_next_step()
        self.assertTrue(
            wait_until(lambda: self._row_text("P1_Short") == "已完成", 60, app=self.app)
        )

        # 旧端不支持的 cal_import_file：明确"版本较低"提示，绝不假装成功
        self.assertTrue(self.wizard.import_calibration_file(str(REAL_V2_FILE)))
        self.assertTrue(
            wait_until(
                lambda: "版本较低" in self.wizard.status_label.text(), 15, app=self.app
            ),
            f"旧端不支持导入必须可见（实际: {self.wizard.status_label.text()}）",
        )
        self.assertIsNone(self.service._session_calibration, "旧端拒绝后不得伪造成效")


class TestR8_ContractBoundaries(_E2ECase):
    """captain 风险 3/5/6 独立实测（黑匣子命令入口 + 校验器边界）。"""

    def test_import_config_save_failure_preserves_old_file_bytes_and_memory(self) -> None:
        """F3 正式回归（t10）：同名导入在配置写盘失败时必须整笔失败且
        旧同名生效文件的字节、配置引用、内存校准三者全部不变。

        修复前现状 = 文件先 os.replace 后 save_config，失败仅回滚配置字符串，
        旧同名文件内容被新内容永久覆盖（本用例红，修复后转绿）。
        """
        content_a = json.loads(REAL_V2_FILE.read_text(encoding="utf-8"))
        self.assertTrue(
            self.service.handle_command(
                "cal_import_file",
                {"filename": "case.json", "content": content_a, "activate": True,
                 "request_id": "imp_a"},
            )["ok"]
        )
        target = self.service.output_root / "_calibration" / "imports" / "case.json"
        self.assertTrue(target.exists())
        bytes_a = target.read_bytes()
        active_before = self.service.active_calibration_id
        cal_before = self.service._session_calibration

        self.service.save_config = lambda: False  # 受控注入：模拟磁盘写失败
        content_b = json.loads(json.dumps(content_a))
        content_b["created_at"] = "2026-10-13T19:00:00"
        result = self.service.handle_command(
            "cal_import_file",
            {"filename": "case.json", "content": content_b, "activate": True,
             "request_id": "imp_b"},
        )
        self.assertFalse(result["ok"], "配置写盘失败必须整笔失败（补遗修订7）")
        # F3 契约①：旧同名生效文件字节不变（修复前现状=被覆盖，红）
        self.assertEqual(
            target.read_bytes(),
            bytes_a,
            "F3 契约：配置写盘失败时旧同名生效文件字节必须保持不变"
            "（实测被新内容覆盖，文件域未回滚）",
        )
        # F3 契约②：内存现有效校准不变
        self.assertEqual(
            self.service._session_calibration,
            cal_before,
            "内存现有效校准不得被失败导入破坏",
        )
        self.assertEqual(self.service.active_calibration_id, active_before)
        # F3 契约③：失败导入的新内容不得残留在 imports 目录成为半套文件
        self.assertEqual(
            json.loads(target.read_bytes().decode("utf-8")).get("created_at"),
            content_a.get("created_at"),
            "imports 目录中的生效文件必须仍是旧内容（无半套覆盖）",
        )

    def test_air_import_filename_cannot_escape_imports_dir(self) -> None:
        """F3b 正式回归（t10）：空中端 cal_import_file 的 filename 必须按
        basename 净化，`..` 相对路径不得把文件写出 imports 目录。

        修复前现状 = 未净化，探针文件成功逃逸（本用例红，修复后转绿）。
        全部探针位于用例自身临时目录内。
        """
        content = {
            "format": "uav_gpr_solt_calibration_v2",
            "measurements": [
                {"type": "P1_Open", "data": {"points": [
                    {"frequency": 1e9, "real": 0.9, "imag": 0.0},
                    {"frequency": 2e9, "real": 0.8, "imag": 0.1}]}},
                {"type": "P1_Short", "data": {"points": [
                    {"frequency": 1e9, "real": -0.9, "imag": 0.0},
                    {"frequency": 2e9, "real": -0.8, "imag": -0.1}]}},
                {"type": "P1_Load", "data": {"points": [
                    {"frequency": 1e9, "real": 0.0, "imag": 0.0},
                    {"frequency": 2e9, "real": 0.01, "imag": 0.0}]}},
            ],
        }
        result = self.service.handle_command(
            "cal_import_file",
            {"filename": os.path.join("..", "escaped_probe", "escaped_probe.json"),
             "content": content, "activate": False, "request_id": "imp_escape"},
        )
        imports_dir = self.service.output_root / "_calibration" / "imports"
        # 探针：imports 目录之外、用例临时树之内的任何 json 都是逃逸证据
        escaped = [
            p for p in self.tmp.rglob("escaped_probe*.json")
            if imports_dir not in p.parents
        ]
        if result.get("ok"):
            self.assertFalse(
                escaped,
                "F3 契约：filename 含 `..` 时必须被净化（落在 imports 目录内）"
                f"或明确拒绝，实测逃逸写出 imports 目录（探针={escaped}）",
            )
        else:
            self.assertFalse(
                escaped,
                "拒绝路径下不得有逃逸文件残留",
            )

    def test_import_failing_config_path_reports_failure_and_keeps_memory(self) -> None:
        # config_path 指向目录 → save_config 真实 OSError（无 monkeypatch）
        dir_cfg = self.tmp / "air_config_as_dir.json"
        dir_cfg.mkdir(exist_ok=True)
        content = json.loads(REAL_V2_FILE.read_text(encoding="utf-8"))
        service2 = self._make_service(
            dir_cfg,
            backend_factory=lambda: PersistentLibreVnaBackend(
                transport=ControlledStandardTransport(pace=False, noise_level=0.0)
            ),
            start_sender=False,
            output_dir=self.tmp / "runs2",
        )
        result = service2.handle_command(
            "cal_import_file",
            {"filename": "never_active.json", "content": content, "activate": True},
        )
        self.assertFalse(result["ok"], "写盘失败必须返回失败（真实 OSError 路径）")
        self.assertIn("配置持久化保存失败", str(result.get("error")))
        self.assertIsNone(service2._session_calibration, "内存不得生效失败导入")
        q = service2.handle_command("cal_query_active", {})
        self.assertFalse(
            (q.get("active_osl") or {}).get("active"),
            f"查询必须如实报告无生效 OSL（实测 q={q}）",
        )

    def test_activate_persistence_failure_must_not_broadcast_activated(self) -> None:
        """F4 正式回归（t10）：cal_activate 在持久化写盘失败时必须拒绝或
        原子回滚，绝不广播假 `activated`（否则地面显示已生效、空中重启即丢失）。

        修复前现状 = save_config 失败被忽略、activated 照常广播
        （本用例红，修复后转绿）。持久化失败用 config_path 指向目录的
        真实 OSError 注入（自己的服务实例，无 monkeypatch）。
        """
        dir_cfg = self.tmp / "air_config_as_dir2.json"
        dir_cfg.mkdir(exist_ok=True)
        broadcasts: list[tuple[str, str, dict]] = []
        service2 = self._make_service(
            dir_cfg,
            backend_factory=lambda: PersistentLibreVnaBackend(
                transport=ControlledStandardTransport(pace=False, noise_level=0.0)
            ),
            start_sender=False,
            output_dir=self.tmp / "runs3",
        )
        original_send = service2._send_calibration_message

        def recording_send(message_type: str, cal_id: str, payload: bytes = b"", **fields: object) -> None:
            broadcasts.append((message_type, cal_id, dict(fields)))
            original_send(message_type, cal_id, payload, **fields)

        service2._send_calibration_message = recording_send  # type: ignore[method-assign]

        cal_id = "cal_f4_probe"
        self.assertTrue(
            service2.handle_command("cal_begin", {"cal_id": cal_id, "mode": "osl", "repeats": 1})["ok"]
        )
        for step in cal.OSL_STEP_NAMES:
            self.assertTrue(
                service2.handle_command(
                    "cal_capture", {"cal_id": cal_id, "step": step, "request_id": f"f4_{step}"}
                )["ok"]
            )
            # 命令级 ok 只代表受理；必须等真实采集线程收尾再发下一步
            self.assertTrue(
                wait_until(
                    lambda: service2._calibration_thread is None
                    or not service2._calibration_thread.is_alive(),
                    30,
                ),
                f"步骤 {step} 采集线程未按时收尾",
            )
        self._wait_service_idle(service2)
        self.assertTrue(service2.handle_command("cal_finish", {"cal_id": cal_id})["ok"])
        activate_result = service2.handle_command("cal_activate", {"cal_id": cal_id})
        self.assertFalse(
            dir_cfg.is_file(),
            "持久化写盘失败：air_config.json 不得被伪造成功写入（目录路径真实 OSError）",
        )
        # F4 契约：持久化失败时（a）命令不返回 ok=true 的"已激活"，或
        # （b）不广播 status=activated（或回执显式 persisted=false）。
        activated_broadcasts = [
            b for b in broadcasts
            if b[0] == "calibration_result" and b[2].get("status") == "activated"
        ]
        persisted_claimed = bool(activate_result.get("persisted"))
        if persisted_claimed:
            self.fail("回执声明 persisted=true 但 air_config.json 未写入（矛盾）")
        self.assertFalse(
            activate_result.get("ok") and not persisted_claimed and activated_broadcasts,
            "F4 契约：持久化失败时不得 ok=true 且广播假 activated "
            f"（实测 ack={activate_result}，broadcasts={activated_broadcasts}）",
        )
        _ = activated_broadcasts

    def _wait_service_idle(self, service: "svc.AirBlackboxService") -> None:
        self.assertTrue(
            wait_until(
                lambda: service._calibration_thread is None
                or not service._calibration_thread.is_alive(),
                30,
            ),
            "采集线程未在时限内收尾",
        )

    def test_validator_enforces_10mb_for_dict_and_string_alike(self) -> None:
        """F5 正式回归（t10）：validate_calibration_content 对 dict 与 str
        输入必须执行同一 10MB 上限（补遗修订7）。

        修复前现状 = str 受限、dict 绕过（本用例红，修复后转绿）。
        """
        base_payload = {
            "format": "uav_gpr_solt_calibration_v2",
            "measurements": [
                {"type": "P1_Open", "data": {"points": [
                    {"frequency": 1e9, "real": 0.9, "imag": 0.0},
                    {"frequency": 2e9, "real": 0.8, "imag": 0.1}]}},
                {"type": "P1_Short", "data": {"points": [
                    {"frequency": 1e9, "real": -0.9, "imag": 0.0},
                    {"frequency": 2e9, "real": -0.8, "imag": -0.1}]}},
                {"type": "P1_Load", "data": {"points": [
                    {"frequency": 1e9, "real": 0.0, "imag": 0.0},
                    {"frequency": 2e9, "real": 0.01, "imag": 0.0}]}},
            ],
        }
        big_dict = dict(base_payload)
        big_dict["_oversize_padding"] = "x" * (11 * 1024 * 1024)
        big_str = json.dumps(big_dict)
        self.assertGreater(len(big_str.encode("utf-8")), 10 * 1024 * 1024)
        ok_str, err_str, _ = cal.validate_calibration_content(big_str)
        self.assertFalse(ok_str, "str 输入必须受 10MB 上限约束")
        self.assertIn("最大限制", err_str)
        ok_dict, err_dict, _ = cal.validate_calibration_content(big_dict)
        self.assertFalse(
            ok_dict,
            f"F5 契约：dict 输入必须与 str 同受 10MB 上限约束"
            f"（实测通过，尺寸门被绕过；err={err_dict}）",
        )

    def test_corrupt_dual_port_channel_data_must_be_rejected(self) -> None:
        """F6 正式回归（t10）：声明双端口（含 P2_* 标准件）的文件中任何
        通道数值非有限（channels.S22=NaN）必须被原子拒绝，不得静默丢弃
        P2 后按单端口生效。

        修复前现状 = 校验器放行、load_components 静默 p2=None、导入 ok
        且 ports=["P1"]（本用例红，修复后转绿）。经 handle_command 全链验证。
        """
        corrupt = {
            "format": "uav_gpr_solt_calibration_v2",
            "measurements": [
                {"type": "P1_Open", "data": {"points": [
                    {"frequency": 1e9, "real": 0.9, "imag": 0.0},
                    {"frequency": 2e9, "real": 0.9, "imag": 0.0}]}},
                {"type": "P1_Short", "data": {"points": [
                    {"frequency": 1e9, "real": -0.9, "imag": 0.0},
                    {"frequency": 2e9, "real": -0.9, "imag": 0.0}]}},
                {"type": "P1_Load", "data": {"points": [
                    {"frequency": 1e9, "real": 0.0, "imag": 0.0},
                    {"frequency": 2e9, "real": 0.0, "imag": 0.0}]}},
                {"type": "P2_Open", "data": {"points": [
                    {"frequency": 1e9, "real": 0.9, "imag": 0.0,
                     "channels": {"S22": {"real": float("nan"), "imag": 0.0}}},
                    {"frequency": 2e9, "real": 0.9, "imag": 0.0,
                     "channels": {"S22": {"real": float("nan"), "imag": 0.0}}}]}},
                {"type": "P2_Short", "data": {"points": [
                    {"frequency": 1e9, "real": -0.9, "imag": 0.0},
                    {"frequency": 2e9, "real": -0.9, "imag": 0.0}]}},
                {"type": "P2_Load", "data": {"points": [
                    {"frequency": 1e9, "real": 0.0, "imag": 0.0},
                    {"frequency": 2e9, "real": 0.0, "imag": 0.0}]}},
            ],
        }
        result = self.service.handle_command(
            "cal_import_file",
            {"filename": "corrupt_dual_port.json", "content": corrupt,
             "activate": True, "request_id": "imp_f6"},
        )
        self.assertFalse(
            result.get("ok"),
            f"F6 契约：声明双端口的文件含非有限通道值必须整笔拒绝"
            f"（实测 ok={result.get('ok')}，静默降级单端口）",
        )
        self.assertIsNone(
            self.service._session_calibration,
            "被拒绝的导入不得在内存生效",
        )

    def _make_small_valid_p1(self) -> dict:
        return {
            "format": "uav_gpr_solt_calibration_v2",
            "measurements": [
                {"type": "P1_Open", "data": {"points": [
                    {"frequency": 1e9, "real": 0.9, "imag": 0.0},
                    {"frequency": 2e9, "real": 0.8, "imag": 0.1}]}},
                {"type": "P1_Short", "data": {"points": [
                    {"frequency": 1e9, "real": -0.9, "imag": 0.0},
                    {"frequency": 2e9, "real": -0.8, "imag": -0.1}]}},
                {"type": "P1_Load", "data": {"points": [
                    {"frequency": 1e9, "real": 0.0, "imag": 0.0},
                    {"frequency": 2e9, "real": 0.01, "imag": 0.0}]}},
            ],
        }

    def test_close_current_failure_must_not_fake_step_done(self) -> None:
        """close_current 异常注入（自己的实例桩）：步骤收尾失败时空中端
        不得照常广播权威 step_done（"槽已释放可继续"为假）。

        修复前现状 = driver finally 吞掉 close_current 异常后仍广播
        step_done（本用例红，修复后转绿：应回 failed/收尾失败，或等真实
        idle 后再发 step_done）。经真实命令入口 + 真实采集线程 + 模拟设备。
        """
        cal_id = "cal_close_stub"
        self.assertTrue(
            self.service.handle_command(
                "cal_begin", {"cal_id": cal_id, "mode": "osl", "repeats": 1}
            )["ok"]
        )
        proxy = self.service._calibration_session.capturer

        def _failing_close(step_name: str) -> None:
            raise RuntimeError("stub: close_current 注入失败（t10）")

        proxy.close_current = _failing_close  # type: ignore[method-assign]
        step_done_baseline = self.collector.count_cal(
            type="calibration_result", status="step_done", cal_id=cal_id
        )
        self.assertTrue(
            self.service.handle_command(
                "cal_capture",
                {"cal_id": cal_id, "step": "P1_Open", "request_id": "close_stub_1"},
            )["ok"]
        )
        self.assertTrue(
            wait_until(
                lambda: self.service._calibration_thread is None
                or not self.service._calibration_thread.is_alive(),
                60,
            ),
            "采集线程未收尾",
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(0.05)
        step_done_count = self.collector.count_cal(
            type="calibration_result", status="step_done", cal_id=cal_id
        )
        failed_count = self.collector.count_cal(
            type="calibration_result", status="failed", cal_id=cal_id
        )
        self.assertEqual(
            step_done_count,
            step_done_baseline,
            "close_current 注入失败时严禁照常广播权威 step_done"
            f"（实测 step_done={step_done_count}，failed={failed_count}）",
        )

    def test_import_ack_and_query_report_consistent_complete_fields(self) -> None:
        """F10 正式回归（t10）：cal_import_file 的 command_ack 必须携带补遗
        §修订7 的完整字段（cal_id/filename/format/ports/points/valid_band_hz），
        且与随后的 cal_query_active.active_osl 一致，地面才不依赖二次查询。

        修复前现状 = ack 仅 ok/filename/activated/cal_id/request_id
        （本用例红，修复后转绿）。经真实向导按钮 + 真实 TCP。
        """
        self.assertTrue(self.wizard.import_calibration_file(str(REAL_V2_FILE)))
        self.assertTrue(
            wait_until(
                lambda: self.service._session_calibration is not None, 30
            ),
            "空中端未真实落盘并生效导入文件",
        )
        self.assertTrue(
            wait_until(
                lambda: self.collector.has_cmd(action="cal_import_file",
                                               result={"ok": True}),
                15,
                app=self.app,
            ),
            "地面未收到导入 command_ack",
        )
        ack = [
            h for h in self.collector.command_results
            if h.get("action") == "cal_import_file" and (h.get("result") or {}).get("ok")
        ][-1]
        ack_result = ack["result"]
        for field in ("cal_id", "filename", "format", "ports", "points", "valid_band_hz"):
            self.assertIn(
                field,
                ack_result,
                f"F10 契约：import ack 缺少契约字段 {field}（实测 keys={sorted(ack_result)}）",
            )
        self.assertEqual(ack_result.get("format"), "uav_gpr_solt_calibration_v2")
        self.assertEqual(ack_result.get("ports"), ["P1", "P2"])
        self.assertEqual(ack_result.get("points"), 201)
        self.assertEqual(ack_result.get("valid_band_hz"), [400000000.0, 4000000000.0])
        # ack 与查询一致（不靠二次查询补显示）
        self.wizard.query_active_calibration()
        self.assertTrue(
            wait_until(
                lambda: (self.wizard._active_calibration or {}).get("ports") == ["P1", "P2"],
                15,
                app=self.app,
            ),
            "查询未同步",
        )
        self.assertEqual(
            (self.wizard._active_calibration or {}).get("cal_id"),
            ack_result.get("cal_id"),
            "F10 契约：import ack 与 cal_query_active 的 cal_id 必须一致",
        )

    def test_clear_background_only_must_not_clear_osl_display(self) -> None:
        """F10 正式回归（t10）：cal_clear_active(target=background) 的 ack
        必须携带 cleared 字段；地面只清背景显示，严禁把 OSL 一并伪清
        （修复前现状 = ack 无 cleared、地面默认 all 清掉 OSL 显示）。
        """
        # 真实建立 OSL（导入）+ 背景（真实会话）双生效
        self.assertTrue(self.wizard.import_calibration_file(str(REAL_V2_FILE)))
        self.assertTrue(
            wait_until(lambda: self.service._session_calibration is not None, 30)
        )
        bg_cal_id = self._begin_background()
        self._capture_and_wait_done("Background")
        self._finish_and_solve(bg_cal_id)
        self._activate(bg_cal_id)
        self.assertIsNotNone(self.wizard._active_background)
        self.assertIsNotNone(self.wizard._active_calibration)
        # 真实按钮入口：仅清背景
        self.wizard.clear_active_calibration("background")
        self.assertTrue(
            wait_until(
                lambda: self.collector.has_cmd(action="cal_clear_active",
                                               result={"ok": True}),
                15,
                app=self.app,
            ),
            "clear ack 未到达",
        )
        clear_ack = [
            h for h in self.collector.command_results
            if h.get("action") == "cal_clear_active" and (h.get("result") or {}).get("ok")
        ][-1]
        self.assertIn(
            "cleared",
            clear_ack["result"],
            f"F10 契约：clear ack 必须显式携带 cleared 字段"
            f"（实测 keys={sorted(clear_ack['result'])}）",
        )
        self.assertEqual(clear_ack["result"].get("cleared"), "background")
        # 空中端真实状态：背景清、OSL 保持
        q = self.service.handle_command("cal_query_active", {})
        self.assertTrue(q["active_osl"]["active"])
        self.assertFalse(q["active_background"]["active"])
        # F10 契约：地面显示必须与空中一致——OSL 不得被伪清
        self.assertIsNotNone(
            self.wizard._active_calibration,
            "F10 契约：仅清背景时地面 OSL 生效显示不得被伪清（实测被清空）",
        )
        self.assertIsNone(self.wizard._active_background)

    def test_export_double_click_writes_each_target_correctly(self) -> None:
        """F10 正式回归（t10）：并发/连点导出必须按各自请求写各自目标文件
        （ack 按请求关联），不得把 A 回执写到 B 的目标路径。

        修复前现状 = 两次连点共用 _pending_export_path，A 回执写到 B 的
        路径、A 的目标文件从未落盘（本用例红，修复后转绿）。
        """
        self.assertTrue(self.wizard.import_calibration_file(str(REAL_V2_FILE)))
        self.assertTrue(
            wait_until(lambda: self.service._session_calibration is not None, 30)
        )
        path_a = self.tmp / "export_a.json"
        path_b = self.tmp / "export_b.json"
        self.assertTrue(self.wizard.export_calibration_file(str(path_a)))
        self.assertTrue(self.wizard.export_calibration_file(str(path_b)))
        self.assertTrue(
            wait_until(
                lambda: self.collector.count_cmd(action="cal_export_file") >= 2,
                20,
                app=self.app,
            ),
            "两次导出 ack 未全部到达",
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.app.processEvents()
            time.sleep(0.05)
        self.assertTrue(
            path_a.exists(),
            "F10 契约：第一次导出的目标文件必须真实落盘"
            "（实测被第二次请求的目标路径抢占）",
        )
        self.assertTrue(path_b.exists(), "第二次导出的目标文件必须真实落盘")
        repo_json = json.loads(REAL_V2_FILE.read_text(encoding="utf-8"))
        self.assertEqual(json.loads(path_a.read_text(encoding="utf-8")), repo_json)
        self.assertEqual(json.loads(path_b.read_text(encoding="utf-8")), repo_json)

    def test_activated_broadcast_carries_contract_fields(self) -> None:
        """F10 正式回归（t10）：calibration_result(status=activated) 必须携带
        补遗 §3.1 的 mode/ports/points/valid_band_hz/active_osl_id 契约字段。

        修复前现状 = 仅 profile_summary/calibration_id（本用例红，修复后转绿）。
        """
        dir_cfg = self.tmp / "air_config_f10.json"
        service2 = self._make_service(
            dir_cfg,
            backend_factory=lambda: PersistentLibreVnaBackend(
                transport=ControlledStandardTransport(pace=False, noise_level=0.0)
            ),
            start_sender=False,
            output_dir=self.tmp / "runs_f10",
        )
        broadcasts: list[tuple[str, str, dict]] = []
        original_send = service2._send_calibration_message

        def recording_send(message_type: str, cal_id: str, payload: bytes = b"", **fields: object) -> None:
            broadcasts.append((message_type, cal_id, dict(fields)))
            original_send(message_type, cal_id, payload, **fields)

        service2._send_calibration_message = recording_send  # type: ignore[method-assign]
        cal_id = "cal_f10_probe"
        self.assertTrue(
            service2.handle_command("cal_begin", {"cal_id": cal_id, "mode": "osl", "repeats": 1})["ok"]
        )
        for step in cal.OSL_STEP_NAMES:
            self.assertTrue(
                service2.handle_command(
                    "cal_capture", {"cal_id": cal_id, "step": step, "request_id": f"f10_{step}"}
                )["ok"]
            )
            self.assertTrue(
                wait_until(
                    lambda: service2._calibration_thread is None
                    or not service2._calibration_thread.is_alive(),
                    30,
                ),
                f"步骤 {step} 采集线程未按时收尾",
            )
        self.assertTrue(service2.handle_command("cal_finish", {"cal_id": cal_id})["ok"])
        self.assertTrue(service2.handle_command("cal_activate", {"cal_id": cal_id})["ok"])
        activated = [
            b for b in broadcasts
            if b[0] == "calibration_result" and b[1] == cal_id
            and b[2].get("status") == "activated"
        ]
        self.assertTrue(activated, "activated 广播缺失")
        fields = activated[-1][2]
        for field in ("mode", "ports", "points", "valid_band_hz", "active_osl_id"):
            self.assertIn(
                field,
                fields,
                f"F10 契约：activated 广播缺少契约字段 {field}（实测 keys={sorted(fields)}）",
            )
        self.assertEqual(fields.get("mode"), "osl")
        self.assertEqual(fields.get("active_osl_id"), cal_id)


if __name__ == "__main__":
    unittest.main()
