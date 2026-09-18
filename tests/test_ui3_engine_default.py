"""第4轮：采集引擎“默认走高速连续”契约测试（团队任务卡 t1 验收基准）。

真机实测约 2.75 s/帧 的根因（captain 已逐行核实，本模块把每条钉成可执行断言）：

1. 地面站 ``command_params`` 只发 count/interval_s/single，**不含引擎选择**——
   故引擎必须由空中端默认值保证（本模块 ``test_ground_command_params_*``）；
2. 空中端 ``start_acquisition`` 依 ``continuous_enabled`` 勾选框二选一；
3. 该勾选框默认值取环境变量 ``UAV_GPR_ACQUISITION_MODE``，未设即 legacy；
4. 第3轮界面精简删掉操作栏后 ``addWidget(self.continuous_enabled)`` 命中 0 ——
   勾选框失去布局宿主，界面上已无法打开它；
5. legacy 每帧调 ``acquire_sparameters``（``librevna_usb_s11.py:362 open_device`` …
   ``:393-398`` 释放关闭），故 2.75 s/帧且控制台每帧打印一次 ``Connected``。

验收核心（**非文字描述**）：不设任何环境变量的缺省 ``MainWindow``，点“开始”后进
``_start_continuous_acquisition`` / ``AcquisitionService`` 路径，而 legacy 的
``acquire_sparameters`` **一次都未被调用**（``test_default_start_never_calls_acquire_sparameters``）。
"""

from __future__ import annotations

import inspect
import os
import tempfile
import unittest
import unittest.mock
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
from tests.test_round2_air_ui_compact import _FlowTransport, _wait_until_qt


class _FakeContinuousWorker(QtCore.QObject):
    """ContinuousAcquisitionWorker 替身：覆盖 _factory 会连接的全部信号。"""

    finished_run = QtCore.Signal()
    status = QtCore.Signal(str)
    failed = QtCore.Signal(str)
    trace_ready = QtCore.Signal(object)
    paused = QtCore.Signal()
    resumed = QtCore.Signal()
    statistics_updated = QtCore.Signal(dict)

    def __init__(self, **kwargs: object) -> None:
        super().__init__()
        self.kwargs = kwargs
        self.started = False

    def isRunning(self) -> bool:  # noqa: N802 - 对齐 QThread 语义
        return self.started

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def wait(self, _ms: int = 5000) -> bool:
        self.started = False
        return True


class _FakeLegacyWorker(QtCore.QObject):
    """AcquisitionWorker 替身（legacy 分支只连接 trace_ready/status/failed/finished_run）。"""

    finished_run = QtCore.Signal()
    status = QtCore.Signal(str)
    failed = QtCore.Signal(str)
    trace_ready = QtCore.Signal(object)

    def __init__(self, **_kwargs: object) -> None:
        super().__init__()
        self.started = False

    def isRunning(self) -> bool:  # noqa: N802
        return self.started

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def wait(self, _ms: int = 5000) -> bool:
        self.started = False
        return True


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class EngineDefaultContractTests(unittest.TestCase):
    """① 默认锁高速连续引擎：零操作、零环境变量的缺省窗口就必须走高速路径。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        os.environ.pop(air.ACQUISITION_MODE_ENV, None)
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

    def test_env_default_is_continuous(self) -> None:
        """③ 环境变量未设时默认值不再是 legacy——这正是真机走慢引擎的第一因。"""
        self.assertEqual(air.DEFAULT_ACQUISITION_MODE, "continuous")
        self.assertEqual(air.acquisition_mode(), "continuous")
        self.assertTrue(air.is_continuous_enabled())

    def test_window_checkbox_default_checked(self) -> None:
        """默认窗口的引擎勾选框必须已勾选（用户不必做任何操作就走高速引擎）。"""
        self.assertTrue(
            self.window.continuous_enabled.isChecked(),
            "缺省窗口未勾选连续采集 → 真机将继续走 2.75 s/帧 的 legacy 引擎",
        )

    def test_default_start_routes_to_continuous_worker(self) -> None:
        """② 默认（零操作）点开始采集必须走 ContinuousAcquisitionWorker 分支。"""
        with unittest.mock.patch.object(air, "ContinuousAcquisitionWorker", _FakeContinuousWorker):
            with unittest.mock.patch.object(air, "AcquisitionWorker", _FakeLegacyWorker):
                self.window.start_acquisition(single=False)
        self.assertIsInstance(self.window.worker, _FakeContinuousWorker)
        self.assertTrue(self.window.worker.started)

    def test_default_start_never_calls_acquire_sparameters(self) -> None:
        """验收核心（真实采集，非替身）：默认路径跑完，legacy 的 acquire_sparameters 0 次。

        用真实 ``MainWindow`` + 注入 ``_FlowTransport``（协议线假 LibreVNA）跑默认
        （连续）路径的完整采集，同时 patch ``acquire_sparameters`` 计数：
        - 计数必须为 0 → 默认没有走 legacy 逐帧开关 USB 的老路；
        - ``AcquisitionService`` 必须真的进入 acquiring（服务 open/configure 一次）；
        - trace 必须真的到 B-scan（不是"假装启动"）。
        """
        tmp = tempfile.TemporaryDirectory()
        try:
            self.window.output_dir.setText(tmp.name)
            self.window.points.setValue(5)
            self.window.start_mhz.setValue(400.0)
            self.window.stop_mhz.setValue(440.0)
            self.window.count.setValue(3)
            transport = _FlowTransport(5)
            service = air.AcquisitionService(
                backend_factory=lambda: PersistentLibreVnaBackend(transport=transport)
            )
            self.window.acquisition_service = service

            with unittest.mock.patch.object(
                air, "acquire_sparameters", side_effect=AssertionError("默认路径不得走 legacy")
            ) as probe:
                self.window.start_acquisition(single=False)
                self.assertIsNotNone(self.window.worker)
                self.assertEqual(
                    probe.call_count, 0,
                    "默认引擎不得调用 legacy 的 acquire_sparameters（每帧开关 USB 的老路）",
                )
                self.assertIn(
                    service.state,
                    ("acquiring", "idle_waiting"),
                    f"默认路径必须经 AcquisitionService 常驻服务（当前 {service.state}）",
                )
                self.assertTrue(
                    _wait_until_qt(
                        lambda: self.window.bscan_trace_count() >= 3, 15.0, self.app
                    ),
                    f"默认路径未产出 trace（当前 {self.window.bscan_trace_count()} 道）",
                )
                self.assertEqual(
                    probe.call_count, 0,
                    "整个默认采集过程都不得调用 acquire_sparameters",
                )
            self.assertTrue(transport.open_count >= 1, "连续路径应通过持久后端打开 USB")
        finally:
            tmp.cleanup()

    def test_legacy_route_only_when_explicitly_unticked(self) -> None:
        """legacy 退路保留：显式取消勾选才走旧引擎（不再由默认值决定）。"""
        self.window.continuous_enabled.setChecked(False)
        self.assertFalse(self.window.continuous_enabled.isChecked())
        self.assertEqual(self.window.engine_mode_combo.currentData(), "legacy")

    def test_legacy_engine_source_kept_as_fallback(self) -> None:
        """legacy 引擎代码不得删除（调试退路），只是不再做默认。"""
        self.assertTrue(hasattr(air, "AcquisitionWorker"))
        self.assertEqual(air.ENGINE_MODE_LEGACY, "legacy")

    def test_start_log_names_the_engine_actually_used(self) -> None:
        """每次启动采集都写明本轮引擎——真机自测不必猜是哪条路径。"""
        self.window.continuous_enabled.setChecked(False)
        with unittest.mock.patch.object(air, "AcquisitionWorker", _FakeLegacyWorker):
            self.window.start_acquisition(single=False)
        legacy_log = self.window.device_log.toPlainText()
        self.assertIn("本任务引擎: legacy", legacy_log)
        self.assertIn("2.75", legacy_log)
        self.window.worker = None  # 假 worker 不参与后续清理

        self.window.continuous_enabled.setChecked(True)
        with unittest.mock.patch.object(air, "ContinuousAcquisitionWorker", _FakeContinuousWorker):
            self.window.start_acquisition(single=False)
        self.assertIn("本任务引擎: continuous", self.window.device_log.toPlainText())

    def test_ground_command_params_has_no_engine_field(self) -> None:
        """① 地面站 command_params 仍只有 count/interval_s/single：引擎由空中端默认保证。

        这是"为什么不改地面端"的证据：三个既有键语义零变化，引擎选择不靠协议。
        """
        from uav_gpr_ground_station import AcquisitionUiSettings

        params = AcquisitionUiSettings(count=5, interval_s=0.25).command_params()
        self.assertEqual(set(params), {"count", "interval_s", "single"})
        self.assertEqual(params["count"], 5)
        self.assertEqual(params["interval_s"], 0.25)
        self.assertFalse(params["single"])
        # 控制器动作字典也不得新增引擎选择字段。
        source = Path("uav_gpr_ground_station.py").read_text(encoding="utf-8")
        self.assertNotIn('"engine"', source)
        self.assertNotIn("'engine'", source)


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class EngineVisibilityContractTests(unittest.TestCase):
    """④ 引擎开关可见化：勾选框必须有布局宿主，另给下拉入口与只读回显。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self) -> None:
        os.environ.pop(air.ACQUISITION_MODE_ENV, None)
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

    def test_checkbox_has_layout_host_in_central_widget(self) -> None:
        """④ 勾选框必须挂在中央部件的可见容器里（第3轮精简后它曾无宿主）。"""
        central = self.window.centralWidget()
        self.assertIsNotNone(central)
        descendants = set(central.findChildren(QtWidgets.QWidget))
        self.assertIn(
            self.window.continuous_enabled, descendants,
            "continuous_enabled 必须有布局宿主，否则界面上无法打开高速引擎",
        )
        self.assertTrue(self.window.continuous_enabled.isVisibleTo(self.window))
        # 真正的宿主是布局：它在某个 layout 的 item 列表里（不是孤儿控件）。
        hosted = False
        for group in descendants:
            layout = group.layout()
            if layout is None:
                continue
            if any(
                layout.itemAt(i).widget() is self.window.continuous_enabled
                for i in range(layout.count())
            ):
                hosted = True
                break
        self.assertTrue(hosted, "勾选框必须在某个可见 layout 中被 addWidget")

    def test_engine_widgets_hosted_in_connection_bar(self) -> None:
        """引擎行落在连接栏第 3 行；不复活 operator_bar、不新增本地启停按钮。"""
        bar = self.window.findChild(QtWidgets.QWidget, "connection_bar")
        self.assertIsNotNone(bar)
        descendants = set(bar.findChildren(QtWidgets.QWidget))
        self.assertIn(self.window.engine_mode_combo, descendants)
        self.assertIn(self.window.engine_mode_label, descendants)
        self.assertIn(self.window.continuous_enabled, descendants)
        # 第3轮契约不回退：operator_bar 不复活，主界面仍无本地任务按钮。
        self.assertIsNone(self.window.findChild(QtWidgets.QWidget, "operator_bar"))
        self.assertIsNone(self.window.findChild(QtWidgets.QWidget, "diagnostics_group"))
        central_buttons = set(self.window.centralWidget().findChildren(QtWidgets.QPushButton))
        for attr in ("start_btn", "stop_btn", "single_btn", "pause_btn", "resume_btn"):
            self.assertNotIn(getattr(self.window, attr), central_buttons)

    def test_engine_row_text_says_high_speed_continuous(self) -> None:
        """文案必须直白含“高速连续采集”（非专业用户一眼看懂默认是什么）。"""
        texts = [
            self.window.engine_mode_combo.itemText(0),
            self.window.engine_mode_combo.itemText(1),
            self.window.engine_mode_label.text(),
            self.window.continuous_enabled.text(),
        ]
        self.assertTrue(
            any("高速连续采集" in text for text in texts),
            f"引擎控件文案缺少“高速连续采集”: {texts}",
        )

    def test_engine_row_is_readable_for_operator(self) -> None:
        """只读回显写清“当前引擎 + 快/慢 + 是否默认”，用户不必猜。"""
        text = self.window.engine_mode_label.text()
        self.assertIn("连续", text)
        self.assertIn("默认", text)
        self.assertIn("高速", text)
        self.window.continuous_enabled.setChecked(False)
        slow = self.window.engine_mode_label.text()
        self.assertIn("legacy", slow)
        self.assertIn("慢", slow)

    def test_combo_and_checkbox_stay_in_sync(self) -> None:
        """下拉 ↔ 勾选框同一语义两个入口，绝不各说各话。"""
        # 勾选框 → 下拉
        self.window.continuous_enabled.setChecked(False)
        self.assertEqual(self.window.engine_mode_combo.currentData(), "legacy")
        self.window.continuous_enabled.setChecked(True)
        self.assertEqual(self.window.engine_mode_combo.currentData(), "continuous")
        # 下拉 → 勾选框
        legacy_index = self.window.engine_mode_combo.findData("legacy")
        self.assertGreaterEqual(legacy_index, 0)
        self.window.engine_mode_combo.setCurrentIndex(legacy_index)
        self.assertFalse(self.window.continuous_enabled.isChecked())
        continuous_index = self.window.engine_mode_combo.findData("continuous")
        self.window.engine_mode_combo.setCurrentIndex(continuous_index)
        self.assertTrue(self.window.continuous_enabled.isChecked())

    def test_engine_choice_locked_during_task(self) -> None:
        """任务运行中不得改引擎（与既有机上锁语义一致）。"""
        self.window._set_task_controls_locked(True)
        try:
            self.assertFalse(self.window.continuous_enabled.isEnabled())
        finally:
            self.window._set_task_controls_locked(False)
        self.assertTrue(self.window.continuous_enabled.isEnabled())

    def test_legacy_mode_surfaces_slow_engine_hint(self) -> None:
        """选了慢引擎必须有关键提示（状态区文案写明 legacy 很慢）——状态不撒谎。"""
        self.window.continuous_enabled.setChecked(False)
        self.window.on_worker_finished()
        self.assertIn("legacy", self.window.continuous_status_label.text())
        self.assertIn("慢", self.window.continuous_status_label.text())
        # legacy 态该行必须写明"本轮/当前"用 legacy（而不是含糊的"未启用"）。
        self.assertIn("本轮 legacy", self.window.continuous_status_label.text())
        # 切回高速后，状态区不再宣称 legacy。
        self.window.continuous_enabled.setChecked(True)
        self.window.on_worker_finished()
        self.assertIn("高速连续采集", self.window.continuous_status_label.text())
        self.assertNotIn("legacy", self.window.continuous_status_label.text())

    def test_continuous_status_label_has_no_disabled_ambiguity(self) -> None:
        """t3 修复：continuous 态两处文案都不得出现「未启用」歧义，且与真实状态一致。

        缺陷本体：窗口起来时写「连续模式: 未启用」、任务收尾后写
        「连续模式: 未启用（下次启动按高速连续采集）」，而引擎行同时写着
        「当前引擎：连续 · 高速 · 默认」——同一界面自相矛盾，操作员会以为默认
        引擎没生效（正是本团队要修的那类"状态撒谎"）。

        锁定三件事：
        1. 缺省（零环境变量）窗口起来就是「高速连续采集（默认，未运行）」——
           与 `DEFAULT_ACQUISITION_MODE == "continuous"` 一致；
        2. 一轮任务收尾后是「高速连续采集（待命，下次启动即用）」；
        3. 两处都**不含**「未启用」；只有显式 legacy 态才允许"未启用"字样。
        """
        # ① 窗口刚起来（缺省引擎 = continuous）。
        self.assertTrue(
            air.is_continuous_enabled(),
            "本测试前提：缺省引擎必须是 continuous（否则断言的是 legacy 契约）",
        )
        startup_text = self.window.continuous_status_label.text()
        self.assertEqual(startup_text, air.CONTINUOUS_STATUS_READY)
        self.assertIn("高速连续采集", startup_text)
        self.assertIn("默认", startup_text)
        self.assertNotIn("未启用", startup_text, "continuous 态不得出现「未启用」歧义")

        # ② 一轮任务收尾后（USB 待命，下次启动即用同一高速路径）。
        self.window.on_worker_finished()
        standby_text = self.window.continuous_status_label.text()
        self.assertEqual(standby_text, air.CONTINUOUS_STATUS_STANDBY)
        self.assertIn("高速连续采集", standby_text)
        self.assertIn("待命", standby_text)
        self.assertNotIn("未启用", standby_text, "continuous 态不得出现「未启用」歧义")

        # ③ 只有显式 legacy 态才允许"未启用"，且必须同时写明慢的原因。
        #    legacy 态有两条入口（启动时 env=legacy 的初始化文案 / 运行中取消勾选后的
        #    收尾文案），两条都必须自洽：都写明"慢"，且不出现 continuous 字样冒充高速。
        self.window.continuous_enabled.setChecked(False)
        self.window.on_worker_finished()
        legacy_text = self.window.continuous_status_label.text()
        self.assertEqual(legacy_text, air.CONTINUOUS_STATUS_LEGACY_RUNTIME)
        self.assertIn("未启用", legacy_text)
        self.assertIn("legacy", legacy_text)
        self.assertIn("慢", legacy_text)
        # 初始化文案（env=legacy 那条）单独锁一次：两态四条文案都不得含糊。
        self.assertIn("未启用", air.CONTINUOUS_STATUS_LEGACY)
        self.assertIn("legacy", air.CONTINUOUS_STATUS_LEGACY)
        self.assertIn("慢", air.CONTINUOUS_STATUS_LEGACY)
        for text in (
            air.CONTINUOUS_STATUS_READY,
            air.CONTINUOUS_STATUS_STANDBY,
            air.CONTINUOUS_STATUS_LEGACY,
            air.CONTINUOUS_STATUS_LEGACY_RUNTIME,
        ):
            self.assertNotIn(
                "本轮已完成", text,
                f"不得用含糊的“本轮已完成”冒充主状态：{text}",
            )


@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class LegacyEngineCostEvidenceTests(unittest.TestCase):
    """⑤ 源码级证据：legacy 每帧开关一次 USB，因此不能做默认引擎。"""

    def test_legacy_worker_calls_acquire_sparameters_per_frame(self) -> None:
        source = inspect.getsource(air.AcquisitionWorker)
        self.assertIn(
            "acquire_sparameters", source,
            "legacy worker 的逐帧 acquire_sparameters 是其慢的根因（每帧 open/close USB）",
        )

    def test_continuous_engine_uses_persistent_service(self) -> None:
        """continuous 走常驻服务 + 持久后端（USB 只开一次），故为默认。"""
        source = inspect.getsource(air.MainWindow._start_continuous_acquisition)
        self.assertIn("start_task", source)
        self.assertIn("AcquisitionService", inspect.getsource(air.MainWindow.__init__))


if __name__ == "__main__":
    unittest.main()
