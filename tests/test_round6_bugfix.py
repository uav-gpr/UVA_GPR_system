"""Round 6 bugfix regression tests（真机 2026-09-18 三 bug 修复轮）.

t1 锁定：空中端 ``MainWindow._handle_set_params_command`` 的校准会话互斥
守卫曾引用全项目不存在的三个 ``CAL_*STATE_*`` 前缀状态名（planned/
in_progress/solved），导致 ``_calibration_session`` 非 None 时任何 set_params
远程命令在构建拒绝元组处必然 NameError（用户每次在地面端点"采集设置"必现；
时窗等参数永远发不到空中端——真机 bug2 时窗部分的根因）。第 6 轮修复为
``CalibrationSession.STATE_*``（与 ``_calibration_acquisition_conflict``
同款正确写法，三态口径与 R-C-14 session_open 一致）。

两方向回归锁定：
1. 拒绝方向：校准会话存在且 state ∈ {planned, in_progress, solved} →
   set_params 返回 False、参数控件/配置不被修改、日志含拒绝原因，且全程
   不抛 NameError（覆盖 ``_handle_set_params_command`` 直调、
   ``handle_remote_command`` 真实分派链路、真实 CalibrationSession 对象）。
2. 应用方向：无会话 / 会话处于终态（cancelled/failed/activated）→
   set_params 正常应用，含 time_window_ns 写入 time_window 控件与
   config()（bug2 时窗部分复活的直接证据）。

t3 锁定：地面端 B-scan 长宽比真实窗口可见几何。取证结论（offscreen show
实测，_t3_forensics/probe_aspect_geometry.txt）：apply 路径与
AspectRatioWidget.heightForWidth 均正常，但父链 QSplitter 不传播
heightForWidth，QVBoxLayout 对两个等宽 stretch=1 容器的均分把比例差完全
抵消（h = w/r + (H−slider−2w/r)/2 ≡ 常数）→ 改长宽比零视觉效果。修复 =
AspectRatioWidget 按当前宽度给自己加 maximumHeight=width/ratio 硬上限
（minimumSizeHint 托底），保留 stretch=1 填充：可见高度 =
min(均分份额, width/ratio)——默认 2.0 大窗口外观不变、比例可容纳时精确
生效、视图放不下时物理回落均分。另按 captain 批准方案一在设置中心
"处理参数"页顶部加 B-scan 显示设置直达入口（UX 入口困惑层）。

t5 锁定（真机反馈：空中端开机自动连接 127.0.0.1 且 IP 无法更改）：
1. 默认"地面站 IP"为空（不再预填 127.0.0.1），占位文字提示格式；
   开机 1.2s 自动连接定时器遇空 IP 跳过（不创建 sender）。
2. Connect 按钮为连接/断开切换：连接后 IP/端口输入框禁用、按钮变
   "断开 Disconnect"；点击断开 → sender.stop(drain=False)（未发送数据
   保留磁盘 outbox，重连自动补传）→ sender 置 None、输入框解锁、
   按钮回 "Connect" → 可改 IP 后重连到新地址（RemoteSender 以新 host
   构造）。旧实现无任何断开路径，IP 一旦创建发送器即被钉死到重启。
"""

from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

from PySide6 import QtWidgets

import uav_gpr_ground_station as ground
import uav_gpr_qt as air


class SetParamsCalibrationGuardTests(unittest.TestCase):
    """t1：set_params 校准会话守卫 NameError 回归（两方向）。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance()
        if cls.app is None:
            cls.app = QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.window = air.MainWindow()

    def tearDown(self) -> None:
        self.window._calibration_session = None
        self.window.close()
        self.app.processEvents()

    @staticmethod
    def _fake_session(state: str) -> MagicMock:
        session = MagicMock()
        session.state = state
        return session

    @staticmethod
    def _real_planned_session() -> air.CalibrationSession:
        plan = air.build_osl_plan(
            cal_id="CAL-BUGFIX6-T1", repeats=1, config_snapshot={}
        )
        return air.CalibrationSession(plan, MagicMock())

    # ---- 方向①：校准会话进行中 → 拒绝，且绝不 NameError ----

    def test_open_session_rejects_set_params_without_nameerror(self) -> None:
        """三种进行中状态逐一拒绝：返回 False、配置不变、日志含拒绝原因。"""
        params = {"start_mhz": 800.0, "stop_mhz": 2000.0, "time_window_ns": 30.0}
        for state in (
            air.CalibrationSession.STATE_PLANNED,
            air.CalibrationSession.STATE_IN_PROGRESS,
            air.CalibrationSession.STATE_SOLVED,
        ):
            with self.subTest(state=state):
                self.window.device_log.clear()
                self.window._calibration_session = self._fake_session(state)
                initial_cfg = self.window.config()

                returned = self.window._handle_set_params_command(dict(params))

                self.assertFalse(returned)
                self.assertEqual(self.window.config(), initial_cfg)
                self.assertIn(
                    "set_params 拒绝: 校准会话进行中不可修改配置",
                    self.window.device_log.toPlainText(),
                )

    def test_open_session_rejects_real_session_object(self) -> None:
        """真实 CalibrationSession（默认 planned）走同一拒绝路径——真机崩溃原样探针。"""
        self.window.device_log.clear()
        session = self._real_planned_session()
        self.assertEqual(session.state, air.CalibrationSession.STATE_PLANNED)
        self.window._calibration_session = session
        initial_cfg = self.window.config()

        returned = self.window._handle_set_params_command(
            {"start_mhz": 800.0, "stop_mhz": 2000.0}
        )

        self.assertFalse(returned)
        self.assertEqual(self.window.config(), initial_cfg)
        self.assertIn(
            "set_params 拒绝: 校准会话进行中不可修改配置",
            self.window.device_log.toPlainText(),
        )

    def test_remote_dispatch_with_open_session_does_not_raise(self) -> None:
        """真实分派链路 handle_remote_command("set_params") 全程不抛异常。"""
        self.window._calibration_session = self._fake_session(
            air.CalibrationSession.STATE_IN_PROGRESS
        )
        initial_cfg = self.window.config()

        self.window.handle_remote_command(
            "set_params", {"start_mhz": 900.0, "stop_mhz": 2500.0}
        )

        self.assertEqual(self.window.config(), initial_cfg)
        self.assertIn(
            "set_params 拒绝: 校准会话进行中不可修改配置",
            self.window.device_log.toPlainText(),
        )

    # ---- 方向②：无会话 / 终态会话 → 参数正常应用（时窗复活取证）----

    def test_set_params_applies_without_session(self) -> None:
        """无会话：start/stop/points/time_window 写入控件与 config()。"""
        self.assertIsNone(self.window._calibration_session)

        returned = self.window._handle_set_params_command(
            {
                "start_mhz": 600.0,
                "stop_mhz": 2400.0,
                "points": 101,
                "time_window_ns": 30.0,
            }
        )

        self.assertTrue(returned)
        self.assertEqual(self.window.start_mhz.value(), 600.0)
        self.assertEqual(self.window.stop_mhz.value(), 2400.0)
        self.assertEqual(self.window.points.value(), 101)
        self.assertEqual(self.window.time_window.value(), 30.0)
        cfg = self.window.config()
        self.assertEqual(cfg.start_mhz, 600.0)
        self.assertEqual(cfg.stop_mhz, 2400.0)
        self.assertEqual(cfg.points, 101)
        self.assertEqual(cfg.time_window_ns, 30.0)

    def test_set_params_applies_with_terminal_state_session(self) -> None:
        """终态会话（cancelled/failed/activated）不阻塞下发，口径与
        _calibration_acquisition_conflict 的 session_open 一致。"""
        for state, tw in (
            (air.CalibrationSession.STATE_CANCELLED, 31.0),
            (air.CalibrationSession.STATE_FAILED, 32.0),
            (air.CalibrationSession.STATE_ACTIVATED, 33.0),
        ):
            with self.subTest(state=state):
                self.window._calibration_session = self._fake_session(state)

                returned = self.window._handle_set_params_command(
                    {"time_window_ns": tw}
                )

                self.assertTrue(returned)
                self.assertEqual(self.window.time_window.value(), tw)
                self.assertEqual(self.window.config().time_window_ns, tw)

    def test_set_params_applies_with_cancelled_real_session(self) -> None:
        """真实会话 cancel() 后（终态 cancelled）参数恢复正常下发。"""
        session = self._real_planned_session()
        self.assertEqual(session.cancel("测试取消"), air.CalibrationSession.STATE_CANCELLED)
        self.window._calibration_session = session

        returned = self.window._handle_set_params_command({"time_window_ns": 45.0})

        self.assertTrue(returned)
        self.assertEqual(self.window.time_window.value(), 45.0)
        self.assertEqual(self.window.config().time_window_ns, 45.0)


class AspectRatioRealWindowTests(unittest.TestCase):
    """t3：B-scan 长宽比实窗取证与修复锁定（真实 GroundStationWindow）。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance()
        if cls.app is None:
            cls.app = QtWidgets.QApplication([])

    def setUp(self) -> None:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.window = ground.GroundStationWindow(
            host="127.0.0.1",
            port=0,
            out_dir=Path(tmp.name),
            receiver=MagicMock(),
            start_receiver=False,
        )
        self.window.resize(1380, 860)
        self.window.show()
        self._pump()

    def tearDown(self) -> None:
        self.window.close()
        self.app.processEvents()

    def _pump(self) -> None:
        self.app.processEvents()
        self.app.sendPostedEvents()
        self.app.processEvents()

    def _container_heights(self) -> dict[str, int]:
        return {
            name: container.height()
            for name, container in self.window.bscan_containers.items()
        }

    def _apply_dialog_ratio(self, value: float) -> None:
        """完整用户路径：打开 B-scan 设置对话框 → 改值 → 应用（OK）。"""
        self.window.open_bscan_settings()
        dialog = self.window.bscan_dialog
        dialog.aspect.setValue(value)
        dialog.apply()
        self._pump()

    # ---- 锁①：默认 ratio=2.0 大窗口外观与修复前一致（填充不回归）----

    def test_default_ratio_keeps_fill_layout(self) -> None:
        self.assertEqual(self.window.bscan_settings.aspect_ratio, 2.0)
        left = self.window.bscan_containers["S11"].parentWidget()
        self.assertGreater(left.height(), 100)
        slider_row = self.window.findChild(QtWidgets.QWidget, "bscan_history_row")
        self.assertIsNotNone(slider_row)
        for name in ("S11", "S22"):
            container = self.window.bscan_containers[name]
            # 填充行为仍在：两容器高度相同（均分）且非 letterbox 留白。
            self.assertGreater(container.height(), 100)
        hh = self.window.bscan_containers["S11"]
        vv = self.window.bscan_containers["S22"]
        self.assertAlmostEqual(hh.height(), vv.height(), delta=1)
        # VV 底边贴住历史滑条行顶部（布局间距 8）——填充覆盖到滑条为止。
        self.assertLessEqual(vv.geometry().bottom(), slider_row.geometry().top())
        self.assertGreaterEqual(
            vv.geometry().bottom(), slider_row.geometry().top() - 9
        )

    # ---- 锁②：比例可容纳时可见几何精确变化（修复核心）----

    def test_dialog_ratio_change_produces_real_visible_geometry_change(self) -> None:
        base = self._container_heights()
        self._apply_dialog_ratio(4.0)
        self.assertEqual(self.window.bscan_settings.aspect_ratio, 4.0)
        changed = self._container_heights()
        for name in ("S11", "S22"):
            container = self.window.bscan_containers[name]
            expected = round(container.width() / 4.0)
            # 可见高度精确 = width/ratio（不是只看 heightForWidth 数值）。
            self.assertAlmostEqual(changed[name], expected, delta=1)
            # 相对默认 2.0 的真实可见变化。
            self.assertLess(changed[name], base[name])
            # B-scan 视图本体同步变化。
            self.assertAlmostEqual(
                self.window.bscan_plots[name].height(), changed[name], delta=1
            )

    def test_ratio_restore_returns_to_baseline(self) -> None:
        base = self._container_heights()
        self._apply_dialog_ratio(4.0)
        self._apply_dialog_ratio(2.0)
        self.assertEqual(self._container_heights(), base)

    # ---- 锁③：视图放不下时物理回落均分（不溢出、不出界）----

    def test_too_tall_ratio_falls_back_to_fill_without_overflow(self) -> None:
        base = self._container_heights()
        self._apply_dialog_ratio(1.0)  # width/1 ≫ 可用高度 → 无法精确呈现
        self.assertEqual(self.window.bscan_settings.aspect_ratio, 1.0)
        left = self.window.bscan_containers["S11"].parentWidget()
        for name in ("S11", "S22"):
            container = self.window.bscan_containers[name]
            # 回落填充：高度与默认一致（均分），且绝不溢出左栏。
            self.assertAlmostEqual(container.height(), base[name], delta=1)
            self.assertLessEqual(container.geometry().bottom(), left.height())

    # ---- 托底分支：maximumHeight 永不低于 minimumSizeHint ----

    def test_height_cap_floor_respects_minimum_size_hint(self) -> None:
        # 多行 QLabel 抬高 minimumSizeHint，使托底分支真正生效
        # （round(200/6)=33 < minimumSizeHint）。
        child = QtWidgets.QLabel("B-scan\n" * 6)
        widget = ground.AspectRatioWidget(child, 2.0)
        widget.resize(200, 100)
        widget.show()  # 隐藏控件不派发 resizeEvent，show 后才有实时事件
        self.app.processEvents()
        try:
            # 正常 regime：cap = round(width/ratio)。
            widget.set_aspect_ratio(2.0)
            self.assertAlmostEqual(widget.maximumHeight(), round(200 / 2.0), delta=1)
            # 极端比例：width/ratio 低于 minimumSizeHint → 以托底为准。
            widget.set_aspect_ratio(6.0)
            floor_h = widget.minimumSizeHint().height()
            self.assertGreater(floor_h, round(200 / 6.0))
            self.assertEqual(widget.maximumHeight(), floor_h)
            # resizeEvent 跟随宽度：加宽后 ratio 分支重新生效。
            widget.resize(600, widget.maximumHeight())
            self.app.processEvents()
            self.assertAlmostEqual(widget.maximumHeight(), round(600 / 6.0), delta=1)
            self.assertGreaterEqual(widget.maximumHeight(), floor_h)
        finally:
            widget.close()
            self.app.processEvents()

    # ---- UX（captain 批准方案一）：设置中心直达 B-scan 显示设置 ----

    def test_settings_center_bscan_entry_button(self) -> None:
        dialog = self.window.acquisition_dialog
        button = dialog.bscan_entry_button
        self.assertEqual(button.objectName(), "bscan_entry_button")
        self.assertFalse(button.objectName().endswith("_input"))  # 非参数控件
        hint = dialog.bscan_entry_hint
        self.assertIn("B-scan", hint.text())
        # 页签结构不受影响（3 页签契约保持）。
        self.assertEqual(dialog.tab_widget.count(), 3)
        # 点击直达既有 B-scan 设置对话框。
        self.assertFalse(self.window.bscan_dialog.isVisible())
        button.click()
        self._pump()
        self.assertTrue(self.window.bscan_dialog.isVisible())
        # 对话框加载的是已提交值。
        self.assertAlmostEqual(
            self.window.bscan_dialog.aspect.value(),
            self.window.bscan_settings.aspect_ratio,
        )


class RemoteConnectToggleTests(unittest.TestCase):
    """t5：开机不自动连 127.0.0.1 + Connect 按钮连接/断开切换。

    真机反馈：空中端默认预填 127.0.0.1 且开机自动连接，连接后 IP 被钉死
    （旧实现无任何断开路径，Connect 点击只会提示 already running）。
    修复 = 默认空 IP（自动连接定时器遇空跳过）+ Connect 按钮切换连接/
    断开（断开 drain=False，未发送数据留磁盘 outbox 重连补传）+ 连接中
    锁定 IP/端口输入框。
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance()
        if cls.app is None:
            cls.app = QtWidgets.QApplication([])

    def setUp(self) -> None:
        self.window = air.MainWindow()

    def tearDown(self) -> None:
        self.window.close()
        self.app.processEvents()

    def test_default_host_empty_not_127(self) -> None:
        self.assertEqual(self.window.remote_host.text(), "")
        self.assertNotEqual(self.window.remote_host.text(), "127.0.0.1")

    def test_startup_connect_with_empty_host_creates_no_sender(self) -> None:
        # 开机定时器（1.2s）触发的就是 connect_remote_sender：空 IP 必须跳过。
        self.window.connect_remote_sender()
        self.assertIsNone(self.window.remote_sender)

    def test_connect_toggle_locks_fields_and_disconnect_releases(self) -> None:
        constructed_hosts: list[str] = []

        class FakeSender:
            def __init__(self, host, port, run_id, status_cb, command_cb, outbox_dir=None):
                constructed_hosts.append(host)
                self.host = host
                self.start = MagicMock()
                self.stop = MagicMock(return_value=True)

        with unittest.mock.patch.object(air, "RemoteSender", FakeSender):
            # 1. 填入第一版地址并点击连接按钮：IP/端口锁定、按钮变"断开"
            self.window.remote_host.setText("192.168.144.99")
            self.window.remote_connect.click()
            self.assertIsNotNone(self.window.remote_sender)
            first_sender = self.window.remote_sender
            self.assertFalse(self.window.remote_host.isEnabled(), "连接中必须锁定 IP 输入")
            self.assertFalse(self.window.remote_port.isEnabled())
            self.assertIn("断开", self.window.remote_connect.text())
            self.assertEqual(constructed_hosts, ["192.168.144.99"])

            # 2. 内部再次调用 connect_remote_sender（模拟 start_acquisition 时的调用）：
            # 必须保持连接，绝对不可误断开！
            self.window.connect_remote_sender()
            self.assertIs(self.window.remote_sender, first_sender, "内部再次调用 connect_remote_sender 绝不可断开已有连接")
            first_sender.stop.assert_not_called()

            # 3. 再次点击 UI 按钮 = 主动断开：stop(drain=False) 被调、sender 置空、输入解锁
            self.window.remote_connect.click()
            self.assertIsNone(self.window.remote_sender)
            first_sender.stop.assert_called_with(timeout_ms=2000, drain=False)
            self.assertTrue(self.window.remote_host.isEnabled(), "断开后必须可编辑 IP")
            self.assertEqual(self.window.remote_connect.text(), "Connect")

            # 4. 改成新地址重连：新地址真实生效（旧实现被钉死无法做到）
            self.window.remote_host.setText("10.0.0.5")
            self.window.remote_connect.click()
            self.assertEqual(constructed_hosts[-1], "10.0.0.5", "断开后必须能连到新 IP")


if __name__ == "__main__":
    unittest.main()
