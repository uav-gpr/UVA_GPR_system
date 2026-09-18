"""地面端 UI 精简——第1轮验收测试（紧凑主界面、中文菜单、独立设置窗口）。

覆盖（roadmap 之外的 UI 精简需求）：
1. 主窗口菜单均存在且为中文；
2. 主界面只有“开始”和“停止”两个任务快捷按钮；
3. “单次采集”位于菜单并发送 single 命令；
4. HH (S11) 在上、VV (S22) 在下；
5. GNSS 地图始终在主界面；
6. B-scan 与 GNSS 使用可拖动 QSplitter；
7. B-scan 历史滑块常驻主界面；
8. 状态栏只有连接状态 / Trace / 采集状态三个业务标签；
9. 运行日志默认隐藏；
10. 日志隐藏时仍能追加，重新打开内容仍在；
11. 详细状态默认隐藏并可打开；
12. 采集设置“取消”不生效，“应用”后 command_params() 更新；
13. B-scan 设置“取消”不生效；
14. B-scan 设置“应用”后刷新长宽比和显示；
15. 清空 B-scan 菜单正确调用 clear_bscan()；
16. 开始/停止按钮发送原有 start/stop 动作；
17. S-parameter 和 A-scan 不在可见布局中；
18. GNSS、B-scan 和 ReceiverThread 原有测试无回归。

第1轮修正新增覆盖：
19. 已提交设置（AcquisitionUiSettings/BscanUiSettings）为 frozen dataclass，原地修改
    抛 FrozenInstanceError，且抛错后 command_params / 长宽比 / 颜色表 / Flat Filter
    状态不变化；应用新实例仍正常生效；
20. 采集状态 ACK 状态机：start/stop/single 的 “Remote accepted:” 正确更新状态，
    发送前/发送后待确认/ACK 后三阶段均断言，失败状态不被未知成功文本覆盖。

第2轮“彻底移除 S-parameter/A-scan”新增覆盖：
21. 地面端生产代码不再存在 compat_container / sparam_plot / sparam_curves /
    ascan_plot / ascan_curves 属性；
22. GroundStationWindow 只创建两个 pyqtgraph PlotWidget（即 HH/VV 两幅 B-scan）；
23. 主界面布局、GNSS、B-scan、滑块、状态栏、菜单仍满足第1轮验收，无回归。

第2轮审查修复新增覆盖：
24. on_trace 双通道分支必须检查 _append_bscan_trace() 返回值；时间轴或 Dewow
    signature 不兼容的 Trace 不得更新 GNSS / metadata / 当前 Trace / Dewow / legacy；
    仅 B-scan 拒绝原因、最近事件与运行日志允许更新；成功 Trace 仍正常追加并更新。

第3轮视觉收口与窗口适配新增覆盖：
25. 主窗口尺寸策略：最小 960×600、目标 1380×860、上限 1920×1080；初始尺寸
    不超过当前屏幕可用区域（1366×768、1920×1080 模拟均覆盖）；
26. “开始/停止”按钮不横向无限拉伸（maximumWidth 限定）；
27. show() 后主界面关键控件（开始/停止、HH/VV B-scan、GNSS、历史滑块、
    状态栏三项）几何尺寸均非零；
28. 1366×768 与 1920×1080 模拟下主窗口结构相同、HH/VV 顺序不变、PlotWidget
    仍恰好两个；
29. B-scan 0 在上、最多 64 道/屏；aspect_ratio 设置仍生效；
30. 第2轮审查修复的 3 条 on_trace 测试继续通过。

第3轮审查修复新增覆盖（独立审查发现的窗口尺寸策略缺陷 + 布局测试缺口）：
31. 屏幕可用区域小于最小尺寸 960×600（800×600 模拟）时，初始窗口尺寸
    必须 ≤ 可用区域，不得溢出；
32. 800×600 屏幕上 setMinimumSize 同步收紧到 ``min(design_min, initial)``
    （即 760×570），避免 Qt 把窗口重新撑大到 960×600 再次溢出；
33. 1024×600 屏幕（宽度大于 960 但高度刚好 600）下初始尺寸仍为 972×570，
    完整装入屏幕；
34. 设计契约 ``min ≥ 960×600`` 在 1280×800 模拟屏幕下保持成立（契约验证
    必须在已知足够大的屏幕下进行，不受 offscreen 默认 800×800 平台限制）。

第3轮第二次审查修复新增覆盖（宽高耦合 + 真实布局几何 + 异常屏幕）：
35. 宽高完全独立计算：七种分辨率（800×600 / 1024×600 / 1280×800 /
    1366×768 / 1920×1080 / 1920×600 / 800×1080）逐一断言精确初始尺寸，
    其中 1920×600 → (1380,570)、800×1080 → (760,860) 专门抓宽高耦合
    回归（旧实现会返回 (1824,570) / (760,1026)）；
36. 异常屏幕信息：QApplication.instance() 为 None、primaryScreen() 为
    None、availableGeometry() 抛异常、可用宽/高为 0 时全部退化到
    1280×800；极小正数可用区域下初始尺寸始终为正；
37. 动态 minimumSize 按维独立收紧（1920×600 → (960,570)、800×1080 →
    (760,600)）；
38. show() + processEvents 后的真实布局几何（1366×768 / 1920×1080）：
    关键控件对主窗口可见、非零、完整位于窗口矩形内；HH/VV/滑块/B-scan
    区/GNSS 互不重叠；splitter 实际比例 B-scan 60–80%、GNSS 20–40%；
    GNSS 有效宽度 ≥ 150、B-scan 区 ≥ 300；HH 几何在 VV 上方；
39. 1920×1080 相比 1366×768 窗口尺寸不减、B-scan 区/GNSS 面积增加、
    HH/VV 容器面积不减。

全部使用 DummyReceiver（几何测试加 start_receiver=False），无真实网络监听。
"""

from __future__ import annotations

import contextlib
import os
import tempfile
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path
from unittest.mock import patch

import io
import json

import numpy as np
import pyqtgraph as pg

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

try:
    from PySide6 import QtCore, QtWidgets
    import uav_gpr_ground_station as ground
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover - environment dependent
    QtCore = None
    QtWidgets = None
    ground = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


class DummyReceiver(QtCore.QObject):
    status = QtCore.Signal(str)
    command_status = QtCore.Signal(str)
    trace_received = QtCore.Signal(dict, bytes, str)

    def __init__(self, *_args) -> None:
        super().__init__()
        self.calls: list[tuple[str, dict | None]] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def wait(self, _timeout: int) -> bool:
        return True

    def send_command(self, action: str, params=None) -> None:
        self.calls.append((action, params))

    def begin_run(self, run_id: str) -> None:  # 第3轮修复需要
        pass

    def note_signal_consumed(self) -> None:  # 第3轮修复需要
        self._emitted_underflow_errors = getattr(self, "_emitted_underflow_errors", 0) + 1

    # 字段：第3轮修复需要
    _emitted_underflow_errors: int = 0


@unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
class GroundStationCompactUiTests(unittest.TestCase):
    def setUp(self) -> None:
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.temp = tempfile.TemporaryDirectory()
        with patch.object(ground, "ReceiverThread", DummyReceiver):
            self.window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.temp.name))
        self.central = self.window.centralWidget()

    def tearDown(self) -> None:
        try:
            self.window.close()
            self.window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        except Exception:
            pass
        self.app.processEvents()
        self.temp.cleanup()

    # ---- 主窗口结构与可见性 ----

    def test_chinese_menus_and_actions_exist(self) -> None:
        menubar = self.window.menuBar()
        actions = [action.text() for action in menubar.actions()]
        self.assertEqual(actions, ["任务", "B-scan", "视图", "帮助"])
        for action in (
            self.window.action_single_acquisition,
            self.window.action_acquisition_settings,
            self.window.action_exit,
            self.window.action_bscan_settings,
            self.window.action_clear_bscan,
            self.window.action_show_details,
            self.window.action_show_log,
            self.window.action_about,
        ):
            self.assertTrue(action.text(), "菜单动作文字不能为空")
        # 所有动作都有可定位 objectName。
        self.assertEqual(self.window.action_single_acquisition.objectName(), "action_single_acquisition")
        self.assertEqual(self.window.action_acquisition_settings.objectName(), "action_acquisition_settings")
        self.assertEqual(self.window.action_exit.objectName(), "action_exit")
        self.assertEqual(self.window.action_bscan_settings.objectName(), "action_bscan_settings")
        self.assertEqual(self.window.action_clear_bscan.objectName(), "action_clear_bscan")
        self.assertEqual(self.window.action_show_details.objectName(), "action_show_details")
        self.assertEqual(self.window.action_show_log.objectName(), "action_show_log")
        self.assertEqual(self.window.action_about.objectName(), "action_about")
        # 任务菜单顺序：单次采集 / 采集设置 / 分隔线 / 退出。
        task = self.window.menu_task
        self.assertEqual(task.title(), "任务")
        self.assertEqual(task.actions()[0], self.window.action_single_acquisition)
        self.assertEqual(task.actions()[1], self.window.action_acquisition_settings)
        self.assertTrue(task.actions()[2].isSeparator())
        self.assertEqual(task.actions()[3], self.window.action_exit)

    def test_only_start_and_stop_buttons_on_main_window(self) -> None:
        # 第3轮控制迁移：地面端补齐暂停/继续快捷按钮（任务控制完整迁移地面端，共开始/停止/暂停/继续）
        buttons = self.central.findChildren(QtWidgets.QPushButton)
        self.assertEqual({button.text() for button in buttons}, {"开始", "停止", "暂停", "继续"})
        self.assertEqual(self.window.start_button.text(), "开始")
        self.assertEqual(self.window.stop_button.text(), "停止")
        self.assertEqual(self.window.pause_button.text(), "暂停")
        self.assertEqual(self.window.resume_button.text(), "继续")

    def test_start_and_stop_buttons_send_original_actions(self) -> None:
        self.window.start_button.click()
        self.assertEqual(self.window.receiver.calls[-1][0], "start")
        self.assertEqual(
            self.window.receiver.calls[-1][1],
            self.window.command_params(),
        )
        self.window.stop_button.click()
        self.assertEqual(self.window.receiver.calls[-1][0], "stop")
        self.assertIsNone(self.window.receiver.calls[-1][1])
        # 第3轮控制迁移：暂停与继续按钮发送对应远程命令
        self.window.pause_button.click()
        self.assertEqual(self.window.receiver.calls[-1][0], "pause")
        self.window.resume_button.click()
        self.assertEqual(self.window.receiver.calls[-1][0], "resume")

    def test_single_acquisition_in_menu_sends_single(self) -> None:
        self.window.action_single_acquisition.trigger()
        action, params = self.window.receiver.calls[-1]
        self.assertEqual(action, "single")
        self.assertEqual(params["single"], True)
        self.assertEqual(params["count"], 1)
        self.assertEqual(params["interval_s"], self.window.command_params()["interval_s"])

    def test_clear_bscan_action_calls_clear_bscan(self) -> None:
        self.window.bscan_rows = {
            name: [np.arange(4, dtype=float)] for name in ground.UI_CHANNELS
        }
        self.window.bscan_time_ns = np.arange(4, dtype=float)
        with patch.object(self.window, "clear_bscan", wraps=self.window.clear_bscan) as mocked:
            self.window.action_clear_bscan.trigger()
            mocked.assert_called_once()
        self.assertEqual(self.window.bscan_trace_count(), 0)
        self.assertIsNone(self.window.bscan_time_ns)

    # ---- B-scan / GNSS 布局 ----

    def test_hh_above_vv_and_gnss_always_in_main_window(self) -> None:
        left = self.window.findChild(QtWidgets.QWidget, "bscan_area")
        self.assertIsNotNone(left, "B-scan 区域必须在主界面")
        container_s11 = self.window.bscan_containers[ground.UI_CHANNELS[0]]
        container_s22 = self.window.bscan_containers[ground.UI_CHANNELS[1]]
        self.assertEqual(ground.UI_CHANNELS, ("S11", "S22"))
        left_layout = left.layout()
        index_s11 = left_layout.indexOf(container_s11)
        index_s22 = left_layout.indexOf(container_s22)
        self.assertLess(index_s11, index_s22, "HH (S11) 应在 VV (S22) 上方")
        # GNSS 地图属于主窗口中央布局（不放入 Tab/菜单/隐藏区）。
        self.assertIsNotNone(self.window.gnss_map)
        self.assertIsInstance(self.window.gnss_map.parentWidget(), QtWidgets.QSplitter)
        self.assertTrue(self.window.gnss_map.parentWidget().parentWidget() is self.central or
                        self.window.gnss_map.parentWidget().parentWidget() is self.window.centralWidget())

    def test_bscan_and_gnss_share_a_draggable_splitter(self) -> None:
        splitter = self.window.findChild(QtWidgets.QSplitter, "main_splitter")
        self.assertIsNotNone(splitter)
        self.assertEqual(splitter.orientation(), QtCore.Qt.Orientation.Horizontal)
        self.assertTrue(splitter.handle(0).isEnabled(), "Splitter 必须可拖动")
        left = self.window.findChild(QtWidgets.QWidget, "bscan_area")
        self.assertEqual(splitter.indexOf(left), 0)
        self.assertEqual(splitter.indexOf(self.window.gnss_map), 1)
        # 初始比例约 70:30（分裂柄落在中间偏左）。
        sizes = splitter.sizes()
        self.assertGreater(sizes[0], sizes[1], "B-scan 区应比 GNSS 宽")

    def test_history_slider_is_permanent_on_main_window(self) -> None:
        self.assertIsNotNone(self.window.bscan_window_slider)
        # 滑块必须属于主窗口中央区域（bscan_area 的子树）。
        self.assertIn(
            self.window.bscan_window_slider,
            self.window.findChildren(QtWidgets.QWidget, "bscan_area")[0].findChildren(QtWidgets.QWidget),
        )
        self.assertIsNotNone(self.window.bscan_window_label)
        # 滑块加入后仍能切换窗口并更新计数。
        self.window.bscan_rows = {
            name: [np.full(4, float(index)) for index in range(65)]
            for name in ground.UI_CHANNELS
        }
        self.window.bscan_time_ns = np.arange(4, dtype=float)
        self.window.refresh_bscan_views()
        self.assertEqual(self.window.bscan_window_slider.maximum(), 2)
        self.window.bscan_window_slider.setValue(2)
        self.assertIn("2", self.window.bscan_window_label.text())

    # ---- 状态栏 ----

    def test_status_bar_has_only_three_business_labels(self) -> None:
        status = self.window.statusBar()
        labels = [
            label
            for label in status.findChildren(QtWidgets.QLabel)
            if label.objectName()
            in {"connection_status_label", "trace_status_label", "acquisition_status_label"}
        ]
        self.assertEqual(len(labels), 3)
        self.assertEqual(self.window.connection_status_label.text(), "连接状态：等待连接")
        self.assertEqual(self.window.trace_status_label.text(), "当前 Trace：-")
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：未采集")

    def test_command_status_updates_acquisition_label(self) -> None:
        self.window.send_start()
        self.assertIn("正在发送启动命令", self.window.acquisition_status_label.text())
        self.window.send_stop()
        self.assertIn("正在发送停止命令", self.window.acquisition_status_label.text())
        self.window.send_single()
        self.assertIn("单次采集命令已发送", self.window.acquisition_status_label.text())
        self.window._on_command_status("Command failed: boom")
        self.assertIn("命令发送失败", self.window.acquisition_status_label.text())

    def test_receiver_status_updates_connection_label(self) -> None:
        self.window._on_receiver_status("Connected: 127.0.0.1:9999")
        self.assertEqual(self.window.connection_status_label.text(), "连接状态：已连接")
        self.window._on_receiver_status("Disconnected: socket closed")
        self.assertEqual(self.window.connection_status_label.text(), "连接状态：已断开")

    # ---- 运行日志 / 详细状态窗口 ----

    def test_log_window_hidden_by_default_and_retains_content(self) -> None:
        self.assertFalse(self.window.log_window.isVisible())
        self.window.log("hidden append")
        self.assertIn("hidden append", self.window.log_box.toPlainText())
        self.window.show_log_window()
        self.assertTrue(self.window.log_window.isVisible())
        self.assertIn("hidden append", self.window.log_box.toPlainText())

    def test_details_window_hidden_by_default_and_openable(self) -> None:
        self.assertFalse(self.window.details_window.isVisible())
        self.window.show_details_window()
        self.assertTrue(self.window.details_window.isVisible())

    # ---- 采集设置对话框 ----

    def test_acquisition_settings_defaults(self) -> None:
        dialog = self.window.acquisition_dialog
        dialog.load_from(self.window.acq_settings)
        self.assertEqual(dialog.interval.value(), 1.0)
        self.assertEqual(dialog.count.value(), 0)
        self.assertIn("持续采集", dialog.count_hint.text())
        self.assertEqual(dialog.interval.minimum(), 0.0)
        self.assertEqual(dialog.interval.maximum(), 3600.0)
        self.assertEqual(dialog.count.minimum(), 0)
        self.assertEqual(dialog.count.maximum(), 1000000)

    def test_acquisition_cancel_does_not_change_command_params(self) -> None:
        dialog = self.window.acquisition_dialog
        dialog.load_from(self.window.acq_settings)
        dialog.mode.setCurrentIndex(1)  # 固定时间间隔
        dialog.interval.setValue(5.0)
        dialog.count.setValue(50)
        dialog.reject()
        self.assertEqual(self.window.command_params(), {"count": 0, "interval_s": 0.0, "single": False})
        self.assertEqual(self.window.acq_settings.interval_s, 0.0)
        self.assertEqual(self.window.acq_settings.count, 0)

    def test_acquisition_apply_updates_command_params(self) -> None:
        dialog = self.window.acquisition_dialog
        dialog.load_from(self.window.acq_settings)
        dialog.mode.setCurrentIndex(1)  # 固定时间间隔
        dialog.interval.setValue(5.0)
        dialog.count.setValue(50)
        dialog.apply()
        self.assertEqual(self.window.command_params(), {"count": 50, "interval_s": 5.0, "single": False})
        self.assertEqual(self.window.acq_settings.interval_s, 5.0)
        self.assertEqual(self.window.acq_settings.count, 50)

    # ---- B-scan 设置对话框 ----

    def test_bscan_settings_defaults(self) -> None:
        dialog = self.window.bscan_dialog
        dialog.load_from(self.window.bscan_settings)
        self.assertEqual(dialog.cmap.currentText(), "gray")
        self.assertEqual(dialog.aspect.value(), 2.0)
        self.assertFalse(dialog.flat_enabled.isChecked())
        self.assertFalse(dialog.flat_window.isEnabled())
        self.assertEqual(dialog.flat_window.value(), 31)

    def test_bscan_cancel_does_not_change_settings(self) -> None:
        dialog = self.window.bscan_dialog
        dialog.load_from(self.window.bscan_settings)
        dialog.cmap.setCurrentText("jet")
        dialog.aspect.setValue(3.0)
        dialog.flat_enabled.setChecked(True)
        dialog.reject()
        self.assertEqual(self.window.bscan_settings.colormap, "gray")
        self.assertEqual(self.window.bscan_settings.aspect_ratio, 2.0)
        self.assertFalse(self.window.bscan_settings.flat_filter_enabled)
        self.assertEqual(self.window.bscan_settings.flat_window_traces, 31)

    def test_bscan_apply_updates_aspect_and_refreshes_display(self) -> None:
        dialog = self.window.bscan_dialog
        self.window.bscan_rows = {
            name: [np.full(4, float(index)) for index in range(40)]
            for name in ground.UI_CHANNELS
        }
        self.window.bscan_time_ns = np.arange(4, dtype=float)
        with patch.object(
            ground, "flat_reflection_filter", wraps=ground.flat_reflection_filter
        ) as mocked:
            self.window.refresh_bscan_views()
            self.assertEqual(mocked.call_count, 0)
            dialog.load_from(self.window.bscan_settings)
            dialog.cmap.setCurrentText("jet")
            dialog.aspect.setValue(3.0)
            dialog.flat_enabled.setChecked(True)
            self.assertTrue(dialog.flat_window.isEnabled())
            dialog.apply()
        self.assertEqual(self.window.bscan_settings.colormap, "jet")
        self.assertEqual(self.window.bscan_settings.aspect_ratio, 3.0)
        self.assertTrue(self.window.bscan_settings.flat_filter_enabled)
        self.assertEqual(mocked.call_count, 2, "应用后应刷新两幅 B-scan")
        self.assertAlmostEqual(self.window.bscan_containers["S11"]._aspect_ratio, 3.0)
        self.assertAlmostEqual(self.window.bscan_containers["S22"]._aspect_ratio, 3.0)

    def test_bscan_flat_window_kept_odd_on_apply(self) -> None:
        dialog = self.window.bscan_dialog
        dialog.load_from(self.window.bscan_settings)
        dialog.flat_enabled.setChecked(True)
        dialog.flat_window.setValue(32)
        dialog.apply()
        self.assertEqual(self.window.bscan_settings.flat_window_traces, 31)

    # ---- 已提交设置不可变（frozen dataclass）----

    def test_acq_settings_frozen_rejects_in_place_mutation(self) -> None:
        settings = self.window.acq_settings
        with self.assertRaises(FrozenInstanceError):
            settings.interval_s = 5.0  # type: ignore[misc]
        with self.assertRaises(FrozenInstanceError):
            settings.count = 42  # type: ignore[misc]
        # 抛错后 command_params 与已提交值不变（默认 interval_s=0.0：每个完整Sweep）。
        self.assertEqual(self.window.command_params(), {"count": 0, "interval_s": 0.0, "single": False})
        self.assertEqual(self.window.acq_settings.interval_s, 0.0)
        self.assertEqual(self.window.acq_settings.count, 0)

    def test_bscan_settings_frozen_rejects_in_place_mutation(self) -> None:
        settings = self.window.bscan_settings
        for field, value in (
            ("colormap", "jet"),
            ("aspect_ratio", 3.0),
            ("flat_filter_enabled", True),
            ("flat_window_traces", 41),
        ):
            with self.subTest(field=field):
                with self.assertRaises(FrozenInstanceError):
                    setattr(settings, field, value)
        # 抛错后长宽比 / 颜色表 / Flat Filter 状态不变。
        self.assertEqual(self.window.bscan_settings.colormap, "gray")
        self.assertEqual(self.window.bscan_settings.aspect_ratio, 2.0)
        self.assertFalse(self.window.bscan_settings.flat_filter_enabled)
        self.assertEqual(self.window.bscan_settings.flat_window_traces, 31)
        self.assertAlmostEqual(self.window.bscan_containers["S11"]._aspect_ratio, 2.0)
        self.assertEqual(self.window.flat_filter_window(), 31)

    def test_apply_new_frozen_instances_still_works(self) -> None:
        # 应用全新实例仍正常生效（替换完整快照，而非原地修改）。
        self.window.apply_acquisition_settings(
            ground.AcquisitionUiSettings(interval_s=5.0, count=42)
        )
        self.assertEqual(self.window.command_params(), {"count": 42, "interval_s": 5.0, "single": False})
        self.window.apply_bscan_settings(
            ground.BscanUiSettings(colormap="jet", aspect_ratio=3.0)
        )
        self.assertEqual(self.window.bscan_settings.colormap, "jet")
        self.assertEqual(self.window.bscan_settings.aspect_ratio, 3.0)
        self.assertAlmostEqual(self.window.bscan_containers["S11"]._aspect_ratio, 3.0)
        self.assertAlmostEqual(self.window.bscan_containers["S22"]._aspect_ratio, 3.0)

    # ---- 采集状态 ACK 状态机 ----

    def test_command_ack_start_three_phases(self) -> None:
        # 阶段1：发送前。
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：未采集")
        # 阶段2：发送后待确认。
        self.window.send_start()
        self.assertIn("正在发送启动命令", self.window.acquisition_status_label.text())
        # 阶段3：明确成功 ACK 后。
        self.window._on_command_status("Remote accepted: start")
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：采集中")

    def test_command_ack_stop_three_phases(self) -> None:
        # 阶段1：发送前。
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：未采集")
        # 阶段2：发送后待确认。
        self.window.send_stop()
        self.assertIn("正在发送停止命令", self.window.acquisition_status_label.text())
        # 阶段3：明确成功 ACK 后——command_ack 只表示停止命令已被接受，
        # "已停止"必须等 acquisition_status 报告 stopped(drain_state=drained)
        # 且地面解码/显示队列归零（第1轮复审统一语义）。
        self.window._on_command_status("Remote accepted: stop")
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：正在停止（等待排空）")
        self.window._on_acquisition_status({"state": "stopped", "drain_state": "drained"})
        # 空转后 decoder/FIFO 均空 → 对账完成，最终显示已停止。
        self.window._reconcile_stop_state()
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：已停止（数据已全部排空）")

    def test_command_ack_single_three_phases(self) -> None:
        # 阶段1：发送前。
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：未采集")
        # 阶段2：发送后待确认。
        self.window.send_single()
        self.assertIn("单次采集命令已发送", self.window.acquisition_status_label.text())
        # 阶段3：明确成功 ACK 后。
        self.window._on_command_status("Remote accepted: single")
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：单次采集已接受")

    def test_command_ack_unknown_action_does_not_show_collecting(self) -> None:
        self.window.acquisition_status_label.setText("采集状态：未采集")
        self.window._on_command_status("Remote accepted: unknown_action")
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：未采集")

    def test_command_failure_not_overwritten_by_unknown_success(self) -> None:
        self.window._on_command_status("Command failed: timeout")
        self.assertIn("命令发送失败", self.window.acquisition_status_label.text())
        # 未知动作的成功文本不得覆盖失败状态。
        self.window._on_command_status("Remote accepted: unknown_action")
        self.assertIn("命令发送失败", self.window.acquisition_status_label.text())

    def test_command_status_kept_verbatim_in_log(self) -> None:
        for text in ("Remote accepted: start", "Command failed: boom", "Remote accepted: pause"):
            self.window._on_command_status(text)
            self.assertIn(text, self.window.log_box.toPlainText())

    # ---- 第2轮：S-parameter / A-scan 已彻底移除 ----

    def test_no_compat_or_sparam_ascan_attributes(self) -> None:
        for attr in (
            "compat_container",
            "sparam_plot",
            "sparam_curves",
            "ascan_plot",
            "ascan_curves",
        ):
            self.assertFalse(hasattr(self.window, attr), f"不应存在 {attr}")

    def test_ground_station_only_creates_two_plot_widgets(self) -> None:
        plots = self.window.findChildren(pg.PlotWidget)
        self.assertEqual(len(plots), 2, f"只能有 HH/VV 两幅 B-scan PlotWidget，实际 {len(plots)}")
        self.assertIn(self.window.bscan_plots["S11"], plots)
        self.assertIn(self.window.bscan_plots["S22"], plots)
        self.assertEqual(set(plots), {self.window.bscan_plots["S11"], self.window.bscan_plots["S22"]})

    def test_main_content_and_menus_still_satisfy_round1(self) -> None:
        # 主界面中央只显示 action bar 与 B-scan/GNSS 主内容。
        central = self.window.centralWidget()
        self.assertIsNotNone(central)
        buttons = central.findChildren(QtWidgets.QPushButton)
        self.assertEqual({button.text() for button in buttons}, {"开始", "停止", "暂停", "继续"})
        # GNSS 仍在 Splitter 右侧。
        self.assertIsInstance(self.window.gnss_map.parentWidget(), QtWidgets.QSplitter)
        splitter = self.window.findChild(QtWidgets.QSplitter, "main_splitter")
        self.assertIsNotNone(splitter)
        self.assertEqual(splitter.indexOf(self.window.gnss_map), 1)
        # 中文菜单与动作仍在。
        self.assertEqual([a.text() for a in self.window.menuBar().actions()], ["任务", "B-scan", "视图", "帮助"])
        self.assertEqual(self.window.action_show_details.objectName(), "action_show_details")
        # 状态栏仍只有三个业务标签。
        labels = [
            label
            for label in self.window.statusBar().findChildren(QtWidgets.QLabel)
            if label.objectName()
            in {"connection_status_label", "trace_status_label", "acquisition_status_label"}
        ]
        self.assertEqual(len(labels), 3)
        # B-scan 滑块仍在主界面。
        self.assertIsNotNone(self.window.bscan_window_slider)

    # ---- 第2轮审查修复：on_trace 提交边界 ----

    @staticmethod
    def _dual_payload(
        time_ns: np.ndarray,
        *,
        offset: float = 0.0,
        channels: list[str] | None = None,
        processing: dict[str, object] | None = None,
        acquisition: dict[str, object] | None = None,
        gnss: dict[str, object] | None = None,
    ) -> bytes:
        buffer = io.BytesIO()
        payload: dict[str, object] = {
            "frequency_hz": np.array([1.0, 2.0, 3.0]),
            "time_ns": time_ns,
            "display_S11_real": np.arange(3, dtype=float) + offset,
            "display_S11_imag": np.zeros(3),
            "display_S22_real": np.arange(3, dtype=float) + offset + 1,
            "display_S22_imag": np.zeros(3),
            "trace_S11": np.arange(len(time_ns), dtype=float) + offset,
            "trace_S22": np.arange(len(time_ns), dtype=float) + offset + 1,
        }
        if channels is not None:
            payload["channels"] = json.dumps(channels)
        if processing is not None:
            payload["processing"] = json.dumps(processing)
        if acquisition is not None:
            payload["acquisition"] = json.dumps(acquisition)
        if gnss is not None:
            payload["gnss"] = json.dumps(gnss)
        np.savez_compressed(buffer, **payload)
        return buffer.getvalue()

    @staticmethod
    def _trace_header(
        trace_index: int,
        *,
        channels: list[str] | None = None,
        acquisition: dict[str, object] | None = None,
        gnss: dict[str, object] | None = None,
    ) -> dict[str, object]:
        header: dict[str, object] = {
            "type": "trace",
            "protocol": "uav-gpr-telemetry-v1",
            "run_id": "reviewfix",
            "trace_index": trace_index,
            "timestamp": "2026-08-14T00:00:00",
        }
        if channels is not None:
            header["channels"] = channels
        if acquisition is not None:
            header["acquisition"] = acquisition
        if gnss is not None:
            header["gnss"] = gnss
        return header

    def _snapshot(self) -> dict[str, object]:
        w = self.window
        return {
            "bscan_s11": [np.asarray(r).copy() for r in w.bscan_rows["S11"]],
            "bscan_s22": [np.asarray(r).copy() for r in w.bscan_rows["S22"]],
            "bscan_time": None if w.bscan_time_ns is None else np.asarray(w.bscan_time_ns).copy(),
            "bscan_sig": w.bscan_processing_signature,
            "acq": w.acquisition_label.text(),
            "legacy": w.legacy_status.text(),
            "dewow": w.dewow_status.text(),
            "trace_status": w.trace_status_label.text(),
        }

    def _assert_unchanged(self, before: dict[str, object], after: dict[str, object]) -> None:
        for key in before:
            b, a = before[key], after[key]
            if isinstance(b, np.ndarray):
                np.testing.assert_array_equal(a, b, err_msg=f"{key} 应保持不变")
            elif isinstance(b, list):
                self.assertEqual(len(a), len(b), f"{key} 数量不应变化")
                for ba, bb in zip(a, b):
                    np.testing.assert_array_equal(np.asarray(ba), np.asarray(bb), err_msg=f"{key} 内容不应变化")
            else:
                self.assertEqual(a, b, f"{key} 不应变化")

    def test_time_axis_incompatible_trace_rejected_without_committing_state(self) -> None:
        w = self.window
        headers = [
            self._trace_header(
                1,
                channels=["S11", "S22"],
                acquisition={"acquired_at_utc": "2026-08-14T00:00:00", "device_sweep_index": 1},
                gnss={"fix_quality": 1, "latitude": 30.0, "longitude": 114.0},
            ),
            self._trace_header(
                2,
                channels=["S11", "S22"],
                acquisition={"acquired_at_utc": "2026-08-15T00:00:00", "device_sweep_index": 2},
                gnss={"fix_quality": 1, "latitude": 40.0, "longitude": 120.0},
            ),
        ]
        payloads = [
            self._dual_payload(
                np.arange(4, dtype=float),
                channels=["S11", "S22"],
                processing={"dew" + "ow_enabled": False},
            ),
            self._dual_payload(
                np.arange(5, dtype=float),
                offset=5.0,
                channels=["S11", "S22"],
                processing={"dew" + "ow_enabled": False},
            ),
        ]
        w.on_trace(headers[0], payloads[0], "t1.npz")
        before = self._snapshot()
        log_before = w.log_box.toPlainText()

        gnss_calls = [0]

        def counted(*args, **kwargs):
            gnss_calls[0] += 1
            return w.gnss_map._orig_update_fix(*args, **kwargs)

        w.gnss_map._orig_update_fix = w.gnss_map.update_fix
        w.gnss_map.update_fix = counted

        w.on_trace(headers[1], payloads[1], "t2.npz")
        after = self._snapshot()

        self.assertEqual(gnss_calls[0], 0, "被拒绝的 Trace 不得更新 GNSS")
        self._assert_unchanged(before, after)
        self.assertIn("B-scan已拒绝", w.bscan_filter_status.text())
        self.assertIn("时间轴", w.bscan_filter_status.text())
        log = w.log_box.toPlainText()
        self.assertIn("B-scan已拒绝", log)
        # 第二次定向修复：普通成功日志每 25 条汇总，"已收到 Trace N"不再
        # 逐条出现；被拒绝 Trace 只产生即时拒绝日志。
        self.assertNotIn("已收到 Trace 2", log)
        w.gnss_map.update_fix = w.gnss_map._orig_update_fix

    def test_dewow_signature_incompatible_trace_rejected_without_committing_state(self) -> None:
        w = self.window
        headers = [
            self._trace_header(
                1,
                channels=["S11", "S22"],
                acquisition={"acquired_at_utc": "2026-08-14T00:00:00", "device_sweep_index": 1},
                gnss={"fix_quality": 1, "latitude": 30.0, "longitude": 114.0},
            ),
            self._trace_header(
                2,
                channels=["S11", "S22"],
                acquisition={"acquired_at_utc": "2026-08-15T00:00:00", "device_sweep_index": 2},
                gnss={"fix_quality": 1, "latitude": 40.0, "longitude": 120.0},
            ),
        ]
        payloads = [
            self._dual_payload(
                np.arange(4, dtype=float),
                channels=["S11", "S22"],
                processing={"dew" + "ow_enabled": False},
            ),
            self._dual_payload(
                np.arange(4, dtype=float),
                offset=5.0,
                channels=["S11", "S22"],
                processing={"dew" + "ow_enabled": True, "dew" + "ow_window_ns": 4.0},
            ),
        ]
        w.on_trace(headers[0], payloads[0], "t1.npz")
        before = self._snapshot()
        log_before = w.log_box.toPlainText()

        gnss_calls = [0]

        def counted(*args, **kwargs):
            gnss_calls[0] += 1
            return w.gnss_map._orig_update_fix(*args, **kwargs)

        w.gnss_map._orig_update_fix = w.gnss_map.update_fix
        w.gnss_map.update_fix = counted

        w.on_trace(headers[1], payloads[1], "t2.npz")
        after = self._snapshot()

        self.assertEqual(gnss_calls[0], 0, "被拒绝的 Trace 不得更新 GNSS")
        self._assert_unchanged(before, after)
        self.assertIn("B-scan已拒绝", w.bscan_filter_status.text())
        self.assertIn("Dewow", w.bscan_filter_status.text())
        log = w.log_box.toPlainText()
        self.assertIn("B-scan已拒绝", log)
        # 普通成功日志每 25 条汇总（第二次定向修复），无逐条"已收到 Trace N"。
        self.assertNotIn("已收到 Trace 2", log)
        w.gnss_map.update_fix = w.gnss_map._orig_update_fix

    def test_success_path_appends_and_updates_state(self) -> None:
        w = self.window
        w.on_trace(
            self._trace_header(
                1,
                channels=["S11", "S22"],
                acquisition={"acquired_at_utc": "2026-08-14T00:00:00", "device_sweep_index": 1},
                gnss={"fix_quality": 1, "latitude": 30.0, "longitude": 114.0},
            ),
            self._dual_payload(
                np.arange(4, dtype=float),
                channels=["S11", "S22"],
                processing={"dew" + "ow_enabled": False},
            ),
            "t1.npz",
        )
        w.on_trace(
            self._trace_header(
                2,
                channels=["S11", "S22"],
                acquisition={"acquired_at_utc": "2026-08-15T00:00:00", "device_sweep_index": 2},
                gnss={"fix_quality": 1, "latitude": 40.0, "longitude": 120.0},
            ),
            self._dual_payload(
                np.arange(4, dtype=float),
                offset=5.0,
                channels=["S11", "S22"],
                processing={"dew" + "ow_enabled": False},
            ),
            "t2.npz",
        )
        self.assertEqual(len(self.window.bscan_rows["S11"]), 2, "成功 Trace 应追加 HH")
        self.assertEqual(len(self.window.bscan_rows["S22"]), 2, "成功 Trace 应追加 VV")
        # 第二次定向修复：成功 Trace 不逐条记日志（每 25 条汇总）。
        self.assertNotIn("已收到 Trace 2", self.window.log_box.toPlainText())
        self.assertIn("当前 Trace：2", self.window.trace_status_label.text())

    # ---- 第3轮视觉收口：窗口尺寸适配 + 主界面布局 ----

    def test_window_minimum_size_protects_usability(self) -> None:
        # 最小尺寸：保证菜单/按钮/两幅 B-scan/GNSS/状态栏在极端缩放下仍可显示。
        # 测试在 1280×800 的模拟屏幕下进行（设计契约：屏幕 ≥ 960×600 时最小尺寸
        # 保持 960×600）；offscreen 平台默认 800×800 不满足契约，会被
        # ``min(design_min, initial)`` 收紧，契约验证需在已知足够大的屏幕下进行。
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        screen = app.primaryScreen()
        self.assertIsNotNone(screen, "测试环境应至少有一个 primaryScreen")
        with patch.object(
            screen, "availableGeometry", return_value=QtCore.QRect(0, 0, 1280, 800)
        ):
            with patch.object(ground, "ReceiverThread", DummyReceiver):
                window = ground.GroundStationWindow(
                    "127.0.0.1", 0, Path(tempfile.mkdtemp())
                )
            min_size = window.minimumSize()
            self.assertGreaterEqual(min_size.width(), 960, "宽度最小 960")
            self.assertGreaterEqual(min_size.height(), 600, "高度最小 600")
            window.close()
            window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(
                None, QtCore.QEvent.Type.DeferredDelete
            )

    def test_start_stop_buttons_do_not_stretch_horizontally(self) -> None:
        # 按钮在主窗口里宽度受限，不应无限拉伸；最小高度保证可点击。
        self.assertLessEqual(self.window.start_button.maximumWidth(), 160)
        self.assertLessEqual(self.window.stop_button.maximumWidth(), 160)
        self.assertGreaterEqual(self.window.start_button.minimumHeight(), 28)
        self.assertGreaterEqual(self.window.stop_button.minimumHeight(), 28)

    def test_compute_initial_window_size_exact_matrix(self) -> None:
        # 七种分辨率下逐一断言精确初始尺寸（第3轮第二次审查修复：宽高必须完全
        # 独立计算，任一维受限不得影响另一维的 target 限制）。
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        screen = app.primaryScreen()
        self.assertIsNotNone(screen, "测试环境应至少有一个 primaryScreen")
        # (可用宽, 可用高) -> 期望 (initial_w, initial_h)。
        # 每维独立：initial = min(target, int(avail*0.95))（不与 min 耦合）。
        expected_matrix = (
            (800, 600, (760, 570)),
            (1024, 600, (972, 570)),
            (1280, 800, (1216, 760)),
            (1366, 768, (1297, 729)),
            (1920, 1080, (1380, 860)),
            # 宽高解耦回归保护：高度 600 受限不影响宽度命中 target 1380。
            (1920, 600, (1380, 570)),
            # 宽度 800 受限不影响高度命中 target 860。
            (800, 1080, (760, 860)),
        )
        for width, height, expected in expected_matrix:
            with self.subTest(available=(width, height)):
                with patch.object(
                    screen,
                    "availableGeometry",
                    return_value=QtCore.QRect(0, 0, width, height),
                ):
                    size = ground._compute_initial_window_size()
                # 与生产代码一致的整数规则：cap = int(avail * 0.95)。
                expected_cap_w = int(width * 0.95)
                expected_cap_h = int(height * 0.95)
                self.assertEqual(size, expected, "初始尺寸必须精确匹配")
                self.assertLessEqual(size[0], expected_cap_w, "宽度不得超过 95% 可用区域")
                self.assertLessEqual(size[1], expected_cap_h, "高度不得超过 95% 可用区域")
                self.assertGreater(size[0], 0, "宽度必须为正")
                self.assertGreater(size[1], 0, "高度必须为正")

    def test_compute_initial_window_size_decoupled_dimensions(self) -> None:
        # 专门抓住宽高耦合错误：旧实现“任一维 cap < min 就整体返回 cap”会使
        # 1920×600 → (1824, 570)、800×1080 → (760, 1026)。独立按维后：
        # 1920×600 宽度命中 target 1380；800×1080 高度命中 target 860。
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        screen = app.primaryScreen()
        self.assertIsNotNone(screen)
        with patch.object(
            screen, "availableGeometry", return_value=QtCore.QRect(0, 0, 1920, 600)
        ):
            size_for_1920x600 = ground._compute_initial_window_size()
        with patch.object(
            screen, "availableGeometry", return_value=QtCore.QRect(0, 0, 800, 1080)
        ):
            size_for_800x1080 = ground._compute_initial_window_size()
        self.assertEqual(size_for_1920x600, (1380, 570))
        self.assertEqual(size_for_800x1080, (760, 860))

    def test_compute_initial_window_size_fits_available_geometry(self) -> None:
        # 每个维度独立满足：≤ 该维 95% 可用区域（与生产代码一致的整数规则）。
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        screen = app.primaryScreen()
        self.assertIsNotNone(screen, "测试环境应至少有一个 primaryScreen")
        for width, height in ((1366, 768), (1920, 1080), (1280, 800)):
            with patch.object(
                screen, "availableGeometry", return_value=QtCore.QRect(0, 0, width, height)
            ):
                w, h = ground._compute_initial_window_size()
                expected_cap_w = int(width * 0.95)
                expected_cap_h = int(height * 0.95)
                self.assertLessEqual(w, expected_cap_w, f"{width}×{height} 下宽度应不超过 95%，实际 {w}")
                self.assertLessEqual(h, expected_cap_h, f"{width}×{height} 下高度应不超过 95%，实际 {h}")
                self.assertGreater(w, 0, "宽度必须为正")
                self.assertGreater(h, 0, "高度必须为正")

    def test_compute_initial_window_size_no_qappinstance_falls_back(self) -> None:
        # QApplication.instance() 为 None 时（模块调用点 patch，不销毁全局
        # QApplication）退化到 1280×800。
        with patch.object(
            ground.QtWidgets.QApplication, "instance", return_value=None
        ):
            size = ground._compute_initial_window_size()
        self.assertEqual(size, (1280, 800))

    def test_compute_initial_window_size_no_screen_falls_back(self) -> None:
        # primaryScreen() 为 None 时退化到 1280×800；离屏/未初始化场景不崩溃。
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with patch.object(app, "primaryScreen", return_value=None):
            size = ground._compute_initial_window_size()
        self.assertEqual(size, (1280, 800))

    def test_compute_initial_window_size_geometry_raises_falls_back(self) -> None:
        # availableGeometry() 抛异常时退化到 1280×800。
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        screen = app.primaryScreen()
        self.assertIsNotNone(screen)
        with patch.object(
            screen,
            "availableGeometry",
            side_effect=RuntimeError("screen subsystem failure"),
        ):
            size = ground._compute_initial_window_size()
        self.assertEqual(size, (1280, 800))

    def test_compute_initial_window_size_zero_width_falls_back(self) -> None:
        # 可用宽度为 0 时退化到 1280×800（不产生 0 宽初始尺寸）。
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        screen = app.primaryScreen()
        self.assertIsNotNone(screen)
        with patch.object(
            screen, "availableGeometry", return_value=QtCore.QRect(0, 0, 0, 800)
        ):
            size = ground._compute_initial_window_size()
        self.assertEqual(size, (1280, 800))

    def test_compute_initial_window_size_zero_height_falls_back(self) -> None:
        # 可用高度为 0 时退化到 1280×800（不产生 0 高初始尺寸）。
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        screen = app.primaryScreen()
        self.assertIsNotNone(screen)
        with patch.object(
            screen, "availableGeometry", return_value=QtCore.QRect(0, 0, 1366, 0)
        ):
            size = ground._compute_initial_window_size()
        self.assertEqual(size, (1280, 800))

    def test_compute_initial_window_size_tiny_positive_stays_positive(self) -> None:
        # 极小但正数的可用区域：初始尺寸不会为 0 或负数（max(1, ...) 兜底）。
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        screen = app.primaryScreen()
        self.assertIsNotNone(screen)
        for width, height in ((1, 1), (7, 3), (10, 10), (100, 40)):
            with self.subTest(available=(width, height)):
                with patch.object(
                    screen,
                    "availableGeometry",
                    return_value=QtCore.QRect(0, 0, width, height),
                ):
                    size = ground._compute_initial_window_size()
                self.assertGreater(size[0], 0, f"{width}×{height} 下宽度必须为正")
                self.assertGreater(size[1], 0, f"{width}×{height} 下高度必须为正")
                # 极小屏幕同样不得超过该维 95% 上限（cap 被 max(1,...) 兜底为 1）。
                self.assertLessEqual(size[0], max(1, int(width * 0.95)))
                self.assertLessEqual(size[1], max(1, int(height * 0.95)))

    def _build_window_for_screen(self, width: int, height: int):
        """在指定模拟屏幕下构造新窗口对象（不共享 setUp 的窗口，避免污染）。"""
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        screen = app.primaryScreen()
        self.assertIsNotNone(screen)
        with patch.object(
            screen, "availableGeometry", return_value=QtCore.QRect(0, 0, width, height)
        ):
            with patch.object(ground, "ReceiverThread", DummyReceiver):
                window = ground.GroundStationWindow(
                    "127.0.0.1", 0, Path(tempfile.mkdtemp())
                )
        return window

    def test_initial_window_fits_small_screen_1366x768(self) -> None:
        window = self._build_window_for_screen(1366, 768)
        try:
            self.assertLessEqual(window.size().width(), int(1366 * 0.95))
            self.assertLessEqual(window.size().height(), int(768 * 0.95))
            self.assertGreaterEqual(window.size().width(), 960)
            self.assertGreaterEqual(window.size().height(), 600)
        finally:
            window.close()
            window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(
                None, QtCore.QEvent.Type.DeferredDelete
            )

    def test_initial_window_fits_fullhd_1920x1080(self) -> None:
        window = self._build_window_for_screen(1920, 1080)
        try:
            self.assertLessEqual(window.size().width(), int(1920 * 0.95))
            self.assertLessEqual(window.size().height(), int(1080 * 0.95))
            self.assertEqual(window.size().width(), 1380, "1080p 充裕时应达到目标尺寸")
            self.assertEqual(window.size().height(), 860)
        finally:
            window.close()
            window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(
                None, QtCore.QEvent.Type.DeferredDelete
            )

    def test_initial_window_fits_tiny_screen_800x600(self) -> None:
        # 小屏幕（800×600）下窗口不得超出屏幕可视范围：两维 cap=(760, 570)
        # 均低于设计最小值，初始尺寸直接取 cap。
        window = self._build_window_for_screen(800, 600)
        try:
            w, h = window.size().width(), window.size().height()
            self.assertLessEqual(w, 800, f"800×600 屏幕上窗口宽度不得超过 800，实际 {w}")
            self.assertLessEqual(h, 600, f"800×600 屏幕上窗口高度不得超过 600，实际 {h}")
            self.assertGreater(w, 0, "窗口宽度必须为正")
            self.assertGreater(h, 0, "窗口高度必须为正")
            # 与函数层契约一致：初始尺寸 = 95% 可用区域 = (760, 570)。
            self.assertEqual(w, 760, "800×600 屏幕上初始宽度应为 95%×800=760")
            self.assertEqual(h, 570, "800×600 屏幕上初始高度应为 95%×600=570")
        finally:
            window.close()
            window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(
                None, QtCore.QEvent.Type.DeferredDelete
            )

    def test_minimum_size_relaxes_on_tiny_screen(self) -> None:
        # 小屏幕上 setMinimumSize 收紧到 ``min(design_min, initial)``，
        # 避免 Qt 把窗口重新撑大到 960×600 再次溢出。
        window = self._build_window_for_screen(800, 600)
        try:
            min_size = window.minimumSize()
            self.assertEqual(
                min_size.width(), 760,
                "800×600 屏幕上最小宽度应被收紧到 760，避免溢出",
            )
            self.assertEqual(
                min_size.height(), 570,
                "800×600 屏幕上最小高度应被收紧到 570，避免溢出",
            )
        finally:
            window.close()
            window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(
                None, QtCore.QEvent.Type.DeferredDelete
            )

    def test_minimum_size_relaxes_per_dimension_on_odd_screens(self) -> None:
        # 宽高解耦的动态 min：1920×600 → initial=(1380,570), minimum=(960,570)；
        # 800×1080 → initial=(760,860), minimum=(760,600)。每维独立收紧。
        for width, height, expected_initial, expected_min in (
            (1920, 600, (1380, 570), (960, 570)),
            (800, 1080, (760, 860), (760, 600)),
        ):
            with self.subTest(available=(width, height)):
                window = self._build_window_for_screen(width, height)
                try:
                    self.assertEqual(
                        (window.size().width(), window.size().height()),
                        expected_initial,
                    )
                    self.assertEqual(
                        (window.minimumSize().width(), window.minimumSize().height()),
                        expected_min,
                    )
                finally:
                    window.close()
                    window.deleteLater()
                    QtCore.QCoreApplication.sendPostedEvents(
                        None, QtCore.QEvent.Type.DeferredDelete
                    )

    def test_initial_window_does_not_exceed_screen_on_medium_1024x600(self) -> None:
        # 1024×600 屏幕：宽度 cap=972 ≥ min_w=960，高度 cap=570 < min_h=600。
        # 独立按维算法下高度不受 min 强制抬高，初始尺寸仍为 (972, 570)，
        # 完整装入 1024×600 屏幕（高度 570 ≤ 600）。
        with patch.object(
            QtWidgets.QApplication.instance().primaryScreen(),
            "availableGeometry",
            return_value=QtCore.QRect(0, 0, 1024, 600),
        ):
            size = ground._compute_initial_window_size()
            self.assertEqual(size, (972, 570))
            self.assertLessEqual(size[0], 1024, "宽度不得超过屏幕宽度")
            self.assertLessEqual(size[1], 600, "高度不得超过屏幕高度")

    # ---- 第3轮第二次审查修复：真实布局几何测试（show + processEvents 后） ----

    @contextlib.contextmanager
    def _shown_window_for_screen(self, width: int, height: int):
        """在模拟屏幕下构造并显示窗口，patch 在显示与几何读取期间保持有效。

        构造用 DummyReceiver 且 start_receiver=False，不启动真实网络监听；
        上下文退出时 hide/close/deleteLater 清理窗口。
        """
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        screen = app.primaryScreen()
        self.assertIsNotNone(screen)
        with patch.object(
            screen, "availableGeometry", return_value=QtCore.QRect(0, 0, width, height)
        ):
            with patch.object(ground, "ReceiverThread", DummyReceiver):
                window = ground.GroundStationWindow(
                    "127.0.0.1", 0, Path(tempfile.mkdtemp()), start_receiver=False
                )
            window.show()
            app.processEvents()
            try:
                yield window
            finally:
                window.hide()
                window.close()
                window.deleteLater()
                QtCore.QCoreApplication.sendPostedEvents(
                    None, QtCore.QEvent.Type.DeferredDelete
                )
                app.processEvents()

    def _widget_rect_in_window(
        self, window: QtWidgets.QWidget, widget: QtWidgets.QWidget
    ) -> QtCore.QRect:
        """把控件矩形映射到主窗口坐标系。"""
        top_left = widget.mapTo(window, QtCore.QPoint(0, 0))
        return QtCore.QRect(top_left, widget.size())

    def _key_widgets(self, window) -> list[tuple[QtWidgets.QWidget, str]]:
        return [
            (window.start_button, "开始按钮"),
            (window.stop_button, "停止按钮"),
            (window.bscan_containers["S11"], "HH(S11) B-scan"),
            (window.bscan_containers["S22"], "VV(S22) B-scan"),
            (window.gnss_map, "GNSS 地图"),
            (window.bscan_window_slider, "历史滑块"),
            (window.bscan_window_label, "历史滑块标签"),
            (window.connection_status_label, "连接状态"),
            (window.trace_status_label, "当前 Trace"),
            (window.acquisition_status_label, "采集状态"),
        ]

    def test_main_widgets_visible_nonzero_and_inside_window_when_shown(self) -> None:
        # show() + processEvents 后：关键控件对主窗口可见、尺寸非零、且完整
        # 位于主窗口矩形内（越界即布局缺陷，不只检查“大于 0”）。
        for width, height in ((1366, 768), (1920, 1080)):
            with self.subTest(available=(width, height)):
                with self._shown_window_for_screen(width, height) as window:
                    window_rect = window.rect()
                    for widget, name in self._key_widgets(window):
                        self.assertTrue(
                            widget.isVisibleTo(window), f"{name} 应对主窗口可见"
                        )
                        size = widget.size()
                        self.assertGreater(size.width(), 0, f"{name} 宽度为 0")
                        self.assertGreater(size.height(), 0, f"{name} 高度为 0")
                        rect = self._widget_rect_in_window(window, widget)
                        self.assertTrue(
                            window_rect.contains(rect),
                            f"{name} 矩形 {rect} 超出主窗口 {window_rect}",
                        )

    def test_key_widgets_do_not_overlap_when_shown(self) -> None:
        # 同一坐标系下验证关键业务区块互不重叠：HH/VV/滑块/B-scan 区/GNSS。
        for width, height in ((1366, 768), (1920, 1080)):
            with self.subTest(available=(width, height)):
                with self._shown_window_for_screen(width, height) as window:
                    hh = self._widget_rect_in_window(
                        window, window.bscan_containers["S11"]
                    )
                    vv = self._widget_rect_in_window(
                        window, window.bscan_containers["S22"]
                    )
                    slider = self._widget_rect_in_window(
                        window, window.bscan_window_slider
                    )
                    area = self._widget_rect_in_window(
                        window, window.findChild(QtWidgets.QWidget, "bscan_area")
                    )
                    gnss = self._widget_rect_in_window(window, window.gnss_map)
                    for name_a, rect_a, name_b, rect_b in (
                        ("HH B-scan", hh, "VV B-scan", vv),
                        ("HH B-scan", hh, "历史滑块", slider),
                        ("VV B-scan", vv, "历史滑块", slider),
                        ("B-scan 区", area, "GNSS", gnss),
                    ):
                        self.assertFalse(
                            rect_a.intersects(rect_b),
                            f"{name_a} 与 {name_b} 不应重叠：{rect_a} ∩ {rect_b}",
                        )

    def test_splitter_ratio_and_effective_widths_when_shown(self) -> None:
        # 显示后读取 splitter 实际尺寸：B-scan 左区约 60–80%、GNSS 约 20–40%；
        # GNSS 有效宽度 ≥ 150、B-scan 区 ≥ 300（仅对 1366×768 / 1920×1080）。
        for width, height in ((1366, 768), (1920, 1080)):
            with self.subTest(available=(width, height)):
                with self._shown_window_for_screen(width, height) as window:
                    splitter = window.findChild(QtWidgets.QSplitter, "main_splitter")
                    self.assertIsNotNone(splitter)
                    sizes = splitter.sizes()
                    self.assertEqual(len(sizes), 2)
                    total = sum(sizes)
                    self.assertGreater(total, 0, "splitter 总宽必须为正")
                    left_ratio = sizes[0] / total
                    gnss_ratio = sizes[1] / total
                    self.assertGreaterEqual(
                        left_ratio, 0.60, f"B-scan 区占比过低: {left_ratio:.3f}"
                    )
                    self.assertLessEqual(
                        left_ratio, 0.80, f"B-scan 区占比过高: {left_ratio:.3f}"
                    )
                    self.assertGreaterEqual(
                        gnss_ratio, 0.20, f"GNSS 占比过低: {gnss_ratio:.3f}"
                    )
                    self.assertLessEqual(
                        gnss_ratio, 0.40, f"GNSS 占比过高: {gnss_ratio:.3f}"
                    )
                    area = window.findChild(QtWidgets.QWidget, "bscan_area")
                    self.assertGreaterEqual(
                        area.size().width(), 300, "B-scan 区宽度至少 300"
                    )
                    self.assertGreaterEqual(
                        window.gnss_map.size().width(), 150, "GNSS 有效宽度至少 150"
                    )

    def test_two_resolutions_share_same_layout_structure(self) -> None:
        # 1366×768 与 1920×1080 显示后结构一致：HH 几何在 VV 上方、仅两幅
        # PlotWidget、菜单完整。
        for width, height in ((1366, 768), (1920, 1080)):
            with self.subTest(available=(width, height)):
                with self._shown_window_for_screen(width, height) as window:
                    self.assertIn(
                        "HH", window.bscan_plots["S11"].plotItem.titleLabel.text
                    )
                    self.assertIn(
                        "VV", window.bscan_plots["S22"].plotItem.titleLabel.text
                    )
                    hh_rect = self._widget_rect_in_window(
                        window, window.bscan_containers["S11"]
                    )
                    vv_rect = self._widget_rect_in_window(
                        window, window.bscan_containers["S22"]
                    )
                    # HH 在 VV 上方（几何坐标，而非仅布局顺序）。
                    self.assertLess(
                        hh_rect.bottom(), vv_rect.top() + 1,
                        f"{width}×{height} 下 HH({hh_rect}) 应在 VV({vv_rect}) 上方",
                    )
                    plots = window.findChildren(pg.PlotWidget)
                    self.assertEqual(
                        len(plots), 2,
                        f"{width}×{height} 下主窗口仅两幅 B-scan PlotWidget",
                    )
                    actions = [a.text() for a in window.menuBar().actions()]
                    self.assertEqual(actions, ["任务", "B-scan", "视图", "帮助"])

    def test_large_screen_provides_more_display_space(self) -> None:
        # 1920×1080 相比 1366×768：窗口更大、B-scan 区/GNSS/HH/VV 面积不减，
        # splitter 保持两栏。
        metrics = {}
        for width, height in ((1366, 768), (1920, 1080)):
            with self._shown_window_for_screen(width, height) as window:
                area = window.findChild(QtWidgets.QWidget, "bscan_area")
                splitter = window.findChild(QtWidgets.QSplitter, "main_splitter")
                metrics[(width, height)] = {
                    "window": (window.width(), window.height()),
                    "area": (area.width(), area.height()),
                    "gnss": (window.gnss_map.width(), window.gnss_map.height()),
                    "hh": (
                        window.bscan_containers["S11"].width(),
                        window.bscan_containers["S11"].height(),
                    ),
                    "vv": (
                        window.bscan_containers["S22"].width(),
                        window.bscan_containers["S22"].height(),
                    ),
                    "splitter": tuple(splitter.sizes()),
                }
        small, large = metrics[(1366, 768)], metrics[(1920, 1080)]
        self.assertGreaterEqual(
            large["window"][0], small["window"][0], "大屏窗口宽度不应小于小屏"
        )
        self.assertGreaterEqual(
            large["window"][1], small["window"][1], "大屏窗口高度不应小于小屏"
        )
        self.assertGreater(
            large["area"][0] * large["area"][1],
            small["area"][0] * small["area"][1],
            "大屏 B-scan 区面积应增加",
        )
        self.assertGreater(
            large["gnss"][0] * large["gnss"][1],
            small["gnss"][0] * small["gnss"][1],
            "大屏 GNSS 面积应增加",
        )
        self.assertGreaterEqual(
            large["hh"][0] * large["hh"][1],
            small["hh"][0] * small["hh"][1],
            "大屏 HH 容器面积不应减少",
        )
        self.assertGreaterEqual(
            large["vv"][0] * large["vv"][1],
            small["vv"][0] * small["vv"][1],
            "大屏 VV 容器面积不应减少",
        )
        self.assertEqual(len(large["splitter"]), 2, "splitter 应保持两栏")
        self.assertEqual(len(small["splitter"]), 2, "splitter 应保持两栏")

    def test_bscan_y_axis_zero_is_on_top(self) -> None:
        # 0 时间在图顶（yInverted True），单位为 ns。
        for name, plot in self.window.bscan_plots.items():
            view = plot.getPlotItem().getViewBox()
            self.assertTrue(
                view.yInverted(), f"{name} B-scan 0 应在图顶（yInverted）"
            )
            label = plot.getPlotItem().getAxis("left").labelText
            self.assertIn("时间", label)
            units = plot.getPlotItem().getAxis("left").labelUnits
            self.assertEqual(units, "ns")

    def test_bscan_max_64_traces_per_screen(self) -> None:
        # B-scan 每屏固定 64 道，初始 X 范围与之匹配。
        for name, plot in self.window.bscan_plots.items():
            x_range = plot.getViewBox().viewRange()[0]
            self.assertEqual(
                x_range[1] - x_range[0], 64.0,
                f"{name} 初始 X 范围应覆盖 64 道",
            )
        # BSCAN_WINDOW_TRACES 仍是 64（常量契约）。
        self.assertEqual(ground.BSCAN_WINDOW_TRACES, 64)

    def test_slider_value_drives_bscan_window_change(self) -> None:
        # 滑块跟随总道数变化；on_bscan_window_changed 会更新视图与文字。
        # 先喂 65 条兼容 Trace，让滑块范围变成 1..2；标签应反映“共 65 道”。
        from bscan_view import bscan_window_max_start  # type: ignore
        time_ns = np.arange(4, dtype=float)
        payload = self._dual_payload(
            time_ns,
            channels=["S11", "S22"],
            processing={"dew" + "ow_enabled": False},
        )
        for i in range(1, 66):
            header = self._trace_header(
                i,
                channels=["S11", "S22"],
                acquisition={
                    "acquired_at_utc": f"2026-08-14T00:00:{i:02d}",
                    "device_sweep_index": i,
                },
                gnss={"fix_quality": 1, "latitude": 30.0, "longitude": 114.0},
            )
            self.window.on_trace(header, payload, f"t{i}.npz")
        # 共 65 道时，最大可拖起点为 65-64+1=2。
        self.assertEqual(self.window.bscan_window_slider.maximum(), 2)
        self.assertEqual(
            self.window.bscan_window_slider.maximum(),
            bscan_window_max_start(65),
        )
        # 拖到第 2 道起点：标签应反映"显示 2–65 / 共 65 道"。
        self.window.bscan_window_slider.setValue(2)
        self.app.processEvents()
        self.assertIn("共 65 道", self.window.bscan_window_label.text())

    def test_aspect_ratio_setting_still_takes_effect(self) -> None:
        # 长宽比设置仍生效：先记录基线，再 apply 一份不同长宽比，确认容器更新。
        before = self.window.bscan_containers["S11"]._aspect_ratio
        new_settings = ground.BscanUiSettings(aspect_ratio=3.5)
        self.window.apply_bscan_settings(new_settings)
        self.assertAlmostEqual(self.window.bscan_containers["S11"]._aspect_ratio, 3.5)
        self.assertAlmostEqual(self.window.bscan_containers["S22"]._aspect_ratio, 3.5)
        # 还原默认长宽比。
        self.window.apply_bscan_settings(ground.BscanUiSettings(aspect_ratio=before))

    def test_status_bar_only_has_three_business_labels(self) -> None:
        # 状态栏只能保留 3 个业务标签；不允许把 GNSS/采集参数/Dewow 等塞回。
        status = self.window.statusBar()
        labels = status.findChildren(QtWidgets.QLabel)
        object_names = {label.objectName() for label in labels if label.objectName()}
        self.assertEqual(
            object_names,
            {
                "connection_status_label",
                "trace_status_label",
                "acquisition_status_label",
            },
        )

    def test_auxiliary_windows_default_hidden(self) -> None:
        # 日志/详细状态窗口默认隐藏；菜单可重复打开；关闭只隐藏。
        self.assertFalse(self.window.log_window.isVisible(), "日志窗口应默认隐藏")
        self.assertFalse(self.window.details_window.isVisible(), "详细状态窗口应默认隐藏")
        # 打开/关闭各两次，确保可以重复触发且不被销毁。
        for _ in range(2):
            self.window.show_log_window()
            self.assertTrue(self.window.log_window.isVisible())
            self.window.log_window.close()
            self.assertFalse(self.window.log_window.isVisible())
            self.assertIsNotNone(
                self.window.log_window, "日志窗口对象在关闭后仍存在"
            )
            self.window.show_details_window()
            self.assertTrue(self.window.details_window.isVisible())
            self.window.details_window.close()
            self.assertFalse(self.window.details_window.isVisible())
            self.assertIsNotNone(self.window.details_window)

    def test_log_keeps_appending_when_hidden(self) -> None:
        # 详细状态窗口关闭时，log_box 仍能继续追加日志。
        self.window.show_log_window()
        self.window.log_window.close()
        self.window.log("隐藏时仍能写入")
        self.assertIn("隐藏时仍能写入", self.window.log_box.toPlainText())


if __name__ == "__main__":
    unittest.main()
