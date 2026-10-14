"""第7轮：地面端「空中端设备」远程面板（t3；与 t1 黑匣子并行开发）。

覆盖（对照任务卡 acceptance 逐条取证）：
1. 面板五项能力齐全：状态总览（在线/重连中、VNA USB、run_id、是否采集、
   积压、磁盘剩余、程序版本、当前配置，默认 5 秒刷新、间隔可注入）、
   测试雷达、GNSS 三件套（列串口/选串口+波特率/测试 GNSS）、引擎切换、
   模拟器切换；
2. 协议铁律：只新增 action 值（词表与任务卡/t1 完全一致）、无新增消息
   type（线上帧 type 恒为既有 "command"）、framing 既有字段零改动
   （真实 socket 往返 recv_frame 成功 + command header 键集合不变）；
3. 空中端回『不支持』（老空中端纯 ack / supported=false）时面板明确显示
   「对方版本不支持此功能」，不崩不静默；
4. 设置类操作（set_gnss_port/set_engine/set_device_source）有回执确认显示，
   失败回执显示错误原因；
5. 既有地面端功能零回归：开始/停止发送原 action、校准向导连接门控、
   B-scan 追加/拒绝、第1轮冻结 UI 契约（中央按钮集合/菜单栏/两幅
   PlotWidget/状态栏三标签）全部保持；
6. 真实 ReceiverThread 端到端：面板 → 既有 command 帧 → 模拟空中端应答
   （复用 loopback socket 基建伪造）→ 既有 command_ack 通道 + result 字段
   → 面板更新；主界面采集状态不被设备面板流量污染。

全部 offscreen Qt，无真实硬件/USB/网络外联。
"""

from __future__ import annotations

import os
import socket
import struct
import tempfile
import threading
import time
import unittest
import unittest.mock
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

import numpy as np

try:
    from PySide6 import QtCore, QtWidgets
    import uav_gpr_ground_station as ground
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover - env dependent
    QtCore = None
    QtWidgets = None
    ground = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

#: 任务卡/t1 定死的 action 词表（评审对照源）。
TASK_CARD_ACTIONS: tuple[str, ...] = (
    "get_status",
    "test_vna",
    "list_serial_ports",
    "set_gnss_port",
    "test_gnss",
    "set_engine",
    "set_device_source",
)


def _wait_until(predicate, timeout_s: float, interval_s: float = 0.01) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return predicate()


def _wait_until_qt(
    predicate, timeout_s: float, app: QtWidgets.QApplication, interval_s: float = 0.01
) -> bool:
    """带 Qt 事件泵的有界等待（跨线程 queued 信号与定时器必须 processEvents）。"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        app.processEvents()
        if predicate():
            return True
        time.sleep(interval_s)
    app.processEvents()
    return predicate()


# ----------------------------------------------------------------------
# 测试基建：假 receiver（复用 test_ground_station_compact_ui 的 DummyReceiver
# 模式并补第7轮 command_result 信号）
# ----------------------------------------------------------------------


class _FakeReceiver(QtCore.QObject):
    status = QtCore.Signal(str)
    command_status = QtCore.Signal(str)
    trace_received = QtCore.Signal(dict, bytes, str)
    command_result = QtCore.Signal(dict)

    def __init__(self, *args) -> None:
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

    def begin_run(self, _run_id: str) -> None:
        pass

    def note_signal_consumed(self) -> None:
        pass


@unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
class _PanelWindowHarness(unittest.TestCase):
    """真实 GroundStationWindow + 假 receiver（面板行为主战场）。"""

    def setUp(self) -> None:
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.temp = tempfile.TemporaryDirectory()
        self.receiver = _FakeReceiver()
        with unittest.mock.patch.object(
            ground, "ReceiverThread", lambda *args, **kwargs: self.receiver
        ):
            self.window = ground.GroundStationWindow("127.0.0.1", 0, Path(self.temp.name))
        self.panel = self.window.device_panel
        # 定时器停用：除刷新节奏用例外，全部用例确定性驱动（杜绝 5s 拍
        # 偶发插入多余 get_status 调用）。
        self.panel._timer.stop()

    def tearDown(self) -> None:
        try:
            self.panel._timer.stop()
            self.window.close()
            self.window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        except Exception:  # noqa: BLE001 - 清理失败不掩盖断言结果
            pass
        self.app.processEvents()
        self.temp.cleanup()

    # ---- 驱动辅助 ----

    def _connect(self) -> None:
        self.window._set_remote_connected(True)

    def _emit_ack(self, header: dict) -> None:
        """模拟空中端回执（主线程直发，连接为同线程直连，即时生效）。"""
        self.receiver.command_result.emit(dict(header))
        self.app.processEvents()

    def _send_and_ack(self, action: str, params: dict | None, result: dict) -> None:
        """经面板发送 action 并回带回执（完整请求→回执链路）。"""
        before = len(self.receiver.calls)
        self.assertTrue(
            self.panel._send_action(action, params),
            f"{action} 在已连接状态下必须成功发送",
        )
        self.assertEqual(len(self.receiver.calls), before + 1)
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": action,
                "supported": True,
                "result": dict(result),
            }
        )


@unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
class WordListAndPanelStructureTests(_PanelWindowHarness):
    """协议词表 + 面板五项能力控件结构。"""

    def test_word_list_matches_task_contract_exactly(self) -> None:
        # 词表与任务卡/t1 逐字一致（顺序也不变）；改词表必须先上报 captain。
        self.assertEqual(ground.AIR_DEVICE_ACTIONS, TASK_CARD_ACTIONS)
        for action in TASK_CARD_ACTIONS:
            self.assertIn(action, ground.AIR_DEVICE_ACTION_LABELS)
            self.assertTrue(ground.AIR_DEVICE_ACTION_LABELS[action])

    def test_engine_and_source_value_vocabularies(self) -> None:
        self.assertEqual(ground.AIR_ENGINE_VALUES, ("continuous", "legacy"))
        self.assertEqual(ground.AIR_DEVICE_SOURCE_VALUES, ("usb", "simulated"))

    def test_panel_exists_hidden_by_default_and_menu_opens_it(self) -> None:
        panel = self.window.device_panel
        self.assertIsInstance(panel, ground.AirDevicePanel)
        self.assertIsInstance(panel, QtWidgets.QWidget)
        self.assertEqual(panel.objectName(), "device_panel")
        self.assertEqual(panel.windowTitle(), "空中端设备")
        self.assertFalse(panel.isVisible(), "设备面板应默认隐藏（与日志/详细状态同模式）")
        action = self.window.action_show_device_panel
        self.assertEqual(action.objectName(), "action_show_device_panel")
        self.assertIn(action, self.window.menu_view.actions())
        action.trigger()
        self.assertTrue(panel.isVisible(), "视图菜单必须能打开设备面板")
        panel.close()
        self.assertFalse(panel.isVisible())
        self.assertIsNotNone(self.window.device_panel, "关闭只是隐藏，对象仍在")

    def test_five_capabilities_widgets_all_present(self) -> None:
        panel = self.panel
        # ① 状态总览：八项字段标签 + 链路状态 + 手动刷新。
        self.assertIsNotNone(panel.findChild(QtWidgets.QLabel, "device_link_label"))
        self.assertIsNotNone(panel.findChild(QtWidgets.QGroupBox, "device_overview_box"))
        for name in (
            "device_vna_label",
            "device_run_id_label",
            "device_acquiring_label",
            "device_backlog_label",
            "device_disk_label",
            "device_version_label",
            "device_config_label",
            "device_status_updated_label",
        ):
            self.assertIsNotNone(panel.findChild(QtWidgets.QLabel, name), name)
        self.assertIsNotNone(panel.findChild(QtWidgets.QPushButton, "device_refresh_button"))
        # ② 测试雷达。
        self.assertIsNotNone(panel.findChild(QtWidgets.QPushButton, "test_vna_button"))
        self.assertIsNotNone(panel.findChild(QtWidgets.QLabel, "test_vna_result_label"))
        # ③ GNSS 三件套。
        self.assertIsNotNone(panel.findChild(QtWidgets.QPushButton, "list_serial_ports_button"))
        self.assertIsNotNone(panel.findChild(QtWidgets.QComboBox, "gnss_port_combo"))
        self.assertIsNotNone(panel.findChild(QtWidgets.QSpinBox, "gnss_baud_spin"))
        self.assertIsNotNone(panel.findChild(QtWidgets.QPushButton, "set_gnss_port_button"))
        self.assertIsNotNone(panel.findChild(QtWidgets.QPushButton, "test_gnss_button"))
        self.assertIsNotNone(panel.findChild(QtWidgets.QLabel, "gnss_result_label"))
        self.assertEqual(panel.gnss_baud_spin.value(), 115200)
        # ④ 引擎切换 / ⑤ 模拟器切换。
        engine_combo = panel.findChild(QtWidgets.QComboBox, "engine_combo")
        self.assertEqual(
            [engine_combo.itemData(i) for i in range(engine_combo.count())],
            ["continuous", "legacy"],
        )
        self.assertIsNotNone(panel.findChild(QtWidgets.QPushButton, "apply_engine_button"))
        source_combo = panel.findChild(QtWidgets.QComboBox, "panel_device_source_combo")
        self.assertEqual(
            [source_combo.itemData(i) for i in range(source_combo.count())],
            ["usb", "simulated"],
        )
        self.assertIsNotNone(panel.findChild(QtWidgets.QPushButton, "apply_device_source_button"))
        # 回执/提示栏固定存在。
        self.assertIsNotNone(panel.findChild(QtWidgets.QLabel, "panel_notice_label"))

    def test_panel_has_no_abs_display_control(self) -> None:
        # 红线：只显示实部（禁 abs 控件）。
        names = [
            widget.objectName()
            for widget in self.panel.findChildren(QtWidgets.QWidget)
            if widget.objectName()
        ]
        self.assertFalse([name for name in names if "abs" in name.lower()])
        for button in self.panel.findChildren(QtWidgets.QPushButton):
            self.assertNotIn("abs", button.text().lower())


@unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
class RefreshCadenceTests(_PanelWindowHarness):
    """刷新节奏：默认 5 秒、间隔可注入、未连接/隐藏不发送。"""

    def test_default_interval_is_five_seconds(self) -> None:
        # 用全新面板验证（harness 已停用共享面板的定时器以保证确定性）。
        # 面板以 window 为父，tearDown 销毁 window 时一并销毁，无需额外清理。
        fresh = ground.AirDevicePanel(self.window)
        self.assertEqual(fresh.refresh_interval_ms, 5000)
        self.assertEqual(fresh._timer.interval(), 5000)
        self.assertTrue(fresh._timer.isActive(), "构造后定时器应自动启动")

    def test_interval_injectable_at_construction_and_runtime(self) -> None:
        injected = ground.AirDevicePanel(self.window, refresh_interval_ms=25)
        self.assertEqual(injected.refresh_interval_ms, 25)
        self.assertEqual(injected._timer.interval(), 25)
        self.panel.set_refresh_interval_ms(40)
        self.assertEqual(self.panel.refresh_interval_ms, 40)
        self.assertEqual(self.panel._timer.interval(), 40)
        # 非法值（<=0）拒绝并保持原值。
        self.panel.set_refresh_interval_ms(0)
        self.panel.set_refresh_interval_ms(-5)
        self.assertEqual(self.panel.refresh_interval_ms, 40)

    def test_auto_refresh_sends_get_status_only_when_connected_and_visible(self) -> None:
        self.panel.set_refresh_interval_ms(20)
        self.panel._timer.start()
        self.panel.show()
        # 未连接：不发。
        self.assertTrue(
            _wait_until_qt(
                lambda: not self.receiver.calls, 0.5, self.app
            ),
            f"未连接时不应发送，实际 {self.receiver.calls}",
        )
        # 已连接 + 可见：按注入间隔连发 get_status。
        self._connect()
        self.assertTrue(
            _wait_until_qt(
                lambda: len(self.receiver.calls) >= 3, 3.0, self.app
            ),
            f"已连接且可见时 20ms 间隔应连发多次，实际 {self.receiver.calls}",
        )
        for action, params in self.receiver.calls:
            self.assertEqual(action, "get_status")
            self.assertEqual(params, {})
        # 面板隐藏：刷新停止（不再增长）。
        self.panel.hide()
        self.app.processEvents()
        count_at_hide = len(self.receiver.calls)
        self.assertTrue(
            _wait_until_qt(lambda: True, 0.25, self.app),
            "等待几个刷新拍",
        )
        self.assertEqual(
            len(self.receiver.calls), count_at_hide, "面板隐藏后必须停止自动刷新"
        )

    def test_manual_refresh_gated_on_connection(self) -> None:
        # 未连接：按钮不发送并明确提示。
        self.panel.show()
        self.panel.manual_refresh_button.click()
        self.assertEqual(self.receiver.calls, [])
        self.assertIn("未连接", self.panel.panel_notice_label.text())
        # 已连接：发送 get_status。
        self._connect()
        self.panel.manual_refresh_button.click()
        self.assertEqual(self.receiver.calls, [("get_status", {})])

    def test_all_sends_blocked_before_connection_even_for_set_ops(self) -> None:
        self.panel.show()
        self.panel.send_test_vna()
        self.panel.send_list_serial_ports()
        self.panel.send_test_gnss()
        self.panel.send_set_engine()
        self.panel.send_set_device_source()
        self.assertEqual(self.receiver.calls, [])
        self.assertIn("未连接", self.panel.panel_notice_label.text())


@unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
class StatusOverviewTests(_PanelWindowHarness):
    """状态总览回执应用：八项字段、容错、自动同步、脏标记。"""

    def test_full_status_result_updates_all_fields(self) -> None:
        self._connect()
        self.panel.show()
        before = self.panel.device_status_updated_label.text()
        self.assertTrue(self.panel.refresh_now())
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "get_status",
                "supported": True,
                "result": {
                    "online": True,
                    "reconnecting": False,
                    "vna_usb": True,
                    "run_id": "air_task_20260930_120100_0003",
                    "acquiring": True,
                    "backlog": 3,
                    "disk_free_gb": 123.45,
                    "version": "round7-test-1",
                    "config": {
                        "engine": "continuous",
                        "device_source": "usb",
                        "gnss_port": "",
                        "gnss_baud": 115200,
                        "ground_host": "192.168.144.20",
                        "output_dir": "gpr_runs",
                    },
                },
            }
        )
        self.assertEqual(self.panel.device_vna_label.text(), "雷达 USB：已连接")
        self.assertEqual(
            self.panel.device_run_id_label.text(),
            "run_id：air_task_20260930_120100_0003",
        )
        self.assertEqual(self.panel.device_acquiring_label.text(), "是否采集：采集中")
        self.assertEqual(self.panel.device_backlog_label.text(), "积压：3 条")
        self.assertEqual(self.panel.device_disk_label.text(), "磁盘剩余：123.5 GB")
        self.assertEqual(self.panel.device_version_label.text(), "程序版本：round7-test-1")
        config_text = self.panel.device_config_label.text()
        for key in ("engine=continuous", "device_source=usb", "gnss_baud=115200"):
            self.assertIn(key, config_text)
        self.assertEqual(
            self.panel.device_link_label.text(), "连接状态：已连接（空中端在线）"
        )
        self.assertNotEqual(
            self.panel.device_status_updated_label.text(), before, "总览更新时间必须刷新"
        )
        self.assertEqual(self.panel.last_status_result["version"], "round7-test-1")
        # 本次请求触发的回执 → 明确提示。
        self.assertIn("已更新", self.panel.panel_notice_label.text())

    def test_reconnecting_and_offline_flags_displayed(self) -> None:
        self._connect()
        self.panel._send_action("get_status", {})
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "get_status",
                "supported": True,
                "result": {"online": True, "reconnecting": True},
            }
        )
        self.assertEqual(
            self.panel.device_link_label.text(), "连接状态：已连接（对方重连中）"
        )
        self.panel._send_action("get_status", {})
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "get_status",
                "supported": True,
                "result": {"online": False, "reconnecting": False},
            }
        )
        self.assertEqual(
            self.panel.device_link_label.text(), "连接状态：已连接（对方未就绪）"
        )

    def test_missing_fields_tolerated_without_crash(self) -> None:
        """黑匣子 result 超集容错：字段缺失显示 未知/-，绝不崩。"""
        self._connect()
        self.panel._send_action("get_status", {})
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "get_status",
                "supported": True,
                "result": {},
            }
        )
        self.assertEqual(self.panel.device_vna_label.text(), "雷达 USB：未知")
        self.assertEqual(self.panel.device_run_id_label.text(), "run_id：-")
        self.assertEqual(self.panel.device_acquiring_label.text(), "是否采集：未知")
        self.assertEqual(self.panel.device_backlog_label.text(), "积压：未知")
        self.assertEqual(self.panel.device_disk_label.text(), "磁盘剩余：未知")
        self.assertEqual(self.panel.device_version_label.text(), "程序版本：-")
        self.assertEqual(self.panel.device_config_label.text(), "当前配置：-")

    def test_config_syncs_combos_and_dirty_flag_protects_unapplied_edit(self) -> None:
        self._connect()
        config = {"engine": "continuous", "device_source": "usb"}
        self.panel._send_action("get_status", {})
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "get_status",
                "supported": True,
                "result": {"config": config},
            }
        )
        self.assertEqual(self.panel.engine_combo.currentData(), "continuous")
        self.assertEqual(self.panel.device_source_combo.currentData(), "usb")
        # 用户改选 legacy 未应用（activated 仅用户触发，程序同步不置脏）。
        legacy_index = self.panel.engine_combo.findData("legacy")
        self.panel.engine_combo.setCurrentIndex(legacy_index)
        self.panel.engine_combo.activated.emit(legacy_index)
        # 新总览上报 continuous：不得覆盖用户未应用的选择。
        self.panel._send_action("get_status", {})
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "get_status",
                "supported": True,
                "result": {"config": dict(config)},
            }
        )
        self.assertEqual(
            self.panel.engine_combo.currentData(), "legacy", "未应用的用户选择不得被自动同步覆盖"
        )
        # set_engine 成功回执 → 脏标记清除 → 下次总览恢复同步。
        self._send_and_ack("set_engine", {"engine": "legacy"}, {"ok": True, "engine": "legacy"})
        self.panel._send_action("get_status", {})
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "get_status",
                "supported": True,
                "result": {"config": dict(config)},
            }
        )
        self.assertEqual(self.panel.engine_combo.currentData(), "continuous")


@unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
class UnsupportedInteropTests(_PanelWindowHarness):
    """互操作兜底：老空中端/旧版黑匣子回『不支持』时明确显示、不崩、不静默。"""

    def _assert_unsupported_displayed(self, ack: dict, action: str) -> None:
        label = ground.AIR_DEVICE_ACTION_LABELS[action]
        self.assertIn(ground.AirDevicePanel.UNSUPPORTED_TEXT, self.panel.panel_notice_label.text())
        self.assertIn(label, self.panel.panel_notice_label.text())

    def test_plain_ack_without_result_shows_unsupported(self) -> None:
        """老 GUI 空中端对未知 action 只回纯 ack（无 result 字段）。"""
        self._connect()
        self.panel.show()
        self.panel.send_test_vna()
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "test_vna",
            }
        )
        self._assert_unsupported_displayed({}, "test_vna")
        self.assertIn("对方版本不支持此功能", self.panel.test_vna_result_label.text())

    def test_supported_false_flag_shows_unsupported(self) -> None:
        """黑匣子对不认识的 action 回 supported=false（engA 对齐的回执形态）。"""
        self._connect()
        self.panel.send_list_serial_ports()
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "list_serial_ports",
                "supported": False,
            }
        )
        self._assert_unsupported_displayed({}, "list_serial_ports")

    def test_get_status_unsupported_marks_overview_placeholder(self) -> None:
        self._connect()
        self.panel.show()
        self.panel.refresh_now()
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "get_status",
            }
        )
        self.assertIn("对方不支持", self.panel.device_run_id_label.text())
        self.assertIn("对方版本不支持此功能", self.panel.panel_notice_label.text())

    def test_test_gnss_unsupported_shown_in_gnss_section(self) -> None:
        self._connect()
        self.panel.send_test_gnss()
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "test_gnss",
            }
        )
        self.assertIn("对方版本不支持此功能", self.panel.gnss_result_label.text())

    def test_existing_actions_ack_ignored_by_panel(self) -> None:
        """既有命令（start/stop/…）的 ack 不进设备面板（不误报不支持）。"""
        self._connect()
        self.panel._set_notice("就绪")
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "start",
            }
        )
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "stop",
            }
        )
        self.assertEqual(self.panel.panel_notice_label.text(), "就绪")

    def test_unsupported_then_reconnect_clears_state(self) -> None:
        self._connect()
        self.panel.send_test_vna()
        self._emit_ack({"type": "command_ack", "protocol": ground.PROTOCOL, "action": "test_vna"})
        self.window._set_remote_connected(False)
        self.assertIn("未连接", self.panel.device_link_label.text())
        self.assertIn("已断开", self.panel.panel_notice_label.text())
        self.assertEqual(self.panel._pending, {}, "断链必须清空待回执表")


@unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
class ReceiptConfirmationTests(_PanelWindowHarness):
    """设置类操作回执确认（GNSS 三件套 / 引擎 / 模拟器）与测试雷达。"""

    def test_list_serial_ports_populates_combo(self) -> None:
        self._connect()
        self._send_and_ack(
            "list_serial_ports", None, {"ports": ["COM3", {"port": "COM7", "description": "USB-UART"}]}
        )
        combo = self.panel.gnss_port_combo
        self.assertEqual(combo.count(), 2)
        self.assertEqual(combo.currentData(), "COM3")
        self.assertIn("USB-UART", combo.itemText(1))
        self.assertIn("发现 2 个", self.panel.panel_notice_label.text())

    def test_list_serial_ports_empty_shows_notice(self) -> None:
        self._connect()
        self._send_and_ack("list_serial_ports", None, {"ports": []})
        self.assertEqual(self.panel.gnss_port_combo.count(), 0)
        self.assertIn("未发现可用串口", self.panel.panel_notice_label.text())

    def test_set_gnss_port_requires_selected_port(self) -> None:
        self._connect()
        self.panel.set_gnss_button.click()
        self.assertEqual(self.receiver.calls, [])
        self.assertIn("列出串口", self.panel.panel_notice_label.text())

    def test_set_gnss_port_sends_params_and_shows_receipt(self) -> None:
        self._connect()
        self.panel.gnss_port_combo.addItem("COM7（USB-UART）", "COM7")
        self.panel.gnss_port_combo.setCurrentIndex(0)
        self.panel.gnss_baud_spin.setValue(9600)
        self.panel.set_gnss_button.click()
        self.assertEqual(
            self.receiver.calls[-1], ("set_gnss_port", {"port": "COM7", "baud": 9600})
        )
        self.assertIn("等待空中端回执", self.panel.panel_notice_label.text())
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "set_gnss_port",
                "supported": True,
                "result": {"ok": True, "port": "COM7", "baud": 9600},
            }
        )
        self.assertIn("回执确认", self.panel.panel_notice_label.text())
        self.assertIn("COM7 @ 9600", self.panel.panel_notice_label.text())
        self.assertIn("空中端已确认", self.panel.gnss_result_label.text())

    def test_test_vna_result_display(self) -> None:
        self._connect()
        self.panel.send_test_vna()
        self.assertEqual(self.receiver.calls[-1], ("test_vna", {}))
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "test_vna",
                "supported": True,
                "result": {
                    "online": True,
                    "firmware": "v2.1",
                    "protocol": 33,
                    "max_freq_hz": 6_000_000_000,
                },
            }
        )
        text = self.panel.test_vna_result_label.text()
        self.assertIn("在线", text)
        self.assertIn("v2.1", text)
        self.assertIn("33", text)
        self.assertIn("6.00 GHz", text)

    def test_test_gnss_fix_and_no_fix_display(self) -> None:
        self._connect()
        self.panel.send_test_gnss()
        self.assertEqual(self.receiver.calls[-1], ("test_gnss", {}))
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "test_gnss",
                "supported": True,
                "result": {"ok": True, "fix": True, "lat": 31.03, "lon": 121.44},
            }
        )
        self.assertIn("定位成功", self.panel.gnss_result_label.text())
        self.assertIn("31.03", self.panel.gnss_result_label.text())
        self.panel.send_test_gnss()
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "test_gnss",
                "supported": True,
                "result": {"ok": True, "fix": False, "detail": "串口正常但暂无定位"},
            }
        )
        self.assertIn("串口正常", self.panel.gnss_result_label.text())

    def test_set_engine_receipt(self) -> None:
        self._connect()
        self.panel.apply_engine_button.click()
        self.assertEqual(self.receiver.calls[-1], ("set_engine", {"engine": "continuous"}))
        self.assertIn("等待空中端回执", self.panel.panel_notice_label.text())
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "set_engine",
                "supported": True,
                "result": {"ok": True, "engine": "legacy"},
            }
        )
        self.assertIn("回执确认", self.panel.panel_notice_label.text())
        self.assertIn("兼容", self.panel.panel_notice_label.text())
        self.assertIn("legacy", self.panel.panel_notice_label.text())

    def test_set_device_source_receipt(self) -> None:
        self._connect()
        simulated_index = self.panel.device_source_combo.findData("simulated")
        self.panel.device_source_combo.setCurrentIndex(simulated_index)
        self.panel.apply_source_button.click()
        self.assertEqual(
            self.receiver.calls[-1], ("set_device_source", {"device_source": "simulated"})
        )
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "set_device_source",
                "supported": True,
                "result": {"ok": True, "device_source": "simulated"},
            }
        )
        self.assertIn("回执确认", self.panel.panel_notice_label.text())
        self.assertIn("模拟器", self.panel.panel_notice_label.text())

    def test_failed_receipt_shows_error_not_silent(self) -> None:
        self._connect()
        self.panel.apply_engine_button.click()
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "set_engine",
                "supported": True,
                "result": {"ok": False, "engine": "legacy", "error": "采集任务运行中"},
            }
        )
        self.assertIn("回执失败", self.panel.panel_notice_label.text())
        self.assertIn("采集任务运行中", self.panel.panel_notice_label.text())

    def test_receipts_arrive_via_window_forward(self) -> None:
        """回执经主窗口 _on_command_result 转发（既有 ack 通道接线）。"""
        self._connect()
        self.panel.show()
        self.panel.manual_refresh_button.click()
        header = {
            "type": "command_ack",
            "protocol": ground.PROTOCOL,
            "action": "get_status",
            "supported": True,
            "result": {"online": True, "vna_usb": False},
        }
        self.window._on_command_result(header)
        self.assertEqual(self.panel.device_vna_label.text(), "雷达 USB：未连接")


# ----------------------------------------------------------------------
# 既有功能零回归（第1轮冻结契约 + 开始/停止/校准门控/B-scan）
# ----------------------------------------------------------------------


@unittest.skipIf(ground is None, f"ground GUI dependencies unavailable: {IMPORT_ERROR}")
class ExistingGroundZeroRegressionTests(_PanelWindowHarness):
    """设备面板加入后，既有冻结契约与任务控制路径零回归。"""

    def test_round1_frozen_ui_contract_unchanged(self) -> None:
        # 主界面中央按钮仍只有 开始/停止/暂停/继续（设备面板不在中央布局）。
        central = self.window.centralWidget()
        buttons = central.findChildren(QtWidgets.QPushButton)
        self.assertEqual({b.text() for b in buttons}, {"开始", "停止", "暂停", "继续"})
        # 菜单栏四个一级菜单不变。
        self.assertEqual(
            [a.text() for a in self.window.menuBar().actions()],
            ["任务", "B-scan", "视图", "帮助"],
        )
        # 任务菜单前四项冻结顺序不变（单次采集/采集设置/分隔线/退出）。
        task = self.window.menu_task
        self.assertEqual(task.actions()[0], self.window.action_single_acquisition)
        self.assertEqual(task.actions()[1], self.window.action_acquisition_settings)
        self.assertTrue(task.actions()[2].isSeparator())
        self.assertEqual(task.actions()[3], self.window.action_exit)
        # 仍只有两幅 B-scan PlotWidget。
        import pyqtgraph as pg

        plots = self.window.findChildren(pg.PlotWidget)
        self.assertEqual(len(plots), 2)
        # 状态栏仍只有三个业务标签。
        labels = [
            label.objectName()
            for label in self.window.statusBar().findChildren(QtWidgets.QLabel)
            if label.objectName()
        ]
        self.assertEqual(
            set(labels),
            {"connection_status_label", "trace_status_label", "acquisition_status_label"},
        )

    def test_start_stop_single_send_original_actions(self) -> None:
        self.window.start_button.click()
        self.assertEqual(self.receiver.calls[-1][0], "start")
        self.assertEqual(self.receiver.calls[-1][1], self.window.command_params())
        self.window.stop_button.click()
        self.assertEqual(self.receiver.calls[-1], ("stop", None))
        self.window.pause_button.click()
        self.assertEqual(self.receiver.calls[-1][0], "pause")
        self.window.resume_button.click()
        self.assertEqual(self.receiver.calls[-1][0], "resume")
        self.window.action_single_acquisition.trigger()
        action, params = self.receiver.calls[-1]
        self.assertEqual(action, "single")
        self.assertTrue(params["single"])

    def test_calibration_wizard_connection_gating_intact(self) -> None:
        wizard = self.window.calibration_dialog
        self.window._set_remote_connected(False)
        self.assertFalse(wizard.begin_button.isEnabled(), "未连接时开始校准必须禁用")
        self.assertFalse(self.panel._link_connected, "设备面板同步断开")
        self.window._set_remote_connected(True)
        self.assertTrue(wizard.begin_button.isEnabled(), "连接后开始校准必须启用")
        self.assertTrue(self.panel._link_connected, "设备面板同步连接")

    def test_bscan_append_and_reject_unchanged(self) -> None:
        time_ns = np.linspace(0.0, 8.0, 16)
        rows = {"S11": np.arange(16, dtype=float), "S22": np.arange(16, dtype=float) + 1.0}
        self.assertTrue(self.window._append_bscan_trace(time_ns, rows))
        self.assertEqual(self.window.bscan_trace_count(), 1)
        # 时间轴不兼容 → 拒绝（原语义不变）。
        self.assertFalse(self.window._append_bscan_trace(np.linspace(0, 9, 16), rows))
        self.assertEqual(self.window.bscan_trace_count(), 1)

    def test_device_panel_flow_does_not_pollute_main_acquisition_status(self) -> None:
        """设备面板流量不得改动主界面采集状态（第1轮 ACK 状态机专属）。"""
        self._connect()
        self.panel.show()
        self.panel.refresh_now()
        self._emit_ack(
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "get_status",
                "supported": True,
                "result": {"online": True, "acquiring": False},
            }
        )
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：未采集")
        # 既有 start ACK 状态机照常工作。
        self.window._on_command_status("Remote accepted: start")
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：采集中")


# ----------------------------------------------------------------------
# 真实 ReceiverThread 线上协议（framing 零改动 + 无新增消息 type）
# ----------------------------------------------------------------------


def _read_raw_frame(sock: socket.socket) -> tuple[dict, bytes, int, int]:
    """按既有 framing 裸读一帧（4 字节头长 !I + 头 + 8 字节载荷长 !Q）。"""
    header_len = struct.unpack("!I", _recv_exact(sock, 4))[0]
    header_bytes = _recv_exact(sock, header_len)
    header = __import__("json").loads(header_bytes.decode("utf-8"))
    payload_len = struct.unpack("!Q", _recv_exact(sock, 8))[0]
    payload = _recv_exact(sock, payload_len) if payload_len else b""
    return header, payload, header_len, payload_len


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("socket closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


@unittest.skipIf(ground is None, f"ground dependencies unavailable: {IMPORT_ERROR}")
class WireProtocolTests(unittest.TestCase):
    """真实 ReceiverThread：命令帧走既有 type=command + 新 action，framing 不变。"""

    def setUp(self) -> None:
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.tmp = tempfile.TemporaryDirectory()
        self.receiver = ground.ReceiverThread("127.0.0.1", 0, Path(self.tmp.name))
        self.receiver.start()
        self.assertTrue(_wait_until(lambda: self.receiver.bound_port is not None, 5.0))
        self.client = socket.create_connection(("127.0.0.1", self.receiver.bound_port), timeout=5)
        self.client.settimeout(5)
        # accept 循环 0.5s 一拍：等 receiver 真正接管 client 再发命令（免竞态）。
        self.assertTrue(
            _wait_until(lambda: self.receiver._client is not None, 5.0),
            "接收线程未在时限内接管客户端连接",
        )

    def tearDown(self) -> None:
        try:
            self.receiver.stop()
            self.receiver.wait(3000)
        finally:
            try:
                self.client.close()
            except OSError:
                pass
            self.tmp.cleanup()

    def test_all_seven_actions_sent_as_existing_command_frames(self) -> None:
        for action in ground.AIR_DEVICE_ACTIONS:
            params = {"port": "COM9", "baud": 115200} if action == "set_gnss_port" else {}
            self.receiver.send_command(action, dict(params))
        seen: list[str] = []
        for _expected in ground.AIR_DEVICE_ACTIONS:
            header, payload, header_len, payload_len = _read_raw_frame(self.client)
            # 既有 framing 字段零改动：4 字节 !I 头长 + 8 字节 !Q 载荷长，
            # recv_frame/裸读都能解出同一帧。
            self.assertGreater(header_len, 0)
            self.assertEqual(payload_len, 0, "命令帧无载荷（与既有行为一致）")
            # 无新增消息 type：恒为既有 "command"；协议常量不变。
            self.assertEqual(header.get("type"), "command")
            self.assertEqual(header.get("protocol"), ground.PROTOCOL)
            # 只新增 action 值，且取值在词表内。
            self.assertEqual(header.get("action"), _expected)
            seen.append(str(header.get("action")))
            # 既有 command header 键集合不变（type/protocol/action/params/timestamp）。
            self.assertEqual(
                set(header.keys()),
                {"type", "protocol", "action", "params", "timestamp"},
            )
            if _expected == "set_gnss_port":
                self.assertEqual(header.get("params"), {"port": "COM9", "baud": 115200})
        self.assertEqual(seen, list(ground.AIR_DEVICE_ACTIONS))

    def test_plain_ack_and_result_ack_both_surface_on_command_result(self) -> None:
        """老空中端纯 ack / 黑匣子带 result 的 ack 都经 command_result 通道。"""
        received: list[dict] = []
        self.receiver.command_result.connect(
            received.append, type=QtCore.Qt.ConnectionType.DirectConnection
        )
        self.receiver.send_command("get_status", {})
        header, _payload, _hl, _pl = _read_raw_frame(self.client)
        self.assertEqual(header.get("type"), "command")
        # ① 老 GUI 空中端形态：纯 command_ack（无 result/supported 字段）。
        ground.send_frame(
            self.client,
            {"type": "command_ack", "protocol": ground.PROTOCOL, "action": "get_status"},
        )
        self.assertTrue(_wait_until(lambda: len(received) == 1, 3.0))
        self.assertEqual(received[0].get("action"), "get_status")
        self.assertNotIn("result", received[0])
        # ② 黑匣子形态：supported + result 字段原样转发（面板负责解析）。
        ground.send_frame(
            self.client,
            {
                "type": "command_ack",
                "protocol": ground.PROTOCOL,
                "action": "get_status",
                "supported": True,
                "result": {"online": True, "backlog": 0},
            },
        )
        self.assertTrue(_wait_until(lambda: len(received) == 2, 3.0))
        self.assertEqual(received[1].get("result"), {"online": True, "backlog": 0})
        self.assertTrue(received[1].get("supported"))


# ----------------------------------------------------------------------
# 端到端：真实窗口 + 真实 ReceiverThread + 模拟空中端（loopback 伪造应答）
# ----------------------------------------------------------------------


class _FakeAirService(threading.Thread):
    """模拟空中端：get_status 回 result；test_vna 回纯 ack（不支持）。

    复用既有 framing 收发（ground.send_frame/recv_frame），零新消息 type。
    """

    def __init__(self, port: int) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.got: list[dict] = []

    def run(self) -> None:
        sock = socket.create_connection(("127.0.0.1", self.port), timeout=5)
        sock.settimeout(0.2)
        try:
            ground.send_frame(
                sock,
                {
                    "type": "hello",
                    "protocol": ground.PROTOCOL,
                    "run_id": "20260930_120000_idle",
                },
            )
            self.ready.set()
            while not self.stop_event.is_set():
                try:
                    header, _payload = ground.recv_frame(sock)
                except socket.timeout:
                    continue
                except (ConnectionError, OSError):
                    break
                self.got.append(header)
                if header.get("type") != "command":
                    continue
                action = str(header.get("action") or "")
                if action == "get_status":
                    ground.send_frame(
                        sock,
                        {
                            "type": "command_ack",
                            "protocol": ground.PROTOCOL,
                            "action": action,
                            "supported": True,
                            "result": {
                                "online": True,
                                "reconnecting": False,
                                "vna_usb": True,
                                "run_id": "air_task_20260930_120100_0007",
                                "acquiring": False,
                                "backlog": 0,
                                "disk_free_gb": 200.0,
                                "version": "round7-fake-air",
                                "config": {
                                    "engine": "legacy",
                                    "device_source": "usb",
                                    "gnss_port": "",
                                    "gnss_baud": 115200,
                                },
                            },
                        },
                    )
                else:
                    # 老 GUI 空中端形态：纯 ack（无结果字段）。
                    ground.send_frame(
                        sock,
                        {
                            "type": "command_ack",
                            "protocol": ground.PROTOCOL,
                            "action": action,
                        },
                    )
        finally:
            try:
                sock.close()
            except OSError:
                pass


@unittest.skipIf(ground is None, f"ground dependencies unavailable: {IMPORT_ERROR}")
class GroundPanelEndToEndTests(unittest.TestCase):
    """双开端到端（t5 冒烟的前置自测）：面板发送 → 假空中端应答 → 面板更新。"""

    def setUp(self) -> None:
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.tmp = tempfile.TemporaryDirectory()
        self.window = ground.GroundStationWindow(
            "127.0.0.1", 0, Path(self.tmp.name), start_receiver=True
        )
        self.assertTrue(
            _wait_until(lambda: self.window.receiver.bound_port is not None, 5.0)
        )
        self.air = _FakeAirService(int(self.window.receiver.bound_port))
        self.air.start()
        self.assertTrue(self.air.ready.wait(5.0), "模拟空中端未在时限内连上地面端")
        self.panel = self.window.device_panel
        self.panel._timer.stop()

    def tearDown(self) -> None:
        try:
            self.air.stop_event.set()
            self.air.join(3.0)
            self.window.receiver.stop()
            self.assertTrue(self.window.receiver.wait(3000), "接收线程未在时限内退出")
            self.window.close()
            self.window.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        except Exception:  # noqa: BLE001 - 清理失败不掩盖断言结果
            pass
        self.app.processEvents()
        self.tmp.cleanup()

    def test_panel_round_trip_over_real_wire(self) -> None:
        # ① hello 后主窗口连接状态就绪，设备面板同步已连接。
        self.assertTrue(
            _wait_until_qt(
                lambda: self.window.connection_status_label.text() == "连接状态：已连接",
                5.0,
                self.app,
            ),
            f"hello 后未显示已连接: {self.window.connection_status_label.text()}",
        )
        self.panel.show()
        self.assertTrue(
            _wait_until_qt(lambda: self.panel._link_connected, 5.0, self.app),
            "设备面板未同步连接状态",
        )
        # ② 手动刷新 → 假空中端回 get_status result → 总览八项全部更新。
        self.panel.manual_refresh_button.click()
        self.assertTrue(
            _wait_until_qt(
                lambda: self.panel.device_run_id_label.text()
                == "run_id：air_task_20260930_120100_0007",
                5.0,
                self.app,
            ),
            f"状态总览未更新: {self.panel.device_run_id_label.text()} / "
            f"{self.panel.panel_notice_label.text()}",
        )
        self.assertEqual(self.panel.device_vna_label.text(), "雷达 USB：已连接")
        self.assertEqual(self.panel.device_backlog_label.text(), "积压：0 条")
        self.assertEqual(self.panel.device_disk_label.text(), "磁盘剩余：200.0 GB")
        self.assertEqual(self.panel.device_version_label.text(), "程序版本：round7-fake-air")
        self.assertEqual(self.panel.engine_combo.currentData(), "legacy")
        # ③ 测试雷达 → 假空中端（老版形态）纯 ack → 明确显示不支持。
        self.panel.send_test_vna()
        self.assertTrue(
            _wait_until_qt(
                lambda: "对方版本不支持此功能" in self.panel.test_vna_result_label.text(),
                5.0,
                self.app,
            ),
            f"不支持提示未出现: {self.panel.test_vna_result_label.text()}",
        )
        # ④ 线上只有既有消息 type（地面→空中 ack/command；空中→地面由
        #    receiver 校验）。面板流量全部是 command 帧 + 词表内 action。
        self.assertEqual(
            {h.get("type") for h in self.air.got}, {"ack", "command"},
            f"出现了非既有消息 type: {self.air.got}",
        )
        command_frames = [h for h in self.air.got if h.get("type") == "command"]
        for header in command_frames:
            self.assertIn(header.get("action"), ground.AIR_DEVICE_ACTIONS)
        sent_actions = [h.get("action") for h in command_frames]
        self.assertIn("get_status", sent_actions)
        self.assertIn("test_vna", sent_actions)
        # ⑤ 设备面板流量未污染主界面采集状态（hello 用 idle 身份，未建任务）。
        self.assertEqual(self.window.acquisition_status_label.text(), "采集状态：未采集")
        # ⑥ 既有冻结契约在同窗口内仍成立。
        central_buttons = {
            b.text() for b in self.window.centralWidget().findChildren(QtWidgets.QPushButton)
        }
        self.assertEqual(central_buttons, {"开始", "停止", "暂停", "继续"})


if __name__ == "__main__":
    unittest.main()
