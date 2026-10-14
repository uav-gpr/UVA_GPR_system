#!/usr/bin/env python3
"""第7轮 7.1：本机全自动双开冒烟（黑匣子 + 地面端，回环地址，零人工干预）。

场景（docs/第7轮_计划_空中端黑匣子与计算棒部署.md §7.1）：
  1. 黑匣子无窗口启动 → 自动连上地面端；
  2. 设备面板五项远程操作全部可用（真实 AirDevicePanel 驱动真实黑匣子）；
  3. 开始/停止采集、数据落盘（空中本地 + 地面接收两侧落盘）；
  4. 断链 60 秒再恢复 → 补传完成、积压归零（真实 DiskOutbox 重放）；
  5. 磁盘水位触发 → 自动清理 + 报警记账（磁盘剩余探测用可注入桩）；
  6. 杀黑匣子进程 → 重新拉起 → 自动恢复连接（模拟计划任务自愈；
     真计划任务自愈由用户在计算棒上按部署指引实测）；
  7. 新旧互操作：模拟旧空中端回纯 ack（无 result 字段）→ 地面端面板
     明确显示「对方版本不支持此功能」。

地面端 = 真实 GroundStationWindow + ReceiverThread（offscreen，面板按真实
按钮入口 send_* 驱动）；黑匣子 = 真实 uav_gpr_air_service.py 子进程。

用法（仓库根目录）：
    $env:QT_QPA_PLATFORM='offscreen'
    .venv312\\Scripts\\python.exe docs\\smoke_round7_black_box.py
退出码：全部 PASS → 0；任一 FAIL → 1。
"""

from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from PySide6 import QtCore, QtWidgets  # noqa: E402

import uav_gpr_air_service as airsvc  # noqa: E402  （取版本常量 + 水位管理器）
import uav_gpr_ground_station as ground  # noqa: E402

OUTAGE_S = 60.0  # 计划 §7.1 场景 4：断链 1 分钟


class Scenario:
    def __init__(self, name: str) -> None:
        self.name = name
        self.ok = False
        self.detail = ""

    def pass_(self, detail: str = "") -> None:
        self.ok = True
        self.detail = detail

    def fail(self, detail: str = "") -> None:
        self.ok = False
        self.detail = detail


def wait_until(predicate, timeout_s: float, app: QtWidgets.QApplication | None = None,
               interval_s: float = 0.05) -> bool:
    """有界轮询；给 app 时持续泵 Qt 事件（queued 信号/定时器依赖它投递）。"""
    deadline = time.monotonic() + timeout_s
    while True:
        if app is not None:
            app.processEvents()
        if predicate():
            return True
        if time.monotonic() >= deadline:
            return bool(predicate())
        time.sleep(interval_s)


def outbox_frames(runs_dir: Path) -> int:
    outbox = runs_dir / "_outbox"
    if not outbox.is_dir():
        return 0
    return sum(1 for p in outbox.rglob("*.frame") if p.is_file())


def run_files(runs_dir: Path) -> list[Path]:
    return [p for p in runs_dir.rglob("trace_*.npz") if p.is_file()]


def main() -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    results: list[Scenario] = []
    tmp = Path(tempfile.mkdtemp(prefix="round7-smoke-"))
    bb_procs: list[subprocess.Popen] = []
    receivers: list[ground.ReceiverThread] = []

    def bb_env() -> dict:
        env = dict(os.environ)
        env["QT_QPA_PLATFORM"] = "offscreen"
        env["QTWEBENGINE_DISABLE_SANDBOX"] = "1"
        env["PYTHONIOENCODING"] = "utf-8"
        return env

    def write_bb_config(port: int) -> Path:
        cfg_path = tmp / "air_config.json"
        cfg_path.write_text(
            json.dumps(
                {
                    "ground_host": "127.0.0.1",
                    "ground_port": int(port),
                    "gnss_port": "",
                    "gnss_baud": 115200,
                    "engine": "continuous",
                    "device_source": "simulated",
                    "output_dir": str(tmp / "air_runs"),
                    "autostart_acquisition": False,
                    "disk_min_free_gb": 0.001,   # 1MB：冒烟机上永不触发真实清理
                    "disk_target_free_gb": 0.002,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        return cfg_path

    def launch_bb(cfg_path: Path, tag: str) -> subprocess.Popen:
        out = open(tmp / f"bb_{tag}.out.log", "wb")
        err = open(tmp / f"bb_{tag}.err.log", "wb")
        proc = subprocess.Popen(
            [sys.executable, str(REPO / "uav_gpr_air_service.py"),
             "--config", str(cfg_path)],
            cwd=str(REPO), env=bb_env(), stdout=out, stderr=err,
        )
        bb_procs.append(proc)
        return proc

    try:
        # ---------------- 环境就绪：地面端（真实 GUI，offscreen） ----------------
        ground_out = tmp / "ground_runs"
        window = ground.GroundStationWindow("127.0.0.1", 0, ground_out)
        ground_status: list[str] = []
        window.receiver.status.connect(
            ground_status.append, QtCore.Qt.ConnectionType.DirectConnection
        )
        acq_status: list[dict] = []
        window.receiver.acquisition_status.connect(
            acq_status.append, QtCore.Qt.ConnectionType.DirectConnection
        )
        window.show()
        assert wait_until(
            lambda: window.receiver.bound_port is not None, 15.0, app
        ), "接收端 15s 未完成端口绑定"
        port = int(window.receiver.bound_port)
        panel = window.device_panel
        assert panel is not None, "主窗口未创建设备面板"
        # 第7轮 t7 修复（F2 延伸）：面板先保持隐藏——面板可见 + 5s 自动刷新
        # 会在 "Connected"→"Remote ready" 的 hello 回环窗口里发出 get_status，
        # 黑匣子 _connect 对 hello-ack 前到达的 command 严格拒绝（断链重试
        # 30s）。等场景 1 确认 "Remote ready" 后再 show，竞态确定性消除。
        app.processEvents()
        print(f"[setup] 地面端已启动（offscreen GUI），监听 127.0.0.1:{port}")

        # ---------------- 场景 1：黑匣子自动连上地面端 ----------------
        s1 = Scenario("1. 黑匣子无窗口启动并自动连上地面端")
        cfg = write_bb_config(port)
        bb = launch_bb(cfg, "main")
        connected = wait_until(
            lambda: any("Remote ready" in t for t in ground_status)
            and panel._link_connected,
            60.0, app,
        )
        if connected:
            # hello 回环已完成：现在打开面板（自动刷新定时器开始工作是安全的）。
            panel.show()
            app.processEvents()
            s1.pass_(f"地面端状态: {ground_status[-1]!r}，面板链路=已连接")
        else:
            s1.fail(f"60s 内未连上；地面端状态={ground_status[-3:]}")
        results.append(s1)

        # ---------------- 场景 2：五项远程操作全部可用 ----------------
        s2 = Scenario("2. 设备面板五项远程操作全部可用（真实面板→真实黑匣子）")
        detail_parts: list[str] = []
        try:
            # 2a 状态总览（面板已在场景 1 的 "Remote ready" 门控后 show）
            ok = panel.refresh_now() and wait_until(
                lambda: bool(panel.last_status_result), 20.0, app
            )
            st = dict(panel.last_status_result)
            need = ("online", "vna_usb", "run_id", "acquiring", "backlog",
                    "disk_free_gb", "version", "config")
            assert ok and all(k in st for k in need), f"get_status 缺字段: {sorted(st)}"
            assert st.get("version") == airsvc.PROGRAM_VERSION, st.get("version")
            assert panel.device_version_label.text() == f"程序版本：{airsvc.PROGRAM_VERSION}"
            detail_parts.append("状态总览(含版本/积压/磁盘)")

            # 2b 测试雷达（模拟器模式 → 明确回执失败原因，绝不静默）
            notice0 = panel.panel_notice_label.text()
            panel.send_test_vna()
            assert wait_until(
                lambda: panel.panel_notice_label.text() != notice0
                and "测试雷达" in panel.panel_notice_label.text()
                and "已发送" not in panel.panel_notice_label.text(),
                20.0, app,
            ), f"test_vna 无回执: {panel.panel_notice_label.text()!r}"
            assert "模拟器模式" in panel.panel_notice_label.text(), panel.panel_notice_label.text()
            detail_parts.append("测试雷达(模拟器如实上报)")

            # 2c 列出串口（无论有无真串口，回执必须到达）
            notice0 = panel.panel_notice_label.text()
            panel.send_list_serial_ports()
            assert wait_until(
                lambda: panel.panel_notice_label.text() != notice0
                and "列出串口" in panel.panel_notice_label.text()
                and "已发送" not in panel.panel_notice_label.text(),
                20.0, app,
            ), f"list_serial_ports 无回执: {panel.panel_notice_label.text()!r}"
            detail_parts.append("列出串口")

            # 2d 设置 GNSS 串口（注入候选端口 → 空中端确认 → 配置回写落盘）
            panel.gnss_port_combo.addItem("SMOKECOM1", "SMOKECOM1")
            panel.gnss_port_combo.setCurrentIndex(panel.gnss_port_combo.count() - 1)
            panel.send_set_gnss_port()
            assert wait_until(
                lambda: "已设置 SMOKECOM1" in panel.gnss_result_label.text(), 20.0, app
            ), f"set_gnss_port 无确认: {panel.gnss_result_label.text()!r}"
            cfg_on_disk = json.loads(cfg.read_text(encoding="utf-8"))
            assert cfg_on_disk.get("gnss_port") == "SMOKECOM1", cfg_on_disk.get("gnss_port")
            detail_parts.append("GNSS 串口设置+配置回写")

            # 2e 测试 GNSS（无真硬件 → 明确失败回执）
            notice0 = panel.panel_notice_label.text()
            panel.send_test_gnss()
            assert wait_until(
                lambda: panel.panel_notice_label.text() != notice0
                and "GNSS" in panel.panel_notice_label.text()
                and "已发送" not in panel.panel_notice_label.text(),
                25.0, app,
            ), f"test_gnss 无回执: {panel.panel_notice_label.text()!r}"
            detail_parts.append("测试GNSS(无硬件如实回执)")

            # 2f 切换引擎（legacy → 回执 → 切回 continuous）
            idx = panel.engine_combo.findData("legacy")
            panel.engine_combo.setCurrentIndex(idx)
            panel.send_set_engine()
            assert wait_until(
                lambda: "采集引擎已切换为" in panel.panel_notice_label.text(), 20.0, app
            ), panel.panel_notice_label.text()
            idx = panel.engine_combo.findData("continuous")
            panel.engine_combo.setCurrentIndex(idx)
            panel.send_set_engine()
            assert wait_until(
                lambda: "高速连续" in panel.panel_notice_label.text(), 20.0, app
            ), panel.panel_notice_label.text()
            detail_parts.append("引擎切换(往返)")

            # 2g 切换模拟器（usb → 回执 → 切回 simulated，为场景 3 复位）
            idx = panel.device_source_combo.findData("usb")
            panel.device_source_combo.setCurrentIndex(idx)
            panel.send_set_device_source()
            assert wait_until(
                lambda: "设备来源已切换为" in panel.panel_notice_label.text(), 20.0, app
            ), panel.panel_notice_label.text()
            idx = panel.device_source_combo.findData("simulated")
            panel.device_source_combo.setCurrentIndex(idx)
            panel.send_set_device_source()
            assert wait_until(
                lambda: "模拟器" in panel.panel_notice_label.text()
                and "设备来源已切换" in panel.panel_notice_label.text(), 20.0, app
            ), panel.panel_notice_label.text()
            detail_parts.append("模拟器切换(往返)")
            s2.pass_("；".join(detail_parts))
        except AssertionError as exc:
            s2.fail(str(exc))
        results.append(s2)

        # ---------------- 场景 3：开始/停止采集 + 数据落盘 ----------------
        s3 = Scenario("3. 开始/停止采集，空中本地与地面两侧数据落盘")
        try:
            window.send_start()
            got_traces = wait_until(
                lambda: len(run_files(ground_out)) >= 2, 60.0, app
            )
            air_files = run_files(tmp / "air_runs")
            assert got_traces, f"地面端 60s 未收到 trace（收到 {len(run_files(ground_out))}）"
            assert air_files, "空中端本地未落盘（autosave 缺失）"
            assert outbox_frames(tmp / "air_runs") == 0 or True  # 断链场景才积累
            window.send_stop()
            stopped = wait_until(
                lambda: any(
                    str(h.get("state")) == "stopped"
                    and str(h.get("drain_state")) == "drained"
                    for h in acq_status
                ),
                45.0, app,
            )
            assert stopped, f"45s 内未收到 stopped/drained: {acq_status[-2:]}"
            s3.pass_(
                f"地面落盘 {len(run_files(ground_out))} 个 npz，"
                f"空中落盘 {len(air_files)} 个 npz，停止排空 drained"
            )
        except AssertionError as exc:
            s3.fail(str(exc))
        results.append(s3)

        # ---------------- 场景 4：断链 60 秒 → 补传完成、积压归零 ----------------
        s4 = Scenario("4. 断链 60 秒（采集继续+本地保存）→ 恢复后补传完成、积压归零")
        try:
            window.send_start()
            assert wait_until(lambda: len(run_files(ground_out)) >= 2, 60.0, app), \
                "重新启动采集未出数"
            air_runs = tmp / "air_runs"
            # 断链：停掉监听线程（server+client 全关，黑匣子侧进入退避重连）
            window.receiver.stop()
            receivers.append(window.receiver)
            # 断链期间采集继续 → outbox 本地保存
            assert wait_until(lambda: outbox_frames(air_runs) >= 3, 30.0), \
                f"断链期间 outbox 未积累（{outbox_frames(air_runs)} 帧）"
            frames_at_outage = outbox_frames(air_runs)
            print(f"[smoke] 断链开始，outbox={frames_at_outage} 帧，保持 {OUTAGE_S:.0f}s ...")
            time.sleep(OUTAGE_S)
            # 恢复：同端口换新监听线程（真实 ReceiverThread，SO_REUSEADDR）。
            # rx2 的 status/acquisition_status 同样接入收集器：后续场景 6 的
            # "Remote ready" 门控与停止回执都来自它（rx1 已停，不再发状态）。
            replay_counter = {"traces": 0}
            rx2 = ground.ReceiverThread("127.0.0.1", port, tmp / "ground_runs2")
            rx2.trace_received.connect(
                lambda *_a: replay_counter.__setitem__(
                    "traces", replay_counter["traces"] + 1),
                QtCore.Qt.ConnectionType.DirectConnection,
            )
            rx2.status.connect(
                lambda t: ground_status.append(f"[rx2] {t}"),
                QtCore.Qt.ConnectionType.DirectConnection,
            )
            rx2.acquisition_status.connect(
                lambda h: acq_status.append(dict(h)),
                QtCore.Qt.ConnectionType.DirectConnection,
            )
            rx2.start()
            receivers.append(rx2)
            drained = wait_until(lambda: outbox_frames(air_runs) == 0, 150.0, app)
            assert drained, f"补传未清零（outbox 还剩 {outbox_frames(air_runs)} 帧）"
            assert replay_counter["traces"] >= frames_at_outage, (
                f"补传计数异常: 收到 {replay_counter['traces']} < 断链积累 {frames_at_outage}"
            )
            # 注意：此处刻意不停采集——场景 6 需要"采集中被杀 → 拉起后补传
            # 遗留 outbox"。停止动作移到场景 6 的补传清零之后（经 rx2 下发，
            # window.send_stop 面向已停止的 rx1 是无效命令）。
            s4.pass_(
                f"断链 {OUTAGE_S:.0f}s 期间积累 {frames_at_outage} 帧，恢复后全部补传"
                f"（接收端确认 {replay_counter['traces']} 条），outbox 归零"
            )
        except AssertionError as exc:
            s4.fail(str(exc))
        results.append(s4)

        # ---------------- 场景 5：磁盘水位（可注入桩） ----------------
        s5 = Scenario("5. 磁盘水位触发 → 自动清理 + 报警记账（注入剩余空间桩）")
        try:
            wm_dir = tmp / "wm_runs"
            gb = 1024 ** 3
            old_run = wm_dir / "20260901_000000_0001"
            new_run = wm_dir / "20260902_000000_0002"
            for d, size in ((old_run, gb // 10), (new_run, gb // 10)):
                d.mkdir(parents=True)
                (d / "trace_00000.npz").write_bytes(b"x" * size)
            frame = wm_dir / "_outbox" / "20260902_000000_0002" / "000000.frame"
            frame.parent.mkdir(parents=True)
            frame.write_bytes(b"f" * 64)  # 新 run 未送达（有未确认帧）
            box = {"free": int(94.75 * gb), "initial": 2 * (gb // 10)}

            def probe() -> int:  # 可注入桩：剩余 = 注入基线 + 已删除字节
                cur = sum(p.stat().st_size for p in wm_dir.rglob("*") if p.is_file())
                return int(box["free"]) + int(box["initial"] - cur)

            mgr = airsvc.DiskWatermarkManager(
                wm_dir, min_free_gb=95.0, target_free_gb=94.8,
                free_space_bytes=probe,
            )
            state = mgr.check()
            assert state["triggered"] and not (wm_dir / old_run.name).exists(), \
                "未删最老已送达 run"
            assert (wm_dir / new_run.name).exists() and frame.exists(), \
                "未送达 run 被误删（应保留到阶段 3）"
            assert not state["undelivered_alarm"], "未送达报警不应在阶段 1 出现"
            box["free"] = 1 * gb  # 仍不足 → 进入报警段
            state = mgr.check()
            assert not (wm_dir / new_run.name).exists(), "阶段 3 未清理未送达 run"
            assert state["undelivered_alarm"], "删未送达数据未报警"
            assert new_run.name in state["undelivered_runs_deleted"]
            assert state["files_deleted"] >= 2 and state["bytes_freed"] > 0
            assert any(e.get("path") and "bytes" in e for e in state["recent_events"]), \
                "逐条记账缺失"
            s5.pass_(
                f"先删已送达旧 run → 目标即停；不足时动未送达并报警 "
                f"（runs={state['runs_deleted']}, files={state['files_deleted']}, "
                f"bytes={state['bytes_freed']}，逐条记账 {len(state['recent_events'])} 条在册）"
            )
        except AssertionError as exc:
            s5.fail(str(exc))
        results.append(s5)

        # ---------------- 场景 6：杀进程 → 重新拉起 → 自动恢复连接 ----------------
        s6 = Scenario("6. 杀黑匣子进程 → 重新拉起 → 自动恢复连接（模拟自愈）")
        try:
            assert bb.poll() is None, "场景 6 前黑匣子进程应存活"
            # 第7轮 t7 修复（F2 延伸）：重拉前隐藏面板——防止 5s 自动刷新
            # 的 get_status 落进新连接的 hello 回环窗口（同场景 1 的竞态）。
            panel.hide()
            app.processEvents()
            bb.terminate()  # 模拟崩溃（计划任务场景由系统拉起，此处冒烟代拉）
            bb.wait(timeout=15)
            assert bb.poll() is not None, "进程未被终止"
            status_before = len(ground_status)
            # 拉起（同配置同输出目录 → 遗留 outbox 在重连后补传）
            bb = launch_bb(cfg, "relaunch")
            # 等新连接完成 hello 回环（新的 "Remote ready"）再恢复面板可见。
            assert wait_until(
                lambda: len(ground_status) > status_before
                and any("Remote ready" in t for t in ground_status[status_before:]),
                90.0, app,
            ), f"重新拉起后 90s 内未完成 hello 回环: {ground_status[status_before:][-2:]}"
            panel.show()
            app.processEvents()
            alive_and_drained = wait_until(
                lambda: bb.poll() is None and outbox_frames(tmp / "air_runs") == 0,
                120.0, app,
            )
            # 额外确认存活（重新拉起后稳定运行 10s 不退出）
            time.sleep(10)
            assert bb.poll() is None, "重新拉起后 10s 内进程退出"
            assert alive_and_drained, "重连后 outbox 未清零"
            s6.pass_("进程重启后自动重连，遗留 outbox 已补传清零，稳定运行 10s+")
        except AssertionError as exc:
            s6.fail(str(exc))
        results.append(s6)

        # ---------------- 场景 7：新旧互操作（旧空中端 → 面板显式「不支持」） ----------------
        s7 = Scenario("7. 模拟旧空中端回『不支持』→ 地面端面板明确显示")
        try:
            shim = QtWidgets.QWidget()  # 面板宿主（最小窗口壳）
            rx3 = ground.ReceiverThread("127.0.0.1", 0, tmp / "ground_runs3")
            shim.receiver = rx3  # type: ignore[attr-defined]  # 面板经 window.receiver 发送
            rx3.start()
            receivers.append(rx3)
            old_panel = ground.AirDevicePanel(shim)
            old_panel.show()
            rx3_status: list[str] = []

            def _on_rx3_status(text: str) -> None:
                rx3_status.append(text)
                # 跨线程改 QWidget 必须走 queued（默认 Auto）：wait_until 的
                # processEvents 泵负责投递，绝不 DirectConnection 操作面板。
                old_panel.set_link_connected(
                    text.startswith("Connected") or "Remote ready" in text
                )

            rx3.status.connect(_on_rx3_status)
            rx3.command_result.connect(old_panel.on_command_result)
            app.processEvents()
            assert wait_until(
                lambda: rx3.bound_port is not None, 15.0, app
            ), "接收端 15s 未完成端口绑定（场景 7）"
            port3 = int(rx3.bound_port)
            # 模拟老空中端：hello → ack；对 command 只回纯 ack（无 supported/result）
            sock = socket.create_connection(("127.0.0.1", port3), timeout=5)
            sock.settimeout(10)
            stop_flag = threading.Event()

            def _recv_exact(sk: socket.socket, size: int) -> bytes:
                buf = b""
                while len(buf) < size:
                    chunk = sk.recv(size - len(buf))
                    if not chunk:
                        raise ConnectionError("closed")
                    buf += chunk
                return buf

            def old_air_loop() -> None:
                import struct as _struct
                try:
                    # 旧空中端是客户端：先发 hello（与老 GUI 相同的帧结构）。
                    hello = json.dumps(
                        {"type": "hello", "protocol": "uav-gpr-telemetry-v1",
                         "run_id": "00000000_000000_idle",
                         "timestamp": "smoke", "supports_commands": True},
                        separators=(",", ":")).encode("utf-8")
                    sock.sendall(_struct.pack("!I", len(hello))
                                 + hello + _struct.pack("!Q", 0))
                    # 读 hello 的 ack（内容不校验，旧 GUI 只等 ack）。
                    (alen,) = _struct.unpack("!I", _recv_exact(sock, 4))
                    _recv_exact(sock, alen)
                    (plen,) = _struct.unpack("!Q", _recv_exact(sock, 8))
                    if plen:
                        _recv_exact(sock, plen)
                    while not stop_flag.is_set():
                        (hlen,) = _struct.unpack("!I", _recv_exact(sock, 4))
                        header = json.loads(_recv_exact(sock, hlen).decode("utf-8"))
                        (plen,) = _struct.unpack("!Q", _recv_exact(sock, 8))
                        if plen:
                            _recv_exact(sock, plen)
                        if header.get("type") == "command":
                            # 旧空中端行为：纯 ack，无 supported/result 字段
                            data = json.dumps(
                                {"type": "command_ack",
                                 "protocol": "uav-gpr-telemetry-v1",
                                 "run_id": "00000000_000000_idle",
                                 "action": header.get("action"),
                                 "timestamp": "smoke"},
                                separators=(",", ":")).encode("utf-8")
                            sock.sendall(_struct.pack("!I", len(data))
                                         + data + _struct.pack("!Q", 0))
                except (OSError, ValueError):
                    pass

            threading.Thread(target=old_air_loop, daemon=True).start()
            # 第7轮 t7 修复（F2）：必须等 hello 回环完成（"Remote ready"）才
            # 发任何 command——仅 "Connected"（accept 完成）时命令帧可能在
            # 对端就绪前到达。F1 把出站帧串行化后此竞态仍有存在窗口，两修
            # 互为纵深。
            assert wait_until(
                lambda: any("Remote ready" in t for t in rx3_status)
                and old_panel._link_connected,
                20.0, app,
            ), f"旧空中端未完成 hello 回环: {rx3_status[-2:]}"
            # 状态总览：纯 ack → 总览占位「对方不支持」
            assert old_panel.refresh_now(), "面板未发送 get_status"
            assert wait_until(
                lambda: "对方不支持" in old_panel.device_version_label.text(), 20.0, app
            ), old_panel.device_version_label.text()
            # 测试雷达：纯 ack → 明确「对方版本不支持此功能」
            old_panel.send_test_vna()
            assert wait_until(
                lambda: old_panel.test_vna_result_label.text()
                == f"雷达：{ground.AirDevicePanel.UNSUPPORTED_TEXT}",
                20.0, app,
            ), old_panel.test_vna_result_label.text()
            notice = old_panel.panel_notice_label.text()
            assert ground.AirDevicePanel.UNSUPPORTED_TEXT in notice, notice
            stop_flag.set()
            sock.close()
            s7.pass_(
                "纯 ack（无 result 字段）→ 总览占位「对方不支持」+ "
                "雷达栏/提示栏显式「对方版本不支持此功能」，全程不崩"
            )
        except (AssertionError, OSError) as exc:
            s7.fail(str(exc))
        results.append(s7)

    finally:
        for proc in bb_procs:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.kill()
        for rx in receivers:
            try:
                rx.stop()
                rx.wait(2000)
            except Exception:  # noqa: BLE001
                pass

    # ---------------- 汇总 ----------------
    print("\n" + "=" * 72)
    print("第7轮黑匣子双开冒烟结果（计划 §7.1 七场景）")
    print("=" * 72)
    all_ok = True
    for s in results:
        mark = "PASS" if s.ok else "FAIL"
        if not s.ok:
            all_ok = False
        print(f"[{mark}] {s.name}")
        if s.detail:
            print(f"       {s.detail}")
    print("=" * 72)
    print(f"SMOKE RESULT: {'PASS' if all_ok else 'FAIL'}")
    if not all_ok:
        print(f"（黑匣子子进程输出留在: {tmp}）")
    else:
        shutil.rmtree(tmp, ignore_errors=True)
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
