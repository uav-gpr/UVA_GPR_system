"""Quick GNSS-only regression and actual air-to-ground socket/UI check."""
import os
os.environ["QT_QPA_PLATFORM"] = "windows"
import io
import queue
import socket
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import uav_gpr_air_service as service_module
import uav_gpr_qt as air
from air_gnss import AirGnssMonitor
from PySide6 import QtWidgets

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "地面站便携包"))
import uav_gpr_ground_station as ground


def gga(talker="GN", utc="010203", quality=1, sats=8):
    coord = "4807.038,N,01131.000,E" if quality else ",,,"
    body = f"{talker}GGA,{utc},{coord},{quality},{sats:02d},0.9,545.4,M,46.9,M,,"
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f"${body}*{checksum:02X}\r\n"


class SerialRig:
    def __init__(self):
        self.available = False
        self.port = "COM7"
        self.lines = queue.Queue()
        self.attempts = []
        self.commands = []
        self.connections = []

    def ports(self):
        if not self.available:
            return []
        return [SimpleNamespace(device=self.port, description="SinoGNSS M100",
                                manufacturer="ComNav", vid=1234, pid=5678, serial_number="m100-test")]

    def open(self, port, baud, **kwargs):
        self.attempts.append((port, baud, time.monotonic()))
        if not self.available:
            raise OSError("M100 not powered")
        rig = self
        class Serial:
            is_open = True
            def write(self, data):
                rig.commands.append(data)
                return len(data)
            def readline(self, size=1024):
                if not rig.available:
                    raise OSError("USB unplugged")
                try:
                    return rig.lines.get(timeout=0.01)
                except queue.Empty:
                    return b""
            def close(self):
                self.is_open = False
        instance = Serial()
        self.connections.append(instance)
        return instance


class AirGnssQuickCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def wait_for(self, predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.app.processEvents()
            if predicate():
                return True
            time.sleep(0.01)
        return False

    def test_parse_no_fix_spp_and_reader_commands(self):
        for talker in ("GP", "GN", "BD"):
            fix = air.parse_gga(gga(talker))
            self.assertTrue(air.is_valid_gnss_fix(fix))
            self.assertAlmostEqual(fix.latitude, 48.1173)
            self.assertAlmostEqual(fix.longitude, 11.5166666667)
            self.assertEqual((fix.fix_quality, fix.satellites, fix.altitude_m), (1, 8, 545.4))
            no_fix = air.parse_gga(gga(talker, quality=0, sats=4))
            self.assertEqual(no_fix.satellites, 4)
            self.assertIsNone(no_fix.latitude)
        rig = SerialRig()
        rig.available = True
        rig.lines.put(b"$GNGGA,bad*00\r\n")
        raw = gga(quality=0, sats=0).encode()
        rig.lines.put(raw[:10])
        rig.lines.put(raw[10:])
        with patch.object(air.serial, "Serial", side_effect=rig.open):
            reader = air.GnssReader("COM7")
            try:
                fix = reader.read_gga(0.2)
                self.assertTrue(fix.nmea_received)
                self.assertEqual(fix.satellites, 0)
                self.assertEqual(rig.commands, [b"log gpgga ontime 1\r\n", b"log gprmc ontime 1\r\n"])
                reader.read_gga(0.02)
                self.assertEqual(len(rig.attempts), 1)
            finally:
                reader.close()

    def test_provider_error_does_not_touch_sender_queue(self):
        sender = air.RemoteSender("127.0.0.1", 1, "test", lambda text: None)
        sender.gnss_snapshot = lambda: (_ for _ in ()).throw(ValueError("bad GNSS"))
        sender._send_gnss_snapshot(None)
        self.assertEqual(sender.outstanding_count(), 0)
        self.assertTrue(sender.queue.empty())

    def test_start_reconnect_no_fix_and_ground_compatibility(self):
        rig = SerialRig()
        monitor = AirGnssMonitor("COM7", port_scanner=rig.ports,
                                initial_retry_s=0.03, max_retry_s=0.06, data_timeout_s=0.6)
        logs = []
        with tempfile.TemporaryDirectory() as tmp, patch.object(air.serial, "Serial", side_effect=rig.open):
            window = ground.GroundStationWindow("127.0.0.1", 0, Path(tmp) / "ground")
            service = None
            try:
                self.assertTrue(self.wait_for(lambda: window.receiver._bound_port is not None))
                config = service_module.AirConfig(ground_host="127.0.0.1",
                    ground_port=window.receiver._bound_port, output_dir="air", device_source="simulated")
                service = service_module.AirBlackboxService(config, program_dir=Path(tmp), log=logs.append,
                                                            reconnect_initial_s=0.03, reconnect_max_s=0.06)
                service.gnss = monitor
                service.sender.gnss_snapshot = monitor.snapshot
                service.start()
                self.assertTrue(self.wait_for(lambda: len(rig.attempts) >= 2 and window._remote_connected))
                self.assertTrue(monitor._thread.is_alive())
                self.assertFalse(service._is_acquiring())  # GNSS starts even while radar is idle.
                self.assertGreaterEqual(rig.attempts[1][2] - rig.attempts[0][2], 0.02)
                started = time.monotonic()
                service._cmd_test_gnss({})
                self.assertLess(time.monotonic() - started, 0.1)

                rig.available = True
                self.assertTrue(self.wait_for(lambda: bool(rig.connections)))
                rig.lines.put(gga(quality=0, sats=0).encode())
                self.assertTrue(self.wait_for(lambda: monitor.snapshot()["state"] == "no_satellites"))
                service.sender._last_gnss_sent_at = float("-inf")
                self.assertTrue(self.wait_for(lambda: "卫星数为 0" in window.gnss_map.status.text()))
                rig.lines.put(gga(utc="010204", quality=0, sats=4).encode())
                self.assertTrue(self.wait_for(lambda: monitor.snapshot()["satellites"] == 4))
                service.sender._last_gnss_sent_at = float("-inf")
                self.assertTrue(self.wait_for(lambda: "尚未定位" in window.gnss_map.status.text()))
                rig.lines.put(gga(utc="010205").encode())
                self.assertTrue(self.wait_for(lambda: monitor.snapshot()["fix"]))
                service.sender._last_gnss_sent_at = float("-inf")
                self.assertTrue(self.wait_for(lambda: "48.11730000" in window.gnss_map.status.text()))

                # Real simulated radar engine continues through missing GNSS / serial errors.
                service._interval_s = 0.03
                self.assertTrue(service.start_acquisition())
                self.assertTrue(self.wait_for(lambda: window.display_fifo.statistics()["total_committed"] > 0))
                before = window.display_fifo.statistics()["total_committed"]
                rig.available = False
                self.assertTrue(self.wait_for(lambda: not monitor.snapshot()["serial_open"]))
                self.assertTrue(self.wait_for(lambda: window.display_fifo.statistics()["total_committed"] > before))
                self.assertTrue(service._is_acquiring())

                # Same USB identity, changed Windows COM number; reconnect without a command.
                rig.port = "COM9"
                rig.available = True
                rig.lines.put(gga("BD", "010206").encode())
                self.assertTrue(self.wait_for(lambda: monitor.snapshot()["fix"] and monitor.snapshot()["port"] == "COM9"))
                self.assertTrue(service._is_acquiring())
                attempts = len(rig.attempts)
                self.assertTrue(self.wait_for(lambda: len(rig.attempts) > attempts))  # silence -> reopen
                rig.lines.put(gga("GP", "010207").encode())
                self.assertTrue(self.wait_for(lambda: monitor.snapshot()["fix"]))
                self.assertTrue(service._is_acquiring())
                self.assertTrue(all(not s.is_open for s in rig.connections[:-1]))
                self.assertFalse(any("采集失败" in line for line in logs), logs[-10:])
            finally:
                if service is not None:
                    service.shutdown()
                else:
                    monitor.stop()
                self.wait_for(lambda: window.display_fifo.statistics()["depth"] == 0)
                window.close()
                self.app.processEvents()

    def test_auto_discovery_and_ambiguous_ports(self):
        rig = SerialRig()
        monitor = AirGnssMonitor(port_scanner=rig.ports)
        self.assertIsNone(monitor._find_port(""))
        rig.available = True
        self.assertEqual(monitor._find_port(""), "COM7")
        rig.port = "COM9"
        self.assertEqual(monitor._find_port(""), "COM9")

    def test_teacher_poll_cr_only_framing_and_no_nmea_diagnostics(self):
        """老师版本 log gpgga 轮询 + CR 行尾容错 + "有字节非 NMEA"分类。"""
        rig = SerialRig()
        rig.available = True
        with patch.object(air.serial, "Serial", side_effect=rig.open):
            reader = air.GnssReader("COM7")
            logs: list[str] = []
            reader.log = logs.append
            reader.open()
            self.assertTrue(any("[GNSS] serial parameters" in line for line in logs), logs)
            # 一条 NMEA 都没收到过：必须补发老师版本原命令，并附只读链路探针。
            reader.read_gga(0.02)
            self.assertIn(b"log gpgga\r\n", rig.commands)
            self.assertIn(b"log version\r\n", rig.commands)
            self.assertIn("收不到任何字节", reader.diagnostics())
            # 串口有字节但不是 NMEA：明确分类，不伪装成定位数据。
            rig.lines.put(bytes([0xFF, 0xFD, 0xFB, 0x7F]))
            reader._last_sentence_at = time.monotonic() - 5.0
            reader.read_gga(0.05)
            self.assertIn("没有一条 NMEA 句子", reader.diagnostics())
            self.assertIn("\\xff", reader.diagnostics())
            # 字节统计 + 判定：高位字节 → 判为噪声/电平不对。
            self.assertIn(">=0x80", reader.diagnostics())
            self.assertIn("噪声", reader.diagnostics())
            self.assertNotIn(b"unlogall", [c for c in rig.commands[:2]])  # 只在恢复路径清理一次
            self.assertIn(b"unlogall\r\n", rig.commands)
            # 只发 CR（无 LF）的 GGA 也必须成行（旧实现只认 LF）。
            rig.lines.put(gga().encode().replace(b"\r\n", b"\r"))
            fix = reader.read_gga(0.3)
            self.assertTrue(fix.nmea_received)
            self.assertAlmostEqual(fix.latitude, 48.1173)
            self.assertAlmostEqual(fix.longitude, 11.5166666667)
            reader.close()

    def test_port_probe_and_baud_rescue_after_two_cycles(self):
        """两轮物理口轮询（无端口/COM1/COM3）后按手册降速救援，再切回一次。"""
        rig = SerialRig()
        rig.available = True
        readers = []

        def open_and_track(port, baud, **kwargs):
            readers.append((port, baud))
            return rig.open(port, baud, **kwargs)

        with patch.object(air.serial, "Serial", side_effect=open_and_track):
            reader = air.GnssReader("COM7")
            logs: list[str] = []
            reader.log = logs.append
            reader.open()
            now = time.monotonic()
            # 7 次阶段切换 = 跑满两轮 (0→1→2) + 触发救援后的新一轮起点。
            for _ in range(7):
                reader._last_sentence_at = now - 100.0
                reader._last_poll_at = now - 2.0
                reader._probe_phase_started_at = now - 20.0
                reader._poll_output(now)
                now += 1.0
            self.assertIn(b"log com1 gpgga ontime 1\r\n", rig.commands)
            self.assertIn(b"log com3 gpgga ontime 1\r\n", rig.commands)
            self.assertIn(b"log loglista once\r\n", rig.commands)
            self.assertIn(b"com com1 9600\r\n", rig.commands)
            self.assertIn(b"com com3 9600\r\n", rig.commands)
            self.assertIn(9600, [baud for _, baud in readers])   # 救援后按 9600 重开
            self.assertTrue(any("波特率救援" in line for line in logs), logs)
            reader.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
