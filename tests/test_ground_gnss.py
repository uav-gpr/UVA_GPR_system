"""Single quick GNSS check: python -m unittest test_ground_gnss -v."""
import io
import hashlib
import socket
import time
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PySide6 import QtWidgets, QtTest
import uav_gpr_ground_station as ground
from gnss_map_widget import GnssMapWidget


class GroundGnssCheck(unittest.TestCase):
    @classmethod
    def wait_for(cls, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cls.app.processEvents()
            if predicate():
                return True
            time.sleep(0.01)
        return False

    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls.tmp = tempfile.TemporaryDirectory()
        cls.window = ground.GroundStationWindow(
            "127.0.0.1", 0, Path(cls.tmp.name))
        cls.window.show()
        assert cls.wait_for(lambda: "等待连接" in cls.window.connection_status_label.text()
                            and cls.window.receiver._bound_port), "receiver failed to start"
        cls.app.processEvents()

    @classmethod
    def tearDownClass(cls):
        cls.window.close()
        cls.app.processEvents()
        cls.tmp.cleanup()

    def test_bad_optional_npz_gnss_does_not_reject_radar(self):
        payload = io.BytesIO()
        np.savez(payload, frequency_hz=[1., 2.], time_ns=[0., 1.],
                 trace=[1., 2.], gnss=np.array({"bad": "object array"}, dtype=object))
        decoded = ground.decode_ground_trace_payload({"trace_index": 1}, payload.getvalue(), "test.npz")
        self.assertIsNotNone(decoded.legacy_trace)
        payload = io.BytesIO()
        np.savez(payload, gnss="{}")
        with np.load(io.BytesIO(payload.getvalue())) as data:
            self.assertEqual(ground.gnss_from_sources(data, {"gnss": {"fix_quality": 1}})["fix_quality"], 1)

    def test_test_ack_reaches_main_gnss_widget(self):
        w = self.window
        w._set_remote_connected(True)
        w._on_command_result({"action": "test_gnss", "result": {
            "ok": True, "fix": True, "lat": 31.2, "lon": 121.5, "satellites": 8}})
        self.assertIn("31.20000000", w.gnss_map.status.text())

    def test_gga_variants_states_and_nonblocking_loss(self):
        widget = self.window.gnss_map
        widget.set_link_connected(False)
        self.assertIn("数据链未连接", widget.status.text())
        widget.set_link_connected(True)
        self.assertIn("尚未收到", widget.status.text())
        for talker in ("GP", "GN", "BD"):
            widget.update_data({"raw": f"${talker}GGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"})
            self.assertIn("单点定位成功", widget.status.text())
            self.assertIn("48.11730000", widget.status.text())
            self.assertIn("11.51666667", widget.status.text())
            self.assertIn("545.4", widget.status.text())
        widget.update_data({"raw": "$GNGGA,123520,,,,,0,00,99.9,,M,,M,,"})
        self.assertIn("卫星数为 0", widget.status.text())
        widget.update_data({"raw": "$BDGGA,123521,,,,,0,04,99.9,,M,,M,,"})
        self.assertIn("尚未定位", widget.status.text())
        widget.update_data({"raw": "$GPGGA,123522,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"})
        widget._last_data_at -= widget.DATA_TIMEOUT_S + 1
        # Same cached UTC must not reset the timeout, even while traces arrive.
        widget.update_data({"utc": "123522", "latitude": 48.1173, "longitude": 11.5167,
                            "fix_quality": 1, "satellites": 8})
        QtTest.QTest.qWait(1100)
        self.assertIn("数据中断", widget.status.text())
        widget.update_data({"raw": "$GNGGA,123523,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"})
        self.assertIn("单点定位成功", widget.status.text())
        widget.update_data({"raw": "$GNGGA,broken*00"})
        self.assertIn("异常", widget.status.text())

    def test_extra_talkers_and_sky_end_detail(self):
        """GGA talker 泛化（含 GL/GAGGA）与天空端分类明细直显。"""
        widget = self.window.gnss_map
        widget.set_link_connected(True)
        widget.update_data({"raw": "$GLGGA,123530,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"})
        self.assertIn("单点定位成功", widget.status.text())
        self.assertIn("48.11730000", widget.status.text())
        widget.update_data({"serial_open": True, "state": "raw_not_nmea",
                            "detail": "收到 48 字节 / 0 行，没有一条 NMEA 句子"})
        self.assertIn("天空端", widget.status.text())
        self.assertIn("没有一条 NMEA 句子", widget.status.text())

    def test_gnss_render_failure_still_commits_radar(self):
        w = self.window
        w._set_remote_connected(True)
        decoded = ground.DecodedGroundTrace(
            trace_index=2, frequency_hz=np.array([1., 2.]), time_ns=np.array([0., 1.]),
            trace_s11=None, trace_s22=None, legacy_trace=np.array([1., 2.]), channels=None,
            processing_signature=(True, None), dewow_state="legacy",
            gnss={"latitude": 31.2, "longitude": 121.5, "fix_quality": 1})
        before = w.display_fifo.statistics()["total_committed"]
        # A map failure used to reach the common trace failure handler.
        with patch.object(GnssMapWidget, "update_fix", side_effect=RuntimeError("map error")):
            w.display_fifo.enqueue(decoded)
            w._process_one_display_item()
        self.assertEqual(w.display_fifo.statistics()["total_committed"], before + 1)
        self.assertIn("异常", w.gnss_map.status.text())

    def test_tcp_ack_and_bad_gnss_do_not_break_receiver(self):
        w = self.window
        with socket.create_connection(("127.0.0.1", w.receiver._bound_port), timeout=2) as sock:
            ground.send_frame(sock, {"protocol": ground.PROTOCOL, "type": "hello", "run_id": "gnss_check"})
            ground.recv_frame(sock)
            ground.send_frame(sock, {"protocol": ground.PROTOCOL, "type": "command_ack",
                "action": "test_gnss", "result": {"ok": True, "raw":
                    "$GNGGA,123524,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"}})
            self.assertTrue(self.wait_for(lambda: "48.11730000" in w.gnss_map.status.text()))
            self.assertIn("48.11730000", w.gnss_map.status.text())
            payload = io.BytesIO()
            np.savez(payload, frequency_hz=[1., 2.], time_ns=[0., 1.], trace=[1., 2.],
                     gnss=np.array({"bad": "object array"}, dtype=object))
            blob = payload.getvalue()
            before = w.display_fifo.statistics()["total_committed"]
            ground.send_frame(sock, {"protocol": ground.PROTOCOL, "type": "trace", "run_id": "gnss_check",
                "trace_index": 1, "sha256": hashlib.sha256(blob).hexdigest()}, blob)
            ack, _ = ground.recv_frame(sock)
            self.assertEqual(ack["type"], "ack_trace")
            self.assertTrue(self.wait_for(lambda: w.display_fifo.statistics()["total_committed"] > before))
            self.assertEqual(w.display_fifo.statistics()["total_committed"], before + 1)
            self.assertTrue(w.receiver.isRunning())


if __name__ == "__main__":
    unittest.main()
