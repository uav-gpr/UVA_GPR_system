"""Comprehensive specialist unit and integration tests for the GNSS fix track."""
from __future__ import annotations

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

from dataclasses import asdict, fields
import io
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from PySide6 import QtWidgets

import air_core as air
from air_gnss import AirGnssMonitor
from gnss_diagnostics import (
    DIAG_LOG_ENV,
    FIX_POLICY_DGPS_OR_BETTER,
    FIX_POLICY_ENV,
    FIX_POLICY_OFF,
    FIX_POLICY_RTK_ANY,
    FIX_POLICY_STRICT_RTK_FIXED,
    SOLUTION_DGPS,
    SOLUTION_NO_FIX,
    SOLUTION_RTK_FIXED,
    SOLUTION_RTK_FLOAT,
    SOLUTION_SPP,
    GnssLogWriter,
    fix_policy,
    solution_type,
)
import ground_gnss
from librevna.librevna_continuous import PersistentLibreVnaBackend
from librevna.simulated_transport import SimulatedLibreVnaTransport


def _nmea_sentence(body: str, ending: str = "\r\n") -> str:
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    return f"${body}*{checksum:02X}{ending}"


def make_gga(
    *,
    talker: str = "GN",
    utc: str = "094146.00",
    lat_dm: str = "3113.8240",
    ns: str = "N",
    lon_dm: str = "12128.4220",
    ew: str = "E",
    quality: int = 1,
    sats: int = 8,
    hdop: str = "0.9",
    alt: str = "54.2",
    ending: str = "\r\n",
) -> str:
    if quality <= 0:
        coord = ",,,"
        alt_part = ",M,,M"
    else:
        coord = f"{lat_dm},{ns},{lon_dm},{ew}"
        alt_part = f"{alt},M,8.1,M"
    body = f"{talker}GGA,{utc},{coord},{quality},{sats:02d},{hdop},{alt_part},,"
    return _nmea_sentence(body, ending=ending)


def make_rmc(
    *,
    talker: str = "GN",
    utc: str = "094146.00",
    status: str = "A",
    lat_dm: str = "3113.8240",
    ns: str = "N",
    lon_dm: str = "12128.4220",
    ew: str = "E",
    date: str = "061126",
    ending: str = "\r\n",
) -> str:
    coord = f"{lat_dm},{ns},{lon_dm},{ew}" if status == "A" else ",,,"
    mode = "A" if status == "A" else "N"
    body = f"{talker}RMC,{utc},{status},{coord},0.05,180.0,{date},,,{mode}"
    return _nmea_sentence(body, ending=ending)


class FakeChunkSerial:
    """Controllable serial double supporting queued chunks and instant stream modes."""

    def __init__(self, mode: str = "queue", stream_chunk: bytes = b"") -> None:
        self.is_open = True
        self.mode = mode
        self.stream_chunk = stream_chunk
        self.chunks: queue.Queue[bytes] = queue.Queue()
        self.writes: list[bytes] = []
        self.readline_calls = 0

    def write(self, payload: bytes) -> int:
        self.writes.append(bytes(payload))
        return len(payload)

    def readline(self, size: int = 1024) -> bytes:
        if not self.is_open:
            raise OSError("serial port closed")
        self.readline_calls += 1
        if self.mode == "instant_empty":
            return b""
        if self.mode == "instant_stream":
            return self.stream_chunk[:size]
        try:
            return self.chunks.get_nowait()
        except queue.Empty:
            return b""

    def close(self) -> None:
        self.is_open = False


class GnssGilStarvationTests(unittest.TestCase):
    """Fix 1: Prevent GIL busy-spin on instant empty/garbage streams without slowing FIFO drain."""

    def test_instant_empty_and_garbage_streams_cap_loop_rate(self) -> None:
        bad_gga = b"$GNGGA,094146.00,,,,,0,06,,,,,,,*00\r\n"
        cases = [
            ("instant_empty", FakeChunkSerial(mode="instant_empty")),
            ("garbage_no_crlf", FakeChunkSerial(mode="instant_stream", stream_chunk=b"\xff\xfd\xfb\x80" * 16)),
            ("garbage_crlf", FakeChunkSerial(mode="instant_stream", stream_chunk=b"\xff\xfd\xfb\r\n")),
            ("rejected_nmea", FakeChunkSerial(mode="instant_stream", stream_chunk=bad_gga)),
        ]
        for label, fake_ser in cases:
            with self.subTest(stream=label):
                with patch.object(air.serial, "Serial", return_value=fake_ser):
                    reader = air.GnssReader("COM7", 9600)
                    t0 = time.monotonic()
                    fix = reader.read_gga(timeout_s=0.12)
                    elapsed = time.monotonic() - t0
                    reader.close()
                self.assertFalse(fix.nmea_received)
                self.assertGreaterEqual(elapsed, 0.10)
                # At READ_LOOP_MIN_INTERVAL_S=0.01s, 0.12s window does ~12 iterations (never thousands).
                self.assertLessEqual(
                    fake_ser.readline_calls,
                    30,
                    f"{label} spun {fake_ser.readline_calls} times in {elapsed:.3f}s",
                )

    def test_continuous_radar_sweep_rate_unstarved_under_half_disconnected_gnss(self) -> None:
        fake_port = SimpleNamespace(
            device="COM7",
            serial_number="M100-GIL",
            vid=1,
            pid=2,
            description="SinoGNSS M100",
            manufacturer="ComNav",
            product="M100",
        )
        fake_ser = FakeChunkSerial(mode="instant_stream", stream_chunk=b"\xff\xfd\xfb\r\n")

        def make_reader(port: str, baud: int) -> air.GnssReader:
            return air.GnssReader(port, baud)

        with tempfile.TemporaryDirectory() as tmp, patch.object(air.serial, "Serial", return_value=fake_ser):
            monitor = AirGnssMonitor(
                "COM7",
                9600,
                reader_factory=make_reader,
                port_scanner=lambda: [fake_port],
                data_timeout_s=30.0,
            )
            backend = PersistentLibreVnaBackend(transport=SimulatedLibreVnaTransport(seed=11, pace=True))
            cfg = air.SweepConfig(
                start_mhz=400.0,
                stop_mhz=4000.0,
                points=201,
                ifbw_hz=10000,
                power_dbm=-20.0,
                dwell_us=0,
            )
            worker = air.ContinuousAcquisitionWorker(
                cfg=cfg,
                interval_s=0.0,
                out_dir=Path(tmp),
                count=0,
                gnss_port="",
                baud=9600,
                calibration_path="",
                session_calibration=None,
                session_calibration_p2=None,
                through_measurement=None,
                background_frequency_hz=None,
                background=None,
                autosave=False,
                backend=backend,
                gnss_cache=monitor,
                channels=("S11", "S22"),
            )
            try:
                monitor.start()
                worker.start()
                time.sleep(0.6)
                c0 = backend.statistics.completed_sweeps
                t0 = time.perf_counter()
                time.sleep(1.5)
                c1 = backend.statistics.completed_sweeps
                t1 = time.perf_counter()
                sweep_rate_hz = (c1 - c0) / (t1 - t0)
                # Before fix: dropped to ~0.7 sweeps/s; with fix: stays near ~9 sweeps/s.
                self.assertGreater(sweep_rate_hz, 5.5, f"Radar starved by GNSS spin: {sweep_rate_hz:.2f} Hz")
            finally:
                worker.request_stop()
                worker.wait(5000)
                backend.close()
                monitor.stop()

    def test_draining_buffered_fifo_lines_adds_zero_sleep(self) -> None:
        fake_ser = FakeChunkSerial()
        burst = "".join(
            make_gga(utc=f"0941{sec:02d}.00", quality=1, sats=8) for sec in range(10, 20)
        ).encode("ascii")
        fake_ser.chunks.put(burst)

        with patch.object(air.serial, "Serial", return_value=fake_ser):
            reader = air.GnssReader("COM7", 9600)
            reader.open()
            try:
                with patch.object(air.time, "sleep") as sleep_mock:
                    t0 = time.perf_counter()
                    utcs = [reader.read_gga(timeout_s=0.5).utc for _ in range(10)]
                    elapsed = time.perf_counter() - t0
                self.assertEqual(sleep_mock.call_count, 0)
                self.assertLess(elapsed, 0.08)
                self.assertEqual(utcs, [f"0941{sec:02d}.00" for sec in range(10, 20)])
                self.assertEqual(fake_ser.readline_calls, 1)
            finally:
                reader.close()


class GnssMultiLineFifoTests(unittest.TestCase):
    """Fix 2: Bounded 256-line FIFO preserves multi-line chunks and split partials in order."""

    def test_multi_line_single_chunk_preserves_quality_0_1_4_order(self) -> None:
        fake_ser = FakeChunkSerial()
        chunk = (
            make_gga(utc="094146.00", quality=0, sats=5, ending="\r")
            + make_gga(utc="094147.00", quality=1, sats=8, ending="\n")
            + make_gga(utc="094148.00", quality=4, sats=14, ending="\r\n")
        ).encode("ascii")
        fake_ser.chunks.put(chunk)

        with patch.object(air.serial, "Serial", return_value=fake_ser):
            reader = air.GnssReader("COM7", 9600)
            try:
                fixes = [reader.read_gga(timeout_s=0.2) for _ in range(3)]
            finally:
                reader.close()

        self.assertEqual([f.fix_quality for f in fixes], [0, 1, 4])
        self.assertEqual([f.utc for f in fixes], ["094146.00", "094147.00", "094148.00"])
        self.assertIsNone(fixes[0].latitude)
        self.assertIsNotNone(fixes[1].latitude)
        self.assertIsNotNone(fixes[2].latitude)

    def test_partial_sentence_across_chunks_and_trailing_complete_lines(self) -> None:
        fake_ser = FakeChunkSerial()
        s1 = make_gga(utc="100001.00", quality=0, sats=4)
        s2 = make_gga(utc="100002.00", quality=1, sats=7)
        s3 = make_gga(utc="100003.00", quality=4, sats=12)
        s4 = make_gga(utc="100004.00", quality=5, sats=11)
        combined = (s1 + s2 + s3 + s4).encode("ascii")
        split_at = len((s1 + s2).encode("ascii")) + 17  # mid-way through s3
        fake_ser.chunks.put(combined[:split_at])
        fake_ser.chunks.put(combined[split_at:])

        with patch.object(air.serial, "Serial", return_value=fake_ser):
            reader = air.GnssReader("COM7", 9600)
            try:
                fixes = [reader.read_gga(timeout_s=0.2) for _ in range(4)]
            finally:
                reader.close()

        self.assertEqual([f.fix_quality for f in fixes], [0, 1, 4, 5])
        self.assertEqual([f.utc for f in fixes], ["100001.00", "100002.00", "100003.00", "100004.00"])

    def test_fifo_bounded_at_256_lines_with_overflow_counter(self) -> None:
        self.assertEqual(air.GnssReader.LINE_FIFO_CAPACITY, 256)
        fake_ser = FakeChunkSerial()
        total_lines = 270
        sentences = []
        for idx in range(total_lines):
            mm, ss = divmod(idx, 60)
            sentences.append(make_gga(utc=f"11{mm:02d}{ss:02d}.00", quality=1, sats=9))
        fake_ser.chunks.put("".join(sentences).encode("ascii"))

        with patch.object(air.serial, "Serial", return_value=fake_ser):
            reader = air.GnssReader("COM7", 9600)
            try:
                first = reader.read_gga(timeout_s=0.2)
                self.assertEqual(reader.fifo_overflows, total_lines - 256)
                self.assertEqual(reader.line_fifo_overflows, total_lines - 256)
                # Remaining 255 in FIFO after popping the oldest retained (idx=14).
                self.assertEqual(len(reader._line_fifo), 255)
                rest = [reader.read_gga(timeout_s=0.2) for _ in range(255)]
            finally:
                reader.close()

        all_utcs = [first.utc] + [f.utc for f in rest]
        expected_utcs = [
            f"11{idx // 60:02d}{idx % 60:02d}.00" for idx in range(total_lines - 256, total_lines)
        ]
        self.assertEqual(all_utcs, expected_utcs)


class GnssGgaRmcAuditTests(unittest.TestCase):
    """Fix 3: GGA/RMC parsing, auditing, deduplication, conflict recording, stale RMC, midnight."""

    def test_both_gga_and_rmc_parsed_and_audited_with_same_epoch_dedup(self) -> None:
        fake_ser = FakeChunkSerial()
        payload = (
            make_gga(utc="094146.00", quality=4, sats=12)
            + make_rmc(utc="094146.00", status="A")
            + "$GNRMC,094146.00,A,bad*00\r\n"
            + make_gga(talker="GP", utc="094147.00", quality=1, sats=8)
            + make_gga(talker="GN", utc="094147.00", quality=1, sats=8)
        ).encode("ascii")
        fake_ser.chunks.put(payload)

        with patch.object(air.serial, "Serial", return_value=fake_ser):
            reader = air.GnssReader("COM7", 9600)
            try:
                f1 = reader.read_gga(timeout_s=0.2)
                f2 = reader.read_gga(timeout_s=0.2)
                f3 = reader.read_gga(timeout_s=0.03)
            finally:
                reader.close()

        self.assertEqual((f1.utc, f1.fix_quality), ("094146.00", 4))
        self.assertEqual((f2.utc, f2.fix_quality), ("094147.00", 1))
        self.assertFalse(f3.nmea_received)
        self.assertEqual(reader.gga_count, 3)
        self.assertEqual(reader.rmc_count, 1)
        self.assertEqual(reader.parse_errors, 1)
        self.assertEqual(reader.duplicate_epochs_skipped, 2)
        reasons = [(r["kind"], r["utc"], r["selected"], r["reason"]) for r in reader.sentence_audit]
        self.assertEqual(
            reasons,
            [
                ("GGA", "094146.00", True, "gga_primary"),
                ("RMC", "094146.00", False, "same_epoch_gga_preferred"),
                ("RMC", "", False, "parse_error"),
                ("GGA", "094147.00", True, "gga_primary"),
                ("GGA", "094147.00", False, "duplicate_epoch_gga"),
            ],
        )

    def test_same_epoch_conflict_recorded_and_gga_priority_enforced(self) -> None:
        fake_ser = FakeChunkSerial()
        payload = (
            # Epoch 1: GGA valid (RTK fixed) vs RMC shifted coordinates -> conflict, GGA kept
            make_gga(utc="120001.00", lat_dm="3113.8240", lon_dm="12128.4220", quality=4, sats=13)
            + make_rmc(utc="120001.00", status="A", lat_dm="3120.0000", lon_dm="12135.0000")
            # Epoch 2: GGA no-fix (quality=0) vs same-epoch RMC valid ('A') -> conflict, GGA priority kept
            + make_gga(utc="120002.00", quality=0, sats=6)
            + make_rmc(utc="120002.00", status="A")
        ).encode("ascii")
        fake_ser.chunks.put(payload)

        with patch.object(air.serial, "Serial", return_value=fake_ser):
            reader = air.GnssReader("COM7", 9600)
            try:
                fix1 = reader.read_gga(timeout_s=0.2)
                fix2 = reader.read_gga(timeout_s=0.2)
                tail = reader.read_gga(timeout_s=0.03)
            finally:
                reader.close()

        self.assertEqual(fix1.fix_quality, 4)
        self.assertAlmostEqual(fix1.latitude, 31.0 + 13.8240 / 60.0)
        self.assertEqual(fix2.fix_quality, 0)
        self.assertEqual(fix2.satellites, 6)
        self.assertFalse(tail.nmea_received)
        self.assertEqual(reader.epoch_conflicts, 2)
        conflict_entries = [r for r in reader.sentence_audit if r["conflict"]]
        self.assertEqual(len(conflict_entries), 2)
        self.assertTrue(all(r["reason"] == "conflict_gga_preferred" for r in conflict_entries))

    def test_stale_rmc_never_overwrites_newer_gga_even_after_silence(self) -> None:
        fake_ser = FakeChunkSerial()
        fake_ser.chunks.put(make_gga(utc="140010.00", quality=1, sats=9).encode("ascii"))

        with patch.object(air.serial, "Serial", return_value=fake_ser):
            reader = air.GnssReader("COM7", 9600)
            try:
                first = reader.read_gga(timeout_s=0.2)
                self.assertEqual(first.utc, "140010.00")
                # Simulate >3s GGA silence so RMC fallback window is open, then send older RMC (140008.00).
                reader._last_gga_at = time.monotonic() - 10.0
                reader._last_valid_gga_at = time.monotonic() - 10.0
                fake_ser.chunks.put(
                    make_rmc(utc="140008.00", status="A", lat_dm="3000.0000", lon_dm="12000.0000").encode("ascii")
                )
                stale_attempt = reader.read_gga(timeout_s=0.04)
                self.assertFalse(stale_attempt.nmea_received)
                self.assertEqual(reader.stale_rmc_skipped, 1)

                # A genuinely newer RMC (140015.00) during GGA silence IS accepted as fallback.
                fake_ser.chunks.put(
                    make_rmc(utc="140015.00", status="A", lat_dm="3115.0000", lon_dm="12130.0000").encode("ascii")
                )
                fallback = reader.read_gga(timeout_s=0.2)
                self.assertTrue(fallback.nmea_received)
                self.assertEqual(fallback.utc, "140015.00")
                self.assertEqual(fallback.fix_quality, 1)
            finally:
                reader.close()

    def test_midnight_rollover_gga_and_rmc_both_directions(self) -> None:
        fake_ser = FakeChunkSerial()
        # 1) Forward rollover: GGA 23:59:59 -> GGA 00:00:00 -> GGA 00:00:01 (q=0) -> valid RMC 00:00:02
        #    followed by delayed stale pre-midnight RMC 23:59:58!
        fake_ser.chunks.put(
            (
                make_gga(utc="235959.00", quality=1, sats=8)
                + make_gga(utc="000000.00", quality=4, sats=12)
            ).encode("ascii")
        )
        with patch.object(air.serial, "Serial", return_value=fake_ser):
            reader = air.GnssReader("COM7", 9600)
            try:
                f_pre = reader.read_gga(timeout_s=0.2)
                f_mid = reader.read_gga(timeout_s=0.2)
                self.assertEqual(f_pre.utc, "235959.00")
                self.assertEqual((f_mid.utc, f_mid.fix_quality), ("000000.00", 4))

                # Simulate GGA dropout right before midnight on a second reader to test RMC rollover + stale guard.
                reader.reset_diagnostics()
                fake_ser.chunks.put(make_gga(utc="235959.00", quality=0, sats=5).encode("ascii"))
                f_q0 = reader.read_gga(timeout_s=0.2)
                self.assertEqual((f_q0.utc, f_q0.fix_quality), ("235959.00", 0))

                # Newer RMC across midnight (00:00:01.00) must be treated as +2s newer, not stale!
                fake_ser.chunks.put(make_rmc(utc="000001.00", status="A").encode("ascii"))
                f_rmc_new = reader.read_gga(timeout_s=0.2)
                self.assertTrue(f_rmc_new.nmea_received)
                self.assertEqual(f_rmc_new.utc, "000001.00")

                # Delayed pre-midnight RMC (23:59:58.00) arriving after 00:00:01.00 must be rejected as stale!
                reader._last_gga_at = time.monotonic() - 10.0
                reader._last_valid_gga_at = time.monotonic() - 10.0
                fake_ser.chunks.put(make_rmc(utc="235958.00", status="A").encode("ascii"))
                f_rmc_stale = reader.read_gga(timeout_s=0.04)
                self.assertFalse(f_rmc_stale.nmea_received)
                self.assertEqual(reader.stale_rmc_skipped, 1)
            finally:
                reader.close()

    def test_continuous_quality_zero_gga_preserves_satellites_and_allows_newer_valid_rmc(self) -> None:
        fake_ser = FakeChunkSerial()
        stream = "".join(
            make_gga(utc=f"0941{46 + i:02d}.00", quality=0, sats=6)
            + make_rmc(utc=f"0941{46 + i:02d}.00", status="V")
            for i in range(4)
        )
        # At 094150.00 (without waiting 3s!), a newer valid RMC arrives while GGA had only quality=0.
        stream += make_rmc(utc="094150.00", status="A")
        fake_ser.chunks.put(stream.encode("ascii"))

        with patch.object(air.serial, "Serial", return_value=fake_ser):
            reader = air.GnssReader("COM7", 9600)
            try:
                no_fixes = [reader.read_gga(timeout_s=0.2) for _ in range(4)]
                recovered = reader.read_gga(timeout_s=0.2)
            finally:
                reader.close()

        self.assertEqual([f.satellites for f in no_fixes], [6, 6, 6, 6])
        self.assertTrue(all(f.fix_quality == 0 for f in no_fixes))
        self.assertTrue(recovered.nmea_received)
        self.assertEqual((recovered.utc, recovered.fix_quality), ("094150.00", 1))
        self.assertIsNotNone(recovered.latitude)

    def test_rmc_before_gga_same_epoch_conflict_and_upgrade(self) -> None:
        fake_ser = FakeChunkSerial()
        # Startup: RMC('A') arrives first, followed by same-epoch RTK-fixed GGA with conflicting coords
        payload = (
            make_rmc(utc="094146.00", status="A", lat_dm="3110.0000", lon_dm="12120.0000")
            + make_gga(utc="094146.00", lat_dm="3113.8240", lon_dm="12128.4220", quality=4, sats=14)
            # Next epoch while valid GGA is active: RMC arrives before GGA -> RMC waits for GGA!
            + make_rmc(utc="094147.00", status="A")
            + make_gga(utc="094147.00", quality=4, sats=15)
        ).encode("ascii")
        fake_ser.chunks.put(payload)

        with patch.object(air.serial, "Serial", return_value=fake_ser):
            reader = air.GnssReader("COM7", 9600)
            try:
                f1 = reader.read_gga(timeout_s=0.2)
                f2 = reader.read_gga(timeout_s=0.2)
                f3 = reader.read_gga(timeout_s=0.2)
            finally:
                reader.close()

        # Startup RMC returned as initial fallback, then immediately upgraded by same-epoch GGA(q=4)
        self.assertEqual((f1.utc, f1.fix_quality), ("094146.00", 1))
        self.assertEqual((f2.utc, f2.fix_quality, f2.satellites), ("094146.00", 4, 14))
        # On epoch 094147.00, RMC was suppressed in favor of active valid GGA, so f3 is directly GGA(q=4)
        self.assertEqual((f3.utc, f3.fix_quality, f3.satellites), ("094147.00", 4, 15))
        self.assertEqual(reader.epoch_conflicts, 1)
        self.assertEqual(reader.suppressed_rmc_count, 1)

    def test_stale_gga_skipped_while_cold_boot_time_sync_jump_accepted(self) -> None:
        fake_ser = FakeChunkSerial()
        payload = (
            # Cold boot internal clock 00:00:05 (q=0) -> GPS UTC lock 15:12:56 (q=1)
            make_gga(utc="000005.00", quality=0, sats=0)
            + make_gga(utc="151256.00", quality=1, sats=7)
            # Small out-of-order stale GGA 15:12:55 -> skipped
            + make_gga(utc="151255.00", quality=1, sats=7)
            + make_gga(utc="151257.00", quality=4, sats=12)
        ).encode("ascii")
        fake_ser.chunks.put(payload)

        with patch.object(air.serial, "Serial", return_value=fake_ser):
            reader = air.GnssReader("COM7", 9600)
            try:
                f1 = reader.read_gga(timeout_s=0.2)
                f2 = reader.read_gga(timeout_s=0.2)
                f3 = reader.read_gga(timeout_s=0.2)
            finally:
                reader.close()

        self.assertEqual([f1.utc, f2.utc, f3.utc], ["000005.00", "151256.00", "151257.00"])
        self.assertEqual(reader.stale_gga_skipped, 1)


class GnssSilentProbeSafetyTests(unittest.TestCase):
    """Fix 4: Disable 115200 baud rescue and flash writes on silent timeouts."""

    def test_silent_timeout_keeps_9600_and_never_writes_com_or_saveconfig(self) -> None:
        opened_bauds: list[tuple[str, int]] = []
        fake_ser = FakeChunkSerial(mode="instant_empty")

        def fake_serial_open(port: str, baud: int, **_kwargs: object) -> FakeChunkSerial:
            opened_bauds.append((port, baud))
            fake_ser.is_open = True
            return fake_ser

        with patch.object(air.serial, "Serial", side_effect=fake_serial_open):
            reader = air.GnssReader("COM7", 9600)
            reader.open()
            now = time.monotonic()
            for _ in range(10):
                reader._last_sentence_at = now - 120.0
                reader._last_poll_at = now - 5.0
                reader._probe_phase_started_at = now - 15.0
                reader._poll_output(now)
                now += 2.0
            self.assertEqual(reader.baud, 9600)
            reader.close()

        self.assertEqual(opened_bauds, [("COM7", 9600)])
        self.assertFalse(any(cmd.lower().startswith(b"com ") for cmd in fake_ser.writes), fake_ser.writes)
        self.assertFalse(any(b"saveconfig" in cmd.lower() for cmd in fake_ser.writes), fake_ser.writes)
        self.assertIn(b"log com1 gpgga ontime 1\r\n", fake_ser.writes)
        self.assertIn(b"log com3 gpgga ontime 1\r\n", fake_ser.writes)


class GnssDiagnosticsAndPolicyTests(unittest.TestCase):
    """Fix 5: GnssLogWriter, solution_type, fix_policy, port/baud distinction, compatibility."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_gnss_log_writer_default_disabled_and_env_opt_in(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            unused_path = Path(tmp) / "should_not_exist.jsonl"
            writer_default = GnssLogWriter()
            self.assertFalse(writer_default.enabled)
            self.assertFalse(writer_default.log_raw_bytes(b"$GNGGA\r\n"))

            writer_with_path_disabled = GnssLogWriter(unused_path)
            self.assertFalse(writer_with_path_disabled.enabled)
            self.assertFalse(writer_with_path_disabled.write_event("test"))
            self.assertFalse(unused_path.exists())

            with patch.dict(os.environ, {DIAG_LOG_ENV: ""}, clear=False):
                env_disabled = GnssLogWriter.from_env()
                self.assertFalse(env_disabled.enabled)

            env_target = Path(tmp) / "env_enabled.jsonl"
            with patch.dict(os.environ, {DIAG_LOG_ENV: str(env_target)}, clear=False):
                env_enabled = GnssLogWriter.from_env()
                try:
                    self.assertTrue(env_enabled.enabled)
                    self.assertTrue(env_enabled.write_event("ping", ok=True))
                finally:
                    env_enabled.close()
                self.assertTrue(env_target.exists())

    def test_gnss_log_writer_rolling_rotation_reconnects_and_disk_error_resilience(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "diag" / "gnss.jsonl"
            fail_writes = {"active": False}

            class FaultInjectableHandle:
                def __init__(self, path: Path) -> None:
                    if fail_writes["active"]:
                        raise PermissionError("simulated read-only disk")
                    self._inner = path.open("ab")

                def write(self, data: bytes) -> int:
                    if fail_writes["active"]:
                        raise OSError("simulated disk full mid-write")
                    return self._inner.write(data)

                def flush(self) -> None:
                    self._inner.flush()

                def close(self) -> None:
                    self._inner.close()

            writer = GnssLogWriter(
                log_path,
                enabled=True,
                max_bytes=2400,
                backup_count=2,
                opener=FaultInjectableHandle,
            )
            current_port = {"name": "COM7", "available": True}
            serials: list[FakeChunkSerial] = []

            def scan_ports() -> list[SimpleNamespace]:
                if not current_port["available"]:
                    return []
                return [
                    SimpleNamespace(
                        device=current_port["name"],
                        description="SinoGNSS M100",
                        manufacturer="ComNav",
                        vid=100,
                        pid=200,
                        serial_number="m100-sn",
                    )
                ]

            def open_serial(port: str, baud: int, **_kwargs: object) -> FakeChunkSerial:
                if not current_port["available"]:
                    raise OSError("serial disconnected")
                ser = FakeChunkSerial()
                ser.chunks.put(b"#LOGLISTA,COM1,GPGGA,ONTIME,1*\r\n")
                ser.chunks.put(
                    (
                        make_gga(utc="094146.00" if port == "COM7" else "094150.00", quality=4, sats=14)
                        + make_rmc(utc="094146.00" if port == "COM7" else "094150.00", status="A")
                    ).encode("ascii")
                )
                serials.append(ser)
                return ser

            def collect_logged_events() -> list[dict[str, object]]:
                found: list[dict[str, object]] = []
                for candidate in (Path(f"{log_path}.2"), Path(f"{log_path}.1"), log_path):
                    if candidate.exists():
                        for ln in candidate.read_text(encoding="utf-8").splitlines():
                            if ln.strip():
                                found.append(json.loads(ln))
                return found

            with patch.object(air.serial, "Serial", side_effect=open_serial):
                monitor = AirGnssMonitor(
                    "",
                    9600,
                    port_scanner=scan_ports,
                    initial_retry_s=0.02,
                    max_retry_s=0.05,
                    data_timeout_s=0.5,
                    diag_logger=writer,
                )
                try:
                    monitor.start()
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline and monitor.snapshot()["solution_type"] != SOLUTION_RTK_FIXED:
                        time.sleep(0.01)
                    self.assertEqual(monitor.snapshot()["connected_port"], "COM7")
                    # Wait briefly for trailing RMC audit in same burst
                    time.sleep(0.05)
                    pre_fault_events = collect_logged_events()
                    pre_kinds = {item["event"] for item in pre_fault_events}
                    for expected_kind in ("connection", "command", "raw_rx", "response", "sentence", "publish"):
                        self.assertIn(expected_kind, pre_kinds, f"Missing {expected_kind} in {pre_kinds}")

                    # Inject disk write failures during disconnect + reconnect to COM9.
                    fail_writes["active"] = True
                    current_port["available"] = False
                    for s in serials:
                        s.close()
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline and monitor.snapshot()["serial_open"]:
                        time.sleep(0.01)

                    current_port["name"] = "COM9"
                    current_port["available"] = True
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline and monitor.snapshot()["connected_port"] != "COM9":
                        time.sleep(0.01)
                    self.assertEqual(monitor.snapshot()["connected_port"], "COM9")
                    self.assertGreater(writer.write_errors, 0)

                    # Restore disk health and ensure logging automatically recovers and rotates.
                    fail_writes["active"] = False
                    for idx in range(12):
                        monitor._publish(
                            air.parse_gga(make_gga(utc=f"0942{idx:02d}.00", quality=4, sats=15)),
                            f"post-recovery-{idx}",
                        )
                finally:
                    monitor.stop()

            self.assertTrue(log_path.exists())
            self.assertTrue(Path(f"{log_path}.1").exists())
            self.assertTrue(Path(f"{log_path}.2").exists())
            self.assertFalse(Path(f"{log_path}.3").exists())
            self.assertGreater(writer.rotation_count, 0)

            all_events = collect_logged_events()
            self.assertGreater(len(all_events), 4)
            seqs = [int(item["seq"]) for item in all_events]
            self.assertEqual(seqs, sorted(seqs))
            event_kinds = {item["event"] for item in all_events}
            self.assertIn("publish", event_kinds)

    def test_solution_type_and_fix_policy_across_all_five_qualities_and_ground_compat(self) -> None:
        # 1) Five canonical GGA qualities: 0, 1, 2, 4, 5
        q_cases = [
            (0, SOLUTION_NO_FIX, False, False),
            (1, SOLUTION_SPP, True, False),
            (2, SOLUTION_DGPS, True, False),
            (4, SOLUTION_RTK_FIXED, True, True),
            (5, SOLUTION_RTK_FLOAT, True, False),
        ]
        monitor = AirGnssMonitor("COM7", 9600)
        widget = ground_gnss.GroundGnssMapWidget()
        widget.set_link_connected(True)
        cfg = air.SweepConfig()
        freq_hz = np.linspace(400e6, 4000e6, 8)
        zeros = {"S11": np.zeros(8, dtype=np.complex128), "S22": np.zeros(8, dtype=np.complex128)}
        time_ns = np.linspace(0.0, 10.0, 8)
        traces = {"S11": np.zeros(8, dtype=float), "S22": np.zeros(8, dtype=float)}
        expected_gga_fields = {f.name for f in fields(air.GgaFix)}
        self.assertEqual(len(expected_gga_fields), 10)

        for quality, expected_sol, expected_valid, expected_rtk_fixed in q_cases:
            raw_gga = make_gga(utc=f"10100{quality}.00", quality=quality, sats=10 if quality else 4)
            parsed = air.parse_gga(raw_gga)
            self.assertEqual(solution_type(parsed), expected_sol)

            # Default policy ('off'): any valid fix is eligible
            monitor.set_fix_policy(FIX_POLICY_OFF)
            state = expected_sol if expected_valid else "no_fix"
            monitor._publish(air.replace(parsed, serial_open=True, state=state), f"q={quality}")
            snap_off = monitor.snapshot()
            self.assertEqual(snap_off["fix"], expected_valid)
            self.assertEqual(snap_off["solution_type"], expected_sol)
            self.assertEqual(snap_off["is_rtk_fixed"], expected_rtk_fixed)
            self.assertFalse(snap_off["strict_fix_policy"])
            self.assertEqual(snap_off["fix_eligible"], expected_valid)

            # Strict RTK fixed policy: ONLY quality==4 valid GGA is eligible,
            # while 'fix', 'lat', 'lon', NPZ 10-field GNSS dict & ground telemetry stay unchanged!
            monitor.set_fix_policy(FIX_POLICY_STRICT_RTK_FIXED)
            snap_strict = monitor.snapshot()
            self.assertEqual(snap_strict["fix"], expected_valid)
            self.assertEqual(snap_strict["lat"], snap_off["lat"])
            self.assertEqual(snap_strict["lon"], snap_off["lon"])
            self.assertTrue(snap_strict["strict_fix_policy"])
            self.assertEqual(snap_strict["fix_eligible"], expected_rtk_fixed)

            # JSON wire serialization (allow_nan=False) + GroundGnssMapWidget compatibility
            wire = json.dumps(snap_strict, allow_nan=False)
            decoded_snap = json.loads(wire)
            norm = ground_gnss.normalize_gnss(decoded_snap)
            self.assertEqual(norm["fix_quality"], quality if expected_valid else 0)
            widget.update_data(decoded_snap)
            if expected_valid:
                self.assertIn("定位成功", widget.status.text())
            else:
                self.assertIn("尚未定位", widget.status.text())

            # NPZ 10-field GNSS dict invariance check
            lookup = monitor.latest()
            tf = monitor.telemetry_fix(lookup)
            blob = air.trace_npz_bytes(1, cfg, freq_hz, zeros, zeros, time_ns, traces, tf, channels=("S11", "S22"))
            with np.load(io.BytesIO(blob), allow_pickle=False) as npz:
                npz_gnss = json.loads(str(npz["gnss"].item()))
            self.assertEqual(set(npz_gnss.keys()), expected_gga_fields)

        widget.close()

        # 2) Guard: quality==4 without valid coordinates or from RMC must NEVER be classified as rtk_fixed!
        forged_rmc = air.replace(air.parse_rmc(make_rmc(status="A")), fix_quality=4)
        self.assertEqual(solution_type(forged_rmc), SOLUTION_SPP)
        self.assertFalse(fix_policy(forged_rmc, FIX_POLICY_STRICT_RTK_FIXED)["fix_eligible"])

        invalid_q4 = air.GgaFix(raw="$GNGGA,...", latitude=None, longitude=121.0, fix_quality=4)
        self.assertEqual(solution_type(invalid_q4), SOLUTION_NO_FIX)
        nan_q4 = air.GgaFix(raw="$GNGGA,...", latitude=float("nan"), longitude=121.0, fix_quality=4)
        self.assertEqual(solution_type(nan_q4), SOLUTION_NO_FIX)

        # Additional policy tiers
        float_fix = air.parse_gga(make_gga(quality=5))
        dgps_fix = air.parse_gga(make_gga(quality=2))
        self.assertTrue(fix_policy(float_fix, FIX_POLICY_RTK_ANY)["fix_eligible"])
        self.assertFalse(fix_policy(dgps_fix, FIX_POLICY_RTK_ANY)["fix_eligible"])
        self.assertTrue(fix_policy(dgps_fix, FIX_POLICY_DGPS_OR_BETTER)["fix_eligible"])

    def test_snapshot_distinguishes_configured_baud_and_actual_connected_port_baud(self) -> None:
        port_info = SimpleNamespace(
            device="COM11",
            description="SinoGNSS M100",
            manufacturer="ComNav",
            vid=12,
            pid=34,
            serial_number="sn-11",
        )
        gate = threading.Event()

        class CustomBaudReader:
            SILENCE_BEFORE_POLL_S = 2.0

            def __init__(self, port: str, _configured_baud: int) -> None:
                self.port = port
                self.baud = 9600  # Actual connected serial baud differs from configured 115200
                self.log = None
                self.diagnostics_logger = None
                self._emitted = False

            def open(self) -> None:
                pass

            def read_gga(self, timeout_s: float = 0.5) -> air.GgaFix:
                if not self._emitted:
                    self._emitted = True
                    return air.parse_gga(make_gga(utc="080001.00", quality=4, sats=14))
                gate.wait(timeout_s)
                return air.GgaFix(serial_open=True)

            def close(self) -> None:
                pass

        with patch.dict(os.environ, {FIX_POLICY_ENV: "off"}, clear=False):
            monitor = AirGnssMonitor(
                "",
                115200,
                reader_factory=CustomBaudReader,
                port_scanner=lambda: [port_info],
                initial_retry_s=0.02,
                max_retry_s=0.05,
            )
            pre = monitor.snapshot()
            self.assertEqual(pre["configured_port"], "")
            self.assertEqual(pre["configured_baud"], 115200)
            self.assertEqual(pre["connected_port"], "")
            self.assertIsNone(pre["connected_baud"])
            self.assertEqual(pre["baud"], 115200)

            try:
                monitor.start()
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline and not monitor.snapshot()["fix"]:
                    time.sleep(0.01)
                live = monitor.snapshot()
                self.assertTrue(live["fix"])
                self.assertEqual(live["configured_port"], "")
                self.assertEqual(live["configured_baud"], 115200)
                self.assertEqual(live["connected_port"], "COM11")
                self.assertEqual(live["connected_baud"], 9600)
                self.assertEqual(live["port"], "COM11")
                self.assertEqual(live["baud"], 9600)
                self.assertEqual(live["solution_type"], SOLUTION_RTK_FIXED)
            finally:
                gate.set()
                monitor.stop()

    def test_unmodified_air_service_instantiates_monitor_with_default_disabled_logger(self) -> None:
        import uav_gpr_air_service as svc

        with tempfile.TemporaryDirectory() as tmp:
            cfg = svc.AirConfig(gnss_port="COM7", gnss_baud=9600, device_source="simulated")
            service = svc.AirBlackboxService(cfg, program_dir=Path(tmp))
            try:
                self.assertFalse(service.gnss.diag_logger.enabled)
                self.assertEqual(service.gnss.fix_policy_mode, FIX_POLICY_OFF)
                res = service.handle_command("test_gnss", {})
                self.assertTrue(res["ok"])
                self.assertEqual(res["configured_port"], "COM7")
                self.assertEqual(res["configured_baud"], 9600)
                self.assertEqual(res["connected_port"], "")
                self.assertIsNone(res["connected_baud"])
                self.assertEqual(res["solution_type"], SOLUTION_NO_FIX)
                self.assertFalse(res["fix_eligible"])
            finally:
                service.shutdown()


class GnssHistoricalReplayTests(unittest.TestCase):
    """Replay historical Nov 6 field flight GGAs from 20261106_151256_0009 if available."""

    @staticmethod
    def _find_historical_metadata() -> Path | None:
        repo_root = Path(__file__).resolve().parent.parent
        candidates = [
            repo_root / "docs" / "261110无人机GNSS问题汇总" / "20261106_151256_0009" / "20261106_151256_0009" / "run_metadata.jsonl",
            repo_root / "docs" / "261110无人机GNSS问题汇总" / "20261106_151256_0009" / "run_metadata.jsonl",
            Path(r"d:\博士任务\UVA_GPR_system\docs\261110无人机GNSS问题汇总\20261106_151256_0009\20261106_151256_0009\run_metadata.jsonl"),
            Path(r"d:\博士任务\UVA_GPR_system\docs\261110无人机GNSS问题汇总\20261106_151256_0009\run_metadata.jsonl"),
        ]
        for path in candidates:
            if path.is_file():
                return path
        return None

    def test_historical_20261106_151256_0009_gga_burst_replay_parity(self) -> None:
        meta_path = self._find_historical_metadata()
        if meta_path is None:
            self.skipTest("Historical dataset 20261106_151256_0009 not present on this machine")

        epochs: list[dict[str, object]] = []
        for line in meta_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            gnss = row["gnss"]
            if not epochs or gnss["raw"] != epochs[-1]["raw"]:
                epochs.append(gnss)

        self.assertEqual(len(epochs), 125)
        fake_ser = FakeChunkSerial()
        # Pack into 25-sentence multi-line chunks to exercise FIFO + parser + epoch audit together.
        for start in range(0, len(epochs), 25):
            block = "".join(str(ep["raw"]).strip() + "\r\n" for ep in epochs[start : start + 25])
            fake_ser.chunks.put(block.encode("ascii"))

        replayed: list[air.GgaFix] = []
        with patch.object(air.serial, "Serial", return_value=fake_ser):
            reader = air.GnssReader("COM7", 9600)
            try:
                for _ in range(len(epochs)):
                    replayed.append(reader.read_gga(timeout_s=0.2))
            finally:
                reader.close()

        self.assertEqual(reader.fifo_overflows, 0)
        self.assertEqual(reader.parse_errors, 0)
        valid_count = 0
        for expected, actual in zip(epochs, replayed):
            self.assertTrue(actual.nmea_received)
            self.assertEqual(actual.utc, expected["utc"])
            self.assertEqual(actual.fix_quality, expected["fix_quality"])
            self.assertEqual(actual.satellites, expected["satellites"])
            if expected["latitude"] is None:
                self.assertIsNone(actual.latitude)
                self.assertIsNone(actual.longitude)
                self.assertEqual(solution_type(actual), SOLUTION_NO_FIX)
            else:
                valid_count += 1
                self.assertAlmostEqual(actual.latitude, expected["latitude"], places=9)
                self.assertAlmostEqual(actual.longitude, expected["longitude"], places=9)
                self.assertEqual(solution_type(actual), SOLUTION_SPP)
            if expected["altitude_m"] is None:
                self.assertIsNone(actual.altitude_m)
            else:
                self.assertAlmostEqual(actual.altitude_m, expected["altitude_m"], places=6)
        self.assertEqual(valid_count, 3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
