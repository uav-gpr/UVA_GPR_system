"""LibreVNA USB protocol-layer unit tests (no hardware, pure Python).

Adapted from the field-validated tests of the read-only reference project
(``E:\\钢筋仪软件开发\\tests\\test_librevna_protocol.py`` and
``test_librevna_dual_protocol.py``).  Fixed byte vectors were computed with a
venv Python and hard-coded here; the protocol bytes are identical because the
module behaviour is byte-aligned with the reference.

Covers the roadmap section 15.1 protocol test matrix:

- fixed packet encoding and CRC;
- PacketStream noise / glue packets / half packets / illegal lengths;
- ``0x1240`` / ``0x1241`` fixed payloads;
- S11/S22 same-packet parsing;
- descriptor missing / duplicate / reordered;
- zero reference, NaN/Inf;
- truncated payload and illegal group lengths;
- protocol module introduces no new dependencies or hardware access.
"""

from __future__ import annotations

import ast
import inspect
import json
import struct
import unittest

from librevna.librevna_protocol import (
    ACK,
    ALLOWED_STAGES_BITMAPS,
    DEVICE_INFO,
    NACK,
    REQUEST_DEVICE_INFO,
    SET_IDLE,
    S11_S22_STAGES_BITMAP,
    S11_STAGES_BITMAP,
    SWEEP_SETTINGS,
    VNA_DATAPOINT,
    Packet,
    PacketStream,
    SweepSettings,
    crc32,
    datapoint_to_s11,
    decode_device_info,
    encode_packet,
    encode_sweep_settings,
    parse_s11_point,
    parse_s11_s22_point,
    parse_vna_datapoint,
)

# ---- fixed vectors (computed with a venv Python, hard-coded) ----

ACK_PACKET_HEX = "5a080007c1f48315"
REQ_DEV_INFO_HEX = "5a08000ff37c581b"
SET_IDLE_HEX = "5a0800141fb53d91"
# encode_sweep_settings(0.5e9, 2.5e9, 101, 10000, -10.0, 0)
SWEEP_SETTINGS_HEX = "0065cd1d0000000000f902950000000065001027000018fc0c401218fc0000"
# VNADatapoint (BLOCKED layout: 12-byte header + all real + all imag + all desc):
# freq=500MHz, cdbm=-1000, point=0, reals=[1.0(ref), -0.5(port1)],
# imags=[0.0(ref), 0.25(port1)], descs=[0x10(ref), 0x01(port1)]
VNA_DATAPOINT_PAYLOAD_HEX = "0065cd1d0000000018fc00000000803f000000bf000000000000803e1001"
VNA_DATAPOINT_PACKET_HEX = (
    "5a26001b0065cd1d0000000018fc00000000803f000000bf000000000000803e1001fecc9f61"
)
# DeviceInfo: protocol=14, fw=1.2.3, hw_version=5, rev='A',
# min_freq=100MHz, max_freq=6GHz, min_ifbw=1, max_ifbw=1MHz, max_points=10001,
# min_dbm=-30, max_dbm=10, min_rbw=1, max_rbw=1MHz, max_amplitude_points=200,
# max_harmonic=12GHz, num_ports=2, max_dwell=65535
DEVICE_INFO_PAYLOAD_HEX = (
    "0e00010203054100e1f5050000000000bca065010000000100000040420f00112748f4e803"
    "0100000040420f00c8007841cb0200000002ffff"
)

ACK_PACKET_BYTES = bytes.fromhex(ACK_PACKET_HEX)


def _vna_payload(
    *,
    freq_hz: int = 500_000_000,
    point_number: int = 0,
    ref: complex = 1.0 + 0j,
    port1: complex = -0.5 + 0.25j,
) -> bytes:
    """Build a stage-0 S11 datapoint payload (ref + port1 receivers)."""
    payload = struct.pack("<QhH", freq_hz, -1000, point_number)
    payload += struct.pack("<ff", ref.real, port1.real)  # reals block
    payload += struct.pack("<ff", ref.imag, port1.imag)  # imags block
    payload += bytes([0x10, 0x01])  # descs block: ref, port1
    return payload


def _dual_payload(
    receivers: list[tuple[int, complex]],
    *,
    freq_hz: int = 500_000_000,
    point_number: int = 0,
    cdbm: int = -1000,
) -> bytes:
    """Build a dual-port VNADatapoint payload (BLOCKED layout, arbitrary order).

    ``receivers`` is a ``(desc_byte, complex)`` list in payload order (duplicate
    desc and arbitrary ordering allowed).
    """
    n = len(receivers)
    payload = struct.pack("<QhH", freq_hz, cdbm, point_number)
    payload += struct.pack("<" + "f" * n, *[v.real for _, v in receivers])
    payload += struct.pack("<" + "f" * n, *[v.imag for _, v in receivers])
    payload += bytes(d for d, _ in receivers)
    return payload


def _full_two_port_receivers() -> list[tuple[int, complex]]:
    """A complete dual-port datapoint (6 receivers) with hand-computed S11/S22:
    S11 = (-0.5+0.25j) / (1.0+0j) = -0.5+0.25j;
    S22 = (0.6-0.3j) / (2.0+0j) = 0.3-0.15j.
    ``0x02`` / ``0x21`` are the S21 / S12 inputs, this round ignored.
    """
    return [
        (0x01, complex(-0.5, 0.25)),   # stage0 Port1 (S11 input)
        (0x02, complex(9.0, 9.0)),      # stage0 Port2 (S21 input, ignored)
        (0x13, complex(1.0, 0.0)),      # stage0 Reference (S11 ref)
        (0x21, complex(7.0, 7.0)),      # stage1 Port1 (S12 input, ignored)
        (0x22, complex(0.6, -0.3)),     # stage1 Port2 (S22 input)
        (0x33, complex(2.0, 0.0)),      # stage1 Reference (S22 ref)
    ]


class Crc32Tests(unittest.TestCase):
    def test_fixed_vector(self) -> None:
        # zlib known vector: crc32("123456789") = 0xCBF43926
        self.assertEqual(crc32(b"123456789"), 0xCBF43926)

    def test_empty_data(self) -> None:
        self.assertEqual(crc32(b""), 0)

    def test_high_bit_range(self) -> None:
        # must keep only the low 32 bits
        self.assertEqual(crc32(b"a"), crc32(b"a") & 0xFFFFFFFF)


class EncodePacketTests(unittest.TestCase):
    def test_ack_fixed_bytes(self) -> None:
        self.assertEqual(encode_packet(ACK).hex(), ACK_PACKET_HEX)

    def test_request_device_info_fixed_bytes(self) -> None:
        self.assertEqual(encode_packet(REQUEST_DEVICE_INFO).hex(), REQ_DEV_INFO_HEX)

    def test_set_idle_fixed_bytes(self) -> None:
        self.assertEqual(encode_packet(SET_IDLE).hex(), SET_IDLE_HEX)

    def test_packet_length_field(self) -> None:
        data = encode_packet(SWEEP_SETTINGS, bytes.fromhex(SWEEP_SETTINGS_HEX))
        # length = HEADER(1) + length(2) + type(1) + payload(31) + CRC(4) = 39
        self.assertEqual(struct.unpack_from("<H", data, 1)[0], 39)
        self.assertEqual(len(data), 39)

    def test_crc_covers_body(self) -> None:
        data = encode_packet(ACK)
        body = data[:-4]
        self.assertEqual(struct.unpack_from("<I", data, 4)[0], crc32(body))


class PacketStreamTests(unittest.TestCase):
    def test_single_packet(self) -> None:
        stream = PacketStream()
        packets = stream.feed(ACK_PACKET_BYTES)
        self.assertEqual(packets, [Packet(ACK, b"")])

    def test_noise_byte_prefix(self) -> None:
        stream = PacketStream()
        packets = stream.feed(b"\xde\xad\xbe\xef" + ACK_PACKET_BYTES)
        self.assertEqual(packets, [Packet(ACK, b"")])

    def test_multiple_packets_one_read(self) -> None:
        stream = PacketStream()
        packets = stream.feed(ACK_PACKET_BYTES + bytes.fromhex(SET_IDLE_HEX))
        self.assertEqual(packets, [Packet(ACK, b""), Packet(SET_IDLE, b"")])

    def test_split_across_reads(self) -> None:
        stream = PacketStream()
        self.assertEqual(stream.feed(ACK_PACKET_BYTES[:5]), [])
        packets = stream.feed(ACK_PACKET_BYTES[5:])
        self.assertEqual(packets, [Packet(ACK, b"")])

    def test_buffer_persists_across_reads(self) -> None:
        stream = PacketStream()
        self.assertEqual(stream.feed(b"\x00\x01\x02"), [])
        packets = stream.feed(ACK_PACKET_BYTES)
        self.assertEqual(packets, [Packet(ACK, b"")])

    def test_invalid_length_drops_byte(self) -> None:
        # fake header with length=0 (< 8): drop that byte, realign to next packet
        stream = PacketStream()
        packets = stream.feed(b"\x5a\x00\x00" + ACK_PACKET_BYTES)
        self.assertEqual(packets, [Packet(ACK, b"")])

    def test_crc_error_drops_non_datapoint_packet(self) -> None:
        # non-VNA_DATAPOINT packet with CRC error is dropped; the valid packet
        # that follows is still extracted.
        bad = bytearray(ACK_PACKET_BYTES)
        bad[-1] ^= 0xFF
        stream = PacketStream()
        packets = stream.feed(bytes(bad) + ACK_PACKET_BYTES)
        self.assertEqual(packets, [Packet(ACK, b"")])

    def test_vna_datapoint_crc_is_skipped(self) -> None:
        # Established reference behaviour: VNA_DATAPOINT (type 27) skips CRC
        # validation -- a CRC error still yields the packet. Do not "fix".
        bad = bytearray(bytes.fromhex(VNA_DATAPOINT_PACKET_HEX))
        bad[-1] ^= 0xFF
        stream = PacketStream()
        packets = stream.feed(bytes(bad))
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].packet_type, VNA_DATAPOINT)
        self.assertEqual(packets[0].payload.hex(), VNA_DATAPOINT_PAYLOAD_HEX)

    def test_feed_vna_datapoint_packet(self) -> None:
        stream = PacketStream()
        packets = stream.feed(bytes.fromhex(VNA_DATAPOINT_PACKET_HEX))
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].packet_type, VNA_DATAPOINT)
        self.assertEqual(packets[0].payload.hex(), VNA_DATAPOINT_PAYLOAD_HEX)


class DeviceInfoTests(unittest.TestCase):
    def test_fixed_payload_decode(self) -> None:
        info = decode_device_info(bytes.fromhex(DEVICE_INFO_PAYLOAD_HEX))
        self.assertEqual(info.protocol, 14)
        self.assertEqual(info.firmware, "1.2.3")
        self.assertEqual(info.hardware_version, 5)
        self.assertEqual(info.hardware_revision, "A")
        self.assertEqual(info.min_freq_hz, 100_000_000)
        self.assertEqual(info.max_freq_hz, 6_000_000_000)
        self.assertEqual(info.min_ifbw_hz, 1)
        self.assertEqual(info.max_ifbw_hz, 1_000_000)
        self.assertEqual(info.max_points, 10001)
        self.assertAlmostEqual(info.min_dbm, -30.0)
        self.assertAlmostEqual(info.max_dbm, 10.0)
        self.assertEqual(info.min_rbw_hz, 1)
        self.assertEqual(info.max_rbw_hz, 1_000_000)
        self.assertEqual(info.max_amplitude_points, 200)
        self.assertEqual(info.max_harmonic_freq_hz, 12_000_000_000)
        self.assertEqual(info.num_ports, 2)
        self.assertEqual(info.max_dwell_time_us, 65535)

    def test_extras_dict_json_serializable(self) -> None:
        info = decode_device_info(bytes.fromhex(DEVICE_INFO_PAYLOAD_HEX))
        json.dumps(info.to_extras_dict())  # must not raise

    def test_short_payload_raises(self) -> None:
        with self.assertRaises(struct.error):
            decode_device_info(b"\x00" * 10)


class SweepSettingsTests(unittest.TestCase):
    def test_fixed_payload_encode(self) -> None:
        data = encode_sweep_settings(500_000_000, 2_500_000_000, 101, 10_000, -10.0, 0)
        self.assertEqual(data.hex(), SWEEP_SETTINGS_HEX)

    def test_sweep_settings_dataclass_encode(self) -> None:
        settings = SweepSettings(
            start_hz=500_000_000,
            stop_hz=2_500_000_000,
            points=101,
            ifbw_hz=10_000,
            power_dbm=-10.0,
            dwell_us=0,
        )
        self.assertEqual(settings.encode().hex(), SWEEP_SETTINGS_HEX)

    def test_validation(self) -> None:
        with self.assertRaises(ValueError):
            SweepSettings(start_hz=0, stop_hz=1, points=2, ifbw_hz=1000)
        with self.assertRaises(ValueError):
            SweepSettings(start_hz=1, stop_hz=1, points=2, ifbw_hz=1000)
        with self.assertRaises(ValueError):
            SweepSettings(start_hz=1, stop_hz=2, points=1, ifbw_hz=1000)
        with self.assertRaises(TypeError):
            SweepSettings(start_hz=1, stop_hz=2, points=True, ifbw_hz=1000)  # type: ignore[arg-type]


class SweepSettingsStagesBitmapTests(unittest.TestCase):
    """Explicit stages_bitmap encode API: 0x1240 / 0x1241 at offset 25."""

    def test_default_payload_matches_legacy_hex(self) -> None:
        # no stages_bitmap passed: byte-identical to the historic fixed vector
        data = encode_sweep_settings(500_000_000, 2_500_000_000, 101, 10_000, -10.0, 0)
        self.assertEqual(data.hex(), SWEEP_SETTINGS_HEX)

    def test_explicit_s11_matches_default(self) -> None:
        default = encode_sweep_settings(500_000_000, 2_500_000_000, 101, 10_000, -10.0, 0)
        explicit = encode_sweep_settings(
            500_000_000, 2_500_000_000, 101, 10_000, -10.0, 0,
            stages_bitmap=S11_STAGES_BITMAP,
        )
        self.assertEqual(default, explicit)

    def test_s11_s22_stages_bitmap_offset_value(self) -> None:
        data = encode_sweep_settings(
            500_000_000, 2_500_000_000, 101, 10_000, -10.0, 0,
            stages_bitmap=S11_S22_STAGES_BITMAP,
        )
        # stages field at offset 25, uint16 LE
        value = struct.unpack_from("<H", data, 25)[0]
        self.assertEqual(value, 0x1241)
        self.assertEqual(data[25], 0x41)  # low byte
        self.assertEqual(data[26], 0x12)  # high byte

    def test_s11_stages_bitmap_offset_value(self) -> None:
        data = encode_sweep_settings(500_000_000, 2_500_000_000, 101, 10_000, -10.0, 0)
        value = struct.unpack_from("<H", data, 25)[0]
        self.assertEqual(value, 0x1240)

    def test_only_stages_field_differs(self) -> None:
        single = encode_sweep_settings(500_000_000, 2_500_000_000, 101, 10_000, -10.0, 0)
        dual = encode_sweep_settings(
            500_000_000, 2_500_000_000, 101, 10_000, -10.0, 0,
            stages_bitmap=S11_S22_STAGES_BITMAP,
        )
        self.assertEqual(len(single), len(dual))
        diffs = [i for i in range(len(single)) if single[i] != dual[i]]
        # only offset 25 (low byte 0x40->0x41) differs; offset 26 (0x12) unchanged
        self.assertEqual(diffs, [25])

    def test_invalid_stages_bitmap_rejected(self) -> None:
        base = (500_000_000, 2_500_000_000, 101, 10_000, -10.0, 0)
        # bool
        with self.assertRaises(TypeError):
            encode_sweep_settings(*base, stages_bitmap=True)  # type: ignore[arg-type]
        # float
        with self.assertRaises(TypeError):
            encode_sweep_settings(*base, stages_bitmap=4656.0)  # type: ignore[arg-type]
        # str
        with self.assertRaises(TypeError):
            encode_sweep_settings(*base, stages_bitmap="0x1240")  # type: ignore[arg-type]
        # negative
        with self.assertRaises(ValueError):
            encode_sweep_settings(*base, stages_bitmap=-1)
        # beyond uint16
        with self.assertRaises(ValueError):
            encode_sweep_settings(*base, stages_bitmap=0x10000)
        # in-range but not hardware-validated
        with self.assertRaises(ValueError):
            encode_sweep_settings(*base, stages_bitmap=0x1242)
        with self.assertRaises(ValueError):
            encode_sweep_settings(*base, stages_bitmap=0x0000)

    def test_dataclass_encode_matches_function(self) -> None:
        settings = SweepSettings(
            start_hz=500_000_000, stop_hz=2_500_000_000, points=101,
            ifbw_hz=10_000, power_dbm=-10.0, dwell_us=0,
        )
        self.assertEqual(
            settings.encode(),
            encode_sweep_settings(500_000_000, 2_500_000_000, 101, 10_000, -10.0, 0),
        )
        dual = SweepSettings(
            start_hz=500_000_000, stop_hz=2_500_000_000, points=101,
            ifbw_hz=10_000, power_dbm=-10.0, dwell_us=0,
            stages_bitmap=S11_S22_STAGES_BITMAP,
        )
        self.assertEqual(
            dual.encode(),
            encode_sweep_settings(
                500_000_000, 2_500_000_000, 101, 10_000, -10.0, 0,
                stages_bitmap=S11_S22_STAGES_BITMAP,
            ),
        )

    def test_dataclass_rejects_invalid_stages_bitmap(self) -> None:
        with self.assertRaises(ValueError):
            SweepSettings(
                start_hz=500_000_000, stop_hz=2_500_000_000, points=101,
                ifbw_hz=10_000, stages_bitmap=0x1242,
            )
        with self.assertRaises(TypeError):
            SweepSettings(
                start_hz=500_000_000, stop_hz=2_500_000_000, points=101,
                ifbw_hz=10_000, stages_bitmap=True,  # type: ignore[arg-type]
            )

    def test_allowed_stages_bitmaps_constant(self) -> None:
        self.assertEqual(ALLOWED_STAGES_BITMAPS, (0x1240, 0x1241))
        self.assertIn(S11_STAGES_BITMAP, ALLOWED_STAGES_BITMAPS)
        self.assertIn(S11_S22_STAGES_BITMAP, ALLOWED_STAGES_BITMAPS)


class VnaDatapointTests(unittest.TestCase):
    def test_fixed_payload_parse(self) -> None:
        payload = bytes.fromhex(VNA_DATAPOINT_PAYLOAD_HEX)
        frequency, cdbm, point_number, values = parse_vna_datapoint(payload)
        self.assertEqual(frequency, 500_000_000)
        self.assertEqual(cdbm, -1000)
        self.assertEqual(point_number, 0)
        self.assertEqual(len(values), 2)
        desc_map = {d: v for d, v in values}
        self.assertAlmostEqual(desc_map[0x10].real, 1.0)
        self.assertAlmostEqual(desc_map[0x10].imag, 0.0)
        self.assertAlmostEqual(desc_map[0x01].real, -0.5)
        self.assertAlmostEqual(desc_map[0x01].imag, 0.25)

    def test_s11_complex_ratio(self) -> None:
        payload = bytes.fromhex(VNA_DATAPOINT_PAYLOAD_HEX)
        frequency, s11 = datapoint_to_s11(payload)
        self.assertEqual(frequency, 500_000_000)
        self.assertAlmostEqual(s11.real, -0.5)
        self.assertAlmostEqual(s11.imag, 0.25)

    def test_parse_s11_point(self) -> None:
        payload = bytes.fromhex(VNA_DATAPOINT_PAYLOAD_HEX)
        point_number, freq, s11 = parse_s11_point(payload)
        self.assertEqual(point_number, 0)
        self.assertEqual(freq, 500_000_000)
        self.assertAlmostEqual(s11.real, -0.5)
        self.assertAlmostEqual(s11.imag, 0.25)

    def test_reference_zero(self) -> None:
        payload = _vna_payload(ref=0.0 + 0j)
        self.assertIsNone(datapoint_to_s11(payload))
        self.assertIsNone(parse_s11_point(payload))

    def test_missing_reference(self) -> None:
        # only a port1 receiver -> reference missing -> None
        payload = struct.pack("<QhH", 500_000_000, -1000, 0)
        payload += struct.pack("<f", -0.5)  # reals
        payload += struct.pack("<f", 0.25)  # imags
        payload += bytes([0x01])  # descs
        self.assertIsNone(datapoint_to_s11(payload))
        self.assertIsNone(parse_s11_point(payload))

    def test_non_finite_s11(self) -> None:
        # port1 = inf -> S11 non-finite: datapoint_to_s11 returns non-finite
        # (reference behaviour), parse_s11_point (strict) returns None.
        payload = _vna_payload(port1=complex(float("inf"), 0.0))
        freq, s11 = datapoint_to_s11(payload)
        self.assertTrue(freq > 0)
        self.assertTrue(s11.real == float("inf") or s11.real == float("-inf"))
        self.assertIsNone(parse_s11_point(payload))

    def test_non_finite_reference(self) -> None:
        # reference = NaN -> S11 non-finite -> parse returns None (not accepted)
        payload = _vna_payload(ref=complex(float("nan"), 0.0))
        self.assertIsNone(parse_s11_point(payload))

    def test_infinite_reference_yields_finite_zero(self) -> None:
        # reference = inf -> Python complex division gives finite 0; not
        # non-finite data, so it is accepted.
        payload = _vna_payload(ref=complex(float("inf"), 0.0))
        result = parse_s11_point(payload)
        self.assertIsNotNone(result)
        _, _, s11 = result  # type: ignore[misc]
        self.assertTrue(s11.real == 0.0 or s11.real == -0.0)
        self.assertTrue(s11.imag == 0.0 or s11.imag == -0.0)

    def test_truncated_payload_raises(self) -> None:
        # too short for the 12-byte header: struct.error (caller wraps as error)
        payload = b"\x00" * 8
        with self.assertRaises(struct.error):
            parse_vna_datapoint(payload)
        with self.assertRaises(struct.error):
            parse_s11_point(payload)

    def test_exact_header_only_payload_returns_empty_values(self) -> None:
        # exactly 12 bytes (header only, zero receivers) is a legal payload:
        # values=[], and the S11/S22 parsers return None (missing receivers).
        payload = struct.pack("<QhH", 500_000_000, -1000, 0)
        self.assertEqual(parse_vna_datapoint(payload)[3], [])
        self.assertIsNone(datapoint_to_s11(payload))
        self.assertIsNone(parse_s11_point(payload))
        self.assertIsNone(parse_s11_s22_point(payload))

    def test_single_receiver_missing_port_returns_none(self) -> None:
        # 12 + 9 (one complete receiver group) but only the reference, missing
        # port1 -> S11 is None.  12+9 is a full receiver, not a "half group".
        payload = struct.pack("<QhH", 500_000_000, -1000, 0)
        payload += struct.pack("<f", 1.0)  # reals
        payload += struct.pack("<f", 0.0)  # imags
        payload += bytes([0x10])  # descs
        self.assertIsNone(datapoint_to_s11(payload))
        self.assertIsNone(parse_s11_point(payload))

    def test_ignores_other_stages(self) -> None:
        # stage-1 receivers are not counted for S11
        payload = struct.pack("<QhH", 500_000_000, -1000, 0)
        payload += struct.pack("<ff", 1.0, 1.0)  # reals
        payload += struct.pack("<ff", 0.0, 0.0)  # imags
        payload += bytes([(1 << 5) | 0x10, (1 << 5) | 0x01])  # stage-1 ref/port1
        self.assertIsNone(datapoint_to_s11(payload))


class MalformedReceiverGroupLengthTests(unittest.TestCase):
    """A receiver section length that is not a multiple of 9 must raise.

    ``12 + 9*N + (1..8)`` bytes means 1..8 stray trailing bytes: they are never
    floor-divided away.  ``parse_vna_datapoint``, ``parse_s11_point`` and
    ``parse_s11_s22_point`` all surface ``struct.error`` so a truncated
    datapoint can never be patched into a sweep.
    """

    def _assert_all_raise_struct_error(self, payload: bytes) -> None:
        with self.assertRaises(struct.error):
            parse_vna_datapoint(payload)
        with self.assertRaises(struct.error):
            parse_s11_point(payload)
        with self.assertRaises(struct.error):
            parse_s11_s22_point(payload)

    def test_header_plus_1_to_8_stray_bytes_raise(self) -> None:
        # 12-byte header + 1..8 stray bytes -> illegal receiver section.
        header = struct.pack("<QhH", 500_000_000, -1000, 0)
        for extra in range(1, 9):
            payload = header + b"\x00" * extra
            self._assert_all_raise_struct_error(payload)
            # struct.error must carry a stable, descriptive message
            try:
                parse_vna_datapoint(payload)
            except struct.error as exc:
                self.assertIn("receiver section length", str(exc))

    def test_single_channel_payload_plus_stray_bytes_raise(self) -> None:
        # a legal 2-receiver (ref + port1) S11 payload plus 1..8 trailing bytes.
        base = _vna_payload()
        for extra in range(1, 9):
            self._assert_all_raise_struct_error(base + b"\x00" * extra)

    def test_dual_channel_payload_plus_stray_bytes_raise(self) -> None:
        # a legal 6-receiver dual S11/S22 payload plus 1..8 trailing bytes.
        base = _dual_payload(_full_two_port_receivers())
        for extra in range(1, 9):
            self._assert_all_raise_struct_error(base + b"\x00" * extra)

    def test_legal_lengths_still_parse(self) -> None:
        # regression guard: legal 12+9*N payloads are unchanged.
        single = _vna_payload()
        dual = _dual_payload(_full_two_port_receivers())
        self.assertEqual((len(single) - 12) % 9, 0)
        self.assertEqual((len(dual) - 12) % 9, 0)
        self.assertIsNotNone(parse_s11_point(single))
        self.assertIsNotNone(parse_s11_s22_point(dual))


class ParseS11S22Tests(unittest.TestCase):
    """Contract of ``parse_s11_s22_point``: same-packet S11/S22."""

    def test_fixed_payload_computes_s11_and_s22(self) -> None:
        # hand-computed: S11=(-0.5+0.25j)/1.0=-0.5+0.25j; S22=(0.6-0.3j)/2.0=0.3-0.15j
        payload = _dual_payload(_full_two_port_receivers())
        result = parse_s11_s22_point(payload)
        self.assertIsNotNone(result)
        point_number, freq, s11, s22 = result  # type: ignore[misc]
        self.assertEqual(point_number, 0)
        self.assertEqual(freq, 500_000_000)
        self.assertAlmostEqual(s11.real, -0.5, places=6)
        self.assertAlmostEqual(s11.imag, 0.25, places=6)
        self.assertAlmostEqual(s22.real, 0.3, places=6)
        self.assertAlmostEqual(s22.imag, -0.15, places=6)

    def test_complex_division_preserves_real_and_imag(self) -> None:
        # hand-computed: S11=(1-1j)/(1+1j)=-1j; S22=1/(2+1j)=(2-1j)/5=0.4-0.2j
        receivers = [
            (0x01, complex(1.0, -1.0)),   # s11_port1
            (0x02, complex(0.0, 0.0)),     # ignored
            (0x13, complex(1.0, 1.0)),     # s11_ref
            (0x21, complex(0.0, 0.0)),     # ignored
            (0x22, complex(1.0, 0.0)),     # s22_port2
            (0x33, complex(2.0, 1.0)),     # s22_ref
        ]
        result = parse_s11_s22_point(_dual_payload(receivers))
        self.assertIsNotNone(result)
        _, _, s11, s22 = result  # type: ignore[misc]
        self.assertAlmostEqual(s11.real, 0.0, places=6)
        self.assertAlmostEqual(s11.imag, -1.0, places=6)
        self.assertAlmostEqual(s22.real, 0.4, places=6)
        self.assertAlmostEqual(s22.imag, -0.2, places=6)

    def test_receiver_order_shuffled_same_result(self) -> None:
        ordered = _full_two_port_receivers()
        shuffled = list(reversed(ordered))
        r1 = parse_s11_s22_point(_dual_payload(ordered))
        r2 = parse_s11_s22_point(_dual_payload(shuffled))
        self.assertIsNotNone(r1)
        self.assertIsNotNone(r2)
        _, _, s11_a, s22_a = r1  # type: ignore[misc]
        _, _, s11_b, s22_b = r2  # type: ignore[misc]
        self.assertAlmostEqual(s11_a.real, s11_b.real, places=6)
        self.assertAlmostEqual(s11_a.imag, s11_b.imag, places=6)
        self.assertAlmostEqual(s22_a.real, s22_b.real, places=6)
        self.assertAlmostEqual(s22_a.imag, s22_b.imag, places=6)
        self.assertAlmostEqual(s11_b.real, -0.5, places=6)
        self.assertAlmostEqual(s22_b.real, 0.3, places=6)

    def test_payload_contains_both_stage0_and_stage1(self) -> None:
        payload = _dual_payload(_full_two_port_receivers())
        _, _, _, values = parse_vna_datapoint(payload)
        stages = {desc >> 5 for desc, _ in values}
        self.assertIn(0, stages)
        self.assertIn(1, stages)
        self.assertIsNotNone(parse_s11_s22_point(payload))

    def test_missing_stage0_reference_returns_none(self) -> None:
        receivers = [
            (0x01, complex(-0.5, 0.25)),
            (0x02, complex(9.0, 9.0)),
            # missing 0x13 (stage0 reference)
            (0x21, complex(7.0, 7.0)),
            (0x22, complex(0.6, -0.3)),
            (0x33, complex(2.0, 0.0)),
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_missing_stage0_port1_returns_none(self) -> None:
        receivers = [
            # missing 0x01 (stage0 Port1)
            (0x02, complex(9.0, 9.0)),
            (0x13, complex(1.0, 0.0)),
            (0x21, complex(7.0, 7.0)),
            (0x22, complex(0.6, -0.3)),
            (0x33, complex(2.0, 0.0)),
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_missing_stage1_reference_returns_none(self) -> None:
        receivers = [
            (0x01, complex(-0.5, 0.25)),
            (0x02, complex(9.0, 9.0)),
            (0x13, complex(1.0, 0.0)),
            (0x21, complex(7.0, 7.0)),
            (0x22, complex(0.6, -0.3)),
            # missing 0x33 (stage1 reference)
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_missing_stage1_port2_returns_none(self) -> None:
        receivers = [
            (0x01, complex(-0.5, 0.25)),
            (0x02, complex(9.0, 9.0)),
            (0x13, complex(1.0, 0.0)),
            (0x21, complex(7.0, 7.0)),
            # missing 0x22 (stage1 Port2)
            (0x33, complex(2.0, 0.0)),
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_stage0_reference_zero_returns_none(self) -> None:
        receivers = [
            (0x01, complex(-0.5, 0.25)),
            (0x02, complex(9.0, 9.0)),
            (0x13, complex(0.0, 0.0)),   # stage0 reference = 0
            (0x21, complex(7.0, 7.0)),
            (0x22, complex(0.6, -0.3)),
            (0x33, complex(2.0, 0.0)),
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_stage1_reference_zero_returns_none(self) -> None:
        receivers = [
            (0x01, complex(-0.5, 0.25)),
            (0x02, complex(9.0, 9.0)),
            (0x13, complex(1.0, 0.0)),
            (0x21, complex(7.0, 7.0)),
            (0x22, complex(0.6, -0.3)),
            (0x33, complex(0.0, 0.0)),   # stage1 reference = 0
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_stage0_reference_inf_returns_none(self) -> None:
        # phase-1 spec: reference 0 / NaN / Inf are all invalid.  Unlike the
        # reference implementation (which accepts inf and yields finite 0), the
        # dual parser rejects a non-finite reference outright.
        receivers = [
            (0x01, complex(-0.5, 0.25)),
            (0x02, complex(9.0, 9.0)),
            (0x13, complex(float("inf"), 0.0)),   # stage0 reference = inf
            (0x21, complex(7.0, 7.0)),
            (0x22, complex(0.6, -0.3)),
            (0x33, complex(2.0, 0.0)),
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_stage1_reference_inf_returns_none(self) -> None:
        receivers = [
            (0x01, complex(-0.5, 0.25)),
            (0x02, complex(9.0, 9.0)),
            (0x13, complex(1.0, 0.0)),
            (0x21, complex(7.0, 7.0)),
            (0x22, complex(0.6, -0.3)),
            (0x33, complex(float("inf"), 0.0)),   # stage1 reference = inf
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_stage0_reference_nan_returns_none(self) -> None:
        receivers = [
            (0x01, complex(-0.5, 0.25)),
            (0x02, complex(9.0, 9.0)),
            (0x13, complex(float("nan"), 0.0)),   # stage0 reference = NaN
            (0x21, complex(7.0, 7.0)),
            (0x22, complex(0.6, -0.3)),
            (0x33, complex(2.0, 0.0)),
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_stage1_reference_nan_returns_none(self) -> None:
        receivers = [
            (0x01, complex(-0.5, 0.25)),
            (0x02, complex(9.0, 9.0)),
            (0x13, complex(1.0, 0.0)),
            (0x21, complex(7.0, 7.0)),
            (0x22, complex(0.6, -0.3)),
            (0x33, complex(float("nan"), 0.0)),   # stage1 reference = NaN
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_s11_non_finite_returns_none(self) -> None:
        receivers = [
            (0x01, complex(float("inf"), 0.0)),
            (0x02, complex(0.0, 0.0)),
            (0x13, complex(1.0, 0.0)),
            (0x21, complex(0.0, 0.0)),
            (0x22, complex(0.6, -0.3)),
            (0x33, complex(2.0, 0.0)),
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_s22_non_finite_returns_none(self) -> None:
        receivers = [
            (0x01, complex(-0.5, 0.25)),
            (0x02, complex(0.0, 0.0)),
            (0x13, complex(1.0, 0.0)),
            (0x21, complex(0.0, 0.0)),
            (0x22, complex(float("inf"), 0.0)),
            (0x33, complex(2.0, 0.0)),
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_duplicate_required_descriptor_returns_none(self) -> None:
        # two stage0 references (same desc 0x13, different values) -> not
        # silently first/last -> None
        receivers = [
            (0x01, complex(-0.5, 0.25)),
            (0x13, complex(1.0, 0.0)),     # stage0 ref #1
            (0x13, complex(2.0, 0.0)),     # stage0 ref #2 (duplicate)
            (0x02, complex(0.0, 0.0)),
            (0x22, complex(0.6, -0.3)),
            (0x33, complex(2.0, 0.0)),
        ]
        self.assertIsNone(parse_s11_s22_point(_dual_payload(receivers)))

    def test_unrelated_stage_ignored(self) -> None:
        # an extra stage-2 receiver (0x41 = P1, stage 2) is ignored
        receivers = _full_two_port_receivers() + [(0x41, complex(-99.0, 99.0))]
        result = parse_s11_s22_point(_dual_payload(receivers))
        self.assertIsNotNone(result)
        _, _, s11, s22 = result  # type: ignore[misc]
        self.assertAlmostEqual(s11.real, -0.5, places=6)
        self.assertAlmostEqual(s11.imag, 0.25, places=6)
        self.assertAlmostEqual(s22.real, 0.3, places=6)
        self.assertAlmostEqual(s22.imag, -0.15, places=6)

    def test_point_number_and_frequency_preserved(self) -> None:
        payload = _dual_payload(
            _full_two_port_receivers(),
            freq_hz=1_500_000_000,
            point_number=42,
        )
        result = parse_s11_s22_point(payload)
        self.assertIsNotNone(result)
        point_number, freq, _s11, _s22 = result  # type: ignore[misc]
        self.assertEqual(point_number, 42)
        self.assertEqual(freq, 1_500_000_000)

    def test_truncated_payload_raises_struct_error(self) -> None:
        # shorter than the 12-byte header -> struct.error (backend wraps it)
        with self.assertRaises(struct.error):
            parse_s11_s22_point(b"\x00" * 8)

    def test_stage1_absent_new_rejects_old_s11_succeeds(self) -> None:
        # stage 0 only (S11 inputs complete, no stage 1): new dual parser
        # rejects; old S11 parser still succeeds.
        receivers = [
            (0x01, complex(-0.5, 0.25)),   # stage0 Port1
            (0x13, complex(1.0, 0.0)),      # stage0 Reference
        ]
        payload = _dual_payload(receivers)
        self.assertIsNone(parse_s11_s22_point(payload))
        old = parse_s11_point(payload)
        self.assertIsNotNone(old)
        _point, _freq, s11 = old  # type: ignore[misc]
        self.assertAlmostEqual(s11.real, -0.5, places=6)
        self.assertAlmostEqual(s11.imag, 0.25, places=6)

    def test_does_not_modify_input_bytes(self) -> None:
        payload = _dual_payload(_full_two_port_receivers())
        before = bytes(payload)
        parse_s11_s22_point(payload)
        self.assertEqual(payload, before)

    def test_sweep_settings_stages_bitmap_decodes_to_single_stage(self) -> None:
        # the production fixed vector's stages field at offset 25 (uint16 LE).
        # Hand-computed: stages_bitmap=(1<<6)|(1<<9)|(1<<12)=0x1240.
        payload = bytes.fromhex(SWEEP_SETTINGS_HEX)
        self.assertEqual(
            encode_sweep_settings(500_000_000, 2_500_000_000, 101, 10_000, -10.0, 0),
            payload,
        )
        stages_bitmap = struct.unpack_from("<H", payload, 25)[0]
        self.assertEqual(stages_bitmap, 0x1240)
        # field decode (bits2-0=Stages-1, bits5-3=P1, bits8-6=P2)
        stages_count = (stages_bitmap & 0b111) + 1
        p1_stage = (stages_bitmap >> 3) & 0b111
        p2_stage = (stages_bitmap >> 6) & 0b111
        self.assertEqual(stages_count, 1)
        self.assertEqual(p1_stage, 0)
        self.assertEqual(p2_stage, 1)  # placeholder: not executed when Stages=0
        two_port = stages_bitmap | 0b001
        self.assertEqual((two_port & 0b111) + 1, 2)

    def test_protocol_module_introduces_no_new_dependency_or_hardware(self) -> None:
        import librevna.librevna_protocol as proto

        self.assertTrue(callable(proto.parse_s11_s22_point))
        # parse the AST of the real import statements (ignores literal text in
        # docstrings/comments).
        tree = ast.parse(inspect.getsource(proto))
        imported_roots: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_roots.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_roots.add(node.module.split(".")[0])
        # protocol layer stays pure Python: no usb / numpy / libusb / Qt
        self.assertNotIn("usb", imported_roots)
        self.assertNotIn("numpy", imported_roots)
        self.assertNotIn("libusb_package", imported_roots)
        self.assertNotIn("PySide6", imported_roots)
        # actually only the standard library
        self.assertTrue(
            imported_roots <= {"__future__", "binascii", "math", "struct", "dataclasses"}
        )


if __name__ == "__main__":
    unittest.main()
