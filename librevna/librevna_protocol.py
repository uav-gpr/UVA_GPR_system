"""Pure LibreVNA device-protocol logic (standard library only, no PyUSB/Qt).

This module is the protocol layer of the persistent continuous-acquisition
effort (see ``docs/LibreVNA连续采集功能开发路线图.md``, phase 1).  It is
adapted from the field-validated protocol module of the read-only reference
project ``E:\\钢筋仪软件开发\\src\\rebar_inspector\\acquisition\\librevna_protocol.py``,
**not rewritten from memory**.  Behaviour is kept byte-for-byte aligned with
that reference and with the historical probe code, including:

- ``VNA_DATAPOINT`` (type 27) **skips CRC validation** -- this is the
  established reference behaviour for datapoint CRC; without device evidence
  and a side-by-side test we must not "fix" it.
- ``PacketStream.feed`` sticky/glue-packet boundaries and illegal-length /
  CRC handling match the reference exactly.
- ``DeviceInfo`` decoding pads the payload with ``b"\\x00\\x00"`` before
  unpacking the fixed layout (matches the reference).

Two deliberate divergences exist, and both are **input-validation semantics
only, never fixed bytes**:

1. :func:`parse_s11_s22_point` rejects a non-finite (NaN/Inf) reference
   outright, per the phase-1 spec ("reference 0 / NaN / Inf invalid").  The
   reference implementation accepts an ``inf`` reference and yields a finite
   0 S11/S22.
2. :func:`parse_vna_datapoint` rejects a payload whose receiver section length
   is not divisible by 9 (``(len - 12) % 9 != 0``) with ``struct.error``; the
   reference implementation floor-divides and silently ignores 1..8 trailing
   bytes.  This strictness is required by the roadmap's malformed-payload
   rejection, so a truncated datapoint can never be patched into a sweep.

All fixed packet bytes, field layouts and legal-payload parsing remain
byte-identical / behaviour-identical to the reference.

Stages bitmaps ``0x1240`` and ``0x1241`` were ACK-confirmed on real hardware
(LibreVNA firmware 1.6.5 / protocol 14 / HW 1/B, 2026-08-05).  Production
code only sends these validated values.

This module is pure protocol: it must never ``import usb`` / ``libusb_package``
/ ``numpy`` / Qt.  USB transport belongs in ``librevna_transport`` and the
continuous acquisition state machine in ``librevna_continuous``.
"""

from __future__ import annotations

import binascii
import math
import struct
from dataclasses import dataclass

# ---- device constants (match the reference) ----

VID = 0x1209
PID = 0x4121
EP_OUT = 0x01
EP_IN = 0x81

HEADER = 0x5A

# ---- packet types ----

SWEEP_SETTINGS = 2
DEVICE_INFO = 5
ACK = 7
NACK = 10
REQUEST_DEVICE_INFO = 15
SET_IDLE = 20
DEVICE_STATUS = 25
VNA_DATAPOINT = 27

#: ``DeviceInfo`` decode layout (protocol v14): field-by-field struct format.
_DEVICE_INFO_FORMAT = "<HBBBBcQQIIHhhIIBQBH"

#: ``SweepSettings`` encode layout.
_SWEEP_SETTINGS_FORMAT = "<QQHIhBHhH"

#: ``VNADatapoint`` header: frequency(8) + cdbm(2) + point_number(2).
_VNA_DATAPOINT_HEADER = "<QhH"

#: ``VNADatapoint`` per-receiver group size: real(4) + imag(4) + desc(1).
_VNA_DATAPOINT_GROUP_SIZE = 9

#: ``VNADatapoint`` fixed header length.
_VNA_DATAPOINT_HEADER_SIZE = 12

# ---- VNADatapoint descriptor bitmasks ----
# Source: LibreVNA ``Device_protocol_v13.tex`` "Data description bitmask" and
# the production S11 parsing (``datapoint_to_s11`` / ``parse_s11_point``).
# descriptor byte layout:
#   bits 7-5 = stage (stage 0 = Port-1 excitation, stage 1 = Port-2 excitation)
#   bit 4    = reference-receiver flag
#   bit 3    = Port 4 receiver
#   bit 2    = Port 3 receiver
#   bit 1    = Port 2 receiver   <- S22 input (stage 1)
#   bit 0    = Port 1 receiver   <- S11 input (stage 0)
# The two-port example masks from the doc: 0x01(P1,s0) 0x02(P2,s0)
# 0x13(Ref+P1+P2,s0) 0x21(P1,s1) 0x22(P2,s1) 0x33(Ref+P1+P2,s1).  A reference
# descriptor may also carry port bits.
_DESC_STAGE_SHIFT = 5
_DESC_MASK_REFERENCE = 0x10
_DESC_MASK_PORT1 = 0x01
_DESC_MASK_PORT2 = 0x02

# ---- SweepSettings stages_bitmap (protocol 14, confirmed on hardware) ----
# The stages field (uint16 LE at payload offset 25).  Field decode
# (Device_protocol_v13.tex): bits2-0 = Stages-1, bits5-3 = stage exciting
# port 1, bits8-6 = stage exciting port 2; remaining high bits keep the fixed
# production receiver-enable vector.
#
# 0x1240 = (1<<6)|(1<<9)|(1<<12): Stages=0 (1 stage), excites port 1 / stage 0
#   only -> single-reflection S11.
# 0x1241 = 0x1240 | (1<<0): Stages=1 (2 stages), port 1 stage 0 + port 2
#   stage 1 -> dual-reflection S11+S22.
# Both were ACK-confirmed on hardware (LibreVNA 1.6.5 / protocol 14 / HW 1/B,
# 2026-08-05).  Production code must not send unvalidated combinations.

#: Single-reflection (S11) stages_bitmap: Stages=0, port 1 / stage 0 only.
S11_STAGES_BITMAP = 0x1240
#: Dual-reflection (S11+S22) stages_bitmap: Stages=1, port 1 stage 0 + port 2 stage 1.
S11_S22_STAGES_BITMAP = 0x1241
#: Hardware-validated stages_bitmaps only; extending this set is forbidden.
ALLOWED_STAGES_BITMAPS: tuple[int, ...] = (S11_STAGES_BITMAP, S11_S22_STAGES_BITMAP)


def _validate_stages_bitmap(stages_bitmap: int) -> None:
    """Validate stages_bitmap: int (not bool), uint16 range, and a validated value.

    Production code only sends hardware-validated stages_bitmaps (see
    :data:`ALLOWED_STAGES_BITMAPS`); bool/float/str, out-of-range ints and
    unvalidated in-range values (e.g. ``0x1242``) are all rejected.
    """
    if isinstance(stages_bitmap, bool) or not isinstance(stages_bitmap, int):
        raise TypeError(
            f"stages_bitmap must be int, got {type(stages_bitmap).__name__}"
        )
    if stages_bitmap < 0 or stages_bitmap > 0xFFFF:
        raise ValueError(
            f"stages_bitmap must be within 0..0xFFFF, got {stages_bitmap:#x}"
        )
    if stages_bitmap not in ALLOWED_STAGES_BITMAPS:
        raise ValueError(
            f"stages_bitmap must be one of the hardware-validated values "
            f"{ALLOWED_STAGES_BITMAPS}, got {stages_bitmap:#x}"
        )


# ---- packet structures ----

@dataclass(frozen=True)
class Packet:
    """One decoded protocol packet."""

    packet_type: int
    payload: bytes


def crc32(data: bytes) -> int:
    """CRC32 (IEEE 802.3, matches zlib), lower 32 bits only."""
    return binascii.crc32(data) & 0xFFFFFFFF


def encode_packet(packet_type: int, payload: bytes = b"") -> bytes:
    """Encode one packet: HEADER + length + type + payload + CRC32.

    length covers the whole packet: HEADER(1) + length(2) + type(1) +
    payload + CRC32(4).
    """
    length = 1 + 2 + 1 + len(payload) + 4
    body = struct.pack("<BHB", HEADER, length, packet_type) + payload
    return body + struct.pack("<I", crc32(body))


class PacketStream:
    """Sticky de-glue-packet splitter that keeps a buffer across reads.

    ``feed(data)`` appends new bytes to the internal buffer, returns complete
    packets and keeps any trailing half-packet for later data.  Behaviour is
    identical to the reference ``PacketStream``:

    - noise bytes before ``HEADER`` are dropped;
    - out-of-range lengths (< 8 or > 4096) drop the current byte and realign;
    - incomplete data waits for more bytes;
    - non-``VNA_DATAPOINT`` packets have their CRC32 validated; ``VNA_DATAPOINT``
      (type 27) skips CRC validation (established reference behaviour).
    """

    def __init__(self) -> None:
        self.buffer = bytearray()

    def reset(self) -> None:
        """Clear the internal buffer (used at open/configure session boundaries)."""
        self.buffer.clear()

    def feed(self, data: bytes) -> list[Packet]:
        self.buffer.extend(data)
        packets: list[Packet] = []
        while True:
            while self.buffer and self.buffer[0] != HEADER:
                del self.buffer[0]
            if len(self.buffer) < 4:
                return packets
            length = struct.unpack_from("<H", self.buffer, 1)[0]
            if length < 8 or length > 4096:
                del self.buffer[0]
                continue
            if len(self.buffer) < length:
                return packets
            raw = bytes(self.buffer[:length])
            del self.buffer[:length]
            packet_type = raw[3]
            expected_crc = struct.unpack_from("<I", raw, length - 4)[0]
            if packet_type != VNA_DATAPOINT and expected_crc != crc32(raw[:-4]):
                continue
            packets.append(Packet(packet_type, raw[4:-4]))


# ---- DeviceInfo (protocol v14) ----

@dataclass(frozen=True)
class LibreVnaDeviceInfo:
    """RequestDeviceInfo result summary (protocol v14 fields)."""

    protocol: int
    firmware: str
    hardware_version: int
    hardware_revision: str
    min_freq_hz: int
    max_freq_hz: int
    min_ifbw_hz: int
    max_ifbw_hz: int
    max_points: int
    min_dbm: float
    max_dbm: float
    min_rbw_hz: int
    max_rbw_hz: int
    max_amplitude_points: int
    max_harmonic_freq_hz: int
    num_ports: int
    max_dwell_time_us: int

    def to_extras_dict(self) -> dict[str, object]:
        """JSON-serializable summary (all values finite / serializable)."""
        return {
            "source": "librevna_usb",
            "firmware": self.firmware,
            "protocol": self.protocol,
            "hardware_version": self.hardware_version,
            "hardware_revision": self.hardware_revision,
            "min_freq_hz": self.min_freq_hz,
            "max_freq_hz": self.max_freq_hz,
            "min_ifbw_hz": self.min_ifbw_hz,
            "max_ifbw_hz": self.max_ifbw_hz,
            "max_points": self.max_points,
            "min_dbm": self.min_dbm,
            "max_dbm": self.max_dbm,
            "num_ports": self.num_ports,
            "max_dwell_time_us": self.max_dwell_time_us,
        }

    @property
    def summary(self) -> str:
        """One-line device summary (for the UI)."""
        return (
            f"LibreVNA FW {self.firmware} · protocol v{self.protocol} · "
            f"HW rev {self.hardware_revision} · "
            f"{self.min_freq_hz / 1e9:g}–{self.max_freq_hz / 1e9:g} GHz · "
            f"≤{self.max_points} points · {self.num_ports} ports"
        )


def decode_device_info(payload: bytes) -> LibreVnaDeviceInfo:
    """Decode a protocol v14 DeviceInfo payload (field-aligned with reference).

    A payload shorter than 55 bytes is padded with 2 bytes before unpacking
    (matching the reference); anything shorter still raises ``struct.error``,
    which the caller should wrap as a protocol error.
    """
    fields = struct.unpack_from(_DEVICE_INFO_FORMAT, payload + b"\x00\x00", 0)
    return LibreVnaDeviceInfo(
        protocol=fields[0],
        firmware=f"{fields[1]}.{fields[2]}.{fields[3]}",
        hardware_version=fields[4],
        hardware_revision=fields[5].decode(errors="replace"),
        min_freq_hz=fields[6],
        max_freq_hz=fields[7],
        min_ifbw_hz=fields[8],
        max_ifbw_hz=fields[9],
        max_points=fields[10],
        min_dbm=fields[11] / 100.0,
        max_dbm=fields[12] / 100.0,
        min_rbw_hz=fields[13],
        max_rbw_hz=fields[14],
        max_amplitude_points=fields[15],
        max_harmonic_freq_hz=fields[16],
        num_ports=fields[17],
        max_dwell_time_us=fields[18],
    )


# ---- SweepSettings ----

@dataclass(frozen=True)
class SweepSettings:
    """One sweep configuration (encoded as a protocol v14 SweepSettings payload).

    ``power_dbm`` / ``dwell_us`` are fixed backend settings for this stage of
    the LibreVNA work (default -10 dBm / 0 us) and are not part of a wider
    public sweep-config model.

    ``stages_bitmap`` selects the excitation stage combination: default
    :data:`S11_STAGES_BITMAP` (single-reflection S11); dual-reflection S11+S22
    uses :data:`S11_S22_STAGES_BITMAP`.  Only :data:`ALLOWED_STAGES_BITMAPS`
    values are accepted.
    """

    start_hz: int
    stop_hz: int
    points: int
    ifbw_hz: int
    power_dbm: float = -10.0
    dwell_us: int = 0
    stages_bitmap: int = S11_STAGES_BITMAP

    def __post_init__(self) -> None:
        if not isinstance(self.start_hz, int) or isinstance(self.start_hz, bool):
            raise TypeError("start_hz must be int")
        if not isinstance(self.stop_hz, int) or isinstance(self.stop_hz, bool):
            raise TypeError("stop_hz must be int")
        if not isinstance(self.points, int) or isinstance(self.points, bool):
            raise TypeError("points must be int")
        if not isinstance(self.ifbw_hz, int) or isinstance(self.ifbw_hz, bool):
            raise TypeError("ifbw_hz must be int")
        if not isinstance(self.dwell_us, int) or isinstance(self.dwell_us, bool):
            raise TypeError("dwell_us must be int")
        if self.start_hz <= 0 or self.stop_hz <= self.start_hz:
            raise ValueError("SweepSettings start/stop frequencies are invalid")
        if self.points < 2:
            raise ValueError("SweepSettings points must be >= 2")
        if self.ifbw_hz <= 0:
            raise ValueError("SweepSettings ifbw_hz must be positive")
        if not math.isfinite(self.power_dbm):
            raise ValueError("SweepSettings power_dbm must be finite")
        _validate_stages_bitmap(self.stages_bitmap)

    def encode(self) -> bytes:
        """Encode to a SweepSettings payload (identical to reference bytes)."""
        return encode_sweep_settings(
            self.start_hz,
            self.stop_hz,
            self.points,
            self.ifbw_hz,
            self.power_dbm,
            self.dwell_us,
            stages_bitmap=self.stages_bitmap,
        )


def encode_sweep_settings(
    start_hz: int,
    stop_hz: int,
    points: int,
    ifbw_hz: int,
    power_dbm: float,
    dwell_us: int = 0,
    *,
    stages_bitmap: int = S11_STAGES_BITMAP,
) -> bytes:
    """Encode SweepSettings (protocol v14, byte-identical to reference).

    - ``config`` fixed to ``(1<<2) | (1<<3)``: suppress peaks + fixed power;
    - ``stages_bitmap`` validated via :func:`_validate_stages_bitmap`, default
      :data:`S11_STAGES_BITMAP` (0x1240, single-reflection S11); dual-reflection
      S11+S22 passes :data:`S11_S22_STAGES_BITMAP` (0x1241).  The stages field
      sits at offset 25 (uint16 LE).

    Omitting ``stages_bitmap`` produces output byte-identical to the historic
    fixed vector (backwards compatible).
    """
    cdbm = int(round(power_dbm * 100))
    config = (1 << 2) | (1 << 3)  # suppress peaks, fixed power
    _validate_stages_bitmap(stages_bitmap)
    return struct.pack(
        _SWEEP_SETTINGS_FORMAT,
        int(start_hz),
        int(stop_hz),
        int(points),
        int(ifbw_hz),
        cdbm,
        config,
        int(stages_bitmap),
        cdbm,
        int(dwell_us),
    )


# ---- VNADatapoint parsing and S11/S22 ----

def parse_vna_datapoint(payload: bytes) -> tuple[int, int, int, list[tuple[int, complex]]]:
    """Parse a VNADatapoint payload into ``(frequency, cdbm, point_number, values)``.

    ``values`` is a ``(desc, complex)`` list; ``desc`` describes the receiver
    (stage / port / reference).

    The payload must be exactly ``12 + 9*N`` bytes (a fixed 12-byte header plus
    N receiver groups of real(4) + imag(4) + desc(1)).  A payload shorter than
    the header raises ``struct.error`` (from the header unpack), and a receiver
    section whose length is not divisible by 9 also raises ``struct.error`` --
    trailing 1..8 stray bytes are never silently dropped (deliberate
    validation-semantics difference from the reference, see module docstring).
    A legal ``12``-byte payload parses to ``values = []``.
    """
    frequency, cdbm, point_number = struct.unpack_from(_VNA_DATAPOINT_HEADER, payload, 0)
    remaining = len(payload) - _VNA_DATAPOINT_HEADER_SIZE
    if remaining % _VNA_DATAPOINT_GROUP_SIZE != 0:
        raise struct.error(
            "VNADatapoint payload receiver section length must be divisible by "
            f"{_VNA_DATAPOINT_GROUP_SIZE}; payload is {len(payload)} bytes"
        )
    count = remaining // _VNA_DATAPOINT_GROUP_SIZE
    reals = struct.unpack_from("<" + "f" * count, payload, _VNA_DATAPOINT_HEADER_SIZE)
    imag_offset = _VNA_DATAPOINT_HEADER_SIZE + 4 * count
    imags = struct.unpack_from("<" + "f" * count, payload, imag_offset)
    desc_offset = imag_offset + 4 * count
    desc = payload[desc_offset : desc_offset + count]
    values = [(desc[i], complex(reals[i], imags[i])) for i in range(count)]
    return frequency, cdbm, point_number, values


def datapoint_to_s11(payload: bytes) -> tuple[float, complex] | None:
    """Compute raw S11 = stage-0 Port-1 receiver / stage-0 Reference receiver.

    Returns ``None`` when stage 0 lacks a Port-1 or Reference value, or the
    reference magnitude is 0 (matching the reference ``datapoint_to_s11``; a
    zero reference is treated as an invalid point by the caller).
    """
    frequency, _cdbm, _point_number, values = parse_vna_datapoint(payload)
    input_value = None
    ref_value = None
    for desc, value in values:
        stage = desc >> 5
        is_ref = bool(desc & 0x10)
        is_port1 = bool(desc & 0x01)
        if stage != 0:
            continue
        if is_ref:
            ref_value = value
        elif is_port1:
            input_value = value
    if input_value is None or ref_value is None or abs(ref_value) == 0:
        return None
    return float(frequency), input_value / ref_value


def parse_s11_point(
    payload: bytes,
) -> tuple[int, float, complex] | None:
    """Parse one VNADatapoint as ``(point_number, frequency_hz, s11)``.

    Unlike ``datapoint_to_s11`` this also requires S11 to be finite; a zero
    reference, a missing Port-1/Reference, or NaN/Infinity S11 returns ``None``
    (the caller discards the "unusable" point).  Structurally broken payloads
    raise ``struct.error``.
    """
    frequency, _cdbm, point_number, values = parse_vna_datapoint(payload)
    input_value = None
    ref_value = None
    for desc, value in values:
        stage = desc >> 5
        is_ref = bool(desc & 0x10)
        is_port1 = bool(desc & 0x01)
        if stage != 0:
            continue
        if is_ref:
            ref_value = value
        elif is_port1:
            input_value = value
    if input_value is None or ref_value is None or abs(ref_value) == 0:
        return None
    s11 = input_value / ref_value
    if not (math.isfinite(s11.real) and math.isfinite(s11.imag)):
        return None
    return point_number, float(frequency), complex(s11)


def parse_s11_s22_point(
    payload: bytes,
) -> tuple[int, float, complex, complex] | None:
    """Parse one VNADatapoint as ``(point_number, frequency_hz, s11, s22)``.

    Builds S11 and S22 **within a single payload** (no cross-packet caching,
    no cross-sweep stitching):

    - S11 = stage-0 Port-1 receiver / stage-0 reference;
    - S22 = stage-1 Port-2 receiver / stage-1 reference.

    Descriptor bitmasks are :data:`_DESC_MASK_REFERENCE` /
    :data:`_DESC_MASK_PORT1` / :data:`_DESC_MASK_PORT2` (from the LibreVNA
    protocol doc and production S11 parsing).  Receiver ordering inside the
    payload does not matter (lookup is by descriptor bits, not array position).

    Returns ``None`` (no exception) in any of these cases:

    - any required slot is missing (stage-0 reference / stage-0 Port1 /
      stage-1 reference / stage-1 Port2), including an entirely absent stage 1;
    - a required slot appears more than once (duplicate descriptors with
      possibly conflicting values; we do **not** silently take first/last);
    - either reference magnitude is 0 **or** either reference is non-finite
      (NaN/Inf) -- the phase-1 spec rejects a reference of 0 / NaN / Inf;
    - S11 or S22 has a non-finite real/imag part.

    Structurally broken/truncated payloads continue to raise
    :class:`struct.error` (the backend wraps it as a protocol error).  Unrelated
    receivers are ignored: stage-0 Port2 (S21 input), stage-1 Port1 (S12 input),
    and any stage >= 2 receiver never affect the result.

    .. note::
       This function requires all four S11/S22 inputs to live in the **same**
       VNADatapoint payload.  ``Device_protocol_v13.tex`` states that each
       completed frequency point produces one VNADatapoint and a point can span
       multiple stages, so the "stage 0/1 in the same payload" contract holds.
       It was confirmed on hardware (2026-08-05, protocol 14): under ``0x1241``
       each point is exactly one payload with 6 receivers (``0x01 0x02 0x13 0x21
       0x22 0x33``, stages {0,1}) and ``parse_s11_s22_point`` succeeded for all
       303 real points.  If a future firmware/protocol splits stage 0 and
       stage 1 into separate packets, this function returns ``None`` for each
       packet; cross-packet pairing belongs to a later
       ``ContinuousSweepAssembler`` concern and must not be guessed here.

    .. note::
       This function does not change the public behaviour of
       ``parse_vna_datapoint`` / ``datapoint_to_s11`` / ``parse_s11_point``;
       the old S11 parsing still works when stage 1 is absent.  This round only
       provides the protocol parsing capability, not a claim that the production
       backend supports S22 yet.
    """
    frequency, _cdbm, point_number, values = parse_vna_datapoint(payload)
    s11_port1: complex | None = None
    s11_ref: complex | None = None
    s22_port2: complex | None = None
    s22_ref: complex | None = None
    s11_port1_count = 0
    s11_ref_count = 0
    s22_port2_count = 0
    s22_ref_count = 0
    for desc, value in values:
        stage = desc >> _DESC_STAGE_SHIFT
        is_ref = bool(desc & _DESC_MASK_REFERENCE)
        if stage == 0:
            if is_ref:
                s11_ref = value
                s11_ref_count += 1
            elif desc & _DESC_MASK_PORT1:
                s11_port1 = value
                s11_port1_count += 1
        elif stage == 1:
            if is_ref:
                s22_ref = value
                s22_ref_count += 1
            elif desc & _DESC_MASK_PORT2:
                s22_port2 = value
                s22_port2_count += 1
    # Each required slot must occur exactly once; missing or duplicate -> None.
    if (
        s11_port1_count != 1
        or s11_ref_count != 1
        or s22_port2_count != 1
        or s22_ref_count != 1
    ):
        return None
    assert s11_port1 is not None and s11_ref is not None
    assert s22_port2 is not None and s22_ref is not None
    if abs(s11_ref) == 0 or abs(s22_ref) == 0:
        return None
    # A non-finite reference (NaN/Inf) is also invalid: phase-1 spec rejects
    # reference 0 / NaN / Inf outright.  This is deliberately stricter than the
    # reference implementation (which would accept an inf reference and produce a
    # finite 0); it is a validation-semantics requirement, not a fixed-byte
    # difference, and it is checked here explicitly so the intent is visible.
    if not (math.isfinite(s11_ref.real) and math.isfinite(s11_ref.imag)):
        return None
    if not (math.isfinite(s22_ref.real) and math.isfinite(s22_ref.imag)):
        return None
    s11 = s11_port1 / s11_ref
    s22 = s22_port2 / s22_ref
    if not (math.isfinite(s11.real) and math.isfinite(s11.imag)):
        return None
    if not (math.isfinite(s22.real) and math.isfinite(s22.imag)):
        return None
    return point_number, float(frequency), complex(s11), complex(s22)


__all__ = [
    "ACK",
    "ALLOWED_STAGES_BITMAPS",
    "DEVICE_INFO",
    "DEVICE_STATUS",
    "EP_IN",
    "EP_OUT",
    "HEADER",
    "LibreVnaDeviceInfo",
    "NACK",
    "PID",
    "Packet",
    "PacketStream",
    "REQUEST_DEVICE_INFO",
    "SET_IDLE",
    "S11_S22_STAGES_BITMAP",
    "S11_STAGES_BITMAP",
    "SWEEP_SETTINGS",
    "SweepSettings",
    "VNA_DATAPOINT",
    "VID",
    "crc32",
    "datapoint_to_s11",
    "decode_device_info",
    "encode_packet",
    "encode_sweep_settings",
    "parse_s11_point",
    "parse_s11_s22_point",
    "parse_vna_datapoint",
]
