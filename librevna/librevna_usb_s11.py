#!/usr/bin/env python3
"""Acquire S11 from LibreVNA directly over USB, without LibreVNA-GUI.

How this skips LibreVNA-GUI
---------------------------
LibreVNA-GUI normally acts as a host-side controller: it opens the USB device,
sends device-protocol packets, receives VNADatapoint packets, then exposes a
TCP/SCPI server for external scripts. This file removes that middle layer.

The script talks to the hardware directly:

1. Open USB device VID_1209 / PID_4121 with PyUSB/libusb.
2. Send framed LibreVNA device-protocol packets to bulk OUT endpoint 0x01.
3. Read framed packets from bulk IN endpoint 0x81.
4. Send RequestDeviceInfo to verify firmware/protocol support.
5. Send SweepSettings to configure and start a VNA sweep.
6. Receive one VNADatapoint packet per frequency.
7. Compute raw S11 as Port1 receiver / Reference receiver for stage 0.
8. Optionally load a LibreVNA .cal file and apply one-port OSL correction.

So the chain is:

    Python -> USB bulk packets -> LibreVNA hardware

instead of:

    Python -> TCP/SCPI -> LibreVNA-GUI -> USB -> LibreVNA hardware
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt

from librevna_usb_probe import (
    EP_IN,
    EP_OUT,
    Packet,
    PacketStream,
    PacketType,
    decode_device_info,
    encode_packet,
    open_device,
)


@dataclass
class OSLPoint:
    frequency: float
    directivity: complex
    reflection_tracking: complex
    source_match: complex

    def correct(self, measured: complex) -> complex:
        m_minus_d = measured - self.directivity
        return m_minus_d / (self.reflection_tracking + self.source_match * m_minus_d)


class OSLCalibration:
    def __init__(self, points: list[OSLPoint], filename: Path):
        self.points = sorted(points, key=lambda p: p.frequency)
        self.filename = filename
        if not self.points:
            raise ValueError("Calibration contains no points.")

    @classmethod
    def from_file(cls, filename: str | Path) -> "OSLCalibration":
        path = Path(filename)
        data = json.loads(path.read_text(encoding="utf-8"))
        measurements = data.get("measurements", [])
        measured: dict[str, dict[float, complex]] = {}
        for measurement in measurements:
            mtype = measurement.get("type")
            points = measurement.get("data", {}).get("points", [])
            measured[mtype] = {
                float(p["frequency"]): complex(float(p["real"]), float(p["imag"]))
                for p in points
            }

        missing = [name for name in ("Open", "Short", "Load") if name not in measured]
        if missing:
            raise ValueError(f"Calibration file is missing measurements: {', '.join(missing)}")

        common_freqs = sorted(set(measured["Open"]) & set(measured["Short"]) & set(measured["Load"]))
        cal_points: list[OSLPoint] = []
        # The provided calkit uses ideal standards. If non-ideal standards are
        # needed later, this is the one place to evaluate their actual gamma(f).
        open_actual = complex(1.0, 0.0)
        short_actual = complex(-1.0, 0.0)
        load_actual = complex(0.0, 0.0)
        for freq in common_freqs:
            cal_points.append(
                compute_osl_point(
                    freq,
                    open_measured=measured["Open"][freq],
                    short_measured=measured["Short"][freq],
                    load_measured=measured["Load"][freq],
                    open_actual=open_actual,
                    short_actual=short_actual,
                    load_actual=load_actual,
                )
            )
        return cls(cal_points, path)

    def interpolate(self, frequency: float) -> OSLPoint:
        if frequency <= self.points[0].frequency:
            return self.points[0]
        if frequency >= self.points[-1].frequency:
            return self.points[-1]
        lo = 0
        hi = len(self.points) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.points[mid].frequency <= frequency:
                lo = mid
            else:
                hi = mid
        p0 = self.points[lo]
        p1 = self.points[hi]
        alpha = (frequency - p0.frequency) / (p1.frequency - p0.frequency)
        return OSLPoint(
            frequency=frequency,
            directivity=p0.directivity + alpha * (p1.directivity - p0.directivity),
            reflection_tracking=p0.reflection_tracking
            + alpha * (p1.reflection_tracking - p0.reflection_tracking),
            source_match=p0.source_match + alpha * (p1.source_match - p0.source_match),
        )

    def correct(self, frequency: float, measured: complex) -> complex:
        return self.interpolate(frequency).correct(measured)


def compute_osl_point(
    frequency: float,
    open_measured: complex,
    short_measured: complex,
    load_measured: complex,
    open_actual: complex,
    short_actual: complex,
    load_actual: complex,
) -> OSLPoint:
    lc = load_actual
    oc = open_actual
    sc = short_actual
    lm = load_measured
    om = open_measured
    sm = short_measured
    denom = lc * oc * (om - lm) + lc * sc * (lm - sm) + oc * sc * (sm - om)
    if abs(denom) == 0:
        raise ValueError(f"Invalid OSL calibration denominator at {frequency:g} Hz")
    directivity = (
        lc * om * (sm * (oc - sc) + lm * sc)
        - lc * oc * lm * sm
        + oc * lm * sc * (sm - om)
    ) / denom
    source_match = (lc * (om - sm) + oc * (sm - lm) + sc * (lm - om)) / denom
    delta = (lc * lm * (om - sm) + oc * om * (sm - lm) + sc * sm * (lm - om)) / denom
    reflection_tracking = directivity * source_match - delta
    return OSLPoint(frequency, directivity, reflection_tracking, source_match)


class Pkt:
    SWEEP_SETTINGS = 2
    ACK = 7
    NACK = 10
    SET_IDLE = 20
    DEVICE_STATUS = 25
    VNA_DATAPOINT = 27


def encode_sweep_settings(
    start_hz: int,
    stop_hz: int,
    points: int,
    ifbw_hz: int,
    power_dbm: float,
    dwell_us: int = 0,
    excited_ports: tuple[int, ...] = (1,),
) -> bytes:
    if not excited_ports:
        raise ValueError("At least one excited port is required")
    cdbm = int(round(power_dbm * 100))
    # Protocol v14 C++ bitfield order:
    # bit0 standby, bit1 syncMaster, bit2 suppressPeaks, bit3 fixedPower,
    # bit4 logSweep, bits5-6 syncMode, bit7 unused.
    config = (1 << 2) | (1 << 3)  # suppress peaks, fixed power
    port_stages = {}
    for stage, port in enumerate(excited_ports):
        port_stages[port] = stage
    inactive_stage = len(excited_ports)
    stages_bitmap = (
        ((len(excited_ports) - 1) & 0x07)
        | ((port_stages.get(1, inactive_stage) & 0x07) << 3)
        | ((port_stages.get(2, inactive_stage) & 0x07) << 6)
        | ((port_stages.get(3, inactive_stage) & 0x07) << 9)
        | ((port_stages.get(4, inactive_stage) & 0x07) << 12)
    )
    return struct.pack(
        "<QQHIhBHhH",
        int(start_hz),
        int(stop_hz),
        int(points),
        int(ifbw_hz),
        cdbm,
        config,
        stages_bitmap,
        cdbm,
        int(dwell_us),
    )


def read_usb_packets(dev, stream: PacketStream, timeout_ms: int = 200) -> list[Packet]:
    try:
        data = dev.read(EP_IN, 512, timeout=timeout_ms)
    except Exception as exc:
        if exc.__class__.__name__ == "USBTimeoutError":
            return []
        raise
    return stream.feed(bytes(data))


def request_device_info(dev, stream: PacketStream) -> dict[str, object]:
    dev.write(EP_OUT, encode_packet(PacketType.REQUEST_DEVICE_INFO), timeout=1000)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        for packet in read_usb_packets(dev, stream):
            if packet.packet_type == PacketType.DEVICE_INFO:
                return decode_device_info(packet.payload)
            if packet.packet_type == Pkt.NACK:
                raise RuntimeError("Device returned NACK to RequestDeviceInfo")
    raise RuntimeError("Timed out waiting for DeviceInfo")


def send_command(dev, stream: PacketStream, packet_type: int, payload: bytes = b"", timeout_s: float = 2.0) -> None:
    dev.write(EP_OUT, encode_packet(packet_type, payload), timeout=1000)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for packet in read_usb_packets(dev, stream):
            if packet.packet_type == Pkt.ACK:
                return
            if packet.packet_type == Pkt.NACK:
                raise RuntimeError(f"Device returned NACK to packet type {packet_type}")
    raise RuntimeError(f"Timed out waiting for ACK to packet type {packet_type}")


def parse_vna_datapoint(payload: bytes) -> tuple[int, int, int, list[tuple[int, complex]]]:
    frequency, cdbm, point_number = struct.unpack_from("<QhH", payload, 0)
    count = (len(payload) - 12) // 9
    reals = struct.unpack_from("<" + "f" * count, payload, 12)
    imag_offset = 12 + 4 * count
    imags = struct.unpack_from("<" + "f" * count, payload, imag_offset)
    desc_offset = imag_offset + 4 * count
    desc = payload[desc_offset : desc_offset + count]
    values = [(desc[i], complex(reals[i], imags[i])) for i in range(count)]
    return frequency, cdbm, point_number, values


def receiver_value(values: list[tuple[int, complex]], target_stage: int, port: int, reference: bool) -> complex | None:
    source_mask = 1 << (port - 1)
    if reference:
        source_mask |= 0x10
    for desc, value in values:
        stage = desc >> 5
        if stage != target_stage:
            continue
        if (desc & source_mask) == source_mask:
            if reference == bool(desc & 0x10):
                return value
    return None


def datapoint_to_sparameters(payload: bytes) -> tuple[float, dict[str, complex]] | None:
    frequency, _cdbm, _point_number, values = parse_vna_datapoint(payload)
    stage_by_source_port = {1: 0, 2: 1}
    result: dict[str, complex] = {}
    for source_port, stage in stage_by_source_port.items():
        ref_value = receiver_value(values, stage, source_port, True)
        if ref_value is None or abs(ref_value) == 0:
            continue
        for receiver_port in (1, 2):
            input_value = receiver_value(values, stage, receiver_port, False)
            if input_value is not None:
                result[f"S{receiver_port}{source_port}"] = input_value / ref_value
    if not result:
        return None
    return float(frequency), result


def datapoint_to_s11(payload: bytes) -> tuple[float, complex] | None:
    parsed = datapoint_to_sparameters(payload)
    if parsed is None:
        return None
    frequency, values = parsed
    if "S11" not in values:
        return None
    return frequency, values["S11"]


def acquire_s11(
    start_hz: int,
    stop_hz: int,
    points: int,
    ifbw_hz: int,
    power_dbm: float,
    dwell_us: int = 0,
) -> list[tuple[float, complex]]:
    dev, interface_number = open_device()
    stream = PacketStream()
    try:
        info = request_device_info(dev, stream)
        print(
            "Connected: "
            f"FW {info['firmware']} protocol {info['protocol']} "
            f"max {info['max_freq_hz'] / 1e9:g} GHz"
        )
        send_command(dev, stream, Pkt.SET_IDLE, timeout_s=2.0)
        payload = encode_sweep_settings(start_hz, stop_hz, points, ifbw_hz, power_dbm, dwell_us)
        send_command(dev, stream, Pkt.SWEEP_SETTINGS, payload, timeout_s=2.0)

        data: dict[int, tuple[float, complex]] = {}
        deadline = time.monotonic() + max(10.0, points * 0.05)
        while time.monotonic() < deadline and len(data) < points:
            for packet in read_usb_packets(dev, stream, timeout_ms=250):
                if packet.packet_type == Pkt.VNA_DATAPOINT:
                    parsed = datapoint_to_s11(packet.payload)
                    if parsed is not None:
                        freq, s11 = parsed
                        _frequency, _cdbm, point_number, _values = parse_vna_datapoint(packet.payload)
                        data[point_number] = (freq, s11)
                elif packet.packet_type == Pkt.NACK:
                    raise RuntimeError("Device returned NACK during sweep")
        send_command(dev, stream, Pkt.SET_IDLE, timeout_s=2.0)
        if len(data) < points:
            raise RuntimeError(f"Only received {len(data)} / {points} S11 points")
        return [data[i] for i in sorted(data)]
    finally:
        try:
            import usb.util

            usb.util.release_interface(dev, interface_number)
            usb.util.dispose_resources(dev)
        except Exception:
            pass


def acquire_sparameters(
    start_hz: int,
    stop_hz: int,
    points: int,
    ifbw_hz: int,
    power_dbm: float,
    dwell_us: int = 0,
) -> list[tuple[float, dict[str, complex]]]:
    dev, interface_number = open_device()
    stream = PacketStream()
    try:
        info = request_device_info(dev, stream)
        print(
            "Connected: "
            f"FW {info['firmware']} protocol {info['protocol']} "
            f"max {info['max_freq_hz'] / 1e9:g} GHz"
        )
        send_command(dev, stream, Pkt.SET_IDLE, timeout_s=2.0)
        payload = encode_sweep_settings(start_hz, stop_hz, points, ifbw_hz, power_dbm, dwell_us, excited_ports=(1, 2))
        send_command(dev, stream, Pkt.SWEEP_SETTINGS, payload, timeout_s=2.0)

        data: dict[int, tuple[float, dict[str, complex]]] = {}
        deadline = time.monotonic() + max(10.0, points * 0.1)
        required = {"S11", "S21", "S12", "S22"}
        while time.monotonic() < deadline and len(data) < points:
            for packet in read_usb_packets(dev, stream, timeout_ms=250):
                if packet.packet_type == Pkt.VNA_DATAPOINT:
                    parsed = datapoint_to_sparameters(packet.payload)
                    if parsed is not None:
                        freq, sparams = parsed
                        if required.issubset(sparams):
                            _frequency, _cdbm, point_number, _values = parse_vna_datapoint(packet.payload)
                            data[point_number] = (freq, sparams)
                elif packet.packet_type == Pkt.NACK:
                    raise RuntimeError("Device returned NACK during two-port sweep")
        send_command(dev, stream, Pkt.SET_IDLE, timeout_s=2.0)
        if len(data) < points:
            raise RuntimeError(f"Only received {len(data)} / {points} two-port S-parameter points")
        return [data[i] for i in sorted(data)]
    finally:
        try:
            import usb.util

            usb.util.release_interface(dev, interface_number)
            usb.util.dispose_resources(dev)
        except Exception:
            pass


def write_csv(path: Path, points: list[tuple[float, complex]]) -> None:
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frequency_hz", "frequency_mhz", "s11_real_raw", "s11_imag_raw", "s11_db_raw", "phase_deg_raw"])
        for frequency, s11 in points:
            mag = abs(s11)
            writer.writerow(
                [
                    f"{frequency:.12g}",
                    f"{frequency / 1e6:.9f}",
                    f"{s11.real:.12g}",
                    f"{s11.imag:.12g}",
                    f"{20 * math.log10(mag) if mag > 0 else float('-inf'):.9g}",
                    f"{math.degrees(math.atan2(s11.imag, s11.real)):.9g}",
                ]
            )


def write_calibrated_csv(path: Path, raw_points: list[tuple[float, complex]], cal_points: list[tuple[float, complex]]) -> None:
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "frequency_hz",
                "frequency_mhz",
                "s11_real_raw",
                "s11_imag_raw",
                "s11_db_raw",
                "s11_real_cal",
                "s11_imag_cal",
                "s11_db_cal",
                "phase_deg_cal",
            ]
        )
        for (frequency, raw), (_, calibrated) in zip(raw_points, cal_points):
            raw_mag = abs(raw)
            cal_mag = abs(calibrated)
            writer.writerow(
                [
                    f"{frequency:.12g}",
                    f"{frequency / 1e6:.9f}",
                    f"{raw.real:.12g}",
                    f"{raw.imag:.12g}",
                    f"{20 * math.log10(raw_mag) if raw_mag > 0 else float('-inf'):.9g}",
                    f"{calibrated.real:.12g}",
                    f"{calibrated.imag:.12g}",
                    f"{20 * math.log10(cal_mag) if cal_mag > 0 else float('-inf'):.9g}",
                    f"{math.degrees(math.atan2(calibrated.imag, calibrated.real)):.9g}",
                ]
            )


def plot_png(path: Path, points: list[tuple[float, complex]], title_prefix: str = "raw") -> None:
    freq_mhz = [frequency / 1e6 for frequency, _ in points]
    mag_db = [20 * math.log10(abs(s11)) if abs(s11) > 0 else float("-inf") for _, s11 in points]
    phase = [math.degrees(math.atan2(s11.imag, s11.real)) for _, s11 in points]
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6), dpi=140)
    ax1.plot(freq_mhz, mag_db)
    ax2.plot(freq_mhz, phase, color="#b44a00")
    ax1.set_ylabel(f"{title_prefix} |S11| (dB)")
    ax2.set_ylabel(f"{title_prefix} phase (deg)")
    ax2.set_xlabel("Frequency (MHz)")
    ax1.grid(True, alpha=0.3)
    ax2.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-mhz", type=float, default=1.0)
    parser.add_argument("--stop-mhz", type=float, default=6000.0)
    parser.add_argument("--points", type=int, default=501)
    parser.add_argument("--ifbw", type=int, default=1000)
    parser.add_argument("--power", type=float, default=-10.0)
    parser.add_argument("--dwell-us", type=int, default=0)
    parser.add_argument("--csv", default="s11_usb_raw.csv")
    parser.add_argument("--png", default="s11_usb_raw.png")
    parser.add_argument("--cal", default=None, help="LibreVNA .cal file for one-port OSL/SOLT correction")
    args = parser.parse_args()

    points = acquire_s11(
        int(args.start_mhz * 1e6),
        int(args.stop_mhz * 1e6),
        args.points,
        args.ifbw,
        args.power,
        args.dwell_us,
    )
    db = [20 * math.log10(abs(s11)) if abs(s11) > 0 else float("-inf") for _, s11 in points]
    min_i = min(range(len(points)), key=lambda i: db[i])
    print(f"received {len(points)} raw S11 points")
    print(f"frequency: {points[0][0] / 1e6:.6f} - {points[-1][0] / 1e6:.6f} MHz")
    print(f"raw S11 dB: {min(db):.3f} to {max(db):.3f} dB")
    print(f"min raw |S11|: {db[min_i]:.3f} dB @ {points[min_i][0] / 1e6:.6f} MHz")

    output_points = points
    title_prefix = "raw"
    if args.cal:
        calibration = OSLCalibration.from_file(args.cal)
        output_points = [(freq, calibration.correct(freq, raw)) for freq, raw in points]
        cal_db = [20 * math.log10(abs(s11)) if abs(s11) > 0 else float("-inf") for _, s11 in output_points]
        cal_min_i = min(range(len(output_points)), key=lambda i: cal_db[i])
        print(f"loaded calibration: {calibration.filename} ({len(calibration.points)} points)")
        print(f"calibrated S11 dB: {min(cal_db):.3f} to {max(cal_db):.3f} dB")
        print(
            f"min calibrated |S11|: {cal_db[cal_min_i]:.3f} dB "
            f"@ {output_points[cal_min_i][0] / 1e6:.6f} MHz"
        )
        write_calibrated_csv(Path(args.csv), points, output_points)
        title_prefix = "calibrated"
    else:
        write_csv(Path(args.csv), points)
    plot_png(Path(args.png), output_points, title_prefix=title_prefix)
    print(f"wrote {Path(args.csv).resolve()}")
    print(f"wrote {Path(args.png).resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
