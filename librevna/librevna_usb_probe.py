#!/usr/bin/env python3
"""Probe LibreVNA directly over USB, without LibreVNA-GUI.

This is the first step toward a headless LibreVNA backend. It only sends
RequestDeviceInfo and decodes Ack / DeviceInfo packets.
"""

from __future__ import annotations

import argparse
import binascii
import ctypes
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path


try:
    import libusb_package

    # Make PyUSB able to find the bundled libusb-1.0.dll on Windows.
    dll_dir = Path(libusb_package.__file__).resolve().parent
    LIBUSB_DLL = dll_dir / "libusb-1.0.dll"
    if sys.platform.startswith("win"):
        ctypes.WinDLL(str(LIBUSB_DLL))
except Exception:
    LIBUSB_DLL = None
    pass

import usb.core
import usb.util
import usb.backend.libusb1


VID = 0x1209
PID = 0x4121
EP_OUT = 0x01
EP_IN = 0x81
EP_DEBUG = 0x82

HEADER = 0x5A


class PacketType:
    ACK = 7
    NACK = 10
    REQUEST_DEVICE_INFO = 15
    DEVICE_INFO = 5


@dataclass
class Packet:
    packet_type: int
    payload: bytes


def crc32(data: bytes) -> int:
    return binascii.crc32(data) & 0xFFFFFFFF


def encode_packet(packet_type: int, payload: bytes = b"") -> bytes:
    length = 1 + 2 + 1 + len(payload) + 4
    body = struct.pack("<BHB", HEADER, length, packet_type) + payload
    return body + struct.pack("<I", crc32(body))


class PacketStream:
    def __init__(self) -> None:
        self.buffer = bytearray()

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
            if packet_type != 27 and expected_crc != crc32(raw[:-4]):
                continue
            packets.append(Packet(packet_type, raw[4:-4]))


def open_device():
    backend = None
    if LIBUSB_DLL:
        backend = usb.backend.libusb1.get_backend(find_library=lambda _: str(LIBUSB_DLL))
    dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
    if dev is None:
        raise RuntimeError("LibreVNA USB device not found (VID_1209 PID_4121).")

    try:
        dev.set_configuration()
    except usb.core.USBError as exc:
        # On Windows this can fail if the kernel/WinUSB configuration is already active.
        print(f"set_configuration warning: {exc}")

    cfg = dev.get_active_configuration()
    interface = cfg[(0, 0)]
    try:
        if dev.is_kernel_driver_active(interface.bInterfaceNumber):
            dev.detach_kernel_driver(interface.bInterfaceNumber)
    except (NotImplementedError, usb.core.USBError):
        pass
    try:
        usb.util.claim_interface(dev, interface.bInterfaceNumber)
    except usb.core.USBError as exc:
        print(f"claim_interface warning: {exc}")
    return dev, interface.bInterfaceNumber


def read_packets(dev, timeout_ms: int = 1000, total_ms: int = 3000) -> list[Packet]:
    stream = PacketStream()
    packets: list[Packet] = []
    deadline = time.monotonic() + total_ms / 1000.0
    while time.monotonic() < deadline:
        try:
            data = dev.read(EP_IN, 512, timeout=min(timeout_ms, 200))
        except usb.core.USBTimeoutError:
            continue
        except usb.core.USBError as exc:
            raise RuntimeError(f"USB read failed: {exc}") from exc
        packets.extend(stream.feed(bytes(data)))
    return packets


def decode_device_info(payload: bytes) -> dict[str, object]:
    # Protocol v14 adds max dwell time after the v13 fields.
    fields = struct.unpack_from("<HBBBBcQQIIHhhIIBQBH", payload + b"\x00\x00", 0)
    return {
        "protocol": fields[0],
        "firmware": f"{fields[1]}.{fields[2]}.{fields[3]}",
        "hardware_version": fields[4],
        "hardware_revision": fields[5].decode(errors="replace"),
        "min_freq_hz": fields[6],
        "max_freq_hz": fields[7],
        "min_ifbw_hz": fields[8],
        "max_ifbw_hz": fields[9],
        "max_points": fields[10],
        "min_dbm": fields[11] / 100.0,
        "max_dbm": fields[12] / 100.0,
        "min_rbw_hz": fields[13],
        "max_rbw_hz": fields[14],
        "max_amplitude_points": fields[15],
        "max_harmonic_freq_hz": fields[16],
        "num_ports": fields[17],
        "max_dwell_time_us": fields[18],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-ms", type=int, default=1000)
    args = parser.parse_args()

    dev, interface_number = open_device()
    try:
        packet = encode_packet(PacketType.REQUEST_DEVICE_INFO)
        written = dev.write(EP_OUT, packet, timeout=args.timeout_ms)
        print(f"wrote RequestDeviceInfo: {written} bytes")
        packets = read_packets(dev, timeout_ms=args.timeout_ms, total_ms=args.timeout_ms)
        if not packets:
            raise RuntimeError("No packet received from LibreVNA.")
        for p in packets:
            print(f"received packet type={p.packet_type} payload_len={len(p.payload)}")
            if p.packet_type == PacketType.DEVICE_INFO:
                for key, value in decode_device_info(p.payload).items():
                    print(f"  {key}: {value}")
    finally:
        try:
            usb.util.release_interface(dev, interface_number)
        except Exception:
            pass
        usb.util.dispose_resources(dev)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
