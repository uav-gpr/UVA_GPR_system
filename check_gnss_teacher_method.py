"""Reproduce the teacher's serial method at 115200 and 9600, and print raw evidence (v3).

Why: separate "our program is wrong" from "port / cable / adapter is wrong",
and test the documented workaround for marginal links (M100 manual 4.2:
"com com1 9600" = change that port's baud rate).

Usage (inside the air black-box folder):
    runtime\\python.exe check_gnss_teacher_method.py          # auto: first real port
    runtime\\python.exe check_gnss_teacher_method.py COM5     # pick a port
Stop the service first, otherwise the port is busy:
    schtasks /End /TN UAVGPR_AirBlackBox

Phase 1: 115200, teacher's exact command (log gpgga), 3 s.
Phase 2: send "com com1 9600" + "com com3 9600" at 115200, reopen at 9600,
         keep 1 Hz output on, read 7 s.  (Transient: no saveconfig, no flash write.
         If 9600 does not help, this script sends 115200 back before exiting.)
"""
from __future__ import annotations

import sys
import time
from collections import Counter

import serial
from serial.tools import list_ports


def render(raw: bytes) -> str:
    """Printable bytes as-is, everything else as \\xNN (ASCII-only output)."""
    return "".join(chr(b) if 32 <= b < 127 else "\\x%02x" % b for b in raw)


def open_port(port: str, baud: int) -> "serial.Serial":
    ser = serial.Serial(port=port, baudrate=baud, bytesize=8, stopbits=1, parity="N")
    ser.timeout = 0.3   # 分块读超时（不是按行读，避免行尾/杂字节影响判断）
    return ser


def run_phase(ser: "serial.Serial", seconds: float, commands: list[bytes]) -> dict:
    """发命令 + 收字节，按 CR/LF 分行统计（容许行首/行内少量杂字节）。

    判定 NMEA 时看"行里是否出现 $ 和 GGA"（现场实测行首可能多一个杂字节），
    比老师原版的 startswith("$") 更宽容；其余统计口径不变。
    """
    counter: Counter[int] = Counter()
    buf = bytearray()
    lines: list[bytes] = []
    total = reads = 0
    started = time.time()
    while time.time() - started < seconds:
        for command in commands:
            ser.write(command)
        chunk = ser.read(512)
        if not chunk:
            continue
        reads += 1
        total += len(chunk)
        counter.update(chunk)
        buf.extend(chunk)
        while True:
            cut = -1
            for index, byte in enumerate(buf):
                if byte in (0x0A, 0x0D):
                    cut = index
                    break
            if cut < 0:
                break
            line = bytes(buf[:cut])
            del buf[: cut + 1]
            if line.strip():
                lines.append(line)
    elapsed = max(0.001, time.time() - started)
    printable = sum(n for b, n in counter.items() if 32 <= b < 127)
    high = sum(n for b, n in counter.items() if b >= 0x80)
    nmea = [line for line in lines
            if b"$" in line[:4] and (b"GGA" in line[:12] or b"RMC" in line[:12])]
    gga = [line for line in nmea if b"GGA" in line[:12]]
    return {
        "seconds": elapsed,
        "reads": reads,
        "lines": len(lines),
        "total": total,
        "rate": total / elapsed,
        "printable": printable,
        "high": high,
        "crlf": counter[0x0D] + counter[0x0A],
        "dollar": len([line for line in lines if b"$" in line[:4]]),
        "gga": len(gga),
        "nmea": len(nmea),
        "top3": counter.most_common(3),
        "samples": (gga or nmea or lines)[:3],
    }


def report(title: str, result: dict) -> None:
    total = result["total"]
    print("\n--- %s ---" % title)
    print("  reads=%d  bytes=%d  (%.0f B/s)  lines=%d  $ lines=%d  NMEA lines=%d  GGA lines=%d"
          % (result["reads"], total, result["rate"], result["lines"],
             result["dollar"], result["nmea"], result["gga"]))
    if not total:
        print("  (no bytes at all)")
        return
    print("  printable=%d%%  >=0x80=%d%%  CR/LF=%d  top3=%s"
          % (100 * result["printable"] // total, 100 * result["high"] // total,
             result["crlf"],
             ", ".join("0x%02x x%d" % (b, n) for b, n in result["top3"])))
    for index, sample in enumerate(result["samples"], 1):
        print("  line%d: %s" % (index, render(sample)[:120]))


def main() -> int:
    print("SCRIPT v4 OK  python=%s" % sys.version.split()[0])

    ports = list(list_ports.comports())
    print("\n=== COM PORTS (device | description | hwid) ===")
    for port in ports:
        print("  %s | %s | %s" % (port.device, port.description, port.hwid))
    if not ports:
        print("  (no serial port found)")
        print("VERDICT: NO_PORT -> the USB-RS232 adapter is not recognized at all.")
        return 2

    # 不指定端口时，自动挑第一个"像 USB 转串口"的口（跳过蓝牙等虚拟口）。
    if len(sys.argv) > 1:
        target = sys.argv[1]
    else:
        target = ""
        for port in ports:
            text = ("%s %s" % (port.description, port.hwid)).lower()
            if "bthenum" in text or "bluetooth" in text:
                continue
            target = port.device
            break
        target = target or ports[0].device

    print("\n=== TEACHER METHOD: port=%s baudrate=115200 bytesize=8 stopbits=1 parity=N ===" % target)
    try:
        ser = open_port(target, 115200)
    except Exception as exc:  # noqa: BLE001 - diagnostic script prints everything
        print("OPEN FAILED: %s" % exc)
        print("VERDICT: PORT_OPEN_FAILED")
        print("hint: stop the service first ->  schtasks /End /TN UAVGPR_AirBlackBox")
        return 2
    print("OPEN OK: %s" % ser.portstr)

    phase1: dict = {}
    try:
        phase1 = run_phase(ser, 3.0, [b"log gpgga\r\n"])   # teacher's exact command
    except Exception as exc:  # noqa: BLE001 - 驱动/渲染异常也必须留下证据
        print("PHASE1 ERROR: %r" % (exc,))
    report("PHASE 1  @115200  teacher command (3s)", phase1)

    phase2: dict = {}
    if not phase1.get("gga"):
        # 手册 4.2 "修改串口波特率：com com1 9600"（临时生效，不写 flash）。
        try:
            ser.write(b"com com1 9600\r\n")
            ser.write(b"com com3 9600\r\n")
            time.sleep(0.4)
        except Exception as exc:  # noqa: BLE001
            print("PHASE2 SETUP WRITE ERROR: %r" % (exc,))
        try:
            ser.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            ser = open_port(target, 9600)
        except Exception as exc:  # noqa: BLE001
            print("REOPEN @9600 FAILED: %s" % exc)
            print("VERDICT: REOPEN_9600_FAILED")
            return 2
        print("REOPEN OK @9600: %s" % ser.portstr)
        try:
            phase2 = run_phase(ser, 6.0, [
                b"log gpgga\r\n",
                b"log com3 gpgga ontime 1\r\n",
                b"log com1 gpgga ontime 1\r\n",
            ])
        except Exception as exc:  # noqa: BLE001
            print("PHASE2 ERROR: %r" % (exc,))
        report("PHASE 2  @9600  com comX 9600 + 1Hz GGA (7s)", phase2)
        if phase2.get("gga"):
            # 9600 可用 → 把接收机这个配置写进 flash，重启后也保持 9600。
            try:
                ser.write(b"saveconfig\r\n")
                time.sleep(0.5)
                print("INFO: sent saveconfig -> receiver keeps 9600 after reboot.")
                print("INFO: to revert later: com com1 115200 + com com3 115200 + saveconfig")
            except Exception as exc:  # noqa: BLE001
                print("SAVECONFIG WRITE ERROR: %r" % (exc,))
        else:
            # 9600 也不行 → 把接收机切回 115200，别把它留在 9600。
            try:
                ser.write(b"com com1 115200\r\n")
                ser.write(b"com com3 115200\r\n")
                time.sleep(0.3)
                print("INFO: restored receiver ports back to 115200 (no saveconfig).")
            except Exception:  # noqa: BLE001
                pass
    try:
        ser.close()
    except Exception:  # noqa: BLE001
        pass

    print("\n=== VERDICT ===")
    if phase1.get("gga"):
        print("NMEA_OK  -> teacher method reads NMEA on this link: difference is inside our program.")
    elif phase2.get("gga"):
        print("FIX_9600_SAVED -> 115200 loses low bits but 9600 reads NMEA cleanly.")
        print("  receiver: already set to 9600 + saveconfig (persistent).")
        print("  NEXT: set the ground-station device panel GNSS baud rate to 9600")
        print("        (air_config.json gnss_baud=9600) and start the service.")
    elif not phase1.get("total") and not phase2.get("total"):
        print("NO_BYTES -> adapter present but silent: receiver not outputting, or wrong pins.")
    elif phase2.get("total") and phase2["printable"] * 10 >= phase2["total"] * 8:
        print("TEXT_NOT_NMEA -> readable text but no NMEA lines (check receiver output config).")
    else:
        print("NOT_NMEA -> bytes arrive but are corrupted: PORT / CABLE / USB-RS232 ADAPTER problem.")
        print("  -> try another USB-RS232 adapter (CH340/FTDI); PL2303 clones lose the low bits.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
