"""老师原版方法复现：只用老师的串口参数/命令/读法，直接打印真实读数。

用途：把"我们程序的问题"和"线路/端口/转换器的问题"一次性分开。

用法（在天空端黑匣子文件夹里）：
    runtime\\python.exe 老师方法复现.py          # 默认 COM3
    runtime\\python.exe 老师方法复现.py COM5     # 指定端口

注意：运行前必须先停掉黑匣子服务，否则串口被占用打不开：
    schtasks /End /TN UAVGPR_AirBlackBox
"""
from __future__ import annotations

import sys
import time

import serial
from serial.tools import list_ports


def main() -> int:
    ports = list(list_ports.comports())
    print("=== 本机串口（端口 | 描述 | HWID）===")
    for port in ports:
        print(f"  {port.device} | {port.description} | {port.hwid}")
    if not ports:
        print("  （没有发现任何串口）")

    target = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    print(f"\n=== 老师原版方法：port={target} baudrate=115200 bytesize=8 stopbits=1 parity=N ===")
    try:
        ser = serial.Serial(port=target, baudrate=115200, bytesize=8, stopbits=1, parity="N")
    except Exception as exc:  # noqa: BLE001 - 现场诊断脚本，如实打印
        print(f"打不开串口：{exc}")
        print("提示：先停服务 schtasks /End /TN UAVGPR_AirBlackBox，再确认端口号/描述。")
        return 2
    ser.timeout = 1.0
    print(f"串口已打开：{ser.portstr}")

    ascii_lines = 0
    total = 0
    deadline = time.time() + 10.0
    try:
        while time.time() < deadline:
            ser.write("log gpgga\r\n".encode("utf-8"))  # 老师原命令，一字不改
            raw = ser.readline()                        # 老师原读法（按行读）
            if not raw:
                continue
            total += len(raw)
            text = "".join(ch if 32 <= b < 127 else f"\\x{b:02x}" for b in raw)
            print(f"  RX {len(raw):4d}  {text[:100]}")
            if raw.startswith(b"$") and b"GGA" in raw[:10]:
                ascii_lines += 1
    finally:
        ser.close()

    print(f"\n=== 10 秒共收到 {total} 字节，其中形如 $??GGA 的行 {ascii_lines} 条 ===")
    if ascii_lines:
        print("→ 老师原版方法在你这条链路上能读到 NMEA：差异在我们程序这一侧，请把上面输出发我。")
    elif total == 0:
        print("→ 一条字节都没收到：端口可能不是 M100（对照上面的描述），或接收机没有输出。")
    else:
        print("→ 收到字节但不是 NMEA 行：与老师的差异在线路/端口/转换器，程序侧修不了。")
        print("  把上面输出发我即可（特别是：COM 号对不对、描述是不是 USB 转串口）。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
