"""Interactive seven-capture example for the dual-channel reference code.

Run only when LibreVNA and the required Open/Short/Load/Through standards are
physically connected.  Importing this module does not access the hardware.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from .dual_channel_acquisition import SweepSettings, acquire_dual_channel
from .dual_channel_calibration import DualChannelCalibration


def wait_and_capture(label: str, settings: SweepSettings, directory: Path):
    input(f"Connect {label}, keep cables/reference plane fixed, then press Enter...")
    sweep = acquire_dual_channel(settings)
    sweep.save_npz(directory / f"{label.lower().replace(' ', '_')}.npz")
    print(f"Captured {label}: {sweep.frequency_hz.size} points")
    return sweep


def main() -> int:
    parser = argparse.ArgumentParser(description="Seven-step dual-channel calibration")
    parser.add_argument("--out", type=Path, default=Path("dual_channel_calibration"))
    parser.add_argument("--start-hz", type=int, default=400_000_000)
    parser.add_argument("--stop-hz", type=int, default=4_000_000_000)
    parser.add_argument("--points", type=int, default=201)
    parser.add_argument("--ifbw-hz", type=int, default=10_000)
    parser.add_argument("--power-dbm", type=float, default=-20.0)
    parser.add_argument("--dwell-us", type=int, default=0)
    args = parser.parse_args()

    settings = SweepSettings(
        start_hz=args.start_hz,
        stop_hz=args.stop_hz,
        points=args.points,
        ifbw_hz=args.ifbw_hz,
        power_dbm=args.power_dbm,
        dwell_us=args.dwell_us,
    )
    args.out.mkdir(parents=True, exist_ok=True)

    captures = {
        "p1_open": wait_and_capture("P1 Open", settings, args.out),
        "p1_short": wait_and_capture("P1 Short", settings, args.out),
        "p1_load": wait_and_capture("P1 Load", settings, args.out),
        "p2_open": wait_and_capture("P2 Open", settings, args.out),
        "p2_short": wait_and_capture("P2 Short", settings, args.out),
        "p2_load": wait_and_capture("P2 Load", settings, args.out),
        "through": wait_and_capture("Through", settings, args.out),
    }
    calibration = DualChannelCalibration.from_captures(**captures)
    calibration_path = calibration.save_json(args.out / "dual_channel_calibration.json")
    print(f"Calibration saved: {calibration_path.resolve()}")

    input("Connect the DUT/antennas, then press Enter to acquire one corrected sweep...")
    raw = acquire_dual_channel(settings)
    corrected = calibration.apply(raw)
    raw.save_npz(args.out / "dut_raw.npz")
    corrected.save_npz(args.out / "dut_corrected.npz")
    print(f"Raw and corrected DUT sweeps saved under {args.out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
