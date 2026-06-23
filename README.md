# UAV-GPR Control System

Python/Qt control software for a UAV-mounted ground penetrating radar (GPR) prototype using:

- LibreVNA as the two-port VNA backend
- GNSS receiver over serial NMEA messages
- PySide6 + pyqtgraph + Matplotlib for the desktop UI

The application controls LibreVNA directly over USB, without requiring LibreVNA-GUI. It supports two-port S-parameter acquisition, SOLT/SLOT-style calibration workflow, GNSS logging, real-time A-scan/B-scan display, and CSV/NPZ data export.

## Current Features

- Direct LibreVNA USB communication through PyUSB/libusb
- Two-port sweep acquisition:
  - `S11`
  - `S21`
  - `S12`
  - `S22`
- Default sweep settings:
  - Start: `400 MHz`
  - Stop: `4000 MHz`
  - Points: `201`
  - IFBW: `10000 Hz`
  - Power: `-20 dBm`
- GNSS serial input from NMEA `GGA` / `RMC`
- GNSS map view with track display when valid latitude/longitude are available
- Calibration workflow:
  - One-port P1 OSL: `P1 Open`, `P1 Short`, `P1 Load`
  - Two-port seven-step workflow: `P1 Open`, `P1 Short`, `P1 Load`, `P2 Open`, `P2 Short`, `P2 Load`, `Through`
- Real-time plots:
  - S-parameter magnitude with separate axes for reflection and transmission channels
  - Four-channel normalized A-scan
  - Four-channel B-scan, MATLAB `imagesc` style
- Output formats:
  - CSV for readable frequency-domain S-parameter data
  - NPZ for structured NumPy data
  - JSONL / sidecar JSON metadata with sweep settings and GNSS status

## Repository Layout

```text
.
├── uav_gpr_qt.py              # Main PySide6 application
├── requirements.txt           # Python dependencies
├── VNA_GPS.py                 # Original early GNSS + VNA reference script
├── librevna/
│   ├── librevna_usb_probe.py  # LibreVNA USB device probe
│   └── librevna_usb_s11.py    # LibreVNA USB sweep and S-parameter backend
└── GPS/                       # GNSS receiver manuals / NMEA references
```

Runtime output folders such as `gpr_runs/`, `gpr_runs_test/`, calibration exports, CSV/NPZ files, and Python cache files are intentionally ignored by Git.

## Installation

Python 3.11+ is recommended.

```powershell
python -m pip install -r requirements.txt
```

On Windows, LibreVNA USB access requires PyUSB plus libusb. This project uses `libusb-package` from `requirements.txt`.

## Hardware

### LibreVNA

Connect LibreVNA to the PC by USB. The current backend expects:

- USB VID: `0x1209`
- USB PID: `0x4121`
- Bulk OUT endpoint: `0x01`
- Bulk IN endpoint: `0x81`

The software performs a two-stage sweep:

- Stage 0: Port 1 excitation, producing `S11` and `S21`
- Stage 1: Port 2 excitation, producing `S12` and `S22`

### GNSS

Connect the GNSS receiver as a serial device. The default baud rate is `115200`.

The software reads NMEA:

- `GGA` for fix quality, satellites, altitude, latitude/longitude
- `RMC` as a fallback source for valid latitude/longitude

If the UI shows `fix=0` or `lat=None, lon=None`, the map cannot draw a point until the receiver outputs a valid position.

## Running

```powershell
python uav_gpr_qt.py
```

The first page contains device connection, VNA sweep settings, and calibration controls.

The second page contains data acquisition, GNSS map, S-parameter display, A-scan display, and four B-scan panels.

## Calibration Workflow

### One-Port P1 Calibration

Use this when only `S11` correction is needed:

1. Connect Open to Port 1, click `P1 Open`
2. Connect Short to Port 1, click `P1 Short`
3. Connect Load to Port 1, click `P1 Load`
4. Click `Generate and Enable SOLT/SLOT Calibration`

### Two-Port Seven-Step Workflow

For the current two-port workflow:

1. `P1 Open`
2. `P1 Short`
3. `P1 Load`
4. `P2 Open`
5. `P2 Short`
6. `P2 Load`
7. `Through`

The application currently applies:

- P1 OSL correction to `S11`
- P2 OSL correction to `S22`
- Through normalization to `S21` and `S12`

The saved calibration JSON stores all four measured channels for each calibration step.

## Data Output

Each trace may generate:

- `trace_XXXXX_YYYYMMDD_HHMMSS_mmm.csv`
- `trace_XXXXX_YYYYMMDD_HHMMSS_mmm.npz`
- `run_metadata.jsonl`

If `run_metadata.jsonl` is locked by a sync tool, a sidecar metadata file is written beside the trace instead.

CSV columns include raw and displayed/calibrated values for all channels:

- `s11_*`
- `s21_*`
- `s12_*`
- `s22_*`

The NPZ file stores the same channels as NumPy arrays, plus time-domain trace and GNSS metadata.

## Development Notes

- The LibreVNA backend is vendored from the LibreVNA USB protocol reference work and extended here for four S-parameters.
- `GPS/` contains local GNSS receiver documentation and NMEA references.
- Generated acquisition data should not be committed.

## Quick Smoke Tests

```powershell
python -m py_compile .\uav_gpr_qt.py .\librevna\librevna_usb_s11.py .\librevna\librevna_usb_probe.py
```

Optional hardware probe:

```powershell
python .\librevna\librevna_usb_probe.py
```

