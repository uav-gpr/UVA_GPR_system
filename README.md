# UAV-GPR Control System

UAV-GPR is a Python/Qt control system for a UAV-mounted ground penetrating radar prototype. It controls a LibreVNA directly over USB, reads GNSS NMEA data, saves local traces, and can send VNA trace data from the air-side computer to the ground-side computer over an HM30 Ethernet link.

## Current Hardware Topology

```text
Air side:
  LibreVNA + GNSS -> air-side PC -> HM30 air unit

Ground side:
  HM30 ground unit -> ground-side PC
```

Validated HM30 IP plan:

```text
HM30 air unit:       192.168.144.11
HM30 ground unit:    192.168.144.12
Air-side PC:         192.168.144.21
Ground-side PC:      192.168.144.20
Subnet mask:         255.255.255.0
Telemetry TCP port:  8765
```

## Main Programs

```text
uav_gpr_qt.py              Air-side acquisition GUI
uav_gpr_ground_station.py  Ground-side receiving/control GUI
uav_gpr_receiver.py        Headless TCP trace receiver
run_ground_station.bat     Windows launcher for the ground station
librevna/                  LibreVNA direct USB backend
docs/                      Formal project and operation documents
```

## Install

Python 3.11+ is recommended.

```powershell
python -m pip install -r requirements.txt
```

Note: the old `.venv\Scripts\python.exe` in this workspace may fail with `uv trampoline failed to spawn Python child process`. Use the normal `python` command, currently backed by `E:\Anaconda\python.exe`, or recreate the virtual environment.

## Run Locally On The Air-Side PC

```powershell
python .\uav_gpr_qt.py
```

The air-side GUI handles:

- LibreVNA USB connection and sweep control
- GNSS serial input
- calibration capture
- local CSV/NPZ saving
- optional remote sending to the ground-side computer

The remote sender default is:

```text
Remote IP: 192.168.144.111
Port: 8765
```

## Run The Ground Station

On the ground-side PC, start the receiver before clicking `Connect` on the air-side GUI:

```powershell
.\run_ground_station.bat
```

Equivalent command:

```powershell
python .\uav_gpr_ground_station.py --host 0.0.0.0 --port 8765 --out .\remote_runs
```

After the air-side PC connects, the ground station can send:

- `Single`: acquire one trace
- `Start`: continuous acquisition
- `Stop`: stop acquisition

Received files are saved under `remote_runs/<run_id>/`.

## Validated HM30 Remote Test

On 2026-07-24, the HM30 link was validated end to end:

```text
Ground-side PC: 192.168.144.111
Air-side PC:    192.168.144.100
Ground listen:  0.0.0.0:8765
Remote command: single
Received trace: trace_00001.npz
Payload size:   81362 bytes
Result:         command, acquisition, SHA-256 checked transfer all succeeded
```

See [docs/hm30_remote_operation.md](docs/hm30_remote_operation.md) for the field operation checklist.

## Data Output

Local air-side acquisition output:

```text
gpr_runs/
  trace_XXXXX_YYYYMMDD_HHMMSS_mmm.csv
  trace_XXXXX_YYYYMMDD_HHMMSS_mmm.npz
  run_metadata.jsonl
```

Ground-side received output:

```text
remote_runs/<run_id>/
  trace_XXXXX.npz
  run_metadata.jsonl
```

Generated acquisition data is ignored by Git.

## Smoke Tests

```powershell
python -m py_compile .\uav_gpr_qt.py .\uav_gpr_ground_station.py .\uav_gpr_receiver.py .\librevna\librevna_usb_s11.py .\librevna\librevna_usb_probe.py
```

Optional LibreVNA USB probe:

```powershell
python .\librevna\librevna_usb_probe.py
```
