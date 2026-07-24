# HM30 Remote VNA Acquisition Operation

This document records the validated field workflow for controlling the air-side UAV-GPR acquisition computer from the ground-side computer through the SIYI HM30 Ethernet bridge.

## Validated Result

Validation time: 2026-07-24.

```text
Ground-side PC IP: 192.168.144.111
Air-side PC IP:    192.168.144.100
HM30 ground unit:  192.168.144.12
HM30 air unit:     192.168.144.11
TCP port:          8765
Protocol:          uav-gpr-telemetry-v1
Command tested:    single
Trace received:    trace_00001.npz
Payload size:      81362 bytes
Result:            success
```

The ground side successfully received the air-side `hello` frame, sent a `single` acquisition command, received `command_ack`, then received and saved a SHA-256 checked NPZ trace.

## Network Configuration

Use static IPv4 addresses in the HM30 LAN subnet.

Ground-side PC:

```text
IP address: 192.168.144.111
Subnet mask: 255.255.255.0
Gateway: 192.168.144.12
```

Air-side PC:

```text
IP address: 192.168.144.100
Subnet mask: 255.255.255.0
Gateway: 192.168.144.12
```

Reserved/common HM30 addresses:

```text
192.168.144.11  HM30 air unit
192.168.144.12  HM30 ground unit
192.168.144.20  handheld ground station Android system
192.168.144.25  common camera/gimbal address
```

Do not assign any PC the same IP as an HM30 unit or camera.

## Startup Order

1. Power on HM30 ground and air units.
2. Connect the HM30 ground unit LAN port to the ground-side PC Ethernet port.
3. Connect the HM30 air unit LAN port to the air-side PC Ethernet port.
4. On the ground-side PC, start the ground station:

```powershell
.\run_ground_station.bat
```

5. On the air-side PC, start acquisition:

```powershell
python .\uav_gpr_qt.py
```

6. In the air-side GUI, confirm:

```text
Remote send: enabled
Remote IP: 192.168.144.111
Port: 8765
```

7. Click `Connect` on the air-side GUI.
8. Use the ground-side GUI to send `Single`, `Start`, or `Stop`.

## Connectivity Checks

Ground-side PC:

```powershell
ping 192.168.144.12
ping 192.168.144.11
ping 192.168.144.100
Get-NetTCPConnection -LocalPort 8765
```

Air-side PC:

```powershell
ping 192.168.144.111
Test-NetConnection 192.168.144.111 -Port 8765
```

Expected:

```text
PingSucceeded: True
TcpTestSucceeded: True after the ground station is running
```

If the air side reports "target computer actively refused the connection", the network is reachable but the ground station is not listening on port `8765`.

## Program Notes

- The air-side `uav_gpr_qt.py` default remote IP is set to `192.168.144.111`.
- The ground-side launcher `run_ground_station.bat` uses the system `python` command because the old `.venv` launcher in this workspace can fail with `uv trampoline failed to spawn Python child process`.
- Received ground-side data is saved to `remote_runs/<run_id>/`.
- Air-side local data is saved to `gpr_runs/`.

## Troubleshooting

No air-side connection:

```text
Check ground station is listening on 0.0.0.0:8765.
Check the air-side Remote IP is 192.168.144.111.
Check Windows Firewall allows private-network access for Python.
```

Ping works but TCP fails:

```text
Start the ground station first.
Confirm no other program occupies port 8765.
Allow Python through Windows Firewall if prompted.
```

No VNA trace after command:

```text
Check LibreVNA USB connection on the air-side PC.
Check the air-side GUI can perform a local single acquisition.
Check the air-side log for acquisition errors.
```
