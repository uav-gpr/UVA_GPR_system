#!/usr/bin/env python3
"""Qt control software for UAV-GPR acquisition with GNSS and LibreVNA."""

from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import queue
import socket
import struct
import sys
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import serial
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtWebEngineWidgets import QWebEngineView
from serial.tools import list_ports

sys.path.insert(0, str(Path(__file__).resolve().parent / "librevna"))
from librevna_usb_probe import PacketStream, open_device  # noqa: E402
from librevna_usb_s11 import (  # noqa: E402
    OSLCalibration,
    OSLPoint,
    acquire_s11,
    acquire_sparameters,
    compute_osl_point,
    request_device_info,
)


@dataclass
class SweepConfig:
    start_mhz: float = 400.0
    stop_mhz: float = 4000.0
    points: int = 201
    ifbw_hz: int = 10000
    power_dbm: float = -20.0
    dwell_us: int = 0
    ifft_points: int = 2048
    time_window_ns: float = 40.0


@dataclass
class GgaFix:
    raw: str = ""
    utc: str = ""
    latitude: float | None = None
    longitude: float | None = None
    fix_quality: int = 0
    satellites: int = 0
    altitude_m: float | None = None


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


@dataclass
class CalibrationMeasurement:
    frequency_hz: list[float]
    real: list[float]
    imag: list[float]
    channels: dict[str, dict[str, list[float]]] | None = None

    @classmethod
    def from_points(cls, points: list[tuple[float, complex]]) -> "CalibrationMeasurement":
        return cls(
            frequency_hz=[float(frequency) for frequency, _ in points],
            real=[float(s11.real) for _, s11 in points],
            imag=[float(s11.imag) for _, s11 in points],
        )

    @classmethod
    def from_sparameter_points(cls, points: list[tuple[float, dict[str, complex]]], primary: str = "S11") -> "CalibrationMeasurement":
        frequency_hz = [float(frequency) for frequency, _ in points]
        channels: dict[str, dict[str, list[float]]] = {}
        for name in ("S11", "S21", "S12", "S22"):
            values = [sparams[name] for _frequency, sparams in points if name in sparams]
            if len(values) == len(points):
                channels[name] = {
                    "real": [float(value.real) for value in values],
                    "imag": [float(value.imag) for value in values],
                }
        primary_values = [
            sparams.get(primary) or sparams.get("S11") or next(iter(sparams.values()))
            for _frequency, sparams in points
        ]
        return cls(
            frequency_hz=frequency_hz,
            real=[float(value.real) for value in primary_values],
            imag=[float(value.imag) for value in primary_values],
            channels=channels,
        )

    def as_complex_by_frequency(self, channel: str | None = None) -> dict[float, complex]:
        if channel and self.channels and channel in self.channels:
            real_values = self.channels[channel]["real"]
            imag_values = self.channels[channel]["imag"]
            return {
                float(frequency): complex(float(real), float(imag))
                for frequency, real, imag in zip(self.frequency_hz, real_values, imag_values)
            }
        return {
            float(frequency): complex(float(real), float(imag))
            for frequency, real, imag in zip(self.frequency_hz, self.real, self.imag)
        }


CALIBRATION_STEPS = [
    ("P1_Open", "P1 Open"),
    ("P1_Short", "P1 Short"),
    ("P1_Load", "P1 Load"),
    ("P2_Open", "P2 Open"),
    ("P2_Short", "P2 Short"),
    ("P2_Load", "P2 Load"),
    ("Through", "Through"),
]
CALIBRATION_STEP_LABELS = dict(CALIBRATION_STEPS)
P1_OSL_KEYS = ("P1_Open", "P1_Short", "P1_Load")
LEGACY_OSL_MAP = {"P1_Open": "Open", "P1_Short": "Short", "P1_Load": "Load"}
S_PARAMETERS = ("S11", "S21", "S12", "S22")


class BScanWidget(QtWidgets.QWidget):
    def __init__(self, title: str) -> None:
        super().__init__()
        self.title = title
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)
        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.title_label.setStyleSheet("font-weight: 600;")
        self.figure = Figure(figsize=(4.0, 1.85), dpi=100)
        self.figure.subplots_adjust(left=0.12, right=0.90, bottom=0.24, top=0.95)
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumHeight(130)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel("Trace")
        self.ax.set_ylabel("Time (ns)")
        self.image_artist = None
        self.colorbar = None
        self.cmap = "gray"
        layout.addWidget(self.title_label)
        layout.addWidget(self.canvas)
        self._draw_empty()

    def set_colormap(self, name: str) -> None:
        self.cmap = "gray" if name == "gray" else "jet"
        if self.image_artist is not None:
            self.image_artist.set_cmap(self.cmap)
            self.canvas.draw_idle()

    def set_data(self, image: np.ndarray, time_ns: np.ndarray | None) -> None:
        self.figure.clear()
        self.figure.subplots_adjust(left=0.12, right=0.90, bottom=0.24, top=0.95)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel("Trace")
        self.ax.set_ylabel("Time (ns)")
        if time_ns is not None and len(time_ns) > 1:
            x0, x1 = (0.5, 1.5) if image.shape[1] <= 1 else (1, image.shape[1])
            extent = [x0, x1, float(time_ns[-1]), float(time_ns[0])]
        else:
            x0, x1 = (0.5, 1.5) if image.shape[1] <= 1 else (1, image.shape[1])
            extent = [x0, x1, image.shape[0], 0]
        finite = image[np.isfinite(image)]
        if finite.size:
            vmin, vmax = np.percentile(finite, [2, 98])
            if math.isclose(vmin, vmax):
                vmin, vmax = float(finite.min()), float(finite.max())
        else:
            vmin, vmax = 0.0, 1.0
        self.image_artist = self.ax.imshow(
            image,
            cmap=self.cmap,
            aspect="auto",
            interpolation="nearest",
            extent=extent,
            vmin=vmin,
            vmax=vmax,
        )
        self.colorbar = self.figure.colorbar(self.image_artist, ax=self.ax, fraction=0.046, pad=0.02)
        self.colorbar.ax.tick_params(labelsize=8)
        self.canvas.draw_idle()

    def clear(self) -> None:
        self._draw_empty()

    def _draw_empty(self) -> None:
        self.figure.clear()
        self.figure.subplots_adjust(left=0.12, right=0.96, bottom=0.24, top=0.95)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel("Trace")
        self.ax.set_ylabel("Time (ns)")
        self.ax.set_xlim(0.5, 1.5)
        self.ax.set_ylim(40, 0)
        self.ax.grid(True, alpha=0.2)
        self.image_artist = None
        self.colorbar = None
        self.canvas.draw_idle()


class GnssMapWidget(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.status = QtWidgets.QLabel("GNSS: no fix")
        self.status.setStyleSheet("padding: 4px; background: #fff3cd; color: #7a4f00;")
        self.web = QWebEngineView()
        self.web.setHtml(self._html())
        layout.addWidget(self.status)
        layout.addWidget(self.web, 1)

    def _html(self) -> str:
        return """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    html, body, #map { height: 100%; margin: 0; }
    .leaflet-container { font-family: Segoe UI, Arial, sans-serif; }
  </style>
</head>
<body>
  <div id="map"></div>
  <script>
    const map = L.map('map').setView([31.2304, 121.4737], 14);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 20,
      attribution: '&copy; OpenStreetMap'
    }).addTo(map);
    let marker = null;
    const path = [];
    const polyline = L.polyline(path, { color: '#e63946', weight: 4 }).addTo(map);
    window.updateGnss = function(lat, lon, text) {
      if (lat === null || lon === null || Number.isNaN(lat) || Number.isNaN(lon)) {
        return;
      }
      const ll = [lat, lon];
      path.push(ll);
      polyline.setLatLngs(path);
      if (!marker) {
        marker = L.circleMarker(ll, { radius: 7, color: '#005f73', fillColor: '#0a9396', fillOpacity: 0.9 }).addTo(map);
        map.setView(ll, 18);
      } else {
        marker.setLatLng(ll);
        map.panTo(ll);
      }
      marker.bindPopup(text || '').openPopup();
    };
  </script>
</body>
</html>
"""

    def update_fix(self, fix: GgaFix) -> None:
        lat = fix.latitude
        lon = fix.longitude
        self.status.setText(
            f"GNSS: fix={fix.fix_quality}, sats={fix.satellites}, "
            f"lat={lat}, lon={lon}, alt={fix.altitude_m}"
        )
        if lat is None or lon is None:
            self.status.setText(
                f"GNSS: no valid lat/lon, fix={fix.fix_quality}, sats={fix.satellites}; "
                "check antenna sky view or RTK/GNSS output"
            )
            self.status.setStyleSheet("padding: 4px; background: #fff3cd; color: #7a4f00;")
            return
        self.status.setStyleSheet("padding: 4px; background: #d1fae5; color: #064e3b;")
        popup = (
            f"UTC {fix.utc}<br>lat {lat:.8f}<br>lon {lon:.8f}<br>"
            f"fix {fix.fix_quality}, sats {fix.satellites}"
        )
        self.web.page().runJavaScript(
            f"window.updateGnss({lat:.12f}, {lon:.12f}, {json.dumps(popup)});"
        )


def osl_measurements_for_port(measurements: dict[str, CalibrationMeasurement], port: int) -> dict[str, CalibrationMeasurement]:
    if port == 1:
        mapping = {"P1_Open": "Open", "P1_Short": "Short", "P1_Load": "Load"}
    elif port == 2:
        mapping = {"P2_Open": "Open", "P2_Short": "Short", "P2_Load": "Load"}
    else:
        raise ValueError(f"Unsupported calibration port: {port}")
    selected: dict[str, CalibrationMeasurement] = {}
    for key, legacy_key in mapping.items():
        measurement = measurements.get(key) or measurements.get(legacy_key)
        if measurement:
            selected[legacy_key] = measurement
    return selected


def s11_osl_measurements(measurements: dict[str, CalibrationMeasurement]) -> dict[str, CalibrationMeasurement]:
    return osl_measurements_for_port(measurements, 1)


def nmea_coord(value: str, hemi: str) -> float | None:
    if not value or not hemi:
        return None
    dot = value.find(".")
    deg_len = dot - 2 if dot >= 0 else len(value) - 2
    deg = float(value[:deg_len])
    minutes = float(value[deg_len:])
    signed = deg + minutes / 60.0
    return -signed if hemi in {"S", "W"} else signed


def parse_gga(line: str) -> GgaFix:
    parts = line.strip().split(",")
    if len(parts) < 10 or not parts[0].endswith("GGA"):
        return GgaFix(raw=line)
    return GgaFix(
        raw=line,
        utc=parts[1],
        latitude=nmea_coord(parts[2], parts[3]),
        longitude=nmea_coord(parts[4], parts[5]),
        fix_quality=int(parts[6] or 0),
        satellites=int(parts[7] or 0),
        altitude_m=float(parts[9]) if parts[9] else None,
    )


def parse_rmc(line: str) -> GgaFix:
    parts = line.strip().split(",")
    if len(parts) < 10 or not parts[0].endswith("RMC"):
        return GgaFix(raw=line)
    valid = parts[2] == "A"
    return GgaFix(
        raw=line,
        utc=parts[1],
        latitude=nmea_coord(parts[3], parts[4]),
        longitude=nmea_coord(parts[5], parts[6]),
        fix_quality=1 if valid else 0,
        satellites=0,
        altitude_m=None,
    )


def calibration_from_measurements(
    measurements: dict[str, CalibrationMeasurement],
    filename: Path | None = None,
    port: int = 1,
) -> OSLCalibration:
    osl_measurements = osl_measurements_for_port(measurements, port)
    missing = [name for name in ("Open", "Short", "Load") if name not in osl_measurements]
    if missing:
        raise ValueError(f"P{port} 一端口校准缺少标准件: {', '.join(missing)}")

    channel = "S11" if port == 1 else "S22"
    measured = {name: measurement.as_complex_by_frequency(channel) for name, measurement in osl_measurements.items()}
    common_freqs = sorted(set(measured["Open"]) & set(measured["Short"]) & set(measured["Load"]))
    if not common_freqs:
        raise ValueError("Open/Short/Load 的频率点不一致，不能生成校准")

    points: list[OSLPoint] = []
    for frequency in common_freqs:
        points.append(
            compute_osl_point(
                frequency,
                open_measured=measured["Open"][frequency],
                short_measured=measured["Short"][frequency],
                load_measured=measured["Load"][frequency],
                open_actual=complex(1.0, 0.0),
                short_actual=complex(-1.0, 0.0),
                load_actual=complex(0.0, 0.0),
            )
        )
    return OSLCalibration(points, filename or Path("session_osl_calibration.json"))


def calibration_points_payload(measurement: CalibrationMeasurement) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for idx, (frequency, real, imag) in enumerate(zip(measurement.frequency_hz, measurement.real, measurement.imag)):
        channels = {
            channel: {
                "real": values["real"][idx],
                "imag": values["imag"][idx],
            }
            for channel, values in (measurement.channels or {}).items()
            if idx < len(values["real"]) and idx < len(values["imag"])
        }
        rows.append({"frequency": frequency, "real": real, "imag": imag, "channels": channels})
    return rows


def write_calibration_file(path: Path, measurements: dict[str, CalibrationMeasurement], cfg: SweepConfig) -> None:
    payload = {
        "format": "uav_gpr_solt_calibration_v2",
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "method": "SOLT/SLOT; one-port S11 uses P1 Open/Short/Load, two-port workflow records P1/P2 OSL plus Through",
        "s_parameter": "S11 now; S11/S21/S12/S22 ready for two-port extension",
        "sweep_config": asdict(cfg),
        "measurements": [
            {
                "type": name,
                "data": {"points": calibration_points_payload(measurement)},
            }
            for name, measurement in measurements.items()
        ],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def read_calibration_measurements(path: Path) -> dict[str, CalibrationMeasurement]:
    data = json.loads(path.read_text(encoding="utf-8"))
    measurements: dict[str, CalibrationMeasurement] = {}
    for measurement in data.get("measurements", []):
        name = measurement.get("type")
        raw_points = measurement.get("data", {}).get("points", [])
        if name in set(CALIBRATION_STEP_LABELS) | {"Open", "Short", "Load"}:
            channels: dict[str, dict[str, list[float]]] = {}
            for point in raw_points:
                for channel, value in point.get("channels", {}).items():
                    channels.setdefault(channel, {"real": [], "imag": []})
                    channels[channel]["real"].append(float(value["real"]))
                    channels[channel]["imag"].append(float(value["imag"]))
            measurements[name] = CalibrationMeasurement(
                frequency_hz=[float(point["frequency"]) for point in raw_points],
                real=[float(point["real"]) for point in raw_points],
                imag=[float(point["imag"]) for point in raw_points],
                channels=channels or None,
            )
    return measurements


class GnssReader:
    def __init__(self, port: str, baud: int = 115200) -> None:
        self.port = port
        self.baud = baud
        self.ser: serial.Serial | None = None

    def open(self) -> None:
        if self.ser and self.ser.is_open:
            return
        self.ser = serial.Serial(self.port, self.baud, timeout=0.8)
        self.ser.write(b"log gpgga\r\n")
        self.ser.write(b"log gprmc\r\n")

    def close(self) -> None:
        if self.ser:
            self.ser.close()
        self.ser = None

    def read_gga(self, timeout_s: float = 2.0) -> GgaFix:
        self.open()
        assert self.ser is not None
        deadline = time.monotonic() + timeout_s
        last = ""
        while time.monotonic() < deadline:
            line = self.ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            last = line
            if line.startswith(("$GPGGA", "$GNGGA", "$BDGGA")):
                fix = parse_gga(line)
                if fix.latitude is not None and fix.longitude is not None:
                    return fix
            if line.startswith(("$GPRMC", "$GNRMC", "$BDRMC")):
                fix = parse_rmc(line)
                if fix.latitude is not None and fix.longitude is not None:
                    return fix
        return GgaFix(raw=last)


def db20(values: np.ndarray) -> np.ndarray:
    return 20.0 * np.log10(np.maximum(np.abs(values), 1e-15))


def normalize_trace_for_display(trace: np.ndarray) -> np.ndarray:
    peak = float(np.max(np.abs(trace))) if trace.size else 0.0
    if peak <= 1e-15:
        return trace.copy()
    return trace / peak


def robust_db_range(values: list[np.ndarray], default: tuple[float, float]) -> tuple[float, float]:
    if not values:
        return default
    merged = np.concatenate([np.asarray(v, dtype=float).ravel() for v in values if len(v)])
    merged = merged[np.isfinite(merged)]
    if merged.size == 0:
        return default
    low, high = np.percentile(merged, [5, 95])
    if math.isclose(float(low), float(high), abs_tol=1e-6):
        low, high = float(merged.min()), float(merged.max())
    pad = max((high - low) * 0.15, 3.0)
    return float(low - pad), float(high + pad)


def sparameter_arrays(points: list[tuple[float, dict[str, complex]]]) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    frequency_hz = np.array([frequency for frequency, _sparams in points], dtype=float)
    arrays = {
        name: np.array([sparams[name] for _frequency, sparams in points], dtype=np.complex128)
        for name in S_PARAMETERS
    }
    return frequency_hz, arrays


def interpolate_measurement_channel(
    measurement: CalibrationMeasurement,
    channel: str,
    frequency_hz: np.ndarray,
) -> np.ndarray:
    measured = measurement.as_complex_by_frequency(channel)
    cal_freq = np.array(sorted(measured), dtype=float)
    cal_values = np.array([measured[float(f)] for f in cal_freq], dtype=np.complex128)
    real = np.interp(frequency_hz, cal_freq, cal_values.real)
    imag = np.interp(frequency_hz, cal_freq, cal_values.imag)
    return real + 1j * imag


def apply_reflection_and_through_calibration(
    frequency_hz: np.ndarray,
    raw_sparams: dict[str, np.ndarray],
    calibration_p1: OSLCalibration | None,
    calibration_p2: OSLCalibration | None,
    through: CalibrationMeasurement | None,
) -> dict[str, np.ndarray]:
    display = {name: values.copy() for name, values in raw_sparams.items()}
    if calibration_p1:
        display["S11"] = np.array([calibration_p1.correct(f, s) for f, s in zip(frequency_hz, raw_sparams["S11"])], dtype=np.complex128)
    if calibration_p2:
        display["S22"] = np.array([calibration_p2.correct(f, s) for f, s in zip(frequency_hz, raw_sparams["S22"])], dtype=np.complex128)
    if through:
        for channel in ("S21", "S12"):
            ref = interpolate_measurement_channel(through, channel, frequency_hz)
            display[channel] = np.divide(raw_sparams[channel], ref, out=raw_sparams[channel].copy(), where=np.abs(ref) > 1e-15)
    return display


def make_time_trace(s11: np.ndarray, cfg: SweepConfig) -> tuple[np.ndarray, np.ndarray]:
    n = max(cfg.ifft_points, len(s11))
    spectrum = np.zeros(n, dtype=np.complex128)
    spectrum[: len(s11)] = s11 - np.mean(s11)
    trace = np.real(np.fft.ifft(spectrum))
    bandwidth_hz = max((cfg.stop_mhz - cfg.start_mhz) * 1e6, 1.0)
    dt_s = 1.0 / bandwidth_hz / n * len(s11)
    time_ns = np.arange(n) * dt_s * 1e9
    mask = time_ns <= cfg.time_window_ns
    return time_ns[mask], trace[mask]


def save_trace(
    out_dir: Path,
    index: int,
    cfg: SweepConfig,
    frequency_hz: np.ndarray,
    raw_sparams: dict[str, np.ndarray],
    display_sparams: dict[str, np.ndarray],
    time_ns: np.ndarray,
    trace: np.ndarray,
    fix: GgaFix,
) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    csv_path = out_dir / f"trace_{index:05d}_{stamp}.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        header = ["frequency_hz", "frequency_mhz"]
        for name in S_PARAMETERS:
            header += [
                f"{name.lower()}_real_raw",
                f"{name.lower()}_imag_raw",
                f"{name.lower()}_db_raw",
                f"{name.lower()}_real_display",
                f"{name.lower()}_imag_display",
                f"{name.lower()}_db_display",
            ]
        writer.writerow(header)
        for idx, f in enumerate(frequency_hz):
            row: list[float] = [float(f), float(f / 1e6)]
            for name in S_PARAMETERS:
                raw = raw_sparams[name][idx]
                disp = display_sparams[name][idx]
                row += [raw.real, raw.imag, db20(np.array([raw]))[0], disp.real, disp.imag, db20(np.array([disp]))[0]]
            writer.writerow(row)

    npz_path = csv_path.with_suffix(".npz")
    npz_payload = {
        "config": json.dumps(asdict(cfg), ensure_ascii=False),
        "frequency_hz": frequency_hz,
        "time_ns": time_ns,
        "trace": trace,
        "gnss": json.dumps(asdict(fix), ensure_ascii=False),
    }
    for name in S_PARAMETERS:
        npz_payload[f"raw_{name}_real"] = raw_sparams[name].real
        npz_payload[f"raw_{name}_imag"] = raw_sparams[name].imag
        npz_payload[f"display_{name}_real"] = display_sparams[name].real
        npz_payload[f"display_{name}_imag"] = display_sparams[name].imag
    np.savez_compressed(
        npz_path,
        **npz_payload,
    )

    meta = {
        "index": index,
        "timestamp": stamp,
        "frequency_range_hz": [float(frequency_hz[0]), float(frequency_hz[-1])],
        "s_parameter": list(S_PARAMETERS),
        "config": asdict(cfg),
        "gnss": asdict(fix),
        "csv": csv_path.name,
        "npz": npz_path.name,
    }
    meta_path = out_dir / "run_metadata.jsonl"
    try:
        with meta_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(meta, ensure_ascii=False) + "\n")
    except PermissionError:
        sidecar_path = csv_path.with_suffix(".metadata.json")
        sidecar_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    return csv_path


def trace_npz_bytes(
    index: int,
    cfg: SweepConfig,
    frequency_hz: np.ndarray,
    raw_sparams: dict[str, np.ndarray],
    display_sparams: dict[str, np.ndarray],
    time_ns: np.ndarray,
    trace_by_channel: dict[str, np.ndarray],
    fix: GgaFix,
) -> bytes:
    payload: dict[str, object] = {
        "index": np.array(index, dtype=np.int64),
        "config": json.dumps(asdict(cfg), ensure_ascii=False),
        "frequency_hz": np.asarray(frequency_hz),
        "time_ns": np.asarray(time_ns),
        "gnss": json.dumps(asdict(fix), ensure_ascii=False),
    }
    for name in S_PARAMETERS:
        payload[f"raw_{name}_real"] = raw_sparams[name].real
        payload[f"raw_{name}_imag"] = raw_sparams[name].imag
        payload[f"display_{name}_real"] = display_sparams[name].real
        payload[f"display_{name}_imag"] = display_sparams[name].imag
        payload[f"trace_{name}"] = np.asarray(trace_by_channel[name], dtype=float)
    buffer = io.BytesIO()
    np.savez_compressed(buffer, **payload)
    return buffer.getvalue()


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("socket closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def send_remote_frame(sock: socket.socket, header: dict[str, object], payload: bytes = b"") -> None:
    header_bytes = json.dumps(header, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    sock.sendall(struct.pack("!I", len(header_bytes)))
    sock.sendall(header_bytes)
    sock.sendall(struct.pack("!Q", len(payload)))
    if payload:
        sock.sendall(payload)


def recv_remote_header(sock: socket.socket) -> dict[str, object]:
    header_len = struct.unpack("!I", recv_exact(sock, 4))[0]
    header = json.loads(recv_exact(sock, header_len).decode("utf-8"))
    payload_len = struct.unpack("!Q", recv_exact(sock, 8))[0]
    if payload_len:
        raise ValueError(f"unexpected ack payload: {payload_len} bytes")
    return header


def recv_remote_frame(sock: socket.socket) -> tuple[dict[str, object], bytes]:
    header_len = struct.unpack("!I", recv_exact(sock, 4))[0]
    header = json.loads(recv_exact(sock, header_len).decode("utf-8"))
    payload_len = struct.unpack("!Q", recv_exact(sock, 8))[0]
    payload = recv_exact(sock, payload_len) if payload_len else b""
    return header, payload


class RemoteSender:
    def __init__(self, host: str, port: int, run_id: str, status_callback, command_callback=None) -> None:
        self.host = host
        self.port = port
        self.run_id = run_id
        self.status_callback = status_callback
        self.command_callback = command_callback
        self.queue: queue.Queue[tuple[dict[str, object], bytes] | None] = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="uav-gpr-remote-sender", daemon=True)
        self._sock: socket.socket | None = None

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.queue.put(None)
        self._close()

    def enqueue_trace(
        self,
        index: int,
        cfg: SweepConfig,
        frequency_hz: np.ndarray,
        raw_sparams: dict[str, np.ndarray],
        display_sparams: dict[str, np.ndarray],
        time_ns: np.ndarray,
        trace_by_channel: dict[str, np.ndarray],
        fix: GgaFix,
    ) -> None:
        payload = trace_npz_bytes(index, cfg, frequency_hz, raw_sparams, display_sparams, time_ns, trace_by_channel, fix)
        header = {
            "type": "trace",
            "protocol": "uav-gpr-telemetry-v1",
            "run_id": self.run_id,
            "trace_index": index,
            "timestamp": utc_timestamp(),
            "payload_format": "npz",
            "payload_bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
            "config": asdict(cfg),
            "gnss": asdict(fix),
        }
        self.queue.put((header, payload))

    def _connect(self) -> socket.socket:
        sock = socket.create_connection((self.host, self.port), timeout=5.0)
        sock.settimeout(0.5)
        send_remote_frame(
            sock,
            {
                "type": "hello",
                "protocol": "uav-gpr-telemetry-v1",
                "run_id": self.run_id,
                "timestamp": utc_timestamp(),
                "supports_commands": True,
            },
        )
        ack, _payload = self._read_frame(sock)
        if ack.get("type") != "ack":
            raise ConnectionError(f"unexpected hello ack: {ack}")
        self.status_callback(f"Remote connected: {self.host}:{self.port}")
        return sock

    def _close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _read_frame(self, sock: socket.socket) -> tuple[dict[str, object], bytes]:
        while not self._stop.is_set():
            try:
                return recv_remote_frame(sock)
            except socket.timeout:
                continue
        raise ConnectionError("remote sender stopped")

    def _handle_command(self, header: dict[str, object]) -> None:
        action = str(header.get("action") or "")
        params = header.get("params") if isinstance(header.get("params"), dict) else {}
        if self.command_callback is not None:
            self.command_callback(action, params)
        if self._sock is not None:
            send_remote_frame(
                self._sock,
                {
                    "type": "command_ack",
                    "protocol": "uav-gpr-telemetry-v1",
                    "run_id": self.run_id,
                    "action": action,
                    "timestamp": utc_timestamp(),
                },
            )

    def _wait_for_trace_ack(self, sock: socket.socket, trace_index: object) -> None:
        while not self._stop.is_set():
            header, _payload = self._read_frame(sock)
            msg_type = header.get("type")
            if msg_type == "ack_trace" and header.get("trace_index") == trace_index:
                return
            if msg_type == "command":
                self._handle_command(header)
                continue
            raise ConnectionError(f"unexpected trace ack: {header}")

    def _run(self) -> None:
        pending: tuple[dict[str, object], bytes] | None = None
        while not self._stop.is_set():
            try:
                if self._sock is None:
                    self._sock = self._connect()
                try:
                    item = pending or self.queue.get(timeout=0.2)
                except queue.Empty:
                    try:
                        header, _payload = recv_remote_frame(self._sock)
                    except socket.timeout:
                        continue
                    if header.get("type") == "command":
                        self._handle_command(header)
                        continue
                    raise ConnectionError(f"unexpected remote frame: {header}")
                pending = None
                if item is None:
                    break
                header, payload = item
                send_remote_frame(self._sock, header, payload)
                self._wait_for_trace_ack(self._sock, header.get("trace_index"))
                self.status_callback(f"Remote sent trace {header['trace_index']}")
            except queue.Empty:
                continue
            except Exception as exc:
                if pending is None and "item" in locals() and item is not None:
                    pending = item
                self._close()
                self.status_callback(f"Remote send retrying: {exc}")
                time.sleep(2.0)


class AcquisitionWorker(QtCore.QThread):
    trace_ready = QtCore.Signal(int, object, object, object, object, object, object, str)
    status = QtCore.Signal(str)
    failed = QtCore.Signal(str)
    finished_run = QtCore.Signal()

    def __init__(
        self,
        cfg: SweepConfig,
        gnss_port: str,
        baud: int,
        out_dir: Path,
        count: int,
        interval_s: float,
        calibration_path: str,
        session_calibration: OSLCalibration | None,
        session_calibration_p2: OSLCalibration | None,
        through_measurement: CalibrationMeasurement | None,
        background: np.ndarray | None,
        autosave: bool,
    ) -> None:
        super().__init__()
        self.cfg = cfg
        self.gnss_port = gnss_port
        self.baud = baud
        self.out_dir = out_dir
        self.count = count
        self.interval_s = interval_s
        self.calibration_path = calibration_path
        self.session_calibration = session_calibration
        self.session_calibration_p2 = session_calibration_p2
        self.through_measurement = through_measurement
        self.background = background
        self.autosave = autosave
        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def run(self) -> None:
        gnss = GnssReader(self.gnss_port, self.baud) if self.gnss_port else None
        calibration_p1 = self.session_calibration or (OSLCalibration.from_file(self.calibration_path) if self.calibration_path else None)
        calibration_p2 = self.session_calibration_p2
        index = 0
        try:
            while not self._stop and (self.count <= 0 or index < self.count):
                index += 1
                self.status.emit(f"Acquiring trace {index}...")
                points = acquire_sparameters(
                    int(self.cfg.start_mhz * 1e6),
                    int(self.cfg.stop_mhz * 1e6),
                    self.cfg.points,
                    self.cfg.ifbw_hz,
                    self.cfg.power_dbm,
                    self.cfg.dwell_us,
                )
                freq, raw = sparameter_arrays(points)
                display = apply_reflection_and_through_calibration(
                    freq,
                    raw,
                    calibration_p1,
                    calibration_p2,
                    self.through_measurement,
                )
                if self.background is not None and len(self.background) == len(display["S11"]):
                    display["S11"] = display["S11"] - self.background
                fix = gnss.read_gga() if gnss else GgaFix()
                trace_by_channel: dict[str, np.ndarray] = {}
                time_ns = None
                for name in S_PARAMETERS:
                    channel_time_ns, channel_trace = make_time_trace(display[name], self.cfg)
                    if time_ns is None:
                        time_ns = channel_time_ns
                    trace_by_channel[name] = channel_trace
                assert time_ns is not None
                path_text = ""
                if self.autosave:
                    path_text = str(save_trace(self.out_dir, index, self.cfg, freq, raw, display, time_ns, trace_by_channel["S11"], fix))
                self.trace_ready.emit(index, freq, raw, display, time_ns, trace_by_channel, fix, path_text)
                if self.count > 0 and index >= self.count:
                    break
                slept = 0.0
                while not self._stop and slept < self.interval_s:
                    time.sleep(min(0.1, self.interval_s - slept))
                    slept += 0.1
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            if gnss:
                gnss.close()
            self.finished_run.emit()


class MainWindow(QtWidgets.QMainWindow):
    remote_status = QtCore.Signal(str)
    remote_command = QtCore.Signal(str, object)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("UAV-GPR Control System")
        self.resize(1380, 860)
        self.worker: AcquisitionWorker | None = None
        self.remote_sender: RemoteSender | None = None
        self.remote_run_id = ""
        self.background: np.ndarray | None = None
        self.cal_measurements: dict[str, CalibrationMeasurement] = {}
        self.session_calibration: OSLCalibration | None = None
        self.session_calibration_p2: OSLCalibration | None = None
        self.through_measurement: CalibrationMeasurement | None = None
        self.bscan: dict[str, list[np.ndarray]] = {name: [] for name in S_PARAMETERS}
        self.last_display_sparams: dict[str, np.ndarray] | None = None
        self.last_time_ns: np.ndarray | None = None
        self.remote_status.connect(self.log)
        self.remote_command.connect(self.handle_remote_command)
        self._build_ui()
        self.refresh_ports()
        QtCore.QTimer.singleShot(1200, self.connect_remote_sender)

    def _build_ui(self) -> None:
        tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(tabs)
        tabs.addTab(self._connection_page(), "连接/参数/校准")
        tabs.addTab(self._acquisition_page(), "数据采集/B-scan")
        self.statusBar().showMessage("Ready")

    def _spin(self, value: float, min_v: float, max_v: float, decimals: int = 0) -> QtWidgets.QDoubleSpinBox:
        box = QtWidgets.QDoubleSpinBox()
        box.setRange(min_v, max_v)
        box.setDecimals(decimals)
        box.setValue(value)
        box.setKeyboardTracking(False)
        return box

    def _connection_page(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(page)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)

        dev_group = QtWidgets.QGroupBox("设备连接")
        dev = QtWidgets.QGridLayout(dev_group)
        self.port_combo = QtWidgets.QComboBox()
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["115200", "9600", "38400", "57600"])
        refresh = QtWidgets.QPushButton("刷新串口")
        refresh.clicked.connect(self.refresh_ports)
        test_gnss = QtWidgets.QPushButton("测试 GNSS")
        test_gnss.clicked.connect(self.test_gnss)
        test_vna = QtWidgets.QPushButton("测试 LibreVNA USB")
        test_vna.clicked.connect(self.test_vna)
        self.device_log = QtWidgets.QPlainTextEdit()
        self.device_log.setReadOnly(True)
        self.device_log.setMaximumBlockCount(200)
        self.device_log.setMinimumHeight(180)
        dev.addWidget(QtWidgets.QLabel("GNSS 串口"), 0, 0)
        dev.addWidget(self.port_combo, 0, 1)
        dev.addWidget(refresh, 0, 2)
        dev.addWidget(QtWidgets.QLabel("波特率"), 1, 0)
        dev.addWidget(self.baud_combo, 1, 1)
        dev.addWidget(test_gnss, 2, 0, 1, 2)
        dev.addWidget(test_vna, 2, 2)
        dev.addWidget(self.device_log, 3, 0, 1, 3)
        dev.setColumnStretch(1, 1)

        sweep_group = QtWidgets.QGroupBox("VNA 基础参数")
        sweep = QtWidgets.QGridLayout(sweep_group)
        self.start_mhz = self._spin(400, 0.001, 6000, 3)
        self.stop_mhz = self._spin(4000, 0.001, 6000, 3)
        self.points = QtWidgets.QSpinBox()
        self.points.setRange(2, 4501)
        self.points.setValue(201)
        self.ifbw = QtWidgets.QSpinBox()
        self.ifbw.setRange(6, 50000)
        self.ifbw.setValue(10000)
        self.power = self._spin(-20, -40, 0, 1)
        self.dwell = QtWidgets.QSpinBox()
        self.dwell.setRange(0, 10239)
        self.ifft_points = QtWidgets.QSpinBox()
        self.ifft_points.setRange(256, 65536)
        self.ifft_points.setValue(2048)
        self.time_window = self._spin(40, 1, 1000, 1)
        fields = [
            ("Start MHz", self.start_mhz), ("Stop MHz", self.stop_mhz),
            ("Points", self.points), ("IFBW Hz", self.ifbw),
            ("Power dBm", self.power), ("Dwell us", self.dwell),
            ("IFFT points", self.ifft_points), ("Time window ns", self.time_window),
        ]
        for index, (label, widget) in enumerate(fields):
            row = index // 2
            col = (index % 2) * 2
            sweep.addWidget(QtWidgets.QLabel(label), row, col)
            sweep.addWidget(widget, row, col + 1)
        apply_sweep = QtWidgets.QPushButton("按当前设置测试扫频")
        apply_sweep.clicked.connect(self.test_sweep_settings)
        sweep.addWidget(apply_sweep, 4, 0, 1, 4)
        sweep.setColumnStretch(1, 1)
        sweep.setColumnStretch(3, 1)

        cal_group = QtWidgets.QGroupBox("校准")
        cal = QtWidgets.QGridLayout(cal_group)
        self.cal_path = QtWidgets.QLineEdit()
        browse_cal = QtWidgets.QPushButton("加载校准文件")
        browse_cal.clicked.connect(self.choose_cal)
        self.cal_mode = QtWidgets.QComboBox()
        self.cal_mode.addItems(["一端口校准 P1 OSL（三项）", "两端口校准 SOLT/SLOT（七项）"])
        self.cal_mode.currentIndexChanged.connect(lambda: self.update_calibration_status())
        self.cal_lamps: dict[str, QtWidgets.QLabel] = {}
        build_cal = QtWidgets.QPushButton("生成并启用 SOLT/SLOT 校准")
        build_cal.clicked.connect(self.build_session_calibration)
        save_cal = QtWidgets.QPushButton("保存当前校准")
        save_cal.clicked.connect(self.save_session_calibration)
        bg_btn = QtWidgets.QPushButton("采集背景扣除")
        bg_btn.clicked.connect(self.capture_background)
        clear_bg = QtWidgets.QPushButton("清除背景")
        clear_bg.clicked.connect(self.clear_background)
        self.bg_label = QtWidgets.QLabel("未采集背景")
        self.cal_status = QtWidgets.QLabel("未启用校准")
        self.cal_status.setWordWrap(True)
        cal.addWidget(QtWidgets.QLabel("LibreVNA 校准文件"), 0, 0)
        cal.addWidget(self.cal_path, 0, 1, 1, 2)
        cal.addWidget(browse_cal, 0, 3)
        cal.addWidget(QtWidgets.QLabel("校准模式"), 1, 0)
        cal.addWidget(self.cal_mode, 1, 1, 1, 3)

        for index, (key, label) in enumerate(CALIBRATION_STEPS, start=1):
            row = 1 + index
            lamp = QtWidgets.QLabel()
            lamp.setFixedSize(16, 16)
            lamp.setToolTip(f"{label} 未采集")
            self.cal_lamps[key] = lamp
            button = QtWidgets.QPushButton(f"{index} 采集 {label}")
            button.clicked.connect(lambda _checked=False, step_key=key: self.capture_calibration_standard(step_key))
            cal.addWidget(lamp, row, 0, QtCore.Qt.AlignmentFlag.AlignCenter)
            cal.addWidget(button, row, 1, 1, 3)

        cal.addWidget(build_cal, 9, 0, 1, 2)
        cal.addWidget(save_cal, 9, 2, 1, 2)
        cal.addWidget(bg_btn, 10, 0, 1, 2)
        cal.addWidget(clear_bg, 10, 2, 1, 2)
        cal.addWidget(self.cal_status, 11, 0, 1, 4)
        cal.addWidget(self.bg_label, 12, 0, 1, 4)
        cal.setColumnStretch(1, 1)
        self.update_calibration_lamps()

        layout.addWidget(dev_group, 0, 0)
        layout.addWidget(sweep_group, 1, 0)
        layout.addWidget(cal_group, 0, 1, 2, 1)
        layout.setColumnStretch(0, 3)
        layout.setColumnStretch(1, 2)
        layout.setRowStretch(0, 1)
        layout.setRowStretch(1, 0)
        return page

    def _acquisition_page(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(page)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)
        controls = QtWidgets.QGroupBox("采集控制")
        ctl = QtWidgets.QGridLayout(controls)
        self.output_dir = QtWidgets.QLineEdit(str(Path.cwd() / "gpr_runs"))
        browse_out = QtWidgets.QPushButton("输出目录")
        browse_out.clicked.connect(self.choose_output)
        self.interval = self._spin(1.0, 0.0, 3600.0, 2)
        self.count = QtWidgets.QSpinBox()
        self.count.setRange(0, 1000000)
        self.count.setValue(0)
        self.autosave = QtWidgets.QCheckBox("自动保存 CSV/NPZ")
        self.autosave.setChecked(True)
        self.remote_enable = QtWidgets.QCheckBox("Remote send")
        self.remote_enable.setChecked(True)
        self.remote_host = QtWidgets.QLineEdit("127.0.0.1")
        self.remote_host.setMaximumWidth(140)
        self.remote_port = QtWidgets.QSpinBox()
        self.remote_port.setRange(1, 65535)
        self.remote_port.setValue(8765)
        self.remote_port.setMaximumWidth(90)
        self.remote_connect = QtWidgets.QPushButton("Connect")
        self.remote_connect.clicked.connect(self.connect_remote_sender)
        self.bscan_cmap = QtWidgets.QComboBox()
        self.bscan_cmap.addItems(["gray", "jet"])
        self.bscan_cmap.currentTextChanged.connect(lambda _text: self.refresh_bscan_views())
        start = QtWidgets.QPushButton("开始采集")
        start.clicked.connect(self.start_acquisition)
        single = QtWidgets.QPushButton("采集一次")
        single.clicked.connect(lambda: self.start_acquisition(single=True))
        stop = QtWidgets.QPushButton("停止")
        stop.clicked.connect(self.stop_acquisition)
        clear = QtWidgets.QPushButton("清空 B-scan")
        clear.clicked.connect(self.clear_bscan)
        self.last_info = QtWidgets.QLabel("No data")
        ctl.addWidget(QtWidgets.QLabel("输出目录"), 0, 0)
        ctl.addWidget(self.output_dir, 0, 1, 1, 6)
        ctl.addWidget(browse_out, 0, 7)
        ctl.addWidget(QtWidgets.QLabel("间隔 s"), 1, 0)
        ctl.addWidget(self.interval, 1, 1)
        ctl.addWidget(QtWidgets.QLabel("次数(0=连续)"), 1, 2)
        ctl.addWidget(self.count, 1, 3)
        ctl.addWidget(self.autosave, 1, 4)
        ctl.addWidget(single, 1, 5)
        ctl.addWidget(start, 1, 6)
        ctl.addWidget(stop, 1, 7)
        ctl.addWidget(QtWidgets.QLabel("B-scan 色图"), 2, 0)
        ctl.addWidget(self.bscan_cmap, 2, 1)
        ctl.addWidget(clear, 2, 2)
        ctl.addWidget(self.last_info, 2, 3, 1, 5)
        ctl.addWidget(self.remote_enable, 3, 0)
        ctl.addWidget(QtWidgets.QLabel("Remote IP"), 3, 1)
        ctl.addWidget(self.remote_host, 3, 2)
        ctl.addWidget(QtWidgets.QLabel("Port"), 3, 3)
        ctl.addWidget(self.remote_port, 3, 4)
        ctl.addWidget(self.remote_connect, 3, 5)
        ctl.setColumnStretch(1, 1)
        ctl.setColumnStretch(6, 1)

        channel_colors = {
            "S11": "#0077b6",
            "S21": "#2a9d8f",
            "S12": "#e76f51",
            "S22": "#6d597a",
        }
        self.sparam_plot = pg.PlotWidget(title="S-parameter magnitude")
        self.sparam_plot.setLabel("bottom", "Frequency", units="Hz")
        self.sparam_plot.setLabel("left", "S11/S22", units="dB")
        self.sparam_plot.showGrid(x=True, y=True, alpha=0.25)
        self.sparam_plot.setMouseEnabled(x=False, y=False)
        self.sparam_plot.setMenuEnabled(False)
        self.sparam_plot.getPlotItem().hideButtons()
        self.sparam_plot.getPlotItem().showAxis("right")
        self.sparam_plot.getPlotItem().getAxis("right").setLabel("S21/S12", units="dB")
        self.sparam_right_view = pg.ViewBox()
        self.sparam_right_view.setMouseEnabled(x=False, y=False)
        self.sparam_plot.getPlotItem().scene().addItem(self.sparam_right_view)
        self.sparam_plot.getPlotItem().getAxis("right").linkToView(self.sparam_right_view)
        self.sparam_right_view.setXLink(self.sparam_plot.getPlotItem().vb)
        self.sparam_plot.getPlotItem().vb.sigResized.connect(self.update_sparam_right_axis)
        self.sparam_legend = self.sparam_plot.getPlotItem().addLegend(offset=(10, 10))
        self.sparam_curves: dict[str, pg.PlotDataItem] = {}
        for name in S_PARAMETERS:
            curve = pg.PlotDataItem(pen=pg.mkPen(channel_colors[name], width=2), name=name)
            self.sparam_curves[name] = curve
            if name in {"S11", "S22"}:
                self.sparam_plot.addItem(curve)
            else:
                self.sparam_right_view.addItem(curve)
            self.sparam_legend.addItem(curve, name)
        self.sparam_plot.setXRange(400e6, 4000e6, padding=0)
        self.sparam_plot.setYRange(-60, 10, padding=0)
        self.sparam_right_view.setYRange(-120, 10, padding=0)

        self.ascan_plot = pg.PlotWidget(title="A-scan")
        self.ascan_plot.setLabel("bottom", "Time", units="ns")
        self.ascan_plot.setLabel("left", "Normalized amplitude")
        self.ascan_plot.showGrid(x=True, y=True, alpha=0.25)
        self.ascan_plot.setMouseEnabled(x=False, y=False)
        self.ascan_plot.setMenuEnabled(False)
        self.ascan_plot.getPlotItem().hideButtons()
        self.ascan_plot.addLegend(offset=(10, 10))
        self.ascan_curves: dict[str, pg.PlotDataItem] = {}
        for name in S_PARAMETERS:
            self.ascan_curves[name] = self.ascan_plot.plot(
                pen=pg.mkPen(channel_colors[name], width=2),
                name=name,
            )
        self.ascan_plot.setXRange(0, 40, padding=0)
        self.ascan_plot.setYRange(-1.05, 1.05, padding=0)

        bscan_panel = QtWidgets.QWidget()
        bscan_panel.setMinimumHeight(340)
        bscan_panel.setMaximumHeight(430)
        bscan_grid = QtWidgets.QGridLayout(bscan_panel)
        bscan_grid.setContentsMargins(0, 0, 0, 0)
        bscan_grid.setSpacing(8)
        self.bscan_views: dict[str, BScanWidget] = {}
        for index, name in enumerate(S_PARAMETERS):
            view = BScanWidget(f"{name} B-scan")
            view.set_colormap(self.bscan_cmap.currentText())
            self.bscan_views[name] = view
            bscan_grid.addWidget(view, index // 2, index % 2)

        plot_area = QtWidgets.QWidget()
        plot_area_layout = QtWidgets.QHBoxLayout(plot_area)
        plot_area_layout.setContentsMargins(0, 0, 0, 0)
        plot_area_layout.setSpacing(8)
        left_plots = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_plots)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(8)
        left_layout.addWidget(self.sparam_plot, 1)
        left_layout.addWidget(self.ascan_plot, 1)
        self.gnss_map = GnssMapWidget()
        plot_area_layout.addWidget(left_plots, 3)
        plot_area_layout.addWidget(self.gnss_map, 2)

        layout.addWidget(controls, 0, 0)
        layout.addWidget(plot_area, 1, 0)
        layout.addWidget(bscan_panel, 2, 0)
        layout.setRowStretch(1, 3)
        layout.setRowStretch(2, 1)
        layout.setColumnStretch(0, 1)
        return page

    def config(self) -> SweepConfig:
        return SweepConfig(
            start_mhz=self.start_mhz.value(),
            stop_mhz=self.stop_mhz.value(),
            points=self.points.value(),
            ifbw_hz=self.ifbw.value(),
            power_dbm=self.power.value(),
            dwell_us=self.dwell.value(),
            ifft_points=self.ifft_points.value(),
            time_window_ns=self.time_window.value(),
        )

    def update_sparam_right_axis(self) -> None:
        if not hasattr(self, "sparam_right_view"):
            return
        plot_item = self.sparam_plot.getPlotItem()
        self.sparam_right_view.setGeometry(plot_item.vb.sceneBoundingRect())
        self.sparam_right_view.linkedViewChanged(plot_item.vb, self.sparam_right_view.XAxis)

    def update_sparam_ranges(self, frequency_hz: np.ndarray, display: dict[str, np.ndarray]) -> None:
        if frequency_hz.size:
            self.sparam_plot.setXRange(float(frequency_hz[0]), float(frequency_hz[-1]), padding=0)
        reflection_range = robust_db_range(
            [db20(display["S11"]), db20(display["S22"])],
            default=(-60.0, 10.0),
        )
        transmission_range = robust_db_range(
            [db20(display["S21"]), db20(display["S12"])],
            default=(-120.0, 10.0),
        )
        self.sparam_plot.setYRange(*reflection_range, padding=0)
        self.sparam_right_view.setYRange(*transmission_range, padding=0)
        self.update_sparam_right_axis()

    def update_ascan_range(self, time_ns: np.ndarray) -> None:
        if time_ns.size:
            self.ascan_plot.setXRange(float(time_ns[0]), float(time_ns[-1]), padding=0)
        self.ascan_plot.setYRange(-1.05, 1.05, padding=0)

    def log(self, text: str) -> None:
        self.device_log.appendPlainText(f"[{datetime.now().strftime('%H:%M:%S')}] {text}")
        self.statusBar().showMessage(text)

    def refresh_ports(self) -> None:
        self.port_combo.clear()
        ports = list(list_ports.comports())
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}", port.device)
        com3 = self.port_combo.findData("COM3")
        if com3 >= 0:
            self.port_combo.setCurrentIndex(com3)
        self.log(f"发现 {len(ports)} 个串口")

    def selected_port(self) -> str:
        return self.port_combo.currentData() or ""

    def test_gnss(self) -> None:
        try:
            reader = GnssReader(self.selected_port(), int(self.baud_combo.currentText()))
            fix = reader.read_gga()
            reader.close()
            self.log(f"GNSS: {fix.raw}")
            self.log(f"fix={fix.fix_quality}, sats={fix.satellites}, lat={fix.latitude}, lon={fix.longitude}")
            if hasattr(self, "gnss_map"):
                self.gnss_map.update_fix(fix)
        except Exception as exc:
            self.log(f"GNSS 测试失败: {exc}")

    def test_vna(self) -> None:
        try:
            dev, interface = open_device()
            try:
                info = request_device_info(dev, PacketStream())
            finally:
                import usb.util
                usb.util.release_interface(dev, interface)
                usb.util.dispose_resources(dev)
            self.log(f"LibreVNA: FW {info['firmware']}, max {info['max_freq_hz'] / 1e9:g} GHz, points {info['max_points']}")
        except Exception as exc:
            self.log(f"LibreVNA 测试失败: {exc}")

    def test_sweep_settings(self) -> None:
        try:
            cfg = self.config()
            if cfg.stop_mhz <= cfg.start_mhz:
                self.log("Stop MHz 必须大于 Start MHz")
                return
            points = acquire_sparameters(
                int(cfg.start_mhz * 1e6),
                int(cfg.stop_mhz * 1e6),
                cfg.points,
                cfg.ifbw_hz,
                cfg.power_dbm,
                cfg.dwell_us,
            )
            freq, raw = sparameter_arrays(points)
            display = self.apply_active_calibration(freq, raw)
            for name in S_PARAMETERS:
                self.sparam_curves[name].setData(freq, db20(display[name]))
                time_ns, trace = make_time_trace(display[name], cfg)
                self.ascan_curves[name].setData(time_ns, normalize_trace_for_display(trace))
            self.update_sparam_ranges(freq, display)
            self.update_ascan_range(time_ns)
            self.log(
                f"当前参数扫频成功: {cfg.start_mhz:g}-{cfg.stop_mhz:g} MHz, "
                f"{cfg.points} 点, IFBW {cfg.ifbw_hz} Hz, power {cfg.power_dbm:g} dBm, "
                "通道 S11/S21/S12/S22"
            )
        except Exception as exc:
            self.log(f"参数测试扫频失败: {exc}")

    def choose_cal(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "选择校准文件",
            str(Path.cwd()),
            "Calibration (*.json *.cal);;All files (*)",
        )
        if path:
            self.cal_path.setText(path)
            try:
                measurements = read_calibration_measurements(Path(path))
                if measurements:
                    for new_key, legacy_key in LEGACY_OSL_MAP.items():
                        if legacy_key in measurements and new_key not in measurements:
                            measurements[new_key] = measurements[legacy_key]
                    self.cal_measurements = measurements
                    self.session_calibration = calibration_from_measurements(measurements, Path(path))
                    try:
                        self.session_calibration_p2 = calibration_from_measurements(measurements, Path(path), port=2)
                    except Exception:
                        self.session_calibration_p2 = None
                    self.through_measurement = measurements.get("Through")
                    self.update_calibration_status(f"已加载并启用 {Path(path).name}")
                else:
                    self.session_calibration = None
                    self.session_calibration_p2 = None
                    self.through_measurement = None
                    self.update_calibration_status(f"将使用外部校准文件 {Path(path).name}")
            except Exception:
                self.session_calibration = None
                self.session_calibration_p2 = None
                self.through_measurement = None
                self.update_calibration_status(f"将使用外部校准文件 {Path(path).name}")

    def choose_output(self) -> None:
        path = QtWidgets.QFileDialog.getExistingDirectory(self, "选择输出目录", self.output_dir.text())
        if path:
            self.output_dir.setText(path)

    def update_calibration_status(self, message: str | None = None) -> None:
        self.update_calibration_lamps()
        done = ", ".join(CALIBRATION_STEP_LABELS[name] for name, _label in CALIBRATION_STEPS if name in self.cal_measurements)
        required = list(P1_OSL_KEYS) if self.cal_mode.currentIndex() == 0 else [key for key, _label in CALIBRATION_STEPS]
        ready = sum(1 for key in required if key in self.cal_measurements)
        prefix = f"已采集: {done or '无'}; 当前模式 {ready}/{len(required)}"
        if self.session_calibration:
            prefix += f"; 已启用 {len(self.session_calibration.points)} 点校准"
        self.cal_status.setText(message or prefix)
        self.log(message or prefix)

    def update_calibration_lamps(self) -> None:
        for key, label in CALIBRATION_STEPS:
            lamp = self.cal_lamps.get(key)
            if not lamp:
                continue
            captured = key in self.cal_measurements
            color = "#22c55e" if captured else "#9ca3af"
            border = "#166534" if captured else "#6b7280"
            lamp.setStyleSheet(f"border-radius: 8px; background: {color}; border: 1px solid {border};")
            lamp.setToolTip(f"{label} {'已采集' if captured else '未采集'}")

    def capture_calibration_standard(self, standard: str) -> None:
        try:
            cfg = self.config()
            if cfg.stop_mhz <= cfg.start_mhz:
                self.log("Stop MHz 必须大于 Start MHz")
                return
            points = acquire_sparameters(
                int(cfg.start_mhz * 1e6),
                int(cfg.stop_mhz * 1e6),
                cfg.points,
                cfg.ifbw_hz,
                cfg.power_dbm,
                cfg.dwell_us,
            )
            primary = "S11"
            if standard.startswith("P2_"):
                primary = "S22"
            elif standard == "Through":
                primary = "S21"
            self.cal_measurements[standard] = CalibrationMeasurement.from_sparameter_points(points, primary=primary)
            self.session_calibration = None
            self.session_calibration_p2 = None
            self.through_measurement = self.cal_measurements.get("Through")
            label = CALIBRATION_STEP_LABELS.get(standard, standard)
            self.update_calibration_status(f"{label} 已采集: {len(points)} 点，已记录 S11/S21/S12/S22")
        except Exception as exc:
            label = CALIBRATION_STEP_LABELS.get(standard, standard)
            self.log(f"{label} 校准采集失败: {exc}")

    def build_session_calibration(self) -> None:
        try:
            if self.cal_mode.currentIndex() == 1:
                missing_two_port = [CALIBRATION_STEP_LABELS[key] for key, _label in CALIBRATION_STEPS if key not in self.cal_measurements]
                if missing_two_port:
                    self.log(f"两端口校准还缺少: {', '.join(missing_two_port)}")
                    return
            self.session_calibration = calibration_from_measurements(self.cal_measurements)
            try:
                self.session_calibration_p2 = calibration_from_measurements(self.cal_measurements, port=2)
            except Exception:
                self.session_calibration_p2 = None
            self.through_measurement = self.cal_measurements.get("Through")
            self.cal_path.clear()
            message = "P1 S11 一端口 OSL 校准已生成并启用"
            if self.session_calibration_p2:
                message += "；P2 S22 一端口 OSL 校准已生成并启用"
            if self.cal_mode.currentIndex() == 1:
                message += "；七项两端口数据已记录，采集文件保存 S11/S21/S12/S22"
            self.update_calibration_status(message)
        except Exception as exc:
            self.log(f"生成校准失败: {exc}")

    def save_session_calibration(self) -> None:
        try:
            required = list(P1_OSL_KEYS) if self.cal_mode.currentIndex() == 0 else [key for key, _label in CALIBRATION_STEPS]
            missing = [CALIBRATION_STEP_LABELS[key] for key in required if key not in self.cal_measurements]
            if missing:
                self.log(f"需要先采集: {', '.join(missing)}")
                return
            default = Path.cwd() / "calibration" / f"solt_2port_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self,
                "保存校准文件",
                str(default),
                "Calibration JSON (*.json);;All files (*)",
            )
            if not path:
                return
            write_calibration_file(Path(path), self.cal_measurements, self.config())
            self.cal_path.setText(path)
            self.session_calibration = calibration_from_measurements(self.cal_measurements, Path(path))
            try:
                self.session_calibration_p2 = calibration_from_measurements(self.cal_measurements, Path(path), port=2)
            except Exception:
                self.session_calibration_p2 = None
            self.through_measurement = self.cal_measurements.get("Through")
            self.update_calibration_status(f"校准已保存并启用: {Path(path).name}")
        except Exception as exc:
            self.log(f"保存校准失败: {exc}")

    def apply_active_calibration(self, frequency_hz: np.ndarray, raw_sparams: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        calibration_p1 = self.session_calibration
        calibration_p2 = self.session_calibration_p2
        if calibration_p1 is None and self.cal_path.text().strip():
            calibration_p1 = OSLCalibration.from_file(self.cal_path.text().strip())
        return apply_reflection_and_through_calibration(
            frequency_hz,
            raw_sparams,
            calibration_p1,
            calibration_p2,
            self.through_measurement,
        )

    def capture_background(self) -> None:
        try:
            cfg = self.config()
            points = acquire_sparameters(int(cfg.start_mhz * 1e6), int(cfg.stop_mhz * 1e6), cfg.points, cfg.ifbw_hz, cfg.power_dbm, cfg.dwell_us)
            freq, raw = sparameter_arrays(points)
            self.background = self.apply_active_calibration(freq, raw)["S11"]
            self.bg_label.setText(f"已采集 {len(self.background)} 点背景")
            self.log("背景扣除数据已采集")
        except Exception as exc:
            self.log(f"背景采集失败: {exc}")

    def clear_background(self) -> None:
        self.background = None
        self.bg_label.setText("未采集背景")
        self.log("已清除背景")

    def start_acquisition(self, single: bool = False) -> None:
        if self.worker and self.worker.isRunning():
            self.log("采集已经在运行")
            return
        cfg = self.config()
        if cfg.stop_mhz <= cfg.start_mhz:
            self.log("Stop MHz 必须大于 Start MHz")
            return
        if self.remote_enable.isChecked():
            self.connect_remote_sender()
        self.worker = AcquisitionWorker(
            cfg=cfg,
            gnss_port=self.selected_port(),
            baud=int(self.baud_combo.currentText()),
            out_dir=Path(self.output_dir.text()),
            count=1 if single else self.count.value(),
            interval_s=self.interval.value(),
            calibration_path=self.cal_path.text().strip(),
            session_calibration=self.session_calibration,
            session_calibration_p2=self.session_calibration_p2,
            through_measurement=self.through_measurement,
            background=self.background,
            autosave=self.autosave.isChecked(),
        )
        self.worker.trace_ready.connect(self.on_trace)
        self.worker.status.connect(self.log)
        self.worker.failed.connect(lambda msg: self.log(f"采集失败: {msg}"))
        self.worker.finished_run.connect(self.on_worker_finished)
        self.worker.start()

    def connect_remote_sender(self) -> None:
        if not hasattr(self, "remote_enable") or not self.remote_enable.isChecked():
            return
        if self.remote_sender is not None:
            self.log("Remote sender already running")
            return
        host = self.remote_host.text().strip()
        if not host:
            self.log("Remote IP is empty")
            return
        self.remote_run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.remote_sender = RemoteSender(
            host,
            int(self.remote_port.value()),
            self.remote_run_id,
            self.remote_status.emit,
            self.remote_command.emit,
        )
        self.remote_sender.start()
        self.log(f"Remote sender connecting: {host}:{self.remote_port.value()}")

    def handle_remote_command(self, action: str, params: object) -> None:
        self.log(f"Remote command: {action}")
        if isinstance(params, dict):
            if "count" in params:
                self.count.setValue(max(0, min(int(params["count"]), self.count.maximum())))
            if "interval_s" in params:
                self.interval.setValue(float(params["interval_s"]))
        if action == "start":
            single = bool(params.get("single", False)) if isinstance(params, dict) else False
            self.start_acquisition(single=single)
        elif action == "stop":
            self.stop_acquisition()
        elif action == "single":
            self.start_acquisition(single=True)
        else:
            self.log(f"Unknown remote command: {action}")

    def stop_acquisition(self) -> None:
        if self.worker:
            self.worker.stop()
            self.log("正在停止采集...")

    def on_worker_finished(self) -> None:
        self.log("采集结束")
        self.worker = None

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self.remote_sender is not None:
            self.remote_sender.stop()
            self.remote_sender = None
        super().closeEvent(event)

    def clear_bscan(self) -> None:
        self.bscan = {name: [] for name in S_PARAMETERS}
        for view in self.bscan_views.values():
            view.clear()
        self.log("B-scan 已清空")

    def refresh_bscan_views(self) -> None:
        cmap = self.bscan_cmap.currentText() if hasattr(self, "bscan_cmap") else "gray"
        for name in S_PARAMETERS:
            traces = self.bscan.get(name, [])
            view = self.bscan_views.get(name)
            if not traces or view is None:
                continue
            image = np.vstack(traces).T
            view.set_colormap(cmap)
            view.set_data(image, getattr(self, "last_time_ns", None))

    def on_trace(self, index, freq, raw, display, time_ns, trace_by_channel, fix, path_text) -> None:
        self.last_display_sparams = display
        self.last_time_ns = time_ns
        for name in S_PARAMETERS:
            self.sparam_curves[name].setData(freq, db20(display[name]))
            self.ascan_curves[name].setData(time_ns, normalize_trace_for_display(trace_by_channel[name]))
            self.bscan[name].append(np.asarray(trace_by_channel[name], dtype=float))
        self.update_sparam_ranges(freq, display)
        self.update_ascan_range(time_ns)
        self.refresh_bscan_views()
        fix_text = f"fix={fix.fix_quality}, sats={fix.satellites}, lat={fix.latitude}, lon={fix.longitude}"
        self.last_info.setText(f"#{index} {fix_text}")
        self.gnss_map.update_fix(fix)
        if self.remote_sender is not None:
            self.remote_sender.enqueue_trace(index, self.config(), freq, raw, display, time_ns, trace_by_channel, fix)
        self.log(f"Trace {index} complete. {fix_text}" + (f" saved: {Path(path_text).name}" if path_text else ""))


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True, background="w", foreground="k")
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
