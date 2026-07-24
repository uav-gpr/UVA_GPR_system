#!/usr/bin/env python3
"""Ground station GUI for receiving and controlling UAV-GPR acquisition."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import socket
import struct
import sys
import threading
from datetime import datetime
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtWidgets


PROTOCOL = "uav-gpr-telemetry-v1"
S_PARAMETERS = ("S11", "S21", "S12", "S22")


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


def recv_frame(sock: socket.socket) -> tuple[dict[str, object], bytes]:
    header_len = struct.unpack("!I", recv_exact(sock, 4))[0]
    header = json.loads(recv_exact(sock, header_len).decode("utf-8"))
    payload_len = struct.unpack("!Q", recv_exact(sock, 8))[0]
    payload = recv_exact(sock, payload_len) if payload_len else b""
    return header, payload


def send_frame(sock: socket.socket, header: dict[str, object], payload: bytes = b"") -> None:
    header_bytes = json.dumps(header, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    sock.sendall(struct.pack("!I", len(header_bytes)))
    sock.sendall(header_bytes)
    sock.sendall(struct.pack("!Q", len(payload)))
    if payload:
        sock.sendall(payload)


def db20(values: np.ndarray) -> np.ndarray:
    return 20.0 * np.log10(np.maximum(np.abs(values), 1e-15))


def safe_name(value: object, fallback: str) -> str:
    text = str(value or fallback)
    return "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in text)


class ReceiverThread(QtCore.QThread):
    status = QtCore.Signal(str)
    trace_received = QtCore.Signal(dict, bytes, str)
    command_status = QtCore.Signal(str)

    def __init__(self, host: str, port: int, out_dir: Path) -> None:
        super().__init__()
        self.host = host
        self.port = port
        self.out_dir = out_dir
        self._stop = threading.Event()
        self._client: socket.socket | None = None
        self._send_lock = threading.Lock()

    def stop(self) -> None:
        self._stop.set()
        if self._client is not None:
            try:
                self._client.close()
            except OSError:
                pass

    def send_command(self, action: str, params: dict[str, object] | None = None) -> None:
        if self._client is None:
            self.command_status.emit("No remote acquisition computer connected")
            return
        header = {
            "type": "command",
            "protocol": PROTOCOL,
            "action": action,
            "params": params or {},
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        }
        try:
            with self._send_lock:
                send_frame(self._client, header)
            self.command_status.emit(f"Command sent: {action}")
        except Exception as exc:
            self.command_status.emit(f"Command failed: {exc}")

    def run(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.host, self.port))
            server.listen(1)
            server.settimeout(0.5)
            self.status.emit(f"Listening on {self.host}:{self.port}")
            while not self._stop.is_set():
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                with client:
                    self._client = client
                    client.settimeout(30.0)
                    self.status.emit(f"Connected: {address[0]}:{address[1]}")
                    try:
                        self.handle_client(client)
                    except Exception as exc:
                        self.status.emit(f"Disconnected: {exc}")
                    finally:
                        self._client = None

    def handle_client(self, sock: socket.socket) -> None:
        while not self._stop.is_set():
            header, payload = recv_frame(sock)
            msg_type = header.get("type")
            if header.get("protocol") != PROTOCOL:
                raise ValueError(f"unsupported protocol: {header.get('protocol')}")
            if msg_type == "hello":
                send_frame(sock, {"type": "ack", "protocol": PROTOCOL, "timestamp": datetime.now().isoformat(timespec="seconds")})
                self.status.emit(f"Remote ready, run_id={header.get('run_id')}")
            elif msg_type == "trace":
                path = self.save_trace(header, payload)
                send_frame(
                    sock,
                    {
                        "type": "ack_trace",
                        "protocol": PROTOCOL,
                        "run_id": header.get("run_id"),
                        "trace_index": header.get("trace_index"),
                        "path": path.name,
                    },
                )
                self.trace_received.emit(header, payload, str(path))
            elif msg_type == "command_ack":
                self.command_status.emit(f"Remote accepted: {header.get('action')}")
            else:
                self.status.emit(f"Ignored message: {msg_type}")

    def save_trace(self, header: dict[str, object], payload: bytes) -> Path:
        expected_sha = str(header.get("sha256") or "")
        actual_sha = hashlib.sha256(payload).hexdigest()
        if expected_sha and expected_sha != actual_sha:
            raise ValueError(f"sha256 mismatch for trace {header.get('trace_index')}")

        run_id = safe_name(header.get("run_id"), "run")
        trace_index = int(header.get("trace_index") or 0)
        run_dir = self.out_dir / run_id
        run_dir.mkdir(parents=True, exist_ok=True)
        npz_path = run_dir / f"trace_{trace_index:05d}.npz"
        if not npz_path.exists():
            npz_path.write_bytes(payload)

        metadata = dict(header)
        metadata["receiver_saved_at"] = datetime.now().isoformat(timespec="milliseconds")
        metadata["receiver_file"] = npz_path.name
        metadata["receiver_sha256"] = actual_sha
        with (run_dir / "run_metadata.jsonl").open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(metadata, ensure_ascii=False) + "\n")
        return npz_path


class GroundStationWindow(QtWidgets.QMainWindow):
    def __init__(self, host: str, port: int, out_dir: Path) -> None:
        super().__init__()
        self.setWindowTitle("UAV-GPR Ground Station")
        self.resize(1380, 860)
        self.bscan_rows: dict[str, list[np.ndarray]] = {name: [] for name in S_PARAMETERS}
        self.last_time_ns: np.ndarray | None = None
        self.receiver = ReceiverThread(host, port, out_dir)
        self.receiver.status.connect(self.log)
        self.receiver.command_status.connect(self.log)
        self.receiver.trace_received.connect(self.on_trace)
        self._build_ui(host, port, out_dir)
        self.receiver.start()

    def _build_ui(self, host: str, port: int, out_dir: Path) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QGridLayout(central)

        controls = QtWidgets.QGroupBox("Ground station")
        ctl = QtWidgets.QGridLayout(controls)
        start = QtWidgets.QPushButton("Start")
        single = QtWidgets.QPushButton("Single")
        stop = QtWidgets.QPushButton("Stop")
        clear = QtWidgets.QPushButton("Clear B-scan")
        self.interval = QtWidgets.QDoubleSpinBox()
        self.interval.setRange(0.0, 3600.0)
        self.interval.setDecimals(2)
        self.interval.setValue(1.0)
        self.count = QtWidgets.QSpinBox()
        self.count.setRange(0, 1000000)
        self.count.setValue(0)
        self.bscan_cmap = QtWidgets.QComboBox()
        self.bscan_cmap.addItems(["gray", "jet"])
        self.bscan_cmap.currentTextChanged.connect(self.refresh_bscan_views)
        start.clicked.connect(self.send_start)
        single.clicked.connect(self.send_single)
        stop.clicked.connect(lambda: self.receiver.send_command("stop"))
        clear.clicked.connect(self.clear_bscan)
        self.status_label = QtWidgets.QLabel(f"Listening {host}:{port} | {out_dir}")
        self.trace_label = QtWidgets.QLabel("No trace")
        ctl.addWidget(QtWidgets.QLabel("Interval s"), 0, 0)
        ctl.addWidget(self.interval, 0, 1)
        ctl.addWidget(QtWidgets.QLabel("A-scan count (0=continuous)"), 0, 2)
        ctl.addWidget(self.count, 0, 3)
        ctl.addWidget(single, 0, 4)
        ctl.addWidget(start, 0, 5)
        ctl.addWidget(stop, 0, 6)
        ctl.addWidget(QtWidgets.QLabel("B-scan colormap"), 1, 0)
        ctl.addWidget(self.bscan_cmap, 1, 1)
        ctl.addWidget(clear, 1, 2)
        ctl.addWidget(self.trace_label, 1, 3, 1, 4)
        ctl.addWidget(self.status_label, 2, 0, 1, 7)
        ctl.setColumnStretch(3, 1)

        colors = {
            "S11": "#0077b6",
            "S21": "#2a9d8f",
            "S12": "#e76f51",
            "S22": "#6d597a",
        }
        self.sparam_plot = pg.PlotWidget(title="S-parameter magnitude")
        self.sparam_plot.setLabel("bottom", "Frequency", units="Hz")
        self.sparam_plot.setLabel("left", "Magnitude", units="dB")
        self.sparam_plot.showGrid(x=True, y=True, alpha=0.25)
        self.sparam_plot.addLegend()
        self.sparam_curves = {
            name: self.sparam_plot.plot(pen=pg.mkPen(colors[name], width=2), name=name)
            for name in S_PARAMETERS
        }

        self.ascan_plot = pg.PlotWidget(title="A-scan")
        self.ascan_plot.setLabel("bottom", "Time", units="ns")
        self.ascan_plot.setLabel("left", "Amplitude")
        self.ascan_plot.showGrid(x=True, y=True, alpha=0.25)
        self.ascan_plot.addLegend()
        self.ascan_curves = {
            name: self.ascan_plot.plot(pen=pg.mkPen(colors[name], width=2), name=name)
            for name in S_PARAMETERS
        }

        bscan_panel = QtWidgets.QWidget()
        bscan_grid = QtWidgets.QGridLayout(bscan_panel)
        bscan_grid.setContentsMargins(0, 0, 0, 0)
        bscan_grid.setSpacing(8)
        self.bscan_plots: dict[str, pg.PlotWidget] = {}
        self.bscan_images: dict[str, pg.ImageItem] = {}
        for index, name in enumerate(S_PARAMETERS):
            plot = pg.PlotWidget(title=f"{name} B-scan")
            plot.setLabel("bottom", "Trace")
            plot.setLabel("left", "Time", units="ns")
            plot.showGrid(x=True, y=True, alpha=0.15)
            plot.setMouseEnabled(x=False, y=False)
            plot.setMenuEnabled(False)
            plot.getPlotItem().hideButtons()
            image = pg.ImageItem()
            plot.addItem(image)
            self.bscan_plots[name] = plot
            self.bscan_images[name] = image
            bscan_grid.addWidget(plot, index // 2, index % 2)

        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumHeight(140)

        layout.addWidget(controls, 0, 0, 1, 2)
        layout.addWidget(self.sparam_plot, 1, 0)
        layout.addWidget(self.ascan_plot, 1, 1)
        layout.addWidget(bscan_panel, 2, 0, 1, 2)
        layout.addWidget(self.log_box, 3, 0, 1, 2)
        layout.setRowStretch(1, 2)
        layout.setRowStretch(2, 2)

    def log(self, text: str) -> None:
        self.status_label.setText(text)
        self.log_box.appendPlainText(f"[{datetime.now().strftime('%H:%M:%S')}] {text}")

    def command_params(self, single: bool = False) -> dict[str, object]:
        return {
            "count": 1 if single else int(self.count.value()),
            "interval_s": float(self.interval.value()),
            "single": single,
        }

    def send_start(self) -> None:
        self.receiver.send_command("start", self.command_params())

    def send_single(self) -> None:
        self.receiver.send_command("single", self.command_params(single=True))

    def clear_bscan(self) -> None:
        self.bscan_rows = {name: [] for name in S_PARAMETERS}
        for image in self.bscan_images.values():
            image.clear()
        self.log("B-scan cleared")

    def refresh_bscan_views(self) -> None:
        cmap_name = self.bscan_cmap.currentText() if hasattr(self, "bscan_cmap") else "gray"
        lut = None
        if cmap_name == "jet":
            lut = pg.colormap.get("CET-R4").getLookupTable(0.0, 1.0, 256)
        for name, rows in self.bscan_rows.items():
            image_item = self.bscan_images.get(name)
            if image_item is None or not rows:
                continue
            image = np.vstack(rows)
            image_item.setImage(image, autoLevels=True)
            image_item.setLookupTable(lut)
            if self.last_time_ns is not None and self.last_time_ns.size:
                time_start = float(self.last_time_ns[0])
                time_stop = float(self.last_time_ns[-1])
                time_span = max(time_stop - time_start, 1e-9)
                image_item.setRect(QtCore.QRectF(0.0, time_start, float(len(rows)), time_span))

    def on_trace(self, header: dict, payload: bytes, path: str) -> None:
        data = np.load(io.BytesIO(payload), allow_pickle=False)
        freq = data["frequency_hz"]
        time_ns = data["time_ns"]
        self.last_time_ns = time_ns
        for name in S_PARAMETERS:
            real = data[f"display_{name}_real"]
            imag = data[f"display_{name}_imag"]
            values = real + 1j * imag
            self.sparam_curves[name].setData(freq, db20(values))
            trace_key = f"trace_{name}"
            if trace_key in data:
                self.ascan_curves[name].setData(time_ns, data[trace_key])

        for name in S_PARAMETERS:
            trace_key = f"trace_{name}"
            if trace_key in data:
                self.bscan_rows[name].append(np.asarray(data[trace_key], dtype=float))
        self.refresh_bscan_views()

        gnss = json.loads(str(data["gnss"]))
        trace_index = header.get("trace_index")
        self.trace_label.setText(
            f"Trace {trace_index} | fix={gnss.get('fix_quality')} sats={gnss.get('satellites')} "
            f"lat={gnss.get('latitude')} lon={gnss.get('longitude')} | {Path(path).name}"
        )
        self.log(f"Trace {trace_index} received")

    def closeEvent(self, event) -> None:
        self.receiver.stop()
        self.receiver.wait(1500)
        super().closeEvent(event)


def main() -> int:
    parser = argparse.ArgumentParser(description="UAV-GPR ground station GUI.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--out", type=Path, default=Path("remote_runs"))
    args = parser.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True, background="w", foreground="k")
    window = GroundStationWindow(args.host, args.port, args.out)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
