#!/usr/bin/env python3
"""TCP receiver for UAV-GPR trace telemetry."""

from __future__ import annotations

import argparse
import hashlib
import json
import socket
import struct
from datetime import datetime
from pathlib import Path


PROTOCOL = "uav-gpr-telemetry-v1"


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


def safe_name(value: object, fallback: str) -> str:
    text = str(value or fallback)
    return "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in text)


class TraceReceiver:
    def __init__(self, host: str, port: int, out_dir: Path) -> None:
        self.host = host
        self.port = port
        self.out_dir = out_dir

    def serve_forever(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.host, self.port))
            server.listen(5)
            print(f"[receiver] listening on {self.host}:{self.port}")
            print(f"[receiver] output: {self.out_dir.resolve()}")
            while True:
                client, address = server.accept()
                with client:
                    client.settimeout(30.0)
                    print(f"[receiver] connected: {address[0]}:{address[1]}")
                    try:
                        self.handle_client(client)
                    except Exception as exc:
                        print(f"[receiver] disconnected: {exc}")

    def handle_client(self, sock: socket.socket) -> None:
        while True:
            header, payload = recv_frame(sock)
            msg_type = header.get("type")
            if header.get("protocol") != PROTOCOL:
                raise ValueError(f"unsupported protocol: {header.get('protocol')}")
            if msg_type == "hello":
                send_frame(sock, {"type": "ack", "protocol": PROTOCOL, "timestamp": datetime.now().isoformat(timespec="seconds")})
                print(f"[receiver] hello run_id={header.get('run_id')}")
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
                print(f"[receiver] trace {header.get('trace_index')} saved: {path}")
            else:
                raise ValueError(f"unsupported message type: {msg_type}")

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


def main() -> int:
    parser = argparse.ArgumentParser(description="Receive UAV-GPR traces over TCP.")
    parser.add_argument("--host", default="0.0.0.0", help="listen address")
    parser.add_argument("--port", type=int, default=8765, help="listen port")
    parser.add_argument("--out", type=Path, default=Path("remote_runs"), help="output directory")
    args = parser.parse_args()
    TraceReceiver(args.host, args.port, args.out).serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
