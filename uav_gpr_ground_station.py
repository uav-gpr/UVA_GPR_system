#!/usr/bin/env python3
"""Ground station GUI for receiving and controlling UAV-GPR acquisition."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtWidgets

from bscan_view import (
    BSCAN_WINDOW_TRACES,
    apply_flat_filter_windowed,
    bscan_start_after_append,
    bscan_processing_signature,
    bscan_time_axes_compatible,
    bscan_window_bounds,
    bscan_window_from_rows,
    bscan_window_max_start,
    select_bscan_window,
    validate_bscan_trace,
)
from ground_trace_pipeline import (
    DecodedGroundTrace,
    GroundDisplayFIFO,
    GroundTraceDecoder,
    decode_ground_trace_payload,
    set_metrics as set_ground_decoder_metrics,
)
from gnss_map_widget import GnssMapWidget
from gpr_processing import flat_reflection_filter
from pipeline_metrics import PipelineMetrics


PROTOCOL = "uav-gpr-telemetry-v1"
S_PARAMETERS = ("S11", "S21", "S12", "S22")
UI_CHANNELS = ("S11", "S22")
CHANNEL_LABELS = {"S11": "HH (S11)", "S22": "VV (S22)"}


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


def _scalar(value: object) -> object:
    """Unwrap numpy scalars/arrays and decode byte strings from old NPZ files."""
    while hasattr(value, "item"):
        try:
            unwrapped = value.item()
        except Exception:
            break
        if unwrapped is value:
            break
        value = unwrapped
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def _json_object(value: object) -> dict[str, object]:
    value = _scalar(value)
    if isinstance(value, dict):
        return dict(value)
    if not isinstance(value, str) or not value.strip():
        return {}
    try:
        parsed = json.loads(value)
    except (TypeError, ValueError):
        return {}
    return dict(parsed) if isinstance(parsed, dict) else {}


def gnss_from_sources(data: object, header: dict[str, object]) -> dict[str, object]:
    """Prefer NPZ GNSS metadata and fall back to the telemetry trace header."""
    values: list[object] = []
    files = getattr(data, "files", ())
    if "gnss" in files:
        values.append(data["gnss"])
    values.append(header.get("gnss"))
    for value in values:
        parsed = _json_object(value)
        if parsed:
            return parsed
    return {}


def acquisition_from_sources(data: object, header: dict[str, object]) -> dict[str, object]:
    """Prefer NPZ acquisition metadata and fall back to the telemetry header.

    旧 payload 缺少 ``acquisition`` 时返回空 dict，调用方必须容忍。
    """
    values: list[object] = []
    files = getattr(data, "files", ())
    if "acquisition" in files:
        values.append(data["acquisition"])
    values.append(header.get("acquisition"))
    for value in values:
        parsed = _json_object(value)
        if parsed:
            return parsed
    return {}


def flight_from_sources(data: object, header: dict[str, object]) -> dict[str, object]:
    """Prefer NPZ flight metadata and fall back to the telemetry header.

    旧 payload 缺少 ``flight`` 时返回空 dict；无真实 AGL 来源时
    ``flight_height_agl_m`` 保持缺失/None，地面端显示为 unknown。
    """
    values: list[object] = []
    files = getattr(data, "files", ())
    if "flight" in files:
        values.append(data["flight"])
    values.append(header.get("flight"))
    for value in values:
        parsed = _json_object(value)
        if parsed:
            return parsed
    return {}


def _parse_channels_declaration(present: bool, value: object, source_name: str) -> list[str] | None:
    """严格解析 channels 声明，区分"key 缺失"与"显式损坏"。

    ``present=False``（来源没有 channels key）→ 返回 None，表示缺失。
    ``present=True`` → 必须解析为**非空、无重复、元素均为非空字符串**的列表；
    显式 None / 空字符串 / 空列表 / 非法 JSON / 数字 / bool / 重复项全部抛
    ValueError，绝不当作缺失处理。

    NPZ 中 channels 是 JSON 字符串，telemetry header 中是字符串列表。
    """
    if not present:
        return None
    value = _scalar(value)
    if value is None:
        raise ValueError(f"{source_name} channels 存在但为 None")
    if isinstance(value, bytes):
        try:
            value = json.loads(value.decode("utf-8"))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{source_name} channels 不是合法 JSON") from exc
    if isinstance(value, str):
        if not value.strip():
            raise ValueError(f"{source_name} channels 存在但为空字符串")
        try:
            value = json.loads(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{source_name} channels 不是合法 JSON") from exc
    if not isinstance(value, list):
        raise ValueError(f"{source_name} channels 必须是字符串列表")
    if not value:
        raise ValueError(f"{source_name} channels 存在但为空列表")
    channels: list[str] = []
    seen: set[str] = set()
    for entry in value:
        entry = _scalar(entry)
        if isinstance(entry, bool) or not isinstance(entry, str) or not entry.strip():
            raise ValueError(f"{source_name} channels 包含非法项: {entry!r}")
        normalized = entry.strip()
        if normalized in seen:
            raise ValueError(f"{source_name} channels 含重复通道: {normalized}")
        seen.add(normalized)
        channels.append(normalized)
    return channels


def _number_text(value: object, precision: str = "g") -> str:
    value = _scalar(value)
    if value is None or (isinstance(value, str) and not value):
        return "-"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(number):
        return "-"
    return f"{number:{precision}}"


def _session_stats_text(acquisition: dict[str, object]) -> list[str]:
    """从 acquisition.session_statistics 提取精简完整性统计（缺失/非法值跳过）。

    缺失字段兼容；bool/NaN/Inf/非有限值绝不显示成可信统计。
    """
    session = _json_object(acquisition.get("session_statistics"))
    if not session:
        return []
    backend = session.get("backend")
    backend = dict(backend) if isinstance(backend, dict) else {}
    queue = session.get("processing_queue")
    queue = dict(queue) if isinstance(queue, dict) else {}
    parts: list[str] = []

    def _finite_number(value: object) -> float | None:
        value = _scalar(value)
        if isinstance(value, bool):
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    # 内部 key 保留，仅用户可见标签映射为中文。
    for key, label in (
        ("completed_sweeps", "完整Sweep"),
        ("dropped_sweeps", "丢弃Sweep"),
        ("incomplete_sweeps", "不完整Sweep"),
        ("sweep_timeouts", "Sweep超时"),
        ("pending_overflows", "Pending溢出"),
    ):
        number = _finite_number(backend.get(key))
        if number is not None:
            parts.append(f"{label}={number:g}")
    for key, label in (
        ("overflows", "队列溢出"),
        ("dropped", "队列丢弃"),
    ):
        number = _finite_number(queue.get(key))
        if number is not None:
            parts.append(f"{label}={number:g}")
    return parts


def _validate_optional_display_fields(
    data: object,
    files: set[str],
    freq: np.ndarray,
) -> None:
    """校验可选的 display_* 字段，验证后丢弃（不保存为任何 UI 状态）。

    只做结构校验，不保留数组：display real/imag 只存在一半、数组非一维、形状与
    ``freq`` 不一致、或含 NaN/Inf 时抛 ValueError（在任何 UI/B-scan 提交前原子拒绝）。
    完全缺失的可选 display 字段按既有 schema 规则跳过；已声明的 channels 与实际
    字段的一致性检查在调用方（on_trace）单独完成，不做放松。
    """
    for name in UI_CHANNELS:
        real_key = f"display_{name}_real"
        imag_key = f"display_{name}_imag"
        real_present = real_key in files
        imag_present = imag_key in files
        if real_present != imag_present:
            raise ValueError(f"display data for {name} 只存在一半")
        if not real_present:
            continue
        real = np.asarray(data[real_key], dtype=float)
        imag = np.asarray(data[imag_key], dtype=float)
        if (
            real.ndim != 1
            or imag.ndim != 1
            or real.shape != freq.shape
            or imag.shape != freq.shape
            or not np.all(np.isfinite(real))
            or not np.all(np.isfinite(imag))
        ):
            raise ValueError(f"display data for {name} has invalid shape or values")


def _format_acquisition_label(acquisition: dict[str, object], flight: dict[str, object]) -> str:
    """把 acquisition/flight 元数据渲染成一行中文状态文本（缺失字段一律容忍）。

    完全没有 metadata（旧 payload）时显示 ``采集：-``；否则显示已知字段，
    无真实 AGL 来源时 ``对地高度：未知``；最后附精简会话完整性统计。
    """
    if not acquisition and not flight:
        return "采集：-"
    parts: list[str] = []
    acquired = acquisition.get("acquired_at_utc")
    if acquired:
        parts.append(f"采集时间：{acquired}")
    # 会话完整性统计与设备/调度字段使用中文标签；保留专业缩写与数值。
    if _number_text(acquisition.get("device_sweep_index")) != "-":
        parts.append(f"设备Sweep编号：{_number_text(acquisition.get('device_sweep_index'))}")
    if _number_text(acquisition.get("selected_trace_index")) != "-":
        parts.append(f"选中Trace编号：{_number_text(acquisition.get('selected_trace_index'))}")
    if _number_text(acquisition.get("connection_generation")) != "-":
        parts.append(f"连接代数：{_number_text(acquisition.get('connection_generation'))}")
    age = acquisition.get("gnss_age_ms")
    if age is not None:
        parts.append(f"GNSS时延：{_number_text(age)} ms")
    valid = flight.get("gnss_valid")
    if valid is not None:
        parts.append("GNSS有效" if bool(valid) else "GNSS无效")
    msl = flight.get("gnss_altitude_msl_m")
    if msl is not None:
        parts.append(f"GNSS海拔：{_number_text(msl)} m")
    agl = flight.get("flight_height_agl_m")
    parts.append("对地高度：未知" if agl is None else f"对地高度：{_number_text(agl)} m")
    interval = acquisition.get("target_interval_s")
    if interval is not None:
        parts.append(f"目标间隔：{_number_text(interval)} s")
    actual = acquisition.get("actual_interval_s")
    if actual is not None:
        parts.append(f"实际间隔：{_number_text(actual)} s")
    error = acquisition.get("schedule_error_ms")
    if error is not None:
        parts.append(f"调度误差：{_number_text(error)} ms")
    parts.extend(_session_stats_text(acquisition))
    if not parts:
        return "采集：-"
    return "采集：" + " | ".join(parts)


def _bool_or_none(value: object) -> bool | None:
    value = _scalar(value)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, np.integer)) and not isinstance(value, bool):
        if int(value) in (0, 1):
            return bool(value)
        return None
    if isinstance(value, (float, np.floating)):
        if np.isfinite(value) and float(value) in (0.0, 1.0):
            return bool(value)
        return None
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off"}:
            return False
    return None


def _dewow_source_object(value: object, source_name: str) -> dict[str, object]:
    """Decode one Dewow metadata source, rejecting malformed explicit objects."""
    value = _scalar(value)
    if value is None:
        return {}
    if isinstance(value, dict):
        return dict(value)
    if isinstance(value, str):
        if not value.strip():
            return {}
        try:
            parsed = json.loads(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{source_name} metadata is not valid JSON") from exc
        if not isinstance(parsed, dict):
            raise ValueError(f"{source_name} metadata must be an object")
        return dict(parsed)
    raise ValueError(f"{source_name} metadata must be an object")


def _dewow_settings_from_object(
    value: object,
    source_name: str,
) -> tuple[tuple[bool, float | None], str] | None:
    """Parse one source, returning None only when it has no Dewow fields."""
    parsed = _dewow_source_object(value, source_name)
    nested_key = "dew" + "ow"
    enabled_key = "dew" + "ow_enabled"
    window_key = "dew" + "ow_window_ns"
    has_nested = nested_key in parsed
    has_top_enabled = enabled_key in parsed
    has_top_window = window_key in parsed
    if not (has_nested or has_top_enabled or has_top_window):
        return None

    nested: dict[str, object] = {}
    if has_nested:
        raw_nested = _scalar(parsed[nested_key])
        if not isinstance(raw_nested, dict):
            raise ValueError(f"{source_name}.Dewow must be an object")
        nested = dict(raw_nested)

    if has_nested:
        if "enabled" not in nested:
            raise ValueError(f"{source_name}.Dewow metadata is missing enabled")
        enabled = _bool_or_none(nested["enabled"])
        if enabled is None:
            raise ValueError(f"{source_name}.Dewow.enabled is invalid")
    elif has_top_enabled:
        enabled = _bool_or_none(parsed[enabled_key])
        if enabled is None:
            raise ValueError(f"{source_name}.Dewow enabled flag is invalid")
    elif has_top_window:
        # The original sender emitted only dewow_window_ns.  That format
        # means Dewow was enabled, and the recorded window is authoritative.
        try:
            signature = bscan_processing_signature(True, parsed[window_key])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"{source_name}.Dewow legacy window_ns must be a finite positive number"
            ) from exc
        return signature, "legacy"
    else:
        raise ValueError(f"{source_name} Dewow metadata is missing enabled")

    if not enabled:
        # An explicitly disabled source is authoritative.  Its window is not
        # part of the signature and therefore must not invalidate the trace.
        return (False, None), "disabled"

    if "window_ns" in nested:
        window = nested["window_ns"]
    elif has_top_window:
        window = parsed[window_key]
    else:
        window = 4.0
    try:
        signature = bscan_processing_signature(True, window)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{source_name}.Dewow window_ns must be a finite positive number") from exc
    return signature, "enabled"


def dewow_settings_from_sources(
    data: object,
    header: dict[str, object],
) -> tuple[tuple[bool, float | None], str]:
    """Read source-side Dewow metadata without applying Dewow again.

    Payloads produced before the explicit flag existed represented Dewow
    output, so missing metadata is intentionally interpreted as legacy
    enabled behavior.
    """
    values: list[tuple[str, object]] = []
    files = getattr(data, "files", ())
    if "processing" in files:
        values.append(("NPZ processing", data["processing"]))
    if "config" in files:
        values.append(("NPZ config", data["config"]))
    if header.get("processing") is not None:
        values.append(("telemetry header processing", header["processing"]))
    if header.get("config") is not None:
        values.append(("telemetry header config", header["config"]))

    for source_name, value in values:
        parsed = _dewow_settings_from_object(value, source_name)
        if parsed is not None:
            return parsed

    return bscan_processing_signature(True, 4.0), "legacy"


def safe_name(value: object, fallback: str) -> str:
    text = str(value or fallback)
    return "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in text)


class AspectRatioWidget(QtWidgets.QWidget):
    """A height-for-width container for a visual width:height ratio."""

    def __init__(self, child: QtWidgets.QWidget, width_to_height: float = 2.0) -> None:
        super().__init__()
        self._aspect_ratio = 2.0
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(child)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Preferred,
        )
        self.set_aspect_ratio(width_to_height)

    def hasHeightForWidth(self) -> bool:
        return True

    def heightForWidth(self, width: int) -> int:
        return max(1, int(round(float(width) / self._aspect_ratio)))

    def set_aspect_ratio(self, width_to_height: float) -> None:
        self._aspect_ratio = min(max(float(width_to_height), 0.5), 6.0)
        self.updateGeometry()


class ReceiverThread(QtCore.QThread):
    status = QtCore.Signal(str)
    trace_received = QtCore.Signal(dict, bytes, str)
    command_status = QtCore.Signal(str)
    #: 第1轮检查点D：空中端 acquisition_status 状态消息（state/queues等）。
    acquisition_status = QtCore.Signal(dict)

    def __init__(self, host: str, port: int, out_dir: Path, metrics=None) -> None:
        super().__init__()
        self.host = host
        self.port = port
        self.out_dir = out_dir
        #: 可选性能统计（接收/落盘阶段耗时；未注入则不记录，绝不影响接收）。
        self.metrics = metrics
        self._stop = threading.Event()
        # 第1轮检查点D：幂等去重。key=(run_id, trace_index) → sha256。
        # 完全相同重传：重复 ACK 但不再保存/显示；同编号不同 sha：拒绝并报错。
        self._seen_traces: dict[tuple[str, int], str] = {}
        # 第二次定向修复：receiver→主线程待处理信号计数（emit 前加 1，
        # 主线程槽执行时减 1）。closeEvent 用它判断"receiver 已退出但仍有
        # trace_received 信号在 Qt 事件队列中"。
        self._emitted_pending = 0
        self._emitted_lock = threading.Lock()
        #: 第三次定向修复：note_signal_consumed 双归还错误累计。
        self._emitted_underflow_errors = 0
        #: 监听 socket 与当前 client 的引用，由 _sock_lock 保护（stop/run 并发安全）。
        self._server: socket.socket | None = None
        self._client: socket.socket | None = None
        self._sock_lock = threading.Lock()
        self._send_lock = threading.Lock()
        #: 实际绑定端口（port=0 时由 OS 分配；bind 后在 run() 内更新）。
        self._bound_port: int | None = None

    def begin_run(self, run_id: str) -> None:
        """第三次定向修复：切换 run_id 时调用，重置当前内存去重表。

        仅清空当前会话的 ``_seen_traces``；磁盘上已存在的 trace_*.npz 仍
        走跨重启重读/冲突拒绝路径（不删除磁盘文件）。
        """
        _ = run_id
        with self._emitted_lock:
            self._seen_traces.clear()

    def _note_emitted(self) -> None:
        with self._emitted_lock:
            self._emitted_pending += 1

    def note_signal_consumed(self) -> None:
        """主线程处理完一条 trace_received 信号后归还计数。

        第三次定向修复：禁止 ``max(0, n-1)`` 掩盖 underflow；underflow 必须
        被检测并记录（_emitted_underflow_errors 累计），调用方应据此判定
        decoder 屏障所有权异常。
        """
        with self._emitted_lock:
            if self._emitted_pending <= 0:
                # underflow：双重归还或未配对 take 记录。
                self._emitted_underflow_errors += 1
                return
            self._emitted_pending -= 1

    def emitted_pending(self) -> int:
        """已 emit 但主线程尚未处理的 trace_received 信号数。"""
        with self._emitted_lock:
            return self._emitted_pending

    # ---- 不可达死代码已删除（第三次定向修复）：emitted_pending 后续旧版本残留的
    # _server/_client/_sock_lock/_send_lock 初始化块现归入正常 __init__。
        #: 实际绑定端口（port=0 时由 OS 分配；bind 后在 run() 内更新）。
        self._bound_port: int | None = None

    @property
    def bound_port(self) -> int | None:
        """当前监听 socket 实际绑定的端口（未启动/已停止为 None）。"""
        return self._bound_port

    def stop(self) -> None:
        """幂等停止：置停止标记，并 shutdown/close server 与 client 以解除 accept/recv 阻塞。

        竞态处理：stop 与 run 对 ``_server``/``_client`` 的读写都经 ``_sock_lock``，
        关闭在锁内取引用、锁外执行，确保 stop 后任何已赋值/将赋值的 socket 都会被关闭。
        """
        self._stop.set()
        with self._sock_lock:
            server = self._server
            client = self._client
        for sock in (server, client):
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    sock.close()
                except OSError:
                    pass

    def send_command(self, action: str, params: dict[str, object] | None = None) -> None:
        with self._sock_lock:
            client = self._client
        if client is None:
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
                send_frame(client, header)
            self.command_status.emit(f"Command sent: {action}")
        except Exception as exc:
            self.command_status.emit(f"Command failed: {exc}")

    def run(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.host, self.port))
            server.listen(1)
            server.settimeout(0.5)
            with self._sock_lock:
                self._server = server
                self._bound_port = int(server.getsockname()[1])
            self.status.emit(f"Listening on {self.host}:{self._bound_port}")
            while not self._stop.is_set():
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                except OSError as exc:
                    # stop 关闭 server 引发的预期 OSError：非故障；真实错误则上报。
                    if self._stop.is_set():
                        return
                    self.status.emit(f"Accept error: {exc}")
                    break
                # 在锁内检查 stop 并赋值 _client，避免 accept 返回与 stop 之间的窗口：
                # stop 已发生则不启动该 client（关闭并退出）。
                with self._sock_lock:
                    if self._stop.is_set():
                        stop_pending = True
                    else:
                        self._client = client
                        stop_pending = False
                if stop_pending:
                    try:
                        client.close()
                    except OSError:
                        pass
                    break
                client.settimeout(30.0)
                self.status.emit(f"Connected: {address[0]}:{address[1]}")
                try:
                    self.handle_client(client)
                except OSError as exc:
                    # stop 关闭 client 引发的 recv OSError 属预期；非 stop 的真实 socket
                    # 错误仍须上报（连接断开也算一种 client 结束）。
                    if not self._stop.is_set():
                        self.status.emit(f"Client error: {exc}")
                except Exception as exc:
                    self.status.emit(f"Disconnected: {exc}")
                finally:
                    try:
                        client.close()
                    except OSError:
                        pass
                    with self._sock_lock:
                        if self._client is client:
                            self._client = None
        finally:
            with self._sock_lock:
                self._server = None
                self._client = None
                self._bound_port = None
            try:
                server.close()
            except OSError:
                pass

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
                _t0 = time.perf_counter()
                outcome = self._accept_trace(sock, header, payload)
                if outcome is None:
                    continue
                path = outcome
                if self.metrics is not None:
                    try:
                        self.metrics.record_stage("ground_received", time.perf_counter() - _t0)
                    except Exception:  # noqa: BLE001
                        pass
                self._note_emitted()  # emit 前占用待处理计数（Qt queued 屏障）。
                self.trace_received.emit(header, payload, str(path))
            elif msg_type == "command_ack":
                # command_ack 只表示停止/启动命令已被空中端接受，不等于采集
                # 已停止（第1轮检查点D）：最终状态由 acquisition_status 决定。
                self.command_status.emit(f"Remote accepted: {header.get('action')}")
            elif msg_type == "acquisition_status":
                # 第1轮检查点D：向后兼容状态消息（旧接收端会落入 else 分支忽略）。
                self.acquisition_status.emit(dict(header))
            else:
                self.status.emit(f"Ignored message: {msg_type}")

    def _accept_trace(self, sock: socket.socket, header: dict[str, object], payload: bytes) -> Path | None:
        """幂等去重后落盘；返回保存路径（重复时返回 None），冲突/无效返回 None。

        复审修复（跨重启去重 + 严格校验）：

        - trace_index 必须是严格正整数（bool/字符串垃圾/0/负数 → 拒绝，不 ACK，
          绝不默认为 trace_00000）；
        - 始终用 payload 计算 actual_sha 并校验 header sha256（不信任 header）；
        - 首次 (run_id, trace_index)：sha 校验、保存、ACK、进入解码/显示，
          ``_seen_traces`` 记录**验证过的 actual_sha**；
        - 重复（内存命中，actual_sha 相同）：duplicate ACK，不重复保存/显示；
        - 跨重启（内存未命中但磁盘文件已存在）：读取磁盘文件 SHA——
          磁盘 SHA == actual_sha → duplicate ACK；不同 → 冲突拒绝（不 ACK、
          不覆盖）；读取失败 → fail-closed 拒绝；
        - 同编号不同 sha256：严重冲突，不 ACK、记录错误。
        """
        run_key = safe_name(header.get("run_id"), "run")
        raw_index = header.get("trace_index")
        # 严格正整数校验：bool 是 int 子类，必须先排除。
        if isinstance(raw_index, bool) or not isinstance(raw_index, (int, np.integer)):
            self.status.emit(f"Trace 拒绝: trace_index 非法 ({raw_index!r})，不落盘不 ACK")
            return None
        trace_index = int(raw_index)
        if trace_index <= 0:
            self.status.emit(f"Trace 拒绝: trace_index 非正整数 ({trace_index})，不落盘不 ACK")
            return None
        # 始终以 payload 实际 sha256 为准；header sha256 必须存在且为 64 位
        # 十六进制字符串（缺失/格式非法 → fail-closed，不 ACK、不落盘）。
        actual_sha = hashlib.sha256(payload).hexdigest()
        expected_sha = str(header.get("sha256") or "")
        if not expected_sha or len(expected_sha) != 64 or any(
            c not in "0123456789abcdefABCDEF" for c in expected_sha
        ):
            self.status.emit(
                f"Trace 拒绝: run={run_key} trace={trace_index} sha256 缺失或格式非法"
                f"（hex64 校验失败，不落盘不 ACK）"
            )
            return None
        if expected_sha != actual_sha:
            self.status.emit(
                f"Trace 拒绝: run={run_key} trace={trace_index} sha256 校验失败"
                f"（header={expected_sha[:12]}… payload={actual_sha[:12]}…）"
            )
            return None
        key = (run_key, trace_index)
        seen_sha = self._seen_traces.get(key)
        npz_path = self.out_dir / run_key / f"trace_{trace_index:05d}.npz"
        if seen_sha is None and npz_path.exists():
            # 跨重启：内存集合未命中但磁盘已有文件——按磁盘内容对账。
            try:
                disk_sha = hashlib.sha256(npz_path.read_bytes()).hexdigest()
            except OSError as exc:
                self.status.emit(
                    f"Trace 拒绝: run={run_key} trace={trace_index} 读取已存文件失败"
                    f"（fail-closed，不 ACK）: {exc}"
                )
                return None
            if disk_sha != actual_sha:
                self.status.emit(
                    f"Trace 冲突拒绝: run={run_key} trace={trace_index} "
                    f"sha256 与磁盘已存版本不一致（拒绝并保留已存文件）"
                )
                return None
            # 磁盘一致：作为重复包处理（不重复写 metadata/显示），并回填内存集合。
            seen_sha = disk_sha
            self._seen_traces[key] = disk_sha
        if seen_sha is not None:
            if seen_sha == actual_sha:
                send_frame(
                    sock,
                    {
                        "type": "ack_trace",
                        "protocol": PROTOCOL,
                        "run_id": header.get("run_id"),
                        "trace_index": trace_index,
                        "path": npz_path.name,
                        "duplicate": True,
                    },
                )
                return None  # 幂等：不重新进入解码/显示
            self.status.emit(
                f"Trace 冲突拒绝: run={run_key} trace={trace_index} "
                f"sha256 与已保存版本不一致（拒绝并保留已存文件）"
            )
            return None
        _save_t0 = time.perf_counter()
        path = self.save_trace(header, payload, actual_sha=actual_sha)
        if self.metrics is not None:
            try:
                self.metrics.record_stage("ground_saved", time.perf_counter() - _save_t0)
            except Exception:  # noqa: BLE001
                pass
        # 只记录经过验证的 actual_sha（不直接信任 header）。
        self._seen_traces[key] = actual_sha
        send_frame(
            sock,
            {
                "type": "ack_trace",
                "protocol": PROTOCOL,
                "run_id": header.get("run_id"),
                "trace_index": trace_index,
                "path": path.name,
            },
        )
        return path

    def save_trace(self, header: dict[str, object], payload: bytes, *, actual_sha: str | None = None) -> Path:
        """落盘（sha 校验在 _accept_trace 已完成；本函数只写文件与 metadata）。"""
        if actual_sha is None:
            expected_sha = str(header.get("sha256") or "")
            actual_sha = hashlib.sha256(payload).hexdigest()
            if (
                not expected_sha
                or len(expected_sha) != 64
                or any(c not in "0123456789abcdefABCDEF" for c in expected_sha)
                or expected_sha != actual_sha
            ):
                raise ValueError(
                    f"sha256 缺失/格式非法/不匹配 for trace {header.get('trace_index')}"
                )

        run_id = safe_name(header.get("run_id"), "run")
        raw_index = header.get("trace_index")
        if isinstance(raw_index, bool) or not isinstance(raw_index, (int, np.integer)):
            raise ValueError(f"trace_index 非法: {raw_index!r}")
        trace_index = int(raw_index)
        if trace_index <= 0:
            raise ValueError(f"trace_index 必须为正整数: {trace_index}")
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


@dataclass(frozen=True)
class AcquisitionUiSettings:
    """采集参数已提交状态（对话框草稿未应用前，command_params 只读这里）。

    frozen：已应用实例不可被外部原地修改；对话框每次“应用”都生成新快照。
    """

    interval_s: float = 0.0
    count: int = 0

    def command_params(self, single: bool = False) -> dict[str, object]:
        return {
            "count": 1 if single else int(self.count),
            "interval_s": float(self.interval_s),
            "single": single,
        }


@dataclass(frozen=True)
class BscanUiSettings:
    """B-scan 显示参数已提交状态。

    frozen：已应用实例不可被外部原地修改；对话框每次“应用”都生成新快照。
    """

    colormap: str = "gray"
    aspect_ratio: float = 2.0
    flat_filter_enabled: bool = False
    flat_window_traces: int = 31


class AcquisitionSettingsDialog(QtWidgets.QDialog):
    """采集设置对话框：编辑草稿，点“应用”才提交给主窗口。"""

    def __init__(self, window: "GroundStationWindow") -> None:
        super().__init__(window)
        self._window = window
        self.setObjectName("acquisition_settings_dialog")
        self.setWindowTitle("采集设置")
        form = QtWidgets.QFormLayout(self)
        # 第1轮检查点C：显式采集模式，避免"0 秒间隔"这类易误解的纯数字。
        # interval_s=0.0 是底层兼容表示（"每个完整Sweep"）。
        self.mode = QtWidgets.QComboBox()
        self.mode.setObjectName("acquisition_mode_input")
        self.mode.addItem("每个完整Sweep（默认）", userData=0.0)
        self.mode.addItem("固定时间间隔", userData=1.0)
        self.mode.currentIndexChanged.connect(self._sync_interval_enabled)
        self.interval = QtWidgets.QDoubleSpinBox()
        self.interval.setObjectName("acquisition_interval_input")
        self.interval.setRange(0.0, 3600.0)
        self.interval.setDecimals(2)
        self.interval.setValue(1.0)
        self.count = QtWidgets.QSpinBox()
        self.count.setObjectName("acquisition_count_input")
        self.count.setRange(0, 1000000)
        self.count.setValue(0)
        self.count_hint = QtWidgets.QLabel("0 表示持续采集")
        self.count_hint.setObjectName("acquisition_count_hint")
        form.addRow("采集模式", self.mode)
        form.addRow("采集间隔 (s)", self.interval)
        form.addRow("采集数量", self.count)
        form.addRow("", self.count_hint)
        buttons = QtWidgets.QDialogButtonBox()
        self.apply_button = buttons.addButton(
            "应用", QtWidgets.QDialogButtonBox.ButtonRole.AcceptRole
        )
        self.cancel_button = buttons.addButton(
            "取消", QtWidgets.QDialogButtonBox.ButtonRole.RejectRole
        )
        self.apply_button.setObjectName("acquisition_apply_button")
        self.cancel_button.setObjectName("acquisition_cancel_button")
        buttons.accepted.connect(self.apply)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def _sync_interval_enabled(self) -> None:
        # 选择"固定时间间隔"后才启用秒数输入框。
        self.interval.setEnabled(self.mode.currentIndex() == 1)

    def load_from(self, settings: AcquisitionUiSettings) -> None:
        if float(settings.interval_s) <= 0.0:
            self.mode.setCurrentIndex(0)
        else:
            self.mode.setCurrentIndex(1)
        self.interval.setValue(
            float(settings.interval_s) if settings.interval_s > 0.0 else 1.0
        )
        self.count.setValue(int(settings.count))
        self._sync_interval_enabled()

    def draft_settings(self) -> AcquisitionUiSettings:
        if self.mode.currentIndex() == 0:
            interval = 0.0
        else:
            interval = float(self.interval.value())
            if interval <= 0.0:
                interval = 0.0  # 固定模式下输入 0 等价于"每个完整Sweep"。
        return AcquisitionUiSettings(
            interval_s=interval,
            count=int(self.count.value()),
        )

    def apply(self) -> None:
        self._window.apply_acquisition_settings(self.draft_settings())
        self.accept()


class BscanSettingsDialog(QtWidgets.QDialog):
    """B-scan 设置对话框：编辑草稿，点“应用”才提交并刷新现有 B-scan。"""

    def __init__(self, window: "GroundStationWindow") -> None:
        super().__init__(window)
        self._window = window
        self.setObjectName("bscan_settings_dialog")
        self.setWindowTitle("B-scan 设置")
        form = QtWidgets.QFormLayout(self)
        self.cmap = QtWidgets.QComboBox()
        self.cmap.setObjectName("bscan_colormap_input")
        self.cmap.addItems(["gray", "jet"])
        self.aspect = QtWidgets.QDoubleSpinBox()
        self.aspect.setObjectName("bscan_aspect_input")
        self.aspect.setRange(0.5, 6.0)
        self.aspect.setDecimals(2)
        self.aspect.setSingleStep(0.1)
        self.aspect.setValue(2.0)
        self.flat_enabled = QtWidgets.QCheckBox("启用 Flat Filter")
        self.flat_enabled.setObjectName("bscan_flat_enabled_input")
        self.flat_window = QtWidgets.QSpinBox()
        self.flat_window.setObjectName("bscan_flat_window_input")
        self.flat_window.setRange(3, 10001)
        self.flat_window.setSingleStep(2)
        self.flat_window.setValue(31)
        self.flat_enabled.toggled.connect(self._sync_window_enabled)
        self._sync_window_enabled(self.flat_enabled.isChecked())
        form.addRow("颜色表", self.cmap)
        form.addRow("B-scan 长宽比", self.aspect)
        form.addRow("", self.flat_enabled)
        form.addRow("Flat Filter 窗口道数", self.flat_window)
        buttons = QtWidgets.QDialogButtonBox()
        self.apply_button = buttons.addButton(
            "应用", QtWidgets.QDialogButtonBox.ButtonRole.AcceptRole
        )
        self.cancel_button = buttons.addButton(
            "取消", QtWidgets.QDialogButtonBox.ButtonRole.RejectRole
        )
        self.apply_button.setObjectName("bscan_apply_button")
        self.cancel_button.setObjectName("bscan_cancel_button")
        buttons.accepted.connect(self.apply)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def _sync_window_enabled(self, enabled: bool) -> None:
        self.flat_window.setEnabled(bool(enabled))

    def load_from(self, settings: BscanUiSettings) -> None:
        index = self.cmap.findText(settings.colormap)
        self.cmap.setCurrentIndex(index if index >= 0 else 0)
        self.aspect.setValue(float(settings.aspect_ratio))
        self.flat_enabled.setChecked(bool(settings.flat_filter_enabled))
        self.flat_window.setValue(int(settings.flat_window_traces))
        self._sync_window_enabled(self.flat_enabled.isChecked())

    def draft_settings(self) -> BscanUiSettings:
        window = int(self.flat_window.value())
        if window % 2 == 0:
            window -= 1  # Flat Filter 窗口保持奇数约束。
        return BscanUiSettings(
            colormap=self.cmap.currentText(),
            aspect_ratio=float(self.aspect.value()),
            flat_filter_enabled=bool(self.flat_enabled.isChecked()),
            flat_window_traces=window,
        )

    def apply(self) -> None:
        self._window.apply_bscan_settings(self.draft_settings())
        self.accept()


# 地面站主窗口尺寸约束：最小尺寸保证菜单/按钮/两幅 B-scan/GNSS/状态栏均可
# 显示；目标尺寸为常见桌面分辨率；初始窗口尺寸上限防止初始窗口大于屏幕
# 可视区域（注意：代码未调用 setMaximumSize()，用户仍可手动把窗口拉大）。
_GROUND_STATION_MIN_SIZE = (960, 600)
_GROUND_STATION_TARGET_SIZE = (1380, 860)
_GROUND_STATION_MAX_SIZE = (1920, 1080)
# 在没有可用屏幕（headless 测试或 QApplication 尚未初始化）时的保守回退。
_GROUND_STATION_FALLBACK_SIZE = (1280, 800)


def _compute_initial_window_size() -> tuple[int, int]:
    """根据当前主屏幕可用区域计算初始窗口尺寸（不超出可视区域，留 5% 余量）。

    宽度和高度**完全独立**计算，每个维度取 ``min(target, cap)``，其中
    ``cap = min(max_size, int(avail * 0.95))``。这保证：

    - 任一维度不超过该维度 95% 的可用区域（如 800×600 屏幕上宽度为 760，
      不会溢出屏幕）；
    - 宽高互不耦合：一个维度受限不会导致另一个维度跳过 target 限制
      （如 1920×600 屏幕上宽度仍为 target 1380，高度为 570；800×1080
      屏幕上宽度为 760，高度仍为 target 860）；
    - 1366×768 / 1920×1080 等常见分辨率下达到或接近目标尺寸 1380×860。

    当某一维度的可用区域小于设计最小尺寸 960×600 时，该维度初始尺寸低于
    最小尺寸；调用方（``GroundStationWindow.__init__``）会把
    ``setMinimumSize`` 收紧到 ``min(design_min, initial)``，避免 Qt 把
    窗口重新撑大导致溢出屏幕。
    """
    app = QtWidgets.QApplication.instance()
    target_w, target_h = _GROUND_STATION_TARGET_SIZE
    max_w, max_h = _GROUND_STATION_MAX_SIZE
    if app is None:
        return _GROUND_STATION_FALLBACK_SIZE
    screen = app.primaryScreen()
    if screen is None:
        return _GROUND_STATION_FALLBACK_SIZE
    try:
        available = screen.availableGeometry()
    except Exception:
        return _GROUND_STATION_FALLBACK_SIZE
    avail_w = int(available.width())
    avail_h = int(available.height())
    if avail_w <= 0 or avail_h <= 0:
        return _GROUND_STATION_FALLBACK_SIZE
    # 每个维度独立：95% 可用区域作为该维度的上限，保留任务栏/标题栏余量；
    # 极小屏幕上 max(1, ...) 保证尺寸始终为正。
    cap_w = max(1, min(max_w, int(avail_w * 0.95)))
    cap_h = max(1, min(max_h, int(avail_h * 0.95)))
    initial_w = min(target_w, cap_w)
    initial_h = min(target_h, cap_h)
    return initial_w, initial_h


class _DecoderBridge(QtCore.QObject):
    """把后台解码线程结果经 Qt 队列信号送回主线程。"""

    decoded = QtCore.Signal(object)
    rejected = QtCore.Signal(object, str, object)

    def emit_decoded(self, decoded: DecodedGroundTrace) -> None:
        self.decoded.emit(decoded)

    def emit_rejected(self, header: dict[str, object], path: str, exc: BaseException) -> None:
        self.rejected.emit(header, path, exc)


class GroundStationWindow(QtWidgets.QMainWindow):
    def __init__(
        self,
        host: str,
        port: int,
        out_dir: Path,
        *,
        receiver: "ReceiverThread | None" = None,
        start_receiver: bool = True,
    ) -> None:
        super().__init__()
        self.setWindowTitle("UAV-GPR 地面站")
        # 初始窗口尺寸：优先采用屏幕可用区域的 95% 上限并保留最小可用尺寸，
        # 避免在 1366×768 / 1920×1080 等常见分辨率下超出可视区域。
        # 屏幕不可用（headless/未初始化）时退化到保守的 1280×800。
        initial_w, initial_h = _compute_initial_window_size()
        # 最小尺寸约束：设计契约为 960×600，保证菜单/按钮/两幅 B-scan/GNSS/
        # 状态栏在极端缩放下仍可显示。但当屏幕可用区域小于该契约时（例如
        # 800×600 的小屏幕或 800×800 的 offscreen 测试环境），若仍硬性
        # ``setMinimumSize(960, 600)``，Qt 会把窗口撑大到 960×600 并导致
        # 窗口宽度/高度超过屏幕可视范围。此处把实际最小尺寸收紧到
        # ``min(design_min, initial)``，既不破坏大屏幕下的设计契约，又避免
        # 小屏幕下窗口溢出。
        design_min_w, design_min_h = _GROUND_STATION_MIN_SIZE
        actual_min_w = min(design_min_w, initial_w)
        actual_min_h = min(design_min_h, initial_h)
        self.setMinimumSize(actual_min_w, actual_min_h)
        self.resize(initial_w, initial_h)
        # 第三次定向修复：任务级计数基线（run_id 隔离）。新任务开始时记录
        # decoder/FIFO 当前累计值的不可变快照；停止对账只比较本轮增量。
        # 进程累计值仍可继续累加（其他线程正在使用），绝不粗暴 reset。
        self._task_baseline: dict[str, int] = {}
        self._task_run_id: str | None = None
        self._last_ground_run_id: str | None = None
        # 已提交设置：主界面与 command_params 只读这里，对话框编辑独立草稿。
        self.acq_settings = AcquisitionUiSettings()
        self.bscan_settings = BscanUiSettings()
        self.bscan_rows: dict[str, list[np.ndarray]] = {name: [] for name in UI_CHANNELS}
        self.bscan_time_ns: np.ndarray | None = None
        self.bscan_processing_signature: tuple[bool, float | None] | None = None
        self.bscan_filter_status_override: str | None = None
        self.bscan_window_start = 1
        self.bscan_follow_latest = True
        # ---- 第1轮检查点B：显示 FIFO + 后台解码线程 + 性能统计 ----
        # 显示队列是显式 FIFO：按到达顺序逐条提交，每条 Trace 恰好提交一次；
        # 一次定时回调最多提交一条，然后把控制权还给 Qt 事件循环（25Hz 下每条
        # Trace 都进入绘图调用）。积压只告警，绝不丢弃。
        self.display_fifo = GroundDisplayFIFO()
        self.metrics = PipelineMetrics()
        self._display_timer_active = False
        # 解码线程内的解码耗时也计入同一统计（模块级注入）。
        set_ground_decoder_metrics(self.metrics)
        # 解码线程回调经 Qt 信号桥接回主线程（解码在后台线程执行）。
        self._decoder_bridge = _DecoderBridge()
        self._decoder_bridge.decoded.connect(self._on_trace_decoded)
        self._decoder_bridge.rejected.connect(self._on_trace_rejected)
        self.decoder = GroundTraceDecoder(
            self._decoder_bridge.emit_decoded,
            self._decoder_bridge.emit_rejected,
        )
        self.decoder.start()
        # 高频日志节流：连续 Trace 到达时每 N 条记一条日志（关键事件仍即时）。
        self._trace_log_counter = 0
        self._trace_log_accumulated = 0
        # 停止对账（复审）：空中端最终 stopped 状态（drain_state=drained）。
        self._final_stop_state: dict | None = None
        # 停止提交屏障（第二次定向修复）：最终状态到达时的 FIFO 入队计数；
        # 屏障之前的全部 Trace 完成提交（committed+rejected+failed）才可停止。
        self._stop_commit_barrier: int | None = None
        # 受控依赖注入：纯显示测试可传 start_receiver=False 避免启动真实监听线程。
        self.receiver = receiver if receiver is not None else ReceiverThread(host, port, out_dir)
        # 接收/落盘阶段耗时计入同一统计（测试注入的假 receiver 无此属性则跳过）。
        if hasattr(self.receiver, "__dict__"):
            try:
                self.receiver.metrics = self.metrics
            except Exception:  # noqa: BLE001 - 注入失败不影响功能
                pass
        self._build_ui(host, port, out_dir)
        self.receiver.status.connect(self._on_receiver_status)
        self.receiver.command_status.connect(self._on_command_status)
        if hasattr(self.receiver, "acquisition_status"):
            self.receiver.acquisition_status.connect(self._on_acquisition_status)
        # 生产路径：接收线程收到的 Trace 交给后台解码线程，不再在主线程解压。
        # （on_trace 保留为同步兼容/测试入口。）
        self.receiver.trace_received.connect(self._on_receiver_trace_received)
        if start_receiver:
            self.receiver.start()

    def _build_ui(self, host: str, port: int, out_dir: Path) -> None:
        central = QtWidgets.QWidget()
        central.setObjectName("main_central")
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        self._build_menu_bar()
        outer.addWidget(self._build_action_bar())
        outer.addWidget(self._build_main_content(), 1)
        self._build_status_bar()
        self._build_auxiliary_windows(host, port, out_dir)

    def _build_menu_bar(self) -> None:
        menubar = self.menuBar()
        # 保持菜单强引用（QMenu 由 addMenu 返回，需存实例属性供测试定位）。
        self.menu_task = menubar.addMenu("任务")
        self.action_single_acquisition = self.menu_task.addAction("单次采集")
        self.action_single_acquisition.setObjectName("action_single_acquisition")
        self.action_acquisition_settings = self.menu_task.addAction("采集设置……")
        self.action_acquisition_settings.setObjectName("action_acquisition_settings")
        self.menu_task.addSeparator()
        self.action_exit = self.menu_task.addAction("退出")
        self.action_exit.setObjectName("action_exit")

        self.menu_bscan = menubar.addMenu("B-scan")
        self.action_bscan_settings = self.menu_bscan.addAction("B-scan 设置……")
        self.action_bscan_settings.setObjectName("action_bscan_settings")
        self.action_clear_bscan = self.menu_bscan.addAction("清空 B-scan")
        self.action_clear_bscan.setObjectName("action_clear_bscan")

        self.menu_view = menubar.addMenu("视图")
        self.action_show_details = self.menu_view.addAction("详细状态")
        self.action_show_details.setObjectName("action_show_details")
        self.action_show_log = self.menu_view.addAction("运行日志")
        self.action_show_log.setObjectName("action_show_log")
        # 第1轮检查点A：链路性能诊断入口（详细指标只在需要时查看）。
        self.action_show_pipeline_metrics = self.menu_view.addAction("链路性能诊断")
        self.action_show_pipeline_metrics.setObjectName("action_show_pipeline_metrics")

        self.menu_help = menubar.addMenu("帮助")
        self.action_about = self.menu_help.addAction("关于")
        self.action_about.setObjectName("action_about")

        self.action_single_acquisition.triggered.connect(self.send_single)
        self.action_acquisition_settings.triggered.connect(self.open_acquisition_settings)
        self.action_exit.triggered.connect(self.close)
        self.action_bscan_settings.triggered.connect(self.open_bscan_settings)
        self.action_clear_bscan.triggered.connect(self.clear_bscan)
        self.action_show_details.triggered.connect(self.show_details_window)
        self.action_show_log.triggered.connect(self.show_log_window)
        self.action_show_pipeline_metrics.triggered.connect(self.show_pipeline_metrics)
        # 复审修复：关于 action 在 _build_menu_bar 中只连接一次（此前错误地
        # 放在 show_pipeline_metrics 内，每打开一次诊断就多一条重复连接）。
        self.action_about.triggered.connect(self.show_about)

    def show_pipeline_metrics(self) -> None:
        """把链路性能统计快照写入运行日志（诊断入口，不影响主界面）。"""
        import json

        try:
            snapshot = {
                "stages": {
                    name: report
                    for name, report in self.metrics.snapshot()["stages"].items()
                },
                "display_fifo": self.display_fifo.statistics(),
                "decoder": self.decoder.statistics(),
            }
            self.log(f"链路性能诊断: {json.dumps(snapshot, ensure_ascii=False)}")
        except Exception as exc:  # noqa: BLE001 - 诊断不得抛出
            self.log(f"链路性能诊断失败: {exc}")
        self.show_log_window()

    def _build_action_bar(self) -> QtWidgets.QWidget:
        bar = QtWidgets.QWidget()
        bar.setObjectName("action_bar")
        row = QtWidgets.QHBoxLayout(bar)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)
        self.start_button = QtWidgets.QPushButton("开始")
        self.start_button.setObjectName("start_button")
        # 限制按钮宽度，避免在窄窗口下被水平拉伸；最小高度保证可点击区域。
        self.start_button.setMaximumWidth(120)
        self.start_button.setMinimumHeight(32)
        self.stop_button = QtWidgets.QPushButton("停止")
        self.stop_button.setObjectName("stop_button")
        self.stop_button.setMaximumWidth(120)
        self.stop_button.setMinimumHeight(32)
        self.start_button.clicked.connect(self.send_start)
        self.stop_button.clicked.connect(self.send_stop)
        row.addWidget(self.start_button)
        row.addWidget(self.stop_button)
        row.addStretch(1)
        return bar

    def _build_main_content(self) -> QtWidgets.QWidget:
        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        splitter.setObjectName("main_splitter")
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(6)

        left = QtWidgets.QWidget()
        left.setObjectName("bscan_area")
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(8)

        # B-scan 是地面站主要产品：上下排列，HH 在上、VV 在下。
        self.bscan_plots: dict[str, pg.PlotWidget] = {}
        self.bscan_images: dict[str, pg.ImageItem] = {}
        self.bscan_containers: dict[str, AspectRatioWidget] = {}
        for index, name in enumerate(UI_CHANNELS):
            plot = pg.PlotWidget(title=f"{CHANNEL_LABELS[name]} B-scan")
            plot.setObjectName(f"bscan_plot_{name}")
            plot.setLabel("bottom", "Trace")
            plot.setLabel("left", "时间", units="ns")
            plot.showGrid(x=True, y=True, alpha=0.15)
            plot.setMouseEnabled(x=False, y=False)
            plot.setMenuEnabled(False)
            plot.getPlotItem().hideButtons()
            plot.getPlotItem().getViewBox().invertY(True)
            plot.setXRange(0.5, BSCAN_WINDOW_TRACES + 0.5, padding=0)
            plot.setMinimumHeight(0)
            # row-major：图像数组按 [row=y=时间, col=x=Trace] 解释，直接传入
            # [n_time, n_trace] 显示矩阵，避免每条 Trace 转置/复制。
            image = pg.ImageItem(axisOrder="row-major")
            plot.addItem(image)
            container = AspectRatioWidget(plot, self.bscan_settings.aspect_ratio)
            container.setObjectName(f"bscan_container_{name}")
            self.bscan_plots[name] = plot
            self.bscan_images[name] = image
            self.bscan_containers[name] = container
            left_layout.addWidget(container, 1)

        slider_row = QtWidgets.QWidget()
        slider_row.setObjectName("bscan_history_row")
        slider_layout = QtWidgets.QHBoxLayout(slider_row)
        slider_layout.setContentsMargins(0, 0, 0, 0)
        self.bscan_window_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.bscan_window_slider.setObjectName("bscan_window_slider")
        self.bscan_window_slider.setRange(1, 1)
        self.bscan_window_slider.setValue(1)
        self.bscan_window_slider.valueChanged.connect(self.on_bscan_window_changed)
        self.bscan_window_label = QtWidgets.QLabel("显示 1–0 / 共 0 道")
        self.bscan_window_label.setObjectName("bscan_window_label")
        # 防止“显示 X–Y / 共 N 道”在窄窗口下被截断。
        self.bscan_window_label.setMinimumWidth(140)
        slider_layout.addWidget(self.bscan_window_slider, 1)
        slider_layout.addWidget(self.bscan_window_label)
        left_layout.addWidget(slider_row)

        self.gnss_map = GnssMapWidget()
        self.gnss_map.setObjectName("gnss_map")

        splitter.addWidget(left)
        splitter.addWidget(self.gnss_map)
        splitter.setStretchFactor(0, 7)
        splitter.setStretchFactor(1, 3)
        splitter.setSizes([966, 414])  # 约 70:30
        return splitter

    def _build_status_bar(self) -> None:
        status = self.statusBar()
        status.setObjectName("main_status_bar")
        self.connection_status_label = QtWidgets.QLabel("连接状态：等待连接")
        self.connection_status_label.setObjectName("connection_status_label")
        self.trace_status_label = QtWidgets.QLabel("当前 Trace：-")
        self.trace_status_label.setObjectName("trace_status_label")
        self.acquisition_status_label = QtWidgets.QLabel("采集状态：未采集")
        self.acquisition_status_label.setObjectName("acquisition_status_label")
        status.addWidget(self.connection_status_label)
        status.addPermanentWidget(self.trace_status_label)
        status.addPermanentWidget(self.acquisition_status_label)

    def _build_auxiliary_windows(self, host: str, port: int, out_dir: Path) -> None:
        # ---- 运行日志窗口（默认隐藏，关闭只隐藏不销毁）----
        self.log_window = QtWidgets.QWidget(self, QtCore.Qt.WindowType.Window)
        self.log_window.setObjectName("log_window")
        self.log_window.setWindowTitle("运行日志")
        log_layout = QtWidgets.QVBoxLayout(self.log_window)
        log_layout.setContentsMargins(6, 6, 6, 6)
        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setObjectName("log_box")
        self.log_box.setReadOnly(True)
        # 长期运行防无限增长：日志框保留最近 2000 块（第1轮检查点B）。
        self.log_box.setMaximumBlockCount(2000)
        log_layout.addWidget(self.log_box)
        self.log_window.resize(700, 300)

        # ---- 详细状态窗口（默认隐藏，迁移原主界面长文本状态）----
        self.details_window = QtWidgets.QWidget(self, QtCore.Qt.WindowType.Window)
        self.details_window.setObjectName("details_window")
        self.details_window.setWindowTitle("详细状态")
        details_layout = QtWidgets.QFormLayout(self.details_window)
        details_layout.setContentsMargins(10, 10, 10, 10)
        self.listen_label = QtWidgets.QLabel(f"{host}:{port}")
        self.listen_label.setObjectName("listen_label")
        self.out_dir_label = QtWidgets.QLabel(str(out_dir))
        self.out_dir_label.setObjectName("out_dir_label")
        self.status_label = QtWidgets.QLabel("等待事件")
        self.status_label.setObjectName("status_label")
        self.status_label.setWordWrap(True)
        self.acquisition_label = QtWidgets.QLabel("采集：-")
        self.acquisition_label.setObjectName("acquisition_label")
        self.acquisition_label.setWordWrap(True)
        self.legacy_status = QtWidgets.QLabel("")
        self.legacy_status.setObjectName("legacy_status")
        self.legacy_status.setWordWrap(True)
        self.dewow_status = QtWidgets.QLabel("Dewow：等待数据")
        self.dewow_status.setObjectName("dewow_status")
        self.bscan_filter_status = QtWidgets.QLabel("Flat Filter：已禁用")
        self.bscan_filter_status.setObjectName("bscan_filter_status")
        self.trace_label = QtWidgets.QLabel("暂无 Trace")
        self.trace_label.setObjectName("trace_label")
        self.trace_label.setWordWrap(True)
        details_layout.addRow("监听地址", self.listen_label)
        details_layout.addRow("保存目录", self.out_dir_label)
        details_layout.addRow("最近事件", self.status_label)
        details_layout.addRow("当前 Trace", self.trace_label)
        details_layout.addRow("采集元数据", self.acquisition_label)
        details_layout.addRow("Dewow", self.dewow_status)
        details_layout.addRow("Flat Filter", self.bscan_filter_status)
        details_layout.addRow("旧格式", self.legacy_status)
        self.details_window.resize(640, 360)

        # ---- 两个设置对话框（非模态，打开时加载已提交值）----
        self.acquisition_dialog = AcquisitionSettingsDialog(self)
        self.bscan_dialog = BscanSettingsDialog(self)

    def open_acquisition_settings(self) -> None:
        self.acquisition_dialog.load_from(self.acq_settings)
        self.acquisition_dialog.show()
        self.acquisition_dialog.raise_()

    def open_bscan_settings(self) -> None:
        self.bscan_dialog.load_from(self.bscan_settings)
        self.bscan_dialog.show()
        self.bscan_dialog.raise_()

    def show_log_window(self) -> None:
        self.log_window.show()
        self.log_window.raise_()

    def show_details_window(self) -> None:
        self.details_window.show()
        self.details_window.raise_()

    def show_about(self) -> None:
        QtWidgets.QMessageBox.about(self, "关于", "UAV-GPR 地面站")

    def apply_acquisition_settings(self, settings: AcquisitionUiSettings) -> None:
        self.acq_settings = settings
        self.log("采集设置已应用")

    def apply_bscan_settings(self, settings: BscanUiSettings) -> None:
        self.bscan_settings = settings
        self.update_bscan_aspect_ratio(settings.aspect_ratio)
        self.refresh_bscan_views()
        self.log("B-scan 设置已应用")

    def log(self, text: str) -> None:
        self.status_label.setText(text)
        self.log_box.appendPlainText(f"[{datetime.now().strftime('%H:%M:%S')}] {text}")

    def _on_receiver_status(self, text: str) -> None:
        self.log(text)
        lowered = text.lower()
        if text.startswith("Connected") or "remote ready" in lowered:
            self.connection_status_label.setText("连接状态：已连接")
        elif "disconnected" in lowered or "client error" in lowered:
            self.connection_status_label.setText("连接状态：已断开")
        elif lowered.startswith("listening"):
            self.connection_status_label.setText("连接状态：等待连接")

    def _on_command_status(self, text: str) -> None:
        self.log(text)
        lowered = text.lower()
        # 失败 / 无连接先判定；二者都不算明确成功 ACK。
        if "failed" in lowered or "no remote" in lowered:
            self.acquisition_status_label.setText("采集状态：命令发送失败")
            return
        # 只有“Remote accepted: <action>”这类明确成功 ACK 才更新为已确认状态。
        if lowered.startswith("remote accepted:"):
            action = text.split(":", 1)[1].strip().lower()
            if action == "start":
                self.acquisition_status_label.setText("采集状态：采集中")
            elif action == "stop":
                # command_ack 只表示停止命令已被空中端接受；"已停止"必须等
                # acquisition_status 报告 stopped 且队列归零（第1轮检查点D）。
                self.acquisition_status_label.setText("采集状态：正在停止（等待排空）")
            elif action == "single":
                self.acquisition_status_label.setText("采集状态：单次采集已接受")
            # 未知 action 不改变状态，避免误显示为“采集中”。

    def _on_acquisition_status(self, header: dict) -> None:
        """空中端 acquisition_status 状态消息（第1轮检查点D + 复审统一）。

        空中端最终一律发 ``state="stopped"``（``drain_failed`` 除外），排空
        结果在 ``drain_state`` 字段。"已停止（数据已全部排空）"显示必须等
        :meth:`_reconcile_stop_state` 全部条件满足，不得在收到状态时立即显示。
        """
        state = str(header.get("state") or "")
        drain_state = str(header.get("drain_state") or "")
        detail = {
            key: header.get(key)
            for key in (
                "run_id",
                "drain_state",
                "selected_count",
                "processed_count",
                "processing_queue_depth",
                "reason",
            )
            if header.get(key) is not None
        }
        if state == "stopping":
            self.acquisition_status_label.setText("采集状态：正在停止（排空中）")
        elif state == "stopped":
            # 迟到的旧 run_id stopped 不得结束新任务（第三次定向修复）。
            new_run = str(header.get("run_id") or "")
            cur = self._task_run_id
            if cur and new_run and new_run != cur:
                self.log(
                    f"忽略迟到停止状态: run_id={new_run}（当前任务={cur}）"
                )
                return
            # 记录最终状态；真正的"已停止（数据已全部排空）"由对账决定。
            self._final_stop_state = dict(header)
            # 停止提交屏障：此刻已入队的全部 Trace 必须全部完成提交
            # （committed+rejected+failed 达到该屏障）才可进入已停止。
            fifo_stats = self.display_fifo.statistics()
            self._stop_commit_barrier = int(fifo_stats["total_enqueued"])
            self.acquisition_status_label.setText("采集状态：正在停止（等待地面显示完成）")
            self._reconcile_stop_state()
        elif state == "drain_failed":
            new_run = str(header.get("run_id") or "")
            cur = self._task_run_id
            if cur and new_run and new_run != cur:
                self.log(
                    f"忽略迟到失败状态: run_id={new_run}（当前任务={cur}）"
                )
                return
            self._final_stop_state = None  # 失败终态：直接显示，不再对账。
            self.acquisition_status_label.setText("采集状态：排空失败（部分数据未处理）")
        elif state == "acquiring":
            # 收到 acquiring：新任务开始。run_id 切换时建立新基线。
            new_run = str(header.get("run_id") or "")
            if new_run and new_run != self._task_run_id:
                self._begin_task(run_id=new_run)
                self._final_stop_state = None
                self._stop_commit_barrier = None
                self._last_ground_run_id = new_run
            self.acquisition_status_label.setText("采集状态：采集中")
        else:
            # 未知/原始英文状态值一律映射为中文未知，不透出原始值。
            self.acquisition_status_label.setText("采集状态：未知状态")
        if detail:
            self.log(f"空中端状态: {state} {detail}")
        _ = drain_state

    def _begin_task(self, run_id: str) -> None:
        """建立任务级计数基线（第三次定向修复）。

        记录 decoder/FIFO 当前累计值的不可变快照（baseline）。所有停止对账
        只看本轮增量，绝不再用窗口生命周期累计值。多次 _begin_task 不会
        影响正在被其他线程使用的累计值——只覆盖 baseline 引用。
        """
        decoder_stats = self.decoder.statistics()
        fifo_stats = self.display_fifo.statistics()
        self._task_baseline = {
            "decoder_submitted": decoder_stats["submitted"],
            "decoder_decoded_success": decoder_stats["decoded_success"],
            "decoder_rejected": decoder_stats["rejected"],
            "decoder_callback_errors": decoder_stats["callback_errors"],
            "decoder_callback_consumed": decoder_stats["callback_consumed"],
            "fifo_enqueued": fifo_stats["total_enqueued"],
            "fifo_dequeued": fifo_stats["total_dequeued"],
            "fifo_committed": fifo_stats["total_committed"],
            "fifo_rejected": fifo_stats["total_rejected"],
            "fifo_failed": fifo_stats["total_failed"],
        }
        self._task_run_id = run_id
        # 显式覆盖 receiver 端的跨重启去重状态——新 run_id 全新开始。
        receiver = getattr(self, "receiver", None)
        if receiver is not None and hasattr(receiver, "begin_run"):
            receiver.begin_run(run_id)

    def _task_delta(self) -> dict[str, int]:
        """返回本任务（自 _begin_task 起）的所有计数增量。"""
        baseline = self._task_baseline or {}
        decoder_stats = self.decoder.statistics()
        fifo_stats = self.display_fifo.statistics()
        deltas: dict[str, int] = {}
        for key, current in (
            ("decoder_submitted", decoder_stats["submitted"]),
            ("decoder_decoded_success", decoder_stats["decoded_success"]),
            ("decoder_rejected", decoder_stats["rejected"]),
            ("decoder_callback_errors", decoder_stats["callback_errors"]),
            ("decoder_callback_consumed", decoder_stats["callback_consumed"]),
            ("fifo_enqueued", fifo_stats["total_enqueued"]),
            ("fifo_dequeued", fifo_stats["total_dequeued"]),
            ("fifo_committed", fifo_stats["total_committed"]),
            ("fifo_rejected", fifo_stats["total_rejected"]),
            ("fifo_failed", fifo_stats["total_failed"]),
        ):
            deltas[key] = current - baseline.get(key, 0)
        return deltas

    def _reconcile_stop_state(self) -> bool:
        """停止对账：全部条件满足才显示"已停止（数据已全部排空）"。

        **第三次定向修复**（与第二次并存）：

        1. 任务级增量：使用 ``_task_delta()``（本轮基线之后）评估成功/
           失败，绝不再用窗口生命周期累计值。
        2. 失败终态分类：任意一种（解码拒绝 / 回调异常 / 显示兼容性拒绝
           / UI 提交异常）必须显示对应的中文失败文案；**绝不可**被
           "数据已全部排空"覆盖。
        3. 屏障完整性：FIFO depth==0、unresolved==0、decoder queued/
           in_flight/in_callback==0 显式检查（不再依赖隐式状态机）。
        4. 提交屏障：committed+rejected+failed ≥ 屏障值。
        5. 进程累计值仍按全生命周期累加（其他线程/历史任务仍在使用）；
           本函数只看本任务增量。
        """
        final_state = getattr(self, "_final_stop_state", None)
        if final_state is None:
            return False
        drain_state = str(final_state.get("drain_state") or "")
        if drain_state != "drained":
            # 空中端排空结果未知/失败：保持"正在停止"或失败文案。
            return False
        delta = self._task_delta()
        decoder_stats = self.decoder.statistics()
        # ---- 失败终态分类（第三次定向修复）----
        failure_message = self._classify_task_failure(delta)
        if failure_message is not None:
            self.acquisition_status_label.setText(failure_message)
            self._final_stop_state = None
            self._stop_commit_barrier = None
            return False
        # ---- 屏障完整性（显式）----
        if (
            decoder_stats["queued"] > 0
            or decoder_stats["in_flight"] > 0
            or decoder_stats["in_callback"] > 0
        ):
            pending = (
                decoder_stats["queued"]
                + decoder_stats["in_flight"]
                + decoder_stats["in_callback"]
            )
            self.acquisition_status_label.setText(
                f"采集状态：正在停止（解码/回调 {pending} 条）"
            )
            return False
        fifo_stats = self.display_fifo.statistics()
        if fifo_stats["depth"] > 0 or fifo_stats["unresolved"] > 0:
            pending = fifo_stats["depth"] + fifo_stats["unresolved"]
            self.acquisition_status_label.setText(
                f"采集状态：正在停止（待显示/未结算 {pending} 条）"
            )
            return False
        # 停止提交屏障：最终状态到达时记录的提交计数屏障。
        barrier = getattr(self, "_stop_commit_barrier", None)
        if barrier is not None:
            resolved = (
                fifo_stats["total_committed"]
                + fifo_stats["total_rejected"]
                + fifo_stats["total_failed"]
            )
            if resolved < barrier:
                self.acquisition_status_label.setText(
                    f"采集状态：正在停止（最后几条提交中 {barrier - resolved} 条）"
                )
                return False
        # 全部条件满足：最终显示。
        self.acquisition_status_label.setText("采集状态：已停止（数据已全部排空）")
        self.log("停止对账完成：空中已排空、地面解码回调与显示提交均已归零")
        self._final_stop_state = None  # 终态已呈现，避免重复触发。
        self._stop_commit_barrier = None
        return True

    def _classify_task_failure(self, delta: dict[str, int]) -> str | None:
        """第三次定向修复：四类失败优先级分类，**不冲突**。

        返回失败文案（含计数明细），均明确否定"数据已全部排空"。
        优先级：回调异常 → 解码拒绝 → 显示兼容拒绝 → UI 提交异常。
        """
        ce = delta.get("decoder_callback_errors", 0)
        if ce > 0:
            return f"采集状态：地面处理失败（解码回调异常 {ce} 条，未视为已停止）"
        dr = delta.get("decoder_rejected", 0)
        if dr > 0:
            return f"采集状态：地面处理失败（解码拒绝 {dr} 条，未视为已停止）"
        fr = delta.get("fifo_rejected", 0)
        if fr > 0:
            return f"采集状态：地面处理失败（显示兼容性拒绝 {fr} 条，未视为已停止）"
        ff = delta.get("fifo_failed", 0)
        if ff > 0:
            return f"采集状态：地面处理失败（UI 提交异常 {ff} 条，未视为已停止）"
        return None

    def command_params(self, single: bool = False) -> dict[str, object]:
        return self.acq_settings.command_params(single=single)

    def send_start(self) -> None:
        self._final_stop_state = None  # 新任务：清除旧停止对账状态。
        self._stop_commit_barrier = None
        self._begin_task(run_id=f"start-{int(time.time())}")
        self.acquisition_status_label.setText("采集状态：正在发送启动命令")
        self.receiver.send_command("start", self.command_params())

    def send_single(self) -> None:
        self._final_stop_state = None
        self._stop_commit_barrier = None
        self._begin_task(run_id=f"single-{int(time.time())}")
        self.acquisition_status_label.setText("采集状态：单次采集命令已发送")
        self.receiver.send_command("single", self.command_params(single=True))

    def send_stop(self) -> None:
        self.acquisition_status_label.setText("采集状态：正在发送停止命令")
        self.receiver.send_command("stop")

    def clear_bscan(self) -> None:
        self.bscan_rows = {name: [] for name in UI_CHANNELS}
        self.bscan_time_ns = None
        self.bscan_processing_signature = None
        self.bscan_filter_status_override = None
        self.bscan_window_start = 1
        self.bscan_follow_latest = True
        self.bscan_window_slider.blockSignals(True)
        self.bscan_window_slider.setRange(1, 1)
        self.bscan_window_slider.setValue(1)
        self.bscan_window_slider.blockSignals(False)
        self.bscan_window_label.setText("显示 1–0 / 共 0 道")
        self.bscan_filter_status.setText(
            "Flat Filter：已禁用"
            if not self.bscan_settings.flat_filter_enabled
            else "Flat Filter：无数据"
        )
        self.dewow_status.setText("Dewow：等待数据")
        for name, image in self.bscan_images.items():
            image.clear()
            self.bscan_plots[name].setXRange(0.5, BSCAN_WINDOW_TRACES + 0.5, padding=0)
        self.log("B-scan 已清空")

    def _append_bscan_trace(
        self,
        time_ns: np.ndarray,
        trace_by_channel: dict[str, np.ndarray],
        processing_signature: tuple[bool, float | None] | None = None,
    ) -> bool:
        try:
            candidate_time, candidate_traces = validate_bscan_trace(time_ns, trace_by_channel)
        except (TypeError, ValueError) as exc:
            self.bscan_filter_status_override = f"B-scan已拒绝：{exc}；请清空B-scan后重新开始"
            return False
        if self.bscan_time_ns is not None and not bscan_time_axes_compatible(self.bscan_time_ns, candidate_time):
            self.bscan_filter_status_override = (
                "B-scan已拒绝：时间轴已变化；请清空B-scan后重新开始"
            )
            return False
        if (
            self.bscan_processing_signature is not None
            and processing_signature is not None
            and self.bscan_processing_signature != processing_signature
        ):
            self.bscan_filter_status_override = (
                "B-scan已拒绝：Dewow设置已变化；请清空B-scan后继续"
            )
            return False

        total_before = self.bscan_trace_count()
        was_latest = self.bscan_follow_latest or self.bscan_window_start >= bscan_window_max_start(total_before)
        if self.bscan_time_ns is None:
            self.bscan_time_ns = candidate_time
        if self.bscan_processing_signature is None and processing_signature is not None:
            self.bscan_processing_signature = processing_signature
        for name in UI_CHANNELS:
            self.bscan_rows[name].append(candidate_traces[name])
        total_after = self.bscan_trace_count()
        self.bscan_window_start = bscan_start_after_append(
            self.bscan_window_start,
            total_before,
            total_after,
            was_latest,
        )
        self.bscan_follow_latest = was_latest
        self.bscan_filter_status_override = None
        return True

    def bscan_trace_count(self) -> int:
        return max((len(self.bscan_rows.get(name, [])) for name in UI_CHANNELS), default=0)

    def _update_bscan_window_controls(self) -> None:
        total = self.bscan_trace_count()
        maximum = bscan_window_max_start(total)
        self.bscan_window_start = min(max(self.bscan_window_start, 1), maximum)
        start, end = bscan_window_bounds(total, self.bscan_window_start)
        self.bscan_window_slider.blockSignals(True)
        self.bscan_window_slider.setRange(1, maximum)
        self.bscan_window_slider.setValue(self.bscan_window_start)
        self.bscan_window_slider.blockSignals(False)
        self.bscan_window_label.setText(f"显示 {start}–{end} / 共 {total} 道")

    def on_bscan_window_changed(self, value: int) -> None:
        total = self.bscan_trace_count()
        maximum = bscan_window_max_start(total)
        self.bscan_window_start = min(max(int(value), 1), maximum)
        self.bscan_follow_latest = self.bscan_window_start >= maximum
        self.refresh_bscan_views()

    def update_bscan_aspect_ratio(self, value: float) -> None:
        for container in getattr(self, "bscan_containers", {}).values():
            container.set_aspect_ratio(value)

    def flat_filter_window(self) -> int:
        value = max(3, int(self.bscan_settings.flat_window_traces))
        if value % 2 == 0:
            value -= 1
        return value

    def _bscan_matrix_for_display(self, name: str) -> np.ndarray | None:
        rows = self.bscan_rows.get(name, [])
        if not rows:
            return None
        matrix = np.column_stack([np.asarray(row, dtype=float) for row in rows])
        flat_window = self.flat_filter_window()
        if self.bscan_settings.flat_filter_enabled and matrix.shape[1] >= flat_window:
            matrix = flat_reflection_filter(matrix, window_size=flat_window)
        return matrix

    def _bscan_lut(self):
        """缓存的 LUT：只在色图设置变化时重算（第1轮检查点B）。"""
        cmap_name = self.bscan_settings.colormap
        cached_name = getattr(self, "_bscan_lut_name", None)
        if cached_name == cmap_name:
            return getattr(self, "_bscan_lut_value", None)
        lut = None
        if cmap_name == "jet":
            lut = pg.colormap.get("CET-R4").getLookupTable(0.0, 1.0, 256)
        self._bscan_lut_name = cmap_name
        self._bscan_lut_value = lut
        return lut

    def _bscan_display_window(self, name: str) -> tuple[np.ndarray | None, tuple[float, float] | None]:
        """构造当前通道的 [n_time, 64] 显示窗口与显式色阶。

        - 只构造可见窗口（成本与历史长度无关）；
        - Flat Filter 开启时执行邻域窗口化滤波（结果与全历史滤波一致）；
        - 色阶显式计算：实部关于 0 对称 [-max_abs, +max_abs]；全零/全 NaN
          时使用安全非零范围；S11/S22 各自独立计算。
        """
        rows = self.bscan_rows.get(name, [])
        if not rows:
            return None, None
        flat_window = self.flat_filter_window()
        if self.bscan_settings.flat_filter_enabled:
            matrix, _applied = apply_flat_filter_windowed(
                rows, self.bscan_window_start, flat_window, filter_func=flat_reflection_filter
            )
        else:
            matrix = bscan_window_from_rows(rows, self.bscan_window_start).matrix
        finite = matrix[np.isfinite(matrix)]
        if finite.size:
            max_abs = float(np.max(np.abs(finite)))
        else:
            max_abs = 0.0
        if not np.isfinite(max_abs) or max_abs <= 0.0:
            max_abs = 1.0
        return matrix, (-max_abs, max_abs)

    def refresh_bscan_views(self) -> None:
        self._update_bscan_window_controls()
        lut = self._bscan_lut()
        lut_changed = lut is not getattr(self, "_bscan_lut_applied", None)
        total = self.bscan_trace_count()
        flat_window = self.flat_filter_window()
        if self.bscan_filter_status_override is not None:
            self.bscan_filter_status.setText(self.bscan_filter_status_override)
        elif not self.bscan_settings.flat_filter_enabled:
            self.bscan_filter_status.setText("Flat Filter：已禁用")
        elif total == 0:
            self.bscan_filter_status.setText("Flat Filter：无数据")
        elif total < flat_window:
            self.bscan_filter_status.setText(
                f"Flat Filter：等待{flat_window}道，当前显示未滤波数据"
            )
        else:
            self.bscan_filter_status.setText(f"Flat Filter：已启用，窗口{flat_window}道")
        for name in UI_CHANNELS:
            image_item = self.bscan_images.get(name)
            if image_item is None:
                continue
            matrix, levels = self._bscan_display_window(name)
            if matrix is None:
                continue
            # row-major：直接传 [n_time, n_trace]，无转置；显式 levels。
            image_item.setImage(np.ascontiguousarray(matrix), autoLevels=False, levels=levels)
            if lut_changed:
                image_item.setLookupTable(lut)
            if self.bscan_time_ns is not None and self.bscan_time_ns.size:
                time_start = float(self.bscan_time_ns[0])
                time_stop = float(self.bscan_time_ns[-1])
                time_span = max(time_stop - time_start, 1e-9)
                x_start = float(self.bscan_window_start) - 0.5
                image_item.setRect(QtCore.QRectF(x_start, time_start, float(BSCAN_WINDOW_TRACES), time_span))
                plot = self.bscan_plots[name]
                plot.setXRange(x_start, x_start + BSCAN_WINDOW_TRACES, padding=0)
                plot.setYRange(time_start, time_stop, padding=0)
        self._bscan_lut_applied = lut

    def on_trace(self, header: dict, payload: bytes, path: str) -> None:
        """同步兼容/测试入口：解码入队后**立即逐条提交**（不绕开 FIFO 语义）。

        生产路径不走这里：ReceiverThread 的 trace_received 信号连接到
        :meth:`_on_receiver_trace_received`，由后台解码线程执行
        :func:`decode_ground_trace_payload`，主线程只做显示提交。
        """
        import time as _time

        try:
            decoded = decode_ground_trace_payload(header, payload, path)
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.log(f"Trace已忽略：NPZ数据无效 ({exc})")
            return
        started = _time.perf_counter()
        self.display_fifo.enqueue(decoded)
        self._process_one_display_item()
        # 兼容入口允许把积压一次提交完（同步语义），但每条仍逐个提交。
        while self.display_fifo.depth() > 0:
            if not self._process_one_display_item():
                break

    # ---- 第1轮检查点B：后台解码 + 显式 FIFO 显示提交 ----

    def _on_receiver_trace_received(self, header: dict, payload: bytes, path: str) -> None:
        """生产路径：接收线程 → 后台解码线程（不在主线程解压 NPZ）。

        处理完成（含异常/线程退出后的拒绝）必须在 finally 归还 receiver
        的待处理信号计数（closeEvent 的 Qt 屏障判断依赖它）。
        """
        try:
            self.decoder.put_job(header, payload, path)
        except RuntimeError as exc:
            # decoder 已退出（极端关闭竞态）：明确记录，绝不静默丢弃。
            self.log(f"解码线程已退出，Trace {header.get('trace_index')} 无法处理: {exc}")
        finally:
            consumed = getattr(self.receiver, "note_signal_consumed", None)
            if callable(consumed):
                try:
                    consumed()
                except Exception:  # noqa: BLE001
                    pass

    def _on_trace_rejected(self, header: dict, path: str, exc: BaseException) -> None:
        """解码失败回调（主线程，经 _DecoderBridge 信号）。

        处理完成（含异常路径）必须在 finally 中确认 callback consumed，
        否则 decoder 屏障计数永久卡住。
        """
        try:
            trace_index = header.get("trace_index") if isinstance(header, dict) else "?"
            self.log(f"Trace {trace_index} 已拒绝：NPZ数据无效 ({exc})")
        except Exception as exc2:  # noqa: BLE001 - 回调异常明确上报
            self.log(f"解码拒绝回调处理失败: {type(exc2).__name__}: {exc2}")
        finally:
            self.decoder.mark_callback_consumed()

    def _on_trace_decoded(self, decoded: DecodedGroundTrace) -> None:
        """解码成功回调（主线程，经 _DecoderBridge 信号）。

        复审修复（逐条提交）：这里**只入队**——与历史相关的兼容性校验、
        S11/S22 追加、滑块、当前 Trace/GNSS/metadata 状态与 refresh 全部
        延迟到该 Trace 被 pop 提交时执行；尚未 pop 的 Trace 不得提前出现在
        当前显示矩阵或状态中。
        处理完成（含异常路径）必须在 finally 中确认 callback consumed。

        第三次定向修复：``ground_display_enqueued`` 测的是 decoder 完成
        → Qt 主线程收到 queued 信号并入显示 FIFO 的真实等待（用
        ``decoded.decoded_at_perf``），不再只是 enqueue() 自身的几微秒。
        ``enqueue`` 自身成本另起 ``ground_display_enqueue_cost`` 指标。
        """
        import time as _time

        try:
            cross_thread_t0 = _time.perf_counter()
            enqueue_t0 = _time.perf_counter()
            self.display_fifo.enqueue(decoded)
            self.metrics.record_stage(
                "ground_display_enqueue_cost", _time.perf_counter() - enqueue_t0
            )
            # 跨线程真实等待：decoder 线程完成到主线程入队之间的延迟。
            decoded_at = getattr(decoded, "decoded_at_perf", 0.0) or 0.0
            if decoded_at > 0.0:
                self.metrics.record_stage(
                    "ground_display_enqueued", cross_thread_t0 - decoded_at
                )
            self._schedule_display_commit()
        except Exception as exc:  # noqa: BLE001 - 回调异常明确上报
            self.log(f"解码回调处理失败: {type(exc).__name__}: {exc}")
        finally:
            self.decoder.mark_callback_consumed()

    def _process_one_display_item(self) -> bool:
        """弹出并提交**一条** Trace（校验+追加+状态+一次 refresh）。

        返回 False 表示控件已销毁（放弃绘制）。计数语义：
        - 成功（校验+追加+setImage+状态全部完成）→ ``mark_committed``；
        - 兼容性拒绝（时间轴/Dewow/无效 Trace）→ ``mark_rejected``；
        - UI 提交异常 → ``mark_failed``。
        被拒绝的 Trace 不更新历史、当前 Trace、GNSS 或 committed 计数。
        """
        import time as _time

        decoded = self.display_fifo.pop()
        if decoded is None:
            return False
        started = _time.perf_counter()
        try:
            gnss = decoded.gnss
            flight = decoded.flight
            acquisition = decoded.acquisition
            if decoded.legacy_trace is not None:
                # 真实旧格式：legacy trace 明确映射为 S11；不生成/伪造 S22 显示。
                try:
                    validate_bscan_trace(
                        decoded.time_ns, {"S11": decoded.legacy_trace}, channels=("S11",)
                    )
                except (TypeError, ValueError) as exc:
                    self.log(f"Trace已忽略：NPZ数据无效 ({exc})")
                    self.display_fifo.mark_rejected()
                    return True
                self.legacy_status.setText("旧格式NPZ仅包含S11；没有S22，未加入双通道B-scan")
                self.dewow_status.setText("Dewow：旧格式默认启用")
            else:
                trace_candidates = {"S11": decoded.trace_s11, "S22": decoded.trace_s22}
                try:
                    validated_time, validated_traces = validate_bscan_trace(
                        decoded.time_ns, trace_candidates
                    )
                except (TypeError, ValueError) as exc:
                    self.log(f"Trace {decoded.trace_index} 已拒绝：{exc}")
                    self.display_fifo.mark_rejected()
                    return True  # 拒绝不更新历史/状态/committed。
                appended = self._append_bscan_trace(
                    validated_time, validated_traces, decoded.processing_signature
                )
                if not appended:
                    # 兼容性拒绝（时间轴/Dewow signature）：只显示拒绝原因，不提交状态。
                    try:
                        self.refresh_bscan_views()
                    except RuntimeError:
                        self.display_fifo.mark_failed()
                        return False
                    self.log("B-scan已拒绝：未追加该 Trace")
                    self.display_fifo.mark_rejected()
                    return True
                if decoded.dewow_state == "legacy":
                    self.dewow_status.setText("Dewow：旧格式默认启用")
                elif decoded.processing_signature[0]:
                    self.dewow_status.setText(
                        f"Dewow：已启用，窗口{decoded.processing_signature[1]:g} ns"
                    )
                else:
                    self.dewow_status.setText("Dewow：已禁用")
                self.legacy_status.setText("")

            # ---- 该 Trace 的状态提交（pop 时才可见）----
            self._update_bscan_window_controls()
            try:
                self.refresh_bscan_views()
            except RuntimeError:
                # 窗口/控件已销毁（如测试 tearDown 期间延迟回调触发）：放弃绘制。
                self.display_fifo.mark_failed()
                return False
            self.acquisition_label.setText(_format_acquisition_label(acquisition, flight))
            self.gnss_map.update_fix(
                latitude=gnss.get("latitude"),
                longitude=gnss.get("longitude"),
                fix_quality=gnss.get("fix_quality", 0),
                satellites=gnss.get("satellites", 0),
                altitude_m=gnss.get("altitude_m"),
                utc=gnss.get("utc", ""),
            )
            self.trace_status_label.setText(f"当前 Trace：{decoded.trace_index}")
            self.trace_label.setText(
                f"Trace {decoded.trace_index} | 定位={gnss.get('fix_quality', 0)} 卫星数={gnss.get('satellites', 0)} "
                f"纬度={gnss.get('latitude', '-')} 经度={gnss.get('longitude', '-')} "
                f"高度={gnss.get('altitude_m', '-')} | {Path(decoded.receiver_path).name}"
            )
            # 成功提交：普通成功日志按每 25 条汇总（拒绝/错误/积压仍即时）。
            self._trace_log_counter += 1
            self._trace_log_accumulated += 1
            if self._trace_log_accumulated >= 25:
                self._trace_log_accumulated = 0
                self.log(f"已持续显示 Trace（累计 {self._trace_log_counter} 条提交）")
            # committed 只在成功追加并完成 setImage 提交后计数。
            self.display_fifo.mark_committed()
            self.metrics.record_stage("ground_displayed", _time.perf_counter() - started)
        except Exception as exc:  # noqa: BLE001 - 提交异常明确上报，不伪装成功
            self.display_fifo.mark_failed()
            self.log(
                f"Trace {getattr(decoded, 'trace_index', '?')} 显示提交失败: "
                f"{type(exc).__name__}: {exc}"
            )
            return True
        fifo_stats = self.display_fifo.statistics()
        depth = int(fifo_stats["depth"])
        oldest = fifo_stats["oldest_age_ms"]
        # 积压告警：不丢弃，只提示（限频避免日志刷屏）。
        if depth >= 8 and oldest is not None and oldest > 500.0:
            if _time.perf_counter() - getattr(self, "_last_backlog_warn", 0.0) > 1.0:
                self._last_backlog_warn = _time.perf_counter()
                self.log(f"显示队列积压：depth={depth}，最旧 {oldest:.0f}ms（继续逐条显示，不丢弃）")
        return True

    def _schedule_display_commit(self) -> None:
        """一次定时回调最多提交一条 Trace，随后把控制权还给 Qt 事件循环。"""
        if self._display_timer_active:
            return
        self._display_timer_active = True
        QtCore.QTimer.singleShot(0, self._commit_one_display)

    def _commit_one_display(self) -> None:
        alive = self._process_one_display_item()
        if not alive:
            # 控件已销毁：停止调度（不清空待显示数据以外的状态）。
            self._display_timer_active = False
            return
        if self.display_fifo.depth() > 0:
            QtCore.QTimer.singleShot(0, self._commit_one_display)
        else:
            self._display_timer_active = False
            # 队列排空：若停止对账条件已满足，最终显示"已停止"。
            self._reconcile_stop_state()

    def closeEvent(self, event) -> None:
        """fail-closed 关闭（第二次定向修复的完整屏障链）：

        1. 先停止并**等待** ReceiverThread 退出——此后不再产生新解码任务；
        2. receiver 已退出后，仍要检查**已 emit 但主线程尚未执行的
           trace_received 信号**（``emitted_pending``）；
        3. decoder 排空已有任务并停止，**检查 stop 返回值**；
        4. decoder 已退出后仍要检查**回调屏障**（in_callback：已解码完成、
           Qt queued decoded/rejected 回调尚未在主线程处理）；
        5. 显示 FIFO 非空、或有出队未确认（unresolved）→ ignore；
        6. 任一非零 → ``event.ignore()``：保持窗口，事件循环继续排空，
           用户稍后再次关闭即可成功。绝不 terminate、绝不清空队列、
           绝不丢弃回调。
        """
        receiver = self.receiver
        if receiver is None:
            super().closeEvent(event)
            return
        # ---- 1. receiver 先停（不再产生新解码任务）----
        try:
            receiver.stop()
        except Exception as exc:  # noqa: BLE001 - 清理失败不阻止记录，但 fail-closed
            self.log(f"receiver.stop() 抛错: {type(exc).__name__}: {exc}")
        try:
            receiver_exited = receiver.wait(1500)
        except Exception as exc:  # noqa: BLE001 - wait 异常按未退出处理
            receiver_exited = False
            self.log(f"receiver.wait() 抛错: {type(exc).__name__}: {exc}")
        if not receiver_exited:
            event.ignore()
            self.log("接收线程仍在退出中，窗口关闭已忽略；请稍后重试关闭")
            return
        # ---- 2. receiver→主线程的 Qt 信号屏障 ----
        pending_signals = 0
        pending_fn = getattr(receiver, "emitted_pending", None)
        if callable(pending_fn):
            try:
                pending_signals = int(pending_fn())
            except Exception:  # noqa: BLE001
                pending_signals = -1
        if pending_signals != 0:
            event.ignore()
            self.log(f"仍有 {pending_signals} 条接收信号待主线程处理，窗口关闭已忽略；稍后重试")
            return
        # ---- 3. decoder 排空并停止（检查返回值）----
        try:
            decoder_exited = self.decoder.stop(2000)
        except Exception as exc:  # noqa: BLE001
            decoder_exited = False
            self.log(f"decoder.stop() 抛错: {type(exc).__name__}: {exc}")
        if not decoder_exited:
            event.ignore()
            self.log("解码线程未在时限内退出，窗口关闭已忽略；请稍后重试关闭")
            return
        # ---- 4. decoder→主线程回调屏障 + 队列 ----
        decoder_stats = self.decoder.statistics()
        if (
            decoder_stats["queued"] > 0
            or decoder_stats["in_flight"] > 0
            or decoder_stats["in_callback"] > 0
        ):
            event.ignore()
            self.log(
                "解码线程已退出但仍有未处理任务/待处理回调，窗口关闭已忽略"
            )
            return
        # ---- 5. 显示 FIFO 与出队未确认 ----
        fifo_stats = self.display_fifo.statistics()
        if fifo_stats["depth"] > 0 or fifo_stats["unresolved"] > 0:
            event.ignore()
            self.log(
                f"仍有 {fifo_stats['depth']} 条待显示 Trace"
                f"（未确认 {fifo_stats['unresolved']} 条），窗口关闭已忽略；稍后重试"
            )
            return
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
