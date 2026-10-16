#!/usr/bin/env python3
"""Ground station GUI for receiving and controlling UAV-GPR acquisition."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import os
import socket
import struct
import sys
import threading
import time
import uuid
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
from ground_gnss import GroundGnssMapWidget as GnssMapWidget, normalize_gnss
from gpr_processing import flat_reflection_filter
from pipeline_metrics import PipelineMetrics
# 第2轮 §3.7-9：地面 run_id 词汇表的**唯一正则源**（air_task/idle/校准/local
# 四类单一分类）。禁止在本模块再写第二套 run_id 正则（R-A-19/R-A-19b/R-A-20
# 三处判据共用同一函数）。
from disk_outbox import (  # noqa: E402
    RUN_ID_CLASS_AIR_TASK,
    RUN_ID_CLASS_CALIBRATION,
    RUN_ID_CLASS_IDLE,
    RUN_ID_CLASS_LOCAL,
    classify_run_id,
)


PROTOCOL = "uav-gpr-telemetry-v1"
S_PARAMETERS = ("S11", "S21", "S12", "S22")
UI_CHANNELS = ("S11", "S22")
CHANNEL_LABELS = {"S11": "HH (S11)", "S22": "VV (S22)"}

#: §5.4：空中→地面校准回程消息类型前缀（旧地面端落入 handle_client 的
#: "忽略未知消息"分支；R-C-13 有测试）。
_CALIBRATION_MESSAGE_PREFIX = "calibration_"

# ---------------------------------------------------------------------------
# 第7轮（D1+D5）：地面端「空中端设备」面板——协议词表
#
# 协议铁律（第7轮计划 §4.6/§5）：全部经既有 ``command`` 消息**新增 action 值**
# 实现，严禁新增消息 type（空中端接收逻辑对未知 type 会直接断链）；回执走
# 既有 ``command_ack`` 通道 + 新增 ``result``/``supported`` 结果字段（既有
# ack 字段零改动）。词表与 t1 黑匣子（uav_gpr_air_service.py）完全一致；
# 需改词表必须先上报 captain。
# ---------------------------------------------------------------------------

#: 设备面板远程 action 词表（与 t1 黑匣子逐字一致）。
AIR_DEVICE_ACTIONS: tuple[str, ...] = (
    "get_status",
    "test_vna",
    "list_serial_ports",
    "set_gnss_port",
    "test_gnss",
    "set_engine",
    "set_device_source",
)

#: 各 action 的中文标签（面板提示与测试定位用）。
AIR_DEVICE_ACTION_LABELS: dict[str, str] = {
    "get_status": "设备状态总览",
    "test_vna": "测试雷达",
    "list_serial_ports": "列出串口",
    "set_gnss_port": "设置 GNSS 串口",
    "test_gnss": "测试 GNSS",
    "set_engine": "切换采集引擎",
    "set_device_source": "切换模拟器",
}

#: 采集引擎取值（与空中端 air_config.json 的 engine 字段一致）。
AIR_ENGINE_VALUES: tuple[str, ...] = ("continuous", "legacy")

#: 设备来源取值（与空中端 air_config.json 的 device_source 字段一致）。
AIR_DEVICE_SOURCE_VALUES: tuple[str, ...] = ("usb", "simulated")


def is_not_older_than(run_x: object, baseline: object) -> bool:
    """R-A-19b 唯一比较函数：完整 run_id 的**字典序**比较（基线空 = 满足）。

    ``YYYYmmdd_HHMMSS_%04d`` 定长格式保证"字典序 == 时间序"；**禁止**只比
    seq 数值——空中进程重启后 seq 回到 1，只比 seq 会把更晚的新任务误判为
    补传（R-A-19b 写死）。
    """
    run = run_x if isinstance(run_x, str) else ""
    base = baseline if isinstance(baseline, str) else ""
    if not base:
        return True
    return run >= base


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
    try:
        if "gnss" in files:
            values.append(data["gnss"])
    except Exception:
        # GNSS is optional: a corrupt/object-array field cannot reject radar data.
        pass
    values.append(header.get("gnss"))
    for value in values:
        try:
            parsed = normalize_gnss(value)
        except Exception:
            continue
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
    """A height-for-width container for a visual width:height ratio.

    第6轮 t3 修复：父链上的 QSplitter 不传播 heightForWidth（取证
    D:\\博士任务\\_t3_forensics\\probe_aspect_geometry_after_fix.txt：容器
    hfw 随比例正确变化但可见几何恒为均分值），QVBoxLayout 对两个等宽
    stretch=1 容器的均分会把比例差完全抵消
    （h = w/r + (H−slider−2·w/r)/2 ≡ 常数）。因此这里按当前宽度给自己加
    ``maximumHeight = width/ratio`` 硬上限（resizeEvent 实时跟随宽度、
    minimumSizeHint 托底防 max<min 冲突），保留父布局 stretch=1 填充：
    可见高度 = min(均分份额, width/ratio)——比例可容纳时精确呈现长宽比
    （letterbox 留白），视图总高放不下时上限不绑定、回落既有均分行为
    （物理极限），默认 ratio=2.0 大窗口外观与修复前一致。
    """

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
        self._apply_height_cap()
        self.updateGeometry()

    def _apply_height_cap(self) -> None:
        """按当前宽度把可见高度上限锁到 width/ratio（t3 修复核心）。"""
        if self.width() <= 0:
            return
        floor_h = self.minimumSizeHint().height()
        self.setMaximumHeight(max(floor_h, int(round(self.width() / self._aspect_ratio))))

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt 命名
        super().resizeEvent(event)
        self._apply_height_cap()


class ReceiverThread(QtCore.QThread):
    status = QtCore.Signal(str)
    trace_received = QtCore.Signal(dict, bytes, str)
    command_status = QtCore.Signal(str)
    #: 第1轮检查点D：空中端 acquisition_status 状态消息（state/queues等）。
    acquisition_status = QtCore.Signal(dict)
    #: 第2轮 R-A-19/R-C-20：hello 触发的 run 切换（窗口合成 acquiring 路径）。
    run_switched = QtCore.Signal(str)
    #: 第2轮 §5.4：校准回程消息（绝不进 _accept_trace/display FIFO，R-C-17）。
    calibration_message = QtCore.Signal(dict)
    #: 第7轮：带结果字段的 command_ack 完整回执（设备面板消费；既有 ack 通道
    #: + 新增 result/supported 结果字段，既有字段零改动）。老空中端的纯 ack
    #: 同样转发，由面板据此判定「对方版本不支持此功能」。
    command_result = QtCore.Signal(dict)

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
        # ---- 第2轮 R-A-19/R-A-19b：地面 run 切换与补传模式 ----
        # 仅在接收线程内读写（handle_client/_accept_trace/_on_hello_run_id
        # 同线程串行），无锁。
        #: 当前基线身份（镜像的空中 run_id；空 = 无基线，重放按正常处理）。
        self._mirrored_air_run_id: str = ""
        #: 补传模式中的旧 run（safe_name 形态；None = 不在补传模式）。
        self._replay_run_id: str | None = None
        #: 补传模式按 run 计数（R-A-19b：可观测性不得静默）。
        self._replay_counts: dict[str, int] = {}

    def begin_run(self, run_id: str) -> None:
        """第三次定向修复：切换 run_id 时调用，重置当前内存去重表。

        仅清空当前会话的 ``_seen_traces``；磁盘上已存在的 trace_*.npz 仍
        走跨重启重读/冲突拒绝路径（不删除磁盘文件）。
        """
        _ = run_id
        with self._emitted_lock:
            self._seen_traces.clear()

    # ---- 第2轮 R-A-19 / R-A-19b / R-C-20：run 切换与补传模式 ----

    def _on_hello_run_id(self, run_id: object) -> None:
        """hello 触发的 run 切换判定（接收线程内串行执行）。

        词汇表唯一判据来源 = ``disk_outbox.classify_run_id``（§3.7-9，禁止
        第二套正则）；比较 = :func:`is_not_older_than`（完整字典序）。
        """
        run_text = run_id if isinstance(run_id, str) else ""
        cls = classify_run_id(run_text)
        if cls in (RUN_ID_CLASS_IDLE, RUN_ID_CLASS_LOCAL):
            return  # 连接身份/本地式：不切换、不改基线（词汇表处置）
        if cls == RUN_ID_CLASS_CALIBRATION:
            # 词汇表：校准身份本轮不上线；一旦出现按补传模式处置（trace 照常
            # 落盘+ACK，绝不 begin_run/绝不显示）+ 记一条意外日志。
            self.status.emit(f"意外的校准身份上线: run_id={run_text!r}（按补传模式处置）")
            self._enter_replay_mode(run_text)
            return
        # ---- air_task ----
        if run_text == self._mirrored_air_run_id:
            return  # R-A-19③：同 run 重连 hello 不得重置窗口任务基线
        if is_not_older_than(run_text, self._mirrored_air_run_id):
            self._exit_replay_mode()
            self.begin_run(run_text)  # 清内存去重表（磁盘判重路径不受影响 [冻结]）
            self._mirrored_air_run_id = run_text
            self.run_switched.emit(run_text)  # 窗口侧合成 acquiring 路径（R-C-20①）
            self.status.emit(f"Run 切换（hello）: {run_text}")
        else:
            self._enter_replay_mode(run_text)

    def _enter_replay_mode(self, run_id: str) -> None:
        """进入补传模式：旧 run 重放仅落盘+ACK，不进显示（R-A-19b）。"""
        run_key = safe_name(run_id, "run")
        if self._replay_run_id == run_key:
            return
        if self._replay_run_id is not None:
            self._exit_replay_mode()
        self._replay_run_id = run_key
        self._replay_counts[run_key] = self._replay_counts.get(run_key, 0)
        self.status.emit(
            f"补传模式: run={run_id} 早于基线（{self._mirrored_air_run_id or '空'}），"
            "该 run 帧仅落盘+ACK，不进入显示"
        )

    def _exit_replay_mode(self) -> None:
        """退出补传模式：按 run 记一条汇总日志（R-A-19b：不静默吞显示）。"""
        if self._replay_run_id is None:
            return
        count = self._replay_counts.get(self._replay_run_id, 0)
        self.status.emit(
            f"补传模式结束: run={self._replay_run_id} 共接收 {count} 条（仅落盘，不显示）"
        )
        self._replay_run_id = None

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

    def _send_frame_locked(
        self, sock: socket.socket, header: dict[str, object], payload: bytes = b""
    ) -> None:
        """第7轮 t7 修复（F1）：全部出站帧统一经既有 _send_lock 串行化。

        一帧 = 3-4 段 sendall；hello-ack/ack_trace 在接收线程、command 在
        主线程并发裸发时同一条 client socket 上会字节交错破帧（终验 F1）。
        send_command 自 R-A-05 起已持 _send_lock；本助手把 hello-ack 与
        ack_trace 纳入同一把锁。锁内只做一次完整帧发送，绝不嵌套
        _sock_lock（无锁序倒置：send_command 先取后放 _sock_lock 再取
        _send_lock；接收线程只在 _send_lock 内短暂阻塞）。
        """
        with self._send_lock:
            send_frame(sock, header, payload)

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
            # 第7轮 t7 修复（F1）：改经统一加锁出口，行为不变。
            self._send_frame_locked(client, header)
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
                # 第7轮 t7 修复（F1）：hello-ack 纳入 _send_lock 串行化。
                self._send_frame_locked(
                    sock,
                    {"type": "ack", "protocol": PROTOCOL, "timestamp": datetime.now().isoformat(timespec="seconds")},
                )
                # 第2轮 R-A-19/R-A-19b：hello 是 run 切换双触发之一（接收线程
                # 内串行执行；词汇表唯一判据 = classify_run_id）。
                self._on_hello_run_id(header.get("run_id"))
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
            elif isinstance(msg_type, str) and msg_type.startswith(_CALIBRATION_MESSAGE_PREFIX):
                # 第2轮 §5.4/R-C-17：校准回程按 type 分派到校准展示分支——
                # 绝不进 _accept_trace/_seen_traces/display FIFO，不占 trace 序列。
                self.calibration_message.emit(dict(header))
            elif msg_type == "command_ack":
                # command_ack 只表示停止/启动命令已被空中端接受，不等于采集
                # 已停止（第1轮检查点D）：最终状态由 acquisition_status 决定。
                self.command_status.emit(f"Remote accepted: {header.get('action')}")
                # 第7轮：完整回执（含 result/supported 结果字段）转发设备面板；
                # 旧空中端的纯 ack 同样转发，面板据此显示「对方版本不支持此功能」。
                self.command_result.emit(dict(header))
            elif msg_type == "heartbeat":
                # 链路空闲保活心跳：直接忽略无需处理，保持 TCP 连接活跃防止超时
                pass
            elif msg_type == "acquisition_status":
                # 第1轮检查点D：向后兼容状态消息（旧接收端会落入 else 分支忽略）。
                # R-A-19b：补传 run 的 stopped/drain_failed 到达即该 run 结束——
                # 在接收线程记汇总日志（主线程随后按 R-A-20 迟到护栏忽略）。
                if str(header.get("state") or "") in ("stopped", "drain_failed"):
                    run_key = safe_name(header.get("run_id"), "run")
                    if run_key == self._replay_run_id:
                        self._exit_replay_mode()
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
                # 第7轮 t7 修复（F1）：duplicate ack_trace 纳入 _send_lock。
                self._send_frame_locked(
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
                if run_key == self._replay_run_id:
                    # R-A-19b：补传重放被 duplicate 吸收也计数（重发≠丢数）。
                    self._replay_counts[run_key] = self._replay_counts.get(run_key, 0) + 1
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
        # 第7轮 t7 修复（F1）：ack_trace 纳入 _send_lock 串行化。
        self._send_frame_locked(
            sock,
            {
                "type": "ack_trace",
                "protocol": PROTOCOL,
                "run_id": header.get("run_id"),
                "trace_index": trace_index,
                "path": path.name,
            },
        )
        if run_key == self._replay_run_id:
            # R-A-19b：补传模式——已照常落盘+ACK（数据完整性优先，outbox 依赖
            # ACK 划除），但绝不进显示/对账基线：不 _note_emitted、不 emit。
            self._replay_counts[run_key] = self._replay_counts.get(run_key, 0) + 1
            return None
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
    """采集参数与雷达扫描、处理参数已提交状态。

    frozen：已应用实例不可被外部原地修改；对话框每次“应用”都生成新快照。
    """

    interval_s: float = 0.0
    count: int = 0
    start_mhz: float = 400.0
    stop_mhz: float = 4000.0
    points: int = 201
    ifbw_hz: int = 10000
    power_dbm: float = -20.0
    dwell_us: int = 0
    ifft_points: int = 2048
    time_window_ns: float = 40.0
    dewow_enabled: bool = False
    dewow_window_ns: float = 4.0
    #: 采集通道：dual（HH+VV）/ S11（只 HH）/ S22（只 VV）。
    channels: str = "dual"

    def command_params(self, single: bool = False) -> dict[str, object]:
        return {
            "count": 1 if single else int(self.count),
            "interval_s": float(self.interval_s),
            "single": single,
        }

    def sweep_params(self) -> dict[str, object]:
        return {
            "start_mhz": float(self.start_mhz),
            "stop_mhz": float(self.stop_mhz),
            "points": int(self.points),
            "ifbw_hz": int(self.ifbw_hz),
            "power_dbm": float(self.power_dbm),
            "dwell_us": int(self.dwell_us),
            "ifft_points": int(self.ifft_points),
            "time_window_ns": float(self.time_window_ns),
            "dewow_enabled": bool(self.dewow_enabled),
            "dewow_window_ns": float(self.dewow_window_ns),
            "channels": str(self.channels),
        }


GroundSettings = AcquisitionUiSettings
#: 采集通道选项（与空中端 CHANNEL_MODES 一致）。
CHANNEL_MODE_CHOICES = (
    ("dual", "双通道 HH (S11) + VV (S22)"),
    ("S11", "只采 HH (S11)"),
    ("S22", "只采 VV (S22)"),
)
CHANNEL_MODE_CHANNELS = {"dual": ("S11", "S22"), "S11": ("S11",), "S22": ("S22",)}


@dataclass(frozen=True)
class BscanUiSettings:
    """B-scan 显示参数已提交状态。

    frozen：已应用实例不可被外部原地修改；对话框每次“应用”都生成新快照。
    """

    colormap: str = "gray"
    aspect_ratio: float = 2.0
    flat_filter_enabled: bool = False
    flat_window_traces: int = 31


# 色阶（colorbar）显示范围滑块：百分比 = 显示范围占当前窗口 max|x| 的比例。
# 100% 与改动前行为完全一致（[-max_abs, +max_abs]）；调小则范围收窄、
# 弱反射颜色更深、强反射饱和。S11/S22 各自按自身 max_abs 缩放。
BSCAN_CONTRAST_MIN_PERCENT = 1
BSCAN_CONTRAST_MAX_PERCENT = 100
BSCAN_CONTRAST_DEFAULT_PERCENT = 100


def bscan_contrast_levels(max_abs: float, percent: float) -> tuple[float, float]:
    """按色阶百分比计算对称显示范围 ``(-v, +v)``，``v = max_abs * percent / 100``。

    ``percent`` 裁剪到 [BSCAN_CONTRAST_MIN_PERCENT, BSCAN_CONTRAST_MAX_PERCENT]；
    ``max_abs`` 非有限或非正时按 1.0 处理，结果恒为有限非零范围。
    """
    try:
        base = float(max_abs)
    except (TypeError, ValueError):
        base = 1.0
    if not math.isfinite(base) or base <= 0.0:
        base = 1.0
    try:
        pct = float(percent)
    except (TypeError, ValueError):
        pct = float(BSCAN_CONTRAST_DEFAULT_PERCENT)
    if not math.isfinite(pct):
        pct = float(BSCAN_CONTRAST_DEFAULT_PERCENT)
    pct = min(max(pct, float(BSCAN_CONTRAST_MIN_PERCENT)), float(BSCAN_CONTRAST_MAX_PERCENT))
    value = base * pct / 100.0
    return -value, value


class AcquisitionSettingsDialog(QtWidgets.QDialog):
    """地面端设置中心对话框：包含采集任务控制、VNA 扫描参数与信号处理参数页。"""

    def __init__(self, window: "GroundStationWindow") -> None:
        super().__init__(window)
        self._window = window
        self.setObjectName("acquisition_settings_dialog")
        self.setWindowTitle("设置中心")
        self.resize(520, 420)

        main_layout = QtWidgets.QVBoxLayout(self)

        self.tab_widget = QtWidgets.QTabWidget(self)
        self.tab_widget.setObjectName("settings_tab_widget")

        # ---- Tab 1: 采集控制 ----
        self.acq_page = QtWidgets.QWidget()
        self.acq_page.setObjectName("acquisition_page")
        form = QtWidgets.QFormLayout(self.acq_page)
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
        self.channels_input = QtWidgets.QComboBox()
        self.channels_input.setObjectName("acquisition_channels_input")
        for mode_key, mode_label in CHANNEL_MODE_CHOICES:
            self.channels_input.addItem(mode_label, userData=mode_key)
        self.channels_hint = QtWidgets.QLabel("只采一个通道时，另一幅 B-scan 显示“本次未采集”")
        self.channels_hint.setObjectName("acquisition_channels_hint")
        form.addRow("采集模式", self.mode)
        form.addRow("采集间隔 (s)", self.interval)
        form.addRow("采集数量", self.count)
        form.addRow("", self.count_hint)
        form.addRow("采集通道", self.channels_input)
        form.addRow("", self.channels_hint)
        self.tab_widget.addTab(self.acq_page, "采集控制")

        # ---- Tab 2: 扫描参数 (SweepConfig 对齐) ----
        self.sweep_page = QtWidgets.QWidget()
        self.sweep_page.setObjectName("sweep_page")
        sweep_form = QtWidgets.QFormLayout(self.sweep_page)
        self.start_mhz = QtWidgets.QDoubleSpinBox()
        self.start_mhz.setObjectName("start_mhz_input")
        self.start_mhz.setRange(0.001, 6000.0)
        self.start_mhz.setDecimals(3)
        self.start_mhz.setValue(400.0)
        self.start_mhz.setKeyboardTracking(False)
        self.stop_mhz = QtWidgets.QDoubleSpinBox()
        self.stop_mhz.setObjectName("stop_mhz_input")
        self.stop_mhz.setRange(0.001, 6000.0)
        self.stop_mhz.setDecimals(3)
        self.stop_mhz.setValue(4000.0)
        self.stop_mhz.setKeyboardTracking(False)
        self.points = QtWidgets.QSpinBox()
        self.points.setObjectName("points_input")
        self.points.setRange(2, 4501)
        self.points.setValue(201)
        self.ifbw = QtWidgets.QSpinBox()
        self.ifbw.setObjectName("ifbw_input")
        self.ifbw.setRange(6, 50000)
        self.ifbw.setValue(10000)
        self.power = QtWidgets.QDoubleSpinBox()
        self.power.setObjectName("power_input")
        self.power.setRange(-40.0, 0.0)
        self.power.setDecimals(1)
        self.power.setValue(-20.0)
        self.power.setKeyboardTracking(False)
        self.dwell = QtWidgets.QSpinBox()
        self.dwell.setObjectName("dwell_input")
        self.dwell.setRange(0, 10239)
        self.dwell.setValue(0)
        sweep_form.addRow("起始频率 (MHz)", self.start_mhz)
        sweep_form.addRow("终止频率 (MHz)", self.stop_mhz)
        sweep_form.addRow("频点数", self.points)
        sweep_form.addRow("中频带宽 (Hz)", self.ifbw)
        sweep_form.addRow("发射功率 (dBm)", self.power)
        sweep_form.addRow("驻留时间 (μs)", self.dwell)
        self.tab_widget.addTab(self.sweep_page, "扫描参数")

        # ---- Tab 3: 处理参数 (Dewow / 时间窗 对齐) ----
        self.processing_page = QtWidgets.QWidget()
        self.processing_page.setObjectName("processing_page")
        proc_form = QtWidgets.QFormLayout(self.processing_page)
        # 第6轮 t3 UX（captain 批准方案一）：设置中心此前无 B-scan 页，
        # 长宽比唯一入口在菜单"B-scan"→"B-scan 设置……"——用户在设置中心
        # 找不到长宽比（真机 bug2 入口困惑层）。在处理参数页顶部加一行
        # 说明 + 直达按钮，沿用既有 open_bscan_settings 打开逻辑；
        # 不动 3 页签结构与既有 *_input 控件。
        self.bscan_entry_hint = QtWidgets.QLabel(
            "长宽比 / 颜色表等 B-scan 显示选项在 B-scan 显示设置中"
        )
        self.bscan_entry_hint.setObjectName("bscan_entry_hint")
        self.bscan_entry_button = QtWidgets.QPushButton(
            "B-scan 显示设置（长宽比/颜色表）…"
        )
        self.bscan_entry_button.setObjectName("bscan_entry_button")
        self.bscan_entry_button.clicked.connect(self._open_bscan_settings_entry)
        proc_form.addRow(self.bscan_entry_hint)
        proc_form.addRow("", self.bscan_entry_button)
        self.dewow_enabled = QtWidgets.QCheckBox("启用 Dewow")
        self.dewow_enabled.setObjectName("dewow_enabled_input")
        self.dewow_enabled.setChecked(False)
        self.dewow_window = QtWidgets.QDoubleSpinBox()
        self.dewow_window.setObjectName("dewow_window_input")
        self.dewow_window.setRange(0.001, 1000.0)
        self.dewow_window.setDecimals(3)
        self.dewow_window.setValue(4.0)
        self.dewow_window.setEnabled(False)
        self.dewow_enabled.toggled.connect(self.dewow_window.setEnabled)
        self.time_window = QtWidgets.QDoubleSpinBox()
        self.time_window.setObjectName("time_window_input")
        self.time_window.setRange(1.0, 1000.0)
        self.time_window.setDecimals(1)
        self.time_window.setValue(40.0)
        self.ifft_points = QtWidgets.QSpinBox()
        self.ifft_points.setObjectName("ifft_points_input")
        self.ifft_points.setRange(256, 65536)
        self.ifft_points.setValue(2048)
        proc_form.addRow("", self.dewow_enabled)
        proc_form.addRow("Dewow 窗长 (ns)", self.dewow_window)
        proc_form.addRow("时间窗 (ns)", self.time_window)
        proc_form.addRow("IFFT 点数", self.ifft_points)
        self.tab_widget.addTab(self.processing_page, "处理参数")

        main_layout.addWidget(self.tab_widget)

        # 底部按钮区：下发参数、应用、取消
        buttons = QtWidgets.QDialogButtonBox()
        self.send_params_button = buttons.addButton(
            "下发参数", QtWidgets.QDialogButtonBox.ButtonRole.ActionRole
        )
        self.send_params_button.setObjectName("send_params_button")
        self.send_params_button.clicked.connect(self.send_params_to_air)

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
        main_layout.addWidget(buttons)

    def _sync_interval_enabled(self) -> None:
        self.interval.setEnabled(self.mode.currentIndex() == 1)

    def _open_bscan_settings_entry(self) -> None:
        """第6轮 t3 UX：设置中心直达 B-scan 显示设置（打开既有非模态对话框）。"""
        self._window.open_bscan_settings()

    def validate(self) -> str | None:
        if self.stop_mhz.value() <= self.start_mhz.value():
            return "终止频率必须大于起始频率"
        if self.points.value() < 2 or self.points.value() > 4501:
            return "频点数超出范围 [2, 4501]"
        if self.ifbw.value() < 6 or self.ifbw.value() > 50000:
            return "中频带宽超出范围 [6, 50000] Hz"
        if self.power.value() < -40.0 or self.power.value() > 0.0:
            return "发射功率超出范围 [-40, 0] dBm"
        if self.dwell.value() < 0 or self.dwell.value() > 10239:
            return "驻留时间超出范围 [0, 10239] us"
        if self.ifft_points.value() < 256 or self.ifft_points.value() > 65536:
            return "IFFT 点数超出范围 [256, 65536]"
        if self.time_window.value() < 1.0 or self.time_window.value() > 1000.0:
            return "时间窗超出范围 [1, 1000] ns"
        if self.dewow_window.value() <= 0.0 or self.dewow_window.value() > 1000.0:
            return "Dewow 窗长超出范围 (0, 1000] ns"
        return None

    def load_from(self, settings: AcquisitionUiSettings) -> None:
        if float(settings.interval_s) <= 0.0:
            self.mode.setCurrentIndex(0)
        else:
            self.mode.setCurrentIndex(1)
        self.interval.setValue(
            float(settings.interval_s) if settings.interval_s > 0.0 else 1.0
        )
        self.count.setValue(int(settings.count))
        self.start_mhz.setValue(float(settings.start_mhz))
        self.stop_mhz.setValue(float(settings.stop_mhz))
        self.points.setValue(int(settings.points))
        self.ifbw.setValue(int(settings.ifbw_hz))
        self.power.setValue(float(settings.power_dbm))
        self.dwell.setValue(int(settings.dwell_us))
        self.ifft_points.setValue(int(settings.ifft_points))
        self.time_window.setValue(float(settings.time_window_ns))
        self.dewow_enabled.setChecked(bool(settings.dewow_enabled))
        self.dewow_window.setValue(float(settings.dewow_window_ns))
        self.dewow_window.setEnabled(bool(settings.dewow_enabled))
        channel_index = self.channels_input.findData(str(getattr(settings, "channels", "dual")))
        self.channels_input.setCurrentIndex(channel_index if channel_index >= 0 else 0)
        self._sync_interval_enabled()

    def draft_settings(self) -> AcquisitionUiSettings:
        if self.mode.currentIndex() == 0:
            interval = 0.0
        else:
            interval = float(self.interval.value())
            if interval <= 0.0:
                interval = 0.0
        return AcquisitionUiSettings(
            interval_s=interval,
            count=int(self.count.value()),
            start_mhz=float(self.start_mhz.value()),
            stop_mhz=float(self.stop_mhz.value()),
            points=int(self.points.value()),
            ifbw_hz=int(self.ifbw.value()),
            power_dbm=float(self.power.value()),
            dwell_us=int(self.dwell.value()),
            ifft_points=int(self.ifft_points.value()),
            time_window_ns=float(self.time_window.value()),
            dewow_enabled=bool(self.dewow_enabled.isChecked()),
            dewow_window_ns=float(self.dewow_window.value()),
            channels=str(self.channels_input.currentData() or "dual"),
        )

    def send_params_to_air(self) -> bool:
        err = self.validate()
        if err is not None:
            self._window.log(f"参数验证失败: {err}")
            return False
        draft = self.draft_settings()
        self._window.apply_acquisition_settings(draft)
        self._window.send_set_params(draft.sweep_params())
        return True

    def apply(self) -> None:
        if not self.send_params_to_air():
            return
        self.accept()


GroundSettingsDialog = AcquisitionSettingsDialog


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


class AirDevicePanel(QtWidgets.QWidget):
    """第7轮 D1+D5：空中端设备远程面板（黑匣子全遥控，独立非模态窗口）。

    五项能力全部经既有 ``command`` 消息新增 action 实现（:data:`AIR_DEVICE_ACTIONS`，
    与 t1 黑匣子词表逐字一致）；回执走既有 ``command_ack`` 通道 + 新增
    ``result``/``supported`` 结果字段（既有字段零改动）。老空中端对未知
    action 只回纯 ack（无 result 字段）→ 面板明确显示「对方版本不支持此
    功能」，不崩不静默（§4.6 互操作红线）。

    主动作面板是独立顶层子窗口（Qt.Window），不进主界面中央布局——第1轮
    冻结契约「主界面按钮只有 开始/停止/暂停/继续」不受影响。
    """

    #: 状态总览默认刷新间隔（计划第九节：每 5 秒；构造可注入便于测试）。
    DEFAULT_REFRESH_INTERVAL_MS = 5000
    #: 互操作兜底文案（老空中端 / 旧版黑匣子未实现该 action 时显示）。
    UNSUPPORTED_TEXT = "对方版本不支持此功能"

    def __init__(
        self,
        window: "GroundStationWindow",
        *,
        refresh_interval_ms: int | None = None,
    ) -> None:
        super().__init__(window, QtCore.Qt.WindowType.Window)
        self.setObjectName("device_panel")
        self.setWindowTitle("空中端设备")
        self.resize(620, 720)
        self._window = window
        self._refresh_interval_ms = (
            self.DEFAULT_REFRESH_INTERVAL_MS
            if refresh_interval_ms is None
            else int(refresh_interval_ms)
        )
        #: 已发送待回执的 action → 发送时刻（monotonic 秒）。
        self._pending: dict[str, float] = {}
        #: 链路连接状态（由主窗口 _set_remote_connected 同步；未连接绝不发送）。
        self._link_connected = False
        #: 最近一次 get_status 回执（原样保留供测试与诊断）。
        self.last_status_result: dict[str, object] = {}
        self.last_status_at: str = ""
        #: 用户改过下拉但尚未应用的脏标记：自动同步不得覆盖未应用的选择。
        self._engine_dirty = False
        self._source_dirty = False
        self._build_ui()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(self._refresh_interval_ms)
        self._timer.timeout.connect(self.refresh_now)
        self._timer.start()

    # ------------------------------------------------------------------
    # UI 构建
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # ---- 0. 链路状态 ----
        self.device_link_label = QtWidgets.QLabel("连接状态：未连接")
        self.device_link_label.setObjectName("device_link_label")
        layout.addWidget(self.device_link_label)

        # ---- 1. 设备状态总览（在线/重连中、VNA USB、run_id、是否采集、
        #         积压、磁盘剩余、程序版本、当前配置）----
        overview_box = QtWidgets.QGroupBox("设备状态总览")
        overview_box.setObjectName("device_overview_box")
        form = QtWidgets.QFormLayout(overview_box)
        self.device_vna_label = self._form_row(form, "device_vna_label", "雷达 USB")
        self.device_run_id_label = self._form_row(form, "device_run_id_label", "run_id")
        self.device_acquiring_label = self._form_row(
            form, "device_acquiring_label", "是否采集"
        )
        self.device_backlog_label = self._form_row(
            form, "device_backlog_label", "积压"
        )
        self.device_disk_label = self._form_row(
            form, "device_disk_label", "磁盘剩余"
        )
        self.device_version_label = self._form_row(
            form, "device_version_label", "程序版本"
        )
        self.device_config_label = self._form_row(
            form, "device_config_label", "当前配置"
        )
        self.device_config_label.setWordWrap(True)
        self.device_status_updated_label = self._form_row(
            form, "device_status_updated_label", "总览更新"
        )
        layout.addWidget(overview_box)

        refresh_row = QtWidgets.QHBoxLayout()
        self.manual_refresh_button = QtWidgets.QPushButton("立即刷新")
        self.manual_refresh_button.setObjectName("device_refresh_button")
        self.manual_refresh_button.clicked.connect(self.refresh_now)
        refresh_row.addWidget(self.manual_refresh_button)
        refresh_row.addStretch(1)
        layout.addLayout(refresh_row)

        # ---- 2. 测试雷达（固件版本/协议号/最大频率/在线与否）----
        vna_box = QtWidgets.QGroupBox("测试雷达")
        vna_box.setObjectName("device_vna_box")
        vna_layout = QtWidgets.QVBoxLayout(vna_box)
        self.test_vna_button = QtWidgets.QPushButton("测试雷达")
        self.test_vna_button.setObjectName("test_vna_button")
        self.test_vna_button.clicked.connect(self.send_test_vna)
        self.test_vna_result_label = QtWidgets.QLabel("雷达：未测试")
        self.test_vna_result_label.setObjectName("test_vna_result_label")
        self.test_vna_result_label.setWordWrap(True)
        vna_layout.addWidget(self.test_vna_button)
        vna_layout.addWidget(self.test_vna_result_label)
        layout.addWidget(vna_box)

        # ---- 3. GNSS 串口管理（列出 → 选择+波特率 → 测试）----
        gnss_box = QtWidgets.QGroupBox("GNSS 串口管理")
        gnss_box.setObjectName("device_gnss_box")
        gnss_layout = QtWidgets.QVBoxLayout(gnss_box)
        ports_row = QtWidgets.QHBoxLayout()
        self.list_ports_button = QtWidgets.QPushButton("列出串口")
        self.list_ports_button.setObjectName("list_serial_ports_button")
        self.list_ports_button.clicked.connect(self.send_list_serial_ports)
        self.gnss_port_combo = QtWidgets.QComboBox()
        self.gnss_port_combo.setObjectName("gnss_port_combo")
        ports_row.addWidget(self.list_ports_button)
        ports_row.addWidget(self.gnss_port_combo, 1)
        gnss_layout.addLayout(ports_row)
        set_row = QtWidgets.QHBoxLayout()
        self.gnss_baud_spin = QtWidgets.QSpinBox()
        self.gnss_baud_spin.setObjectName("gnss_baud_spin")
        self.gnss_baud_spin.setRange(1200, 921600)
        self.gnss_baud_spin.setValue(115200)
        # 用户手改过就不再用空中端上报值覆盖；否则按空中端实际配置同步
        # （2026-10-09 现场：M100 在 115200 下丢低电平位，实际用的是 9600，
        #   面板必须显示真实值，否则一点"设置 GNSS 串口"就把 9600 改回 115200）。
        self._baud_dirty = False
        self._syncing_baud = False
        self.gnss_baud_spin.valueChanged.connect(self._on_baud_edited)
        self.set_gnss_button = QtWidgets.QPushButton("设置 GNSS 串口")
        self.set_gnss_button.setObjectName("set_gnss_port_button")
        self.set_gnss_button.clicked.connect(self.send_set_gnss_port)
        set_row.addWidget(self.gnss_baud_spin)
        set_row.addWidget(self.set_gnss_button)
        set_row.addStretch(1)
        gnss_layout.addLayout(set_row)
        test_row = QtWidgets.QHBoxLayout()
        self.test_gnss_button = QtWidgets.QPushButton("测试 GNSS")
        self.test_gnss_button.setObjectName("test_gnss_button")
        self.test_gnss_button.clicked.connect(self.send_test_gnss)
        test_row.addWidget(self.test_gnss_button)
        test_row.addStretch(1)
        gnss_layout.addLayout(test_row)
        self.gnss_result_label = QtWidgets.QLabel("GNSS：未设置")
        self.gnss_result_label.setObjectName("gnss_result_label")
        self.gnss_result_label.setWordWrap(True)
        gnss_layout.addWidget(self.gnss_result_label)
        layout.addWidget(gnss_box)

        # ---- 4. 采集引擎切换（continuous | legacy）----
        engine_box = QtWidgets.QGroupBox("采集引擎切换")
        engine_box.setObjectName("device_engine_box")
        engine_layout = QtWidgets.QHBoxLayout(engine_box)
        self.engine_combo = QtWidgets.QComboBox()
        self.engine_combo.setObjectName("engine_combo")
        self.engine_combo.addItem("高速连续 (continuous)", "continuous")
        self.engine_combo.addItem("兼容 (legacy)", "legacy")
        # activated 仅用户选择时触发：程序化同步不置脏标记。
        self.engine_combo.activated.connect(self._mark_engine_dirty)
        self.apply_engine_button = QtWidgets.QPushButton("应用引擎切换")
        self.apply_engine_button.setObjectName("apply_engine_button")
        self.apply_engine_button.clicked.connect(self.send_set_engine)
        engine_layout.addWidget(self.engine_combo, 1)
        engine_layout.addWidget(self.apply_engine_button)
        layout.addWidget(engine_box)

        # ---- 5. 模拟器切换（usb | simulated）----
        source_box = QtWidgets.QGroupBox("模拟器切换")
        source_box.setObjectName("device_source_box")
        source_layout = QtWidgets.QHBoxLayout(source_box)
        self.device_source_combo = QtWidgets.QComboBox()
        self.device_source_combo.setObjectName("panel_device_source_combo")
        self.device_source_combo.addItem("USB 真机 (usb)", "usb")
        self.device_source_combo.addItem("模拟器 (simulated)", "simulated")
        self.device_source_combo.activated.connect(self._mark_source_dirty)
        self.apply_source_button = QtWidgets.QPushButton("应用模拟器切换")
        self.apply_source_button.setObjectName("apply_device_source_button")
        self.apply_source_button.clicked.connect(self.send_set_device_source)
        source_layout.addWidget(self.device_source_combo, 1)
        source_layout.addWidget(self.apply_source_button)
        layout.addWidget(source_box)

        layout.addStretch(1)

        # ---- 回执 / 提示栏（回执确认与「不支持」显示的固定位置）----
        self.panel_notice_label = QtWidgets.QLabel("就绪")
        self.panel_notice_label.setObjectName("panel_notice_label")
        self.panel_notice_label.setWordWrap(True)
        layout.addWidget(self.panel_notice_label)

    @staticmethod
    def _form_row(
        form: QtWidgets.QFormLayout, object_name: str, title: str
    ) -> QtWidgets.QLabel:
        label = QtWidgets.QLabel(f"{title}：-")
        label.setObjectName(object_name)
        form.addRow(title, label)
        return label

    # ------------------------------------------------------------------
    # 刷新节奏（默认 5 秒，可注入）
    # ------------------------------------------------------------------

    @property
    def refresh_interval_ms(self) -> int:
        return self._refresh_interval_ms

    def set_refresh_interval_ms(self, ms: int) -> None:
        """注入刷新间隔；非法值（<=0）拒绝并保持原值（防误注入）。"""
        value = int(ms)
        if value <= 0:
            return
        self._refresh_interval_ms = value
        self._timer.setInterval(value)

    def refresh_now(self) -> bool:
        """立即请求一次状态总览（定时器与「立即刷新」按钮共用入口）。

        仅在链路已连接且面板可见时发送（面板隐藏/未连接绝不发送，避免往
        主界面刷「命令发送失败」噪音）；未连接时手动刷新给出明确提示，
        返回是否实际发送。
        """
        if not self._link_connected:
            self._set_notice("设备状态总览：未连接空中端，无法获取")
            return False
        if not self.isVisible():
            return False
        return self._send_action("get_status", {})

    # ------------------------------------------------------------------
    # 链路状态同步（主窗口 _set_remote_connected 调用）
    # ------------------------------------------------------------------

    def set_link_connected(self, connected: bool) -> None:
        self._link_connected = bool(connected)
        if connected:
            self.device_link_label.setText("连接状态：已连接")
            return
        self.device_link_label.setText("连接状态：未连接")
        self._pending.clear()
        self._engine_dirty = False
        self._source_dirty = False
        self._set_notice("与空中端连接已断开")
        self._set_overview_placeholder("未连接")

    # ------------------------------------------------------------------
    # 发送（全部经既有 command 通道 + AIR_DEVICE_ACTIONS 新 action）
    # ------------------------------------------------------------------

    def _send_action(
        self, action: str, params: dict[str, object] | None = None
    ) -> bool:
        receiver = getattr(self._window, "receiver", None)
        send = getattr(receiver, "send_command", None)
        if not callable(send) or not self._link_connected:
            self._set_notice(f"{self._action_label(action)}：未连接空中端，无法发送")
            return False
        self._pending[action] = time.monotonic()
        try:
            send(action, dict(params or {}))
        except Exception as exc:  # noqa: BLE001 - 发送失败必须可见，不得静默
            self._pending.pop(action, None)
            self._set_notice(f"{self._action_label(action)}：发送失败（{exc}）")
            return False
        self._set_notice(f"{self._action_label(action)}：已发送，等待空中端回执…")
        return True

    @staticmethod
    def _action_label(action: str) -> str:
        return AIR_DEVICE_ACTION_LABELS.get(action, action)

    def send_test_vna(self) -> None:
        self._send_action("test_vna")

    def send_list_serial_ports(self) -> None:
        self._send_action("list_serial_ports")

    def send_set_gnss_port(self) -> None:
        # 下拉项由 list_serial_ports 回执填充时 currentData=纯端口名；
        # 优先取 data，回退到显示文本（手工输入场景）。
        port = str(
            self.gnss_port_combo.currentData()
            or self.gnss_port_combo.currentText()
            or ""
        ).strip()
        if not port:
            self._set_notice("设置 GNSS 串口：请先「列出串口」并选择串口")
            return
        self._baud_dirty = False   # 本次按下即采用当前框内值，之后继续跟随空中端上报
        self._send_action(
            "set_gnss_port",
            {"port": port, "baud": int(self.gnss_baud_spin.value())},
        )

    def _on_baud_edited(self, _value: int) -> None:
        """用户手改波特率 → 标记 dirty，自动同步不再覆盖他的输入。"""
        if not self._syncing_baud:
            self._baud_dirty = True

    def send_test_gnss(self) -> None:
        self._send_action("test_gnss")

    def send_set_engine(self) -> None:
        engine = str(self.engine_combo.currentData() or "")
        self._send_action("set_engine", {"engine": engine})

    def send_set_device_source(self) -> None:
        source = str(self.device_source_combo.currentData() or "")
        self._send_action("set_device_source", {"device_source": source})

    def _mark_engine_dirty(self, _index: int) -> None:
        self._engine_dirty = True

    def _mark_source_dirty(self, _index: int) -> None:
        self._source_dirty = True

    # ------------------------------------------------------------------
    # 回执处理（既有 command_ack 通道 + result/supported 结果字段）
    # ------------------------------------------------------------------

    def on_command_result(self, header: dict) -> None:
        """主窗口转发的 command_ack 完整回执（仅处理设备面板词表内 action）。"""
        action = str(header.get("action") or "")
        if action not in AIR_DEVICE_ACTIONS:
            return  # 既有命令（start/stop/…）的 ack 与设备面板无关
        label = self._action_label(action)
        was_pending = self._pending.pop(action, None) is not None
        result = header.get("result")
        if header.get("supported") is False or not isinstance(result, dict):
            # 互操作兜底：老空中端/旧版黑匣子对该 action 无结果字段 →
            # 明确显示「不支持」，不崩不静默（§4.6 红线 5）。
            self._mark_unsupported(action)
            return
        assert isinstance(result, dict)
        ok_value = result.get("ok")
        error_text = result.get("error")
        if ok_value is False or (
            ok_value is None and isinstance(error_text, str) and bool(error_text)
        ):
            self._set_notice(f"{label}：空中端回执失败（{error_text or '未知错误'}）")
            self._log(f"{label}：空中端回执失败（{error_text or '未知错误'}）")
            return
        if action == "get_status":
            self._apply_status(result, was_pending=was_pending)
        elif action == "test_vna":
            self._apply_vna(result)
        elif action == "list_serial_ports":
            self._apply_ports(result)
        elif action == "set_gnss_port":
            self._apply_set_gnss(result)
        elif action == "test_gnss":
            self._apply_test_gnss(result)
        elif action == "set_engine":
            self._apply_set_engine(result)
        elif action == "set_device_source":
            self._apply_set_source(result)

    def _mark_unsupported(self, action: str) -> None:
        label = self._action_label(action)
        self._set_notice(f"{label}：{self.UNSUPPORTED_TEXT}")
        self._log(f"{label}：{self.UNSUPPORTED_TEXT}")
        if action == "get_status":
            self._set_overview_placeholder("对方不支持")
        elif action == "test_vna":
            self.test_vna_result_label.setText(f"雷达：{self.UNSUPPORTED_TEXT}")
        elif action == "test_gnss":
            self.gnss_result_label.setText(f"GNSS 测试：{self.UNSUPPORTED_TEXT}")

    # ---- 各 action 结果应用 ----

    def _apply_status(self, result: dict[str, object], *, was_pending: bool) -> None:
        self.last_status_result = dict(result)
        self.last_status_at = datetime.now().strftime("%H:%M:%S")
        if bool(result.get("reconnecting")):
            link_text = "连接状态：已连接（对方重连中）"
        elif result.get("online") is True:
            link_text = "连接状态：已连接（空中端在线）"
        elif result.get("online") is False:
            link_text = "连接状态：已连接（对方未就绪）"
        else:
            link_text = "连接状态：已连接"
        self.device_link_label.setText(link_text)
        self.device_vna_label.setText(
            "雷达 USB：" + self._tri_text(result.get("vna_usb"), "已连接", "未连接")
        )
        self.device_run_id_label.setText(f"run_id：{result.get('run_id') or '-'}")
        self.device_acquiring_label.setText(
            "是否采集：" + self._tri_text(result.get("acquiring"), "采集中", "未采集")
        )
        backlog = result.get("backlog")
        self.device_backlog_label.setText(
            "积压：未知" if backlog is None else f"积压：{backlog} 条"
        )
        disk = result.get("disk_free_gb")
        self.device_disk_label.setText(
            "磁盘剩余：未知"
            if not isinstance(disk, (int, float)) or isinstance(disk, bool)
            else f"磁盘剩余：{float(disk):.1f} GB"
        )
        self.device_version_label.setText(f"程序版本：{result.get('version') or '-'}")
        config = result.get("config") if isinstance(result.get("config"), dict) else {}
        self.device_config_label.setText("当前配置：" + self._config_text(config))
        self.device_status_updated_label.setText(f"总览更新：{self.last_status_at}")
        self._sync_combos_from_config(config)
        # 自动刷新（非本次请求触发）不覆盖「等待回执」类提示。
        if was_pending:
            self._set_notice("设备状态总览：已更新（回执正常）")

    def _apply_vna(self, result: dict[str, object]) -> None:
        online = self._tri_text(result.get("online"), "在线", "离线")
        firmware = str(result.get("firmware") or "-")
        protocol = result.get("protocol")
        protocol_text = "-" if protocol is None else str(protocol)
        max_freq = result.get("max_freq_hz")
        if isinstance(max_freq, (int, float)) and not isinstance(max_freq, bool) and max_freq > 0:
            max_text = f"{float(max_freq) / 1e9:.2f} GHz"
        else:
            max_text = "-"
        self.test_vna_result_label.setText(
            f"雷达：{online}｜固件 {firmware}｜协议 {protocol_text}｜最大频率 {max_text}"
        )
        self._set_notice("测试雷达：已收到回执")

    def _apply_ports(self, result: dict[str, object]) -> None:
        ports = result.get("ports")
        items: list[tuple[str, str]] = []
        if isinstance(ports, list):
            for entry in ports:
                if isinstance(entry, dict) and entry.get("port"):
                    items.append(
                        (str(entry["port"]), str(entry.get("description") or ""))
                    )
                elif isinstance(entry, str) and entry.strip():
                    items.append((entry.strip(), ""))
        self.gnss_port_combo.clear()
        if items:
            for port, description in items:
                text = f"{port}（{description}）" if description else port
                self.gnss_port_combo.addItem(text, port)
            self._set_notice(f"列出串口：发现 {len(items)} 个，请在下拉中选择")
        else:
            self._set_notice("列出串口：空中端未发现可用串口")

    def _apply_set_gnss(self, result: dict[str, object]) -> None:
        port = str(result.get("port") or "-")
        baud = result.get("baud")
        baud_text = str(baud) if baud is not None else "-"
        self.gnss_result_label.setText(
            f"GNSS 串口：已设置 {port} @ {baud_text}（空中端已确认）"
        )
        self._set_notice(
            f"回执确认：GNSS 串口已设置（{port} @ {baud_text}），空中端已回写配置"
        )

    def _apply_test_gnss(self, result: dict[str, object]) -> None:
        self.gnss_result_label.setText(self._window.gnss_map.status.text())
        self._set_notice("测试 GNSS：已收到回执")

    def _apply_set_engine(self, result: dict[str, object]) -> None:
        engine = str(result.get("engine") or "")
        name = {"continuous": "高速连续", "legacy": "兼容"}.get(engine, engine or "-")
        self._engine_dirty = False
        self._set_notice(
            f"回执确认：采集引擎已切换为 {name}（{engine or '-'}），空中端已回写配置"
        )

    def _apply_set_source(self, result: dict[str, object]) -> None:
        source = str(result.get("device_source") or "")
        name = {"usb": "USB 真机", "simulated": "模拟器"}.get(source, source or "-")
        self._source_dirty = False
        self._set_notice(
            f"回执确认：设备来源已切换为 {name}（{source or '-'}），空中端已回写配置"
        )

    # ---- 展示辅助 ----

    def _sync_combos_from_config(self, config: dict[str, object]) -> None:
        """把空中端上报的当前配置同步进下拉（未应用的本地编辑不被覆盖）。"""
        baud = config.get("gnss_baud")
        if (isinstance(baud, int) and not isinstance(baud, bool)
                and not self._baud_dirty and not self._syncing_baud
                and self.gnss_baud_spin.minimum() <= baud <= self.gnss_baud_spin.maximum()):
            self._syncing_baud = True
            try:
                self.gnss_baud_spin.setValue(baud)
            finally:
                self._syncing_baud = False
        engine = config.get("engine")
        if engine in AIR_ENGINE_VALUES and not self._engine_dirty:
            index = self.engine_combo.findData(engine)
            if index >= 0:
                self.engine_combo.setCurrentIndex(index)
        source = config.get("device_source")
        if source in AIR_DEVICE_SOURCE_VALUES and not self._source_dirty:
            index = self.device_source_combo.findData(source)
            if index >= 0:
                self.device_source_combo.setCurrentIndex(index)

    def _set_overview_placeholder(self, reason: str) -> None:
        self.device_vna_label.setText(f"雷达 USB：{reason}")
        self.device_run_id_label.setText(f"run_id：{reason}")
        self.device_acquiring_label.setText(f"是否采集：{reason}")
        self.device_backlog_label.setText(f"积压：{reason}")
        self.device_disk_label.setText(f"磁盘剩余：{reason}")
        self.device_version_label.setText(f"程序版本：{reason}")
        self.device_config_label.setText(f"当前配置：{reason}")
        self.device_status_updated_label.setText("总览更新：从未")

    def _set_notice(self, text: str) -> None:
        self.panel_notice_label.setText(text)

    def _log(self, text: str) -> None:
        log = getattr(self._window, "log", None)
        if callable(log):
            try:
                log(f"[设备面板] {text}")
            except Exception:  # noqa: BLE001 - 日志失败不得影响面板
                pass

    @staticmethod
    def _tri_text(value: object, true_text: str, false_text: str) -> str:
        if value is True:
            return true_text
        if value is False:
            return false_text
        return "未知"

    @staticmethod
    def _config_text(config: object) -> str:
        if not isinstance(config, dict) or not config:
            return "-"
        known_order = (
            "engine",
            "device_source",
            "gnss_port",
            "gnss_baud",
            "ground_host",
            "output_dir",
        )
        parts: list[str] = []
        for key in known_order:
            if key in config:
                parts.append(f"{key}={config[key]}")
        for key in sorted(config):
            if key not in known_order:
                parts.append(f"{key}={config[key]}")
        return "，".join(parts)


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


_STEP_INSTRUCTIONS: dict[str, str] = {
    "P1_Open": "步骤 1/6 [P1_Open]: 请在天线端口 1 (Port 1) 安装开路器 (Open)，端口 2 保持连接匹配负载 (Load)。",
    "P1_Short": "步骤 2/6 [P1_Short]: 请在天线端口 1 (Port 1) 安装短路器 (Short)，端口 2 保持连接匹配负载 (Load)。",
    "P1_Load": "步骤 3/6 [P1_Load]: 请在天线端口 1 (Port 1) 安装匹配负载 (Load)，端口 2 保持连接匹配负载 (Load)。",
    "P2_Open": "步骤 4/6 [P2_Open]: 请在天线端口 2 (Port 2) 安装开路器 (Open)，端口 1 保持连接匹配负载 (Load)。",
    "P2_Short": "步骤 5/6 [P2_Short]: 请在天线端口 2 (Port 2) 安装短路器 (Short)，端口 1 保持连接匹配负载 (Load)。",
    "P2_Load": "步骤 6/6 [P2_Load]: 请在天线端口 2 (Port 2) 安装匹配负载 (Load)，端口 1 保持连接匹配负载 (Load)。",
    "Background": "空采背景参考 [Background]: 请将雷达天线对准净空（远离地面及障碍物 1.5 米以上）测量直达波与天线串扰。",
}

_STEP_GUIDES: dict[str, str] = {
    "P1_Open": "P1: Open / P2: Load",
    "P1_Short": "P1: Short / P2: Load",
    "P1_Load": "P1: Load / P2: Load",
    "P2_Open": "P2: Open / P1: Load",
    "P2_Short": "P2: Short / P1: Load",
    "P2_Load": "P2: Load / P1: Load",
    "Background": "天线净空对空 (空采)",
}


class CalibrationWizard(QtWidgets.QDialog):
    """地面分步校准与空采向导对话框（参考 rebar-inspector reference_wizard）。

    提供分步状态机、标准件安装提示、采集进度指示、失败重采、以及收齐后解算/激活控制。
    复用第2轮 cal_* 命令链与 CalibrationSession（服务层零改动）。
    采用垂直步骤列表 + 底部清晰分排操作栏，杜绝控件重叠。
    """

    #: OSL 六步顺序（§5.2 编排流程；与空中端 CALIBRATION 步骤一致）。
    OSL_STEPS = ("P1_Open", "P1_Short", "P1_Load", "P2_Open", "P2_Short", "P2_Load")

    def __init__(self, window: "GroundStationWindow") -> None:
        super().__init__(window)
        self._window = window
        self.setObjectName("calibration_dialog")
        self.setWindowTitle("分步校准向导（地面发起 / 空中执行）")
        self.setModal(False)
        self.setMinimumSize(720, 580)
        self.resize(780, 640)

        self._connected = False
        self._cal_id: str | None = None
        self._mode: str = "osl"
        self._steps: list[str] = []
        self._step_index = 0
        self._steps_done = 0
        self._completed_steps: set[str] = set()
        self._solved = False
        self._capturing = False

        # 第8轮：会话与生效校准解耦、文件完整内容导出、能力与幂等
        self._active_calibration: dict | None = None
        self._active_background: dict | None = None
        self._last_osl_json_content: str | None = None
        self._pending_step: str | None = None
        self._pending_is_recapture = False
        self._pending_request_id: str | None = None
        self._completed_request_ids: set[str] = set()
        self._air_supports_step_done: bool | None = None
        self._pending_export_path: str | None = None
        self._pending_export_paths: dict[str, str | None] = {}
        self._pending_import_content: str | None = None
        self._pending_import_filename: str | None = None
        self._import_pending_cal_id: str | None = None
        self._watchdog_query_token: str | None = None
        self._watchdog_query_pending: bool = False
        self._watchdog_query_context: tuple[str | None, str | None, str | None] | None = None
        self._legacy_fallback_count: int = 0

        self._capture_watchdog_timer = QtCore.QTimer(self)
        self._capture_watchdog_timer.setSingleShot(True)
        self._capture_watchdog_timer.timeout.connect(self._on_capture_watchdog_timeout)

        self._import_timer = QtCore.QTimer(self)
        self._import_timer.setSingleShot(True)
        self._import_timer.timeout.connect(self._on_import_timeout)

        self._legacy_fallback_timer = QtCore.QTimer(self)
        self._legacy_fallback_timer.setSingleShot(True)
        self._legacy_fallback_timer.timeout.connect(self._on_legacy_fallback_timeout)
        self._legacy_fallback_step: str | None = None

        self._build_ui()
        self._set_session_controls()

    def _build_ui(self) -> None:
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(8)

        # ---- 1. 校准计划配置区 ----
        plan_group = QtWidgets.QGroupBox("校准计划与扫描参数")
        plan_group.setObjectName("calibration_plan_group")
        plan_grid = QtWidgets.QGridLayout(plan_group)
        plan_grid.setSpacing(6)

        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.setObjectName("calibration_mode_input")
        self.mode_combo.addItem("OSL（六步双反射）", userData="osl")
        self.mode_combo.addItem("空采参考（背景）", userData="background")
        # 单通道采集用：只校一个端口（3 步），另一端口不用接标准件。
        self.mode_combo.addItem("OSL（只校 P1 / HH，三步）", userData="osl_p1")
        self.mode_combo.addItem("OSL（只校 P2 / VV，三步）", userData="osl_p2")
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)

        self.repeats = QtWidgets.QSpinBox()
        self.repeats.setObjectName("calibration_repeats_input")
        self.repeats.setRange(1, 256)
        self.repeats.setValue(16)

        self.cal_start = QtWidgets.QDoubleSpinBox()
        self.cal_start.setObjectName("calibration_start_mhz_input")
        self.cal_start.setRange(0.001, 6000.0)
        self.cal_start.setDecimals(3)
        self.cal_start.setValue(400.0)

        self.cal_stop = QtWidgets.QDoubleSpinBox()
        self.cal_stop.setObjectName("calibration_stop_mhz_input")
        self.cal_stop.setRange(0.001, 6000.0)
        self.cal_stop.setDecimals(3)
        self.cal_stop.setValue(4000.0)

        self.cal_points = QtWidgets.QSpinBox()
        self.cal_points.setObjectName("calibration_points_input")
        self.cal_points.setRange(2, 4501)
        self.cal_points.setValue(201)

        self.cal_ifbw = QtWidgets.QSpinBox()
        self.cal_ifbw.setObjectName("calibration_ifbw_input")
        self.cal_ifbw.setRange(6, 50000)
        self.cal_ifbw.setValue(10000)

        self.cal_power = QtWidgets.QDoubleSpinBox()
        self.cal_power.setObjectName("calibration_power_input")
        self.cal_power.setRange(-40.0, 0.0)
        self.cal_power.setDecimals(1)
        self.cal_power.setValue(-20.0)

        self.cal_dwell = QtWidgets.QSpinBox()
        self.cal_dwell.setObjectName("calibration_dwell_input")
        self.cal_dwell.setRange(0, 10239)
        self.cal_dwell.setValue(0)

        plan_grid.addWidget(QtWidgets.QLabel("校准模式:"), 0, 0)
        plan_grid.addWidget(self.mode_combo, 0, 1)
        plan_grid.addWidget(QtWidgets.QLabel("每步重复采样:"), 0, 2)
        plan_grid.addWidget(self.repeats, 0, 3)

        plan_grid.addWidget(QtWidgets.QLabel("起始频率 (MHz):"), 1, 0)
        plan_grid.addWidget(self.cal_start, 1, 1)
        plan_grid.addWidget(QtWidgets.QLabel("终止频率 (MHz):"), 1, 2)
        plan_grid.addWidget(self.cal_stop, 1, 3)

        plan_grid.addWidget(QtWidgets.QLabel("频点数:"), 2, 0)
        plan_grid.addWidget(self.cal_points, 2, 1)
        plan_grid.addWidget(QtWidgets.QLabel("中频带宽 (Hz):"), 2, 2)
        plan_grid.addWidget(self.cal_ifbw, 2, 3)

        plan_grid.addWidget(QtWidgets.QLabel("发射功率 (dBm):"), 3, 0)
        plan_grid.addWidget(self.cal_power, 3, 1)
        plan_grid.addWidget(QtWidgets.QLabel("驻留时间 (us):"), 3, 2)
        plan_grid.addWidget(self.cal_dwell, 3, 3)

        main_layout.addWidget(plan_group)

        # ---- 第8轮：校准文件加载与导出操作栏 ----
        file_bar = QtWidgets.QHBoxLayout()
        file_bar.setObjectName("calibration_file_bar")
        file_bar.setSpacing(8)

        self.import_button = QtWidgets.QPushButton("导入校准文件")
        self.import_button.setObjectName("calibration_import_button")
        self.import_button.setMinimumHeight(30)
        self.import_button.clicked.connect(lambda: self.import_calibration_file())

        self.export_button = QtWidgets.QPushButton("导出校准文件")
        self.export_button.setObjectName("calibration_export_button")
        self.export_button.setMinimumHeight(30)
        self.export_button.clicked.connect(lambda: self.export_calibration_file())

        self.query_button = QtWidgets.QPushButton("查询生效状态")
        self.query_button.setObjectName("calibration_query_button")
        self.query_button.setMinimumHeight(30)
        self.query_button.clicked.connect(self.query_active_calibration)

        self.active_cal_label = QtWidgets.QLabel("当前生效: OSL[未启用] | 背景[未启用]")
        self.active_cal_label.setObjectName("calibration_active_label")
        self.active_cal_label.setStyleSheet("QLabel { font-weight: bold; color: #0d47a1; }")

        file_bar.addWidget(self.import_button)
        file_bar.addWidget(self.export_button)
        file_bar.addWidget(self.query_button)
        file_bar.addWidget(self.active_cal_label, 1)

        main_layout.addLayout(file_bar)

        # ---- 2. 分步采集表格与进度区 ----
        steps_group = QtWidgets.QGroupBox("分步采集状态与进度")
        steps_group.setObjectName("calibration_steps_group")
        steps_layout = QtWidgets.QVBoxLayout(steps_group)
        steps_layout.setSpacing(6)

        self.steps_table = QtWidgets.QTableWidget(0, 4)
        self.steps_table.setObjectName("calibration_steps_table")
        self.steps_table.setHorizontalHeaderLabels(["步骤", "状态", "安装指引", "质量 / 备注"])
        self.steps_table.horizontalHeader().setStretchLastSection(True)
        self.steps_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.steps_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.steps_table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.steps_table.verticalHeader().setVisible(False)
        self.steps_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.steps_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.steps_table.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)
        self.steps_table.setFixedHeight(150)
        steps_layout.addWidget(self.steps_table)

        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setObjectName("calibration_progress_bar")
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setFormat("总进度: %p%")
        steps_layout.addWidget(self.progress_bar)

        main_layout.addWidget(steps_group)

        # ---- 3. 当前步骤指引与状态反馈区 ----
        instruction_box = QtWidgets.QGroupBox("当前步骤操作指引")
        instruction_box.setObjectName("calibration_instruction_box")
        instruction_layout = QtWidgets.QVBoxLayout(instruction_box)
        instruction_layout.setSpacing(4)

        self.instruction_label = QtWidgets.QLabel("请配置校准计划后点击“开始校准”启动向导。")
        self.instruction_label.setObjectName("calibration_instruction_label")
        self.instruction_label.setWordWrap(True)
        self.instruction_label.setStyleSheet("QLabel { font-weight: bold; font-size: 13px; color: #1a73e8; }")
        instruction_layout.addWidget(self.instruction_label)

        self.warning_label = QtWidgets.QLabel("⚠ 提示：更换物理标准件后请务必确认接头紧固连接，再点击“采集下一步”。")
        self.warning_label.setObjectName("calibration_warning_label")
        self.warning_label.setWordWrap(True)
        self.warning_label.setStyleSheet("QLabel { color: #d93025; font-size: 11px; }")
        instruction_layout.addWidget(self.warning_label)

        self.status_label = QtWidgets.QLabel("未连接空中端")
        self.status_label.setObjectName("calibration_status_label")
        self.status_label.setWordWrap(True)
        instruction_layout.addWidget(self.status_label)

        self.quality_label = QtWidgets.QLabel("质量指标：-")
        self.quality_label.setObjectName("calibration_quality_label")
        self.quality_label.setWordWrap(True)
        instruction_layout.addWidget(self.quality_label)

        self.result_label = QtWidgets.QLabel("结果摘要：-")
        self.result_label.setObjectName("calibration_result_label")
        self.result_label.setWordWrap(True)
        instruction_layout.addWidget(self.result_label)

        main_layout.addWidget(instruction_box)

        # ---- 4. 底部操作栏（杜绝按钮重叠：分排水平流式布局）----
        bar1 = QtWidgets.QHBoxLayout()
        bar1.setSpacing(8)

        self.begin_button = QtWidgets.QPushButton("开始校准")
        self.begin_button.setObjectName("calibration_begin_button")
        self.begin_button.setMinimumHeight(32)
        self.begin_button.clicked.connect(self.begin_calibration)

        self.capture_button = QtWidgets.QPushButton("采集下一步")
        self.capture_button.setObjectName("calibration_capture_button")
        self.capture_button.setMinimumHeight(32)
        self.capture_button.clicked.connect(self.capture_next_step)
        self.capture_button.setEnabled(False)

        recapture_label = QtWidgets.QLabel("重采目标:")
        recapture_label.setObjectName("calibration_recapture_label")

        self.recapture_combo = QtWidgets.QComboBox()
        self.recapture_combo.setObjectName("calibration_recapture_combo")
        self.recapture_combo.setMinimumHeight(30)
        self.recapture_combo.setMinimumWidth(100)

        self.recapture_button = QtWidgets.QPushButton("重采该步骤")
        self.recapture_button.setObjectName("calibration_recapture_button")
        self.recapture_button.setMinimumHeight(32)
        self.recapture_button.clicked.connect(self.recapture_selected_step)
        self.recapture_button.setEnabled(False)

        bar1.addWidget(self.begin_button)
        bar1.addWidget(self.capture_button)
        bar1.addWidget(recapture_label)
        bar1.addWidget(self.recapture_combo)
        bar1.addWidget(self.recapture_button)
        bar1.addStretch(1)
        main_layout.addLayout(bar1)

        bar2 = QtWidgets.QHBoxLayout()
        bar2.setSpacing(8)

        self.finish_button = QtWidgets.QPushButton("解算并生成")
        self.finish_button.setObjectName("calibration_finish_button")
        self.finish_button.setMinimumHeight(32)
        self.finish_button.clicked.connect(self.finish_calibration)
        self.finish_button.setEnabled(False)

        self.activate_button = QtWidgets.QPushButton("激活校准")
        self.activate_button.setObjectName("calibration_activate_button")
        self.activate_button.setMinimumHeight(32)
        self.activate_button.clicked.connect(self.activate_calibration)
        self.activate_button.setEnabled(False)

        self.cancel_button = QtWidgets.QPushButton("取消会话")
        self.cancel_button.setObjectName("calibration_cancel_button")
        self.cancel_button.setMinimumHeight(32)
        self.cancel_button.clicked.connect(self.cancel_session)
        self.cancel_button.setEnabled(False)

        self.close_button = QtWidgets.QPushButton("关闭")
        self.close_button.setObjectName("calibration_close_button")
        self.close_button.setMinimumHeight(32)
        self.close_button.clicked.connect(self.close)

        bar2.addWidget(self.finish_button)
        bar2.addWidget(self.activate_button)
        bar2.addWidget(self.cancel_button)
        bar2.addStretch(1)
        bar2.addWidget(self.close_button)
        main_layout.addLayout(bar2)

    def _on_mode_changed(self) -> None:
        mode = str(self.mode_combo.currentData() or "osl")
        if mode == "background":
            self.instruction_label.setText("空采背景参考模式：请将天线对准净空后点击“开始校准”。")
        elif mode in ("osl_p1", "osl_p2"):
            port = "P1（HH）" if mode == "osl_p1" else "P2（VV）"
            self.instruction_label.setText(f"单端口 OSL 模式：只校 {port}，共三步，请配置计划后点击“开始校准”。")
        else:
            self.instruction_label.setText("六步双反射 OSL 模式：请配置计划后点击“开始校准”。")

    def _get_step_guide(self, step: str) -> str:
        return _STEP_GUIDES.get(step, "按向导提示操作")

    def _get_step_instruction(self, step: str) -> str:
        return _STEP_INSTRUCTIONS.get(step, f"请准备并采集步骤 {step}")

    # ---- 连接状态（R-C-11）----

    def set_connected(self, connected: bool) -> None:
        self._connected = bool(connected)
        if not self._connected:
            self._stop_capture_watchdog()
            self._cancel_legacy_fallback()
            if self._capturing:
                self._capturing = False
                if self._pending_step:
                    self._revert_step_row(self._pending_step, "空中端断开连接")
                self._revert_all_pending_steps("空中端断开连接")
            self.status_label.setText("未连接空中端（开始校准已禁用）")
        else:
            if self._cal_id is None:
                self.status_label.setText("已连接空中端：请配置计划后点击\"开始校准\"")
            else:
                self.status_label.setText(f"已连接空中端（会话 cal_id={self._cal_id}）")
        self._set_session_controls()

    def _set_session_controls(self) -> None:
        session_active = self._cal_id is not None
        self.begin_button.setEnabled(self._connected and not session_active and not self._capturing)
        self.capture_button.setEnabled(
            self._connected and session_active and self._step_index < len(self._steps) and not self._capturing
        )
        self.recapture_button.setEnabled(
            self._connected and session_active and self.recapture_combo.count() > 0 and not self._capturing
        )
        self.finish_button.setEnabled(
            self._connected and session_active and self._steps_done >= len(self._steps) and not self._solved and not self._capturing
        )
        self.activate_button.setEnabled(self._connected and session_active and self._solved and not self._capturing)
        self.cancel_button.setEnabled(self._connected and session_active and not self._capturing)

        self.import_button.setEnabled(self._connected and not self._capturing)
        self.export_button.setEnabled(self._connected and not self._capturing)
        self.query_button.setEnabled(self._connected and not self._capturing)

        for widget in (
            self.mode_combo,
            self.repeats,
            self.cal_start,
            self.cal_stop,
            self.cal_points,
            self.cal_ifbw,
            self.cal_power,
            self.cal_dwell,
        ):
            widget.setEnabled(self._connected and not session_active and not self._capturing)

    def _refresh_step_highlight(self) -> None:
        total = len(self._steps)
        if total > 0:
            pct = int((self._steps_done / total) * 100)
            self.progress_bar.setValue(pct)
            self.progress_bar.setFormat(f"总进度: {self._steps_done}/{total} ({pct}%)")

        current_step = self._steps[self._step_index] if self._step_index < total else None
        if current_step:
            self.instruction_label.setText(self._get_step_instruction(current_step))
        elif self._steps_done >= total and total > 0:
            self.instruction_label.setText("所有步骤已采集完毕！请点击“解算并生成”完成校准解算。")

        self.recapture_combo.clear()
        for step_name in self._steps:
            if step_name in self._completed_steps:
                self.recapture_combo.addItem(step_name, step_name)

        if current_step:
            for row in range(self.steps_table.rowCount()):
                if self.steps_table.item(row, 0) and self.steps_table.item(row, 0).text() == current_step:
                    self.steps_table.selectRow(row)
                    break

    def current_step_name(self) -> str | None:
        """返回当前待采步骤名称（供测试与调度使用）。"""
        if 0 <= self._step_index < len(self._steps):
            return self._steps[self._step_index]
        return None

    # ---- 编排命令（既有 type="command" + 新 action，R-C-12）----

    def _send(self, action: str, params: dict[str, object]) -> bool:
        receiver = getattr(self._window, "receiver", None)
        send = getattr(receiver, "send_command", None)
        if not callable(send):
            self.status_label.setText("未连接空中端（无法发送校准命令）")
            return False
        send(action, params)
        return True

    # ---- Watchdog 超时防抖与旧端平滑降级 ----

    def _reset_watchdog_query(self) -> None:
        """Retire replies belonging to a completed/cancelled capture."""
        self._watchdog_query_token = None
        self._watchdog_query_pending = False
        self._watchdog_query_context = None

    def _start_capture_watchdog(self) -> None:
        self._reset_watchdog_query()
        repeats = int(self.repeats.value())
        pts = int(self.cal_points.value())
        ifbw = int(self.cal_ifbw.value())
        t_sweep = pts / max(ifbw, 100) + 0.05
        total_sec = repeats * t_sweep + 20.0
        timeout_ms = int(max(30.0, total_sec * 2.0) * 1000)
        self._capture_watchdog_timer.start(timeout_ms)

    def _pet_capture_watchdog(self) -> None:
        if self._capture_watchdog_timer.isActive():
            repeats = int(self.repeats.value())
            pts = int(self.cal_points.value())
            ifbw = int(self.cal_ifbw.value())
            t_sweep = pts / max(ifbw, 100) + 0.05
            total_sec = repeats * t_sweep + 20.0
            timeout_ms = int(max(30.0, total_sec * 2.0) * 1000)
            self._capture_watchdog_timer.start(timeout_ms)

    def _stop_capture_watchdog(self) -> None:
        if self._capture_watchdog_timer.isActive():
            self._capture_watchdog_timer.stop()
        self._reset_watchdog_query()

    def _on_capture_watchdog_timeout(self) -> None:
        if not self._capturing or self._cal_id is None or not self._pending_request_id:
            self._reset_watchdog_query()
            return
        self.status_label.setText("等待空中端响应超时，正在查询远程实际状态…")
        req_id = f"req_wd_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"
        self._watchdog_query_token = req_id
        self._watchdog_query_pending = True
        self._watchdog_query_context = (
            self._cal_id, self._pending_request_id, self._pending_step
        )
        self._send("cal_query_active", {"request_id": req_id})

    def _schedule_legacy_fallback(self, step: str) -> None:
        self._cancel_legacy_fallback()
        self._legacy_fallback_step = step
        if self._air_supports_step_done is False:
            self._complete_step(step)
        # None is unknown, not an old peer. True owes a real step_done.
        # Elapsed time is never evidence that either peer finished its worker.

    def _cancel_legacy_fallback(self) -> None:
        if self._legacy_fallback_timer.isActive():
            self._legacy_fallback_timer.stop()
        self._legacy_fallback_step = None

    def _on_legacy_fallback_timeout(self) -> None:
        # A queued timeout can outlive cancellation or capability negotiation.
        if (self._air_supports_step_done is False
                and self._legacy_fallback_step and self._capturing
                and self._legacy_fallback_step == self._pending_step):
            step = self._legacy_fallback_step
            self._complete_step(step)
            self.status_label.setText(f"步骤 {step} 完成（已确认旧空中端模式）")

    def _on_import_timeout(self) -> None:
        self.status_label.setText("导入校准文件超时：空中端未响应")
        self.query_active_calibration()

    def _complete_step(self, step: str) -> None:
        self._stop_capture_watchdog()
        self._cancel_legacy_fallback()
        self._capturing = False
        self._pending_step = None
        self._pending_request_id = None
        self._pending_is_recapture = False
        self._legacy_fallback_count = 0
        if step:
            self._completed_steps.add(step)
            self._update_step_row(step, "已完成", "")
        self._steps_done = len(self._completed_steps)

        first_uncompleted_idx = next(
            (i for i, s in enumerate(self._steps) if s not in self._completed_steps),
            len(self._steps)
        )
        self._step_index = first_uncompleted_idx

        if self._step_index >= len(self._steps):
            self.instruction_label.setText("所有步骤已采集完毕！请点击“解算并生成”完成校准解算。")
        else:
            next_step = self._steps[self._step_index]
            self.instruction_label.setText(self._get_step_instruction(next_step))

        self.status_label.setText(f"步骤 {step} 完成（{self._steps_done}/{len(self._steps)}）")
        self._refresh_step_highlight()
        self._set_session_controls()

    def _revert_step_row(self, step: str, reason: str = "") -> None:
        if step in self._completed_steps:
            self._update_step_row(step, "已完成", f"重采拒绝: {reason}" if reason else "")
        else:
            self._update_step_row(step, "待采集", f"拒绝: {reason}" if reason else "")

    def _revert_all_pending_steps(self, reason: str = "") -> None:
        for row in range(self.steps_table.rowCount()):
            item_step = self.steps_table.item(row, 0)
            item_status = self.steps_table.item(row, 1)
            if item_step and item_status and "请求" in item_status.text():
                s = item_step.text()
                self._revert_step_row(s, reason)

    def _update_active_calibration_display(self) -> None:
        osl_info = "未启用"
        if self._active_calibration:
            ports = self._active_calibration.get("ports") or []
            ports_str = "+".join(ports) if ports else "P1"
            pts = self._active_calibration.get("points")
            pts_str = f", {pts}点" if pts else ""
            band = self._active_calibration.get("valid_band_hz")
            band_str = ""
            if isinstance(band, (list, tuple)) and len(band) == 2:
                band_str = f", {band[0]/1e6:.1f}-{band[1]/1e6:.1f} MHz"
            elif isinstance(band, str):
                band_str = f", {band}"
            cal_id = self._active_calibration.get("cal_id") or ""
            fn = self._active_calibration.get("filename")
            fn_str = f" [{fn}]" if fn else ""
            osl_info = f"{ports_str}{pts_str}{band_str}{fn_str}"

        bg_info = "未启用"
        if self._active_background:
            pts = self._active_background.get("points")
            pts_str = f"{pts}点" if pts else "已生效"
            bg_channels = self._active_background.get("channels")
            if isinstance(bg_channels, (list, tuple)) and bg_channels:
                names = {"S11": "HH", "S22": "VV"}
                pts_str += ", " + "+".join(names.get(str(c), str(c)) for c in bg_channels)
            bg_info = f"{pts_str}"

        text = f"当前生效: OSL[{osl_info}] | 背景[{bg_info}]"
        self.active_cal_label.setText(text)

        if hasattr(self._window, "calibration_status_label"):
            self._window.calibration_status_label.setText(f"校准: OSL[{osl_info}]")

    # ---- 交互动作实现 ----

    def begin_calibration(self) -> None:
        if not self._connected:
            self.status_label.setText("未连接空中端（开始校准已禁用）")
            return
        if self._cal_id is not None:
            return
        self._stop_capture_watchdog()
        self._cancel_legacy_fallback()
        cal_id = f"cal_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:4]}" 
        choice = str(self.mode_combo.currentData() or "osl")
        ports: str | None = None
        if choice == "osl_p1":
            mode, ports = "osl", "P1"
        elif choice == "osl_p2":
            mode, ports = "osl", "P2"
        else:
            mode = choice
        if mode == "osl":
            steps = [s for s in self.OSL_STEPS if ports is None or s.startswith(ports + "_")]
        else:
            steps = ["Background"]
        params: dict[str, object] = {
            "cal_id": cal_id,
            "mode": mode,
            "repeats": int(self.repeats.value()),
            "config": {
                "start_mhz": float(self.cal_start.value()),
                "stop_mhz": float(self.cal_stop.value()),
                "points": int(self.cal_points.value()),
                "ifbw_hz": int(self.cal_ifbw.value()),
                "power_dbm": float(self.cal_power.value()),
                "dwell_us": int(self.cal_dwell.value()),
            },
        }
        if ports is not None:
            params["ports"] = ports
        self._cal_id = cal_id
        self._mode = mode
        self._steps = steps
        self._step_index = 0
        self._steps_done = 0
        self._completed_steps = set()
        self._solved = False
        self._capturing = False
        self._pending_step = None
        self._pending_request_id = None

        self.steps_table.setRowCount(0)
        for name in steps:
            self._append_step_row(name, "待采集", "")

        self._refresh_step_highlight()
        self.status_label.setText(f"已发送 cal_begin（cal_id={cal_id}）…")
        self._send("cal_begin", params)
        self._set_session_controls()

    def capture_next_step(self) -> None:
        if self._cal_id is None or self._capturing or self._step_index >= len(self._steps):
            return
        step = self._steps[self._step_index]
        req_id = f"req_cap_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"
        self._capturing = True
        self._pending_step = step
        self._pending_is_recapture = False
        self._pending_request_id = req_id
        self._update_step_row(step, "请求采集中…", "")
        self.status_label.setText(f"已发送 cal_capture（{step}）…")
        self._refresh_step_highlight()
        self._set_session_controls()
        self._start_capture_watchdog()
        self._send("cal_capture", {
            "cal_id": self._cal_id,
            "step": step,
            "recapture": False,
            "request_id": req_id,
        })

    def recapture_selected_step(self) -> None:
        step = self.recapture_combo.currentText().strip()
        if not step:
            return
        self.recapture_step(step)

    def recapture_step(self, step: str) -> None:
        if self._cal_id is None or self._capturing or step not in self._steps:
            return
        req_id = f"req_recap_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"
        self._capturing = True
        self._pending_step = step
        self._pending_is_recapture = True
        self._pending_request_id = req_id
        self._solved = False
        step_idx = self._steps.index(step)
        self._step_index = step_idx
        self._update_step_row(step, "请求重采中…", "")
        self.status_label.setText(f"已发送 cal_capture（重采 {step}，单次点击直接执行）…")
        self._refresh_step_highlight()
        self._set_session_controls()
        self._start_capture_watchdog()
        self._send("cal_capture", {
            "cal_id": self._cal_id,
            "step": step,
            "recapture": True,
            "request_id": req_id,
        })

    def finish_calibration(self) -> None:
        if self._cal_id is None or self._capturing:
            return
        self._send("cal_finish", {"cal_id": self._cal_id})
        self.status_label.setText(f"已发送 cal_finish（cal_id={self._cal_id}）…")

    def activate_calibration(self) -> None:
        if self._cal_id is None or self._capturing:
            return
        self._send("cal_activate", {"cal_id": self._cal_id})
        self.status_label.setText(f"已发送 cal_activate（cal_id={self._cal_id}）…")

    def cancel_session(self) -> None:
        if self._cal_id is None:
            return
        cal_id = self._cal_id
        self._send("cal_cancel", {"cal_id": cal_id, "reason": "地面操作员取消"})
        self._stop_capture_watchdog()
        self._cancel_legacy_fallback()
        self.status_label.setText(f"已发送 cal_cancel（cal_id={cal_id}）…")
        self._cal_id = None
        self._capturing = False
        self._pending_step = None
        self._pending_request_id = None
        self._revert_all_pending_steps("地面操作员取消")
        self._set_session_controls()

    def _notify_warning(self, title: str, text: str) -> None:
        self.status_label.setText(f"{title}：{text}")
        if os.environ.get("QT_QPA_PLATFORM") != "offscreen":
            QtWidgets.QMessageBox.warning(self, title, text)

    def import_calibration_file(self, file_path: str | None = None) -> bool:
        """加载已有校准文件并完整传输到空中端激活。"""
        if not self._connected:
            self.status_label.setText("未连接空中端（无法导入校准文件）")
            return False
        if not isinstance(file_path, str) or not file_path:
            file_path = None
        if file_path is None:
            file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "选择校准文件", "", "JSON 校准文件 (*.json);;所有文件 (*.*)"
            )
        if not file_path:
            return False

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content_str = f.read()
            data = json.loads(content_str)
        except Exception as exc:
            self.status_label.setText(f"读取校准文件失败: {exc}")
            self._notify_warning("读取失败", f"校准文件不是有效的 JSON: {exc}")
            return False

        if not isinstance(data, dict):
            self.status_label.setText("无效校准文件: 根节点必须为 JSON 对象")
            self._notify_warning("格式错误", "校准文件根节点必须为 JSON 对象")
            return False

        # 补遗上限 10MB 检查
        if len(content_str.encode("utf-8")) > 10 * 1024 * 1024:
            self.status_label.setText("校准文件过大（超过 10 MB 上限）")
            self._notify_warning("超限", "校准文件超过 10 MB 传输上限")
            return False

        has_v2 = data.get("format") == "uav_gpr_solt_calibration_v2"
        has_meas = "measurements" in data or any(k in data for k in ("Open", "Short", "Load", "P1_Open", "P1_Short", "P1_Load"))
        if not (has_v2 or has_meas):
            self.status_label.setText("无效校准文件: 缺少 SOLT 标准件测量数据")
            self._notify_warning("格式错误", "校准文件缺少必要的 SOLT 标准件数据")
            return False

        filename = os.path.basename(file_path)
        req_id = f"req_imp_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"
        self._pending_import_content = content_str
        self._pending_import_filename = filename
        self._import_pending_cal_id = self._cal_id
        self.status_label.setText(f"正在向空中端导入校准文件 {filename}…")
        self._send("cal_import_file", {
            "filename": filename,
            "content": content_str,
            "activate": True,
            "request_id": req_id,
        })
        self._import_timer.start(10000)
        return True

    def export_calibration_file(self, target_path: str | None = None) -> bool:
        """从空中端拉取完整 OSL 校准文件并保存至地面本地。"""
        if not isinstance(target_path, str) or not target_path:
            target_path = None
        if not self._connected:
            if self._last_osl_json_content:
                save_path = target_path
                if not save_path:
                    save_path, _ = QtWidgets.QFileDialog.getSaveFileName(
                        self, "保存/导出校准文件", f"solt_calibration_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json", "JSON 校准文件 (*.json);;所有文件 (*.*)"
                    )
                if save_path:
                    try:
                        with open(save_path, "w", encoding="utf-8") as f:
                            f.write(self._last_osl_json_content)
                        self.status_label.setText(f"校准文件已成功导出至本地: {os.path.basename(save_path)}")
                        return True
                    except Exception as exc:
                        self.status_label.setText(f"写入本地文件失败: {exc}")
                        self._notify_warning("导出失败", f"写入失败: {exc}")
                        return False
            self.status_label.setText("未连接空中端（且无本地生效校准内容）")
            self._notify_warning("导出提示", "未连接空中端，且本地无生效校准内容可供导出。")
            return False

        self.status_label.setText("正在请求空中端导出完整校准文件…")
        req_id = f"req_exp_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"
        self._pending_export_paths[req_id] = target_path
        self._pending_export_path = target_path
        self._send("cal_export_file", {
            "cal_id": self._cal_id,
            "format": "uav_gpr_solt_calibration_v2",
            "request_id": req_id,
        })
        return True

    def query_active_calibration(self) -> None:
        """向空中端查询当前生效的 OSL 与背景校准组件及能力。"""
        if not self._connected:
            return
        self.status_label.setText("正在查询空中端当前生效状态…")
        self._send("cal_query_active", {})

    def clear_active_calibration(self, target: str = "all") -> None:
        """清除空中端当前生效的校准或背景（遵循补遗精准边界）。"""
        if not self._connected:
            return
        self._send("cal_clear_active", {"target": target})

    # ---- 回程消息处理（双通道：command_result + calibration_message）----

    def on_command_result(self, header: dict) -> None:
        """处理空中端 command_ack 回执（含 result/supported）。"""
        action = str(header.get("action") or "")
        if action == "cal_import_file":
            self._import_timer.stop()
            if header.get("supported") is False:
                msg = "空中端版本较低，不支持远程文件导入/重采，请升级空中端程序"
                self.status_label.setText(f"导入失败：{msg}")
                self._notify_warning("不支持", msg)
                return
            result = header.get("result")
            if isinstance(result, dict) and result.get("ok"):
                self._active_calibration = dict(result)
                self._last_osl_json_content = getattr(self, "_pending_import_content", None)
                # 补遗明确：OSL 变更彻底失效背景
                if self._active_background:
                    self._active_background = None
                    self.status_label.setText("校准文件导入并激活成功。注意：先前背景已失效，请重新采集背景。")
                else:
                    ports = result.get("ports") or []
                    ports_str = "+".join(ports) if ports else "P1"
                    pts = result.get("points") or ""
                    band = result.get("valid_band_hz") or []
                    band_str = f"{band[0]/1e6:.1f}-{band[1]/1e6:.1f} MHz" if isinstance(band, (list, tuple)) and len(band) == 2 else ""
                    summary = f"{ports_str}, {pts}频点, {band_str}"
                    self.status_label.setText(f"校准文件导入并激活成功: {summary}")
                self._update_active_calibration_display()

                import_cal_id = getattr(self, "_import_pending_cal_id", None)
                if self._mode == "osl" and self._cal_id == import_cal_id:
                    for step_name in self.OSL_STEPS:
                        self._update_step_row(step_name, "已导入", "从文件加载")
                    self._completed_steps = set(self.OSL_STEPS)
                    self._steps_done = len(self._completed_steps)
                    self._step_index = len(self.OSL_STEPS)
                    self._refresh_step_highlight()
                    self.instruction_label.setText("校准文件已导入并生效！可直接进行雷达采集或切换模式新建背景采集。")
                if self._cal_id == import_cal_id:
                    self._cal_id = None
                    self._set_session_controls()
                self._import_pending_cal_id = None
            else:
                err = (result.get("error") if isinstance(result, dict) else None) or "未知错误"
                self.status_label.setText(f"导入校准文件失败: {err}")
                self._notify_warning("导入失败", f"空中端拒绝校准文件: {err}")

        elif action == "cal_export_file":
            result = header.get("result")
            req_id = str(
                (result.get("request_id") if isinstance(result, dict) else None)
                or header.get("request_id") or ""
            )
            # Membership distinguishes a valid chooser request (value=None)
            # from a missing, already-consumed or foreign request.
            if not req_id or req_id not in self._pending_export_paths:
                return
            save_path = self._pending_export_paths.pop(req_id)
            if header.get("supported") is False:
                msg = "空中端版本较低，不支持远程文件导出，请升级空中端程序"
                self.status_label.setText(f"导出失败：{msg}")
                self._notify_warning("不支持", msg)
                return
            result = header.get("result")
            if isinstance(result, dict) and result.get("ok"):
                content = result.get("content")
                if isinstance(content, dict):
                    content_str = json.dumps(content, indent=2, ensure_ascii=False)
                else:
                    content_str = str(content or "")

                try:
                    parsed = json.loads(content_str)
                    if not isinstance(parsed, dict) or not content_str.strip():
                        raise ValueError("导出的校准数据为空或非有效 JSON 对象")
                except Exception as exc:
                    self.status_label.setText(f"空中端导出的内容非有效 JSON: {exc}")
                    self._notify_warning("导出失败", f"校准数据损坏: {exc}")
                    return

                default_name = str(result.get("filename") or f"solt_calibration_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
                if not save_path:
                    save_path, _ = QtWidgets.QFileDialog.getSaveFileName(
                        self, "保存/导出校准文件", default_name, "JSON 校准文件 (*.json);;所有文件 (*.*)"
                    )
                    if not save_path:
                        self.status_label.setText("导出已取消，未保存校准文件。")
                        return
                if save_path:
                    try:
                        with open(save_path, "w", encoding="utf-8") as f:
                            f.write(content_str)
                        self.status_label.setText(f"校准文件已完整导出至本地: {os.path.basename(save_path)}")
                        self._last_osl_json_content = content_str
                    except Exception as exc:
                        self.status_label.setText(f"写入本地文件失败: {exc}")
                        self._notify_warning("导出失败", f"无法写入文件: {exc}")
            else:
                err = (result.get("error") if isinstance(result, dict) else None) or "未知错误"
                self.status_label.setText(f"导出校准文件失败: {err}")
                self._notify_warning("导出失败", f"空中端拒绝导出: {err}")

        elif action == "cal_capture":
            if header.get("supported") is False:
                req_id = str(header.get("request_id") or "")
                if self._capturing and (not req_id or not self._pending_request_id or req_id != self._pending_request_id):
                    return
                self._stop_capture_watchdog()
                self._cancel_legacy_fallback()
                self._capturing = False
                if self._pending_step:
                    self._revert_step_row(self._pending_step, "空中端不支持重采")
                first_uncompleted_idx = next(
                    (i for i, s in enumerate(self._steps) if s not in self._completed_steps),
                    len(self._steps)
                )
                self._step_index = first_uncompleted_idx
                self._pending_step = None
                self._pending_request_id = None
                self._pending_is_recapture = False
                msg = "空中端版本较低，不支持远程文件导入/重采，请升级空中端程序"
                self.status_label.setText(f"采集被拒绝: {msg}")
                self._notify_warning("不支持", msg)
                self._refresh_step_highlight()
                self._set_session_controls()
                return
            result = header.get("result")
            if isinstance(result, dict) and result.get("ok") is False:
                req_id = str(result.get("request_id") or header.get("request_id") or "")
                # 当处于在途采集时，任何不携带有效且匹配 request_id 的 cal_capture 拒绝绝不得重置 _capturing 或回退步骤行 (FINDING-GND-09)
                if self._capturing and (not req_id or not self._pending_request_id or req_id != self._pending_request_id):
                    return
                if req_id and req_id in self._completed_request_ids:
                    return
                if req_id:
                    self._completed_request_ids.add(req_id)
                self._stop_capture_watchdog()
                self._cancel_legacy_fallback()
                self._capturing = False
                err = str(result.get("error") or "未知拒绝")
                if self._pending_step:
                    self._revert_step_row(self._pending_step, err)
                self._pending_step = None
                self._pending_request_id = None
                self._pending_is_recapture = False
                first_uncompleted_idx = next(
                    (i for i, s in enumerate(self._steps) if s not in self._completed_steps),
                    len(self._steps)
                )
                self._step_index = first_uncompleted_idx
                self.status_label.setText(f"采集被拒绝: {err}")
                self._refresh_step_highlight()
                self._set_session_controls()

        elif action == "cal_query_active":
            result = header.get("result")
            req_id = str(
                (result.get("request_id") if isinstance(result, dict) else None)
                or header.get("params", {}).get("request_id")
                or header.get("request_id") or ""
            )
            is_watchdog = bool(
                self._capturing and self._watchdog_query_pending
                and self._watchdog_query_token and req_id
                and req_id == self._watchdog_query_token
                and self._watchdog_query_context == (
                    self._cal_id, self._pending_request_id, self._pending_step
                )
            )
            in_prog = result.get("in_progress_session") if isinstance(result, dict) else None
            session_matches = bool(
                isinstance(in_prog, dict) and self._cal_id is not None
                and in_prog.get("cal_id") == self._cal_id
            )
            current_busy = bool(
                session_matches and in_prog.get("busy")
                and in_prog.get("active_request_id") == self._pending_request_id
            )
            if self._capturing and not is_watchdog and not current_busy:
                # Do not let a retired query rewrite even the new step's row.
                return
            if is_watchdog:
                self._reset_watchdog_query()
            if header.get("supported") is False:
                if self._capturing:
                    # Unsupported status queries do not prove the worker idle.
                    self.status_label.setText("空中端不支持状态查询，仍等待采集回执；请配套升级两端软件。")
                    self._capture_watchdog_timer.start(10000)
                else:
                    self._air_supports_step_done = False
                return
            result = header.get("result")
            if isinstance(result, dict) and result.get("ok"):
                caps = result.get("capabilities") or {}
                if isinstance(caps, dict) and caps.get("step_done") is not None:
                    self._air_supports_step_done = bool(caps.get("step_done"))

                in_prog = result.get("in_progress_session") or {}
                if isinstance(in_prog, dict) and in_prog.get("busy"):
                    self.status_label.setText(f"空中端仍在忙碌执行采集任务（步骤: {in_prog.get('current_step')}），保持等待…")
                    self._capture_watchdog_timer.start(10000)
                    return

                # 若空中端返回步骤状态字典，同步地面步骤集合
                steps_status = in_prog.get("steps_status") if session_matches else None
                if isinstance(steps_status, dict) and steps_status:
                    for s, st in steps_status.items():
                        if st in ("done", "completed"):
                            self._completed_steps.add(s)
                            self._update_step_row(s, "已完成", "")
                        elif st == "pending" and s not in self._completed_steps:
                            self._update_step_row(s, "待采集", "")
                    self._steps_done = len(self._completed_steps)
                    first_uncompleted_idx = next(
                        (i for i, s in enumerate(self._steps) if s not in self._completed_steps),
                        len(self._steps)
                    )
                    self._step_index = first_uncompleted_idx
                    self._refresh_step_highlight()

                # 仅响应由 watchdog 超时显式触发的查询，陈旧普通查询绝不得强行重置 _capturing！
                if is_watchdog and self._capturing:
                    self._stop_capture_watchdog()
                    self._capturing = False
                    if self._pending_step:
                        self._revert_step_row(self._pending_step, "超时已释放")
                    first_uncompleted_idx = next(
                        (i for i, s in enumerate(self._steps) if s not in self._completed_steps),
                        len(self._steps)
                    )
                    self._step_index = first_uncompleted_idx
                    self._pending_step = None
                    self._pending_request_id = None
                    self._pending_is_recapture = False
                    self.status_label.setText("远程采集已结束，已恢复操作控制。")
                    self._refresh_step_highlight()
                    self._set_session_controls()

                active_osl = result.get("active_osl")
                if isinstance(active_osl, dict) and active_osl.get("active"):
                    self._active_calibration = dict(active_osl)
                else:
                    self._active_calibration = None

                active_bg = result.get("active_background")
                if isinstance(active_bg, dict) and active_bg.get("active"):
                    self._active_background = dict(active_bg)
                else:
                    self._active_background = None

                self._update_active_calibration_display()

        elif action == "cal_clear_active":
            result = header.get("result")
            if isinstance(result, dict) and result.get("ok"):
                cleared = str(result.get("cleared") or result.get("target") or "all")
                if cleared in ("all", "osl"):
                    self._active_calibration = None
                    self._last_osl_json_content = None
                if cleared in ("all", "background"):
                    self._active_background = None
                self._update_active_calibration_display()
                self.status_label.setText(f"已清除生效校准/背景（目标: {cleared}）")

    def on_calibration_message(self, header: dict) -> None:
        msg_type = str(header.get("type") or "")
        cal_id = str(header.get("cal_id") or "")
        if self._cal_id is not None and cal_id and cal_id != self._cal_id:
            return

        if msg_type == "calibration_capture":
            step = str(header.get("step") or "")
            index = header.get("index")
            of = header.get("of")
            self._pet_capture_watchdog()
            self._update_step_row(
                step,
                f"采集中 {index}/{of}" if of else "采集中…",
                "",
            )
            if isinstance(index, int) and isinstance(of, int) and index >= of:
                if self._air_supports_step_done is False:
                    self._complete_step(step)
                else:
                    # 新端或能力未知（None）：严禁在末样本到达后提前解锁，保持 _capturing 锁定，必须等待权威 step_done
                    self._update_step_row(step, "已完成", "等待收尾…")
                    self.status_label.setText(f"步骤 {step} 样本已收齐，等待空中端完成硬件收尾…")

        elif msg_type == "calibration_quality":
            step = str(header.get("step") or "")
            repeat_rms = header.get("repeat_rms")
            max_abs = header.get("max_abs_err")
            sample = header.get("sample_count")
            note = str(header.get("note") or "")
            rms_text = "-" if repeat_rms is None else f"{float(repeat_rms):.3e}"
            max_text = "-" if max_abs is None else f"{float(max_abs):.3e}"
            self._update_step_row(
                step,
                "已完成",
                f"重复RMS={rms_text} 最大误差={max_text} 样本={sample} {note}".strip(),
            )
            self.quality_label.setText(
                f"质量指标: {step} 重复RMS={rms_text} 最大绝对误差={max_text} 样本数={sample}"
                f"{'（' + note + '）' if note else ''}"
            )
            if self._air_supports_step_done is False and self._capturing:
                self._complete_step(step)

        elif msg_type == "calibration_result":
            status = str(header.get("status") or "")
            if status == "step_done":
                req_id = str(header.get("request_id") or "")
                if req_id and self._pending_request_id and req_id != self._pending_request_id:
                    return
                if req_id and req_id in self._completed_request_ids:
                    return
                if req_id:
                    self._completed_request_ids.add(req_id)
                self._stop_capture_watchdog()
                self._cancel_legacy_fallback()
                self._air_supports_step_done = True
                self._legacy_fallback_count = 0
                step = str(header.get("step") or self._pending_step or "")
                self._complete_step(step)

            elif status == "planned":
                self.status_label.setText(f"会话已就绪（cal_id={cal_id}，mode={header.get('mode')}）")
                self._step_index = 0
                self._steps_done = 0
                self._completed_steps = set()
                self._solved = False
                self._capturing = False
                steps_from_air = header.get("steps")
                if isinstance(steps_from_air, list) and steps_from_air:
                    self._steps = [str(s) for s in steps_from_air]
                    self.steps_table.setRowCount(0)
                    for s in self._steps:
                        self._append_step_row(s, "待采集", "")
                self._refresh_step_highlight()
                self._set_session_controls()

            elif status == "solved":
                self._stop_capture_watchdog()
                self._solved = True
                self._capturing = False
                p = header.get("path")
                summary = str(header.get("profile_summary") or "")
                band = header.get("valid_band_hz") or "-"
                self.result_label.setText(
                    f"解算完成：路径={p or '内存'} 有效频段={band}"
                    + (f" | {summary}" if summary else "")
                )
                self.status_label.setText("校准已解算成功，请点击“激活校准”使其生效。")
                if header.get("content"):
                    self._last_osl_json_content = str(header.get("content"))
                self._refresh_step_highlight()
                self._set_session_controls()

            elif status == "activated":
                self._stop_capture_watchdog()
                self._capturing = False
                self._solved = False
                mode = str(header.get("mode") or self._mode)
                if mode == "osl":
                    self._active_calibration = dict(header)
                    if self._active_background:
                        self._active_background = None
                        self.status_label.setText("OSL 校准已激活。注意：先前背景已失效，请重新采集背景。")
                    else:
                        self.status_label.setText(f"OSL 校准已激活（cal_id={cal_id}）：后续采集立即生效。")
                elif mode == "background":
                    self._active_background = dict(header)
                    self.status_label.setText(f"背景参考已激活（cal_id={cal_id}）：后续探地采集将自动扣除背景。")
                self._cal_id = None
                self._update_active_calibration_display()
                self._set_session_controls()

            elif status == "cancelled":
                self._stop_capture_watchdog()
                self.status_label.setText(f"会话已取消（cal_id={cal_id}）；已收 capture 文件保留")
                self._cal_id = None
                self._capturing = False
                self._pending_step = None
                self._pending_request_id = None
                self._revert_all_pending_steps("会话已取消")
                self._set_session_controls()

            elif status in ("rejected", "failed"):
                req_id = str(header.get("request_id") or "")
                # 当处于在途采集时，任何不携带有效且匹配 request_id 的拒绝绝不得重置 _capturing 或回退步骤行 (FINDING-GND-09)
                if self._capturing and (not req_id or not self._pending_request_id or req_id != self._pending_request_id):
                    return
                if req_id and req_id in self._completed_request_ids:
                    return
                if req_id:
                    self._completed_request_ids.add(req_id)
                self._stop_capture_watchdog()
                self._cancel_legacy_fallback()
                self._capturing = False
                reason = str(header.get("reason") or header.get("error") or "-")
                self.status_label.setText(
                    f"空中端拒绝/失败（{status}）：{reason}"
                )
                step = str(header.get("step") or self._pending_step or "")
                if step:
                    self._revert_step_row(step, reason)
                else:
                    self._revert_all_pending_steps(reason)
                if status == "failed":
                    self._cal_id = None
                if status == "rejected" and not step:
                    self._cal_id = None
                self._pending_step = None
                self._pending_request_id = None
                self._pending_is_recapture = False
                first_uncompleted_idx = next(
                    (i for i, s in enumerate(self._steps) if s not in self._completed_steps),
                    len(self._steps)
                )
                self._step_index = first_uncompleted_idx
                if self._step_index >= len(self._steps):
                    self.instruction_label.setText("所有步骤已采集完毕！请点击“解算并生成”完成校准解算。")
                else:
                    next_step = self._steps[self._step_index]
                    self.instruction_label.setText(self._get_step_instruction(next_step))
                self._refresh_step_highlight()
                self._set_session_controls()

    # ---- 步骤表工具 ----

    def _append_step_row(self, name: str, state: str, remark: str) -> None:
        row = self.steps_table.rowCount()
        self.steps_table.insertRow(row)
        self.steps_table.setItem(row, 0, QtWidgets.QTableWidgetItem(name))
        self.steps_table.setItem(row, 1, QtWidgets.QTableWidgetItem(state))
        self.steps_table.setItem(row, 2, QtWidgets.QTableWidgetItem(self._get_step_guide(name)))
        self.steps_table.setItem(row, 3, QtWidgets.QTableWidgetItem(remark))

    def _update_step_row(self, name: str, state: str, remark: str) -> None:
        for row in range(self.steps_table.rowCount()):
            if self.steps_table.item(row, 0) is not None and self.steps_table.item(row, 0).text() == name:
                if state:
                    self.steps_table.setItem(row, 1, QtWidgets.QTableWidgetItem(state))
                if remark:
                    self.steps_table.setItem(row, 3, QtWidgets.QTableWidgetItem(remark))
                return


CalibrationDialog = CalibrationWizard


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
        # 第2轮 R-A-19/R-C-20：窗口侧镜像的空中 run_id（hello/acquiring 切换
        # 时更新；R-A-20 迟到护栏用它放行"基线镜像"身份）。
        self._mirrored_air_run_id: str | None = None
        # 第2轮 R-C-11：远程连接状态（校准对话框"开始校准"门控）。
        self._remote_connected = False
        # 已提交设置：主界面与 command_params 只读这里，对话框编辑独立草稿。
        self.acq_settings = AcquisitionUiSettings()
        self.bscan_settings = BscanUiSettings()
        self.bscan_rows: dict[str, list[np.ndarray]] = {name: [] for name in UI_CHANNELS}
        self.bscan_rows_complex: dict[str, list[np.ndarray]] = {name: [] for name in UI_CHANNELS}
        self.bscan_time_ns: np.ndarray | None = None
        self.bscan_processing_signature: tuple[bool, float | None] | None = None
        self.bscan_filter_status_override: str | None = None
        self.bscan_window_start = 1
        self.bscan_follow_latest = True
        self.bscan_contrast_percent = BSCAN_CONTRAST_DEFAULT_PERCENT
        self._bscan_max_abs: dict[str, float] = {}
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
        # 第5轮：链路积压可见化
        self._air_inflight: int = 0
        self._air_unacknowledged: int = 0
        self._air_paused: bool = False  # 第5轮 Q1=B：空中端暂停标记（paused 状态置位）。
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
        # 第2轮 R-A-19/R-C-20：hello 触发的 run 切换 → 合成 acquiring 路径
        #（不新增第二套基线逻辑）；校准回程 → 校准展示分支（R-C-17）。
        if hasattr(self.receiver, "run_switched"):
            self.receiver.run_switched.connect(self._on_hello_run_switched)
        if hasattr(self.receiver, "calibration_message"):
            self.receiver.calibration_message.connect(self._on_calibration_message)
        # 第7轮：带结果字段的 command_ack → 设备面板回执（既有 ack 通道；
        # 测试注入的假 receiver 可能没有该信号，按 hasattr 兜底）。
        if hasattr(self.receiver, "command_result"):
            self.receiver.command_result.connect(self._on_command_result)
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
        # 第2轮 §5.2/R-C-11：地面校准入口（编排与展示，不直接访问硬件）。
        # 追加在"退出"之后：既有冻结测试锁定 [2]=分隔线、[3]=退出 的顺序
        #（test_ground_station_compact_ui.test_chinese_menus_and_actions_exist）。
        self.action_calibration = self.menu_task.addAction("校准……")
        self.action_calibration.setObjectName("action_calibration")
        self.action_pause = self.menu_task.addAction("暂停")
        self.action_pause.setObjectName("action_pause")
        self.action_resume = self.menu_task.addAction("继续")
        self.action_resume.setObjectName("action_resume")

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
        # 第7轮 D5：空中端设备面板入口（独立非模态窗口，不动主界面布局）。
        self.action_show_device_panel = self.menu_view.addAction("空中端设备……")
        self.action_show_device_panel.setObjectName("action_show_device_panel")

        self.menu_help = menubar.addMenu("帮助")
        self.action_about = self.menu_help.addAction("关于")
        self.action_about.setObjectName("action_about")

        self.action_single_acquisition.triggered.connect(self.send_single)
        self.action_acquisition_settings.triggered.connect(self.open_acquisition_settings)
        self.action_calibration.triggered.connect(self.open_calibration_dialog)
        self.action_pause.triggered.connect(self.send_pause)
        self.action_resume.triggered.connect(self.send_resume)
        self.action_exit.triggered.connect(self.close)
        self.action_bscan_settings.triggered.connect(self.open_bscan_settings)
        self.action_clear_bscan.triggered.connect(self.clear_bscan)
        self.action_show_details.triggered.connect(self.show_details_window)
        self.action_show_log.triggered.connect(self.show_log_window)
        self.action_show_pipeline_metrics.triggered.connect(self.show_pipeline_metrics)
        # 第7轮：设备面板入口。
        self.action_show_device_panel.triggered.connect(self.show_device_panel)
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
        self.pause_button = QtWidgets.QPushButton("暂停")
        self.pause_button.setObjectName("pause_button")
        self.pause_button.setMaximumWidth(120)
        self.pause_button.setMinimumHeight(32)
        self.resume_button = QtWidgets.QPushButton("继续")
        self.resume_button.setObjectName("resume_button")
        self.resume_button.setMaximumWidth(120)
        self.resume_button.setMinimumHeight(32)
        self.stop_button = QtWidgets.QPushButton("停止")
        self.stop_button.setObjectName("stop_button")
        self.stop_button.setMaximumWidth(120)
        self.stop_button.setMinimumHeight(32)
        self.start_button.clicked.connect(self.send_start)
        self.pause_button.clicked.connect(self.send_pause)
        self.resume_button.clicked.connect(self.send_resume)
        self.stop_button.clicked.connect(self.send_stop)
        row.addWidget(self.start_button)
        row.addWidget(self.pause_button)
        row.addWidget(self.resume_button)
        row.addWidget(self.stop_button)
        row.addStretch(1)
        # 校准失效/提醒横条（改扫频参数后与生效校准对不上时显示；平时隐藏）。
        # 放在操作栏右侧而非状态栏：状态栏冻结为 3 个业务标签。
        self.calibration_fit_label = QtWidgets.QLabel("")
        self.calibration_fit_label.setObjectName("calibration_fit_label")
        self.calibration_fit_label.setVisible(False)
        self._calibration_fit_level = "ok"
        row.addWidget(self.calibration_fit_label)
        return bar

    def _build_main_content(self) -> QtWidgets.QWidget:
        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        splitter.setObjectName("main_splitter")
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(6)

        left = QtWidgets.QWidget()
        left.setObjectName("bscan_area")
        # 网格布局：第 0 列 = HH / VV / 历史滑块（行序与原 QVBoxLayout 一致），
        # 第 1 列 = 竖向色阶滑块（跨 HH、VV 两行）。
        left_layout = QtWidgets.QGridLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setHorizontalSpacing(4)
        left_layout.setVerticalSpacing(8)
        left_layout.setColumnStretch(0, 1)
        left_layout.setColumnStretch(1, 0)

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
            left_layout.addWidget(container, index, 0)
            left_layout.setRowStretch(index, 1)
        left_layout.addWidget(
            self._build_bscan_contrast_panel(), 0, 1, len(UI_CHANNELS), 1
        )

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
        left_layout.addWidget(slider_row, len(UI_CHANNELS), 0)
        left_layout.setRowStretch(len(UI_CHANNELS), 0)

        self.gnss_map = GnssMapWidget()
        self.gnss_map.setObjectName("gnss_map")

        splitter.addWidget(left)
        splitter.addWidget(self.gnss_map)
        splitter.setStretchFactor(0, 7)
        splitter.setStretchFactor(1, 3)
        splitter.setSizes([966, 414])  # 约 70:30
        return splitter

    def _build_bscan_contrast_panel(self) -> QtWidgets.QWidget:
        """B-scan 右侧竖向色阶滑块：调整 colorbar 显示范围（颜色深浅）。"""
        panel = QtWidgets.QWidget()
        panel.setObjectName("bscan_contrast_panel")
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        title = QtWidgets.QLabel("色阶")
        title.setObjectName("bscan_contrast_title")
        title.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        self.bscan_contrast_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Vertical)
        self.bscan_contrast_slider.setObjectName("bscan_contrast_slider")
        self.bscan_contrast_slider.setRange(BSCAN_CONTRAST_MIN_PERCENT, BSCAN_CONTRAST_MAX_PERCENT)
        self.bscan_contrast_slider.setValue(int(self.bscan_contrast_percent))
        self.bscan_contrast_slider.setSingleStep(1)
        self.bscan_contrast_slider.setPageStep(10)
        self.bscan_contrast_slider.setToolTip(
            "调整 B-scan 色阶显示范围（相对当前窗口最大幅值的百分比）。\n"
            "调小：范围收窄，弱反射颜色更深、更明显；调大：颜色变浅。"
        )
        self.bscan_contrast_slider.valueChanged.connect(self.on_bscan_contrast_changed)
        self.bscan_contrast_label = QtWidgets.QLabel(f"{int(self.bscan_contrast_percent)}%")
        self.bscan_contrast_label.setObjectName("bscan_contrast_label")
        self.bscan_contrast_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        self.bscan_contrast_label.setMinimumWidth(40)
        # QToolButton（非 QPushButton）：主窗口业务按钮仍只有开始/停止/暂停/继续。
        self.bscan_contrast_reset_button = QtWidgets.QToolButton()
        self.bscan_contrast_reset_button.setText("重置")
        self.bscan_contrast_reset_button.setObjectName("bscan_contrast_reset_button")
        self.bscan_contrast_reset_button.setToolTip("恢复 100%（自动色阶，与原显示一致）")
        self.bscan_contrast_reset_button.setMaximumWidth(48)
        self.bscan_contrast_reset_button.clicked.connect(self.reset_bscan_contrast)
        layout.addWidget(title)
        layout.addWidget(self.bscan_contrast_slider, 1, QtCore.Qt.AlignmentFlag.AlignHCenter)
        layout.addWidget(self.bscan_contrast_label)
        layout.addWidget(self.bscan_contrast_reset_button, 0, QtCore.Qt.AlignmentFlag.AlignHCenter)
        return panel

    def on_bscan_contrast_changed(self, value: int) -> None:
        """色阶滑块变化：只改各通道 ImageItem 的 levels，不重算显示矩阵。"""
        percent = min(max(int(value), BSCAN_CONTRAST_MIN_PERCENT), BSCAN_CONTRAST_MAX_PERCENT)
        self.bscan_contrast_percent = percent
        self.bscan_contrast_label.setText(f"{percent}%")
        for name, image_item in getattr(self, "bscan_images", {}).items():
            max_abs = self._bscan_max_abs.get(name)
            if max_abs is None or image_item.image is None:
                continue
            image_item.setLevels(bscan_contrast_levels(max_abs, percent))

    def reset_bscan_contrast(self) -> None:
        self.bscan_contrast_slider.setValue(BSCAN_CONTRAST_DEFAULT_PERCENT)

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
        self.backlog_label = QtWidgets.QLabel("空闲/已停止")
        self.backlog_label.setObjectName("backlog_label")
        self.backlog_label.setWordWrap(True)
        details_layout.addRow("监听地址", self.listen_label)
        details_layout.addRow("保存目录", self.out_dir_label)
        details_layout.addRow("最近事件", self.status_label)
        details_layout.addRow("当前 Trace", self.trace_label)
        details_layout.addRow("采集元数据", self.acquisition_label)
        details_layout.addRow("链路积压", self.backlog_label)
        details_layout.addRow("Dewow", self.dewow_status)
        details_layout.addRow("Flat Filter", self.bscan_filter_status)
        details_layout.addRow("旧格式", self.legacy_status)
        self.details_window.resize(640, 360)

        # ---- 两个设置对话框（非模态，打开时加载已提交值）----
        self.acquisition_dialog = AcquisitionSettingsDialog(self)
        self.settings_dialog = self.acquisition_dialog
        self.bscan_dialog = BscanSettingsDialog(self)
        # ---- 第2轮 §5.2：校准编排向导（非模态，R-C-04/R-C-11）----
        self.calibration_dialog = CalibrationWizard(self)
        self.calibration_wizard = self.calibration_dialog
        self.calibration_dialog.set_connected(self._remote_connected)
        # ---- 第7轮 D1+D5：空中端设备面板（黑匣子全遥控，独立非模态窗口，
        #      默认隐藏；与运行日志/详细状态同模式，不推翻主界面布局）----
        self.device_panel = AirDevicePanel(self)

    def show_device_panel(self) -> None:
        """打开空中端设备面板（非模态；打开时同步当前连接状态）。"""
        panel = getattr(self, "device_panel", None)
        if panel is None:
            return
        panel.set_link_connected(self._remote_connected)
        panel.show()
        panel.raise_()

    def open_calibration_dialog(self) -> None:
        """打开校准对话框（非模态；未连接时"开始校准"禁用并明确提示）。"""
        self.calibration_dialog.set_connected(self._remote_connected)
        self._sync_calibration_plan_to_acquisition()
        self.calibration_dialog.show()
        self.calibration_dialog.raise_()

    def _sync_calibration_plan_to_acquisition(self) -> None:
        """打开校准对话框时，把校准的频率范围/频点数预填成当前采集设置。

        校准必须和采集用同一套频率轴才有效（否则空中端会判"校准已失效"）。
        校准会话进行中不动输入框；用户仍可手动改。
        """
        dialog = self.calibration_dialog
        if getattr(dialog, "_cal_id", None) is not None:
            return
        settings = getattr(self, "acq_settings", None)
        if settings is None:
            return
        try:
            dialog.cal_start.setValue(float(settings.start_mhz))
            dialog.cal_stop.setValue(float(settings.stop_mhz))
            dialog.cal_points.setValue(int(settings.points))
        except (AttributeError, TypeError, ValueError):
            pass
        # 校准端口跟着采集通道走（只在 OSL 几种之间切换，不动“空采背景”选择）。
        combo = getattr(dialog, "mode_combo", None)
        if combo is not None and str(combo.currentData() or "") in ("osl", "osl_p1", "osl_p2"):
            wanted = {"S11": "osl_p1", "S22": "osl_p2"}.get(str(getattr(settings, "channels", "dual")), "osl")
            index = combo.findData(wanted)
            if index >= 0:
                combo.setCurrentIndex(index)

    def open_calibration_wizard(self) -> None:
        """打开校准向导（open_calibration_dialog 的别名）。"""
        self.open_calibration_dialog()

    def _on_calibration_message(self, header: dict) -> None:
        """校准回程分派（R-C-17：不进 trace 路径，只到展示层）。"""
        msg_type = str(header.get("type") or "")
        self.calibration_dialog.on_calibration_message(header)
        if msg_type == "calibration_result":
            status = str(header.get("status") or "")
            self.log(f"校准回程: cal_id={header.get('cal_id')} status={status}")

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

    def backlog_counts(self) -> dict[str, int]:
        """获取当前空中端链路积压计数的快照字典。"""
        return {
            "inflight": int(getattr(self, "_air_inflight", 0)),
            "unacknowledged": int(getattr(self, "_air_unacknowledged", 0)),
        }

    def backlog_text(self, state: str = "") -> str:
        """获取当前链路积压的展示文案。"""
        inflight = int(getattr(self, "_air_inflight", 0))
        unack = int(getattr(self, "_air_unacknowledged", 0))
        # 第5轮 Q1=B：暂停排空可见化——"已暂停 · 还差 K 条"随逐条到达递减，
        # 归零后"已暂停（待命）"；恢复采集由 command_ack resume 清除暂停标记。
        if state == "paused" or getattr(self, "_air_paused", False):
            # 第5轮 F-A：K 口径与 stopping 统一——remaining(=outstanding) 已含
            # 在途，不再叠加 inflight。
            pending = unack
            if pending > 0:
                return f"已暂停 · 还差 {pending} 条"
            return "已暂停（待命）"
        if state == "stopping":
            if unack > 0:
                return f"排空中 · 还差 {unack} 条"
            return "排空中"
        if inflight > 0 or unack > 0:
            return f"在途 {inflight} / 未确认 {unack}"
        return "空闲/已停止"

    def show_about(self) -> None:
        QtWidgets.QMessageBox.about(self, "关于", "UAV-GPR 地面站")

    def apply_acquisition_settings(self, settings: AcquisitionUiSettings) -> None:
        self.acq_settings = settings
        self.log("采集设置已应用")

    def sweep_params(self) -> dict[str, object]:
        return self.acq_settings.sweep_params()

    def send_set_params(self, params: dict[str, object] | None = None) -> None:
        payload = dict(params if params is not None else self.acq_settings.sweep_params())
        self.acquisition_status_label.setText("采集状态：正在下发参数")
        if hasattr(self.receiver, "send_command"):
            self.receiver.send_command("set_params", payload)

    def open_settings_center(self) -> None:
        self.open_acquisition_settings()

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
        if "remote ready" in lowered:
            self.connection_status_label.setText("连接状态：已连接")
            self._set_remote_connected(True)
        elif text.startswith("Connected"):
            # 第7轮 t9 修复（修法 a，engB 方案）：accept 时刻 hello 握手未
            # 完成——连接状态标签语义不变（仍显示"已连接"），但刻意不置
            # connected：设备面板 5s 自动刷新与校准门控等 hello 回环完成后
            # 再放行，消除窗口期 command 撞空中端 _connect 严格拒绝的竞态
            # （第2轮 finding⑥ 现行违例，revA F1-r2）。
            self.connection_status_label.setText("连接状态：已连接")
        elif "disconnected" in lowered or "client error" in lowered:
            self.connection_status_label.setText("连接状态：已断开")
            self._set_remote_connected(False)
        elif lowered.startswith("listening"):
            self.connection_status_label.setText("连接状态：等待连接")
            self._set_remote_connected(False)

    def _set_remote_connected(self, connected: bool) -> None:
        """R-C-11：连接状态变化同步到校准对话框（未连接→"开始校准"禁用）。"""
        self._remote_connected = bool(connected)
        self.gnss_map.set_link_connected(self._remote_connected)
        dialog = getattr(self, "calibration_dialog", None)
        if dialog is not None:
            dialog.set_connected(self._remote_connected)
        # 第7轮：同步到设备面板（未连接时面板禁发并显示断开）。
        panel = getattr(self, "device_panel", None)
        if panel is not None:
            panel.set_link_connected(self._remote_connected)

    def apply_calibration_fit(self, fit: object) -> None:
        """显示空中端回报的"校准与当前扫频参数是否对得上"。

        ``level``：ok（隐藏）/ warning（橙色提醒，校准仍在用）/
        stale（红色"校准已失效，请重新校准"，空中端已停用该校准）。
        """
        if not isinstance(fit, dict):
            return
        level = str(fit.get("level") or "ok")
        message = str(fit.get("message") or "")
        label = getattr(self, "calibration_fit_label", None)
        if label is None:
            return
        if level == "stale":
            label.setText("⚠ " + (message or "校准已失效，请重新校准"))
            label.setStyleSheet("color: #ffffff; background: #c0392b; padding: 1px 6px; font-weight: bold;")
            label.setToolTip(message)
            label.setVisible(True)
        elif level == "warning":
            label.setText("⚠ " + (message or "校准提醒"))
            label.setStyleSheet("color: #000000; background: #f5b041; padding: 1px 6px;")
            label.setToolTip(message)
            label.setVisible(True)
        else:
            label.setText("")
            label.setToolTip("")
            label.setVisible(False)
        if level != getattr(self, "_calibration_fit_level", "ok"):
            if level == "ok":
                self.log("扫频参数与生效校准重新一致：空中端恢复使用校准")
            else:
                self.log(message)
        self._calibration_fit_level = level

    def _on_command_result(self, header: dict) -> None:
        """第7轮：把带结果字段的 command_ack 转发给设备面板（既有 ack 通道）。"""
        result_for_fit = header.get("result")
        if isinstance(result_for_fit, dict) and isinstance(result_for_fit.get("calibration_fit"), dict):
            self.apply_calibration_fit(result_for_fit["calibration_fit"])
        if (
            header.get("action") == "set_params"
            and isinstance(result_for_fit, dict)
            and result_for_fit.get("ok") is False
        ):
            reason = str(result_for_fit.get("error") or "未知原因")
            self.log(f"空中端拒绝参数修改: {reason}")
            self.acquisition_status_label.setText(f"采集状态：参数被空中端拒绝（{reason}）")
        if header.get("action") == "test_gnss":
            result = header.get("result")
            if isinstance(result, dict):
                self.gnss_map.update_data(result)
                if result.get("ok") is False:
                    self.gnss_map.show_gnss_status(
                        "测试未取得定位（连接情况以天空端回执为准）："
                        + str(result.get("error") or result.get("detail") or "无详细信息")
                    )
        panel = getattr(self, "device_panel", None)
        if panel is not None:
            panel.on_command_result(dict(header))
        dialog = getattr(self, "calibration_dialog", None)
        if dialog is not None and hasattr(dialog, "on_command_result"):
            dialog.on_command_result(dict(header))

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
            elif action == "pause":
                self.acquisition_status_label.setText("采集状态：已暂停")
            elif action == "resume":
                # 第5轮 Q1=B：恢复采集清除暂停标记（积压计数自然归零）。
                self._air_paused = False
                self.acquisition_status_label.setText("采集状态：采集中")
            elif action == "set_params":
                self.acquisition_status_label.setText("采集状态：参数已下发（以空中应用结果为准）")
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
        # 第5轮：链路积压更新
        remaining = header.get("remaining")
        inflight = header.get("inflight")
        if remaining is not None:
            self._air_unacknowledged = int(remaining)
        if inflight is not None:
            self._air_inflight = int(inflight)
        if state in ("stopped", "drain_failed"):
            self._air_inflight = 0
            self._air_unacknowledged = 0
            self._air_paused = False
        if state == "paused":
            # 第5轮 F-B（t6 评审）：迟到护栏必须**先于**置粘性标记——旧写法先置
            # _air_paused=True 再判迟到，旧 run 的 paused 迟到会把新任务全程
            # 污染成"已暂停"（与 R-A-20 冲突，评审生产路径实测复现）。
            if self._is_late_air_status(str(header.get("run_id") or "")):
                self.log(
                    f"忽略迟到暂停状态: run_id={str(header.get('run_id') or '')}"
                    f"（当前任务={self._task_run_id}，镜像={self._mirrored_air_run_id or '空'}）"
                )
                return
            # 第5轮 Q1=B：空中端在扫频边界暂停到位（带真实积压计数）。
            self._air_paused = True
        if hasattr(self, "backlog_label"):
            self.backlog_label.setText(self.backlog_text(state=state))

        if state == "paused":
            # 第5轮 F-A：K 口径与 stopping 统一——remaining(=outstanding) 已含
            # 在途，不再叠加 inflight（旧公式首屏多计 ≤inflight 条）。
            pending = int(getattr(self, "_air_unacknowledged", 0))
            if pending > 0:
                self.acquisition_status_label.setText(
                    f"采集状态：已暂停（排空中 · 还差 {pending} 条）"
                )
            else:
                self.acquisition_status_label.setText("采集状态：已暂停（待命）")
        elif state == "stopping":
            rem = getattr(self, "_air_unacknowledged", 0)
            if rem > 0:
                self.acquisition_status_label.setText(f"采集状态：正在停止（排空中 · 还差 {rem} 条）")
            else:
                self.acquisition_status_label.setText("采集状态：正在停止（排空中）")
        elif state == "stopped":
            # 迟到的旧 run_id stopped 不得结束新任务（第三次定向修复 +
            # 第2轮 R-A-20 词汇表护栏）。
            new_run = str(header.get("run_id") or "")
            if self._is_late_air_status(new_run):
                self.log(
                    f"忽略迟到停止状态: run_id={new_run}（当前任务={self._task_run_id}，"
                    f"镜像={self._mirrored_air_run_id or '空'}）"
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
            if self._is_late_air_status(new_run):
                self.log(
                    f"忽略迟到失败状态: run_id={new_run}（当前任务={self._task_run_id}，"
                    f"镜像={self._mirrored_air_run_id or '空'}）"
                )
                return
            self._final_stop_state = None  # 失败终态：直接显示，不再对账。
            self.acquisition_status_label.setText("采集状态：排空失败（部分数据未处理）")
        elif state == "acquiring":
            # 收到 acquiring：新任务开始（第2轮 R-A-19/R-A-19b/R-A-20 词汇表
            # 护栏：判据唯一来源 = classify_run_id + is_not_older_than）。
            new_run = str(header.get("run_id") or "")
            cls = classify_run_id(new_run)
            if cls == RUN_ID_CLASS_AIR_TASK:
                if new_run == self._task_run_id or new_run == self._mirrored_air_run_id:
                    # R-A-19② 幂等：不重复 _begin_task、不清 _final_stop_state、
                    # 不重置已建基线。
                    # 第5轮 F-B：acquiring 接受即清除迟到 paused 的粘性标记。
                    self._air_paused = False
                    self.acquisition_status_label.setText("采集状态：采集中")
                elif is_not_older_than(new_run, self._mirrored_air_run_id or ""):
                    self._accept_air_run(new_run)
                    # 第5轮 F-B：同上，接受新 run 即清暂停粘性标记。
                    self._air_paused = False
                    self.acquisition_status_label.setText("采集状态：采集中")
                else:
                    # 早于基线的旧 run acquiring：补传模式由接收线程处置，
                    # 绝不为旧 run 重建任务基线（R-A-19b）。
                    self.log(
                        f"忽略早于基线的 acquiring: run_id={new_run}"
                        f"（基线={self._mirrored_air_run_id or '空'}）"
                    )
            else:
                # R-A-20③：非 air_task（本地占位/idle/校准）按当前 run 处理，
                # 沿用第 1 轮语义（第 1 轮冻结测试兼容）。
                if new_run and new_run != self._task_run_id:
                    self._begin_task(run_id=new_run)
                    self._final_stop_state = None
                    self._stop_commit_barrier = None
                    self._last_ground_run_id = new_run
                # 第5轮 F-B：新任务基线建立即清暂停粘性标记。
                self._air_paused = False
                self.acquisition_status_label.setText("采集状态：采集中")
        else:
            # 未知/原始英文状态值一律映射为中文未知，不透出原始值。
            self.acquisition_status_label.setText("采集状态：未知状态")
        if detail:
            self.log(f"空中端状态: {state} {detail}")
        _ = drain_state

    def _is_late_air_status(self, run_id: str) -> bool:
        """R-A-20 迟到护栏（身份类别的唯一分类函数见 R-A-19/R-A-19b 调用点）。

        本护栏只做**身份相等性**判定（第1轮冻结语义的镜像扩展）：

        ① run_id == 当前基线（含本地占位 start-/single-<ts>）→ 正常；
        ② == 基线镜像的空中 run_id（切换时记录）→ 正常；
        ③④ 其余（真旧空中任务、旧式本地 id 等）→ 迟到忽略——与第 1 轮冻结
        测试 ``test_late_old_run_id_stopped_ignored``（new-run/old-run）语义
        逐字一致；"本地占位不判迟到"由 acquiring/hello 切换后的基线替换保证
        （R-A-20 的 run₂ 场景在切换后 run₂ == 基线，走①）。
        词汇表分类（classify_run_id）已在 R-A-19①/R-A-19b 切换判定处生效，
        能到达本函数③④分支的身份均已通过词汇表路由。
        """
        if not run_id or run_id == self._task_run_id:
            return False
        if run_id == self._mirrored_air_run_id:
            return False
        return True

    def _accept_air_run(self, run_id: str) -> None:
        """接受新空中任务身份：窗口侧任务基线（hello 与 acquiring 共用路径）。

        R-C-20①：hello 触发的切换经 :meth:`_on_hello_run_switched` 合成等价的
        ``acquisition_status{state:"acquiring"}`` 到这里，不新增第二套基线逻辑。
        """
        self._begin_task(run_id=run_id)
        self._final_stop_state = None
        self._stop_commit_barrier = None
        self._last_ground_run_id = run_id
        self._mirrored_air_run_id = run_id

    def _on_hello_run_switched(self, run_id: str) -> None:
        """hello 触发的 run 切换（R-C-20①）：合成 acquiring 路径建任务基线。"""
        self.log(f"空中 run 切换（hello）: {run_id}")
        self._on_acquisition_status(
            {"state": "acquiring", "run_id": run_id, "via": "hello"}
        )

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
        self._air_inflight = 0
        self._air_unacknowledged = 0
        if hasattr(self, "backlog_label"):
            self.backlog_label.setText("空闲/已停止")
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
        self._air_paused = False  # 新任务：清除暂停标记。
        # 第5轮 Q2=A：开始新采集时自动清空 B-scan（停止后可回看，新任务干净画布）。
        self.clear_bscan()
        self._begin_task(run_id=f"start-{int(time.time())}")
        self.acquisition_status_label.setText("采集状态：正在发送启动命令")
        self.receiver.send_command("start", self.command_params())

    def send_single(self) -> None:
        self._final_stop_state = None
        self._stop_commit_barrier = None
        self._air_paused = False
        # 第5轮 Q2=A：单次采集同样是新任务，画布同步清空。
        self.clear_bscan()
        self._begin_task(run_id=f"single-{int(time.time())}")
        self.acquisition_status_label.setText("采集状态：单次采集命令已发送")
        self.receiver.send_command("single", self.command_params(single=True))

    def send_stop(self) -> None:
        self.acquisition_status_label.setText("采集状态：正在发送停止命令")
        self.receiver.send_command("stop")

    def send_pause(self) -> None:
        self.acquisition_status_label.setText("采集状态：正在发送暂停命令")
        self.receiver.send_command("pause")

    def send_resume(self) -> None:
        self.acquisition_status_label.setText("采集状态：正在发送继续命令")
        self.receiver.send_command("resume")

    def clear_bscan(self) -> None:
        self.bscan_rows = {name: [] for name in UI_CHANNELS}
        self.bscan_rows_complex = {name: [] for name in UI_CHANNELS}
        self._set_bscan_missing_channels(())
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
        self._bscan_max_abs.clear()
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
        traces_real = {
            name: (np.asarray(trace).real if np.iscomplexobj(trace) else trace)
            for name, trace in trace_by_channel.items()
        }
        # 单通道采集：只有一个通道有数据，另一幅 B-scan 补零列保持两幅道数同步。
        present_channels = tuple(name for name in UI_CHANNELS if name in traces_real) or UI_CHANNELS
        try:
            candidate_time, candidate_traces = validate_bscan_trace(
                time_ns, traces_real, channels=present_channels
            )
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
        if not hasattr(self, "bscan_rows_complex"):
            self.bscan_rows_complex = {name: [] for name in UI_CHANNELS}
        missing_channels = tuple(name for name in UI_CHANNELS if name not in candidate_traces)
        for name in UI_CHANNELS:
            if name in candidate_traces:
                self.bscan_rows[name].append(candidate_traces[name])
                self.bscan_rows_complex[name].append(np.asarray(trace_by_channel[name], dtype=np.complex128))
            else:
                self.bscan_rows[name].append(np.zeros_like(candidate_time))
                self.bscan_rows_complex[name].append(np.zeros(candidate_time.shape, dtype=np.complex128))
        self._set_bscan_missing_channels(missing_channels)
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
        rows_c = getattr(self, "bscan_rows_complex", {}).get(name, [])
        if rows_c and len(rows_c) == len(self.bscan_rows.get(name, [])):
            matrix_c = np.column_stack([np.asarray(row, dtype=np.complex128) for row in rows_c])
            flat_window = self.flat_filter_window()
            if self.bscan_settings.flat_filter_enabled and matrix_c.shape[1] >= flat_window:
                matrix_c = flat_reflection_filter(matrix_c, window_size=flat_window)
            return matrix_c.real
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
          时使用安全非零范围；S11/S22 各自独立计算；
        - 再乘以色阶滑块百分比（默认 100% = 原行为），见 bscan_contrast_levels。
        """
        rows_c = getattr(self, "bscan_rows_complex", {}).get(name, [])
        flat_window = self.flat_filter_window()
        if rows_c and len(rows_c) == len(self.bscan_rows.get(name, [])):
            if self.bscan_settings.flat_filter_enabled:
                matrix_c, _applied = apply_flat_filter_windowed(
                    rows_c, self.bscan_window_start, flat_window, filter_func=flat_reflection_filter
                )
                matrix = matrix_c.real
            else:
                matrix = bscan_window_from_rows(self.bscan_rows[name], self.bscan_window_start).matrix
        else:
            rows = self.bscan_rows.get(name, [])
            if not rows:
                return None, None
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
        self._bscan_max_abs[name] = max_abs
        return matrix, bscan_contrast_levels(max_abs, self.bscan_contrast_percent)

    def _set_bscan_missing_channels(self, missing: tuple[str, ...]) -> None:
        """单通道采集时在没采的那幅 B-scan 标题上注明“本次未采集”。"""
        if tuple(missing) == getattr(self, "_bscan_missing_channels", ()):
            return
        self._bscan_missing_channels = tuple(missing)
        for name in UI_CHANNELS:
            plot = getattr(self, "bscan_plots", {}).get(name)
            if plot is None:
                continue
            suffix = "（本次未采集）" if name in missing else ""
            plot.setTitle(f"{CHANNEL_LABELS[name]} B-scan{suffix}")

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
            if getattr(self, "_air_unacknowledged", 0) > 0:
                self._air_unacknowledged = max(0, self._air_unacknowledged - 1)
            if getattr(self, "_air_inflight", 0) > 0:
                self._air_inflight = max(0, self._air_inflight - 1)
            if hasattr(self, "backlog_label"):
                if getattr(self, "_air_paused", False):
                    curr_state = "paused"
                elif "正在停止" in self.acquisition_status_label.text():
                    curr_state = "stopping"
                else:
                    curr_state = ""
                self.backlog_label.setText(self.backlog_text(state=curr_state))
            if getattr(self, "_air_paused", False):
                # 第5轮 Q1=B + F-A：暂停排空中逐条递减（K 口径=remaining，与
                # stopping 统一），归零切"已暂停（待命）"。
                pending = int(getattr(self, "_air_unacknowledged", 0))
                if pending > 0:
                    self.acquisition_status_label.setText(
                        f"采集状态：已暂停（排空中 · 还差 {pending} 条）"
                    )
                else:
                    self.acquisition_status_label.setText("采集状态：已暂停（待命）")
            elif "正在停止（排空中" in self.acquisition_status_label.text():
                rem = getattr(self, "_air_unacknowledged", 0)
                if rem > 0:
                    self.acquisition_status_label.setText(f"采集状态：正在停止（排空中 · 还差 {rem} 条）")
                else:
                    self.acquisition_status_label.setText("采集状态：正在停止（排空中）")

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
                trace_candidates = {
                    name: trace
                    for name, trace in (("S11", decoded.trace_s11), ("S22", decoded.trace_s22))
                    if trace is not None
                }
                try:
                    validated_time, validated_traces = validate_bscan_trace(
                        decoded.time_ns, trace_candidates, channels=tuple(trace_candidates) or UI_CHANNELS
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
            self.gnss_map.update_data(gnss, age_ms=acquisition.get("gnss_age_ms"))
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
