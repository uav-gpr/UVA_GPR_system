"""Continuous acquisition building blocks: strict S11/S22 assembly + persistent backend.

Phases 1 and 2 of the persistent-continuous-acquisition roadmap (see
``docs/LibreVNA连续采集功能开发路线图.md``):

- Phase 1 (this file): the data types and the strict
  :class:`ContinuousSweepAssembler`.
- Phase 2 (also this file): the persistent USB backend state machine
  :class:`PersistentLibreVnaBackend` with :class:`BackendState`,
  :class:`SessionStatistics`, the pending queue, ACK-batch data retention and
  fail-closed error handling.  USB lifecycle lives in
  ``librevna_transport``; this module imports it but never imports PyUSB at
  module load time.

The assembler is transport- and protocol-independent: it consumes
:class:`S11S22Point` objects (produced by the phase-2 backend from
``librevna_protocol.parse_s11_s22_point``) and emits :class:`CompleteSweep`
objects.  It strictly accepts only ``0 -> 1 -> ... -> N-1``; any drop, repeat,
regression, jump, out-of-range value, invalid complex value or non-finite /
non-increasing frequency discards the whole current sweep and waits for the
next point 0.  Data is never joined across sweeps, pause boundaries, timeouts
or reconnects (the caller calls :meth:`ContinuousSweepAssembler.reset_current`
on those boundaries).

Design is adapted from the validated ``ContinuousSweepAssembler`` in
``reference_code/rebar_inspector_s11_s22_reference.py``, extended with
``jump_points`` and ``completed_sweeps`` diagnostics.  The acceptance semantics
(discard rules, counters) match the reference exactly.
"""

from __future__ import annotations

import math
import numbers
import struct
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import NoReturn, Sequence, TypeAlias

import numpy as np

from librevna.librevna_errors import (
    LibreVnaDisconnectedError,
    LibreVnaError,
    LibreVnaNackError,
    LibreVnaPendingOverflowError,
    LibreVnaProtocolError,
    LibreVnaReleaseError,
    LibreVnaStateError,
    LibreVnaSweepError,
    LibreVnaTimeoutError,
    LibreVnaUnsupportedConfigError,
)
from librevna.librevna_protocol import (
    ACK,
    DEVICE_INFO,
    NACK,
    REQUEST_DEVICE_INFO,
    SET_IDLE,
    S11_S22_STAGES_BITMAP,
    S11_STAGES_BITMAP,
    S22_STAGES_BITMAP,
    SWEEP_SETTINGS,
    VNA_DATAPOINT,
    LibreVnaDeviceInfo,
    Packet,
    PacketStream,
    SweepSettings,
    decode_device_info,
    encode_packet,
    parse_s11_point,
    parse_s11_s22_point,
    parse_s22_point,
)
from librevna.librevna_transport import LibreVnaTransportProtocol, LibreVnaUsbTransport


@dataclass(frozen=True)
class S11S22Point:
    """One frequency point carrying both reflections from one datapoint."""

    point_number: int
    frequency_hz: float
    s11: complex
    s22: complex


@dataclass(frozen=True)
class InvalidPoint:
    """Marker for a datapoint that cannot be decoded safely.

    The caller (phase-2 backend) emits this instead of a
    :class:`S11S22Point` when a ``VNADatapoint`` fails the same-packet
    S11/S22 contract, so the whole current sweep is discarded rather than
    patched with stale data.
    """

    point_number: int | None = None
    reason: str = "invalid or incomplete S11/S22 datapoint"


StreamEvent: TypeAlias = S11S22Point | InvalidPoint


@dataclass(frozen=True)
class CompleteSweep:
    """One complete, validated sweep; ``raw`` has shape ``(2, frequency)``.

    Channel order is fixed: ``raw[0]`` = S11, ``raw[1]`` = S22.
    ``trace_index`` is assigned by the caller (backend) and is 0 when built by
    the assembler.  ``statistics`` is a snapshot of the assembler counters at
    the moment the sweep completed.
    """

    frequency_hz: np.ndarray
    raw: np.ndarray
    trace_index: int = 0
    statistics: dict[str, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        frequency = _frequency_axis(self.frequency_hz)
        raw = _complex_array(self.raw, "raw")
        if raw.shape != (2, frequency.size):
            raise ValueError(
                f"raw must have shape (2, {frequency.size}); got {raw.shape}"
            )
        object.__setattr__(self, "frequency_hz", _readonly(frequency))
        object.__setattr__(self, "raw", _readonly(raw))
        object.__setattr__(self, "statistics", dict(self.statistics))

    @property
    def s11(self) -> np.ndarray:
        return self.raw[0]

    @property
    def s22(self) -> np.ndarray:
        return self.raw[1]


class ContinuousSweepAssembler:
    """Strictly assemble ``0..n_points-1`` without joining different sweeps.

    Only ``0 -> 1 -> ... -> N-1`` is accepted (roadmap section 8):

    1. Only point 0 can start a new sweep.
    2. A new point 0 interrupting a partial sweep counts the old sweep as
       dropped/incomplete.
    3. A duplicate, regression, jump or out-of-range point immediately
       invalidates the current sweep.
    4. After desynchronisation all non-zero points are ignored until the next
       point 0.
    5. S11/S22 receiver/reference fields must come from the same datapoint
       (enforced by the parse layer before points reach this assembler).
    6. Required descriptors must each occur exactly once (parse layer).
    7. Zero reference or NaN/Inf values discard the current sweep (parse layer
       emits :class:`InvalidPoint`).
    8. Completion re-verifies point count and that frequencies are finite and
       strictly increasing.
    9. The device's actual returned frequency axis is used.
    10. pause / timeout / reconnect / configure boundaries must call
        :meth:`reset_current` to clear any partial sweep.
    """

    def __init__(self, n_points: int) -> None:
        if isinstance(n_points, bool) or not isinstance(n_points, int) or n_points < 2:
            raise ValueError("n_points must be an integer >= 2")
        self.n_points = n_points
        #: cumulative diagnostics (snapshot available via ``statistics``)
        self.completed_sweeps = 0
        self.dropped_sweeps = 0
        self.incomplete_sweeps = 0
        self.duplicate_points = 0
        self.jump_points = 0
        self.out_of_range_points = 0
        self.invalid_points = 0
        self._active = False
        self._next_expected = 0
        self._points: list[S11S22Point] = []
        self._unsynchronised_drop_counted = False

    @property
    def statistics(self) -> dict[str, int]:
        """Snapshot of cumulative diagnostics (fresh dict each call)."""
        return {
            "completed_sweeps": self.completed_sweeps,
            "dropped_sweeps": self.dropped_sweeps,
            "incomplete_sweeps": self.incomplete_sweeps,
            "duplicate_points": self.duplicate_points,
            "jump_points": self.jump_points,
            "out_of_range_points": self.out_of_range_points,
            "invalid_points": self.invalid_points,
        }

    def reset_current(self) -> None:
        """Drop local partial state while preserving cumulative diagnostics.

        Use at pause / timeout / reconnect / configure boundaries so no data
        is joined across them.
        """
        self._active = False
        self._next_expected = 0
        self._points = []
        self._unsynchronised_drop_counted = False

    def mark_invalid(self, _point_number: int | None = None) -> None:
        """Record an invalid point and discard the current sweep if active."""
        self.invalid_points += 1
        self._discard_or_mark_unsynchronised()

    def feed(self, point: S11S22Point) -> CompleteSweep | None:
        if not isinstance(point, S11S22Point):
            raise TypeError("point must be S11S22Point")
        if not _valid_point_values(point):
            self.mark_invalid(point.point_number)
            return None

        number = point.point_number
        if number < 0 or number >= self.n_points:
            self.out_of_range_points += 1
            self._discard_or_mark_unsynchronised()
            return None

        if number == 0:
            if self._active:
                self._discard_active()
            self._active = True
            self._next_expected = 1
            self._points = [point]
            self._unsynchronised_drop_counted = False
            return None

        if not self._active:
            self._mark_unsynchronised_drop()
            return None

        if number != self._next_expected:
            if number < self._next_expected:
                self.duplicate_points += 1
            else:
                self.jump_points += 1
            self._discard_active()
            return None

        self._points.append(point)
        self._next_expected += 1
        if self._next_expected != self.n_points:
            return None

        frequency = np.asarray([item.frequency_hz for item in self._points], dtype=np.float64)
        if not np.all(np.diff(frequency) > 0.0):
            self.invalid_points += 1
            self._discard_active()
            return None
        raw = np.asarray(
            [[item.s11 for item in self._points], [item.s22 for item in self._points]],
            dtype=np.complex128,
        )
        self.completed_sweeps += 1
        result = CompleteSweep(frequency, raw, statistics=self.statistics)
        self._active = False
        self._next_expected = 0
        self._points = []
        self._unsynchronised_drop_counted = False
        return result

    def _discard_active(self) -> None:
        if self._active:
            self.dropped_sweeps += 1
            self.incomplete_sweeps += 1
        self._active = False
        self._next_expected = 0
        self._points = []
        self._unsynchronised_drop_counted = True

    def _mark_unsynchronised_drop(self) -> None:
        if not self._unsynchronised_drop_counted:
            self.dropped_sweeps += 1
            self._unsynchronised_drop_counted = True

    def _discard_or_mark_unsynchronised(self) -> None:
        if self._active:
            self._discard_active()
        else:
            self._mark_unsynchronised_drop()


def _valid_point_values(point: S11S22Point) -> bool:
    return (
        isinstance(point.point_number, int)
        and not isinstance(point.point_number, bool)
        and math.isfinite(point.frequency_hz)
        and _finite_complex(point.s11)
        and _finite_complex(point.s22)
    )


def _finite_complex(value: complex) -> bool:
    return math.isfinite(value.real) and math.isfinite(value.imag)


def _frequency_axis(values: Sequence[float] | np.ndarray) -> np.ndarray:
    frequency = np.asarray(values, dtype=np.float64)
    if frequency.ndim != 1 or frequency.size < 2:
        raise ValueError("frequency_hz must be a one-dimensional vector with >= 2 points")
    if not np.all(np.isfinite(frequency)) or not np.all(np.diff(frequency) > 0.0):
        raise ValueError("frequency_hz must be finite and strictly increasing")
    return np.array(frequency, dtype=np.float64, copy=True)


def _complex_array(values: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(values)
    if not np.issubdtype(array.dtype, np.complexfloating):
        array = np.asarray(array, dtype=np.complex128)
    result = np.array(array, dtype=np.complex128, copy=True)
    if not np.all(np.isfinite(result.real)) or not np.all(np.isfinite(result.imag)):
        raise ValueError(f"{name} contains NaN or infinity")
    return result


def _readonly(array: np.ndarray) -> np.ndarray:
    result = np.array(array, copy=True)
    result.setflags(write=False)
    return result


# =====================================================================
# Phase 2: persistent backend state machine (see roadmap sections 6, 8, 13)
# =====================================================================

#: 每次 USB read 的单次超时（短，便于停止/断开快速响应）。
DEFAULT_USB_READ_TIMEOUT_MS = 50
#: 等待 DeviceInfo 的总体超时。
DEFAULT_DEVICE_INFO_TIMEOUT_S = 3.0
#: 等待命令 ACK 的总体超时。
DEFAULT_COMMAND_TIMEOUT_S = 2.0
#: 完整一道的总体超时下限。
DEFAULT_SWEEP_TIMEOUT_MIN_S = 2.0
#: pending 上限默认（roadmap §8）：``max(4*N, 1024)``。
PENDING_SLOTS_PER_POINT = 4
MIN_PENDING_SLOTS = 1024


class BackendState(Enum):
    """PersistentLibreVnaBackend 状态机（roadmap §6）。

    ``CLOSED -> OPEN_IDLE -> STREAMING -> PAUSED``；命令或设备状态不可确定时
    进入 :attr:`FAULTED`，只允许 ``close()`` / ``open()``。
    """

    CLOSED = "closed"
    OPEN_IDLE = "open_idle"
    STREAMING = "streaming"
    PAUSED = "paused"
    FAULTED = "faulted"


@dataclass
class SessionStatistics:
    """后端会话统计（快照；``statistics`` 属性每次返回新鲜副本）。

    ``open_count`` / ``connection_generation`` 跨 open 会话累计；其余字段按
    open 会话重置（新会话 trace_index 从 0 重新开始）。assembler 派生的计数
    （dropped / incomplete / duplicate / jump / out_of_range / invalid /
    completed_sweeps）在 ``statistics`` 属性里从活跃 assembler 实时读取。
    """

    open_count: int = 0
    configure_count: int = 0
    connection_generation: int = 0
    trace_index: int = 0
    completed_sweeps: int = 0
    dropped_sweeps: int = 0
    incomplete_sweeps: int = 0
    duplicate_points: int = 0
    jump_points: int = 0
    out_of_range_points: int = 0
    invalid_points: int = 0
    unexpected_acks: int = 0
    ignored_packets: int = 0
    pending_depth_current: int = 0
    pending_depth_peak: int = 0
    pending_overflows: int = 0
    sweep_timeouts: int = 0
    disconnects: int = 0
    faults: int = 0

    def to_dict(self) -> dict[str, int]:
        """JSON 可序列化快照（全部为 int）。"""
        return dict(asdict(self))


def _check_positive_real(name: str, value: object) -> None:
    """实数秒数/值字段：拒绝 bool 与非 Real；NaN/Inf/<=0 抛 ValueError。

    类型检查先于 ``math.isfinite``，避免泄漏 ``math.isfinite`` 对字符串/非数字
    的偶然 TypeError（把该 TypeError 当未设计接口）。
    """
    if isinstance(value, bool):
        raise TypeError(f"{name} 必须是实数（bool 被拒绝），实际得到 bool")
    if not isinstance(value, numbers.Real):
        raise TypeError(
            f"{name} 必须是实数（int/float/NumPy 标量），"
            f"实际得到 {type(value).__name__}"
        )
    if not math.isfinite(value):
        raise ValueError(
            f"{name} 必须是有限值（不允许 NaN/Infinity），实际得到 {value!r}"
        )
    if value <= 0:
        raise ValueError(f"{name} 必须是正数，实际得到 {value!r}")


def _check_positive_int(name: str, value: object) -> None:
    """整数字段：拒绝 bool/float/str/非 int；<=0 抛 ValueError。"""
    if isinstance(value, bool):
        raise TypeError(f"{name} 必须是 int（bool 被拒绝），实际得到 bool")
    if not isinstance(value, int):
        raise TypeError(f"{name} 必须是 int，实际得到 {type(value).__name__}")
    if value <= 0:
        raise ValueError(f"{name} 必须是正 int，实际得到 {value!r}")


def _add_exc_note(exc: BaseException, note: str) -> None:
    """Python 3.10+ 兼容地把诊断 note 附加到异常，且绝不抛异常。

    Python 3.11+ 使用内置 ``BaseException.add_note()``；3.10 无该方法，改为
    手工维护 ``__notes__`` 列表，使诊断统一可通过 ``getattr(exc, "__notes__", [])``
    读取。任何情况下都不会让诊断机制自身抛出异常。
    """
    if not isinstance(note, str):
        return
    add_note = getattr(exc, "add_note", None)
    if callable(add_note):
        try:
            add_note(note)
            return
        except Exception:  # noqa: BLE001 - 诊断机制不得抛异常
            pass
    try:
        notes = getattr(exc, "__notes__", None)
        if not isinstance(notes, list):
            notes = []
            setattr(exc, "__notes__", notes)
        notes.append(note)
    except Exception:  # noqa: BLE001 - 诊断机制不得抛异常
        pass


@dataclass(frozen=True)
class PersistentBackendSettings:
    """PersistentLibreVnaBackend 不可变运行参数。

    功率 / dwell 不在此处：它们由调用方在 :class:`SweepSettings` 里指定，随
    冻结配置一起在 resume 时原样重发。
    """

    usb_read_timeout_ms: int = DEFAULT_USB_READ_TIMEOUT_MS
    device_info_timeout_s: float = DEFAULT_DEVICE_INFO_TIMEOUT_S
    command_timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S
    sweep_timeout_s: float | None = None
    pending_max: int | None = None

    def __post_init__(self) -> None:
        _check_positive_int("usb_read_timeout_ms", self.usb_read_timeout_ms)
        _check_positive_real("device_info_timeout_s", self.device_info_timeout_s)
        _check_positive_real("command_timeout_s", self.command_timeout_s)
        if self.sweep_timeout_s is not None:
            _check_positive_real("sweep_timeout_s", self.sweep_timeout_s)
        if self.pending_max is not None:
            _check_positive_int("pending_max", self.pending_max)


def _state_error(action: str, state: BackendState) -> LibreVnaStateError:
    """针对具体 action/state 生成状态机违规错误（FAULTED 语义优先）。"""
    if state is BackendState.FAULTED:
        return LibreVnaStateError(
            f"后端处于 FAULTED（设备状态不可确定），{action}() 被禁止；"
            "必须 close() 后重新 open()"
        )
    if state is BackendState.CLOSED:
        return LibreVnaStateError(
            f"后端未打开（CLOSED），{action}() 被禁止；必须先 open()"
        )
    if action == "acquire_one" and state is BackendState.OPEN_IDLE:
        return LibreVnaStateError(
            "后端未配置（OPEN_IDLE），必须先 configure() 再 acquire_one()"
        )
    if action == "pause" and state is BackendState.OPEN_IDLE:
        return LibreVnaStateError(
            "后端未配置（OPEN_IDLE），必须先 configure() 再 pause()"
        )
    if action == "configure" and state is BackendState.STREAMING:
        return LibreVnaStateError(
            "后端已在 STREAMING，不能重复 configure()（正常任务只 configure 一次）"
        )
    if action == "configure" and state is BackendState.PAUSED:
        return LibreVnaStateError(
            "后端已暂停，不能 configure()；先 resume() 或 close()"
        )
    if action == "resume" and state in (
        BackendState.OPEN_IDLE,
        BackendState.STREAMING,
    ):
        return LibreVnaStateError("后端未暂停，无法 resume()")
    return LibreVnaStateError(f"后端状态 {state.value} 不允许 {action}()")


class PersistentLibreVnaBackend:
    """LibreVNA 持久连续采集后端（双反射 S11/S22，仅 stages_bitmap 0x1241）。

    生命周期（roadmap §6）：
    ``CLOSED -> open() -> OPEN_IDLE -> configure() -> STREAMING``；
    ``STREAMING -> pause() -> PAUSED -> resume() -> STREAMING``；
    命令或设备状态不可确定 -> :attr:`BackendState.FAULTED`，只允许 close/reopen。

    - 正常任务只 ``open()`` 一次、``configure()`` 一次（重复调用被拒绝或幂等）。
    - ``pause()`` 在完整 sweep 边界发送 ``SET_IDLE`` 并清边界残留；``resume()``
      用同一冻结 :class:`SweepSettings` 重发 ``SWEEP_SETTINGS``，**不** reopen、
      不重建 assembler、不重置 trace_index。
    - ``acquire_one()`` 先消费 pending（configure/resume ACK 等待期间、一次 read
      跨两道时下一道的点），再执行 USB read；一次 read 内完成的 sweep 之后的
      packet 进入 pending，不丢失、不跨 sweep 拼接。
    - pending 溢出 / NACK / 协议错误 / 断开 -> fail-closed 进入 FAULTED。
    - ``close()`` 尽力 ``SET_IDLE``（失败不阻止），始终释放 USB；幂等。
    - 返回 :class:`CompleteSweep`（trace_index 由后端赋值），legacy
      ``acquire_sparameters()`` 路径不受影响。
    """

    def __init__(
        self,
        *,
        transport: LibreVnaTransportProtocol | None = None,
        settings: PersistentBackendSettings | None = None,
    ) -> None:
        if transport is not None and not all(
            hasattr(transport, name)
            for name in ("open", "read", "write", "close", "is_open")
        ):
            raise TypeError("transport 必须实现 open/read/write/close/is_open")
        self._transport = (
            transport if transport is not None else LibreVnaUsbTransport()
        )
        self._settings = (
            settings if settings is not None else PersistentBackendSettings()
        )
        if not isinstance(self._settings, PersistentBackendSettings):
            raise TypeError("settings 必须是 PersistentBackendSettings 或 None")

        self._state = BackendState.CLOSED
        self._device_info: LibreVnaDeviceInfo | None = None
        self._stream = PacketStream()
        self._pending: deque[Packet] = deque()
        self._assembler: ContinuousSweepAssembler | None = None
        self._frozen_settings: SweepSettings | None = None
        #: 上一条完整 sweep 已组装但尚未交付给调用方；后续 datapoint 进 pending。
        self._sweep_held = False
        self._pending_max = MIN_PENDING_SLOTS
        self._sweep_timeout_s = DEFAULT_SWEEP_TIMEOUT_MIN_S
        self._stats = SessionStatistics()
        #: assembler 消失前保存的最终统计快照（close/FAULTED 后仍可读取）。
        self._saved_assembler_stats: dict[str, int] | None = None

    # ---- 查询 ----

    @property
    def state(self) -> BackendState:
        return self._state

    @property
    def is_open(self) -> bool:
        """非 CLOSED 即视为已打开（FAULTED 时 transport 仍保持打开待 close）。"""
        return self._state is not BackendState.CLOSED

    @property
    def device_info(self) -> LibreVnaDeviceInfo | None:
        """open 成功后读取到的设备信息（open 前 / close 后为 None）。"""
        return self._device_info

    @property
    def transport(self) -> LibreVnaTransportProtocol:
        return self._transport

    @property
    def config(self) -> SweepSettings | None:
        """当前冻结的 SweepSettings（STREAMING/PAUSED 时有值；其余为 None）。"""
        return self._frozen_settings

    @property
    def statistics(self) -> SessionStatistics:
        """会话统计快照（新鲜副本；assembler 派生计数实时读取，assembler 消失后
        读取 close/FAULTED 时保存的最终快照）。"""
        return SessionStatistics(
            open_count=self._stats.open_count,
            configure_count=self._stats.configure_count,
            connection_generation=self._stats.connection_generation,
            trace_index=self._stats.trace_index,
            completed_sweeps=self._assembler_stat("completed_sweeps"),
            dropped_sweeps=self._assembler_stat("dropped_sweeps"),
            incomplete_sweeps=self._assembler_stat("incomplete_sweeps"),
            duplicate_points=self._assembler_stat("duplicate_points"),
            jump_points=self._assembler_stat("jump_points"),
            out_of_range_points=self._assembler_stat("out_of_range_points"),
            invalid_points=self._assembler_stat("invalid_points"),
            unexpected_acks=self._stats.unexpected_acks,
            ignored_packets=self._stats.ignored_packets,
            pending_depth_current=len(self._pending),
            pending_depth_peak=self._stats.pending_depth_peak,
            pending_overflows=self._stats.pending_overflows,
            sweep_timeouts=self._stats.sweep_timeouts,
            disconnects=self._stats.disconnects,
            faults=self._stats.faults,
        )

    def _assembler_stat(self, name: str) -> int:
        """读取 assembler 派生计数；assembler 已消失时回退到保存的最终快照。"""
        assembler = self._assembler
        if assembler is not None:
            return getattr(assembler, name)
        saved = self._saved_assembler_stats
        return saved[name] if saved is not None else 0

    # ---- 生命周期 ----

    def open(self) -> None:
        """打开、claim、读取 DeviceInfo、SET_IDLE，进入 OPEN_IDLE。

        幂等：已处于 OPEN_IDLE/STREAMING/PAUSED 时为 no-op；FAULTED 下必须先
        close()。任一阶段失败（transport.open() / RequestDeviceInfo / 初始
        SET_IDLE）都进入统一清理：重置本地会话状态并尽力释放 transport；只有
        transport 真正释放后才回到 CLOSED，否则进入 FAULTED 等待后续 close()
        重试释放。原始打开异常始终重新抛出，清理失败信息经 add_note 保留。
        """
        if self._state in (
            BackendState.OPEN_IDLE,
            BackendState.STREAMING,
            BackendState.PAUSED,
        ):
            return
        if self._state is BackendState.FAULTED:
            raise _state_error("open", self._state)
        try:
            # transport.open() 自身异常也必须进入清理路径：一个合法的可注入
            # transport 可能已部分打开、占有资源后才抛错。
            self._transport.open()
            info = self._wait_for_device_info(
                timeout_s=self._settings.device_info_timeout_s
            )
            self._device_info = info
            self._send_command(SET_IDLE, timeout_s=self._settings.command_timeout_s)
        except Exception as open_error:
            self._handle_open_failure(open_error)
            raise
        # 只有成功完成 open + DeviceInfo + SET_IDLE 后才重置新会话统计、增加
        # open_count/connection_generation 并进入 OPEN_IDLE。
        self._reset_stats()
        self._reset_local_session()
        self._stats.open_count += 1
        self._stats.connection_generation += 1
        self._state = BackendState.OPEN_IDLE

    def _handle_open_failure(self, open_error: Exception) -> None:
        """open 失败统一清理：重置本地状态并尽力释放 transport。

        重置 device_info / PacketStream / pending / assembler / frozen config /
        held 状态，随后尽力 ``transport.close()``：

        - 只要 close 抛错，就把释放诊断附加到原始打开异常（无论 ``is_open``
          最终是 True 还是 False）；
        - transport 明确报告关闭（``is_open`` 为 False）-> 进入 CLOSED；
        - transport 仍报告打开 -> 进入 FAULTED 允许后续 :meth:`close` 重试释放，
          并附加对应诊断。

        原始打开异常始终由调用方重新抛出，本方法只用 :func:`_add_exc_note`
        保留清理信息，不替换原始异常。
        """
        self._reset_local_session()
        self._device_info = None
        release_error: Exception | None = None
        try:
            self._transport.close()
        except Exception as exc:  # noqa: BLE001 - 清理失败保留诊断
            release_error = exc
        if release_error is not None:
            _add_exc_note(
                open_error,
                f"open 失败后释放 transport 抛错（{type(release_error).__name__}: "
                f"{release_error}）",
            )
        if self._transport.is_open:
            # 资源仍被 transport 持有 -> 不能伪装安全 CLOSED。
            self._transition_to_faulted()
            if release_error is None:
                _add_exc_note(
                    open_error,
                    "open 失败后 transport.close() 未抛错但 transport 仍报告打开；"
                    "backend 进入 FAULTED，可调用 close() 重试释放",
                )
            else:
                _add_exc_note(
                    open_error,
                    "backend 进入 FAULTED，可调用 close() 重试释放",
                )
        else:
            self._state = BackendState.CLOSED

    def configure(self, settings: SweepSettings) -> None:
        """校验能力、清旧流、发送一次 SWEEP_SETTINGS，进入 STREAMING。

        参数预校验（频率/IFBW/点数/stages_bitmap）在任何 USB 写入前完成。
        命令阶段（SET_IDLE + SWEEP_SETTINGS）任一失败（NACK / ACK 超时 /
        transport / 协议错误）即进入 :attr:`BackendState.FAULTED`。
        """
        if self._state is not BackendState.OPEN_IDLE:
            raise _state_error("configure", self._state)
        if not isinstance(settings, SweepSettings):
            raise TypeError("settings 必须是 SweepSettings")
        info = self._device_info
        if info is None:
            raise LibreVnaError("设备信息缺失（open 未成功获取 DeviceInfo）")
        self._validate_capability(settings, info)
        # 配置派生值在命令阶段前计算并启用：使 SWEEP_SETTINGS 的 ACK 共批
        # datapoint 受本次实际 pending_max 约束（不能继续用默认 1024）；
        # sweep_timeout 同属配置派生值也在此边界准备好。参数预校验已在前一步
        # 完成，未发送任何 USB 命令，也不留下半更新状态。
        self._pending_max = self._resolve_pending_max(settings)
        self._sweep_timeout_s = self._compute_sweep_timeout(settings)
        try:
            self._send_command(SET_IDLE, timeout_s=self._settings.command_timeout_s)
            self._stream.reset()
            self._pending.clear()
            self._stats.pending_depth_current = 0
            # SWEEP_SETTINGS 后设备进入连续扫频；ACK 等待期间到达的 VNADatapoint
            # 经 _wait_for_ack 进入 pending，受 _pending_max 约束（不得静默丢弃），
            # 供第一次 acquire_one 消费（不得丢失）。
            self._send_command(
                SWEEP_SETTINGS,
                payload=settings.encode(),
                timeout_s=self._settings.command_timeout_s,
            )
        except Exception:
            self._enter_faulted()
            raise
        self._assembler = ContinuousSweepAssembler(settings.points)
        self._frozen_settings = settings
        self._stats.configure_count += 1
        self._state = BackendState.STREAMING

    def acquire_one(self) -> CompleteSweep:
        """返回下一条完整有效 sweep（STREAMING 专用；不重新 open/configure）。

        先消费 pending，再执行 USB read。单次超时丢弃 partial、计数并抛
        :class:`LibreVnaSweepError`（继续重同步）；NACK / 协议错误 / pending
        溢出 / 断开 -> fail-closed 进入 :attr:`BackendState.FAULTED`。
        """
        if self._state is not BackendState.STREAMING:
            raise _state_error("acquire_one", self._state)
        if self._assembler is None:
            raise LibreVnaError("内部错误：STREAMING 状态缺少 assembler")
        deadline = time.monotonic() + self._sweep_timeout_s
        while True:
            # 1) 先消费 pending（ACK 等待期间 / 一次 read 跨两道时提前到达的点）。
            while self._pending:
                packet = self._pending.popleft()
                self._stats.pending_depth_current = len(self._pending)
                sweep = self._handle_packet_or_fault(packet)
                if sweep is not None:
                    return self._deliver(sweep)
            # 2) 读取 USB。
            try:
                data = self._transport.read(512, self._settings.usb_read_timeout_ms)
            except LibreVnaTimeoutError:
                if time.monotonic() >= deadline:
                    self._raise_sweep_timeout()
                continue
            except LibreVnaDisconnectedError:
                self._stats.disconnects += 1
                self._enter_faulted()
                raise
            completed: CompleteSweep | None = None
            for packet in self._stream.feed(data):
                sweep = self._handle_packet_or_fault(packet)
                if sweep is not None:
                    completed = sweep
            if completed is not None:
                return self._deliver(completed)
            if time.monotonic() >= deadline:
                self._raise_sweep_timeout()

    def pause(self) -> None:
        """在完整 sweep 边界发送 SET_IDLE 并清边界残留，进入 PAUSED。

        上层调用者应在上一次 ``acquire_one()`` 返回之后调用（不截断已交付的
        完整帧）。SET_IDLE 或边界清理任一失败 -> FAULTED。
        """
        if self._state is not BackendState.STREAMING:
            raise _state_error("pause", self._state)
        try:
            self._send_command(SET_IDLE, timeout_s=self._settings.command_timeout_s)
            self._discard_stale_sweep_data()
        except Exception:
            self._enter_faulted()
            raise
        self._state = BackendState.PAUSED

    def resume(self) -> None:
        """用冻结 SweepSettings 重发 SWEEP_SETTINGS，进入 STREAMING。

        不 reopen、不重建 assembler、不重置 trace_index/config。发送前清空旧
        PacketStream/pending 并重置 assembler 当前 sweep（保留统计）；ACK 成功后
        与 ACK 同批到达的首批 VNADatapoint 保留在 pending，供第一次 acquire_one
        消费。命令失败 -> FAULTED。
        """
        if self._state is not BackendState.PAUSED:
            raise _state_error("resume", self._state)
        settings = self._frozen_settings
        if settings is None:
            raise LibreVnaStateError("缺少冻结配置，无法继续采集")
        try:
            self._discard_stale_sweep_data()
            self._send_command(
                SWEEP_SETTINGS,
                payload=settings.encode(),
                timeout_s=self._settings.command_timeout_s,
            )
        except Exception:
            self._enter_faulted()
            raise
        self._state = BackendState.STREAMING

    def reconfigure(self, settings: SweepSettings) -> None:
        """PAUSED → STREAMING：用**新** SweepSettings 重配，不 reopen（第2轮 §3.4）。

        命令序列与 :meth:`configure` 相同（SET_IDLE → 清 stream/pending →
        SWEEP_SETTINGS；任一失败 -> FAULTED）。与 configure 的区别：

        - 仅接受 PAUSED（CLOSED/OPEN_IDLE/STREAMING/FAULTED 一律拒绝，
          configure 的既有拒绝语义不受影响）；
        - 重建 assembler（点数可能变化），但旧 assembler 的累计诊断统计**折叠
          保留**（completed/dropped/... 只增不减，pause/resume 的统计保留语义
          跨 reconfigure 同样成立）；
        - ``configure_count += 1``、``open_count`` / ``connection_generation`` /
          ``trace_index`` 连续（不重置、不重开连接）。
        """
        if self._state is not BackendState.PAUSED:
            raise _state_error("reconfigure", self._state)
        if not isinstance(settings, SweepSettings):
            raise TypeError("settings 必须是 SweepSettings")
        info = self._device_info
        if info is None:
            raise LibreVnaError("设备信息缺失（open 未成功获取 DeviceInfo）")
        self._validate_capability(settings, info)
        self._pending_max = self._resolve_pending_max(settings)
        self._sweep_timeout_s = self._compute_sweep_timeout(settings)
        # 先保存旧 assembler 累计统计快照——命令阶段的 _discard_stale_sweep_data
        # 与随后的 assembler 重建都会触碰它，顺序错误会把折叠基线清零。
        self._snapshot_assembler_stats()
        saved = self._saved_assembler_stats
        try:
            self._send_command(SET_IDLE, timeout_s=self._settings.command_timeout_s)
            self._discard_stale_sweep_data()
            self._send_command(
                SWEEP_SETTINGS,
                payload=settings.encode(),
                timeout_s=self._settings.command_timeout_s,
            )
        except Exception:
            self._enter_faulted()
            raise
        self._assembler = ContinuousSweepAssembler(settings.points)
        if saved is not None:
            # 折叠旧会话累计统计（只增不减）。
            self._assembler.completed_sweeps = saved["completed_sweeps"]
            self._assembler.dropped_sweeps = saved["dropped_sweeps"]
            self._assembler.incomplete_sweeps = saved["incomplete_sweeps"]
            self._assembler.duplicate_points = saved["duplicate_points"]
            self._assembler.jump_points = saved["jump_points"]
            self._assembler.out_of_range_points = saved["out_of_range_points"]
            self._assembler.invalid_points = saved["invalid_points"]
            self._saved_assembler_stats = None
        self._frozen_settings = settings
        self._stats.configure_count += 1
        self._state = BackendState.STREAMING

    def close(self) -> None:
        """尽力 SET_IDLE，始终尝试释放 USB；幂等。fail-closed 状态判定。

        依据 transport.close() 是否抛错 + ``transport.is_open`` 的最终值，
        四种结果严格为：

        - close 未抛错，is_open=False -> CLOSED，正常返回；
        - close 抛错，is_open=False -> CLOSED，抛 :class:`LibreVnaReleaseError`
          并保留原始 close 异常为 cause；
        - close 抛错，is_open=True -> FAULTED，抛 :class:`LibreVnaReleaseError`；
        - close 未抛错，is_open=True -> FAULTED，抛 :class:`LibreVnaReleaseError`
          并明确说明 close 已返回但 transport 仍报告打开（不得静默返回）。

        只有 ``is_open`` 明确为 False 才进入 CLOSED。已处于 FAULTED 时重复关闭
        仍失败：保持 FAULTED、每次抛错、faults 不重复增加。transport 恢复可关闭
        后再次 close 最终进入 CLOSED，随后 close 幂等。

        SET_IDLE 失败（设备已断开 / NACK / 超时）不阻止释放。即使逻辑状态已为
        CLOSED，只要 transport 仍报告打开，本方法也会重试释放。
        """
        if self._state is BackendState.CLOSED and not self._transport.is_open:
            return
        if self._transport.is_open:
            try:
                self._send_command(SET_IDLE, timeout_s=self._settings.command_timeout_s)
            except Exception:  # noqa: BLE001 - 尽力而为，不阻止释放
                pass
        release_error: Exception | None = None
        try:
            self._transport.close()
        except Exception as exc:  # noqa: BLE001 - 包装为 LibreVnaReleaseError
            release_error = exc
        # 本地状态清理（device_info / stream / pending / assembler / frozen config）。
        self._device_info = None
        self._reset_local_session()
        if self._transport.is_open:
            # 资源仍被 transport 持有 -> 不进入 CLOSED。
            self._transition_to_faulted()
        else:
            self._state = BackendState.CLOSED
        if release_error is not None:
            raise LibreVnaReleaseError(f"释放 LibreVNA 失败: {release_error}") from release_error
        if self._transport.is_open:
            # transport.close() 已返回但 transport 仍报告打开 -> 不得静默返回。
            raise LibreVnaReleaseError(
                "释放 LibreVNA 失败: transport.close() 已返回但 transport 仍报告打开"
                "（is_open=True）；请重试 close() 释放资源"
            )

    # ---- 命令与响应等待 ----

    def _wait_for_device_info(self, timeout_s: float) -> LibreVnaDeviceInfo:
        self._transport.write(encode_packet(REQUEST_DEVICE_INFO))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                data = self._transport.read(512, self._settings.usb_read_timeout_ms)
            except LibreVnaTimeoutError:
                continue
            info: LibreVnaDeviceInfo | None = None
            for packet in self._stream.feed(data):
                if packet.packet_type == DEVICE_INFO:
                    try:
                        info = decode_device_info(packet.payload)
                    except struct.error as exc:
                        raise LibreVnaProtocolError(
                            f"DeviceInfo payload 损坏/截断（{len(packet.payload)} 字节）"
                        ) from exc
                elif packet.packet_type == NACK:
                    raise LibreVnaNackError("设备对 RequestDeviceInfo 返回 NACK")
                else:
                    self._enqueue_pending(packet)
            if info is not None:
                return info
        raise LibreVnaTimeoutError("等待 DeviceInfo 超时")

    def _send_command(
        self, packet_type: int, *, payload: bytes = b"", timeout_s: float
    ) -> None:
        self._transport.write(encode_packet(packet_type, payload))
        self._wait_for_ack(packet_type, timeout_s=timeout_s)

    def _wait_for_ack(self, packet_type: int, *, timeout_s: float) -> None:
        """等待命令 ACK；NACK 抛错；其他非 ACK packet 进入 pending（不得丢弃）。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                data = self._transport.read(512, self._settings.usb_read_timeout_ms)
            except LibreVnaTimeoutError:
                continue
            acked = False
            for packet in self._stream.feed(data):
                if packet.packet_type == ACK:
                    acked = True
                elif packet.packet_type == NACK:
                    raise LibreVnaNackError(
                        f"设备对 packet type {packet_type} 返回 NACK"
                    )
                else:
                    self._enqueue_pending(packet)
            if acked:
                return
        raise LibreVnaTimeoutError(f"等待 ACK 超时（packet type {packet_type}）")

    # ---- 数据包处理 ----

    def _handle_packet_or_fault(self, packet: Packet) -> CompleteSweep | None:
        """处理一个 packet；采集相关的协议错误 -> fail-closed 进入 FAULTED。"""
        try:
            return self._handle_packet(packet)
        except (LibreVnaNackError, LibreVnaProtocolError, LibreVnaPendingOverflowError):
            self._enter_faulted()
            raise

    def _handle_packet(self, packet: Packet) -> CompleteSweep | None:
        """处理一个 packet；返回本次完成的一条 sweep（若有）。

        VNADatapoint 用 :func:`parse_s11_s22_point`（S11/S22 位于同一个 payload，
        不跨 packet 拼接）；parse 返回 None 走 invalid 重新同步。一条 sweep 已
        完成但未交付时，后续 datapoint 属于下一道 -> pending（不丢失、不拼接）。
        """
        if packet.packet_type == VNA_DATAPOINT:
            if self._assembler is None or self._sweep_held:
                self._enqueue_pending(packet)
                return None
            try:
                parsed = self._parse_point(packet.payload)
            except struct.error as exc:
                raise LibreVnaProtocolError(
                    f"VNA 数据包损坏/截断（payload {len(packet.payload)} 字节）"
                ) from exc
            if parsed is None:
                # reference=0 / 非有限 / 缺必需 receiver 槽位 -> invalid 重新同步
                self._assembler.mark_invalid()
                return None
            point_number, freq_hz, s11, s22 = parsed
            result = self._assembler.feed(
                S11S22Point(point_number, freq_hz, s11, s22)
            )
            if result is not None:
                self._sweep_held = True
            return result
        if packet.packet_type == NACK:
            raise LibreVnaNackError("采集中设备返回 NACK")
        if packet.packet_type == ACK:
            self._stats.unexpected_acks += 1
            return None
        self._stats.ignored_packets += 1
        return None

    def _parse_point(self, payload: bytes) -> tuple[int, float, complex, complex] | None:
        """按当前扫法解析一个数据点，统一成 (点号, 频率, S11, S22)。

        单端口扫法（0x1240 只采 S11 / 0x1208 只采 S22）里没测的那一路填 0；
        上层按所选通道只保留测到的那一路，占位值不会进入处理/保存。
        """
        bitmap = (
            self._frozen_settings.stages_bitmap
            if self._frozen_settings is not None
            else S11_S22_STAGES_BITMAP
        )
        if bitmap == S11_STAGES_BITMAP:
            single = parse_s11_point(payload)
            if single is None:
                return None
            return single[0], single[1], single[2], 0j
        if bitmap == S22_STAGES_BITMAP:
            single = parse_s22_point(payload)
            if single is None:
                return None
            return single[0], single[1], 0j, single[2]
        return parse_s11_s22_point(payload)

    def _enqueue_pending(self, packet: Packet) -> None:
        """把 packet 加入 pending；溢出 fail-closed（抛溢出错误）。"""
        if len(self._pending) >= self._pending_max:
            self._stats.pending_overflows += 1
            raise LibreVnaPendingOverflowError(
                f"pending 队列溢出（上限 {self._pending_max}），fail-closed"
            )
        self._pending.append(packet)
        depth = len(self._pending)
        self._stats.pending_depth_current = depth
        if depth > self._stats.pending_depth_peak:
            self._stats.pending_depth_peak = depth

    # ---- 结果交付 ----

    def _deliver(self, sweep: CompleteSweep) -> CompleteSweep:
        """把一条已组装 sweep 交付给调用方（赋 trace_index、清 held 标记）。"""
        self._sweep_held = False
        delivered = CompleteSweep(
            frequency_hz=sweep.frequency_hz,
            raw=sweep.raw,
            trace_index=self._stats.trace_index,
            statistics=sweep.statistics,
        )
        self._stats.trace_index += 1
        return delivered

    # ---- 校验与超时 ----

    def _validate_capability(
        self, settings: SweepSettings, info: LibreVnaDeviceInfo
    ) -> None:
        """在**任何 USB 写入前**校验设备能力；不支持即抛错（不破坏当前会话）。"""
        if settings.stages_bitmap not in (
            S11_S22_STAGES_BITMAP,
            S11_STAGES_BITMAP,
            S22_STAGES_BITMAP,
        ):
            raise LibreVnaUnsupportedConfigError(
                f"PersistentLibreVnaBackend 仅支持双反射 0x1241、只采 S11 0x1240、"
                f"只采 S22 0x1208（实验）；收到 {settings.stages_bitmap:#x}。"
            )
        if settings.stages_bitmap != S11_STAGES_BITMAP and info.num_ports < 2:
            raise LibreVnaUnsupportedConfigError(
                f"双反射 S11/S22 需要设备 num_ports>=2，实际 {info.num_ports}"
            )
        if settings.start_hz < info.min_freq_hz or settings.stop_hz > info.max_freq_hz:
            raise LibreVnaUnsupportedConfigError(
                f"设备频率范围 {info.min_freq_hz / 1e9:g}–{info.max_freq_hz / 1e9:g} GHz，"
                f"不支持 {settings.start_hz / 1e9:g}–{settings.stop_hz / 1e9:g} GHz"
            )
        if settings.ifbw_hz < info.min_ifbw_hz or settings.ifbw_hz > info.max_ifbw_hz:
            raise LibreVnaUnsupportedConfigError(
                f"设备 IFBW 范围 {info.min_ifbw_hz:g}–{info.max_ifbw_hz:g} Hz，"
                f"不支持 {settings.ifbw_hz:g} Hz"
            )
        if settings.points > info.max_points:
            raise LibreVnaUnsupportedConfigError(
                f"设备最多支持 {info.max_points} 点，收到 {settings.points}"
            )

    def _resolve_pending_max(self, settings: SweepSettings) -> int:
        if self._settings.pending_max is not None:
            return self._settings.pending_max
        return max(PENDING_SLOTS_PER_POINT * settings.points, MIN_PENDING_SLOTS)

    def _compute_sweep_timeout(self, settings: SweepSettings) -> float:
        if self._settings.sweep_timeout_s is not None:
            return self._settings.sweep_timeout_s
        # 双反射每频点激励两次（stage0 + stage1），按 2 倍估算，不硬编码机器速度。
        n_stages = 2
        expected_s = settings.points * n_stages / max(settings.ifbw_hz, 1.0)
        return max(DEFAULT_SWEEP_TIMEOUT_MIN_S, expected_s * 5.0)

    def _raise_sweep_timeout(self) -> NoReturn:
        """单次 sweep 超时：原子边界清理后抛 :class:`LibreVnaSweepError`。

        保持 STREAMING、保留 transport/冻结配置/已交付统计；丢弃当前未完成
        sweep（并累计 dropped/incomplete）、清空 PacketStream 残留半包字节和
        pending 队列、清除 held/stale 状态——保证下一次 acquire_one 从全新的
        完整包边界和新 sweep 开始，绝不把 timeout 前的包头与 timeout 后的包尾
        拼成一个 datapoint。不重置 trace_index 和已完成的会话统计。
        """
        self._stats.sweep_timeouts += 1
        if self._assembler is not None:
            if self._assembler._active:  # noqa: SLF001 - 同模块；timeout 丢弃 partial
                self._assembler.dropped_sweeps += 1
                self._assembler.incomplete_sweeps += 1
            self._assembler.reset_current()
        # 清空 PacketStream 内所有残留字节（含半个 VNADatapoint）。
        self._stream.reset()
        # 清空 pending 队列并同步 pending 深度统计。
        self._pending.clear()
        self._stats.pending_depth_current = 0
        # 清除与当前 sweep 有关的 held/stale 状态。
        self._sweep_held = False
        n_points = self._frozen_settings.points if self._frozen_settings else 0
        stats = self.statistics
        raise LibreVnaSweepError(
            f"无法在 {self._sweep_timeout_s:g}s 内得到完整一道（{n_points} 点）；"
            f"已丢弃 {stats.dropped_sweeps} 道、重复点 {stats.duplicate_points}、"
            f"越界点 {stats.out_of_range_points}、无效点 {stats.invalid_points}"
        )

    # ---- 状态清理 ----

    def _snapshot_assembler_stats(self) -> None:
        """assembler 消失前保存其最终统计快照（供 close/FAULTED 后读取）。"""
        assembler = self._assembler
        if assembler is None:
            return
        self._saved_assembler_stats = {
            "completed_sweeps": assembler.completed_sweeps,
            "dropped_sweeps": assembler.dropped_sweeps,
            "incomplete_sweeps": assembler.incomplete_sweeps,
            "duplicate_points": assembler.duplicate_points,
            "jump_points": assembler.jump_points,
            "out_of_range_points": assembler.out_of_range_points,
            "invalid_points": assembler.invalid_points,
        }

    def _reset_local_session(self) -> None:
        """清空本地会话状态（stream/pending/assembler/冻结配置），保留统计。

        assembler 消失前先保存其最终统计快照（``_saved_assembler_stats``），
        使正常 close() 或进入 FAULTED 后仍可读取刚结束会话的完整统计。
        """
        self._snapshot_assembler_stats()
        self._stream.reset()
        self._pending.clear()
        self._stats.pending_depth_current = 0
        self._assembler = None
        self._frozen_settings = None
        self._sweep_held = False
        self._pending_max = MIN_PENDING_SLOTS
        self._sweep_timeout_s = DEFAULT_SWEEP_TIMEOUT_MIN_S

    def _reset_stats(self) -> None:
        """重置会话统计（保留跨会话累计的 open_count / connection_generation；
        同时清空上一会话保存的 assembler 最终统计快照）。只有成功的 open() 开始
        新会话时才调用。"""
        self._stats = SessionStatistics(
            open_count=self._stats.open_count,
            connection_generation=self._stats.connection_generation,
        )
        self._saved_assembler_stats = None

    def _discard_stale_sweep_data(self) -> None:
        """丢弃暂停/恢复边界的陈旧或部分 sweep 数据，保留累计统计与模式状态。

        transport 保持打开；不重置 trace_index/config/assembler 模式
        （n_points/统计保留，只清当前组装态）。
        """
        self._stream.reset()
        self._pending.clear()
        self._stats.pending_depth_current = 0
        self._sweep_held = False
        if self._assembler is not None:
            self._assembler.reset_current()

    def _enter_faulted(self) -> None:
        """命令或设备状态不可确定：清本地配置，进入 FAULTED（只允许 close/reopen）。"""
        self._reset_local_session()
        self._transition_to_faulted()

    def _transition_to_faulted(self) -> None:
        """进入 FAULTED；仅在从非 FAULTED 状态进入时增加 ``faults`` 计数。

        避免连续释放失败等场景对同一故障事件重复计数（例如 open 失败清理后
        transport 仍打开，随后 close() 再次失败——faults 只计一次）。
        """
        if self._state is not BackendState.FAULTED:
            self._stats.faults += 1
        self._state = BackendState.FAULTED


__all__ = [
    "BackendState",
    "CompleteSweep",
    "ContinuousSweepAssembler",
    "DEFAULT_COMMAND_TIMEOUT_S",
    "DEFAULT_DEVICE_INFO_TIMEOUT_S",
    "DEFAULT_SWEEP_TIMEOUT_MIN_S",
    "DEFAULT_USB_READ_TIMEOUT_MS",
    "InvalidPoint",
    "MIN_PENDING_SLOTS",
    "PENDING_SLOTS_PER_POINT",
    "PersistentBackendSettings",
    "PersistentLibreVnaBackend",
    "S11S22Point",
    "SessionStatistics",
    "StreamEvent",
]
