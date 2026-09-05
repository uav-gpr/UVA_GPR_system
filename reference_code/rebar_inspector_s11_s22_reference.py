"""S11/S22 continuous acquisition and dual-reflection OSL reference.

This module distils the useful design from ``E:\\钢筋仪软件开发`` without
depending on that project.  It is deliberately split at the transport boundary:
the current project's LibreVNA packet writer/reader can be adapted to
``ContinuousDualReflectionTransport`` while the state machine and calibration
math can be tested without USB hardware.

Channel order is fixed throughout this module::

    channel 0 = HH:S11 = stage 0 Port1 receiver / stage 0 reference
    channel 1 = VV:S22 = stage 1 Port2 receiver / stage 1 reference

This is dual *reflection* acquisition and calibration.  It is not full SOLT and
does not calibrate S21 or S12.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable, Protocol, Sequence, TypeAlias

import numpy as np


S11_S22_STAGES_BITMAP = 0x1241
MIN_DENOMINATOR = 1.0e-12

_DESC_STAGE_SHIFT = 5
_DESC_MASK_REFERENCE = 0x10
_DESC_MASK_PORT1 = 0x01
_DESC_MASK_PORT2 = 0x02


class AcquisitionReferenceError(RuntimeError):
    """Base error for the transport-independent acquisition reference."""


class SweepTimeoutError(AcquisitionReferenceError):
    """A complete, valid sweep was not assembled before the deadline."""


class CalibrationReferenceError(ValueError):
    """Invalid or numerically singular calibration input."""


@dataclass(frozen=True)
class SweepConfig:
    """Configuration for continuous two-stage reflection acquisition."""

    start_hz: int
    stop_hz: int
    n_points: int
    ifbw_hz: int
    power_dbm: float = -10.0
    dwell_us: int = 0
    usb_read_timeout_ms: int = 50
    sweep_timeout_s: float | None = None

    def __post_init__(self) -> None:
        if self.start_hz <= 0 or self.stop_hz <= self.start_hz:
            raise ValueError("require 0 < start_hz < stop_hz")
        if (
            isinstance(self.n_points, bool)
            or not isinstance(self.n_points, int)
            or self.n_points < 2
        ):
            raise ValueError("n_points must be an integer >= 2")
        if (
            isinstance(self.ifbw_hz, bool)
            or not isinstance(self.ifbw_hz, int)
            or self.ifbw_hz <= 0
        ):
            raise ValueError("ifbw_hz must be a positive integer")
        if not math.isfinite(float(self.power_dbm)):
            raise ValueError("power_dbm must be finite")
        if isinstance(self.dwell_us, bool) or not isinstance(self.dwell_us, int) or self.dwell_us < 0:
            raise ValueError("dwell_us must be >= 0")
        if (
            isinstance(self.usb_read_timeout_ms, bool)
            or not isinstance(self.usb_read_timeout_ms, int)
            or self.usb_read_timeout_ms <= 0
        ):
            raise ValueError("usb_read_timeout_ms must be > 0")
        if self.sweep_timeout_s is not None and self.sweep_timeout_s <= 0:
            raise ValueError("sweep_timeout_s must be > 0 when supplied")

    @property
    def effective_sweep_timeout_s(self) -> float:
        """Five-times safety margin, accounting for the two active stages."""

        if self.sweep_timeout_s is not None:
            return float(self.sweep_timeout_s)
        expected_s = self.n_points * 2 / float(self.ifbw_hz)
        return max(2.0, expected_s * 5.0)


@dataclass(frozen=True)
class DualReflectionPoint:
    """One frequency point containing both reflections from one datapoint."""

    point_number: int
    frequency_hz: float
    s11: complex
    s22: complex


@dataclass(frozen=True)
class InvalidPoint:
    """Marker emitted by a transport when a datapoint cannot be decoded safely."""

    point_number: int | None = None
    reason: str = "invalid or incomplete S11/S22 datapoint"


StreamEvent: TypeAlias = DualReflectionPoint | InvalidPoint


@dataclass(frozen=True)
class DualReflectionSweep:
    """One complete sweep; ``raw`` has shape ``(2, frequency)``."""

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


def decode_s11_s22_receivers(
    point_number: int,
    frequency_hz: float,
    receivers: Iterable[tuple[int, complex]],
) -> DualReflectionPoint:
    """Decode S11/S22 from already-unpacked LibreVNA receiver descriptors.

    Required fields must occur exactly once in the *same* VNADatapoint:

    - S11: stage-0 Port-1 receiver / stage-0 reference;
    - S22: stage-1 Port-2 receiver / stage-1 reference.

    Missing, duplicate, zero-reference or non-finite values are rejected.  The
    caller should turn the exception into :class:`InvalidPoint` so the current
    whole sweep is discarded rather than patched with stale data.
    """

    slots: dict[str, list[complex]] = {
        "s11_receiver": [],
        "s11_reference": [],
        "s22_receiver": [],
        "s22_reference": [],
    }
    for descriptor, value in receivers:
        if not isinstance(descriptor, int):
            raise AcquisitionReferenceError("receiver descriptor must be int")
        stage = descriptor >> _DESC_STAGE_SHIFT
        is_reference = bool(descriptor & _DESC_MASK_REFERENCE)
        if stage == 0:
            if is_reference:
                slots["s11_reference"].append(complex(value))
            elif descriptor & _DESC_MASK_PORT1:
                slots["s11_receiver"].append(complex(value))
        elif stage == 1:
            if is_reference:
                slots["s22_reference"].append(complex(value))
            elif descriptor & _DESC_MASK_PORT2:
                slots["s22_receiver"].append(complex(value))

    bad_counts = {name: len(values) for name, values in slots.items() if len(values) != 1}
    if bad_counts:
        raise AcquisitionReferenceError(
            f"S11/S22 receiver fields must occur exactly once: {bad_counts}"
        )

    s11_reference = slots["s11_reference"][0]
    s22_reference = slots["s22_reference"][0]
    if abs(s11_reference) == 0 or abs(s22_reference) == 0:
        raise AcquisitionReferenceError("S11/S22 reference receiver is zero")
    s11 = slots["s11_receiver"][0] / s11_reference
    s22 = slots["s22_receiver"][0] / s22_reference
    if not _finite_complex(s11) or not _finite_complex(s22):
        raise AcquisitionReferenceError("decoded S11/S22 contains NaN or infinity")
    if not math.isfinite(float(frequency_hz)):
        raise AcquisitionReferenceError("frequency_hz must be finite")
    return DualReflectionPoint(int(point_number), float(frequency_hz), s11, s22)


class ContinuousSweepAssembler:
    """Strictly assemble ``0..n_points-1`` without joining different sweeps."""

    def __init__(self, n_points: int) -> None:
        if isinstance(n_points, bool) or not isinstance(n_points, int) or n_points < 2:
            raise ValueError("n_points must be an integer >= 2")
        self.n_points = n_points
        self.dropped_sweeps = 0
        self.incomplete_sweeps = 0
        self.duplicate_points = 0
        self.out_of_range_points = 0
        self.invalid_points = 0
        self._active = False
        self._next_expected = 0
        self._points: list[DualReflectionPoint] = []
        self._unsynchronised_drop_counted = False

    @property
    def statistics(self) -> dict[str, int]:
        return {
            "dropped_sweeps": self.dropped_sweeps,
            "incomplete_sweeps": self.incomplete_sweeps,
            "duplicate_points": self.duplicate_points,
            "out_of_range_points": self.out_of_range_points,
            "invalid_points": self.invalid_points,
        }

    def reset_current(self) -> None:
        """Drop local partial state while preserving cumulative diagnostics."""

        self._active = False
        self._next_expected = 0
        self._points = []
        self._unsynchronised_drop_counted = False

    def mark_invalid(self, _point_number: int | None = None) -> None:
        self.invalid_points += 1
        self._discard_or_mark_unsynchronised()

    def feed(self, point: DualReflectionPoint) -> DualReflectionSweep | None:
        if not isinstance(point, DualReflectionPoint):
            raise TypeError("point must be DualReflectionPoint")
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
        result = DualReflectionSweep(frequency, raw, statistics=self.statistics)
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


class ContinuousDualReflectionTransport(Protocol):
    """USB-specific adapter required by :class:`PersistentDualReflectionAcquirer`.

    ``start_continuous`` must preserve any datapoints received while it waits for
    the command ACK and return them from a later ``read_events`` call.
    """

    def open(self) -> None: ...

    def set_idle(self) -> None: ...

    def start_continuous(self, config: SweepConfig, stages_bitmap: int) -> None: ...

    def read_events(self, timeout_ms: int) -> Iterable[StreamEvent]: ...

    def close(self) -> None: ...


class PersistentDualReflectionAcquirer:
    """Open/configure once, then consume one complete sweep per call."""

    def __init__(
        self,
        transport: ContinuousDualReflectionTransport,
        *,
        monotonic=time.monotonic,
    ) -> None:
        self._transport = transport
        self._monotonic = monotonic
        self._open = False
        self._config: SweepConfig | None = None
        self._assembler: ContinuousSweepAssembler | None = None
        self._pending: deque[StreamEvent] = deque()
        self._trace_index = 0

    def open(self) -> None:
        if self._open:
            raise AcquisitionReferenceError("session is already open")
        self._transport.open()
        try:
            self._transport.set_idle()
        except Exception:
            self._transport.close()
            raise
        self._open = True

    def configure(self, config: SweepConfig) -> None:
        if not self._open:
            raise AcquisitionReferenceError("open the session before configure")
        self._transport.set_idle()
        self._pending.clear()
        self._assembler = ContinuousSweepAssembler(config.n_points)
        self._transport.start_continuous(config, S11_S22_STAGES_BITMAP)
        self._config = config
        self._trace_index = 0

    def acquire_one(self) -> DualReflectionSweep:
        config = self._config
        assembler = self._assembler
        if not self._open or config is None or assembler is None:
            raise AcquisitionReferenceError("open and configure before acquisition")

        deadline = self._monotonic() + config.effective_sweep_timeout_s
        while self._monotonic() < deadline:
            while self._pending:
                result = self._consume(self._pending.popleft())
                if result is not None:
                    return self._number_sweep(result)

            events = list(self._transport.read_events(config.usb_read_timeout_ms))
            for index, event in enumerate(events):
                result = self._consume(event)
                if result is not None:
                    self._pending.extend(events[index + 1 :])
                    return self._number_sweep(result)

        stats = assembler.statistics
        raise SweepTimeoutError(
            f"no complete {config.n_points}-point sweep before timeout; stats={stats}"
        )

    def pause(self) -> None:
        if self._config is None or self._assembler is None:
            raise AcquisitionReferenceError("session is not configured")
        self._transport.set_idle()
        self._pending.clear()
        self._assembler.reset_current()

    def resume(self) -> None:
        if self._config is None or self._assembler is None:
            raise AcquisitionReferenceError("session is not configured")
        self._pending.clear()
        self._assembler.reset_current()
        self._transport.start_continuous(self._config, S11_S22_STAGES_BITMAP)

    def close(self) -> None:
        if not self._open:
            return
        idle_error: Exception | None = None
        try:
            self._transport.set_idle()
        except Exception as exc:  # release resources even when SET_IDLE fails
            idle_error = exc
        finally:
            self._transport.close()
            self._open = False
            self._config = None
            self._assembler = None
            self._pending.clear()
        if idle_error is not None:
            raise idle_error

    def __enter__(self) -> "PersistentDualReflectionAcquirer":
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def _consume(self, event: StreamEvent) -> DualReflectionSweep | None:
        assert self._assembler is not None
        if isinstance(event, InvalidPoint):
            self._assembler.mark_invalid(event.point_number)
            return None
        return self._assembler.feed(event)

    def _number_sweep(self, sweep: DualReflectionSweep) -> DualReflectionSweep:
        numbered = DualReflectionSweep(
            sweep.frequency_hz,
            sweep.raw,
            trace_index=self._trace_index,
            statistics=sweep.statistics,
        )
        self._trace_index += 1
        return numbered


def collect_repeated_sweeps(
    acquirer: PersistentDualReflectionAcquirer,
    capture_count: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Collect coherent calibration repeats as ``(capture, channel, frequency)``."""

    if (
        isinstance(capture_count, bool)
        or not isinstance(capture_count, int)
        or capture_count < 1
    ):
        raise ValueError("capture_count must be an integer >= 1")
    captures: list[np.ndarray] = []
    frequency: np.ndarray | None = None
    for _ in range(capture_count):
        sweep = acquirer.acquire_one()
        if frequency is None:
            frequency = sweep.frequency_hz
        elif not np.array_equal(frequency, sweep.frequency_hz):
            raise AcquisitionReferenceError("frequency grid changed between captures")
        captures.append(sweep.raw)
    assert frequency is not None
    return frequency.copy(), np.asarray(captures, dtype=np.complex128)


@dataclass(frozen=True)
class OnePortOslProfile:
    """One-port three-term error model for exactly one reflection channel."""

    frequency_hz: np.ndarray
    directivity: np.ndarray
    reflection_tracking: np.ndarray
    source_match: np.ndarray
    s_parameter: str
    quality_metrics: dict[str, float]
    capture_counts: tuple[int, int, int]

    def __post_init__(self) -> None:
        frequency = _frequency_axis(self.frequency_hz)
        if self.s_parameter not in {"S11", "S22"}:
            raise ValueError("s_parameter must be 'S11' or 'S22'")
        terms = []
        for name, values in (
            ("directivity", self.directivity),
            ("reflection_tracking", self.reflection_tracking),
            ("source_match", self.source_match),
        ):
            term = _complex_array(values, name)
            if term.shape != frequency.shape:
                raise ValueError(f"{name} must match frequency_hz")
            terms.append(_readonly(term))
        object.__setattr__(self, "frequency_hz", _readonly(frequency))
        object.__setattr__(self, "directivity", terms[0])
        object.__setattr__(self, "reflection_tracking", terms[1])
        object.__setattr__(self, "source_match", terms[2])
        object.__setattr__(self, "quality_metrics", dict(self.quality_metrics))

    def correct(self, frequency_hz: Sequence[float], measured: np.ndarray) -> np.ndarray:
        requested = _frequency_axis(frequency_hz)
        measured_array = _complex_array(measured, "measured")
        if measured_array.ndim == 0 or measured_array.shape[-1] != requested.size:
            raise CalibrationReferenceError(
                "measured's last dimension must match frequency_hz"
            )
        directivity = _interpolate_complex(requested, self.frequency_hz, self.directivity)
        tracking = _interpolate_complex(
            requested, self.frequency_hz, self.reflection_tracking
        )
        source_match = _interpolate_complex(
            requested, self.frequency_hz, self.source_match
        )
        return _apply_error_model(
            measured_array, directivity, tracking, source_match, requested
        )


@dataclass(frozen=True)
class DualReflectionOslProfile:
    """Two independent one-port profiles in fixed ``(S11, S22)`` order."""

    s11: OnePortOslProfile
    s22: OnePortOslProfile

    def __post_init__(self) -> None:
        if self.s11.s_parameter != "S11" or self.s22.s_parameter != "S22":
            raise CalibrationReferenceError("profile order must be (S11, S22)")
        if not np.array_equal(self.s11.frequency_hz, self.s22.frequency_hz):
            raise CalibrationReferenceError(
                "S11 and S22 calibration frequency grids must be exactly equal"
            )

    @property
    def frequency_hz(self) -> np.ndarray:
        return self.s11.frequency_hz

    def correct(self, frequency_hz: Sequence[float], measured: np.ndarray) -> np.ndarray:
        array = _complex_array(measured, "measured")
        requested = _frequency_axis(frequency_hz)
        if array.ndim < 2 or array.shape[-2:] != (2, requested.size):
            raise CalibrationReferenceError(
                f"measured must end in (channel=2, frequency={requested.size})"
            )
        corrected = np.stack(
            (
                self.s11.correct(requested, array[..., 0, :]),
                self.s22.correct(requested, array[..., 1, :]),
            ),
            axis=-2,
        )
        return _readonly(np.ascontiguousarray(corrected, dtype=np.complex128))


def build_one_port_osl(
    frequency_hz: Sequence[float],
    open_measured: np.ndarray,
    short_measured: np.ndarray,
    load_measured: np.ndarray,
    *,
    s_parameter: str,
    open_actual: complex | Sequence[complex] = 1.0 + 0.0j,
    short_actual: complex | Sequence[complex] = -1.0 + 0.0j,
    load_actual: complex | Sequence[complex] = 0.0 + 0.0j,
) -> OnePortOslProfile:
    """Average coherent repeats, solve D/T/M, and report fit/repeat errors."""

    frequency = _frequency_axis(frequency_hz)
    open_captures = _capture_array(open_measured, "open_measured", frequency.size)
    short_captures = _capture_array(short_measured, "short_measured", frequency.size)
    load_captures = _capture_array(load_measured, "load_measured", frequency.size)
    open_mean = np.mean(open_captures, axis=0)
    short_mean = np.mean(short_captures, axis=0)
    load_mean = np.mean(load_captures, axis=0)
    open_standard = _standard_array(open_actual, frequency.size, "open_actual")
    short_standard = _standard_array(short_actual, frequency.size, "short_actual")
    load_standard = _standard_array(load_actual, frequency.size, "load_actual")

    directivity, tracking, source_match = _solve_osl(
        open_mean,
        short_mean,
        load_mean,
        open_standard,
        short_standard,
        load_standard,
        frequency,
    )
    corrected_standards = (
        _apply_error_model(open_mean, directivity, tracking, source_match, frequency),
        _apply_error_model(short_mean, directivity, tracking, source_match, frequency),
        _apply_error_model(load_mean, directivity, tracking, source_match, frequency),
    )
    metrics: dict[str, float] = {}
    for label, corrected, actual, captures in zip(
        ("open", "short", "load"),
        corrected_standards,
        (open_standard, short_standard, load_standard),
        (open_captures, short_captures, load_captures),
    ):
        metrics[f"{label}_fit_max_abs_error"] = float(np.max(np.abs(corrected - actual)))
        corrected_repeats = _apply_error_model(
            captures, directivity, tracking, source_match, frequency
        )
        repeat_errors = np.abs(corrected_repeats - actual)
        metrics[f"{label}_capture_rms_abs_error"] = float(
            np.sqrt(np.mean(np.square(repeat_errors)))
        )
        metrics[f"{label}_capture_max_abs_error"] = float(np.max(repeat_errors))

    return OnePortOslProfile(
        frequency,
        directivity,
        tracking,
        source_match,
        s_parameter,
        metrics,
        (open_captures.shape[0], short_captures.shape[0], load_captures.shape[0]),
    )


def build_dual_reflection_osl(
    frequency_hz: Sequence[float],
    *,
    p1_open_s11: np.ndarray,
    p1_short_s11: np.ndarray,
    p1_load_s11: np.ndarray,
    p2_open_s22: np.ndarray,
    p2_short_s22: np.ndarray,
    p2_load_s22: np.ndarray,
) -> DualReflectionOslProfile:
    """Build independent profiles from the physical six-step calibration.

    During P1 standards, P2 must be terminated with a matched load.  During P2
    standards, P1 must be terminated with a matched load.  Inputs may be one
    sweep ``(frequency,)`` or coherent repeats ``(capture, frequency)``.
    """

    s11 = build_one_port_osl(
        frequency_hz,
        p1_open_s11,
        p1_short_s11,
        p1_load_s11,
        s_parameter="S11",
    )
    s22 = build_one_port_osl(
        frequency_hz,
        p2_open_s22,
        p2_short_s22,
        p2_load_s22,
        s_parameter="S22",
    )
    return DualReflectionOslProfile(s11, s22)


def _solve_osl(
    open_mean: np.ndarray,
    short_mean: np.ndarray,
    load_mean: np.ndarray,
    open_actual: np.ndarray,
    short_actual: np.ndarray,
    load_actual: np.ndarray,
    frequency_hz: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lc, oc, sc = load_actual, open_actual, short_actual
    lm, om, sm = load_mean, open_mean, short_mean
    denominator = lc * oc * (om - lm) + lc * sc * (lm - sm) + oc * sc * (sm - om)
    if np.any(np.abs(denominator) < MIN_DENOMINATOR):
        bad = int(np.argmin(np.abs(denominator)))
        raise CalibrationReferenceError(
            f"OSL solve is singular near {frequency_hz[bad]:g} Hz"
        )
    directivity = (
        lc * om * (sm * (oc - sc) + lm * sc)
        - lc * oc * lm * sm
        + oc * lm * sc * (sm - om)
    ) / denominator
    source_match = (lc * (om - sm) + oc * (sm - lm) + sc * (lm - om)) / denominator
    delta = (
        lc * lm * (om - sm) + oc * om * (sm - lm) + sc * sm * (lm - om)
    ) / denominator
    reflection_tracking = directivity * source_match - delta
    return directivity, reflection_tracking, source_match


def _apply_error_model(
    measured: np.ndarray,
    directivity: np.ndarray,
    reflection_tracking: np.ndarray,
    source_match: np.ndarray,
    frequency_hz: np.ndarray,
) -> np.ndarray:
    x = measured - directivity
    denominator = reflection_tracking + source_match * x
    if np.any(np.abs(denominator) < MIN_DENOMINATOR):
        flat_index = int(np.argmin(np.abs(denominator)))
        frequency_index = np.unravel_index(flat_index, denominator.shape)[-1]
        raise CalibrationReferenceError(
            f"correction is singular near {frequency_hz[frequency_index]:g} Hz"
        )
    return _readonly(np.asarray(x / denominator, dtype=np.complex128))


def _capture_array(values: np.ndarray, name: str, n_frequency: int) -> np.ndarray:
    array = _complex_array(values, name)
    if array.ndim == 1:
        array = array[np.newaxis, :]
    if array.ndim != 2 or array.shape[1] != n_frequency or array.shape[0] < 1:
        raise CalibrationReferenceError(
            f"{name} must have shape (frequency,) or (capture, frequency)"
        )
    return array


def _standard_array(
    values: complex | Sequence[complex], n_frequency: int, name: str
) -> np.ndarray:
    array = np.asarray(values)
    if array.ndim == 0:
        result = np.full(n_frequency, complex(array.item()), dtype=np.complex128)
    else:
        result = _complex_array(array, name)
        if result.shape != (n_frequency,):
            raise CalibrationReferenceError(f"{name} must be scalar or one value per frequency")
    return result


def _interpolate_complex(
    requested: np.ndarray, source_x: np.ndarray, source_y: np.ndarray
) -> np.ndarray:
    if requested[0] < source_x[0] or requested[-1] > source_x[-1]:
        raise CalibrationReferenceError("calibration extrapolation is forbidden")
    real = np.interp(requested, source_x, source_y.real)
    imag = np.interp(requested, source_x, source_y.imag)
    return real + 1j * imag


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


def _finite_complex(value: complex) -> bool:
    return math.isfinite(value.real) and math.isfinite(value.imag)


def _valid_point_values(point: DualReflectionPoint) -> bool:
    return (
        isinstance(point.point_number, int)
        and not isinstance(point.point_number, bool)
        and math.isfinite(point.frequency_hz)
        and _finite_complex(point.s11)
        and _finite_complex(point.s22)
    )


__all__ = [
    "AcquisitionReferenceError",
    "CalibrationReferenceError",
    "ContinuousDualReflectionTransport",
    "ContinuousSweepAssembler",
    "DualReflectionOslProfile",
    "DualReflectionPoint",
    "DualReflectionSweep",
    "InvalidPoint",
    "OnePortOslProfile",
    "PersistentDualReflectionAcquirer",
    "S11_S22_STAGES_BITMAP",
    "StreamEvent",
    "SweepConfig",
    "SweepTimeoutError",
    "build_dual_reflection_osl",
    "build_one_port_osl",
    "collect_repeated_sweeps",
    "decode_s11_s22_receivers",
]
