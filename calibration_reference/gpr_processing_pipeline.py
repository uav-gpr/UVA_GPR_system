"""Reusable S11 processing pipeline for the UAV-GPR workflow.

The stages are deliberately kept separate:

    raw complex S11
        -> one-port OSL correction
        -> complex background subtraction
        -> optional frequency-domain mean removal
        -> zero padding and IFFT
        -> real time-domain trace

Background subtraction and mean removal are not VNA calibration.  Each stage
is returned in :class:`ProcessingResult` so another application can display,
save, and compare the intermediate data.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np

try:
    from .osl_calibration import OSLCalibration, complex_interpolate
except ImportError:  # Allows direct import when this folder is on sys.path.
    from osl_calibration import OSLCalibration, complex_interpolate


BACKGROUND_FORMAT = "uav_gpr_complex_background_v1"


def _frequency_vector(values: Any, name: str = "frequency_hz") -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 1 or array.size < 2:
        raise ValueError(f"{name} must contain at least two frequency points")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains a non-finite value")
    if np.any(np.diff(array) <= 0):
        raise ValueError(f"{name} must be strictly increasing")
    return array


def _complex_vector(values: Any, name: str, count: int) -> np.ndarray:
    array = np.asarray(values, dtype=np.complex128)
    if array.ndim != 1 or array.size != count:
        raise ValueError(f"{name} must contain {count} complex points")
    if not np.all(np.isfinite(array.real)) or not np.all(np.isfinite(array.imag)):
        raise ValueError(f"{name} contains a non-finite value")
    return array


def _complex_payload(values: np.ndarray) -> dict[str, list[float]]:
    return {
        "real": values.real.astype(float).tolist(),
        "imag": values.imag.astype(float).tolist(),
    }


def _complex_from_payload(payload: dict[str, Any], name: str) -> np.ndarray:
    try:
        real = np.asarray(payload["real"], dtype=float)
        imag = np.asarray(payload["imag"], dtype=float)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid complex payload for {name}") from exc
    if real.shape != imag.shape:
        raise ValueError(f"real and imaginary lengths differ for {name}")
    return real + 1j * imag


@dataclass
class BackgroundReference:
    """A complex S11 background measured after the active OSL correction.

    The background should be captured with the same VNA settings, cables,
    antenna, reference plane, and calibration used for the DUT measurement.
    Multiple captures can be averaged with :meth:`from_raw_captures`.
    """

    frequency_hz: np.ndarray
    s11: np.ndarray
    capture_count: int = 1
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.frequency_hz = _frequency_vector(self.frequency_hz)
        self.s11 = _complex_vector(
            self.s11, "background s11", self.frequency_hz.size
        )
        self.capture_count = int(self.capture_count)
        if self.capture_count < 1:
            raise ValueError("capture_count must be at least 1")

    @classmethod
    def from_raw_captures(
        cls,
        frequency_hz: Any,
        raw_s11_captures: Sequence[Any] | np.ndarray,
        *,
        calibration: OSLCalibration | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> "BackgroundReference":
        """Calibrate and coherently average one or more background sweeps."""

        frequency = _frequency_vector(frequency_hz)
        captures = np.asarray(raw_s11_captures, dtype=np.complex128)
        if captures.ndim == 1:
            captures = captures[np.newaxis, :]
        if captures.ndim != 2 or captures.shape[1] != frequency.size:
            raise ValueError(
                "raw_s11_captures must have shape (captures, frequency_points)"
            )
        if captures.shape[0] < 1:
            raise ValueError("at least one background capture is required")
        if not np.all(np.isfinite(captures.real)) or not np.all(
            np.isfinite(captures.imag)
        ):
            raise ValueError("raw_s11_captures contains a non-finite value")

        if calibration is not None:
            calibrated = np.vstack(
                [calibration.correct(frequency, trace) for trace in captures]
            )
        else:
            calibrated = captures.copy()

        result_metadata = {
            "domain": "complex S11 after active OSL correction",
            "averaging": "coherent complex average",
        }
        if metadata:
            result_metadata.update(metadata)
        return cls(
            frequency_hz=frequency,
            s11=np.mean(calibrated, axis=0),
            capture_count=calibrated.shape[0],
            metadata=result_metadata,
        )

    def values_at(self, frequency_hz: Any) -> np.ndarray:
        """Return the background on a requested in-band frequency grid."""

        return complex_interpolate(frequency_hz, self.frequency_hz, self.s11)

    def save_json(self, filename: str | Path) -> None:
        path = Path(filename)
        payload = {
            "format": BACKGROUND_FORMAT,
            "frequency_hz": self.frequency_hz.astype(float).tolist(),
            "s11": _complex_payload(self.s11),
            "capture_count": self.capture_count,
            "metadata": self.metadata,
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    @classmethod
    def load_json(cls, filename: str | Path) -> "BackgroundReference":
        path = Path(filename)
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("format") != BACKGROUND_FORMAT:
            raise ValueError(f"unsupported background format: {payload.get('format')}")
        return cls(
            frequency_hz=payload["frequency_hz"],
            s11=_complex_from_payload(payload.get("s11", {}), "s11"),
            capture_count=payload.get("capture_count", 1),
            metadata=dict(payload.get("metadata", {})),
        )


@dataclass
class ProcessingResult:
    """Intermediate and final values produced by :func:`process_s11_trace`."""

    frequency_hz: np.ndarray
    raw_s11: np.ndarray
    calibrated_s11: np.ndarray
    background_subtracted_s11: np.ndarray
    processed_s11: np.ndarray
    removed_frequency_mean: complex
    time_ns: np.ndarray
    time_trace: np.ndarray
    calibration_applied: bool
    background_applied: bool
    frequency_mean_removed: bool


def process_s11_trace(
    frequency_hz: Any,
    raw_s11: Any,
    *,
    calibration: OSLCalibration | None = None,
    background: BackgroundReference | None = None,
    remove_frequency_mean: bool = False,
    ifft_points: int = 2048,
    time_window_ns: float | None = None,
) -> ProcessingResult:
    """Run correction, background subtraction, optional de-meaning, and IFFT.

    ``remove_frequency_mean`` defaults to ``False`` because this operation can
    alter shallow reflections.  Enable it explicitly only after comparing the
    calibrated/background-subtracted result with and without mean removal.

    The time-axis calculation matches the current UAV-GPR application.
    """

    frequency = _frequency_vector(frequency_hz)
    raw = _complex_vector(raw_s11, "raw_s11", frequency.size)

    spacing = np.diff(frequency)
    if not np.allclose(spacing, spacing[0], rtol=1e-6, atol=0.0):
        raise ValueError("IFFT requires a uniformly spaced frequency grid")

    if calibration is None:
        calibrated = raw.copy()
    else:
        calibrated = calibration.correct(frequency, raw)

    if background is None:
        after_background = calibrated.copy()
    else:
        after_background = calibrated - background.values_at(frequency)

    removed_mean = 0.0 + 0.0j
    processed = after_background.copy()
    if remove_frequency_mean:
        removed_mean = complex(np.mean(processed))
        processed = processed - removed_mean

    point_count = frequency.size
    transform_count = max(int(ifft_points), point_count)
    if transform_count < 2:
        raise ValueError("ifft_points must be at least 2")

    spectrum = np.zeros(transform_count, dtype=np.complex128)
    spectrum[:point_count] = processed
    full_trace = np.real(np.fft.ifft(spectrum))

    bandwidth_hz = float(frequency[-1] - frequency[0])
    if bandwidth_hz <= 0.0:
        raise ValueError("frequency bandwidth must be positive")
    # Kept consistent with uav_gpr_qt.py:550-559.
    dt_s = point_count / (bandwidth_hz * transform_count)
    full_time_ns = np.arange(transform_count, dtype=float) * dt_s * 1e9

    if time_window_ns is None:
        mask = np.ones(transform_count, dtype=bool)
    else:
        if not np.isfinite(time_window_ns) or time_window_ns <= 0.0:
            raise ValueError("time_window_ns must be positive")
        mask = full_time_ns <= float(time_window_ns)

    return ProcessingResult(
        frequency_hz=frequency,
        raw_s11=raw,
        calibrated_s11=calibrated,
        background_subtracted_s11=after_background,
        processed_s11=processed,
        removed_frequency_mean=removed_mean,
        time_ns=full_time_ns[mask],
        time_trace=full_trace[mask],
        calibration_applied=calibration is not None,
        background_applied=background is not None,
        frequency_mean_removed=bool(remove_frequency_mean),
    )
