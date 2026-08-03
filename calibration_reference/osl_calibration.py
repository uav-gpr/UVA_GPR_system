"""Standalone one-port OSL calibration reference.

This module summarizes the calibration math used by the UAV-GPR project:

1. Measure a physical Open, Short, and Load on the same frequency grid.
2. Solve directivity, reflection-tracking, and source-match error terms.
3. Interpolate those complex error terms when required.
4. Correct a raw S11 or S22 measurement.

The default standard model is ideal:

* Open:  gamma = +1
* Short: gamma = -1
* Load:  gamma =  0

Real, frequency-dependent standard definitions can be supplied through the
``*_actual`` arguments of :func:`build_osl_calibration`.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


FORMAT_NAME = "standalone_osl_calibration_v1"
MIN_DENOMINATOR = 1e-15


def _float_vector(values: Any, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 1 or array.size == 0:
        raise ValueError(f"{name} must be a non-empty one-dimensional array")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains a non-finite value")
    return array


def _complex_vector(values: Any, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=np.complex128)
    if array.ndim != 1 or array.size == 0:
        raise ValueError(f"{name} must be a non-empty one-dimensional array")
    if not np.all(np.isfinite(array.real)) or not np.all(np.isfinite(array.imag)):
        raise ValueError(f"{name} contains a non-finite value")
    return array


def _standard_vector(values: Any, count: int, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=np.complex128)
    if array.ndim == 0:
        return np.full(count, complex(array), dtype=np.complex128)
    array = _complex_vector(array, name)
    if array.size != count:
        raise ValueError(f"{name} must contain {count} points")
    return array


def _check_frequency_grid(frequency_hz: np.ndarray) -> None:
    if np.any(np.diff(frequency_hz) <= 0):
        raise ValueError("frequency_hz must be strictly increasing")


def _same_length(reference: np.ndarray, **arrays: np.ndarray) -> None:
    for name, array in arrays.items():
        if array.size != reference.size:
            raise ValueError(
                f"{name} has {array.size} points; expected {reference.size}"
            )


def complex_interpolate(
    requested_frequency_hz: Any,
    calibration_frequency_hz: Any,
    values: Any,
) -> np.ndarray:
    """Linearly interpolate the real and imaginary parts of complex data.

    Extrapolation is rejected because using a calibration outside its measured
    band normally produces misleading results.
    """

    requested = _float_vector(requested_frequency_hz, "requested_frequency_hz")
    calibration_frequency = _float_vector(
        calibration_frequency_hz, "calibration_frequency_hz"
    )
    complex_values = _complex_vector(values, "values")
    _check_frequency_grid(calibration_frequency)
    _same_length(calibration_frequency, values=complex_values)

    tolerance = max(abs(calibration_frequency[-1]), 1.0) * 1e-12
    if (
        requested.min() < calibration_frequency[0] - tolerance
        or requested.max() > calibration_frequency[-1] + tolerance
    ):
        raise ValueError("measurement frequency is outside the calibration band")

    real = np.interp(requested, calibration_frequency, complex_values.real)
    imag = np.interp(requested, calibration_frequency, complex_values.imag)
    return real + 1j * imag


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
class OSLCalibration:
    """Solved three-term one-port error model."""

    frequency_hz: np.ndarray
    directivity: np.ndarray
    reflection_tracking: np.ndarray
    source_match: np.ndarray
    raw_standards: dict[str, np.ndarray] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.frequency_hz = _float_vector(self.frequency_hz, "frequency_hz")
        self.directivity = _complex_vector(self.directivity, "directivity")
        self.reflection_tracking = _complex_vector(
            self.reflection_tracking, "reflection_tracking"
        )
        self.source_match = _complex_vector(self.source_match, "source_match")
        _check_frequency_grid(self.frequency_hz)
        _same_length(
            self.frequency_hz,
            directivity=self.directivity,
            reflection_tracking=self.reflection_tracking,
            source_match=self.source_match,
        )

        checked_standards: dict[str, np.ndarray] = {}
        for name, values in self.raw_standards.items():
            array = _complex_vector(values, f"raw_standards[{name}]")
            _same_length(self.frequency_hz, **{name: array})
            checked_standards[name] = array
        self.raw_standards = checked_standards

    def correct(self, frequency_hz: Any, measured: Any) -> np.ndarray:
        """Correct raw S11 or S22 complex values."""

        requested = _float_vector(frequency_hz, "frequency_hz")
        measured_array = _complex_vector(measured, "measured")
        _same_length(requested, measured=measured_array)

        directivity = complex_interpolate(
            requested, self.frequency_hz, self.directivity
        )
        tracking = complex_interpolate(
            requested, self.frequency_hz, self.reflection_tracking
        )
        source_match = complex_interpolate(
            requested, self.frequency_hz, self.source_match
        )

        measured_minus_directivity = measured_array - directivity
        denominator = tracking + source_match * measured_minus_directivity
        if np.any(np.abs(denominator) < MIN_DENOMINATOR):
            raise ValueError("calibration correction denominator is too small")
        return measured_minus_directivity / denominator

    def save_json(self, filename: str | Path) -> None:
        """Save solved terms and optional raw standard captures."""

        path = Path(filename)
        payload = {
            "format": FORMAT_NAME,
            "frequency_hz": self.frequency_hz.astype(float).tolist(),
            "error_terms": {
                "directivity": _complex_payload(self.directivity),
                "reflection_tracking": _complex_payload(
                    self.reflection_tracking
                ),
                "source_match": _complex_payload(self.source_match),
            },
            "raw_standards": {
                name: _complex_payload(values)
                for name, values in self.raw_standards.items()
            },
            "metadata": self.metadata,
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    @classmethod
    def load_json(cls, filename: str | Path) -> "OSLCalibration":
        """Load a file produced by :meth:`save_json`."""

        path = Path(filename)
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("format") != FORMAT_NAME:
            raise ValueError(f"unsupported calibration format: {payload.get('format')}")
        terms = payload.get("error_terms", {})
        raw_standards = {
            name: _complex_from_payload(values, f"raw_standards[{name}]")
            for name, values in payload.get("raw_standards", {}).items()
        }
        return cls(
            frequency_hz=payload["frequency_hz"],
            directivity=_complex_from_payload(
                terms.get("directivity", {}), "directivity"
            ),
            reflection_tracking=_complex_from_payload(
                terms.get("reflection_tracking", {}), "reflection_tracking"
            ),
            source_match=_complex_from_payload(
                terms.get("source_match", {}), "source_match"
            ),
            raw_standards=raw_standards,
            metadata=dict(payload.get("metadata", {})),
        )


def build_osl_calibration(
    frequency_hz: Any,
    open_measured: Any,
    short_measured: Any,
    load_measured: Any,
    *,
    open_actual: Any = 1.0 + 0.0j,
    short_actual: Any = -1.0 + 0.0j,
    load_actual: Any = 0.0 + 0.0j,
    metadata: dict[str, Any] | None = None,
) -> OSLCalibration:
    """Solve a one-port OSL calibration from three physical captures.

    ``open_actual``, ``short_actual``, and ``load_actual`` may be scalars for
    ideal standards or complex arrays for frequency-dependent cal-kit models.
    """

    frequency = _float_vector(frequency_hz, "frequency_hz")
    _check_frequency_grid(frequency)
    open_m = _complex_vector(open_measured, "open_measured")
    short_m = _complex_vector(short_measured, "short_measured")
    load_m = _complex_vector(load_measured, "load_measured")
    _same_length(
        frequency,
        open_measured=open_m,
        short_measured=short_m,
        load_measured=load_m,
    )

    count = frequency.size
    open_a = _standard_vector(open_actual, count, "open_actual")
    short_a = _standard_vector(short_actual, count, "short_actual")
    load_a = _standard_vector(load_actual, count, "load_actual")

    lc = load_a
    oc = open_a
    sc = short_a
    lm = load_m
    om = open_m
    sm = short_m

    denominator = (
        lc * oc * (om - lm)
        + lc * sc * (lm - sm)
        + oc * sc * (sm - om)
    )
    if np.any(np.abs(denominator) < MIN_DENOMINATOR):
        bad_index = int(np.argmin(np.abs(denominator)))
        raise ValueError(
            "invalid OSL calibration denominator near "
            f"{frequency[bad_index]:g} Hz"
        )

    directivity = (
        lc * om * (sm * (oc - sc) + lm * sc)
        - lc * oc * lm * sm
        + oc * lm * sc * (sm - om)
    ) / denominator
    source_match = (
        lc * (om - sm) + oc * (sm - lm) + sc * (lm - om)
    ) / denominator
    delta = (
        lc * lm * (om - sm)
        + oc * om * (sm - lm)
        + sc * sm * (lm - om)
    ) / denominator
    reflection_tracking = directivity * source_match - delta

    result_metadata = {
        "method": "one-port OSL three-term error model",
        "standard_model": "ideal unless frequency-dependent actual values were supplied",
    }
    if metadata:
        result_metadata.update(metadata)

    return OSLCalibration(
        frequency_hz=frequency,
        directivity=directivity,
        reflection_tracking=reflection_tracking,
        source_match=source_match,
        raw_standards={
            "Open": open_m,
            "Short": short_m,
            "Load": load_m,
        },
        metadata=result_metadata,
    )
