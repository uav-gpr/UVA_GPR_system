"""The simplified two-port correction currently used by UAV-GPR.

This is intentionally named "simplified".  It performs:

* P1 OSL correction for S11.
* P2 OSL correction for S22.
* Complex Through normalization for S21 and S12.

It is not a full forward/reverse 12-term SOLT implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

try:
    from .osl_calibration import OSLCalibration, complex_interpolate
except ImportError:  # Allows direct execution from this folder.
    from osl_calibration import OSLCalibration, complex_interpolate


S_PARAMETERS = ("S11", "S21", "S12", "S22")


def _frequency_vector(values: Any) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 1 or array.size == 0:
        raise ValueError("frequency_hz must be a non-empty one-dimensional array")
    if not np.all(np.isfinite(array)) or np.any(np.diff(array) <= 0):
        raise ValueError("frequency_hz must contain finite, increasing values")
    return array


def _complex_vector(values: Any, name: str, count: int) -> np.ndarray:
    array = np.asarray(values, dtype=np.complex128)
    if array.ndim != 1 or array.size != count:
        raise ValueError(f"{name} must contain {count} complex points")
    if not np.all(np.isfinite(array.real)) or not np.all(np.isfinite(array.imag)):
        raise ValueError(f"{name} contains a non-finite value")
    return array


@dataclass
class ThroughMeasurement:
    frequency_hz: np.ndarray
    s21: np.ndarray
    s12: np.ndarray

    def __post_init__(self) -> None:
        self.frequency_hz = _frequency_vector(self.frequency_hz)
        count = self.frequency_hz.size
        self.s21 = _complex_vector(self.s21, "s21", count)
        self.s12 = _complex_vector(self.s12, "s12", count)


def apply_simplified_two_port_calibration(
    frequency_hz: Any,
    raw_sparams: Mapping[str, Any],
    *,
    port1_osl: OSLCalibration | None = None,
    port2_osl: OSLCalibration | None = None,
    through: ThroughMeasurement | None = None,
) -> dict[str, np.ndarray]:
    """Apply the same channel-level correction strategy as the main project."""

    frequency = _frequency_vector(frequency_hz)
    count = frequency.size
    missing = [name for name in S_PARAMETERS if name not in raw_sparams]
    if missing:
        raise ValueError(f"raw_sparams is missing: {', '.join(missing)}")

    corrected = {
        name: _complex_vector(raw_sparams[name], name, count).copy()
        for name in S_PARAMETERS
    }

    if port1_osl is not None:
        corrected["S11"] = port1_osl.correct(frequency, corrected["S11"])
    if port2_osl is not None:
        corrected["S22"] = port2_osl.correct(frequency, corrected["S22"])

    if through is not None:
        references = {
            "S21": complex_interpolate(
                frequency, through.frequency_hz, through.s21
            ),
            "S12": complex_interpolate(
                frequency, through.frequency_hz, through.s12
            ),
        }
        for channel, reference in references.items():
            if np.any(np.abs(reference) < 1e-15):
                raise ValueError(f"Through reference for {channel} is too small")
            corrected[channel] = corrected[channel] / reference

    return corrected
