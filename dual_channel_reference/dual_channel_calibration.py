"""Simplified two-port calibration reference for dual-channel UAV-GPR data.

The correction strategy intentionally matches the main project:

* Port 1 Open/Short/Load solves a three-term OSL model for S11.
* Port 2 Open/Short/Load solves a three-term OSL model for S22.
* A Through capture provides complex normalization for S21 and S12.

This is not a complete forward/reverse 12-term SOLT implementation.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np
from numpy.typing import NDArray

from .dual_channel_acquisition import S_PARAMETERS, DualChannelSweep


MIN_DENOMINATOR = 1e-15
CALIBRATION_FORMAT = "uav_gpr_dual_channel_calibration_v1"


def _frequency_axis(values: object, name: str = "frequency_hz") -> NDArray[np.float64]:
    frequency = np.asarray(values, dtype=np.float64)
    if frequency.ndim != 1 or frequency.size < 2:
        raise ValueError(f"{name} must be a one-dimensional array with at least 2 points")
    if not np.all(np.isfinite(frequency)) or np.any(np.diff(frequency) <= 0.0):
        raise ValueError(f"{name} must contain finite, strictly increasing values")
    return frequency


def _complex_vector(values: object, name: str, count: int) -> NDArray[np.complex128]:
    array = np.asarray(values, dtype=np.complex128)
    if array.ndim != 1 or array.size != count:
        raise ValueError(f"{name} must contain {count} complex points")
    if not np.all(np.isfinite(array.real)) or not np.all(np.isfinite(array.imag)):
        raise ValueError(f"{name} contains a non-finite value")
    return array


def _same_frequency_axis(reference: NDArray[np.float64], sweep: DualChannelSweep, name: str) -> None:
    if not np.array_equal(reference, sweep.frequency_hz):
        raise ValueError(f"{name} frequency axis differs from the other calibration captures")


def _complex_payload(values: NDArray[np.complex128]) -> dict[str, list[float]]:
    return {
        "real": values.real.astype(float).tolist(),
        "imag": values.imag.astype(float).tolist(),
    }


def _complex_from_payload(payload: Mapping[str, object], name: str) -> NDArray[np.complex128]:
    try:
        real = np.asarray(payload["real"], dtype=np.float64)
        imag = np.asarray(payload["imag"], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid complex payload for {name}") from exc
    if real.shape != imag.shape:
        raise ValueError(f"real and imaginary lengths differ for {name}")
    return real + 1j * imag


def complex_interpolate(
    requested_hz: object,
    source_hz: object,
    source_values: object,
) -> NDArray[np.complex128]:
    """Interpolate real/imaginary parts while rejecting extrapolation."""

    requested = _frequency_axis(requested_hz, "requested_hz")
    source = _frequency_axis(source_hz, "source_hz")
    values = _complex_vector(source_values, "source_values", source.size)
    tolerance = max(abs(source[0]), abs(source[-1]), 1.0) * 1e-12
    if requested[0] < source[0] - tolerance or requested[-1] > source[-1] + tolerance:
        raise ValueError("requested frequencies lie outside the calibration range")
    real = np.interp(requested, source, values.real)
    imag = np.interp(requested, source, values.imag)
    return real + 1j * imag


@dataclass
class OnePortOSL:
    """Solved directivity/tracking/source-match terms for one reflection port."""

    frequency_hz: NDArray[np.float64]
    directivity: NDArray[np.complex128]
    reflection_tracking: NDArray[np.complex128]
    source_match: NDArray[np.complex128]

    def __post_init__(self) -> None:
        self.frequency_hz = _frequency_axis(self.frequency_hz)
        count = self.frequency_hz.size
        self.directivity = _complex_vector(self.directivity, "directivity", count)
        self.reflection_tracking = _complex_vector(
            self.reflection_tracking, "reflection_tracking", count
        )
        self.source_match = _complex_vector(self.source_match, "source_match", count)

    @classmethod
    def solve(
        cls,
        frequency_hz: object,
        open_measured: object,
        short_measured: object,
        load_measured: object,
        *,
        open_actual: complex = 1.0 + 0.0j,
        short_actual: complex = -1.0 + 0.0j,
        load_actual: complex = 0.0 + 0.0j,
    ) -> "OnePortOSL":
        """Solve the three-term model independently at every frequency."""

        frequency = _frequency_axis(frequency_hz)
        count = frequency.size
        om = _complex_vector(open_measured, "open_measured", count)
        sm = _complex_vector(short_measured, "short_measured", count)
        lm = _complex_vector(load_measured, "load_measured", count)
        oc = np.full(count, complex(open_actual), dtype=np.complex128)
        sc = np.full(count, complex(short_actual), dtype=np.complex128)
        lc = np.full(count, complex(load_actual), dtype=np.complex128)

        denominator = (
            lc * oc * (om - lm)
            + lc * sc * (lm - sm)
            + oc * sc * (sm - om)
        )
        if np.any(np.abs(denominator) < MIN_DENOMINATOR):
            bad = int(np.argmin(np.abs(denominator)))
            raise ValueError(f"invalid OSL denominator near {frequency[bad]:g} Hz")

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
        return cls(frequency, directivity, reflection_tracking, source_match)

    def correct(self, frequency_hz: object, measured: object) -> NDArray[np.complex128]:
        frequency = _frequency_axis(frequency_hz)
        measured_values = _complex_vector(measured, "measured", frequency.size)
        directivity = complex_interpolate(frequency, self.frequency_hz, self.directivity)
        tracking = complex_interpolate(
            frequency, self.frequency_hz, self.reflection_tracking
        )
        source_match = complex_interpolate(frequency, self.frequency_hz, self.source_match)
        measured_minus_directivity = measured_values - directivity
        denominator = tracking + source_match * measured_minus_directivity
        if np.any(np.abs(denominator) < MIN_DENOMINATOR):
            raise ValueError("OSL correction denominator is too small")
        return measured_minus_directivity / denominator

    def to_payload(self) -> dict[str, object]:
        return {
            "frequency_hz": self.frequency_hz.astype(float).tolist(),
            "directivity": _complex_payload(self.directivity),
            "reflection_tracking": _complex_payload(self.reflection_tracking),
            "source_match": _complex_payload(self.source_match),
        }

    @classmethod
    def from_payload(cls, payload: Mapping[str, object]) -> "OnePortOSL":
        return cls(
            frequency_hz=payload["frequency_hz"],
            directivity=_complex_from_payload(payload["directivity"], "directivity"),
            reflection_tracking=_complex_from_payload(
                payload["reflection_tracking"], "reflection_tracking"
            ),
            source_match=_complex_from_payload(payload["source_match"], "source_match"),
        )


@dataclass
class ThroughReference:
    frequency_hz: NDArray[np.float64]
    s21: NDArray[np.complex128]
    s12: NDArray[np.complex128]

    def __post_init__(self) -> None:
        self.frequency_hz = _frequency_axis(self.frequency_hz)
        count = self.frequency_hz.size
        self.s21 = _complex_vector(self.s21, "through S21", count)
        self.s12 = _complex_vector(self.s12, "through S12", count)

    def values_at(
        self, frequency_hz: object
    ) -> tuple[NDArray[np.complex128], NDArray[np.complex128]]:
        frequency = _frequency_axis(frequency_hz)
        s21 = complex_interpolate(frequency, self.frequency_hz, self.s21)
        s12 = complex_interpolate(frequency, self.frequency_hz, self.s12)
        if np.any(np.abs(s21) < MIN_DENOMINATOR) or np.any(np.abs(s12) < MIN_DENOMINATOR):
            raise ValueError("Through reference is too small for stable division")
        return s21, s12

    def to_payload(self) -> dict[str, object]:
        return {
            "frequency_hz": self.frequency_hz.astype(float).tolist(),
            "S21": _complex_payload(self.s21),
            "S12": _complex_payload(self.s12),
        }

    @classmethod
    def from_payload(cls, payload: Mapping[str, object]) -> "ThroughReference":
        return cls(
            frequency_hz=payload["frequency_hz"],
            s21=_complex_from_payload(payload["S21"], "Through S21"),
            s12=_complex_from_payload(payload["S12"], "Through S12"),
        )


@dataclass
class DualChannelCalibration:
    """P1/P2 OSL plus bidirectional Through normalization."""

    port1: OnePortOSL
    port2: OnePortOSL
    through: ThroughReference

    @classmethod
    def from_captures(
        cls,
        *,
        p1_open: DualChannelSweep,
        p1_short: DualChannelSweep,
        p1_load: DualChannelSweep,
        p2_open: DualChannelSweep,
        p2_short: DualChannelSweep,
        p2_load: DualChannelSweep,
        through: DualChannelSweep,
    ) -> "DualChannelCalibration":
        """Build a calibration from the seven physical captures."""

        frequency = p1_open.frequency_hz
        captures = {
            "P1 Short": p1_short,
            "P1 Load": p1_load,
            "P2 Open": p2_open,
            "P2 Short": p2_short,
            "P2 Load": p2_load,
            "Through": through,
        }
        for name, capture in captures.items():
            _same_frequency_axis(frequency, capture, name)

        port1 = OnePortOSL.solve(
            frequency,
            p1_open.sparams["S11"],
            p1_short.sparams["S11"],
            p1_load.sparams["S11"],
        )
        port2 = OnePortOSL.solve(
            frequency,
            p2_open.sparams["S22"],
            p2_short.sparams["S22"],
            p2_load.sparams["S22"],
        )
        through_reference = ThroughReference(
            frequency,
            through.sparams["S21"],
            through.sparams["S12"],
        )
        return cls(port1, port2, through_reference)

    def apply(self, raw: DualChannelSweep) -> DualChannelSweep:
        """Correct all four S-parameters and preserve acquisition metadata."""

        frequency = raw.frequency_hz
        through_s21, through_s12 = self.through.values_at(frequency)
        corrected = {name: raw.sparams[name].copy() for name in S_PARAMETERS}
        corrected["S11"] = self.port1.correct(frequency, raw.sparams["S11"])
        corrected["S22"] = self.port2.correct(frequency, raw.sparams["S22"])
        corrected["S21"] = raw.sparams["S21"] / through_s21
        corrected["S12"] = raw.sparams["S12"] / through_s12
        return DualChannelSweep(
            frequency_hz=frequency.copy(),
            sparams=corrected,
            settings=raw.settings,
            captured_at=raw.captured_at,
        )

    def save_json(self, filename: str | Path) -> Path:
        path = Path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "format": CALIBRATION_FORMAT,
            "method": "P1/P2 one-port OSL plus complex S21/S12 Through normalization",
            "warning": "This is not a full 12-term two-port SOLT calibration.",
            "port1_osl": self.port1.to_payload(),
            "port2_osl": self.port2.to_payload(),
            "through": self.through.to_payload(),
        }
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        return path

    @classmethod
    def load_json(cls, filename: str | Path) -> "DualChannelCalibration":
        path = Path(filename)
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("format") != CALIBRATION_FORMAT:
            raise ValueError(f"unsupported calibration format: {payload.get('format')}")
        return cls(
            port1=OnePortOSL.from_payload(payload["port1_osl"]),
            port2=OnePortOSL.from_payload(payload["port2_osl"]),
            through=ThroughReference.from_payload(payload["through"]),
        )
