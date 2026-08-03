"""Executable example and numerical self-test for the calibration module.

Run from the project root with:

    python -m calibration_reference.example_usage
"""

from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

try:
    from .osl_calibration import OSLCalibration, build_osl_calibration
    from .two_port_reference import (
        ThroughMeasurement,
        apply_simplified_two_port_calibration,
    )
except ImportError:  # Allows ``python example_usage.py`` from this folder.
    from osl_calibration import OSLCalibration, build_osl_calibration
    from two_port_reference import (
        ThroughMeasurement,
        apply_simplified_two_port_calibration,
    )


def simulated_vna_measurement(
    actual_gamma: np.ndarray,
    directivity: np.ndarray,
    reflection_tracking: np.ndarray,
    source_match: np.ndarray,
) -> np.ndarray:
    """Forward model corresponding to OSLCalibration.correct()."""

    return directivity + (
        actual_gamma * reflection_tracking
        / (1.0 - actual_gamma * source_match)
    )


def main() -> None:
    frequency_hz = np.linspace(400e6, 800e6, 201)
    normalized = (frequency_hz - frequency_hz[0]) / (
        frequency_hz[-1] - frequency_hz[0]
    )

    # Synthetic VNA systematic errors used only to verify the algorithm.
    directivity = 0.025 + 0.008j * np.exp(1j * normalized)
    tracking = (0.91 - 0.04j) * np.exp(-0.08j * normalized)
    source_match = 0.07 + 0.015j * normalized

    open_measured = simulated_vna_measurement(
        np.ones_like(frequency_hz, dtype=np.complex128),
        directivity,
        tracking,
        source_match,
    )
    short_measured = simulated_vna_measurement(
        -np.ones_like(frequency_hz, dtype=np.complex128),
        directivity,
        tracking,
        source_match,
    )
    load_measured = simulated_vna_measurement(
        np.zeros_like(frequency_hz, dtype=np.complex128),
        directivity,
        tracking,
        source_match,
    )

    calibration = build_osl_calibration(
        frequency_hz,
        open_measured,
        short_measured,
        load_measured,
        metadata={"example": "synthetic numerical self-test"},
    )

    dut_actual = 0.45 * np.exp(-1j * 2.5 * normalized)
    dut_measured = simulated_vna_measurement(
        dut_actual,
        directivity,
        tracking,
        source_match,
    )
    dut_corrected = calibration.correct(frequency_hz, dut_measured)
    max_error = float(np.max(np.abs(dut_corrected - dut_actual)))

    # Demonstrate persistence without leaving generated data in the source tree.
    with TemporaryDirectory(prefix="uav_gpr_calibration_test_") as temp_dir:
        output = Path(temp_dir) / "example_calibration.json"
        calibration.save_json(output)
        loaded = OSLCalibration.load_json(output)
        reload_error = float(
            np.max(
                np.abs(loaded.correct(frequency_hz, dut_measured) - dut_actual)
            )
        )

    # Verify the separate, simplified Through-normalization helper.
    actual_s21 = 0.32 * np.exp(-1j * 1.8 * normalized)
    actual_s12 = 0.30 * np.exp(-1j * 1.9 * normalized)
    through_s21 = 0.96 * np.exp(-1j * 0.22 * normalized)
    through_s12 = 0.95 * np.exp(-1j * 0.24 * normalized)
    corrected_sparams = apply_simplified_two_port_calibration(
        frequency_hz,
        {
            "S11": dut_measured,
            "S21": actual_s21 * through_s21,
            "S12": actual_s12 * through_s12,
            "S22": dut_measured,
        },
        port1_osl=calibration,
        port2_osl=calibration,
        through=ThroughMeasurement(frequency_hz, through_s21, through_s12),
    )
    two_port_error = max(
        float(np.max(np.abs(corrected_sparams["S11"] - dut_actual))),
        float(np.max(np.abs(corrected_sparams["S22"] - dut_actual))),
        float(np.max(np.abs(corrected_sparams["S21"] - actual_s21))),
        float(np.max(np.abs(corrected_sparams["S12"] - actual_s12))),
    )

    print(f"OSL maximum correction error: {max_error:.3e}")
    print(f"Reloaded-file maximum error: {reload_error:.3e}")
    print(f"Simplified two-port maximum error: {two_port_error:.3e}")
    print("Temporary JSON save/load: passed")

    if max(max_error, reload_error, two_port_error) > 1e-10:
        raise RuntimeError("OSL numerical self-test failed")


if __name__ == "__main__":
    main()
