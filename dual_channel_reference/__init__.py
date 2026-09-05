"""Reusable two-port acquisition and simplified calibration reference code."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .dual_channel_acquisition import (
        DualChannelSweep,
        SweepSettings,
    )
    from .dual_channel_calibration import (
        DualChannelCalibration,
        OnePortOSL,
        ThroughReference,
    )

__all__ = [
    "S_PARAMETERS",
    "DualChannelCalibration",
    "DualChannelSweep",
    "OnePortOSL",
    "SweepSettings",
    "ThroughReference",
    "acquire_dual_channel",
]


def __getattr__(name: str):
    if name in {"S_PARAMETERS", "DualChannelSweep", "SweepSettings", "acquire_dual_channel"}:
        from . import dual_channel_acquisition

        return getattr(dual_channel_acquisition, name)
    if name in {"DualChannelCalibration", "OnePortOSL", "ThroughReference"}:
        from . import dual_channel_calibration

        return getattr(dual_channel_calibration, name)
    raise AttributeError(name)
