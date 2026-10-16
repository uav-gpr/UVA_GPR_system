"""校准与扫频参数是否还"对得上"的判断（纯 Python，无 Qt、无硬件）。

大白话：校准就像给秤归零，归零只对当时那套称法有效。校准做完以后如果改了
扫频参数，这份校准可能就不再适用：

- **频率范围或频点数变了** → 校准"失效"：黑匣子停用这份校准，数据按未校准
  处理，地面站醒目提示"请重新校准"。参数改回原样后自动恢复生效。
- **只改了发射功率或中频带宽** → 影响小，只"提醒"，校准照常使用。

判断依据直接取自生效校准本身的频率轴（OSL 误差项 / Through / 背景各自的
频点），再加上校准当时记下的扫频参数（有就比功率和带宽，没有就跳过）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Mapping, Sequence

import numpy as np

#: 判定"同一个频点"的最小容差（Hz）。VNA 实际频点可能有量化误差。
MIN_FREQUENCY_TOLERANCE_HZ = 1_000.0
#: 判定功率"没改"的容差（dBm）。
POWER_TOLERANCE_DBM = 0.05

STALE_HINT = "校准已失效，请重新校准"


@dataclass(frozen=True)
class FrequencyGrid:
    start_hz: float
    stop_hz: float
    points: int

    def describe(self) -> str:
        return f"{self.start_hz / 1e6:g}-{self.stop_hz / 1e6:g} MHz, {self.points} 点"

    def to_dict(self) -> dict[str, object]:
        return {
            "start_mhz": round(self.start_hz / 1e6, 6),
            "stop_mhz": round(self.stop_hz / 1e6, 6),
            "points": int(self.points),
        }


@dataclass
class CalibrationFit:
    """当前扫频参数与生效校准的匹配结果。"""

    osl_stale: bool = False
    background_stale: bool = False
    warnings: list[str] = field(default_factory=list)
    reasons: list[str] = field(default_factory=list)
    calibration_grid: FrequencyGrid | None = None

    @property
    def stale(self) -> bool:
        return self.osl_stale or self.background_stale

    @property
    def level(self) -> str:
        if self.stale:
            return "stale"
        if self.warnings:
            return "warning"
        return "ok"

    def message(self) -> str:
        if self.stale:
            return f"{STALE_HINT}：" + "；".join(self.reasons)
        if self.warnings:
            return "校准提醒：" + "；".join(self.warnings)
        return ""

    def to_dict(self) -> dict[str, object]:
        return {
            "level": self.level,
            "stale": self.stale,
            "osl_stale": self.osl_stale,
            "background_stale": self.background_stale,
            "reasons": list(self.reasons),
            "warnings": list(self.warnings),
            "message": self.message(),
            "calibration_grid": self.calibration_grid.to_dict() if self.calibration_grid else None,
        }


def grid_from_frequencies(frequency_hz: Iterable[float] | None) -> FrequencyGrid | None:
    if frequency_hz is None:
        return None
    values = np.asarray(list(frequency_hz) if not isinstance(frequency_hz, np.ndarray) else frequency_hz, dtype=np.float64)
    values = values.reshape(-1)
    if values.size == 0 or not np.all(np.isfinite(values)):
        return None
    values = np.sort(values)
    return FrequencyGrid(float(values[0]), float(values[-1]), int(values.size))


def grid_from_osl(calibration: object | None) -> FrequencyGrid | None:
    """从 OSLCalibration（``points[i].frequency``）取频率轴。"""
    if calibration is None:
        return None
    points: Sequence[object] = getattr(calibration, "points", None) or []
    try:
        return grid_from_frequencies([float(getattr(p, "frequency")) for p in points])
    except (TypeError, ValueError):
        return None


def grid_from_sweep(sweep: object) -> FrequencyGrid:
    return FrequencyGrid(
        float(getattr(sweep, "start_mhz")) * 1e6,
        float(getattr(sweep, "stop_mhz")) * 1e6,
        int(getattr(sweep, "points")),
    )


def grids_match(calibration: FrequencyGrid, current: FrequencyGrid) -> bool:
    if calibration.points != current.points:
        return False
    span = max(abs(calibration.stop_hz - calibration.start_hz), 0.0)
    step = span / (calibration.points - 1) if calibration.points > 1 else 0.0
    tolerance = max(MIN_FREQUENCY_TOLERANCE_HZ, 0.5 * step)
    return (
        abs(calibration.start_hz - current.start_hz) <= tolerance
        and abs(calibration.stop_hz - current.stop_hz) <= tolerance
    )


def _mismatch_reason(label: str, calibration: FrequencyGrid, current: FrequencyGrid) -> str:
    return f"{label}校准是 {calibration.describe()}，当前扫频是 {current.describe()}"


def evaluate_calibration_fit(
    sweep: object,
    *,
    osl_grids: Sequence[FrequencyGrid | None] = (),
    background_grid: FrequencyGrid | None = None,
    reference_sweep: Mapping[str, object] | None = None,
) -> CalibrationFit:
    """比较当前扫频与生效校准。

    ``osl_grids``：OSL P1/P2、Through 等组件各自的频率轴（None 跳过）。
    ``background_grid``：背景校准频率轴。
    ``reference_sweep``：校准当时记下的扫频参数（可缺省），用于功率/带宽提醒。
    """
    fit = CalibrationFit()
    current = grid_from_sweep(sweep)
    grids = [g for g in osl_grids if g is not None]
    if grids:
        fit.calibration_grid = grids[0]
        for grid in grids:
            if not grids_match(grid, current):
                fit.osl_stale = True
                fit.reasons.append(_mismatch_reason("OSL ", grid, current))
                break
    if background_grid is not None:
        if fit.calibration_grid is None:
            fit.calibration_grid = background_grid
        if fit.osl_stale or not grids_match(background_grid, current):
            # OSL 失效时背景（建立在 OSL 域上）一并失效。
            fit.background_stale = True
            if not fit.osl_stale:
                fit.reasons.append(_mismatch_reason("背景", background_grid, current))
    if (grids or background_grid is not None) and reference_sweep:
        try:
            ref_power = reference_sweep.get("power_dbm")
            if ref_power is not None and abs(float(ref_power) - float(getattr(sweep, "power_dbm"))) > POWER_TOLERANCE_DBM:
                fit.warnings.append(
                    f"发射功率由校准时的 {float(ref_power):g} dBm 改为 {float(getattr(sweep, 'power_dbm')):g} dBm"
                    "（校准仍在使用，建议重新校准）"
                )
            ref_ifbw = reference_sweep.get("ifbw_hz")
            if ref_ifbw is not None and int(float(ref_ifbw)) != int(getattr(sweep, "ifbw_hz")):
                fit.warnings.append(
                    f"中频带宽由校准时的 {int(float(ref_ifbw))} Hz 改为 {int(getattr(sweep, 'ifbw_hz'))} Hz"
                    "（校准仍在使用，建议重新校准）"
                )
        except (TypeError, ValueError):
            pass
    return fit
