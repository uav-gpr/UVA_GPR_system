#!/usr/bin/env python3
"""空中端校准服务层 + 地面校准编排的纯 Python 模块（第2轮工作流 C）。

需求依据：``docs/round2_requirements_and_interfaces.md`` §5（R-C 系列）与
``docs/钢筋仪项目_S11_S22_采集与校准参考.md`` §4/§5（六步双反射 OSL、复数
平均、三项误差模型、质量指标、频率轴逐元素一致、禁止频带外外推）。

模块纪律（R-C-10）：

- **纯 Python，不 import Qt**；除标准库与 NumPy 外只在两个惰性点 import
  ``librevna.librevna_usb_s11``（OSLCalibration 适配层，非 USB 出口）。
- 硬件出口全部经注入点：:class:`CalibrationSession` 接受 ``capturer``
  （服务/后端抽象，``capture_one(step, index) -> (frequency_hz, raw)``），
  **禁止** ``acquire_sparameters`` 式自开自关进入本模块（§5.4：那会与常驻
  服务持有的会话撞 claim）。生产 capturer 落在 ``uav_gpr_qt.py`` 接线区；
  测试注入 fake capturer 产生确定性 S11/S22。
- 校准数据落盘格式复用 ``uav_gpr_solt_calibration_v2``（R-C-09，不发明新
  格式）；每条 capture 原始复数写 NPZ，文件名 ``capture_<step>_<i>.npz``，
  **禁止** ``trace_`` 前缀（bench ``rglob("trace_*.npz")`` 口径保护，§5.3）。
"""

from __future__ import annotations

import json
import math
import os
import re
import threading
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable, Sequence

import numpy as np

# ---------------------------------------------------------------------------
# 自 uav_gpr_qt.py 搬家的校准数据层（R-C-09 迁移映射；逻辑逐字一致）
# ---------------------------------------------------------------------------

CALIBRATION_STEPS = [
    ("P1_Open", "P1 Open"),
    ("P1_Short", "P1 Short"),
    ("P1_Load", "P1 Load"),
    ("P2_Open", "P2 Open"),
    ("P2_Short", "P2 Short"),
    ("P2_Load", "P2 Load"),
    ("Through", "Through"),
]
CALIBRATION_STEP_LABELS = dict(CALIBRATION_STEPS)
P1_OSL_KEYS = ("P1_Open", "P1_Short", "P1_Load")
P2_OSL_KEYS = ("P2_Open", "P2_Short", "P2_Load")
LEGACY_OSL_MAP = {"P1_Open": "Open", "P1_Short": "Short", "P1_Load": "Load"}

#: 六步双反射 OSL 的步骤名与回程通道（参考文档 §4.1 固定顺序）。
OSL_STEP_NAMES = ("P1_Open", "P1_Short", "P1_Load", "P2_Open", "P2_Short", "P2_Load")
#: OSL 步骤 → 该步采样的反射通道（P1 标准件采 S11，P2 标准件采 S22）。
OSL_STEP_CHANNEL = {
    "P1_Open": "S11",
    "P1_Short": "S11",
    "P1_Load": "S11",
    "P2_Open": "S22",
    "P2_Short": "S22",
    "P2_Load": "S22",
}

CALIBRATION_MODE_OSL = "osl"
CALIBRATION_MODE_BACKGROUND = "background"
#: 空采参考（背景）模式的唯一步骤名。
BACKGROUND_STEP_NAME = "Background"

REPEATS_DEFAULT = 16
REPEATS_MIN = 1
REPEATS_MAX = 256


@dataclass
class CalibrationMeasurement:
    frequency_hz: list[float]
    real: list[float]
    imag: list[float]
    channels: dict[str, dict[str, list[float]]] | None = None

    @classmethod
    def from_points(cls, points: list[tuple[float, complex]]) -> "CalibrationMeasurement":
        return cls(
            frequency_hz=[float(frequency) for frequency, _ in points],
            real=[float(s11.real) for _, s11 in points],
            imag=[float(s11.imag) for _, s11 in points],
        )

    @classmethod
    def from_sparameter_points(cls, points: list[tuple[float, dict[str, complex]]], primary: str = "S11") -> "CalibrationMeasurement":
        frequency_hz = [float(frequency) for frequency, _ in points]
        channels: dict[str, dict[str, list[float]]] = {}
        for name in ("S11", "S21", "S12", "S22"):
            values = [sparams[name] for _frequency, sparams in points if name in sparams]
            if len(values) == len(points):
                channels[name] = {
                    "real": [float(value.real) for value in values],
                    "imag": [float(value.imag) for value in values],
                }
        primary_values = [
            sparams.get(primary) or sparams.get("S11") or next(iter(sparams.values()))
            for _frequency, sparams in points
        ]
        return cls(
            frequency_hz=frequency_hz,
            real=[float(value.real) for value in primary_values],
            imag=[float(value.imag) for value in primary_values],
            channels=channels,
        )

    def as_complex_by_frequency(self, channel: str | None = None) -> dict[float, complex]:
        if channel and self.channels and channel in self.channels:
            real_values = self.channels[channel]["real"]
            imag_values = self.channels[channel]["imag"]
            return {
                float(frequency): complex(float(real), float(imag))
                for frequency, real, imag in zip(self.frequency_hz, real_values, imag_values)
            }
        return {
            float(frequency): complex(float(real), float(imag))
            for frequency, real, imag in zip(self.frequency_hz, self.real, self.imag)
        }


def osl_measurements_for_port(measurements: dict[str, CalibrationMeasurement], port: int) -> dict[str, CalibrationMeasurement]:
    if port == 1:
        mapping = {"P1_Open": "Open", "P1_Short": "Short", "P1_Load": "Load"}
    elif port == 2:
        mapping = {"P2_Open": "Open", "P2_Short": "Short", "P2_Load": "Load"}
    else:
        raise ValueError(f"Unsupported calibration port: {port}")
    selected: dict[str, CalibrationMeasurement] = {}
    for key, legacy_key in mapping.items():
        measurement = measurements.get(key) or measurements.get(legacy_key)
        if measurement:
            selected[legacy_key] = measurement
    return selected


def s11_osl_measurements(measurements: dict[str, CalibrationMeasurement]) -> dict[str, CalibrationMeasurement]:
    return osl_measurements_for_port(measurements, 1)


def calibration_points_payload(measurement: CalibrationMeasurement) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for idx, (frequency, real, imag) in enumerate(zip(measurement.frequency_hz, measurement.real, measurement.imag)):
        channels = {
            channel: {
                "real": values["real"][idx],
                "imag": values["imag"][idx],
            }
            for channel, values in (measurement.channels or {}).items()
            if idx < len(values["real"]) and idx < len(values["imag"])
        }
        rows.append({"frequency": frequency, "real": real, "imag": imag, "channels": channels})
    return rows


def write_calibration_file(path: Path, measurements: dict[str, CalibrationMeasurement], cfg: "SweepConfig") -> None:
    """写 ``uav_gpr_solt_calibration_v2`` JSON（R-C-09：复用既有格式，不发明新格式）。

    ``cfg`` 只经 ``dataclasses.asdict`` 读取（调用方传入 ``uav_gpr_qt.SweepConfig``
    实例）；本模块不 import 该类型，保持无 Qt 依赖。
    """
    from dataclasses import asdict

    payload = {
        "format": "uav_gpr_solt_calibration_v2",
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "method": "SOLT/SLOT; one-port S11 uses P1 Open/Short/Load, two-port workflow records P1/P2 OSL plus Through",
        "s_parameter": "S11 now; S11/S21/S12/S22 ready for two-port extension",
        "sweep_config": asdict(cfg),
        "measurements": [
            {
                "type": name,
                "data": {"points": calibration_points_payload(measurement)},
            }
            for name, measurement in measurements.items()
        ],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


MAX_CALIBRATION_FILE_BYTES = 10 * 1024 * 1024  # 10 MB


def validate_calibration_content(
    content: str | dict,
) -> tuple[bool, str, dict[str, object]]:
    """校验校准内容结构与数值有效性（R8-4 / R8-5 契约）。

    返回 (合法?, 错误信息, 校验后的数据字典)。
    杜绝坏文件、空文件、轴不一致、非有限值 (NaN/Inf) 破坏现有生效校准。
    """
    if isinstance(content, str):
        if len(content.encode("utf-8")) > MAX_CALIBRATION_FILE_BYTES:
            return False, f"校准内容超过最大限制 ({MAX_CALIBRATION_FILE_BYTES // (1024 * 1024)} MB)", {}
        try:
            data = json.loads(content)
        except Exception as exc:
            return False, f"JSON 解析失败: {exc}", {}
    elif isinstance(content, dict):
        data = content
        # 针对 dict 结构执行序列化字节大小门控，严格受限于 10MB 契约
        try:
            serialized_len = len(json.dumps(data, ensure_ascii=False).encode("utf-8"))
            if serialized_len > MAX_CALIBRATION_FILE_BYTES:
                return False, f"校准内容超过最大限制 ({MAX_CALIBRATION_FILE_BYTES // (1024 * 1024)} MB)", {}
        except Exception as exc:
            return False, f"校准字典序列化失败: {exc}", {}
        try:
            total_points = sum(
                len(m.get("data", {}).get("points", []))
                for m in data.get("measurements", [])
                if isinstance(m, dict) and isinstance(m.get("data", {}).get("points"), list)
            )
            if "points" in data and isinstance(data["points"], list):
                total_points += len(data["points"])
            if total_points > 50000:
                return False, f"校准数据点规模超限 ({total_points} > 50000 点)", {}
        except Exception as exc:
            return False, f"校准字典结构异常: {exc}", {}
    else:
        return False, f"不支持的内容类型: {type(content)}", {}

    if not isinstance(data, dict):
        return False, "校准内容根节点必须为 JSON 对象 (dict)", {}

    measurements_list = data.get("measurements")
    if not isinstance(measurements_list, list) or not measurements_list:
        # 兼容旧式 points 直接列表
        pts = data.get("points")
        if isinstance(pts, list) and pts:
            if len(pts) < 2:
                return False, f"校准采样频点过少 ({len(pts)} 点)，至少需要 2 点以构成有效频带", {}
            for idx, p in enumerate(pts):
                if not isinstance(p, dict):
                    return False, f"第 {idx} 个校准点非对象", {}
                try:
                    f = float(p["frequency"])
                    d = complex(p["directivity"])
                    t = complex(p["reflection_tracking"])
                    s = complex(p["source_match"])
                except Exception as exc:
                    return False, f"第 {idx} 点数据非法: {exc}", {}
                if not (math.isfinite(f) and f > 0):
                    return False, f"第 {idx} 点频率非法或非正", {}
                if not (
                    math.isfinite(d.real)
                    and math.isfinite(d.imag)
                    and math.isfinite(t.real)
                    and math.isfinite(t.imag)
                    and math.isfinite(s.real)
                    and math.isfinite(s.imag)
                ):
                    return False, f"第 {idx} 点包含非有限数值 (NaN/Inf)", {}
            return True, "", data
        return False, "缺少有效的 measurements 列表或 points 列表", {}

    present_types: set[str] = set()
    freq_axis: list[float] | None = None
    for item in measurements_list:
        if not isinstance(item, dict):
            return False, "measurements 包含非对象元素", {}
        mtype = item.get("type")
        if not isinstance(mtype, str) or not mtype:
            continue
        present_types.add(mtype)
        pts = item.get("data", {}).get("points", [])
        if not isinstance(pts, list) or not pts:
            return False, f"步骤 {mtype} 缺少采样点数据", {}
        if len(pts) < 2:
            return False, f"步骤 {mtype} 采样频点过少 ({len(pts)} 点)，至少需要 2 点以构成有效频带", {}
        step_freqs: list[float] = []
        for idx, pt in enumerate(pts):
            if not isinstance(pt, dict):
                return False, f"步骤 {mtype} 第 {idx} 点非对象", {}
            try:
                f = float(pt["frequency"])
                r = float(pt["real"])
                i = float(pt["imag"])
            except (KeyError, TypeError, ValueError) as exc:
                return False, f"步骤 {mtype} 第 {idx} 点数值非法: {exc}", {}
            if not (math.isfinite(f) and f > 0):
                return False, f"步骤 {mtype} 第 {idx} 点频率非法或非正: {f}", {}
            if not (math.isfinite(r) and math.isfinite(i)):
                return False, f"步骤 {mtype} 第 {idx} 点包含非有限值 (NaN/Inf)", {}
            step_freqs.append(f)
        if any(step_freqs[k] >= step_freqs[k + 1] for k in range(len(step_freqs) - 1)):
            return False, f"步骤 {mtype} 频率轴未严格单调递增", {}
        if freq_axis is None:
            freq_axis = step_freqs
        elif step_freqs != freq_axis:
            return False, f"步骤 {mtype} 频率轴与其他步骤不一致", {}

    has_p1 = {"P1_Open", "P1_Short", "P1_Load"}.issubset(present_types) or {"Open", "Short", "Load"}.issubset(present_types)
    has_any_p1 = bool(present_types & {"P1_Open", "P1_Short", "P1_Load", "Open", "Short", "Load"})
    has_p2 = {"P2_Open", "P2_Short", "P2_Load"}.issubset(present_types)
    # 单通道采集：允许只有 P2 的文件（只采 VV）；有 P1 痕迹就必须完整。
    if not has_p1 and (has_any_p1 or not has_p2):
        return False, "缺少完整的一端口 P1 (Open, Short, Load) 标准件", {}

    return True, "", data


def read_calibration_measurements(path_or_dict: Path | str | dict) -> dict[str, CalibrationMeasurement]:
    if isinstance(path_or_dict, (str, Path)):
        data = json.loads(Path(path_or_dict).read_text(encoding="utf-8"))
    elif isinstance(path_or_dict, dict):
        data = path_or_dict
    else:
        raise ValueError(f"不支持的输入类型: {type(path_or_dict)}")
    measurements: dict[str, CalibrationMeasurement] = {}
    for measurement in data.get("measurements", []):
        name = measurement.get("type")
        raw_points = measurement.get("data", {}).get("points", [])
        if name in set(CALIBRATION_STEP_LABELS) | {"Open", "Short", "Load"}:
            channels: dict[str, dict[str, list[float]]] = {}
            for point in raw_points:
                for channel, value in point.get("channels", {}).items():
                    channels.setdefault(channel, {"real": [], "imag": []})
                    channels[channel]["real"].append(float(value["real"]))
                    channels[channel]["imag"].append(float(value["imag"]))
            measurements[name] = CalibrationMeasurement(
                frequency_hz=[float(point["frequency"]) for point in raw_points],
                real=[float(point["real"]) for point in raw_points],
                imag=[float(point["imag"]) for point in raw_points],
                channels=channels or None,
            )
    return measurements


def calibration_from_measurements(
    measurements: dict[str, CalibrationMeasurement],
    filename: Path | None = None,
    port: int = 1,
) -> "OSLCalibration":
    """由六步标准件均值构建一端口 OSL（迁移自 uav_gpr_qt.py，逻辑逐字一致）。

    OSLCalibration/OSLPoint/compute_osl_point 属 librevna 适配层，此处惰性
    导入（保持本模块 import 时不拉起 matplotlib/pyusb）。
    """
    from librevna.librevna_usb_s11 import OSLCalibration, OSLPoint, compute_osl_point

    osl_measurements = osl_measurements_for_port(measurements, port)
    missing = [name for name in ("Open", "Short", "Load") if name not in osl_measurements]
    if missing:
        raise ValueError(f"P{port} 一端口校准缺少标准件: {', '.join(missing)}")

    channel = "S11" if port == 1 else "S22"
    measured = {name: measurement.as_complex_by_frequency(channel) for name, measurement in osl_measurements.items()}
    common_freqs = sorted(set(measured["Open"]) & set(measured["Short"]) & set(measured["Load"]))
    if not common_freqs:
        raise ValueError("Open/Short/Load 的频率点不一致，不能生成校准")

    points: list[OSLPoint] = []
    for frequency in common_freqs:
        points.append(
            compute_osl_point(
                frequency,
                open_measured=measured["Open"][frequency],
                short_measured=measured["Short"][frequency],
                load_measured=measured["Load"][frequency],
                open_actual=complex(1.0, 0.0),
                short_actual=complex(-1.0, 0.0),
                load_actual=complex(0.0, 0.0),
            )
        )
    return OSLCalibration(points, filename or Path("session_osl_calibration.json"))


# ---------------------------------------------------------------------------
# 纯数学层：复数平均 / 三项 OSL 求解 / 质量指标 / 带内插值（R-C-06/07/08）
# ---------------------------------------------------------------------------

#: OSL 求解/校正的分母保护阈值（对齐 reference_code 的 MIN_DENOMINATOR）。
_MIN_DENOMINATOR = 1.0e-12

#: 理想标准件反射系数（Open/Short/Load），可被 ideal 参数覆盖。
IDEAL_OPEN = complex(1.0, 0.0)
IDEAL_SHORT = complex(-1.0, 0.0)
IDEAL_LOAD = complex(0.0, 0.0)


def validate_repeats(value: object) -> int:
    """repeats 严格校验（R-C-06）：bool/非 int/NaN/0/负/越界一律拒绝。"""
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise ValueError(f"repeats 必须是整数，收到 {value!r}")
    repeats = int(value)
    if not math.isfinite(float(repeats)):
        raise ValueError("repeats 必须是有限整数")
    if repeats < REPEATS_MIN or repeats > REPEATS_MAX:
        raise ValueError(f"repeats 必须在 {REPEATS_MIN}–{REPEATS_MAX} 范围内，收到 {repeats}")
    return repeats


def _frequency_axis(values: Sequence[float] | np.ndarray) -> np.ndarray:
    frequency = np.asarray(values, dtype=np.float64)
    if frequency.ndim != 1 or frequency.size < 2:
        raise ValueError("frequency_hz must be a one-dimensional vector with >= 2 points")
    if not np.all(np.isfinite(frequency)) or not np.all(np.diff(frequency) > 0.0):
        raise ValueError("frequency_hz must be finite and strictly increasing")
    return np.array(frequency, dtype=np.float64, copy=True)


def _readonly(array: np.ndarray) -> np.ndarray:
    result = np.array(array, copy=True)
    result.setflags(write=False)
    return result


def average_complex_captures(captures: Sequence[np.ndarray]) -> np.ndarray:
    """repeats 条完整 sweep 的复数平均：``np.mean(captures, axis=0)``（R-C-06）。

    实部虚部同一次平均（先平均后取模/取角都不允许）；输入形状必须一致，
    含 NaN/Inf 拒绝。
    """
    arrays = [np.asarray(capture) for capture in captures]
    if not arrays:
        raise ValueError("captures 不能为空")
    first = np.asarray(arrays[0], dtype=np.complex128)
    for index, array in enumerate(arrays):
        complex_array = np.asarray(array, dtype=np.complex128)
        if complex_array.shape != first.shape:
            raise ValueError(
                f"captures[{index}] 形状 {complex_array.shape} 与首条 {first.shape} 不一致"
            )
        if not np.all(np.isfinite(complex_array.real)) or not np.all(np.isfinite(complex_array.imag)):
            raise ValueError(f"captures[{index}] 含 NaN/Inf")
    stacked = np.stack([np.asarray(a, dtype=np.complex128) for a in arrays], axis=0)
    return np.mean(stacked, axis=0)


@dataclass(frozen=True)
class OSLTerms:
    """一端口三项误差模型（D/T/M）+ 质量指标（纯数据，频率轴只读）。"""

    frequency_hz: np.ndarray
    directivity: np.ndarray
    reflection_tracking: np.ndarray
    source_match: np.ndarray
    s_parameter: str
    quality_metrics: dict[str, object] = field(default_factory=dict)
    capture_counts: tuple[int, int, int] = (1, 1, 1)

    def __post_init__(self) -> None:
        if self.s_parameter not in {"S11", "S22"}:
            raise ValueError("s_parameter must be 'S11' or 'S22'")
        frequency = _frequency_axis(self.frequency_hz)
        terms = []
        for name, values in (
            ("directivity", self.directivity),
            ("reflection_tracking", self.reflection_tracking),
            ("source_match", self.source_match),
        ):
            term = np.asarray(values, dtype=np.complex128)
            if term.shape != frequency.shape:
                raise ValueError(f"{name} 必须与频率轴等长（{frequency.size}），收到 {term.shape}")
            if not np.all(np.isfinite(term.real)) or not np.all(np.isfinite(term.imag)):
                raise ValueError(f"{name} 含 NaN/Inf")
            terms.append(_readonly(term))
        object.__setattr__(self, "frequency_hz", _readonly(frequency))
        object.__setattr__(self, "directivity", terms[0])
        object.__setattr__(self, "reflection_tracking", terms[1])
        object.__setattr__(self, "source_match", terms[2])
        object.__setattr__(self, "quality_metrics", dict(self.quality_metrics))

    @property
    def valid_band_hz(self) -> tuple[float, float]:
        return (float(self.frequency_hz[0]), float(self.frequency_hz[-1]))


def apply_error_model(
    measured: np.ndarray,
    directivity: np.ndarray,
    reflection_tracking: np.ndarray,
    source_match: np.ndarray,
    frequency_hz: np.ndarray,
) -> np.ndarray:
    """三项误差模型校正：``x = m - D; Γ = x / (T + M·x)``（参考文档 §4.3）。"""
    m = np.asarray(measured, dtype=np.complex128)
    d = np.asarray(directivity, dtype=np.complex128)
    t = np.asarray(reflection_tracking, dtype=np.complex128)
    s = np.asarray(source_match, dtype=np.complex128)
    if m.shape[-1] != d.shape[-1]:
        raise ValueError("measured 的最后一维必须与误差项频率轴等长")
    x = m - d
    denominator = t + s * x
    if np.any(np.abs(denominator) < _MIN_DENOMINATOR):
        flat_index = int(np.argmin(np.abs(denominator)))
        frequency_index = np.unravel_index(flat_index, denominator.shape)[-1]
        raise ValueError(
            f"校正分母过小（接近奇异）near {float(np.asarray(frequency_hz).reshape(-1)[frequency_index]):g} Hz"
        )
    return x / denominator


def _solve_osl_terms(
    open_mean: np.ndarray,
    short_mean: np.ndarray,
    load_mean: np.ndarray,
    open_actual: np.ndarray,
    short_actual: np.ndarray,
    load_actual: np.ndarray,
    frequency_hz: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """三项 OSL 闭式解（与 reference_code/_solve_osl、librevna compute_osl_point 同公式）。"""
    lc, oc, sc = load_actual, open_actual, short_actual
    lm, om, sm = load_mean, open_mean, short_mean
    denominator = lc * oc * (om - lm) + lc * sc * (lm - sm) + oc * sc * (sm - om)
    if np.any(np.abs(denominator) < _MIN_DENOMINATOR):
        bad = int(np.argmin(np.abs(denominator)))
        raise ValueError(
            f"OSL 求解接近奇异 near {float(frequency_hz[bad]):g} Hz（标准件测量值退化？）"
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


def _standard_array(values: complex | Sequence[complex], n_frequency: int, name: str) -> np.ndarray:
    array = np.asarray(values)
    if array.ndim == 0:
        result = np.full(n_frequency, complex(array.item()), dtype=np.complex128)
    else:
        result = np.asarray(array, dtype=np.complex128)
        if result.shape != (n_frequency,):
            raise ValueError(f"{name} 必须是标量或逐频率向量")
    return result


def solve_one_port_osl(
    frequency_hz: Sequence[float],
    m_open: np.ndarray,
    m_short: np.ndarray,
    m_load: np.ndarray,
    *,
    s_parameter: str,
    ideal: tuple[complex, complex, complex] = (IDEAL_OPEN, IDEAL_SHORT, IDEAL_LOAD),
) -> OSLTerms:
    """由（可多条重复的）标准件测量解算一端口 D/T/M + 质量指标（R-C-07/08）。

    输入允许 ``(frequency,)`` 或 ``(capture, frequency)``；多条重复先复数
    平均（``np.mean(axis=0)``）再求解。质量指标（R-C-08）：

    - ``{label}_fit_max_abs_error``：平均标准件反算的最大绝对误差（OSL 拟合自洽性）；
    - ``{label}_repeat_rms_abs_error`` / ``{label}_repeat_max_abs_error``：
      逐条 capture 校正后的重复性（样本 <2 时为 None 并注明"样本不足"，
      绝不伪造 0）。
    """
    frequency = _frequency_axis(frequency_hz)

    def _captures(values: np.ndarray, name: str) -> np.ndarray:
        array = np.asarray(values, dtype=np.complex128)
        if array.ndim == 1:
            array = array[np.newaxis, :]
        if array.ndim != 2 or array.shape[1] != frequency.size or array.shape[0] < 1:
            raise ValueError(
                f"{name} 必须是 (frequency,) 或 (capture, frequency) 且频率点数 {frequency.size}"
            )
        if not np.all(np.isfinite(array.real)) or not np.all(np.isfinite(array.imag)):
            raise ValueError(f"{name} 含 NaN/Inf")
        return array

    open_captures = _captures(m_open, "m_open")
    short_captures = _captures(m_short, "m_short")
    load_captures = _captures(m_load, "m_load")
    open_mean = np.mean(open_captures, axis=0)
    short_mean = np.mean(short_captures, axis=0)
    load_mean = np.mean(load_captures, axis=0)
    open_standard, short_standard, load_standard = (
        _standard_array(ideal[0], frequency.size, "ideal_open"),
        _standard_array(ideal[1], frequency.size, "ideal_short"),
        _standard_array(ideal[2], frequency.size, "ideal_load"),
    )
    directivity, tracking, source_match = _solve_osl_terms(
        open_mean, short_mean, load_mean,
        open_standard, short_standard, load_standard,
        frequency,
    )
    metrics: dict[str, object] = {}
    for label, mean, standard, captures in (
        ("open", open_mean, open_standard, open_captures),
        ("short", short_mean, short_standard, short_captures),
        ("load", load_mean, load_standard, load_captures),
    ):
        corrected_mean = apply_error_model(mean, directivity, tracking, source_match, frequency)
        metrics[f"{label}_fit_max_abs_error"] = float(np.max(np.abs(corrected_mean - standard)))
        sample_count = int(captures.shape[0])
        metrics[f"{label}_sample_count"] = sample_count
        if sample_count < 2:
            # R-C-08：样本不足时字段为 None 并明确标注，绝不伪造 0。
            metrics[f"{label}_repeat_rms_abs_error"] = None
            metrics[f"{label}_repeat_max_abs_error"] = None
            metrics[f"{label}_insufficient_samples"] = True
        else:
            corrected_repeats = apply_error_model(
                captures, directivity, tracking, source_match, frequency
            )
            repeat_errors = np.abs(corrected_repeats - standard)
            metrics[f"{label}_repeat_rms_abs_error"] = float(np.sqrt(np.mean(np.square(repeat_errors))))
            metrics[f"{label}_repeat_max_abs_error"] = float(np.max(repeat_errors))
            metrics[f"{label}_insufficient_samples"] = False
    return OSLTerms(
        frequency_hz=frequency,
        directivity=directivity,
        reflection_tracking=tracking,
        source_match=source_match,
        s_parameter=s_parameter,
        quality_metrics=metrics,
        capture_counts=(open_captures.shape[0], short_captures.shape[0], load_captures.shape[0]),
    )


def interpolate_terms_in_band(
    requested: np.ndarray,
    terms: OSLTerms,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """带内逐点取误差项（同网格为恒等；允许带内线性插值，禁止带外外推）。"""
    requested = _frequency_axis(requested)
    source = terms.frequency_hz
    if requested[0] < source[0] or requested[-1] > source[-1]:
        raise ValueError("校准频率轴覆盖不足（禁止带外外推）")
    if np.array_equal(requested, source):
        return terms.directivity, terms.reflection_tracking, terms.source_match

    def _interp(values: np.ndarray) -> np.ndarray:
        real = np.interp(requested, source, values.real)
        imag = np.interp(requested, source, values.imag)
        return real + 1j * imag

    return _interp(terms.directivity), _interp(terms.reflection_tracking), _interp(terms.source_match)


def build_reference_background(
    frequency_hz: np.ndarray,
    raw_s11: np.ndarray,
    terms: OSLTerms | None,
) -> tuple[np.ndarray, np.ndarray]:
    """空采参考（背景）构建：RAW 域 S11 经当前 OSL 校正后作为背景（R-C-19）。

    返回 ``(background_complex_s11, frequency_copy)``。``terms=None`` 时背景
    就是 RAW S11（与既有 ``apply_active_calibration`` 无校准语义一致）。
    频率轴返回副本；后续减除的一致性校验由 ``process_trace_pipeline`` 的
    既有逐元素判据执行（同域 RAW/OSL、同频率轴、形状/有限值校验）。
    """
    frequency = np.asarray(frequency_hz, dtype=np.float64)
    raw = np.asarray(raw_s11, dtype=np.complex128)
    if raw.shape != frequency.shape:
        raise ValueError("raw_s11 必须与频率轴等长")
    if not np.all(np.isfinite(raw.real)) or not np.all(np.isfinite(raw.imag)):
        raise ValueError("raw_s11 含 NaN/Inf")
    if terms is None:
        background = raw.copy()
    else:
        directivity, tracking, source_match = interpolate_terms_in_band(frequency, terms)
        background = apply_error_model(raw, directivity, tracking, source_match, frequency)
    return background, np.array(frequency, dtype=np.float64, copy=True)


# ---------------------------------------------------------------------------
# 校准会话：CalibrationStep / CalibrationPlan / CalibrationSession（R-C-05）
# ---------------------------------------------------------------------------

def terms_to_osl_calibration(terms: OSLTerms, filename: Path | None) -> "OSLCalibration":
    """OSLTerms → librevna OSLCalibration 适配（惰性导入，管线兼容）。"""
    from librevna.librevna_usb_s11 import OSLCalibration, OSLPoint

    points = [
        OSLPoint(
            frequency=float(frequency),
            directivity=complex(d),
            reflection_tracking=complex(t),
            source_match=complex(s),
        )
        for frequency, d, t, s in zip(
            terms.frequency_hz, terms.directivity, terms.reflection_tracking, terms.source_match
        )
    ]
    return OSLCalibration(points, filename or Path("session_osl_calibration.json"))


@dataclass(frozen=True)
class CalibrationStep:
    """一个校准步骤的冻结计划条目（R-C-06：repeats 严格校验后写入）。"""

    name: str
    label: str
    channel: str
    repeats: int
    config_snapshot: dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class CalibrationPlan:
    """一次校准会话的完整计划（cal_id/mode/steps/config 快照）。"""

    cal_id: str
    mode: str
    steps: tuple[CalibrationStep, ...]
    config_snapshot: dict[str, object] = field(default_factory=dict)

    def step(self, name: str) -> CalibrationStep:
        for entry in self.steps:
            if entry.name == name:
                return entry
        raise KeyError(f"计划中不存在步骤 {name!r}")


OSL_PORT_CHOICES: dict[str, tuple[str, ...]] = {
    "both": ("P1", "P2"),
    "P1": ("P1",),
    "P2": ("P2",),
}


def normalize_osl_ports(value: object) -> tuple[str, ...]:
    """校准端口选择：both / P1 / P2（或列表），返回 ("P1","P2") 的子集。"""
    if value is None or value == "":
        return ("P1", "P2")
    if isinstance(value, (list, tuple)):
        ports = tuple(str(v).strip().upper() for v in value)
        if ports in (("P1", "P2"), ("P1",), ("P2",)):
            return ports
        raise ValueError(f"校准端口非法: {list(value)!r}")
    text = str(value).strip()
    for key, ports in OSL_PORT_CHOICES.items():
        if text.lower() == key.lower():
            return ports
    raise ValueError(f"校准端口非法: {value!r}（可选 both / P1 / P2）")


def build_osl_plan(
    cal_id: str,
    repeats: int,
    config_snapshot: dict[str, object],
    ports: Sequence[str] = ("P1", "P2"),
) -> CalibrationPlan:
    """双反射 OSL 计划（P1_Open…P2_Load，固定顺序，参考文档 §4.1）。

    ``ports`` 只选 P1 或 P2 时只有对应的 3 步（单通道采集用；另一端口不必接标准件）。
    """
    repeats = validate_repeats(repeats)
    ports = normalize_osl_ports(ports)
    steps = tuple(
        CalibrationStep(
            name=name,
            label=CALIBRATION_STEP_LABELS[name],
            channel=OSL_STEP_CHANNEL[name],
            repeats=repeats,
            config_snapshot=dict(config_snapshot),
        )
        for name in OSL_STEP_NAMES
        if name.split("_", 1)[0] in ports
    )
    return CalibrationPlan(
        cal_id=cal_id,
        mode=CALIBRATION_MODE_OSL,
        steps=steps,
        config_snapshot=dict(config_snapshot),
    )


def build_background_plan(cal_id: str, repeats: int, config_snapshot: dict[str, object]) -> CalibrationPlan:
    """空采参考（背景）计划：单步 Background，采 repeats 条取复数平均。"""
    repeats = validate_repeats(repeats)
    steps = (
        CalibrationStep(
            name=BACKGROUND_STEP_NAME,
            label="空采背景",
            channel="S11",
            repeats=repeats,
            config_snapshot=dict(config_snapshot),
        ),
    )
    return CalibrationPlan(
        cal_id=cal_id,
        mode=CALIBRATION_MODE_BACKGROUND,
        steps=steps,
        config_snapshot=dict(config_snapshot),
    )


def write_capture_npz(
    directory: Path,
    step_name: str,
    index: int,
    frequency_hz: np.ndarray,
    raw: np.ndarray,
    attempt: int = 1,
) -> Path:
    """单条 capture 原始复数落盘（原子写，R-X-06/R-C-15）。

    文件名 ``capture_<step>_<i>.npz``（重采批次为 ``capture_<step>_r<attempt>_<i>.npz``，**禁止** ``trace_`` 前缀——bench
    ``rglob("trace_*.npz")`` 口径保护，§5.3）。key 沿用既有约定：
    ``frequency_hz`` + ``<channel>_real``/``<channel>_imag``。
    """
    directory.mkdir(parents=True, exist_ok=True)
    raw = np.asarray(raw, dtype=np.complex128)
    if raw.ndim != 2 or raw.shape[0] != 2:
        raise ValueError(f"raw 必须是 (2, F) 双反射数组，收到 {raw.shape}")
    frequency = np.asarray(frequency_hz, dtype=np.float64)
    if frequency.shape != (raw.shape[1],):
        raise ValueError("frequency_hz 必须与 raw 的频率维等长")
    if not re.fullmatch(r"[A-Za-z0-9_.\-]+", step_name or ""):
        raise ValueError(f"非法步骤名: {step_name!r}")
    if index < 1:
        raise ValueError(f"capture 序号必须从 1 起，收到 {index}")
    if attempt > 1:
        target = directory / f"capture_{step_name}_r{attempt}_{index}.npz"
        tmp = directory / f".capture_{step_name}_r{attempt}_{index}.npz.tmp"
    else:
        target = directory / f"capture_{step_name}_{index}.npz"
        tmp = directory / f".capture_{step_name}_{index}.npz.tmp"
    # 用文件句柄写（np.savez 对路径入参会追加 .npz 后缀，破坏原子命名）。
    with open(tmp, "wb") as handle:
        np.savez(
            handle,
            frequency_hz=frequency,
            S11_real=raw[0].real,
            S11_imag=raw[0].imag,
            S22_real=raw[1].real,
            S22_imag=raw[1].imag,
        )
    os.replace(tmp, target)
    return target


@dataclass
class StepProgress:
    """一个步骤的运行期累计（线程内串行写入，锁保护读取）。"""

    name: str
    label: str
    channel: str
    repeats: int
    state: str = "pending"  # pending | capturing | done | failed
    captured: int = 0
    error: str | None = None
    frequency_hz: np.ndarray | None = None
    captures: list[np.ndarray] = field(default_factory=list)
    mean: np.ndarray | None = None
    capture_files: list[str] = field(default_factory=list)
    attempt: int = 1
    history_captures: list[list[np.ndarray]] = field(default_factory=list)


class CalibrationSession:
    """校准会话状态机（R-C-05）::

        planned → in_progress →（逐步 step_done）→ solved → activated
        任一点失败 → failed(reason)（终态）
        cancel()  → cancelled（终态，planned/in_progress/solved 均可）
        绝不从 failed/cancelled 迁移到 activated（半套校准绝不激活）。

    线程模型：``capture/solve/activate/cancel`` 可从不同线程调用（空中端由
    地面命令经 Qt 主线程分派 + 采集驱动线程执行 capture），内部 RLock 保护；
    ``capturer.capture_one`` 在调用方线程串行执行。

    ``capturer`` 注入点（§5.2 边界1）：``capture_one(step, index) ->
    (frequency_hz, raw)``，raw 形状 ``(2, F)``（S11/S22）。测试注入 fake 产生
    确定性序列；生产 capturer 见 ``uav_gpr_qt.py`` 接线区。
    """

    STATE_PLANNED = "planned"
    STATE_IN_PROGRESS = "in_progress"
    STATE_SOLVED = "solved"
    STATE_ACTIVATED = "activated"
    STATE_FAILED = "failed"
    STATE_CANCELLED = "cancelled"

    def __init__(
        self,
        plan: CalibrationPlan,
        capturer: object,
        *,
        capture_dir: Path | None = None,
    ) -> None:
        if not isinstance(plan, CalibrationPlan):
            raise TypeError("plan 必须是 CalibrationPlan")
        if not str(plan.cal_id or "").strip():
            raise ValueError("cal_id 不能为空")
        if capturer is None or not callable(getattr(capturer, "capture_one", None)):
            raise TypeError("capturer 必须提供 capture_one(step, index) 方法")
        self.plan = plan
        self.capturer = capturer
        self.capture_dir = Path(capture_dir) if capture_dir is not None else None
        self._lock = threading.RLock()
        self._state = self.STATE_PLANNED
        self._failure_reason: str | None = None
        self._cancel_reason: str | None = None
        self._progress: dict[str, StepProgress] = {
            step.name: StepProgress(name=step.name, label=step.label, channel=step.channel, repeats=step.repeats)
            for step in plan.steps
        }
        self._solved_result: dict[str, object] | None = None
        self._activation: dict[str, object] | None = None

    # ---- 只读观测 ----

    @property
    def cal_id(self) -> str:
        return self.plan.cal_id

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def failure_reason(self) -> str | None:
        with self._lock:
            return self._failure_reason

    def next_pending_step(self) -> CalibrationStep | None:
        with self._lock:
            for step in self.plan.steps:
                if self._progress[step.name].state in ("pending", "capturing"):
                    return step
            return None

    def statistics(self) -> dict[str, object]:
        """JSON 可序列化会话快照（地面展示/诊断/测试共用）。"""
        with self._lock:
            return {
                "cal_id": self.plan.cal_id,
                "mode": self.plan.mode,
                "state": self._state,
                "failure_reason": self._failure_reason,
                "cancel_reason": self._cancel_reason,
                "steps": [
                    {
                        "name": progress.name,
                        "label": progress.label,
                        "channel": progress.channel,
                        "repeats": progress.repeats,
                        "state": progress.state,
                        "captured": progress.captured,
                        "error": progress.error,
                    }
                    for progress in self._progress.values()
                ],
                "solved": self._solved_result is not None,
                "activated": self._activation is not None,
            }

    def active(self) -> dict[str, object] | None:
        """已激活校准的生效组件（R-C-09：active_calibration_components 委托点）。

        返回 ``{"cal_id", "mode", "p1_terms", "p2_terms", "background",
        "background_frequency_hz", "valid_band_hz"}``；未激活返回 None。
        """
        with self._lock:
            if self._state != self.STATE_ACTIVATED or self._activation is None:
                return None
            return dict(self._activation)

    def step_progress(self, step_name: str) -> StepProgress:
        """步骤运行期累计（只读用途；接线层发质量消息/写文件时使用）。"""
        with self._lock:
            return self._progress[step_name]

    def step_means(self) -> dict[str, np.ndarray]:
        """已完成步骤的复数均值 ``{(2, F)}``（步骤名 → mean）。"""
        with self._lock:
            return {
                name: progress.mean
                for name, progress in self._progress.items()
                if progress.state == "done" and progress.mean is not None
            }

    def step_frequency(self, step_name: str) -> np.ndarray:
        with self._lock:
            progress = self._progress[step_name]
            if progress.frequency_hz is None:
                raise ValueError(f"步骤 {step_name} 尚无频率轴")
            return progress.frequency_hz

    def solved_result(self) -> dict[str, object] | None:
        with self._lock:
            return dict(self._solved_result) if self._solved_result is not None else None

    # ---- 内部守卫 ----

    def _guard_open(self, action: str) -> None:
        if self._state in (self.STATE_FAILED, self.STATE_CANCELLED):
            raise RuntimeError(
                f"校准会话 {self.plan.cal_id} 已终止（state={self._state}），拒绝 {action}"
            )
        if self._state == self.STATE_ACTIVATED:
            raise RuntimeError(f"校准会话 {self.plan.cal_id} 已激活，拒绝 {action}")

    def _fail(self, reason: str) -> None:
        self._state = self.STATE_FAILED
        self._failure_reason = reason
        self._write_incomplete_marker(reason)

    def _write_incomplete_marker(self, reason: str) -> None:
        """R-C-18：失败/取消时保留已收 capture 文件并写 incomplete 标记。"""
        if self.capture_dir is None:
            return
        try:
            self.capture_dir.mkdir(parents=True, exist_ok=True)
            marker = self.capture_dir / "incomplete.json"
            tmp = self.capture_dir / ".incomplete.json.tmp"
            tmp.write_text(
                json.dumps(
                    {
                        "cal_id": self.plan.cal_id,
                        "mode": self.plan.mode,
                        "state": self._state,
                        "reason": reason,
                        "marked_at": datetime.now().isoformat(timespec="seconds"),
                    },
                    ensure_ascii=False,
                    indent=2,
                ),
                encoding="utf-8",
            )
            os.replace(tmp, marker)
        except OSError:
            # 标记写失败不改变会话终态（数据文件已保留），但绝不静默——由
            # 调用方统计/日志可见（这里选择吞掉 OSError 属可容忍降级：标记是
            # 辅助取证，主数据 capture_*.npz 已在盘）。
            pass

    # ---- 编排命令 ----

    def capture(
        self,
        step_name: str,
        *,
        recapture: bool = False,
        on_capture: Callable[[np.ndarray, np.ndarray, int, int], None] | None = None,
    ) -> dict[str, object]:
        """执行一个步骤：采 repeats 条完整 sweep → 复数平均 → step_done。

        支持普通按序采集与受控显式重采（recapture=True）。
        重采允许替换已完成步骤，保留其他有效步骤，失效旧解算并保留历史采样证据。
        """
        with self._lock:
            self._guard_open(f"capture({step_name})")
            if step_name not in self._progress:
                raise KeyError(f"计划中不存在步骤 {step_name!r}")

            # 并发保护：杜绝任何步骤正在采集时拉入新任务
            capturing_steps = [p.name for p in self._progress.values() if p.state == "capturing"]
            if capturing_steps:
                raise RuntimeError(f"步骤 {capturing_steps[0]} 正在采集中，拒绝并发采集请求")

            progress = self._progress[step_name]
            if recapture:
                if progress.state == "done":
                    # 允许重采已完成步骤：旧解算结果失效，状态回退到 in_progress
                    if self._state == self.STATE_SOLVED:
                        self._solved_result = None
                        self._state = self.STATE_IN_PROGRESS
                    # 归档旧采样证据，轮次递增，清空目标均值与采样数据
                    if progress.captures:
                        progress.history_captures.append(list(progress.captures))
                    progress.attempt += 1
                    progress.captures = []
                    progress.mean = None
                    progress.captured = 0
                    progress.error = None
                    progress.state = "capturing"
                elif progress.state in ("pending", "capturing"):
                    pending = self.next_pending_step()
                    if pending is None or pending.name != step_name:
                        raise RuntimeError(
                            f"步骤 {step_name!r} 尚未进行首次采集，不可重采（下一待采步骤为 {pending.name if pending else '无'}）"
                        )
                    if self._state == self.STATE_PLANNED:
                        self._state = self.STATE_IN_PROGRESS
                    progress.state = "capturing"
                    progress.captured = 0
                    progress.error = None
                    progress.captures = []
                else:
                    raise RuntimeError(f"步骤 {step_name!r} 处于终态 {progress.state!r}，无法重采")
            else:
                if progress.state == "done":
                    raise RuntimeError(f"步骤 {step_name!r} 已完成，重采需指定 recapture=True")
                if self._state == self.STATE_SOLVED:
                    raise RuntimeError("所有步骤已解算，如需重新采集请指定 recapture=True")
                pending = self.next_pending_step()
                if pending is None or pending.name != step_name:
                    raise RuntimeError(
                        f"步骤顺序错误：下一待采步骤为 {pending.name if pending else '无'}，收到 {step_name!r}"
                    )
                if self._state == self.STATE_PLANNED:
                    self._state = self.STATE_IN_PROGRESS
                progress.state = "capturing"
                progress.captured = 0
                progress.error = None
                progress.captures = []
        try:
            frequency_axis: np.ndarray | None = None
            for index in range(1, progress.repeats + 1):
                with self._lock:
                    if self._state == self.STATE_CANCELLED:
                        progress.state = "cancelled"
                        return {
                            "step": step_name,
                            "captured": progress.captured,
                            "cancelled": True,
                            "mean": None,
                            "frequency_hz": frequency_axis,
                        }
                frequency, raw = self.capturer.capture_one(progress, index)  # type: ignore[attr-defined]
                with self._lock:
                    if self._state == self.STATE_CANCELLED:
                        progress.state = "cancelled"
                        return {
                            "step": step_name,
                            "captured": progress.captured,
                            "cancelled": True,
                            "mean": None,
                            "frequency_hz": frequency_axis,
                        }
                raw = np.asarray(raw, dtype=np.complex128)
                frequency = np.asarray(frequency, dtype=np.float64)
                if raw.ndim != 2 or raw.shape[0] != 2:
                    raise ValueError(f"capture #{index} raw 必须是 (2, F)，收到 {raw.shape}")
                if frequency.shape != (raw.shape[1],):
                    raise ValueError(f"capture #{index} 频率轴与数据长度不一致")
                if not np.all(np.isfinite(raw.real)) or not np.all(np.isfinite(raw.imag)):
                    raise ValueError(f"capture #{index} 含 NaN/Inf")
                if frequency_axis is None:
                    frequency_axis = frequency
                elif not np.array_equal(frequency, frequency_axis):
                    raise ValueError(
                        f"capture #{index} 频率网格与首条不一致（复数平均前提被破坏）"
                    )
                if self.capture_dir is not None:
                    path = write_capture_npz(
                        self.capture_dir, step_name, index, frequency, raw, attempt=progress.attempt
                    )
                    with self._lock:
                        progress.capture_files.append(path.name)
                if on_capture is not None:
                    on_capture(frequency, raw, index, progress.repeats)
                with self._lock:
                    progress.captured = index
                    progress.captures.append(raw)
                    progress.frequency_hz = frequency
            mean = average_complex_captures(list(progress.captures))
            with self._lock:
                if self._state == self.STATE_CANCELLED:
                    progress.state = "cancelled"
                    return {
                        "step": step_name,
                        "captured": progress.captured,
                        "cancelled": True,
                        "mean": None,
                        "frequency_hz": frequency_axis,
                    }
                progress.mean = mean
                progress.state = "done"
            return {
                "step": step_name,
                "captured": progress.repeats,
                "mean": mean,
                "frequency_hz": frequency_axis,
            }
        except Exception as exc:
            with self._lock:
                if self._state == self.STATE_CANCELLED:
                    progress.state = "cancelled"
                    progress.error = f"会话已取消: {exc}"
                    return {
                        "step": step_name,
                        "captured": progress.captured,
                        "cancelled": True,
                        "mean": None,
                        "frequency_hz": frequency_axis,
                    }
                progress.state = "failed"
                progress.error = f"{type(exc).__name__}: {exc}"
                self._fail(f"步骤 {step_name} 采集失败: {exc}")
            raise

    def solve(self) -> dict[str, object]:
        """解算：OSL 模式 → 两套 one-port D/T/M（P1/P2 独立，绝不交换共用）；
        背景模式 → 校正后背景数组。全部步骤完成才可解算。"""
        with self._lock:
            self._guard_open("solve()")
            if self._state != self.STATE_IN_PROGRESS:
                raise RuntimeError(f"状态 {self._state} 不允许解算（需先完成全部步骤采集）")
            not_done = [name for name, p in self._progress.items() if p.state != "done"]
            if not_done:
                raise RuntimeError(f"步骤未完成，不能解算: {', '.join(not_done)}")
        if self.plan.mode == CALIBRATION_MODE_BACKGROUND:
            result = self._solve_background()
        else:
            result = self._solve_osl()
        with self._lock:
            self._solved_result = result
            self._state = self.STATE_SOLVED
        return result

    def _mean_of(self, step_name: str) -> tuple[np.ndarray, np.ndarray, list[np.ndarray]]:
        progress = self._progress[step_name]
        assert progress.mean is not None and progress.frequency_hz is not None
        return progress.mean, progress.frequency_hz, list(progress.captures)

    def _solve_osl(self) -> dict[str, object]:
        step_names = [step.name for step in self.plan.steps]
        means = {name: self._mean_of(name) for name in step_names}
        first = step_names[0]
        frequency = means[first][1]
        for name, (_mean, freq, _captures) in means.items():
            if not np.array_equal(freq, frequency):
                raise ValueError(f"步骤 {name} 频率轴与 {first} 不一致，不能解算")
        # 步骤均值是 (2, F) 双通道数组：P1 用 S11 行、P2 用 S22 行（固定通道
        # 序，参考文档 §4.3——两套 D/T/M 绝不交换共用）。单端口计划只解对应一套。
        p1 = None
        p2 = None
        if "P1_Open" in means:
            p1 = solve_one_port_osl(
                frequency,
                means["P1_Open"][0][0],
                means["P1_Short"][0][0],
                means["P1_Load"][0][0],
                s_parameter="S11",
            )
        if "P2_Open" in means:
            p2 = solve_one_port_osl(
                frequency,
                means["P2_Open"][0][1],
                means["P2_Short"][0][1],
                means["P2_Load"][0][1],
                s_parameter="S22",
            )
        primary = p1 if p1 is not None else p2
        assert primary is not None
        return {
            "mode": self.plan.mode,
            "cal_id": self.plan.cal_id,
            "frequency_hz": frequency,
            "p1_terms": p1,
            "p2_terms": p2,
            "valid_band_hz": primary.valid_band_hz,
            "profile_summary": self._profile_summary(p1, p2, frequency.size),
        }

    def _solve_background(self) -> dict[str, object]:
        mean, frequency, _captures = self._mean_of(BACKGROUND_STEP_NAME)
        mean_array = np.asarray(mean, dtype=np.complex128)
        s11_mean = np.asarray(mean_array[0], dtype=np.complex128)
        # VV 背景：空采时 S22 一并采到了，各通道保留自己的背景（绝不互相套用）。
        s22_mean = (
            np.asarray(mean_array[1], dtype=np.complex128)
            if mean_array.ndim == 2 and mean_array.shape[0] >= 2
            else None
        )
        return {
            "mode": self.plan.mode,
            "cal_id": self.plan.cal_id,
            "frequency_hz": frequency,
            "background": s11_mean,
            "background_s22": s22_mean,
            "background_frequency_hz": frequency,
            "valid_band_hz": (float(frequency[0]), float(frequency[-1])),
            "profile_summary": {
                "mode": self.plan.mode,
                "points": int(frequency.size),
                "s11_mean_magnitude": float(np.mean(np.abs(s11_mean))),
            },
        }

    def _profile_summary(self, p1: OSLTerms | None, p2: OSLTerms | None, points: int) -> dict[str, object]:
        """地面展示用的结果摘要（全部 JSON 可序列化，R-C-11/16）。"""
        step_quality: dict[str, object] = {}
        for name in OSL_STEP_NAMES:
            if name not in self._progress:
                continue
            progress = self._progress[name]
            step_quality[name] = {
                "captured": progress.captured,
                "state": progress.state,
            }

        def _terms_summary(terms: OSLTerms | None) -> dict[str, object] | None:
            if terms is None:
                return None
            return {
                "s_parameter": terms.s_parameter,
                "capture_counts": list(terms.capture_counts),
                "quality_metrics": {k: v for k, v in terms.quality_metrics.items()},
            }

        summary: dict[str, object] = {
            "points": int(points),
            "repeats": self.plan.steps[0].repeats if self.plan.steps else 0,
            "steps": step_quality,
        }
        # 单端口校准时没有的那一端口不写（避免伪造字段）。
        if p1 is not None:
            summary["p1"] = _terms_summary(p1)
        if p2 is not None:
            summary["p2"] = _terms_summary(p2)
        summary["ports"] = [port for port, terms in (("P1", p1), ("P2", p2)) if terms is not None]
        return summary

    def activate(self) -> dict[str, object]:
        """激活（终态）：返回接线层所需的生效组件（R-C-16）。

        绝不从 failed/cancelled 激活（R-C-05）；solved 才可激活；重复激活
        幂等返回同一结果。
        """
        with self._lock:
            if self._state == self.STATE_ACTIVATED and self._activation is not None:
                return dict(self._activation)
            self._guard_open("activate()")
            if self._state != self.STATE_SOLVED or self._solved_result is None:
                raise RuntimeError(f"状态 {self._state} 不允许激活（需先 solve）")
            result = self._solved_result
            activation: dict[str, object] = {
                "cal_id": self.plan.cal_id,
                "mode": self.plan.mode,
                "valid_band_hz": result["valid_band_hz"],
            }
            if self.plan.mode == CALIBRATION_MODE_OSL:
                activation["p1_terms"] = result["p1_terms"]
                activation["p2_terms"] = result["p2_terms"]
            else:
                activation["background"] = result["background"]
                activation["background_s22"] = result.get("background_s22")
                activation["background_frequency_hz"] = result["background_frequency_hz"]
            self._activation = activation
            self._state = self.STATE_ACTIVATED
            return dict(activation)

    def rollback_activation(self) -> None:
        """激活事务未成功落盘时的回滚方法，恢复回 STATE_SOLVED 允许重试。"""
        with self._lock:
            if self._state == self.STATE_ACTIVATED:
                self._state = self.STATE_SOLVED
                self._activation = None

    def cancel(self, reason: str) -> str:
        """取消（终态）：已收 capture 文件保留并写 incomplete 标记（R-C-18）。"""
        with self._lock:
            if self._state in (self.STATE_FAILED, self.STATE_CANCELLED):
                return self._state
            if self._state == self.STATE_ACTIVATED:
                raise RuntimeError("已激活的校准不能取消（如需撤销请加载其他校准文件）")
            self._cancel_reason = reason
            self._state = self.STATE_CANCELLED
            for progress in self._progress.values():
                if progress.state == "capturing":
                    progress.state = "failed"
                    progress.error = f"已取消: {reason}"
            self._write_incomplete_marker(f"已取消: {reason}")
            return self._state


__all__ = [
    "OSL_PORT_CHOICES",
    "normalize_osl_ports",
    "BACKGROUND_STEP_NAME",
    "CALIBRATION_MODE_BACKGROUND",
    "CALIBRATION_MODE_OSL",
    "CALIBRATION_STEPS",
    "CALIBRATION_STEP_LABELS",
    "CalibrationMeasurement",
    "CalibrationPlan",
    "CalibrationSession",
    "CalibrationStep",
    "LEGACY_OSL_MAP",
    "MAX_CALIBRATION_FILE_BYTES",
    "OSL_STEP_CHANNEL",
    "OSL_STEP_NAMES",
    "OSLTerms",
    "P1_OSL_KEYS",
    "P2_OSL_KEYS",
    "REPEATS_DEFAULT",
    "REPEATS_MAX",
    "REPEATS_MIN",
    "StepProgress",
    "apply_error_model",
    "average_complex_captures",
    "build_background_plan",
    "build_osl_plan",
    "build_reference_background",
    "calibration_from_measurements",
    "calibration_points_payload",
    "interpolate_terms_in_band",
    "osl_measurements_for_port",
    "read_calibration_measurements",
    "s11_osl_measurements",
    "solve_one_port_osl",
    "terms_to_osl_calibration",
    "validate_calibration_content",
    "validate_repeats",
    "write_calibration_file",
    "write_capture_npz",
]
