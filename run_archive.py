"""整次采集合并与离线重处理（纯 numpy，不依赖 Qt）。

用途（2026-10-22 用户需求）：
1. 采集结束后，把一个 run 文件夹里一条一条的 ``trace_*.npz`` 合并成一个文件
   ``<run_id>_merged.npz``（**额外多做一步**，原始小文件一律保留）；
2. 地面站离线回看窗口读取历史采集（合并文件或原始文件夹），重新应用
   Dewow / Flat Filter 后处理。

设计要点（对应给用户的“踩坑清单”）：
- 合并在地面端做；地面 ``remote_runs/<run_id>/trace_00001.npz`` 与空中端
  ``gpr_runs/<run_id>/trace_00001_<时间戳>.npz`` 两种命名都认。
- 断线补发可能乱序 / 重复 / 缺号：按 trace 编号排序、去重，缺号写进
  ``missing_trace_indices`` 并在报告里列出。
- 采集中空中端拒绝改参数，所以同一 run 频率/通道/时间轴应一致；不一致的
  文件跳过并记账（不猜、不硬拼）。
- 先写临时文件再 ``os.replace`` 原子改名，半截文件不会冒充合并结果。
- 离线重处理从 ``display_*``（空中端已做完校准 + 背景扣除的频域数据）出发，
  重新做 BPF → IFFT →（可选）Dewow，与空中端 ``air_core.make_time_trace``
  使用同一组 ``gpr_processing`` 函数、同一参数，保证“同参数重算 = 实时结果”。
  老文件没有 ``display_*`` 时退回用已存的时域 trace。
- 校准系数与空采背景目前**不在** trace 里（只有 calibration_id），第一版
  离线处理不需要它们；“换背景”留待后续。
"""

from __future__ import annotations

import json
import math
import os
import re
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import numpy as np

from gpr_processing import apply_frequency_bpf, dewow, flat_reflection_filter, frequency_to_time_trace

MERGED_FORMAT = "uav_gpr_run_merged_v1"
MERGED_SUFFIX = "_merged.npz"
PROCESSED_FORMAT = "uav_gpr_run_processed_v1"
CHANNEL_ORDER = ("S11", "S22", "S21", "S12")
CHANNEL_LABELS = {"S11": "HH", "S22": "VV", "S21": "S21", "S12": "S12"}

_TRACE_NAME = re.compile(r"^trace_(\d+)(?:_[0-9_]+)?\.npz$", re.IGNORECASE)

# 默认处理参数（与 air_core.SweepConfig 默认一致，仅在文件缺 config 时兜底）。
_DEFAULT_IFFT_POINTS = 2048
_DEFAULT_TIME_WINDOW_NS = 40.0
_DEFAULT_DEWOW_WINDOW_NS = 4.0


class RunArchiveError(ValueError):
    """合并 / 读取失败（给界面显示的中文原因）。"""


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------


@dataclass
class RunData:
    """一次采集（合并后）在内存里的样子。数组第 0 维都是 trace（按编号升序）。"""

    run_id: str
    source: str
    channels: tuple[str, ...]
    trace_index: np.ndarray  # (N,) int64
    frequency_hz: np.ndarray  # (F,)
    time_ns: np.ndarray  # (T,)
    raw: dict[str, np.ndarray] = field(default_factory=dict)  # ch -> (N, F) complex
    display: dict[str, np.ndarray] = field(default_factory=dict)  # ch -> (N, F) complex
    trace: dict[str, np.ndarray] = field(default_factory=dict)  # ch -> (N, T) complex
    gnss_json: list[str] = field(default_factory=list)
    acquisition_json: list[str] = field(default_factory=list)
    config: dict[str, object] = field(default_factory=dict)
    processing: dict[str, object] = field(default_factory=dict)
    missing_trace_indices: list[int] = field(default_factory=list)
    duplicate_trace_indices: list[int] = field(default_factory=list)
    skipped_files: list[dict[str, str]] = field(default_factory=list)

    @property
    def n_traces(self) -> int:
        return int(self.trace_index.size)

    def gnss_arrays(self) -> dict[str, np.ndarray]:
        """把逐道 GNSS JSON 展开成数组（缺值为 NaN / 0），方便 MATLAB/Python 直接用。"""
        n = len(self.gnss_json)
        lat = np.full(n, np.nan)
        lon = np.full(n, np.nan)
        alt = np.full(n, np.nan)
        quality = np.zeros(n, dtype=np.int64)
        sats = np.zeros(n, dtype=np.int64)
        utc: list[str] = []
        for i, text in enumerate(self.gnss_json):
            try:
                obj = json.loads(text) if text else {}
            except ValueError:
                obj = {}
            if not isinstance(obj, dict):
                obj = {}
            lat[i] = _float_or_nan(obj.get("latitude"))
            lon[i] = _float_or_nan(obj.get("longitude"))
            alt[i] = _float_or_nan(obj.get("altitude_m"))
            quality[i] = _int_or_zero(obj.get("fix_quality"))
            sats[i] = _int_or_zero(obj.get("satellites"))
            utc.append(str(obj.get("utc", "") or ""))
        return {
            "gnss_latitude": lat,
            "gnss_longitude": lon,
            "gnss_altitude_m": alt,
            "gnss_fix_quality": quality,
            "gnss_satellites": sats,
            "gnss_utc": np.asarray(utc, dtype=str) if utc else np.zeros(0, dtype="<U1"),
        }


@dataclass
class MergeReport:
    run_id: str
    output_path: Path | None
    n_traces: int
    channels: tuple[str, ...]
    missing_trace_indices: list[int]
    duplicate_trace_indices: list[int]
    skipped_files: list[dict[str, str]]

    def summary_text(self) -> str:
        """给用户看的一段中文总结。"""
        lines = [f"采集 {self.run_id}：合并 {self.n_traces} 道，通道 "
                 + "+".join(CHANNEL_LABELS.get(c, c) for c in self.channels)]
        if self.missing_trace_indices:
            lines.append("缺号 " + format_index_ranges(self.missing_trace_indices)
                         + f"（共 {len(self.missing_trace_indices)} 道）")
        else:
            lines.append("编号连续，无缺号")
        if self.duplicate_trace_indices:
            lines.append(f"重复编号已去重：{format_index_ranges(self.duplicate_trace_indices)}")
        if self.skipped_files:
            lines.append(f"跳过 {len(self.skipped_files)} 个文件（损坏或参数不一致）")
        if self.output_path is not None:
            lines.append(f"已保存：{self.output_path}")
        return "；".join(lines)


@dataclass
class ProcessingParams:
    dewow_enabled: bool = False
    dewow_window_ns: float = _DEFAULT_DEWOW_WINDOW_NS
    flat_enabled: bool = False
    flat_window_traces: int = 31

    def as_dict(self) -> dict[str, object]:
        return {
            "dewow_enabled": bool(self.dewow_enabled),
            "dewow_window_ns": float(self.dewow_window_ns),
            "flat_enabled": bool(self.flat_enabled),
            "flat_window_traces": int(self.flat_window_traces),
        }


@dataclass
class ProcessedRun:
    time_ns: np.ndarray
    matrices: dict[str, np.ndarray]  # ch -> (T, N) complex，列 = trace
    params: ProcessingParams
    source_kind: str  # "display"（从频域重算）或 "stored_trace"（老文件退回）
    notes: list[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------


def _float_or_nan(value: object) -> float:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _int_or_zero(value: object) -> int:
    if isinstance(value, bool):
        return int(value)
    try:
        return int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return 0


def format_index_ranges(indices: Iterable[int]) -> str:
    """[1,2,3,7,9,10] -> '1–3, 7, 9–10'（太长时截断）。"""
    values = sorted(set(int(v) for v in indices))
    if not values:
        return ""
    parts: list[str] = []
    start = prev = values[0]
    for value in values[1:]:
        if value == prev + 1:
            prev = value
            continue
        parts.append(f"{start}" if start == prev else f"{start}–{prev}")
        start = prev = value
    parts.append(f"{start}" if start == prev else f"{start}–{prev}")
    if len(parts) > 12:
        parts = parts[:12] + ["…"]
    return ", ".join(parts)


def _json_obj(data: "np.lib.npyio.NpzFile", key: str) -> dict[str, object]:
    if key not in data.files:
        return {}
    try:
        value = json.loads(str(data[key]))
    except (TypeError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def _json_text(data: "np.lib.npyio.NpzFile", key: str) -> str:
    if key not in data.files:
        return ""
    try:
        return str(data[key])
    except (TypeError, ValueError):
        return ""


def _declared_channels(data: "np.lib.npyio.NpzFile") -> tuple[str, ...]:
    if "channels" in data.files:
        try:
            parsed = json.loads(str(data["channels"]))
        except (TypeError, ValueError):
            parsed = None
        if isinstance(parsed, list) and parsed and all(isinstance(c, str) for c in parsed):
            return tuple(c for c in CHANNEL_ORDER if c in parsed)
    # 老文件：按实际字段推断。
    found = []
    for name in CHANNEL_ORDER:
        if f"trace_{name}" in data.files or f"display_{name}_real" in data.files:
            found.append(name)
    if not found and "trace" in data.files:
        found.append("S11")  # 最早期只存单通道 trace
    return tuple(found)


def is_merged_file(path: Path) -> bool:
    return Path(path).name.lower().endswith(MERGED_SUFFIX)


def list_trace_files(run_dir: Path) -> list[tuple[int, Path]]:
    """列出文件夹里的单道文件 (编号, 路径)，按编号、文件名排序。"""
    run_dir = Path(run_dir)
    if not run_dir.is_dir():
        raise RunArchiveError(f"不是文件夹：{run_dir}")
    items: list[tuple[int, Path]] = []
    for path in run_dir.iterdir():
        if not path.is_file():
            continue
        match = _TRACE_NAME.match(path.name)
        if match is None:
            continue
        items.append((int(match.group(1)), path))
    items.sort(key=lambda item: (item[0], item[1].name))
    return items


def merged_path_for(run_dir: Path) -> Path:
    run_dir = Path(run_dir)
    return run_dir / f"{run_dir.name}{MERGED_SUFFIX}"


# ---------------------------------------------------------------------------
# 读取单道 + 合并
# ---------------------------------------------------------------------------


def _read_trace_file(path: Path) -> dict[str, object]:
    with np.load(path, allow_pickle=False) as data:
        files = set(data.files)
        if "frequency_hz" not in files or "time_ns" not in files:
            raise RunArchiveError("缺少 frequency_hz 或 time_ns")
        channels = _declared_channels(data)
        if not channels:
            raise RunArchiveError("文件里没有任何通道数据")
        freq = np.asarray(data["frequency_hz"], dtype=np.float64)
        time_ns = np.asarray(data["time_ns"], dtype=np.float64)
        record: dict[str, object] = {
            "channels": channels,
            "frequency_hz": freq,
            "time_ns": time_ns,
            "config": _json_obj(data, "config"),
            "processing": _json_obj(data, "processing"),
            "gnss": _json_text(data, "gnss"),
            "acquisition": _json_text(data, "acquisition"),
            "raw": {},
            "display": {},
            "trace": {},
        }
        for name in channels:
            if f"raw_{name}_real" in files and f"raw_{name}_imag" in files:
                record["raw"][name] = (  # type: ignore[index]
                    np.asarray(data[f"raw_{name}_real"], dtype=np.float64)
                    + 1j * np.asarray(data[f"raw_{name}_imag"], dtype=np.float64)
                )
            if f"display_{name}_real" in files and f"display_{name}_imag" in files:
                record["display"][name] = (  # type: ignore[index]
                    np.asarray(data[f"display_{name}_real"], dtype=np.float64)
                    + 1j * np.asarray(data[f"display_{name}_imag"], dtype=np.float64)
                )
            if f"trace_{name}_complex" in files:
                trace = np.asarray(data[f"trace_{name}_complex"], dtype=np.complex128)
            elif f"trace_{name}" in files:
                trace = np.asarray(data[f"trace_{name}"], dtype=np.float64).astype(np.complex128)
            elif name == "S11" and "trace" in files:
                trace = np.asarray(data["trace"], dtype=np.float64).astype(np.complex128)
            else:
                raise RunArchiveError(f"通道 {name} 缺少时域 trace")
            if trace.shape != time_ns.shape:
                raise RunArchiveError(f"通道 {name} trace 长度与 time_ns 不一致")
            record["trace"][name] = trace  # type: ignore[index]
    return record


def _stack(rows: list[np.ndarray]) -> np.ndarray:
    return np.ascontiguousarray(np.vstack([np.asarray(r)[None, :] for r in rows]))


def build_run_from_folder(run_dir: Path) -> RunData:
    """把一个 run 文件夹读成 RunData（不写盘）。"""
    run_dir = Path(run_dir)
    files = list_trace_files(run_dir)
    if not files:
        raise RunArchiveError(f"文件夹里没有 trace_*.npz：{run_dir}")

    skipped: list[dict[str, str]] = []
    duplicates: list[int] = []
    kept: list[tuple[int, dict[str, object]]] = []
    seen: set[int] = set()
    reference: dict[str, object] | None = None

    for index, path in files:
        if index <= 0:
            skipped.append({"file": path.name, "reason": "编号非法"})
            continue
        if index in seen:
            duplicates.append(index)
            skipped.append({"file": path.name, "reason": f"重复编号 {index}（已保留第一份）"})
            continue
        try:
            record = _read_trace_file(path)
        except Exception as exc:  # 损坏文件只跳过，不让整次合并失败
            skipped.append({"file": path.name, "reason": f"读取失败：{exc}"})
            continue
        if reference is None:
            reference = record
        else:
            reason = _consistency_problem(reference, record)
            if reason:
                skipped.append({"file": path.name, "reason": reason})
                continue
        seen.add(index)
        kept.append((index, record))

    if not kept or reference is None:
        raise RunArchiveError("没有可用的 trace（全部损坏或参数不一致）")

    indices = np.asarray([i for i, _ in kept], dtype=np.int64)
    channels: tuple[str, ...] = reference["channels"]  # type: ignore[assignment]
    # 上界取文件夹里出现过的最大编号（被跳过的坏文件也算“该有但用不了”）。
    highest = max(int(indices.max()), max((i for i, _ in files if i > 0), default=0))
    full = set(range(1, highest + 1))
    missing = sorted(full - set(int(i) for i in indices))

    run = RunData(
        run_id=run_dir.name,
        source=str(run_dir),
        channels=channels,
        trace_index=indices,
        frequency_hz=np.asarray(reference["frequency_hz"]),
        time_ns=np.asarray(reference["time_ns"]),
        gnss_json=[str(r["gnss"]) for _, r in kept],
        acquisition_json=[str(r["acquisition"]) for _, r in kept],
        config=dict(reference["config"]),  # type: ignore[arg-type]
        processing=dict(reference["processing"]),  # type: ignore[arg-type]
        missing_trace_indices=missing,
        duplicate_trace_indices=sorted(set(duplicates)),
        skipped_files=skipped,
    )
    for name in channels:
        run.trace[name] = _stack([r["trace"][name] for _, r in kept])  # type: ignore[index]
        if all(name in r["raw"] for _, r in kept):  # type: ignore[operator]
            run.raw[name] = _stack([r["raw"][name] for _, r in kept])  # type: ignore[index]
        if all(name in r["display"] for _, r in kept):  # type: ignore[operator]
            run.display[name] = _stack([r["display"][name] for _, r in kept])  # type: ignore[index]
    return run


def _consistency_problem(reference: dict[str, object], record: dict[str, object]) -> str:
    if tuple(record["channels"]) != tuple(reference["channels"]):  # type: ignore[arg-type]
        return f"通道不一致：{record['channels']} ≠ {reference['channels']}"
    ref_f = np.asarray(reference["frequency_hz"])
    cur_f = np.asarray(record["frequency_hz"])
    if ref_f.shape != cur_f.shape or not np.allclose(ref_f, cur_f, rtol=0, atol=1e-3):
        return "频率点不一致"
    ref_t = np.asarray(reference["time_ns"])
    cur_t = np.asarray(record["time_ns"])
    if ref_t.shape != cur_t.shape or not np.allclose(ref_t, cur_t, rtol=1e-9, atol=1e-9):
        return "时间轴不一致"
    return ""


def save_merged(run: RunData, out_path: Path) -> Path:
    """原子写合并文件（先写 .tmp 再改名）。"""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    payload: dict[str, object] = {
        "format": MERGED_FORMAT,
        "run_id": run.run_id,
        "source": run.source,
        "merged_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "channels": json.dumps(list(run.channels), ensure_ascii=False),
        "trace_index": run.trace_index,
        "frequency_hz": run.frequency_hz,
        "time_ns": run.time_ns,
        "config": json.dumps(run.config, ensure_ascii=False),
        "processing": json.dumps(run.processing, ensure_ascii=False),
        "gnss_json": np.asarray(run.gnss_json, dtype=str),
        "acquisition_json": np.asarray(run.acquisition_json, dtype=str),
        "missing_trace_indices": np.asarray(run.missing_trace_indices, dtype=np.int64),
        "duplicate_trace_indices": np.asarray(run.duplicate_trace_indices, dtype=np.int64),
        "skipped_files": json.dumps(run.skipped_files, ensure_ascii=False),
    }
    payload.update(run.gnss_arrays())
    for name in run.channels:
        payload[f"trace_{name}_complex"] = run.trace[name]
        if name in run.raw:
            payload[f"raw_{name}"] = run.raw[name]
        if name in run.display:
            payload[f"display_{name}"] = run.display[name]
    tmp_path = out_path.with_name(out_path.name + ".tmp")
    try:
        with tmp_path.open("wb") as handle:
            np.savez_compressed(handle, **payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_path, out_path)
    finally:
        if tmp_path.exists():
            try:
                tmp_path.unlink()
            except OSError:
                pass
    return out_path


def merge_run(run_dir: Path, out_path: Path | None = None) -> MergeReport:
    """合并一个 run 文件夹 → ``<run_dir>/<run_id>_merged.npz``。原始文件不动。"""
    run = build_run_from_folder(Path(run_dir))
    target = Path(out_path) if out_path is not None else merged_path_for(Path(run_dir))
    save_merged(run, target)
    return MergeReport(
        run_id=run.run_id,
        output_path=target,
        n_traces=run.n_traces,
        channels=run.channels,
        missing_trace_indices=run.missing_trace_indices,
        duplicate_trace_indices=run.duplicate_trace_indices,
        skipped_files=run.skipped_files,
    )


def load_merged(path: Path) -> RunData:
    path = Path(path)
    with np.load(path, allow_pickle=False) as data:
        files = set(data.files)
        if "format" not in files or str(data["format"]) != MERGED_FORMAT:
            raise RunArchiveError(f"不是合并文件（format 不是 {MERGED_FORMAT}）：{path.name}")
        try:
            channels = tuple(json.loads(str(data["channels"])))
        except (KeyError, ValueError) as exc:
            raise RunArchiveError("合并文件缺少 channels") from exc
        run = RunData(
            run_id=str(data["run_id"]) if "run_id" in files else path.stem,
            source=str(path),
            channels=channels,
            trace_index=np.asarray(data["trace_index"], dtype=np.int64),
            frequency_hz=np.asarray(data["frequency_hz"], dtype=np.float64),
            time_ns=np.asarray(data["time_ns"], dtype=np.float64),
            gnss_json=[str(v) for v in data["gnss_json"]] if "gnss_json" in files else [],
            acquisition_json=[str(v) for v in data["acquisition_json"]] if "acquisition_json" in files else [],
            config=_json_obj(data, "config"),
            processing=_json_obj(data, "processing"),
            missing_trace_indices=[int(v) for v in data["missing_trace_indices"]] if "missing_trace_indices" in files else [],
            duplicate_trace_indices=[int(v) for v in data["duplicate_trace_indices"]] if "duplicate_trace_indices" in files else [],
        )
        if "skipped_files" in files:
            try:
                skipped = json.loads(str(data["skipped_files"]))
                if isinstance(skipped, list):
                    run.skipped_files = skipped
            except ValueError:
                pass
        for name in channels:
            run.trace[name] = np.asarray(data[f"trace_{name}_complex"], dtype=np.complex128)
            if f"raw_{name}" in files:
                run.raw[name] = np.asarray(data[f"raw_{name}"], dtype=np.complex128)
            if f"display_{name}" in files:
                run.display[name] = np.asarray(data[f"display_{name}"], dtype=np.complex128)
    return run


def load_run(path: Path) -> RunData:
    """打开历史采集：合并文件、run 文件夹，或文件夹里任意一个单道文件都行。

    文件夹里已有合并文件、且没有比它更新的单道文件时直接读合并文件（快）；
    否则现场从单道文件组装（不写盘）。
    """
    path = Path(path)
    if path.is_file():
        if is_merged_file(path):
            return load_merged(path)
        if _TRACE_NAME.match(path.name):
            path = path.parent
        else:
            raise RunArchiveError(f"认不出的文件：{path.name}")
    if not path.is_dir():
        raise RunArchiveError(f"找不到：{path}")
    merged = merged_path_for(path)
    if merged.exists():
        merged_mtime = merged.stat().st_mtime
        newest = max((p.stat().st_mtime for _, p in list_trace_files(path)), default=0.0)
        if newest <= merged_mtime:
            return load_merged(merged)
    return build_run_from_folder(path)


# ---------------------------------------------------------------------------
# 离线重处理
# ---------------------------------------------------------------------------


def _config_number(config: dict[str, object], key: str, default: float) -> float:
    value = _float_or_nan(config.get(key))
    return default if math.isnan(value) else value


def spectrum_to_trace(
    frequency_hz: np.ndarray,
    spectrum: np.ndarray,
    *,
    ifft_points: int,
    time_window_ns: float,
    dewow_enabled: bool,
    dewow_window_ns: float,
) -> tuple[np.ndarray, np.ndarray]:
    """与 ``air_core.make_time_trace`` 完全相同的一道处理：BPF → IFFT →（可选）Dewow。"""
    filtered = apply_frequency_bpf(frequency_hz, spectrum)
    return frequency_to_time_trace(
        frequency_hz,
        filtered,
        ifft_points=int(ifft_points),
        time_window_ns=float(time_window_ns),
        dewow_window_ns=float(dewow_window_ns),
        apply_dewow=bool(dewow_enabled),
    )


def stored_dewow_enabled(run: RunData) -> bool:
    dewow_info = run.processing.get("dewow") if isinstance(run.processing, dict) else None
    if isinstance(dewow_info, dict) and isinstance(dewow_info.get("enabled"), bool):
        return bool(dewow_info["enabled"])
    value = run.processing.get("dewow_enabled") if isinstance(run.processing, dict) else None
    if isinstance(value, bool):
        return value
    return bool(run.config.get("dewow_enabled", False))


def reprocess(run: RunData, params: ProcessingParams) -> ProcessedRun:
    """按新参数重算整次采集。返回每通道 (T, N) 复数矩阵（列 = trace）。"""
    notes: list[str] = []
    ifft_points = int(_config_number(run.config, "ifft_points", _DEFAULT_IFFT_POINTS))
    time_window_ns = _config_number(run.config, "time_window_ns", _DEFAULT_TIME_WINDOW_NS)
    matrices: dict[str, np.ndarray] = {}
    time_ns: np.ndarray | None = None
    use_display = all(name in run.display for name in run.channels)

    if use_display:
        source_kind = "display"
        for name in run.channels:
            spectra = run.display[name]
            columns: list[np.ndarray] = []
            for row in spectra:
                t_ns, trace = spectrum_to_trace(
                    run.frequency_hz,
                    row,
                    ifft_points=ifft_points,
                    time_window_ns=time_window_ns,
                    dewow_enabled=params.dewow_enabled,
                    dewow_window_ns=params.dewow_window_ns,
                )
                if time_ns is None:
                    time_ns = np.asarray(t_ns)
                columns.append(np.asarray(trace, dtype=np.complex128))
            matrices[name] = np.column_stack(columns) if columns else np.zeros((0, 0), complex)
    else:
        source_kind = "stored_trace"
        notes.append("老文件没有校准后的频域数据，改用已存的时域波形")
        time_ns = np.asarray(run.time_ns)
        already = stored_dewow_enabled(run)
        for name in run.channels:
            matrix = np.asarray(run.trace[name], dtype=np.complex128).T.copy()
            if params.dewow_enabled and not already and time_ns.size >= 2:
                dt_s = float(time_ns[1] - time_ns[0]) * 1e-9
                matrix = dewow(matrix, dt_s=dt_s, window_ns=params.dewow_window_ns, time_axis=0)
            elif params.dewow_enabled and already:
                notes.append("该文件采集时已做过 Dewow，不重复做")
            elif not params.dewow_enabled and already:
                notes.append("该文件采集时已做过 Dewow，老格式无法撤销")
            matrices[name] = matrix

    if params.flat_enabled:
        window = int(params.flat_window_traces)
        if window % 2 == 0:
            window -= 1
        window = max(window, 3)
        for name, matrix in list(matrices.items()):
            if matrix.shape[1] >= window:
                matrices[name] = flat_reflection_filter(matrix, window_size=window)
            else:
                notes.append(f"道数 {matrix.shape[1]} 少于 Flat Filter 窗口 {window}，未滤波")
                break

    assert time_ns is not None
    return ProcessedRun(
        time_ns=np.asarray(time_ns),
        matrices=matrices,
        params=params,
        source_kind=source_kind,
        notes=sorted(set(notes), key=notes.index),
    )


def save_processed(run: RunData, result: ProcessedRun, out_path: Path) -> Path:
    """另存离线处理结果（不覆盖原始数据）。"""
    out_path = Path(out_path)
    payload: dict[str, object] = {
        "format": PROCESSED_FORMAT,
        "run_id": run.run_id,
        "source": run.source,
        "processed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "processing_params": json.dumps(result.params.as_dict(), ensure_ascii=False),
        "processing_source": result.source_kind,
        "acquisition_processing": json.dumps(run.processing, ensure_ascii=False),
        "config": json.dumps(run.config, ensure_ascii=False),
        "channels": json.dumps(list(result.matrices), ensure_ascii=False),
        "trace_index": run.trace_index,
        "time_ns": result.time_ns,
    }
    payload.update(run.gnss_arrays())
    for name, matrix in result.matrices.items():
        payload[f"bscan_{name}"] = matrix  # (T, N) 复数
    tmp_path = out_path.with_name(out_path.name + ".tmp")
    with tmp_path.open("wb") as handle:
        np.savez_compressed(handle, **payload)
    os.replace(tmp_path, out_path)
    return out_path
