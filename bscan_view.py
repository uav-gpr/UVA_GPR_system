"""Pure B-scan windowing helpers shared by both GUI applications."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


BSCAN_WINDOW_TRACES = 256
BSCAN_TIME_RTOL = 1e-9
BSCAN_TIME_ATOL = 1e-9


def bscan_processing_signature(
    dewow_enabled: bool,
    dewow_window_ns: float = 4.0,
) -> tuple[bool, float | None]:
    """Return the processing identity that may be mixed in one B-scan.

    Flat Filter is deliberately absent: it is a local display option and does
    not change the stored trace history.  A disabled Dewow window is likewise
    ignored because it has no effect on the trace.
    """
    if not isinstance(dewow_enabled, bool):
        raise TypeError("dewow_enabled must be a bool")
    if not dewow_enabled:
        return False, None
    try:
        window = float(dewow_window_ns)
    except (TypeError, ValueError) as exc:
        raise ValueError("dewow_window_ns must be a finite positive number") from exc
    if not math.isfinite(window) or window <= 0.0:
        raise ValueError("dewow_window_ns must be a finite positive number")
    return True, window


@dataclass(frozen=True)
class BScanWindow:
    start_trace: int
    end_trace: int
    max_start: int
    total_traces: int
    matrix: np.ndarray


def _validated_trace_count(total_traces: int) -> int:
    if isinstance(total_traces, bool) or not isinstance(total_traces, (int, np.integer)):
        raise TypeError("total_traces must be an int or numpy integer")
    total = int(total_traces)
    if total < 0:
        raise ValueError("total_traces must be non-negative")
    return total


def bscan_window_max_start(total_traces: int) -> int:
    """Return the one-based first-trace position of the final 256-trace window."""
    total = _validated_trace_count(total_traces)
    return max(1, total - BSCAN_WINDOW_TRACES + 1)


def bscan_window_bounds(total_traces: int, start_trace: int = 1) -> tuple[int, int]:
    """Return inclusive one-based bounds for a requested display window."""
    total = _validated_trace_count(total_traces)
    if isinstance(start_trace, bool) or not isinstance(start_trace, (int, np.integer)):
        raise TypeError("start_trace must be an int or numpy integer")
    requested = int(start_trace)
    maximum = bscan_window_max_start(total)
    start = min(max(requested, 1), maximum)
    end = min(total, start + BSCAN_WINDOW_TRACES - 1)
    return start, end


def validate_bscan_trace(
    time_ns: np.ndarray,
    channel_traces: dict[str, np.ndarray],
    channels: tuple[str, ...] = ("S11", "S22"),
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Validate one atomic HH/VV B-scan append and return float copies."""
    time_values = np.asarray(time_ns, dtype=np.float64)
    if time_values.ndim != 1 or time_values.size == 0:
        raise ValueError("B-scan time_ns must be a non-empty one-dimensional array")
    if not np.all(np.isfinite(time_values)):
        raise ValueError("B-scan time_ns must contain only finite values")

    validated: dict[str, np.ndarray] = {}
    missing = [name for name in channels if name not in channel_traces]
    if missing:
        raise ValueError(f"B-scan trace is missing channels: {', '.join(missing)}")
    for name in channels:
        values = np.asarray(channel_traces[name], dtype=np.float64)
        if values.ndim != 1:
            raise ValueError(f"B-scan trace {name} must be one-dimensional")
        if values.size != time_values.size:
            raise ValueError(f"B-scan trace {name} length must equal time_ns length")
        if not np.all(np.isfinite(values)):
            raise ValueError(f"B-scan trace {name} must contain only finite values")
        validated[name] = values.copy()
    return time_values.copy(), validated


def bscan_time_axes_compatible(current: np.ndarray, candidate: np.ndarray) -> bool:
    """Compare B-scan time axes with an explicit, strict numerical tolerance."""
    current_values = np.asarray(current, dtype=np.float64)
    candidate_values = np.asarray(candidate, dtype=np.float64)
    return current_values.shape == candidate_values.shape and np.allclose(
        current_values,
        candidate_values,
        rtol=BSCAN_TIME_RTOL,
        atol=BSCAN_TIME_ATOL,
        equal_nan=False,
    )


def bscan_start_after_append(
    current_start: int,
    total_before: int,
    total_after: int,
    follow_latest: bool,
) -> int:
    """Choose the window after appending traces without stealing a history view."""
    before = _validated_trace_count(total_before)
    after = _validated_trace_count(total_after)
    if after < before:
        raise ValueError("total_after must not be less than total_before")
    if follow_latest or current_start >= bscan_window_max_start(before):
        return bscan_window_max_start(after)
    return bscan_window_bounds(after, current_start)[0]


def select_bscan_window(matrix: np.ndarray, start_trace: int = 1) -> BScanWindow:
    """Pad/select a ``[n_samples, n_traces]`` matrix for the fixed display width."""
    values = np.asarray(matrix)
    if values.ndim != 2:
        raise ValueError("B-scan matrix must have shape [n_samples, n_traces]")
    total = int(values.shape[1])
    start, end = bscan_window_bounds(total, start_trace)
    display = np.full(
        (values.shape[0], BSCAN_WINDOW_TRACES),
        np.nan,
        dtype=np.result_type(values.dtype, np.float64),
    )
    count = max(0, end - start + 1)
    if count:
        display[:, :count] = values[:, start - 1:start - 1 + count]
    return BScanWindow(
        start_trace=start,
        end_trace=end,
        max_start=bscan_window_max_start(total),
        total_traces=total,
        matrix=display,
    )


def bscan_window_from_rows(
    rows: list[np.ndarray] | tuple[np.ndarray, ...],
    start_trace: int = 1,
    n_samples: int | None = None,
) -> BScanWindow:
    """直接从历史行列表构造当前 256 道显示窗口（不构造全部历史矩阵）。

    ``rows`` 为按时间顺序追加的 1-D trace 列表（每条长度相同）。本函数只
    切片 ``[start-1, start-1+256)`` 的行并拷贝进 NaN 填充的
    ``[n_samples, 256]`` 矩阵，成本受窗口大小约束，与历史总道数无关。

    ``n_samples`` 缺省时取第一行的长度；不足 256 道时用 NaN 右侧填充；
    空历史返回全 NaN 窗口（``start_trace`` 收敛为 1、``end_trace`` 为 0）。
    返回的 ``matrix`` 总是 ``float64``、形状 ``[n_samples, BSCAN_WINDOW_TRACES]``。
    """
    total = _validated_trace_count(len(rows))
    if rows:
        first = np.asarray(rows[0])
        if first.ndim != 1:
            raise ValueError("B-scan history rows must be one-dimensional traces")
        row_length = int(first.shape[0])
    else:
        row_length = 0
    if n_samples is None:
        n_samples_value = row_length
    else:
        if isinstance(n_samples, bool) or not isinstance(n_samples, (int, np.integer)):
            raise TypeError("n_samples must be an int or numpy integer")
        n_samples_value = int(n_samples)
        if n_samples_value < 0:
            raise ValueError("n_samples must be non-negative")
        if total and n_samples_value != row_length:
            raise ValueError(
                f"n_samples={n_samples_value} 与历史行长度 {row_length} 不一致"
            )
    if total and n_samples_value == 0:
        # 有历史但显式要求 0 采样点：返回空窗口仍带真实 total。
        start, end = bscan_window_bounds(total, start_trace)
        return BScanWindow(
            start_trace=start,
            end_trace=end,
            max_start=bscan_window_max_start(total),
            total_traces=total,
            matrix=np.zeros((0, BSCAN_WINDOW_TRACES), dtype=np.float64),
        )
    start, end = bscan_window_bounds(total, start_trace)
    display = np.full((n_samples_value, BSCAN_WINDOW_TRACES), np.nan, dtype=np.float64)
    count = max(0, end - start + 1)
    if count:
        for offset, row_index in enumerate(range(start - 1, start - 1 + count)):
            display[:, offset] = rows[row_index]
    return BScanWindow(
        start_trace=start,
        end_trace=end,
        max_start=bscan_window_max_start(total),
        total_traces=total,
        matrix=display,
    )


def flat_filter_window_bounds(
    total_traces: int,
    start_trace: int,
    flat_window_size: int,
) -> tuple[int, int]:
    """返回可见 256 道窗口加上 Flat Filter 半窗邻域后的 1-based 切片边界。

    边界裁剪到真实历史范围。构造 ``[lo, hi]`` 邻域子矩阵执行
    :func:`gpr_processing.flat_reflection_filter` 后裁剪回可见窗口，与对
    完整历史逐元素滤波结果一致：滤波每个输出列只依赖
    ``[j-half, j+half]`` 邻域，子矩阵边界外的列要么是真实邻域（已被包含），
    要么位于全局历史边缘（edge padding 语义相同）。
    """
    total = _validated_trace_count(total_traces)
    if isinstance(flat_window_size, bool) or not isinstance(flat_window_size, (int, np.integer)):
        raise TypeError("flat_window_size must be an integer")
    window = int(flat_window_size)
    if window < 1:
        raise ValueError("flat_window_size must be at least 1")
    if window % 2 == 0:
        raise ValueError("flat_window_size must be odd")
    vis_start, vis_end = bscan_window_bounds(total, start_trace)
    half = window // 2
    lo = max(1, vis_start - half)
    hi = min(total, vis_end + half)
    return lo, hi


def apply_flat_filter_windowed(
    rows: list[np.ndarray] | tuple[np.ndarray, ...],
    start_trace: int,
    flat_window_size: int,
    filter_func=None,
) -> tuple[np.ndarray, bool]:
    """对可见窗口执行 Flat Filter，成本受窗口+邻域约束而非完整历史。

    返回 ``(filtered_window_matrix, filtered)``：``filtered_window_matrix``
    是 ``[n_samples, BSCAN_WINDOW_TRACES]``（不足 256 道右侧 NaN 填充）；
    ``filtered=False`` 表示历史道数不足以支撑该滤波窗口（与既有"等待N道，
    显示未滤波数据"语义一致），此时返回未滤波窗口。

    ``filter_func`` 缺省使用 :func:`gpr_processing.flat_reflection_filter`，
    可注入用于测试。结果与对完整历史调用滤波后切片**逐元素一致**
    （见 :func:`flat_filter_window_bounds` 的论证；测试
    ``tests/test_bscan_view.py`` 验证该等价性）。
    """
    if filter_func is None:
        from gpr_processing import flat_reflection_filter as filter_func
    total = _validated_trace_count(len(rows))
    vis_start, vis_end = bscan_window_bounds(total, start_trace)
    window = bscan_window_from_rows(
        rows,
        vis_start,
        n_samples=(len(rows[0]) if rows else 0),
    )
    if total < flat_window_size or total == 0:
        return window.matrix, False
    lo, hi = flat_filter_window_bounds(total, vis_start, flat_window_size)
    subset = np.column_stack([np.asarray(rows[i - 1], dtype=np.float64) for i in range(lo, hi + 1)])
    filtered = filter_func(subset, window_size=int(flat_window_size))
    offset = vis_start - lo
    count = vis_end - vis_start + 1
    window.matrix[:, :count] = filtered[:, offset : offset + count]
    return window.matrix, True
