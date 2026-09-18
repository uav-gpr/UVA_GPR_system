"""Pure, testable signal-processing primitives for GPR traces.

The functions in this module deliberately do not depend on either Qt GUI or
LibreVNA code.  Frequency-domain traces use the first axis as frequency when
an array has more than one dimension; :func:`frequency_to_time_trace` is
intentionally limited to one A-scan.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray


def _validated_frequency_axis(frequency_hz: ArrayLike) -> NDArray[np.float64]:
    frequency = np.asarray(frequency_hz, dtype=np.float64)
    if frequency.ndim != 1 or frequency.size < 2:
        raise ValueError("frequency_hz must be a one-dimensional array with at least 2 samples")
    if not np.all(np.isfinite(frequency)):
        raise ValueError("frequency_hz must contain only finite values")
    if not np.all(np.diff(frequency) > 0.0):
        raise ValueError("frequency_hz must be strictly increasing")
    return frequency


def _validated_positive_float(value: float, name: str) -> float:
    try:
        converted = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite positive number") from exc
    if not math.isfinite(converted) or converted <= 0.0:
        raise ValueError(f"{name} must be a finite positive number")
    return converted


def frequency_bpf_weights(frequency_hz: ArrayLike) -> NDArray[np.float64]:
    """Return the sin-squared band-pass weights for a frequency axis.

    The four breakpoints are derived from the supplied range:

    ``f0=fmin, f1=fmin+range/4, f2=fmax-range/4, f3=fmax``.

    The response is zero at ``f0`` and ``f3``, rises to one between ``f0``
    and ``f1``, stays at one through ``f2``, and falls to zero by ``f3``.
    """
    frequency = _validated_frequency_axis(frequency_hz)
    f0 = float(frequency[0])
    f3 = float(frequency[-1])
    span = f3 - f0
    f1 = f0 + span / 4.0
    f2 = f3 - span / 4.0

    weights = np.zeros(frequency.shape, dtype=np.float64)

    rising = (frequency >= f0) & (frequency <= f1)
    rising_phase = (frequency[rising] - f0) / (f1 - f0)
    weights[rising] = np.sin(0.5 * np.pi * rising_phase) ** 2

    plateau = (frequency > f1) & (frequency < f2)
    weights[plateau] = 1.0

    falling = (frequency >= f2) & (frequency <= f3)
    falling_phase = (f3 - frequency[falling]) / (f3 - f2)
    weights[falling] = np.sin(0.5 * np.pi * falling_phase) ** 2

    return weights


def apply_frequency_bpf(
    frequency_hz: ArrayLike,
    complex_spectrum: ArrayLike,
) -> NDArray[np.complex128]:
    """Apply :func:`frequency_bpf_weights` directly to a complex spectrum.

    For a multidimensional spectrum, frequency is the first axis and the
    weights are broadcast over the remaining axes.  The output keeps the
    input shape and is always a complex array.
    """
    frequency = _validated_frequency_axis(frequency_hz)
    spectrum = np.asarray(complex_spectrum)
    if spectrum.ndim == 0 or spectrum.shape[0] != frequency.size:
        raise ValueError(
            "complex_spectrum must have frequency_hz.length samples on its first axis"
        )
    spectrum = spectrum.astype(np.complex128, copy=False)
    if not np.all(np.isfinite(spectrum.real)) or not np.all(np.isfinite(spectrum.imag)):
        raise ValueError("complex_spectrum must contain only finite values")
    shape = (frequency.size,) + (1,) * (spectrum.ndim - 1)
    return spectrum * frequency_bpf_weights(frequency).reshape(shape)


def _next_power_of_two(value: int) -> int:
    """Return the smallest power of two greater than or equal to value."""
    if value < 1:
        raise ValueError("FFT input length must be positive.")
    return 1 << (value - 1).bit_length()


def _compute_ifft_grid(
    frequencies_hz: ArrayLike,
    oversampling: int = 8,
) -> tuple[float, int, int, float, NDArray[np.float64]]:
    """Compute IFFT grid from frequency axis.

    Returns (df, fft_size, first_bin, start_frequency_offset_hz, time_s).
    Aligned with rebar-inspector processing/ifft.py.
    """
    frequency = _validated_frequency_axis(frequencies_hz)
    if frequency[0] < 0.0:
        raise ValueError("negative frequencies not supported")
    steps = np.diff(frequency)
    df = float(np.median(steps))
    allowed_error = max(1.0, abs(df) * 1e-6)
    max_error = float(np.max(np.abs(steps - df)))
    if max_error > allowed_error:
        raise ValueError(
            f"IFFT requires uniform frequency spacing: median step {df:g} Hz, max deviation {max_error:g} Hz"
        )
    first_bin = max(0, int(round(frequency[0] / df)))
    grid_start_hz = first_bin * df
    start_frequency_offset_hz = float(frequency[0] - grid_start_hz)
    padded_length = first_bin + frequency.size
    fft_size = _next_power_of_two(padded_length) * oversampling
    time_s = np.arange(fft_size, dtype=np.float64) / (fft_size * df)
    return df, fft_size, first_bin, start_frequency_offset_hz, time_s


def dewow(
    data: ArrayLike,
    dt_s: float,
    window_ns: float,
    time_axis: int = 0,
) -> NDArray:
    """Remove the local moving-average wow along the time axis.

    Supports both real float64 and complex128 data. For complex inputs,
    moving average is computed over real and imaginary parts independently,
    preserving complex phase. One- or two-dimensional arrays supported;
    the returned shape and dtype match the input.
    """
    x = np.asarray(data)
    if x.ndim not in (1, 2):
        raise ValueError("data must be a one- or two-dimensional A-scan/B-scan array")
    is_complex = np.iscomplexobj(x)
    if is_complex:
        if not (np.all(np.isfinite(x.real)) and np.all(np.isfinite(x.imag))):
            raise ValueError("data must contain only finite values")
        dtype = np.complex128
    else:
        x = x.astype(np.float64, copy=False)
        if not np.all(np.isfinite(x)):
            raise ValueError("data must contain only finite values")
        dtype = np.float64

    dt = _validated_positive_float(dt_s, "dt_s")
    window = _validated_positive_float(window_ns, "window_ns")
    if isinstance(time_axis, bool) or not isinstance(time_axis, (int, np.integer)):
        raise TypeError("time_axis must be an integer axis")
    if time_axis not in range(-x.ndim, x.ndim):
        raise ValueError(f"time_axis={time_axis} is outside the data dimensions")

    time_axis = int(time_axis) % x.ndim
    n_samples = x.shape[time_axis]
    window_samples = max(1, int(round(window * 1e-9 / dt)))
    if window_samples % 2 == 0:
        window_samples += 1
    if window_samples > n_samples:
        raise ValueError(
            f"dewow window requires {window_samples} samples, but time axis has {n_samples}"
        )
    if window_samples == 1:
        raise ValueError("dewow window must cover at least 3 samples")

    moved = np.moveaxis(x, time_axis, 0)
    half_window = window_samples // 2
    padded = np.pad(
        moved,
        pad_width=[(half_window, half_window)] + [(0, 0)] * (moved.ndim - 1),
        mode="reflect",
    )
    cumulative_sum = np.cumsum(padded, axis=0, dtype=dtype)
    zero_shape = list(cumulative_sum.shape)
    zero_shape[0] = 1
    cumulative_sum = np.concatenate(
        [np.zeros(zero_shape, dtype=dtype), cumulative_sum],
        axis=0,
    )
    local_sum = cumulative_sum[window_samples:] - cumulative_sum[:-window_samples]
    local_mean = local_sum / window_samples
    filtered = moved - local_mean
    return np.moveaxis(filtered, 0, time_axis)


def flat_reflection_filter(
    data: ArrayLike,
    window_size: int = 101,
) -> NDArray:
    """Remove the local trace-direction background from a B-scan.

    Input is always ``[n_samples, n_traces]``. The moving average is taken
    across the second axis only, so this function is not intended for a single
    A-scan. Supports both real float64 and complex128 data, filtering real and
    imaginary components independently.
    """
    x = np.asarray(data)
    if x.ndim != 2:
        raise ValueError("data must be a two-dimensional [n_samples, n_traces] array")
    is_complex = np.iscomplexobj(x)
    if is_complex:
        if not (np.all(np.isfinite(x.real)) and np.all(np.isfinite(x.imag))):
            raise ValueError("data must contain only finite values")
        dtype = np.complex128
    else:
        x = x.astype(np.float64, copy=False)
        if not np.all(np.isfinite(x)):
            raise ValueError("data must contain only finite values")
        dtype = np.float64

    if isinstance(window_size, bool) or not isinstance(window_size, (int, np.integer)):
        raise TypeError("window_size must be an integer")
    window_size = int(window_size)
    if window_size < 1:
        raise ValueError("window_size must be at least 1")
    if window_size % 2 == 0:
        raise ValueError("window_size must be odd")

    n_samples, n_traces = x.shape
    if window_size > n_traces:
        raise ValueError(
            f"window_size={window_size} exceeds the number of traces {n_traces}"
        )

    half_window = window_size // 2
    padded = np.pad(
        x,
        pad_width=((0, 0), (half_window, half_window)),
        mode="edge",
    )
    cumulative_sum = np.cumsum(padded, axis=1, dtype=dtype)
    cumulative_sum = np.concatenate(
        [np.zeros((n_samples, 1), dtype=dtype), cumulative_sum],
        axis=1,
    )
    local_sum = cumulative_sum[:, window_size:] - cumulative_sum[:, :-window_size]
    local_mean = local_sum / window_size
    return x - local_mean


def frequency_to_time_trace(
    frequency_hz: ArrayLike,
    complex_spectrum: ArrayLike,
    ifft_points: int = 2048,
    time_window_ns: float = 40.0,
    dewow_window_ns: float = 4.0,
    apply_dewow: bool = True,
    oversampling: int = 8,
    aligned: bool | None = None,
    return_complex: bool | None = None,
) -> tuple[NDArray, NDArray]:
    """Convert one BPF-processed complex frequency trace into an A-scan.

    When aligned is True (or ifft_points >= 32 with aligned=None), aligns with
    the rebar-inspector algorithm:
    1. DC zero-padding to the actual first frequency bin (first_bin = round(f0 / df)).
    2. Residual frequency offset correction via phase ramp exp(2j * pi * offset * t).
    3. Oversampling (production default 8; the rebar reference uses 16 — physics
       identical, 8 restores the pre-change per-trace sample count / throughput headroom):
       fft_size = _next_power_of_two(padded_len) * oversampling.
    4. Time window truncation to sample_count = round(max_time / dt + 0.5).
    5. Full-chain complex processing: Dewow is applied on the complex trace.

    When aligned is False (or ifft_points < 32 with aligned=None), falls back
    to the legacy zero-bin IFFT for backwards compatibility with low-point tests.
    """
    if not isinstance(apply_dewow, bool):
        raise TypeError("apply_dewow must be a bool")
    frequency = _validated_frequency_axis(frequency_hz)
    spectrum = np.asarray(complex_spectrum).astype(np.complex128, copy=False)
    if spectrum.ndim != 1 or spectrum.size != frequency.size:
        raise ValueError("complex_spectrum must be a one-dimensional array matching frequency_hz")
    if not np.all(np.isfinite(spectrum.real)) or not np.all(np.isfinite(spectrum.imag)):
        raise ValueError("complex_spectrum must contain only finite values")
    if isinstance(ifft_points, bool) or not isinstance(ifft_points, (int, np.integer)):
        raise TypeError("ifft_points must be an integer")
    if int(ifft_points) < 2:
        raise ValueError("ifft_points must be at least 2")
    time_window = _validated_positive_float(time_window_ns, "time_window_ns")

    spacing = np.diff(frequency)
    if not np.allclose(spacing, spacing[0], rtol=1e-6, atol=max(abs(spacing[0]) * 1e-6, 1.0)):
        raise ValueError("frequency_hz must be uniformly spaced for IFFT")

    is_aligned = (int(ifft_points) >= 32) if (aligned is None) else bool(aligned)
    ret_complex = is_aligned if (return_complex is None) else bool(return_complex)

    if is_aligned:
        df, fft_size, first_bin, offset, time_s_full = _compute_ifft_grid(
            frequency, oversampling=oversampling
        )
        padded = np.zeros(fft_size, dtype=np.complex128)
        padded[first_bin : first_bin + spectrum.size] = spectrum
        time_response = np.fft.ifft(padded)
        if offset != 0.0:
            time_response = time_response * np.exp(2j * np.pi * offset * time_s_full)
        dt_s = float(time_s_full[1])
        sample_count = int(math.floor(time_window * 1e-9 / dt_s + 0.5))
        sample_count = min(fft_size, max(1, sample_count))
        time_axis_ns = time_s_full[:sample_count] * 1e9
        trace = time_response[:sample_count]
        if apply_dewow:
            trace = dewow(trace, dt_s=dt_s, window_ns=dewow_window_ns)
        if not ret_complex:
            trace = trace.real.astype(np.float64, copy=False)
        return time_axis_ns, trace
    else:
        # Legacy mode (backward compatibility for synthetic unit tests with small N)
        n_fft = max(int(ifft_points), spectrum.size)
        padded = np.zeros(n_fft, dtype=np.complex128)
        padded[: spectrum.size] = spectrum
        dt_s = 1.0 / (n_fft * float(spacing[0]))
        full_time_ns = np.arange(n_fft, dtype=np.float64) * dt_s * 1e9
        full_trace = np.fft.ifft(padded)
        if not ret_complex:
            full_trace = full_trace.real.astype(np.float64, copy=False)
        if apply_dewow:
            full_trace = dewow(full_trace, dt_s=dt_s, window_ns=dewow_window_ns)
        mask = full_time_ns <= time_window
        return full_time_ns[mask], full_trace[mask]
