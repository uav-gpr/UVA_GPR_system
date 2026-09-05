import numpy as np
from numpy.typing import ArrayLike, NDArray


def dewow(
    data: ArrayLike,
    dt_s: float,
    window_ns: float,
    time_axis: int = 0,
) -> NDArray[np.float64]:
    """
    GPR Dewow：沿时间轴减去局部滑动平均。

    Parameters
    ----------
    data:
        一维 A-scan 或二维 B-scan 数据。

        默认二维数据格式为：
            [n_samples, n_traces]

    dt_s:
        时间采样间隔，单位为秒。

        例如：
            20 ps -> 20e-12
            50 ps -> 50e-12

    window_ns:
        Dewow 滑动窗口时间长度，单位为 ns。

        例如：
            window_ns=4.0 表示使用约 4 ns 的滑动窗口。

    time_axis:
        时间采样点所在的轴。

        默认：
            time_axis=0

        如果数据格式为 [n_traces, n_samples]，
        则设置：
            time_axis=1

    Returns
    -------
    filtered:
        Dewow 后的数据，形状与输入一致，类型为 float64。

    Notes
    -----
    算法为：

        output = original - moving_average(original)

    窗口采样点数自动计算为：

        window_samples = round(window_ns * 1e-9 / dt_s)

    并自动调整为奇数。
    """
    x = np.asarray(data, dtype=np.float64)

    if x.ndim not in (1, 2):
        raise ValueError("data 必须是一维 A-scan 或二维 B-scan 数据")

    if not np.isfinite(dt_s) or dt_s <= 0:
        raise ValueError("dt_s 必须是大于 0 的有限数值")

    if not np.isfinite(window_ns) or window_ns <= 0:
        raise ValueError("window_ns 必须是大于 0 的有限数值")

    if time_axis not in range(-x.ndim, x.ndim):
        raise ValueError(
            f"time_axis={time_axis} 超出数据维度范围"
        )

    time_axis = time_axis % x.ndim
    n_samples = x.shape[time_axis]

    # 时间窗口由 ns 转换为采样点数
    window_samples = int(round(window_ns * 1e-9 / dt_s))

    # 至少使用 1 个采样点
    window_samples = max(1, window_samples)

    # 调整为奇数，使当前采样点位于窗口中心
    if window_samples % 2 == 0:
        window_samples += 1

    if window_samples > n_samples:
        raise ValueError(
            f"Dewow 窗口为 {window_samples} 个采样点，"
            f"但时间轴只有 {n_samples} 个采样点。"
            "请减小 window_ns。"
        )

    # window_samples=1 时，局部均值就是信号自身，
    # 会导致结果全为 0，因此禁止这种无实际意义的设置。
    if window_samples == 1:
        raise ValueError(
            "时间窗口换算后只有 1 个采样点，请增大 window_ns"
        )

    # 把时间轴移到第 0 维，便于统一处理
    moved = np.moveaxis(x, time_axis, 0)

    half_window = window_samples // 2

    # 使用反射填充，减小时间轴首尾的边界失真
    padded = np.pad(
        moved,
        pad_width=[
            (half_window, half_window),
            *[(0, 0)] * (moved.ndim - 1),
        ],
        mode="reflect",
    )

    # 沿时间轴计算累积和
    cumulative_sum = np.cumsum(
        padded,
        axis=0,
        dtype=np.float64,
    )

    # 在累积和前补一个零切片
    zero_shape = list(cumulative_sum.shape)
    zero_shape[0] = 1

    cumulative_sum = np.concatenate(
        [
            np.zeros(zero_shape, dtype=np.float64),
            cumulative_sum,
        ],
        axis=0,
    )

    # 利用累积和计算滑动窗口均值
    local_sum = (
        cumulative_sum[window_samples:]
        - cumulative_sum[:-window_samples]
    )

    local_mean = local_sum / window_samples

    filtered = moved - local_mean

    # 恢复原始轴顺序
    return np.moveaxis(filtered, 0, time_axis)