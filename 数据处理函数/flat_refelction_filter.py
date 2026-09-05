import numpy as np
from numpy.typing import ArrayLike, NDArray


def flat_reflection_filter(
    data: ArrayLike,
    window_size: int = 101,
) -> NDArray[np.float64]:
    """
    基本的 Flat Reflection Filter（局部背景减除）。

    Parameters
    ----------
    data:
        二维 GPR B-scan 数据，形状为：
        [n_samples, n_traces]

        n_samples：每道的时间采样点数
        n_traces：雷达道数

    window_size:
        沿测线方向计算局部平均的窗口道数。
        建议使用奇数，例如 101、201、301。

    Returns
    -------
    filtered_data:
        滤波后的二维数据，形状与输入相同。

    Notes
    -----
    算法：
        filtered(sample, trace)
        = original(sample, trace)
        - local_mean(sample, trace)
    """
    x = np.asarray(data, dtype=np.float64)

    if x.ndim != 2:
        raise ValueError(
            "data 必须是二维数组，形状应为 [n_samples, n_traces]"
        )

    if not isinstance(window_size, int):
        raise TypeError("window_size 必须是整数")

    if window_size < 1:
        raise ValueError("window_size 必须大于等于 1")

    if window_size % 2 == 0:
        raise ValueError("window_size 建议使用奇数")

    n_samples, n_traces = x.shape

    if window_size > n_traces:
        raise ValueError(
            f"window_size={window_size} 超过总道数 {n_traces}"
        )

    half_window = window_size // 2

    # 沿道方向进行边界扩展。
    # edge 模式会复制首尾道，避免边界位置出现零填充伪影。
    padded = np.pad(
        x,
        pad_width=((0, 0), (half_window, half_window)),
        mode="edge",
    )

    # 累积和前面补一列 0，便于计算长度恰好为 window_size 的滑动和。
    cumulative_sum = np.cumsum(padded, axis=1, dtype=np.float64)
    cumulative_sum = np.concatenate(
        [
            np.zeros((n_samples, 1), dtype=np.float64),
            cumulative_sum,
        ],
        axis=1,
    )

    # 利用累积和计算局部窗口均值。
    local_sum = (
        cumulative_sum[:, window_size:]
        - cumulative_sum[:, :-window_size]
    )

    local_mean = local_sum / window_size

    return x - local_mean