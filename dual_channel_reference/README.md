# 双通道采集与校正参考代码

该目录与 Qt GUI 解耦，分成两个模块：

- `dual_channel_acquisition.py`：封装 LibreVNA 两端口采集，返回 `S11/S21/S12/S22`，支持原始 NPZ 保存/读取和测试后端注入。
- `dual_channel_calibration.py`：P1/P2 分别进行一端口 OSL，`S21/S12` 使用 Through 复数归一化。
- `example_physical_calibration.py`：需要实际连接七个校准状态的交互式示例。

## 单次采集

```powershell
python -m dual_channel_reference.dual_channel_acquisition --out dual_channel_raw.npz
```

## 七步校准和 DUT 测量

```powershell
python -m dual_channel_reference.example_physical_calibration --out calibration_run
```

七个物理步骤依次为：P1 Open、P1 Short、P1 Load、P2 Open、P2 Short、P2 Load、Through。所有步骤必须使用完全一致的频率范围、频点数、IFBW、功率、电缆和参考面。

该算法不是完整的十二项双端口 SOLT。它适合作为当前 UAV-GPR 原型的软件参考，不应替代可追溯的计量级 VNA 校准。
