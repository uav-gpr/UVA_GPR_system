# UAV-GPR 校准代码独立参考

这个文件夹把原软件中的 VNA 校准逻辑整理成可以迁移到其他 Python 软件的独立模块。

## 文件说明

- `osl_calibration.py`：一端口三项 OSL 数学模型、频率插值、校正和 JSON 保存/读取。
- `gpr_processing_pipeline.py`：S11 校正、复数背景扣除、可选频域减均值和 IFFT 的完整处理流程。
- `two_port_reference.py`：当前项目采用的简化两端口处理，即 P1/P2 分别 OSL，S21/S12 使用 Through 复数归一化。
- `example_usage.py`：生成模拟 VNA 系统误差，执行 OSL 校准并验证结果。
- `__init__.py`：对外导出常用接口。

## 与原项目代码的对应关系

- `uav_gpr_qt.py:1420`：采集 Open、Short、Load、Through。
- `uav_gpr_qt.py:357`：根据标准件测量生成 OSL 校准。
- `uav_gpr_qt.py:531`：应用 S11/S22 OSL 和 S21/S12 Through 修正。
- `librevna/librevna_usb_s11.py:55-167`：三项 OSL 误差模型和数学公式。

## 最小使用流程

1. 固定扫频范围、点数、功率、IFBW、线缆和参考面。
2. 实际连接 Open，采集一组复数 S11。
3. 实际连接 Short，采集一组复数 S11。
4. 实际连接 50 欧姆 Load，采集一组复数 S11。
5. 调用 `build_osl_calibration()`。
6. 测量被测件并调用 `calibration.correct()`。
7. 同时保存原始值与校准后的值。

```python
from calibration_reference import build_osl_calibration

calibration = build_osl_calibration(
    frequency_hz,
    open_measured,
    short_measured,
    load_measured,
)

corrected_s11 = calibration.correct(frequency_hz, raw_s11)
calibration.save_json("my_calibration.json")
```

## 校正、背景扣除和减均值流程

```python
from calibration_reference import (
    BackgroundReference,
    process_s11_trace,
)

# 推荐采集多条背景并进行复数平均；这里的数组形状为
# (背景采集次数, 频率点数)。
background = BackgroundReference.from_raw_captures(
    frequency_hz,
    raw_background_s11_captures,
    calibration=calibration,
)

result = process_s11_trace(
    frequency_hz,
    raw_dut_s11,
    calibration=calibration,
    background=background,
    remove_frequency_mean=False,  # 确认不会损伤浅层目标后再打开
    ifft_points=2048,
    time_window_ns=40.0,
)

raw_s11 = result.raw_s11
calibrated_s11 = result.calibrated_s11
background_removed_s11 = result.background_subtracted_s11
time_trace = result.time_trace
```

处理顺序固定为：原始复数 S11 → OSL 校正 → 复数背景扣除 → 可选频域减均值 → 补零/IFFT。每一个中间结果都会保留，便于保存和对照。

## 运行自测

```powershell
python -m calibration_reference.example_usage
```

## 边界说明

- 默认使用理想校准件模型：Open = +1、Short = -1、Load = 0。
- `build_osl_calibration()` 允许传入随频率变化的真实校准件反射系数，便于以后加入 Cal Kit 参数。
- `two_port_reference.py` 不是完整的两端口 12 项 SOLT 算法。
- Through 处理只是复数除法归一化，不能替代专业两端口误差模型。
- 校准频段之外拒绝外推，避免错误地继续使用校准数据。
