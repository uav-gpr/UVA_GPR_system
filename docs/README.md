# 文档目录（docs/）

本目录存放 UAV-GPR 系统的正式文档：各轮开发计划、审查与验证报告、现场操作指引，以及回归测试脚本。项目总览见根目录 [`README.md`](../README.md)，开发交接规则见 [`PROGRESS.md`](../PROGRESS.md)。

> 当前版本：**v8.0.0-round8**（空中端黑匣子服务 + 地面站遥控架构）。

## 推荐阅读顺序

1. **[`第7轮_部署指引.md`](第7轮_部署指引.md)** — 计算棒黑匣子 + 地面站的现场部署、日常使用、验收清单与常见问题（现场必读）。
2. **[`round8_release_notes.md`](round8_release_notes.md)** — 第 8 轮发布说明：便携包产物与 SHA-256、三大现场故障（OSL/背景会话、校准文件导入导出、P1 重采）的根因与修复。
3. **[`第7轮_计划_空中端黑匣子与计算棒部署.md`](第7轮_计划_空中端黑匣子与计算棒部署.md)** — 黑匣子服务的设计：配置三层、自动重连、远程命令词表、磁盘水位。
4. **[`第8轮_校准与背景采集修复计划.md`](第8轮_校准与背景采集修复计划.md)** 与 [`round8_requirements_and_interfaces.md`](round8_requirements_and_interfaces.md) — 校准相关需求与接口契约。
5. **[`uav_gpr_remote_transmission_final_hm30_plan.md`](uav_gpr_remote_transmission_final_hm30_plan.md)** / [`air_to_ground_radar_data_transmission_plan.md`](air_to_ground_radar_data_transmission_plan.md) — HM30 空地传输方案与软件架构。
6. **[`钢筋仪项目_S11_S22_采集与校准参考.md`](钢筋仪项目_S11_S22_采集与校准参考.md)** — S11/S22 采集与校准算法参考。
7. `HM30 v1.7.pdf` / `HM30 v1.7.txt` — SIYI HM30 厂商手册。

## 按轮次索引

| 轮次 | 主要内容 | 关键文档 |
|---|---|---|
| Round 1 | 实时链路与绘图性能 | `实时链路与绘图性能_第1轮*.md`、`bench_round1_*.py` |
| Round 2 | 可靠性：常驻采集服务、磁盘发件箱 | `实时链路与可靠性_第2轮完成报告.md`、`round2_*.md` |
| Round 3–4 | UI3 重构、采集速度修复 | `实时链路与UI_第3轮完成报告.md`、`实时链路与UI_第4轮采集速度修复报告.md`、`第3轮_真机高速采集自测指引.md` |
| Round 5 | 滑动窗口重传、时域算法对齐钢筋仪 | `第5轮_*.md`、`round5_*.png`、`round5_ifft_ab_diagnostic.py` |
| Round 6 | B-scan 窗口 64 → 256、无人值守与全远程 | `第6轮_计划_无人值守与全远程.md`、`round6_bench_window256*` |
| Round 7 | 空中端黑匣子无头服务与计算棒部署 | `第7轮_计划_*.md`、`第7轮_部署指引.md`、`smoke_round7_black_box.py` |
| Round 8 | 校准与背景采集修复 | `第8轮_*.md`、`round8_*.md`（需求、审查、验证、发布说明） |

早期 HM30 选型与测试报告（`remote_ground_station_*.md`、`uav_gpr_remote_link_options_and_5p8g_bridge_plan.md`、`hm30_remote_operation.md` 等）保留作历史参考，其中的 IP（`192.168.144.111/.100`）和空中端 GUI 操作流程已被第 7 轮之后的黑匣子架构取代。

## 测试脚本

- **`run_tests_batched.py`** — 全量回归入口（每个测试模块独立子进程，规避 Qt WebEngine 单进程崩溃）：
  ```powershell
  & .venv312\Scripts\python.exe docs\run_tests_batched.py
  ```
- `smoke_round7_black_box.py` — 黑匣子双开冒烟测试。
- `bench_*.py`、`test_reproduce_*.py`、`round2_repro_runid_collision.py` — 性能基准与问题复现脚本。

## 运行数据

采集数据不放在本目录，也不纳入 Git：

```text
gpr_runs/      空中端（计算棒）本地采集数据、_outbox/、_calibration/
remote_runs/   地面站接收数据
```
