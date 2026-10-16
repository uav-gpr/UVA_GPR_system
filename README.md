# UAV-GPR 无人机探地雷达系统

**当前版本：v8.0.0-round8（2026-10-14）**

UAV-GPR 是一套搭载在无人机上的探地雷达（GPR）采集与遥控系统，由两部分组成：

- **空中端黑匣子**：跑在计算棒上的无界面后台服务，通过 USB 驱动 LibreVNA 扫频、读取 GNSS，数据本地落盘并经 HM30 图传链路回传，断线自动缓存与补传。
- **地面站**：PySide6 / Qt 图形界面，负责全部远程控制、B-scan 实时成像、GNSS 地图、OSL/SOLT 校准向导与校准文件导入导出。

> 一句话：**计算棒 = 黑匣子（插电就干活，永远不用碰）；地面端电脑 = 遥控器（所有按钮都在这）。**

> ⚠️ 修改代码前请先通读 [`PROGRESS.md`](PROGRESS.md)（项目交接本），完成后更新它并随代码一同提交。

---

## 系统架构

```text
空中端（无人机）                                   地面端
┌──────────────────────────────┐                ┌──────────────────────────────┐
│ LibreVNA ──USB──┐            │                │  uav_gpr_ground_station.py   │
│ GNSS ──串口─────┤ 计算棒      │   HM30 链路    │  · B-scan 实时成像           │
│                 └ uav_gpr_   │ ◀────────────▶ │  · 「空中端设备」遥控面板    │
│   air_service.py（后台服务） │   TCP 8765     │  · 校准向导 / 校准导入导出   │
│   · disk outbox 断线补传     │                │  · GNSS 地图                 │
│   · 磁盘水位自动清理         │                │  · 数据落盘 remote_runs/     │
└──────────────────────────────┘                └──────────────────────────────┘
```

- 通信协议：`uav-gpr-telemetry-v1`，带内 TCP，滑动窗口重传（窗口 16）+ 心跳。
- 空中端开机即主动连接地面端；连不上按 5 秒起步、30 秒封顶的指数退避一直重试。
- 断线期间空中端照常采集并本地保存（`gpr_runs/_outbox/`），链路恢复后自动补传。
- **通信不变量**：严禁新增消息 type，新增远程功能一律走 `command` + `action`。

## 网络规划（HM30）

| 设备 | IP |
|---|---|
| HM30 空中单元 | 192.168.144.11 |
| HM30 地面单元 | 192.168.144.12 |
| 空中端计算棒 | **192.168.144.21** |
| 地面站电脑 | **192.168.144.20** |
| 子网掩码 | 255.255.255.0 |
| 遥测端口 | TCP 8765（地面端监听，需在防火墙放行） |

> 旧文档中的 `192.168.144.111 / .100` 是 2026-07 早期测试地址，已废弃。

## 目录结构

```text
uav_gpr_air_service.py      空中端黑匣子服务（无界面后台，v8.0.0-round8）
uav_gpr_ground_station.py   地面站 GUI（遥控面板、B-scan、校准向导）
uav_gpr_qt.py               采集/发送/补传核心零件（空中服务直接复用）
calibration_service.py      双端共享校准核心（OSL/SOLT、重采、校验、导入导出）
air_gnss.py / ground_gnss.py  空中 / 地面 GNSS 模块
disk_outbox.py              磁盘发件箱（断点续传）
acquisition_scheduler.py    采集调度
ground_trace_pipeline.py    地面端数据流水线
gpr_processing.py           雷达信号处理（IFFT 等）
bscan_view.py               B-scan 成像视图
gnss_map_widget.py          GNSS 地图控件
pipeline_metrics.py         流水线性能度量
uav_gpr_receiver.py         无界面 TCP 接收器（调试用）
air_config.json             空中端配置
librevna/                   LibreVNA USB 直连驱动（含模拟器）
calibration_reference/      校准参考实现
dual_channel_reference/     双通道参考
数据处理函数/               dewow、带通、平面反射滤波等后处理函数
tools/                      便携包构建脚本（build_round8_portable.py）等
tests/                      单元 / 集成测试（42 个模块）
docs/                       各轮计划、审查、验证报告与操作文档
run_air_station.bat         旧版空中端 GUI 启动器（uav_gpr_qt.py）
run_ground_station.bat      地面站启动器
```

## 环境与安装（开发）

- Windows x64，**Python 3.12**，虚拟环境固定为 `.venv312`。

```powershell
py -3.12 -m venv .venv312
& .venv312\Scripts\python.exe -m pip install -r requirements.txt
```

主要依赖：numpy、PySide6、pyqtgraph、pyserial、pyusb、libusb-package、matplotlib。LibreVNA 需安装 WinUSB 驱动（设备管理器中为 `VID_1209&PID_4121`）。

## 现场部署（便携包，推荐）

现场使用两个配套的绿色免安装便携包（自带 Python 运行时和全部依赖），由 `tools/build_round8_portable.py` 构建，**两端版本必须一致**：

| 包 | 拷到哪 | 入口 |
|---|---|---|
| `空中端黑匣子便携包` | 计算棒 | `安装到计算棒.bat`（注册开机自启计划任务） |
| `地面站便携包` | 地面站电脑 | `run_ground_station.bat` |

**计算棒一次性安装：**

1. 把 `空中端黑匣子便携包` 整个文件夹拷到计算棒（如 `D:\air_blackbox\`）。
2. 右键 `安装到计算棒.bat` → 以管理员身份运行。会注册计划任务 `UAVGPR_AirBlackBox`（开机延迟 30 秒 + 登录双触发，崩溃 60 秒自动拉起）并立即启动服务。
3. 确认计算棒有线网卡静态 IP 为 `192.168.144.21`。
4. USB 插上 LibreVNA，脚本显示 `[OK] LibreVNA USB device present`。
5. 拔掉屏幕键盘，只留电源 + 网线（接 HM30 空中单元网口）。

停服务 / 升级：包内 `关闭自启.bat`、`升级_替换程序.bat`（非 purge 安全升级，配置、日志、数据、校准文件全部保留）。详见 [`docs/第7轮_部署指引.md`](docs/第7轮_部署指引.md)。

## 日常使用（全部在地面端操作）

1. 地面站电脑双击 `run_ground_station.bat`（等价于 `python uav_gpr_ground_station.py --host 0.0.0.0 --port 8765 --out remote_runs`）。
2. 打开 **「空中端设备」面板**：查看连接状态、雷达 USB、run_id、积压、磁盘剩余、程序版本。
3. 点 **「测试雷达」**，显示固件 / 协议号即雷达在线。
4. 如需校准：用校准向导完成 OSL，再采集背景（Through/Air）；也可直接 **导入已有校准文件**（如 `solt_2port_*.json`）下发到空中端，或把空中端当前校准 **导出** 到地面留存。
5. 主界面点 **「开始」** 采集：B-scan 实时出图，数据同时落地面端和计算棒本地。
6. GNSS 串口、采集引擎、USB/模拟器切换等都在面板上设置，空中端自动回写配置并回执确认。

### 远程命令（`command` 消息的 action）

| 类别 | action |
|---|---|
| 采集 | `start` `stop` `single` `pause` `resume` `set_params` |
| 设备 | `get_status` `test_vna` `list_serial_ports` `set_gnss_port` `test_gnss` `set_engine` `set_device_source` |
| 校准 | `cal_begin` `cal_capture` `cal_finish` `cal_activate` `cal_cancel` `cal_status` |
| 校准文件 | `cal_import_file` `cal_export_file` `cal_query_active` `cal_clear_active` |

未知 action 一律回执 `supported=false`，不会导致服务崩溃。

## 空中端配置（`air_config.json`）

```json
{
  "ground_host": "192.168.144.20",
  "ground_port": 8765,
  "gnss_port": "",
  "gnss_baud": 9600,
  "engine": "continuous",
  "device_source": "usb",
  "output_dir": "gpr_runs",
  "autostart_acquisition": false,
  "disk_min_free_gb": 20.0,
  "disk_target_free_gb": 40.0,
  "log_max_bytes": 20971520,
  "log_backup_count": 10,
  "calibration_path": "",
  "experimental_s22_fast_sweep": false
}
```

- `gnss_port` 为空表示自动识别，`off` 表示禁用。
- 采集通道在地面站“设置中心 → 采集控制 → 采集通道”里选。只采 HH 时雷达只让端口 1 发射（扫法 `0x1240`，真机验证过），速度约快一倍；只采 VV 默认仍两路一起扫再丢掉 HH（速度不变）。把 `experimental_s22_fast_sweep` 设为 `true` 后，只采 VV 改用只让端口 2 发射的扫法 `0x1208`——**这个扫法还没在真机上验证**，第一次用请对照双通道结果检查 VV 数据是否一致。
- 磁盘剩余低于 `disk_min_free_gb` 时从最老的 run 开始清理到 `disk_target_free_gb`，优先删已送达数据；动到未送达数据会显式告警（DISK-ALARM）。
- 命令行可覆盖配置：`--config` `--server HOST:PORT` `--gnss-port` `--gnss-baud` `--engine` `--autostart {true,false}`。

## 数据输出

```text
空中端（计算棒）
  gpr_runs/<run_id>/                 本地采集数据
  gpr_runs/_outbox/<run_id>/         未确认送达的帧（断线补传）
  gpr_runs/_calibration/             校准会话与导入的校准文件（imports/）
  logs/air_service.log               滚动日志，20MB × 10 份

地面端
  remote_runs/<run_id>/              接收到的数据
```

采集数据不纳入 Git。

## 测试

全量回归**必须**用分批子进程脚本（单进程 `unittest discover` 会在 Qt WebEngine 下崩溃）：

```powershell
& .venv312\Scripts\python.exe docs\run_tests_batched.py
```

- 当前基线：**42 个模块 1191 项测试全绿 + 30 项黑匣子端到端（E2E）测试**。
- 判绿以子进程输出的 `OK` 为准；部分 Qt 进程 teardown 时以 `0xC0000005` 退出属正常，不算回归。
- 任何涉及空地通信的改动都必须跑全量回归。

## 常见问题

- **面板一直「重连中」**：先确认地面站已开；计算棒 IP 是否 `192.168.144.21`；网线是否接 HM30 空中单元；地面端防火墙是否放行 8765。仍不行查看计算棒 `logs\air_service.log` 中 `Remote send retrying` 后的原因。
- **「测试雷达」失败**：检查 USB 与 WinUSB 驱动。面板切到「模拟器」时显示模拟器模式属正常。
- **升级后面板版本没变**：确认升级脚本已重启计划任务，或重跑 `安装到计算棒.bat`。

## 版本历史

| 轮次 | 主要内容 |
|---|---|
| Round 1–5 | 实时流水线性能优化、常驻采集服务、磁盘发件箱、UI3 界面重构、滑动窗口重传 |
| Round 6 | B-scan 显示窗口 64 → 256，空中端远程连接体验优化 |
| Round 7 | 空中端黑匣子无头服务与计算棒远程部署 |
| Round 8 | 校准与背景采集修复：OSL / 背景会话解耦、校准文件导入导出、P1 重采状态机修复 |

详细说明见 [`docs/round8_release_notes.md`](docs/round8_release_notes.md) 及 `docs/` 下各轮报告。
