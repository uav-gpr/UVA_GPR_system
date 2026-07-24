# UAV-GPR 天空端到地面端雷达数据远程传输方案

版本日期：2026-07-10

## 1. 目标与边界

本方案面向“两台电脑之间只传输雷达数据”的现场应用：天空端电脑随无人机或雷达平台工作，连接 LibreVNA 与 GNSS，负责采集和本地保存；地面端电脑负责接收天空端回传的数据，进行实时显示、远程备份和后续处理。

本方案不把远程控制作为核心目标。地面端可以保留最小的启动、停止、单次采集命令能力，但系统设计的优先级应放在雷达数据可靠回传上。

## 2. 当前项目基础

当前工程已经具备以下基础能力：

- `uav_gpr_qt.py`：天空端采集程序，连接 LibreVNA 和 GNSS，完成 S 参数采集、A-scan/B-scan 计算、本地 CSV/NPZ 保存，并已包含 `RemoteSender` 发送模块。
- `uav_gpr_ground_station.py`：地面端 GUI，监听 TCP 端口，接收天空端回传的 trace 数据，保存到 `remote_runs/`，并实时显示 S 参数、A-scan、四通道 B-scan 和 GNSS 信息。
- `uav_gpr_receiver.py`：轻量命令行接收端，可用于无 GUI 链路测试。
- 当前协议名为 `uav-gpr-telemetry-v1`。
- 当前通信方式为 Python 标准库 TCP socket，不需要额外安装 WebSocket、MQTT 或 gRPC 依赖。

当前每条 trace 的主要数据以压缩 NPZ 形式传输，包含：

- `frequency_hz`
- `time_ns`
- `gnss`
- `raw_S11/raw_S21/raw_S12/raw_S22` 的实部和虚部
- `display_S11/display_S21/display_S12/display_S22` 的实部和虚部
- `trace_S11/trace_S21/trace_S12/trace_S22`
- `config`
- `index`

这与“只传输雷达数据”的目标基本一致，建议继续沿用 NPZ 作为实时传输主格式。

## 3. 推荐总体架构

推荐采用“天空端本地保存 + TCP 实时回传 + 地面端校验存档”的架构。

```text
LibreVNA / GNSS
    |
    v
天空端电脑 B
  - 运行 uav_gpr_qt.py
  - 控制 LibreVNA 采集雷达数据
  - 解析 GNSS
  - 本地保存 CSV/NPZ/metadata
  - 将每条 trace 压缩为 NPZ 并通过 TCP 发送
    |
    | Wi-Fi 热点 / 数传网桥 / 5G CPE / 局域网
    v
地面端电脑 A
  - 运行 uav_gpr_ground_station.py 或 uav_gpr_receiver.py
  - 监听 TCP 端口
  - 校验 sha256
  - 保存远程副本到 remote_runs/
  - 可选实时显示 S 参数、A-scan、B-scan、GNSS
```

核心原则：

- 天空端先本地落盘，再尝试发送，避免链路抖动造成原始数据丢失。
- 地面端以 `run_id + trace_index` 管理数据，便于去重、补传和后处理。
- 实时链路传 NPZ，不传 CSV。CSV 适合人工查看，但体积更大、解析更慢。
- 通信层只承载数据帧、确认帧和少量状态帧，避免把控制逻辑做得过重。

## 4. 网络连接方案

### 4.1 现场优先方案：地面端开热点

当前项目已经按这个方式做过适配：

```text
地面端电脑 A
  - 开 Windows 移动热点
  - 默认热点 IP: 192.168.137.1
  - 监听端口: 8765

天空端电脑 B
  - 连接 A 的热点
  - 连接 192.168.137.1:8765
  - 发送雷达 trace 数据
```

优点：

- 不需要额外路由器。
- 地面端 IP 相对固定，通常为 `192.168.137.1`。
- 适合近距离现场调试和初步试验。

注意事项：

- 地面端 Windows 防火墙需要允许 Python 或 TCP `8765` 端口通过专用网络。
- 天空端需要确认能 `ping 192.168.137.1`。
- 热点距离、天线姿态和无人机姿态会影响链路质量，正式试验前需要做距离测试。

### 4.2 稳定增强方案：专用 Wi-Fi/以太网数传网桥

如果无人机和地面端距离增加，建议使用专用无线网桥或机载路由模块，而不是只依赖笔记本热点。

推荐方式：

- 地面端和天空端接入同一个专用局域网。
- 地面端固定 IP，例如 `192.168.10.10`。
- 天空端配置地面端 IP 和端口 `8765`。
- 天线尽量外置，保证天空端与地面端视距链路。

优点：

- 链路距离和稳定性通常优于笔记本热点。
- 可以固定 IP，减少现场配置不确定性。
- 后续可以扩展到多设备局域网。

### 4.3 远距离备选方案：5G CPE 或 VPN

如果天空端和地面端不在同一局域网，建议通过 5G CPE、Tailscale、WireGuard 或 ZeroTier 组成虚拟局域网。

不建议直接把 TCP 端口暴露到公网。跨公网传输时应至少使用 VPN；如果后续需要云端接入，再考虑 TLS、MQTT 或 NATS。

## 5. 数据传输协议

当前工程的 TCP 帧格式可以继续使用：

```text
4 bytes: JSON header 长度，uint32 big-endian
N bytes: JSON header，UTF-8
8 bytes: binary payload 长度，uint64 big-endian
M bytes: binary payload，NPZ 二进制数据
```

trace 消息建议保持如下结构：

```json
{
  "type": "trace",
  "protocol": "uav-gpr-telemetry-v1",
  "run_id": "20260710_153000",
  "trace_index": 12,
  "timestamp": "2026-07-10T15:30:12.240Z",
  "payload_format": "npz",
  "payload_bytes": 59742,
  "sha256": "...",
  "config": {
    "start_mhz": 400,
    "stop_mhz": 4000,
    "points": 201,
    "ifbw_hz": 10000,
    "power_dbm": -20
  },
  "gnss": {
    "fix_quality": 1,
    "satellites": 12,
    "latitude": 31.0,
    "longitude": 121.0
  }
}
```

地面端收到 payload 后：

1. 计算 payload 的 `sha256`。
2. 与 header 中的 `sha256` 比对。
3. 校验通过后保存为 `remote_runs/<run_id>/trace_00012.npz`。
4. 写入 `remote_runs/<run_id>/run_metadata.jsonl`。
5. 返回 `ack_trace`。

## 6. 数据可靠性设计

### 6.1 天空端本地优先

天空端必须把本地保存作为第一优先级：

```text
采集 trace
  -> 天空端本地保存 NPZ/CSV/metadata
  -> 放入发送队列
  -> TCP 发送到地面端
  -> 收到 ack_trace 后标记已发送
```

这样即使链路中断，天空端仍保留完整数据。

### 6.2 地面端去重

地面端保存前应检查：

- `run_id + trace_index` 是否已经存在。
- 已存在文件的 `sha256` 是否一致。
- 一致则直接返回 ACK，不重复写入。
- 不一致则保存为冲突副本，并在日志中报警。

### 6.3 断线处理

当前 `RemoteSender` 已具备基础重连能力：发送失败后关闭 socket，等待 2 秒后重连，并保留当前 pending trace。

建议下一步增强为持久化发送队列：

```text
outbox_manifest.jsonl 或 SQLite
  run_id
  trace_index
  local_npz_path
  sha256
  sent_status
  ack_status
  retry_count
  last_send_time
```

这样即使天空端程序重启，也能继续补传未确认的数据。

### 6.4 补传机制

建议增加两种补传方式：

- 自动补传：天空端重连后扫描本地 manifest，把未 ACK 的 trace 重新发送。
- 地面端请求补传：地面端发现 `trace_index` 缺号后发送 `resend_request`。

补传请求示例：

```json
{
  "type": "resend_request",
  "protocol": "uav-gpr-telemetry-v1",
  "run_id": "20260710_153000",
  "missing_trace_indices": [15, 16, 17]
}
```

## 7. 带宽估算

当前默认配置为 400 MHz 到 4000 MHz、201 个频点、四个 S 参数通道。

粗略估算：

- 四通道原始复数 S 参数：约 12.9 KB。
- 四通道显示/校准后复数 S 参数：约 12.9 KB。
- A-scan、GNSS、配置、NPZ 压缩开销后，单条 trace 通常约几十 KB。
- 1 Hz 采集时，链路需求通常低于 100 KB/s。
- 10 Hz 采集时，链路需求通常约数百 KB/s 到 1 MB/s 量级。

普通局域网或稳定 Wi-Fi 足够承载当前 201 点数据。真正需要关注的是无线链路抖动、短时断连和地面端防火墙，而不是平均带宽。

## 8. 软件落地建议

### 8.1 保留当前 TCP 方案作为第一阶段

当前代码已经可用，建议第一阶段不引入新通信框架，继续使用：

```powershell
# 地面端电脑 A
python .\uav_gpr_ground_station.py --host 0.0.0.0 --port 8765 --out .\remote_runs

# 天空端电脑 B
python .\uav_gpr_qt.py
```

天空端默认连接：

```text
Remote IP: 192.168.137.1
Port: 8765
```

如果只需要接收和存档，不需要 GUI 显示，也可以在地面端运行：

```powershell
python .\uav_gpr_receiver.py --host 0.0.0.0 --port 8765 --out .\remote_runs
```

### 8.2 简化为“数据发送模式”

由于当前目标只是传输雷达数据，建议在 UI 或配置中增加一个模式：

```text
Remote mode:
  - Data only
  - Data + simple command
```

`Data only` 模式下：

- 天空端按本地按钮或本地任务流程采集。
- 地面端只接收 trace、状态和日志。
- 地面端不主动下发 start/single/stop。

这样现场调试更清晰，也减少误触发采集的风险。

### 8.3 增加状态心跳

建议天空端每 1-2 秒发送一次轻量 heartbeat：

```json
{
  "type": "heartbeat",
  "protocol": "uav-gpr-telemetry-v1",
  "run_id": "20260710_153000",
  "timestamp": "2026-07-10T15:30:12.000Z",
  "acquiring": true,
  "last_trace_index": 128,
  "queue_size": 3,
  "gnss_fix_quality": 1,
  "vna_connected": true
}
```

地面端可据此显示：

- 天空端在线/离线
- 正在采集/空闲
- 最近 trace 序号
- 待发送队列长度
- GNSS 状态
- VNA 状态

### 8.4 增加配置文件

建议增加：

```text
config/air_client.json
config/ground_station.json
```

天空端配置：

```json
{
  "remote_enabled": true,
  "remote_mode": "data_only",
  "ground_host": "192.168.137.1",
  "ground_port": 8765,
  "local_out_dir": "gpr_runs",
  "retry_interval_s": 2
}
```

地面端配置：

```json
{
  "listen_host": "0.0.0.0",
  "listen_port": 8765,
  "remote_out_dir": "remote_runs",
  "allow_commands": false
}
```

## 9. 测试流程

### 9.1 单机回环测试

在同一台电脑上测试协议和保存流程：

```powershell
python .\uav_gpr_receiver.py --host 127.0.0.1 --port 8765 --out .\remote_runs
```

天空端配置 `Remote IP = 127.0.0.1`，采集一条 trace，确认 `remote_runs/` 中出现 NPZ 和 metadata。

### 9.2 双电脑热点测试

1. 地面端 A 开 Windows 移动热点。
2. 天空端 B 连接该热点。
3. B 执行 `ping 192.168.137.1`。
4. A 启动地面站：

```powershell
python .\uav_gpr_ground_station.py --host 0.0.0.0 --port 8765 --out .\remote_runs
```

5. B 启动采集程序：

```powershell
python .\uav_gpr_qt.py
```

6. 连续采集 50-100 条 trace。
7. 检查 A、B 两端 trace 数量是否一致。
8. 抽查 5-10 个 NPZ 文件，确认 `sha256` 和字段完整性。

### 9.3 断线恢复测试

1. 连续采集过程中关闭 B 的 Wi-Fi 30 秒。
2. 确认 B 端继续本地保存。
3. 恢复 Wi-Fi。
4. 确认 B 端自动重连。
5. 检查地面端是否收到恢复后的 trace。
6. 在加入持久化 outbox 后，再验证断线期间的缺失 trace 能否自动补齐。

## 10. 推荐实施顺序

1. 继续使用当前 TCP + NPZ 方案完成双电脑数据回传试验。
2. 增加 `Data only` 模式，弱化不必要的远程控制。
3. 增加 heartbeat，让地面端明确显示天空端在线状态、采集状态和队列长度。
4. 增加本地 outbox manifest 或 SQLite，解决程序重启后的断点续传。
5. 增加 `resend_request`，让地面端可以主动请求缺失 trace。
6. 完成热点、无线网桥、远距离链路三种场景的稳定性测试。
7. 如果后续需要多地面端或云端接入，再评估 MQTT/NATS/gRPC，不建议在当前阶段过早引入。

## 11. 结论

针对当前“天空端和地面端两台电脑，只为传输雷达数据”的需求，最合适的方案是沿用现有 TCP socket 协议和压缩 NPZ 数据格式，采用天空端本地保存优先、地面端实时接收校验存档的架构。

短期内不需要重写为 WebSocket、MQTT 或 gRPC。当前代码已经具备可验证的基础链路，下一步应集中补强数据模式、心跳状态、持久化发送队列和断线补传。这样既能快速用于现场试验，又能逐步提高无人机实际飞行场景下的数据可靠性。
