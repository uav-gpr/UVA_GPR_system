# UAV-GPR 双电脑远程数据传输技术方案

## 1. 目标与边界

本方案用于将 UAV-GPR 采集电脑上的雷达与 GNSS 数据实时传输到另一台电脑，实现远程监看、存档和后处理。

- 采集电脑 A：连接 LibreVNA、GNSS，运行现有 `uav_gpr_qt.py`，负责采集、校准、A-scan/B-scan 生成和本地落盘。
- 接收电脑 B：通过网络接收数据，显示当前轨迹、S 参数、A-scan/B-scan，并保存远程副本。
- 传输对象：每条 trace 的频域 S 参数、时域 trace、GNSS、采集参数、文件索引、时间戳、状态日志。
- 可靠性要求：网络断开时采集不中断；A 端继续本地保存，网络恢复后可补传缺失数据。

## 2. 推荐总体架构

采用“本地保存 + 实时消息推送 + 文件补传”的三层结构：

```text
LibreVNA/GNSS
    |
    v
采集电脑 A
  - UAV-GPR 采集程序
  - 本地 CSV/NPZ/metadata 保存
  - 数据发送服务 TCP/WebSocket
    |
    | Wi-Fi / 网线 / 5G 路由器局域网
    v
接收电脑 B
  - 数据接收服务
  - 实时显示界面
  - 远程数据目录
```

核心原则：

- A 端永远先落盘，再发送，避免传输失败导致原始数据丢失。
- 实时显示使用轻量二进制消息，完整数据使用压缩 NPZ 文件或分块传输。
- B 端按 `run_id + trace_index` 去重，支持重复发送和断点续传。

## 3. 网络与部署方式

### 3.1 优先方案：同一局域网 TCP/WebSocket

适用场景：两台电脑在同一个路由器、无人机地面站局域网、网线直连或移动 5G CPE 局域网内。

- A 端开启发送服务，默认连接 B 端 `IP:端口`。
- B 端开启接收服务，监听 `0.0.0.0:8765`。
- 推荐协议：WebSocket。

选择 WebSocket 的原因：

- Python 和 Qt 集成简单。
- 支持长连接、心跳、状态消息、二进制 payload。
- 后续可以复用同一协议接浏览器页面或独立监控程序。

### 3.2 备选方案：共享文件夹同步

适用场景：实时性要求低，但要求实现最快。

- A 端继续保存到 `gpr_runs/`。
- 将输出目录映射为 Windows 共享文件夹或同步盘。
- B 端监控目录新增文件，读取 CSV/NPZ。

缺点是延迟和文件锁不可控；现有程序已经考虑 `run_metadata.jsonl` 被同步工具锁定时写 sidecar metadata，这说明同步盘方式可用但不宜作为实时链路主方案。

### 3.3 备选方案：MQTT

适用场景：后续要扩展到多接收端、云端、指挥中心。

- 部署 Mosquitto/EMQX broker。
- A 端发布 `uav_gpr/{run_id}/trace`。
- B 端订阅 topic。

缺点是需要额外 broker，对“两台电脑直连”略重。

## 4. 数据内容与格式

### 4.1 每条 trace 的逻辑数据

现有程序每次采集已具备以下数据：

- `index`：trace 序号。
- `frequency_hz`：频率数组。
- `raw_sparams`：`S11/S21/S12/S22` 原始复数数组。
- `display_sparams`：校准、背景扣除后的显示复数数组。
- `time_ns`：时域横轴。
- `trace_by_channel`：四通道时域 trace。
- `fix`：GNSS GGA/RMC 解析结果。
- `path_text`：本地保存路径。

传输层不需要重新设计采集数据结构，只需要把这些对象序列化。

### 4.2 推荐消息类型

使用 JSON header + binary payload：

```json
{
  "type": "trace",
  "protocol": "uav-gpr-telemetry-v1",
  "run_id": "20260702_153000_A",
  "trace_index": 12,
  "timestamp": "2026-07-02T15:30:12.240+08:00",
  "payload_format": "npz",
  "payload_bytes": 59742,
  "sha256": "...",
  "config": {
    "start_mhz": 400,
    "stop_mhz": 4000,
    "points": 201
  },
  "gnss": {
    "lat": 31.0,
    "lon": 121.0,
    "fix_quality": 1
  }
}
```

随后发送二进制 payload。payload 建议直接使用内存中的 `np.savez_compressed` 生成，字段与现有 `.npz` 保持一致：

- `frequency_hz`
- `time_ns`
- `trace`
- `raw_S11_real/raw_S11_imag` 等四通道数据
- `display_S11_real/display_S11_imag` 等四通道数据
- `config`
- `gnss`

### 4.3 状态消息

除 trace 外，还需要发送轻量状态：

- `hello`：连接建立，包含软件版本、电脑名、run_id。
- `heartbeat`：每 2 秒发送一次，包含采集状态、当前 trace、队列积压数量。
- `log`：采集日志，例如 LibreVNA 连接、GNSS 状态、校准状态。
- `end_run`：一次采集任务结束。
- `resend_request`：B 端请求补传缺失 trace。

## 5. 传输流程

### 5.1 实时 trace 推送

1. B 端启动接收服务，监听端口。
2. A 端配置 B 端 IP 和端口，点击“连接远程端”。
3. A 端发送 `hello`，B 端回复 `ack`。
4. A 端每采集一条 trace：
   - 先调用现有 `save_trace()` 保存本地 CSV/NPZ。
   - 生成同等内容的压缩 NPZ payload。
   - 发送 trace header。
   - 发送 binary payload。
   - 等待 B 端 `ack_trace`。
5. B 端校验 `sha256`，保存到远程目录。
6. B 端更新实时图像和轨迹。

### 5.2 断线处理

A 端维护发送队列：

- 已保存但未确认的数据进入 `pending` 队列。
- 网络断开时不停止采集，只记录“远程断开”状态。
- 连接恢复后从 `pending` 队列按 trace 序号补传。
- B 端如果发现序号缺口，主动发送 `resend_request`。

### 5.3 去重与一致性

B 端保存前检查：

- `run_id + trace_index` 是否已存在。
- `sha256` 是否一致。
- 如果已存在且校验一致，直接返回 ack。
- 如果已存在但校验不一致，保存为冲突文件并报警。

## 6. 带宽估算

以当前默认 201 点、四个 S 参数为例：

- 原始复数：4 通道 x 201 点 x real/imag x 8 字节，约 12.9 KB。
- display 复数同量级，约 12.9 KB。
- 时域 trace、metadata、压缩开销后，现有 NPZ 样例约 47 KB 到 60 KB。
- 如果 1 秒 1 条 trace，带宽约 60 KB/s，普通 Wi-Fi 足够。
- 如果 10 Hz 采集，约 600 KB/s，仍在局域网可承受范围内。

CSV 文件样例约 100 KB 到 210 KB，适合人工查看，不建议作为实时传输主格式。

## 7. 软件改造方案

### 7.1 A 端采集程序改造

在 `uav_gpr_qt.py` 中增加远程传输模块：

- 新增 `RemoteSender` 类：
  - 管理 WebSocket 连接。
  - 发送 hello、heartbeat、trace、log。
  - 维护 pending 队列。
  - 支持自动重连。
- 在采集页面增加配置：
  - 远程传输启用开关。
  - 接收端 IP。
  - 接收端端口。
  - 连接/断开按钮。
  - 发送状态、队列数量、最后 ack trace。
- 在 `AcquisitionWorker.trace_ready` 或保存完成后触发发送。

建议不要让发送逻辑阻塞采集线程。采集线程只把数据放入队列，发送线程独立工作。

### 7.2 B 端接收程序

新增独立程序，例如 `uav_gpr_receiver.py`：

- 启动 WebSocket server。
- 接收 header 和 binary payload。
- 校验 sha256。
- 保存到 `remote_runs/{run_id}/`。
- 写入 `run_metadata.jsonl`。
- 可选：使用 PySide6/pyqtgraph 复用现有绘图组件显示实时 B-scan。

第一阶段可以先做命令行接收服务，确认链路稳定后再做 GUI。

### 7.3 依赖建议

在 `requirements.txt` 增加：

```text
websockets>=12.0
```

如果希望与 Qt 事件循环更紧密，也可以使用 `PySide6.QtWebSockets`，但 `websockets` 的服务端和断线重连实现更直接。

## 8. 安全与配置

局域网内建议先使用简单 token：

- B 端配置 `REMOTE_TOKEN`。
- A 端 hello 消息带 token。
- B 端校验失败直接断开。

如果跨公网传输，不建议裸 WebSocket，应使用：

- VPN，例如 Tailscale、WireGuard、ZeroTier。
- 或 `wss://` TLS 证书。

## 9. 测试计划

### 9.1 单机回环测试

- B 端监听 `127.0.0.1:8765`。
- A 端连接 `127.0.0.1:8765`。
- 采集一次，确认 B 端收到 NPZ 并可读取。

### 9.2 双电脑局域网测试

- B 端执行 `ipconfig` 获取 IPv4 地址。
- A 端连接 B 端 IP。
- Windows 防火墙放行 B 端 Python 或端口 `8765`。
- 连续采集 100 条，检查两端 trace 数量一致。

### 9.3 断线恢复测试

- 采集中断开 Wi-Fi 30 秒。
- A 端应继续本地保存。
- 恢复网络后 B 端应补齐缺失序号。

### 9.4 数据一致性测试

- 随机抽取 10 条 trace。
- 比较 A 端和 B 端 NPZ 的 sha256。
- 检查 GNSS、频率数组、四通道 S 参数长度一致。

## 10. 分阶段实施计划

### 阶段 1：最小可用链路

- 新建 `uav_gpr_receiver.py` 命令行接收端。
- A 端增加发送线程和 IP/端口配置。
- 实现 trace NPZ payload 发送和远程保存。
- 完成单机与双电脑测试。

### 阶段 2：可靠性增强

- 增加 pending 队列。
- 增加 ack、resend_request、断线重连。
- 增加 sha256 校验和去重。
- 增加状态日志显示。

### 阶段 3：远程实时显示

- B 端增加 PySide6 GUI。
- 显示 GNSS 轨迹、S 参数、A-scan、B-scan。
- 增加运行选择、数据回放、导出。

### 阶段 4：工程化部署

- 配置文件保存 IP、端口、token、输出目录。
- 增加启动脚本。
- 编写现场操作手册。
- 如需公网，接入 VPN 或 TLS。

## 11. 推荐结论

建议采用 WebSocket 双电脑直连方案：

- A 端继续作为唯一采集主控，保证硬件链路稳定。
- A 端本地保存作为第一可靠存储。
- B 端实时接收压缩 NPZ，并按 trace 序号确认。
- 通过 pending 队列和补传机制处理网络波动。

该方案对现有代码侵入较小，能从命令行接收端快速验证，再逐步扩展到完整远程显示界面。

## 12. 当前 TCP 地面站版本操作步骤

当前工程已加入一个标准库 TCP 地面站版本，不需要额外安装 `websockets`。

### 12.1 A 电脑：热点主机、接收端和显示端

A 电脑已经开启热点，热点 IP 为：

```text
192.168.137.1
```

在 A 电脑工程目录运行地面站界面：

```powershell
python .\uav_gpr_ground_station.py --host 0.0.0.0 --port 8765 --out .\remote_runs
```

界面功能：

- `Start`：命令 B 电脑开始连续采集。
- `Single`：命令 B 电脑采集一次。
- `Stop`：命令 B 电脑停止采集。
- 实时显示 S 参数、A-scan、S11 B-scan、GNSS 信息。
- 自动保存远程数据到 `remote_runs\<run_id>\`。

### 12.2 B 电脑：采集端与远程执行端

B 电脑连接 A 的热点后，运行采集程序：

```powershell
python .\uav_gpr_qt.py
```

采集程序默认启用远程连接：

- `Remote send` 默认勾选。
- `Remote IP` 默认 `192.168.137.1`。
- `Port` 默认 `8765`。

如果 A 地面站已经启动，B 端打开采集程序后会自动连接 A。之后不需要在 B 电脑上点击“开始采集”，在 A 地面站点击 `Start` 或 `Single` 即可。

B 端每完成一条 trace 会自动发送到 A 端。A 端会保存到：

```text
remote_runs\<run_id>\trace_00001.npz
remote_runs\<run_id>\run_metadata.jsonl
```

### 12.3 防火墙

如果 B 能 ping 通 `192.168.137.1`，但接收端没有任何连接日志，需要在 A 电脑 Windows 防火墙中允许 Python 访问专用网络，或放行 TCP 端口 `8765`。
