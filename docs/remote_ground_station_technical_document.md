# UAV-GPR 双电脑远程采集地面站技术文档

版本日期：2026-07-05

## 1. 项目目标

当前目标是实现两台电脑之间的 UAV-GPR 远程采集控制与数据传输：

- A 电脑：作为地面站、热点主机、远程接收端和实时显示端。
- B 电脑：作为采集执行端，连接 LibreVNA 与 GNSS，执行实际采集。
- 操作逻辑：现场人员主要在 A 电脑操作，A 电脑下发开始、单次采集、停止命令；B 电脑接收命令后采集，并把每条 A-scan/trace 回传给 A。
- 显示目标：A 电脑实时显示 S 参数、A-scan、全极化 B-scan，即 `S11/S21/S12/S22` 四通道。
- 存储目标：B 电脑保留本地原始数据，A 电脑保存远程副本，降低网络波动导致的数据丢失风险。

## 2. 当前网络拓扑

当前现场连接方式：

```text
A 电脑
  - Windows 移动热点
  - 热点 IP: 192.168.137.1
  - 地面站 GUI: uav_gpr_ground_station.py
  - 监听端口: 8765
        ^
        | TCP 长连接
        |
B 电脑
  - 连接 A 电脑热点
  - 采集软件: uav_gpr_qt.py
  - LibreVNA USB
  - GNSS 串口
```

已经验证：

- B 电脑可以 `ping 192.168.137.1`。
- A 电脑端口 `8765` 可作为地面站监听端口。
- 当前协议 smoke test 已通过。
- 当前 Python 语法检查已通过。

## 3. 当前已实现功能

### 3.1 A 端地面站

脚本：

```text
uav_gpr_ground_station.py
```

启动命令：

```powershell
python .\uav_gpr_ground_station.py --host 0.0.0.0 --port 8765 --out .\remote_runs
```

主要功能：

- 监听 B 电脑连接。
- 下发远程采集命令：
  - `Start`：开始连续采集。
  - `Single`：采集一次。
  - `Stop`：停止采集。
- 可设置采集参数：
  - `Interval s`：采集间隔。
  - `A-scan count (0=continuous)`：采集多少个 A-scan，`0` 表示连续采集。
- 接收 B 端回传的 trace 数据。
- 保存远程数据到：

```text
remote_runs\<run_id>\trace_00001.npz
remote_runs\<run_id>\run_metadata.jsonl
```

- 实时显示：
  - 四通道 S 参数曲线：`S11/S21/S12/S22`
  - 四通道 A-scan
  - 四通道全极化 B-scan：`S11/S21/S12/S22`
  - GNSS fix、卫星数、经纬度等信息

B-scan 坐标定义：

```text
横轴：Trace
纵轴：Time, ns
```

### 3.2 B 端采集执行端

脚本：

```text
uav_gpr_qt.py
```

启动命令：

```powershell
python .\uav_gpr_qt.py
```

当前行为：

- 默认勾选 `Remote send`。
- 默认连接 A 电脑：

```text
Remote IP: 192.168.137.1
Port: 8765
```

- 打开程序后自动尝试连接 A 地面站。
- 收到 A 端命令后执行采集：
  - `start`
  - `single`
  - `stop`
- 收到 A 端下发的 `count` 和 `interval_s` 后，同步到本地采集参数并启动采集。
- 每完成一条 trace 后，将当前完整数据回传 A 电脑。

### 3.3 当前传输数据内容

每条 trace 通过内存压缩 NPZ 发送。payload 包含：

- `index`
- `config`
- `frequency_hz`
- `time_ns`
- `gnss`
- `raw_S11_real/raw_S11_imag`
- `raw_S21_real/raw_S21_imag`
- `raw_S12_real/raw_S12_imag`
- `raw_S22_real/raw_S22_imag`
- `display_S11_real/display_S11_imag`
- `display_S21_real/display_S21_imag`
- `display_S12_real/display_S12_imag`
- `display_S22_real/display_S22_imag`
- `trace_S11`
- `trace_S21`
- `trace_S12`
- `trace_S22`

这保证 A 端可以脱离 B 端本地文件，直接完成实时显示与远程存档。

## 4. 当前通信协议

当前版本使用 Python 标准库 TCP socket，不额外依赖第三方通信库。

### 4.1 帧格式

每个消息帧由三部分组成：

```text
4 bytes: JSON header 长度，uint32 big-endian
N bytes: JSON header, UTF-8
8 bytes: binary payload 长度，uint64 big-endian
M bytes: binary payload
```

### 4.2 消息类型

当前已实现：

```text
hello
ack
command
command_ack
trace
ack_trace
```

连接流程：

```text
B -> A: hello
A -> B: ack
A -> B: command(start/single/stop)
B -> A: command_ack
B -> A: trace + NPZ payload
A -> B: ack_trace
```

### 4.3 数据校验

每条 trace header 中包含：

```text
sha256
payload_bytes
run_id
trace_index
timestamp
```

A 端保存前会计算 payload 的 sha256，与 header 中的 sha256 对比。不一致时拒绝保存并报错。

## 5. 当前软件文件说明

```text
uav_gpr_qt.py
```

B 端主采集软件。新增了 `RemoteSender`，用于连接 A 地面站、接收远程命令、发送 trace 数据。

```text
uav_gpr_ground_station.py
```

A 端地面站 GUI。负责监听、远程控制、实时显示和远程保存。

```text
uav_gpr_receiver.py
```

早期命令行接收端，用于无 GUI 的链路测试。当前主要使用 `uav_gpr_ground_station.py`。

```text
docs/data_transmission_plan.md
```

早期传输方案文档，包含总体设计和操作说明。

```text
docs/remote_ground_station_technical_document.md
```

当前文档，用于描述已经实现的 A 地面站 + B 采集执行端方案，并给出下一阶段工作计划。

## 6. 当前方案优点

- 部署简单：A 开热点，B 连接热点即可。
- 不需要额外路由器。
- 不需要新增 Python 网络依赖。
- A 端可以直接控制 B 端采集，不再需要在远程 B 电脑点击启动。
- 已支持全极化数据传输和显示。
- 数据传输使用压缩 NPZ，比 CSV 更适合实时链路。
- B 端采集和远程发送分线程，不会直接阻塞采集 UI。

## 7. 当前方案限制

当前版本属于“最小可用验证版”，可以用于现场验证，但还不是最终稳定工程版本。主要限制如下：

- 自定义 TCP 协议还缺少完整状态机。
- 没有独立 heartbeat 心跳。
- 没有持久化发送队列，断线期间的待发数据目前主要依赖内存队列和本地文件。
- 没有完整的 `resend_request` 补传流程。
- 命令没有 `command_id`，只能做基础确认，不能精确追踪每条命令的执行结果。
- A 端当前只支持一个 B 端连接。
- 没有 token 或认证机制。
- 没有统一配置文件，IP、端口、输出目录仍主要在界面和命令行中配置。
- 没有专门的模拟采集模式，离线测试仍不够方便。
- Windows 防火墙、热点网络变化仍可能影响连接。

## 8. 下一步工作计划

### 阶段 1：固化当前 TCP 地面站版本

目标：让当前版本更适合现场试验。

计划任务：

- 增加连接状态指示：
  - 未连接
  - 已连接
  - 正在采集
  - 断线重连中
  - 发送队列积压
- 增加 heartbeat：
  - B 每 1-2 秒向 A 发送状态。
  - A 超时后标记 B 离线。
- 增加 B 端状态消息：
  - LibreVNA 连接状态
  - GNSS fix 状态
  - 当前 trace index
  - 采集是否运行
  - 本地保存目录
- 增加 `command_id`：
  - A 下发命令时生成唯一 ID。
  - B 回传命令已收到、已执行、执行失败。
- 增加 `end_run`：
  - B 采集完成后通知 A。
  - A 端界面明确显示任务结束。
- 增加日志文件：
  - A 端地面站日志。
  - B 端远程通信日志。
- 增加一键启动脚本：

```powershell
.\start_ground_station.ps1
.\start_acquisition_client.ps1
```

### 阶段 2：断线续传与数据一致性

目标：网络波动时不丢数据，恢复后自动补齐。

计划任务：

- B 端建立本地发送 manifest：

```text
trace_index
local_npz_path
sha256
sent_status
ack_status
last_send_time
retry_count
```

- B 端保存后再发送，发送成功后标记 ACK。
- A 端维护已接收 trace index 集合。
- A 端发现缺号后发送 `resend_request`。
- B 端按 `run_id + trace_index` 从本地 NPZ 补传。
- A 端保存前做去重：
  - 已存在且 sha256 一致：直接 ACK。
  - 已存在但 sha256 不一致：保存冲突副本并报警。

### 阶段 3：更强更稳定的工程化通信方案

当前自定义 TCP 协议适合快速验证，但长期建议升级为：

```text
gRPC 双向流 + Protocol Buffers + SQLite 本地发送队列
```

推荐原因：

- gRPC 支持双向 streaming，天然适合 A 下发控制命令、B 回传状态和 trace 数据。
- Protocol Buffers 可以固定消息结构，减少 JSON 字段随意变化导致的兼容问题。
- gRPC 自带 deadline、状态码、连接错误语义，比手写 TCP 更清晰。
- HTTP/2 流控机制更适合连续数据传输。
- SQLite outbox 可以实现可靠本地队列，断线后自动续传。
- 后续可以从 Python GUI 扩展到其他语言客户端。

建议协议结构：

```protobuf
service UavGprLink {
  rpc Stream(stream ClientMessage) returns (stream ServerMessage);
}

message ClientMessage {
  oneof payload {
    Hello hello = 1;
    Status status = 2;
    TraceHeader trace_header = 3;
    TraceChunk trace_chunk = 4;
    EndRun end_run = 5;
    ErrorReport error = 6;
  }
}

message ServerMessage {
  oneof payload {
    Ack ack = 1;
    Command command = 2;
    ResendRequest resend_request = 3;
  }
}
```

数据传输建议：

- 小消息：状态、命令、ACK 直接走 protobuf。
- 大数据：trace NPZ 按 chunk 分块发送。
- 每个 trace 使用：
  - `run_id`
  - `trace_index`
  - `chunk_index`
  - `chunk_count`
  - `sha256`
- A 端完成所有 chunk 后重组并校验。

SQLite 队列建议：

```text
outbox.db
  runs
  traces
  chunks
  acknowledgements
```

这样即使 B 端程序重启，也能继续知道哪些 trace 已经发送、哪些还需要补传。

### 阶段 4：多接收端或远距离链路

如果后续需要多个地面站、指挥中心、云端备份，建议引入消息中间件：

```text
MQTT 5.0 / NATS / EMQX
```

推荐场景：

- 多个 A 端同时观察同一台 B 端数据。
- 需要云端归档。
- 需要跨网络、跨地点接入。
- 需要 topic 级别权限控制。

topic 设计示例：

```text
uav_gpr/{device_id}/status
uav_gpr/{device_id}/command
uav_gpr/{device_id}/trace/header
uav_gpr/{device_id}/trace/chunk
uav_gpr/{device_id}/ack
```

对于当前“两台电脑热点直连”的现场形态，优先建议先完成 gRPC + SQLite 版本；等需要多端接入时再升级到 MQTT/NATS。

### 阶段 5：部署与现场可靠性

计划任务：

- 固定 A 端热点配置。
- 固定 B 端远程地面站 IP。
- 增加配置文件：

```text
config/ground_station.json
config/acquisition_client.json
```

- 增加自动启动选项。
- 增加防火墙检查脚本。
- 增加链路测试按钮：
  - ping
  - 端口连通
  - hello/ack
  - 测试 trace
- 增加模拟数据模式，方便无 LibreVNA 时测试 A 端显示和传输。
- 增加现场操作手册。

## 9. 推荐实施优先级

建议按以下顺序推进：

1. 先把当前 TCP 地面站版本用于短时间现场试采，验证 UI、全极化显示、A 端控制流程。
2. 修补当前版本的 heartbeat、状态显示、命令 ID、end_run。
3. 增加断线续传和本地 manifest。
4. 设计并实现 gRPC + protobuf + SQLite outbox 版本。
5. 完成对比测试：当前 TCP 版 vs gRPC 稳定版。
6. 再考虑 MQTT/NATS 多端或云端架构。

## 10. 验收测试建议

### 10.1 双电脑基础测试

- A 开热点。
- B 连接热点。
- B `ping 192.168.137.1`。
- A 启动地面站。
- B 启动采集端。
- A 显示 B 已连接。

### 10.2 远程控制测试

- A 设置 `A-scan count = 5`。
- A 点击 `Start`。
- B 自动采集 5 条。
- A 收到 5 条 trace。
- A 端四个 B-scan 均更新。

### 10.3 单次采集测试

- A 点击 `Single`。
- B 只采集 1 条。
- A 保存 1 个 NPZ 文件。

### 10.4 停止测试

- A 设置连续采集。
- A 点击 `Start`。
- 采集过程中点击 `Stop`。
- B 停止采集。
- A 显示任务停止或断开状态。

### 10.5 B-scan 坐标测试

- 连续采集多条。
- 检查 B-scan：

```text
横轴随 trace 数增加
纵轴保持 time ns
```

### 10.6 数据一致性测试

- 随机抽取 A 端保存的 NPZ。
- 检查四通道字段是否完整。
- 检查 `sha256` 是否与 metadata 一致。
- 检查 `trace_index` 是否连续。

## 11. 当前结论

当前版本已经完成从“B 端手动启动采集”到“A 端地面站统一控制”的关键转变，具备现场初步验证能力。

短期内应继续完善当前 TCP 地面站版本，尽快完成一次真实双电脑采集试验。中期建议升级为 gRPC 双向流 + SQLite 本地队列，以获得更稳定、更可维护、更容易扩展的远程采集通信架构。
