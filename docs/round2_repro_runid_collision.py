"""第2轮开工前预研：run_id 复用导致的跨任务去重身份冲突复现（真实代码，无 mock）。

背景（第1轮报告 §11 去重 / §18 第2轮接口）：地面端幂等身份是
``(run_id, trace_index) + sha256``。空中端当前形态下每次任务都会重建
RemoteSender，run_id 必然不同，该身份唯一性**侥幸成立**。第2轮"常驻采集
服务"让 sender 跨任务常驻后，这个不变量会被打破。

链路（全部真实生产代码）：
  RemoteSender 语义（uav_gpr_qt.py:1554 _wait_for_trace_ack）
    → 真实 gs.ReceiverThread（port=0 自绑，去重+落盘+ACK，:721-825）

四个场景：
  A 同 run_id + 同 trace_index + 不同 payload → 地面冲突拒绝、**不回 ACK**
    → 发送端 _wait_for_trace_ack 抛 ConnectionError → 无限重连重试 →
    _outstanding 永不归零 → drain 永久失败、队列卡死（死锁，非丢一条）
  B 同 run_id + 同 trace_index + 相同 payload → duplicate ACK、不重复落盘
    （磁盘 outbox 断点补传的前提：重放被幂等吸收）
  C 换新 run_id（方案 a：run_id 随任务轮换）→ 正常 ACK，冲突消失
    （地面 _on_acquisition_status 收到新 run_id 的 acquiring 会
    _begin_task → receiver.begin_run 重置内存去重表）
  D 地面进程重启（新建 ReceiverThread 实例）+ 原样重放 → 跨重启读盘比
    sha → duplicate ACK、不重复写 metadata（outbox 续传遇到地面重启的取证）

退出码：全部符合预期 0；任一不符 1（并打印实际值，供 CI/复审直接判读）。

运行：QT_QPA_PLATFORM=offscreen python docs/round2_repro_runid_collision.py
"""

from __future__ import annotations

import hashlib
import json
import os
import socket
import struct
import sys
import tempfile
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--no-sandbox")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import uav_gpr_ground_station as ground  # noqa: E402

#: 等待 ACK 的有界超时。超时即视为"地面未回 ACK"（场景 A 的死锁征兆）。
ACK_TIMEOUT_S = 2.0


def _frame(header: dict[str, object], payload: bytes = b"") -> bytes:
    header_bytes = json.dumps(header, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    return struct.pack("!I", len(header_bytes)) + header_bytes + struct.pack("!Q", len(payload)) + payload


def _read_frame(sock: socket.socket) -> dict[str, object]:
    (header_len,) = struct.unpack("!I", ground.recv_exact(sock, 4))
    header = json.loads(ground.recv_exact(sock, header_len).decode("utf-8"))
    (payload_len,) = struct.unpack("!Q", ground.recv_exact(sock, 8))
    if payload_len:
        ground.recv_exact(sock, payload_len)
    return header


def _start_receiver(out_dir: Path) -> tuple[ground.ReceiverThread, int]:
    """启动真实 ReceiverThread（QThread，port=0 由 OS 分配）。"""
    receiver = ground.ReceiverThread("127.0.0.1", 0, out_dir)
    receiver.start()
    for _ in range(200):
        if receiver.bound_port:
            return receiver, int(receiver.bound_port)
        time.sleep(0.02)
    raise RuntimeError("ReceiverThread 未在预期时间内绑定端口")


def _connect(port: int, run_id: str) -> socket.socket:
    sock = socket.create_connection(("127.0.0.1", port), timeout=5)
    sock.sendall(_frame({"type": "hello", "protocol": ground.PROTOCOL, "run_id": run_id, "timestamp": "repro"}))
    sock.settimeout(ACK_TIMEOUT_S)
    ack = _read_frame(sock)
    if ack.get("type") != "ack":
        raise RuntimeError(f"hello ack 异常: {ack}")
    return sock


def _send_trace(sock: socket.socket, run_id: str, index: int, marker: str) -> dict[str, object]:
    """按 uav-gpr-telemetry-v1 发送一条 trace，返回地面回包（或 no_ack 标记）。"""
    payload = f"NPZ-PAYLOAD::{marker}".encode("utf-8")
    header = {
        "type": "trace",
        "protocol": ground.PROTOCOL,
        "run_id": run_id,
        "trace_index": index,
        "timestamp": "repro",
        "payload_format": "npz",
        "payload_bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }
    sock.sendall(_frame(header, payload))
    try:
        return _read_frame(sock)
    except Exception as exc:  # noqa: BLE001 - 超时/断连都归为"没有 ACK"
        return {"type": f"<no_ack:{type(exc).__name__}>"}


def _brief(ack: dict[str, object]) -> str:
    return json.dumps(
        {k: ack.get(k) for k in ("type", "trace_index", "duplicate", "run_id") if k in ack},
        ensure_ascii=False,
    )


def main() -> int:  # noqa: C901 - 顺序复现场景，拆分反而降低可读性
    out_dir = Path(tempfile.mkdtemp(prefix="round2-runid-repro-"))
    failures: list[str] = []
    receiver, port = _start_receiver(out_dir)
    print(f"地面端：真实 ReceiverThread，落盘目录 {out_dir}")

    # ---- 场景 A：同 run_id + 同 index + 不同 payload → 无 ACK（死锁根源） ----
    print("\n=== A. sender 跨任务复用（同 run_id，trace_index 从 1 重来，payload 不同）===")
    task_a = _connect(port, "R1")
    ack_a1 = _send_trace(task_a, "R1", 1, "task1-trace1")
    print(f"  任务1 trace#1 → {_brief(ack_a1)}")
    ack_a2 = _send_trace(task_a, "R1", 1, "task2-trace1")
    print(f"  任务2 trace#1（同 (R1,1) 不同内容）→ {_brief(ack_a2)}")
    if ack_a1.get("type") != "ack_trace":
        failures.append(f"A1 期望 ack_trace，实际 {_brief(ack_a1)}")
    if ack_a2.get("type") == "ack_trace":
        failures.append(f"A2 期望无 ACK（冲突拒绝），实际收到 {_brief(ack_a2)} —— 死锁前提不成立")
    else:
        print("  [OK] 无 ACK → 对应发送端 _wait_for_trace_ack 抛 ConnectionError → 无限重试")
    task_a.close()

    # ---- 场景 B：相同 payload 重放 → duplicate ACK、不重复落盘（outbox 前提） ----
    print("\n=== B. 原样重放同一 payload（磁盘 outbox 补传的核心前提）===")
    disk_before = sorted(p.name for p in (out_dir / "R1").glob("*.npz"))
    metadata_before = (out_dir / "R1" / "run_metadata.jsonl")
    lines_before = len(metadata_before.read_text(encoding="utf-8").splitlines()) if metadata_before.exists() else 0
    replay = _connect(port, "R1")
    ack_b = _send_trace(replay, "R1", 1, "task1-trace1")
    print(f"  重放 (R1,1) 同内容 → {_brief(ack_b)}")
    disk_after = sorted(p.name for p in (out_dir / "R1").glob("*.npz"))
    lines_after = len(metadata_before.read_text(encoding="utf-8").splitlines()) if metadata_before.exists() else 0
    if ack_b.get("type") != "ack_trace" or not ack_b.get("duplicate"):
        failures.append(f"B 期望 duplicate ack_trace，实际 {_brief(ack_b)}")
    if disk_before != disk_after:
        failures.append(f"B 磁盘文件集变化：{disk_before} → {disk_after}")
    if lines_after != lines_before:
        failures.append(f"B metadata 行数变化（重复写入）：{lines_before} → {lines_after}")
    else:
        print(f"  [OK] duplicate ACK、磁盘无新文件、metadata 仍 {lines_after} 行 → 重放被幂等吸收")
    replay.close()

    # ---- 场景 C：换新 run_id（方案 a）→ 冲突消失 ----
    print("\n=== C. run_id 随任务轮换（方案 a：R1 → R2，trace_index 重新从 1 开始）===")
    receiver.begin_run("R2")  # 地面收到新 run_id 的 acquiring 时的等价动作（:1546/:1583）
    task_c = _connect(port, "R2")
    ack_c = _send_trace(task_c, "R2", 1, "task2-trace1")
    print(f"  新 run_id trace#1（与 A 中 payload 相同）→ {_brief(ack_c)}")
    if ack_c.get("type") != "ack_trace" or ack_c.get("duplicate"):
        failures.append(f"C 期望全新 ack_trace（非 duplicate），实际 {_brief(ack_c)}")
    else:
        print("  [OK] 换 run_id 后冲突消失，方案 a 可行")
    task_c.close()

    # ---- 场景 D：地面进程重启 + 原样重放 → 跨重启读盘比 sha → duplicate ----
    print("\n=== D. 地面重启后接收重放（跨重启去重，outbox 续传遇到地面重启）===")
    receiver.stop()
    time.sleep(0.2)
    restarted, port_d = _start_receiver(out_dir)
    restarted.begin_run("R1")
    replay_d = _connect(port_d, "R1")
    ack_d = _send_trace(replay_d, "R1", 1, "task1-trace1")
    print(f"  新地面实例收 (R1,1) 同内容 → {_brief(ack_d)}")
    disk_d = sorted(p.name for p in (out_dir / "R1").glob("*.npz"))
    if ack_d.get("type") != "ack_trace" or not ack_d.get("duplicate"):
        failures.append(f"D 期望跨重启 duplicate ack_trace，实际 {_brief(ack_d)}")
    if disk_d != disk_before:
        failures.append(f"D 磁盘文件集变化：{disk_before} → {disk_d}")
    else:
        print("  [OK] 内存集合为空仍按磁盘内容判重，不重复落盘")
    conflict_d = _send_trace(replay_d, "R1", 1, "different-bytes")
    if conflict_d.get("type") == "ack_trace":
        failures.append(f"D2 期望跨重启冲突拒绝无 ACK，实际 {_brief(conflict_d)}")
    else:
        print(f"  [OK] 跨重启同编号不同内容 → 拒绝且不 ACK：{_brief(conflict_d)}")
    replay_d.close()
    restarted.stop()
    time.sleep(0.1)

    print("\n=== 最终磁盘内容 ===")
    for path in sorted(out_dir.rglob("*")):
        if path.is_file():
            print(f"  {path.relative_to(out_dir)}  ({path.stat().st_size} bytes)")

    print("\n" + ("全部场景符合预期。" if not failures else f"{len(failures)} 项与预期不符："))
    for item in failures:
        print(f"  [FAIL] {item}")
    print(
        "\n结论：地面去重身份 (run_id, trace_index) 必须在单个 RemoteSender 生命周期内"
        "唯一对应一个 payload。\n常驻采集服务（t3）必须满足二者之一："
        "(a) run_id 随任务轮换（本例场景 C 已验证可行）；\n(b) trace_index 由 sender 统一单调发号。"
        "磁盘 outbox（t5）的重放身份必须等于原始发送身份（场景 B/D 已验证幂等吸收）。"
    )
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
