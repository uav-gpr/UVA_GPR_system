"""第2轮 t5：磁盘 outbox 断点补传（R-B-01…R-B-10 全覆盖）。

分两段：
* **模块段**（前半）：disk_outbox.py 自身契约——原子写、帧字节与线上字节一致、
  扫描排序、身份门禁、损坏隔离、划除幂等、写盘失败不静默、R-B-03 的
  subprocess **新进程**崩溃恢复基准、R-B-08 专项计时+保守上界断言。
* **接线段**（``RemoteSenderWiringTests`` 起）：RemoteSender 与 outbox 的衔接
  ——R-B-01 落盘前置、§4.5 legacy 身份 skip、R-B-05 恢复条目计入 outstanding
  与"划除失败不减 outstanding"、§4.4 重放复用同一条连接靠带内 hello 切身份
  （零新增连接/零新增线程）、R-B-09 framing 零变更、R-B-10 ``outbox_dir=None``
  默认禁用且不留任何目录。

除"注入磁盘故障"类用例外，全部使用真实 ``ReceiverThread`` + port=0 + 真实
``RemoteSender``，零 mock；不创建任何 Qt 控件，因此不受第2轮环境漂移
（WebEngine 拆除段错误）影响。

运行：QT_QPA_PLATFORM=offscreen python -m unittest tests.test_round2_disk_outbox
"""

from __future__ import annotations

import hashlib
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import uav_gpr_ground_station as ground  # noqa: E402
from disk_outbox import (  # noqa: E402
    CORRUPT_DIR_NAME,
    FRAME_SUFFIX,
    OUTBOX_DIR_NAME,
    RUN_ID_CLASS_AIR_TASK,
    RUN_ID_CLASS_CALIBRATION,
    RUN_ID_CLASS_IDLE,
    RUN_ID_CLASS_LOCAL,
    DiskOutbox,
    OutboxFrameError,
    OutboxWriteError,
    classify_run_id,
    frame_bytes,
    parse_frame,
    sanitize_run_dir_name,
)

#: 采集任务式身份（§3.7-1）。
RUN_A = "20260906_101112_0001"
RUN_B = "20260906_101113_0002"


def make_payload(marker: str = "trace") -> bytes:
    """一段确定性的假 NPZ payload（outbox 不解析内容，只按字节保护）。"""
    return f"NPZ::{marker}::{'x' * 64}".encode("utf-8")


def make_header(
    run_id: str = RUN_A,
    trace_index: int = 1,
    *,
    payload: bytes | None = None,
    msg_type: str = "trace",
    **overrides: object,
) -> dict[str, object]:
    body = make_payload(f"{run_id}-{trace_index}") if payload is None else payload
    header: dict[str, object] = {
        "type": msg_type,
        "protocol": ground.PROTOCOL,
        "run_id": run_id,
        "trace_index": trace_index,
        "timestamp": "2026-09-06T10:11:12+00:00",
        "payload_format": "npz",
        "payload_bytes": len(body),
        "sha256": hashlib.sha256(body).hexdigest(),
    }
    header.update(overrides)
    return header


class TempOutboxCase(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory(prefix="round2-outbox-")
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.outbox = DiskOutbox(self.root)


class ModulePurityTests(unittest.TestCase):
    """R-B-10：disk_outbox.py 必须是纯 Python（不依赖 Qt）。"""

    def test_no_qt_imports_in_module_source(self) -> None:
        source = (ROOT / "disk_outbox.py").read_text(encoding="utf-8")
        for token in ("PySide6", "QtCore", "QtWidgets", "import uav_gpr_qt", "uav_gpr_ground_station"):
            with self.subTest(token=token):
                # 允许出现在注释/文档字符串里（说明"刻意不 import"），
                # 但不得作为真实 import 语句存在。
                lines = [
                    ln.strip()
                    for ln in source.splitlines()
                    if ln.strip().startswith(("import ", "from "))
                ]
                self.assertFalse(
                    any(token in ln for ln in lines),
                    f"disk_outbox.py 不得 import {token}（R-B-10 纯 Python 要求）",
                )

    def test_import_works_without_qt_loaded(self) -> None:
        # 子进程里只 import disk_outbox：若它隐式拖入 Qt，Qt 缺失/线程问题会暴露。
        proc = subprocess.run(
            [
                sys.executable,
                "-c",
                "import sys; sys.path.insert(0, %r); import disk_outbox, json; "
                "print(json.dumps(sorted(n for n in sys.modules if 'PySide' in n)))" % str(ROOT),
            ],
            capture_output=True,
            text=True,
            timeout=120,
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(json.loads(proc.stdout.strip()), [], "import disk_outbox 不得拖入 Qt 模块")


class IdentityVocabularyTests(unittest.TestCase):
    """§3.7-9 身份词汇表：四类身份的判据唯一来源。"""

    def test_four_classes(self) -> None:
        self.assertEqual(classify_run_id(RUN_A), RUN_ID_CLASS_AIR_TASK)
        self.assertEqual(classify_run_id("20260906_101112_0001_c"), RUN_ID_CLASS_CALIBRATION)
        self.assertEqual(classify_run_id("20260906_101112_idle"), RUN_ID_CLASS_IDLE)
        self.assertEqual(classify_run_id("start-1757000000"), RUN_ID_CLASS_LOCAL)
        self.assertEqual(classify_run_id(None), RUN_ID_CLASS_LOCAL)
        self.assertEqual(classify_run_id(123), RUN_ID_CLASS_LOCAL)

    def test_bare_seq_not_air_task(self) -> None:
        # 裸 seq（P1 裁定要消灭的写法）不得被当成采集任务身份。
        self.assertEqual(classify_run_id("20260906_101112_1"), RUN_ID_CLASS_LOCAL)

    def test_sanitize_parity_with_ground_safe_name(self) -> None:
        # 与地面 safe_name 逐字符等价（防止两边漂移导致目录名不一致）。
        for value in (RUN_A, "a/b\\c", "run name", "", "start-1.2_3", "..\\evil", "ab-cd.ef"):
            with self.subTest(value=value):
                self.assertEqual(sanitize_run_dir_name(value), ground.safe_name(value, "run"))

    def test_traversal_cannot_escape_outbox(self) -> None:
        box = DiskOutbox(Path(tempfile.mkdtemp()))
        # '.'/'..' 会被 sanitize 成 '_./_'/'_._'（safe_name 允许 '.'），
        # 但也可能原样通过 —— run_dir 必须 fail-closed 拒绝任何点号开头名字。
        for hostile in ("..", "../..", "..\\..", ".", "./x"):
            with self.subTest(hostile=hostile):
                with self.assertRaises(OutboxFrameError):
                    box.run_dir(hostile)
        # 正常名字仍解析在 outbox 根之下。
        target = box.run_dir(RUN_A)
        self.assertEqual(target.parent, box.outbox_dir)


class AtomicWriteTests(TempOutboxCase):
    """R-B-01 / §4.3-1：原子写 + 无残留 tmp。"""

    def test_append_creates_frame_and_no_tmp_leftover(self) -> None:
        payload = make_payload("a1")
        header = make_header(trace_index=1, payload=payload)
        path = self.outbox.append_message(header, payload)
        self.assertEqual(path, self.root / OUTBOX_DIR_NAME / RUN_A / f"000001{FRAME_SUFFIX}")
        self.assertTrue(path.is_file())
        leftovers = [p.name for p in path.parent.iterdir() if p.name.endswith(".tmp")]
        self.assertEqual(leftovers, [], "os.replace 后不得残留 .tmp")

    def test_written_bytes_equal_wire_frame(self) -> None:
        payload = make_payload("parity")
        header = make_header(trace_index=7, payload=payload)
        path = self.outbox.append_message(header, payload)
        self.assertEqual(path.read_bytes(), frame_bytes(header, payload))

    def test_same_identity_rewrite_is_idempotent(self) -> None:
        payload = make_payload("dup")
        header = make_header(trace_index=3, payload=payload)
        first = self.outbox.append_message(header, payload)
        second = self.outbox.append_message(header, payload)
        self.assertEqual(first, second)
        self.assertEqual(self.outbox.pending_count(), 1)

    def test_directory_layout_matches_contract(self) -> None:
        payload = make_payload("layout")
        self.outbox.append_message(make_header(run_id=RUN_B, trace_index=42, payload=payload), payload)
        self.assertTrue((self.root / OUTBOX_DIR_NAME / RUN_B / f"000042{FRAME_SUFFIX}").is_file())
        self.assertEqual(self.outbox.frame_path(RUN_B, 42).name, f"000042{FRAME_SUFFIX}")


class FrameParityTests(unittest.TestCase):
    """R-B-01/R-B-09：帧字节必须与 send_remote_frame 写到 socket 的字节逐字节一致。"""

    def test_frame_bytes_match_real_sendall(self) -> None:
        import air_core as air  # 测试侧允许依赖 Qt 模块（本模块本身不依赖）

        payload = make_payload("wire")
        header = make_header(trace_index=11, payload=payload)

        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        self.addCleanup(srv.close)
        port = srv.getsockname()[1]
        captured: list[bytes] = []

        def accept_and_capture() -> None:
            try:
                conn, _ = srv.accept()
            except OSError:
                return  # 监听 socket 已被 cleanup 关闭
            conn.settimeout(5.0)
            self.addCleanup(conn.close)
            try:
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    captured.append(chunk)
            except OSError:
                pass

        thread = threading.Thread(target=accept_and_capture, daemon=True)
        thread.start()
        client = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.addCleanup(client.close)
        try:
            air.send_remote_frame(client, header, payload)
        finally:
            client.close()
        thread.join(timeout=6.0)

        wire = b"".join(captured)
        self.assertEqual(
            frame_bytes(header, payload),
            wire,
            "disk_outbox.frame_bytes 与 uav_gpr_qt.send_remote_frame 线上字节不一致（R-B-01 前提破裂）",
        )
        # 反向：解析线上字节必须还原出同一 (header, payload)。
        parsed_header, parsed_payload = parse_frame(wire)
        self.assertEqual(parsed_header, header)
        self.assertEqual(parsed_payload, payload)

    def test_json_compact_form_is_pinned(self) -> None:
        # header 定型必须是无空格的紧凑形式且 ensure_ascii=False（中文值不转义）。
        header = make_header(trace_index=1)
        header["note"] = "中文值"
        raw = frame_bytes(header, b"")
        (_hlen,) = struct.unpack("!I", raw[:4])
        text = raw[4 : 4 + _hlen].decode("utf-8")
        self.assertIn("中文值", text, "ensure_ascii=False 必须保留原始字符")
        self.assertNotIn(", ", text)
        self.assertNotIn(": ", text)


class ScanOrderingTests(TempOutboxCase):
    """§4.1/§4.4：扫描按 (run_id, trace_index) 排序，字典序==时间序。"""

    def test_sorted_by_run_then_index(self) -> None:
        for run in (RUN_B, RUN_A):
            for index in (3, 1, 2):
                payload = make_payload(f"{run}-{index}")
                self.outbox.append_message(make_header(run_id=run, trace_index=index, payload=payload), payload)
        entries = self.outbox.scan_pending()
        keys = [(e.run_id, e.trace_index) for e in entries]
        self.assertEqual(
            keys,
            [(RUN_A, 1), (RUN_A, 2), (RUN_A, 3), (RUN_B, 1), (RUN_B, 2), (RUN_B, 3)],
        )

    def test_lexicographic_equals_temporal_with_padded_seq(self) -> None:
        # %04d 的意义：同秒连号 9/10 不倒挂（P1 裁定）。
        early = "20260906_101112_0009"
        later = "20260906_101112_0010"
        self.assertLess(early, later)
        for run in (later, early):
            payload = make_payload(run)
            self.outbox.append_message(make_header(run_id=run, trace_index=1, payload=payload), payload)
        self.assertEqual([e.run_id for e in self.outbox.scan_pending()], [early, later])

    def test_entry_carries_identity_and_bytes(self) -> None:
        payload = make_payload("full")
        header = make_header(trace_index=5, payload=payload)
        self.outbox.append_message(header, payload)
        entry = self.outbox.scan_pending()[0]
        self.assertEqual(entry.run_id, RUN_A)
        self.assertEqual(entry.trace_index, 5)
        self.assertEqual(entry.sha256, hashlib.sha256(payload).hexdigest())
        self.assertEqual(entry.frame, frame_bytes(header, payload))
        self.assertEqual(entry.payload, payload)
        self.assertEqual(entry.header, header)
        self.assertTrue(entry.is_trace)
        self.assertEqual(entry.size_bytes, len(entry.frame))

    def test_empty_when_absent(self) -> None:
        self.assertEqual(self.outbox.scan_pending(), [])
        self.assertEqual(self.outbox.pending_count(), 0)
        self.assertEqual(self.outbox.pending_bytes(), 0)


class RemovalTests(TempOutboxCase):
    """§4.2：划除即持久化确认；duplicate ACK 重复划除安全。"""

    def test_remove_marks_acked(self) -> None:
        payload = make_payload("rm")
        self.outbox.append_message(make_header(trace_index=2, payload=payload), payload)
        entry = self.outbox.scan_pending()[0]
        self.assertTrue(self.outbox.remove(entry))
        self.assertFalse(entry.path.exists())
        self.assertEqual(self.outbox.pending_count(), 0)
        stats = self.outbox.statistics()
        self.assertEqual(stats["removed"], 1)
        self.assertEqual(stats["ack_removal_failures"], 0)

    def test_double_removal_is_idempotent(self) -> None:
        payload = make_payload("dbl")
        self.outbox.append_message(make_header(trace_index=2, payload=payload), payload)
        entry = self.outbox.scan_pending()[0]
        self.assertTrue(self.outbox.mark_acked(entry))
        self.assertTrue(self.outbox.mark_acked(entry), "重复划除必须安全（duplicate ACK 场景）")
        stats = self.outbox.statistics()
        self.assertEqual(stats["removed"], 1)
        self.assertEqual(stats["already_removed"], 1)
        self.assertEqual(stats["ack_removal_failures"], 0)

    def test_empty_run_dir_pruned(self) -> None:
        payload = make_payload("prune")
        self.outbox.append_message(make_header(trace_index=1, payload=payload), payload)
        self.outbox.remove(self.outbox.scan_pending()[0])
        self.assertFalse((self.root / OUTBOX_DIR_NAME / RUN_A).exists(), "空 run 目录应清理，避免误统计")

    def test_removal_failure_is_reported_not_faked(self) -> None:
        payload = make_payload("lock")
        path = self.outbox.append_message(make_header(trace_index=4, payload=payload), payload)

        class _Entry:
            """伪条目：删除时抛 OSError，模拟"文件被占用"真故障。"""

            def __init__(self, real: Path) -> None:
                self.path = real

        original_unlink = Path.unlink

        def fake_unlink(self, *args, **kwargs):  # noqa: ANN001
            raise OSError("simulated: device busy")

        Path.unlink = fake_unlink  # type: ignore[method-assign]
        try:
            self.assertFalse(self.outbox.remove(_Entry(path)), "删除失败必须返回 False（不得伪装确认）")
        finally:
            Path.unlink = original_unlink  # type: ignore[method-assign]
        self.assertEqual(self.outbox.statistics()["ack_removal_failures"], 1)
        self.assertEqual(self.outbox.statistics()["removed"], 0)
        self.assertTrue(path.exists(), "删除失败时帧文件必须仍在（下次扫描继续补发）")


class FailClosedWriteTests(TempOutboxCase):
    """§4.3-2：写盘失败明确上报，绝不静默伪装成功。"""

    def test_write_failure_raises_and_counts(self) -> None:
        payload = make_payload("fail")
        header = make_header(trace_index=1, payload=payload)
        # 注入：让 outbox 根成为一个普通文件 ⇒ mkdir 必然失败。
        box = DiskOutbox(self.root)
        (self.root / OUTBOX_DIR_NAME).write_text("not a directory", encoding="utf-8")
        with self.assertRaises(OutboxWriteError):
            box.append_message(header, payload)
        stats = box.statistics()
        self.assertEqual(stats["write_failures"], 1)
        self.assertEqual(stats["appended"], 0)

    def test_unreadable_existing_frame_is_not_overwritten(self) -> None:
        # 已存在但读不出来 ⇒ 绝不能当"不存在"继续覆盖（那会静默丢失一条未确认
        # 数据）。fail-closed：抛出 + 计数 + 原文件字节不变。
        payload = make_payload("keepme")
        path = self.outbox.append_message(make_header(trace_index=6, payload=payload), payload)
        original = path.read_bytes()

        original_read = Path.read_bytes

        def fake_read(self, *args, **kwargs):  # noqa: ANN001
            if Path(self).name == path.name:
                raise OSError("simulated: I/O error")
            return original_read(self, *args, **kwargs)

        Path.read_bytes = fake_read  # type: ignore[method-assign]
        try:
            new_payload = make_payload("different-but-valid")
            with self.assertRaises(OutboxWriteError):
                self.outbox.append_message(make_header(trace_index=6, payload=new_payload), new_payload)
        finally:
            Path.read_bytes = original_read  # type: ignore[method-assign]
        self.assertEqual(path.read_bytes(), original, "不可读时必须拒绝覆盖未确认数据")
        self.assertEqual(self.outbox.statistics()["write_failures"], 1)
        self.assertEqual(self.outbox.statistics()["appended"], 1, "被拒绝的重写不得计入 appended")

    def test_no_tmp_left_after_failed_write(self) -> None:
        payload = make_payload("fail2")
        header = make_header(trace_index=2, payload=payload)
        box = DiskOutbox(self.root)
        (self.root / OUTBOX_DIR_NAME).write_text("not a directory", encoding="utf-8")
        with self.assertRaises(OutboxWriteError):
            box.append_message(header, payload)
        self.assertEqual(list(self.root.rglob("*" + ".tmp")), [])


class IdentityGuardTests(TempOutboxCase):
    """§3.7：outbox 只接受采集任务身份（idle/校准/本地一律拒收）。"""

    def test_rejects_non_air_task_identity(self) -> None:
        for bad_run in ("20260906_101112_idle", "20260906_101112_0001_c", "start-1757000000", "", "RUN"):
            with self.subTest(run_id=bad_run):
                payload = make_payload(bad_run or "empty")
                header = make_header(run_id=bad_run, trace_index=1, payload=payload)
                with self.assertRaises(OutboxFrameError):
                    self.outbox.append_message(header, payload)
        self.assertEqual(self.outbox.pending_count(), 0)

    def test_rejects_bad_trace_index(self) -> None:
        for bad_index in (0, -1, True, "3", None, 2.5):
            with self.subTest(trace_index=bad_index):
                payload = make_payload("idx")
                header = make_header(trace_index=1, payload=payload)
                header["trace_index"] = bad_index
                with self.assertRaises((OutboxFrameError, ValueError)):
                    self.outbox.append_message(header, payload)

    def test_rejects_non_trace_type(self) -> None:
        # 状态消息绝不落 outbox：陈旧 acquiring/stopped 重放会污染地面当前基线。
        header = make_header(trace_index=1, msg_type="acquisition_status")
        with self.assertRaises(OutboxFrameError):
            self.outbox.append(frame_bytes(header, b""), RUN_A, 1)
        self.assertEqual(self.outbox.pending_count(), 0)

    def test_rejects_mismatched_header_argument(self) -> None:
        payload = make_payload("mm")
        good = make_header(trace_index=1, payload=payload)
        other = make_header(trace_index=1, payload=make_payload("other"))
        with self.assertRaises(OutboxFrameError):
            self.outbox.append(frame_bytes(good, payload), RUN_A, 1, header=other)


class CorruptIsolationTests(TempOutboxCase):
    """§4.3-3 / R-B-06：损坏/半文件隔离不删除、不重发、计数可观测。"""

    def _write_raw(self, name: str, data: bytes, run: str = RUN_A) -> Path:
        target = self.root / OUTBOX_DIR_NAME / run / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        return target

    def test_truncated_frame_isolated(self) -> None:
        payload = make_payload("trunc")
        path = self._write_raw("000001" + FRAME_SUFFIX, frame_bytes(make_header(trace_index=1, payload=payload), payload)[:-9])
        self.assertEqual(self.outbox.scan_pending(), [])
        self.assertFalse(path.exists(), "损坏帧必须被移出原位")
        archived = list(self.outbox.corrupt_dir.iterdir())
        self.assertEqual(len(archived), 1)
        self.assertTrue(archived[0].name.startswith(f"{RUN_A}__"), f"隔离文件名应携带来源 run：{archived[0].name}")
        self.assertTrue(archived[0].name.endswith(FRAME_SUFFIX), "隔离保留原始后缀便于取证")
        stats = self.outbox.statistics()
        self.assertEqual(stats["corrupt_files"], 1)
        self.assertEqual(stats["pending_files"], 0)
        report = json.dumps(self.outbox.damage_report, ensure_ascii=False)
        self.assertIn("截断", report)
        self.assertEqual(self.outbox.corrupt_dir.relative_to(self.root).as_posix(), f"{OUTBOX_DIR_NAME}/{CORRUPT_DIR_NAME}")

    def test_filename_index_disagrees_with_header_isolated(self) -> None:
        payload = make_payload("indexswitch")
        header = make_header(trace_index=11, payload=payload)
        path = self._write_raw("00012" + FRAME_SUFFIX, frame_bytes(header, payload))
        self.assertEqual(self.outbox.scan_pending(), [])
        self.assertFalse(path.exists())

    def test_garbage_json_isolated(self) -> None:
        body = struct.pack("!I", 5) + b"{not " + struct.pack("!Q", 0)
        path = self._write_raw("000002" + FRAME_SUFFIX, body)
        self.assertEqual(self.outbox.scan_pending(), [])
        self.assertFalse(path.exists())
        self.assertEqual(self.outbox.statistics()["corrupt_files"], 1)

    def test_sha_mismatch_isolated(self) -> None:
        payload = make_payload("shamismatch")
        header = make_header(trace_index=3, payload=payload)
        header["sha256"] = "0" * 64
        path = self._write_raw("000003" + FRAME_SUFFIX, frame_bytes(header, payload))
        self.assertEqual(self.outbox.scan_pending(), [])
        self.assertFalse(path.exists())
        report = json.dumps(self.outbox.damage_report, ensure_ascii=False)
        self.assertIn("sha256", report)

    def test_missing_identity_isolated(self) -> None:
        payload = make_payload("noid")
        header = make_header(trace_index=4, payload=payload)
        header.pop("run_id")
        path = self._write_raw("000004" + FRAME_SUFFIX, frame_bytes(header, payload))
        self.assertEqual(self.outbox.scan_pending(), [])
        self.assertFalse(path.exists())

    def test_payload_bytes_mismatch_isolated(self) -> None:
        payload = make_payload("pblen")
        header = make_header(trace_index=5, payload=payload)
        header["payload_bytes"] = len(payload) + 7
        path = self._write_raw("000005" + FRAME_SUFFIX, frame_bytes(header, payload))
        self.assertEqual(self.outbox.scan_pending(), [])
        self.assertFalse(path.exists())

    def test_dir_name_vs_header_run_id_mismatch_isolated(self) -> None:
        payload = make_payload("dirswitch")
        header = make_header(run_id=RUN_B, trace_index=1, payload=payload)
        path = self._write_raw("000001" + FRAME_SUFFIX, frame_bytes(header, payload), run=RUN_A)
        self.assertEqual(self.outbox.scan_pending(), [])
        self.assertFalse(path.exists())

    def test_non_numeric_filename_isolated(self) -> None:
        payload = make_payload("badname")
        header = make_header(trace_index=1, payload=payload)
        path = self._write_raw("notanumber" + FRAME_SUFFIX, frame_bytes(header, payload))
        self.assertEqual(self.outbox.scan_pending(), [])
        self.assertFalse(path.exists())

    def test_tmp_orphan_isolated_not_sent(self) -> None:
        # 崩溃遗留 .tmp（原子写未完成）：按未完成处理，隔离且不进重放。
        payload = make_payload("tmporphan")
        real = self._write_raw(
            "000009" + FRAME_SUFFIX + ".tmp",
            frame_bytes(make_header(trace_index=9, payload=payload), payload),
        )
        self.assertEqual(self.outbox.scan_pending(), [])
        self.assertFalse(real.exists())
        self.assertEqual(self.outbox.statistics()["tmp_orphans"], 1)

    def test_corrupted_files_never_resend_and_are_not_deleted(self) -> None:
        payload = make_payload("keep")
        frame = frame_bytes(make_header(trace_index=1, payload=payload), payload)
        self._write_raw("000001" + FRAME_SUFFIX, frame[:-5])
        self.outbox.scan_pending()
        archived = list(self.outbox.corrupt_dir.iterdir())
        self.assertEqual(len(archived), 1)
        self.assertGreater(archived[0].stat().st_size, 0, "取证文件不得清空")
        # 反复扫描不得再次隔离同一文件（否则计数膨胀、取证被搬来搬去）。
        before = self.outbox.statistics()["corrupt_files"]
        self.outbox.scan_pending()
        self.outbox.scan_pending()
        self.assertEqual(self.outbox.statistics()["corrupt_files"], before, "隔离目录必须被跳过（不反复隔离）")

    def test_dry_scan_does_not_mutate_disk(self) -> None:
        payload = make_payload("dry")
        self._write_raw("000003" + FRAME_SUFFIX, frame_bytes(make_header(trace_index=3, payload=payload), payload)[:-9])
        self.assertEqual(self.outbox.scan_pending(isolate_damage=False), [])
        self.assertTrue((self.root / OUTBOX_DIR_NAME / RUN_A / ("000003" + FRAME_SUFFIX)).exists())
        self.assertEqual(self.outbox.statistics()["corrupt_files"], 0)

    def test_healthy_entries_survive_scan_of_mixed_dir(self) -> None:
        good = make_payload("good")
        self._write_raw("000020" + FRAME_SUFFIX, frame_bytes(make_header(trace_index=20, payload=good), good))
        bad = make_payload("bad")
        self._write_raw("000021" + FRAME_SUFFIX, frame_bytes(make_header(trace_index=21, payload=bad), bad)[:-4])
        entries = self.outbox.scan_pending()
        self.assertEqual([(e.run_id, e.trace_index) for e in entries], [(RUN_A, 20)])


class StatisticsTests(TempOutboxCase):
    """R-B-02：计数可观测 + JSON 可序列化。"""

    def test_statistics_json_serializable_and_accurate(self) -> None:
        total = 0
        for index in range(1, 4):
            payload = make_payload(f"s{index}")
            self.outbox.append_message(make_header(trace_index=index, payload=payload), payload)
            total += len(frame_bytes(make_header(trace_index=index, payload=payload), payload))
        stats = self.outbox.statistics()
        self.assertEqual(stats["pending_files"], 3)
        self.assertEqual(stats["pending_bytes"], total)
        self.assertEqual(stats["appended"], 3)
        json.dumps(stats, ensure_ascii=False)  # 不得抛
        self.outbox.note_replayed(duplicate_acked=True)
        after = self.outbox.statistics()
        self.assertEqual(after["replayed"], 1)
        self.assertEqual(after["replay_duplicate_acked"], 1)

    def test_counters_are_per_instance(self) -> None:
        # 类级可变默认会造成跨实例污染（第1轮复审同类暗病）。
        box_a = DiskOutbox(self.root / "a")
        box_b = DiskOutbox(self.root / "b")
        payload = make_payload("isolate")
        box_a.append_message(make_header(trace_index=1, payload=payload), payload)
        self.assertEqual(box_a.statistics()["appended"], 1)
        self.assertEqual(box_b.statistics()["appended"], 0)
        self.assertEqual(box_b.damage_report, [])

    def test_metrics_injection_failure_never_breaks_write(self) -> None:
        class _Boom:
            def record_stage(self, *args, **kwargs):
                raise RuntimeError("metrics exploded")

        box = DiskOutbox(self.root / "m", metrics=_Boom())
        payload = make_payload("metric")
        path = box.append_message(make_header(trace_index=1, payload=payload), payload)
        self.assertTrue(path.is_file(), "统计故障绝不影响落盘主链路")


class ReplayIdentityTests(TempOutboxCase):
    """R-B-04：重放不引入新身份——字节级原样。"""

    def test_replayed_frame_bytes_equal_original_file(self) -> None:
        payload = make_payload("replay")
        header = make_header(trace_index=8, payload=payload)
        path = self.outbox.append_message(header, payload)
        original = path.read_bytes()
        entry = self.outbox.scan_pending()[0]
        self.assertEqual(entry.frame, original)
        self.assertEqual(frame_bytes(entry.header, entry.payload), original, "重放字节必须可零改写复用")
        self.assertEqual(entry.header["run_id"], RUN_A)
        self.assertEqual(entry.header["trace_index"], 8)


class _ReceiverHarness(unittest.TestCase):
    """真实 ReceiverThread + port=0（零 mock；不碰 WebEngine）。"""

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory(prefix="round2-outbox-ground-")
        self.addCleanup(self._tmp.cleanup)
        self.ground_dir = Path(self._tmp.name)
        self.receiver = ground.ReceiverThread("127.0.0.1", 0, self.ground_dir)
        self.receiver.start()
        self.addCleanup(self._shutdown_receiver)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not self.receiver.bound_port:
            time.sleep(0.02)
        self.port = int(self.receiver.bound_port or 0)
        self.assertTrue(self.port, "ReceiverThread 未绑定端口")

    def _shutdown_receiver(self) -> None:
        self.receiver.stop()
        self.receiver.wait(3000)

    def connect(self, run_id: str) -> socket.socket:
        sock = socket.create_connection(("127.0.0.1", self.port), timeout=5)
        sock.settimeout(10.0)
        self.addCleanup(sock.close)  # 不留未关连接（也确保释放地面唯一客户槽）
        sock.sendall(frame_bytes({"type": "hello", "protocol": ground.PROTOCOL, "run_id": run_id, "timestamp": "t"}))
        header = self._read_frame(sock)
        self.assertEqual(header.get("type"), "ack", f"hello ack 异常：{header}")
        return sock

    def _read_frame(self, sock: socket.socket) -> dict[str, object]:
        (header_len,) = struct.unpack("!I", ground.recv_exact(sock, 4))
        header = json.loads(ground.recv_exact(sock, header_len).decode("utf-8"))
        (payload_len,) = struct.unpack("!Q", ground.recv_exact(sock, 8))
        if payload_len:
            ground.recv_exact(sock, payload_len)
        return header

    def send_raw_frame(self, sock: socket.socket, frame: bytes) -> dict[str, object]:
        sock.sendall(frame)
        return self._read_frame(sock)

    def ground_saved_files(self) -> list[str]:
        return sorted(str(p.relative_to(self.ground_dir)) for p in self.ground_dir.rglob("trace_*.npz"))

    def ground_metadata_lines(self, run_id: str) -> list[dict[str, object]]:
        path = self.ground_dir / run_id / "run_metadata.jsonl"
        if not path.exists():
            return []
        return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


class GroundAbsorptionTests(_ReceiverHarness):
    """R-B-07（地面零改动）+ R-B-03 的进程内等价：全部重发被既有去重吸收。"""

    def test_replay_absorbed_by_existing_dedup(self) -> None:
        box = DiskOutbox(self.ground_dir / "air")
        frames = []
        for index in range(1, 4):
            payload = make_payload(f"absorb-{index}")
            header = make_header(trace_index=index, payload=payload)
            path = box.append_message(header, payload)
            frames.append(path.read_bytes())
        sock = self.connect(RUN_A)
        # 第一轮：全部新保存 + ACK → 逐条划除（未 ACK 不得划除，故顺序执行）。
        for index, frame in enumerate(frames, start=1):
            ack = self.send_raw_frame(sock, frame)
            self.assertEqual(ack.get("type"), "ack_trace", f"trace {index} 未 ACK")
            self.assertNotIn("duplicate", ack)
            pending = box.scan_pending()
            self.assertEqual(len(pending), 4 - index, f"第 {index} 条 ACK 前磁盘应有 {4 - index} 条未确认")
            self.assertTrue(box.remove(pending[0]))
        self.assertEqual(box.pending_count(), 0, "全部 ACK 后 outbox 必须清空")
        self.assertEqual(len(self.ground_saved_files()), 3)
        # 第二轮：把**同样的字节**再发一遍（模拟 outbox 补传撞上去）。
        duplicates = 0
        for frame in frames:
            ack = self.send_raw_frame(sock, frame)
            self.assertEqual(ack.get("type"), "ack_trace")
            duplicates += 1 if ack.get("duplicate") else 0
        self.assertEqual(duplicates, 3, "全部重发必须被 duplicate 吸收")
        self.assertEqual(len(self.ground_saved_files()), 3, "重发不得新增磁盘文件")
        self.assertEqual(len(self.ground_metadata_lines(RUN_A)), 3, "重发不得重复写 metadata")

    def test_replay_after_ground_restart_absorbed_via_disk_sha(self) -> None:
        # 地面进程重启（内存表清空）后重放：走跨重启读盘比 sha 路径。
        payload = make_payload("restart")
        frame = frame_bytes(make_header(trace_index=1, payload=payload), payload)
        sock = self.connect(RUN_A)
        self.assertEqual(self.send_raw_frame(sock, frame).get("type"), "ack_trace")
        sock.close()
        self.receiver.stop()
        self.receiver.wait(3000)
        restarted = ground.ReceiverThread("127.0.0.1", 0, self.ground_dir)
        restarted.start()
        self.receiver = restarted
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not restarted.bound_port:
            time.sleep(0.02)
        self.port = int(restarted.bound_port)
        sock2 = self.connect(RUN_A)
        ack = self.send_raw_frame(sock2, frame)
        self.assertEqual(ack.get("type"), "ack_trace")
        self.assertTrue(ack.get("duplicate"), "跨重启同内容重放必须 duplicate ACK")
        self.assertEqual(len(self.ground_saved_files()), 1)
        self.assertEqual(len(self.ground_metadata_lines(RUN_A)), 1)

    def test_cross_run_same_index_no_collision(self) -> None:
        # 身份轮换（§3.7 方案 a）后同编号不再冲突：两个 run 各自 trace#1 都保存。
        sock = self.connect(RUN_A)
        payload_a = make_payload("xrun-a")
        self.assertEqual(self.send_raw_frame(sock, frame_bytes(make_header(run_id=RUN_A, trace_index=1, payload=payload_a), payload_a)).get("type"), "ack_trace")
        payload_b = make_payload("xrun-b")
        self.assertEqual(self.send_raw_frame(sock, frame_bytes(make_header(run_id=RUN_B, trace_index=1, payload=payload_b), payload_b)).get("type"), "ack_trace")
        self.assertEqual(len(self.ground_saved_files()), 2)


_CRASH_WORKER = '''
"""子进程：从磁盘 outbox 扫描未确认帧并重放到地面（R-B-03 崩溃恢复的"新进程"）。

逐条打印进度并 flush —— 硬杀（os._exit）后父进程仍能读到已确认条数。
"""
import json, os, socket, struct, sys
sys.path.insert(0, {root!r})
from disk_outbox import DiskOutbox

outbox_dir, port, run_id, mode, kill_after = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4], int(sys.argv[5])


def log(obj):
    sys.stdout.write(json.dumps(obj) + "\\n")
    sys.stdout.flush()


def recv_exactly(n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise RuntimeError("connection closed")
        buf += chunk
    return buf


def read_frame():
    (n,) = struct.unpack("!I", recv_exactly(4))
    header = json.loads(recv_exactly(n).decode("utf-8"))
    (m,) = struct.unpack("!Q", recv_exactly(8))
    if m:
        recv_exactly(m)
    return header


box = DiskOutbox(outbox_dir)
entries = box.scan_pending()
log({{"event": "scanned", "count": len(entries), "pid": os.getpid()}})
sock = socket.create_connection(("127.0.0.1", port), timeout=5)
sock.settimeout(10.0)
hello = {{"type": "hello", "protocol": "uav-gpr-telemetry-v1", "run_id": run_id, "timestamp": "w"}}
hb = json.dumps(hello, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
sock.sendall(struct.pack("!I", len(hb)) + hb + struct.pack("!Q", 0))
read_frame()
acked = 0
for i, entry in enumerate(entries, 1):
    sock.sendall(entry.frame)
    ack = read_frame()
    if ack.get("type") != "ack_trace":
        log({{"event": "fatal", "reason": "no ack_trace", "ack": ack, "trace": entry.trace_index}})
        sys.exit(3)
    box.remove(entry)
    acked += 1
    log({{"event": "acked", "trace_index": entry.trace_index, "duplicate": bool(ack.get("duplicate"))}})
    if mode == "kill" and i == kill_after:
        log({{"event": "about-to-crash", "acked": acked}})
        os._exit(9)
log({{"event": "done", "scanned": len(entries), "acked": acked, "pending": box.pending_count()}})
sys.exit(0)
'''


class CrashRecoverySubprocessTests(_ReceiverHarness):
    """R-B-03：subprocess **新进程**加载同一 outbox 目录续传（同进程造对象不算）。"""

    def setUp(self) -> None:
        super().setUp()
        self.air_root = self.ground_dir / "air"
        self.box = DiskOutbox(self.air_root)

    def _worker_script(self) -> Path:
        script = self.air_root / "crash_worker.py"
        script.parent.mkdir(parents=True, exist_ok=True)
        script.write_text(_CRASH_WORKER.format(root=str(ROOT)), encoding="utf-8")
        return script

    def _spawn(self, run_id: str, *, kill: bool, kill_after: int = 10**6) -> tuple[int, dict[str, object], list[dict[str, object]]]:
        """起**新 python 进程**重放同一 outbox 目录（R-B-03 口径）。"""
        script = self._worker_script()
        proc = subprocess.run(
            [
                sys.executable,
                str(script),
                str(self.air_root),
                str(self.port),
                run_id,
                "kill" if kill else "clean",
                str(kill_after),
            ],
            capture_output=True,
            text=True,
            timeout=120,
            cwd=str(ROOT),
        )
        events: list[dict[str, object]] = []
        for line in (proc.stdout or "").splitlines():
            line = line.strip()
            if line.startswith("{"):
                events.append(json.loads(line))
        last = events[-1] if events else {"event": "no-output", "stderr": (proc.stderr or "")[-500:]}
        return proc.returncode, last, events

    def test_kill_midway_then_new_process_resumes(self) -> None:
        total = 8
        kill_after = 3
        for index in range(1, total + 1):
            payload = make_payload(f"crash-{index}")
            self.box.append_message(make_header(trace_index=index, payload=payload), payload)
        self.assertEqual(self.box.pending_count(), total)

        # 进程 1：发到第 kill_after 条时 os._exit(9) 硬杀（模拟崩溃/断电）。
        code, _last, events = self._spawn(RUN_A, kill=True, kill_after=kill_after)
        self.assertEqual(code, 9, f"子进程未按预期硬杀：{events}")
        pid1 = next(ev["pid"] for ev in events if ev.get("event") == "scanned")
        self.assertNotEqual(pid1, os.getpid(), "重放必须在**新进程**里发生（R-B-03：同进程造对象不算）")
        acked_events = [ev for ev in events if ev.get("event") == "acked"]
        self.assertEqual(len(acked_events), kill_after)
        self.assertEqual([ev["trace_index"] for ev in acked_events], [1, 2, 3])
        survivors = self.box.scan_pending()
        self.assertEqual(
            [e.trace_index for e in survivors],
            list(range(kill_after + 1, total + 1)),
            "已 ACK 的必须已划除，未 ACK 的必须仍在磁盘",
        )

        # 进程 2（全新解释器、全新对象）：扫描同一目录续传。
        code2, report2, events2 = self._spawn(RUN_A, kill=False)
        self.assertEqual(code2, 0, f"续传进程失败：{report2}")
        pid2 = next(ev["pid"] for ev in events2 if ev.get("event") == "scanned")
        # 两个重放者都是独立进程（各自 != 测试进程，且各自独立 exit 码可观测）。
        # 不断言 pid1 != pid2：Windows 允许 PID 复用，那样会变成偶发失败；
        # "新进程实例"由 subprocess 起停 + 各自退出码 9/0 结构性保证。
        self.assertNotEqual(pid2, os.getpid(), "续传也必须在独立进程里发生")
        self.assertEqual(report2.get("scanned"), total - kill_after)
        self.assertEqual(report2.get("acked"), total - kill_after)
        self.assertEqual(report2.get("pending"), 0)
        self.assertEqual(self.box.pending_count(), 0)
        # 续传的 5 条地面从未收过 ⇒ 全部是新保存，不该有 duplicate。
        self.assertEqual([ev.get("duplicate") for ev in events2 if ev.get("event") == "acked"], [False] * 5)

        # 端到端对账：地面最终收到的 trace 数 == 选中的条数，且无脏数据。
        saved = self.ground_saved_files()
        self.assertEqual(len(saved), total, f"地面落盘数对账失败：{saved}")
        metadata = self.ground_metadata_lines(RUN_A)
        self.assertEqual(len(metadata), total, "metadata 行数必须等于条数（无重复行）")
        self.assertNotIn(
            True,
            [str(name).endswith("trace_00000.npz") for name in saved],
            f"不得出现 trace_00000 类脏文件：{saved}",
        )
        for index in range(1, total + 1):
            expected = self.air_root / OUTBOX_DIR_NAME / RUN_A / f"{index:06d}{FRAME_SUFFIX}"
            self.assertFalse(expected.exists())

    def test_replayed_bytes_never_recreate_conflicts(self) -> None:
        # 补传撞地面已有数据 ⇒ duplicate 吸收，不产生 §3.7 冲突拒绝（无 ACK）。
        for index in range(1, 4):
            payload = make_payload(f"conflict-{index}")
            self.box.append_message(make_header(trace_index=index, payload=payload), payload)
        frames = [entry.frame for entry in self.box.scan_pending()]
        sock = self.connect(RUN_A)
        for frame in frames:
            self.assertEqual(self.send_raw_frame(sock, frame).get("type"), "ack_trace")
        for entry in self.box.scan_pending():
            self.box.remove(entry)
        # 关键：**必须释放地面唯一客户槽**再起子进程——地面 accept 串行
        # （§3.7 读码事实），旧连接不退会让补传连接的 hello 得不到 ack。
        sock.close()
        # 把原始字节再塞回 outbox（模拟"崩溃发生在划除之前"）。
        for index, frame in enumerate(frames, start=1):
            self.box.append(frame, RUN_A, index)
        code, report, events = self._spawn(RUN_A, kill=False)
        self.assertEqual(code, 0, f"重放被地面拒绝（死锁复活）：{report}")
        self.assertEqual(report.get("acked"), 3)
        self.assertEqual(
            [ev.get("duplicate") for ev in events if ev.get("event") == "acked"],
            [True, True, True],
            "重放的 3 条必须全部被判 duplicate（吸收，而非新存/冲突）",
        )
        self.assertEqual(len(self.ground_saved_files()), 3, "重放不得新增文件")
        self.assertEqual(len(self.ground_metadata_lines(RUN_A)), 3, "重放不得重复 metadata")

    def test_two_processes_serialized_no_duplicate_send(self) -> None:
        # 顺序起两个进程（等价于"崩溃后重启"）：第二个扫到 0 条，不做无谓重发。
        for index in range(1, 3):
            payload = make_payload(f"serial-{index}")
            self.box.append_message(make_header(trace_index=index, payload=payload), payload)
        code, report, _events = self._spawn(RUN_A, kill=False)
        self.assertEqual(code, 0, report)
        self.assertEqual(report.get("scanned"), 2)
        code2, report2, _events2 = self._spawn(RUN_A, kill=False)
        self.assertEqual(code2, 0, report2)
        self.assertEqual(report2.get("scanned"), 0, "划除即持久化确认：无残留可重发")


class WriteCostTests(unittest.TestCase):
    """R-B-08 专项计时：写盘开销必须是**真实测量数字**并有上界断言，不口头声明。

    用真实生产 payload 生成器（``air.trace_npz_bytes``，201 频点 / 2048 时域点
    双通道，≈75KB NPZ）度量 append/remove/scan。断言的是**保守上界**（远小于
    第1轮 25Hz 的 40ms/条预算），真机磁盘更慢时测试会红——这正是本条要防的
    "性能数字造假/口头声明"。实测数字同时打进 stdout 供 t9 终验转抄。
    """

    N_FREQ = 201
    N_TIME = 2048
    SAMPLES = 40

    def _real_payload(self, seed: int) -> bytes:
        import air_core as air
        import numpy as np

        cfg = air.SweepConfig(
            start_mhz=300.0, stop_mhz=3000.0, points=self.N_FREQ, ifft_points=self.N_TIME
        )
        freq = np.linspace(300e6, 3000e6, self.N_FREQ)
        rng = np.random.default_rng(seed)
        raw = {ch: rng.normal(0, 0.2, self.N_FREQ) + 1j * rng.normal(0, 0.2, self.N_FREQ) for ch in ("S11", "S22")}
        disp = {ch: rng.normal(0, 0.1, self.N_FREQ) + 1j * rng.normal(0, 0.1, self.N_FREQ) for ch in ("S11", "S22")}
        traces = {ch: rng.normal(0, 1.0, self.N_TIME) for ch in ("S11", "S22")}
        time_ns = np.linspace(0.0, 80.0, self.N_TIME)
        acq = {"acquired_at_utc": "2026-09-06T10:00:00+00:00", "saved_at_utc": "2026-09-06T10:00:01+00:00"}
        return air.trace_npz_bytes(
            seed, cfg, freq, raw, disp, time_ns, traces, air.GgaFix(),
            channels=("S11", "S22"), acquisition=acq, flight={"gnss_valid": False},
        )

    def test_append_remove_scan_cost_within_budget(self) -> None:
        from pipeline_metrics import PipelineMetrics

        tmp = tempfile.TemporaryDirectory(prefix="round2-outbox-cost-")
        self.addCleanup(tmp.cleanup)
        metrics = PipelineMetrics()
        box = DiskOutbox(Path(tmp.name), metrics=metrics)

        payloads = [self._real_payload(i) for i in range(1, self.SAMPLES + 1)]
        self.assertGreater(len(payloads[0]), 10_000, "payload 必须是真实量级（否则计时无意义）")

        # 逐条测 append（含 fsync + os.replace）：每条新 trace_index，绝不走
        # 幂等重写快路径（那会低估成本）。
        append_ms: list[float] = []
        for index, payload in enumerate(payloads, start=1):
            header = make_header(run_id=RUN_A, trace_index=index, payload=payload)
            frame = frame_bytes(header, payload)
            t0 = time.perf_counter()
            box.append(frame, RUN_A, index)
            append_ms.append((time.perf_counter() - t0) * 1000.0)

        t_scan = time.perf_counter()
        rescanned = box.scan_pending()
        scan_ms = (time.perf_counter() - t_scan) * 1000.0
        self.assertEqual(len(rescanned), self.SAMPLES)

        remove_ms = []
        for entry in rescanned:
            t0 = time.perf_counter()
            self.assertTrue(box.remove(entry))
            remove_ms.append((time.perf_counter() - t0) * 1000.0)

        append_sorted = sorted(append_ms)
        p50 = append_sorted[len(append_sorted) // 2]
        p95 = append_sorted[max(0, int(len(append_sorted) * 0.95) - 1)]
        print(
            f"\n[outbox R-B-08 专项计时] payload={len(payloads[0])}B n={self.SAMPLES} "
            f"append p50={p50:.3f}ms p95={p95:.3f}ms max={append_sorted[-1]:.3f}ms | "
            f"remove p50={sorted(remove_ms)[len(remove_ms)//2]:.3f}ms | "
            f"scan({self.SAMPLES}条全量校验)={scan_ms:.2f}ms ({scan_ms / self.SAMPLES * 1000:.0f}us/条)"
        )
        # 保守上界：单条写盘（含 fsync）必须远小于 25Hz 的 40ms 预算。
        self.assertLess(p95, 15.0, f"单条 outbox 写盘 p95={p95:.2f}ms 超出 15ms 上界（40ms 预算的 37.5%）")
        self.assertLess(append_sorted[-1], 40.0, "单次写盘不得吃掉一整条 25Hz 预算")
        self.assertLess(scan_ms / self.SAMPLES * 1000, 5_000, "扫描单条成本上界 5ms/条（重启恢复不得卡顿）")
        # 阶段必须真实进 metrics（R-B-08 要求 outbox_write 可在全链路报告出现）。
        snapshot = metrics.snapshot()
        stages = json.dumps(snapshot, ensure_ascii=False, default=str)
        self.assertIn("outbox_write", stages, f"outbox_write 阶段未进 metrics：{stages[:300]}")

    def test_metrics_stage_recorded_even_when_write_fails(self) -> None:
        from pipeline_metrics import PipelineMetrics

        tmp = tempfile.TemporaryDirectory(prefix="round2-outbox-cost2-")
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name)
        metrics = PipelineMetrics()
        box = DiskOutbox(root, metrics=metrics)
        (root / OUTBOX_DIR_NAME).write_text("not a dir", encoding="utf-8")
        payload = make_payload("metricfail")
        with self.assertRaises(OutboxWriteError):
            box.append_message(make_header(trace_index=1, payload=payload), payload)
        self.assertIn("outbox_write", json.dumps(metrics.snapshot(), ensure_ascii=False, default=str))


# =====================================================================
# RemoteSender 接线段（放行后追加）：R-B-01/02/05/06/09/10 + §4.4/§4.5
# =====================================================================

import numpy as np  # noqa: E402
import air_core as air  # noqa: E402

TASK_RUN = "20260906_101112_0001"
IDLE_RUN = "20260906_101112_idle"


def _trace_args(seed: int = 7) -> dict[str, object]:
    """enqueue_trace 的最小真实参数组（形状沿用第1轮测试）。"""
    rng = np.random.default_rng(seed)
    return dict(
        cfg=air.SweepConfig(start_mhz=400.0, stop_mhz=4000.0, points=5, ifft_points=8),
        frequency_hz=np.linspace(400e6, 4000e6, 5),
        raw={"S11": rng.standard_normal(5) + 1j * rng.standard_normal(5),
             "S22": rng.standard_normal(5) + 1j * rng.standard_normal(5)},
        time_ns=np.linspace(0, 8, 16),
        traces={"S11": rng.standard_normal(16), "S22": rng.standard_normal(16)},
    )


class _WiringHarness(unittest.TestCase):
    """真实 ReceiverThread + 真实 RemoteSender（零 mock，port=0，不建 Qt 控件）。"""

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory(prefix="round2-wire-")
        self.addCleanup(self._tmp.cleanup)
        base = Path(self._tmp.name)
        self.ground_dir = base / "ground"
        self.air_dir = base / "air"
        self.ground_dir.mkdir()
        self.air_dir.mkdir()
        self.receiver = ground.ReceiverThread("127.0.0.1", 0, self.ground_dir)
        self.receiver.start()
        self.addCleanup(self._stop_receiver)
        if not self._wait(lambda: self.receiver.bound_port):
            self.fail("ReceiverThread 未绑定端口")
        self.port = int(self.receiver.bound_port)
        self.statuses: list[str] = []

    def _stop_receiver(self) -> None:
        self.receiver.stop()
        self.receiver.wait(3000)

    @staticmethod
    def _wait(predicate, timeout: float = 20.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        return bool(predicate())

    def sender(self, *, outbox: Path | None, port: int | None = None) -> "air.RemoteSender":
        instance = air.RemoteSender(
            "127.0.0.1", port if port is not None else self.port,
            IDLE_RUN, self.statuses.append, None, outbox_dir=outbox,
        )
        self.addCleanup(self._quiet_stop, instance)
        return instance

    @staticmethod
    def _quiet_stop(instance: "air.RemoteSender") -> None:
        try:
            instance.stop(1000, drain=False)
        except Exception:  # noqa: BLE001
            pass

    def push(self, instance: "air.RemoteSender", index: int, run_id: str, *, seed: int = 7) -> None:
        args = _trace_args(seed)
        instance.enqueue_trace(
            index, args["cfg"], args["frequency_hz"], args["raw"], dict(args["raw"]),
            args["time_ns"], args["traces"], air.GgaFix(),
            channels=("S11", "S22"), run_id=run_id,
        )

    def push_legacy(self, instance: "air.RemoteSender", index: int, *, seed: int = 7) -> None:
        """第1轮 legacy 语义：**不显式传 run_id**（身份取连接身份）——这正是要测的点。

        ``channels`` 仍显式传双通道：legacy 生产调用走 ``S_PARAMETERS`` 四通道
        默认，而本测试只构造 S11/S22 数据；通道数与 skip 判据无关，不能因为
        校验器缺通道而失败。
        """
        args = _trace_args(seed)
        instance.enqueue_trace(
            index, args["cfg"], args["frequency_hz"], args["raw"], dict(args["raw"]),
            args["time_ns"], args["traces"], air.GgaFix(), channels=("S11", "S22"),
        )

    def ground_npz(self, run_id: str) -> list[str]:
        target = self.ground_dir / run_id
        return sorted(p.name for p in target.glob("trace_*.npz")) if target.exists() else []

    def ground_metadata_lines(self, run_id: str) -> int:
        path = self.ground_dir / run_id / "run_metadata.jsonl"
        return len(path.read_text(encoding="utf-8").splitlines()) if path.exists() else 0


class SenderDisabledTests(_WiringHarness):
    """R-B-10：outbox_dir=None 必须零副作用，第1轮 sender 逐字兼容。"""

    def test_no_directory_created_and_no_frames(self) -> None:
        args = _trace_args()
        instance = self.sender(outbox=None)
        instance.enqueue_trace(
            1, args["cfg"], args["frequency_hz"], args["raw"], dict(args["raw"]),
            args["time_ns"], args["traces"], air.GgaFix(), channels=("S11", "S22"),
            run_id=TASK_RUN,
        )
        self.assertIsNone(instance.outbox, "None ⇒ 不建 DiskOutbox 实例")
        self.assertEqual(list(self.air_dir.rglob("*")), [], "禁用时不得产生任何目录/文件")
        header, _payload = instance.queue.get_nowait()
        self.assertEqual(header["run_id"], TASK_RUN)
        self.assertEqual(instance.outstanding_count(), 1)

    def test_constructing_enabled_sender_creates_no_directory(self) -> None:
        # MainWindow 默认输出目录在 cwd/gpr_runs；构造期 mkdir 会留下垃圾目录。
        instance = self.sender(outbox=self.air_dir)
        self.assertIsNotNone(instance.outbox)
        self.assertFalse((self.air_dir / OUTBOX_DIR_NAME).exists(), "目录必须惰性创建（首次 append 时）")


class SenderPersistTests(_WiringHarness):
    """R-B-01/09/§4.5：落盘前置、字节一致、framing 不变、legacy 与状态消息 skip。"""

    def test_frame_written_before_enqueue_and_byte_identical(self) -> None:
        instance = self.sender(outbox=self.air_dir)
        self.push(instance, 5, TASK_RUN)
        stats = instance.outbox.statistics()
        self.assertEqual(stats["pending_files"], 1, "queue.put 之前必须已落盘")
        self.assertEqual(stats["write_failures"], 0)
        item = instance.queue.get()
        frame_on_disk = (self.air_dir / OUTBOX_DIR_NAME / TASK_RUN / f"000005{FRAME_SUFFIX}").read_bytes()
        self.assertEqual(frame_on_disk, frame_bytes(item.header, item.payload))
        self.assertEqual(item.outbox_path.name, f"000005{FRAME_SUFFIX}")
        self.assertFalse(item.replay)

    def test_header_identical_with_and_without_outbox(self) -> None:
        """R-B-09：启用 outbox 不得增删改任何 framing 字段。"""
        off = self.sender(outbox=None)
        on = self.sender(outbox=self.air_dir)
        self.push(off, 2, TASK_RUN, seed=21)
        self.push(on, 2, TASK_RUN, seed=21)
        header_off, payload_off = off.queue.get()
        item_on = on.queue.get()
        # timestamp 是每条独立生成的墙上时间（两次调用必然不同），非 framing 差异。
        for header in (header_off, item_on.header):
            header.pop("timestamp", None)
        self.assertEqual(header_off, item_on.header)
        self.assertEqual(payload_off, item_on.payload)
        for key in ("type", "protocol", "run_id", "trace_index", "sha256", "payload_bytes"):
            self.assertIn(key, item_on.header)

    def test_status_messages_never_persisted(self) -> None:
        instance = self.sender(outbox=self.air_dir)
        instance.enqueue_status("acquiring", run_id=TASK_RUN)
        instance.enqueue_status("stopped", drain_state="drained", run_id=TASK_RUN)
        self.assertEqual(instance.outbox.statistics()["pending_files"], 0)
        self.assertFalse((self.air_dir / OUTBOX_DIR_NAME).exists(), "状态消息不得触发建目录")
        self.assertEqual(instance.outstanding_count(), 2, "状态消息仍走第1轮 outstanding 语义")

    def test_legacy_path_skipped_without_failure_or_alert(self) -> None:
        """§4.5：legacy（不显式传 run_id）⇒ 契约 skip，不计失败、不告警。"""
        instance = self.sender(outbox=self.air_dir)
        self.push_legacy(instance, 1)
        self.assertEqual(instance.outbox_skipped, 1, "skip 必须单独可观测")
        stats = instance.outbox.statistics()
        self.assertEqual(stats["pending_files"], 0)
        self.assertEqual(stats["write_failures"], 0, "skip 不得计成写盘失败")
        self.assertEqual(stats["appended"], 0)
        self.assertFalse(any("outbox_alert" in text for text in self.statuses))
        self.assertFalse(any("写盘失败" in text for text in self.statuses))
        header, _payload = instance.queue.get()
        self.assertEqual(header["run_id"], IDLE_RUN, "第1轮 legacy 行为不变")

    def test_legacy_trace_still_skipped_after_identity_switch(self) -> None:
        """判据必须是"调用方是否显式传身份"，不是字符串格式。

        带内切换后连接身份已是 air_task 式；此时 legacy 条目的 header run_id
        形式上合法，但它属于**别的任务的 run**——落盘会与旧 run 同编号真数据
        撞车（正是 §3.7 死锁面）。
        """
        instance = self.sender(outbox=self.air_dir)
        instance.run_id = TASK_RUN  # 模拟 _switch_connection_identity 之后的状态
        self.push_legacy(instance, 1)
        self.assertEqual(instance.outbox_skipped, 1)
        self.assertEqual(instance.outbox.statistics()["pending_files"], 0)
        self.assertEqual(instance.outbox.statistics()["write_failures"], 0)

    def test_unparseable_identity_skipped_and_logged(self) -> None:
        instance = self.sender(outbox=self.air_dir)
        self.push(instance, 1, "20260906_101112_1")  # 裸 seq（P1 要消灭的写法）
        self.assertEqual(instance.outbox.statistics()["pending_files"], 0)
        self.assertEqual(instance.outbox.statistics()["write_failures"], 0)
        self.assertTrue(any("非采集任务身份" in text for text in self.statuses), self.statuses)


class SenderWriteFailureAlertTests(_WiringHarness):
    """R-B-06/§4.3-2：写盘失败实时链路不断、计数如实、outbox_alert 每 run 一次。"""

    def test_write_failure_alerts_once_and_still_enqueues(self) -> None:
        instance = self.sender(outbox=self.air_dir)
        (self.air_dir / OUTBOX_DIR_NAME).write_text("not a directory", encoding="utf-8")
        self.push(instance, 1, TASK_RUN)
        self.push(instance, 2, TASK_RUN)
        stats = instance.outbox.statistics()
        self.assertEqual(stats["write_failures"], 2, "两条都必须如实计数")
        self.assertEqual(stats["appended"], 0)
        self.assertTrue(any("写盘失败" in text for text in self.statuses), self.statuses)
        # 入队侧：两条 trace 仍在（实时链路不中断）+ 恰好一条 outbox_alert 状态消息。
        queued = []
        while not instance.queue.empty():
            item = instance.queue.get()
            queued.append(item.header.get("type"))
        self.assertEqual(queued.count("trace"), 2, "写盘失败不得阻断实时入队")
        self.assertEqual(
            queued.count("acquisition_status"), 1,
            f"outbox_alert 每 run 只发一条（不刷屏）：{queued}",
        )

    def test_write_failure_outstanding_matches_queued_messages(self) -> None:
        """fail-closed 的账必须平：2 trace + 1 告警 ⇒ outstanding==3。"""
        instance = self.sender(outbox=self.air_dir)
        (self.air_dir / OUTBOX_DIR_NAME).write_text("x", encoding="utf-8")
        self.push(instance, 1, TASK_RUN)
        self.assertEqual(instance.outstanding_count(), 2)
        self.assertEqual(instance.queue.qsize(), 2)

    def test_alert_message_uses_existing_framing(self) -> None:
        instance = self.sender(outbox=self.air_dir)
        (self.air_dir / OUTBOX_DIR_NAME).write_text("x", encoding="utf-8")
        instance.enqueue_status("outbox_alert", kind="write_failed", reason="disk full", run_id=TASK_RUN)
        header = instance.queue.get().header
        self.assertEqual(header["type"], "acquisition_status")
        self.assertEqual(header["state"], "outbox_alert")
        self.assertEqual(header["protocol"], ground.PROTOCOL)
        self.assertNotIn("payload_format", header, "状态消息必须保持空载荷旧形状")


class SenderEndToEndTests(_WiringHarness):
    """真实 sender→真实地面：ACK 后划除；崩溃残留由新实例续传。"""

    def test_happy_path_ack_clears_outbox(self) -> None:
        instance = self.sender(outbox=self.air_dir)
        instance.start()
        for index in (1, 2, 3):
            self.push(instance, index, TASK_RUN, seed=index)
        # 不在这里断言中间 pending 数：sender 已在跑，划除与入队天然并发
        # （"入队前已落盘"由 SenderPersistTests 的未启动用例确定性覆盖）。
        self.assertTrue(instance.stop(25000, drain=True), "三条必须全部 ACK 排空")
        stats = instance.outbox.statistics()
        self.assertEqual(stats["pending_files"], 0, "ACK 后逐条划除")
        self.assertEqual(
            stats["appended"], 3,
            "三条都必须成功落盘（appended 不足=有条目未被保护）。"
            f" stats={stats} skipped={instance.outbox_skipped} statuses={self.statuses}",
        )
        self.assertEqual(
            stats["rejections"], 0, f"happy path 不得有任何拒绝（曾把合法 run 误判越界）：{stats}"
        )
        self.assertEqual(
            stats["removed"] + stats["already_removed"], 3,
            f"三条各划除一次，账目必须闭合：{stats} statuses={self.statuses}",
        )
        self.assertEqual(instance.outstanding_count(), 0)
        self.assertEqual(self.ground_npz(TASK_RUN),
                         ["trace_00001.npz", "trace_00002.npz", "trace_00003.npz"])
        self.assertEqual(self.ground_metadata_lines(TASK_RUN), 3)

    def test_unacked_survivors_resumed_by_second_instance_end_to_end(self) -> None:
        """R-B-02/R-B-03（进程内等价）：入队未发→丢弃内存→新实例扫描续传。

        对账口径：selected(5) == ground acked(5)，无冲突拒绝、无重复文件。
        """
        first = self.sender(outbox=self.air_dir)
        for index in range(1, 6):
            self.push(first, index, TASK_RUN, seed=100 + index)
        self.assertEqual(first.outbox.statistics()["pending_files"], 5)
        first.stop(500, drain=False)  # 模拟进程被强杀（内存队列整体消失）
        self.assertEqual(first.outbox.statistics()["pending_files"], 5, "未 ACK 的必须留在磁盘")

        survivor = self.sender(outbox=self.air_dir)
        survivor.start()
        self.assertTrue(survivor.stop(30000, drain=True), "新实例必须把 5 条全部补传完")
        self.assertEqual(survivor.outbox.statistics()["pending_files"], 0)
        self.assertEqual(len(self.ground_npz(TASK_RUN)), 5)
        self.assertEqual(self.ground_metadata_lines(TASK_RUN), 5)
        self.assertEqual(survivor.outbox.statistics()["replayed"], 5)
        self.assertEqual(survivor.outbox.statistics()["replay_duplicate_acked"], 0,
                         "地面从未收过 ⇒ 全部是新保存，不该有 duplicate")

    def test_replay_absorbed_as_duplicate_over_same_connection(self) -> None:
        """§4.4：重放复用同一条连接，靠带内 hello 切身份，被地面 duplicate 吸收。"""
        live = self.sender(outbox=self.air_dir)
        live.start()
        for index in (1, 2):
            self.push(live, index, TASK_RUN, seed=200 + index)
        self.assertTrue(live.stop(25000, drain=True))
        self.assertEqual(live.outbox.statistics()["pending_files"], 0)
        # 手工把两条原始帧塞回磁盘（等价"崩溃发生在划除之前"）。
        box = DiskOutbox(self.air_dir)
        for index in (1, 2):
            args = _trace_args(seed=200 + index)
            payload = air.trace_npz_bytes(
                index, args["cfg"], args["frequency_hz"], args["raw"], dict(args["raw"]),
                args["time_ns"], args["traces"], air.GgaFix(), channels=("S11", "S22"),
            )
            header = {
                "type": "trace", "protocol": ground.PROTOCOL, "run_id": TASK_RUN,
                "trace_index": index, "timestamp": "2026-09-06T10:11:12+00:00",
                "payload_format": "npz", "payload_bytes": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(), "channels": ["S11", "S22"],
            }
            box.append_message(header, payload)
        self.assertEqual(box.pending_count(), 2)

        replay = self.sender(outbox=self.air_dir)
        self.assertEqual(replay.outbox.statistics()["pending_files"], 2)
        replay.start()
        self.assertTrue(replay.stop(25000, drain=True), "重放必须被 ACK（否则 §3.7 死锁复活）")
        stats = replay.outbox.statistics()
        self.assertEqual(stats["pending_files"], 0)
        self.assertEqual(stats["replayed"], 2)
        self.assertEqual(stats["replay_duplicate_acked"], 2, "两条都必须判 duplicate 吸收")
        self.assertEqual(len(self.ground_npz(TASK_RUN)), 2, "重放不得新增文件")
        self.assertEqual(self.ground_metadata_lines(TASK_RUN), 2, "重放不得重复 metadata")
        self.assertEqual(replay.run_id, TASK_RUN, "带内 hello 已把连接身份切到重放条目身份")


class RecoveryOutstandingTests(_WiringHarness):
    """R-B-05：恢复条目计入 outstanding；划除失败不减 outstanding。"""

    def test_scan_counted_synchronously_in_start(self) -> None:
        """扫描必须在 start() 内完成——否则其与 stop(drain) 之间有漏保护窗口。"""
        box = DiskOutbox(self.air_dir)
        for index in (1, 2, 3):
            args = _trace_args(seed=index)
            payload = air.trace_npz_bytes(
                index, args["cfg"], args["frequency_hz"], args["raw"], dict(args["raw"]),
                args["time_ns"], args["traces"], air.GgaFix(), channels=("S11", "S22"),
            )
            box.append_message(
                {"type": "trace", "protocol": ground.PROTOCOL, "run_id": TASK_RUN,
                 "trace_index": index, "timestamp": "t", "payload_format": "npz",
                 "payload_bytes": len(payload),
                 "sha256": hashlib.sha256(payload).hexdigest(), "channels": ["S11", "S22"]},
                payload,
            )
        # 指向一个 accept 但永不回包的哑监听 ⇒ sender 卡在握手，无法完成任何条目。
        dummy = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        dummy.bind(("127.0.0.1", 0))
        dummy.listen(1)
        self.addCleanup(dummy.close)
        blocked_port = dummy.getsockname()[1]

        instance = self.sender(outbox=self.air_dir, port=blocked_port)
        instance.start()
        self.assertEqual(
            instance.outstanding_count(), 3,
            "start() 返回后 outstanding 立即==恢复条数（不得后台补记）",
        )
        self.assertFalse(
            instance.stop(600, drain=True),
            "有恢复条目未 ACK 时 stop(drain=True) 必须返回 False（绝不伪装排空）",
        )
        self.assertEqual(instance.outbox.statistics()["pending_files"], 3, "排空失败不得丢数据")
        instance.stop(1000, drain=False)

    def test_removal_failure_keeps_outstanding(self) -> None:
        """ACK 已到但文件删不掉 ⇒ 持久化确认未落地，不得计完成。

        刻意在 push **之前**就装好"删除必失败"的桩：若在 start() 运行后才打桩，
        sender 线程可能已经 ACK 并划除成功，outstanding 归零——那是测试自己的
        竞态，不是被测语义（第一版就是这么写的，实测偶发红）。
        """
        instance = self.sender(outbox=self.air_dir)
        original_remove = DiskOutbox.remove
        DiskOutbox.remove = lambda self, entry: False  # type: ignore[assignment]
        try:
            instance.start()
            self.push(instance, 1, TASK_RUN, seed=31)
            self.assertFalse(instance.stop(2500, drain=True), "划除失败必须让排空失败")
            self.assertEqual(instance.outstanding_count(), 1, "划除失败不得计完成")
        finally:
            DiskOutbox.remove = original_remove  # type: ignore[assignment]
        self.assertTrue(any("划除失败" in text for text in self.statuses), self.statuses)
        self.assertEqual(instance.outbox.pending_count(), 1, "帧文件必须仍在，下次启动继续补传")
        instance.stop(2000, drain=False)
        # 恢复删除能力后，新实例可正常完成（duplicate 吸收）。
        resume = self.sender(outbox=self.air_dir)
        resume.start()
        self.assertTrue(resume.stop(25000, drain=True), "重划除应被 duplicate ACK 吸收")
        self.assertEqual(resume.outbox.pending_count(), 0)
        self.assertEqual(len(self.ground_npz(TASK_RUN)), 1, "不得产生第二份文件")


class OutboxMetricsWiringTests(_WiringHarness):
    """R-B-08 接线面：outbox_write 真实进 AIR_METRICS（未注入则 no-op）。"""

    def test_outbox_write_stage_recorded_into_air_metrics(self) -> None:
        from pipeline_metrics import PipelineMetrics

        previous = air.AIR_METRICS
        metrics = PipelineMetrics()
        air.set_air_metrics(metrics)
        self.addCleanup(air.set_air_metrics, previous)
        instance = self.sender(outbox=self.air_dir)
        self.push(instance, 1, TASK_RUN, seed=41)
        snapshot = json.dumps(metrics.snapshot(), ensure_ascii=False, default=str)
        self.assertIn("outbox_write", snapshot, "写盘耗时必须进真实阶段统计，不口头声明")

    def test_disabled_sender_records_no_outbox_stage(self) -> None:
        from pipeline_metrics import PipelineMetrics

        previous = air.AIR_METRICS
        metrics = PipelineMetrics()
        air.set_air_metrics(metrics)
        self.addCleanup(air.set_air_metrics, previous)
        instance = self.sender(outbox=None)
        self.push(instance, 1, TASK_RUN, seed=42)
        self.assertNotIn("outbox_write", json.dumps(metrics.snapshot(), ensure_ascii=False, default=str))


class PruneThenAppendTests(TempOutboxCase):
    """t5 抓到的真实竞态回归锁定：划除会 prune 空 run 目录，紧随其后的同 run
    落盘面对的正是"目录不存在"状态。

    历史缺陷：越界判据曾用 ``Path.resolve()`` 比较。目录**存在**时两侧都解析为
    长名、相等；目录刚被 prune 掉时，Windows 对不存在路径按 8.3 短名解析缺失段，
    与长名父目录**字符串不等** ⇒ 合法 run_id 被误判"逃出 outbox 根"，该条 trace
    被静默降级成未持久化（丢崩溃保护）。表现为间歇失败（实测 1/3~1/6），且只在
    "ACK 划除与下一条入队交错"时出现——正是生产中 25Hz 连续采集的常态。
    修法是判据纯词法化（sanitize 后段内不可能含分隔符，以 '.' 打头即拒绝），
    不依赖任何文件系统状态。
    """

    def test_append_after_empty_run_dir_pruned_still_succeeds(self) -> None:
        for cycle in range(1, 4):
            payload = make_payload(f"prune-cycle-{cycle}")
            path = self.outbox.append_message(
                make_header(trace_index=cycle, payload=payload), payload
            )
            self.assertTrue(path.is_file(), f"第 {cycle} 轮：落盘必须成功")
            pending = self.outbox.scan_pending()
            self.assertEqual(len(pending), 1)
            self.assertTrue(self.outbox.remove(pending[0]))
            self.assertFalse(
                path.parent.exists(),
                f"第 {cycle} 轮：空 run 目录应被清理（刻意制造下一轮的'目录不存在'态）",
            )
        stats = self.outbox.statistics()
        self.assertEqual(stats["appended"], 3, f"三轮都不得被误拒：{stats}")
        self.assertEqual(stats["write_failures"], 0, f"误拒会以写盘失败面目出现：{stats}")
        self.assertEqual(stats["rejections"], 0, f"合法帧不得被判拒绝：{stats}")
        self.assertEqual(stats["pending_files"], 0)
        self.assertEqual(stats["removed"], 3)

    def test_rejected_append_is_counted_not_silent(self) -> None:
        """R-B-02 核心：任何"未受保护"都必须计数可观测。

        历史缺陷：身份/路径校验在 try 之外抛 OutboxFrameError ⇒ appended 少 1
        而 write_failures/rejections 都是 0，"本该受保护却没保住"成了静默事件。
        """
        before = self.outbox.statistics()
        payload = make_payload("reject-me")
        # 非任务身份 ⇒ _validate_identity 拒绝
        with self.assertRaises(OutboxFrameError):
            self.outbox.append_message(
                make_header(run_id="20260906_101112_idle", trace_index=1, payload=payload), payload
            )
        # 非 trace 类型 ⇒ _validate_entry_bytes 拒绝
        with self.assertRaises(OutboxFrameError):
            self.outbox.append(
                frame_bytes(make_header(trace_index=2, payload=payload, msg_type="acquisition_status"), payload),
                RUN_A, 2,
            )
        after = self.outbox.statistics()
        self.assertEqual(after["rejections"] - before["rejections"], 2, f"两次拒绝都要入账：{after}")
        self.assertEqual(after["appended"] - before["appended"], 0)
        self.assertEqual(after["write_failures"] - before["write_failures"], 0, "拒绝不得混进磁盘故障账")

    def test_same_identity_different_content_rejected_and_counted(self) -> None:
        payload_a = make_payload("lock-a")
        path = self.outbox.append_message(make_header(trace_index=9, payload=payload_a), payload_a)
        payload_b = make_payload("lock-b")
        with self.assertRaises(OutboxFrameError):
            self.outbox.append_message(make_header(trace_index=9, payload=payload_b), payload_b)
        stats = self.outbox.statistics()
        self.assertEqual(stats["rejections"], 1, f"覆盖拒绝必须入账：{stats}")
        self.assertEqual(path.read_bytes(), frame_bytes(make_header(trace_index=9, payload=payload_a), payload_a))

    def test_hostile_names_cannot_escape(self) -> None:
        # 词法判据与文件系统状态无关：sanitize 把所有分隔符换成 "_"，唯一逃逸
        # 面是以 '.' 打头的段（".."/"..\..\x"/"." 都逃不出这个集合）⇒ 拒绝。
        for hostile in ("..", "../..", "..\\..", ".", "./x", "..\\..\\windows", "../x"):
            with self.subTest(hostile=hostile):
                with self.assertRaises(OutboxFrameError):
                    self.outbox.run_dir(hostile)
        # 其余含分隔符的名字被 sanitize 压成单个安全段（不拒绝但不逃逸）。
        for neutralized in ("/etc/passwd", "a/b/c", "a\\b", "run name"):
            with self.subTest(neutralized=neutralized):
                target = self.outbox.run_dir(neutralized)
                self.assertEqual(target.parent, self.outbox.outbox_dir)
                self.assertNotIn("/", target.name)
                self.assertNotIn("\\", target.name)
                self.assertFalse(target.name.startswith("."))


if __name__ == "__main__":
    unittest.main()
