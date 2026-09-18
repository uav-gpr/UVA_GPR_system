"""磁盘 outbox：待发遥测 trace 帧的原子落盘 + 崩溃后断点补传（第2轮 §4，R-B-01…R-B-10）。

纯 Python 模块（**不依赖 Qt**，R-B-10）：临时目录 + 本模块即可完整单测，不需要
MainWindow/WebEngine/QThread。

设计要点（与契约逐条对应）
--------------------------
* **落盘内容 = 原始 wire 帧字节**（§4.1 / R-B-01 / R-B-04）：
  ``pack("!I",len(hb)) + hb + pack("!Q",len(pb)) + pb``，与
  :func:`uav_gpr_qt.send_remote_frame` 写到 socket 的字节逐字节一致（本模块
  :func:`frame_bytes` 是该函数的镜像实现，:class:`FrameParityTests` 用真实
  socket 抓取 sendall 字节做回归锁定）。重放 = 把文件字节 ``sendall``，
  **不重新序列化 header**——这是"重放字节 == 原始线上字节"的结构化保证。
* **划除即持久化确认**（§4.2）：`.frame` 文件存在 == 未确认；删除 == 已确认。
  无独立 ledger，因此不存在"ledger 与帧文件双账不一致"这一类新故障面。
  duplicate ACK 重复划除安全（文件已不存在视为幂等成功，不算失败）。
* **原子写**（§4.3-1 / R-X-06）：``<name>.tmp`` → write → flush → ``fsync`` →
  ``os.replace``。崩溃遗留的 ``*.tmp`` 一律按未完成处理：扫描时隔离进
  ``_corrupt/``（**不删除**，保留取证）并计数，绝不作为待发条目重发。
* **fail-closed**（§4.3-3 / R-B-06）：扫描中发现 帧截断 / JSON 非法 / header 非
  dict / 缺 ``run_id``|``trace_index``|``sha256`` / header sha256 与 payload
  实算不符 / ``payload_bytes`` 与实际不符 / 身份不是采集任务式 ⇒ 移入
  ``_corrupt/`` + 计数 + 逐条记录，**绝不猜测身份重发**（猜错身份重发会在
  地面制造 §3.7 的"同 (run_id,trace_index) 不同 payload 冲突拒绝且不 ACK"
  死锁）。
* **只持久化 trace，绝不持久化状态消息**（本模块刻意的窄化，见 §4.3-1 原文
  只要求 trace）：``acquisition_status`` 是"某时刻的状态快照"，重启后重放一条
  陈旧的 ``acquiring``/``stopped`` 会污染地面当前任务的停止对账基线与
  ``_stop_commit_barrier``（第1轮复审点名的"状态失联"类暗病），且地面本就对旧
  run 的迟到 stopped 走忽略分支——重放它没有任何正向作用，只有风险。
* **扫描排序**（§4.1）：目录名 = ``run_id``（定长时间戳 + ``%04d`` seq ⇒
  字典序 == 时间序 == 数值序），文件名 = ``<trace_index:06d>.frame``；
  排序键 ``(run_id, trace_index)``，与 §4.4"恢复列表按 (run_id, trace_index)
  排序注入 pending 位"直接对齐。
* **写盘失败不静默、也不阻断实时链路**（§4.3-2）：:meth:`DiskOutbox.append`
  失败时计数后抛 :class:`OutboxWriteError`，由调用方（RemoteSender）决定"仍
  入内存队列继续发送 + 发 outbox_alert 状态消息"。本模块绝不吞异常伪装成功。

目录布局::

    <out_dir>/_outbox/
        <run_id>/<trace_index:06d>.frame      # 未确认（存在==需补发）
        <run_id>/<trace_index:06d>.frame.tmp  # 写盘途中崩溃的残留（扫描时隔离）
        _corrupt/<隔离文件>                    # 损坏/身份非法（取证保留）
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path

#: outbox 根目录名（与 run 数据目录隔离；run 目录名恒以数字开头，"_outbox" 永不撞车）。
OUTBOX_DIR_NAME = "_outbox"
#: 损坏/非法身份文件的隔离目录（保留取证，不删除）。
CORRUPT_DIR_NAME = "_corrupt"
#: 未确认帧文件后缀。存在 == 尚未收到匹配 ack_trace。
FRAME_SUFFIX = ".frame"
#: 原子写的临时后缀（崩溃遗留一律按未完成处理）。
TMP_SUFFIX = ".tmp"

#: 采集任务身份（§3.7-1：定长时间戳 + 四位零填充 seq ⇒ 字典序==时间序）。
AIR_TASK_RUN_ID_RE = re.compile(r"^\d{8}_\d{6}_\d{4}$")
#: 校准任务身份（本轮不上线，§5.4）。
CALIBRATION_RUN_ID_RE = re.compile(r"^\d{8}_\d{6}_\d{4}_c$")
#: 待机通道身份（永不携带 trace）。
IDLE_RUN_ID_RE = re.compile(r"^\d{8}_\d{6}_idle$")

#: 地面身份词汇表的四类取值（§3.7-9）。本模块只用它筛掉"永不可能出现在 outbox
#: 里重发"的身份；地面侧的同一判据由 t7 在 uav_gpr_ground_station.py 落地。
RUN_ID_CLASS_AIR_TASK = "air_task"
RUN_ID_CLASS_CALIBRATION = "calibration"
RUN_ID_CLASS_IDLE = "idle"
RUN_ID_CLASS_LOCAL = "local"


def classify_run_id(run_id: object) -> str:
    """run_id → 身份类别（§3.7-9 词汇表的可复用实现）。

    不匹配空中三式的一律归 ``local``（地面本地占位 ``start-<epoch>`` 等），
    调用方据此判"非空中签发身份"。
    """
    text = run_id if isinstance(run_id, str) else ""
    if AIR_TASK_RUN_ID_RE.match(text):
        return RUN_ID_CLASS_AIR_TASK
    if CALIBRATION_RUN_ID_RE.match(text):
        return RUN_ID_CLASS_CALIBRATION
    if IDLE_RUN_ID_RE.match(text):
        return RUN_ID_CLASS_IDLE
    return RUN_ID_CLASS_LOCAL


def sanitize_run_dir_name(run_id: object) -> str:
    """run_id → 安全目录名。

    与地面端 ``safe_name``（uav_gpr_ground_station.py:475）**同规则**：非
    字母数字且不在 ``-_."`` 内的字符替换为 ``_``。此处**刻意不 import 地面
    模块**（那是 Qt 依赖模块，且 t5 文件域禁止触碰），规则自带并由
    :class:`SanitizeParityTests` 逐字符锁定等价，防止两边漂移。
    """
    text = str(run_id or "run")
    return "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in text)


class OutboxError(Exception):
    """本模块的异常基类。"""


class OutboxWriteError(OutboxError):
    """原子写盘失败（磁盘满/权限/IO 错误）。

    抛出即代表**该条未受 outbox 保护**；调用方必须显式上报（计数 +
    ``outbox_alert`` 状态消息），绝不静默（§4.3-2）。
    """


class OutboxFrameError(OutboxError):
    """帧损坏或身份/校验非法（fail-closed，隔离不重发）。"""


def frame_bytes(header: dict[str, object], payload: bytes = b"") -> bytes:
    """把 (header, payload) 定型为**线上帧字节**。

    必须是 :func:`uav_gpr_qt.send_remote_frame` 的逐字节镜像：同一
    ``json.dumps(ensure_ascii=False, separators=(",", ":"))``、同一
    ``!I``/``!Q`` 大端长度前缀。任何一侧改动都要同步另一侧
    （:class:`FrameParityTests` 用真实 socket 抓取字节做回归）。
    """
    header_bytes = json.dumps(header, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    return struct.pack("!I", len(header_bytes)) + header_bytes + struct.pack("!Q", len(payload)) + payload


def parse_frame(raw: bytes) -> tuple[dict[str, object], bytes]:
    """从帧字节解出 (header, payload)；结构非法即抛 :class:`OutboxFrameError`。"""
    if len(raw) < 4:
        raise OutboxFrameError(f"帧头不完整（{len(raw)} 字节）")
    (header_len,) = struct.unpack("!I", raw[:4])
    if header_len <= 0 or 4 + header_len > len(raw):
        raise OutboxFrameError(f"header 长度越界：{header_len} / 文件 {len(raw)} 字节")
    try:
        header = json.loads(raw[4 : 4 + header_len].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise OutboxFrameError(f"header JSON 非法：{exc}") from exc
    if not isinstance(header, dict):
        raise OutboxFrameError(f"header 不是 JSON 对象：{type(header).__name__}")
    tail = 4 + header_len
    if tail + 8 > len(raw):
        raise OutboxFrameError("payload 长度前缀不完整")
    (payload_len,) = struct.unpack("!Q", raw[tail : tail + 8])
    if tail + 8 + payload_len != len(raw):
        raise OutboxFrameError(
            f"帧长不符：声明 payload {payload_len} 字节，实际剩余 {len(raw) - tail - 8} 字节（疑似截断）"
        )
    return header, raw[tail + 8 : tail + 8 + payload_len]


@dataclass(frozen=True)
class OutboxEntry:
    """一条未确认的待发帧（重放所需的全部信息，身份与字节同源）。"""

    path: Path
    run_id: str
    trace_index: int
    sha256: str
    frame: bytes
    header: dict[str, object]
    payload: bytes

    @property
    def size_bytes(self) -> int:
        return len(self.frame)

    @property
    def is_trace(self) -> bool:
        return self.header.get("type") == "trace"

    @property
    def sort_key(self) -> tuple[str, int]:
        return (self.run_id, self.trace_index)


def _positive_int(value: object) -> int | None:
    """严格正整数校验（与地面 ``_accept_trace`` 同口径：bool 必须先排除）。"""
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value if value > 0 else None


def _valid_sha(value: object) -> str | None:
    text = value if isinstance(value, str) else ""
    if len(text) != 64:
        return None
    return text if all(ch in "0123456789abcdefABCDEF" for ch in text) else None


class DiskOutbox:
    """遥测 trace 帧的磁盘 outbox（原子写 + 扫描续传 + 划除确认）。

    线程安全：内部计数与文件系统操作在一把锁内完成（发送线程与主线程诊断
    可能并发）。本类**不发起网络 IO**，只负责磁盘真相；重放时机与 outstanding
    记账由 :class:`uav_gpr_qt.RemoteSender` 决定。
    """

    def __init__(self, root: Path, *, metrics=None) -> None:
        #: 采集输出目录（outbox 落在 ``<root>/_outbox/`` 下）。
        self.root = Path(root)
        #: 可选性能统计（R-B-08 的 ``outbox_write`` 阶段；注入失败绝不影响写盘）。
        self.metrics = metrics
        self._lock = threading.RLock()
        self._counters = {
            "appended": 0,
            "write_failures": 0,
            "removed": 0,
            "already_removed": 0,
            "already_present": 0,
            "ack_removal_failures": 0,
            #: 帧/身份/路径非法被拒绝的条数（与 write_failures 分账：前者=上游
            #: 不变量破坏，后者=磁盘故障。两者都属"该条未受保护"）。
            "rejections": 0,
            "corrupt_files": 0,
            "tmp_orphans": 0,
            "replayed": 0,
            "replay_duplicate_acked": 0,
        }
        #: 损坏隔离明细（**实例属性**：类级可变默认会在多实例间共享，制造
        #: 跨 run/跨测试污染——第1轮复审点名的"计数说谎"同类陷阱）。
        self._damage_log: list[dict[str, str]] = []

    # ---- 路径 ----

    @property
    def outbox_dir(self) -> Path:
        return self.root / OUTBOX_DIR_NAME

    @property
    def corrupt_dir(self) -> Path:
        return self.outbox_dir / CORRUPT_DIR_NAME

    def run_dir(self, run_id: object) -> Path:
        name = sanitize_run_dir_name(run_id)
        # **纯词法**越界判据。sanitize 已把 "/"、"\\"、空格等全部换成 "_"，
        # 因此单个路径段内唯一可能逃逸的形态是以 '.' 打头的名字（".."、
        # "../.." 经 sanitize 后仍以 '.' 开头）——该前提下词法判据完备。
        #
        # 为什么不用 Path.resolve() 比较父目录：t5 实测出现过
        # "run_id 目录名 '20260906_101112_0001' 解析后逃出 outbox 根" 的
        # **间歇**误拒（该错误串的唯一来源就是那条 resolve() 比较；约 1/6~1/3
        # 的跑次触发，出现在"上一条 ACK 划除把空 run 目录 prune 掉"与"下一条
        # 入队落盘"交错时）。隔离探针在本机未能复现该差异（本机对缺失目录的
        # resolve() 两侧仍相等），因此**底层确切原因未定论**。
        # 但结论不依赖根因：安全判据绝不能依赖文件系统瞬时状态——它会让同一条
        # 合法数据"是否受崩溃保护"变成不确定事件，且误拒以 write_failure 面目
        # 掩盖一条本该落盘的数据（真实丢保护，正是本模块要消灭的失败模式）。
        if name.startswith(".") or "/" in name or "\\" in name:
            raise OutboxFrameError(f"outbox 拒绝：run_id 目录名 {name!r} 可能造成路径越界")
        return self.outbox_dir / name

    def frame_path(self, run_id: object, trace_index: int) -> Path:
        return self.run_dir(run_id) / f"{int(trace_index):06d}{FRAME_SUFFIX}"

    # ---- 写入（原子） ----

    def append(
        self,
        frame: bytes,
        run_id: object,
        trace_index: object,
        *,
        header: dict[str, object] | None = None,
    ) -> Path:
        """原子落盘一帧，返回最终路径。

        两条失败通道**分账且都不得静默**（R-B-02）：

        * :class:`OutboxFrameError` → 计 ``rejections``：帧内容/身份/路径非法，
          属上游不变量被破坏（例如 §3.7 同身份多载荷），与磁盘健康无关；
        * :class:`OutboxWriteError` → 计 ``write_failures``：mkdir/write/fsync/
          replace 的真实 IO 故障（磁盘满、权限、设备忙）。

        历史缺陷（t5 自抓）：身份/路径校验原先在计数**之外**，抛错时两个计数器
        都不动 ⇒ "本该受保护却没保住"变成静默事件，正是 flaky 用例里
        ``appended`` 少 1 而 ``write_failures`` 仍为 0 的原因。

        ``header`` 仅用于**写前自校验**（身份/长度/sha256 与帧内一致），不改变
        落盘字节——落盘的永远是调用方给的 :attr:`frame` 原字节。
        """
        try:
            identity = self._validate_identity(run_id, trace_index)
            parsed_header, parsed_payload = parse_frame(frame)
            self._validate_entry_bytes(parsed_header, parsed_payload, source="append")
            if header is not None and header != parsed_header:
                raise OutboxFrameError("传入 header 与帧内 header 不一致（拒绝落盘，防双身份）")
            path = self.frame_path(identity[0], identity[1])
        except OutboxFrameError as exc:
            # 拒绝同样是"该条未受保护"，必须计数可观测，绝不静默逃逸。
            with self._lock:
                self._counters["rejections"] += 1
            raise
        tmp_path = path.with_name(path.name + TMP_SUFFIX)
        _t0 = time.perf_counter()
        try:
            with self._lock:
                existing = self._read_existing(path)
                if existing is not None:
                    if existing == frame:
                        # 完全相同重写：幂等，不产生第二个身份、不改动字节。
                        self._counters["already_present"] += 1
                        return path
                    # 同 (run_id, trace_index) 不同内容 ⇒ §3.7 不变量已被上游
                    # 破坏。此处**绝不覆盖**（覆盖=静默丢失先前未确认数据），
                    # fail-closed 抛出，让调用方与地面都不会看到"半新半旧"。
                    raise OutboxFrameError(
                        f"outbox 拒绝覆盖：run={identity[0]} trace={identity[1]} 已存在"
                        f"不同内容（{len(existing)}B vs {len(frame)}B）——同身份多载荷违反 §3.7"
                    )
                path.parent.mkdir(parents=True, exist_ok=True)
                with open(tmp_path, "wb") as handle:
                    handle.write(frame)
                    handle.flush()
                    os.fsync(handle.fileno())
                os.replace(tmp_path, path)
                self._counters["appended"] += 1
        except OSError as exc:
            # 半成品不外露：尽力清理 tmp（失败不掩盖原始异常）。
            try:
                tmp_path.unlink(missing_ok=True)
            except OSError:
                pass
            with self._lock:
                self._counters["write_failures"] += 1
            raise OutboxWriteError(f"outbox 写盘失败 run={identity[0]} trace={identity[1]}: {exc}") from exc
        except OutboxFrameError:
            # 锁内拒绝（同身份不同内容）与锁外拒绝同账：一律计 rejections，
            # 不得只抛不记（否则"本该受保护却没保住"再次静默）。
            with self._lock:
                self._counters["rejections"] += 1
            raise
        except OutboxWriteError:
            # 已存在帧不可读（同样"该条未受保护"）：计数与抛出一致，不得只抛不记。
            with self._lock:
                self._counters["write_failures"] += 1
            raise
        finally:
            self._record_stage("outbox_write", time.perf_counter() - _t0)
        return path

    def append_message(self, header: dict[str, object], payload: bytes) -> Path:
        """便利入口：由 (header, payload) 定型帧字节后原子落盘。

        落盘字节 == 线上字节（同一 :func:`frame_bytes`），重放零改写。
        """
        return self.append(frame_bytes(header, payload), header.get("run_id"), header.get("trace_index"))

    # ---- 扫描（恢复） ----

    def scan_pending(self, *, isolate_damage: bool = True) -> list[OutboxEntry]:
        """扫描全部未确认条目，按 ``(run_id, trace_index)`` 排序（§4.4）。

        ``isolate_damage=True``（默认）：损坏/身份非法的文件移入 ``_corrupt/``
        并计数（不删除、不重发）；崩溃遗留的 ``*.tmp`` 一并隔离（按未完成处理）。
        ``False`` 用于只读诊断（不改动磁盘）。
        """
        entries: list[OutboxEntry] = []
        with self._lock:
            if not self.outbox_dir.is_dir():
                return entries
            corrupt_root = self.corrupt_dir
            for candidate in sorted(self.outbox_dir.rglob("*")):
                if not candidate.is_file():
                    continue
                # 隔离目录整体跳过：里面的文件**仍带 .frame 后缀**（隔离只改名
                # 不改后缀），不排除会把它们反复隔离、corrupt_files 计数无限膨胀。
                try:
                    candidate.relative_to(corrupt_root)
                    continue
                except ValueError:
                    pass
                name = candidate.name
                if name.endswith(TMP_SUFFIX):
                    if isolate_damage:
                        self._isolate(candidate, kind="tmp_orphan")
                    continue
                if not name.endswith(FRAME_SUFFIX):
                    continue  # 非帧文件（日志/杂项）不动。
                entry, problem = self._load(candidate)
                if entry is None:
                    if isolate_damage:
                        self._isolate(candidate, kind="corrupt", reason=problem or "unknown")
                    continue
                entries.append(entry)
        entries.sort(key=lambda item: item.sort_key)
        return entries

    def _iter_frame_paths(self) -> list[Path]:
        """当前未确认帧文件列表（**排除** ``_corrupt/`` 隔离子树）。

        隔离只改名不改后缀（仍带 ``.frame``），不排除会把"已判定损坏、保留取证"
        的文件重新计入 pending——统计说谎即失真。
        """
        if not self.outbox_dir.is_dir():
            return []
        corrupt_root = self.corrupt_dir
        found: list[Path] = []
        for candidate in self.outbox_dir.rglob("*"):
            if not candidate.is_file():
                continue
            name = candidate.name
            if not name.endswith(FRAME_SUFFIX) or name.endswith(TMP_SUFFIX):
                continue
            try:
                candidate.relative_to(corrupt_root)
                continue
            except ValueError:
                pass
            found.append(candidate)
        return found

    def pending_count(self) -> int:
        """未确认帧文件数（不隔离、不改磁盘）。"""
        with self._lock:
            return len(self._iter_frame_paths())

    def pending_bytes(self) -> int:
        with self._lock:
            total = 0
            for path in self._iter_frame_paths():
                try:
                    total += path.stat().st_size
                except OSError:
                    continue
            return total

    # ---- 划除（ACK 持久化确认） ----

    def remove(self, entry: OutboxEntry | Path) -> bool:
        """收到匹配 ack_trace 后划除。返回是否处于"已确认"状态。

        幂等：文件本就不存在 ⇒ 视为成功（duplicate ACK 重复划除安全，§4.2），
        计入 ``already_removed``。真删除失败（被占用等）⇒ **不**算确认成功，
        返回 False 并计入 ``ack_removal_failures``；调用方据此决定不减
        outstanding（下一条条款：删除失败不得伪装排空完成）。
        """
        path_value = entry.path if isinstance(entry, OutboxEntry) else getattr(entry, "path", None)
        path = Path(path_value) if isinstance(path_value, (str, Path)) else Path(entry)  # type: ignore[arg-type]
        with self._lock:
            try:
                path.unlink()
            except FileNotFoundError:
                self._counters["already_removed"] += 1
                return True
            except OSError:
                self._counters["ack_removal_failures"] += 1
                return False
            self._counters["removed"] += 1
            self._prune_empty_run_dir(path.parent)
            return True

    #: 契约 §4.7 的别名：ACK 回执持久化 = 划除帧文件。
    def mark_acked(self, entry: OutboxEntry | Path) -> bool:
        return self.remove(entry)

    def note_replayed(self, *, duplicate_acked: bool = False) -> None:
        """重放计数（供诊断：补传条数、其中被地面判为 duplicate 的条数）。"""
        with self._lock:
            self._counters["replayed"] += 1
            if duplicate_acked:
                self._counters["replay_duplicate_acked"] += 1

    # ---- 统计 ----

    def statistics(self) -> dict[str, object]:
        """JSON 可序列化快照（诊断/测试/终报取证；绝不抛异常）。"""
        with self._lock:
            snapshot = dict(self._counters)
        return {
            "enabled": True,
            "root": str(self.outbox_dir),
            "pending_files": self.pending_count(),
            "pending_bytes": self.pending_bytes(),
            **snapshot,
        }

    # ---- 内部 ----

    def _read_existing(self, path: Path) -> bytes | None:
        """已存在帧文件的字节；不存在返回 None。

        存在但**读不出来**（权限/IO 故障）绝不能当作"不存在"继续覆盖——那会
        静默丢失一条尚未确认的数据。此时抛错（fail-closed），由调用方上报。
        """
        try:
            return path.read_bytes()
        except FileNotFoundError:
            return None
        except IsADirectoryError as exc:
            raise OutboxWriteError(f"outbox 路径被目录占用，拒绝覆盖：{path.name}: {exc}") from exc
        except OSError as exc:
            raise OutboxWriteError(f"outbox 无法读取已存在帧，拒绝覆盖未确认数据：{path.name}: {exc}") from exc

    def _validate_identity(self, run_id: object, trace_index: object) -> tuple[str, int]:
        text = run_id if isinstance(run_id, str) else ""
        if not text:
            raise OutboxFrameError("outbox 拒绝：run_id 缺失或非字符串")
        if classify_run_id(text) != RUN_ID_CLASS_AIR_TASK:
            # 非采集任务身份（idle/校准/本地）永不可能需要补发；出现即
            # 身份被误用，fail-closed 拒绝，绝不落盘成"看起来像数据"的文件。
            raise OutboxFrameError(
                f"outbox 拒绝：run_id={text!r} 不属采集任务式（类别={classify_run_id(text)}），"
                f"重放它只会污染地面当前任务身份"
            )
        index = _positive_int(trace_index)
        if index is None:
            raise OutboxFrameError(f"outbox 拒绝：trace_index 非法（{trace_index!r}）")
        return text, index

    def _validate_entry_bytes(self, header: dict[str, object], payload: bytes, *, source: str) -> None:
        where = f"{source}: "
        if header.get("type") != "trace":
            raise OutboxFrameError(f"{where}只允许持久化 trace 帧（实际 type={header.get('type')!r}）")
        sha = _valid_sha(header.get("sha256"))
        if sha is None:
            raise OutboxFrameError(f"{where}sha256 缺失或非法（需 64 位十六进制）")
        actual = hashlib.sha256(payload).hexdigest()
        if sha != actual:
            raise OutboxFrameError(f"{where}sha256 与 payload 实算不符（header={sha[:12]}… payload={actual[:12]}…）")
        declared = header.get("payload_bytes")
        if isinstance(declared, bool) or not isinstance(declared, int) or declared != len(payload):
            raise OutboxFrameError(f"{where}payload_bytes 声明 {declared!r} 与实际 {len(payload)} 不符")
        if _positive_int(header.get("trace_index")) is None:
            raise OutboxFrameError(f"{where}trace_index 非法（{header.get('trace_index')!r}）")
        if classify_run_id(header.get("run_id")) != RUN_ID_CLASS_AIR_TASK:
            raise OutboxFrameError(f"{where}header run_id 非采集任务式（{header.get('run_id')!r}）")

    def _load(self, path: Path) -> tuple[OutboxEntry | None, str | None]:
        """读一个帧文件并完整校验；不合法返回 (None, 原因)。"""
        try:
            raw = path.read_bytes()
        except OSError as exc:
            return None, f"读取失败：{exc}"
        try:
            header, payload = parse_frame(raw)
            self._validate_entry_bytes(header, payload, source="scan")
            run_dir_name = path.parent.name
            if run_dir_name != sanitize_run_dir_name(header.get("run_id")):
                raise OutboxFrameError(
                    f"目录名 {run_dir_name!r} 与 header run_id {header.get('run_id')!r} 不符"
                )
            stem = path.name[: -len(FRAME_SUFFIX)]
            if _positive_int(int(stem)) != _positive_int(header.get("trace_index")):
                raise OutboxFrameError(f"文件名 {stem!r} 与 header trace_index 不符")
        except (OutboxFrameError, ValueError) as exc:
            return None, str(exc)
        entry = OutboxEntry(
            path=path,
            run_id=str(header["run_id"]),
            trace_index=int(header["trace_index"]),
            sha256=str(header["sha256"]),
            frame=raw,
            header=header,
            payload=payload,
        )
        return entry, None

    def _isolate(self, path: Path, *, kind: str, reason: str = "") -> Path | None:
        """把损坏/未完成文件移入 ``_corrupt/``（保留取证，不删除）。"""
        try:
            self.corrupt_dir.mkdir(parents=True, exist_ok=True)
            target = self.corrupt_dir / f"{sanitize_run_dir_name(path.parent.name)}__{path.name}"
            if target.exists():  # 同名冲突加时间戳，绝不覆盖既有取证。
                target = self.corrupt_dir / f"{target.stem}.{time.time_ns()}{target.suffix}"
            os.replace(path, target)
        except OSError:
            # 连隔离都失败（磁盘故障级）：只计数并留下告警线索，不抛出杀死扫描。
            with self._lock:
                self._counters["corrupt_files"] += 1
            return None
        with self._lock:
            self._counters["corrupt_files" if kind == "corrupt" else "tmp_orphans"] += 1
            self._damage_log.append({"kind": kind, "file": path.name, "reason": reason})
        return target

    @property
    def damage_report(self) -> list[dict[str, str]]:
        """损坏隔离明细（fail-closed 的可观测面）。"""
        with self._lock:
            return list(self._damage_log)

    def _prune_empty_run_dir(self, directory: Path) -> None:
        """run 目录已空则删除（避免"崩溃恢复基准"里残留空目录被误统计）。"""
        try:
            if directory.is_dir() and not any(directory.iterdir()):
                directory.rmdir()
        except OSError:
            pass

    def _record_stage(self, stage: str, duration_s: float) -> None:
        metrics = self.metrics
        if metrics is None:
            return
        try:
            metrics.record_stage(stage, duration_s)
        except Exception:  # noqa: BLE001 - 统计故障绝不影响落盘主链路
            pass
