#!/usr/bin/env python3
"""第7轮 t1：空中端黑匣子服务（无界面后台程序，docs/第7轮_计划_空中端黑匣子与计算棒部署.md §4）。

设计要点（与计划/契约逐条对应）：

- **复用而非第二套**：采集/发送/补传核心全部直接 import ``air_core.py``（2026-10-15 前名为 ``uav_gpr_qt.py``）
  既有零件（AcquisitionService、ContinuousAcquisitionWorker、AcquisitionWorker、
  RemoteSender、GnssReader 等），本文件只做无界面接线（QCoreApplication +
  offscreen），绝不复制采集/发送/补传逻辑；
- **配置三层**：``air_config.json``（程序目录旁）→ 默认值；损坏/非法字段逐条
  回退默认值并在日志说明（不崩）；命令行覆盖优先级最高（--server/--gnss-port/
  --gnss-baud/--engine/--autostart/--config）；
- **自动连接与断线自愈**：启动即连 ground_host:ground_port，复用 RemoteSender
  既有重连线程；经其第7轮新增 ``reconnect_backoff`` 注入点施加 5s 起步、
  30s 封顶的指数退避（GUI 默认路径零变化）；断线期间采集照跑、DiskOutbox
  本地保存、恢复后自动补传（全部现状行为）；
- **远程命令词表**（两端接头暗号，与 t3 对齐；沿既有 ``command`` 消息只加
  action 值，严禁新增消息 type；回执走既有 command_ack + 第7轮共享挂钩新增的
  ``supported``/``result`` 字段）：
    get_status / test_vna / list_serial_ports / set_gnss_port / test_gnss /
    set_engine / set_device_source；
  同时支持既有 start/stop/single/pause/resume/set_params（老地面端互操作）；
  远程校准 cal_begin/cal_capture/cal_finish/cal_activate/cal_cancel/cal_status
  已移植 MainWindow 会话编排的无界面等价实现（复用 CalibrationSession +
  ServiceStepCapturer，扫频回退/落盘/日志取黑匣子自身状态）；
  未知 action 一律回执 supported=false（"不支持"），绝不崩；
- **磁盘水位**（D10）：定期检查 output_dir 所在盘剩余空间，低于
  disk_min_free_gb 触发，从最老 run 删起清回 ≥ disk_target_free_gb；优先删
  已成功送达的 run（outbox 无未确认帧）；动到未送达数据必须显式报警（日志 +
  get_status 字段）并逐条记账；剩余空间探测可注入（测试/t5 冒烟）；
- **滚动日志**：logs/air_service.log，默认 20MB×10 份。

红线自查：禁 git 写操作；framing 字段零改动；无新增消息 type；
disk_outbox.py / acquisition_scheduler.py 零改动。
"""

from __future__ import annotations

import argparse
import json
import logging
import logging.handlers
import os
import shutil
import signal
import sys
import threading
import time
from dataclasses import asdict, dataclass, fields as dataclass_fields
from pathlib import Path
from typing import Callable

import numpy as np

# offscreen 必须在任何 Qt 平台选择之前生效（黑匣子无任何窗口）。
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, str(Path(__file__).resolve().parent))

import air_core as air  # noqa: E402  （既有核心零件唯一来源，禁止复制第二套）
from air_gnss import AirGnssMonitor
from disk_outbox import (  # noqa: E402
    AIR_TASK_RUN_ID_RE,
    CALIBRATION_RUN_ID_RE,
    OUTBOX_DIR_NAME,
    sanitize_run_dir_name,
)

from PySide6 import QtCore  # noqa: E402

PROGRAM_VERSION = "8.0.0-round8"

#: 计划 §9：重连退避 5 秒起步、最长 30 秒。
RECONNECT_INITIAL_S = 5.0
RECONNECT_MAX_S = 30.0
#: 计划 §9：日志滚动默认 20MB × 10 份。
LOG_MAX_BYTES_DEFAULT = 20 * 1024 * 1024
LOG_BACKUP_COUNT_DEFAULT = 10
#: 磁盘水位检查周期（计划 §4.4"定期检查"）。
DISK_CHECK_INTERVAL_S = 60.0

ENGINE_CHOICES = ("continuous", "legacy")
DEVICE_SOURCE_CHOICES = ("usb", "simulated")

#: 黑匣子暂不支持的既有词表 action（回执 supported=false，不崩、不静默）。
#: 历史上 cal_*（远程校准编排）曾在此集合中；现已移植 MainWindow 会话编排
#: 的无界面等价实现（见 AirBlackboxService 远程校准一节），全部支持。
#: 保留该常量供未来"已知但暂不支持"的 action 使用。
UNSUPPORTED_KNOWN_ACTIONS = frozenset()


# =========================================================================
# 配置（air_config.json 三层：默认值 → 文件 → 命令行）
# =========================================================================


@dataclass
class AirConfig:
    """黑匣子配置（字段与默认值 = 契约原文；损坏/非法逐字段回退默认）。"""

    ground_host: str = "192.168.144.20"
    ground_port: int = 8765
    gnss_port: str = ""  # 空 = 自动识别 M100；off = 显式禁用
    #: 2026-10-09 现场实测：这条链路 115200 丢低电平位，9600 正常 → 默认 9600。
    gnss_baud: int = 9600
    engine: str = "continuous"  # continuous | legacy
    device_source: str = "usb"  # usb | simulated
    output_dir: str = "gpr_runs"
    autostart_acquisition: bool = False  # D11：等地面端命令
    disk_min_free_gb: float = 20.0  # D10 触发线
    disk_target_free_gb: float = 40.0  # D10 清理目标线
    log_max_bytes: int = LOG_MAX_BYTES_DEFAULT
    log_backup_count: int = LOG_BACKUP_COUNT_DEFAULT
    calibration_path: str = ""


def _field_names() -> tuple[str, ...]:
    return tuple(f.name for f in dataclass_fields(AirConfig))


def _validate_field(name: str, value: object) -> tuple[bool, object]:
    """校验单个配置字段。返回 (合法?, 规范化值)。非法由调用方回退默认值。"""
    if name == "calibration_path":
        return isinstance(value, str), str(value or "")
    if name == "ground_host":
        return isinstance(value, str) and bool(value.strip()), value
    if name == "ground_port":
        return isinstance(value, int) and not isinstance(value, bool) and 1 <= value <= 65535, value
    if name == "gnss_port":
        if value is None:
            return True, ""  # null 与空串同义：自动识别 M100
        return isinstance(value, str), value
    if name == "gnss_baud":
        return isinstance(value, int) and not isinstance(value, bool) and value > 0, value
    if name == "engine":
        return value in ENGINE_CHOICES, value
    if name == "device_source":
        return value in DEVICE_SOURCE_CHOICES, value
    if name == "output_dir":
        return isinstance(value, str) and bool(value.strip()), value
    if name == "autostart_acquisition":
        return isinstance(value, bool), value
    if name in ("disk_min_free_gb", "disk_target_free_gb"):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0, value
    if name == "log_max_bytes":
        return isinstance(value, int) and not isinstance(value, bool) and value > 0, value
    if name == "log_backup_count":
        return isinstance(value, int) and not isinstance(value, bool) and value >= 0, value
    return False, value  # 未知字段：视为非法（调用方忽略并记 issue）


def load_air_config(
    path: Path | None,
    *,
    log: Callable[[str], None] | None = None,
) -> tuple[AirConfig, list[str]]:
    """读取配置文件。损坏/非法 → 默认值 + issue 说明，绝不抛错、绝不崩。

    返回 (配置, issues)；issues 为人读说明（同时写日志），供测试与冒烟断言。
    """
    issues: list[str] = []

    def note(message: str) -> None:
        issues.append(message)
        if log is not None:
            log(f"配置: {message}")

    config = AirConfig()
    if path is None or not Path(path).exists():
        if path is not None:
            note(f"配置文件不存在，使用默认值: {path}")
        return config, issues
    try:
        raw = Path(path).read_text(encoding="utf-8-sig")
    except OSError as exc:
        note(f"配置文件读取失败（使用默认值）: {exc}")
        return config, issues
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        note(f"配置文件损坏（{exc}），整体使用默认值")
        return config, issues
    if not isinstance(data, dict):
        note(f"配置文件顶层不是 JSON 对象（{type(data).__name__}），整体使用默认值")
        return config, issues
    defaults = AirConfig()
    for name in _field_names():
        if name not in data:
            continue
        value = data[name]
        ok, normalized = _validate_field(name, value)
        if not ok:
            note(
                f"字段 {name} 非法（{value!r}），回退默认值 {getattr(defaults, name)!r}"
            )
            continue
        setattr(config, name, normalized)
    # 目标线必须 ≥ 触发线，否则清理永远达不到目标；非法 → 目标线回默认。
    if config.disk_target_free_gb < config.disk_min_free_gb:
        note(
            f"disk_target_free_gb({config.disk_target_free_gb}) < disk_min_free_gb"
            f"({config.disk_min_free_gb})，目标线回退默认值 {defaults.disk_target_free_gb}"
        )
        config.disk_target_free_gb = defaults.disk_target_free_gb
    unknown = sorted(set(data) - set(_field_names()))
    if unknown:
        note(f"忽略未知配置字段: {', '.join(unknown)}")
    return config, issues


def apply_cli_overrides(
    config: AirConfig, args: argparse.Namespace, *, log: Callable[[str], None] | None = None
) -> AirConfig:
    """命令行覆盖（优先级：命令行 > 配置 > 默认）。非法命令行值已在 argparse 拦下。"""

    def note(message: str) -> None:
        if log is not None:
            log(f"命令行覆盖: {message}")

    if getattr(args, "server", None):
        text = str(args.server).strip()
        host, sep, port_text = text.rpartition(":")
        if not sep or not host:
            raise ValueError(f"--server 需要 HOST:PORT 形式，收到: {text!r}")
        try:
            port = int(port_text)
        except ValueError as exc:
            raise ValueError(f"--server 端口非法: {text!r}") from exc
        if not 1 <= port <= 65535:
            raise ValueError(f"--server 端口越界: {port}")
        note(f"ground_host={host} ground_port={port}")
        config.ground_host = host
        config.ground_port = port
    if getattr(args, "gnss_port", None) is not None:
        config.gnss_port = str(args.gnss_port)
        note(f"gnss_port={config.gnss_port!r}")
    if getattr(args, "gnss_baud", None) is not None:
        config.gnss_baud = int(args.gnss_baud)
        note(f"gnss_baud={config.gnss_baud}")
    if getattr(args, "engine", None):
        config.engine = str(args.engine)
        note(f"engine={config.engine}")
    if getattr(args, "autostart", None) is not None:
        config.autostart_acquisition = bool(args.autostart)
        note(f"autostart_acquisition={config.autostart_acquisition}")
    return config


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="uav_gpr_air_service.py",
        description="UAV-GPR 空中端黑匣子服务（无界面后台；自动连接地面端 + 远程遥控 + 磁盘水位）",
    )
    parser.add_argument(
        "--config",
        default=None,
        help="配置文件路径（默认：程序目录旁 air_config.json）",
    )
    parser.add_argument("--server", default=None, help="地面端 HOST:PORT（覆盖配置）")
    parser.add_argument("--gnss-port", default=None, help="GNSS 串口（空串=自动识别 M100；off=禁用；覆盖配置）")
    parser.add_argument("--gnss-baud", type=int, default=None, help="GNSS 波特率（覆盖配置）")
    parser.add_argument(
        "--engine", choices=list(ENGINE_CHOICES), default=None, help="采集引擎（覆盖配置）"
    )
    parser.add_argument(
        "--autostart",
        type=_parse_bool_arg,
        default=None,
        metavar="{true,false}",
        help="开机自动采集（默认 false=等地面命令；覆盖配置）",
    )
    return parser


def _parse_bool_arg(text: str) -> bool:
    lowered = str(text).strip().lower()
    if lowered in ("1", "true", "yes", "on"):
        return True
    if lowered in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"布尔参数非法: {text!r}")


# =========================================================================
# 重连退避（5s 起步、×2、30s 封顶；注入 RemoteSender.reconnect_backoff）
# =========================================================================


class AirReconnectBackoff:
    """线程安全退避策略：成功连接后 reset() 归位 5s。"""

    def __init__(self, initial_s: float = RECONNECT_INITIAL_S, max_s: float = RECONNECT_MAX_S) -> None:
        self.initial_s = max(0.1, float(initial_s))
        self.max_s = max(self.initial_s, float(max_s))
        self._next = self.initial_s
        self._lock = threading.Lock()

    def reset(self) -> None:
        with self._lock:
            self._next = self.initial_s

    def next_delay(self) -> float:
        with self._lock:
            delay = self._next
            self._next = min(self._next * 2.0, self.max_s)
        return delay

    def __call__(self) -> float:
        return self.next_delay()


# =========================================================================
# 磁盘水位自动清理（D10：触发 20GB / 目标 40GB；透明记账；未送达报警）
# =========================================================================


class DiskWatermarkManager:
    """从最老 run 删起，优先删已送达 run；删到未送达数据必须显式报警 + 逐条记账。

    "run" = output_dir 下名字符合空中 run_id 目录名规则的子目录
    （数据按 ``output_dir/<run_id>/`` 落盘；outbox 在 ``output_dir/_outbox/<run_id>/``）。
    "已送达" = 该 run 在 outbox 中已无未确认帧（全部收到 ack_trace 被划除）。
    磁盘剩余探测 ``free_space_bytes`` 可注入（测试与 t5 冒烟）。
    """

    def __init__(
        self,
        output_dir: Path,
        *,
        min_free_gb: float,
        target_free_gb: float,
        free_space_bytes: Callable[[], int] | None = None,
        outbox_dir: Path | None = None,
        active_run_id: Callable[[], str] | None = None,
        on_event: Callable[[str], None] | None = None,
        on_alarm: Callable[[str], None] | None = None,
        max_events: int = 200,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.min_free_gb = float(min_free_gb)
        self.target_free_gb = float(target_free_gb)
        self._free_space_bytes = free_space_bytes or self._default_free_space_bytes
        self.outbox_dir = Path(outbox_dir) if outbox_dir else self.output_dir / OUTBOX_DIR_NAME
        self._active_run_id = active_run_id or (lambda: "")
        self._on_event = on_event or (lambda message: None)
        self._on_alarm = on_alarm or self._on_event
        # 透明记账（零静默）：每条删除事件 {kind, path, bytes, delivered}
        self.events: list[dict[str, object]] = []
        self._max_events = int(max_events)
        # 状态快照（get_status 暴露给地面端面板）
        self.triggered = False
        self.undelivered_alarm = False
        self.undelivered_runs_deleted: list[str] = []
        self.runs_deleted = 0
        self.files_deleted = 0
        self.bytes_freed = 0
        self.last_check_free_gb: float | None = None
        self.last_check_at: str | None = None

    # ---- 探测 ----

    def _default_free_space_bytes(self) -> int:
        """output_dir 所在盘剩余字节（目录未建时向上找最近存在的祖先）。"""
        anchor = self.output_dir
        while not anchor.exists():
            parent = anchor.parent
            if parent == anchor:
                break
            anchor = parent
        return shutil.disk_usage(str(anchor)).free

    def free_bytes(self) -> int:
        return int(self._free_space_bytes())

    # ---- run 识别与送达判定 ----

    @staticmethod
    def is_run_dir_name(name: str) -> bool:
        return bool(AIR_TASK_RUN_ID_RE.match(name) or CALIBRATION_RUN_ID_RE.match(name))

    def pending_frame_count(self, run_name: str) -> int:
        """该 run 在 outbox 中的未确认帧数（存在 == 尚未收到 ack_trace）。

        返回 -1 表示探测失败（调用方一律不删，宁可不删也不误判"已送达"）。
        """
        directory = self.outbox_dir / run_name
        if not directory.is_dir():
            return 0
        try:
            return sum(1 for p in directory.iterdir() if p.is_file() and p.name.endswith(".frame"))
        except OSError:
            return -1

    def _candidate_runs(self) -> list[tuple[str, bool]]:
        """[(run_name, delivered)]，delivered 优先、同组内名字序==时间序（§3.7-1）。"""
        if not self.output_dir.is_dir():
            return []
        active = sanitize_run_dir_name(self._active_run_id())
        candidates: list[tuple[str, bool]] = []
        try:
            names = [p.name for p in self.output_dir.iterdir() if p.is_dir()]
        except OSError:
            return []
        for name in sorted(n for n in names if self.is_run_dir_name(n)):
            if name == active and active:
                continue  # 正在采集的 run 绝不删
            pending = self.pending_frame_count(name)
            if pending < 0:
                continue  # outbox 探测失败：宁可不删（fail-safe）
            candidates.append((name, pending == 0))
        candidates.sort(key=lambda item: (0 if item[1] else 1, item[0]))
        return candidates

    def _loose_legacy_files(self) -> list[Path]:
        """历史散落 trace 文件（GUI 时代直接落在 output_dir 的 layout）。

        其未送达副本若存在，必在 ``_outbox/<run>/`` 里（阶段3保护），删散落文件
        不影响补传；但删除仍逐条记账。
        """
        if not self.output_dir.is_dir():
            return []
        files: list[Path] = []
        try:
            for p in self.output_dir.iterdir():
                if p.is_file() and p.name.startswith("trace_") and p.suffix in (".npz", ".csv"):
                    files.append(p)
        except OSError:
            return []
        files.sort(key=lambda p: (p.stat().st_mtime if p.exists() else 0, p.name))
        return files

    # ---- 记账 ----

    def _record(self, kind: str, path: Path, size: int, *, delivered: bool | None = None) -> None:
        event: dict[str, object] = {
            "kind": kind,
            "path": str(path),
            "bytes": int(size),
            "at": air.utc_iso_now(),
        }
        if delivered is not None:
            event["delivered"] = bool(delivered)
        self.events.append(event)
        if len(self.events) > self._max_events:
            del self.events[: len(self.events) - self._max_events]
        self.files_deleted += 1
        self.bytes_freed += int(size)
        self._on_event(f"DISK-CLEANUP 记账 {kind} {path} ({size} 字节)")

    def _delete_run_dir(self, name: str, delivered: bool) -> int:
        """删除一个 run 目录，逐文件记账。返回释放字节数。"""
        directory = self.output_dir / name
        total = 0
        for file_path in sorted(directory.rglob("*")):
            if not file_path.is_file():
                continue
            try:
                size = file_path.stat().st_size
            except OSError:
                size = 0
            try:
                file_path.unlink()
            except OSError as exc:
                self._on_event(f"DISK-CLEANUP 删除失败（保留） {file_path}: {exc}")
                continue
            total += size
            self._record("run_file", file_path, size, delivered=delivered)
        try:
            shutil.rmtree(directory, ignore_errors=True)
        except OSError as exc:
            self._on_event(f"DISK-CLEANUP 目录收尾失败 {directory}: {exc}")
        self.runs_deleted += 1
        if not delivered:
            # 显式报警（绝不静默）：删掉了未送达数据。
            self.undelivered_alarm = True
            self.undelivered_runs_deleted.append(name)
            self._on_alarm(
                f"DISK-ALARM run {name} 含未送达数据仍被磁盘水位清理删除"
                f"（该 run 数据将无法补传；已逐条记账）"
            )
        self._on_event(
            f"DISK-CLEANUP run {name} 已删除（delivered={delivered}，释放 {total} 字节）"
        )
        return total

    def _delete_loose_file(self, path: Path) -> int:
        try:
            size = path.stat().st_size
        except OSError:
            size = 0
        try:
            path.unlink()
        except OSError as exc:
            self._on_event(f"DISK-CLEANUP 删除失败（保留） {path}: {exc}")
            return 0
        self._record("legacy_file", path, size, delivered=True)
        return size

    # ---- 主入口 ----

    def check(self) -> dict[str, object]:
        """检查水位；低于触发线即清理到目标线或候选耗尽。返回状态快照。"""
        free = self.free_bytes()
        self.last_check_free_gb = free / (1024 ** 3)
        self.last_check_at = air.utc_iso_now()
        if free >= int(self.min_free_gb * (1024 ** 3)):
            if self.triggered:
                self._on_event(
                    f"DISK-CLEANUP 水位恢复（剩余 {self.last_check_free_gb:.2f}GB ≥ "
                    f"{self.min_free_gb}GB，解除清理状态）"
                )
            self.triggered = False
            return self.state_dict()
        self.triggered = True
        self._on_event(
            f"DISK-CLEANUP 触发：剩余 {self.last_check_free_gb:.2f}GB < "
            f"{self.min_free_gb}GB，开始从最老 run 清理到 ≥{self.target_free_gb}GB"
        )
        target = int(self.target_free_gb * (1024 ** 3))
        # 阶段1：已送达 run（老→新）→ 阶段2：散落历史文件 → 阶段3：未送达 run（报警）。
        # 候选每阶段重算（active run 保护随采集状态变化）。
        for phase in range(3):
            if self.free_bytes() >= target:
                break
            if phase == 0:
                candidates = [item for item in self._candidate_runs() if item[1]]
                for name, delivered in candidates:
                    if self.free_bytes() >= target:
                        break
                    self._delete_run_dir(name, delivered)
            elif phase == 1:
                for path in self._loose_legacy_files():
                    if self.free_bytes() >= target:
                        break
                    self._delete_loose_file(path)
            else:
                candidates = [item for item in self._candidate_runs() if not item[1]]
                for name, delivered in candidates:
                    if self.free_bytes() >= target:
                        break
                    self._delete_run_dir(name, delivered)
        self._on_event(
            f"DISK-CLEANUP 本轮结束：剩余 {self.free_bytes() / (1024 ** 3):.2f}GB"
            f"（目标 ≥{self.target_free_gb}GB）"
        )
        return self.state_dict()

    def state_dict(self) -> dict[str, object]:
        """get_status 透出快照（含未送达报警标志，地面端面板可显式显示）。"""
        return {
            "triggered": self.triggered,
            "undelivered_alarm": self.undelivered_alarm,
            "undelivered_runs_deleted": list(self.undelivered_runs_deleted),
            "runs_deleted": self.runs_deleted,
            "files_deleted": self.files_deleted,
            "bytes_freed": self.bytes_freed,
            "last_check_free_gb": self.last_check_free_gb,
            "last_check_at": self.last_check_at,
            "min_free_gb": self.min_free_gb,
            "target_free_gb": self.target_free_gb,
            "recent_events": list(self.events[-20:]),
        }


# =========================================================================
# 远程校准会话级 capturer（无界面版）
# =========================================================================


class _BlackboxCalibrationCapturerProxy:
    """会话级 capturer：按步骤惰性创建 :class:`air.ServiceStepCapturer`。

    与 ``uav_gpr_qt._ServiceCalibrationCapturerProxy`` 同一协议
    （:class:`CalibrationSession` 只见 ``capture_one``/``close_current``），
    差异仅是配置与目录来源：扫频回退取黑匣子自身 ``_sweep``（经
    :meth:`AirBlackboxService._calibration_config_from_snapshot`），落盘取
    ``output_root``，日志与会话取黑匣子自身状态——绝不碰任何 QWidget。
    """

    def __init__(self, service: "AirBlackboxService", cal_id: str) -> None:
        self._service = service
        self._cal_id = cal_id
        self._step_capturers: dict[str, air.ServiceStepCapturer] = {}

    def _capturer_for(self, step: air.CalibrationStep) -> air.ServiceStepCapturer:
        capturer = self._step_capturers.get(step.name)
        if capturer is None:
            session = self._service._calibration_session
            snapshot = dict(getattr(session, "config_snapshot", {}) or {})
            cfg = self._service._calibration_config_from_snapshot(snapshot)
            if session is not None and session.capture_dir is not None:
                out_dir = session.capture_dir
            else:
                out_dir = (
                    self._service.output_root / "_calibration" / air.safe_cal_dir_name(self._cal_id)
                )
            capturer = air.ServiceStepCapturer(
                service=self._service.acquisition,
                cfg=cfg,
                cal_id=self._cal_id,
                run_id=self._service.acquisition.new_run_id(air.RUN_ID_KIND_CALIBRATION),
                step=step,
                out_dir=out_dir,
            )
            self._step_capturers[step.name] = capturer
        return capturer

    def capture_one(self, step: air.CalibrationStep, index: int) -> tuple[np.ndarray, np.ndarray]:
        return self._capturer_for(step).capture_one(step, index)

    def close_current(self, step_name: str) -> None:
        capturer = self._step_capturers.pop(step_name, None)
        if capturer is not None:
            capturer.close()


# =========================================================================
# 黑匣子服务本体
# =========================================================================


class AirBlackboxService:
    """无界面黑匣子：自动连接 + 远程命令 + 采集编排 + 磁盘水位。"""

    def __init__(
        self,
        config: AirConfig,
        *,
        program_dir: Path,
        config_path: Path | None = None,
        log: Callable[[str], None] | None = None,
        backend_factory: Callable[[], object] | None = None,
        free_space_bytes: Callable[[], int] | None = None,
        gnss_reader_factory: Callable[[str, int], object] | None = None,
        reconnect_initial_s: float = RECONNECT_INITIAL_S,
        reconnect_max_s: float = RECONNECT_MAX_S,
    ) -> None:
        self.config = config
        self.program_dir = Path(program_dir)
        self.config_path = Path(config_path) if config_path else self.program_dir / "air_config.json"
        self._log = log or (lambda message: None)
        self._config_lock = threading.RLock()
        self._started_at = time.monotonic()
        # 相对 output_dir 一律相对程序目录解析（计划任务下 CWD 不可靠）。
        self.output_root = Path(config.output_dir)
        if not self.output_root.is_absolute():
            self.output_root = self.program_dir / self.output_root
        # 进程级设备来源（后端装配回退点读取；非法值已在配置层拦下）。
        air.set_default_device_source(config.device_source)
        # 采集常驻服务（会话/状态机/run_id 签发全部复用既有实现）。
        self.acquisition = air.AcquisitionService(
            backend_factory=backend_factory,  # None → 按进程 device_source 装配
            status_callback=self._log,
        )
        # 无界面扫频任务状态（对应 MainWindow 各输入框的等价物）。
        self._sweep = air.SweepConfig()
        self._interval_s = 0.0
        self._count = 0
        self._autosave = True
        self._save_csv = False  # GUI 默认：关闭 CSV，提高棒子采集流畅度
        self._calibration_path = ""
        # 远程校准编排状态（与 MainWindow 同名同语义的无界面等价物）。
        self.active_calibration_id: str | None = None
        self._active_osl_id: str | None = None
        self._active_calibration_path: Path | None = None
        self._session_calibration = None
        self._session_calibration_p2 = None
        self._active_through = None
        self._background = None
        self._background_frequency_hz = None
        self._background_cal_id: str | None = None
        self._recent_requests: dict[str, dict[str, object]] = {}
        self._current_capture_step: str | None = None
        self._current_capture_request_id: str | None = None
        self._calibration_lock = threading.RLock()
        self._calibration_session = None
        self._calibration_thread: threading.Thread | None = None
        self._active_run_id = ""
        self._last_run_id = ""
        self._legacy_worker: air.AcquisitionWorker | None = None

        # 自启动恢复生效校准（R8-5 / R8-8 契约：持久化配置恢复）
        if self.config.calibration_path:
            cal_p = Path(self.config.calibration_path)
            if not cal_p.is_absolute():
                cal_p = self.program_dir / cal_p
            if cal_p.exists():
                try:
                    from librevna.librevna_usb_s11 import OSLCalibration
                    p1_cal, p2_cal, through_meas = OSLCalibration.load_components(cal_p)
                    self._session_calibration = p1_cal
                    self._session_calibration_p2 = p2_cal
                    self._active_through = through_meas
                    self._active_calibration_path = cal_p
                    self.active_calibration_id = cal_p.stem
                    self._active_osl_id = cal_p.stem
                    self._log(f"自启动恢复生效校准: {cal_p.name}")
                except Exception as exc:
                    self._log(f"自启动加载生效校准失败（已回退未校准）: {cal_p}: {exc}")
        self._usb_probe = _usb_probe_device_info  # 可测试注入点
        self._gnss_reader_factory = gnss_reader_factory
        self.gnss = AirGnssMonitor(config.gnss_port, config.gnss_baud,
                                   reader_factory=gnss_reader_factory, log=self._log)
        # 重连退避（生产 5s→30s；测试/冒烟可注入更短节奏）。
        self._backoff = AirReconnectBackoff(reconnect_initial_s, reconnect_max_s)
        self.connected = False
        self.reconnecting = False
        # 连接身份由采集服务签发（idle 式，§3.7 与 GUI 现状一致）。
        self.connection_run_id = self.acquisition.new_run_id(air.RUN_ID_KIND_IDLE)
        self.sender = air.RemoteSender(
            config.ground_host,
            int(config.ground_port),
            self.connection_run_id,
            self._on_sender_status,
            self.handle_command,
            outbox_dir=self.output_root,  # DiskOutbox 落位与 GUI 现状一致
            reconnect_backoff=self._backoff,
        )
        self.sender.gnss_snapshot = self.gnss.snapshot
        # 磁盘水位（探测可注入）。
        self.disk = DiskWatermarkManager(
            self.output_root,
            min_free_gb=config.disk_min_free_gb,
            target_free_gb=config.disk_target_free_gb,
            free_space_bytes=free_space_bytes,
            active_run_id=lambda: self._active_run_id,
            on_event=self._log,
        )

    # ---- 日志与连接状态 ----

    def _on_sender_status(self, text: str) -> None:
        """RemoteSender 状态文本：写日志 + 维护连接/退避状态（get_status 用）。"""
        self._log(text)
        if text.startswith("Remote connected:"):
            self.connected = True
            self.reconnecting = False
            self._backoff.reset()
        elif "retrying" in text or "Remote connect failed" in text:
            self.connected = False
            self.reconnecting = True

    # ---- 配置回写（设置类 action 持久化）----

    def save_config(self) -> bool:
        with self._config_lock:
            data = asdict(self.config)
            path = self.config_path
            tmp = path.with_name(path.name + ".tmp")
            try:
                tmp.write_text(
                    json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
                )
                os.replace(tmp, path)
            except OSError as exc:
                self._log(f"配置回写失败（保留内存值，下轮重试）: {exc}")
                return False
        self._log(f"配置已回写: {path}")
        return True

    # ---- 远程命令分发（既有 command 消息 + action；回执经共享挂钩带 supported/result）----

    def handle_command(self, action: str, params: object) -> dict[str, object]:
        """RemoteSender.command_callback：返回 dict → ack 带 supported/result。"""
        params_dict = params if isinstance(params, dict) else {}
        self._log(f"远程命令: {action} params={params_dict}")
        handler = self._dispatch_table().get(action)
        try:
            if handler is not None:
                result = dict(handler(params_dict))
                result.setdefault("supported", True)
                return result
            if action in UNSUPPORTED_KNOWN_ACTIONS:
                self._log(f"远程命令 {action}: 黑匣子暂不支持（回执 supported=false）")
                return {"supported": False}
            self._log(f"未知远程命令: {action}（回执 supported=false，不崩）")
            return {"supported": False}
        except Exception as exc:  # noqa: BLE001 - 单命令故障绝不炸 sender 线程
            self._log(f"远程命令 {action} 处理异常: {exc}")
            return {"supported": True, "ok": False, "error": str(exc)}

    def _record_completed_request(self, request_id: str, result: dict[str, object]) -> None:
        if not request_id:
            return
        if len(self._recent_requests) >= 128:
            oldest_key = next(iter(self._recent_requests))
            self._recent_requests.pop(oldest_key, None)
        self._recent_requests[request_id] = result

    def _dispatch_table(self) -> dict[str, Callable[[dict], dict[str, object]]]:
        return {
            "get_status": self._cmd_get_status,
            "test_vna": self._cmd_test_vna,
            "list_serial_ports": self._cmd_list_serial_ports,
            "set_gnss_port": self._cmd_set_gnss_port,
            "test_gnss": self._cmd_test_gnss,
            "set_engine": self._cmd_set_engine,
            "set_device_source": self._cmd_set_device_source,
            "start": self._cmd_start,
            "stop": self._cmd_stop,
            "single": self._cmd_single,
            "pause": self._cmd_pause,
            "resume": self._cmd_resume,
            "set_params": self._cmd_set_params,
            "cal_begin": self._cmd_cal_begin,
            "cal_capture": self._cmd_cal_capture,
            "cal_finish": self._cmd_cal_finish,
            "cal_activate": self._cmd_cal_activate,
            "cal_cancel": self._cmd_cal_cancel,
            "cal_status": self._cmd_cal_status,
            "cal_import_file": self._cmd_cal_import_file,
            "cal_export_file": self._cmd_cal_export_file,
            "cal_query_active": self._cmd_cal_query_active,
            "cal_clear_active": self._cmd_cal_clear_active,
        }

    # ---- 状态快照 ----

    def _active_config_dict(self) -> dict[str, object]:
        with self._config_lock:
            payload: dict[str, object] = {
                "engine": self.config.engine,
                "device_source": air.get_default_device_source(),
                "gnss_port": self.config.gnss_port,
                "gnss_baud": self.config.gnss_baud,
                "ground_host": self.config.ground_host,
                "ground_port": self.config.ground_port,
                "output_dir": self.config.output_dir,
                "autostart_acquisition": self.config.autostart_acquisition,
                "disk_min_free_gb": self.config.disk_min_free_gb,
                "disk_target_free_gb": self.config.disk_target_free_gb,
            }
        payload.update(asdict(self._sweep))
        payload["interval_s"] = self._interval_s
        payload["count"] = self._count
        payload["autosave"] = self._autosave
        payload["save_csv"] = self._save_csv
        return payload

    def _is_acquiring(self) -> bool:
        if self.acquisition.state in (air.SERVICE_STATE_ACQUIRING, air.SERVICE_STATE_STOPPING):
            return True
        worker = self._legacy_worker
        return worker is not None and worker.isRunning()

    def _status_payload(self) -> dict[str, object]:
        counts = self.sender.backlog_counts()
        outbox_pending = self.sender.outbox.pending_count() if self.sender.outbox is not None else 0
        run_id = self._active_run_id or self._last_run_id
        acquiring = self._is_acquiring()
        disk_state = self.disk.state_dict()
        payload: dict[str, object] = {
            "ok": True,
            "online": self.connected,
            "connected": self.connected,
            "reconnecting": self.reconnecting,
            "vna_usb": self._vna_present(),
            "run_id": run_id,
            "acquiring": acquiring,
            "backlog": int(counts["outstanding"]) + int(outbox_pending),
            "backlog_detail": {**counts, "outbox_pending": outbox_pending},
            "disk_free_gb": (
                round(self.disk.last_check_free_gb, 3)
                if self.disk.last_check_free_gb is not None
                else round(self.disk.free_bytes() / (1024 ** 3), 3)
            ),
            "version": PROGRAM_VERSION,
            "program_version": PROGRAM_VERSION,
            "config": self._active_config_dict(),
            "active_config": self._active_config_dict(),
            "sweep": asdict(self._sweep),
            "service_state": self.acquisition.state,
            "engine": self.config.engine,
            "device_source": air.get_default_device_source(),
            "disk_cleanup": disk_state,
            "pid": os.getpid(),
            "uptime_s": round(time.monotonic() - self._started_at, 1),
        }
        return payload

    def _cmd_get_status(self, _params: dict) -> dict[str, object]:
        return self._status_payload()

    # ---- VNA 探测 ----

    def _vna_present(self) -> bool:
        """雷达 USB 在位判定：常驻会话持有 → 在位；否则只读枚举（不 claim，不撞会话）。"""
        backend = self.acquisition.backend
        if backend is not None and getattr(backend, "is_open", False):
            return True
        if air.get_default_device_source() == air.DEVICE_SOURCE_SIMULATED:
            return False  # 模拟器旁路无真实 USB，如实上报
        try:
            from librevna.librevna_usb_probe import VID, PID  # noqa: PLC0415

            import usb.core  # noqa: PLC0415

            return usb.core.find(idVendor=VID, idProduct=PID) is not None
        except Exception:  # noqa: BLE001 - 探测失败不谎报在位
            return False

    def _cmd_test_vna(self, _params: dict) -> dict[str, object]:
        backend = self.acquisition.backend
        if backend is not None and getattr(backend, "is_open", False):
            info = getattr(backend, "device_info", None)
            if info is not None:
                return {
                    "ok": True,
                    "online": True,
                    "firmware": info.firmware,
                    "protocol": info.protocol,
                    "max_freq_hz": int(info.max_freq_hz),
                    "summary": info.summary,
                    "source": "session",
                }
        if air.get_default_device_source() == air.DEVICE_SOURCE_SIMULATED:
            return {
                "ok": False,
                "online": True,
                "error": "模拟器模式（无真实 USB 雷达）",
                "source": "simulated",
            }
        return self._usb_probe()

    # ---- 串口 / GNSS ----

    def _cmd_list_serial_ports(self, _params: dict) -> dict[str, object]:
        from serial.tools import list_ports  # noqa: PLC0415

        ports = [
            {"port": p.device, "description": p.description}
            for p in list_ports.comports()
        ]
        return {"ok": True, "ports": ports, "count": len(ports)}

    def _cmd_set_gnss_port(self, params: dict) -> dict[str, object]:
        port = params.get("port", "")
        if port is None:
            port = ""
        if not isinstance(port, str):
            return {"ok": False, "error": f"port 非法: {port!r}"}
        baud_raw = params.get("baud", self.config.gnss_baud)
        try:
            baud = int(baud_raw)
        except (TypeError, ValueError):
            return {"ok": False, "error": f"baud 非法: {baud_raw!r}"}
        if baud <= 0:
            return {"ok": False, "error": f"baud 必须 > 0: {baud}"}
        with self._config_lock:
            self.config.gnss_port = port
            self.config.gnss_baud = baud
            saved = self.save_config()
        self.gnss.configure(port, baud)
        self._log(f"GNSS 串口设置: port={port!r} baud={baud}（后台立即重连）回写={'成功' if saved else '失败'}")
        return {"ok": True, "port": port, "baud": baud, "saved": saved, "applies": "immediate"}

    def _cmd_test_gnss(self, params: dict) -> dict[str, object]:
        # The persistent GNSS owner holds the serial port; never reopen it here.
        return self.gnss.snapshot()

    # ---- 设置类（回写 air_config.json）----

    def _cmd_set_engine(self, params: dict) -> dict[str, object]:
        engine = params.get("engine")
        if engine not in ENGINE_CHOICES:
            return {"ok": False, "error": f"engine 非法: {engine!r}（可选 {'|'.join(ENGINE_CHOICES)}）"}
        with self._config_lock:
            self.config.engine = str(engine)
            saved = self.save_config()
        self._log(f"采集引擎切换: {engine}（下一采集任务生效）回写={'成功' if saved else '失败'}")
        return {"ok": True, "engine": engine, "saved": saved}

    def _cmd_set_device_source(self, params: dict) -> dict[str, object]:
        source = params.get("device_source")
        if source not in DEVICE_SOURCE_CHOICES:
            return {
                "ok": False,
                "error": f"device_source 非法: {source!r}（可选 {'|'.join(DEVICE_SOURCE_CHOICES)}）",
            }
        try:
            air.set_default_device_source(str(source))
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        with self._config_lock:
            self.config.device_source = str(source)
            saved = self.save_config()
        backend = self.acquisition.backend
        busy = backend is not None and getattr(backend, "is_open", False)
        self._log(
            f"设备来源切换: {source} 回写={'成功' if saved else '失败'}"
            f"（{'当前 USB 会话保持原来源，下一会话生效' if busy else '下一采集任务生效'}）"
        )
        return {
            "ok": True,
            "device_source": source,
            "saved": saved,
            "applies": "next_session" if busy else "next_task",
        }

    # ---- 采集命令（start/stop/single/pause/resume/set_params）----

    def _cmd_start(self, params: dict) -> dict[str, object]:
        count_override = params.get("count")
        if count_override is not None:
            try:
                self._count = max(0, int(count_override))
            except (TypeError, ValueError):
                return {"ok": False, "error": f"count 非法: {count_override!r}"}
        started = self.start_acquisition(single=bool(params.get("single", False)))
        return {
            "ok": True,
            "started": bool(started),
            "state": self.acquisition.state,
            "run_id": self._active_run_id,
            "reason": None if started else "服务拒绝启动（详见空中端日志）",
        }

    def _cmd_single(self, params: dict) -> dict[str, object]:
        return self._cmd_start({**params, "single": True})

    def _cmd_stop(self, _params: dict) -> dict[str, object]:
        stopped = self.stop_acquisition()
        return {"ok": True, "stopped": bool(stopped), "state": self.acquisition.state}

    def _cmd_pause(self, _params: dict) -> dict[str, object]:
        paused = self.acquisition.request_pause()
        return {"ok": True, "paused": bool(paused)}

    def _cmd_resume(self, _params: dict) -> dict[str, object]:
        resumed = self.acquisition.request_resume()
        return {"ok": True, "resumed": bool(resumed)}

    def _cmd_set_params(self, params: dict) -> dict[str, object]:
        if self._is_acquiring():
            return {"ok": False, "error": "采集运行中不可修改配置"}
        sweep = self._sweep
        try:
            start = float(params.get("start_mhz", sweep.start_mhz))
            stop = float(params.get("stop_mhz", sweep.stop_mhz))
            points = int(params.get("points", sweep.points))
            ifbw = int(params.get("ifbw_hz", params.get("ifbw", sweep.ifbw_hz)))
            power = float(params.get("power_dbm", params.get("power", sweep.power_dbm)))
            dwell = int(params.get("dwell_us", params.get("dwell", sweep.dwell_us)))
            ifft = int(params.get("ifft_points", sweep.ifft_points))
            time_window = float(params.get("time_window_ns", params.get("time_window", sweep.time_window_ns)))
            dewow_enabled = bool(params.get("dewow_enabled", sweep.dewow_enabled))
            dewow_window = float(params.get("dewow_window_ns", params.get("dewow_window", sweep.dewow_window_ns)))
            interval = float(params.get("interval_s", self._interval_s))
            count = int(params.get("count", self._count))
        except (TypeError, ValueError) as exc:
            return {"ok": False, "error": f"参数类型解析错误: {exc}"}
        # 校验范围与 MainWindow._handle_set_params_command 同口径。
        if stop <= start:
            return {"ok": False, "error": f"终止频率 ({stop} MHz) 必须大于起始频率 ({start} MHz)"}
        if not 2 <= points <= 4501:
            return {"ok": False, "error": f"频点数 ({points}) 超出范围 [2, 4501]"}
        if not 6 <= ifbw <= 50000:
            return {"ok": False, "error": f"中频带宽 ({ifbw} Hz) 超出范围 [6, 50000]"}
        if not -40.0 <= power <= 0.0:
            return {"ok": False, "error": f"发射功率 ({power} dBm) 超出范围 [-40, 0]"}
        if not 0 <= dwell <= 10239:
            return {"ok": False, "error": f"驻留时间 ({dwell} us) 超出范围 [0, 10239]"}
        if not 256 <= ifft <= 65536:
            return {"ok": False, "error": f"IFFT 点数 ({ifft}) 超出范围 [256, 65536]"}
        if not 1.0 <= time_window <= 1000.0:
            return {"ok": False, "error": f"时间窗 ({time_window} ns) 必须在 [1, 1000] 内"}
        if not 0 < dewow_window <= 1000.0:
            return {"ok": False, "error": f"Dewow 窗长 ({dewow_window} ns) 必须在 (0, 1000] 内"}
        if interval < 0:
            return {"ok": False, "error": f"采集间隔 ({interval}s) 不能为负"}
        if count < 0:
            return {"ok": False, "error": f"采集道数 ({count}) 不能为负"}
        self._sweep = air.SweepConfig(
            start_mhz=start,
            stop_mhz=stop,
            points=points,
            ifbw_hz=ifbw,
            power_dbm=power,
            dwell_us=dwell,
            ifft_points=ifft,
            time_window_ns=time_window,
            dewow_window_ns=dewow_window,
            dewow_enabled=dewow_enabled,
        )
        self._interval_s = interval
        self._count = count
        self._log(
            f"远程参数更新成功: {start:g}-{stop:g} MHz, {points} 点, IFBW={ifbw} Hz, "
            f"功率={power:g} dBm, interval={interval}s, count={count}"
        )
        return {"ok": True, "sweep": asdict(self._sweep), "interval_s": interval, "count": count}

    # ---- 远程校准编排（cal_*：MainWindow 会话编排的无界面等价实现）----
    #
    # 与 MainWindow._handle_calibration_command 一族逐动作对齐：互斥判据
    # （R-C-14）、回程消息（§5.4 calibration_*）、会话状态机、解算落盘
    # （uav_gpr_solt_calibration_v2）、激活写回（R-C-16/R-C-19）语义一致；
    # 差异仅是：扫频回退取自身 _sweep、落盘取 output_root、状态写回自身
    # 字段（后续采集任务经 task spec 生效）、日志经 self._log。

    def _cmd_cal_begin(self, params: dict) -> dict[str, object]:
        return self._handle_calibration_command("cal_begin", params)

    def _cmd_cal_capture(self, params: dict) -> dict[str, object]:
        return self._handle_calibration_command("cal_capture", params)

    def _cmd_cal_finish(self, params: dict) -> dict[str, object]:
        return self._handle_calibration_command("cal_finish", params)

    def _cmd_cal_activate(self, params: dict) -> dict[str, object]:
        return self._handle_calibration_command("cal_activate", params)

    def _cmd_cal_cancel(self, params: dict) -> dict[str, object]:
        return self._handle_calibration_command("cal_cancel", params)

    def _cmd_cal_status(self, params: dict) -> dict[str, object]:
        return self._handle_calibration_command("cal_status", params)

    def _cmd_cal_import_file(self, params: dict) -> dict[str, object]:
        return self._handle_calibration_command("cal_import_file", params)

    def _cmd_cal_export_file(self, params: dict) -> dict[str, object]:
        return self._handle_calibration_command("cal_export_file", params)

    def _cmd_cal_query_active(self, params: dict) -> dict[str, object]:
        return self._handle_calibration_command("cal_query_active", params)

    def _cmd_cal_clear_active(self, params: dict) -> dict[str, object]:
        return self._handle_calibration_command("cal_clear_active", params)

    def _handle_calibration_command(self, action: str, params: dict) -> dict[str, object]:
        """地面校准命令入口（ack 回执 + calibration_* 回程双通道）。

        全部动作先过 :meth:`_calibration_acquisition_conflict`（R-C-14）；
        拒绝/失败时既回 ``calibration_result``（地面向导主通道，与 GUI 一致），
        ack 也带 ``ok=false``（供未来地面消费，现有向导忽略 ack）。
        """
        params_dict = params if isinstance(params, dict) else {}
        cal_id = str(params_dict.get("cal_id") or "")
        conflict = self._calibration_acquisition_conflict(action, cal_id=cal_id)
        if conflict:
            self._log(conflict)
            self._send_calibration_message(
                "calibration_result", cal_id, status="rejected", reason=conflict
            )
            return {"ok": False, "error": conflict}
        if action == "cal_import_file":
            return self._handle_cal_import_file(params_dict)
        if action == "cal_export_file":
            return self._handle_cal_export_file(params_dict)
        if action == "cal_query_active":
            return self._handle_cal_query_active(params_dict)
        if action == "cal_clear_active":
            return self._handle_cal_clear_active(params_dict)

        accepted = True
        if action == "cal_begin":
            accepted = self._begin_calibration_session(params_dict)
        elif action == "cal_capture":
            step = str(params_dict.get("step") or "")
            recapture = bool(params_dict.get("recapture", False))
            request_id = str(params_dict.get("request_id") or "")
            accepted = self._command_calibration_capture(
                cal_id, step, recapture=recapture, request_id=request_id
            )
        elif action == "cal_finish":
            accepted = self._command_calibration_finish(cal_id)
        elif action == "cal_activate":
            accepted = self._command_calibration_activate(cal_id)
        elif action == "cal_cancel":
            accepted = self._command_calibration_cancel(
                cal_id, str(params_dict.get("reason") or "地面请求取消")
            )
        elif action == "cal_status":
            self._command_calibration_status(cal_id)
        if isinstance(accepted, dict):
            return accepted
        return {"ok": bool(accepted)}

    def _calibration_acquisition_conflict(self, action: str, cal_id: str = "") -> str | None:
        """校准/采集互斥判据（R-C-14：与 MainWindow 同规则无界面版）。"""
        if action in ("cal_query_active", "cal_export_file", "cal_status"):
            return None
        state = self.acquisition.state
        worker = self._legacy_worker
        worker_running = worker is not None and worker.isRunning()
        if worker_running or state in (
            air.SERVICE_STATE_ACQUIRING,
            air.SERVICE_STATE_STOPPING,
            air.SERVICE_STATE_SHUTTING_DOWN,
        ):
            return f"{action} 被拒绝: 采集任务运行中或服务正在收尾（state={state}）"
        session = self._calibration_session
        session_open = (
            session is not None
            and session.state
            in (
                air.CalibrationSession.STATE_PLANNED,
                air.CalibrationSession.STATE_IN_PROGRESS,
                air.CalibrationSession.STATE_SOLVED,
            )
        )
        if not session_open:
            return None
        if cal_id and session.cal_id == cal_id:
            return None
        return (
            f"{action} 被拒绝: 校准会话进行中"
            f"（cal_id={session.cal_id}, state={session.state}）"
        )

    def _send_calibration_message(
        self, message_type: str, cal_id: str, payload: bytes = b"", **fields: object
    ) -> None:
        """校准回程（§5.4）：run_id 沿用当前连接身份，不触发带内身份切换。"""
        header: dict[str, object] = {"type": message_type, "cal_id": cal_id}
        for key, value in fields.items():
            if value is not None:
                header[key] = value
        try:
            self.sender.enqueue_calibration(header, payload)
        except Exception as exc:  # noqa: BLE001 - 回程失败必须可见，不影响采集
            self._log(f"校准回程入队失败: {message_type} cal_id={cal_id}: {exc}")

    def _calibration_config_from_snapshot(self, snapshot: dict[str, object]) -> air.SweepConfig:
        """由地面 cal_begin 的 SweepConfig 快照构建扫频配置（缺省回退自身 _sweep）。"""

        def _number(key: str, fallback: float) -> float:
            value = snapshot.get(key, fallback)
            try:
                return float(value)  # type: ignore[arg-type]
            except (TypeError, ValueError):
                return float(fallback)

        base = self._sweep
        return air.SweepConfig(
            start_mhz=_number("start_mhz", base.start_mhz),
            stop_mhz=_number("stop_mhz", base.stop_mhz),
            points=int(_number("points", base.points)),
            ifbw_hz=int(_number("ifbw_hz", base.ifbw_hz)),
            power_dbm=_number("power_dbm", base.power_dbm),
            dwell_us=int(_number("dwell_us", base.dwell_us)),
            ifft_points=base.ifft_points,
            time_window_ns=base.time_window_ns,
            dewow_enabled=base.dewow_enabled,
            dewow_window_ns=base.dewow_window_ns,
        )

    def _begin_calibration_session(self, params: dict[str, object]) -> bool:
        cal_id = str(params.get("cal_id") or "").strip()
        mode = str(params.get("mode") or air.CALIBRATION_MODE_OSL)
        if not cal_id:
            self._send_calibration_message(
                "calibration_result", "", status="rejected", reason="cal_begin 缺少 cal_id"
            )
            return False
        with self._calibration_lock:
            existing = self._calibration_session
            if (
                existing is not None
                and existing.state
                in (
                    air.CalibrationSession.STATE_PLANNED,
                    air.CalibrationSession.STATE_IN_PROGRESS,
                    air.CalibrationSession.STATE_SOLVED,
                )
            ):
                self._send_calibration_message(
                    "calibration_result",
                    cal_id,
                    status="rejected",
                    reason=f"已有校准会话进行中（cal_id={existing.cal_id}）",
                )
                return False
            try:
                repeats = air.validate_repeats(params.get("repeats", air.REPEATS_DEFAULT))
                if mode == air.CALIBRATION_MODE_OSL:
                    plan = air.build_osl_plan(cal_id, repeats, {})
                elif mode == air.CALIBRATION_MODE_BACKGROUND:
                    plan = air.build_background_plan(cal_id, repeats, {})
                else:
                    raise ValueError(f"未知校准模式: {mode!r}")
                cfg = self._calibration_config_from_snapshot(
                    params.get("config") if isinstance(params.get("config"), dict) else {}
                )
                if cfg.stop_mhz <= cfg.start_mhz:
                    raise ValueError("Stop MHz 必须大于 Start MHz")
                capture_dir = self.output_root / "_calibration" / air.safe_cal_dir_name(cal_id)
                session = air.CalibrationSession(
                    plan, _BlackboxCalibrationCapturerProxy(self, cal_id), capture_dir=capture_dir
                )
                session.config_snapshot = {
                    "start_mhz": cfg.start_mhz,
                    "stop_mhz": cfg.stop_mhz,
                    "points": cfg.points,
                    "ifbw_hz": cfg.ifbw_hz,
                    "power_dbm": cfg.power_dbm,
                    "dwell_us": cfg.dwell_us,
                }
            except Exception as exc:  # noqa: BLE001 - 计划非法：拒绝并回传原因
                self._log(f"cal_begin 计划非法: {exc}")
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected", reason=str(exc)
                )
                return False
            self._calibration_session = session
        self._log(f"校准会话已建立: cal_id={cal_id} mode={mode} repeats={repeats}")
        self._send_calibration_message(
            "calibration_result",
            cal_id,
            status="planned",
            mode=mode,
            steps=[step.name for step in plan.steps],
            repeats=repeats,
        )
        return True

    def _handle_cal_import_file(self, params: dict[str, object]) -> dict[str, object]:
        filename = str(params.get("filename") or "").strip()
        content = params.get("content")
        activate = bool(params.get("activate", True))
        request_id = str(params.get("request_id") or "").strip()

        safe_filename = Path(filename).name.strip()
        if not safe_filename or safe_filename in (".", "..") or not all(c.isalnum() or c in "._-" for c in safe_filename):
            return {"ok": False, "error": f"非法文件名: {filename!r}"}
        if not content:
            return {"ok": False, "error": "cal_import_file 缺少 content"}

        valid, error_msg, parsed_data = air.validate_calibration_content(content)
        if not valid:
            return {"ok": False, "error": f"校准数据校验失败: {error_msg}"}

        from librevna.librevna_usb_s11 import OSLCalibration

        try:
            p1_cal, p2_cal, through_meas = OSLCalibration.load_components(parsed_data)
        except Exception as exc:
            return {"ok": False, "error": f"校准解析解算失败: {exc}"}

        imports_dir = self.output_root / "_calibration" / "imports"
        try:
            imports_dir.mkdir(parents=True, exist_ok=True)
            target_path = imports_dir / safe_filename
            tmp_path = imports_dir / f".{safe_filename}.tmp"
            content_str = (
                json.dumps(parsed_data, ensure_ascii=False, indent=2)
                if isinstance(content, dict)
                else str(content)
            )
            tmp_path.write_text(content_str, encoding="utf-8")
        except OSError as exc:
            return {"ok": False, "error": f"校准文件写入临时文件失败: {exc}"}

        backup_bytes: bytes | None = None
        if target_path.exists():
            try:
                backup_bytes = target_path.read_bytes()
            except OSError as exc:
                if tmp_path.exists():
                    tmp_path.unlink(missing_ok=True)
                return {"ok": False, "error": f"备份既有校准文件失败: {exc}"}

        # 先完成文件 replace 就位并保留备份
        try:
            os.replace(tmp_path, target_path)
        except OSError as exc:
            if tmp_path.exists():
                tmp_path.unlink(missing_ok=True)
            return {"ok": False, "error": f"校准文件写入替换失败: {exc}"}

        if activate:
            old_config_path = self.config.calibration_path
            with self._config_lock:
                self.config.calibration_path = str(target_path)
                saved = self.save_config()
                if not saved:
                    self.config.calibration_path = old_config_path
                    if backup_bytes is not None:
                        try:
                            target_path.write_bytes(backup_bytes)
                        except OSError:
                            pass
                    else:
                        target_path.unlink(missing_ok=True)
                    return {"ok": False, "error": "配置持久化保存失败，已回滚恢复原文件与配置"}

            with self._calibration_lock:
                self._session_calibration = p1_cal
                self._session_calibration_p2 = p2_cal
                self._active_through = through_meas
                self._active_calibration_path = target_path
                cal_id_str = str(parsed_data.get("cal_id") or target_path.stem)
                self.active_calibration_id = cal_id_str
                self._active_osl_id = cal_id_str

                if self._background is not None:
                    self._background = None
                    self._background_frequency_hz = None
                    self._background_cal_id = None
                    self._log("新 OSL 校准已导入生效：先前的背景校准已显式失效")

            self._log(
                f"校准文件导入并生效成功: {safe_filename}（P1: {len(p1_cal.points)} 点，"
                f"P2: {len(p2_cal.points) if p2_cal else 0} 点）"
            )

        return {
            "ok": True,
            "filename": safe_filename,
            "activated": activate,
            "cal_id": str(parsed_data.get("cal_id") or safe_filename),
            "ports": ["P1", "P2"] if p2_cal else ["P1"],
            "points": len(p1_cal.points),
            "valid_band_hz": [float(p1_cal.points[0].frequency), float(p1_cal.points[-1].frequency)],
            "format": str(parsed_data.get("format") or "uav_gpr_solt_calibration_v2"),
            "has_through": through_meas is not None,
            "request_id": request_id,
        }

    def _handle_cal_export_file(self, params: dict[str, object]) -> dict[str, object]:
        target = str(params.get("target") or "osl").lower()
        request_id = str(params.get("request_id") or "").strip()
        with self._calibration_lock:
            if target == "background":
                if self._background is None:
                    return {"ok": False, "error": "当前无生效的背景校准可导出"}
                payload = {
                    "format": "uav_gpr_background_v1",
                    "cal_id": getattr(self, "_background_cal_id", "bg_active") or "bg_active",
                    "created_at": datetime.now().isoformat(timespec="seconds"),
                    "frequency_hz": (
                        [float(f) for f in self._background_frequency_hz]
                        if self._background_frequency_hz is not None
                        else []
                    ),
                    "background_real": [float(v.real) for v in self._background],
                    "background_imag": [float(v.imag) for v in self._background],
                }
                content_str = json.dumps(payload, ensure_ascii=False, indent=2)
                filename = f"background_{getattr(self, '_background_cal_id', 'active') or 'active'}.json"
                return {
                    "ok": True,
                    "filename": filename,
                    "content": content_str,
                    "format": "uav_gpr_background_v1",
                    "cal_id": getattr(self, "_background_cal_id", "") or "",
                    "request_id": request_id,
                }
            else:
                path = self._active_calibration_path
                if path and Path(path).exists():
                    try:
                        content_str = Path(path).read_text(encoding="utf-8")
                        data = json.loads(content_str)
                        fmt = str(data.get("format") or "uav_gpr_solt_calibration_v2")
                        return {
                            "ok": True,
                            "filename": Path(path).name,
                            "content": content_str,
                            "format": fmt,
                            "cal_id": self._active_osl_id or self.active_calibration_id or Path(path).stem,
                            "request_id": request_id,
                        }
                    except Exception as exc:
                        return {"ok": False, "error": f"读取生效校准文件失败: {exc}"}
                elif self._session_calibration is not None:
                    pts = [
                        {
                            "frequency": float(p.frequency),
                            "directivity": str(p.directivity),
                            "reflection_tracking": str(p.reflection_tracking),
                            "source_match": str(p.source_match),
                        }
                        for p in self._session_calibration.points
                    ]
                    payload = {
                        "format": "uav_gpr_solt_calibration_v2",
                        "cal_id": self._active_osl_id or self.active_calibration_id or "mem_osl",
                        "points": pts,
                    }
                    content_str = json.dumps(payload, ensure_ascii=False, indent=2)
                    return {
                        "ok": True,
                        "filename": f"{self._active_osl_id or self.active_calibration_id or 'osl'}.json",
                        "content": content_str,
                        "format": "uav_gpr_solt_calibration_v2",
                        "cal_id": self._active_osl_id or self.active_calibration_id or "mem_osl",
                        "request_id": request_id,
                    }
                else:
                    return {"ok": False, "error": "当前无生效的 OSL 校准可导出"}

    def _handle_cal_query_active(self, params: dict[str, object]) -> dict[str, object]:
        with self._calibration_lock:
            osl_p1 = self._active_osl_p1()
            osl_p2 = self._active_osl_p2()
            active_osl_cal_id = self._active_osl_id or (self.active_calibration_id if osl_p1 is not None else None)
            active_osl: dict[str, object] = {
                "active": osl_p1 is not None,
                "cal_id": active_osl_cal_id,
                "path": str(self._active_calibration_path) if self._active_calibration_path else None,
                "filename": self._active_calibration_path.name if self._active_calibration_path else None,
                "valid_band_hz": (
                    [float(osl_p1.points[0].frequency), float(osl_p1.points[-1].frequency)]
                    if osl_p1
                    else None
                ),
                "points": len(osl_p1.points) if osl_p1 else 0,
                "ports": (["P1", "P2"] if osl_p2 else ["P1"]) if osl_p1 else [],
                "has_through": getattr(self, "_active_through", None) is not None,
            }
            active_bg: dict[str, object] = {
                "active": self._background is not None,
                "cal_id": getattr(self, "_background_cal_id", None),
                "points": len(self._background) if self._background is not None else 0,
                "frequency_range_mhz": (
                    [
                        float(self._background_frequency_hz[0] / 1e6),
                        float(self._background_frequency_hz[-1] / 1e6),
                    ]
                    if self._background_frequency_hz is not None
                    else None
                ),
            }
            session = self._calibration_session
            if session is not None and session.state in (
                air.CalibrationSession.STATE_PLANNED,
                air.CalibrationSession.STATE_IN_PROGRESS,
                air.CalibrationSession.STATE_SOLVED,
            ):
                pending = session.next_pending_step()
                is_busy = bool(self._calibration_thread is not None and self._calibration_thread.is_alive())
                current_step = getattr(self, "_current_capture_step", None) or (
                    pending.name if pending else None
                )
                steps_status = {name: p.state for name, p in session._progress.items()}
                in_prog: dict[str, object] = {
                    "active": True,
                    "cal_id": session.cal_id,
                    "mode": session.plan.mode,
                    "state": session.state,
                    "busy": is_busy,
                    "current_step": current_step,
                    "active_request_id": getattr(self, "_current_capture_request_id", None),
                    "steps_status": steps_status,
                    "steps_done": sum(1 for p in session._progress.values() if p.state == "done"),
                    "total_steps": len(session.plan.steps),
                }
            else:
                in_prog = {
                    "active": False,
                    "cal_id": None,
                    "mode": None,
                    "state": None,
                    "busy": False,
                    "current_step": None,
                    "active_request_id": None,
                    "steps_status": {},
                    "steps_done": 0,
                    "total_steps": 0,
                }
            capabilities = {
                "step_done": True,
                "cal_import_file": True,
                "cal_export_file": True,
                "recapture": True,
            }
            return {
                "ok": True,
                "active_osl": active_osl,
                "active_background": active_bg,
                "in_progress_session": in_prog,
                "capabilities": capabilities,
            }

    def _handle_cal_clear_active(self, params: dict[str, object]) -> dict[str, object]:
        target = str(params.get("target") or "all").lower()
        with self._calibration_lock:
            if target in ("all", "osl"):
                self._session_calibration = None
                self._session_calibration_p2 = None
                self._active_through = None
                self._active_calibration_path = None
                self.active_calibration_id = None
                self._active_osl_id = None
                with self._config_lock:
                    self.config.calibration_path = ""
                    self.save_config()
                self._background = None
                self._background_frequency_hz = None
                self._background_cal_id = None
                self._log("已清除生效的 OSL 校准与背景")
            elif target == "background":
                self._background = None
                self._background_frequency_hz = None
                self._background_cal_id = None
                if self._session_calibration is None:
                    self.active_calibration_id = None
                self._log("已清除生效的背景校准（OSL 校准保持有效）")
            else:
                return {"ok": False, "error": f"未知清除目标: {target!r}（可选 all|osl|background）"}
        return {"ok": True, "cleared": target, "target": target}

    def _command_calibration_capture(
        self, cal_id: str, step_name: str, recapture: bool = False, request_id: str = ""
    ) -> bool:
        with self._calibration_lock:
            if request_id and request_id in self._recent_requests:
                cached = dict(self._recent_requests[request_id])
                cached.pop("cal_id", None)
                self._send_calibration_message("calibration_result", cal_id, **cached)
                return True

            session = self._calibration_session
            if session is None or session.cal_id != cal_id:
                self._send_calibration_message(
                    "calibration_result",
                    cal_id,
                    status="rejected",
                    step=step_name,
                    request_id=request_id,
                    reason="无进行中的校准会话或 cal_id 不匹配（防跨会话错配）",
                )
                return False

            if self._calibration_thread is not None and self._calibration_thread.is_alive():
                if self._current_capture_step == step_name and self._current_capture_request_id == request_id:
                    return True
                self._send_calibration_message(
                    "calibration_result",
                    cal_id,
                    status="rejected",
                    step=step_name,
                    request_id=request_id,
                    reason="上一步骤采集仍在进行",
                )
                return False

            if session.state in (
                air.CalibrationSession.STATE_ACTIVATED,
                air.CalibrationSession.STATE_FAILED,
                air.CalibrationSession.STATE_CANCELLED,
            ):
                self._send_calibration_message(
                    "calibration_result",
                    cal_id,
                    status="rejected",
                    step=step_name,
                    request_id=request_id,
                    reason=f"会话状态 {session.state} 不允许采集",
                )
                return False

            if recapture:
                progress = session._progress.get(step_name)
                if progress is None:
                    self._send_calibration_message(
                        "calibration_result",
                        cal_id,
                        status="rejected",
                        step=step_name,
                        request_id=request_id,
                        reason=f"计划中不存在步骤 {step_name!r}",
                    )
                    return False
                if progress.state == "done":
                    pass
                elif progress.state in ("pending", "capturing"):
                    pending = session.next_pending_step()
                    if pending is None or pending.name != step_name:
                        self._send_calibration_message(
                            "calibration_result",
                            cal_id,
                            status="rejected",
                            step=step_name,
                            request_id=request_id,
                            reason=f"步骤 {step_name!r} 尚未进行首次采集，不可重采（下一待采步骤为 {pending.name if pending else '无'}）",
                        )
                        return False
                else:
                    self._send_calibration_message(
                        "calibration_result",
                        cal_id,
                        status="rejected",
                        step=step_name,
                        request_id=request_id,
                        reason=f"步骤 {step_name!r} 处于终态，无法重采",
                    )
                    return False
            else:
                progress = session._progress.get(step_name)
                if progress and progress.state == "done":
                    self._send_calibration_message(
                        "calibration_result",
                        cal_id,
                        status="rejected",
                        step=step_name,
                        request_id=request_id,
                        reason=f"步骤 {step_name!r} 已完成，重采需指定 recapture=True",
                    )
                    return False
                if session.state == air.CalibrationSession.STATE_SOLVED:
                    self._send_calibration_message(
                        "calibration_result",
                        cal_id,
                        status="rejected",
                        step=step_name,
                        request_id=request_id,
                        reason="所有步骤已解算，如需重新采集请指定 recapture=True",
                    )
                    return False
                pending = session.next_pending_step()
                if pending is None or pending.name != step_name:
                    self._send_calibration_message(
                        "calibration_result",
                        cal_id,
                        status="rejected",
                        step=step_name,
                        request_id=request_id,
                        reason=f"下一待采步骤为 {pending.name if pending else '无'}，收到 {step_name!r}",
                    )
                    return False

            self._current_capture_step = step_name
            self._current_capture_request_id = request_id
            thread = threading.Thread(
                target=self._calibration_capture_driver,
                args=(session, step_name),
                kwargs={"recapture": recapture, "request_id": request_id},
                name=f"uav-gpr-cal-capture-{step_name}",
                daemon=True,
            )
            self._calibration_thread = thread
        self._log(f"校准步骤开始采集: cal_id={cal_id} step={step_name} (recapture={recapture})")
        thread.start()
        return True

    def _calibration_capture_driver(
        self,
        session: air.CalibrationSession,
        step_name: str,
        recapture: bool = False,
        request_id: str = "",
    ) -> None:
        """校准采集驱动线程：执行一个步骤的 repeats 条采集（会话状态机内）。"""
        success = False
        try:
            step = session.plan.step(step_name)

            def _on_capture(frequency: np.ndarray, raw: np.ndarray, index: int, of: int) -> None:
                payload = air._calibration_capture_npz_bytes(frequency, raw)
                self._send_calibration_message(
                    "calibration_capture",
                    session.cal_id,
                    payload,
                    step=step_name,
                    index=index,
                    of=of,
                    channel=step.channel,
                    points=int(frequency.size),
                    payload_format="npz",
                    request_id=request_id,
                )

            session.capture(step_name, recapture=recapture, on_capture=_on_capture)
            self._log(f"校准步骤采集完成: cal_id={session.cal_id} step={step_name}")
            success = True
        except Exception as exc:  # noqa: BLE001 - 步骤失败：会话已判 failed
            self._log(f"校准步骤采集失败: cal_id={session.cal_id} step={step_name}: {exc}")
            self._send_calibration_message(
                "calibration_result",
                session.cal_id,
                status="failed",
                step=step_name,
                request_id=request_id,
                reason=str(exc),
            )
        close_ok = True
        try:
            proxy = session.capturer
            close = getattr(proxy, "close_current", None)
            if callable(close):
                try:
                    close(step_name)
                except Exception as close_exc:
                    close_ok = False
                    self._log(f"步骤 {step_name} 释放底层采集器异常: {close_exc}")
        finally:
            with self._calibration_lock:
                self._calibration_thread = None
                self._current_capture_step = None
                self._current_capture_request_id = None

        if success and close_ok:
            steps_done = sum(1 for p in session._progress.values() if p.state == "done")
            next_step = session.next_pending_step().name if session.next_pending_step() else None
            result_payload = {
                "status": "step_done",
                "step": step_name,
                "request_id": request_id,
                "next_step": next_step,
                "steps_done": steps_done,
                "total_steps": len(session.plan.steps),
            }
            if request_id:
                self._record_completed_request(request_id, dict(result_payload, cal_id=session.cal_id))
            self._send_calibration_message("calibration_result", session.cal_id, **result_payload)
        elif not close_ok:
            self._send_calibration_message(
                "calibration_result",
                session.cal_id,
                status="failed",
                step=step_name,
                request_id=request_id,
                reason=f"步骤 {step_name} 采集后释放底层采集器异常，不可继续",
            )

    def _command_calibration_finish(self, cal_id: str) -> bool:
        with self._calibration_lock:
            session = self._calibration_session
            if session is None or session.cal_id != cal_id:
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected",
                    reason="无进行中的校准会话或 cal_id 不匹配",
                )
                return False
            if self._calibration_thread is not None and self._calibration_thread.is_alive():
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected",
                    reason="校准采集仍在进行，不能解算",
                )
                return False
            try:
                result = session.solve()
            except Exception as exc:  # noqa: BLE001 - 解算失败：会话已判 failed
                self._log(f"校准解算失败: cal_id={cal_id}: {exc}")
                self._send_calibration_message(
                    "calibration_result", cal_id, status="failed", reason=str(exc)
                )
                return False
        self._log(f"校准解算完成: cal_id={cal_id}")
        # 每步回传校正后质量指标（R-C-08：逐条校正后的重复 RMS/最大绝对误差）。
        self._send_step_quality(session, result)
        # 解算结果落盘（复用 uav_gpr_solt_calibration_v2 格式，R-C-09）。
        path: Path | None = None
        if session.plan.mode == air.CALIBRATION_MODE_OSL:
            measurements = self._measurements_from_result(session, result)
            cfg = self._calibration_config_from_snapshot(getattr(session, "config_snapshot", {}) or {})
            path = session.capture_dir / f"{air.safe_cal_dir_name(cal_id)}.json" if session.capture_dir else None
            if path is not None:
                try:
                    air.write_calibration_file(path, measurements, cfg)
                except Exception as exc:  # noqa: BLE001
                    self._log(f"校准文件写盘失败: {exc}")
                    path = None
            if path is not None:
                self._active_calibration_path = path
        summary = json.dumps(result.get("profile_summary", {}), ensure_ascii=False)
        band = result.get("valid_band_hz") or (None, None)
        self._send_calibration_message(
            "calibration_result",
            cal_id,
            status="solved",
            path=str(path) if path else "",
            format="uav_gpr_solt_calibration_v2",
            profile_summary=summary,
            valid_band_hz=f"{band[0]:g}-{band[1]:g}" if band[0] is not None else "",
        )
        return True

    def _measurements_from_result(
        self, session: air.CalibrationSession, result: dict[str, object]
    ) -> dict[str, air.CalibrationMeasurement]:
        """解算结果 → 六步标准件均值（写 uav_gpr_solt_calibration_v2 用）。"""
        measurements: dict[str, air.CalibrationMeasurement] = {}
        means = session.step_means()
        for name, mean in means.items():
            frequency = session.step_frequency(name)
            points = [
                (float(f), {"S11": complex(mean[0][index]), "S22": complex(mean[1][index])})
                for index, f in enumerate(frequency)
            ]
            measurements[name] = air.CalibrationMeasurement.from_sparameter_points(
                points, primary=air.OSL_STEP_CHANNEL[name]
            )
        return measurements

    def _send_step_quality(self, session: air.CalibrationSession, result: dict[str, object]) -> None:
        if session.plan.mode == air.CALIBRATION_MODE_BACKGROUND:
            progress = session.step_progress(air.BACKGROUND_STEP_NAME)
            captures = np.asarray(progress.captures, dtype=np.complex128)
            sample_count = int(captures.shape[0]) if captures.ndim == 3 else 0
            if sample_count >= 2 and captures.ndim == 3:
                mean = np.mean(captures, axis=0)
                errors = np.abs(captures - mean[np.newaxis, ...])
                repeat_rms: float | None = float(np.sqrt(np.mean(np.square(errors))))
                max_abs: float | None = float(np.max(errors))
            else:
                repeat_rms = None
                max_abs = None
            self._send_calibration_message(
                "calibration_quality",
                session.cal_id,
                step=air.BACKGROUND_STEP_NAME,
                repeat_rms=repeat_rms,
                max_abs_err=max_abs,
                sample_count=sample_count,
                note=None if sample_count >= 2 else "样本不足",
            )
            return
        p1: air.OSLTerms = result["p1_terms"]  # type: ignore[assignment]
        p2: air.OSLTerms = result["p2_terms"]  # type: ignore[assignment]
        for step_name in air.OSL_STEP_NAMES:
            port_terms = p1 if step_name.startswith("P1_") else p2
            label = step_name.split("_", 1)[1].lower()
            metrics = port_terms.quality_metrics
            self._send_calibration_message(
                "calibration_quality",
                session.cal_id,
                step=step_name,
                repeat_rms=metrics.get(f"{label}_repeat_rms_abs_error"),
                max_abs_err=metrics.get(f"{label}_repeat_max_abs_error"),
                fit_max_abs_error=metrics.get(f"{label}_fit_max_abs_error"),
                sample_count=metrics.get(f"{label}_sample_count"),
                note="样本不足" if metrics.get(f"{label}_insufficient_samples") else None,
            )

    def _active_osl_p1(self) -> object | None:
        """当前生效的 P1 OSL 校准（会话激活优先，外部文件兜底；背景域规则用）。"""
        if self._session_calibration is not None:
            return self._session_calibration
        path_text = str(self._calibration_path or "").strip()
        if not path_text:
            return None
        try:
            return air.OSLCalibration.from_file(path_text, port=1)
        except Exception as exc:  # noqa: BLE001 - 外部文件损坏不炸激活流程
            self._log(f"外部校准文件加载失败（背景按原始域保存）: {exc}")
            return None

    def _active_osl_p2(self) -> object | None:
        """当前生效的 P2 OSL 校准（会话激活优先，外部文件兜底）。"""
        if self._session_calibration_p2 is not None:
            return self._session_calibration_p2
        path_text = str(self._calibration_path or "").strip()
        if not path_text:
            return None
        try:
            return air.OSLCalibration.from_file(path_text, port=2)
        except Exception:
            return None

    def _command_calibration_activate(self, cal_id: str) -> bool:
        with self._calibration_lock:
            session = self._calibration_session
            if session is None or session.cal_id != cal_id:
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected",
                    reason="无进行中的校准会话或 cal_id 不匹配（防跨会话错配）",
                )
                return False
            try:
                activation = session.activate()
            except Exception as exc:  # noqa: BLE001 - 激活失败：原因回传
                self._log(f"校准激活失败: cal_id={cal_id}: {exc}")
                self._send_calibration_message(
                    "calibration_result", cal_id, status="failed", reason=str(exc)
                )
                return False
            # ---- 接线：生效组件写回黑匣子自身状态（R-C-16 无界面版）----
            # 后续采集任务经 task spec（session_calibration/background/
            # calibration_id）生效。
            if session.plan.mode == air.CALIBRATION_MODE_OSL:
                cal_path = self._active_calibration_path
                if cal_path:
                    with self._config_lock:
                        old_cal_path = self.config.calibration_path
                        self.config.calibration_path = str(cal_path)
                        saved = self.save_config()
                        if not saved:
                            self.config.calibration_path = old_cal_path
                            session.rollback_activation()
                            self._log(f"校准激活持久化失败: {cal_path}，已回滚会话至已解算状态")
                            self._send_calibration_message(
                                "calibration_result",
                                cal_id,
                                status="failed",
                                reason="配置文件回写失败，校准未持久化",
                            )
                            return False

                p1_terms: air.OSLTerms = activation["p1_terms"]  # type: ignore[assignment]
                p2_terms: air.OSLTerms = activation["p2_terms"]  # type: ignore[assignment]
                self._session_calibration = air.terms_to_osl_calibration(p1_terms, cal_path)
                self._session_calibration_p2 = air.terms_to_osl_calibration(p2_terms, cal_path)
                self.active_calibration_id = cal_id
                self._active_osl_id = cal_id

                if self._background is not None:
                    self._background = None
                    self._background_frequency_hz = None
                    self._background_cal_id = None
                    self._log(f"新 OSL 校准已激活（cal_id={cal_id}）：先前的背景校准已显式失效")
                else:
                    self._log(
                        f"远程校准已激活: cal_id={cal_id}（P1/P2 双反射 OSL，"
                        f"{len(p1_terms.frequency_hz)} 点）"
                    )
            else:
                frequency = np.asarray(activation["background_frequency_hz"], dtype=np.float64)
                raw_mean = np.asarray(activation["background"], dtype=np.complex128)
                # R-C-19 域约束：背景保存为"当前生效 OSL 域"的 S11（与既有
                # capture_background 语义一致），绝不把单通道背景广播到双通道。
                calibration_p1 = self._active_osl_p1()
                if calibration_p1 is not None:
                    background = np.array(
                        [calibration_p1.correct(float(f), complex(v)) for f, v in zip(frequency, raw_mean)],
                        dtype=np.complex128,
                    )
                else:
                    background = raw_mean.copy()
                self._background = background
                self._background_frequency_hz = np.array(frequency, dtype=np.float64, copy=True)
                self._background_cal_id = cal_id
                if self._session_calibration is None:
                    self.active_calibration_id = cal_id
                self._log(f"远程空采背景已激活: {len(background)} 点（cal_id={cal_id}）")
            summary = json.dumps(
                (session.solved_result() or {}).get("profile_summary", {}), ensure_ascii=False
            )
        self._log(f"校准已激活: cal_id={cal_id}")
        act_fields: dict[str, object] = {
            "status": "activated",
            "mode": session.plan.mode,
            "profile_summary": summary,
            "calibration_id": cal_id,
        }
        if session.plan.mode == air.CALIBRATION_MODE_OSL:
            act_fields["active_osl_id"] = self._active_osl_id or cal_id
            if self._session_calibration is not None:
                act_fields["points"] = len(self._session_calibration.points)
                act_fields["ports"] = ["P1", "P2"] if self._session_calibration_p2 else ["P1"]
                p_pts = self._session_calibration.points
                act_fields["valid_band_hz"] = f"{p_pts[0].frequency:g}-{p_pts[-1].frequency:g}"
        else:
            act_fields["background_cal_id"] = self._background_cal_id or cal_id
            if self._background is not None:
                act_fields["points"] = len(self._background)

        self._send_calibration_message("calibration_result", cal_id, **act_fields)
        return True

    def _command_calibration_cancel(self, cal_id: str, reason: str) -> bool:
        with self._calibration_lock:
            session = self._calibration_session
            if session is None or (cal_id and session.cal_id != cal_id):
                # R-C-18：重连后携带旧 cal_id 的命令被拒绝并记录日志。
                self._log(f"cal_cancel 拒绝: 无匹配会话（cal_id={cal_id!r}）")
                self._send_calibration_message(
                    "calibration_result", cal_id, status="rejected",
                    reason="无匹配的校准会话",
                )
                return False
            try:
                session.cancel(reason)
            except Exception as exc:  # noqa: BLE001
                self._log(f"校准取消失败: {exc}")
                return False
            self._calibration_session = None
        self._log(f"校准会话已取消: cal_id={session.cal_id}（{reason}）；已收 capture 文件保留")
        self._send_calibration_message(
            "calibration_result", session.cal_id, status="cancelled", reason=reason
        )
        return True

    def _command_calibration_status(self, cal_id: str) -> None:
        """R-C-16：地面查询当前生效校准摘要。"""
        with self._calibration_lock:
            session = self._calibration_session
            active = session.active() if session is not None else None
        payload_fields: dict[str, object] = {
            "status": "active" if active else "none",
            "calibration_id": self.active_calibration_id or "",
        }
        if active:
            payload_fields["valid_band_hz"] = "{:g}-{:g}".format(*active["valid_band_hz"])
            payload_fields["mode"] = str(active.get("mode", ""))
        self._send_calibration_message("calibration_result", cal_id or "query", **payload_fields)

    # ---- 采集编排（复用既有 worker / service，绝不复制核心逻辑）----

    def start_acquisition(self, single: bool = False) -> bool:
        if self._is_acquiring():
            self._log("启动被拒: 采集已在运行")
            return False
        conflict = self._calibration_acquisition_conflict("启动采集")
        if conflict:
            # R-C-14 采集方向：校准会话进行中（planned/in_progress/solved）
            # 拒绝启动（cal_id 缺省 = 采集调用）。
            self._log(conflict)
            return False
        cfg = self._sweep
        if cfg.stop_mhz <= cfg.start_mhz:
            self._log("启动被拒: Stop MHz 必须大于 Start MHz")
            return False
        engine = self.config.engine
        if engine not in ENGINE_CHOICES:
            self._log(f"启动被拒: engine 非法 {engine!r}")
            return False
        if (
            air.get_default_device_source() == air.DEVICE_SOURCE_SIMULATED
            and engine == "legacy"
        ):
            # 与 MainWindow 同规则：模拟器只支持连续采集协议线假设备。
            self._log("启动被拒: 模拟器（无硬件演示）只支持 continuous 引擎")
            return False
        if engine == "legacy":
            return self._start_legacy(cfg, single)
        return self._start_continuous(cfg, single)

    def _start_continuous(self, cfg: air.SweepConfig, single: bool) -> bool:
        self._log(f"启动采集: 引擎 continuous，设备来源 {air.get_default_device_source()}")
        spec = air.AcquisitionTaskSpec(
            cfg=cfg,
            interval_s=self._interval_s,
            out_dir=self.output_root,  # 实际落盘由工厂改为 run 子目录（水位清理以 run 为单位）
            count=1 if single else self._count,
            gnss_port=self.config.gnss_port,
            baud=self.config.gnss_baud,
            calibration_path=self._calibration_path,
            session_calibration=self._session_calibration,
            session_calibration_p2=self._session_calibration_p2,
            through_measurement=self._active_through,
            background_frequency_hz=self._background_frequency_hz,
            background=self._background,
            autosave=self._autosave,
            save_csv=self._save_csv,
            remote_sender=self.sender,
            calibration_id=self.active_calibration_id or "",
        )

        def _factory(task_spec: air.AcquisitionTaskSpec) -> air.ContinuousAcquisitionWorker:
            run_dir = self.output_root / sanitize_run_dir_name(task_spec.run_id)
            self._active_run_id = str(task_spec.run_id or "")
            self._last_run_id = self._active_run_id
            worker = air.ContinuousAcquisitionWorker(
                cfg=task_spec.cfg,
                interval_s=task_spec.interval_s,
                out_dir=run_dir,
                count=task_spec.count,
                gnss_port="",  # Service-owned GNSS continues across radar tasks.
                baud=task_spec.baud,
                calibration_path=task_spec.calibration_path,
                session_calibration=task_spec.session_calibration,
                session_calibration_p2=task_spec.session_calibration_p2,
                through_measurement=task_spec.through_measurement,
                background_frequency_hz=task_spec.background_frequency_hz,
                background=task_spec.background,
                autosave=task_spec.autosave,
                save_csv=task_spec.save_csv,
                remote_sender=task_spec.remote_sender,
                service=self.acquisition,
                task_spec=task_spec,
                gnss_cache=self.gnss,
            )
            # 黑匣子回调只做日志/原子字段更新（不碰任何 UI），显式 DirectConnection：
            # 事件循环停摆/无 exec 场景（崩溃收尾、测试）下日志与收尾仍可靠，
            # 绝不让状态可见性依赖主线程队列。
            _direct = QtCore.Qt.ConnectionType.DirectConnection
            worker.trace_ready.connect(self._on_continuous_trace, _direct)
            worker.status.connect(self._on_worker_text, _direct)
            worker.failed.connect(self._on_worker_failed, _direct)
            worker.finished_run.connect(self._on_task_finished, _direct)
            worker.statistics_updated.connect(self._on_statistics, _direct)
            return worker

        worker = self.acquisition.start_task(spec, _factory)
        if worker is None:
            self._log("常驻服务拒绝启动采集任务（详见服务状态日志）")
            return False
        self._log(f"采集任务已启动: run_id={self._active_run_id}")
        return True

    def _start_legacy(self, cfg: air.SweepConfig, single: bool) -> bool:
        self._log("启动采集: 引擎 legacy（逐帧开关 USB，慢；调试退路）")
        run_id = self.acquisition.new_run_id(air.RUN_ID_KIND_TASK)
        run_dir = self.output_root / sanitize_run_dir_name(run_id)
        worker = air.AcquisitionWorker(
            cfg=cfg,
            gnss_port="",  # Legacy radar must never read the serial port inline.
            baud=self.config.gnss_baud,
            out_dir=run_dir,
            count=1 if single else self._count,
            interval_s=self._interval_s,
            calibration_path=self._calibration_path,
            session_calibration=self._session_calibration,
            session_calibration_p2=self._session_calibration_p2,
            through_measurement=self._active_through,
            background_frequency_hz=self._background_frequency_hz,
            background=self._background,
            autosave=self._autosave,
            gnss_cache=self.gnss,
        )
        worker.trace_ready.connect(self._on_legacy_trace, QtCore.Qt.ConnectionType.DirectConnection)
        worker.status.connect(self._on_worker_text, QtCore.Qt.ConnectionType.DirectConnection)
        worker.failed.connect(self._on_worker_failed, QtCore.Qt.ConnectionType.DirectConnection)
        worker.finished_run.connect(self._on_task_finished, QtCore.Qt.ConnectionType.DirectConnection)
        self._legacy_worker = worker
        worker.start()
        self._active_run_id = run_id
        self._last_run_id = run_id
        self._log(f"采集任务已启动: run_id={run_id} (legacy)")
        return True

    def stop_acquisition(self) -> bool:
        stopped = self.acquisition.request_stop("黑匣子收到地面停止命令")
        worker = self._legacy_worker
        if worker is not None:
            try:
                worker.stop()
            except Exception:  # noqa: BLE001
                pass
            stopped = True
        if stopped:
            self._log("正在停止采集...")
        else:
            self._log("停止被拒: 当前无运行中的采集任务")
        return stopped

    # ---- worker 信号接线（无界面回写：日志 + 积压观测）----

    def _on_worker_text(self, text: object) -> None:
        self._log(str(text))

    def _on_worker_failed(self, message: object) -> None:
        self._log(f"采集失败: {message}")

    def _on_statistics(self, stats: dict) -> None:
        rate = stats.get("sweep_rate_hz")
        if isinstance(rate, (int, float)):
            self._log(f"统计: sweep_rate={rate:.2f}Hz")

    def _on_continuous_trace(self, index, *_args: object) -> None:
        # 连续模式：远程发送已由 ProcessingConsumer 完成（与 GUI 同一约定），
        # 黑匣子只记日志。
        self._log(f"Trace {index} 完成")

    def _on_legacy_trace(self, index, freq, raw, display, time_ns, trace_by_channel, fix, cfg, path_text) -> None:
        # legacy 路径与 GUI on_trace 同一发送语义（无显式 run_id → 连接身份）。
        if self.sender is not None:
            self.sender.enqueue_trace(index, cfg, freq, raw, display, time_ns, trace_by_channel, fix)
        self._log(f"Trace {index} complete" + (f" saved: {Path(str(path_text)).name}" if path_text else ""))

    def _on_task_finished(self) -> None:
        self._log(f"采集结束: run_id={self._active_run_id}")
        self.acquisition.note_worker_finished(self._legacy_worker)  # legacy 兜底登记（no-op 于服务任务）
        self._legacy_worker = None
        self._active_run_id = ""

    # ---- 生命周期 ----

    def start(self) -> None:
        try:
            self.gnss.start()
        except Exception as exc:
            self._log(f"GNSS 后台启动失败（不影响其他功能）: {exc}")
        self.sender.start()
        self._log(
            f"黑匣子启动: v{PROGRAM_VERSION} pid={os.getpid()} 目标地面端 "
            f"{self.config.ground_host}:{self.config.ground_port}（5s→30s 退避重试）"
        )
        if self.config.autostart_acquisition:
            self._log("autostart_acquisition=true：开机自动开始采集")
            self.start_acquisition()
        else:
            self._log("autostart_acquisition=false：等待地面端命令（D11）")

    def disk_tick(self) -> dict[str, object]:
        """磁盘水位定期检查入口（QTimer / 测试直接调用）。"""
        try:
            return self.disk.check()
        except Exception as exc:  # noqa: BLE001 - 水位检查故障绝不影响采集
            self._log(f"磁盘水位检查异常（本轮跳过）: {exc}")
            return self.disk.state_dict()

    def outbox_pending_count(self) -> int:
        """磁盘 outbox 未确认帧总数（断线补传积压观测；测试/t5 冒烟用）。"""
        outbox = self.output_root / OUTBOX_DIR_NAME
        if not outbox.is_dir():
            return 0
        try:
            return sum(1 for p in outbox.rglob("*.frame") if p.is_file())
        except OSError:
            return -1

    def shutdown(self) -> None:
        self.gnss.stop()
        # R-C-18：关闭时进行中的校准会话作废（capture 文件保留并标记
        # incomplete，绝不激活半套校准；与 MainWindow.closeEvent 同规则）。
        with self._calibration_lock:
            session = self._calibration_session
            if session is not None and session.state not in (
                air.CalibrationSession.STATE_FAILED,
                air.CalibrationSession.STATE_CANCELLED,
            ):
                try:
                    session.cancel("黑匣子关闭")
                except Exception:  # noqa: BLE001 - 取消失败不阻止关闭流程
                    pass
                self._calibration_session = None
        self._log("黑匣子收尾: 停止采集 → 释放 USB → 断开发送线程（outbox 数据保留待补传）")
        self.acquisition.shutdown(10000)
        worker = self._legacy_worker
        if worker is not None:
            try:
                worker.stop()
                worker.wait(5000)
            except Exception:  # noqa: BLE001
                pass
            self._legacy_worker = None
        try:
            self.sender.stop(timeout_ms=2000, drain=False)
        except Exception:  # noqa: BLE001
            pass
        state = self.disk.state_dict()
        self._log(
            "黑匣子已停止。磁盘清理累计: "
            f"runs={state['runs_deleted']} files={state['files_deleted']} "
            f"bytes={state['bytes_freed']} undelivered_alarm={state['undelivered_alarm']}"
        )


# =========================================================================
# USB 独立探测（test_vna 兜底路径；可被测试整体替换）
# =========================================================================


def _usb_probe_device_info() -> dict[str, object]:
    """独立短探测：open_device → RequestDeviceInfo → 释放（不经过常驻会话）。

    仅在常驻会话未持有 USB 时使用；绝不 claim 正被服务持有的设备。
    """
    try:
        from librevna.librevna_usb_probe import open_device  # noqa: PLC0415
        from librevna.librevna_usb_s11 import request_device_info  # noqa: PLC0415
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "online": False, "error": f"USB 探测依赖不可用: {exc}"}
    try:
        dev, interface_number = open_device()
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "online": False, "error": str(exc)}
    try:
        from librevna.librevna_usb_probe import PacketStream  # noqa: PLC0415

        info = request_device_info(dev, PacketStream())
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "online": False, "error": str(exc)}
    finally:
        try:
            usb_util_release(dev, interface_number)
        except Exception:  # noqa: BLE001
            pass
    return {
        "ok": True,
        "online": True,
        "firmware": str(info.get("firmware", "")),
        "protocol": int(info.get("protocol", 0)),
        "max_freq_hz": int(info.get("max_freq_hz", 0)),
        "source": "probe",
    }


def usb_util_release(dev: object, interface_number: int) -> None:
    """释放 USB 接口（独立小函数：便于异常路径下尽力释放）。"""
    import usb.util  # noqa: PLC0415

    usb.util.release_interface(dev, interface_number)  # type: ignore[arg-type]
    usb.util.dispose_resources(dev)  # type: ignore[arg-type]


# =========================================================================
# 日志与入口
# =========================================================================


def build_logger(program_dir: Path, *, max_bytes: int, backup_count: int) -> logging.Logger:
    logger = logging.getLogger("air_service")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    if logger.handlers:  # 重复构建（测试）时先清空
        for handler in list(logger.handlers):
            logger.removeHandler(handler)
            try:
                handler.close()
            except Exception:  # noqa: BLE001
                pass
    log_dir = program_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    file_handler = logging.handlers.RotatingFileHandler(
        log_dir / "air_service.log",
        maxBytes=max(1024, int(max_bytes)),
        backupCount=max(0, int(backup_count)),
        encoding="utf-8",
    )
    file_handler.setFormatter(
        logging.Formatter("%(asctime)s %(levelname)s %(message)s", "%Y-%m-%d %H:%M:%S")
    )
    logger.addHandler(file_handler)
    console = logging.StreamHandler(sys.stdout)
    console.setFormatter(logging.Formatter("%(asctime)s %(message)s", "%H:%M:%S"))
    logger.addHandler(console)
    return logger


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    program_dir = Path(__file__).resolve().parent
    bootstrap_log = lambda message: print(message, flush=True)  # noqa: E731
    config_path = Path(args.config) if args.config else program_dir / "air_config.json"
    config, issues = load_air_config(config_path, log=None)  # issues 稍后统一进滚动日志
    override_notes: list[str] = []
    try:
        config = apply_cli_overrides(
            config, args, log=lambda message: override_notes.append(message)
        )
    except ValueError as exc:
        bootstrap_log(f"命令行参数错误: {exc}")
        return 2
    logger = build_logger(
        program_dir, max_bytes=config.log_max_bytes, backup_count=config.log_backup_count
    )
    log = lambda message: logger.info(message)  # noqa: E731
    for issue in issues:
        log(f"配置: {issue}")
    for note in override_notes:
        log(note)

    app = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication(["uav-gpr-air-service"])
    app.setApplicationName("uav-gpr-air-service")
    service = AirBlackboxService(
        config,
        program_dir=program_dir,
        config_path=config_path,
        log=log,
    )

    def _on_signal(signum, _frame) -> None:
        log(f"收到信号 {signum}，开始收尾...")
        app.quit()

    for sig_name in ("SIGINT", "SIGTERM", "SIGBREAK"):
        sig = getattr(signal, sig_name, None)
        if sig is None:
            continue
        try:
            signal.signal(sig, _on_signal)
        except (ValueError, OSError):  # 非 主线程等场景
            pass

    # 信号泵：让 Python 层 signal handler 在 Qt 事件循环阻塞期间也能得到执行。
    pump = QtCore.QTimer()
    pump.setInterval(200)
    pump.timeout.connect(lambda: None)
    pump.start()

    disk_timer = QtCore.QTimer()
    disk_timer.setInterval(int(DISK_CHECK_INTERVAL_S * 1000))
    disk_timer.timeout.connect(service.disk_tick)
    disk_timer.start()

    service.start()
    result = app.exec()
    pump.stop()
    disk_timer.stop()
    service.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
