"""地面端解析与显示流水线（第1轮检查点B）。

把原 :meth:`GroundStationWindow.on_trace` 在 Qt 主线程执行的 NPZ 解压、
schema/数组严格校验、metadata/GNSS 解析全部移到后台解码线程；主线程只做
原子追加 + 显示 FIFO 提交。

组成：

- :class:`DecodedGroundTrace`：不可变解码结果（frozen dataclass）。
- :func:`decode_ground_trace_payload`：纯函数解码（供线程与测试复用），
  失败抛出 ValueError/TypeError/OSError/KeyError，由线程捕获上报。
- :class:`GroundTraceDecoder`：QThread，从内部 FIFO 取 (header, payload,
  path) 解码，``trace_decoded`` 信号回主线程；解码不在主线程执行。
- :class:`GroundDisplayFIFO`：显式 FIFO 显示队列，按到达顺序逐条提交；
  记录 depth / peak / oldest_age_ms，绝不合并、覆盖或丢弃。

严格校验规则（时间轴/Dewow/通道声明）保持与原 on_trace 相同，由主线程在
追加阶段执行（与历史状态相关，必须串行）；本模块完成与历史无关的
payload 解析。
"""

from __future__ import annotations

import io
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np


#: 可选全局性能统计（由地面站注入；解码线程内记录解码耗时）。
_metrics: PipelineMetrics | None = None


def set_metrics(metrics: "PipelineMetrics | None") -> None:
    """注入/清除性能统计对象（None 表示禁用）。线程安全。"""
    global _metrics
    _metrics = metrics


@dataclass(frozen=True)
class DecodedGroundTrace:
    """一条已解码、已通过 payload 级校验的地面 Trace（不可变）。"""

    trace_index: Any
    frequency_hz: np.ndarray
    time_ns: np.ndarray
    trace_s11: np.ndarray | None
    trace_s22: np.ndarray | None
    legacy_trace: np.ndarray | None
    channels: tuple[str, ...] | None
    processing_signature: tuple[bool, float | None]
    dewow_state: str
    acquisition: dict[str, Any] = field(default_factory=dict)
    flight: dict[str, Any] = field(default_factory=dict)
    gnss: dict[str, Any] = field(default_factory=dict)
    receiver_path: str = ""
    decoded_at_perf: float = 0.0


def decode_ground_trace_payload(
    header: dict[str, object],
    payload: bytes,
    path: str,
) -> DecodedGroundTrace:
    """解压 NPZ 并执行 payload 级严格校验（原 on_trace 前半段，纯函数）。

    校验失败抛出 ValueError/TypeError/OSError/KeyError。本函数不访问任何
    Qt 对象，也不读取窗口历史状态（时间轴/Dewow 与历史的比较在主线程
    追加阶段执行）。

    校验助手延迟导入自 :mod:`uav_gpr_ground_station`（避免循环导入；
    Qt 主模块仅在本函数首次调用时加载，解码线程中已可用）。
    """
    from uav_gpr_ground_station import (  # noqa: PLC0415 - 延迟导入避免循环
        _parse_channels_declaration,
        _validate_optional_display_fields,
        acquisition_from_sources,
        dewow_settings_from_sources,
        flight_from_sources,
        gnss_from_sources,
    )

    started = time.perf_counter()
    with np.load(io.BytesIO(payload), allow_pickle=False) as data:
        files = set(data.files)
        if "frequency_hz" not in files or "time_ns" not in files:
            raise ValueError("NPZ is missing frequency_hz or time_ns")
        freq = np.asarray(data["frequency_hz"], dtype=float)
        time_ns = np.asarray(data["time_ns"], dtype=float)
        if freq.ndim != 1 or not np.all(np.isfinite(freq)):
            raise ValueError("frequency_hz must be a finite one-dimensional array")

        # 通道声明一致性校验（fail-closed）。
        npz_channels = _parse_channels_declaration(
            "channels" in files,
            data["channels"] if "channels" in files else None,
            "NPZ",
        )
        header_channels = _parse_channels_declaration(
            "channels" in header,
            header.get("channels"),
            "telemetry",
        )
        if (
            npz_channels is not None
            and header_channels is not None
            and list(npz_channels) != list(header_channels)
        ):
            raise ValueError(
                f"NPZ/telemetry channels 冲突: {npz_channels} vs {header_channels}"
            )
        declared_channels = npz_channels if npz_channels is not None else header_channels
        trace_keys = {name for name in ("S11", "S21", "S12", "S22") if f"trace_{name}" in files}
        display_names = {
            name
            for name in ("S11", "S21", "S12", "S22")
            if f"display_{name}_real" in files and f"display_{name}_imag" in files
        }
        if declared_channels is not None:
            actual_names = trace_keys | display_names
            if actual_names != set(declared_channels):
                raise ValueError(
                    f"channels 声明与实际字段不符: 声明 {declared_channels}, "
                    f"实际 {sorted(actual_names)}"
                )

        processing_signature, dewow_state = dewow_settings_from_sources(data, header)
        gnss = gnss_from_sources(data, header)
        acquisition = acquisition_from_sources(data, header)
        flight = flight_from_sources(data, header)

        if {"S11", "S22"}.issubset(trace_keys):
            _validate_optional_display_fields(data, files, freq)
            trace_s11 = (
                np.asarray(data["trace_S11"], dtype=float)
                if "trace_S11" in files
                else None
            )
            trace_s22 = (
                np.asarray(data["trace_S22"], dtype=float)
                if "trace_S22" in files
                else None
            )
            legacy_trace = None
        elif (
            declared_channels is not None
            and list(declared_channels) in (["S11"], ["S22"])
            and trace_keys == set(declared_channels)
        ):
            # 单通道采集（只采 HH 或只采 VV）：必须显式声明 channels，且只含该通道。
            _validate_optional_display_fields(data, files, freq)
            only = declared_channels[0]
            trace_s11 = np.asarray(data["trace_S11"], dtype=float) if only == "S11" else None
            trace_s22 = np.asarray(data["trace_S22"], dtype=float) if only == "S22" else None
            legacy_trace = None
        elif (
            npz_channels is None
            and header_channels is None
            and not trace_keys
            and "trace" in files
        ):
            _validate_optional_display_fields(data, files, freq)
            trace_s11 = None
            trace_s22 = None
            legacy_trace = np.asarray(data["trace"], dtype=float)
        else:
            raise ValueError(
                "NPZ 缺少双通道 B-scan 所需的 trace_S11/trace_S22，且无与 time_ns 匹配的 legacy trace"
            )

        result = DecodedGroundTrace(
            trace_index=header.get("trace_index"),
            frequency_hz=freq,
            time_ns=time_ns,
            trace_s11=trace_s11,
            trace_s22=trace_s22,
            legacy_trace=legacy_trace,
            channels=tuple(declared_channels) if declared_channels is not None else None,
            processing_signature=processing_signature,
            dewow_state=dewow_state,
            acquisition=acquisition,
            flight=flight,
            gnss=gnss,
            receiver_path=path,
            decoded_at_perf=time.perf_counter(),
        )
        if _metrics is not None:
            _metrics.record_stage("ground_decoded", time.perf_counter() - started)
        return result


class GroundTraceDecoder(queue.Queue):
    """后台解码线程（QThread 语义由使用方以 start() 启动）。

    内部为无界 FIFO（解码线程恒在消费，积压有界），``put_job`` 永不阻塞
    接收线程；``stop()`` 幂等（排空已有任务后退出，有界 join）。

    **第三次定向修复**：

    - **accepting 状态机**（问题四）：``start()`` 成功后 accepting=True；
      ``put_job`` 严格校验 accepting，未启动/已停止/已退出时抛 RuntimeError；
      ``stop()`` 在同一把锁内将 accepting=False，关闭竞态期不再接受新任务。
    - **回调屏障所有权**（问题三）：在 *发出 Qt 信号之前* 占用 in_callback；
      回调自身同步抛异常时（信号未投递）由 decoder 线程立即归还屏障并
      计入 ``callback_errors``；正常发出时由主线程槽 finally 调用
      ``mark_callback_consumed`` 归还。**禁止** ``max(0, n-1)`` 掩盖
      underflow；第三次定向修复后改为失败返还并累加 underflow_errors。
    - **统计**（``statistics()``）：submitted / queued（不含哨兵）/
      in_flight / in_callback / callback_consumed / decoded_success /
      rejected / callback_errors / callback_underflow / in_flight_underflow。

    ``stop()`` 幂等：未启动/已退出直接 True（不插哨兵）；同一 stop 周期
    最多一个哨兵；多次 stop queued 不会增加。
    """

    def __init__(self, on_decoded, on_rejected) -> None:
        super().__init__()
        self._on_decoded = on_decoded
        self._on_rejected = on_rejected
        self._stop = threading.Event()
        self._sentinel_pending = False
        self.thread = threading.Thread(target=self._run, name="ground-trace-decoder", daemon=True)
        self._started = False
        self._exited = False
        # accepting：在 start() 后、stop() 接受期间为 True；严格门控 put_job。
        self._accepting = False
        self._accepting_lock = threading.Lock()  # 与 _stats_lock 同序：先 _stats_lock 后 _accepting_lock
        self._stats_lock = threading.Lock()
        self._in_flight = 0
        self._in_callback = 0
        self._submitted = 0
        self._callback_consumed = 0
        self._decoded_success = 0
        self._rejected = 0
        # 第三次定向修复：明确累加，不再用 max(0, …) 掩盖。
        self.callback_errors: list[str] = []
        self._callback_underflow = 0
        self._in_flight_underflow = 0
        # 接受任务失败原因（最后一次）
        self.last_put_error: str | None = None

    def start(self) -> None:
        if self._started:
            return
        with self._stats_lock, self._accepting_lock:
            self._started = True
            self._accepting = True
        self.thread.start()

    @property
    def accepting(self) -> bool:
        with self._accepting_lock:
            return self._accepting

    def stop(self, timeout_s: float = 2.0) -> bool:
        """幂等停止：同一把锁内关闭 accepting；排空已有任务后退出。

        - 未启动 / 已退出：直接返回 True，**不插入哨兵**；
        - 同一 stop 周期最多一个哨兵（``_sentinel_pending``）；
        - 多次 stop：第一次把 accepting 关闭、插哨兵；后续 stop 立即返回
          True（不重复插哨兵、不再改 accepting 状态）。
        - 第三次定向修复：显式 ``_stop.set()`` 让 ``_run_loop`` 在哨兵之后
          立刻看到停止标志；避免出现"sentinel continue 路径"被反复跑
          而永远看不到 is_set() 的窗口（旧版仅靠 sentinel 唤醒后检查
          in_flight==0/qsize==0，但已退出线程在 in_flight>0 时会继续等）。
        """
        with self._stats_lock, self._accepting_lock:
            thread_alive = self._started and not self._exited and self.thread.is_alive()
            need_sentinel = False
            if thread_alive and not self._sentinel_pending:
                self._sentinel_pending = True
                need_sentinel = True
            # 同一锁内关闭 accepting（防止 put_job 趁 is_alive 校验通过后
            # 又被线程先取走哨兵退出而留下 put 不消费）。
            self._accepting = False
        # 第三次定向修复：先 set 再插哨兵（之前漏了 set，导致 _run_loop
        # 看到 sentinel 后走"sentinel continue"路径——is_set() 仍 False，
        # 不会进入排空检查，线程陷入"取到 None → 跳过 → get(timeout=0.1)
        # → 永远在哨兵 continue 循环"死循环）。
        self._stop.set()
        if need_sentinel:
            try:
                self.put_nowait(None)  # 哨兵唤醒（本周期唯一一个）
            except queue.Full:  # pragma: no cover - 无界队列不会满
                pass
        if (
            self._started
            and not self._exited
            and self.thread.is_alive()
            and self.thread is not threading.current_thread()
        ):
            self.thread.join(timeout_s)
        return not (self._started and not self._exited and self.thread.is_alive())

    def put_job(self, header: dict[str, object], payload: bytes, path: str) -> None:
        """提交解码任务。

        第三次定向修复：严格 accepting 校验；未启动/已停止/已退出/线程
        已死 任意之一均抛 RuntimeError（受控异常），绝不静默接受。
        """
        with self._accepting_lock, self._stats_lock:
            alive = self._started and not self._exited and self.thread.is_alive()
            if not (self._accepting and alive):
                self.last_put_error = (
                    "decoder 不可接受新任务（accepting=False 或线程未启动/已退出）"
                )
                raise RuntimeError(self.last_put_error)
            self._submitted += 1
        self.put((header, payload, path))

    def depth(self) -> int:
        """队列深度（不含控制哨兵）。"""
        with self._stats_lock:
            queued = self.qsize()
            if self._sentinel_pending:
                queued = max(0, queued - 1)
            return queued

    @property
    def in_flight(self) -> int:
        with self._stats_lock:
            return self._in_flight

    @property
    def callback_pending(self) -> int:
        """已完成解码但 Qt 主线程尚未处理完的回调数（屏障计数）。"""
        with self._stats_lock:
            return self._in_callback

    def mark_callback_consumed(self) -> None:
        """主线程处理完一条 decoded/rejected 回调后归还屏障计数。

        第三次定向修复：禁止 ``max(0, n-1)``；underflow 必须检测并累加
        callback_underflow。
        """
        with self._stats_lock:
            if self._in_callback <= 0:
                self._callback_underflow += 1
                return
            self._in_callback -= 1
            self._callback_consumed += 1

    def statistics(self) -> dict[str, int]:
        with self._stats_lock:
            queued = self.qsize()
            if self._sentinel_pending:
                queued = max(0, queued - 1)
            return {
                "submitted": self._submitted,
                "queued": queued,
                "in_flight": self._in_flight,
                "in_callback": self._in_callback,
                "callback_consumed": self._callback_consumed,
                "decoded_success": self._decoded_success,
                "rejected": self._rejected,
                "callback_errors": len(self.callback_errors),
                "callback_underflow": self._callback_underflow,
                "in_flight_underflow": self._in_flight_underflow,
            }

    @property
    def idle(self) -> bool:
        """decoder 侧真正空闲：无排队（哨兵除外）、无 in-flight、无待处理回调。"""
        stats = self.statistics()
        return (
            stats["queued"] == 0
            and stats["in_flight"] == 0
            and stats["in_callback"] == 0
        )

    def _run(self) -> None:
        try:
            self._run_loop()
        finally:
            # 线程已退出：put_job 必须严格拒绝。
            with self._stats_lock, self._accepting_lock:
                self._exited = True
                self._accepting = False

    def _run_loop(self) -> None:
        while True:
            try:
                item = self.get(timeout=0.1)
            except queue.Empty:
                if self._stop.is_set():
                    with self._stats_lock:
                        if self._in_flight == 0 and self.qsize() == 0:
                            return
                continue
            if item is None:
                # 哨兵被线程取出：本 stop 周期结束（后续 stop 可再插新的）。
                with self._stats_lock:
                    self._sentinel_pending = False
                if self._stop.is_set():
                    with self._stats_lock:
                        if self._in_flight == 0 and self.qsize() == 0:
                            return
                continue
            header, payload, path = item
            with self._stats_lock:
                self._in_flight += 1
            try:
                # ---- 解码 ----
                try:
                    decoded = decode_ground_trace_payload(header, payload, path)
                except (OSError, ValueError, TypeError, KeyError) as exc:
                    # 拒绝：发出回调前先占屏障计数。
                    callback_delivered = False
                    with self._stats_lock:
                        self._rejected += 1
                        # 释放 in_flight（直接 0 安全路径：in_flight 必为正）
                        if self._in_flight <= 0:
                            self._in_flight_underflow += 1
                        else:
                            self._in_flight -= 1
                        self._in_callback += 1
                    try:
                        self._on_rejected(header, path, exc)
                        callback_delivered = True
                    except Exception as cb_exc:  # noqa: BLE001 - 同步回调异常，信号未投递，立即归还
                        with self._stats_lock:
                            self.callback_errors.append(
                                f"rejected回调异常: {type(cb_exc).__name__}: {cb_exc}"
                            )
                    if not callback_delivered:
                        # 信号未投递：主线程永远不会 mark_callback_consumed。
                        # decoder 线程必须立即归还该屏障，否则永久泄漏。
                        self._release_inflight_after_callback_failure()
                    continue
                # ---- 成功：发出回调前先占屏障计数 ----
                callback_delivered = False
                with self._stats_lock:
                    self._decoded_success += 1
                    if self._in_flight <= 0:
                        self._in_flight_underflow += 1
                    else:
                        self._in_flight -= 1
                    self._in_callback += 1
                try:
                    self._on_decoded(decoded)
                    callback_delivered = True
                except Exception as cb_exc:  # noqa: BLE001 - 同步回调异常
                    with self._stats_lock:
                        self.callback_errors.append(
                            f"decoded回调异常: {type(cb_exc).__name__}: {cb_exc}"
                        )
                if not callback_delivered:
                    self._release_inflight_after_callback_failure()
            except BaseException as exc:  # noqa: BLE001 - 解码线程不得被任何异常杀死
                with self._stats_lock:
                    if self._in_flight <= 0:
                        self._in_flight_underflow += 1
                    else:
                        self._in_flight -= 1
                    self.callback_errors.append(
                        f"解码线程异常: {type(exc).__name__}: {exc}"
                    )

    def _release_inflight_after_callback_failure(self) -> None:
        """回调同步失败（信号未投递）：归还 in_callback 屏障，不留泄漏。

        与 mark_callback_consumed 同样禁止 underflow 掩盖：失败累加
        callback_underflow。
        """
        with self._stats_lock:
            if self._in_callback <= 0:
                self._callback_underflow += 1
                return
            self._in_callback -= 1
            # 回调未投递主线程不会处理 → 仍记 callback_consumed==0（语义保留）


class GroundDisplayFIFO:
    """显式 FIFO 显示队列。

    每条 Trace 只入队/提交一次；按到达顺序弹出，绝不取"最新一条"。

    计数语义（第二次定向修复）：

    - ``total_enqueued``：入队数；
    - ``total_dequeued``：pop 出队数（**只代表 dequeued，不代表已显示**）；
    - ``total_committed``：成功提交数（校验通过 + 原子追加成功 + setImage
      成功 + 当前 Trace 状态提交成功，由 ``mark_committed`` 确认）；
    - ``total_rejected``：兼容性拒绝数（时间轴/Dewow/无效 Trace，
      ``mark_rejected``）；
    - ``total_failed``：UI 提交异常数（``mark_failed``）。

    停止屏障必须使用 committed/rejected/failed，不得把 pop 数当 displayed。
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        # deque + popleft：O(1) 弹出（复审修复：list.pop(0) 是 O(n)）。
        self._items: deque[tuple[float, object]] = deque()
        self.peak = 0
        self.total_enqueued = 0
        self.total_dequeued = 0
        self.total_committed = 0
        self.total_rejected = 0
        self.total_failed = 0

    def enqueue(self, item: object) -> None:
        now = time.perf_counter()
        with self._lock:
            self._items.append((now, item))
            self.total_enqueued += 1
            if len(self._items) > self.peak:
                self.peak = len(self._items)

    def pop(self) -> object | None:
        """弹出最旧一条（仅计 dequeued）；空队列返回 None。

        成功提交必须随后调用 :meth:`mark_committed`；拒绝/失败调用
        :meth:`mark_rejected` / :meth:`mark_failed`。三者必须对每条出队项
        恰好调用一次，否则出队未确认（``unresolved`` > 0）。
        """
        with self._lock:
            if not self._items:
                return None
            _ts, item = self._items.popleft()
            self.total_dequeued += 1
            return item

    def mark_committed(self) -> None:
        """一条 Trace 成功完成校验+追加+setImage+状态提交后确认。"""
        with self._lock:
            self.total_committed += 1

    def mark_rejected(self) -> None:
        """一条 Trace 被兼容性拒绝（不增加 committed）。"""
        with self._lock:
            self.total_rejected += 1

    def mark_failed(self) -> None:
        """一条 Trace 的 UI 提交异常（不增加 committed）。"""
        with self._lock:
            self.total_failed += 1

    def depth(self) -> int:
        with self._lock:
            return len(self._items)

    def oldest_age_ms(self) -> float | None:
        with self._lock:
            if not self._items:
                return None
            return (time.perf_counter() - self._items[0][0]) * 1000.0

    def statistics(self) -> dict[str, object]:
        with self._lock:
            depth = len(self._items)
            oldest = (
                (time.perf_counter() - self._items[0][0]) * 1000.0 if self._items else None
            )
            return {
                "depth": depth,
                "peak": self.peak,
                "total_enqueued": self.total_enqueued,
                "total_dequeued": self.total_dequeued,
                "total_committed": self.total_committed,
                "total_rejected": self.total_rejected,
                "total_failed": self.total_failed,
                # 出队但未确认（committed/rejected/failed 均未调用）。
                "unresolved": self.total_dequeued
                - self.total_committed
                - self.total_rejected
                - self.total_failed,
                "oldest_age_ms": oldest,
            }

    def clear(self) -> None:
        with self._lock:
            self._items.clear()


__all__ = [
    "DecodedGroundTrace",
    "GroundDisplayFIFO",
    "GroundTraceDecoder",
    "decode_ground_trace_payload",
]
