"""PersistentLibreVnaBackend FakeTransport tests (no real hardware).

Adapted from the field-validated reference tests
``E:\\钢筋仪软件开发\\tests\\test_librevna_usb_backend.py`` and
``test_librevna_pause_resume.py``, re-expressed against the current project's
API (``SweepSettings`` config, :class:`CompleteSweep` results,
:class:`BackendState` state machine, dual-reflection S11/S22 only).

Covers the roadmap phase-2 / section 15.3 acceptance matrix:

- **100 sweeps with one open/configure** (open_count=1, configure_count=1,
  SWEEP_SETTINGS sent once);
- open/configure/acquire_one/pause/resume/close lifecycle and exact command
  sequence; pause/resume never reopen the transport;
- state machine CLOSED -> OPEN_IDLE -> STREAMING -> PAUSED and FAULTED
  (only close/reopen allowed); every invalid action raises
  :class:`LibreVnaStateError`;
- pending queue / ACK-batch retention: datapoints received while waiting for an
  ACK go to pending; one read spanning two sweeps loses nothing; next sweep
  enters pending; pending overflow fails closed;
- deterministic behaviour for timeout / NACK / SET_IDLE failure / USB
  disconnect / protocol error (all fail-closed to FAULTED except single sweep
  timeout which stays STREAMING and resynchronises);
- ``close()`` releases USB even when the best-effort SET_IDLE fails;
- legacy ``acquire_sparameters()`` entry point remains available and the new
  backend module imports no hardware.
"""

from __future__ import annotations

import ast
import inspect
import struct
import unittest

import numpy as np

from librevna.librevna_continuous import (
    BackendState,
    CompleteSweep,
    PersistentBackendSettings,
    PersistentLibreVnaBackend,
    SessionStatistics,
)
from librevna.librevna_errors import (
    LibreVnaDisconnectedError,
    LibreVnaNackError,
    LibreVnaPendingOverflowError,
    LibreVnaProtocolError,
    LibreVnaReleaseError,
    LibreVnaStateError,
    LibreVnaSweepError,
    LibreVnaTimeoutError,
    LibreVnaUnsupportedConfigError,
)
from librevna.librevna_protocol import (
    ACK,
    DEVICE_INFO,
    DEVICE_STATUS,
    NACK,
    REQUEST_DEVICE_INFO,
    SET_IDLE,
    S11_S22_STAGES_BITMAP,
    S11_STAGES_BITMAP,
    SWEEP_SETTINGS,
    VNA_DATAPOINT,
    SweepSettings,
    encode_packet,
)


# ---------------------------------------------------------------- packet 合成


def _device_info_payload(num_ports: int = 2) -> bytes:
    return struct.pack(
        "<HBBBBcQQIIHhhIIBQBH",
        14, 1, 2, 3, 5, b"A",
        100_000_000, 6_000_000_000, 1, 1_000_000, 10_001,
        -3000, 1000, 1, 1_000_000, 200, 12_000_000_000, num_ports, 65_535,
    )


def _dual_receivers(s11: complex, s22: complex) -> list[tuple[int, complex]]:
    """6-receiver 双端口点（真机 descriptor 顺序）；ref=1+0j -> S11=s11, S22=s22。"""
    return [
        (0x01, s11),
        (0x02, complex(9.0, 9.0)),
        (0x13, complex(1.0, 0.0)),
        (0x21, complex(7.0, 7.0)),
        (0x22, s22),
        (0x33, complex(1.0, 0.0)),
    ]


def _dual_payload(
    receivers: list[tuple[int, complex]],
    *,
    freq_hz: int = 500_000_000,
    point_number: int = 0,
) -> bytes:
    """BLOCKED 布局：头 + reals + imags + descs。"""
    n = len(receivers)
    payload = struct.pack("<QhH", int(freq_hz), -1000, int(point_number))
    payload += struct.pack("<" + "f" * n, *[v.real for _, v in receivers])
    payload += struct.pack("<" + "f" * n, *[v.imag for _, v in receivers])
    payload += bytes(d for d, _ in receivers)
    return payload


def _point_packets(
    freq_hz: float,
    point_number: int,
    *,
    s11: complex | None = None,
    s22: complex | None = None,
) -> bytes:
    s11 = s11 if s11 is not None else complex(0.5 + 0.001 * point_number, -0.2)
    s22 = s22 if s22 is not None else complex(0.3, 0.1 - 0.001 * point_number)
    return encode_packet(
        VNA_DATAPOINT,
        _dual_payload(
            _dual_receivers(s11, s22),
            freq_hz=int(freq_hz),
            point_number=point_number,
        ),
    )


def _full_sweep(freqs: list[float], start_offset: int = 0) -> list[bytes]:
    """一条完整 sweep 的单个 packet 列表（频率整体偏移 start_offset Hz）。"""
    return [_point_packets(float(f) + start_offset, i) for i, f in enumerate(freqs)]


def _sweep_settings(
    *,
    n_points: int = 5,
    start_hz: int = 500_000_000,
    stop_hz: int = 2_500_000_000,
    ifbw_hz: int = 10_000,
    stages_bitmap: int = S11_S22_STAGES_BITMAP,
) -> SweepSettings:
    return SweepSettings(
        start_hz=start_hz,
        stop_hz=stop_hz,
        points=n_points,
        ifbw_hz=ifbw_hz,
        stages_bitmap=stages_bitmap,
    )


# ---------------------------------------------------------------- FakeTransport


class _ScriptedTransport:
    """脚本化 fake transport：每次 read 弹出一个 chunk（bytes 或异常）。"""

    def __init__(self, chunks: list | None = None) -> None:
        self._chunks: list = list(chunks or [])
        self.writes: list[bytes] = []
        self.open_count = 0
        self.close_count = 0
        self.opened = False

    def open(self) -> None:
        self.open_count += 1
        self.opened = True

    def close(self) -> None:
        self.close_count += 1
        self.opened = False

    @property
    def is_open(self) -> bool:
        return self.opened

    def feed_reads(self, chunks: list) -> None:
        self._chunks.extend(chunks)

    def read(self, max_length: int, timeout_ms: int) -> bytes:
        if not self.opened:
            raise LibreVnaTimeoutError("not open")
        if not self._chunks:
            raise LibreVnaTimeoutError("no data")
        item = self._chunks.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    def write(self, data: bytes) -> None:
        if not self.opened:
            raise LibreVnaTimeoutError("not open")
        self.writes.append(bytes(data))

    def sent_types(self) -> list[int]:
        return [d[3] for d in self.writes if len(d) >= 4 and d[0] == 0x5A]

    def sweep_settings_writes(self) -> list[bytes]:
        return [
            w for w in self.writes
            if len(w) >= 4 and w[0] == 0x5A and w[3] == SWEEP_SETTINGS
        ]


class _FailingOpenTransport:
    """open/close 失败可配置的 fake transport（open 失败清理路径测试）。

    ``open()`` 先置 ``opened=True`` 再抛错，模拟"部分打开并占有资源后失败"；
    ``close()`` 在配置了 ``close_raises`` 时抛错。默认抛错时保持 ``opened=True``
    （释放失败且资源仍被持有）；``close_release_then_raise=True`` 时先置
    ``opened=False`` 再抛错（已释放但报告清理错误）；
    ``close_silently_keeps_open=True`` 时 close 正常返回但**不**把 ``opened``
    置 False（模拟 close 未抛错但资源仍被持有的传输层 bug）。
    """

    def __init__(
        self,
        *,
        open_raises: Exception | None = None,
        close_raises: Exception | None = None,
        close_release_then_raise: bool = False,
        close_silently_keeps_open: bool = False,
    ) -> None:
        self.open_count = 0
        self.close_count = 0
        self.opened = False
        self._open_raises = open_raises
        self._close_raises = close_raises
        self._close_release_then_raise = close_release_then_raise
        self._close_silently_keeps_open = close_silently_keeps_open

    def open(self) -> None:
        self.open_count += 1
        self.opened = True
        if self._open_raises is not None:
            raise self._open_raises

    def close(self) -> None:
        self.close_count += 1
        if self._close_silently_keeps_open:
            return  # 不抛错、不置 opened=False（静默保持打开）
        if not self._close_release_then_raise and self._close_raises is not None:
            raise self._close_raises
        self.opened = False
        if self._close_release_then_raise and self._close_raises is not None:
            raise self._close_raises

    @property
    def is_open(self) -> bool:
        return self.opened

    def read(self, max_length: int, timeout_ms: int) -> bytes:
        raise LibreVnaTimeoutError("no data")

    def write(self, data: bytes) -> None:
        pass

    def make_closable(self) -> None:
        """让后续 close() 正常释放（测试中模拟资源恢复可关闭）。"""
        self._close_raises = None
        self._close_silently_keeps_open = False


def _session_transport(
    *, sweep_chunks: list | None = None, num_ports: int = 2
) -> _ScriptedTransport:
    chunks: list = [
        encode_packet(DEVICE_INFO, _device_info_payload(num_ports)),
        encode_packet(ACK),  # open SET_IDLE
        encode_packet(ACK),  # configure SET_IDLE
        encode_packet(ACK),  # configure SWEEP_SETTINGS
    ]
    chunks.extend(list(sweep_chunks or []))
    return _ScriptedTransport(chunks)


def _open_backend(
    *,
    transport: _ScriptedTransport | None = None,
    sweep_chunks: list | None = None,
    num_ports: int = 2,
    command_timeout_s: float = 0.05,
    usb_read_timeout_ms: int = 1,
) -> tuple[PersistentLibreVnaBackend, _ScriptedTransport]:
    if transport is None:
        transport = _session_transport(sweep_chunks=sweep_chunks, num_ports=num_ports)
    backend = PersistentLibreVnaBackend(
        transport=transport,
        settings=PersistentBackendSettings(
            command_timeout_s=command_timeout_s,
            usb_read_timeout_ms=usb_read_timeout_ms,
        ),
    )
    backend.open()
    return backend, transport


def _make_backend(
    *,
    config: SweepSettings | None = None,
    transport: _ScriptedTransport | None = None,
    sweep_chunks: list | None = None,
    num_ports: int = 2,
    sweep_timeout_s: float = 2.0,
    command_timeout_s: float = 0.05,
    usb_read_timeout_ms: int = 1,
    pending_max: int | None = None,
) -> tuple[PersistentLibreVnaBackend, _ScriptedTransport, SweepSettings]:
    cfg = config if config is not None else _sweep_settings()
    if transport is None:
        transport = _session_transport(sweep_chunks=sweep_chunks, num_ports=num_ports)
    backend = PersistentLibreVnaBackend(
        transport=transport,
        settings=PersistentBackendSettings(
            sweep_timeout_s=sweep_timeout_s,
            command_timeout_s=command_timeout_s,
            usb_read_timeout_ms=usb_read_timeout_ms,
            pending_max=pending_max,
        ),
    )
    backend.open()
    backend.configure(cfg)
    return backend, transport, cfg


def _freqs(n: int = 5) -> list[float]:
    return [0.5e9 + i * 0.5e9 for i in range(n)]


# ================================================================ 状态机


class BackendStateMachineTests(unittest.TestCase):
    def test_state_transitions(self) -> None:
        backend, transport = _open_backend()
        self.assertIs(backend.state, BackendState.OPEN_IDLE)
        self.assertTrue(backend.is_open)
        self.assertIsNone(backend.config)
        transport.feed_reads([encode_packet(ACK), encode_packet(ACK)])
        backend.configure(_sweep_settings())
        self.assertIs(backend.state, BackendState.STREAMING)
        self.assertIsNotNone(backend.config)
        transport.feed_reads([encode_packet(ACK)])
        backend.pause()
        self.assertIs(backend.state, BackendState.PAUSED)
        transport.feed_reads([encode_packet(ACK) + _point_packets(1e9, 0)])
        backend.resume()
        self.assertIs(backend.state, BackendState.STREAMING)
        backend.close()
        self.assertIs(backend.state, BackendState.CLOSED)
        self.assertFalse(backend.is_open)

    def test_acquire_without_configure_rejected(self) -> None:
        backend, transport = _open_backend()
        with self.assertRaises(LibreVnaStateError):
            backend.acquire_one()
        backend.close()

    def test_acquire_after_close_rejected(self) -> None:
        backend, _transport, _ = _make_backend()
        backend.close()
        with self.assertRaises(LibreVnaStateError):
            backend.acquire_one()

    def test_configure_from_streaming_rejected(self) -> None:
        backend, _transport, cfg = _make_backend()
        with self.assertRaises(LibreVnaStateError):
            backend.configure(cfg)
        backend.close()

    def test_configure_from_paused_rejected(self) -> None:
        backend, transport, cfg = _make_backend()
        transport.feed_reads([encode_packet(ACK)])
        backend.pause()
        with self.assertRaises(LibreVnaStateError):
            backend.configure(cfg)
        backend.close()

    def test_pause_from_open_idle_rejected(self) -> None:
        backend, _transport = _open_backend()
        with self.assertRaises(LibreVnaStateError):
            backend.pause()
        backend.close()

    def test_resume_without_pause_rejected(self) -> None:
        backend, _transport, _ = _make_backend()
        with self.assertRaises(LibreVnaStateError):
            backend.resume()
        backend.close()

    def test_double_pause_rejected(self) -> None:
        backend, transport, _ = _make_backend()
        transport.feed_reads([encode_packet(ACK)])
        backend.pause()
        with self.assertRaises(LibreVnaStateError):
            backend.pause()
        backend.close()

    def test_open_idempotent_keeps_single_open(self) -> None:
        backend, transport = _open_backend()
        backend.open()  # 幂等 no-op
        self.assertEqual(transport.open_count, 1)
        backend.close()


# ================================================================ 生命周期


class BackendLifecycleTests(unittest.TestCase):
    def test_open_requests_device_info_then_set_idle(self) -> None:
        backend, transport = _open_backend()
        self.assertEqual(
            transport.sent_types()[:2], [REQUEST_DEVICE_INFO, SET_IDLE]
        )
        self.assertIsNotNone(backend.device_info)
        self.assertEqual(backend.device_info.firmware, "1.2.3")
        backend.close()

    def test_configure_sends_set_idle_and_sweep_settings_once(self) -> None:
        backend, transport, _ = _make_backend()
        self.assertEqual(
            transport.sent_types(),
            [REQUEST_DEVICE_INFO, SET_IDLE, SET_IDLE, SWEEP_SETTINGS],
        )
        self.assertEqual(len(transport.sweep_settings_writes()), 1)
        self.assertEqual(backend.statistics.open_count, 1)
        self.assertEqual(backend.statistics.configure_count, 1)
        self.assertEqual(backend.statistics.connection_generation, 1)
        backend.close()

    def test_acquire_one_does_not_reopen_or_resend(self) -> None:
        backend, transport, _ = _make_backend(sweep_chunks=_full_sweep(_freqs()))
        writes_before = len(transport.writes)
        backend.acquire_one()
        self.assertEqual(len(transport.writes), writes_before)
        self.assertEqual(transport.open_count, 1)
        backend.close()

    def test_close_sends_set_idle_and_releases(self) -> None:
        backend, transport, _ = _make_backend()
        backend.close()
        self.assertEqual(transport.sent_types()[-1], SET_IDLE)
        self.assertEqual(transport.close_count, 1)
        self.assertIs(backend.state, BackendState.CLOSED)

    def test_close_idempotent(self) -> None:
        backend, transport, _ = _make_backend()
        backend.close()
        backend.close()
        self.assertEqual(transport.close_count, 1)

    def test_trace_index_increments(self) -> None:
        freqs = _freqs()
        backend, transport, _ = _make_backend(sweep_chunks=_full_sweep(freqs))
        self.assertEqual(backend.acquire_one().trace_index, 0)
        transport.feed_reads(_full_sweep(freqs, start_offset=1))
        self.assertEqual(backend.acquire_one().trace_index, 1)
        transport.feed_reads(_full_sweep(freqs, start_offset=2))
        self.assertEqual(backend.acquire_one().trace_index, 2)
        backend.close()

    def test_open_failure_releases_transport(self) -> None:
        # 只回 ACK，不回 DEVICE_INFO -> 超时；open 失败必须释放资源并回到 CLOSED。
        transport = _ScriptedTransport([encode_packet(ACK), encode_packet(ACK)])
        backend = PersistentLibreVnaBackend(
            transport=transport,
            settings=PersistentBackendSettings(
                device_info_timeout_s=0.1, usb_read_timeout_ms=1
            ),
        )
        with self.assertRaises(LibreVnaTimeoutError):
            backend.open()
        self.assertIs(backend.state, BackendState.CLOSED)
        self.assertFalse(transport.is_open)
        backend.close()  # 幂等

    def test_open_transport_failure_releases_and_reraises(self) -> None:
        # transport.open() 部分打开后抛错 -> 也必须进入清理：close 被调用、
        # 资源释放、原始异常重新抛出、后续 close() 幂等。
        transport = _FailingOpenTransport(open_raises=RuntimeError("open boom"))
        backend = PersistentLibreVnaBackend(
            transport=transport,
            settings=PersistentBackendSettings(
                command_timeout_s=0.05, usb_read_timeout_ms=1
            ),
        )
        with self.assertRaises(RuntimeError) as ctx:
            backend.open()
        self.assertEqual(str(ctx.exception), "open boom")  # 原始异常未替换
        self.assertEqual(transport.close_count, 1)  # 清理路径调用了 close
        self.assertFalse(transport.is_open)  # 资源已释放
        self.assertIs(backend.state, BackendState.CLOSED)
        # 随后 close() 幂等，不再重复 close
        backend.close()
        self.assertEqual(transport.close_count, 1)

    def test_open_failure_with_failed_cleanup_enters_faulted(self) -> None:
        # transport.open() 抛错且首次 close() 也抛错并保持 opened=True：
        # backend 不得伪装安全 CLOSED -> FAULTED，保留原始异常 + 清理失败诊断；
        # FAULTED 禁止 configure/acquire；transport 恢复可关闭后 close() 能重试。
        transport = _FailingOpenTransport(
            open_raises=RuntimeError("open boom"),
            close_raises=RuntimeError("close boom"),
        )
        backend = PersistentLibreVnaBackend(
            transport=transport,
            settings=PersistentBackendSettings(
                command_timeout_s=0.05, usb_read_timeout_ms=1
            ),
        )
        with self.assertRaises(RuntimeError) as ctx:
            backend.open()
        # 原始打开错误保留
        self.assertEqual(str(ctx.exception), "open boom")
        # 清理失败诊断经 add_note 保留
        notes = list(getattr(ctx.exception, "__notes__", []))
        self.assertTrue(notes)
        self.assertTrue(any("close" in note.lower() for note in notes))
        self.assertTrue(any("FAULTED" in note for note in notes))
        # 不得伪装安全 CLOSED：transport 仍打开 -> FAULTED
        self.assertIs(backend.state, BackendState.FAULTED)
        self.assertTrue(transport.is_open)
        self.assertEqual(transport.close_count, 1)  # 已尝试过一次 close
        # FAULTED 禁止 configure / acquire
        with self.assertRaises(LibreVnaStateError):
            backend.configure(_sweep_settings())
        with self.assertRaises(LibreVnaStateError):
            backend.acquire_one()
        # transport 恢复可关闭后，backend.close() 重试并最终进入 CLOSED
        transport.make_closable()
        backend.close()
        self.assertIs(backend.state, BackendState.CLOSED)
        self.assertFalse(transport.is_open)


# ================================================================ 连续采集


class BackendSweepTests(unittest.TestCase):
    def test_returns_complete_sweep(self) -> None:
        freqs = _freqs()
        backend, _transport, _ = _make_backend(sweep_chunks=_full_sweep(freqs))
        sweep = backend.acquire_one()
        self.assertIsInstance(sweep, CompleteSweep)
        self.assertEqual(sweep.trace_index, 0)
        self.assertEqual(sweep.raw.shape, (2, 5))
        # 使用实际 packet 频率（非 linspace 伪造）
        np.testing.assert_allclose(sweep.frequency_hz, freqs, rtol=0, atol=1e-3)
        # point 0 的 S11 = 0.5 - 0.2j（ref=1+0j）
        np.testing.assert_allclose(sweep.s11[0], complex(0.5, -0.2), atol=1e-6)
        # S22 = 0.3 + 0.1j
        np.testing.assert_allclose(sweep.s22[0], complex(0.3, 0.1), atol=1e-6)
        backend.close()

    def test_same_read_spans_two_sweeps(self) -> None:
        freqs = _freqs()
        sweep1 = b"".join(_point_packets(float(freqs[i]), i) for i in range(5))
        sweep2_head = b"".join(
            _point_packets(float(freqs[i]) + 1, i) for i in range(2)
        )
        backend, transport, _ = _make_backend(sweep_chunks=[sweep1 + sweep2_head])
        frame1 = backend.acquire_one()
        self.assertEqual(frame1.trace_index, 0)
        # 第二道剩余点补齐；一次 read 跨两道的剩余 packet 不丢失
        transport.feed_reads(
            [_point_packets(float(freqs[i]) + 1, i) for i in range(2, 5)]
        )
        frame2 = backend.acquire_one()
        self.assertEqual(frame2.trace_index, 1)
        np.testing.assert_allclose(frame2.frequency_hz, [f + 1 for f in freqs], atol=1e-3)
        backend.close()

    def test_next_sweep_enters_pending_not_lost(self) -> None:
        freqs = _freqs()
        sweep1 = b"".join(_point_packets(float(freqs[i]), i) for i in range(5))
        sweep2 = b"".join(_point_packets(float(freqs[i]) + 1, i) for i in range(5))
        backend, transport, _ = _make_backend(sweep_chunks=[sweep1 + sweep2])
        frame1 = backend.acquire_one()
        self.assertEqual(frame1.trace_index, 0)
        # 第二道全部提前进入 pending -> 下一次 acquire_one 立即返回
        frame2 = backend.acquire_one()
        self.assertEqual(frame2.trace_index, 1)
        self.assertNotAlmostEqual(frame2.frequency_hz[0], frame1.frequency_hz[0])
        backend.close()

    def test_missing_point_then_timeout_stays_streaming(self) -> None:
        freqs = _freqs()
        chunks = [_point_packets(float(freqs[i]), i) for i in range(5) if i != 3]
        backend, _transport, _ = _make_backend(sweep_chunks=chunks, sweep_timeout_s=0.05)
        with self.assertRaises(LibreVnaSweepError) as ctx:
            backend.acquire_one()
        self.assertIn("完整一道", str(ctx.exception))
        # 单次超时：丢 partial、计数、状态保持 STREAMING（继续重同步）
        self.assertIs(backend.state, BackendState.STREAMING)
        self.assertEqual(backend.statistics.sweep_timeouts, 1)
        backend.close()

    def test_no_silent_partial_sweep(self) -> None:
        freqs = _freqs()
        chunks = [_point_packets(float(freqs[i]), i) for i in range(3)]
        backend, _transport, _ = _make_backend(sweep_chunks=chunks, sweep_timeout_s=0.05)
        with self.assertRaises(LibreVnaSweepError):
            backend.acquire_one()
        backend.close()

    def test_timeout_clears_half_packet_and_never_stitches(self) -> None:
        """sweep 超时原子边界：清空半包/Pending/held，绝不跨边界拼包。"""
        freqs = _freqs()
        full_packet = _point_packets(float(freqs[0]), 0)
        half = len(full_packet) // 2
        first_half = full_packet[:half]
        second_half = full_packet[half:]
        # 第一次：point0 完整到达 + point1 的半个包 -> 超时（assembler 有 partial）
        backend, transport, _ = _make_backend(
            sweep_chunks=[_point_packets(float(freqs[0]), 0), first_half],
            sweep_timeout_s=0.05,
        )
        with self.assertRaises(LibreVnaSweepError):
            backend.acquire_one()
        # stream / 当前 sweep / pending / held 状态全部清空
        self.assertEqual(backend._stream.buffer, b"")  # noqa: SLF001
        self.assertEqual(len(backend._pending), 0)  # noqa: SLF001
        self.assertFalse(backend._sweep_held)  # noqa: SLF001
        self.assertEqual(backend._assembler._points, [])  # noqa: SLF001
        # 状态保持 STREAMING；sweep_timeouts 与丢弃统计正确
        self.assertIs(backend.state, BackendState.STREAMING)
        self.assertEqual(backend.statistics.sweep_timeouts, 1)
        self.assertGreaterEqual(backend.statistics.dropped_sweeps, 1)
        self.assertGreaterEqual(backend.statistics.incomplete_sweeps, 1)
        trace_before = backend.statistics.trace_index
        # 第二次：旧包后半段 + 其余点 -> 不能生成 frame（timeout 前后不拼包）
        transport.feed_reads(
            [second_half] + [_point_packets(float(freqs[i]), i) for i in range(1, 5)]
        )
        with self.assertRaises(LibreVnaSweepError):
            backend.acquire_one()
        self.assertEqual(backend.statistics.trace_index, trace_before)
        self.assertEqual(backend.statistics.completed_sweeps, 0)
        # 第三次：完整新 sweep -> 正常返回
        transport.feed_reads(_full_sweep(freqs, start_offset=1))
        frame = backend.acquire_one()
        self.assertEqual(frame.trace_index, trace_before)
        np.testing.assert_allclose(frame.frequency_hz, [f + 1 for f in freqs], atol=1e-3)
        backend.close()

    def test_nack_during_acquire_faults(self) -> None:
        freqs = _freqs()
        chunks = [_point_packets(float(freqs[0]), 0), encode_packet(NACK)]
        backend, _transport, _ = _make_backend(sweep_chunks=chunks)
        with self.assertRaises(LibreVnaNackError):
            backend.acquire_one()
        self.assertIs(backend.state, BackendState.FAULTED)
        backend.close()
        self.assertIs(backend.state, BackendState.CLOSED)

    def test_disconnect_mid_sweep_faults(self) -> None:
        freqs = _freqs()
        chunks = [
            _point_packets(float(freqs[0]), 0),
            LibreVnaDisconnectedError("device unplugged"),
        ]
        backend, _transport, _ = _make_backend(sweep_chunks=chunks)
        with self.assertRaises(LibreVnaDisconnectedError):
            backend.acquire_one()
        self.assertIs(backend.state, BackendState.FAULTED)
        self.assertEqual(backend.statistics.disconnects, 1)
        backend.close()

    def test_malformed_datapoint_faults(self) -> None:
        chunks = [encode_packet(VNA_DATAPOINT, b"\x00\x01")]
        backend, _transport, _ = _make_backend(sweep_chunks=chunks)
        with self.assertRaises(LibreVnaProtocolError):
            backend.acquire_one()
        self.assertIs(backend.state, BackendState.FAULTED)
        backend.close()

    def test_malformed_point0_receiver_group_faults_without_delivering(self) -> None:
        # 一个原本合法的双通道 point0 payload 追加 1 个残字节 -> receiver 分组
        # 长度 (len-12)%9 != 0。经 PacketStream -> parse_s11_s22_point -> backend
        # 真实处理入口抛 LibreVnaProtocolError 并进入 FAULTED；后续合法
        # point1..N-1 存在也不得组装出 sweep。
        freqs = _freqs()
        legal_p0 = _dual_payload(
            _dual_receivers(complex(0.5, 0), complex(0.3, 0)),
            freq_hz=int(freqs[0]),
            point_number=0,
        )
        self.assertEqual((len(legal_p0) - 12) % 9, 0)
        malformed_p0 = legal_p0 + b"\x00"  # (len - 12) % 9 == 1
        chunks = [
            encode_packet(VNA_DATAPOINT, malformed_p0),
            *[_point_packets(float(freqs[i]), i) for i in range(1, len(freqs))],
        ]
        backend, _transport, _ = _make_backend(sweep_chunks=chunks)
        with self.assertRaises(LibreVnaProtocolError):
            backend.acquire_one()
        self.assertIs(backend.state, BackendState.FAULTED)
        stats = backend.statistics
        self.assertEqual(stats.completed_sweeps, 0)
        self.assertEqual(stats.trace_index, 0)
        backend.close()

    def test_reference_zero_discards_and_resyncs(self) -> None:
        freqs = _freqs()
        # point 1 reference=0 -> invalid -> 整道丢弃；随后新道正常返回
        bad_payload = bytearray(
            _dual_payload(
                _dual_receivers(complex(0.5, 0), complex(0.3, 0)),
                freq_hz=int(freqs[1]),
                point_number=1,
            )
        )
        struct.pack_into("<f", bad_payload, 12 + 4 * 2, 0.0)  # stage0 ref real=0
        chunks = [
            _point_packets(float(freqs[0]), 0),
            encode_packet(VNA_DATAPOINT, bytes(bad_payload)),
        ]
        backend, transport, _ = _make_backend(sweep_chunks=chunks)
        transport.feed_reads(_full_sweep(freqs, start_offset=2))
        frame = backend.acquire_one()
        self.assertEqual(frame.trace_index, 0)
        stats = backend.statistics
        self.assertGreaterEqual(stats.invalid_points, 1)
        self.assertGreaterEqual(stats.dropped_sweeps, 1)
        backend.close()

    def test_unexpected_ack_and_ignored_packets_counted(self) -> None:
        freqs = _freqs()
        chunks = [
            encode_packet(ACK),
            encode_packet(DEVICE_STATUS),
            _point_packets(float(freqs[0]), 0),
        ]
        backend, transport, _ = _make_backend(sweep_chunks=chunks)
        for i in range(1, 5):
            transport.feed_reads([_point_packets(float(freqs[i]), i)])
        frame = backend.acquire_one()
        self.assertEqual(frame.trace_index, 0)
        stats = backend.statistics
        self.assertEqual(stats.unexpected_acks, 1)
        self.assertEqual(stats.ignored_packets, 1)
        backend.close()

    def test_101_point_sweep(self) -> None:
        freqs = list(np.linspace(0.5e9, 2.5e9, 101))
        backend, _transport, _ = _make_backend(
            config=_sweep_settings(n_points=101), sweep_chunks=_full_sweep(freqs)
        )
        frame = backend.acquire_one()
        self.assertEqual(frame.frequency_hz.size, 101)
        self.assertEqual(frame.raw.shape, (2, 101))
        backend.close()


# ================================================================ pending


class BackendPendingTests(unittest.TestCase):
    def test_configure_ack_and_datapoints_same_read_preserved(self) -> None:
        freqs = _freqs()
        # SWEEP_SETTINGS ACK 与前两个 datapoint 同一次 USB read：不得丢失
        transport = _ScriptedTransport([
            encode_packet(DEVICE_INFO, _device_info_payload()),
            encode_packet(ACK),  # open SET_IDLE
            encode_packet(ACK),  # configure SET_IDLE
            encode_packet(ACK)
            + _point_packets(float(freqs[0]), 0)
            + _point_packets(float(freqs[1]), 1),
            *[_point_packets(float(freqs[i]), i) for i in range(2, 5)],
        ])
        backend, transport, _ = _make_backend(transport=transport)
        frame = backend.acquire_one()
        self.assertEqual(frame.trace_index, 0)
        np.testing.assert_allclose(frame.frequency_hz, freqs, atol=1e-3)
        np.testing.assert_allclose(frame.s11[0], complex(0.5, -0.2), atol=1e-6)
        backend.close()

    def test_acquire_one_consumes_pending_before_usb_read(self) -> None:
        # 第一道完成后同批的第二道全部进 pending；第二次 acquire_one 立即返回
        #（不依赖新的 USB read）。
        freqs = _freqs()
        sweep1 = b"".join(_point_packets(float(freqs[i]), i) for i in range(5))
        sweep2 = b"".join(_point_packets(float(freqs[i]) + 1, i) for i in range(5))
        backend, transport, _ = _make_backend(sweep_chunks=[sweep1 + sweep2])
        backend.acquire_one()
        self.assertGreaterEqual(backend.statistics.pending_depth_peak, 5)
        reads_before = len(transport.writes)  # 采集中不应有写
        frame2 = backend.acquire_one()
        self.assertEqual(len(transport.writes), reads_before)
        self.assertEqual(frame2.trace_index, 1)
        backend.close()

    def test_pending_overflow_fails_closed(self) -> None:
        freqs = _freqs()
        # 上限 2：第一道完成（held）后，第二道第 3 个点即溢出 -> FAULTED
        sweep1 = b"".join(_point_packets(float(freqs[i]), i) for i in range(5))
        sweep2 = b"".join(_point_packets(float(freqs[i]) + 1, i) for i in range(5))
        backend, _transport, _ = _make_backend(
            sweep_chunks=[sweep1 + sweep2], pending_max=2
        )
        with self.assertRaises(LibreVnaPendingOverflowError):
            backend.acquire_one()
        self.assertIs(backend.state, BackendState.FAULTED)
        self.assertGreaterEqual(backend.statistics.pending_overflows, 1)
        backend.close()

    def test_configure_ack_overflow_fails_closed(self) -> None:
        """configure ACK 阶段 pending_max 必须生效：ACK 与 3 个 datapoint 同 read。

        pending_max=2 时第 3 个 datapoint 溢出 -> LibreVnaPendingOverflowError +
        FAULTED；不得静默丢弃最旧或最新 datapoint。
        """
        transport = _ScriptedTransport([
            encode_packet(DEVICE_INFO, _device_info_payload()),
            encode_packet(ACK),  # open SET_IDLE
            encode_packet(ACK),  # configure SET_IDLE
            encode_packet(ACK)  # configure SWEEP_SETTINGS ACK + 3 datapoints
            + _point_packets(5e8, 0)
            + _point_packets(1e9, 1)
            + _point_packets(1.5e9, 2),
        ])
        backend = PersistentLibreVnaBackend(
            transport=transport,
            settings=PersistentBackendSettings(
                command_timeout_s=0.05, usb_read_timeout_ms=1, pending_max=2
            ),
        )
        backend.open()
        with self.assertRaises(LibreVnaPendingOverflowError):
            backend.configure(_sweep_settings())
        self.assertIs(backend.state, BackendState.FAULTED)
        stats = backend.statistics
        self.assertGreaterEqual(stats.pending_overflows, 1)
        self.assertGreaterEqual(stats.pending_depth_peak, 2)
        self.assertEqual(stats.faults, 1)
        backend.close()


# ================================================================ pause/resume


class BackendPauseResumeTests(unittest.TestCase):
    def test_pause_preserves_transport_config_trace_and_mode(self) -> None:
        freqs = _freqs()
        backend, transport, cfg = _make_backend(sweep_chunks=_full_sweep(freqs))
        frame0 = backend.acquire_one()
        self.assertEqual(frame0.trace_index, 0)
        stats_before = backend.statistics.to_dict()
        transport.feed_reads([encode_packet(ACK)])
        backend.pause()
        self.assertIs(backend.state, BackendState.PAUSED)
        # transport 保持打开，不 close/reopen
        self.assertTrue(transport.is_open)
        self.assertEqual(transport.open_count, 1)
        self.assertEqual(transport.close_count, 0)
        # config 身份、trace_index、统计保持
        self.assertIs(backend.config, cfg)
        self.assertEqual(backend.statistics.trace_index, 1)
        for key, value in stats_before.items():
            self.assertGreaterEqual(
                backend.statistics.to_dict()[key], value, f"{key} 不得倒退"
            )
        # 边界残留清空：pending 空、PacketStream 空、assembler 当前态为空
        self.assertEqual(len(backend._pending), 0)  # noqa: SLF001
        self.assertEqual(backend._stream.buffer, b"")  # noqa: SLF001
        self.assertIsNotNone(backend._assembler)  # noqa: SLF001
        self.assertEqual(backend._assembler._points, [])  # noqa: SLF001
        backend.close()

    def test_resume_resends_same_sweep_settings_no_reopen(self) -> None:
        freqs = _freqs()
        backend, transport, _ = _make_backend(sweep_chunks=_full_sweep(freqs))
        backend.acquire_one()
        configure_writes = transport.sweep_settings_writes()
        self.assertEqual(len(configure_writes), 1)
        transport.feed_reads([encode_packet(ACK)])
        backend.pause()
        transport.feed_reads([
            encode_packet(ACK) + _point_packets(float(freqs[0]), 0),
            *[_point_packets(float(freqs[i]), i) for i in range(1, 5)],
        ])
        backend.resume()
        self.assertIs(backend.state, BackendState.STREAMING)
        resume_writes = transport.sweep_settings_writes()
        self.assertEqual(len(resume_writes), 2)
        # resume 发送与 configure 字节等价（同一冻结 SweepSettings）
        self.assertEqual(resume_writes[1], resume_writes[0])
        self.assertEqual(transport.open_count, 1)
        self.assertEqual(transport.close_count, 0)
        self.assertTrue(transport.is_open)
        frame1 = backend.acquire_one()
        self.assertEqual(frame1.trace_index, 1)  # trace 从原值继续
        backend.close()

    def test_multiple_pause_resume_cycles_trace_continuous(self) -> None:
        freqs = _freqs()
        backend, transport, _ = _make_backend(sweep_chunks=_full_sweep(freqs))
        backend.acquire_one()
        for cycle in range(3):
            transport.feed_reads([encode_packet(ACK)])
            backend.pause()
            self.assertIs(backend.state, BackendState.PAUSED)
            transport.feed_reads([
                encode_packet(ACK) + _point_packets(float(freqs[0]), 0),
                *[_point_packets(float(freqs[i]), i) for i in range(1, 5)],
            ])
            backend.resume()
            frame = backend.acquire_one()
            self.assertEqual(frame.trace_index, cycle + 1)
        self.assertEqual(backend.statistics.trace_index, 4)
        backend.close()

    def test_full_next_sweep_during_pause_ack_wait_discarded(self) -> None:
        freqs = _freqs()
        sweep0 = _full_sweep(freqs)
        sweep1 = [_point_packets(float(f) + 1, i) for i, f in enumerate(freqs)]
        backend, transport, _ = _make_backend(
            sweep_chunks=[*sweep0, *sweep1, encode_packet(ACK)]
        )
        frame0 = backend.acquire_one()
        self.assertEqual(frame0.trace_index, 0)
        backend.pause()
        # sweep1 在 ACK 等待期间被接收，随后被 pause 丢弃（不返回、不拼入）
        self.assertEqual(len(backend._pending), 0)  # noqa: SLF001
        self.assertIsNotNone(backend._assembler)  # noqa: SLF001
        self.assertEqual(backend._assembler._points, [])  # noqa: SLF001
        self.assertEqual(backend.statistics.trace_index, 1)
        transport.feed_reads([
            encode_packet(ACK) + _point_packets(float(freqs[0]), 0),
            *[_point_packets(float(freqs[i]), i) for i in range(1, 5)],
        ])
        backend.resume()
        frame1 = backend.acquire_one()
        self.assertEqual(frame1.trace_index, 1)
        np.testing.assert_allclose(frame1.frequency_hz, freqs, atol=1e-3)
        backend.close()

    def test_partial_pre_pause_not_stitched_with_resume(self) -> None:
        freqs = _freqs()
        backend, transport, _ = _make_backend(sweep_chunks=[
            *_full_sweep(freqs),
            _point_packets(float(freqs[0]) + 1, 0),  # partial pre-pause sweep
            _point_packets(float(freqs[1]) + 1, 1),
            encode_packet(ACK),
        ])
        backend.acquire_one()
        backend.pause()
        # resume 后完整新 sweep（point 0 起步）——不把暂停前 partial 拼入
        transport.feed_reads([
            encode_packet(ACK) + _point_packets(float(freqs[0]), 0),
            *[_point_packets(float(freqs[i]), i) for i in range(1, 5)],
        ])
        backend.resume()
        frame = backend.acquire_one()
        self.assertEqual(frame.trace_index, 1)
        self.assertEqual(frame.frequency_hz.size, 5)
        np.testing.assert_allclose(frame.frequency_hz, freqs, atol=1e-3)
        backend.close()

    def test_resume_ack_and_datapoints_same_read_preserved(self) -> None:
        freqs = _freqs()
        backend, transport, _ = _make_backend(sweep_chunks=_full_sweep(freqs))
        backend.acquire_one()
        transport.feed_reads([encode_packet(ACK)])
        backend.pause()
        # resume ACK + 前两个 datapoint 同一次 USB read
        transport.feed_reads([
            encode_packet(ACK)
            + _point_packets(float(freqs[0]), 0)
            + _point_packets(float(freqs[1]), 1),
            *[_point_packets(float(freqs[i]), i) for i in range(2, 5)],
        ])
        backend.resume()
        frame = backend.acquire_one()
        self.assertEqual(frame.trace_index, 1)
        np.testing.assert_allclose(frame.frequency_hz, freqs, atol=1e-3)
        backend.close()

    def test_resume_ignores_non_point_zero_start(self) -> None:
        freqs = _freqs()
        backend, transport, _ = _make_backend(sweep_chunks=_full_sweep(freqs))
        backend.acquire_one()
        transport.feed_reads([encode_packet(ACK)])
        backend.pause()
        # resume ACK + 非 point 0 的 datapoint（point 3）——不得组装成帧
        transport.feed_reads([
            encode_packet(ACK) + _point_packets(float(freqs[3]) + 1, 3),
            *_full_sweep(freqs),
        ])
        backend.resume()
        frame = backend.acquire_one()
        self.assertEqual(frame.trace_index, 1)
        self.assertGreaterEqual(backend.statistics.dropped_sweeps, 1)
        np.testing.assert_allclose(frame.frequency_hz, freqs, atol=1e-3)
        backend.close()

    def test_exact_command_sequence(self) -> None:
        freqs = _freqs()
        backend, transport, _ = _make_backend(sweep_chunks=[
            *_full_sweep(freqs),
            encode_packet(ACK),  # pause SET_IDLE ACK
            encode_packet(ACK) + _point_packets(float(freqs[0]), 0),  # resume ACK
        ])
        backend.acquire_one()
        backend.pause()
        backend.resume()
        backend.close()
        self.assertEqual(
            transport.sent_types(),
            [REQUEST_DEVICE_INFO, SET_IDLE, SET_IDLE, SWEEP_SETTINGS,
             SET_IDLE, SWEEP_SETTINGS, SET_IDLE],
        )
        self.assertEqual(transport.open_count, 1)
        self.assertEqual(transport.close_count, 1)


# ================================================================ 错误路径


class BackendErrorPathTests(unittest.TestCase):
    @staticmethod
    def _raw_backend(transport: _ScriptedTransport) -> PersistentLibreVnaBackend:
        return PersistentLibreVnaBackend(
            transport=transport,
            settings=PersistentBackendSettings(
                command_timeout_s=0.05, usb_read_timeout_ms=1
            ),
        )

    def test_configure_nack_faults(self) -> None:
        transport = _ScriptedTransport([
            encode_packet(DEVICE_INFO, _device_info_payload()),
            encode_packet(ACK),   # open SET_IDLE
            encode_packet(NACK),  # configure SET_IDLE -> NACK
        ])
        backend = self._raw_backend(transport)
        backend.open()
        with self.assertRaises(LibreVnaNackError):
            backend.configure(_sweep_settings())
        self.assertIs(backend.state, BackendState.FAULTED)
        self.assertIsNone(backend.config)
        self.assertEqual(backend.statistics.faults, 1)
        # FAULTED 下除 close 外全部拒绝
        with self.assertRaises(LibreVnaStateError):
            backend.acquire_one()
        with self.assertRaises(LibreVnaStateError):
            backend.configure(_sweep_settings())
        with self.assertRaises(LibreVnaStateError):
            backend.pause()
        with self.assertRaises(LibreVnaStateError):
            backend.resume()
        backend.close()
        self.assertIs(backend.state, BackendState.CLOSED)

    def test_configure_ack_timeout_faults(self) -> None:
        transport = _ScriptedTransport([
            encode_packet(DEVICE_INFO, _device_info_payload()),
            encode_packet(ACK),  # open SET_IDLE
            encode_packet(ACK),  # configure SET_IDLE
            # 不喂 SWEEP_SETTINGS ACK -> 超时
        ])
        backend = self._raw_backend(transport)
        backend.open()
        with self.assertRaises(LibreVnaTimeoutError):
            backend.configure(_sweep_settings())
        self.assertIs(backend.state, BackendState.FAULTED)
        backend.close()

    def test_pause_nack_faults(self) -> None:
        backend, transport, _ = _make_backend()
        transport.feed_reads([encode_packet(NACK)])
        with self.assertRaises(LibreVnaNackError):
            backend.pause()
        self.assertIs(backend.state, BackendState.FAULTED)
        self.assertIsNone(backend.config)
        backend.close()

    def test_pause_ack_timeout_faults(self) -> None:
        backend, _transport, _ = _make_backend()
        with self.assertRaises(LibreVnaTimeoutError):
            backend.pause()
        self.assertIs(backend.state, BackendState.FAULTED)
        backend.close()

    def test_resume_nack_faults(self) -> None:
        backend, transport, _ = _make_backend()
        transport.feed_reads([encode_packet(ACK)])
        backend.pause()
        transport.feed_reads([encode_packet(NACK)])
        with self.assertRaises(LibreVnaNackError):
            backend.resume()
        self.assertIs(backend.state, BackendState.FAULTED)
        backend.close()

    def test_resume_ack_timeout_faults(self) -> None:
        backend, transport, _ = _make_backend()
        transport.feed_reads([encode_packet(ACK)])
        backend.pause()
        with self.assertRaises(LibreVnaTimeoutError):
            backend.resume()
        self.assertIs(backend.state, BackendState.FAULTED)
        backend.close()

    def test_close_releases_even_if_set_idle_times_out(self) -> None:
        # 脚本不含 close 的 ACK -> close 的 SET_IDLE 超时后仍释放 USB
        backend, transport, _ = _make_backend()
        backend.close()
        self.assertEqual(transport.close_count, 1)
        self.assertFalse(transport.is_open)
        self.assertIs(backend.state, BackendState.CLOSED)

    def test_close_releases_even_if_set_idle_nacks(self) -> None:
        transport = _ScriptedTransport([
            encode_packet(DEVICE_INFO, _device_info_payload()),
            encode_packet(ACK), encode_packet(ACK), encode_packet(ACK),
            encode_packet(NACK),  # close SET_IDLE -> NACK（尽力而为，吞掉）
        ])
        backend = self._raw_backend(transport)
        backend.open()
        backend.configure(_sweep_settings())
        backend.close()
        self.assertEqual(transport.close_count, 1)
        self.assertFalse(transport.is_open)
        self.assertIs(backend.state, BackendState.CLOSED)

    def test_faulted_reopen_requires_close_then_open(self) -> None:
        backend, transport, _ = _make_backend()
        transport.feed_reads([encode_packet(NACK)])
        with self.assertRaises(LibreVnaNackError):
            backend.pause()
        self.assertIs(backend.state, BackendState.FAULTED)
        # FAULTED 中 open 被拒绝
        with self.assertRaises(LibreVnaStateError):
            backend.open()
        backend.close()
        # close 后可重新 open（新会话，connection_generation 递增）
        transport.feed_reads([
            encode_packet(DEVICE_INFO, _device_info_payload()),
            encode_packet(ACK),
        ])
        backend.open()
        self.assertIs(backend.state, BackendState.OPEN_IDLE)
        self.assertEqual(backend.statistics.connection_generation, 2)
        backend.close()

    def test_close_release_error_enters_faulted_when_still_open(self) -> None:
        # transport.close 抛释放错误且 transport 仍报告打开 -> LibreVnaReleaseError，
        # 且状态为 FAULTED（不得伪装安全 CLOSED）；恢复可关闭后 close 重试成功。
        backend, transport, _ = _make_backend()

        def _failing_close() -> None:
            raise RuntimeError("release_interface failed")

        transport.close = _failing_close  # type: ignore[method-assign]
        with self.assertRaises(LibreVnaReleaseError):
            backend.close()
        self.assertIs(backend.state, BackendState.FAULTED)
        self.assertTrue(transport.is_open)
        # FAULTED 拒绝 open / configure / acquire
        with self.assertRaises(LibreVnaStateError):
            backend.open()
        with self.assertRaises(LibreVnaStateError):
            backend.configure(_sweep_settings())
        with self.assertRaises(LibreVnaStateError):
            backend.acquire_one()
        # 恢复可关闭后 close 重试成功 -> CLOSED
        del transport.close  # 恢复类方法绑定
        backend.close()
        self.assertIs(backend.state, BackendState.CLOSED)
        self.assertFalse(transport.is_open)

    def test_normal_close_error_after_release_goes_closed(self) -> None:
        # 普通会话 close：transport.close() 先置 is_open=False 再抛错 ->
        # 资源已释放，状态 CLOSED，但仍抛 LibreVnaReleaseError（保留释放错误）。
        backend, transport, _ = _make_backend()

        def _release_then_raise() -> None:
            transport.opened = False
            raise RuntimeError("dispose boom")

        transport.close = _release_then_raise  # type: ignore[method-assign]
        with self.assertRaises(LibreVnaReleaseError) as ctx:
            backend.close()
        self.assertIn("dispose boom", str(ctx.exception))
        self.assertIs(backend.state, BackendState.CLOSED)
        self.assertFalse(transport.is_open)
        backend.close()  # 幂等

    def test_exception_note_helper_is_python310_compatible(self) -> None:
        # _add_exc_note：Python 3.11+ 走内置 add_note()，3.10 走手工 __notes__；
        # 均不改变原始 str(exception)，且诊断机制自身绝不抛异常。
        from librevna.librevna_continuous import _add_exc_note

        exc = RuntimeError("open boom")
        _add_exc_note(exc, "first")
        _add_exc_note(exc, "second")
        self.assertEqual(str(exc), "open boom")
        self.assertEqual(getattr(exc, "__notes__", []), ["first", "second"])

        # 无 add_note() 的对象走 3.10 兼容的 __notes__ 手工路径
        class _NoteHolder:
            pass

        holder = _NoteHolder()
        _add_exc_note(holder, "holder-note")
        self.assertEqual(getattr(holder, "__notes__", []), ["holder-note"])
        # 非字符串 note 不抛异常
        _add_exc_note(exc, None)  # type: ignore[arg-type]
        self.assertEqual(str(exc), "open boom")

    def test_repeated_release_failure_stays_faulted_then_recovers(self) -> None:
        # open 失败后首次清理失败（transport 仍打开）-> FAULTED（faults=1）；
        # 再 close() 仍失败且 is_open=True -> 保持 FAULTED（faults 不重复）；
        # open/configure/acquire 继续拒绝；恢复可关闭后 close 重试最终 CLOSED。
        transport = _FailingOpenTransport(
            open_raises=RuntimeError("open boom"),
            close_raises=RuntimeError("close boom"),
        )
        backend = PersistentLibreVnaBackend(
            transport=transport,
            settings=PersistentBackendSettings(
                command_timeout_s=0.05, usb_read_timeout_ms=1
            ),
        )
        with self.assertRaises(RuntimeError):
            backend.open()
        self.assertIs(backend.state, BackendState.FAULTED)
        self.assertEqual(backend.statistics.faults, 1)
        # 连续释放失败：再次 close 仍失败且 is_open=True -> 保持 FAULTED
        with self.assertRaises(LibreVnaReleaseError):
            backend.close()
        self.assertIs(backend.state, BackendState.FAULTED)
        self.assertTrue(transport.is_open)
        self.assertEqual(backend.statistics.faults, 1)  # 不重复计数
        # FAULTED 下继续拒绝
        with self.assertRaises(LibreVnaStateError):
            backend.open()
        with self.assertRaises(LibreVnaStateError):
            backend.configure(_sweep_settings())
        with self.assertRaises(LibreVnaStateError):
            backend.acquire_one()
        # 恢复可关闭后 close 重试成功 -> CLOSED
        transport.make_closable()
        backend.close()
        self.assertIs(backend.state, BackendState.CLOSED)
        self.assertFalse(transport.is_open)

    def test_open_failure_close_released_then_raised_goes_closed(self) -> None:
        # open 失败清理：transport.close() 先置 is_open=False 再抛 "dispose boom"
        # -> 已释放，backend CLOSED；原始 open 异常保持不变；诊断含 "dispose boom"。
        transport = _FailingOpenTransport(
            open_raises=RuntimeError("open boom"),
            close_raises=RuntimeError("dispose boom"),
            close_release_then_raise=True,
        )
        backend = PersistentLibreVnaBackend(
            transport=transport,
            settings=PersistentBackendSettings(
                command_timeout_s=0.05, usb_read_timeout_ms=1
            ),
        )
        with self.assertRaises(RuntimeError) as ctx:
            backend.open()
        self.assertEqual(str(ctx.exception), "open boom")  # 原始异常未替换
        notes = list(getattr(ctx.exception, "__notes__", []))
        self.assertTrue(any("dispose boom" in note for note in notes))
        self.assertIs(backend.state, BackendState.CLOSED)
        self.assertFalse(transport.is_open)
        backend.close()  # 幂等

    def test_close_returning_while_still_open_raises_and_faults(self) -> None:
        # transport.close() 正常返回但 transport 仍报告打开：close() 不得静默
        # 成功——必须抛 LibreVnaReleaseError（说明 close 已返回但仍打开）、
        # 状态 FAULTED、faults=1、拒绝 open/configure/acquire；恢复后重试成功。
        backend, transport, _ = _make_backend()

        def _silent_keep_open() -> None:
            pass  # 不抛错、不把 opened 改为 False

        transport.close = _silent_keep_open  # type: ignore[method-assign]
        with self.assertRaises(LibreVnaReleaseError) as ctx:
            backend.close()
        self.assertIn("transport.close() 已返回但 transport 仍报告打开", str(ctx.exception))
        self.assertIs(backend.state, BackendState.FAULTED)
        self.assertTrue(transport.is_open)
        self.assertEqual(backend.statistics.faults, 1)
        # FAULTED 拒绝 open / configure / acquire
        with self.assertRaises(LibreVnaStateError):
            backend.open()
        with self.assertRaises(LibreVnaStateError):
            backend.configure(_sweep_settings())
        with self.assertRaises(LibreVnaStateError):
            backend.acquire_one()
        # 恢复正常 close 后重试 -> CLOSED，随后 close 幂等
        del transport.close  # 恢复类方法绑定
        backend.close()
        self.assertIs(backend.state, BackendState.CLOSED)
        self.assertFalse(transport.is_open)
        backend.close()  # 幂等

    def test_repeated_silent_close_failure_stays_faulted(self) -> None:
        # 连续两次 transport.close 正常返回但仍保持打开：
        # 两次 backend.close() 都必须抛 LibreVnaReleaseError；状态始终 FAULTED；
        # faults 始终为 1；恢复后可成功关闭。
        backend, transport, _ = _make_backend()

        def _silent_keep_open() -> None:
            pass

        transport.close = _silent_keep_open  # type: ignore[method-assign]
        for _ in range(2):
            with self.assertRaises(LibreVnaReleaseError) as ctx:
                backend.close()
            self.assertIn("仍报告打开", str(ctx.exception))
            self.assertIs(backend.state, BackendState.FAULTED)
            self.assertTrue(transport.is_open)
            self.assertEqual(backend.statistics.faults, 1)  # 不重复计数
        # 恢复后成功关闭
        del transport.close
        backend.close()
        self.assertIs(backend.state, BackendState.CLOSED)
        self.assertFalse(transport.is_open)

    def test_open_failure_close_returns_but_still_open_symmetry(self) -> None:
        # open 失败清理的对称场景：transport.open() 部分打开后抛 "open boom"，
        # 清理 close 不抛错但仍保持打开 -> 最终抛出的仍是原始 RuntimeError，
        # note 含 "close 返回但仍打开" 诊断，状态 FAULTED、is_open=True；
        # 恢复后 close 成功。
        transport = _FailingOpenTransport(
            open_raises=RuntimeError("open boom"),
            close_silently_keeps_open=True,
        )
        backend = PersistentLibreVnaBackend(
            transport=transport,
            settings=PersistentBackendSettings(
                command_timeout_s=0.05, usb_read_timeout_ms=1
            ),
        )
        with self.assertRaises(RuntimeError) as ctx:
            backend.open()
        self.assertEqual(str(ctx.exception), "open boom")  # 原始异常未替换
        notes = list(getattr(ctx.exception, "__notes__", []))
        self.assertTrue(
            any("transport.close()" in note and "仍报告打开" in note for note in notes)
        )
        self.assertIs(backend.state, BackendState.FAULTED)
        self.assertTrue(transport.is_open)
        self.assertEqual(backend.statistics.faults, 1)
        # 恢复可关闭后 close 成功 -> CLOSED
        transport.make_closable()
        backend.close()
        self.assertIs(backend.state, BackendState.CLOSED)
        self.assertFalse(transport.is_open)


# ================================================================ 配置校验


class BackendConfigValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.backend, self.transport = _open_backend()

    def tearDown(self) -> None:
        self.backend.close()

    def _assert_rejected(self, settings: SweepSettings, fragment: str) -> None:
        writes_before = len(self.transport.writes)
        with self.assertRaises(LibreVnaUnsupportedConfigError) as ctx:
            self.backend.configure(settings)
        self.assertIn(fragment, str(ctx.exception))
        # 预校验在任何 USB 写入前完成（不发送命令、不破坏会话）
        self.assertEqual(len(self.transport.writes), writes_before)
        self.assertIs(self.backend.state, BackendState.OPEN_IDLE)

    def test_single_channel_0x1240_rejected(self) -> None:
        self._assert_rejected(
            _sweep_settings(stages_bitmap=S11_STAGES_BITMAP), "0x1241"
        )

    def test_frequency_out_of_range_rejected(self) -> None:
        self._assert_rejected(_sweep_settings(stop_hz=7_000_000_000), "频率范围")

    def test_ifbw_out_of_range_rejected(self) -> None:
        self._assert_rejected(_sweep_settings(ifbw_hz=10_000_000), "IFBW")

    def test_too_many_points_rejected(self) -> None:
        self._assert_rejected(_sweep_settings(n_points=100_000), "点")

    def test_num_ports_1_rejected(self) -> None:
        transport = _ScriptedTransport([
            encode_packet(DEVICE_INFO, _device_info_payload(num_ports=1)),
            encode_packet(ACK),
        ])
        backend = PersistentLibreVnaBackend(
            transport=transport,
            settings=PersistentBackendSettings(
                command_timeout_s=0.05, usb_read_timeout_ms=1
            ),
        )
        backend.open()
        with self.assertRaises(LibreVnaUnsupportedConfigError):
            backend.configure(_sweep_settings())
        backend.close()


# ================================================================ 验收：100 道


class BackendHundredSweepTests(unittest.TestCase):
    def test_100_sweeps_one_open_configure(self) -> None:
        n_points = 5
        freqs = list(np.linspace(0.5e9, 2.5e9, n_points))
        chunks: list = [
            encode_packet(DEVICE_INFO, _device_info_payload()),
            encode_packet(ACK),  # open SET_IDLE
            encode_packet(ACK),  # configure SET_IDLE
            encode_packet(ACK),  # configure SWEEP_SETTINGS
        ]
        for s in range(100):
            chunks.append(
                b"".join(
                    _point_packets(float(freqs[i]) + s, i) for i in range(n_points)
                )
            )
        transport = _ScriptedTransport(chunks)
        backend = PersistentLibreVnaBackend(
            transport=transport,
            settings=PersistentBackendSettings(
                command_timeout_s=0.05, usb_read_timeout_ms=1, sweep_timeout_s=2.0
            ),
        )
        backend.open()
        backend.configure(_sweep_settings(n_points=n_points))
        self.assertEqual(backend.statistics.open_count, 1)
        self.assertEqual(backend.statistics.configure_count, 1)
        self.assertEqual(backend.statistics.connection_generation, 1)
        self.assertEqual(len(transport.sweep_settings_writes()), 1)
        for s in range(100):
            frame = backend.acquire_one()
            self.assertEqual(frame.trace_index, s)
            np.testing.assert_allclose(
                frame.frequency_hz, [f + s for f in freqs], atol=1e-3
            )
            self.assertEqual(frame.statistics["completed_sweeps"], s + 1)
        # 采集全程不写任何命令；不重新 open/configure
        self.assertEqual(
            transport.sent_types(),
            [REQUEST_DEVICE_INFO, SET_IDLE, SET_IDLE, SWEEP_SETTINGS],
        )
        self.assertEqual(transport.open_count, 1)
        self.assertEqual(backend.statistics.trace_index, 100)
        self.assertEqual(backend.statistics.completed_sweeps, 100)
        self.assertEqual(backend.statistics.dropped_sweeps, 0)
        backend.close()
        self.assertEqual(transport.close_count, 1)
        self.assertIs(backend.state, BackendState.CLOSED)


# ================================================================ 统计 / 注入 / legacy


class SessionStatisticsTests(unittest.TestCase):
    def test_initial_backend_statistics(self) -> None:
        backend = PersistentLibreVnaBackend(transport=_ScriptedTransport())
        self.assertIs(backend.state, BackendState.CLOSED)
        stats = backend.statistics
        self.assertEqual(stats.open_count, 0)
        self.assertEqual(stats.connection_generation, 0)
        d = stats.to_dict()
        self.assertEqual(set(d), set(SessionStatistics().__dict__))
        self.assertTrue(all(isinstance(v, int) for v in d.values()))

    def test_statistics_snapshot_is_fresh_copy(self) -> None:
        backend = PersistentLibreVnaBackend(transport=_ScriptedTransport())
        first = backend.statistics
        first.trace_index = 999
        self.assertEqual(backend.statistics.trace_index, 0)


class SessionStatisticsPersistenceTests(unittest.TestCase):
    """close/FAULTED 后统计保留；重新 open 才重置会话统计（累计字段保留）。"""

    def test_close_preserves_assembler_statistics(self) -> None:
        freqs = _freqs()
        backend, transport, _ = _make_backend(sweep_chunks=_full_sweep(freqs))
        backend.acquire_one()
        transport.feed_reads(_full_sweep(freqs, start_offset=1))
        backend.acquire_one()
        before = backend.statistics
        self.assertEqual(before.trace_index, 2)
        self.assertEqual(before.completed_sweeps, 2)
        backend.close()
        after = backend.statistics
        self.assertEqual(after.trace_index, 2)
        self.assertEqual(after.completed_sweeps, 2)
        self.assertEqual(after.dropped_sweeps, 0)
        # 多次读取不变化
        self.assertEqual(after.to_dict(), backend.statistics.to_dict())
        backend.close()  # 幂等，统计不变
        self.assertEqual(backend.statistics.to_dict(), after.to_dict())

    def test_fault_preserves_statistics(self) -> None:
        freqs = _freqs()
        backend, transport, _ = _make_backend(sweep_chunks=_full_sweep(freqs))
        frame0 = backend.acquire_one()
        self.assertEqual(frame0.trace_index, 0)
        self.assertEqual(backend.statistics.completed_sweeps, 1)
        # 下一次 acquire 触发 NACK -> FAULTED；故障前统计保留
        transport.feed_reads([encode_packet(NACK)])
        with self.assertRaises(LibreVnaNackError):
            backend.acquire_one()
        self.assertIs(backend.state, BackendState.FAULTED)
        stats = backend.statistics
        self.assertEqual(stats.trace_index, 1)
        self.assertEqual(stats.completed_sweeps, 1)
        self.assertEqual(stats.dropped_sweeps, 0)
        self.assertEqual(stats.faults, 1)
        self.assertEqual(stats.to_dict(), backend.statistics.to_dict())
        backend.close()

    def test_fault_after_overflow_preserves_completed_sweeps(self) -> None:
        freqs = _freqs()
        backend, transport, _ = _make_backend(
            sweep_chunks=_full_sweep(freqs), pending_max=1
        )
        backend.acquire_one()  # 交付 sweep1 -> completed_sweeps=1, trace_index=1
        self.assertEqual(backend.statistics.completed_sweeps, 1)
        self.assertEqual(backend.statistics.trace_index, 1)
        # 下一道完成（held）后再来 2 个点 -> pending_max=1 第 2 个即溢出 -> FAULTED；
        # assembler 已看到 2 道，统计保留（completed_sweeps=2, trace_index=1）
        sweep2 = b"".join(_point_packets(float(freqs[i]) + 1, i) for i in range(5))
        extra = (
            _point_packets(float(freqs[0]) + 2, 0)
            + _point_packets(float(freqs[1]) + 2, 1)
        )
        transport.feed_reads([sweep2 + extra])
        with self.assertRaises(LibreVnaPendingOverflowError):
            backend.acquire_one()
        self.assertIs(backend.state, BackendState.FAULTED)
        stats = backend.statistics
        self.assertEqual(stats.completed_sweeps, 2)
        self.assertEqual(stats.trace_index, 1)
        self.assertGreaterEqual(stats.pending_overflows, 1)
        # 多次读取稳定
        self.assertEqual(stats.to_dict(), backend.statistics.to_dict())
        backend.close()

    def test_reopen_resets_session_stats_keeps_cumulative(self) -> None:
        freqs = _freqs()
        backend, transport, _ = _make_backend(sweep_chunks=_full_sweep(freqs))
        backend.acquire_one()
        backend.close()
        self.assertEqual(backend.statistics.trace_index, 1)
        self.assertEqual(backend.statistics.completed_sweeps, 1)
        # reopen -> 新会话：trace/completed 重置；open_count/generation 累计
        transport.feed_reads([
            encode_packet(DEVICE_INFO, _device_info_payload()),
            encode_packet(ACK),
        ])
        backend.open()
        self.assertEqual(backend.statistics.open_count, 2)
        self.assertEqual(backend.statistics.connection_generation, 2)
        self.assertEqual(backend.statistics.trace_index, 0)
        self.assertEqual(backend.statistics.completed_sweeps, 0)
        # 新会话正常采集
        transport.feed_reads([encode_packet(ACK), encode_packet(ACK)])
        backend.configure(_sweep_settings())
        transport.feed_reads(_full_sweep(freqs))
        frame = backend.acquire_one()
        self.assertEqual(frame.trace_index, 0)
        self.assertEqual(backend.statistics.completed_sweeps, 1)
        backend.close()


class PersistentBackendSettingsValidationTests(unittest.TestCase):
    """PersistentBackendSettings 严格校验：bool/非 Real -> TypeError；NaN/Inf/0/负 -> ValueError。"""

    def test_bool_rejected(self) -> None:
        for kwargs in (
            {"usb_read_timeout_ms": True},
            {"device_info_timeout_s": True},
            {"command_timeout_s": True},
            {"sweep_timeout_s": True},
            {"pending_max": True},
        ):
            with self.assertRaises(TypeError):
                PersistentBackendSettings(**kwargs)  # type: ignore[arg-type]

    def test_non_real_rejected(self) -> None:
        for kwargs in (
            {"command_timeout_s": "2"},
            {"device_info_timeout_s": "2"},
            {"sweep_timeout_s": "2"},
            {"usb_read_timeout_ms": "2"},
            {"usb_read_timeout_ms": 1.5},
            {"pending_max": 2.5},
            {"command_timeout_s": object()},
        ):
            with self.assertRaises(TypeError):
                PersistentBackendSettings(**kwargs)  # type: ignore[arg-type]

    def test_nan_inf_zero_negative_rejected(self) -> None:
        for bad in (float("nan"), float("inf"), -float("inf"), 0, -1, -0.5):
            for field in (
                "device_info_timeout_s",
                "command_timeout_s",
                "sweep_timeout_s",
            ):
                with self.assertRaises(ValueError):
                    PersistentBackendSettings(**{field: bad})
        with self.assertRaises(ValueError):
            PersistentBackendSettings(usb_read_timeout_ms=0)
        with self.assertRaises(ValueError):
            PersistentBackendSettings(usb_read_timeout_ms=-5)
        with self.assertRaises(ValueError):
            PersistentBackendSettings(pending_max=0)

    def test_valid_python_and_numpy_values_accepted(self) -> None:
        s = PersistentBackendSettings(
            usb_read_timeout_ms=50,
            device_info_timeout_s=3,
            command_timeout_s=0.05,
            sweep_timeout_s=2.0,
            pending_max=2048,
        )
        self.assertEqual(s.usb_read_timeout_ms, 50)
        self.assertEqual(s.sweep_timeout_s, 2.0)
        # NumPy 实数按现有契约接受（不报错）
        s2 = PersistentBackendSettings(
            device_info_timeout_s=np.float64(3.0),
            command_timeout_s=np.float32(0.05),
            sweep_timeout_s=np.int64(2),
        )
        self.assertEqual(s2.device_info_timeout_s, 3.0)
        # np.float32(0.05) 为低精度浮点，跨 numpy 版本与 Python float 不恒等，
        # 用容差比较（避免 numpy 1.x / 2.x 的标量比较行为差异）。
        self.assertAlmostEqual(float(s2.command_timeout_s), 0.05, places=6)
        self.assertEqual(s2.sweep_timeout_s, 2)

    def test_none_still_allowed(self) -> None:
        s = PersistentBackendSettings(sweep_timeout_s=None, pending_max=None)
        self.assertIsNone(s.sweep_timeout_s)
        self.assertIsNone(s.pending_max)


class TransportInjectionTests(unittest.TestCase):
    def test_transport_missing_is_open_rejected(self) -> None:
        class _NoIsOpen:
            def open(self) -> None:
                pass

            def read(self, max_length: int, timeout_ms: int) -> bytes:
                return b""

            def write(self, data: bytes) -> None:
                pass

            def close(self) -> None:
                pass

        with self.assertRaises(TypeError):
            PersistentLibreVnaBackend(transport=_NoIsOpen())  # type: ignore[arg-type]

    def test_transport_missing_read_rejected(self) -> None:
        class _NoRead:
            @property
            def is_open(self) -> bool:
                return False

            def open(self) -> None:
                pass

            def write(self, data: bytes) -> None:
                pass

            def close(self) -> None:
                pass

        with self.assertRaises(TypeError):
            PersistentLibreVnaBackend(transport=_NoRead())  # type: ignore[arg-type]


class LegacyAndNoHardwareImportTests(unittest.TestCase):
    def test_legacy_acquire_sparameters_still_available(self) -> None:
        # legacy 入口是 librevna/ 目录下的顶层模块，应用通过
        # ``sys.path.insert(0, <repo>/librevna)`` 导入（见 uav_gpr_qt.py）。
        import sys
        from pathlib import Path

        librevna_dir = str(Path(__file__).resolve().parents[1] / "librevna")
        if librevna_dir not in sys.path:
            sys.path.insert(0, librevna_dir)
        from librevna_usb_s11 import acquire_s11, acquire_sparameters

        self.assertTrue(callable(acquire_sparameters))
        self.assertTrue(callable(acquire_s11))

    def test_continuous_module_imports_no_hardware_or_qt(self) -> None:
        import librevna.librevna_continuous as module

        tree = ast.parse(inspect.getsource(module))
        imported_roots: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_roots.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_roots.add(node.module.split(".")[0])
        self.assertNotIn("usb", imported_roots)
        self.assertNotIn("libusb_package", imported_roots)
        self.assertNotIn("PySide6", imported_roots)


if __name__ == "__main__":
    unittest.main()
