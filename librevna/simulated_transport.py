"""模拟 LibreVNA 传输层（假设备）：无硬件演示 / 日常回归测试用。

在**既有 USB 协议线**上扮演一台"假 LibreVNA"：实现
:class:`librevna.librevna_transport.LibreVnaTransportProtocol`
（``open/read/write/close`` + ``is_open``），与真实
:class:`librevna.librevna_transport.LibreVnaUsbTransport` 完全同接口，可注入
:class:`librevna.librevna_continuous.PersistentLibreVnaBackend` 的 ``transport``
参数——后端 / worker / 流水线 / 地面站**零适配**。它是 USB 旁路（默认仍走真机），
由空中端"设备来源"开关选择（见 ``uav_gpr_qt.MainWindow``）。

与真实设备一致的协议行为
------------------------
- ``open()`` 只建立会话（不握手）；握手由后端发送 :data:`REQUEST_DEVICE_INFO`
  完成——模拟器回送与真实协议 v14 布局一致的 DeviceInfo（FW 1.6.5、HW "S"）；
- ``SET_IDLE`` 回 :data:`ACK` 并停止扫频数据流（pause/停止语义）；
- ``SWEEP_SETTINGS`` 解码 payload（起始/截止频率、点数、IFBW），回 :data:`ACK`
  并按该配置连续产生 :data:`VNA_DATAPOINT`（点号 0..N-1 循环，频率轴与配置一致，
  逐点严格递增），数据布局与真实硬件相同（S11/S22 双反射六接收机 payload）；
- 每次 ``read`` 返回**一个**完整 packet，单点节奏按 IFBW 模拟真实扫频时长
  （``pace=True``，可关闭加速测试）。

信号物理模型（抄自 rebar-inspector 的反射器模型，只搬模型不搬接口）
--------------------------------------------------------------------
每个反射器在频域产生带相位延迟的反射响应 ``H(f) = A·exp(-j2π·f·τ)``，
多个反射器线性叠加，再加复高斯噪声——对频域数据做 IFFT 会在时延 τ 处出现
峰值（A-scan/B-scan 的基础，B-scan 表现为水平反射带）。默认场景为**冰层
探测场景**的离散化：表面耦合（τ=0，强反射）+ 浅层界面（τ≈1 ns）+ 深层
界面（τ≈2.5 ns），幅度递减；两端口（S11/S22）使用不同幅度的同一反射
场景，体现双通道差异。

确定性：噪声由 ``np.random.default_rng(seed)`` 驱动，``open()`` 重置 RNG 与
点号/道计数——同 seed 同配置的每次 open 会话数据可复现（与参考
SimulatedBackend 的会话级确定性一致）。
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from librevna.librevna_errors import LibreVnaTimeoutError
from librevna.librevna_protocol import (
    ACK,
    DEVICE_INFO,
    REQUEST_DEVICE_INFO,
    SET_IDLE,
    SWEEP_SETTINGS,
    VNA_DATAPOINT,
    encode_packet,
)

#: 模拟器广播的固件 / 硬件版本（HW revision 用 "S" 标识 Simulated）。
SIMULATED_FIRMWARE = (1, 6, 5)
SIMULATED_HW_VERSION = 0x53  # ord("S")
SIMULATED_HW_REVISION = b"S"

#: 默认演示/回归场景的三个反射时延（秒）：表面耦合 0 / 浅层界面 ≈1 ns /
#: 深层界面 ≈2.5 ns（量级与 rebar-inspector 参考默认场景一致；幅度递减）。
SURFACE_DELAY_S = 0.0
SHALLOW_DELAY_S = 1.0e-9
DEEP_DELAY_S = 2.5e-9

#: 每频点额外固件开销（秒），默认 0：扫频时长由 2 阶段/IFBW 主导，默认
#: 201 点 / IFBW 10 kHz ≈ 25 道/秒（对标真机连续扫频节奏，演示友好）。
FIRMWARE_POINT_OVERHEAD_S = 0.0

#: 默认噪声电平（复高斯幅度 rms）。
DEFAULT_NOISE_LEVEL = 0.004


@dataclass(frozen=True)
class Reflector:
    """一个点反射器：双程时延与复幅度（同 rebar-inspector 的 Reflector 语义）。"""

    delay_s: float
    amplitude: complex

    def __post_init__(self) -> None:
        if isinstance(self.delay_s, bool) or not isinstance(self.delay_s, (int, float)):
            raise TypeError("delay_s 必须是实数")
        if not math.isfinite(self.delay_s) or self.delay_s < 0:
            raise ValueError(f"delay_s 必须为非负有限值，实际 {self.delay_s}")
        if not isinstance(self.amplitude, complex) and not isinstance(
            self.amplitude, (int, float)
        ):
            raise TypeError("amplitude 必须是 complex/实数")
        if not math.isfinite(self.amplitude.real) or not math.isfinite(self.amplitude.imag):
            raise ValueError(f"amplitude 必须有限，实际 {self.amplitude}")
        object.__setattr__(self, "amplitude", complex(self.amplitude))


#: 默认 S11 场景：表面耦合 + 浅层界面 + 深层界面（0 / 1 ns / 2.5 ns，幅度递减）。
DEFAULT_ICE_SCENE_S11: tuple[Reflector, ...] = (
    Reflector(delay_s=SURFACE_DELAY_S, amplitude=-0.26 + 0.0j),
    Reflector(delay_s=SHALLOW_DELAY_S, amplitude=-0.09 + 0.0j),
    Reflector(delay_s=DEEP_DELAY_S, amplitude=-0.04 + 0.0j),
)

#: 默认 S22 场景：与 S11 同几何、耦合较弱（体现双通道差异）。
DEFAULT_ICE_SCENE_S22: tuple[Reflector, ...] = (
    Reflector(delay_s=SURFACE_DELAY_S, amplitude=-0.16 + 0.0j),
    Reflector(delay_s=SHALLOW_DELAY_S, amplitude=-0.055 + 0.0j),
    Reflector(delay_s=DEEP_DELAY_S, amplitude=-0.024 + 0.0j),
)


def _device_info_payload() -> bytes:
    """与协议 v14 布局一致的 DeviceInfo payload（能力覆盖 UI 全范围）。"""
    protocol, fw, hw_rev = 14, SIMULATED_FIRMWARE, SIMULATED_HW_REVISION
    return struct.pack(
        "<HBBBBcQQIIHhhIIBQBH",
        protocol,
        fw[0],
        fw[1],
        fw[2],
        SIMULATED_HW_VERSION,
        hw_rev,
        100_000,  # min_freq_hz：0.1 MHz（对齐真机下限）
        6_000_000_000,  # max_freq_hz：6 GHz
        1,  # min_ifbw_hz
        1_000_000,  # max_ifbw_hz
        10_001,  # max_points
        -5000,  # min_dbm → -50 dBm
        1000,  # max_dbm → +10 dBm
        1,  # min_rbw_hz
        1_000_000,  # max_rbw_hz
        200,  # max_amplitude_points
        12_000_000_000,  # max_harmonic_freq_hz
        2,  # num_ports（双反射 S11/S22）
        65_535,  # max_dwell_time_us
    )


def reflector_response(
    freq_hz: float, reflectors: Sequence[Reflector]
) -> complex:
    """Σ A_k·exp(-j·2π·f·τ_k)（rebar-inspector 反射器物理模型，标量求值）。"""
    total = 0.0 + 0.0j
    for reflector in reflectors:
        theta = 2.0 * math.pi * freq_hz * reflector.delay_s
        total += reflector.amplitude * complex(math.cos(theta), -math.sin(theta))
    return total


class SimulatedLibreVnaTransport:
    """“假 LibreVNA”：协议线旁的模拟采集通道（无硬件演示 / 回归）。

    与 :class:`librevna.librevna_transport.LibreVnaUsbTransport` 同接口；
    后端只依赖该接口，因此把本 transport 注入 ``PersistentLibreVnaBackend``
    即可让整条 worker/服务/流水线/地面站链路以完全相同的代码路径消费模拟
    数据（上层零适配）。
    """

    def __init__(
        self,
        *,
        seed: int = 0,
        noise_level: float = DEFAULT_NOISE_LEVEL,
        reflectors_s11: Sequence[Reflector] | None = None,
        reflectors_s22: Sequence[Reflector] | None = None,
        pace: bool = True,
    ) -> None:
        if not math.isfinite(noise_level) or noise_level < 0:
            raise ValueError(f"noise_level 必须为非负有限值，实际 {noise_level}")
        s11 = tuple(reflectors_s11) if reflectors_s11 is not None else DEFAULT_ICE_SCENE_S11
        s22 = tuple(reflectors_s22) if reflectors_s22 is not None else DEFAULT_ICE_SCENE_S22
        if not s11 or not s22:
            raise ValueError("reflectors_s11/reflectors_s22 不能为空")
        for group in (s11, s22):
            for reflector in group:
                if not isinstance(reflector, Reflector):
                    raise TypeError("reflectors 必须全部是 Reflector 实例")
        self._seed = int(seed)
        self._noise_level = float(noise_level)
        self._reflectors_s11: tuple[Reflector, ...] = s11
        self._reflectors_s22: tuple[Reflector, ...] = s22
        self._pace = bool(pace)

        # ---- 会话内状态（open() 重置）----
        self._opened = False
        self._rng: np.random.Generator = np.random.default_rng(self._seed)
        self._point = 0
        self._sweep_count = 0
        self._streaming = False
        self._reply: str | None = None
        self._freq_axis: list[int] = []
        self._pace_point_s = 0.0

        # ---- 生命周期统计（跨会话累计）----
        self.open_count = 0
        self.close_count = 0
        self.read_count = 0

    # ---- 只读观测 ----

    @property
    def seed(self) -> int:
        return self._seed

    @property
    def noise_level(self) -> float:
        return self._noise_level

    @property
    def reflectors_s11(self) -> tuple[Reflector, ...]:
        return self._reflectors_s11

    @property
    def reflectors_s22(self) -> tuple[Reflector, ...]:
        return self._reflectors_s22

    @property
    def pace(self) -> bool:
        return self._pace

    @property
    def is_open(self) -> bool:
        return self._opened

    @property
    def streaming(self) -> bool:
        """是否处于按 SWEEP_SETTINGS 配置连续出点的状态（SET_IDLE 后为 False）。"""
        return self._streaming

    @property
    def sweeps_generated(self) -> int:
        """本 open 会话已完成（回绕）的整道数（open 重置）。"""
        return self._sweep_count

    @property
    def point(self) -> int:
        return self._point

    # ---- 生命周期 ----

    def open(self) -> None:
        """建立“连接”并重置会话（RNG/点号/道数/扫频配置）。"""
        self.open_count += 1
        self._opened = True
        self._rng = np.random.default_rng(self._seed)
        self._point = 0
        self._sweep_count = 0
        self._streaming = False
        self._reply = None
        self._freq_axis = []
        self._pace_point_s = 0.0

    def close(self) -> None:
        """断开“连接”（幂等）。"""
        self.close_count += 1
        self._opened = False

    # ---- 命令 / 数据 ----

    def write(self, data: bytes) -> None:
        """解析主机下发的协议命令并准备响应（不阻塞）。"""
        if not self._opened:
            raise LibreVnaTimeoutError("simulated transport not open")
        if len(data) < 4 or data[0] != 0x5A:
            return  # 噪声字节：真实 USB 层也静默丢弃
        packet_type = data[3]
        if packet_type == REQUEST_DEVICE_INFO:
            self._reply = "device_info"
        elif packet_type == SET_IDLE:
            self._reply = "ack"
            self._streaming = False
        elif packet_type == SWEEP_SETTINGS:
            self._apply_sweep_settings(data[4 : 4 + 31] if len(data) >= 35 else b"")
            self._reply = "ack"
        # 其他类型（DEVICE_STATUS 等）：无响应，不影响数据流。

    def read(self, max_length: int, timeout_ms: int) -> bytes:
        """返回一个完整协议 packet（DeviceInfo / ACK / VNADatapoint）。

        - DeviceInfo / ACK 立即返回（握手不模拟扫频时长）；
        - 数据流按 IFBW 模拟逐点扫频节奏（``pace=True``）后返回下一个
          VNADatapoint；未配置/已停止时按真实设备抛超时。
        """
        if not self._opened:
            raise LibreVnaTimeoutError("simulated transport not open")
        self.read_count += 1
        if self._reply == "device_info":
            self._reply = None
            return encode_packet(DEVICE_INFO, _device_info_payload())
        if self._reply == "ack":
            self._reply = None
            return encode_packet(ACK)
        if not self._streaming or not self._freq_axis:
            raise LibreVnaTimeoutError("simulated device idle (not streaming)")
        if self._pace and self._pace_point_s > 0:
            time.sleep(self._pace_point_s)
        point = self._point
        freq_hz = self._freq_axis[point]
        s11 = self._next_point_value(self._reflectors_s11, freq_hz)
        s22 = self._next_point_value(self._reflectors_s22, freq_hz)
        self._point += 1
        if self._point >= len(self._freq_axis):
            self._point = 0
            self._sweep_count += 1
        return _vna_point_packet(freq_hz, point, s11, s22)

    # ---- 内部 ----

    def _apply_sweep_settings(self, payload: bytes) -> None:
        """解码 SWEEP_SETTINGS payload 并准备对应频率轴 / 节奏。"""
        if len(payload) != 31:
            raise ValueError(
                f"SWEEP_SETTINGS payload 长度必须为 31，实际 {len(payload)}"
            )
        start_hz, stop_hz, points, ifbw_hz = struct.unpack_from("<QQHI", payload)
        if stop_hz <= start_hz or points < 2 or ifbw_hz <= 0:
            raise ValueError(
                f"非法扫频配置: start={start_hz} stop={stop_hz} "
                f"points={points} ifbw={ifbw_hz}"
            )
        # 设备侧频率轴必须是**均匀**整数步进（真实 LibreVNA 返回的逐点频率；
        # 下游 IFFT 流水线要求 uniformly spaced）。step 取整后末点与 stop 的
        # 偏差 < 1 个步长，属设备量化行为，后续处理只依赖该返回轴本身。
        step_hz = max(1, int(round((stop_hz - start_hz) / (points - 1))))
        axis = [start_hz + step_hz * index for index in range(points)]
        self._freq_axis = axis
        # 逐点节奏 ≈ 双反射 2 阶段 / IFBW + 固件开销。
        self._pace_point_s = (2.0 / max(int(ifbw_hz), 1)) + (
            FIRMWARE_POINT_OVERHEAD_S if self._pace else 0.0
        )
        self._point = 0
        self._sweep_count = 0
        self._streaming = True

    def _next_point_value(
        self, reflectors: Sequence[Reflector], freq_hz: int
    ) -> complex:
        """物理模型值 + 复高斯噪声（每通道独立 RNG 流）。"""
        value = reflector_response(float(freq_hz), reflectors)
        if self._noise_level > 0:
            scale = self._noise_level / math.sqrt(2.0)
            value += scale * (
                complex(self._rng.standard_normal(), self._rng.standard_normal())
            )
        return value


def _vna_point_packet(freq_hz: int, point: int, s11: complex, s22: complex) -> bytes:
    """构造 protocol v14 双反射 VNADatapoint（六接收机，与真实硬件布局一致）。"""
    receivers = [
        (0x01, s11),  # stage0 Port-1 接收机 → S11 分子
        (0x02, complex(9.0, 9.0)),  # stage0 Port-2（无关接收机，填充恒定值）
        (0x13, complex(1.0, 0.0)),  # stage0 参考 → S11 分母
        (0x21, complex(7.0, 7.0)),  # stage1 Port-1（无关接收机，填充恒定值）
        (0x22, s22),  # stage1 Port-2 接收机 → S22 分子
        (0x33, complex(1.0, 0.0)),  # stage1 参考 → S22 分母
    ]
    count = len(receivers)
    payload = struct.pack("<QhH", int(freq_hz), -1000, int(point))
    payload += struct.pack("<" + "f" * count, *[v.real for _, v in receivers])
    payload += struct.pack("<" + "f" * count, *[v.imag for _, v in receivers])
    payload += bytes(d for d, _ in receivers)
    return encode_packet(VNA_DATAPOINT, payload)


__all__ = [
    "DEFAULT_ICE_SCENE_S11",
    "DEFAULT_ICE_SCENE_S22",
    "DEFAULT_NOISE_LEVEL",
    "DEEP_DELAY_S",
    "FIRMWARE_POINT_OVERHEAD_S",
    "Reflector",
    "SHALLOW_DELAY_S",
    "SIMULATED_FIRMWARE",
    "SIMULATED_HW_REVISION",
    "SIMULATED_HW_VERSION",
    "SimulatedLibreVnaTransport",
    "SURFACE_DELAY_S",
    "reflector_response",
]
