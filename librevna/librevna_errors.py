"""LibreVNA 采集层异常类型（transport / continuous backend 共用）。

Adapted from the read-only reference project
``E:\\钢筋仪软件开发\\src\\rebar_inspector\\acquisition\\errors.py``, trimmed to
the current project: there is no acquisition-framework base class here, so the
state-machine violations of the persistent backend raise
:class:`LibreVnaStateError` (roadmap: FAULTED 只允许 close/reopen) and a full
pending queue raises :class:`LibreVnaPendingOverflowError` (roadmap: pending
溢出 fail-closed).

原始 ``usb.core.USBError`` / ``struct.error`` / 驱动异常一律由公开边界
（``LibreVnaUsbTransport`` / ``PersistentLibreVnaBackend``）包装为本模块的异常
并保留 cause，不泄漏到上层。
"""

from __future__ import annotations


class LibreVnaError(Exception):
    """LibreVNA USB/采集基础异常。"""


class LibreVnaMissingDependencyError(LibreVnaError):
    """缺少 PyUSB / libusb-package 运行依赖（构造或 open 时才惰性加载）。"""


class LibreVnaDeviceNotFoundError(LibreVnaError):
    """未找到 LibreVNA 设备（VID/PID 无匹配）。"""


class LibreVnaBusyError(LibreVnaError):
    """USB 设备被其他程序（如 LibreVNA-GUI）占用，或系统拒绝访问。"""


class LibreVnaTimeoutError(LibreVnaError):
    """等待响应超时（DeviceInfo / ACK / USB 读 / 完整 sweep）。"""


class LibreVnaNackError(LibreVnaError):
    """设备返回 NACK。"""


class LibreVnaProtocolError(LibreVnaError):
    """数据包损坏 / 截断 / 格式错误。"""


class LibreVnaDisconnectedError(LibreVnaError):
    """USB 设备中途断开或读写失败。"""


class LibreVnaUnsupportedConfigError(LibreVnaError):
    """不支持的设备参数 / stages_bitmap / 通道配置。"""


class LibreVnaSweepError(LibreVnaError):
    """连续 sweep 无法在超时内得到完整数据（单次超时：丢 partial、继续重同步）。"""


class LibreVnaReleaseError(LibreVnaError):
    """释放 USB 设备（release_interface / dispose_resources）失败。"""


class LibreVnaStateError(LibreVnaError):
    """后端状态机违规：未 open 就 configure、PAUSED 中 acquire、FAULTED 中
    除 close/reopen 外的任何操作等。"""


class LibreVnaPendingOverflowError(LibreVnaError):
    """pending 队列溢出（fail-closed，进入 FAULTED）。"""


__all__ = [
    "LibreVnaBusyError",
    "LibreVnaDeviceNotFoundError",
    "LibreVnaDisconnectedError",
    "LibreVnaError",
    "LibreVnaMissingDependencyError",
    "LibreVnaNackError",
    "LibreVnaPendingOverflowError",
    "LibreVnaProtocolError",
    "LibreVnaReleaseError",
    "LibreVnaStateError",
    "LibreVnaSweepError",
    "LibreVnaTimeoutError",
    "LibreVnaUnsupportedConfigError",
]
