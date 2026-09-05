"""LibreVNA USB 传输层：PyUSB/libusb 设备查找、打开、读写、claim/release。

Adapted from the field-validated transport of the read-only reference project
(``E:\\钢筋仪软件开发\\src\\rebar_inspector\\acquisition\\librevna_usb_transport.py``)
and the current project's ``librevna_usb_probe.open_device``:

- VID_1209 / PID_4121，bulk EP_OUT=0x01 / EP_IN=0x81；
- 打开后 ``set_configuration``（Windows 上失败只警告，不中断）；
- claim 失败视为设备被其他程序占用；
- Windows 上通过 ``libusb_package`` 附带 DLL 定位 libusb-1.0；
- ``open``/``close`` 幂等，释放接口与资源。

本模块**惰性加载** PyUSB/libusb：构造与导入模块本身都不加载 ``usb`` /
``libusb_package``，只有真正 ``open()`` 时才加载；缺依赖抛带安装提示的
``LibreVnaMissingDependencyError``。原始 ``usb.core.USBError`` 一律包装为
LibreVNA 异常（保留 cause），不泄漏到上层。

Fake transport 契约：任何实现 :class:`LibreVnaTransportProtocol`
（``open/read/write/close`` + ``is_open``）的对象都可替换本类——后端只依赖该
接口（见 ``librevna_continuous.PersistentLibreVnaBackend`` 的 ``transport`` 注入）。
"""

from __future__ import annotations

import ctypes
import importlib
import sys
import warnings
from pathlib import Path
from typing import Protocol

from librevna.librevna_errors import (
    LibreVnaBusyError,
    LibreVnaDeviceNotFoundError,
    LibreVnaDisconnectedError,
    LibreVnaError,
    LibreVnaMissingDependencyError,
    LibreVnaReleaseError,
    LibreVnaTimeoutError,
)
from librevna.librevna_protocol import EP_IN, EP_OUT, PID, VID

#: 缺依赖时的安装提示。
_HARDWARE_INSTALL_HINT = "pip install pyusb libusb-package"


class LibreVnaTransportProtocol(Protocol):
    """后端依赖的最小传输接口（fake transport 也要实现它）。"""

    @property
    def is_open(self) -> bool:
        """设备是否已打开（后端 close 时据此决定是否发 SET_IDLE/释放）。"""

    def open(self) -> None:
        """打开 USB 设备并 claim 接口。幂等：已 open 时为空操作。"""

    def read(self, max_length: int, timeout_ms: int) -> bytes:
        """从 IN 端点读取原始字节。超时抛 :class:`LibreVnaTimeoutError`。"""

    def write(self, data: bytes) -> None:
        """把原始字节写到 OUT 端点。"""

    def close(self) -> None:
        """释放接口与资源。幂等。"""


class LibreVnaUsbTransport:
    """PyUSB/libusb 实现的 LibreVNA 传输层。"""

    def __init__(
        self,
        *,
        vid: int = VID,
        pid: int = PID,
        ep_out: int = EP_OUT,
        ep_in: int = EP_IN,
    ) -> None:
        if not isinstance(vid, int) or isinstance(vid, bool):
            raise TypeError("vid/pid 必须是 int")
        if not isinstance(pid, int) or isinstance(pid, bool):
            raise TypeError("vid/pid 必须是 int")
        if not isinstance(ep_out, int) or isinstance(ep_out, bool):
            raise TypeError("ep_out/ep_in 必须是 int")
        if not isinstance(ep_in, int) or isinstance(ep_in, bool):
            raise TypeError("ep_out/ep_in 必须是 int")
        self._vid = vid
        self._pid = pid
        self._ep_out = ep_out
        self._ep_in = ep_in
        self._dev = None
        self._interface_number: int | None = None
        self._usb_core = None  # 惰性加载后的模块引用
        self._usb_util = None

    # ---- 查询 ----

    @property
    def is_open(self) -> bool:
        return self._dev is not None

    # ---- 生命周期 ----

    def open(self) -> None:
        """打开设备、set_configuration、claim 接口（幂等）。

        find() 成功后的任意步骤失败：不泄漏原始 USBError，尽力 release_interface
        与 dispose_resources，清空内部 device/endpoint/open 状态，再抛对应的
        LibreVNA 异常并保留原始异常为 cause。
        """
        if self._dev is not None:
            return
        usb_core, usb_util = self._load_usb_modules()
        # 可能抛 LibreVnaMissingDependencyError（native libusb backend 缺失）。
        backend = self._build_libusb1_backend(usb_core)
        try:
            dev = usb_core.find(
                idVendor=self._vid, idProduct=self._pid, backend=backend
            )
        except usb_core.USBError as exc:
            raise LibreVnaBusyError(
                f"无法访问 LibreVNA（VID={self._vid:#06x} PID={self._pid:#06x}），"
                f"可能被其他程序占用: {exc}"
            ) from exc
        if dev is None:
            raise LibreVnaDeviceNotFoundError(
                f"未找到 LibreVNA 设备（VID={self._vid:#06x} PID={self._pid:#06x}），"
                "请确认设备已通过 USB 连接"
            )

        interface_number: int | None = None
        try:
            try:
                dev.set_configuration()
            except usb_core.USBError as exc:
                # Windows 上内核/WinUSB 配置已激活时可能失败；参考实现警告后继续。
                warnings.warn(f"LibreVNA set_configuration 警告：{exc}")
            cfg = dev.get_active_configuration()
            interface = cfg[(0, 0)]
            interface_number = interface.bInterfaceNumber
            try:
                if dev.is_kernel_driver_active(interface_number):
                    dev.detach_kernel_driver(interface_number)
            except (NotImplementedError, usb_core.USBError):
                pass
            usb_util.claim_interface(dev, interface_number)
        except usb_core.USBError as exc:
            self._cleanup_failed_open(dev, interface_number, usb_util)
            raise LibreVnaBusyError(
                f"打开 LibreVNA（VID={self._vid:#06x} PID={self._pid:#06x}）失败，"
                "设备可能被其他程序占用或配置不可用: "
                f"{exc}"
            ) from exc
        except Exception as exc:  # noqa: BLE001 - 非 USBError 一律包装
            self._cleanup_failed_open(dev, interface_number, usb_util)
            raise LibreVnaError(
                f"打开 LibreVNA（VID={self._vid:#06x} PID={self._pid:#06x}）失败: "
                f"{exc}"
            ) from exc

        assert interface_number is not None
        self._dev = dev
        self._interface_number = interface_number
        self._usb_core = usb_core
        self._usb_util = usb_util

    def close(self) -> None:
        """释放接口与资源（幂等）。

        无论 release_interface 是否成功都尝试 dispose_resources；任一失败都抛
        :class:`LibreVnaReleaseError`（两者都失败时保留完整诊断信息）。失败后
        本地 device/endpoint/open 状态已清空，允许后续重新 open。
        """
        if self._dev is None:
            return
        dev = self._dev
        interface_number = self._interface_number
        usb_util = self._usb_util
        self._dev = None
        self._interface_number = None

        details: list[str] = []
        cause: Exception | None = None
        if interface_number is not None and usb_util is not None:
            try:
                usb_util.release_interface(dev, interface_number)
            except Exception as exc:  # noqa: BLE001 - 包装为 LibreVnaReleaseError
                details.append(f"release_interface: {exc}")
                cause = exc
        if usb_util is not None:
            try:
                usb_util.dispose_resources(dev)
            except Exception as exc:  # noqa: BLE001 - 不再静默成功
                details.append(f"dispose_resources: {exc}")
                if cause is None:
                    cause = exc
        if details:
            raise LibreVnaReleaseError(
                f"释放 LibreVNA 设备失败: {'；'.join(details)}"
            ) from cause

    # ---- 读写 ----

    def read(self, max_length: int, timeout_ms: int) -> bytes:
        if self._dev is None or self._usb_core is None:
            raise LibreVnaError("USB 设备未打开，无法读取")
        try:
            data = self._dev.read(self._ep_in, max_length, timeout=timeout_ms)
        except self._usb_core.USBTimeoutError:
            raise LibreVnaTimeoutError("USB 读取超时") from None
        except self._usb_core.USBError as exc:
            raise LibreVnaDisconnectedError(
                f"USB 读取失败（设备可能已断开）: {exc}"
            ) from exc
        return bytes(data)

    def write(self, data: bytes) -> None:
        if self._dev is None or self._usb_core is None:
            raise LibreVnaError("USB 设备未打开，无法写入")
        try:
            written = self._dev.write(self._ep_out, data, timeout=1000)
        except self._usb_core.USBTimeoutError:
            raise LibreVnaTimeoutError("USB 写入超时") from None
        except Exception as exc:  # noqa: BLE001 - 其他 I/O/断开统一包装
            raise LibreVnaDisconnectedError(
                f"USB 写入失败（设备可能已断开）: {exc}"
            ) from exc
        if written != len(data):
            raise LibreVnaError(
                f"USB 写入长度不足：期望 {len(data)} 字节，实际 {written} 字节"
            )

    # ---- 内部 ----

    def _load_usb_modules(self):
        """惰性加载 usb 模块；缺依赖抛带安装提示的 LibreVnaMissingDependencyError。"""
        try:
            usb_core = importlib.import_module("usb.core")
            usb_util = importlib.import_module("usb.util")
            return usb_core, usb_util
        except ModuleNotFoundError as exc:
            name = (exc.name or "").split(".")[0]
            raise LibreVnaMissingDependencyError(
                f"缺少 LibreVNA USB 运行依赖（{name or 'pyusb'}）。请安装："
                f"{_HARDWARE_INSTALL_HINT}"
            ) from exc

    def _build_libusb1_backend(self, usb_core):
        """在 Windows 上用 libusb_package 附带 DLL 构造 libusb-1.0 backend。

        PyUSB 已安装但 native libusb backend 无法获得（Windows 上 libusb_package
        未提供 libusb-1.0.dll 或 backend 构造失败）时，抛
        :class:`LibreVnaMissingDependencyError` 并给安装提示，不能误报设备忙。
        非 Windows 或无 libusb_package 时返回 None，让 pyusb 走默认 backend。
        """
        backend = None
        dll = None
        try:
            libusb_package = importlib.import_module("libusb_package")
            dll_dir = Path(libusb_package.__file__).resolve().parent
            candidate = dll_dir / "libusb-1.0.dll"
            if candidate.is_file():
                dll = candidate
        except (ModuleNotFoundError, OSError):
            dll = None
        if dll is not None and sys.platform.startswith("win"):
            try:
                ctypes.WinDLL(str(dll))
            except OSError:
                dll = None
        if dll is not None:
            try:
                usb_backend = importlib.import_module("usb.backend.libusb1")
                backend = usb_backend.get_backend(
                    find_library=lambda _: str(dll)
                )
            except Exception:  # noqa: BLE001 - 回退默认 backend
                backend = None
        if backend is None and sys.platform.startswith("win"):
            # PyUSB 已装但 native libusb backend 缺失 -> MissingDependency，非忙。
            raise LibreVnaMissingDependencyError(
                "PyUSB 已安装但缺少 native libusb backend（libusb-1.0 未找到）。"
                f"请安装：{_HARDWARE_INSTALL_HINT}"
            )
        return backend

    def _cleanup_failed_open(
        self,
        dev,
        interface_number: int | None,
        usb_util,
    ) -> None:
        """open() 在 find() 成功后失败：尽力释放接口与资源，清空内部状态。"""
        if interface_number is not None:
            try:
                usb_util.release_interface(dev, interface_number)
            except Exception:  # noqa: BLE001 - 尽力而为
                pass
        try:
            usb_util.dispose_resources(dev)
        except Exception:  # noqa: BLE001 - 尽力而为
            pass
        self._dev = None
        self._interface_number = None


__all__ = [
    "LibreVnaTransportProtocol",
    "LibreVnaUsbTransport",
]
