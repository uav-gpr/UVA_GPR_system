"""LibreVNA USB transport-layer unit tests (no real hardware).

Covers the roadmap phase-2 transport requirements:

- ``open/read/write/close/is_open`` contract;
- PyUSB / libusb_package **lazy** loading (importing/constructing must not
  load them; only ``open()`` does);
- ``open``/``close`` idempotent;
- mapping of device-not-found / busy / timeout / disconnect / release /
  missing-dependency errors to the ``LibreVnaError`` family (cause preserved);
- Fake transport protocol contract (any ``open/read/write/close/is_open``
  object is accepted by the backend -- verified in test_librevna_backend.py).

No real hardware is required: the PyUSB interaction is exercised through
injected fake ``usb.core`` / ``usb.util`` modules (deterministic), and the
"no hardware attached" path is covered via a hardware-presence guard.
"""

from __future__ import annotations

import ast
import importlib
import inspect
import sys
import unittest
from unittest import mock

from librevna.librevna_errors import (
    LibreVnaBusyError,
    LibreVnaDeviceNotFoundError,
    LibreVnaDisconnectedError,
    LibreVnaError,
    LibreVnaMissingDependencyError,
    LibreVnaReleaseError,
    LibreVnaTimeoutError,
)
from librevna.librevna_transport import (
    LibreVnaTransportProtocol,
    LibreVnaUsbTransport,
)


# ---- fake PyUSB modules (injected via _load_usb_modules) ----

class _FakeUSBError(Exception):
    pass


class _FakeUSBTimeoutError(_FakeUSBError):
    pass


class _FakeInterface:
    def __init__(self, number: int = 0) -> None:
        self.bInterfaceNumber = number


class _FakeConfig:
    def __init__(self) -> None:
        self._interface = _FakeInterface()

    def __getitem__(self, key) -> _FakeInterface:
        return self._interface


class _FakeDevice:
    """Minimal fake PyUSB device (only the calls the transport makes)."""

    def __init__(self) -> None:
        self.set_configuration_calls = 0
        self.config = _FakeConfig()
        self.read_payload = b""
        self.read_raises: Exception | None = None
        self.write_raises: Exception | None = None
        self.write_result: int | None = None  # 若设置则返回该值（短写测试）
        self.active_config_raises: Exception | None = None
        self.kernel_driver_active = False

    def set_configuration(self) -> None:
        self.set_configuration_calls += 1

    def get_active_configuration(self) -> _FakeConfig:
        if self.active_config_raises is not None:
            raise self.active_config_raises
        return self.config

    def is_kernel_driver_active(self, number: int) -> bool:
        return self.kernel_driver_active

    def detach_kernel_driver(self, number: int) -> None:
        self.kernel_driver_active = False

    def read(self, ep: int, length: int, timeout: int) -> bytes:
        if self.read_raises is not None:
            raise self.read_raises
        return self.read_payload

    def write(self, ep: int, data: bytes, timeout: int) -> int:
        if self.write_raises is not None:
            raise self.write_raises
        return self.write_result if self.write_result is not None else len(data)


class _FakeUsbCore:
    USBError = _FakeUSBError
    USBTimeoutError = _FakeUSBTimeoutError

    def __init__(
        self, dev: _FakeDevice | None = None, find_raises: Exception | None = None
    ) -> None:
        self._dev = dev
        self._find_raises = find_raises

    def find(self, **kwargs) -> _FakeDevice | None:
        if self._find_raises is not None:
            raise self._find_raises
        return self._dev


class _FakeUsbUtil:
    def __init__(self) -> None:
        self.claimed: list = []
        self.released: list = []
        self.disposed: list = []
        self.claim_raises: Exception | None = None
        self.release_raises: Exception | None = None
        self.dispose_raises: Exception | None = None

    def claim_interface(self, dev, number: int) -> None:
        if self.claim_raises is not None:
            raise self.claim_raises
        self.claimed.append((dev, number))

    def release_interface(self, dev, number: int) -> None:
        if self.release_raises is not None:
            raise self.release_raises
        self.released.append((dev, number))

    def dispose_resources(self, dev) -> None:
        if self.dispose_raises is not None:
            raise self.dispose_raises
        self.disposed.append(dev)


def _patch_usb(dev: _FakeDevice | None = None, find_raises: Exception | None = None):
    """Return context managers patching transport._load_usb_modules /
    _build_libusb1_backend with deterministic fakes."""
    core = _FakeUsbCore(dev=dev, find_raises=find_raises)
    util = _FakeUsbUtil()
    return core, util, (
        mock.patch.object(
            LibreVnaUsbTransport,
            "_load_usb_modules",
            return_value=(core, util),
        ),
        mock.patch.object(
            LibreVnaUsbTransport,
            "_build_libusb1_backend",
            return_value=None,
        ),
    )


def _transport_with_dev(
    *,
    read_raises: Exception | None = None,
    write_raises: Exception | None = None,
    read_payload: bytes = b"",
) -> tuple[LibreVnaUsbTransport, _FakeDevice, _FakeUsbUtil]:
    dev = _FakeDevice()
    dev.read_raises = read_raises
    dev.write_raises = write_raises
    dev.read_payload = read_payload
    core = _FakeUsbCore(dev)
    util = _FakeUsbUtil()
    transport = LibreVnaUsbTransport()
    transport._dev = dev  # noqa: SLF001 - test seam
    transport._interface_number = 0  # noqa: SLF001
    transport._usb_core = core  # noqa: SLF001
    transport._usb_util = util  # noqa: SLF001
    return transport, dev, util


def _librevna_device_present() -> bool:
    """探测本机是否已连接 LibreVNA（自动测试不得依赖/假设硬件在场或缺场）。"""
    try:
        from pathlib import Path

        import libusb_package  # noqa: PLC0415
        import usb.backend.libusb1  # noqa: PLC0415
        import usb.core  # noqa: PLC0415

        dll = Path(libusb_package.__file__).resolve().parent / "libusb-1.0.dll"
        if dll.exists() and sys.platform.startswith("win"):
            import ctypes  # noqa: PLC0415

            try:
                ctypes.WinDLL(str(dll))
            except OSError:
                pass
            backend = usb.backend.libusb1.get_backend(
                find_library=lambda _: str(dll)
            )
            return (
                usb.core.find(idVendor=0x1209, idProduct=0x4121, backend=backend)
                is not None
            )
        return usb.core.find(idVendor=0x1209, idProduct=0x4121) is not None
    except Exception:  # noqa: BLE001 - 缺依赖/枚举失败按"设备不在场"处理
        return False


class TransportConstructionTests(unittest.TestCase):
    def test_rejects_non_int_vid_pid(self) -> None:
        for bad in (True, "0x1209", 3.5, None):
            with self.assertRaises(TypeError):
                LibreVnaUsbTransport(vid=bad)  # type: ignore[arg-type]
            with self.assertRaises(TypeError):
                LibreVnaUsbTransport(pid=bad)  # type: ignore[arg-type]

    def test_rejects_non_int_ep(self) -> None:
        for bad in (True, "0x81", 3.5, None):
            with self.assertRaises(TypeError):
                LibreVnaUsbTransport(ep_in=bad)  # type: ignore[arg-type]
            with self.assertRaises(TypeError):
                LibreVnaUsbTransport(ep_out=bad)  # type: ignore[arg-type]

    def test_construction_does_not_load_usb(self) -> None:
        # open() is the only place PyUSB/libusb get loaded; constructing a
        # transport must never import them (roadmap: 惰性加载).
        with mock.patch.object(
            LibreVnaUsbTransport, "_load_usb_modules"
        ) as loader:
            transport = LibreVnaUsbTransport()
            loader.assert_not_called()
        self.assertFalse(transport.is_open)
        self.assertIsNone(transport._dev)  # noqa: SLF001

    def test_transport_ast_has_no_hardware_imports(self) -> None:
        tree = ast.parse(inspect.getsource(importlib.import_module(
            "librevna.librevna_transport"
        )))
        imported_roots: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_roots.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_roots.add(node.module.split(".")[0])
        self.assertNotIn("usb", imported_roots)
        self.assertNotIn("libusb_package", imported_roots)
        self.assertNotIn("numpy", imported_roots)
        self.assertNotIn("PySide6", imported_roots)

    def test_transport_protocol_contract(self) -> None:
        # LibreVnaTransportProtocol names the exact methods the backend needs.
        names = {"open", "read", "write", "close", "is_open"}
        self.assertTrue(names.issubset(dir(LibreVnaTransportProtocol)))


class TransportOpenCloseTests(unittest.TestCase):
    def test_open_idempotent(self) -> None:
        dev = _FakeDevice()
        core, util, patches = _patch_usb(dev=dev)
        with patches[0], patches[1]:
            transport = LibreVnaUsbTransport()
            transport.open()
            transport.open()  # 幂等：第二次 no-op
            self.assertTrue(transport.is_open)
            self.assertEqual(core._dev, dev)
            self.assertEqual(len(util.claimed), 1)
            self.assertEqual(dev.set_configuration_calls, 1)
            transport.close()

    def test_close_idempotent(self) -> None:
        dev = _FakeDevice()
        core, util, patches = _patch_usb(dev=dev)
        with patches[0], patches[1]:
            transport = LibreVnaUsbTransport()
            transport.open()
            transport.close()
            transport.close()  # 幂等：第二次 no-op
            self.assertFalse(transport.is_open)
            self.assertEqual(len(util.released), 1)
            self.assertEqual(len(util.disposed), 1)
            transport.close()  # 已关闭，仍 no-op

    def test_close_without_open_noop(self) -> None:
        transport = LibreVnaUsbTransport()
        transport.close()  # 不抛错
        self.assertFalse(transport.is_open)


class TransportErrorMappingTests(unittest.TestCase):
    def test_open_not_found(self) -> None:
        core, util, patches = _patch_usb(dev=None)
        with patches[0], patches[1]:
            transport = LibreVnaUsbTransport()
            with self.assertRaises(LibreVnaDeviceNotFoundError):
                transport.open()
            self.assertFalse(transport.is_open)

    def test_open_busy_on_find(self) -> None:
        core, util, patches = _patch_usb(find_raises=_FakeUSBError("denied"))
        with patches[0], patches[1]:
            transport = LibreVnaUsbTransport()
            with self.assertRaises(LibreVnaBusyError):
                transport.open()
            self.assertFalse(transport.is_open)

    def test_open_busy_on_claim_releases_resources(self) -> None:
        dev = _FakeDevice()
        core, util, patches = _patch_usb(dev=dev)
        with patches[0], patches[1]:
            util.claim_raises = _FakeUSBError("claimed by LibreVNA-GUI")
            transport = LibreVnaUsbTransport()
            with self.assertRaises(LibreVnaBusyError) as ctx:
                transport.open()
            self.assertIn("占用", str(ctx.exception))
            # claim 失败时尽力 dispose_resources
            self.assertEqual(len(util.disposed), 1)
            self.assertFalse(transport.is_open)

    def test_open_missing_dependency(self) -> None:
        transport = LibreVnaUsbTransport()
        with mock.patch(
            "librevna.librevna_transport.importlib.import_module",
            side_effect=ModuleNotFoundError("No module named 'usb.core'"),
        ):
            with self.assertRaises(LibreVnaMissingDependencyError) as ctx:
                transport.open()
        self.assertIn("pip install", str(ctx.exception))
        self.assertFalse(transport.is_open)

    def test_read_not_open(self) -> None:
        transport = LibreVnaUsbTransport()
        with self.assertRaises(LibreVnaError):
            transport.read(512, 10)

    def test_read_timeout_mapped(self) -> None:
        transport, _dev, _util = _transport_with_dev(
            read_raises=_FakeUSBTimeoutError()
        )
        with self.assertRaises(LibreVnaTimeoutError):
            transport.read(512, 10)

    def test_read_disconnect_mapped(self) -> None:
        transport, _dev, _util = _transport_with_dev(
            read_raises=_FakeUSBError("no device")
        )
        with self.assertRaises(LibreVnaDisconnectedError) as ctx:
            transport.read(512, 10)
        self.assertIsNotNone(ctx.exception.__cause__)

    def test_read_returns_bytes(self) -> None:
        transport, _dev, _util = _transport_with_dev(read_payload=b"\x5a\x08")
        self.assertEqual(transport.read(512, 10), b"\x5a\x08")

    def test_write_not_open(self) -> None:
        transport = LibreVnaUsbTransport()
        with self.assertRaises(LibreVnaError):
            transport.write(b"\x5a")

    def test_write_disconnect_mapped(self) -> None:
        transport, _dev, _util = _transport_with_dev(
            write_raises=_FakeUSBError("no device")
        )
        with self.assertRaises(LibreVnaDisconnectedError):
            transport.write(b"\x5a")

    def test_close_release_error_still_disposes(self) -> None:
        dev = _FakeDevice()
        core, util, patches = _patch_usb(dev=dev)
        with patches[0], patches[1]:
            transport = LibreVnaUsbTransport()
            transport.open()
            util.release_raises = _FakeUSBError("release failed")
            with self.assertRaises(LibreVnaReleaseError):
                transport.close()
            # 即使 release_interface 失败，dispose_resources 仍尽力执行且状态已清空
            self.assertEqual(len(util.disposed), 1)
            self.assertFalse(transport.is_open)
            transport.close()  # 幂等，可再次 close

    def test_write_timeout_mapped(self) -> None:
        transport, _dev, _util = _transport_with_dev(
            write_raises=_FakeUSBTimeoutError()
        )
        with self.assertRaises(LibreVnaTimeoutError):
            transport.write(b"\x5a" + b"\x00" * 8)

    def test_write_short_write_rejected(self) -> None:
        transport, dev, _util = _transport_with_dev()
        data = b"\x5a" + b"\x00" * 8
        dev.write_result = len(data) - 1  # 短写
        with self.assertRaises(LibreVnaError) as ctx:
            transport.write(data)
        self.assertIn("写入长度不足", str(ctx.exception))

    def test_close_dispose_failure_raises_release_error(self) -> None:
        dev = _FakeDevice()
        core, util, patches = _patch_usb(dev=dev)
        with patches[0], patches[1]:
            transport = LibreVnaUsbTransport()
            transport.open()
            util.dispose_raises = _FakeUSBError("dispose failed")
            with self.assertRaises(LibreVnaReleaseError):
                transport.close()
            # release 成功、dispose 失败 -> 状态仍已清空、可再次 close
            self.assertEqual(len(util.released), 1)
            self.assertFalse(transport.is_open)
            transport.close()  # 幂等

    def test_close_release_and_dispose_both_fail_diagnostics(self) -> None:
        dev = _FakeDevice()
        core, util, patches = _patch_usb(dev=dev)
        with patches[0], patches[1]:
            transport = LibreVnaUsbTransport()
            transport.open()
            util.release_raises = _FakeUSBError("release boom")
            util.dispose_raises = _FakeUSBError("dispose boom")
            with self.assertRaises(LibreVnaReleaseError) as ctx:
                transport.close()
            msg = str(ctx.exception)
            # 两者都失败 -> 完整诊断信息
            self.assertIn("release_interface", msg)
            self.assertIn("dispose_resources", msg)
            self.assertIn("release boom", msg)
            self.assertIn("dispose boom", msg)
            self.assertFalse(transport.is_open)


class TransportOpenFailureCleanupTests(unittest.TestCase):
    """open() 在 find() 成功后失败：清理、错误映射、状态清空与重新 open。"""

    def test_open_get_active_configuration_failure(self) -> None:
        dev = _FakeDevice()
        core, util, patches = _patch_usb(dev=dev)
        with patches[0], patches[1]:
            dev.active_config_raises = _FakeUSBError("no active config")
            transport = LibreVnaUsbTransport()
            with self.assertRaises(LibreVnaBusyError) as ctx:
                transport.open()
            self.assertIsNotNone(ctx.exception.__cause__)  # 原始 USBError 保留
            self.assertFalse(transport.is_open)
            self.assertIsNone(transport._dev)  # noqa: SLF001
            # 尽力 release + dispose
            self.assertEqual(len(util.disposed), 1)

    def test_open_claim_failure_clears_state_then_reopen(self) -> None:
        dev = _FakeDevice()
        core, util, patches = _patch_usb(dev=dev)
        with patches[0], patches[1]:
            transport = LibreVnaUsbTransport()
            util.claim_raises = _FakeUSBError("claimed elsewhere")
            with self.assertRaises(LibreVnaBusyError):
                transport.open()
            self.assertFalse(transport.is_open)
            self.assertIsNone(transport._dev)  # noqa: SLF001
            # 占用解除后可在同一 transport 上重新 open
            util.claim_raises = None
            transport.open()
            self.assertTrue(transport.is_open)
            self.assertEqual(len(util.claimed), 1)
            transport.close()

    @unittest.skipUnless(
        sys.platform.startswith("win"), "Windows-only native libusb backend path"
    )
    def test_open_native_backend_missing_is_missing_dependency(self) -> None:
        # PyUSB 已装但 libusb_package 拿不到 DLL -> LibreVnaMissingDependencyError
        #（不能误报设备忙）。
        transport = LibreVnaUsbTransport()
        with mock.patch.object(
            LibreVnaUsbTransport,
            "_load_usb_modules",
            return_value=(_FakeUsbCore(), _FakeUsbUtil()),
        ), mock.patch(
            "librevna.librevna_transport.importlib.import_module",
            side_effect=ModuleNotFoundError("No module named 'libusb_package'"),
        ):
            with self.assertRaises(LibreVnaMissingDependencyError) as ctx:
                transport.open()
        self.assertIn("pip install", str(ctx.exception))
        self.assertFalse(transport.is_open)


class DefaultTransportTests(unittest.TestCase):
    def test_default_transport_constructed_without_hardware(self) -> None:
        # 构造默认后端（真实 transport）不应加载/打开硬件。
        transport = LibreVnaUsbTransport()
        self.assertFalse(transport.is_open)
        self.assertIsNone(transport._dev)  # noqa: SLF001
        transport.close()  # 未 open，no-op

    @unittest.skipIf(
        _librevna_device_present(),
        "本机已连接 LibreVNA，无法测‘未找到设备’路径（自动测试不得依赖硬件在场或缺场）",
    )
    def test_default_transport_open_not_found_friendly(self) -> None:
        # 无真实硬件时 open 抛 LibreVnaDeviceNotFoundError（不是裸异常）。
        transport = LibreVnaUsbTransport()
        try:
            with self.assertRaises(LibreVnaDeviceNotFoundError):
                transport.open()
        finally:
            try:
                transport.close()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    unittest.main()
