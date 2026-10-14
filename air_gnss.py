"""Always-on M100 GNSS owner for the black-box service (no USB radar/network I/O)."""
from __future__ import annotations

import threading
import time
from dataclasses import asdict, replace

from serial.tools import list_ports
import uav_gpr_qt as air


class AirGnssMonitor(air.GnssFixCache):
    #: 无 NMEA 时诊断状态的重复发布间隔（状态变化即刻发布；同状态按此节流）。
    DIAGNOSTIC_REPUBLISH_S = 5.0

    def __init__(self, port="", baud=115200, *, reader_factory=None, port_scanner=None,
                 log=None, initial_retry_s=1.0, max_retry_s=5.0, data_timeout_s=30.0):
        super().__init__(max_age_s=data_timeout_s)
        self._settings_lock = threading.Lock()
        self._settings = (port, baud)
        self._generation = 0
        self._wake = threading.Event()
        self._stopping = threading.Event()
        self._reader_factory = reader_factory or air.GnssReader
        self._port_scanner = port_scanner or list_ports.comports
        self._log = log or (lambda text: None)
        self._initial_retry_s = initial_retry_s
        self._max_retry_s = max_retry_s
        self._data_timeout_s = data_timeout_s
        self._identity = None
        self._port = ""
        self._detail = "等待 GNSS 后台启动"
        # 无数据分类（no_bytes / raw_not_nmea / nmea_rejected）发布节流状态。
        self._diag_state = ""
        self._diag_detail = ""
        self._diag_published_at = float("-inf")
        self._thread = threading.Thread(target=self._run, name="m100-gnss", daemon=True)
        self._publish(air.GgaFix(serial_open=False, state="waiting"))

    def start(self):
        if not self._thread.is_alive():
            self._thread.start()

    def stop(self):
        self._stopping.set()
        self._wake.set()
        if self._thread.is_alive():
            self._thread.join(2.5)  # A stuck serial driver cannot hold service shutdown forever.

    def configure(self, port, baud):
        with self._settings_lock:
            self._settings = (port, baud)
            self._generation += 1
            self._identity = None
        self._wake.set()

    @staticmethod
    def _device_identity(device):
        serial_number = getattr(device, "serial_number", None)
        if serial_number:
            return (getattr(device, "vid", None), getattr(device, "pid", None), serial_number)
        return None

    def _find_port(self, configured):
        devices = list(self._port_scanner())
        if self._identity is not None:
            matches = [p for p in devices if self._device_identity(p) == self._identity]
            if len(matches) == 1:
                return matches[0].device
            return None
        if configured:
            for p in devices:
                if p.device == configured:
                    self._identity = self._device_identity(p)
            return configured
        # Do not write receiver commands to arbitrary COM ports / unrelated devices.
        matches = [p for p in devices if any(name in " ".join(
            str(getattr(p, attr, "") or "") for attr in ("description", "manufacturer", "product")
        ).lower() for name in ("comnav", "sinognss", "司南", "m100"))]
        if len(matches) == 1:
            self._identity = self._device_identity(matches[0])
            return matches[0].device
        return None

    def _publish(self, fix, detail=""):
        with self._lock:
            previous = self._fixes[-1].fix.state if self._fixes else None
            self._fixes.append(air.TimestampedFix(fix, self._clock(), self._utc_now()))
            self._detail = detail
            self.update_count += 1
        if fix.state != previous:
            try:
                self._log(f"GNSS: {fix.state} {self._port} {detail}")
            except Exception:
                pass

    def latest(self, as_of_monotonic=None):
        lookup = super().latest(as_of_monotonic)
        if lookup.fix is None:
            return lookup
        if not lookup.valid:
            return air.GnssFixLookup(air.GgaFix(serial_open=False, state="interrupted"), lookup.age_ms, False)
        return air.GnssFixLookup(lookup.fix, lookup.age_ms, air.is_valid_gnss_fix(lookup.fix))

    def telemetry_fix(self, lookup):
        fix = lookup.fix or air.GgaFix(serial_open=False, state="waiting")
        if lookup.valid:
            return fix
        # Keep no-fix NMEA/satellite information, without advertising invalid coordinates.
        return replace(fix, latitude=None, longitude=None, fix_quality=0)

    def snapshot(self):
        lookup = self.latest()
        result = asdict(self.telemetry_fix(lookup))
        with self._settings_lock:
            baud = self._settings[1]
        with self._lock:
            detail = self._detail
        result.update(ok=True, fix=lookup.valid, port=self._port, baud=baud, detail=detail,
                      lat=result["latitude"], lon=result["longitude"])
        return result

    def _wait(self, seconds):
        self._wake.wait(seconds)

    def _make_reader(self, port, baud):
        """构造读取器并把 [GNSS] 诊断日志接到服务日志（无 log 的工厂也能用）。"""
        reader = self._reader_factory(port, baud)
        try:
            reader.log = self._log
        except Exception:  # noqa: BLE001 - 不支持注入日志的读取器照常工作
            pass
        return reader

    def _publish_reader_diagnostics(self, reader):
        """把读取器的无数据分类发布给地面端（区分无字节 / 有字节非 NMEA / NMEA 被拒）。

        只在分类或诊断文本变化时（或 5s 节流到点）写一条占位 Fix，避免把缓存刷成
        永远"新鲜"从而掩盖真实的数据中断。
        """
        diagnose = getattr(reader, "diagnostics", None)
        if not callable(diagnose):
            return
        # 只在"确实安静"（≥2s 没有一条 NMEA）时发布占位状态：正常 1Hz 报文流
        # 间隔导致的空读不覆盖刚收到的 no_satellites / no_fix 状态。
        silence = getattr(reader, "silence_seconds", None)
        if callable(silence):
            try:
                if float(silence()) < air.GnssReader.SILENCE_BEFORE_POLL_S:
                    return
            except Exception:  # noqa: BLE001 - 诊断失败不影响重连/采集
                return
        try:
            detail = str(diagnose())
        except Exception:  # noqa: BLE001 - 诊断失败不影响重连/采集
            return
        rx_bytes = getattr(reader, "rx_bytes", 0)
        if rx_bytes and getattr(reader, "parse_errors", 0):
            state = "nmea_rejected"
        elif rx_bytes and getattr(reader, "detected_sentences", 0):
            state = "nmea_gap"
        elif rx_bytes:
            state = "raw_not_nmea"
        else:
            state = "no_bytes"
        now = time.monotonic()
        if (state == self._diag_state and detail == self._diag_detail
                and now - self._diag_published_at < self.DIAGNOSTIC_REPUBLISH_S):
            return
        self._diag_state = state
        self._diag_detail = detail
        self._diag_published_at = now
        self._publish(air.GgaFix(serial_open=True, state=state), detail)

    def _run(self):
        retry_s = self._initial_retry_s
        while not self._stopping.is_set():
            self._wake.clear()
            with self._settings_lock:
                configured, baud = self._settings
                generation = self._generation
            reader = None
            try:
                if configured.lower() in {"off", "disabled"}:
                    self._publish(air.GgaFix(serial_open=False, state="disabled"))
                    self._wait(self._max_retry_s)
                    continue
                self._port = self._find_port(configured) or ""
                if not self._port:
                    self._publish(air.GgaFix(serial_open=False, state="port_not_found"),
                                  "等待 M100；通用 USB 转串口请配置 gnss_port，避免误连其他设备")
                    self._wait(retry_s)
                    retry_s = min(retry_s * 2, self._max_retry_s)
                    continue
                self._publish(air.GgaFix(serial_open=False, state="connecting"))
                reader = self._make_reader(self._port, baud)
                reader.open()
                self._publish(air.GgaFix(serial_open=True, state="connected"))
                last_data = time.monotonic()
                self._diag_state = ""
                self._diag_detail = ""
                while not self._stopping.is_set() and not self._wake.is_set():
                    with self._settings_lock:
                        if generation != self._generation:
                            break
                    fix = reader.read_gga(timeout_s=0.5)
                    if fix.nmea_received:
                        last_data = time.monotonic()
                        retry_s = self._initial_retry_s
                        self._diag_state = ""
                        self._diag_detail = ""
                        state = ("spp_fix" if fix.fix_quality == 1 else "fix") if air.is_valid_gnss_fix(fix) else (
                            "no_fix" if fix.satellites > 0 else "no_satellites")
                        self._publish(replace(fix, serial_open=True, state=state),
                                      f"satellites={fix.satellites} fix={fix.fix_quality}")
                    else:
                        # 串口已打开但本轮没有可解析 NMEA：[GNSS] 明细已由 GnssReader
                        # 记录；这里把分类（无字节 / 有字节非 NMEA / NMEA 被拒）发布给
                        # 地面端，再按既有 30s 超时重连恢复输出。
                        self._publish_reader_diagnostics(reader)
                        if time.monotonic() - last_data >= self._data_timeout_s:
                            raise TimeoutError("持续无有效 NMEA，重新打开并恢复输出")
                    self._wake.wait(0.02)  # Also bounds CPU use with nonblocking/mock readers.
            except Exception as exc:
                self._publish(air.GgaFix(serial_open=False, state="reconnecting"), str(exc))
            finally:
                if reader is not None:
                    try:
                        reader.close()
                    except Exception:
                        pass
            if not self._stopping.is_set():
                self._wait(retry_s)
                retry_s = min(retry_s * 2, self._max_retry_s)
