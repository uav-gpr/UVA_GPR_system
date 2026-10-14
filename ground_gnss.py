"""Ground-only GNSS telemetry display; never opens a serial port or stops acquisition."""
from __future__ import annotations

import json

from PySide6 import QtCore
from gnss_map_widget import GnssMapWidget, _float_or_none, _int_or_default, _scalar


#: NMEA-0183 GGA 句子最短结构（"$xxGGA," 需要 7 个字符）。
_GGA_MIN_CHARS = 7


def _is_gga_sentence(raw: str) -> bool:
    """是否 NMEA-0183 GGA 报文。

    原实现只认 ``$GPGGA,`` / ``$GNGGA,`` / ``$BDGGA,`` 三个字面量；M100 可能
    用别的 talker（GAGGA/GLGGA/GBGGA…）输出同一 GGA 结构，这里按 talker+type
    判定，三种必需形态仍然覆盖。
    """
    return (len(raw) >= _GGA_MIN_CHARS and raw[0] == "$" and raw[6] == ","
            and raw[1:6].isalpha() and raw[3:6].upper() == "GGA")


def normalize_gnss(value: object) -> dict:
    """Accept existing GNSS JSON/ack fields and forwarded GGA, including no-fix GGA."""
    value = _scalar(value)
    if isinstance(value, str):
        value = {"raw": value} if value.strip().startswith("$") else json.loads(value)
    if not isinstance(value, dict):
        return {}
    result = dict(value)
    raw = str(result.get("raw") or "").strip()
    if _is_gga_sentence(raw):
        body, sep, checksum = raw.partition("*")
        if sep:
            actual = 0
            for char in body[1:]:
                actual ^= ord(char)
            if actual != int(checksum, 16):
                raise ValueError("GGA checksum mismatch")
        parts = body.split(",")
        if len(parts) < 10:
            raise ValueError("Incomplete GGA")

        def coordinate(text: str, hemisphere: str, positive: str, negative: str):
            if not text or not hemisphere:
                return None
            if hemisphere not in (positive, negative):
                raise ValueError("Invalid GGA hemisphere")
            number = float(text)
            degrees, minutes = divmod(number, 100)
            if number < 0 or not 0 <= minutes < 60:
                raise ValueError("Invalid GGA coordinate")
            return (degrees + minutes / 60) * (-1 if hemisphere == negative else 1)

        result.update(
            utc=parts[1], latitude=coordinate(parts[2], parts[3], "N", "S"),
            longitude=coordinate(parts[4], parts[5], "E", "W"),
            fix_quality=int(parts[6] or 0), satellites=int(parts[7] or 0),
            altitude_m=float(parts[9]) if parts[9] else None,
        )
    if "latitude" not in result and "lat" in result:
        result["latitude"] = result.get("lat")
    if "longitude" not in result and "lon" in result:
        result["longitude"] = result.get("lon")
    if "fix_quality" not in result and result.get("fix") is True:
        # Legacy test_gnss ack only reports a boolean fix; no RTK requirement.
        result["fix_quality"] = 1
    return result


class GroundGnssMapWidget(GnssMapWidget):
    # Data freshness only: no deadline on satellite search / acquiring a fix.
    DATA_TIMEOUT_S = 15.0

    def __init__(self):
        super().__init__()
        self._link_connected = False
        self._last_data_at = None
        self._last_utc = None
        self._gnss_watchdog = QtCore.QTimer(self)
        self._gnss_watchdog.setInterval(1000)
        self._gnss_watchdog.timeout.connect(self.check_data_timeout)
        self._gnss_watchdog.start()
        self.set_link_connected(False)

    def show_gnss_status(self, text: str):
        self.status.setText("GNSS：" + text)
        self.status.setStyleSheet("padding: 4px; background: #fff3cd; color: #7a4f00;")
        self._pending_map_update = None
        self._map_timer.stop()

    def set_link_connected(self, connected: bool):
        changed = self._link_connected != bool(connected)
        self._link_connected = bool(connected)
        if changed:
            self._last_data_at = None
            self._last_utc = None
        if not connected:
            self.show_gnss_status("数据链未连接")
        elif changed:
            self.show_gnss_status("数据链已连接，尚未收到 GNSS 数据（等待天空端转发）")

    def check_data_timeout(self):
        if (self._link_connected and self._last_data_at is not None
                and self._monotonic() - self._last_data_at > self.DATA_TIMEOUT_S):
            self.show_gnss_status("数据中断/无新数据（最后坐标仅供历史参考）")

    def update_data(self, value: object, *, age_ms: object = None):
        """GNSS errors stay in this widget; never escape into the radar commit path."""
        try:
            data = normalize_gnss(value)
            if not self._link_connected:
                return
            if data.get("serial_open") is False or data.get("connected") is False:
                self.show_gnss_status("天空端 GNSS 串口未连接")
                return
            lat = _float_or_none(data.get("latitude"))
            lon = _float_or_none(data.get("longitude"))
            quality = _int_or_default(data.get("fix_quality"))
            sats = _int_or_default(data.get("satellites"))
            utc = str(data.get("utc") or "")
            # Empty GgaFix is an upstream placeholder, not proof of zero satellites.
            received = bool(data.get("raw") or utc or lat is not None or lon is not None
                            or sats > 0 or quality > 0 or data.get("nmea_received") is True)
            if not received:
                # 天空端带上来的分类明细（无字节 / 有字节非 NMEA / NMEA 被拒）直接显示，
                # 现场不必再猜"到底是打不开、收不到还是格式不对"。
                detail = str(_scalar(data.get("detail")) or "").strip()
                state = str(_scalar(data.get("state")) or "").strip()
                if detail or state:
                    self.show_gnss_status(
                        f"数据链已连接，尚未收到 GNSS 数据（天空端：{detail or state}）")
                elif self._last_data_at is None:
                    self.show_gnss_status("数据链已连接，尚未收到 GNSS 数据（天空端未提供 NMEA 状态）")
                else:
                    self.show_gnss_status("无当前 GNSS 数据（最后坐标仅供历史参考）")
                self.check_data_timeout()
                return
            now = self._monotonic()
            # Repeated cached fixes with the same UTC must not stay green forever.
            if not utc or utc != self._last_utc:
                age = _float_or_none(age_ms)
                self._last_data_at = now - max(0.0, age or 0.0) / 1000.0
                self._last_utc = utc
            if self._last_data_at is not None and now - self._last_data_at > self.DATA_TIMEOUT_S:
                self.check_data_timeout()
                return
            super().update_fix(latitude=lat, longitude=lon, fix_quality=quality,
                               satellites=sats, altitude_m=data.get("altitude_m"), utc=utc)
            if quality > 0 and lat is not None and lon is not None and -90 <= lat <= 90 and -180 <= lon <= 180:
                kind = "普通 GNSS 单点定位成功" if quality == 1 else "GNSS 定位成功"
                self.status.setText(kind + " | " + self.status.text())
            else:
                if sats > 0:
                    reason = "已收到卫星，尚未定位"
                elif data.get("raw") or utc or data.get("nmea_received") is True:
                    reason = "已收到 NMEA，卫星数为 0，等待搜星"
                else:
                    reason = "已收到 GNSS 数据，尚未定位"
                self.show_gnss_status(f"{reason} | Fix={quality} 卫星={sats}")
        except Exception:
            self.show_gnss_status("数据格式/显示异常，等待下一条 GNSS 数据")
