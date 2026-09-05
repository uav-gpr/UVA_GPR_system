"""Shared GNSS status and Leaflet map widget for the UAV-GPR GUIs."""

from __future__ import annotations

import json
import math
import time

from PySide6 import QtCore, QtWidgets
from PySide6.QtWebEngineWidgets import QWebEngineView


#: 地图 JavaScript 更新的最小间隔（毫秒）。GNSS 状态文本每条 Trace 都更新，
#: 但 WebEngine 地图最多以此频率重绘（第1轮检查点B：2～4Hz）。
MAP_UPDATE_INTERVAL_MS = 250


def _scalar(value: object) -> object:
    """Unwrap numpy scalars/zero-dimensional arrays without importing numpy."""
    while hasattr(value, "item"):
        try:
            unwrapped = value.item()
        except Exception:
            break
        if unwrapped is value:
            break
        value = unwrapped
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def _float_or_none(value: object) -> float | None:
    value = _scalar(value)
    if value is None or (isinstance(value, str) and not value):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _int_or_default(value: object, default: int = 0) -> int:
    """Convert a GNSS integer field without leaking numeric conversion errors."""
    value = _scalar(value)
    try:
        return int(value)
    except (OverflowError, TypeError, ValueError):
        return default


def _text(value: object, default: str = "") -> str:
    value = _scalar(value)
    return default if value is None else str(value)


class GnssMapWidget(QtWidgets.QWidget):
    """Show GNSS fields and maintain a map marker/track.

    The widget accepts primitive GNSS fields rather than an application-specific
    fix class, so it can be shared by the acquisition and ground-station GUIs.
    The status label remains useful when the WebEngine page or map tiles are
    unavailable.
    """

    def __init__(self) -> None:
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.status = QtWidgets.QLabel("GNSS: no valid fix (waiting for data)")
        self.status.setWordWrap(True)
        self.status.setStyleSheet("padding: 4px; background: #fff3cd; color: #7a4f00;")
        self._page_ready = False
        self._page_load_failed = False
        self._pending_map_update: tuple[float, float, str] | None = None
        # 地图限频（第1轮检查点B）：状态每条 Trace 更新，地图更新最多
        # 250ms 一次；坐标未实际变化时不调用 runJavaScript。
        self._map_timer = QtCore.QTimer(self)
        self._map_timer.setSingleShot(True)
        self._map_timer.setInterval(MAP_UPDATE_INTERVAL_MS)
        self._map_timer.timeout.connect(self._flush_map_update)
        self._last_sent_coords: tuple[float, float] | None = None
        # 复审修复：注入式 monotonic 时钟 + 最近一次成功 runJavaScript 时刻，
        # 保证任意两次 JS 调用间隔 >= 250ms（定时器之外的double-check）。
        self._monotonic = time.monotonic
        self._last_js_at: float | None = None
        self.web = QWebEngineView()
        self.web.loadFinished.connect(self._on_load_finished)
        try:
            self.web.setHtml(self._html())
        except Exception:
            self._on_load_finished(False)
        layout.addWidget(self.status)
        layout.addWidget(self.web, 1)

    def _on_load_finished(self, success: bool) -> None:
        """Track WebEngine readiness and replay the newest pending position.

        复审修复：loadFinished 后重放 pending 也必须进入 250ms 冷却（不再
        直接绕过限频发送）。
        """
        self._page_ready = bool(success)
        self._page_load_failed = not self._page_ready
        if not self._page_ready or self._pending_map_update is None:
            return
        # 经冷却路径重放：冷却外立即发送（并启动新冷却），冷却内等定时器。
        self._queue_or_send_map_update(*self._pending_map_update)

    def _send_map_update(self, latitude: float, longitude: float, popup: str) -> None:
        if not self._page_ready:
            raise RuntimeError("GNSS map page is not ready")
        page = self.web.page()
        if page is None:
            raise RuntimeError("GNSS map page is unavailable")
        script = f"window.updateGnss({latitude:.12f}, {longitude:.12f}, {json.dumps(popup)});"
        page.runJavaScript(script)

    def _queue_or_send_map_update(self, latitude: float, longitude: float, popup: str) -> None:
        """限频提交地图更新。

        - 坐标与上次已发送值完全一致时不调度任何 JavaScript；
        - 距上次成功发送不足 250ms（冷却中）时只保留最新待更新位置，定时器
          到期后补发；
        - 冷却外立即发送，且**每次成功发送后重新建立 250ms 最小间隔**；
        - 页面未就绪时只保留最新待更新位置（既有语义）。
        """
        if self._last_sent_coords == (latitude, longitude):
            return
        # Replacing the pending value guarantees that the newest valid fix is
        # the one replayed after a delayed page load or timer expiry.
        self._pending_map_update = (latitude, longitude, popup)
        if not self._page_ready:
            return
        if self._map_timer.isActive():
            # 冷却中：pending 保留，定时器到期由 _flush_map_update 补发。
            return
        # 冷却期外：立即发送（_flush_map_update 成功后会启动新冷却）。
        self._flush_map_update()

    def _flush_map_update(self) -> None:
        pending = self._pending_map_update
        if pending is None:
            return
        if not self._page_ready:
            return
        # 双保险：即使定时器异常提前触发，也用注入时钟检查最小间隔。
        now = self._monotonic()
        if self._last_js_at is not None and (now - self._last_js_at) < MAP_UPDATE_INTERVAL_MS / 1000.0 - 1e-9:
            self._map_timer.start()  # 未到 250ms：延后重试。
            return
        self._pending_map_update = None
        try:
            self._send_map_update(*pending)
        except Exception:
            self._pending_map_update = pending
            self._page_ready = False
            self._page_load_failed = True
            return
        self._last_sent_coords = (pending[0], pending[1])
        self._last_js_at = self._monotonic()
        # 每次成功 runJavaScript 后重新建立 250ms 最小间隔（复审修复：
        # 定时器到期发送后不再自动冷却，导致下一次可紧贴发送）。
        self._map_timer.start()
        if self._pending_map_update is not None:
            pass  # 冷却期内的新坐标由已启动的定时器到期补发。

    @staticmethod
    def _html() -> str:
        return """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    html, body, #map { height: 100%; margin: 0; }
    .leaflet-container { font-family: Segoe UI, Arial, sans-serif; }
  </style>
</head>
<body>
  <div id="map"></div>
  <script>
    const map = L.map('map').setView([31.2304, 121.4737], 14);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 20,
      attribution: '&copy; OpenStreetMap'
    }).addTo(map);
    let marker = null;
    const path = [];
    const polyline = L.polyline(path, { color: '#e63946', weight: 4 }).addTo(map);
    window.updateGnss = function(lat, lon, text) {
      if (lat === null || lon === null || Number.isNaN(lat) || Number.isNaN(lon)) {
        return;
      }
      const ll = [lat, lon];
      path.push(ll);
      polyline.setLatLngs(path);
      if (!marker) {
        marker = L.circleMarker(ll, { radius: 7, color: '#005f73', fillColor: '#0a9396', fillOpacity: 0.9 }).addTo(map);
        map.setView(ll, 18);
      } else {
        marker.setLatLng(ll);
        map.panTo(ll);
      }
      marker.bindPopup(text || '').openPopup();
    };
  </script>
</body>
</html>
"""

    def update_fix(
        self,
        *,
        latitude: object = None,
        longitude: object = None,
        fix_quality: object = 0,
        satellites: object = 0,
        altitude_m: object = None,
        utc: object = "",
    ) -> None:
        lat = _float_or_none(latitude)
        lon = _float_or_none(longitude)
        quality = _int_or_default(fix_quality)
        satellite_count = _int_or_default(satellites)
        altitude = _float_or_none(altitude_m)
        utc_text = _text(utc, "-")
        alt_text = f"{altitude:g} m" if altitude is not None else "-"
        coordinate_valid = lat is not None and lon is not None and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0
        fix_valid = quality > 0

        if coordinate_valid and fix_valid:
            self.status.setText(
                f"GNSS: fix={quality}, sats={satellite_count}, "
                f"lat={lat:.8f}, lon={lon:.8f}, alt={alt_text}, UTC={utc_text}"
            )
            self.status.setStyleSheet("padding: 4px; background: #d1fae5; color: #064e3b;")
            popup = (
                f"UTC {utc_text}<br>lat {lat:.8f}<br>lon {lon:.8f}<br>"
                f"alt {alt_text}<br>fix {quality}, sats {satellite_count}"
            )
            self._queue_or_send_map_update(lat, lon, popup)
            return

        if coordinate_valid:
            status_reason = "no valid fix; position ignored"
        else:
            status_reason = "no valid lat/lon; waiting for a valid position"
        self.status.setText(
            f"GNSS: {status_reason}, fix={quality}, sats={satellite_count}, "
            f"alt={alt_text}, UTC={utc_text}"
        )
        self.status.setStyleSheet("padding: 4px; background: #fff3cd; color: #7a4f00;")
