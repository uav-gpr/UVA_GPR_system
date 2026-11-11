"""Opt-in GNSS diagnostics logger, solution classification, and fix eligibility policy."""
from __future__ import annotations

from collections import deque
from collections.abc import Callable, Mapping
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import time

SOLUTION_NO_FIX = "no_fix"
SOLUTION_SPP = "spp_fix"
SOLUTION_DGPS = "dgps_fix"
SOLUTION_RTK_FIXED = "rtk_fixed"
SOLUTION_RTK_FLOAT = "rtk_float"
SOLUTION_OTHER = "other_fix"

SOLUTION_TYPES = (
    SOLUTION_NO_FIX,
    SOLUTION_SPP,
    SOLUTION_DGPS,
    SOLUTION_RTK_FIXED,
    SOLUTION_RTK_FLOAT,
)

FIX_POLICY_OFF = "off"
FIX_POLICY_STRICT_RTK_FIXED = "strict_rtk_fixed"
FIX_POLICY_RTK_ANY = "rtk_any"
FIX_POLICY_DGPS_OR_BETTER = "dgps_or_better"

FIX_POLICY_ENV = "UAV_GPR_GNSS_FIX_POLICY"
DIAG_LOG_ENV = "UAV_GPR_GNSS_DIAG_LOG"
DIAG_MAX_BYTES_ENV = "UAV_GPR_GNSS_DIAG_MAX_BYTES"
DIAG_BACKUPS_ENV = "UAV_GPR_GNSS_DIAG_BACKUPS"

DEFAULT_DIAG_LOG_REL_PATH = Path("logs") / "gnss_diagnostics.jsonl"
DEFAULT_DIAG_MAX_BYTES = 5 * 1024 * 1024
DEFAULT_DIAG_BACKUP_COUNT = 5


def _utc_iso_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _extract_field(fix: object, name: str, alt_name: str | None = None) -> object:
    if isinstance(fix, Mapping):
        if name in fix:
            return fix[name]
        if alt_name is not None and alt_name in fix:
            return fix[alt_name]
        return None
    val = getattr(fix, name, None)
    if val is None and alt_name is not None:
        val = getattr(fix, alt_name, None)
    return val


def _sentence_kind_from_raw(raw: str) -> str:
    text = raw.strip()
    if not text:
        return ""
    if len(text) >= 7 and text[0] == "$" and text[6] == "," and text[1:6].isalpha():
        return text[3:6].upper()
    return "UNKNOWN"


def _is_geometrically_valid(fix: object) -> tuple[bool, int]:
    if fix is None or isinstance(fix, (bool, int, float, str, bytes)):
        return False, 0
    quality = _extract_field(fix, "fix_quality")
    if isinstance(quality, bool) or not isinstance(quality, int) or quality <= 0:
        return False, 0
    lat_raw = _extract_field(fix, "latitude", "lat")
    lon_raw = _extract_field(fix, "longitude", "lon")
    if isinstance(lat_raw, bool) or not isinstance(lat_raw, (int, float)):
        return False, 0
    if isinstance(lon_raw, bool) or not isinstance(lon_raw, (int, float)):
        return False, 0
    lat = float(lat_raw)
    lon = float(lon_raw)
    if not math.isfinite(lat) or not math.isfinite(lon):
        return False, 0
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return False, 0
    return True, quality


def solution_type(fix: object, *, source_kind: str = "") -> str:
    """Classify GNSS fix into no_fix / spp_fix / dgps_fix / rtk_fixed / rtk_float.

    Only a geometrically valid GGA fix with ``fix_quality == 4`` is classified
    as ``rtk_fixed`` (and ``fix_quality == 5`` as ``rtk_float``). Non-GGA fixes
    (such as RMC) never claim RTK status even if quality metadata was altered.
    """
    valid, quality = _is_geometrically_valid(fix)
    if not valid:
        return SOLUTION_NO_FIX

    raw_val = _extract_field(fix, "raw")
    raw_text = raw_val if isinstance(raw_val, str) else ""
    raw_kind = _sentence_kind_from_raw(raw_text)
    explicit_kind = source_kind.strip().upper()
    is_gga_source = explicit_kind in ("", "GGA") and raw_kind in ("", "GGA")

    if quality == 1:
        return SOLUTION_SPP
    if quality == 2:
        return SOLUTION_DGPS
    if quality == 4:
        return SOLUTION_RTK_FIXED if is_gga_source else SOLUTION_SPP
    if quality == 5:
        return SOLUTION_RTK_FLOAT if is_gga_source else SOLUTION_SPP
    return SOLUTION_OTHER


def normalize_fix_policy(policy: object = None, *, strict_validate: bool = True) -> str:
    """Normalize fix eligibility policy setting (defaults to 'off')."""
    if policy is None:
        env_raw = os.environ.get(FIX_POLICY_ENV, "").strip()
        if not env_raw:
            return FIX_POLICY_OFF
        return normalize_fix_policy(env_raw, strict_validate=False)
    if isinstance(policy, bool):
        return FIX_POLICY_STRICT_RTK_FIXED if policy else FIX_POLICY_OFF
    if isinstance(policy, str):
        token = policy.strip().lower()
        if token in ("", "off", "false", "0", "none", "default", "any", "any_valid"):
            return FIX_POLICY_OFF
        if token in ("strict", "strict_rtk_fixed", "rtk_fixed", "rtk_fixed_only", "on", "true", "1"):
            return FIX_POLICY_STRICT_RTK_FIXED
        if token in ("rtk", "rtk_any", "rtk_float_or_fixed"):
            return FIX_POLICY_RTK_ANY
        if token in ("dgps", "dgps_or_better"):
            return FIX_POLICY_DGPS_OR_BETTER
    if strict_validate:
        raise ValueError(f"Unsupported GNSS fix policy: {policy!r}")
    return FIX_POLICY_OFF


def fix_policy(
    fix: object,
    policy: object = FIX_POLICY_OFF,
    *,
    source_kind: str = "",
) -> dict[str, object]:
    """Evaluate solution classification and optional fix eligibility metadata.

    Strict fix policy is opt-in (default ``off``) and affects ONLY eligibility
    metadata fields (``fix_eligible``, ``eligibility_reason``), never altering
    existing ``fix`` validity, NPZ 10-field GNSS dicts, or ground telemetry.
    """
    mode = normalize_fix_policy(policy)
    sol = solution_type(fix, source_kind=source_kind)
    if sol == SOLUTION_NO_FIX:
        eligible = False
        reason = "no_fix"
    elif mode == FIX_POLICY_OFF:
        eligible = True
        reason = "any_valid_fix"
    elif mode == FIX_POLICY_STRICT_RTK_FIXED:
        eligible = sol == SOLUTION_RTK_FIXED
        reason = "rtk_fixed_ok" if eligible else f"requires_rtk_fixed(got={sol})"
    elif mode == FIX_POLICY_RTK_ANY:
        eligible = sol in (SOLUTION_RTK_FIXED, SOLUTION_RTK_FLOAT)
        reason = "rtk_ok" if eligible else f"requires_rtk(got={sol})"
    else:
        eligible = sol in (SOLUTION_DGPS, SOLUTION_RTK_FLOAT, SOLUTION_RTK_FIXED)
        reason = "dgps_or_better_ok" if eligible else f"requires_dgps_or_better(got={sol})"
    return {
        "solution_type": sol,
        "is_rtk_fixed": sol == SOLUTION_RTK_FIXED,
        "fix_policy": mode,
        "strict_fix_policy": mode != FIX_POLICY_OFF,
        "fix_eligible": eligible,
        "eligibility_reason": reason,
    }


def _json_safe(value: object) -> object:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (bytes, bytearray, memoryview)):
        raw = bytes(value)
        return {"len": len(raw), "hex": raw.hex()}
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, deque)):
        return [_json_safe(v) for v in value]
    return repr(value)


def _ascii_preview(data: bytes, limit: int = 128) -> str:
    clipped = data[:limit]
    return "".join(chr(b) if 32 <= b < 127 else f"\\x{b:02x}" for b in clipped)


class GnssLogWriter:
    """Opt-in rolling JSONL diagnostic logger for GNSS serial I/O and state.

    Default disabled (``enabled=False``): zero disk touches unless explicitly
    enabled in code or via ``UAV_GPR_GNSS_DIAG_LOG``. Resilient to serial
    reconnects, file size rotation, and disk write/permission errors.
    """

    def __init__(
        self,
        path: str | Path | None = None,
        *,
        enabled: bool = False,
        max_bytes: int = DEFAULT_DIAG_MAX_BYTES,
        backup_count: int = DEFAULT_DIAG_BACKUP_COUNT,
        utc_now: Callable[[], str] | None = None,
        opener: Callable[[Path], object] | None = None,
    ) -> None:
        if isinstance(max_bytes, bool) or not isinstance(max_bytes, int) or max_bytes <= 0:
            raise ValueError(f"max_bytes must be a positive int, got {max_bytes!r}")
        if isinstance(backup_count, bool) or not isinstance(backup_count, int) or backup_count < 0:
            raise ValueError(f"backup_count must be a non-negative int, got {backup_count!r}")
        self._lock = threading.Lock()
        self.enabled = bool(enabled)
        self._path: Path | None = (
            Path(path) if path is not None else (DEFAULT_DIAG_LOG_REL_PATH if self.enabled else None)
        )
        self.max_bytes = max_bytes
        self.backup_count = backup_count
        self._utc_now = utc_now or _utc_iso_now
        self._opener = opener
        self._fh: object | None = None
        self.seq = 0
        self.records_written = 0
        self.bytes_written = 0
        self.rotation_count = 0
        self.write_errors = 0
        self.last_error = ""

    @classmethod
    def from_env(cls) -> GnssLogWriter:
        """Build a GnssLogWriter from environment variables (disabled by default)."""
        raw = os.environ.get(DIAG_LOG_ENV, "").strip()
        if not raw or raw.lower() in ("0", "false", "off", "disabled", "no"):
            return cls(enabled=False)
        path = (
            DEFAULT_DIAG_LOG_REL_PATH
            if raw.lower() in ("1", "true", "on", "enabled", "yes")
            else Path(raw)
        )
        max_bytes = DEFAULT_DIAG_MAX_BYTES
        raw_max = os.environ.get(DIAG_MAX_BYTES_ENV, "").strip()
        if raw_max:
            try:
                parsed_max = int(raw_max)
                if parsed_max > 0:
                    max_bytes = parsed_max
            except ValueError:
                pass
        backups = DEFAULT_DIAG_BACKUP_COUNT
        raw_backups = os.environ.get(DIAG_BACKUPS_ENV, "").strip()
        if raw_backups:
            try:
                parsed_backups = int(raw_backups)
                if parsed_backups >= 0:
                    backups = parsed_backups
            except ValueError:
                pass
        return cls(path=path, enabled=True, max_bytes=max_bytes, backup_count=backups)

    @property
    def path(self) -> Path | None:
        return self._path

    def configure(self, path: str | Path | None = None, *, enabled: bool | None = None) -> None:
        with self._lock:
            self._close_unlocked()
            if path is not None:
                self._path = Path(path)
            if enabled is not None:
                self.enabled = bool(enabled)
                if self.enabled and self._path is None:
                    self._path = DEFAULT_DIAG_LOG_REL_PATH

    def close(self) -> None:
        with self._lock:
            self._close_unlocked()

    def _close_unlocked(self) -> None:
        fh = self._fh
        self._fh = None
        if fh is not None:
            try:
                fh.close()
            except Exception:  # noqa: BLE001
                pass

    def _rotate_unlocked(self, target: Path) -> None:
        self._close_unlocked()
        if self.backup_count <= 0:
            if target.exists():
                target.unlink()
            self.rotation_count += 1
            return
        oldest = Path(f"{target}.{self.backup_count}")
        if oldest.exists():
            oldest.unlink()
        for idx in range(self.backup_count - 1, 0, -1):
            src = Path(f"{target}.{idx}")
            dst = Path(f"{target}.{idx + 1}")
            if src.exists():
                os.replace(src, dst)
        if target.exists():
            os.replace(target, Path(f"{target}.1"))
        self.rotation_count += 1

    def write_event(self, event: str, **fields: object) -> bool:
        """Write a single JSONL record; never raises on I/O or encoding errors."""
        if not self.enabled:
            return False
        with self._lock:
            if not self.enabled or self._path is None:
                return False
            self.seq += 1
            record: dict[str, object] = {
                "seq": self.seq,
                "ts": self._utc_now(),
                "monotonic": round(time.monotonic(), 6),
                "event": str(event),
            }
            for key, value in fields.items():
                record[str(key)] = _json_safe(value)
            try:
                encoded = (json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")
                target = self._path
                target.parent.mkdir(parents=True, exist_ok=True)
                current_size = target.stat().st_size if target.exists() else 0
                if self.max_bytes > 0 and current_size > 0 and current_size + len(encoded) > self.max_bytes:
                    self._rotate_unlocked(target)
                if self._fh is None:
                    self._fh = (
                        self._opener(target)
                        if self._opener is not None
                        else target.open("ab")
                    )
                self._fh.write(encoded)
                self._fh.flush()
                self.records_written += 1
                self.bytes_written += len(encoded)
                return True
            except Exception as exc:  # noqa: BLE001 - diagnostic logging must never disrupt GNSS/radar
                self._close_unlocked()
                self.write_errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                return False

    def log_raw_bytes(self, chunk: bytes, *, port: str = "", baud: int | None = None) -> bool:
        if not self.enabled or not chunk:
            return False
        raw = bytes(chunk)
        return self.write_event(
            "raw_rx",
            port=port,
            baud=baud,
            length=len(raw),
            hex=raw.hex(),
            ascii=_ascii_preview(raw),
        )

    def log_sentence(self, record: Mapping[str, object], *, port: str = "", baud: int | None = None) -> bool:
        if not self.enabled:
            return False
        return self.write_event("sentence", port=port, baud=baud, **dict(record))

    def log_command(
        self,
        payload: bytes | str,
        *,
        direction: str = "tx",
        kind: str = "command",
        port: str = "",
        baud: int | None = None,
        phase: int | None = None,
    ) -> bool:
        if not self.enabled:
            return False
        if isinstance(payload, (bytes, bytearray)):
            raw = bytes(payload)
            text = raw.decode("ascii", errors="replace").strip()
            hex_text = raw.hex()
        else:
            text = str(payload).strip()
            hex_text = text.encode("utf-8", errors="replace").hex()
        return self.write_event(
            "command",
            direction=direction,
            kind=kind,
            port=port,
            baud=baud,
            phase=phase,
            text=text,
            hex=hex_text,
        )

    def log_response(
        self,
        line: str,
        *,
        kind: str = "response",
        port: str = "",
        baud: int | None = None,
    ) -> bool:
        if not self.enabled:
            return False
        return self.write_event(
            "response",
            kind=kind,
            port=port,
            baud=baud,
            text=str(line)[:200],
        )

    def log_connection(
        self,
        state: str,
        *,
        port: str = "",
        baud: int | None = None,
        configured_port: str = "",
        configured_baud: int | None = None,
        detail: str = "",
    ) -> bool:
        if not self.enabled:
            return False
        return self.write_event(
            "connection",
            state=state,
            port=port,
            baud=baud,
            configured_port=configured_port,
            configured_baud=configured_baud,
            detail=detail,
        )

    def log_publish(self, snapshot: Mapping[str, object], *, detail: str = "") -> bool:
        if not self.enabled:
            return False
        return self.write_event("publish", detail=detail, snapshot=dict(snapshot))

    def statistics(self) -> dict[str, object]:
        with self._lock:
            return {
                "enabled": self.enabled,
                "path": str(self._path) if self._path is not None else None,
                "seq": self.seq,
                "records_written": self.records_written,
                "bytes_written": self.bytes_written,
                "rotation_count": self.rotation_count,
                "write_errors": self.write_errors,
                "last_error": self.last_error,
            }
