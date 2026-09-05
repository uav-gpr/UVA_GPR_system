"""Two-port LibreVNA acquisition reference for the UAV-GPR project.

"Dual channel" means two VNA ports.  A complete sweep excites Port 1 and
Port 2 in separate stages and returns all four complex S-parameters:

    Port 1 excitation: S11 = Rx1 / Ref1, S21 = Rx2 / Ref1
    Port 2 excitation: S12 = Rx1 / Ref2, S22 = Rx2 / Ref2

The production USB packet parsing remains in ``librevna/librevna_usb_s11.py``.
This module supplies a small, validated API and an NPZ interchange format that
can also be exercised with a fake backend in unit tests.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Mapping, Sequence

import numpy as np
from numpy.typing import NDArray


S_PARAMETERS = ("S11", "S21", "S12", "S22")
RawPoint = tuple[float, Mapping[str, complex]]
AcquisitionBackend = Callable[
    [int, int, int, int, float, int],
    Sequence[RawPoint],
]


@dataclass(frozen=True)
class SweepSettings:
    """Hardware sweep settings shared by measurement and calibration."""

    start_hz: int = 400_000_000
    stop_hz: int = 4_000_000_000
    points: int = 201
    ifbw_hz: int = 10_000
    power_dbm: float = -20.0
    dwell_us: int = 0

    def __post_init__(self) -> None:
        if self.start_hz <= 0:
            raise ValueError("start_hz must be positive")
        if self.stop_hz <= self.start_hz:
            raise ValueError("stop_hz must be greater than start_hz")
        if self.points < 2:
            raise ValueError("points must be at least 2")
        if self.ifbw_hz <= 0:
            raise ValueError("ifbw_hz must be positive")
        if not np.isfinite(self.power_dbm):
            raise ValueError("power_dbm must be finite")
        if self.dwell_us < 0:
            raise ValueError("dwell_us must not be negative")


def _frequency_axis(values: object) -> NDArray[np.float64]:
    frequency = np.asarray(values, dtype=np.float64)
    if frequency.ndim != 1 or frequency.size < 2:
        raise ValueError("frequency_hz must be a one-dimensional array with at least 2 points")
    if not np.all(np.isfinite(frequency)):
        raise ValueError("frequency_hz contains a non-finite value")
    if np.any(np.diff(frequency) <= 0.0):
        raise ValueError("frequency_hz must be strictly increasing")
    return frequency


def _complex_channel(values: object, name: str, count: int) -> NDArray[np.complex128]:
    channel = np.asarray(values, dtype=np.complex128)
    if channel.ndim != 1 or channel.size != count:
        raise ValueError(f"{name} must contain {count} complex points")
    if not np.all(np.isfinite(channel.real)) or not np.all(np.isfinite(channel.imag)):
        raise ValueError(f"{name} contains a non-finite value")
    return channel


@dataclass
class DualChannelSweep:
    """Validated frequency-domain result of one two-port sweep."""

    frequency_hz: NDArray[np.float64]
    sparams: dict[str, NDArray[np.complex128]]
    settings: SweepSettings | None = None
    captured_at: str = ""

    def __post_init__(self) -> None:
        self.frequency_hz = _frequency_axis(self.frequency_hz)
        missing = [name for name in S_PARAMETERS if name not in self.sparams]
        if missing:
            raise ValueError(f"sparams is missing: {', '.join(missing)}")
        self.sparams = {
            name: _complex_channel(self.sparams[name], name, self.frequency_hz.size)
            for name in S_PARAMETERS
        }
        if not self.captured_at:
            self.captured_at = datetime.now(timezone.utc).astimezone().isoformat(
                timespec="milliseconds"
            )

    @property
    def channel_1(self) -> NDArray[np.complex128]:
        """Port 1 reflection channel, normally labelled HH/S11."""

        return self.sparams["S11"]

    @property
    def channel_2(self) -> NDArray[np.complex128]:
        """Port 2 reflection channel, normally labelled VV/S22."""

        return self.sparams["S22"]

    def copy(self) -> "DualChannelSweep":
        return DualChannelSweep(
            frequency_hz=self.frequency_hz.copy(),
            sparams={name: values.copy() for name, values in self.sparams.items()},
            settings=self.settings,
            captured_at=self.captured_at,
        )

    def save_npz(self, filename: str | Path) -> Path:
        """Save raw complex channels without using pickle."""

        path = Path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload: dict[str, object] = {
            "format": np.array("uav_gpr_dual_channel_sweep_v1"),
            "frequency_hz": self.frequency_hz,
            "captured_at": np.array(self.captured_at),
            "settings": np.array(
                json.dumps(asdict(self.settings), ensure_ascii=False)
                if self.settings is not None
                else "null"
            ),
        }
        for name in S_PARAMETERS:
            payload[f"{name}_real"] = self.sparams[name].real
            payload[f"{name}_imag"] = self.sparams[name].imag
        np.savez_compressed(path, **payload)
        return path

    @classmethod
    def load_npz(cls, filename: str | Path) -> "DualChannelSweep":
        path = Path(filename)
        with np.load(path, allow_pickle=False) as archive:
            file_format = str(archive["format"].item())
            if file_format != "uav_gpr_dual_channel_sweep_v1":
                raise ValueError(f"unsupported sweep format: {file_format}")
            settings_payload = json.loads(str(archive["settings"].item()))
            settings = SweepSettings(**settings_payload) if settings_payload is not None else None
            sparams = {
                name: np.asarray(archive[f"{name}_real"], dtype=np.float64)
                + 1j * np.asarray(archive[f"{name}_imag"], dtype=np.float64)
                for name in S_PARAMETERS
            }
            return cls(
                frequency_hz=np.asarray(archive["frequency_hz"], dtype=np.float64),
                sparams=sparams,
                settings=settings,
                captured_at=str(archive["captured_at"].item()),
            )


def points_to_sweep(
    points: Sequence[RawPoint],
    settings: SweepSettings | None = None,
) -> DualChannelSweep:
    """Convert the LibreVNA backend's point list into validated arrays."""

    if len(points) < 2:
        raise ValueError("the backend returned fewer than 2 frequency points")
    missing_by_point = [
        (index, [name for name in S_PARAMETERS if name not in values])
        for index, (_frequency, values) in enumerate(points)
        if any(name not in values for name in S_PARAMETERS)
    ]
    if missing_by_point:
        index, missing = missing_by_point[0]
        raise ValueError(f"point {index} is missing: {', '.join(missing)}")

    frequency_hz = np.asarray([frequency for frequency, _values in points], dtype=np.float64)
    sparams = {
        name: np.asarray(
            [complex(values[name]) for _frequency, values in points],
            dtype=np.complex128,
        )
        for name in S_PARAMETERS
    }
    return DualChannelSweep(frequency_hz, sparams, settings=settings)


def _production_backend(
    start_hz: int,
    stop_hz: int,
    points: int,
    ifbw_hz: int,
    power_dbm: float,
    dwell_us: int,
) -> Sequence[RawPoint]:
    """Load the project's direct-USB backend only when hardware is requested."""

    backend_directory = Path(__file__).resolve().parents[1] / "librevna"
    backend_text = str(backend_directory)
    if backend_text not in sys.path:
        sys.path.insert(0, backend_text)
    from librevna_usb_s11 import acquire_sparameters

    return acquire_sparameters(
        start_hz,
        stop_hz,
        points,
        ifbw_hz,
        power_dbm,
        dwell_us,
    )


def acquire_dual_channel(
    settings: SweepSettings,
    backend: AcquisitionBackend | None = None,
) -> DualChannelSweep:
    """Acquire S11/S21/S12/S22 using the production or an injected backend."""

    acquire = backend or _production_backend
    points = acquire(
        settings.start_hz,
        settings.stop_hz,
        settings.points,
        settings.ifbw_hz,
        settings.power_dbm,
        settings.dwell_us,
    )
    sweep = points_to_sweep(points, settings=settings)
    if sweep.frequency_hz.size != settings.points:
        raise ValueError(
            f"expected {settings.points} points, received {sweep.frequency_hz.size}"
        )
    return sweep


def main() -> int:
    parser = argparse.ArgumentParser(description="Acquire one two-port LibreVNA sweep")
    parser.add_argument("--start-hz", type=int, default=400_000_000)
    parser.add_argument("--stop-hz", type=int, default=4_000_000_000)
    parser.add_argument("--points", type=int, default=201)
    parser.add_argument("--ifbw-hz", type=int, default=10_000)
    parser.add_argument("--power-dbm", type=float, default=-20.0)
    parser.add_argument("--dwell-us", type=int, default=0)
    parser.add_argument("--out", type=Path, default=Path("dual_channel_raw.npz"))
    args = parser.parse_args()

    settings = SweepSettings(
        start_hz=args.start_hz,
        stop_hz=args.stop_hz,
        points=args.points,
        ifbw_hz=args.ifbw_hz,
        power_dbm=args.power_dbm,
        dwell_us=args.dwell_us,
    )
    sweep = acquire_dual_channel(settings)
    output = sweep.save_npz(args.out)
    print(f"Saved {sweep.frequency_hz.size} points to {output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
