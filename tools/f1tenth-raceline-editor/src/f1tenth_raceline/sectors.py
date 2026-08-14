from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml


SECTOR_PROFILE_VERSION = 1
SPEED_TRANSITION_HALF_WINDOW = 10
DEFAULT_GLOBAL_LIMIT = 0.5
DEFAULT_SPEED_SCALING = 0.5
DEFAULT_YEET_FACTOR = 1.25
DEFAULT_SPLINE_LEN = 30
DEFAULT_OT_SECTOR_BEGIN = 0.5


@dataclass(frozen=True)
class RacelineData:
    s_m: np.ndarray
    x_m: np.ndarray
    y_m: np.ndarray

    @property
    def n_points(self) -> int:
        return int(self.s_m.size)

    @property
    def s_max(self) -> float:
        return float(self.s_m[-1])


@dataclass(frozen=True)
class ExportedSectorFiles:
    profile_path: Path
    speed_yaml_path: Path
    ot_yaml_path: Path
    warnings: tuple[dict, ...]


def load_raceline_csv(path: Path) -> RacelineData:
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Raceline CSV not found: {path}")

    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        required = {"s_m", "x_m", "y_m"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(
                f"Raceline CSV must contain {sorted(required)}; got {reader.fieldnames}."
            )
        rows = list(reader)

    if len(rows) < 2:
        raise ValueError("Raceline must contain at least two waypoints.")

    s = np.asarray([float(r["s_m"]) for r in rows], dtype=float)
    x = np.asarray([float(r["x_m"]) for r in rows], dtype=float)
    y = np.asarray([float(r["y_m"]) for r in rows], dtype=float)

    if not (np.all(np.isfinite(s)) and np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
        raise ValueError("Raceline contains non-finite values.")
    if np.any(np.diff(s) < -1e-9):
        raise ValueError("Raceline s_m must be monotonically non-decreasing.")
    if s[-1] <= s[0]:
        raise ValueError("Raceline must have positive longitudinal extent.")

    # The optimizer normally starts s at zero. Normalizing an offset here makes
    # physical sector positions independent from a CSV writer that retained a
    # constant s offset.
    s = s - s[0]
    return RacelineData(s_m=s, x_m=x, y_m=y)


def default_sector_profile() -> dict:
    return {
        "version": SECTOR_PROFILE_VERSION,
        "speed": {
            "global_limit": DEFAULT_GLOBAL_LIMIT,
            "splits_s_m": [],
            "sectors": [
                {
                    "scaling": DEFAULT_SPEED_SCALING,
                    "only_FTG": False,
                    "no_FTG": False,
                }
            ],
        },
        "overtaking": {
            "yeet_factor": DEFAULT_YEET_FACTOR,
            "spline_len": DEFAULT_SPLINE_LEN,
            "ot_sector_begin": DEFAULT_OT_SECTOR_BEGIN,
            "splits_s_m": [],
            "sectors": [{"ot_flag": False}],
        },
    }


def _finite_float(value: object, *, name: str) -> float:
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite.")
    return out


def nearest_waypoint_index_for_s(raceline: RacelineData, s_m: float) -> int:
    s = float(s_m)
    pos = int(np.searchsorted(raceline.s_m, s, side="left"))
    if pos <= 0:
        return 0
    if pos >= raceline.n_points:
        return raceline.n_points - 1
    before = pos - 1
    return pos if abs(raceline.s_m[pos] - s) < abs(s - raceline.s_m[before]) else before


def _normalize_splits(values: object, raceline: RacelineData) -> list[float]:
    if values is None:
        return []
    if not isinstance(values, list):
        raise ValueError("splits_s_m must be a list.")

    raw_values = [
        _finite_float(raw, name=f"splits_s_m[{i}]")
        for i, raw in enumerate(values)
    ]
    if any(b <= a for a, b in zip(raw_values, raw_values[1:])):
        raise ValueError("splits_s_m must be strictly increasing.")

    indices: list[int] = []
    for value in raw_values:
        idx = nearest_waypoint_index_for_s(raceline, value)
        if not 0 < idx < raceline.n_points - 1:
            raise ValueError("Sector splits must map to interior waypoints, not start/end.")
        if indices and idx <= indices[-1]:
            raise ValueError(
                "Adjacent sector splits collapse to the same waypoint after resampling; "
                "move them farther apart."
            )
        indices.append(idx)

    return [float(raceline.s_m[i]) for i in indices]


def _resize_settings(settings: object, count: int, default_factory) -> list[dict]:
    source = settings if isinstance(settings, list) else []
    out: list[dict] = []
    for i in range(count):
        if i < len(source) and isinstance(source[i], dict):
            out.append(dict(source[i]))
        elif out:
            out.append(dict(out[-1]))
        else:
            out.append(default_factory())
    return out


def normalize_sector_profile(profile: dict | None, raceline: RacelineData) -> dict:
    if profile is None:
        profile = default_sector_profile()
    if not isinstance(profile, dict):
        raise ValueError("Sector profile must be a JSON object.")

    speed_in = profile.get("speed") if isinstance(profile.get("speed"), dict) else {}
    ot_in = profile.get("overtaking") if isinstance(profile.get("overtaking"), dict) else {}

    global_limit = float(np.clip(
        _finite_float(speed_in.get("global_limit", DEFAULT_GLOBAL_LIMIT), name="global_limit"),
        0.0,
        1.0,
    ))
    speed_splits = _normalize_splits(speed_in.get("splits_s_m", []), raceline)
    speed_count = len(speed_splits) + 1
    speed_settings = _resize_settings(
        speed_in.get("sectors"),
        speed_count,
        lambda: {"scaling": DEFAULT_SPEED_SCALING, "only_FTG": False, "no_FTG": False},
    )
    normalized_speed: list[dict] = []
    for i, sector in enumerate(speed_settings):
        scaling = float(np.clip(
            _finite_float(sector.get("scaling", DEFAULT_SPEED_SCALING), name=f"speed.sectors[{i}].scaling"),
            0.0,
            1.0,
        ))
        normalized_speed.append(
            {
                "scaling": scaling,
                "only_FTG": bool(sector.get("only_FTG", False)),
                "no_FTG": bool(sector.get("no_FTG", False)),
            }
        )

    yeet_factor = _finite_float(ot_in.get("yeet_factor", DEFAULT_YEET_FACTOR), name="yeet_factor")
    if yeet_factor <= 0.0:
        raise ValueError("yeet_factor must be > 0.")
    spline_len = int(ot_in.get("spline_len", DEFAULT_SPLINE_LEN))
    if spline_len < 1:
        raise ValueError("spline_len must be >= 1 waypoint.")
    ot_sector_begin = float(np.clip(
        _finite_float(ot_in.get("ot_sector_begin", DEFAULT_OT_SECTOR_BEGIN), name="ot_sector_begin"),
        0.0,
        1.0,
    ))
    ot_splits = _normalize_splits(ot_in.get("splits_s_m", []), raceline)
    ot_count = len(ot_splits) + 1
    ot_settings = _resize_settings(
        ot_in.get("sectors"),
        ot_count,
        lambda: {"ot_flag": False},
    )
    normalized_ot = [{"ot_flag": bool(v.get("ot_flag", False))} for v in ot_settings]

    return {
        "version": SECTOR_PROFILE_VERSION,
        "speed": {
            "global_limit": global_limit,
            "splits_s_m": speed_splits,
            "sectors": normalized_speed,
        },
        "overtaking": {
            "yeet_factor": float(yeet_factor),
            "spline_len": spline_len,
            "ot_sector_begin": ot_sector_begin,
            "splits_s_m": ot_splits,
            "sectors": normalized_ot,
        },
    }


def split_indices_from_s(raceline: RacelineData, splits_s_m: Iterable[float]) -> list[int]:
    indices = sorted({nearest_waypoint_index_for_s(raceline, float(v)) for v in splits_s_m})
    return [i for i in indices if 0 < i < raceline.n_points - 1]


def legacy_sector_ranges(n_points: int, split_indices: Iterable[int]) -> list[tuple[int, int]]:
    if n_points < 2:
        raise ValueError("n_points must be >= 2.")
    splits = sorted({int(v) for v in split_indices})
    if any(v <= 0 or v >= n_points - 1 for v in splits):
        raise ValueError("split indices must be strictly inside the waypoint range.")

    ranges: list[tuple[int, int]] = []
    start = 0
    for split in splits:
        if split < start:
            raise ValueError("split indices must form non-empty ordered sectors.")
        ranges.append((start, split))
        start = split + 1
    # Keep the original sector_slicer convention: the final end is len(wpnts),
    # used as a sentinel by the ROS runtime code.
    ranges.append((start, n_points))
    return ranges


def speed_scaling_yaml(profile: dict, raceline: RacelineData) -> dict:
    p = normalize_sector_profile(profile, raceline)
    speed = p["speed"]
    ranges = legacy_sector_ranges(
        raceline.n_points,
        split_indices_from_s(raceline, speed["splits_s_m"]),
    )
    params: dict = {
        "global_limit": float(speed["global_limit"]),
        "n_sectors": len(ranges),
    }
    for i, ((start, end), settings) in enumerate(zip(ranges, speed["sectors"])):
        params[f"Sector{i}"] = {
            "start": int(start),
            "end": int(end),
            "scaling": float(settings["scaling"]),
            "only_FTG": bool(settings["only_FTG"]),
            "no_FTG": bool(settings["no_FTG"]),
        }
    return {"sector_tuner": {"ros__parameters": params}}


def overtaking_yaml(profile: dict, raceline: RacelineData) -> dict:
    p = normalize_sector_profile(profile, raceline)
    ot = p["overtaking"]
    ranges = legacy_sector_ranges(
        raceline.n_points,
        split_indices_from_s(raceline, ot["splits_s_m"]),
    )
    params: dict = {
        "n_sectors": len(ranges),
        "yeet_factor": float(ot["yeet_factor"]),
        "spline_len": int(ot["spline_len"]),
        "ot_sector_begin": float(ot["ot_sector_begin"]),
    }
    for i, ((start, end), settings) in enumerate(zip(ranges, ot["sectors"])):
        params[f"Overtaking_sector{i}"] = {
            "start": int(start),
            "end": int(end),
            "ot_flag": bool(settings["ot_flag"]),
        }
    return {"ot_interpolator": {"ros__parameters": params}}


def _actual_range_point_count(start: int, end: int, n_points: int) -> int:
    inclusive_end = min(end, n_points - 1)
    return max(0, inclusive_end - start + 1)


def validate_sector_profile(profile: dict, raceline: RacelineData) -> list[dict]:
    p = normalize_sector_profile(profile, raceline)
    warnings: list[dict] = []

    speed = p["speed"]
    speed_ranges = legacy_sector_ranges(
        raceline.n_points,
        split_indices_from_s(raceline, speed["splits_s_m"]),
    )
    for i, ((start, end), settings) in enumerate(zip(speed_ranges, speed["sectors"])):
        count = _actual_range_point_count(start, end, raceline.n_points)
        if count <= 2 * SPEED_TRANSITION_HALF_WINDOW:
            warnings.append(
                {
                    "severity": "warning",
                    "code": "speed_sector_short",
                    "sector": i,
                    "message": (
                        f"Speed Sector{i} has {count} waypoint(s); the legacy runtime uses "
                        f"±{SPEED_TRANSITION_HALF_WINDOW} waypoint blending, so transitions may overlap."
                    ),
                }
            )
        if settings["only_FTG"] and settings["no_FTG"]:
            warnings.append(
                {
                    "severity": "error",
                    "code": "ftg_conflict",
                    "sector": i,
                    "message": f"Speed Sector{i} cannot set only_FTG and no_FTG at the same time.",
                }
            )
        if settings["scaling"] > speed["global_limit"] + 1e-12:
            warnings.append(
                {
                    "severity": "warning",
                    "code": "scaling_above_global_limit",
                    "sector": i,
                    "message": (
                        f"Speed Sector{i} scaling={settings['scaling']:.3f} exceeds "
                        f"global_limit={speed['global_limit']:.3f}; the ROS runtime clips it."
                    ),
                }
            )

    ot = p["overtaking"]
    ot_ranges = legacy_sector_ranges(
        raceline.n_points,
        split_indices_from_s(raceline, ot["splits_s_m"]),
    )
    for i, ((start, end), settings) in enumerate(zip(ot_ranges, ot["sectors"])):
        if not settings["ot_flag"]:
            continue
        count = _actual_range_point_count(start, end, raceline.n_points)
        if count <= 2 * ot["spline_len"]:
            warnings.append(
                {
                    "severity": "warning",
                    "code": "ot_sector_short",
                    "sector": i,
                    "message": (
                        f"Overtaking_sector{i} has {count} waypoint(s), shorter than twice "
                        f"spline_len={ot['spline_len']}; entry/exit interpolation can overlap."
                    ),
                }
            )

    return warnings


def load_sector_profile(path: Path, raceline: RacelineData) -> dict:
    if not path.is_file():
        return normalize_sector_profile(default_sector_profile(), raceline)
    raw = json.loads(path.read_text(encoding="utf-8"))
    return normalize_sector_profile(raw, raceline)


def save_sector_profile(path: Path, profile: dict, raceline: RacelineData) -> dict:
    normalized = normalize_sector_profile(profile, raceline)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(normalized, indent=2), encoding="utf-8")
    return normalized


def export_sector_files(
    *,
    map_dir: Path,
    profile_path: Path,
    raceline: RacelineData,
    profile: dict,
) -> ExportedSectorFiles:
    normalized = normalize_sector_profile(profile, raceline)
    warnings = validate_sector_profile(normalized, raceline)
    errors = [w for w in warnings if w["severity"] == "error"]
    if errors:
        raise ValueError("; ".join(e["message"] for e in errors))

    save_sector_profile(profile_path, normalized, raceline)
    map_dir.mkdir(parents=True, exist_ok=True)
    speed_path = map_dir / "speed_scaling.yaml"
    ot_path = map_dir / "ot_sectors.yaml"
    speed_path.write_text(
        yaml.safe_dump(speed_scaling_yaml(normalized, raceline), sort_keys=False),
        encoding="utf-8",
    )
    ot_path.write_text(
        yaml.safe_dump(overtaking_yaml(normalized, raceline), sort_keys=False),
        encoding="utf-8",
    )
    return ExportedSectorFiles(
        profile_path=profile_path,
        speed_yaml_path=speed_path,
        ot_yaml_path=ot_path,
        warnings=tuple(warnings),
    )


def world_to_pixel(
    x_m: np.ndarray | float,
    y_m: np.ndarray | float,
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
    image_height: int,
) -> tuple[np.ndarray, np.ndarray]:
    if resolution <= 0.0:
        raise ValueError("resolution must be > 0.")
    x = np.asarray(x_m, dtype=float)
    y = np.asarray(y_m, dtype=float)
    px = (x - origin_x) / resolution
    py_flipped = (y - origin_y) / resolution
    py = (image_height - 1) - py_flipped
    return px, py


def pixel_to_world(
    px: np.ndarray | float,
    py: np.ndarray | float,
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
    image_height: int,
) -> tuple[np.ndarray, np.ndarray]:
    if resolution <= 0.0:
        raise ValueError("resolution must be > 0.")
    x_px = np.asarray(px, dtype=float)
    y_px = np.asarray(py, dtype=float)
    y_flipped = (image_height - 1) - y_px
    x = x_px * resolution + origin_x
    y = y_flipped * resolution + origin_y
    return x, y
