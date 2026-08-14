from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import yaml

from f1tenth_raceline.sectors import (
    RacelineData,
    default_sector_profile,
    export_sector_files,
    legacy_sector_ranges,
    normalize_sector_profile,
    overtaking_yaml,
    pixel_to_world,
    speed_scaling_yaml,
    split_indices_from_s,
    validate_sector_profile,
    world_to_pixel,
)


def raceline(step: float = 1.0, end: float = 10.0) -> RacelineData:
    s = np.arange(0.0, end + 1e-9, step)
    return RacelineData(s_m=s, x_m=s.copy(), y_m=np.zeros_like(s))


def test_legacy_ranges_match_original_sector_slicer_convention() -> None:
    assert legacy_sector_ranges(100, [20, 50]) == [(0, 20), (21, 50), (51, 100)]


def test_s_based_split_is_not_tied_to_waypoint_index() -> None:
    coarse = raceline(step=1.0, end=10.0)
    fine = raceline(step=0.5, end=10.0)
    raw = default_sector_profile()
    raw["speed"]["splits_s_m"] = [4.4]
    raw["speed"]["sectors"] = [
        {"scaling": 0.8, "only_FTG": False, "no_FTG": False},
        {"scaling": 0.6, "only_FTG": False, "no_FTG": False},
    ]

    c = normalize_sector_profile(raw, coarse)
    f = normalize_sector_profile(raw, fine)
    c_idx = split_indices_from_s(coarse, c["speed"]["splits_s_m"])[0]
    f_idx = split_indices_from_s(fine, f["speed"]["splits_s_m"])[0]

    assert c_idx != f_idx
    assert abs(c["speed"]["splits_s_m"][0] - 4.4) <= 0.5
    assert abs(f["speed"]["splits_s_m"][0] - 4.4) <= 0.25


def test_speed_yaml_preserves_legacy_ros_schema() -> None:
    r = raceline(step=1.0, end=9.0)  # 10 waypoints
    p = default_sector_profile()
    p["speed"]["global_limit"] = 0.9
    p["speed"]["splits_s_m"] = [2.0, 5.0]
    p["speed"]["sectors"] = [
        {"scaling": 0.8, "only_FTG": False, "no_FTG": True},
        {"scaling": 0.6, "only_FTG": True, "no_FTG": False},
        {"scaling": 0.9, "only_FTG": False, "no_FTG": False},
    ]

    data = speed_scaling_yaml(p, r)["sector_tuner"]["ros__parameters"]
    assert data["global_limit"] == 0.9
    assert data["n_sectors"] == 3
    assert data["Sector0"] == {
        "start": 0,
        "end": 2,
        "scaling": 0.8,
        "only_FTG": False,
        "no_FTG": True,
    }
    assert data["Sector1"]["start"] == 3
    assert data["Sector1"]["end"] == 5
    assert data["Sector2"]["start"] == 6
    assert data["Sector2"]["end"] == 10  # legacy len(wpnts) sentinel


def test_overtaking_yaml_partitions_full_track_with_explicit_flags() -> None:
    r = raceline(step=1.0, end=9.0)
    p = default_sector_profile()
    p["overtaking"]["splits_s_m"] = [3.0, 7.0]
    p["overtaking"]["sectors"] = [
        {"ot_flag": False},
        {"ot_flag": True},
        {"ot_flag": False},
    ]

    data = overtaking_yaml(p, r)["ot_interpolator"]["ros__parameters"]
    assert data["n_sectors"] == 3
    assert data["Overtaking_sector0"] == {"start": 0, "end": 3, "ot_flag": False}
    assert data["Overtaking_sector1"] == {"start": 4, "end": 7, "ot_flag": True}
    assert data["Overtaking_sector2"] == {"start": 8, "end": 10, "ot_flag": False}


def test_validation_detects_known_runtime_hazards() -> None:
    r = raceline(step=0.1, end=5.0)  # 51 points
    p = default_sector_profile()
    p["speed"]["global_limit"] = 0.5
    p["speed"]["splits_s_m"] = [1.0]
    p["speed"]["sectors"] = [
        {"scaling": 0.8, "only_FTG": True, "no_FTG": True},
        {"scaling": 0.4, "only_FTG": False, "no_FTG": False},
    ]
    p["overtaking"]["splits_s_m"] = [2.0]
    p["overtaking"]["spline_len"] = 30
    p["overtaking"]["sectors"] = [{"ot_flag": True}, {"ot_flag": False}]

    codes = {w["code"] for w in validate_sector_profile(p, r)}
    assert "ftg_conflict" in codes
    assert "scaling_above_global_limit" in codes
    assert "speed_sector_short" in codes
    assert "ot_sector_short" in codes


def test_world_pixel_transform_round_trip() -> None:
    x = np.array([-1.0, 0.0, 2.5])
    y = np.array([3.0, 4.0, 5.5])
    px, py = world_to_pixel(
        x,
        y,
        resolution=0.05,
        origin_x=-2.0,
        origin_y=1.0,
        image_height=200,
    )
    rx, ry = pixel_to_world(
        px,
        py,
        resolution=0.05,
        origin_x=-2.0,
        origin_y=1.0,
        image_height=200,
    )
    np.testing.assert_allclose(rx, x)
    np.testing.assert_allclose(ry, y)


def test_export_writes_profile_and_both_ros_yaml_files(tmp_path: Path) -> None:
    r = raceline(step=1.0, end=20.0)
    p = default_sector_profile()
    profile_path = tmp_path / "edit" / "sectors.json"

    result = export_sector_files(
        map_dir=tmp_path,
        profile_path=profile_path,
        raceline=r,
        profile=p,
    )

    assert result.profile_path.is_file()
    assert result.speed_yaml_path.is_file()
    assert result.ot_yaml_path.is_file()
    assert json.loads(profile_path.read_text())["version"] == 1
    speed = yaml.safe_load(result.speed_yaml_path.read_text())
    ot = yaml.safe_load(result.ot_yaml_path.read_text())
    assert "sector_tuner" in speed
    assert "ot_interpolator" in ot


def test_unsorted_splits_are_rejected_instead_of_reordering_settings() -> None:
    import pytest
    r = raceline(step=1.0, end=10.0)
    p = default_sector_profile()
    p["speed"]["splits_s_m"] = [7.0, 3.0]
    p["speed"]["sectors"] = [
        {"scaling": 0.2, "only_FTG": False, "no_FTG": False},
        {"scaling": 0.4, "only_FTG": False, "no_FTG": False},
        {"scaling": 0.6, "only_FTG": False, "no_FTG": False},
    ]
    with pytest.raises(ValueError, match="strictly increasing"):
        normalize_sector_profile(p, r)


def test_splits_that_collapse_to_same_waypoint_are_rejected() -> None:
    import pytest
    r = raceline(step=1.0, end=10.0)
    p = default_sector_profile()
    p["speed"]["splits_s_m"] = [4.1, 4.2]
    p["speed"]["sectors"] = [
        {"scaling": 0.2, "only_FTG": False, "no_FTG": False},
        {"scaling": 0.4, "only_FTG": False, "no_FTG": False},
        {"scaling": 0.6, "only_FTG": False, "no_FTG": False},
    ]
    with pytest.raises(ValueError, match="collapse to the same waypoint"):
        normalize_sector_profile(p, r)


def test_world_to_pixel_matches_planner_vertical_flip_convention() -> None:
    h = 100
    res = 0.1
    ox, oy = -2.0, 3.0
    flipped_px_x = np.array([10.0, 25.0])
    flipped_px_y = np.array([20.0, 80.0])
    world_x = flipped_px_x * res + ox
    world_y = flipped_px_y * res + oy
    display_x, display_y = world_to_pixel(
        world_x, world_y, resolution=res, origin_x=ox, origin_y=oy, image_height=h
    )
    np.testing.assert_allclose(display_x, flipped_px_x)
    np.testing.assert_allclose(display_y, (h - 1) - flipped_px_y)
