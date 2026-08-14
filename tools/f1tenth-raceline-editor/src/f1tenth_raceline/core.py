
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml
from scipy.signal import savgol_filter
from skimage.morphology import skeletonize
from skimage.segmentation import watershed

from .vendor import activate_optimizer_imports, default_config_dir, ensure_vendor


@dataclass(frozen=True)
class MapMeta:
    resolution: float
    origin_x: float
    origin_y: float


@dataclass
class RacelineResult:
    centerline: np.ndarray
    centerline_with_width: np.ndarray
    bound_right: np.ndarray
    bound_left: np.ndarray
    raceline_iqp: np.ndarray
    raceline_shortest: np.ndarray
    ltpl: np.ndarray
    est_lap_time_iqp: float
    est_lap_time_shortest: float


def _optimizer_modules():
    checkout = ensure_vendor()
    activate_optimizer_imports(checkout)

    import trajectory_planning_helpers as tph
    from global_racetrajectory_optimization import helper_funcs_glob
    from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer

    return tph, helper_funcs_glob, trajectory_optimizer


def load_map_meta(yaml_path: Path) -> MapMeta:
    with yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    return MapMeta(
        resolution=float(data["resolution"]),
        origin_x=float(data["origin"][0]),
        origin_y=float(data["origin"][1]),
    )


def map_image_path(yaml_path: Path) -> Path:
    with yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    image = data.get("image")
    if image:
        return (yaml_path.parent / image).resolve()
    return yaml_path.with_suffix(".png").resolve()


def extract_centerline(
    skeleton: np.ndarray,
    cent_length: float,
    map_resolution: float,
    map_editor_mode: bool = True,
) -> np.ndarray:
    contours, hierarchy = cv2.findContours(
        skeleton.astype(np.uint8),
        cv2.RETR_CCOMP,
        cv2.CHAIN_APPROX_NONE,
    )
    if hierarchy is None:
        raise RuntimeError("No contours found in skeleton.")

    closed_contours = []
    for i, elem in enumerate(contours):
        opened = hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0
        if not opened:
            closed_contours.append(elem)

    if not closed_contours:
        raise RuntimeError("No closed contours found in skeleton.")

    line_lengths = [np.inf] * len(closed_contours)
    for i, cont in enumerate(closed_contours):
        pts = cont[:, 0, :]
        shifted = np.roll(pts, 1, axis=0)
        line_length = np.linalg.norm(pts - shifted, axis=1).sum() * map_resolution

        if cent_length > 0.0 and abs(cent_length / line_length - 1.0) < 0.15:
            line_lengths[i] = line_length
        elif map_editor_mode or cent_length == 0.0:
            line_lengths[i] = line_length

    min_line_length = min(line_lengths)
    if not np.isfinite(min_line_length):
        raise RuntimeError(
            "No valid closed contour matched the expected centerline length."
        )

    idx = line_lengths.index(min_line_length)
    return closed_contours[idx][:, 0, :].astype(float)


def smooth_centerline(centerline: np.ndarray) -> np.ndarray:
    n = len(centerline)
    if n < 7:
        return centerline.copy()

    if n > 2000:
        filter_length = int(n / 200) * 10 + 1
    elif n > 1000:
        filter_length = 81
    elif n > 500:
        filter_length = 41
    else:
        filter_length = 21

    filter_length = min(filter_length, n - 1 if n % 2 == 0 else n)
    if filter_length % 2 == 0:
        filter_length -= 1
    filter_length = max(filter_length, 5)

    smoothed = savgol_filter(centerline, filter_length, 3, axis=0)

    half = n // 2
    shifted = np.append(centerline[half:], centerline[:half], axis=0)
    shifted_smoothed = savgol_filter(shifted, filter_length, 3, axis=0)

    edge = min(filter_length, half)
    smoothed[:edge] = shifted_smoothed[half:half + edge]
    smoothed[-edge:] = shifted_smoothed[half - edge:half]
    return smoothed


def compare_direction(alpha: float, beta: float) -> bool:
    delta = abs(alpha - beta)
    if delta > np.pi:
        delta = 2 * np.pi - delta
    return delta < np.pi / 2


def centerline_to_meter(
    centerline_smooth: np.ndarray,
    meta: MapMeta,
    helper_funcs_glob,
    step: float = 0.1,
) -> np.ndarray:
    meter = np.zeros_like(centerline_smooth, dtype=float)
    meter[:, 0] = centerline_smooth[:, 0] * meta.resolution + meta.origin_x
    meter[:, 1] = centerline_smooth[:, 1] * meta.resolution + meta.origin_y

    reftrack = np.column_stack((meter, np.zeros((meter.shape[0], 2))))
    return helper_funcs_glob.src.interp_track.interp_track(
        reftrack=reftrack,
        stepsize_approx=step,
    )[:, :2]


def orient_centerline(
    centerline_px: np.ndarray,
    centerline_m: np.ndarray,
    initial_position: Optional[tuple[float, float, float]],
    reverse: bool,
) -> tuple[np.ndarray, np.ndarray]:
    px = centerline_px
    meter = centerline_m

    if initial_position is not None:
        x0, y0, yaw0 = initial_position
        distances = np.hypot(meter[:, 0] - x0, meter[:, 1] - y0)
        i = int(np.argmin(distances))
        prev = (i - 1) % len(meter)
        dx = meter[i, 0] - meter[prev, 0]
        dy = meter[i, 1] - meter[prev, 1]
        direction = np.angle(complex(dx, dy))

        if not compare_direction(direction, yaw0):
            px = np.flip(px, axis=0)
            meter = np.flip(meter, axis=0)

    if reverse:
        px = np.flip(px, axis=0)
        meter = np.flip(meter, axis=0)

    return px, meter


def extract_track_bounds(
    centerline_px: np.ndarray,
    filtered_bw: np.ndarray,
    meta: MapMeta,
) -> tuple[np.ndarray, np.ndarray]:
    cent_img = np.zeros_like(filtered_bw, dtype=np.uint8)
    cv2.drawContours(
        cent_img,
        [centerline_px.astype(int)],
        0,
        255,
        2,
        cv2.LINE_8,
    )
    _, cent_markers = cv2.connectedComponents(cent_img)

    dist_transform = cv2.distanceTransform(filtered_bw, cv2.DIST_L2, 5)
    labels = watershed(-dist_transform, cent_markers, mask=filtered_bw)

    closed_contours = []
    for label in np.unique(labels):
        if label == 0:
            continue

        mask = np.zeros_like(filtered_bw, dtype=np.uint8)
        mask[labels == label] = 255
        contours, hierarchy = cv2.findContours(
            mask,
            cv2.RETR_CCOMP,
            cv2.CHAIN_APPROX_NONE,
        )
        if hierarchy is None:
            continue

        for i, cont in enumerate(contours):
            opened = hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0
            if not opened:
                closed_contours.append(cont)

    if len(closed_contours) != 2:
        raise RuntimeError(
            f"Watershed expected exactly 2 track bounds, found {len(closed_contours)}."
        )

    bound_long = max(closed_contours, key=len)[:, 0, :].astype(float)
    bound_short = min(closed_contours, key=len)[:, 0, :].astype(float)

    def to_meter(points: np.ndarray) -> np.ndarray:
        out = np.zeros_like(points, dtype=float)
        out[:, 0] = points[:, 0] * meta.resolution + meta.origin_x
        out[:, 1] = points[:, 1] * meta.resolution + meta.origin_y
        return out

    # This intentionally preserves the source stack's current semantics.
    return to_meter(bound_long), to_meter(bound_short)


def distances_to_bounds(
    trajectory: np.ndarray,
    bound_r: np.ndarray,
    bound_l: np.ndarray,
    helper_funcs_glob,
    reverse: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    pts = trajectory[:, 1:3] if trajectory.shape[1] > 2 else trajectory[:, :2]

    def interp_bound(bound: np.ndarray) -> np.ndarray:
        tmp = np.column_stack((bound, np.zeros((bound.shape[0], 2))))
        return helper_funcs_glob.src.interp_track.interp_track(
            reftrack=tmp,
            stepsize_approx=0.1,
        )[:, :2]

    br = interp_bound(bound_r)
    bl = interp_bound(bound_l)

    d_r = np.array(
        [np.hypot(br[:, 0] - p[0], br[:, 1] - p[1]).min() for p in pts]
    )
    d_l = np.array(
        [np.hypot(bl[:, 0] - p[0], bl[:, 1] - p[1]).min() for p in pts]
    )

    return (d_l, d_r) if reverse else (d_r, d_l)


def centerline_with_width(
    centerline_px: np.ndarray,
    centerline_m: np.ndarray,
    meta: MapMeta,
    filtered_map: np.ndarray,
    bound_r: Optional[np.ndarray],
    bound_l: Optional[np.ndarray],
    helper_funcs_glob,
    reverse: bool,
) -> np.ndarray:
    if bound_r is not None and bound_l is not None:
        wr, wl = distances_to_bounds(
            centerline_m,
            bound_r,
            bound_l,
            helper_funcs_glob,
            reverse=reverse,
        )
    else:
        dist_transform = cv2.distanceTransform(filtered_map, cv2.DIST_L2, 5)
        width = (
            dist_transform[
                centerline_px[:, 1].astype(int),
                centerline_px[:, 0].astype(int),
            ]
            * meta.resolution
        )

        if len(width) != len(centerline_m):
            width = np.interp(
                np.arange(len(centerline_m)),
                np.arange(len(width)),
                width,
            )

        wr = width
        wl = width

    return np.column_stack((centerline_m[:, 0], centerline_m[:, 1], wr, wl))


def write_track_csv(path: Path, track: np.ndarray) -> None:
    np.savetxt(
        path,
        track,
        delimiter=",",
        fmt="%.9f",
    )


def generate_racelines(
    map_yaml: Path,
    *,
    config_dir: Optional[Path] = None,
    safety_width: float = 0.4,
    safety_width_sp: float = 0.35,
    reverse: bool = False,
    initial_position: Optional[tuple[float, float, float]] = None,
    work_dir: Optional[Path] = None,
) -> RacelineResult:
    _, helper_funcs_glob, trajectory_optimizer = _optimizer_modules()

    map_yaml = map_yaml.resolve()
    if not map_yaml.is_file():
        raise FileNotFoundError(f"Map YAML not found: {map_yaml}")

    image_path = map_image_path(map_yaml)
    if not image_path.is_file():
        raise FileNotFoundError(f"Map image not found: {image_path}")

    config_dir = (config_dir or default_config_dir()).resolve()
    work_dir = (work_dir or map_yaml.parent / ".raceline_work").resolve()
    work_dir.mkdir(parents=True, exist_ok=True)

    meta = load_map_meta(map_yaml)

    img = cv2.imread(str(image_path), 0)
    if img is None:
        raise FileNotFoundError(f"Could not decode map image: {image_path}")

    filtered_map = cv2.flip(img, 0)
    skeleton = skeletonize(filtered_map, method="lee")

    centerline_px = extract_centerline(
        skeleton=skeleton,
        cent_length=0.0,
        map_resolution=meta.resolution,
        map_editor_mode=True,
    )
    centerline_px = smooth_centerline(centerline_px)
    centerline_m = centerline_to_meter(
        centerline_px,
        meta,
        helper_funcs_glob,
        step=0.1,
    )
    centerline_px, centerline_m = orient_centerline(
        centerline_px,
        centerline_m,
        initial_position=initial_position,
        reverse=reverse,
    )

    bound_r = bound_l = None
    try:
        bound_r, bound_l = extract_track_bounds(
            centerline_px,
            filtered_map,
            meta,
        )
    except RuntimeError as exc:
        print(f"[WARN] {exc}")
        print("[WARN] Falling back to symmetric distance-transform track widths.")

    cent_w = centerline_with_width(
        centerline_px,
        centerline_m,
        meta,
        filtered_map,
        bound_r,
        bound_l,
        helper_funcs_glob,
        reverse,
    )

    iqp_track = work_dir / "map_centerline"
    sp_track = work_dir / "map_centerline_2"
    write_track_csv(iqp_track.with_suffix(".csv"), cent_w)

    result_iqp = trajectory_optimizer(
        input_path=str(config_dir),
        track_name=str(iqp_track),
        curv_opt_type="mincurv_iqp",
        safety_width=safety_width,
        plot=False,
    )
    if len(result_iqp) != 5:
        raise RuntimeError(
            "Pinned ssupath optimizer did not return the expected 5 values. "
            f"Got {len(result_iqp)}."
        )

    raceline_iqp, br_iqp, bl_iqp, t_iqp, ltpl = result_iqp

    if bound_r is None or bound_l is None:
        bound_r, bound_l = br_iqp, bl_iqp

    result_iqp_ot = trajectory_optimizer(
        input_path=str(config_dir),
        track_name=str(iqp_track),
        curv_opt_type="mincurv_iqp",
        safety_width=safety_width_sp,
        plot=False,
    )
    raceline_iqp_ot = result_iqp_ot[0]

    new_center = centerline_with_width(
        raceline_iqp_ot[:, 1:3],
        raceline_iqp_ot[:, 1:3],
        meta,
        filtered_map,
        bound_r,
        bound_l,
        helper_funcs_glob,
        reverse,
    )
    write_track_csv(sp_track.with_suffix(".csv"), new_center)

    result_sp = trajectory_optimizer(
        input_path=str(config_dir),
        track_name=str(sp_track),
        curv_opt_type="shortest_path",
        safety_width=safety_width_sp,
        plot=False,
    )
    raceline_sp, _, _, t_sp, _ = result_sp

    return RacelineResult(
        centerline=centerline_m,
        centerline_with_width=cent_w,
        bound_right=bound_r,
        bound_left=bound_l,
        raceline_iqp=raceline_iqp,
        raceline_shortest=raceline_sp,
        ltpl=ltpl,
        est_lap_time_iqp=float(t_iqp),
        est_lap_time_shortest=float(t_sp),
    )
