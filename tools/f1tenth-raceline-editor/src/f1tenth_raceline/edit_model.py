from __future__ import annotations

import json
import tempfile
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import cv2
import numpy as np
import yaml

PROFILE_VERSION = 1
VALID_TOOLS = {"free", "occupied"}


@dataclass(frozen=True)
class MaterializedMap:
    yaml_path: Path
    image_path: Path


def load_base_image(map_yaml: Path) -> tuple[np.ndarray, dict, Path]:
    map_yaml = map_yaml.resolve()
    if not map_yaml.is_file():
        raise FileNotFoundError(f"Map YAML not found: {map_yaml}")
    with map_yaml.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    image_value = data.get("image")
    image_path = (map_yaml.parent / image_value).resolve() if image_value else map_yaml.with_suffix(".png").resolve()
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Could not decode map image: {image_path}")
    return image, data, image_path


def empty_profile(map_yaml: Path, image: np.ndarray) -> dict:
    h, w = image.shape[:2]
    return {"version": PROFILE_VERSION, "base_map": map_yaml.name, "image_width": int(w), "image_height": int(h), "operations": []}


def normalize_profile(profile: dict, map_yaml: Path, image: np.ndarray) -> dict:
    if not isinstance(profile, dict):
        raise ValueError("Edit profile must be a JSON object.")
    h, w = image.shape[:2]
    operations = profile.get("operations", [])
    if not isinstance(operations, list):
        raise ValueError("profile.operations must be a list.")
    normalized_ops = []
    for i, op in enumerate(operations):
        if not isinstance(op, dict):
            raise ValueError(f"operations[{i}] must be an object.")
        tool = op.get("tool")
        if tool not in VALID_TOOLS:
            raise ValueError(f"operations[{i}].tool must be one of {sorted(VALID_TOOLS)}.")
        radius = int(op.get("radius", 1))
        if radius < 1 or radius > 500:
            raise ValueError(f"operations[{i}].radius is out of range.")
        points = op.get("points", [])
        if not isinstance(points, list) or not points:
            continue
        norm_points = []
        for j, point in enumerate(points):
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError(f"operations[{i}].points[{j}] must be [x, y].")
            x = max(0, min(w - 1, int(round(float(point[0])))))
            y = max(0, min(h - 1, int(round(float(point[1])))))
            norm_points.append([x, y])
        normalized_ops.append({"tool": tool, "radius": radius, "points": norm_points})
    return {"version": PROFILE_VERSION, "base_map": map_yaml.name, "image_width": int(w), "image_height": int(h), "operations": normalized_ops}


def load_profile(profile_path: Path, map_yaml: Path, image: np.ndarray) -> dict:
    if not profile_path.is_file():
        return empty_profile(map_yaml, image)
    return normalize_profile(json.loads(profile_path.read_text(encoding="utf-8")), map_yaml, image)


def save_profile(profile_path: Path, profile: dict, map_yaml: Path, image: np.ndarray) -> dict:
    normalized = normalize_profile(profile, map_yaml, image)
    profile_path.parent.mkdir(parents=True, exist_ok=True)
    profile_path.write_text(json.dumps(normalized, indent=2), encoding="utf-8")
    return normalized


def _draw_stroke(mask: np.ndarray, points: list[list[int]], radius: int) -> None:
    thickness = max(1, radius * 2)
    if len(points) == 1:
        cv2.circle(mask, tuple(points[0]), radius, 255, thickness=-1, lineType=cv2.LINE_AA)
        return
    for p0, p1 in zip(points[:-1], points[1:]):
        cv2.line(mask, tuple(p0), tuple(p1), 255, thickness=thickness, lineType=cv2.LINE_AA)
    cv2.circle(mask, tuple(points[0]), radius, 255, thickness=-1, lineType=cv2.LINE_AA)
    cv2.circle(mask, tuple(points[-1]), radius, 255, thickness=-1, lineType=cv2.LINE_AA)


def apply_profile(base_image: np.ndarray, profile: dict) -> np.ndarray:
    edited = base_image.copy()
    for op in profile.get("operations", []):
        points = op["points"]
        if not points:
            continue
        mask = np.zeros(base_image.shape[:2], dtype=np.uint8)
        _draw_stroke(mask, points, int(op["radius"]))
        edited[mask > 0] = 255 if op["tool"] == "free" else 0
    return edited


@contextmanager
def materialize_profile(map_yaml: Path, profile_path: Path) -> Iterator[MaterializedMap]:
    map_yaml = map_yaml.resolve()
    image, data, _ = load_base_image(map_yaml)
    profile = load_profile(profile_path.resolve(), map_yaml, image)
    edited = apply_profile(image, profile)
    with tempfile.TemporaryDirectory(prefix="f1tenth-raceline-edit-") as tmp:
        tmp_dir = Path(tmp)
        image_path = tmp_dir / "map_edited.png"
        yaml_path = tmp_dir / "map_edited.yaml"
        if not cv2.imwrite(str(image_path), edited):
            raise RuntimeError(f"Could not write temporary edited image: {image_path}")
        derived = dict(data)
        derived["image"] = image_path.name
        yaml_path.write_text(yaml.safe_dump(derived, sort_keys=False), encoding="utf-8")
        yield MaterializedMap(yaml_path=yaml_path, image_path=image_path)
