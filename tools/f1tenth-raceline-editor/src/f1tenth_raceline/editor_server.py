from __future__ import annotations

import json
import threading
import webbrowser
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np

from .core import extract_centerline, smooth_centerline
from .edit_model import apply_profile, load_base_image, load_profile, normalize_profile, save_profile
from .sectors import export_sector_files, load_raceline_csv, load_sector_profile, validate_sector_profile, world_to_pixel


@dataclass
class EditorState:
    map_yaml: Path
    profile_path: Path
    sector_profile_path: Path
    raceline_dir: Path
    base_image: np.ndarray
    map_data: dict
    image_path: Path


class SectorDataUnavailable(RuntimeError):
    pass


def _json_bytes(data: object) -> bytes:
    return json.dumps(data).encode("utf-8")


def _preview_centerline(state: EditorState, profile: dict) -> list[list[float]]:
    normalized = normalize_profile(profile, state.map_yaml, state.base_image)
    edited = apply_profile(state.base_image, normalized)
    filtered_map = cv2.flip(edited, 0)
    from skimage.morphology import skeletonize
    skeleton = skeletonize(filtered_map, method="lee")
    resolution = float(state.map_data["resolution"])
    centerline = extract_centerline(skeleton=skeleton, cent_length=0.0, map_resolution=resolution, map_editor_mode=True)
    centerline = smooth_centerline(centerline)
    height = edited.shape[0]
    raw = centerline.copy()
    raw[:, 1] = (height - 1) - raw[:, 1]
    stride = max(1, len(raw) // 2500)
    return [[float(x), float(y)] for x, y in raw[::stride]]


def _world_xy_to_canvas(state: EditorState, x: np.ndarray, y: np.ndarray) -> list[list[float]]:
    origin = state.map_data.get("origin", [0.0, 0.0, 0.0])
    px, py = world_to_pixel(x, y, resolution=float(state.map_data["resolution"]), origin_x=float(origin[0]), origin_y=float(origin[1]), image_height=int(state.base_image.shape[0]))
    return [[float(a), float(b)] for a, b in zip(px, py)]


def _read_xy_csv(path: Path) -> tuple[np.ndarray, np.ndarray] | None:
    if not path.is_file():
        return None
    data = np.genfromtxt(path, delimiter=",", names=True)
    data = np.atleast_1d(data)
    names = set(data.dtype.names or ())
    if not {"x_m", "y_m"}.issubset(names):
        return None
    x = np.asarray(data["x_m"], dtype=float)
    y = np.asarray(data["y_m"], dtype=float)
    if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
        return None
    return x, y


def _sector_payload(state: EditorState) -> dict:
    iqp_path = state.raceline_dir / "raceline_iqp.csv"
    if not iqp_path.is_file():
        raise SectorDataUnavailable(f"{iqp_path} not found. Generate racelines first, then reopen or refresh the editor.")
    raceline = load_raceline_csv(iqp_path)
    profile = load_sector_profile(state.sector_profile_path, raceline)
    warnings = validate_sector_profile(profile, raceline)
    raceline_mtime = iqp_path.stat().st_mtime
    stale_sources = [src for src in (state.image_path, state.profile_path) if src.is_file() and src.stat().st_mtime > raceline_mtime]
    if stale_sources:
        warnings.append({
            "severity": "warning",
            "code": "raceline_stale",
            "message": "Map/image edits are newer than raceline_iqp.csv. Regenerate the raceline before finalizing sectors: " + ", ".join(str(p) for p in stale_sources),
        })
    iqp_xy = _world_xy_to_canvas(state, raceline.x_m, raceline.y_m)
    iqp_points = [[float(raceline.s_m[i]), float(iqp_xy[i][0]), float(iqp_xy[i][1]), int(i)] for i in range(raceline.n_points)]
    shortest_points: list[list[float]] = []
    shortest_path = state.raceline_dir / "raceline_shortest.csv"
    if shortest_path.is_file():
        shortest = load_raceline_csv(shortest_path)
        shortest_xy = _world_xy_to_canvas(state, shortest.x_m, shortest.y_m)
        shortest_points = [[float(shortest.s_m[i]), float(shortest_xy[i][0]), float(shortest_xy[i][1]), int(i)] for i in range(shortest.n_points)]
    bounds: dict[str, list[list[float]]] = {"right": [], "left": []}
    for key, filename in (("right", "bound_right.csv"), ("left", "bound_left.csv")):
        xy = _read_xy_csv(state.raceline_dir / filename)
        if xy is not None:
            bounds[key] = _world_xy_to_canvas(state, xy[0], xy[1])
    return {
        "available": True,
        "raceline_path": str(iqp_path),
        "profile_path": str(state.sector_profile_path),
        "speed_yaml_path": str(state.map_yaml.parent / "speed_scaling.yaml"),
        "ot_yaml_path": str(state.map_yaml.parent / "ot_sectors.yaml"),
        "n_points": raceline.n_points,
        "s_max": raceline.s_max,
        "raceline": iqp_points,
        "shortest": shortest_points,
        "shortest_available": bool(shortest_points),
        "bounds": bounds,
        "profile": profile,
        "warnings": warnings,
    }


def run_editor(map_yaml: Path, profile_path: Path | None = None, sector_profile_path: Path | None = None, raceline_dir: Path | None = None, host: str = "127.0.0.1", port: int = 8765, open_browser: bool = True) -> None:
    map_yaml = map_yaml.resolve()
    base_image, map_data, image_path = load_base_image(map_yaml)
    profile_path = profile_path.resolve() if profile_path else (map_yaml.parent / "edit" / "map_edit.json").resolve()
    sector_profile_path = sector_profile_path.resolve() if sector_profile_path else (map_yaml.parent / "edit" / "sectors.json").resolve()
    raceline_dir = raceline_dir.resolve() if raceline_dir else (map_yaml.parent / "raceline_offline").resolve()
    state = EditorState(map_yaml=map_yaml, profile_path=profile_path, sector_profile_path=sector_profile_path, raceline_dir=raceline_dir, base_image=base_image, map_data=map_data, image_path=image_path)
    web_path = Path(__file__).with_name("web") / "index.html"
    html = web_path.read_bytes()
    editor_js = (web_path.parent / "editor.js").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        server_version = "F1TenthRacelineEditor/1.1"
        def log_message(self, fmt: str, *args) -> None:
            print(f"[editor] {self.address_string()} - {fmt % args}")
        def send_bytes(self, body: bytes, content_type: str, status: int = 200) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        def send_json(self, data: object, status: int = 200) -> None:
            self.send_bytes(_json_bytes(data), "application/json; charset=utf-8", status)
        def read_json(self) -> dict:
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length)
            return json.loads(raw.decode("utf-8")) if raw else {}
        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path in {"/", "/index.html"}:
                self.send_bytes(html, "text/html; charset=utf-8")
                return
            if path == "/editor.js":
                self.send_bytes(editor_js, "application/javascript; charset=utf-8")
                return
            if path == "/map.png":
                ok, encoded = cv2.imencode(".png", state.base_image)
                if not ok:
                    self.send_json({"error": "could not encode map"}, HTTPStatus.INTERNAL_SERVER_ERROR)
                    return
                self.send_bytes(encoded.tobytes(), "image/png")
                return
            if path == "/api/state":
                profile = load_profile(state.profile_path, state.map_yaml, state.base_image)
                h, w = state.base_image.shape[:2]
                self.send_json({
                    "map": str(state.map_yaml), "profile": str(state.profile_path), "sector_profile": str(state.sector_profile_path), "raceline_dir": str(state.raceline_dir),
                    "width": int(w), "height": int(h), "operations": profile["operations"],
                    "generate_command": f'uv run raceline generate --map "{state.map_yaml}" --edit "{state.profile_path}"',
                })
                return
            if path == "/api/sectors":
                try:
                    self.send_json(_sector_payload(state))
                except SectorDataUnavailable as exc:
                    self.send_json({"available": False, "message": str(exc)})
                return
            self.send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
        def do_POST(self) -> None:
            path = urlparse(self.path).path
            try:
                payload = self.read_json()
                if path == "/api/save":
                    saved = save_profile(state.profile_path, payload, state.map_yaml, state.base_image)
                    self.send_json({"ok": True, "operations": len(saved["operations"])})
                    return
                if path == "/api/preview-centerline":
                    self.send_json({"ok": True, "points": _preview_centerline(state, payload)})
                    return
                if path == "/api/sectors/validate":
                    raceline = load_raceline_csv(state.raceline_dir / "raceline_iqp.csv")
                    self.send_json({"ok": True, "warnings": validate_sector_profile(payload, raceline)})
                    return
                if path == "/api/sectors/save":
                    raceline = load_raceline_csv(state.raceline_dir / "raceline_iqp.csv")
                    exported = export_sector_files(map_dir=state.map_yaml.parent, profile_path=state.sector_profile_path, raceline=raceline, profile=payload)
                    saved_profile = load_sector_profile(state.sector_profile_path, raceline)
                    self.send_json({
                        "ok": True, "profile": saved_profile, "profile_path": str(exported.profile_path),
                        "speed_yaml_path": str(exported.speed_yaml_path), "ot_yaml_path": str(exported.ot_yaml_path), "warnings": list(exported.warnings),
                    })
                    return
                if path == "/api/shutdown":
                    self.send_json({"ok": True})
                    threading.Thread(target=self.server.shutdown, daemon=True).start()
                    return
                self.send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except FileNotFoundError as exc:
                self.send_json({"ok": False, "error": str(exc)}, HTTPStatus.NOT_FOUND)
            except Exception as exc:
                self.send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)

    server = ThreadingHTTPServer((host, port), Handler)
    url = f"http://{host}:{server.server_port}"
    print(f"Map editor: {url}")
    print(f"Map: {map_yaml}")
    print(f"Edit profile: {profile_path}")
    print(f"Sector profile: {sector_profile_path}")
    print(f"Raceline dir: {raceline_dir}")
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
