"""Livox 3D 포인트클라우드 -> 학습과 동일 포맷의 의사 2D 스캔(32빔).

학습 racing_env._scan docstring 이 지정한 파이프라인 그대로:
    deskew -> 지면 위 높이 밴드 필터 -> 차체 기준 ±scan_fov 를 n_beams 개
    방위각 섹터로 분할 -> 섹터별 최소거리 -> max_range 클립.

실측(lobby_0728 bag, MID360)에서 확인한 사항:
  - 무반사 점이 (0,0,0) 으로 들어온다 -> 원점 근처 점은 버려야 한다.
  - 차체/전장품 자체 반사가 base_link 원점 부근(수평거리 0.05~0.15m)과
    z>0.3m 반경 0.4~0.55m 에 상시 존재한다 -> footprint 박스로 제외.
  - 덕트 벽은 base_link 기준 z 0.0~0.3m 에 잡히고 지면은 z≈-0.05m 이다.
  - 한 프레임은 100ms 스윕이고 점마다 절대 타임스탬프(ns)가 들어 있다.
    5m/s 면 스윕 동안 0.5m 를 움직이므로 점별 디스큐가 필수다.
"""
from __future__ import annotations

import math
from typing import Callable, Optional, Tuple

import numpy as np

_DTYPE_MAP = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
              5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def parse_pointcloud(msg) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """PointCloud2 -> (xyz (n,3) float32, t_sec (n,) float64 | None).

    Livox pc2 포맷은 x,y,z,intensity,tag,line,timestamp(float64 절대 ns).
    필드 이름으로 찾으므로 다른 드라이버의 클라우드에도 동작한다.
    """
    names = {f.name: f for f in msg.fields}
    if not all(k in names for k in ("x", "y", "z")):
        raise ValueError("PointCloud2 에 x/y/z 필드가 없습니다")
    fields, offsets, formats = [], [], []
    for key in ("x", "y", "z"):
        f = names[key]
        fields.append(key)
        offsets.append(f.offset)
        formats.append(_DTYPE_MAP[f.datatype])
    has_t = "timestamp" in names
    if has_t:
        f = names["timestamp"]
        fields.append("t")
        offsets.append(f.offset)
        formats.append(_DTYPE_MAP[f.datatype])
    dt = np.dtype({"names": fields, "formats": formats, "offsets": offsets,
                   "itemsize": msg.point_step})
    arr = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)
    if not has_t:
        return xyz, None
    t = np.asarray(arr["t"], dtype=np.float64)
    # Livox 는 절대 ns. 초 단위로 환산(이미 초 단위인 드라이버도 있어 크기로 판별).
    if t.size and t.max() > 1e15:
        t = t * 1e-9
    return xyz, t


class PseudoScanBuilder:
    """포인트클라우드를 받아 두었다가 제어 시점의 차체 기준 스캔을 만든다."""

    def __init__(self, n_beams: int = 32, fov: float = 2.356, max_range: float = 10.0,
                 z_min: float = 0.02, z_max: float = 0.30, min_range: float = 0.05,
                 self_box: Tuple[float, float, float] = (-0.40, 0.55, 0.20),
                 deskew: bool = True):
        self.n_beams = int(n_beams)
        self.fov = float(fov)
        self.max_range = float(max_range)
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.min_range = float(min_range)
        self.self_box = tuple(float(v) for v in self_box)
        self.deskew = bool(deskew)
        self.angles = np.linspace(-self.fov, self.fov, self.n_beams)
        self.spacing = 2.0 * self.fov / max(self.n_beams - 1, 1)

        # 라이다 -> base_link 외부 파라미터 (기본값은 race_stack static TF)
        self.set_extrinsic(np.array([0.27, 0.0, 0.07]), np.deg2rad(90.0))

        self._xy: Optional[np.ndarray] = None      # (m,2) 점 시각의 base_link 좌표
        self._t: Optional[np.ndarray] = None       # (m,) 점 시각 [s]
        self._stamp: float = -1.0                  # 프레임 헤더 시각 [s]
        self._n_raw: int = 0

    # ------------------------------------------------------------------ #
    def set_extrinsic(self, translation: np.ndarray, yaw: float,
                      pitch: float = 0.0, roll: float = 0.0) -> None:
        """base_link <- lidar 변환. race_stack 기본은 t=(0.27,0,0.07), yaw=+90deg."""
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        self._R = (Rz @ Ry @ Rx).astype(np.float64)
        self._t_ext = np.asarray(translation, dtype=np.float64).reshape(3)

    # ------------------------------------------------------------------ #
    def ingest_cloud(self, msg) -> int:
        """PointCloud2 를 필터링해 보관. 살아남은 점 개수를 반환."""
        xyz, t = parse_pointcloud(msg)
        self._n_raw = len(xyz)
        if self._n_raw == 0:
            return 0
        # 무반사 점 (원점) 제거
        valid = (np.abs(xyz).sum(axis=1) > 1e-3)
        xyz = xyz[valid].astype(np.float64)
        if t is not None:
            t = t[valid]

        pb = xyz @ self._R.T + self._t_ext
        r = np.hypot(pb[:, 0], pb[:, 1])
        in_box = ((pb[:, 0] > self.self_box[0]) & (pb[:, 0] < self.self_box[1])
                  & (np.abs(pb[:, 1]) < self.self_box[2]))
        keep = ((pb[:, 2] >= self.z_min) & (pb[:, 2] <= self.z_max)
                & (r >= self.min_range) & (r < self.max_range) & (~in_box))

        self._xy = pb[keep, :2]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._t = t[keep] if t is not None else np.full(int(keep.sum()), stamp)
        self._stamp = stamp
        return int(keep.sum())

    def ingest_laserscan(self, msg) -> int:
        """시뮬레이터용 2D LaserScan 경로 (디스큐 없음)."""
        rng = np.asarray(msg.ranges, dtype=np.float64)
        ang = msg.angle_min + np.arange(len(rng)) * msg.angle_increment
        ok = np.isfinite(rng) & (rng >= max(self.min_range, msg.range_min)) & (rng < self.max_range)
        rng, ang = rng[ok], ang[ok]
        # LaserScan 은 라이다 프레임 -> base_link 로 평면 변환
        yaw = math.atan2(self._R[1, 0], self._R[0, 0])
        x = rng * np.cos(ang + yaw) + self._t_ext[0]
        y = rng * np.sin(ang + yaw) + self._t_ext[1]
        self._xy = np.stack([x, y], axis=1)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._t = np.full(len(x), stamp)
        self._stamp = stamp
        self._n_raw = len(rng)
        return len(x)

    # ------------------------------------------------------------------ #
    @property
    def stamp(self) -> float:
        return self._stamp

    def build(self, now_pose: Optional[Tuple[float, float, float]] = None,
              pose_at: Optional[Callable[[np.ndarray], Tuple[np.ndarray, np.ndarray,
                                                             np.ndarray]]] = None
              ) -> Tuple[np.ndarray, int]:
        """현재 차체 기준 (n_beams,) 스캔과 유효 빔 수를 반환.

        now_pose/pose_at 가 주어지고 deskew=True 면 점마다 측정 시각의 포즈로
        맵 좌표에 되돌린 뒤 '현재' 차체 좌표로 옮긴다(스윕 왜곡 + 지연 보상).
        """
        scan = np.full(self.n_beams, self.max_range)
        if self._xy is None or len(self._xy) == 0:
            return scan, 0

        xy = self._xy
        if self.deskew and now_pose is not None and pose_at is not None and self._t is not None:
            px, py, pyaw = pose_at(self._t)
            if px is not None:
                x0, y0, yaw0 = now_pose
                dyaw = pyaw - yaw0
                c, s = np.cos(dyaw), np.sin(dyaw)
                cn, sn = math.cos(-yaw0), math.sin(-yaw0)
                dx, dy = px - x0, py - y0
                bx = cn * dx - sn * dy
                by = sn * dx + cn * dy
                rx = c * xy[:, 0] - s * xy[:, 1] + bx
                ry = s * xy[:, 0] + c * xy[:, 1] + by
                xy = np.stack([rx, ry], axis=1)

        r = np.hypot(xy[:, 0], xy[:, 1])
        a = np.arctan2(xy[:, 1], xy[:, 0])
        m = (np.abs(a) <= self.fov + 0.5 * self.spacing) & (r < self.max_range) \
            & (r >= self.min_range)
        if not m.any():
            return scan, 0
        idx = np.clip(np.rint((a[m] + self.fov) / self.spacing).astype(np.int64),
                      0, self.n_beams - 1)
        np.minimum.at(scan, idx, r[m])
        return scan, int(np.bincount(idx, minlength=self.n_beams).astype(bool).sum())
