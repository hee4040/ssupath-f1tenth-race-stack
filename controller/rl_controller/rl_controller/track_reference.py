"""학습과 동일한 규약의 '중심선 기준선'을 실차에서 재구성한다.

왜 그대로 옮겨야 하는가: 정책 관측의 횡오차/폭/곡률은 전부 '중심선 ± 반폭
대칭 트랙' 정의를 기준으로 정규화되어 있다. 학습에서 실측 대회장 맵은

    race_stack global_waypoints.json 의 centerline_waypoints
      -> scripts/convert_map_json.py  (x_m, y_m, d_right, d_left -> CSV)
      -> dacerpp_lab/tracks.py: load_f1tenth_centerline()
         (좌우 비대칭 -> 법선 재정렬로 대칭화, ds=0.15m 등호장 리샘플,
          박스카 평활 2회, 곡률/근접 기반 폭 클립, 폭 평활, 벽 교차 제거)

경로로 트랙이 되었다. 배포에서 이 전처리를 생략하면 (특히 대칭화와 폭 클립)
정책이 학습과 다른 좌표계/스케일의 lateral·width 를 보게 된다. 그래서 아래는
dacerpp_lab/tracks.py 의 해당 함수들을 numpy 그대로 옮긴 것이다.

주의: 학습 로더는 마지막에 center -= center.mean() 으로 원점화하지만, 여기서는
맵 좌표계의 차량 위치를 투영해야 하므로 원점화하지 않는다(관측은 상대량이라
무관하다).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

# dacerpp_lab/env_cfg.py TrackParams 기본값 (학습에 쓰인 값 — 바꾸지 말 것)
DS = 0.15
HW_CURV_ALPHA = 0.7
HW_CURV_FLOOR = 0.35
HW_SMOOTH_WIN = 9


# --------------------------------------------------------------------------- #
# tracks.py 이식 (동일 수식)
# --------------------------------------------------------------------------- #
def centerline_features(cl: np.ndarray) -> dict:
    """중심선 -> seg/seglen/s/psi/kappa (폐곡선 가정)."""
    nxt = np.roll(cl, -1, axis=0)
    seg = nxt - cl
    seglen = np.maximum(np.linalg.norm(seg, axis=1), 1e-9)
    s = np.concatenate([[0.0], np.cumsum(seglen)[:-1]])
    total_s = float(np.sum(seglen))
    psi = np.arctan2(seg[:, 1], seg[:, 0])
    dpsi = (np.diff(psi, append=psi[0]) + np.pi) % (2 * np.pi) - np.pi
    kappa = dpsi / seglen
    return dict(seg=seg, seglen=seglen, s=s, total_s=total_s, psi=psi, kappa=kappa)


def _resample_equal_arclength(xy: np.ndarray, n: int, attrs: np.ndarray | None = None):
    seg = np.roll(xy, -1, axis=0) - xy
    seglen = np.linalg.norm(seg, axis=1)
    s = np.concatenate([[0.0], np.cumsum(seglen)])
    target = np.linspace(0.0, s[-1], n, endpoint=False)
    closed = np.vstack([xy, xy[0]])
    closed_at = np.vstack([attrs, attrs[0]]) if attrs is not None else None
    out = np.empty((n, 2))
    out_at = np.empty((n, attrs.shape[1])) if attrs is not None else None
    j = 0
    for k, ts in enumerate(target):
        while j < len(s) - 1 and s[j + 1] < ts:
            j += 1
        a = (ts - s[j]) / max(s[j + 1] - s[j], 1e-9)
        out[k] = closed[j] * (1 - a) + closed[j + 1] * a
        if closed_at is not None:
            out_at[k] = closed_at[j] * (1 - a) + closed_at[j + 1] * a
    return (out, out_at) if attrs is not None else out


def _polyline_self_intersects(poly: np.ndarray, min_gap: int = 10) -> bool:
    n = len(poly)
    if n < 8:
        return False
    a = poly
    d = np.roll(poly, -1, axis=0) - poly
    A, D = a[:, None, :], d[:, None, :]
    C, E = a[None, :, :], d[None, :, :]
    denom = D[..., 0] * E[..., 1] - D[..., 1] * E[..., 0]
    ok = np.abs(denom) > 1e-12
    dd = np.where(ok, denom, 1.0)
    AC = C - A
    t = (AC[..., 0] * E[..., 1] - AC[..., 1] * E[..., 0]) / dd
    u = (AC[..., 0] * D[..., 1] - AC[..., 1] * D[..., 0]) / dd
    hit = ok & (t > 1e-6) & (t < 1 - 1e-6) & (u > 1e-6) & (u < 1 - 1e-6)
    idx = np.arange(n)
    gap = np.abs(idx[:, None] - idx[None, :])
    gap = np.minimum(gap, n - gap)
    return bool((hit & (gap >= min_gap)).any())


def _corridor_self_interference(cl: np.ndarray, hw: np.ndarray, ds: float,
                                min_gap: int = 2) -> bool:
    step = max(1, int(round(0.3 / max(ds, 1e-6))))
    p, w = cl[::step], hw[::step]
    psi = centerline_features(p)["psi"]
    nrm = np.stack([-np.sin(psi), np.cos(psi)], axis=1)
    for side in (+1.0, -1.0):
        if _polyline_self_intersects(p + side * nrm * w[:, None], min_gap=min_gap):
            return True
    return False


def clip_width_by_curvature(cl: np.ndarray, hw: np.ndarray) -> np.ndarray:
    R = 1.0 / np.maximum(np.abs(centerline_features(cl)["kappa"]), 1e-6)
    return np.minimum(hw, np.maximum(HW_CURV_ALPHA * R, HW_CURV_FLOOR))


def clip_width_by_proximity(cl: np.ndarray, hw: np.ndarray, ds: float = DS,
                            min_arc: float = 2.0, safety: float = 0.95) -> np.ndarray:
    n_full = len(cl)
    step = max(1, int(round(0.3 / ds)))
    p = cl[::step]
    n = len(p)
    idx = np.arange(n)
    di = np.abs(idx[:, None] - idx[None, :])
    arc = np.minimum(di, n - di) * step * ds
    dist = np.linalg.norm(p[:, None] - p[None, :], axis=2)
    dmin = np.where(arc > min_arc, dist, np.inf).min(axis=1) * 0.5 * safety
    if not np.isfinite(dmin).all():
        dmin = np.where(np.isfinite(dmin), dmin, hw.max())
    full = np.interp(np.arange(n_full), np.arange(0, n_full, step)[:n], dmin, period=n_full)
    return np.minimum(hw, full)


def smooth_half_width(hw: np.ndarray, win: int = HW_SMOOTH_WIN) -> np.ndarray:
    if win < 3 or len(hw) <= win:
        return hw
    h = win // 2
    k = np.ones(win) / win
    pad = np.concatenate([hw[-h:], hw, hw[:h]])
    return np.convolve(pad, k, mode="valid")[:len(hw)].astype(np.float64)


def shrink_width_until_clean(cl: np.ndarray, hw: np.ndarray, ds: float = DS,
                             max_iter: int = 8, factor: float = 0.88):
    for _ in range(max_iter):
        if not _corridor_self_interference(cl, hw, ds):
            return hw
        hw = np.maximum(hw * factor, 0.30)
    return None if _corridor_self_interference(cl, hw, ds) else hw


# --------------------------------------------------------------------------- #
@dataclass
class TrackReference:
    """중심선 기준 투영/전방 조회. 학습 VectorizedTracks 의 단일차량 판."""

    pts: np.ndarray          # (n,2) 중심선 (맵 좌표)
    seg: np.ndarray          # (n,2)
    seglen: np.ndarray       # (n,)
    s: np.ndarray            # (n,)
    psi: np.ndarray          # (n,)
    kappa: np.ndarray        # (n,)
    hw: np.ndarray           # (n,) 반폭
    total_s: float
    n_shrunk: bool = False

    # ------------------------------------------------------------------ #
    @classmethod
    def from_centerline_waypoints(cls, x: Sequence[float], y: Sequence[float],
                                  d_left: Sequence[float], d_right: Sequence[float],
                                  ds: float = DS, logger=None) -> "TrackReference":
        xy = np.stack([np.asarray(x, dtype=np.float64),
                       np.asarray(y, dtype=np.float64)], axis=1)
        w_l = np.asarray(d_left, dtype=np.float64)
        w_r = np.asarray(d_right, dtype=np.float64)
        if len(xy) < 8:
            raise ValueError(f"중심선 웨이포인트가 너무 적습니다: {len(xy)}")
        if np.linalg.norm(xy[0] - xy[-1]) < 1e-6:        # 폐곡선 중복 끝점 제거
            xy, w_l, w_r = xy[:-1], w_l[:-1], w_r[:-1]

        # 좌/우 비대칭 -> 법선 재정렬로 대칭 half-width 화 (load_f1tenth_centerline)
        seg = np.roll(xy, -1, axis=0) - xy
        psi = np.arctan2(seg[:, 1], seg[:, 0])
        normal = np.stack([-np.sin(psi), np.cos(psi)], axis=1)     # 좌측(+) 법선
        center = xy + normal * ((w_l - w_r) * 0.5)[:, None]
        hw = 0.5 * (w_l + w_r)

        # 등호장 리샘플 -> 박스카 평활 2회(±0.75m) -> 재리샘플
        total = np.linalg.norm(np.roll(center, -1, axis=0) - center, axis=1).sum()
        n = max(64, int(round(total / ds)))
        center, at = _resample_equal_arclength(center, n, attrs=hw[:, None])
        k = np.ones(11) / 11.0
        for _ in range(2):
            pad = np.vstack([center[-5:], center, center[:5]])
            center = np.stack([np.convolve(pad[:, d], k, mode="valid") for d in (0, 1)], axis=1)
            padw = np.concatenate([at[-5:, 0], at[:, 0], at[:5, 0]])
            at = np.convolve(padw, k, mode="valid")[:, None]
        total = np.linalg.norm(np.roll(center, -1, axis=0) - center, axis=1).sum()
        n = max(64, int(round(total / ds)))
        center, at = _resample_equal_arclength(center, n, attrs=at)
        center = center.astype(np.float64)

        # 폭 정리 (안쪽 벽 접힘/코리도 겹침 방지) — 학습 트랙과 동일 처리
        hw_out = clip_width_by_curvature(center, at[:, 0].astype(np.float64))
        hw_out = clip_width_by_proximity(center, hw_out, ds)
        hw_out = smooth_half_width(hw_out)
        cleaned = shrink_width_until_clean(center, hw_out, ds)
        shrunk = cleaned is None
        if shrunk:
            cleaned = np.maximum(hw_out * 0.88 ** 8, 0.30)
            if logger is not None:
                logger.warn("[track] 벽 교차가 남아 반폭을 최대 축소했습니다 "
                            "(맵 중심선이 자기교차하는지 확인).")

        f = centerline_features(center)
        return cls(pts=center, seg=f["seg"], seglen=f["seglen"], s=f["s"], psi=f["psi"],
                   kappa=f["kappa"], hw=np.asarray(cleaned, dtype=np.float64),
                   total_s=f["total_s"], n_shrunk=shrunk)

    # ------------------------------------------------------------------ #
    def project(self, x: float, y: float) -> dict:
        """차량 (x,y) -> dict(s, lateral, idx, psi).

        학습 VectorizedTracks.project 와 동일: 최근접 정점 i0 를 찾고 인접 두
        세그먼트(i0-1, i0)에 투영해 더 가까운 쪽을 채택. lateral 은 좌측(+).
        """
        p = np.array([x, y], dtype=np.float64)
        n = len(self.pts)
        i0 = int(np.argmin(((self.pts - p) ** 2).sum(axis=1)))

        best = None
        for idx in ((i0 - 1) % n, i0):
            a = self.pts[idx]
            sg = self.seg[idx]
            L2 = max(self.seglen[idx] ** 2, 1e-9)
            t = float(np.clip(((p - a) @ sg) / L2, 0.0, 1.0))
            proj = a + t * sg
            diff = p - proj
            dist = float(np.linalg.norm(diff))
            if best is None or dist < best[0]:
                cross = sg[0] * diff[1] - sg[1] * diff[0]
                best = (dist, float(self.s[idx] + t * self.seglen[idx]),
                        dist * (1.0 if cross >= 0 else -1.0), idx)
        _, s_val, lateral, idx = best
        return dict(s=s_val, lateral=lateral, idx=idx, psi=float(self.psi[idx]))

    def lookahead_curvature(self, idx: int, offsets: Sequence[int]) -> np.ndarray:
        n = len(self.pts)
        return np.array([self.kappa[(idx + o) % n] for o in offsets], dtype=np.float64)

    def lookahead_width(self, idx: int, offsets: Sequence[int]) -> np.ndarray:
        n = len(self.pts)
        return np.array([self.hw[(idx + o) % n] for o in offsets], dtype=np.float64)

    def summary(self) -> str:
        return (f"n={len(self.pts)} lap={self.total_s:.1f}m ds={self.total_s/len(self.pts):.3f}m "
                f"hw=[{self.hw.min():.2f},{self.hw.max():.2f}]m "
                f"|kappa|max={np.abs(self.kappa).max():.2f}"
                + (" (폭 최대축소됨)" if self.n_shrunk else ""))
