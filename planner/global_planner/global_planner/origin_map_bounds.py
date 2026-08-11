#!/usr/bin/env python3
"""
편집 맵이 아니라 **원본 맵**의 벽 위치를 기준으로 트랙 경계(d_left/d_right)를 다시 계산한다.

배경
----
`global_waypoints.json` 의 d_left/d_right 는 global planner 가 편집 맵
(`<map>.png`) 에서 뽑은 값이다. 편집 맵은 레이스라인이 이상한 구석으로 새지
않도록 알코브/움푹한 곳을 까맣게 메워 놓기 때문에, 그 구간의 경계는 **실제 벽보다
안쪽**에 있다.

그런데 `cluster_to_obstacle` 는 이 d_left/d_right 를 "여기까지가 트랙"이라는
장애물 판정 기준으로 그대로 쓴다(laserPointOnTrack). 그래서 편집으로 메워진
구간에서는

  * 그 자리에 실제로 놓인 장애물이 "트랙 밖"으로 분류돼 통째로 버려지고,
  * 반대로 편집이 트랙을 넓힌 구간에서는 진짜 벽 점군이 트랙 안으로 들어와
    유령 장애물이 된다.

주행 경로(레이스라인/회피 corridor)는 편집 맵 기준이 맞다. 하지만 "저게 장애물인가
벽인가" 판정만큼은 실제 벽 = 원본 맵(`<map>_origin.png`) 을 봐야 한다.
이 모듈이 그 차이만큼만 경계를 보정해 준다.

방식 — 절대값이 아니라 '차이'를 쓴다
------------------------------------
각 웨이포인트에서 좌/우 법선 방향으로 편집 맵과 원본 맵 각각에 레이캐스팅을 해서
벽까지의 거리를 구하고, **그 차이만** 저장된 d_left/d_right 에 더한다.

    d_left_new = d_left_json + (ray_origin_left - ray_edited_left)

두 맵이 같은 구간에서는 차이가 정확히 0 이라 경계가 그대로 유지된다. 즉
`boundaries_inflation`, `min_intrusion` 같은 기존 튜닝값이 의미를 그대로 지킨다.
경계를 처음부터 다시 뽑으면(레이캐스팅 절대값으로 대체) json 의 safety-width 여유가
사라져서 튜닝을 전부 다시 해야 한다.

기본은 '넓히는 방향'만 (widen_only)
-----------------------------------
차이가 음수(원본이 더 좁음)인 경우는 거의 두 가지다.
  1. 편집자가 원본 맵의 벽 가장자리 삐죽한 부분(1~5 셀)을 다듬은 것 — 노이즈다.
  2. 레이스라인이 벽에 바짝 붙은 구간에서 레이가 벽을 스치듯 지나가(grazing) 두 맵의
     한두 셀 차이가 거리 수십 cm 차이로 증폭된 것.
lobby_0811 실측에서 음수 델타는 전부 이 둘이었고(편집이 흰색으로 바꾼 픽셀은 총 43개,
가장 큰 덩어리도 27셀), 최대 -0.21 m 까지 튀었다. 그대로 반영하면 그 구간 탐지 폭이
0.07 m 로 무너져서 오히려 장애물을 더 놓친다.
그래서 기본값은 `widen_only=True` — 편집 맵 경계보다 좁아지지 않는다. 최소한
지금까지의 동작보다 나빠지지는 않는다는 보장이 된다.

좌표 규약은 global_planner_logic.py 와 동일하다.
  * 이미지를 세로로 뒤집어(cv2.flip(img, 0)) 행 인덱스 = 아래에서부터의 y 셀
  * x_world = col * resolution + origin_x,  y_world = row * resolution + origin_y
  * yaml origin 의 3번째 값(yaw)은 global planner 도 무시하므로 여기서도 무시한다
  * psi_rad 는 ROS 규약(+x 가 0), 좌측 법선 = (-sin psi, cos psi)
"""

import copy
import glob
import os

import numpy as np

# 흰색(자유공간) 판정 문턱. png 는 순수 0/255 라 값 자체는 중요하지 않다.
_FREE_THRESH = 128

#: 원본 png 접미사. 앞으로 원본 맵은 `<map>_origin.png` 로 저장한다.
ORIGIN_SUFFIX = '_origin'


def find_origin_map(map_dir: str):
    """맵 디렉토리에서 (원본 png 경로, 맵 yaml 경로) 를 찾는다. 없으면 (None, None).

    `<base>_origin.png` 를 찾고, 해상도/원점은 짝이 되는 `<base>.yaml` 에서 읽는다
    (원본과 편집본은 같은 크롭이라 같은 yaml 을 공유한다는 전제).
    """
    candidates = sorted(glob.glob(os.path.join(map_dir, '*%s.png' % ORIGIN_SUFFIX)))
    for origin_png in candidates:
        base = os.path.basename(origin_png)[:-len(ORIGIN_SUFFIX + '.png')]
        map_yaml = os.path.join(map_dir, base + '.yaml')
        edited_png = os.path.join(map_dir, base + '.png')
        if os.path.isfile(map_yaml) and os.path.isfile(edited_png):
            return origin_png, map_yaml
    return None, None


def _load_grid(png_path: str) -> np.ndarray:
    """png 를 자유공간 bool 격자로 읽는다. 행 인덱스는 아래에서부터의 y 셀."""
    import cv2  # 지연 import: cv2 가 없어도 republisher 본체는 살아 있어야 한다

    img = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise IOError('cannot read %s' % png_path)
    return cv2.flip(img, 0) > _FREE_THRESH


def _raycast(free: np.ndarray, xs, ys, nxs, nys, res, ox, oy, max_ray):
    """(xs, ys) 에서 (nxs, nys) 방향으로 첫 점유 셀까지의 거리를 벡터화해서 구한다.

    격자 밖은 점유로 본다. 끝까지 자유공간이면 max_ray 를 돌려준다.
    """
    height, width = free.shape
    step = res * 0.25
    n_steps = max(1, int(np.ceil(max_ray / step)))
    dists = np.arange(1, n_steps + 1) * step            # (N,)

    px = xs[:, None] + nxs[:, None] * dists[None, :]    # (M, N)
    py = ys[:, None] + nys[:, None] * dists[None, :]

    cols = np.rint((px - ox) / res).astype(np.int64)
    rows = np.rint((py - oy) / res).astype(np.int64)
    inside = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)

    free_at = np.zeros(cols.shape, dtype=bool)
    free_at[inside] = free[rows[inside], cols[inside]]
    occupied = ~free_at                                  # 격자 밖 = 점유

    hit = occupied.any(axis=1)
    first = np.argmax(occupied, axis=1)                  # hit 인 행에서만 유효
    # dists[k] = (k+1)*step 이므로, 첫 점유가 k 면 마지막 자유 샘플은 k*step.
    out = np.where(hit, first * step, float(max_ray))
    return out


def compute_origin_bounds(map_dir, glb_wpnts, logger=None,
                          max_delta=0.6, max_ray=4.0, min_bound=0.05,
                          widen_only=True):
    """편집 맵 대비 원본 맵의 벽 위치 차이만큼 d_left/d_right 를 보정한 WpntArray 반환.

    Args:
        map_dir:    맵 디렉토리 (`<base>.png`, `<base>_origin.png`, `<base>.yaml` 이 있는 곳)
        glb_wpnts:  편집 맵 기준 글로벌 웨이포인트 (f110_msgs/WpntArray)
        logger:     rclpy logger (선택)
        max_delta:  한 웨이포인트에서 허용할 경계 보정량의 상한 [m].
                    원본 맵의 구멍으로 레이가 빠져나가 경계가 폭주하는 것을 막는다.
        max_ray:    레이캐스팅 최대 거리 [m]
        min_bound:  보정 후 경계 하한 [m]
        widen_only: True 면 편집 맵 경계보다 좁히지 않는다(권장, 이유는 모듈 docstring).
                    False 로 두면 원본이 더 좁은 구간도 그대로 반영한다.

    Returns:
        (보정된 WpntArray, 사람이 읽을 요약 문자열).
        원본 맵이 없거나 계산에 실패하면 (None, 사유) 를 돌려준다. 호출자는 이때
        편집 맵 기준 경계를 그대로 쓰면 된다(= 지금까지의 동작).
    """
    def _log(msg):
        if logger is not None:
            logger.info(msg)

    if glb_wpnts is None or not glb_wpnts.wpnts:
        return None, 'no global waypoints'

    origin_png, map_yaml = find_origin_map(map_dir)
    if origin_png is None:
        return None, ('no *%s.png in %s -> 편집 맵 경계를 그대로 사용'
                      % (ORIGIN_SUFFIX, map_dir))

    base = os.path.basename(origin_png)[:-len(ORIGIN_SUFFIX + '.png')]
    edited_png = os.path.join(map_dir, base + '.png')

    try:
        import yaml
        with open(map_yaml) as f:
            info = yaml.safe_load(f)
        res = float(info['resolution'])
        ox, oy = float(info['origin'][0]), float(info['origin'][1])

        free_edit = _load_grid(edited_png)
        free_orig = _load_grid(origin_png)
    except Exception as exc:  # noqa: BLE001 - 어떤 실패든 편집 맵으로 폴백해야 한다
        return None, 'origin map load failed (%s)' % exc

    if free_edit.shape != free_orig.shape:
        return None, ('map size mismatch: %s %s vs %s %s (같은 크롭이어야 한다)'
                      % (base + '.png', free_edit.shape,
                         os.path.basename(origin_png), free_orig.shape))

    wpnts = glb_wpnts.wpnts
    xs = np.array([w.x_m for w in wpnts])
    ys = np.array([w.y_m for w in wpnts])
    psis = np.array([w.psi_rad for w in wpnts])
    nxs, nys = -np.sin(psis), np.cos(psis)               # 좌측 법선

    edit_l = _raycast(free_edit, xs, ys, nxs, nys, res, ox, oy, max_ray)
    edit_r = _raycast(free_edit, xs, ys, -nxs, -nys, res, ox, oy, max_ray)
    orig_l = _raycast(free_orig, xs, ys, nxs, nys, res, ox, oy, max_ray)
    orig_r = _raycast(free_orig, xs, ys, -nxs, -nys, res, ox, oy, max_ray)

    lo = 0.0 if widen_only else -max_delta
    delta_l = np.clip(orig_l - edit_l, lo, max_delta)
    delta_r = np.clip(orig_r - edit_r, lo, max_delta)

    # 원본 맵에서 레이스라인 자체가 벽 안이면(편집이 트랙을 뚫어 만든 구간 등)
    # 좌우 거리가 0 으로 무너진다. 그런 점은 보정하지 않고 편집 맵 값을 유지한다.
    cols = np.rint((xs - ox) / res).astype(np.int64)
    rows = np.rint((ys - oy) / res).astype(np.int64)
    height, width = free_orig.shape
    inside = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
    on_free = np.zeros(len(wpnts), dtype=bool)
    on_free[inside] = free_orig[rows[inside], cols[inside]]
    delta_l[~on_free] = 0.0
    delta_r[~on_free] = 0.0

    out = copy.deepcopy(glb_wpnts)
    for i, w in enumerate(out.wpnts):
        w.d_left = max(min_bound, w.d_left + float(delta_l[i]))
        w.d_right = max(min_bound, w.d_right + float(delta_r[i]))

    n_changed = int(np.count_nonzero((np.abs(delta_l) > 1e-6) | (np.abs(delta_r) > 1e-6)))
    summary = ('origin map %s: %d/%d wpnts 경계 보정 '
               '(left %+.3f~%+.3f m, right %+.3f~%+.3f m, widen_only=%s, 벽 안 wpnt %d)'
               % (os.path.basename(origin_png), n_changed, len(wpnts),
                  delta_l.min(), delta_l.max(), delta_r.min(), delta_r.max(),
                  widen_only, int(np.count_nonzero(~on_free))))
    _log(summary)
    return out, summary
