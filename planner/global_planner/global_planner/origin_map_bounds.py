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

역할 분담이 이렇다:
  * 편집 맵 (`<map>.png`)        -> 레이스라인/회피 corridor 를 원하는 모양으로 뽑는 용도.
                                    global_waypoints.json 이 만들어지면 역할 끝.
  * 원본 맵 (`<map>_origin.png`) -> "저게 장애물인가 벽인가" 판정 기준. 실제 벽이어야 한다.

방식 — 원본 맵 벽까지의 '최단거리' 절대값
----------------------------------------
각 웨이포인트에서 좌/우 반평면 안의 가장 가까운 점유 셀까지의 거리를 재서
d_left/d_right 를 그대로 대체한다(_nearest_bound).

이건 새 로직이 아니라 **스택이 원래 쓰던 방식**이다. global_planner 는
`extract_track_bounds`(watershed) 가 실패하면 `cv2.distanceTransform` 으로 폴백하고,
`dist_to_bounds()` 는 `np.amin(...)` 으로 최단거리를 쓴다. lobby_0812 는 편집본·원본
모두 watershed 가 실패해서 애초에 거리변환 경로로 갔다. 검증: 편집본을 이 방식으로
재면 json 의 d_left/d_right 와 4.4~4.7 cm 차이로 일치한다(그 차이가 safety_width 여유).

★ 레이캐스팅(법선 방향으로 쏘기)을 쓰면 안 된다 — 2026-08-11 에 그렇게 만들었다가
  2026-08-12 에 걷어냈다. 벽을 비스듬히 만나는 구간에서 실제 벽보다 최대 +89 cm 까지
  튀고(lobby_0812 실측, 우측 p90 +43 cm), 그 구간은 벽이 통째로 트랙 안으로 들어온다.
  그걸 피하려고 '두 맵 레이캐스팅의 차이만 더하기 + widen_only' 라는 우회로를 만들었는데,
  그러면 편집 경계에서 델타가 0/+0.5 로 갈려 경계가 계단처럼 튀었다(0.1 m 초과 점프
  13 군데, 최대 34 cm). 최단거리로 재면 그런 튐이 없다(점프 0 군데, json 과 동급).

주의 — 경계가 넓어지므로 min_intrusion 을 같이 올려야 한다
--------------------------------------------------------
절대값을 쓰면 json 에 있던 safety_width 여유가 사라져 경계가 진짜 벽까지 나간다
(lobby_0812: 좌 +4.6 cm, 우 +14.9 cm). 경계를 δ 넓히는 것은 min_intrusion 을 δ
내리는 것과 같으므로, 그만큼 문턱을 올리지 않으면 벽 유령이 늘어난다.
2026-08-12 기준 cluster_to_obstacle 의 min_intrusion 은 0.37 -> 0.42 로 같이 올렸다.
얻는 것은 평균 성능이 아니라 **일관성**이다 — 50 cm 상자가 벽에서 떨어져야 하는 거리가
지점에 따라 16~39 cm 로 흔들리던 것이 전 구간 17 cm 로 균일해진다.

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


def _nearest_bound(free: np.ndarray, xs, ys, nxs, nys, res, ox, oy, max_ray):
    """각 웨이포인트에서 좌/우 **최단거리** 로 원본 맵 벽까지의 거리를 구한다.

    좌/우 반평면 안에서 가장 가까운 점유 셀까지의 거리를 쓴다.
    global_planner 의 dist_to_bounds() 와 같은 방식이라(np.amin) json 의
    d_left/d_right 와 측정 기준이 일치한다. 레이캐스팅을 쓰면 안 되는 이유는
    모듈 docstring 참조.

    좌/우 구분은 법선 방향 투영 부호로 한다(좌측 법선 = (-sin psi, cos psi)).
    max_ray 안에 점유 셀이 없으면 max_ray 를 돌려준다.
    """
    from scipy.spatial import cKDTree

    rows_occ, cols_occ = np.where(~free)
    if len(rows_occ) == 0:
        return np.full(len(xs), float(max_ray)), np.full(len(xs), float(max_ray))

    wx = cols_occ * res + ox
    wy = rows_occ * res + oy
    tree = cKDTree(np.column_stack([wx, wy]))

    left = np.full(len(xs), float(max_ray))
    right = np.full(len(xs), float(max_ray))
    for i in range(len(xs)):
        idx = tree.query_ball_point([xs[i], ys[i]], max_ray)
        if not idx:
            continue
        idx = np.asarray(idx)
        rx = wx[idx] - xs[i]
        ry = wy[idx] - ys[i]
        side = rx * nxs[i] + ry * nys[i]      # >0 이면 좌측
        dist = np.hypot(rx, ry)
        if np.any(side > 0):
            left[i] = dist[side > 0].min()
        if np.any(side < 0):
            right[i] = dist[side < 0].min()
    return left, right


def compute_origin_bounds(map_dir, glb_wpnts, logger=None,
                          max_ray=4.0, min_bound=0.05):
    """원본 맵의 진짜 벽 위치로 d_left/d_right 를 대체한 WpntArray 반환.

    x/y/psi/s/속도는 건드리지 않는다. 프레네 기준선이 그대로이므로 하류의 s/d 좌표도
    전혀 바뀌지 않는다 — 바뀌는 건 "어디까지가 트랙인가" 하나뿐이다.

    Args:
        map_dir:    맵 디렉토리 (`<base>.png`, `<base>_origin.png`, `<base>.yaml` 이 있는 곳)
        glb_wpnts:  편집 맵 기준 글로벌 웨이포인트 (f110_msgs/WpntArray)
        logger:     rclpy logger (선택)
        max_ray:    벽을 찾는 최대 반경 [m]. 이 안에 점유 셀이 없으면 max_ray 를 쓴다.
        min_bound:  경계 하한 [m]

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

    # 원본 맵에서 레이스라인 자체가 벽 안이면(편집이 트랙을 뚫어 만든 구간 등)
    # 좌우 거리가 0 으로 무너진다. 그런 점은 보정하지 않고 편집 맵 값을 유지한다.
    cols = np.rint((xs - ox) / res).astype(np.int64)
    rows = np.rint((ys - oy) / res).astype(np.int64)
    height, width = free_orig.shape
    inside = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
    on_free = np.zeros(len(wpnts), dtype=bool)
    on_free[inside] = free_orig[rows[inside], cols[inside]]

    # 원본 맵의 진짜 벽까지 '최단거리' 를 그대로 경계로 쓴다.
    # 편집 맵은 레이스라인을 원하는 모양으로 뽑기 위한 것이고, 거기서 역할이 끝난다.
    # 실주행 중 "저게 장애물인가 벽인가" 판정은 진짜 벽을 봐야 하므로 원본만 본다.
    orig_l, orig_r = _nearest_bound(free_orig, xs, ys, nxs, nys, res, ox, oy, max_ray)
    json_l = np.array([w.d_left for w in wpnts])
    json_r = np.array([w.d_right for w in wpnts])
    # 원본 맵에서 레이스라인 자체가 벽 안인 지점은 거리가 0 으로 무너지므로 손대지 않는다.
    new_l = np.where(on_free, orig_l, json_l)
    new_r = np.where(on_free, orig_r, json_r)
    delta_l = new_l - json_l
    delta_r = new_r - json_r

    out = copy.deepcopy(glb_wpnts)
    for i, w in enumerate(out.wpnts):
        w.d_left = max(min_bound, float(new_l[i]))
        w.d_right = max(min_bound, float(new_r[i]))

    n_changed = int(np.count_nonzero((np.abs(delta_l) > 1e-6) | (np.abs(delta_r) > 1e-6)))
    summary = ('origin map %s: %d/%d wpnts 경계 보정 '
               '(left %+.3f~%+.3f m, right %+.3f~%+.3f m, 벽 안 wpnt %d)'
               % (os.path.basename(origin_png), n_changed, len(wpnts),
                  delta_l.min(), delta_l.max(), delta_r.min(), delta_r.max(),
                  int(np.count_nonzero(~on_free))))
    _log(summary)
    return out, summary


def compute_origin_ltpl_widths(map_dir, ltpl_wpnts, logger=None,
                               max_ray=4.0, max_widen=0.6, min_width=0.05):
    """원본 맵의 진짜 벽 위치로 ltpl 웨이포인트의 width_left_m / width_right_m 을 넓힌다.

    왜 필요한가
    -----------
    graph_planner 는 회피 노드를 오직 이 두 폭에서 만든다
    (graph_planner.cpp createNodeMap):

        num_nodes      = (width_right_m + width_left_m - veh_width) / lat_resolution
        raceline_index = (width_left_m + alpha_m - veh_width/2) / lat_resolution

    그런데 이 폭은 **편집 맵**(`<map>.png`) 에서 나온 값이다. 편집 맵은 레이스라인을
    원하는 모양으로 뽑으려고 트랙 가장자리를 검게 칠해 좁혀 놓은 것이라, 그 붓질이
    그대로 회피용 노드를 지운다. 즉 "레이스라인을 만들려고 지운 공간" 때문에
    "장애물을 돌아갈 공간" 까지 없어진다 — 이 둘은 원래 무관해야 한다.

    실측 (2026-08-21, 최단거리 방식):
        lobby_0820  폭 1.26 -> 1.52 m,  노드/레이어  9 -> 14
        hall_0820   폭 1.13 -> 1.26 m,  노드/레이어  8 -> 10
        hall_0821   폭 0.84 -> 1.12 m,  노드/레이어  5 -> 10
      hall_0821 은 레이어당 노드가 중앙 5개(최소 2개)라, 0.3 m 장애물이 기본 여유로
      막는 폭 ±0.44 m = 노드 9칸이면 레이어가 통째로 막힌다. 여기가 가장 심하다.

    이건 새 발상이 아니라 이 모듈이 이미 cluster_to_obstacle 에 해 주고 있는 보정
    (compute_origin_bounds) 을 회피 corridor 에도 적용하는 것이다. 모듈 상단 주석의
    역할 분담 — "편집 맵 -> 레이스라인, 원본 맵 -> 진짜 벽" — 에서 회피 corridor 만
    편집 맵 쪽에 남아 있었다.

    측정 방식과 안전장치
    --------------------
    * 좌우 **최단거리** (compute_origin_bounds 와 동일한 `_nearest_bound`).
      레이캐스팅은 쓰지 않는다 — 이유는 모듈 docstring 참조.
      검증: 이 방식으로 **편집** 맵을 재면 json 의 width 와 중앙 3.3~4.3 cm 차이로
      일치한다 (그 차이가 safety_width 여유). 즉 측정 기준이 서로 같다.
    * **넓히기만 한다** (`max(json, origin)`). 좁히면 alpha_m 이 폭 밖으로 나가
      raceline_index 가 음수가 되어 노드맵이 깨진다. 원본 자유공간은 편집 자유공간의
      상위집합이므로 정상적으로는 넓어지기만 하지만, 측정 오차 대비 방어로 둔다.
    * `max_widen` 으로 증가분 상한을 둔다. 편집으로 막아 둔 곳이 실제로는 넓은 홀과
      이어져 있으면(문간 등) 노드가 홀 안쪽까지 뻗을 수 있다. 상한이 그걸 막는다.
    * 원본 맵에서 기준점 자체가 벽 안이면 그 점은 손대지 않는다.

    한계 — 판단해서 쓸 것
    ---------------------
    원본 맵은 라이다가 본 벽만 담고 있다. 트랙 경계가 벽이 아니라 테이프/콘으로
    정의돼 있다면 원본 맵에는 그 경계가 없고, 회피 중 차가 그 밖으로 나갈 수 있다.
    "회피할 때는 트랙 밖으로 나가도 된다" 가 아니라면 `max_widen` 을 작게 잡거나
    기능을 끄는 게 맞다 (`ltpl_origin_widths:=false`).

    Args:
        map_dir:    맵 디렉토리 (`<base>.png`, `<base>_origin.png`, `<base>.yaml`)
        ltpl_wpnts: 편집 맵 기준 ltpl 웨이포인트 (f110_msgs/LtplWpntArray)
        logger:     rclpy logger (선택)
        max_ray:    벽을 찾는 최대 반경 [m]
        max_widen:  한쪽 폭을 넓힐 수 있는 최대량 [m]
        min_width:  폭 하한 [m]

    Returns:
        (보정된 LtplWpntArray, 사람이 읽을 요약 문자열).
        원본 맵이 없거나 실패하면 (None, 사유) — 호출자는 편집 맵 폭을 그대로 쓰면
        되고, 그게 지금까지의 동작이다.
    """
    def _log(msg):
        if logger is not None:
            logger.info(msg)

    if ltpl_wpnts is None or not ltpl_wpnts.ltplwpnts:
        return None, 'no ltpl waypoints'

    origin_png, map_yaml = find_origin_map(map_dir)
    if origin_png is None:
        return None, ('no *%s.png in %s -> 편집 맵 폭을 그대로 사용'
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

    wpnts = ltpl_wpnts.ltplwpnts
    # 폭은 레이스라인이 아니라 **기준선 위의 점**(x_ref_m, y_ref_m) 에서 잰 값이다.
    xs = np.array([w.x_ref_m for w in wpnts])
    ys = np.array([w.y_ref_m for w in wpnts])
    # normvec 은 진행방향 오른쪽을 가리킨다 (raceline = ref + alpha_m * normvec 이고
    # w_tr_right = w_tr_right_ref - alpha 이므로). 따라서 좌측 법선 = -normvec.
    # 2026-08-21 실측으로도 확인: 이 부호로 편집 맵을 재면 json 폭과 중앙 3.3~4.3 cm
    # 차이인 반면, 반대 부호면 30~47 cm 어긋난다.
    nxs = np.array([-w.x_normvec_m for w in wpnts])
    nys = np.array([-w.y_normvec_m for w in wpnts])

    # 원본 맵에서 기준점 자체가 벽 안이면 거리가 0 으로 무너진다. 그런 점은 건드리지 않는다.
    cols = np.rint((xs - ox) / res).astype(np.int64)
    rows = np.rint((ys - oy) / res).astype(np.int64)
    height, width = free_orig.shape
    inside = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
    on_free = np.zeros(len(wpnts), dtype=bool)
    on_free[inside] = free_orig[rows[inside], cols[inside]]

    orig_l, orig_r = _nearest_bound(free_orig, xs, ys, nxs, nys, res, ox, oy, max_ray)
    json_l = np.array([w.width_left_m for w in wpnts])
    json_r = np.array([w.width_right_m for w in wpnts])

    # 넓히기만 하고, 증가분에 상한을 건다.
    new_l = np.where(on_free, np.minimum(np.maximum(json_l, orig_l), json_l + max_widen), json_l)
    new_r = np.where(on_free, np.minimum(np.maximum(json_r, orig_r), json_r + max_widen), json_r)
    delta_l = new_l - json_l
    delta_r = new_r - json_r

    out = copy.deepcopy(ltpl_wpnts)
    for i, w in enumerate(out.ltplwpnts):
        w.width_left_m = max(min_width, float(new_l[i]))
        w.width_right_m = max(min_width, float(new_r[i]))

    # graph_planner 가 실제로 얻는 것 = 레이어당 노드 수. 요약에 같이 찍는다.
    # (veh_width 0.28 / lat_resolution 0.1 은 offline_params.yaml 의 실사용값)
    n_before = np.floor((json_l + json_r - 0.28) / 0.1)
    n_after = np.floor((new_l + new_r - 0.28) / 0.1)
    n_changed = int(np.count_nonzero((delta_l > 1e-6) | (delta_r > 1e-6)))
    n_capped = int(np.count_nonzero((delta_l >= max_widen - 1e-6)
                                    | (delta_r >= max_widen - 1e-6)))
    summary = ('origin map %s: ltpl 폭 %d/%d 넓힘 '
               '(left %+.3f~%+.3f m, right %+.3f~%+.3f m, 상한걸림 %d, 벽 안 %d) '
               '-> 노드/레이어 중앙 %d->%d, 최소 %d->%d'
               % (os.path.basename(origin_png), n_changed, len(wpnts),
                  delta_l.min(), delta_l.max(), delta_r.min(), delta_r.max(),
                  n_capped, int(np.count_nonzero(~on_free)),
                  int(np.median(n_before)), int(np.median(n_after)),
                  int(n_before.min()), int(n_after.min())))
    _log(summary)
    return out, summary
