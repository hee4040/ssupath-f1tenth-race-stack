#!/usr/bin/env bash
# 장애물 탐지/회피 점검용 bag 녹화.
#
# 사용법: head_to_head 스택 띄운 뒤, 주행 시작 전에 실행
#   ~/forza_ws/race_stack/loc_check/record_obs.sh              # 라이다 포함(기본, ~8MB/s)
#   ~/forza_ws/race_stack/loc_check/record_obs.sh --no-lidar   # 라이다 제외(~0.3MB/s)
#   ~/forza_ws/race_stack/loc_check/record_obs.sh -o my_run    # 이름 지정
# bag은 ~/forza_ws/race_stack/obs_debug_MMDD_HHMM 으로 저장됨.
#
# 목적: 장애물이 "탐지 체인의 어느 단에서 사라지는지"를 사후에 특정하는 것.
#   /clusters                            euclidean_cluster 출력 (클러스터링까지는 됐나?)
#     -> /perception/detection/raw_obstacles   cluster_to_obstacle 출력
#        여기서 없으면: max_viewing_distance / laserPointOnTrack 경계필터 / min_obs_size / max_obs_size
#     -> /perception/obstacles                 tracking 출력 (상태머신이 실제로 보는 것)
#        여기서 없으면: max_dist(연관 게이트) / nb_meas<=6 / dist_infront / dist_deletion
#     -> /planner/avoidance/otwpnts            graph_planner 회피선 (비어 있으면 회피 경로 실패)
#     -> /state_machine/state                  최종 판단 (GB_TRACK/TRAILING/OVERTAKE)
# 2026-08-04 기준 고속 회피 실패는 tracking 단에서 끊겼다. pose 토픽을 같이 담는 이유는
# 그 원인이 속도 비례 측위 지연(NDT-EKF 괴리 4m/s에서 0.84m > max_dist 0.5m)이었기 때문.
#
# 주의: /global_waypoints 와 /perception/detect_bound 는 latched(transient_local) 라
#   스택보다 늦게 시작해도 보통 잡히지만, 확실히 하려면 스택 직후에 실행할 것.
#   재생 시 Frenet 좌표 해석에 /global_waypoints 가 반드시 필요하다.

set -e
SELF="$(readlink -f "$0")"   # 아래 cd 때문에 $0 상대경로가 깨지므로 먼저 절대경로로 고정
source /opt/ros/humble/setup.bash
cd ~/forza_ws/race_stack

WITH_LIDAR=1
OUT="obs_debug_$(date +%m%d_%H%M)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-lidar) WITH_LIDAR=0; shift ;;
    -o|--output) OUT="$2"; shift 2 ;;
    -h|--help)  sed -n '2,23p' "$SELF"; exit 0 ;;
    *) echo "알 수 없는 인자: $1 (사용법은 --help)"; exit 1 ;;
  esac
done

# --- 퍼셉션 체인 (단계별로 어디서 끊기는지 보려고 전 단계를 다 담는다) ---
TOPICS=(
  /clusters
  /perception/detection/raw_obstacles
  /perception/detection/obstacles_markers
  /perception/detect_bound
  /perception/raw_obstacles
  /perception/obstacles
  /perception/static_dynamic_marker_pub
  /perception/tracking/latency
)

# --- 플래너 / 상태머신 ---
TOPICS+=(
  /planner/avoidance/otwpnts
  /planner/avoidance/markers
  /planner/avoidance/propagated_obs
  /graph_planner/avoidance/latency
  # state_machine 은 상대 토픽명('state' 등)을 쓰는데 네임스페이스가 루트라
  # 노드 이름이 아니라 '/state', '/local_waypoints' 로 해석된다.
  # (l1_controller.cpp 가 "/state", "/local_waypoints" 를 구독하는 것으로 확인)
  # 0804_1612 녹화 때 '/state_machine/...' 로 적어서 상태머신 토픽이 하나도 안 담겼다.
  /state
  /state_marker
  /local_waypoints
  /local_waypoints/markers
  /ot_section_check
)

# --- 기준 경로 (재생 시 Frenet 해석에 필수) ---
TOPICS+=(
  /global_waypoints
  /global_waypoints_scaled
)

# --- 자차 상태 / 측위 (고속 실패 원인 규명에 필요) ---
TOPICS+=(
  /car_state/odom
  /car_state/pose
  /car_state/frenet/odom
  /ndt_pose
  /ekf_pose
  /ekf_odom
  /odom
  /tf
  /tf_static
)

# --- 제어 출력 ---
TOPICS+=(
  /drive
  /scan
)

if [[ $WITH_LIDAR -eq 1 ]]; then
  # 오프라인으로 클러스터링부터 다시 돌려야 할 때 필요. 없으면 raw_obstacles 이전 단은 못 판다.
  TOPICS+=(/livox/lidar)
  echo "[record_obs] 라이다 포함 (~8MB/s). 제외하려면 --no-lidar"
else
  echo "[record_obs] 라이다 제외 — /clusters 이전 단계는 사후 재현 불가"
fi

echo "[record_obs] 저장 위치: $(pwd)/${OUT}"
echo "[record_obs] 토픽 ${#TOPICS[@]}개. Ctrl-C 로 종료."

exec ros2 bag record -o "$OUT" "${TOPICS[@]}"
