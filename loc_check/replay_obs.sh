#!/usr/bin/env bash
# 녹화한 obs_debug bag 을 rviz2 로 재생해서 탐지·회피를 눈으로 다시 본다.
# ★ base_system / head_to_head 를 띄우지 않으므로 차는 움직이지 않는다.
#   (bag 에 담긴 마커를 그대로 다시 그릴 뿐이다)
#
# 사용법:
#   ~/forza_ws/race_stack/loc_check/replay_obs.sh <bag폴더> [재생속도=1.0] [시작초=0]
# 예:
#   ~/forza_ws/race_stack/loc_check/replay_obs.sh ~/forza_ws/race_stack/obs_debug_0804_2103
#   ~/forza_ws/race_stack/loc_check/replay_obs.sh ~/forza_ws/race_stack/obs_debug_0804_2103 0.3 180
#
# 재생 중 조작: 스페이스바 = 일시정지/재개
#
# 보는 법 (이 순서로 켜지면 정상):
#   초록 점(scan) -> 1) 빨간 네모(원시 탐지) -> 2) 구(추적: 초록=정적/자홍=미정/빨강=동적)
#   -> 상태등(초록 GB_TRACK -> 파랑 TRAILING -> 흰색 OVERTAKE)
#   -> 4) 연보라 회피선 -> 3) 초록 로컬경로가 옆으로 벌어짐
#   3) 초록 점의 '높이'는 명령 속도다. 감속하면 바닥으로 가라앉는다.
#
# bag 에 없어서 안 보이는 것:
#   /map                     -> 이 스크립트가 map.pcd 에서 직접 발행한다
#   /global_waypoints/markers, /trackbounds/markers -> 레이싱라인·트랙경계선은 안 나온다
#   /ground_segmentation/lidar -> 'Cluster input' 대신 Raw LiDAR(기본 off)를 켜서 볼 것
set -e
BAG=${1:?사용법: replay_obs.sh <bag폴더> [재생속도=1.0] [시작초=0]}
RATE=${2:-1.0}
OFFSET=${3:-0}
DIR=$(cd "$(dirname "$0")" && pwd)
MAP_PCD=${MAP_PCD:-$HOME/forza_ws/race_stack/map.pcd}
source /opt/ros/humble/setup.bash
[ -f "$HOME/forza_ws/race_stack/install/setup.bash" ] && \
  source "$HOME/forza_ws/race_stack/install/setup.bash"

[ -d "$BAG" ] || { echo "bag 폴더가 없다: $BAG"; exit 1; }

# ros2 run 래퍼만 죽이면 실제 노드가 고아로 남아 다음 녹화를 방해한다
# (5.8MB /map 을 2초마다 발행 -> 레코더 /tf 유실 사고 전례).
# setsid 로 프로세스 그룹을 분리해 그룹째 죽이고, 패턴 kill 로 한 번 더 보강한다.
PGIDS=()
cleanup() {
  for pg in "${PGIDS[@]}"; do kill -TERM -- -"$pg" 2>/dev/null || true; done
  sleep 0.5
  pkill -f "pcl_ros/pcd_to_pointcloud" 2>/dev/null || true
  pkill -f "rviz2 -d $DIR/replay_obs.rviz" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 트랙 맵 발행 (bag 에 /map 이 없다)
if [ -f "$MAP_PCD" ]; then
  setsid ros2 run pcl_ros pcd_to_pointcloud --ros-args \
    -p file_name:="$MAP_PCD" -p tf_frame:=map -p interval:=2.0 \
    -r /cloud_pcd:=/map >/dev/null 2>&1 &
  PGIDS+=($!)
else
  echo "[경고] map.pcd 없음($MAP_PCD) — 맵 없이 재생한다. MAP_PCD=... 로 지정 가능"
fi

setsid rviz2 -d "$DIR/replay_obs.rviz" --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
PGIDS+=($!)

echo "[replay_obs] bag   : $BAG"
echo "[replay_obs] 속도  : ${RATE}x   시작: ${OFFSET}s"
echo "[replay_obs] rviz 가 뜰 때까지 3초 대기... (스페이스바로 일시정지)"
sleep 3

exec ros2 bag play "$BAG" --clock --rate "$RATE" --start-offset "$OFFSET"
