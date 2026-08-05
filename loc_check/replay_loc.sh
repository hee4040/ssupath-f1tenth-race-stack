#!/usr/bin/env bash
# 녹화한 bag을 rviz2로 재생해서 측위 품질을 눈으로 확인.
# 사용법: ~/forza_ws/race_stack/loc_check/replay_loc.sh <bag폴더> [재생속도=1.0]
# 예:     ~/forza_ws/race_stack/loc_check/replay_loc.sh ~/forza_ws/race_stack/loc_debug_0715_1349 0.5
#
# 보는 법: 주황(라이다 스캔)이 흰 맵 벽에 붙어 다니면 측위 정상.
# 빨강 화살표(/ndt_pose)와 파랑 화살표(/car_state/pose)가 벌어지면 EKF 지연/폐기.
# 재생 중 스페이스바 = 일시정지/재개.
set -e
BAG=${1:?사용법: replay_loc.sh <bag폴더> [재생속도=1.0]}
RATE=${2:-1.0}
DIR=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash

# ros2 run 래퍼만 죽이면 실제 노드가 고아로 살아남아 다음 주행 녹화를 방해함
# (5.8MB /map을 2초마다 발행 -> 레코더 /tf 유실 사고 원인).
# setsid로 프로세스 그룹을 분리해 그룹째 죽이고, 패턴 kill로 한 번 더 보강.
PGIDS=()
cleanup() {
  for pg in "${PGIDS[@]}"; do kill -TERM -- -"$pg" 2>/dev/null || true; done
  sleep 0.5
  pkill -f "pcl_ros/pcd_to_pointcloud" 2>/dev/null || true
  pkill -f "rviz2 -d $DIR/replay.rviz" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 트랙 맵을 pcd에서 직접 발행 (bag에 /map이 없어도 표시되도록 2초마다 재발행)
setsid ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:="$HOME/forza_ws/race_stack/map.pcd" \
  -p tf_frame:=map -p interval:=2.0 \
  -r /cloud_pcd:=/map >/dev/null 2>&1 &
PGIDS+=($!)

setsid rviz2 -d "$DIR/replay.rviz" --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
PGIDS+=($!)

sleep 3
ros2 bag play "$BAG" --clock --rate "$RATE"
