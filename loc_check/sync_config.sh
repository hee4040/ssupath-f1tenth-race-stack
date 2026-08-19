#!/usr/bin/env bash
# src 의 config yaml 을 install 로 동기화한다.
#
# 왜 필요한가: 이 워크스페이스는 colcon --symlink-install 이 아니라서
#   install/ 아래 사본이 진짜 복사본이다. 런치는 find-pkg-share(=install)를 읽으므로
#   src 만 고치면 반영되지 않는다. (lidarslam 처럼 심볼릭 링크인 패키지도 있으니
#   새 파일을 다룰 땐 ls -la 로 확인할 것.)
#
# 사용법:
#   ~/forza_ws/race_stack/loc_check/sync_config.sh          # 어긋난 것만 복사
#   ~/forza_ws/race_stack/loc_check/sync_config.sh --check  # 확인만, 복사 안 함
#
# yaml 은 컴파일 대상이 아니라 복사만으로 충분하다. .cpp 를 고쳤다면 이게 아니라
#   MAKEFLAGS=-j2 colcon build --packages-select <pkg>
# 를 해야 한다.
#
# 복사 후에는 노드를 재시작해야 값이 먹는다. 단, dynamic param 은 주행 중에도 가능:
#   ros2 param set /state_machine hs_brake_factor 0.35
#   ros2 param set /graph_planner  safety_margin_hi 0.20
# (min_cluster_size / boundaries_inflation / max_viewing_distance 는 콜백이 없어
#  반드시 재시작해야 한다.)

set -u
cd ~/forza_ws/race_stack || exit 1
CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

# "src파일:install목적지디렉터리" 목록
PAIRS=(
  "stack_master/config:install/stack_master/share/stack_master/config"
  "perception/src/clustering/cluster_to_obstacle_cpp/config:install/cluster_to_obstacle_cpp/share/cluster_to_obstacle_cpp/config"
  "perception/src/clustering/autoware_euclidean_cluster/config:install/autoware_euclidean_cluster/share/autoware_euclidean_cluster/config"
  "perception/src/preprocessing/autoware_pointcloud_preprocessor/config:install/autoware_pointcloud_preprocessor/share/autoware_pointcloud_preprocessor/config"
  "perception/src/preprocessing/autoware_ground_segmentation/config:install/autoware_ground_segmentation/share/autoware_ground_segmentation/config"
)

n_diff=0; n_copy=0; n_skip=0
for pair in "${PAIRS[@]}"; do
  SRCD="${pair%%:*}"; DSTD="${pair##*:}"
  [ -d "$SRCD" ] || continue
  [ -d "$DSTD" ] || { echo "  (install 없음, 건너뜀) $DSTD"; continue; }
  for s in "$SRCD"/*.yaml; do
    [ -f "$s" ] || continue
    d="$DSTD/$(basename "$s")"
    if [ -L "$d" ]; then
      n_skip=$((n_skip+1)); continue          # 심볼릭 링크는 손댈 필요 없음
    fi
    if [ ! -f "$d" ] || ! diff -q "$s" "$d" >/dev/null 2>&1; then
      n_diff=$((n_diff+1))
      if [ $CHECK_ONLY -eq 1 ]; then
        echo "  [다름] $s"
      else
        cp "$s" "$d" && { echo "  [복사] $(basename "$s")  ->  $DSTD"; n_copy=$((n_copy+1)); }
      fi
    fi
  done
done

echo
if [ $CHECK_ONLY -eq 1 ]; then
  [ $n_diff -eq 0 ] && echo "전부 동기화돼 있음." || echo "어긋난 파일 ${n_diff}개. 인자 없이 다시 실행하면 복사한다."
else
  [ $n_copy -eq 0 ] && echo "복사할 것 없음 (이미 동기화됨)." || echo "${n_copy}개 복사 완료. ★ 노드를 재시작해야 적용된다."
fi
[ $n_skip -gt 0 ] && echo "(심볼릭 링크 ${n_skip}개는 건너뜀 — 복사 불필요)"
exit 0
