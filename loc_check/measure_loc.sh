#!/usr/bin/env bash
# 주행 중 VGICP 측위 지연 실측.
#
# 사용법: base_system + time_trials 를 띄운 뒤, 주행 시작 전에 세 번째 터미널에서 실행.
#   1) ros2 launch stack_master base_system_3D_launch.xml racecar_version:=NUC2 \
#        map_dir:=lobby_0723 map_name:=lobby_0723 sim:=false rviz:=true
#   2) ros2 launch stack_master time_trials_launch.xml racecar_version:=NUC2 ctrl_algo:=PP
#   3) ~/forza_ws/race_stack/loc_check/measure_loc.sh          <-- 이 스크립트
#
# 주행이 끝나면 Ctrl+C. 최종 요약이 출력되고 CSV가 저장된다.
#
# 보는 법:
#   - p95 지연이 목표(150ms) 아래면 양호. 과거 진단에선 평시 150ms인데 고속에서만 튀었으므로
#     '속도 구간별' p95 를 반드시 같이 볼 것 (3~4 m/s 구간이 핵심).
#   - '폐기된 pose 측정치'가 0이 아니면 EKF가 스캔을 버리고 추측항법으로 흐른 것 = 위치오차 누적.
#   - ndt vs ekf 위치차가 벌어지면 EKF가 스캔을 못 따라가고 있다는 신호.
#
# 옵션 예:
#   measure_loc.sh --target-ms 120          목표 p95 변경
#   measure_loc.sh --no-csv                 CSV 저장 안 함
#   measure_loc.sh --csv /tmp/run1.csv      CSV 경로 지정
set -e

DIR=$(cd "$(dirname "$0")" && pwd)

# 표준 메시지(geometry_msgs/nav_msgs/diagnostic_msgs)만 쓰므로 오버레이는 필요 없다.
# ROS_DOMAIN_ID 는 현재 셸 환경을 그대로 물려받는다 (측위 노드들과 같아야 함).
source /opt/ros/humble/setup.bash

# -u : 파이프(tee 등)로 넘길 때도 진행 상황이 실시간으로 보이도록 버퍼링 끔
exec python3 -u "$DIR/loc_latency_node.py" "$@"
