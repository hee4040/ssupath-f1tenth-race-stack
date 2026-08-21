#!/usr/bin/env bash
# 물리계수 실측용 bag 녹화 (라이다 제외라 ~0.15 MB/s).
#
# 사용법: base_system 띄운 뒤
#   ~/forza_ws/race_stack/loc_check/record_sysid.sh              # 자동 이름
#   ~/forza_ws/race_stack/loc_check/record_sysid.sh -o sysid_circle_L
#   ~/forza_ws/race_stack/loc_check/record_sysid.sh --lidar      # 라이다 포함(측위 재현용)
#
# 보통은 run_sysid.sh 가 이걸 알아서 부른다. 조이스틱으로 직접 몰면서
# 따로 녹화하고 싶을 때만 단독으로 쓴다.
#
# 왜 이 토픽들인가:
#   /sensors/core   ERPM 실측 + 모터전류 + duty + 배터리전압.
#                   ERPM 실측 vs pose 실측 속도의 차이가 '휠 슬립'이고,
#                   그게 가속이 힘 상한에 걸렸는지 마찰 상한에 걸렸는지를 가른다.
#                   duty 가 1 에 붙으면 역기전력/전압 한계다.
#   /car_state/pose 속도의 유일한 신뢰 소스. /odom 은 ERPM/게인이라 순환 논리.
#   /sensors/imu/raw 요레이트(자이로 z). 횡가속 v*wz 의 wz.
set -e
source /opt/ros/humble/setup.bash
cd ~/forza_ws/race_stack

WITH_LIDAR=0
OUT="sysid_$(date +%m%d_%H%M%S)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --lidar) WITH_LIDAR=1; shift ;;
    -o|--output) OUT="$2"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$(readlink -f "$0")"; exit 0 ;;
    *) echo "알 수 없는 인자: $1"; exit 1 ;;
  esac
done

TOPICS=(
  # 명령
  /drive /commands/motor/speed /commands/servo/position /commands/motor/current
  /sensors/servo_position_command
  # 차량 텔레메트리
  /sensors/core /sensors/imu/raw /sensors/imu
  # 측위 (속도/요레이트의 근거)
  /car_state/pose /car_state/odom /ekf_pose /ekf_odom /ndt_pose /odom
  /tf /tf_static
  # 조이스틱 (데드맨/e-stop 이 언제 걸렸는지 사후 확인)
  /joy
)
[[ $WITH_LIDAR -eq 1 ]] && TOPICS+=(/livox/lidar /livox/imu)

echo "[record_sysid] 저장: $(pwd)/${OUT}  (토픽 ${#TOPICS[@]}개)"
exec ros2 bag record -o "$OUT" "${TOPICS[@]}"
