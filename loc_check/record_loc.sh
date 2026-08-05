#!/usr/bin/env bash
# 측위 점검용 bag 녹화.
# 사용법: base_system 띄운 뒤, 주행 시작 전에 실행
#   ~/forza_ws/race_stack/loc_check/record_loc.sh
# bag은 ~/forza_ws/race_stack/loc_debug_MMDD_HHMM 으로 저장됨.
set -e
source /opt/ros/humble/setup.bash
cd ~/forza_ws/race_stack

exec ros2 bag record -o loc_debug_$(date +%m%d_%H%M) \
  /tf /tf_static /map \
  /livox/lidar /sensors/imu/raw /odom \
  /vehicle/twist_with_covariance /gyro_twist_with_covariance \
  /ndt_pose /ndt_pose_with_covariance \
  /ekf_pose /ekf_pose_with_covariance /ekf_odom /ekf_twist /estimated_yaw_bias \
  /car_state/odom /car_state/pose /car_state/frenet/odom \
  /path /diagnostics /drive /initialpose
