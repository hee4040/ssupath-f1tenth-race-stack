# Sensor → EKF 큐 사이즈 = 1

VESC / IMU / LiDAR 센서 데이터가 EKF에 도달하기까지의 **ROS pub/sub depth(및 경로상 내부 홀딩 큐)** 를 **1(keep-latest)** 로 통일했다.  
목적: 지연 누적 최소화, 최신 측정만 사용.

## 데이터 경로 요약

```text
[VESC]
  sensors/core, sensors/imu/raw  ──► vesc_to_odom ──► /odom
       │                                              │
       │                                    odom_to_twist (이미 1)
       │                                              ▼
       │                              /vehicle/twist_with_covariance
       │                                              │
       └─(optional) report_imu ──► /report_imu/fused ──┤
                                                        ▼
[IMU /sensor raw or fused]  ──► gyro_odometer ──► twist_with_covariance ──► EKF
                                                                              ▲
[Livox] /livox/lidar ──► NDT(scanmatcher) ──► /ndt_pose                       │
                         (already sub keep_last 1)                            │
                                      │                                       │
                                      ▼                                       │
                         pose_to_pose_with_cov (이미 1) ──► in_pose_with_covariance
```

## Before → After

| 단계 | 노드 / 위치 | 토픽·큐 | **기존** | **변경** |
|------|-------------|---------|----------|----------|
| VESC 드라이버 | `vesc_driver.cpp` | `sensors/core`, `sensors/imu`, `sensors/imu/raw` pub | **10** | **1** |
| VESC→odom | `vesc_to_odom.cpp` | `odom` pub, `sensors/core` / servo sub | **10** | **1** |
| odom→twist | `odom_to_twist_converter.cpp` | `/odom` sub, `/vehicle/twist...` pub | **1** | **1** (유지) |
| report_imu | `report_imu_node.py` | fused 등 pub | **10** | **1** |
| report_imu | 동상 | VESC/Livox IMU sub | SensorDataQoS depth **5** | QoS depth **1** |
| gyro 입력 | `gyro_odometer_core.cpp` | vehicle twist / imu sub | **1** | **1** (유지) |
| gyro 출력 | 동상 | `twist`, `twist_raw` pub | **10** | **1** |
| gyro 내부 | 동상 | `vehicle_twist_queue_`, `gyro_queue_` | 매칭 전 **무제한 push** | **최대 1** (latest only) |
| gyro cov 출력 | 동상 | `twist_with_covariance` pub | **1** | **1** (유지) |
| Livox ROS pub | `lddc.cpp` ROS2 | `/livox/lidar`, `/livox/imu` | multi **64** / single **256** (`kMinEth*2` / `*8`) | **1** |
| Livox 프레임 큐 | `comm.cpp` `CalculatePacketQueueSize` | 내부 `StoragePacket` 큐 | **10** (freq>10이면 `freq+1`) | **1** |
| NDT 입력 | `scanmatcher_component.cpp` | cloud / imu sub | keep_last **1** | **1** (유지) |
| NDT 출력 | 동상 | `ndt_pose` pub | **10** | **1** |
| pose cov | `pose_to_pose_with_cov.cpp` | sub/pub | **1** | **1** (유지) |
| EKF 입력 | `ekf_localizer.cpp` | pose / twist sub | **1** | **1** (유지) |
| EKF 출력 | 동상 | ekf_pose/odom/twist 등 | **1** | **1** (유지) |
| EKF 내부 age | yaml `pose_smoothing_steps` / `twist_smoothing_steps` | AgedObjectQueue max_age | yaml 이미 **1** (코드 기본값 5 / 2) | **1** 유지 |

### 의도적으로 안 줄인 것 (EKF 센서 경로 밖)

| 항목 | 기존 | 비고 |
|------|------|------|
| VESC 모터/서보 **command** sub/pub | 10 | control 경로 |
| `servo_position_command` pub | 10 | 조향 명령 피드백 |
| NDT `map` / `path` / map_array | 10 또는 1 | 맵 시각화·저장 |
| EKF `/diagnostics` | 10 | 진단 |
| report_imu `/diagnostics` | 10 | 진단 |
| Livox eth 패킷 버퍼 `kMinEthPacketQueueSize=32` | 32 | 드라이버 수신 링; ROS publish depth와 별개 (2^n 제약) |

## 파일별 변경 위치

| 파일 | 요약 |
|------|------|
| `sensors/vesc/vesc_driver/src/vesc_driver.cpp` | core/imu pub `QoS{10}` → `{1}` |
| `sensors/vesc/vesc_ackermann/src/vesc_to_odom.cpp` | odom pub, core/servo sub `10` → `1` |
| `state_estimation/src/gyro_odometer/src/gyro_odometer_core.cpp` | twist pub 10→1, deque max size 1 |
| `state_estimation/src/lidarslam_ros2/scanmatcher/src/scanmatcher_component.cpp` | `ndt_pose` QoS 10→1 |
| `state_estimation/src/livox_ros_driver2/src/lddc.cpp` | ROS2 lidar/imu publisher depth=1 |
| `state_estimation/src/livox_ros_driver2/src/comm/comm.cpp` | packet queue size 계산값=1 |
| `utilities/nodes/report_imu/report_imu/report_imu_node.py` | fused 경로 pub/sub depth=1 |

## 재빌드 (변경 패키지)

```bash
cd /home/misys/forza_ws/race_stack
source /opt/ros/humble/setup.bash
# underlay 가 있으면 install 까지 source 후
colcon build --packages-select \
  vesc_driver vesc_ackermann gyro_odometer scanmatcher livox_ros_driver2 report_imu \
  --symlink-install
```

## 효과

- 중간 노드가 밀린 메시지로 과거 센서값을 EKF에 밀어넣는 경우를 줄인다.
- QoS depth 1은 **최신 샘플 1개만 유지**(가득 차면 오래된 것 drop).
- Livox 내부 eth 링(32)은 패킷 조립용으로 유지; ROS 쪽 publish/consumer는 1.
