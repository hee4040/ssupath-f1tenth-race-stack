# Total_CodeChange — 3d_timestamp_relay / IMUmux **단독 재현 명세서**

> **이 파일 하나만** 다른 컨테이너·머신으로 가져가도,  
> 동일 스택(ROS 2 Humble + race_stack 계열 측위) 위에서 **기능 재현**과 **코드 변경**이 가능하도록 작성함.  
> (`IMUmux.md` 등 다른 plusresult 문서는 **필수가 아님**.)

| 항목 | 값 |
|------|-----|
| 대상 기능 | (1) 센서 stamp 릴레이 + 4종 delay CSV (2) IMUmux (3) Livox IMU 50 Hz |
| 기준 git (있을 때) | 브랜치 **IFAC**, 기능 커밋 `862ff9e` + **weighted `weight_vesc` only** |
| 문서·스펙 갱신 | **2026-08-11**: Livox CSV bias **0.01**; IMUmux CSV에 **vesc/livox header.stamp** 열 추가 |
| 가정 워크스페이스 | `…/race_stack` (경로 하드코딩 시 `/home/misys/forza_ws/race_stack` — **자기 경로로 치환**) |
| 빌드 | `colcon build` + `source install/setup.bash` — 최신 변경 빌드 패키지는 **§R0.3.1** |

---

# R0. 다른 컨테이너에서 재현하는 전체 절차

## R0.1 전제

- ROS 2 **Humble**, 워크스페이스에 최소한 아래 패키지 **소스가 있음**:
  - `livox_ros_driver2`
  - `scanmatcher` (패키지명 예: lidarslam 내 scanmatcher)
  - `pose_to_pose_with_cov`
  - `vesc_driver`, `vesc_ackermann`
  - `odom_to_twist_converter`
  - `gyro_odometer`
  - `ekf_localizer`
  - `stack_master` (launch 진입점)

- 베이스 = Autoware 계열 gyro+EKF / F1TENTH VESC / Livox MID360.  
  IFAC 커밋 diff가 있으면 `git cherry-pick`·`git show 862ff9e` 로 맞춰도 되고, **없으면 본 문서 §I 스펙대로 직접 패치**.

## R0.2 구현 순서 (권장)

| 단계 | 작업 | 완료 기준 |
|------|------|-----------|
| 1 | §I-A stamp 정의 (Livox eth mean, VESC now) | `/livox/lidar`·`/sensors/core` stamp 의미 일치 |
| 2 | §I-B stamp 릴레이 (NDT → pose_cov → odom → twist) | 중간 노드가 stamp를 now()로 덮지 않음 |
| 3 | §I-C gyro max stamp + pre-max delay 토픽 | `/timestamp_relay_3d/imu_vesc_delay` 발행 |
| 4 | §I-D EKF delay CSV + output stamp max | `3d_timestamp_relay_*.csv` |
| 5 | §I-E IMUmux + launch CLI | `imu_mode:=… weight_vesc:=…` |
| 6 | §I-F Livox IMU 50 Hz | `ros2 topic hz /livox/imu` ≈ 50 |
| 7 | §R1 빌드·검증 | 아래 체크리스트 |

## R0.3 빌드 (최소)

```bash
# WORKSPACE 를 자기 경로로 변경
export WORKSPACE=/home/misys/forza_ws/race_stack
cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
# 기존 install 이 있으면: source install/setup.bash

colcon build --packages-select \
  livox_ros_driver2 \
  gyro_odometer \
  ekf_localizer \
  stack_master \
  odom_to_twist_converter \
  pose_to_pose_with_cov \
  vesc_driver vesc_ackermann
# scanmatcher / lidarslam 패키지명은 워크스페이스에 맞게 추가

source install/setup.bash
```

### R0.3.1 2026-08-11 변경 후 **반드시 빌드할 패키지**

코드: Livox CSV bias 기본값 **0.01**, IMUmux CSV 열 `vesc_header_stamp` / `livox_header_stamp`, launch default 동기화.

| 패키지 | 이유 |
|--------|------|
| **`gyro_odometer`** | bias 기본값, CSV 스키마 (필수) |
| **`stack_master`** | `base_system_3D_launch.xml` 의 `imu_mux_livox_bias_wz` default |
| **`ekf_localizer`** | `full_localization.launch.xml` arg default 전달 |

```bash
cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash   # 기존 install 있으면
colcon build --packages-select gyro_odometer stack_master ekf_localizer
source install/setup.bash
```

> PCD stamp / EKF delay / Livox 50 Hz 를 건드리지 **않은** 증분 변경이면 위 3개만 충분.  
> 전체 기능 첫 재현이면 §R0.3 전체 목록.

## R0.4 런치 (재현용 예시)

```bash
ros2 launch stack_master base_system_3D_launch.xml \
  map_name:=YOUR_MAP sim:=false \
  imu_mode:=vesc

# IMUmux weighted (VESC 70% / Livox 30% 자동)
ros2 launch stack_master base_system_3D_launch.xml \
  map_name:=YOUR_MAP sim:=false \
  imu_mode:=weighted weight_vesc:=70

# VESC 운행 + 양쪽 CSV 비교
ros2 launch stack_master base_system_3D_launch.xml \
  map_name:=YOUR_MAP sim:=false \
  imu_mode:=test
```

## R0.5 검증 체크리스트 (Acceptance)

```bash
# rates
ros2 topic hz /livox/imu          # ≈50
ros2 topic hz /sensors/imu/raw    # ≈50
ros2 topic hz /timestamp_relay_3d/imu_vesc_delay

# stamp 의미 (3d_loc_debug=true 일 때 상대 비교)
ros2 topic echo /livox/lidar --field header.stamp --once
ros2 topic echo /ndt_pose --field header.stamp --once
ros2 topic echo /vehicle/twist_with_covariance --field header.stamp --once
ros2 topic echo /gyro_twist_with_covariance --field header.stamp --once

# params
ros2 param get /gyro_odometer imu_mux_mode
ros2 param get /gyro_odometer imu_mux_weight_vesc
ros2 param get /ekf_localizer 3d_loc_debug

# logs (경로 = sensor_delay_csv_dir / imu_mux_csv_dir)
ls "$WORKSPACE/plusresult"/3d_timestamp_relay_*.csv
ls "$WORKSPACE/plusresult"/*_imu_*.csv
```

**PASS 조건 (요약)**

1. `/gyro_twist_with_covariance.header.stamp == max(vehicle_stamp, imu_stamp)` (개념)  
2. EKF 출력 stamp: `3d_loc_debug=true` 이면 `max(last_pose_sensor, last_twist_sensor)`  
3. delay CSV 열 9개, delay_*는 **이벤트 시점** 값 (쓰기 때 재계산 금지)  
4. weighted: `weight_vesc=70` → 실효 가중 0.7 / 0.3; Livox 파라미터 입력 불필요  
5. test: feed는 VESC만, CSV에 livox_wz = raw−**0.01**; **vesc_header_stamp / livox_header_stamp** 기록  
6. `/livox/imu` ≈ 50 Hz, PCD 경로 불변  

---

# R1. 설계 원칙 (변경 시 깨면 안 됨)

| ID | 원칙 | 구현 함의 |
|----|------|-----------|
| P1 | 센서 stamp **정의 지점 1곳** | Livox eth 수신 mean / VESC 패킷 수신 `now()` |
| P2 | 중간 노드 **덮어쓰기 금지** | NDT, pose_cov, odom, odom_to_twist |
| P3 | 융합 출력 stamp = **늦은 입력** | gyro `max(vehicle,imu)` |
| P4 | delay = `now−header` **이벤트 샘플** | EKF 출력 max 후 재-age 금지 |
| P5 | imu/vesc delay는 gyro **max 직전** | `/timestamp_relay_3d/imu_vesc_delay` → EKF latch |
| P6 | NDT↔IMU 고정 α 없음 | EKF는 R/Q; IMUmux는 모드·weight_vesc |
| P7 | Livox bias는 **CSV만** | feed raw; `livox_wz` 열만 −**0.01** (param `imu_mux_livox_bias_wz`) |

---

# R2. 엔드-투-엔드 데이터 흐름 (목표 구조)

```text
[LiDAR]
 eth 패킷 now() ──mean──► /livox/lidar.stamp
        → NDT: sensor_input_stamp_ 를 pose.stamp 에 복사 (now() 금지)
        → pose_to_pose_with_cov: out.stamp = in.stamp
        → EKF pose 업데이트 후: delay_lidar 샘플, last_pose_sensor_stamp_

[VESC wheel]
 Values: now() → /sensors/core.stamp
        → /odom.stamp (릴레이)
        → /vehicle/twist_with_covariance.stamp (릴레이)
        → gyro vehicle_queue

[IMU mux]
 /sensors/imu/raw ──┐
 /livox/imu ────────┤ processImuMux → selected Imu → callbackImu → gyro_queue
                    │ (모드: vesc|livox|weighted|test)

[gyro fuse]
 ① sampleImuVescDelayBeforeMax → /timestamp_relay_3d/imu_vesc_delay  [4 doubles]
 ② stamp = max(vehicle, imu) → /gyro_twist_with_covariance
 ③ EKF twist; delay_gyro_twist; getOutputStamp()
 ④ CSV 3d_timestamp_relay_*.csv
```

**EKF timer ~40 Hz 순서 (필수)**

1. predict  
2. pose 큐 → measurementUpdatePose → `last_pose_sensor_stamp_`  
3. **sampleSensorDelay("lidar")**  
4. twist 큐 → measurementUpdateTwist → `last_twist_sensor_stamp_`  
5. **sampleSensorDelay("gyro_twist")**  
6. getOutputStamp() — **여기서 delay 재계산 금지**  
7. publish  

---

# CSV. 저장 컬럼 명세 (필수 참고)

두 종류의 CSV가 나간다. **경로·파일명·열 이름·단위**를 이 절이 정의한다.

| 종류 | 노드 | 파일명 패턴 | 기본 디렉터리 param |
|------|------|-------------|---------------------|
| **IMUmux** | `/gyro_odometer` | `{imu_mux_mode}_imu_YYYYMMDD_HHMMSS.csv` | `imu_mux_csv_dir` (기본 `$WORKSPACE/plusresult`) |
| **3d_timestamp_relay** | `/ekf_localizer` | `3d_timestamp_relay_YYYYMMDD_HHMMSS.csv` | `sensor_delay_csv_dir` (동일 기본) |

`imu_mode:=test` → 예: `test_imu_20260811_133000.csv`

---

## CSV-A. IMUmux — `{mode}_imu_*.csv`

### A.1 한 줄 헤더 (현재 코드, 열 **21개**)

```text
ros_time,msg_stamp,selected_mode,selected_wz,vesc_wz,livox_wz,livox_wz_raw,livox_bias_wz,vesc_weight,livox_weight,vesc_valid,livox_valid,fallback_used,vesc_header_stamp,livox_header_stamp,angular_velocity_x,angular_velocity_y,angular_velocity_z,linear_acceleration_x,linear_acceleration_y,linear_acceleration_z
```

### A.2 열 정의

| # | 열 이름 | 단위/타입 | 정의 |
|---|---------|-----------|------|
| 1 | `ros_time` | s (float) | 행 기록 `gyro_odometer` 의 `this->now()` |
| 2 | `msg_stamp` | s | selected Imu (로그용) `header.stamp` |
| 3 | `selected_mode` | string | `vesc` / `livox` / `weighted` / `test` |
| 4 | `selected_wz` | rad/s | gyro feed 대상 yaw rate (output_frame, TF 후). **test=vesc** |
| 5 | `vesc_wz` | rad/s | VESC IMU TF 후 \(w_z\); invalid → NaN |
| 6 | **`livox_wz`** | rad/s | **Livox TF 후 \(w_z\) − `livox_bias_wz`** (비교용 de-bias). feed 아님 |
| 7 | `livox_wz_raw` | rad/s | Livox TF 후 raw \(w_z\) (= livox_wz + bias when valid) |
| 8 | **`livox_bias_wz`** | rad/s | 상수. 기본 **0.01** (`imu_mux_livox_bias_wz`) |
| 9 | `vesc_weight` | — | 적용 VESC 가중 [0,1] |
| 10 | `livox_weight` | — | 적용 Livox 가중 [0,1] (weighted 양 valid 시 합=1) |
| 11 | `vesc_valid` | 0/1 | fresh + TF 성공 |
| 12 | `livox_valid` | 0/1 | 동일 |
| 13 | `fallback_used` | 0/1 | weighted 에서 한쪽-only |
| 14 | **`vesc_header_stamp`** | s | **`/sensors/imu/raw` 센싱 `header.stamp`** (캐시 없으면 NaN) |
| 15 | **`livox_header_stamp`** | s | **`/livox/imu` 센싱 `header.stamp`** (캐시 없으면 NaN) |
| 16–18 | `angular_velocity_{x,y,z}` | rad/s | CSV 로그용 최종 벡터 (output_frame 계열) |
| 19–21 | `linear_acceleration_{x,y,z}` | m/s² | 동일 |

### A.3 계산 식 (test / 비교 시)

```text
livox_wz_raw = ang_livox.z                    # TF 후
livox_wz     = ang_livox.z - imu_mux_livox_bias_wz   # default bias = 0.01
selected_wz  = ang_vesc.z                     # test 모드 (feed = VESC raw, bias 미적용)
```

### A.4 모드별 특징

| mode | feed gyro | CSV livox 열 | stamp 열 |
|------|-----------|--------------|---------|
| vesc | VESC | livox 보통 invalid/NaN | 캐시 있으면 stamp 기록 |
| livox | Livox raw | debias 기록; feed는 raw | 둘 다 |
| weighted | blend | debias | 둘 다 |
| **test** | **VESC only** | **debias 0.01** + raw | **둘 다 기록 (핵심)** |

---

## CSV-B. EKF delay — `3d_timestamp_relay_*.csv`

### B.1 한 줄 헤더 (열 **9개**)

```text
t_now_sec,delay_imu_s,delay_vesc_s,delay_lidar_s,delay_gyro_twist_s,stamp_imu_sec,stamp_vesc_sec,stamp_lidar_sec,stamp_gyro_twist_sec
```

### B.2 열 정의

| # | 열 이름 | 단위 | 정의 |
|---|---------|------|------|
| 1 | `t_now_sec` | s | 행 쓰기 시각 EKF `now()` (**delay 재계산 아님**) |
| 2 | `delay_imu_s` | s | gyro pre-max 샘플: `now − selected_imu.stamp` (latch) |
| 3 | `delay_vesc_s` | s | gyro pre-max: `now − vehicle_twist.stamp` |
| 4 | `delay_lidar_s` | s | EKF pose 직후: `now − ndt_pose.stamp` |
| 5 | `delay_gyro_twist_s` | s | EKF twist 직후: `now − gyro_twist.stamp` |
| 6 | `stamp_imu_sec` | s | multiarray 로부터 복원된 IMU header.stamp |
| 7 | `stamp_vesc_sec` | s | vehicle twist header.stamp |
| 8 | `stamp_lidar_sec` | s | last pose sensor stamp |
| 9 | `stamp_gyro_twist_sec` | s | last gyro twist (max vehicle/imu) stamp |

미수신·미latch: delay/stamp 열 **NaN**.

### B.3 토픽

Gyro → EKF: `/timestamp_relay_3d/imu_vesc_delay`  
`Float64MultiArray.data = [delay_imu_s, delay_vesc_s, stamp_imu_sec, stamp_vesc_sec]`

---

## CSV-C. 검증 스니펫

```bash
# IMUmux 헤더 확인 (bias=0.01, stamp 열 존재)
head -1 "$WORKSPACE/plusresult"/test_imu_*.csv | tr ',' '\n' | nl
# 기대: vesc_header_stamp, livox_header_stamp, livox_bias_wz 값 0.010...

# delay CSV
head -1 "$WORKSPACE/plusresult"/3d_timestamp_relay_*.csv
```


# I. 구현 스펙 (파일별 — 코드 변경 지침)

아래 경로는 워크스페이스 루트 기준 상대 경로. 이름은 환경에 따라 약간 다를 수 있으나 **역할은 동일**.

---

## I-A. Livox PCD stamp = eth 수신 mean (`3d_loc_debug`)

### 파일

| 경로 | 역할 |
|------|------|
| `state_estimation/src/livox_ros_driver2/src/comm/pub_handler.*` | 패킷마다 `receive_time_ns = now()`; 프레임 mean |
| `…/comm/comm.h`, `ldq.*`, `lds.*` | `mean_receive_time_ns` 필드 전파 |
| `…/lddc.cpp`, `lddc.h` | `ResolveFrameHeaderStampNs` |
| `…/livox_ros_driver2.cpp` + MID360 launch | `3d_loc_debug` 파라미터 |

### 필수 동작

1. 이더넷/USB 패킷 수신 콜백에서 **호스트 ROS 시각(ns)** 기록.  
2. PCD 프레임 조립 시 포함 패킷들의 수신 시각 **산술 평균** → `pkg.mean_receive_time_ns`.  
3. 헤더 stamp 결정:

```cpp
// 의사코드 — lddc
uint64_t MeanPacketReceiveTimeNs(const StoragePacket& pkg) {
  if (pkg.mean_receive_time_ns != 0) return pkg.mean_receive_time_ns;
  return pkg.base_time;  // fallback
}
uint64_t ResolveFrameHeaderStampNs(const StoragePacket& pkg) const {
  if (loc_debug_3d_) return MeanPacketReceiveTimeNs(pkg);
  if (!pkg.points.empty()) return pkg.base_time;  // 장치 구간 시작
  return 0;
}
// InitPointcloud2Msg: cloud.header.stamp = ResolveFrameHeaderStampNs(pkg)
```

4. 노드 파라미터:

```text
3d_loc_debug: bool, default false at driver declare; launch 에서 True 권장
driver get 후: lddc_ptr_->SetLocDebug3d(value)
```

5. **비변경:** 포인트 좌표, PCD publish_freq, IMU 쓰로틀은 §I-F 별개.

---

## I-B. Stamp 릴레이 체인 (덮어쓰기 금지)

### B1. NDT / scanmatcher

- 멤버: `rclcpp::Time sensor_input_stamp_;`  
- cloud 콜백: `sensor_input_stamp_ = msg->header.stamp;`  
- **출력 pose / TF** 의 stamp = `sensor_input_stamp_`  
- **금지:** NDT 완료 시각 `this->now()` 로 pose stamp 설정.

### B2. pose_to_pose_with_cov

```cpp
const auto sensor_input_stamp = msg->header.stamp;
out.header.stamp = sensor_input_stamp;   // NDT stamp 보존
out.header.frame_id = expected_frame;  // 보통 "map"
// cov: 고정 σ (EKF R) — stamp와 독립
// 예: sigma_xy_m=0.01, sigma_yaw_deg=0.3
```

### B3. VESC driver (`vesc_driver.cpp`)

- State(Values) 패킷 수신 시:
  - `receive_stamp = this->now()` (또는 노드 clock now)
  - `state_msg.header.stamp = receive_stamp` → `/sensors/core`
- IMU 패킷 동일: `imu.header.stamp = now()` → `/sensors/imu/raw`
- **장치 내부 시각 사용 금지** (호스트 수신 시각이 stamp 정의 지점).

### B4. vesc_to_odom

```cpp
odom.header.stamp = state->header.stamp;  // 릴레이
// TF stamp 동일
```

### B5. odom_to_twist_converter

```cpp
out_msg.header = msg->header;  // stamp + frame 통째 릴레이
out_msg.twist = msg->twist;    // odom twist 복사
```

### 검증 항등식 (파이프라인 정상 시)

```text
/sensors/core.stamp
  == /odom.stamp
  == /vehicle/twist_with_covariance.stamp
```

---

## I-C. gyro_odometer — max stamp + pre-max delay + IMUmux

### C0. 파일

- `state_estimation/src/gyro_odometer/include/gyro_odometer/gyro_odometer_core.hpp`
- `…/src/gyro_odometer_core.cpp`
- `…/launch/gyro_odometer.launch.xml`

### C1. concat: 출력 stamp = max(vehicle, imu)

`concatGyroAndOdometer(vehicle_queue, gyro_queue)` 내부:

```cpp
const auto tv = rclcpp::Time(vehicle_twist_queue.back().header.stamp);
const auto ti = rclcpp::Time(gyro_queue.back().header.stamp);
twist_with_cov.header.stamp = (tv < ti) ? ti : tv;
// linear from vehicle mean; angular from gyro mean
// published as twist_with_covariance → EKF in_twist
```

### C2. pre-max delay 발행

**순서 고정:** fuse 시 `sampleImuVescDelayBeforeMax` **→** concat **→** publish.

```cpp
void sampleImuVescDelayBeforeMax(const Time& vehicle_stamp, const Time& imu_stamp) {
  if (!enable_sensor_delay_log_) return;
  const Time t_now = this->now();
  Float64MultiArray msg;
  msg.data = {
    (t_now - imu_stamp).seconds(),      // [0] delay_imu_s
    (t_now - vehicle_stamp).seconds(),  // [1] delay_vesc_s
    imu_stamp.seconds(),                // [2] stamp_imu_sec
    vehicle_stamp.seconds()             // [3] stamp_vesc_sec
  };
  imu_vesc_delay_pub_->publish(msg);
}
// topic 절대 경로 (필수 이름 — ROS: 토큰이 숫자로 시작 불가 → 절대 /3d_... 금지):
//   "/timestamp_relay_3d/imu_vesc_delay"
```

params: `enable_sensor_delay_log` (true), `sensor_delay_log_throttle_sec` (1.0).

### C3. IMUmux — 멤버·구독

**제거:** 단일 remapped 토픽 `"imu"` → `callbackImu` 직결.

**추가:**

| 항목 | 내용 |
|------|------|
| 구독 | `/sensors/imu/raw` → `callbackImuMuxVesc` |
| 구독 | `/livox/imu` → `callbackImuMuxLivox` |
| 캐시 | `latest_vesc_imu_`, `latest_livox_imu_`, has_* flags |
| handoff | `processImuMux` 끝에서 `callbackImu(selected)` (기존 본체 유지) |

### C4. IMUmux 파라미터 (재현 시 이 표만 따르면 됨)

| 파라미터 | 기본 | 의미 |
|----------|------|------|
| `imu_mux_mode` | `"vesc"` | `vesc` \| `livox` \| `weighted` \| `test` |
| `imu_mux_weight_vesc` | **50.0** | weighted 시 VESC 몫 [0,100]; **Livox = 100 − vesc (코드 자동)** |
| `imu_mux_livox_bias_wz` | **0.01** | **CSV 전용** de-bias (feed에 빼지 않음); launch CLI 동일 기본값 |
| `imu_mux_timeout_sec` | 0.1 | 캐시 age; `\|now−stamp\| ≤ timeout` |
| `imu_mux_vesc_topic` | `/sensors/imu/raw` | |
| `imu_mux_livox_topic` | `/livox/imu` | |
| `enable_imu_mux_csv` | true | |
| `imu_mux_csv_dir` | `$WORKSPACE/plusresult` | 환경에 맞게 수정 |

**기동 시 정규화 (필수 수식):**

```text
weight_vesc  = clamp(imu_mux_weight_vesc, 0, 100)
weight_livox = 100 - weight_vesc          # 입력 파라미터 없음
w_v = weight_vesc / 100
w_l = weight_livox / 100                   # 항상 합 1
```

**삭제된 파라미터 (재현 시 만들지 말 것):**  
`imu_mux_livox_weight` [0,1], `imu_mux_weight_livox` (별도 입력).

### C5. processImuMux 알고리즘 (완전 명세)

```text
# trigger in {"vesc","livox"} from 구독 콜백

if mode=="vesc"  and trigger!="vesc":  return
if mode=="livox" and trigger!="livox": return
# weighted, test: 양쪽 트리거 허용

want_vesc  = mode in {vesc, weighted, test}
want_livox = mode in {livox, weighted, test}

for each wanted source with cache:
  valid = isFresh(stamp) AND TF(output_frame, imu.frame) 성공
  → ang_*, acc_* in output_frame (보통 base_link)

feed_gyro = true

switch mode:
  vesc:
    if !vesc_valid: return
    selected = latest_vesc_imu_  # 원본 (callbackImu 가 TF 재적용)
    selected_wz = ang_vesc.z;  w=(1,0)

  livox:
    if !livox_valid: return
    selected = latest_livox_imu_
    selected_wz = ang_livox.z; w=(0,1)
    # gyro feed 의 wz 는 raw (bias 미적용)

  test:
    w=(1,0)
    if vesc_valid:
      selected = latest_vesc; selected_wz = ang_vesc.z
      feed_gyro = (trigger == "vesc")   # livox 트리거: CSV only
    else if livox_valid:
      selected = latest_livox; selected_wz = NaN; feed_gyro = false
    else: return

  weighted:
    if both valid:
      w = (w_v, w_l) from C4; fallback=false
    else if vesc only:
      w=(1,0); fallback=true
    else if livox only:
      w=(0,1); fallback=true
    else: return
    selected = 합성 Imu:
      header = 더 늦은 stamp (또는 livox 우선 규칙 가능)
      frame_id = output_frame_
      ω = w_v*ang_vesc + w_l*ang_livox  (없으면 0)
      a = 동일 가중합
      ang_cov 대각 = max(c_v, c_l) after transformCovariance
    selected_wz = selected.angular_velocity.z

# CSV
livox_wz_csv = livox_valid ? (ang_livox.z - bias) : nan
append row(selected_wz, vesc_wz, livox_wz_csv, weights, flags, …)

if feed_gyro:
  callbackImu(selected)
```

**CSV 파일명:** `$imu_mux_csv_dir/{mode}_imu_YYYYMMDD_HHMMSS.csv`  
(기본 dir: `plusresult`; 예: `test_imu_20260811_120920.csv`)

전체 열 정의는 **§CSV-A** (IMUmux) 참고.

TF: `getLatestTransform(output_frame, imu.header.frame_id)` 후 angular/linear doTransform.  
실패 시 해당 소스 invalid.

### C6. gyro launch 인자 (복붙 가능)

```xml
<!-- gyro_odometer.launch.xml IMUmux 핵심 -->
<arg name="imu_mux_mode" default="vesc"/>
<arg name="weight_vesc" default="50.0"/>
<arg name="imu_mux_weight_vesc" default="$(var weight_vesc)"/>
<arg name="imu_mux_livox_bias_wz" default="0.01"/>
<arg name="imu_mux_timeout_sec" default="0.1"/>
<arg name="input_imu_vesc_topic" default="/sensors/imu/raw"/>
<arg name="input_imu_livox_topic" default="/livox/imu"/>
<arg name="enable_imu_mux_csv" default="true"/>
<arg name="imu_mux_csv_dir" default="/home/misys/forza_ws/race_stack/plusresult"/>

<node pkg="gyro_odometer" exec="gyro_odometer" name="gyro_odometer">
  <param name="imu_mux_mode" value="$(var imu_mux_mode)"/>
  <param name="imu_mux_weight_vesc" value="$(var imu_mux_weight_vesc)"/>
  <param name="imu_mux_livox_bias_wz" value="$(var imu_mux_livox_bias_wz)"/>
  <param name="imu_mux_timeout_sec" value="$(var imu_mux_timeout_sec)"/>
  <param name="imu_mux_vesc_topic" value="$(var input_imu_vesc_topic)"/>
  <param name="imu_mux_livox_topic" value="$(var input_imu_livox_topic)"/>
  <param name="enable_imu_mux_csv" value="$(var enable_imu_mux_csv)"/>
  <param name="imu_mux_csv_dir" value="$(var imu_mux_csv_dir)"/>
  <!-- 기존 vehicle twist / twist output remap 유지 -->
</node>
```

---

## I-D. ekf_localizer — delay 4종 + 출력 stamp

### 파일

- `state_estimation/src/ekf_localizer/src/ekf_localizer.cpp`
- `…/include/.../ekf_localizer.hpp` (멤버)
- `…/config/ekf_localizer.param.yaml`

### D1. 파라미터 (yaml 필수 키)

```yaml
/**:
  ros__parameters:
    3d_loc_debug: true
    predict_frequency: 40.0
    enable_sensor_delay_log: true
    sensor_delay_csv_dir: "/home/misys/forza_ws/race_stack/plusresult"  # 변경 가능
    sensor_delay_csv_path: ""   # 비면 auto: 3d_timestamp_relay_YYYYMMDD_HHMMSS.csv
    sensor_delay_log_throttle_sec: 1.0
    sensor_delay_imu_vesc_topic: "/timestamp_relay_3d/imu_vesc_delay"
```

### D2. 멤버 (최소)

- `last_pose_sensor_stamp_`, `has_pose_sensor_stamp_`
- `last_twist_sensor_stamp_`, `has_twist_sensor_stamp_`
- delay latch: imu, vesc, lidar, gyro_twist (+ has_*)
- header stamp latch for CSV stamp_* columns
- `ofstream sensor_delay_csv_`
- sub: Float64MultiArray on imu_vesc topic

### D3. getOutputStamp

```cpp
rclcpp::Time getOutputStamp() const {
  if (!params_.loc_debug_3d) return this->now();
  if (has_pose && has_twist)
    return max(last_pose_sensor_stamp_, last_twist_sensor_stamp_);
  if (has_pose) return last_pose_sensor_stamp_;
  if (has_twist) return last_twist_sensor_stamp_;
  return this->now();
}
// pose/twist/odom/TF publish header.stamp 에 사용
```

- pose 업데이트 성공 시 (debug on): `last_pose_sensor_stamp_ = pose.header.stamp`
- twist 업데이트 성공 시: `last_twist_sensor_stamp_ = twist.header.stamp`

### D4. callbackImuVescDelay

```cpp
// msg.data size >= 4
last_imu_delay_s_  = data[0];  // 재계산 금지, latch only
last_vesc_delay_s_ = data[1];
// stamp from data[2], data[3] as sec → rclcpp::Time
flushSensorDelayCsv();
```

### D5. sampleSensorDelay

```cpp
delay_out = (now() - header_stamp).seconds();
// name "lidar" | "gyro_twist"
// timer 순서: pose 갱신 직후 lidar; twist 갱신 직후 gyro_twist (getOutputStamp 전)
```

### D6. delay CSV 요약

전체 열 정의는 **§CSV-B** (`3d_timestamp_relay_*.csv`).

- `t_now_sec` = **행 쓰기 시각** (재-age 아님)
- delay_* = latch 값 그대로 (없으면 NaN)

\[
\mathrm{delay}=t_{\mathrm{sample}}-t_{\mathrm{header}}
\]

| delay | sample 위치 | header |
|-------|-------------|--------|
| imu | gyro fuse **max 직전** | 선택 IMU stamp |
| vesc | 동일 | vehicle twist stamp |
| lidar | EKF pose 직후 | NDT pose stamp |
| gyro_twist | EKF twist 직후 · output max **전** | max(vehicle,imu) twist stamp |


---

## I-E. Launch 전달 체인 (CLI)

```text
base_system_3D_launch.xml
  imu_mode          → full_localization: imu_mux_mode
  weight_vesc       → imu_mux_weight_vesc
  imu_mux_livox_bias_wz, imu_mux_timeout_sec, enable_imu_mux_csv
        │
        ▼
full_localization.launch.xml  → include gyro_odometer.launch.xml
        │
        ▼
/gyro_odometer  params
```

**base_system 핵심 arg:**

```xml
<arg name="imu_mode" default="vesc"/>
<arg name="imu_mux" default="$(var imu_mode)"/>
<arg name="weight_vesc" default="50.0"/>
<arg name="imu_mux_weight_vesc" default="$(var weight_vesc)"/>
<arg name="imu_mux_livox_bias_wz" default="0.01"/>
<arg name="imu_mux_timeout_sec" default="0.1"/>
<arg name="enable_imu_mux_csv" default="true"/>
```

full_localization include 시 동일 이름으로 gyro에 전달.

---

## I-F. Livox IMU 50 Hz throttle

### 파일: `lddc.h` / `lddc.cpp`

멤버:

```cpp
double imu_publish_frq_{50.0};
uint64_t imu_publish_period_ns_;  // 1e9 / 50
uint64_t last_imu_pub_stamp_ns_[kMaxSourceLidar];
```

`PollingLidarImuData`:

```text
while queue not empty: pop → keep latest only
if no sample: return
PublishImuData(latest)
```

`PublishImuData` 쓰로틀:

```cpp
const uint64_t stamp_ns = imu_data.time_stamp;
if (last != 0 && stamp_ns >= last && (stamp_ns - last) < period_ns)
  return;  // drop
last = stamp_ns;
// else publish Imu msg
```

**영향:** `/livox/imu` 만 ≈50 Hz. **PCD stamp·NDT 경로 무관.**

---

# J. 파일 체크리스트 (다른 컨테이너 diff용)

| 파일 | stamp | delay | mux | 50Hz |
|------|-------|-------|-----|------|
| livox `pub_handler.*` | ● | | | |
| livox `comm/ldq/lds` | ● | | | |
| livox `lddc.*` | ● | | | ● |
| livox driver node + launch | ● | | | |
| `scanmatcher_component.*` | ● | | | |
| `pose_to_pose_with_cov.cpp` | ● | | | |
| `vesc_driver.cpp` | ● | | | |
| `vesc_to_odom.cpp` | ● | | | |
| `odom_to_twist_converter.cpp` | ● | | | |
| `gyro_odometer_core.hpp/cpp` | max | pre-max | ● | |
| `gyro_odometer.launch.xml` | | | ● | |
| `ekf_localizer.hpp/cpp` | out | 4delay | | |
| `ekf_localizer.param.yaml` | ● | ● | | |
| `base_system_3D_launch.xml` | | | ● | |
| `full_localization.launch.xml` | | | ● | |

(선택) 원 git이 있으면 참고 커밋 순서:

`1120674` → `2746b62` → `8904c5d` → `5fe3593` → `862ff9e`  
이후 워킹: weighted **weight_vesc only** (본 문서 §I-C4).

---

# K. 함정 (재현 실패 시)

| 증상 / 실수 | 대처 |
|-------------|------|
| 토픽 이름 `/3d_…` | **금지** → `/timestamp_relay_3d/imu_vesc_delay` |
| delay를 getOutputStamp 뒤 계산 | max stamp로 age 왜곡 → **금지** |
| `imu_mode:=livox` 에 TF 없음 | `base_link ← livox_imu_frame` 추가 |
| weighted에 livox weight 파라미터 또 만듦 | 만들지 말 것; **weight_vesc만** |
| test 에서 livox feed 됨 | `feed_gyro` 만 VESC 트리거 true |
| CSV path 권한 / 디렉터리 | `create_directories` + 쓰기 가능 경로 |
| 50Hz 적용을 PCD에 걸음 | IMU Publish 경로만 |

---

# L. 한 페이지 요약 (암기용)

1. **Stamp 정의:** Livox eth mean · VESC host now  
2. **Stamp 보존:** NDT · pose_cov · odom · odom_to_twist  
3. **Stamp 융합:** gyro max(vehicle, imu)  
4. **Delay:** imu/vesc = max **전**; lidar/gyro = EKF 이벤트; CSV 재-age **금지**  
5. **EKF 출력 stamp:** debug on → max(pose_in, twist_in)  
6. **IMUmux:** 모드 + **weight_vesc (Livox=100−)** + CSV bias **0.01** + **vesc/livox header.stamp 열**  
7. **Livox IMU 토픽:** 50 Hz keep-latest (**PCD 무관**)  

---

# M. 변경 작업자 체크리스트 (코드 수정 시)

새 컨테이너에서 코드를 **수정**할 때도 이 문서만 보고:

- [ ] 수정 영역이 J 체크리스트 어느 열인지 표시  
- [ ] P1–P7 깨지지 않는지 확인  
- [ ] acceptance §R0.5 전부 재실행  
- [ ] launch CLI 변경 시 I-E 체인 3단 파일 동시 수정  
- [ ] 경로 하드코딩 (`/home/misys/...`) 은 **자기 WORKSPACE** 로 교체  
- [ ] CSV 컬럼 변경 시 **§CSV-A / §CSV-B** 동시 수정
- [ ] 이 파일 `Total_CodeChange` 동기 갱신 (이 문서가 단일 SSOT)

---

*문서 역할: 단독 SSOT · IFAC `862ff9e` + weight_vesc + bias 0.01 + CSV stamps · 2026-08-11*  
*파일명: `plusresult/Total_CodeChange`*
