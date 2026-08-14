# SSUPATH F1TENTH — Perception 파이프라인 분석 및 튜닝 가이드

> **대상**: `perception/src/tracking/launch/full_perception.launch.xml`
> **작성일**: 2026-08-13
> **저장소 루트**: `/home/song/ssupath-f1tenth-race-stack`

이 문서는 두 부분으로 되어 있습니다.

- **PART 1** — 파이프라인의 알고리즘과 전체 파라미터 레퍼런스
- **PART 2** — 실차에서 관측된 세 증상(고속 미탐지 / 벽 오인 / 지면 오인)의 원인 분석과 파라미터별 트레이드오프

---

# PART 1. 파이프라인 및 알고리즘 상세

## 0. 전체 구조

`full_perception.launch.xml`은 노드 6개를 띄우는 얇은 래퍼이고, 실제 알고리즘은 각 패키지에 있습니다. 파일에 적힌 순서(tracking이 맨 위)와 **데이터가 흐르는 순서는 반대**입니다.

```
/livox/lidar  (Livox MID-360, PointCloud2, frame=livox_frame, 10 Hz)
   │
   ├─① passthrough_filter        크롭 박스 (관심영역만 남김)
   │      → /passthrough/lidar
   │
   ├─② ray_ground_filter         지면 제거 (Ray Ground Filter)
   │      → /ground_segmentation/lidar
   │
   ├─③ euclidean_cluster         2D 유클리드 클러스터링 + 크기 필터
   │      → /clusters   (DetectedObjectsWithFeature)
   │      │
   │      └─④ cluster_to_obstacle   TF(livox→map) + Frenet(s,d) 변환 + 트랙 경계 필터
   │             → /perception/detection/raw_obstacles  (ObstacleArray)
   │             │
   │             └─⑥ tracking      데이터 어소시에이션 + 정/동적 분류 + 칼만필터
   │                    → /perception/obstacles     ← 상태머신/플래너가 쓰는 최종 토픽
   │                    → /perception/raw_obstacles
   │
   └─⑤ pc2_to_laserscan          3D 점군 → 2D LaserScan 투영
          → /scan   (FTG/RL 컨트롤러, tracking의 inFOV에서 사용)
```

이 런치는 [head_to_head_3D_launch.xml](ssupath-f1tenth-race-stack/stack_master/launch/head_to_head_3D_launch.xml)과 [head_to_head_fsdp_launch.xml](ssupath-f1tenth-race-stack/stack_master/launch/head_to_head_fsdp_launch.xml)에서 통째로 include됩니다.

---

## 1. 사전 지식 — 좌표계

### livox_frame (라이다 프레임)

[localization.launch.py:39](ssupath-f1tenth-race-stack/state_estimation/src/lidarslam_ros2/lidarslam/launch/localization.launch.py#L39)에서 정적 TF가 정의됩니다:

```
base_link → livox_frame : t=(0.27, 0, 0.07), q=(0,0,0.7171,0.7171)  ≈ z축 +90° 회전
```

라이다는 차량 기준 **앞으로 0.27 m, 위로 0.07 m**에 있고 프레임이 90° 돌아가 있습니다. `p_base = Rz(90°)·p_livox = (−y_livox, x_livox)` 이므로:

| livox 프레임 | 차량 기준 의미 |
|---|---|
| `−y` 방향 | **전방** |
| `+x` 방향 | **좌측** |
| `z` | 라이다 높이 기준 (지면 ≈ z = −0.07) |

이게 크롭 박스 `x[−4,4] y[−7,0]`이 "좌우 ±4 m, 전방 0~7 m"인 이유입니다.

> ⚠ [slam.launch.py:35](ssupath-f1tenth-race-stack/state_estimation/src/lidarslam_ros2/lidarslam/launch/slam.launch.py#L35)는 `z=0`, `localization.launch.py:39`는 `z=0.07`로 **서로 다릅니다.** ② 단계의 모든 계산이 이 높이 `h`에 비례하므로 반드시 실측 확인이 필요합니다.

### Frenet 좌표 (s, d)

글로벌 레이스라인을 따라가는 곡선 좌표계입니다.

- `s` : 레이스라인을 따라 잰 거리 [m], `0 ~ track_length`에서 순환(wrap)
- `d` : 레이스라인에서 좌우로 벗어난 횡방향 거리 [m] (좌 +, 우 −)

④와 ⑥은 전부 이 `(s,d)` 위에서 동작합니다. 변환기는 [frenet_converter_cpp.hpp](ssupath-f1tenth-race-stack/utilities/libraries/frenet_conversion_cpp/include/frenet_conversion_cpp/frenet_converter_cpp.hpp)의 `FrenetConverter`이고, 웨이포인트 (x,y,psi)로 자연 3차 스플라인을 만든 뒤 최근접 웨이포인트 → 수직 투영 반복(`iter_max=3`)으로 `(s,d)`를 구합니다.

---

## 2. ① passthrough_filter — 관심영역 크롭

**런치** [passthrough_filter.launch.xml](ssupath-f1tenth-race-stack/perception/src/preprocessing/autoware_pointcloud_preprocessor/launch/passthrough_filter.launch.xml)
**코드** [passthrough_filter_node.cpp](ssupath-f1tenth-race-stack/perception/src/preprocessing/autoware_pointcloud_preprocessor/src/passthrough_filter/passthrough_filter_node.cpp)
**파라미터** [passthrough_filter_node.param.yaml](ssupath-f1tenth-race-stack/perception/src/preprocessing/autoware_pointcloud_preprocessor/config/passthrough_filter_node.param.yaml)

```
/livox/lidar  →  /passthrough/lidar
```

### 알고리즘

PCL의 `pcl::PassThrough`를 축마다 3번 연속 적용합니다. 각 단계는 "지정 필드값이 `[min, max]` 밖인 점을 버린다"가 전부입니다.

1. **z축**: `setFilterLimits(-DBL_MAX, z_max_)` — 위쪽만 자름
2. **x축**: `[x_min, x_max]`
3. **y축**: `[y_min, y_max]`

결과는 축정렬 직육면체(AABB) 크롭입니다. 맨 앞에 두는 이유는 순전히 **연산량**입니다. 뒤의 클러스터링은 KdTree라 O(N log N)인데, 여기서 점을 90% 이상 날려버립니다.

### 파라미터

| 파라미터 | 값 | 의미 |
|---|---|---|
| `z_max` | 0.3 | 라이다 기준 이 높이 초과 점 제거. 지면이 z≈−0.07이므로 **지면 위 약 0.37 m**까지만 봄 |
| `z_min` | 0.01 | **⚠ 실제로 안 쓰입니다.** [cpp:60](ssupath-f1tenth-race-stack/perception/src/preprocessing/autoware_pointcloud_preprocessor/src/passthrough_filter/passthrough_filter_node.cpp#L60)에서 `-DBL_MAX`가 들어감. 아래쪽 제거는 ②가 담당 |
| `x_min/x_max` | −4.0 / 4.0 | 좌우 ±4 m |
| `y_min/y_max` | −7.0 / 0.0 | 전방 0~7 m (뒤쪽은 통째로 버림) |

> `y_max=0` 때문에 **차 뒤쪽 점군은 아예 존재하지 않습니다.** 그리고 `y_min=−7`이 **퍼셉션의 진짜 물리적 상한**입니다.

### 기반 클래스 (`Filter`)

[filter.cpp](ssupath-f1tenth-race-stack/perception/src/preprocessing/autoware_pointcloud_preprocessor/src/filter.cpp)의 `Filter`를 상속합니다.

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `input_frame` / `output_frame` | `""` | 비어 있으면 TF 변환 안 함. 출력 frame_id는 그대로 `livox_frame` |
| `max_queue_size` | 5 | 구독 큐 깊이 |
| `use_indices` | false | `PointIndices` 토픽으로 부분집합만 처리할지 |
| `approximate_sync` | false | 위 인덱스와의 시간 동기화 방식 |

---

## 3. ② ray_ground_filter — 지면 제거

**런치** [ray_ground_filter.launch.xml](ssupath-f1tenth-race-stack/perception/src/preprocessing/autoware_ground_segmentation/launch/ray_ground_filter.launch.xml)
**코드** [node.cpp](ssupath-f1tenth-race-stack/perception/src/preprocessing/autoware_ground_segmentation/src/ray_ground_filter/node.cpp)
**파라미터** [ray_ground_filter.param.yaml](ssupath-f1tenth-race-stack/perception/src/preprocessing/autoware_ground_segmentation/config/ray_ground_filter.param.yaml)

```
/passthrough/lidar  →  /ground_segmentation/lidar   (지면이 아닌 점만)
```

Autoware의 고전적인 **Ray Ground Filter**입니다. RANSAC 평면 피팅과 달리 "지면이 평면이라고 가정하지 않고", 센서에서 뻗어나가는 광선(ray)을 따라 **점 사이의 국소 기울기**만 봅니다. 오르막/내리막/요철에 강합니다.

### 단계 1: 원통좌표 변환 + 부채꼴 분할 (`ConvertXYZIToRTZColor`)

```cpp
radius = sqrt(x² + y²)
theta  = atan2(y, x) [deg], 0~360으로 정규화
radial_div = floor(theta / radial_divider_angle_)
```

`radial_divider_angle = 1.0°` 이므로 **360개의 부채꼴(ray)** 로 나눕니다. 그리고 각 부채꼴 안에서 `radius` 오름차순 정렬 — **센서에서 가까운 점부터 먼 점 순서**로 줄을 세웁니다.

### 단계 2: 광선을 따라가며 분류 (`ClassifyPointCloud`)

**(a) 국소 기울기 문턱** — 직전 점 대비:

```cpp
points_distance  = r_j − r_{j−1}
height_threshold = tan(local_max_slope) × points_distance
if (height_threshold < min_height_threshold) height_threshold = min_height_threshold;
```

직전 점과의 반경 차이에 최대 허용 경사를 곱해 "이만큼까지는 높이가 튀어도 지면"이라고 봅니다. 아주 가까운 두 점은 문턱이 0이 되므로 하한을 겁니다.

**(b) 전역 높이 문턱** — 센서 원점 대비:

```cpp
general_height_threshold = tan(general_max_slope) × r_j
```

"센서에서 r만큼 떨어진 지면이라면 높이는 최대 이 정도"라는 절대 상한입니다. 국소 기울기만 보면 오차가 누적돼 지면이 서서히 떠오르는 문제(drift)를 잡아줍니다.

**판정 로직** ([node.cpp:217-248](ssupath-f1tenth-race-stack/perception/src/preprocessing/autoware_ground_segmentation/src/ray_ground_filter/node.cpp#L217-L248)):

```
if |z_j − z_{j−1}| ≤ height_threshold:          # 국소적으로 완만함
      직전 점이 지면  → 지면
      직전 점이 비지면 → |z_j| ≤ general_height_threshold 이면 지면, 아니면 비지면
else:                                            # 국소 기울기 초과
      points_distance > reclass_distance_threshold  AND  |z_j| ≤ general_height_threshold
            → 지면 (재분류)
      else  → 비지면
```

마지막 `else` 안의 **재분류(reclassification)** 가 핵심 트릭입니다. 앞에 장애물이 있으면 그 뒤로 그림자(빈 구간)가 생기는데, 그림자 건너편 첫 점은 직전 점과 멀리 떨어져 있어 국소 기울기가 엉망으로 나옵니다. "점 간격이 `reclass_distance_threshold`(0.1 m)보다 멀면 국소 기울기는 못 믿으니 전역 문턱만으로 다시 판단한다"는 뜻입니다.

**첫 점(j==0)의 특수 처리**: `prev_radius=0, prev_height=0` (= 센서 원점)으로 시작하고 `local_max_slope` 대신 `initial_max_slope`를 씁니다.

### 단계 3: 분리 발행 (`ExtractPointsIndices`)

지면 인덱스 불리언 마스크를 만들고 `memcpy`로 원본 바이트를 두 클라우드에 나눠 담습니다. 최종 출력은 **`no_ground` 쪽**입니다 ([node.cpp:365](ssupath-f1tenth-race-stack/perception/src/preprocessing/autoware_ground_segmentation/src/ray_ground_filter/node.cpp#L365)).

### 파라미터

| 파라미터 | 값 | 의미 |
|---|---|---|
| `radial_divider_angle` | 1.0 | 부채꼴 하나의 각도 [deg]. 360개 생성 |
| `general_max_slope` | 8.0 | 센서 원점 기준 전역 최대 경사 [deg]. yaml 코멘트: "6~7이 정배" |
| `local_max_slope` | 6.0 | 인접 두 점 사이 최대 경사 [deg]. 코멘트: "2~3이 정배" |
| `initial_max_slope` | 3.0 | 부채꼴의 **첫 점**에만 적용 [deg]. 코멘트: "1이 정배" |
| `min_height_threshold` | 0.02 | 높이 문턱의 하한 [m]. 근거리 점 보호용 |
| `concentric_divider_distance` | 0.0 | **0이면 조건이 성립 안 하므로 사실상 비활성화** |
| `reclass_distance_threshold` | 0.1 | 재분류 트리거 거리 [m] |
| `use_vehicle_footprint` | false | **false이므로 `min_x/max_x/min_y/max_y`(±0.01)는 미사용** |
| `publish_processing_time_detail` | false | true면 함수별 소요시간 발행 |

> **F1TENTH 튜닝 감각**: 대회 장애물은 높이가 낮습니다. `initial_max_slope`/`local_max_slope`를 크게 잡으면 장애물 밑동이 지면으로 먹혀 클러스터 점 개수가 줄고, 그러면 ③의 `min_cluster_size` 문턱에 걸려 원거리에서 물체가 통째로 사라집니다.

> **참고**: `#pragma omp for`가 `#pragma omp parallel` 블록 밖에 있어 실제 병렬화는 안 됩니다(단일 스레드 동작).

---

## 4. ③ euclidean_cluster — 클러스터링

**런치** [euclidean_cluster.launch.xml](ssupath-f1tenth-race-stack/perception/src/clustering/autoware_euclidean_cluster/launch/euclidean_cluster.launch.xml) → [euclidean_cluster.launch.py](ssupath-f1tenth-race-stack/perception/src/clustering/autoware_euclidean_cluster/launch/euclidean_cluster.launch.py)
**코드** [euclidean_cluster.cpp](ssupath-f1tenth-race-stack/perception/src/clustering/autoware_euclidean_cluster/lib/euclidean_cluster.cpp), [euclidean_cluster_node.cpp](ssupath-f1tenth-race-stack/perception/src/clustering/autoware_euclidean_cluster/src/euclidean_cluster_node.cpp), [utils.cpp](ssupath-f1tenth-race-stack/perception/src/clustering/autoware_euclidean_cluster/lib/utils.cpp)
**파라미터** [euclidean_cluster.param.yaml](ssupath-f1tenth-race-stack/perception/src/clustering/autoware_euclidean_cluster/config/euclidean_cluster.param.yaml)

```
/ground_segmentation/lidar  →  /clusters
```

### 런치 구조 (함정 하나)

xml이 py를 include하고, py가 `ComposableNodeContainer`(`euclidean_cluster_container`)를 만들어 그 안에 `EuclideanClusterNode` 컴포넌트를 로드합니다. `use_low_height_cropbox=false`이므로 추가 CropBox 없이 입력을 바로 구독합니다.

> ⚠ **주의**: xml은 `<arg name="input_points_raw_list" .../>`로 넘기는데 py가 실제로 읽는 이름은 `input_pointcloud`입니다 ([launch.py:44](ssupath-f1tenth-race-stack/perception/src/clustering/autoware_euclidean_cluster/launch/euclidean_cluster.launch.py#L44)). 이름이 안 맞습니다. 그런데도 동작하는 이유는 ROS 2의 `<include>`가 **스코프를 만들지 않아서** 부모 xml이 선언한 `input_pointcloud=/ground_segmentation/lidar` 런치 컨피그가 그대로 상속되고, py의 `DeclareLaunchArgument`는 "이미 설정된 값이 있으면 기본값을 안 덮어쓰기" 때문입니다. 즉 **우연히** 맞습니다. arg 이름을 바꾸거나 include를 `<group>`으로 감싸면 조용히 `/perception/obstacle_segmentation/pointcloud`(없는 토픽)로 떨어져 퍼셉션 전체가 죽습니다.

### 단계 1: 2D 투영

`use_height = false`이므로 모든 점의 `z`를 0으로 눌러버립니다 ([cpp:54-63](ssupath-f1tenth-race-stack/perception/src/clustering/autoware_euclidean_cluster/lib/euclidean_cluster.cpp#L54-L63)). 라이다 링(ring) 간격 때문에 같은 물체의 위/아래 점이 z 방향으로 멀리 떨어져 하나로 안 묶이는 문제를 회피합니다.

### 단계 2: KdTree + 유클리드 클러스터 추출

```cpp
pcl::search::KdTree tree;  tree.setInputCloud(2D 점군);
pcl::EuclideanClusterExtraction ec;
ec.setClusterTolerance(tolerance_);       // 0.2 m
ec.setMinClusterSize(min_cluster_size_);  // 3
ec.setMaxClusterSize(max_cluster_size_);  // 750
```

PCL 유클리드 클러스터링 = **반경 기반 region growing (BFS/flood-fill)**:

1. 아직 처리 안 된 점 하나를 큐에 넣음
2. 큐에서 점을 꺼내 KdTree로 반경 `tolerance` 이내 이웃을 모두 찾음
3. 처음 보는 이웃은 같은 클러스터에 추가하고 큐에 넣음
4. 큐가 빌 때까지 반복 → 하나의 연결 성분이 클러스터
5. 크기가 `[min, max]` 밖이면 폐기

인덱스를 얻은 뒤 **원본 3D 점**을 다시 담으므로 z 정보는 보존됩니다.

### 단계 3: 크기 필터

```cpp
if (dx > max_x_ || dy > max_y_ || dz > max_z_) continue;  // 통째로 폐기
```

한 축이라도 넘으면 버립니다. **벽·펜스처럼 긴 구조물 제거**용입니다.

### 단계 4: 메시지 변환

`tier4_perception_msgs/DetectedObjectsWithFeature`로 포장합니다. 클러스터 하나당:

- `feature.cluster` = 클러스터 점군 그대로
- `object.kinematics...pose.position` = **점들의 산술 평균(centroid)** ← ④가 사용
- `object.shape.dimensions` = (dx, dy, dz), `shape.type = 0 (BOX)` ← ④가 `size`로 사용
- `object.classification` = `UNKNOWN`, 확률 1.0 (분류기 없음)

### 파라미터

| 파라미터 | 값 | 의미 |
|---|---|---|
| `tolerance` | 0.2 | 클러스터 연결 반경 [m] |
| `min_cluster_size` | 3 | 클러스터로 인정할 최소 점 개수 |
| `max_cluster_size` | 750 | 최대 점 개수 |
| `use_height` | false | z를 무시하고 2D로 클러스터링 |
| `max_x / max_y / max_z` | 0.8 | 바운딩박스 상한 [m]. **livox 프레임 축**이므로 `max_x`=횡, `max_y`=종, `max_z`=높이 |

### yaml에 남은 튜닝 기록

- **`max_*` 0.6 → 0.8**: 규격상 장애물은 0.5×0.5 m지만 비스듬히 보면 축정렬 박스가 커집니다(15° 0.612 / 30° 0.683 / 45° 0.707 m). 0.6이면 약 15° 이상 틀어진 장애물을 버립니다. A/B: 6~7 m 연속성 24.1% → 28.3%.
- **`min_cluster_size` 4 → 3**: 6~7 m에서 물체당 점 개수 중앙값이 5~6개라 문턱(4) 바로 위에서 널뛰며 물체가 깜빡였습니다. 연속성 23.1% → 28.2%.
- **`tolerance`는 올리지 말 것**: 0.2 → 0.3은 역효과(연속성 23.1% → 13.3%). 반경을 키우면 원거리 물체가 주변 벽/바닥 점과 **한 덩어리로 흡수**돼 크기 필터에 같이 걸려 죽습니다(근거리 클러스터 점 중앙값 53 → 71이 증거).

---

## 5. ④ cluster_to_obstacle — 클러스터를 트랙 좌표 장애물로

**런치** [cluster_to_obstacle_cpp.launch.xml](ssupath-f1tenth-race-stack/perception/src/clustering/cluster_to_obstacle_cpp/launch/cluster_to_obstacle_cpp.launch.xml)
**코드** [cluster_to_obstacle_node.cpp](ssupath-f1tenth-race-stack/perception/src/clustering/cluster_to_obstacle_cpp/src/cluster_to_obstacle_node.cpp)
**파라미터** [cluster_to_obstacle_node.param.yaml](ssupath-f1tenth-race-stack/perception/src/clustering/cluster_to_obstacle_cpp/config/cluster_to_obstacle_node.param.yaml)

```
/clusters + /global_waypoints + /car_state/frenet/odom
  →  /perception/detection/raw_obstacles   (map 프레임, Frenet 좌표)
  +  /perception/detection/obstacles_markers
  +  /perception/detect_bound  (latched)
```

### 초기화: `pathCb` — 트랙 경계 구성

1. `FrenetConverter`를 (x, y, psi) 배열로 생성
2. 각 웨이포인트의 트랙 반폭을 **안쪽으로 축소**: `d_right − boundaries_inflation`, `d_left − boundaries_inflation`
3. `smallest_d_` = 전 구간 최소 반폭, `biggest_d_` = 전 구간 최대 반폭
4. `track_length_` = 마지막 웨이포인트의 `s_m`
5. 경계선을 `SPHERE_LIST` 마커로 `/perception/detect_bound`에 latched 발행

### 메인: `clusterCb`

#### 단계 1: TF 변환 (livox_frame → map)

[node.cpp:207-226](ssupath-f1tenth-race-stack/perception/src/clustering/cluster_to_obstacle_cpp/src/cluster_to_obstacle_node.cpp#L207-L226)의 **2단 폴백**:

- **(A)** `tf_buffer_.transform(in_pt, map, 0.05s)` — 라이다 메시지의 **타임스탬프 시점** TF로 변환 시도 (최대 50 ms 대기)
- **(B)** 실패 시 `lookupTransform(map, livox_frame, TimePointZero)` — **가장 최신** TF로 강제 변환
- 둘 다 실패하면 그 클러스터는 버림

> ⚠ **(B) 경로가 이 파이프라인의 알려진 약점입니다.** yaml 실측: NDT/SLAM이 약 0.2 s 뒤처져 있어 폴백이 걸리면 **위치 오차 = 속도 × 0.2 s**(6.6 m/s면 1.3 m). 실측 검출률이 1~2 m/s에서 30.6%인데 5 m/s 이상에서 3.7%로 **8배 붕괴**했고, 바로 앞단인 `/clusters`는 멀쩡했습니다. 근본 수정은 lidarslam의 `use_odom:true`.

#### 단계 2~3: 크기 추출 및 Frenet 변환

```cpp
size = max(dim.x, dim.y);                       // ③이 넣어준 바운딩박스 치수 중 큰 쪽
fr_converter_->get_frenet(xs, ys);              // (s,d) 배치 변환
```

#### 단계 4: 트랙 내부 판정 — `laserPointOnTrack`

```cpp
if (wrap_s(s − car_s) > max_viewing_distance_) return false;  // ① 너무 먼 전방
if (|d| >= biggest_d_)  return false;                          // ② 확실히 트랙 밖
if (|d| <= smallest_d_) return true;                           // ③ 확실히 트랙 안
// ④ 애매한 구간 → 해당 s의 웨이포인트를 찾아 그 지점 실제 반폭과 비교
idx = upper_bound(s_array_, s) − 1;
if (d <= −d_right_array_[idx] || d >= d_left_array_[idx]) return false;
return true;
```

②·③은 조기 종료용입니다. `wrap_s`는 `s` 차이를 `[−L/2, L/2)`로 접으므로 **뒤쪽 장애물은 음수가 되어 ①을 항상 통과**합니다(거리 제한은 전방에만).

#### 단계 5: 메시지 구성

```cpp
ob.s_center = s;              ob.d_center = d;      ob.size = size;
ob.s_start  = s − size/2;     ob.s_end    = s + size/2;
ob.d_left   = d + size/2;     ob.d_right  = d − size/2;
```

장애물을 `(s,d)` 평면의 정사각형으로 근사합니다. 배열은 **비어 있어도 항상 발행**합니다.

### 파라미터

| 파라미터 | 값 | 의미 |
|---|---|---|
| `boundaries_inflation` | 0.03 | 트랙 경계를 안쪽으로 줄이는 폭 [m] |
| `max_viewing_distance` | 10.0 | 전방 탐지 거리 [m] |
| `input_frame` | `livox_frame` | 클러스터 좌표의 원본 프레임 |
| `map_frame` | `map` | 출력 프레임 |
| `min_obs_size` / `max_obs_size` | 0.05 / 0.5 | **선언만 되고 사용 안 됨** (코드 주석 처리) |
| `use_sim_time` | false | 실차에서 true면 `/clock`을 기다리다 노드가 멈춤 |

### yaml 튜닝 근거

- **`boundaries_inflation` 0.07 → 0.03**: 트랙 반폭이 좌 0.710 / 우 0.509 m인데 0.07을 깎으면 폭의 10~14%가 날아갑니다. 탐지 장애물의 횡위치 분포가 경계 대비 0.85~0.95에서 급격히 끊겼고(전체의 11.5%가 문턱 5 cm 이내), 즉 **벽에 붙은 상자가 버려지는 띠 안에 들어가 통째로 무시**되고 있었습니다. 부작용은 유령 증가.
- **`max_viewing_distance` 10.0**: 크롭이 7 m라 물리적으로 무의미해 보이지만, TF 폴백 위치 오차(최대 1.3 m)에 대한 여유입니다.

---

## 6. ⑤ pc2_to_laserscan — 3D 점군을 2D 스캔으로

**런치** [pc2_to_laserscan.launch.xml](ssupath-f1tenth-race-stack/perception/src/preprocessing/pc2_to_scan_cpp/launch/pc2_to_laserscan.launch.xml)
**코드** [pc2_to_laserscan.cpp](ssupath-f1tenth-race-stack/perception/src/preprocessing/pc2_to_scan_cpp/src/pc2_to_laserscan.cpp)
**파라미터** [pc2_to_scan_node.param.yaml](ssupath-f1tenth-race-stack/perception/src/preprocessing/pc2_to_scan_cpp/config/pc2_to_scan_node.param.yaml)

```
/ground_segmentation/lidar  →  /scan  (sensor_msgs/LaserScan)
```

①~④와는 **병렬 가지**입니다. 3D 라이다를 2D LiDAR처럼 보이게 만들어, 2D 스캔을 전제로 짜인 기존 코드(FTG/RL 컨트롤러, ⑥의 `inFOV`)를 그대로 쓸 수 있게 합니다.

### 알고리즘

```cpp
num_bins = round((angle_max − angle_min) / angle_increment);   // 1260
ranges.assign(num_bins, range_max);                            // ← 초기값이 range_max

for each point:
    r = hypot(x, y);
    if (r < range_min || r > range_max) continue;
    angle = atan2(y, x);
    idx   = round((angle − angle_min) / angle_increment);
    if (idx 범위 밖) continue;
    ranges[idx] = min(ranges[idx], r);      // z-buffer 식 최근접 유지
```

- z는 읽기만 하고 **쓰지 않습니다**(높이 무시). 지면은 ②가 이미 제거.
- `use_closest_point`는 true/false 양쪽 분기가 실질적으로 동일합니다.
- 헤더는 입력 그대로 복사 → **frame_id는 `livox_frame`**.

### 파라미터

| 파라미터 | 값 | 의미 |
|---|---|---|
| `range_min / range_max` | 0.1 / 7.0 | 유효 거리 [m] |
| `angle_increment_deg` | 0.25 | 각도 분해능 [deg] |
| `angle_min / angle_max` | −6π/4 (−270°) / +π/4 (+45°) | 스캔 각도 범위 (코드 기본값) |
| `scan_time` | 0.1 (하드코딩) | LaserScan 헤더 필드 |

### 주의할 두 가지

1. **빈 방향이 "7 m 벽"으로 보입니다.** `ranges` 초기값이 `+inf`가 아니라 `range_max`(7.0)입니다. 점이 안 떨어진 각도 빈은 "7 m 앞에 뭔가 있다"로 읽힙니다.
2. **앞쪽 360개 빈은 절대 채워지지 않습니다.** `atan2`의 치역은 `[−180°, 180°]`인데 `angle_min`이 −270°입니다. 인덱스 0~359는 영원히 `range_max`로 남습니다. 실제 유효 구간은 `[−180°, +45°]`이고, livox 프레임 기준 전방은 −90°(≈720번 빈)입니다.

---

## 7. ⑥ tracking — 추적, 정/동적 분류, 상대차 상태추정

**런치** [tracking_launch.xml](ssupath-f1tenth-race-stack/perception/src/tracking/launch/tracking_launch.xml)
**코드** [tracking.cpp](ssupath-f1tenth-race-stack/perception/src/tracking/src/tracking.cpp)
**파라미터** [opponent_tracker_params.yaml](ssupath-f1tenth-race-stack/stack_master/config/opponent_tracker_params.yaml)

```
/perception/detection/raw_obstacles + /global_waypoints
+ /car_state/frenet/odom + /car_state/odom + /scan
   →  /perception/obstacles          ← 상태머신·플래너·컨트롤러가 쓰는 최종 토픽
   →  /perception/raw_obstacles
   →  /perception/static_dynamic_marker_pub
```

> ⚠ **먼저 알아둘 것**: 넘겨주는 yaml은 현재 `tracking: ros__parameters: {}` — **비어 있습니다.** 따라서 아래 모든 값은 `tracking.cpp`의 `declare_parameter` 기본값입니다. 원래 최상위 키가 `perception:`이라 노드 이름(`tracking`)과 안 맞아 파일이 통째로 무시되고 있었고, 그동안의 모든 튜닝이 cpp 기본값 기준으로 이뤄졌기 때문에 지금 값을 켜면 전제가 깨집니다.

### 자료구조 1: `ObstacleSD` — 추적 중인 후보 하나

| 필드 | 의미 |
|---|---|
| `meas_s`, `meas_d` | 최근 측정값 덱 (30개 넘으면 20개로 자름) |
| `mean_s`, `mean_d` | 이동평균 위치 |
| `nb_meas` | 누적 측정 횟수 |
| `ttl` | Time-To-Live. 측정이 안 붙으면 감소, 0 이하면 삭제 |
| `staticFlag` | `nullopt`(미정) / `true`(정적) / `false`(동적) |
| `isInFront`, `isVisible` | 전방 여부, 가시 여부 |

**`update_mean` — s는 원형 평균(circular mean)**: `d`는 산술 이동평균이지만, `s`는 결승선에서 `L → 0`으로 점프하므로 산술 평균이 트랙 반대편으로 튑니다. 그래서 각도로 바꿔(`s·2π/L`) cos/sin을 각각 평균낸 뒤 `atan2`로 되돌립니다.

```cpp
c = (cos(prev)·n + cos(cur)) / (n+1);
s = (sin(prev)·n + sin(cur)) / (n+1);
mean_s = atan2(s, c) · L / 2π;   // 음수면 +L
```

**`isStatic` — 이중 문턱 히스테리시스**:

```
if (nb_meas > min_nb_meas):        # 측정이 최소 3회 쌓인 뒤에만 판정
    std_s < min_std && std_d < min_std  → static_count++
    std_s > max_std || std_d > max_std  → static_count = 0   (리셋)
    total_count++
    staticFlag = (static_count / total_count) >= 0.5
else:
    staticFlag = nullopt
```

**"위치 산포가 작으면 정적, 크면 동적"** 이 원리입니다. `min_std`(0.16)와 `max_std`(0.22) 사이는 아무 일도 안 일어나는 완충 구간이라 경계에서 채터링하지 않습니다.

### 자료구조 2: `OpponentState` — 상대 차량 1대 전용 칼만 필터

상태 벡터는 Frenet 좌표의 **등속(Constant Velocity) 모델**: `x = [s, vs, d, vd]ᵀ`

**예측 (`predict`)**:

```
F = I,  F(0,1) = dt,  F(2,3) = dt          # s += vs·dt,  d += vd·dt
x = F·x + B·u
x(0) = wrap(x(0), L)                        # s를 [0,L)로 접기
P = F·P·Fᵀ + Q
```

제어 입력 `u`가 이 필터의 도메인 지식입니다:

| 모드 | `u` | 의미 |
|---|---|---|
| `useTargetVel = false` (평상시) | `[0, 0, −P_d·d, −P_vd·vd]` | **d와 vd를 0으로 끌어당김.** "상대도 결국 레이스라인으로 복귀한다" |
| `useTargetVel = true` (트랙 소실 후) | `[0, P_vs·(v_target − vs), −P_d·d, −P_vd·vd]` | 위에 더해 **속도를 글로벌 경로 목표속도로 수렴** |

```cpp
idx = floor(s × s_index_scale) % N;       // s_index_scale = 10 → 0.1 m 해상도
v_target = ratio_to_glob_path × path_vx[idx];  // 0.3배 (보수적 가정)
```

**프로세스 잡음 `Q`** — 연속시간 백색가속도 CV 모델의 표준 이산화:

```
Q_block(q) = q · [ dt⁴/4   dt³/2 ]
                 [ dt³/2   dt²   ]
```

`q_vs = 2`, `q_vd = 8` — 횡방향에 4배 큰 잡음을 줘서 급격한 좌우 기동에 빨리 반응합니다.

**갱신 (`update`)**: `H = I`이므로 `[s, vs, d, vd]` 전부를 "측정"으로 받습니다. 속도 측정은 실제 센서값이 아니라 **위치의 유한차분**입니다:

```cpp
vs = (2/3)·(s_n − s_{n−1})·rate + (1/3)·(s_{n−1} − s_{n−2})·rate;   // 가중 이동평균 미분
vd = (d_n − d_{n−1})·rate;
if (vs <= −1.0 || vs >= 8.0) { opponent_.isInitialised = false; }    // 아웃라이어 → 필터 리셋
```

잔차 계산에서 `s` 성분만 wrap 처리합니다 — 이 부분이 "그냥 KF"가 아닌 이유입니다:

```cpp
y = z − h(x);
y(0) = wrap_s_residual_sym(y(0), L);   // [−L/2, L/2)로 접어 결승선 점프 방지
K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹;
x = x + K·y;   P = (I − K·H)·P;
```

갱신 후 `vs_filt`/`vd_filt`(5칸 시프트 레지스터)에 밀어 넣고, 발행할 때 **5-tap 이동평균**으로 내보냅니다.

### 메인 루프 (40 Hz 타이머)

```
loop():
  1. opponent_.predict()   (초기화됐으면)
  2. updateTracking()
  3. publishObstacles()
  4. publishMarkers()
```

### `updateTracking` — 데이터 어소시에이션

기존 트랙마다 이번 프레임 측정 중 하나를 붙입니다 (**greedy nearest-neighbour, 1:1**).

`verifyPosition`은 쿼리점을 상황에 따라 바꿉니다:

| 트랙 상태 | 쿼리점 | 탐색 반경 |
|---|---|---|
| 정적 / 미정 | `(mean_s, mean_d)` | `max_dist` = 0.5 |
| **동적 + EKF 초기화됨** | **EKF 예측 위치 `(x(0), x(2))`** | `max_dist × aggro_multi` = 1.0 |

> `getClosestWithin`은 `hypot(Δs, Δd)`를 쓰며 **`s`의 wrap을 고려하지 않습니다** — 결승선 근처에서 매칭이 끊길 수 있습니다.

**매칭 성공 시**: 측정 추가 → `update_mean` → `isStatic` 재판정 → `ttl` 리셋(20). 동적이면 EKF `update` 호출(또는 `initializeDynamic`). 사용한 측정은 후보 목록에서 제거.

**매칭 실패 시**:

```
ttl <= 0                              → 트랙 삭제. 동적이었으면 useTargetVel = true
staticFlag 미정                        → ttl--
정적 && noMemoryMode(true)             → ttl--                  ← 현재 기본 경로
정적 && !noMemoryMode && dist_s < 6.0  → inFOV()로 가시성 확인
                                          보이는데 측정 없음 → ttl--
                                          가려짐            → 유지, isVisible=false
동적                                   → ttl--
```

`inFOV`는 "장애물 방향의 스캔 거리보다 장애물이 가까운가"로 폐색(occlusion)을 판정합니다. 다만 **`noMemoryMode_` 기본값이 true라 이 분기는 실행되지 않습니다.** (또한 이 함수는 차량 헤딩 기준 bearing으로 `/scan` 인덱스를 계산하는데 `/scan`은 livox 프레임 각도로 채워지므로 90°(=360빈) 어긋납니다.)

### `publishObstacles` — 발행 규칙

```cpp
if (!t.isInFront || t.nb_meas <= 6) continue;   // 전방 + 최소 7회 관측된 것만
```

| 트랙 상태 | 좌표 | 발행 토픽 |
|---|---|---|
| 미정 (`nullopt`) | 최신 측정 | `publish_static_`이면 `/perception/obstacles`, 아니면 `/perception/raw_obstacles` |
| 정적 | **평균** `(mean_s, mean_d)` | `publish_static_`일 때만 `/perception/obstacles` |
| 동적 | 최신 측정 | 항상 `/perception/raw_obstacles` (최종 토픽엔 안 나감) |

동적 장애물이 최종 토픽에 나가는 유일한 경로는 **EKF 추정치**입니다:

```cpp
if (opponent_.isInitialised && opponent_.P(0,0) < var_pub_ && checkInFront(opponent_.x(0))) {
    msg.vs = vs_filt 5개 평균;  msg.vd = vd_filt 5개 평균;
    msg.is_static = false;
    out.obstacles.push_back(msg);   // → /perception/obstacles
}
```

`P(0,0) < var_pub_`은 **"s 추정 분산이 충분히 작을 때만 내보낸다"**는 신뢰도 게이트입니다. 측정이 끊겨 predict만 반복되면 `P`가 커져 자동으로 발행이 멈춥니다.

### 파라미터 (전부 cpp 기본값)

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `rate` | 40 | 루프 주기 [Hz]. `dt = 1/40` |
| `max_dist` | 0.5 | 데이터 어소시에이션 최대 매칭 거리 [m] |
| `aggro_multi` | 2.0 | 동적 트랙일 때 매칭 반경 배수 |
| `dist_infront` | 7.0 | 이보다 먼 전방 장애물은 추적/발행 안 함 [m] |
| `dist_deletion` | 6.0 | 정적 트랙을 가시성 검사로 유지하는 거리 [m] |
| `var_pub` | 1.0 | EKF 발행 게이트 (`P(0,0)` 상한) |
| `publish_static` | true | 정적/미정 장애물을 `/perception/obstacles`로 낼지 |
| `noMemoryMode` | true | true면 정적 장애물을 기억하지 않고 안 보이면 바로 소멸 |
| `P_vs / P_d / P_vd` | 0.2 / 0.02 / 0.2 | 제어입력 게인 |
| `measurment_var_s / _d` | 0.002 | 위치 측정 잡음 분산 `R` |
| `measurment_var_vs / _vd` | 0.2 | 속도 측정 잡음 분산 `R` (유한차분이라 큼) |
| `process_var_vs / _vd` | 2.0 / 8.0 | 프로세스 잡음 `Q`의 가속도 강도 |
| `ratio_to_glob_path` | 0.3 | 목표속도 모델에서 글로벌 속도 대비 비율 |
| `sd_min_nb_meas` | 2 | 정/동적 판정을 시작할 최소 측정 횟수 |
| `sd_ttl` | 20 | 트랙 TTL (40 Hz 기준 0.5 s) |
| `sd_min_std / sd_max_std` | 0.16 / 0.22 | 정/동적 히스테리시스 이중 문턱 [m] |
| `measure` | false | true면 `/perception/tracking/latency` 발행 |
| `vs_reset` | 0.1 | **선언만 되고 사용 안 됨** |

---

## 8. 알고리즘 계보 요약

| 단계 | 알고리즘 종류 |
|---|---|
| ① passthrough | 축정렬 크롭 박스 (PCL PassThrough) |
| ② ray_ground_filter | 광선별 국소 기울기 기반 지면 분할 (Autoware Ray Ground Filter) |
| ③ euclidean_cluster | KdTree 반경 기반 region growing (PCL EuclideanClusterExtraction), 2D 투영 + AABB 크기 필터 |
| ④ cluster_to_obstacle | TF 변환 + 3차 스플라인 Frenet 투영 + 이진탐색 트랙 경계 판정 |
| ⑤ pc2_to_laserscan | 각도 빈 최근접 투영 (3D→2D) |
| ⑥ tracking | Greedy NN 데이터 어소시에이션 + 산포 기반 정/동적 분류(히스테리시스) + s-wrap 처리 선형 칼만 필터(CV 모델 + 도메인 제어입력) |

## 9. 발견된 코드 이슈

1. **[euclidean_cluster.launch.xml:13](ssupath-f1tenth-race-stack/perception/src/clustering/autoware_euclidean_cluster/launch/euclidean_cluster.launch.xml#L13)** — `input_points_raw_list`는 py가 읽지 않는 이름. 런치 컨피그 누수 덕에 우연히 동작 중이니 `input_pointcloud`로 고칠 것.
2. **[passthrough_filter_node.cpp:60](ssupath-f1tenth-race-stack/perception/src/preprocessing/autoware_pointcloud_preprocessor/src/passthrough_filter/passthrough_filter_node.cpp#L60)** — `z_min`이 실제로 적용되지 않음. 다른 파일 코멘트들이 "크롭 박스 z[0.01, 0.3]"이라고 적은 것은 사실과 다름.
3. **[pc2_to_laserscan.cpp:56](ssupath-f1tenth-race-stack/perception/src/preprocessing/pc2_to_scan_cpp/src/pc2_to_laserscan.cpp#L56)** — 빈 각도 빈이 `+inf`가 아닌 `range_max`(7.0)로 채워짐.
4. **[tracking.cpp:496](ssupath-f1tenth-race-stack/perception/src/tracking/src/tracking.cpp#L496) `inFOV`** — bearing(차량 프레임)으로 `/scan`(livox 프레임) 인덱스를 계산해 90° 어긋남.
5. **[cluster_to_obstacle_node.cpp:213](ssupath-f1tenth-race-stack/perception/src/clustering/cluster_to_obstacle_cpp/src/cluster_to_obstacle_node.cpp#L213) TF 폴백 (B)** — 검출률 8배 붕괴의 유력 원인. 근본 해결은 lidarslam `use_odom:true`.

---
---

# PART 2. 실차 증상별 원인 및 파라미터 분석

**관측된 증상**

- **A.** 고속에서 장애물을 못 보거나 거의 부딪히기 직전에 봄
- **B.** 직선 구간에서 벽을 장애물로 인식
- **C.** 가끔 지면을 장애물로 인식

세 증상은 서로 독립이 아니라 **같은 파라미터들의 반대편 끝**입니다. 그래서 하나를 고치면 다른 하나가 나빠지는 구조입니다.

## A. 증상 → 원인 지도

| | A. 고속 미탐지/지연탐지 | B. 직선에서 벽을 장애물로 | C. 지면을 장애물로 |
|---|---|---|---|
| ① passthrough | ★★★ `y_min=-7` (물리적 상한) | – | `z_min` 미적용 |
| ② ray_ground | ★★★ 원거리 장애물 점을 지면으로 흡수 | ★ | ★★★ `general_max_slope` 근거리 여유 없음 |
| ③ euclidean | ★★ `min_cluster_size`, `tolerance` | ★★★ 직선에서 벽이 조각남 | ★ 지면 유령이 클러스터가 됨 |
| ④ cluster_to_obs | ★★★ TF 폴백 위치오차 | ★★★ `boundaries_inflation=0.03` | ★ |
| ⑤ pc2_to_scan | – | ★ (FTG가 보는 벽) | ★ |
| ⑥ tracking | ★★ `nb_meas>6`, `max_dist=0.5` | ★★ 유령 게이트가 `nb_meas` 뿐 | ★ |
| **로컬라이제이션** | ★★★ | ★★★ | ★ |

> **결론부터**: A와 B의 최대 기여자는 퍼셉션 파라미터가 아니라 **측위 오차**일 가능성이 높습니다(§C).

---

## B. 단계별 파라미터 분석

### ① passthrough_filter

#### `y_min = -7.0` — 증상 A의 **하드 리밋**

livox 프레임 `−y`가 전방이므로 **전방 7 m 밖에는 점 자체가 없습니다.**

| 차속 | 7 m 도달 시간 | ⑥의 `nb_meas>6` 게이트 후 남는 시간 |
|---|---|---|
| 3 m/s | 2.33 s | ~2.1 s |
| 5 m/s | 1.40 s | ~1.2 s |
| 7 m/s | 1.00 s | ~0.8 s |

여기서 다시 ②~④의 원거리 손실(6~7 m 구간 연속성 28%)을 빼면, 고속에서 실제로 안정적으로 보이는 거리는 5 m 안쪽입니다. 7 m/s면 **0.7 s**. 이게 "거의 부딪히기 직전에 본다"의 산술적 정체입니다.

- **`y_min`을 −10 ~ −12로 키우면**: 탐지 거리 확보. ④의 `max_viewing_distance=10.0`과 ⑥의 `dist_infront=7.0`이 이미 7 m보다 크게 잡혀 있으므로 **`y_min`을 먼저 키워야 그 값들이 의미를 갖습니다**.
- **부작용**:
  - 점 개수 증가 → ③ KdTree 비용 증가(7→12 m면 면적 기준 약 1.7배)
  - **7 m 밖의 벽이 새로 시야에 들어와 B가 악화**됩니다. 멀리 있는 벽일수록 입사각이 얕아 §③의 "조각남"이 심해집니다.
  - 원거리는 점밀도가 낮아 `min_cluster_size=3`을 못 넘겨 실질 이득이 적을 수 있습니다.
  - → **②를 먼저 고치지 않으면 `y_min`만 키워봐야 유령만 늡니다.**

#### `z_max = 0.3`

라이다 원점 기준 높이. 실제 장착 높이를 `h`라 하면 지면 위 `0.3 + h`까지 봅니다.

- **키우면**: 키 큰 장애물의 윗부분까지 확보 → 점 개수 증가(A 개선). **부작용**: 사람 상체·기둥·펜스 상단이 들어와 B·C 유령 증가.
- **줄이면**: 유령 감소. **부작용**: 원거리 장애물은 이미 점이 5~6개뿐이라 A가 급격히 악화.

#### `z_min = 0.01` — **현재 코드에서 적용되지 않음**

증상 C(지면 오인)에 대한 **가장 값싼 방어선이 꺼져 있는 상태**입니다.

- **살리면**: 지면 점이 ②에 도달하기 전에 통째로 제거됩니다. C가 크게 줄고, ② 입력 점 수가 줄어 ②의 오분류 기회 자체가 사라집니다.
- **부작용 (중요)**: 지면은 라이다 프레임에서 `z ≈ −h`인데, `z_min=0.01`은 **지면 위 `h+0.01` m 아래를 전부 잘라냅니다.** 원거리 장애물은 아래쪽 몇 점만 보이는 경우가 많아 A가 악화될 수 있습니다. 또 브레이킹 피치·범프로 차체가 기울면 지면이 `z_min` 위로 올라와 방어선이 뚫립니다.
- **권장**: `z_min`을 지면보다 조금 위(예: `−h + 0.03`, TF의 h=0.07이면 `−0.04`)로 잡아 **지면만 걷어내고 장애물 밑동은 남기는** 값으로 튜닝. 살리기 전에 `/passthrough/lidar`의 지면 점 z 중앙값을 실측해 `h`를 확정하세요.

---

### ② ray_ground_filter — **A와 C가 정면충돌하는 지점**

#### 먼저 알아야 할 구조적 사실

이 필터의 두 문턱은 이렇게 생겼습니다.

```
지역 문턱:  height_threshold = max( tan(local_max_slope) × Δr , min_height_threshold )
전역 문턱:  general_height_threshold = tan(general_max_slope) × r      ← 센서 원점 기준!
```

**전역 문턱이 "센서 원점"에서 재는 값인데 지면은 `z = −h`에 있습니다.** 여기서 두 가지가 동시에 따라 나옵니다.

##### (1) 근거리: 지면이 장애물로 찍히는 하한 반경이 생깁니다 (증상 C)

지면 점이 지면으로 인정받으려면 `h ≤ tan(general_max_slope) × r`, 즉

```
r ≥ h / tan(8°) = 7.12·h
```

한편 MID-360의 하단 FOV는 −7°이므로 지면이 처음 보이는 반경은 `h / tan(7°) = 8.14·h`.

> **여유가 겨우 14%입니다.** 브레이킹으로 코가 1°만 숙여도 유효 하단 FOV가 −8°가 되어 지면 최근접 반경이 `h/tan(8°)`로 내려오고, 그 순간 **각 부채꼴의 첫 지면 점이 통째로 "장애물"로 분류**됩니다. 게다가 이 유령은 라이다에서 `≈7·h` (h=0.07이면 0.5 m, h=0.2면 1.4 m) 앞, 즉 `d ≈ 0` — **레이스라인 정중앙**에 뜹니다. ④의 경계 필터를 무조건 통과하고 ⑥의 `checkInFront`도 통과합니다.
>
> 이게 "가끔 지면을 장애물로 인식"의 유력한 정체이며, **동시에 "부딪히기 직전에 장애물이 튀어나온다"의 일부**일 수 있습니다. 급제동/범프 순간에만 나타나므로 "가끔"인 것과도 맞습니다.

##### (2) 원거리: 전역 문턱이 무력화됩니다 (증상 A)

`r = 6 m`에서 `tan(8°)×6 = 0.84 m`. 그런데 ①의 크롭이 `z ≤ 0.3`이므로 **전역 검사는 통과하지 않을 수가 없습니다.** 즉 `r > 0.3/tan(8°) ≈ 2.1 m` 밖에서는 전역 검사가 아무것도 걸러내지 못합니다.

그러면 재분류 분기가 이렇게 됩니다:

```cpp
if (points_distance > reclass_distance_threshold_ && |z| <= general_height_threshold)
    current_ground = true;     // ← 2.1 m 밖에서는 사실상 "간격만 벌어지면 지면"
```

지면 점의 반경 간격은 `Δr ≈ r²·Δθ / h`로 **거리 제곱에 비례해 폭발**합니다(Δθ = 수직 각분해능 ≈ 0.2° = 0.0035 rad):

| r | Δr (h=0.07) | Δr (h=0.2) |
|---|---|---|
| 2 m | 0.20 m | 0.07 m |
| 4 m | 0.80 m | 0.28 m |
| 6 m | 1.80 m | 0.63 m |
| 7 m | 2.45 m | 0.86 m |

**장애물의 앞면 최하단 점은 직전 지면 점과 이만큼 떨어져 있으므로 `points_distance > 0.1`을 항상 만족 → 지면으로 흡수됩니다.** 그리고 한 번 흡수되면 `prev_ground = true`가 되어, 그 다음 점이 `|Δz| ≤ min_height_threshold(0.02)`이기만 하면 연쇄로 지면이 됩니다.

수직 면은 반경이 거의 같아서 **반경 정렬이 z 순서를 무작위로 섞습니다.** 높이 0.3 m 면에 점 N개가 무작위 순서로 늘어서면 인접 쌍의 `|Δz| ≤ 0.02`일 확률이 약 13%. 즉 **장애물당 "최하단 1점 + 나머지의 약 13%"가 사라집니다.**

6~7 m에서 물체당 점이 5~6개라는 저장소 실측에 대입하면 **1~2점이 날아가 3~4점이 남고, `min_cluster_size=3` 문턱에서 프레임마다 깜빡입니다.** yaml이 관찰한 "4→3→5로 널뛰며 3개인 순간 물체가 통째로 사라진다"가 정확히 이 현상입니다.

#### 파라미터별 분석

| 파라미터 | 현재 | 관련 증상 | 올리면 | 내리면 |
|---|---|---|---|---|
| `general_max_slope` | 8.0 | **C(근거리)**, A(원거리) | 근거리 지면 유령↓ (하한반경 `7.12h`가 작아져 피치 여유 확보). **부작용: 원거리 전역문턱이 더 커져 흡수 악화 → A 악화** | 원거리 흡수는 다소 개선. **부작용: 하한반경이 커져 근거리 지면이 통째로 장애물 → C 폭발.** `tan(θ)·r > h`를 못 지키면 즉시 터짐 |
| `local_max_slope` | 6.0 (코멘트 "2~3이 정배") | A, C | 지면 판정이 관대 → C↓. **부작용: `tan(6°)·Δr`이 원거리에서 0.19 m(h=0.07)까지 커져 낮은 장애물이 통째로 지면 → A 악화** | 장애물 보존↑(A 개선). **부작용: 요철·타이어 자국·범프에서 지면이 장애물로 → C 악화** |
| `initial_max_slope` | 3.0 (코멘트 "1이 정배") | C | 각 부채꼴 첫 점이 지면으로 인정받기 쉬워짐 → C↓ | **부작용: 첫 점이 재분류 경로로 넘어가고, 거기서도 실패하면 장애물 → C 악화** |
| `min_height_threshold` | 0.02 | **A ↔ C 직결** | 흡수 연쇄 심화 → **A 악화** | **위에서 계산한 "13% 흡수"가 줄어 A 개선.** 평탄 지면의 z 노이즈는 `σ_r·sin(atan(h/r))` ≈ 0.7 mm 수준이라 **0.01까지는 저위험**. 부작용: 요철/피치가 있는 노면에서 C 악화 |
| `reclass_distance_threshold` | 0.1 | **A** | 재분류-지면 경로가 좁아져 장애물 점 보존↑. 실제 지면은 3 m 밖에서 Δr ≥ 0.45 m(h=0.07)라 **0.3까지 올려도 지면 판정에는 영향 없음** → 저위험 개선책. **부작용: 근거리(r<2~3 m) 폐색 그림자 뒤 지면이 재분류를 못 받아 → C 악화** | 흡수 악화 |
| `concentric_divider_distance` | 0.0 | – | "가까운 두 점은 직전 판정 유지"가 켜져 판정이 안정화되지만, 장애물 시작점을 놓칠 위험 | 비활성 상태 유지 |
| `radial_divider_angle` | 1.0 | A | 부채꼴이 커져 점이 많아짐 → 순서 섞임 심화, 판정 불안정 | 부채꼴당 점이 3~4개로 줄어 `prev` 기반 판정이 통계적으로 무의미 |
| `use_vehicle_footprint` | false | C | true로 켜면 첫 점의 `prev_radius`를 차체 경계로 잡아 근거리 오분류 완화. **단 `min_x/max_x/min_y/max_y`가 ±0.01(2 cm 상자)로 무의미하므로 실제 차체 치수로 채워야 함** | – |

#### ② 요약 권고

이 단계는 **파라미터만으로는 A와 C를 동시에 못 잡습니다.** 근본 원인이 "전역 문턱이 센서 원점 기준"이기 때문입니다. 두 가지 우회로:

1. **저위험**: `min_height_threshold` 0.02 → 0.01, `reclass_distance_threshold` 0.1 → 0.3. (A 개선, C는 근거리에서만 소폭 악화)
2. **근본**: `Filter` 기반 클래스의 `input_frame`을 **지면 높이에 원점이 있는 프레임**으로 지정. 그러면 지면이 `z ≈ 0`이 되어 `general_max_slope`를 2~3°까지 낮출 수 있고, 전역 검사가 전 거리에서 실제로 작동합니다. **부작용**: 출력 frame_id가 바뀌므로 `output_frame`을 `livox_frame`으로 되돌려 놓아야 ④의 `input_frame: livox_frame`이 유지되고, 클라우드마다 TF 조회가 추가되어 §C의 TF 지연 문제에 노출됩니다.

> **작업 전 필수 실측**: `h`(라이다의 실제 지면 위 높이). TF는 `base_link + 0.07`이라지만 두 런치 파일이 서로 다릅니다. 위 계산은 전부 `h`에 비례하므로 이 값이 틀리면 결론이 바뀝니다. `/passthrough/lidar`에서 지면 점 z의 중앙값을 찍어 `−h`를 확정하세요.

---

### ③ euclidean_cluster — **왜 하필 직선에서 벽인가**

#### 직선에서 벽이 조각나는 메커니즘 (증상 B의 핵심)

벽 위 인접 점의 간격은 **입사각**에 지배됩니다:

```
벽면 점 간격 = r · Δθ / sin(입사각)
```

직선 구간에서 벽은 진행 방향과 거의 평행 → 입사각이 매우 작습니다. 벽이 횡방향 0.6 m 옆에 있을 때:

| 전방거리 y | 입사각 | 벽면 점 간격 | `tolerance=0.2` 대비 |
|---|---|---|---|
| 3 m | 11.3° | 0.054 m | 연결 ✓ → 긴 클러스터 → 크기필터가 제거 ✓ |
| 5 m | 6.8° | 0.15 m | 아슬아슬하게 연결 |
| 6.5 m | 5.3° | **0.25 m** | **끊김 ✗ → 3점짜리 조각들** |

**끊긴 조각은 각각 0.8 m보다 작으므로 크기 필터를 통과하고, 점이 3개면 `min_cluster_size`도 통과합니다.** 코너에서는 벽이 정면에 가까워 입사각이 커지고 → 점이 촘촘 → 하나의 긴 덩어리 → 크기 필터가 제거. **그래서 코너가 아니라 직선에서만 벽이 뜹니다.**

#### 파라미터별 분석

| 파라미터 | 현재 | 관련 증상 | 올리면 | 내리면 |
|---|---|---|---|---|
| `tolerance` | 0.2 | **A ↔ B 직결** | 벽 조각들이 하나로 병합 → 크기 필터가 제거 → **B 개선**. **부작용: 저장소 실측상 A가 크게 악화** — 0.2→0.3에서 6~7 m 연속성 23.1%→13.3%. 원거리 장애물이 주변 벽/바닥 점과 한 덩어리로 흡수되어 크기 필터에 같이 걸려 죽음(근거리 클러스터 점 중앙값 53→71이 증거) | 조각화 심화 → **B 악화**. 원거리 장애물도 쪼개져 A도 악화. 양쪽 다 나쁨 |
| `min_cluster_size` | 3 (이전 4) | **A ↔ B 직결** | 4로 되돌리면 3점짜리 벽 조각이 사라져 **B 개선**. **부작용: 6~7 m 연속성 28.2%→23.1%(−18%), A 악화** — 이 값을 3으로 내린 게 바로 A를 살리려던 조치였음 | 이미 최저 수준. 2로 내리면 유령이 폭증 |
| `max_x/max_y/max_z` | 0.8 (이전 0.6) | **A ↔ B** | 큰 벽 덩어리까지 통과 → B 악화 | 0.6으로 되돌리면 벽 덩어리 제거↑(B 개선). **부작용: 15° 이상 틀어진 규격 장애물(축정렬 박스 0.612 m)이 버려짐 → A 악화.** A/B: 0.6은 5~6 m 연속성 39.9%, 0.8은 44.2% |
| `max_cluster_size` | 750 | B | 큰 벽까지 클러스터로 인정 | 큰 벽이 결과에서 제외되므로 벽 제거에 쓸 수 있지만, **벽이 조각나 있으면 아무 효과 없음** |
| `use_height` | false | A, B | true면 3D 클러스터링 → 벽(높은 면)과 낮은 장애물이 z로 분리되어 **B 개선 가능**. **부작용: 라이다 스캔선 간격 때문에 같은 물체의 상하 점이 분리되어 원거리 장애물이 쪼개짐 → A 크게 악화.** 원래 false로 둔 이유 | – |

#### ③ 요약 권고

`tolerance`와 `min_cluster_size`는 **A와 B를 정확히 반대로 움직입니다.** 두 값으로 동시 해결은 불가능하며, 저장소의 A/B 기록이 이미 그 벽에 부딪힌 상태입니다. 여기서 나가는 유일한 길은 **크기가 아니라 형상으로 벽을 구분**하는 것입니다:

- 클러스터의 **주성분 비율(PCA λ₁/λ₂) 또는 직선 적합 잔차**를 계산해 "가늘고 긴 것 = 벽 조각"을 걸러내기. 3점만 있어도 공선성 판정은 가능합니다.
- 또는 클러스터 중심의 **트랙 경계까지 거리**를 ④에서 이미 계산하므로, "경계에서 X cm 이내 + 점 3개 + 세장형"이면 벽으로 라벨링.

---

### ④ cluster_to_obstacle — 증상 B의 직접 방아쇠

#### `boundaries_inflation = 0.03` — **B의 1순위 용의자**

yaml이 스스로 경고합니다: *"부작용: 너무 줄이면 벽 점군 자체가 장애물로 잡혀 유령이 생긴다. 유령이 늘면 0.05로 되돌리고."* 그리고 실측 트랙 반폭은 **좌 0.710 / 우 0.509 m**입니다.

```
유효 경계:  좌 0.680 m,  우 0.479 m
```

우측 여유가 **3 cm**입니다. 그런데 §C에서 볼 측위의 주행 중 프레임간 튐이 **33.5 cm**입니다. 벽이 `d = 0.479` 안쪽으로 들어오는 데 필요한 오차보다 측위 오차가 10배 큽니다. **파라미터가 감당할 수 있는 범위를 넘어섰습니다.**

- **0.03 → 0.05~0.07로 되돌리면**: 벽 유령 감소(B 개선). **부작용은 명확히 문서화돼 있습니다** — 탐지 장애물의 횡위치 분포에서 경계 대비 0.85~0.95 구간에 31개, 전체의 11.5%가 문턱 5 cm 이내였습니다. 즉 **벽에 붙은 실제 장애물이 다시 통째로 무시됩니다.** 이건 A(못 봄)의 또 다른 형태입니다.
- **비대칭 적용을 고려하세요**: 좌 0.710 / 우 0.509로 반폭이 다른데 동일한 값을 빼고 있어, 우측이 비율상 2배 손해(14% vs 10%)를 봅니다.

#### `max_viewing_distance = 10.0`

- **현재 무의미합니다.** ①의 크롭이 7 m라 10 m 밖 클러스터는 존재할 수 없습니다.
- **줄이면(예: 5.0)**: 위치오차로 튄 장애물이 걸러져 유령↓. **부작용: TF 오차가 낀 정상 장애물도 같이 버려져 A 악화.** 저장소가 5.0→10.0으로 올린 이유가 정확히 그것입니다.
- **`y_min`을 −12로 키운다면** 이 값도 같이 재검토해야 합니다.

#### TF 폴백 — **A의 1순위 용의자**

파라미터가 아니라 코드 로직입니다. 저장소 실측:

```
검출률:  1~2 m/s 30.6%  |  2~3 9.5%  |  3~4 8.5%  |  4~5 3.3%  |  5+ 3.7%
바로 앞단 /clusters 는 속도 무관하게 정상 (2.5~2.9개/프레임)
```

**붕괴 지점이 2~3 m/s 사이**입니다. TF 지연 0.2 s × 2.5 m/s = **0.5 m** — ⑥의 `max_dist`와 정확히 같은 값이고, 트랙 반폭(0.48~0.68 m)과도 같은 크기입니다. 이 속도를 넘는 순간:

1. 위치오차가 트랙 반폭에 육박 → `laserPointOnTrack`이 정상 장애물을 경계 밖으로 판정해 버림
2. 프레임마다 (A)/(B) 경로가 번갈아 걸리면 측정 위치가 0.5~1.3 m씩 점프 → ⑥의 연관 실패
3. 연관 실패 → 매 프레임 새 트랙 생성 → `nb_meas`가 6을 못 넘김 → **아무것도 발행되지 않음**

`transform()`의 타임아웃 0.05 s를 늘리면 (B) 진입 빈도가 줄지만, 그만큼 클러스터 콜백이 블로킹되어 파이프라인 전체가 지연됩니다(단일 스레드 spin). **근본 수정은 §C입니다.**

#### `min_obs_size` / `max_obs_size` (0.05 / 0.5)

**선언만 되고 사용되지 않습니다.** B 대응으로 되살릴 여지가 있습니다: 여기서는 `size = max(dx,dy)` 스칼라 하나만 보므로 `max_obs_size = 0.75` 정도로 걸면 큰 벽 덩어리를 한 번 더 거를 수 있습니다. **부작용**: ③에서 이미 0.8로 통과한 것이라 중복이고, 45° 틀어진 규격 장애물(0.707 m)과의 마진이 4 cm뿐이라 A 악화 위험.

---

### ⑤ pc2_to_laserscan

직접적인 원인은 아니지만 **증상 B·C를 하류로 증폭**시킵니다.

| 파라미터 | 현재 | 영향 |
|---|---|---|
| `range_max` | 7.0 | **빈 각도 빈이 `+inf`가 아니라 7.0으로 채워집니다.** 점이 없는 방향이 "7 m 벽"으로 보여 FTG/RL 컨트롤러가 헛 반응합니다. `+inf`로 바꾸면 정직해지지만, `inf`를 처리 못 하는 하류 코드가 있으면 터집니다 |
| `range_min` | 0.1 | ②가 만든 근거리 지면 유령(`r ≈ 7h`)이 그대로 통과합니다. `0.6~0.8`로 올리면 그 유령이 `/scan`에서 사라집니다. **부작용: 진짜로 코앞에 있는 장애물도 안 보임** |
| `angle_increment_deg` | 0.25 | 줄이면 분해능↑·연산↑. 원거리에서 한 빈에 점이 안 들어가 구멍이 늘어남 |

② 단계에서 지면이 장애물로 잘못 나가면 `/ground_segmentation/lidar` → `/scan`으로 그대로 흘러 **컨트롤러가 코앞에 벽이 있다고 판단**합니다. C가 주행에 미치는 영향은 `/perception/obstacles`보다 이 경로가 더 클 수 있습니다.

---

### ⑥ tracking

넘겨주는 yaml이 비어 있어 **전부 cpp 기본값**입니다.

| 파라미터 | 현재 | 관련 증상 | 올리면 | 내리면 |
|---|---|---|---|---|
| `nb_meas > 6` (하드코딩) | 6 | **A ↔ B 직결** | 순간적 유령이 걸러져 **B 개선**. **부작용: 발행 지연 증가 → A 악화** | 반응 빨라짐(A 개선). **부작용: 1~2프레임 유령이 그대로 발행 → B 악화**. ★ **이게 tracking 단계의 유일한 유령 게이트입니다** — `staticFlag`가 미정(`nullopt`)인 트랙도 `publish_static_=true`면 그대로 `/perception/obstacles`로 나가기 때문에, `sd_min_std`/`sd_max_std`를 조여도 유령은 안 걸러집니다 |
| `max_dist` | 0.5 | **A** | 연관 성공률↑ → 트랙 유지(A 개선). **부작용: 서로 다른 장애물이 하나로 병합, 유령이 실제 트랙을 가로챔** | 연관 실패↑ → 새 트랙 남발 → `nb_meas` 리셋 → 미발행 |
| `dist_infront` | 7.0 | A | 크롭이 7 m라 **현재 무의미**. `y_min`을 키운 뒤 같이 올려야 의미 생김 | 탐지 거리 축소 |
| `sd_ttl` | 20 (=0.5 s @40Hz) | **B** | 폐색 중 트랙 유지↑ | **유령이 빨리 사라짐 → B 개선**. 부작용: 실제 장애물이 1~2프레임 놓칠 때 트랙이 끊겨 `nb_meas` 재시작 → A 악화 |
| `sd_min_std / sd_max_std` | 0.16 / 0.22 | (제한적) | 정/동적 분류만 바꿈. 위 이유로 **유령 억제 효과는 거의 없음** | 동일 |
| `noMemoryMode` | true | B | 현재 true = 정적 장애물을 기억하지 않음 → 유령이 오래 안 남음. **B 관점에서 올바른 설정이므로 건드리지 말 것** | false로 하면 `inFOV` 경로가 켜지는데, 그 함수는 `/scan` 인덱스가 90° 어긋나 있어 **먼저 고쳐야 합니다** |
| `publish_static` | true | B | false로 하면 정적·미정 장애물이 `/perception/obstacles`에서 전부 빠짐 → 벽 유령 사라짐. **부작용: 진짜 정적 장애물도 안 나감 → 회피 불가.** 사실상 사용 불가 |
| `var_pub` | 1.0 | A | EKF 상대차 발행 게이트. 내리면 신뢰도 높은 추정만 발행(유령↓), 올리면 빨리 발행(반응↑·오탐↑) |

#### ⑥의 구조적 문제: 같은 측정을 4번 소비합니다

- 루프는 **40 Hz**, 측정(`/perception/detection/raw_obstacles`)은 라이다와 같은 **10 Hz**([msg_MID360_launch.py:11](ssupath-f1tenth-race-stack/state_estimation/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py#L11) `publish_freq = 10.0`).
- `meas_obstacles_`는 콜백에서 덮어쓸 뿐 **소비 후 비워지지 않습니다.** 따라서 한 라이다 프레임의 측정이 **4번 반복 사용**됩니다.

결과:

- `meas_s`/`meas_d`에 같은 값이 4번씩 쌓임 → `std_s`/`std_d`가 인위적으로 **작아짐** → 웬만하면 **정적으로 분류**. 벽 유령도 "안정적인 정적 장애물"로 승격됩니다(**B 악화**).
- `initializeDynamic`이 마지막 두 측정의 차분으로 초기 속도를 잡는데, 그 둘이 **동일한 중복값이라 `vs = 0`**. EKF 갱신의 `vs` 유한차분도 대부분 0 → 상대 속도가 체계적으로 과소평가되고 EKF가 뒤처집니다(**A 악화 — "본다고 해도 늦게 본다"**).
- 반대로 `nb_meas`는 4배 빨리 차서 게이트 통과는 빨라집니다(이 부분만 A에 유리).

**수정 방향**: `obstacleCallback`에 새 프레임 플래그를 두고 `updateTracking()`을 새 측정이 있을 때만 실행하거나, 아예 타이머를 없애고 측정 콜백 구동으로 바꾸세요. **부작용**: 루프가 10 Hz가 되면 `rate=40` 기준으로 잡힌 `dt`, `sd_ttl`, `ttl=40` 등이 전부 4배 틀어지므로 **동시에 재조정**해야 합니다.

---

## C. 파라미터로 못 고치는 것 — 측위 (증상 A·B의 진짜 상한)

[lidarslam.yaml](ssupath-f1tenth-race-stack/state_estimation/src/lidarslam_ros2/lidarslam/param/lidarslam.yaml)에 **이미 측정되고 문서화된, 그러나 적용되지 않은 수정 두 가지**가 있습니다.

### (1) `use_odom: false`

주석: *"휠 오도메트리 델타를 NDT 초기 추정치로 사용 (고속에서 정합 지연 방지)"*. [cluster_to_obstacle_node.param.yaml](ssupath-f1tenth-race-stack/perception/src/clustering/cluster_to_obstacle_cpp/config/cluster_to_obstacle_node.param.yaml)도 *"근본 수정은 lidarslam의 use_odom:true"*라고 지목합니다. **현재 false**입니다.

→ 켜면 §④의 TF 폴백 빈도와 지연이 줄어 A가 직접 개선됩니다.
**부작용**: `vesc_to_odom`의 `publish_tf(odom→base_link)`가 켜져 있어야 하고, 휠 슬립이 큰 상황에서는 초기 추정이 오히려 NDT를 잘못된 국소최적으로 끌 수 있습니다.

### (2) `enable_z_filter: false`

주석의 실측:

```
필터 없음: 종방향 오차 −0.889 m ± 0.108,  벽정합 47.8%
필터 적용: 종방향 오차 +0.262 m ± 0.061,  벽정합 66.3%
정지 프레임간 최대 튐 16.3cm → 5.2cm,  주행 중 79.4cm → 33.5cm
```

**주행 중 프레임간 위치 튐이 최대 79.4 cm입니다.** 이 값과 세 증상을 나란히 놓으면:

| 값 | 비교 대상 | 결과 |
|---|---|---|
| 튐 79.4 cm | `boundaries_inflation` 3 cm | 벽이 트랙 안으로 들어옴 → **증상 B** |
| 튐 79.4 cm | ⑥ `max_dist` 50 cm | 연관 실패 → `nb_meas` 리셋 → **증상 A** |
| 종방향 오차 −0.889 m | 크롭 7 m | 장애물의 `s`가 통째로 틀어짐 → **증상 A** |

**퍼셉션 파라미터를 아무리 조여도 이 크기의 오차는 못 이깁니다.** 그런데 이 필터가 꺼진 이유는 정확도가 아니라 *"RViz에서 맵이 거의 안 보이게 된다"* — 즉 시각화 문제입니다. 주석이 제시한 대로 "정합 target만 필터, `/map` 발행은 원본"으로 분리하면 부작용 없이 켤 수 있습니다.

---

## D. 권장 실험 순서

한 번에 하나씩, 오프라인 bag A/B로 검증하세요. **위쪽일수록 비용 대비 효과가 큽니다.**

| # | 조치 | 대상 증상 | 위험도 |
|---|---|---|---|
| 1 | `h`(라이다 실제 높이) 실측 + TF 두 파일 불일치 해소 | 전부 (모든 계산의 전제) | 없음 |
| 2 | `use_odom: true` | **A** | 낮음 (휠슬립 시 재검토) |
| 3 | `enable_z_filter` 코드 분리 후 `true` | **A, B** | 중 (코드 수정 필요) |
| 4 | ② `min_height_threshold` 0.02→0.01, `reclass_distance_threshold` 0.1→0.3 | **A** (C 소폭 악화) | 낮음 |
| 5 | ⑥ 측정 중복 소비 제거 (+ `rate` 재조정) | **A, B** | 중 (타이밍 파라미터 연쇄 조정) |
| 6 | ④ `boundaries_inflation` 0.03→0.05 (좌우 비대칭 적용) | **B** (벽 붙은 장애물 A 악화) | 낮음, 되돌리기 쉬움 |
| 7 | ① `z_min` 복구 (`−h + 0.03`) | **C** | 중 (원거리 A 악화 가능) |
| 8 | ③ 클러스터 세장형(PCA) 판별 추가 | **B** (파라미터 충돌 없이 해결) | 중 (신규 코드) |
| 9 | ① `y_min` −7→−10, ⑥ `dist_infront` 동반 상향 | **A** | 높음 (2~8 완료 후에만) |

## E. 진단용 계측 제안

원인 배분을 확정하려면 다음 세 개를 한 bag에서 동시에 로깅하세요.

1. **단계별 생존율**
   `/passthrough/lidar` → `/ground_segmentation/lidar` → `/clusters` → `/perception/detection/raw_obstacles` → `/perception/obstacles`의 **거리 구간별(1 m 단위) 개수**. 어느 단계에서 몇 %가 죽는지가 바로 나옵니다.
   ②에서 죽으면 지면 흡수 / ③이면 크기·개수 필터 / ④면 TF·경계 / ⑥이면 `nb_meas` 게이트.

2. **TF 폴백 카운터**
   `cluster_to_obstacle_node.cpp:218`의 `RCLCPP_DEBUG`를 카운터+`WARN_THROTTLE`로 바꿔 (B) 경로 진입률을 속도별로 기록. 이게 속도에 비례해 오르면 §C가 주범임이 확정됩니다.

3. **지면 유령 확인**
   `/ground_segmentation/lidar`를 RViz에서 보며 **급제동 순간에 차 앞 `r ≈ 7h` 부근에 호(arc) 모양 점군이 생기는지** 확인. 생긴다면 증상 C는 §②의 `general_max_slope` 근거리 여유 문제로 확정됩니다.
