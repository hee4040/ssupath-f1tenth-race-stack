# rl_controller

Isaac Lab(`dacerpp_isaaclab`)에서 학습한 **DACER++ 디퓨전 정책**을 실차에서
구동하는 컨트롤러. 학습 관측(58차원)을 실차 토픽으로 재구성해 30Hz 로
`/l1controller/control` 을 발행하고, 기존 `mux_controller` 가 그것을 `/drive` 로 내보낸다.
따라서 **조이스틱 오버라이드(LB 토글)와 `/e_stop` 이 그대로 살아 있다.**

**가중치와 코드는 전부 이 레포 안에 있다.** 학습 워크스페이스(`~/shared_dir/...`)를
참조하는 경로는 없다 (`models/README.md` 참조).

```
/livox/lidar        ─┐
/car_state/odom      ├─> rl_controller ─> /l1controller/control ─> mux_controller ─> /drive
/centerline_waypoints│        (30Hz)                                 (joy/e_stop 우선)
/perception/obstacles┘   (상대차 5개 특징, 선택)
```

## 실행

```bash
# 1) 측위/기반 (기존과 동일)
ros2 launch stack_master base_system_3D_launch.xml racecar_version:=NUC2 \
     map_dir:=lobby_0806 map_name:=lobby_0806 sim:=false rviz:=true

# 2-a) RL 주행 + 장애물/상대차 인지  ← 권장
ros2 launch stack_master time_trials_rl_launch.xml racecar_version:=NUC2 v_max:=5

# 2-b) RL 주행만 (인지 없음. 상대차 관측은 항상 0 = 미검출)
ros2 launch stack_master time_trials_launch.xml racecar_version:=NUC2 ctrl_algo:=RL v_max:=5
```

`ctrl_algo:=RL` 이면 `l1_controller` 대신 `rl_controller` 가 뜬다. `v_max` 는
**행동 -> 속도 매핑의 상한**이다(스로틀 +1 = v_max, -1 = v_min = 1.0 m/s).

`time_trials_launch.xml` 과 `base_system_3D_launch.xml` 어디에도 perception 체인이
없다 — 그래서 2-b 로 돌리면 `/perception/obstacles` 가 아예 발행되지 않는다.
`time_trials_rl_launch.xml` 이 그 체인을 얹은 판이다.

주행 전에 오프라인 점검:

```bash
ros2 run rl_controller check_rl_setup.py \
    --map ~/forza_ws/race_stack/stack_master/maps/lobby_0806 \
    --bag ~/forza_ws/race_stack/obs_debug_0806_1157
```

## 관측 (dacerpp_lab/racing_env.py `_observe_car` 와 동일 순서/정규화)

| 인덱스 | 내용 | 출처 |
|---|---|---|
| 0:32 | 스캔 32빔 / 10m (전방 ±135°) — **벽 + 장애물 + 상대차가 모두 섞여 들어온다** | `/livox/lidar` -> 높이밴드 + 자차박스 제외 + 방위각 섹터 최소거리 |
| 32 | 속력 / **10.0** (학습 정규화 상수, 주행 v_max 와 무관) | `/car_state/odom` twist |
| 33:35 | sin/cos(헤딩오차) | odom yaw - 중심선 접선각 |
| 35 | 횡오차 / 지역 반폭 (±1 = 벽) | 중심선 투영 |
| 36:41 | 전방 곡률 5개 (+5/15/30/60/90 idx = 0.75/2.25/4.5/9/13.5 m), **±2 클립** | 중심선 |
| 41:47 | 현재+전방 반폭 6개 / 2.5m | 중심선 |
| 47:51 | 직전 2스텝의 '명령' 행동 (지연 하 Markov 복원) | 노드 내부 |
| 51:56 | 상대차량 5개 `[rel_x/10, rel_y/10, (v자차-v상대)/10, gap_s/10, visible]` | `/perception/obstacles` 중 `is_static=False` 항목 |
| 56 | 요레이트 / 4.0 | odom `twist.angular.z` (EKF wz), 폴백 IMU |
| 57 | 횡속도 / 3.0 | odom `twist.linear.y` (carstate_3d 추정) |

### 장애물과 상대차는 다른 채널이다

학습(`dacerpp_lab/env_cfg.py` `obstacles_enabled` 주석)에서:

- **장애물**(맵에 없는 ≤50cm 물체, 대회에서 리셋마다 2~3개): 상태 채널이 **없다.**
  32빔 스캔에 footprint 를 오버레이할 뿐이다. 원문: *"별도 상태 채널을 안 써서
  obs_dim 불변(58) = 실차에 장애물 감지 모듈 불필요."*
  실차에서도 Livox 원본 클라우드에 물리적으로 잡히므로 이 노드의 의사 스캔이 그대로
  재현한다. **인지 노드가 없어도 장애물 회피는 동작한다.**
- **상대차**: 스캔에도 잡히고(overlay_opponent), 추가로 위 5개로 명시적으로 들어간다.
  이건 실차에서 따로 채워 줘야 한다.

perception 의 `tracking` 노드는 이 둘을 이미 구분해서 낸다 —
`/perception/obstacles` 안에서 `is_static=False` + `vs`/`vd` 를 가진 항목이
opponent EKF 로 추적 중인 동적 물체이고, 정적/미정 장애물은 `is_static=True` 다.
그래서 **인지 노드를 고칠 필요가 없고**, 그 출력에서 동적 객체만 골라 학습 포맷으로
바꾸면 된다(`obstacles_cb` / `opponent_features`).

`tracking` 의 s/d 기준선은 **레이스라인**(`/global_waypoints`)이고 RL 관측은
**중심선** 기준이라, 상대차를 일단 맵 좌표로 되돌린 뒤(`global_path.py`) 컨트롤러의
`TrackReference` 에 다시 투영해 `gap_s` 를 구한다.

`visible` 게이트는 학습(거리 <10m, |방위각| ≤135°, 벽에 가림 없음)을 따른다. 실차는
검출 자체가 LiDAR 라 가림이 이미 반영돼 있어 거리/시야각만 다시 건다. 미검출이면
5개 전부 0 — 학습에서 상대가 멀거나 가려졌을 때와 정확히 같은 입력이다.

#### 실차 인지와 학습 가정의 차이 (전부 '더 비어 보이는' 쪽이라 안전)

| 항목 | 학습 | 실차 perception |
|---|---|---|
| 검출 거리 | 10m (`scan_max_range`) | **7m** — 크롭 박스 `passthrough_filter_node.param.yaml` `y_min: -7.0` + `tracking` 의 `dist_infront: 7.0` |
| 뒤쪽 상대차 | ±135° 안이면 보임 | **안 보임** — `tracking::checkInFront` 이 `0 < ds < dist_infront` 만 발행 |
| 가림/미검출 | `visible=0` 즉시 | 놓친 뒤 최대 ~1초 EKF 외삽(`ttl=40 @40Hz`, `useTargetVel`)이 이어짐. 공분산 게이트(`var_pub`)가 스스로 끊는다 |

7~10m 구간과 뒤쪽 상대차는 '미검출(전부 0)'로 들어가는데, 이는 학습에서 상대가
10m 밖이거나 벽에 가려졌을 때와 **정확히 같은 입력**이다. 즉 정책이 없는 상대를
쫓는 방향의 오류는 나지 않는다. 거리를 늘리려면 크롭 박스부터 키워야 하고
(`dist_infront` 만 올려도 통과시킬 점군이 없다 — `stack_master/config/opponent_tracker_params.yaml`
의 2026-08-04 실측 기록 참조) 그건 CPU/오검출 트레이드오프가 따로 있으므로
여기서는 건드리지 않았다.

EKF 외삽 구간이 거슬리면 `opp_timeout` 을 줄이는 대신 `opp_min_speed` 를 올리는 쪽이
낫다(외삽 중에는 vs 가 `0.3 × 레이스라인 속도` 로 끌려간다).

### 기준선(중심선) 재구성

관측의 횡오차/폭/곡률은 학습의 '중심선 ± 반폭 대칭 트랙' 정의를 기준으로 정규화되어
있다. 학습에서 실측 맵은
`global_waypoints.json` 의 `centerline_waypoints` -> `convert_map_json.py` ->
`tracks.load_f1tenth_centerline()` 경로로 트랙이 되었으므로, `track_reference.py` 가
그 전처리(좌우 비대칭 대칭화, ds=0.15m 등호장 리샘플, 박스카 평활 2회, 곡률/근접 기반
폭 클립, 폭 평활, 벽 교차 제거)를 **그대로** 재현한다. 이 전처리를 생략하면 정책이
학습과 다른 스케일의 lateral/width 를 보게 된다.

기준선은 `/centerline_waypoints` (global_planner 가 5초마다 재발행) 로 받는다.
**주행 방향은 웨이포인트 순서와 같아야 한다** — 반대로 돌면 곡률 부호가 뒤집힌다.

### 스캔

Livox MID360 은 100ms 스윕이고 점마다 절대 타임스탬프가 들어 있다. 5m/s 면 스윕
동안 0.5m 를 움직이므로 점별로 측정 시각의 포즈로 되돌린 뒤 현재 차체 좌표로 옮긴다
(`scan_deskew: true`). 포즈 버퍼가 라이다 시각을 못 덮으면 경고 후 디스큐를 생략한다.

lobby_0728 bag 실측 기준값 (`config/rl_controller.yaml`):
- 높이 밴드 `z ∈ [0.02, 0.30]` (base_link) — 지면은 z≈-0.05m, 덕트 벽은 0~0.3m
- 자차 반사 제외 박스 `x∈[-0.40,0.55], |y|<0.20` — 원점 부근 전장품 반사가 상시 잡힌다
- 이 설정에서 빔 점유율 99.3%, 0.3m 미만 오검출 0.5%

**벽 높이가 다른 코스에서는 밴드를 다시 잡을 것.** `check_rl_setup.py --bag` 이
밴드별 통계를 뽑아준다. 주행 중에는 `/rl_controller/scan` (LaserScan) 을 RViz 로 확인.

## 안전

- 측위 0.2s / 라이다 0.5s 이상 끊기면 **speed 0 명령**을 계속 발행한다.
- 추론 예외 시에도 정지 명령. 노드 종료 직전에도 정지 명령을 한 번 보낸다.
- 정지 상태에서 복구되면 `주행 시작` 로그가 뜬다.
- 정책은 **완전 정지를 학습하지 않았다**(`v_min = 1.0 m/s`). 시동 즉시 1m/s 이상을
  명령하므로 반드시 조이스틱 데드맨/e-stop 을 잡고 시작할 것.

## 파라미터

`config/rl_controller.yaml` 참조. 자주 만질 것:

| 파라미터 | 기본 | 설명 |
|---|---|---|
| `checkpoint` | `20260805/pow.pt` | 상대경로면 패키지 `models/` 기준. **`pow.pt` 가 실차 배포 대상**(학습 car_b) — `models/README.md` 참조 |
| `curv_clip` | 2.0 | 곡률 관측 클립. 20260805 세대부터 ±2. 구 `models/cvar.pt` 를 쓰면 1.0 으로 되돌릴 것 |
| `v_max` | 5.0 (launch 인자) | 행동->속도 상한 |
| `speed_mode` | `scale` | `scale`=v_min~v_max 로 선형 매핑, `clip`=학습대로 v_min~10 매핑 후 v_max 로 절단 |
| `opponent_enabled` | `true` | 상대차 관측 사용. perception 이 안 떠 있으면 자동으로 0(미검출) |
| `opp_min_speed` | 0.3 | 이보다 느린 '동적' 물체는 유령으로 보고 무시 |
| `opp_timeout` | 0.4 | 이보다 오래 갱신 없으면 미검출로 되돌림 |
| `num_action_candidates` | 1 | >1 이면 QVN 으로 후보 중 최선 선택. 3이면 추론 4.2 -> 11.4ms |
| `scan_z_min/max` | 0.02/0.30 | 높이 밴드 |
| `device` | `cpu` | 이 젯슨의 pip torch(2.11)는 sm_87 커널이 없어 CUDA 실행이 실패한다. CPU 추론 5ms 로 충분 |

### 학습 코드가 바뀌면 같이 볼 것

관측 포맷을 정하는 곳은 `dacerpp_lab/racing_env.py` 의 `_observe_car` 와
`dacerpp_lab/env_cfg.py` 의 `RacingCfg` 딱 두 곳이다. 노드 기동 시
`n_beams / curv_lookahead / act_hist_len` 로 계산한 차원과 체크포인트의 `obs_dim` 이
다르면 죽으므로 **차원이 바뀌는 변경은 자동으로 잡힌다.** 위험한 건 차원은 그대로인데
정규화/클립만 바뀌는 경우다 (20260805 의 곡률 ±1 → ±2 가 정확히 그 경우였다).

## ★ 새 맵은 주행 전에 반드시 시뮬로 돌려볼 것

```bash
ros2 run rl_controller check_rl_setup.py --map stack_master/maps/<맵이름> --rollout
```

학습 타이어 모델을 이식한 폐루프 시뮬(`rl_controller/offline_sim.py`)로 여러 시작점 ×
여러 v_max 를 돌려 완주율과 **실패 지점 s** 를 보고한다. lobby_0730 사고를 실차와
같은 지점(s≈3.4m), 같은 속도 문턱(v_max 2.0 통과 / 2.5 실패)으로 재현한다.

> **곡률/폭 임계값으로 맵을 걸러내려던 시도는 실패했다.** 학습에 쓰인 절차 생성 트랙
> 60종을 실제로 재생성해 비교하면 `|κ|max` 중앙 1.44 / 최대 1.84, 급코너 방향 반전
> 최소간격 중앙 1.05m 로 **사고가 난 lobby_0730(1.85 / 1.20m)보다 오히려 빡빡하다.**
> 즉 이 맵은 형상 지표상으로는 학습 분포 밖이 아니며, 임계값 기반 판정은 신뢰할 수
> 없다. 돌려보는 것이 유일하게 맞는 검사다.

### 실주행 사고 (lobby_0730, v_max=2.0)

s≈3.8m 에서 우코너인데 정책이 좌 풀락(+0.42)을 내며 벽에 충돌. bag 재생으로 확인한 것:

- **이식 문제 아님** — bag 의 포즈/라이다로 관측을 재구성해 정책에 넣으면 실제 기록된
  `/drive` 명령이 그대로 재현된다(조향 std 0.207 vs 재생 0.212).
- **오프라인 시뮬에서 같은 지점(s≈3.4)에서 같은 방식으로 실패**한다. 속도를 낮춰도
  안정적으로 해결되지는 않는다: 8 스타트 기준 v_max 1.2/1.5/1.8/2.0/2.2/2.5 →
  완주 1/8, 0/8, 0/8, 3/8, 1/8, 0/8. 실패 지점은 전부 s=1.8~3.8 (V 구간).
  (4 스타트/20초로 줄이면 v_max=2.0 이 4/4 가 되는 등 표본 편차가 크다 — 통과율
  자체보다 '실패가 s=1.8~3.8 한 곳에 몰린다'는 사실이 신뢰할 수 있는 신호다.)
- V 구간에서 정책 출력은 **±0.42 포화 + 256샘플 표준편차 0.00** 이다.
  실제 bag 상태에서 곡률 채널이나 스캔 채널을 중립화하면 조향 부호가 정상으로 돌아온다.
- 좌우 흔들림(조향 부호반전 4~5회/초)은 이 정책의 고유 특성이라 `hall` 처럼 잘 도는
  트랙에서도 같은 빈도로 나타난다. 폭 1.0~1.5m 코리도에서 눈에 띄게 보일 뿐이다.
- 효과 없던 완화책(오프라인 검증): 조향 EMA, 후보 샘플 3개, 코리도 폭 보정.

### 별건: 맵 코리도가 실제보다 좁다

bag 스캔과 맵을 비교하면 실제 덕트 간격이 맵의 `d_left+d_right` 보다 **0.24~0.61m
넓다**(전 구간). 학습에서는 스캔과 `lat/hw` 가 같은 벽을 가리키므로 이 불일치는
학습에 없던 것이다. 사고의 직접 원인은 아니었지만(오프라인에서 재현 시 오히려 통과율이
올랐다) 맵 품질 문제이므로 재매핑 시 확인할 것.
