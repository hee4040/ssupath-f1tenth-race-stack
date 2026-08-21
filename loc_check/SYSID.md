# 연습장 물리계수 실측 절차 (lobby_0819 실패분석 PART 2)

대상: `dacerpp-isaaclab/dacerpp_lab/env_cfg.py` 의 `TireModelCfg` / `RacingCfg`.
com_height(2.5 기울임법)은 이번 회차에서 제외한다 — 아래 M2 가 그 대체 경로를 겸한다.

---

## 0. 시작 전에 (10분)

### 0.1 반드시 알아둘 것 — 보고서와 실제 코드가 다른 지점

| 보고서 서술 | 실제 |
|---|---|
| "Ctrl+C 가 유일한 정지 수단" | 틀림. `vesc_driver.cpp:361,408` 에 조이스틱 e-stop 이 있다. **Circle(버튼 2)** 을 누르면 speed/current/duty 를 전부 0 으로 막고 래치한다. **Triangle(버튼 3)** 으로 해제. base_system 만 띄운 상태에서도 살아 있다. |
| "throttle_interpolator 는 주석만 풀면 켜진다" (`bringup_3D_launch.py:150`) | 부족하다. `ackermann_to_vesc` 는 `commands/motor/speed` 로 **직접** 발행하는데(`ackermann_to_vesc.cpp:78`) 인터폴레이터의 입력은 `commands/motor/unsmoothed_speed` 다. 주석만 풀면 인터폴레이터는 아무것도 못 받고 원래 명령이 그대로 VESC 로 간다. `ackermann_to_vesc_node` 에 remapping 을 같이 넣어야 한다(§4). |
| `v_max:=10` 이면 10 m/s 명령 | joy 데드맨(L1)을 쥐면 `joy_teleop` 가 **/drive 로 직접** 발행한다(`topic_name: drive`, install 본). 즉 조이스틱과 컨트롤러/이 툴이 같은 토픽을 두고 싸운다. 그래서 이 툴의 데드맨은 **R1(5)** 로 잡았다. |

### 0.2 어떤 종목이 측위를 필요로 하나

종목마다 다르다. 속도를 pose 미분으로 내야 하는 것만 맵이 필요하다
(휠 오도메트리는 `ERPM/speed_to_erpm_gain` 이라 게인이 의심스러운 지금은 순환 논리).

| 종목 | 측위 | 이유 |
|---|---|---|
| M0 ERPM 게인 | **필요** (또는 줄자) | pose 거리와 비교하는 게 핵심. 줄자로 대체 가능 |
| M1 alpha_char | **필요** | 횡속도(beta)를 pose 에서만 얻는다 |
| M1 mu | 불필요* | 콘 원 반경 R 을 알면 `a_lat = wz^2·R` — 자이로만으로 된다 |
| M2 f_drive/f_brake | **필요** | 지면속도가 없으면 휠슬립을 못 봐서 힘/마찰 한계가 안 갈린다 |
| M3 k_drive | 불필요 | ERPM 시정수. 게인이 틀려도 tau 는 안 변한다 |
| M4 c_roll | 불필요 | 감속 기울기. 게인 오차만큼만 비례해서 틀린다 |
| M5 조향 t63 | 불필요 | 자이로 요레이트만 쓴다 |

**측위가 필요 없는 종목**은 훨씬 가벼운 쪽으로 충분하다:
```bash
ros2 launch f1tenth_stack bringup_3D_launch.py     # 조이스틱 + VESC + 라이다만
```
설치본 `f1tenth_stack/config/vesc.yaml` 도 게인 3576 / 조향 −0.65 로 NUC2 와 같고,
Circle e-stop 은 `vesc_driver.cpp:74-77` 코드 기본값이라 yaml 에 없어도 동작한다. 확인해 뒀다.
분석할 때 `--no-pose` 를 붙이면 된다 — 붙이면 무효인 항목을 알아서 건너뛰고 그 이유를 찍는다.

\* mu 를 자이로만으로 낼 때는 **운전자가 콘 원을 손으로 따라가야 한다.**
`circle` 모드는 조향각을 고정하므로 슬립이 커질수록 반경이 커진다 — 그 주행에 `--radius` 를
쓰면 틀린다(pose 가 있으면 툴이 실제 반경과 비교해서 경고해 준다).
콘 원 방식은 조이스틱으로 몰면서:
```bash
python3 ~/forza_ws/race_stack/loc_check/measure_grip.py --live --radius 2.0
```
`mu` 가 실시간으로 찍히고 최대값이 갱신된다. **mu 는 R 에 정비례하므로 R 을 줄자로 잴 것.**

> ★**조이스틱 속도 상한을 먼저 올려야 한다.** 설치본
> `install/f1tenth_stack/share/f1tenth_stack/config/joy_teleop.yaml` 의
> `human_control.axis_mappings.drive-speed.scale` 이 **1.0** 이다 = 스틱을 끝까지 밀어도 1 m/s.
> R=2 m 원에서 1 m/s 면 `mu = 1/(9.81x2) = 0.05` 라 한계 근처에도 못 간다.
> mu 1.0 을 보려면 `v = sqrt(1.0 x 9.81 x 2.0) = 4.4 m/s` 가 필요하니 `scale` 을 5.0 으로 올리고
> 런치를 재시작할 것. (`sync_config.sh` 는 stack_master/perception 만 다루므로 f1tenth_stack
> 설치본은 직접 고쳐야 한다. 측정이 끝나면 되돌릴 것 — 수동주행이 5배 빨라진다.)

### 0.3 툴체인 자기검증 (차 없이)
```bash
cd ~/forza_ws/race_stack/loc_check
python3 sysid_selftest.py            # 계수를 아는 가짜 차 -> 분석기가 되찾는지 확인
```
`mu 0.95 -> 0.950`, `alpha_char 0.075 -> 0.080`, `k_drive 40 -> 39.0`,
`f_brake_max 14.2 -> 14.6`, `c_roll 0.015 -> 0.0145` 가 나오면 정상이다.

### 0.3b 레이스라인이 아직 없는 새 장소라면

`base_system_3D_launch.xml` 은 `stack_master/maps/<map_dir>/` 에
`global_waypoints.json` / `speed_scaling.yaml` / `ot_sectors.yaml` 이 있다고 전제한다.
`slam.launch.py` 로 `map.pcd` 만 만든 단계에서는 그게 없어서
`global_trajectory_publisher` 가 "not publishing" 경고만 내고, `sector_tuner` 와
`ot_interpolator` 는 "Parameter file path is not a file" 경고와 함께 기본값으로 뜬다
(치명적이진 않지만 쓸모없는 노드가 붙는다). 그래서 측위 전용 런치를 따로 뒀다:

```bash
ros2 launch stack_master localization_only_launch.xml racecar_version:=NUC2
# map.pcd 가 다른 곳이면
ros2 launch stack_master localization_only_launch.xml map_pcd:=/경로/map.pcd
# VESC/조이스틱을 이미 띄웠으면
ros2 launch stack_master localization_only_launch.xml bringup:=false
```

측위 체인(scanmatcher → ekf_localizer → carstate_3d)은 레이스라인과 무관하다.
`carstate_3d` 는 `/global_waypoints` 가 없으면 Frenet 변환만 건너뛰고
**`/car_state/pose` 와 `/car_state/odom` 은 그대로 발행**한다 — sysid 에 필요한 건 그게 전부다.

> **초기 포즈**: `slam.launch.py` 로 갓 만든 맵은 *매핑을 시작한 그 자리*가 원점이다.
> 차를 그 지점에 두고 띄우면 기본값(원점·무회전) 그대로 맞는다. 다른 곳에 두었다면
> RViz "2D Pose Estimate" 로 잡아줄 것 — NDT 는 국소 최적화라 초기 포즈가 크게 틀리면
> 스스로 회복하지 못한다.
> (`lidarslam.yaml` 의 `initital_pose_*` 는 오타 키라 영구 무효다. 코드가 읽는 이름은
>  `initial_pose_*` 이고, 이 런치가 그 이름으로 인자를 넘긴다.)

### 0.4 스택 띄우기 — base_system **만**
```bash
ros2 launch stack_master base_system_3D_launch.xml racecar_version:=NUC2 \
  map_dir:=lobby_0819 map_name:=lobby_0819 sim:=false rviz:=true
```
rviz 에서 2D Pose Estimate 로 초기 위치를 맞추고 `/car_state/pose` 가 나오는지 확인.
**time_trials 는 띄우지 않는다.** 띄우면 `mux_controller` 가 `/drive` 를 계속 발행해 명령이 섞인다
(`run_sysid.sh` 가 사전점검에서 잡아준다).

### 0.4b 데드맨 버튼 번호부터 확인 (처음 한 번)

기본값은 R1 = 버튼 5 지만 컨트롤러/드라이버에 따라 다르다. 먼저 확인할 것:
```bash
cd ~/forza_ws/race_stack/loc_check && ./sysid_cmd.py --probe
```
R1 을 눌러 몇 번인지 보고, 5 가 아니면 모든 실행에 `--deadman <번호>` 를 붙인다.
**joy_teleop 의 데드맨(L1, 보통 4)과는 다른 번호여야 한다** — 같으면 joy_teleop 가
`/drive` 를 같이 쏴서 명령이 섞인다.

> 차가 안 움직이는데 `[ 0.02/20.0s spool]` 같은 상태 줄이 **안 나오면** 데드맨 문제다.
> 대기 중에는 `[대기] 데드맨 버튼 5 를 누르고 있어야 시작한다. 지금 눌린 버튼: [4]` 처럼
> 원인이 찍힌다.
> 상태 줄은 나오는데 차가 안 가면 → VESC e-stop 래치(Triangle 로 해제) 또는 배선/전원.

### 0.5 조작 규칙 — 한 번에 한 종목, 사람이 붙어 있어야 한다

`run_sysid.sh` 는 **한 번에 하나만** 돌린다. 둘을 동시에 띄우면 `/drive` 에 두 노드가
동시에 발행해서 명령이 섞이고 bag 도 두 개가 된다.

**저절로 돌지 않는다.** 실행하면 이런 순서로 간다:

1. 사전점검 (문제 있으면 `y/N` 을 물어본다)
2. bag 녹화 시작 (2초)
3. 프로파일을 출력하고 **대기** — 이 동안 계속 정지 명령을 쏜다
4. **R1(버튼 5)을 누르고 있으면** 1초 카운트다운 뒤 시퀀스 시작
5. 시퀀스가 끝나면 정지 명령을 2초 더 쏘고 **스스로 종료** → 분석까지 자동
   (`--hold-after -1` 을 주면 Ctrl-C 를 기다린다)

* **R1 을 놓으면 그 즉시 제동하고 시퀀스가 처음으로 되감긴다.** 도중에 손을 떼면 그 런은 버리고
  다시 눌러 처음부터 하면 된다 (CSV 에 `중단 N회` 로 남는다).
* **L1 은 절대 쥐지 말 것** — joy_teleop 가 `/drive` 를 같이 쏜다.
* 비상: **Circle(2)** 래치 정지 → **Triangle(3)** 해제. VESC 레벨이라 이 툴과 무관하게 듣는다.
* 차를 출발점에 놓는 것, 사람이 R1 을 쥐고 있는 것, 이탈하면 Circle 을 누르는 것 — 이 셋은 사람 몫이다.

---

## 1. 측정 순서와 공간

아래 수치는 **시뮬레이터로 궤적을 실제로 그려서 잰 것**이다(`sysid_selftest.py` 와 같은 차 모델).
괄호 안은 그 설정으로 계수가 얼마나 회수되는지 — 참값 대비.

**직선 15 m 기준**(권장). 괄호는 공간이 좁을 때의 축소판.

| # | 항목 | 궤적 | **실측 궤적 크기** | 시간 | 회수 확인 |
|---|---|---|---|---|---|
| M0 | `speed_to_erpm_gain` | 직선 | **11.3 m** (8.4 / 6.6) | 8 s | 3575.5 (참 3576), IQR 26 |
| M1 | **mu + alpha_char** | 원 | **2.6 × 2.5 m** | 13 s | mu 0.96 / α_c 0.082 (참 0.95 / 0.075) |
| M2 | f_drive / f_brake | 직선 | **11.7 m** (7.9 / 4.1) | 6 s | f_brake 14.7 N (참 14.2), 0~5 m/s 5구간 |
| M3 | k_drive | 원 | **2.4 × 2.3 m** | 12 s | 38.4 (참 40.0) |
| M4 | c_roll | 직선 | **12.0 m** (8.4 / 4.0) | 7 s | 0.0154 (참 0.015) |
| M5 | 조향 t63 | 원 | **2.7 × 2.6 m** | 8 s | 70 ms (큰 공간판과 동일) |

**직선 15 m + 4×4 m 박스**. 둘이 겹쳐도 되므로 실질적으로 **15 m × 4 m 한 구획**이면 전부 된다.

직선 여유분의 우선순위는 **M2 > M0 > M4** 다.
* M2 는 가속-속도 곡선의 구간 수가 곧 '역기전력 처짐' 관측력이다. 4.1 m→2구간, 7.9 m→4구간,
  11.7 m→**5구간(0~5 m/s)**.
* M0 은 정속 구간이 길수록 게인 산포와 줄자 오차가 같이 준다 (IQR 42→34→**26**).
* M4 는 Δv 가 커지고 측정 속도대가 레이스에 가까워진다 (0.29→0.37→**0.44 m/s**).
* **원 3종(M1/M3/M5)은 직선을 늘려도 좋아지지 않는다.** 여긴 아커만 기하 오차(M1)와
  정착 시간(M3/M5)이 한계라서, 필요한 건 직선이 아니라 더 넓은 박스다.

의존성: **M0 → M1 → M2** (mu 를 모르면 M2 의 해석이 안 갈린다). M3/M4/M5 는 독립.

### 무엇을 포기하는가
* **M2 를 v=3.0 으로 줄이면 고속 처짐(역기전력)을 못 본다.** 이건 원래 15 m 직선이 필요한 항목인데,
  전용 직선 없이 대신할 방법이 있다 — §2 의 M2b 참조.
* **M4 는 반드시 직선에서.** 원에서 하면 코너링 저항이 c_roll 에 그대로 섞인다.
  시뮬로 확인: 직선 0.0156 / R=3.3 m 원 0.0162(+4%) / R=1.6 m 원 0.0182(**+17%**).
* **M1 을 더 조이면**(δ 0.40, 2.1×2.2 m) 아커만 기하 오차가 커진다. δ 0.32 가 타협점이다.
* **M5 를 v≥4 로 올리지 말 것.** δ 0.35 에서 횡가속이 한계를 넘어 응답이 깨진다
  (시뮬에서 t63 이 50/160 ms 로 튀었다). v 1.5~3.0 구간은 전부 70 ms 로 일치한다.

---

## 2. 종목별 실행 (축소판)

전부 `run_sysid.sh <모드> <옵션...>` 한 줄이다. 사전점검 → bag 녹화 → 명령 → 분석까지 한다.
결과는 `~/forza_ws/race_stack/sysid_<모드>_<시각>/` 에 `bag/ cmd.csv report.txt plot.png` 로 남는다.

### M0. speed_to_erpm_gain 검증 — 직선 11.3 m
```bash
./run_sysid.sh const --v 2.0 --hold 5
```
* 출발점에 테이프를 붙이고 멈춘 자리까지 **줄자로 재서** 리포트의 `이동거리` 와 비교한다.
  이 설정의 정속 구간은 **10.0 m**(리포트 이동거리 9.5 m) — 줄자 읽기 오차(±2 cm)가 0.2% 로 묻힌다.
* 좁으면 `--v 1.5 --hold 5`(8.4 m) → `--v 1.2 --hold 5`(6.6 m) → `--hold 3`(4.6 m) 순으로 줄인다.
* 게인 산포(IQR): v 2.0 → 26, v 1.5 → 34, v 1.2 → 42. 속도가 높을수록 pose 미분이 정확하다.
* 3% 이상 어긋나면 `stack_master/config/NUC2/vesc.yaml` 을 고치고 `sync_config.sh` → 노드 재시작.

### M1. mu + alpha_char — 원 2.6 × 2.5 m  ★최우선
```bash
./run_sysid.sh circle --steer  0.32 --v0 1.0 --v1 4.0 --t 9 --t-entry 2
./run_sysid.sh circle --steer -0.32 --v0 1.0 --v1 4.0 --t 9 --t-entry 2   # 반대 방향
```
* 좌/우 각 3회. 좌우 하중이 51.4:48.6 으로 비대칭이다. 중앙값을 쓴다.
* 공간이 더 있으면 `--steer 0.25 --v1 5.0 --t 12` (3.1×3.1 m) 쪽이 아커만 오차가 작아 낫다.
* 리포트에서 볼 것: `tanh 피팅 (rear slip)` 의 `mu` 와 `alpha_char` ← **이게 최종 답이다.**
  *"슬립각이 alpha_char 까지 못 갔다"* 가 뜨면 `--steer` 를 키우거나 `--v1` 을 올린다.
  `IMU wz vs pose 요레이트 상관` 이 0.9 미만이면 자이로 축/부호부터 확인.

### M2. f_drive_max / f_brake_max — 직선 11.7 m
```bash
./run_sysid.sh accel --v 5.5 --t 1.8
python3 analyze_sysid.py <결과>/cmd.csv --mu 0.95      # M1 끝난 뒤 다시 보면 해석이 좁아진다
```
| `--v` | `--t` | v_peak | 궤적 | 제동이 20% 약할 때 |
|---|---|---|---|---|
| 3.0 | 1.2 | 2.98 | 4.1 m | 4.4 m |
| 4.0 | 1.4 | 3.98 | 6.5 m | 7.1 m |
| 4.5 | 1.5 | 4.48 | 7.9 m | 8.7 m |
| 5.0 | 1.6 | 4.98 | 9.4 m | 10.4 m |
| **5.5** | **1.8** | **5.48** | **11.7 m** | **12.8 m** ← 15 m 안에서 가장 멀리 간다 |
| 6.0 | 2.0 | 5.98 | 14.1 m | **15.5 m** ✗ |

* 오른쪽 열은 실차 `f_brake_max` 가 시뮬(14.2 N)보다 20% 낮을 경우다. 아직 실측 전이니
  이 여유를 보고 골라야 한다. **첫 런은 `--v 3.0` 으로 제동거리를 눈으로 확인한 뒤 올릴 것.**
* 역방향 제동(`--v-brake -1.5`)은 거리를 줄여주지 못한다(시뮬 4.08 → 4.07 m).
  제동은 `f_brake_max` 에 걸려 있지 명령 부호에 걸려 있지 않다.
* **핵심은 `휠슬립 (ERPM속도 − 지면속도)` 줄이다.** 슬립≈0 이면 힘 한계라
  `f_drive_max = m·a` 로 바로 나오고 com_height 와 무관하다. 슬립이 크면 마찰 한계고,
  `--mu` 를 주면 `a = mu·g·lf/(L − mu·h)` 로 com_height 를 역산한다.

### M2b. 고속 처짐 — 공간 0 m
전용 직선 없이, **아무 주행 로그에서나** 전압 한계를 본다:
```bash
python3 analyze_sysid.py <PP랩 bag 또는 아무 sysid CSV> --mode duty
```
duty(전압 여유)를 속도구간별로 보여주고, `duty=1.0` 으로 외삽해 **이 배터리에서의 최고속**을 낸다.
학습 `v_max` 가 그 값을 넘으면 그 구간은 시뮬에만 존재하는 속도다 — 정책은 낼 수 있다고 배우는데
실차는 못 낸다. 1~2일차 PP 랩 로그로 그냥 돌리면 된다.

### M3. k_drive — 원 2.4 × 2.3 m
```bash
./run_sysid.sh step --steer 0.28 --v0 1.2 --dv 0.4 --t-step 0.8 --n 5
```
* **계단은 반드시 작게.** 포화 전 선형구간이 `f_drive_max/k_drive ≈ 0.78 m/s` 뿐이라
  풀스로틀 로그로는 원리적으로 식별이 안 된다.
* `--t-step 0.8` 은 tau(≈0.13 s)의 6배다. 4배(0.5 s)면 이미 정착하므로 더 줄여도 되지만
  피팅 표본이 줄어든다.
* 이 종목만 **ERPM 환산 속도**로 시정수를 잰다(게인이 틀려도 tau 는 불변). 리포트에 찍힌다.

### M4. c_roll — 직선 12.0 m  (우선순위 낮음)
```bash
./run_sysid.sh coast --v 3.0 --hold 0.4 --t-coast 3.0 --ramp 1.2
```
* 높은 속도에서 재는 게 레이스 속도대에 가깝다(구름저항의 점성 성분이 속도에 따라 다르다).
  이 설정은 2.55~2.94 m/s 구간을 275 표본으로 본다.
* 좁으면 `--v 2.5 --t-coast 2.5`(8.4 m, Δv 0.37) → `--v 1.5 --t-coast 2.0`(4.0 m, Δv 0.29).
* `--v 3.5` 이상은 14 m 를 넘어 여유가 없다.
* **조향 0 으로, 직선에서만.** 원에서 하면 코너링 저항이 섞여 c_roll 이 부풀려진다(위 §1).
* 타력주행은 `/commands/motor/current 0` 으로 낸다. 속도 0 을 쏘면 그건 "0 rpm 유지" = 제동이라
  코스트다운이 안 된다. 리포트에 *"타력주행 중 모터전류가 …"* 경고가 뜨면 그 런은 버린다.

### M5. 조향 t63 — 원 2.7 × 2.6 m
```bash
./run_sysid.sh steer --v 2.0 --amp 0.10 --bias 0.25 --period 0.8 --n 6
```
* `--bias 0.25 --amp 0.10` → 조향이 0.15 ↔ 0.35 rad. **부호가 안 바뀌어 한 방향 선회를 유지**하므로
  원 하나에 들어간다. `--bias 0`(슬라럼)은 직선 18 m 가 필요하다.
* `--period 0.8`(반주기 0.4 s)이 하한이다. 0.7 로 줄이면 정착 전에 뒤집혀 t63 이 짧게 나온다.
* 무부하 서보 응답(43.8 ms)은 이미 있다. 여기서 재는 건 **[서보 + 타이어 완화 + 요관성]** 합,
  즉 시뮬이 맞춰야 할 대상이다. `--v 0` 으로 차를 들고 돌리면 요레이트가 안 나와 t63 을 못 잰다.

### 측위 없이 (가벼운 bringup 만)
```bash
ros2 launch f1tenth_stack bringup_3D_launch.py

./run_sysid.sh step  --no-pose --steer 0.28 --v0 1.2 --dv 0.4 --t-step 0.8 --n 5
./run_sysid.sh coast --no-pose --v 3.0 --hold 0.4 --t-coast 3.0 --ramp 1.2
./run_sysid.sh steer --no-pose --v 2.0 --amp 0.10 --bias 0.25 --period 0.8 --n 6
python3 measure_grip.py --live --radius 2.0        # mu (조이스틱으로 콘 원, 속도스케일 상향 필요)
python3 analyze_sysid.py <로그> --mode duty        # 고속 처짐
```

---

## 3. 결과를 어디에 넣나

| 리포트 항목 | 반영 위치 |
|---|---|
| `tanh 피팅 mu` | `env_cfg.py` `--mu_range` — **좁게**. 정책은 mu 를 관측 못 해서 넓은 밴드에서는 하한에 수렴한다 |
| `tanh 피팅 alpha_char` | `TireModelCfg.alpha_char` (현재 0.08 은 **한 번도 측정 안 된 추정치**) |
| `k_drive` | `TireModelCfg.k_drive` (현재 40.0, 식별 불가였던 값) |
| `f_drive_max` (힘 한계일 때만) | `TireModelCfg.f_drive_max` (현재 31.1 N, 2→5 m/s 구간만으로 낸 값) |
| 속도구간별 가속 감소 | 모터 모델에 속도 의존성 추가 여부 판단 |
| `f_brake_max` | `TireModelCfg.f_brake_max` (현재 14.2 N) |
| `c_roll` | `TireModelCfg.c_roll` (현재 0.015 추정) |
| 요레이트 `t63` | `car_cfg.py` steering damping. 실차가 빠르면 8.0 → 2.0 (tau 100 ms → 25 ms). ★그 다음 `k_smooth_steer` 재조정 필요 |
| `실측 ERPM/속도` | `stack_master/config/NUC2/vesc.yaml` `speed_to_erpm_gain` |

---

## 4. 대회장에서 (전용 실험 없이)

```bash
# 1~2일차 PP 랩 로그를 그대로
python3 ~/forza_ws/race_stack/loc_check/measure_grip.py <bag디렉터리> --combined
# 주행 중 실시간
python3 ~/forza_ws/race_stack/loc_check/measure_grip.py --live
```
`mu >= p99(|v·wz|)/g` 는 전용 공간도 시간도 필요 없다. 출력에 학습용 `mu_range` 제안이 붙는다.
종방향으로는 mu 를 못 캘 수 있다(가속이 힘 상한에 걸리면 mu 에 반응하지 않는다) — 그 경우를 리포트가 알려준다.

### throttle_interpolator 를 켤 때
`bringup_3D_launch.py:150` 주석만 풀면 **아무 일도 일어나지 않는다.** 같이 remapping 을 넣어야 한다:
```python
ackermann_to_vesc_node = Node(
    package='vesc_ackermann', executable='ackermann_to_vesc_node',
    name='ackermann_to_vesc_node',
    parameters=[LaunchConfiguration('vesc_config')],
    remappings=[('ackermann_cmd', 'drive'),
                ('commands/motor/speed',   'commands/motor/unsmoothed_speed'),
                ('commands/servo/position','commands/servo/unsmoothed_position')])
```
* 벤치 확인 결과 노드 자체는 정상 동작한다(0→35760 ERPM 에 4.0 s = 2.5 m/s², 설정대로).
* ★부작용: 같은 노드가 **조향도** 율제한한다. `max_servo_speed 3.2 rad/s` + 게인 −0.65 면
  풀스윙에 약 **225 ms** — 실차 서보(43.8 ms)의 5배로 느려진다. 가속만 제한하고 싶으면
  `max_servo_speed` 를 20 이상으로 올려 사실상 무력화할 것.
* 켠 뒤에는 M2/M3 를 **다시** 재야 한다. 측정 대상이 차가 아니라 [차+제한기]로 바뀐다.

---

## 5. 파일

| 파일 | 역할 |
|---|---|
| `sysid_cmd.py` | 오픈루프 명령 노드(데드맨/타임아웃/프로파일). 단독 사용 가능 |
| `record_sysid.sh` | 물리계수용 bag 녹화(라이다 제외, ~0.15 MB/s) |
| `run_sysid.sh` | 사전점검 + 녹화 + 명령 + 분석 한 방 |
| `analyze_sysid.py` | CSV/bag → mu, alpha_char, k_drive, f_drive_max, f_brake_max, c_roll, t63, ERPM 게인 |
| `measure_grip.py` | 아무 주행 로그에서나 mu 하한 (대회장용) |
| `sysid_io.py` | CSV/bag 공통 로더 (pose 미분, 슬립각) |
| `sysid_selftest.py` | 차 없이 툴체인 검증 |
