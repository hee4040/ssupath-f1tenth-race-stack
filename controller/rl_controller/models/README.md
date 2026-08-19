# rl_controller 가중치

Isaac Lab 학습(`dacerpp_isaaclab`)의 산출물을 **레포 안으로 복사한 것**이다.
노드는 학습 워크스페이스(`~/shared_dir/...`)를 전혀 참조하지 않는다.
`config/rl_controller.yaml` 의 `checkpoint` 가 상대경로면 이 디렉터리 기준으로 해석된다.

**이 디렉터리 최상단에는 항상 배포 대상 두 파일만 둔다.**
`setup.py` 가 `models/*.pt` 를 글롭으로 설치하므로 — **하위 폴더는 install 공간에
들어가지 않는다.** 날짜 폴더를 두는 건 참고용이고, 배포하려면 최상단으로 올려야 한다.

| 파일 | risk | 용도 |
| --- | --- | --- |
| `pow_healthy.pt` | pow(1.3) | **실차 주행 기본값** — 학습의 car_b = 배포 대상 |
| `cvar_healthy.pt` | cvar(0.5) | 같은 런의 보수적 정책. 헤어핀에서 더 안전하게 갈 때 |

현재 실린 런: **20260818_85110** (`obs_dim=60`, `curv_clip=3.0`,
`curv_lookahead=[5,15,30,60,90,120]`, `mu_range=(0.85,1.10)`).
전체 학습 설정은 `run_config.json`, 학습 iteration 은 `train_meta.json`,
healthy 스냅샷 지점은 `healthy_meta.json` 에 있다.

## 왜 `*_healthy.pt` 인가 (★가장 중요)

`train.py` 는 배포 대상(Pow=car_b)의 **조향 다양성 `pstd_b_steer` 가 붕괴 기준
0.02 이상인 동안만** `*_healthy.pt` 를 갱신한다. 그 아래로 떨어지면
`"붕괴 영역 — 배포는 pow_healthy.pt 를 쓸 것"` 경고를 찍고 갱신을 멈춘다.

20260818 스윕 3개 런 모두 **최종 `pow.pt`/`cvar.pt` 는 붕괴 영역**이다
(`collapse_log.csv` 마지막 구간 pstd 0.010~0.018). 즉 '가장 최근' 가중치가
'가장 좋은' 가중치가 아니다. 실린 런의 healthy 스냅샷은 it=1087000, pstd=0.0260.

| 런 | mu_range | healthy it | healthy pstd_b |
| --- | --- | --- | --- |
| `20260818_5585` | 0.55~0.85 | 1112800 | 0.0206 |
| `20260818_7095` | 0.70~0.95 | 1067800 | 0.0209 |
| **`20260818_85110`** | **0.85~1.10** | **1087000** | **0.0260** |

`_85110` 을 고른 이유: 2026-08-18 실차 실측 노면 마찰이 **0.85~1.07** 이라
이 밴드가 그것을 그대로 브래킷하고, healthy pstd 도 셋 중 가장 높다.

## car_a / car_b — 왜 pow 인가

학습은 한 환경에 두 대를 태워 동시에 학습한다 (`scripts/train.py`):

```
agent_cvar = make_agent(conservative_cvar(...))   # car_a — 속도 핸디캡(4~8m/s)이 걸린 스파링 상대
agent_pow  = make_agent(aggressive_pow(...))      # car_b — 핸디캡 없음, 실차 배포 대상
```

`env_cfg.RacingCfg.v_cap_a_range` 주석: *"Car B(Pow, 실차 배포 대상)가 추월을 실제로
경험하도록 Car A(CVaR)의 명령 속도 상한을 에피소드마다 랜덤으로 뽑는다. A 는 자기
상한을 관측하지 못하므로 A 의 주행 품질은 다소 희생된다(의도됨)."*

즉 `cvar_healthy.pt` 는 '이동 장애물' 역할로 열화된 정책이다.

## 갱신 방법

```bash
cd controller/rl_controller/models
rm -f *_healthy.pt *.json
cp <run_dir>/{pow_healthy.pt,cvar_healthy.pt,train_meta.json,healthy_meta.json,run_config.json} .
colcon build --packages-select rl_controller
```

## 갱신할 때 같이 확인할 것 (관측 규약)

체크포인트 차원이 바뀌면 노드가 기동 시 죽으므로 **차원 변경은 자동으로 잡힌다.**
위험한 건 **차원은 그대로인데 정규화/클립/기준점만 바뀌는 경우**다.
`run_config.json` 이 이 값들을 다 담고 있으니 **배포 yaml 과 한 줄씩 대조할 것.**

| 학습 변경 | 배포 쪽 대응 |
| --- | --- |
| 곡률 클립 ±1 → ±2 (20260805 세대) | `curv_clip: 2.0` |
| 상대차 검출 기준점: 차중심 → 뒷부분 감지 박스 (20260807 세대) | `opp_det_box_rear: 0.20` |
| 곡률 클립 ±2 → ±3 (20260818 세대) | `curv_clip: 3.0` ← **차원 불변 = 조용히 틀린다** |
| 전방 예견 +120(18m), obs_dim 58 → 60 (20260818 세대) | `curv_lookahead: [5,15,30,60,90,120]` |

관측 포맷을 정하는 곳은 `dacerpp_lab/racing_env.py` 의 `_observe_car` 와
`dacerpp_lab/env_cfg.py` 의 `RacingCfg` 두 곳뿐이니, 갱신 시 이 둘의 diff 만 보면 된다.
물리(마찰/슬립 벌점 등) 변경은 주행 코드에는 영향이 없지만
`rl_controller/offline_sim.py` 의 검증 상수(`MU_NOM`, 질량/축거/구동·제동력)는
맞춰 두어야 검증이 의미가 있다.

## `prev_obs58/`

2026-08-18 까지 실려 있던 obs_dim=58 세대. 글롭 경로 밖으로 치워 둔 것이라
install 공간에는 들어가지 않는다. 되돌리는 절차는 그 안의 README 참조.
