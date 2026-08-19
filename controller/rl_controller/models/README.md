# rl_controller 가중치

Isaac Lab 학습(`dacerpp_isaaclab`)의 산출물을 **레포 안으로 복사한 것**이다.
노드는 학습 워크스페이스(`~/shared_dir/...`)를 전혀 참조하지 않는다.
`config/rl_controller.yaml` 의 `checkpoint` 가 상대경로면 이 디렉터리 기준으로 해석된다.

**이 디렉터리는 항상 최신 런의 두 파일만 둔다** (날짜 폴더를 만들지 않는다).
그래서 갱신할 때 `setup.py` 나 설정 경로를 고칠 일이 없다.

| 파일 | risk | 용도 |
| --- | --- | --- |
| `pow.pt` | pow(1.3) | **실차 주행 기본값** — 학습의 car_b = 배포 대상 |
| `cvar.pt` | cvar(0.5) | 같은 런의 보수적 정책. 헤어핀에서 더 안전하게 갈 때 |

현재 실린 런: **20260810** (5.5M step, `obs_dim=58`, 곡률 클립 ±2).
`train_meta.json` 에 학습 iteration 이 들어 있다.

## car_a / car_b — 왜 pow.pt 인가

학습은 한 환경에 두 대를 태워 동시에 학습한다 (`scripts/train.py`):

```
agent_cvar = make_agent(conservative_cvar(...))   # car_a — 속도 핸디캡(4~8m/s)이 걸린 스파링 상대
agent_pow  = make_agent(aggressive_pow(...))      # car_b — 핸디캡 없음, 실차 배포 대상
```

`env_cfg.RacingCfg.v_cap_a_range` 주석: *"Car B(Pow, 실차 배포 대상)가 추월을 실제로
경험하도록 Car A(CVaR)의 명령 속도 상한을 에피소드마다 랜덤으로 뽑는다. A 는 자기
상한을 관측하지 못하므로 A 의 주행 품질은 다소 희생된다(의도됨)."*

즉 `cvar.pt` 는 '이동 장애물' 역할로 열화된 정책이다. 실차에는 `pow.pt` 를 쓴다.

## 갱신 방법

```bash
cd controller/rl_controller/models
rm -f pow.pt cvar.pt train_meta.json
cp <run_dir>/{pow.pt,cvar.pt,train_meta.json} .
colcon build --packages-select rl_controller
```

`setup.py` 는 `models/*.pt` 를 글롭으로 설치하므로 수정할 필요가 없다.

## 갱신할 때 같이 확인할 것 (관측 규약)

체크포인트 차원이 바뀌면 노드가 기동 시 죽으므로 **차원 변경은 자동으로 잡힌다.**
위험한 건 **차원은 그대로인데 정규화/클립/기준점만 바뀌는 경우**다. 지금까지 그런 게 둘 있었다:

| 학습 변경 | 배포 쪽 대응 |
| --- | --- |
| 곡률 클립 ±1 → ±2 (20260805 세대) | `curv_clip: 2.0` |
| 상대차 검출 기준점: 차중심 → 뒷부분 감지 박스 (20260807 세대) | `opp_det_box_rear: 0.20` |

관측 포맷을 정하는 곳은 `dacerpp_lab/racing_env.py` 의 `_observe_car` 와
`dacerpp_lab/env_cfg.py` 의 `RacingCfg` 두 곳뿐이니, 갱신 시 이 둘의 diff 만 보면 된다.
물리(마찰/슬립 벌점 등) 변경은 주행 코드에는 영향이 없지만
`rl_controller/offline_sim.py` 의 검증 상수(`MU_NOM`)는 맞춰 두어야 검증이 의미가 있다.
