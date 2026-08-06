# rl_controller 가중치

Isaac Lab 학습(`dacerpp_isaaclab`)의 산출물을 **레포 안으로 복사한 것**이다.
노드는 학습 워크스페이스(`~/shared_dir/...`)를 전혀 참조하지 않는다.
`config/rl_controller.yaml` 의 `checkpoint` 가 상대경로면 이 디렉터리 기준으로 해석된다.

| 파일 | 학습 런 | step | risk | 곡률 클립 | 용도 |
| --- | --- | --- | --- | --- | --- |
| `20260805/pow.pt` | 20260805 | 2.5M | pow(1.3) | **±2** | **실차 주행 기본값** (학습의 car_b = 배포 대상) |
| `20260805/cvar.pt` | 20260805 | 2.5M | cvar(0.5) | **±2** | 같은 런의 보수적 정책. 헤어핀에서 더 안전하게 갈 때 |
| `cvar.pt` | 20260726 | 4.4M | cvar(0.5) | ±1 | 구세대. 쓰려면 `curv_clip: 1.0` 으로 되돌려야 한다 |

관측 차원은 셋 다 58 로 같지만 **곡률 채널의 스케일이 다르다**. 20260805 세대는
`|kappa|` 를 ±2 까지 노출하도록 학습됐고(구 ±1 클립이 1.0 이상을 전부 뭉개
헤어핀 대응을 망쳤다), 구세대는 ±1 이다. 체크포인트를 바꿀 때 `curv_clip` 을
같이 맞추지 않으면 정책이 학습과 다른 입력을 보게 된다.

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
cp <run_dir>/pow.pt  models/<날짜>/pow.pt
cp <run_dir>/cvar.pt models/<날짜>/cvar.pt
cp <run_dir>/train_meta.json models/<날짜>/
# setup.py 의 data_files 에 새 날짜 디렉터리를 추가한 뒤
colcon build --packages-select rl_controller
```

학습 쪽에서 관측 구성(`RacingCfg.obs_dim()`, 정규화 상수, 클립 범위)이 바뀌었다면
`rl_controller_node.build_observation` 과 `config/rl_controller.yaml` 도 같이 맞출 것.
노드는 체크포인트의 `obs_dim` 과 파라미터로 계산한 차원이 다르면 기동 시 죽는다.
