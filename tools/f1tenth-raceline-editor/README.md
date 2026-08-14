# F1TENTH Raceline Offline — uv standalone

`ssupath-f1tenth-race-stack`의 global raceline 생성부를 ROS2 launch 없이 실행하기 위한 독립 프로젝트입니다.

## 목표

새 PC에 필요한 것은 `uv`, `git`, 인터넷 연결(첫 설치/첫 bootstrap), 그리고 맵 `.yaml` + `.png`입니다. ROS2, colcon workspace, `f110_msgs`, RViz, F1TENTH Gym은 offline raceline 생성에는 필요하지 않습니다.

이 프로젝트는 `hee4040/ssupath-f1tenth-race-stack`의 optimizer/config를 아래 커밋으로 고정합니다.

```text
acbf008694eef416ed1b189779da2b8f26996909
```

## Dependency baseline

고정된 ssupath optimizer가 최신 IQP API(`spline_len`, `psi`, `kappa`, `dkappa`)를 사용하므로 `trajectory-planning-helpers==0.80`을 사용합니다. TPH upstream의 `spline_approximation.dist_to_p()`가 최신 SciPy에서 1-D vector 오류를 일으키는 문제는 좁은 runtime compatibility shim으로 보정합니다. optimizer 수식, 파라미터, 출력 파일 형식은 변경하지 않습니다.

## 설치

```bash
uv sync
uv run raceline bootstrap
uv run raceline doctor
```

`.python-version`은 Python 3.10으로 고정되어 있습니다.

## 레이스라인 생성

```bash
uv run raceline generate --map maps/teras/teras.yaml
```

맵 편집을 적용하려면:

```bash
uv run raceline generate \
  --map maps/teras/teras.yaml \
  --edit maps/teras/edit/map_edit.json
```

기본 출력은 `<map-dir>/raceline_offline/`입니다.

```text
centerline.csv
raceline_iqp.csv
raceline_shortest.csv
ltpl.csv
bound_left.csv
bound_right.csv
summary.json
.work/
```

기존 output contract는 유지됩니다. `raceline_iqp.csv`/`raceline_shortest.csv`의 헤더는 다음과 같습니다.

```text
s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2
```

## Browser editor

```bash
uv run raceline edit --map maps/teras/teras.yaml
```

하나의 browser editor에서 다음을 처리합니다.

- 비파괴 occupancy map brush 편집
- centerline preview
- Speed sector 편집
- Overtaking sector 편집
- ROS 호환 YAML export

### Sector editor

레이스라인을 먼저 생성한 뒤 `Speed sectors` / `Overtaking sectors` 탭을 사용합니다. 맵 위 minimum-curvature raceline을 클릭하면 split이 추가됩니다.

내부 canonical representation은 waypoint index가 아니라 **물리 거리 `s_m`** 입니다. raceline point density가 바뀌어도 split이 가능한 한 같은 물리 위치에 유지되고, 기존 ROS YAML을 export할 때만 현재 raceline의 nearest waypoint index로 변환합니다.

저장 파일:

```text
<map-dir>/edit/sectors.json       # s_m 기반 canonical source
<map-dir>/speed_scaling.yaml      # 기존 ROS sector_tuner schema
<map-dir>/ot_sectors.yaml         # 기존 ROS ot_interpolator schema
```

Speed sector에서는 `scaling`, `only_FTG`, `no_FTG`를 설정할 수 있습니다. Overtaking sector에서는 `ot_flag`, `yeet_factor`, `spline_len`, `ot_sector_begin`을 설정합니다.

에디터/validator는 다음 위험을 검사합니다.

- `only_FTG`와 `no_FTG` 동시 활성화: error
- `scaling > global_limit`: runtime clip warning
- speed sector가 legacy ±10 waypoint blending보다 너무 짧음
- active overtaking sector가 `2 * spline_len`보다 짧아 entry/exit interpolation이 겹칠 수 있음
- map/edit가 raceline보다 최신이면 stale raceline warning
- `raceline_shortest.csv`가 없으면 overtaking editor 비활성화
- raceline에서 너무 먼 클릭은 sector split으로 받지 않음

GUI 없이 기존 ROS YAML을 다시 export할 수도 있습니다.

```bash
uv run raceline sectors --map maps/teras/teras.yaml
```

## Tests

```bash
uv run pytest -q
```

테스트는 다음을 검증합니다.

- `s_m` ↔ legacy waypoint index 변환
- 기존 `speed_scaling.yaml` / `ot_sectors.yaml` schema
- speed/OT transition hazard 검출
- map/world 좌표 변환 round-trip
- empty map edit pixel identity
- 기존 raceline CSV/JSON output contract
- editor safety guard 존재 여부

GitHub Actions에서도 push/PR마다 같은 pytest suite를 실행합니다.
