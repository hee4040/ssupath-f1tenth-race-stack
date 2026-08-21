#!/usr/bin/env bash
# 실측 한 종목을 통째로 실행한다: 사전점검 -> bag 녹화 -> 명령 노드 -> 분석.
#
#   ~/forza_ws/race_stack/loc_check/run_sysid.sh circle --steer 0.25 --v0 1.0 --v1 5.0 --t 12
#   ~/forza_ws/race_stack/loc_check/run_sysid.sh accel  --v 6.0 --t 2.5
#   ~/forza_ws/race_stack/loc_check/run_sysid.sh step   --v0 2.0 --dv 0.4 --n 4
#   ~/forza_ws/race_stack/loc_check/run_sysid.sh coast  --v 5.0 --t-coast 4
#   ~/forza_ws/race_stack/loc_check/run_sysid.sh steer  --v 3.0 --amp 0.25 --n 6
#   ~/forza_ws/race_stack/loc_check/run_sysid.sh const  --v 1.5 --hold 6      # ERPM 게인 검증
#
# 측위 없이 (조이스틱+VESC만 띄운 경우):
#   ~/forza_ws/race_stack/loc_check/run_sysid.sh step   --no-pose --steer 0.10 --v0 2.0 --dv 0.4
#   ~/forza_ws/race_stack/loc_check/run_sysid.sh circle --no-pose --radius 2.0 --steer 0.25 --v1 5.0
#   (--no-pose / --radius / --mu 는 이 래퍼가 분석기로 넘긴다)
#
# 첫 인자 뒤의 것은 전부 sysid_cmd.py 로 그대로 넘어간다 (--help 로 목록 확인).
# 결과는 ~/forza_ws/race_stack/sysid_<모드>_<시각>/ 에 bag / cmd.csv / report.txt 로 남는다.
#
# ★ 반드시 base_system 만 띄운 상태에서 쓸 것:
#     ros2 launch stack_master base_system_3D_launch.xml racecar_version:=NUC2 \
#       map_dir:=lobby_0819 map_name:=lobby_0819 sim:=false rviz:=false
#   time_trials 를 같이 띄우면 mux_controller 가 /drive 를 계속 쏴서 명령이 섞인다.
#   측위가 필요 없는 종목(step/coast/steer, --radius 준 circle)은 더 가벼운 쪽으로 충분하다:
#     ros2 launch f1tenth_stack bringup_3D_launch.py     # 조이스틱+VESC+라이다만
#   (설치본 f1tenth_stack/config/vesc.yaml 도 게인 3576 / 조향 -0.65 로 NUC2 와 같고,
#    Circle e-stop 은 vesc_driver 코드 기본값이라 yaml 에 없어도 동작한다)
# ★ 운전자는 R1(버튼 5)을 누르고 있어야 차가 움직인다. 떼면 즉시 제동.
#   비상시 Circle(버튼 2) = VESC 레벨 래치 정지, Triangle(버튼 3) = 해제.
# ROS setup.bash 가 미정의 변수를 참조하므로 set -u 는 source 뒤에 켠다
source /opt/ros/humble/setup.bash
[ -f ~/forza_ws/race_stack/install/setup.bash ] && source ~/forza_ws/race_stack/install/setup.bash
set -u
SELF="$(readlink -f "$0")"   # 아래 cd 로 $0 상대경로가 깨지므로 먼저 고정
HERE="$(dirname "$SELF")"
cd ~/forza_ws/race_stack

if [ $# -lt 1 ]; then sed -n '2,24p' "$SELF"; exit 1; fi
MODE="$1"; shift

# --no-pose / --radius 는 이 래퍼가 먹고 분석기로 넘긴다 (sysid_cmd 로는 안 보낸다)
NOPOSE=0; ANALYZE_EXTRA=()
ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --no-pose) NOPOSE=1; ANALYZE_EXTRA+=(--no-pose); shift ;;
    --radius)  ANALYZE_EXTRA+=(--radius "$2"); ARGS+=(--radius "$2"); shift 2 ;;
    --mu)      ANALYZE_EXTRA+=(--mu "$2"); shift 2 ;;
    *) ARGS+=("$1"); shift ;;
  esac
done
set -- "${ARGS[@]+"${ARGS[@]}"}"
DIR=~/forza_ws/race_stack/sysid_${MODE}_$(date +%m%d_%H%M%S)
mkdir -p "$DIR"

# ---------- 사전 점검 ----------
echo "=== 사전 점검 ==="
NODES="$(ros2 node list 2>/dev/null || true)"
fail=0
for n in vesc_driver_node ackermann_to_vesc_node; do
  if grep -q "$n" <<<"$NODES"; then echo "  [OK] $n"; else echo "  [없음] $n  <- base_system 이 안 떠 있다"; fail=1; fi
done
if grep -qE "mux_controller|l1_controller|rl_controller" <<<"$NODES"; then
  echo "  [경고] 컨트롤러가 떠 있다. /drive 명령이 섞인다 — time_trials 를 끄고 다시 할 것."
  fail=1
fi
CHECK_TOPICS=(/joy /sensors/imu/raw /sensors/core)
[ $NOPOSE -eq 0 ] && CHECK_TOPICS+=(/car_state/pose)
for t in "${CHECK_TOPICS[@]}"; do
  n=$(timeout 3 ros2 topic hz "$t" --window 5 2>/dev/null | grep -m1 "average rate" || true)
  if [ -n "$n" ]; then echo "  [OK] $t  ${n}"; else echo "  [죽음] $t 가 안 온다"; fail=1; fi
done
if [ $fail -ne 0 ]; then
  read -r -p "  문제가 있다. 그래도 진행? [y/N] " a; [[ "${a:-N}" =~ ^[yY]$ ]] || exit 1
fi

# ---------- 녹화 ----------
echo; echo "=== 녹화 시작: $DIR/bag ==="
"$HERE/record_sysid.sh" -o "$DIR/bag" >"$DIR/bag.log" 2>&1 &
BAGPID=$!
sleep 2

cleanup() {
  kill -INT $BAGPID 2>/dev/null || true
  wait $BAGPID 2>/dev/null || true
}
trap cleanup EXIT

# ---------- 주행 ----------
echo "=== 명령 노드 (R1 을 누르고 있어야 움직인다) ==="
python3 "$HERE/sysid_cmd.py" --mode "$MODE" --out "$DIR/cmd.csv" "$@" || true

cleanup; trap - EXIT
sleep 1

# ---------- 분석 ----------
echo; echo "=== 분석 ==="
if [ -f "$DIR/cmd.csv" ]; then
  python3 "$HERE/analyze_sysid.py" "$DIR/cmd.csv" --plot "$DIR/plot.png" \
    "${ANALYZE_EXTRA[@]+"${ANALYZE_EXTRA[@]}"}" 2>&1 | tee "$DIR/report.txt"
else
  echo "cmd.csv 가 없다. bag 으로 직접:  python3 $HERE/analyze_sysid.py $DIR/bag"
fi
echo; echo "결과: $DIR"
