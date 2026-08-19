from __future__ import annotations

from typing import List, TYPE_CHECKING
import numpy as np
from f110_msgs.msg import Wpnt
from state_machine.state_types import StateType
if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine

def DefaultStateLogic(state_machine: StateMachine) -> List[Wpnt]:
    """
    This is a global state that incorporates the other states
    """
    match state_machine.state:
        case StateType.GB_TRACK:
            return GlobalTracking(state_machine)
        case StateType.TRAILING:
            return Trailing(state_machine)
        case StateType.OVERTAKE:
            return Overtaking(state_machine)
        case StateType.FTGONLY:
            return FTGOnly(state_machine)

        case StateType.TRAILING_TO_GBTRACK:
            return Trailing_to_gbtrack(state_machine)

        case _:
            raise NotImplementedError(f"State {state_machine.state} not recognized")

"""
Here we define the behaviour in the different states.
Every function should be fairly concise, and output an array of f110_msgs.Wpnt
"""
def GlobalTracking(state_machine: StateMachine) -> List[Wpnt]:
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def Trailing_to_gbtrack(state_machine: StateMachine) -> List[Wpnt]:
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def _highspeed_brake_factor(state_machine: StateMachine) -> float:
    """현재 속도에 따라 회피 궤적 속도에 곱할 추가 배율을 돌려준다.

    저속에서는 이미 잘 피하므로 건드리면 손해다. 그래서 hs_brake_v_lo 이하에서는
    정확히 1.0 을 돌려주어 기존 거동을 그대로 보존한다.
    hs_brake_factor 가 1.0 이면 기능 자체가 꺼진 것으로 보고 즉시 1.0 을 돌려준다.
    """
    p = state_machine.params
    f_hi = p.hs_brake_factor
    if f_hi >= 1.0:
        return 1.0
    lo, hi = p.hs_brake_v_lo, p.hs_brake_v_hi
    v = state_machine.cur_vs
    if hi <= lo or v <= lo:
        return 1.0
    if v >= hi:
        return f_hi
    return 1.0 + (f_hi - 1.0) * (v - lo) / (hi - lo)


def Trailing(state_machine: StateMachine) -> List[Wpnt]:
    # This allows us to trail on the last valid spline if necessary
    if state_machine.last_valid_avoidance_wpnts is not None:
        splini_wpts = state_machine.get_splini_wpts()
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        # 기하는 스플라인에서, 속도는 전역(스케일링된) 웨이포인트에서 가져온다.
        # graph_planner 의 속도 프로파일은 vel_max(11 m/s) 기준 raw raceline 속도라
        # speed_scaling.yaml 의 sector scaling / global_limit 이 전혀 반영돼 있지 않다.
        # 그대로 쓰면 TRAILING 중 속도 상한이 실제 목표의 2~3배가 된다. (Overtaking 과 동일 처리)
        # 고속에서는 여기에 추가 감속 배율을 곱한다(저속에서는 1.0 이라 변화 없음).
        return _fuse_speed_from_global(state_machine, splini_wpts, s,
                                       _highspeed_brake_factor(state_machine))
    else:
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]


def _fuse_speed_from_global(state_machine: StateMachine, splini_wpts, s: int,
                            speed_scaling: float = 1.0) -> List[Wpnt]:
    """splini_wpts 의 기하 + glb_wpnts 의 속도로 새 Wpnt 배열을 만든다.

    splini_wpts 는 glb_wpnts 의 얕은 복사라 원소를 직접 고치면 전역 웨이포인트가 오염된다.
    반드시 새 Wpnt 를 만들어 반환할 것.

    전역 속도는 '레이싱라인' 기준으로 최적화된 값이라 회피 궤적에는 그대로 쓸 수 없다.
    회피선은 장애물을 비켜가느라 레이싱라인보다 훨씬 굽어 있는데(측정: 최대 kappa 4.1,
    반지름 0.24 m), 직선 구간의 전역 속도(3.0 m/s)를 그대로 명령하면 필요 횡가속도가
    37 m/s^2 까지 올라간다. 접지 한계(ay_max 5.4)의 7배라 차가 그냥 직진해서 들이받는다.
    저속에서는 명령 속도가 이미 곡률 한계 아래라 문제가 드러나지 않는다.

    그래서 여기서 두 가지를 건다:
      1) 곡률 한계   v <= sqrt(ay_max / |kappa|)
      2) 역방향 패스 앞 구간 속도를 낮춰 ax_max 로 감속이 가능하게 (미리 감속)
    graph_planner 도 같은 프로파일을 계산해 vx_mps 로 실어 보내지만, 그 값은
    스케일링 전 레이싱라인 속도(최대 10 m/s)를 시점으로 잡은 것이라 절대값을 쓸 수 없고,
    회피 구간 밖(진입 전 감속)을 커버하지도 못한다. 로컬 배열 전체에 여기서 다시 건다.
    """
    ay_max = state_machine.params.ay_max_mps2
    ax_max = state_machine.params.ax_max_mps2

    out: List[Wpnt] = []
    for i in range(state_machine.params.n_loc_wpnts):
        idx = (s + i) % state_machine.num_glb_wpnts
        src = splini_wpts[idx]

        wp = Wpnt()
        wp.id = src.id
        wp.s_m = src.s_m
        wp.d_m = src.d_m
        wp.x_m = src.x_m
        wp.y_m = src.y_m
        wp.d_right = src.d_right
        wp.d_left = src.d_left
        wp.psi_rad = src.psi_rad
        wp.kappa_radpm = src.kappa_radpm
        wp.ax_mps2 = src.ax_mps2

        vx = state_machine.glb_wpnts[idx].vx_mps * speed_scaling
        # 1) 이 지점의 곡률에서 접지가 버티는 속도로 제한
        kappa = abs(src.kappa_radpm)
        if kappa > 1e-6:
            vx = min(vx, np.sqrt(ay_max / kappa))
        wp.vx_mps = vx
        out.append(wp)

    # 2) 역방향 패스: 뒤 지점의 속도를 ax_max 로 맞출 수 있을 만큼 앞 지점을 미리 낮춘다.
    #    이게 없으면 회피 구간 '안'에서만 느려져서 이미 늦는다.
    for i in range(len(out) - 2, -1, -1):
        ds = np.hypot(out[i + 1].x_m - out[i].x_m, out[i + 1].y_m - out[i].y_m)
        v_lim = np.sqrt(out[i + 1].vx_mps ** 2 + 2.0 * ax_max * ds)
        if out[i].vx_mps > v_lim:
            out[i].vx_mps = float(v_lim)

    return out

# def Overtaking(state_machine: StateMachine) -> List[Wpnt]:
#     splini_wpts = state_machine.get_splini_wpts()
#     s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
#     return [splini_wpts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]


def Overtaking(state_machine: StateMachine) -> List[Wpnt]:
    # 기하는 회피 스플라인에서, 속도는 전역(스케일링된) 웨이포인트에서 가져온다.
    # (ot_speed_scaling 은 아직 파라미터로 안 만들었음. 일단 코드 내에서 변경해보면서 관찰)
    # 여기에 속도 의존 배율을 곱한다. 저속에서는 1.0 이라 기존 0.8 그대로 유지된다.
    ot_speed_scaling = 0.8 * _highspeed_brake_factor(state_machine)
    splini_wpts = state_machine.get_splini_wpts()
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return _fuse_speed_from_global(state_machine, splini_wpts, s, ot_speed_scaling)

def FTGOnly(state_machine: StateMachine) -> List[Wpnt]:
    """No waypoints are generated in this follow the gap only state, all the 
    control inputs are generated in the control node.
    """
    return []