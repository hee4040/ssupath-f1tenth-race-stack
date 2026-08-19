from __future__ import annotations

from typing import TYPE_CHECKING

from state_machine.state_types import StateType

if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine


def dummy_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            if state_machine._low_bat:
                return StateType.LOW_BAT
            else:
                return StateType.GB_TRACK
        case StateType.LOW_BAT:
            return StateType.LOW_BAT
        case default:
            return StateType.GB_TRACK
        
        
def timetrials_transition(state_machine: StateMachine)->str:
    return StateType.GB_TRACK

def head_to_head_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            # print("i'm innnnnnnnnnnnnnnnnnn")
            return SpliniGlobalTrackingTransition(state_machine)
        case StateType.TRAILING:
            return SpliniTrailingTransition(state_machine)

        case StateType.TRAILING_TO_GBTRACK:
            return SpliniTrailingToGbtrackTransition(state_machine)

        case StateType.OVERTAKE:
            return SpliniOvertakingTransition(state_machine)
        case StateType.FTGONLY:
            return SpliniFTGOnlyTransition(state_machine)
        case default:
            raise ValueError(f"Invalid state {state_machine.state}")


def SpliniGlobalTrackingTransition(state_machine: StateMachine) -> StateType:
    # print("HELLOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")

    """Transitions for being in `StateType.GB_TRACK`"""
    if not state_machine._check_only_ftg_zone:
        if state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY

def SpliniTrailingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING`"""
    gb_free = state_machine._check_gbfree
    ot_sector = state_machine._check_ot_sector

    if not state_machine._check_only_ftg_zone:
        # If we have been sitting around in TRAILING for a while then FTG
        # print("gb_freeeeeeeeeeeeeee eeeeeeeeeee", gb_free)
        # print("ooooooooooooooooooooooooooooooooot_sector", ot_sector)
        # print("check_availability:   ", state_machine._check_availability_splini_wpts)
        # print("ot_free:   ", state_machine._check_ofree)
        # print("gb_free: ", gb_free)
        # print("ot_sector: ", ot_sector)

        # print("ccccccccccccccccccccccccccccccccheck_ofree", state_machine._check_ofree)
        if state_machine._check_ftg:
            return StateType.FTGONLY
        if not gb_free and not ot_sector:
            return StateType.TRAILING

        # 아래와 같이 바로 GB_TRACK로 전환하지 않고 TRAILING_TO_GBTRACK로 전환하도록 수정
        elif gb_free and state_machine._check_close_to_raceline:
            # print("Trailing to GB_TRACKKKKKKKKKKKKKKKKKk")
            # print("gb_free: ", gb_free)
            # print("close_to_gb: ", state_machine._check_close_to_raceline)

            return StateType.TRAILING_TO_GBTRACK

        elif (
            not gb_free 
            and ot_sector 
            and state_machine._check_availability_splini_wpts 
            and state_machine._check_ofree
        ):
            print("START___________________________OVERTAKING",gb_free, ot_sector, state_machine._check_availability_splini_wpts, state_machine._check_ofree)
            return StateType.OVERTAKE
        else:
            print("AGAIN_TRAILING", gb_free, ot_sector, state_machine._check_availability_splini_wpts, state_machine._check_ofree)
            return StateType.TRAILING
    else:
        return StateType.FTGONLY

def SpliniTrailingToGbtrackTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING_TO_GBTRACK`"""
    # GB_TRACK 이외의 다른 상태로 return 할 때에는 trailing_to_gbtrack_count를 0으로 리셋해주기
    print("ENTER TRAILING_TO_GBTRACK")
    gb_free = state_machine._check_gbfree
    ot_sector = state_machine._check_ot_sector

    if not state_machine._check_only_ftg_zone:
        # If we have been sitting around in TRAILING for a while then FTG
        if state_machine._check_ftg:
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.FTGONLY
        if not gb_free and not ot_sector:
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.TRAILING


        elif gb_free and state_machine._check_close_to_raceline:

            state_machine.trailing_to_gbtrack_count += 1

            # gb_free의 횟수가 threshold를 넘기면 그때는 진짜로 gbtrack으로 전환
            if state_machine.trailing_to_gbtrack_count >= state_machine.trailing_to_gbtrack_counting_threshold:
                state_machine.trailing_to_gbtrack_count = 0
                return StateType.GB_TRACK

            else:
                return StateType.TRAILING_TO_GBTRACK

        elif (
            not gb_free
            and ot_sector
            and state_machine._check_availability_splini_wpts
            and state_machine._check_ofree
        ):
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.OVERTAKE
        else:
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.TRAILING
    else:
        state_machine.trailing_to_gbtrack_count = 0
        return StateType.FTGONLY



def SpliniOvertakingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.OVERTAKE`"""
    if not state_machine._check_only_ftg_zone:
        in_ot_sector = state_machine._check_ot_sector
        spline_valid = state_machine._check_availability_splini_wpts
        o_free = state_machine._check_ofree

        # If spline is on an obstacle we trail
        if not o_free:
            return StateType.TRAILING

        # ★ 2026-08-11: 여기에 `gb_free and close_to_raceline -> GB_TRACK` 복귀 분기를
        #   넣었다가 **실차에서 역효과라 철회**했다. 같은 길 다시 가지 말 것.
        #
        #   의도는 맞았다: 이 저장소의 모든 맵이 ot_sectors.yaml 을 ot_flag: true 로
        #   트랙 전체에 깔아 놓아서(lobby_0806/0807/0811) in_ot_sector 가 항상 True 이고,
        #   그래서 아래 `not in_ot_sector and _check_gbfree` 복귀 경로는 한 번도 실행된
        #   적이 없다. OVERTAKE 는 사실상 흡수상태(TRAILING 경유로만 탈출)다.
        #
        #   문제는 종료 신호로 _check_gbfree 를 쓴 것이다. 이 스택의 탐지는 간헐적이라
        #   (원거리 클러스터 연속성 23~28%) "전방에 장애물 없음"이 수시로 거짓으로 뜬다.
        #   obs_debug_0811_1726(327초) 실측 — 이 분기가 26회 발동했는데 그중 10회(38%)는
        #   복귀 직후 3초 안에 장애물이 다시 전방 6.9 m 에 나타났다. 재출현까지 중앙 0.25초,
        #   최소 0.05초. 9회는 0.5초 이내 = 탐지가 한두 프레임 끊긴 사이에 복귀한 것이다.
        #   복귀 시점 차량 횡위치는 중앙 0.33 m 로 회피 기동 한복판이었다.
        #   즉 회피선을 타다가 장애물 앞에서 레이싱라인으로 되돌아갔다가 다시 급제동한다.
        #
        #   구제 시도도 전부 실패했다(obs_debug_0811_1726 오프라인 시뮬레이션):
        #     gb_free 연속 유지 디바운스 0.5/0.75/1.0/1.5초 : 조기복귀 38% -> 23% 가 한계
        #     + 스플라인 TTL 만료 조건 / 스플라인 끝 통과 조건 : 37회 중 1회만 발동(무의미)
        #   탐지 끊김이 초 단위라 '앞이 비었다'는 신호 자체를 종료조건으로 쓸 수 없다.
        #
        #   ★ 제대로 고치려면 ot_sectors.yaml 에서 실제 추월 구간에만 ot_flag: true 를
        #     주면 된다. 그러면 아래 원래 분기(`not in_ot_sector and gb_free`)가 의도대로
        #     동작한다 — 그건 차의 s 위치로만 판정하므로 탐지 끊김에 영향받지 않는다.
        #     유령 때문에 OVERTAKE 가 길어지는 문제는 상태머신이 아니라 퍼셉션 쪽
        #     (cluster_to_obstacle 의 min_intrusion) 에서 잡는 게 맞다.

        if in_ot_sector and o_free and spline_valid:
            return StateType.OVERTAKE
        # If spline becomes unvalid while overtaking, we trail
        elif in_ot_sector and not spline_valid and not o_free:
            return StateType.TRAILING
        # go to GB_TRACK if not in ot_sector and the GB is free
        elif not in_ot_sector and state_machine._check_gbfree:
            return StateType.GB_TRACK
        # go to Trailing if not in ot_sector and the GB is not free
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniFTGOnlyTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    else:
        if state_machine._check_close_to_raceline and state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.FTGONLY
        