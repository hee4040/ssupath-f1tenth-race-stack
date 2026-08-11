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

        # ★ 2026-08-11 신설: 피할 것이 없어졌으면 레이싱라인으로 돌아간다.
        #   아래 `not in_ot_sector and _check_gbfree` 가 원래의 유일한 GB_TRACK 복귀
        #   경로인데, 이 저장소의 모든 맵이 ot_sectors.yaml 을 ot_flag: true 로 트랙
        #   전체에 깔아 놓아서(lobby_0806/0807/0811 전부 확인) in_ot_sector 가 항상
        #   True 다. 즉 그 분기는 한 번도 실행된 적이 없고, OVERTAKE 는 사실상
        #   흡수상태(TRAILING 을 경유해야만 빠져나옴)였다.
        #   장애물이 있을 때는 티가 안 나지만 유령이 섞이면 비용이 크다 —
        #   obs_debug_0811_1503(186초, 장애물 0개) 실측: OVERTAKE 63.8초(전체의 34%)
        #   중 69%(44.3초)는 장애물이 아예 없는 상태였고, 그중 30.7초는 차가 이미
        #   레이싱라인 0.4 m 안에 있었다. 유령 하나가 회피선 주행 수 초로 증폭된다.
        #   _check_gbfree(전방 6.9 m, |d|<0.8 이내 장애물 없음) + 레이싱라인 근접을
        #   모두 만족할 때만 복귀하므로, 진짜 추월 중에는 발동하지 않는다
        #   (추월 중이면 상대차가 gb_horizon 안에 있어 gb_free 가 False).
        if state_machine._check_gbfree and state_machine._check_close_to_raceline:
            return StateType.GB_TRACK

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
        