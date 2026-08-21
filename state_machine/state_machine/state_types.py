import enum

class StateType(enum.Enum):
    GB_TRACK = 'GB_TRACK' 
    TRAILING = 'TRAILING' 
    OVERTAKE = 'OVERTAKE' 
    FTGONLY = 'FTGONLY'

    TRAILING_TO_GBTRACK = 'TRAILING_TO_GBTRACK'

    # 장애물 앞에서 해가 없어 정지해 버렸을 때 중심선을 따라 뒤로 물러나는 복구 상태.
    # 진입/이탈 조건은 transitions.SpliniTrailingTransition / SpliniRecoverTransition,
    # 실제 후진 웨이포인트 생성은 states.Recovering 참고.
    RECOVER = 'RECOVER'
    # LOW_BAT = 'LOW_BAT'
