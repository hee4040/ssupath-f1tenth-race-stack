"""체크포인트 경로 해석.

가중치는 레포 안(`models/`)에 복사돼 있고 노드는 학습 워크스페이스
(`~/shared_dir/dacerpp_isaaclab`)를 참조하지 않는다. 설정에 상대경로를 쓰면
이 모듈이 패키지의 models/ 를 기준으로 절대경로를 만들어 준다.

ROS 의존이 없는 별도 모듈인 이유: check_rl_setup.py 같은 오프라인 도구가
rclpy/f110_msgs 없이도 같은 규칙을 쓸 수 있어야 하기 때문이다.
"""
from __future__ import annotations

import os

# 배포 기본 가중치. 학습 car_b(pow) 가 실차 배포 대상이다 (models/README.md 참조).
# ★2026-08-19: *_healthy.pt 로 바꿨다. 학습 train.py 는 Pow 의 조향 다양성 pstd 가
#   붕괴 기준(0.02) 위인 동안만 *_healthy.pt 를 갱신한다 — 런 최종 pow.pt 는 붕괴
#   영역(pstd 0.013~0.018)이라 배포하면 안 된다.
# models/ 에는 배포 대상 두 개만 평평하게 둔다 (setup.py 가 models/*.pt 만 설치하므로
# 날짜 폴더 안의 파일은 install 공간에 들어가지 않는다).
DEFAULT_CKPT = "pow_healthy.pt"


def models_roots() -> list:
    """models/ 후보 디렉터리 — install 공간 우선, 없으면 소스 트리."""
    roots = []
    try:
        from ament_index_python.packages import get_package_share_directory
        roots.append(os.path.join(get_package_share_directory("rl_controller"), "models"))
    except Exception:                                              # noqa: BLE001
        pass
    roots.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              "models"))
    return roots


def resolve_checkpoint(raw: str) -> str:
    """절대경로는 그대로, 상대경로는 패키지 models/ 기준으로 해석."""
    path = os.path.expanduser(str(raw))
    if os.path.isabs(path):
        return path
    roots = models_roots()
    for root in roots:
        cand = os.path.join(root, path)
        if os.path.isfile(cand):
            return cand
    return os.path.join(roots[0], path) if roots else path
