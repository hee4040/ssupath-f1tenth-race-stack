"""레이스라인(/global_waypoints) 기준 Frenet(s, d) -> 맵 좌표 변환.

perception 의 tracking 노드는 장애물/상대차를 Frenet 좌표(s_center, d_center)로
발행하는데, 그 기준선은 **레이스라인**(`/global_waypoints`)이다. RL 컨트롤러의
관측은 **중심선**(`/centerline_waypoints`) 기준이라 두 s 를 직접 비교할 수 없다.
그래서 상대차를 일단 맵 좌표로 되돌린 뒤(여기), 컨트롤러의 TrackReference 에
다시 투영해 중심선 기준 s/횡오차를 얻는다.

변환식은 frenet_conversion_cpp 의 `get_cartesian` 과 같다:

    x = X(s) + d * cos(psi(s) + pi/2)
    y = Y(s) + d * sin(psi(s) + pi/2)          (즉 d > 0 = 진행방향 왼쪽)

원본은 s 에 대한 3차 스플라인을 쓰지만, 웨이포인트 간격이 0.1m 라 구간 선형
보간과의 차이는 mm 수준이다. psi 는 메시지의 `psi_rad` 를 쓰지 않고 이웃
웨이포인트의 차분에서 직접 구한다(발행부의 psi 관례 변화에 영향받지 않도록).
"""
from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np


class GlobalPath:
    """s 로 인덱싱되는 닫힌 폴리라인."""

    def __init__(self, s: np.ndarray, x: np.ndarray, y: np.ndarray):
        self.s = s
        self.x = x
        self.y = y
        self.total_s = float(s[-1])          # 닫힘점 포함 -> 마지막이 랩 길이
        self.dx = np.diff(x)
        self.dy = np.diff(y)
        self.seglen = np.maximum(np.hypot(self.dx, self.dy), 1e-9)

    # ------------------------------------------------------------------ #
    @classmethod
    def from_waypoints(cls, s_m: Sequence[float], x_m: Sequence[float],
                       y_m: Sequence[float]) -> "GlobalPath":
        s = np.asarray(s_m, dtype=np.float64)
        x = np.asarray(x_m, dtype=np.float64)
        y = np.asarray(y_m, dtype=np.float64)
        if len(s) < 4:
            raise ValueError(f"레이스라인 웨이포인트가 너무 적습니다: {len(s)}")
        if not np.all(np.diff(s) > 0.0):
            raise ValueError("레이스라인 s 가 단조증가가 아닙니다")
        # 폐곡선 닫기: 마지막 점 뒤에 첫 점을 s = 랩길이 위치로 덧붙인다.
        close = math.hypot(x[0] - x[-1], y[0] - y[-1])
        s = np.append(s, s[-1] + max(close, 1e-6))
        x = np.append(x, x[0])
        y = np.append(y, y[0])
        return cls(s, x, y)

    # ------------------------------------------------------------------ #
    def to_cartesian(self, s: float, d: float) -> Tuple[float, float]:
        """Frenet (s, d) -> 맵 (x, y). s 는 [0, 랩길이) 로 래핑한다."""
        s = math.fmod(float(s), self.total_s)
        if s < 0.0:
            s += self.total_s
        i = int(np.searchsorted(self.s, s, side="right") - 1)
        i = min(max(i, 0), len(self.seglen) - 1)
        t = (s - self.s[i]) / self.seglen[i]
        px = self.x[i] + t * self.dx[i]
        py = self.y[i] + t * self.dy[i]
        # 좌측(+) 법선 = 접선을 +90도 회전
        nx = -self.dy[i] / self.seglen[i]
        ny = self.dx[i] / self.seglen[i]
        return float(px + d * nx), float(py + d * ny)

    def summary(self) -> str:
        return f"n={len(self.s) - 1} lap={self.total_s:.1f}m"
