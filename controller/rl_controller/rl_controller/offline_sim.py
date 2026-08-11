"""맵을 실제로 주행해 보는 오프라인 폐루프 검증기.

주행 전에 "이 맵에서 이 정책이 도는가"를 직접 확인하기 위한 것이다. 임계값 몇 개로
맵 난이도를 점치는 것보다 훨씬 정확하다 — 2026-07-30 lobby_0730 사고 때, 곡률/폭
임계값으로는 학습 트랙과 구분되지 않았지만(절차 생성 트랙 60종을 재생성해 비교:
|kappa|max 중앙 1.44/최대 1.84, 급코너 반전 최소간격 중앙 1.05m 로 lobby_0730 보다
오히려 빡빡했다) 이 폐루프 검증은 실차와 같은 지점(s≈3.4m)에서 같은 방식으로
실패해 사고를 정확히 재현했다.

동역학은 dacerpp_lab/racing_env.py 의 `_apply_tire_forces` / `_drive_one` 을 그대로
옮긴 것이다(같은 수식, 물리 120Hz / 제어 30Hz). PhysX 강체 대신 평면 3자유도
(vx, vy, r)를 적분하고 전복/충돌은 다루지 않는다. 관성 Izz 는 학습 자산의
assets/f1tenth/f1tenth.urdf 링크 관성 합(≈0.084).

★ 순수 운동학 자전거 모델을 쓰면 안 된다: 그립 한계가 없어 풀락에서 요레이트가
  5.7rad/s(그립 한계 ~3.4의 1.7배)까지 나오고, 학습 분포 밖 관측이 되어 정책이
  발진한다. 정책이 고장난 것처럼 보이지만 하네스 문제다.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np

# ---- 학습 파라미터 (dacerpp_lab/env_cfg.py TireModelCfg / RacingCfg) ----
# 공칭 마찰. 학습 TireModelCfg.mu 와 같은 값을 쓴다 — 이 값이 학습과 다르면
# "정책이 자기 세계에서 도는가"라는 질문 자체가 성립하지 않는다.
# 이력: 0.75 -> 0.90(2026-08-07) -> 1.05(2026-08-09). 실차가 코너에서 바깥벽에
# 붙는 과보수 주행을 그립 부족 인식으로 보고 학습 그립을 올린 실험이다
# (학습 mu_range 0.85~1.25).
# ★ 학습 주석의 경고 그대로: 실제 도막 그립이 이보다 낮으면 정책이 그립을
#   과대평가해 언더스티어가 재발할 수 있다. check_rl_setup.py --mu 로 낮은 마찰에서도
#   돌려 보고, 현장 스키드패드 실측이 나오면 이 값을 그 값으로 고칠 것.
MU_NOM, ALPHA_CHAR = 1.05, 0.08
MASS, COM_H, LF, LR = 3.94, 0.07, 0.149, 0.181
K_DRIVE, F_DRIVE_MAX, C_ROLL, V_LAT_TAPER = 40.0, 22.0, 0.015, 0.3
IZZ, G = 0.084, 9.81
MAX_STEER, STEER_LIMIT, STEER_K, STEER_VLIM = 0.42, 0.44, 10.0, 20.0
PHYS_DT, DECIMATION = 1.0 / 120.0, 4
CTRL_DT = PHYS_DT * DECIMATION
N_BEAMS, FOV, RMAX = 32, 2.356, 10.0
HW_REF, OBS_VMAX, V_MIN = 2.5, 10.0, 1.0
CURV_OFF = (5, 15, 30, 60, 90)
WIDTH_OFF = (0,) + CURV_OFF
# 곡률 관측 클립. 20260805 세대 학습부터 ±2 (config 의 curv_clip 과 같은 값을 유지할 것).
CURV_CLIP = 2.0
OFFTRACK_MARGIN, SPIN_HERR = -0.20, 1.745
ANGLES = np.linspace(-FOV, FOV, N_BEAMS)


class CarSim:
    """평면 3자유도 + 학습 해석 타이어 모델."""

    def __init__(self, x, y, yaw, mu=MU_NOM, v0=1.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.vx, self.vy, self.r, self.delta = v0, 0.0, 0.0, 0.0
        self.mu = mu

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)

    def step(self, act, v_cmd_max):
        delta_cmd = float(np.clip(act[0], -1, 1)) * MAX_STEER
        v_cmd = V_MIN + (float(np.clip(act[1], -1, 1)) + 1.0) * 0.5 * (v_cmd_max - V_MIN)
        for _ in range(DECIMATION):
            self._substep(delta_cmd, v_cmd)

    def _substep(self, delta_cmd, v_cmd):
        d_dot = float(np.clip(STEER_K * (delta_cmd - self.delta), -STEER_VLIM, STEER_VLIM))
        self.delta = float(np.clip(self.delta + d_dot * PHYS_DT, -STEER_LIMIT, STEER_LIMIT))
        delta, vx, vy, r = self.delta, self.vx, self.vy, self.r
        L = LF + LR

        fx_des = float(np.clip(K_DRIVE * (v_cmd - vx), -F_DRIVE_MAX, F_DRIVE_MAX))
        ax_est = fx_des / MASS
        nf = max(MASS * G * LR / L - MASS * ax_est * COM_H / L, 0.0)
        nr = max(MASS * G * LF / L + MASS * ax_est * COM_H / L, 0.0)

        vx_eff = max(abs(vx), 0.5)
        taper = math.tanh(abs(vx) / V_LAT_TAPER)
        fyf = -self.mu * nf * math.tanh((math.atan2(vy + LF * r, vx_eff) - delta)
                                        / ALPHA_CHAR) * taper
        fyr = -self.mu * nr * math.tanh(math.atan2(vy - LR * r, vx_eff) / ALPHA_CHAR) * taper

        scale_r = min(self.mu * nr / max(math.hypot(fx_des, fyr), 1e-6), 1.0)
        fx_r, fyr = fx_des * scale_r, fyr * scale_r
        f_roll = -C_ROLL * MASS * G * math.tanh(vx / 0.2)
        cd, sd = math.cos(delta), math.sin(delta)

        self.vx += ((fx_r + f_roll - fyf * sd) / MASS + vy * r) * PHYS_DT
        self.vy += ((fyr + fyf * cd) / MASS - vx * r) * PHYS_DT
        self.r += ((LF * fyf * cd - LR * fyr) / IZZ) * PHYS_DT
        self.yaw += self.r * PHYS_DT
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (self.vx * cy - self.vy * sy) * PHYS_DT
        self.y += (self.vx * sy + self.vy * cy) * PHYS_DT


def build_walls(tr) -> np.ndarray:
    """중심선 ± 반폭 벽 폴리라인 -> (S,2,2) 세그먼트(시작점, 방향)."""
    nrm = np.stack([-np.sin(tr.psi), np.cos(tr.psi)], axis=1)
    out = []
    for side in (+1.0, -1.0):
        poly = tr.pts + side * nrm * tr.hw[:, None]
        out.append(np.stack([poly, np.roll(poly, -1, axis=0) - poly], axis=1))
    return np.concatenate(out, axis=0)


def raycast(pos: np.ndarray, angles: np.ndarray, walls: np.ndarray,
            max_range: float = RMAX) -> np.ndarray:
    a, s = walls[:, 0, :], walls[:, 1, :]
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    e = a - pos
    den = d[:, None, 0] * s[None, :, 1] - d[:, None, 1] * s[None, :, 0]
    ok = np.abs(den) > 1e-9
    q = np.where(ok, den, 1.0)
    t = (e[None, :, 0] * s[None, :, 1] - e[None, :, 1] * s[None, :, 0]) / q
    u = (e[None, :, 0] * d[:, None, 1] - e[None, :, 1] * d[:, None, 0]) / q
    hit = ok & (u >= -1e-6) & (u <= 1 + 1e-6) & (t >= 0)
    return np.minimum(np.where(hit, t, np.inf).min(axis=1), max_range)


def make_obs(tr, walls, car: CarSim, hist: np.ndarray, scan_noise: float = 0.0,
             rng: Optional[np.random.Generator] = None):
    proj = tr.project(car.x, car.y)
    idx = proj["idx"]
    hw = float(tr.hw[idx])
    herr = (car.yaw - proj["psi"] + math.pi) % (2 * math.pi) - math.pi
    scan = raycast(np.array([car.x, car.y]), car.yaw + ANGLES, walls)
    if scan_noise and rng is not None:
        scan = np.clip(scan + rng.normal(0.0, scan_noise, N_BEAMS), 0.02, RMAX)
    obs = np.concatenate([
        np.clip(scan / RMAX, 0, 1),
        [car.speed / OBS_VMAX, math.sin(herr), math.cos(herr)],
        [np.clip(proj["lateral"] / hw, -2, 2)],
        np.clip(tr.lookahead_curvature(idx, CURV_OFF), -CURV_CLIP, CURV_CLIP),
        np.clip(tr.lookahead_width(idx, WIDTH_OFF) / HW_REF, 0, 1),
        # 상대차 5개는 0 = 미검출. 이 검증기는 단독 주행만 본다.
        np.clip(hist, -1, 1), np.zeros(5),
        [np.clip(car.r / 4.0, -1, 1), np.clip(car.vy / 3.0, -1, 1)],
    ]).astype(np.float32)
    return obs, proj, hw, herr


def rollout(tr, walls, policy, v_max: float, seconds: float = 30.0, start_idx: int = 0,
            mu: float = MU_NOM, delay: int = 1, scan_noise: float = 0.02,
            seed: int = 0) -> dict:
    """한 번 주행. delay 는 명령 인가 지연(제어 스텝, 학습 act_delay 0~2 의 중앙값)."""
    rng = np.random.default_rng(seed)
    car = CarSim(float(tr.pts[start_idx, 0]), float(tr.pts[start_idx, 1]),
                 float(tr.psi[start_idx]), mu=mu, v0=1.0)
    hist = np.zeros(4, dtype=np.float32)
    queue = [np.zeros(2, dtype=np.float32) for _ in range(delay)]
    prev_s = tr.project(car.x, car.y)["s"]
    travelled, min_margin, steers = 0.0, 9.9, []

    for _ in range(int(seconds / CTRL_DT)):
        obs, proj, hw, herr = make_obs(tr, walls, car, hist, scan_noise, rng)
        min_margin = min(min_margin, hw - abs(proj["lateral"]))
        if abs(proj["lateral"]) > hw + OFFTRACK_MARGIN:
            return dict(ok=False, reason="이탈", s=proj["s"], travelled=travelled,
                        min_margin=min_margin, steers=np.array(steers))
        if abs(herr) > SPIN_HERR:
            return dict(ok=False, reason="스핀", s=proj["s"], travelled=travelled,
                        min_margin=min_margin, steers=np.array(steers))
        act = np.asarray(policy.act(obs), dtype=np.float32)
        queue.append(act)
        car.step(queue.pop(0), v_max)
        steers.append(float(act[0]) * MAX_STEER)
        hist = np.concatenate([act[:2], hist[:2]]).astype(np.float32)

        s_now = tr.project(car.x, car.y)["s"]
        ds = s_now - prev_s
        if ds < -0.5 * tr.total_s:
            ds += tr.total_s
        elif ds > 0.5 * tr.total_s:
            ds -= tr.total_s
        travelled += ds
        prev_s = s_now

    return dict(ok=True, reason="완주", s=prev_s, travelled=travelled,
                min_margin=min_margin, steers=np.array(steers))


def evaluate(tr, policy, v_max_list: Sequence[float] = (2.0, 3.0, 5.0),
             starts: int = 6, seconds: float = 30.0, mu: float = MU_NOM,
             delay: int = 1, log=print) -> dict:
    """여러 시작점 x 여러 v_max 로 돌려 완주율과 실패 지점을 보고."""
    walls = build_walls(tr)
    n = len(tr.pts)
    idxs = [int(n * j / starts) for j in range(starts)]
    result = {}
    for v_max in v_max_list:
        ok, prog, fails, sat = 0, [], [], []
        for si, s0 in enumerate(idxs):
            r = rollout(tr, walls, policy, v_max, seconds, s0, mu=mu, delay=delay, seed=si)
            ok += int(r["ok"])
            prog.append(r["travelled"])
            if len(r["steers"]):
                sat.append(float(np.mean(np.abs(r["steers"]) > 0.41)))
            if not r["ok"]:
                fails.append(f"{r['reason']}@s={r['s']:.1f}m")
        result[v_max] = dict(ok=ok, n=starts, travelled=float(np.mean(prog)), fails=fails)
        log(f"    v_max={v_max:4.1f}: 완주 {ok}/{starts}  "
            f"평균진행 {np.mean(prog):6.1f}m ({np.mean(prog) / tr.total_s:.2f}랩/{seconds:.0f}s)  "
            f"조향포화 {np.mean(sat) if sat else 0:.0%}")
        if fails:
            log(f"              실패: {', '.join(fails)}")
    return result
