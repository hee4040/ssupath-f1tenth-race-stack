"""DACER++ 디퓨전 정책 추론 전용 구현.

dacerpp_isaaclab/dacer_pp 의 networks.py / diffusion.py / risk.py 에서 추론에
필요한 부분만 그대로 옮겨온 것이다(수식·레이어 구성 동일). 학습 패키지를
그대로 import 하지 않는 이유:
  - 학습 패키지는 torch.compile / CUDA Graphs 경로를 켜고 들어가는데, 실차에서는
    배치 1 추론이라 이득이 없고 첫 호출 컴파일이 수 초 걸린다.
  - 배포 노드가 ~/shared_dir 경로 존재에 의존하지 않게 하기 위함.
체크포인트(cvar.pt)는 학습 코드가 저장한 그대로 사용한다.

체크포인트 구조: {"policy": state_dict, "qvn1"/"qvn2": state_dict, "cfg": DACERppConfig,
"log_alpha": tensor, "step": int}. cfg 는 pickle 된 학습 측 dataclass 라
언피클에 dacer_pp.config 모듈이 필요하므로 스텁 모듈을 미리 심는다.
"""
from __future__ import annotations

import math
import sys
import types
from typing import Optional, Sequence

import numpy as np

from .torch_loader import import_torch

torch = import_torch()
import torch.nn as nn  # noqa: E402  (torch import 이후여야 함)


# --------------------------------------------------------------------------- #
# 체크포인트 언피클용 스텁 (학습 패키지 없이 cfg 를 복원)
# --------------------------------------------------------------------------- #
def _install_unpickle_stubs():
    if "dacer_pp.config" in sys.modules:
        return

    class DACERppConfig:  # noqa: D401 - 언피클 대상 자리표시자
        pass

    class RiskConfig:
        pass

    pkg = sys.modules.get("dacer_pp") or types.ModuleType("dacer_pp")
    cfg_mod = types.ModuleType("dacer_pp.config")
    cfg_mod.DACERppConfig = DACERppConfig
    cfg_mod.RiskConfig = RiskConfig
    pkg.config = cfg_mod
    sys.modules.setdefault("dacer_pp", pkg)
    sys.modules["dacer_pp.config"] = cfg_mod


# --------------------------------------------------------------------------- #
# 네트워크 (학습본 dacer_pp/networks.py 와 동일 구성)
# --------------------------------------------------------------------------- #
def _mlp(in_dim: int, hidden: Sequence[int], out_dim: int) -> nn.Sequential:
    layers, last = [], in_dim
    for h in hidden:
        layers += [nn.Linear(last, h), nn.ReLU()]
        last = h
    layers += [nn.Linear(last, out_dim)]
    return nn.Sequential(*layers)


class DiffusionPolicyNet(nn.Module):
    """역확산 노이즈 예측망."""

    def __init__(self, obs_dim: int, act_dim: int, hidden: Sequence[int], time_dim: int = 16):
        super().__init__()
        self.act_dim = act_dim
        self.time_dim = time_dim
        self.time_mlp = nn.Sequential(
            nn.Linear(time_dim, time_dim * 2), nn.ReLU(), nn.Linear(time_dim * 2, time_dim))
        self.net = _mlp(obs_dim + act_dim + time_dim, hidden, act_dim)
        half = time_dim // 2
        freq = torch.arange(half, dtype=torch.float32) / half
        self.register_buffer("inv_freq", 10000.0 ** (-freq))
        self.time_scale = 1.0 / math.sqrt(time_dim)

    def forward(self, obs: torch.Tensor, x: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
        t = t.float().reshape(-1)
        emb = t.unsqueeze(-1) * self.inv_freq
        emb = torch.cat([torch.sin(emb), torch.cos(emb)], dim=-1) * self.time_scale
        if emb.shape[0] == 1 and obs.shape[0] != 1:
            emb = emb.expand(obs.shape[0], -1)
        return self.net(torch.cat([obs, x, self.time_mlp(emb)], dim=-1))


class QuantileValueNetwork(nn.Module):
    """IQN 스타일 QVN. num_action_candidates > 1 일 때만 사용."""

    def __init__(self, obs_dim: int, act_dim: int, hidden: Sequence[int],
                 embedding_dim: int = 256, num_cosines: int = 64):
        super().__init__()
        self.psi = _mlp(obs_dim + act_dim, hidden, embedding_dim)
        self.phi = nn.Sequential(nn.Linear(num_cosines, embedding_dim), nn.ReLU())
        self.f = _mlp(embedding_dim, hidden, 1)
        self.register_buffer(
            "cos_idx", torch.arange(1, num_cosines + 1, dtype=torch.float32) * math.pi)

    def forward(self, obs: torch.Tensor, act: torch.Tensor, taus: torch.Tensor) -> torch.Tensor:
        psi = self.psi(torch.cat([obs, act], dim=-1))
        phi = self.phi(torch.cos(taus.unsqueeze(-1) * self.cos_idx))
        return self.f(psi.unsqueeze(1) * phi).squeeze(-1)


def _distort(tau: torch.Tensor, measure: str, eta: float) -> torch.Tensor:
    """왜곡 위험측도 beta(tau) (dacer_pp/risk.py)."""
    if measure == "neutral":
        return tau
    if measure == "cvar":
        return eta * tau
    if measure == "pow":
        p = 1.0 / (1.0 + abs(eta))
        if eta >= 0:
            return tau.clamp(0.0, 1.0) ** p
        return 1.0 - (1.0 - tau).clamp(0.0, 1.0) ** p
    raise ValueError(f"unknown risk measure: {measure}")


def _vp_beta_schedule(timesteps: int) -> np.ndarray:
    t = np.arange(1, timesteps + 1)
    b_max, b_min = 10.0, 0.1
    alpha = np.exp(-b_min / timesteps - 0.5 * (b_max - b_min) * (2 * t - 1) / timesteps ** 2)
    return 1.0 - alpha


# --------------------------------------------------------------------------- #
class DacerPolicy:
    """체크포인트에서 복원한 추론 전용 정책."""

    def __init__(self, policy_net: DiffusionPolicyNet, diffusion_coeffs: dict,
                 obs_dim: int, act_dim: int, device: str,
                 qvns: Optional[Sequence[QuantileValueNetwork]] = None,
                 risk_measure: str = "neutral", risk_eta: float = 1.0,
                 risk_num_quantiles: int = 32, num_candidates: int = 1,
                 explore_noise_std: float = 0.0, seed: Optional[int] = None):
        self.net = policy_net
        self.d = diffusion_coeffs
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.device = device
        self.qvns = list(qvns) if qvns else []
        self.risk_measure = risk_measure
        self.risk_eta = risk_eta
        self.risk_num_quantiles = risk_num_quantiles
        self.num_candidates = max(1, int(num_candidates))
        self.explore_noise_std = float(explore_noise_std)
        self.gen = torch.Generator(device="cpu")
        if seed is not None:
            self.gen.manual_seed(int(seed))
        self._T = int(self.d["T"])

    # ------------------------------------------------------------------ #
    @classmethod
    def load(cls, path: str, device: str = "cpu", num_candidates: Optional[int] = None,
             num_timesteps: Optional[int] = None, seed: Optional[int] = None,
             explore_noise_std: float = 0.0) -> "DacerPolicy":
        _install_unpickle_stubs()
        sd = torch.load(path, map_location="cpu", weights_only=False)
        if "policy" not in sd:
            raise ValueError(f"{path}: 'policy' 키가 없습니다 (DACER++ 체크포인트가 아님)")
        pol_sd = sd["policy"]
        cfg = sd.get("cfg")

        # 차원/구조는 state_dict 형상에서 직접 복원한다 (cfg 언피클 실패에도 안전).
        time_dim = int(pol_sd["time_mlp.0.weight"].shape[1])
        idxs = sorted(int(k.split(".")[1]) for k in pol_sd
                      if k.startswith("net.") and k.endswith(".weight"))
        act_dim = int(pol_sd[f"net.{idxs[-1]}.weight"].shape[0])
        obs_dim = int(pol_sd[f"net.{idxs[0]}.weight"].shape[1]) - act_dim - time_dim
        hidden = tuple(int(pol_sd[f"net.{i}.weight"].shape[0]) for i in idxs[:-1])

        net = DiffusionPolicyNet(obs_dim, act_dim, hidden, time_dim=time_dim)
        net.load_state_dict(pol_sd)
        net.to(device).eval()
        for p in net.parameters():
            p.requires_grad_(False)

        T = int(num_timesteps if num_timesteps else getattr(cfg, "num_timesteps", 10))
        coeffs = cls._make_coeffs(T, device)

        risk = getattr(cfg, "risk", None)
        measure = str(getattr(risk, "measure", "neutral"))
        eta = float(getattr(risk, "eta", 1.0))
        rq = int(getattr(risk, "num_quantiles", 32))
        n_cand = int(num_candidates if num_candidates is not None
                     else getattr(cfg, "num_action_candidates", 1))

        qvns = []
        if n_cand > 1:
            emb = int(sd["qvn1"]["phi.0.weight"].shape[0])
            ncos = int(sd["qvn1"]["phi.0.weight"].shape[1])
            q_idxs = sorted(int(k.split(".")[1]) for k in sd["qvn1"]
                            if k.startswith("psi.") and k.endswith(".weight"))
            q_hidden = tuple(int(sd["qvn1"][f"psi.{i}.weight"].shape[0]) for i in q_idxs[:-1])
            for key in ("qvn1", "qvn2"):
                q = QuantileValueNetwork(obs_dim, act_dim, q_hidden,
                                         embedding_dim=emb, num_cosines=ncos)
                q.load_state_dict(sd[key])
                q.to(device).eval()
                for p in q.parameters():
                    p.requires_grad_(False)
                qvns.append(q)

        return cls(net, coeffs, obs_dim, act_dim, device, qvns=qvns,
                   risk_measure=measure, risk_eta=eta, risk_num_quantiles=rq,
                   num_candidates=n_cand, explore_noise_std=explore_noise_std, seed=seed)

    @staticmethod
    def _make_coeffs(T: int, device: str) -> dict:
        betas = _vp_beta_schedule(T)
        alphas = 1.0 - betas
        ac = np.cumprod(alphas, axis=0)
        acp = np.append(1.0, ac[:-1])
        post_var = betas * (1.0 - acp) / (1.0 - ac)

        def t(x):
            return torch.as_tensor(x, dtype=torch.float32, device=device)

        return {
            "T": T,
            "sqrt_recip_ac": t(np.sqrt(1.0 / ac)),
            "sqrt_recipm1_ac": t(np.sqrt(1.0 / ac - 1.0)),
            "post_std": t(np.sqrt(np.maximum(post_var, 1e-20))),
            "mean_c1": t(betas * np.sqrt(acp) / (1.0 - ac)),
            "mean_c2": t((1.0 - acp) * np.sqrt(alphas) / (1.0 - ac)),
            "ts": torch.arange(T, device=device),
        }

    # ------------------------------------------------------------------ #
    @torch.no_grad()
    def _sample(self, obs: torch.Tensor) -> torch.Tensor:
        """역확산 샘플링 (학습 agent._build_sampler 와 동일 수식)."""
        d = self.d
        x = torch.randn(obs.shape[0], self.act_dim, generator=self.gen,
                        dtype=obs.dtype).to(obs.device)
        for i in range(self._T - 1, -1, -1):
            eps = self.net(obs, x, d["ts"][i])
            x0 = (x * d["sqrt_recip_ac"][i] - eps * d["sqrt_recipm1_ac"][i]).clamp(-1.0, 1.0)
            mean = x0 * d["mean_c1"][i] + x * d["mean_c2"][i]
            if i > 0:
                noise = torch.randn(x.shape, generator=self.gen, dtype=x.dtype).to(x.device)
                x = mean + d["post_std"][i] * noise
            else:
                x = mean
        return x

    @torch.no_grad()
    def _q_beta(self, qvn: QuantileValueNetwork, obs: torch.Tensor,
                act: torch.Tensor) -> torch.Tensor:
        taus = torch.rand(obs.shape[0], self.risk_num_quantiles, generator=self.gen,
                          dtype=obs.dtype).to(obs.device)
        taus = _distort(taus, self.risk_measure, self.risk_eta)
        return qvn(obs, act, taus).mean(dim=1)

    @torch.no_grad()
    def act(self, obs_np: np.ndarray) -> np.ndarray:
        """(obs_dim,) -> (act_dim,) numpy, 각 성분 [-1, 1]."""
        obs = torch.as_tensor(np.asarray(obs_np, dtype=np.float32).reshape(1, -1),
                              device=self.device)
        M = self.num_candidates
        rep = obs if M <= 1 else obs.expand(M, obs.shape[-1]).contiguous()
        act = self._sample(rep)
        if self.explore_noise_std > 0.0:
            noise = torch.randn(act.shape, generator=self.gen,
                                dtype=act.dtype).to(act.device) * self.explore_noise_std
            act = act + noise
        act = act.clamp(-1.0, 1.0)
        if M > 1:
            # 학습 select_action 과 동일: 왜곡 위험측도 Q 로 후보 중 최선을 고른다.
            q = torch.min(self._q_beta(self.qvns[0], rep, act),
                          self._q_beta(self.qvns[1], rep, act))
            act = act[int(q.argmax())]
        else:
            act = act[0]
        return act.cpu().numpy()

    def warmup(self, n: int = 10) -> None:
        dummy = np.zeros(self.obs_dim, dtype=np.float32)
        for _ in range(n):
            self.act(dummy)
