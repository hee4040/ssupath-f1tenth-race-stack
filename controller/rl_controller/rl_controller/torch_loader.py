"""torch import 헬퍼.

이 젯슨의 torch(2.11, /usr/local/lib/python3.10/dist-packages)는 CUDA 런타임
공유라이브러리를 ~/.local/lib/python3.10/site-packages/nvidia/*/lib 에 두고 있어
(설치 경로가 갈려 torch 의 자체 preload 가 실패) `import torch` 가

    ImportError: libcudss.so.0: cannot open shared object file

로 죽는다. LD_LIBRARY_PATH 를 launch 에 박는 대신, import 실패 시 '에러가 지목한
그 라이브러리만' 찾아 ctypes 로 RTLD_GLOBAL 선적재하고 재시도한다.

★ 디렉터리의 .so 를 전부 선적재하면 안 된다: libnvblas.so 는 BLAS 호출을 가로채는
   인터셉터라 CPU BLAS 백엔드 없이 올라가면 그 뒤 numpy/torch 연산에서 SIGSEGV 로
   죽는다 (실측 2026-07-28).
"""
from __future__ import annotations

import ctypes
import glob
import os
import re
import site
import sys

_MISSING_RE = re.compile(r"(lib[\w.+-]*\.so(?:\.\d+)*)\s*:\s*cannot open shared object file")
_MAX_RETRY = 32


def _candidate_dirs():
    dirs = []
    try:
        usr = site.getusersitepackages()
        dirs.append(usr if isinstance(usr, str) else usr[0])
    except Exception:
        pass
    try:
        dirs.extend(site.getsitepackages())
    except Exception:
        pass
    dirs.extend(p for p in sys.path if p and os.path.isdir(p))
    seen, out = set(), []
    for d in dirs:
        if d not in seen:
            seen.add(d)
            out.append(d)
    return out


def find_shared_object(soname: str):
    """nvidia/* 휠 안에서 soname 을 찾아 절대경로를 반환."""
    for base in _candidate_dirs():
        for pat in ("nvidia/*/lib/" + soname, "nvidia/*/lib64/" + soname,
                    "nvidia/*/lib/" + soname + ".*"):
            hits = sorted(glob.glob(os.path.join(base, pat)))
            if hits:
                return hits[0]
    return None


def _purge_torch_modules():
    for name in [k for k in list(sys.modules) if k == "torch" or k.startswith("torch.")]:
        del sys.modules[name]


def import_torch(verbose: bool = False):
    """torch 를 import 한다. 없는 CUDA 런타임 .so 는 필요한 것만 선적재하며 재시도."""
    loaded = []
    for _ in range(_MAX_RETRY):
        try:
            import torch  # noqa: F401
            if verbose and loaded:
                print(f"[rl_controller] 선적재한 CUDA 런타임: {', '.join(loaded)}")
            return torch
        except (ImportError, OSError) as exc:
            m = _MISSING_RE.search(str(exc))
            if m is None:
                raise
            soname = m.group(1)
            path = find_shared_object(soname)
            if path is None:
                raise ImportError(
                    f"{soname} 를 찾을 수 없습니다. torch 가 요구하는 CUDA 런타임이 "
                    f"설치되어 있는지 확인하세요 (원본 오류: {exc})") from exc
            ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
            loaded.append(soname)
            _purge_torch_modules()
    raise ImportError(f"torch import 재시도 한계 초과 (선적재: {loaded})")


def resolve_device(torch, requested: str) -> str:
    """'auto'/'cuda'/'cpu' -> 실제 사용 가능한 device 문자열.

    젯슨에서 pip torch 의 CUDA 커널이 이 GPU 아키텍처(sm_87)로 빌드되지 않은
    경우 `cuda.is_available()` 은 True 인데 커널 실행에서 'device kernel image is
    invalid' 로 죽는다. 그래서 실제 연산을 한 번 돌려보고 판정한다.
    """
    req = (requested or "auto").lower()
    if req == "cpu":
        return "cpu"
    if not torch.cuda.is_available():
        return "cpu"
    try:
        x = torch.ones(8, device="cuda")
        _ = float((x * 2.0).sum().item())
        return "cuda"
    except Exception:
        return "cpu"
