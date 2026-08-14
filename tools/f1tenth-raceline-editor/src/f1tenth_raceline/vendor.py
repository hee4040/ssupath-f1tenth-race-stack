from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

from platformdirs import user_cache_path

UPSTREAM_URL = "https://github.com/hee4040/ssupath-f1tenth-race-stack.git"
UPSTREAM_COMMIT = "acbf008694eef416ed1b189779da2b8f26996909"

SPARSE_PATHS = [
    "planner/global_planner/global_planner/global_racetrajectory_optimization",
    "stack_master/config/global_planner",
]


class VendorError(RuntimeError):
    pass


def _run(cmd: list[str], cwd: Path | None = None) -> str:
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(cwd) if cwd else None,
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
    except FileNotFoundError as exc:
        raise VendorError(
            f"Required executable '{cmd[0]}' was not found. Install Git and ensure it is on PATH."
        ) from exc
    except subprocess.CalledProcessError as exc:
        raise VendorError(
            "Command failed:\n" + " ".join(cmd) + "\n\n" + (exc.stdout or "")
        ) from exc
    return proc.stdout.strip()


def cache_root() -> Path:
    return Path(user_cache_path("f1tenth-raceline", appauthor=False))


def checkout_dir() -> Path:
    return cache_root() / f"ssupath-{UPSTREAM_COMMIT[:12]}"


def current_commit(path: Path) -> str | None:
    if not (path / ".git").exists():
        return None
    try:
        return _run(["git", "rev-parse", "HEAD"], cwd=path)
    except VendorError:
        return None


def ensure_vendor(force: bool = False) -> Path:
    dst = checkout_dir()
    if force and dst.exists():
        shutil.rmtree(dst)
    if current_commit(dst) == UPSTREAM_COMMIT:
        return dst

    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        shutil.rmtree(dst)

    _run(["git", "clone", "--filter=blob:none", "--no-checkout", UPSTREAM_URL, str(dst)])
    _run(["git", "sparse-checkout", "init", "--cone"], cwd=dst)
    _run(["git", "sparse-checkout", "set", *SPARSE_PATHS], cwd=dst)
    _run(["git", "checkout", "--detach", UPSTREAM_COMMIT], cwd=dst)

    actual = current_commit(dst)
    if actual != UPSTREAM_COMMIT:
        raise VendorError(f"Vendor checkout mismatch: expected {UPSTREAM_COMMIT}, got {actual}")
    return dst


def optimizer_python_root(checkout: Path | None = None) -> Path:
    checkout = checkout or ensure_vendor()
    return checkout / "planner" / "global_planner" / "global_planner" / "global_racetrajectory_optimization"


def default_config_dir(checkout: Path | None = None) -> Path:
    checkout = checkout or ensure_vendor()
    return checkout / "stack_master" / "config" / "global_planner"


def activate_optimizer_imports(checkout: Path | None = None) -> Path:
    os.environ.setdefault("MPLBACKEND", "Agg")
    root = optimizer_python_root(checkout)
    if not root.exists():
        raise VendorError(f"Optimizer source directory not found: {root}")
    value = str(root)
    if value not in sys.path:
        sys.path.insert(0, value)

    # TPH 0.80 still carries older SciPy shape behavior.
    from .compat import apply_tph_spline_approximation_compat
    apply_tph_spline_approximation_compat()
    return root
