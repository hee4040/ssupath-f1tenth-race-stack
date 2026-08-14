from __future__ import annotations

import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from f1tenth_raceline import cli


EXPECTED_FILES = {
    "centerline.csv": "x_m,y_m,width_right_m,width_left_m",
    "raceline_iqp.csv": "s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2",
    "raceline_shortest.csv": "s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2",
    "ltpl.csv": (
        "x_ref_m,y_ref_m,width_right_m,width_left_m,x_normvec_m,y_normvec_m,"
        "alpha_m,s_racetraj_m,psi_racetraj_rad,kappa_racetraj_radpm,"
        "vx_racetraj_mps,ax_racetraj_mps2"
    ),
    "bound_right.csv": "x_m,y_m",
    "bound_left.csv": "x_m,y_m",
}


def test_generate_output_contract_is_unchanged(monkeypatch, tmp_path: Path) -> None:
    map_yaml = tmp_path / "track.yaml"
    map_yaml.write_text("image: track.png\nresolution: 0.05\norigin: [0,0,0]\n")
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    fake = SimpleNamespace(
        centerline_with_width=np.array([[0.0, 0.0, 1.0, 1.0]]),
        raceline_iqp=np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]]),
        raceline_shortest=np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]]),
        ltpl=np.zeros((1, 12)),
        bound_right=np.array([[0.0, -1.0]]),
        bound_left=np.array([[0.0, 1.0]]),
        est_lap_time_iqp=1.23,
        est_lap_time_shortest=1.20,
    )
    calls = []
    def fake_generate(path, **kwargs):
        calls.append((path, kwargs))
        return fake
    monkeypatch.setattr(cli, "generate_racelines", fake_generate)
    out = tmp_path / "out"
    args = argparse.Namespace(
        map=map_yaml, output_dir=out, edit=None, config_dir=config_dir,
        safety_width=0.4, safety_width_sp=0.35, reverse=False, initial_pose=None,
    )
    assert cli.cmd_generate(args) == 0
    assert calls[0][0] == map_yaml.resolve()
    for filename, header in EXPECTED_FILES.items():
        path = out / filename
        assert path.is_file(), filename
        assert path.read_text().splitlines()[0] == header
    summary = json.loads((out / "summary.json").read_text())
    assert set(summary["outputs"]) == {"centerline", "raceline_iqp", "raceline_shortest", "ltpl", "bound_right", "bound_left"}
    assert summary["safety_width"] == 0.4
    assert summary["safety_width_sp"] == 0.35


def test_parser_exposes_sector_workflow() -> None:
    parser = cli.build_parser()
    args = parser.parse_args(["sectors", "--map", "track.yaml"])
    assert args.command == "sectors"
    assert args.func is cli.cmd_sectors
