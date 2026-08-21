#!/usr/bin/env python3
"""
analyze_sysid.py 자기검증. 계수를 '아는' 가짜 차를 굴려 CSV 를 만들고,
분석기가 그 값을 되찾아오는지 본다. 차 없이, ROS 없이 돌아간다.

    python3 sysid_selftest.py            # 전 모드 생성 + 분석까지
    python3 sysid_selftest.py --only circle

현장에 나가기 전에 한 번 돌려서 툴체인이 살아 있는지 확인하는 용도다.
가짜 차의 모델은 학습 레포(env_cfg.py TireModelCfg)와 같은 구조를 쓴다:
    F_y = mu*N*tanh(alpha/alpha_char),  F_drive = clip(k*(v_cmd-v), -f_brake, f_drive)
"""

import argparse
import csv
import math
import os
import subprocess
import sys

import numpy as np

TRUE = dict(mass=4.987, lf=0.171, lr=0.159, Iz=0.05,
            mu=0.95, alpha_char=0.075, alpha_char_f=None, alpha_char_r=None, k_drive=40.0,
            f_drive_max=31.1, f_brake_max=14.2, c_roll=0.015,
            tau_steer=0.05, erpm_gain=3576.0)
G = 9.80665


def simulate(profile, dt=0.002, noise=0.003, P=TRUE):
    L = P['lf'] + P['lr']
    Nf, Nr = P['mass'] * G * P['lr'] / L, P['mass'] * G * P['lf'] / L
    x = y = yaw = vy = wz = 0.0
    vx = 0.0
    delta = 0.0
    rows = []
    t = 0.0
    total = sum(d for _, d, _, _, _ in profile)
    rng = np.random.default_rng(0)
    while t < total:
        # 프로파일 조회
        acc = 0.0
        phase, kind, v_cmd, d_cmd = 'end', 'stop', 0.0, 0.0
        for name, dur, k, vf, df in profile:
            if t < acc + dur:
                phase, kind, v_cmd, d_cmd = name, k, float(vf(t - acc)), float(df(t - acc)); break
            acc += dur
        delta += (d_cmd - delta) * dt / P['tau_steer']

        # 저속 정칙화: vx -> 0 이면 슬립각이 발산해 차가 출발조차 못 한다.
        # 시뮬/실차 모두 쓰는 관용 처리(0.5 m/s 이하에서 횡력을 선형으로 죽인다).
        vxs = max(vx, 0.5)
        blend = min(1.0, vx / 0.5)
        af = delta - math.atan2(vy + P['lf'] * wz, vxs)
        ar = -math.atan2(vy - P['lr'] * wz, vxs)
        acf = P.get('alpha_char_f') or P['alpha_char']
        acr = P.get('alpha_char_r') or P['alpha_char']
        Fyf = blend * P['mu'] * Nf * math.tanh(af / acf)
        Fyr = blend * P['mu'] * Nr * math.tanh(ar / acr)

        if kind == 'coast':
            Fd = 0.0
        else:
            Fd = P['k_drive'] * (v_cmd - vx)
            Fd = max(-P['f_brake_max'], min(P['f_drive_max'], Fd))
            # 뒤축 마찰서클: 횡력을 쓰고 남은 만큼만 종방향에 쓸 수 있다
            cap = math.sqrt(max((P['mu'] * Nr) ** 2 - Fyr ** 2, 0.0))
            Fd = max(-cap, min(cap, Fd))
        Fd -= P['c_roll'] * P['mass'] * G * np.sign(vx) if vx > 0.02 else 0.0

        ax = Fd / P['mass'] - Fyf * math.sin(delta) / P['mass'] + vy * wz
        ay = (Fyf * math.cos(delta) + Fyr) / P['mass'] - vx * wz
        dwz = (P['lf'] * Fyf * math.cos(delta) - P['lr'] * Fyr) / P['Iz']
        vx = max(0.0, vx + ax * dt); vy += ay * dt; wz += dwz * dt
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        yaw += wz * dt

        if len(rows) * 0.02 <= t:      # 50 Hz 로만 기록 (실제 CSV 와 동일)
            v_wheel = vx + max(Fd, 0.0) / max(P['mu'] * Nr, 1e-6) * 0.4 * (1 if Fd > 0.95 * (
                math.sqrt(max((P['mu'] * Nr) ** 2 - Fyr ** 2, 0.0))) else 0)
            rows.append(dict(
                t=t, phase=phase, kind=kind, v_cmd=v_cmd, delta_cmd=d_cmd,
                servo_cmd=0.5 - 0.65 * d_cmd, erpm_cmd=v_cmd * P['erpm_gain'],
                erpm_meas=v_wheel * P['erpm_gain'],
                current_motor=abs(Fd) * 2.0, duty=min(0.99, vx / 9.0 + abs(Fd) / 120.0),
                v_batt=15.5, wz=wz, ax=ax, ay=ay, v_pose=vx,
                # ★실차 base_link 는 뒤축이다(2026-08-21 실증). pose 도 뒤축으로 낸다 —
                #   CoM 으로 내면 사이드슬립에 lr/R 이 섞여 alpha_char 검증이 어긋난다.
                x=x - P['lr'] * math.cos(yaw) + rng.normal(0, noise),
                y=y - P['lr'] * math.sin(yaw) + rng.normal(0, noise),
                yaw=yaw + rng.normal(0, noise / 2), pose_t=1e9 + t))
        t += dt
    return rows


def write(rows, path):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    return path


def profiles():
    k = lambda c: (lambda t: c)
    return {
        'circle': [('entry', 3.0, 'drive', k(1.0), lambda t: 0.28 * min(1, t / 1.5)),
                   ('ramp', 14.0, 'drive', lambda t: 1.0 + 4.0 * t / 14.0, k(0.28)),
                   ('brake', 2.0, 'drive', k(0.0), k(0.28))],
        'accel':  [('launch', 3.0, 'drive', k(7.0), k(0.0)),
                   ('brake', 2.5, 'drive', k(0.0), k(0.0))],
        'step':   [('spool', 1.5, 'drive', lambda t: 2.0 * t / 1.5, k(0.0)),
                   ('base', 1.5, 'drive', k(2.0), k(0.0)),
                   ('up0', 1.5, 'drive', k(2.4), k(0.0)), ('dn0', 1.5, 'drive', k(2.0), k(0.0)),
                   ('up1', 1.5, 'drive', k(2.4), k(0.0)), ('dn1', 1.5, 'drive', k(2.0), k(0.0)),
                   ('up2', 1.5, 'drive', k(2.4), k(0.0)), ('dn2', 1.5, 'drive', k(2.0), k(0.0)),
                   ('brake', 2.0, 'stop', k(0.0), k(0.0))],
        'coast':  [('spool', 1.5, 'drive', lambda t: 5.0 * t / 1.5, k(0.0)),
                   ('hold', 2.0, 'drive', k(5.0), k(0.0)),
                   ('coast', 8.0, 'coast', k(0.0), k(0.0))],
        'steer':  [('spool', 2.0, 'drive', lambda t: 3.0 * min(1, t / 1.5), k(0.0)),
                   ('L0', 0.6, 'drive', k(3.0), k(0.25)), ('R0', 0.6, 'drive', k(3.0), k(-0.25)),
                   ('L1', 0.6, 'drive', k(3.0), k(0.25)), ('R1', 0.6, 'drive', k(3.0), k(-0.25)),
                   ('brake', 1.5, 'stop', k(0.0), k(0.0))],
        'const':  [('spool', 1.5, 'drive', lambda t: 1.5 * t / 1.5, k(0.0)),
                   ('hold', 6.0, 'drive', k(1.5), k(0.0)),
                   ('brake', 1.5, 'stop', k(0.0), k(0.0))],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', default=None)
    ap.add_argument('--outdir', default='/tmp/sysid_selftest')
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    here = os.path.dirname(os.path.abspath(__file__))

    print('가짜 차의 참값:', {k: v for k, v in TRUE.items() if k in
                          ('mu', 'alpha_char', 'k_drive', 'f_drive_max', 'f_brake_max', 'c_roll')})
    for name, prof in profiles().items():
        if a.only and name != a.only:
            continue
        path = write(simulate(prof), os.path.join(a.outdir, f'sysid_{name}.csv'))
        print(f'\n\n############ {name}  ->  {path}')
        subprocess.run([sys.executable, os.path.join(here, 'analyze_sysid.py'), path,
                        '--mode', {'const': 'gain'}.get(name, 'auto')])


if __name__ == '__main__':
    main()
