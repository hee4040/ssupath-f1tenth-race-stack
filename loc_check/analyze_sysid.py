#!/usr/bin/env python3
"""
sysid_cmd.py 로 딴 런(CSV 또는 bag)에서 물리계수를 역산한다.

    python3 analyze_sysid.py ~/forza_ws/race_stack/sysid_circle_0820_1512.csv
    python3 analyze_sysid.py <bag디렉터리> --mode accel
    python3 analyze_sysid.py <csv> --mu 0.95        # mu 를 이미 알 때(2.1 선행)
    python3 analyze_sysid.py <csv> --plot out.png

모드는 CSV 의 phase 라벨로 자동 판별한다(--mode 로 강제 가능).

식별 한계 (0819 보고서 2.3) 를 코드가 직접 판정한다:
    가속 = min(f_drive_max, mu*N_r)/m 이라 가속 로그 하나로는 둘이 분리되지 않는다.
    그런데 어느 쪽에 걸렸는지는 '휠 슬립'으로 구분된다:
      ERPM 환산 휠속도 > pose 실측 지면속도  ->  바퀴가 헛돈다 = 마찰 한계
      두 속도가 붙어 있다                    ->  힘 한계. 이때 f_drive_max = m*a 로
                                                 곧장 나오고 com_height 는 무관하다.
    그래서 com_height 를 못 재도 f_drive_max 는 잡을 수 있다(힘 한계인 경우).
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sysid_io as S

G = S.G


# ------------------------------------------------------------------ 공통 유틸
def runs(labels):
    """['a','a','b'] -> [('a',0,2), ('b',2,3)]"""
    out, i = [], 0
    for j in range(1, len(labels) + 1):
        if j == len(labels) or labels[j] != labels[i]:
            out.append((labels[i], i, j)); i = j
    return out


def pick(tr, prefixes):
    return [(l, i, j) for l, i, j in runs(tr.phase)
            if any(l.startswith(p) for p in prefixes) and j - i > 5]


def hdr(s):
    print(f'\n{"="*72}\n{s}\n{"="*72}')


def kv(k, v, note=''):
    print(f'  {k:<34s} {v:>12s}   {note}')


# ------------------------------------------------------------------ 게인 검증
def an_gain(tr, C):
    hdr('speed_to_erpm_gain 검증  (ERPM 실측 vs pose 실측 속도)')
    if tr.no_pose:
        print('  측위가 없으면 ERPM 을 ERPM 과 비교하는 꼴이라 아무 의미가 없다.')
        print('  이 경우의 유일한 방법은 줄자다: const 모드로 직선을 달리고 실제 이동거리를 재서')
        print('  ros2 topic echo /odom 의 position 이동량과 비교한다 (/odom 이 ERPM 환산이므로).')
        return
    m = (tr.v > 1.0) & (np.abs(tr.a_long) < 0.8) & np.isfinite(tr.erpm) & (tr.erpm > 500)
    if m.sum() < 50:
        print('  정속 구간 표본 부족 — const 모드로 1~2 m/s 를 5초쯤 유지한 런이 필요하다')
        return
    ratio = tr.erpm[m] / tr.v[m]
    g_meas = float(np.median(ratio))
    kv('현재 설정값 (vesc.yaml)', f'{C.erpm_gain:.1f}')
    kv('실측 ERPM/속도', f'{g_meas:.1f}', f'표본 {int(m.sum())}, IQR {np.percentile(ratio,75)-np.percentile(ratio,25):.0f}')
    kv('오차', f'{100*(C.erpm_gain/g_meas-1):+.1f} %',
       '(+면 차가 명령보다 느리고 /odom 은 거리를 과대계상)')
    s_pose = float(np.trapz(tr.v[m], tr.t[m]))
    s_erpm = float(np.trapz(tr.erpm[m] / C.erpm_gain, tr.t[m]))
    kv('같은 구간 이동거리', f'{s_pose:.2f} m', f'(휠오돔 환산 {s_erpm:.2f} m)')
    if abs(C.erpm_gain / g_meas - 1) > 0.03:
        print(f'  ★ vesc.yaml 의 speed_to_erpm_gain 을 {g_meas:.0f} 로 바꾸고 sync_config.sh 실행.')
        print('    줄자 교차검증: const 모드로 5 m 를 달리고 실제 이동거리와 위 pose 거리를 비교.')


# ------------------------------------------------------------------ 2.1 / 2.2
def an_circle(tr, C):
    hdr('2.1 mu  +  2.2 alpha_char   (정상 조향각 + 속도 램프)')
    segs = pick(tr, ['ramp', 'entry'])
    if segs:
        i0 = min(i for _, i, _ in segs); i1 = max(j for _, _, j in segs)
        m = np.zeros_like(tr.t, bool); m[i0:i1] = True
    else:
        m = (np.abs(tr.delta) > 0.05)
    m &= (tr.v > C.vmin) & np.isfinite(tr.wz)
    if m.sum() < 100:
        print('  표본 부족 (v>%.1f, |delta|>0.05 인 구간이 없다)' % C.vmin)
        return

    v, wz, d = tr.v[m], tr.wz[m], tr.delta[m]
    if C.prefer_erpm and np.isfinite(tr.v_erpm).any():
        # pose 속도를 못 믿을 때. 선회 중 휠슬립은 종방향보다 작아서
        # 정상원 구간에서는 ERPM 이 pose 보다 낫다.
        v = tr.v_erpm[m]
        print('  a_lat 을 ERPM 환산 속도로 계산한다 (--prefer-erpm)')
    if C.radius:
        # v = wz*R 이므로 a_lat = v*wz = wz^2*R. 속도 계측이 통째로 빠진다.
        # ★단 차가 실제로 그 반경을 돌고 있어야 한다. circle 모드는 조향각을 고정하므로
        #   슬립이 커질수록 반경이 커진다 -> --radius 는 '운전자가 콘 원을 손으로 따라간'
        #   주행에만 쓸 것.
        a_lat = wz * wz * C.radius * np.sign(wz)
        print(f'  a_lat 을 wz^2 * R (R={C.radius} m) 로 계산한다 — 속도 계측/ERPM 게인과 무관하다.')
        if not tr.no_pose:
            R_meas = float(np.nanmedian(np.abs(v / np.where(np.abs(wz) < 1e-3, np.nan, wz))))
            if abs(R_meas - C.radius) / C.radius > 0.15:
                print(f'  ★ 실제로 돈 반경은 {R_meas:.2f} m 다 (준 값 {C.radius:.2f} m, '
                      f'{100*(R_meas/C.radius-1):+.0f}%). mu 는 R 에 비례하므로 그만큼 틀린다.')
    else:
        a_lat = v * wz
    R = v / np.where(np.abs(wz) < 1e-3, np.nan, wz)
    af, ar = tr.slip_angles(C.lf, C.lr)
    af, ar = af[m], ar[m]

    # --- IMU 자이로 부호/축 확인 (이게 틀리면 아래가 전부 무의미) ---
    r = np.corrcoef(wz, tr.yawrate_pose[m])[0, 1]
    kv('IMU wz vs pose 요레이트 상관', f'{r:+.3f}',
       'OK' if r > 0.9 else '★ 축/부호 확인 필요')

    # --- 한계: 횡가속 최대 ---
    k = int(np.argmax(np.abs(a_lat)))
    mu_peak = abs(a_lat[k]) / G
    kv('실현 반경 R (중앙값)', f'{np.nanmedian(np.abs(R)):.2f} m')
    kv('|a_lat| 최대', f'{abs(a_lat[k]):.2f} m/s^2', f'v={v[k]:.2f} m/s, R={abs(R[k]):.2f} m')
    kv('|a_lat| p99', f'{np.percentile(np.abs(a_lat),99):.2f} m/s^2')
    kv('-> mu (하한)', f'{mu_peak:.3f}', 'p99 기준 %.3f' % (np.percentile(np.abs(a_lat),99)/G))

    if tr.no_pose:
        print('\n  측위가 없어 횡속도(beta)를 모른다 -> 슬립각이 무의미하므로'
              ' alpha_char 피팅은 건너뛴다.')
        if not C.radius:
            print('  ★ 게다가 v 가 ERPM 환산이라 바퀴가 헛돌면 mu 가 과대평가된다.'
                  ' --radius 를 주고 다시 볼 것.')
        return

    # --- tanh 피팅: |a_lat| = mu*g*tanh(|alpha|/alpha_c) ---
    # 이 타이어모델은 정적 하중배분에서 alpha_f = alpha_r 이 되도록 짜여 있다.
    # 그래서 언더스티어 기울기로는 alpha_char 가 안 나오고, 슬립각 자체가 필요하다.
    # 슬립각은 pose 미분으로 얻은 차체 횡속도(beta)에서 온다.
    for nm, al in (('rear', ar), ('front', af)):
        x, y = np.abs(al), np.abs(a_lat)
        good = np.isfinite(x) & np.isfinite(y) & (x < 0.5)
        if good.sum() < 50:
            continue
        best = None
        for mu_t in np.arange(0.30, 1.81, 0.01):
            for ac in np.arange(0.010, 0.301, 0.002):
                res = y[good] - mu_t * G * np.tanh(x[good] / ac)
                sse = float(res @ res)
                if best is None or sse < best[0]:
                    best = (sse, mu_t, ac)
        sse, mu_f, ac_f = best
        rms = math.sqrt(sse / good.sum())
        kv(f'tanh 피팅 ({nm} slip)', f'mu={mu_f:.3f}', f'alpha_char={ac_f:.3f} rad ({math.degrees(ac_f):.1f} deg), RMS {rms:.2f} m/s^2')
        if x[good].max() < 1.2 * ac_f:
            print(f'     ※ 슬립각이 alpha_char 까지 못 갔다(최대 {x[good].max():.3f} rad).'
                  ' 조향각을 키우거나 속도를 더 올려야 alpha_char 가 확정된다.')

    # --- 간이 교차검증: 기하 요레이트 대비 실측 비 ---
    wz_kin = v * np.tan(d) / C.L
    ok = np.abs(wz_kin) > 0.2
    if ok.sum() > 50:
        ratio = np.abs(wz[ok] / wz_kin[ok])
        order = np.argsort(np.abs(a_lat[ok]))
        rr = ratio[order]; aa = np.abs(a_lat[ok])[order]
        n = max(10, len(rr)//20)
        lo = float(np.median(rr[:n]))
        drop = np.where(rr < 0.9 * lo)[0]
        kv('요레이트비 (실측/기하)', f'{lo:.2f} @저횡가속',
           f'0.9배로 떨어지는 a_lat = {aa[drop[0]]:.2f} m/s^2' if len(drop) else '한계 전까지 유지')

    print('\n  해석:')
    print('    - mu 는 좌/우 회전 각각 3회 이상 돌려 중앙값을 쓸 것(좌우 하중 51.4:48.6 비대칭).')
    print('    - tanh 피팅의 mu 와 위 "하한" mu 가 크게 다르면 한계까지 안 간 것이다.')


# ------------------------------------------------------------------ 2.3 / 2.4
def an_accel(tr, C):
    hdr('2.3 f_drive_max / 고속 처짐   +   2.4 f_brake_max')
    segs = pick(tr, ['launch', 'spool'])
    if not segs:
        j = np.where(np.diff(tr.v_cmd) > 1.0)[0]
        if len(j) == 0:
            print('  가속 구간을 못 찾았다'); return
        segs = [('launch', int(j[0]), min(len(tr.t) - 1, int(j[0]) + 400))]
    l, i0, i1 = segs[0]

    m = np.zeros_like(tr.t, bool); m[i0:i1] = True
    m &= tr.v > 0.3
    if m.sum() < 20:
        print('  가속 표본 부족'); return

    if tr.no_pose:
        print('  ★ 측위가 없으면 지면속도를 모른다 = 휠슬립을 검출할 수 없다.')
        print('    가속이 힘 한계인지 마찰 한계인지 갈리지 않으므로 f_drive_max 를 낼 수 없다.')
        print('    (바퀴가 헛돌면 ERPM 가속이 실제보다 크게 나오는데 그걸 알 방법이 없다)')
        print('    아래 숫자는 참고용이다. 이 종목만은 측위가 있는 곳에서 다시 잴 것.')

    # --- 휠 슬립 판정: 힘 한계인가 마찰 한계인가 ---
    slip_ok = (not tr.no_pose) and np.isfinite(tr.v_erpm).mean() > 0.5
    if slip_ok:
        slip = tr.v_erpm[m] - tr.v[m]
        s50 = float(np.median(slip)); s90 = float(np.percentile(slip, 90))
        kv('휠슬립 (ERPM속도 - 지면속도)', f'{s50:+.2f} m/s', f'p90 {s90:+.2f}')
        limited = 'friction' if s90 > C.slip_thresh else 'force'
    else:
        limited = 'unknown'
        print('  /sensors/core 가 없어 슬립 판정 불가')

    # 목표속도에 도달한 뒤 구간은 '가속 중'이 아니라 정속이다. 빼지 않으면
    # 고속 구간 가속이 0 으로 찍혀 역기전력 처짐과 구분되지 않는다.
    m &= (tr.v_cmd - tr.v) > 0.5
    if m.sum() < 20:
        print('  포화 가속 구간 표본 부족 — --v 를 더 크게 주고 다시 딸 것'); return

    a = tr.a_long[m]; v = tr.v[m]
    a_lo = float(np.percentile(a[v < 2.0], 90)) if (v < 2.0).sum() > 10 else float(np.percentile(a, 90))
    kv('정지~2 m/s 구간 최대가속', f'{a_lo:.2f} m/s^2')

    # 속도 구간별 가속 (역기전력 처짐 확인)
    print('\n  속도구간별 가속 / 듀티 / 전류:  (명령속도까지 0.5 m/s 이상 남은 구간만)')
    edges = np.arange(0, math.ceil(v.max()) + 1, 1.0)
    for k in range(len(edges) - 1):
        b = (v >= edges[k]) & (v < edges[k + 1])
        if b.sum() < 5:
            continue
        du = np.nanmedian(tr.duty[m][b]) if np.isfinite(tr.duty).any() else np.nan
        cu = np.nanmedian(tr.current[m][b]) if np.isfinite(tr.current).any() else np.nan
        print(f'    {edges[k]:.0f}~{edges[k+1]:.0f} m/s : a={np.median(a[b]):5.2f} m/s^2'
              f'   duty={du:5.2f}   I={cu:6.1f} A   (n={int(b.sum())})')
    if np.isfinite(tr.duty).any():
        dmax = float(np.nanmax(np.abs(tr.duty[m])))
        if dmax > 0.9:
            print(f'    ★ duty 최대 {dmax:.2f} — 전압/역기전력 한계에 닿았다. 이 위 속도의'
                  ' 가속은 힘 상한이 아니라 전압으로 정해진다(모델에 속도 의존성이 없다).')

    if limited == 'force':
        f = C.mass * (a_lo + C.c_roll * G)
        kv('\n  -> f_drive_max', f'{f:.1f} N', f'= {C.mass:.3f} kg x ({a_lo:.2f} + c_roll*g)')
        print('     휠 슬립이 없다 = 힘 한계다. 이 값은 com_height 와 무관하게 확정된다.')
    elif limited == 'friction':
        print('\n  -> 바퀴가 헛돌았다 = 마찰 한계. 이 런으로는 f_drive_max 를 못 낸다.')
        print('     대신 mu 와 com_height 의 관계가 나온다:  a = mu*g*lf / (L - mu*h)')
        if C.mu:
            h = (C.L - C.mu * G * C.lf / a_lo) / C.mu
            kv('  주어진 mu 로 역산한 com_height', f'{h*100:.1f} cm',
               'OK' if 0.03 < h < 0.15 else '★ 비물리적 — 4WD 라 뒤축 마찰서클 가정이 틀렸을 수 있다')
        else:
            print('     --mu 를 주면 com_height 를 역산한다 (2.5 기울임법 대체).')
        f = C.mass * (a_lo + C.c_roll * G)
        kv('  참고: 이 가속에 해당하는 힘', f'{f:.1f} N', '(= f_drive_max 의 하한)')

    # --- 제동 ---
    bs = pick(tr, ['brake'])
    if bs:
        _, j0, j1 = bs[0]
        mb = np.zeros_like(tr.t, bool); mb[j0:j1] = True
        mb &= tr.v > 0.5
        if mb.sum() > 10:
            ab = float(np.percentile(tr.a_long[mb], 10))     # 가장 강한 감속 쪽
            kv('\n  제동 감속 (p10)', f'{ab:.2f} m/s^2')
            kv('  -> f_brake_max', f'{C.mass*(abs(ab)-C.c_roll*G):.1f} N',
               '(c_roll 분 제외)')
            if slip_ok:
                sb = float(np.median(tr.v_erpm[mb] - tr.v[mb]))
                kv('  제동 중 휠슬립', f'{sb:+.2f} m/s',
                   '음수 크면 락업 = 마찰 한계' if sb < -C.slip_thresh else '락업 없음 = 힘 한계')


# ------------------------------------------------------------------ k_drive
def an_step(tr, C):
    hdr('2.3 k_drive   (작은 속도 계단의 접근 시정수)')
    segs = pick(tr, ['up', 'dn'])
    if not segs:
        print('  step 모드 phase 라벨이 없다. sysid_cmd.py --mode step 으로 딴 CSV 가 필요하다.')
        return

    # ★ 여기서만은 ERPM 환산 속도를 쓴다. 시정수는 게인에 불변이기 때문:
    #   v 와 v_cmd 가 같은 게인으로 스케일되면 지수 e^(-t/tau) 의 tau 는 안 변한다.
    #   반대로 pose 미분 속도는 잡음이 0.1 m/s 급이라 0.4 m/s 계단에서 tau 가 안 잡힌다.
    use_erpm = np.isfinite(tr.v_erpm).mean() > 0.5
    vsig = tr.v_erpm if use_erpm else tr.v
    print(f'  속도 신호: {"ERPM 환산 (게인 오차는 tau 에 영향 없음)" if use_erpm else "pose 미분 (잡음 주의)"}')

    taus = []
    print('  구간   v0->v_cmd    dv     tau[s]   k_drive[N/(m/s)]  RMS[m/s]  포화?')
    for l, i0, i1 in segs:
        vt = float(np.median(tr.v_cmd[i0:i1]))
        v0 = float(np.median(vsig[max(i0 - 3, 0):i0 + 2]))
        dv = vt - v0
        if abs(dv) < 0.05:
            continue
        t = tr.t[i0:i1] - tr.t[i0]
        y = vsig[i0:i1]
        if not np.isfinite(y).all() or len(t) < 10:
            continue
        # tau 그리드 탐색 + (v_inf, A) 는 매 tau 마다 선형 최소자승으로 푼다.
        # log 선형화보다 안정적이다(꼬리에서 log 가 발산하지 않는다).
        best = None
        for tau in np.arange(0.02, 1.001, 0.002):
            X = np.vstack([np.ones_like(t), np.exp(-t / tau)]).T
            c, *_ = np.linalg.lstsq(X, y, rcond=None)
            r = y - X @ c
            sse = float(r @ r)
            if best is None or sse < best[0]:
                best = (sse, tau, c)
        sse, tau, c = best
        rms = math.sqrt(sse / len(t))
        amp = -c[1]                     # 정착값에서 시작값을 뺀 크기
        if not (0.03 < tau < 0.9) or abs(amp) < 0.4 * abs(dv) or abs(amp) > 2.0 * abs(dv):
            print(f'  {l:>5s}  {v0:4.2f}->{vt:4.2f}  {dv:+5.2f}      -- 피팅 기각'
                  f' (tau={tau:.3f}, 진폭 {amp:+.2f})')
            continue
        k = C.mass / tau
        sat = 'YES' if abs(dv) > C.f_drive_max / max(k, 1e-6) else ''
        taus.append(tau)
        print(f'  {l:>5s}  {v0:4.2f}->{vt:4.2f}  {dv:+5.2f}   {tau:6.3f}   {k:8.1f}       {rms:6.3f}   {sat}')
    if taus:
        tau = float(np.median(taus))
        kv('\n  중앙값 tau', f'{tau:.3f} s', f'표본 {len(taus)}개, 산포 {np.std(taus):.3f}')
        kv('  -> k_drive', f'{C.mass/tau:.1f} N/(m/s)', f'(현재 모델값 {C.k_drive:.1f})')
        kv('  선형구간 폭 f_drive_max/k', f'{C.f_drive_max/(C.mass/tau):.2f} m/s',
           '계단 크기가 이보다 작아야 유효')
        if len(taus) < 3:
            print('  ※ 유효 계단이 3개 미만이다. --n 을 늘려 반복 측정할 것.')
    else:
        print('  유효한 계단이 없다 — --dv 를 줄이거나 --t-step 을 늘릴 것')


# ------------------------------------------------------------------ c_roll
def an_coast(tr, C):
    hdr('2.6 c_roll   (타력주행 감쇠)')
    segs = pick(tr, ['coast'])
    if not segs:
        print('  coast phase 가 없다'); return
    _, i0, i1 = segs[0]
    m = np.zeros_like(tr.t, bool); m[i0 + 20:i1 - 5] = True     # 전류 전환 과도 제외
    m &= tr.v > 0.8
    if m.sum() < 20:
        print('  표본 부족'); return
    v, a = tr.v[m], tr.a_long[m]
    kv('속도 범위', f'{v.min():.2f}~{v.max():.2f} m/s', f'표본 {int(m.sum())}')
    kv('평균 감속', f'{np.mean(a):.3f} m/s^2')
    c0 = -float(np.mean(a)) / G
    kv('-> c_roll (공기저항 무시)', f'{c0:.4f}', f'(현재 추정값 {C.c_roll})')
    if v.max() - v.min() > 2.5:
        A = np.vstack([np.ones_like(v), v ** 2]).T
        coef, *_ = np.linalg.lstsq(A, -a, rcond=None)
        kv('   구름저항/공기저항 분리(참고)', f'c_roll={coef[0]/G:.4f}',
           f'k_air={coef[1]*C.mass:.4f} N/(m/s)^2')
    else:
        print('     (속도폭이 2.5 m/s 미만이라 구름/공기 분리는 하지 않는다 —'
              ' 이 속도대에서 공기저항은 어차피 무시할 수준이다)')
    if np.isfinite(tr.current).any():
        ci = float(np.nanmedian(np.abs(tr.current[m])))
        if ci > 2.0:
            print(f'  ★ 타력주행 중 모터전류가 {ci:.1f} A 다. 모터가 안 놓였다 —'
                  ' /commands/motor/current 0 이 안 먹은 것이니 결과를 믿지 말 것.')


# ------------------------------------------------------------------ 조향 응답
def an_steer(tr, C):
    hdr('2.7 조향 액추에이터 + 차량 요응답  (계단 응답 t63)')
    segs = pick(tr, ['L', 'R'])
    if len(segs) < 2:
        print('  steer phase 가 없다'); return
    v_run = float(np.median(tr.v[segs[0][1]:segs[-1][2]]))
    if v_run < 0.5:
        print(f'  주행속도가 {v_run:.2f} m/s 다 — 차를 든 무부하 상태면 요레이트가 안 나온다.')
        print('  이 모드에서 t63 을 얻으려면 --v 를 2~3 m/s 로 주고 부하 상태에서 돌릴 것.')
        print('  (무부하 서보 응답은 240fps 촬영이 정확하다. 이미 43.8 ms 로 실측돼 있다.)')
        return
    t63s, ovs = [], []
    for l, i0, i1 in segs[1:]:
        t = tr.t[i0:i1] - tr.t[i0]
        w = tr.wz[i0:i1]
        if len(t) < 10 or not np.isfinite(w).all():
            continue
        w0 = float(np.mean(tr.wz[max(0, i0 - 5):i0]))
        wss = float(np.median(w[int(len(w) * 0.6):]))
        dw = wss - w0
        if abs(dw) < 0.3:
            continue
        target = w0 + 0.632 * dw
        cross = np.where((w - target) * np.sign(dw) >= 0)[0]
        if len(cross) == 0:
            continue
        t63s.append(float(t[cross[0]]))
        # 오버슛은 '정착값을 얼마나 넘어갔나'다. 기준선(w0)이 0 이 아닌 계단
        # (--bias 로 한 방향 선회 중에 주는 계단)에서도 맞도록 정착값 기준으로 잰다.
        over = float(np.max((w - wss) * np.sign(dw)))
        ovs.append(max(over, 0.0) / abs(dw) * 100.0)
    if not t63s:
        print('  유효한 계단이 없다'); return
    kv('주행속도', f'{v_run:.2f} m/s', f'계단 {len(t63s)}개')
    kv('요레이트 t63 (중앙값)', f'{1000*np.median(t63s):.0f} ms',
       f'개별 {[round(1000*x) for x in t63s]}')
    kv('오버슛', f'{np.median(ovs):.0f} %')
    print('\n  주의: 이건 서보 단독이 아니라 [서보 + 타이어 완화 + 요관성] 합이다.')
    print('  시뮬(car_cfg.py) 의 조향 PD 는 tau = Kd/Kp = 8/80 = 100 ms 인데,')
    print('  실차 서보 무부하는 43.8 ms 였다. 위 t63 이 시뮬 tau 보다 작으면')
    print('  damping 8.0 -> 2.0 (tau 25 ms) 방향으로 낮추고, 그 다음 k_smooth_steer 를')
    print('  다시 잡아야 한다(조향이 빨라지면 PWM 뱅뱅 조향이 쉬워진다).')


# ------------------------------------------------------------------ alpha_char
def an_alpha(tr, C):
    hdr('2.2 alpha_char_f / alpha_char_r   (정상원 계단 dwell, 뒤축 기준 동시추정)')
    if tr.no_pose:
        print('  측위 없이는 사이드슬립을 못 낸다. 측위 켜고 다시 딸 것.')
        return
    segs = pick(tr, ['hold'])
    if not segs:
        print('  hold 계단이 없다. --mode circle --v-steps "..." 로 딴 런이 필요하다.')
        return
    if 'ndt' not in tr.name.lower():
        print('  ★경고: pose 소스가 NDT 가 아닌 것 같다. ekf_localizer 는 상태에 VY 가 없어')
        print('    사이드슬립을 1.5~2.0배 과소평가한다. bag 을 --pose-topic /ndt_pose 로 읽을 것.')

    mu = C.mu
    if mu is None:
        v_all = tr.v_erpm if np.isfinite(tr.v_erpm).mean() > 0.5 else tr.v
        mu = float(np.nanpercentile(np.abs(v_all * tr.wz), 99) / G)
        print(f'  --mu 미지정 -> 이 런의 p99 횡가속으로 추정: mu = {mu:.3f}')
    kv('사용 mu', f'{mu:.3f}', 'mu*g = %.2f m/s^2' % (mu * G))
    vsig = tr.v_erpm if np.isfinite(tr.v_erpm).mean() > 0.5 else tr.v

    # ★pose 기준점은 뒤축이다 (2026-08-21 실증: 저속 beta x R = -0.045~-0.001 m).
    #     alpha_r = beta - beta0,   alpha_f = beta - beta0 + L/R - delta
    #   beta0(센서/기하 편향)와 alpha_char 를 따로 구하려 하면 퇴화한다 —
    #   영점 계단에도 실제 슬립이 있어서, 그걸 빼면 alpha_char 가 0 으로 끌려간다.
    #   tanh 모델에서 alpha = ac * atanh(T) 이므로 아래 선형회귀로 **동시에** 푼다:
    #       beta                     = beta0 + ac_r * atanh(T)
    #       beta + L/R - delta       = beta0 + ac_f * atanh(T)
    #   미지수 3개(beta0, ac_r, ac_f), 계단마다 식 2개. 최소자승으로 한 번에.
    # 중단된 런에서는 마지막 계단이 잘려 있다. 정착 전 구간을 섞으면 그 점만
    # 엉뚱한 값을 내므로, 계단 길이가 표준의 80% 미만이면 버린다.
    lens = [j - i for _, i, j in segs]
    full = np.median(lens)
    kept = [(l, i, j) for (l, i, j), n_ in zip(segs, lens) if n_ >= 0.8 * full]
    if len(kept) < len(segs):
        drop = [l for (l, i, j), n_ in zip(segs, lens) if n_ < 0.8 * full]
        print(f'  ※ 미완성 계단 {len(segs)-len(kept)}개 제외: {drop}'
              f' (런이 중단됐다 — 나머지 {len(kept)}개는 유효하다)')
    segs = kept

    pts = []
    for l, i0_, i1_ in segs:
        sl = slice(i0_ + int((i1_ - i0_) * 0.3), i1_)
        v = float(np.median(vsig[sl])); wz = float(np.median(tr.wz[sl]))
        if v < 0.5 or abs(wz) < 0.1:
            continue
        dl = float(np.median(tr.delta[sl])); be = float(np.median(tr.beta[sl]))
        a_lat = v * wz; T = a_lat / (mu * G)
        if abs(T) >= 0.995:
            print(f'  {l:>7s}: 포화(|T|={abs(T):.3f}) — 제외. mu 를 올려야 할 수도 있다')
            continue
        pts.append((l, v, wz, dl, be, a_lat, T, math.atanh(T), be + C.L / (v / wz) - dl))
    if len(pts) < 3:
        print('  유효 계단 3개 미만 — 추정 불가'); return

    u = np.array([p[7] for p in pts])
    br = np.array([p[4] for p in pts])          # = beta            (뒤축식 좌변)
    bf = np.array([p[8] for p in pts])          # = beta + L/R - d  (앞축식 좌변)
    # 절편을 앞/뒤 따로 둔다. 하나로 묶으면 조향 캘리브 오차가 ac_f 를 오염시킨다.
    #   뒤: beta               = beta0_r + ac_r * atanh(T)
    #   앞: beta + L/R - delta = beta0_f + ac_f * atanh(T)
    # beta0_r 은 pose 편향, beta0_f - beta0_r 은 **조향각 오차**를 그대로 담는다
    # (2026-08-21 06시 실측: 명령 delta 가 클수록 실효각이 더 커지는 아커만 progressive).
    n = len(u)
    A = np.zeros((2 * n, 4)); y = np.zeros(2 * n)
    A[:n, 0] = 1.0; A[:n, 2] = u;  y[:n] = br
    A[n:, 1] = 1.0; A[n:, 3] = u;  y[n:] = bf
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    beta0r, beta0f, acr, acf = coef
    beta0 = beta0r
    resid = y - A @ coef
    rms = float(np.sqrt((resid @ resid) / len(resid)))

    print()
    print('  계단     v      wz    delta    beta   atanh(T)   a_lat    T     alpha_r  alpha_f')
    for i, p in enumerate(pts):
        l, v, wz, dl, be, a_lat, T, ut, bfi = p
        print(f'  {l:>7s} {v:5.2f} {wz:+6.3f} {dl:+7.3f} {be:+7.4f} {ut:+8.3f}'
              f' {a_lat:+7.2f} {T:+6.3f} {be-beta0r:+8.4f} {bfi-beta0f:+8.4f}')
    print()
    kv('beta0_r (pose 편향)', f'{beta0r:+.4f} rad', '0 에 가까울수록 좋다')
    kv('beta0_f - beta0_r', f'{beta0f-beta0r:+.4f} rad',
       f'= 조향각 오차. 명령 {abs(np.median([p[3] for p in pts])):.2f} 대비 '
       f'{100*(beta0f-beta0r)/max(abs(np.median([p[3] for p in pts])),1e-6):+.1f}%')
    kv('alpha_char_f', f'{abs(acf):.3f} rad', f'{math.degrees(abs(acf)):.1f} deg')
    kv('alpha_char_r', f'{abs(acr):.3f} rad', f'{math.degrees(abs(acr)):.1f} deg')
    kv('차이', f'{abs(acf)-abs(acr):.3f} rad', '언더스티어의 원인')
    kv('잔차 RMS', f'{rms*1000:.1f} mrad', 'NDT beta 잡음이 ~10 mrad 이므로 그 수준이면 양호')

    # ★tanh 적합성: 잔차가 atanh(T) 와 상관되면 형상이 틀린 것이다
    rr = resid[:n]; rf = resid[n:]
    # 판정은 '상관'이 아니라 '체계오차의 크기'로 한다. 상관계수만 보면 과민하다 —
    # 순수 tanh 시뮬에서도 마찰서클/구름저항/저속테이퍼 때문에 상관 0.9 가 나오지만
    # 진폭은 3 mrad 로 NDT 잡음(~10 mrad)에 묻힌다.
    NOISE = 10.0   # mrad, NDT beta 측정 잡음 수준
    print(f'\n  ★tanh 적합성 — 체계오차 진폭을 NDT 잡음({NOISE:.0f} mrad)과 비교한다')
    for nm, r_ in (('뒤 alpha_r', rr), ('앞 alpha_f', rf)):
        if n < 3 or np.std(u) < 1e-6 or np.std(r_) < 1e-9:
            continue
        c = float(np.corrcoef(u, r_)[0, 1])
        # 잔차 중 atanh(T) 로 설명되는 성분의 진폭 = |상관| x 잔차RMS x (범위/표준편차)
        amp = abs(c) * np.std(r_) * (u.max() - u.min()) / max(np.std(u), 1e-9) * 1000
        if abs(c) < 0.7:
            v_ = '양호 (추세 없음)'
        elif amp < 2 * NOISE:
            v_ = f'추세는 있으나 진폭 {amp:.0f} mrad 로 잡음 수준 — tanh 로 충분'
        else:
            v_ = f'★체계오차 {amp:.0f} mrad — tanh 형상 부적합, Pacejka 검토'
        print(f'    {nm}: 상관 {c:+.2f}, 잔차RMS {np.std(r_)*1000:.1f} mrad, 추세진폭 {amp:.0f} mrad')
        print(f'      -> {v_}')
    print('\n  ※ 좌/우 런을 따로 돌려 beta0 이 부호를 안 바꾸는지 확인할 것.')
    print('    바꾸면 그건 편향이 아니라 좌우 조향 비대칭(실측 5~6%)이 섞인 것이다.')


# ------------------------------------------------------------------ pose 신뢰도
def an_posecheck(tr, C):
    hdr('pose 신뢰도 교차검증  (EKF pose vs ERPM 적분거리)')
    if tr.no_pose or not np.isfinite(tr.v_erpm).any():
        print('  pose 나 ERPM 중 하나가 없어 교차검증 불가')
        return None
    # 정속 + 직진 구간에서만. 가감속 중에는 휠슬립이 섞이고, 선회 중에는
    # 구동륜 경로와 CoM 경로의 반경이 달라 정당한 비교가 안 된다.
    m = (tr.v > 1.0) & (np.abs(tr.a_long) < 1.0) & (np.abs(tr.wz) < 0.15)
    if m.sum() < 100:
        print('  정속 직진 구간 표본 부족 (const 모드 런이 필요하다)')
        return None
    d_pose = float(np.trapz(tr.v[m], tr.t[m]))
    d_erpm = float(np.trapz(tr.v_erpm[m], tr.t[m]))
    ratio = d_pose / d_erpm
    kv('정속직진 구간', f'{m.sum()*tr.dt:.1f} s')
    kv('pose 적분거리', f'{d_pose:.2f} m')
    kv('ERPM 적분거리', f'{d_erpm:.2f} m', f'(게인 {C.erpm_gain:.0f})')
    kv('pose / ERPM', f'{ratio:.4f}', f'{100*(ratio-1):+.1f} %')
    if abs(ratio - 1) > 0.02:
        print('  ★ 2% 넘게 어긋난다. 원인이 둘 중 어느 쪽인지는 bag 의 /ndt_pose 로 갈린다:')
        print('     /ndt_pose 와 ERPM 이 일치하면  -> EKF(=/car_state/pose)가 틀린 것이다.')
        print('        (2026-08-21 실측: NDT 9.35 m == 휠 9.35 m, EKF 9.64 m = +3.1%)')
        print('     /ndt_pose 와 EKF 가 일치하면   -> speed_to_erpm_gain 이 틀린 것이다.')
        print('     아래 mu / 가속 계산은 pose 대신 ERPM 을 쓰는 쪽이 안전하다 (--prefer-erpm).')
    return ratio


# ------------------------------------------------------------------ 고속 처짐
def an_duty(tr, C):
    hdr('역기전력/전압 한계 점검   (전용 직선 없이, 아무 주행 로그에서나)')
    if not np.isfinite(tr.duty).any():
        print('  /sensors/core 가 없다. record_sysid.sh 로 딴 로그가 필요하다.')
        return
    v = tr.v_erpm if np.isfinite(tr.v_erpm).mean() > 0.5 else tr.v
    d = np.abs(tr.duty)
    m = (v > 0.5) & np.isfinite(d) & (d > 0.02)
    if m.sum() < 100:
        print('  표본 부족'); return

    print('  속도구간별 duty / 전류  (duty 는 전압 여유의 지표: 1.0 이면 더 못 밟는다)')
    edges = np.arange(0, np.ceil(v[m].max()) + 1, 1.0)
    for k in range(len(edges) - 1):
        b = m & (v >= edges[k]) & (v < edges[k + 1])
        if b.sum() < 20:
            continue
        print(f'    {edges[k]:.0f}~{edges[k+1]:.0f} m/s : duty p50={np.median(d[b]):4.2f}'
              f' p95={np.percentile(d[b],95):4.2f}   I p50={np.nanmedian(tr.current[b]):6.1f} A'
              f'   (n={int(b.sum())})')

    # duty 는 속도에 거의 비례한다(역기전력). duty=1 로 외삽하면 이 배터리에서의 최고속이 나온다.
    mm = m & (d > 0.15)
    if mm.sum() > 100:
        c = np.polyfit(v[mm], d[mm], 1)
        if c[0] > 1e-3:
            v_top = (1.0 - c[1]) / c[0]
            kv('duty-속도 기울기', f'{c[0]:.4f} /(m/s)', f'절편 {c[1]:+.3f}')
            kv('duty=1.0 외삽 최고속', f'{v_top:.2f} m/s',
               '이 위로는 명령해도 안 나간다')
            if np.isfinite(tr.vbat).any():
                kv('배터리 전압', f'{np.nanmedian(tr.vbat[mm]):.2f} V',
                   f'최저 {np.nanmin(tr.vbat[mm]):.2f} V')
            print(f'\n  -> 학습 v_max 가 {v_top:.1f} m/s 를 넘으면 그 구간은 시뮬에만 존재한다.')
            print('     (실차는 전압 한계로 그 속도를 못 내는데 정책은 낼 수 있다고 배운다)')
    print('\n  ※ 이 점검은 PP 랩 로그로도 된다 — 전용 직선이 필요 없다.')


# ------------------------------------------------------------------ 플롯
def plot(tr, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    ax[0].plot(tr.t, tr.v, label='v (pose 미분)')
    ax[0].plot(tr.t, tr.v_cmd, '--', label='v_cmd')
    if np.isfinite(tr.v_erpm).any():
        ax[0].plot(tr.t, tr.v_erpm, ':', label='v (ERPM 환산)')
    ax[0].set_ylabel('speed [m/s]'); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
    ax[1].plot(tr.t, tr.a_long, label='a_long')
    ax[1].plot(tr.t, tr.v * tr.wz, label='a_lat = v*wz')
    ax[1].set_ylabel('accel [m/s^2]'); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
    ax[2].plot(tr.t, tr.delta, label='delta_cmd')
    ax[2].plot(tr.t, tr.wz, label='wz (IMU)')
    ax[2].plot(tr.t, tr.beta, label='beta')
    ax[2].set_ylabel('rad, rad/s'); ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)
    ax[3].plot(tr.t, tr.duty, label='duty')
    ax[3].plot(tr.t, tr.current / 50.0, label='I/50 [A]')
    ax[3].set_ylabel('duty / I'); ax[3].set_xlabel('t [s]')
    ax[3].legend(fontsize=8); ax[3].grid(alpha=.3)
    fig.suptitle(tr.name)
    fig.tight_layout(); fig.savefig(path, dpi=110)
    print(f'\n그림 저장: {path}')


# ------------------------------------------------------------------ main
def main():
    p = argparse.ArgumentParser(description='sysid 런에서 물리계수 역산',
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__)
    p.add_argument('path', help='sysid CSV 또는 bag 디렉터리')
    p.add_argument('--mode', default='auto',
                   choices=['auto', 'gain', 'circle', 'accel', 'step', 'coast', 'steer',
                            'duty', 'posecheck', 'alpha'])
    p.add_argument('--mass', type=float, default=4.987, help='실측 코너웨이트 합 [kg]')
    p.add_argument('--lf', type=float, default=0.171, help='CoM~앞축 [m]')
    p.add_argument('--lr', type=float, default=0.159, help='CoM~뒤축 [m]')
    p.add_argument('--mu', type=float, default=None, help='2.1 에서 확정한 mu (있으면 더 좁힌다)')
    p.add_argument('--c-roll', dest='c_roll', type=float, default=0.015)
    p.add_argument('--k-drive', dest='k_drive', type=float, default=40.0)
    p.add_argument('--f-drive-max', dest='f_drive_max', type=float, default=31.1)
    p.add_argument('--erpm-gain', dest='erpm_gain', type=float, default=3576.0)
    p.add_argument('--slip-thresh', dest='slip_thresh', type=float, default=0.25)
    p.add_argument('--vmin', type=float, default=1.0)
    p.add_argument('--win', type=float, default=0.25, help='pose 미분 평활 창 [s]')
    p.add_argument('--no-pose', dest='no_pose', action='store_true',
                   help='측위 없이 딴 로그. 속도를 ERPM 환산, 방위를 자이로 적분으로 만든다. '
                        'k_drive / c_roll / 조향 t63 만 유효하고 alpha_char 는 못 낸다')
    p.add_argument('--prefer-erpm', dest='prefer_erpm', action='store_true',
                   help='속도를 pose 대신 ERPM 환산으로 쓴다. pose 신뢰도 교차검증에서 '
                        '2% 넘게 어긋났을 때 사용')
    p.add_argument('--radius', type=float, default=None,
                   help='콘 원의 반경 [m]. circle 분석에서 a_lat = wz^2*R 로 계산해 '
                        '속도 계측을 아예 건너뛴다 (측위도 ERPM 게인도 무관)')
    p.add_argument('--pose-topic', dest='pose_topic', default=None,
                   help='bag 에서 읽을 pose 토픽. alpha 측정은 반드시 /ndt_pose '
                        '(ekf_localizer 는 상태에 VY 가 없어 사이드슬립을 1.5~2배 과소평가)')
    p.add_argument('--plot', default=None)
    C = p.parse_args()
    C.L = C.lf + C.lr

    kw = {}
    if C.pose_topic:
        if not os.path.isdir(C.path):
            print('※ --pose-topic 은 bag 에만 적용된다 (CSV 는 /car_state/pose 만 담는다)')
        else:
            kw['pose_topic'] = C.pose_topic
    tr = S.load(C.path, erpm_gain=C.erpm_gain, win_s=C.win, no_pose=C.no_pose, **kw)
    # bag 에는 phase 라벨이 없다. 옆에 cmd.csv 가 있으면 시각으로 이어 붙인다.
    if os.path.isdir(C.path):
        cand = [os.path.join(os.path.dirname(C.path.rstrip('/')), 'cmd.csv'),
                os.path.join(C.path, '..', 'cmd.csv')]
        for c in cand:
            if os.path.exists(c) and S.attach_phase_from_csv(tr, c):
                print(f'phase 라벨을 {os.path.relpath(c)} 에서 가져왔다')
                break
    if tr.no_pose:
        print('※ 측위 없는 경로다. 속도=ERPM 환산, 횡속도(beta)=0 가정.\n'
              '   유효: k_drive / c_roll / 조향 t63 / (--radius 를 줬다면) mu\n'
              '   무효: alpha_char, 휠슬립 판정, f_drive_max 힘/마찰 구분, ERPM 게인 검증')
    print(f'입력: {tr.name}   샘플 {len(tr.t)}개 / {tr.t[-1]-tr.t[0]:.1f} s'
          f'   v {tr.v.min():.2f}~{tr.v.max():.2f} m/s')
    if C.mu:
        print(f'주어진 mu = {C.mu}')

    phases = set(tr.phase)
    todo = []
    if C.mode != 'auto':
        todo = [C.mode]
    else:
        is_steer = any(p.startswith(('L', 'R')) for p in phases)
        if not is_steer and (any(p.startswith(('ramp', 'entry')) for p in phases)
                             or (np.abs(tr.delta) > 0.05).mean() > 0.3):
            todo.append('circle')
        if any(p.startswith('launch') for p in phases):
            todo.append('accel')
        if any(p.startswith(('up', 'dn')) for p in phases):
            todo.append('step')
        if any(p.startswith('hold') and p != 'hold' for p in phases):
            todo.append('alpha')
        if any(p.startswith('coast') for p in phases):
            todo.append('coast')
        if any(p.startswith(('L', 'R')) for p in phases):
            todo.append('steer')
        todo.append('gain')
        todo.append('posecheck')      # 항상 — pose 를 믿어도 되는지가 나머지 전부의 전제다
        if not todo:
            todo = ['gain']

    fn = dict(gain=an_gain, circle=an_circle, accel=an_accel,
              step=an_step, coast=an_coast, steer=an_steer, duty=an_duty,
              posecheck=an_posecheck, alpha=an_alpha)
    for k in todo:
        try:
            fn[k](tr, C)
        except Exception as e:
            print(f'\n[{k}] 분석 실패: {type(e).__name__}: {e}')

    if C.plot:
        plot(tr, C.plot)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, FileNotFoundError) as e:      # 사용자 입력 문제는 역추적 없이
        print(f'\n[에러] {e}')
        sys.exit(1)
