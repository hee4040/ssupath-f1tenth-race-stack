#!/usr/bin/env python3
"""
sysid CSV / rosbag2 를 공통 표로 읽는 모듈. analyze_sysid.py 와 measure_grip.py 가 함께 쓴다.

핵심 원칙 (0819 보고서 2.3):
  속도는 절대 휠 오도메트리(/odom, /car_state/odom 의 twist)로 재지 않는다.
  그건 결국 ERPM / speed_to_erpm_gain 이라, 게인이 의심스러운 지금은 순환 논리다.
  pose(x, y)를 미분해서 쓴다. ERPM 속도는 '비교 대상'으로만 싣는다.
"""

import math
import os

import numpy as np

try:
    from scipy.signal import savgol_filter
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False

G = 9.80665

POSE_TOPICS = ['/car_state/pose', '/ekf_pose', '/ndt_pose']


# ---------------------------------------------------------------- 신호 유틸
def _uniform(t, y, dt):
    """비균일 시계열을 균일 격자로. yaw 는 미리 unwrap 해서 넣을 것."""
    tt = np.arange(t[0], t[-1], dt)
    return tt, np.interp(tt, t, y)


def smooth_deriv(t, y, dt=0.01, win_s=0.25, poly=2):
    """균일격자 + Savitzky-Golay 미분. 반환: (격자시간, 평활값, 미분값)"""
    if len(t) < 5:
        return np.array([]), np.array([]), np.array([])
    tt, yy = _uniform(np.asarray(t, float), np.asarray(y, float), dt)
    win = max(5, int(win_s / dt) | 1)
    win = min(win, (len(yy) - 1) | 1)
    if win < 5 or not HAVE_SCIPY:
        d = np.gradient(yy, dt)
        return tt, yy, d
    ys = savgol_filter(yy, win, poly)
    d = savgol_filter(yy, win, poly, deriv=1, delta=dt)
    return tt, ys, d


def _deriv_uniform(y, dt, win_s=0.25, poly=2):
    """이미 균일격자인 신호의 미분. 길이를 그대로 유지한다."""
    win = max(5, int(win_s / dt) | 1)
    win = min(win, (len(y) - 1) | 1)
    if win < 5 or not HAVE_SCIPY:
        return np.gradient(y, dt)
    return savgol_filter(y, win, poly, deriv=1, delta=dt)


class Track:
    """한 번의 런에서 뽑은 신호 묶음. 모든 배열은 self.t(균일 100Hz) 위에 정렬돼 있다."""

    def __init__(self, name):
        self.name = name
        self.warn = []
        self.no_pose = False

    # ---- 원본 pose 로부터 속도/슬립각까지 채운다 ----
    def build(self, pose_t, px, py, pyaw, dt=0.01, win_s=0.25):
        pose_t = np.asarray(pose_t, float)
        ok = np.concatenate([[True], np.diff(pose_t) > 1e-6])
        pose_t, px, py, pyaw = pose_t[ok], np.asarray(px)[ok], np.asarray(py)[ok], np.asarray(pyaw)[ok]
        if len(pose_t) < 10:
            raise ValueError('pose 표본이 10개 미만이다 — 측위가 안 붙은 로그다')

        self.t, x, vx = smooth_deriv(pose_t, px, dt, win_s)
        _, y, vy = smooth_deriv(pose_t, py, dt, win_s)
        _, yaw, yawrate = smooth_deriv(pose_t, np.unwrap(pyaw), dt, win_s)
        self.x, self.y, self.yaw = x, y, yaw
        self.vx_map, self.vy_map = vx, vy
        self.yawrate_pose = yawrate
        self.v = np.hypot(vx, vy)
        # a_long 은 이미 균일격자인 self.v 위에서 바로 미분한다.
        # (다시 smooth_deriv 에 넣으면 격자가 한 칸 짧아져 길이가 어긋난다)
        self.a_long = _deriv_uniform(self.v, dt, win_s)

        c, s = np.cos(yaw), np.sin(yaw)
        self.vx_b = c * vx + s * vy          # 차체 전방 속도
        self.vy_b = -s * vx + c * vy         # 차체 좌측 속도
        # 정지 근처에서는 beta 가 의미가 없다(vx_b -> 0). 아래 분석은 전부 v 로 마스킹한다.
        self.beta = np.arctan2(self.vy_b, np.maximum(self.vx_b, 1e-3))
        self.dt = dt
        self.t0_abs = pose_t[0]
        return self

    def build_deadreckon(self, t, v_wheel, wz, dt=0.01, win_s=0.25):
        """측위가 없을 때. 속도는 ERPM 환산, 방위는 자이로 적분으로 만든다.

        ★한계를 분명히 할 것:
          - v 가 ERPM/게인이라 speed_to_erpm_gain 오차가 그대로 곱해진다.
          - 바퀴가 헛돌면 v 가 지면속도보다 크게 나온다(그리고 그걸 검출할 방법이 없다).
          - 횡속도(beta)를 알 수 없어 0 으로 둔다 -> 슬립각이 무의미해지고
            alpha_char 는 원리적으로 못 낸다.
        그래서 이 경로로 유효한 건 k_drive / c_roll / 조향 t63 / (반경을 알 때의) mu 뿐이다.
        """
        t = np.asarray(t, float)
        self.t, self.v, self.a_long = smooth_deriv(t, np.asarray(v_wheel, float), dt, win_s)
        self.a_long = _deriv_uniform(self.v, dt, win_s)
        self.wz = np.interp(self.t, t, np.asarray(wz, float))
        self.yaw = np.concatenate([[0.0], np.cumsum(self.wz[:-1]) * dt])
        self.yawrate_pose = self.wz.copy()
        self.vx_b, self.vy_b = self.v.copy(), np.zeros_like(self.v)
        self.beta = np.zeros_like(self.v)
        self.vx_map = self.v * np.cos(self.yaw)
        self.vy_map = self.v * np.sin(self.yaw)
        self.x = np.concatenate([[0.0], np.cumsum(self.vx_map[:-1]) * dt])
        self.y = np.concatenate([[0.0], np.cumsum(self.vy_map[:-1]) * dt])
        self.dt = dt
        self.no_pose = True
        self.t0_abs = t[0]
        return self

    def put(self, name, src_t, src_v):
        """다른 시간축의 신호를 self.t 로 보간해 붙인다."""
        src_t = np.asarray(src_t, float); src_v = np.asarray(src_v, float)
        if len(src_t) < 2:
            setattr(self, name, np.full_like(self.t, np.nan))
            return
        setattr(self, name, np.interp(self.t, src_t, src_v))

    def slip_angles(self, lf, lr):
        """자전거모델 슬립각. wz 는 IMU 자이로를 쓴다(pose 미분보다 깨끗)."""
        wz = self.wz if hasattr(self, 'wz') and np.isfinite(self.wz).any() else self.yawrate_pose
        vx = np.maximum(self.vx_b, 1e-3)
        af = self.delta - np.arctan2(self.vy_b + lf * wz, vx)
        ar = -np.arctan2(self.vy_b - lr * wz, vx)
        return af, ar

    def mask(self, phases=None, vmin=None, vmax=None):
        m = np.ones_like(self.t, dtype=bool)
        if phases is not None and hasattr(self, 'phase'):
            want = set(phases)
            m &= np.array([p in want for p in self.phase])
        if vmin is not None:
            m &= self.v >= vmin
        if vmax is not None:
            m &= self.v <= vmax
        return m


# ---------------------------------------------------------------- CSV 입력
def load_csv(path, erpm_gain=3576.0, servo_gain=-0.65, servo_offset=0.5,
              no_pose=False, **kw):
    import csv as _csv
    rows = list(_csv.DictReader(open(path)))
    if not rows:
        raise ValueError(f'{path} 가 비었다')
    f = lambda k: np.array([float(r[k]) for r in rows])
    t = f('t')
    tr = Track(os.path.basename(path))

    pt = f('pose_t')
    have_pose = np.nanmax(pt) > 0
    if no_pose or not have_pose:
        if not no_pose:
            print('※ pose_t 가 전부 0 이다 — 측위 없는 로그로 보고 ERPM+자이로 경로로 간다.'
                  ' (k_drive / c_roll / 조향 t63 만 유효)')
        tr.build_deadreckon(t, f('erpm_meas') / erpm_gain, f('wz'), **kw)
    else:
        good = pt > 0
        # pose_t(ROS 시각) -> CSV t(런 상대시각) 로 옮긴다. 두 축 모두 벽시계라 오프셋만 다르다.
        off = np.median(pt[good] - t[good])
        tr.build(pt[good] - off, f('x')[good], f('y')[good], f('yaw')[good], **kw)

    tr.put('v_cmd', t, f('v_cmd'))
    tr.put('delta', t, f('delta_cmd'))
    tr.put('wz', t, f('wz'))
    tr.put('erpm', t, f('erpm_meas'))
    tr.put('current', t, f('current_motor'))
    tr.put('duty', t, f('duty'))
    tr.put('vbat', t, f('v_batt'))
    tr.v_erpm = tr.erpm / erpm_gain
    idx = np.clip(np.searchsorted(t, tr.t), 0, len(rows) - 1)
    tr.phase = np.array([rows[i]['phase'] for i in idx])
    return tr


# ---------------------------------------------------------------- bag 입력
def load_bag(path, erpm_gain=3576.0, servo_gain=-0.65, servo_offset=0.5,
             pose_topic=None, no_pose=False, **kw):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    want = ['/drive', '/sensors/core', '/sensors/imu/raw',
            '/sensors/servo_position_command', '/commands/motor/speed'] + POSE_TOPICS
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    # 메시지 타입을 못 찾는 토픽은 건너뛴다 (워크스페이스를 source 안 한 노트북에서도
    # pose/드라이브만으로 돌아가게).
    have, cls = [], {}
    for w in want:
        if w not in types:
            continue
        try:
            cls[w] = get_message(types[w])
            have.append(w)
        except Exception:
            print(f'※ {w} ({types[w]}) 타입을 못 읽어 건너뛴다 — install/setup.bash 를 source 했는가?')
    reader.set_filter(rosbag2_py.StorageFilter(topics=have))

    buf = {w: [] for w in have}
    while reader.has_next():
        tn, data, ts = reader.read_next()
        buf[tn].append((ts * 1e-9, deserialize_message(data, cls[tn])))

    # 지정한 토픽이 실제로 bag 에 있고 메시지도 있어야 쓴다. 없으면 다음 후보로.
    cand = list(dict.fromkeys(([pose_topic] if pose_topic else []) + POSE_TOPICS))
    ptopic = next((p for p in cand if buf.get(p)), None)
    if ptopic is None:
        got = sorted(k for k, v in buf.items() if v)
        raise ValueError(
            f'pose 토픽이 bag 에 없다. 찾은 것: {cand}\n'
            f'   bag 에 있는 토픽: {got}\n'
            '   -> 측위(base_system_3D_launch)를 띄운 상태에서 녹화해야 한다. '
            '매핑용 bag 에는 pose 가 없다.')
    if pose_topic and ptopic != pose_topic:
        print(f'※ {pose_topic} 가 bag 에 없어 {ptopic} 를 대신 쓴다')

    P = buf[ptopic]
    def _yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    pt = np.array([p[0] for p in P])
    t0 = pt[0]
    tr = Track(f'{os.path.basename(path)} [{ptopic}]')
    tr.build(pt - t0,
             [p[1].pose.position.x for p in P],
             [p[1].pose.position.y for p in P],
             [_yaw(p[1].pose.orientation) for p in P], **kw)
    # build 에는 상대시각을 넘겼으므로 t0_abs 가 0 이 된다. 절대 기준시각으로 되돌린다
    # (attach_phase_from_csv 가 CSV 와 시각을 맞출 때 이걸 쓴다).
    tr.t0_abs = float(t0)

    def series(topic, fn):
        d = buf.get(topic) or []
        return np.array([x[0] - t0 for x in d]), np.array([fn(x[1]) for x in d])

    tr.put('v_cmd', *series('/drive', lambda m: m.drive.speed))
    tr.put('delta', *series('/drive', lambda m: m.drive.steering_angle))
    tr.put('wz', *series('/sensors/imu/raw', lambda m: m.angular_velocity.z))
    tr.put('erpm', *series('/sensors/core', lambda m: m.state.speed))
    tr.put('current', *series('/sensors/core', lambda m: m.state.current_motor))
    tr.put('duty', *series('/sensors/core', lambda m: m.state.duty_cycle))
    tr.put('vbat', *series('/sensors/core', lambda m: m.state.voltage_input))
    if '/sensors/servo_position_command' in have:
        st, sv = series('/sensors/servo_position_command', lambda m: m.data)
        tr.put('servo', st, sv)
        if not np.isfinite(tr.delta).any():
            tr.delta = (tr.servo - servo_offset) / servo_gain
    tr.v_erpm = tr.erpm / erpm_gain
    tr.phase = np.array(['?'] * len(tr.t))
    return tr


def attach_phase_from_csv(tr, csv_path):
    """bag 으로 읽은 Track 에 CSV 의 phase 라벨을 붙인다.

    왜 필요한가: phase 라벨은 sysid_cmd 가 CSV 에만 남긴다. 그런데 alpha 측정은
    beta 때문에 /ndt_pose 를 써야 하고 그건 bag 에만 있다. 둘을 시각으로 잇는다.
    CSV 의 t 는 런 상대시각, pose_t 는 ROS 절대시각이므로 그 차이가 오프셋이다.
    """
    import csv as _csv
    rows = list(_csv.DictReader(open(csv_path)))
    if not rows:
        return False
    t = np.array([float(r['t']) for r in rows])
    pt = np.array([float(r['pose_t']) for r in rows])
    good = pt > 0
    if good.sum() < 10:
        return False
    off = float(np.median(pt[good] - t[good]))       # CSV 절대시각 = t + off
    t_abs = t + off
    tr_abs = tr.t + tr.t0_abs
    idx = np.clip(np.searchsorted(t_abs, tr_abs), 0, len(rows) - 1)
    tr.phase = np.array([rows[i]['phase'] for i in idx])
    # 명령 신호도 CSV 쪽이 정확하다(bag 의 /drive 는 발행 시점만 남는다)
    tr.put('v_cmd', t_abs - tr.t0_abs, np.array([float(r['v_cmd']) for r in rows]))
    tr.put('delta', t_abs - tr.t0_abs, np.array([float(r['delta_cmd']) for r in rows]))
    return True


def load(path, **kw):
    if os.path.isdir(path):
        return load_bag(path, **kw)
    if path.endswith('.csv'):
        return load_csv(path, **kw)
    raise ValueError(f'CSV 파일이나 bag 디렉터리를 달라: {path}')
