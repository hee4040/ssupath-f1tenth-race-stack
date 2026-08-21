#!/usr/bin/env python3
"""
주행 로그에서 노면 마찰계수 mu 의 '하한'을 추정한다.

원리
    차가 실제로 낸 횡가속은 반드시 마찰이 허용한 범위 안이다. 따라서
        mu >= p99(|a_lat|) / g,     a_lat = v * wz
    전용 실험 공간도, 전용 주행도 필요 없다. Pure Pursuit 로 어차피 도는
    랩 로그를 그대로 넣으면 된다. 코너에서 한계까지 안 갔다면 그만큼 느슨한
    하한이 나올 뿐, 틀린 값이 나오지는 않는다.

    속도는 pose 미분으로 낸다. /odom 이나 /car_state/odom 의 twist 를 쓰면
    그건 ERPM / speed_to_erpm_gain 이라 게인 오차가 그대로 들어간다.

사용법
    # bag (녹화해 둔 것)
    python3 measure_grip.py ~/forza_ws/race_stack/loc_debug_0820_1530
    # sysid_cmd.py 가 남긴 CSV
    python3 measure_grip.py ~/forza_ws/race_stack/sysid_circle_0820_1530.csv
    # 주행 중 실시간 (Ctrl-C 로 끝내면 요약이 나온다)
    python3 measure_grip.py --live

    --vmin 1.5      이 속도 미만 구간은 버린다 (저속은 pose 미분 잡음이 크다)
    --combined      횡+종 합성 가속(마찰서클 반경)으로도 같이 낸다
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sysid_io as S

G = S.G


def report(v, wz, a_long, vmin, combined, label):
    m = (v >= vmin) & np.isfinite(wz) & np.isfinite(v)
    if m.sum() < 50:
        print(f'  표본이 부족하다 (v>={vmin} 인 샘플 {int(m.sum())}개)')
        return None
    a_lat = np.abs(v[m] * wz[m])
    print(f'\n=== {label} ===')
    print(f'  표본 {int(m.sum())}개 / 속도 {v[m].min():.2f}~{v[m].max():.2f} m/s')
    print('  |a_lat| [m/s^2]   p50=%.2f  p90=%.2f  p99=%.2f  max=%.2f'
          % (np.percentile(a_lat, 50), np.percentile(a_lat, 90),
             np.percentile(a_lat, 99), a_lat.max()))
    mu_lat = np.percentile(a_lat, 99) / G
    print(f'  -> 횡가속 기준  mu >= {mu_lat:.3f}   (p99 기준. max 기준이면 {a_lat.max()/G:.3f})')

    mu_out = mu_lat
    if combined and a_long is not None:
        a_tot = np.hypot(a_lat, np.abs(a_long[m]))
        mu_c = np.percentile(a_tot, 99) / G
        print('  |a_total|         p50=%.2f  p90=%.2f  p99=%.2f'
              % (np.percentile(a_tot, 50), np.percentile(a_tot, 90), np.percentile(a_tot, 99)))
        print(f'  -> 마찰서클 기준 mu >= {mu_c:.3f}')
        mu_out = max(mu_out, mu_c)

    if a_long is not None:
        al = a_long[m]
        print('  종가속 [m/s^2]     최대가속=%.2f  최대감속=%.2f'
              % (np.percentile(al, 99), np.percentile(al, 1)))
        # 힘 상한(31.1N / 4.987kg = 6.24 m/s^2) 근처면 종방향으로는 mu 를 못 캔다
        if np.percentile(al, 99) > 5.8:
            print('  ※ 가속이 힘 상한(6.24 m/s^2) 근처다 — 이 구간에서 종방향은 mu 에 반응하지'
                  ' 않는다. 종가속으로 mu 를 식별하려 하지 말 것.')
    return mu_out


def suggest(mu):
    """학습에 넣을 mu_range 제안. 정책은 mu 를 관측하지 못하므로 밴드는 좁게 —
    넓으면 하한에 맞춰 수렴한다(0819 보고서 PART 3)."""
    lo, hi = mu * 0.95, mu * 1.20
    print('\n--- 학습 mu_range 제안 ---')
    print(f'  실측 하한 mu >= {mu:.3f} (실제는 이보다 높다: 한계까지 안 갔을 수 있다)')
    print(f'  좁은 밴드 권장:  --mu_range {lo:.2f} {hi:.2f}   (중앙 {(lo+hi)/2:.3f})')
    print('  ★ 학습 mu < 실제 : 정책이 과속 -> throttle_interpolator 로 배포 시 차단 가능')
    print('    학습 mu > 실제 : 정책이 언더스티어 -> 배포 시 차단 수단 없음')
    print('    => 애매하면 낮은 쪽 밴드를 고르고 스로틀 제한기를 켠다.')


def run_live_gyro(args):
    """자이로만으로 mu. 콘 원(반경 R)을 조이스틱으로 돌면서 쓴다.
    v = wz*R 이므로 a_lat = v*wz = wz^2*R — 속도를 아예 재지 않는다.
    측위도, speed_to_erpm_gain 도 필요 없다. bringup_3D_launch.py 만 띄워도 된다."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    class GripGyro(Node):
        def __init__(self):
            super().__init__('measure_grip_gyro')
            self.wz = []
            self.best = 0.0
            self.create_subscription(Imu, '/sensors/imu/raw', self.on_imu, qos_profile_sensor_data)
            self.create_timer(0.25, self.tick)
            self.get_logger().info(
                f'반경 {args.radius} m 원을 돌면서 속도를 조금씩 올릴 것. '
                '뒤가 흐르기 시작하면 그 직전이 mu 다. Ctrl-C 로 요약.')

        def on_imu(self, m):
            self.wz.append(m.angular_velocity.z)

        def tick(self):
            if len(self.wz) < 20:
                return
            # 최근 0.5초 중앙값 — 스파이크 하나에 최대치가 오염되지 않게
            w = float(np.median(np.abs(self.wz[-25:])))
            a = w * w * args.radius
            self.best = max(self.best, a / G)
            print('\r현재 wz=%+6.3f rad/s  v(=wz*R)=%4.2f m/s  a_lat=%5.2f m/s^2  '
                  'mu=%4.2f   [최대 %4.2f]      ' % (w, w * args.radius, a, a / G, self.best),
                  end='', flush=True)

    rclpy.init()
    n = GripGyro()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if n.best > 0:
            print(f'\n=== 콘 원 기하 기준 (R={args.radius} m, 자이로만) ===')
            print(f'  최대 |a_lat| = {n.best*G:.2f} m/s^2  ->  mu >= {n.best:.3f}')
            print('  ※ 차가 실제로 그 반경을 돌았을 때만 맞다. mu 는 R 에 비례하므로')
            print('    R 을 10% 크게 잡으면 mu 도 10% 크게 나온다. 줄자로 잴 것.')
            suggest(n.best)
        else:
            print('표본이 너무 적다.')
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_live(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Imu

    class Grip(Node):
        def __init__(self):
            super().__init__('measure_grip')
            self.buf_t, self.buf_x, self.buf_y = [], [], []
            self.wz_t, self.wz_v = [], []
            self.create_subscription(PoseStamped, args.pose_topic, self.on_pose, 10)
            self.create_subscription(Imu, '/sensors/imu/raw', self.on_imu, qos_profile_sensor_data)
            self.create_timer(1.0, self.tick)
            self.get_logger().info(f'{args.pose_topic} + /sensors/imu/raw 수집 중. Ctrl-C 로 요약.')

        def on_pose(self, m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.buf_t.append(t); self.buf_x.append(m.pose.position.x); self.buf_y.append(m.pose.position.y)

        def on_imu(self, m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.wz_t.append(t); self.wz_v.append(m.angular_velocity.z)

        def tick(self):
            if len(self.buf_t) < 200:
                return
            v, wz, _ = self.arrays()
            m = v >= args.vmin
            if m.sum() < 50:
                return
            a = np.abs(v[m] * wz[m])
            print('\r수집 %.0fs  v_max=%.2f  |a_lat| p99=%.2f -> mu>=%.3f      '
                  % (self.buf_t[-1] - self.buf_t[0], v.max(),
                     np.percentile(a, 99), np.percentile(a, 99) / G), end='', flush=True)

        def arrays(self):
            tr = S.Track('live').build(np.array(self.buf_t) - self.buf_t[0],
                                       self.buf_x, self.buf_y,
                                       np.zeros(len(self.buf_t)))
            wz = np.interp(tr.t, np.array(self.wz_t) - self.buf_t[0], self.wz_v) \
                if len(self.wz_t) > 2 else tr.yawrate_pose
            return tr.v, wz, tr.a_long

    rclpy.init()
    n = Grip()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if len(n.buf_t) > 200:
            v, wz, al = n.arrays()
            mu = report(v, wz, al, args.vmin, args.combined, '실시간 수집')
            if mu:
                suggest(mu)
        else:
            print('표본이 너무 적다.')
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description='주행 로그에서 mu 하한 추정',
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__)
    p.add_argument('path', nargs='?', help='bag 디렉터리 또는 sysid CSV')
    p.add_argument('--live', action='store_true', help='주행 중 실시간 구독')
    p.add_argument('--vmin', type=float, default=1.5)
    p.add_argument('--combined', action='store_true', help='횡+종 합성 가속도 함께')
    p.add_argument('--pose-topic', dest='pose_topic', default='/car_state/pose')
    p.add_argument('--radius', type=float, default=None,
                   help='콘으로 표시한 원의 반경 [m]. 주면 자이로만으로 mu 를 낸다 '
                        '(a_lat = wz^2*R). 측위도 ERPM 게인도 필요 없다. '
                        '단 차가 실제로 그 원을 따라가고 있어야 한다')
    p.add_argument('--erpm-gain', dest='erpm_gain', type=float, default=3576.0)
    a = p.parse_args()

    if a.live:
        return run_live_gyro(a) if a.radius else run_live(a)
    if not a.path:
        p.error('bag/CSV 경로를 주거나 --live 를 쓸 것')

    tr = S.load(a.path, erpm_gain=a.erpm_gain,
                **({'pose_topic': a.pose_topic} if os.path.isdir(a.path) else {}))
    wz = tr.wz if np.isfinite(tr.wz).mean() > 0.5 else tr.yawrate_pose
    if not np.isfinite(tr.wz).mean() > 0.5:
        print('※ /sensors/imu/raw 가 없어 pose 미분 요레이트를 쓴다 (잡음이 더 크다)')
    mu = report(tr.v, wz, tr.a_long, a.vmin, a.combined, tr.name)
    if mu:
        suggest(mu)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, FileNotFoundError) as e:      # 사용자 입력 문제는 역추적 없이
        print(f'\n[에러] {e}')
        sys.exit(1)
