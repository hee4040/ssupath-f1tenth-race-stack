#!/usr/bin/env python3
"""
연습장 물리계수 실측용 오픈루프 명령 노드 (mu / alpha_char / k_drive /
f_drive_max / f_brake_max / c_roll / 조향응답).

왜 ros2 topic pub 대신 노드인가
  - 조이스틱 데드맨을 물고 있다: 버튼을 떼면 그 즉시 정지 명령이 나간다.
    (0819 보고서는 "Ctrl+C 가 유일한 정지 수단"이라고 적었지만 사실이 아니다.
     vesc_driver 자체에 조이스틱 e-stop 이 들어 있고 -- vesc_driver.cpp:408 --
     Circle(idx 2) 을 누르면 speed/current/duty 를 전부 0 으로 막는다.
     base_system 만 띄운 상태에서도 살아 있다. 이 노드의 데드맨은 그 위에
     한 겹 더 얹는 것이다.)
  - 프로파일(계단/램프/원선회)을 재현 가능하게 낸다. 손으로 스틱을 밀면
    같은 입력을 두 번 못 만들고, 그러면 k_drive 같은 건 식별이 안 된다.
  - 명령을 CSV 로 같이 남긴다. bag 만으로도 되지만 현장에서 즉시 확인이 된다.

기본 발행 대상은 /drive (AckermannDriveStamped) 다. ackermann_to_vesc 가
ERPM/서보로 변환한다. --raw 를 주면 /commands/motor/speed 와
/commands/servo/position 에 직접 쏜다(변환 게인을 의심할 때).

★ base_system 만 띄운 상태에서 쓸 것. time_trials 를 같이 띄우면
  mux_controller 가 /drive 를 계속 발행해 명령이 섞인다.
★ 조이스틱 데드맨은 R1(idx 5) 이 기본이다. joy_teleop 의 데드맨 L1(idx 4) 과
  일부러 다르게 잡았다 -- L1 을 잡으면 joy_teleop 이 /drive 를 같이 쏘기 때문.

사용 예
  ./sysid_cmd.py --mode circle --steer 0.25 --v0 1.0 --v1 5.0 --t 12
  ./sysid_cmd.py --mode accel  --v 6.0 --t 2.5
  ./sysid_cmd.py --mode step   --v0 2.0 --dv 0.4 --t-step 1.5 --n 4
  ./sysid_cmd.py --mode coast  --v 5.0 --t-coast 4.0
  ./sysid_cmd.py --mode steer  --amp 0.30 --period 1.0 --n 6 --v 0.0
  ./sysid_cmd.py --mode const  --v 1.0 --hold 5.0        # 줄자 5m / ERPM 게인 확인
  ./sysid_cmd.py --mode accel --v 6 --t 2.5 --dry-run    # 발행 없이 프로파일만 확인
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float64

try:
    from vesc_msgs.msg import VescStateStamped
except ImportError:      # vesc_msgs 가 없으면 텔레메트리만 포기하고 계속 간다
    VescStateStamped = None

G = 9.80665


# ----------------------------------------------------------------------------
# 프로파일: (이름, 지속시간, 종류, v(tau), delta(tau)) 세그먼트의 나열
#   종류 'drive' = /drive 발행, 'coast' = 아무것도 안 쏘고 모터 전류 0 (타력주행),
#        'stop'  = 속도 0 발행(= VESC 0rpm 유지 = 제동)
# ----------------------------------------------------------------------------
def build_profile(a):
    seg = []
    k = lambda c: (lambda tau: c)      # 상수 함수

    if a.mode == 'const':
        seg.append(('spool', a.ramp, 'drive', lambda t: a.v * t / max(a.ramp, 1e-3), k(a.steer)))
        seg.append(('hold', a.hold, 'drive', k(a.v), k(a.steer)))
        seg.append(('brake', a.t_brake, 'stop', k(0.0), k(0.0)))

    elif a.mode == 'accel':
        # 정지 -> v_cmd 계단. 힘 상한이면 여기 기울기가 f_drive_max/m 다.
        seg.append(('launch', a.t, 'drive', k(a.v), k(a.steer)))
        seg.append(('brake', a.t_brake, 'drive', k(a.v_brake), k(0.0)))
        seg.append(('settle', 1.0, 'stop', k(0.0), k(0.0)))

    elif a.mode == 'step':
        # k_drive 전용. 포화 전 선형구간은 f_drive_max/k_drive ~= 0.78 m/s 뿐이라
        # 계단 크기를 그보다 작게 줘야 k_drive 가 식별된다. 풀스로틀 로그로는 안 된다.
        seg.append(('spool', a.ramp, 'drive', lambda t: a.v0 * t / max(a.ramp, 1e-3), k(a.steer)))
        seg.append(('base', a.t_step, 'drive', k(a.v0), k(a.steer)))
        for i in range(a.n):
            seg.append((f'up{i}', a.t_step, 'drive', k(a.v0 + a.dv), k(a.steer)))
            seg.append((f'dn{i}', a.t_step, 'drive', k(a.v0), k(a.steer)))
        seg.append(('brake', a.t_brake, 'stop', k(0.0), k(0.0)))

    elif a.mode == 'coast':
        seg.append(('spool', a.ramp, 'drive', lambda t: a.v * t / max(a.ramp, 1e-3), k(a.steer)))
        seg.append(('hold', a.hold, 'drive', k(a.v), k(a.steer)))
        seg.append(('coast', a.t_coast, 'coast', k(0.0), k(a.steer)))
        seg.append(('brake', a.t_brake, 'stop', k(0.0), k(0.0)))

    elif a.mode == 'circle' and a.v_steps:
        # ★alpha_char 측정용. 램프 대신 각 속도에서 정상상태로 머문다.
        # 이유 둘:
        #   (1) 슬립각 계산은 정상상태(dbeta/dt=0)를 전제한다. 램프 중에는 성립 안 함.
        #   (2) beta(횡속도 방향)는 pose 미분에서 오는 잡음이 큰 양이라, 한 점에
        #       수백 표본을 쌓아 평균해야 alpha_char 가 나온다.
        vs = [float(x) for x in a.v_steps.split(',')]
        seg.append(('entry', a.t_entry, 'drive', k(vs[0]),
                    lambda t: a.steer * min(1.0, t / max(a.t_entry * 0.5, 1e-3))))
        for i, vv in enumerate(vs):
            # 각 계단 앞에 짧은 전이구간을 두고, 그 뒤 dwell 만 분석에 쓴다
            seg.append((f'tr{i}', a.t_trans, 'drive', k(vv), k(a.steer)))
            seg.append((f'hold{i}', a.t_hold, 'drive', k(vv), k(a.steer)))
        seg.append(('brake', a.t_brake, 'drive', k(0.0), k(a.steer)))
        seg.append(('settle', 1.0, 'stop', k(0.0), k(0.0)))

    elif a.mode == 'circle':
        # 정상 조향각 + 속도 램프. 콘 원을 손으로 따라가는 정반경 방식보다
        # 훨씬 재현성이 좋고, 한 런에서 언더스티어 곡선 전체(= alpha_char)와
        # 한계(= mu)를 같이 준다. 반경은 사후에 v/wz 로 나온다.
        seg.append(('entry', a.t_entry, 'drive', k(a.v0),
                    lambda t: a.steer * min(1.0, t / max(a.t_entry * 0.5, 1e-3))))
        seg.append(('ramp', a.t, 'drive',
                    lambda t: a.v0 + (a.v1 - a.v0) * t / max(a.t, 1e-3), k(a.steer)))
        seg.append(('brake', a.t_brake, 'drive', k(0.0), k(a.steer)))
        seg.append(('settle', 1.0, 'stop', k(0.0), k(0.0)))

    elif a.mode == 'steer':
        # 조향 계단 응답. --v 0 이면 차를 들어올린 무부하 측정, >0 이면 부하 측정.
        seg.append(('spool', a.ramp if a.v > 0 else 0.3, 'drive',
                    lambda t: a.v * min(1.0, t / max(a.ramp, 1e-3)), k(0.0)))
        # --bias 를 주면 좌우로 흔드는 대신 '한쪽으로 도는 중에' 조향을 계단으로 바꾼다.
        #   bias=0     -> 슬라럼. 직선 레인이 필요하다(20 m 급).
        #   bias>=amp  -> 조향 부호가 안 바뀌므로 계속 한 방향으로 돈다. 원 하나에 들어간다.
        for i in range(a.n):
            seg.append((f'L{i}', a.period / 2.0, 'drive', k(a.v), k(a.bias + a.amp)))
            seg.append((f'R{i}', a.period / 2.0, 'drive', k(a.v), k(a.bias - a.amp)))
        seg.append(('brake', a.t_brake, 'stop', k(0.0), k(0.0)))

    else:
        raise SystemExit(f'모르는 모드: {a.mode}')

    return seg


class SysIdCmd(Node):
    def __init__(self, a, seg):
        super().__init__('sysid_cmd')
        self.a = a
        self.seg = seg
        self.total = sum(s[1] for s in seg)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.erpm_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.cur_pub = self.create_publisher(Float64, '/commands/motor/current', 10)

        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_subscription(Imu, '/sensors/imu/raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, a.pose_topic, self.on_pose, 10)
        if VescStateStamped is not None:
            self.create_subscription(VescStateStamped, '/sensors/core', self.on_core, 10)

        # 측정 상태
        self.joy_t = 0.0
        self.joy_seen = False
        self.btns = []
        self.axes = []
        self.armed = False          # 데드맨 눌림
        self.run_t0 = None          # 시퀀스 시작 시각 (arm 후 arm_delay 지나서)
        self.done = False
        self.done_t = 0.0
        self.finished = False
        self.aborts = 0
        self.wz = self.ax = self.ay = 0.0
        self.erpm = self.cur_m = self.duty = self.vbat = 0.0
        self.px = self.py = self.pyaw = 0.0
        self.pt = 0.0
        self.v_pose = 0.0
        self._last_pose = None
        self._vmap = None
        self.beta = 0.0
        self.last_print = 0.0

        self.rows = []
        self.t_start_wall = time.time()
        self.timer = self.create_timer(1.0 / a.rate, self.tick)

        self.get_logger().info(
            f"모드={a.mode} 총길이={self.total:.1f}s  데드맨=버튼{a.deadman} "
            f"(누르고 있어야 움직인다, 떼면 즉시 제동)")
        if a.no_deadman:
            self.get_logger().warn(
                "★데드맨 없음 — 차가 스스로 출발한다. 정지 수단은 아래 둘뿐이다.")
        self.get_logger().warn(
            "e-stop: 조이스틱 Circle(idx 2) = VESC 레벨 래치 정지 / Triangle(idx 3) = 해제")
        self.get_logger().warn("       터미널 Ctrl-C = 시퀀스 중단 후 정지 명령 발행")

    # ---------------- 콜백 ----------------
    def on_joy(self, m):
        self.joy_t = time.time()
        self.joy_seen = True
        self.btns = list(m.buttons)
        self.axes = list(m.axes)
        i = self.a.deadman
        self.armed = (i < len(m.buttons)) and (m.buttons[i] == 1)
        j = self.a.abort_button
        if 0 <= j < len(m.buttons) and m.buttons[j] == 1:
            self.armed = False

    def on_imu(self, m):
        self.wz = m.angular_velocity.z
        self.ax = m.linear_acceleration.x
        self.ay = m.linear_acceleration.y

    def on_core(self, m):
        self.erpm = m.state.speed
        self.cur_m = m.state.current_motor
        self.duty = m.state.duty_cycle
        self.vbat = m.state.voltage_input

    def on_pose(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x, y = m.pose.position.x, m.pose.position.y
        q = m.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self._last_pose is not None:
            t0, x0, y0 = self._last_pose
            dt = t - t0
            if 1e-3 < dt < 0.2:
                vxm, vym = (x - x0) / dt, (y - y0) / dt
                v = math.hypot(vxm, vym)
                self.v_pose = 0.7 * self.v_pose + 0.3 * v     # 가벼운 저역통과
                if self._vmap is None:
                    self._vmap = (vxm, vym)
                else:
                    self._vmap = (0.7 * self._vmap[0] + 0.3 * vxm,
                                  0.7 * self._vmap[1] + 0.3 * vym)
        self._last_pose = (t, x, y)
        # beta(차체 슬립각) — alpha_char 측정의 핵심 신호라 현장에서 눈으로 확인한다
        if self._vmap is not None and self.v_pose > 0.5:
            vxm, vym = self._vmap
            self.beta = math.atan2(-math.sin(yaw) * vxm + math.cos(yaw) * vym,
                                   math.cos(yaw) * vxm + math.sin(yaw) * vym)
        self.px, self.py, self.pyaw, self.pt = x, y, yaw, t

    # ---------------- 프로파일 조회 ----------------
    def sample(self, t):
        acc = 0.0
        for name, dur, kind, vf, df in self.seg:
            if t < acc + dur:
                tau = t - acc
                return name, kind, float(vf(tau)), float(df(tau))
            acc += dur
        return 'end', 'stop', 0.0, 0.0

    # ---------------- 메인 루프 ----------------
    def tick(self):
        now = time.time()
        a = self.a
        joy_ok = (now - self.joy_t) < a.joy_timeout
        if a.no_deadman:
            # 데드맨 없이 자동 실행. 단 /joy 가 한 번이라도 잡혔다면 그 뒤로는
            # 살아 있어야 한다 — VESC e-stop(Circle)이 /joy 를 구독하므로,
            # /joy 가 끊기면 유일한 정지 수단이 사라진 것이다.
            gate = joy_ok or not self.joy_seen
        else:
            gate = self.armed and joy_ok

        if self.done:
            self.publish_stop()
            # 완료 후 hold_after 동안 정지 명령을 더 쏘고 스스로 끝낸다.
            # (Ctrl-C 를 기다리면 run_sysid.sh 가 분석으로 못 넘어간다)
            if self.a.hold_after >= 0 and (now - self.done_t) > self.a.hold_after:
                self.finished = True
            return

        if not gate:
            if self.run_t0 is not None:
                self.aborts += 1
                self.get_logger().warn(
                    '/joy 끊김 -> 정지. e-stop 경로가 사라졌으므로 진행하지 않는다. '
                    '복구되면 처음부터 다시 시작한다.'
                    if self.a.no_deadman else
                    '데드맨 해제/조이 끊김 -> 정지. 다시 누르면 시퀀스를 처음부터 재시작한다.')
                self.run_t0 = None
            self.publish_stop()
            self.log(now, 'IDLE', 'stop', 0.0, 0.0)
            self.idle_status(now, joy_ok)
            return

        if self.run_t0 is None:
            delay = a.arm_delay if not a.no_deadman else max(a.arm_delay, 3.0)
            self.run_t0 = now + delay
            self.get_logger().warn(f'{delay:.0f}초 후 자동 출발 — 차에서 떨어질 것')

        t = now - self.run_t0
        if t < 0.0:
            self.publish_stop()
            self.log(now, 'ARM', 'stop', 0.0, 0.0)
            if now - self.last_print > 0.2:
                self.last_print = now
                print(f'\r[출발 {-t:4.1f}초 전]  (Ctrl-C 또는 Circle 버튼으로 취소)      ',
                      end='', flush=True)
            return

        if t >= self.total:
            self.done = True
            self.done_t = now
            self.publish_stop()
            self.get_logger().info(
                f'시퀀스 완료. {self.a.hold_after:.0f}s 뒤 자동 종료 (즉시 끝내려면 Ctrl-C).'
                if self.a.hold_after >= 0 else '시퀀스 완료. Ctrl-C 로 종료.')
            return

        name, kind, v, d = self.sample(t)
        d = max(-a.max_steer, min(a.max_steer, d))
        v = max(-a.max_speed, min(a.max_speed, v))

        if kind == 'coast':
            # 타력주행: /drive 를 끊고 전류 0 을 쏜다. VESC 는 최소전류 미만이면
            # 모터를 놓는다(free-wheel). 속도 0 을 쏘면 그건 '0rpm 유지' = 제동이라
            # 코스트다운이 안 된다.
            msg = Float64(); msg.data = 0.0
            self.cur_pub.publish(msg)
        elif a.raw:
            e = Float64(); e.data = v * a.erpm_gain
            s = Float64(); s.data = a.servo_offset + a.servo_gain * d
            self.erpm_pub.publish(e); self.servo_pub.publish(s)
        else:
            self.drive_pub.publish(self.mk_drive(v, d))

        self.log(now, name, kind, v, d)
        self.status(now, t, name, v, d)

    def mk_drive(self, v, d):
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.drive.speed = float(v)
        m.drive.steering_angle = float(d)
        return m

    def publish_stop(self):
        if self.a.raw:
            e = Float64(); e.data = 0.0
            s = Float64(); s.data = self.a.servo_offset
            self.erpm_pub.publish(e); self.servo_pub.publish(s)
        else:
            self.drive_pub.publish(self.mk_drive(0.0, 0.0))

    def log(self, now, phase, kind, v, d):
        self.rows.append(dict(
            t=now - self.t_start_wall, phase=phase, kind=kind,
            v_cmd=v, delta_cmd=d,
            servo_cmd=self.a.servo_offset + self.a.servo_gain * d,
            erpm_cmd=v * self.a.erpm_gain,
            erpm_meas=self.erpm, current_motor=self.cur_m, duty=self.duty, v_batt=self.vbat,
            wz=self.wz, ax=self.ax, ay=self.ay,
            v_pose=self.v_pose, x=self.px, y=self.py, yaw=self.pyaw, pose_t=self.pt))

    def idle_status(self, now, joy_ok):
        """게이트가 안 열린 채로 조용히 서 있으면 사용자가 원인을 알 수 없다.
        무엇이 막고 있는지 계속 찍어준다."""
        if now - self.last_print < 0.5:
            return
        self.last_print = now
        if not joy_ok:
            msg = (f'/joy 가 {now - self.joy_t:.1f}s 째 안 온다 '
                   + ('★e-stop 이 사라졌다. 시퀀스 중단' if self.a.no_deadman
                      else '(조이스틱 연결/페어링 확인)')) if self.joy_t \
                else '/joy 를 아직 한 번도 못 받았다'
        else:
            pressed = [i for i, b in enumerate(self.btns) if b]
            n = len(self.btns)
            if self.a.deadman >= n:
                msg = (f'데드맨 idx {self.a.deadman} 가 버튼 개수({n})를 넘는다 '
                       f'-> --deadman 을 0~{n-1} 로 줄 것')
            else:
                msg = (f'데드맨 버튼 {self.a.deadman} 을 누르고 있어야 시작한다. '
                       f'지금 눌린 버튼: {pressed if pressed else "없음"}  (버튼 {n}개)')
        print(f'\r[대기] {msg}   (버튼 번호는 --probe 로 확인)      ',
              end='', flush=True)

    def status(self, now, t, phase, v, d):
        if now - self.last_print < 0.2:
            return
        self.last_print = now
        v_erpm = self.erpm / self.a.erpm_gain if self.a.erpm_gain else 0.0
        alat = self.v_pose * self.wz
        extra = ''
        if self.a.radius:
            # 콘 원을 따라 도는 중이면 v = wz*R 이라 측위 없이도 횡가속이 나온다.
            ag = self.wz * self.wz * self.a.radius
            self.mu_geo_max = max(getattr(self, 'mu_geo_max', 0.0), ag / G)
            extra = f' | R={self.a.radius:.1f}m a_geo={ag:5.2f} mu_geo={ag/G:4.2f} (max {self.mu_geo_max:4.2f})'
        print(f'\r[{t:5.2f}/{self.total:.1f}s {phase:>7s}] '
              f'v_cmd={v:5.2f} v_pose={self.v_pose:5.2f} v_erpm={v_erpm:5.2f} '
              f'delta={d:+.3f} wz={self.wz:+6.3f} a_lat={alat:+6.2f} '
              f'(mu>={abs(alat)/G:4.2f}) beta={self.beta:+.3f} I={self.cur_m:5.1f}A{extra}',
              end='', flush=True)

    def save(self):
        if not self.rows or not self.a.out:
            return
        os.makedirs(os.path.dirname(os.path.abspath(self.a.out)) or '.', exist_ok=True)
        with open(self.a.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader(); w.writerows(self.rows)
        print(f'\nCSV 저장: {self.a.out}  ({len(self.rows)}행, 중단 {self.aborts}회)')
        if getattr(self, 'mu_geo_max', 0.0) > 0:
            print(f'콘 원 기하 기준 최대 mu = {self.mu_geo_max:.3f}  (a_lat = wz^2 * R, 측위 무관)')


def probe_joy():
    """/joy 를 그대로 보여준다. 데드맨으로 쓸 버튼 번호를 찾는 용도."""
    from rclpy.node import Node as _Node

    class Probe(_Node):
        def __init__(self):
            super().__init__('sysid_joy_probe')
            self.m = None
            self.create_subscription(Joy, '/joy', self.cb, 10)
            self.create_timer(0.1, self.tick)
            print('버튼을 하나씩 눌러 번호를 확인할 것. Ctrl-C 로 종료.')
            print('(vesc.yaml 주석 기준: 2=Circle, 3=Triangle, 4=L1, 5=R1)')

        def cb(self, m):
            self.m = m

        def tick(self):
            if self.m is None:
                print('\r/joy 대기 중...', end='', flush=True); return
            pressed = str([i for i, b in enumerate(self.m.buttons) if b]) if any(self.m.buttons) else '없음'
            ax = ' '.join(f'{i}:{v:+.2f}' for i, v in enumerate(self.m.axes) if abs(v) > 0.15)
            print(f'\r버튼 {len(self.m.buttons)}개 중 눌림: {pressed:<20s} '
                  f'| 축: {ax:<40s}', end='', flush=True)

    rclpy.init()
    n = Probe()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        print()
    except Exception as e:                      # SIGTERM 등 외부 종료
        if type(e).__name__ != 'ExternalShutdownException':
            raise
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(
        description='연습장 물리계수 실측용 오픈루프 명령 노드',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('--mode', required=False,
                   choices=['const', 'accel', 'step', 'coast', 'circle', 'steer'])
    p.add_argument('--probe', action='store_true',
                   help='/joy 버튼/축 번호를 실시간으로 보여준다 (데드맨 번호 찾기용)')
    p.add_argument('--v', type=float, default=5.0, help='목표 속도 [m/s]')
    p.add_argument('--v0', type=float, default=1.0, help='시작 속도 (circle/step)')
    p.add_argument('--v1', type=float, default=5.0, help='끝 속도 (circle)')
    p.add_argument('--dv', type=float, default=0.4, help='계단 크기 (step). 0.78 미만이어야 k_drive 가 식별된다')
    p.add_argument('--n', type=int, default=4, help='반복 수 (step/steer)')
    p.add_argument('--t', type=float, default=2.5, help='본 구간 길이 [s]')
    p.add_argument('--t-step', dest='t_step', type=float, default=0.8,
                   help='계단 하나의 길이 [s]. tau~0.13s 라 0.5s(=4tau)면 이미 정착한다')
    p.add_argument('--t-coast', dest='t_coast', type=float, default=4.0)
    p.add_argument('--t-entry', dest='t_entry', type=float, default=2.0)
    p.add_argument('--v-steps', dest='v_steps', default=None,
                   help='circle 모드를 속도 계단으로 (예: "2.0,2.5,3.0,3.3,3.6"). '
                        'alpha_char 측정용 — 각 속도에서 정상상태로 머문다')
    p.add_argument('--t-hold', dest='t_hold', type=float, default=3.0,
                   help='계단당 정상상태 유지 시간 [s] (--v-steps)')
    p.add_argument('--t-trans', dest='t_trans', type=float, default=1.2,
                   help='계단 사이 전이 시간 [s] — 분석에서 제외된다 (--v-steps)')
    p.add_argument('--t-brake', dest='t_brake', type=float, default=2.0)
    p.add_argument('--hold', type=float, default=2.0)
    p.add_argument('--ramp', type=float, default=1.0, help='초기 가속 구간 길이 [s]')
    p.add_argument('--steer', type=float, default=0.0, help='조향각 [rad] (circle 은 여기 값으로 고정)')
    p.add_argument('--amp', type=float, default=0.30, help='조향 진폭 [rad] (steer)')
    p.add_argument('--period', type=float, default=1.0, help='조향 주기 [s] (steer)')
    p.add_argument('--bias', type=float, default=0.0,
                   help='조향 계단의 중심각 [rad] (steer). 0=슬라럼(직선 레인 필요), '
                        'amp 이상이면 한 방향 선회 안에서 계단을 준다(원 하나로 끝난다)')
    p.add_argument('--v-brake', dest='v_brake', type=float, default=0.0,
                   help='제동 구간 명령속도. 0=0rpm 유지, 음수=역방향(더 강함)')

    p.add_argument('--max-steer', dest='max_steer', type=float, default=0.40,
                   help='조향 클램프 [rad] (기계한계 0.44)')
    p.add_argument('--max-speed', dest='max_speed', type=float, default=8.0,
                   help='속도 클램프 [m/s]')
    p.add_argument('--deadman', type=int, default=5, help='데드맨 버튼 idx (5=R1)')
    p.add_argument('--no-deadman', dest='no_deadman', action='store_true',
                   help='데드맨을 쓰지 않고 명령만으로 자동 실행한다. '
                        '★차가 스스로 출발한다. 정지 수단은 VESC e-stop(Circle, 버튼2)뿐이고 '
                        '그건 /joy 가 살아 있어야 동작하므로, /joy 가 끊기면 시퀀스를 중단한다')
    p.add_argument('--abort-button', dest='abort_button', type=int, default=-1)
    p.add_argument('--joy-timeout', dest='joy_timeout', type=float, default=0.3)
    p.add_argument('--arm-delay', dest='arm_delay', type=float, default=1.0)
    p.add_argument('--rate', type=float, default=50.0)
    p.add_argument('--hold-after', dest='hold_after', type=float, default=2.0,
                   help='시퀀스 완료 뒤 정지 명령을 더 쏘고 자동 종료하기까지 [s]. '
                        '-1 이면 자동 종료하지 않고 Ctrl-C 를 기다린다')
    p.add_argument('--raw', action='store_true', help='/drive 대신 ERPM/서보 직접 발행')
    p.add_argument('--erpm-gain', dest='erpm_gain', type=float, default=3576.0)
    p.add_argument('--servo-gain', dest='servo_gain', type=float, default=-0.65)
    p.add_argument('--servo-offset', dest='servo_offset', type=float, default=0.5)
    p.add_argument('--pose-topic', dest='pose_topic', default='/car_state/pose')
    p.add_argument('--radius', type=float, default=None,
                   help='콘으로 표시한 원 반경 [m]. 주면 측위 없이도 mu 를 읽어준다: '
                        'a_lat = wz^2 * R (v = wz*R 이므로). 측위가 없는 연습장에서 쓴다.')
    p.add_argument('--out', default=None, help='명령/측정 CSV 경로')
    p.add_argument('--dry-run', dest='dry_run', action='store_true',
                   help='ROS 발행 없이 프로파일만 출력')
    a = p.parse_args()

    if a.probe:
        return probe_joy()
    if not a.mode:
        p.error('--mode 를 주거나 --probe 를 쓸 것')

    if a.out is None:
        a.out = os.path.expanduser(
            f'~/forza_ws/race_stack/sysid_{a.mode}_{time.strftime("%m%d_%H%M%S")}.csv')

    seg = build_profile(a)
    total = sum(s[1] for s in seg)
    print(f'--- 프로파일 ({a.mode}, 총 {total:.1f}s) ---')
    acc = 0.0
    for name, dur, kind, vf, df in seg:
        print(f'  {acc:6.2f}~{acc+dur:6.2f}s  {name:>8s} [{kind:5s}] '
              f'v: {vf(0):.2f} -> {vf(max(dur-1e-6,0)):.2f}   '
              f'delta: {df(0):+.3f} -> {df(max(dur-1e-6,0)):+.3f}')
        acc += dur
    if a.dry_run:
        return

    rclpy.init()
    node = SysIdCmd(a, seg)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception as e:                      # SIGTERM 등 외부 종료
        if type(e).__name__ != 'ExternalShutdownException':
            raise
    finally:
        try:
            for _ in range(10):
                node.publish_stop()
                time.sleep(0.02)
        except Exception:
            pass
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
