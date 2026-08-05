#!/usr/bin/env python3
"""DACER++ 강화학습 정책(dacerpp_isaaclab) 실차 컨트롤러.

Isaac Lab 에서 학습한 디퓨전 정책을 그대로 싣고, 학습 관측(58차원)을 실차
토픽으로 재구성해 30Hz 로 /l1controller/control 을 발행한다. mux_controller 가
그 값을 /drive 로 내보내므로 조이스틱 오버라이드와 /e_stop 이 그대로 살아 있다.

관측 구성 (dacerpp_lab/racing_env.py `_observe_car` 와 순서/정규화 동일):
    [0:32]  스캔 32빔 / 10m,  전방 ±135도
    [32]    속력 / v_max(10)                <- 학습 정규화 상수, 주행 v_max 와 무관
    [33:35] sin/cos(헤딩오차)
    [35]    횡오차 / 지역 반폭   (±1 = 벽)
    [36:41] 전방 곡률 5개  (+5/15/30/60/90 idx = 0.75/2.25/4.5/9/13.5m)
    [41:47] 현재+전방 반폭 6개 / 2.5m
    [47:51] 직전 2스텝의 '명령' 행동 (지연 하 Markov 복원용)
    [51:56] 상대차량 5개  <- 타임트라이얼이라 항상 0 (= 미검출)
    [56]    요레이트 / 4.0
    [57]    횡속도 / 3.0

행동 -> 명령: steer = a0 * max_steering_angle,
             speed = v_min + (a1+1)/2 * (v_max - v_min)   (학습의 속도 커리큘럼과 동일 형태)
"""
from __future__ import annotations

import math
import os
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from f110_msgs.msg import L1controllerControl, WpntArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import Float32MultiArray

from .scan_builder import PseudoScanBuilder
from .track_reference import TrackReference

DEFAULT_CKPT = os.path.expanduser("~/shared_dir/dacerpp_isaaclab/dacerpp_runs/20260726/cvar.pt")


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RLControllerNode(Node):

    def __init__(self):
        super().__init__("rl_controller")

        # ---------------- 파라미터 ---------------- #
        p = self.declare_parameter
        p("checkpoint", DEFAULT_CKPT)
        p("device", "cpu")                  # cpu / cuda / auto (젯슨 pip torch 는 cuda 커널 없음)
        p("torch_threads", 1)
        p("policy_seed", 0)
        p("num_action_candidates", 1)       # >1 이면 QVN 으로 후보 선택 (학습 기본 1)
        p("explore_noise_std", 0.0)         # 실차는 탐험 노이즈 없음

        p("control_rate", 30.0)             # 학습 제어 주기 (decimation 4 @120Hz)
        p("v_max", 5.0)                     # 행동->속도 매핑 상한 [m/s]
        p("v_min", 1.0)                     # 학습 RacingCfg.v_min
        p("obs_v_max", 10.0)                # 학습 RacingCfg.v_max (관측 정규화 — 고정)
        p("max_steering_angle", 0.42)       # 학습 RacingCfg.max_steering_angle
        p("speed_mode", "scale")            # scale | clip

        p("n_beams", 32)
        p("scan_fov", 2.356)
        p("scan_max_range", 10.0)
        p("scan_z_min", 0.02)               # base_link 기준 높이 밴드 [m]
        p("scan_z_max", 0.30)
        p("scan_min_range", 0.05)
        p("self_box", [-0.40, 0.55, 0.20])  # 자차 반사 제외 박스 [x_min, x_max, |y|]
        p("scan_deskew", True)
        p("lidar_frame", "livox_frame")
        p("lidar_translation", [0.27, 0.0, 0.07])
        p("lidar_yaw", 1.5707963)

        p("curv_lookahead", [5, 15, 30, 60, 90])
        p("hw_ref", 2.5)                    # 학습 0.5*TrackParams.width_max
        p("yawrate_norm", 4.0)
        p("vy_norm", 3.0)
        p("act_hist_len", 2)

        p("odom_topic", "/car_state/odom")
        p("centerline_topic", "/centerline_waypoints")
        p("lidar_topic", "/livox/lidar")
        p("laserscan_topic", "/scan")
        p("scan_source", "cloud")           # cloud | laserscan
        p("imu_topic", "/sensors/imu/raw")
        p("yawrate_source", "auto")         # auto | odom | imu
        p("drive_topic", "/l1controller/control")

        p("odom_timeout", 0.2)
        p("scan_timeout", 0.5)
        p("publish_debug", True)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.rate = float(g("control_rate"))
        self.v_max = float(g("v_max"))
        self.v_min = float(g("v_min"))
        self.obs_v_max = float(g("obs_v_max"))
        self.max_steer = float(g("max_steering_angle"))
        self.speed_mode = str(g("speed_mode")).lower()
        self.hw_ref = float(g("hw_ref"))
        self.yawrate_norm = float(g("yawrate_norm"))
        self.vy_norm = float(g("vy_norm"))
        self.curv_off = [int(v) for v in g("curv_lookahead")]
        self.width_off = [0] + self.curv_off
        self.act_hist_len = int(g("act_hist_len"))
        self.odom_timeout = float(g("odom_timeout"))
        self.scan_timeout = float(g("scan_timeout"))
        self.publish_debug = bool(g("publish_debug"))
        self.yawrate_source = str(g("yawrate_source")).lower()

        if self.v_max <= self.v_min:
            raise RuntimeError(f"v_max({self.v_max}) 는 v_min({self.v_min}) 보다 커야 합니다")

        # ---------------- 정책 로드 ---------------- #
        from .torch_loader import import_torch, resolve_device
        torch = import_torch()
        torch.set_num_threads(max(1, int(g("torch_threads"))))
        device = resolve_device(torch, str(g("device")))
        ckpt = os.path.expanduser(str(g("checkpoint")))
        if not os.path.isfile(ckpt):
            raise RuntimeError(f"체크포인트를 찾을 수 없습니다: {ckpt}")

        from .policy import DacerPolicy
        t0 = time.perf_counter()
        self.policy = DacerPolicy.load(
            ckpt, device=device, num_candidates=int(g("num_action_candidates")),
            seed=int(g("policy_seed")), explore_noise_std=float(g("explore_noise_std")))
        self.policy.warmup(10)
        self.get_logger().info(
            f"정책 로드 완료: {os.path.basename(ckpt)} obs_dim={self.policy.obs_dim} "
            f"act_dim={self.policy.act_dim} risk={self.policy.risk_measure}"
            f"({self.policy.risk_eta}) T={self.policy._T} "
            f"cand={self.policy.num_candidates} device={device} "
            f"({time.perf_counter() - t0:.1f}s)")

        self.obs_dim = self.policy.obs_dim
        expected = (int(g("n_beams")) + 1 + 2 + 1 + len(self.curv_off)
                    + len(self.width_off) + 2 * self.act_hist_len + 5 + 2)
        if expected != self.obs_dim:
            raise RuntimeError(
                f"관측 차원 불일치: 체크포인트 {self.obs_dim} vs 구성 {expected}. "
                f"n_beams/curv_lookahead/act_hist_len 파라미터를 학습값과 맞추세요.")

        # ---------------- 스캔 빌더 ---------------- #
        self.scan = PseudoScanBuilder(
            n_beams=int(g("n_beams")), fov=float(g("scan_fov")),
            max_range=float(g("scan_max_range")), z_min=float(g("scan_z_min")),
            z_max=float(g("scan_z_max")), min_range=float(g("scan_min_range")),
            self_box=tuple(float(v) for v in g("self_box")), deskew=bool(g("scan_deskew")))
        self.scan.set_extrinsic(np.array([float(v) for v in g("lidar_translation")]),
                                float(g("lidar_yaw")))
        self.scan_source = str(g("scan_source")).lower()

        # ---------------- 상태 ---------------- #
        self.track: Optional[TrackReference] = None
        self.pose: Optional[Tuple[float, float, float]] = None    # x, y, yaw
        self.vx = self.vy = self.wz = 0.0
        self.imu_wz: Optional[float] = None
        self.t_odom = -1.0
        self.t_scan = -1.0
        self.hist = np.zeros(2 * self.act_hist_len, dtype=np.float32)
        self.last_obs: Optional[np.ndarray] = None
        self._stopped = True
        self._odom_wz_zero_since: Optional[float] = None
        self._infer_ms = 0.0

        # 디스큐용 포즈 링버퍼 (2초 @100Hz)
        self._buf_n = 256
        self._buf_t = np.zeros(self._buf_n)
        self._buf_x = np.zeros(self._buf_n)
        self._buf_y = np.zeros(self._buf_n)
        self._buf_yaw = np.zeros(self._buf_n)
        self._buf_i = 0
        self._buf_filled = 0

        # ---------------- 통신 ---------------- #
        # global_republisher 는 5초마다 재발행한다(volatile) -> 일반 QoS 로 충분.
        self.create_subscription(WpntArray, str(g("centerline_topic")), self.wpnt_cb, 10)
        self.create_subscription(Odometry, str(g("odom_topic")), self.odom_cb, 10)
        self.create_subscription(Imu, str(g("imu_topic")), self.imu_cb, 10)
        if self.scan_source == "laserscan":
            self.create_subscription(LaserScan, str(g("laserscan_topic")), self.laserscan_cb, 1)
        else:
            self.create_subscription(PointCloud2, str(g("lidar_topic")), self.cloud_cb, 1)

        self.drive_pub = self.create_publisher(L1controllerControl, str(g("drive_topic")), 1)
        if self.publish_debug:
            self.scan_pub = self.create_publisher(LaserScan, "/rl_controller/scan", 1)
            self.obs_pub = self.create_publisher(Float32MultiArray, "/rl_controller/obs", 1)

        self.timer = self.create_timer(1.0 / self.rate, self.control_loop)
        self.get_logger().info(
            f"rl_controller 시작: {self.rate:.0f}Hz, v={self.v_min:.1f}~{self.v_max:.1f}m/s "
            f"({self.speed_mode}), |steer|<={self.max_steer:.2f}rad, "
            f"스캔소스={self.scan_source}")

    # ------------------------------------------------------------------ #
    # 콜백
    # ------------------------------------------------------------------ #
    def wpnt_cb(self, msg: WpntArray):
        if self.track is not None or not msg.wpnts:
            return
        try:
            self.track = TrackReference.from_centerline_waypoints(
                [w.x_m for w in msg.wpnts], [w.y_m for w in msg.wpnts],
                [w.d_left for w in msg.wpnts], [w.d_right for w in msg.wpnts],
                logger=self.get_logger())
        except Exception as exc:                                   # noqa: BLE001
            self.get_logger().error(f"중심선 기준선 생성 실패: {exc}")
            return
        self.get_logger().info(f"중심선 기준선 생성: {self.track.summary()}")

    def odom_cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.wz = msg.twist.twist.angular.z
        self.t_odom = t

        i = self._buf_i
        self._buf_t[i] = t
        self._buf_x[i] = self.pose[0]
        self._buf_y[i] = self.pose[1]
        # 보간을 위해 yaw 를 연속(unwrap)으로 저장
        prev = self._buf_yaw[(i - 1) % self._buf_n] if self._buf_filled else yaw
        self._buf_yaw[i] = prev + wrap_to_pi(yaw - prev)
        self._buf_i = (i + 1) % self._buf_n
        self._buf_filled = min(self._buf_filled + 1, self._buf_n)

        # 요레이트 소스 자동 판정: EKF wz 가 계속 정확히 0 이면 IMU 로 전환
        if self.yawrate_source == "auto":
            now = self._now()
            if self.wz == 0.0:
                if self._odom_wz_zero_since is None:
                    self._odom_wz_zero_since = now
                elif (now - self._odom_wz_zero_since > 3.0 and self.imu_wz is not None
                        and abs(self.vx) > 0.2):
                    self.yawrate_source = "imu"
                    self.get_logger().warn(
                        "/car_state/odom 의 angular.z 가 계속 0 -> 요레이트를 IMU 에서 받습니다.")
            else:
                self._odom_wz_zero_since = None

    def imu_cb(self, msg: Imu):
        # base_link <- imu 는 yaw +90도 회전(static TF)이라 z축은 그대로다.
        self.imu_wz = msg.angular_velocity.z

    def cloud_cb(self, msg: PointCloud2):
        try:
            n = self.scan.ingest_cloud(msg)
        except Exception as exc:                                   # noqa: BLE001
            self.get_logger().error(f"포인트클라우드 처리 실패: {exc}", throttle_duration_sec=2.0)
            return
        self.t_scan = self._now()
        if n == 0:
            self.get_logger().warn("높이 밴드 안에 점이 하나도 없습니다 "
                                   "(scan_z_min/scan_z_max 확인)", throttle_duration_sec=5.0)

    def laserscan_cb(self, msg: LaserScan):
        self.scan.ingest_laserscan(msg)
        self.t_scan = self._now()

    # ------------------------------------------------------------------ #
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose_at(self, times: np.ndarray):
        """점 시각 배열 -> (x, y, yaw). 버퍼가 못 덮으면 (None, None, None)."""
        if self._buf_filled < 4:
            return None, None, None
        if self._buf_filled < self._buf_n:
            ts = self._buf_t[:self._buf_filled]
            xs, ys, yaws = (self._buf_x[:self._buf_filled], self._buf_y[:self._buf_filled],
                            self._buf_yaw[:self._buf_filled])
        else:
            order = np.arange(self._buf_i, self._buf_i + self._buf_n) % self._buf_n
            ts, xs, ys, yaws = (self._buf_t[order], self._buf_x[order],
                                self._buf_y[order], self._buf_yaw[order])
        # 라이다/EKF 시계가 어긋나 있으면 보간이 무의미하다 -> 디스큐 생략
        if times.max() < ts[0] - 0.5 or times.min() > ts[-1] + 0.5:
            self.get_logger().warn(
                "라이다 타임스탬프가 포즈 버퍼 범위 밖입니다 -> 디스큐 생략 "
                f"(scan {times.min():.3f}~{times.max():.3f}, pose {ts[0]:.3f}~{ts[-1]:.3f})",
                throttle_duration_sec=10.0)
            return None, None, None
        return (np.interp(times, ts, xs), np.interp(times, ts, ys),
                np.interp(times, ts, yaws))

    # ------------------------------------------------------------------ #
    def build_observation(self) -> Optional[np.ndarray]:
        assert self.track is not None and self.pose is not None
        x, y, yaw = self.pose
        proj = self.track.project(x, y)
        idx = proj["idx"]
        hw_local = float(self.track.hw[idx])
        herr = wrap_to_pi(yaw - proj["psi"])

        scan, n_beams_hit = self.scan.build(self.pose, self._pose_at)
        self._last_scan = scan
        self._last_beams_hit = n_beams_hit

        curv = self.track.lookahead_curvature(idx, self.curv_off)
        width = self.track.lookahead_width(idx, self.width_off)
        speed = math.hypot(self.vx, self.vy)
        wz = self.imu_wz if (self.yawrate_source == "imu" and self.imu_wz is not None) else self.wz

        obs = np.concatenate([
            np.clip(scan / self.scan.max_range, 0.0, 1.0),
            [speed / self.obs_v_max, math.sin(herr), math.cos(herr)],
            [np.clip(proj["lateral"] / max(hw_local, 1e-6), -2.0, 2.0)],
            np.clip(curv, -1.0, 1.0),
            np.clip(width / self.hw_ref, 0.0, 1.0),
            np.clip(self.hist, -1.0, 1.0),
            np.zeros(5),                      # 상대차량 미검출 (타임트라이얼)
            [np.clip(wz / self.yawrate_norm, -1.0, 1.0),
             np.clip(self.vy / self.vy_norm, -1.0, 1.0)],
        ]).astype(np.float32)

        self._dbg = dict(lat=proj["lateral"], hw=hw_local, herr=herr, speed=speed,
                         wz=wz, s=proj["s"], idx=idx)
        return obs

    # ------------------------------------------------------------------ #
    def control_loop(self):
        reason = self._not_ready_reason()
        if reason is not None:
            if not self._stopped:
                self.get_logger().error(f"주행 중단 -> 정지 명령: {reason}")
            self._stopped = True
            self.hist[:] = 0.0
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn(f"대기 중: {reason}", throttle_duration_sec=2.0)
            return

        try:
            obs = self.build_observation()
            t0 = time.perf_counter()
            act = self.policy.act(obs)
            self._infer_ms = (time.perf_counter() - t0) * 1e3
        except Exception as exc:                                   # noqa: BLE001
            self.get_logger().error(f"추론 실패 -> 정지: {exc}", throttle_duration_sec=1.0)
            self._stopped = True
            self.publish_cmd(0.0, 0.0)
            return

        if self._stopped:
            self.get_logger().info("주행 시작 (관측/추론 정상)")
            self._stopped = False

        steer = float(np.clip(act[0], -1.0, 1.0)) * self.max_steer
        thr = float(np.clip(act[1], -1.0, 1.0))
        if self.speed_mode == "clip":
            speed = min(self.v_min + (thr + 1.0) * 0.5 * (self.obs_v_max - self.v_min),
                        self.v_max)
        else:
            speed = self.v_min + (thr + 1.0) * 0.5 * (self.v_max - self.v_min)
        self.publish_cmd(steer, speed)

        # 명령 히스토리 롤 (최신이 앞) — 학습 racing_env 와 동일
        self.hist = np.concatenate([[act[0], act[1]], self.hist[:-2]]).astype(np.float32)
        self.last_obs = obs

        if self.publish_debug:
            self.publish_debug_msgs(obs)
        d = self._dbg
        self.get_logger().info(
            f"v={d['speed']:.2f}->{speed:.2f} steer={steer:+.3f} lat={d['lat']:+.2f}/"
            f"{d['hw']:.2f} herr={d['herr']:+.2f} wz={d['wz']:+.2f} "
            f"beams={self._last_beams_hit}/{self.scan.n_beams} inf={self._infer_ms:.1f}ms",
            throttle_duration_sec=1.0)

    def _not_ready_reason(self) -> Optional[str]:
        if self.track is None:
            return "중심선 웨이포인트 대기 중 (/centerline_waypoints)"
        if self.pose is None:
            return "측위 대기 중 (/car_state/odom)"
        now = self._now()
        if now - self.t_odom > self.odom_timeout:
            return f"측위 끊김 ({now - self.t_odom:.2f}s)"
        if self.t_scan < 0.0:
            return "라이다 대기 중"
        if now - self.t_scan > self.scan_timeout:
            return f"라이다 끊김 ({now - self.t_scan:.2f}s)"
        return None

    # ------------------------------------------------------------------ #
    def publish_cmd(self, steering_angle: float, speed: float):
        msg = L1controllerControl()
        msg.steering_angle = float(steering_angle)
        msg.speed = float(speed)
        self.drive_pub.publish(msg)

    def publish_debug_msgs(self, obs: np.ndarray):
        ls = LaserScan()
        ls.header.stamp = self.get_clock().now().to_msg()
        ls.header.frame_id = "base_link"
        ls.angle_min = -self.scan.fov
        ls.angle_max = self.scan.fov
        ls.angle_increment = self.scan.spacing
        ls.range_min = float(self.scan.min_range)
        ls.range_max = float(self.scan.max_range)
        ls.ranges = [float(v) for v in self._last_scan]
        self.scan_pub.publish(ls)

        m = Float32MultiArray()
        m.data = [float(v) for v in obs]
        self.obs_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RLControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            # 종료 직전 정지 명령 (mux 가 마지막 값을 붙들고 있지 않게)
            try:
                node.publish_cmd(0.0, 0.0)
            except Exception:                                      # noqa: BLE001
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
