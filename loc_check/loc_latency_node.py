#!/usr/bin/env python3
"""
주행 중 VGICP 측위 지연 실측 노드. (실행은 measure_loc.sh 를 쓸 것)

측정 원리:
  scanmatcher는 /ndt_pose 를 발행할 때 header.stamp 에 '스캔 타임스탬프'를 그대로 싣는다
  (scanmatcher_component.cpp 의 publishMapAndPose: current_pose_stamped_.header.stamp = stamp).
  따라서  수신시각 - header.stamp  = 라이다 스캔 → 정합 완료까지의 end-to-end 지연이며,
  이 값이 EKF 지연 게이트를 넘으면 ekf_localizer가 해당 측정치를 '폐기'하고 추측항법으로 흐른다.

게이트 초과는 추정하지 않고 실측한다:
  ekf_localizer가 /diagnostics 로 pose_is_passed_delay_gate / pose_delay_time /
  pose_delay_time_threshold 를 직접 발행하므로(diagnostics.cpp) 그대로 집계한다.
  (게이트 한계 = extend_state_step * ekf_dt)

핵심 지표는 평균이 아니라 '꼬리와 속도 상관'이다. 과거 진단에서 문제는 평시 150ms인데
고속(3~4 m/s)에서만 600~870ms로 튀는 형태였으므로, 속도 구간별 p95를 반드시 같이 본다.
"""

import argparse
import csv
import math
import os
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def percentile(sorted_vals, q: float) -> float:
    """선형보간 백분위수. sorted_vals는 정렬되어 있어야 함."""
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return sorted_vals[int(k)]
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def fmt(v: float, nd: int = 1) -> str:
    return "n/a" if (v is None or math.isnan(v)) else f"{v:.{nd}f}"


# 속도 구간 [하한, 상한) — 고속에서만 지연이 튀는지 보기 위한 핵심 분해축
SPEED_BINS = [(0.0, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, float("inf"))]


class LocLatencyMonitor(Node):
    def __init__(self, args):
        super().__init__("loc_latency_monitor")

        self.target_ms = args.target_ms
        self.gate_ms = args.gate_ms          # /diagnostics 수신 전까지 쓰는 잠정값
        self.gate_from_diag = False
        self.match_tol = args.match_tol

        # --- 누적 통계 ---
        self.lat_ms = []                     # end-to-end 지연 전체 샘플
        self.lat_by_bin = {b: [] for b in SPEED_BINS}
        self.gaps_ms = []                    # /ndt_pose 연속 수신 간격 (드롭/정체 감지)
        self.diff_cm = []                    # ndt vs ekf 위치차
        self.gate_total = 0
        self.gate_failed = 0
        self.n_over_target = 0
        self.clock_warned = False

        # --- 최신값 ---
        self.speed = float("nan")
        self.last_recv = None
        self.last_lat = float("nan")
        self.t0 = time.time()
        self._t0_ros = self.get_clock().now().nanoseconds * 1e-9
        self.last_gate_passed = ""
        self.last_gate_delay_ms = ""
        self.last_gate_thresh_ms = ""

        # ndt pose 이력 (ekf와 스탬프 매칭용): (stamp_sec, x, y)
        self.ndt_hist = deque(maxlen=400)

        # --- CSV ---
        self.csv_file = None
        self.csv = None
        if args.csv:
            self.csv_file = open(args.csv, "w", newline="")
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(
                ["t_rel_s", "scan_stamp", "latency_ms", "gap_ms", "speed_mps",
                 "ndt_ekf_cm", "gate_passed", "gate_delay_ms", "gate_thresh_ms"]
            )

        # /ndt_pose 는 rclcpp::QoS(10) = RELIABLE. 측정 누락을 줄이려 depth만 키움.
        qos_rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(PoseStamped, args.ndt_topic, self.on_ndt, qos_rel)
        self.create_subscription(PoseStamped, args.ekf_topic, self.on_ekf, qos_rel)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, qos_rel)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, qos_rel)

        self.create_timer(1.0, self.on_tick)

        print(f"[측정 시작] ndt={args.ndt_topic}  ekf={args.ekf_topic}  odom={args.odom_topic}")
        print(f"  목표 p95 < {self.target_ms:.0f}ms | 게이트 한계는 /diagnostics 에서 자동 취득")
        if args.csv:
            print(f"  CSV: {args.csv}")
        print("  주행이 끝나면 Ctrl+C — 최종 요약이 출력됩니다.\n")

    # ------------------------------------------------------------------ 콜백
    def on_ndt(self, msg: PoseStamped):
        now = self.get_clock().now().nanoseconds * 1e-9
        scan_t = stamp_to_sec(msg.header.stamp)
        lat = (now - scan_t) * 1e3

        # 스탬프 기준이 어긋나면(라이다 클럭 미동기 등) 측정 자체가 무의미하므로 한 번 경고
        if (lat < -50.0 or lat > 10000.0) and not self.clock_warned:
            self.clock_warned = True
            print(f"  [경고] 지연이 비정상({lat:.0f}ms). 라이다/시스템 클럭 동기 또는 "
                  f"use_sim_time 설정을 확인하세요. 이후 값은 신뢰할 수 없습니다.")

        gap = float("nan")
        if self.last_recv is not None:
            gap = (now - self.last_recv) * 1e3
            self.gaps_ms.append(gap)
        self.last_recv = now

        self.last_lat = lat
        self.lat_ms.append(lat)
        for b in SPEED_BINS:
            if not math.isnan(self.speed) and b[0] <= self.speed < b[1]:
                self.lat_by_bin[b].append(lat)
                break

        if lat > self.target_ms:
            self.n_over_target += 1

        self.ndt_hist.append((scan_t, msg.pose.position.x, msg.pose.position.y))

        if self.csv:
            self.csv.writerow([
                f"{now - self._t0_ros:.3f}", f"{scan_t:.6f}", f"{lat:.2f}",
                "" if math.isnan(gap) else f"{gap:.2f}",
                "" if math.isnan(self.speed) else f"{self.speed:.3f}",
                f"{self.diff_cm[-1]:.2f}" if self.diff_cm else "",
                self.last_gate_passed, self.last_gate_delay_ms, self.last_gate_thresh_ms,
            ])

    def on_ekf(self, msg: PoseStamped):
        """ndt와 가장 가까운 스탬프끼리 짝지어 위치차를 계산 (두 토픽은 rate가 다름)."""
        if not self.ndt_hist:
            return
        t = stamp_to_sec(msg.header.stamp)
        best = min(self.ndt_hist, key=lambda r: abs(r[0] - t))
        if abs(best[0] - t) > self.match_tol:
            return
        d = math.hypot(msg.pose.position.x - best[1], msg.pose.position.y - best[2])
        self.diff_cm.append(d * 100.0)

    def on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.speed = math.hypot(v.x, v.y)

    def on_diag(self, msg: DiagnosticArray):
        """ekf_localizer가 직접 알려주는 지연 게이트 통과 여부를 집계.

        주의: publishDiagnostics()는 timerCallback 끝에서 predict_frequency(100Hz)로 호출되고,
        pose_is_passed_delay_gate_ 는 매 사이클 true로 리셋된다(ekf_localizer.cpp:192).
        즉 pose 측정치가 없던 사이클도 'True'로 찍히므로 그대로 세면 10배 부풀려진다.
        실제로 측정치가 처리된 사이클만 delay_time 이 0이 아니므로 그것만 집계한다.
        """
        for st in msg.status:
            if "ekf_localizer" not in st.name:
                continue
            kv = {v.key: v.value for v in st.values}
            if "pose_is_passed_delay_gate" not in kv:
                continue
            try:
                delay = float(kv.get("pose_delay_time", "0"))
            except ValueError:
                continue
            if delay == 0.0:      # 이번 사이클엔 처리된 pose 측정치가 없음
                continue

            passed = kv["pose_is_passed_delay_gate"] == "True"
            self.gate_total += 1
            if not passed:
                self.gate_failed += 1

            self.last_gate_passed = "True" if passed else "False"
            self.last_gate_delay_ms = f"{delay * 1e3:.2f}"
            try:
                thr = float(kv.get("pose_delay_time_threshold", "0"))
                if thr > 0.0:
                    self.gate_ms = thr * 1e3
                    self.gate_from_diag = True
                    self.last_gate_thresh_ms = f"{thr * 1e3:.1f}"
            except ValueError:
                pass

    # ------------------------------------------------------------------ 출력
    def on_tick(self):
        if not self.lat_ms:
            print(f"[{time.time() - self.t0:6.1f}s] /ndt_pose 수신 대기 중...")
            return
        s = sorted(self.lat_ms)
        el = time.time() - self.t0
        hz = len(self.lat_ms) / el if el > 0 else 0.0
        diff = f"{percentile(sorted(self.diff_cm), 0.5):.1f}" if self.diff_cm else "n/a"
        print(
            f"[{el:6.1f}s] ndt {hz:4.1f}Hz | 지연 now {self.last_lat:5.0f} "
            f"p95 {percentile(s, 0.95):5.0f} max {s[-1]:5.0f} ms | "
            f"v {fmt(self.speed, 2):>5} m/s | ndt-ekf {diff:>4} cm | "
            f"게이트초과 {self.gate_failed}"
        )

    def print_summary(self):
        print("\n" + "=" * 72)
        print(" VGICP 측위 지연 요약")
        print("=" * 72)

        if not self.lat_ms:
            print(" /ndt_pose 를 한 건도 받지 못했습니다.")
            print("   - base_system 이 떠 있는지, ROS_DOMAIN_ID 가 같은지 확인하세요.")
            print("=" * 72)
            return

        el = time.time() - self.t0
        s = sorted(self.lat_ms)
        n = len(s)
        print(f" 측정시간 {el:.1f}s | 샘플 {n}개 | 평균 rate {n / el:.2f} Hz (라이다 10Hz 기준)")
        print()
        print(" [end-to-end 지연: 스캔 → /ndt_pose 도착]")
        print(f"   mean {sum(s) / n:6.1f} ms   p50 {percentile(s, .5):6.1f}   "
              f"p90 {percentile(s, .9):6.1f}")
        print(f"   p95  {percentile(s, .95):6.1f} ms   p99 {percentile(s, .99):6.1f}   "
              f"max {s[-1]:6.1f}")

        p95 = percentile(s, 0.95)
        verdict = "양호" if p95 < self.target_ms else "목표 초과"
        print(f"   → p95 {p95:.0f}ms vs 목표 {self.target_ms:.0f}ms : {verdict}"
              f"  (목표 초과 프레임 {self.n_over_target}/{n}, {100 * self.n_over_target / n:.1f}%)")

        if self.gaps_ms:
            g = sorted(self.gaps_ms)
            print(f"\n [수신 간격] p95 {percentile(g, .95):.0f} ms  max {g[-1]:.0f} ms"
                  f"   (100ms 이상 = 프레임 유실/정체)")

        print("\n [속도 구간별 지연 — 고속에서 튀는지가 핵심]")
        any_bin = False
        for b in SPEED_BINS:
            vals = sorted(self.lat_by_bin[b])
            if not vals:
                continue
            any_bin = True
            hi = "inf" if b[1] == float("inf") else f"{b[1]:.0f}"
            print(f"   {b[0]:.0f}~{hi:>3} m/s : n={len(vals):5d}  "
                  f"mean {sum(vals) / len(vals):6.1f}  p95 {percentile(vals, .95):6.1f}  "
                  f"max {vals[-1]:6.1f} ms")
        if not any_bin:
            print("   (속도 정보 없음 — /car_state/odom 미수신)")

        print("\n [EKF 지연 게이트]")
        src = "/diagnostics 실측" if self.gate_from_diag else "미수신, 잠정값"
        print(f"   한계 {self.gate_ms:.0f} ms ({src})")
        if self.gate_total:
            pctf = 100.0 * self.gate_failed / self.gate_total
            print(f"   폐기된 pose 측정치 {self.gate_failed}/{self.gate_total} ({pctf:.2f}%)")
            if self.gate_failed:
                print("   → 폐기 구간은 추측항법으로 흘러 위치 오차가 누적됩니다.")
        else:
            print("   /diagnostics 미수신 (ekf_localizer 진단 발행 여부 확인)")

        if self.diff_cm:
            d = sorted(self.diff_cm)
            print(f"\n [ndt vs ekf 위치차] mean {sum(d) / len(d):.1f} cm  "
                  f"p95 {percentile(d, .95):.1f}  max {d[-1]:.1f} cm")
            print("   (크게 벌어지면 EKF가 스캔을 못 따라가고 있다는 신호)")

        print("=" * 72)
        if self.csv_file:
            print(f" CSV 저장됨: {self.csv_file.name}")
            print("=" * 72)

    def close(self):
        if self.csv_file:
            self.csv_file.close()


def main():
    ap = argparse.ArgumentParser(description="주행 중 VGICP 측위 지연 실측")
    ap.add_argument("--csv", default=None, help="CSV 저장 경로 (기본: 자동 생성, --no-csv 로 끄기)")
    ap.add_argument("--no-csv", action="store_true", help="CSV 저장 안 함")
    ap.add_argument("--target-ms", type=float, default=150.0, help="목표 p95 지연 (기본 150)")
    ap.add_argument("--gate-ms", type=float, default=1000.0,
                    help="/diagnostics 수신 전 잠정 게이트 한계 (기본 1000 = 100step/100Hz)")
    # /ndt_pose 는 '스캔 시각', /car_state/pose 는 'EKF 현재 시각'을 stamp 로 쓴다.
    # 둘은 정합 지연(~120ms)만큼 태생적으로 어긋나므로 허용오차가 그보다 커야 한다.
    # (0723 주행에서 기본 0.05s 였다가 한 건도 매칭되지 않아 위치차가 통째로 비었음)
    ap.add_argument("--match-tol", type=float, default=0.2,
                    help="ndt/ekf 스탬프 매칭 허용오차 [s] (기본 0.2)")
    ap.add_argument("--ndt-topic", default="/ndt_pose")
    ap.add_argument("--ekf-topic", default="/car_state/pose")
    ap.add_argument("--odom-topic", default="/car_state/odom")
    args = ap.parse_args()

    if args.no_csv:
        args.csv = None
    elif args.csv is None:
        root = os.path.expanduser("~/forza_ws/race_stack")
        args.csv = os.path.join(root, f"loc_latency_{time.strftime('%m%d_%H%M')}.csv")

    rclpy.init()
    node = LocLatencyMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
