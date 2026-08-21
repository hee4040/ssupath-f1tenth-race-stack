#!/usr/bin/env python3
"""로컬 웨이포인트(/local_waypoints)가 '어떻게' 만들어졌는지를 rviz 에 그리는 진단 노드.

보통은 직접 실행하지 않는다. record_obs.sh --viz (주행 중), replay_obs.sh (재생 중) 가
알아서 띄운다. 직접 쓸 일이 있으면:
    source /opt/ros/humble/setup.bash && source ~/forza_ws/race_stack/install/setup.bash
    ~/forza_ws/race_stack/loc_check/local_planner_viz.py             # 실차/실시간
    ~/forza_ws/race_stack/loc_check/local_planner_viz.py --sim-time  # bag 재생에 붙일 때
(colcon 패키지가 아니라 스크립트라 ros2 run 이 아니라 경로로 실행한다.)

왜 필요한가
-----------
state_machine 은 '결과' 배열만 내보낸다. rviz 에 보이는 건 초록 점 하나
(/local_waypoints/markers, 높이=속도)뿐이라, 그 점이 왜 거기 있고 왜 그 속도인지는 안 보인다.
정작 알고 싶은 건 두 가지다.

  1) 기하: 이 점이 전역 레이싱라인인가, graph_planner 회피선인가,
           회피선 마지막 점이 반복 복사된(clamped) 구간인가.
  2) 속도: 전역속도에서 '어느 단계'가 이 점의 속도를 결정했나.
           전역(scaled) -> 상태배율(ot_speed_scaling x 고속감속) -> 곡률한계(ay_max)
           -> 역방향 선제감속(ax_max)

이 노드는 state_machine 과 똑같은 입력만 보고 states.py 의 계산을 그대로 재현해서,
단계별 속도 곡선을 높이로 겹쳐 그리고 각 점을 '무엇이 깎았는지' 색으로 칠한다.
재현값은 매 사이클 실제 /local_waypoints 와 비교한다(model err). 스택 코드가 바뀌어
재현이 틀어지면 err 이 커지므로, 그림을 믿어도 되는지 그림 안에서 바로 확인된다.

재현 대상 코드 (2026-08-21 기준)
  state_machine/state_machine/states.py
    GlobalTracking / Trailing_to_gbtrack : 전역 웨이포인트 그대로 (속도 손 안 댐)
    Trailing / Overtaking                : get_splini_wpts() 기하 + _fuse_speed_from_global()
    Recovering                           : 중심선 후진 (재현 안 하고 그리기만)
  state_machine/state_machine/state_machine.py : get_splini_wpts() 의 인덱스 병합/clamp

읽는 법
  바닥 점/선 : 초록=전역 기하, 자홍=회피선 기하, 주황=회피선 끝점 반복(clamped)
  높이 곡선  : 흰색=전역속도, 노랑=상태배율 후, 주황=곡률한계 후, 초록 굵은선=실제 출력
  기둥(stem) : 그 점의 속도를 '최종적으로 깎은' 단계 색
               회색=제한 없음, 노랑=상태배율, 주황=곡률한계, 빨강=선제감속
  글자       : 상태 / 회피선 구간 / 배율 / 제한 비율 / 모델오차 / 스플라인 나이

토픽
  구독: /local_waypoints /global_waypoints_scaled /planner/avoidance/otwpnts
        /state /car_state/frenet/odom
  발행: /local_planner_viz/markers (MarkerArray, rviz 용)
        /local_planner_viz/summary (String, 글자와 같은 내용 — 녹화/echo/grep 용)
"""

import argparse
import csv
import math
import os
import sys

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from f110_msgs.msg import OTWpntArray, WpntArray

# state_machine_params.yaml 을 못 읽었을 때만 쓰는 최후의 기본값.
# (실제로는 아래 load_yaml_params 나 실행 중인 /state_machine 노드에서 읽어온다)
DEFAULT_PARAMS = {
    "ay_max_mps2": 5.4,
    "ax_max_mps2": 3.8,
    "ot_speed_scaling": 0.9,
    "hs_brake_v_lo": 3.0,
    "hs_brake_v_hi": 4.5,
    "hs_brake_factor": 0.6,
    "n_loc_wpnts": 70,
    "splini_ttl": 3.0,
    "splini_hyst_timer_sec": 0.4,
}
PARAM_KEYS = list(DEFAULT_PARAMS.keys())

C_GLOBAL = (0.30, 0.85, 0.35)   # 전역 레이싱라인 기하
C_SPLINE = (1.00, 0.25, 0.90)   # 회피선 기하
C_CLAMP = (1.00, 0.45, 0.05)    # 회피선 끝점 반복(clamped)
C_RECOVER = (0.30, 0.60, 1.00)  # 후진 복구 경로

# 속도를 '최종적으로 깎은' 단계별 색
LIMIT_COLORS = {
    "none": (0.80, 0.80, 0.80),
    "scale": (1.00, 0.90, 0.15),
    "ay": (1.00, 0.55, 0.00),
    "ax": (0.95, 0.15, 0.15),
}
LIMIT_ORDER = ["none", "scale", "ay", "ax"]

STAGE_STYLE = {  # (색, 선굵기)
    "glb": ((0.90, 0.90, 0.90), 0.02),
    "scaled": ((1.00, 0.90, 0.15), 0.02),
    "kappa": ((1.00, 0.55, 0.00), 0.02),
}


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def rgba(c, a=1.0):
    return ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=float(a))


def load_yaml_params(path):
    """state_machine_params.yaml 에서 재현에 필요한 값만 뽑는다."""
    with open(path, "r") as f:
        d = yaml.safe_load(f)["state_machine"]["ros__parameters"]
    return {k: d[k] for k in PARAM_KEYS if k in d}


def find_params_yaml(explicit=None):
    if explicit:
        return explicit
    here = os.path.dirname(os.path.realpath(__file__))
    cands = [
        os.path.join(here, "..", "stack_master", "config", "state_machine_params.yaml"),
        os.path.join(here, "..", "install", "stack_master", "share", "stack_master",
                     "config", "state_machine_params.yaml"),
    ]
    for c in cands:
        c = os.path.normpath(c)
        if os.path.isfile(c):
            return c
    return None


class LocalPlannerViz(Node):
    def __init__(self, args):
        super().__init__("local_planner_viz")
        self.args = args
        self.z_scale = args.z_scale
        self.min_period = 1.0 / max(args.rate, 0.1)
        self.err_tol = args.err_tol
        self.last_pub = -1e9

        # --- 파라미터 확보 (yaml -> 실행 중인 state_machine 순으로 덮어씀) ---
        self.p = dict(DEFAULT_PARAMS)
        self.param_src = "기본값(하드코딩)"
        yml = find_params_yaml(args.params_file)
        if yml:
            try:
                self.p.update(load_yaml_params(yml))
                self.param_src = f"yaml({os.path.basename(yml)})"
            except Exception as e:  # yaml 이 깨졌다고 진단 노드가 죽을 이유는 없다
                self.get_logger().warn(f"params yaml 읽기 실패({yml}): {e}")
        if args.params in ("auto", "node"):
            if self._fetch_live_params(required=(args.params == "node")):
                self.param_src = "실행 중인 /state_machine"
        self.get_logger().info(f"파라미터 출처: {self.param_src} -> {self.p}")

        # --- 상태 ---
        self.glb = None          # /global_waypoints_scaled (마지막 점 제외)
        self.wp_dist = 0.1
        self.track_length = None
        self.avoid = None          # state_machine 의 last_valid_avoidance_wpnts 재현
        self.avoid_stamp = None    # 그것을 채택한 시각 [s]
        self.avoid_rx_stamp = None # 비어있지 않은 회피선을 마지막으로 '받은' 시각 (ttl 기준)
        self.avoid_rejected = 0    # 히스테리시스로 기각한 연속 횟수 (진단용)
        self.state = "?"
        self.cur_vs = 0.0
        self.csv_w = None
        self.csv_f = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(WpntArray, "/local_waypoints", self.on_local, qos)
        self.create_subscription(OTWpntArray, "/planner/avoidance/otwpnts", self.on_avoid, qos)
        self.create_subscription(String, "/state", self.on_state, qos)
        self.create_subscription(Odometry, "/car_state/frenet/odom", self.on_frenet, qos)
        # /global_waypoints_scaled 는 sector_tuner 가 2 Hz 로 계속 재발행하므로 volatile 로 충분하다.
        # (transient_local 로 구독하면 volatile 발행자와 QoS 불일치 경고만 뜨고 아무것도 못 받는다)
        self.create_subscription(
            WpntArray, "/global_waypoints_scaled", self.on_glb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))

        self.mrk_pub = self.create_publisher(MarkerArray, "/local_planner_viz/markers", 10)
        self.txt_pub = self.create_publisher(String, "/local_planner_viz/summary", 10)

        if args.csv:
            self.csv_f = open(args.csv, "w", newline="")
            self.csv_w = csv.writer(self.csv_f)
            self.csv_w.writerow([
                "t", "state", "n", "n_spline", "n_clamped", "scale", "hs_factor",
                "v_glb_mean", "v_out_mean", "ratio",
                "pct_none", "pct_scale", "pct_ay", "pct_ax",
                "model_err_max", "geom_err_max", "splini_age"])
            self.get_logger().info(f"CSV 기록: {args.csv}")

        self.get_logger().info("local_planner_viz 시작 — /local_planner_viz/markers 를 rviz 에 추가할 것")

    # ------------------------------------------------------------------ params
    def _fetch_live_params(self, required):
        """실행 중인 state_machine 에서 현재 파라미터를 읽어온다(동적 변경 반영)."""
        from rcl_interfaces.srv import GetParameters
        cli = self.create_client(GetParameters, "/state_machine/get_parameters")
        if not cli.wait_for_service(timeout_sec=2.0):
            if required:
                self.get_logger().error("/state_machine 파라미터 서비스 없음")
            return False
        req = GetParameters.Request()
        req.names = PARAM_KEYS
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().warn("state_machine 파라미터 읽기 실패")
            return False
        got = {}
        for name, val in zip(PARAM_KEYS, fut.result().values):
            if val.type == 3:      # PARAMETER_DOUBLE
                got[name] = val.double_value
            elif val.type == 2:    # PARAMETER_INTEGER
                got[name] = val.integer_value
        if not got:
            return False
        self.p.update(got)
        return True

    # --------------------------------------------------------------- callbacks
    def on_glb(self, msg: WpntArray):
        if len(msg.wpnts) < 3:
            return
        # state_machine 과 동일하게 마지막 점(=첫 점 중복)을 뺀다
        self.glb = msg.wpnts[:-1]
        self.wp_dist = msg.wpnts[1].s_m - msg.wpnts[0].s_m or 0.1
        self.track_length = msg.wpnts[-1].s_m

    def on_avoid(self, msg: OTWpntArray):
        """state_machine 의 avoidance_cb + _check_availability_splini_wpts 재현.

        빈 배열은 무시하고(기존 회피선을 덮어쓰지 않는다), 좌/우 전환 직후
        splini_hyst_timer_sec 안의 회피선은 state_machine 이 채택하지 않으므로
        여기서도 last_valid 로 승격시키지 않는다. 그래야 그림이 실제와 같아진다.
        """
        if len(msg.wpnts) == 0:
            return
        self.avoid_rx_stamp = self._now()   # ttl 은 '받은' 시각 기준 (avoidance_cb 와 동일)
        hyst = abs(stamp_to_sec(msg.header.stamp) - stamp_to_sec(msg.last_switch_time))
        if hyst < self.p["splini_hyst_timer_sec"]:
            self.avoid_rejected += 1
            return
        self.avoid_rejected = 0
        self.avoid = msg.wpnts
        self.avoid_stamp = self._now()

    def on_state(self, msg: String):
        self.state = msg.data.split(".")[-1]  # "StateType.OVERTAKE" -> "OVERTAKE"

    def on_frenet(self, msg: Odometry):
        self.cur_vs = msg.twist.twist.linear.x

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------- model
    def _hs_factor(self):
        """states._highspeed_brake_factor 재현."""
        f = self.p["hs_brake_factor"]
        if f >= 1.0:
            return 1.0
        lo, hi = self.p["hs_brake_v_lo"], self.p["hs_brake_v_hi"]
        v = self.cur_vs
        if hi <= lo or v <= lo:
            return 1.0
        if v >= hi:
            return f
        return 1.0 + (f - 1.0) * (v - lo) / (hi - lo)

    def _splini_alive(self):
        """state_machine 의 splini_ttl_counter 재현: 마지막 '수신' 이후 ttl 초까지만 유효."""
        if self.avoid is None or self.avoid_rx_stamp is None:
            return False
        return (self._now() - self.avoid_rx_stamp) <= self.p["splini_ttl"]

    def _clamped_global_idxs(self):
        """get_splini_wpts 에서 '회피선 마지막 점이 반복 복사되는' 전역 인덱스 집합.

        회피선이 덮어야 할 인덱스 수보다 회피선 점 수가 모자라면
        splini_glob[s] = avoid[min(i, len-1)] 이 되어 끝점이 계속 복사된다.
        기하가 한 점에 뭉쳐 보이면 여기부터 의심할 것.
        """
        if not self._splini_alive() or self.track_length is None:
            return set(), None
        wd = self.wp_dist
        a = self.avoid
        i0 = int(a[0].s_m / wd + 0.5)
        i1 = int(a[-1].s_m / wd + 0.5)
        if a[-1].s_m > a[0].s_m:
            idxs = list(range(i0, i1))
        else:  # 결승선 wrap
            idxs = [int(s % (self.track_length / wd) + 0.5)
                    for s in range(i0, int((self.track_length + a[-1].s_m) / wd + 0.5))]
        clamped = {idx for k, idx in enumerate(idxs) if k > len(a) - 1}
        return clamped, (a[0].s_m, a[-1].s_m)

    def on_local(self, msg: WpntArray):
        now = self._now()
        if now - self.last_pub < self.min_period:
            return
        if self.glb is None:
            self.get_logger().warn("전역 웨이포인트(/global_waypoints_scaled) 대기 중",
                                   throttle_duration_sec=3.0)
            return
        self.last_pub = now
        loc = msg.wpnts
        if len(loc) == 0:
            self._publish(MarkerArray(), f"state={self.state}  로컬 웨이포인트 0개 (FTGONLY?)")
            return
        if self.state == "RECOVER":
            self._publish_recover(loc)
            return

        n = len(self.glb)
        wd = self.wp_dist
        idx0 = int(loc[0].s_m / wd + 0.5) % n
        idxs = [(idx0 + j) % n for j in range(len(loc))]

        xy = np.array([[w.x_m, w.y_m] for w in loc])
        gxy = np.array([[self.glb[i].x_m, self.glb[i].y_m] for i in idxs])
        geom_d = np.hypot(*(xy - gxy).T)          # 전역선과의 거리 -> 회피선 유래 판정
        from_spline = geom_d > 1e-3
        clamped_set, avoid_s = self._clamped_global_idxs()
        clamped = np.array([(i in clamped_set) and from_spline[j]
                            for j, i in enumerate(idxs)])

        v_out = np.array([w.vx_mps for w in loc])
        v_glb = np.array([self.glb[i].vx_mps for i in idxs])

        # --- states.py 재현 -------------------------------------------------
        hs = self._hs_factor()
        if self.state == "OVERTAKE":
            scale = self.p["ot_speed_scaling"] * hs
        elif self.state == "TRAILING":
            scale = hs
        else:  # GB_TRACK / TRAILING_TO_GBTRACK 는 전역 배열을 그대로 내보낸다
            scale = 1.0
        fused = self.state in ("OVERTAKE", "TRAILING")

        v_scaled = v_glb * scale
        kappa = np.abs(np.array([w.kappa_radpm for w in loc]))
        v_kappa_cap = np.where(kappa > 1e-6,
                               np.sqrt(self.p["ay_max_mps2"] / np.maximum(kappa, 1e-9)),
                               np.inf)
        v_kappa = np.minimum(v_scaled, v_kappa_cap)
        v_model = v_kappa.copy()
        ds = np.hypot(*(xy[1:] - xy[:-1]).T)
        ax = self.p["ax_max_mps2"]
        for i in range(len(v_model) - 2, -1, -1):   # 역방향 선제감속 패스
            lim = math.sqrt(v_model[i + 1] ** 2 + 2.0 * ax * ds[i])
            if v_model[i] > lim:
                v_model[i] = lim

        if not fused:
            # 회피선이 없으면 TRAILING 도 전역 배열을 그대로 내보낸다.
            # 어느 쪽인지는 실제 출력과 비교해서 오차가 작은 쪽으로 고른다.
            v_model, v_scaled, v_kappa = v_glb.copy(), v_glb.copy(), v_glb.copy()
        elif not self._splini_alive():
            err_fused = float(np.max(np.abs(v_model - v_out)))
            err_pass = float(np.max(np.abs(v_glb - v_out)))
            if err_pass < err_fused:
                v_model, v_scaled, v_kappa = v_glb.copy(), v_glb.copy(), v_glb.copy()
                scale, fused = 1.0, False

        # 각 점의 속도를 '마지막으로' 깎은 단계
        eps = 1e-3
        limiter = []
        for i in range(len(loc)):
            if v_model[i] < v_kappa[i] - eps:
                limiter.append("ax")
            elif v_kappa[i] < v_scaled[i] - eps:
                limiter.append("ay")
            elif scale < 1.0 - eps:
                limiter.append("scale")
            else:
                limiter.append("none")

        model_err = np.abs(v_model - v_out)
        stats = dict(
            n=len(loc), n_spline=int(from_spline.sum()), n_clamped=int(clamped.sum()),
            scale=scale, hs=hs, fused=fused,
            v_glb=float(v_glb.mean()), v_out=float(v_out.mean()),
            ratio=float(v_out.mean() / v_glb.mean()) if v_glb.mean() > 1e-6 else float("nan"),
            pct={k: 100.0 * limiter.count(k) / len(limiter) for k in LIMIT_ORDER},
            err=float(model_err.max()), geom=float(geom_d.max()),
            avoid_s=avoid_s, s0=loc[0].s_m,
            age=(now - self.avoid_stamp) if self.avoid_stamp else float("nan"),
            rejected=self.avoid_rejected,
        )
        txt = self._summary_text(stats)
        self._publish(self._build_markers(loc, xy, from_spline, clamped, limiter,
                                          v_glb, v_scaled, v_kappa, v_out, model_err, txt),
                      txt)
        if self.csv_w:
            self.csv_w.writerow([
                f"{now:.3f}", self.state, stats["n"], stats["n_spline"], stats["n_clamped"],
                f"{scale:.3f}", f"{hs:.3f}", f"{stats['v_glb']:.3f}", f"{stats['v_out']:.3f}",
                f"{stats['ratio']:.3f}"] +
                [f"{stats['pct'][k]:.1f}" for k in LIMIT_ORDER] +
                [f"{stats['err']:.3f}", f"{stats['geom']:.3f}", f"{stats['age']:.2f}"])
            self.csv_f.flush()   # 주행 중 Ctrl-C 로 끊겨도 그때까지 쓴 건 남게

    # -------------------------------------------------------------- 텍스트/마커
    def _summary_text(self, s):
        sp = ""
        if s["n_spline"]:
            sp = f"  s {s['avoid_s'][0]:.1f}~{s['avoid_s'][1]:.1f}" if s["avoid_s"] else ""
        lines = [
            f"[{self.state}]  v={self.cur_vs:.2f} m/s   s={s['s0']:.1f} m",
            f"기하: 회피선 {s['n_spline']}/{s['n']}{sp}"
            + (f"   ★clamped {s['n_clamped']}" if s["n_clamped"] else ""),
            f"배율: {'ot ' + format(self.p['ot_speed_scaling'], '.2f') + ' x ' if self.state == 'OVERTAKE' else ''}"
            f"hs {s['hs']:.2f} = {s['scale']:.2f}" + ("" if s["fused"] else "  (회피선 미사용: 전역 그대로)"),
            "제한: " + " | ".join(f"{k} {s['pct'][k]:.0f}%" for k in LIMIT_ORDER),
            f"속도: 전역 {s['v_glb']:.2f} -> 출력 {s['v_out']:.2f} m/s ({s['ratio']:.2f}x)",
            f"모델오차 {s['err']:.3f} m/s" + ("  ✓" if s["err"] < self.err_tol else "  ← 재현 불일치!"),
            f"스플라인 나이 {s['age']:.2f} s (ttl {self.p['splini_ttl']:.1f})"
            + (f"   히스테리시스 기각 {s['rejected']}" if s["rejected"] else ""),
        ]
        return "\n".join(lines)

    def _base(self, ns, mtype, scale, color=None, mid=0):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = float(scale)
        if color:
            m.color = rgba(color)
        m.lifetime.sec = 1
        return m

    def _build_markers(self, loc, xy, from_spline, clamped, limiter,
                       v_glb, v_scaled, v_kappa, v_out, model_err, txt):
        arr = MarkerArray()
        z0 = 0.02

        # 1) 바닥 경로: 기하의 출처를 점별 색으로
        path = self._base("1_geom_path", Marker.LINE_STRIP, 0.05)
        pts = self._base("1_geom_pts", Marker.SPHERE_LIST, 0.10)
        for j in range(len(loc)):
            c = C_CLAMP if clamped[j] else (C_SPLINE if from_spline[j] else C_GLOBAL)
            p = Point(x=float(xy[j, 0]), y=float(xy[j, 1]), z=z0)
            path.points.append(p)
            path.colors.append(rgba(c))
            pts.points.append(p)
            pts.colors.append(rgba(c))
        arr.markers += [path, pts]

        # 2) 단계별 속도 곡선 (높이 = 속도 x z_scale)
        for key, vals in (("glb", v_glb), ("scaled", v_scaled), ("kappa", v_kappa)):
            col, w = STAGE_STYLE[key]
            m = self._base(f"2_v_{key}", Marker.LINE_STRIP, w, col)
            m.points = [Point(x=float(xy[j, 0]), y=float(xy[j, 1]),
                              z=float(vals[j]) * self.z_scale) for j in range(len(loc))]
            arr.markers.append(m)
        act = self._base("2_v_out", Marker.LINE_STRIP, 0.05, (0.2, 1.0, 0.3))
        act.points = [Point(x=float(xy[j, 0]), y=float(xy[j, 1]),
                            z=float(v_out[j]) * self.z_scale) for j in range(len(loc))]
        arr.markers.append(act)

        # 3) 기둥: 그 점의 속도를 최종적으로 깎은 단계
        stems = self._base("3_limiter_stems", Marker.LINE_LIST, 0.015)
        for j in range(len(loc)):
            c = rgba(LIMIT_COLORS[limiter[j]])
            stems.points.append(Point(x=float(xy[j, 0]), y=float(xy[j, 1]), z=z0))
            stems.points.append(Point(x=float(xy[j, 0]), y=float(xy[j, 1]),
                                      z=float(v_out[j]) * self.z_scale))
            stems.colors += [c, c]
        arr.markers.append(stems)

        # 4) 회피선이 전역선에 접붙는 지점 (여기가 곧 진입/복귀 지점)
        graft = self._base("4_graft", Marker.CUBE_LIST, 0.22, (1.0, 1.0, 1.0))
        idx_sp = np.flatnonzero(from_spline)
        if len(idx_sp):
            for j in (idx_sp[0], idx_sp[-1]):
                graft.points.append(Point(x=float(xy[j, 0]), y=float(xy[j, 1]), z=z0))
        arr.markers.append(graft)

        # 5) 모델이 실제 출력과 어긋난 점 — 재현이 틀렸거나 스택이 바뀐 곳
        bad = self._base("5_model_mismatch", Marker.SPHERE_LIST, 0.18, (0.1, 0.9, 1.0))
        for j in np.flatnonzero(model_err > self.err_tol):
            bad.points.append(Point(x=float(xy[j, 0]), y=float(xy[j, 1]),
                                    z=float(v_out[j]) * self.z_scale))
        arr.markers.append(bad)

        # 6) 글자
        t = self._base("6_info", Marker.TEXT_VIEW_FACING, 0.22, (1.0, 1.0, 1.0))
        t.pose.position.x = float(xy[0, 0])
        t.pose.position.y = float(xy[0, 1])
        t.pose.position.z = 2.2
        t.text = txt
        arr.markers.append(t)
        return arr

    def _publish_recover(self, loc):
        """RECOVER 는 중심선 후진이라 속도 모델이 없다. 경로와 방향만 그린다."""
        arr = MarkerArray()
        path = self._base("1_geom_path", Marker.LINE_STRIP, 0.06, C_RECOVER)
        pts = self._base("1_geom_pts", Marker.SPHERE_LIST, 0.12, C_RECOVER)
        for w in loc:
            p = Point(x=float(w.x_m), y=float(w.y_m), z=0.02)
            path.points.append(p)
            pts.points.append(p)
        arr.markers += [path, pts]
        v = [w.vx_mps for w in loc]
        txt = (f"[RECOVER]  중심선 후진 {len(loc)}점\n"
               f"속도 {min(v):.2f}~{max(v):.2f} m/s (음수=후진)\n"
               f"전역/회피 속도 모델 적용 안 됨")
        t = self._base("6_info", Marker.TEXT_VIEW_FACING, 0.22, C_RECOVER)
        t.pose.position.x = float(loc[0].x_m)
        t.pose.position.y = float(loc[0].y_m)
        t.pose.position.z = 2.2
        t.text = txt
        arr.markers.append(t)
        self._publish(arr, txt)
        if self.csv_w:  # 재현하지 않는 상태도 '언제 얼마나 있었는지'는 남긴다
            self.csv_w.writerow([f"{self._now():.3f}", "RECOVER", len(loc), 0, 0] + [""] * 12)
            self.csv_f.flush()

    def _publish(self, arr, txt):
        self.mrk_pub.publish(arr)
        self.txt_pub.publish(String(data=txt))

    def destroy_node(self):
        if self.csv_f:
            self.csv_f.close()
        return super().destroy_node()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rate", type=float, default=10.0, help="마커 발행 상한 [Hz] (기본 10)")
    ap.add_argument("--z-scale", type=float, default=0.3,
                    help="속도 1 m/s 를 몇 m 높이로 그릴지 (기본 0.3)")
    ap.add_argument("--params", choices=["auto", "node", "yaml"], default="auto",
                    help="파라미터 출처: auto=실행 중이면 노드에서, 아니면 yaml (기본)")
    ap.add_argument("--params-file", default=None, help="state_machine_params.yaml 경로 직접 지정")
    ap.add_argument("--err-tol", type=float, default=0.1,
                    help="재현 오차 경고 기준 [m/s] (기본 0.1). 고속감속 배율은 자차속도 "
                         "샘플링 시점 차이로 0.1 m/s 안팎의 오차가 정상적으로 난다")
    ap.add_argument("--csv", default=None, help="사이클별 요약을 CSV 로 기록")
    ap.add_argument("--sim-time", action="store_true", help="bag 재생에 붙을 때 (use_sim_time)")
    args, ros_args = ap.parse_known_args()

    # use_sim_time 은 파라미터 오버라이드로 넘긴다 (bag 재생 시 /clock 을 따라가게)
    if args.sim_time:
        if "--ros-args" not in ros_args:
            ros_args = ros_args + ["--ros-args"]
        ros_args = ros_args + ["-p", "use_sim_time:=true"]
    rclpy.init(args=sys.argv[:1] + ros_args)
    node = LocalPlannerViz(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
