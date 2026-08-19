#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import numpy as np

import rclpy
from rclpy.node import Node
from f110_msgs.msg import WpntArray  # 각 waypoint에 x_m, y_m 존재한다고 가정


class MPCCJsonTools(Node):
    def __init__(self):
        super().__init__('mpcc_json_tools')

        # ---------------- Parameters ----------------
        self.declare_parameter('subscribe_topic', '/centerline_waypoints')  # /centerline_waypoints 또는 /global_waypoints
        self.declare_parameter('r_in', 1.0)                      # 트랙 우측(안쪽) 폭 [m]
        self.declare_parameter('r_out', 1.0)                     # 트랙 좌측(바깥) 폭 [m]

        self.declare_parameter('output_track_path', 'track.json')
        self.declare_parameter('output_bounds_path', 'bounds.json')
        self.declare_parameter('output_normalization_path', 'normalization.json')

        self.declare_parameter('map', '')                        # 예: hall -> *_hall.json
        self.declare_parameter('write_once', True)               # 첫 메시지에서만 저장
        self.declare_parameter('close_loop', True)               # 루프 강제 폐쇄(끝점=시작점)
        self.declare_parameter('flip_inner_outer', False)        # 내/외 스왑 필요시
        self.declare_parameter('epsilon_norm', 1e-12)            # 법선 정규화 안정화용

        # -------------- Read params --------------
        self.topic = self.get_parameter('subscribe_topic').get_parameter_value().string_value
        self.r_in = float(self.get_parameter('r_in').value)
        self.r_out = float(self.get_parameter('r_out').value)

        self.map_name = self.get_parameter('map').get_parameter_value().string_value.strip()
        self.map_name = self.map_name.replace("/", "_").replace(" ", "_")

        raw_track = self.get_parameter('output_track_path').get_parameter_value().string_value
        raw_bounds = self.get_parameter('output_bounds_path').get_parameter_value().string_value
        raw_norm = self.get_parameter('output_normalization_path').get_parameter_value().string_value

        self.output_track_path = self._with_map_suffix(raw_track)
        self.output_bounds_path = self._with_map_suffix(raw_bounds)
        self.output_norm_path = self._with_map_suffix(raw_norm)

        self.write_once = bool(self.get_parameter('write_once').value)
        self.close_loop = bool(self.get_parameter('close_loop').value)
        self.flip_io = bool(self.get_parameter('flip_inner_outer').value)
        self.eps = float(self.get_parameter('epsilon_norm').value)

        self._wrote = False
        self.sub = self.create_subscription(WpntArray, self.topic, self.cb_wpnts, 10)

        self.get_logger().info(
            f"[mpcc_json_tools] topic='{self.topic}', r_in={self.r_in} m, r_out={self.r_out} m, map='{self.map_name}', "
            f"track='{self.output_track_path}', bounds='{self.output_bounds_path}', norm='{self.output_norm_path}'"
        )

    # ---------- helpers ----------
    def _with_map_suffix(self, path: str) -> str:

        if not self.map_name:
            return path

        base = os.path.basename(path)
        directory = os.path.dirname(path)
        name, ext = os.path.splitext(base)

        new_base = f"{name}_{self.map_name}{ext}"
        return os.path.join(directory, new_base)

    @staticmethod
    def _finite_diff_closed(xy: np.ndarray) -> np.ndarray:
        """중심차분(폐루프)으로 접선 벡터 계산"""
        xm1 = np.roll(xy, 1, axis=0)
        xp1 = np.roll(xy, -1, axis=0)
        tan = (xp1 - xm1) * 0.5
        return tan

    @staticmethod
    def _arc_length_s(xy: np.ndarray) -> np.ndarray:
        """centerline 누적 호길이 s 계산"""
        d = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(d)])
        return s

    @staticmethod
    def _ensure_dir(path: str):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

    # ---------- main ----------
    def cb_wpnts(self, msg: WpntArray):
        if self.write_once and self._wrote:
            return

        if len(msg.wpnts) < 3:
            self.get_logger().warn("waypoints < 3 → 스킵")
            return

        X = np.array([w.x_m for w in msg.wpnts], dtype=np.float64)
        Y = np.array([w.y_m for w in msg.wpnts], dtype=np.float64)
        xy = np.stack([X, Y], axis=1)

        if self.close_loop:
            if np.linalg.norm(xy[-1] - xy[0]) > 1e-6:
                xy = np.vstack([xy, xy[0]])

        # 접선/법선
        tan = self._finite_diff_closed(xy)
        tnorm = np.linalg.norm(tan, axis=1, keepdims=True)
        t_hat = tan / (tnorm + self.eps)                          # 단위 접선
        n_hat = np.stack([-t_hat[:, 1], t_hat[:, 0]], axis=1)     # 좌측 법선 (-ty, tx)

        # 내/외곽선 (규약: inner = center - r_in*n, outer = center + r_out*n)
        inner = xy - self.r_in * n_hat
        outer = xy + self.r_out * n_hat
        if self.flip_io:
            inner, outer = outer, inner

        # ---------------- 1) track.json ----------------
        track = {
            "X": xy[:, 0].tolist(),
            "Y": xy[:, 1].tolist(),
            "X_i": inner[:, 0].tolist(),
            "Y_i": inner[:, 1].tolist(),
            "X_o": outer[:, 0].tolist(),
            "Y_o": outer[:, 1].tolist()
        }

        # ---------------- 2) bounds.json ----------------
        all_bd = np.vstack([inner, outer])
        Xl = float(np.min(all_bd[:, 0]))
        Xu = float(np.max(all_bd[:, 0]))
        Yl = float(np.min(all_bd[:, 1]))
        Yu = float(np.max(all_bd[:, 1]))

        # s는 centerline 누적 호길이
        s_arr = self._arc_length_s(xy)
        sl = float(np.min(s_arr))
        su = float(np.max(s_arr))

        bounds = {
            "Xl": Xl,
            "Yl": Yl,
            "phil": -10.0,
            "vxl": 0.05,
            "vyl": -3.0,
            "rl": -5.0,
            "sl": sl,
            "Dl": -1.0,
            "deltal": -0.46,
            "vsl": 0.0,

            "Xu": Xu,
            "Yu": Yu,
            "phiu": 10.0,
            "vxu": 10.0,
            "vyu": 3.0,
            "ru": 5.0,
            "su": su,
            "Du": 1.0,
            "deltau": 0.46,
            "vsu": 10.0,

            "dDl": -10.0,
            "dDeltal": -5.0,
            "dVsl": -100.0,

            "dDu": 10.0,
            "dDeltau": 5.0,
            "dVsu": 100.0
        }

        # ---------------- 3) normalization.json ----------------
        X_norm = float(max(abs(bounds["Xl"]), abs(bounds["Xu"])))
        Y_norm = float(max(abs(bounds["Yl"]), abs(bounds["Yu"])))
        s_norm = float(max(abs(bounds["sl"]), abs(bounds["su"])))

        normalization = {
            "X": X_norm,
            "Y": Y_norm,
            "phi": 1.0,
            "vx": 10.0,
            "vy": 3.0,
            "r": 5.0,
            "s": s_norm,
            "D": 1.0,
            "delta": 0.46,
            "vs": 10.0,

            "dD": 10.0,
            "dDelta": 5.0,
            "dVs": 100.0
        }

        # ---------------- save ----------------
        try:
            # track.json
            self._ensure_dir(self.output_track_path)
            with open(self.output_track_path, "w") as f:
                json.dump(track, f, indent=2)

            # bounds.json
            self._ensure_dir(self.output_bounds_path)
            with open(self.output_bounds_path, "w") as f:
                json.dump(bounds, f, indent=2)

            # normalization.json
            self._ensure_dir(self.output_norm_path)
            with open(self.output_norm_path, "w") as f:
                json.dump(normalization, f, indent=2)

            self.get_logger().info(
                f"[mpcc_json_tools] saved track({len(xy)} pts), bounds, normalization"
            )
            self.get_logger().info(
                f"[auto bounds] Xl={Xl:.3f}, Xu={Xu:.3f}, Yl={Yl:.3f}, Yu={Yu:.3f}, sl={sl:.3f}, su={su:.3f}"
            )
            self.get_logger().info(
                f"[auto norm] X={X_norm:.3f}, Y={Y_norm:.3f}, s={s_norm:.3f}"
            )

            self._wrote = True

        except Exception as e:
            self.get_logger().error(f"JSON 저장 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MPCCJsonTools()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
