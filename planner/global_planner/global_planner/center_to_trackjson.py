#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import numpy as np
from typing import Tuple

import rclpy
from rclpy.node import Node
from f110_msgs.msg import WpntArray  # 각 waypoint에 x_m, y_m 존재한다고 가정

class CenterToTrackJson(Node):
    """
    Subscribe /center_waypoints (또는 파라미터로 지정한 토픽) -> MPCC track.json 생성
    출력 키: X, Y, X_i, Y_i, X_o, Y_o
    """

    def __init__(self):
        super().__init__('center_to_trackjson')

        # Parameters
        self.declare_parameter('subscribe_topic', '/centerline_waypoints')   # 또는 /centerline_waypoints
        self.declare_parameter('r_in', 1.0)                              # 트랙 우측(안쪽) 폭 [m]
        self.declare_parameter('r_out', 1.0)                             # 트랙 좌측(바깥) 폭 [m]
        self.declare_parameter('output_path', 'track_1026.json')         # 저장 파일 경로
        self.declare_parameter('write_once', True)                       # 첫 메시지에서만 저장
        self.declare_parameter('close_loop', True)                       # 루프 강제 폐쇄
        self.declare_parameter('flip_inner_outer', False)                # 내/외 스왑 필요시
        self.declare_parameter('epsilon_norm', 1e-12)                    # 법선 정규화 안정화용

        self.topic = self.get_parameter('subscribe_topic').get_parameter_value().string_value
        self.r_in = float(self.get_parameter('r_in').value)
        self.r_out = float(self.get_parameter('r_out').value)
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.write_once = bool(self.get_parameter('write_once').value)
        self.close_loop = bool(self.get_parameter('close_loop').value)
        self.flip_io = bool(self.get_parameter('flip_inner_outer').value)
        self.eps = float(self.get_parameter('epsilon_norm').value)

        self._wrote = False
        self.sub = self.create_subscription(WpntArray, self.topic, self.cb_wpnts, 10)

        self.get_logger().info(
            f"[center_to_trackjson] topic='{self.topic}', r_in={self.r_in} m, r_out={self.r_out} m, output='{self.output_path}'"
        )

    # ---------- helpers ----------
    @staticmethod
    def _finite_diff_closed(xy: np.ndarray) -> np.ndarray:
        """중심차분(폐루프)으로 접선 벡터 계산"""
        xm1 = np.roll(xy, 1, axis=0)
        xp1 = np.roll(xy, -1, axis=0)
        tan = (xp1 - xm1) * 0.5
        return tan

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

        # 폐루프 보정
        if self.close_loop:
            if np.linalg.norm(xy[-1] - xy[0]) > 1e-6:
                xy = np.vstack([xy, xy[0]])

        # 접선/법선
        tan = self._finite_diff_closed(xy)
        tnorm = np.linalg.norm(tan, axis=1, keepdims=True)
        t_hat = tan / (tnorm + self.eps)                     # 단위 접선
        n_hat = np.stack([-t_hat[:, 1], t_hat[:, 0]], axis=1)  # 좌측 법선 (-ty, tx)

        # 내/외곽선 (C++ Constraints와 동일 규약)
        inner = xy - self.r_in * n_hat       # 안쪽(우측) = center - r_in * n_hat
        outer = xy + self.r_out * n_hat      # 바깥(좌측) = center + r_out * n_hat
        if self.flip_io:
            inner, outer = outer, inner

        # MPCC에서 요구하는 키 이름에 맞춰 구성
        track = {
            "X": xy[:, 0].tolist(),
            "Y": xy[:, 1].tolist(),
            "X_i": inner[:, 0].tolist(),
            "Y_i": inner[:, 1].tolist(),
            "X_o": outer[:, 0].tolist(),
            "Y_o": outer[:, 1].tolist()
        }

        # 저장
        try:
            os.makedirs(os.path.dirname(self.output_path) or ".", exist_ok=True)
            with open(self.output_path, "w") as f:
                json.dump(track, f, indent=2)
            self.get_logger().info(
                f"[center_to_trackjson] saved {len(xy)} pts to {self.output_path}"
            )
            self._wrote = True
        except Exception as e:
            self.get_logger().error(f"JSON 저장 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CenterToTrackJson()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
