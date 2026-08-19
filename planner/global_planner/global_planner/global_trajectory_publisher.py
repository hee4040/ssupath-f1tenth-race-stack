#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from f110_msgs.msg import WpntArray, LtplWpntArray
from visualization_msgs.msg import MarkerArray

# the `.` tells Python to look in the current directory at runtime
from .readwrite_global_waypoints import read_global_waypoints, read_ltpl_waypoints
from .origin_map_bounds import compute_origin_bounds

class GlobalRepublisher(Node):
    """
    Node for publishing the global waypoints/markers and track bounds markers frequently after they have been calculated
    """
    def __init__(self):
        super().__init__('global_republisher_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.glb_markers = None
        self.glb_wpnts = None
        self.track_bounds = None
        self.ltpl_wpnts = None

        self.create_subscription(WpntArray, '/global_waypoints', self.glb_wpnts_cb, 10)
        self.create_subscription(MarkerArray, '/global_waypoints/markers', self.glb_markers_cb, 10)
        self.create_subscription(MarkerArray, '/trackbounds/markers', self.bounds_cb, 10)

        self.glb_wpnts_pub = self.create_publisher(WpntArray, 'global_waypoints', 10)
        self.glb_markers_pub = self.create_publisher(MarkerArray, 'global_waypoints/markers', 10)
        self.vis_track_bnds = self.create_publisher(MarkerArray, 'trackbounds/markers', 10)

        # shortest_path
        self.glb_sp_markers = None
        self.glb_sp_wpnts = None
        self.create_subscription(WpntArray, '/global_waypoints/shortest_path', self.glb_sp_wpnts_cb, 10)
        # self.create_subscription(MarkerArray, '/global_waypoints/shortest_path/markers', self.glb_sp_markers_cb, 10)
        self.glb_sp_wpnts_pub = self.create_publisher(WpntArray, 'global_waypoints/shortest_path', 10)
        # self.glb_sp_markers_pub = self.create_publisher(MarkerArray, 'global_waypoints/shortest_path/markers', 10)

        # centerline
        self.centerline_wpnts = None
        self.centerline_markers = None
        self.create_subscription(WpntArray, '/centerline_waypoints', self.centerline_wpnt_cb, 10)
        # self.create_subscription(MarkerArray, '/centerline_waypoints/markers', self.centerline_markers_cb, 10)
        self.centerline_wpnts_pub = self.create_publisher(WpntArray, '/centerline_waypoints', 10)
        # self.centerline_markers_pub = self.create_publisher(MarkerArray, '/centerline_waypoints/markers', 10)

        # 원본 맵 기준 트랙 경계 (장애물 탐지 전용)
        # x/y/psi/s 는 /global_waypoints 와 완전히 동일하고 d_left/d_right 만
        # 원본 맵(<map>_origin.png)의 실제 벽 위치로 보정한 사본이다.
        # cluster_to_obstacle 이 "트랙 안이냐 벽이냐" 를 판정할 때 쓴다.
        # 자세한 이유는 origin_map_bounds.py 참고.
        self.glb_wpnts_orig_bounds = None
        self.glb_wpnts_orig_bounds_pub = self.create_publisher(
            WpntArray, 'global_waypoints/original_bounds', 10)

        # map infos
        self.map_infos = None
        self.create_subscription(String, '/map_infos', self.map_info_cb, 10)
        self.map_info_pub = self.create_publisher(String, 'map_infos', 10)

        # [jimin] ltpl
        self.map_info_ltpl_pub = self.create_publisher(String, '/map_infos_ltpl', 10)
        self.ltpl_wpnts_pub = self.create_publisher(LtplWpntArray, '/ltpl_waypoints', 10)

        # self.est_lap_time = None
        # self.create_subscription(Float32, 'estimated_lap_time', self.est_lap_time_cb, 10)
        # self.est_lap_time_pub = self.create_publisher(Float32, 'estimated_lap_time', 10)

        # graph lattice 
        # self.graph_lattice = None
        # self.create_subscription(MarkerArray, '/lattice_viz', self.lattice_cb, 10)
        # self.lattice_pub = self.create_publisher(MarkerArray, '/lattice_viz', 10)

        # Read info from json file if it is provided, so everything is always published
        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        self.map_path = map_path
        self._orig_bounds_sig = None
        if map_path:
            self.get_logger().info(f"Reading parameters from {map_path}")
            try:
                (
                    self.map_infos, self.est_lap_time, self.centerline_markers,
                    self.centerline_wpnts, self.glb_markers, self.glb_wpnts,
                    self.glb_sp_markers, self.glb_sp_wpnts, self.track_bounds
                ) = read_global_waypoints(map_dir=map_path)

                self.map_infos_ltpl, self.ltpl_wpnts = read_ltpl_waypoints(map_dir=map_path)
                self.build_origin_bounds(map_path)
                # 연산 감소를 위해 마커는 처음 파일을 읽었을 때 한 번만 퍼블리시
                self.pub_marker_once()

            except FileNotFoundError:
                self.get_logger().warn(f"{map_path} param not found. Not publishing")
        else:
            self.get_logger().info(f"global_trajectory_publisher did not find any map_path param {map_path}")

        # # Publish at 2 Hz
        # self.create_timer(0.5, self.global_republisher)


        # 0820 수정
        self.create_timer(5.0, self.global_republisher)
        

    def build_origin_bounds(self, map_path):
        """원본 맵(<map>_origin.png) 기준으로 d_left/d_right 를 보정한 사본을 만든다.

        원본 png 가 없거나 계산이 실패하면 편집 맵 웨이포인트를 그대로 쓴다.
        즉 이 기능이 죽어도 동작은 예전과 같아지고, 구독자(cluster_to_obstacle)가
        경계를 못 받아 클러스터를 전부 버리는 일은 생기지 않는다.
        """
        if self.glb_wpnts is None:
            return
        self._orig_bounds_sig = self._wpnts_signature(self.glb_wpnts)
        try:
            adjusted, reason = compute_origin_bounds(map_path, self.glb_wpnts,
                                                     logger=self.get_logger())
        except Exception as e:  # noqa: BLE001 - 어떤 실패든 편집 맵으로 폴백
            adjusted, reason = None, f'unexpected error: {e}'

        if adjusted is not None:
            self.glb_wpnts_orig_bounds = adjusted
        else:
            self.glb_wpnts_orig_bounds = self.glb_wpnts
            self.get_logger().warn(
                f"원본 맵 경계를 쓰지 못해 편집 맵 경계로 폴백한다 ({reason})")

    @staticmethod
    def _wpnts_signature(msg):
        """웨이포인트가 실제로 바뀌었는지만 싸게 판별하기 위한 서명."""
        w0, wl = msg.wpnts[0], msg.wpnts[-1]
        return (len(msg.wpnts), wl.s_m, w0.x_m, w0.y_m, w0.d_left, w0.d_right)

    def glb_wpnts_cb(self, msg):
        self.glb_wpnts = msg
        track_length = msg.wpnts[-1].s_m
        gb_len = rclpy.Parameter('track_length', rclpy.Parameter.Type.DOUBLE, track_length)
        self.set_parameters([gb_len])

        # global_planner 가 새 궤적을 내보냈으면 경계도 다시 보정한다.
        # 같은 궤적이 반복 발행될 때는 서명이 같아서 다시 계산하지 않는다.
        if self.map_path and self._wpnts_signature(msg) != self._orig_bounds_sig:
            self.build_origin_bounds(self.map_path)

    def glb_markers_cb(self, msg):
        self.glb_markers = msg

    def glb_sp_wpnts_cb(self, msg):
        self.glb_sp_wpnts = msg

    def glb_sp_markers_cb(self, msg):
        self.glb_sp_markers = msg

    def centerline_wpnt_cb(self, msg):
        self.centerline_wpnts = msg

    def centerline_markers_cb(self, msg):
        self.centerline_markers = msg

    def bounds_cb(self, msg):
        self.track_bounds = msg

    def map_info_cb(self, msg):
        self.map_infos = msg

    def est_lap_time_cb(self, msg):
        self.est_lap_time = msg

    def lattice_cb(self, msg):
        self.graph_lattice = msg

    # def global_republisher(self):

    #     if self.glb_wpnts is not None and self.glb_markers is not None:
    #         self.glb_wpnts_pub.publish(self.glb_wpnts)
    #         self.glb_markers_pub.publish(self.glb_markers)
    #     if self.glb_sp_wpnts is not None and self.glb_sp_markers is not None:
    #         self.glb_sp_wpnts_pub.publish(self.glb_sp_wpnts)
    #         self.glb_sp_markers_pub.publish(self.glb_sp_markers)
    #     if self.centerline_wpnts is not None and self.centerline_markers is not None:
    #         self.centerline_wpnts_pub.publish(self.centerline_wpnts)
    #         self.centerline_markers_pub.publish(self.centerline_markers)
    #     if self.track_bounds is not None:
    #         self.vis_track_bnds.publish(self.track_bounds)
    #     if self.map_infos is not None:
    #         self.map_info_pub.publish(self.map_infos)
    #     if self.est_lap_time is not None:
    #         self.est_lap_time_pub.publish(self.est_lap_time)
    #     if self.graph_lattice is not None:
    #         self.lattice_pub.publish(self.graph_lattice)

    def global_republisher(self):

        # 원본 맵 경계를 먼저 내보낸다. cluster_to_obstacle 이 폴백(/global_waypoints)을
        # 잡기 전에 이쪽이 먼저 도착하게 해서 불필요한 폴백 경고를 줄인다.
        if self.glb_wpnts_orig_bounds is not None:
            self.glb_wpnts_orig_bounds_pub.publish(self.glb_wpnts_orig_bounds)

        if self.glb_wpnts is not None:
            self.glb_wpnts_pub.publish(self.glb_wpnts)

        if self.glb_sp_wpnts is not None:
            self.glb_sp_wpnts_pub.publish(self.glb_sp_wpnts)

        if self.centerline_wpnts is not None:
            self.centerline_wpnts_pub.publish(self.centerline_wpnts)

        if self.ltpl_wpnts is not None:
            self.ltpl_wpnts_pub.publish(self.ltpl_wpnts)

    def pub_marker_once(self):
        if self.glb_markers is not None:
            self.glb_markers_pub.publish(self.glb_markers)

        # if self.glb_sp_markers is not None:
            # self.glb_sp_markers_pub.publish(self.glb_sp_markers)

        # if self.centerline_markers is not None:
            # self.centerline_markers_pub.publish(self.centerline_markers)

        if self.track_bounds is not None:
            self.vis_track_bnds.publish(self.track_bounds)

def main(args=None):
    rclpy.init(args=args)
    republisher = GlobalRepublisher()
    rclpy.spin(republisher)
    republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
