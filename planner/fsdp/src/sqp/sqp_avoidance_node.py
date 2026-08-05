#!/usr/bin/env python3
import time
import traceback
from fsdp import ros_compat as rospy
import numpy as np
from nav_msgs.msg import Odometry
from f110_msgs.msg import Wpnt, WpntArray, Obstacle, ObstacleArray, OTWpntArray, OpponentTrajectory, OppWpnt
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32MultiArray, Float32
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from frenet_conversion.frenet_converter import FrenetConverter
from std_msgs.msg import Bool
from copy import deepcopy
from common.differential_flatness import PolynomialPath
from common.qp_fit import QPFit
from common.converter_utils import InitialRefSpline
from geometry_msgs.msg import Point
from mpc.mpc_tracking_controller_ca import MPC_Tracking_Controller, CodeTimer
from mpc.mpc_builder_loader import load_mpc_builder
from scipy.interpolate import CubicSpline
import os
from datetime import datetime
try:
    mpc_builder = load_mpc_builder()
    rospy.loginfo("[SQP_Node] Successfully imported C++ mpc_builder for Spline Fitting.")
except ImportError:
    rospy.logerr("[SQP_Node] Failed to import mpc_builder! Make sure it is compiled.")




class SQPAvoidanceNode:
    """
    This class implements a ROS node that creates a overtaking trajectory around osbtacles and opponents.

    It subscribes to the following topics:
        - `/perception/obstacles`: Subscribes to the obstacle array.
        - `/collision_prediction/obstacles`: Subscribes to the predicted obstacle array (ROCs).
        - `/car_state/frenet/odom`: Subscribes to the car state in Frenet coordinates.
        - `/global_waypoints`: Subscribes to the global waypoints.
        - `/global_waypoints_scaled`: Subscribes to the scaled global waypoints.
        - `/global_waypoints_updated`: Subscribes to the updated global waypoints.
        - `/local_waypoints`: Subscribes to the local waypoints.
        - `/opponent_waypoints`: Subscribes to the opponent waypoints.
        - `/ot_section_check`: Subscribes to the overtaking section check.
        - `/dynamic_sqp_tuner_node/parameter_updates`: Subscribes to the dynamic reconfigure updates.

    The node publishes the following topics:
        - `/planner/avoidance/markers_sqp`: Publishes the markers for the avoidance trajectory.
        - `/planner/avoidance/otwpnts`: Publishes the overtaking waypoints.
        - `/planner/avoidance/merger`: Publishes the merger region of the overtaking trajectory.
        - `/planner/pspliner_sqp/latency`: Publishes the latency of the SQP solver. (Only if measure is set to True)
    """

    def __init__(self):
        # Initialize node
        rospy.init_node('sqp_avoidance_node')
        self.rate = rospy.Rate(20)

        # Params
        self.frenet_state = Odometry()
        self.cart_state = Odometry()
        self.local_wpnts = None
        self.lookahead = 15
        self.past_avoidance_d = []
        # Scaled waypoints params
        self.scaled_wpnts = None
        self.scaled_wpnts_msg = WpntArray()
        self.scaled_vmax = None
        self.scaled_max_idx = None
        self.scaled_max_s = None
        self.scaled_delta_s = None
        # Updated waypoints params
        self.wpnts_updated = None
        self.max_s_updated = None
        self.max_idx_updated = None
        # Obstalces params
        self.obs = ObstacleArray()
        self.obs_perception = ObstacleArray()
        self.obs_predict = ObstacleArray()
        self.obs_downsampled_half_width = np.array([])
        self.corridor_feasible = True
        # Opponent waypoint params
        self.opponent_waypoints = OpponentTrajectory()
        self.max_opp_idx = None
        self.opponent_wpnts_sm = None
        # OT params
        self.last_ot_side = ""
        self.ot_section_check = False
        # Solver params
        self.min_radius = 0.05  # wheelbase / np.tan(max_steering)
        self.max_kappa = 1/self.min_radius
        # 실측 차폭 (2026-07-31 측정). graph_planner 의 offline_params.yaml veh_width 와
        # 반드시 같은 값이어야 한다. 궤적의 d 는 차량 '중심'이므로 장애물/벽 여유는
        # 전부 이 값의 절반(half_width)으로 계산된다.
        # 아래 값은 dyn_param_cb 에서 파라미터로 덮인다.
        self.width_car = 0.28
        self.min_evasion_dist = 0.05
        # 좁은 코리도어에서 MPC 마진을 여기까지 깎아 통과를 시도한다 (0 이면 차체가 벽에 닿음)
        self.min_mpc_margin = 0.02
        self.avoidance_resolution = 20
        self.back_to_raceline_before = 5
        self.back_to_raceline_after = 5
        self.obs_traj_tresh = 2
        self.merge_speed_factor = 1.5  # Dynamic merge distance = max(5, v * factor)

        # Dynamic sovler params
        self.down_sampled_delta_s = None
        self.global_traj_kappas = None

        # ROS Parameters
        self.opponent_traj_topic = '/opponent_trajectory'
        self.measure = rospy.get_param("/measure", False)

        # Dynamic reconf params
        self.avoid_static_obs = True

        self.converter = None
        self.global_waypoints = None

        self.initial_ref_spline = None

        self.qp_fit = QPFit()
        self.qp_fit_poly_x = None
        self.qp_fit_poly_y = None
        self.half_width = self.width_car / 2.0   # dyn_param_cb 에서 파라미터 값으로 갱신된다

        # Subscribers
        rospy.Subscriber("/perception/obstacles", ObstacleArray, self.obs_perception_cb)
        rospy.Subscriber("/collision_prediction/obstacles", ObstacleArray, self.obs_prediction_cb)
        rospy.Subscriber("/car_state/frenet/odom", Odometry, self.state_cb)
        rospy.Subscriber("/global_waypoints_scaled", WpntArray, self.scaled_wpnts_cb)
        rospy.Subscriber("/local_waypoints", WpntArray, self.local_wpnts_cb)
        rospy.Subscriber("/global_waypoints", WpntArray, self.gb_cb)
        rospy.Subscriber("/global_waypoints_updated", WpntArray, self.updated_wpnts_cb)
        rospy.Subscriber(self.opponent_traj_topic, OpponentTrajectory, self.opponent_trajectory_cb)
        rospy.Subscriber("/ot_section_check", Bool, self.ot_sections_check_cb)
        rospy.Subscriber("/car_state/odom", Odometry, self.cart_state_cb)
        # Publishers
        self.mrks_pub = rospy.Publisher("/planner/avoidance/markers_sqp", MarkerArray, queue_size=10)
        self.df_mrks_pub = rospy.Publisher("/planner/avoidance/df_markers", MarkerArray, queue_size=10)
        self.mpc_mrks_pub = rospy.Publisher("/planner/avoidance/mpc_markers", MarkerArray, queue_size=10)
        self.ob_line_pub = rospy.Publisher('/planner/avoidance/ob_line_marker', MarkerArray, queue_size=10)
        self.raw_mrks_pub = rospy.Publisher("/planner/avoidance/raw_traj_markers", MarkerArray, queue_size=10)
        self.evasion_pub = rospy.Publisher("/planner/avoidance/otwpnts", OTWpntArray, queue_size=10)
        self.merger_pub = rospy.Publisher("/planner/avoidance/merger", Float32MultiArray, queue_size=10)
        if self.measure:
            self.measure_pub = rospy.Publisher("/planner/pspliner_sqp/latency", Float32, queue_size=10)

        # ROS1 dynamic_reconfigure 서버가 없으므로 파라미터를 한 번 읽어준다.
        # (이게 없으면 evasion_dist / spline_bound_mindist 가 정의되지 않아 첫 회피 계산에서 죽는다)
        self.dyn_param_cb(None)

        self.converter = self.initialize_converter()
        self.initial_ref_spline = self.initialize_spline()
        self.mpc_controller = MPC_Tracking_Controller()
        # MPC 의 CA 제약도 같은 차폭을 써야 한다 (기본값이 따로 0.28 로 박혀 있다)
        self.mpc_controller.half_width = self.half_width
        # 좁은 코리도어에서 프레임 단위로 완화했다가 되돌리기 위한 기준값
        self.nominal_mpc_margin = self.mpc_controller.margin
        rospy.wait_for_message("/global_waypoints", WpntArray)
        self.mpc_controller.update_vehicle_Lmodel(self.s_ref, self.kapparef)
        rospy.loginfo("[FSDP] initial vehicle linear model!")

    ### Callbacks ###
    def obs_perception_cb(self, data: ObstacleArray):
        self.obs_perception = ObstacleArray(header=data.header)
        self.obs_perception.obstacles = [obs for obs in data.obstacles if obs.is_static == True]
        self._merge_obstacles(data.header)

    def obs_prediction_cb(self, data: ObstacleArray):
        self.obs_predict = ObstacleArray(header=data.header)
        self.obs_predict.obstacles = list(data.obstacles)
        self._merge_obstacles(data.header)

    def _merge_obstacles(self, header):
        """정적 장애물(perception)과 충돌영역 예측(ROC)을 합쳐 self.obs 를 새로 만든다.

        이전 구현은 self.obs = self.obs_predict 로 앨리어싱한 뒤 obstacles 에 += 를 했기 때문에
        콜백이 돌 때마다 같은 정적 장애물이 obs_predict 안에 계속 누적됐다.
        매번 새 ObstacleArray 를 만들어 누적을 끊는다."""
        merged = ObstacleArray(header=header)
        merged.obstacles = list(self.obs_predict.obstacles)
        if self.avoid_static_obs == True:
            merged.obstacles = merged.obstacles + list(self.obs_perception.obstacles)
        self.obs = merged

    def state_cb(self, data: Odometry):
        self.frenet_state = data

    def cart_state_cb(self, data: Odometry):
        self.cart_state = data
    
    def gb_cb(self, data: WpntArray):
        self.global_waypoints = np.array([[wpnt.x_m, wpnt.y_m, wpnt.psi_rad] for wpnt in data.wpnts]) 
        self.kapparef = np.array([x.kappa_radpm for x in data.wpnts])
        self.s_ref = np.array([x.s_m for x in data.wpnts])
        self.d_left_ref = np.array([x.d_left for x in data.wpnts])
        self.d_right_ref = np.array([x.d_right for x in data.wpnts]) 

    # Everything is refered to the SCALED global waypoints
    def scaled_wpnts_cb(self, data: WpntArray):
        self.scaled_wpnts = np.array([[wpnt.s_m, wpnt.d_m] for wpnt in data.wpnts])
        self.scaled_wpnts_msg = data
        v_max = np.max(np.array([wpnt.vx_mps for wpnt in data.wpnts]))
        if self.scaled_vmax != v_max:
            self.scaled_vmax = v_max
            self.scaled_max_idx = data.wpnts[-1].id
            self.scaled_max_s = data.wpnts[-1].s_m
            self.scaled_delta_s = data.wpnts[1].s_m - data.wpnts[0].s_m

    def updated_wpnts_cb(self, data: WpntArray):
        self.wpnts_updated = data.wpnts[:-1]
        self.max_s_updated = self.wpnts_updated[-1].s_m
        self.max_idx_updated = self.wpnts_updated[-1].id

    def local_wpnts_cb(self, data: WpntArray):
        self.local_wpnts = np.array([[wpnt.s_m, wpnt.d_m] for wpnt in data.wpnts])

    def opponent_trajectory_cb(self, data: OpponentTrajectory):
        self.opponent_waypoints = data.oppwpnts
        self.max_opp_idx = len(data.oppwpnts)-1
        self.opponent_wpnts_sm = np.array([wpnt.s_m for wpnt in data.oppwpnts])

    def ot_sections_check_cb(self, data: Bool):
        self.ot_section_check = data.data

    # Callback triggered by dynamic spline reconf
    def dyn_param_cb(self, params):
        self.evasion_dist = rospy.get_param("dynamic_sqp_tuner_node/evasion_dist", 0.65)
        self.obs_traj_tresh = rospy.get_param("dynamic_sqp_tuner_node/obs_traj_tresh", 1.5)
        self.spline_bound_mindist = rospy.get_param("dynamic_sqp_tuner_node/spline_bound_mindist", 0.2)
        self.lookahead = rospy.get_param("dynamic_sqp_tuner_node/lookahead_dist", 15)
        self.avoidance_resolution = rospy.get_param("dynamic_sqp_tuner_node/avoidance_resolution", 20)
        self.back_to_raceline_before = rospy.get_param("dynamic_sqp_tuner_node/back_to_raceline_before", 5)
        self.back_to_raceline_after = rospy.get_param("dynamic_sqp_tuner_node/back_to_raceline_after", 5)
        self.avoid_static_obs = rospy.get_param("dynamic_sqp_tuner_node/avoid_static_obs", True)
        self.merge_speed_factor = rospy.get_param("dynamic_sqp_tuner_node/merge_speed_factor", 1.5)
        self.width_car = rospy.get_param("dynamic_sqp_tuner_node/width_car", 0.28)
        self.min_evasion_dist = rospy.get_param("dynamic_sqp_tuner_node/min_evasion_dist", 0.05)

        # 차폭은 여러 곳에서 반차폭으로 쓰이므로 한 곳에서만 유도한다.
        self.half_width = self.width_car / 2.0
        if getattr(self, "mpc_controller", None) is not None:
            self.mpc_controller.half_width = self.half_width
        if self.min_evasion_dist > self.evasion_dist:
            rospy.logwarn(
                f"[Planner] min_evasion_dist({self.min_evasion_dist}) > evasion_dist({self.evasion_dist}) "
                f"- min_evasion_dist 로 맞춤")
            self.min_evasion_dist = self.evasion_dist

        print(
            f"[Planner] Dynamic reconf triggered new spline params: \n"
            f" Car width: {self.width_car} [m],\n"
            f" Evasion apex distance: {self.evasion_dist} [m] (하한 {self.min_evasion_dist} [m]),\n"
            f" Obstacle trajectory treshold: {self.obs_traj_tresh} [m]\n"
            f" Spline boundary mindist: {self.spline_bound_mindist} [m]\n"
            f" Lookahead distance: {self.lookahead} [m]\n"
            f" Avoid static obstacles: {self.avoid_static_obs}\n"
            f" Avoidance resolution: {self.avoidance_resolution}\n"
            f" Back to raceline before: {self.back_to_raceline_before} [m]\n"
            f" Back to raceline after: {self.back_to_raceline_after} [m]\n"
            f" Merge speed factor: {self.merge_speed_factor}\n"
        )

    def loop(self):
        # Wait for critical Messages and services
        rospy.loginfo("[OBS Spliner] Waiting for messages and services...")
        rospy.wait_for_message("/global_waypoints_scaled", WpntArray)
        rospy.wait_for_message("/car_state/odom", Odometry)
        rospy.wait_for_message("/local_waypoints", WpntArray)
        # fit_curve 가 wpnts_updated / max_idx_updated 를 쓰므로 이것도 기다려야 한다
        updated_msg = rospy.wait_for_message("/global_waypoints_updated", WpntArray)
        if self.wpnts_updated is None:
            self.updated_wpnts_cb(updated_msg)
        rospy.loginfo("[OBS Spliner] Ready!")

        # Frame counter for performance reporting
        frame_count = 0

        while not rospy.is_shutdown():
          # 한 프레임의 예외로 노드가 죽으면 /planner/avoidance/otwpnts 가 영구히 끊기고
          # state machine 이 OVERTAKE 로 못 넘어가 차가 TRAILING 에 갇힌다(장애물 앞에서 정지).
          # 프레임 단위로 잡아서 로그만 남기고 다음 프레임으로 넘어간다.
          try:
            # Measure the total loop time
            with CodeTimer("0. Total Loop Time", verbose=False):
                start_time = time.perf_counter()
                obs = deepcopy(self.obs)
                mrks = MarkerArray()
                frenet_state = self.frenet_state
                ct_state = self.cart_state
                self.current_d = frenet_state.pose.pose.position.y
                self.cur_s = frenet_state.pose.pose.position.x
                
                # Obstacle pre-processing
                obs.obstacles = sorted(obs.obstacles, key=lambda obs: obs.s_start)
                considered_obs = []
                for obs in obs.obstacles:               
                    if abs(obs.d_center) < self.obs_traj_tresh and (obs.s_start - self.cur_s) % self.scaled_max_s < self.lookahead:
                        considered_obs.append(obs)
                
                # If there is an obstacle and we are in OT section
                if len(considered_obs) > 0 and self.ot_section_check == True and self.wpnts_updated is not None:
                    # print('#################begin overtake###################')
                    
                    # [Timer 1] Measure Fit Curve (Geometric Planning)
                    # This includes GenRawTraj (C++) and QPFit (C++)
                    with CodeTimer("1. Fit Curve (Total)", verbose=False):
                        evasion_x, evasion_y, evasion_s, evasion_d, evasion_v = self.fit_curve(considered_obs, frenet_state.pose.pose.position.x)
                    
                    mpc_x = []
                    mpc_y = []
                    mpc_s = []
                    mpc_d = []
                    mpc_v = []

                    if len(evasion_x) != 0:
                        wheel_base = 0.32
                        dt = 0.05
                        
                        # [Timer 2] Measure Differential Flatness (Analytical Conversion)
                        with CodeTimer("2. Diff Flatness", verbose=False):
                            poly_traj = PolynomialPath(wheel_base, dt, evasion_x, evasion_y, evasion_v, self.max_s_updated, self.converter, self.initial_ref_spline)
                            frenet_states_list, frenet_inputs_list = poly_traj.get_df_frenet(evasion_s, evasion_d, self.qp_fit_poly_x, self.qp_fit_poly_y)
                        
                        ########## MPC Tracking ##########
                        # Note: The timer for MPC Build & Solve is inside mpc_tracking_controller_ca.py
                        x0 = np.array([frenet_states_list[0, 0], frenet_states_list[0, 1], frenet_states_list[0, 3]])
                        mpc_x, mpc_y, mpc_s, mpc_d, mpc_v = self.mpc_tracking(x0, frenet_states_list, frenet_inputs_list)

                    # Publish MPC trajectory
                    mpc_wpnts_msg = OTWpntArray(header=rospy.Header(stamp=rospy.Time.now(), frame_id="map"))
                    # 속도는 MPC 해(mpc_v)에서 가져온다.
                    #  - 예전엔 evasion_v(기하 계획 단계의 속도)를 zip 에 넣었는데, 길이가
                    #    mpc_* 와 달라서 zip 이 짧은 쪽에 맞춰 **궤적을 조용히 잘라먹었다**.
                    #    (mpc_s 는 span/0.1 개, evasion_v 는 avoidance_resolution 기반이라 서로 다르다)
                    #  - id=len(mpc_wpnts) 는 리스트가 비어 있을 때 평가돼 전부 0 이었다.
                    mpc_wpnts_msg.wpnts = [
                        Wpnt(id=i, s_m=s, d_m=d, x_m=x, y_m=y, vx_mps=v)
                        for i, (x, y, s, d, v) in enumerate(zip(mpc_x, mpc_y, mpc_s, mpc_d, mpc_v))
                    ]

                    self.evasion_pub.publish(mpc_wpnts_msg)
                    self.visualize_mpc(mpc_s, mpc_d, mpc_x, mpc_y, mpc_v)

                    if len(mpc_s) > 0:
                        self.merger_pub.publish(Float32MultiArray(data=[considered_obs[-1].s_end % self.scaled_max_s, mpc_s[-1] % self.scaled_max_s]))
                    frame_count += 1
                    if frame_count % 30 == 0:
                        CodeTimer.print_summary(clear_history=True)
                # IF there is no point in overtaking anymore delete all markers
                else:
                    mrks = MarkerArray()
                    del_mrk = Marker(header=rospy.Header(stamp=rospy.Time.now()))
                    del_mrk.action = Marker.DELETEALL
                    mrks.markers = []
                    mrks.markers.append(del_mrk)
                    self.mrks_pub.publish(mrks)
                    self.raw_mrks_pub.publish(mrks)
            
                # Publish latency
                if self.measure:
                    self.measure_pub.publish(Float32(data=time.perf_counter() - start_time))
          except Exception:
            rospy.logerr_throttle(
                2.0,
                "[SQP_Node] exception in planning loop - skipping this frame:\n" + traceback.format_exc()
            )

          self.rate.sleep()

    def mpc_tracking(self, x0, frenet_states_list, frenet_inputs_list):
        """ mpc tracking controller """
        # [s, d, delta_phi]
        raw_s = frenet_states_list[:, 0]
        raw_d = frenet_states_list[:, 1]
        for i in range(len(raw_s) - 1):
            if raw_s[i] > raw_s[i + 1]:
                raw_s[i + 1:] = [s + self.scaled_max_s for s in raw_s[i + 1:]]
                break
        
        # Detect merge-back region (where trajectory returns to raceline)
        # Criterion: |d| is decreasing towards 0 in the second half of trajectory
        is_merge_region = np.zeros(len(raw_d), dtype=bool)
        if len(raw_d) > 10:
            mid_point = len(raw_d) // 2
            d_gradient = np.gradient(np.abs(raw_d[mid_point:]))
            # Mark points where |d| is decreasing (negative gradient) and |d| < threshold
            merge_threshold = 0.3  # Consider merge when |d| < 0.3m
            for i in range(mid_point, len(raw_d)):
                if np.abs(raw_d[i]) < merge_threshold and (i > mid_point and np.abs(raw_d[i]) < np.abs(raw_d[i-1])):
                    is_merge_region[i] = True
        
        x_ref = np.column_stack((
            raw_s, 
            frenet_states_list[:, 1], 
            frenet_states_list[:, 3]
        ))

        # [v, delta]
        u_ref = np.column_stack((
            frenet_states_list[:, 2], 
            frenet_inputs_list[:, 1]
        ))

        #print bounds time
        start_time = time.time()
        # cal left boundary and right boundary for raw_s (vectorized nearest-index lookup)
        s_mod = np.asarray(raw_s, dtype=float) % self.scaled_max_s
        idx_r = np.searchsorted(self.s_ref, s_mod)
        idx_r = np.clip(idx_r, 1, len(self.s_ref) - 1)
        idx_l = idx_r - 1
        bound_raw_s_idx = np.where(
            np.abs(self.s_ref[idx_l] - s_mod) <= np.abs(self.s_ref[idx_r] - s_mod),
            idx_l, idx_r
        )
        # MPC 의 CA 제약(lb_ca = r_bound + half_width + margin, ub_ca = l_bound - half_width - margin)은
        # l_bound / r_bound 를 "차체 끝이 넘어서는 안 되는 벽" 으로 해석한다.
        # 예전 코드는 여기서 미리 0.5*width_car 를 빼서 '중심 한계'를 넘겼기 때문에 차폭이
        # 이중으로 계산됐고, 게다가 get_obs_pre_resp 가 장애물 구간만 다른 규약(벽 위치)으로
        # 덮어써서 한 배열 안에 두 규약이 섞여 있었다. 그 결과 MPC 가 트랙을 1 m 넘게
        # 벗어나는 궤적을 내기도 했다. 여기서는 규약을 "벽 위치" 하나로 통일한다.
        left_bound = self.d_left_ref[bound_raw_s_idx].astype(float)
        right_bound = -self.d_right_ref[bound_raw_s_idx].astype(float)
        s_bound = raw_s % self.scaled_max_s
        self.get_obs_pre_resp()

        # 장애물 끝과 트랙 경계 사이가 차폭보다 좁으면 회피를 포기한다.
        # 억지로 코리도어를 벌리면 MPC 가 장애물을 통과하는 궤적을 내서 그대로 들이받는다.
        if not self.corridor_feasible:
            return [], [], [], [], []

        # Extract s values from observed responses (N observations)
        obs_s = self.obs_pre_resp[:, 0]  # Observed s values (first column)
        obs_left = self.obs_pre_resp[:, 1]  # Left boundary values from observations
        obs_right = self.obs_pre_resp[:, 2]  # Right boundary values from observations
        for i in range(len(obs_s) - 1):
            if obs_s[i] > obs_s[i + 1]:
                obs_s[i + 1:] = [s + self.scaled_max_s for s in obs_s[i + 1:]]
                break

        # Find the range of obs_s (ensure ref_s is within this range)
        min_s = np.min(obs_s)
        max_s = np.max(obs_s)

        # Create a mask to only interpolate ref_s values within the range of obs_s
        valid_mask = (raw_s >= min_s) & (raw_s <= max_s)

        left_bound[valid_mask] = np.interp(raw_s[valid_mask], obs_s, obs_left)  # Interpolate left bound
        right_bound[valid_mask] = np.interp(raw_s[valid_mask], obs_s, obs_right)  # Interpolate right bound

        overtake_left_resp = self.converter.get_cartesian(s_bound, left_bound)
        overtake_right_resp = self.converter.get_cartesian(s_bound, right_bound)
        start_point = overtake_left_resp.T
        end_point = overtake_right_resp.T

        self.visualize_ob_lines(start_point, end_point)
        end_time = time.time()-start_time
        #
        # print(f"bounds time: {end_time*1000:.3f}ms")

        if len(x_ref) >= self.mpc_controller.N:
            # Relax margin in merge region for smoother transition
            if np.any(is_merge_region[:self.mpc_controller.N]):
                original_margin = self.mpc_controller.margin
                self.mpc_controller.margin = 0.02  # Reduce margin in merge region
                res_u, res_x = self.mpc_controller.solve_qp(x0, x_ref, u_ref, left_bound, right_bound)
                self.mpc_controller.margin = original_margin  # Restore original margin
            else:
                res_u, res_x = self.mpc_controller.solve_qp(x0, x_ref, u_ref, left_bound, right_bound)
        else:
            res_u = None
            res_x = None

        mpc_x = []
        mpc_y = []
        mpc_s = []
        mpc_d = []
        mpc_v = []
        
        if res_u is not None:
            res_x = np.array(res_x).reshape(-1, 3)
            res_u = np.array(res_u).reshape(-1, 2)

            # output of mpc
            origin_s = res_x[:, 0]
            origin_d = res_x[:, 1]
            origin_v = res_u[:, 0]

            # QP 가 발산/비수렴 해를 돌려주는 경우가 있다. 그대로 쓰면 아래 np.arange 가
            # 천문학적인 크기를 할당하려다 ValueError: Maximum allowed size exceeded 로
            # **노드가 죽고**, 그러면 otwpnts 가 영구히 안 나와서 차가 TRAILING 에 갇힌다.
            # 해를 쓰기 전에 유한성과 s 구간 크기를 반드시 검증한다.
            span = float(origin_s[-1] - origin_s[0])
            solution_sane = (
                np.all(np.isfinite(res_x))
                and np.all(np.isfinite(res_u))
                and span > 0.0
                and span <= self.scaled_max_s      # 회피 기동이 한 바퀴를 넘을 수는 없다
            )
            if not solution_sane:
                rospy.logwarn_throttle(
                    2.0,
                    f"[SQP_Node] discarding diverged MPC solution (span={span:.3g} m, "
                    f"finite={bool(np.all(np.isfinite(res_x)) and np.all(np.isfinite(res_u)))})"
                )
                return [], [], [], [], []

            # interpolate the output of mpc
            ds = 0.1
            n_pts = max(int(span / ds), 2)
            mpc_s = np.linspace(origin_s[0], origin_s[-1], n_pts, endpoint=False)
            mpc_d = np.interp(mpc_s, origin_s, origin_d)
            mpc_v = np.interp(mpc_s, origin_s, origin_v)

            # spline_d = CubicSpline(origin_s, origin_d)
            # spline_v = CubicSpline(origin_s, origin_v)
            # mpc_d = spline_d(mpc_s)
            # mpc_v = spline_v(mpc_s)

            mpc_s = mpc_s % self.scaled_max_s

            # 발행 전 최종 안전 검증.
            # MPC 는 선형화된 제약을 쓰므로 해가 실제로는 트랙을 벗어나거나 장애물을
            # 스치는 경우가 있다(측정 결과 트랙 경계를 1 m 넘게 벗어나는 해도 나왔다).
            # 중간 제약을 믿지 말고 "실제로 발행할 궤적" 을 직접 검사한다.
            if not self._trajectory_safe(mpc_s, mpc_d):
                return [], [], [], [], []

            resp = self.converter.get_cartesian(mpc_s, mpc_d)
            mpc_x = resp[0, :]
            mpc_y = resp[1, :]

        return mpc_x, mpc_y, mpc_s, mpc_d, mpc_v

    def _trajectory_safe(self, mpc_s, mpc_d) -> bool:
        """발행 직전 궤적이 트랙 안에 있고 장애물을 비키는지 검사한다."""
        if len(mpc_s) == 0:
            return False
        s = np.asarray(mpc_s, dtype=float) % self.scaled_max_s
        d = np.asarray(mpc_d, dtype=float)
        if not (np.all(np.isfinite(s)) and np.all(np.isfinite(d))):
            rospy.logwarn_throttle(2.0, "[SQP_Node] rejecting trajectory: non-finite values")
            return False

        idx = np.clip(np.searchsorted(self.s_ref, s), 0, len(self.s_ref) - 1)
        tol = 0.02
        if np.any(d + self.half_width > self.d_left_ref[idx] + tol):
            over = float(np.max(d + self.half_width - self.d_left_ref[idx]))
            rospy.logwarn_throttle(2.0, f"[SQP_Node] rejecting trajectory: {over:.3f} m over the left track bound")
            return False
        if np.any(d - self.half_width < -self.d_right_ref[idx] - tol):
            over = float(np.max(-self.d_right_ref[idx] - (d - self.half_width)))
            rospy.logwarn_throttle(2.0, f"[SQP_Node] rejecting trajectory: {over:.3f} m over the right track bound")
            return False

        n_obs = self.obs_downsampled_indices.size
        if n_obs and self.obs_downsampled_half_width.size == n_obs:
            reach = max(self.down_sampled_delta_s or 0.3, 0.15)
            obs_s = self.s_avoidance[self.obs_downsampled_indices] % self.scaled_max_s
            half_s = self.scaled_max_s / 2.0
            for os_, oc, oh in zip(obs_s, self.obs_downsampled_center_d, self.obs_downsampled_half_width):
                ds = np.abs(((s - os_ + half_s) % self.scaled_max_s) - half_s)
                near = ds <= reach
                if near.any() and np.any(np.abs(d[near] - oc) < oh + self.half_width):
                    rospy.logwarn_throttle(2.0, "[SQP_Node] rejecting trajectory: clips an obstacle")
                    return False
        return True

   
    
    def fit_curve(self, considered_obs: list, cur_s: float):
        danger_flag = False
        
        # --------------------------------------------------------
        # 1. Decision Making & ROI Selection
        # --------------------------------------------------------
        
        # Get the initial guess of the overtaking side (see spliner)
        initial_guess_object = self.group_objects(considered_obs)
        
        # Get the total number of waypoints for correct wrapping
        num_points = len(self.scaled_wpnts)

        # Find start and end indices of the obstacle in the global path
        # Note: Using [:, 0] to ensure we search only in the 's' column
        initial_guess_object_start_idx = np.abs(self.scaled_wpnts[:, 0] - initial_guess_object.s_start).argmin()
        initial_guess_object_end_idx = np.abs(self.scaled_wpnts[:, 0] - initial_guess_object.s_end).argmin()
        
        # Calculate the number of points covering the Region of Collision (ROC)
        # Handle wrap-around case (e.g., obstacle crosses the finish line)
        if initial_guess_object_end_idx < initial_guess_object_start_idx:
            n_points_roc = (initial_guess_object_end_idx + num_points) - initial_guess_object_start_idx
        else:
            n_points_roc = initial_guess_object_end_idx - initial_guess_object_start_idx

        # Get array of indexes of the global waypoints overlapping with the ROC
        # Using modulo operator to handle circular track indices safely
        gb_idxs = np.array([(initial_guess_object_start_idx + i) % num_points for i in range(n_points_roc)])
        
        # If the ROC is too short, we take the next 20 waypoints to ensure stability
        if len(gb_idxs) < 20:
            # Re-calculate start index approximation to ensure integer type
            start_idx_approx = int(initial_guess_object.s_center / self.scaled_delta_s)
            gb_idxs = np.array([(start_idx_approx + i) % num_points for i in range(20)])

        # Determine overtaking side (left/right) based on available space
        side, initial_apex = self._more_space(initial_guess_object, self.scaled_wpnts_msg.wpnts, gb_idxs)
        
        # Analyze curvature to adjust strategy
        kappas = np.array([self.scaled_wpnts_msg.wpnts[gb_idx].kappa_radpm for gb_idx in gb_idxs])
        max_kappa = np.max(np.abs(kappas))
        outside = "left" if np.sum(kappas) < 0 else "right"

        # Enlongate the ROC if our initial guess suggests that we are overtaking on the outside
        # This provides more margin for the maneuver on the longer path
        if side == outside:
            for i in range(len(considered_obs)):
                considered_obs[i].s_end = considered_obs[i].s_end + (considered_obs[i].s_end - considered_obs[i].s_start)%self.max_s_updated * max_kappa * (self.width_car + self.evasion_dist)

        # --------------------------------------------------------
        # 2. Discretization & Sampling
        # --------------------------------------------------------

        # 시작선(s=0)을 걸친 장애물 때문에 s 가 뒤집히면 end_avoidance < start_avoidance 가 되어
        # 아래 linspace 가 음수 샘플 수로 죽는다. cur_s 기준으로 펼친(unwrapped) s 를 쓴다.
        # 이후 waypoint 조회는 s_avoidance % scaled_max_s 로 다시 감아서 처리한다.
        obs_s_bounds = []
        for obs in considered_obs:
            s_start_uw = cur_s + (obs.s_start - cur_s) % self.scaled_max_s
            s_end_uw = s_start_uw + (obs.s_end - obs.s_start) % self.scaled_max_s
            obs_s_bounds.append((s_start_uw, s_end_uw))

        # considered_obs 는 loop() 에서 raw s_start 로 정렬돼 들어온다. 장애물이 시작선(s=0)을
        # 걸치면 언랩 후 순서가 뒤집혀서(예: s=38 장애물이 s=0.5 장애물보다 뒤로 감)
        # 아래 투영 루프가 인덱스를 역순으로 append 하고, gen_raw_traj 의
        # start/mid/end 인덱스가 뒤집혀 np.linspace(num=음수) 로 죽는다.
        # 언랩된 s 기준으로 다시 정렬해 단조성을 보장한다.
        order = sorted(range(len(obs_s_bounds)), key=lambda i: obs_s_bounds[i][0])
        considered_obs = [considered_obs[i] for i in order]
        obs_s_bounds = [obs_s_bounds[i] for i in order]

        min_s_obs_start = min(b[0] for b in obs_s_bounds)
        max_s_obs_end = max(b[1] for b in obs_s_bounds)
        for obs in considered_obs:
            # Check if it is a really wide obstacle (Safety Flag)
            if obs.d_left > 3 or obs.d_right < -3:
                danger_flag = True

        # Define the longitudinal range for the avoidance maneuver
        # Dynamic merge distance based on current velocity
        current_velocity = max(self.frenet_state.twist.twist.linear.x, 1.0)  # Avoid zero velocity
        dynamic_merge_dist = max(self.back_to_raceline_after, current_velocity * self.merge_speed_factor)
        
        start_avoidance = max((min_s_obs_start - self.back_to_raceline_before), cur_s)
        end_avoidance = max_s_obs_end + dynamic_merge_dist

        # Generate downsampled s-coordinates for the maneuver
        s_avoidance = np.linspace(start_avoidance, end_avoidance, self.avoidance_resolution)
        self.down_sampled_delta_s = s_avoidance[1] - s_avoidance[0]
        
        # Find corresponding global waypoints for boundary extraction (vectorized)
        s_mod = s_avoidance % self.scaled_max_s
        scaled_wpnts_s = self.scaled_wpnts[:, 0]
        idx_r = np.searchsorted(scaled_wpnts_s, s_mod)
        idx_r = np.clip(idx_r, 1, len(scaled_wpnts_s) - 1)
        idx_l = idx_r - 1
        scaled_wpnts_indices = np.where(
            np.abs(scaled_wpnts_s[idx_l] - s_mod) <= np.abs(scaled_wpnts_s[idx_r] - s_mod),
            idx_l, idx_r
        )
        corresponding_scaled_wpnts = [self.scaled_wpnts_msg.wpnts[i] for i in scaled_wpnts_indices]
        
        # Pre-extract wpnt arrays for faster access
        wpnts_d_left = np.array([w.d_left for w in corresponding_scaled_wpnts])
        wpnts_d_right = np.array([w.d_right for w in corresponding_scaled_wpnts])
        wpnts_x = np.array([w.x_m for w in corresponding_scaled_wpnts])
        wpnts_y = np.array([w.y_m for w in corresponding_scaled_wpnts])
        wpnts_v = np.array([w.vx_mps for w in corresponding_scaled_wpnts])
        
        # Calculate Global Trajectory Curvature (vectorized)
        x_prime = np.diff(wpnts_x)
        x_prime = np.where(x_prime == 0, 1e-6, x_prime)
        y_prime = np.diff(wpnts_y)
        y_prime = np.where(y_prime == 0, 1e-6, y_prime)
        x_prime_prime = np.diff(x_prime)
        y_prime_prime = np.diff(y_prime)
        x_prime = x_prime[:-1]
        y_prime = y_prime[:-1]
        self.global_traj_kappas = (x_prime*y_prime_prime - y_prime*x_prime_prime) / ((x_prime**2 + y_prime**2)**(3/2))
       
        # --------------------------------------------------------
        # 3. Obstacle Projection
        # --------------------------------------------------------
        
        # Map obstacles onto the sampled s-avoidance grid (optimized with list pre-allocation)
        obs_indices_list = []
        obs_center_list = []
        obs_dist_list = []
        # 장애물 자체의 반폭. MPC 코리도어를 "장애물의 실제 끝(벽)" 으로 주기 위해 필요하다.
        # min_dist 는 soft 여유까지 포함한 값이라 그대로 벽으로 쓰면 이중 계산이 된다.
        obs_half_list = []
        inflation_idx = 2
        n_avoidance = len(s_avoidance)

        # /opponent_trajectory 가 아직 안 왔으면(GP 체인 기동 전) 동적 장애물도 정적처럼 처리한다
        opp_traj_ready = (
            self.opponent_wpnts_sm is not None
            and self.max_opp_idx is not None
            and self.max_opp_idx > 0
            and len(self.opponent_wpnts_sm) > 1
        )

        for obs, (obs_s_start_uw, obs_s_end_uw) in zip(considered_obs, obs_s_bounds):
            obs_idx_start = np.searchsorted(s_avoidance, obs_s_start_uw)
            obs_idx_start = min(obs_idx_start, n_avoidance - 1)
            obs_idx_end = np.searchsorted(s_avoidance, obs_s_end_uw)
            obs_idx_end = min(obs_idx_end, n_avoidance - 1)
            obs_idx_start = max(0, obs_idx_start - inflation_idx)
            obs_idx_end = min(n_avoidance - 1, obs_idx_end + inflation_idx)
            
            if obs_idx_start < n_avoidance - 2:
                if obs.is_static or obs_idx_end == obs_idx_start or not opp_traj_ready:
                    if obs_idx_end == obs_idx_start:
                        obs_idx_end = obs_idx_start + 1
                    n_pts = obs_idx_end - obs_idx_start + 1
                    obs_indices_list.append(np.arange(obs_idx_start, obs_idx_end + 1))
                    obs_center_list.append(np.full(n_pts, (obs.d_left + obs.d_right) * 0.5))
                    # min_dist = 장애물 중심에서 차량 '중심'까지 확보하고 싶은 횡거리.
                    # 궤적의 d 는 차량 중심이므로 필요한 것은 (장애물 반폭 + 반차폭 + 여유) 다.
                    # 예전에는 full width_car 를 더해서 반차폭을 이중으로 셌고(0.14 m 추가),
                    # 그 결과 차체 여유를 0.29 m 나 요구했다. 폭 1.5 m 트랙에서 장애물 옆
                    # 공간이 0.6 m 남짓인 걸 감안하면 사실상 항상 infeasible 이었다.
                    # (아래 동적 장애물 분기는 obs_half=0.5*width_car 라서 원래부터 올바르다)
                    obs_dist_list.append(np.full(
                        n_pts,
                        (obs.d_left - obs.d_right) * 0.5 + self.half_width + self.evasion_dist))
                    obs_half_list.append(np.full(n_pts, max((obs.d_left - obs.d_right) * 0.5, 0.0)))
                else:
                    indices = np.arange(obs_idx_start, obs_idx_end + 1)
                    obs_indices_list.append(indices)
                    # Vectorized opponent waypoint lookup
                    s_query = s_avoidance[indices] % self.max_opp_idx
                    opp_idx_r = np.searchsorted(self.opponent_wpnts_sm, s_query)
                    opp_idx_r = np.clip(opp_idx_r, 1, len(self.opponent_wpnts_sm) - 1)
                    opp_idx_l = opp_idx_r - 1
                    opp_wpnts_idx = np.where(
                        np.abs(self.opponent_wpnts_sm[opp_idx_l] - s_query) <= np.abs(self.opponent_wpnts_sm[opp_idx_r] - s_query),
                        opp_idx_l, opp_idx_r
                    )
                    d_opp = np.array([self.opponent_waypoints[i].d_m for i in opp_wpnts_idx])
                    obs_center_list.append(d_opp)
                    # 상대차는 폭을 ego 와 같다고 보므로 (상대 반폭 + 내 반폭) = width_car
                    obs_dist_list.append(np.full(len(indices), self.width_car + self.evasion_dist))
                    obs_half_list.append(np.full(len(indices), self.half_width))

        if obs_indices_list:
            idx_all = np.concatenate(obs_indices_list).astype(int)
            ctr_all = np.concatenate(obs_center_list)
            dst_all = np.concatenate(obs_dist_list)
            half_all = np.concatenate(obs_half_list)
            # 인덱스 오름차순으로 정렬하고 중복 인덱스는 하나만 남긴다.
            # 장애물 구간이 겹치면 같은 인덱스가 여러 번 들어오는데, 그대로 두면
            # gen_raw_traj / get_obs_pre_resp 가 비단조 배열을 받는다.
            # 중복 시 더 보수적인(min_dist 가 큰) 쪽을 채택한다.
            order = np.lexsort((-dst_all, idx_all))
            idx_all, ctr_all, dst_all, half_all = idx_all[order], ctr_all[order], dst_all[order], half_all[order]
            keep = np.concatenate(([True], np.diff(idx_all) != 0))
            self.obs_downsampled_indices = idx_all[keep]
            self.obs_downsampled_center_d = ctr_all[keep]
            self.obs_downsampled_min_dist = dst_all[keep]
            self.obs_downsampled_half_width = half_all[keep]
        else:
            self.obs_downsampled_indices = np.array([], dtype=int)
            self.obs_downsampled_center_d = np.array([])
            self.obs_downsampled_min_dist = np.array([])
            self.obs_downsampled_half_width = np.array([])
        self.s_avoidance = s_avoidance
        self.side = side

        # --------------------------------------------------------
        # 4. Raw Trajectory Generation (C++ Optimized)
        # --------------------------------------------------------

        raw_evasion_v = wpnts_v
        ref_d_left = wpnts_d_left
        ref_d_right = wpnts_d_right
        raw_evasion_s = s_avoidance[:]
        
        # Generate the initial piecewise linear guess
        # Note: Optimized via C++ extension (mpc_builder) if integrated
        raw_evasion_d, poly_x, poly_y, t_data, is_traj_valid = self.gen_raw_traj(
            raw_evasion_s, raw_evasion_v, ref_d_left, ref_d_right, side
        )

        # --------------------------------------------------------
        # 5. Trajectory Fitting (C++ / QP Optimized)
        # --------------------------------------------------------

        # Update minimum radius constraints based on speed
        clipped_speed = np.clip(self.frenet_state.twist.twist.linear.x, 1, 6.5)
        radius_speed = min([clipped_speed, self.wpnts_updated[(scaled_wpnts_indices[0])%self.max_idx_updated].vx_mps])
        self.min_radius = np.interp(radius_speed, [1, 6, 7], [0.2, 2, 4])
        self.max_kappa = 1/self.min_radius

        if len(self.past_avoidance_d) == 0:
            initial_guess = np.full(len(s_avoidance), initial_apex)
        elif len(self.past_avoidance_d) > 0:
            initial_guess = self.past_avoidance_d
        else:
            if self.last_ot_side == "left":
                initial_guess = np.full(len(s_avoidance), 2)
            else:
                initial_guess = np.full(len(s_avoidance), -2)
            
        # Note: The original Python SQP solver call is commented out in favor of the new pipeline
        # result = self.solve_sqp(initial_guess, bounds)

        if is_traj_valid == True:
            # Prepare data for fitting
            x_data, y_data, t_data = self.get_fit_variable(raw_evasion_s, raw_evasion_d, raw_evasion_v)
            
            # Solve QP for Quintic Spline Coefficients
            # (This step is now accelerated via C++ Closed-form solution)
            poly_x, poly_y = self.get_qp_fit_coeffs(x_data, y_data, t_data, raw_evasion_s, raw_evasion_v)
            
            # Validate the fitted trajectory against boundaries
            closedform_evasion_valid = self.check_fit_traj_valid(poly_x, poly_y, t_data, ref_d_left if side == 'left' else ref_d_right)
            
            if closedform_evasion_valid == False:
                # Return empty if validation fails
                return [], [], [], [], []
                
            # --------------------------------------------------------
            # 6. Interpolation & Output Generation
            # --------------------------------------------------------

            # Resample to global resolution
            n_global_avoidance_points = int((end_avoidance - start_avoidance) / self.scaled_delta_s)
            s_array = np.linspace(start_avoidance, end_avoidance, n_global_avoidance_points)
            
            # Interpolate lateral deviation d
            evasion_d = np.interp(s_array, s_avoidance, raw_evasion_d)
            
            # Handle wrapping for s
            evasion_s = np.mod(s_array, self.scaled_max_s)
            
            # Convert to Cartesian coordinates
            resp = self.converter.get_cartesian(evasion_s, evasion_d)
            evasion_x = resp[0, :]
            evasion_y = resp[1, :]
            
            # Interpolate velocity (use pre-extracted wpnts_v)
            evasion_v = np.interp(s_array, s_avoidance, wpnts_v)
            
            # Post-overtake speed reduction: detect upcoming corners in merge region
            merge_start_idx = int(len(evasion_s) * 0.7)  # Last 30% is merge region
            if merge_start_idx < len(evasion_s):
                # Get upcoming curvature after merge
                s_merge = evasion_s[merge_start_idx:]
                idx_merge = np.searchsorted(self.s_ref, s_merge)
                idx_merge = np.clip(idx_merge, 0, len(self.kapparef) - 1)
                kappas_ahead = self.kapparef[idx_merge]
                max_kappa_ahead = np.max(np.abs(kappas_ahead)) if len(kappas_ahead) > 0 else 0
                
                # If high curvature ahead (sharp corner), reduce speed in merge region
                if max_kappa_ahead > 0.15:  # Threshold for "sharp corner"
                    speed_reduction_factor = np.clip(1.0 - (max_kappa_ahead - 0.15) * 2.0, 0.6, 1.0)
                    # Apply smooth speed reduction in merge region
                    blend = np.linspace(1.0, speed_reduction_factor, len(evasion_s) - merge_start_idx)
                    evasion_v[merge_start_idx:] *= blend
            
            # Update internal state
            self.past_avoidance_d = raw_evasion_d[:]
            self.qp_fit_poly_x = poly_x[:]
            self.qp_fit_poly_y = poly_y[:]
            
            if np.mean(evasion_d) > 0:
                self.last_ot_side = "left"
            else:
                self.last_ot_side = "right"

        else:
            # Return empty lists if trajectory generation failed
            evasion_x = []
            evasion_y = []
            evasion_s = []
            evasion_d = []
            evasion_v = []
            self.past_avoidance_d = []
            self.qp_fit_poly_x = np.array([])
            self.qp_fit_poly_y = np.array([])
        
        # Visualize the result in Rviz
        self.visualize_sqp(evasion_s, evasion_d, evasion_x, evasion_y, evasion_v) 

        return evasion_x, evasion_y, evasion_s, evasion_d, evasion_v
    
    def get_obs_pre_resp(self):
        # Vectorized nearest s_ref index for obstacle downsampled points
        s_obs = self.s_avoidance[self.obs_downsampled_indices].astype(float) % self.scaled_max_s
        idx_r = np.searchsorted(self.s_ref, s_obs)
        idx_r = np.clip(idx_r, 1, len(self.s_ref) - 1)
        idx_l = idx_r - 1
        opp_wpnts_glbidx = np.where(
            np.abs(self.s_ref[idx_l] - s_obs) <= np.abs(self.s_ref[idx_r] - s_obs),
            idx_l, idx_r
        )

        track_left_bound = self.d_left_ref[opp_wpnts_glbidx]
        track_right_bound = -self.d_right_ref[opp_wpnts_glbidx]
        obs_center = self.obs_downsampled_center_d
        safe_dist = self.obs_downsampled_min_dist

        # MPC 의 충돌회피 제약은 lb_ca = r_bound + half_width + margin,
        # ub_ca = l_bound - half_width - margin 이다.
        # 즉 l_bound / r_bound 는 "차체 끝이 넘어서는 안 되는 벽" 이고, MPC 가 스스로
        # half_width + margin 을 안쪽으로 확보한다.
        #
        # 따라서 여기서 넘겨야 하는 값은 장애물의 **실제 끝(벽)** 이다.
        # 예전 코드는 obs_center + min_dist (= 반폭 + 차폭 + evasion_dist) 를 벽으로 넘겼는데,
        # MPC 가 거기에 half_width + margin 을 또 더하므로 차폭이 이중으로 계산됐다.
        # 그래서 코리도어가 거의 항상 음수 폭이 되어 QP 가 실패했고, 이를 무리하게 살리려고
        # 코리도어를 강제로 벌리면(이전 시도) 하한이 장애물 안쪽으로 내려가
        # **MPC 가 장애물을 통과하는 궤적을 계획**하게 된다.
        #
        # 이제 장애물 끝 + 작은 고정 패드만 벽으로 주고, 그래도 차가 못 지나가면
        # 완화하지 않고 infeasible 로 처리한다(회피 포기 -> trailing, 충돌보다 안전).
        edge_pad = 0.03
        obs_half = self.obs_downsampled_half_width[
            :len(obs_center)
        ] if self.obs_downsampled_half_width.size == obs_center.size else np.maximum(
            safe_dist - self.width_car - self.evasion_dist, 0.0
        )
        obs_edge_left = obs_center + obs_half + edge_pad
        obs_edge_right = obs_center - obs_half - edge_pad

        if self.side == 'left':
            ub = track_left_bound
            lb = np.maximum(obs_edge_left, track_right_bound)
        else:
            lb = track_right_bound
            ub = np.minimum(obs_edge_right, track_left_bound)

        # 코리도어가 차폭 + 마진보다 좁으면 이 side 로는 물리적으로 통과 불가능하다.
        # 다만 마진(MPC 의 여유)은 좁은 구간에서 깎을 수 있는 soft 값이다.
        # 예전에는 nominal 마진 하나로만 판정해서, 마진만 조금 줄이면 지나갈 수 있는
        # 구간에서도 회피를 통째로 포기했다(= 사람이 보기엔 뚫려 있는데 정지).
        worst = float(np.min(ub - lb))
        # 기준 마진은 반드시 저장해 둔 nominal 을 쓴다. mpc_controller.margin 을 그대로 읽으면
        # 이전 프레임에서 완화한 값이 기준이 되어 마진이 프레임마다 계속 깎여 내려간다.
        nominal_margin = self.nominal_mpc_margin
        needed_margin = worst / 2.0 - self.half_width      # 이 코리도어가 허용하는 최대 마진

        if needed_margin >= nominal_margin:
            self.corridor_feasible = True
            self.mpc_controller.margin = nominal_margin
        elif needed_margin >= self.min_mpc_margin:
            # 마진을 깎으면 통과 가능 -> 이번 solve 에 한해 완화한다
            self.corridor_feasible = True
            self.mpc_controller.margin = float(max(needed_margin, self.min_mpc_margin))
            rospy.loginfo_throttle(
                2.0,
                f"[SQP_Node] corridor 가 좁아 MPC margin 을 {nominal_margin:.3f} -> "
                f"{self.mpc_controller.margin:.3f} m 로 완화 (corridor {worst:.3f} m, {self.side})"
            )
        else:
            # 최소 마진으로도 차폭이 안 들어간다 = 실제로 통과 불가
            self.corridor_feasible = False
            self.mpc_controller.margin = nominal_margin
            rospy.logwarn_throttle(
                2.0,
                f"[SQP_Node] corridor too narrow for a pass on the {self.side} "
                f"(min {worst:.3f} m < required {2.0 * (self.half_width + self.min_mpc_margin):.3f} m) "
                f"- not overtaking"
            )

        first_sequence = ub.tolist()
        second_sequence = lb.tolist()
        s_sequence = self.s_avoidance[self.obs_downsampled_indices].copy()

        # Points ahead of ROC for safer overtaking (prepend without insert(0))
        ahead_num = 5
        safe_point_start_idx = max(self.obs_downsampled_indices[0] - ahead_num, 0)
        if safe_point_start_idx > 0:
            start_idx = self.obs_downsampled_indices[0] - ahead_num
            end_idx = self.obs_downsampled_indices[0] - 1
            index_range = np.arange(start_idx, end_idx + 1)
            s_ahead = self.s_avoidance[index_range].astype(float) % self.scaled_max_s
            idx_r = np.searchsorted(self.s_ref, s_ahead)
            idx_r = np.clip(idx_r, 1, len(self.s_ref) - 1)
            idx_l = idx_r - 1
            larger_wpnts_glbidx = np.where(
                np.abs(self.s_ref[idx_l] - s_ahead) <= np.abs(self.s_ref[idx_r] - s_ahead),
                idx_l, idx_r
            )
            # Order for prepend: obs_start-1, obs_start-2, ..., obs_start-5 -> use larger_wpnts_glbidx[::-1]
            rev_idx = larger_wpnts_glbidx[::-1]
            new_s = np.array([self.s_avoidance[self.obs_downsampled_indices[0] - (i + 1)] for i in range(ahead_num)], dtype=float)
            if self.side == 'left':
                new_first = self.d_left_ref[rev_idx].tolist()
                limit0 = second_sequence[0]
                new_second = np.maximum.accumulate(
                    np.concatenate([[limit0], -self.d_right_ref[rev_idx]])
                )[1:].tolist()
            else:
                new_second = (-self.d_right_ref[rev_idx]).tolist()
                limit0 = first_sequence[0]
                new_first = np.minimum.accumulate(
                    np.concatenate([[limit0], self.d_left_ref[rev_idx]])
                )[1:].tolist()
            first_sequence = new_first + first_sequence
            second_sequence = new_second + second_sequence
            s_sequence = np.concatenate([new_s, s_sequence])

        self.obs_pre_resp = np.column_stack((s_sequence, first_sequence, second_sequence))

    
    def plot_sample(self, corresponding_scaled_wpnts, poly_x, poly_y, t_data, raw_evasion_s, raw_evasion_d, ref_d_left, ref_d_right, side, is_traj_valid):
        x_global_points = np.array([wpnt.x_m for wpnt in corresponding_scaled_wpnts])
        y_global_points = np.array([wpnt.y_m for wpnt in corresponding_scaled_wpnts])

        raw_resp = self.converter.get_cartesian(raw_evasion_s, raw_evasion_d)
        raw_evasion_x = raw_resp[0, :]
        raw_evasion_y = raw_resp[1, :]

        left_resp = self.converter.get_cartesian(raw_evasion_s, ref_d_left)
        left_x = left_resp[0, :]
        left_y = left_resp[1, :]

        right_resp = self.converter.get_cartesian(raw_evasion_s, -ref_d_right)
        right_x = right_resp[0, :]
        right_y = right_resp[1, :]


        obs_s = np.array(raw_evasion_s)[self.obs_downsampled_indices]
        obs_resp = self.converter.get_cartesian(obs_s, self.obs_downsampled_center_d)
        obs_x = obs_resp[0, :]
        obs_y = obs_resp[1, :]

        
        if side == 'left':
            safe_d = self.obs_downsampled_min_dist
        else:
            safe_d = -self.obs_downsampled_min_dist
        safe_resp = self.converter.get_cartesian(obs_s, safe_d)
        safe_x = safe_resp[0, :]
        safe_y = safe_resp[1, :]

        save_dir = '~/eth_race/sample'
        save_dir = os.path.expanduser(save_dir)
        # Ensure the save directory exists
        os.makedirs(save_dir, exist_ok=True)

        # Generate a unique filename using the current timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        file_path = os.path.join(save_dir, f"sample_{side}_{timestamp}_valid_{is_traj_valid}.png")

        # Initialize the plot
        plt.figure()
        plt.title("Sample Visualization")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)

        current_pos = self.converter.get_cartesian(self.frenet_state.pose.pose.position.x, self.frenet_state.pose.pose.position.y)
        print(f'current_pos size:{current_pos.shape}')
        current_x = current_pos[0]
        current_y = current_pos[1]
        plt.scatter(current_x, current_y, color='r', s=200, marker='*')
        plt.plot(x_global_points, y_global_points, color='k', linestyle='--')
        plt.plot(raw_evasion_x, raw_evasion_y, color="r", linestyle="-")
        plt.plot(left_x, left_y, color="k", linestyle="-")
        plt.plot(right_x, right_y, color="k", linestyle="-")

        plt.plot(safe_x, safe_y, color="g", linestyle="--")
        plt.scatter(obs_x, obs_y, color='b', s=100, marker='o')

        x_fit = self.qp_fit.poly(t_data, poly_x)  # x(t)
        y_fit = self.qp_fit.poly(t_data, poly_y)  # y(t)
        plt.plot(x_fit, y_fit, 'purple')
        # Save the plot
        plt.savefig(file_path)
        plt.close()  # Close the figure to free memory

    # def gen_raw_traj(self, raw_evasion_s, raw_evasion_v, ref_d_left, ref_d_right, side):
    #     # Convert inputs to C++ compatible format
    #     s_vec = raw_evasion_s.astype(np.float64)
    #     ref_l = ref_d_left.astype(np.float64)
    #     ref_r = ref_d_right.astype(np.float64)
    #     obs_idx = self.obs_downsampled_indices.astype(np.int32)
    #     obs_center = self.obs_downsampled_center_d.astype(np.float64)
    #     obs_min = self.obs_downsampled_min_dist.astype(np.float64)

    #     current_d = self.frenet_state.pose.pose.position.y
    #     if self.cur_s % self.scaled_max_s < raw_evasion_s[0] % self.scaled_max_s:
    #          current_d = 0.0

    #     # Call C++
    #     raw_d, is_valid = mpc_builder.gen_raw_traj_cpp(
    #         s_vec, current_d, ref_l, ref_r, 
    #         obs_idx, obs_center, obs_min, side
    #     )

    #     if not is_valid:
    #         return np.array([]), np.array([]), np.array([]), np.array([]), False
        
    #     return raw_d, np.array([]), np.array([]), np.array([]), True   
    def _target_d(self, obs_idx: int, bound: float, safe_dist: float, side: str):
        """회피 궤적의 목표 d 를 하나 계산한다.

        obs_downsampled_min_dist 는 '장애물 중심에서 확보하고 싶은 횡방향 거리'
        (장애물 반폭 + 차폭 + evasion_dist)이다. 절대 |d| 목표가 아니다.

        이전 구현은 이 값을 |d| 의 하한으로 강제(max(..., min_dist))했기 때문에
          - 장애물이 트랙 한쪽에 붙어 있어도 중앙에 있는 것처럼 큰 d 를 요구했고
          - 트랙이 min_dist 보다 좁으면 목표 d 가 항상 경계 밖으로 나가
            check_interp_traj_valid 가 무조건 실패했다 (= 회피 궤적을 영원히 못 만듦).

        여기서는 장애물 중심 기준으로 원하는 여유를 잡고, 트랙 경계를 우선해서 clamp 한다.
        경계로 clamp 한 결과가 장애물을 물리적으로도 못 비키는 경우에만 실패로 본다.

        Returns
        -------
        (target_d, feasible)
        """
        sign = 1.0 if side == "left" else -1.0
        obs_center = self.obs_downsampled_center_d[obs_idx]
        min_dist = self.obs_downsampled_min_dist[obs_idx]

        want = obs_center + sign * min_dist          # 여유를 다 확보한 이상적인 위치
        limit = sign * (bound - safe_dist)           # 트랙 경계 (bound 는 항상 양수)
        target = want if sign * want <= sign * limit else limit

        # 물리적 하한: soft 여유(evasion_dist)를 min_evasion_dist 까지만 깎는다.
        hard = obs_center + sign * self._hard_min_dist(obs_idx)
        feasible = bool(sign * target >= sign * hard)
        return float(target), feasible

    def _hard_min_dist(self, obs_idx: int) -> float:
        """장애물 중심에서 차량 중심까지 '절대로' 줄일 수 없는 횡거리.

        soft 여유(evasion_dist)를 min_evasion_dist 까지 완화한 값이다.
        min_evasion_dist 를 0 으로 두면 차체가 장애물에 딱 닿는 한계까지 허용하므로
        실제로는 몇 cm 를 남겨두는 것이 맞다.
        """
        min_dist = float(self.obs_downsampled_min_dist[obs_idx])
        return max(min_dist - self.evasion_dist + self.min_evasion_dist, 0.0)

    def _clamp_to_track(self, raw_evasion_d, ref_d_left, ref_d_right, idx_mid_in_es=None, idx_mid_in_obs=None):
        """궤적을 트랙 경계 안으로 눌러 넣고, 장애물 여유가 남는지 확인한다.

        start/mid/end 앵커는 각자 자기 위치의 경계로 clamp 되지만, 그 사이를 linspace 로
        이으면 트랙이 좁아지는 중간 지점에서 경계를 넘을 수 있다. 예전에는 그 경우
        check_interp_traj_valid 가 실패해서 기동 전체를 포기했고, 이게 회피 거절의
        가장 큰 원인이었다(좁은 구간 때문에 넓은 구간의 회피까지 버려짐).

        여기서는 경계로 눌러 넣은 뒤, 눌린 결과가 장애물을 물리적으로 못 비키는
        경우에만 실패로 판정한다.

        Returns
        -------
        (clamped_d_list, ok)
        """
        safe = 0.1
        d = np.asarray(raw_evasion_d, dtype=float)
        ub = np.asarray(ref_d_left, dtype=float) - safe
        lb = -(np.asarray(ref_d_right, dtype=float) - safe)
        ub = np.maximum(ub, lb)          # 트랙이 2*safe 보다 좁은 병리적 경우 방어
        d = np.clip(d, lb, ub)

        # 여유 검증은 apex 에서만 한다.
        # 진입/복귀 램프 구간의 장애물 인덱스까지 전부 요구하면(초기 시도에서 그렇게 했다)
        # 램프가 아직 d 를 다 올리지 못한 지점 때문에 대부분의 정상 회피가 거절된다.
        # 램프 구간의 실제 충돌회피는 get_obs_pre_resp 의 코리도어 + MPC CA 제약이 담당한다.
        if idx_mid_in_es is not None and self.obs_downsampled_indices.size:
            obs_c = self.obs_downsampled_center_d[idx_mid_in_obs]
            hard = self._hard_min_dist(idx_mid_in_obs)
            if abs(d[idx_mid_in_es] - obs_c) < hard:
                return d.tolist(), False
        return d.tolist(), True

    def gen_raw_traj(self, raw_evasion_s, raw_evasion_v, ref_d_left, ref_d_right, side):
        if self.obs_downsampled_indices.size == 0:
            return np.array([]), np.array([]), np.array([]), np.array([]), False
        
        # Initialize raw_evasion_d as a zero list with the same length as raw_evasion_s
        raw_evasion_d = [0] * len(raw_evasion_s)
        # Get the indices of the obstacles
        idx_start_in_es = self.obs_downsampled_indices[0]  # The first obstacle point
        idx_start_in_es = max(idx_start_in_es - 2, 0)
        idx_mid_in_es = self.obs_downsampled_indices[len(self.obs_downsampled_indices) // 2]  # The middle obstacle point
        idx_end_in_es = self.obs_downsampled_indices[-1]  # The last obstacle point
        idx_end_in_es = min(idx_end_in_es + 6, len(raw_evasion_s) - 1)
        idx_start_in_obs = 0
        idx_mid_in_obs = len(self.obs_downsampled_indices) // 2
        idx_end_in_obs = -1
        safe_dist = 0.2
        lon_dist = 1.5

        # 아래 세 구간을 np.linspace 로 채우므로 인덱스가 반드시 단조여야 한다.
        # 뒤집히면 num 이 음수가 되어 ValueError 로 노드가 죽는다. 방어적으로 막는다.
        # (정상 경로에서는 fit_curve 가 인덱스를 정렬해 주므로 여기 걸리지 않는다)
        if not (idx_start_in_es <= idx_mid_in_es <= idx_end_in_es):
            rospy.logwarn_throttle(
                2.0,
                f"[SQP_Node] non-monotonic obstacle indices "
                f"(start={idx_start_in_es}, mid={idx_mid_in_es}, end={idx_end_in_es}) - skipping frame"
            )
            return np.array([]), np.array([]), np.array([]), np.array([]), False

        raw_evasion_valid = False
        closedform_evasion_valid = False
        frenet_state = self.frenet_state

        current_s = frenet_state.pose.pose.position.x
        current_d = frenet_state.pose.pose.position.y
        if current_s % self.scaled_max_s < raw_evasion_s[0] % self.scaled_max_s:
            # the current pos of ego car is not the planning start
            current_d = 0
        # the ego car and the planning start are not on the same side
        # and the longitudinal distance between the ego car and the planning start is too small. 
        if current_d < 0:
            if side == "left":
                if np.fabs(raw_evasion_s[idx_start_in_es] - raw_evasion_s[0]) < lon_dist:
                    # not suitable for overtaking
                    return np.array([]), np.array([]), np.array([]), np.array([]), False
        if current_d > 0:
            if side == "right":
                if np.fabs(raw_evasion_s[idx_start_in_es] - raw_evasion_s[0]) < lon_dist:
                    # not suitable for overtaking
                    return np.array([]), np.array([]), np.array([]), np.array([]), False

        # Calculate the lateral offset absolute value for each of the points
        # Curvature compensation for outside overtaking to avoid cutting corners
        kappas_start_to_mid = self.global_traj_kappas[0:max(idx_mid_in_es - 2, 1)] if len(self.global_traj_kappas) > 0 else np.array([0])
        kappas_merge = self.global_traj_kappas[max(idx_end_in_es - 2, 0):] if len(self.global_traj_kappas) > idx_end_in_es else np.array([0])
        avg_kappa = np.mean(kappas_start_to_mid)  # Keep sign for direction
        avg_kappa_merge = np.mean(kappas_merge) if len(kappas_merge) > 0 else 0
        outside = "left" if avg_kappa < 0 else "right"
        is_outside_overtake = (side == outside)
        curvature_gain =2.5#tuning parameter for gain for extreme curves
        
        if side == "left":
            mid_d, mid_ok = self._target_d(idx_mid_in_obs, ref_d_left[idx_mid_in_es], safe_dist, "left")
            start_d, _ = self._target_d(idx_start_in_obs, ref_d_left[idx_start_in_es], safe_dist, "left")
            end_d, _ = self._target_d(idx_end_in_obs, ref_d_left[idx_end_in_es], safe_dist, "left")
            if not mid_ok:
                print(f'[Left Overtake] infeasible: track too narrow (mid_d:{mid_d:.3f}, d_left:{ref_d_left[idx_mid_in_es]:.3f})')
                return np.array([]), np.array([]), np.array([]), np.array([]), False

            # Outside overtake curvature compensation: increase start_d to avoid cutting
            if is_outside_overtake and idx_start_in_es > 0:
                lon_dist_to_mid = (idx_mid_in_es - idx_start_in_es) * self.down_sampled_delta_s
                curvature_comp = curvature_gain * abs(avg_kappa) * lon_dist_to_mid * mid_d
                start_d = min(start_d + curvature_comp, ref_d_left[idx_start_in_es] - safe_dist)
            
            # Ensure that the lateral offsets at idx_start and idx_end are less than mid_d
            if idx_start_in_es == 0:
                start_d = current_d
            elif 0 < idx_start_in_es < 4:
                start_d = current_d + (idx_start_in_es / idx_mid_in_es) * (mid_d - current_d)
            else:
                start_d = min(start_d, mid_d)
            end_d = min(end_d, mid_d)
            print(f'[Left Overtake] start_d:{start_d:.3f}, mid_d:{mid_d:.3f}, end_d:{end_d:.3f}, outside:{is_outside_overtake}')
            print(f'[Merge Info] idx_end:{idx_end_in_es}, merge_points:{len(raw_evasion_s) - idx_end_in_es}, merge_dist:{(len(raw_evasion_s) - idx_end_in_es)*self.scaled_delta_s:.2f}m')

            # For the segments: 0 to idx_start, idx_start to idx_mid, idx_mid to idx_end, idx_end to len
            if idx_start_in_es == 0:
                raw_evasion_d[0] = current_d
                start_d = current_d
            else:
                raw_evasion_d[0:idx_start_in_es] = np.linspace(current_d, start_d, num=idx_start_in_es)
            raw_evasion_d[idx_start_in_es:idx_mid_in_es] = np.linspace(start_d, mid_d, num=idx_mid_in_es - idx_start_in_es)
            raw_evasion_d[idx_mid_in_es:idx_end_in_es] = np.linspace(mid_d, end_d, num=idx_end_in_es - idx_mid_in_es)
            
            # Curvature-aware merge-back: adjust based on track curvature direction
            n_merge = len(raw_evasion_s) - idx_end_in_es
            if n_merge > 0:
                # Check if merge direction conflicts with track curvature (convex/concave mismatch)
                merge_curvature_conflict = (end_d > 0 and avg_kappa_merge > 0) or (end_d < 0 and avg_kappa_merge < 0)
                if merge_curvature_conflict:
                    # Use slower S-curve for smoother transition when curvature conflicts
                    t = np.linspace(0, 1, n_merge)
                    s_curve = 3 * t**2 - 2 * t**3  # Smooth S-curve: 0->1
                    raw_evasion_d[idx_end_in_es:] = end_d * (1 - s_curve)
                else:
                    t = np.linspace(0, np.pi/2, n_merge)
                    raw_evasion_d[idx_end_in_es:] = end_d * np.cos(t)

            # check the raw d is not out of boundary
            # 경계로 눌러 넣고 장애물 여유만 재검증한다 (구: check_interp_traj_valid 로 전체 포기)
            raw_evasion_d, raw_evasion_valid = self._clamp_to_track(raw_evasion_d, ref_d_left, ref_d_right, idx_mid_in_es, idx_mid_in_obs)
            if raw_evasion_valid == False:
                return raw_evasion_d, np.array([]), np.array([]), np.array([]), False
            else:
                x_data, y_data, t_data = self.get_fit_variable(raw_evasion_s, raw_evasion_d, raw_evasion_v)
                start_qp_fit_time = time.time()
                poly_x, poly_y = self.get_qp_fit_coeffs(x_data, y_data, t_data, raw_evasion_s, raw_evasion_v)
                end_qp_fit_time = time.time()
                qp_fit_time = end_qp_fit_time - start_qp_fit_time
                print(f"qp_fit_time: {qp_fit_time*1000:.3f}ms")
                if len(poly_x) == 0 or len(poly_y) == 0:
                    return raw_evasion_d, np.array([]), np.array([]), np.array([]), False
                start_check_closed_time = time.time()
                closedform_evasion_valid = self.check_fit_traj_valid(poly_x, poly_y, t_data, ref_d_left)
                end_check_closed_time = time.time()
                check_closed_time = end_check_closed_time - start_check_closed_time
                print(f"check fit traj time: {check_closed_time*1000:.3f}ms")
                if closedform_evasion_valid == False:
                    # return np.array([]), np.array([]), np.array([]), False
                    return raw_evasion_d, poly_x, poly_y, t_data, False
        elif side == "right":
            mid_d, mid_ok = self._target_d(idx_mid_in_obs, ref_d_right[idx_mid_in_es], safe_dist, "right")
            start_d, _ = self._target_d(idx_start_in_obs, ref_d_right[idx_start_in_es], safe_dist, "right")
            end_d, _ = self._target_d(idx_end_in_obs, ref_d_right[idx_end_in_es], safe_dist, "right")
            if not mid_ok:
                print(f'[Right Overtake] infeasible: track too narrow (mid_d:{mid_d:.3f}, d_right:{ref_d_right[idx_mid_in_es]:.3f})')
                return np.array([]), np.array([]), np.array([]), np.array([]), False

            # Outside overtake curvature compensation: decrease start_d (more negative) to avoid cutting
            if is_outside_overtake and idx_start_in_es > 0:
                lon_dist_to_mid = (idx_mid_in_es - idx_start_in_es) * self.down_sampled_delta_s
                curvature_comp = curvature_gain * abs(avg_kappa) * lon_dist_to_mid * abs(mid_d)
                start_d = max(start_d - curvature_comp, -(ref_d_right[idx_start_in_es] - safe_dist))

            # Ensure that the lateral offsets at idx_start and idx_end are less than mid_d
            if idx_start_in_es == 0:
                start_d = current_d
            elif 0 < idx_start_in_es < 4:
                start_d = current_d + (idx_start_in_es / idx_mid_in_es) * (mid_d - current_d)
            else:
                start_d = max(start_d, mid_d)
            end_d = max(end_d, mid_d)
            print(f'[Right Overtake] start_d:{start_d:.3f}, mid_d:{mid_d:.3f}, end_d:{end_d:.3f}, outside:{is_outside_overtake}')
            print(f'[Merge Info] idx_end:{idx_end_in_es}, merge_points:{len(raw_evasion_s) - idx_end_in_es}, merge_dist:{(len(raw_evasion_s) - idx_end_in_es)*self.scaled_delta_s:.2f}m')

            # For the segments: 0 to idx_start, idx_start to idx_mid, idx_mid to idx_end, idx_end to len
            raw_evasion_d[0:idx_start_in_es] = np.linspace(current_d, start_d, num=idx_start_in_es)
            raw_evasion_d[idx_start_in_es:idx_mid_in_es] = np.linspace(start_d, mid_d, num=idx_mid_in_es - idx_start_in_es)
            raw_evasion_d[idx_mid_in_es:idx_end_in_es] = np.linspace(mid_d, end_d, num=idx_end_in_es - idx_mid_in_es)
            
            # Curvature-aware merge-back (right side)
            n_merge = len(raw_evasion_s) - idx_end_in_es
            if n_merge > 0:
                merge_curvature_conflict = (end_d > 0 and avg_kappa_merge > 0) or (end_d < 0 and avg_kappa_merge < 0)
                if merge_curvature_conflict:
                    t = np.linspace(0, 1, n_merge)
                    s_curve = 3 * t**2 - 2 * t**3
                    raw_evasion_d[idx_end_in_es:] = end_d * (1 - s_curve)
                else:
                    t = np.linspace(0, np.pi/2, n_merge)
                    raw_evasion_d[idx_end_in_es:] = end_d * np.cos(t)

            # check the raw d is not out of boundary
            # 경계로 눌러 넣고 장애물 여유만 재검증한다 (구: check_interp_traj_valid 로 전체 포기)
            raw_evasion_d, raw_evasion_valid = self._clamp_to_track(raw_evasion_d, ref_d_left, ref_d_right, idx_mid_in_es, idx_mid_in_obs)
            if raw_evasion_valid == False:
                return raw_evasion_d, np.array([]), np.array([]), np.array([]), False
            else:
                x_data, y_data, t_data = self.get_fit_variable(raw_evasion_s, raw_evasion_d, raw_evasion_v)
                start_qp_fit_time = time.time()
                poly_x, poly_y = self.get_qp_fit_coeffs(x_data, y_data, t_data, raw_evasion_s, raw_evasion_v)
                end_qp_fit_time = time.time()
                qp_fit_time = end_qp_fit_time - start_qp_fit_time
                print(f"qp_fit_time: {qp_fit_time*1000:.3f}ms")
                if len(poly_x) == 0 or len(poly_y) == 0:
                    return raw_evasion_d, np.array([]), np.array([]), np.array([]), False
                start_check_closed_time = time.time()
                closedform_evasion_valid = self.check_fit_traj_valid(poly_x, poly_y, t_data, ref_d_right)
                end_check_closed_time = time.time()
                check_closed_time = end_check_closed_time - start_check_closed_time
                print(f"check fit traj time: {check_closed_time*1000:.3f}ms")
                if closedform_evasion_valid == False:
                    # return np.array([]), np.array([]), np.array([]), False
                    return raw_evasion_d, poly_x, poly_y, t_data, False
        
        return raw_evasion_d, poly_x, poly_y, t_data, True
    def get_qp_fit_coeffs(self, x_data, y_data, t_data, raw_evasion_s, raw_evasion_v):
        phi_start = self.initial_ref_spline.get_phi(raw_evasion_s[0])
        phi_end = self.initial_ref_spline.get_phi(raw_evasion_s[-1])
        v_start_x = raw_evasion_v[0]*np.cos(phi_start)
        v_start_y = raw_evasion_v[0]*np.sin(phi_start)
        v_end_x = raw_evasion_v[-1]*np.cos(phi_end)
        v_end_y = raw_evasion_v[-1]*np.sin(phi_end)

        # Call C++ Closed-form Solver
        try:
            poly_x, poly_y = mpc_builder.solve_quintic_spline(
                t_data.astype(np.float64), 
                x_data.astype(np.float64), 
                y_data.astype(np.float64),
                v_start_x, v_start_y, v_end_x, v_end_y
            )
        except Exception as e:
            rospy.logerr(f"C++ Fit failed: {e}")
            return np.array([]), np.array([])

        return poly_x, poly_y
    def get_fit_variable(self, raw_evasion_s, raw_evasion_d, raw_evasion_v):
        waypts_resp = self.converter.get_cartesian(raw_evasion_s, raw_evasion_d)
        x_data = waypts_resp[0, :]
        y_data = waypts_resp[1, :] 
        
        # Call C++
        t_data = mpc_builder.calculate_path_time_cpp(
            x_data.astype(np.float64), 
            y_data.astype(np.float64), 
            raw_evasion_v.astype(np.float64)
        )
        return x_data, y_data, t_data
    
    def check_interp_traj_valid(self, raw_evasion_d, ref_d):
        safe_dist = 0.1
        return np.all(np.abs(raw_evasion_d) <= (np.abs(ref_d)- safe_dist))

    def check_fit_traj_valid(self, poly_x, poly_y, t_data, ref_d):
        safe_dist = 0.1
        x_fit = self.qp_fit.poly(t_data, poly_x)
        y_fit = self.qp_fit.poly(t_data, poly_y)
        resp = self.converter.get_frenet(x_fit, y_fit)
        s, d = resp[0, :], resp[1, :]
        return np.all(np.abs(d) <= (np.abs(ref_d)- safe_dist))
    
    ### Optimal Trajectory Solver ###
    def objective_function(self, d):
        return np.sum((d) ** 2) * 10  + np.sum(np.diff(np.diff(d))**2) * 100 + (np.diff(d)[0] ** 2) * 1000

    ## Constraint functions ##
    def start_on_raceline_constraint(self, d): # And end on raceline
        return np.array([0.02 - abs(d[0] - self.current_d), 0.02 - abs(d[-2]), 0.02 - abs(d[-1])])        

    def obstacle_constraint(self, d):
        distance_to_obstacle = np.abs(d[self.obs_downsampled_indices] - self.obs_downsampled_center_d)
        violation = distance_to_obstacle - self.obs_downsampled_min_dist
        return violation
    
    # Prevents points from jumping trhough obstacles due to resoultion isses
    def consecutive_points_constraint(self, d):
        # Extract the relevant points
        points = d[self.obs_downsampled_indices]
    
        # Check the condition for each pair of consecutive points
        violations = []
        for i in range(len(points) - 1):
            if not ((points[i] > self.obs_downsampled_center_d[i] and points[i+1] > self.obs_downsampled_center_d[i+1]) or
                    (points[i] < self.obs_downsampled_center_d[i] and points[i+1] < self.obs_downsampled_center_d[i+1])):
                violations.append(-1)  # Add a violation as a negative value if the condition is not met
            else:
                violations.append(1)
        return violations

    def turning_radius_constraint(self, d):
        # Calculate curvature at each point using numerical differentiation
        # k = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
        # x' = self.down_sampled_delta_s, x'' = 0
        
        y_prime = np.diff(d)
        y_prime = np.where(y_prime == 0, 1e-6, y_prime) # Avoid division by zero
        y_prime_prime = np.diff(y_prime)
        y_prime = y_prime[:-1] # Make it the same length as y_prime_prime
        
        kappa = (self.down_sampled_delta_s * y_prime_prime) / ((self.down_sampled_delta_s ** 2) ** (3/2))
        # np.diff losses last two points so we delete them from self.global_traj_kappas
        total_kappa = self.global_traj_kappas - kappa
        violation = self.max_kappa - abs(total_kappa)
        return violation
    
    # The arctan of of (d[1]-d[0])/ delta_s_sample_points < than 45 degrees
    def first_point_constraint(self, d):
        return np.array([self.down_sampled_delta_s - abs(d[1]-d[0])])

    def combined_equality_constraints(self, d):
        return self.start_on_raceline_constraint(d)

    def combined_inequality_constraints(self, d):
        return np.concatenate([self.obstacle_constraint(d), self.consecutive_points_constraint(d), self.turning_radius_constraint(d), self.first_point_constraint(d)]) #

    def solve_sqp(self, d_array, track_boundaries):
        result = minimize(
        self.objective_function, d_array, method='SLSQP', jac='10-point',
        bounds=track_boundaries,
        constraints=[
            {'type': 'eq', 'fun': self.combined_equality_constraints},
            {'type': 'ineq', 'fun': self.combined_inequality_constraints}
            ],
        options={'ftol': 1e-1, 'maxiter': 20, 'disp': False},
        )
        return result

    def group_objects(self, obstacles: list):
        # Group obstacles that are close to each other
        initial_guess_object = obstacles[0]
        for obs in obstacles:
            if obs.d_left > initial_guess_object.d_left:
                initial_guess_object.d_left = obs.d_left
            if obs.d_right < initial_guess_object.d_right:
                initial_guess_object.d_right = obs.d_right
            if obs.s_start < initial_guess_object.s_start:
                initial_guess_object.s_start = obs.s_start
            if obs.s_end > initial_guess_object.s_end:
                initial_guess_object.s_end = obs.s_end
        initial_guess_object.s_center = (initial_guess_object.s_start + initial_guess_object.s_end) / 2
        return initial_guess_object

    def _more_space(self, obstacle: Obstacle, gb_wpnts, gb_idxs):
        """회피할 side 와 초기 apex(= 차량 중심의 목표 d) 를 정한다.

        left_gap / right_gap 은 '장애물 끝 ~ 트랙 경계' 거리다. 차가 그 틈으로 들어가려면
        최소한 차폭이 필요한데, 예전 min_space 는 evasion_dist + spline_bound_mindist
        (= 0.15 + 0.05 = 0.20 m) 라서 **차폭(0.28 m)을 아예 세지 않았다**.
        그래서 차가 물리적으로 못 들어가는 쪽을 고르고, 그 다음 get_obs_pre_resp 가
        corridor infeasible 로 회피를 통째로 포기했다 = 반대쪽이 뚫려 있는데도 정지.

        apex 도 마찬가지로 obstacle.d_left + evasion_dist 였는데, 궤적의 d 는 차량 '중심'
        이므로 반차폭을 더해야 한다. 0.15 만 더하면 차체가 장애물에서 1 cm 떨어진 위치를
        목표로 삼게 된다.
        """
        left_boundary_mean = np.mean([gb_wpnts[gb_idx].d_left for gb_idx in gb_idxs])
        right_boundary_mean = np.mean([gb_wpnts[gb_idx].d_right for gb_idx in gb_idxs])
        left_gap = abs(left_boundary_mean - obstacle.d_left)
        right_gap = abs(right_boundary_mean + obstacle.d_right)

        # 차가 들어가기 위한 물리적 최소 폭 / 여유까지 확보한 이상적인 폭
        need_hard = self.width_car + self.spline_bound_mindist
        need_soft = need_hard + self.evasion_dist

        def apex(side):
            if side == "left":
                # 장애물 왼쪽 끝에서 (반차폭 + 여유) 만큼 떨어진 곳에 차량 중심을 둔다.
                # apex 가 raceline 반대편으로 넘어가지는 않게 clamp.
                return "left", max(obstacle.d_left + self.half_width + self.evasion_dist, 0.0)
            return "right", min(obstacle.d_right - self.half_width - self.evasion_dist, 0.0)

        left_ok = left_gap >= need_soft
        right_ok = right_gap >= need_soft

        if left_ok and not right_ok:
            return apex("left")
        if right_ok and not left_ok:
            return apex("right")
        if left_ok and right_ok:
            # 둘 다 여유가 충분하면 raceline 에서 덜 벗어나는 쪽
            side_l, d_l = apex("left")
            side_r, d_r = apex("right")
            return (side_l, d_l) if abs(d_l) <= abs(d_r) else (side_r, d_r)

        # 둘 다 soft 여유가 부족한 경우: 여기서 포기하지 않고 **더 넓은 쪽**으로 시도한다.
        # 진짜 통과 가능 여부는 아래 corridor 검사와 _trajectory_safe 가 판정한다.
        if max(left_gap, right_gap) < need_hard:
            rospy.logwarn_throttle(
                2.0,
                f"[SQP_Node] 양쪽 모두 차폭 부족 (left {left_gap:.2f} m / right {right_gap:.2f} m "
                f"< {need_hard:.2f} m) - 회피 불가 구간일 가능성이 높다"
            )
        return apex("left" if left_gap >= right_gap else "right")
    
    ### Visualize SQP Rviz###
    def visualize_sqp(self, evasion_s, evasion_d, evasion_x, evasion_y, evasion_v):
        mrks = MarkerArray()
        if len(evasion_s) == 0:
            pass
        else:
            resp = self.converter.get_cartesian(evasion_s, evasion_d)
            for i in range(len(evasion_s)):
                mrk = Marker(header=rospy.Header(stamp=rospy.Time.now(), frame_id="map"))
                mrk.type = mrk.CYLINDER
                mrk.scale.x = 0.1
                mrk.scale.y = 0.1
                mrk.scale.z = evasion_v[i] / self.scaled_vmax
                mrk.color.a = 1.0
                mrk.color.g = 0.13
                mrk.color.r = 0.63
                mrk.color.b = 0.94

                mrk.id = i
                mrk.pose.position.x = evasion_x[i]
                mrk.pose.position.y = evasion_y[i]
                mrk.pose.position.z = evasion_v[i] / self.scaled_vmax / 2
                mrk.pose.orientation.w = 1.0
                mrks.markers.append(mrk)
            self.mrks_pub.publish(mrks)

    def visualize_df(self, evasion_s, evasion_d, evasion_x, evasion_y, evasion_v):
        mrks = MarkerArray()
        if len(evasion_s) == 0:
            pass
        else:
            resp = self.converter.get_cartesian(evasion_s, evasion_d)
            for i in range(len(evasion_s)):
                mrk = Marker(header=rospy.Header(stamp=rospy.Time.now(), frame_id="map"))
                mrk.type = mrk.CYLINDER
                mrk.scale.x = 0.1
                mrk.scale.y = 0.1
                mrk.scale.z = evasion_v[i] / self.scaled_vmax
                mrk.color.a = 1.0
                mrk.color.g = 0.53
                mrk.color.r = 0.83
                mrk.color.b = 0.34

                mrk.id = i
                mrk.pose.position.x = evasion_x[i]
                mrk.pose.position.y = evasion_y[i]
                mrk.pose.position.z = evasion_v[i] / self.scaled_vmax / 2
                mrk.pose.orientation.w = 1.0
                mrks.markers.append(mrk)
            self.df_mrks_pub.publish(mrks)

    def visualize_mpc(self, evasion_s, evasion_d, evasion_x, evasion_y, evasion_v):
        mrks = MarkerArray()
        if len(evasion_s) == 0:
            pass
        else:
            for i in range(len(evasion_s)):
                mrk = Marker(header=rospy.Header(stamp=rospy.Time.now(), frame_id="map"))
                mrk.type = mrk.CYLINDER
                mrk.scale.x = 0.1
                mrk.scale.y = 0.1
                mrk.scale.z = evasion_v[i] / self.scaled_vmax
                mrk.color.a = 1.0
                mrk.color.g = 0.13
                mrk.color.r = 0.93
                mrk.color.b = 0.14

                mrk.id = i
                mrk.pose.position.x = evasion_x[i]
                mrk.pose.position.y = evasion_y[i]
                mrk.pose.position.z = evasion_v[i] / self.scaled_vmax / 2
                mrk.pose.orientation.w = 1.0
                mrks.markers.append(mrk)
            self.mpc_mrks_pub.publish(mrks)
    
    def visualize_ob_lines(self, start_points, end_points, color=[1.0, 0.0, 0.0], step=5):
        """
        Creates an RViz MarkerArray for lines connecting multiple points, with even sampling.
        
        :param step: The step size for sampling points (default: 10).
        """
        marker_array = MarkerArray()

        if len(start_points) == 0:
            return

        # Sample the points with the specified step size for visualization
        sampled_start_points = start_points[::step]  # Select points with a step interval
        sampled_end_points = end_points[::step]      # Select points with the same step interval

        for i in range(len(sampled_start_points)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "line_visualization"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0  # Alpha (transparency)

            marker.scale.x = 0.05  # Line width
            marker.pose.orientation.w = 1.0  # No rotation

            # Create points for the start and end of the line
            p1 = Point(x=float(sampled_start_points[i, 0]), y=float(sampled_start_points[i, 1]), z=0.0)
            p2 = Point(x=float(sampled_end_points[i, 0]), y=float(sampled_end_points[i, 1]), z=0.0)
            marker.points.append(p1)
            marker.points.append(p2)

            # Add the marker to the array
            marker_array.markers.append(marker)

        # Publish the marker array
        self.ob_line_pub.publish(marker_array)


        
    
    def visualize_raw_traj(self, evasion_s, evasion_d, evasion_x, evasion_y, evasion_v):
        mrks = MarkerArray()
        if len(evasion_s) == 0:
            pass
        else:
            mrk = Marker(header=rospy.Header(stamp=rospy.Time.now(), frame_id="map"))
            mrk.type = mrk.LINE_STRIP
            mrk.scale.x = 0.05
            mrk.color.a = 1.0
            mrk.color.r = 1.0

            for i in range(len(evasion_s)):
                point = Point()
                point.x = evasion_x[i]
                point.y = evasion_y[i]
                point.z = 0.0
                mrk.points.append(point)

            mrks.markers.append(mrk)
            self.raw_mrks_pub.publish(mrks)


    def initialize_converter(self) -> bool:
            """
            Initialize the FrenetConverter object"""
            # wait_for_message 는 별도 구독으로 받으므로 gb_cb 가 먼저 돌았다는 보장이 없다.
            # 받은 메시지를 직접 써서 기동 레이스를 없앤다.
            msg = rospy.wait_for_message("/global_waypoints", WpntArray)
            if self.global_waypoints is None:
                self.gb_cb(msg)

            # Initialize the FrenetConverter object
            converter = FrenetConverter(self.global_waypoints[:, 0], self.global_waypoints[:, 1], self.global_waypoints[:, 2])
            rospy.loginfo("[Spliner] initialized FrenetConverter object")

            return converter
    
    def initialize_spline(self):
        """
        Initialize the spline Converter object"""

        initial_ref_spline = InitialRefSpline()

        return initial_ref_spline

if __name__ == "__main__":
    SQPAvoidance = SQPAvoidanceNode()
    SQPAvoidance.loop()
