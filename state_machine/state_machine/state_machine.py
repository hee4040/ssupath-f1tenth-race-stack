import os
import yaml
import rclpy
import numpy as np
from typing import List
from builtin_interfaces.msg import Time
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from ament_index_python import get_package_share_directory
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from f110_msgs.msg import Wpnt, WpntArray, OTWpntArray, ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray
try:
    from vesc_msgs.msg import VescStateStamped
except:
    pass

from state_machine.transitions import dummy_transition, timetrials_transition, head_to_head_transition
from state_machine.state_types import StateType
from state_machine.states import DefaultStateLogic
from stack_master.parameter_event_handler import ParameterEventHandler
from state_machine.state_machine_params import StateMachineParams

def time_to_float(time_instant: Time):
    return time_instant.sec + time_instant.nanosec * 1e-9

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        # PARAMETER DECLARATION
        self.params = StateMachineParams(self)
        
        self.ftg_disabled = True # TODO fix with global prams?
        self.cur_vs: float = 0.0  # car_state_frenet_cb 가 채운다. 콜백 전 접근 대비 초기화.

        # update on parameter changes for rate
        self.add_on_set_parameters_callback(self.params.parameters_callback)
        
        # SUBSCRIPTIONS
        if self.params.test_on_car:
            self.battery_sub = self.create_subscription(VescStateStamped, "/vesc/sensors/core", self.battery_cb, 10)
        else:
            self.battery_level = self.params.volt_threshold + 1
        self.max_speed = None
        self.create_subscription(WpntArray, "/global_waypoints", self.glb_wpnts_cb, 10)
        while self.max_speed is None: # equivalent of wait for message
            self.get_logger().info("Waiting for global waypoints message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)

        
        self.glb_wpnts = None
        self.num_glb_wpnts = 0
        self.track_length = 1
        self.gb_max_idx = 10
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.glb_wpnts_scaled_cb, 10)
        while self.glb_wpnts is None: # equivalent of wait for message
            self.get_logger().info("Waiting for scaled global waypoints message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)

        
        self.cur_s = None
        self.cur_d = None
        self.create_subscription(Odometry, '/car_state/frenet/odom',self.car_state_frenet_cb, 10) # car frenet coordinates
        while self.cur_s is None:
            self.get_logger().info("Waiting for car state frenet message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)

        if self.params.mode == 'head_to_head':
            self.current_position = None
            self.create_subscription(PoseStamped, '/car_state/pose', self.car_state_cb, 10)
            while self.current_position is None:
                self.get_logger().info("Waiting for car state pose message", throttle_duration_sec=0.5)
                rclpy.spin_once(self)

            # self.get_logger().info("hiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii")
            self.last_valid_avoidance_wpnts = None
            self.splini_ttl_counter = 0
            self.avoidance_wpnts = None
            self.create_subscription(OTWpntArray, '/planner/avoidance/otwpnts', self.avoidance_cb, 10)

            self.obstacles = []
            self.create_subscription(ObstacleArray, "/perception/obstacles", self.obstacles_cb, 10)

            # 정지 복구(후진)용 중심선. global_trajectory_publisher 가 5 초마다 재발행하므로
            # 여기서 기다리지 않는다 — 못 받으면 recover 를 그냥 비활성으로 둔다.
            # ★ 중심선의 s 파라미터화는 레이스라인과 다르다(lobby_0819: 45.0 m vs 40.45 m).
            #   그래서 cur_s 로 중심선을 인덱싱하면 안 되고, 반드시 (x, y) 최근접으로 찾는다.
            self.center_wpnts = None
            self.center_xy = None
            self.center_dist = 0.1
            self.create_subscription(WpntArray, '/centerline_waypoints', self.centerline_wpnts_cb, 10)

            # 정지 복구 상태값
            self.stall_counter = 0
            self.recover_armed = True
            self.recover_start_position = None
            self.recover_start_time = 0.0
            self.recover_end_s = None
            self.recover_idx_step = -1
            self.recover_warned_no_centerline = False
        
            # TODO setup things for sectors and overtaking sectors
            self.only_ftg_zones = []
            self.ftg_counter = 0

            self.overtake_zones = []

        # INITIALIZATIONS        
        self.waypoints_dist = 0.1
        self.state = StateType(self.params.initial_state)
        self.local_waypoints = WpntArray()
        self.first_visualization = True
        self.x_viz = 0
        self.y_viz = 0

        # choose how to transition between states
        if self.params.mode == 'dummy':
            self.state_transition = dummy_transition
        elif self.params.mode == 'timetrials':
            self.state_transition = timetrials_transition
        elif self.params.mode == 'head_to_head':
            # self.get_logger().info("hiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii")
            self.state_transition = head_to_head_transition
        else:
            raise NotImplementedError(f"Mode {self.params.mode} not recognized")
                
        # choose what to do in the different states
        self.state_logic = DefaultStateLogic
            
        # PUBLICATIONS
        self.state_pub = self.create_publisher(String, 'state', 10)
        self.state_marker_pub = self.create_publisher(Marker, 'state_marker', 10)
        self.loc_wpnt_pub = self.create_publisher(WpntArray, 'local_waypoints', 10)
        self.vis_loc_wpnt_pub = self.create_publisher(MarkerArray, 'local_waypoints/markers', 10)
        # OT sector 안에 있는지 여부. 오버테이킹 플래너(FSDP 등)가 이 값으로 회피 궤적 계산을 게이팅한다.
        self.ot_section_check_pub = self.create_publisher(Bool, 'ot_section_check', 10)

        # 카운터 초기화
        self.trailing_to_gbtrack_count = 0
        self.trailing_to_gbtrack_counting_threshold = self.params.Trailing_to_GBtrack_counting_threshold

        # set up ot param reading
        # init_ot_params() 안에서 spin 이 돌기 때문에, main loop 타이머보다 반드시 먼저 끝나야 한다.
        # (먼저 타이머를 걸면 ot_sectors 가 None 인 채로 transition 이 돌아 죽는다)
        self.init_ot_params()

        # main loop
        self.main_loop = self.create_timer(1/self.params.rate_hz, self.main_loop_callback)

    #############
    # CALLBACKS #
    #############
    def battery_cb(self, msg: VescStateStamped):
        self.battery_level = msg.state.voltage_input

    def glb_wpnts_scaled_cb(self, data: WpntArray):
        """
        Callback function of velocity interpolator subscriber.

        Parameters
        ----------
        data
            Data received from velocity interpolator topic
        """
        self.glb_wpnts = data.wpnts[:-1]  # exclude last point (because last point == first point)
        self.num_glb_wpnts = len(self.glb_wpnts)
        self.track_length = data.wpnts[-1].s_m
        # Get spacing between wpnts for rough approximations
        self.wpnt_dist = data.wpnts[1].s_m - data.wpnts[0].s_m
        self.gb_max_idx = data.wpnts[-1].id

    def glb_wpnts_cb(self, data: WpntArray):
        self.max_speed = max([wpnt.vx_mps for wpnt in data.wpnts])

    def car_state_frenet_cb(self, msg: Odometry):
        self.cur_s = msg.pose.pose.position.x
        self.cur_d = msg.pose.pose.position.y
        # cur_vs 는 _check_ftg 와 고속 감속 배율(_highspeed_brake_factor)이 쓴다.
        # 2026-08-04 이전에는 '읽히기만 하고 어디서도 할당되지 않는' 상태였다.
        # ftg_disabled=True 라 _check_ftg 가 조기 return 해서 AttributeError 가
        # 드러나지 않았을 뿐이다. FTG 를 켜는 순간 터졌을 잠복 버그.
        self.cur_vs = msg.twist.twist.linear.x

    def avoidance_cb(self, data: OTWpntArray):
        """Subscribes to spliner waypoints"""
        if len(data.wpnts) > 0:
            self.splini_ttl_counter = int(self.params.splini_ttl * self.params.rate_hz)
            self.avoidance_wpnts = data
        else:
            # self.get_logger().info("no_____waypoints receiveddddddddddddddddddddddddddddddddddddddddddddd")
        # If empty we don't overwrite the avoidance waypoints
            pass

    def obstacles_cb(self, data):
        if len(data.obstacles) != 0:
            self.obstacles = data.obstacles
        else:
            self.obstacles = []
    
    def centerline_wpnts_cb(self, data: WpntArray):
        """중심선 웨이포인트를 받아 후진 복구용으로 보관한다."""
        wpnts = data.wpnts
        if len(wpnts) < 3:
            return
        # 닫힌 루프면 마지막 점이 첫 점과 겹치므로 하나 뺀다 (glb_wpnts 와 동일한 처리)
        if (abs(wpnts[-1].x_m - wpnts[0].x_m) < 1e-6 and abs(wpnts[-1].y_m - wpnts[0].y_m) < 1e-6):
            wpnts = wpnts[:-1]
        self.center_wpnts = wpnts
        self.center_xy = np.array([[w.x_m, w.y_m] for w in wpnts])
        ds = wpnts[1].s_m - wpnts[0].s_m
        self.center_dist = ds if ds > 1e-3 else 0.1

    def car_state_cb(self, data: PoseStamped):
        x = data.pose.position.x
        y = data.pose.position.y
        rot = Rotation.from_quat([data.pose.orientation.x, data.pose.orientation.y, 
                                       data.pose.orientation.z, data.pose.orientation.w])
        rot_euler = rot.as_euler('xyz', degrees=False)
        theta = rot_euler[2]

        self.current_position = [x, y, theta]
    
    ### Properties to evaulate state transitions
    @property
    def _low_bat(self)->bool:
        if self.battery_level < self.params.volt_threshold:
            return True
        else:
            return False
    
    @property
    def _check_only_ftg_zone(self) -> bool:
        ftg_only = False
        # check if the car is in a ftg only zone, but only if there is an only ftg zone
        if len(self.only_ftg_zones) != 0:
            for sector in self.only_ftg_zones:
                if sector[0] <= self.cur_s / self.waypoints_dist <= sector[1]:
                    ftg_only = True
                    # rospy.logwarn(f"[{self.name}] IN FTG ONLY ZONE")
                    break  # cannot be in two ftg zones
        return ftg_only

    @property
    def _check_close_to_raceline(self) -> bool:
        return np.abs(self.cur_d) < self.params.gb_ego_width_m  # [m]

    @property
    def _check_ot_sector(self) -> bool:
        if not self.ot_sectors:  # 아직 못 받았거나 OT 섹터가 없는 맵
            return False
        for sector in self.ot_sectors:
            if sector['ot_flag']:
                if (sector['start'] <= self.cur_s / self.waypoints_dist <= (sector['end']+1)):
                    return True
        return False

    @property
    def _check_ofree(self) -> bool:
        o_free = True

        if self.params.overtake_mode == "spliner":
            if self.last_valid_avoidance_wpnts is not None:
                # self.get_logger().info(f"O_FREE False, obs dist to ot lane: {ot_obs_dist} m")
                horizon = self.params.overtaking_horizon_m  # Horizon in front of cur_s [m]

                for obs in self.obstacles:
                    obs_s = obs.s_center
                    # Wrapping madness to check if infront
                    dist_to_obj = (obs_s - self.cur_s) % self.track_length
                    if dist_to_obj < horizon and len(self.last_valid_avoidance_wpnts):
                        obs_d = obs.d_center
                        # Get d wrt to mincurv from the overtaking line
                        avoid_wpnt_idx = np.argmin(
                            np.array([abs(avoid_s.s_m - obs_s) for avoid_s in self.last_valid_avoidance_wpnts])
                        )
                        ot_d = self.last_valid_avoidance_wpnts[avoid_wpnt_idx].d_m
                        ot_obs_dist = ot_d - obs_d
                        # ot_obs_dist 는 "회피선 <-> 장애물 중심" 거리이므로 장애물 반폭을 빼야
                        # 실제 표면까지의 여유가 된다. 예전에는 장애물 크기를 무시하고
                        # 0.1 m 와만 비교해서, 장애물을 스치는 회피선도 전부 통과시켰다.
                        required = obs.size / 2.0 + self.params.lateral_width_ot_m
                        if abs(ot_obs_dist) < required:
                            o_free = False
                            # print("o_freeeeeeeeeee:" ,abs(ot_obs_dist))
                            # self.get_logger().info(f"O_FREE False, obs dist to ot lane: {ot_obs_dist} m")
                            break
            else:
                o_free = True
            return o_free
        else:
            self.get_logger().error(f"Unknown overtake planner")
            raise NotImplementedError

    @property
    def _check_gbfree(self) -> bool:
        gb_free = True
        # If we are in time trial only mode -> return free overtake i.e. GB_FREE True
        horizon = self.params.gb_horizon_m  # Horizon in front of cur_s [m]

        for obs in self.obstacles:
            obs_s = (obs.s_start + obs.s_end) / 2
            obs_s = obs.s_center
            gap = (obs_s - self.cur_s) % self.track_length
            if gap < horizon:
                obs_d = obs.d_center
                # print("gap is small tan horizon!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                # Get d wrt to mincurv from the overtaking line
                # if abs(obs_d -self.cur_d) < self.params.lateral_width_gb_m:
                if abs(obs_d) < self.params.lateral_width_gb_m:
                    gb_free = False
                    #self.get_logger().info(f"GB_FREE False, obs dist to ot lane: {obs_d} m")
                    break

        return gb_free

    @property
    def _check_enemy_in_front(self) -> bool:
        horizon = self.params.gb_horizon_m  # Horizon in front of cur_s [m]
        for obs in self.obstacles:
            gap = (obs.s_start - self.cur_s) % self.track_length
            if gap < horizon:
                return True
        return False

    @property
    def _check_availability_splini_wpts(self) -> bool:
        if self.avoidance_wpnts is None:
            # print("case11111111111111111111111111111111111111")

            self.get_logger().info
            return False
        elif len(self.avoidance_wpnts.wpnts) == 0: #이거땜에 추월이 안되네..
            # print("case22222222222222222222222222222222222222")
            return False
        # Say no to the ot line if the last switch was less than 0.75 seconds ago
        elif (abs(time_to_float(self.avoidance_wpnts.header.stamp) - time_to_float(self.avoidance_wpnts.last_switch_time))< self.params.splini_hyst_timer_sec):
            self.get_logger().debug(f"Still too fresh into the switch...{abs(time_to_float(self.avoidance_wpnts.last_switch_time) - time_to_float(self.get_clock().now().to_msg()))}")
            # print("case33333333333333333333333333333333333333")
            # print(abs(time_to_float(self.avoidance_wpnts.header.stamp) - time_to_float(self.avoidance_wpnts.last_switch_time)))

            return False
        else:
            # If the splinis are valid update the last valid ones
            self.last_valid_avoidance_wpnts = self.avoidance_wpnts.wpnts.copy()
            return True

    @property
    def _check_ftg(self) -> bool:
        # If we have been standing still for 3 seconds inside TRAILING -> FTG
        threshold = self.params.ftg_timer_sec * self.params.rate_hz
        if self.ftg_disabled:
            return False
        else:
            if self.cur_state == StateType.TRAILING and self.cur_vs < self.params.ftg_threshold_speed:
                self.ftg_counter += 1
                self.get_logger().warn(f"[{self.name}] FTG counter: {self.ftg_counter}/{threshold}")
            else:
                self.ftg_counter = 0

            if self.ftg_counter > threshold:
                return True
            else:
                return False

    @property
    def _check_emergency_break(self) -> bool:
        # NOTE: unused flag, but could be useful
        emergency_break = False
        if self.obstacles != []:
            horizon = self.emergency_break_horizon # Horizon in front of cur_s [m]

            for obs in self.obstacles:
                # Wrapping madness to check if infront
                dist_to_obj = (obs.s_center - self.cur_s) % self.track_length
                if dist_to_obj < horizon:
            
                    # Get d wrt to mincurv from the overtaking line
                    local_wpnt_idx = np.argmin(
                        np.array([abs(avoid_s.s_m - obs.s_center) for avoid_s in self.local_waypoints.wpnts])
                    )
                    ot_d = self.local_waypoints.wpnts[local_wpnt_idx].d_m
                    ot_obs_dist = ot_d - obs.d_center # 추월 경로의 d값과 장애물의 중심 d값의 차를 구한다
                    if abs(ot_obs_dist) < self.params.lateral_width_ot_m:
                        emergency_break = True
                        self.get_logger().info("emergency break")
        else:
            emergency_break = False
        return emergency_break
    # ---- 정지 복구(후진) ---------------------------------------------------
    # 로컬 플래너가 해를 못 내(otwpnts 빈 배열) TRAILING 에 갇혀 멈춰 있을 때, 중심선을 따라
    # recover_distance_m 만큼 물러난 뒤 기존 로직을 다시 태운다. 후방 센싱이 없으므로
    # (passthrough 크롭이 전방만) 아래 검사는 전부 기하학적이다.

    @property
    def _check_recover_needed(self) -> bool:
        """RECOVER 로 진입해야 하는지. TRAILING 전이에서만 호출된다."""
        p = self.params
        if not p.recover_enabled or not self.recover_armed:
            return False
        if self.center_wpnts is None:
            if not self.recover_warned_no_centerline:
                self.recover_warned_no_centerline = True
                self.get_logger().warn(
                    "[RECOVER] /centerline_waypoints 를 못 받아 정지 복구가 비활성 상태다")
            return False
        if self.stall_counter < int(p.recover_stall_time_sec * p.rate_hz):
            return False
        if not self._check_recover_path_free:
            self.get_logger().warn(
                "[RECOVER] 정지 조건은 맞지만 후진 경로가 좁거나 뒤가 막혀 후진하지 않는다",
                throttle_duration_sec=2.0)
            return False
        return True

    @property
    def _check_recover_path_free(self) -> bool:
        """후진 경로가 트랙 안이고, 아는 한 뒤가 비어 있는지 (기하학적 검사만)."""
        p = self.params
        if self.center_wpnts is None or self.current_position is None:
            return False

        # 1) 후진하며 지나갈 중심선 점들이 전부 충분히 넓은가
        n_center = len(self.center_wpnts)
        n_back = int(p.recover_distance_m / self.center_dist + 0.5)
        i0 = self._nearest_center_idx(self.current_position[0], self.current_position[1])
        step = self._center_backward_step(i0)
        for k in range(n_back + 1):
            w = self.center_wpnts[(i0 + step * k) % n_center]
            if min(w.d_left, w.d_right) < p.recover_edge_margin_m:
                return False

        # 2) 뒤에 아는 장애물이 있으면 포기한다. 퍼셉션이 후방을 못 보므로 보통 비어 있지만,
        #    트래킹이 아직 들고 있는 장애물이 있으면 그건 믿을 만한 정보다.
        back_horizon = p.recover_distance_m + 0.5
        for obs in self.obstacles:
            behind = (self.cur_s - obs.s_center) % self.track_length
            if behind < back_horizon and abs(obs.d_center) < p.lateral_width_gb_m:
                return False
        return True

    @property
    def _check_recover_finished(self) -> bool:
        """후진을 끝낼 때가 됐는지 (거리 도달 또는 타임아웃)."""
        p = self.params
        if self.recover_start_position is None or self.current_position is None:
            return True
        elapsed = self._now_sec() - self.recover_start_time
        if elapsed > p.recover_timeout_sec:
            self.get_logger().warn(
                f"[RECOVER] 타임아웃 {elapsed:.1f}s - 후진 종료 (이동 {self._recover_travelled():.2f} m)")
            return True
        return self._recover_travelled() >= p.recover_distance_m

    ###########
    # HELPERS #
    ###########
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _nearest_center_idx(self, x: float, y: float) -> int:
        """중심선에서 (x, y) 에 가장 가까운 인덱스. s 파라미터화가 레이스라인과 다르므로
        cur_s 로 인덱싱하면 안 되고 반드시 유클리드 최근접으로 찾아야 한다."""
        d2 = (self.center_xy[:, 0] - x) ** 2 + (self.center_xy[:, 1] - y) ** 2
        return int(np.argmin(d2))

    def _center_backward_step(self, i0: int) -> int:
        """중심선 인덱스를 어느 쪽으로 세어야 '차 뒤쪽'인지 (-1 또는 +1).

        보통 인덱스 증가 = 주행 방향이라 -1 이지만, 맵에 따라 뒤집혀 있을 수 있으므로
        차의 헤딩과 중심선 접선을 실제로 비교해서 정한다.
        """
        n = len(self.center_wpnts)
        tangent = self.center_xy[(i0 + 1) % n] - self.center_xy[(i0 - 1) % n]
        theta = self.current_position[2]
        heading = np.array([np.cos(theta), np.sin(theta)])
        return -1 if float(np.dot(tangent, heading)) >= 0.0 else 1

    def _recover_travelled(self) -> float:
        """후진 시작 지점에서의 유클리드 변위. s 의 wrap 을 신경 쓰지 않아도 된다."""
        if self.recover_start_position is None or self.current_position is None:
            return 0.0
        return float(np.hypot(self.current_position[0] - self.recover_start_position[0],
                              self.current_position[1] - self.recover_start_position[1]))

    def _recover_start(self):
        """RECOVER 진입 시 1회. 시작 자세/시각과 후진 방향을 고정한다."""
        self.recover_start_position = list(self.current_position)
        self.recover_start_time = self._now_sec()
        i0 = self._nearest_center_idx(self.current_position[0], self.current_position[1])
        # 방향은 진입 시점에 고정한다. 후진 중 헤딩이 흔들려도 방향이 뒤집히면 안 된다.
        self.recover_idx_step = self._center_backward_step(i0)
        self.stall_counter = 0
        self.get_logger().warn(
            f"[RECOVER] 정지 복구 시작: 중심선을 따라 {self.params.recover_distance_m:.2f} m 후진 "
            f"(s={self.cur_s:.2f}, d={self.cur_d:.2f}, center_idx={i0}, step={self.recover_idx_step})")

    def _recover_end(self):
        """RECOVER 이탈 시 1회. 재무장을 잠근다 (같은 자리에서 1회만 후진)."""
        self.get_logger().warn(
            f"[RECOVER] 정지 복구 종료: {self._recover_travelled():.2f} m 후진함. "
            f"전방으로 {self.params.recover_rearm_dist_m:.1f} m 진행하기 전에는 다시 후진하지 않는다")
        self.recover_start_position = None
        self.recover_end_s = self.cur_s
        self.recover_armed = False

    def _update_recover_bookkeeping(self):
        """매 사이클 맨 앞에서 호출. 정지 카운터와 재무장 여부를 갱신한다."""
        p = self.params

        # 정지 판정: TRAILING + 거의 안 움직임 + 회피 웨이포인트 없음 + 전방에 장애물.
        # 전방 장애물 조건이 있어야 출발 대기 중이거나 사람이 세워 둔 상황에서 후진하지 않는다.
        no_solution = (self.avoidance_wpnts is None or len(self.avoidance_wpnts.wpnts) == 0)
        if (self.state == StateType.TRAILING
                and abs(self.cur_vs) < p.recover_stall_speed_mps
                and no_solution
                and not self._check_gbfree):
            self.stall_counter += 1
            # 재무장이 잠겨 있으면(=이미 한 번 후진했으면) 어차피 후진하지 않으므로
            # 매초 찍지 않는다. 그대로 두면 남은 주행 내내 콘솔을 덮어버린다.
            if self.recover_armed and self.stall_counter % int(p.rate_hz) == 0:
                self.get_logger().warn(
                    f"[RECOVER] 정지 지속 {self.stall_counter / p.rate_hz:.1f}s "
                    f"/ {p.recover_stall_time_sec:.1f}s (해 없음, 전방 장애물 있음)")
            elif not self.recover_armed:
                self.get_logger().warn(
                    "[RECOVER] 정지했지만 이미 이 구간에서 후진을 썼다 - 재무장 대기 중",
                    throttle_duration_sec=5.0)
        elif self.state != StateType.RECOVER:
            self.stall_counter = 0

        # 재무장: 후진했던 지점에서 전방으로 recover_rearm_dist_m 이상 실제로 진행했을 때만.
        # 뒤로 살짝 밀린 것이 wrap 때문에 '거의 한 바퀴 전진'으로 보이지 않도록 상한을 둔다.
        if not self.recover_armed and self.recover_end_s is not None:
            progressed = (self.cur_s - self.recover_end_s) % self.track_length
            if p.recover_rearm_dist_m <= progressed < 0.5 * self.track_length:
                self.recover_armed = True
                # 잠겨 있는 동안 쌓인 카운터를 그대로 두면 재무장 즉시 후진해 버린다.
                # 재무장 후에도 recover_stall_time_sec 을 처음부터 다시 채우게 한다.
                self.stall_counter = 0
                self.get_logger().info(
                    f"[RECOVER] 전방으로 {progressed:.1f} m 진행 - 정지 복구 재무장")

    def get_recover_wpnts(self) -> List[Wpnt]:
        """후진용 로컬 웨이포인트. 배열 0번이 차에서 가장 가까운 중심선 점이고,
        인덱스가 커질수록 차 뒤쪽으로 간다. 컨트롤러(PP)의 RECOVER 분기가 이 순서를 전제로
        인덱스를 키워 후방 lookahead 점을 잡는다.

        속도는 음수로 실어 보낸다 (= 후진 명령). 시작 직후 recover_dwell_sec 동안은 0 을
        내보내 VESC 속도 폐루프가 0 에서 한 번 안정된 뒤 방향을 바꾸게 한다.
        """
        p = self.params
        if self.center_wpnts is None or self.current_position is None:
            return []

        elapsed = self._now_sec() - self.recover_start_time
        speed = 0.0 if elapsed < p.recover_dwell_sec else -abs(p.recover_speed_mps)

        n_center = len(self.center_wpnts)
        # 매 사이클 현재 위치에서 다시 만든다. 그래야 lookahead 거리가 일정하게 유지되고,
        # 후진하는 동안 차가 중심선 위로 수렴한다(= d 가 0 으로 정렬된다).
        i0 = self._nearest_center_idx(self.current_position[0], self.current_position[1])
        step = self.recover_idx_step

        out: List[Wpnt] = []
        for k in range(p.recover_n_wpnts):
            src = self.center_wpnts[(i0 + step * k) % n_center]
            wp = Wpnt()
            wp.id = k
            wp.s_m = src.s_m
            wp.d_m = src.d_m
            wp.x_m = src.x_m
            wp.y_m = src.y_m
            wp.d_right = src.d_right
            wp.d_left = src.d_left
            wp.psi_rad = src.psi_rad
            wp.kappa_radpm = src.kappa_radpm
            wp.ax_mps2 = 0.0
            wp.vx_mps = speed
            out.append(wp)
        return out

    def get_splini_wpts(self) -> WpntArray:
        """Obtain the waypoints by fusing those obtained by spliner with the
        global ones.
        """
        splini_glob = self.glb_wpnts.copy()

        # Handle wrapping
        if self.last_valid_avoidance_wpnts is not None:
            if self.last_valid_avoidance_wpnts[-1].s_m > self.last_valid_avoidance_wpnts[0].s_m:
                splini_idxs = [
                    s
                    for s in range(
                        int(self.last_valid_avoidance_wpnts[0].s_m / self.waypoints_dist + 0.5),
                        int(self.last_valid_avoidance_wpnts[-1].s_m / self.waypoints_dist + 0.5),
                    )
                ]
            else:
                splini_idxs = [
                    int(s % (self.track_length / self.waypoints_dist) + 0.5)
                    for s in range(
                        int(self.last_valid_avoidance_wpnts[0].s_m / self.waypoints_dist + 0.5),
                        int((self.track_length + self.last_valid_avoidance_wpnts[-1].s_m) / self.waypoints_dist + 0.5),
                    )
                ]

            # with self.lock:  # was needed in ROS1 but hopefuly in ROS2 were fine
            for i, s in enumerate(splini_idxs):
                # splini_glob[s] = self.last_valid_avoidance_wpnts[i]
                splini_glob[s] = self.last_valid_avoidance_wpnts[min(i, len(self.last_valid_avoidance_wpnts) - 1)]

        # If the last valid points have been reset, then we just pass the global waypoints
        else:
            self.get_logger().warn(f"No valid avoidance waypoints, passing global waypoints")
            pass

        return splini_glob

    def init_ot_params(self):
        """Obtain the initial overtaking parameters from the parameter server.
        Then instantiate a parameter event handler to get the updates on those parameters.
        """
        self.parameter_client = self.create_client(GetParameters, '/ot_interpolator/get_parameters')
        self.parameter_client.wait_for_service()
        self.n_ot_sectors = None
        self.ot_param_names = None
        self.ot_sectors = None
        
        request = GetParameters.Request()
        request.names = ['n_sectors']
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # TODO structure of request is also very ugly dependent on the parameter structure on the other side, could be improved
        if future.result() is not None:
            self.n_ot_sectors = future.result().values[0].integer_value # TODO careful with this types, TY util could be better
            self.get_logger().info(f'Got {self.n_ot_sectors} sectors')

            starts = [f'Overtaking_sector{i}.start' for i in range(self.n_ot_sectors)]
            ends = [f'Overtaking_sector{i}.end' for i in range(self.n_ot_sectors)]
            ot_flags = [f'Overtaking_sector{i}.ot_flag' for i in range(self.n_ot_sectors)]
            self.ot_param_names = ot_flags
            self.ot_sectors = []

            request = GetParameters.Request()
            request.names = starts + ends + ot_flags
            self.get_logger().info(f'Getting OT params: {request.names}')
            future = self.parameter_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                for i in range(self.n_ot_sectors):
                    start = future.result().values[i].integer_value # TODO careful with this types, TY util could be better
                    end = future.result().values[i + self.n_ot_sectors].integer_value # TODO careful with this types, TY util could be better
                    ot_flag = future.result().values[i + 2 * self.n_ot_sectors].bool_value # TODO careful with this types, TY util could be better
                    self.ot_sectors.append({'start': start, 'end': end, 'ot_flag': ot_flag})
                self.get_logger().info(f'OT sectors obtained: {self.ot_sectors}')
            else:
                self.get_logger().error(f'Service call failed {future.exception()}')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
        
        # once initialization is finished, the event handler can be used
        self.handler = ParameterEventHandler(self)

        for ot_flag in ot_flags:
            self.handler.add_parameter_callback(
                ot_flag,
                'ot_interpolator',
                callback=self.ot_flag_cb
            )

    def ot_flag_cb(self, p: Parameter):
        self.get_logger().info(f'OT flag changed: {p.name} {p.value.bool_value}')
        changed_sector_int = int(p.name.split('.')[0][-1]) # TODO ultra hardcoded ugliness
        self.ot_sectors[changed_sector_int]['ot_flag'] = p.value.bool_value # TODO careful with this types, TY util could be better

    def get_ot_params(self):
        # the first time we need to get the number of sectors
        self.get_logger().info(f'Getting only ot flags params')
        # otherwise we only update the flags
        request = GetParameters.Request()
        request.names = self.ot_param_names
        future = self.parameter_client.call_async(request)
        self.executor.spin_until_future_complete(future)
        if future.result() is not None:
            for i in range(self.n_ot_sectors):
                self.ot_sectors[i]['ot_flag'] = future.result().values[i].bool_value
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')

        self.get_logger().info(f'OT sectors: {self.ot_sectors}')

    ###############
    # PUB HELPERS #
    ###############
    def _pub_local_waypoints(self, wpts: WpntArray):
        loc_markers = MarkerArray()
        loc_wpnts = wpts
        # set stamp to now         
        loc_wpnts.header.stamp = self.get_clock().now().to_msg()
        loc_wpnts.header.frame_id = "map"

        for i, wpnt in enumerate(loc_wpnts.wpnts):
            mrk = Marker()
            mrk.header.frame_id = "map"
            mrk.type = mrk.SPHERE
            mrk.scale.x = 0.15
            mrk.scale.y = 0.15
            mrk.scale.z = 0.15
            mrk.color.a = 1.0
            mrk.color.g = 1.0

            mrk.id = i
            mrk.pose.position.x = wpnt.x_m
            mrk.pose.position.y = wpnt.y_m
            mrk.pose.position.z = wpnt.vx_mps / self.max_speed  # Visualise speed in z dimension
            mrk.pose.orientation.w = 1.0
            loc_markers.markers.append(mrk)

        # ...

        if len(loc_wpnts.wpnts) == 0:
            self.get_logger().warn("No local waypoints published...")
        else:
            self.loc_wpnt_pub.publish(loc_wpnts)

        self.vis_loc_wpnt_pub.publish(loc_markers)
    
    def visualize_state(self, state: StateType):
        """
        Function that visualizes the state of the car by displaying a colored cube in RVIZ.

        Parameters
        ----------
        action
            Current state of the car to be displayed
        """
        if self.first_visualization:
            self.first_visualization = False
            x0 = self.glb_wpnts[0].x_m
            y0 = self.glb_wpnts[0].y_m
            x1 = self.glb_wpnts[1].x_m
            y1 = self.glb_wpnts[1].y_m
            # compute normal vector of 125% length of trackboundary but to the left of the trajectory
            xy_norm = (
                -np.array([y1 - y0, x0 - x1]) / np.linalg.norm([y1 - y0, x0 - x1]) * 1.25 * self.glb_wpnts[0].d_left
            )

            self.x_viz = x0 + xy_norm[0]
            self.y_viz = y0 + xy_norm[1]

        mrk = Marker()
        mrk.type = mrk.SPHERE
        mrk.id = int(1)
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.color.a = 1.0
        mrk.color.g = 1.0
        mrk.pose.position.x = float(self.x_viz)
        mrk.pose.position.y = float(self.y_viz)
        mrk.pose.position.z = 0.0
        mrk.pose.orientation.w = 1.0
        mrk.scale.x = 1.0
        mrk.scale.y = 1.0
        mrk.scale.z = 1.0

        # Set color and log info based on the state of the car
        if state == StateType.GB_TRACK:
            mrk.color.g = 1.0
        elif state == StateType.OVERTAKE:
            mrk.color.r = 1.0
            mrk.color.g = 1.0
            mrk.color.b = 1.0
        elif state == StateType.TRAILING:
            mrk.color.r = 0.0
            mrk.color.g = 0.0
            mrk.color.b = 1.0
        elif state == StateType.FTGONLY:
            mrk.color.r = 1.0
            mrk.color.g = 0.0
            mrk.color.b = 0.0
        elif state == StateType.TRAILING_TO_GBTRACK: # 주황색
            mrk.color.r = 1.0
            mrk.color.g = 0.5
            mrk.color.b = 0.0
        elif state == StateType.RECOVER: # 자홍색 (후진 중)
            mrk.color.r = 1.0
            mrk.color.g = 0.0
            mrk.color.b = 1.0

        self.state_marker_pub.publish(mrk)

    #############
    # MAIN LOOP #
    #############    
    def main_loop_callback(self):
        # self.get_logger().info("i'm in main_looppppppppppppppppppppppp!")
        self.get_logger().debug(f"Current state: {self.state}")

        # 정지 복구용 카운터/재무장 갱신. 전이 판정(_check_recover_needed)이 이 값을 읽으므로
        # 반드시 state_transition 보다 먼저 돌아야 한다.
        if self.params.mode == 'head_to_head':
            self._update_recover_bookkeeping()

        # transition logic
        prev_state = self.state
        if self.params.force_state:
            self.state = self.params.force_state_choice
        else:
            self.state = self.state_transition(self)

        # RECOVER 진입/이탈은 각각 한 번씩만 처리한다
        if self.state == StateType.RECOVER and prev_state != StateType.RECOVER:
            self._recover_start()
        elif prev_state == StateType.RECOVER and self.state != StateType.RECOVER:
            self._recover_end()
        msg = String()
        msg.data = str(self.state)
        self.state_pub.publish(msg)
        self.visualize_state(state=self.state)

        # 오버테이킹 플래너용 OT sector 플래그
        if self.ot_sectors is not None:
            self.ot_section_check_pub.publish(Bool(data=self._check_ot_sector))

        self.local_waypoints.wpnts = self.state_logic(self)
        self._pub_local_waypoints(self.local_waypoints)

        if self.params.mode=="head_to_head" and self.params.overtake_mode == "spliner":
            self.splini_ttl_counter -= 1
            # Once ttl has reached 0 we overwrite the avoidance waypoints with the empty waypoints
            if self.splini_ttl_counter <= 0:
                self.last_valid_avoidance_wpnts = None
                self.avoidance_wpnts = WpntArray()
                self.splini_ttl_counter = -1

# defined as entry point in setup.py:
def main(args=None):
    rclpy.init(args=args)

    state_machine = StateMachine()

    rclpy.spin(state_machine)

    state_machine.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
