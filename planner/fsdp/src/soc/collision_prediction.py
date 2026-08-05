#!/usr/bin/env python3

from fsdp import ros_compat as rospy
import numpy as np
from std_msgs.msg import Bool, Float32, String
from nav_msgs.msg import Odometry
from f110_msgs.msg import ObstacleArray, WpntArray, Wpnt, Obstacle, ObstacleArray, OpponentTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from frenet_conversion.frenet_converter import FrenetConverter
import time
import copy

class CollisionPredictor:
    """
    Predict the region of collision between the ego car and the opponent.
    """

    def __init__(self):
        # Initialize the node
        rospy.init_node('collision_predictor', anonymous=True)

        # ROS Parameters
        self.opponent_traj_topic = '/opponent_trajectory'
        

        # Subscriber
        rospy.Subscriber("/perception/obstacles", ObstacleArray, self.opponent_state_cb)
        rospy.Subscriber("/car_state/frenet/odom", Odometry, self.odom_cb)
        rospy.Subscriber(self.opponent_traj_topic, OpponentTrajectory, self.opponent_trajectory_cb)
        rospy.Subscriber('/global_waypoints_updated', WpntArray, self.wpnts_updated_cb)
        rospy.Subscriber("/state_machine", String, self.state_cb)
        

        self.converter = None

        self.loop_rate = 10 #Hz

        # Callback data
        self.opponent_pos = ObstacleArray()
        self.car_odom = Odometry()
        self.wpnts_opponent = list()
        self.wpnts_updated = list()
        self.state = String()

        self.speed_offset = 0 # m/s

        # Simulation parameters
        self.time_steps = 200
        self.dt = 0.02 # s
        self.save_distance_front = 0.6 # m
        self.save_distance_back = 0.4 # m
        self.max_v = 10 # m/s
        self.min_v = 0 # m/s
        self.max_a = 5.5 # m/s^2
        self.min_a = 5 # m/s^2
        self.max_expire_counter = 10

        # Number of time steps before prediction expires. Set when collision is published.
        self.expire_counter = 0 

        # Visualization
        self.marker_beginn = self.marker_init(a = 0.5, r = 0.63, g = 0.13, b = 0.94, id = 0)
        self.marker_end = self.marker_init(a = 0.5, r = 0.63, g = 0.13, b = 0.94, id = 1)

        # Opponent
        self.opponent_lap_count = None
  
        # Publisher
        self.marker_pub_beginn = rospy.Publisher("/collision_predict/beginn", Marker, queue_size=10)
        self.marker_pub_end = rospy.Publisher("/collision_predict/end", Marker, queue_size=10)
        self.collision_obs_pub = rospy.Publisher("/collision_prediction/obstacles", ObstacleArray, queue_size=10)
        self.force_trailing_pub = rospy.Publisher("collision_prediction/force_trailing", Bool, queue_size=10)
        
    
    ### CALLBACKS ###
    # TODO: Only sees the first dynamic obstacle as opponent...
    def opponent_state_cb(self, data: ObstacleArray):
        self.opponent_pos.header = data.header
        is_dynamic = False
        if len(data.obstacles) > 0:
            for obs in data.obstacles:
                if obs.is_static == False: # and obs.is_opponent == True: # Changed by Tino and Nicolas for compatibility with new obstacle message
                    self.opponent_pos.obstacles = [obs]
                    is_dynamic = True
                    break
        if is_dynamic == False:
            self.opponent_pos.obstacles = []

    def odom_cb(self, data: Odometry): self.car_odom = data

    def opponent_trajectory_cb(self, data: OpponentTrajectory):
        self.wpnts_opponent = data.oppwpnts  # exclude last point (because last point == first point) <- Hopefully this is still the case?
        self.max_s_opponent = self.wpnts_opponent[-1].s_m
        self.opponent_lap_count = data.lap_count

    def wpnts_updated_cb(self, data: WpntArray):
        self.wpnts_updated = data.wpnts[:-1]
        self.max_s_updated = self.wpnts_updated[-1].s_m # Should be the same as self.max_s but just in case. Only used for wrap around
        self.converter = FrenetConverter(
            np.array([wpnt.x_m for wpnt in self.wpnts_updated]),
            np.array([wpnt.y_m for wpnt in self.wpnts_updated]),
            np.array([wpnt.psi_rad for wpnt in self.wpnts_updated]),
        )

    def state_cb(self, data: String):
        # ROS2 state machine 은 "StateType.TRAILING" 형태로 발행한다
        self.state = data.data.split(".")[-1]

        # Callback triggered by dynamic spline reconf
    def dyn_param_cb(self, params):
        """
        Notices the change in the parameters and changes spline params
        """
        self.time_steps = rospy.get_param("dynamic_collision_tuner_node/n_time_steps", 200)
        self.dt = rospy.get_param("dynamic_collision_tuner_node/dt", 0.02)
        self.save_distance_front = rospy.get_param("dynamic_collision_tuner_node/save_distance_front", 0.6)
        self.save_distance_back = rospy.get_param("dynamic_collision_tuner_node/save_distance_back", 0.4)
        self.max_v = rospy.get_param("dynamic_collision_tuner_node/max_v", 10)
        self.min_v = rospy.get_param("dynamic_collision_tuner_node/min_v", 0)
        self.max_a = rospy.get_param("dynamic_collision_tuner_node/max_a", 5.5)
        self.min_a = rospy.get_param("dynamic_collision_tuner_node/min_a", 5)
        self.max_expire_counter = rospy.get_param("dynamic_collision_tuner_node/max_expire_counter", 10)
        self.speed_offset = rospy.get_param("dynamic_collision_tuner_node/max_expire_counter", 0)

        print(
            f"[Coll. Pred.] Dynamic reconf triggered new params:\n"
            f" N time stepts: {self.time_steps}, \n"
            f" dt: {self.dt} [s], \n"
            f" save_distance_front: {self.save_distance_front} [m], \n"
            f" save_distance_back: {self.save_distance_back} [m], \n"
            f" max_v, min_v, max_a, min_a: {self.max_v, self.min_v, self.max_a, self.min_a}, \n"
            f" max_expire_counter: {self.max_expire_counter}"
        )



    def frenet_to_cartesian(self, s_values, d_values):
        if self.converter is None:
            raise RuntimeError("FrenetConverter is not initialized; wait for /global_waypoints_updated first")
        xy = self.converter.get_cartesian(np.asarray(s_values, dtype=float), np.asarray(d_values, dtype=float))
        class Result:
            pass
        result = Result()
        result.x = xy[0]
        result.y = xy[1]
        return result

    ### HELPER FUNCTIONS ###
    def marker_init(self, a = 1, r = 1, g = 0, b = 0, id = 0):
        marker = Marker(header=rospy.Header(stamp = rospy.Time.now(), frame_id = "map"),id = id, type = Marker.SPHERE)
        # Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.orientation.w = 1.0
        # Set the marker's scale
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        # Set the marker's color
        marker.color.a = a  # Alpha (transparency)
        marker.color.r = r  # Red
        marker.color.g = g  # Green
        marker.color.b = b  # Blue
        return marker

    def delete_all(self) -> None:
        empty_marker = Marker(header=rospy.Header(stamp = rospy.Time.now(), frame_id = "map"),id = 0)
        empty_marker.action = Marker.DELETE
        self.marker_pub_beginn.publish(empty_marker)
        empty_marker.id = 1
        self.marker_pub_end.publish(empty_marker)

        empty_obs_arr = ObstacleArray(header = rospy.Header(stamp = rospy.Time.now(), frame_id = "map"))
        self.collision_obs_pub.publish(empty_obs_arr)

    ### MAIN LOOP ###
    def loop(self):
        """
        Compute the Region of collision.
        """
        rate = rospy.Rate(self.loop_rate)
        rospy.loginfo("[Coll. Pred.] Collision Predictor wating...")
        rospy.wait_for_message("/global_waypoints", WpntArray)
        rospy.wait_for_message("/global_waypoints_scaled", WpntArray)
        rospy.wait_for_message("/global_waypoints_updated", WpntArray)
        rospy.loginfo("[Coll. Pred.] Updated waypoints recived!")
        # /opponent_trajectory 는 상대차를 한 바퀴 이상 추적해야 처음 나온다.
        # 여기서 무한 대기하면 정적 장애물만 있는 주행에서는 노드가 영원히 기동하지 않고,
        # 상대차가 나타나도 이미 루프에 진입하지 못한 상태다. 타임아웃 후 진행하고,
        # 궤적이 실제로 도착했는지는 루프 안에서 매번 확인한다.
        try:
            rospy.wait_for_message(self.opponent_traj_topic, OpponentTrajectory, timeout=5.0)
            rospy.loginfo("[Coll. Pred.] Opponent waypoints recived!")
        except TimeoutError:
            rospy.logwarn(
                "[Coll. Pred.] No /opponent_trajectory yet - starting anyway. "
                "Collision prediction stays idle until the GP chain publishes one."
            )
        rospy.wait_for_message("/perception/obstacles", ObstacleArray)
        rospy.loginfo("[Coll. Pred.] Obstacles reveived!")
        rospy.loginfo("[Coll. Pred.] Collision Predictor ready!")

        while not rospy.is_shutdown():

            opponent_pos_copy = copy.deepcopy(self.opponent_pos)

            # 상대 궤적(GP)이 아직 없으면 ROC 를 계산할 근거가 없다.
            # (wpnts_opponent 가 비어 있으면 self.max_s_opponent 자체가 정의되지 않는다)
            opp_traj_ready = len(self.wpnts_opponent) > 0 and self.opponent_lap_count is not None

            if len(opponent_pos_copy.obstacles) != 0 and opp_traj_ready:
                # Get the global s points
                s_points_global_array = np.array([wpnt.s_m for wpnt in self.wpnts_opponent]) # Opponent
                s_points_updated_array = np.array([wpnt.s_m for wpnt in self.wpnts_updated]) # Ego (updated)

                # Get current position of the car and opponent
                ego_position = self.car_odom.pose.pose.position.x
                opponent_position = opponent_pos_copy.obstacles[0].s_center
                # Handle wrap around
                if ego_position > self.max_s_updated * (2/3) and opponent_position < self.max_s_updated * (1/3):
                    opponent_position += self.max_s_updated
                
                # Check if opponent is close to predicted opponent race line
                current_opponent_d = self.opponent_pos.obstacles[0].d_center
                opponent_approx_indx = np.abs(s_points_global_array - opponent_position).argmin()
                opponent_raceline_d = self.wpnts_opponent[opponent_approx_indx].d_m

                if abs(current_opponent_d - opponent_raceline_d) > 0.25 or self.opponent_lap_count < 1:
                    self.force_trailing_pub.publish(True)
                else:
                    self.force_trailing_pub.publish(False)

                # Get current speed of the ego car
                current_ego_speed = self.car_odom.twist.twist.linear.x
                
                # Temp params
                beginn = False
                end = False
                beginn_s = 0
                end_s = 0
                beginn_d = 0
                end_d = 0

                start = time.process_time()
                for i in range(self.time_steps):
                    # Get the speed at position i + 1
                    opponent_approx_indx = np.abs(s_points_global_array - opponent_position % self.max_s_opponent).argmin() # Opponent
                    ego_approx_indx = np.abs(s_points_updated_array- ego_position % self.max_s_updated).argmin() # Ego (scaled)
                    
                    opponent_speed = self.wpnts_opponent[opponent_approx_indx].proj_vs_mps # Opponent
                    ego_speed = self.wpnts_updated[ego_approx_indx].vx_mps # Ego (updated)
                    
                    # Interpolate the acceleration
                    acceleration = self.min_a + (self.max_a - self.min_a) / max([self.max_v - self.min_v, 10e-6]) * (self.max_v - current_ego_speed)

                    if current_ego_speed < ego_speed:
                        ego_position = (ego_position + current_ego_speed * self.dt + 0.5 * acceleration * self.dt**2)
                        opponent_position = (opponent_position + opponent_speed * self.dt)
                        current_ego_speed += acceleration * self.dt
                    else: 
                        ego_position = (ego_position + ego_speed * self.dt)
                        opponent_position = (opponent_position + opponent_speed * self.dt)

                    opponent_d = self.wpnts_opponent[opponent_approx_indx].d_m
                    
                    # Find begin of the collision
                    if (beginn == False
                        and ((opponent_position - ego_position)%self.max_s_updated < self.save_distance_front
                             or abs(opponent_position - ego_position) < self.save_distance_front)):
                       
                        beginn_s = ego_position
                        beginn_d = opponent_d
                        beginn = True
                    # Find the end of the collision
                    elif (beginn == True and end == False and ((ego_position - opponent_position) > self.save_distance_back)):
                        end_s = ego_position
                        end_d = opponent_d
                        end = True
                        break
                
                # If no end has been found use latest predicted opponent position
                if beginn == True and end == False:
                    end_s = opponent_position
                    end_d = opponent_d

                # Check which d is upper and lower bound
                if beginn == True:
                    if end == True: # or end == False: # Add or end == False for more agressive "oppoent loop" behavior
                        if beginn_d < end_d:
                            d_right = beginn_d - 0.25
                            d_left = end_d + 0.25
                        else:
                            d_right = end_d - 0.25
                            d_left = beginn_d + 0.25
                    else: # Create a wall if there is no end of the collision in sight
                        d_right = -10
                        d_left = 10

                    collision_obs = Obstacle(id = 0, d_left = d_left, d_right = d_right, d_center = (d_right + d_left)/2, vs = 0, vd = 0, is_actually_a_gap = False, is_static = False)
                    collision_obs.s_start = beginn_s
                    collision_obs.s_end = end_s
                    collision_obs.s_center = (beginn_s + end_s)/2

                    collision_obs_arr = ObstacleArray(header = rospy.Header(stamp = rospy.Time.now(), frame_id = "map"), obstacles = [collision_obs])
                    self.collision_obs_pub.publish(collision_obs_arr)
                    self.expire_counter = 0

                    # Visualize the collision (Watchout for wrap around)
                    position_beginn = self.frenet_to_cartesian([beginn_s%self.max_s_updated], [beginn_d])
                    self.marker_beginn.pose.position.x = position_beginn.x[0]
                    self.marker_beginn.pose.position.y = position_beginn.y[0]
                    self.marker_pub_beginn.publish(self.marker_beginn)

                    position_end = self.frenet_to_cartesian([end_s%self.max_s_updated], [end_d])
                    self.marker_end.pose.position.x = position_end.x[0]
                    self.marker_end.pose.position.y = position_end.y[0]
                    self.marker_pub_end.publish(self.marker_end)


            self.expire_counter += 1
            if self.expire_counter >= self.max_expire_counter:
                self.expire_counter = self.max_expire_counter
                self.delete_all()

            # print("Time: {}".format(time.process_time() - start))

            # ros_compat 의 Rate.sleep() 이 유일하게 rclpy.spin_once 를 호출한다.
            # while 밖에 있으면 콜백이 한 번도 돌지 않아 opponent_pos 가 영원히 갱신되지 않는다.
            rate.sleep()

if __name__ == '__main__':
    """
    <Lets predict some collisions!>
    """
    node = CollisionPredictor()
    node.loop()
