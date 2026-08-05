#!/usr/bin/env python3
from fsdp import ros_compat as rospy

PARAMETERS = {
    "dynamic_collision_tuner_node.n_time_steps": 400,
    "dynamic_collision_tuner_node.dt": 0.02,
    "dynamic_collision_tuner_node.save_distance_front": 0.6,
    "dynamic_collision_tuner_node.save_distance_back": 0.6,
    "dynamic_collision_tuner_node.max_v": 10.0,
    "dynamic_collision_tuner_node.min_v": 0.0,
    "dynamic_collision_tuner_node.max_a": 7.0,
    "dynamic_collision_tuner_node.min_a": 5.0,
    "dynamic_collision_tuner_node.max_expire_counter": 10,
    "dynamic_collision_tuner_node.update_waypoints": True,
    "dynamic_collision_tuner_node.speed_offset": 0.0,
}

if __name__ == "__main__":
    rospy.init_node("dynamic_collision_tuner_node", anonymous=False)
    rospy.declare_parameters(PARAMETERS)
    rospy.loginfo("[Planner] ROS 2 collision parameter node launched")
    rospy.spin()
