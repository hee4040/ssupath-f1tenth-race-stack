#!/usr/bin/env python3
from fsdp import ros_compat as rospy

PARAMETERS = {
    "dynamic_sqp_tuner_node.evasion_dist": 0.35,
    "dynamic_sqp_tuner_node.obs_traj_tresh": 1.5,
    "dynamic_sqp_tuner_node.spline_bound_mindist": 0.20,
    "dynamic_sqp_tuner_node.lookahead_dist": 15.0,
    "dynamic_sqp_tuner_node.avoidance_resolution": 30,
    "dynamic_sqp_tuner_node.back_to_raceline_before": 6.0,
    "dynamic_sqp_tuner_node.back_to_raceline_after": 8.0,
    "dynamic_sqp_tuner_node.avoid_static_obs": False,
    "dynamic_sqp_tuner_node.merge_speed_factor": 1.5,
}

if __name__ == "__main__":
    rospy.init_node("dynamic_sqp_tuner_node", anonymous=False)
    rospy.declare_parameters(PARAMETERS)
    rospy.loginfo("[Planner] ROS 2 SQP parameter node launched")
    rospy.spin()
