from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_state_machine = LaunchConfiguration("launch_state_machine")
    launch_gp = LaunchConfiguration("launch_gp")
    launch_waypoint_updater = LaunchConfiguration("launch_waypoint_updater")

    # sqp_avoidance_node 는 fsdp/ros_compat.get_param 으로 파라미터를 읽는다.
    # get_param 은 "다른 노드"(dynamic_sqp_tuner_node)를 조회하는 게 아니라 자기 노드의
    # 로컬 파라미터만 본다. 따라서 dynamic_sqp_server.py 가 선언하는 값은 planner 에
    # 전달되지 않고, 여기서 planner_sqp 노드에 직접 넘겨야 실제로 반영된다.
    # (ros_compat 이 automatically_declare_parameters_from_overrides=True 로 노드를 만들기 때문에
    #  아래 오버라이드가 get_param 의 기본값을 이긴다)
    # 또한 dyn_param_cb 는 __init__ 에서 단 한 번만 호출되므로 런타임 변경은 반영되지 않는다.
    sqp_params = {
        # 차폭 [m]. 궤적의 d 는 차량 '중심'이므로 장애물/벽 여유는 전부 반차폭 기준으로 계산된다.
        # 실측 0.28 (2026-07-31). graph_planner 의 offline_params.yaml veh_width 와 같은 값을 쓸 것.
        # 실제보다 좁게 잡으면 그만큼 여유를 낙관해 장애물을 스친다.
        "dynamic_sqp_tuner_node.width_car": ParameterValue(
            LaunchConfiguration("width_car"), value_type=float),
        # 장애물 표면과 '차체' 사이에 확보할 soft 여유 [m] (반차폭은 코드가 따로 더한다).
        # 넓은 구간에서 노리는 목표값이고, 좁으면 아래 min_evasion_dist 까지 자동으로 깎인다.
        "dynamic_sqp_tuner_node.evasion_dist": ParameterValue(
            LaunchConfiguration("evasion_dist"), value_type=float),
        # 좁은 구간에서 여유를 여기까지만 깎는다 [m]. 0 이면 차체가 장애물에 닿는 한계.
        "dynamic_sqp_tuner_node.min_evasion_dist": ParameterValue(
            LaunchConfiguration("min_evasion_dist"), value_type=float),
        # _more_space 의 좌/우 여유 판정 임계값에 더해지는 값 [m]
        "dynamic_sqp_tuner_node.spline_bound_mindist": ParameterValue(
            LaunchConfiguration("spline_bound_mindist"), value_type=float),
        # |d_center| 가 이 값보다 큰 장애물은 무시 [m]
        "dynamic_sqp_tuner_node.obs_traj_tresh": ParameterValue(
            LaunchConfiguration("obs_traj_tresh"), value_type=float),
        # 전방 이 거리 안의 장애물만 고려 [m]
        "dynamic_sqp_tuner_node.lookahead_dist": ParameterValue(
            LaunchConfiguration("lookahead_dist"), value_type=float),
        # 정적 장애물도 회피 대상에 포함할지
        "dynamic_sqp_tuner_node.avoid_static_obs": ParameterValue(
            LaunchConfiguration("avoid_static_obs"), value_type=bool),
        # 회피 구간 다운샘플 점 개수 (int 여야 np.linspace 에 들어간다)
        "dynamic_sqp_tuner_node.avoidance_resolution": ParameterValue(
            LaunchConfiguration("avoidance_resolution"), value_type=int),
        "dynamic_sqp_tuner_node.back_to_raceline_before": ParameterValue(
            LaunchConfiguration("back_to_raceline_before"), value_type=float),
        "dynamic_sqp_tuner_node.back_to_raceline_after": ParameterValue(
            LaunchConfiguration("back_to_raceline_after"), value_type=float),
        "dynamic_sqp_tuner_node.merge_speed_factor": ParameterValue(
            LaunchConfiguration("merge_speed_factor"), value_type=float),
    }

    return LaunchDescription([
        DeclareLaunchArgument("launch_state_machine", default_value="true"),
        DeclareLaunchArgument("launch_gp", default_value="true"),
        DeclareLaunchArgument("launch_waypoint_updater", default_value="true"),

        DeclareLaunchArgument("width_car", default_value="0.28"),
        DeclareLaunchArgument("evasion_dist", default_value="0.15"),
        DeclareLaunchArgument("min_evasion_dist", default_value="0.05"),
        DeclareLaunchArgument("spline_bound_mindist", default_value="0.05"),
        DeclareLaunchArgument("obs_traj_tresh", default_value="1.5"),
        DeclareLaunchArgument("lookahead_dist", default_value="15.0"),
        DeclareLaunchArgument("avoid_static_obs", default_value="true"),
        DeclareLaunchArgument("avoidance_resolution", default_value="20"),
        DeclareLaunchArgument("back_to_raceline_before", default_value="5.0"),
        DeclareLaunchArgument("back_to_raceline_after", default_value="5.0"),
        DeclareLaunchArgument("merge_speed_factor", default_value="1.5"),

        GroupAction(condition=IfCondition(launch_state_machine), actions=[
            Node(package="state_machine", executable="dynamic_statemachine_server.py", name="dynamic_statemachine_server", output="both"),
            Node(package="state_machine", executable="state_machine_node.py", name="state_machine", output="both", parameters=[{"timetrials_only": False, "ot_planner": "predictive_spliner"}]),
        ]),
        Node(package="fsdp", executable="dynamic_collision_server.py", name="dynamic_collision_tuner_node", output="both"),
        Node(package="fsdp", executable="collision_prediction.py", name="collision_predictor", output="both"),
        Node(package="fsdp", executable="dynamic_sqp_server.py", name="dynamic_sqp_tuner_node", output="both"),
        Node(package="fsdp", executable="sqp_avoidance_node.py", name="planner_sqp", output="both",
             parameters=[sqp_params]),
        GroupAction(condition=IfCondition(launch_waypoint_updater), actions=[Node(package="fsdp", executable="update_waypoints.py", name="waypoint_updater", output="both")]),
        GroupAction(condition=IfCondition(launch_gp), actions=[
            Node(package="fsdp", executable="opponent_trajectory.py", name="OpponentHalflap", output="both"),
            Node(package="fsdp", executable="gaussian_process_opp_traj.py", name="GP_trajectory", output="both"),
            Node(package="fsdp", executable="predictor_opponent_trajectory.py", name="Predictor_Opp", output="both"),
        ]),
    ])
