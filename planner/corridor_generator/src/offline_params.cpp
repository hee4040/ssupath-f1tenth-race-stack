#include "offline_params.hpp"

OfflineParams load_offline_params(rclcpp::Node * node) {
    OfflineParams p;

    // ---- raceline ----
    p.rl_kappa = node->declare_parameter<int>("rl_kappa", 9);
    p.rl_s = node->declare_parameter<int>("rl_s", 7);

    // ---- offline ----
    p.lat_resolution = node->declare_parameter<double>("lat_resolution", 0.15);
    p.d_straight = node->declare_parameter<double>("d_straight", 2.0);
    p.d_curve = node->declare_parameter<double>("d_curve", 0.8);
    p.curve_thr = node->declare_parameter<double>("curve_thr", 0.001);
    p.lat_offset = node->declare_parameter<double>("lat_offset", 3.5);
    p.max_lat_steps = node->declare_parameter<int>("max_lat_steps", 2);
    p.min_vel_race = node->declare_parameter<double>("min_vel_race", 1.0);
    p.max_lateral_accel = node->declare_parameter<double>("max_lateral_accel", 7.0);
    p.no_interp_points = node->declare_parameter<int>("no_interp_points", 50);

    p.veh_width = node->declare_parameter<double>("veh_width", 0.28);  // 실측 차폭
    p.veh_length = node->declare_parameter<double>("veh_length", 0.535);
    p.veh_turn = node->declare_parameter<double>("veh_turn", 0.5);
    p.vel_max = node->declare_parameter<double>("vel_max", 15.0);
    p.gg_scale = node->declare_parameter<double>("gg_scale", 1.0);

    p.w_raceline = node->declare_parameter<double>("w_raceline", 1.0);
    p.w_raceline_sat = node->declare_parameter<double>("w_raceline_sat", 1.0);
    p.w_length = node->declare_parameter<double>("w_length", 0.0);
    p.w_curv_avg = node->declare_parameter<double>("w_curv_avg", 7500.0);
    p.w_curv_peak = node->declare_parameter<double>("w_curv_peak", 2500.0);
    p.w_virt_goal = node->declare_parameter<double>("w_virt_goal", 10000.0);

    p.max_heading_offset = node->declare_parameter<double>("max_heading_offset", 0.7853981633974483);
    p.map_name = node->declare_parameter<std::string>("map_name", "teras");
    p.csv_output_path = node->declare_parameter<std::string>(
        "csv_output_path", "/home/misys/forza_ws/race_stack/stack_master/maps/");

    return p;
}
