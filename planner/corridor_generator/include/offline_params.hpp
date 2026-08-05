#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>

struct OfflineParams {
    // ---- raceline ----
    int rl_kappa;
    int rl_s;

    // ---- offline ----
    double lat_resolution;
    double d_straight;
    double d_curve;
    double curve_thr;
    double lat_offset;
    int max_lat_steps;
    double min_vel_race;
    double max_lateral_accel;

    double min_plan_horizon;
    int no_interp_points;

    double veh_width;
    double veh_length;
    double veh_turn;
    double vel_max;
    double gg_scale;

    double w_raceline;
    double w_raceline_sat;
    double w_length;
    double w_curv_avg;
    double w_curv_peak;
    double w_virt_goal;

    double max_heading_offset;
    std::string map_name;
    std::string csv_output_path;
};

// 함수 선언
OfflineParams load_offline_params(rclcpp::Node * node);
