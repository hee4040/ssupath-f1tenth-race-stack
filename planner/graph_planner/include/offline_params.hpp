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
    double w_inner;
    double w_outer;

    double max_heading_offset;
    std::string map_name;
    std::string csv_output_path;

    double ax_max;          // 최대 종가속도 [m/s²]
    double ay_max;          // 최대 횡가속도 [m/s²]
    double drag_coeff;      // 항력 계수 (0.5*rho*C_d*A)
    double m_veh;           // 차량 질량 [kg]
    double dyn_model_exp;   // 동역학 모델 지수 [1.0~2.0]
    double mu;              // 노면 마찰 계수
    bool closed_track;      // 폐회로 여부

};

// 함수 선언
OfflineParams load_offline_params(rclcpp::Node * node);
