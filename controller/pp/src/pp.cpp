#include "pp.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

PP_Controller::PP_Controller(
    double t_clip_min_,
    double t_clip_max_,
    double m_l1_,
    double q_l1_,
    double speed_lookahead_,
    double lat_err_coeff_,
    double acc_scaler_for_steer_,
    double dec_scaler_for_steer_,
    double start_scale_speed_,
    double end_scale_speed_,
    double downscale_factor_,
    double speed_lookahead_for_steer_,

    bool   prioritize_dyn_,
    double trailing_gap_,
    double trailing_p_gain_,
    double trailing_i_gain_,
    double trailing_d_gain_,
    double blind_trailing_speed_,
    double trailing_to_gbtrack_speed_scale_,
    double curvature_scale_,
    double ok_thresh_,
    double hard_thresh_,
    double min_scale_,
    double steer_rate_thresh_,

    double loop_rate_,
    double wheelbase_,

    Logger logger_info_,
    Logger logger_warn_)

  : t_clip_min(t_clip_min_), t_clip_max(t_clip_max_),
    m_l1(m_l1_), q_l1(q_l1_),
    speed_lookahead(speed_lookahead_), lat_err_coeff(lat_err_coeff_),
    acc_scaler_for_steer(acc_scaler_for_steer_), dec_scaler_for_steer(dec_scaler_for_steer_),
    start_scale_speed(start_scale_speed_), end_scale_speed(end_scale_speed_),
    downscale_factor(downscale_factor_), speed_lookahead_for_steer(speed_lookahead_for_steer_),
    prioritize_dyn(prioritize_dyn_),
    trailing_gap(trailing_gap_), trailing_p_gain(trailing_p_gain_),
    trailing_i_gain(trailing_i_gain_), trailing_d_gain(trailing_d_gain_),
    blind_trailing_speed(blind_trailing_speed_),
    trailing_to_gbtrack_speed_scale(trailing_to_gbtrack_speed_scale_),
    curvature_scale(curvature_scale_),
    ok_thresh(ok_thresh_),
    hard_thresh(hard_thresh_),
    min_scale(min_scale_),
    steer_rate_thresh(steer_rate_thresh_),
    loop_rate(loop_rate_), wheelbase(wheelbase_),
    logger_info(logger_info_), logger_warn(logger_warn_) {}

PP_Controller::Output
PP_Controller::main_loop(const std::string& state,
                         const std::optional<Pose3>& position_in_map,
                         const std::vector<WpRow>& waypoint_array_in_map,
                         double speed_now,
                         const std::optional<Opp5>& opponent,
                         const std::optional<Fren4>& position_in_map_frenet,
                         const std::vector<double>& acc_now,
                         double track_length)
{
  // 업데이트
  state_         = state;
  if (position_in_map) pose_ = *position_in_map;
  wp_            = waypoint_array_in_map;
  speed_now_     = speed_now;
  opp_           = opponent;
  frenet_        = position_in_map_frenet;
  acc_now_       = acc_now;
  track_length_  = track_length;

  // 정지 복구(후진). 아래 전진 로직은 전부 전진 전제(속도 하한 0, lookahead 는 항상 앞쪽,
  // 조향 부호도 전진 기준)라 그대로는 후진에 쓸 수 없다. 그래서 별도 경로로 빠진다.
  if (state_ == "StateType.RECOVER") {
    return reverse_loop();
  }

  // PREPROCESS
  const double yaw = pose_[2];
  const Vec2 v{ std::cos(yaw)*speed_now_, std::sin(yaw)*speed_now_ };

  // lateral error (from frenet d)
  auto [lat_e_norm, lateral_error] = calc_lateral_error_norm();
  auto [L1_point, L1_distance] = calc_L1_point(lateral_error);


  // LONGITUDINAL
  const double adv_ts_sp = speed_lookahead;
  const Vec2 la_pos{ pose_[0] + v[0]*adv_ts_sp, pose_[1] + v[1]*adv_ts_sp };
  const int idx_la = nearest_waypoint(la_pos, wp_);


  double global_speed = wp_[idx_la][2];


  if (state_ == "StateType.TRAILING" && opp_ && frenet_) {
    speed_command_ = trailing_controller(global_speed);
  } else if (state_ == "StateType.TRAILING_TO_GBTRACK") {
    speed_command_ = global_speed * trailing_to_gbtrack_speed_scale;
  } else {
    i_gap_ = 0.0;
    speed_command_ = global_speed;
  }

  speed_command_ = speed_adjust_lat_err(speed_command_, lat_e_norm);
  speed_command_ = speed_adjust_heading(speed_command_);


  double speed = std::isfinite(speed_command_) ? std::max(speed_command_, 0.0) : 0.0;

  double steering_angle = calc_steering_angle(L1_point, L1_distance, yaw, lat_e_norm, v);

  return {speed, 0.0, 0.0, steering_angle, L1_point, L1_distance, idx_nearest_wp_};
}

int PP_Controller::nearest_waypoint(const Vec2& position, const std::vector<WpRow>& waypoints) const {
  if (waypoints.empty()) return -1;
  int best = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (int i=0;i<(int)waypoints.size();++i) {
    double dx = position[0] - waypoints[i][0];
    double dy = position[1] - waypoints[i][1];
    double d2 = dx*dx + dy*dy;
    if (d2 < best_d2) { best_d2 = d2; best = i; }
  }
  return best;
}

PP_Controller::Vec2
PP_Controller::waypoint_at_distance_before_car(double distance,
                                               const std::vector<WpRow>& waypoints,
                                               int idx_waypoint_behind_car) const
{
  if (distance <= 0.0) distance = t_clip_min;
  const double waypoints_distance = 0.1;
  const int d_index = static_cast<int>(distance/waypoints_distance + 0.5);
  const int idx = std::min((int)waypoints.size()-1, idx_waypoint_behind_car + d_index);
  return Vec2{ waypoints[idx][0], waypoints[idx][1] };
}

std::pair<double,double> PP_Controller::calc_lateral_error_norm() const {
  double lateral_error = 0.0;
  if (frenet_) lateral_error = std::abs((*frenet_)[1]); // d
  const double max_lat_e = 0.5, min_lat_e = 0.0;
  const double lat_e_clip = clip(lateral_error, min_lat_e, max_lat_e);
  const double lat_e_norm = 0.5 * ((lat_e_clip - min_lat_e) / (max_lat_e - min_lat_e));
  return {lat_e_norm, lateral_error};
}

double PP_Controller::speed_adjust_lat_err(double global_speed, double lat_e_norm) const {
  const double lat_e_coeff = clip(lat_err_coeff, 0.0, 1.0);
  const double lat_e = lat_e_norm * 2.0;
  const double curv = curvature_waypoints_ / curvature_scale;
  return global_speed * (1.0 - lat_e_coeff + lat_e_coeff*std::exp(-lat_e*curv));
}

double PP_Controller::speed_adjust_heading(double speed_command) const {
  if (wp_.empty()) return speed_command;

  const int idx = nearest_waypoint({pose_[0], pose_[1]}, wp_);
  if (idx < 0 || idx >= static_cast<int>(wp_.size())) return speed_command;

  const double heading     = pose_[2];
  const double map_heading = wp_[idx][6];

  const double dpsi = std::atan2(std::sin(heading - map_heading), std::cos(heading - map_heading));
  const double heading_error = std::abs(dpsi);

  // Convert thresholds from DEG → RAD
  // --------------------------------------------------------
  const double ok_thresh_rad   = ok_thresh   * (M_PI / 180.0);
  const double hard_thresh_rad = hard_thresh * (M_PI / 180.0);
  // --------------------------------------------------------

  double scale = 1.0;
  if (heading_error <= ok_thresh_rad) {
    scale = 1.0;
  } else if (heading_error < hard_thresh_rad) {
    const double t = (heading_error - ok_thresh_rad) / (hard_thresh_rad - ok_thresh_rad); // normalized
    scale = min_scale + (1.0 - min_scale) * (1.0 - t);
  } else {
    scale = min_scale;
  }

  return speed_command * scale;
}


double PP_Controller::acc_scaling(double steer) const {
  double mean_acc = 0.0;
  if (!acc_now_.empty()) {
    for (double a : acc_now_) mean_acc += a;
    mean_acc /= (double)acc_now_.size();
  }
  if (mean_acc >= 1.0)      steer *= acc_scaler_for_steer;
  else if (mean_acc <= -1.0)steer *= dec_scaler_for_steer;
  return steer;
}

double PP_Controller::speed_steer_scaling(double steer, double speed) const {
  const double speed_diff = std::max(0.1, end_scale_speed - start_scale_speed);
  const double factor = 1.0 - clip((speed - start_scale_speed)/speed_diff, 0.0, 1.0) * downscale_factor;
  return steer * factor;
}

double PP_Controller::trailing_controller(double global_speed) {
  // opp_[2]=vs, frenet_[0]=s, [2]=vs
  const double opp_s = (*opp_)[0];
  const double opp_vs= (*opp_)[2];
  const bool opp_visible = ((*opp_)[4] > 0.5);

  const double my_s  = (*frenet_)[0];
  const double my_vs = (*frenet_)[2];

  gap_ = std::fmod(opp_s - my_s + track_length_, track_length_);
  gap_actual_ = gap_;
  gap_should_ = trailing_gap;
  gap_error_  = gap_should_ - gap_actual_;
  v_diff_     = my_vs - opp_vs;
  i_gap_ = clip(i_gap_ + gap_error_/loop_rate, -10.0, 10.0);

  const double p_value = gap_error_ * trailing_p_gain;
  const double d_value = v_diff_    * trailing_d_gain;
  const double i_value = i_gap_     * trailing_i_gain;

  trailing_command_ = clip(opp_vs - p_value - i_value - d_value, 0.0, global_speed);
  if (!opp_visible && gap_actual_ > gap_should_) {
    trailing_command_ = std::max(blind_trailing_speed, trailing_command_);
  }
  return trailing_command_;
}

PP_Controller::Output PP_Controller::reverse_loop() {
  // 후진 pure pursuit.
  //
  // 상태머신(states.Recovering)이 보내는 wp_ 는 '차에서 가장 가까운 중심선 점 -> 뒤쪽'
  // 순서이고 vx 가 음수다. 따라서 인덱스를 키우면 차 뒤쪽으로 간다.
  //
  // 조향은 헤딩을 180도 돌린 가상의 전진 문제로 풀고 부호를 뒤집는다:
  //     r     = (-cos yaw, -sin yaw)                 (후진 진행 방향)
  //     alpha = atan2(r x L1_vec, r . L1_vec)        (후진 방향 기준 목표점 각도)
  //     delta = -atan(2 * L * sin(alpha) / |L1_vec|)
  // 부호 검산: yaw=0 에서 목표점이 (-1, +0.3) 이면 alpha<0 -> delta>0 (좌조향).
  // 후진 중 좌조향이면 요레이트가 음수라 차 뒤쪽이 +y 로 간다 -> 목표점 쪽. 맞다.
  const Vec2 here{ pose_[0], pose_[1] };

  if (wp_.empty()) {
    logger_warn("[PP] RECOVER: 로컬 웨이포인트가 비어 있어 정지");
    curr_steering_angle_ = 0.0;
    return {0.0, 0.0, 0.0, 0.0, here, 0.0, -1};
  }

  idx_nearest_wp_ = nearest_waypoint(here, wp_);
  if (idx_nearest_wp_ < 0) idx_nearest_wp_ = 0;

  // 후방 lookahead 점. 배열 간격은 중심선 간격(0.1 m)이다.
  const double waypoints_distance = 0.1;
  const int d_index = static_cast<int>(clip(reverse_l1_dist, 0.2, 3.0)/waypoints_distance + 0.5);
  const int idx_la  = std::min((int)wp_.size()-1, idx_nearest_wp_ + d_index);
  const Vec2 L1_point{ wp_[idx_la][0], wp_[idx_la][1] };

  const double yaw = pose_[2];
  const Vec2 L1_vec{ L1_point[0] - here[0], L1_point[1] - here[1] };
  const double L1_norm = std::hypot(L1_vec[0], L1_vec[1]);

  double steer = 0.0;
  if (L1_norm > 1e-6) {
    const double rx = -std::cos(yaw), ry = -std::sin(yaw);
    const double along = rx*L1_vec[0] + ry*L1_vec[1];
    const double lat   = rx*L1_vec[1] - ry*L1_vec[0];
    if (along <= 0.0) {
      // 목표점이 차 앞쪽이다 = 웨이포인트 순서가 뒤집혔거나 차가 돌아버린 것.
      // 억지로 조향하면 엉뚱한 데로 가므로 직진 후진으로 버틴다(상태머신 타임아웃이 끊는다).
      logger_warn("[PP] RECOVER: lookahead 점이 차 앞쪽이다 - 조향 0 으로 후진");
      steer = 0.0;
    } else {
      const double alpha = std::atan2(lat, along);
      steer = -std::atan(2.0*wheelbase*std::sin(alpha) / std::max(1e-6, L1_norm));
    }
  }

  steer = clip(steer, -std::abs(reverse_steer_max), std::abs(reverse_steer_max));
  steer = clip(steer, curr_steering_angle_ - steer_rate_thresh, curr_steering_angle_ + steer_rate_thresh);
  curr_steering_angle_ = steer;

  // 속도는 상태머신이 실어 보낸 값(음수, dwell 구간에서는 0)을 그대로 쓰되 크기를 제한한다.
  // 여기서 양수가 절대 못 나가게 막는 것이 중요하다 - RECOVER 인데 전진하면 장애물로 간다.
  double speed = wp_[idx_nearest_wp_][2];
  if (!std::isfinite(speed)) speed = 0.0;
  speed = clip(speed, -std::abs(reverse_speed_max), 0.0);

  return {speed, 0.0, 0.0, steer, L1_point, L1_norm, idx_nearest_wp_};
}

std::pair<PP_Controller::Vec2,double>
PP_Controller::calc_L1_point(double lateral_error) {
  idx_nearest_wp_ = nearest_waypoint({pose_[0], pose_[1]}, wp_);
  if (idx_nearest_wp_ < 0) idx_nearest_wp_ = 0;

  if ((int)wp_.size() - idx_nearest_wp_ > 2) {
    double sum=0.0; int cnt=0;
    for (int i=idx_nearest_wp_; i<(int)wp_.size(); ++i) { sum += std::abs(wp_[i][5]); ++cnt; }
    curvature_waypoints_ = (cnt>0) ? (sum/cnt) : 0.0;
  }

  double L1_distance = q_l1 + speed_now_ * m_l1;
  const double lower_bound = std::max(t_clip_min, std::sqrt(2.0)*std::abs(lateral_error));
  L1_distance = clip(L1_distance, lower_bound, t_clip_max);

  Vec2 L1_point = waypoint_at_distance_before_car(L1_distance, wp_, idx_nearest_wp_);
  return {L1_point, L1_distance};
}

double PP_Controller::calc_steering_angle(const Vec2& L1_point,
                                          double L1_distance,
                                          double yaw,
                                          double lat_e_norm,
                                          const Vec2& v)
{
  double speed_la_for_lu = 0.0;
  if (state_ == "StateType.TRAILING" && opp_) {
    speed_la_for_lu = speed_now_;
  } else {
    const double adv_ts_st = speed_lookahead_for_steer;
    const Vec2 la_pos{ pose_[0] + v[0]*adv_ts_st, pose_[1] + v[1]*adv_ts_st };
    const int idx = nearest_waypoint(la_pos, wp_);
    speed_la_for_lu = (idx>=0) ? wp_[idx][2] : speed_now_;
  }
  const double speed_for_lu = speed_adjust_lat_err(speed_la_for_lu, lat_e_norm);

  // eta
  const Vec2 L1_vec{ L1_point[0] - pose_[0], L1_point[1] - pose_[1] };
  const double L1_norm = std::hypot(L1_vec[0], L1_vec[1]);
  double eta = 0.0;
  if (L1_norm <= 1e-8) {
    logger_warn("[PP] norm(L1_vector)==0, eta=0");
  } else {
    const double dot = (-std::sin(yaw))*L1_vec[0] + ( std::cos(yaw))*L1_vec[1];
    eta = std::asin( clip(dot / L1_norm, -1.0, 1.0) );
  }

  double steer = std::atan( 2.0*wheelbase*std::sin(eta) / std::max(1e-6, L1_distance) );

  steer = acc_scaling(steer);
  steer = speed_steer_scaling(steer, speed_for_lu);

  // 0715: velocity-based scaling(steer *= 1+v/10) 제거 —
  // 속도에 비례해 이득을 키워 위빙(1.8Hz 리밋사이클)을 유발하던 항

  // rate limit
  steer = clip(steer, curr_steering_angle_ - steer_rate_thresh, curr_steering_angle_ + steer_rate_thresh);
  curr_steering_angle_ = steer;

  return steer;
}
