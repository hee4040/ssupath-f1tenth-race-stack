#pragma once
#include <array>
#include <vector>
#include <string>
#include <tuple>
#include <functional>
#include <cmath>
#include <algorithm>
#include <optional>

class PP_Controller {
public:
  using Vec2  = std::array<double,2>;
  using Pose3 = std::array<double,3>;   // x, y, yaw
  using Fren4 = std::array<double,4>;   // s, d, vs, vd
  using Opp5  = std::array<double,5>;   // s, d, vs, is_static(0/1), is_visible(0/1)
  // waypoint row: [x, y, v, share, s, kappa, psi, ax]
  using WpRow = std::array<double,8>;

  using Logger = std::function<void(const std::string&)>;

  PP_Controller(double t_clip_min,
                double t_clip_max,
                double m_l1,
                double q_l1,
                double speed_lookahead,
                double lat_err_coeff,
                double acc_scaler_for_steer,
                double dec_scaler_for_steer,
                double start_scale_speed,
                double end_scale_speed,
                double downscale_factor,
                double speed_lookahead_for_steer,

                bool   prioritize_dyn,
                double trailing_gap,
                double trailing_p_gain,
                double trailing_i_gain,
                double trailing_d_gain,
                double blind_trailing_speed,
                double trailing_to_gbtrack_speed_scale,
                double curvature_scale,
                double ok_thresh,
                double hard_thresh,
                double min_scale,
                double steer_rate_thresh,

                double loop_rate,
                double wheelbase,

                Logger logger_info  = [](auto const&){},
                Logger logger_warn  = [](auto const&){});

  // Python과 동일한 반환 순서:
  // (speed, acceleration, jerk, steering_angle, L1_point, L1_distance, idx_nearest_waypoint)
  using Output = std::tuple<double,double,double,double,Vec2,double,int>;

  Output main_loop(const std::string& state,
                   const std::optional<Pose3>& position_in_map,
                   const std::vector<WpRow>& waypoint_array_in_map,
                   double speed_now,
                   const std::optional<Opp5>& opponent,
                   const std::optional<Fren4>& position_in_map_frenet,
                   const std::vector<double>& acc_now,
                   double track_length);

  // 동적 파라미터 갱신을 위해 public 멤버 그대로 노출 (Python과 동일)
  double t_clip_min, t_clip_max, m_l1, q_l1, speed_lookahead, lat_err_coeff;
  double acc_scaler_for_steer, dec_scaler_for_steer;
  double start_scale_speed, end_scale_speed, downscale_factor, speed_lookahead_for_steer;
  bool   prioritize_dyn;
  double trailing_gap, trailing_p_gain, trailing_i_gain, trailing_d_gain, blind_trailing_speed, trailing_to_gbtrack_speed_scale, curvature_scale, ok_thresh, hard_thresh, min_scale, steer_rate_thresh;
  double loop_rate, wheelbase;

  // ---- 횡오차 기준 / 횡오차 보정 항 (2026-08-21) --------------------------
  // 생성자 인자를 늘리지 않고 l1_controller 가 생성 후 YAML 값으로 덮어쓴다.
  //
  // lat_err_from_path: 컨트롤러가 말하는 "횡오차"를 무엇으로 볼지.
  //   true  = 지금 따라가는 경로(wp_)까지의 거리   <- 기본
  //   false = 예전 동작. |frenet d| = 레이싱라인까지의 거리.
  //   예전 동작은 회피 중에 문제가 된다. 회피란 레이싱라인을 일부러 벗어나는 것인데
  //   그 거리가 (1) calc_L1_point 의 lower_bound 를 밀어올려 lookahead 를 늘리고
  //   (2) speed_adjust_lat_err 로 속도를 깎는다. 둘 다 회피를 방해하는 방향이다.
  //   실측(obs_debug_0821_2001): 회피 중 |frenet d| 중앙 0.28 m / p90 0.92 m 인데
  //   실제 경로 기준 오차는 중앙 0.14 m 였다. lower_bound 가 lookahead 를 덮어쓴
  //   비율이 전역주행 1% 대 회피 23% 로, 튜닝이 원하는 곳에서만 무효화되고 있었다.
  bool   lat_err_from_path{true};
  // ct_gain: 횡오차 보정 항의 이득. 0 이면 항 자체가 꺼진다(= 예전 동작).
  //   순수추종 조향은 lookahead 의 제곱에 반비례해 약해진다(steer ~ 2*L*e/L1^2).
  //   그래서 위빙을 줄이려고 lookahead 를 늘리면 경로 복귀력이 급격히 사라진다.
  //   이 항은 경로 위에 있으면 정확히 0 이라 전역주행 위빙에 영향을 주지 않고,
  //   벗어났을 때만 작용한다. Stanley 형(속도로 나눔)이라 고속에서 과조향을 막는다.
  //   같은 조향각이 만드는 횡가속도가 v^2 에 비례하므로 나누는 게 맞다.
  //   ct_gain 0.5, 오차 0.5 m 기준 필요 횡가속: 2 m/s 1.5 / 3 m/s 2.3 / 5 m/s 3.8 m/s^2
  //   (실측 접지 한계 5.3). 1.0 으로 올리면 5 m/s 에서 7.8 로 한계를 넘는다.
  double ct_gain{0.0};
  double ct_max_rad{0.15};   // [rad] 이 항이 더할 수 있는 조향의 절대 상한
  double ct_v_min{1.0};      // [m/s] 나눗셈에 쓰는 속도의 하한(저속 과조향 방지)

  // ---- 정지 복구(후진) 파라미터 ------------------------------------------
  // state == "StateType.RECOVER" 일 때만 쓰인다. 생성자 인자를 늘리지 않고
  // l1_controller 가 생성 후에 YAML 값으로 덮어쓴다(동적 파라미터 갱신과 같은 방식).
  double reverse_l1_dist{0.7};      // [m] 후방 lookahead 거리
  double reverse_speed_max{0.5};    // [m/s] 후진 속도 크기 상한 (안전장치)
  double reverse_steer_max{0.32};   // [rad] 후진 중 조향각 상한
  int    reverse_stale_cycles{20};  // 로컬 웨이포인트가 이만큼 갱신 안 되면 정지

  // 상태 플래그 (원본 코드와 호환)
  bool flag1{false};

  // 내부 로깅 함수 교체 가능
  Logger logger_info, logger_warn;

private:
  // 내부 상태
  std::string state_;
  Pose3 pose_{0,0,0};
  std::vector<WpRow> wp_;
  double speed_now_{0.0};
  std::optional<Opp5> opp_;
  std::optional<Fren4> frenet_;
  std::vector<double> acc_now_;
  double track_length_{0.0};

  // controller 내부 파라미터
  std::vector<double> lateral_error_list_; // (미사용: 추후 분석 시 유지)
  double curr_steering_angle_{0.0};
  // 경로(wp_)까지의 부호 있는 횡오차 [m]. 양수 = 경로가 차의 왼쪽에 있다
  // (= 왼쪽으로 꺾어야 한다). calc_lateral_error_norm 이 매 주기 갱신한다.
  double cross_track_signed_{0.0};
  int idx_nearest_wp_{-1};
  double curvature_waypoints_{0.0};
  double max_curvature_{0.0};

  std::array<double,10> d_vs_{{0}};
  double acceleration_command_{0.0};

  // trailing 상태
  double gap_{0.0}, gap_should_{0.0}, gap_error_{0.0}, gap_actual_{0.0}, v_diff_{0.0};
  double i_gap_{0.0};
  double trailing_command_{2.0};
  double speed_command_{0.0};
  double trailing_speed_{0.0};

  // 내부 유틸
  static inline double clip(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }
  static inline double norm2(double x, double y) { return std::sqrt(x*x + y*y); }

  int nearest_waypoint(const Vec2& position, const std::vector<WpRow>& waypoints) const;
  Vec2 waypoint_at_distance_before_car(double distance,
                                       const std::vector<WpRow>& waypoints,
                                       int idx_waypoint_behind_car) const;

  std::pair<double,double> calc_lateral_error_norm();       // (lat_e_norm, lateral_error)
  double speed_adjust_lat_err(double global_speed, double lat_e_norm) const;
  double speed_adjust_heading(double speed_command) const;

  double acc_scaling(double steer) const;
  double speed_steer_scaling(double steer, double speed) const;

  double trailing_controller(double global_speed);

  Output reverse_loop();

  std::pair<Vec2,double> calc_L1_point(double lateral_error);
  double calc_steering_angle(const Vec2& L1_point, double L1_distance,
                             double yaw, double lat_e_norm,
                             const Vec2& v);
};
