#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <f110_msgs/msg/wpnt.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/obstacle_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>
#include <array>
#include <vector>
#include <deque>
#include <optional>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <memory>

#include <frenet_conversion_cpp/frenet_converter_cpp.hpp>

// ===================== helpers 
// 잔차용 s-래핑: [-L/2, L/2) 로 접기 (이름을 명확히 분리)
static inline double wrap_s_residual_sym(double ds, double L) {
  if (L <= 0.0) return ds;
  ds = std::fmod(ds, L);
  if (ds < -0.5 * L) ds += L;
  else if (ds >= 0.5 * L) ds -= L;
  return ds;
}

// [0, L)로 좌표 래핑
static inline double wrap_s_coord(double s, double L) {
  if (L <= 0.0) return s;
  s = std::fmod(s, L);
  if (s < 0.0) s += L;
  return s;
}

struct Waypoint { double x{}, y{}, psi{}, s{}; };

// ===================== ObstacleSD =====================
struct ObstacleSD {
  static int    min_nb_meas;
  static int    ttl_param;
  static double min_std;
  static double max_std;

  int id;
  std::deque<double> meas_s;
  std::deque<double> meas_d;
  double mean_s{0.0};
  double mean_d{0.0};
  int static_count{0};
  int total_count{0};
  int nb_meas{0};
  int ttl{ttl_param};
  bool isInFront{true};
  int  current_lap{0};
  std::optional<bool> staticFlag;  // nullopt == 미정
  double size{0.0};
  int nb_detection{0};
  bool isVisible{true};

  ObstacleSD(int id_, double s_meas, double d_meas, int lap, double sz, bool vis)
  : id(id_), meas_s{ s_meas }, meas_d{ d_meas }, mean_s(s_meas), mean_d(d_meas),
    current_lap(lap), size(sz), isVisible(vis) {}

  double std_s(double track_length) const {
    if (meas_s.empty()) return 0.0;
    double acc=0.0;
    for (double s : meas_s) {
      double d = wrap_s_residual_sym(s - mean_s, track_length);
      acc += d*d;
    }
    return std::sqrt(acc / double(meas_s.size()));
  }
  double std_d() const {
    if (meas_d.empty()) return 0.0;
    double mu=0.0; for (double v:meas_d) mu+=v; mu/=double(meas_d.size());
    double acc=0.0; for (double v:meas_d){double dv=v-mu; acc+=dv*dv;}
    return std::sqrt(acc / double(meas_d.size()));
  }

  void update_mean(double track_length){
    if (nb_meas==0){ mean_s = meas_s.back(); mean_d = meas_d.back(); }
    else{
      // d: 일반 평균
      mean_d = (mean_d * nb_meas + meas_d.back()) / double(nb_meas+1);
      // s: 원형 평균
      double prev = mean_s * 2.0*M_PI/track_length;
      double cur  = meas_s.back() * 2.0*M_PI/track_length;
      double c = (std::cos(prev)*nb_meas + std::cos(cur)) / double(nb_meas+1);
      double s = (std::sin(prev)*nb_meas + std::sin(cur)) / double(nb_meas+1);
      double ang = std::atan2(s,c);
      double ms = ang * track_length / (2.0*M_PI);
      mean_s = (ms>=0.0)? ms : (ms+track_length);
    }
  }
  void isStatic(double track_length){
    if (nb_meas > min_nb_meas){
      double sstd = std_s(track_length);
      double dstd = std_d();
      if (sstd < min_std && dstd < min_std) static_count++;
      else if (sstd > max_std || dstd > max_std) static_count = 0;
      total_count++;
      staticFlag = (double(static_count)/std::max(1,total_count)) >= 0.5;
    } else staticFlag = std::nullopt;
  }
};

int ObstacleSD::min_nb_meas = 2;
int ObstacleSD::ttl_param   = 20;
double ObstacleSD::min_std  = 0.16;
double ObstacleSD::max_std  = 0.22;

// ===================== Opponent EKF =====================
class OpponentState {
public:
  // ---- static params ----
  static double track_length;
  static int    rate;
  static double dt;
  static double P_vs, P_d, P_vd;
  static double meas_var_s, meas_var_d, meas_var_vs, meas_var_vd;
  static double proc_var_vs, proc_var_vd;
  static double ratio_to_glob_path;
  static Eigen::VectorXd path_vx;   // 샘플링된 목표 속도 [m/s] (optional)
  static double s_index_scale;      // s[m] * scale = index (e.g., 10 -> 0.1m 해상도)

  // ---- state / buffers ----
  bool isInitialised{false};
  int id{0};
  double size{0.0};
  int ttl{40};
  bool useTargetVel{false};
  std::deque<double> vs_list;
  double avg_vs{0.0};

  Eigen::Vector4d x;            // [s, vs, d, vd]
  Eigen::Matrix4d F,Q,H,R,P,B;
  std::array<double,5> vs_filt{{0,0,0,0,0}};
  std::array<double,5> vd_filt{{0,0,0,0,0}};

  OpponentState(){
    H.setIdentity();
    B.setIdentity();
    x.setZero();
    rebuild_matrices(); // 파라미터를 반영한 F/Q/R 구성
    // 초기 P를 측정/프로세스 분산으로 설정
    P.setZero();
    P(0,0) = meas_var_s;
    P(1,1) = proc_var_vs;
    P(2,2) = meas_var_d;
    P(3,3) = meas_var_vd;
  }

  // 파라미터/주기 변경 시 반드시 호출
  void rebuild_matrices() {
    F.setIdentity();
    F(0,1) = dt;
    F(2,3) = dt;
    Q = make_Q_cv_block(dt, proc_var_vs, proc_var_vd);

    R.setZero();
    R(0,0) = meas_var_s;
    R(1,1) = meas_var_vs;
    R(2,2) = meas_var_d;
    R(3,3) = meas_var_vd;
  }

  void predict(){
    Eigen::Vector4d u;
    if (useTargetVel) {
      u << 0.0,
           P_vs * (target_velocity() - x(1)),
          -P_d * x(2),
          -P_vd * x(3);
    } else {
      u << 0.0,
           0.0,
          -P_d * x(2),
          -P_vd * x(3);
    }

    x = F * x + B * u;
    x(0) = wrap_s_coord(x(0), track_length);
    P = F * P * F.transpose() + Q;
  }

  // z = [s, vs, d, vd]
  void update(double zs, double zvs, double zd, double zvd){
    Eigen::Vector4d z;  z << zs, zvs, zd, zvd;

    Eigen::Vector4d hx;
    hx << wrap_s_coord(x(0), track_length), x(1), x(2), x(3);

    Eigen::Vector4d y = z - hx;
    y(0) = wrap_s_residual_sym(y(0), track_length);

    const Eigen::Matrix4d S = H * P * H.transpose() + R;
    const Eigen::Matrix4d K = P * H.transpose() * S.inverse();

    x = x + K * y;
    x(0) = wrap_s_coord(x(0), track_length);

    const Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    P = (I - K * H) * P;

    vs_list.push_back(x(1));
    if (vs_list.size() > 20) vs_list.erase(vs_list.begin(), vs_list.end() - 10);

    avg_vs = 0.0;
    for (double v : vs_list) avg_vs += v;
    if (!vs_list.empty()) avg_vs /= static_cast<double>(vs_list.size());

    for (int i = 4; i > 0; --i) {
      vs_filt[i] = vs_filt[i-1];
      vd_filt[i] = vd_filt[i-1];
    }
    vs_filt[0] = x(1);
    vd_filt[0] = x(3);
  }

  // EKF 초기화 시 P/Q/R 리셋 (권장)
  void reset_covariances_for_init() {
    P.setIdentity();
    P(0,0)=meas_var_s*10.0;
    P(1,1)=proc_var_vs*10.0;
    P(2,2)=meas_var_d*10.0;
    P(3,3)=meas_var_vd*10.0;
    rebuild_matrices();
  }

private:
  static inline Eigen::Matrix2d make_Q_cv(double dt_, double q) {
    const double dt2 = dt_ * dt_;
    const double dt3 = dt2 * dt_;
    const double dt4 = dt3 * dt_;
    Eigen::Matrix2d Qcv;
    Qcv << 0.25 * dt4 * q, 0.5 * dt3 * q,
           0.5 * dt3 * q,       dt2 * q;
    return Qcv;
  }
  static inline Eigen::Matrix4d make_Q_cv_block(double dt_, double q_vs, double q_vd) {
    Eigen::Matrix4d Qblk; Qblk.setZero();
    Qblk.block<2,2>(0,0) = make_Q_cv(dt_, q_vs);
    Qblk.block<2,2>(2,2) = make_Q_cv(dt_, q_vd);
    return Qblk;
  }

  double target_velocity() const {
    const Eigen::Index N = path_vx.size();
    if (N <= 0 || track_length <= 0.0 || s_index_scale <= 0.0) return 0.0;
    long long idx = static_cast<long long>(std::floor(x(0) * s_index_scale));
    long long m = static_cast<long long>(N);
    idx %= m; if (idx < 0) idx += m;
    return ratio_to_glob_path * path_vx(static_cast<Eigen::Index>(idx));
  }
};

double OpponentState::track_length = -1.0;
int    OpponentState::rate = 40;
double OpponentState::dt = 1.0/40.0;
double OpponentState::P_vs = 0.2;
double OpponentState::P_d  = 0.02;
double OpponentState::P_vd = 0.2;
double OpponentState::meas_var_s  = 0.002;
double OpponentState::meas_var_d  = 0.002;
double OpponentState::meas_var_vs = 0.2;
double OpponentState::meas_var_vd = 0.2;
double OpponentState::proc_var_vs = 2.0;
double OpponentState::proc_var_vd = 8.0;
double OpponentState::ratio_to_glob_path = 0.3;
Eigen::VectorXd OpponentState::path_vx;       // 기본은 비어 있음
double OpponentState::s_index_scale = 10.0;   // s[m] * 10 -> 0.1 m 인덱싱

// ===================== Node =====================
class StaticDynamicNode : public rclcpp::Node {
public:
  StaticDynamicNode()
  : rclcpp::Node("tracking")
  {
    // --- parameters (기존 + 추가 노출) ---
    update_rate_ = declare_parameter<int>("rate", 40);
    OpponentState::rate = update_rate_;
    OpponentState::dt   = 1.0 / std::max(1, update_rate_);
    OpponentState::P_vs = declare_parameter<double>("P_vs", 0.2);
    OpponentState::P_d  = declare_parameter<double>("P_d",  0.02);
    OpponentState::P_vd = declare_parameter<double>("P_vd", 0.2);
    OpponentState::meas_var_s  = declare_parameter<double>("measurment_var_s", 0.002);
    OpponentState::meas_var_d  = declare_parameter<double>("measurment_var_d", 0.002);
    OpponentState::meas_var_vs = declare_parameter<double>("measurment_var_vs", 0.2);
    OpponentState::meas_var_vd = declare_parameter<double>("measurment_var_vd", 0.2);
    OpponentState::proc_var_vs = declare_parameter<double>("process_var_vs", 2.0);
    OpponentState::proc_var_vd = declare_parameter<double>("process_var_vd", 8.0);
    max_dist_       = declare_parameter<double>("max_dist", 0.5);
    var_pub_        = declare_parameter<double>("var_pub", 1.0); // double로 변경
    dist_deletion_  = declare_parameter<double>("dist_deletion", 6.0);
    dist_infront_   = declare_parameter<double>("dist_infront", 7.0);
    vs_reset_       = declare_parameter<double>("vs_reset", 0.1);
    publish_static_ = declare_parameter<bool>("publish_static", true);
    noMemoryMode_   = declare_parameter<bool>("noMemoryMode", true);
    OpponentState::ratio_to_glob_path = declare_parameter<double>("ratio_to_glob_path", 0.3);
    aggro_multiplier_ = declare_parameter<double>("aggro_multi", 2.0);

    // ObstacleSD 임계치도 파라미터화
    ObstacleSD::min_nb_meas = declare_parameter<int>("sd_min_nb_meas", 2);
    ObstacleSD::ttl_param   = declare_parameter<int>("sd_ttl", 20);
    ObstacleSD::min_std     = declare_parameter<double>("sd_min_std", 0.16);
    ObstacleSD::max_std     = declare_parameter<double>("sd_max_std", 0.22);

    // --- pubs ---
    static_dynamic_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/perception/static_dynamic_marker_pub", 5);
    estimated_obstacles_pub_   = create_publisher<f110_msgs::msg::ObstacleArray>("/perception/obstacles", 5);
    raw_obstacles_pub_         = create_publisher<f110_msgs::msg::ObstacleArray>("/perception/raw_obstacles", 5);
    if (declare_parameter<bool>("measure", false)) {
      latency_pub_ = create_publisher<std_msgs::msg::Float32>("/perception/tracking/latency", 10);
      measuring_ = true;
    }

    // --- subs ---
    obs_sub_  = create_subscription<f110_msgs::msg::ObstacleArray>("/perception/detection/raw_obstacles", 10,
                std::bind(&StaticDynamicNode::obstacleCallback, this, std::placeholders::_1));
    wps_sub_  = create_subscription<f110_msgs::msg::WpntArray>("/global_waypoints", 10,
                std::bind(&StaticDynamicNode::pathCallback, this, std::placeholders::_1));
    fr_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/car_state/frenet/odom", 10,
                std::bind(&StaticDynamicNode::carStateFrenetCB, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/car_state/odom", 10,
                std::bind(&StaticDynamicNode::carStateGlobCB, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
                std::bind(&StaticDynamicNode::scanCB, this, std::placeholders::_1));

    // --- timer ---
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1, update_rate_)),
              std::bind(&StaticDynamicNode::loop, this));

    // --- 동적 파라미터 콜백: 변경 즉시 EKF 행렬 재빌드 ---
    param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params)
      -> rcl_interfaces::msg::SetParametersResult {
        for (const auto& p : params) {
          const auto &name = p.get_name();
          if (name=="P_vs") OpponentState::P_vs = p.as_double();
          else if (name=="P_d") OpponentState::P_d = p.as_double();
          else if (name=="P_vd") OpponentState::P_vd = p.as_double();
          else if (name=="measurment_var_s") OpponentState::meas_var_s = p.as_double();
          else if (name=="measurment_var_d") OpponentState::meas_var_d = p.as_double();
          else if (name=="measurment_var_vs") OpponentState::meas_var_vs = p.as_double();
          else if (name=="measurment_var_vd") OpponentState::meas_var_vd = p.as_double();
          else if (name=="process_var_vs") OpponentState::proc_var_vs = p.as_double();
          else if (name=="process_var_vd") OpponentState::proc_var_vd = p.as_double();
          else if (name=="rate") {
            update_rate_ = p.as_int();
            OpponentState::rate = update_rate_;
            OpponentState::dt = 1.0 / std::max(1, update_rate_);
          }
          else if (name=="ratio_to_glob_path") OpponentState::ratio_to_glob_path = p.as_double();
          else if (name=="sd_min_nb_meas") ObstacleSD::min_nb_meas = p.as_int();
          else if (name=="sd_ttl") ObstacleSD::ttl_param = p.as_int();
          else if (name=="sd_min_std") ObstacleSD::min_std = p.as_double();
          else if (name=="sd_max_std") ObstacleSD::max_std = p.as_double();
          else if (name=="max_dist") max_dist_ = p.as_double();
          else if (name=="var_pub") var_pub_ = p.as_double();
          else if (name=="dist_deletion") dist_deletion_ = p.as_double();
          else if (name=="dist_infront") dist_infront_ = p.as_double();
          else if (name=="publish_static") publish_static_ = p.as_bool();
          else if (name=="noMemoryMode") noMemoryMode_ = p.as_bool();
          else if (name=="aggro_multi") aggro_multiplier_ = p.as_double();
        }
        opponent_.rebuild_matrices();

        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        res.reason = "updated";
        return res;
      });


    // EKF 행렬 초기 구성
    opponent_.rebuild_matrices();

    RCLCPP_INFO(get_logger(), "[Tracking] init, rate=%d", update_rate_);
  }

private:
  // --- Callbacks ---
  void obstacleCallback(const f110_msgs::msg::ObstacleArray::SharedPtr msg) {
    meas_obstacles_ = msg->obstacles;
    current_stamp_  = msg->header.stamp;
  }

  void pathCallback(const f110_msgs::msg::WpntArray::SharedPtr msg) {
    if (initialized_track_) return;
    RCLCPP_INFO(get_logger(), "[Tracking] received global path");

    std::vector<double> xs, ys, psis, vxs;
    xs.reserve(msg->wpnts.size()); ys.reserve(msg->wpnts.size()); psis.reserve(msg->wpnts.size()); vxs.reserve(msg->wpnts.size());
    waypoints_.clear(); waypoints_.reserve(msg->wpnts.size());
    for (auto &w : msg->wpnts) {
      xs.push_back(w.x_m); ys.push_back(w.y_m); psis.push_back(w.psi_rad);
      waypoints_.push_back({w.x_m, w.y_m, w.psi_rad, w.s_m});
      // vx_mps 필드가 메시지에 존재한다고 가정 (없으면 주석 처리)
      vxs.push_back(w.vx_mps);
    }

    frenet_ = std::make_unique<FrenetConverter>(xs, ys, psis);
    track_length_ = frenet_->raceline_length();
    OpponentState::track_length = track_length_;

    // 타깃 속도 벡터 주입 (옵션)
    if (!vxs.empty()) {
      OpponentState::path_vx = Eigen::Map<Eigen::VectorXd>(vxs.data(), static_cast<Eigen::Index>(vxs.size()));
      OpponentState::s_index_scale = 10.0; // s*10 -> 0.1m 인덱싱 (파이썬과 동등)
    } else {
      OpponentState::path_vx.resize(0); // 비우기
    }

    initialized_track_ = true;
  }

  void carStateFrenetCB(const nav_msgs::msg::Odometry::SharedPtr msg) {
    car_s_ = msg->pose.pose.position.x;
    if (!last_car_s_.has_value()) last_car_s_ = car_s_;
  }

  void carStateGlobCB(const nav_msgs::msg::Odometry::SharedPtr msg) {
    car_pos_[0] = msg->pose.pose.position.x;
    car_pos_[1] = msg->pose.pose.position.y;

    const auto &q = msg->pose.pose.orientation;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw; tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    car_ori_[0] = std::cos(yaw);
    car_ori_[1] = std::sin(yaw);
  }

  void scanCB(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scans_ = msg->ranges;
    scan_max_angle_ = msg->angle_max;
    scan_min_angle_ = msg->angle_min;
    scan_increment_ = msg->angle_increment;
  }

  // --- Main loop ---
  void loop() {
    if (!initialized_track_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "did not get path yet");
      return;
    }
    if (!car_s_.has_value()) return;

    auto t0 = std::chrono::steady_clock::now();

    if (opponent_.isInitialised) opponent_.predict();
    updateTracking();
    publishObstacles();
    publishMarkers();

    auto t1 = std::chrono::steady_clock::now();
    if (measuring_ && latency_pub_) {
      std_msgs::msg::Float32 ms;
      ms.data = std::chrono::duration<float,std::milli>(t1 - t0).count();
      latency_pub_->publish(ms);
    }
  }

  // --- Utils ---
  bool checkInFront(double obj_s) const {
    double car_s = car_s_.value_or(0.0);
    double dist_front = wrap_s_residual_sym(obj_s - car_s, track_length_);
    return (0.0 < dist_front && dist_front < dist_infront_);
  }
  double calcDistanceObsCarS(double obs_s) const {
    double car_s = car_s_.value_or(0.0);
    double d = std::fmod(obs_s - car_s, track_length_);
    if (d < 0) d += track_length_;
    return d;
  }
  double angleToObs(const Eigen::Vector2d &vec_to_obs, const Eigen::Vector2d &car_ori) const {
    Eigen::Matrix2d R; R << car_ori[0], car_ori[1], -car_ori[1], car_ori[0];
    Eigen::Vector2d v = R * vec_to_obs;
    return std::atan2(v[1], v[0]);
  }
  bool inFOV(const Eigen::Vector2d &vec_to_obs) const {
    double dist = vec_to_obs.norm();
    double bearing = angleToObs(vec_to_obs, car_ori_);
    if (bearing > scan_max_angle_ || bearing < scan_min_angle_) return false;
    int idx = static_cast<int>(std::round((bearing - scan_min_angle_) / scan_increment_));
    if (idx < 0 || idx >= static_cast<int>(scans_.size())) return false;
    int lo = std::max(0, idx - 4), hi = std::min(idx + 4, static_cast<int>(scans_.size()));
    float min_scan = std::numeric_limits<float>::infinity();
    for (int i=lo;i<hi;++i) {
      float v = scans_[i];
      if (std::isfinite(v) && v > 0.05f) {
        if (v < min_scan) min_scan = v;
      }
    }
    if (!std::isfinite(min_scan)) return false;
    return dist < static_cast<double>(min_scan) * 0.98; // 약간 보수적 여유
  }

  std::pair<std::vector<f110_msgs::msg::Obstacle>, std::vector<double>>
  getClosestWithin(double max_dist, std::pair<double,double> sd,
                   const std::vector<f110_msgs::msg::Obstacle> &cands) const
  {
    std::vector<f110_msgs::msg::Obstacle> outs;
    std::vector<double> dists;
    for (auto &m : cands) {
      double d = std::hypot(sd.first - m.s_center, sd.second - m.d_center);
      if (d < max_dist) { outs.push_back(m); dists.push_back(d); }
    }
    return {outs, dists};
  }

  std::optional<f110_msgs::msg::Obstacle>
  verifyPosition(const ObstacleSD &trk, const std::vector<f110_msgs::msg::Obstacle> &cands) const
  {
    double maxd = max_dist_;
    std::pair<double,double> query_sd;
    if (trk.staticFlag.has_value() && trk.staticFlag.value() == false && opponent_.isInitialised) {
      double s = wrap_s_coord(opponent_.x(0), track_length_);
      query_sd = { s, opponent_.x(2) };
      maxd *= aggro_multiplier_;
    } else {
      query_sd = { trk.mean_s, trk.mean_d };
    }
    auto [outs, dists] = getClosestWithin(maxd, query_sd, cands);
    if (!dists.empty()){
      auto it = std::min_element(dists.begin(), dists.end());
      return outs[std::distance(dists.begin(), it)];
    }
    if (trk.staticFlag.has_value() && trk.staticFlag.value()==false){
      auto [o2,d2] = getClosestWithin(maxd, {trk.mean_s, trk.mean_d}, cands);
      if (!d2.empty()){
        auto it = std::min_element(d2.begin(), d2.end());
        return o2[std::distance(d2.begin(), it)];
      }
    }
    return std::nullopt;
  }

  inline std::pair<double,double> sdToXY(double s, double d) const {
    return frenet_->get_cartesian(s, d);
  }

  void initializeDynamic(ObstacleSD &trk){
    if (trk.meas_s.size()<2 || trk.meas_d.size()<2) return;
    opponent_.x <<
      trk.meas_s.back(),
      (trk.meas_s.back() - trk.meas_s[trk.meas_s.size()-2]) * OpponentState::rate,
      trk.meas_d.back(),
      (trk.meas_d.back() - trk.meas_d[trk.meas_d.size()-2]) * OpponentState::rate;
    opponent_.isInitialised = true;
    opponent_.id = trk.id;
    opponent_.ttl = 40;
    opponent_.size = trk.size;
    opponent_.avg_vs = 0.0;
    opponent_.vs_list.clear();
    opponent_.reset_covariances_for_init(); // 초기 P/Q/R 리셋
  }

  void updateTrackedObstacle(ObstacleSD &trk, const f110_msgs::msg::Obstacle &meas){
    trk.meas_s.push_back(meas.s_center);
    trk.meas_d.push_back(meas.d_center);
    if (trk.meas_s.size()>30){
      while(trk.meas_s.size()>20) trk.meas_s.pop_front();
      while(trk.meas_d.size()>20) trk.meas_d.pop_front();
    }
    trk.update_mean(track_length_);
    trk.nb_meas += 1;
    trk.isInFront = true;
    trk.isVisible = true;
    trk.current_lap = current_lap_;
    trk.size = meas.size;
    trk.isStatic(track_length_);
    trk.ttl = ObstacleSD::ttl_param;
  }

  void updateTracking(){
    if (!car_s_.has_value() || !initialized_track_) return;
    auto meas_copy = meas_obstacles_;
    std::vector<size_t> to_rm_idx; // 안전하게 index로 제거

    for (size_t i=0; i<tracked_.size(); ++i){
      auto &trk = tracked_[i];
      auto mopt = verifyPosition(trk, meas_copy);
      if (mopt.has_value()){
        auto meas = mopt.value();
        updateTrackedObstacle(trk, meas);
        if (trk.staticFlag.has_value() && trk.staticFlag.value()==false){
          if (opponent_.isInitialised){
            opponent_.useTargetVel = false;
            if (trk.meas_s.size()>=3){
              size_t n = trk.meas_s.size();
              double vs = ( (2.0/3.0)*(trk.meas_s[n-1]-trk.meas_s[n-2])*OpponentState::rate
                          + (1.0/3.0)*(trk.meas_s[n-2]-trk.meas_s[n-3])*OpponentState::rate );
              if (vs>-1.0 && vs<8.0){
                double vd = (trk.meas_d[n-1]-trk.meas_d[n-2]) * OpponentState::rate;
                opponent_.update(trk.meas_s.back(), vs, trk.meas_d.back(), vd);
                opponent_.id = trk.id; opponent_.ttl = 40; opponent_.size = trk.size;
              } else {
                opponent_.isInitialised = false;
              }
            }
          } else initializeDynamic(trk);
        }
        // meas_copy에서 방금 사용한 측정 제거 (가장 가까운 것)
        if (!meas_copy.empty()){
          auto it = std::min_element(meas_copy.begin(), meas_copy.end(),
            [&](const auto &a, const auto &b){
              double da = std::hypot(a.s_center - meas.s_center, a.d_center - meas.d_center);
              double db = std::hypot(b.s_center - meas.s_center, b.d_center - meas.d_center);
              return da < db;
            });
          if (it!=meas_copy.end()) meas_copy.erase(it);
        }
      } else {
        if (trk.ttl<=0){
          if (trk.staticFlag.has_value() && trk.staticFlag.value()==false) opponent_.useTargetVel = true;
          to_rm_idx.push_back(i);
        } else if (!trk.staticFlag.has_value()){
          trk.ttl -= 1;
        } else {
          trk.isInFront = checkInFront(trk.meas_s.back());
          double dist_s = calcDistanceObsCarS(trk.meas_s.back());

          if (trk.staticFlag.value() && noMemoryMode_) trk.ttl -= 1;
          else if (trk.staticFlag.value() && dist_s < dist_deletion_){
            auto xy = sdToXY(trk.mean_s, trk.mean_d);
            Eigen::Vector2d obs_xy(xy.first, xy.second);
            Eigen::Vector2d car_xy(car_pos_[0], car_pos_[1]);
            if (inFOV(obs_xy - car_xy)){ trk.ttl -= 1; trk.isVisible = true; }
            else trk.isVisible = false;
          } else if (trk.staticFlag.value()==false) trk.ttl -= 1;
          else trk.isVisible = false;
        }
      }
    }

    if (!to_rm_idx.empty()){
      // 뒤에서부터 제거
      std::sort(to_rm_idx.begin(), to_rm_idx.end());
      to_rm_idx.erase(std::unique(to_rm_idx.begin(), to_rm_idx.end()), to_rm_idx.end());
      for (int k = static_cast<int>(to_rm_idx.size())-1; k>=0; --k){
        tracked_.erase(tracked_.begin() + static_cast<long>(to_rm_idx[k]));
      }
    }

    if (opponent_.isInitialised){
      if (opponent_.ttl<=0){ opponent_.isInitialised=false; opponent_.useTargetVel=false; }
      else opponent_.ttl -= 1;
    }

    for (auto &m : meas_copy){
      tracked_.emplace_back(current_id_++, m.s_center, m.d_center, current_lap_, m.size, true);
    }
  }

  void publishMarkers() {
  if (!initialized_track_) return;

  // 1) 먼저 모두 지우고
  visualization_msgs::msg::MarkerArray clear;
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  clear.markers.push_back(del);
  static_dynamic_marker_pub_->publish(clear);

  // 2) 새 마커들 생성
  visualization_msgs::msg::MarkerArray arr;

  // (A) 정적/미정 장애물만 시각화 (publish_static_가 true일 때만)
  for (const auto &t : tracked_) {
    if (!t.isInFront) continue;

    // 파이썬 로직: staticFlag == None 이고 publish_static_=true → 그리기
    //             staticFlag == true 이고 publish_static_=true → 그리기
    //             그 외(동적 또는 publish_static_=false) → 스킵
    bool draw_unknown = (!t.staticFlag.has_value() && publish_static_);
    bool draw_static  = (t.staticFlag.has_value() && t.staticFlag.value() && publish_static_);
    if (!draw_unknown && !draw_static) continue;

    // 좌표 선택: 미정 → 가장 최근 측정, 정적 → 평균
    const double s_draw = draw_unknown ? t.meas_s.back() : t.mean_s;
    const double d_draw = draw_unknown ? t.meas_d.back() : t.mean_d;
    auto xy = sdToXY(s_draw, d_draw);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp    = current_stamp_;
    m.ns  = draw_static ? "tracked_static" : "tracked_unknown";
    m.id  = t.id;
    m.type = visualization_msgs::msg::Marker::SPHERE;

    if (t.isInFront) { m.scale.x = m.scale.y = m.scale.z = 0.5; }
    else             { m.scale.x = m.scale.y = m.scale.z = 0.25; }

    m.color.a = 0.5;
    if (draw_unknown) { m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0; } // magenta (unknown)
    if (draw_static)  { m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; } // green (static)

    m.pose.orientation.w = 1.0;
    m.pose.position.x = xy.first;
    m.pose.position.y = xy.second;

    arr.markers.push_back(m);
  }

  // (B) EKF 상대 차량 마커 (항상 별도 ns: "opponent")
  if (opponent_.isInitialised && checkInFront(opponent_.x(0))) {
    const double s_c = wrap_s_coord(opponent_.x(0), track_length_);
    auto xy = sdToXY(s_c, opponent_.x(2));

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp    = current_stamp_;
    m.ns  = "opponent";
    m.id  = opponent_.id;
    m.type = visualization_msgs::msg::Marker::SPHERE;

    const bool small_var = (opponent_.P(0,0) < var_pub_);
    m.scale.x = m.scale.y = m.scale.z = small_var ? 0.5 : 0.25;

    m.color.a = 0.5; m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; // red
    m.pose.orientation.w = 1.0;
    m.pose.position.x = xy.first;
    m.pose.position.y = xy.second;

    arr.markers.push_back(m);
  }

  // 3) 새 마커들 게시
  if (!arr.markers.empty()) {
    static_dynamic_marker_pub_->publish(arr);
  }
}

  // void publishMarkers(){
  //   if (!initialized_track_) return;
  //   visualization_msgs::msg::MarkerArray clear, out;
  //   visualization_msgs::msg::Marker del; del.action = visualization_msgs::msg::Marker::DELETEALL;
  //   clear.markers.push_back(del);
  //   static_dynamic_marker_pub_->publish(clear);

  //   for (const auto &t : tracked_){
  //     if (!t.isInFront) continue;
  //     if (!t.staticFlag.has_value() && !publish_static_) continue;
  //     if (t.staticFlag.has_value() && !t.staticFlag.value() && publish_static_) continue;

  //     auto xy = sdToXY(
  //       (t.staticFlag.has_value() && t.staticFlag.value()) ? t.mean_s : t.meas_s.back(),
  //       (t.staticFlag.has_value() && t.staticFlag.value()) ? t.mean_d : t.meas_d.back()
  //     );

  //     visualization_msgs::msg::Marker m;
  //     m.header.frame_id = "map";
  //     m.header.stamp = current_stamp_;
  //     m.ns = t.staticFlag.has_value()
  //       ? (t.staticFlag.value() ? "tracked_static" : "tracked_dynamic")
  //       : "tracked_unknown";
  //     m.id = t.id;
  //     m.type = visualization_msgs::msg::Marker::SPHERE;
  //     m.scale.x = m.scale.y = m.scale.z = t.isInFront ? 0.5 : 0.25;
  //     m.color.a = 0.5;
  //     if (!t.staticFlag.has_value()) { m.color.r=1.0; m.color.g=0.0; m.color.b=1.0; }
  //     else if (t.staticFlag.value()) { m.color.r=0.0; m.color.g=1.0; m.color.b=0.0; }
  //     m.pose.orientation.w = 1.0;
  //     m.pose.position.x = xy.first;
  //     m.pose.position.y = xy.second;
  //     out.markers.push_back(m);
  //   }

  //   if (opponent_.isInitialised && checkInFront(opponent_.x(0))){
  //     double s_c = wrap_s_coord(opponent_.x(0), track_length_);
  //     auto xy = sdToXY(s_c, opponent_.x(2));
  //     visualization_msgs::msg::Marker m;
  //     m.header.frame_id = "map";
  //     m.header.stamp = current_stamp_;
  //     m.ns = "opponent";
  //     m.id = opponent_.id;
  //     m.type = visualization_msgs::msg::Marker::SPHERE;
  //     m.scale.x = m.scale.y = m.scale.z = (opponent_.P(0,0) < var_pub_ ? 0.5 : 0.25);
  //     m.color.a = 0.5; m.color.r=1.0; m.color.g=0.0; m.color.b=0.0;
  //     m.pose.orientation.w = 1.0;
  //     m.pose.position.x = xy.first;
  //     m.pose.position.y = xy.second;
  //     out.markers.push_back(m);
  //   }
  //   if (!out.markers.empty()) static_dynamic_marker_pub_->publish(out);
  // }

  void fillBounds(f110_msgs::msg::Obstacle &m) const {
    m.s_start = wrap_s_coord(m.s_center - m.size*0.5, track_length_);
    m.s_end   = wrap_s_coord(m.s_center + m.size*0.5, track_length_);
    m.d_right = m.d_center - m.size*0.5;
    m.d_left  = m.d_center + m.size*0.5;
  }

  void publishObstacles(){
    f110_msgs::msg::ObstacleArray out, raw;
    out.header.frame_id = "map"; out.header.stamp = current_stamp_;
    raw.header = out.header;

    for (const auto &t : tracked_){

      // if (!t.isInFront) continue;
      if (!t.isInFront || t.nb_meas <=6) continue;

      f110_msgs::msg::Obstacle msg;
      msg.id = t.id; msg.size = t.size;
      msg.vs = 0.0f; msg.vd = 0.0f;
      msg.is_static = true; msg.is_actually_a_gap = false; msg.is_visible = t.isVisible;

      if (!t.staticFlag.has_value()){
        msg.s_center = wrap_s_coord(t.meas_s.back(), track_length_);
        msg.d_center = t.meas_d.back();
        fillBounds(msg);
        (publish_static_? out.obstacles : raw.obstacles).push_back(msg);
      } else if (t.staticFlag.value()){
        if (publish_static_){
          msg.s_center = t.mean_s; msg.d_center = t.mean_d;
          fillBounds(msg);
          out.obstacles.push_back(msg);
        }
      } else {
        msg.s_center = wrap_s_coord(t.meas_s.back(), track_length_);
        msg.d_center = t.meas_d.back();
        fillBounds(msg);
        raw.obstacles.push_back(msg);
      }
    }

    if (opponent_.isInitialised && opponent_.P(0,0) < var_pub_ && checkInFront(opponent_.x(0))){
      f110_msgs::msg::Obstacle msg;
      msg.id = opponent_.id; msg.size = opponent_.size;
      msg.vs = float((opponent_.vs_filt[0]+opponent_.vs_filt[1]+opponent_.vs_filt[2]+opponent_.vs_filt[3]+opponent_.vs_filt[4])/5.0);
      msg.vd = float((opponent_.vd_filt[0]+opponent_.vd_filt[1]+opponent_.vd_filt[2]+opponent_.vd_filt[3]+opponent_.vd_filt[4])/5.0);
      msg.is_static=false; msg.is_actually_a_gap=false; msg.is_visible=true;
      double s_c = wrap_s_coord(opponent_.x(0), track_length_);
      msg.s_center = s_c; msg.d_center = opponent_.x(2);
      fillBounds(msg);
      out.obstacles.push_back(msg);
    }

    estimated_obstacles_pub_->publish(out);
    raw_obstacles_pub_->publish(raw);
  }

private:
  // pubs/subs
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_dynamic_marker_pub_;
  rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr estimated_obstacles_pub_;
  rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr raw_obstacles_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_pub_;
  bool measuring_{false};

  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obs_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr wps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fr_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 동적 파라미터 핸들
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // params/state
  int update_rate_{40};
  double max_dist_{0.5};
  double var_pub_{1.0};
  double dist_deletion_{6.0};
  double dist_infront_{7.0};
  double vs_reset_{0.1};
  bool publish_static_{true};
  bool noMemoryMode_{true};
  double aggro_multiplier_{2.0};

  // track & Frenet
  bool initialized_track_{false};
  double track_length_{-1.0};
  std::vector<Waypoint> waypoints_;
  std::unique_ptr<FrenetConverter> frenet_;

  // car state
  std::optional<double> car_s_;
  std::optional<double> last_car_s_;
  int current_lap_{0};
  Eigen::Vector2d car_pos_{0.0, 0.0};
  Eigen::Vector2d car_ori_{1.0, 0.0};

  // scan
  std::vector<float> scans_;
  double scan_max_angle_{0.0}, scan_min_angle_{0.0}, scan_increment_{0.0};

  // data
  std::vector<f110_msgs::msg::Obstacle> meas_obstacles_;
  std_msgs::msg::Header::_stamp_type current_stamp_;
  std::vector<ObstacleSD> tracked_;
  int current_id_{1};

  OpponentState opponent_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticDynamicNode>());
  rclcpp::shutdown();
  return 0;
}
