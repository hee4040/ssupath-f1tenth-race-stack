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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
  // 매칭 실패가 몇 주기 연속됐는지. 측정이 들어오면 0 으로 리셋된다.
  // ttl 과 목적이 다르다 — ttl 은 '수명', 이건 '재검출 없이 버틴 시간'이다.
  // 래치가 걸린 트랙은 ttl 을 안 깎으므로 해제 판단은 이 값으로 한다.
  int unseen{0};

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

    // ======================= 정적 장애물 래치 =======================
    // 기본 동작(래치 off): 정적 트랙도 매 주기 ttl 이 깎여 sd_ttl/rate 초 뒤 삭제된다.
    //   sd_ttl 20 + rate 40 -> 0.5 s. 검출이 끊기면 0.5 초 만에 장애물이 없어진다.
    //   obs_debug_0819_1608 실측: 트랙 수명 중앙값이 정확히 0.50 s 였고,
    //   검출 끊김이 0.5 s 를 넘는 비율이 60.4% 였다 -> 열 번 중 여섯 번 트랙이 죽는다.
    //   그 결과가 "멈췄다가 0.5 초 뒤 재가속해서 같은 장애물을 밟고 지나감" 이다.
    //
    // 래치를 켜면 정적 트랙의 해제 조건이 '시간'에서 '사건'으로 바뀐다:
    //   차가 그 지점을 지나가기 전까지는 재검출이 없어도 유지한다.
    //
    // ※ 래치는 오검출도 같이 영구화하므로 아래 두 안전장치가 함께 필요하다:
    //   - latch_min_hits : 한 번 튄 검출은 래치하지 않는다(연속 측정 N 회 필요)
    //   - latch_max_hold : 시야가 뚫려 있는데 그만큼 재검출이 없으면 오검출로 보고 해제
    //
    // ※ 전제: 유령 비율이 낮아야 한다. obs_debug_0819_1209(min_intrusion 0.18)는
    //   검출의 32.3% 가 트랙 밖 유령이었고, 그 상태로 래치를 켜면 유령마다 영구 정지한다.
    //   1608(min_intrusion 0.33)은 4.5% 라 실용 범위다. min_intrusion 을 낮출 거면
    //   래치도 같이 재검토할 것.
    latch_until_passed_   = declare_parameter<bool>("latch_static_until_passed", false);

    // 래치를 걸기 전 필요한 측정 횟수. 1 이면 단발 검출도 래치된다(권장하지 않음).
    // staticFlag 자체가 sd_min_nb_meas(2) 초과에서만 정해지므로 실효 하한은 3 이다.
    latch_min_hits_       = declare_parameter<int>("latch_min_hits", 2);

    // 차가 이만큼 지나쳤으면 해제 [m]. 차 뒤쪽 여유를 준다.
    latch_release_behind_ = declare_parameter<double>("latch_release_behind", 0.5);

    // 오검출 해제까지 버티는 시간 [s]. 시야가 뚫려 있는데(inFOV) 이만큼 재검출이
    // 없으면 놓아준다. 1608 기준 검출 끊김 p90 이 4.90 s 라 그보다 넉넉해야
    // 진짜 장애물을 성급히 버리지 않는다. 0 이하면 이 해제를 끈다(영구 유지).
    latch_max_hold_       = declare_parameter<double>("latch_max_hold", 5.0);

    // ★ 교착 방지 하드 캡 [s]. 가시성 판정과 **무관하게** 이만큼 재검출이 없으면 해제한다.
    //   latch_max_hold 는 inFOV(="그 자리가 비었나")가 맞아야만 동작하는데, 그 판정이
    //   틀리면 차가 영원히 서 있게 된다. 실제로 2026-08-19 주행에서 그렇게 됐다.
    //   이건 그 안전망이라 항상 켜 두는 것을 권장한다. 0 이하면 끔(권장 안 함).
    latch_hard_release_   = declare_parameter<double>("latch_hard_release", 10.0);

    // inFOV 의 방위각 기준 프레임. /scan 의 frame_id 와 이 프레임 사이 회전을 TF 로 받는다.
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

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
          else if (name=="latch_static_until_passed") latch_until_passed_ = p.as_bool();
          else if (name=="latch_min_hits") latch_min_hits_ = p.as_int();
          else if (name=="latch_release_behind") latch_release_behind_ = p.as_double();
          else if (name=="latch_max_hold") latch_max_hold_ = p.as_double();
          else if (name=="latch_hard_release") latch_hard_release_ = p.as_double();
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
    scan_frame_ = msg->header.frame_id;
    resolveScanYaw();
  }

  // ★ 2026-08-19: /scan 은 라이다 프레임(livox_frame)이고 angleToObs 는 base_link
  //   기준 방위각을 낸다. 이 회전을 빼지 않으면 **엉뚱한 빔을 읽는다.**
  //   이 차량은 base_link->livox_frame 이 +91.6도라 거의 직각으로 틀어져 있었다:
  //     obs_debug_0819_1608, 정면 2.5 m 이내 장애물 896건
  //       보정 전 조회 빔 min_scan 중앙 0.88 m  (옆 벽을 읽고 있었다)
  //       보정 후 조회 빔 min_scan 중앙 1.71 m  (실제 장애물 거리 1.72 m 와 일치)
  //   그래서 inFOV 가 62.9% 확률로 "안 비었다"를 반환했고, 래치가 영원히 안 풀렸다.
  //   (noMemoryMode_=true 인 동안은 inFOV 자체가 죽은 코드라 아무도 몰랐다.)
  void resolveScanYaw() {
    if (scan_yaw_ok_ || scan_frame_.empty()) return;
    try {
      const auto tf = tf_buffer_.lookupTransform(robot_frame_, scan_frame_, tf2::TimePointZero);
      const auto &q = tf.transform.rotation;
      scan_yaw_offset_ = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                                    1.0 - 2.0*(q.y*q.y + q.z*q.z));
      scan_yaw_ok_ = true;
      RCLCPP_INFO(get_logger(), "[tracking] scan frame '%s' yaw offset = %.1f deg (기준 '%s')",
                  scan_frame_.c_str(), scan_yaw_offset_*180.0/M_PI, robot_frame_.c_str());
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "[tracking] %s->%s TF 없음 — inFOV 가 엉뚱한 빔을 읽는다: %s",
        robot_frame_.c_str(), scan_frame_.c_str(), e.what());
    }
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
    // base_link 기준 방위각 -> 스캔(라이다) 프레임 기준으로 변환한다.
    double bearing = angleToObs(vec_to_obs, car_ori_) - scan_yaw_offset_;
    // 스캔 범위가 [-270, +45] 처럼 -pi..pi 밖으로 뻗어 있을 수 있어 한 바퀴 접어서도 시도한다.
    if (bearing > scan_max_angle_) {
      if (bearing - 2.0*M_PI >= scan_min_angle_) bearing -= 2.0*M_PI;
      else return false;
    } else if (bearing < scan_min_angle_) {
      if (bearing + 2.0*M_PI <= scan_max_angle_) bearing += 2.0*M_PI;
      else return false;
    }
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
    trk.unseen = 0;
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
        // 이번 주기에 매칭된 측정이 없다.
        trk.unseen += 1;

        // ---------------- 정적 장애물 래치 ----------------
        // 조건을 만족하면 ttl 을 아예 건드리지 않는다. 따라서 아래 ttl<=0 경로로
        // 내려가지 않고, 해제는 오직 (A) 지나감 / (B) 오검출 판정 두 가지로만 일어난다.
        const bool latched =
            latch_until_passed_ &&
            trk.staticFlag.has_value() && trk.staticFlag.value() &&
            trk.nb_meas >= latch_min_hits_;

        if (latched) {
          trk.isInFront = checkInFront(trk.meas_s.back());

          // (A) 차가 지나갔나?  gap>0 앞, gap<0 뒤.
          //     calcDistanceObsCarS 는 [0,L) 이라 '뒤'와 '한 바퀴 앞'을 구분 못 해서
          //     여기서는 대칭 래핑을 쓴다.
          const double gap =
              wrap_s_residual_sym(trk.mean_s - car_s_.value(), track_length_);

          if (gap < -latch_release_behind_) {
            to_rm_idx.push_back(i);          // 지나쳤다 -> 해제
          } else {
            // (B) 시야가 뚫려 있는데 너무 오래 재검출이 없으면 오검출로 보고 해제.
            //     이게 없으면 한 번 튄 유령이 영원히 트랙을 막는다.
            auto xy = sdToXY(trk.mean_s, trk.mean_d);
            Eigen::Vector2d obs_xy(xy.first, xy.second);
            Eigen::Vector2d car_xy(car_pos_[0], car_pos_[1]);
            trk.isVisible = inFOV(obs_xy - car_xy);

            const int hold_cycles =
                (latch_max_hold_ > 0.0)
                  ? static_cast<int>(latch_max_hold_ * std::max(1, OpponentState::rate))
                  : std::numeric_limits<int>::max();

            const int hard_cycles =
                (latch_hard_release_ > 0.0)
                  ? static_cast<int>(latch_hard_release_ * std::max(1, OpponentState::rate))
                  : std::numeric_limits<int>::max();

            if (trk.isVisible && trk.unseen > hold_cycles) {
              to_rm_idx.push_back(i);        // 보일 자리인데 계속 안 보인다 -> 해제
            } else if (trk.unseen > hard_cycles) {
              // ★ 교착 방지: 가시성 판정과 무관한 상한. 이게 없으면 inFOV 가 틀렸을 때
              //   차가 영원히 서 있는다(2026-08-19 실차에서 발생).
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[tracking] 래치 하드 캡으로 해제 (id=%d, %.1f s 무검출). "
                "inFOV 가 계속 false 라면 scan 프레임 보정을 확인할 것.",
                trk.id, latch_hard_release_);
              to_rm_idx.push_back(i);
            }
            // 그 외에는 아무것도 하지 않는다 = 유지된다.
          }
        }
        else if (trk.ttl<=0){
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

  // ★ 2026-08-22 (1): DELETEALL 을 같은 MarkerArray 의 첫 원소로 넣는다.
  //   예전에는 clear 를 별도 메시지로 먼저 publish 하고 마커를 두 번째 메시지로 보냈다.
  //   rviz 는 메시지 단위로 처리하므로 두 메시지 사이에 렌더 틱이 끼면 그 프레임은
  //   통째로 빈 화면이 된다. 실시간에서는 두 메시지 간격이 중앙 0.08 ms(p90 0.27 ms,
  //   max 12.4 ms) 라 티가 잘 안 나지만, 백 재생에서는 rosbag2 가 디스크에서 묶어
  //   읽어 내보내므로 간격이 훨씬 커진다 — "라이브에서는 보이는데 녹화본에서는
  //   드문드문 보인다" 의 정체가 이것이다. (녹화 손실이 아니다: obs_debug_0822_1211
  //   에서 DELETEALL 이 정확히 40.01 Hz = 노드 rate 로 다 잡혀 있었다.)
  //   한 메시지로 보내면 clear -> add 가 원자적으로 처리돼 빈 프레임이 없어진다.
  //   덤으로 토픽 발행률이 절반(80 -> 40 Hz)이 되어 QoS depth 5 에도 여유가 생긴다.
  visualization_msgs::msg::MarkerArray arr;
  {
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);
  }

  // (A) 추적 중인 장애물. 정적/동적/미정을 색으로 구분한다.
  //       초록 = 정적 / 빨강 = 동적 / 자홍 = 미정(측정이 sd_min_nb_meas 이하라 판정 전)
  //
  // ★ 2026-08-22 (2): 동적 트랙을 그리는 가지를 추가했다.
  //   예전에는 draw_unknown / draw_static 두 가지뿐이라 staticFlag == false 인 트랙이
  //   통째로 스킵됐다. 즉 '동적으로 판정된 장애물' 은 rviz 에서 완전히 투명했고,
  //   화면의 빨간 원은 아래 (B) 의 opponent EKF 마커 하나뿐이었다.
  //   (주석 처리된 옛 버전에도 tracked_dynamic ns 는 있었지만 색 분기가 없어
  //    rgb(0,0,0) = 검정으로 그려졌다. 결국 안 보이기는 마찬가지였다.)
  for (const auto &t : tracked_) {
    if (!t.isInFront) continue;

    const bool is_unknown = !t.staticFlag.has_value();
    const bool is_static  = !is_unknown &&  t.staticFlag.value();
    const bool is_dynamic = !is_unknown && !t.staticFlag.value();

    // publish_static_ 는 '정적 장애물을 하류로 내보낼지' 를 정하는 값이므로
    // 정적/미정에만 건다. 동적은 이 값과 무관하게 항상 그린다.
    if ((is_unknown || is_static) && !publish_static_) continue;

    // 좌표: 정적만 평균을 쓴다(제자리에 있으니 평균이 더 안정적).
    // 동적/미정은 움직이므로 평균이 의미가 없다 - 가장 최근 측정을 쓴다.
    const double s_draw = is_static ? t.mean_s : t.meas_s.back();
    const double d_draw = is_static ? t.mean_d : t.meas_d.back();
    auto xy = sdToXY(s_draw, d_draw);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp    = current_stamp_;
    m.ns  = is_static  ? "tracked_static"
          : is_dynamic ? "tracked_dynamic"
                       : "tracked_unknown";
    m.id  = t.id;
    m.type = visualization_msgs::msg::Marker::SPHERE;

    if (t.isInFront) { m.scale.x = m.scale.y = m.scale.z = 0.5; }
    else             { m.scale.x = m.scale.y = m.scale.z = 0.25; }

    m.color.a = 0.5;
    if (is_static)       { m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; } // 초록 = 정적
    else if (is_dynamic) { m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; } // 빨강 = 동적
    else                 { m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0; } // 자홍 = 미정

    m.pose.orientation.w = 1.0;
    m.pose.position.x = xy.first;
    m.pose.position.y = xy.second;

    arr.markers.push_back(m);
  }

  // (B) EKF 상대 차량 마커 (항상 별도 ns: "opponent")
  //   위 tracked_dynamic 과 같은 물체를 가리키며 색도 같은 빨강이다 — 하나는 측정,
  //   하나는 EKF 추정이라 둘이 조금 어긋나 보이는 것이 정상이다. 구분해서 보려면
  //   rviz 의 Namespaces 에서 opponent 를 끄면 된다.
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

  // 3) 게시. DELETEALL 이 항상 첫 원소로 들어 있으므로 빈 메시지가 되는 경우는 없고,
  //    "지울 것만 있는 사이클" 도 이 한 번의 publish 로 처리된다.
  static_dynamic_marker_pub_->publish(arr);
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
      if (!t.isInFront || t.nb_meas <= 2) continue;

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
  // 정적 장애물 래치 (선언 위치의 주석은 constructor 쪽 참조)
  bool   latch_until_passed_{false};
  int    latch_min_hits_{2};
  double latch_release_behind_{0.5};
  double latch_max_hold_{5.0};
  double latch_hard_release_{10.0};
  // scan 프레임 보정
  std::string robot_frame_{"base_link"}, scan_frame_;
  double scan_yaw_offset_{0.0};
  bool   scan_yaw_ok_{false};
  tf2_ros::Buffer tf_buffer_{this->get_clock()};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
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
