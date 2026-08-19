#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
#include "Plotting/plotting.h"
#include "Spline/arc_length_spline.h"

#include <nlohmann/json.hpp>
#include <fstream>

#include <deque>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

#include "mpcc_ros/msg/s_hint.hpp"
#include "mpcc_ros/msg/corridor.hpp"
#include "mpcc_ros/msg/mpcc_control.hpp"

using json = nlohmann::json;
using namespace mpcc;
using std::placeholders::_1;

class MPCCNode : public rclcpp::Node {
public:
  MPCCNode() : Node("mpcc_node") {

      // ========= 파라미터 선언 =========
      // JSON 설정 파일 경로
      config_file_ = this->declare_parameter<std::string>(
          "config_file",
          "/home/misys/forza_ws/race_stack/controller/mpcc/MPCC/C++/Params/config_sim.json"
      );

      // 제어 루프 주기 (Hz)
      loop_rate_hz_ = this->declare_parameter<double>("loop_rate_hz", 40.0);

      // Corridor 사용 조건 (|s_corridor - s_state| < threshold)
      corridor_s_threshold_ = this->declare_parameter<double>("corridor_s_threshold", 0.5);

      // 스티어링 ↔ 서보 변환 계수 (vesc.yaml 값과 동일하게 맞추면 됨)
      steering_angle_to_servo_gain_ =
          this->declare_parameter<double>("steering_angle_to_servo_gain", -1.0);
      steering_angle_to_servo_offset_ =
          this->declare_parameter<double>("steering_angle_to_servo_offset", 0.5);

      // 초기 상태 x0 (실제로는 Odom 콜백으로 덮어쓰지만, 초기값은 파라미터에서 받게 해둠)
      std::vector<double> x0_vec_default(10, 0.0);
      auto x0_vec = this->declare_parameter<std::vector<double>>("x0", x0_vec_default);
      if (x0_vec.size() != 10) {
        RCLCPP_WARN(this->get_logger(),
          "Parameter 'x0' size (%zu) != 10. Falling back to default zeros.", x0_vec.size());
        x0_vec = x0_vec_default;
      }

      // ========= JSON 로드 =========
      std::ifstream iConfig(config_file_);
      if (!iConfig.is_open()) {
        RCLCPP_FATAL(this->get_logger(), "Failed to open config file: %s", config_file_.c_str());
        throw std::runtime_error("Cannot open config file");
      }

      json jsonConfig;
      iConfig >> jsonConfig;

      json_paths_ = PathToJson{
          jsonConfig["model_path"],
          jsonConfig["cost_path"],
          jsonConfig["bounds_path"],
          jsonConfig["track_path"],
          jsonConfig["normalization_path"]
      };
      Ts_ = jsonConfig["Ts"];
      v0_ = jsonConfig["v0"];

      // ========= MPC/모델 초기화 =========
      mpc_ = std::make_shared<MPC>(
          jsonConfig["n_sqp"],
          jsonConfig["n_reset"],
          jsonConfig["sqp_mixing"],
          Ts_, json_paths_);
      integrator_ = std::make_shared<Integrator>(Ts_, json_paths_);
      plotter_    = std::make_shared<Plotting>(Ts_, json_paths_);

      Track track(json_paths_.track_path);
      track_xy_ = track.getTrack();
      mpc_->setTrack(track_xy_.X, track_xy_.Y);

      // 스플라인 (s-wrap 및 corridor 시각화용)
      spline_ = ArcLengthSpline(json_paths_);
      spline_.gen2DSpline(track_xy_.X, track_xy_.Y);

      // ========= 상태 초기화 =========
      latest_state_ = {
        x0_vec[0], x0_vec[1], x0_vec[2], x0_vec[3], x0_vec[4],
        x0_vec[5], x0_vec[6], x0_vec[7], x0_vec[8], x0_vec[9]
      };
      next_state_   = latest_state_;

      duty_cycle_    = 0.0;
      steering_angle_ = 0.0;

      // ========= 서브/퍼브 =========
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/car_state/odom", 1, std::bind(&MPCCNode::odom_callback, this, _1));

      duty_cycle_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
          "/sensors/core", 1, std::bind(&MPCCNode::duty_cycle_callback, this, _1));

      servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
          "sensors/servo_position_command", 1, std::bind(&MPCCNode::servo_callback, this, _1));

      pred_traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "mpcc_predicted_traj", 1);

      track_center_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "track_center", 1);
      track_inner_pub_  = this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "track_inner", 1);
      track_outer_pub_  = this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "track_outer", 1);

      s_hint_pub_ = this->create_publisher<mpcc_ros::msg::SHint>("/mpcc/s_hint", 1);

      corridor_sub_ = this->create_subscription<mpcc_ros::msg::Corridor>(
          "/mpcc/corridor", 1, std::bind(&MPCCNode::corridor_callback, this, _1));

      corridor_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "corridor_markers/mpcc", 1);

      control_pub_ = this->create_publisher<mpcc_ros::msg::MpccControl>(
          "/mpcc/control", 1);

      // ========= 타이머 =========
      const double period = 1.0 / loop_rate_hz_;
      timer_ = this->create_wall_timer(
          std::chrono::duration<double>(period),
          std::bind(&MPCCNode::timer_callback, this));

      // ========= 트랙 시각화 =========
      publish_track_center(track_xy_.X, track_xy_.Y);
      publish_track_inner(track_xy_.X_inner, track_xy_.Y_inner);
      publish_track_outer(track_xy_.X_outer, track_xy_.Y_outer);

      // 종료시 플롯
      rclcpp::on_shutdown([this]()
      {
        RCLCPP_INFO(this->get_logger(), "Node shutting down, plotting results...");
        plotter_->plotRun(log_, track_xy_);
      });
  }

private:

  inline std::vector<double> wrap_s_vec(const std::vector<double>& S) const {
    std::vector<double> out;
    out.reserve(S.size());
    const double L = spline_.getLength();
    // std::cout << "Spline length: " << L << std::endl;
    for (double v : S) {
      double w = std::fmod(v, L);
      if (w < 0.0) w += L;
      out.push_back(w);
    }
    return out;
  }
  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double curr_x = msg->pose.pose.position.x;
    double curr_y = msg->pose.pose.position.y;

    tf2::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double curr_roll, curr_pitch, curr_yaw;
    tf2::Matrix3x3(quat).getRPY(curr_roll, curr_pitch, curr_yaw);
    

    double curr_vx = (msg->twist.twist.linear.x <= v0_) ? v0_ : msg->twist.twist.linear.x;
    double curr_vy = msg->twist.twist.linear.y;
    double omega = msg->twist.twist.angular.z;

    latest_state_ = {curr_x, curr_y, curr_yaw, curr_vx, curr_vy, omega, next_state_.s, duty_cycle_, steering_angle_, next_state_.vs};

  }

  void duty_cycle_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg) {
    duty_cycle_ = msg->state.duty_cycle;

    // std::cout << "duty cycle: " << duty_cycle_ << std::endl;
  }

  void servo_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle (radians) + steering_angle_to_servo_offset
    // steering angle = (servo value - steering_angle_to_servo_offset) / steering_angle_to_servo_gain

    double servo = msg->data;
    steering_angle_ = (servo - steering_angle_to_servo_offset_) / steering_angle_to_servo_gain_;

  }


  void timer_callback() {
    // 1) 초기추정 s 시퀀스 생성 & SHint 발행
    last_s_hint_ = mpc_->prepareInitialGuess(latest_state_);
    const auto s_wrapped = wrap_s_vec(last_s_hint_);

    {
      mpcc_ros::msg::SHint hint;
      hint.s = s_wrapped;
      hint.x = latest_state_.X;
      hint.y = latest_state_.Y;
      s_hint_pub_->publish(hint);
    }

    // 2) Corridor 사용 여부 판단
    const double diff = std::fabs(last_corridor_.init_s - latest_state_.s);

    MPCReturn mpc_sol;
    if (corridor_available_ && diff < corridor_s_threshold_) {
      mpc_sol = mpc_->runMPCWithCorridor(latest_state_, last_corridor_);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Using Corridor (diff=%.3f < %.3f).", diff, corridor_s_threshold_);
    } else {
      mpc_sol = mpc_->runMPC(latest_state_);
      if (corridor_available_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Corridor available but diff=%.3f >= %.3f, fallback to default MPC.",
          diff, corridor_s_threshold_);
      }
    }

    next_state_ = integrator_->simTimeStep(latest_state_, mpc_sol.u0, Ts_);

    pub_control_drive(next_state_, mpc_sol.solver_status);
    pub_pred_traj(mpc_sol.mpc_horizon);

    log_.push_back(mpc_sol);
  }

  void corridor_callback(const mpcc_ros::msg::Corridor &msg) {
    last_corridor_.d_left  = std::vector<double>(msg.d_left.begin(),  msg.d_left.end());
    last_corridor_.d_right = std::vector<double>(msg.d_right.begin(), msg.d_right.end());
    last_corridor_.init_s  = msg.init_s;

    // 1) s_hint를 발행한 적 있는가?  2) 크기 충분한가?
    corridor_available_ = (!last_s_hint_.empty()) && last_corridor_.d_left.size()  >= last_s_hint_.size() && last_corridor_.d_right.size() >= last_s_hint_.size();

    publish_corridor_markers();
  }

  void publish_corridor_markers() {
    if (last_s_hint_.empty()) return;
    const size_t n = last_s_hint_.size();
    const double L = spline_.getLength();
    auto wrapS = [&](double s){
      if (L <= 0.0) return s;
      s = std::fmod(s, L);
      if (s < 0.0) s += L;
      return s;
    };

    visualization_msgs::msg::MarkerArray arr;
    const rclcpp::Time now = this->now();

    auto makeLine = [&](int id, const char* ns, double r, double g, double b){
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now;
      m.ns = ns; m.id = id;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.10;
      m.color.a = 1.0; m.color.r = r; m.color.g = g; m.color.b = b;
      m.points.reserve(n);
      return m;
    };
    auto left  = makeLine(0, "corridor_left_xy",  0.10, 0.80, 0.10);
    auto right = makeLine(1, "corridor_right_xy", 0.90, 0.20, 0.20);

    for (size_t i=0; i<n; ++i) {
      const double si = wrapS(last_s_hint_[i]);
      Eigen::Vector2d pC = spline_.getPostion(si);
      Eigen::Vector2d dC = spline_.getDerivative(si);
      Eigen::Vector2d nC(-dC(1), dC(0));
      if (nC.norm() > 1e-9) nC /= nC.norm();

      const double dl = last_corridor_.d_left[i];
      const double dr = last_corridor_.d_right[i];
      Eigen::Vector2d pL = pC + dl * nC;
      Eigen::Vector2d pR = pC - dr * nC;

      geometry_msgs::msg::Point gl, gr;
      gl.x = pL(0); gl.y = pL(1); gl.z = 0.0;
      gr.x = pR(0); gr.y = pR(1); gr.z = 0.0;
      left.points.push_back(gl);
      right.points.push_back(gr);
    }
    arr.markers.push_back(left);
    arr.markers.push_back(right);
    corridor_marker_pub_->publish(arr);
  }

  void pub_control_drive(const State x1, const int status)
  {
    mpcc_ros::msg::MpccControl ctrl;
    ctrl.duty_cycle = x1.D;
    ctrl.servo = steering_angle_to_servo_gain_ * x1.delta + steering_angle_to_servo_offset_;
    ctrl.solver_status = status;

    control_pub_->publish(ctrl);
  }

  void pub_pred_traj(const std::array<OptVariables,N+1> mpc_horizon)
  {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "map";
    // marker.header.frame_id = "sim";
    marker.header.stamp = this->now();
    marker.ns = "mpcc_pred_traj";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS; 
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.0; 
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    for (const auto& opt : mpc_horizon)
    {
        geometry_msgs::msg::Point pt;
        pt.x = opt.xk.X;
        pt.y = opt.xk.Y;
        pt.z = 0.0;
        marker.points.push_back(pt);
    }

    marker_array.markers.push_back(marker);
    pred_traj_pub_->publish(marker_array);
  }

  void publish_track_center(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "track_center";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;


    for (int i = 0; i < X.size(); ++i) {
        geometry_msgs::msg::Point pt;
        pt.x = X[i];
        pt.y = Y[i];
        pt.z = 0.0;
        marker.points.push_back(pt);
    }

    marker_array.markers.push_back(marker);
    track_center_pub_->publish(marker_array);
  };

  void publish_track_inner(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "track_inner";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    for (int i = 0; i < X.size(); ++i) {
        geometry_msgs::msg::Point pt;
        pt.x = X[i];
        pt.y = Y[i];
        pt.z = 0.0;
        marker.points.push_back(pt);
    }

    marker_array.markers.push_back(marker);
    track_inner_pub_->publish(marker_array);
  };

  void publish_track_outer(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "track_outer";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;


    for (int i = 0; i < X.size(); ++i) {
        geometry_msgs::msg::Point pt;
        pt.x = X[i];
        pt.y = Y[i];
        pt.z = 0.0;
        marker.points.push_back(pt);
    }

    marker_array.markers.push_back(marker);
    track_outer_pub_->publish(marker_array);
  };

   

  double duty_cycle_, steering_angle_;
  double Ts_, v0_;

  // 파라미터
  std::string config_file_;
  double loop_rate_hz_{40.0};
  double corridor_s_threshold_{0.5};
  double steering_angle_to_servo_gain_{-1.0};
  double steering_angle_to_servo_offset_{0.5};

  State latest_state_, next_state_;

  TrackPos track_xy_;
  PathToJson json_paths_;
  ArcLengthSpline spline_;

  std::vector<double> last_s_hint_;
  CorridorData last_corridor_;
  bool corridor_available_ = false;

  std::shared_ptr<MPC>        mpc_;
  std::shared_ptr<Integrator> integrator_;
  std::shared_ptr<Plotting>   plotter_;

  std::list<MPCReturn> log_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr              odom_sub_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr     duty_cycle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr               servo_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr    pred_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr    track_center_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr    track_inner_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr    track_outer_pub_;

  rclcpp::Publisher<mpcc_ros::msg::SHint>::SharedPtr             s_hint_pub_;
  rclcpp::Subscription<mpcc_ros::msg::Corridor>::SharedPtr       corridor_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr    corridor_marker_pub_;

  rclcpp::Publisher<mpcc_ros::msg::MpccControl>::SharedPtr       control_pub_;

  rclcpp::TimerBase::SharedPtr timer_;


  
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCCNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
