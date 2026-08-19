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

#include "mpcc_ros/msg/s_hint.hpp"
#include "mpcc_ros/msg/corridor.hpp"

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

#include <chrono>
#include <iomanip>

using json = nlohmann::json;
using namespace mpcc;
using std::placeholders::_1;


class MPCCNode : public rclcpp::Node {
public:
  MPCCNode() : Node("mpcc_node") {

      // ---- 파라미터 ----
      // JSON 설정 파일 경로
      config_file_ = this->declare_parameter<std::string>(
          "config_file",
          "/home/misys/forza_ws/race_stack/controller/mpcc/MPCC/C++/Params/config_teras.json"
      );

      // 타이머 주기(Hz 단위)
      loop_rate_hz_ = this->declare_parameter<double>("loop_rate_hz", 40.0);

      // corridor 사용 조건 임계치
      corridor_s_threshold_ = this->declare_parameter<double>("corridor_s_threshold", 0.5);

      // x0 초기화
      std::vector<double> x0_vec_default = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      auto x0_vec = this->declare_parameter<std::vector<double>>("x0", x0_vec_default);
      if (x0_vec.size() != 10) {
        RCLCPP_WARN(this->get_logger(),
          "Parameter 'x0' size (%zu) != 10. Falling back to default.", x0_vec.size());
        x0_vec = x0_vec_default;
      }

      x0_ = {
        x0_vec[0], x0_vec[1], x0_vec[2], x0_vec[3], x0_vec[4],
        x0_vec[5], x0_vec[6], x0_vec[7], x0_vec[8], x0_vec[9]
      };

      // ---- JSON 로드 ----
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

      // ---- MPC/모델 초기화 ----
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

      spline_ = ArcLengthSpline(json_paths_);
      spline_.gen2DSpline(track_xy_.X, track_xy_.Y);


      // ---- 퍼블리셔 ----
      sim_pose_pub_  = this->create_publisher<geometry_msgs::msg::PoseStamped>("sim/pose", 1);
      pred_traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mpcc_predicted_traj", 1);

      track_center_pub_    = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_center", 1);
      track_inner_pub_     = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_inner", 1);
      track_outer_pub_     = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_outer", 1);
      corridor_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("corridor_markers/mpcc", 1);

      s_hint_pub_ = this->create_publisher<mpcc_ros::msg::SHint>("/mpcc/s_hint", 1);

      corridor_sub_ = this->create_subscription<mpcc_ros::msg::Corridor>(
          "/mpcc/corridor", 1,
          std::bind(&MPCCNode::corridor_callback, this, std::placeholders::_1));

      // ---- 타이머  ----
      const double period = 1.0 / loop_rate_hz_;
      timer_ = this->create_wall_timer(
          std::chrono::duration<double>(period),
          std::bind(&MPCCNode::timer_callback, this));

      // 시각화용 트랙 출판
      publish_track_center(track_xy_.X, track_xy_.Y);
      publish_track_inner(track_xy_.X_inner, track_xy_.Y_inner);
      publish_track_outer(track_xy_.X_outer, track_xy_.Y_outer);

      // 디버깅용 종료 시 플롯
      rclcpp::on_shutdown([this]() 
      {
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
  
  void timer_callback() {

    // 1) 초기추정 s 시퀀스 생성 & SHint 발행
    last_s_hint_ = mpc_->prepareInitialGuess(x0_);
    const auto s_wrapped = wrap_s_vec(last_s_hint_);

    {
      mpcc_ros::msg::SHint hint;
      hint.s = s_wrapped;
      hint.x = x0_.X;
      hint.y = x0_.Y;
      s_hint_pub_->publish(hint);
    }

    // 2) Corridor 사용 여부 판단
    const double diff = std::fabs(last_corridor_.init_s - x0_.s);
    MPCReturn sol;
    if(corridor_available_ && diff < corridor_s_threshold_) {
      // Corridor 기반으로 MPC 풀기
      sol = mpc_->runMPCWithCorridor(x0_, last_corridor_);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Using Corridor (diff=%.3f < %.3f).", diff, corridor_s_threshold_);
    } 
    else {
      // 일반 MPC
      sol = mpc_->runMPC(x0_);
      if(corridor_available_)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Corridor available but diff=%.3f >= %.3f, fallback to default MPC.", diff, corridor_s_threshold_);
    }

    // 3) 시뮬레이션 & 시각화
    x0_ = integrator_->simTimeStep(x0_, sol.u0, Ts_);
    pub_sim_pose(x0_);
    pub_pred_traj(sol.mpc_horizon);

    // 시각화용 트랙 출판
    publish_track_center(track_xy_.X, track_xy_.Y);
    publish_track_inner(track_xy_.X_inner, track_xy_.Y_inner);
    publish_track_outer(track_xy_.X_outer, track_xy_.Y_outer);

    log_.push_back(sol);
  }


  void corridor_callback(const mpcc_ros::msg::Corridor &msg) {
    last_corridor_.d_left  = std::vector<double>(msg.d_left.begin(),  msg.d_left.end());
    last_corridor_.d_right = std::vector<double>(msg.d_right.begin(), msg.d_right.end());
    last_corridor_.init_s  = msg.init_s;

    // 1. s_hint를 적어도 한 번은 먼저 발행해는지? 2. d_left 배열의 사이즈는 충분한지? 3. d_right 배열의 사이즈는 충분한지?  -> 3가지를 모두 만족하면 corridor_available
    corridor_available_ = (!last_s_hint_.empty()) && last_corridor_.d_left.size() >= last_s_hint_.size() && last_corridor_.d_right.size()>= last_s_hint_.size();

    publish_corridor_markers();
  }


  void pub_sim_pose(const State &state) {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.frame_id = "sim";
    pose_msg.header.stamp = this->now();

    pose_msg.pose.position.x = state.X;
    pose_msg.pose.position.y = state.Y;
    pose_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, state.phi);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    sim_pose_pub_->publish(pose_msg);
  }

  void pub_pred_traj(const std::array<OptVariables,N+1> mpc_horizon)
  {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "sim";
    marker.header.stamp = this->now();
    marker.ns = "mpcc_pred_traj";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS; 
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1; 
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

    marker.header.frame_id = "sim";
    marker.header.stamp = this->now();
    marker.ns = "track_center";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS; 
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05; 
    marker.color.a = 0.5;
    marker.color.r = 0.0;
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
    track_center_pub_->publish(marker_array);
  };

  void publish_track_inner(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "sim";
    marker.header.stamp = this->now();
    marker.ns = "track_inner_wall_surface";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    double height = 0.4;

    for (int i = 0; i < X.size() - 1; ++i) {
        geometry_msgs::msg::Point p1, p2, p1_top, p2_top;

        p1.x = X[i];
        p1.y = Y[i];
        p1.z = 0.0;

        p2.x = X[i + 1];
        p2.y = Y[i + 1];
        p2.z = 0.0;

        p1_top = p1;
        p1_top.z = height;

        p2_top = p2;
        p2_top.z = height;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p1_top);

        marker.points.push_back(p1_top);
        marker.points.push_back(p2);
        marker.points.push_back(p2_top);
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    track_inner_pub_->publish(marker_array);
  };

  void publish_track_outer(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "sim";
    marker.header.stamp = this->now();
    marker.ns = "track_outer_wall_surface";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    double height = 0.4;

    for (int i = 0; i < X.size() - 1; ++i) {
        geometry_msgs::msg::Point p1, p2, p1_top, p2_top;

        p1.x = X[i];
        p1.y = Y[i];
        p1.z = 0.0;

        p2.x = X[i + 1];
        p2.y = Y[i + 1];
        p2.z = 0.0;

        p1_top = p1;
        p1_top.z = height;

        p2_top = p2;
        p2_top.z = height;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p1_top);

        marker.points.push_back(p1_top);
        marker.points.push_back(p2);
        marker.points.push_back(p2_top);
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    track_outer_pub_->publish(marker_array);
  }

  void publish_corridor_markers() {
    const size_t n = last_s_hint_.size();

    const double L = spline_.getLength();
    auto wrapS = [&](double s) {
      if (L <= 0.0) return s;
      s = std::fmod(s, L);
      if (s < 0.0) s += L;
      return s;
    };

    visualization_msgs::msg::MarkerArray arr;
    const rclcpp::Time now = this->now();

    auto makeLine = [&](int id, const char* ns, double r, double g, double b) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "sim";
      m.header.stamp = now;
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.15;           // 선 두께
      m.color.a = 1.0; m.color.r = r; m.color.g = g; m.color.b = b;
      m.points.reserve(n);
      return m;
    };

    auto m_left  = makeLine(0, "corridor_left_xy",   0.10, 0.80, 0.10);
    auto m_right = makeLine(1, "corridor_right_xy",  0.90, 0.20, 0.20);


    for (size_t i = 0; i < n; ++i) {
      const double si = wrapS(last_s_hint_[i]);

      // 중심점/접선
      Eigen::Vector2d pC = spline_.getPostion(si);
      Eigen::Vector2d dC = spline_.getDerivative(si);

      // 법선(정규화)
      Eigen::Vector2d nC(-dC(1), dC(0));
      nC /= nC.norm();

      // 좌/우 경계
      const double dl = last_corridor_.d_left[i];
      const double dr = last_corridor_.d_right[i];

      Eigen::Vector2d pL = pC + dl * nC;  // left
      Eigen::Vector2d pR = pC - dr * nC;  // right

      geometry_msgs::msg::Point gl, gr;
      gl.x = pL(0); gl.y = pL(1); gl.z = 0.0;
      gr.x = pR(0); gr.y = pR(1); gr.z = 0.0;

      m_left.points.push_back(gl);
      m_right.points.push_back(gr);
    }


    arr.markers.push_back(m_left);
    arr.markers.push_back(m_right);

    corridor_marker_pub_->publish(arr);
  }

  double Ts_, v0_;

  State x0_;

  TrackPos track_xy_;
  PathToJson json_paths_;
  ArcLengthSpline spline_;

  std::vector<double> last_s_hint_;
  CorridorData last_corridor_;
  bool corridor_available_ = false;

  std::string config_file_;
  double loop_rate_hz_{40.0};          
  double corridor_s_threshold_{0.5}; 

  std::shared_ptr<MPC> mpc_;
  std::shared_ptr<Integrator> integrator_;
  std::shared_ptr<Plotting> plotter_;

  std::list<MPCReturn> log_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sim_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pred_traj_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_center_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_inner_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_outer_pub_;

  rclcpp::Publisher<mpcc_ros::msg::SHint>::SharedPtr s_hint_pub_;
  rclcpp::Subscription<mpcc_ros::msg::Corridor>::SharedPtr corridor_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_marker_pub_;



  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCCNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}