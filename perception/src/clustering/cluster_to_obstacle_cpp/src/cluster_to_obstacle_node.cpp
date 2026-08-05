#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   

#include <frenet_conversion_cpp/frenet_converter_cpp.hpp>  

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>

using std::placeholders::_1;

class ClusterToObstacle : public rclcpp::Node {
public:
  ClusterToObstacle()
  : rclcpp::Node("cluster_to_obstacle"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {

    // ---------- Parameters ----------
    this->declare_parameter<double>("boundaries_inflation", 0.1);
    frenet_waypoints_topic_ = declare_parameter<std::string>("frenet_waypoints_topic", "/global_waypoints");
    cluster_topic_          = declare_parameter<std::string>("cluster_topic", "/clusters");
    obstacle_pub_topic_     = declare_parameter<std::string>("obstacle_pub_topic", "/perception/detection/raw_obstacles");
    obstacle_marker_topic_  = declare_parameter<std::string>("obstacle_marker_topic", "/perception/detection/obstacles_markers");

    min_obs_size_           = declare_parameter<double>("min_obs_size", 0.05);
    max_obs_size_           = declare_parameter<double>("max_obs_size", 0.5);
    max_viewing_distance_   = declare_parameter<double>("max_viewing_distance", 5.0); //최대 장애물 탐지거리, 5m
    boundaries_inflation_   = this->get_parameter("boundaries_inflation").as_double();


    input_frame_            = declare_parameter<std::string>("input_frame", "livox_frame");
    map_frame_              = declare_parameter<std::string>("map_frame", "map");

    // ---------- Publishers ----------
    obstacles_pub_ = create_publisher<f110_msgs::msg::ObstacleArray>(obstacle_pub_topic_, 10);
    boundary_mrk_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/perception/detect_bound", rclcpp::QoS(1).transient_local()); // latch
    obstacle_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(obstacle_marker_topic_, 10);

    // ---------- Subscribers ----------
    wp_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
      frenet_waypoints_topic_, 10, std::bind(&ClusterToObstacle::pathCb, this, _1));

    car_state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/car_state/frenet/odom", 10, std::bind(&ClusterToObstacle::carStateCb, this, _1));

    cluster_sub_ = create_subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
      cluster_topic_, 10, std::bind(&ClusterToObstacle::clusterCb, this, _1));

    RCLCPP_INFO(get_logger(), "[cluster_to_obstacle] node initialized (using FrenetConverter C++)");
  }

private:
  // ---------- Waypoints / Frenet setup ----------
  void pathCb(const f110_msgs::msg::WpntArray::SharedPtr msg) {
    const auto& wps = msg->wpnts;
    if (wps.size() < 2) {
      RCLCPP_WARN(get_logger(), "Received too few waypoints: %zu", wps.size());
      return;
    }

    std::vector<double> xs; xs.reserve(wps.size());
    std::vector<double> ys; ys.reserve(wps.size());
    std::vector<double> psis; psis.reserve(wps.size());
    s_array_.clear(); d_right_array_.clear(); d_left_array_.clear();
    s_array_.reserve(wps.size());
    d_right_array_.reserve(wps.size());
    d_left_array_.reserve(wps.size());

    for (const auto &w : wps) {
      xs.push_back(w.x_m); ys.push_back(w.y_m); psis.push_back(w.psi_rad);
      s_array_.push_back(w.s_m);
      d_right_array_.push_back(w.d_right - boundaries_inflation_);
      d_left_array_.push_back(w.d_left  - boundaries_inflation_);
    }

    try {
      fr_converter_ = std::make_unique<FrenetConverter>(xs, ys, psis);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "FrenetConverter init failed: %s", e.what());
      return;
    }

    // smallest/biggest d 계산
    smallest_d_ = std::numeric_limits<double>::infinity();
    biggest_d_  = 0.0;
    for (size_t i=0; i<d_right_array_.size(); ++i) {
      smallest_d_ = std::min(smallest_d_, std::min(d_right_array_[i], d_left_array_[i]));
      biggest_d_  = std::max(biggest_d_,  std::max(d_right_array_[i], d_left_array_[i]));
    }

    track_length_ = wps.back().s_m;

    // 경계 마커 생성 (SPHERE_LIST)
    std::vector<geometry_msgs::msg::Point> boundary_pts;
    boundary_pts.reserve(2 * wps.size());
    for (const auto &w : wps) {
      // 오른쪽 경계점 (d: -d_right + infl)
      auto pR = getCartesianChecked(w.s_m, -(w.d_right - boundaries_inflation_));
      geometry_msgs::msg::Point pr_msg; pr_msg.x = pR.first; pr_msg.y = pR.second; pr_msg.z = 0.0;
      boundary_pts.push_back(pr_msg);

      // 왼쪽 경계점 (d: +d_left - infl)
      auto pL = getCartesianChecked(w.s_m,  (w.d_left  - boundaries_inflation_));
      geometry_msgs::msg::Point pl_msg; pl_msg.x = pL.first; pl_msg.y = pL.second; pl_msg.z = 0.0;
      boundary_pts.push_back(pl_msg);
    }

    detection_boundaries_marker_.header.frame_id = map_frame_;
    detection_boundaries_marker_.header.stamp = now();
    detection_boundaries_marker_.ns = "detect_bound";
    detection_boundaries_marker_.id = 0;
    detection_boundaries_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    detection_boundaries_marker_.action = visualization_msgs::msg::Marker::ADD;
    detection_boundaries_marker_.scale.x = 0.02;
    detection_boundaries_marker_.scale.y = 0.02;
    detection_boundaries_marker_.scale.z = 0.02;
    detection_boundaries_marker_.color.a = 1.0;
    detection_boundaries_marker_.color.r = 1.0;
    detection_boundaries_marker_.color.g = 0.0;
    detection_boundaries_marker_.color.b = 0.0;
    detection_boundaries_marker_.points = boundary_pts;

    boundary_mrk_pub_->publish(detection_boundaries_marker_);
    // RCLCPP_INFO(get_logger(), "Global path received & boundaries published. track_length=%.3f", track_length_);
  }

  // ---------- Car s callback ----------
  void carStateCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    car_s_ = msg->pose.pose.position.x;  // frenet/odom: pos.x = s
  }

  // ---------- Cluster callback ----------
  void clusterCb(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::SharedPtr msg) {
    if (!fr_converter_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "FrenetConverter not ready; dropping clusters");
      return;
    }
    if (s_array_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Track boundaries not ready; dropping clusters");
      return;
    }

    f110_msgs::msg::ObstacleArray out_array;
    out_array.header.stamp = now();
    out_array.header.frame_id = map_frame_;

    visualization_msgs::msg::MarkerArray marker_array;
    size_t published = 0;

    // 1) 입력 프레임 -> map 프레임 변환 + size 추출
    struct VObj { double x,y,size; };
    std::vector<VObj> valids; valids.reserve(msg->feature_objects.size());

    for (const auto &fo : msg->feature_objects) {
      geometry_msgs::msg::PointStamped in_pt, map_pt;
      in_pt.header.frame_id = input_frame_;


      // 기존 민규 코드
      // in_pt.header.stamp = now();
      // in_pt.point.x = fo.object.kinematics.pose_with_covariance.pose.position.x;
      // in_pt.point.y = fo.object.kinematics.pose_with_covariance.pose.position.y;
      // in_pt.point.z = fo.object.kinematics.pose_with_covariance.pose.position.z;

      // try {
      //   map_pt = tf_buffer_.transform(in_pt, map_frame_, tf2::durationFromSec(0.05));
      // } catch (const std::exception &e) {
      //   RCLCPP_DEBUG(get_logger(), "TF transform failed: %s", e.what());
      //   continue;
      // }

      
      // 1102.23.20. 준성 수정 코드
      // 1. 기존: now() 일 때의 tf를 사용하여 변환 -> 변경: msg->header.stamp 일 때의 tf를 사용하여 변환
      // 2. 기존: tf_buffer_.transform 실패 시 경고 메시지만 출력 -> 변경: tf_buffer_.transform 실패 시 가장 최신 tf라도 일단 활용

      if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) 
      {
        in_pt.header.stamp = msg->header.stamp; // 라이다 데이터의 타임스탬프 기준 변환을 시도
      } else {
        in_pt.header.stamp = now();  // 최후 fallback (가능하면 지양)
      }

      in_pt.point.x = fo.object.kinematics.pose_with_covariance.pose.position.x;
      in_pt.point.y = fo.object.kinematics.pose_with_covariance.pose.position.y;
      in_pt.point.z = fo.object.kinematics.pose_with_covariance.pose.position.z;

      try {
        // --- (A) 라이다 데이터의 타임스탬프 기준 변환 시도 ---
        map_pt = tf_buffer_.transform(in_pt, map_frame_, tf2::durationFromSec(0.05));

      } catch (const std::exception &e_ts) {
        // --- (B) 실패 시 최신 TF로 강제 변환 ---
        try {
          const auto tf_latest =
              tf_buffer_.lookupTransform(map_frame_, input_frame_, tf2::TimePointZero);
          tf2::doTransform(in_pt, map_pt, tf_latest);

          RCLCPP_DEBUG(get_logger(),
                      "Timestamped TF missing (%s). Fallback to latest TF succeeded.",
                      e_ts.what());
        } catch (const std::exception &e_latest) {
          RCLCPP_DEBUG(get_logger(),
                      "Both timestamped and latest TF failed: %s", e_latest.what());
          continue;
        }
      }


      double size = 0.0;
      try {
        const auto &dim = fo.object.shape.dimensions;
        size = std::max(dim.x, dim.y);
      } catch (...) {
        RCLCPP_WARN(get_logger(), "Detected object missing shape.dimensions; size=0");
      }

      valids.push_back({map_pt.point.x, map_pt.point.y, size});
    }

    // 2) 프레네 변환 + 트랙 필터
    if (!valids.empty()) {
      std::vector<double> xs, ys; xs.reserve(valids.size()); ys.reserve(valids.size());
      for (auto &v : valids) { xs.push_back(v.x); ys.push_back(v.y); }

      std::vector<double> s0_guess;  //
      auto sd_pair = fr_converter_->get_frenet(xs, ys, nullptr);  //
      const std::vector<double>& s_out = sd_pair.first;
      const std::vector<double>& d_out = sd_pair.second;

      for (size_t i=0; i<valids.size(); ++i) {
        const double s = s_out[i];
        const double d = d_out[i];
        const double size = valids[i].size;

        if (!laserPointOnTrack(s, d, car_s_)) continue;
      //   if ( size > max_obs_size_){ // size filtering
      //       RCLCPP_INFO(get_logger(),
      //         "BIGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG:");
      //     continue;
      // }
        f110_msgs::msg::Obstacle ob;
        ob.id = static_cast<int32_t>(published);
        ob.s_center = s;
        ob.d_center = d;
        ob.size = size;
        ob.s_start = s - size/2.0;
        ob.s_end   = s + size/2.0;
        ob.d_left  = d + size/2.0;
        ob.d_right = d - size/2.0;
        out_array.obstacles.push_back(ob);

        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame_;
        m.header.stamp = now();
        m.ns = "obstacles";
        m.id = static_cast<int>(published);
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = size;
        m.scale.y = size;
        m.scale.z = size;
        m.color.a = 0.5;
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        m.pose.position.x = valids[i].x;
        m.pose.position.y = valids[i].y;
        m.pose.position.z = 0.0;
        m.pose.orientation.w = 1.0;

        marker_array.markers.push_back(m);
        published++;
      }

      obstacle_marker_pub_->publish(marker_array);
    //   RCLCPP_INFO(get_logger(), "Published %zu obstacles after filtering", published);
    }

    // 3) 항상 배열 퍼블리시
    obstacles_pub_->publish(out_array);

    // 4) 경계 마커 재퍼블리시(래치 유지)
    boundary_mrk_pub_->publish(detection_boundaries_marker_);
  }

  // ---------- Helpers ----------
  inline double wrap_s(double x) const {
    if (track_length_ <= 0.0) return x;
    double m = std::fmod(x, track_length_);
    if (m >  track_length_/2.0) m -= track_length_;
    if (m < -track_length_/2.0) m += track_length_;
    return m;
  }

  bool laserPointOnTrack(double s, double d, double car_s) const {
    if (wrap_s(s - car_s) > max_viewing_distance_) return false;
    if (std::fabs(d) >= biggest_d_) return false;
    if (std::fabs(d) <= smallest_d_) return true;

    // s 구간 인덱스 (bisect_left)
    size_t idx = 0;
    auto it = std::upper_bound(s_array_.begin(), s_array_.end(), s);
    if (it == s_array_.begin()) idx = 0;
    else idx = static_cast<size_t>(std::distance(s_array_.begin(), it) - 1);
    if (idx >= d_right_array_.size()) idx = d_right_array_.size() - 1;

    if (d <= -d_right_array_[idx] || d >= d_left_array_[idx]) return false;
    return true;
  }

  std::pair<double,double> getCartesianChecked(double s, double d) {
    try {
      auto xy = fr_converter_->get_cartesian(s, d); // (x,y) 반환 시그니처일 때
      return xy;
    } catch (...) {
      // 일부 구현에서 (x[], y[]) 벡터 반환일 수 있으므로 대비
      auto xyv = fr_converter_->get_cartesian(std::vector<double>{s}, std::vector<double>{d});
      return {xyv.first[0], xyv.second[0]};
    }
  }

private:
  // Params
  std::string frenet_waypoints_topic_, cluster_topic_, obstacle_pub_topic_, obstacle_marker_topic_;
  double min_obs_size_{0.05}, max_obs_size_{0.5}, max_viewing_distance_{5.0}, boundaries_inflation_{0.0};
  std::string input_frame_{"livox_frame"}, map_frame_{"map"};

  // Pubs/Subs
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr wp_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr car_state_sub_;
  rclcpp::Subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr cluster_sub_;
  rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boundary_mrk_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Frenet
  std::unique_ptr<FrenetConverter> fr_converter_;
  std::vector<double> s_array_, d_right_array_, d_left_array_;
  double smallest_d_{0.0}, biggest_d_{0.0}, track_length_{0.0};
  double car_s_{0.0};

  // Marker (latched)
  visualization_msgs::msg::Marker detection_boundaries_marker_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClusterToObstacle>());
  rclcpp::shutdown();
  return 0;
}
