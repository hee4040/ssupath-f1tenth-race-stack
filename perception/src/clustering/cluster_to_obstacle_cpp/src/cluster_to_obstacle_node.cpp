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
    // ★ 2026-08-12: boundaries_inflation 을 없애고 min_intrusion 하나로 합쳤다.
    //   두 값은 이 필터 안에서 수학적으로 완전히 동일했다 —
    //     예전: 경계를 infl 만큼 안으로 줄이고(d_left - infl), 거기서 다시 min_intrusion 을 뺌
    //     즉 실제 문턱은 언제나 (infl + min_intrusion) 하나였고, 따로 둘 이유가 없었다.
    //   게다가 마커(/perception/detect_bound)는 infl 만 반영해서 그렸기 때문에,
    //   rviz 의 빨간 점이 '탐지 시작선'이 아니라 거기서 min_intrusion 만큼 더 들어가야
    //   하는 선이었다. 보이는 것과 실제 판정이 30 cm 어긋나 있었다는 뜻이다.
    //   이제 마커도 min_intrusion 을 반영해 그리므로 빨간 점 = 진짜 문턱선이다.
    //   합친 값 0.37 = 예전 boundaries_inflation 0.07 + min_intrusion 0.30 이라
    //   탐지 동작은 완전히 동일하다(마커 위치만 안쪽으로 30 cm 이동).
    // 트랙 경계(d_left/d_right)를 어디서 받을지.
    // 기본값은 원본 맵(<map>_origin.png) 기준으로 보정된 경계다. 편집 맵은 레이스라인이
    // 새지 않도록 알코브를 메워 놓기 때문에 그 구간 경계가 실제 벽보다 안쪽이고,
    // 그러면 거기 놓인 진짜 장애물이 "트랙 밖"으로 버려진다. 판정 기준은 실제 벽이어야 한다.
    // (발행: global_trajectory_publisher / 계산: global_planner/origin_map_bounds.py)
    // s/d 프레네 기준선 자체는 /global_waypoints 와 완전히 동일하므로 하류 좌표는 안 바뀐다.
    frenet_waypoints_topic_ = declare_parameter<std::string>("frenet_waypoints_topic",
                                                             "/global_waypoints/original_bounds");
    // 위 토픽이 안 뜰 때(구버전 global_trajectory_publisher 등) 쓰는 대비책.
    // 이게 없으면 경계를 못 받아 클러스터를 전부 버린다 = 장애물 탐지가 통째로 죽는다.
    fallback_waypoints_topic_ = declare_parameter<std::string>("fallback_waypoints_topic",
                                                               "/global_waypoints");
    // 폴백을 받아들이기 전에 원본 경계를 기다려 주는 시간 [s].
    // global_trajectory_publisher 는 두 토픽을 같은 주기(5 s)에 연달아 발행하므로
    // 이 유예가 없으면 매 기동마다 폴백 경고가 한 번씩 헛되이 뜬다.
    fallback_grace_period_ = declare_parameter<double>("fallback_grace_period", 8.0);
    node_start_ = now();
    cluster_topic_          = declare_parameter<std::string>("cluster_topic", "/clusters");
    obstacle_pub_topic_     = declare_parameter<std::string>("obstacle_pub_topic", "/perception/detection/raw_obstacles");
    obstacle_marker_topic_  = declare_parameter<std::string>("obstacle_marker_topic", "/perception/detection/obstacles_markers");

    min_obs_size_           = declare_parameter<double>("min_obs_size", 0.05);
    max_obs_size_           = declare_parameter<double>("max_obs_size", 0.5);
    max_viewing_distance_   = declare_parameter<double>("max_viewing_distance", 5.0); //최대 장애물 탐지거리, 5m
    // 장애물의 '안쪽 끝'이 트랙 경계(d_left/d_right)보다 이만큼 안으로 들어와야 인정 [m].
    // 예전 boundaries_inflation 의 역할까지 흡수한 단일 문턱이다.
    min_intrusion_          = declare_parameter<double>("min_intrusion", 0.0);


    input_frame_            = declare_parameter<std::string>("input_frame", "livox_frame");
    map_frame_              = declare_parameter<std::string>("map_frame", "map");

    // ---------- Publishers ----------
    obstacles_pub_ = create_publisher<f110_msgs::msg::ObstacleArray>(obstacle_pub_topic_, 10);
    boundary_mrk_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/perception/detect_bound", rclcpp::QoS(1).transient_local()); // latch
    obstacle_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(obstacle_marker_topic_, 10);

    // ---------- Subscribers ----------
    wp_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
      frenet_waypoints_topic_, 10,
      [this](const f110_msgs::msg::WpntArray::SharedPtr msg) { pathCb(msg, true); });

    if (!fallback_waypoints_topic_.empty() && fallback_waypoints_topic_ != frenet_waypoints_topic_) {
      wp_fallback_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
        fallback_waypoints_topic_, 10,
        [this](const f110_msgs::msg::WpntArray::SharedPtr msg) { pathCb(msg, false); });
    }

    car_state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/car_state/frenet/odom", 10, std::bind(&ClusterToObstacle::carStateCb, this, _1));

    cluster_sub_ = create_subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
      cluster_topic_, 10, std::bind(&ClusterToObstacle::clusterCb, this, _1));

    RCLCPP_INFO(get_logger(), "[cluster_to_obstacle] node initialized (using FrenetConverter C++)");
  }

private:
  // ---------- Waypoints / Frenet setup ----------
  // is_primary=false 는 폴백 토픽(/global_waypoints, 편집 맵 경계)에서 온 것이다.
  // 원본 맵 경계를 한 번이라도 받았으면 폴백은 무시한다.
  void pathCb(const f110_msgs::msg::WpntArray::SharedPtr msg, bool is_primary) {
    if (!is_primary) {
      if (got_primary_bounds_) return;
      // 원본 경계가 올 시간을 준다. 아직 유예 중이면 폴백을 무시한다.
      if ((now() - node_start_).seconds() < fallback_grace_period_) return;
    }

    const auto& wps = msg->wpnts;
    if (wps.size() < 2) {
      RCLCPP_WARN(get_logger(), "Received too few waypoints: %zu", wps.size());
      return;
    }

    if (is_primary && !got_primary_bounds_) {
      got_primary_bounds_ = true;
      RCLCPP_INFO(get_logger(), "[cluster_to_obstacle] track bounds from '%s' (원본 맵 기준)",
                  frenet_waypoints_topic_.c_str());
    } else if (!is_primary) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "[cluster_to_obstacle] '%s' 미수신 -> 폴백 '%s' (편집 맵 경계) 사용 중",
                           frenet_waypoints_topic_.c_str(), fallback_waypoints_topic_.c_str());
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
      // 문턱을 여기서 한 번에 반영해 둔다. 이 배열이 곧 '탐지 시작선'이고,
      // 아래 마커도 laserPointOnTrack 도 같은 값을 쓴다(보이는 것 = 판정하는 것).
      d_right_array_.push_back(w.d_right - min_intrusion_);
      d_left_array_.push_back(w.d_left  - min_intrusion_);
    }

    try {
      fr_converter_ = std::make_unique<FrenetConverter>(xs, ys, psis);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "FrenetConverter init failed: %s", e.what());
      return;
    }

    // ★ 조기 판정용 전역 최소/최대는 '문턱 적용 전' 원래 경계로 구한다.
    //   d_*_array_ 에는 min_intrusion_ 이 이미 빠져 있는데, 그걸 그대로 쓰면
    //   아래 laserPointOnTrack 의 "트랙 최협부 안쪽이면 무조건 통과" 예외까지
    //   min_intrusion_ 만큼 좁아진다. 그 예외는 레이싱라인 정중앙(=차가 실제로
    //   지나가는 자리)에 놓인 작은 장애물을 살리는 안전장치라 유지해야 한다.
    //   2026-08-12 병합 때 이걸 놓쳐서, 백 재생 A/B 로 obs_debug_0811_1726 의
    //   |d|<0.10 짜리 클러스터 148개가 통째로 사라지는 걸 확인하고 되돌렸다.
    smallest_d_ = std::numeric_limits<double>::infinity();
    biggest_d_  = 0.0;
    for (const auto &w : wps) {
      smallest_d_ = std::min(smallest_d_, std::min(w.d_right, w.d_left));
      biggest_d_  = std::max(biggest_d_,  std::max(w.d_right, w.d_left));
    }

    track_length_ = wps.back().s_m;

    // 경계 마커 생성 (SPHERE_LIST)
    std::vector<geometry_msgs::msg::Point> boundary_pts;
    boundary_pts.reserve(2 * wps.size());
    for (const auto &w : wps) {
      // 오른쪽 문턱선 (d: -(d_right - min_intrusion))
      auto pR = getCartesianChecked(w.s_m, -(w.d_right - min_intrusion_));
      geometry_msgs::msg::Point pr_msg; pr_msg.x = pR.first; pr_msg.y = pR.second; pr_msg.z = 0.0;
      boundary_pts.push_back(pr_msg);

      // 왼쪽 문턱선 (d: +(d_left - min_intrusion))
      auto pL = getCartesianChecked(w.s_m,  (w.d_left  - min_intrusion_));
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

        if (!laserPointOnTrack(s, d, car_s_, size)) continue;
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
        m.header.stamp = now(); //msg->header.stamp; // 라이다 데이터의 타임스탬프 기준 변환을 시도
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

  // 장애물의 '가로 끝'까지 보고 트랙 안팎을 판정한다.
  // 예전에는 중심점 (s,d) 한 점만 검사해서, 벽에 붙은 장애물은 중심이 경계 밖이라는
  // 이유로 통째로 버려졌다(= raw_obstacles 에 아예 안 실림 -> 하류 전부 못 봄).
  // 이제 [d - size/2, d + size/2] 구간이 트랙 폭과 겹치는지를 본다.
  bool laserPointOnTrack(double s, double d, double car_s, double size = 0.0) const {
    const double half = std::max(0.0, size * 0.5);
    const double d_lo = d - half;   // 장애물 오른쪽 끝
    const double d_hi = d + half;   // 장애물 왼쪽 끝

    if (wrap_s(s - car_s) > max_viewing_distance_) return false;
    // 아래 둘은 '문턱 적용 전' 전역 경계 기준이다(pathCb 주석 참조).
    if (std::fabs(d) - half >= biggest_d_) return false;  // 트랙 최광부 밖 -> 조기 기각
    if (std::fabs(d) <= smallest_d_) return true;         // 트랙 최협부 안 -> 무조건 통과
                                                          //   (레이싱라인 정중앙 장애물 보호)

    // s 구간 인덱스 (bisect_left)
    size_t idx = 0;
    auto it = std::upper_bound(s_array_.begin(), s_array_.end(), s);
    if (it == s_array_.begin()) idx = 0;
    else idx = static_cast<size_t>(std::distance(s_array_.begin(), it) - 1);
    if (idx >= d_right_array_.size()) idx = d_right_array_.size() - 1;

    // d_*_array_ 에는 min_intrusion_ 이 이미 반영돼 있다(= 문턱선, 마커와 동일한 값).
    // 장애물의 '안쪽 끝'이 그 선을 넘어야 통과한다.
    //   왼쪽 끝  d_hi 가 오른쪽 문턱선보다 바깥이면 기각
    //   오른쪽 끝 d_lo 가 왼쪽 문턱선보다 바깥이면 기각
    if (d_hi <= -d_right_array_[idx]) return false;
    if (d_lo >=  d_left_array_[idx])  return false;
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
  std::string frenet_waypoints_topic_, fallback_waypoints_topic_;
  std::string cluster_topic_, obstacle_pub_topic_, obstacle_marker_topic_;
  bool got_primary_bounds_{false};
  double fallback_grace_period_{8.0};
  rclcpp::Time node_start_;
  double min_obs_size_{0.05}, max_obs_size_{0.5}, max_viewing_distance_{5.0};
  double min_intrusion_{0.0};   // 단일 문턱 (예전 boundaries_inflation + min_intrusion)
  std::string input_frame_{"livox_frame"}, map_frame_{"map"};

  // Pubs/Subs
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr wp_sub_, wp_fallback_sub_;
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
