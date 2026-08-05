#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <algorithm>

class Pc2ToLaserScan : public rclcpp::Node {
public:
  Pc2ToLaserScan()
  : rclcpp::Node("pc2_to_laserscan")
  {
    this->declare_parameter<double>("range_min", 0.1);
    this->declare_parameter<double>("range_max", 7.0);
    this->declare_parameter<double>("angle_increment_deg", 0.25);

    // ---- Parameters (same defaults as Python) ----
    // pc_topic_  = declare_parameter<std::string>("pc_topic", "/passthrough/lidar");
    pc_topic_  = declare_parameter<std::string>("pc_topic", "/ground_segmentation/lidar");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    angle_min_ = declare_parameter<double>("angle_min", -6.0 * M_PI / 4.0);
    angle_max_ = declare_parameter<double>("angle_max",  1.0 * M_PI / 4.0);
    const double angle_inc_deg = this->get_parameter("angle_increment_deg").as_double();
    angle_increment_ = angle_inc_deg * M_PI / 180.0;
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    use_closest_point_ = declare_parameter<bool>("use_closest_point", true);

    // ---- Derived ----
    num_bins_ = static_cast<int>(std::round((angle_max_ - angle_min_) / angle_increment_));
    if (num_bins_ <= 0) {
      RCLCPP_FATAL(get_logger(), "Invalid angle range/increment. (bins=%d)", num_bins_);
      throw std::runtime_error("Invalid angle range");
    }

    // ---- QoS: sensor data, best effort ----
    auto sensor_qos = rclcpp::SensorDataQoS().reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // ---- Pub/Sub ----
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, sensor_qos);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pc_topic_, sensor_qos,
      std::bind(&Pc2ToLaserScan::pcCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "pc2->scan node ready, bins=%d", num_bins_);
  }

private:
  void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // ranges init to range_max
    std::vector<float> ranges(num_bins_, static_cast<float>(range_max_));

    // Iterate over XYZ (skip NaNs is automatic with iterator validity checks)
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");  // z 읽지만 사용X

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      // const float z = *iter_z;

      // 필드에 NaN이 있을 수 있으니 방어
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }

      const double r = std::hypot(x, y);
      if (r < range_min_ || r > range_max_) {
        continue;
      }

      const double angle = std::atan2(y, x);   // -pi..pi
      const double idx_f = (angle - angle_min_) / angle_increment_;
      const int idx = static_cast<int>(std::round(idx_f));

      if (idx < 0 || idx >= num_bins_) {
        continue;
      }

      if (use_closest_point_) {
        if (r < ranges[idx]) ranges[idx] = static_cast<float>(r);
      } else {
        ranges[idx] = std::min(ranges[idx], static_cast<float>(r));
      }
    }

    // Compose LaserScan
    sensor_msgs::msg::LaserScan scan;
    scan.header = msg->header;
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(angle_max_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.time_increment = 0.0f;
    scan.scan_time = 0.1f;   // same as Python
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges = ranges;

    pub_->publish(scan);
  }

  // params
  std::string pc_topic_;
  std::string scan_topic_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double range_min_;
  double range_max_;
  bool use_closest_point_;
  int num_bins_;

  // ros
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pc2ToLaserScan>());
  rclcpp::shutdown();
  return 0;
}
