#include <rclcpp/rclcpp.hpp>
#include "mpcc_ros/msg/s_hint.hpp"
#include "mpcc_ros/msg/corridor.hpp"

class CorridorGenerator : public rclcpp::Node {
public:
  CorridorGenerator()
  : Node("dummy_corridor_generator")
  {
    using std::placeholders::_1;

    pub_ = this->create_publisher<mpcc_ros::msg::Corridor>("/mpcc/corridor", 10);

    sub_ = this->create_subscription<mpcc_ros::msg::SHint>(
      "/mpcc/s_hint", rclcpp::QoS(10),
      std::bind(&CorridorGenerator::onSHint, this, _1));

    RCLCPP_INFO(this->get_logger(),
      "corridor_generator node started (s-based region widths: 20~20.5 & 27~27.5).");
  }

private:
  void onSHint(const mpcc_ros::msg::SHint & msg)
  {
    const auto & s = msg.s;
    if (s.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Received empty s[] from /mpcc/s_hint. Skipping corridor publish.");
      return;
    }

    const std::size_t n = s.size();
    mpcc_ros::msg::Corridor corridor;
    corridor.init_s = s.front();

    corridor.d_left.resize(n);
    corridor.d_right.resize(n);

    for (std::size_t i = 0; i < n; ++i) {
      double si = s[i];

      // 기본값
      double dL = 1.0;
      double dR = 1.0;

      // 특정 구간 조건 설정
      if (si >= 27.0 && si <= 34.0) {
        dL = 1.0;
        dR = -0.5;
      } 
      // else if (si >= 34.0) {
      //   dL = 1.0;
      //   dR = -0.25;
      // }

      corridor.d_left[i]  = dL;
      corridor.d_right[i] = dR;
    }

    pub_->publish(corridor);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Published Corridor: init_s=%.3f, len=%zu", corridor.init_s, n);
  }

  // ROS2 objects
  rclcpp::Subscription<mpcc_ros::msg::SHint>::SharedPtr sub_;
  rclcpp::Publisher<mpcc_ros::msg::Corridor>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CorridorGenerator>());
  rclcpp::shutdown();
  return 0;
}
