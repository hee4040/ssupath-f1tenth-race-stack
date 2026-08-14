// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/vesc_to_odom.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <cmath>
#include <string>

#include <algorithm>  // odom 공분산 수정을 위해 추가

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_node", options),
  x_(0.0),
  y_(0.0),
  yaw_(0.0)
{
  // declare default ROS parameters
  declare_parameter("odom_frame", "odom");
  declare_parameter("base_frame", "base_link");
  declare_parameter("use_servo_cmd_to_calc_angular_velocity", true);
  declare_parameter("speed_to_erpm_gain", 1.0);
  declare_parameter("speed_to_erpm_offset", 0.0);
  declare_parameter("steering_angle_to_servo_gain", 1.0);
  declare_parameter("steering_angle_to_servo_offset", 0.0);
  declare_parameter("wheelbase", 0.2);
  declare_parameter("publish_tf", true);
  declare_parameter("use_imu_angular_velocity", false);
  declare_parameter("imu_timeout", 0.2);


  // get ROS parameters
  get_parameter("odom_frame", odom_frame_);
  get_parameter("base_frame", base_frame_);
  get_parameter("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);

  get_parameter("speed_to_erpm_gain", speed_to_erpm_gain_);
  get_parameter("speed_to_erpm_offset", speed_to_erpm_offset_);

  if (use_servo_cmd_) {
    get_parameter("steering_angle_to_servo_gain", steering_to_servo_gain_);
    get_parameter("steering_angle_to_servo_offset", steering_to_servo_offset_);
    get_parameter("wheelbase", wheelbase_);
  }

  get_parameter("publish_tf", publish_tf_);
  get_parameter("use_imu_angular_velocity", use_imu_angular_velocity_);
  get_parameter("imu_timeout", imu_timeout_s_);

  if (use_servo_cmd_) {
    // 첫 서보 명령 전에도 odom/TF가 나가도록 중앙(조향각 0)으로 초기화.
    // 속도 0이라 적분에 영향 없고, scanmatcher use_odom이 시작부터 odom 프레임을 찾을 수 있음
    last_servo_cmd_ = std::make_shared<Float64>();
    last_servo_cmd_->data = steering_to_servo_offset_;
  }

  // create odom publisher
  odom_pub_ = create_publisher<Odometry>("odom", 1);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  // subscribe to vesc state and. optionally, servo command
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 1, std::bind(&VescToOdom::vescStateCallback, this, _1));

  if (use_servo_cmd_) {
    servo_sub_ = create_subscription<Float64>(
      "sensors/servo_position_command", 1, std::bind(&VescToOdom::servoCmdCallback, this, _1));
  }

  if (use_imu_angular_velocity_) {
    // base_link->imu 정적 TF는 z축 90도 회전뿐이라 각속도 z 성분은 그대로 base_link 값이다
    // (Rz는 z 성분을 섞지 않음). gyro_odometer가 쓰는 것과 동일한 토픽/부호.
    imu_sub_ = create_subscription<Imu>(
      "sensors/imu/raw", rclcpp::SensorDataQoS(),
      std::bind(&VescToOdom::imuCallback, this, _1));
    RCLCPP_INFO(get_logger(), "odom yaw rate 소스: IMU 자이로 (서보 자전거모델 아님)");
  }
}

void VescToOdom::imuCallback(const Imu::SharedPtr imu)
{
  last_imu_ = imu;
}

void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  // check that we have a last servo command if we are depending on it for angular velocity
  if (use_servo_cmd_ && !last_servo_cmd_) {
    return;
  }

  // convert to engineering units
  double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05) {
    current_speed = 0.0;
  }
  double current_steering_angle(0.0), current_angular_velocity(0.0);
  bool have_angular_velocity = false;
  if (use_servo_cmd_) {
    current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
    have_angular_velocity = true;
  }

  // 자전거 모델은 '명령한' 조향각을 쓰므로 서보 지연(~100ms)과 타이어 슬립을 전혀 못 담는다.
  // 0728_1644 bag: 2.15초 주행에 yaw가 -4.58도 틀렸고 t=15.4s에서는 부호까지 반대였다
  // (모델 -0.146 rad/s vs 실제 +0.262 rad/s). 그 결과 map->odom 보정이 2.2초 만에
  // 1.3m/11도 흔들렸다(= replay에서 odom 프레임이 요동치는 현상). IMU를 쓰면 오차 -0.45도.
  if (use_imu_angular_velocity_ && last_imu_) {
    const double imu_age =
      (rclcpp::Time(state->header.stamp) - rclcpp::Time(last_imu_->header.stamp)).seconds();
    if (std::fabs(imu_age) < imu_timeout_s_) {
      current_angular_velocity = last_imu_->angular_velocity.z;
      have_angular_velocity = true;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "IMU가 %.2fs 오래됨 - yaw rate를 서보 자전거모델로 폴백", imu_age);
    }
  }

  // use current state as last state if this is our first time here
  if (!last_state_) {
    last_state_ = state;
  }

  // calc elapsed time
  auto dt = rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp);

  /** @todo could probably do better propigating odometry, e.g. trapezoidal integration */

  // propigate odometry
  double x_dot = current_speed * cos(yaw_);
  double y_dot = current_speed * sin(yaw_);
  x_ += x_dot * dt.seconds();
  y_ += y_dot * dt.seconds();
  if (have_angular_velocity) {
    yaw_ += current_angular_velocity * dt.seconds();
  }

  // save state for next time
  last_state_ = state;

  // publish odometry message
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = state->header.stamp;
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
  odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

  // Position uncertainty
  // 기본값 전부 1e-4로 채우고, x/y/yaw만 덮어씀
  std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 1e-4);
  odom.pose.covariance[0]  = 0.25;  // var(x)
  odom.pose.covariance[7]  = 0.50;  // var(y)
  odom.pose.covariance[35] = 0.40;  // var(yaw)

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom.twist.twist.linear.x = current_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = current_angular_velocity;

  // Velocity uncertainty
  // 기본값 전부 1e-4로 채우고, vx/vy/wz만 덮어씀
  std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 1e-4);
  odom.twist.covariance[0]  = 0.02; // var(vx)
  odom.twist.covariance[7]  = 0.05; // var(vy)
  odom.twist.covariance[35] = 0.00; // var(wz)

  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = state->header.stamp;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(odom);
  }
}

void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)