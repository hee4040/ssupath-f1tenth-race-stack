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

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <sstream>
#include <string>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using sensor_msgs::msg::Joy;
using std::placeholders::_1;
using std_msgs::msg::Float64;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // get conversion parameters
  declare_parameter("speed_to_erpm_gain", 1.0); // Example default value of 1.0
  declare_parameter("speed_to_erpm_offset", 1.0);
  declare_parameter("steering_angle_to_servo_gain", 1.0);
  declare_parameter("steering_angle_to_servo_offset", 1.0);
  declare_parameter("joy_estop_enabled", true);
  declare_parameter("joy_estop_button_idx", 2);
  declare_parameter("joy_estop_release_button_idx", 3);
  declare_parameter("joy_estop_publish_rate_hz", 40.0);
  
  get_parameter("speed_to_erpm_gain", speed_to_erpm_gain_);
  get_parameter("speed_to_erpm_offset", speed_to_erpm_offset_);
  get_parameter("steering_angle_to_servo_gain", steering_to_servo_gain_);
  get_parameter("steering_angle_to_servo_offset", steering_to_servo_offset_);
  get_parameter("joy_estop_enabled", joy_estop_enabled_);
  get_parameter("joy_estop_button_idx", joy_estop_button_idx_);
  get_parameter("joy_estop_release_button_idx", joy_estop_release_button_idx_);
  get_parameter("joy_estop_publish_rate_hz", joy_estop_publish_rate_hz_);
  if (joy_estop_publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "joy_estop_publish_rate_hz must be positive. Falling back to 40 Hz.");
    joy_estop_publish_rate_hz_ = 40.0;
  }

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));

  joy_sub_ = create_subscription<Joy>(
    "/joy", 10, std::bind(&AckermannToVesc::joyCallback, this, _1));

  estop_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / joy_estop_publish_rate_hz_),
    [this]() {
      if (joy_estop_enabled_ && joy_estop_latched_) {
        publishStopCommand();
      }
    });

  RCLCPP_INFO(
    get_logger(),
    "Joystick e-stop %s. stop button idx=%d, release button idx=%d",
    joy_estop_enabled_ ? "enabled" : "disabled",
    joy_estop_button_idx_,
    joy_estop_release_button_idx_);
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  if (joy_estop_enabled_ && joy_estop_latched_) {
    publishStopCommand();
    return;
  }

  // calc vesc electric RPM (speed)
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // calc steering angle (servo)
  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // publish
  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
    servo_pub_->publish(servo_msg);
  }
}

void AckermannToVesc::joyCallback(const Joy::SharedPtr msg)
{
  const auto & buttons = msg->buttons;

  if (!have_last_buttons_) {
    last_buttons_ = std::vector<int>(buttons.size(), 0);
    have_last_buttons_ = true;
  }

  const bool stop_edge = isRisingEdge(buttons, joy_estop_button_idx_);
  const bool release_edge = isRisingEdge(buttons, joy_estop_release_button_idx_);

  if (!joy_estop_latched_ && stop_edge) {
    joy_estop_latched_ = true;
    publishStopCommand();
    RCLCPP_ERROR(
      get_logger(),
      "JOYSTICK E-STOP latched (button idx=%d). Press release button idx=%d to resume.",
      joy_estop_button_idx_,
      joy_estop_release_button_idx_);
  } else if (joy_estop_latched_ && release_edge) {
    joy_estop_latched_ = false;
    RCLCPP_WARN(
      get_logger(),
      "Joystick e-stop released (button idx=%d).",
      joy_estop_release_button_idx_);
  }

  last_buttons_ = buttons;
}

bool AckermannToVesc::isRisingEdge(const std::vector<int> & buttons, int idx) const
{
  if (idx < 0) {
    return false;
  }

  const auto button_idx = static_cast<size_t>(idx);
  const bool current = button_idx < buttons.size() && buttons[button_idx] == 1;
  const bool previous = button_idx < last_buttons_.size() && last_buttons_[button_idx] == 1;
  return current && !previous;
}

void AckermannToVesc::publishStopCommand()
{
  Float64 erpm_msg;
  erpm_msg.data = 0.0;

  Float64 servo_msg;
  servo_msg.data = steering_to_servo_offset_;

  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
    servo_pub_->publish(servo_msg);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
