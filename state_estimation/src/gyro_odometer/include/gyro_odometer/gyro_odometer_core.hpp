// Copyright 2015-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GYRO_ODOMETER__GYRO_ODOMETER_CORE_HPP_
#define GYRO_ODOMETER__GYRO_ODOMETER_CORE_HPP_

#include "tier4_autoware_utils/ros/msg_covariance.hpp"
#include "tier4_autoware_utils/ros/transform_listener.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2/transform_datatypes.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <array>
#include <deque>
#include <fstream>
#include <memory>
#include <string>

class GyroOdometer : public rclcpp::Node
{
private:
  using COV_IDX = tier4_autoware_utils::xyz_covariance_index::XYZ_COV_IDX;

public:
  explicit GyroOdometer(const rclcpp::NodeOptions & options);
  ~GyroOdometer();

private:
  void callbackVehicleTwist(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr vehicle_twist_msg_ptr);
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);
  void callbackImuMuxVesc(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);
  void callbackImuMuxLivox(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);
  void processImuMux(const std::string & trigger);
  void sampleImuVescDelayBeforeMax(
    const rclcpp::Time & vehicle_stamp, const rclcpp::Time & imu_stamp);
  bool isImuFresh(const builtin_interfaces::msg::Time & stamp) const;
  bool transformImuToOutput(
    const sensor_msgs::msg::Imu & imu, geometry_msgs::msg::Vector3 & ang_out,
    geometry_msgs::msg::Vector3 & acc_out, std::array<double, 9> & ang_cov_out);
  void appendImuMuxCsv(
    const sensor_msgs::msg::Imu & selected, double selected_wz, double vesc_wz, double livox_wz,
    double livox_wz_raw, double vesc_weight, double livox_weight, bool vesc_valid, bool livox_valid,
    bool fallback_used);
  void tryFuseAndPublish();

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    vehicle_twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_vesc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_livox_sub_;

  void publishData(const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_raw);

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_with_covariance_raw_pub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_with_covariance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr imu_vesc_delay_pub_;

  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;

  std::string output_frame_;
  double message_timeout_sec_;

  std::string imu_mux_mode_;
  double imu_mux_weight_vesc_pct_;
  double imu_mux_livox_bias_wz_;
  double imu_mux_timeout_sec_;
  std::string imu_mux_vesc_topic_;
  std::string imu_mux_livox_topic_;
  bool enable_imu_mux_csv_;
  std::string imu_mux_csv_dir_;
  bool enable_sensor_delay_log_;
  double sensor_delay_log_throttle_sec_;

  double weight_vesc_;
  double weight_livox_;

  sensor_msgs::msg::Imu latest_vesc_imu_;
  sensor_msgs::msg::Imu latest_livox_imu_;
  bool has_vesc_imu_;
  bool has_livox_imu_;

  std::ofstream imu_mux_csv_;

  bool vehicle_twist_arrived_;
  bool imu_arrived_;
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> vehicle_twist_queue_;
  std::deque<sensor_msgs::msg::Imu> gyro_queue_;
};

#endif  // GYRO_ODOMETER__GYRO_ODOMETER_CORE_HPP_
