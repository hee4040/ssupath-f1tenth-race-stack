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

#include "gyro_odometer/gyro_odometer_core.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>

std::array<double, 9> transformCovariance(const std::array<double, 9> & cov)
{
  using COV_IDX = tier4_autoware_utils::xyz_covariance_index::XYZ_COV_IDX;

  double max_cov = std::max({cov[COV_IDX::X_X], cov[COV_IDX::Y_Y], cov[COV_IDX::Z_Z]});

  std::array<double, 9> cov_transformed;
  cov_transformed.fill(0.);
  cov_transformed[COV_IDX::X_X] = max_cov;
  cov_transformed[COV_IDX::Y_Y] = max_cov;
  cov_transformed[COV_IDX::Z_Z] = max_cov;
  return cov_transformed;
}

geometry_msgs::msg::TwistWithCovarianceStamped concatGyroAndOdometer(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & vehicle_twist_queue,
  const std::deque<sensor_msgs::msg::Imu> & gyro_queue)
{
  using COV_IDX_XYZ = tier4_autoware_utils::xyz_covariance_index::XYZ_COV_IDX;
  using COV_IDX_XYZRPY = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

  double vx_mean = 0.0;
  double vy_mean = 0.0;
  double vx_covariance_original = 0.0;
  double vy_covariance_original = 0.0; 


  geometry_msgs::msg::Vector3 gyro_mean{};
  geometry_msgs::msg::Vector3 gyro_covariance_original{};
  
  for (const auto & vehicle_twist : vehicle_twist_queue) {
    vx_mean += vehicle_twist.twist.twist.linear.x;
    vy_mean += vehicle_twist.twist.twist.linear.y;                         
    vx_covariance_original += vehicle_twist.twist.covariance[0 * 6 + 0];
    vy_covariance_original += vehicle_twist.twist.covariance[1 * 6 + 1]; 
  }
  vx_mean /= vehicle_twist_queue.size();
  vy_mean = vy_mean / vehicle_twist_queue.size();           
  vx_covariance_original /= vehicle_twist_queue.size();
  vy_covariance_original = vy_covariance_original / vehicle_twist_queue.size();

  for (const auto & gyro : gyro_queue) {
    gyro_mean.x += gyro.angular_velocity.x;
    gyro_mean.y += gyro.angular_velocity.y;
    gyro_mean.z += gyro.angular_velocity.z;
    gyro_covariance_original.x += gyro.angular_velocity_covariance[COV_IDX_XYZ::X_X];
    gyro_covariance_original.y += gyro.angular_velocity_covariance[COV_IDX_XYZ::Y_Y];
    gyro_covariance_original.z += gyro.angular_velocity_covariance[COV_IDX_XYZ::Z_Z];
  }
  gyro_mean.x /= gyro_queue.size();
  gyro_mean.y /= gyro_queue.size();
  gyro_mean.z /= gyro_queue.size();
  gyro_covariance_original.x /= gyro_queue.size();
  gyro_covariance_original.y /= gyro_queue.size();
  gyro_covariance_original.z /= gyro_queue.size();

  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov;
  const auto latest_vehicle_twist_stamp = rclcpp::Time(vehicle_twist_queue.back().header.stamp);
  const auto latest_imu_stamp = rclcpp::Time(gyro_queue.back().header.stamp);
  if (latest_vehicle_twist_stamp < latest_imu_stamp) {
    twist_with_cov.header.stamp = latest_imu_stamp;
  } else {
    twist_with_cov.header.stamp = latest_vehicle_twist_stamp;
  }
  twist_with_cov.header.frame_id = gyro_queue.front().header.frame_id;
  twist_with_cov.twist.twist.linear.x = vx_mean;
  twist_with_cov.twist.twist.linear.y = vy_mean; 
  twist_with_cov.twist.twist.angular = gyro_mean;


  // ------------------------------------------------------------------
  // 여기부터가 covariance 조절하는 핵심 부분
  // ------------------------------------------------------------------

  // 1) 평균 속도의 분산 / 평균 각속도의 분산 (원래 코드와 같은 의미)
  double vx_cov  = vx_covariance_original / vehicle_twist_queue.size();
  double vy_cov  = vy_covariance_original / vehicle_twist_queue.size();
  double yaw_cov = gyro_covariance_original.z / gyro_queue.size();

  // 2) 최소 표준편차 바닥값 설정 (원하는 값으로 튜닝)
  //    예) vx: 0.2 m/s, vy: 0.2 m/s, yaw: 0.2 rad/s
  constexpr double MIN_VX_STD  = 0.2;
  constexpr double MIN_VY_STD  = 0.2;
  constexpr double MIN_YAW_STD = 0.2;

  const double MIN_VX_COV  = MIN_VX_STD  * MIN_VX_STD;
  const double MIN_VY_COV  = MIN_VY_STD  * MIN_VY_STD;
  const double MIN_YAW_COV = MIN_YAW_STD * MIN_YAW_STD;

  vx_cov  = std::max(vx_cov,  MIN_VX_COV);
  vy_cov  = std::max(vy_cov,  MIN_VY_COV);
  yaw_cov = std::max(yaw_cov, MIN_YAW_COV);

  // 3) 전체적인 신뢰도를 떨어뜨리기 위한 inflation factor
  //    (처음에는 5.0 정도로 시작해서, 필요하면 2~10 사이로 조절)
  constexpr double COV_INFLATE_FACTOR = 5.0;

  vx_cov  *= COV_INFLATE_FACTOR;
  vy_cov  *= COV_INFLATE_FACTOR;
  yaw_cov *= COV_INFLATE_FACTOR;

  // 4) 최종 covariance 대입
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::X_X]     = vx_cov;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Y_Y]     = vy_cov;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Z_Z]     = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::ROLL_ROLL]  = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::PITCH_PITCH] = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::YAW_YAW] = yaw_cov;

  // From a statistical point of view, here we reduce the covariances according to the number of
  // observed data
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::X_X] =
    vx_covariance_original / vehicle_twist_queue.size();

  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Y_Y] = 100000.0;
  // twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Y_Y] =
  //   vy_covariance_original / vehicle_twist_queue.size();
    
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Z_Z] = 100000.0;
  // twist_with_cov.twist.covariance[COV_IDX_XYZRPY::ROLL_ROLL] =
  // gyro_covariance_original.x / gyro_queue.size();
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::ROLL_ROLL] = 100000.0;

  // twist_with_cov.twist.covariance[COV_IDX_XYZRPY::PITCH_PITCH] =
  //  gyro_covariance_original.y / gyro_queue.size();
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::PITCH_PITCH] = 100000.0;

  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::YAW_YAW] =
    gyro_covariance_original.z / gyro_queue.size();
  // twist_with_cov.twist.covariance[COV_IDX_XYZRPY::YAW_YAW] = 100000.0;

  return twist_with_cov;
}

GyroOdometer::GyroOdometer(const rclcpp::NodeOptions & options)
: Node("gyro_odometer", options),
  output_frame_(declare_parameter("output_frame", "base_link")),
  message_timeout_sec_(declare_parameter("message_timeout_sec", 0.2)),
  imu_mux_mode_(declare_parameter("imu_mux_mode", std::string("vesc"))),
  imu_mux_weight_vesc_pct_(declare_parameter("imu_mux_weight_vesc", 50.0)),
  imu_mux_livox_bias_wz_(declare_parameter("imu_mux_livox_bias_wz", 0.01)),
  imu_mux_timeout_sec_(declare_parameter("imu_mux_timeout_sec", 0.1)),
  imu_mux_vesc_topic_(declare_parameter("imu_mux_vesc_topic", std::string("/sensors/imu/raw"))),
  imu_mux_livox_topic_(declare_parameter("imu_mux_livox_topic", std::string("/livox/imu"))),
  enable_imu_mux_csv_(declare_parameter("enable_imu_mux_csv", true)),
  imu_mux_csv_dir_(declare_parameter(
    "imu_mux_csv_dir", std::string("/home/misys/forza_ws/race_stack/plusresult"))),
  enable_sensor_delay_log_(declare_parameter("enable_sensor_delay_log", true)),
  sensor_delay_log_throttle_sec_(declare_parameter("sensor_delay_log_throttle_sec", 1.0)),
  has_vesc_imu_(false),
  has_livox_imu_(false),
  vehicle_twist_arrived_(false),
  imu_arrived_(false)
{
  imu_mux_weight_vesc_pct_ = std::clamp(imu_mux_weight_vesc_pct_, 0.0, 100.0);
  weight_vesc_ = imu_mux_weight_vesc_pct_ / 100.0;
  weight_livox_ = (100.0 - imu_mux_weight_vesc_pct_) / 100.0;

  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  vehicle_twist_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "vehicle/twist_with_covariance", rclcpp::QoS{1},
    std::bind(&GyroOdometer::callbackVehicleTwist, this, std::placeholders::_1));
  imu_vesc_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_mux_vesc_topic_, rclcpp::QoS{1},
    std::bind(&GyroOdometer::callbackImuMuxVesc, this, std::placeholders::_1));
  imu_livox_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_mux_livox_topic_, rclcpp::QoS{1},
    std::bind(&GyroOdometer::callbackImuMuxLivox, this, std::placeholders::_1));

  twist_raw_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist_raw", rclcpp::QoS{1});
  twist_with_covariance_raw_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance_raw", rclcpp::QoS{1});

  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist", rclcpp::QoS{1});
  twist_with_covariance_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", rclcpp::QoS{1});
  imu_vesc_delay_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/timestamp_relay_3d/imu_vesc_delay", rclcpp::QoS{10});

  if (enable_imu_mux_csv_) {
    std::error_code ec;
    std::filesystem::create_directories(imu_mux_csv_dir_, ec);
    const auto now_sys = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now_sys);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    const std::string csv_path =
      imu_mux_csv_dir_ + "/" + imu_mux_mode_ + "_imu_" + buf + ".csv";
    imu_mux_csv_.open(csv_path, std::ios::out);
    if (imu_mux_csv_.is_open()) {
      imu_mux_csv_
        << "ros_time,msg_stamp,selected_mode,selected_wz,vesc_wz,livox_wz,livox_wz_raw,"
           "livox_bias_wz,vesc_weight,livox_weight,vesc_valid,livox_valid,fallback_used,"
           "vesc_header_stamp,livox_header_stamp,angular_velocity_x,angular_velocity_y,"
           "angular_velocity_z,linear_acceleration_x,linear_acceleration_y,linear_acceleration_z\n";
      RCLCPP_INFO(get_logger(), "IMUmux CSV: %s", csv_path.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to open IMUmux CSV: %s", csv_path.c_str());
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "IMUmux mode=%s weight_vesc=%.1f%% (livox=%.1f%%) bias_wz=%.4f timeout=%.3f",
    imu_mux_mode_.c_str(), imu_mux_weight_vesc_pct_, 100.0 - imu_mux_weight_vesc_pct_,
    imu_mux_livox_bias_wz_, imu_mux_timeout_sec_);
}

GyroOdometer::~GyroOdometer()
{
  if (imu_mux_csv_.is_open()) {
    imu_mux_csv_.close();
  }
}

void GyroOdometer::callbackVehicleTwist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr vehicle_twist_ptr)
{
  vehicle_twist_arrived_ = true;
  if (!imu_arrived_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Imu msg is not subscribed");
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  const double twist_dt = std::abs((this->now() - vehicle_twist_ptr->header.stamp).seconds());
  if (twist_dt > message_timeout_sec_) {
    const std::string error_msg = fmt::format(
      "Twist msg is timeout. twist_dt: {}[sec], tolerance {}[sec]", twist_dt, message_timeout_sec_);
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, error_msg.c_str());
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  vehicle_twist_queue_.clear();
  vehicle_twist_queue_.push_back(*vehicle_twist_ptr);  // keep-latest 1
  tryFuseAndPublish();
}

void GyroOdometer::callbackImuMuxVesc(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  latest_vesc_imu_ = *imu_msg_ptr;
  has_vesc_imu_ = true;
  processImuMux("vesc");
}

void GyroOdometer::callbackImuMuxLivox(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  latest_livox_imu_ = *imu_msg_ptr;
  has_livox_imu_ = true;
  processImuMux("livox");
}

bool GyroOdometer::isImuFresh(const builtin_interfaces::msg::Time & stamp) const
{
  return std::abs((this->now() - rclcpp::Time(stamp)).seconds()) <= imu_mux_timeout_sec_;
}

bool GyroOdometer::transformImuToOutput(
  const sensor_msgs::msg::Imu & imu, geometry_msgs::msg::Vector3 & ang_out,
  geometry_msgs::msg::Vector3 & acc_out, std::array<double, 9> & ang_cov_out)
{
  geometry_msgs::msg::TransformStamped tf;
  if (imu.header.frame_id == output_frame_) {
    tf.header = imu.header;
    tf.child_frame_id = output_frame_;
    tf.transform.rotation.w = 1.0;
  } else {
    const auto tf_ptr =
      transform_listener_->getLatestTransform(imu.header.frame_id, output_frame_);
    if (!tf_ptr) {
      return false;
    }
    tf = *tf_ptr;
  }

  geometry_msgs::msg::Vector3Stamped ang_in;
  ang_in.header = imu.header;
  ang_in.vector = imu.angular_velocity;
  geometry_msgs::msg::Vector3Stamped acc_in;
  acc_in.header = imu.header;
  acc_in.vector = imu.linear_acceleration;
  geometry_msgs::msg::Vector3Stamped ang_tf;
  geometry_msgs::msg::Vector3Stamped acc_tf;
  tf2::doTransform(ang_in, ang_tf, tf);
  tf2::doTransform(acc_in, acc_tf, tf);
  ang_out = ang_tf.vector;
  acc_out = acc_tf.vector;
  ang_cov_out = transformCovariance(imu.angular_velocity_covariance);
  return true;
}

void GyroOdometer::appendImuMuxCsv(
  const sensor_msgs::msg::Imu & selected, double selected_wz, double vesc_wz, double livox_wz,
  double livox_wz_raw, double vesc_weight, double livox_weight, bool vesc_valid, bool livox_valid,
  bool fallback_used)
{
  if (!enable_imu_mux_csv_ || !imu_mux_csv_.is_open()) {
    return;
  }
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double vesc_header_stamp =
    has_vesc_imu_ ? rclcpp::Time(latest_vesc_imu_.header.stamp).seconds() : nan;
  const double livox_header_stamp =
    has_livox_imu_ ? rclcpp::Time(latest_livox_imu_.header.stamp).seconds() : nan;

  imu_mux_csv_ << std::setprecision(17) << this->now().seconds() << ","
               << rclcpp::Time(selected.header.stamp).seconds() << "," << imu_mux_mode_ << ","
               << selected_wz << "," << vesc_wz << "," << livox_wz << "," << livox_wz_raw << ","
               << imu_mux_livox_bias_wz_ << "," << vesc_weight << "," << livox_weight << ","
               << (vesc_valid ? 1 : 0) << "," << (livox_valid ? 1 : 0) << ","
               << (fallback_used ? 1 : 0) << "," << vesc_header_stamp << "," << livox_header_stamp
               << "," << selected.angular_velocity.x << "," << selected.angular_velocity.y << ","
               << selected.angular_velocity.z << "," << selected.linear_acceleration.x << ","
               << selected.linear_acceleration.y << "," << selected.linear_acceleration.z << "\n";
}

void GyroOdometer::processImuMux(const std::string & trigger)
{
  if (imu_mux_mode_ == "vesc" && trigger != "vesc") {
    return;
  }
  if (imu_mux_mode_ == "livox" && trigger != "livox") {
    return;
  }

  const bool want_vesc =
    imu_mux_mode_ == "vesc" || imu_mux_mode_ == "weighted" || imu_mux_mode_ == "test";
  const bool want_livox =
    imu_mux_mode_ == "livox" || imu_mux_mode_ == "weighted" || imu_mux_mode_ == "test";

  const double nan = std::numeric_limits<double>::quiet_NaN();
  geometry_msgs::msg::Vector3 ang_vesc{};
  geometry_msgs::msg::Vector3 acc_vesc{};
  geometry_msgs::msg::Vector3 ang_livox{};
  geometry_msgs::msg::Vector3 acc_livox{};
  std::array<double, 9> cov_vesc{};
  std::array<double, 9> cov_livox{};
  bool vesc_valid = false;
  bool livox_valid = false;

  if (want_vesc && has_vesc_imu_) {
    vesc_valid = isImuFresh(latest_vesc_imu_.header.stamp) &&
                 transformImuToOutput(latest_vesc_imu_, ang_vesc, acc_vesc, cov_vesc);
  }
  if (want_livox && has_livox_imu_) {
    livox_valid = isImuFresh(latest_livox_imu_.header.stamp) &&
                  transformImuToOutput(latest_livox_imu_, ang_livox, acc_livox, cov_livox);
  }

  bool feed_gyro = true;
  bool fallback_used = false;
  double w_v = 0.0;
  double w_l = 0.0;
  double selected_wz = nan;
  sensor_msgs::msg::Imu selected;

  if (imu_mux_mode_ == "vesc") {
    if (!vesc_valid) {
      return;
    }
    selected = latest_vesc_imu_;
    selected_wz = ang_vesc.z;
    w_v = 1.0;
    w_l = 0.0;
  } else if (imu_mux_mode_ == "livox") {
    if (!livox_valid) {
      return;
    }
    selected = latest_livox_imu_;
    selected_wz = ang_livox.z;
    w_v = 0.0;
    w_l = 1.0;
  } else if (imu_mux_mode_ == "test") {
    w_v = 1.0;
    w_l = 0.0;
    if (vesc_valid) {
      selected = latest_vesc_imu_;
      selected_wz = ang_vesc.z;
      feed_gyro = (trigger == "vesc");
    } else if (livox_valid) {
      selected = latest_livox_imu_;
      selected_wz = nan;
      feed_gyro = false;
    } else {
      return;
    }
  } else if (imu_mux_mode_ == "weighted") {
    if (vesc_valid && livox_valid) {
      w_v = weight_vesc_;
      w_l = weight_livox_;
      fallback_used = false;
    } else if (vesc_valid) {
      w_v = 1.0;
      w_l = 0.0;
      fallback_used = true;
    } else if (livox_valid) {
      w_v = 0.0;
      w_l = 1.0;
      fallback_used = true;
    } else {
      return;
    }

    const auto tv = vesc_valid ? rclcpp::Time(latest_vesc_imu_.header.stamp) : rclcpp::Time(0);
    const auto tl = livox_valid ? rclcpp::Time(latest_livox_imu_.header.stamp) : rclcpp::Time(0);
    if (vesc_valid && livox_valid) {
      selected.header.stamp = (tv < tl) ? latest_livox_imu_.header.stamp : latest_vesc_imu_.header.stamp;
    } else if (livox_valid) {
      selected.header.stamp = latest_livox_imu_.header.stamp;
    } else {
      selected.header.stamp = latest_vesc_imu_.header.stamp;
    }
    selected.header.frame_id = output_frame_;
    selected.angular_velocity.x = w_v * ang_vesc.x + w_l * ang_livox.x;
    selected.angular_velocity.y = w_v * ang_vesc.y + w_l * ang_livox.y;
    selected.angular_velocity.z = w_v * ang_vesc.z + w_l * ang_livox.z;
    selected.linear_acceleration.x = w_v * acc_vesc.x + w_l * acc_livox.x;
    selected.linear_acceleration.y = w_v * acc_vesc.y + w_l * acc_livox.y;
    selected.linear_acceleration.z = w_v * acc_vesc.z + w_l * acc_livox.z;
    selected.angular_velocity_covariance.fill(0.0);
    using COV_IDX = tier4_autoware_utils::xyz_covariance_index::XYZ_COV_IDX;
    selected.angular_velocity_covariance[COV_IDX::X_X] =
      std::max(cov_vesc[COV_IDX::X_X], cov_livox[COV_IDX::X_X]);
    selected.angular_velocity_covariance[COV_IDX::Y_Y] =
      std::max(cov_vesc[COV_IDX::Y_Y], cov_livox[COV_IDX::Y_Y]);
    selected.angular_velocity_covariance[COV_IDX::Z_Z] =
      std::max(cov_vesc[COV_IDX::Z_Z], cov_livox[COV_IDX::Z_Z]);
    selected_wz = selected.angular_velocity.z;
  } else {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "Unknown imu_mux_mode: %s", imu_mux_mode_.c_str());
    return;
  }

  const double vesc_wz_csv = vesc_valid ? ang_vesc.z : nan;
  const double livox_wz_raw = livox_valid ? ang_livox.z : nan;
  const double livox_wz_csv = livox_valid ? (ang_livox.z - imu_mux_livox_bias_wz_) : nan;
  appendImuMuxCsv(
    selected, selected_wz, vesc_wz_csv, livox_wz_csv, livox_wz_raw, w_v, w_l, vesc_valid,
    livox_valid, fallback_used);

  if (feed_gyro) {
    callbackImu(std::make_shared<sensor_msgs::msg::Imu>(selected));
  }
}

void GyroOdometer::sampleImuVescDelayBeforeMax(
  const rclcpp::Time & vehicle_stamp, const rclcpp::Time & imu_stamp)
{
  if (!enable_sensor_delay_log_) {
    return;
  }
  const rclcpp::Time t_now = this->now();
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    (t_now - imu_stamp).seconds(), (t_now - vehicle_stamp).seconds(), imu_stamp.seconds(),
    vehicle_stamp.seconds()};
  imu_vesc_delay_pub_->publish(msg);
}

void GyroOdometer::tryFuseAndPublish()
{
  if (vehicle_twist_queue_.empty() || gyro_queue_.empty()) {
    return;
  }
  const double imu_dt = std::abs((this->now() - gyro_queue_.back().header.stamp).seconds());
  if (imu_dt > message_timeout_sec_) {
    const std::string error_msg = fmt::format(
      "Imu msg is timeout. imu_dt: {}[sec], tolerance {}[sec]", imu_dt, message_timeout_sec_);
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, error_msg.c_str());
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }
  const double twist_dt =
    std::abs((this->now() - vehicle_twist_queue_.back().header.stamp).seconds());
  if (twist_dt > message_timeout_sec_) {
    const std::string error_msg = fmt::format(
      "Twist msg is timeout. twist_dt: {}[sec], tolerance {}[sec]", twist_dt, message_timeout_sec_);
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, error_msg.c_str());
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  sampleImuVescDelayBeforeMax(
    rclcpp::Time(vehicle_twist_queue_.back().header.stamp),
    rclcpp::Time(gyro_queue_.back().header.stamp));
  const geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov_raw =
    concatGyroAndOdometer(vehicle_twist_queue_, gyro_queue_);
  publishData(twist_with_cov_raw);
  vehicle_twist_queue_.clear();
  gyro_queue_.clear();
}

void GyroOdometer::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  imu_arrived_ = true;
  if (!vehicle_twist_arrived_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Twist msg is not subscribed");
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  const double imu_dt = std::abs((this->now() - imu_msg_ptr->header.stamp).seconds());
  if (imu_dt > message_timeout_sec_) {
    const std::string error_msg = fmt::format(
      "Imu msg is timeout. imu_dt: {}[sec], tolerance {}[sec]", imu_dt, message_timeout_sec_);
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, error_msg.c_str());
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr;
  if (imu_msg_ptr->header.frame_id == output_frame_) {
    auto identity = std::make_shared<geometry_msgs::msg::TransformStamped>();
    identity->header = imu_msg_ptr->header;
    identity->child_frame_id = output_frame_;
    identity->transform.rotation.w = 1.0;
    tf_imu2base_ptr = identity;
  } else {
    tf_imu2base_ptr =
      transform_listener_->getLatestTransform(imu_msg_ptr->header.frame_id, output_frame_);
  }
  if (!tf_imu2base_ptr) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", output_frame_.c_str(),
      (imu_msg_ptr->header.frame_id).c_str());
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.header = imu_msg_ptr->header;
  angular_velocity.vector = imu_msg_ptr->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  transformed_angular_velocity.header = tf_imu2base_ptr->header;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_imu2base_ptr);

  sensor_msgs::msg::Imu gyro_base_link;
  gyro_base_link.header = imu_msg_ptr->header;
  gyro_base_link.header.frame_id = output_frame_;
  gyro_base_link.angular_velocity = transformed_angular_velocity.vector;
  gyro_base_link.angular_velocity_covariance =
    transformCovariance(imu_msg_ptr->angular_velocity_covariance);

  gyro_queue_.clear();
  gyro_queue_.push_back(gyro_base_link);  // keep-latest 1
  tryFuseAndPublish();
}

void GyroOdometer::publishData(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_raw)
{
  geometry_msgs::msg::TwistStamped twist_raw;
  twist_raw.header = twist_with_cov_raw.header;
  twist_raw.twist = twist_with_cov_raw.twist.twist;

  twist_raw_pub_->publish(twist_raw);
  twist_with_covariance_raw_pub_->publish(twist_with_cov_raw);

  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance = twist_with_cov_raw;
  geometry_msgs::msg::TwistStamped twist = twist_raw;

  // clear imu yaw bias if vehicle is stopped
  if (
    std::fabs(twist_with_cov_raw.twist.twist.angular.z) < 0.01 &&
    std::fabs(twist_with_cov_raw.twist.twist.linear.x) < 0.01) {
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;
    twist_with_covariance.twist.twist.angular.x = 0.0;
    twist_with_covariance.twist.twist.angular.y = 0.0;
    twist_with_covariance.twist.twist.angular.z = 0.0;
  }

  twist_pub_->publish(twist);
  twist_with_covariance_pub_->publish(twist_with_covariance);
}
