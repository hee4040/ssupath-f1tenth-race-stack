// src/mux_controller.cpp
#include <optional>
#include <algorithm>
#include <vector>
#include <deque>               

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

// 커스텀 메시지
#include "mpcc_ros/msg/mpcc_control.hpp"              // duty_cycle, servo, solver_status
#include "f110_msgs/msg/l1controller_control.hpp"      // steering_angle, speed

class TopController : public rclcpp::Node {
public:
  TopController()
  : rclcpp::Node("mux_controller")
  {
    // ---- 파라미터 ----
    loop_rate_hz_ = this->declare_parameter<double>("loop_rate_hz", 40.0);

    joy_toggle_button_idx_  = this->declare_parameter<int>("joy_toggle_button_idx", 5);
    joy_estop_enabled_ = this->declare_parameter<bool>("joy_estop_enabled", true);
    joy_estop_button_idx_ = this->declare_parameter<int>("joy_estop_button_idx", 2);
    joy_estop_release_button_idx_ = this->declare_parameter<int>("joy_estop_release_button_idx", 3);
    start_hold_enabled_ = this->declare_parameter<bool>("start_hold_enabled", true);
    start_hold_button_idx_ = this->declare_parameter<int>("start_hold_button_idx", 1);
    start_hold_servo_position_ = this->declare_parameter<double>("start_hold_servo_position", 0.5);

    // AUTO→JOY 전환 시 정지 펄스 지속 시간(ms)
    joy_stop_dwell_ms_ = this->declare_parameter<int>("joy_stop_dwell_ms", 300);

    // MPCC 안정성 히스토리 길이
    mpcc_hist_len_ = this->declare_parameter<int>("mpcc_hist_len", 5);

    // MPCC 메시지 신선도 한계: 이보다 오래되면 불안정 취급 (노드 사망 시 마지막 duty로 폭주 방지)
    mpcc_timeout_ms_ = this->declare_parameter<int>("mpcc_timeout_ms", 200);

    // 제어 알고리즘 고정: "PP"/"MAP"이면 항상 L1(/drive), "MPCC"면 항상 MPCC(duty 직접).
    // state(TRAILING/GB_TRACK)와 무관 — TRAILING 거동은 l1_controller가 /state로 자체 처리
    ctrl_mode_ = this->declare_parameter<std::string>("ctrl_mode", "PP");

    // ---- 퍼블리셔 ----
    duty_pub_  = this->create_publisher<std_msgs::msg::Float64>("/commands/motor/duty_cycle", 1);
    servo_pub_ = this->create_publisher<std_msgs::msg::Float64>("/commands/servo/position", 1);
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);

    // ---- 서브스크립션 ----
    mpcc_sub_ = this->create_subscription<mpcc_ros::msg::MpccControl>(
      "/mpcc/control", rclcpp::QoS(1),
      std::bind(&TopController::on_mpcc, this, std::placeholders::_1));

    l1_sub_ = this->create_subscription<f110_msgs::msg::L1controllerControl>(
      "/l1controller/control", rclcpp::QoS(1),
      std::bind(&TopController::on_l1, this, std::placeholders::_1));

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(1),
      std::bind(&TopController::on_joy, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/state", rclcpp::QoS(1),
      [this](const std_msgs::msg::String::SharedPtr msg){
        state_ = msg->data;
      });

    // ---- 타이머 루프 ----
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / loop_rate_hz_),
      std::bind(&TopController::loop, this));

    RCLCPP_INFO(
      get_logger(),
      "TopController started. Mode=AUTO ctrl_mode=%s (toggle btn idx=%d, dwell=%d ms, estop=%s stop idx=%d release idx=%d, start_hold=%s idx=%d)",
      ctrl_mode_.c_str(),
      joy_toggle_button_idx_,
      joy_stop_dwell_ms_,
      joy_estop_enabled_ ? "on" : "off",
      joy_estop_button_idx_,
      joy_estop_release_button_idx_,
      start_hold_enabled_ ? "on" : "off",
      start_hold_button_idx_);
  }

private:
  // 모드 정의
  enum class ControlMode { AUTO, JOY_ARMING, JOY };
  ControlMode mode_{ControlMode::AUTO};

  void on_mpcc(const mpcc_ros::msg::MpccControl::SharedPtr msg) {
    last_mpcc_ = *msg;
    last_mpcc_rx_time_ = this->get_clock()->now();

    const bool ok = (msg->solver_status == 0);
    mpcc_ok_hist_.push_back(ok);
    if (mpcc_ok_hist_.size() > mpcc_hist_len_) mpcc_ok_hist_.pop_front();
  }

  void on_l1(const f110_msgs::msg::L1controllerControl::SharedPtr msg) { last_l1_ = *msg; }

  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
    const auto &btns = msg->buttons;

    if (!last_buttons_) {
      last_buttons_ = std::vector<int>(btns.size(), 0);
    }

    handle_joy_estop(btns);
    handle_start_hold(btns);

    // JOY 토글 (버튼으로 AUTO<->JOY 토글)
    if (is_rising_edge(btns, joy_toggle_button_idx_)) {
      if (mode_ == ControlMode::JOY || mode_ == ControlMode::JOY_ARMING) {
        mode_ = ControlMode::AUTO;
        RCLCPP_INFO(get_logger(), "JOY toggle -> Mode=AUTO");
      } else { // AUTO -> JOY_ARMING (정지 펄스 구간)
        mode_ = ControlMode::JOY_ARMING;
        const auto now_t = this->get_clock()->now();
        rclcpp::Duration dwell = rclcpp::Duration::from_nanoseconds(
            static_cast<int64_t>(joy_stop_dwell_ms_ * 1e6));
        joy_stop_until_ = now_t + dwell;
        RCLCPP_INFO(get_logger(),
          "JOY toggle -> Mode=JOY_ARMING (publishing stop for %d ms)", joy_stop_dwell_ms_);
      }
    }

    last_buttons_ = btns; // 다음 상승에지 비교를 위해 저장
  }

  bool is_rising_edge(const std::vector<int>& buttons, int idx) const {
    if (idx < 0) return false;
    const bool curr = (static_cast<size_t>(idx) < buttons.size()) ? (buttons[idx] == 1) : false;
    bool prev = false;
    if (last_buttons_.has_value() && static_cast<size_t>(idx) < last_buttons_->size())
      prev = ((*last_buttons_)[idx] == 1);
    return (curr && !prev);
  }

  void handle_joy_estop(const std::vector<int>& buttons) {
    if (!joy_estop_enabled_) return;

    const bool stop_edge = is_rising_edge(buttons, joy_estop_button_idx_);
    const bool release_edge = is_rising_edge(buttons, joy_estop_release_button_idx_);

    if (!joy_estop_latched_ && stop_edge) {
      joy_estop_latched_ = true;
      publish_estop_stop();
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
  }

  void handle_start_hold(const std::vector<int>& buttons) {
    if (!start_hold_enabled_ || start_hold_button_idx_ < 0) {
      start_hold_pressed_ = false;
      return;
    }

    const auto idx = static_cast<size_t>(start_hold_button_idx_);
    const bool pressed = idx < buttons.size() && buttons[idx] == 1;

    if (pressed && !start_hold_pressed_) {
      RCLCPP_WARN(
        get_logger(),
        "Start hold button pressed (idx=%d). AUTO commands paused until release.",
        start_hold_button_idx_);
    } else if (!pressed && start_hold_pressed_) {
      RCLCPP_WARN(
        get_logger(),
        "Start hold released (idx=%d) -> AUTO commands resumed.",
        start_hold_button_idx_);
    }
    start_hold_pressed_ = pressed;
  }

  // ---- 메인 루프 ----
  void loop() {
    if (joy_estop_enabled_ && joy_estop_latched_) {
      publish_estop_stop();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "JOYSTICK E-STOP active.");
      return;
    }

    switch (mode_) {
      case ControlMode::JOY_ARMING:
        // dwell 동안 정지 명령만 퍼블리시 (안전 정지)
        if (this->get_clock()->now() < joy_stop_until_) {
          publish_stop();
          return;
        } else {
          mode_ = ControlMode::JOY;
          RCLCPP_INFO(get_logger(), "JOY arming done -> Mode=JOY (mux_controller stops publishing /drive)");
          return; // 이번 틱은 퍼블리시 없이 종료 (다음 틱부터 완전 JOY)
        }

      case ControlMode::JOY:
        // JOY 모드: mux_controller는 /drive 퍼블리시 안 함 (joy_teleop가 퍼블리시)
        return;

      case ControlMode::AUTO:
      default:
        break;
    }

    if (start_hold_enabled_ && start_hold_pressed_) {
      publish_start_hold_stop();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "START HOLD active. Release button idx=%d to resume AUTO.",
        start_hold_button_idx_);
      return;
    }

    // ===== AUTO 모드 로직 =====
    // ctrl_mode로 소스를 고정: state와 무관하게 선택한 컨트롤러 하나만 사용

    if (ctrl_mode_ == "MPCC") {
      if (mpcc_is_stable()) {
        publish_low_level_from_mpcc(*last_mpcc_);
      } else {
        // 솔버 불안정 시 다른 컨트롤러로 몰래 전환하지 않고 안전 정지
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "ctrl_mode=MPCC but MPCC not stable -> publishing stop.");
        publish_stop();
      }
      return;
    }

    // PP / MAP: 항상 L1 경로 (/drive, 폐루프 속도 제어)
    if (last_l1_) {
      publish_ackermann_from_l1(*last_l1_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "ctrl_mode=%s but no /l1controller/control yet.", ctrl_mode_.c_str());
    }
  }


  // ---- 헬퍼 함수들 ----
  bool mpcc_is_stable() const {
    if (!last_mpcc_) return false;
    if (mpcc_ok_hist_.size() < mpcc_hist_len_) return false;
    // 히스토리는 수신 시에만 갱신되므로, 발행이 끊기면(솔버/노드 사망) 여기서 걸러야 함
    const double age_ms =
      (this->now() - last_mpcc_rx_time_).nanoseconds() / 1e6;
    if (age_ms > static_cast<double>(mpcc_timeout_ms_)) return false;
    return std::all_of(mpcc_ok_hist_.begin(), mpcc_ok_hist_.end(),
                       [](bool v){ return v; });
  }

  void publish_low_level_from_mpcc(const mpcc_ros::msg::MpccControl &mc) {
    std_msgs::msg::Float64 duty;  duty.data  = mc.duty_cycle; duty_pub_->publish(duty);
    std_msgs::msg::Float64 servo; servo.data = mc.servo;      servo_pub_->publish(servo);
  }

  void publish_ackermann_from_l1(const f110_msgs::msg::L1controllerControl &l1) {
    ackermann_msgs::msg::AckermannDriveStamped ack;
    ack.header.stamp = now();
    ack.header.frame_id = "base_link";
    ack.drive.steering_angle = l1.steering_angle;
    ack.drive.speed = l1.speed;
    drive_pub_->publish(ack);
  }

  void publish_stop() {
    ackermann_msgs::msg::AckermannDriveStamped stop;
    stop.header.stamp = now();
    stop.header.frame_id = "base_link";
    stop.drive.steering_angle = 0.0;
    stop.drive.speed = 0.0;
    drive_pub_->publish(stop);
  }

  void publish_estop_stop() {
    publish_stop();

    std_msgs::msg::Float64 duty;
    duty.data = 0.0;
    duty_pub_->publish(duty);
  }

  void publish_start_hold_stop() {
    publish_estop_stop();

    std_msgs::msg::Float64 servo;
    servo.data = start_hold_servo_position_;
    servo_pub_->publish(servo);
  }

private:
  // ---- 파라미터 ----
  double loop_rate_hz_{40.0};
  int joy_toggle_button_idx_{5};
  bool joy_estop_enabled_{true};
  int joy_estop_button_idx_{2};
  int joy_estop_release_button_idx_{3};
  bool start_hold_enabled_{true};
  int start_hold_button_idx_{1};
  double start_hold_servo_position_{0.5};
  int joy_stop_dwell_ms_{300};     // AUTO→JOY 정지 펄스 시간
  std::string ctrl_mode_{"PP"};    // PP/MAP: 항상 L1, MPCC: 항상 MPCC (state 무관)

  // ---- 퍼블리셔 ----
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duty_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

  // ---- 서브스크립션 ----
  rclcpp::Subscription<mpcc_ros::msg::MpccControl>::SharedPtr mpcc_sub_;
  rclcpp::Subscription<f110_msgs::msg::L1controllerControl>::SharedPtr l1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_; 
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- 상태 ----
  std::optional<mpcc_ros::msg::MpccControl> last_mpcc_;
  std::optional<f110_msgs::msg::L1controllerControl> last_l1_;
  std::optional<std::vector<int>> last_buttons_;
  bool joy_estop_latched_{false};
  bool start_hold_pressed_{false};
  std::string state_; 

  // JOY 전이(arming) 끝나는 시각
  rclcpp::Time joy_stop_until_;

  // MPCC 안정성 히스토리 (최근 5개)
  size_t mpcc_hist_len_{5};
  std::deque<bool> mpcc_ok_hist_;
  int mpcc_timeout_ms_{200};        // 이보다 오래된 MPCC 메시지는 불안정 취급
  rclcpp::Time last_mpcc_rx_time_;  // on_mpcc에서만 설정 (last_mpcc_ 있을 때만 읽음)
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopController>());
  rclcpp::shutdown();
  return 0;
}
