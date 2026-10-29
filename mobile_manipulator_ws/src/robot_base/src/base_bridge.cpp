// ROS 2 <-> STM32 chassis bridge.
//
// Rules that shaped this file (see the M4 implementation note):
//  * robot_base owns the UART. No other ROS node opens /dev/robot_stm32.
//  * SET_VELOCITY is streamed from a 20 ms timer, not from the cmd_vel
//    callback: the MCU stops itself after 200 ms without a velocity frame.
//  * /odom pose is the MCU's own integration; only the heading is unwrapped.
//    Twist is computed here because REPORT_ODOM carries vl/vr, not wz.
//  * Only commands with an ACK bit set are sent through send_with_ack().

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rmw/qos_profiles.h"
#include "robot_base/cmd_vel_monitor.hpp"
#include "robot_base/heading_unwrapper.hpp"
#include "robot_base/protocol.hpp"
#include "robot_base/serial_io.hpp"
#include "robot_base/stm32_link.hpp"
#include "robot_interfaces/msg/robot_status.hpp"
#include "robot_interfaces/msg/system_event.hpp"
#include "robot_interfaces/srv/clear_fault.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace robot_base
{
namespace
{

using namespace std::chrono_literals;

int16_t to_i16_milli(double value)
{
  const double clamped = std::clamp(value, -32768.0, 32767.0);
  return static_cast<int16_t>(std::lround(clamped));
}

double steady_now_s()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

class BaseBridge : public rclcpp::Node
{
public:
  BaseBridge()
  : Node("base_bridge")
  {
    declare_parameters();
    validate_parameters();

    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", reliable,
      std::bind(&BaseBridge::on_cmd_vel, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", reliable);
    status_pub_ = create_publisher<robot_interfaces::msg::RobotStatus>(
      "robot_status", reliable);
    event_pub_ = create_publisher<robot_interfaces::msg::SystemEvent>(
      "system_event", reliable);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)));

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Service callbacks block while waiting for an ACK (up to 4 x 200 ms).
    // They live in a reentrant group and the executor runs several threads so
    // the 20 ms velocity timer keeps streaming during that wait.
    services_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    set_enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "set_enable",
      std::bind(
        &BaseBridge::on_set_enable, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, services_group_);
    clear_fault_srv_ = create_service<robot_interfaces::srv::ClearFault>(
      "clear_fault",
      std::bind(
        &BaseBridge::on_clear_fault, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, services_group_);

    link_ = std::make_unique<Stm32Link>(
      make_termios_serial_io(),
      std::bind(&BaseBridge::on_frame, this, std::placeholders::_1));
    link_->set_link_timeout(std::chrono::milliseconds(
        static_cast<int64_t>(link_timeout_s_ * 1000.0)));

    RCLCPP_INFO(
      get_logger(), "firmware version unknown (protocol v1 has no GET_VERSION)");
    RCLCPP_INFO(
      get_logger(),
      "wheel_separation_m=%.3f must match firmware SET_PARAM 0x01; "
      "verify a positive w increases yaw on the bench",
      wheel_separation_m_);

    std::string error;
    if (link_->open(port_, baud_, &error)) {
      RCLCPP_INFO(get_logger(), "serial port %s opened at %d baud",
        port_.c_str(), baud_);
    } else {
      RCLCPP_ERROR(get_logger(), "serial open failed: %s; will retry",
        error.c_str());
      next_open_attempt_ = std::chrono::steady_clock::now() + 2s;
    }

    velocity_timer_ = create_wall_timer(
      std::chrono::milliseconds(velocity_period_ms_),
      std::bind(&BaseBridge::on_velocity_tick, this));
    link_timer_ = create_wall_timer(
      1s, std::bind(&BaseBridge::on_link_timer, this));
  }

  ~BaseBridge() override
  {
    // Stop the RX thread before the publishers and TF broadcaster are
    // destroyed. Without this, a shutdown can crash while a late report
    // callback publishes through a half-destroyed node.
    if (link_) {
      link_->close();
    }
  }

private:
  void declare_parameters()
  {
    port_ = declare_parameter<std::string>("port", "/dev/robot_stm32");
    baud_ = declare_parameter<int>("baud", 115200);
    wheel_separation_m_ = declare_parameter<double>("wheel_separation_m", 0.24);
    velocity_period_ms_ = declare_parameter<int>("velocity_period_ms", 20);
    cmd_vel_timeout_s_ = declare_parameter<double>("cmd_vel_timeout_s", 0.5);
    link_timeout_s_ = declare_parameter<double>("link_timeout_s", 0.5);
    ack_timeout_ms_ = declare_parameter<int>("ack_timeout_ms", 200);
    ack_retries_ = declare_parameter<int>("ack_retries", 3);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    warn_unsupported_cmd_vel_ =
      declare_parameter<bool>("warn_unsupported_cmd_vel", true);
    unsupported_cmd_vel_warn_period_s_ =
      declare_parameter<double>("unsupported_cmd_vel_warn_period_s", 5.0);
    odom_pose_covariance_xy_ =
      declare_parameter<double>("odom_pose_covariance_xy", 0.05);
    odom_pose_covariance_yaw_ =
      declare_parameter<double>("odom_pose_covariance_yaw", 0.02);
    odom_twist_covariance_xy_ =
      declare_parameter<double>("odom_twist_covariance_xy", 0.05);
    odom_twist_covariance_yaw_ =
      declare_parameter<double>("odom_twist_covariance_yaw", 0.05);

    cmd_vel_monitor_ = std::make_unique<CmdVelMonitor>(
      warn_unsupported_cmd_vel_, unsupported_cmd_vel_warn_period_s_);
  }

  void validate_parameters()
  {
    if (wheel_separation_m_ <= 0.0) {
      throw std::invalid_argument("wheel_separation_m must be > 0");
    }
    if (velocity_period_ms_ <= 0) {
      throw std::invalid_argument("velocity_period_ms must be > 0");
    }
    if (cmd_vel_timeout_s_ <= 0.0) {
      throw std::invalid_argument("cmd_vel_timeout_s must be > 0");
    }
    if (link_timeout_s_ <= 0.0) {
      throw std::invalid_argument("link_timeout_s must be > 0");
    }
    if (ack_timeout_ms_ <= 0) {
      throw std::invalid_argument("ack_timeout_ms must be > 0");
    }
    if (ack_retries_ < 0) {
      throw std::invalid_argument("ack_retries must be >= 0");
    }
    if (unsupported_cmd_vel_warn_period_s_ < 0.0) {
      throw std::invalid_argument(
              "unsupported_cmd_vel_warn_period_s must be >= 0");
    }
    if (odom_pose_covariance_xy_ < 0.0 || odom_pose_covariance_yaw_ < 0.0 ||
      odom_twist_covariance_xy_ < 0.0 || odom_twist_covariance_yaw_ < 0.0)
    {
      throw std::invalid_argument("odometry covariances must be >= 0");
    }
  }

  // ---------------------------------------------------------------- inputs

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    const double linear_x = message->linear.x;
    const double linear_y = message->linear.y;
    const double linear_z = message->linear.z;
    const double angular_x = message->angular.x;
    const double angular_y = message->angular.y;
    const double angular_z = message->angular.z;

    if (!std::isfinite(linear_x) || !std::isfinite(linear_y) ||
      !std::isfinite(linear_z) || !std::isfinite(angular_x) ||
      !std::isfinite(angular_y) || !std::isfinite(angular_z))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "ignoring non-finite cmd_vel");
      return;
    }

    const auto warning = cmd_vel_monitor_->update(
      linear_x, linear_y, linear_z, angular_x, angular_y, angular_z,
      steady_now_s());
    if (warning) {
      RCLCPP_WARN(get_logger(), "%s", warning->c_str());
    }

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_linear_x_ = linear_x;
    cmd_angular_z_ = angular_z;
    cmd_stamp_ = std::chrono::steady_clock::now();
    has_cmd_ = true;
  }

  // The stream must never stop while the link is alive. It carries the last
  // command, or zero when the command is stale or the chassis is not enabled.
  void on_velocity_tick()
  {
    if (!link_->is_open() || !link_->link_alive()) {
      return;
    }

    double linear_x = 0.0;
    double angular_z = 0.0;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      const double age_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - cmd_stamp_).count();
      if (enabled_.load() && has_cmd_ && age_s <= cmd_vel_timeout_s_) {
        linear_x = cmd_linear_x_;
        angular_z = cmd_angular_z_;
      }
    }

    const auto payload = protocol::encode_set_velocity(
      to_i16_milli(linear_x * 1000.0), to_i16_milli(angular_z * 1000.0));

    std::string error;
    if (!link_->send(
        static_cast<uint8_t>(protocol::Cmd::SET_VELOCITY), payload, &error))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "SET_VELOCITY not sent: %s",
        error.c_str());
    }
  }

  // --------------------------------------------------------------- reports

  void on_frame(const protocol::Frame & frame)
  {
    switch (frame.cmd) {
      case static_cast<uint8_t>(protocol::Report::ODOM):
        handle_odom(frame);
        break;
      case static_cast<uint8_t>(protocol::Report::STATUS):
        handle_status(frame);
        break;
      case static_cast<uint8_t>(protocol::Report::EVENT):
        handle_event(frame);
        break;
      default:
        // REPORT_ARM / WHEEL_* / STORAGE / LOG / CRASH belong to robot_arm or
        // to bench tools. Ignore them here instead of growing a second owner.
        break;
    }
  }

  void handle_odom(const protocol::Frame & frame)
  {
    const auto odom = protocol::decode_odom(frame.payload);
    if (!odom) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "bad REPORT_ODOM length %zu",
        frame.payload.size());
      return;
    }

    if (resync_odom_.exchange(false)) {
      heading_.reset();
    }
    const double yaw = heading_.update(odom->theta_centi_deg);

    const auto stamp = now();
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_id_;
    message.child_frame_id = base_frame_id_;

    message.pose.pose.position.x = static_cast<double>(odom->x_mm) / 1000.0;
    message.pose.pose.position.y = static_cast<double>(odom->y_mm) / 1000.0;
    message.pose.pose.position.z = 0.0;

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();

    // REPORT_ODOM has no wz; derive it from the two side speeds. The wheel
    // separation must match the firmware calibration (SET_PARAM 0x01).
    const double vl = static_cast<double>(odom->vl_mm_s);
    const double vr = static_cast<double>(odom->vr_mm_s);
    message.twist.twist.linear.x = (vl + vr) / 2000.0;
    message.twist.twist.angular.z = (vr - vl) / 1000.0 / wheel_separation_m_;

    message.pose.covariance[0] = odom_pose_covariance_xy_;
    message.pose.covariance[7] = odom_pose_covariance_xy_;
    message.pose.covariance[35] = odom_pose_covariance_yaw_;
    message.twist.covariance[0] = odom_twist_covariance_xy_;
    message.twist.covariance[7] = odom_twist_covariance_xy_;
    message.twist.covariance[35] = odom_twist_covariance_yaw_;

    odom_pub_->publish(message);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = odom_frame_id_;
      transform.child_frame_id = base_frame_id_;
      transform.transform.translation.x = message.pose.pose.position.x;
      transform.transform.translation.y = message.pose.pose.position.y;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation = message.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void handle_status(const protocol::Frame & frame)
  {
    const auto status = protocol::decode_status(frame.payload);
    if (!status) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "bad REPORT_STATUS length %zu",
        frame.payload.size());
      return;
    }

    const uint8_t previous_state = state_.exchange(status->state);
    if (status->state == static_cast<uint8_t>(protocol::State::INIT)) {
      // The MCU restarted (or is still initializing): its heading starts over,
      // so the unwrapper must reseed from the next report. A mere serial
      // disconnect does not reset it -- the MCU kept integrating.
      resync_odom_.store(true);
    }
    if (status->state != previous_state) {
      RCLCPP_INFO(
        get_logger(), "state %s -> %s", protocol::state_name(previous_state).c_str(),
        protocol::state_name(status->state).c_str());
    }
    if (status->fault != fault_code_.exchange(status->fault)) {
      if (status->fault == 0U) {
        RCLCPP_INFO(get_logger(), "fault cleared");
      } else {
        RCLCPP_WARN(
          get_logger(), "fault 0x%04X %s", status->fault,
          protocol::fault_name(status->fault).c_str());
      }
    }
    battery_mv_.store(status->battery_mv);
    temperature_c_.store(status->temperature_c);
    jitter_us_.store(status->jitter_us);
    enabled_.store(status->state == static_cast<uint8_t>(protocol::State::MANUAL));

    robot_interfaces::msg::RobotStatus message;
    message.header.stamp = now();
    message.state = status->state;
    message.fault_code = status->fault;
    message.battery_mv = status->battery_mv;
    message.temperature_c = status->temperature_c;
    message.max_sched_jitter_us = status->jitter_us;
    status_pub_->publish(message);
  }

  void handle_event(const protocol::Frame & frame)
  {
    const auto event = protocol::decode_event(frame.payload);
    if (!event) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "bad REPORT_EVENT length %zu",
        frame.payload.size());
      return;
    }

    robot_interfaces::msg::SystemEvent message;
    message.header.stamp = now();
    message.fault_code = event->code;
    message.mcu_uptime_ms = event->timestamp_ms;
    event_pub_->publish(message);
  }

  // -------------------------------------------------------------- services

  Stm32Link::AckResult send_acked_command(
    protocol::Cmd cmd, const std::vector<uint8_t> & payload, std::string & error)
  {
    return link_->send_with_ack(
      static_cast<uint8_t>(cmd), payload,
      std::chrono::milliseconds(ack_timeout_ms_), ack_retries_, &error);
  }

  void on_set_enable(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (!link_->is_open() || !link_->link_alive()) {
      response->success = false;
      response->message = "serial link is not available";
      publish_ack_failure("set_enable", 0x04, response->message);
      return;
    }

    std::string error;
    const auto ack = send_acked_command(
      protocol::Cmd::SET_ENABLE, protocol::encode_set_enable(request->data), error);

    if (!ack.received) {
      response->success = false;
      response->message = error.empty() ? "no ACK" : error;
      publish_ack_failure("set_enable", 0x04, response->message);
      return;
    }

    const bool ok = ack.result == static_cast<uint8_t>(protocol::Result::OK);
    response->success = ok;
    response->message = protocol::result_name(ack.result);

    if (ok && !request->data) {
      // Stop passing commands through immediately; the firmware takes effect
      // on its own schedule but there is no reason to wait for it.
      enabled_.store(false);
    } else if (ok && request->data) {
      // Commands stay gated until REPORT_STATUS says MANUAL, so a lost status
      // frame can never let a velocity command reach a disabled chassis.
      RCLCPP_INFO(get_logger(), "SET_ENABLE(1) accepted; waiting for MANUAL");
    }
  }

  void on_clear_fault(
    const robot_interfaces::srv::ClearFault::Request::SharedPtr request,
    robot_interfaces::srv::ClearFault::Response::SharedPtr response)
  {
    if (!link_->is_open() || !link_->link_alive()) {
      response->success = false;
      response->message = "serial link is not available";
      publish_ack_failure("clear_fault", 0x07, response->message);
      return;
    }

    std::string error;
    const auto ack = send_acked_command(
      protocol::Cmd::CLEAR_FAULT,
      protocol::encode_clear_fault(request->fault_code), error);

    if (!ack.received) {
      response->success = false;
      response->message = error.empty() ? "no ACK" : error;
      publish_ack_failure("clear_fault", 0x07, response->message);
      return;
    }

    response->result_code = ack.result;
    response->success = ack.result == static_cast<uint8_t>(protocol::Result::OK);
    response->message = protocol::result_name(ack.result);
  }

  // ---------------------------------------------------------- link / diag

  void on_link_timer()
  {
    const auto now_steady = std::chrono::steady_clock::now();

    // A disconnected USB adapter can leave a valid-looking fd whose read()
    // returns 0 forever. If the port is still "open" but no frame has arrived
    // for a while, close it so the reopen path below gets a chance.
    const int64_t stale_after_ms =
      static_cast<int64_t>(link_timeout_s_ * 1000.0) + 2000;
    if (link_->is_open() && !link_->link_alive() &&
      link_->last_frame_age_ms() > stale_after_ms)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "closing stale serial port (%lld ms without a frame); will reopen",
        static_cast<long long>(link_->last_frame_age_ms()));
      link_->close();
      next_open_attempt_ = now_steady + 2s;
    }

    if (!link_->is_open() && now_steady >= next_open_attempt_) {
      std::string error;
      if (link_->open(port_, baud_, &error)) {
        RCLCPP_INFO(
          get_logger(), "serial port %s opened at %d baud", port_.c_str(), baud_);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000, "serial open failed: %s",
          error.c_str());
        next_open_attempt_ = now_steady + 2s;
      }
    }

    const bool open = link_->is_open();
    const bool alive = open && link_->link_alive();
    const bool was_alive = link_was_alive_.exchange(alive);

    if (alive && !was_alive) {
      // The heading is not reset here: a serial gap does not mean the MCU
      // rebooted, and resetting mid-stream would reintroduce a yaw jump.
      RCLCPP_INFO(get_logger(), "serial link up");
    } else if (!alive && was_alive) {
      RCLCPP_ERROR(
        get_logger(),
        "serial link lost (%lld ms since last frame); stopping velocity stream",
        static_cast<long long>(link_->last_frame_age_ms()));
    } else if (!alive && open) {
      if (link_->has_frame()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "no frames from MCU (%lld ms since last frame)",
          static_cast<long long>(link_->last_frame_age_ms()));
      } else if (
        link_->last_frame_age_ms() >
        static_cast<int64_t>(link_timeout_s_ * 1000.0) + 1000)
      {
        // Reopening is followed by a short quiet period before the first
        // report arrives; only warn once that window has clearly passed.
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "no frames since %s was opened", port_.c_str());
      }
    }

    publish_diagnostics(open, alive);
  }

  void publish_diagnostics(bool open, bool alive)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "robot_base: serial link";
    status.hardware_id = port_;

    if (!open) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "serial port is not open";
    } else if (!alive) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = link_->has_frame() ?
        "no frame within link_timeout" : "no frame since port opened";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "link ok";
    }

    add_kv(&status, "port", port_);
    add_kv(&status, "baud", std::to_string(baud_));
    add_kv(&status, "rx_frames", std::to_string(link_->rx_frames()));
    add_kv(&status, "crc_errors", std::to_string(link_->crc_errors()));
    add_kv(&status, "tx_frames", std::to_string(link_->tx_frames()));
    add_kv(&status, "ack_timeouts", std::to_string(link_->ack_timeouts()));
    add_kv(&status, "last_frame_age_ms", std::to_string(link_->last_frame_age_ms()));
    add_kv(&status, "control_state", protocol::state_name(state_.load()));
    add_kv(&status, "fault_code", protocol::fault_name(fault_code_.load()));
    add_kv(&status, "battery_mv", std::to_string(battery_mv_.load()));
    add_kv(&status, "temperature_c", std::to_string(temperature_c_.load()));
    add_kv(&status, "max_sched_jitter_us", std::to_string(jitter_us_.load()));

    array.status.push_back(status);
    diag_pub_->publish(array);
  }

  void publish_ack_failure(
    const std::string & operation, uint8_t command, const std::string & message)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.name = "robot_base: command ack";
    status.hardware_id = port_;
    status.message = operation + ": " + message;

    char command_text[8];
    std::snprintf(command_text, sizeof(command_text), "0x%02X", command);
    add_kv(&status, "command", command_text);
    add_kv(&status, "operation", operation);

    array.status.push_back(status);
    diag_pub_->publish(array);
  }

  static void add_kv(
    diagnostic_msgs::msg::DiagnosticStatus * status, const std::string & key,
    const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status->values.push_back(kv);
  }

  // ------------------------------------------------------------- members

  std::string port_;
  int baud_{115200};
  double wheel_separation_m_{0.24};
  int velocity_period_ms_{20};
  double cmd_vel_timeout_s_{0.5};
  double link_timeout_s_{0.5};
  int ack_timeout_ms_{200};
  int ack_retries_{3};
  bool publish_tf_{true};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};
  bool warn_unsupported_cmd_vel_{true};
  double unsupported_cmd_vel_warn_period_s_{5.0};
  double odom_pose_covariance_xy_{0.05};
  double odom_pose_covariance_yaw_{0.02};
  double odom_twist_covariance_xy_{0.05};
  double odom_twist_covariance_yaw_{0.05};

  std::unique_ptr<CmdVelMonitor> cmd_vel_monitor_;
  std::unique_ptr<Stm32Link> link_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::CallbackGroup::SharedPtr services_group_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<robot_interfaces::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<robot_interfaces::msg::SystemEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enable_srv_;
  rclcpp::Service<robot_interfaces::srv::ClearFault>::SharedPtr clear_fault_srv_;
  rclcpp::TimerBase::SharedPtr velocity_timer_;
  rclcpp::TimerBase::SharedPtr link_timer_;

  std::mutex cmd_mutex_;
  double cmd_linear_x_{0.0};
  double cmd_angular_z_{0.0};
  bool has_cmd_{false};
  std::chrono::steady_clock::time_point cmd_stamp_{};

  std::atomic<bool> enabled_{false};
  std::atomic<uint8_t> state_{0};
  std::atomic<uint16_t> fault_code_{0};
  std::atomic<uint16_t> battery_mv_{0};
  std::atomic<int8_t> temperature_c_{0};
  std::atomic<uint16_t> jitter_us_{0};

  HeadingUnwrapper heading_;
  std::atomic<bool> resync_odom_{false};
  std::atomic<bool> link_was_alive_{false};
  std::chrono::steady_clock::time_point next_open_attempt_{};
};

}  // namespace robot_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_base::BaseBridge>();
    // 4 threads so a blocking service callback never starves the velocity
    // timer; two concurrent services still leave two threads for timers.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("base_bridge"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
