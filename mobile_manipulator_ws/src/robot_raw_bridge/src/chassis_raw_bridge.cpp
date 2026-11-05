// Test bridge: host-published STM32 raw frames -> /odom + TF + status/event.
//
// Input:  std_msgs/msg/UInt8MultiArray on /chassis/raw (parameter input_topic)
// Output: nav_msgs/Odometry on /odom
//         TF odom -> base_link
//         robot_interfaces/RobotStatus on /robot_status
//         robot_interfaces/SystemEvent on /system_event
//
// Only reports are consumed. This node does not send commands and does not
// open a local serial port; it is for the host-forwarded raw stream.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "robot_base/heading_unwrapper.hpp"
#include "robot_base/protocol.hpp"
#include "robot_interfaces/msg/robot_status.hpp"
#include "robot_interfaces/msg/system_event.hpp"

namespace robot_raw_bridge
{

class ChassisRawBridge : public rclcpp::Node
{
public:
  ChassisRawBridge()
  : Node("chassis_raw_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/stm32/raw");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    wheel_separation_m_ = declare_parameter<double>("wheel_separation_m", 0.165);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    odom_pose_covariance_xy_ =
      declare_parameter<double>("odom_pose_covariance_xy", 0.05);
    odom_pose_covariance_yaw_ =
      declare_parameter<double>("odom_pose_covariance_yaw", 0.02);
    odom_twist_covariance_xy_ =
      declare_parameter<double>("odom_twist_covariance_xy", 0.05);
    odom_twist_covariance_yaw_ =
      declare_parameter<double>("odom_twist_covariance_yaw", 0.05);

    if (wheel_separation_m_ <= 0.0) {
      throw std::invalid_argument("wheel_separation_m must be > 0");
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10).reliable());
    status_pub_ = create_publisher<robot_interfaces::msg::RobotStatus>(
      "robot_status", rclcpp::QoS(10).reliable());
    event_pub_ = create_publisher<robot_interfaces::msg::SystemEvent>(
      "system_event", rclcpp::QoS(10).reliable());
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    raw_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      input_topic_, rclcpp::QoS(20).best_effort(),
      std::bind(&ChassisRawBridge::on_raw, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "chassis_raw_bridge: input=%s wheel_separation=%.3f",
      input_topic_.c_str(), wheel_separation_m_);
  }

private:
  void on_raw(const std_msgs::msg::UInt8MultiArray::SharedPtr message)
  {
    for (const auto & frame : parser_.feed(message->data.data(), message->data.size())) {
      handle_frame(frame);
    }
  }

  void handle_frame(const robot_base::protocol::Frame & frame)
  {
    switch (frame.cmd) {
      case static_cast<uint8_t>(robot_base::protocol::Report::ODOM):
        handle_odom(frame);
        break;
      case static_cast<uint8_t>(robot_base::protocol::Report::STATUS):
        handle_status(frame);
        break;
      case static_cast<uint8_t>(robot_base::protocol::Report::EVENT):
        handle_event(frame);
        break;
      default:
        break;
    }
  }

  void handle_odom(const robot_base::protocol::Frame & frame)
  {
    const auto odom = robot_base::protocol::decode_odom(frame.payload);
    if (!odom) {
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

  void handle_status(const robot_base::protocol::Frame & frame)
  {
    const auto status = robot_base::protocol::decode_status(frame.payload);
    if (!status) {
      return;
    }

    if (status->state == static_cast<uint8_t>(robot_base::protocol::State::INIT)) {
      resync_odom_.store(true);
    }

    robot_interfaces::msg::RobotStatus message;
    message.header.stamp = now();
    message.state = status->state;
    message.fault_code = status->fault;
    message.battery_mv = status->battery_mv;
    message.temperature_c = status->temperature_c;
    message.max_sched_jitter_us = status->jitter_us;
    status_pub_->publish(message);
  }

  void handle_event(const robot_base::protocol::Frame & frame)
  {
    const auto event = robot_base::protocol::decode_event(frame.payload);
    if (!event) {
      return;
    }

    robot_interfaces::msg::SystemEvent message;
    message.header.stamp = now();
    message.fault_code = event->code;
    message.mcu_uptime_ms = event->timestamp_ms;
    event_pub_->publish(message);
  }

  std::string input_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  double wheel_separation_m_{0.165};
  bool publish_tf_{true};
  double odom_pose_covariance_xy_{0.05};
  double odom_pose_covariance_yaw_{0.02};
  double odom_twist_covariance_xy_{0.05};
  double odom_twist_covariance_yaw_{0.05};

  robot_base::protocol::FrameParser parser_;
  robot_base::HeadingUnwrapper heading_;
  std::atomic<bool> resync_odom_{false};

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<robot_interfaces::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<robot_interfaces::msg::SystemEvent>::SharedPtr event_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace robot_raw_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_raw_bridge::ChassisRawBridge>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("chassis_raw_bridge"), "startup failed: %s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
