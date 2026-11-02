// Test bridge: host-published X2 raw bytes -> /scan + /lidar/points.
//
// Input:  std_msgs/msg/UInt8MultiArray on /lidar/raw (parameter input_topic)
// Output: sensor_msgs/LaserScan on /scan
//         sensor_msgs/PointCloud2 on /lidar/points
//
// The parsing/accumulation logic is the same as robot_lidar/x2_node.cpp; only
// the byte source changed from a local UART to a ROS topic.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "robot_lidar/x2_protocol.hpp"

namespace robot_raw_bridge
{
namespace
{

struct CloudPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

}  // namespace

class LidarRawBridge : public rclcpp::Node
{
public:
  LidarRawBridge()
  : Node("lidar_raw_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar/raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "laser_link");
    angle_offset_deg_ = declare_parameter<double>("angle_offset_deg", 0.0);
    invert_angle_ = declare_parameter<bool>("invert_angle", true);
    range_min_m_ = declare_parameter<double>("range_min_m", 0.12);
    range_max_m_ = declare_parameter<double>("range_max_m", 8.0);
    scan_bins_ = declare_parameter<int>("scan_bins", 720);
    publish_scan_ = declare_parameter<bool>("publish_scan", true);
    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", true);
    scan_frequency_hz_ = declare_parameter<double>("scan_frequency_hz", 6.0);

    if (scan_bins_ <= 0) {
      throw std::invalid_argument("scan_bins must be > 0");
    }
    if (range_min_m_ <= 0.0 || range_max_m_ <= range_min_m_) {
      throw std::invalid_argument("range_min_m must be > 0 and < range_max_m");
    }

    angle_increment_ = 2.0 * robot_lidar::x2::kPi /
      static_cast<double>(scan_bins_);
    ranges_.assign(
      static_cast<std::size_t>(scan_bins_),
      std::numeric_limits<float>::infinity());

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(10).best_effort());
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "lidar/points", rclcpp::QoS(10).best_effort());

    raw_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      input_topic_, rclcpp::QoS(20).best_effort(),
      std::bind(&LidarRawBridge::on_raw, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "lidar_raw_bridge: input=%s frame=%s bins=%d",
      input_topic_.c_str(), frame_id_.c_str(), scan_bins_);
  }

private:
  void on_raw(const std_msgs::msg::UInt8MultiArray::SharedPtr message)
  {
    for (const auto & packet : parser_.feed(message->data.data(), message->data.size())) {
      handle_packet(packet);
    }
  }

  void handle_packet(const robot_lidar::x2::Packet & packet)
  {
    if (packet.is_start) {
      if (points_in_scan_ > 0U) {
        publish_scan();
      }
      reset_scan();
      has_last_packet_ = false;
      has_start_packets_ = true;
      if (packet.scan_frequency_hz > 0.0) {
        measured_frequency_hz_ = packet.scan_frequency_hz;
      }
      return;
    }

    if (packet.points.empty()) {
      return;
    }

    // Fallback only when the host stream never contains start packets.
    if (!has_start_packets_ && has_last_packet_ &&
      packet.start_angle_deg + 180.0 < last_start_angle_deg_)
    {
      if (points_in_scan_ > 0U) {
        publish_scan();
      }
      reset_scan();
    }

    last_start_angle_deg_ = packet.start_angle_deg;
    has_last_packet_ = true;
    accumulate(packet);
  }

  void accumulate(const robot_lidar::x2::Packet & packet)
  {
    const double offset_rad = angle_offset_deg_ * robot_lidar::x2::kPi / 180.0;

    for (const auto & point : packet.points) {
      if (point.raw_distance == 0U) {
        continue;
      }
      if (point.distance_m < range_min_m_ || point.distance_m > range_max_m_) {
        continue;
      }

      const double raw_rad = point.angle_deg * robot_lidar::x2::kPi / 180.0;
      const double theta = robot_lidar::x2::normalize_rad_pi(
        (invert_angle_ ? -raw_rad : raw_rad) + offset_rad);

      const int bin = static_cast<int>(
        std::floor((theta + robot_lidar::x2::kPi) / angle_increment_));
      if (bin >= 0 && bin < scan_bins_) {
        const float range = static_cast<float>(point.distance_m);
        if (range < ranges_[static_cast<std::size_t>(bin)]) {
          ranges_[static_cast<std::size_t>(bin)] = range;
        }
      }

      if (publish_pointcloud_) {
        CloudPoint cloud_point;
        cloud_point.x = static_cast<float>(point.distance_m * std::cos(theta));
        cloud_point.y = static_cast<float>(point.distance_m * std::sin(theta));
        cloud_point.z = 0.0F;
        scan_points_.push_back(cloud_point);
      }

      ++points_in_scan_;
    }
  }

  void publish_scan()
  {
    if (points_in_scan_ == 0U) {
      reset_scan();
      return;
    }

    const auto stamp = now();

    if (publish_scan_) {
      sensor_msgs::msg::LaserScan message;
      message.header.stamp = stamp;
      message.header.frame_id = frame_id_;
      message.angle_min = -robot_lidar::x2::kPi;
      message.angle_max = robot_lidar::x2::kPi;
      message.angle_increment = angle_increment_;
      message.time_increment = 0.0F;
      const double frequency =
        measured_frequency_hz_ > 0.0 ? measured_frequency_hz_ : scan_frequency_hz_;
      message.scan_time = static_cast<float>(1.0 / frequency);
      message.range_min = static_cast<float>(range_min_m_);
      message.range_max = static_cast<float>(range_max_m_);
      message.ranges = ranges_;
      scan_pub_->publish(message);
    }

    if (publish_pointcloud_ && !scan_points_.empty()) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.stamp = stamp;
      cloud.header.frame_id = frame_id_;
      cloud.height = 1;
      cloud.width = static_cast<uint32_t>(scan_points_.size());
      cloud.is_dense = true;

      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(scan_points_.size());

      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
      for (const auto & point : scan_points_) {
        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
      cloud_pub_->publish(cloud);
    }

    reset_scan();
  }

  void reset_scan()
  {
    std::fill(
      ranges_.begin(), ranges_.end(), std::numeric_limits<float>::infinity());
    scan_points_.clear();
    points_in_scan_ = 0U;
  }

  std::string input_topic_;
  std::string frame_id_;
  double angle_offset_deg_{0.0};
  bool invert_angle_{true};
  double range_min_m_{0.12};
  double range_max_m_{8.0};
  int scan_bins_{720};
  bool publish_scan_{true};
  bool publish_pointcloud_{true};
  double scan_frequency_hz_{6.0};

  robot_lidar::x2::PacketParser parser_;
  std::vector<float> ranges_;
  std::vector<CloudPoint> scan_points_;
  std::size_t points_in_scan_{0U};
  bool has_last_packet_{false};
  bool has_start_packets_{false};
  double last_start_angle_deg_{0.0};
  double measured_frequency_hz_{0.0};
  double angle_increment_{robot_lidar::x2::kPi / 360.0};

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace robot_raw_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_raw_bridge::LidarRawBridge>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("lidar_raw_bridge"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
