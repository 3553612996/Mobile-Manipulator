// Minimal YDLIDAR X2 driver for M4.
//
// Publishes a full-revolution LaserScan (for SLAM Toolbox / Nav2 later) and a
// PointCloud2 (for a quick RViz check). The serial port is read from a 10 ms
// timer; at 115200 baud the X2 produces about 12 KB/s, so one non-blocking
// drain per tick is plenty.
//
// Coordinates: the X2 datasheet defines positive angles clockwise from the
// front, while ROS REP-103 uses CCW. invert_angle defaults to true so the
// scan matches the laser_link frame; set it false to mirror the vendor X2.yaml
// (reversion=false, inverted=false), which leaves the raw clockwise angles.
// A missing sample becomes +inf in LaserScan ranges.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "robot_lidar/x2_protocol.hpp"
#include "robot_lidar/x2_serial.hpp"

namespace robot_lidar
{
namespace
{

using namespace std::chrono_literals;

struct CloudPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

}  // namespace

class X2Lidar : public rclcpp::Node
{
public:
  X2Lidar()
  : Node("x2_lidar")
  {
    declare_parameters();
    validate_parameters();

    angle_increment_ = 2.0 * x2::kPi / static_cast<double>(scan_bins_);
    ranges_.assign(static_cast<std::size_t>(scan_bins_),
      std::numeric_limits<float>::infinity());

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(10).best_effort());
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "lidar/points", rclcpp::QoS(10).best_effort());

    RCLCPP_INFO(
      get_logger(),
      "X2 lidar: port=%s baud=%d frame=%s bins=%d motor_dtr=%s",
      port_.c_str(), baud_, frame_id_.c_str(), scan_bins_,
      motor_dtr_ ? "true" : "false");

    next_open_attempt_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(timer_period_ms_),
      std::bind(&X2Lidar::on_timer, this));
  }

  ~X2Lidar() override
  {
    // Dropping DTR stops the adapter's motor; also closes the fd.
    serial_.close();
  }

private:
  void declare_parameters()
  {
    port_ = declare_parameter<std::string>("port", "/dev/robot_lidar");
    baud_ = declare_parameter<int>("baud", 115200);
    frame_id_ = declare_parameter<std::string>("frame_id", "laser_link");
    angle_offset_deg_ = declare_parameter<double>("angle_offset_deg", 0.0);
    invert_angle_ = declare_parameter<bool>("invert_angle", true);
    range_min_m_ = declare_parameter<double>("range_min_m", 0.12);
    range_max_m_ = declare_parameter<double>("range_max_m", 8.0);
    scan_bins_ = declare_parameter<int>("scan_bins", 720);
    publish_scan_ = declare_parameter<bool>("publish_scan", true);
    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", true);
    motor_dtr_ = declare_parameter<bool>("motor_dtr", true);
    scan_frequency_hz_ = declare_parameter<double>("scan_frequency_hz", 6.0);
    timer_period_ms_ = declare_parameter<int>("timer_period_ms", 10);
    reconnect_period_s_ = declare_parameter<double>("reconnect_period_s", 2.0);
  }

  void validate_parameters()
  {
    if (scan_bins_ <= 0) {
      throw std::invalid_argument("scan_bins must be > 0");
    }
    if (range_min_m_ <= 0.0 || range_max_m_ <= range_min_m_) {
      throw std::invalid_argument("range_min_m must be > 0 and < range_max_m");
    }
    if (scan_frequency_hz_ <= 0.0) {
      throw std::invalid_argument("scan_frequency_hz must be > 0");
    }
    if (timer_period_ms_ <= 0) {
      throw std::invalid_argument("timer_period_ms must be > 0");
    }
    if (reconnect_period_s_ <= 0.0) {
      throw std::invalid_argument("reconnect_period_s must be > 0");
    }
  }

  void try_open()
  {
    std::string error;
    if (!serial_.open(port_, baud_, error)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "cannot open %s: %s; retrying", port_.c_str(), error.c_str());
      next_open_attempt_ = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(reconnect_period_s_));
      return;
    }

    parser_.reset();
    reset_scan();
    has_last_packet_ = false;
    has_start_packets_ = false;

    if (motor_dtr_) {
      std::string dtr_error;
      if (!serial_.set_dtr(true, dtr_error)) {
        // A direct UART has no DTR line. The USB adapter does; without this
        // the X2 stays idle and no point packets arrive.
        RCLCPP_WARN(
          get_logger(),
          "DTR assert failed: %s (expected on a direct UART, not on the USB adapter)",
          dtr_error.c_str());
      }
    }

    RCLCPP_INFO(
      get_logger(), "X2 lidar serial port %s opened at %d baud",
      port_.c_str(), baud_);
  }

  void on_timer()
  {
    if (!serial_.is_open()) {
      if (std::chrono::steady_clock::now() >= next_open_attempt_) {
        try_open();
      }
      return;
    }

    uint8_t buffer[4096];
    for (;;) {
      const int count = serial_.read(buffer, sizeof(buffer));
      if (count == 0) {
        break;
      }
      if (count < 0) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "X2 lidar read failed; closing %s", port_.c_str());
        serial_.close();
        next_open_attempt_ = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(reconnect_period_s_));
        break;
      }

      for (const auto & packet : parser_.feed(buffer, static_cast<std::size_t>(count))) {
        handle_packet(packet);
      }
    }
  }

  void handle_packet(const x2::Packet & packet)
  {
    if (packet.is_start) {
      // Start packet marks the beginning of a lap. Publish the completed scan
      // before starting the next one.
      if (points_in_scan_ > 0) {
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

    // Fallback only for devices that never emit start packets. The X2 sends
    // one every revolution at ~338 deg; after that start packet the next
    // point packet is at ~7 deg, so angle-wrap detection would split every
    // lap in half if it stayed enabled.
    if (!has_start_packets_ && has_last_packet_ &&
      packet.start_angle_deg + 180.0 < last_start_angle_deg_)
    {
      if (points_in_scan_ > 0) {
        publish_scan();
      }
      reset_scan();
    }

    last_start_angle_deg_ = packet.start_angle_deg;
    has_last_packet_ = true;

    accumulate(packet);
  }

  void accumulate(const x2::Packet & packet)
  {
    const double offset_rad = angle_offset_deg_ * x2::kPi / 180.0;

    for (const auto & point : packet.points) {
      if (point.raw_distance == 0U) {
        continue;
      }
      if (point.distance_m < range_min_m_ || point.distance_m > range_max_m_) {
        continue;
      }

      if (points_in_scan_ == 0U) {
        // First accepted point of this revolution. The published scan is
        // stamped at the midpoint between here and the last point, so keep
        // the arrival time (see publish_scan()).
        scan_start_stamp_ = now();
        has_scan_start_ = true;
      }

      const double raw_rad = point.angle_deg * x2::kPi / 180.0;
      const double theta = x2::normalize_rad_pi(
        (invert_angle_ ? -raw_rad : raw_rad) + offset_rad);

      const int bin = static_cast<int>(
        std::floor((theta + x2::kPi) / angle_increment_));
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

    // One revolution takes about 166 ms at 6 Hz. Stamping it when the last
    // point arrives would make the first point up to 166 ms stale; the
    // midpoint halves the worst-case error and pairs with time_increment.
    const rclcpp::Time end = now();
    rclcpp::Time stamp = end;
    if (has_scan_start_) {
      const int64_t start_ns = scan_start_stamp_.nanoseconds();
      stamp = rclcpp::Time(
        start_ns + (end.nanoseconds() - start_ns) / 2, end.get_clock_type());
    }

    if (publish_scan_) {
      sensor_msgs::msg::LaserScan message;
      message.header.stamp = stamp;
      message.header.frame_id = frame_id_;
      message.angle_min = -x2::kPi;
      // REP-117: angle_max = angle_min + (N - 1) * increment. Using +pi here
      // instead would make (max - min) / increment report 720 bins for a
      // 720-entry ranges array, which some consumers flag as inconsistent.
      message.angle_max = static_cast<float>(-x2::kPi) +
        static_cast<float>(scan_bins_ - 1) * static_cast<float>(angle_increment_);
      message.angle_increment = static_cast<float>(angle_increment_);
      const double frequency =
        measured_frequency_hz_ > 0.0 ? measured_frequency_hz_ : scan_frequency_hz_;
      message.scan_time = static_cast<float>(1.0 / frequency);
      // Points within one revolution are not simultaneous; declaring the
      // spacing lets SLAM and the costmap compensate for motion during a scan.
      message.time_increment = message.scan_time / static_cast<float>(scan_bins_);
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
    has_scan_start_ = false;
  }

  // parameters
  std::string port_;
  int baud_{115200};
  std::string frame_id_{"laser_link"};
  double angle_offset_deg_{0.0};
  bool invert_angle_{true};
  double range_min_m_{0.12};
  double range_max_m_{8.0};
  int scan_bins_{720};
  bool publish_scan_{true};
  bool publish_pointcloud_{true};
  bool motor_dtr_{true};
  double scan_frequency_hz_{6.0};
  int timer_period_ms_{10};
  double reconnect_period_s_{2.0};

  // runtime
  X2Serial serial_;
  x2::PacketParser parser_;
  std::vector<float> ranges_;
  std::vector<CloudPoint> scan_points_;
  std::size_t points_in_scan_{0U};
  rclcpp::Time scan_start_stamp_{0, 0, RCL_ROS_TIME};
  bool has_scan_start_{false};
  bool has_last_packet_{false};
  bool has_start_packets_{false};
  double last_start_angle_deg_{0.0};
  double measured_frequency_hz_{0.0};
  double angle_increment_{x2::kPi / 360.0};
  std::chrono::steady_clock::time_point next_open_attempt_{};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_lidar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_lidar::X2Lidar>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("x2_lidar"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
