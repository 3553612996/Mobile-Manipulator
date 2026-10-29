#include "robot_base/cmd_vel_monitor.hpp"

#include <cmath>
#include <sstream>

namespace robot_base
{

std::optional<std::string> CmdVelMonitor::update(
  double linear_x, double linear_y, double linear_z,
  double angular_x, double angular_y, double angular_z,
  double now_s)
{
  (void)linear_x;
  (void)angular_z;

  if (!enabled_) {
    return std::nullopt;
  }

  const bool has_unsupported =
    std::fabs(linear_y) > threshold_ ||
    std::fabs(linear_z) > threshold_ ||
    std::fabs(angular_x) > threshold_ ||
    std::fabs(angular_y) > threshold_;

  if (!has_unsupported) {
    return std::nullopt;
  }

  if (warned_ && (now_s - last_warn_s_) < warn_period_s_) {
    return std::nullopt;
  }

  warned_ = true;
  last_warn_s_ = now_s;

  std::ostringstream text;
  text << "cmd_vel has components SET_VELOCITY cannot carry (discarded):";
  if (std::fabs(linear_y) > threshold_) {
    text << " linear.y=" << linear_y << " m/s";
  }
  if (std::fabs(linear_z) > threshold_) {
    text << " linear.z=" << linear_z << " m/s";
  }
  if (std::fabs(angular_x) > threshold_) {
    text << " angular.x=" << angular_x << " rad/s";
  }
  if (std::fabs(angular_y) > threshold_) {
    text << " angular.y=" << angular_y << " rad/s";
  }
  text << "; only linear.x and angular.z are sent";
  return text.str();
}

}  // namespace robot_base
