// SET_VELOCITY carries only v and w. A Twist's linear.y/linear.z/angular.x/
// angular.y have nowhere to go, and silently dropping them makes "the robot
// ignored my strafe command" very hard to diagnose. This class turns that into
// a rate-limited warning that includes the values.

#ifndef ROBOT_BASE__CMD_VEL_MONITOR_HPP_
#define ROBOT_BASE__CMD_VEL_MONITOR_HPP_

#include <optional>
#include <string>

namespace robot_base
{

class CmdVelMonitor
{
public:
  CmdVelMonitor(bool enabled, double warn_period_s, double threshold_m_s = 0.01)
  : enabled_(enabled), warn_period_s_(warn_period_s), threshold_(threshold_m_s)
  {
  }

  // Returns a warning message when one is due, otherwise std::nullopt.
  // `now_s` is a monotonic timestamp in seconds.
  std::optional<std::string> update(
    double linear_x, double linear_y, double linear_z,
    double angular_x, double angular_y, double angular_z,
    double now_s);

private:
  bool enabled_;
  double warn_period_s_;
  double threshold_;
  bool warned_{false};
  double last_warn_s_{0.0};
};

}  // namespace robot_base

#endif  // ROBOT_BASE__CMD_VEL_MONITOR_HPP_
