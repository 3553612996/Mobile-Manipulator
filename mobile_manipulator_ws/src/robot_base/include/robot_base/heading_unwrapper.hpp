// The MCU wraps heading to (-180, 180] before reporting it (see
// service_chassis.c), so consecutive reports jump by ~358 degrees at the seam.
// Feeding that straight into /odom or TF makes an EKF see a full turn of
// rotation. This class integrates the shortest angular delta instead and keeps
// a continuous yaw.

#ifndef ROBOT_BASE__HEADING_UNWRAPPER_HPP_
#define ROBOT_BASE__HEADING_UNWRAPPER_HPP_

#include <cmath>
#include <cstdint>

namespace robot_base
{

class HeadingUnwrapper
{
public:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kCentiDegToRad = 0.01 * kPi / 180.0;

  // Returns the continuous heading in radians.
  double update(int16_t theta_centi_deg)
  {
    const double raw = static_cast<double>(theta_centi_deg) * kCentiDegToRad;
    if (!has_last_) {
      has_last_ = true;
      last_raw_rad_ = raw;
      continuous_rad_ = raw;
      return continuous_rad_;
    }

    const double delta = std::atan2(
      std::sin(raw - last_raw_rad_), std::cos(raw - last_raw_rad_));
    continuous_rad_ += delta;
    last_raw_rad_ = raw;
    return continuous_rad_;
  }

  void reset()
  {
    has_last_ = false;
    last_raw_rad_ = 0.0;
    continuous_rad_ = 0.0;
  }

  bool has_value() const {return has_last_;}
  double radians() const {return continuous_rad_;}

private:
  bool has_last_{false};
  double last_raw_rad_{0.0};
  double continuous_rad_{0.0};
};

}  // namespace robot_base

#endif  // ROBOT_BASE__HEADING_UNWRAPPER_HPP_
