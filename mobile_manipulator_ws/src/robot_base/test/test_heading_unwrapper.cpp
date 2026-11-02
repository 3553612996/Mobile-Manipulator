// The firmware reports heading in (-180, 180]. The bridge must turn that into
// a continuous yaw before it reaches /odom or TF.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "robot_base/heading_unwrapper.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

double deg_to_rad(double deg)
{
  return deg * kPi / 180.0;
}

int16_t wrap_centi_deg(double deg)
{
  // Same range as the firmware: (-180, 180].
  return static_cast<int16_t>(std::lround(std::remainder(deg, 360.0) * 100.0));
}

}  // namespace

TEST(HeadingUnwrapper, InitialValueIsTheRawHeading)
{
  robot_base::HeadingUnwrapper unwrapper;
  EXPECT_NEAR(unwrapper.update(17900), deg_to_rad(179.0), 1e-9);
  EXPECT_TRUE(unwrapper.has_value());
}

TEST(HeadingUnwrapper, CrossingPositiveSeamGoesTo181)
{
  robot_base::HeadingUnwrapper unwrapper;
  EXPECT_NEAR(unwrapper.update(17900), deg_to_rad(179.0), 1e-9);
  // The MCU now reports -179; the shortest delta is +2 degrees, not -358.
  EXPECT_NEAR(unwrapper.update(-17900), deg_to_rad(181.0), 1e-9);
}

TEST(HeadingUnwrapper, CrossingNegativeSeamGoesToMinus181)
{
  robot_base::HeadingUnwrapper unwrapper;
  EXPECT_NEAR(unwrapper.update(-17900), deg_to_rad(-179.0), 1e-9);
  EXPECT_NEAR(unwrapper.update(17900), deg_to_rad(-181.0), 1e-9);
}

TEST(HeadingUnwrapper, ThreeFullTurnsStayContinuous)
{
  robot_base::HeadingUnwrapper unwrapper;
  for (int degrees = 0; degrees <= 1080; degrees += 10) {
    unwrapper.update(wrap_centi_deg(static_cast<double>(degrees)));
  }
  EXPECT_NEAR(unwrapper.radians(), deg_to_rad(1080.0), 1e-6);
}

TEST(HeadingUnwrapper, ResetDropsHistory)
{
  robot_base::HeadingUnwrapper unwrapper;
  unwrapper.update(17900);
  unwrapper.update(-17900);
  unwrapper.reset();
  EXPECT_FALSE(unwrapper.has_value());
  // After a reset the first report seeds the value again, exactly like boot.
  EXPECT_NEAR(unwrapper.update(-17900), deg_to_rad(-179.0), 1e-9);
}
