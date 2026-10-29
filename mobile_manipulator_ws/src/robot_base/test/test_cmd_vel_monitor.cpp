// SET_VELOCITY has no lateral or roll/pitch channel. The warning must say so
// with the offending values, but at most once per period: cmd_vel runs at
// 50 Hz and an unthrottled log would drown everything else.

#include <gtest/gtest.h>

#include "robot_base/cmd_vel_monitor.hpp"

TEST(CmdVelMonitor, SupportedComponentsDoNotWarn)
{
  robot_base::CmdVelMonitor monitor(true, 5.0);
  EXPECT_FALSE(monitor.update(0.2, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0).has_value());
}

TEST(CmdVelMonitor, LinearYWarnsOncePerPeriod)
{
  robot_base::CmdVelMonitor monitor(true, 5.0);

  const auto first = monitor.update(0.2, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(first.has_value());
  EXPECT_NE(first->find("linear.y=0.3"), std::string::npos);

  EXPECT_FALSE(monitor.update(0.2, 0.3, 0.0, 0.0, 0.0, 0.0, 1.0).has_value());
  EXPECT_FALSE(monitor.update(0.2, 0.3, 0.0, 0.0, 0.0, 0.0, 4.9).has_value());
  EXPECT_TRUE(monitor.update(0.2, 0.3, 0.0, 0.0, 0.0, 0.0, 5.0).has_value());
}

TEST(CmdVelMonitor, BelowThresholdIsIgnored)
{
  robot_base::CmdVelMonitor monitor(true, 5.0, 0.01);
  EXPECT_FALSE(monitor.update(0.2, 0.005, -0.009, 0.0, 0.0, 0.0, 0.0).has_value());
}

TEST(CmdVelMonitor, AngularXAndLinearZAreReported)
{
  robot_base::CmdVelMonitor monitor(true, 5.0);
  const auto warning = monitor.update(0.0, 0.0, 0.1, 0.2, 0.0, 0.0, 0.0);
  ASSERT_TRUE(warning.has_value());
  EXPECT_NE(warning->find("linear.z=0.1"), std::string::npos);
  EXPECT_NE(warning->find("angular.x=0.2"), std::string::npos);
}

TEST(CmdVelMonitor, DisabledMonitorNeverWarns)
{
  robot_base::CmdVelMonitor monitor(false, 5.0);
  EXPECT_FALSE(monitor.update(0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0).has_value());
  EXPECT_FALSE(monitor.update(0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 10.0).has_value());
}
