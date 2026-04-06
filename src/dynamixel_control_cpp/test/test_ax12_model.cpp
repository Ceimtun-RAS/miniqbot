#include "dynamixel_control_cpp/ax12_model.hpp"
#include "dynamixel_control_cpp/demo_motion.hpp"
#include "dynamixel_control_cpp/monitor_display.hpp"

#include <gtest/gtest.h>

using dynamixel_control_cpp::ax12::decode_load_percent;
using dynamixel_control_cpp::ax12::decode_present_speed;
using dynamixel_control_cpp::ax12::degrees_to_goal_position;
using dynamixel_control_cpp::ax12::parse_servo_id;
using dynamixel_control_cpp::ax12::raw_position_to_degrees;
using dynamixel_control_cpp::demo_angle_degrees;
using dynamixel_control_cpp::MonitorDashboard;

TEST(Ax12Model, RawPositionToDegrees)
{
  EXPECT_NEAR(raw_position_to_degrees(512), 512.0 * 300.0 / 1023.0, 1e-6);
  EXPECT_NEAR(raw_position_to_degrees(0), 0.0, 1e-9);
}

TEST(Ax12Model, DegreesToGoalClamps)
{
  EXPECT_EQ(degrees_to_goal_position(0.0), 0u);
  EXPECT_EQ(degrees_to_goal_position(300.0), 1023u);
  EXPECT_EQ(degrees_to_goal_position(-10.0), 0u);
  EXPECT_EQ(degrees_to_goal_position(400.0), 1023u);
}

TEST(Ax12Model, DecodeSpeedSign)
{
  EXPECT_DOUBLE_EQ(decode_present_speed(0x000), 0.0);
  EXPECT_DOUBLE_EQ(decode_present_speed(0x3FF), static_cast<double>(0x3FF));
  EXPECT_DOUBLE_EQ(decode_present_speed(0x400 | 100), -100.0);
}

TEST(Ax12Model, DecodeLoadPercentSign)
{
  EXPECT_NEAR(decode_load_percent(0x200), (512.0 * 100.0 / 1023.0), 1e-6);
  const uint16_t neg_200 = static_cast<uint16_t>(0x400U | 200U);
  EXPECT_NEAR(decode_load_percent(neg_200), -(200.0 * 100.0 / 1023.0), 1e-6);
}

TEST(Ax12Model, ParseServoId)
{
  EXPECT_EQ(parse_servo_id("servo_9"), 9);
  EXPECT_EQ(parse_servo_id("prefix_servo_12"), 12);
  EXPECT_EQ(parse_servo_id("bad"), -1);
}

TEST(DemoMotion, AngleRange)
{
  const double a = demo_angle_degrees(0, 0, 3);
  EXPECT_GE(a, 60.0);
  EXPECT_LE(a, 240.0);
}

TEST(MonitorDisplay, BarAndLabel)
{
  EXPECT_FALSE(MonitorDashboard::bar(10.0, 10).empty());
  EXPECT_NE(MonitorDashboard::load_label(80.0), MonitorDashboard::load_label(10.0));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
