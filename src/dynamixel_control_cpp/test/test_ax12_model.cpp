#include "dynamixel_control_cpp/ax12_model.hpp"
#include "dynamixel_control_cpp/demo_motion.hpp"
#include "dynamixel_control_cpp/hold_command.hpp"
#include "dynamixel_control_cpp/monitor_display.hpp"

#include <gtest/gtest.h>

#include <sstream>
#include <vector>

using dynamixel_control_cpp::ax12::decode_load_percent;
using dynamixel_control_cpp::ax12::decode_present_speed;
using dynamixel_control_cpp::ax12::degrees_to_goal_position;
using dynamixel_control_cpp::ax12::parse_servo_id;
using dynamixel_control_cpp::ax12::raw_position_to_degrees;
using dynamixel_control_cpp::ax12::wheel_speed_to_raw;
using dynamixel_control_cpp::demo_angle_degrees;
using dynamixel_control_cpp::build_hold_command;
using dynamixel_control_cpp::MonitorServoDiagnostics;
using dynamixel_control_cpp::MonitorDashboard;
using dynamixel_control_cpp::MonitorServoDefinition;

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

TEST(Ax12Model, WheelSpeedToRaw)
{
  EXPECT_EQ(wheel_speed_to_raw(0.0), 0u);
  EXPECT_EQ(wheel_speed_to_raw(100.0), 100u);
  EXPECT_EQ(wheel_speed_to_raw(-100.0), static_cast<uint16_t>(0x400U | 100U));
}

TEST(Ax12Model, WheelSpeedToRawClampsMagnitude)
{
  EXPECT_EQ(wheel_speed_to_raw(2000.0), 1023u);
  EXPECT_EQ(wheel_speed_to_raw(-2000.0), static_cast<uint16_t>(0x400U | 1023U));
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

TEST(MonitorDisplay, ShowsConfiguredServoModesAndMissingData)
{
  sensor_msgs::msg::JointState msg;
  msg.name.push_back("servo_9");
  msg.position.push_back(42.0);
  msg.velocity.push_back(120.0);
  msg.effort.push_back(25.0);

  const std::vector<MonitorServoDefinition> servos = {
    {9, true},
    {10, false},
  };

  std::ostringstream output;
  int frame_count = 0;
  MonitorDashboard dashboard;
  dashboard.render_frame(output, msg, frame_count, servos);

  const auto text = output.str();
  EXPECT_NE(text.find("servo_9"), std::string::npos);
  EXPECT_NE(text.find("wheel"), std::string::npos);
  EXPECT_NE(text.find("servo_10"), std::string::npos);
  EXPECT_NE(text.find("joint"), std::string::npos);
  EXPECT_NE(text.find("no data"), std::string::npos);
}

TEST(MonitorDisplay, ShowsDiagnosticsTemperatureAndVoltage)
{
  sensor_msgs::msg::JointState msg;
  msg.name.push_back("servo_9");
  msg.position.push_back(120.0);
  msg.velocity.push_back(0.0);
  msg.effort.push_back(55.0);

  const std::vector<MonitorServoDefinition> servos = {
    {9, false},
  };
  const std::vector<MonitorServoDiagnostics> diagnostics = {
    {9, 48.0, 11.8, "OK"},
  };

  std::ostringstream output;
  int frame_count = 0;
  MonitorDashboard dashboard;
  dashboard.render_frame(output, msg, frame_count, servos, diagnostics);

  const auto text = output.str();
  EXPECT_NE(text.find("48.0 C"), std::string::npos);
  EXPECT_NE(text.find("11.8 V"), std::string::npos);
  EXPECT_NE(text.find("OK"), std::string::npos);
}

TEST(HoldCommand, BuildsPositionCommandForJointServosOnly)
{
  const auto msg = build_hold_command({9, 10}, {9}, 135.0);

  ASSERT_EQ(msg.name.size(), 1u);
  EXPECT_EQ(msg.name[0], "servo_10");
  ASSERT_EQ(msg.position.size(), 1u);
  EXPECT_DOUBLE_EQ(msg.position[0], 135.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
