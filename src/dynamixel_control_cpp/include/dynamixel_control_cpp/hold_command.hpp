// ============================================================================
// hold_command.hpp — Pure helper for static hold-test commands.
//
// This helper builds a JointState command for joint-mode servos only. Wheel
// servos are intentionally skipped because holding a wheel-mode servo at an
// angle is not meaningful.
// ============================================================================

#ifndef DYNAMIXEL_CONTROL_CPP__HOLD_COMMAND_HPP_
#define DYNAMIXEL_CONTROL_CPP__HOLD_COMMAND_HPP_

#include "sensor_msgs/msg/joint_state.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace dynamixel_control_cpp
{

inline bool contains_id(const std::vector<int> & ids, int id)
{
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

inline sensor_msgs::msg::JointState build_hold_command(
  const std::vector<int> & servo_ids,
  const std::vector<int> & wheel_servo_ids,
  double hold_position_degrees)
{
  auto msg = sensor_msgs::msg::JointState();

  for (int id : servo_ids) {
    if (contains_id(wheel_servo_ids, id)) {
      continue;
    }
    msg.name.push_back("servo_" + std::to_string(id));
    msg.position.push_back(hold_position_degrees);
    msg.velocity.push_back(0.0);
  }

  return msg;
}

}  // namespace dynamixel_control_cpp

#endif  // DYNAMIXEL_CONTROL_CPP__HOLD_COMMAND_HPP_
