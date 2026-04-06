// ============================================================================
// demo_motion.hpp — Trajectory generator for the demo node.
//
// This header computes sinusoidal motion patterns for testing servo movement.
// It has NO ROS 2 or hardware dependencies, so it can be unit-tested easily.
//
// HOW IT WORKS:
//   Each servo gets a different phase offset so they move in a wave pattern.
//   The formula for each servo at a given step is:
//
//     angle = 150° + 90° × sin(step + phase_offset)
//
//   This keeps all angles within the safe range of the AX-12A (60°–240°),
//   well inside the physical limits of 0°–300°.
//
//   The phase offset for servo i out of N servos is:
//     phase_offset = 2π × i / N
//
//   This spreads the servos evenly across one full sine cycle, creating
//   a smooth wave that looks like walking motion.
// ============================================================================

#ifndef DYNAMIXEL_CONTROL_CPP__DEMO_MOTION_HPP_
#define DYNAMIXEL_CONTROL_CPP__DEMO_MOTION_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

namespace dynamixel_control_cpp
{

/// Compute one servo's angle (in degrees) for a given step of the demo.
///
/// @param step         The current time step (increments each cycle).
/// @param servo_index  This servo's index in the array (0, 1, 2, ...).
/// @param servo_count  Total number of servos in the demo.
/// @return Angle in degrees, always between 60° and 240°.
inline double demo_angle_degrees(int step, std::size_t servo_index, std::size_t servo_count)
{
  // Safety: if there are no servos, return the center position.
  if (servo_count == 0) {
    return 150.0;
  }
  const double phase =
    2.0 * M_PI * static_cast<double>(servo_index) / static_cast<double>(servo_count);
  return 150.0 + 90.0 * std::sin(static_cast<double>(step) * 1.0 + phase);
}

/// Compute all servo positions for a given step.
///
/// @param step       The current time step.
/// @param servo_ids  Vector of servo IDs (used only for its size here).
/// @return Vector of angles in degrees, one per servo.
inline std::vector<double> demo_positions_for_step(
  int step,
  const std::vector<int> & servo_ids)
{
  std::vector<double> out;
  out.reserve(servo_ids.size());
  const auto n = servo_ids.size();
  for (std::size_t i = 0; i < n; ++i) {
    out.push_back(demo_angle_degrees(step, i, n));
  }
  return out;
}

}  // namespace dynamixel_control_cpp

#endif  // DYNAMIXEL_CONTROL_CPP__DEMO_MOTION_HPP_
