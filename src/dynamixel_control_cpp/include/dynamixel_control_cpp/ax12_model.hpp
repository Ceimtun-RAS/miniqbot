// ============================================================================
// ax12_model.hpp — Pure math and data helpers for the Dynamixel AX-12A servo.
//
// This header has NO ROS 2 or hardware dependencies. It only contains
// constants, conversion functions, and parsing utilities that translate
// between the raw register values the AX-12A uses and the human-readable
// units (degrees, percentages, signed speed) that the rest of the code uses.
//
// WHY it exists:
//   The Henki best-practices guide says "separate application logic into its
//   own class or library, keeping ROS 2 nodes focused solely on communication."
//   By putting all AX-12 math here, we can unit-test it without hardware
//   and keep the ROS 2 nodes small and focused.
//
// HOW the AX-12A encodes data (Protocol 1.0):
//   - Position:  10 bits (0–1023) representing 0–300 degrees.
//   - Speed:     10-bit magnitude, bit 10 is direction (0 = CCW, 1 = CW).
//   - Load:      Same layout as speed; represents torque as % of max.
//   - Registers are documented in the Robotis e-Manual:
//     https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
// ============================================================================

#ifndef DYNAMIXEL_CONTROL_CPP__AX12_MODEL_HPP_
#define DYNAMIXEL_CONTROL_CPP__AX12_MODEL_HPP_

#include <algorithm>
#include <cstdint>
#include <string>

namespace dynamixel_control_cpp
{
namespace ax12
{

// ---------------------------------------------------------------------------
// Register addresses (AX-12A control table, Protocol 1.0).
//
// Each constant is the starting byte address you pass to the Dynamixel SDK's
// read/write functions. For example, to read the current position you call:
//   packet_handler->read2ByteTxRx(port, id, reg::PRESENT_POSITION, &val, &err);
//
// The full table is in the Robotis e-Manual (link above).
// ---------------------------------------------------------------------------
namespace reg
{
constexpr uint8_t CW_ANGLE_LIMIT = 6;       // clockwise angle limit (2 bytes)
constexpr uint8_t CCW_ANGLE_LIMIT = 8;      // counter-clockwise angle limit (2 bytes)
constexpr uint8_t TORQUE_ENABLE = 24;       // 1 = motor on, 0 = motor off
constexpr uint8_t CW_COMPLIANCE_MARGIN = 26;   // position dead-band, CW side
constexpr uint8_t CCW_COMPLIANCE_MARGIN = 27;  // position dead-band, CCW side
constexpr uint8_t CW_COMPLIANCE_SLOPE = 28;    // how aggressively the motor corrects, CW
constexpr uint8_t CCW_COMPLIANCE_SLOPE = 29;   // how aggressively the motor corrects, CCW
constexpr uint8_t GOAL_POSITION = 30;       // target position (2 bytes, 0–1023)
constexpr uint8_t PRESENT_POSITION = 36;    // current position (2 bytes, 0–1023)
constexpr uint8_t PRESENT_SPEED = 38;       // current speed (2 bytes, see encoding above)
constexpr uint8_t PRESENT_LOAD = 40;        // current load (2 bytes, see encoding above)
constexpr uint8_t PRESENT_VOLTAGE = 42;     // supply voltage (1 byte, unit: 0.1 V)
constexpr uint8_t PRESENT_TEMP = 43;        // internal temperature (1 byte, unit: 1 °C)
}  // namespace reg

// ---------------------------------------------------------------------------
// Conversion constants.
//
// The AX-12A maps 0–1023 raw units to 0–300 degrees.
//   degrees = raw * (300 / 1023)
//   raw     = degrees * (1023 / 300)
// ---------------------------------------------------------------------------
constexpr double POS_TO_DEG = 300.0 / 1023.0;
constexpr double DEG_TO_POS = 1023.0 / 300.0;

// ---------------------------------------------------------------------------
// Conversion functions.
// ---------------------------------------------------------------------------

/// Convert a raw position register value (0–1023) to degrees (0–300).
inline double raw_position_to_degrees(uint16_t raw_pos)
{
  return static_cast<double>(raw_pos) * POS_TO_DEG;
}

/// Convert degrees to a raw goal position (0–1023).
/// Values outside [0, 300] are clamped so the servo never receives an
/// out-of-range command.
inline uint16_t degrees_to_goal_position(double degrees)
{
  const double raw = degrees * DEG_TO_POS;
  const auto clamped = std::clamp(raw, 0.0, 1023.0);
  return static_cast<uint16_t>(clamped + 0.5);   // round to nearest integer
}

/// Decode the AX-12A "present speed" register.
///
/// The raw value is 11 bits:
///   - Bits 0–9 (0x3FF): speed magnitude
///   - Bit 10   (0x400): direction flag (1 = clockwise → negative by convention)
///
/// Returns a signed double so positive = CCW and negative = CW.
inline double decode_present_speed(uint16_t raw_speed)
{
  int speed_val = static_cast<int>(raw_speed & 0x3FFU);
  if (raw_speed & 0x400U) {
    speed_val = -speed_val;
  }
  return static_cast<double>(speed_val);
}

/// Decode the AX-12A "present load" register as a signed percentage (0–100%).
///
/// Layout is the same as speed: 10-bit magnitude + direction bit.
/// The result is scaled to [0, 100] so callers don't need to know the raw range.
/// Positive = CW load, negative = CCW load.
inline double decode_load_percent(uint16_t raw_load)
{
  double load_pct = (raw_load & 0x3FFU) * 100.0 / 1023.0;
  if (raw_load & 0x400U) {
    load_pct = -load_pct;
  }
  return load_pct;
}

/// Parse a servo name like "servo_9" and return the numeric ID (9).
/// Returns -1 if the name doesn't match the expected format.
inline int parse_servo_id(const std::string & name)
{
  const auto pos = name.rfind('_');
  if (pos == std::string::npos) {
    return -1;
  }
  try {
    return std::stoi(name.substr(pos + 1));
  } catch (...) {
    return -1;
  }
}

}  // namespace ax12
}  // namespace dynamixel_control_cpp

#endif  // DYNAMIXEL_CONTROL_CPP__AX12_MODEL_HPP_
