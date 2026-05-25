// ============================================================================
// monitor_display.hpp — Terminal dashboard renderer for servo state.
//
// This class takes a JointState message and draws a color-coded terminal UI
// showing each servo's position, velocity, and load. It uses ANSI escape
// codes for colors and cursor movement (works in most Linux terminals).
//
// WHY it uses stdout instead of the ROS 2 logger:
//   The ROS 2 logger adds timestamps and prefixes to every line, which makes
//   it impossible to draw a formatted dashboard that overwrites itself each
//   frame. So we use std::cout directly for the display, while the node
//   itself uses RCLCPP_INFO for system messages.
//
// HOW the display works:
//   1. On the first frame, it prints the header and all servo lines.
//   2. On subsequent frames, it moves the cursor UP to the start of the
//      header and redraws everything, creating an in-place animation.
//   3. Load bars are color-coded: green (< 40%), yellow (40–75%), red (> 75%).
// ============================================================================

#ifndef DYNAMIXEL_CONTROL_CPP__MONITOR_DISPLAY_HPP_
#define DYNAMIXEL_CONTROL_CPP__MONITOR_DISPLAY_HPP_

#include "sensor_msgs/msg/joint_state.hpp"

#include <cmath>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace dynamixel_control_cpp
{

/// Static monitor metadata for one configured servo.
struct MonitorServoDefinition
{
  int id;
  bool wheel_mode;
};

/// Optional telemetry published outside JointState.
struct MonitorServoDiagnostics
{
  int id;
  double temperature_c;
  double voltage_v;
  std::string status;
};

/// Renders a live terminal dashboard for JointState messages.
class MonitorDashboard
{
public:
  /// Build a text progress bar for a given percentage.
  ///
  /// @param percent  Value to display (0–100, can be negative for direction).
  /// @param width    Character width of the bar.
  /// @return A string with ANSI color codes embedded.
  static std::string bar(double percent, int width = 30)
  {
    const double abs_pct = std::abs(percent);
    int filled = static_cast<int>(abs_pct / 100.0 * width);
    if (filled > width) {
      filled = width;
    }

    std::string result;
    for (int i = 0; i < width; i++) {
      if (i < filled) {
        // Color depends on how high the load is:
        //   > 75% = red (danger), > 40% = yellow (caution), else green (ok)
        if (abs_pct > 75) {
          result += "\033[31m#\033[0m";     // red
        } else if (abs_pct > 40) {
          result += "\033[33m#\033[0m";     // yellow
        } else {
          result += "\033[32m#\033[0m";     // green
        }
      } else {
        result += "\033[90m-\033[0m";       // dim gray for empty portion
      }
    }
    return result;
  }

  /// Return a colored text label describing the load level.
  static const char * load_label(double abs_pct)
  {
    if (abs_pct > 75) {
      return "\033[31m HIGH!\033[0m";       // red: high load warning
    }
    if (abs_pct > 40) {
      return "\033[33m  med \033[0m";       // yellow: moderate load
    }
    return "\033[32m  low \033[0m";         // green: low load
  }

  /// Draw one complete frame of the dashboard to the given output stream.
  ///
  /// @param os           Output stream (typically std::cout).
  /// @param msg          The JointState message to display.
  /// @param frame_count  Tracks how many frames have been drawn. The method
  ///                     uses this to know whether to move the cursor up
  ///                     (overwriting the previous frame) or just print fresh.
  void render_frame(
    std::ostream & os,
    const sensor_msgs::msg::JointState & msg,
    int & frame_count)
  {
    render_frame(os, msg, frame_count, {});
  }

  /// Draw one complete frame using a configured servo list.
  ///
  /// When configured servos are provided, the dashboard preserves that order and
  /// marks servos that have not appeared in the latest JointState message yet.
  void render_frame(
    std::ostream & os,
    const sensor_msgs::msg::JointState & msg,
    int & frame_count,
    const std::vector<MonitorServoDefinition> & configured_servos)
  {
    render_frame(os, msg, frame_count, configured_servos, {});
  }

  /// Draw one complete frame with configured servos and optional diagnostics.
  void render_frame(
    std::ostream & os,
    const sensor_msgs::msg::JointState & msg,
    int & frame_count,
    const std::vector<MonitorServoDefinition> & configured_servos,
    const std::vector<MonitorServoDiagnostics> & diagnostics)
  {
    const auto rows = build_rows(msg, configured_servos, diagnostics);
    const size_t n = rows.size();
    if (n == 0) {
      return;
    }

    // After the first frame, move cursor up to overwrite the previous output.
    // The magic number: 3 header lines + 6 lines per servo.
    if (frame_count > 0) {
      os << "\033[" << (3 + n * 6) << "A";
    }
    frame_count++;

    // --- Header ---
    os << "\033[36m  ╔═══════════════════════════════════════════════════════════╗\033[0m\n"
       << "\033[36m  ║    DYNAMIXEL ROBOT MONITOR  (" << n << " servos)       frame "
       << std::setw(5) << frame_count << "  ║\033[0m\n"
       << "\033[36m  ╚═══════════════════════════════════════════════════════════╝\033[0m\n";

    // --- One block per servo ---
    for (const auto & row : rows) {
      const double pos = row.has_position ? row.position : 0.0;
      const double vel = row.has_velocity ? row.velocity : 0.0;
      const double load = row.has_effort ? row.effort : 0.0;
      const double abs_load = std::abs(load);
      const char * dir = (load >= 0) ? "CW " : "CCW";

      os << "  \033[1;37m── " << std::setw(10) << std::left << row.name
         << " ───────────────────────────────────────────────\033[0m\n"
         << "    Mode: " << std::setw(5) << std::left << row.mode
         << "    Data: " << row.status
         << "    Health: " << row.diagnostics_status
         << "    Temp: " << std::fixed << std::setprecision(1)
         << std::setw(4) << row.temperature_c << " C"
         << "    Volt: " << std::setw(4) << row.voltage_v << " V     \n"
         << "    Pos: " << std::fixed << std::setprecision(1)
         << std::setw(6) << std::right << pos << "\u00b0"
         << "    Vel: " << std::setprecision(0) << std::setw(5) << vel
         << "                                \n"
         << "    Load: " << std::setprecision(1) << std::setw(5) << abs_load
         << "% " << dir << " [" << bar(load) << "] " << load_label(abs_load)
         << "     \n\n\n";
    }

    os << std::flush;
  }

private:
  struct ServoRow
  {
    std::string name;
    const char * mode;
    const char * status;
    std::string diagnostics_status;
    double position;
    double velocity;
    double effort;
    double temperature_c;
    double voltage_v;
    bool has_position;
    bool has_velocity;
    bool has_effort;
  };

  static std::vector<ServoRow> build_rows(
    const sensor_msgs::msg::JointState & msg,
    const std::vector<MonitorServoDefinition> & configured_servos,
    const std::vector<MonitorServoDiagnostics> & diagnostics)
  {
    std::vector<ServoRow> rows;
    if (configured_servos.empty()) {
      rows.reserve(msg.name.size());
      for (size_t i = 0; i < msg.name.size(); i++) {
        rows.push_back(make_row(msg, i, "auto", diagnostics));
      }
      return rows;
    }

    rows.reserve(configured_servos.size());
    for (const auto & servo : configured_servos) {
      const auto name = "servo_" + std::to_string(servo.id);
      const auto index = find_joint_index(msg, name);
      if (index < msg.name.size()) {
        rows.push_back(make_row(msg, index, servo.wheel_mode ? "wheel" : "joint", diagnostics));
      } else {
        const auto diagnostic = find_diagnostics(diagnostics, servo.id);
        rows.push_back(
          {
            name,
            servo.wheel_mode ? "wheel" : "joint",
            "\033[33mno data\033[0m",
            diagnostic ? diagnostic->status : "unknown",
            0.0,
            0.0,
            0.0,
            diagnostic ? diagnostic->temperature_c : 0.0,
            diagnostic ? diagnostic->voltage_v : 0.0,
            false,
            false,
            false,
          });
      }
    }
    return rows;
  }

  static ServoRow make_row(
    const sensor_msgs::msg::JointState & msg,
    size_t index,
    const char * mode,
    const std::vector<MonitorServoDiagnostics> & diagnostics)
  {
    const bool has_position = index < msg.position.size();
    const bool has_velocity = index < msg.velocity.size();
    const bool has_effort = index < msg.effort.size();
    const int servo_id = parse_servo_id(msg.name[index]);
    const auto diagnostic = find_diagnostics(diagnostics, servo_id);
    return {
      msg.name[index],
      mode,
      "\033[32mactive\033[0m",
      diagnostic ? diagnostic->status : "unknown",
      has_position ? msg.position[index] : 0.0,
      has_velocity ? msg.velocity[index] : 0.0,
      has_effort ? msg.effort[index] : 0.0,
      diagnostic ? diagnostic->temperature_c : 0.0,
      diagnostic ? diagnostic->voltage_v : 0.0,
      has_position,
      has_velocity,
      has_effort,
    };
  }

  static size_t find_joint_index(
    const sensor_msgs::msg::JointState & msg,
    const std::string & name)
  {
    for (size_t i = 0; i < msg.name.size(); i++) {
      if (msg.name[i] == name) {
        return i;
      }
    }
    return msg.name.size();
  }

  static int parse_servo_id(const std::string & name)
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

  static const MonitorServoDiagnostics * find_diagnostics(
    const std::vector<MonitorServoDiagnostics> & diagnostics,
    int id)
  {
    for (const auto & item : diagnostics) {
      if (item.id == id) {
        return &item;
      }
    }
    return nullptr;
  }
};

}  // namespace dynamixel_control_cpp

#endif  // DYNAMIXEL_CONTROL_CPP__MONITOR_DISPLAY_HPP_
