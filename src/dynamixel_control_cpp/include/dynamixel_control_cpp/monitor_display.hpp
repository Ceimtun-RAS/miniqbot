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
#include <string>

namespace dynamixel_control_cpp
{

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
    const size_t n = msg.name.size();
    if (n == 0) {
      return;
    }

    // After the first frame, move cursor up to overwrite the previous output.
    // The magic number: 3 header lines + 5 lines per servo.
    if (frame_count > 0) {
      os << "\033[" << (3 + n * 5) << "A";
    }
    frame_count++;

    // --- Header ---
    os << "\033[36m  ╔═══════════════════════════════════════════════════════════╗\033[0m\n"
       << "\033[36m  ║    DYNAMIXEL ROBOT MONITOR  (" << n << " servos)       frame "
       << std::setw(5) << frame_count << "  ║\033[0m\n"
       << "\033[36m  ╚═══════════════════════════════════════════════════════════╝\033[0m\n";

    // --- One block per servo ---
    for (size_t i = 0; i < n; i++) {
      // Safely read each field (the arrays might have different sizes).
      const double pos = (i < msg.position.size()) ? msg.position[i] : 0.0;
      const double vel = (i < msg.velocity.size()) ? msg.velocity[i] : 0.0;
      const double load = (i < msg.effort.size()) ? msg.effort[i] : 0.0;
      const double abs_load = std::abs(load);
      const char * dir = (load >= 0) ? "CW " : "CCW";

      os << "  \033[1;37m── " << std::setw(10) << std::left << msg.name[i]
         << " ───────────────────────────────────────────────\033[0m\n"
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
};

}  // namespace dynamixel_control_cpp

#endif  // DYNAMIXEL_CONTROL_CPP__MONITOR_DISPLAY_HPP_
