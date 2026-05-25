// ============================================================================
// monitor_node.cpp — Live terminal dashboard for servo state.
//
// WHAT IT DOES:
//   Subscribes to /joint_states and /servo_diagnostics and draws a color-coded
//   terminal display showing each servo's position, velocity, load, voltage,
//   and temperature in real time.
//
// HOW IT WORKS:
//   1. Creates a subscription to /joint_states.
//   2. Stores the newest message with minimal callback work.
//   3. Redraws the terminal at a configurable refresh rate using a timer.
//   4. The display logic lives in monitor_display.hpp, keeping this node
//      file small and focused on ROS 2 communication only.
//
// NOTE: This node has NO hardware access. It only reads from the topic.
//       You can feed it fake data for testing.
// ============================================================================

#include "dynamixel_control_cpp/monitor_display.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

class MonitorNode : public rclcpp::Node
{
public:
  MonitorNode()
  : Node("monitor_node"), frame_count_(0)
  {
    declare_parameter("joint_states_topic", "/joint_states");
    declare_parameter("servo_diagnostics_topic", "/servo_diagnostics");
    declare_parameter("servo_ids", std::vector<int64_t>{9, 10});
    declare_parameter("wheel_servo_ids", std::vector<int64_t>{9});
    declare_parameter("refresh_rate_hz", 10.0);

    const auto joint_states_topic = get_parameter("joint_states_topic").as_string();
    const auto diagnostics_topic = get_parameter("servo_diagnostics_topic").as_string();
    const auto servo_ids = get_parameter("servo_ids").as_integer_array();
    const auto wheel_servo_ids = get_parameter("wheel_servo_ids").as_integer_array();
    const auto refresh_rate_hz = std::max(1.0, get_parameter("refresh_rate_hz").as_double());

    for (auto id : servo_ids) {
      const auto servo_id = static_cast<int>(id);
      configured_servos_.push_back(
      {
        servo_id,
        is_wheel_servo(servo_id, wheel_servo_ids),
      });
    }

    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {latest_msg_ = *msg;});

    diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic, 10,
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        latest_diagnostics_ = parse_diagnostics(*msg);
      });

    const auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / refresh_rate_hz));
    timer_ = create_wall_timer(period_ms, [this]() {display();});

    RCLCPP_INFO(
      get_logger(), "Monitoring %zu servos from %s and %s at %.1f Hz",
      configured_servos_.size(), joint_states_topic.c_str(), diagnostics_topic.c_str(),
      refresh_rate_hz);
  }

private:
  void display()
  {
    dashboard_.render_frame(
      std::cout, latest_msg_, frame_count_, configured_servos_, latest_diagnostics_);
  }

  static bool is_wheel_servo(
    int servo_id,
    const std::vector<int64_t> & wheel_servo_ids)
  {
    return std::find(wheel_servo_ids.begin(), wheel_servo_ids.end(), servo_id) !=
           wheel_servo_ids.end();
  }

  static std::vector<dynamixel_control_cpp::MonitorServoDiagnostics> parse_diagnostics(
    const diagnostic_msgs::msg::DiagnosticArray & msg)
  {
    std::vector<dynamixel_control_cpp::MonitorServoDiagnostics> diagnostics;
    diagnostics.reserve(msg.status.size());

    for (const auto & status : msg.status) {
      const int id = read_int_value(status.values, "id", -1);
      if (id < 0) {
        continue;
      }

      diagnostics.push_back(
      {
        id,
        read_double_value(status.values, "temperature_c", 0.0),
        read_double_value(status.values, "voltage_v", 0.0),
        status.message,
      });
    }
    return diagnostics;
  }

  static double read_double_value(
    const std::vector<diagnostic_msgs::msg::KeyValue> & values,
    const std::string & key,
    double default_value)
  {
    for (const auto & value : values) {
      if (value.key == key) {
        return std::atof(value.value.c_str());
      }
    }
    return default_value;
  }

  static int read_int_value(
    const std::vector<diagnostic_msgs::msg::KeyValue> & values,
    const std::string & key,
    int default_value)
  {
    return static_cast<int>(read_double_value(values, key, default_value));
  }

  dynamixel_control_cpp::MonitorDashboard dashboard_;
  std::vector<dynamixel_control_cpp::MonitorServoDefinition> configured_servos_;
  std::vector<dynamixel_control_cpp::MonitorServoDiagnostics> latest_diagnostics_;
  sensor_msgs::msg::JointState latest_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int frame_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "\033[2J\033[H";
  rclcpp::spin(std::make_shared<MonitorNode>());
  rclcpp::shutdown();
  return 0;
}
