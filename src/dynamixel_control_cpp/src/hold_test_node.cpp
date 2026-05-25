// ============================================================================
// hold_test_node.cpp — Static load test for joint-mode servos.
//
// This node publishes a fixed position command on /joint_commands so the
// monitor can show how load, temperature, and voltage respond while the robot
// holds a posture. It never talks to hardware directly.
// ============================================================================

#include "dynamixel_control_cpp/hold_command.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>

class HoldTestNode : public rclcpp::Node
{
public:
  HoldTestNode()
  : Node("hold_test_node")
  {
    declare_parameter("servo_ids", std::vector<int64_t>{9, 10});
    declare_parameter("wheel_servo_ids", std::vector<int64_t>{9});
    declare_parameter("hold_position_deg", 150.0);
    declare_parameter("publish_rate_hz", 2.0);

    const auto servo_ids_param = get_parameter("servo_ids").as_integer_array();
    const auto wheel_servo_ids_param = get_parameter("wheel_servo_ids").as_integer_array();
    hold_position_deg_ = get_parameter("hold_position_deg").as_double();
    const auto publish_rate_hz = std::max(0.5, get_parameter("publish_rate_hz").as_double());

    servo_ids_ = to_int_vector(servo_ids_param);
    wheel_servo_ids_ = to_int_vector(wheel_servo_ids_param);

    cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz)),
      [this]() {publish_hold_command();});

    RCLCPP_INFO(
      get_logger(), "Holding joint servos at %.1f degrees at %.1f Hz",
      hold_position_deg_, publish_rate_hz);
  }

private:
  void publish_hold_command()
  {
    auto msg = dynamixel_control_cpp::build_hold_command(
      servo_ids_, wheel_servo_ids_, hold_position_deg_);
    msg.header.stamp = now();

    if (msg.name.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No joint-mode servos are configured for hold testing.");
      return;
    }

    cmd_pub_->publish(msg);
  }

  static std::vector<int> to_int_vector(const std::vector<int64_t> & values)
  {
    std::vector<int> result;
    result.reserve(values.size());
    for (auto value : values) {
      result.push_back(static_cast<int>(value));
    }
    return result;
  }

  std::vector<int> servo_ids_;
  std::vector<int> wheel_servo_ids_;
  double hold_position_deg_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HoldTestNode>());
  rclcpp::shutdown();
  return 0;
}
