// ============================================================================
// demo_node.cpp — Publishes sinusoidal motion commands for testing servos.
//
// WHAT IT DOES:
//   Generates a smooth wave pattern and publishes goal positions on the
//   /joint_commands topic. The dynamixel_node picks these up and moves
//   the physical servos.
//
// HOW IT WORKS:
//   1. On startup, reads parameters (servo IDs, step count, period).
//   2. Every N seconds (default 3), computes the next set of angles using
//      the formula in demo_motion.hpp and publishes them.
//   3. After all steps are done, shuts down automatically.
//
// PARAMETERS (see config/demo_node.yaml):
//   - servo_ids:        Which servos to command (default: [9, 10])
//   - wheel_servo_ids:  Which servos receive velocity commands (default: [9])
//   - wheel_speed_raw:  AX-12 raw wheel speed magnitude (default: 200)
//   - total_steps:      How many moves before stopping (default: 10)
//   - timer_period_sec: Seconds between moves (default: 3)
// ============================================================================

#include "dynamixel_control_cpp/demo_motion.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using dynamixel_control_cpp::demo_angle_degrees;

class DemoNode : public rclcpp::Node
{
public:
  DemoNode()
  : Node("demo_node"), step_(0)
  {
    declare_parameter("servo_ids", std::vector<int64_t>{9, 10});
    declare_parameter("wheel_servo_ids", std::vector<int64_t>{9});
    declare_parameter("wheel_speed_raw", 200);
    declare_parameter("total_steps", 10);
    declare_parameter("timer_period_sec", 3);

    const auto ids_param = get_parameter("servo_ids").as_integer_array();
    const auto wheel_ids_param = get_parameter("wheel_servo_ids").as_integer_array();
    wheel_speed_raw_ = get_parameter("wheel_speed_raw").as_int();
    total_steps_ = get_parameter("total_steps").as_int();
    const int period_sec = get_parameter("timer_period_sec").as_int();

    for (auto id : ids_param) {
      servo_ids_.push_back(static_cast<int>(id));
    }
    for (auto id : wheel_ids_param) {
      wheel_servo_ids_.push_back(static_cast<int>(id));
    }

    cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);
    timer_ = create_wall_timer(
      std::chrono::seconds(period_sec > 0 ? period_sec : 3),
      [this]() {next_move();});

    RCLCPP_INFO(
      get_logger(), "Demo: %zu servos, %ld steps, moving every %ds",
      servo_ids_.size(), total_steps_, period_sec > 0 ? period_sec : 3);
  }

private:
  void next_move()
  {
    if (step_ >= total_steps_) {
      publish_stop_command();
      RCLCPP_INFO(get_logger(), "Demo finished.");
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();

    const auto n = servo_ids_.size();
    for (size_t i = 0; i < n; i++) {
      const double angle = demo_angle_degrees(step_, i, n);
      const double wheel_speed = (step_ % 2 == 0) ? wheel_speed_raw_ : -wheel_speed_raw_;
      msg.name.push_back("servo_" + std::to_string(servo_ids_[i]));
      msg.position.push_back(is_wheel_servo(servo_ids_[i]) ? 0.0 : angle);
      msg.velocity.push_back(is_wheel_servo(servo_ids_[i]) ? wheel_speed : 0.0);
    }

    std::string commands;
    for (size_t i = 0; i < msg.name.size(); i++) {
      if (i > 0) {
        commands += ", ";
      }
      if (is_wheel_servo(servo_ids_[i])) {
        commands += msg.name[i] + "=" + std::to_string(static_cast<int>(msg.velocity[i])) + " raw";
      } else {
        commands += msg.name[i] + "=" + std::to_string(static_cast<int>(msg.position[i])) +
          "\u00b0";
      }
    }
    RCLCPP_INFO(
      get_logger(), "Step %d/%ld: [%s]", step_ + 1, total_steps_, commands.c_str());

    cmd_pub_->publish(msg);
    step_++;
  }

  void publish_stop_command()
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();

    for (int id : servo_ids_) {
      if (!is_wheel_servo(id)) {
        continue;
      }
      msg.name.push_back("servo_" + std::to_string(id));
      msg.position.push_back(0.0);
      msg.velocity.push_back(0.0);
    }

    if (!msg.name.empty()) {
      cmd_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "Sent stop command to wheel servos.");
    }
  }

  bool is_wheel_servo(int id) const
  {
    return std::find(wheel_servo_ids_.begin(), wheel_servo_ids_.end(), id) !=
           wheel_servo_ids_.end();
  }

  std::vector<int> servo_ids_;
  std::vector<int> wheel_servo_ids_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int wheel_speed_raw_;
  int step_;
  int64_t total_steps_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoNode>());
  rclcpp::shutdown();
  return 0;
}
