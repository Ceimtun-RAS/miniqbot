// ============================================================================
// monitor_node.cpp — Live terminal dashboard for servo state.
//
// WHAT IT DOES:
//   Subscribes to /joint_states and draws a color-coded terminal display
//   showing each servo's position, velocity, and load in real time.
//
// HOW IT WORKS:
//   1. Creates a subscription to /joint_states.
//   2. Every time a message arrives, calls MonitorDashboard::render_frame()
//      which redraws the terminal in-place (using ANSI cursor movement).
//   3. The display logic lives in monitor_display.hpp, keeping this node
//      file small and focused on ROS 2 communication only.
//
// NOTE: This node has NO hardware access. It only reads from the topic.
//       You can feed it fake data for testing.
// ============================================================================

#include "dynamixel_control_cpp/monitor_display.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <iostream>
#include <memory>

class MonitorNode : public rclcpp::Node
{
public:
  MonitorNode()
  : Node("monitor_node"), frame_count_(0)
  {
    declare_parameter("joint_states_topic", "/joint_states");
    const auto topic = get_parameter("joint_states_topic").as_string();

    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      topic, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {display(msg);});

    RCLCPP_INFO(get_logger(), "Waiting for data on %s ...", topic.c_str());
  }

private:
  void display(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    dashboard_.render_frame(std::cout, *msg, frame_count_);
  }

  dynamixel_control_cpp::MonitorDashboard dashboard_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
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
