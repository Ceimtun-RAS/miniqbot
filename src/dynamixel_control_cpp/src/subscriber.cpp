// ============================================================================
// subscriber.cpp — Minimal ROS 2 subscriber example.
//
// This is a learning example, not part of the robot's control system.
// It listens on the "chatter" topic and prints every message it receives.
//
// Run this in one terminal and publisher.cpp in another to see them
// communicate. See docs/ros2_tutorial.md for a full walkthrough.
//
// HOW IT WORKS:
//   1. Creates a subscription on the "chatter" topic.
//   2. When a message arrives, the callback logs its content.
//   3. rclcpp::spin() keeps the node alive until you press Ctrl+C.
// ============================================================================

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [this](const std_msgs::msg::String & msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
