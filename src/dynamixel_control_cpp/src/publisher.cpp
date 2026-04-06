// ============================================================================
// publisher.cpp — Minimal ROS 2 publisher example.
//
// This is a learning example, not part of the robot's control system.
// It publishes a "Hello, enigma!" string on the /chatter topic every 500 ms.
//
// Use this alongside subscriber.cpp to understand the basics of ROS 2
// pub/sub communication. See docs/ros2_tutorial.md for a full walkthrough.
//
// HOW IT WORKS:
//   1. Creates a publisher on the "chatter" topic.
//   2. Sets up a timer that fires every 500 ms.
//   3. Each time the timer fires, publishes a String message with a counter.
//   4. rclcpp::spin() keeps the node alive until you press Ctrl+C.
// ============================================================================

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(500ms, [this]() {publish_msg();});
  }

private:
  void publish_msg()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello, enigma! count = " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
