#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <vector>
#include <string>
#include <cmath>

class DemoNode : public rclcpp::Node {
public:
    DemoNode() : Node("demo_node"), step_(0) {
        declare_parameter("servo_ids", std::vector<int64_t>{9, 10, 12});
        declare_parameter("total_steps", 10);

        auto ids_param = get_parameter("servo_ids").as_integer_array();
        total_steps_ = get_parameter("total_steps").as_int();

        for (auto id : ids_param)
            servo_ids_.push_back(static_cast<int>(id));

        cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);
        timer_ = create_wall_timer(std::chrono::seconds(3), [this]() { next_move(); });

        RCLCPP_INFO(get_logger(), "Demo: %zu servos, %ld pasos, movimiento cada 3s",
                     servo_ids_.size(), total_steps_);
    }

private:
    void next_move() {
        if (step_ >= total_steps_) {
            RCLCPP_INFO(get_logger(), "Demo terminada.");
            timer_->cancel();
            rclcpp::shutdown();
            return;
        }

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now();

        for (size_t i = 0; i < servo_ids_.size(); i++) {
            double phase = 2.0 * M_PI * static_cast<double>(i) / servo_ids_.size();
            double angle = 150.0 + 90.0 * std::sin(step_ * 1.0 + phase);

            msg.name.push_back("servo_" + std::to_string(servo_ids_[i]));
            msg.position.push_back(angle);
        }

        std::string positions;
        for (size_t i = 0; i < msg.position.size(); i++) {
            if (i > 0) positions += ", ";
            positions += std::to_string(static_cast<int>(msg.position[i])) + "\u00b0";
        }
        RCLCPP_INFO(get_logger(), "Paso %d/%ld: [%s]", step_ + 1, total_steps_, positions.c_str());

        cmd_pub_->publish(msg);
        step_++;
    }

    std::vector<int> servo_ids_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int step_;
    int64_t total_steps_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoNode>());
    rclcpp::shutdown();
    return 0;
}
