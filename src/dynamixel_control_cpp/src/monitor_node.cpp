#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

class MonitorNode : public rclcpp::Node {
public:
    MonitorNode() : Node("monitor_node"), frame_count_(0) {
        sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { display(msg); });

        RCLCPP_INFO(get_logger(), "Esperando datos en /joint_states ...");
    }

private:
    static std::string bar(double percent, int width = 30) {
        double abs_pct = std::abs(percent);
        int filled = static_cast<int>(abs_pct / 100.0 * width);
        if (filled > width) filled = width;

        std::string result;
        for (int i = 0; i < width; i++) {
            if (i < filled) {
                if (abs_pct > 75)     result += "\033[31m#\033[0m";
                else if (abs_pct > 40) result += "\033[33m#\033[0m";
                else                   result += "\033[32m#\033[0m";
            } else {
                result += "\033[90m-\033[0m";
            }
        }
        return result;
    }

    static const char* load_label(double abs_pct) {
        if (abs_pct > 75) return "\033[31m ALTA!\033[0m";
        if (abs_pct > 40) return "\033[33m media\033[0m";
        return "\033[32m baja \033[0m";
    }

    void display(const sensor_msgs::msg::JointState::SharedPtr msg) {
        size_t n = msg->name.size();
        if (n == 0) return;

        if (frame_count_ > 0)
            std::cout << "\033[" << (3 + n * 5) << "A";
        frame_count_++;

        std::cout
            << "\033[36m  ╔═══════════════════════════════════════════════════════════╗\033[0m\n"
            << "\033[36m  ║    MONITOR ROBOT DYNAMIXEL  (" << n << " servos)       frame "
            << std::setw(5) << frame_count_ << "  ║\033[0m\n"
            << "\033[36m  ╚═══════════════════════════════════════════════════════════╝\033[0m\n";

        for (size_t i = 0; i < n; i++) {
            double pos  = (i < msg->position.size()) ? msg->position[i] : 0.0;
            double vel  = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
            double load = (i < msg->effort.size())   ? msg->effort[i]   : 0.0;
            double abs_load = std::abs(load);
            const char* dir = (load >= 0) ? "CW " : "CCW";

            std::cout
                << "  \033[1;37m── " << std::setw(10) << std::left << msg->name[i]
                << " ───────────────────────────────────────────────\033[0m\n"
                << "    Pos: " << std::fixed << std::setprecision(1)
                << std::setw(6) << std::right << pos << "\u00b0"
                << "    Vel: " << std::setprecision(0) << std::setw(5) << vel
                << "                                \n"
                << "    Load: " << std::setprecision(1) << std::setw(5) << abs_load
                << "% " << dir << " [" << bar(load) << "] " << load_label(abs_load)
                << "     \n\n\n";
        }

        std::cout << std::flush;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    int frame_count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::cout << "\033[2J\033[H";
    rclcpp::spin(std::make_shared<MonitorNode>());
    rclcpp::shutdown();
    return 0;
}
