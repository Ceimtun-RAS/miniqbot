#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <vector>
#include <string>

using namespace dynamixel;

// AX-12 control table (Protocol 1.0)
namespace ax12 {
    constexpr uint8_t CW_ANGLE_LIMIT      = 6;
    constexpr uint8_t CCW_ANGLE_LIMIT     = 8;
    constexpr uint8_t TORQUE_ENABLE       = 24;
    constexpr uint8_t CW_COMPLIANCE_MARGIN  = 26;
    constexpr uint8_t CCW_COMPLIANCE_MARGIN = 27;
    constexpr uint8_t CW_COMPLIANCE_SLOPE   = 28;
    constexpr uint8_t CCW_COMPLIANCE_SLOPE  = 29;
    constexpr uint8_t GOAL_POSITION       = 30;
    constexpr uint8_t PRESENT_POSITION    = 36;
    constexpr uint8_t PRESENT_SPEED       = 38;
    constexpr uint8_t PRESENT_LOAD        = 40;
    constexpr uint8_t PRESENT_VOLTAGE     = 42;
    constexpr uint8_t PRESENT_TEMP        = 43;

    constexpr double POS_TO_DEG = 300.0 / 1023.0;
    constexpr double DEG_TO_POS = 1023.0 / 300.0;
}

class DynamixelNode : public rclcpp::Node {
public:
    DynamixelNode() : Node("dynamixel_node") {
        declare_parameter("port", "/dev/ttyUSB0");
        declare_parameter("baudrate", 1000000);
        declare_parameter("servo_ids", std::vector<int64_t>{9, 10, 12});
        declare_parameter("publish_rate_hz", 20.0);
        declare_parameter("compliance_margin", 4);
        declare_parameter("compliance_slope", 64);

        auto port_name = get_parameter("port").as_string();
        auto baudrate = get_parameter("baudrate").as_int();
        auto ids_param = get_parameter("servo_ids").as_integer_array();
        auto rate_hz = get_parameter("publish_rate_hz").as_double();
        compliance_margin_ = static_cast<uint8_t>(get_parameter("compliance_margin").as_int());
        compliance_slope_ = static_cast<uint8_t>(get_parameter("compliance_slope").as_int());

        for (auto id : ids_param)
            servo_ids_.push_back(static_cast<int>(id));

        port_handler_ = PortHandler::getPortHandler(port_name.c_str());
        packet_handler_ = PacketHandler::getPacketHandler(1.0);

        if (!port_handler_->openPort()) {
            RCLCPP_FATAL(get_logger(), "No se pudo abrir %s", port_name.c_str());
            throw std::runtime_error("Port open failed");
        }
        port_handler_->setBaudRate(baudrate);
        RCLCPP_INFO(get_logger(), "Puerto %s abierto @ %ld bps", port_name.c_str(), baudrate);

        for (int id : servo_ids_)
            setup_servo(id);

        state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_commands", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_command(msg); });

        auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz));
        timer_ = create_wall_timer(period_ms, [this]() { publish_state(); });

        RCLCPP_INFO(get_logger(), "Listo: %zu servos @ %.0f Hz", servo_ids_.size(), rate_hz);
    }

    ~DynamixelNode() override {
        uint8_t err = 0;
        for (int id : servo_ids_)
            packet_handler_->write1ByteTxRx(port_handler_, id, ax12::TORQUE_ENABLE, 0, &err);
        port_handler_->closePort();
        RCLCPP_INFO(get_logger(), "Torque off. Puerto cerrado.");
    }

private:
    void setup_servo(int id) {
        uint8_t err = 0;
        uint16_t cw = 0, ccw = 0;

        packet_handler_->read2ByteTxRx(port_handler_, id, ax12::CW_ANGLE_LIMIT, &cw, &err);
        packet_handler_->read2ByteTxRx(port_handler_, id, ax12::CCW_ANGLE_LIMIT, &ccw, &err);

        if (cw == 0 && ccw == 0) {
            RCLCPP_WARN(get_logger(), "[ID %d] Modo rueda -> modo posición", id);
            packet_handler_->write2ByteTxRx(port_handler_, id, ax12::CW_ANGLE_LIMIT, 0, &err);
            packet_handler_->write2ByteTxRx(port_handler_, id, ax12::CCW_ANGLE_LIMIT, 1023, &err);
        }

        packet_handler_->write1ByteTxRx(port_handler_, id, ax12::CW_COMPLIANCE_MARGIN, compliance_margin_, &err);
        packet_handler_->write1ByteTxRx(port_handler_, id, ax12::CCW_COMPLIANCE_MARGIN, compliance_margin_, &err);
        packet_handler_->write1ByteTxRx(port_handler_, id, ax12::CW_COMPLIANCE_SLOPE, compliance_slope_, &err);
        packet_handler_->write1ByteTxRx(port_handler_, id, ax12::CCW_COMPLIANCE_SLOPE, compliance_slope_, &err);

        int result = packet_handler_->write1ByteTxRx(port_handler_, id, ax12::TORQUE_ENABLE, 1, &err);
        if (result != COMM_SUCCESS || err != 0)
            RCLCPP_ERROR(get_logger(), "[ID %d] Error torque (comm=%d, err=0x%02x)", id, result, err);
        else
            RCLCPP_INFO(get_logger(), "[ID %d] OK (rango %d-%d, margin=%d, slope=%d)",
                         id, cw, ccw, compliance_margin_, compliance_slope_);
    }

    void publish_state() {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now();

        for (int id : servo_ids_) {
            uint8_t err = 0;
            uint16_t pos = 0, spd = 0, load = 0;
            uint8_t temp = 0;

            packet_handler_->read2ByteTxRx(port_handler_, id, ax12::PRESENT_POSITION, &pos, &err);
            packet_handler_->read2ByteTxRx(port_handler_, id, ax12::PRESENT_SPEED, &spd, &err);
            packet_handler_->read2ByteTxRx(port_handler_, id, ax12::PRESENT_LOAD, &load, &err);
            packet_handler_->read1ByteTxRx(port_handler_, id, ax12::PRESENT_TEMP, &temp, &err);

            msg.name.push_back("servo_" + std::to_string(id));
            msg.position.push_back(pos * ax12::POS_TO_DEG);

            int speed_val = spd & 0x3FF;
            if (spd & 0x400) speed_val = -speed_val;
            msg.velocity.push_back(static_cast<double>(speed_val));

            double load_pct = (load & 0x3FF) * 100.0 / 1023.0;
            if (load & 0x400) load_pct = -load_pct;
            msg.effort.push_back(load_pct);

            if (temp > 55)
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "[ID %d] Temp alta: %d C", id, temp);
        }

        state_pub_->publish(msg);
    }

    void on_command(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++) {
            int id = parse_servo_id(msg->name[i]);
            if (id < 0) continue;

            uint16_t goal = static_cast<uint16_t>(msg->position[i] * ax12::DEG_TO_POS);
            if (goal > 1023) goal = 1023;

            uint8_t err = 0;
            int result = packet_handler_->write2ByteTxRx(
                port_handler_, id, ax12::GOAL_POSITION, goal, &err);

            if (result != COMM_SUCCESS)
                RCLCPP_WARN(get_logger(), "[ID %d] Error enviando goal %d", id, goal);
        }
    }

    static int parse_servo_id(const std::string& name) {
        auto pos = name.rfind('_');
        if (pos == std::string::npos) return -1;
        try { return std::stoi(name.substr(pos + 1)); }
        catch (...) { return -1; }
    }

    std::vector<int> servo_ids_;
    PortHandler* port_handler_;
    PacketHandler* packet_handler_;
    uint8_t compliance_margin_;
    uint8_t compliance_slope_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamixelNode>());
    rclcpp::shutdown();
    return 0;
}
