// ============================================================================
// dynamixel_node.cpp — The hardware driver node (the ONLY node that talks
//                      to physical servos).
//
// WHAT IT DOES:
//   1. Opens the serial port and initializes all AX-12A servos.
//   2. Reads servo state (position, speed, load, temperature) at 20 Hz
//      and publishes it on the /joint_states topic.
//   3. Listens for goal positions on /joint_commands and writes them to
//      the servos via serial.
//
// ARCHITECTURE RULE:
//   This is the ONLY node that touches /dev/ttyUSB0. Every other node
//   communicates exclusively through ROS 2 topics. This isolation means
//   you can test monitor_node or demo_node without any hardware connected.
//
// PARAMETERS (see config/dynamixel_node.yaml):
//   - port:             Serial device path (default: /dev/ttyUSB0)
//   - baudrate:         Serial speed (default: 1000000)
//   - servo_ids:        Array of servo IDs to manage (default: [9, 10, 12])
//   - publish_rate_hz:  How often to read and publish state (default: 20 Hz)
//   - compliance_margin/slope: Motor control smoothness
//   - temp_warn_c:      Temperature threshold for warnings (default: 55 °C)
// ============================================================================

#include "dynamixel_control_cpp/ax12_model.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <memory>
#include <string>
#include <vector>

using namespace dynamixel;
namespace ax12 = dynamixel_control_cpp::ax12;

class DynamixelNode : public rclcpp::Node
{
public:
  DynamixelNode()
  : Node("dynamixel_node")
  {
    declare_parameter("port", "/dev/ttyUSB0");
    declare_parameter("baudrate", 1000000);
    declare_parameter("servo_ids", std::vector<int64_t>{9, 10, 12});
    declare_parameter("publish_rate_hz", 20.0);
    declare_parameter("compliance_margin", 4);
    declare_parameter("compliance_slope", 64);
    declare_parameter("temp_warn_c", 55);

    const auto port_name = get_parameter("port").as_string();
    const auto baudrate = get_parameter("baudrate").as_int();
    const auto ids_param = get_parameter("servo_ids").as_integer_array();
    const auto rate_hz = get_parameter("publish_rate_hz").as_double();
    compliance_margin_ = static_cast<uint8_t>(get_parameter("compliance_margin").as_int());
    compliance_slope_ = static_cast<uint8_t>(get_parameter("compliance_slope").as_int());
    temp_warn_c_ = static_cast<int>(get_parameter("temp_warn_c").as_int());

    for (auto id : ids_param) {
      servo_ids_.push_back(static_cast<int>(id));
    }

    port_handler_ = PortHandler::getPortHandler(port_name.c_str());
    packet_handler_ = PacketHandler::getPacketHandler(1.0);

    if (!port_handler_->openPort()) {
      RCLCPP_FATAL(get_logger(), "Failed to open %s", port_name.c_str());
      throw std::runtime_error("Port open failed");
    }
    port_handler_->setBaudRate(baudrate);
    RCLCPP_INFO(get_logger(), "Port %s opened @ %ld bps", port_name.c_str(), baudrate);

    for (int id : servo_ids_) {
      setup_servo(id);
    }

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_commands", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {on_command(msg);});

    const auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz));
    timer_ = create_wall_timer(period_ms, [this]() {publish_state();});

    RCLCPP_INFO(get_logger(), "Ready: %zu servos @ %.0f Hz", servo_ids_.size(), rate_hz);
  }

  ~DynamixelNode() override
  {
    uint8_t err = 0;
    for (int id : servo_ids_) {
      packet_handler_->write1ByteTxRx(
        port_handler_, id, ax12::reg::TORQUE_ENABLE, 0, &err);
    }
    port_handler_->closePort();
    RCLCPP_INFO(get_logger(), "Torque off. Port closed.");
  }

private:
  void setup_servo(int id)
  {
    uint8_t err = 0;
    uint16_t cw = 0, ccw = 0;

    packet_handler_->read2ByteTxRx(port_handler_, id, ax12::reg::CW_ANGLE_LIMIT, &cw, &err);
    packet_handler_->read2ByteTxRx(port_handler_, id, ax12::reg::CCW_ANGLE_LIMIT, &ccw, &err);

    if (cw == 0 && ccw == 0) {
      RCLCPP_WARN(get_logger(), "[ID %d] Wheel mode detected -> switching to position mode", id);
      packet_handler_->write2ByteTxRx(port_handler_, id, ax12::reg::CW_ANGLE_LIMIT, 0, &err);
      packet_handler_->write2ByteTxRx(port_handler_, id, ax12::reg::CCW_ANGLE_LIMIT, 1023, &err);
    }

    packet_handler_->write1ByteTxRx(
      port_handler_, id, ax12::reg::CW_COMPLIANCE_MARGIN, compliance_margin_, &err);
    packet_handler_->write1ByteTxRx(
      port_handler_, id, ax12::reg::CCW_COMPLIANCE_MARGIN, compliance_margin_, &err);
    packet_handler_->write1ByteTxRx(
      port_handler_, id, ax12::reg::CW_COMPLIANCE_SLOPE, compliance_slope_, &err);
    packet_handler_->write1ByteTxRx(
      port_handler_, id, ax12::reg::CCW_COMPLIANCE_SLOPE, compliance_slope_, &err);

    const int result = packet_handler_->write1ByteTxRx(
      port_handler_, id, ax12::reg::TORQUE_ENABLE, 1, &err);
    if (result != COMM_SUCCESS || err != 0) {
      RCLCPP_ERROR(
        get_logger(), "[ID %d] Error torque (comm=%d, err=0x%02x)", id, result, err);
    } else {
      RCLCPP_INFO(
        get_logger(), "[ID %d] OK (rango %d-%d, margin=%d, slope=%d)",
        id, cw, ccw, compliance_margin_, compliance_slope_);
    }
  }

  void publish_state()
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();

    for (int id : servo_ids_) {
      uint8_t err = 0;
      uint16_t pos = 0, spd = 0, load = 0;
      uint8_t temp = 0;

      packet_handler_->read2ByteTxRx(port_handler_, id, ax12::reg::PRESENT_POSITION, &pos, &err);
      packet_handler_->read2ByteTxRx(port_handler_, id, ax12::reg::PRESENT_SPEED, &spd, &err);
      packet_handler_->read2ByteTxRx(port_handler_, id, ax12::reg::PRESENT_LOAD, &load, &err);
      packet_handler_->read1ByteTxRx(port_handler_, id, ax12::reg::PRESENT_TEMP, &temp, &err);

      msg.name.push_back("servo_" + std::to_string(id));
      msg.position.push_back(ax12::raw_position_to_degrees(pos));
      msg.velocity.push_back(ax12::decode_present_speed(spd));
      msg.effort.push_back(ax12::decode_load_percent(load));

      if (static_cast<int>(temp) > temp_warn_c_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "[ID %d] Temp alta: %d C", id, static_cast<int>(temp));
      }
    }

    state_pub_->publish(msg);
  }

  void on_command(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++) {
      const int id = ax12::parse_servo_id(msg->name[i]);
      if (id < 0) {
        continue;
      }

      const uint16_t goal = ax12::degrees_to_goal_position(msg->position[i]);

      uint8_t err = 0;
      const int result = packet_handler_->write2ByteTxRx(
        port_handler_, id, ax12::reg::GOAL_POSITION, goal, &err);

      if (result != COMM_SUCCESS) {
        RCLCPP_WARN(
          get_logger(), "[ID %d] Error sending goal %u", id,
          static_cast<unsigned>(goal));
      }
    }
  }

  std::vector<int> servo_ids_;
  PortHandler * port_handler_;
  PacketHandler * packet_handler_;
  uint8_t compliance_margin_;
  uint8_t compliance_slope_;
  int temp_warn_c_{55};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamixelNode>());
  rclcpp::shutdown();
  return 0;
}
