#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 36

#define PROTOCOL_VERSION 1.0

#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB0"

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

#define DXL_ID 12

using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

class ReadWriteNode : public rclcpp::Node
{
public:

  ReadWriteNode() : Node("read_write_node")
  {
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (portHandler->openPort())
      RCLCPP_INFO(this->get_logger(), "Puerto abierto");
    else
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el puerto");

    if (portHandler->setBaudRate(BAUDRATE))
      RCLCPP_INFO(this->get_logger(), "Baudrate configurado");

    uint8_t dxl_error = 0;

    packetHandler->write1ByteTxRx(
      portHandler,
      DXL_ID,
      ADDR_TORQUE_ENABLE,
      TORQUE_ENABLE,
      &dxl_error);

    RCLCPP_INFO(this->get_logger(), "Torque habilitado");

    set_position_subscriber_ =
      this->create_subscription<SetPosition>(
      "set_position",
      10,
      std::bind(&ReadWriteNode::set_position_callback, this, std::placeholders::_1));

    get_position_server_ =
      this->create_service<GetPosition>(
      "get_position",
      std::bind(
        &ReadWriteNode::get_position_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }

private:

  void set_position_callback(const SetPosition::SharedPtr msg)
  {
    uint8_t dxl_error = 0;

    packetHandler->write2ByteTxRx(
      portHandler,
      DXL_ID,
      ADDR_GOAL_POSITION,
      msg->position,
      &dxl_error);

    RCLCPP_INFO(this->get_logger(), "Moviendo a %d", msg->position);
  }

  void get_position_callback(
    const std::shared_ptr<GetPosition::Request>,
    std::shared_ptr<GetPosition::Response> res)
  {
    uint8_t dxl_error = 0;
    uint16_t present_position = 0;

    packetHandler->read2ByteTxRx(
      portHandler,
      DXL_ID,
      ADDR_PRESENT_POSITION,
      &present_position,
      &dxl_error);

    res->position = present_position;
  }

  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;

  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReadWriteNode>());
  rclcpp::shutdown();
  return 0;
}