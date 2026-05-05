#include <iostream>
#include "dynamixel_sdk/dynamixel_sdk.h"

#define PROTOCOL_VERSION 1.0
#define DEVICENAME "/dev/ttyUSB0"

#define BAUDRATE 1000000
#define DXL_ID 12

#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30

#define TORQUE_ENABLE 1

int main()
{
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    portHandler->openPort();
    portHandler->setBaudRate(BAUDRATE);

    uint8_t error = 0;

    packetHandler->write1ByteTxRx(
        portHandler,
        DXL_ID,
        ADDR_TORQUE_ENABLE,
        TORQUE_ENABLE,
        &error
    );

    std::cout << "Torque activado\n";

    while (true)
    {
        int pos;
        std::cout << "Ingrese posicion (0-1023): ";
        std::cin >> pos;

        packetHandler->write2ByteTxRx(
            portHandler,
            DXL_ID,
            ADDR_GOAL_POSITION,
            pos,
            &error
        );
    }
}