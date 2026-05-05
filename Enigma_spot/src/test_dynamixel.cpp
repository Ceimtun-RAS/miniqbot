#include <iostream>
#include <vector>
#include "dynamixel_sdk/dynamixel_sdk.h"

#define PROTOCOL_VERSION 1.0
#define DEVICENAME "/dev/ttyUSB0"

int main()
{
    std::vector<int> baudrates = {9600, 57600, 115200, 1000000};

    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort())
    {
        std::cout << "No se pudo abrir el puerto\n";
        return 0;
    }

    std::cout << "Puerto abierto\n";

    for (int baud : baudrates)
    {
        std::cout << "\nProbando baudrate: " << baud << "\n";

        if (!portHandler->setBaudRate(baud))
        {
            std::cout << "No se pudo configurar baudrate\n";
            continue;
        }

        for (int id = 0; id < 253; id++)
        {
            uint8_t dxl_error = 0;
            uint16_t model_number = 0;

            int result = packetHandler->ping(
                portHandler,
                id,
                &model_number,
                &dxl_error
            );

            if (result == COMM_SUCCESS)
            {
                std::cout << "Dynamixel encontrado!\n";
                std::cout << "ID: " << id << "\n";
                std::cout << "Baudrate: " << baud << "\n";
                std::cout << "Modelo: " << model_number << "\n\n";
            }
        }
    }

    portHandler->closePort();
    return 0;
}