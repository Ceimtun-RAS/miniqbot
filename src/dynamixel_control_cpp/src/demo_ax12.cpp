#include "dynamixel_sdk/dynamixel_sdk.h"
#include <iostream>
#include <unistd.h>
#include <vector>

using namespace dynamixel;

static const char* comm_result_str(int result) {
    switch (result) {
        case COMM_SUCCESS: return "OK";
        case COMM_PORT_BUSY: return "PORT_BUSY";
        case COMM_TX_FAIL: return "TX_FAIL";
        case COMM_RX_FAIL: return "RX_FAIL";
        case COMM_TX_ERROR: return "TX_ERROR";
        case COMM_RX_WAITING: return "RX_WAITING";
        case COMM_RX_TIMEOUT: return "RX_TIMEOUT";
        case COMM_RX_CORRUPT: return "RX_CORRUPT";
        case COMM_NOT_AVAILABLE: return "NOT_AVAILABLE";
        default: return "UNKNOWN";
    }
}

static void print_error(uint8_t dxl_error) {
    if (dxl_error & 0x01) std::cout << " [InputVoltage]";
    if (dxl_error & 0x02) std::cout << " [AngleLimit]";
    if (dxl_error & 0x04) std::cout << " [Overheating]";
    if (dxl_error & 0x08) std::cout << " [Range]";
    if (dxl_error & 0x10) std::cout << " [Checksum]";
    if (dxl_error & 0x20) std::cout << " [Overload]";
    if (dxl_error & 0x40) std::cout << " [Instruction]";
}

bool setup_servo(PacketHandler* packetHandler, PortHandler* portHandler, int id) {
    uint8_t dxl_error = 0;
    uint16_t cw_limit = 0, ccw_limit = 0;

    packetHandler->read2ByteTxRx(portHandler, id, 6, &cw_limit, &dxl_error);
    packetHandler->read2ByteTxRx(portHandler, id, 8, &ccw_limit, &dxl_error);

    if (cw_limit == 0 && ccw_limit == 0) {
        std::cout << "  [ID " << id << "] Modo rueda detectado -> cambiando a modo posición (0-1023)" << std::endl;
        packetHandler->write2ByteTxRx(portHandler, id, 6, 0, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, id, 8, 1023, &dxl_error);
    } else {
        std::cout << "  [ID " << id << "] Rango: " << cw_limit << " - " << ccw_limit << std::endl;
    }

    packetHandler->write1ByteTxRx(portHandler, id, 26, 4, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, id, 27, 4, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, id, 28, 64, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, id, 29, 64, &dxl_error);

    int result = packetHandler->write1ByteTxRx(portHandler, id, 24, 1, &dxl_error);
    std::cout << "  [ID " << id << "] Torque enable: " << comm_result_str(result);
    if (dxl_error) { std::cout << " | Error 0x" << std::hex << (int)dxl_error << std::dec; print_error(dxl_error); }
    std::cout << std::endl;

    return result == COMM_SUCCESS && dxl_error == 0;
}

void move_servo(PacketHandler* packetHandler, PortHandler* portHandler, int id, int goal) {
    uint8_t dxl_error = 0;
    int result = packetHandler->write2ByteTxRx(portHandler, id, 30, goal, &dxl_error);
    std::cout << "  [ID " << id << "] -> " << goal << " : " << comm_result_str(result);
    if (dxl_error) { std::cout << " | Error 0x" << std::hex << (int)dxl_error << std::dec; print_error(dxl_error); }
    std::cout << std::endl;
}

void read_position(PacketHandler* packetHandler, PortHandler* portHandler, int id) {
    uint8_t dxl_error = 0;
    uint16_t pos = 0;
    packetHandler->read2ByteTxRx(portHandler, id, 36, &pos, &dxl_error);
    std::cout << "  [ID " << id << "] Posición actual: " << pos;
    if (dxl_error) { std::cout << " | Error 0x" << std::hex << (int)dxl_error << std::dec; print_error(dxl_error); }
    std::cout << std::endl;
}

int main()
{
    const char* DEVICENAME = "/dev/ttyUSB0";
    const int BAUDRATE = 1000000;
    const std::vector<int> SERVO_IDS = {9, 10, 12};

    PortHandler* portHandler = PortHandler::getPortHandler(DEVICENAME);
    PacketHandler* packetHandler = PacketHandler::getPacketHandler(1.0);

    if (portHandler->openPort())
        std::cout << "✅ Puerto abierto" << std::endl;
    else {
        std::cerr << "❌ Error al abrir puerto" << std::endl;
        return 1;
    }
    portHandler->setBaudRate(BAUDRATE);

    // --- Setup: configurar modo posición y activar torque ---
    std::cout << "\n=== Configurando servos ===" << std::endl;
    for (int id : SERVO_IDS)
        setup_servo(packetHandler, portHandler, id);

    // --- Demo: mover todos juntos entre dos posiciones ---
    int positions_a[] = {200, 200, 800};
    int positions_b[] = {800, 512, 200};

    for (int rep = 0; rep < 3; rep++) {
        int* goals = (rep % 2 == 0) ? positions_a : positions_b;
        std::cout << "\n=== Movimiento " << (rep + 1) << " ===" << std::endl;
        for (size_t i = 0; i < SERVO_IDS.size(); i++)
            move_servo(packetHandler, portHandler, SERVO_IDS[i], goals[i]);
        sleep(2);
    }

    // --- Leer posiciones finales ---
    std::cout << "\n=== Posiciones finales ===" << std::endl;
    for (int id : SERVO_IDS)
        read_position(packetHandler, portHandler, id);

    // --- Desactivar torque y cerrar ---
    std::cout << "\n=== Apagando ===" << std::endl;
    uint8_t dxl_error = 0;
    for (int id : SERVO_IDS) {
        packetHandler->write1ByteTxRx(portHandler, id, 24, 0, &dxl_error);
        std::cout << "  [ID " << id << "] Torque off" << std::endl;
    }

    portHandler->closePort();
    std::cout << "✅ Listo" << std::endl;
}
