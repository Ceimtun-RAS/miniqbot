// ============================================================================
// monitor.cpp — Standalone real-time servo monitor WITHOUT ROS 2.
//
// This program reads servo state directly over serial and draws a live
// terminal dashboard. It is useful for:
//   - Debugging hardware without ROS 2 running.
//   - Seeing raw servo values (position, speed, load, voltage, temperature).
//
// HOW IT WORKS:
//   1. Opens the serial port and enables torque on all servos.
//   2. In a loop (10 Hz), reads 5 registers from each servo.
//   3. Draws a color-coded dashboard with ANSI escape codes.
//   4. On Ctrl+C, disables torque and exits cleanly.
//
// NOTE: This bypasses the ROS 2 architecture. For normal operation, use
//       dynamixel_node + monitor_node instead.
// ============================================================================

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <csignal>
#include <unistd.h>

using namespace dynamixel;

static volatile bool running = true;

void signal_handler(int) {running = false;}

struct ServoState
{
  int id;
  uint16_t position;
  uint16_t raw_speed;
  uint16_t raw_load;
  uint8_t voltage;
  uint8_t temperature;

  double angle_deg() const {return position * 300.0 / 1023.0;}

  int speed_signed() const
  {
    int mag = raw_speed & 0x3FF;
    return (raw_speed & 0x400) ? -mag : mag;
  }

  int load_signed() const
  {
    int mag = raw_load & 0x3FF;
    return (raw_load & 0x400) ? -mag : mag;
  }

  double load_percent() const {return (raw_load & 0x3FF) * 100.0 / 1023.0;}
  const char * load_dir() const {return (raw_load & 0x400) ? "CCW" : "CW ";}
  double volts() const {return voltage / 10.0;}
};

std::string bar(double percent, int width = 30)
{
  int filled = static_cast<int>(percent / 100.0 * width);
  if (filled < 0) {filled = 0;}
  if (filled > width) {filled = width;}

  std::string result;
  for (int i = 0; i < width; i++) {
    if (i < filled) {
      if (percent > 75) {
        result += "\033[31m#\033[0m";                             // rojo
      } else if (percent > 40) {
        result += "\033[33m#\033[0m";                             // amarillo
      } else {
        result += "\033[32m#\033[0m";                              // verde
      }
    } else {
      result += "\033[90m-\033[0m";
    }
  }
  return result;
}

void read_servo(PacketHandler * ph, PortHandler * port, ServoState & s)
{
  uint8_t err = 0;
  ph->read2ByteTxRx(port, s.id, 36, &s.position, &err);
  ph->read2ByteTxRx(port, s.id, 38, &s.raw_speed, &err);
  ph->read2ByteTxRx(port, s.id, 40, &s.raw_load, &err);
  ph->read1ByteTxRx(port, s.id, 42, &s.voltage, &err);
  ph->read1ByteTxRx(port, s.id, 43, &s.temperature, &err);
}

int main()
{
  const char * DEVICENAME = "/dev/ttyUSB0";
  const int BAUDRATE = 1000000;
  const std::vector<int> IDS = {9, 10, 12};

  signal(SIGINT, signal_handler);

  PortHandler * portHandler = PortHandler::getPortHandler(DEVICENAME);
  PacketHandler * packetHandler = PacketHandler::getPacketHandler(1.0);

  if (!portHandler->openPort()) {
    std::cerr << "Error: failed to open port" << std::endl;
    return 1;
  }
  portHandler->setBaudRate(BAUDRATE);

  uint8_t dxl_error = 0;
  for (int id : IDS) {
    uint16_t cw = 0, ccw = 0;
    packetHandler->read2ByteTxRx(portHandler, id, 6, &cw, &dxl_error);
    packetHandler->read2ByteTxRx(portHandler, id, 8, &ccw, &dxl_error);
    if (cw == 0 && ccw == 0) {
      packetHandler->write2ByteTxRx(portHandler, id, 6, 0, &dxl_error);
      packetHandler->write2ByteTxRx(portHandler, id, 8, 1023, &dxl_error);
    }
    packetHandler->write1ByteTxRx(portHandler, id, 24, 1, &dxl_error);
  }

  std::vector<ServoState> servos(IDS.size());
  for (size_t i = 0; i < IDS.size(); i++) {
    servos[i].id = IDS[i];
  }

  std::cout << "\033[2J";    // clear screen
  std::cout << "\n  Dynamixel Monitor - Press Ctrl+C to exit\n" << std::endl;

  while (running) {
    for (auto & s : servos) {
      read_servo(packetHandler, portHandler, s);
    }

    // Move cursor up to overwrite (3 header lines + 6 lines per servo)
    std::cout << "\033[" << (3 + servos.size() * 6 + 1) << "A";

    std::cout <<
      "\033[36m  ╔══════════════════════════════════════════════════════════════╗\033[0m" <<
      std::endl;
    std::cout <<
      "\033[36m  ║        DYNAMIXEL REAL-TIME MONITOR                          ║\033[0m" <<
      std::endl;
    std::cout <<
      "\033[36m  ╚══════════════════════════════════════════════════════════════╝\033[0m" <<
      std::endl;

    for (auto & s : servos) {
      double lp = s.load_percent();

      std::cout << "  \033[1;37m─── Motor ID " << std::setw(2) << s.id <<
        " ───────────────────────────────────────────────\033[0m" << std::endl;

      std::cout << "    Pos: " << std::setw(4) << s.position
                << " (" << std::fixed << std::setprecision(1) << std::setw(5) << s.angle_deg() <<
        "°)"
                << "    Vel: " << std::setw(5) << s.speed_signed()
                << "    Temp: " << std::setw(2) << (int)s.temperature << "°C"
                << "    Volt: " << std::setprecision(1) << s.volts() << "V"
                << "          " << std::endl;

      std::cout << "    Load: " << std::setw(5) << std::setprecision(1) << lp << "% "
                << s.load_dir() << " [" << bar(lp) << "] ";
      if (lp > 75) {std::cout << "\033[31m HIGH!\033[0m";} else if (lp > 40) {
        std::cout << "\033[33m  med \033[0m";
      } else {std::cout << "\033[32m  low \033[0m";}
      std::cout << "     " << std::endl;

      std::cout << std::endl;
      std::cout << std::endl;
    }

    usleep(100000);      // 100ms = 10 Hz
  }

  std::cout << "\n\n  Shutting down servos..." << std::endl;
  for (int id : IDS) {
    packetHandler->write1ByteTxRx(portHandler, id, 24, 0, &dxl_error);
  }

  portHandler->closePort();
  std::cout << "  Done." << std::endl;
}
