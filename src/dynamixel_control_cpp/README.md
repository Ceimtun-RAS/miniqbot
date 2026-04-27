# Dynamixel Control - ROS 2

Control of Dynamixel AX-12 servo motors from ROS 2 (Humble) in C++.

---

## What You Need

- **Ubuntu 22.04** with **ROS 2 Humble** installed
- **Dynamixel AX-12 servo motors** (Protocol 1.0)
- **USB adapter** (U2D2, USB2Dynamixel, or similar) connected to `/dev/ttyUSB0`
- The `dynamixel_sdk` package installed in your ROS 2 workspace

---

## Project Structure

```
src/dynamixel_control_cpp/
├── config/                         <-- YAML parameters for each node
│   ├── dynamixel_node.yaml
│   ├── monitor_node.yaml
│   └── demo_node.yaml
├── include/dynamixel_control_cpp/  <-- Pure logic (no ROS), unit-testable
│   ├── ax12_model.hpp              <-- AX-12 constants and conversions
│   ├── demo_motion.hpp             <-- Sinusoidal trajectory formulas
│   └── monitor_display.hpp         <-- Terminal dashboard rendering
├── launch/
│   └── dynamixel_stack.xml         <-- Launches all 3 nodes with one command
├── src/
│   ├── dynamixel_node.cpp          <-- Hardware node (talks to the motors)
│   ├── monitor_node.cpp            <-- Monitor node (terminal dashboard)
│   ├── demo_node.cpp               <-- Demo node (sends movement commands)
│   ├── demo_ax12.cpp               <-- Standalone script (no ROS, direct serial)
│   ├── monitor.cpp                 <-- Standalone monitor (no ROS)
│   ├── publisher.cpp               <-- Basic ROS 2 pub/sub example
│   └── subscriber.cpp              <-- Basic ROS 2 pub/sub example
├── test/
│   └── test_ax12_model.cpp         <-- Unit tests for the header logic
├── CMakeLists.txt
├── package.xml
└── README.md                       <-- This file
```

---

## Key Concepts (for beginners)

### What is a Dynamixel servo motor?

It is a smart motor that you can control from your computer. You tell it
"go to position 500" and the motor moves there on its own. It also reports
its current position, how much force it is applying (load), and its temperature.

### What is ROS 2?

It is a framework for robots. Instead of having one giant program that does
everything, you split your robot into **nodes** (small programs) that
communicate through **topics** (message channels).

### Why split into nodes?

Imagine you have a robot with 12 motors. Without ROS 2 you would have a single
huge program that does everything: read motors, display data, send commands,
plan movements... If anything fails, everything crashes.

With ROS 2, each task is an independent node:

```
┌─────────────────┐     /joint_states      ┌────────────────┐
│  dynamixel_node │ ─────────────────────> │  monitor_node  │
│   (hardware)    │   (position, load,     │  (dashboard)   │
│                 │    velocity)            └────────────────┘
│  /dev/ttyUSB0   │
│                 │ <─────────────────────  ┌────────────────┐
└─────────────────┘     /joint_commands    │   demo_node    │
                      (goal positions)     │  (movements)   │
                                            └────────────────┘
```

- If the monitor crashes, the motors keep running.
- If you want to change how they move, you only modify `demo_node`.
- You can add new nodes (e.g., planner, vision) without touching the others.

---

## How to Build

Always in this order:

```bash
# 1. Load ROS 2 (once per terminal)
source /opt/ros/humble/setup.bash

# 2. Go to the workspace
cd ~/miniqbot

# 3. Build
colcon build --packages-select dynamixel_control_cpp

# 4. Load the freshly built executables
source install/setup.bash
```

> **Golden rule:** `source ROS` → `colcon build` → `source install` → run.

---

## How to Run (full ROS 2 system)

You need **3 terminals**. In each one, first run:

```bash
source /opt/ros/humble/setup.bash
cd ~/miniqbot
source install/setup.bash
```

### Terminal 1 - Hardware node (always start this first)

```bash
ros2 run dynamixel_control_cpp dynamixel_node
```

This node:
- Opens the serial port and connects to the motors
- Reads position, velocity, load, and temperature 20 times per second
- Publishes everything on the `/joint_states` topic
- Listens for commands on `/joint_commands` and moves the motors
- Treats IDs in `wheel_servo_ids` as continuous wheels controlled by velocity

### Terminal 2 - Monitor (real-time dashboard)

```bash
ros2 run dynamixel_control_cpp monitor_node
```

You will see something like this:

```
  ╔═══════════════════════════════════════════════════════════╗
  ║    DYNAMIXEL ROBOT MONITOR  (3 servos)        frame  42  ║
  ╚═══════════════════════════════════════════════════════════╝
  ── servo_9   ─────────────────────────────────────────────
    Pos:  58.9°   Vel:     0
    Load:   2.3% CW  [#-----------------------------]  low

  ── servo_10  ─────────────────────────────────────────────
    Pos: 149.6°   Vel:     0
    Load:   1.5% CW  [------------------------------]  low

  ── servo_12  ─────────────────────────────────────────────
    Pos: 234.0°   Vel:     0
    Load:  62.1% CW  [#############-----------------]  medium
```

- **Green** bar = low load (motor is relaxed)
- **Yellow** bar = moderate load (some resistance)
- **Red** bar = high load (high force or stall)

### Terminal 3 - Demo (send movements)

```bash
ros2 run dynamixel_control_cpp demo_node
```

Runs a wave motion sequence: each motor moves to a different position,
creating a wave-like pattern for joint servos. Wheel servos spin using the
configured `wheel_speed_raw` value. It performs 10 steps, one every 3 seconds.

---

## Launch All with One Command

Instead of opening 3 terminals, you can use the launch file:

```bash
source /opt/ros/humble/setup.bash
cd ~/miniqbot
source install/setup.bash

ros2 launch dynamixel_control_cpp dynamixel_stack.xml
```

This starts `dynamixel_node`, `monitor_node`, and `demo_node` simultaneously,
each with its parameters from the `config/` directory.

---

## How to Run Tests

Unit tests verify the pure logic (conversions, parsing) without needing
hardware connected:

```bash
source /opt/ros/humble/setup.bash
cd ~/miniqbot

# Build with tests enabled
colcon build --packages-select dynamixel_control_cpp

# Run the tests
colcon test --packages-select dynamixel_control_cpp

# View results
colcon test-result --all
```

You should see something like:

```
test_ax12_model.gtest.xml: 7 tests, 0 errors, 0 failures, 0 skipped
```

If all tests pass, the AX-12 logic is working correctly.

---

## Variables Read from Each Motor

| Variable | What it measures | What it is useful for |
|----------|-----------------|----------------------|
| **Position** | Current angle (0–300 degrees) | Knowing where the motor is |
| **Speed** | Rotation velocity | Knowing if it is moving |
| **Load** | Force/effort (0–100%) | Detecting contact, collisions, resistance |
| **Temperature** | Internal heat (Celsius) | Protecting the motor from overheating |

### Load: the most useful variable for robotics

If your robot has a foot and you want to know when it touches the ground:

- Motor lowering freely → load is low (~5%)
- Foot touches the ground → load **jumps suddenly** (~40–80%)
- Motor stalled → load at maximum, velocity 0

---

## Configurable Parameters

The nodes accept parameters via command line:

```bash
# Change servo IDs (e.g., for 12 motors)
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p servo_ids:="[1,2,3,4,5,6,7,8,9,10,11,12]"

# Configure ID 9 as a wheel and ID 10 as a joint
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p servo_ids:="[9,10]" -p wheel_servo_ids:="[9]"

# Change port or baud rate
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p port:="/dev/ttyACM0" -p baudrate:=57600

# Change read frequency (Hz)
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p publish_rate_hz:=50.0

# Adjust compliance (see section below)
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p compliance_margin:=4 -p compliance_slope:=64

# The demo also accepts IDs and step count
ros2 run dynamixel_control_cpp demo_node \
  --ros-args -p servo_ids:="[1,2,3,4,5,6,7,8,9,10,11,12]" -p total_steps:=20

# Demo with ID 9 as a wheel and ID 10 as a joint
ros2 run dynamixel_control_cpp demo_node \
  --ros-args -p servo_ids:="[9,10]" -p wheel_servo_ids:="[9]" -p wheel_speed_raw:=200
```

### Joint mode vs wheel mode

- **Joint servos** receive `position` commands in degrees and move to a fixed angle.
- **Wheel servos** receive `velocity` commands as signed AX-12 raw speed units.
  Positive speed rotates counter-clockwise, negative speed rotates clockwise.

By default, this package treats **ID 9 as a wheel** and **ID 10 as a joint**.

### Compliance: preventing motor oscillation

The AX-12 uses a system called **Compliance** to hold its position.
It has two key settings:

- **Compliance Margin**: dead zone around the goal (in position units,
  ~0.29 degrees each). If the error is within the margin,
  the motor does not try to correct. Factory default = 1 (very tight).
- **Compliance Slope**: how aggressively the motor corrects when the error
  exceeds the margin. Low values = aggressive, high values = smooth.
  Factory default = 32.

With factory defaults, the motors can **oscillate/vibrate** because they
try to correct 0.3-degree errors aggressively. This is especially
noticeable in worn motors or under load.

The `dynamixel_node` configures `margin=4` and `slope=64` by default, which
provide smooth and stable behavior. You can adjust them as needed:

| Case | margin | slope | Result |
|------|--------|-------|--------|
| Motor vibrates a lot | 8 | 128 | Very smooth, less precision |
| Normal operation | 4 | 64 | Good balance (node default) |
| High precision | 2 | 32 | More precise but may oscillate |
| AX-12 factory | 1 | 32 | May vibrate on worn motors |

```bash
# Worn or old motors: more smoothness
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p compliance_margin:=8 -p compliance_slope:=128

# High precision (only if the motors do not vibrate)
ros2 run dynamixel_control_cpp dynamixel_node \
  --ros-args -p compliance_margin:=2 -p compliance_slope:=32
```

---

## Standalone Programs (no ROS)

For quick tests without starting the full ROS 2 infrastructure:

```bash
# Direct demo: moves 3 motors (IDs 9, 10, 12)
ros2 run dynamixel_control_cpp demo_ax12

# Direct monitor: shows load/position without ROS topics
ros2 run dynamixel_control_cpp monitor
```

These talk directly to the motors over serial. Only one can run at a time
(because the serial port cannot be shared).

---

## Common Problems

### "Error opening serial port"

Another program has the port open. Solution:

```bash
# See who is using the port
fuser /dev/ttyUSB0

# Kill it
fuser -k /dev/ttyUSB0
```

> **Tip:** If you cancel a program, use **Ctrl+C** (terminates it). Never **Ctrl+Z**
> (pauses it but leaves the port locked).

### Motor vibrates/oscillates

The motor is trying to correct its position too aggressively. See the
**Compliance** section above. Quick fix: increase margin and slope.

### Motor does not move (Error 0x02 = Angle Limit)

The motor mode does not match the command type. Joint servos need angle limits
configured for position mode. Wheel servos need both angle limits set to `0`.
Set `wheel_servo_ids` so the driver configures each servo correctly.

### Error 0x04 = Overheating

The motor is too hot. Let it cool down for a few minutes. If it persists,
check ventilation or reduce the torque.

### Not sure which port to use

```bash
# List available ports
ls /dev/ttyUSB* /dev/ttyACM*

# Identify: disconnect the USB, check the list, reconnect and see what appeared
```

### Port permissions

```bash
# Permanent solution (requires logging out and back in)
sudo usermod -aG dialout $USER

# Quick fix
sudo chmod 666 /dev/ttyUSB0
```

---

## Next Step: Your Own Node

To add new behavior (e.g., walking, following an object), create a node
that publishes to `/joint_commands`. You do not need to touch the hardware node:

```cpp
// my_controller.cpp - basic skeleton
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class MyController : public rclcpp::Node {
public:
    MyController() : Node("my_controller") {
        // Read motor state
        state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                // Here you can read msg->position, msg->effort (load), etc.
            });

        // Send commands
        cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_commands", 10);
    }
};
```

Add the executable to `CMakeLists.txt`, build, and you have a new node
that controls your robot without modifying any existing code.
