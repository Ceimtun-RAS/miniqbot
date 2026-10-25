# miniqbot_description

The **robot description** of MiniQ-bot: a URDF file that tells ROS 2 what the
robot looks like, how heavy each part is and how its joints move. Other
packages (RViz, Gazebo, the walking controller) read it.

This package has no nodes of its own. It only holds files.

## What is inside

```
miniqbot_description/
├── urdf/miniqbot.urdf          the robot description (GENERATED, do not edit by hand)
├── meshes/*.stl                the 3D shape of each link, used to draw it
├── launch/display.launch.xml   opens RViz with one slider per joint
└── rviz/display.rviz           RViz settings (camera, what to show)
```

`urdf/miniqbot.urdf` and `meshes/` are generated from the Fusion CAD by
[`tools/fusion_to_urdf`](../../tools/fusion_to_urdf/README.md). To change them,
edit `tools/fusion_to_urdf/robot_config.yaml` and generate them again.

## Usage

Build and source the workspace (from the repository root):

```bash
colcon build --packages-select miniqbot_description
source install/setup.bash
```

Show the robot in RViz and move each joint with a slider:

```bash
ros2 launch miniqbot_description display.launch.xml
```

## The robot model

13 rigid blocks (the body plus a hip, a thigh and a shin per leg) and 12
motor joints. `base_link` and the `*_foot` links are only reference points,
they have no shape or mass. Leg names: `fl` front left, `fr` front right,
`bl` back left, `br` back right.

```
base_link
└── body_link
    ├── fl_hip_link    (fl_hip_joint,   motor M1_1)
    │   └── fl_thigh_link    (fl_thigh_joint, motor M2_1)
    │       └── fl_shin_link    (fl_knee_joint,  motor M3_1)
    │           └── fl_foot     (fixed: center of the foot contact sphere)
    ├── fr_... same as fl
    ├── bl_... same as fl
    └── br_... same as fl
```

| Joint | What it moves | Turns around |
|-------|---------------|--------------|
| `*_hip_joint` | the whole leg, sideways | x (forward) |
| `*_thigh_joint` | the thigh, forward and back | y (left) |
| `*_knee_joint` | the shin | y (left) |

Conventions:

- Axes follow ROS [REP 103](https://www.ros.org/reps/rep-0103.html):
  x forward, y left, z up. Units: meters, kilograms, radians.
- Angle 0 of every joint is the pose of the robot in the CAD.
- All legs use the same axis directions, so the same angle means the same
  motion on every leg. The real servo directions are handled by the motor
  driver, not here.

## Provisional values

This first model comes from the CAD **"Fixed for URDF" v43**, with the current
screw-driven knee. Some values are temporary:

| Item | Current value | Why it is temporary |
|------|---------------|---------------------|
| Knee | direct hinge | the screw mechanism cannot be described in a URDF ([details](../../tools/fusion_to_urdf/README.md#the-knee-a-simplification-you-should-know-about)) |
| Joint limits | hip ±0.6 rad, thigh and knee ±1.4 rad | the CAD has no limits; measure them on the real robot |
| Front left leg | an older leg version, 1.5 cm longer | the 4 legs should be the same design |
| Motor mass | 54.6 g (AX-12A datasheet) | the CAD motor weighs 42 g |

## API

This package starts no nodes. `display.launch.xml` starts:

| Node | Publishes |
|------|-----------|
| `robot_state_publisher` | `/robot_description` (the URDF), `/tf` (position of every link) |
| `joint_state_publisher_gui` | `/joint_states` (the slider angles) |
| `rviz2` | nothing, it only displays |
