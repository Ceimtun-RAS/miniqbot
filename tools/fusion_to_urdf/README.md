# fusion_to_urdf

Tools that turn the robot's 3D model from **Autodesk Fusion** into a **URDF**
file, so ROS 2 can show the robot in RViz and simulate it in Gazebo.

You do **not** need to understand the Python code to use these tools. You only
need to follow the steps below.

## Words you will see

| Word | Meaning |
|------|---------|
| **CAD** | The 3D design of the robot, made in Fusion. It has every screw, motor and plastic part. |
| **URDF** | A text file that describes a robot for ROS: its rigid blocks and how they move. |
| **Link** | One rigid block of the robot. Everything inside a link moves together (example: the thigh). |
| **Joint** | The hinge between two links (example: the knee). Each motor turns one joint. |
| **Mesh** | The 3D shape of a link, stored as an `.stl` file. It is only used to draw the robot. |

## The big idea

The CAD has **62 parts**. ROS does not care about every screw: it only needs to
know **which parts move together**. So we group the parts into **13 links**:
the body, and a hip, a thigh and a shin for each of the 4 legs.

Think of your own arm: it has many bones, muscles and tendons, but to explain
how it moves you only say "upper arm, forearm and hand, joined by the shoulder,
the elbow and the wrist". The URDF is that short description.

```
   STEP 1: export (in Fusion)        STEP 2: convert (on Linux)          RESULT
   -------------------------         --------------------------          ------
   MiniQbotExport copies the         build_urdf.py groups the 62         src/miniqbot_description/
   design to plain files:            parts into 13 links, following      ├── urdf/miniqbot.urdf
   - 3D shape of every part          the rules in robot_config.yaml,     └── meshes/*.stl
   - mass of every part              and writes the URDF
   - every joint
        (in centimeters)                  (in meters, ROS axes)
```

Why two steps? Step 1 must run inside Fusion (Windows or macOS). Step 2 runs on
any computer with Python, can be tested automatically, and holds all the rules
about our robot in one readable file: `robot_config.yaml`.

## How to use it

### Step 1: export the design from Fusion

1. Open the robot design (**"Fixed for URDF"**) in Fusion desktop. Fusion in the
   web browser cannot run scripts.
2. Press **Shift + S** to open **Scripts and Add-Ins**.
3. The first time only: click **+** (next to the search box), choose
   **"Script or add-in from device"** and select the folder
   `tools/fusion_to_urdf/MiniQbotExport`.
4. Select **MiniQbotExport** in the list and click **Run**.
5. Wait until a message says **"Export finished"** (around 1 minute).

The script only **reads** the design, it never changes it. The files go to
`Downloads/miniqbot_export/` in your user folder.

### Step 2: build the URDF

From the root of the repository:

```bash
python3 tools/fusion_to_urdf/build_urdf.py --export ~/Downloads/miniqbot_export
```

It prints a table with every link, its number of parts and its mass, and writes
the files into `src/miniqbot_description/`. It needs Python 3 with `numpy` and
`pyyaml` (both come with ROS 2).

### Step 3: look at the robot

See [`src/miniqbot_description/README.md`](../../src/miniqbot_description/README.md).

## When the CAD changes

If someone renames, adds or removes a part, `build_urdf.py` **stops and tells
you what changed**, for example:

```
ERROR: link fl_shin_link names the part 'Pata v12:1', which is not in the CAD. Was it renamed?
```

Fix it by editing **`robot_config.yaml`**. It is a plain text file with a comment
on every section. Then run step 2 again. You should almost never need to edit
the Python files.

## The knee: a simplification you should know about

In the current leg, the knee is not moved by a motor directly: the third motor
turns a **screw** that pushes a rod, and the rod moves the shin. This makes a
closed loop (the shin is held by the thigh from two sides).

A URDF can only describe a **tree** (every link has exactly one parent), so it
cannot describe that loop. We simplify it: the screw mechanism is frozen into
the thigh, and the knee becomes a **direct hinge**. The pushrod is not drawn
(its mass still counts), because a frozen rod would look broken when the knee
moves. The new leg design (motor directly at the knee) will not need this.

## Files

| File | What it is |
|------|-----------|
| `MiniQbotExport/` | Step 1. The script that runs inside Fusion. |
| `build_urdf.py` | Step 2. The converter. |
| `robot_config.yaml` | The rules about our robot: which part belongs to which link, joint names, joint limits. **This is the file you edit.** |
| `test_build_urdf.py` | Automatic tests for the converter. |

## Tests

```bash
python3 -m unittest discover -s tools/fusion_to_urdf -v
```

The tests build a tiny fake robot made of boxes, so they run without Fusion.
