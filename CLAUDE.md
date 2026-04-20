# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context

KU Leuven Technology Valorization project (Semester 6, 2025–2026). The goal is a ROS2 + Gazebo simulation of the **ASS_Robot** — a differential-drive robot with a rotating upper swivel section, modelling a wearable fall-protection system for robotics. See `0_ProjectInformation/` for all non-code assets (URDF source, pitch decks, BOM, sensor specs).

## Robot Architecture

The robot (`ASS_Robot`) was exported from SolidWorks via the sw_urdf_exporter. Key links:

| Link | Mass | Role |
|------|------|------|
| `base_link` | 28.8 kg | Main chassis, STL mesh |
| `Left_wheel` / `Right_wheel` | 1.17 kg each | Differential drive, continuous joints |
| `Castor` + `Castor_Wheel` | ~0.1 kg | Front passive castor, two continuous joints |
| `Upper_Swivel` | 27.7 kg | Rotating upper body, continuous joint on Z axis |
| `lidar_link` | 0.1 kg | Fixed to Upper_Swivel at z+0.17m, hosts LiDAR sensor |

The URDF source lives at `0_ProjectInformation/URDF_FILES/urdf/ASS_Robot.urdf`. STL meshes are in `0_ProjectInformation/URDF_FILES/meshes/`.

## Environment

- **ROS2 Jazzy** (machine runs Ubuntu 24.04)
- **Gazebo Harmonic** (gz-sim8, ros_gz_sim)
- Shell: bash (sourced automatically via `~/.bashrc`)
- Build system: colcon

## Workspaces

### ros2_ws3 (active workspace)
Workspace root: `ros2_ws3/`

```
ros2_ws3/src/mobile_robot/
  model/ass_robot_description/
    urdf/ASS_Robot.urdf.xacro     # main robot description (xacro)
    urdf/robot.gazebo              # all Gazebo plugins, sensors, friction, colors
    meshes/                        # STL mesh files
    launch/display.launch.py       # RViz preview launch
  launch/
    gazebo_model.launch.py         # main simulation launch file
  parameters/
    bridge_parameters.yaml         # ros_gz_bridge topic config
    controllers.yaml               # ros2_control controller definitions
  worlds/
    Test_Figuren.sdf               # test world with obstacles
  CMakeLists.txt                   # installs model, launch, parameters, worlds
```

### ros2_ws (old workspace — for reference only)
```
ros2_ws/src/
  ass_robot_description/   # URDF + STL meshes
  ass_robot_gazebo/        # Gazebo world SDF, launch file
  ass_robot_bringup/       # cmd_vel_bridge, path_follower
```

## Common Commands

```bash
# Source ROS2 + workspace (added to ~/.bashrc, runs automatically)
source /opt/ros/jazzy/setup.bash
source /home/arno/Documents/Technology_Valorization/Fall_Protection_Robot/ros2_ws3/install/setup.bash

# Build workspace
cd ~/Documents/Technology_Valorization/Fall_Protection_Robot/ros2_ws3
colcon build --symlink-install

# Launch simulation — default world (Test_Figuren.sdf)
ros2 launch mobile_robot gazebo_model.launch.py

# Launch with a different world
ros2 launch mobile_robot gazebo_model.launch.py world:=/full/path/to/world.sdf

# Launch with built-in empty world
ros2 launch mobile_robot gazebo_model.launch.py world:=default.sdf

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Control Upper_Swivel (rad/s, negative to reverse)
ros2 topic pub /upper_swivel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0]}"

# Stop Upper_Swivel
ros2 topic pub /upper_swivel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"

# Send velocity commands manually
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

# Check active topics
ros2 topic list

# Check LiDAR data
ros2 topic echo /scan

# Inspect joint states
ros2 topic echo /joint_states

# Verify xacro parses correctly
cd ~/Documents/Technology_Valorization/Fall_Protection_Robot/ros2_ws3
source install/setup.bash
xacro src/mobile_robot/model/ass_robot_description/urdf/ASS_Robot.urdf.xacro
```

## Key Implementation Details

### URDF / Xacro
- Main file is `ASS_Robot.urdf.xacro` — includes `robot.gazebo` via `xacro:include`
- All `<gazebo>` tags live in `robot.gazebo`, not in the main xacro
- Meshes referenced as `package://ass_robot_description/meshes/`
- Collision geometry uses primitives (box/cylinder/sphere), not STL — dartsim doesn't support mesh collisions
- Robot spawns at `z=0.17` to sit correctly on the ground plane

### Collision primitives (dartsim limitation)
| Link | Collision shape |
|------|----------------|
| `base_link` | box 0.40×0.45×0.15, offset x=0.05 z=-0.07 |
| `Left_wheel` / `Right_wheel` | cylinder r=0.0695 l=0.05 |
| `Castor` | box 0.06×0.04×0.05, offset z=-0.025 |
| `Castor_Wheel` | sphere r=0.025 |
| `Upper_Swivel` | box 0.40×0.40×0.30, offset at CoM |

### Gazebo plugins (in robot.gazebo)
- `gz-sim-diff-drive-system` — differential drive, topics: `cmd_vel`, `odom`, `tf`
- `gz-sim-joint-state-publisher-system` — publishes `joint_states` gz topic
- `gpu_lidar` sensor on `lidar_link` — topic: `scan`
- `gz_ros2_control-system` — ros2_control interface for Upper_Swivel

### ros_gz_bridge
Uses YAML config (`bridge_parameters.yaml`). No custom Python bridge needed.
Bridged topics:
- GZ → ROS2: `clock`, `joint_states`, `odom`, `tf`, `scan`
- ROS2 → GZ: `cmd_vel`

### ros2_control (Upper_Swivel)
- Hardware plugin: `gz_ros2_control/GazeboSimSystem`
- Controller: `velocity_controllers/JointGroupVelocityController`
- Command topic: `/upper_swivel_velocity_controller/commands` (Float64MultiArray)
- Config: `parameters/controllers.yaml`
- Requires: `ros-jazzy-gz-ros2-control`, `ros-jazzy-ros2-controllers`

### Wheel joints
- Both wheels use `rpy="-1.5708 0 0"` and `axis="0 0 1"` for symmetric differential drive
- Left wheel visual has `rpy="0 -0.22305 0"` to compensate for the removed joint Z rotation
- Left wheel CoM is at `z=-0.0387` (negative because local Z points opposite to right wheel)

### LiDAR
- Sensor type: `gpu_lidar`
- Mounted on `lidar_link`, fixed to `Upper_Swivel` at `xyz="0 0 0.17"`
- Rotates with Upper_Swivel (fixed joint)
- 360° scan, 10 Hz, 10m range, topic: `scan`

### GZ_SIM_RESOURCE_PATH
Set in `gazebo_model.launch.py` to `share/mobile_robot/model` so Gazebo resolves `model://ass_robot_description/meshes/` URIs correctly.

### World files
Saved in `worlds/` directory, installed via CMakeLists.txt.
Pass at launch time: `world:=/path/to/world.sdf`
To save a world from Gazebo GUI: press Enter instead of clicking OK (Qt5 dialog bug).

### Known warnings (harmless)
- `libEGL` / `dri2 screen` — NVIDIA GPU EGL fallback, cosmetic
- QML binding loop — Qt5 GUI cosmetic
- `gz_frame_id` not in SDF — Gazebo copies and uses it anyway
