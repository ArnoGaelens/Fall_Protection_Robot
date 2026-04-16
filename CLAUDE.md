# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context

KU Leuven Technology Valorization project (Semester 6, 2025–2026). The goal is a ROS2 Jazzy + Gazebo Harmonic simulation of the **ASS_Robot** — a differential-drive robot with a rotating upper swivel section, modelling a wearable fall-protection system for robotics. See `0_ProjectInformation/` for all non-code assets (URDF source, pitch decks, BOM, sensor specs).

## Robot Architecture

The robot (`ASS_Robot`) was exported from SolidWorks via the sw_urdf_exporter. Key links:

| Link | Mass | Role |
|------|------|------|
| `base_link` | 28.8 kg | Main chassis, STL mesh |
| `Left_wheel` / `Right_wheel` | 1.17 kg each | Differential drive, continuous joints |
| `Castor` + `Castor_Wheel` | ~0.1 kg | Front passive castor, two continuous joints |
| `Upper_Swivel` | 27.7 kg | Rotating upper body, continuous joint on Z axis |

The URDF source lives at `0_ProjectInformation/URDF_FILES/urdf/ASS_Robot.urdf`. STL meshes are in `0_ProjectInformation/URDF_FILES/meshes/`.

**Important:** The existing `package.xml` and `CMakeLists.txt` are ROS1 (catkin). Everything in the simulation workspace must use ROS2 (ament_cmake / ament_python).

## Environment

- ROS2 Jazzy + Gazebo Harmonic (gz-harmonic)
- Display: WSLg (no extra X server needed)
- Build system: colcon
- Workspace root: `ros2_ws/` (to be created inside this folder)

## Common Commands

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build workspace
cd ros2_ws && colcon build --symlink-install

# Source workspace overlay
source ros2_ws/install/setup.bash

# Launch Gazebo simulation
ros2 launch ass_robot_gazebo gazebo.launch.py

# Visualize in RViz2
ros2 launch ass_robot_description rviz.launch.py

# Check URDF validity
check_urdf ros2_ws/src/ass_robot_description/urdf/ASS_Robot.urdf

# Inspect joint states
ros2 topic echo /joint_states

# Send velocity commands (differential drive)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

## Package Structure (planned)

```
ros2_ws/src/
  ass_robot_description/   # URDF, meshes, RViz config
  ass_robot_gazebo/        # Gazebo world, launch files, gz_ros2_control config
  ass_robot_bringup/       # Top-level launch combining description + simulation
```

## Key Migration Notes (ROS1 → ROS2)

- Replace `gazebo_ros` spawn with `ros_gz_sim` (`gz_spawn_entity.py` node or `create` service)
- Replace `robot_state_publisher` launch args with URDF passed via `robot_description` parameter
- `joint_state_publisher_gui` still works in ROS2 — use for testing without a controller
- Mesh paths in URDF: keep `package://ass_robot_description/meshes/` pattern (works with `ros_gz_sim`)
- The `Upper_Swivel` joint has no limits and no actuator defined yet — decide whether it is passive or needs a controller before wiring `ros2_control`
