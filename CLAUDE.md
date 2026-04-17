# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context

KU Leuven Technology Valorization project (Semester 6, 2025–2026). The goal is a ROS2 Humble + Gazebo Harmonic simulation of the **ASS_Robot** — a differential-drive robot with a rotating upper swivel section, modelling a wearable fall-protection system for robotics. See `0_ProjectInformation/` for all non-code assets (URDF source, pitch decks, BOM, sensor specs).

## Robot Architecture

The robot (`ASS_Robot`) was exported from SolidWorks via the sw_urdf_exporter. Key links:

| Link | Mass | Role |
|------|------|------|
| `base_link` | 28.8 kg | Main chassis, STL mesh |
| `Left_wheel` / `Right_wheel` | 1.17 kg each | Differential drive, continuous joints |
| `Castor` + `Castor_Wheel` | ~0.1 kg | Front passive castor, two continuous joints |
| `Upper_Swivel` | 27.7 kg | Rotating upper body, continuous joint on Z axis |
| `lidar_link` | 0.1 kg | Fixed to Upper_Swivel at z+0.3m, hosts LiDAR sensor |

The URDF source lives at `0_ProjectInformation/URDF_FILES/urdf/ASS_Robot.urdf`. STL meshes are in `0_ProjectInformation/URDF_FILES/meshes/`.
The working URDF is at `ros2_ws/src/ass_robot_description/urdf/ASS_Robot.urdf` — this is the one that matters for simulation.

**Important:** The existing `package.xml` and `CMakeLists.txt` in the repo root are ROS1 (catkin). Everything in the simulation workspace uses ROS2 (ament_cmake / ament_python).

## Environment

- **ROS2 Humble** (not Jazzy — Jazzy requires Ubuntu 24.04, machine runs 22.04)
- **Gazebo Harmonic** (gz-sim8, version 8.11.0)
- Shell: bash (zsh removed)
- Build system: colcon
- Workspace root: `ros2_ws/`

## Common Commands

```bash
# Source ROS2 + workspace (always needed in a new terminal)
source /opt/ros/humble/setup.bash
source /home/arno/Documents/Technology_Valorization/Fall_Protection_Robot/ros2_ws/install/setup.bash

# Build workspace
cd ros2_ws && colcon build --symlink-install

# Launch full simulation (Gazebo + robot_state_publisher + bridges)
ros2 launch ass_robot_gazebo gazebo.launch.py

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Run automated path
ros2 run ass_robot_bringup path_follower

# Check what topics exist
ros2 topic list

# Check LiDAR data
ros2 topic echo /scan

# Send velocity commands manually
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

# Inspect joint states
ros2 topic echo /joint_states
```

## Package Structure

```
ros2_ws/src/
  ass_robot_description/   # URDF + STL meshes
  ass_robot_gazebo/        # Gazebo world SDF, launch file
  ass_robot_bringup/       # cmd_vel_bridge, path_follower
```

## World SDF Regeneration

The world SDF embeds the full robot model (for persistence across simulation resets).
After any URDF change, regenerate it:

```bash
cd ros2_ws
colcon build --symlink-install && source install/setup.bash
gz sdf -p install/ass_robot_description/share/ass_robot_description/urdf/ASS_Robot.urdf 2>/dev/null > /tmp/ASS_Robot.sdf
python3 /tmp/regen_world.py
```

`/tmp/regen_world.py` contains the world template (physics, plugins, ground plane) and injects the converted robot model. **Keep this script updated** when adding world-level plugins.

## Key Implementation Details

### cmd_vel bridge
`ros_gz_bridge` has a message type incompatibility for Twist (`ignition.msgs` vs `gz.msgs`). A custom Python bridge in `ass_robot_bringup/cmd_vel_bridge.py` uses `gz.transport13` directly to forward `/cmd_vel` from ROS2 to Gazebo.

### Wheel joints
- Both wheels use `rpy="-1.5708 0 0"` and `axis="0 0 1"` for symmetric differential drive
- Left wheel visual has `rpy="0 -0.22305 0"` to compensate for the removed joint Z rotation
- Wheel collisions are cylinders (not STL meshes) to ensure centered contact point
- Left wheel CoM is at `z=-0.0387` (negative because local Z points opposite to right wheel)

### Upper_Swivel
- `damping=5.0, friction=2.0` on the joint to prevent free rotation
- No actuator defined yet — passive in current simulation

### LiDAR
- Sensor type: `lidar` (CPU-based, not `gpu_lidar`) for compatibility
- Mounted on `lidar_link`, fixed to `Upper_Swivel` at `xyz="0 0 0.3"`
- 360° scan, 10 Hz, 10m range
- Bridged via `ros_gz_bridge`: `/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan`
- Requires `gz-sim-sensors-system` plugin in the world SDF

### GZ_SIM_RESOURCE_PATH
Set in `gazebo.launch.py` to the parent of the `ass_robot_description` share dir so that `model://ass_robot_description/meshes/` URIs resolve correctly.
