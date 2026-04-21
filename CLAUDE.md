# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context

KU Leuven Technology Valorization project (Semester 6, 2025–2026). The goal is a ROS2 + Gazebo simulation of the **ASS_Robot** — a differential-drive robot with a rotating upper swivel section, modelling a wearable fall-protection system for robotics. See `0_ProjectInformation/` for all non-code assets (URDF source, pitch decks, BOM, sensor specs).

## Robot Architecture

| Link | Mass | Role |
|------|------|------|
| `base_link` | 28.8 kg | Main chassis, STL mesh |
| `Left_wheel` / `Right_wheel` | 1.17 kg each | Differential drive, continuous joints |
| `Castor` + `Castor_Wheel` | ~0.1 kg | Front passive castor, two continuous joints |
| `Upper_Swivel` | 27.7 kg | Rotating upper body, continuous joint on Z axis |
| `lidar_link` | 0.1 kg | Fixed to Upper_Swivel at z+0.17m, hosts LiDAR sensor |

## Environment

- **ROS2 Jazzy** (Ubuntu 24.04)
- **Gazebo Harmonic** (gz-sim8, ros_gz_sim)
- Shell: bash (sourced via `~/.bashrc`)
- Build system: colcon

## Workspace Structure

```
ros2_ws3/src/
  mobile_robot/
    model/ass_robot_description/
      urdf/ASS_Robot.urdf.xacro     # main robot description
      urdf/robot.gazebo              # all Gazebo plugins, sensors, colors
    launch/
      gazebo_model.launch.py         # Gazebo simulation
      nav2.launch.py                 # Nav2 + RViz
      full_system.launch.py          # everything in one command
    parameters/
      bridge_parameters.yaml         # ros_gz_bridge config
      controllers.yaml               # ros2_control definitions
      nav2_params.yaml               # Nav2 configuration
      nav2_rviz.rviz                 # RViz2 config
    scripts/
      person_sim.py                  # simulated person walking in circle
      person_follower.py             # reactive follower publishing /cmd_vel
    worlds/
      Test_Figuren.sdf               # test world (box, cone, cylinder, person model)
  ass_robot_description/
    meshes/                          # STL mesh files (standalone package for URI resolution)
    package.xml / CMakeLists.txt
```

## Common Commands

```bash
# Build (always from ros2_ws3)
cd ~/Documents/Technology_Valorization/Fall_Protection_Robot/ros2_ws3
colcon build --symlink-install

# Launch everything (Gazebo + Nav2 + RViz + person sim + follower)
ros2 launch mobile_robot full_system.launch.py

# Launch components separately
ros2 launch mobile_robot gazebo_model.launch.py
ros2 launch mobile_robot nav2.launch.py        # also opens RViz
ros2 run mobile_robot person_sim.py
ros2 run mobile_robot person_follower.py

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Control Upper_Swivel
ros2 topic pub /upper_swivel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0]}"
ros2 topic pub /upper_swivel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"

# Debug
ros2 topic list
ros2 topic hz /tf
ros2 topic hz /joint_states
ros2 topic echo /scan
```

## Key Implementation Details

### URDF / Xacro
- Main: `ASS_Robot.urdf.xacro` — includes `robot.gazebo` via `xacro:include`
- Meshes: `package://ass_robot_description/meshes/` (resolved via standalone package)
- Collision: primitives only — dartsim doesn't support STL mesh collisions
- Spawn height: `z=0.17`

### Collision Primitives
| Link | Shape |
|------|-------|
| `base_link` | box 0.40×0.45×0.15, offset x=0.05 z=-0.07 |
| `Left_wheel` / `Right_wheel` | cylinder r=0.0695 l=0.05 |
| `Castor` | box 0.06×0.04×0.05, offset z=-0.025 |
| `Castor_Wheel` | sphere r=0.025 |
| `Upper_Swivel` | box 0.40×0.40×0.30 |

### Gazebo Plugins (robot.gazebo)
- `gz-sim-diff-drive-system` — cmd_vel, odom, tf
- `gz-sim-joint-state-publisher-system` — joint_states
- `gpu_lidar` on `lidar_link` — 360°, 10Hz, 10m, topic: `scan`
- `gz_ros2_control-system` — Upper_Swivel velocity control

### Bridge (bridge_parameters.yaml) — Gazebo Harmonic topic names
| Topic | GZ topic | Direction |
|-------|----------|-----------|
| `/tf` | `/model/ASS_Robot/tf` | GZ→ROS |
| `/joint_states` | `/world/empty/model/ASS_Robot/joint_state` | GZ→ROS |
| `/odom` | `odom` | GZ→ROS |
| `/scan` | `scan` | GZ→ROS |
| `/cmd_vel` | `cmd_vel` | ROS→GZ |
| `/clock` | `clock` | GZ→ROS |

**Critical**: GZ Harmonic uses model-scoped topic names. TF and joint_states must use the full paths above or they won't bridge.

### ros2_control (Upper_Swivel)
- Plugin: `gz_ros2_control/GazeboSimSystem`
- Controller: `velocity_controllers/JointGroupVelocityController`
- Topic: `/upper_swivel_velocity_controller/commands` (Float64MultiArray)
- Requires: `ros-jazzy-gz-ros2-control`, `ros-jazzy-ros2-controllers`

### LiDAR
- Mounted on `lidar_link`, fixed to `Upper_Swivel` at `xyz="0 0 0.17"`
- Rotates with Upper_Swivel — lock swivel at 0 for navigation

### Nav2 (mapless mode)
- No map server, no AMCL — rolling costmaps only
- Custom `nav2.launch.py` — do NOT use `nav2_bringup/navigation_launch.py` (collision_monitor crashes)
- Launches: controller_server, planner_server, behavior_server, bt_navigator only
- Planner: NavFn with A*
- Controller: Regulated Pure Pursuit (RPP)
- `observation_persistence: 0.0` on scan sources — clears old obstacle marks immediately

### Person Following (reactive controller)
- Architecture: `person_sim.py` → `/person_pose` → `person_follower.py` → `/cmd_vel`
- **No Nav2 path planning** — direct reactive velocity control
- Stays 40cm from person, stops if obstacle within 30cm (excluding person direction)
- Person sim moves a red cylinder in Gazebo via `gz service /world/empty/set_pose`
- Person direction is excluded from obstacle check (LiDAR would otherwise see person as obstacle)

### RViz2
- Fixed Frame: `odom`
- Config auto-loaded from `parameters/nav2_rviz.rviz`
- Displays: RobotModel, LaserScan (`/scan`), LocalCostmap, GlobalCostmap, GlobalPath, PersonPose
- Robot appears slightly underground in RViz (cosmetic, z-offset in odom frame) — harmless

### Known Issues / Warnings
- Robot visually underground in RViz — cosmetic only, navigation unaffected
- `libEGL` / `dri2 screen` — NVIDIA GPU EGL fallback, harmless
- QML binding loop — Qt5 GUI cosmetic
- `gz_frame_id` not in SDF — Gazebo handles it anyway

## Person-Following Status

| Step | Status | Description |
|------|--------|-------------|
| 1 | ✅ | Gazebo simulation (LiDAR, odom, cmd_vel, bridges) |
| 2 | ✅ | LiDAR visualization in Gazebo |
| 3 | ✅ | Nav2 mapless mode |
| 4 | ✅ | Obstacle avoidance + RViz costmap |
| 5 | ✅ | RViz with auto-loaded config |
| 6 | ✅ | Person sim node (red cylinder walking in circle) |
| 7 | ✅ | Reactive person follower (direct /cmd_vel, no path planning) |
| 8 | 🔲 | Final tuning for demo |
