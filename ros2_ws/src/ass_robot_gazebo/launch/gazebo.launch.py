import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

os.environ['GZ_IP'] = '127.0.0.1'


def generate_launch_description():
    desc_share = get_package_share_directory('ass_robot_description')
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(desc_share)

    urdf = os.path.join(desc_share, 'urdf', 'ASS_Robot.urdf')
    with open(urdf, 'r') as f:
        robot_description = f.read()

    world = os.path.join(
        get_package_share_directory('ass_robot_gazebo'),
        'worlds', 'robot_world.sdf'
    )

    return LaunchDescription([
        # Publish robot URDF to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Start Gazebo with the world that already contains the robot
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            output='screen'
        ),

        # Bridge /cmd_vel from ROS2 to Gazebo
        Node(
            package='ass_robot_bringup',
            executable='cmd_vel_bridge',
            output='screen'
        ),
    ])
