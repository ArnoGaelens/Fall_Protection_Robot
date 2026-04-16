import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('ass_robot_gazebo'),
        'worlds', 'living_room.sdf'
    )

    return LaunchDescription([
        # Start Gazebo Harmonic with the world
        ExecuteProcess(
            cmd=['gz', 'sim', world],
            output='screen'
        ),

        # Spawn the robot (requires ass_robot_description to publish robot_description)
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'ASS_Robot',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),
    ])
