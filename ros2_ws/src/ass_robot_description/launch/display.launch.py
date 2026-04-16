import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('ass_robot_description'),
        'urdf', 'ASS_Robot.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Publishes the URDF to the /robot_description topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # GUI to manually move joints (useful for testing without a controller)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])
