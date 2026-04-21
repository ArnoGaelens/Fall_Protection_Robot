import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('mobile_robot')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gazebo_model.launch.py')
        )
    )

    # Nav2 + RViz delayed so Gazebo has time to start
    nav2 = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'nav2.launch.py')
                )
            )
        ]
    )

    # Person sim delayed until Nav2 is up
    person_sim = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='mobile_robot',
                executable='person_sim.py',
                name='person_sim',
                output='screen',
            )
        ]
    )

    # Follower starts after person sim
    person_follower = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='mobile_robot',
                executable='person_follower.py',
                name='person_follower',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        nav2,
        person_sim,
        person_follower,
    ])
