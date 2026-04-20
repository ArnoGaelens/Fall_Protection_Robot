import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import SetParameter


def generate_launch_description():

    nav2_params = os.path.join(
        get_package_share_directory('mobile_robot'),
        'parameters', 'nav2_params.yaml'
    )

    configured_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={},
        convert_types=True,
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetParameter(name='use_sim_time', value=True),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
        ),
    ])
