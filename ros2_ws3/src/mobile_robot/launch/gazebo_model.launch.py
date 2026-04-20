################################################################################
# ROS2 and Gazebo Launch File of the differential drive robot
# Adapted from Aleksandar Haber's tutorial for ASS_Robot
################################################################################

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # this name has to match the robot name in the Xacro file
    robotXacroName = 'ASS_Robot'

    # this is the name of our package, at the same time this is the name of the
    # folder that will be used to define the paths
    namePackage = 'mobile_robot'

    # this is a relative path to the xacro file defining the model
    modelFileRelativePath = 'model/ass_robot_description/urdf/ASS_Robot.urdf.xacro'

    # this is the absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

    # launch argument for the world file — defaults to Test_Figuren.sdf
    declareWorldArg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory(namePackage), 'worlds', 'Test_Figuren.sdf'),
        description='Full path to the Gazebo world SDF file'
    )

    pathWorldFile = LaunchConfiguration('world')

    # get the robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # this is the launch file from the gazebo_ros package
    gazebo_rospackage_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )

    # this is the launch description

    # this is if you are using your own world model
    gazeboLaunch = IncludeLaunchDescription(gazebo_rospackage_launch,
        launch_arguments={'gz_args': ['-r -v -v4 ', pathWorldFile]}.items())

    # this is if you are using an empty world model
    # gazeboLaunch = IncludeLaunchDescription(gazebo_rospackage_launch,
    #     launch_arguments={'gz_args': '-r -v -v4 default.sdf'}.items())

    # node for the robot state publisher
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}],
    )

    # node to spawn the robot model in Gazebo
    nodeSpawnModel = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
            '-z', '0.17',
        ],
        output='screen',
    )

    # this is very important so we can control the robot from ROS2
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # set resource path so Gazebo can find meshes via model://ass_robot_description/
    setGazeboResourcePath = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(get_package_share_directory(namePackage), 'model')
    )

    # node to activate the upper swivel velocity controller
    spawnUpperSwivelController = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['upper_swivel_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # here we create an empty launch description object
    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(declareWorldArg)
    launchDescriptionObject.add_action(setGazeboResourcePath)

    # we add gazeboLaunch
    launchDescriptionObject.add_action(gazeboLaunch)

    # we add the two nodes
    launchDescriptionObject.add_action(nodeSpawnModel)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    launchDescriptionObject.add_action(spawnUpperSwivelController)

    return launchDescriptionObject
