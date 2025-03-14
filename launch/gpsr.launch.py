import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.descriptions import ParameterValue, ParameterFile

from ament_index_python.packages import get_package_share_directory
from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
    determine_use_sim_time_param,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)


def generate_launch_description():
    # Set ignition resource path
    gz_resource_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            '/usr/share/gazebo-11/models/',
            ':',
            str(Path(get_package_share_directory('interbotix_common_sim')).parent.resolve()),
            str(Path(get_package_share_directory('interbotix_xsarm_descriptions')).parent.resolve())
        ]
    )

    # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_URI',
        value=['']
    )

    # Set GAZEBO_MODEL_DATABASE_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=['']
    )
    hardware_type = DeclareLaunchArgument('hardware_type', default_value='gz_classic')

    small_house_package = get_package_share_directory('aws_robomaker_small_house_world')
    world_path = DeclareLaunchArgument('world_file_path', default_value=os.path.join(small_house_package, 'worlds', 'small_house.world'))
    boris_description_package = get_package_share_directory('boris_description')
    robot_model = DeclareLaunchArgument(
            'robot_model',
            default_value='wx200',
            choices=get_interbotix_xsarm_models(),
            description='model type of the Interbotix Arm such as `wx200` or `rx150`.'
        )
    use_world_frame = DeclareLaunchArgument('use_world_frame', default_value='false')
    external_urdf_loc = DeclareLaunchArgument('external_urdf_loc', default_value=os.path.join(boris_description_package, 'urdf', 'boris_arm.xacro'))
    moveit_package = get_package_share_directory('interbotix_xsarm_moveit')
    moveit_simulation = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(moveit_package, 'launch', 'xsarm_moveit.launch.py')
        ),
        launch_arguments={
            'world_filepath': LaunchConfiguration('world_file_path'),
            'use_world_frame': LaunchConfiguration('use_world_frame'),
            'external_urdf_loc': LaunchConfiguration('external_urdf_loc'),
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_name': LaunchConfiguration('robot_model'),
            'hardware_type': LaunchConfiguration('hardware_type'),
        }.items()
    )
    navigation_package = get_package_share_directory('fbot_navigation')
    navigation = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(navigation_package, 'launch', 'navigation.launch.py')
        )
    )
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cmd_vel',
        arguments=[
            '/cmd_vel', '/wx200/cmd_vel'
        ]
    )
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='relay_odom',
        arguments=[
            '/wx200/odom', '/hoverboard_base_controller/odom'
        ]
    )
    description_relay = Node(
        package='topic_tools',
        executable='relay',
        name='relay_description',
        arguments=[
            '/wx200/robot_description', '/robot_description'
        ]
    )
    scan_relay = Node(
        package='topic_tools',
        executable='relay',
        name='relay_scan',
        arguments=[
            '/lms1xx/scan', '/scan'
        ]
    )
    return LaunchDescription([gz_resource_path_env_var, gz_model_uri_env_var, hardware_type, use_world_frame, external_urdf_loc, robot_model, world_path, moveit_simulation, cmd_vel_relay, odom_relay, description_relay, scan_relay])
