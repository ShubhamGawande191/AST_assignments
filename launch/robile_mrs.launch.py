#!/usr/bin/env python3
# Authors: Deebul Nair

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_robot_nodes(robot_name, x_pose, y_pose):
    pkg_share = FindPackageShare("robile_safety").find("robile_safety")
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "4_wheel_config.urdf.xacro"])

    xacro_command = [
        FindExecutable(name="xacro"),
        TextSubstitution(text=" "),
        urdf_file,
        TextSubstitution(text=" "),
        TextSubstitution(text="robot_name:=" + robot_name)
    ]

    urdf_content = Command(xacro_command)

    # Wrap the URDF content in ParameterValue to explicitly define it as a string
    robot_description = {'robot_description': ParameterValue(urdf_content, value_type=str)}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        namespace=robot_name
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name, '-topic', 'robot_description', '-x', x_pose, '-y', y_pose],
        output='screen',
        namespace=robot_name
    )

    return GroupAction([robot_state_publisher, spawn_entity])

def generate_launch_description():
    # Set the number of robots
    number_of_robots = 3

    # Define the world
    world = os.path.join(get_package_share_directory('robile_gazebo'), 'worlds', 'closed_walls.world')

    # Common Gazebo nodes
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'))
    )

    # Generate robot nodes
    robot_nodes = []
    for i in range(number_of_robots):
        robot_name = f'robot_{i}'
        x_pose = str(1.0 * i)  
        y_pose = '-3.5'
        robot_nodes.append(generate_robot_nodes(robot_name, x_pose, y_pose))

    return LaunchDescription([
        gzserver,
        gzclient,
        *robot_nodes
    ])

    # urdf_path = os.path.join(
    #    get_package_share_directory('robile_description'),
    #    'robots',
    #    urdf_file_name)

    # with open(urdf_path, 'r') as infp:
    #    robot_desc = infp.read()

    # return LaunchDescription([
    #    DeclareLaunchArgument(
    #        'use_sim_time',
    #        default_value='false',
    #        description='Use simulation (Gazebo) clock if true'),

    #    Node(
    #        package='robot_state_publisher',
    #        executable='robot_state_publisher',
    #        name='robot_state_publisher',
    #        output='screen',
    #        parameters=[{
    #            'use_sim_time': use_sim_time,
    #            'robot_description': robot_desc
    #        }],
    #    ),
    # ])