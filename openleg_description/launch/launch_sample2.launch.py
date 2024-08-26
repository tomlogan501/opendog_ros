import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


import xacro
import yaml


def generate_launch_description():

    ########
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])])
    )

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('openleg_description'), 'urdf', 'openleg.xacro'
        ])
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    robot_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controler', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        robot_description,
        robot_state_publisher_node,
        robot_spawn_node,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller
    ])