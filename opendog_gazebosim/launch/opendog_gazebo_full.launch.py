"""
Full OpenDog V3 simulation pipeline: launches Gazebo + the complete
joystick-to-motion control chain (joy_node -> opendog_teleop_joy_node ->
cmd_manager_node -> IK_node -> opendog_gazebo_joint_ctrl_node), with
timing/ordering matching what was empirically found necessary during
development.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_gazebosim = get_package_share_directory('opendog_gazebosim')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebosim, 'launch', 'opendog_gazebosim_allPkgs.launch.py')
        )
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    teleop_joy_node = Node(
        package='opendog_teleop',
        executable='opendog_teleop_joy_node',
        name='opendog_teleop_joy_node',
        output='screen',
    )
    start_teleop_after_joy = RegisterEventHandler(
        OnProcessStart(
            target_action=joy_node,
            on_start=[teleop_joy_node],
        )
    )

    cmd_manager_node = Node(
        package='opendog_control',
        executable='cmd_manager_node',
        name='cmd_manager_node',
        output='screen',
    )

    ik_node = Node(
        package='opendog_control',
        executable='IK_node',
        name='IK_node',
        output='screen',
    )

    joint_ctrl_bridge_node = Node(
        package='opendog_gazebosim',
        executable='opendog_gazebo_joint_ctrl_node',
        name='opendog_gazebo_joint_ctrl_node',
        output='screen',
    )

    delayed_control_stack = TimerAction(
        period=8.0,
        actions=[cmd_manager_node, ik_node],
    )

    delayed_bridge = TimerAction(
        period=10.0,
        actions=[joint_ctrl_bridge_node],
    )

    return LaunchDescription([
        gazebo_launch,
        joy_node,
        start_teleop_after_joy,
        delayed_control_stack,
        delayed_bridge,
    ])
