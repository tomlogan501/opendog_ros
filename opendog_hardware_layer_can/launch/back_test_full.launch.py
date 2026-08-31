"""
Back-of-robot hardware test: BL + BR legs (6 joints) across ODrive
4, 2 and 5. ODrive 4 has a known unresolved CAN wiring fault --
watch candump/can0 error counters closely during activation.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_hw = get_package_share_directory('opendog_hardware_layer_can')

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hw, 'launch', 'back_test.launch.py')
        )
    )

    joy_node = Node(package='joy', executable='joy_node',
                    name='joy_node', output='screen')

    teleop_joy_node = Node(package='opendog_teleop', executable='opendog_teleop_joy_node',
                           name='opendog_teleop_joy_node', output='screen')
    start_teleop_after_joy = RegisterEventHandler(
        OnProcessStart(target_action=joy_node, on_start=[teleop_joy_node])
    )

    cmd_manager_node = Node(package='opendog_control', executable='cmd_manager_node',
                            name='cmd_manager_node', output='screen')

    ik_node = Node(package='opendog_control', executable='IK_node',
                   name='IK_node', output='screen')

    joint_ctrl_bridge_node = Node(package='opendog_hardware_layer_can',
                                  executable='back_test_joint_ctrl_node',
                                  name='back_test_joint_ctrl_node', output='screen')

    return LaunchDescription([
        hardware_launch,
        joy_node,
        start_teleop_after_joy,
        TimerAction(period=8.0, actions=[cmd_manager_node, ik_node]),
        TimerAction(period=10.0, actions=[joint_ctrl_bridge_node]),
    ])
