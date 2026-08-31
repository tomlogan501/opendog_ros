# Copyright 2026 Reebot

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    declared_arguments = []

    # CAN interface argument
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface to use",
        )
    )

    can_interface = LaunchConfiguration("can_interface")

    # Robot description with CAN URDF (CAN is now the only hardware path)
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("opendog_description"),
                    "urdf",
                    "opendog.urdf.xacro",
                ]
            ),
            " use_gazebo:=false",
        ]
    )

    # Convert to ParameterValue
    robot_description_str = ParameterValue(robot_description_content, value_type=str)

    # CAN controller configuration file
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("opendog_hardware_layer_can"),
            "config",
            "odrive_can_controllers.yaml",
        ]
    )

    # ROS2 control node with CAN
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"robot_description": robot_description_str},
            robot_controllers,
            {"can_interface": can_interface}
        ],
    )

    # Robot state publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_str}],
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # READ ONLY MODE: do not load the position controller
    # all_joints_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["all_joints_controller", "-c", "/controller_manager"],
    # )

    # Startup sequence with delays
    nodes = [
        # 1. Start control_node and robot_state_publisher
        control_node,
        robot_state_pub_node,

        # 2. Start the broadcaster once control_node is ready
        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster_spawner]
        ),

        # 3. READ ONLY MODE: position controller disabled
        # TimerAction(
        #     period=5.0,
        #     actions=[all_joints_controller_spawner]
        # ),
    ]

    return LaunchDescription(declared_arguments + nodes)
