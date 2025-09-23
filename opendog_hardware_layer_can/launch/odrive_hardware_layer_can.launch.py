# Copyright 2021 Factor Robotics

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    declared_arguments = []
    
    # Argument pour l'interface CAN
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="Interface CAN à utiliser",
        )
    )

    can_interface = LaunchConfiguration("can_interface")

    # Configuration CAN système
    can_setup = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', can_interface, 'up', 'type', 'can', 'bitrate', '500000'],
        output='screen'
    )

    # Description du robot avec URDF CAN
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("opendog_hardware_layer_can"),
                    "urdf", 
                    "opendog_can_hardware.urdf.xacro",
                ]
            ),
        ]
    )

    # Fichier de configuration des contrôleurs CAN
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("odrive_hardware_layer_can"),
            "config",
            "odrive_can_controllers.yaml",
        ]
    )

    # Nœud de contrôle ROS2 avec CAN
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"robot_description": robot_description_content},
            robot_controllers,
            {"can_interface": can_interface}
        ],
    )

    # Nœud de publication de l'état du robot
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
    )

    # Broadcasteur d'état des joints
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # 🔥 UN SEUL contrôleur de groupe
    all_joints_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["all_joints_controller", "-c", "/controller_manager"],
    )

    nodes = [
        # Configuration CAN d'abord
        can_setup,
        
        # Attendre que CAN soit configuré
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=can_setup,
                on_exit=[
                    control_node,
                    robot_state_pub_node,
                    joint_state_broadcaster_spawner,
                    all_joints_controller_spawner,
                ]
            )
        )
    ]

    return LaunchDescription(declared_arguments + nodes)