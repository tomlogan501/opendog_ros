# Copyright 2021 Factor Robotics

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

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

    # ✅ Description du robot avec URDF CAN (ARGUMENT use_can:=true AJOUTÉ)
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
            " use_can:=true",  # ← ✅ FORCE LE MODE CAN
            " use_gazebo:=false",
        ]
    )

    # Conversion en ParameterValue
    robot_description_str = ParameterValue(robot_description_content, value_type=str)

    # Fichier de configuration des contrôleurs CAN
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("opendog_hardware_layer_can"),
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
            {"robot_description": robot_description_str},
            robot_controllers,
            {"can_interface": can_interface}
        ],
    )

    # Nœud de publication de l'état du robot
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_str}],
    )

    # Broadcasteur d'état des joints
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # MODE READ ONLY : Ne pas charger le controller de position
    # all_joints_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["all_joints_controller", "-c", "/controller_manager"],
    # )

    # Séquence de démarrage avec délais
    nodes = [
        # 1. Démarrer control_node et robot_state_publisher
        control_node,
        robot_state_pub_node,

        # 2. Démarrer le broadcaster après que control_node soit prêt
        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster_spawner]
        ),

        # 3. MODE READ ONLY : Controller de position désactivé
        # TimerAction(
        #     period=5.0,
        #     actions=[all_joints_controller_spawner]
        # ),
    ]

    return LaunchDescription(declared_arguments + nodes)
