# Copyright 2021 Factor Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []

    # Arguments pour activer/désactiver les contrôleurs individuels
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_individual_controllers",
            default_value="true",
            description="Activer les contrôleurs individuels pour chaque joint",
        )
    )

    # Récupération des arguments
    enable_individual_controllers = LaunchConfiguration("enable_individual_controllers")

    # Description du robot - utilisation de votre URDF OpenDog
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("opendog_description"),
                    "urdf",
                    "opendog.urdf.xacro",
                ]
            ),
        ]
    )
    
    # Convertir en string pour éviter les problèmes de parsing
    robot_description_str = ParameterValue(robot_description_content, value_type=str)

    # Chemin vers le fichier de configuration des contrôleurs
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("odrive_demo_bringup"),
            "config",
            "odrive_multi_interface_forward_controllers.yaml",
        ]
    )

    # Paramètres combinés pour le nœud de contrôle
    controller_params = {
        "robot_description": robot_description_str,
        "robot_controller_config_file": robot_controllers,
    }

    # Nœud de contrôle ROS 2
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[controller_params, robot_controllers],
    )

    # Nœud de publication de l'état du robot
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_str}],
    )

    # Broadcasteur d'état des joints (toujours activé)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # Contrôleurs individuels pour chaque joint (même casse que dans le YAML)
    FR_hip_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FR_hip_joint_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_individual_controllers),
    )

    FL_hip_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FL_hip_joint_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_individual_controllers),
    )

    BR_hip_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["BR_hip_joint_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_individual_controllers),
    )

    BL_hip_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["BL_hip_joint_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_individual_controllers),
    )

    FR_uleg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FR_uleg_joint_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_individual_controllers),
    )

    FL_uleg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FL_uleg_joint_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_individual_controllers),
    )

    BR_uleg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["BR_uleg_joint_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_individual_controllers),
    )

    BL_uleg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["BL_uleg_joint_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_individual_controllers),
    )

    FR_lleg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FR_lleg_joint_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_individual_controllers),
    )

    FL_lleg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FL_lleg_joint_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_individual_controllers),
    )

    BR_lleg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["BR_lleg_joint_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_individual_controllers),
    )

    BL_lleg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["BL_lleg_joint_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_individual_controllers),
    )

    # NOTE: Le contrôleur global "all_joints_position_controller" a été retiré
    # afin d'utiliser uniquement les contrôleurs individuels (claimed individuellement).

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        FR_hip_controller_spawner,
        FL_hip_controller_spawner,
        BR_hip_controller_spawner,
        BL_hip_controller_spawner,
        FR_uleg_controller_spawner,
        FL_uleg_controller_spawner,
        BR_uleg_controller_spawner,
        BL_uleg_controller_spawner,
        FR_lleg_controller_spawner,
        FL_lleg_controller_spawner,
        BR_lleg_controller_spawner,
        BL_lleg_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
