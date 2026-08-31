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

    # Argument to enable/disable individual controllers
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_individual_controllers",
            default_value="true",
            description="Enable individual controllers for each joint",
        )
    )

    # Retrieve arguments
    enable_individual_controllers = LaunchConfiguration("enable_individual_controllers")

    # Robot description - uses the OpenDog URDF
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
    
    # Convert to string to avoid parsing issues
    robot_description_str = ParameterValue(robot_description_content, value_type=str)

    # Path to the controller configuration file
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("odrive_demo_bringup"),
            "config",
            "odrive_multi_interface_forward_controllers.yaml",
        ]
    )

    # Combined parameters for the control node
    controller_params = {
        "robot_description": robot_description_str,
        "robot_controller_config_file": robot_controllers,
    }

    # ROS 2 control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[controller_params, robot_controllers],
    )

    # Robot state publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_str}],
    )

    # Joint state broadcaster (always enabled)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # Individual controllers for each joint (same casing as in the YAML)
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

    # NOTE: The global "all_joints_position_controller" controller was removed
    # so that only the individual controllers are used (claimed individually).

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
