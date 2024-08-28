import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)
from launch.conditions import IfCondition


def generate_launch_description():

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("odrive_demo_description"), 
                    "urdf",
                    "odrive_opendog.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("odrive_demo_bringup"),
            "config",
            "opendog_controllers.yaml",
        ]
    )
    print(robot_controllers)

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    print(control_node)
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    print(robot_state_pub_node)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    print(joint_state_broadcaster_spawner)

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )
    print(robot_controller_spawner)
    
    # Delay joint_state_broadcaster_spawner after control_node
    delay_broadcaster_after_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
# Control part
    node_joy = ExecuteProcess(
        cmd=['ros2', 'run', 'joy', 'joy_node'],
        output='screen'
    )

    node_opendog_teleop_joy = ExecuteProcess(
        cmd=['ros2', 'run', 'opendog_teleop', 'opendog_teleop_joy_node'],
        output='screen'
    )

    node_opendog_control = ExecuteProcess(
        cmd=['ros2', 'run', 'opendog_control', 'cmd_manager_node'],
        output='screen'
    )

    node_IK_node = ExecuteProcess(
        cmd=['ros2', 'run', 'opendog_control', 'IK_node'],
        output='screen'
    )
    opendog_gazebo_joint_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'opendog_gazebosim', 'opendog_joint_ctrl_node'],
        output='screen'
    )

    laod_forward_command_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
            'openDog_controller'],
        output='screen'
    )

    return LaunchDescription([
    	control_node,
        robot_state_pub_node,
        #delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        joint_state_broadcaster_spawner,
        node_joy,
        node_opendog_teleop_joy,
        delay_broadcaster_after_control_node,
        opendog_gazebo_joint_cmd,

        RegisterEventHandler(
            OnProcessStart(
            target_action=node_opendog_teleop_joy,
            on_start=[
                    LogInfo(msg='node_opendog_teleop_joy started, starting opendog_control'),
                    node_opendog_control,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
            target_action=node_opendog_control,
            on_start=[
                    LogInfo(msg='opendog_control started, starting IK_node'),
                    node_IK_node
                ]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[laod_forward_command_controller],
            )
        ),
    ])
