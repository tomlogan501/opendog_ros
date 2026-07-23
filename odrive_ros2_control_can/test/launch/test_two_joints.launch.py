from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare("odrive_ros2_control_can")
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "two_joints_test.urdf.xacro"]),
        ]
    )
    robot_description = ParameterValue(robot_description_content, value_type=str)
    robot_controllers = PathJoinSubstitution(
        [pkg_share, "config", "test_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            robot_controllers,
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    controller_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(period=0.3, actions=[joint_state_broadcaster_spawner]),
                TimerAction(period=0.3, actions=[forward_position_controller_spawner]),
            ],
        )
    )

    return LaunchDescription([
        LogInfo(msg=(
            "Before launch: CAN up, odrivetool closed, ODrive rebooted. "
            "Recommended: odrv0.axis0/1.config.startup_closed_loop_control = False"
        )),
        control_node,
        robot_state_pub_node,
        controller_spawners,
    ])
