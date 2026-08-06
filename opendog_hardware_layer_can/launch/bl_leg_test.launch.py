from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface to use",
        )
    )

    can_interface = LaunchConfiguration("can_interface")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("opendog_hardware_layer_can"),
                    "urdf",
                    "bl_leg_test.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description_str = ParameterValue(robot_description_content, value_type=str)

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("opendog_hardware_layer_can"),
            "config",
            "bl_leg_test_controllers.yaml",
        ]
    )

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

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_str}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    bl_leg_test_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bl_leg_test_controller", "-c", "/controller_manager"],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=5.0, actions=[bl_leg_test_controller_spawner]),
    ]

    return LaunchDescription(declared_arguments + nodes)
