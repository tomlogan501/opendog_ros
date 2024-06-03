
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('openleg_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'openleg.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'false'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

##############################################################
    load_joint_state_controller = ExecuteProcess(
    	cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
    	output='screen'
    )

    load_openleg_controller = ExecuteProcess(
    	cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'openleg_controller'],
    	output='screen'
    )
##############################################################

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'openleg',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    #walking_node = Node(
     #   package='openleg_description',
      #  executable='leg_walking.py',
       # arguments=[
       # ],
       # output='screen'
    #)

    joint_trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['openleg_controller'
        ],
        output='screen'
    )

    return LaunchDescription([
    ##########################################################
        RegisterEventHandler(                     #Event handler to load joint state controller after the robot is spawned
            event_handler=OnProcessExit(
            	target_action=urdf_spawn_node,
            	on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(			  #Event handler to load openDog controller after the joint state controller is loaded
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_openleg_controller],
            )
        ),
    ##########################################################
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        #walking_node,
    ])
