from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

import os
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_file_name = 'openleg.xacro'
    urdf = os.path.join(
        get_package_share_directory('openleg_description'),
        'urdf',
        urdf_file_name)
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['cat ', urdf])}]
    )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['robot_description', '-entity', 'openleg', '-file', urdf],
                        output='screen')
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[PathJoinSubstitution([FindPackageShare('openleg_description'), 'config', 'openleg_controller.yaml'])],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )
    #########################
    #joint_state_broadcaster_spawner = Node(
    #    package='controller_manager',
    #    executable='spawner',
    #    arguments=['joint_state_broadcaster'],
    #    output='screen'
    #)
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )
    ##########################
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2','control','load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    #load_diff_drive_controller = ExecuteProcess(
    #    cmd=['ros2','control','load_controller', '--set-state', 'start',
    #         'diff_drive_controller'],
    #    output='screen'
    #)
    ##########################
    return LaunchDescription([
    #Event Handlers
    	RegisterEventHandler(
          event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
          )
      	),
    #  	RegisterEventHandler(
    #      event_handler=OnProcessExit(
    #            target_action=spawn_entity,
    #            on_exit=[load_diff_drive_controller],
    #      )
    #  	),
    
    ###############
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        #joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ])
