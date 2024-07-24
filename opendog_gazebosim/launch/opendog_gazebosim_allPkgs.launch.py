import os

from click import argument
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


# configure robot's urdf file
pkg_opendog_gazebo = 'opendog_gazebosim'
robot_description_subpath = 'description/opendog.urdf.xacro'
xacro_file = os.path.join(get_package_share_directory(pkg_opendog_gazebo),robot_description_subpath)
robot_description_raw = xacro.process_file(xacro_file).toxml()

teleop_pkg_name = 'opendog_teleop'
teleop_launch_file = "/opendog_teleop.launch.py"

gazebo_controller = 'opendog_joint_controller'

  # Set the path to this package.
pkg_share = FindPackageShare(package='opendog_gazebosim').find('opendog_gazebosim')

# Set the path to the world file
world_file_name = 'test_world.world'
world_path = os.path.join(pkg_share, 'worlds', world_file_name)

def generate_launch_description():

        
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        )
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_description_raw,
                    'use_sim_time':True}])

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', 
                    arguments=['-topic', 'robot_description',
                               '-entity', 'openDog_V3'],
                    output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
            'joint_state_broadcaster'],
        output='screen' )

    laod_forward_command_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 
            'gazebo_joint_controller'],
        output='screen'
    )

    launch_opendog_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(teleop_pkg_name), 'launch'), teleop_launch_file]),)

    launch_joy_cmd_executor = ExecuteProcess(
        cmd=['ros2', 'run', 'opendog_control', 'ctrl_cmd_executor'],
        output='screen'
    )

    opendog_joint_controller = Node(
        package='opendog_control',
        executable='opendog_joint_controller_node',
        output='screen'
    )

   
   

    return LaunchDescription([ 
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        load_joint_state_controller,
        laod_forward_command_controller,
        launch_opendog_teleop,
        launch_joy_cmd_executor,
        opendog_joint_controller,

    ])
