import os

from black import out
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

import xacro

# configure robot's urdf file
pkg_openleg_gazebo = 'openleg_description'
robot_description_subpath = 'urdf/openleg.urdf.xacro'
xacro_file = os.path.join(get_package_share_directory(pkg_openleg_gazebo),robot_description_subpath)
robot_description_raw = xacro.process_file(xacro_file).toxml()

#configure gazebo
pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros') 
pkg_openleg_gazebo = FindPackageShare(package='openleg_description').find('openleg_description')

# Set the path to the world file
#world_file_name = 'contact.world'
#world_path = os.path.join(pkg_hyperdog_gazebo, 'worlds', world_file_name)
# world_path = os.path.join(pkg_hyperdog_gazebo, 'worlds', 'contact.world')

# world_path = "/usr/share/gazebo-11/worlds/friction_demo.world"


# configure hyperdog teleop
#teleop_pkg_name = 'hyperdog_teleop'
#teleop_launch_file = "/hyperdog_teleop.launch.py"

# set the controller
#gazebo_controller = 'hyperdog_joint_controller'
 

 
def generate_launch_description():

  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value='empty.world',#world_path,
    description='Full path to the world model file to load')

  declare_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_description_raw,
                    'use_sim_time':True}])
  
  #hyperdog_gz_joint_ctrl_node = Node(
  #  package='hyperdog_gazebo_sim',
  #  executable='hyperdog_gazebo_joint_ctrl_node',
  #  output='screen'
  #)
    
  spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', 
                    arguments=['-topic', 'robot_description',
                               '-entity', 'openleg'],
                    output='screen')

  load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'],
        output='screen' )
  
  laod_forward_command_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
            'joint_trajectory_controller'],
        output='screen'
    )



  # Specify the actions
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
 

  # Create the launch description and populate
  return  LaunchDescription([    

    RegisterEventHandler(
      event_handler=OnProcessExit(
        target_action=spawn_entity,
        on_exit=[load_joint_state_controller],
      )
    ),
    RegisterEventHandler(
      event_handler=OnProcessExit(
        target_action=load_joint_state_controller,
        on_exit=[laod_forward_command_controller],
      )
    ),
    declare_simulator_cmd,
    declare_use_sim_time_cmd,
    declare_use_simulator_cmd,
    declare_world_cmd,

    start_gazebo_server_cmd,
    start_gazebo_client_cmd,

    spawn_entity,
    declare_robot_state_publisher,
  
 ])
































"""
############################################################################
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable 
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    #gazebo_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([
    #        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])])
    #)
    #
    #openleg_description_path = os.path.join(
    #    get_package_share_directory('greenrover_description'))
    #
    #xacro_file = os.path.join(openleg_description_path,
    #                          'urdf',
    #                          'greenrover.gazebo.xacro')

    #doc = xacro.process_file(xacro_file, mappings={'gazebo' : 'true'})
    #robot_description_config = doc.toprettyxml(indent='  ')
    #robot_description = {'robot_description': robot_description_config}
    #
    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('openleg_description'), 'urdf', 'openleg.xacro'
        ])
    ])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    robot_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
        output='screen'
    )
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )
    #
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
    return LaunchDescription([
        ##########################################################
        RegisterEventHandler(                     #Event handler to load joint state controller after the robot is spawned
            event_handler=OnProcessExit(
            	target_action=robot_spawn_node,
            	on_exit=[load_joint_trajectory_controller],
            )
        ),
        #RegisterEventHandler(			  #Event handler to load openDog controller after the joint state controller is loaded
        #    event_handler=OnProcessExit(
        #        target_action=load_joint_trajectory_controller,
        #        on_exit=[load_openleg_controller],
        #    )
        #),
        
        ##########################################################
        gazebo_launch,
        robot_state_publisher_node,
        robot_spawn_node,
        gazebo_server,
        gazebo_client,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller
    ])
"""