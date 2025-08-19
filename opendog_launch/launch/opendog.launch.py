import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- Configuration de base ---
    # Chemins vers les packages et fichiers
    description_pkg = get_package_share_directory("opendog_description")
    control_pkg = get_package_share_directory("opendog_control")
    
    # CORRECTION: Utilisation du bon nom de fichier
    urdf_file = os.path.join(description_pkg, "urdf", "opendog.urdf.xacro")
    controllers_file = os.path.join(control_pkg, "config", "opendog_joint_controller.yaml")

    # --- Conversion du XACRO en URDF ---
    # IMPORTANT: On passe l'argument use_gazebo=True pour inclure le plugin Gazebo
    robot_description_raw = xacro.process_file(
        urdf_file, 
        mappings={'use_gazebo': 'true'}  # Ajout de l'argument requis par le XACRO
    ).toxml()

    robot_description = ParameterValue(robot_description_raw, value_type=str)

    # --- Nodes de base ---
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

    # Node micro-ROS (commenté car spécifique au matériel)
    # node_uros_agent = ExecuteProcess(
    #     cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '-b', '115200', '--dev', '/dev/ttyUSB0'],
    #     output='screen'
    # )

    # --- Gestionnaire de contrôleurs ROS 2 ---
    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},  # Description URDF du robot
            controllers_file  # Configuration des contrôleurs
        ],
        output="screen"
    )

    # --- Robot State Publisher ---
    # AJOUT: Node pour publier l'état des joints (ESSENTIEL pour ros2_control)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # --- Spawners de contrôleurs ---
    node_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    node_joint_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_position_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # --- Gestion de l'ordre de démarrage ---
    return LaunchDescription([
        # Démarrer d'abord le gestionnaire de contrôleurs et le robot state publisher
        node_controller_manager,
        node_robot_state_publisher,
        
        # Démarrer les contrôleurs après un délai
        TimerAction(period=3.0, actions=[node_joint_state_broadcaster_spawner]),
        TimerAction(period=5.0, actions=[node_joint_position_controller_spawner]),
        
        # Démarrer les autres nodes
        node_joy,
        node_opendog_teleop_joy,

        # Gestion des dépendances entre nodes
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
    ])