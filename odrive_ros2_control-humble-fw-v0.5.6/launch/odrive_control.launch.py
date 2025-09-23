from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odrive_hardware_interface',
            executable='odrive_hardware_interface_node',
            name='odrive_hardware_interface',
            output='screen',
            parameters=[
                # Ici, si le node supporte, on peut indiquer les ports USB
                # {'usb_port0': '/dev/ttyUSB0', 'usb_port1': '/dev/ttyUSB1'}
            ]
        )
    ])
