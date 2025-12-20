from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eggplant_controller',
            executable='thrusters_8_kinematics',
            name='thrusters_8_kinematics'
        ),
        Node(
            package='eggplant_controller',
            executable='serial_manager',
            name='serial_manager',
            parameters=[{'port': '/dev/ttyUSB0'}]
        )
    ])