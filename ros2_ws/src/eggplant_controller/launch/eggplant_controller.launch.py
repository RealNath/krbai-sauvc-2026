from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('balancing', default_value='false'),

        Node(
            package='eggplant_controller',
            executable='thrusters_8_kinematics',
            name='thrusters_8_kinematics',
            parameters=[{'balancing': LaunchConfiguration('balancing')}] # Pass the toggle
        ),
        Node(
            package='eggplant_controller',
            executable='serial_manager',
            name='serial_manager',
            parameters=[{'port': LaunchConfiguration('port')}] # Pass the port
        )
    ])