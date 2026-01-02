from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('eggplant_controller'),
        'config',
        'kinematics.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('balancing', default_value='false'),

        Node(
            package='eggplant_controller',
            executable='thrusters_8_kinematics',
            name='thrusters_8_kinematics',
            parameters=[config, {'balancing': LaunchConfiguration('balancing')}]
        ),
        Node(
            package='eggplant_controller',
            executable='serial_manager',
            name='serial_manager',
            parameters=[{'port': LaunchConfiguration('port')}] # Pass the port
        )
    ])