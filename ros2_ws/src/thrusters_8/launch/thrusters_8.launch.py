import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('thrusters_8'),
        'config',
        'thrusters_8.yaml'
    )

    return LaunchDescription([
        # Node konversi Joy -> Twist
        Node(
            package='thrusters_8',
            executable='joy_to_twist_node',
            name='joy_to_twist'
        ),
        # Node kendali Motor
        Node(
            package='thrusters_8',
            executable='thrusters_8_node',
            name='thrusters_8',
            parameters=[config]
        )
    ])