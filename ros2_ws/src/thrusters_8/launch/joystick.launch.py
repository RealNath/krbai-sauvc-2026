import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Joystick node with r1 namespace
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        parameters=[os.path.join(
            get_package_share_directory('thrusters_8'), 
            'config', 'joystick.yaml'
        )],
        output='screen'
    )
    
    return LaunchDescription([
        joy_node,
    ])