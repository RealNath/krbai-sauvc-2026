import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Declare Arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='sauvc',
        description='Namespace for the displacement visualizer'
    )

    # 2. Static Transform Publisher
    # This node tells RViz that 'odom' exists at the world origin (0 0 0)
    # Arguments: x y z yaw pitch roll parent_frame child_frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_world_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )

    # 3. Displacement Visualizer Node (Inside Namespace)
    visualizer_node = Node(
        package='eggplant_waypoint',
        executable='displacement_visualizer',
        namespace=LaunchConfiguration('namespace'),
        output='screen'
    )

    # 4. RViz2 Node (Global Namespace)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('eggplant_waypoint'), 'rviz', 'view.rviz')]
    )

    return LaunchDescription([
        namespace_arg,
        static_tf_node, # Now RViz recognizes the odom frame
        visualizer_node,
        rviz_node
    ])