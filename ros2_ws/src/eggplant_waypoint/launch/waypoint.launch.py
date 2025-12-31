import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the argument for the filename only
    mission_file_arg = DeclareLaunchArgument(
        'mission', default_value='mission.yaml',
        description='Name of the mission file in the config folder'
    )

    def launch_setup(context, *args, **kwargs):
        mission_file = LaunchConfiguration('mission').perform(context)
        
        # Construct the full path automatically
        package_path = get_package_share_directory('eggplant_waypoint')
        full_mission_path = os.path.join(package_path, 'config', mission_file)

        waypoint_node = Node(
            package='eggplant_waypoint',
            executable='waypoint_sequencer',
            parameters=[{'mission_file': full_mission_path}]
        )
        return [waypoint_node]

    return LaunchDescription([
        mission_file_arg,
        OpaqueFunction(function=launch_setup)
    ])