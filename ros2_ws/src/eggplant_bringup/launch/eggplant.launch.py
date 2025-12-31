from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # 1. Declare Global Arguments
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='Serial port for the robot'
    )
    
    balancing_arg = DeclareLaunchArgument(
        'balancing', default_value='false',
        description='Toggle for the balancing mechanism'
    )

    mission_arg = DeclareLaunchArgument(
        'mission', default_value='mission.yaml',
        description='Mission filename'
    )

    # 2. Define the Namespace Group
    # This ensures all nodes and topics within use the /sauvc namespace
    bringup_group = GroupAction(
        actions=[
            PushRosNamespace('sauvc'),

            # Include Controller Launch
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('eggplant_controller'),
                        'launch',
                        'eggplant_controller.launch.py'
                    ])
                ]),
                launch_arguments={
                    'port': LaunchConfiguration('port'),
                    'balancing': LaunchConfiguration('balancing')
                }.items()
            ),

            # Include Waypoint Launch
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('eggplant_waypoint'),
                        'launch',
                        'waypoint.launch.py'
                    ])
                ]),
                launch_arguments={'mission': LaunchConfiguration('mission')}.items()
            ),
        ]
    )

    return LaunchDescription([
        port_arg,
        balancing_arg,
        mission_arg,
        bringup_group
    ])