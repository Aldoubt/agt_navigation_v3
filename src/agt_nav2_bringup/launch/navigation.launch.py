from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    agt_share = Path(get_package_share_directory('agt_nav2_bringup'))

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to the derived Nav2 map YAML'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(agt_share / 'config' / 'nav2_params.yaml'),
            description='AGT Nav2 parameter file',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),

        # External FAST-LIO2 + global relocalization own map->odom and odom->base_link.
        # Therefore AMCL/localization_launch.py is intentionally not started here.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server'],
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(nav2_share / 'launch' / 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': 'False',
            }.items(),
        ),
    ])
