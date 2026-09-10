"""O1-H2 OctoMap navigation-map baseline with rear dynamic-point suppression."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agt_mapping_bringup'))
    start_fastlio = LaunchConfiguration('start_fastlio')
    filter_statistics = LaunchConfiguration('filter_statistics')

    mapping_launch = share / 'launch' / 'mapping_mode.launch.py'
    lio_config = share / 'config' / 'fastlio2_octomap_baseline.yaml'
    baseline_config = share / 'config' / 'octomap_navigation_baseline.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_fastlio', default_value='true',
            description='Start the dedicated fixed-parameter FAST-LIO2 instance.'),
        DeclareLaunchArgument(
            'filter_statistics',
            default_value=str(Path.home() / '.ros' / 'agt_octomap' / 'rear_filter_statistics.yaml'),
            description='Absolute YAML written when the rear-filter node exits.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(mapping_launch)),
            condition=IfCondition(start_fastlio),
            launch_arguments={
                'enable_pgo': 'false',
                'enable_octomap_navigation': 'false',
                'launch_rviz': 'false',
                'lio_config': str(lio_config),
            }.items()),
        Node(
            package='agt_pointcloud_preprocessor',
            executable='obstacle_cloud_node',
            name='agt_obstacle_cloud_preprocessor',
            output='screen',
            parameters=[str(baseline_config), {'statistics_output': filter_statistics}],
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            remappings=[('cloud_in', '/agt/octomap/rear_filtered_cloud')],
            parameters=[str(baseline_config)],
        ),
    ])
