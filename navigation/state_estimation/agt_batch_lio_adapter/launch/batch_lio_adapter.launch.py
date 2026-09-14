import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('agt_batch_lio_adapter'),
        'config', 'batch_lio_adapter.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'batch_lio_config_file',
            default_value='',
            description=(
                'Exact Batch-LIO runtime YAML. Required for canonical '
                'body->base derivation when use_configured_extrinsic=false.')),
        Node(
            package='agt_batch_lio_adapter',
            executable='batch_lio_adapter',
            name='agt_batch_lio_adapter',
            output='screen',
            parameters=[
                cfg,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'batch_lio_config_file': LaunchConfiguration(
                        'batch_lio_config_file'),
                },
            ],
        ),
    ])
