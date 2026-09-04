from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    agt_share = Path(get_package_share_directory('agt_mapping_bringup'))
    batch_share = Path(get_package_share_directory('batch_lio'))
    adapter_share = Path(get_package_share_directory('agt_batch_lio_adapter'))

    batch_config = LaunchConfiguration('batch_config')
    launch_batch_rviz = LaunchConfiguration('launch_batch_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'batch_config',
            default_value=str(agt_share / 'config' / 'batch_lio_mid360.yaml'),
            description='Batch-LIO MID360 parameter file.',
        ),
        DeclareLaunchArgument(
            'launch_batch_rviz',
            default_value='false',
            description='Launch Batch-LIO upstream RViz. Keep false in the AGT field demo.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(batch_share / 'launch' / 'mapping_avia.launch.py')),
            launch_arguments={
                'config': batch_config,
                'rviz': launch_batch_rviz,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(adapter_share / 'launch' / 'batch_lio_adapter.launch.py')
            ),
        ),
    ])
