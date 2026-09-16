"""Navigation-only local odometry: Batch-LIO plus the AGT frame adapter.

This launch deliberately contains no FAST-LIO2, PGO, map saving, or mapping
topics. The mapping producer is a separate workspace and publishes only a
validated Map Package.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    runtime_share = Path(get_package_share_directory('agt_navigation_runtime'))
    batch_share = Path(get_package_share_directory('batch_lio'))
    adapter_share = Path(get_package_share_directory('agt_batch_lio_adapter'))
    batch_config = LaunchConfiguration('batch_config')
    launch_batch_rviz = LaunchConfiguration('launch_batch_rviz')
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    def includes(context):
        import tempfile

        run_dir = Path(tempfile.mkdtemp(prefix='agt_batch_lio_'))
        cfg = run_dir / 'batch_lio.yaml'
        config_text = Path(batch_config.perform(context)).read_text(encoding='utf-8')
        config_text = config_text.replace(
            '/agt/sensors/lidar/custom', lidar_topic.perform(context)).replace(
            '/agt/sensors/imu/data', imu_topic.perform(context)).replace(
            '/livox/lidar', lidar_topic.perform(context)).replace(
            '/livox/imu', imu_topic.perform(context))
        cfg.write_text(config_text, encoding='utf-8')
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(batch_share / 'launch' / 'mapping_avia.launch.py')),
                launch_arguments={
                    'config': str(cfg),
                    'rviz': launch_batch_rviz.perform(context),
                    'use_sim_time': use_sim_time.perform(context),
                }.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(adapter_share / 'launch' / 'batch_lio_adapter.launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time.perform(context),
                    'batch_lio_config_file': str(cfg),
                }.items()),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'batch_config',
            default_value=str(runtime_share / 'config' / 'batch_lio_mid360.yaml'),
            description='Navigation Batch-LIO runtime parameter file.'),
        DeclareLaunchArgument('launch_batch_rviz', default_value='false'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=includes),
    ])
