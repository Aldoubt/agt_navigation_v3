from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agt_share = Path(get_package_share_directory('agt_mapping_bringup'))
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
        cfg.write_text(Path(batch_config.perform(context)).read_text().replace('/livox/lidar', lidar_topic.perform(context)).replace('/livox/imu', imu_topic.perform(context)))
        return [
            # Batch-LIO publishes camera_init->body where body is its IMU/LIO
            # state frame. This calibrated static transform makes base_link the
            # physical robot frame used by navigation and relocalization.
            Node(
                package='tf2_ros', executable='static_transform_publisher',
                name='agt_body_to_base_link', output='screen',
                arguments=[
                    '--x', '-0.16403417', '--y', '0.02439982', '--z', '-0.49511119',
                    '--qx', '0.000477000', '--qy', '-0.100267018',
                    '--qz', '-0.001592000', '--qw', '0.994959177',
                    '--frame-id', 'body', '--child-frame-id', 'base_link',
                ],
                parameters=[{
                    'use_sim_time': use_sim_time.perform(context).lower() == 'true'
                }],
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(batch_share / 'launch' / 'mapping_avia.launch.py')), launch_arguments={'config': str(cfg), 'rviz': launch_batch_rviz.perform(context), 'use_sim_time': use_sim_time.perform(context)}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(adapter_share / 'launch' / 'batch_lio_adapter.launch.py')), launch_arguments={'use_sim_time': use_sim_time.perform(context)}.items()),
        ]

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
        DeclareLaunchArgument('lidar_topic', default_value='/agt/sensors/lidar/custom'),
        DeclareLaunchArgument('imu_topic', default_value='/agt/sensors/imu/data'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=includes),
    ])
