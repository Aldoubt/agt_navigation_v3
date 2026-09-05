from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agt_share = Path(get_package_share_directory('agt_mapping_bringup'))

    enable_pgo = LaunchConfiguration('enable_pgo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    lio_config = LaunchConfiguration('lio_config')
    pgo_config = LaunchConfiguration('pgo_config')
    rviz_config = LaunchConfiguration('rviz_config')
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    def nodes(context):
        # Upstream LIO/PGO consume a YAML path, so create a run-local overlay.
        # This keeps the algorithm sources untouched while making topic selection
        # explicit at launch time.
        import tempfile
        lio_src = Path(lio_config.perform(context))
        pgo_src = Path(pgo_config.perform(context))
        run_dir = Path(tempfile.mkdtemp(prefix='agt_mapping_'))
        lio_text = lio_src.read_text().replace('/livox/lidar', lidar_topic.perform(context)).replace('/livox/imu', imu_topic.perform(context))
        pgo_text = pgo_src.read_text().replace('/livox/lidar', lidar_topic.perform(context)).replace('/livox/imu', imu_topic.perform(context))
        lio_path = run_dir / 'lio.yaml'; pgo_path = run_dir / 'pgo.yaml'
        lio_path.write_text(lio_text); pgo_path.write_text(pgo_text)
        use_sim_time_value = use_sim_time.perform(context).lower() == 'true'
        actions = [Node(package='fastlio2', namespace='fastlio2', executable='lio_node', name='lio_node', output='screen', parameters=[{'config_path': str(lio_path), 'use_sim_time': use_sim_time_value}])]
        if enable_pgo.perform(context).lower() == 'true':
            actions.append(Node(package='pgo', namespace='pgo', executable='pgo_node', name='pgo_node', output='screen', parameters=[{'config_path': str(pgo_path), 'use_sim_time': use_sim_time_value}]))
        if launch_rviz.perform(context).lower() == 'true':
            actions.append(Node(package='rviz2', executable='rviz2', name='agt_mapping_rviz', output='screen', arguments=['-d', rviz_config.perform(context)]))
        return actions

    return LaunchDescription([
        DeclareLaunchArgument('enable_pgo', default_value='true'),
        DeclareLaunchArgument('lidar_topic', default_value='/agt/sensors/lidar/custom'),
        DeclareLaunchArgument('imu_topic', default_value='/agt/sensors/imu/data'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument(
            'lio_config',
            default_value=str(agt_share / 'config' / 'fastlio2_mid360.yaml'),
            description='Explicit robotics-laboratory/fast-lio2 YAML used for this mapping run.',
        ),
        DeclareLaunchArgument(
            'pgo_config',
            default_value=str(agt_share / 'config' / 'pgo_mid360.yaml'),
            description='Explicit PGO YAML used for this mapping run.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(agt_share / 'config' / 'agt_mapping.rviz'),
            description='AGT mapping RViz layout with LIO clouds/path and PGO loop markers.',
        ),

        # Do NOT include upstream lio_launch.py and pgo_launch.py together: both
        # launch files start a fastlio2 lio_node, which creates two competing LIO
        # instances. Start exactly one front-end and exactly one optional PGO node.
        OpaqueFunction(function=nodes),
    ])
