from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agt_share = Path(get_package_share_directory('agt_mapping_bringup'))
    pgo_share = Path(get_package_share_directory('pgo'))

    enable_pgo = LaunchConfiguration('enable_pgo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    lio_config = LaunchConfiguration('lio_config')
    pgo_config = LaunchConfiguration('pgo_config')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('enable_pgo', default_value='true'),
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
            default_value=str(pgo_share / 'rviz' / 'pgo.rviz'),
        ),

        # Do NOT include upstream lio_launch.py and pgo_launch.py together: both
        # launch files start a fastlio2 lio_node, which creates two competing LIO
        # instances. Start exactly one front-end and exactly one optional PGO node.
        Node(
            package='fastlio2',
            namespace='fastlio2',
            executable='lio_node',
            name='lio_node',
            output='screen',
            parameters=[{
                'config_path': lio_config,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='pgo',
            namespace='pgo',
            executable='pgo_node',
            name='pgo_node',
            output='screen',
            parameters=[{
                'config_path': pgo_config,
                'use_sim_time': use_sim_time,
            }],
            condition=IfCondition(enable_pgo),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='agt_mapping_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
        ),
    ])
