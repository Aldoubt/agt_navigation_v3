from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('agt_gazebo_sim'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    robot_description = ParameterValue(
        Command(['xacro ', str(share / 'urdf' / 'bunker_mid360_sim.urdf.xacro')]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(share / 'worlds' / 'agt_mapping.world')),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('x', default_value='-3.5'),
        DeclareLaunchArgument('y', default_value='-2.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_share / 'launch' / 'gazebo.launch.py')),
            launch_arguments={'world': world, 'gui': gui, 'verbose': 'false'}.items(),
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='agt_sim_robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'agt_bunker', '-topic', 'robot_description',
                       '-x', x, '-y', y, '-z', '0.22', '-Y', yaw],
            output='screen',
        ),
        Node(
            package='agt_gazebo_sim', executable='imu_unit_adapter',
            name='agt_sim_imu_unit_adapter', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
