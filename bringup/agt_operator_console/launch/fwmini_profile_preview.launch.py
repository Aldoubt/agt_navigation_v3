"""Isolated FW-mini body and measured-candidate TF preview; no hardware nodes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from agt_operator_console.fwmini_profile import build_preview_urdf, load_profile


def _preview(context):
    profile_path = Path(LaunchConfiguration('profile_file').perform(context)).expanduser()
    profile = load_profile(profile_path)
    urdf = build_preview_urdf(profile)
    rviz_config = Path(get_package_share_directory('agt_operator_console')) / 'config' / 'fwmini_profile_preview.rviz'
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='fwmini_profile_preview_state_publisher', output='screen',
            parameters=[{
                'robot_description': urdf,
                'frame_prefix': 'fwmini_preview/',
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='fwmini_profile_preview_rviz',
            arguments=['-d', str(rviz_config)], output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]


def generate_launch_description():
    default_profile = Path(get_package_share_directory('agt_operator_console')) / 'config' / 'fwmini_swerve_preview.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('profile_file', default_value=str(default_profile)),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_preview),
    ])
