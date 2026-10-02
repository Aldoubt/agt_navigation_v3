"""One motion guard owner with a Python rollback backend."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _guard(context):
    backend = LaunchConfiguration('backend').perform(context).strip().lower()
    params_file = LaunchConfiguration('params_file').perform(context).strip()
    payload_interlock = ParameterValue(
        LaunchConfiguration('require_payload_drive_permission'), value_type=bool)
    if backend == 'cpp':
        parameters = []
        if params_file:
            parameters.append(params_file)
        parameters.append({
            'output_topic': '/agt/base/cmd_vel',
            'require_payload_drive_permission': payload_interlock,
        })
        return [Node(
            package='agt_base_runtime', executable='motion_guard',
            name='agt_cmd_vel_guard', output='screen', parameters=parameters)]
    if backend == 'python':
        share = Path(get_package_share_directory('agt_base_control'))
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'cmd_vel_guard.launch.py')),
            launch_arguments={
                'params_file': params_file,
                'output_topic': '/agt/base/cmd_vel',
                'require_payload_drive_permission':
                    LaunchConfiguration('require_payload_drive_permission').perform(context),
            }.items(),
        )]
    raise RuntimeError(f'motion guard backend must be cpp|python, got {backend!r}')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='cpp', choices=['cpp', 'python']),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('require_payload_drive_permission', default_value='false'),
        OpaqueFunction(function=_guard),
    ])
