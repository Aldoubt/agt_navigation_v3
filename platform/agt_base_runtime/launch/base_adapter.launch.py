"""Select one Bunker or explicitly configured planar tracked driver boundary."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _adapter(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    selected = value('adapter')
    if selected == 'bunker':
        return [Node(package='agt_base_runtime', executable='bunker_adapter',
                     name='agt_bunker_base_adapter', output='screen', parameters=[{
                         'input_topic': value('input_topic'), 'output_topic': value('output_topic')}])]
    path = Path(value('params_file')).expanduser()
    if not value('params_file') or not path.is_file():
        raise RuntimeError('tracked/YHS/custom adapter requires an explicit measured params_file')
    config = yaml.safe_load(path.read_text())
    params = config.get('agt_tracked_twist_adapter', {}).get('ros__parameters', {})
    if params.get('robot_profile') != value('robot_profile'):
        raise RuntimeError('base adapter profile differs from the selected robot')
    return [Node(package='agt_base_runtime', executable='tracked_twist_adapter',
                 name='agt_tracked_twist_adapter', output='screen', parameters=[str(path)])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('adapter', default_value='bunker', choices=['bunker','tracked','yhs','custom']),
        DeclareLaunchArgument('robot_profile', default_value='bunker_v1'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('input_topic', default_value='/agt/base/cmd_vel'),
        DeclareLaunchArgument('output_topic', default_value='/mux/cmd_vel'),
        OpaqueFunction(function=_adapter),
    ])
