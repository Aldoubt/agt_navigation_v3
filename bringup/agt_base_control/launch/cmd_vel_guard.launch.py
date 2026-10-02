from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _node(context):
    params_file = LaunchConfiguration('params_file').perform(context).strip()
    parameters = [params_file] if params_file else []
    parameters.append({
        'output_topic': LaunchConfiguration('output_topic'),
        'require_payload_drive_permission': ParameterValue(
            LaunchConfiguration('require_payload_drive_permission'), value_type=bool),
    })
    return [Node(
        package='agt_base_control', executable='cmd_vel_guard',
        name='agt_cmd_vel_guard', output='screen', parameters=parameters)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('output_topic', default_value='/agt/base/cmd_vel'),
        DeclareLaunchArgument('require_payload_drive_permission', default_value='false'),
        OpaqueFunction(function=_node),
    ])
