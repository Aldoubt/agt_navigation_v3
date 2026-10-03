from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("agt_local_row_perception")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=PathJoinSubstitution([
            share, "config", "row_perception.yaml"])),
        DeclareLaunchArgument("geometry_file", default_value=PathJoinSubstitution([
            share, "config", "geometry_diagnostic.yaml"])),
        DeclareLaunchArgument("field_verified", default_value="false"),
        DeclareLaunchArgument("input_deskewed", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("cloud_topic", default_value="/agt/livox/points"),
        DeclareLaunchArgument("odom_topic", default_value="/agt/odometry/local"),
        DeclareLaunchArgument("odom_quality_topic", default_value="/agt/odometry/quality"),
        DeclareLaunchArgument("filtered_cloud_topic", default_value="/agt/livox/points_self_filtered"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("carrier_frame", default_value="local_row_carrier"),
        DeclareLaunchArgument("max_input_age_sec", default_value="0.5"),
        DeclareLaunchArgument("max_future_sec", default_value="0.05"),
        DeclareLaunchArgument("pending_wait_sec", default_value="0.20"),
        Node(
            package="agt_local_row_perception", executable="local_row_perception",
            name="local_row_perception", output="screen", parameters=[{
                "config_file": LaunchConfiguration("config_file"),
                "geometry_file": LaunchConfiguration("geometry_file"),
                "field_verified": ParameterValue(LaunchConfiguration("field_verified"), value_type=bool),
                "input_deskewed": ParameterValue(LaunchConfiguration("input_deskewed"), value_type=bool),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "odom_quality_topic": LaunchConfiguration("odom_quality_topic"),
                "filtered_cloud_topic": LaunchConfiguration("filtered_cloud_topic"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "carrier_frame": LaunchConfiguration("carrier_frame"),
                "max_input_age_sec": ParameterValue(LaunchConfiguration("max_input_age_sec"), value_type=float),
                "max_future_sec": ParameterValue(LaunchConfiguration("max_future_sec"), value_type=float),
                "pending_wait_sec": ParameterValue(LaunchConfiguration("pending_wait_sec"), value_type=float),
            }]),
    ])
