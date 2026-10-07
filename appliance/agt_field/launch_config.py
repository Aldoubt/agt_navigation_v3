"""Keep V3 planner/controller behavior; inject only measured YHS profile values."""

from pathlib import Path
import json
from .contracts import atomic_yaml, read_yaml
from .profile import require_real


def merge(target, source):
    for k, v in source.items():
        if k in target and isinstance(v, dict) and isinstance(target[k], dict):
            merge(target[k], v)
        else:
            target[k] = v


def write_navigation_params(profile, destination, config_dir=None):
    require_real(profile, motion=True)
    if config_dir is None:
        from ament_index_python.packages import get_package_share_directory

        config_dir = Path(get_package_share_directory("agt_nav2_bringup")) / "config"
        bt_share = Path(get_package_share_directory("nav2_bt_navigator"))
    else:
        bt_share = None
    config_dir = Path(config_dir)
    params = {}
    for name in ["robot", "navigation", "controller", "costmap", "perception", "safety"]:
        merge(params, read_yaml(config_dir / (name + ".yaml")))
    if bt_share:
        params["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = str(
            bt_share
            / "behavior_trees"
            / "navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml"
        )
    params["controller_server"]["ros__parameters"].update(
        min_x_velocity_threshold=0.01,
        min_y_velocity_threshold=0.0,
        min_theta_velocity_threshold=0.01,
    )
    robot = params["agt_robot_config"]["ros__parameters"]
    for key in ["map_frame", "odom_frame", "base_frame", "rotation_frame", "lidar_frame"]:
        robot[key] = profile["robot"][key]
    robot["footprint"] = json.dumps(profile["navigation"]["footprint"])
    robot["footprint_padding"] = float(profile["navigation"]["footprint_padding"])
    limits = {k: float(v) for k, v in profile["navigation"]["motion_limits"].items()}
    params["agt_motion_limits"]["ros__parameters"] = limits
    for name in ["local_costmap", "global_costmap"]:
        cost = params[name][name]["ros__parameters"]
        cost["robot_base_frame"] = profile["robot"]["base_frame"]
        cost["footprint"] = robot["footprint"]
        cost["footprint_padding"] = robot["footprint_padding"]
    follow = params["controller_server"]["ros__parameters"]["FollowPath"]
    follow.update(
        desired_linear_vel=limits["controller_cruise_mps"],
        min_approach_linear_velocity=limits["controller_approach_mps"],
        regulated_linear_scaling_min_speed=limits["controller_regulated_min_mps"],
        rotate_to_heading_angular_vel=limits["rotate_to_heading_radps"],
        max_angular_accel=limits["controller_bootstrap_angular_accel"],
    )
    behavior = params["behavior_server"]["ros__parameters"]
    behavior.update(
        max_rotational_vel=limits["rotate_to_heading_radps"],
        min_rotational_vel=limits["min_rotational_radps"],
        rotational_acc_lim=limits["angular_accel_radps2"],
    )
    smoother = params["velocity_smoother"]["ros__parameters"]
    smoother.update(
        max_velocity=[limits["forward_mps"], 0.0, limits["angular_radps"]],
        min_velocity=[-limits["reverse_mps"], 0.0, -limits["angular_radps"]],
        max_accel=[limits["linear_accel_mps2"], 0.0, limits["angular_accel_radps2"]],
        max_decel=[-limits["linear_decel_mps2"], 0.0, -limits["angular_decel_radps2"]],
    )
    guard = params["agt_cmd_vel_guard"]["ros__parameters"]
    guard.update(
        max_linear_x=limits["forward_mps"],
        max_reverse_x=limits["reverse_mps"],
        max_angular_z=limits["angular_radps"],
        max_linear_accel=limits["linear_accel_mps2"],
        max_linear_decel=limits["linear_decel_mps2"],
        max_angular_accel=limits["angular_accel_radps2"],
        command_timeout_sec=float(profile["base"]["command_timeout_sec"]),
        output_topic=profile["topics"]["guarded_cmd_vel"],
        manual_input_topic="/agt/hmi/cmd_vel",
        require_localization_status=True,
        allow_degraded_localization=False,
    )
    for name in ["bt_navigator", "behavior_server"]:
        params[name]["ros__parameters"]["robot_base_frame"] = profile["robot"]["base_frame"]
    measured = profile["navigation"]["perception"]
    perception = params["agt_pointcloud_preprocessor"]["ros__parameters"]
    perception["base_frame"] = profile["robot"]["base_frame"]
    perception["self_filter"]["center_xyz"] = [float(v) for v in measured["self_center_xyz"]]
    perception["self_filter"]["size_xyz"] = [float(v) for v in measured["self_size_xyz"]]
    perception["ground_filter"]["sensor_ground_z_m"] = float(measured["sensor_ground_z_m"])
    atomic_yaml(destination, params)
    return Path(destination)
