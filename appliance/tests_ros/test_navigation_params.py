# ruff: noqa: E402
"""ROS2 parser smoke for profile overrides; synthetic geometry never deployed."""

from pathlib import Path
import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.node import Node
from rclpy.parameter import Parameter
from agt_field.profile import load_profile, MOTION_KEYS
from agt_field.launch_config import write_navigation_params


def test_integer_profile_values_emit_ros_double_arrays(tmp_path):
    root = Path("/opt/nav_ws/src/agt_navigation_v3")
    p = load_profile(root / "profiles/mock_yhs")
    p["robot"].update(rotation_frame="base_link", lidar_frame="fixture_lidar")
    p["navigation"].update(
        footprint=[[-1, -1], [-1, 1], [1, 1], [1, -1]],
        footprint_padding=0,
        motion_limits={k: 1 for k in MOTION_KEYS},
        perception=dict(self_center_xyz=[0, 0, 0], self_size_xyz=[1, 1, 1], sensor_ground_z_m=-1),
    )
    path = write_navigation_params(p, tmp_path / "params.yaml", root / "config")
    rclpy.init(args=["--ros-args", "--params-file", str(path)])
    node = Node("velocity_smoother", automatically_declare_parameters_from_overrides=True)
    try:
        assert node.get_parameter("max_velocity").type_ == Parameter.Type.DOUBLE_ARRAY
        assert node.get_parameter("max_velocity").value == [1.0, 0.0, 1.0]
    finally:
        node.destroy_node()
        rclpy.shutdown()
