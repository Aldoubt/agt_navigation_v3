import numpy as np

from agt_map_converter.pcd_to_nav_map import convert


def test_flat_ground_and_step_obstacle():
    # Two ground cells plus one cell containing a 0.5 m vertical span.
    xyz = np.array([
        [0.05, 0.05, 0.0], [0.08, 0.08, 0.01],
        [0.25, 0.05, 0.0], [0.28, 0.08, 0.02],
        [0.45, 0.05, 0.0], [0.45, 0.05, 0.50],
    ], dtype=float)
    layers = convert(
        xyz, resolution=0.2, margin=0.0, min_points=2,
        max_step=0.22, max_slope_deg=45.0)
    occ = np.flipud(layers['occupancy'])
    assert layers['occupied_cells'] >= 1
    assert 0 in occ
    assert 254 in occ


def test_unknown_cells_remain_unknown():
    xyz = np.array([[0.05, 0.05, 0.0], [0.06, 0.06, 0.0]], dtype=float)
    layers = convert(
        xyz, resolution=0.1, margin=0.5, min_points=2,
        max_step=0.22, max_slope_deg=20.0)
    assert 205 in layers['occupancy']
