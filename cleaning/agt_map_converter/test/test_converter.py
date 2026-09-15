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


def test_trajectory_footprint_clears_only_swept_corridor():
    # Two independent vertical obstacles. The supplied trajectory footprint
    # crosses only the first one; the distant obstacle must remain occupied.
    xyz = np.array([
        [0.05, 0.05, 0.0], [0.08, 0.08, 0.01],
        [0.45, 0.05, 0.0], [0.45, 0.05, 0.50],
        [1.45, 0.05, 0.0], [1.45, 0.05, 0.50],
    ], dtype=float)
    layers = convert(
        xyz, resolution=0.2, margin=0.0, min_points=2,
        max_step=0.22, max_slope_deg=89.0,
        trajectory_poses=[(0.45, 0.05, 0.0)],
        trajectory_front_m=0.20,
        trajectory_rear_m=0.20,
        trajectory_half_width_m=0.15)
    occ = np.flipud(layers['occupancy'])
    assert layers['trajectory_cleared_cells'] >= 1
    assert 254 in occ
    assert 0 in occ


def test_trajectory_conflict_evidence_has_regions_and_excludes_outside_obstacle():
    # The first obstacle is in the swept corridor; the second is deliberately
    # outside it and must not be reported as trajectory evidence.
    xyz = np.array([
        [0.05, 0.05, 0.0], [0.08, 0.08, 0.01],
        [0.45, 0.05, 0.0], [0.45, 0.05, 0.50],
        [1.45, 0.05, 0.0], [1.45, 0.05, 0.50],
    ], dtype=float)
    layers = convert(
        xyz, resolution=0.2, margin=0.0, min_points=2,
        max_step=0.22, max_slope_deg=89.0,
        trajectory_poses=[(0.45, 0.05, 0.0)],
        trajectory_front_m=0.20, trajectory_rear_m=0.20,
        trajectory_half_width_m=0.15)
    assert int(layers['trajectory_conflicts'].sum()) == layers['trajectory_cleared_cells']
    assert layers['trajectory_conflict_regions']
    conflict_cells = [
        cell for region in layers['trajectory_conflict_regions'] for cell in region['cells']]
    assert all(cell['map_x_m'] < 1.0 for cell in conflict_cells)
    assert all(region['approximate_area_m2'] > 0.0 for region in layers['trajectory_conflict_regions'])


def test_trajectory_conflict_regions_are_deterministic():
    xyz = np.array([
        [0.05, 0.05, 0.0], [0.08, 0.08, 0.01],
        [0.45, 0.05, 0.0], [0.45, 0.05, 0.50],
    ], dtype=float)
    kwargs = dict(
        resolution=0.2, margin=0.0, min_points=2,
        max_step=0.22, max_slope_deg=89.0,
        trajectory_poses=[(0.45, 0.05, 0.0)],
        trajectory_front_m=0.20, trajectory_rear_m=0.20,
        trajectory_half_width_m=0.15)
    first = convert(xyz, **kwargs)
    second = convert(xyz, **kwargs)
    assert first['trajectory_cleared_cells'] == second['trajectory_cleared_cells']
    assert first['trajectory_conflict_regions'] == second['trajectory_conflict_regions']


def test_zero_conflict_trajectory_is_pass_evidence():
    # Every cell under the swept rectangle has sufficient flat-ground samples.
    xyz = np.array([
        [x, y, z]
        for x in np.arange(-0.45, 0.55, 0.1)
        for y in np.arange(-0.45, 0.55, 0.1)
        for z in (0.0, 0.01)
    ], dtype=float)
    layers = convert(
        xyz, resolution=0.1, margin=0.0, min_points=2,
        max_step=0.22, max_slope_deg=20.0,
        trajectory_poses=[(0.0, 0.0, 0.0)],
        trajectory_front_m=0.20, trajectory_rear_m=0.20,
        trajectory_half_width_m=0.15)
    assert layers['trajectory_cleared_cells'] == 0
    assert layers['trajectory_conflict_regions'] == []


def test_ground_confidence_mode_suppresses_high_only_slope_wall_without_freeing_it():
    # A flat observed ground occupies the left half. The right half contains
    # only elevated returns with small per-cell vertical span. Legacy min-z
    # slope sees a sharp cliff; MQ2-A should reject those high-only cells from
    # the slope surface and keep them unknown instead of silently free.
    xyz = []
    for gy in range(5):
        for gx in range(6):
            base = 0.0 if gx < 3 else 2.0
            xyz.append([gx + 0.05, gy + 0.05, base])
            xyz.append([gx + 0.08, gy + 0.08, base + 0.05])
    xyz = np.asarray(xyz, dtype=float)

    legacy = convert(
        xyz, resolution=1.0, margin=0.5, min_points=2,
        max_step=0.22, max_slope_deg=20.0,
        slope_surface_mode='legacy_min_z')
    mq2 = convert(
        xyz, resolution=1.0, margin=0.5, min_points=2,
        max_step=0.22, max_slope_deg=20.0,
        slope_surface_mode='ground_confidence',
        ground_radius_cells=2,
        ground_height_tolerance_m=0.25)

    legacy_occ = np.flipud(legacy['occupancy'])
    mq2_occ = np.flipud(mq2['occupancy'])

    assert legacy['slope_trigger_cells'] > mq2['slope_trigger_cells']
    assert mq2['low_confidence_valid_cells'] > 0
    assert 205 in mq2_occ
    assert mq2['span_trigger_cells'] == 0


def test_ground_confidence_mode_keeps_gentle_ground_trusted():
    xyz = []
    for gy in range(5):
        for gx in range(6):
            base = 0.05 * gx
            xyz.append([gx + 0.05, gy + 0.05, base])
            xyz.append([gx + 0.08, gy + 0.08, base + 0.01])
    xyz = np.asarray(xyz, dtype=float)

    mq2 = convert(
        xyz, resolution=1.0, margin=0.5, min_points=2,
        max_step=0.22, max_slope_deg=20.0,
        slope_surface_mode='ground_confidence',
        ground_radius_cells=2,
        ground_height_tolerance_m=0.25)

    assert mq2['low_confidence_valid_cells'] == 0
    assert mq2['occupied_cells'] == 0


def test_anchored_ground_confidence_rejects_disconnected_flat_high_surface():
    # Keep samples away from exact grid boundaries. The production converter
    # intentionally preserves its legacy float-to-int binning for frozen-map
    # reproducibility; this test targets connectivity, not bin-edge rounding.
    # Ground is connected to the trajectory on the left. A flat high-only
    # surface exists on the right, separated by an unobserved column. Local
    # confidence alone can accept both; anchored confidence must keep the
    # disconnected high surface unknown.
    xyz = []
    for gy in range(5):
        for gx in (0, 1, 2):
            xyz.append([gx + 0.05, gy + 0.05, 0.0])
            xyz.append([gx + 0.08, gy + 0.08, 0.01])
        for gx in (4, 5, 6):
            xyz.append([gx + 0.05, gy + 0.05, 3.0])
            xyz.append([gx + 0.08, gy + 0.08, 3.01])
    xyz = np.asarray(xyz, dtype=float)

    local = convert(
        xyz, resolution=1.0, margin=0.5, min_points=2,
        max_step=0.22, max_slope_deg=20.0,
        trajectory_poses=[(1.0, 2.0, 0.0)],
        trajectory_front_m=0.4, trajectory_rear_m=0.4,
        trajectory_half_width_m=0.4,
        slope_surface_mode='ground_confidence',
        ground_radius_cells=1,
        ground_height_tolerance_m=0.25)

    anchored = convert(
        xyz, resolution=1.0, margin=0.5, min_points=2,
        max_step=0.22, max_slope_deg=20.0,
        trajectory_poses=[(1.0, 2.0, 0.0)],
        trajectory_front_m=0.4, trajectory_rear_m=0.4,
        trajectory_half_width_m=0.4,
        slope_surface_mode='anchored_ground_confidence',
        ground_radius_cells=1,
        ground_height_tolerance_m=0.25,
        ground_connect_max_slope_deg=45.0)

    local_grid = np.flipud(local['occupancy'])
    anchored_grid = np.flipud(anchored['occupancy'])

    assert local['ground_confident_cells'] > anchored['ground_confident_cells']
    assert anchored['floating_ground_candidate_cells'] > 0
    # A high-island cell is free under local-only confidence but unknown when
    # it lacks trajectory-connected ground support.
    assert local_grid[2, 5] == 254
    assert anchored_grid[2, 5] == 205


def test_anchored_ground_confidence_keeps_connected_gentle_ground():
    xyz = []
    for gy in range(5):
        for gx in range(7):
            base = 0.03 * gx
            xyz.append([gx + 0.05, gy + 0.05, base])
            xyz.append([gx + 0.08, gy + 0.08, base + 0.01])
    xyz = np.asarray(xyz, dtype=float)

    anchored = convert(
        xyz, resolution=1.0, margin=0.5, min_points=2,
        max_step=0.22, max_slope_deg=20.0,
        trajectory_poses=[(1.0, 2.0, 0.0)],
        trajectory_front_m=0.4, trajectory_rear_m=0.4,
        trajectory_half_width_m=0.4,
        slope_surface_mode='anchored_ground_confidence',
        ground_radius_cells=1,
        ground_height_tolerance_m=0.25,
        ground_connect_max_slope_deg=45.0)

    assert anchored['floating_ground_candidate_cells'] == 0
    assert anchored['ground_confident_cells'] == anchored['raw_valid_cells']
    assert anchored['occupied_cells'] == 0
