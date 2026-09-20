import math

import pytest

import generate_distance_missions as generator


def test_target_xy_uses_metric_map_axes():
    x, y = generator.target_xy(2.0, -1.0, 90.0, 7.0)
    assert x == pytest.approx(2.0)
    assert y == pytest.approx(6.0)


def test_mission_repeats_target_and_home():
    mission = generator.build_mission(
        map_id='orchard_v1', distance_m=5.0, start_x=1.0, start_y=2.0,
        heading_deg=30.0, repeats=5, target_hold_sec=15.0, home_hold_sec=2.0)
    assert len(mission['points']) == 10
    assert mission['points'][0]['pose']['x'] == pytest.approx(1.0 + 5.0 * math.cos(math.pi / 6))
    assert mission['points'][0]['settle_time'] == 15.0
    assert mission['points'][1]['pose']['x'] == 1.0
    assert mission['points'][0]['views'] == []
