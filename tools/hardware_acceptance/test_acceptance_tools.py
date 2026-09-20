#!/usr/bin/env python3
"""Pure-logic tests for the hardware-acceptance evidence tools."""

from __future__ import annotations

import json
from pathlib import Path

import yaml

import base_footprint_center_audit as center
import generate_acceptance_report as report
import tf_authority_audit as tf_audit
import topic_rate_audit as rates


def test_topic_rate_boundaries_and_optional_missing():
    spec = {'name': '/sensor', 'min_hz': 8.0, 'max_hz': 12.0, 'required': True}
    stamps = [index * 100_000_000 for index in range(61)]
    assert rates.evaluate_rate(spec, stamps, 5.0)['status'] == 'PASS'
    assert rates.evaluate_rate(spec, stamps[::2], 5.0)['status'] == 'FAIL'
    optional = {**spec, 'required': False}
    assert rates.evaluate_rate(optional, [], 5.0)['status'] == 'WARN'


def test_tf_duplicate_publisher_is_rejected():
    expected = [{
        'parent': 'map', 'child': 'odom',
        'publisher': '/manager', 'topic': '/tf',
    }]
    observations = {
        'odom': [
            {'parent': 'map', 'gid': '01', 'topic': '/tf', 'count': 2},
            {'parent': 'map', 'gid': '02', 'topic': '/tf', 'count': 2},
        ],
    }
    result = tf_audit.evaluate_observations(
        expected, observations, {'01': '/manager', '02': '/duplicate'})
    assert result['verdict'] == 'FAIL'
    assert 'multiple_publishers' in result['observed_frames'][0]['reasons']


def center_profile():
    return {
        'minimum_motion_samples': 4,
        'minimum_observed_yaw_rad': 0.5,
        'maximum_planar_excursion_m': 0.15,
        'maximum_effective_radius_m': 0.2,
        'static_xy_tolerance_m': 0.01,
        'static_z_expected_m': 0.2,
        'static_z_tolerance_m': 0.01,
        'static_angle_tolerance_rad': 0.01,
    }


def test_center_passes_stationary_translation_during_rotation():
    samples = [
        {'x': 0.0, 'y': 0.0, 'yaw': yaw}
        for yaw in (0.0, 0.2, 0.4, 0.6)
    ]
    static = {'x': 0.0, 'y': 0.0, 'z': 0.2, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    result = center.evaluate_center(center_profile(), static, samples)
    assert result['verdict'] == 'PASS'
    assert result['motion']['effective_radius_m'] == 0.0


def test_center_rejects_wrong_static_offset_and_wide_arc():
    samples = [
        {'x': x, 'y': 0.0, 'yaw': yaw}
        for x, yaw in ((0.0, 0.0), (0.1, 0.2), (0.2, 0.4), (0.3, 0.6))
    ]
    static = {'x': 0.1, 'y': 0.0, 'z': 0.2, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    result = center.evaluate_center(center_profile(), static, samples)
    assert result['verdict'] == 'FAIL'
    assert 'static_xy_offset_out_of_tolerance' in result['failures']
    assert 'planar_excursion_exceeded' in result['failures']


def test_report_requires_all_evidence(tmp_path: Path):
    _, pending = report.build_report(tmp_path)
    assert 'Overall verdict: **NOT_RUN**' in pending

    (tmp_path / 'runtime_check.txt').write_text(
        'PASS runtime owner uniqueness\n', encoding='utf-8')
    for name in ('tf_authority.json', 'topic_rates.json', 'base_footprint_center.json'):
        (tmp_path / name).write_text(
            json.dumps({'verdict': 'PASS'}), encoding='utf-8')
    bag = tmp_path / 'navigation_bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text(yaml.safe_dump({
        'rosbag2_bagfile_information': {'message_count': 42},
    }), encoding='utf-8')
    overall, generated = report.build_report(tmp_path)
    assert overall == 'PASS'
    assert 'Overall verdict: **PASS**' in generated
