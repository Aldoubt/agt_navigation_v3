import math
from pathlib import Path

import pytest

from agt_localization_core.pose_math import Pose3
from agt_localization_core.state_machine import LocalizationState as CoreState
from agt_localization_ros.shadow_manager_node import ShadowConfig, ShadowLocalizationModel


def _pose(x=0.0, y=0.0, yaw_rad=0.0):
    return Pose3((x, y, 0.0), (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)))


def test_global_observation_creates_shadow_map_to_odom_without_tf():
    model = ShadowLocalizationModel(ShadowConfig())
    model.observe_odom(1_000_000_000, _pose(2.0, 0.0), 1_000_000_000)
    assert model.accept_global(1_000_000_000, _pose(5.0, 0.0))
    snapshot = model.snapshot()
    assert snapshot.state is CoreState.LOCALIZED
    assert snapshot.correction.position == pytest.approx((3.0, 0.0, 0.0))
    assert snapshot.decision == 'ACCEPTED'


def test_tracking_innovation_rejection_enters_degraded_without_side_effect():
    config = ShadowConfig(tracking_consecutive_accepts=1)
    model = ShadowLocalizationModel(config)
    model.observe_odom(1_000_000_000, _pose(), 1_000_000_000)
    assert model.accept_global(1_000_000_000, _pose())
    assert not model.accept_tracking(1_000_000_000, _pose(1.0, 0.0))
    snapshot = model.snapshot()
    assert snapshot.state is CoreState.DEGRADED
    assert snapshot.decision == 'REJECTED'
    assert snapshot.reason == 'tracking_suspect:innovation_gate'


def test_recovery_required_only_records_would_request():
    model = ShadowLocalizationModel(ShadowConfig())
    model.observe_odom(1_000_000_000, _pose(), 1_000_000_000)
    assert model.accept_global(1_000_000_000, _pose())
    model.observe_tracking_status('RECOVERY_REQUIRED', 10.0)
    snapshot = model.snapshot()
    assert snapshot.state is CoreState.RELOCALIZING
    assert snapshot.recovery_requested
    assert snapshot.decision == 'WOULD_REQUEST_RECOVERY'


def test_shadow_source_never_imports_or_mentions_tf_broadcaster():
    source = Path(__file__).resolve().parents[1] / 'agt_localization_ros' / 'shadow_manager_node.py'
    assert 'TransformBroadcaster' not in source.read_text(encoding='utf-8')
