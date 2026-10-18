"""S7-P0 isolated S6.1 frozen synthetic READY Route viewer matrix."""
from __future__ import annotations

from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import shutil

import pytest
import yaml

from agt_rviz_patrol.ready_route_resolver import inspect_view, read_json_regular

S6 = Path(os.environ["AGT_S7_S6_RUN"])
V4 = Path(os.environ["AGT_S7_V4_FROZEN"])
SOURCE = read_json_regular(S6 / "S6_ROUTE_REF_AND_EVIDENCE.json")


@pytest.fixture
def asset(tmp_path):
    root = tmp_path / "synthetic"
    shutil.copytree(S6, root)
    profile = root / "profile_snapshot.json"
    config = {
        "sites_root": str(root / "sites"),
        "route_store_root": str(root / "route_store"),
        "state_root": str(root / "state"),
        "site_schema": str(V4 / "src/agt_navigation_map_runtime/schemas/site_package.schema.json"),
        "profile_snapshot": str(profile),
        "profile_id": SOURCE["profile_id"],
        "profile_closure_digest": SOURCE["profile_closure_digest"],
        "profile_snapshot_sha256": hashlib.sha256(profile.read_bytes()).hexdigest(),
        "site_compatibility_key": "bunker",
        "site_id": SOURCE["site_id"],
        "site_revision": SOURCE["site_revision"],
        "site_aggregate_map_hash": SOURCE["site_aggregate_map_hash"],
        "activation_generation": SOURCE["activation_generation"],
    }
    return root, config, dict(SOURCE)


def expected(asset, state, code=None):
    root, config, ref = asset
    result = inspect_view(config, ref)
    assert result["state"] == state, result
    assert result["motion_permission"] == "DENIED_OFFLINE_NO_MOTION"
    if code:
        assert result["code"] == code, result
    return result


def test_s6_exact_ready_three_points(asset):
    result = expected(asset, "READY", "EXACT_ROUTE_V1_VERIFIED")
    assert result["point_count"] == 3
    assert result["review_status"] == "MOCK_ONLY"
    assert result["site_aggregate_map_hash"] == SOURCE["site_aggregate_map_hash"]
    assert result["profile_closure_digest"] == SOURCE["profile_closure_digest"]


@pytest.mark.parametrize("key,value,code", [
    ("route_id", "unknown", None),
    ("revision", 7, None),
    ("route_manifest_sha256", "sha256:" + "a" * 64, None),
    ("path_sha256", "sha256:" + "b" * 64, None),
    ("schema", "agt_route_reference_v2", "SCHEMA_INVALID"),
])
def test_wrong_route_ref(asset, key, value, code):
    asset[2][key] = value
    expected(asset, "INVALID", code)


@pytest.mark.parametrize("key,value,code", [
    ("site_id", "other_site", "SITE_SWITCHED"),
    ("site_revision", "r02", "SITE_SWITCHED"),
    ("site_aggregate_map_hash", "f" * 64, "MAP_HASH_DRIFT"),
    ("profile_id", "other_robot", "PROFILE_DRIFT"),
    ("profile_closure_digest", "e" * 64, "PROFILE_DRIFT"),
    ("activation_generation", 2, "GENERATION_DRIFT"),
])
def test_wrong_envelope_identity(asset, key, value, code):
    asset[2][key] = value
    expected(asset, "STALE", code)


def test_missing_generation_fails_closed(asset):
    asset[2].pop("activation_generation")
    expected(asset, "INVALID", "GENERATION_INVALID")


def test_local_draft_never_ready(asset):
    draft = read_json_regular(asset[0] / "three_points_draft.json")
    asset[2].clear()
    asset[2].update(draft)
    expected(asset, "INVALID", "SCHEMA_INVALID")


def test_site_pgm_drift(asset):
    path = asset[0] / "sites/greenhouse_test/r01/map/navigation.pgm"
    path.write_bytes(path.read_bytes() + b"\nS7 tamper")
    expected(asset, "STALE")


def test_selected_profile_snapshot_drift(asset):
    path = asset[0] / "profile_snapshot.json"
    path.write_bytes(path.read_bytes() + b" ")
    expected(asset, "STALE")


def test_configured_profile_closure_wrong(asset):
    asset[1]["profile_closure_digest"] = "d" * 64
    expected(asset, "STALE")


def test_configured_map_hash_wrong(asset):
    asset[1]["site_aggregate_map_hash"] = "d" * 64
    expected(asset, "STALE", "MAP_HASH_DRIFT")


def test_configured_generation_wrong(asset):
    asset[1]["activation_generation"] = 3
    expected(asset, "STALE", "GENERATION_DRIFT")


@pytest.mark.parametrize("filename", ["route.yaml", "route.csv"])
def test_route_sha_tamper(asset, filename):
    path = asset[0] / f"route_store/routes/s6_three_points/1/{filename}"
    path.write_bytes(path.read_bytes() + b"\nforged\n")
    expected(asset, "INVALID")


@pytest.mark.parametrize("state", ["DRAFT", "REVIEW"])
def test_route_not_ready_even_if_manifest_hash_recomputed(asset, state):
    path = asset[0] / "route_store/routes/s6_three_points/1/route.yaml"
    body = yaml.safe_load(path.read_bytes())
    body["status"] = state
    path.write_bytes(yaml.safe_dump(body, sort_keys=False).encode())
    asset[2]["route_manifest_sha256"] = "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()
    expected(asset, "INVALID")


def test_unknown_route_schema_even_if_manifest_hash_recomputed(asset):
    path = asset[0] / "route_store/routes/s6_three_points/1/route.yaml"
    body = yaml.safe_load(path.read_bytes())
    body["schema_version"] = 99
    path.write_bytes(yaml.safe_dump(body, sort_keys=False).encode())
    asset[2]["route_manifest_sha256"] = "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()
    expected(asset, "INVALID")


def test_symlinked_route_directory(asset):
    folder = asset[0] / "route_store/routes/s6_three_points/1"
    other = folder.parent / "1-old"
    folder.rename(other)
    folder.symlink_to(other, target_is_directory=True)
    expected(asset, "INVALID")


def test_site_switched_in_persisted_runtime(asset):
    state = asset[0] / "state/active_site.yaml"
    body = yaml.safe_load(state.read_bytes())
    body["site_id"] = "other_site"
    state.write_text(yaml.safe_dump(body), encoding="utf-8")
    expected(asset, "STALE")


def test_context_changes_during_route_load(asset, monkeypatch):
    from agt_rviz_patrol import ready_route_resolver as module
    true_resolve = module.resolve_active_route

    def switch(*args, **kwargs):
        loaded = true_resolve(*args, **kwargs)
        state = asset[0] / "state/active_site.yaml"
        body = yaml.safe_load(state.read_bytes())
        body["activation_generation"] += 1
        state.write_text(yaml.safe_dump(body), encoding="utf-8")
        return loaded

    monkeypatch.setattr(module, "resolve_active_route", switch)
    expected(asset, "STALE")


def test_selected_json_nofollow(tmp_path):
    file = tmp_path / "route.json"
    file.write_text(json.dumps(SOURCE), encoding="utf-8")
    link = tmp_path / "symlink.json"
    link.symlink_to(file)
    with pytest.raises(OSError):
        read_json_regular(link)
