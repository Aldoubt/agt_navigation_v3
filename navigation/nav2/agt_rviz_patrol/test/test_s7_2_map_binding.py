"""Fail-closed matrix against disposable copy of S6.1 synthetic Site."""
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil

import pytest

from agt_rviz_patrol.ready_route_resolver import _current
from agt_rviz_patrol.map_view_binding import inspect_bound_view, source_grid

S6=Path(os.environ["AGT_S7_S6_RUN"])
V4=Path(os.environ["AGT_S7_V4_FROZEN"])
SOURCE=json.loads((S6/"S6_ROUTE_REF_AND_EVIDENCE.json").read_bytes())


@pytest.fixture
def e(tmp_path,monkeypatch):
    root=tmp_path/"synthetic"
    shutil.copytree(S6,root)
    p=root/"profile_snapshot.json"
    cfg=dict(sites_root=str(root/"sites"),route_store_root=str(root/"route_store"),
        state_root=str(root/"state"),
        site_schema=str(V4/"src/agt_navigation_map_runtime/schemas/site_package.schema.json"),
        profile_snapshot=str(p),profile_id=SOURCE["profile_id"],
        profile_closure_digest=SOURCE["profile_closure_digest"],
        profile_snapshot_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
        site_compatibility_key="bunker",site_id=SOURCE["site_id"],
        site_revision=SOURCE["site_revision"],
        site_aggregate_map_hash=SOURCE["site_aggregate_map_hash"],
        activation_generation=SOURCE["activation_generation"],
        view_mode="synthetic_site_fixture")
    monkeypatch.setenv("AGT_S7_2_SYNTHETIC_MAP_ONLY","1")
    _,summary=_current(cfg)
    expected=source_grid(root/"sites/greenhouse_test/r01",summary)
    view=dict(
        width=expected["width"],height=expected["height"],
        resolution=expected["resolution"],origin=list(expected["origin"]),
        grid_sha256=expected["grid_sha256"],
        yaml_filename=expected["yaml_path"],frame_id="map",
        publisher_count=1,publisher_node="/map_server",
        map_server_node="/map_server",
        map_id=SOURCE["site_id"],map_version=SOURCE["site_revision"])
    return root,cfg,dict(SOURCE),view


def check(e,bound,code=None,asset=True):
    _,config,ref,view=e
    result=inspect_bound_view(config,ref,view)
    assert result["asset_ready_verified"] is asset,result
    assert result["map_bound_to_view"] is bound,result
    assert result["motion_permission"]=="DENIED_OFFLINE_NO_MOTION"
    assert ("_preview_poses" in result)==bound
    if code:
        assert result["code"]==code,result
    return result


def test_true_synthetic_site_navigation_and_map_server(e):
    r=check(e,True,"EXACT_MAP_VIEW_BOUND")
    assert len(r["_preview_poses"])==3
    assert r["state"]=="MAP_BOUND_TO_VIEW"
    assert r["review_status"]=="MOCK_ONLY"
    assert r["view_pgm_sha256"]!=r["site_aggregate_map_hash"]
    assert r["view_yaml_sha256"]!=r["site_aggregate_map_hash"]


@pytest.mark.parametrize("key,value,code",[
    ("map_id","bunker_mid360","CROSS_MAP_IDENTITY_MISMATCH"),
    ("map_version","20260924-trav-integration-v1","CROSS_MAP_IDENTITY_MISMATCH"),
    ("map_id","other_site","CROSS_MAP_IDENTITY_MISMATCH"),
    ("frame_id","odom","MAP_FRAME_MISMATCH"),
    ("resolution",0.10,"MAP_RESOLUTION_MISMATCH"),
    ("origin",[0.2,0.,0.],"MAP_ORIGIN_MISMATCH"),
    ("origin",[0.,0.,0.15],"MAP_ORIGIN_MISMATCH"),
    ("width",2,"MAP_GEOMETRY_MISMATCH"),
    ("grid_sha256","f"*64,"MAP_GRID_CONTENT_MISMATCH"),
    ("yaml_filename","/tmp/other/nav.yaml","MAP_SERVER_SOURCE_MISMATCH"),
    ("publisher_count",2,"MAP_VIEW_SOURCE_UNKNOWN"),
    ("publisher_node","/other_map","MAP_VIEW_SOURCE_UNKNOWN"),
])
def test_wrong_live_map_fails_closed(e,key,value,code):
    e[3][key]=value
    r=check(e,False,code)
    assert r["state"]=="ASSET_READY_VERIFIED"
    assert r["map_binding_state"]=="MAP_NOT_BOUND_TO_VIEW"
    assert r["route_id"]=="s6_three_points"
    assert r["profile_closure_digest"]==SOURCE["profile_closure_digest"]


def test_missing_source(e):
    e[3].clear()
    check(e,False,"MAP_VIEW_SOURCE_UNKNOWN")


def test_no_live_map(e):
    e=(e[0],e[1],e[2],None)
    check(e,False,"MAP_VIEW_SOURCE_UNKNOWN")


def test_same_map_frame_different_pixels(e):
    assert e[3]["frame_id"]=="map"
    e[3]["grid_sha256"]="0"*64
    check(e,False,"MAP_GRID_CONTENT_MISMATCH")


def test_copied_identical_yaml_not_trusted(e,tmp_path):
    new=tmp_path/"other.yaml"
    new.write_bytes(Path(e[3]["yaml_filename"]).read_bytes())
    e[3]["yaml_filename"]=str(new)
    check(e,False,"MAP_SERVER_SOURCE_MISMATCH")


def test_source_pgm_sha_drift(e):
    p=e[0]/"sites/greenhouse_test/r01/map/navigation.pgm"
    p.write_bytes(p.read_bytes()+b"\n# tampered")
    r=check(e,False,asset=False)
    assert r["state"]=="STALE"


def test_source_yaml_sha_drift(e):
    p=e[0]/"sites/greenhouse_test/r01/map/navigation.yaml"
    p.write_bytes(p.read_bytes()+b"\n# tampered")
    r=check(e,False,asset=False)
    assert r["state"]=="STALE"


def test_synthetic_guard_required(e,monkeypatch):
    monkeypatch.delenv("AGT_S7_2_SYNTHETIC_MAP_ONLY")
    check(e,False,"SYNTHETIC_MODE_DENIED")


def test_unknown_mode(e):
    e[1]["view_mode"]="unknown"
    check(e,False,"MAP_VIEW_SOURCE_UNKNOWN")


def test_route_manifest_sha_wrong(e):
    e[2]["route_manifest_sha256"]="sha256:"+"e"*64
    r=check(e,False,asset=False)
    assert r["state"]=="INVALID"


def test_local_draft_cannot_promote(e):
    e[2].clear()
    e[2].update(json.loads((e[0]/"three_points_draft.json").read_bytes()))
    check(e,False,asset=False)


def test_profile_closure_drift(e):
    e[2]["profile_closure_digest"]="a"*64
    r=check(e,False,asset=False)
    assert r["state"]=="STALE"


def test_activation_generation_drift(e):
    e[2]["activation_generation"]=12
    r=check(e,False,asset=False)
    assert r["state"]=="STALE"


def test_site_switch(e):
    e[2]["site_id"]="bunker_mid360"
    r=check(e,False,asset=False)
    assert r["state"]=="STALE"


def test_navigation_yaml_symlink_refused(e):
    # SiteSnapshot verifier is responsible for refusing Site asset symlinks.
    p=e[0]/"sites/greenhouse_test/r01/map/navigation.yaml"
    old=p.with_suffix(".old")
    p.rename(old);p.symlink_to(old)
    check(e,False,asset=False)
