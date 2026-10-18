"""S7-P0 read-only READY viewer binding to the *existing* V4 Route v1 loader.

The view request is the S6 RouteRef/evidence JSON; it is NOT a second RouteRef
schema. All trusted Site/Profile roots and their expected identity are supplied
separately by the offline deployment, never by the selected view request.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import stat
from typing import Mapping

from agt_navigation_map_runtime.activation_store import ActivationStore
from agt_navigation_map_runtime.profile_site_compatibility import ProfileSnapshotProvider
from agt_navigation_map_runtime.registry import SiteRegistry
from agt_navigation_map_runtime.runtime_policy import SiteRuntimePolicy
from agt_navigation_map_runtime.validator import SiteValidator
from agt_navigation_route_runtime.route_runtime import RouteReference, RouteRuntimeError
from agt_navigation_route_runtime.site_route_resolver import (
    active_map_context_from_runtime_context, resolve_active_route,
)


class ViewRefused(ValueError):
    def __init__(self, state: str, code: str, reason: str):
        super().__init__(reason)
        self.state, self.code = state, code


def read_json_regular(path: str | Path, max_bytes: int = 65536) -> dict:
    """Bounded, no-follow read; never interpret a DRAFT as an eligible ref."""
    fd = os.open(str(path), os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        info_before = os.fstat(fd)
        if not stat.S_ISREG(info_before.st_mode) or not 0 < info_before.st_size <= max_bytes:
            raise ValueError("selection must be a bounded regular JSON file")
        raw = os.read(fd, max_bytes + 1)
        info_after = os.fstat(fd)
        if (len(raw) != info_after.st_size or
            (info_before.st_dev, info_before.st_ino, info_before.st_mtime_ns,
             info_before.st_ctime_ns, info_before.st_size) !=
            (info_after.st_dev, info_after.st_ino, info_after.st_mtime_ns,
             info_after.st_ctime_ns, info_after.st_size)):
            raise ValueError("selection changed while being read")
    finally:
        os.close(fd)
    result = json.loads(raw.decode("utf-8"))
    if not isinstance(result, dict):
        raise ValueError("selection must be a JSON object")
    return result


def _required_string(obj: Mapping, field: str) -> str:
    value = obj.get(field)
    if not isinstance(value, str) or not value:
        raise ViewRefused("INVALID", "REF_FIELD_INVALID", f"{field} is required")
    return value


def _require_generation(obj: Mapping) -> int:
    value = obj.get("activation_generation")
    if type(value) is not int or value < 1:
        raise ViewRefused("INVALID", "GENERATION_INVALID", "activation_generation must be a positive integer")
    return value


def _policy(config: Mapping) -> SiteRuntimePolicy:
    sites = Path(_required_string(config, "sites_root"))
    provider = ProfileSnapshotProvider(
        _required_string(config, "profile_snapshot"),
        _required_string(config, "profile_id"),
        _required_string(config, "profile_closure_digest"),
        _required_string(config, "profile_snapshot_sha256"),
        _required_string(config, "site_compatibility_key"),
    )
    return SiteRuntimePolicy(
        SiteRegistry(sites),
        SiteValidator(_required_string(config, "site_compatibility_key"),
                      _required_string(config, "site_schema")),
        ActivationStore(_required_string(config, "state_root"), sites_root=sites),
        profile_provider=provider,
    )


def _current(config: Mapping):
    """Always reconstruct runtime policy from *persisted* authority."""
    policy = _policy(config)
    result = policy.restore()  # validates current Site bytes and Profile closure
    ctx, summary = policy.active_context, policy.active_summary
    if not result.success or ctx is None or summary is None or not summary.valid:
        codes = ",".join(result.blocker_codes) or "NO_ACTIVE_SITE"
        raise ViewRefused("STALE", codes, result.message or "active Site cannot be restored")
    if summary.map_hash != ctx.site_digest or summary.manifest_sha256 != ctx.site_manifest_sha256:
        raise ViewRefused("STALE", "SITE_SUMMARY_DRIFT", "current SiteSummary differs from active context")
    return ctx, summary


def _identity(ctx) -> tuple:
    return (ctx.site_id, ctx.site_revision, ctx.site_digest, ctx.site_manifest_sha256,
            ctx.profile_id, ctx.profile_digest, ctx.profile_snapshot_sha256,
            ctx.site_compatibility_key, ctx.activation_generation)


def verify_ready_view(config: Mapping, selected: Mapping, *, include_preview: bool = False) -> dict:
    """One *view-only* resolution; no publication, Task, action or motion topics."""
    if selected.get("schema") != "agt_route_reference_v1":
        raise ViewRefused("INVALID", "SCHEMA_INVALID", "only agt_route_reference_v1 may be viewed as READY")
    # S6 envelope, not local DRAFT. Require exact Site/Profile/generation evidence
    # rather than guessing them from RouteRef (which does not encode generation).
    fields = ("site_id", "site_revision", "site_aggregate_map_hash",
              "profile_id", "profile_closure_digest")
    for field in fields:
        _required_string(selected, field)
    generation = _require_generation(selected)
    ref = RouteReference(
        route_id=_required_string(selected, "route_id"),
        revision=selected.get("revision"),
        route_manifest_sha256=_required_string(selected, "route_manifest_sha256"),
        path_sha256=_required_string(selected, "path_sha256"),
        schema=selected["schema"],
    )
    first, summary = _current(config)
    if (selected["site_id"], selected["site_revision"]) != (first.site_id, first.site_revision):
        raise ViewRefused("STALE", "SITE_SWITCHED", "selected Site differs from current active Site")
    if selected["site_aggregate_map_hash"] != summary.map_hash:
        raise ViewRefused("STALE", "MAP_HASH_DRIFT", "selected aggregate map_hash differs from current SiteSummary")
    if (selected["profile_id"], selected["profile_closure_digest"]) != (first.profile_id, first.profile_digest):
        raise ViewRefused("STALE", "PROFILE_DRIFT", "selected Profile closure differs from current runtime")
    if generation != first.activation_generation:
        raise ViewRefused("STALE", "GENERATION_DRIFT", "activation_generation changed; reselect authority")
    # The configured viewer must never silently rebind to a different deployment.
    if (first.site_id, first.site_revision, first.profile_id) != (
        _required_string(config, "site_id"), _required_string(config, "site_revision"),
        _required_string(config, "profile_id"),
    ):
        raise ViewRefused("STALE", "DEPLOYMENT_DRIFT", "active identity differs from trusted viewer configuration")
    if first.profile_digest != config["profile_closure_digest"]:
        raise ViewRefused("STALE", "PROFILE_DRIFT", "active closure differs from trusted viewer configuration")
    if summary.map_hash != _required_string(config, "site_aggregate_map_hash"):
        raise ViewRefused("STALE", "MAP_HASH_DRIFT", "active Site map_hash differs from frozen viewer configuration")
    if first.activation_generation != _require_generation(config):
        raise ViewRefused("STALE", "GENERATION_DRIFT", "active generation differs from frozen viewer configuration")

    asset = resolve_active_route(
        _required_string(config, "route_store_root"),
        _required_string(config, "sites_root"),
        ref,
        active_map_context_from_runtime_context(first),
    )
    # Re-open persisted authority *after* disk Route load; never trust a cached
    # ActiveMapContext from before a Site activation or Profile/source mutation.
    second, second_summary = _current(config)
    if _identity(first) != _identity(second) or second_summary.map_hash != summary.map_hash:
        raise ViewRefused("STALE", "CONTEXT_CHANGED_DURING_VIEW",
                          "runtime authority changed during route load")
    result = {
        "state": "READY", "code": "EXACT_ROUTE_V1_VERIFIED",
        "route_id": asset.route_id, "revision": asset.revision,
        "route_manifest_sha256": asset.route_manifest_sha256,
        "path_sha256": asset.path_sha256,
        "site_id": first.site_id, "site_revision": first.site_revision,
        "site_aggregate_map_hash": summary.map_hash,
        "profile_id": first.profile_id, "profile_closure_digest": first.profile_digest,
        "activation_generation": first.activation_generation,
        "point_count": sum(len(segment.points) for segment in asset.segments),
        "review_status": "MOCK_ONLY" if selected.get("status") == "OFFLINE_ROUTE_ASSET_TEST_ONLY" else "NOT_VERIFIED",
        "motion_permission": "DENIED_OFFLINE_NO_MOTION",
        "message": "Verified read-only with V4 resolve_active_route(); never executable",
    }
    if include_preview:
        # Display-only geometry; never publish FollowRoute or edit LOCAL DRAFT.
        points = [p for segment in asset.segments for p in segment.points]
        step = max(1, (len(points) + 1998) // 1999)
        visible = points[::step]
        if points and visible[-1] is not points[-1]:
            visible.append(points[-1])
        result["_preview_poses"] = [(p.x, p.y, p.yaw) for p in visible]
    return result


def inspect_view(config: Mapping, selected: Mapping, *, include_preview: bool = False) -> dict:
    try:
        return verify_ready_view(config, selected, include_preview=include_preview)
    except ViewRefused as exc:
        return {"state": exc.state, "code": exc.code, "message": str(exc),
                "motion_permission": "DENIED_OFFLINE_NO_MOTION"}
    except RouteRuntimeError as exc:
        return {"state": "INVALID", "code": exc.code, "message": str(exc),
                "motion_permission": "DENIED_OFFLINE_NO_MOTION"}
    except Exception as exc:
        # Do not echo paths or stack traces into an operator screen.
        return {"state": "INVALID", "code": "VIEW_VALIDATION_FAILED",
                "message": type(exc).__name__ + ": view request rejected",
                "motion_permission": "DENIED_OFFLINE_NO_MOTION"}
