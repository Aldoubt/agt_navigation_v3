"""S7.2 strict *display* binding. No new MapRef or RouteRef protocol.

V4 SiteSummary supplies source navigation YAML + validated image hashes; V3
resolve_map supplies independently registered map identity in normal RViz mode.
Only the actual Map Server YAML parameter and the live /map grid can attest
what is on screen. Absence/ambiguity means no READY geometry overlay.
"""
from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path
from typing import Mapping

import yaml

from agt_navigation_map_runtime.site_contracts import read_site_asset
from agt_rviz_patrol.ready_route_resolver import ViewRefused, _current


def inspect_bound_view(config: Mapping, selected: Mapping, view: Mapping | None) -> dict:
    """Keep asset validation independent from view binding, and clear unsafe geometry."""
    from agt_rviz_patrol.ready_route_resolver import inspect_view
    asset = inspect_view(config, selected, include_preview=True)
    if asset["state"] != "READY":
        asset["asset_ready_verified"] = False
        asset["map_bound_to_view"] = False
        asset["map_binding_state"] = "STALE" if asset["state"] == "STALE" else "INVALID"
        return asset
    asset.update(state="ASSET_READY_VERIFIED", asset_ready_verified=True,
                 map_bound_to_view=False, map_binding_state="MAP_NOT_BOUND_TO_VIEW")
    try:
        result = map_view_binding(config,view,asset)
    except ViewRefused as exc:
        asset.pop("_preview_poses",None)
        asset.update(code=exc.code, message=str(exc))
        if exc.state=="STALE":
            asset.update(state="STALE",map_binding_state="STALE")
        return asset
    except Exception as exc:
        asset.pop("_preview_poses",None)
        asset.update(code="MAP_VIEW_SOURCE_UNVERIFIED",
                     message=f"{type(exc).__name__}: unverified map binding; refusing overlay")
        return asset
    asset.update(result)
    asset.update(state="MAP_BOUND_TO_VIEW",code="EXACT_MAP_VIEW_BOUND",
                 message="Asset READY and live RViz Map Server geometry/provenance verified; NO MOTION")
    return asset


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _parse_p5(raw: bytes) -> tuple[int, int, bytes]:
    if len(raw) > 128 * 1024 * 1024 or not raw.startswith((b"P5",b"P2")):
        raise ValueError("only bounded grayscale PGM P2/P5 is supported")
    if raw.startswith(b"P2"):
        cleaned=b"\n".join(line.split(b"#",1)[0] for line in raw.splitlines())
        pieces=cleaned.split()
        if len(pieces)<4 or pieces[0]!=b"P2" or pieces[3]!=b"255":
            raise ValueError("invalid grayscale P2 header")
        width,height=int(pieces[1]),int(pieces[2])
        if not 0<width<=10000 or not 0<height<=10000 or width*height>25_000_000:
            raise ValueError("invalid P2 dimensions")
        if len(pieces)!=width*height+4:
            raise ValueError("P2 raster length mismatch")
        pixels=bytes(int(v) for v in pieces[4:])
        return width,height,pixels
    pos = 0
    tokens = []
    while len(tokens) < 4:
        while pos < len(raw) and raw[pos] in b" \n\t\r":
            pos += 1
        if pos < len(raw) and raw[pos] == 35:  # PGM comments
            while pos < len(raw) and raw[pos] not in b"\n\r":
                pos += 1
            continue
        start = pos
        while pos < len(raw) and raw[pos] not in b" \n\t\r":
            pos += 1
        if pos == start or pos - start > 32:
            raise ValueError("malformed PGM header")
        tokens.append(raw[start:pos])
    if tokens[0] != b"P5" or tokens[3] != b"255":
        raise ValueError("unsupported PGM format/depth")
    width, height = int(tokens[1]), int(tokens[2])
    if not 0 < width <= 10000 or not 0 < height <= 10000 or width * height > 25_000_000:
        raise ValueError("invalid grid dimensions")
    if pos >= len(raw) or raw[pos] not in b" \n\r\t":
        raise ValueError("missing PGM header separator")
    pos += 1  # A single separator before binary payload; binary may start with whitespace.
    if len(raw) - pos != width * height:
        raise ValueError("PGM raster size mismatch")
    return width, height, raw[pos:]


def _read_bounded_regular(path: Path, max_bytes: int) -> bytes:
    fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        before = os.fstat(fd)
        if not 0 < before.st_size <= max_bytes or not os.path.isfile(path):
            raise ValueError("map source not regular/within limits")
        data = os.read(fd, max_bytes + 1)
        after = os.fstat(fd)
        fields = ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")
        if len(data) != before.st_size or any(getattr(before,k)!=getattr(after,k) for k in fields):
            raise ValueError("map asset changed while reading")
        return data
    finally:
        os.close(fd)


def _cell_data(pixels: bytes, width: int, height: int, nav: dict) -> bytes:
    # Nav2 trinary pixel semantics, bottom-left ROS occupancy grid row first.
    # Return raw bytes: -1 encoded as 255, 0 free, 100 occupied.
    # Restrict to known trinary mode rather than claim other Nav2 modes equivalent.
    if nav.get("mode", "trinary") != "trinary":
        raise ValueError("unsupported Nav2 map mode")
    negate = nav.get("negate", 0)
    if type(negate) is not int or negate not in (0, 1):
        raise ValueError("invalid map negate")
    occ = float(nav.get("occupied_thresh", 0.65))
    free = float(nav.get("free_thresh", 0.196))
    if not (math.isfinite(occ) and math.isfinite(free) and 0 <= free < occ <= 1):
        raise ValueError("invalid map thresholds")
    table = []
    for gray in range(256):
        p = (gray / 255.0) if negate else (255-gray) / 255.0
        table.append(100 if p > occ else (0 if p < free else 255))
    return bytes(table[b] for row in range(height-1,-1,-1)
                 for b in pixels[row*width:(row+1)*width])


def source_grid(site_root: Path, summary) -> dict:
    """Read only V4-validated Site assets; verify *per asset* SHA separately."""
    nav_path = Path(summary.navigation_yaml)
    relative = nav_path.relative_to(site_root)
    nav_bytes = read_site_asset(site_root, relative.as_posix())
    if _sha(nav_bytes) != summary.navigation_yaml_sha256:
        raise ViewRefused("STALE", "SITE_NAV_YAML_HASH_DRIFT", "Site navigation YAML bytes changed")
    nav = yaml.safe_load(nav_bytes)
    if not isinstance(nav, dict):
        raise ValueError("invalid navigation map YAML")
    raw_image = nav.get("image")
    if (not isinstance(raw_image, str) or not raw_image
        or Path(raw_image).is_absolute() or ".." in Path(raw_image).parts):
        raise ValueError("untrusted navigation image path")
    image_rel = (relative.parent / raw_image).as_posix()
    pgm_bytes = read_site_asset(site_root, image_rel)
    if _sha(pgm_bytes) != summary.navigation_image_sha256:
        raise ViewRefused("STALE", "SITE_NAV_PGM_HASH_DRIFT", "Site navigation PGM bytes changed")
    width, height, pixels = _parse_p5(pgm_bytes)
    origin = nav.get("origin")
    resolution = nav.get("resolution")
    if (not isinstance(origin, list) or len(origin) != 3
        or type(resolution) not in (int, float)
        or not math.isfinite(float(resolution)) or resolution <= 0
        or any(type(v) not in (float, int) or not math.isfinite(float(v)) for v in origin)):
        raise ValueError("invalid site navigation geometry")
    return {
        "yaml_path": str(nav_path),
        "yaml_sha256": summary.navigation_yaml_sha256,
        "pgm_sha256": summary.navigation_image_sha256,
        "width": width, "height": height,
        "resolution": float(resolution),
        "origin": [float(v) for v in origin],
        "grid_sha256": _sha(_cell_data(pixels,width,height,nav)),
    }


def map_view_binding(config: Mapping, view: Mapping | None, asset: Mapping) -> dict:
    """Bind a *validated asset* to a separately observed V3 RViz/Map Server map."""
    if not view:
        raise ViewRefused("MAP_NOT_BOUND_TO_VIEW", "MAP_VIEW_SOURCE_UNKNOWN",
                          "No fresh /map and Map Server authoritative source")
    # The viewer passes this only from real ROS messages/parameter service.
    if (view.get("publisher_count") != 1 or
        view.get("publisher_node") != "/map_server" or
        view.get("map_server_node") != "/map_server" or
        not view.get("yaml_filename")):
        raise ViewRefused("MAP_NOT_BOUND_TO_VIEW", "MAP_VIEW_SOURCE_UNKNOWN",
                          "Map Server origin is unavailable, ambiguous or untrusted")
    if (view.get("map_id"), view.get("map_version")) != (
            asset["site_id"], asset["site_revision"]):
        raise ViewRefused("MAP_NOT_BOUND_TO_VIEW", "CROSS_MAP_IDENTITY_MISMATCH",
                          f"Route Site {asset['site_id']}/{asset['site_revision']} differs "
                          f"from RViz map {view.get('map_id','?')}/{view.get('map_version','?')}")
    if view.get("frame_id") != "map":
        raise ViewRefused("MAP_NOT_BOUND_TO_VIEW", "MAP_FRAME_MISMATCH",
                          "RViz live occupancy grid frame is not the Site Route frame")
    ctx, summary = _current(config)
    if (ctx.site_id, ctx.site_revision) != (asset["site_id"], asset["site_revision"]):
        raise ViewRefused("STALE", "SITE_SWITCHED", "Active Site changed during map binding")
    site_root = Path(config["sites_root"]) / ctx.site_id / ctx.site_revision
    site = source_grid(site_root, summary)
    mode = config.get("view_mode", "registry")
    if mode == "registry":
        # Existing V3 MapManager registry contract. Only validated map package
        # is eligible; nothing inferred from shared 'map' frame alone.
        from agt_map_manager.map_catalog import resolve_map
        resolved = resolve_map(str(config.get("view_map_spec","")),
                               str(config.get("view_robot","")),
                               Path(str(config.get("view_registry",""))))
        if (resolved.map_id,resolved.map_version) != (asset["site_id"],asset["site_revision"]):
            raise ViewRefused("MAP_NOT_BOUND_TO_VIEW", "CROSS_MAP_IDENTITY_MISMATCH",
                              "V3 Registry Map Package differs from Route Site")
        if Path(resolved.navigation_map) != Path(site["yaml_path"]):
            raise ViewRefused("MAP_NOT_BOUND_TO_VIEW", "MAP_PROVENANCE_MISMATCH",
                              "Registry map source has no verified Site nav asset lineage")
        expected_yaml = str(resolved.navigation_map)
    elif mode == "synthetic_site_fixture":
        if os.environ.get("AGT_S7_2_SYNTHETIC_MAP_ONLY") != "1":
            raise ViewRefused("MAP_NOT_BOUND_TO_VIEW", "SYNTHETIC_MODE_DENIED",
                              "test-only Site view binding cannot be used without lab guard")
        expected_yaml = site["yaml_path"]
    else:
        raise ViewRefused("MAP_NOT_BOUND_TO_VIEW", "MAP_VIEW_SOURCE_UNKNOWN",
                          "unrecognized map provenance mode")
    if Path(str(view["yaml_filename"])) != Path(expected_yaml):
        raise ViewRefused("MAP_NOT_BOUND_TO_VIEW", "MAP_SERVER_SOURCE_MISMATCH",
                          "Live Map Server yaml_filename differs from verified Site navigation YAML")
    for name in ("width","height"):
        if type(view.get(name)) is not int or view[name] != site[name]:
            raise ViewRefused("MAP_NOT_BOUND_TO_VIEW","MAP_GEOMETRY_MISMATCH",
                              f"Live map {name} differs from Site grid")
    if type(view.get("resolution")) not in (float,int) or not math.isclose(
            view["resolution"],site["resolution"],rel_tol=0,abs_tol=1e-7):
        raise ViewRefused("MAP_NOT_BOUND_TO_VIEW","MAP_RESOLUTION_MISMATCH",
                          "Live grid resolution differs from Site map YAML")
    origin = view.get("origin")
    if (not isinstance(origin,(tuple,list)) or len(origin)!=3 or
        any(type(v) not in (int,float) or not math.isfinite(v) for v in origin) or
        any(not math.isclose(float(a),b,abs_tol=1e-6,rel_tol=0)
            for a,b in zip(origin,site["origin"]))):
        raise ViewRefused("MAP_NOT_BOUND_TO_VIEW","MAP_ORIGIN_MISMATCH",
                          "Live grid origin/heading differs from Site map YAML")
    if view.get("grid_sha256") != site["grid_sha256"]:
        raise ViewRefused("MAP_NOT_BOUND_TO_VIEW","MAP_GRID_CONTENT_MISMATCH",
                          "Same map frame/geometry but different live occupancy content")
    # Do NOT compare aggregate Site map_hash with YAML or PGM SHA. They are
    # dissimilar contracts. SiteSummary attests both per-file digests.
    return {
        "map_bound_to_view": True, "map_binding_state": "MAP_BOUND_TO_VIEW",
        "view_map_id": asset["site_id"], "view_map_version": asset["site_revision"],
        "view_frame": view["frame_id"], "view_yaml_sha256": site["yaml_sha256"],
        "view_pgm_sha256": site["pgm_sha256"],
        "view_grid_sha256": site["grid_sha256"],
    }
