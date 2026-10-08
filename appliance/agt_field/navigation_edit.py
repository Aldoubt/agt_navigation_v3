"""Publish a reviewed Qt 2D edit as a new immutable bundle, retaining 3D identity."""

from pathlib import Path
import os
import shutil
import uuid

from .bundle import navigation_check, seal, validate_bundle
from .contracts import ContractError, atomic_yaml, contained, files, read_yaml, sha


def immutable_copy(source, destination):
    try:
        return os.link(source, destination)
    except OSError:
        # Some data volumes do not support hard links. Do not require them for correctness.
        return shutil.copy2(source, destination)


def publish_navigation_edit(source, draft_yaml, data, binding, versions, *, mock=False):
    from agt_map_converter.validate_nav_map import read_pgm_header

    source, data = Path(source).resolve(), Path(data).resolve()
    if validate_bundle(source, allow_mock=mock) != binding:
        raise ContractError("edited map source binding changed")
    draft = Path(draft_yaml)
    staging = data / "run/map_edits"
    try:
        relative = draft.relative_to(staging)
    except ValueError as exc:
        raise ContractError("map edit must be staged under run/map_edits") from exc
    draft = contained(staging, str(relative))
    if draft.name != "map.yaml":
        raise ContractError("map edit must use map.yaml")
    files(draft.parent)  # Reject symlinks, including image/topology inputs.
    navigation_check(draft.parent)
    old = read_yaml(source / "navigation/map.yaml")
    new = read_yaml(draft)
    # Qt is an occupancy editor, not a frame/scale editor. Prevent 2D/3D misalignment.
    if new["resolution"] != old["resolution"] or new["origin"] != old["origin"]:
        raise ContractError("edited map changed resolution/origin")
    width, height, payload = read_pgm_header(draft.parent / "map.pgm")
    old_width, old_height, _ = read_pgm_header(contained(source / "navigation", old["image"]))
    if width <= 0 or height <= 0 or (width, height) != (old_width, old_height):
        raise ContractError("edited map changed dimensions or is invalid")
    if new.get("mode", "trinary") != "trinary" or int(new.get("negate", 0)) != 0:
        raise ContractError("Qt navigation edit must be trinary with negate=0")
    if 205 in payload and float(new["free_thresh"]) > 50.0 / 255.0:
        raise ContractError("edited unknown cells would become free")

    identity = dict(
        map_bundle_id=binding["map_bundle_id"],
        map_version=binding["map_version"] + ".qt-" + uuid.uuid4().hex[:12],
    )
    destination = data / "maps" / identity["map_bundle_id"] / identity["map_version"]
    destination.mkdir(parents=True, exist_ok=False)
    try:
        # Frozen 3D files can share storage; every mutable metadata write uses atomic replace.
        for domain in ("mapping", "localization"):
            shutil.copytree(source / domain, destination / domain, copy_function=immutable_copy)
        shutil.copytree(source / "navigation", destination / "navigation")
        for name in ("map.pgm", "map.yaml", "map.topology"):
            if (draft.parent / name).exists():
                target = destination / "navigation" / name
                target.unlink(missing_ok=True)
                shutil.copy2(draft.parent / name, target)
        new["image"] = "map.pgm"
        atomic_yaml(destination / "navigation/map.yaml", new)
        loc = read_yaml(destination / "localization/metadata.yaml")
        loc["map"] = identity
        atomic_yaml(destination / "localization/metadata.yaml", loc)
        provenance = dict(
            source_bundle=binding,
            editor="AGT Qt HMI",
            draft_sha256=sha(draft.parent / "map.pgm"),
            unchanged_geometry=True,
            versions=versions,
        )
        atomic_yaml(destination / "navigation/qt_edit.yaml", provenance)
        atomic_yaml(
            destination / "navigation/review_status.yaml",
            dict(
                status="confirmed",
                source_mapping_package=str(destination / "mapping"),
                editor="AGT Qt HMI",
                source_bundle=binding,
            ),
        )
        seal(destination, identity, versions, mock=mock)
        return destination
    except Exception:
        shutil.rmtree(destination)
        raise
