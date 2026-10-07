"""Mapping-owned artifacts -> immutable, hash-bound V3 consumer bundle.

Native executables create real descriptors/index. Mock markers never enter real mode.
"""

from __future__ import annotations

import math
import shutil
import subprocess
from pathlib import Path

from .contracts import (
    ContractError,
    atomic_yaml,
    contained,
    digest_data,
    files,
    identifier,
    read_yaml,
    sha,
)

MAPPING_FILES = {
    "map.pcd",
    "poses.txt",
    "poses_timed.txt",
    "calibration.yaml",
    "metadata.yaml",
    "manifest.yaml",
    "checksums.sha256",
}
LOCALIZATION_FILES = {
    "polar_context.db",
    "polar_context.yaml",
    "relocalization_assets.yaml",
    "global_map_downsampled.pcd",
    "voxelmaps_coords/voxel_params.txt",
}


def poses(root):
    root = Path(root)
    records = []
    names = set()
    timed = {}
    for line in (root / "poses_timed.txt").read_text().splitlines():
        fields = line.split()
        if len(fields) != 9 or fields[0] in timed:
            raise ContractError("invalid timed pose record")
        values = [float(v) for v in fields[1:]]
        if not all(math.isfinite(v) for v in values):
            raise ContractError("nonfinite timed pose")
        timed[fields[0]] = values
    for line in (root / "poses.txt").read_text().splitlines():
        f = line.split()
        if len(f) != 8 or f[0] in names or Path(f[0]).name != f[0] or not f[0].endswith(".pcd"):
            raise ContractError("invalid pose or patch id")
        v = [float(x) for x in f[1:]]
        if not all(math.isfinite(x) for x in v) or abs(sum(x * x for x in v[3:]) - 1) > 0.002:
            raise ContractError("invalid pose quaternion")
        if f[0] not in timed or any(abs(a - b) > 1e-6 for a, b in zip(v, timed[f[0]][1:])):
            raise ContractError("timed/optimized pose disagreement")
        patch = contained(root / "patches", f[0])
        nonempty_pcd(patch)
        names.add(f[0])
        records.append(
            dict(
                source_keyframe_id=f[0],
                source_pose=v,
                stamp=timed[f[0]][0],
                output_patch="patches/" + f[0],
                output_pose=v,
                sha256=sha(patch),
            )
        )
    if (
        not records
        or names != {p.name for p in (root / "patches").glob("*.pcd")}
        or names != set(timed)
    ):
        raise ContractError("keyframe/pose coverage mismatch")
    return records


def nonempty_pcd(path):
    with Path(path).open("rb") as stream:
        header = b""
        for _ in range(200):
            line = stream.readline(4096)
            header += line
            if len(header) > 65536:
                break
            if line.startswith(b"DATA "):
                counts = [
                    item.split()[1] for item in header.splitlines() if item.startswith(b"POINTS ")
                ]
                if not counts or int(counts[0]) <= 0 or not stream.read(1):
                    raise ContractError("empty PCD")
                return
    raise ContractError("invalid PCD header")


def validate_mapping(root):
    root = Path(root)
    actual = files(root)
    if not MAPPING_FILES <= actual.keys():
        raise ContractError("mapping package incomplete")
    meta = read_yaml(root / "metadata.yaml")
    if meta.get("backend") != "PGO" or meta.get("backend_status", {}).get("optimized") is not True:
        raise ContractError("optimized PGO required")
    manifest = read_yaml(root / "manifest.yaml")
    index = {}
    for line in (root / "checksums.sha256").read_text().splitlines():
        parts = line.split("  ", 1)
        if len(parts) != 2 or parts[1] in index:
            raise ContractError("invalid checksums")
        contained(root, parts[1])
        index[parts[1]] = parts[0]
    excluded = {"checksums.sha256"}
    if manifest.get("package_kind") == "mapping_source":
        excluded.add("manifest.yaml")
        if meta.get("pose_semantics") != "T_map_body" or meta.get("patches_frame") != "body":
            raise ContractError("invalid published mapping frame contract")
        if manifest.get("checksums") != index:
            raise ContractError("manifest/checksum mismatch")
    elif (
        meta.get("pose_semantics", {}).get("optimized_map_pose", "").split(";")[0]
        != "T_map_mapping_body"
    ):
        raise ContractError("invalid raw artifact pose semantics")
    if index != {k: v for k, v in actual.items() if k not in excluded}:
        raise ContractError("mapping hash mismatch or incomplete coverage")
    nonempty_pcd(root / "map.pcd")
    return poses(root)


def tree_sha(root):
    """Exactly V3 map_package.sha256_tree ordering/encoding."""
    import hashlib

    digest = hashlib.sha256()
    for name, value in files(root).items():
        encoded = name.encode()
        digest.update(len(encoded).to_bytes(4, "big"))
        digest.update(encoded)
        digest.update(value.encode())
    return digest.hexdigest()


def build_localization_assets(mapping, output, identity, versions, config, run=subprocess.run):
    records = validate_mapping(mapping)
    output = Path(output)
    if output.exists():
        raise ContractError("asset destination already exists")
    output.mkdir(parents=True)
    shutil.copytree(Path(mapping) / "patches", output / "patches")
    shutil.copy2(Path(mapping) / "poses.txt", output / "poses.txt")
    commands = [
        [
            "ros2",
            "run",
            "agt_global_relocalization_native",
            "build_relocalization_assets",
            "--map",
            str(Path(mapping) / "map.pcd"),
            "--output",
            str(output),
        ],
        [
            "ros2",
            "run",
            "agt_global_relocalization_native",
            "build_relocalization_candidates",
            "--map-dir",
            str(mapping),
            "--output",
            str(output),
        ],
    ]
    for cmd in commands:
        run(cmd, check=True)
    if not LOCALIZATION_FILES <= files(output).keys() or not list(
        (output / "voxelmaps_coords").glob("*.pcd")
    ):
        raise ContractError("native localization output incomplete")
    if read_yaml(output / "polar_context.yaml").get("entries", 0) < 1:
        raise ContractError("no localization candidates")
    atomic_yaml(
        output / "metadata.yaml",
        dict(
            schema_version=1,
            map=identity,
            versions=versions,
            config=config,
            source_mapping_sha256=tree_sha(mapping),
            keyframes=records,
            commands=commands,
            assets=files(output),
            mock=False,
        ),
    )


def navigation_check(root):
    root = Path(root)
    nav = read_yaml(root / "map.yaml")
    image = contained(root, nav.get("image"))
    if image.name != "map.pgm" or not image.is_file():
        raise ContractError("missing navigation image")
    if (
        not isinstance(nav.get("resolution"), (int, float))
        or not math.isfinite(nav["resolution"])
        or nav["resolution"] <= 0
    ):
        raise ContractError("invalid map resolution")
    origin = nav.get("origin")
    if (
        not isinstance(origin, list)
        or len(origin) != 3
        or not all(math.isfinite(float(v)) for v in origin)
    ):
        raise ContractError("invalid map origin")
    # V3's real map validator performs full PGM occupancy checks on activation too.
    for key in ("free_thresh", "occupied_thresh"):
        if not 0 <= float(nav.get(key, -1)) <= 1:
            raise ContractError("invalid map thresholds")
    if nav["free_thresh"] >= nav["occupied_thresh"]:
        raise ContractError("inverted map thresholds")


def seal(root, identity, versions, *, mock=False):
    root = Path(root)
    if (root / "manifest.yaml").exists():
        raise ContractError("bundle already sealed; create a new version")
    validate_mapping(root / "mapping")
    expected_mock = mock
    if bool(read_yaml(root / "mapping/metadata.yaml").get("mock")) != expected_mock:
        raise ContractError("mock mapping artifacts cannot be relabeled")
    navigation_check(root / "navigation")
    review = read_yaml(root / "navigation/review_status.yaml")
    if review.get("status") != "confirmed":
        raise ContractError("map review not confirmed")
    if Path(review.get("source_mapping_package", "")).resolve() != (root / "mapping").resolve():
        raise ContractError("review mapping source mismatch")
    loc = read_yaml(root / "localization/metadata.yaml")
    if loc.get("map") != identity or loc.get("source_mapping_sha256") != tree_sha(root / "mapping"):
        raise ContractError("localization source identity mismatch")
    if loc.get("keyframes") != poses(root / "mapping"):
        raise ContractError("localization keyframe provenance mismatch")
    if loc.get("mock") != mock:
        raise ContractError("mock assets cannot be relabeled")
    for record in poses(root / "mapping"):
        if sha(root / "localization" / record["output_patch"]) != record["sha256"]:
            raise ContractError("localization patch differs from mapping keyframe")
    if sha(root / "localization/poses.txt") != sha(root / "mapping/poses.txt"):
        raise ContractError("localization poses differ from optimized mapping poses")
    assets = {
        "localization_map": dict(path="mapping/map.pcd", sha256=sha(root / "mapping/map.pcd")),
        "navigation_map": dict(
            path="navigation/map.yaml", sha256=sha(root / "navigation/map.yaml")
        ),
        "relocalization_assets": dict(path="localization", sha256=tree_sha(root / "localization")),
    }
    atomic_yaml(
        root / "metadata.yaml",
        dict(
            schema_version=1,
            map_id=identity["map_bundle_id"],
            map_version=identity["map_version"],
            frame_id="map",
            assets=assets,
            relocalization_contract=dict(
                formal_pose_semantics="T_map_body",
                map_cloud_frame="body",
                query_frame_mode="mapping_body",
                query_frame="body",
            ),
        ),
    )
    checksums = files(root)
    manifest = dict(
        schema_version=1,
        **identity,
        map_frame="map",
        versions=versions,
        mock=mock,
        status="READY",
        checksums=checksums,
    )
    manifest["bundle_sha256"] = digest_data(manifest)
    atomic_yaml(root / "manifest.yaml", manifest)
    validate_bundle(root, allow_mock=mock)
    # Keep frozen map producers protected from accidental editor writes.
    # User-controlled versions can still be removed explicitly, never changed by runtime.
    for p in root.rglob("*"):
        if p.is_file():
            p.chmod(0o444)
    return manifest


def validate_bundle(root, *, allow_mock=False):
    root = Path(root)
    m = read_yaml(root / "manifest.yaml")
    expected = m.get("bundle_sha256")
    unsigned = {k: v for k, v in m.items() if k != "bundle_sha256"}
    if (
        expected != digest_data(unsigned)
        or m.get("schema_version") != 1
        or m.get("status") != "READY"
    ):
        raise ContractError("invalid bundle manifest")
    if m.get("mock") and not allow_mock:
        raise ContractError("mock bundle prohibited in real runtime")
    actual = files(root)
    actual.pop("manifest.yaml", None)
    if actual != m.get("checksums"):
        raise ContractError("bundle hash mismatch / missing or unlisted asset")
    identifier(m.get("map_bundle_id"))
    identifier(m.get("map_version"))
    validate_mapping(root / "mapping")
    expected_mock = bool(m.get("mock"))
    if bool(read_yaml(root / "mapping/metadata.yaml").get("mock")) != expected_mock:
        raise ContractError("mock mapping artifacts cannot be relabeled")
    navigation_check(root / "navigation")
    if not LOCALIZATION_FILES <= files(root / "localization").keys() or not list(
        (root / "localization/voxelmaps_coords").glob("*.pcd")
    ):
        raise ContractError("localization assets incomplete")
    if read_yaml(root / "navigation/review_status.yaml").get("status") != "confirmed":
        raise ContractError("review not confirmed")
    loc = read_yaml(root / "localization/metadata.yaml")
    identity = {k: m[k] for k in ("map_bundle_id", "map_version")}
    if (
        loc.get("map") != identity
        or loc.get("source_mapping_sha256") != tree_sha(root / "mapping")
        or loc.get("keyframes") != poses(root / "mapping")
    ):
        raise ContractError("invalid localization provenance")
    if loc.get("mock") != m.get("mock"):
        raise ContractError("mock provenance mismatch")
    return dict(**identity, bundle_sha256=expected)


def activate(root, active_file, *, allow_mock=False):
    root = Path(root).resolve()
    identity = validate_bundle(root, allow_mock=allow_mock)
    if not allow_mock:
        from agt_map_manager.map_package import validate_package

        info = validate_package(root / "metadata.yaml", verify_hashes=True)
        if not info.valid:
            raise ContractError(f"V3 compatibility check failed: {info.reason}")
    active_file = Path(active_file)
    generation = read_yaml(active_file).get("generation", 0) + 1 if active_file.exists() else 1
    state = dict(
        schema_version=1,
        generation=generation,
        **identity,
        map_id=identity["map_bundle_id"],
        metadata_path=str(root / "metadata.yaml"),
        package_path=str(root),
        navigation_map_yaml=str(root / "navigation/map.yaml"),
        localization_map_pcd=str(root / "mapping/map.pcd"),
        relocalization_assets_path=str(root / "localization"),
    )
    atomic_yaml(active_file, state)
    return identity
