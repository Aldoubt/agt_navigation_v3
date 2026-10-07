"""Tiny synthetic fixtures explicitly confined to mock acceptance.

They are not sensor data, PGO evidence, or native BBS/descriptor databases.
"""

from pathlib import Path

from .bundle import seal, tree_sha, validate_mapping
from .contracts import atomic_yaml, files

PCD = """VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 3
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 3
DATA ascii
0 0 0
1 0 0
0 1 0
"""


def mapping_fixture(root):
    root = Path(root)
    (root / "patches").mkdir(parents=True)
    (root / "map.pcd").write_text(PCD)
    (root / "patches/0.pcd").write_text(PCD)
    (root / "poses.txt").write_text("0.pcd 0 0 0 1 0 0 0\n")
    (root / "poses_timed.txt").write_text("0.pcd 1 0 0 0 1 0 0 0\n")
    atomic_yaml(root / "calibration.yaml", dict(status="MOCK_ONLY"))
    atomic_yaml(
        root / "metadata.yaml",
        dict(
            backend="PGO",
            backend_status=dict(optimized=True),
            pose_semantics="T_map_body",
            patches_frame="body",
            mock=True,
        ),
    )
    index = files(root)
    (root / "checksums.sha256").write_text("".join(f"{v}  {k}\n" for k, v in index.items()))
    atomic_yaml(
        root / "manifest.yaml",
        dict(schema_version=1, package_kind="mapping_source", checksums=index),
    )
    return root


def localization_fixture(root, mapping, identity):
    import shutil

    root = Path(root)
    (root / "voxelmaps_coords").mkdir(parents=True)
    for name in [
        "polar_context.db",
        "polar_context.yaml",
        "relocalization_assets.yaml",
        "voxelmaps_coords/voxel_params.txt",
    ]:
        (root / name).write_text("MOCK_ONLY\n")
    (root / "global_map_downsampled.pcd").write_text(PCD)
    (root / "voxelmaps_coords/0.pcd").write_text(PCD)
    shutil.copytree(Path(mapping) / "patches", root / "patches")
    shutil.copy2(Path(mapping) / "poses.txt", root / "poses.txt")
    atomic_yaml(
        root / "metadata.yaml",
        dict(
            map=identity,
            source_mapping_sha256=tree_sha(mapping),
            keyframes=validate_mapping(mapping),
            mock=True,
            assets=files(root),
        ),
    )


def navigation_fixture(root, mapping, *, confirmed=True):
    root = Path(root)
    root.mkdir(parents=True)
    (root / "map.pgm").write_bytes(b"P5\n3 3\n255\n" + bytes([254] * 9))
    atomic_yaml(
        root / "map.yaml",
        dict(
            image="map.pgm",
            resolution=0.1,
            origin=[0, 0, 0],
            negate=0,
            occupied_thresh=0.65,
            free_thresh=0.25,
        ),
    )
    atomic_yaml(root / "keepout_zones.yaml", dict(zones=[]))
    atomic_yaml(
        root / "review_status.yaml",
        dict(
            status="confirmed" if confirmed else "pending",
            source_mapping_package=str(Path(mapping).resolve()),
        ),
    )


def bundle_fixture(root, *, confirmed=True):
    root = Path(root)
    identity = dict(map_bundle_id="mock_field", map_version="1")
    mapping_fixture(root / "mapping")
    localization_fixture(root / "localization", root / "mapping", identity)
    navigation_fixture(root / "navigation", root / "mapping", confirmed=confirmed)
    if confirmed:
        seal(
            root,
            identity,
            dict(mapping_commit="mock", localization_builder_commit="mock"),
            mock=True,
        )
    return root
