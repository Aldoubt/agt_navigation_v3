# ruff: noqa: E402
"""Real native asset builder with explicitly synthetic test points, no robot claim."""

import pytest

pytest.importorskip("rclpy")

from agt_field.bundle import build_localization_assets
from agt_field.contracts import atomic_yaml, files, read_yaml
from agt_field.mock import mapping_fixture


def test_native_assets(tmp_path):
    mapping = mapping_fixture(tmp_path / "mapping")
    points = [(x * 0.2, y * 0.2, (x % 4) * 0.3) for x in range(-15, 15) for y in range(-15, 15)]
    header = (
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n"
    )
    pcd = header + "".join(f"{x} {y} {z}\n" for x, y, z in points)
    for name in ["map.pcd", "patches/0.pcd"]:
        (mapping / name).write_text(pcd)
    index = files(mapping)
    index.pop("manifest.yaml")
    index.pop("checksums.sha256")
    (mapping / "checksums.sha256").write_text("".join(f"{v}  {k}\n" for k, v in index.items()))
    atomic_yaml(mapping / "manifest.yaml", dict(package_kind="mapping_source", checksums=index))
    output = tmp_path / "localization"
    build_localization_assets(
        mapping,
        output,
        dict(map_bundle_id="synthetic_native_test", map_version="1"),
        dict(localization_builder_commit="test"),
        dict(test_only=True),
    )
    assert read_yaml(output / "polar_context.yaml")["entries"] == 1
    assert (output / "polar_context.db").stat().st_size > 100
    assert list((output / "voxelmaps_coords").glob("*.pcd"))
    assert (output / "patches/0.pcd").read_bytes() == (mapping / "patches/0.pcd").read_bytes()
    assert read_yaml(output / "metadata.yaml")["keyframes"][0]["source_keyframe_id"] == "0.pcd"
