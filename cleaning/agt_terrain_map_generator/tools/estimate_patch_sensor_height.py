#!/usr/bin/env python3
"""Estimate Patchwork++ sensor_height from frozen body-frame mapping patches.

Evidence-only helper for MQ3. It does not modify mapping assets. The estimate
is the negative gravity-level z of the dominant near-field low surface,
aggregated robustly across patches. Physical measurement remains the final
calibration check.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import numpy as np


def pcd_header(path: Path):
    meta = {}
    offset = 0
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"{path}: PCD DATA line not found")
            offset += len(line)
            text = line.decode("ascii", errors="strict").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            meta[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                break
    return meta, offset


def pcd_dtype(meta):
    fields = meta["FIELDS"]
    sizes = [int(v) for v in meta["SIZE"]]
    types = meta["TYPE"]
    counts = [int(v) for v in meta.get("COUNT", ["1"] * len(fields))]
    table = {
        ("F", 4): "<f4", ("F", 8): "<f8",
        ("I", 1): "<i1", ("I", 2): "<i2",
        ("I", 4): "<i4", ("I", 8): "<i8",
        ("U", 1): "<u1", ("U", 2): "<u2",
        ("U", 4): "<u4", ("U", 8): "<u8",
    }
    result = []
    for name, size, kind, count in zip(fields, sizes, types, counts):
        if count != 1:
            raise ValueError(f"unsupported COUNT={count} for {name}")
        key = (kind.upper(), size)
        if key not in table:
            raise ValueError(f"unsupported PCD field type {key} for {name}")
        result.append((name, table[key]))
    return np.dtype(result)


def load_xyz(path: Path):
    meta, offset = pcd_header(path)
    fields = meta.get("FIELDS", [])
    for required in ("x", "y", "z"):
        if required not in fields:
            raise ValueError(f"{path}: missing {required} field")
    mode = meta["DATA"][0].lower()
    points = int((meta.get("POINTS") or meta.get("WIDTH") or ["0"])[0])
    if mode == "binary_compressed":
        raise ValueError(f"{path}: binary_compressed is not supported")
    if mode == "binary":
        dtype = pcd_dtype(meta)
        with path.open("rb") as stream:
            stream.seek(offset)
            data = np.fromfile(
                stream, dtype=dtype, count=points if points > 0 else -1)
        xyz = np.column_stack(
            (data["x"], data["y"], data["z"])).astype(np.float64, copy=False)
    elif mode == "ascii":
        with path.open("r", encoding="ascii") as stream:
            while True:
                line = stream.readline()
                if not line:
                    raise ValueError(f"{path}: DATA line not found")
                if line.strip().upper().startswith("DATA "):
                    break
            data = np.loadtxt(stream, dtype=np.float64)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        indices = [fields.index(name) for name in ("x", "y", "z")]
        xyz = data[:, indices]
    else:
        raise ValueError(f"{path}: unsupported DATA mode {mode}")
    return xyz[np.isfinite(xyz).all(axis=1)]


def quaternion_rotation(qw, qx, qy, qz):
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm <= 1.0e-12:
        raise ValueError("zero quaternion")
    w, x, y, z = qw / norm, qx / norm, qy / norm, qz / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def read_poses(path: Path):
    records = []
    for line_number, raw in enumerate(
            path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) != 8:
            raise ValueError(
                f"{path}:{line_number}: expected patch tx ty tz qw qx qy qz")
        records.append((
            parts[0],
            tuple(float(value) for value in parts[1:]),
        ))
    if not records:
        raise ValueError(f"{path}: no pose records")
    return records


def estimate_patch(
        xyz, pose, min_range, max_range, z_min, z_max,
        bin_size, min_bin_points):
    _, _, _, qw, qx, qy, qz = pose
    rotation = quaternion_rotation(qw, qx, qy, qz)
    leveled = xyz @ rotation.T

    radius = np.hypot(leveled[:, 0], leveled[:, 1])
    select = (
        (radius >= min_range) & (radius <= max_range)
        & (leveled[:, 2] >= z_min) & (leveled[:, 2] <= z_max))
    z = leveled[select, 2]
    if z.size < min_bin_points:
        return None

    bins = np.arange(z_min, z_max + bin_size, bin_size)
    histogram, edges = np.histogram(z, bins=bins)
    peak = int(np.argmax(histogram))
    if int(histogram[peak]) < min_bin_points:
        return None

    center = 0.5 * (edges[peak] + edges[peak + 1])
    half_window = max(0.075, 1.5 * bin_size)
    support = z[np.abs(z - center) <= half_window]
    if support.size < min_bin_points:
        return None

    ground_z = float(np.median(support))
    return {
        "height": -ground_z,
        "ground_z": ground_z,
        "selected_points": int(z.size),
        "support_points": int(support.size),
        "peak_points": int(histogram[peak]),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-directory", required=True, type=Path)
    parser.add_argument("--poses-txt", type=Path)
    parser.add_argument("--patches-directory", type=Path)
    parser.add_argument("--patch-step", type=int, default=5)
    parser.add_argument("--min-range-m", type=float, default=1.0)
    parser.add_argument("--max-range-m", type=float, default=4.0)
    parser.add_argument("--z-min-m", type=float, default=-2.0)
    parser.add_argument("--z-max-m", type=float, default=0.2)
    parser.add_argument("--bin-size-m", type=float, default=0.05)
    parser.add_argument("--min-bin-points", type=int, default=20)
    args = parser.parse_args()

    if args.patch_step < 1:
        parser.error("--patch-step must be >= 1")
    if not (0.0 <= args.min_range_m < args.max_range_m):
        parser.error("invalid range limits")
    if not (args.z_min_m < args.z_max_m):
        parser.error("invalid z limits")
    if args.bin_size_m <= 0.0 or args.min_bin_points < 1:
        parser.error("invalid histogram parameters")

    root = args.map_directory.expanduser().resolve()
    poses = (args.poses_txt or (root / "poses.txt")).expanduser().resolve()
    patches = (
        args.patches_directory or (root / "patches")).expanduser().resolve()

    records = read_poses(poses)
    estimates = []
    failures = 0
    for index, (name, pose) in enumerate(records):
        if index % args.patch_step != 0:
            continue
        path = patches / name
        if not path.is_file():
            raise FileNotFoundError(path)
        try:
            result = estimate_patch(
                load_xyz(path), pose,
                args.min_range_m, args.max_range_m,
                args.z_min_m, args.z_max_m,
                args.bin_size_m, args.min_bin_points)
        except Exception as exc:
            print(f"WARN {name}: {exc}", file=sys.stderr)
            failures += 1
            continue
        if result is None:
            failures += 1
            continue
        estimates.append((name, result))

    if not estimates:
        raise RuntimeError("no valid patch height estimates")

    values = np.array(
        [result["height"] for _, result in estimates], dtype=np.float64)
    q10, q25, q50, q75, q90 = np.percentile(
        values, [10, 25, 50, 75, 90])
    mad = float(np.median(np.abs(values - q50)))

    print("===== MQ3 PATCH SENSOR HEIGHT ESTIMATE =====")
    print(f"pose_records: {len(records)}")
    print(f"patch_step: {args.patch_step}")
    print(f"valid_estimates: {len(estimates)}")
    print(f"failed_or_weak: {failures}")
    print(f"p10_m: {q10:.4f}")
    print(f"p25_m: {q25:.4f}")
    print(f"median_m: {q50:.4f}")
    print(f"p75_m: {q75:.4f}")
    print(f"p90_m: {q90:.4f}")
    print(f"iqr_m: {(q75 - q25):.4f}")
    print(f"mad_m: {mad:.4f}")
    print()
    print("candidate_sensor_height_m: %.4f" % q50)

    if q50 <= 0.0:
        print("STATUS: REVIEW (non-positive estimate)")
        return 1
    if (q75 - q25) > 0.15:
        print("STATUS: REVIEW (IQR > 0.15 m; inspect terrain/patch frame)")
        return 1
    print("STATUS: ESTIMATE_STABLE")
    print(
        "NOTE: verify candidate_sensor_height_m against the physical "
        "body/IMU patch-origin height before Patchwork++ acceptance.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
