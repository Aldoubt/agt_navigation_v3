#!/usr/bin/env python3
"""Reproducible SOURCE-ONLY field workspace manager; never enables motion.

Commands:
  init        clone the three pinned repositories without touching ROS
  doctor      report git identity, duplicate ROS packages, missing prerequisites
  build-plan  print isolated package selection (does not build or launch)
  line        create a map-frame test line as JSON (does not execute it)

External FAST-LIO2, Livox/ROS, GNSS and Nav2 source dependencies are not
silently installed: verify the pinned manifests before any live deployment.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from collections import defaultdict
from pathlib import Path
from xml.etree import ElementTree as ET

HERE = Path(__file__).resolve().parent
LOCK = HERE / "workspace.lock.json"
SKIP_PARTS = {
    ".git", "build", "install", "log", ".venv", "venv", "__pycache__",
    "integrations", "test_results", "node_modules", ".pytest_cache"
}
# An explicit SOURCE selection avoids the duplicate package ABI witnessed on
# 2026-10-22 (legacy three-field vs FW-mini four-field CtrlCmd).
NAV_PACKAGES = (
    "agt_navigation_interfaces", "agt_robot_interfaces",
    "agt_livox_tools", "agt_navigation_runtime",
    "agt_global_relocalization", "agt_localization_manager",
    "agt_navigation_capability", "agt_navigation_supervisor",
    "agt_rviz_patrol", "agt_rviz_route_tools", "agt_operator_console",
    "fwmini_bringup", "fwmini_description", "fwmini_chassis_adapter",
    "fwmini_multi_nav_bringup", "yhs_can_interfaces", "yhs_can_control",
    "agt_fastlio_adapter",
)
MAPPING_PACKAGES = (
    "agt_mapping_interfaces", "agt_mapping_frontend_api",
    "agt_mapping_backend_api", "agt_mapping_core", "agt_spatial_map_core",
    "agt_fastlio_backend", "agt_mapping_bringup", "agt_mapping_artifacts",
    "agt_mapping_exporter", "agt_pcd2grid_exporter", "agt_map_studio",
    "agt_map_editor", "agt_map_refinement_core", "agt_mid360_adapter",
)


def lock_data():
    data = json.loads(LOCK.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError("unsupported workspace.lock.json schema version")
    names = [r["name"] for r in data["repositories"]]
    if len(names) != len(set(names)):
        raise ValueError("repository names must be unique")
    for repo in data["repositories"]:
        if len(repo["ref"]) != 40 or not all(c in "0123456789abcdef" for c in repo["ref"]):
            raise ValueError(f"{repo['name']} must be pinned to a 40-byte hex SHA")
    return data


def run_git(*args):
    return subprocess.check_output(["git", *map(str, args)],
                                   text=True, stderr=subprocess.STDOUT).strip()


def workspace(path):
    return Path(path).expanduser().resolve()


def repo_root(root, name):
    return root / "src" / name


def init_workspace(root, dry_run=False):
    """Never reset an existing checkout; refuse mismatched commits."""
    data = lock_data()
    for repo in data["repositories"]:
        target = repo_root(root, repo["name"])
        if target.exists():
            if not (target / ".git").exists():
                raise RuntimeError(f"{target} exists and is not a normal git clone")
            got_url = run_git("-C", target, "remote", "get-url", "origin")
            if got_url != repo["url"]:
                raise RuntimeError(f"{target} origin mismatch: {got_url}")
            got = run_git("-C", target, "rev-parse", "HEAD")
            if got != repo["ref"]:
                raise RuntimeError(
                    f"{target} HEAD={got[:12]} not lock={repo['ref'][:12]}. "
                    "Do not reset your work; use a clean workspace.")
            print(f"OK PINNED {repo['name']} {got[:12]}")
            continue
        print(f"{'WOULD CLONE' if dry_run else 'CLONE'} {repo['name']} @{repo['ref'][:12]}")
        if dry_run:
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        run_git("clone", "--no-checkout", repo["url"], target)
        try:
            run_git("-C", target, "fetch", "origin", repo["ref"])
            run_git("-C", target, "checkout", "--detach", repo["ref"])
        except Exception:
            print(f"INCOMPLETE CHECKOUT: inspect {target}; no reset attempted", file=sys.stderr)
            raise
    print("SOURCE READY only. Run 'doctor' before installing ROS dependencies.")


def packages_in(root, data=None):
    """Enumerate package.xml without descending into a mirrored integration tree."""
    data = data or lock_data()
    result = defaultdict(list)
    for repo in data["repositories"]:
        top = repo_root(root, repo["name"])
        if not top.is_dir():
            continue
        for folder, dirs, filenames in os.walk(top):
            dirs[:] = [x for x in dirs if x not in SKIP_PARTS]
            if "package.xml" in filenames:
                path = Path(folder) / "package.xml"
                try:
                    name = ET.parse(path).getroot().findtext("name")
                except ET.ParseError as exc:
                    raise ValueError(f"invalid package.xml at {path}: {exc}") from exc
                if not name:
                    raise ValueError(f"ROS package missing <name>: {path}")
                result[name].append((repo["name"], path.parent))
                dirs.clear()
    return result


def resolve_packages(root, required):
    data = lock_data()
    found = packages_in(root, data)
    owners = data["preferred_package_owner"]
    selected = []
    for name in required:
        candidates = found.get(name, [])
        if not candidates:
            raise RuntimeError(f"Missing source package {name}; cannot create build plan")
        wanted = owners.get(name)
        if wanted:
            candidates = [x for x in candidates if x[0] == wanted]
            if not candidates:
                raise RuntimeError(f"Missing required owner {wanted} for {name}")
        if len(candidates) != 1:
            raise RuntimeError(f"Ambiguous {name}: {candidates}")
        selected.append(candidates[0][1])
    if len(set(selected)) != len(selected):
        raise RuntimeError("build plan contains repeated source directories")
    return selected


def build_plan(root, role):
    selected = resolve_packages(root, MAPPING_PACKAGES if role == "mapping" else NAV_PACKAGES)
    output = root / "build" / role
    install = root / "install" / role
    args = [
        "colcon", "build", "--symlink-install", "--executor", "sequential",
        "--base-paths", *map(str, selected),
        "--build-base", str(output),
        "--install-base", str(install),
        "--log-base", str(root / "log" / role),
    ]
    import shlex
    print("NOT EXECUTED — inspect and install all pinned external dependencies first.")
    print("No generic 'colcon build' over the whole workspace: duplicate package ABI risk.")
    print(" ".join(shlex.quote(part) for part in args))


def doctor(root):
    data = lock_data()
    warnings = 0
    for repo in data["repositories"]:
        top = repo_root(root, repo["name"])
        if not (top / ".git").is_dir():
            print(f"MISSING {top}; run init")
            warnings += 1
            continue
        got = run_git("-C", top, "rev-parse", "HEAD")
        print(f"{'PINNED' if got == repo['ref'] else 'DIFFERENT'} {repo['name']} {got[:12]}")
        if got != repo["ref"]:
            warnings += 1
        dirty = run_git("-C", top, "status", "--porcelain")
        if dirty:
            print(f"  MODIFIED: {repo['name']} has local changes (not reset)")
    found = packages_in(root, data)
    for name, entries in sorted(found.items()):
        if len(entries) > 1:
            chosen = data["preferred_package_owner"].get(name)
            owner_names = [r for r, _ in entries]
            if not chosen or chosen not in owner_names:
                print(f"UNRESOLVED DUPLICATE {name}: {owner_names}")
                warnings += 1
            else:
                print(f"RESOLVED DUPLICATE {name} => {chosen} (use explicit --base-paths)")
    for tool in ("ros2", "colcon", "vcs"):
        if shutil.which(tool) is None:
            print(f"NOT INSTALLED/NOT SOURCED: {tool}")
            warnings += 1
    print("EXTERNAL DEPENDENCIES: not installed by init. Verify mapping .repos / V3 .repos.")
    print("SENSOR, CAN, TF, MAP HASH, CLOCK, GNSS FACTORS: not field-validated.")
    print(f"DOCTOR WARNINGS={warnings}")
    return 1 if warnings else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--workspace", default="~/agt_field_ws")
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("init", help="clone pinned source (no ROS launch)")
    p.add_argument("--dry-run", action="store_true")
    sub.add_parser("doctor", help="read-only workspace and duplicate package checks")
    p = sub.add_parser("build-plan", help="print, but do not execute, isolated build command")
    p.add_argument("role", choices=("mapping", "navigation"))
    p = sub.add_parser("line", help="generate geometry, never command actuators")
    p.add_argument("--distance", type=float, required=True)
    p.add_argument("--x", type=float, required=True)
    p.add_argument("--y", type=float, required=True)
    p.add_argument("--yaw-deg", type=float, required=True)
    p.add_argument("--spacing", type=float, default=0.05)
    args = parser.parse_args(argv)
    root = workspace(args.workspace)
    try:
        if args.command == "init":
            init_workspace(root, args.dry_run)
        elif args.command == "doctor":
            return doctor(root)
        elif args.command == "build-plan":
            build_plan(root, args.role)
        elif args.command == "line":
            from relative_line import generate_line
            import math
            poses = generate_line(args.x, args.y, math.radians(args.yaw_deg),
                                  args.distance, args.spacing)
            print(json.dumps({
                "type": "DRY_RUN_GEOMETRY_ONLY", "frame": "map",
                "distance_m": args.distance, "poses": poses,
                "operator_confirmation_required": True,
            }, ensure_ascii=False))
        else:
            parser.error(f"unsupported command: {args.command}")
        return 0
    except (ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
