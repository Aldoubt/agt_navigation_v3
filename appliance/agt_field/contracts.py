"""Small, strict data contracts; no ROS dependency and no physical defaults."""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import tempfile
from pathlib import Path

import yaml


class ContractError(ValueError):
    pass


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", value):
        raise ContractError("invalid identifier")
    return value


def number(value, *, nonnegative=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ContractError("finite numeric value required")
    if nonnegative and value < 0:
        raise ContractError("negative value is not permitted")
    return float(value)


def read_yaml(path):
    try:
        data = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ContractError(str(exc)) from exc
    if not isinstance(data, dict):
        raise ContractError(f"mapping document required: {path}")
    return data


def atomic_yaml(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=".agt-", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            yaml.safe_dump(data, stream, sort_keys=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def digest_data(data):
    return hashlib.sha256(
        json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def files(root):
    root = Path(root).resolve()
    result = {}
    for p in sorted(root.rglob("*")):
        if p.is_symlink():
            raise ContractError(f"symbolic link forbidden: {p}")
        if p.is_file():
            result[p.relative_to(root).as_posix()] = sha(p)
    return result


def contained(root, relative):
    root = Path(root).resolve()
    if (
        not isinstance(relative, str)
        or Path(relative).is_absolute()
        or ".." in Path(relative).parts
    ):
        raise ContractError("unsafe relative path")
    p = root / relative
    if not p.resolve().is_relative_to(root) or p.is_symlink():
        raise ContractError("asset escapes root")
    return p


def route(data, binding=None):
    if data.get("schema_version") != 1:
        raise ContractError("route schema_version must be 1")
    identifier(data.get("route_id"))
    identity = data.get("map", {})
    for key in ("map_bundle_id", "map_version", "bundle_sha256"):
        if not isinstance(identity.get(key), str) or not identity[key]:
            raise ContractError(f"route missing {key}")
    if binding is not None and identity != binding:
        raise ContractError("route map binding mismatch")
    points = data.get("waypoints")
    if not isinstance(points, list) or not points:
        raise ContractError("route requires waypoints")
    seen = set()
    normalized = []
    for i, p in enumerate(points):
        name = identifier(p.get("id"))
        if name in seen or p.get("sequence", i) != i:
            raise ContractError("duplicate waypoint or invalid sequence")
        seen.add(name)
        if p.get("action", "wait") != "wait":
            raise ContractError("unsupported waypoint action")
        normalized.append(
            dict(
                id=name,
                x=number(p.get("x")),
                y=number(p.get("y")),
                yaw=number(p.get("yaw")),
                sequence=i,
                dwell_seconds=number(p.get("dwell_seconds", 0), nonnegative=True),
                action="wait",
            )
        )
    return dict(
        schema_version=1, route_id=data["route_id"], map=dict(identity), waypoints=normalized
    )
