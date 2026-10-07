"""Pinned third-party source preparation. Refuses dirty or mismatched checkouts."""

import argparse
import hashlib
from pathlib import Path
import subprocess
import yaml


def git(path, *args):
    return subprocess.check_output(["git", "-C", str(path), *args], text=True).strip()


def checkout(url, commit, path):
    if not path.exists():
        subprocess.run(["git", "clone", "--filter=blob:none", url, str(path)], check=True)
    if git(path, "status", "--porcelain"):
        raise RuntimeError("refusing dirty third-party checkout: " + str(path))
    if git(path, "rev-parse", "HEAD") != commit:
        subprocess.run(["git", "-C", str(path), "fetch", "origin", commit], check=True)
        subprocess.run(["git", "-C", str(path), "checkout", "--detach", commit], check=True)
    if git(path, "rev-parse", "HEAD") != commit:
        raise RuntimeError("source pin mismatch")


parser = argparse.ArgumentParser()
parser.add_argument("--root", required=True)
parser.add_argument("--source-root", required=True)
args = parser.parse_args()
root = Path(args.root).resolve()
sources = Path(args.source_root).resolve()
sources.mkdir(parents=True, exist_ok=True)
lock = yaml.safe_load((root / "appliance/repos.lock.yaml").read_text())
for name in ["mapping", "hmi"]:
    spec = lock[name]
    path = sources / name
    if name == "hmi" and path.exists() and git(path, "status", "--porcelain"):
        if git(path, "rev-parse", "HEAD") != spec["commit"]:
            raise RuntimeError("HMI upstream commit changed")
        patch = root / "appliance/third_party/qt-field.patch"
        existing = subprocess.check_output(["git", "-C", str(path), "diff", "--binary"])
        if hashlib.sha256(existing).hexdigest() != spec["patch_sha256"]:
            raise RuntimeError("HMI patch checkout changed")
        continue
    checkout(spec["url"], spec["commit"], path)
    if name == "hmi":
        patch = root / "appliance/third_party/qt-field.patch"
        if hashlib.sha256(patch.read_bytes()).hexdigest() != spec["patch_sha256"]:
            raise RuntimeError("patch hash mismatch")
        subprocess.run(["git", "-C", str(path), "apply", str(patch)], check=True)
        subprocess.run(["git", "-C", str(path), "add", "-N", "."], check=True)
