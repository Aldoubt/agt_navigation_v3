"""Pinned third-party source preparation. Refuses dirty or mismatched checkouts."""

import argparse
import hashlib
from pathlib import Path
import subprocess
import yaml


def git(path, *args):
    return subprocess.check_output(["git", "-C", str(path), *args], text=True).strip()


def head(path):
    result = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "--verify", "HEAD"], capture_output=True, text=True
    )
    return result.stdout.strip() if result.returncode == 0 else None


def checkout(url, commit, path):
    if not path.exists():
        path.mkdir(parents=True)
        subprocess.run(["git", "init", str(path)], check=True)
        subprocess.run(["git", "-C", str(path), "remote", "add", "origin", url], check=True)
    if git(path, "status", "--porcelain"):
        raise RuntimeError("refusing dirty third-party checkout: " + str(path))
    if head(path) != commit:
        subprocess.run(["git", "-C", str(path), "fetch", "--depth=1", url, commit], check=True)
        subprocess.run(["git", "-C", str(path), "checkout", "--detach", commit], check=True)
    if head(path) != commit:
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
    # A previously generated, exactly matching upstream patch checkout is preserved,
    # not reset/cleaned, when switching to the independently pinned writable fork.
    legacy = spec.get("legacy_patch") if name == "hmi" and not spec.get("patch") else None
    if legacy and path.exists() and head(path) == legacy["commit"]:
        status = git(path, "status", "--porcelain")
        existing = subprocess.check_output(["git", "-C", str(path), "diff", "--binary"])
        if status:
            if (
                any(line.startswith("?? ") for line in status.splitlines())
                or hashlib.sha256(existing).hexdigest() != legacy["patch_sha256"]
            ):
                raise RuntimeError("refusing changed legacy Qt checkout: " + str(path))
            backup = sources / ("hmi.upstream-patch-" + legacy["commit"][:12])
            suffix = 0
            while backup.exists():
                suffix += 1
                backup = sources / (
                    "hmi.upstream-patch-" + legacy["commit"][:12] + "-" + str(suffix)
                )
            path.rename(backup)
            print("Preserved legacy Qt checkout: " + str(backup))
    if name == "hmi" and spec.get("patch") and path.exists() and git(path, "status", "--porcelain"):
        if head(path) != spec["commit"]:
            raise RuntimeError("HMI upstream commit changed")
        patch = root / "appliance" / spec["patch"]
        existing = subprocess.check_output(["git", "-C", str(path), "diff", "--binary"])
        if hashlib.sha256(existing).hexdigest() != spec["patch_sha256"]:
            raise RuntimeError("HMI patch checkout changed")
        continue
    checkout(spec["url"], spec["commit"], path)
    if name == "hmi" and spec.get("patch"):
        patch = root / "appliance" / spec["patch"]
        if hashlib.sha256(patch.read_bytes()).hexdigest() != spec["patch_sha256"]:
            raise RuntimeError("patch hash mismatch")
        subprocess.run(["git", "-C", str(path), "apply", str(patch)], check=True)
        subprocess.run(["git", "-C", str(path), "add", "-N", "."], check=True)
