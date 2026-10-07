"""Exercise independent fork pins and preserve dirty legacy checkouts using local Git."""

import hashlib
from pathlib import Path
import subprocess
import sys
import yaml

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/prepare_sources.py"


def git(path, *args):
    return subprocess.check_output(["git", "-C", str(path), *args], text=True).strip()


def repository(path):
    path.mkdir()
    git(path, "init", "-q")
    git(path, "config", "user.name", "Fixture")
    git(path, "config", "user.email", "fixture@example.invalid")
    (path / "source.txt").write_text("upstream\n")
    git(path, "add", ".")
    git(path, "commit", "-qm", "upstream")
    base = git(path, "rev-parse", "HEAD")
    (path / "source.txt").write_text("field fork\n")
    git(path, "commit", "-qam", "field")
    final = git(path, "rev-parse", "HEAD")
    patch = subprocess.check_output(["git", "-C", str(path), "diff", "--binary", base, final])
    return base, final, patch


def setup(tmp_path):
    upstream = tmp_path / "remote"
    base, final, patch = repository(upstream)
    root = tmp_path / "integration"
    (root / "appliance").mkdir(parents=True)
    lock = dict(
        mapping=dict(url=upstream.as_uri(), commit=base),
        hmi=dict(
            url=upstream.as_uri(),
            commit=final,
            legacy_patch=dict(commit=base, patch_sha256=hashlib.sha256(patch).hexdigest()),
        ),
    )
    (root / "appliance/repos.lock.yaml").write_text(yaml.safe_dump(lock))
    sources = tmp_path / "sources"
    return upstream, base, final, patch, root, sources


def prepare(root, sources):
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--root", str(root), "--source-root", str(sources)],
        capture_output=True,
        text=True,
    )


def test_fork_pin_repeated_and_dirty_refused(tmp_path):
    _, _, final, _, root, sources = setup(tmp_path)
    assert prepare(root, sources).returncode == 0
    assert git(sources / "hmi", "rev-parse", "HEAD") == final
    assert prepare(root, sources).returncode == 0
    (sources / "hmi/source.txt").write_text("operator change\n")
    result = prepare(root, sources)
    assert result.returncode != 0 and "refusing dirty" in result.stderr
    assert (sources / "hmi/source.txt").read_text() == "operator change\n"


def test_matching_legacy_patch_preserved_on_fork_migration(tmp_path):
    remote, base, final, patch, root, sources = setup(tmp_path)
    sources.mkdir()
    subprocess.run(["git", "clone", "-q", str(remote), str(sources / "hmi")], check=True)
    git(sources / "hmi", "checkout", "-q", "--detach", base)
    subprocess.run(["git", "-C", str(sources / "hmi"), "apply", "-"], input=patch, check=True)
    assert prepare(root, sources).returncode == 0
    backup = sources / ("hmi.upstream-patch-" + base[:12])
    assert (backup / "source.txt").read_text() == "field fork\n"
    assert git(backup, "rev-parse", "HEAD") == base
    assert git(sources / "hmi", "rev-parse", "HEAD") == final
    assert prepare(root, sources).returncode == 0


def test_changed_legacy_checkout_never_moved_or_overwritten(tmp_path):
    remote, base, _, _, root, sources = setup(tmp_path)
    sources.mkdir()
    subprocess.run(["git", "clone", "-q", str(remote), str(sources / "hmi")], check=True)
    git(sources / "hmi", "checkout", "-q", "--detach", base)
    (sources / "hmi/source.txt").write_text("my uncommitted work\n")
    result = prepare(root, sources)
    assert result.returncode != 0 and "refusing changed legacy" in result.stderr
    assert (sources / "hmi/source.txt").read_text() == "my uncommitted work\n"
    assert not list(sources.glob("hmi.upstream-patch-*"))
