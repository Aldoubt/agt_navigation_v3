"""Immutable backend jobs and timestamp-bracketed LiDAR deskew primitives.

This module has no ROS dependency. Workers receive no live subscriptions and
never publish; acceptance belongs to the executor and localization manager.
"""
from __future__ import annotations

import bisect
import hashlib
import math
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class QueryJob:
    job_id: str
    request_epoch: int
    odom_epoch: int
    reference_ns: int
    started_ns: int
    deadline_monotonic: float
    rows: tuple
    clouds: tuple
    transforms: tuple
    odom_samples: tuple
    query_options: tuple
    mode: str
    query_frame: str
    body_to_base: tuple
    map_path: str
    assets_path: str
    map_id: str
    map_version: str
    map_generation: int
    expected_hash: str
    map_stat: tuple
    asset_stats: tuple
    command_template: str
    timeout_sec: float
    command_values: tuple
    work_dir: str
    capture_dir: str
    moving: bool


@dataclass(frozen=True)
class CloudSnapshot:
    """Own the byte buffer and field definitions, independent of live ROS data."""
    stamp_ns: int
    frame_id: str
    height: int
    width: int
    fields: tuple
    is_bigendian: bool
    point_step: int
    row_step: int
    data: bytes
    is_dense: bool

    @classmethod
    def freeze(cls, cloud):
        return cls(int(cloud.header.stamp.sec) * 1_000_000_000 + int(cloud.header.stamp.nanosec),
                   cloud.header.frame_id, cloud.height, cloud.width,
                   tuple((f.name, f.offset, f.datatype, f.count) for f in cloud.fields),
                   cloud.is_bigendian, cloud.point_step, cloud.row_step, bytes(cloud.data), cloud.is_dense)

    def message(self):
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import Header
        from builtin_interfaces.msg import Time
        return PointCloud2(header=Header(stamp=Time(sec=self.stamp_ns // 1_000_000_000,
                                                    nanosec=self.stamp_ns % 1_000_000_000),
                                        frame_id=self.frame_id),
                           height=self.height, width=self.width,
                           fields=[PointField(name=n, offset=o, datatype=d, count=c) for n,o,d,c in self.fields],
                           is_bigendian=self.is_bigendian, point_step=self.point_step,
                           row_step=self.row_step, data=self.data, is_dense=self.is_dense)


def asset_identity(directory):
    """Exact file set, including additions/removals, for a backend snapshot."""
    return tuple((str(path), file_identity(path)) for path in sorted(Path(directory).rglob('*'))
                 if path.is_file()) if directory else ()


def file_identity(path):
    stat = Path(path).stat()
    return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)


def hash_file(path, cancelled=None, deadline_monotonic=None):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            if cancelled is not None and cancelled.is_set():
                raise RuntimeError('query cancelled while hashing map')
            if deadline_monotonic is not None and time.monotonic() > deadline_monotonic:
                raise RuntimeError('query deadline exceeded while hashing map')
            h.update(chunk)
    return h.hexdigest()


def run_command(command, timeout_sec, cancelled: threading.Event):
    """Bound one process group; cancellation also terminates native children."""
    import os
    import signal
    if cancelled.is_set():
        raise RuntimeError('query cancelled before process creation')
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, start_new_session=True)
    deadline = time.monotonic() + timeout_sec
    try:
        while True:
            if cancelled.is_set():
                raise RuntimeError('query cancelled')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError('backend deadline exceeded')
            try:
                out, err = proc.communicate(timeout=min(0.1, remaining))
                return subprocess.CompletedProcess(command, proc.returncode, out, err)
            except subprocess.TimeoutExpired:
                pass
    finally:
        # A ros2 launcher may have already exited while a native descendant
        # still owns the pipes. Terminate the whole original group on failures.
        if proc.poll() is None or cancelled.is_set() or time.monotonic() >= deadline:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=0.3)
                try:
                    os.killpg(proc.pid, 0)
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait()


def interpolate_pose(samples, stamp_ns, max_gap_ns, normalize, compose, inverse):
    """Return a bracketed pose; no extrapolation or nearest-sample deskew."""
    times = [sample[0] for sample in samples]
    i = bisect.bisect_left(times, stamp_ns)
    if i < len(samples) and times[i] == stamp_ns:
        return samples[i][1]
    if i == 0 or i == len(samples):
        raise RuntimeError('point timestamp is not bracketed by odometry')
    ta, a = samples[i - 1]
    tb, b = samples[i]
    if tb <= ta or tb - ta > max_gap_ns:
        raise RuntimeError('odometry gap exceeds deskew bound')
    alpha = (stamp_ns - ta) / (tb - ta)
    qa = normalize([a[k] for k in ('qx', 'qy', 'qz', 'qw')], 'odom quaternion')
    qb = normalize([b[k] for k in ('qx', 'qy', 'qz', 'qw')], 'odom quaternion')
    dot = sum(x * y for x, y in zip(qa, qb))
    if dot < 0.0:
        qb = tuple(-v for v in qb)
        dot = -dot
    if dot > 0.9995:
        q = normalize([x + alpha * (y - x) for x, y in zip(qa, qb)], 'interpolated quaternion')
    else:
        theta = math.acos(min(1.0, max(-1.0, dot)))
        q = tuple((math.sin((1.0 - alpha) * theta) * x + math.sin(alpha * theta) * y)
                  / math.sin(theta) for x, y in zip(qa, qb))
    return dict(zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'),
                    [a[k] + alpha * (b[k] - a[k]) for k in ('x', 'y', 'z')] + list(q)))


class PoseInterpolator:
    """Pre-index immutable samples for per-point O(log N) interpolation."""
    def __init__(self, samples, max_gap_ns, normalize):
        self.samples = samples
        self.times = tuple(item[0] for item in samples)
        self.max_gap_ns = max_gap_ns
        self.normalize = normalize

    def at(self, stamp_ns):
        i = bisect.bisect_left(self.times, stamp_ns)
        if i < len(self.samples) and self.times[i] == stamp_ns:
            return self.samples[i][1]
        if i == 0 or i == len(self.samples):
            raise RuntimeError('point timestamp is not bracketed by odometry')
        ta, a = self.samples[i-1]; tb, b = self.samples[i]
        if tb <= ta or tb-ta > self.max_gap_ns:
            raise RuntimeError('odometry gap exceeds deskew bound')
        alpha = (stamp_ns-ta)/(tb-ta)
        qa = self.normalize([a[k] for k in ('qx','qy','qz','qw')], 'odom quaternion')
        qb = self.normalize([b[k] for k in ('qx','qy','qz','qw')], 'odom quaternion')
        dot = sum(x*y for x,y in zip(qa,qb))
        if dot < 0: qb = tuple(-v for v in qb); dot = -dot
        if dot > 0.9995:
            q = self.normalize([x+alpha*(y-x) for x,y in zip(qa,qb)], 'interpolated quaternion')
        else:
            theta = math.acos(min(1.0,max(-1.0,dot)))
            q = tuple((math.sin((1-alpha)*theta)*x+math.sin(alpha*theta)*y)/math.sin(theta) for x,y in zip(qa,qb))
        return dict(zip(('x','y','z','qx','qy','qz','qw'),
                        [a[k]+alpha*(b[k]-a[k]) for k in ('x','y','z')]+list(q)))
