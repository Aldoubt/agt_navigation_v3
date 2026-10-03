"""ROS-independent state policy; invalid hard evidence revokes authority immediately."""
from dataclasses import dataclass, field
from enum import IntEnum
import math


class Mode(IntEnum):
    GLOBAL = 0
    DEGRADED = 1
    LOCAL_TASK = 2
    RECOVERING = 3
    SAFE = 4


@dataclass(frozen=True)
class Config:
    enabled: bool = False
    field_verified: bool = False
    owner_id: str = 'agt_task_continuity'
    source_timeout_sec: float = 0.3
    global_quality_timeout_sec: float = 3.0
    task_timeout_sec: float = 0.5
    authorization_ttl_sec: float = 0.15
    degrade_dwell_sec: float = 0.5
    acquire_dwell_sec: float = 0.5
    q_global_low: float = 0.0
    q_global_high: float = 0.0
    q_row_low: float = 0.0
    q_row_high: float = 0.0
    q_odom_min: float = 0.0
    braking_decel_mps2: float = 0.0
    stopping_latency_sec: float = 0.0
    clearance_margin_m: float = 0.0

    def motion_parameters_valid(self):
        positive = (self.q_global_low, self.q_global_high, self.q_row_low,
                    self.q_row_high, self.q_odom_min, self.braking_decel_mps2,
                    self.stopping_latency_sec, self.clearance_margin_m)
        return (self.field_verified and bool(self.owner_id)
                and all(math.isfinite(v) and v > 0 for v in positive)
                and 0 < self.q_global_low < self.q_global_high <= 1
                and 0 < self.q_row_low < self.q_row_high <= 1
                and self.q_odom_min <= 1
                and self.stopping_latency_sec >= max(self.source_timeout_sec,
                                                      self.task_timeout_sec,
                                                      self.authorization_ttl_sec))


@dataclass(frozen=True)
class Evidence:
    stamp: float = 0.0
    receipt_age: float = math.inf
    epoch: int = 0
    valid: bool = False
    quality: float = 0.0
    position_std: float = math.inf
    yaw_std: float = math.inf
    clearance_valid: bool = False
    clearance_left: float = 0.0
    clearance_right: float = 0.0
    clearance_front: float = 0.0
    map_id: str = ''
    map_version: str = ''
    map_hash: str = ''


@dataclass(frozen=True)
class Task:
    stamp: float = 0.0
    receipt_age: float = math.inf
    task_id: str = ''
    request_epoch: int = 0
    odom_epoch: int = 0
    active: bool = False
    anchor_valid: bool = False
    allow_local: bool = False
    map_id: str = ''
    map_version: str = ''
    map_hash: str = ''
    max_distance: float = 0.0
    max_duration: float = 0.0
    max_position_std: float = 0.0
    max_yaw_std: float = 0.0
    max_linear: float = 0.0
    max_angular: float = 0.0
    plan_stamp_ns: int = 0
    plan_frame: str = ''
    handover_complete: bool = False
    recovery_complete: bool = False
    terminal_distance: float = 0.0


@dataclass(frozen=True)
class Observation:
    now: float
    monotonic: float
    task: Task = field(default_factory=Task)
    global_quality: Evidence = field(default_factory=Evidence)
    row: Evidence = field(default_factory=Evidence)
    odom: Evidence = field(default_factory=Evidence)
    odom_stamp: float = 0.0
    odom_receipt_age: float = math.inf
    odom_x: float = math.nan
    odom_y: float = math.nan
    speed: float = math.nan
    payload_allowed: bool = False
    localization_valid: bool = False
    base_allowed: bool = False
    recovery_active: bool = False


@dataclass(frozen=True)
class Decision:
    mode: Mode
    control_epoch: int
    authorized: bool
    reason: str
    remaining_distance: float
    remaining_time: float
    local_distance: float
    local_elapsed: float


class ContinuityPolicy:
    def __init__(self, config=Config()):
        self.config = config
        self.mode = Mode.SAFE
        self.control_epoch = 1
        self._key = None
        self._plan_key = None
        self._safe_latched = False
        self._global_bad_since = None
        self._local_good_since = None
        self._global_good_since = None
        self._local_since = None
        self._local_distance = 0.0
        self._last_pose = None
        self._last_odom_stamp = 0.0
        self._last_ros_now = None
        self._last_clock_progress = None
        self._transition_plan_ns = 0
        self._was_authorized = False

    def _set_mode(self, mode, current_plan):
        if mode != self.mode:
            self.mode = mode
            self.control_epoch += 1
            # A mode change needs a new plan and cancellation barrier. A heartbeat
            # of the previously active plan cannot grant the new controller epoch.
            self._transition_plan_ns = current_plan

    def _fresh(self, evidence, now, timeout=None):
        limit = self.config.source_timeout_sec if timeout is None else timeout
        return (evidence.stamp > 0 and -0.01 <= now - evidence.stamp <= limit
                and 0 <= evidence.receipt_age <= limit)

    def step(self, obs):
        c, t = self.config, obs.task
        key = (t.task_id, t.request_epoch, t.odom_epoch)
        if key != self._key:
            self._key = key
            self._plan_key = None
            self._safe_latched = False
            self._global_bad_since = self._local_good_since = self._global_good_since = None
            self._local_since = None
            self._local_distance = 0.0
            self._last_pose = None
            self._last_odom_stamp = 0.0
            self._set_mode(Mode.SAFE, t.plan_stamp_ns)
        plan_key = (t.plan_stamp_ns, t.plan_frame, t.handover_complete)
        if plan_key != self._plan_key:
            self._plan_key = plan_key
            self.control_epoch += 1
        elapsed = 0.0 if self._local_since is None else max(0.0, obs.monotonic - self._local_since)
        remaining_time = max(0.0, t.max_duration - elapsed)
        remaining_distance = max(0.0, min(t.max_distance - self._local_distance, t.terminal_distance))

        def answer(reason, permitted=False):
            authorized = permitted and c.enabled and c.motion_parameters_valid()
            if self._was_authorized and not authorized:
                # Reacquiring the same quality gate must not make commands from
                # before withdrawal valid again, even if no mode changed.
                self.control_epoch += 1
                self._transition_plan_ns = t.plan_stamp_ns
            self._was_authorized = authorized
            return Decision(self.mode, self.control_epoch,
                            authorized, reason,
                            remaining_distance, remaining_time, self._local_distance, elapsed)

        def safe(reason, latch=True):
            self._safe_latched = self._safe_latched or latch
            self._set_mode(Mode.SAFE, t.plan_stamp_ns)
            return answer(reason)

        rollback = self._last_ros_now is not None and obs.now < self._last_ros_now
        if self._last_ros_now is None or obs.now != self._last_ros_now:
            self._last_clock_progress = obs.monotonic
        self._last_ros_now = obs.now
        if rollback:
            return safe('clock_reversed')
        if obs.monotonic - self._last_clock_progress > c.source_timeout_sec:
            return safe('clock_not_advancing')
        if not t.active or not t.task_id:
            return safe('no_active_task', latch=False)
        if not (-0.01 <= obs.now - t.stamp <= c.task_timeout_sec and
                0 <= t.receipt_age <= c.task_timeout_sec):
            return safe('task_context_expired')
        if not t.anchor_valid or t.odom_epoch == 0 or not t.map_id or not t.map_version or not t.map_hash:
            return safe('trusted_anchor_missing')
        if obs.odom.epoch != t.odom_epoch or obs.row.epoch not in (0, t.odom_epoch):
            return safe('odom_epoch_changed')
        if self._safe_latched:
            return answer('safe_latched_new_request_required')
        if not obs.payload_allowed:
            return safe('payload_permission_missing')
        if not obs.base_allowed:
            return safe('base_state_or_drive_permission_invalid')
        if not (self._fresh(obs.odom, obs.now) and obs.odom.valid and
                math.isfinite(obs.odom.quality) and obs.odom.quality >= c.q_odom_min and
                math.isfinite(obs.odom.position_std) and 0 <= obs.odom.position_std <= t.max_position_std and
                math.isfinite(obs.odom.yaw_std) and 0 <= obs.odom.yaw_std <= t.max_yaw_std):
            return safe('odom_quality_invalid')
        if not (obs.odom_stamp > 0 and -0.01 <= obs.now - obs.odom_stamp <= c.source_timeout_sec and
                0 <= obs.odom_receipt_age <= c.source_timeout_sec and
                all(math.isfinite(x) for x in (obs.odom_x, obs.odom_y, obs.speed))):
            return safe('odom_pose_expired')
        if obs.odom_stamp < self._last_odom_stamp:
            return safe('odom_stamp_reversed')
        if obs.odom_stamp > self._last_odom_stamp:
            if self._local_since is not None and self._last_pose is not None:
                self._local_distance += math.hypot(obs.odom_x - self._last_pose[0], obs.odom_y - self._last_pose[1])
            self._last_pose = (obs.odom_x, obs.odom_y)
            self._last_odom_stamp = obs.odom_stamp
            remaining_distance = max(0.0, min(t.max_distance - self._local_distance, t.terminal_distance))
        row = obs.row
        clearance_ok = (self._fresh(row, obs.now) and row.clearance_valid and
                        all(math.isfinite(v) for v in (row.clearance_left, row.clearance_right, row.clearance_front))
                        and row.clearance_left >= c.clearance_margin_m
                        and row.clearance_right >= c.clearance_margin_m)
        if c.braking_decel_mps2 > 0:
            stopping_distance = abs(obs.speed) * c.stopping_latency_sec + obs.speed ** 2 / (2 * c.braking_decel_mps2) + c.clearance_margin_m
        else:
            stopping_distance = math.inf
        if not clearance_ok or row.clearance_front < stopping_distance:
            return safe('footprint_or_stopping_envelope_unobserved')
        g = obs.global_quality
        global_identity = (g.map_id, g.map_version, g.map_hash) == (t.map_id, t.map_version, t.map_hash)
        global_valid = (self._fresh(g, obs.now, c.global_quality_timeout_sec) and g.valid and global_identity and
                        g.epoch == t.odom_epoch and obs.localization_valid and math.isfinite(g.quality))
        global_high = global_valid and g.quality >= c.q_global_high
        global_low = not global_valid or g.quality < c.q_global_low
        local_valid = row.valid and row.epoch == t.odom_epoch and math.isfinite(row.quality)
        local_high = local_valid and row.quality >= c.q_row_high
        local_low = not local_valid or row.quality < c.q_row_low
        self._local_good_since = (obs.monotonic if self._local_good_since is None else self._local_good_since) if local_high else None
        self._global_good_since = (obs.monotonic if self._global_good_since is None else self._global_good_since) if global_high else None

        if self.mode == Mode.SAFE:
            if global_high:
                self._set_mode(Mode.GLOBAL, t.plan_stamp_ns)
            else:
                return answer('waiting_trusted_global')
        if self.mode == Mode.GLOBAL:
            if global_low:
                self._global_bad_since = obs.monotonic if self._global_bad_since is None else self._global_bad_since
                if obs.monotonic - self._global_bad_since >= c.degrade_dwell_sec:
                    self._set_mode(Mode.DEGRADED, t.plan_stamp_ns)
                return answer('global_quality_low_authority_revoked')
            self._global_bad_since = None
        if self.mode == Mode.DEGRADED:
            if global_high:
                self._set_mode(Mode.GLOBAL, t.plan_stamp_ns)
            elif not t.allow_local:
                return safe('segment_forbids_local')
            elif self._local_good_since is not None and obs.monotonic - self._local_good_since >= c.acquire_dwell_sec:
                self._local_since = obs.monotonic if self._local_since is None else self._local_since
                self._set_mode(Mode.LOCAL_TASK, t.plan_stamp_ns)
            else:
                return answer('waiting_local_observability')
        if self.mode in (Mode.LOCAL_TASK, Mode.RECOVERING):
            if local_low:
                return safe('row_quality_low')
            if remaining_time <= 0 or remaining_distance <= stopping_distance:
                return safe('local_budget_or_terminal_reached')
            global_acquired = (self._global_good_since is not None
                               and obs.monotonic - self._global_good_since >= c.acquire_dwell_sec)
            if self.mode == Mode.LOCAL_TASK and (obs.recovery_active or global_acquired):
                self._set_mode(Mode.RECOVERING, t.plan_stamp_ns)
            if self.mode == Mode.RECOVERING and not obs.recovery_active and not global_high:
                self._set_mode(Mode.LOCAL_TASK, t.plan_stamp_ns)
            if self.mode == Mode.RECOVERING and global_acquired and t.recovery_complete:
                self._set_mode(Mode.GLOBAL, t.plan_stamp_ns)
        required_frame = 'map' if self.mode == Mode.GLOBAL else 'odom'
        ready = (t.handover_complete and t.plan_stamp_ns > 0 and
                 t.plan_stamp_ns != self._transition_plan_ns and t.plan_frame == required_frame)
        if not ready:
            return answer('awaiting_terminal_barrier_and_new_plan')
        if not c.enabled or not c.motion_parameters_valid():
            return answer('shadow_or_field_parameters_unverified')
        return answer('global_authorized' if self.mode == Mode.GLOBAL else 'bounded_local_authorized', True)
