# AGT Navigation v3 — Localization Health & Degradation Strategy Design

## 1. Purpose

This document defines a product-oriented localization quality and degradation policy for `agt_navigation_v3`.

The design is intentionally separated from the current localization-v1 migration work. It does **not** change:

- `agt_localization_manager` ownership of `map -> odom`;
- Batch-LIO / FAST-LIO ownership of `odom -> base_link`;
- global relocalization algorithms (Polar Context / 3D-BBS / small_gicp);
- map-tracker registration backend;
- Nav2 behavior or controller parameters;
- existing public topic/message compatibility.

The goal is to turn the current localization outputs from "pose + a few diagnostics" into a clear chain:

```text
Estimator / Matcher Evidence
        ↓
Localization Quality Evaluation
        ↓
Localization State Machine
        ↓
System Monitor
        ↓
Behavior Degradation Policy
        ↓
Safety Motion Gate
```

The core principle is:

> An algorithm failure is not automatically a localization-system failure, and a localization-system failure is not automatically a robot motion failure.

---

## 2. Existing Baseline

The repository already has useful foundations.

### Existing typed state

`agt_robot_interfaces/msg/LocalizationStatus.msg` currently exposes:

```text
BOOT
WAIT_LOCAL_ODOM
WAIT_GLOBAL
LOCALIZED
DEGRADED
LOST
RELOCALIZING
```

### Existing localization metrics

`LocalizationMetrics.msg` already includes:

- current `T_map_odom`;
- correction translation / yaw delta;
- tracking innovation translation / yaw;
- global position standard deviation;
- global yaw standard deviation;
- state / tracking-health / reason.

### Existing manager invariants

The current architecture must remain frozen:

```text
Global Relocalization ----                           > Localization Manager ---> map -> odom
Local Map Tracker --------/

Local LIO Adapter -------------------------------> odom -> base_link
```

The new strategy therefore evaluates health around existing observations rather than creating another TF authority.

---

## 3. Problem Statement

The current system can identify several failures, but policy is still too close to individual algorithms.

Examples:

```text
GICP reject
Map tracker timeout
Global covariance too large
Local odom stale
Relocalization failed
```

These events should not all map to the same action.

For example:

### Case A — map tracking unavailable

```text
Local LIO       GOOD
Map tracker     BAD
Local perception GOOD
```

Desired system behavior:

```text
LOCAL_ONLY / DEGRADED
speed limit
freeze map correction
continue short-horizon local task
background relocalization
```

### Case B — map tracking and LIO both unreliable

```text
Local LIO       BAD
Map tracker     BAD
```

Desired system behavior:

```text
LOST
motion not allowed
safe stop
recovery / relocalization
```

### Case C — registration converges but is inconsistent

```text
GICP fitness    GOOD
LIO-vs-map delta very large
temporal consistency BAD
```

Desired system behavior:

```text
reject correction
do not move map -> odom
mark map-match health BAD
keep existing correction if still valid
```

The policy therefore needs multi-dimensional evidence rather than a single boolean or a single confidence score.

---

## 4. Localization Evidence Vector

Do not collapse the localization subsystem too early into `confidence = 0.82`.

Keep a structured evidence vector:

```text
LocalizationEvidence
├── freshness
├── local_odometry
├── global_match
├── map_tracking
├── consistency
├── geometry
├── temporal_stability
├── map_confidence
└── sensor_health
```

Conceptually:

```text
q = [
  q_fresh,
  q_local_odom,
  q_global_match,
  q_tracking,
  q_consistency,
  q_geometry,
  q_temporal,
  q_map,
  q_sensor
]
```

Each dimension should expose both raw evidence and a discrete grade.

Recommended grades:

```text
UNKNOWN
GOOD
MARGINAL
BAD
FAILED
```

---

## 5. Evidence Sources

### 5.1 Freshness

Measure:

- local odometry age;
- LiDAR age;
- IMU age if available;
- tracking observation age;
- global correction age;
- timestamp skew between map observation and nearest odometry.

Existing baseline thresholds must initially remain unchanged:

- local odom stale: `0.30 s`;
- local odom lost: `1.00 s`;
- global-match odom skew: `0.10 s`.

New policy must first mirror these thresholds before tuning.

### 5.2 Local odometry quality

Initial implementation should consume available signals without changing the LIO backend:

- odometry freshness;
- pose covariance if meaningful;
- linear/angular velocity discontinuity;
- short-window innovation / acceleration sanity;
- optional adapter diagnostics;
- optional future LIO degeneracy indicators.

Do **not** synthesize unsupported "LIO confidence" if the backend does not currently provide a reliable estimator.

### 5.3 Global relocalization quality

Use existing outputs from Polar Context / BBS / GICP:

- candidate score;
- coarse-search score;
- GICP fitness;
- overlap / inlier ratio where available;
- result covariance;
- timestamp alignment;
- map identity/version/generation match.

A relocalization result is only an observation candidate. It still passes manager-level consistency and timing gates.

### 5.4 Local map-tracker quality

Use:

- accepted/rejected registration result;
- GICP fitness;
- overlap / inlier ratio when available;
- innovation translation;
- innovation yaw;
- consecutive consistency count;
- rejection streak;
- tracker age / timeout.

Existing manager gates remain the first parity values:

- translation innovation gate: `0.50 m`;
- yaw innovation gate: `5 deg`;
- consistency gate: `0.20 m`, `2 deg`;
- required consecutive accepts: `2`.

### 5.5 Cross-source consistency

For map observation `T_map_base^obs` and local odometry `T_odom_base`:

```text
T_map_odom^obs = T_map_base^obs * inverse(T_odom_base)
```

Compare a candidate with the currently trusted correction:

```text
ΔT = inverse(T_map_odom^trusted) * T_map_odom^obs
```

Represent the residual in SE(3):

```text
ξ = Log(ΔT)
  = [Δx, Δy, Δz, Δroll, Δpitch, Δyaw]^T
```

For current ground-vehicle policy, the first implementation may use:

- planar translation norm;
- yaw residual;
- optional z/roll/pitch sanity gates.

A future implementation may use a covariance-weighted residual:

```text
d² = ξᵀ Σ⁻¹ ξ
```

but only after covariance sources are validated.

### 5.6 Geometry / degeneracy

Registration quality should not depend only on convergence.

Future evidence can include:

- Hessian / normal-matrix eigenvalue spectrum;
- condition number;
- observability by axis;
- local structural entropy;
- correspondence spatial distribution.

This is especially important in repetitive agricultural rows, long corridors, tree lines and low-feature areas.

The initial design reserves fields for these indicators but does not require backend modification in the first patch.

### 5.7 Temporal stability

A single good registration is insufficient.

Maintain short-window history:

- accepted result count;
- rejected result count;
- residual mean/std;
- correction velocity;
- correction direction reversals;
- state dwell time.

All state transitions must use hysteresis and/or consecutive-sample conditions.

---

## 6. Localization Quality Model

Recommended subsystem health objects:

```text
LocalOdomHealth
GlobalRelocHealth
MapTrackingHealth
CorrectionHealth
OverallLocalizationHealth
```

Each health object should contain:

```text
grade
reason
timestamp
age
raw_metrics
```

Example:

```text
MapTrackingHealth
  grade = MARGINAL
  reason = "innovation_high"
  fitness = ...
  overlap = ...
  innovation_translation = ...
  innovation_yaw = ...
  consistent_count = ...
```

The overall localization state is derived from combinations of subsystem health, not a simple average.

---

## 7. Localization State Machine

Recommended internal state model:

```text
BOOT
INITIALIZING
WAIT_GLOBAL
LOCALIZED
DEGRADED
LOCAL_ONLY
RELOCALIZING
LOST
FAULT
```

Public compatibility can continue mapping to the existing wire states until a versioned interface is introduced.

### State semantics

#### LOCALIZED

Requirements:

- local odometry usable;
- a valid global correction exists;
- correction freshness within allowed policy;
- no critical cross-source inconsistency.

#### DEGRADED

Global localization still exists, but one or more quality dimensions are marginal.

Examples:

- tracker reject streak increasing;
- correction old but still usable;
- map confidence low;
- global observation inconsistent but current correction still trustworthy.

Behavior:

- reduce speed;
- increase monitoring rate;
- suppress risky correction updates;
- prepare recovery.

#### LOCAL_ONLY

Requirements:

- local odometry remains good;
- global correction is unavailable, stale beyond navigation policy, or deliberately frozen;
- local perception remains sufficient for constrained motion.

Behavior:

- no new global route assumptions;
- preserve local trajectory continuity;
- bounded travel distance/time;
- background relocalization;
- enter safe stop if local-only budget expires.

#### RELOCALIZING

Global recovery is active.

Motion policy is decided by the behavior/safety layer, not by localization itself.

Possible modes:

- stationary relocalization;
- low-speed local-only relocalization;
- task-dependent recovery.

#### LOST

Localization cannot support motion policy.

Typical causes:

- local odometry lost;
- large unresolved correction inconsistency;
- no valid correction and local-only budget exhausted.

#### FAULT

Non-recoverable or internal contract violation, e.g.:

- frame contract violation;
- invalid quaternion / transform math failure;
- incompatible active map generation;
- repeated internal exception.

---

## 8. Hysteresis Policy

Do not transition state on one sample.

Use:

```text
enter threshold
exit threshold
minimum dwell time
consecutive bad samples
consecutive good samples
```

Example:

```text
LOCALIZED -> DEGRADED
  tracker_bad_count >= N_bad

DEGRADED -> LOCALIZED
  tracker_good_count >= N_good
  AND consistency_good
  AND dwell_time >= T_recover
```

with:

```text
N_good > 1
T_recover > 0
```

to prevent state chatter.

All thresholds belong in one policy YAML.

---

## 9. Separation of Localization, Behavior and Safety

This design requires three distinct decisions.

### 9.1 Localization

Question:

> What localization capability is currently available?

Output:

```text
LOCALIZED / DEGRADED / LOCAL_ONLY / RELOCALIZING / LOST / FAULT
```

Localization does **not** decide the final robot velocity.

### 9.2 Behavior Degradation Manager

Question:

> Given available capabilities, how should the task continue?

Examples:

```text
LOCALIZED
  -> normal navigation

DEGRADED
  -> speed limit
  -> increase obstacle margin
  -> request background relocalization

LOCAL_ONLY
  -> local corridor/row following only
  -> bounded distance/time
  -> no new global task transition

RELOCALIZING
  -> pause task state progression
  -> optionally allow low-speed recovery motion
```

Behavior policy optimizes mission continuity.

### 9.3 Safety Motion Gate

Question:

> Is commanded motion currently safe to allow?

Output should be explicit:

```text
MOTION_ALLOWED
MOTION_ALLOWED_LIMITED
MOTION_NOT_ALLOWED
EMERGENCY_STOP
```

Safety policy must remain simple and independent from BBS/GICP/LIO algorithm details.

Conceptually:

```text
Planner / Controller cmd_vel
        ↓
Safety Motion Gate
        ↓
Chassis
```

The gate should have a watchdog so monitor failure cannot silently leave motion enabled.

---

## 10. Recommended ROS Interfaces

Do not replace existing public interfaces in the first patch.

Add a versioned/internal typed message later, conceptually:

```text
LocalizationHealth.msg
----------------------
builtin_interfaces/Time stamp

uint8 state
uint8 overall_grade

HealthEvidence freshness
HealthEvidence local_odom
HealthEvidence global_reloc
HealthEvidence map_tracking
HealthEvidence consistency
HealthEvidence geometry
HealthEvidence temporal
HealthEvidence map_confidence

bool global_pose_available
bool local_motion_capable
bool relocalization_required

string reason
string map_id
string map_version
```

For the first implementation stage, a shadow topic is preferred:

```text
/agt/localization/health_shadow
```

It must not affect TF, Nav2 or vehicle commands.

---

## 11. Policy YAML

Create a single policy source, e.g.:

```text
config/localization_health_policy.yaml
```

Suggested groups:

```yaml
freshness:
  local_odom_warn_sec: 0.30
  local_odom_fail_sec: 1.00
  observation_max_skew_sec: 0.10

tracking:
  max_translation_innovation_m: 0.50
  max_yaw_innovation_deg: 5.0
  consistency_translation_m: 0.20
  consistency_yaw_deg: 2.0
  consecutive_accepts: 2

state_hysteresis:
  bad_samples_to_degraded: 3
  good_samples_to_nominal: 5
  degraded_min_dwell_sec: 2.0

local_only:
  enabled: false
  max_duration_sec: 0.0
  max_distance_m: 0.0

safety:
  health_watchdog_sec: 0.5
```

Initial values should preserve current behavior. `local_only.enabled` should remain false until vehicle tests prove the policy.

---

## 12. System-Level Health Mapping

A future `agt_system_monitor` should consume localization health together with:

- perception health;
- chassis health;
- controller health;
- sensor health;
- compute/resource health.

Recommended system severity:

```text
OK
WARN
ERROR
FATAL
```

Mapping example:

```text
Localization LOCALIZED -> OK
Localization DEGRADED  -> WARN
Localization LOCAL_ONLY -> WARN or ERROR depending on mission profile
Localization LOST      -> ERROR
Localization FAULT     -> FATAL/ERROR by fault type
```

This mapping is mission/profile dependent and must not be hard-coded in the localization algorithm.

---

## 13. Data Recording Requirements

Every localization test bag should record:

```text
/agt/odometry/local
/agt/localization/status
/agt/localization/metrics
/agt/localization/health_shadow
/agt/relocalization/pose
/agt/global_relocalization/status
/agt/map_tracking/pose
/agt/map_tracking/status
/agt/map/status
/tf
/tf_static
/cmd_vel
/cmd_vel_nav
```

Also retain raw/map query clouds where test-storage budget permits.

---

## 14. Offline Evaluation

Create a policy replay tool that accepts bag-derived metrics and reproduces the state machine without ROS runtime.

Metrics:

- false degradation rate;
- missed failure rate;
- time-to-detect;
- time-to-recover;
- state transition count;
- correction rejection/acceptance trace;
- time spent in LOCALIZED / DEGRADED / LOCAL_ONLY / LOST;
- motion command exposure after first unsafe evidence.

The policy should be tunable offline before vehicle tests.

---

## 15. Required Test Cases

### T1 — normal nominal run

Expected:

```text
LOCALIZED
no unnecessary degradation
stable correction
```

### T2 — temporary map-tracker reject

Inject or replay short tracking failure.

Expected:

```text
LOCALIZED or DEGRADED
no immediate LOST
no TF jump
```

### T3 — persistent map-tracker failure, LIO healthy

Expected:

```text
DEGRADED
correction freeze
relocalization request
```

LOCAL_ONLY is tested only after the feature is explicitly enabled.

### T4 — local odometry stale

Expected:

```text
DEGRADED at stale threshold
LOST at lost threshold
motion policy eventually denied
```

### T5 — false-good registration

Feed a converged registration with excessive innovation.

Expected:

```text
candidate rejected
current map->odom preserved
consistency health BAD
```

### T6 — correction oscillation

Alternate plausible corrections around both sides of the current estimate.

Expected:

```text
temporal health degrades
no oscillating map->odom
```

### T7 — relocalization recovery

Expected:

```text
LOST/DEGRADED
 -> RELOCALIZING
 -> accepted globally consistent correction
 -> hysteresis
 -> LOCALIZED
```

### T8 — monitor watchdog failure

Expected:

```text
health timeout
 -> motion gate fails safe
```

---

## 16. Implementation Phases

### Phase A — Shadow Quality Evaluator

Add only:

- evidence extraction;
- quality grading;
- state decision;
- shadow diagnostics topic;
- offline replay tests.

No TF or command behavior changes.

### Phase B — Typed Health Contract

After shadow parity:

- introduce versioned `LocalizationHealth`;
- bridge existing `LocalizationStatus`;
- preserve current consumers.

### Phase C — System Monitor Integration

Add:

```text
agt_system_monitor
```

to aggregate localization/perception/chassis/controller health.

Still no automatic motion degradation unless explicitly enabled.

### Phase D — Behavior Degradation

Add mission-level actions:

- speed limit;
- pause global task progression;
- background relocalization;
- bounded local-only operation.

### Phase E — Safety Motion Gate

Add independent, watchdog-protected command gating.

Safety testing must precede enabling autonomous LOCAL_ONLY movement.

---

## 17. Non-Goals

This design does not attempt to:

- replace Batch-LIO / FAST-LIO;
- replace BBS / GICP;
- solve long-term map consistency by neural networks;
- merge localization and local perception;
- move `map -> odom` ownership;
- change Nav2 planner/controller tuning;
- enable autonomous degraded motion before evidence-based vehicle validation.

---

## 18. Acceptance Criteria for the Strategy

The design is considered ready for active integration when:

1. every localization state transition is explainable by recorded evidence;
2. no single GICP/BBS failure directly forces emergency stop;
3. a false-good map correction is rejected by consistency/temporal gates;
4. transient map-tracker failures do not cause TF discontinuity;
5. local-odometry loss is detected within defined timing bounds;
6. offline replay reproduces online state transitions deterministically;
7. behavior policy and safety policy remain separate;
8. all active thresholds are centralized in a policy file;
9. TF ownership remains unchanged;
10. existing field launch profiles can run with the health evaluator in shadow mode with zero behavioral effect.

---

## 19. Recommended First Patch

The first code patch on this branch should be intentionally small:

```text
agt_localization_health/
├── evidence.py
├── evaluator.py
├── state_policy.py
├── replay.py
└── tests/
```

Inputs:

```text
LocalizationMetrics
LocalizationStatus
map-tracker status
global-relocalization status
local odometry timing
```

Output:

```text
/agt/localization/health_shadow
```

No TF broadcaster.
No `cmd_vel` subscriber/publisher.
No Nav2 changes.

This creates a measurable quality-policy layer before it is allowed to influence vehicle behavior.
