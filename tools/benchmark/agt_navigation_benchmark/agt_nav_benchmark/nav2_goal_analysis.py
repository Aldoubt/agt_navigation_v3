"""Offline audit of Nav2 goals, NavigateToPose actions, and plan triggers."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass
class GoalTimelineItem:
    timestamp_ns: int
    goal_x: float
    goal_y: float
    goal_yaw: float
    change_distance: float
    source_topic: str


@dataclass
class ActionSummary:
    available: bool
    goal_count: int
    success_count: int
    cancel_count: int
    average_duration_sec: float
    feedback_count: int
    result_count: int
    aborted_count: int
    unknown_result_count: int


@dataclass
class GoalPlanAssociation:
    plan_transition_count: int
    changed_goal_count: int
    goal_triggered_plan_changes: int
    unchanged_goal_plan_changes: int
    costmap_triggered_plan_changes: int
    plan_changes_without_goal_evidence: int


def _quaternion_to_yaw(quaternion) -> float:
    return float(np.arctan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    ))


def _pose_from_object(value):
    """Extract a PoseStamped/Pose from common goal and action message layouts."""
    if value is None:
        return None
    if hasattr(value, 'pose') and hasattr(value.pose, 'position'):
        return value.pose
    if hasattr(value, 'position') and hasattr(value, 'orientation'):
        return value
    return None


def extract_goal_pose(message):
    candidates = [message]
    for attribute in ('goal', 'pose', 'request'):
        value = getattr(message, attribute, None)
        if value is not None:
            candidates.append(value)
            nested = getattr(value, 'pose', None)
            if nested is not None:
                candidates.append(nested)
    for candidate in candidates:
        pose = _pose_from_object(candidate)
        if pose is not None:
            return (
                float(pose.position.x),
                float(pose.position.y),
                _quaternion_to_yaw(pose.orientation),
            )
    return None


def _goal_id(value):
    if value is None:
        return None
    if hasattr(value, 'goal_id'):
        return _goal_id(value.goal_id)
    uuid = getattr(value, 'uuid', value)
    try:
        return tuple(int(item) for item in uuid)
    except (TypeError, ValueError):
        return str(uuid)


def build_goal_timeline(records_by_topic, explicit_topics=None, action_goal_topics=None):
    explicit_topics = explicit_topics or []
    action_goal_topics = action_goal_topics or []
    selected = []
    # Explicit goal topics are preferred. Action goal requests are a fallback;
    # using both would count one user goal twice in many Nav2 bags.
    sources = [topic for topic in explicit_topics if records_by_topic.get(topic)]
    if not sources:
        sources = [topic for topic in action_goal_topics if records_by_topic.get(topic)]
    for topic in sources:
        for record in records_by_topic.get(topic, []):
            pose = extract_goal_pose(record.message)
            if pose is not None:
                selected.append((record.timestamp_ns, *pose, topic))
    selected.sort(key=lambda item: item[0])
    items = []
    previous = None
    for timestamp_ns, x, y, yaw, topic in selected:
        if previous is not None:
            change_distance = float(np.hypot(x - previous[0], y - previous[1]))
            # Deduplicate mirrored goal_pose/action records at the same time.
            if timestamp_ns == previous[3] and change_distance < 1e-6:
                continue
        else:
            change_distance = 0.0
        items.append(GoalTimelineItem(timestamp_ns, x, y, yaw, change_distance, topic))
        previous = (x, y, yaw, timestamp_ns)
    return items


def write_goal_timeline_csv(items: Iterable[GoalTimelineItem], output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['timestamp', 'goal_x', 'goal_y', 'goal_yaw', 'change_distance'])
        for item in items:
            writer.writerow([
                f'{item.timestamp_ns / 1e9:.9f}', f'{item.goal_x:.6f}', f'{item.goal_y:.6f}',
                f'{item.goal_yaw:.6f}', f'{item.change_distance:.6f}',
            ])


def _iter_parameters_or_status(records_by_topic, topics):
    for topic in topics:
        yield from records_by_topic.get(topic, [])


def analyze_navigate_to_pose_action(records_by_topic, action_topics):
    goal_topics = action_topics.get('goal', [])
    feedback_topics = action_topics.get('feedback', [])
    result_topics = action_topics.get('result', [])
    status_topics = action_topics.get('status', [])
    action_topic_names = [name for names in action_topics.values() for name in names]
    available = bool(any(records_by_topic.get(topic) for topic in action_topic_names))
    goals = {}
    goal_times = []
    for record in _iter_parameters_or_status(records_by_topic, goal_topics):
        goal_id = _goal_id(getattr(record.message, 'goal_id', None))
        if goal_id is None:
            goal_id = f'goal_{len(goal_times)}'
        goals[goal_id] = record.timestamp_ns
        goal_times.append(record.timestamp_ns)

    feedback_count = sum(len(records_by_topic.get(topic, [])) for topic in feedback_topics)
    results = {}
    unidentified_results = []
    for record in _iter_parameters_or_status(records_by_topic, result_topics):
        goal_id = _goal_id(getattr(record.message, 'goal_id', None))
        status = getattr(record.message, 'status', None)
        if status is None:
            status = getattr(getattr(record.message, 'result', None), 'status', 0)
        if goal_id is None:
            unidentified_results.append((int(status), record.timestamp_ns))
        else:
            results[goal_id] = (int(status), record.timestamp_ns)

    # NavigateToPose_GetResult responses normally carry status but not the
    # UUID. For that common recording layout, associate results FIFO with
    # requests that have not already been identified by a UUID/status topic.
    identified_goal_ids = set(results)
    pending_goal_ids = [goal_id for goal_id, _ in sorted(goals.items(), key=lambda item: item[1]) if goal_id not in identified_goal_ids]
    for result, goal_id in zip(unidentified_results, pending_goal_ids):
        results[goal_id] = result

    # Action status arrays are useful when result recording is disabled.
    for record in _iter_parameters_or_status(records_by_topic, status_topics):
        for status_item in getattr(record.message, 'status_list', []):
            goal_id = _goal_id(getattr(status_item, 'goal_info', None))
            if goal_id is None:
                goal_id = _goal_id(getattr(status_item, 'goal_id', None))
            status = int(getattr(status_item, 'status', 0))
            if goal_id is not None:
                results.setdefault(goal_id, (status, record.timestamp_ns))

    durations = []
    success_count = cancel_count = aborted_count = unknown_count = 0
    for goal_id, (status, end_time) in results.items():
        if status == 4:
            success_count += 1
        elif status == 5:
            cancel_count += 1
        elif status == 6:
            aborted_count += 1
        else:
            unknown_count += 1
        if goal_id in goals and end_time >= goals[goal_id]:
            durations.append((end_time - goals[goal_id]) / 1e9)
    return ActionSummary(
        available=available,
        goal_count=len(goal_times),
        success_count=success_count,
        cancel_count=cancel_count,
        average_duration_sec=float(np.mean(durations)) if durations else 0.0,
        feedback_count=feedback_count,
        result_count=len(results),
        aborted_count=aborted_count,
        unknown_result_count=unknown_count,
    )


def correlate_goals_plans(goals, plan_records, runtime_metrics, goal_change_threshold_m=0.05, costmap_trigger_threshold=0.10):
    plans = sorted(plan_records, key=lambda record: record.timestamp_ns)
    transitions = 0
    changed_goals = 0
    goal_triggered = 0
    unchanged = 0
    costmap_triggered = 0
    without_evidence = 0
    for index in range(1, min(len(plans), len(runtime_metrics))):
        transitions += 1
        previous_time = plans[index - 1].timestamp_ns
        current_time = plans[index].timestamp_ns
        recent_goals = [goal for goal in goals if previous_time < goal.timestamp_ns <= current_time]
        goal_changed = any(goal.change_distance > goal_change_threshold_m for goal in recent_goals)
        if goal_changed:
            changed_goals += 1
        path_changed = runtime_metrics[index].plan_change_distance > 0.10
        costmap_changed = runtime_metrics[index].costmap_trigger_score >= costmap_trigger_threshold
        if path_changed and goal_changed:
            goal_triggered += 1
        elif path_changed and not goals:
            without_evidence += 1
        elif path_changed and not goal_changed:
            unchanged += 1
        if path_changed and costmap_changed:
            costmap_triggered += 1
    return GoalPlanAssociation(
        transitions, changed_goals, goal_triggered, unchanged,
        costmap_triggered, without_evidence,
    )


def analyze_behavior_tree_logs(records_by_topic, behavior_topics):
    records = [record for topic in behavior_topics for record in records_by_topic.get(topic, [])]
    status_changes = 0
    for record in records:
        message = record.message
        for field in ('event_log', 'events', 'status_changes'):
            value = getattr(message, field, None)
            if value is not None:
                status_changes += len(value)
                break
    return {
        'available': bool(records),
        'message_count': len(records),
        'status_change_count': status_changes,
    }


def write_nav2_runtime_report(output_path: Path, goals, action: ActionSummary, behavior, association: GoalPlanAssociation, runtime_summary, available_topic_names):
    goal_status = 'FOUND' if goals else 'MISSING'
    action_status = 'FOUND' if action.available else 'MISSING'
    if association.goal_triggered_plan_changes:
        conclusion = 'Case A: Goal changes are temporally associated with plan changes.'
        ranking = ['Goal update / repeated goal submission', 'Planner runtime behavior', 'Costmap influence']
    elif association.costmap_triggered_plan_changes:
        conclusion = 'Case C: Plan changes are synchronized with costmap changes.'
        ranking = ['Costmap update', 'Planner runtime behavior', 'Goal handling']
    elif goals and association.unchanged_goal_plan_changes:
        conclusion = 'Case B: Goal evidence is unchanged while plans change.'
        ranking = ['Planner runtime parameters or internal state', 'Goal handling / goal identity', 'Costmap influence']
    elif not goals and runtime_summary.mean_path_deviation > 0.10:
        conclusion = 'Goal evidence is MISSING; plans change while recorded costmap changes remain low. Case A/B cannot be separated from this bag.'
        ranking = ['Planner runtime parameters or internal state', 'Unobserved goal/action traffic', 'Costmap influence (low recorded trigger)']
    else:
        conclusion = 'No dominant goal/action trigger was identified.'
        ranking = ['Insufficient runtime observability', 'Planner runtime behavior', 'Costmap influence']
    with output_path.open('w', encoding='utf-8') as stream:
        stream.write('# Nav2 Goal and Runtime Analysis Report\n\n')
        stream.write('## Goal Timeline\n\n')
        stream.write(f'- Goal topic status: `{goal_status}`\n')
        stream.write(f'- Goal records: `{len(goals)}`\n')
        stream.write('- CSV: `goal_timeline.csv`\n\n')
        if goals:
            stream.write('| timestamp | goal x | goal y | goal yaw | change distance |\n|---:|---:|---:|---:|---:|\n')
            for item in goals:
                stream.write(f'| {item.timestamp_ns / 1e9:.3f} | {item.goal_x:.3f} | {item.goal_y:.3f} | {item.goal_yaw:.3f} | {item.change_distance:.3f} |\n')
            stream.write('\n')
        else:
            stream.write('MISSING: no `/goal_pose`, `/goal`, or action goal pose was recorded.\n\n')

        stream.write('## NavigateToPose Action\n\n')
        stream.write(f'- Action topic status: `{action_status}`\n')
        stream.write(f'- Goal count: `{action.goal_count}`\n')
        stream.write(f'- Success count: `{action.success_count}`\n')
        stream.write(f'- Cancel count: `{action.cancel_count}`\n')
        stream.write(f'- Aborted count: `{action.aborted_count}`\n')
        stream.write(f'- Feedback messages: `{action.feedback_count}`\n')
        stream.write(f'- Result messages: `{action.result_count}`\n')
        stream.write(f'- Average duration: `{action.average_duration_sec:.3f} s`\n')
        stream.write('- CSV: `nav2_action_metrics.csv`\n\n')
        if not action.available:
            stream.write('MISSING: no NavigateToPose action goal/feedback/result/status topic was recorded.\n\n')

        stream.write('## Behavior Tree Runtime\n\n')
        stream.write(f"- Topic status: `{'FOUND' if behavior['available'] else 'MISSING'}`\n")
        stream.write(f"- Messages: `{behavior['message_count']}`\n")
        stream.write(f"- Parsed status changes: `{behavior['status_change_count']}`\n\n")

        stream.write('## Goal / Plan / Costmap Association\n\n')
        stream.write(f'- Plan transitions: `{association.plan_transition_count}`\n')
        stream.write(f'- Goal changes above threshold: `{association.changed_goal_count}`\n')
        stream.write(f'- Goal-triggered plan changes: `{association.goal_triggered_plan_changes}`\n')
        stream.write(f'- Plan changes with unchanged goal: `{association.unchanged_goal_plan_changes}`\n')
        stream.write(f'- Plan changes without goal evidence: `{association.plan_changes_without_goal_evidence}`\n')
        stream.write(f'- Costmap-synchronized plan changes: `{association.costmap_triggered_plan_changes}`\n\n')

        stream.write('## Root Cause Ranking\n\n')
        for index, item in enumerate(ranking, 1):
            stream.write(f'{index}. {item}\n')
        stream.write(f'\n**Conclusion:** {conclusion}\n\n')
        stream.write('Runtime topic availability is part of the result: a missing goal/action trace limits causal attribution and should be fixed in the next recording.\n')


def write_action_metrics_csv(summary: ActionSummary, output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['goal_count', 'success_count', 'cancel_count', 'average_duration'])
        writer.writerow([
            summary.goal_count, summary.success_count, summary.cancel_count,
            f'{summary.average_duration_sec:.6f}',
        ])
