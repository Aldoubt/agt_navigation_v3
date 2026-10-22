#!/usr/bin/env python3
"""Evaluate actual field acceptance observations, never invent measurements.

Requires independent measured target/arrival positions in metres, and measured
30 m gate crossing time. ROS/Nav2 poses are NOT independent ground truth.
No physical motion, ROS imports, or map data are accessed.

  python3 field_ws/acceptance.py --navigation navigation.csv --speed speed.csv
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

NAV_LIMITS_M = {5.0: 0.12, 7.0: 0.15, 10.0: 0.20}
EXPECTED_NAV_REPEATS = 5
SPEED_GATES_M = 30.0
SPEED_REPEATS = 2
SPEED_MIN_MPS = 1.0
INDEPENDENT_MEASUREMENT_SOURCES = frozenset({
    "total_station", "independent_survey", "manual_tape",
})


def number(record, key):
    try:
        value = float(record[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"missing or invalid numeric {key}") from exc
    if not math.isfinite(value):
        raise ValueError(f"nonfinite {key}")
    return value


def check_source(row):
    source = row.get("measurement_source", "")
    if source not in INDEPENDENT_MEASUREMENT_SOURCES:
        raise ValueError(
            f"unsupported measurement_source={source!r}; "
            "use independent survey, total station or manual tape measurement")
    return source


def assess_navigation(records):
    groups = defaultdict(list)
    seen = set()
    for row in records:
        distance = number(row, "distance_m")
        if distance not in NAV_LIMITS_M:
            raise ValueError(f"unsupported distance_m={distance}; use 5/7/10")
        run = row.get("run_id", "")
        if not run or not run.strip():
            raise ValueError("missing run_id")
        if (distance, run) in seen:
            raise ValueError(f"duplicate run_id={run} at {distance}m")
        seen.add((distance, run))
        check_source(row)
        tx = number(row, "target_x_m")
        ty = number(row, "target_y_m")
        ax = number(row, "arrival_x_m")
        ay = number(row, "arrival_y_m")
        error = math.hypot(ax - tx, ay - ty)
        groups[distance].append(error)

    results = []
    complete = True
    for d, limit in NAV_LIMITS_M.items():
        errors = groups[d]
        if len(errors) != EXPECTED_NAV_REPEATS:
            complete = False
        mean = sum(errors) / len(errors) if errors else None
        results.append({
            "distance_m": d,
            "count": len(errors),
            "required_count": EXPECTED_NAV_REPEATS,
            "mean_error_m": round(mean, 6) if mean is not None else None,
            "maximum_allowed_mean_error_m": limit,
            "result": (
                "INCOMPLETE" if len(errors) != EXPECTED_NAV_REPEATS else
                "PASS" if mean <= limit else "FAIL"
            ),
        })
    return {
        "status": "INCOMPLETE" if not complete else
                  "PASS" if all(x["result"] == "PASS" for x in results) else "FAIL",
        "measurements": results,
    }


def assess_speed(records):
    observed = []
    seen = set()
    for row in records:
        run = row.get("run_id", "")
        if not run or not run.strip():
            raise ValueError("speed record missing run_id")
        if run in seen:
            raise ValueError(f"duplicate speed run_id={run}")
        seen.add(run)
        check_source(row)
        distance = number(row, "gate_distance_m")
        duration = number(row, "elapsed_s")
        if abs(distance - SPEED_GATES_M) > 1e-6:
            raise ValueError("speed gate distance must be 30m")
        if duration <= 0.0:
            raise ValueError("elapsed_s must be positive")
        observed.append(round(distance / duration, 6))

    if len(observed) != SPEED_REPEATS:
        status = "INCOMPLETE"
    elif all(speed >= SPEED_MIN_MPS for speed in observed):
        status = "PASS"
    else:
        status = "FAIL"
    return {
        "status": status,
        "count": len(observed),
        "required_count": SPEED_REPEATS,
        "minimum_average_speed_mps": SPEED_MIN_MPS,
        "observed_gate_average_mps": observed,
    }


def combined_report(navigation_records, speed_records):
    nav = assess_navigation(navigation_records)
    spd = assess_speed(speed_records)
    result = "INCOMPLETE" if "INCOMPLETE" in (nav["status"], spd["status"]) else (
        "FAIL" if "FAIL" in (nav["status"], spd["status"]) else "PASS"
    )
    return {
        "schema_version": 1,
        "result": result,
        "truth_source": "INDEPENDENT_MEASUREMENT_REQUIRED",
        "navigation": nav,
        "speed": spd,
        "hardware_slope_and_endurance": "NOT_EVALUATED",
        "obstacle_stop_distance": "NOT_EVALUATED",
    }


def read_csv(path):
    if path is None:
        return []
    with Path(path).open(newline="", encoding="utf-8") as stream:
        table = csv.DictReader(stream)
        if not table.fieldnames:
            raise ValueError(f"no CSV header in {path}")
        return list(table)


def main(argv=None):
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--navigation", help="independent survey endpoint CSV")
    cli.add_argument("--speed", help="30m timed gate CSV")
    cli.add_argument("--output", help="optional JSON report destination")
    args = cli.parse_args(argv)
    try:
        report = combined_report(read_csv(args.navigation), read_csv(args.speed))
        output = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
        if args.output:
            Path(args.output).write_text(output, encoding="utf-8")
        print(output, end="")
        return {"PASS": 0, "FAIL": 1, "INCOMPLETE": 2}[report["result"]]
    except (ValueError, OSError) as exc:
        print(f"INVALID MEASUREMENT INPUT: {exc}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())
