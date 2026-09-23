#!/usr/bin/env python3
"""Compatibility entry point; controller analysis is run by analyze_nav_bag.py."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agt_nav_benchmark.controller_analysis import analyze_cmd_vel, count_sign_changes

__all__ = ['analyze_cmd_vel', 'count_sign_changes']
