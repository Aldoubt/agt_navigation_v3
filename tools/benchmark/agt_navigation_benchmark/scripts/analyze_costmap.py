#!/usr/bin/env python3
"""Compatibility entry point; costmap analysis is run by analyze_nav_bag.py."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agt_nav_benchmark.costmap_analysis import analyze_costmap_message, write_costmap_csv

__all__ = ['analyze_costmap_message', 'write_costmap_csv']
