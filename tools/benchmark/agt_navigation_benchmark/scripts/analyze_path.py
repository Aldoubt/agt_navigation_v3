#!/usr/bin/env python3
"""Compatibility entry point; path analysis is run by analyze_nav_bag.py."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agt_nav_benchmark.path_analysis import analyze_path_message, write_path_csv

__all__ = ['analyze_path_message', 'write_path_csv']
