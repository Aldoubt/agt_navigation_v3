#!/usr/bin/env python3
"""Run the full bag analyzer, including controller tracking outputs."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agt_nav_benchmark.analyze_nav_bag import main


if __name__ == '__main__':
    main()
