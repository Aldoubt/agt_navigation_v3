#!/usr/bin/env python3
"""Source-tree wrapper for the PCD/PGM/YAML asset validator."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agt_nav_benchmark.map_asset_validator import main


if __name__ == '__main__':
    main()
