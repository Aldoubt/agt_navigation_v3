from pathlib import Path
from agt_navigation_runtime.mission_schema import load_mission


def test_example_schema(tmp_path: Path):
    p = tmp_path / 'm.yaml'
    p.write_text('''version: 1\nmission_id: T\nmap_id: M\npoints:\n  - id: P1\n    pose: {x: 1, y: 2, yaw: 0}\n    views:\n      - {tag: front, heading: 0}\n''', encoding='utf-8')
    mission = load_mission(str(p))
    assert mission.mission_id == 'T'
    assert mission.points[0].views[0].tag == 'front'
