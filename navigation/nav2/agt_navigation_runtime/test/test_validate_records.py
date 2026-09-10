from pathlib import Path

from agt_navigation_runtime.validate_records import validate


def test_validate_records_passes_for_one_point_three_views(tmp_path: Path):
    (tmp_path / 'mission.yaml').write_text('version: 1\n', encoding='utf-8')
    (tmp_path / 'manifest.json').write_text('{}', encoding='utf-8')
    images = tmp_path / 'images'
    images.mkdir()
    rows = []
    for index in range(3):
        image = images / f'v{index}.jpg'
        image.write_bytes(b'jpg')
        rows.append(
            f'm,p,P001,v{index},{image},1,0,True,0,0,0,0,0,0,1,True,0.1,1,2,3,0,0,0,0,0\n')
    header = (
        'mission_id,map_id,point_id,view_tag,image_path,image_sec,image_nanosec,'
        'pose_valid,x,y,z,qx,qy,qz,qw,rtk_valid,rtk_age_sec,latitude,longitude,'
        'altitude,navsat_status,gimbal_heading,gimbal_roll,gimbal_pitch,camera_error_code\n')
    (tmp_path / 'captures.csv').write_text(header + ''.join(rows), encoding='utf-8')

    assert validate(tmp_path, expected_points=1, views_per_point=3, require_rtk=True) == []


def test_validate_records_detects_missing_capture(tmp_path: Path):
    (tmp_path / 'mission.yaml').write_text('version: 1\n', encoding='utf-8')
    (tmp_path / 'manifest.json').write_text('{}', encoding='utf-8')
    (tmp_path / 'captures.csv').write_text(
        'mission_id,map_id,point_id,view_tag,image_path,pose_valid,rtk_valid,camera_error_code\n',
        encoding='utf-8')
    errors = validate(tmp_path, expected_points=1, views_per_point=3, require_rtk=False)
    assert any('expected 3 capture rows' in error for error in errors)
