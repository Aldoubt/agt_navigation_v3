from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_exactly_four_top_level_launch_files():
    names = sorted(path.name for path in (ROOT / 'launch').glob('*.launch.py'))
    assert names == [
        'debug.launch.py',
        'hardware.launch.py',
        'localization.launch.py',
        'navigation.launch.py',
    ]


def test_unique_owner_inclusions():
    launch_text = {
        path.name: path.read_text(encoding='utf-8')
        for path in (ROOT / 'launch').glob('*.launch.py')
    }
    all_text = '\n'.join(launch_text.values())
    assert all_text.count("'agt_localization_manager', 'localization_manager.launch.py'") == 1
    assert all_text.count("package='agt_pointcloud_preprocessor'") == 1
    assert "'tracked_chassis_description', 'display.launch.py'" in launch_text[
        'hardware.launch.py']
    assert "tracked_chassis_description" not in launch_text['localization.launch.py']
    assert "tracked_chassis_description" not in launch_text['navigation.launch.py']
    assert "agt_localization_manager" not in launch_text['debug.launch.py']


def test_six_config_sources_are_installed():
    cmake = (ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    for name in (
        'robot.yaml', 'navigation.yaml', 'controller.yaml',
        'costmap.yaml', 'perception.yaml', 'safety.yaml',
    ):
        assert f'config/{name}' in cmake


def test_geometry_motion_and_obstacle_sources_are_not_duplicated():
    config_root = ROOT.parents[1] / 'config'
    configs = {
        path.name: yaml.safe_load(path.read_text(encoding='utf-8'))
        for path in config_root.glob('*.yaml')
    }
    assert sorted(configs) == [
        'controller.yaml', 'costmap.yaml', 'navigation.yaml',
        'perception.yaml', 'robot.yaml', 'safety.yaml',
    ]
    assert 'footprint' in configs['robot.yaml']['agt_robot_config']['ros__parameters']
    assert 'agt_motion_limits' in configs['safety.yaml']
    assert '/agt/navigation/points_obstacles' == (
        configs['perception.yaml']['agt_pointcloud_preprocessor']['ros__parameters'][
            'output_topic'])
    perception = configs['perception.yaml']['agt_pointcloud_preprocessor']['ros__parameters']
    for filter_name in ('self_filter', 'rear_filter', 'ground_filter', 'voxel'):
        assert isinstance(perception[filter_name]['enabled'], bool)
    assert perception['self_filter']['enabled'] is True
    assert perception['ground_filter']['enabled'] is True
    assert perception['ground_filter']['mode'] == 'radial_slope'
    assert perception['voxel']['enabled'] is True
    # Orchard default keeps rear returns; the narrow mask remains configurable.
    assert perception['rear_filter']['enabled'] is False

    serialized = {name: yaml.safe_dump(data) for name, data in configs.items()}
    assert [name for name, text in serialized.items() if 'footprint:' in text] == [
        'robot.yaml']
    assert [name for name, text in serialized.items() if 'forward_mps:' in text] == [
        'safety.yaml']
    assert [name for name, text in serialized.items()
            if 'output_topic: /agt/navigation/points_obstacles' in text] == [
                'perception.yaml']
