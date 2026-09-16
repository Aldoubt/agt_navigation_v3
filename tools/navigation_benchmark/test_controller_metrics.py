import math
from controller_metrics import delta_stats, nearest_path_error, previous_age, signal_stats, wrap_angle, zero_crossings

def test_frequency_and_alignment():
    assert signal_stats([(0, 1), (.1, 2), (.2, 3)])['frequency_hz'] == 10
    assert previous_age([.05, .21], [0, .2]) == [.05, .009999999999999981]
    assert delta_stats([(0, 1), (.1, 2)], [(0.01, 1.5), (.11, 1.0)])['changed_sample_ratio'] == 1

def test_geometry_and_crossings():
    assert math.isclose(wrap_angle(4), -2.2831853071795862)
    cross, heading = nearest_path_error(1, 1, 0, [(0, 0), (2, 0)])
    assert cross == 1 and heading == 0
    result = zero_crossings([(0, -.1), (.1, .01), (.2, .1), (.3, -.1)])
    assert result['count'] == 2 and result['deadband_radps'] == .02
