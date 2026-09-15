"""Pure timestamped controller-analysis primitives; no ROS or vehicle dependency."""
import bisect
import math


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def percentile(values, fraction):
    if not values:
        return float('nan')
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def signal_stats(samples):
    values = [v for _, v in samples]
    if not values:
        return {'count': 0, 'duration_s': 0.0, 'frequency_hz': float('nan')}
    duration = samples[-1][0] - samples[0][0]
    return {'count': len(values), 'duration_s': duration,
            'frequency_hz': (len(values)-1)/duration if duration > 0 else float('nan'),
            'mean': sum(values)/len(values), 'rms': math.sqrt(sum(v*v for v in values)/len(values)),
            'p50': percentile(values, .5), 'p90': percentile(values, .9),
            'min': min(values), 'max': max(values)}


def absolute_stats(samples):
    result = signal_stats([(t, abs(v)) for t, v in samples])
    result['mean_abs'] = result.get('mean', float('nan'))
    return result


def zero_crossings(samples, deadband=0.02):
    """Count sign transitions outside a zero zone; returns stable event timestamps."""
    prior = 0; events = []
    for stamp, value in samples:
        sign = 1 if value > deadband else -1 if value < -deadband else 0
        if sign and prior and sign != prior:
            events.append(stamp)
        if sign:
            prior = sign
    duration = samples[-1][0] - samples[0][0] if len(samples) > 1 else 0.0
    return {'deadband_radps': deadband, 'timestamps': events, 'count': len(events),
            'frequency_hz': len(events)/duration if duration > 0 else 0.0}


def previous_age(query_stamps, state_stamps):
    result = []
    for stamp in query_stamps:
        index = bisect.bisect_right(state_stamps, stamp) - 1
        result.append(float('nan') if index < 0 else stamp - state_stamps[index])
    return result


def nearest_values(query_stamps, samples):
    stamps = [s for s, _ in samples]; values = [v for _, v in samples]; result = []
    for stamp in query_stamps:
        index = bisect.bisect_left(stamps, stamp)
        choices = [i for i in (index-1, index) if 0 <= i < len(stamps)]
        result.append(float('nan') if not choices else values[min(choices, key=lambda i: abs(stamps[i]-stamp))])
    return result


def delta_stats(upstream, downstream):
    stamps = [s for s, _ in upstream]
    aligned = nearest_values([s for s, _ in downstream], upstream)
    deltas = [(s, value - reference) for (s, value), reference in zip(downstream, aligned) if not math.isnan(reference)]
    base = signal_stats(deltas)
    base['changed_sample_ratio'] = sum(abs(v) > 1e-6 for _, v in deltas)/len(deltas) if deltas else float('nan')
    return base


def nearest_path_error(x, y, yaw, path):
    best = None
    for first, second in zip(path, path[1:]):
        dx, dy = second[0]-first[0], second[1]-first[1]; length2 = dx*dx + dy*dy
        if length2 <= 1e-12: continue
        u = max(0.0, min(1.0, ((x-first[0])*dx + (y-first[1])*dy)/length2))
        px, py = first[0]+u*dx, first[1]+u*dy
        candidate = (math.hypot(x-px, y-py), wrap_angle(yaw-math.atan2(dy, dx)))
        if best is None or candidate[0] < best[0]: best = candidate
    return best or (float('nan'), float('nan'))
