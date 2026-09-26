"""Pure validation policy for operator-seeded initialization, not TF ownership."""
import math

MODES = ('auto', 'auto_then_manual', 'manual')


def validate_mode(value):
    if value not in MODES:
        raise ValueError(f'localization mode must be one of {MODES}: {value!r}')
    return value


def validate_seed(pose, frame, map_frame):
    if frame != map_frame:
        raise ValueError(f'manual seed must explicitly use frame {map_frame!r}')
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('manual seed contains non-finite values')
    if sum(v*v for v in values[3:]) < 1e-12:
        raise ValueError('manual seed quaternion is zero')


def validate_refinement(initial, refined, fitness, overlap, max_translation, max_yaw_deg):
    keys = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
    if not all(math.isfinite(float(p[k])) for p in (initial, refined) for k in keys):
        raise ValueError('manual GICP result contains non-finite pose')
    if not math.isfinite(fitness) or not math.isfinite(overlap):
        raise ValueError('manual GICP quality is non-finite')
    if fitness < 0 or not 0 <= overlap <= 1:
        raise ValueError('manual GICP quality is outside its valid range')
    distance = math.sqrt(sum((refined[k]-initial[k])**2 for k in ('x', 'y', 'z')))
    def yaw(p):
        x, y, z, w = (p[k] for k in ('qx', 'qy', 'qz', 'qw'))
        norm = math.sqrt(x*x+y*y+z*z+w*w)
        if norm < 1e-12:
            raise ValueError('manual GICP quaternion is zero')
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    delta_yaw = abs((yaw(refined)-yaw(initial)+math.pi) % (2*math.pi)-math.pi)
    if distance > max_translation or math.degrees(delta_yaw) > max_yaw_deg:
        raise ValueError('manual GICP refinement moved too far from operator seed; choose a closer seed')
