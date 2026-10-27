"""Stop evidence: fresh wheel feedback, zero final command, bounded LIO pose window."""
import math
from collections import deque


class StopGate:
    def __init__(self, window=1.0, displacement=.025, rotation=.035):
        self.window, self.displacement, self.rotation = window, displacement, rotation
        self.poses = deque(maxlen=1000)
        self.wheel = self.command = None

    def pose(self, stamp, received, x, y, yaw, frame):
        values = (stamp, received, x, y, yaw)
        if not all(math.isfinite(v) for v in values):
            self.poses.clear()
            return
        if self.poses and (stamp <= self.poses[-1][0] or frame != self.poses[-1][6]
                           or stamp - self.poses[-1][0] > .5):
            self.poses.clear()
        self.poses.append((*values, 0, frame))
        while len(self.poses) > 2 and stamp - self.poses[1][0] >= self.window:
            self.poses.popleft()

    def evaluate(self, now, ros_now, freshness=.5):
        if not self.poses or self.wheel is None or self.command is None:
            return False, 'waiting for wheel/command/LIO evidence'
        p = self.poses[-1]
        w = self.wheel  # stamp, received, linear speed, angular speed
        c = self.command  # received, linear speed, angular speed
        if not all(math.isfinite(v) for v in (*w, *c)):
            return False, 'nonfinite wheel/command evidence'
        if (not 0 <= now-p[1] <= freshness or not -.05 <= ros_now-p[0] <= freshness
                or not 0 <= now-w[1] <= freshness or not -.05 <= ros_now-w[0] <= freshness
                or not 0 <= now-c[0] <= freshness):
            return False, 'stale wheel/command/LIO evidence'
        if p[0]-self.poses[0][0] < self.window:
            return False, 'waiting for full LIO pose window'
        first = self.poses[0]
        distance = max(math.hypot(q[2]-first[2], q[3]-first[3]) for q in self.poses)
        angle = max(abs(math.atan2(math.sin(q[4]-first[4]), math.cos(q[4]-first[4]))) for q in self.poses)
        good = (abs(w[2]) <= .01 and abs(w[3]) <= .02 and
                abs(c[1]) <= .001 and abs(c[2]) <= .001 and
                distance <= self.displacement and angle <= self.rotation)
        return good, (f'stationary={good}; wheel={w[2]:.3f}m/s,{w[3]:.3f}rad/s; '
                      f'command={c[1]:.3f}m/s,{c[2]:.3f}rad/s; '
                      f'LIO_window={distance:.3f}m,{angle:.3f}rad')
