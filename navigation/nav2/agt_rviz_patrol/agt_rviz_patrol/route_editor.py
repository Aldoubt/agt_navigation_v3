"""Bounded, revisioned route edits. No ROS or GUI dependencies."""
import math


class RouteEditor:
    def __init__(self, max_points=2000, max_length=200.0):
        self.points = []
        self.revision = 0
        self.history = []
        self.redo_stack = []
        self.max_points = max_points
        self.max_length = max_length

    def validate(self, points):
        if len(points) > self.max_points:
            raise ValueError('too many vertices (maximum 2000)')
        clean = []
        for point in points:
            if len(point) != 2:
                raise ValueError('each vertex requires x,y')
            x, y = map(float, point)
            if not all(math.isfinite(v) and abs(v) <= 100000 for v in (x, y)):
                raise ValueError('nonfinite or out-of-bounds vertex')
            if not clean or math.hypot(x-clean[-1][0], y-clean[-1][1]) >= 1e-4:
                clean.append((x, y))
        if sum(math.dist(a, b) for a, b in zip(clean, clean[1:])) > self.max_length:
            raise ValueError('route exceeds 200 m')
        return clean

    def replace(self, points, expected_revision):
        if expected_revision != self.revision:
            raise ValueError('stale edit: reload current draft')
        clean = self.validate(points)
        if clean == self.points:
            return False
        self.history.append(self.points[:])
        self.history = self.history[-20:]
        self.points = clean
        self.redo_stack.clear()
        self.revision += 1
        return True

    def undo(self):
        if not self.history:
            raise ValueError('nothing to undo')
        self.redo_stack.append(self.points[:])
        self.points = self.history.pop()
        self.revision += 1

    def redo(self):
        if not self.redo_stack:
            raise ValueError('nothing to redo')
        self.history.append(self.points[:])
        self.points = self.redo_stack.pop()
        self.revision += 1
