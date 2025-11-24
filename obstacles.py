import math
from typing import List, Tuple


Circle = Tuple[float, float, float]  # (cx, cy, radius)


class World2D:
    """
    Simple 2D world with circular obstacles.
    """
    def __init__(self, bounds: Tuple[float, float, float, float]):
        """
        bounds: (xmin, xmax, ymin, ymax)
        """
        self.xmin, self.xmax, self.ymin, self.ymax = bounds
        self.circles: List[Circle] = []

    def add_circle(self, cx: float, cy: float, r: float):
        self.circles.append((cx, cy, r))

    def in_bounds(self, x: float, y: float) -> bool:
        return (self.xmin <= x <= self.xmax) and (self.ymin <= y <= self.ymax)

    def collides_point(self, x: float, y: float) -> bool:
        for cx, cy, r in self.circles:
            dx = x - cx
            dy = y - cy
            if dx*dx + dy*dy <= r*r:
                return True
        return False

    def collides_segment(self, p1: Tuple[float, float], p2: Tuple[float, float], steps: int = 20) -> bool:
        """
        Approximate collision check along a segment by sampling.
        """
        x1, y1 = p1
        x2, y2 = p2
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if self.collides_point(x, y):
                return True
        return False
