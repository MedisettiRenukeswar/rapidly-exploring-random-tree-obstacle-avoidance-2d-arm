import math
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional

from obstacles import World2D


@dataclass
class Node:
    x: float
    y: float
    parent: Optional[int]  # index of parent node in tree list


def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.hypot(dx, dy)


def steer(from_pt: Tuple[float, float], to_pt: Tuple[float, float], step_size: float) -> Tuple[float, float]:
    """
    Move from 'from_pt' towards 'to_pt' by at most step_size.
    """
    dx = to_pt[0] - from_pt[0]
    dy = to_pt[1] - from_pt[1]
    d = math.hypot(dx, dy)
    if d <= step_size:
        return to_pt
    ux = dx / d
    uy = dy / d
    return (from_pt[0] + ux * step_size, from_pt[1] + uy * step_size)


def rrt_plan(
    world: World2D,
    start: Tuple[float, float],
    goal: Tuple[float, float],
    step_size: float = 0.2,
    goal_tolerance: float = 0.2,
    max_iters: int = 5000,
    goal_sample_rate: float = 0.1,  # probability of sampling goal directly
) -> Optional[List[Tuple[float, float]]]:
    """
    Basic RRT planner in 2D workspace.
    Returns list of waypoints from start to goal (including both), or None.
    """
    if world.collides_point(*start) or world.collides_point(*goal):
        print("Start or goal in collision.")
        return None

    nodes: List[Node] = [Node(start[0], start[1], parent=None)]

    for it in range(max_iters):
        # Random sample
        if random.random() < goal_sample_rate:
            sample = goal
        else:
            x = random.uniform(world.xmin, world.xmax)
            y = random.uniform(world.ymin, world.ymax)
            sample = (x, y)

        # Find nearest node
        nearest_idx = 0
        nearest_dist = float("inf")
        for i, n in enumerate(nodes):
            d = distance((n.x, n.y), sample)
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        nearest_node = nodes[nearest_idx]
        new_x, new_y = steer((nearest_node.x, nearest_node.y), sample, step_size)

        # Bounds + collision
        if not world.in_bounds(new_x, new_y):
            continue
        if world.collides_segment((nearest_node.x, nearest_node.y), (new_x, new_y)):
            continue

        # Add node
        nodes.append(Node(new_x, new_y, parent=nearest_idx))

        # Check if close to goal
        if distance((new_x, new_y), goal) <= goal_tolerance:
            print(f"RRT: goal reached in {it} iterations, nodes={len(nodes)}")
            # Build final node connected to goal
            goal_idx = len(nodes)
            nodes.append(Node(goal[0], goal[1], parent=len(nodes) - 1))
            # Reconstruct path
            path: List[Tuple[float, float]] = []
            curr = goal_idx
            while curr is not None:
                n = nodes[curr]
                path.append((n.x, n.y))
                curr = nodes[curr].parent
            path.reverse()
            return path

    print("RRT: failed to find path.")
    return None
