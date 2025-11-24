import math
import random
from typing import List, Tuple, Optional

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from arm_math import forward_kinematics, inverse_kinematics_2link, L1, L2
from obstacles import World2D
from rrt_planner import rrt_plan


def build_world() -> World2D:
    """
    Create a simple world with circular obstacles.
    """
    world = World2D(bounds=(-2.5, 2.5, -2.5, 2.5))

    # Add some circular obstacles
    world.add_circle(0.5, 0.5, 0.4)
    world.add_circle(-0.8, 0.3, 0.5)
    world.add_circle(0.0, -0.8, 0.5)
    return world


def random_reachable_goal(world: World2D) -> Tuple[float, float]:
    """
    Sample a random reachable, obstacle-free goal for the end-effector.
    """
    margin = 0.1
    r_min = abs(L1 - L2) + margin
    r_max = (L1 + L2) - margin

    for _ in range(200):
        radius = random.uniform(r_min, r_max)
        angle = random.uniform(-math.pi, math.pi)
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)

        if world.in_bounds(x, y) and not world.collides_point(x, y):
            return x, y

    print("WARNING: Failed to find random collision-free goal, using fallback.")
    return 0.5, 0.5


def compute_joint_path(path_xy: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    Convert workspace path (x, y) waypoints into joint-space path (theta1, theta2).
    """
    joint_path: List[Tuple[float, float]] = []
    for (x, y) in path_xy:
        ik = inverse_kinematics_2link(x, y, elbow_up=True)
        if ik is None:
            print(f"[IK] Failed for waypoint ({x:.2f}, {y:.2f}), skipping.")
            continue
        joint_path.append(ik)
    return joint_path


def plan_new_path(
    world: World2D,
    start_xy: Tuple[float, float],
) -> Optional[Tuple[List[Tuple[float, float]], List[Tuple[float, float]], Tuple[float, float]]]:
    """
    Sample a new goal, run RRT, convert to joint path.
    Returns (path_xy, joint_path, goal_xy) or None.
    """
    for attempt in range(30):
        goal_xy = random_reachable_goal(world)
        print(f"[PLAN] Attempt {attempt+1}: start={start_xy}, goal={goal_xy}")

        path_xy = rrt_plan(
            world,
            start_xy,
            goal_xy,
            step_size=0.2,
            goal_tolerance=0.2,
            max_iters=3000,
            goal_sample_rate=0.15,
        )

        if path_xy is None:
            print("[PLAN] RRT failed for this goal, retrying...")
            continue

        joint_path = compute_joint_path(path_xy)
        if len(joint_path) < 3:
            print("[PLAN] Not enough IK waypoints, retrying with new goal...")
            continue

        print(f"[PLAN] SUCCESS: {len(path_xy)} workspace waypoints, "
              f"{len(joint_path)} joint waypoints.")
        return path_xy, joint_path, goal_xy

    print("[PLAN] FAILED: no valid path after multiple attempts.")
    return None


def main():
    world = build_world()

    # Initial pose (straight along +x)
    theta1_init = 0.0
    theta2_init = 0.0
    _, (x_end_init, y_end_init) = forward_kinematics(theta1_init, theta2_init)
    start_xy = (x_end_init, y_end_init)

    plan_res = plan_new_path(world, start_xy)
    if plan_res is None:
        print("Initial planning failed. Exiting.")
        return

    path_xy, joint_path, goal_xy = plan_res

    # ---------- Matplotlib setup ----------
    fig, ax = plt.subplots()
    ax.set_aspect("equal", "box")
    ax.set_xlim(world.xmin, world.xmax)
    ax.set_ylim(world.ymin, world.ymax)
    ax.set_title("2-Link Arm – Dynamic RRT Path Planning with Obstacles")

    # Obstacles
    for cx, cy, r in world.circles:
        circle = plt.Circle((cx, cy), r, color="gray", alpha=0.4)
        ax.add_patch(circle)

    # Path in workspace (updated on every re-plan)
    path_line, = ax.plot(
        [p[0] for p in path_xy],
        [p[1] for p in path_xy],
        "g--",
        linewidth=1.5,
        label="Planned path",
    )

    # Start & goal markers
    start_marker, = ax.plot([start_xy[0]], [start_xy[1]], "bo", label="Start")
    goal_marker, = ax.plot([goal_xy[0]], [goal_xy[1]], "ro", label="Goal")

    # Robot drawing
    base_plot, = ax.plot([], [], "ko", markersize=6)
    elbow_plot, = ax.plot([], [], "bo", markersize=6)
    end_plot, = ax.plot([], [], "ro", markersize=6)
    link1_line, = ax.plot([], [], "b-", linewidth=3)
    link2_line, = ax.plot([], [], "r-", linewidth=3)

    ax.legend(loc="upper left")

    # Animation state
    idx = 0
    current_joint_path = joint_path
    current_path_xy = path_xy

    def init_anim():
        base_plot.set_data([0], [0])
        return base_plot, elbow_plot, end_plot, link1_line, link2_line, path_line, start_marker, goal_marker

    def replan_from_last():
        nonlocal current_joint_path, current_path_xy, start_xy, goal_xy, idx

        # Get last end-effector position from last joint path
        theta1_last, theta2_last = current_joint_path[-1]
        _, (x_end, y_end) = forward_kinematics(theta1_last, theta2_last)
        start_xy = (x_end, y_end)

        plan2 = plan_new_path(world, start_xy)
        if plan2 is None:
            print("[ANIM] Replan FAILED, staying in last pose (will try again next time).")
            return False

        new_path_xy, new_joint_path, new_goal_xy = plan2
        current_path_xy = new_path_xy
        current_joint_path = new_joint_path
        goal_xy = new_goal_xy
        idx = 0

        # Update visuals for new plan
        path_line.set_data(
            [p[0] for p in current_path_xy],
            [p[1] for p in current_path_xy],
        )
        start_marker.set_data([start_xy[0]], [start_xy[1]])
        goal_marker.set_data([goal_xy[0]], [goal_xy[1]])

        print("[ANIM] Replanned successfully to new random goal.")
        return True

    def update(frame_idx):
        nonlocal idx, current_joint_path

        # If we've finished current path → replan
        if idx >= len(current_joint_path):
            ok = replan_from_last()
            if not ok:
                # If replan failed, just hold the last joint pose
                theta1, theta2 = current_joint_path[-1]
            else:
                theta1, theta2 = current_joint_path[0]
                idx = 1  # next frame will use the next waypoint
        else:
            theta1, theta2 = current_joint_path[idx]
            idx += 1  # <-- IMPORTANT: let idx grow past len() to trigger replan

        # FK to get link positions
        (x1, y1), (x2, y2) = forward_kinematics(theta1, theta2)

        base_plot.set_data([0], [0])
        elbow_plot.set_data([x1], [y1])
        end_plot.set_data([x2], [y2])

        link1_line.set_data([0, x1], [0, y1])
        link2_line.set_data([x1, x2], [y1, y2])

        return base_plot, elbow_plot, end_plot, link1_line, link2_line, path_line, start_marker, goal_marker

    anim = FuncAnimation(
        fig,
        update,
        init_func=init_anim,
        frames=300,
        interval=80,
        blit=True,
    )
    SAVE_GIF = True   # set False when only viewing animation
    if SAVE_GIF:
        import os
        from matplotlib.animation import PillowWriter
        os.makedirs("assets", exist_ok=True)
        print("Saving dynamic re-goal GIF...")
        gif_writer = PillowWriter(fps=10)
        anim.save("assets/sim_rrt_ik_arm_dynamic_re_goal.gif", writer=gif_writer, dpi=80)
        print("Saved → assets/sim_rrt_ik_arm_dynamic_re_goal.gif")
    else:
        plt.show()



if __name__ == "__main__":
    main()
