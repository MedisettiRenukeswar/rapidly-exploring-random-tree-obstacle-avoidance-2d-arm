import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import List, Tuple

from arm_math import forward_kinematics, inverse_kinematics_2link, L1, L2
from obstacles import World2D
from rrt_planner import rrt_plan


def build_world() -> World2D:
    # Workspace covers reachable region of arm
    world = World2D(bounds=(-2.5, 2.5, -2.5, 2.5))

    # Add some circular obstacles
    world.add_circle(0.5, 0.5, 0.4)
    world.add_circle(-0.8, 0.3, 0.5)
    world.add_circle(0.0, -0.8, 0.5)
    return world


def compute_joint_path(path_xy: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    Convert workspace path (x, y) waypoints into joint-space path (theta1, theta2).
    Uses IK, elbow_up, and skips waypoints that are not solvable.
    """
    joint_path: List[Tuple[float, float]] = []
    for (x, y) in path_xy:
        ik = inverse_kinematics_2link(x, y, elbow_up=True)
        if ik is None:
            print(f"WARNING: IK failed for waypoint ({x:.2f}, {y:.2f}), skipping.")
            continue
        joint_path.append(ik)
    return joint_path


def main():
    # Start & goal in workspace
    start_xy = (L1 + L2 - 0.2, 0.0)   # near max reach on +x
    goal_xy = (-1.0, 1.0)

    world = build_world()
    path_xy = rrt_plan(world, start_xy, goal_xy, step_size=0.2, goal_tolerance=0.2)
    if path_xy is None:
        print("No path found, exiting.")
        return

    joint_path = compute_joint_path(path_xy)
    if len(joint_path) < 2:
        print("Not enough valid IK waypoints for animation.")
        return

    # ---------- Matplotlib setup ----------
    fig, ax = plt.subplots()
    ax.set_aspect("equal", "box")
    ax.set_xlim(world.xmin, world.xmax)
    ax.set_ylim(world.ymin, world.ymax)
    ax.set_title("2-Link Arm – RRT Path Planning with Obstacles")

    # Plot obstacles
    for cx, cy, r in world.circles:
        circle = plt.Circle((cx, cy), r, color="gray", alpha=0.4)
        ax.add_patch(circle)

    # Plot RRT path in workspace
    xs = [p[0] for p in path_xy]
    ys = [p[1] for p in path_xy]
    ax.plot(xs, ys, "g--", linewidth=1.5, label="Planned path")

    # Plot start & goal
    ax.plot([start_xy[0]], [start_xy[1]], "bo", label="Start")
    ax.plot([goal_xy[0]], [goal_xy[1]], "ro", label="Goal")

    # Robot drawing
    base_plot, = ax.plot([], [], "ko", markersize=6)
    elbow_plot, = ax.plot([], [], "bo", markersize=6)
    end_plot, = ax.plot([], [], "ro", markersize=6)
    link1_line, = ax.plot([], [], "b-", linewidth=3)
    link2_line, = ax.plot([], [], "r-", linewidth=3)

    ax.legend(loc="upper left")

    # Animation state
    idx = 0

    def init():
        base_plot.set_data([0], [0])
        return base_plot, elbow_plot, end_plot, link1_line, link2_line

    def update(frame_idx):
        nonlocal idx
        if idx >= len(joint_path):
            idx = len(joint_path) - 1

        theta1, theta2 = joint_path[idx]
        (x1, y1), (x2, y2) = forward_kinematics(theta1, theta2)

        base_plot.set_data([0], [0])
        elbow_plot.set_data([x1], [y1])
        end_plot.set_data([x2], [y2])

        link1_line.set_data([0, x1], [0, y1])
        link2_line.set_data([x1, x2], [y1, y2])

        if idx < len(joint_path) - 1:
            idx += 1

        return base_plot, elbow_plot, end_plot, link1_line, link2_line

    anim = FuncAnimation(fig, update, init_func=init,
                         frames=len(joint_path), interval=80, blit=True)
    SAVE_GIF = True   # set False if you don't want to save every time
    if SAVE_GIF:
        import os
        from matplotlib.animation import PillowWriter
        os.makedirs("assets", exist_ok=True)
        print("Saving GIF... this will take a few seconds...")
        gif_writer = PillowWriter(fps=5)
        anim.save("assets/sim_rrt_ik_arm.gif", writer=gif_writer, dpi=80)
        print("Saved → assets/sim_rrt_ik_arm.gif")
    else:
        plt.show()



if __name__ == "__main__":
    main()
