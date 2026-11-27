"""Figures for the demonstration / submission evidence."""
import matplotlib
matplotlib.use("Agg")                       # save files, no window needed
import matplotlib.pyplot as plt
import numpy as np

from drone.maps import N, START, GOAL


def plot_plan(grid, result, title, path):
    """Occupancy grid, searched cells, planned route, S and G (Assignment 3 style)."""
    img = np.ones((N, N, 3))
    img[grid == 1] = (0, 0, 0)
    for r, c in result.searched:
        if grid[r - 1, c - 1] == 0:
            img[r - 1, c - 1] = (0.65, 0.82, 1.0)
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(img, extent=(0.5, N + 0.5, N + 0.5, 0.5))
    ax.set_xticks(range(1, N + 1)); ax.set_yticks(range(1, N + 1))
    ax.set_xticks(np.arange(0.5, N + 1), minor=True); ax.set_yticks(np.arange(0.5, N + 1), minor=True)
    ax.grid(which="minor", color="0.6", linewidth=0.5)
    if result.found:
        rows = [r for r, _ in result.route]; cols = [c for _, c in result.route]
        ax.plot(cols, rows, "-", color=(0, 0.65, 0.2), linewidth=3, label=f"A* route ({result.cost} moves)")
        ax.legend(loc="upper center", fontsize=8)
    ax.text(START[1], START[0], "S", color="r", ha="center", va="center", fontweight="bold")
    ax.text(GOAL[1], GOAL[0], "G", color="b", ha="center", va="center", fontweight="bold")
    ax.set_xlabel("column c"); ax.set_ylabel("row r")
    ax.set_title(title)
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)
    return path


def plot_trajectory(waypoints, rows, L, origin, theta, title, path, events=()):
    """Planned waypoints vs estimated trajectory, in world coordinates."""
    xs = [r["stateEstimate.x"] for r in rows if r.get("stateEstimate.x") is not None]
    ys = [r["stateEstimate.y"] for r in rows if r.get("stateEstimate.y") is not None]
    fig, ax = plt.subplots(figsize=(7, 6))
    # map outline and cell grid in world coordinates
    from drone.coords import grid_to_world
    for k in range(N + 1):
        a = [grid_to_world(k * L, 0, origin, theta), grid_to_world(k * L, N * L, origin, theta)]
        b = [grid_to_world(0, k * L, origin, theta), grid_to_world(N * L, k * L, origin, theta)]
        for seg in (a, b):
            ax.plot([p[0] for p in seg], [p[1] for p in seg], color="0.85", linewidth=0.5)
    ax.plot([w[0] for w in waypoints], [w[1] for w in waypoints], "b--o", markersize=3, label="Planned route")
    if xs:
        ax.plot(xs, ys, "r-", linewidth=1.2, label="Estimated trajectory")
    ax.plot(waypoints[0][0], waypoints[0][1], "rs", label="S"); ax.plot(waypoints[-1][0], waypoints[-1][1], "bs", label="G / last")
    ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(title); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)
    return path


def plot_shutter(rows, enter_dark, exit_dark, title, path):
    """Raw shutter, thresholds and classification state over time."""
    fast = [r for r in rows if r.get("motion.shutter") is not None]
    if not fast:
        return None
    t = [r["t"] for r in fast]
    s = [r["motion.shutter"] for r in fast]
    state = [1 if r.get("surface") == "DARK" else 0 if r.get("surface") == "LIGHT" else np.nan for r in fast]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 5), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    a1.plot(t, s, linewidth=0.8, label="motion.shutter (raw)")
    if enter_dark is not None:
        a1.axhline(enter_dark, color="k", linestyle="--", linewidth=0.8, label=f"enter dark {enter_dark:.0f}")
        a1.axhline(exit_dark, color="0.5", linestyle="--", linewidth=0.8, label=f"exit dark {exit_dark:.0f}")
    a1.set_ylabel("shutter (clock cycles)"); a1.legend(fontsize=8); a1.set_title(title)
    a2.step(t, state, where="post"); a2.set_yticks([0, 1]); a2.set_yticklabels(["LIGHT", "DARK"])
    a2.set_xlabel("time (s)")
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)
    return path
