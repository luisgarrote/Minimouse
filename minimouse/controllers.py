"""
controllers.py

Student-friendly controllers for MicromouseEnv:

1) Bug2 controller (simple, readable)
   - make_bug2_controller(env) -> (controller_fn, reset_fn)

2) Path following utilities + Path-planning controller
   - follow_waypoints(env, pose, waypoints, wp_idx) -> (vl, vr, new_idx)
   - make_path_controller(env, planner="astar") -> (controller_fn, reset_fn)

This file intentionally keeps the same mechanical interface as your notebook:
  controller(obs) -> (vl, vr)

Dependencies:
  - numpy, math
  - planners_and_smoothing.plan_waypoints, bundle_adjust_smooth
"""

from __future__ import annotations

import math
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from planners_and_smoothing import plan_waypoints, bundle_adjust_smooth


Point = Tuple[float, float]


# -----------------------------
# Small math helpers
# -----------------------------
def angle_wrap(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    a = float(a)
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def dist(p: Point, q: Point) -> float:
    return math.hypot(float(p[0]) - float(q[0]), float(p[1]) - float(q[1]))


def compute_m_line(start: Point, goal: Point) -> Tuple[Point, Point]:
    """Represent M-line as (start, goal)."""
    return (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))


def point_line_dist(p: Point, line: Tuple[Point, Point]) -> float:
    """Distance from point p to infinite line through line[0] -> line[1]."""
    (x0, y0), (x1, y1) = line
    px, py = p
    dx, dy = (x1 - x0), (y1 - y0)
    denom = math.hypot(dx, dy)
    if denom < 1e-9:
        return math.hypot(px - x0, py - y0)
    # area / base
    return abs(dy * px - dx * py + x1 * y0 - y1 * x0) / denom


# -----------------------------
# Bug2 controller factory
# -----------------------------
def make_bug2_controller(env) -> Tuple[Callable[[Dict], Tuple[float, float]], Callable[[], None]]:
    """
    Returns:
      controller(obs) -> (vl, vr)
      reset()         -> clears controller state (call on env.reset/new maze)
    """
    state = {
        "mode": "go_to_goal",
        "m_line": None,     # ((x0,y0),(xg,yg))
        "hit_dist": None,   # distance-to-goal at hit
        "goal_cell": None,  # to detect changes
    }

    def reset():
        state["mode"] = "go_to_goal"
        state["m_line"] = None
        state["hit_dist"] = None
        state["goal_cell"] = None

    def controller(obs: Dict) -> Tuple[float, float]:
        x, y, th = obs["pose"]
        laser = obs["laser"]
        gx_cell, gy_cell = map(int, obs["goal_cell"])

        # goal in world coords
        gx, gy = env._cell_center_world(gx_cell, gy_cell)
        pos = (float(x), float(y))
        goal = (float(gx), float(gy))

        # reset on goal change
        if state["goal_cell"] != (gx_cell, gy_cell):
            state["goal_cell"] = (gx_cell, gy_cell)
            state["mode"] = "go_to_goal"
            state["m_line"] = None
            state["hit_dist"] = None

        # init M-line once
        if state["m_line"] is None:
            state["m_line"] = compute_m_line(pos, goal)

        # laser sectors (env convention: +angles = left, -angles = right)
        n = len(laser)
        angles = np.linspace(-math.pi / 2, math.pi / 2, n)

        front_mask = np.abs(angles) < math.radians(25)
        left_mask = angles > math.radians(25)
        right_mask = angles < -math.radians(25)

        front = float(np.min(laser[front_mask])) if np.any(front_mask) else float(env.max_range)
        left = float(np.min(laser[left_mask])) if np.any(left_mask) else float(env.max_range)
        right = float(np.min(laser[right_mask])) if np.any(right_mask) else float(env.max_range)

        # parameters
        v_nom = 0.18 * env.cell_size
        front_block = 0.45 * env.cell_size
        wall_dist = 0.35 * env.cell_size

        mline_eps = 0.08 * env.cell_size
        leave_gain = 0.05 * env.cell_size

        goal_stop = 0.45 * env.cell_size

        d_goal = dist(pos, goal)
        if d_goal < goal_stop:
            return 0.0, 0.0

        # Mode: go to goal
        if state["mode"] == "go_to_goal":
            ang_goal = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
            err = angle_wrap(ang_goal - float(th))

            if front < front_block:
                # hit: switch to wall-follow and start turning (don't freeze)
                state["mode"] = "wall_follow"
                state["hit_dist"] = d_goal
                v = 0.0
                w = +1.8
            else:
                v = v_nom
                w = 1.2 * err

        # Mode: wall follow (left-hand rule)
        else:
            # corner handling: if blocked ahead, turn right hard
            if front < front_block:
                v = 0.0
                w = -2.2
            else:
                # keep a distance from left wall
                err_wall = wall_dist - left
                v = 0.12 * env.cell_size
                # If left is too close (left small) -> err positive -> turn right => negative w
                w = -3.0 * err_wall

            # Bug2 leave condition: near M-line AND closer than at hit
            line_dist = point_line_dist(pos, state["m_line"])
            if state["hit_dist"] is not None:
                if (line_dist < mline_eps) and (d_goal < state["hit_dist"] - leave_gain):
                    state["mode"] = "go_to_goal"
                    state["hit_dist"] = None

        # convert (v,w) -> (vl,vr)
        L = float(env.wheel_base)
        vl = float(v - 0.5 * L * w)
        vr = float(v + 0.5 * L * w)

        # clip
        max_wheel = 3.0 * env.cell_size
        vl = float(np.clip(vl, -max_wheel, +max_wheel))
        vr = float(np.clip(vr, -max_wheel, +max_wheel))

        return vl, vr

    return controller, reset


# -----------------------------
# Waypoint following
# -----------------------------
def follow_waypoints(env, pose_xyth, waypoints: Optional[List[Point]], wp_idx: int) -> Tuple[float, float, int]:
    """
    Simple waypoint follower: steer to current waypoint and move forward.
    Returns (vl, vr, new_wp_idx).
    """
    x, y, th = pose_xyth
    x = float(x); y = float(y); th = float(th)

    if waypoints is None or wp_idx >= len(waypoints):
        return 0.0, 0.0, wp_idx

    # parameters
    v_nom = 0.22 * env.cell_size
    w_gain = 1.8
    reach = 0.20 * env.cell_size
    slow_dist = 0.50 * env.cell_size

    # advance waypoint if close
    tx, ty = waypoints[wp_idx]
    d = math.hypot(tx - x, ty - y)
    while d < reach and wp_idx < len(waypoints) - 1:
        wp_idx += 1
        tx, ty = waypoints[wp_idx]
        d = math.hypot(tx - x, ty - y)

    # steer
    ang = math.atan2(ty - y, tx - x)
    err = angle_wrap(ang - th)

    v = v_nom
    if d < slow_dist:
        v *= max(0.25, d / slow_dist)

    w = w_gain * err

    L = float(env.wheel_base)
    vl = float(v - 0.5 * L * w)
    vr = float(v + 0.5 * L * w)

    # clip
    max_wheel = 3.0 * env.cell_size
    vl = float(np.clip(vl, -max_wheel, +max_wheel))
    vr = float(np.clip(vr, -max_wheel, +max_wheel))

    return vl, vr, wp_idx


# -----------------------------
# Path-planning controller factory
# -----------------------------
def make_path_controller(env, planner: str = "astar") -> Tuple[Callable[[Dict], Tuple[float, float]], Callable[[], None]]:
    """
    A* / RRT / RRT* + bundle smoothing + waypoint following.

    - If no path exists (or goal changed), it plans a new path, smooths it, and stores it.
    - Then it follows the stored waypoints each step.

    planner: "astar" | "rrt" | "rrtstar"
    """
    state = {
        "goal_cell": None,
        "waypoints": None,   # list[(x,y)] in world coords
        "wp_idx": 0,
        "planner": str(planner).lower().strip(),
    }

    def reset():
        state["goal_cell"] = None
        state["waypoints"] = None
        state["wp_idx"] = 0

    def controller(obs: Dict) -> Tuple[float, float]:
        x, y, th = obs["pose"]
        gx_cell, gy_cell = map(int, obs["goal_cell"])
        goal_cell = (gx_cell, gy_cell)

        # Stop when close to goal center (matches env's done radius style)
        gx, gy = env._cell_center_world(gx_cell, gy_cell)
        if math.hypot(float(gx) - float(x), float(gy) - float(y)) < 0.45 * env.cell_size:
            return 0.0, 0.0

        # Need (re)plan?
        need_plan = False
        if state["goal_cell"] != goal_cell:
            need_plan = True
        if state["waypoints"] is None or len(state["waypoints"]) < 2:
            need_plan = True
        if state["wp_idx"] >= (len(state["waypoints"]) if state["waypoints"] else 0):
            need_plan = True

        if need_plan:
            state["goal_cell"] = goal_cell
            state["wp_idx"] = 0

            wps = plan_waypoints(env, (x, y, th), goal_cell, planner=state["planner"])
            if wps is None:
                state["waypoints"] = None
                return 0.0, 0.0

            # smooth once and store
            wps = bundle_adjust_smooth(env, wps, iters=25, alpha=0.4)
            state["waypoints"] = wps

        vl, vr, new_idx = follow_waypoints(env, (x, y, th), state["waypoints"], int(state["wp_idx"]))
        state["wp_idx"] = int(new_idx)
        return float(vl), float(vr)

    return controller, reset
