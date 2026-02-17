"""
planners_and_smoothing.py

Student-friendly path planners and path "bundle adjustment" smoothing for MicromouseEnv.

What this file provides
-----------------------
Planners:
  - astar_cells(env, start_cell, goal_cell) -> list[(cx, cy)] or None
  - rrt_plan_world(env, start_xy, goal_xy, ...) -> list[(x, y)] or None
  - rrtstar_plan_world(env, start_xy, goal_xy, ...) -> list[(x, y)] or None
  - plan_waypoints(env, pose, goal_cell, planner="astar") -> list[(x, y)] or None

Smoothing ("bundle adjustment"-style):
  - segment_is_free(env, p0, p1, step=...) -> bool
  - bundle_adjust_smooth(env, waypoints, iters=..., alpha=...) -> list[(x, y)]

Notes
-----
- The "bundle adjustment" here is a simple optimization-style smoothing pass that pulls
  internal waypoints toward the average of their neighbors while keeping the path collision-free.
- RRT/RRT* are continuous planners in world coordinates; they rely on env._is_wall(x, y)
  and simple collision checking along line segments.
"""

from __future__ import annotations

import math
import random
import heapq
from typing import List, Tuple, Optional, Dict

import numpy as np


Point = Tuple[float, float]
Cell = Tuple[int, int]


# -----------------------------
# Collision / geometry helpers
# -----------------------------
def segment_is_free(env, p0: Point, p1: Point, step: float) -> bool:
    """
    Check that the straight segment from p0 to p1 is collision-free by sampling points.

    step: sampling spacing in world units (suggest ~ 0.1*cell_size to 0.2*cell_size)
    """
    x0, y0 = p0
    x1, y1 = p1
    dx, dy = x1 - x0, y1 - y0
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return True

    n = max(2, int(dist / max(step, 1e-6)))
    for i in range(n + 1):
        t = i / n
        x = x0 + t * dx
        y = y0 + t * dy
        if env._is_wall(float(x), float(y)):
            return False
    return True


# -----------------------------
# Smoothing ("bundle adjustment")
# -----------------------------
def bundle_adjust_smooth(
    env,
    waypoints: Optional[List[Point]],
    iters: int = 25,
    alpha: float = 0.4,
) -> Optional[List[Point]]:
    """
    Very simple 'bundle adjustment'-style smoothing:
    Iteratively pulls internal points toward the average of neighbors
    and only accepts updates that keep local segments collision-free.

    - iters: number of smoothing iterations
    - alpha: 0..1 smoothing strength per iteration (0.3-0.5 works well)

    Returns a new list of waypoints, or None.
    """
    if waypoints is None or len(waypoints) < 3:
        return waypoints

    # mutable copy
    wps = [list(p) for p in waypoints]
    step = 0.15 * env.cell_size

    for _ in range(max(0, int(iters))):
        changed = False
        for i in range(1, len(wps) - 1):
            x, y = wps[i]
            x_prev, y_prev = wps[i - 1]
            x_next, y_next = wps[i + 1]

            # target point is neighbor average
            tx = 0.5 * (x_prev + x_next)
            ty = 0.5 * (y_prev + y_next)

            newx = (1.0 - alpha) * x + alpha * tx
            newy = (1.0 - alpha) * y + alpha * ty

            # accept only if safe
            if env._is_wall(float(newx), float(newy)):
                continue
            if not segment_is_free(env, (x_prev, y_prev), (newx, newy), step=step):
                continue
            if not segment_is_free(env, (newx, newy), (x_next, y_next), step=step):
                continue

            wps[i][0] = float(newx)
            wps[i][1] = float(newy)
            changed = True

        if not changed:
            break

    return [(float(x), float(y)) for x, y in wps]


# -----------------------------
# A* on cell graph
# -----------------------------
def _cells_dims_from_occ(env) -> Tuple[int, int]:
    """
    Infer cell grid size from the env's occ grid dimensions.

    Typical Micromouse style:
      occ_width  = 2*cells_w + 1
      occ_height = 2*cells_h + 1
    """
    cells_w = (env.W - 1) // 2
    cells_h = (env.H - 1) // 2
    return int(cells_w), int(cells_h)


def can_move_cell(env, ax: int, ay: int, bx: int, by: int) -> bool:
    """
    True if the wall between two neighbor cells is open.
    Uses env.occ where 0 means free, 1 means wall.
    """
    # cell center in occ coords
    x1, y1 = (2 * ax + 1), (2 * ay + 1)
    x2, y2 = (2 * bx + 1), (2 * by + 1)
    # wall cell lies between centers
    wx, wy = (x1 + x2) // 2, (y1 + y2) // 2
    # occ indexed as [y, x]
    return env.occ[int(wy), int(wx)] == 0


def astar_cells(env, start_cell: Cell, goal_cell: Cell) -> Optional[List[Cell]]:
    """
    A* over maze cells (4-connected).
    Returns a list of cells from start to goal, or None if no path.
    """
    sx, sy = map(int, start_cell)
    gx, gy = map(int, goal_cell)
    cells_w, cells_h = _cells_dims_from_occ(env)

    def h(cx: int, cy: int) -> int:
        return abs(cx - gx) + abs(cy - gy)

    open_heap: List[Tuple[int, int, Cell]] = []
    heapq.heappush(open_heap, (h(sx, sy), 0, (sx, sy)))

    came_from: Dict[Cell, Cell] = {}
    gscore: Dict[Cell, int] = {(sx, sy): 0}
    closed = set()

    nbrs = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while open_heap:
        f, g, (cx, cy) = heapq.heappop(open_heap)
        if (cx, cy) in closed:
            continue
        closed.add((cx, cy))

        if (cx, cy) == (gx, gy):
            path: List[Cell] = [(cx, cy)]
            while (cx, cy) in came_from:
                cx, cy = came_from[(cx, cy)]
                path.append((cx, cy))
            path.reverse()
            return path

        for dx, dy in nbrs:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < cells_w and 0 <= ny < cells_h):
                continue
            if not can_move_cell(env, cx, cy, nx, ny):
                continue

            ng = g + 1
            if (nx, ny) not in gscore or ng < gscore[(nx, ny)]:
                gscore[(nx, ny)] = ng
                came_from[(nx, ny)] = (cx, cy)
                nf = ng + h(nx, ny)
                heapq.heappush(open_heap, (nf, ng, (nx, ny)))

    return None


def cells_to_waypoints(env, cell_path: List[Cell]) -> List[Point]:
    """Convert cell path -> world waypoint path (cell centers)."""
    wps: List[Point] = []
    for cx, cy in cell_path:
        x, y = env._cell_center_world(int(cx), int(cy))
        wps.append((float(x), float(y)))
    return wps


def world_to_nearest_cell(env, x: float, y: float) -> Cell:
    """
    Approximate: convert world pose to a cell index using the same grid structure.
    Works well when the robot is roughly inside a cell.
    """
    ix = int(float(x) / env.cell_size)
    iy = int(float(y) / env.cell_size)

    cells_w, cells_h = _cells_dims_from_occ(env)
    cx = max(0, min(((ix - 1) // 2), cells_w - 1))
    cy = max(0, min(((iy - 1) // 2), cells_h - 1))
    return int(cx), int(cy)


# -----------------------------
# RRT / RRT*
# -----------------------------
def sample_free_point_world(env, max_tries: int = 200) -> Optional[Point]:
    """Uniformly sample a collision-free point in the world."""
    for _ in range(max(1, int(max_tries))):
        x = random.uniform(0.0, float(env.W) * float(env.cell_size))
        y = random.uniform(0.0, float(env.H) * float(env.cell_size))
        if not env._is_wall(float(x), float(y)):
            return (float(x), float(y))
    return None


def nearest_node(nodes: List[Point], p: Point) -> int:
    """Index of nearest node to p (linear scan)."""
    px, py = p
    best_i = 0
    best_d2 = float("inf")
    for i, (x, y) in enumerate(nodes):
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
    return best_i


def steer(a: Point, b: Point, step: float) -> Point:
    """Move from a toward b by at most step distance."""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return (float(ax), float(ay))
    if d <= step:
        return (float(bx), float(by))
    t = step / d
    return (float(ax + t * dx), float(ay + t * dy))


def reconstruct_path(nodes: List[Point], parent: List[int], goal_idx: int) -> List[Point]:
    """Reconstruct path from parent pointers."""
    path: List[Point] = []
    i = int(goal_idx)
    while i != -1:
        path.append(nodes[i])
        i = parent[i]
    path.reverse()
    return path


def rrt_plan_world(
    env,
    start_xy: Point,
    goal_xy: Point,
    step_len: Optional[float] = None,
    goal_sample_rate: float = 0.15,
    max_iters: int = 3000,
) -> Optional[List[Point]]:
    """
    Basic RRT in world coordinates.

    - goal_sample_rate: probability of sampling the goal (goal bias)
    - step_len: extension step size (defaults to ~0.45*cell_size)
    """
    if step_len is None:
        step_len = 0.45 * env.cell_size

    sample_step = 0.15 * env.cell_size

    # quick win: direct connection
    if segment_is_free(env, start_xy, goal_xy, step=sample_step):
        return [start_xy, goal_xy]

    nodes: List[Point] = [start_xy]
    parent: List[int] = [-1]

    for _ in range(max(1, int(max_iters))):
        if random.random() < float(goal_sample_rate):
            rnd = goal_xy
        else:
            rnd = sample_free_point_world(env)
            if rnd is None:
                continue

        ni = nearest_node(nodes, rnd)
        newp = steer(nodes[ni], rnd, step=float(step_len))

        if env._is_wall(float(newp[0]), float(newp[1])):
            continue
        if not segment_is_free(env, nodes[ni], newp, step=sample_step):
            continue

        nodes.append(newp)
        parent.append(int(ni))

        # connect to goal if close
        if math.hypot(newp[0] - goal_xy[0], newp[1] - goal_xy[1]) <= float(step_len):
            if segment_is_free(env, newp, goal_xy, step=sample_step):
                nodes.append(goal_xy)
                parent.append(len(nodes) - 2)
                return reconstruct_path(nodes, parent, len(nodes) - 1)

    return None


def rrtstar_plan_world(
    env,
    start_xy: Point,
    goal_xy: Point,
    step_len: Optional[float] = None,
    goal_sample_rate: float = 0.15,
    max_iters: int = 4000,
    neighbor_radius: Optional[float] = None,
) -> Optional[List[Point]]:
    """
    Basic RRT* (rewiring for lower cost).

    neighbor_radius: fixed neighbor radius (defaults to ~1.2*cell_size)
    """
    if step_len is None:
        step_len = 0.45 * env.cell_size
    if neighbor_radius is None:
        neighbor_radius = 1.2 * env.cell_size

    sample_step = 0.15 * env.cell_size

    if segment_is_free(env, start_xy, goal_xy, step=sample_step):
        return [start_xy, goal_xy]

    nodes: List[Point] = [start_xy]
    parent: List[int] = [-1]
    cost: List[float] = [0.0]

    def near_indices(p: Point) -> List[int]:
        px, py = p
        r2 = float(neighbor_radius) ** 2
        idxs: List[int] = []
        for i, (x, y) in enumerate(nodes):
            if (x - px) ** 2 + (y - py) ** 2 <= r2:
                idxs.append(i)
        return idxs

    best_goal_idx: Optional[int] = None
    best_goal_cost: float = float("inf")

    for _ in range(max(1, int(max_iters))):
        if random.random() < float(goal_sample_rate):
            rnd = goal_xy
        else:
            rnd = sample_free_point_world(env)
            if rnd is None:
                continue

        ni = nearest_node(nodes, rnd)
        newp = steer(nodes[ni], rnd, step=float(step_len))

        if env._is_wall(float(newp[0]), float(newp[1])):
            continue

        near = near_indices(newp)
        if not near:
            near = [ni]

        # choose best parent among neighbors
        best_p = None
        best_p_cost = float("inf")
        for j in near:
            if segment_is_free(env, nodes[j], newp, step=sample_step):
                c = cost[j] + math.hypot(nodes[j][0] - newp[0], nodes[j][1] - newp[1])
                if c < best_p_cost:
                    best_p_cost = c
                    best_p = j

        if best_p is None:
            continue

        nodes.append(newp)
        parent.append(int(best_p))
        cost.append(float(best_p_cost))
        new_idx = len(nodes) - 1

        # rewire
        for j in near:
            if j == new_idx:
                continue
            if segment_is_free(env, newp, nodes[j], step=sample_step):
                new_cost = cost[new_idx] + math.hypot(nodes[j][0] - newp[0], nodes[j][1] - newp[1])
                if new_cost + 1e-9 < cost[j]:
                    parent[j] = new_idx
                    cost[j] = float(new_cost)

        # attempt connect to goal and keep best
        if math.hypot(newp[0] - goal_xy[0], newp[1] - goal_xy[1]) <= float(step_len):
            if segment_is_free(env, newp, goal_xy, step=sample_step):
                goal_cost = cost[new_idx] + math.hypot(newp[0] - goal_xy[0], newp[1] - goal_xy[1])
                if goal_cost < best_goal_cost:
                    nodes.append(goal_xy)
                    parent.append(new_idx)
                    cost.append(float(goal_cost))
                    best_goal_idx = len(nodes) - 1
                    best_goal_cost = float(goal_cost)

    if best_goal_idx is not None:
        return reconstruct_path(nodes, parent, best_goal_idx)

    return None


# -----------------------------
# Unified planner entry point
# -----------------------------
def plan_waypoints(env, pose_xyth, goal_cell: Cell, planner: str = "astar") -> Optional[List[Point]]:
    """
    Plan a world-coordinate waypoint path to goal_cell using the selected planner.

    planner:
      - "astar"  : A* on cell grid (returns cell centers as waypoints)
      - "rrt"    : RRT in world space
      - "rrtstar": RRT* in world space
    """
    x, y, th = pose_xyth
    gx_cell, gy_cell = map(int, goal_cell)
    gx, gy = env._cell_center_world(gx_cell, gy_cell)

    start_xy = (float(x), float(y))
    goal_xy = (float(gx), float(gy))

    planner = str(planner).lower().strip()

    if planner == "astar":
        start_cell = world_to_nearest_cell(env, float(x), float(y))
        cell_path = astar_cells(env, start_cell, (gx_cell, gy_cell))
        if cell_path is None:
            return None
        return cells_to_waypoints(env, cell_path)

    if planner == "rrt":
        return rrt_plan_world(env, start_xy, goal_xy)

    if planner in ("rrtstar", "rrt*"):
        return rrtstar_plan_world(env, start_xy, goal_xy)

    raise ValueError(f"Unknown planner '{planner}'. Use: 'astar', 'rrt', 'rrtstar'.")
