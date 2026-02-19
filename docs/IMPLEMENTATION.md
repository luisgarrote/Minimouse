# Implementation Guide

This document describes the architecture and design decisions behind the
Minimouse library.

## Architecture Overview

```
minimouse/
├── __init__.py        # Public API (MicromouseEnv, CanvasRenderer, Display)
├── maze.py            # Perfect maze generation (DFS)
├── env.py             # Core simulation environment
├── canvas.py          # ipycanvas renderer (4 visual themes)
├── display.py         # Interactive Colab/Jupyter UI
├── controllers.py     # Bug2, waypoint follower, path-planning controller
└── planners.py        # A*, RRT, RRT*, bundle-adjustment smoothing
```

### Data Flow

1. **`maze.py`** generates an occupancy grid and start/goal cells.
2. **`env.py`** wraps the grid in a simulation with differential-drive
   kinematics, laser sensing, and collision handling.
3. **`planners.py`** operates on the occupancy grid to produce waypoint paths.
4. **`controllers.py`** consumes observations and (optionally) planned
   waypoints to produce `(vl, vr)` wheel commands.
5. **`canvas.py`** renders the environment state onto an `ipycanvas.Canvas`.
6. **`display.py`** ties everything together with interactive widgets and a
   tick-driven simulation loop.

---

## Module Details

### maze.py – Maze Generation

`generate_perfect_maze` uses a randomised DFS (recursive backtracker) to carve
passages through an initially solid grid.

- **Grid convention** – the occupancy array has dimensions
  `(2·cells_h + 1, 2·cells_w + 1)`.  Odd-indexed rows/columns hold cell
  centres; even-indexed rows/columns are walls or passages.
- **Goal selection** – a BFS from the start cell computes distances.  The goal
  is picked from the 10 cells closest to the maze centre, biased toward higher
  BFS distance.

### env.py – MicromouseEnv

The core simulation loop runs at a fixed `dt = 0.05` time-step.

- **Kinematics** – standard differential-drive model:
  `v = (vr + vl) / 2`, `ω = (vr − vl) / wheel_base`.
- **Collision** – axis-aligned sliding: if the new position is in a wall, each
  axis is tried independently before falling back to the original position.
- **Laser** – `n_rays` rays are cast over a 180° frontal arc
  (`−π/2 … +π/2` relative to heading) using a fixed step-size ray march
  (`0.05 × cell_size`).  Max range is `8 × cell_size`.
- **Observation** – a dictionary with keys `"pose"` (x, y, θ),
  `"laser"` (n_rays floats), and `"goal_cell"` (cx, cy).
- **Done condition** – the episode ends when the robot is within
  `0.45 × cell_size` of the goal cell centre.

### canvas.py – CanvasRenderer

Converts world coordinates to pixel coordinates (y-axis flipped) and draws
the maze, robot, goal, laser rays, trajectory, planned waypoints, and
lookahead prediction.

Four rendering modes are supported:

| Mode | Style | Notes |
|------|-------|-------|
| 0 | Classic | White floor, black walls |
| 1 | Dungeon | Stone-textured walls, torch glow, fog of war |
| 2 | Sci-fi | Light grey panels, panel seams |
| 3 | Cyber | Dark background, cyan neon outlines, fog of war |

### display.py – Display

The `Display` class is designed for **Google Colab** and uses `ipywidgets`
buttons, sliders, and dropdowns.

- A hidden tick button is clicked periodically by a JavaScript `setInterval`
  loop (at the configured FPS) to drive `step_once()`.
- Keyboard input (WASD) is captured via a JavaScript listener registered
  through `google.colab.output`.
- Controllers can be swapped at runtime by changing
  `display.callbacks["user_controller"]`.

### controllers.py – Controllers

Two controller factories are provided:

1. **`make_bug2_controller`** – reactive Bug2 algorithm:
   - *Go-to-goal* mode steers directly toward the goal.
   - On frontal obstacle detection, switches to *wall-follow* (left-hand rule).
   - Returns to go-to-goal when near the M-line and closer to the goal than
     at the hit point.

2. **`make_path_controller`** – plan-and-follow:
   - Plans a path via the selected planner (`astar`, `rrt`, or `rrtstar`).
   - Smooths the path with `bundle_adjust_smooth`.
   - Follows waypoints using `follow_waypoints` (proportional steering with
     adaptive speed).

Both factories return `(controller_fn, reset_fn, ...)` matching the
`controller(obs) → (vl, vr)` interface used by `Display`.

### planners.py – Planners & Smoothing

- **A\*** – operates on the cell graph (4-connected).  Uses Manhattan distance
  as the heuristic.
- **RRT** – samples random collision-free points in world space, extends the
  nearest node, and connects to the goal when close enough.
- **RRT\*** – extends RRT with neighbour rewiring within a fixed radius to find
  lower-cost paths.
- **Bundle-adjustment smoothing** – iteratively pulls internal waypoints toward
  the average of their neighbours, only accepting moves that keep the path
  collision-free.

---

## Coordinate Conventions

| Concept | Convention |
|---------|-----------|
| Cell indices | `(cx, cy)` – column, row in the logical maze grid |
| Occupancy grid | `occ[iy, ix]` – row-major NumPy array |
| World frame | Origin at bottom-left; x→right, y→up |
| Canvas frame | Origin at top-left; x→right, y→down (y is flipped) |
| Robot heading | `theta` in radians, 0 = east, positive = counter-clockwise |
| Laser angles | `[−π/2, +π/2]` relative to heading; positive = left |

## Dependencies

| Package | Purpose |
|---------|---------|
| `numpy` | Array operations, math |
| `ipycanvas` | Canvas rendering |
| `ipywidgets` | Interactive UI controls |

Optional (for Google Colab):
- `google.colab.output` – keyboard callback registration
