# API Reference

## Table of Contents

- [minimouse.env – MicromouseEnv](#micromouseenv-class)
- [minimouse.maze – Maze Generation](#maze-generation)
- [minimouse.canvas – CanvasRenderer](#canvasrenderer-class)
- [minimouse.display – Display](#display-class)
- [minimouse.controllers – Controllers](#controllers)
- [minimouse.planners – Planners & Smoothing](#planners--smoothing)

---

## MicromouseEnv Class

`minimouse.env.MicromouseEnv`

Core simulation environment for a differential-drive micromouse robot navigating
an occupancy-grid maze.

### Constructor

```python
MicromouseEnv(occ, start_cell, goal_cell, cell_size=1.0, n_rays=12)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `occ` | array-like | *required* | 2-D occupancy grid (H × W), 1 = wall, 0 = free |
| `start_cell` | `(int, int)` | *required* | Logical cell `(cx, cy)` for the start position |
| `goal_cell` | `(int, int)` | *required* | Logical cell `(cx, cy)` for the goal position |
| `cell_size` | `float` | `1.0` | Side length of a cell in world units |
| `n_rays` | `int` | `12` | Number of laser rays (180° frontal arc) |

### Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `occ` | `np.ndarray` | Occupancy grid (`uint8`) |
| `H`, `W` | `int` | Grid dimensions (rows, columns) |
| `x`, `y`, `theta` | `float` | Current robot pose (world frame) |
| `radius` | `float` | Robot body radius (`0.18 × cell_size`) |
| `wheel_base` | `float` | Distance between wheels (`0.35 × cell_size`) |
| `dt` | `float` | Simulation time-step (`0.05`) |
| `max_range` | `float` | Maximum laser range (`8.0 × cell_size`) |
| `trajectory` | `list` | History of `(x, y)` positions |
| `waypoints` | `list` or `None` | External planner waypoints |
| `last_action` | `(float, float)` | Most recent `(vl, vr)` command |

### Public Methods

#### `reset() → dict`

Reset the robot to the start cell and clear the trajectory.

**Returns:** observation dictionary (see `_get_obs()`).

```python
obs = env.reset()
```

---

#### `step(action) → (dict, float, bool, dict)`

Advance the simulation by one time-step.

| Parameter | Type | Description |
|-----------|------|-------------|
| `action` | `(float, float)` | `(vl, vr)` left/right wheel velocities |

**Returns:**

| Value | Type | Description |
|-------|------|-------------|
| `obs` | `dict` | New observation |
| `reward` | `float` | `1.0` at goal, `0.0` otherwise |
| `done` | `bool` | `True` when within goal tolerance |
| `info` | `dict` | `{"done_reason": "goal" \| None}` |

```python
obs, reward, done, info = env.step((0.3, 0.3))
```

---

#### `predict_lookahead(steps=18) → list`

Forward-simulate the last action without modifying the actual state.

**Returns:** list of `(x, y)` positions (including the current one).

```python
future_path = env.predict_lookahead(steps=20)
```

### Observation Dictionary

Returned by `reset()` and `step()`:

```python
{
    "pose":      np.float32 array [x, y, theta],
    "laser":     np.float32 array of n_rays range values,
    "goal_cell": np.int32 array [cx, cy],
}
```

### Private Methods

| Method | Description |
|--------|-------------|
| `_cell_center_world(cx, cy)` | Convert cell indices → world `(x, y)` |
| `_get_obs()` | Build the observation dictionary |
| `_laser_scan()` | Perform a full frontal laser scan |
| `_ray_cast(ang)` | Cast one ray and return hit distance |
| `_is_wall(x, y)` | Check whether a world point is inside a wall |
| `_handle_collision(x, y, nx, ny)` | Resolve collisions with axis sliding |

---

## Maze Generation

`minimouse.maze.generate_perfect_maze`

```python
generate_perfect_maze(seed=3, cells_w=9, cells_h=9) → (occ, start_cell, goal_cell)
```

Generate a random perfect maze (no loops) using randomised depth-first search.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `seed` | `int` | `3` | Random seed |
| `cells_w` | `int` | `9` | Number of logical columns |
| `cells_h` | `int` | `9` | Number of logical rows |

**Returns:**

| Value | Type | Description |
|-------|------|-------------|
| `occ` | `np.ndarray` | Occupancy grid of shape `(2·cells_h+1, 2·cells_w+1)` |
| `start_cell` | `(int, int)` | Always `(0, 0)` |
| `goal_cell` | `(int, int)` | Cell near the maze centre (biased by BFS distance) |

```python
from minimouse.maze import generate_perfect_maze
occ, start, goal = generate_perfect_maze(seed=42, cells_w=5, cells_h=5)
```

---

## CanvasRenderer Class

`minimouse.canvas.CanvasRenderer`

Renders a `MicromouseEnv` onto an `ipycanvas.Canvas`.

### Constructor

```python
CanvasRenderer(canvas, env, px_per_cell=18)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `canvas` | `ipycanvas.Canvas` | *required* | Target canvas widget |
| `env` | `MicromouseEnv` | *required* | Environment to draw |
| `px_per_cell` | `int` | `18` | Pixels per cell side |

### Methods

#### `world_to_px(x, y) → (float, float)`

Convert world coordinates to canvas pixel coordinates (y-axis flipped).

#### `draw(show_laser=True, show_trajectory=True, show_lookahead=True, mode=1, occlusion=0)`

Redraw the full scene.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `show_laser` | `bool` | `True` | Overlay laser rays |
| `show_trajectory` | `bool` | `True` | Show the robot path history |
| `show_lookahead` | `bool` | `True` | Show predicted future path |
| `mode` | `int` | `1` | Visual theme: 0 classic, 1 dungeon, 2 sci-fi, 3 cyber |
| `occlusion` | `float` | `0` | Fog-of-war intensity (0–1, for dungeon/cyber) |

---

## Display Class

`minimouse.display.Display`

Interactive UI wrapper combining environment, renderer, and widgets for
Google Colab / Jupyter notebooks.

### Constructor

```python
Display(
    make_env_fn=None,
    user_controller_fn=None,
    reset_hook_fn=None,
    on_status_fn=None,
    px_per_cell=64,
    fps=20,
    cells_w=4,
    cells_h=4,
    n_rays=12,
    seed=3,
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `make_env_fn` | callable | `None` | Env factory `(seed, cells_w, cells_h, n_rays) → env` |
| `user_controller_fn` | callable | `None` | `controller(obs) → (vl, vr)` |
| `reset_hook_fn` | callable | `None` | Called on reset to clear controller state |
| `on_status_fn` | callable | `None` | `on_status(msg)` status callback |
| `px_per_cell` | `int` | `64` | Pixels per cell |
| `fps` | `float` | `20` | Target tick rate |
| `cells_w` | `int` | `4` | Maze width |
| `cells_h` | `int` | `4` | Maze height |
| `n_rays` | `int` | `12` | Laser rays |
| `seed` | `int` | `3` | Random seed |

### Key Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `env` | `MicromouseEnv` | Active environment instance |
| `canvas` | `Canvas` | ipycanvas widget |
| `renderer` | `CanvasRenderer` | Renderer bound to canvas/env |
| `callbacks` | `dict` | Mutable map of hook functions (see below) |
| `control` | `dict` | Live state: `vl`, `vr`, `running`, `mode` |

### Callbacks Dictionary

| Key | Signature | Purpose |
|-----|-----------|---------|
| `"make_env"` | `(seed, cells_w, cells_h, n_rays) → env` | Environment factory |
| `"user_controller"` | `(obs) → (vl, vr)` | Auto-mode controller |
| `"reset_hook"` | `() → None` | Controller state reset |
| `"on_status"` | `(msg) → None` | Status bar notification |
| `"on_time"` | `(ticks) → None` | Timer notification |
| `"on_step_end"` | `(display) → None` | After each simulation step |
| `"on_newmaze"` | `(display, seed, n_rays) → None` | After generating a new maze |

### Public Methods

#### `show() → Display`

Render the canvas and widgets, install keyboard handlers and the tick loop.

#### `draw()`

Redraw the canvas with current toggle/slider values.

#### `step_once()`

Execute one simulation step and refresh the display.

#### `set_status(msg)`

Update the status bar text.

#### `stop_motion()`

Set both wheel velocities to zero.

### Control Modes

| Mode string | Description |
|-------------|-------------|
| `"manual"` | Drive with WASD buttons/keyboard |
| `"auto"` | Use `callbacks["user_controller"]` |
| `"autobug2"` | Built-in Bug2 wall-following controller |
| `"autoAstar"` | Built-in A\* planner + waypoint follower |
| `"autoRRTstar"` | Built-in RRT\* planner + waypoint follower |

---

## Controllers

`minimouse.controllers`

### `make_bug2_controller(env) → (controller, reset)`

Bug2 reactive controller that alternates between *go-to-goal* and
*wall-following* modes using an M-line leave condition.

```python
controller, reset_ctrl = make_bug2_controller(env)
vl, vr = controller(obs)
```

### `follow_waypoints(env, pose_xyth, waypoints, wp_idx) → (vl, vr, new_idx)`

Simple waypoint follower: steer toward the current waypoint and advance when close.

### `make_path_controller(env, planner="astar") → (controller, reset, state)`

Plan-and-follow controller. Plans a path on first call (or when the goal changes),
smooths it, then follows the waypoints.

| `planner` value | Algorithm |
|-----------------|-----------|
| `"astar"` | A\* on cell grid |
| `"rrt"` | RRT in world space |
| `"rrtstar"` | RRT\* in world space |

```python
controller, reset_ctrl, state = make_path_controller(env, planner="astar")
vl, vr = controller(obs)
```

### Helper Functions

| Function | Description |
|----------|-------------|
| `angle_wrap(a)` | Wrap angle to `[−π, π]` |
| `dist(p, q)` | Euclidean distance between two points |
| `compute_m_line(start, goal)` | Represent the M-line as `(start, goal)` |
| `point_line_dist(p, line)` | Distance from point to infinite line |

---

## Planners & Smoothing

`minimouse.planners`

### Planners

#### `astar_cells(env, start_cell, goal_cell) → list or None`

A\* search on the 4-connected cell graph.  Returns a list of `(cx, cy)` cells or `None`.

#### `rrt_plan_world(env, start_xy, goal_xy, ...) → list or None`

Basic RRT planner in world coordinates.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `step_len` | `0.45 × cell_size` | Extension step size |
| `goal_sample_rate` | `0.15` | Probability of sampling the goal |
| `max_iters` | `3000` | Maximum iterations |

#### `rrtstar_plan_world(env, start_xy, goal_xy, ...) → list or None`

RRT\* planner with neighbour rewiring.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `step_len` | `0.45 × cell_size` | Extension step |
| `goal_sample_rate` | `0.15` | Goal bias |
| `max_iters` | `4000` | Maximum iterations |
| `neighbor_radius` | `1.2 × cell_size` | Rewiring radius |

#### `plan_waypoints(env, pose_xyth, goal_cell, planner="astar") → list or None`

Unified entry point — selects a planner and returns world-coordinate waypoints.

### Smoothing

#### `bundle_adjust_smooth(env, waypoints, iters=25, alpha=0.4) → list or None`

Iteratively pulls internal waypoints toward neighbour averages while keeping
the path collision-free. A lightweight "bundle adjustment"-style smoother.

#### `segment_is_free(env, p0, p1, step) → bool`

Check that a straight segment is collision-free by dense sampling.

### Utility Functions

| Function | Description |
|----------|-------------|
| `cells_to_waypoints(env, cell_path)` | Convert cell path → world waypoints |
| `world_to_nearest_cell(env, x, y)` | Approximate world position → cell indices |
| `can_move_cell(env, ax, ay, bx, by)` | Check if the wall between two cells is open |
| `sample_free_point_world(env)` | Uniform random collision-free point |
| `nearest_node(nodes, p)` | Index of nearest node (linear scan) |
| `steer(a, b, step)` | Move from `a` toward `b` by at most `step` |
| `reconstruct_path(nodes, parent, goal_idx)` | Reconstruct path from parent pointers |
