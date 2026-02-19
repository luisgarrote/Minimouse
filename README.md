# Minimouse

Simple Python library for simulating micromouse maze-solving robots.

## Overview

Minimouse provides a simulation environment for micromouse robots navigating through mazes. The environment simulates a small differential-drive robot equipped with laser range sensors that must navigate from a start position to a goal position while avoiding walls.

<img width="903" height="923" alt="image" src="https://github.com/user-attachments/assets/209e6b69-4794-444f-9787-eeffdf26ba6d" />

### Features

- **Maze generation** – randomised perfect mazes via depth-first search (`generate_perfect_maze`).
- **Physics simulation** – differential-drive kinematics with collision sliding (`MicromouseEnv`).
- **Laser sensing** – configurable frontal 180° ray-cast scanner.
- **Path planners** – built-in A\*, RRT, and RRT\* planners with bundle-adjustment smoothing.
- **Controllers** – Bug2 wall-following and waypoint-following controllers.
- **Interactive UI** – canvas rendering with multiple visual themes, manual/auto control modes, and Google Colab support (`Display`).

## Installation

### From source

```bash
git clone https://github.com/luisgarrote/Minimouse.git
cd Minimouse
pip install -e .
```

### Dependencies

- `numpy >= 1.19.0`
- `ipycanvas`
- `ipywidgets`

## Quick Start

### Google Colab

```python
import minimouse
from google.colab import output
output.enable_custom_widget_manager()

disp = minimouse.Display(cells_w=5, cells_h=5, seed=3)
disp.show()
```

### Programmatic usage

```python
from minimouse.maze import generate_perfect_maze
from minimouse.env import MicromouseEnv

# Generate a maze
occ, start_cell, goal_cell = generate_perfect_maze(seed=42, cells_w=5, cells_h=5)

# Create the environment
env = MicromouseEnv(occ, start_cell, goal_cell, cell_size=1.0, n_rays=12)
obs = env.reset()

# Run a simple loop
done = False
while not done:
    vl, vr = 0.3, 0.3  # drive forward
    obs, reward, done, info = env.step((vl, vr))
```

### Using a built-in controller

```python
from minimouse.controllers import make_path_controller

controller, reset_ctrl, state = make_path_controller(env, planner="astar")
obs = env.reset()

done = False
while not done:
    vl, vr = controller(obs)
    obs, reward, done, info = env.step((vl, vr))
```

## Module Overview

| Module | Description |
|--------|-------------|
| `minimouse.env` | `MicromouseEnv` – core simulation (kinematics, sensing, collision) |
| `minimouse.maze` | `generate_perfect_maze` – random maze generation |
| `minimouse.canvas` | `CanvasRenderer` – ipycanvas-based rendering with four visual themes |
| `minimouse.display` | `Display` – full interactive UI for Colab/Jupyter |
| `minimouse.controllers` | Bug2 and path-following controller factories |
| `minimouse.planners` | A\*, RRT, RRT\* planners and bundle-adjustment smoothing |

## Documentation

- [API Reference](docs/API.md) – detailed class and function documentation.
- [Implementation Guide](docs/IMPLEMENTATION.md) – architecture overview and design notes.

## Support

For issues and questions, please use the [GitHub issue tracker](https://github.com/luisgarrote/Minimouse/issues).
