# Minimouse

A Python library for simulating micromouse maze-solving robots with reinforcement learning support.

## Overview

Minimouse provides a simulation environment for micromouse robots navigating through mazes. The environment simulates a small robot equipped with laser range sensors that must navigate from a start position to a goal position while avoiding walls.

## Installation

### From source

```bash
git clone https://github.com/luisgarrote/Minimouse.git
cd Minimouse
pip install -e .
```

### For development

```bash
pip install -e ".[dev]"
```

## Quick Start

```python
import numpy as np
from minimouse import MicromouseEnv

# Create a simple 5x5 maze (0 = free space, 1 = wall)
maze = np.array([
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0]
])

# Initialize environment
env = MicromouseEnv(
    occ=maze,
    start_cell=(0, 0),
    goal_cell=(4, 4),
    cell_size=1.0,
    n_rays=12
)

# Reset environment
obs = env.reset()

# Take a step
obs, reward, done, info = env.step(action=0)
```

## API Reference

### MicromouseEnv

The main environment class for micromouse simulation.

#### Constructor

```python
MicromouseEnv(occ, start_cell, goal_cell, cell_size=1.0, n_rays=12)
```

**Parameters:**
- `occ` (np.ndarray): 2D occupancy grid representing the maze (1 = wall, 0 = free space)
- `start_cell` (tuple): Starting cell coordinates as (row, col)
- `goal_cell` (tuple): Goal cell coordinates as (row, col)
- `cell_size` (float, optional): Size of each cell in meters. Default: 1.0
- `n_rays` (int, optional): Number of laser rays for sensing. Default: 12

#### Methods

##### reset()

Reset the environment to the initial state.

**Returns:**
- `np.ndarray`: Initial observation containing sensor readings

```python
obs = env.reset()
```

##### step(action)

Execute one time step within the environment.

**Parameters:**
- `action`: The action to take (format depends on action space)

**Returns:**
- `observation` (np.ndarray): New observation after taking the action
- `reward` (float): Reward received for this step
- `done` (bool): Whether the episode has ended
- `info` (dict): Additional diagnostic information

```python
obs, reward, done, info = env.step(action=0)
```

##### predict_lookahead(steps=18)

Predict future states by looking ahead a certain number of steps.

**Parameters:**
- `steps` (int, optional): Number of steps to look ahead. Default: 18

**Returns:**
- Predicted future state information

```python
prediction = env.predict_lookahead(steps=20)
```

### Private Methods

The following methods are internal to the environment and not intended for direct use:

- `_cell_center_world(cx, cy)`: Convert cell coordinates to world coordinates
- `_get_obs()`: Get current observation from the environment
- `_laser_scan()`: Perform a laser scan from robot's current position
- `_ray_cast(ang)`: Cast a single ray at a given angle
- `_is_wall(x, y)`: Check if a position is inside a wall
- `_handle_collision(x, y, nx, ny)`: Handle collision detection and response

## Features

- **Laser Range Sensing**: Simulates laser range sensors with configurable number of rays
- **Collision Detection**: Handles wall collisions realistically
- **Customizable Mazes**: Support for arbitrary maze configurations via occupancy grids
- **Reinforcement Learning Ready**: Compatible with standard RL interfaces
- **Lookahead Prediction**: Built-in support for planning with state prediction

## Use Cases

- Training reinforcement learning agents for maze navigation
- Testing path planning algorithms
- Micromouse competition simulation
- Robotics education and research

## Requirements

- Python 3.7+
- NumPy 1.19.0+

## License

MIT License

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Support

For issues and questions, please use the [GitHub issue tracker](https://github.com/luisgarrote/Minimouse/issues).