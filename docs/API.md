# API Documentation

## MicromouseEnv Class

The `MicromouseEnv` class is the core component of the Minimouse library. It provides a simulation environment for a micromouse robot navigating through a maze.

### Class Definition

```python
class MicromouseEnv:
    def __init__(self, occ, start_cell, goal_cell, cell_size=1.0, n_rays=12):
        ...
```

### Parameters

#### Constructor Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `occ` | `np.ndarray` | Required | 2D occupancy grid representing the maze (1 = wall, 0 = free space) |
| `start_cell` | `Tuple[int, int]` | Required | Starting cell coordinates as (row, col) |
| `goal_cell` | `Tuple[int, int]` | Required | Goal cell coordinates as (row, col) |
| `cell_size` | `float` | 1.0 | Size of each cell in meters |
| `n_rays` | `int` | 12 | Number of laser rays for sensing |

### Public Methods

#### reset()

Resets the environment to the initial state.

**Signature:**
```python
def reset(self) -> np.ndarray
```

**Returns:**
- `np.ndarray`: Initial observation containing sensor readings

**Example:**
```python
env = MicromouseEnv(maze, (0, 0), (4, 4))
obs = env.reset()
```

---

#### step(action)

Executes one time step within the environment.

**Signature:**
```python
def step(self, action: Any) -> Tuple[np.ndarray, float, bool, dict]
```

**Parameters:**
- `action`: The action to take (format depends on action space)

**Returns:**
- `observation` (`np.ndarray`): New observation after taking the action
- `reward` (`float`): Reward received for this step
- `done` (`bool`): Whether the episode has ended
- `info` (`dict`): Additional diagnostic information

**Example:**
```python
obs, reward, done, info = env.step(action=0)
if done:
    print("Episode completed!")
```

---

#### predict_lookahead(steps=18)

Predicts future states by looking ahead a certain number of steps.

**Signature:**
```python
def predict_lookahead(self, steps: int = 18) -> Any
```

**Parameters:**
- `steps` (`int`, optional): Number of steps to look ahead. Default: 18

**Returns:**
- Predicted future state information

**Example:**
```python
prediction = env.predict_lookahead(steps=20)
```

### Private Methods

The following methods are internal implementation details and should not be called directly by users:

#### _cell_center_world(cx, cy)

Converts cell coordinates to world coordinates (center of cell).

**Signature:**
```python
def _cell_center_world(self, cx: int, cy: int) -> Tuple[float, float]
```

---

#### _get_obs()

Gets the current observation from the environment.

**Signature:**
```python
def _get_obs(self) -> np.ndarray
```

---

#### _laser_scan()

Performs a laser scan from the robot's current position.

**Signature:**
```python
def _laser_scan(self) -> np.ndarray
```

---

#### _ray_cast(ang)

Casts a single ray at a given angle and returns the distance to obstacle.

**Signature:**
```python
def _ray_cast(self, ang: float) -> float
```

---

#### _is_wall(x, y)

Checks if a given world position is inside a wall.

**Signature:**
```python
def _is_wall(self, x: float, y: float) -> bool
```

---

#### _handle_collision(x, y, nx, ny)

Handles collision detection and response.

**Signature:**
```python
def _handle_collision(self, x: float, y: float, nx: float, ny: float) -> Tuple[float, float]
```

### Usage Patterns

#### Basic Setup

```python
import numpy as np
from minimouse import MicromouseEnv

# Create a maze
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
```

#### Training Loop

```python
# Reset environment
obs = env.reset()

done = False
total_reward = 0

while not done:
    # Select action (example: random action)
    action = select_action(obs)
    
    # Take step
    obs, reward, done, info = env.step(action)
    total_reward += reward

print(f"Episode finished with total reward: {total_reward}")
```

#### With Lookahead

```python
# Get current state
obs = env.reset()

# Predict future states
prediction = env.predict_lookahead(steps=10)

# Use prediction for planning
action = plan_with_lookahead(obs, prediction)

# Execute action
obs, reward, done, info = env.step(action)
```

### Error Handling

The environment may raise the following exceptions:

- `ValueError`: If start_cell or goal_cell are outside maze bounds
- `ValueError`: If cell_size is not positive
- `ValueError`: If n_rays is less than 1
- `NotImplementedError`: Currently raised by all methods as implementation is pending

### Notes

- The current version (0.1.0) provides the interface only. Implementation will be added in future versions.
- All public methods currently raise `NotImplementedError`
- The interface is stable and can be used for type checking and API design
