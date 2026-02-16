# Implementation Guide

This document provides guidance for implementing the MicromouseEnv class methods.

## Overview

The current version (0.1.0) provides the interface definition only. All methods raise `NotImplementedError`. This guide helps implementers understand what each method should do.

## Implementation Checklist

### Core Simulation Components

#### 1. `__init__(self, occ, start_cell, goal_cell, cell_size=1.0, n_rays=12)`

**Purpose:** Initialize the environment with maze configuration and parameters.

**Implementation steps:**
- [ ] Store occupancy grid, start/goal cells, cell_size, n_rays
- [ ] Validate inputs (check bounds, positive values)
- [ ] Initialize robot state (position, orientation, velocity)
- [ ] Set up laser ray angles (evenly distributed around robot)
- [ ] Initialize any tracking variables (steps, rewards, etc.)

**Validation needed:**
- start_cell and goal_cell must be within maze bounds
- cell_size must be > 0
- n_rays must be >= 1

---

#### 2. `reset(self) -> np.ndarray`

**Purpose:** Reset environment to initial state and return first observation.

**Implementation steps:**
- [ ] Reset robot to start_cell position
- [ ] Reset robot orientation to initial direction
- [ ] Reset any episode-specific state
- [ ] Generate and return initial observation using `_get_obs()`

**Returns:** Observation array (likely containing laser scan + robot state)

---

#### 3. `step(self, action) -> Tuple[np.ndarray, float, bool, dict]`

**Purpose:** Execute one timestep in the environment.

**Implementation steps:**
- [ ] Apply action to robot (update velocity/position/orientation)
- [ ] Use `_handle_collision()` to prevent wall penetration
- [ ] Check if goal reached (done = True)
- [ ] Calculate reward (distance to goal, collision penalty, goal bonus)
- [ ] Get new observation using `_get_obs()`
- [ ] Build info dict with diagnostic data

**Returns:** 
- observation: Current state
- reward: Scalar reward
- done: True if episode ended
- info: Dict with metadata

---

### Sensing and Observation

#### 4. `_get_obs(self) -> np.ndarray`

**Purpose:** Construct observation from current state.

**Implementation steps:**
- [ ] Get laser scan using `_laser_scan()`
- [ ] Optionally add robot state (position, velocity, orientation)
- [ ] Optionally add goal relative position
- [ ] Concatenate into single numpy array

**Typical observation components:**
- Laser distances (n_rays values)
- Robot position (x, y)
- Robot orientation (theta)
- Goal direction/distance

---

#### 5. `_laser_scan(self) -> np.ndarray`

**Purpose:** Simulate laser range sensors.

**Implementation steps:**
- [ ] Loop through each ray angle
- [ ] Call `_ray_cast(ang)` for each angle
- [ ] Return array of distances

**Returns:** Array of shape (n_rays,) with distances

---

#### 6. `_ray_cast(self, ang: float) -> float`

**Purpose:** Cast a single ray and measure distance to obstacle.

**Implementation steps:**
- [ ] Start from robot position
- [ ] Convert angle to world coordinates (robot orientation + ray angle)
- [ ] Step along ray direction
- [ ] Use `_is_wall(x, y)` to check for obstacles
- [ ] Return distance when wall hit or max range reached

**Returns:** Distance to nearest obstacle

---

### Collision and Wall Detection

#### 7. `_is_wall(self, x: float, y: float) -> bool`

**Purpose:** Check if world position is inside a wall.

**Implementation steps:**
- [ ] Convert world (x, y) to grid indices
- [ ] Check if indices are out of bounds (return True)
- [ ] Check occupancy grid at indices
- [ ] Return True if occupied, False otherwise

**Returns:** True if position is in wall

---

#### 8. `_handle_collision(self, x: float, y: float, nx: float, ny: float) -> Tuple[float, float]`

**Purpose:** Prevent robot from moving into walls.

**Implementation steps:**
- [ ] Check if new position (nx, ny) is valid using `_is_wall()`
- [ ] If valid, return (nx, ny)
- [ ] If invalid, return corrected position (could be (x, y) or slide along wall)
- [ ] Optionally implement sliding collision response

**Returns:** Valid position tuple

---

### Utility Methods

#### 9. `_cell_center_world(self, cx: int, cy: int) -> Tuple[float, float]`

**Purpose:** Convert grid cell to world coordinates.

**Implementation steps:**
- [ ] Calculate x = (cx + 0.5) * cell_size
- [ ] Calculate y = (cy + 0.5) * cell_size
- [ ] Return (x, y)

**Returns:** World coordinates of cell center

---

#### 10. `predict_lookahead(self, steps: int = 18) -> Any`

**Purpose:** Predict future states for planning.

**Implementation steps:**
- [ ] Save current state
- [ ] Run simulation forward for N steps
- [ ] Collect predicted states/observations
- [ ] Restore original state
- [ ] Return predictions

**Possible implementations:**
- Return list of future observations
- Return predicted trajectory (positions)
- Return value estimates

---

## Suggested Implementation Order

1. **Basic structure:**
   - `__init__()` - Set up state variables
   - `_cell_center_world()` - Simple coordinate conversion

2. **Wall detection:**
   - `_is_wall()` - Grid lookup

3. **Sensing:**
   - `_ray_cast()` - Single ray
   - `_laser_scan()` - Multiple rays
   - `_get_obs()` - Combine into observation

4. **Core loop:**
   - `reset()` - Initialize episode
   - `_handle_collision()` - Movement validation
   - `step()` - Main simulation step

5. **Advanced:**
   - `predict_lookahead()` - Planning support

## Testing Recommendations

1. **Unit tests for each method:**
   - Test boundary conditions
   - Test with simple known mazes
   - Verify coordinates conversions

2. **Integration tests:**
   - Full episode simulation
   - Verify observations have correct shape
   - Check rewards are reasonable

3. **Edge cases:**
   - Empty maze (all free)
   - Solid maze (all walls)
   - Robot starting in wall
   - Invalid actions

## Dependencies

The implementation will likely need:
- `numpy` - Array operations, math
- Optionally `gymnasium` - For standardized RL interface
- Optionally `matplotlib` - For visualization

## Notes for Implementers

- Consider making the class compatible with OpenAI Gym/Gymnasium interface
- Add rendering capability for visualization
- Consider continuous vs discrete action spaces
- Think about reward shaping for effective learning
- May want to add configuration options (max_steps, rewards, etc.)
