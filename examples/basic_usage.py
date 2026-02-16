"""
Basic example demonstrating how to use the MicromouseEnv interface.

This example shows the basic workflow of creating an environment,
resetting it, and taking steps. Note that the actual implementation
is not yet available, so this will raise NotImplementedError.
"""

import numpy as np
from minimouse import MicromouseEnv


def main():
    """Demonstrate basic usage of MicromouseEnv."""
    
    # Create a simple 5x5 maze
    # 0 = free space, 1 = wall
    maze = np.array([
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ])
    
    # Initialize the environment
    print("Creating Micromouse environment...")
    print(f"Maze shape: {maze.shape}")
    print(f"Start cell: (0, 0)")
    print(f"Goal cell: (4, 4)")
    
    try:
        env = MicromouseEnv(
            occ=maze,
            start_cell=(0, 0),
            goal_cell=(4, 4),
            cell_size=1.0,
            n_rays=12
        )
        
        # Reset the environment
        print("\nResetting environment...")
        obs = env.reset()
        print(f"Initial observation shape: {obs.shape}")
        
        # Take a few steps
        print("\nTaking steps...")
        for i in range(5):
            action = i % 4  # Simple action cycling
            obs, reward, done, info = env.step(action)
            print(f"Step {i+1}: reward={reward}, done={done}")
            
            if done:
                print("Episode finished!")
                break
        
        # Try lookahead prediction
        print("\nPredicting future states...")
        prediction = env.predict_lookahead(steps=10)
        print(f"Prediction: {prediction}")
        
    except NotImplementedError as e:
        print(f"\n⚠ Implementation not yet available: {e}")
        print("\nThis is expected - the interface is ready, implementation comes later!")


if __name__ == "__main__":
    main()
