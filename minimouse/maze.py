import math, time, random, threading
import numpy as np

import ipywidgets as widgets
from IPython.display import display

from ipycanvas import Canvas, hold_canvas

import ipywidgets as widgets
from IPython.display import display, Javascript


def generate_perfect_maze(seed=3, cells_w=9, cells_h=9):
    """Generate a random perfect maze using randomised depth-first search.

    A *perfect* maze has exactly one path between any two cells (i.e. no
    loops).  The occupancy grid uses the convention where cell passages and
    walls alternate, resulting in a grid of size ``(2*cells_h+1, 2*cells_w+1)``.

    The goal cell is chosen near the centre of the maze, biased toward cells
    that are far from the start in terms of BFS distance.

    Parameters
    ----------
    seed : int, optional
        Random seed for reproducibility (default ``3``).
    cells_w : int, optional
        Number of logical columns in the maze (default ``9``).
    cells_h : int, optional
        Number of logical rows in the maze (default ``9``).

    Returns
    -------
    occ : np.ndarray
        ``uint8`` occupancy grid of shape ``(2*cells_h+1, 2*cells_w+1)``.
    start_cell : tuple of int
        ``(cx, cy)`` start cell (always ``(0, 0)``).
    goal_cell : tuple of int
        ``(cx, cy)`` chosen goal cell.
    """
    rng = random.Random(seed)

    visited = [[False]*cells_w for _ in range(cells_h)]
    W = 2*cells_w + 1
    H = 2*cells_h + 1
    occ = np.ones((H, W), dtype=np.uint8)

    def cell_to_occ(cx, cy):
        return 2*cx + 1, 2*cy + 1  # (ox, oy) in occ coords

    sx, sy = 0, 0
    stack = [(sx, sy)]
    visited[sy][sx] = True
    ox, oy = cell_to_occ(sx, sy)
    occ[oy, ox] = 0

    dirs = [(1,0), (-1,0), (0,1), (0,-1)]

    while stack:
        cx, cy = stack[-1]
        nbrs = []
        for dx, dy in dirs:
            nx, ny = cx+dx, cy+dy
            if 0 <= nx < cells_w and 0 <= ny < cells_h and not visited[ny][nx]:
                nbrs.append((nx, ny, dx, dy))
        if not nbrs:
            stack.pop()
            continue

        nx, ny, dx, dy = rng.choice(nbrs)
        visited[ny][nx] = True

        x1, y1 = cell_to_occ(cx, cy)
        x2, y2 = cell_to_occ(nx, ny)
        wx, wy = (x1+x2)//2, (y1+y2)//2
        occ[y2, x2] = 0
        occ[wy, wx] = 0

        stack.append((nx, ny))

    # pick goal near center (reachable)
    center = (cells_w/2.0, cells_h/2.0)

    # BFS distances for tie-break
    from collections import deque
    dist = [[None]*cells_w for _ in range(cells_h)]
    q = deque([(sx, sy)])
    dist[sy][sx] = 0

    def can_move(ax, ay, bx, by):
        x1, y1 = cell_to_occ(ax, ay)
        x2, y2 = cell_to_occ(bx, by)
        wx, wy = (x1+x2)//2, (y1+y2)//2
        return occ[wy, wx] == 0

    while q:
        cx, cy = q.popleft()
        for dx, dy in dirs:
            nx, ny = cx+dx, cy+dy
            if 0 <= nx < cells_w and 0 <= ny < cells_h and dist[ny][nx] is None:
                if can_move(cx, cy, nx, ny):
                    dist[ny][nx] = dist[cy][cx] + 1
                    q.append((nx, ny))

    candidates = []
    for cy in range(cells_h):
        for cx in range(cells_w):
            if dist[cy][cx] is None:
                continue
            cscore = (cx-center[0])**2 + (cy-center[1])**2
            d = dist[cy][cx]
            candidates.append((cscore, -d, cx, cy))
    candidates.sort()
    topk = min(10, len(candidates))
    pick = rng.choice(candidates[:topk])
    _, _, gx, gy = pick
    return occ, (sx, sy), (gx, gy)
