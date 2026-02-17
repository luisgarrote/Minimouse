import math, time, random, threading
import numpy as np

import ipywidgets as widgets
from IPython.display import display

from ipycanvas import Canvas, hold_canvas

import ipywidgets as widgets
from IPython.display import display, Javascript


def generate_perfect_maze(seed=0, cells_w=9, cells_h=9):
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
