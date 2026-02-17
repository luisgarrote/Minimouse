class MicromouseEnv:
    def __init__(self, occ, start_cell, goal_cell, cell_size=1.0, n_rays=12):
        self.occ = np.array(occ, dtype=np.uint8)
        self.cell_size = float(cell_size)
        self.n_rays = int(n_rays)

        self.H, self.W = self.occ.shape

        # robot params
        self.radius = 0.18 * self.cell_size
        self.wheel_base = 0.35 * self.cell_size
        self.dt = 0.05

        # laser params
        self.max_range = 8.0 * self.cell_size

        self.start_cell = start_cell
        self.goal_cell = goal_cell

        self.trajectory = []
        self.last_action = (0.0, 0.0)

        self.reset()

    def _cell_center_world(self, cx, cy):
        # cell centers correspond to occ coords (2*cx+1, 2*cy+1)
        ox = (2*cx + 1) * self.cell_size + 0.5*self.cell_size
        oy = (2*cy + 1) * self.cell_size + 0.5*self.cell_size
        return ox, oy

    def reset(self):
        sx, sy = self.start_cell
        self.x, self.y = self._cell_center_world(sx, sy)
        self.theta = 0.0
        self.last_action = (0.0, 0.0)
        self.trajectory = [(self.x, self.y)]
        return self._get_obs()

    def step(self, action):
        vl, vr = float(action[0]), float(action[1])
        self.last_action = (vl, vr)

        v = 0.5 * (vr + vl)
        w = (vr - vl) / self.wheel_base

        nx = self.x + v * math.cos(self.theta) * self.dt
        ny = self.y + v * math.sin(self.theta) * self.dt
        ntheta = self.theta + w * self.dt

        nx, ny = self._handle_collision(self.x, self.y, nx, ny)

        self.x, self.y, self.theta = nx, ny, ntheta
        self.trajectory.append((self.x, self.y))

        obs = self._get_obs()

        gx, gy = self._cell_center_world(*self.goal_cell)
        done = ((self.x-gx)**2 + (self.y-gy)**2) <= (0.45*self.cell_size)**2
        reward = 1.0 if done else 0.0
        info = {"done_reason": "goal" if done else None}
        return obs, reward, done, info

    def _get_obs(self):
        return {
            "pose": np.array([self.x, self.y, self.theta], dtype=np.float32),
            "laser": self._laser_scan().astype(np.float32),
            "goal_cell": np.array(self.goal_cell, dtype=np.int32),
        }

    # ---------- Laser ----------
    def _laser_scan(self):
        angles = np.linspace(-math.pi/2, math.pi/2, self.n_rays)
        dists = np.empty(self.n_rays, dtype=np.float32)
        for i, a in enumerate(angles):
            dists[i] = self._ray_cast(self.theta + a)
        return dists

    def _ray_cast(self, ang):
        step = 0.05 * self.cell_size
        for r in np.arange(0.0, self.max_range + 1e-9, step):
            px = self.x + r * math.cos(ang)
            py = self.y + r * math.sin(ang)
            if self._is_wall(px, py):
                return r
        return self.max_range

    # ---------- Collision (with sliding) ----------
    def _is_wall(self, x, y):
        ix = int(x / self.cell_size)
        iy = int(y / self.cell_size)
        if ix < 0 or iy < 0 or ix >= self.W or iy >= self.H:
            return True
        return self.occ[iy, ix] == 1

    def _handle_collision(self, x, y, nx, ny):
        if not self._is_wall(nx, ny):
            return nx, ny
        if not self._is_wall(nx, y):
            return nx, y
        if not self._is_wall(x, ny):
            return x, ny
        return x, y

    # ---------- Lookahead ----------
    def predict_lookahead(self, steps=18):
        vl, vr = self.last_action
        x, y, th = self.x, self.y, self.theta
        pts = [(x, y)]
        for _ in range(int(steps)):
            v = 0.5 * (vr + vl)
            w = (vr - vl) / self.wheel_base
            nx = x + v * math.cos(th) * self.dt
            ny = y + v * math.sin(th) * self.dt
            nth = th + w * self.dt
            nx, ny = self._handle_collision(x, y, nx, ny)
            x, y, th = nx, ny, nth
            pts.append((x, y))
        return pts
