import math, time, random, threading
import numpy as np

import ipywidgets as widgets
from IPython.display import display

from ipycanvas import Canvas, hold_canvas

import ipywidgets as widgets
from IPython.display import display, Javascript



class CanvasRenderer:
    def __init__(self, canvas, env, px_per_cell=18):
        self.canvas = canvas
        self.env = env
        self.px_per_cell = int(px_per_cell)

        self.scale = self.px_per_cell / env.cell_size  # px per world unit


    def world_to_px(self, x, y):
        # origin at bottom-left in env; canvas origin is top-left
        px = x * self.scale
        py = self.canvas.height - (y * self.scale)
        return px, py

    def draw(self, show_laser=True, show_trajectory=True, show_lookahead=True,
            mode=1, occlusion=0):

        c = self.canvas
        e = self.env

        # ---------- helpers ----------
        def noise(x, y):
            return (math.sin(x*12.9898 + y*78.233) * 43758.5453) % 1

        def stone_color(px, py):
            n = noise(px*0.05, py*0.05)
            v = int(60 + n*50)
            return f"rgb({v},{v},{v})"

        def draw_goal_glow(cx, cy):
            g = c.create_radial_gradient(
                cx, cy, 2,
                cx, cy, 45,
                [
                    (0.0, "rgba(255,255,160,0.9)"),
                    (0.5, "rgba(255,200,0,0.5)"),
                    (1.0, "rgba(255,200,0,0)")
                ]
            )
            c.fill_style = g
            c.fill_rect(cx-45, cy-45, 90, 90)


        def draw_torch(cx, cy):
            g = c.create_radial_gradient(
                cx, cy, 10,
                cx, cy, 140,
                [
                    (0.0, "rgba(255,220,150,0.45)"),
                    (0.4, "rgba(255,180,80,0.25)"),
                    (1.0, "rgba(0,0,0,0)")
                ]
            )
            c.fill_style = g
            c.fill_rect(cx-140, cy-140, 280, 280)

        def draw_fog(cx, cy):
            g = c.create_radial_gradient(
                cx, cy, 40,
                cx, cy, 160,
                [
                    (0.0, "rgba(0,0,0,0)"),
                    (0.6, "rgba(0,0,0,0.55)"),
                    (1.0, "rgba(0,0,0,0.85)")
                ]
            )
            c.fill_style = g
            c.fill_rect(0, 0, c.width, c.height)

        with hold_canvas(c):
            # -------------------------------------------------
            # CLEAR
            # -------------------------------------------------
            c.clear_rect(0, 0, c.width, c.height)

            # -------------------------------------------------
            # FLOOR TILING
            # -------------------------------------------------
            for iy in range(e.H):
                for ix in range(e.W):
                    x = ix * e.cell_size
                    y = iy * e.cell_size
                    px, py = self.world_to_px(x, y + e.cell_size)
                    s = e.cell_size * self.scale

                    if mode == 1:      # dungeon
                        c.fill_style = "#2a2a2a"
                    elif mode == 2:    # sci-fi
                        c.fill_style = "#e6e8ec"
                    elif mode == 3:    # cyber
                        c.fill_style = "#05060a"
                    else:             # classic
                        c.fill_style = "#ffffff"

                    c.fill_rect(px, py, s, s)

                    # floor seams for sci-fi & cyber
                    if mode in (2, 3):
                        c.stroke_style = "rgba(0,0,0,0.15)" if mode == 2 else "rgba(0,255,255,0.15)"
                        c.line_width = 1
                        c.stroke_rect(px, py, s, s)

            # -------------------------------------------------
            # WALLS
            # -------------------------------------------------
            for iy in range(e.H):
                for ix in range(e.W):
                    if e.occ[iy, ix] != 1:
                        continue

                    x = ix * e.cell_size
                    y = iy * e.cell_size
                    px, py = self.world_to_px(x, y + e.cell_size)
                    s = e.cell_size * self.scale

                    # ---------- CLASSIC ----------
                    if mode == 0:
                        c.fill_style = "black"
                        c.fill_rect(px, py, s, s)

                    # ---------- DUNGEON ----------
                    elif mode == 1:
                        c.fill_style = stone_color(px, py)
                        c.fill_rect(px, py, s, s)

                        # bevel
                        c.fill_style = "rgba(255,255,255,0.12)"
                        c.fill_rect(px, py, s, 3)
                        c.fill_rect(px, py, 3, s)
                        c.fill_style = "rgba(0,0,0,0.25)"
                        c.fill_rect(px, py+s-3, s, 3)
                        c.fill_rect(px+s-3, py, 3, s)

                    # ---------- SCI-FI ----------
                    elif mode == 2:
                        c.fill_style = "#cfd3da"
                        c.fill_rect(px, py, s, s)
                        c.stroke_style = "#9aa1ac"
                        c.line_width = 1
                        c.stroke_rect(px+2, py+2, s-4, s-4)

                    # ---------- CYBER ----------
                    elif mode == 3:
                        c.fill_style = "#0a0d12"
                        c.fill_rect(px, py, s, s)
                        c.stroke_style = "rgba(0,255,255,0.6)"
                        c.line_width = 2
                        c.stroke_rect(px+1, py+1, s-2, s-2)

            # -------------------------------------------------
            # GOAL (with glow)
            # -------------------------------------------------
            gx, gy = e._cell_center_world(*e.goal_cell)
            gpx, gpy = self.world_to_px(gx, gy)

            if mode in (1,2,3):
                draw_goal_glow(gpx, gpy)

            c.stroke_style = "gold"
            c.line_width = 2
            c.stroke_rect(
                gpx - 0.5*e.cell_size*self.scale,
                gpy - 0.5*e.cell_size*self.scale,
                e.cell_size*self.scale,
                e.cell_size*self.scale
            )
            c.fill_style = "gold"
            c.fill_circle(gpx, gpy, 5)

            # -------------------------------------------------
            # TRAJECTORY
            # -------------------------------------------------
            if show_trajectory and len(e.trajectory) >= 2:
                c.stroke_style = "green"
                c.line_width = 2
                c.set_line_dash([])
                pts = [self.world_to_px(x, y) for (x, y) in e.trajectory]
                c.begin_path()
                c.move_to(*pts[0])
                for p in pts[1:]:
                    c.line_to(*p)
                c.stroke()

            print(e.waypoints)
            if e.waypoints and len(e.waypoints) >= 2:
                c.stroke_style = "#00FFFF"
                c.line_width = 2
                c.set_line_dash([])
                pts = [self.world_to_px(x, y) for (x, y) in e.waypoints]
                c.begin_path()
                c.move_to(*pts[0])
                for p in pts[1:]:
                    c.line_to(*p)
                c.stroke()
            # -------------------------------------------------
            # LOOKAHEAD
            # -------------------------------------------------
            if show_lookahead:
                la = e.predict_lookahead(steps=18)
                pts = [self.world_to_px(x, y) for (x, y) in la]
                c.stroke_style = "lime"
                c.line_width = 2
                c.set_line_dash([6, 6])
                c.begin_path()
                c.move_to(*pts[0])
                for p in pts[1:]:
                    c.line_to(*p)
                c.stroke()
                c.set_line_dash([])

            # -------------------------------------------------
            # ROBOT
            # -------------------------------------------------
            rx, ry = self.world_to_px(e.x, e.y)
            c.fill_style = "blue"
            c.fill_circle(rx, ry, e.radius*self.scale)

            # heading
            hx = e.x + (e.radius*1.8) * math.cos(e.theta)
            hy = e.y + (e.radius*1.8) * math.sin(e.theta)
            hpx, hpy = self.world_to_px(hx, hy)
            c.stroke_style = "cyan"
            c.line_width = 2
            c.begin_path()
            c.move_to(rx, ry)
            c.line_to(hpx, hpy)
            c.stroke()

            # -------------------------------------------------
            # LASERS
            # -------------------------------------------------
            if show_laser:
                rays = e._laser_scan()
                angles = np.linspace(-math.pi/2, math.pi/2, e.n_rays)
                c.stroke_style = "rgba(255,0,0,0.35)"
                c.line_width = 1
                for d, a in zip(rays, angles):
                    ang = e.theta + a
                    lx = e.x + d * math.cos(ang)
                    ly = e.y + d * math.sin(ang)
                    lpx, lpy = self.world_to_px(lx, ly)
                    c.begin_path()
                    c.move_to(rx, ry)
                    c.line_to(lpx, lpy)
                    c.stroke()

            # -------------------------------------------------
            # TORCH LIGHT (dungeon)
            # -------------------------------------------------
            if mode == 1:
                draw_torch(rx, ry)

            # -------------------------------------------------
            # FOG OF WAR (dungeon + cyber)
            # -------------------------------------------------
            if mode in (1, 3):
                draw_fog(rx, ry)

