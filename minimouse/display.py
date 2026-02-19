import math, time, random, threading
import numpy as np
import ipywidgets as widgets
from IPython.display import display, Javascript
from ipycanvas import Canvas, hold_canvas
import time, json, queue
import ipywidgets as widgets
from IPython.display import display, Javascript
from google.colab import output

import json, queue
import ipywidgets as widgets
from IPython.display import display, Javascript
from ipycanvas import Canvas
from google.colab import output
from .env import MicromouseEnv
from .canvas import CanvasRenderer
from .maze import generate_perfect_maze
from .controllers import make_bug2_controller, make_path_controller
from zoneinfo import ZoneInfo
from datetime import datetime

class Display:
    """
    Single, self-contained UI + sim loop wrapper.

    - You can swap behavior from outside by passing callbacks (or by assigning
      display.callbacks["..."] = your_fn later).

    Expected external symbols (already in your notebook):
      - generate_perfect_maze
      - MicromouseEnv
      - CanvasRenderer
      - user_controller(obs)   (only if you use auto mode)
      - reset_bug2()           (optional; safe if missing)
    """

    # ----------------------------
    # construction / callbacks
    # ----------------------------
    def __init__(
        self,
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
    ):
        self.px_per_cell = int(px_per_cell)
        self.fps = float(fps)
        self.cells_w = int(cells_w)
        self.cells_h = int(cells_h)
        self.default_n_rays = int(n_rays)
        self.default_seed = int(seed)

        # state
        self.control = {"vl": 0.0, "vr": 0.0, "running": False, "mode": "manual"}
        self.window_keys = {}
        self.ui_queue = queue.Queue()

        # callbacks
        self.callbacks = {
            "make_env": make_env_fn or self._default_make_env,
            "user_controller": user_controller_fn,         # can be None
            "reset_hook": reset_hook_fn,                   # can be None
            "on_status": on_status_fn,                     # can be None
            "on_time": None,                     # can be None
            # optional hooks if you want them
            "on_step_end": None,                           # fn(display)->None
            "on_newmaze": None,                            # fn(display, , n_rays)->None
        }

        # build env/canvas/renderer + widgets
        self.env = self.callbacks["make_env"](
            seed=self.default_seed,
            cells_w=self.cells_w,
            cells_h=self.cells_h,
            n_rays=self.default_n_rays,
        )
        self.canvas = Canvas(
            width=self.env.W * self.px_per_cell,
            height=self.env.H * self.px_per_cell,
        )
        self.renderer = CanvasRenderer(self.canvas, self.env, px_per_cell=self.px_per_cell)

        self._build_widgets()
        self._bind_widgets()

        self.IterCounter=0

    # ----------------------------
    # defaults / utilities
    # ----------------------------
    def _default_make_env(self, seed=3, cells_w=9, cells_h=9, n_rays=12):
        occ, start_cell, goal_cell = generate_perfect_maze(
            seed=seed, cells_w=cells_w, cells_h=cells_h
        )
        return MicromouseEnv(occ, start_cell, goal_cell, cell_size=1.0, n_rays=n_rays)

    def set_status(self, msg: str):
        self.status.value = f"<b>Status:</b> {msg}"
        cb = self.callbacks.get("on_status")
        if callable(cb):
            cb(msg)

    def set_time(self, msg: int):
        self.solveTime.value = f"<b> Time: {msg*1.0/self.fps}s</b>"
        cb = self.callbacks.get("on_time")
        if callable(cb):
            cb(msg)

    def stop_motion(self):
        self.control["vl"] = 0.0
        self.control["vr"] = 0.0


    def reset_controler(self):
        pass
        

    def _safe_reset_hook(self):
        cb = self.callbacks.get("reset_hook")
        if callable(cb):
            cb()
            return
        # backwards-compat: if user didn’t pass reset_hook, try reset_bug2 if it exists
        try:
            reset_controler()
        except Exception:
            pass

    # ----------------------------
    # UI construction
    # ----------------------------
    def _build_widgets(self):
        # buttons
        self.btn_w = widgets.Button(description="W ↑")
        self.btn_s = widgets.Button(description="S ↓")
        self.btn_a = widgets.Button(description="A ←")
        self.btn_d = widgets.Button(description="D →")
        self.btn_zero = widgets.Button(description="Zero")

        self.btn_run = widgets.Button(description="Run", button_style="success")
        self.btn_stop = widgets.Button(description="Stop", button_style="danger")
        self.btn_reset = widgets.Button(description="Reset", button_style="warning")
        self.btn_newmaze = widgets.Button(description="New Maze (Seed)", button_style="info")

        self.cmd_box = widgets.Text(
            description="Cmd:",
            placeholder="w/a/s/d/x",
            layout=widgets.Layout(width="200px"),
        )

        # settings
        self.seed_box = widgets.IntText(value=self.default_seed, description="Seed:")
        self.rays_box = widgets.IntSlider(
            value=self.default_n_rays, min=3, max=41, step=1, description="Rays:"
        )


        
        #self.mode_dd = widgets.Dropdown(
        #    options=[("Manual", "manual"), ("Auto (user_controller)", "auto"),("Auto (BUG2)", "autobug2"), ("Auto (A*)", "autoAstar"), ("Auto (RRT*)", "autoRRTstar")],
        #    value="manual",
        #    description="Mode:",
        #)

        lisbon = ZoneInfo("Europe/Lisbon")
        unlock_time = datetime(2026, 2, 20, 18, 30, tzinfo=lisbon)
        
        if datetime.now(lisbon) >= unlock_time:
            self.mode_dd = widgets.Dropdown(
                options=[
                    ("Manual", "manual"),
                    ("Auto (user_controller)", "auto"),
                    ("Auto (BUG2)", "autobug2"),
                    ("Auto (A*)", "autoAstar"),
                    ("Auto (RRT*)", "autoRRTstar"),
                ],
                value="manual",
                description="Mode:",
            )
        else: 
            self.mode_dd = widgets.Dropdown(
                options=[
                    ("Manual", "manual"),
                    ("Auto (user_controller)", "auto"),
                ],
                value="manual",
                description="Mode:",
            )

        self.V_slider = widgets.FloatSlider(
            value=0.3, min=0.0, max=3.5, step=0.1, description="Forward V"
        )
        self.W_slider = widgets.FloatSlider(
            value=0.6, min=0.0, max=6.0, step=0.1, description="Turn W"
        )
        self.occlusion = widgets.FloatSlider(
            value=0.3, min=0.0, max=1.0, step=0.01, description="Occlusion"
        )
        self.viewmode = widgets.IntSlider(value=1, min=0, max=3, step=1, description="View")
        self.laser_toggle = widgets.Checkbox(value=True, description="Laser")
        self.traj_toggle = widgets.Checkbox(value=True, description="Trajectory")
        self.look_toggle = widgets.Checkbox(value=True, description="Lookahead")

        # status
        self.status = widgets.HTML(value="<b>Status:</b> Ready")
        self.solveTime = widgets.HTML(value="<b> Time: 0s</b>")
        self.hint = widgets.HTML(
            value="<i>Tip:</i> click the canvas once to focus for WASD (buttons always work)."
        )

        # hidden tick button (clock)
        self.tick_btn = widgets.Button(layout=widgets.Layout(display="none"))

    def _bind_widgets(self):
        # command box
        self.cmd_box.observe(self._on_cmd_enter, names="value")

        # button bindings
        self.btn_w.on_click(self._w_clicked)
        self.btn_s.on_click(self._s_clicked)
        self.btn_a.on_click(self._a_clicked)
        self.btn_d.on_click(self._d_clicked)
        self.btn_zero.on_click(self._zero_clicked)

        self.btn_run.on_click(self._run_clicked)
        self.btn_stop.on_click(self._stop_clicked)
        self.btn_reset.on_click(self._reset_clicked)
        self.btn_newmaze.on_click(self._newmaze_clicked)

        self.mode_dd.observe(self._mode_changed, names="value")
        self.rays_box.observe(self._rays_changed, names="value")

        self.tick_btn.on_click(self._on_tick)

    # ----------------------------
    # public API
    # ----------------------------
    def show(self):
        display(self.canvas)
        self.draw()

        display(
            widgets.HBox([self.btn_w, self.btn_s, self.btn_a, self.btn_d, self.btn_zero]),
            widgets.HBox([self.btn_run, self.btn_stop, self.btn_reset, self.btn_newmaze]),
            widgets.HBox([self.mode_dd, self.seed_box, self.rays_box]),
            widgets.HBox([self.V_slider, self.W_slider]),
            widgets.HBox([self.cmd_box, self.status, self.solveTime]),
            widgets.HBox([self.laser_toggle, self.traj_toggle, self.look_toggle, self.viewmode, self.occlusion]),
            self.hint, self.tick_btn
        )
         

        self.set_status("Ready — click Run, then use buttons (or WASD)")
        self._install_keyboard()
        self._install_tick_loop()
        return self

    def draw(self):
        self.renderer.draw(
            show_laser=self.laser_toggle.value,
            show_trajectory=self.traj_toggle.value,
            show_lookahead=self.look_toggle.value,
            mode=int(self.viewmode.value),
            occlusion=float(self.occlusion.value),
        )

    def step_once(self):
        if not self.control["running"]:
            self.draw()
            return

        obs = self.env._get_obs()

        if self.control["mode"] == "auto":
            ctrl = self.callbacks.get("user_controller")
            self.env.waypoints= None  
            if not callable(ctrl):
                # graceful fallback
                vl, vr = self.control["vl"], self.control["vr"]
            else:
                vl, vr = ctrl(obs)
        elif self.control["mode"] == "autobug2":
            ctrl = self.callbacks.get("user_controller")
            if not callable(ctrl):
                # graceful fallback
                bug_controller, reset_bug = make_bug2_controller(self.env)
                self.callbacks["user_controller"]=bug_controller
                self.callbacks["reset_hook"]=reset_bug
                vl, vr = self.control["vl"], self.control["vr"]
                self.env.waypoints= None  
            else:
                vl, vr = ctrl(obs)
        elif self.control["mode"] == "autoAstar":
            ctrl = self.callbacks.get("user_controller")
            if not callable(ctrl):
                # graceful fallback
                path_controller, reset_path, state = make_path_controller(self.env, planner="astar")     # or "rrt" / "rrtstar"
                self.callbacks["user_controller"]=path_controller
                self.callbacks["reset_hook"]=reset_path
                #print(state)
                self.env.waypoints= state["waypoints"]  
                
                vl, vr = self.control["vl"], self.control["vr"]
            else:
                vl, vr = ctrl(obs)
        elif self.control["mode"] == "autoRRTstar":
            ctrl = self.callbacks.get("user_controller")
            if not callable(ctrl):
                # graceful fallback
                path_controller, reset_path, state = make_path_controller(self.env, planner="rrtstar")     # or "rrt" / "rrtstar"
                self.callbacks["user_controller"]=path_controller
                self.callbacks["reset_hook"]=reset_path
                #print(state)
                self.env.waypoints= state["waypoints"]  
                vl, vr = self.control["vl"], self.control["vr"]
            else:
                vl, vr = ctrl(obs)
        
        else:
            # manual: buttons + optional keyboard override
            self.env.waypoints= None  

            vl, vr = self.control["vl"], self.control["vr"]
            if self.window_keys:
                vl, vr = self._keys_to_action(self.window_keys, vl, vr)

        self.env.step((vl, vr))
        self.draw()



        eps = 1e-9
        if abs(vl) > eps and abs(vr) > eps:
            self.IterCounter=self.IterCounter+1

        self.set_time(self.IterCounter)
        cb = self.callbacks.get("on_step_end")
        if callable(cb):
            cb(self)

    # ----------------------------
    # input handling
    # ----------------------------
    def _keys_to_action(self, keys, fallback_vl, fallback_vr):
        V = self.V_slider.value
        W = self.W_slider.value
        if keys.get("w"): return V, V
        if keys.get("s"): return -V, -V
        if keys.get("a"): return -W, W
        if keys.get("d"): return W, -W
        return fallback_vl, fallback_vr

    def _on_cmd_enter(self, change):
        txt = (change.get("new") or "").strip().lower()
        if not txt:
            return

        c = txt[-1]
        V = self.V_slider.value
        W = self.W_slider.value

        if c == "w":
            self.control["vl"], self.control["vr"] = V, V
            self.set_status("Cmd: Forward")
        elif c == "s":
            self.control["vl"], self.control["vr"] = -V, -V
            self.set_status("Cmd: Backward")
        elif c == "a":
            self.control["vl"], self.control["vr"] = -W, W
            self.set_status("Cmd: Left")
        elif c == "d":
            self.control["vl"], self.control["vr"] = W, -W
            self.set_status("Cmd: Right")
        elif c == "x":
            self.stop_motion()
            self.set_status("Cmd: Stop")

        self.cmd_box.value = ""

    # ----------------------------
    # widget callbacks
    # ----------------------------
    def _w_clicked(self, _):
        V = self.V_slider.value
        self.control["vl"], self.control["vr"] = V, V
        self.set_status("Forward")

    def _s_clicked(self, _):
        V = self.V_slider.value
        self.control["vl"], self.control["vr"] = -V, -V
        self.set_status("Backward")

    def _a_clicked(self, _):
        W = self.W_slider.value
        self.control["vl"], self.control["vr"] = -W, W
        self.set_status("Turn left")

    def _d_clicked(self, _):
        W = self.W_slider.value
        self.control["vl"], self.control["vr"] = W, -W
        self.set_status("Turn right")

    def _zero_clicked(self, _):
        self.stop_motion()
        self.set_status("Stopped")

    def _run_clicked(self, _):
        self.control["running"] = True
        self.set_status("Running")

    def _stop_clicked(self, _):
        self.control["running"] = False
        self.stop_motion()
        self.set_status("Stopped")

    def _reset_clicked(self, _):
        self.control["running"] = False
        self.stop_motion()
        self.env.reset()
        self._safe_reset_hook()
        self.draw()
        self.set_status("Reset")

    def _newmaze_clicked(self, _):
        self.control["running"] = False
        self.stop_motion()
        self._safe_reset_hook()

        seed = int(self.seed_box.value)
        n_rays = int(self.rays_box.value)

        self.env = self.callbacks["make_env"](
            seed=seed, cells_w=self.cells_w, cells_h=self.cells_h, n_rays=n_rays
        )
        self.canvas.width = self.env.W * self.px_per_cell
        self.canvas.height = self.env.H * self.px_per_cell
        self.renderer = CanvasRenderer(self.canvas, self.env, px_per_cell=self.px_per_cell)
        self.draw()
        self.set_status(f"New maze seed={seed}")
        self.IterCounter=0
        self.set_time(0)


        cb = self.callbacks.get("on_newmaze")
        if callable(cb):
            cb(self, seed, n_rays)

    def _mode_changed(self, change):
        self.control["mode"] = change["new"]
        self.callbacks["user_controller"]=None
        self.callbacks["reset_hook"]=None
        self.env.waypoints=None
        self.set_status(f"Mode: {self.control['mode']}")
        self.IterCounter=0
        self.set_time(0)


    def _rays_changed(self, change):
        # update live env rays only (doesn't rebuild maze)
        try:
            self.env.n_rays = int(change["new"])
        except Exception:
            pass
        self.draw()

    # ----------------------------
    # tick loop / colab plumbing
    # ----------------------------
    def _on_tick(self, _):
        self.step_once()

    def _install_keyboard(self):
        def _update_keys(json_str):
            try:
                self.window_keys = json.loads(json_str)
            except Exception:
                self.window_keys = {}

        output.register_callback("micromouse.update_keys", _update_keys)

        display(Javascript(r"""
(function(){
  if (window._mm_keys_installed) return;
  window._mm_keys_installed = true;

  const canvases = document.querySelectorAll('canvas');
  if (!canvases.length) return;
  const canvas = canvases[canvases.length - 1];

  canvas.tabIndex = 0;
  canvas.style.outline = '2px solid rgba(0,0,0,0.2)';

  const keys = {};

  function send(){
    try{
      google.colab.output.invokeFunction('micromouse.update_keys',
        [JSON.stringify(keys)], {});
    } catch(e){}
  }

  canvas.addEventListener('click', () => canvas.focus());

  canvas.addEventListener('keydown', (e) => {
    const k = e.key.toLowerCase();
    keys[k] = true;
    if (['w','a','s','d'].includes(k)){
      e.preventDefault(); e.stopPropagation();
    }
    send();
  }, true);

  canvas.addEventListener('keyup', (e) => {
    const k = e.key.toLowerCase();
    keys[k] = false;
    if (['w','a','s','d'].includes(k)){
      e.preventDefault(); e.stopPropagation();
    }
    send();
  }, true);
})();
"""))

    def _install_tick_loop(self):
        interval_ms = max(1, int(1000.0 / max(1.0, self.fps)))
        #print(interval_ms)
        display(Javascript(r"""
(function(){
  if (window._mm_tick_loop_installed) return;
  window._mm_tick_loop_installed = true;

  const btns = document.querySelectorAll('button');
  let tickBtn = null;
  btns.forEach(b => { if (b.style.display === 'none') tickBtn = b; });

  if (!tickBtn){
    console.warn("Tick button not found");
    return;
  }

  const intervalMs = %d;

  setInterval(() => {
    tickBtn.click();
  }, intervalMs);

  console.log("Micromouse tick loop running (button-driven), interval=", intervalMs);
})();
""" % interval_ms))


 

 
