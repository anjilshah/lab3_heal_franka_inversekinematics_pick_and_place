#!/usr/bin/env python3
"""Live side-by-side Franka Panda DLS, Mink, and QP comparison."""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "utils"))
import core  # noqa: E402
from collision import CollisionAudit  # noqa: E402

METHODS = ("dls", "mink", "qp")
ACCENTS = {"dls": "#56b4e9", "mink": "#cc79a7", "qp": "#e69f00"}
PANEL_BG = "#1b2633"
APP_BG = "#101820"
TICK_MS = 30
FRAME_WIDTH = 480
FRAME_HEIGHT = 300
PHASE_NAMES = ("approach", "descend", "grasp", "lift", "transfer",
               "lower", "release", "retreat")
PHASE_DURATIONS_S = (2.0,) * len(PHASE_NAMES)
LIVE_RUNS_DIR = ROOT / "results" / "live_runs"


def make_camera():
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.13, 0.0, 0.43]
    camera.distance = 1.42
    camera.azimuth = 135
    camera.elevation = -23
    return camera


def episode_targets(cube_xy, place_xy):
    pad_z = core.TABLE_TOP + core.CUBE_HALF
    return [
        (np.array([cube_xy[0], cube_xy[1], pad_z + core.SITE_TO_PAD + 0.12]), 255),
        (np.array([cube_xy[0], cube_xy[1], pad_z + core.SITE_TO_PAD]), 255),
        (np.array([cube_xy[0], cube_xy[1], pad_z + core.SITE_TO_PAD]), 0),
        (np.array([cube_xy[0], cube_xy[1], pad_z + core.SITE_TO_PAD + 0.16]), 0),
        (np.array([place_xy[0], place_xy[1], pad_z + core.SITE_TO_PAD + 0.16]), 0),
        (np.array([place_xy[0], place_xy[1], pad_z + core.SITE_TO_PAD]), 0),
        (np.array([place_xy[0], place_xy[1], pad_z + core.SITE_TO_PAD]), 255),
        (np.array([place_xy[0], place_xy[1], pad_z + core.SITE_TO_PAD + 0.12]), 255),
    ]


class EpisodePanel:
    """Independent Franka simulation state for one solver panel."""

    def __init__(self, method, model, data, indices, renderer=None, photo_widget=None):
        self.method = method
        self.model = model
        self.data = data
        self.renderer = renderer
        self.photo_widget = photo_widget
        self.camera = make_camera()
        self.indices = indices
        self.image_updates = 0
        self.reset(20261013, autostart=False)

    def reset(self, seed, autostart=True):
        self.seed = int(seed)
        rng = np.random.default_rng(seed)
        cube_xy = core.PICK_XY.copy()
        place_xy = core.PLACE_XY.copy()
        yaw = float(rng.uniform(-0.12, 0.12))
        core.reset(self.model, self.data, self.indices, cube_xy=cube_xy, yaw=yaw)
        self.cube_xy = cube_xy
        self.place_xy = place_xy
        self.targets = episode_targets(cube_xy, place_xy)
        self.initial_cube = self._cube_position().copy()
        self.peak_z = float(self.initial_cube[2])
        self.rotation = self.data.site_xmat[self.indices[3]].reshape(3, 3).copy()
        self.phase_index = 0
        self.phase_step = 0
        self.phase_steps = max(1, int(PHASE_DURATIONS_S[0] / self.model.opt.timestep))
        self.q_start = None
        self.q_goal = None
        self.waypoints = []
        self.last_solver_status = "not started"
        self.current_position_error = None
        self.current_orientation_error = None
        self.current_phase = "Ready"
        self.failure_reason = None
        self.success = False
        self.done = True
        self.result = None
        self.started_wall_time = time.perf_counter()
        self.ik_solve_time_total_s = 0.0
        self.max_position_residual_m = 0.0
        self.max_orientation_residual_rad = 0.0
        self.yaw = yaw
        self.path_checks = []
        self.collision_audit = CollisionAudit(self.model, stride=25)
        self.path_guard = core.make_path_guard(self.model, self.data, self.indices)
        self.telemetry = core.EpisodeTelemetry(
            self.model, self.data, self.indices, self.initial_cube, self.collision_audit)
        if autostart:
            self.done = False
            self._solve_phase()

    def _cube_position(self):
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        return self.data.xpos[body].copy()

    def _solve_phase(self):
        if self.phase_index >= len(self.targets):
            self._finish()
            return
        self.current_phase = PHASE_NAMES[self.phase_index]
        target, grip = self.targets[self.phase_index]
        self.telemetry.begin_phase(self.current_phase, target, grip)
        try:
            result = core.solve(self.method, self.model, self.data, self.indices, target,
                                self.rotation)
        except Exception as exc:
            self._fail(f"solver_exception:{self.current_phase}:{type(exc).__name__}:{exc}")
            return
        self.waypoints.append({**result, "phase": self.current_phase,
                               "target_xyz": target.tolist()})
        self.ik_solve_time_total_s += float(result["solve_time_s"])
        self.max_position_residual_m = max(self.max_position_residual_m,
                                           float(result["position_residual_m"]))
        self.max_orientation_residual_rad = max(self.max_orientation_residual_rad,
                                                float(result["orientation_residual_rad"]))
        self.last_solver_status = result["status"]
        safe, check = self.path_guard(self.data.qpos[self.indices[0]].copy(),
                                      np.asarray(result["q"], dtype=float))
        self.path_checks.append({"phase": self.current_phase, **check,
                                "accepted": bool(safe)})
        if not safe:
            self._fail(f"sampled_path_rejected_{self.current_phase}")
            return
        self.q_start = self.data.qpos[self.indices[0]].copy()
        self.q_goal = np.asarray(result["q"], dtype=float)
        self.phase_step = 0
        self.phase_steps = max(1, int(PHASE_DURATIONS_S[self.phase_index] /
                                      self.model.opt.timestep))
        if result["status"] != "converged":
            self._fail(f"ik_{self.current_phase}_{result['status']}")

    def advance_one_step(self):
        if self.done:
            return
        target, grip = self.targets[self.phase_index]
        self.phase_step += 1
        u = min(1.0, self.phase_step / self.phase_steps)
        blend = u * u * (3.0 - 2.0 * u)
        q = self.q_start + (self.q_goal - self.q_start) * blend
        core._step(self.model, self.data, self.indices, q, grip)
        self.telemetry.observe(self.current_phase, grip)
        self.peak_z = max(self.peak_z, float(self._cube_position()[2]))
        if self.phase_step >= self.phase_steps:
            self.telemetry.end_phase(target, self.rotation)
            self.phase_index += 1
            if self.phase_index >= len(self.targets):
                self._finish()
            else:
                self._solve_phase()
        self._update_live_errors()

    def _update_live_errors(self):
        if self.phase_index >= len(self.targets):
            target = self.targets[-1][0]
        else:
            target = self.targets[self.phase_index][0]
        site = self.indices[3]
        mujoco.mj_forward(self.model, self.data)
        current = self.data.site_xpos[site].copy()
        current_rot = self.data.site_xmat[site].reshape(3, 3).copy()
        self.current_position_error = float(np.linalg.norm(target - current))
        self.current_orientation_error = float(np.linalg.norm(
            core.rot_error(self.rotation, current_rot)))

    def _fail(self, reason):
        self.telemetry.end_phase()
        self.failure_reason = reason
        self.success = False
        self.done = True
        self._make_result()

    def _finish(self):
        self.telemetry.end_phase()
        final = self._cube_position()
        error = float(np.linalg.norm(final[:2] - self.place_xy))
        self.success = bool(
            self.failure_reason is None
            and self.peak_z > self.initial_cube[2] + 0.045
            and error < core.FINAL_PLACEMENT_TOLERANCE_M
            and 0.39 < final[2] < 0.49
        )
        if not self.success and self.failure_reason is None:
            self.failure_reason = "placement_tolerance_or_lost_grasp"
        self.done = True
        self._make_result(final, error)

    def _make_result(self, final=None, error=None):
        if final is None:
            final = self._cube_position()
        if error is None:
            error = float(np.linalg.norm(final[:2] - self.place_xy))
        audit = self.collision_audit.to_dict()
        if core.collision_audit_has_penetration(audit):
            self.success = False
            self.failure_reason = self.failure_reason or "unexpected_collision"
        clearance_by_category = audit["minimum_signed_clearance_m_by_category"]
        clearance_values = [float(clearance_by_category[key])
                            for key in ("robot-floor", "robot-table", "arm-cube")
                            if key in clearance_by_category and np.isfinite(clearance_by_category[key])]
        clearance_values.extend(float(check["minimum_sampled_clearance_m"])
                                 for check in self.path_checks
                                 if np.isfinite(check["minimum_sampled_clearance_m"]))
        min_clearance = min(clearance_values) if clearance_values else None
        self.result = {
            "seed": int(self.seed),
            "method": self.method,
            "success": bool(self.success),
            "failure_reason": self.failure_reason,
            "initial_cube_xyz": self.initial_cube.tolist(),
            "final_cube_xyz": final.tolist(),
            "place_target_xy": self.place_xy.tolist(),
            "target_error_m": error,
            "final_xy_error_m": error,
            "final_cube_z_m": float(final[2]),
            "lift_peak_z_m": self.peak_z,
            "sim_time_s": float(self.data.time),
            "episode_wall_time_s": time.perf_counter() - self.started_wall_time,
            "ik_solve_time_total_s": self.ik_solve_time_total_s,
            "episode_wall_time_s": time.perf_counter() - self.started_wall_time,
            "max_position_residual_m": self.max_position_residual_m,
            "max_orientation_residual_rad": self.max_orientation_residual_rad,
            "solver_status": self.last_solver_status,
            "waypoints": self.waypoints,
            "path_checks": self.path_checks,
            "task_success_checks": {
                "final_xy_error_lt_0_010_m": bool(error < core.FINAL_PLACEMENT_TOLERANCE_M),
                "lift_peak_gt_initial_z_plus_0_045_m": bool(
                    self.peak_z > self.initial_cube[2] + 0.045),
                "final_cube_z_in_open_0_39_0_49_m": bool(0.39 < final[2] < 0.49),
            },
            "placement_attempted": self.telemetry.release_phase_reached,
            **self.telemetry.summary(min_clearance),
            "episode_specs": {"cube_xy": self.cube_xy.tolist(),
                              "place_xy": self.place_xy.tolist(), "yaw_rad": self.yaw},
            "success_criterion": {
                "final_xy_tolerance_m": core.FINAL_PLACEMENT_TOLERANCE_M,
                "lift_peak_z_m_gt_initial_plus_m": 0.045,
                "final_cube_z_m_open_interval": [0.39, 0.49],
            },
        }

    def status_text(self):
        if self.done and self.result is None:
            return "Ready · press Start / Reset All\nIK: not started\nPlacement: not run"
        if self.current_position_error is None:
            live = "position/orientation: —"
        else:
            live = (f"live pos {self.current_position_error * 1000:.1f} mm · "
                    f"ori {self.current_orientation_error:.3f} rad")
        if self.done and self.result is not None:
            placement = (f"PLACEMENT PASS · XY {self.result['target_error_m']*1000:.1f} mm"
                         if self.success else
                         f"PLACEMENT FAIL · {self.failure_reason} · "
                         f"terminal XY error {self.result['target_error_m']*1000:.1f} mm")
            phase = f"{self.current_phase.upper()} · {len(self.waypoints)}/8 waypoints"
        else:
            phase = f"{self.current_phase.upper()} · {self.phase_step}/{self.phase_steps} steps"
            placement = "PLACEMENT: running · final result pending"
        solver = self.last_solver_status.upper()
        return f"{phase}\nIK: {solver} · {live}\n{placement}"

    def render(self):
        self.renderer.update_scene(self.data, camera=self.camera)
        frame = self.renderer.render()
        return frame


class FrankaComparisonWindow:
    def __init__(self, speed=0.8, seed=20261013, self_test=False):
        import tkinter as tk
        from PIL import Image, ImageTk

        self.tk = tk
        self.Image = Image
        self.ImageTk = ImageTk
        self.speed = speed
        self.seed = seed
        self.self_test = self_test
        self.root = tk.Tk()
        self.root.title("Franka Panda Pick-and-Place · DLS | Mink | QP")
        self.root.configure(bg=APP_BG)
        self.root.geometry("1640x860")
        self.root.minsize(1260, 760)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.panels = []
        self.photo_refs = []
        self.sim_accumulator = 0.0
        self.last_tick = time.perf_counter()
        self.running_once = False
        self.render_count = 0
        self.closed = False
        self.live_run_dir = None
        self.run_records = []
        self.saved_panel_methods = set()
        self.last_saved_run_status = None
        self._build_ui()
        self._make_panels()
        self._render_panels()
        self.root.bind("<space>", lambda _event: self.start_or_reset())
        self.root.after(TICK_MS, self.tick)
        if self_test:
            self.root.after(800, self._self_test_start)

    def _build_ui(self):
        tk = self.tk
        header = tk.Frame(self.root, bg=APP_BG)
        header.pack(fill="x", padx=18, pady=(12, 9))
        tk.Label(header, text="FRANKA PANDA · LIVE IK COMPARISON",
                 font=("TkDefaultFont", 18, "bold"), fg="#f3f6f9",
                 bg=APP_BG).pack(side="left")
        self.speed_label = tk.Label(header,
                                    text=f"Playback speed: {self.speed:g}× simulation",
                                    font=("TkDefaultFont", 12), fg="#cbd5df",
                                    bg=APP_BG)
        self.speed_label.pack(side="left", padx=24)
        self.start_button = tk.Button(
            header, text="Start / Reset All", command=self.start_or_reset,
            font=("TkDefaultFont", 13, "bold"), bg="#e7edf2", fg="#101820",
            activebackground="#ffffff", padx=18, pady=9, relief="flat", cursor="hand2")
        self.start_button.pack(side="right")
        self.panel_row = tk.Frame(self.root, bg=APP_BG)
        self.panel_row.pack(fill="x", padx=12, pady=(0, 4))
        self.panel_row.configure(height=FRAME_HEIGHT + 145)
        self.panel_row.pack_propagate(False)
        self._build_benchmark_table()

    def _build_benchmark_table(self):
        tk = self.tk
        section = tk.Frame(self.root, bg=APP_BG)
        section.pack(fill="x", padx=20, pady=(2, 12))
        self.results_section = section
        self.final_table_title = tk.Label(
            section, text="FINAL RUN RESULTS", font=("TkDefaultFont", 17, "bold"),
            fg="#f3f6f9", bg=APP_BG, anchor="w")
        self.final_table_title.pack(fill="x", pady=(0, 2))
        self.live_run_status = tk.Label(
            section, text="No run yet · click Start / Reset All for a matched live comparison.",
            font=("TkDefaultFont", 9), fg="#cbd5df", bg=APP_BG, anchor="w")
        self.live_run_status.pack(fill="x", pady=(0, 5))
        headers = ("METHOD", "EPISODES COMPLETED", "SUCCESS COUNT / RATE", "MEAN IK SOLVE (ms)",
                   "MEAN WALL TIME (s)", "MIN SAMPLED CLEARANCE (mm)")
        self.live_cells = self._make_results_grid(section, headers, live=True)
        tk.Label(section, text=("Task success: XY <10 mm, lift >45 mm, final cube z in "
                               "(0.39, 0.49) m. IK convergence alone is not task success. "
                               "Wall time uses a monotonic clock; clearance is sampled."),
                 font=("TkDefaultFont", 9), fg="#9eb0c0", bg=APP_BG,
                 anchor="w", wraplength=1560).pack(fill="x", pady=(4, 8))

    def _make_results_grid(self, parent, headers, live):
        tk = self.tk
        grid = tk.Frame(parent, bg="#263544", highlightthickness=1,
                        highlightbackground="#405568")
        grid.pack(fill="x")
        widths = (10, 12, 20, 20, 18, 37)
        for col, (label, width) in enumerate(zip(headers, widths)):
            tk.Label(grid, text=label, width=width, anchor="center",
                     font=("TkDefaultFont", 9, "bold"), fg="#9eb0c0",
                     bg="#263544", padx=3, pady=4).grid(
                         row=0, column=col, sticky="nsew", padx=1, pady=1)
            grid.columnconfigure(col, weight=1)
        cells = {}
        for row_index, method in enumerate(METHODS, start=1):
            row_cells = []
            for col, width in enumerate(widths):
                label = tk.Label(grid, text="—", width=width, anchor="center",
                                 font=("TkDefaultFont", 10, "bold"),
                                 fg=ACCENTS[method] if col == 0 else "#f1f4f7",
                                 bg=PANEL_BG, padx=3, pady=3)
                label.grid(row=row_index, column=col, sticky="nsew", padx=1, pady=1)
                row_cells.append(label)
            cells[method] = row_cells
        if live:
            for method in METHODS:
                self._set_table_row(cells, method,
                                    (method.upper(), "0", "—", "—", "—", "N/A"))
        return cells

    @staticmethod
    def _set_table_row(cells, method, values):
        for cell, value in zip(cells[method], values):
            cell.configure(text=value)

    def _make_panels(self):
        for column, method in enumerate(METHODS):
            model, data, *rest = core.load_model()
            indices = tuple(rest)
            core.reset(model, data, indices, core.PICK_XY, yaw=0.0)
            renderer = mujoco.Renderer(model, height=FRAME_HEIGHT, width=FRAME_WIDTH)
            shell = self.tk.Frame(self.panel_row, bg=PANEL_BG,
                                  highlightthickness=2,
                                  highlightbackground=ACCENTS[method])
            shell.grid(row=0, column=column, sticky="nsew", padx=6)
            self.panel_row.columnconfigure(column, weight=1, uniform="three-panels")
            self.panel_row.rowconfigure(0, weight=1)
            self.tk.Label(shell, text=method.upper(),
                          font=("TkDefaultFont", 22, "bold"),
                          fg=ACCENTS[method], bg=PANEL_BG).pack(fill="x", pady=(9, 4))
            status = self.tk.Label(shell,
                                   text="Ready · press Start / Reset All\nIK: not started\nPlacement: not run",
                                   font=("TkFixedFont", 10), fg="#f1f4f7", bg=PANEL_BG,
                                   justify="center", anchor="center", height=4,
                                   wraplength=FRAME_WIDTH - 18)
            status.pack(fill="x", padx=8, pady=(0, 3))
            image = self.tk.Label(shell, bg="#080c10", bd=0)
            image.pack(fill="both", expand=True, padx=6, pady=(2, 8))
            panel = EpisodePanel(method, model, data, indices, renderer, image)
            panel.status_widget = status
            self.panels.append(panel)

    def start_or_reset(self):
        self._interrupt_current_run("reset_before_completion")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        self.live_run_dir = LIVE_RUNS_DIR / f"run_{stamp}_{time.time_ns() % 1000000:06d}"
        self.live_run_dir.mkdir(parents=True, exist_ok=False)
        self.run_records = []
        self.saved_panel_methods = set()
        self.last_saved_run_status = None
        started = time.perf_counter()
        (self.live_run_dir / "manifest.json").write_text(json.dumps({
            "run_id": self.live_run_dir.name,
            "started_at_utc": datetime.now(timezone.utc).isoformat(),
            "methods": list(METHODS), "seed": int(self.seed),
            "matched_conditions": True,
            "live_run": True,
            "success_criterion": {
                "final_xy_tolerance_m": core.FINAL_PLACEMENT_TOLERANCE_M,
                "lift_peak_z_m_gt_initial_plus_m": 0.045,
                "final_cube_z_m_open_interval": [0.39, 0.49],
            },
            "solver_configuration": {
                "ik_position_tolerance_m": core.IK_POSITION_TOLERANCE_M,
                "ik_orientation_tolerance_rad": core.IK_ORIENTATION_TOLERANCE_RAD,
                "joint_velocity_limit_rad_s": core.IK_VELOCITY_LIMIT_RAD_S,
                "ik_timestep_multiplier": core.IK_TIMESTEP_MULTIPLIER,
            },
            "phase_duration_s": list(PHASE_DURATIONS_S),
        }, indent=2) + "\n")
        self.sim_accumulator = 0.0
        for panel in self.panels:
            panel.reset(self.seed, autostart=True)
            panel.started_wall_time = started
        self.running_once = True
        self.last_tick = time.perf_counter()
        self._collect_finished_episodes()
        self._render_panels()
        self._refresh_live_results()

    def _interrupt_current_run(self, reason):
        if self.live_run_dir is None:
            return
        for panel in self.panels:
            if panel.method in self.saved_panel_methods:
                continue
            if not panel.done:
                panel.failure_reason = reason
                panel.success = False
                panel.done = True
                panel._make_result()
                row = dict(panel.result)
                row["run_status"] = "interrupted"
                self.run_records.append(row)
                self.saved_panel_methods.add(panel.method)
        status = ("completed" if all(panel.done for panel in self.panels)
                  and all(method in self.saved_panel_methods for method in METHODS)
                  and all(row.get("run_status") == "completed" for row in self.run_records)
                  else "interrupted")
        self._persist_live_run(run_status=status)

    def _collect_finished_episodes(self):
        if self.live_run_dir is None:
            return
        before = len(self.run_records)
        for panel in self.panels:
            if panel.done and panel.result is not None and panel.method not in self.saved_panel_methods:
                row = dict(panel.result)
                row["run_status"] = "completed"
                self.run_records.append(row)
                self.saved_panel_methods.add(panel.method)
        status = "completed" if all(panel.done for panel in self.panels) else "running"
        if len(self.run_records) != before or status != self.last_saved_run_status:
            self._persist_live_run(run_status=status)

    def _persist_live_run(self, run_status="running"):
        if self.live_run_dir is None:
            return
        rows_path = self.live_run_dir / "episodes.jsonl"
        rows_path.write_text("".join(json.dumps(row, allow_nan=False) + "\n"
                                            for row in self.run_records))
        completed = {method: [row for row in self.run_records
                              if row["method"] == method and row.get("run_status") == "completed"]
                     for method in METHODS}
        method_summary = {}
        for method, rows in completed.items():
            method_summary[method] = {
                "episodes_completed": len(rows),
                "successes": sum(bool(row["success"]) for row in rows),
                "success_rate": (sum(bool(row["success"]) for row in rows) / len(rows)
                                 if rows else None),
                "mean_ik_solve_time_s": (float(np.mean(
                    [row["ik_solve_time_total_s"] for row in rows])) if rows else None),
                "mean_episode_wall_time_s": (float(np.mean(
                    [row["episode_wall_time_s"] for row in rows])) if rows else None),
                "mean_min_sampled_clearance_m": (float(np.mean(
                    [row["minimum_sampled_clearance_m"] for row in rows
                     if row.get("minimum_sampled_clearance_m") is not None]))
                    if any(row.get("minimum_sampled_clearance_m") is not None
                           for row in rows) else None),
            }
        summary = {
            "run_id": self.live_run_dir.name,
            "status": run_status,
            "records_saved": len(self.run_records),
            "episodes_completed": sum(len(rows) for rows in completed.values()),
            "success_criterion": {
                "final_xy_tolerance_m": core.FINAL_PLACEMENT_TOLERANCE_M,
                "lift_peak_z_m_gt_initial_plus_m": 0.045,
                "final_cube_z_m_open_interval": [0.39, 0.49],
            },
            "methods": method_summary,
            "clearance_metric": ("Minimum signed robot-floor/table and arm-cube geom distance "
                                 "from sampled joint paths and runtime CollisionAudit. "
                                 "Sampling is discrete and is not a collision-free guarantee."),
        }
        (self.live_run_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, allow_nan=False) + "\n")
        self.last_saved_run_status = run_status

    def _refresh_live_results(self):
        for method in METHODS:
            rows = [row for row in self.run_records
                    if row["method"] == method and row.get("run_status") == "completed"]
            completed = len(rows)
            if not completed:
                values = (method.upper(), "0", "—", "—", "—", "N/A")
            else:
                successes = sum(bool(row["success"]) for row in rows)
                values = (method.upper(), str(completed),
                          f"{successes}/{completed} ({successes/completed:.0%})",
                          f"{np.mean([r['ik_solve_time_total_s'] for r in rows])*1000:.2f}",
                          f"{np.mean([r['episode_wall_time_s'] for r in rows]):.2f}",
                          self._format_clearance(rows))
            self._set_table_row(self.live_cells, method, values)
        if self.live_run_dir is None:
            self.live_run_status.configure(
                text="No live run yet · click Start / Reset All to run all three methods.")
        else:
            completed_count = sum(row.get("run_status") == "completed"
                                  for row in self.run_records)
            self.live_run_status.configure(
                text=(f"Run {self.live_run_dir.name} · {completed_count}/3 episodes completed · "
                      f"saved to {self.live_run_dir.relative_to(ROOT)}"))

    @staticmethod
    def _format_clearance(rows):
        values = [row["minimum_sampled_clearance_m"] * 1000
                  for row in rows
                  if row.get("minimum_sampled_clearance_m") is not None]
        return f"{np.mean(values):.2f}" if values else "N/A"

    def tick(self):
        now = time.perf_counter()
        elapsed = max(0.0, now - self.last_tick)
        self.last_tick = now
        dt = self.panels[0].model.opt.timestep
        self.sim_accumulator += elapsed * self.speed
        steps = int(self.sim_accumulator / dt)
        if steps:
            self.sim_accumulator -= steps * dt
            for _ in range(steps):
                for panel in self.panels:
                    panel.advance_one_step()
        self._collect_finished_episodes()
        self._refresh_live_results()
        self._render_panels()
        if (self.self_test and self.running_once and all(p.done for p in self.panels)
                and not getattr(self, "_self_test_completion_scheduled", False)):
            self._self_test_completion_scheduled = True
            self.root.after_idle(self._self_test_complete)
        self.root.after(TICK_MS, self.tick)

    def _render_panels(self):
        self.photo_refs.clear()
        for panel in self.panels:
            panel.status_widget.configure(text=panel.status_text())
            pixels = panel.render()
            if pixels.shape != (FRAME_HEIGHT, FRAME_WIDTH, 3) or pixels.dtype != np.uint8:
                raise RuntimeError(f"Unexpected frame for {panel.method}: {pixels.shape} {pixels.dtype}")
            photo = self.ImageTk.PhotoImage(self.Image.fromarray(pixels, mode="RGB"))
            panel.photo_widget.configure(image=photo)
            self.photo_refs.append(photo)
            panel.image_updates += 1
        self.render_count += 1

    def _self_test_start(self):
        self.root.update_idletasks()
        self.start_button.invoke()
        if not all(panel.done or panel.waypoints for panel in self.panels):
            raise RuntimeError("Start / Reset All did not start every panel")
        if (not all(panel.photo_widget.winfo_ismapped() and
                    panel.status_widget.winfo_ismapped() for panel in self.panels)
                or not self.results_section.winfo_ismapped()
                or self.final_table_title.cget("text") != "FINAL RUN RESULTS"
                or len(self.live_cells) != 3
                or not all(cell.winfo_ismapped() for cells in self.live_cells.values()
                           for cell in cells)):
            raise RuntimeError("Not all simulation and results widgets are visible")
        self._self_test_start_working = True

    def _self_test_complete(self):
        if len(self.run_records) != 3 or any(
                row.get("run_status") != "completed" for row in self.run_records):
            raise RuntimeError("Live run did not persist three completed raw episodes")
        path = self.live_run_dir
        saved = [json.loads(line) for line in (path / "episodes.jsonl").read_text().splitlines()
                 if line.strip()]
        summary = json.loads((path / "summary.json").read_text())
        if len(saved) != 3 or summary["episodes_completed"] != 3:
            raise RuntimeError("Saved live episode records/summary count is incorrect")
        for method in METHODS:
            row = next(r for r in saved if r["method"] == method)
            expected_success = bool(
                row["failure_reason"] is None
                and row["lift_peak_z_m"] > row["initial_cube_xyz"][2] + 0.045
                and row["target_error_m"] < core.FINAL_PLACEMENT_TOLERANCE_M
                and 0.39 < row["final_cube_xyz"][2] < 0.49)
            if row["success"] != expected_success:
                raise RuntimeError(f"Saved live task result is inconsistent for {method}")
            if self.live_cells[method][1].cget("text") != "1":
                raise RuntimeError(f"Live results table did not update for {method}")
            if self.live_cells[method][2].cget("text") != f"{int(expected_success)}/1 ({expected_success:.0%})":
                raise RuntimeError(f"Live success cell disagrees with the saved episode for {method}")
            expected_ik = f"{row['ik_solve_time_total_s'] * 1000:.2f}"
            expected_wall = f"{row['episode_wall_time_s']:.2f}"
            if (self.live_cells[method][3].cget("text") != expected_ik
                    or self.live_cells[method][4].cget("text") != expected_wall):
                raise RuntimeError(f"Live timing cells disagree with the saved episode for {method}")
            clearance = row.get("minimum_sampled_clearance_m")
            expected_clearance = "N/A" if clearance is None else f"{clearance * 1000:.2f}"
            if self.live_cells[method][5].cget("text") != expected_clearance:
                raise RuntimeError(f"Live clearance cell disagrees with the saved episode for {method}")
        rendered = self.render_count >= 3 and all(p.image_updates >= 3 for p in self.panels)
        if not rendered:
            raise RuntimeError("GUI did not render all three live simulation panels")
        completed_path = path
        completed_table_rows = {method: [cell.cget("text")
                                         for cell in self.live_cells[method]]
                                for method in METHODS}
        self.start_button.invoke()
        reset_verified = (self.live_run_dir != completed_path
                          and all(abs(float(panel.data.time)) < 1e-12
                                  and np.allclose(panel.initial_cube[:2], core.PICK_XY)
                                  for panel in self.panels))
        if not reset_verified:
            raise RuntimeError("Start / Reset All did not reset panels into a fresh run")
        self._interrupt_current_run("gui_self_test_reset_verified")
        print(json.dumps({
            "gui_live_run_test": "passed",
            "start_reset_all": "passed; second run reset and saved as interrupted self-test",
            "rendered_panels": list(METHODS),
            "live_table_rows_from_completed_run": completed_table_rows,
            "saved_run_directory": str(completed_path.relative_to(ROOT)),
            "reset_run_directory": str(self.live_run_dir.relative_to(ROOT)),
            "saved_results": [{"method": r["method"], "success": r["success"],
                               "failure_reason": r["failure_reason"],
                               "target_error_m": r["target_error_m"],
                               "ik_solve_time_total_s": r["ik_solve_time_total_s"],
                               "episode_wall_time_s": r["episode_wall_time_s"]}
                              for r in saved],
            "window_geometry": self.root.winfo_geometry(),
        }), flush=True)
        self.root.after(500, self.close)

    def close(self):
        if self.closed:
            return
        self._interrupt_current_run("window_closed")
        self.closed = True
        for panel in self.panels:
            panel.renderer.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def headless_check(seed):
    panels = []
    for method in METHODS:
        model, data, *rest = core.load_model()
        panel = EpisodePanel(method, model, data, tuple(rest))
        panel.reset(seed, autostart=True)
        panels.append(panel)
    dt_values = {p.model.opt.timestep for p in panels}
    if len(dt_values) != 1:
        raise RuntimeError(f"Method models have mismatched physics time steps: {dt_values}")
    max_steps = int(sum(PHASE_DURATIONS_S) / dt_values.pop()) + 10
    for _ in range(max_steps):
        for panel in panels:
            panel.advance_one_step()
        if all(panel.done for panel in panels):
            break
    if not all(panel.result for panel in panels):
        raise RuntimeError("At least one panel did not produce a final task result")
    if not all(np.array_equal(panels[0].initial_cube, p.initial_cube) for p in panels[1:]):
        raise RuntimeError("The panel initial cube poses are not matched")
    payload = {"headless_engine_check": "passed", "initial_conditions_matched": True,
               "placement_tolerance_m": core.FINAL_PLACEMENT_TOLERANCE_M,
               "methods": [p.result for p in panels]}
    print(json.dumps({"headless_engine_check": "passed",
                      "results": [{"method": r["method"], "success": r["success"],
                                   "failure_reason": r["failure_reason"],
                                   "target_error_m": r["target_error_m"]}
                                  for r in payload["methods"]]}, indent=2), flush=True)
    return payload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=0.8,
                        help="simulated seconds per wall second (default: 0.8)")
    parser.add_argument("--seed", type=int, default=20261013)
    parser.add_argument("--headless-check", action="store_true",
                        help="run one synchronized engine check without opening a window")
    parser.add_argument("--gui-self-test", action="store_true",
                        help="open GUI, exercise Start/Reset, verify all renders, then close")
    args = parser.parse_args()
    if args.speed <= 0:
        parser.error("--speed must be positive")
    if args.headless_check:
        headless_check(args.seed)
        return
    app = FrankaComparisonWindow(speed=args.speed, seed=args.seed,
                                  self_test=args.gui_self_test)
    app.run()


if __name__ == "__main__":
    main()
