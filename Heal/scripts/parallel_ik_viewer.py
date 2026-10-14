#!/usr/bin/env python3
"""Run DLS, Mink, and QP pick-and-place side by side in one Tk window.

Each panel owns an independent MuJoCo model/data pair and is rendered with
MuJoCo's offscreen Renderer. A shared Tk window composes those live frames;
the robot motion itself is simulated online, not recorded or replayed.

Examples:
    ./.venv/bin/python scripts/parallel_ik_viewer.py
    ./.venv/bin/python scripts/parallel_ik_viewer.py --speed 0.6
    ./.venv/bin/python scripts/parallel_ik_viewer.py --headless-check
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

import ik_viewer_demo as demo  # noqa: E402
import pick_and_place as baseline  # noqa: E402
from create_video_results_card import load_records, summarize as validate_stage3_records  # noqa: E402
from robot_common import load_model  # noqa: E402
from utils.ik_solvers import SOLVERS  # noqa: E402


METHODS = ("dls", "mink", "qp")
SOLVER_RESULT_METHOD = {"dls": "dls", "mink": "mink", "qp": "qp_daqp"}
TICK_MS = 16
RENDER_HEIGHT = 400
RENDER_WIDTH = 520
BENCHMARK_TITLE = "STAGE 3 BENCHMARK · 25 MATCHED EPISODES PER METHOD"


def load_stage3_benchmark(records_path: Path | None = None,
                          summary_path: Path | None = None) -> dict:
    """Validate saved Stage 3 records and derive GUI values from the raw episodes."""
    records_path = records_path or ROOT / "results/stage3/batch/episodes.jsonl"
    summary_path = summary_path or ROOT / "results/stage3/batch/summary.json"
    try:
        reference = json.loads(summary_path.read_text(encoding="utf-8"))
        records = load_records(records_path)
        # Reuse the results-card validator for matched seeds, task conditions,
        # counts, successes, and raw/summary IK and wall-time means.
        validate_stage3_records(records, reference)
        stats = {}
        for method in METHODS:
            rows = [r for r in records if r.get("method") == method]
            ref = reference["methods"][method]
            success_count = sum(bool(r["success"]) for r in rows)
            success_rate = success_count / len(rows)
            if (int(reference.get("episode_count_per_method", -1)) != 25
                    or len(rows) != 25
                    or not np.isclose(success_rate, ref["success_rate"], rtol=0, atol=1e-12)):
                raise ValueError(f"success rate or sample count mismatch for {method}")

            clearance = np.asarray([
                r["metrics"]["minimum_unexpected_clearance_m"] for r in rows
            ], dtype=float)
            clearance_ref = ref["minimum_unexpected_clearance_m"]
            if (not np.all(np.isfinite(clearance))
                    or not np.isclose(clearance.mean(), clearance_ref["mean"], rtol=1e-10)
                    or not np.isclose(np.median(clearance), clearance_ref["median"], rtol=1e-10)):
                raise ValueError(f"raw/summary unexpected-clearance mismatch for {method}")
            stats[method] = {
                "n": len(rows), "successes": success_count,
                "success_rate": success_rate,
                "ik_mean_ms": float(np.mean([
                    r["metrics"]["ik_solve_time_total_s"] for r in rows
                ]) * 1000.0),
                "wall_mean_s": float(np.mean([
                    r["metrics"]["episode_wall_time_s"] for r in rows
                ])),
                "clearance_mean_mm": float(clearance.mean() * 1000.0),
            }
        return {"valid": True, "stats": stats,
                "records_path": str(records_path), "summary_path": str(summary_path)}
    except Exception as exc:
        return {"valid": False, "warning": f"Stage 3 benchmark unavailable or inconsistent: {exc}",
                "records_path": str(records_path), "summary_path": str(summary_path)}


def make_task_camera():
    """Return the same fixed free-camera view for each independent model."""
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.36, 0.05, 0.32]
    camera.distance = 1.8
    camera.azimuth = 135
    camera.elevation = -25
    return camera


def solver_for(method: str):
    """Return and sanity-check the established implementation for a panel."""
    solver = SOLVERS[method]
    expected_name = f"solve_{method}"
    if solver.__name__ != expected_name:
        raise RuntimeError(f"{method} is routed to {solver.__name__}, expected {expected_name}")
    return solver


def run_headless_check(speed: float, velocity_limit: float, output: Path) -> dict:
    """Step three independent panel engines in lockstep without a GUI."""
    panels = []
    for method in METHODS:
        model, data, *_ = load_model()
        panel = Panel(method, model, data, None, {}, velocity_limit=velocity_limit)
        panel.reset()
        panels.append(panel)

    timesteps = {panel.model.opt.timestep for panel in panels}
    if len(timesteps) != 1:
        raise AssertionError(f"model instances have different physics timesteps: {timesteps}")
    dt = next(iter(timesteps))
    total_steps = int(sum(phase[2] for phase in demo._task_phases()) / dt) + 2
    for _ in range(total_steps):
        for panel in panels:
            panel.advance_one_step()
        if all(panel.done for panel in panels):
            break

    records = []
    for panel in panels:
        if panel.result is None:
            raise AssertionError(f"{panel.method} did not complete its episode")
        result = dict(panel.result)
        waypoint_methods = {item.get("method") for item in result["waypoints"]}
        if waypoint_methods != {SOLVER_RESULT_METHOD[panel.method]}:
            raise AssertionError(f"{panel.method} panel called unexpected solvers: {waypoint_methods}")
        result["episode_status"] = "completed_success" if result["success"] else "failed"
        records.append(result)

    starts = [record["initial_cube_xyz_m"] for record in records]
    targets = [record["place_target_xyz_m"] for record in records]
    if starts[0] != starts[1] or starts[0] != starts[2]:
        raise AssertionError("headless methods did not start from the same cube pose")
    if targets[0] != targets[1] or targets[0] != targets[2]:
        raise AssertionError("headless methods did not receive the same placement target")
    for record in records:
        if not record["success"]:
            raise RuntimeError(f"{record['method']} headless task failed: {record['failure_reason']}")

    payload = {
        "mode": "headless_synchronized_panel_check",
        "simultaneous_gui_run": False,
        "headless_unpaced": True,
        "gui_speed_sim_seconds_per_wall_second": speed,
        "velocity_limit_rad_s_mink_qp": velocity_limit,
        "initial_cube_xyz_m": starts[0],
        "placement_target_xyz_m": targets[0],
        "methods": records,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps({
        "mode": "headless synchronized panel-engine check (no GUI rendering)",
        "identical_start_and_target": True,
        "results": [{
            "method": row["method"], "solver_function": row["solver_function"],
            "success": row["success"], "failure_reason": row["failure_reason"],
            "final_cube_xyz_m": row["final_cube_xyz_m"],
            "completed_phase_count": row["completed_phase_count"],
        } for row in records],
        "log": str(output),
    }, indent=2), flush=True)
    return payload


class Panel:
    """One independently simulated, IK-controlled robot and its live view."""

    def __init__(self, method: str, model, data, renderer, widgets, velocity_limit: float,
                 camera=None):
        self.method = method
        self.solver = solver_for(method)
        self.model = model
        self.data = data
        self.renderer = renderer
        self.camera = make_task_camera() if camera is None else camera
        self.widgets = widgets
        self.velocity_limit = velocity_limit
        self.qpos_ids, self.dof_ids, self.actuator_ids, self.site_id, self.grip_id = demo.arm_indices(model)
        self.cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        self.phase_index = 0
        self.phase_step = 0
        self.phase_steps = 0
        self.phase_q_start = None
        self.phase_q_goal = None
        self.current_phase = "Ready"
        self.solver_status = "not started"
        self.position_error_m = None
        self.orientation_error_rad = None
        self.failure_reason = None
        self.success = False
        self.done = True
        self.completed_phases = 0
        self.lift_peak_z = 0.0
        self.start_cube = None
        self.rotation = None
        self.records = []
        self.result = None

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        demo._set_initial_cube(self.model, self.data)
        self.phase_index = 0
        self.phase_step = 0
        self.completed_phases = 0
        self.failure_reason = None
        self.success = False
        self.done = False
        self.records = []
        self.result = None
        self.rotation = self.data.site_xmat[self.site_id].reshape(3, 3).copy()
        self.start_cube = self.data.xpos[self.cube_id].copy()
        self.lift_peak_z = float(self.start_cube[2])
        self.current_phase = demo.PHASE_NAMES[0]
        self._solve_current_phase()

    def _solve_current_phase(self):
        if self.phase_index >= len(demo.PHASE_NAMES):
            self._finish()
            return
        target_xyz, grip_command, duration = demo._task_phases()[self.phase_index]
        self.current_phase = demo.PHASE_NAMES[self.phase_index]
        kwargs = {} if self.method == "dls" else {"velocity_limit": self.velocity_limit}
        try:
            result = self.solver(self.model, self.data, self.qpos_ids, self.dof_ids,
                                 self.site_id, target_xyz, self.rotation, **kwargs)
            record = result.to_dict()
            if record.get("method") != SOLVER_RESULT_METHOD[self.method]:
                raise RuntimeError(f"solver returned method {record.get('method')!r}")
        except Exception as exc:
            self._fail(f"solver_exception:{self.current_phase}:{type(exc).__name__}:{exc}")
            return
        self.position_error_m = record["position_error_m"]
        self.orientation_error_rad = record["orientation_error_rad"]
        self.solver_status = record["status"]
        record.update({"phase": self.current_phase, "target_xyz_m": target_xyz.tolist()})
        self.records.append(record)

        if record["status"] in {"solver_error", "infeasible", "solver_exception"}:
            self._fail(f"ik_solver_failure:{self.current_phase}:{record['status']}")
            return
        if (record["position_error_m"] > demo.EXECUTION_POSITION_LIMIT_M
                or record["orientation_error_rad"] > demo.EXECUTION_ORIENTATION_LIMIT_RAD):
            self._fail(f"ik_residual_over_execution_threshold:{self.current_phase}")
            return

        _, _, duration = demo._task_phases()[self.phase_index]
        self.phase_steps = int(duration / self.model.opt.timestep)
        self.phase_step = 0
        self.phase_q_start = self.data.qpos[self.qpos_ids].copy()
        self.phase_q_goal = np.asarray(result.q, dtype=float)
        if record["status"] == "max_iterations":
            self.solver_status = "max_iterations (accepted residual)"

    def advance_one_step(self):
        if self.done:
            return
        target_xyz, grip_command, _ = demo._task_phases()[self.phase_index]
        self.phase_step += 1
        u = self.phase_step / max(self.phase_steps, 1)
        blend = u * u * (3.0 - 2.0 * u)
        q_target = self.phase_q_start + (self.phase_q_goal - self.phase_q_start) * blend
        baseline.ctrl_step(self.model, self.data, self.qpos_ids, self.dof_ids,
                           self.actuator_ids, q_target, self.grip_id, grip_command)
        if grip_command == 255 and target_xyz[2] > baseline.PICK[2] + 0.13 + 0.05:
            self.lift_peak_z = max(self.lift_peak_z, float(self.data.xpos[self.cube_id, 2]))
        if self.phase_step >= self.phase_steps:
            self.completed_phases += 1
            self.phase_index += 1
            self._solve_current_phase()

    def _fail(self, reason: str):
        self.failure_reason = reason
        self.done = True
        self.success = False
        self.solver_status = "FAILED"
        self._make_result()

    def _finish(self):
        final_cube = self.data.xpos[self.cube_id].copy()
        lifted = bool(self.lift_peak_z > float(self.start_cube[2]) + 0.05)
        placement_error = float(np.linalg.norm(final_cube[:2] - baseline.PLACE[:2]))
        if not lifted:
            self.failure_reason = "cube_not_lifted"
        elif not (abs(final_cube[0] - baseline.PLACE[0]) < 0.07
                  and abs(final_cube[1] - baseline.PLACE[1]) < 0.07
                  and 0.13 < final_cube[2] < 0.22):
            self.failure_reason = "placement_error"
        self.success = self.failure_reason is None
        self.done = True
        self._make_result(final_cube=final_cube, lifted=lifted,
                          placement_error=placement_error)

    def _make_result(self, final_cube=None, lifted=None, placement_error=None):
        if final_cube is None:
            final_cube = self.data.xpos[self.cube_id].copy()
        if lifted is None:
            lifted = bool(self.lift_peak_z > float(self.start_cube[2]) + 0.05)
        if placement_error is None:
            placement_error = float(np.linalg.norm(final_cube[:2] - baseline.PLACE[:2]))
        self.result = {
            "method": self.method, "solver_function": self.solver.__name__,
            "success": bool(self.success), "failure_reason": self.failure_reason,
            "initial_cube_xyz_m": self.start_cube.tolist(), "initial_cube_yaw_rad": 0.0,
            "pick_target_xyz_m": baseline.PICK.tolist(),
            "place_target_xyz_m": baseline.PLACE.tolist(),
            "final_cube_xyz_m": final_cube.tolist(),
            "placement_error_xy_m": placement_error,
            "lift_peak_z_m": self.lift_peak_z, "lifted": bool(lifted),
            "completed_phase_count": self.completed_phases,
            "sim_time_s": float(self.data.time), "waypoints": self.records,
        }

    def status_text(self):
        if self.done:
            if self.result is None:
                return "Ready · press Start / Reset All"
            if self.success:
                return f"SUCCESS · {self.completed_phases}/8 phases · cube {np.round(self.result['final_cube_xyz_m'], 3)} m"
            return f"FAILED · {self.failure_reason}"
        if self.position_error_m is None:
            ik = self.solver_status
        else:
            ik = (f"{self.solver_status} · pos {self.position_error_m:.4f} m · "
                  f"ori {self.orientation_error_rad:.4f} rad")
        return f"{self.current_phase} · {ik}"

    def render(self):
        self.renderer.update_scene(self.data, camera=self.camera)
        return self.renderer.render()


class ParallelViewer:
    def __init__(self, speed: float, velocity_limit: float, output: Path):
        import tkinter as tk
        from PIL import Image, ImageTk

        self.tk = tk
        self.Image = Image
        self.ImageTk = ImageTk
        self.speed = speed
        self.velocity_limit = velocity_limit
        self.output = output
        self.root = tk.Tk()
        self.root.title("HEAL Pick-and-Place · DLS | Mink | QP")
        self.root.configure(bg="#111820")
        self.root.geometry("1740x600")
        self.live_geometry = "1740x600"
        self.results_geometry = "1740x850"
        self.root.minsize(1260, 480)
        self.models = []
        self.renderers = []
        self.panels = []
        self.photo_refs = []
        self.sim_accumulator = 0.0
        self.last_tick = time.perf_counter()
        self.results_written = False

        header = tk.Frame(self.root, bg="#111820")
        header.pack(fill="x", padx=16, pady=(12, 6))
        tk.Label(header, text="HEAL · ROBOTIQ 2F-85 · LIVE IK COMPARISON",
                 font=("TkDefaultFont", 17, "bold"), fg="white", bg="#111820").pack(side="left")
        self.speed_label = tk.Label(header, text=f"Playback: {speed:g} sim s / wall s",
                                    font=("TkDefaultFont", 11), fg="#c7d5df", bg="#111820")
        self.speed_label.pack(side="left", padx=20)
        tk.Button(header, text="Start / Reset All", command=self.start_or_reset,
                  font=("TkDefaultFont", 12, "bold"), padx=14, pady=7).pack(side="right")

        body = tk.Frame(self.root, bg="#111820")
        body.pack(fill="both", expand=True, padx=12, pady=8)
        colors = {"dls": "#55d6a0", "mink": "#63b8ff", "qp": "#ffc36b"}
        for index, method in enumerate(METHODS):
            model, data, *_ = load_model()
            mujoco.mj_resetData(model, data)
            demo._set_initial_cube(model, data)
            self.models.append((model, data))
            renderer = mujoco.Renderer(model, height=RENDER_HEIGHT, width=RENDER_WIDTH)
            self.renderers.append(renderer)
            frame = tk.Frame(body, bg="#1b2731", highlightthickness=2,
                             highlightbackground=colors[method])
            frame.grid(row=0, column=index, sticky="nsew", padx=5)
            body.columnconfigure(index, weight=1, uniform="panels")
            body.rowconfigure(0, weight=1)
            tk.Label(frame, text=method.upper(), font=("TkDefaultFont", 22, "bold"),
                     fg=colors[method], bg="#1b2731").pack(fill="x", pady=(8, 3))
            status = tk.Label(frame, text="Ready · press Start / Reset All",
                              font=("TkDefaultFont", 10), fg="white", bg="#1b2731",
                              wraplength=RENDER_WIDTH-15, justify="center", height=3)
            status.pack(fill="x", padx=5)
            image_label = tk.Label(frame, bg="black")
            image_label.pack(fill="both", expand=True, padx=5, pady=(3, 8))
            panel = Panel(method, model, data, renderer,
                          {"status": status, "image": image_label}, velocity_limit,
                          camera=make_task_camera())
            self.panels.append(panel)

        # Kept outside the live panel grid: it is hidden during motion and
        # appears beneath the panels only after all three have terminated.
        self.results_frame = tk.Frame(self.root, bg="#101820",
                                      highlightthickness=1, highlightbackground="#526675")
        self.results_title = tk.Label(
            self.results_frame, text=BENCHMARK_TITLE,
            font=("TkDefaultFont", 17, "bold"), fg="white", bg="#101820")
        self.results_title.pack(anchor="w", padx=16, pady=(10, 5))
        self.results_content = tk.Frame(self.results_frame, bg="#101820")
        self.results_content.pack(fill="x", padx=16, pady=(0, 12))
        self.results_frame.pack_forget()

        self.root.bind("<space>", lambda _event: self.start_or_reset())
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._render_panels()
        self.root.after(TICK_MS, self.tick)

    def start_or_reset(self):
        self.sim_accumulator = 0.0
        self.results_frame.pack_forget()
        self.root.geometry(self.live_geometry)
        for panel in self.panels:
            panel.reset()
        self.results_written = False
        self.last_tick = time.perf_counter()
        self._render_panels()

    def tick(self):
        now = time.perf_counter()
        elapsed = min(max(now - self.last_tick, 0.0), 0.2)
        self.last_tick = now
        dt = self.models[0][0].opt.timestep
        self.sim_accumulator += elapsed * self.speed
        steps = int(self.sim_accumulator / dt)
        if steps:
            self.sim_accumulator -= steps * dt
            # Every active instance receives every due MuJoCo timestep.
            # Rendering is less frequent, but physics steps are never skipped.
            for _ in range(steps):
                for panel in self.panels:
                    panel.advance_one_step()
        self._render_panels()
        if all(panel.done for panel in self.panels) and any(panel.result for panel in self.panels):
            if not self.results_written:
                self._save_results()
                self._show_benchmark()
                self.results_written = True
        self.root.after(TICK_MS, self.tick)

    def _render_panels(self):
        self.photo_refs.clear()
        for panel in self.panels:
            panel.widgets["status"].configure(text=panel.status_text())
            frame = panel.render()
            frame = np.asarray(frame)
            if frame.shape != (RENDER_HEIGHT, RENDER_WIDTH, 3) or frame.dtype != np.uint8:
                raise RuntimeError(f"Unexpected MuJoCo frame format: {frame.shape} {frame.dtype}")
            image = self.Image.fromarray(frame, mode="RGB")
            photo = self.ImageTk.PhotoImage(image=image)
            panel.widgets["image"].configure(image=photo)
            self.photo_refs.append(photo)

    def _save_results(self):
        payload = {
            "mode": "single_window_simultaneous_panels",
            "simultaneous_simulation": True,
            "speed_sim_seconds_per_wall_second": self.speed,
            "initial_cube_xyz_m": self.panels[0].result["initial_cube_xyz_m"],
            "initial_cube_yaw_rad": 0.0,
            "pick_target_xyz_m": baseline.PICK.tolist(),
            "place_target_xyz_m": baseline.PLACE.tolist(),
            "methods": [panel.result for panel in self.panels],
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        output = self.output
        if output.exists():
            output = output.with_name(output.stem + "_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + output.suffix)
        output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(json.dumps({
            "mode": payload["mode"], "log": str(output),
            "results": [{"method": row["method"], "success": row["success"],
                         "failure_reason": row["failure_reason"],
                         "final_cube_xyz_m": row["final_cube_xyz_m"]}
                        for row in payload["methods"]],
        }, indent=2), flush=True)

    def _show_benchmark(self):
        """Show validated, saved Stage 3 metrics below the completed live panels."""
        for child in self.results_content.winfo_children():
            child.destroy()
        benchmark = load_stage3_benchmark()
        if not benchmark["valid"]:
            self.results_content.configure(bg="#101820")
            self.tk.Label(
                self.results_content,
                text=benchmark["warning"],
                font=("TkDefaultFont", 13, "bold"), fg="#ffb4a9", bg="#101820",
                justify="left", wraplength=1600,
            ).pack(anchor="w", pady=(4, 8))
            print(benchmark["warning"], file=sys.stderr, flush=True)
        else:
            headers = ("METHOD", "SUCCESS", "MEAN IK SOLVE (ms)",
                       "MEAN EPISODE WALL TIME (s)", "MEAN MIN UNEXPECTED CLEARANCE (mm)")
            for col, title in enumerate(headers):
                self.tk.Label(
                    self.results_content, text=title,
                    font=("TkDefaultFont", 11, "bold"), fg="#b9cddd", bg="#101820",
                    padx=12, pady=5, anchor="w",
                ).grid(row=0, column=col, sticky="ew")
            colors = {"dls": "#55d6a0", "mink": "#63b8ff", "qp": "#ffc36b"}
            for row_index, method in enumerate(METHODS, start=1):
                values = benchmark["stats"][method]
                cells = (
                    method.upper(),
                    f"{values['successes']}/{values['n']}  ({values['success_rate']:.1%})",
                    f"{values['ik_mean_ms']:.2f}",
                    f"{values['wall_mean_s']:.3f}",
                    f"{values['clearance_mean_mm']:.3f}",
                )
                for col, value in enumerate(cells):
                    self.tk.Label(
                        self.results_content, text=value,
                        font=("TkDefaultFont", 13, "bold" if col == 0 else "normal"),
                        fg=colors[method] if col == 0 else "#f4f7fb", bg="#101820",
                        padx=12, pady=4, anchor="w",
                    ).grid(row=row_index, column=col, sticky="ew")
            self.tk.Label(
                self.results_content,
                text="Benchmark values are validated from saved Stage 3 raw episodes and summary; these are separate from the three live runs above.",
                font=("TkDefaultFont", 10), fg="#a9bac9", bg="#101820", anchor="w",
            ).grid(row=4, column=0, columnspan=len(headers), sticky="ew", padx=12, pady=(7, 2))
            print("Validated Stage 3 benchmark loaded from "
                  f"{benchmark['records_path']} and {benchmark['summary_path']}", flush=True)
        self.results_frame.pack(fill="x", padx=12, pady=(0, 10))
        self.root.geometry(self.results_geometry)

    def close(self):
        for renderer in self.renderers:
            renderer.close()
        self.root.destroy()

    def run(self):
        print("One Tk window contains three independently rendered MuJoCo model/data instances.", flush=True)
        print("Press 'Start / Reset All' (or Space) to run all three solvers together.", flush=True)
        print(f"Speed: {self.speed:g} simulated seconds / wall second", flush=True)
        self.root.mainloop()


def dump_debug_frames(directory: Path) -> None:
    """Render one shared-camera start frame from each actual model instance."""
    from PIL import Image

    directory.mkdir(parents=True, exist_ok=True)
    for method in METHODS:
        model, data, *_ = load_model()
        mujoco.mj_resetData(model, data)
        demo._set_initial_cube(model, data)
        renderer = mujoco.Renderer(model, height=RENDER_HEIGHT, width=RENDER_WIDTH)
        try:
            renderer.update_scene(data, camera=make_task_camera())
            frame = np.asarray(renderer.render())
            if frame.shape != (RENDER_HEIGHT, RENDER_WIDTH, 3) or frame.dtype != np.uint8:
                raise RuntimeError(f"Unexpected MuJoCo frame format for {method}: {frame.shape} {frame.dtype}")
            output = directory / f"{method}.png"
            Image.fromarray(frame, mode="RGB").save(output)
            print(f"{method.upper()}: saved actual RGB MuJoCo frame {frame.shape} to {output}")
        finally:
            renderer.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=float, default=0.6,
                        help="simulation seconds per wall-clock second (default: 0.6; larger is faster)")
    parser.add_argument("--velocity-limit", type=float, default=1.2,
                        help="Mink/QP per-joint velocity limit in rad/s (default: 1.2)")
    parser.add_argument("--output", type=Path, default=ROOT / "results/live_runs" / datetime.now(timezone.utc).strftime("run_%Y%m%dT%H%M%S%fZ") / "episode.json",
                        help="JSON output path for the last simultaneous episode")
    parser.add_argument("--headless-check", action="store_true",
                        help="run all three panel simulations in lockstep without rendering a GUI")
    parser.add_argument("--dump-frames", type=Path, metavar="DIR",
                        help="render and save one initial task-scene PNG per method, then exit")
    args = parser.parse_args(argv)
    if args.speed <= 0 or args.velocity_limit <= 0:
        parser.error("speed and velocity limit must be positive")

    # Explicit routing check also guards against accidentally substituting a solver.
    routes = {method: solver_for(method).__name__ for method in METHODS}
    print("Solver routing:", json.dumps(routes, sort_keys=True), flush=True)
    if args.headless_check:
        if args.dump_frames is not None:
            parser.error("--headless-check and --dump-frames cannot be used together")
        run_headless_check(args.speed, args.velocity_limit, args.output)
        return 0
    if args.dump_frames is not None:
        dump_debug_frames(args.dump_frames)
        return 0

    try:
        app = ParallelViewer(args.speed, args.velocity_limit, args.output)
    except Exception as exc:
        parser.error(
            "could not initialize the combined MuJoCo/Tk rendering window. "
            "This runner requires a graphical display, Tk, Pillow, and MuJoCo offscreen rendering. "
            f"Rendering initialization failed: {type(exc).__name__}: {exc}"
        )
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
