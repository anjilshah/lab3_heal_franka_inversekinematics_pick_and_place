#!/usr/bin/env python3
"""Live, method-selectable HEAL + Robotiq pick-and-place demonstration.

Uses the established IK implementations in utils/ik_solvers.py and the
original baseline torque tracker. No trajectory is prerecorded.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

import pick_and_place as baseline  # noqa: E402
from robot_common import arm_indices, load_model  # noqa: E402
from utils.ik_solvers import SOLVERS  # noqa: E402


METHODS = ("dls", "mink", "qp")
PHASE_NAMES = ("approach", "descend", "close_gripper", "lift",
               "transfer", "lower_place", "release", "retreat")
EXECUTION_POSITION_LIMIT_M = 0.06
EXECUTION_ORIENTATION_LIMIT_RAD = 0.25


def _json_safe(value):
    """Replace non-finite solver outputs with JSON null."""
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, (float, np.floating)) and not math.isfinite(float(value)):
        return None
    if isinstance(value, np.integer):
        return int(value)
    return value


def _task_phases():
    """Return the shared baseline pick/lift/transfer/place sequence."""
    pick = baseline.PICK
    place = baseline.PLACE
    grip_height = float(pick[2] + 0.13)
    return [
        (np.array([pick[0], pick[1], grip_height + 0.12]), 0, 2.0),
        (np.array([pick[0], pick[1], grip_height]), 0, 2.5),
        (np.array([pick[0], pick[1], grip_height]), 255, 1.2),
        (np.array([pick[0], pick[1], grip_height + 0.18]), 255, 2.5),
        (np.array([place[0], place[1], grip_height + 0.18]), 255, 2.5),
        (np.array([place[0], place[1], grip_height]), 255, 2.0),
        (np.array([place[0], place[1], grip_height]), 0, 1.0),
        (np.array([place[0], place[1], grip_height + 0.12]), 0, 1.5),
    ]


def _set_initial_cube(model, data):
    cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_joint = int(model.body_jntadr[cube_id])
    qadr = int(model.jnt_qposadr[cube_joint])
    data.qpos[qadr:qadr + 3] = baseline.PICK
    data.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    return cube_id


def run_demo(model, data, method, *, viewer=None, velocity_limit=1.2,
             playback_rate=0.6, pace=True):
    """Run one actual simulated episode; optionally sync/pacing through a viewer.

    ``pace=False`` exists for repeatable no-GUI code-path checks. A real viewer
    run should leave pace enabled so the simulated motion is recordable.
    """
    if method not in METHODS:
        raise ValueError(f"unknown method {method!r}; choose one of {METHODS}")
    if playback_rate <= 0:
        raise ValueError("playback_rate must be positive")

    qpos_ids, dof_ids, actuator_ids, site_id, grip_id = arm_indices(model)
    mujoco.mj_resetData(model, data)
    cube_id = _set_initial_cube(model, data)
    phases = _task_phases()
    rotation = data.site_xmat[site_id].reshape(3, 3).copy()
    start_cube = data.xpos[cube_id].copy()
    lift_peak_z = float(start_cube[2])
    waypoints = []
    completed_phases = 0
    failure_reason = None
    interrupted = False

    print(f"\n=== IK viewer demo: {method.upper()} ===", flush=True)
    print(f"Cube start: {start_cube.tolist()} m | target: {baseline.PLACE.tolist()} m",
          flush=True)
    print(f"Selected speed: {playback_rate:g} simulation seconds / wall second "
          "(larger values run faster)",
          flush=True)

    for phase_index, ((target_xyz, grip_command, duration), phase_name) in enumerate(
            zip(phases, PHASE_NAMES)):
        if viewer is not None and not viewer.is_running():
            interrupted = True
            failure_reason = "viewer_closed_before_episode_finished"
            break

        kwargs = {} if method == "dls" else {"velocity_limit": velocity_limit}
        solve_started = time.perf_counter()
        try:
            result = SOLVERS[method](model, data, qpos_ids, dof_ids, site_id,
                                     target_xyz, rotation, **kwargs)
            record = result.to_dict()
        except Exception as exc:  # keep solver failure visible in the saved result
            solve_elapsed = time.perf_counter() - solve_started
            failure_reason = f"solver_exception:{phase_name}:{type(exc).__name__}:{exc}"
            record = {
                "method": method,
                "status": "solver_exception",
                "message": f"{type(exc).__name__}: {exc}",
                "position_error_m": None,
                "orientation_error_rad": None,
                "ik_iterations": None,
                "backend_iterations": None,
                "solve_time_s": solve_elapsed,
                "joint_position_limit_violation_rad": None,
                "velocity_limit_violation_rad_s": None,
                "velocity_limit_rad_s": None if method == "dls" else velocity_limit,
                "velocity_limit_enforcement": "not_enforced" if method == "dls" else "hard_constraint",
                "solve_time_scope": "wall_clock_for_one_complete_iterative_ik_call",
            }
            record.update({"phase": phase_name, "target_xyz_m": target_xyz.tolist(),
                           "execution_decision": "aborted_solver_exception"})
            waypoints.append(record)
            print(f"{phase_name}: SOLVER EXCEPTION — {failure_reason}", flush=True)
            break

        record.update({"phase": phase_name, "target_xyz_m": target_xyz.tolist(),
                       "execution_decision": "not_run"})
        waypoints.append(record)
        print(
            f"{phase_name}: {record['status']} | "
            f"position={record['position_error_m']:.5f} m, "
            f"orientation={record['orientation_error_rad']:.5f} rad, "
            f"solve={record['solve_time_s']:.4f} s",
            flush=True,
        )

        if record["status"] in {"solver_error", "infeasible", "solver_exception"}:
            failure_reason = f"ik_solver_failure:{phase_name}:{record['status']}"
            record["execution_decision"] = "aborted_solver_failure"
            print(f"{phase_name}: not executed — {failure_reason}", flush=True)
            break
        if (record["position_error_m"] > EXECUTION_POSITION_LIMIT_M
                or record["orientation_error_rad"] > EXECUTION_ORIENTATION_LIMIT_RAD):
            failure_reason = f"ik_residual_over_execution_threshold:{phase_name}"
            record["execution_decision"] = "aborted_residual_over_threshold"
            print(f"{phase_name}: not executed — {failure_reason}", flush=True)
            break

        if record["status"] == "max_iterations":
            print(f"{phase_name}: executing returned pose; residuals pass existing "
                  "0.06 m / 0.25 rad controller thresholds", flush=True)
        record["execution_decision"] = "executed"
        nsteps = int(duration / model.opt.timestep)
        q_start = data.qpos[qpos_ids].copy()
        phase_wall_start = time.perf_counter()
        # Advance physics at every timestep but render at about 60 Hz. At the
        # default speed this keeps the picture smooth without paying for a
        # viewer synchronization on all 500 physics steps per simulated second.
        render_stride = max(1, int(round(1.0 / (60.0 * model.opt.timestep))))
        for step in range(nsteps):
            if viewer is not None and not viewer.is_running():
                interrupted = True
                failure_reason = f"viewer_closed_during:{phase_name}"
                record["execution_decision"] = "interrupted"
                break
            blend_u = (step + 1) / max(nsteps, 1)
            blend = blend_u * blend_u * (3.0 - 2.0 * blend_u)
            q_target = q_start + (np.asarray(result.q) - q_start) * blend
            baseline.ctrl_step(model, data, qpos_ids, dof_ids, actuator_ids,
                               q_target, grip_id, grip_command)
            if grip_command == 255 and target_xyz[2] > baseline.PICK[2] + 0.13 + 0.05:
                lift_peak_z = max(lift_peak_z, float(data.xpos[cube_id, 2]))
            if viewer is not None and (
                    (step + 1) % render_stride == 0 or step == nsteps - 1):
                viewer.sync()
                if pace:
                    target_elapsed = (step + 1) * float(model.opt.timestep) / playback_rate
                    remaining = target_elapsed - (time.perf_counter() - phase_wall_start)
                    if remaining > 0:
                        time.sleep(remaining)
        if interrupted:
            break
        completed_phases += 1

    final_cube = data.xpos[cube_id].copy()
    lifted = bool(lift_peak_z > float(start_cube[2]) + 0.05)
    place = baseline.PLACE.copy()
    placement_error_xy = float(np.linalg.norm(final_cube[:2] - place[:2]))
    if failure_reason is None and not lifted:
        failure_reason = "cube_not_lifted"
    if failure_reason is None and not (
            abs(final_cube[0] - place[0]) < 0.07
            and abs(final_cube[1] - place[1]) < 0.07
            and 0.13 < final_cube[2] < 0.22):
        failure_reason = "placement_error"
    success = failure_reason is None and not interrupted
    return {
        "method": method,
        "success": bool(success),
        "episode_status": "completed_success" if success else ("interrupted" if interrupted else "failed"),
        "failure_reason": failure_reason,
        "initial_cube_xyz_m": start_cube.tolist(),
        "initial_cube_yaw_rad": 0.0,
        "pick_target_xyz_m": baseline.PICK.tolist(),
        "place_target_xyz_m": place.tolist(),
        "final_cube_xyz_m": final_cube.tolist(),
        "placement_error_xy_m": placement_error_xy,
        "lift_peak_z_m": lift_peak_z,
        "lifted": lifted,
        "completed_phase_count": completed_phases,
        "sim_time_s": float(data.time),
        "velocity_limit_rad_s": None if method == "dls" else velocity_limit,
        "velocity_limit_enforcement": "not_enforced" if method == "dls" else "hard_constraint",
        "waypoints": waypoints,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--method", choices=METHODS, required=True,
                        help="IK solver to demonstrate")
    parser.add_argument("--velocity-limit", type=float, default=1.2,
                        help="Mink/QP per-joint limit in rad/s (default: 1.2)")
    parser.add_argument("--speed", "--playback-rate", dest="speed", type=float, default=0.6,
                        help=("simulation seconds per wall-clock second; larger is faster. "
                              "Default 0.6 gives about 25 seconds of simulated task time, "
                              "plus solver/render overhead."))
    parser.add_argument("--output", default=str(ROOT / "results/viewer_demo"),
                        help="directory for one JSON result per selected method")
    parser.add_argument("--headless", action="store_true",
                        help="run the same task without a viewer or real-time pacing (for checks)")
    parser.add_argument("--close-on-completion", action="store_true",
                        help="close the viewer after the task completes instead of waiting for the user")
    args = parser.parse_args()
    if args.velocity_limit <= 0 or args.speed <= 0:
        parser.error("velocity limit and speed must be positive")

    model, data, *_ = load_model()
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    if args.headless:
        result = run_demo(model, data, args.method,
                          velocity_limit=args.velocity_limit,
                          playback_rate=args.speed, pace=False)
        viewer_was_used = False
    else:
        print("Opening MuJoCo passive viewer. Keep the terminal visible for the "
              "selected-method and phase status.", flush=True)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Use MuJoCo's supported passive-viewer text overlay. 250% font
            # scale is legible in screen recordings; the viewer renders this
            # continuously at the top-left until the passive window closes.
            label = f"IK METHOD: {args.method.upper()}"
            viewer.set_texts((
                mujoco.mjtFontScale.mjFONTSCALE_250,
                mujoco.mjtGridPos.mjGRID_TOPLEFT,
                label,
                "HEAL | ROBOTIQ 2F-85",
            ))
            print(f"Viewer overlay: {label}", flush=True)
            result = run_demo(model, data, args.method, viewer=viewer,
                              velocity_limit=args.velocity_limit,
                              playback_rate=args.speed, pace=True)
            viewer_was_used = True
            if not result["success"]:
                print(f"Episode ended without success: {result['failure_reason']}", flush=True)
            if (result["episode_status"] == "completed_success"
                    and not args.close_on_completion):
                print("Episode completed. Close the MuJoCo viewer to exit.", flush=True)
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.02)

    result["viewer_requested"] = not args.headless
    result["viewer_context_used"] = viewer_was_used
    # Keep the existing output key for consumers of Stage 7 demo records.
    result["playback_rate_sim_seconds_per_wall_second"] = args.speed
    output_path = output_dir / f"{args.method}.json"
    output_path.write_text(json.dumps(_json_safe(result), indent=2, allow_nan=False))
    print(json.dumps({
        "method": result["method"],
        "episode_status": result["episode_status"],
        "success": result["success"],
        "failure_reason": result["failure_reason"],
        "completed_phase_count": result["completed_phase_count"],
        "initial_cube_xyz_m": result["initial_cube_xyz_m"],
        "place_target_xyz_m": result["place_target_xyz_m"],
        "final_cube_xyz_m": result["final_cube_xyz_m"],
        "sim_time_s": result["sim_time_s"],
        "log": str(output_path),
    }, indent=2), flush=True)
    if not result["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
