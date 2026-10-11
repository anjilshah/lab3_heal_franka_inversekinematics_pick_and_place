#!/usr/bin/env python3
"""Small collision-aware HEAL pick/place demonstrator and safety test.

The original pick_and_place.py remains the baseline. This opt-in path uses
finite sampled joint-space segment checks plus a post-step clearance guard.
It rejects unsafe segments; it does not search arbitrary obstacle-free paths.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

import pick_and_place as baseline  # noqa: E402
from robot_common import load_model  # noqa: E402
from utils.collision_audit import CollisionMonitor  # noqa: E402
from utils.collision_aware import ClearancePlanner  # noqa: E402
from utils.ik_solvers import solve_dls  # noqa: E402


DEFAULT_CLEARANCE_MARGIN_M = 0.00075
EXECUTION_POSITION_LIMIT_M = 0.06
EXECUTION_ORIENTATION_LIMIT_RAD = 0.25
PLACE_TCP_HEIGHT_M = baseline.PICK[2] + 0.13 + 0.02


def run_episode(model, data, *, case_name="nominal", pick_tcp_height=0.295,
                clearance_margin_m=DEFAULT_CLEARANCE_MARGIN_M, path_samples=25):
    started = time.perf_counter()
    qpos, dof, acts, site, grip = __import__("robot_common").arm_indices(model)
    cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_joint = int(model.body_jntadr[cube_id])
    cube_qadr = int(model.jnt_qposadr[cube_joint])
    mujoco.mj_resetData(model, data)
    data.qpos[cube_qadr:cube_qadr+3] = [baseline.PICK[0], baseline.PICK[1], 0.165]
    data.qpos[cube_qadr+3:cube_qadr+7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)

    monitor = CollisionMonitor(model)
    planner = ClearancePlanner(model, data, monitor, qpos, site,
                               clearance_margin_m, path_samples)
    rotation = data.site_xmat[site].reshape(3, 3).copy()
    phases = [
        (np.array([baseline.PICK[0], baseline.PICK[1], pick_tcp_height+0.12]), 0, 2.0),
        (np.array([baseline.PICK[0], baseline.PICK[1], pick_tcp_height]), 0, 2.5),
        (np.array([baseline.PICK[0], baseline.PICK[1], pick_tcp_height]), 255, 1.2),
        (np.array([baseline.PICK[0], baseline.PICK[1], pick_tcp_height+0.18]), 255, 2.5),
        (np.array([baseline.PLACE[0], baseline.PLACE[1], pick_tcp_height+0.18]), 255, 2.5),
        (np.array([baseline.PLACE[0], baseline.PLACE[1], PLACE_TCP_HEIGHT_M]), 255, 2.0),
        (np.array([baseline.PLACE[0], baseline.PLACE[1], PLACE_TCP_HEIGHT_M]), 0, 1.0),
        (np.array([baseline.PLACE[0], baseline.PLACE[1], pick_tcp_height+0.12]), 0, 1.5),
    ]
    phase_names = ("approach", "descend", "close_gripper", "lift", "transfer",
                   "lower_place", "release", "retreat")
    start_cube = data.xpos[cube_id].copy()
    lift_peak_z = float(start_cube[2])
    waypoints = []
    failure_reason = None
    completed_phases = 0

    def execute_segment(q_goal, phase_index, command, duration, held_transform):
        """Validate, then track one joint segment with a live distance guard."""
        nonlocal lift_peak_z
        q_start_local = data.qpos[qpos].copy()
        check = planner.validate_segment(q_start_local, q_goal, phase_index, held_transform)
        if not check["safe"]:
            return check, f"unsafe_segment:{phase_names[phase_index]}:{check['first_violation_pair']}"
        nsteps = int(duration / model.opt.timestep)
        for step in range(nsteps):
            u = (step+1)/max(nsteps, 1)
            blend = u*u*(3.0-2.0*u)
            target_q = q_start_local + (np.asarray(q_goal)-q_start_local)*blend
            baseline.ctrl_step(model, data, qpos, dof, acts, target_q, grip, command)
            monitor.update(data, phase_index)
            if command == 255 and xyz_for_phase[phase_index][2] > pick_tcp_height + 0.05:
                lift_peak_z = max(lift_peak_z, float(data.xpos[cube_id, 2]))
            if monitor.unexpected_actual_contact_observations > 0:
                return check, f"runtime_unexpected_contact:{phase_names[phase_index]}"
            runtime_safe, guard_info = planner.runtime_safe()
            if not runtime_safe:
                return check, (f"runtime_clearance_guard:{phase_names[phase_index]}:"
                               f"{guard_info['pair']}:clearance={guard_info['minimum_unexpected_clearance_m']:.9g}m")
        return check, None

    xyz_for_phase = [p[0] for p in phases]
    for phase_index, (xyz, command, duration) in enumerate(phases):
        phase_name = phase_names[phase_index]
        result = solve_dls(model, data, qpos, dof, site, xyz, rotation)
        waypoint = {"phase": phase_name, "target_xyz_m": xyz.tolist(), **result.to_dict()}
        waypoint["execution_decision"] = "not_run"
        waypoints.append(waypoint)
        if result.status in {"solver_error", "infeasible", "solver_exception"}:
            failure_reason = f"ik_solver_failure:{phase_name}:{result.status}"
            waypoint["execution_decision"] = "aborted_solver_failure"
            break
        if (result.position_error_m > EXECUTION_POSITION_LIMIT_M
                or result.orientation_error_rad > EXECUTION_ORIENTATION_LIMIT_RAD):
            failure_reason = f"ik_residual_over_execution_limit:{phase_name}"
            waypoint["execution_decision"] = "aborted_residual"
            break

        # Refresh the kinematic carry approximation at each segment. This
        # reduces drift from the contact-based (not welded) simulated grasp.
        held_transform = (planner.capture_tcp_to_cube()
                          if phase_index in (3, 4, 5) else None)
        validation = planner.validate_segment(data.qpos[qpos].copy(), result.q,
                                             phase_index, held_transform)
        waypoint["segment_clearance_check"] = validation
        fallback_checks = []
        runtime_failure = None
        if not validation["safe"] and phase_index in (1, 5):
            # Joint interpolation can sweep a wrist mesh through the table even
            # when the Cartesian endpoint is clear. Refine vertical approach
            # into short Cartesian steps, validating each before it is tracked.
            current_z = float(data.site_xpos[site][2])
            count = max(2, int(np.ceil(abs(current_z-xyz[2])/0.02)))
            waypoint["cartesian_refinement_attempted"] = True
            for sub_z in np.linspace(current_z, xyz[2], count+1)[1:]:
                sub_target = np.array([xyz[0], xyz[1], float(sub_z)])
                sub_result = solve_dls(model, data, qpos, dof, site, sub_target, rotation)
                if (sub_result.position_error_m > EXECUTION_POSITION_LIMIT_M
                        or sub_result.orientation_error_rad > EXECUTION_ORIENTATION_LIMIT_RAD
                        or sub_result.status in {"solver_error", "infeasible", "solver_exception"}):
                    runtime_failure = f"ik_residual_over_execution_limit:{phase_name}:cartesian_refinement"
                    break
                sub_check, runtime_failure = execute_segment(
                    sub_result.q, phase_index, command, duration/count, held_transform)
                fallback_checks.append({"target_xyz_m": sub_target.tolist(),
                                        "ik_status": sub_result.status,
                                        "ik_position_error_m": sub_result.position_error_m,
                                        "ik_orientation_error_rad": sub_result.orientation_error_rad,
                                        "segment_clearance_check": sub_check})
                if runtime_failure is not None:
                    break
            waypoint["cartesian_refinement_segments"] = fallback_checks
            if runtime_failure is None:
                waypoint["execution_decision"] = "executed_with_cartesian_clearance_substeps"
            else:
                failure_reason = runtime_failure
                waypoint["execution_decision"] = "aborted_refined_clearance_check"
        elif not validation["safe"]:
            failure_reason = (f"unsafe_segment:{phase_name}:{validation['first_violation_pair']}"
                              f":clearance={validation['first_violation_distance_m']:.9g}m")
            waypoint["execution_decision"] = "aborted_clearance_precheck"
        else:
            check, runtime_failure = execute_segment(
                result.q, phase_index, command, duration, held_transform)
            if runtime_failure is not None:
                failure_reason = runtime_failure
                waypoint["runtime_guard_trigger"] = monitor.last_unexpected_minimum_pair
                waypoint["execution_decision"] = "aborted_runtime_guard"
            else:
                waypoint["execution_decision"] = "executed_after_clearance_precheck"
        if failure_reason is not None:
            break
        completed_phases += 1

    final_cube = data.xpos[cube_id].copy()
    lifted = bool(lift_peak_z > start_cube[2] + 0.05)
    placed = bool(abs(final_cube[0]-baseline.PLACE[0]) < 0.07
                  and abs(final_cube[1]-baseline.PLACE[1]) < 0.07
                  and 0.13 < final_cube[2] < 0.22)
    if failure_reason is None and not lifted:
        failure_reason = "cube_not_lifted"
    if failure_reason is None and not placed:
        failure_reason = "placement_error"
    success = bool(failure_reason is None and lifted and placed)
    collision_summary = monitor.summary()
    collision_summary["collision_avoidance_enforced_by_controller"] = True
    collision_summary["waypoint_prechecks"] = True
    collision_summary["collision_avoidance_enforcement_scope"] = (
        "sampled segment-clearance rejection plus runtime clearance/contact abort; not a general planner"
    )
    return {
        "case": case_name,
        "success": success,
        "episode_status": "completed_success" if success else "aborted" if failure_reason and (
            failure_reason.startswith(("unsafe_segment", "runtime_", "ik_solver_failure", "ik_residual"))) else "completed_failure",
        "failure_reason": failure_reason,
        "initial_cube_xyz_m": start_cube.tolist(),
        "initial_cube_yaw_rad": 0.0,
        "pick_target_xyz_m": [float(baseline.PICK[0]), float(baseline.PICK[1]), float(pick_tcp_height-0.13)],
        "place_target_xyz_m": baseline.PLACE.tolist(),
        "final_cube_xyz_m": final_cube.tolist(),
        "lift_peak_z_m": lift_peak_z,
        "sim_time_s": float(data.time),
        "episode_wall_time_s": time.perf_counter()-started,
        "clearance_policy": {
            "minimum_unexpected_clearance_m": clearance_margin_m,
            "precheck": f"{path_samples} sampled points on the same smooth joint interpolation used for execution",
            "runtime_guard": "after each physics step, abort next control update if unexpected clearance is below margin or unexpected actual contact occurs",
            "held_cube_model": "during lift/transfer/lower path checks, rigid TCP-to-cube transform captured after close is used; actual physical state is separately monitored each step",
        },
        "release_tcp_height_m": PLACE_TCP_HEIGHT_M,
        "nominal_cube_release_drop_estimate_m": 0.025,
        "completed_phase_count": completed_phases,
        "waypoints": waypoints,
        "collision_audit": collision_summary,
        "active_avoidance_scope": "sampled waypoint-segment validation plus runtime abort guard; no general obstacle detour search or collision-constrained IK",
    }


def plot_clearance_checks(cases, output_path, margin_m):
    import os
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/lab3_mpl_config")
    Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    phases = ("approach", "descend", "close_gripper", "lift", "transfer",
              "lower_place", "release", "retreat")
    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    x = np.arange(len(phases))
    width = 0.36
    colors = ("#2874a6", "#c0392b")
    for case_index, case in enumerate(cases):
        by_phase = {}
        for waypoint in case["waypoints"]:
            checks = waypoint.get("cartesian_refinement_segments")
            if checks:
                vals = [s["segment_clearance_check"]["minimum_unexpected_clearance_m"]
                        for s in checks if s["segment_clearance_check"]["minimum_unexpected_clearance_m"] is not None]
            else:
                val = waypoint.get("segment_clearance_check", {}).get("minimum_unexpected_clearance_m")
                vals = [] if val is None else [val]
            if vals:
                by_phase[waypoint["phase"]] = min(vals)*1000.0
        heights = [by_phase.get(phase, np.nan) for phase in phases]
        ax.bar(x+(case_index-0.5)*width, heights, width, color=colors[case_index],
               label=case["case"].replace("_", " "))
    ax.axhline(margin_m*1000.0, color="black", linestyle="--", linewidth=1.4,
               label=f"Required margin ({margin_m*1000:.2f} mm)")
    ax.axhline(0.0, color="gray", linewidth=.8)
    ax.set_xticks(x, phases, rotation=20, ha="right")
    ax.set_ylabel("Minimum unexpected-pair signed clearance (mm)")
    ax.set_title("Sampled segment clearance checks; missing bars were not planned")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=.25)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="results_stage4")
    parser.add_argument("--clearance-margin-mm", type=float, default=0.75)
    parser.add_argument("--path-samples", type=int, default=25)
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    model, data, *_ = load_model()
    margin_m = args.clearance_margin_mm/1000.0
    nominal = run_episode(model, data, case_name="nominal",
                          clearance_margin_m=margin_m, path_samples=args.path_samples)
    obstructed = run_episode(model, data, case_name="low_pick_target_intersects_table_clearance",
                             pick_tcp_height=0.25, clearance_margin_m=margin_m,
                             path_samples=args.path_samples)
    payload = {
        "model": "robot_descriptions/arm_gripper.xml",
        "table": {"center_xyz_m": [0.36, 0.05, 0.12], "dimensions_xyz_m": [0.56, 0.64, 0.04], "top_z_m": 0.14},
        "tray": {"present": False, "placement_region": "existing placement coordinate only; no tray collision geometry added"},
        "assumptions": [
            "The active HEAL collision meshes and MuJoCo signed geom distances are the geometry authority.",
            "A 0.75 mm minimum margin is selected because the nominal gripper-to-table geometric gap is only a few millimetres; it is a simulation threshold, not a hardware safety rating.",
            "The cube is released with TCP 20 mm higher than the original grasp contact height; using the modeled 130 mm wrist-to-pad offset, this yields an estimated 25 mm cube drop to the table surface.",
            "Path checks sample 25 points on each planned smooth joint interpolation, so collisions between samples may be missed.",
            "The held cube follows a fixed TCP-relative transform only in kinematic path checks; the actual MuJoCo grasp remains contact-based and is monitored during execution.",
            "During close, held-object, and release kinematic checks, cube-to-gripper geom distances are excluded because the cube is intentionally nested in the finger envelope and the rigid hold proxy produced mesh overlap not present in recorded actual contacts; runtime MuJoCo contacts on non-excluded gripper geoms still trigger an abort.",
            "Unsafe segments are rejected. No arbitrary obstacle or tray geometry is introduced and no general detour search is claimed.",
        ],
        "avoidance_design": {
            "sampled_joint_segment_precheck": True,
            "runtime_clearance_abort_guard": True,
            "passive_contact_monitoring_alone": False,
            "general_collision_free_planning_claimed": False,
            "obstacle_detour_search": False,
            "collision_constrained_ik": False,
        },
        "cases": [nominal, obstructed],
    }
    result_path = out / "collision_aware_smoke.json"
    result_path.write_text(json.dumps(payload, indent=2, allow_nan=False))
    plot_path = out / "segment_clearance.png"
    plot_clearance_checks(payload["cases"], plot_path, margin_m)
    print(json.dumps({"results": [{"case": r["case"], "success": r["success"],
                                  "status": r["episode_status"], "failure_reason": r["failure_reason"],
                                  "completed_phases": r["completed_phase_count"],
                                  "sim_time_s": r["sim_time_s"],
                                  "min_unexpected_clearance_m": min(r["collision_audit"]["min_unexpected_signed_distance_by_role_pair_m"].values(), default=None),
                                  "unexpected_actual_contacts": r["collision_audit"]["unexpected_actual_contact_observations_dist_le_0"],
                                  "expected_contacts": r["collision_audit"]["expected_contact_observations"],
                                  "waypoint_checks": [{"phase": w["phase"], "decision": w["execution_decision"],
                                                       "safe": w.get("segment_clearance_check", {}).get("safe"),
                                                       "minimum_clearance_m": w.get("segment_clearance_check", {}).get("minimum_unexpected_clearance_m")}
                                                      for w in r["waypoints"]]}
                                 for r in (nominal, obstructed)], "log": str(result_path),
                      "clearance_plot": str(plot_path)}, indent=2))


if __name__ == "__main__":
    main()
