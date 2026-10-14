#!/usr/bin/env python3
"""Small comparable IK smoke test and one matched pick/place per solver."""
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

import pick_and_place as original  # noqa: E402
from robot_common import arm_indices, load_model  # noqa: E402
from utils.ik_solvers import SOLVERS  # noqa: E402


def solve_one(method, model, data, indices, target, rotation, velocity_limit):
    qpos, dof, _, site, _ = indices
    kwargs = {} if method == "dls" else {"velocity_limit": velocity_limit}
    return SOLVERS[method](model, data, qpos, dof, site, target, rotation, **kwargs)


def make_episode_spec(seed):
    """Generate one deterministic, modestly randomized shared task condition."""
    rng = np.random.default_rng(int(seed))
    return {
        "seed": int(seed),
        "initial_cube_xy_m": [float(v) for v in rng.uniform([0.34, -0.01], [0.36, 0.01])],
        "initial_cube_yaw_rad": float(rng.uniform(-0.10, 0.10)),
        "place_target_xy_m": [float(v) for v in rng.uniform([0.34, 0.17], [0.36, 0.19])],
    }


def run_episode(method, model, data, place_xy=None, cube_xy=None, velocity_limit=1.2,
                collision_monitor=None, seed=7, episode_spec=None):
    episode_started = time.perf_counter()
    qpos, dof, acts, site, grip = arm_indices(model)
    cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    spec = dict(episode_spec or make_episode_spec(seed))
    if cube_xy is not None:
        spec["initial_cube_xy_m"] = [float(v) for v in cube_xy]
    if place_xy is not None:
        spec["place_target_xy_m"] = [float(v) for v in place_xy]
    mujoco.mj_resetData(model, data)
    cube_q = model.body_jntadr[cube_id]
    qadr = model.jnt_qposadr[cube_q]
    cube_xy = np.asarray(spec["initial_cube_xy_m"], dtype=float)
    place_xy = np.asarray(spec["place_target_xy_m"], dtype=float)
    cube_yaw = float(spec["initial_cube_yaw_rad"])
    data.qpos[qadr:qadr+3] = [cube_xy[0], cube_xy[1], 0.165]
    data.qpos[qadr+3:qadr+7] = [np.cos(cube_yaw/2), 0, 0, np.sin(cube_yaw/2)]
    grip_height = original.PICK[2] + 0.13
    phases = [
        (np.array([cube_xy[0], cube_xy[1], grip_height+0.12]), 0, 2.0),
        (np.array([cube_xy[0], cube_xy[1], grip_height]), 0, 2.5),
        (np.array([cube_xy[0], cube_xy[1], grip_height]), 255, 1.2),
        (np.array([cube_xy[0], cube_xy[1], grip_height+0.18]), 255, 2.5),
        (np.array([place_xy[0], place_xy[1], grip_height+0.18]), 255, 2.5),
        (np.array([place_xy[0], place_xy[1], grip_height]), 255, 2.0),
        (np.array([place_xy[0], place_xy[1], grip_height]), 0, 1.0),
        (np.array([place_xy[0], place_xy[1], grip_height+0.12]), 0, 1.5),
    ]
    mujoco.mj_forward(model, data)
    start_cube = data.xpos[cube_id].copy()
    rotation = data.site_xmat[site].reshape(3, 3).copy()
    lift_peak = float(start_cube[2])
    solver_records = []
    failure = None
    executed_phases = 0
    for phase_index, (xyz, command, duration) in enumerate(phases):
        phase_name = ("approach", "descend", "close_gripper", "lift", "transfer",
                      "lower_place", "release", "retreat")[phase_index]
        try:
            result = solve_one(method, model, data, (qpos, dof, acts, site, grip),
                               xyz, rotation, velocity_limit)
            rec = result.to_dict()
        except Exception as exc:
            rec = {
                "method": method, "status": "solver_exception", "message": f"{type(exc).__name__}: {exc}",
                "position_error_m": None, "orientation_error_rad": None,
                "ik_iterations": None, "backend_iterations": None,
                "solve_time_s": None, "joint_position_limit_violation_rad": None,
                "velocity_limit_violation_rad_s": None,
                "velocity_limit_rad_s": None if method == "dls" else velocity_limit,
                "velocity_limit_enforcement": "not_enforced" if method == "dls" else "hard_constraint",
            }
        rec["phase"] = phase_name
        rec["target_xyz_m"] = xyz.tolist()
        rec["execution_decision"] = "not_run"
        solver_records.append(rec)
        if rec["status"] in {"solver_error", "infeasible", "solver_exception"}:
            failure = f"ik_waypoint_failure:{phase_name}:{rec['status']}"
            rec["execution_decision"] = "episode_aborted_solver_failure"
            break
        # Retain the original script's waypoint rejection threshold.
        if result.position_error_m > 0.06 or result.orientation_error_rad > 0.25:
            failure = f"ik_waypoint_failure:{phase_name}:residual_over_execution_threshold"
            rec["execution_decision"] = "episode_aborted_residual_over_threshold"
            break
        rec["execution_decision"] = "executed"
        nsteps = int(duration / model.opt.timestep)
        qstart = data.qpos[qpos].copy()
        for step in range(nsteps):
            u = (step + 1) / max(nsteps, 1)
            blend = u*u*(3.0 - 2.0*u)
            target_q = qstart + (np.asarray(result.q) - qstart) * blend
            original.ctrl_step(model, data, qpos, dof, acts, target_q, grip, command)
            if collision_monitor is not None:
                collision_monitor.update(data, phase_index)
            if command == 255 and xyz[2] > grip_height + 0.05:
                lift_peak = max(lift_peak, float(data.xpos[cube_id, 2]))
        executed_phases += 1
    final = data.xpos[cube_id].copy()
    place = np.array([place_xy[0], place_xy[1], original.PICK[2]])
    lifted = bool(lift_peak > start_cube[2] + 0.05)
    success = bool(lifted and abs(final[0]-place[0]) < 0.07
                   and abs(final[1]-place[1]) < 0.07 and 0.13 < final[2] < 0.22)
    if failure is None:
        if not lifted:
            failure = "cube_not_lifted"
        elif not (abs(final[0]-place[0]) < 0.07 and abs(final[1]-place[1]) < 0.07):
            failure = "placement_error"
    audit = collision_monitor.summary() if collision_monitor is not None else None
    if (audit is not None and audit["unexpected_actual_contact_observations_dist_le_0"] > 0
            and failure is None):
        failure = "unexpected_collision_contact"
    success = bool(failure is None and lifted
                   and abs(final[0]-place[0]) < 0.07 and abs(final[1]-place[1]) < 0.07
                   and 0.13 < final[2] < 0.22)
    return {
        "method": method, "success": success, "failure_reason": failure,
        "episode_status": "completed_success" if success else ("aborted" if failure and failure.startswith("ik_waypoint_failure") else "completed_failure"),
        "seed": int(spec["seed"]),
        "initial_cube_xyz": start_cube.tolist(),
        "initial_cube_yaw_rad": cube_yaw,
        "pick_target_xyz_m": [float(cube_xy[0]), float(cube_xy[1]), float(original.PICK[2])],
        "place_target_xyz": place.tolist(), "final_cube_xyz": final.tolist(),
        "lift_peak_z_m": lift_peak, "sim_time_s": float(data.time),
        "executed_phase_count": executed_phases,
        "ik_solve_time_total_s": float(sum(r["solve_time_s"] for r in solver_records if r.get("solve_time_s") is not None)),
        "ik_iterations_total": (sum(r["ik_iterations"] for r in solver_records if r.get("ik_iterations") is not None)
                                if any(r.get("ik_iterations") is not None for r in solver_records) else None),
        "max_position_residual_m": max((r["position_error_m"] for r in solver_records if r.get("position_error_m") is not None), default=None),
        "max_orientation_residual_rad": max((r["orientation_error_rad"] for r in solver_records if r.get("orientation_error_rad") is not None), default=None),
        "max_joint_position_limit_violation_rad": max((r["joint_position_limit_violation_rad"] for r in solver_records if r.get("joint_position_limit_violation_rad") is not None), default=None),
        "velocity_limit_rad_s": None if method == "dls" else velocity_limit,
        "velocity_limit_enforcement": "not_enforced" if method == "dls" else "hard_constraint",
        "max_velocity_limit_violation_rad_s": (None if method == "dls" else max((r["velocity_limit_violation_rad_s"] for r in solver_records if r.get("velocity_limit_violation_rad_s") is not None), default=None)),
        "episode_wall_time_s": time.perf_counter() - episode_started,
        "sim_time_scope": "MuJoCo simulation clock; excludes wall-clock rendering and planning interpretation",
        "episode_wall_time_scope": "wall-clock duration for the full episode, including IK solves and simulation steps",
        "waypoints": solver_records,
        "collision_audit": audit,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=str(ROOT / "results/ik_smoke"))
    parser.add_argument("--velocity-limit", type=float, default=1.2,
                        help="Mink/QP cap in rad/s per HEAL arm joint")
    args = parser.parse_args()
    model, data, qpos, dof, acts, site, grip = load_model()
    indices = (qpos, dof, acts, site, grip)
    mujoco.mj_forward(model, data)
    initial_rotation = data.site_xmat[site].reshape(3, 3).copy()
    # All methods receive one identical, reachable in-workspace pose target.
    target = np.array([0.35, 0.0, 0.38])
    smoke = []
    for method in ("dls", "mink", "qp"):
        result = solve_one(method, model, data, indices, target, initial_rotation,
                           args.velocity_limit)
        smoke.append(result.to_dict())
    # Matched conditions: the same initial cube and destination for each method.
    episodes = [run_episode(method, model, data, original.PLACE[:2].copy(),
                            original.PICK[:2].copy(), args.velocity_limit)
                for method in ("mink", "dls", "qp")]
    payload = {
        "setup": {
            "model": "robot_descriptions/arm_gripper.xml",
            "arm_joints": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            "end_effector_site": "right_center", "qp_backend": "DAQP",
            "mink_qp_backend": "DAQP", "velocity_limit_rad_s": args.velocity_limit,
            "smoke_target_xyz_m": target.tolist(),
        },
        "metric_definitions": {
            "solve_time_s": "wall-clock time for one complete iterative IK solver call; not full episode time",
            "ik_iterations": "outer method updates: DLS updates, Mink solve_ik calls, or direct-QP subproblem calls",
            "backend_iterations": "null because the selected Mink/DAQP API does not expose its internal QP iteration count",
            "joint_position_limit_violation_rad": "maximum positive excursion of generated arm configurations outside model joint ranges, measured across solver updates after applied constraints/clipping",
            "velocity_limit_violation_rad_s": "maximum per-update arm velocity above the configured hard cap; null means no time-based cap is enforced",
            "episode_wall_time_s": "full episode wall-clock duration; distinct from sim_time_s",
        },
        "smoke_tests": smoke,
        "matched_episodes": episodes,
    }
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "smoke_and_episodes.json").write_text(json.dumps(payload, indent=2))
    print(json.dumps({
        "smoke_tests": [{k: r[k] for k in ("method", "status", "position_error_m",
                                               "orientation_error_rad", "ik_iterations", "backend_iterations",
                                               "ik_iteration_definition", "solve_time_s",
                                               "joint_position_limit_violation_rad", "velocity_limit_violation_rad_s",
                                               "velocity_limit_enforcement",
                                               "message")} for r in smoke],
        "matched_episodes": [{k: e[k] for k in ("method", "success", "failure_reason",
                                                     "initial_cube_xyz", "place_target_xyz", "final_cube_xyz",
                                                     "lift_peak_z_m", "sim_time_s")} for e in episodes],
        "log": str(output / "smoke_and_episodes.json"),
    }, indent=2))


if __name__ == "__main__":
    main()
