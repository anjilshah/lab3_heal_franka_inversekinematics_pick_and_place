"""Small matched runs with fresh FK clearance measurements at every physics step."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "utils"))
import core
from collision import CollisionAudit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--methods", nargs="+", choices=core.METHODS, default=list(core.METHODS))
    parser.add_argument("--seeds", nargs="+", type=int, default=[20261013, 20261014, 20261015])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    model, data, *rest = core.load_model()
    indices = tuple(rest)
    qpos, _, actuators, _, _, _ = indices
    # Mink filters some welded/parent body pairs when assembling IK limits.
    # Runtime validation uses all robot/table-floor pairs audited by the gate.
    pairs = [(a, b) for a, b, category, _ in CollisionAudit(model).pairs
             if category in ("robot-table", "robot-floor")]
    scratch = mujoco.MjData(model)
    manifest = {
        "methods": args.methods, "seeds": args.seeds, "matched_conditions": True,
        "execution_allowance_m": core.COLLISION_EXECUTION_ALLOWANCE_M,
        "numeric_clearance_slack_m": core.COLLISION_REFINEMENT_CLEARANCE_EPSILON_M,
        "calibration_source": "results/retreat_execution_calibration_20261013/summary.json",
        "success_criterion": {"final_xy_error_lt_m": 0.010,
                              "lift_peak_gt_initial_z_plus_m": 0.045,
                              "final_cube_z_open_interval_m": [0.39, 0.49]},
        "runtime_measurement": "Fresh FK of actual qpos after each physics step; signed Panda/table-floor geom distance.",
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    summaries = []
    with (args.output / "episodes.jsonl").open("w") as episode_file:
        for seed in args.seeds:
            rng = np.random.default_rng(seed)
            cube = np.array([.18, -.16]) + rng.uniform([-.025, -.025], [.025, .025])
            place = np.array([.18, .16]) + rng.uniform([-.025, -.025], [.025, .025])
            for method in args.methods:
                trace_name = f"{method}_{seed}_clearance.jsonl"
                samples = 0
                worst = None
                contacts = set()
                with (args.output / trace_name).open("w") as trace_file:
                    def capture(model, live, phase, frame_index):
                        nonlocal samples, worst
                        scratch.qpos[:] = live.qpos
                        scratch.qvel[:] = live.qvel
                        mujoco.mj_forward(model, scratch)
                        distances = [float(mujoco.mj_geomDistance(
                            model, scratch, a, b, 10.0, None)) for a, b in pairs]
                        closest = int(np.argmin(distances))
                        row = {"sim_time_s": float(live.time), "phase": phase,
                               "minimum_clearance_m": distances[closest],
                               "geom_pair_ids": list(pairs[closest]),
                               "q_actual_rad": live.qpos[qpos].copy().tolist(),
                               "q_command_rad": live.ctrl[actuators].copy().tolist()}
                        samples += 1
                        if worst is None or row["minimum_clearance_m"] < worst["minimum_clearance_m"]:
                            worst = row
                        for pair, distance in zip(pairs, distances):
                            if distance < 0.0:
                                contacts.add(tuple(pair))
                        trace_file.write(json.dumps(row) + "\n")

                    result = core.run_episode(method, model, data, indices, seed=seed,
                                              cube_xy=cube, place_xy=place, frame_callback=capture)
                if worst is not None and worst["minimum_clearance_m"] < 0.0:
                    result["success"] = False
                    result["failure_reason"] = result["failure_reason"] or "executed_robot_environment_penetration"
                result["matched_spec"] = {"seed": seed, "cube_xy": cube.tolist(), "place_xy": place.tolist()}
                result["executed_clearance_diagnostics"] = {
                    "samples": samples, "minimum_state": worst,
                    "penetrating_geom_pair_ids": sorted(contacts), "trace_file": trace_name,
                    "limitation": "Physics-step geometry samples, not a continuous-time safety certificate.",
                }
                episode_file.write(json.dumps(result, allow_nan=False) + "\n")
                episode_file.flush()
                summary = {
                    "method": method, "seed": seed, "success": result["success"],
                    "failure_reason": result["failure_reason"],
                    "planned_minimum_clearance_m": min((c["minimum_sampled_clearance_m"] for c in result["path_checks"]), default=None),
                    "executed_minimum_clearance_m": worst["minimum_clearance_m"] if worst else None,
                    "unexpected_contacts": result["collision_diagnostics"]["unexpected_contact_pairs"],
                    "penetrating_geom_pair_ids": sorted(contacts),
                    "completed_phases": [p["phase"] for p in result["phase_execution"] if p["steps"] > 0],
                    "final_xy_error_m": result["final_xy_error_m"],
                    "lift_peak_z_m": result["lift_peak_z_m"],
                    "final_cube_z_m": result["final_cube_z_m"],
                    "joint_limit_violation_rad": max((w["joint_limit_violation_rad"] for w in result["waypoints"]), default=None),
                }
                summaries.append(summary)
                (args.output / "summary.json").write_text(json.dumps({"episodes": summaries}, indent=2) + "\n")
                print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
