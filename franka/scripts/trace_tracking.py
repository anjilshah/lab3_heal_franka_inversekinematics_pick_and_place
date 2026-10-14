"""Capture command-versus-state traces for one matched seed and selected solvers."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "utils"))
from core import METHODS, load_model, run_episode


def geom_name(model, geom_id):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or f"geom_{geom_id}"


def run_trace(method, model, data, indices, seed, cube_xy, place_xy):
    qpos_ids, dof_ids, actuator_ids, site_id, _, _ = indices
    scratch = mujoco.MjData(model)
    rows = []

    def capture(model, live, phase, step_index):
        q_command = live.ctrl[actuator_ids].copy()
        scratch.qpos[:] = live.qpos
        scratch.qvel[:] = live.qvel
        scratch.qpos[qpos_ids] = q_command
        mujoco.mj_forward(model, scratch)
        actual_rotation = live.site_xmat[site_id].reshape(3, 3).copy()
        command_rotation = scratch.site_xmat[site_id].reshape(3, 3).copy()
        contacts = []
        for i in range(live.ncon):
            contact = live.contact[i]
            contacts.append({
                "geom1": geom_name(model, contact.geom1),
                "geom2": geom_name(model, contact.geom2),
                "distance_m": float(contact.dist),
            })
        rows.append({
            "sim_time_s": float(live.time), "phase": phase, "step_index": int(step_index),
            "commanded_joint_positions_rad": q_command.tolist(),
            "actual_joint_positions_rad": live.qpos[qpos_ids].copy().tolist(),
            "actual_joint_velocities_rad_s": live.qvel[dof_ids].copy().tolist(),
            "joint_tracking_error_rad": (q_command-live.qpos[qpos_ids]).tolist(),
            "commanded_ee_xyz_m": scratch.site_xpos[site_id].copy().tolist(),
            "actual_ee_xyz_m": live.site_xpos[site_id].copy().tolist(),
            "ee_position_tracking_error_m": float(np.linalg.norm(
                scratch.site_xpos[site_id]-live.site_xpos[site_id])),
            "commanded_ee_rotation": command_rotation.reshape(-1).tolist(),
            "actual_ee_rotation": actual_rotation.reshape(-1).tolist(),
            "contacts": contacts,
        })

    episode = run_episode(method, model, data, indices, seed=seed,
                          cube_xy=cube_xy, place_xy=place_xy,
                          frame_callback=capture)
    trace_summary = {
        "samples": len(rows),
        "max_joint_tracking_error_rad": (max(
            float(np.max(np.abs(row["joint_tracking_error_rad"]))) for row in rows)
            if rows else None),
        "max_ee_position_tracking_error_m": (max(
            row["ee_position_tracking_error_m"] for row in rows) if rows else None),
        "mean_ee_position_tracking_error_m": (float(np.mean([
            row["ee_position_tracking_error_m"] for row in rows])) if rows else None),
        "observed_contact_pairs": sorted({
            f"{pair['geom1']}<->{pair['geom2']}"
            for row in rows for pair in row["contacts"]
        }),
    }
    return episode, rows, trace_summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=20261016)
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=["mink", "qp"])
    parser.add_argument("--output", type=Path,
                        default=ROOT / "results" / "tracking_trace_seed_20261016")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    model, data, *rest = load_model()
    indices = tuple(rest)
    rng = np.random.default_rng(args.seed)
    cube_xy = np.array([0.18, -0.16]) + rng.uniform([-.025, -.025], [.025, .025])
    place_xy = np.array([0.18, 0.16]) + rng.uniform([-.025, -.025], [.025, .025])
    records = []
    with (args.output / "episodes.jsonl").open("w") as episode_file:
        for method in args.methods:
            episode, trace, summary = run_trace(
                method, model, data, indices, args.seed, cube_xy, place_xy)
            episode["matched_spec"] = {
                "seed": args.seed, "cube_xy": cube_xy.tolist(),
                "place_xy": place_xy.tolist(),
            }
            trace_path = f"{method}_trace.jsonl"
            with (args.output / trace_path).open("w") as trace_file:
                for row in trace:
                    trace_file.write(json.dumps(row, allow_nan=False) + "\n")
            episode["tracking_trace_file"] = trace_path
            episode["tracking_trace_summary"] = summary
            episode_file.write(json.dumps(episode, allow_nan=False) + "\n")
            episode_file.flush()
            records.append({"method": method, **summary,
                            "success": episode["success"],
                            "failure_reason": episode["failure_reason"],
                            "target_error_m": episode["target_error_m"],
                            "lift_peak_z_m": episode["lift_peak_z_m"],
                            "final_cube_z_m": episode["final_cube_z_m"],
                            "path_checks": episode["path_checks"],
                            "unexpected_contacts": episode["collision_diagnostics"]["unexpected_contact_pairs"]})
    (args.output / "summary.json").write_text(json.dumps({
        "seed": args.seed, "cube_xy": cube_xy.tolist(),
        "place_xy": place_xy.tolist(), "methods": records,
        "note": "Command/state trace from executed control steps. The normal shared path gate and runtime collision audit remained enabled.",
    }, indent=2) + "\n")
    print(json.dumps(records, indent=2))


if __name__ == "__main__":
    main()
