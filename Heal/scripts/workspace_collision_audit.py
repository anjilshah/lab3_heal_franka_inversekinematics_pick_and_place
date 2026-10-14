#!/usr/bin/env python3
"""Measured collision audit and sampled HEAL task/dexterous workspace maps."""
import argparse
import csv
import json
import os
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from ik_solver_smoke import run_episode  # noqa: E402
from robot_common import arm_indices, load_model  # noqa: E402
from utils.collision_audit import CollisionMonitor  # noqa: E402
from utils.ik_solvers import solve_dls  # noqa: E402


POSITION_TOLERANCE_M = 0.025
ORIENTATION_TOLERANCE_RAD = 0.05
SAMPLE_Z_M = (0.295, 0.415, 0.475)  # TCP pick, approach, and transfer heights.
YAW_DEGREES = (-90, -60, -30, 0, 30, 60, 90)
GRID_SIZE_XY = 9
TABLE_EDGE_BUFFER_M = 0.03


def collision_smoke(output):
    model, data, qpos, dof, acts, site, grip = load_model()
    monitor = CollisionMonitor(model)
    episode = run_episode("dls", model, data, np.array([0.35, 0.18]),
                          np.array([0.35, 0.0]), 1.2,
                          collision_monitor=monitor)
    record = {
        "test": "one deterministic nominal DLS pick-and-place with per-step collision/contact monitoring",
        "seed": 7,
        "initial_cube_xyz_m": [0.35, 0.0, 0.165],
        "initial_cube_yaw_rad": 0.0,
        "place_target_xyz_m": [0.35, 0.18, 0.165],
        "episode": episode,
        "collision_summary": episode["collision_audit"],
        "contact_failure": bool(episode["collision_audit"]["unexpected_contact_observations"]),
        "collision_avoidance_enforced_by_controller": False,
    }
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    (out / "nominal_collision_smoke.json").write_text(json.dumps(record, indent=2))
    c = record["collision_summary"]
    print(json.dumps({
        "success": episode["success"], "failure_reason": episode["failure_reason"],
        "sim_time_s": episode["sim_time_s"],
        "episode_wall_time_s": episode["episode_wall_time_s"],
        "steps_monitored": c["steps_monitored"],
        "distance_queries": c["distance_queries"],
        "monitored_pair_count": c["monitored_pair_count"],
        "min_signed_distance_by_role_pair_m": c["min_signed_distance_by_role_pair_m"],
        "min_unexpected_signed_distance_by_role_pair_m": c["min_unexpected_signed_distance_by_role_pair_m"],
        "actual_contact_pairs": c["actual_contact_pairs"],
        "expected_contact_observations": c["expected_contact_observations"],
        "unexpected_contact_observations": c["unexpected_contact_observations"],
        "unexpected_contact_pairs": c["unexpected_contact_pairs"],
        "penetrating_contact_observations": c["penetrating_contact_observations_lt_minus_1e_6_m"],
        "deepest_penetration_m": c["deepest_contact_penetration_m"],
        "unexpected_actual_contact_observations": c["unexpected_actual_contact_observations_dist_le_0"],
        "unexpected_penetrating_contact_observations": c["unexpected_penetrating_contact_observations_lt_minus_1e_6_m"],
        "deepest_unexpected_penetration_m": c["deepest_unexpected_penetration_m"],
        "collision_avoidance_enforced_by_controller": c["collision_avoidance_enforced_by_controller"],
        "log": str(out / "nominal_collision_smoke.json"),
    }, indent=2))


def _target_rotation(base, yaw_degrees):
    a = np.deg2rad(yaw_degrees)
    Rz = np.array([[np.cos(a), -np.sin(a), 0.0],
                   [np.sin(a), np.cos(a), 0.0],
                   [0.0, 0.0, 1.0]])
    return Rz @ base


def workspace_analysis(output):
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/lab3_mpl_config")
    Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    model, data, qpos, dof, _, site, _ = load_model()
    mujoco.mj_forward(model, data)
    table_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "worktable")
    table_center = data.geom_xpos[table_id].copy()
    table_half = model.geom_size[table_id].copy()
    cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    cube_half = model.geom_size[cube_id][:2]
    x_bounds = (table_center[0]-table_half[0], table_center[0]+table_half[0])
    y_bounds = (table_center[1]-table_half[1], table_center[1]+table_half[1])
    support_center_bounds = (x_bounds[0]+cube_half[0]+TABLE_EDGE_BUFFER_M,
                             x_bounds[1]-cube_half[0]-TABLE_EDGE_BUFFER_M,
                             y_bounds[0]+cube_half[1]+TABLE_EDGE_BUFFER_M,
                             y_bounds[1]-cube_half[1]-TABLE_EDGE_BUFFER_M)
    xs = np.linspace(support_center_bounds[0], support_center_bounds[1], GRID_SIZE_XY)
    ys = np.linspace(support_center_bounds[2], support_center_bounds[3], GRID_SIZE_XY)
    positions = [(float(x), float(y), "table_center_grid") for x in xs for y in ys]
    positions += [(0.35, 0.0, "nominal_pick"), (0.35, 0.18, "nominal_place")]
    base_rotation = data.site_xmat[site].reshape(3, 3).copy()
    ranges = []
    for joint_name in ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        ranges.append({"joint": joint_name, "lower_rad": float(model.jnt_range[jid, 0]),
                       "upper_rad": float(model.jnt_range[jid, 1])})
    records = []
    started = time.perf_counter()

    def solve_sample(sample_type, x, y, z, yaw, position_kind):
        target = np.array([x, y, z], dtype=float)
        rotation = _target_rotation(base_rotation, yaw)
        result = solve_dls(model, data, qpos, dof, site, target, rotation)
        reachable = bool(result.position_error_m <= POSITION_TOLERANCE_M
                         and result.orientation_error_rad <= ORIENTATION_TOLERANCE_RAD)
        records.append({
            "sample_type": sample_type,
            "position_sample_kind": position_kind,
            "x_m": x, "y_m": y, "z_m": z, "yaw_deg": yaw,
            "reachable": reachable, "solver_status": result.status,
            "position_error_m": result.position_error_m,
            "orientation_error_rad": result.orientation_error_rad,
            "ik_iterations": result.ik_iterations,
            "backend_iterations": result.backend_iterations,
            "ik_solve_time_s": result.solve_time_s,
            "joint_position_limit_violation_rad": result.joint_position_limit_violation_rad,
            "joint_solution_rad": json.dumps(result.q),
        })

    for x, y, kind in positions:
        for z in SAMPLE_Z_M:
            solve_sample("task", x, y, z, 0, kind)
            for yaw in YAW_DEGREES:
                solve_sample("dexterous", x, y, z, yaw, kind)

    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    csv_path = out / "workspace_samples.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)

    task_records = [r for r in records if r["sample_type"] == "task"]
    dex_records = [r for r in records if r["sample_type"] == "dexterous"]
    dex_summary = {}
    for x, y, kind in positions:
        for z in SAMPLE_Z_M:
            sample = [r for r in dex_records if r["x_m"] == x and r["y_m"] == y and r["z_m"] == z]
            good = sum(r["reachable"] for r in sample)
            dex_summary[(x, y, z, kind)] = {"feasible_yaw_count": int(good),
                                            "yaw_sample_count": len(sample),
                                            "all_yaws_reachable": bool(good == len(YAW_DEGREES))}
    center_rows = [r for r in task_records if r["z_m"] == SAMPLE_Z_M[0]
                   and r["position_sample_kind"] == "table_center_grid"]
    table_good = sum(r["reachable"] for r in center_rows)
    pick = [r for r in task_records if r["z_m"] == SAMPLE_Z_M[0] and r["x_m"] == 0.35 and r["y_m"] == 0.0]
    place = [r for r in task_records if r["z_m"] == SAMPLE_Z_M[0] and r["x_m"] == 0.35 and r["y_m"] == 0.18]
    dex_contact = [v for (x, y, z, _), v in dex_summary.items() if z == SAMPLE_Z_M[0]]
    dex_all_count = sum(v["all_yaws_reachable"] for v in dex_contact)
    summary = {
        "model": "robot_descriptions/arm_gripper.xml",
        "method": "custom DLS solver with true MuJoCo joint ranges",
        "sampling": {
            "method": "deterministic Cartesian grid; no random samples",
            "random_seed": 7,
            "seed_note": "seed recorded for reproducibility; grid sampling does not consume random numbers",
            "x_bounds_m": list(x_bounds), "y_bounds_m": list(y_bounds),
            "table_center_sampling_bounds_m": list(support_center_bounds),
            "table_edge_buffer_beyond_cube_half_extent_m": TABLE_EDGE_BUFFER_M,
            "z_levels_tcp_m": list(SAMPLE_Z_M),
            "xy_resolution": f"{GRID_SIZE_XY} x {GRID_SIZE_XY} grid plus exact nominal pick/place points",
            "yaw_range_deg": [min(YAW_DEGREES), max(YAW_DEGREES)],
            "yaw_samples_deg": list(YAW_DEGREES),
            "orientation_definition": "world-z yaw applied to the model default right_center orientation",
            "position_threshold_m": POSITION_TOLERANCE_M,
            "orientation_threshold_rad": ORIENTATION_TOLERANCE_RAD,
            "solver_max_iterations": 180,
            "dls_damping": 0.08,
        },
        "joint_limits_rad": ranges,
        "task_workspace": {
            "samples": len(task_records), "reachable": sum(r["reachable"] for r in task_records),
            "z_0_295_table_center_grid": {"samples": len(center_rows), "reachable": table_good,
                                             "fraction": table_good/len(center_rows)},
            "nominal_pick_reachable_at_tcp_z_0_295": bool(pick and pick[0]["reachable"]),
            "nominal_place_reachable_at_tcp_z_0_295": bool(place and place[0]["reachable"]),
            "reachable_sample_bounds_m": {
                axis: [min(r[axis] for r in task_records if r["reachable"]),
                       max(r[axis] for r in task_records if r["reachable"])]
                for axis in ("x_m", "y_m", "z_m") if any(r["reachable"] for r in task_records)
            },
        },
        "dexterous_workspace": {
            "samples": len(dex_records), "reachable_pose_samples": sum(r["reachable"] for r in dex_records),
            "yaw_samples_required_for_dexterous_classification": len(YAW_DEGREES),
            "contact_height_positions_all_yaws_reachable": dex_all_count,
            "contact_height_positions_sampled": len(dex_contact),
            "definition": "dexterous sample is reachable only when all seven yaw samples pass both residual thresholds",
        },
        "table": {
            "current_center_xyz_m": table_center.tolist(),
            "current_dimensions_xyz_m": [float(2*v) for v in table_half],
            "current_top_z_m": float(table_center[2]+table_half[2]),
            "coordinate_convention": "world x/y are horizontal table axes; z is vertical; dimensions are full extents",
            "cube_half_extents_xy_m": cube_half.tolist(),
            "recommended_dimensions_xyz_m": [float(2*table_half[0]), float(2*table_half[1]), float(2*table_half[2])],
            "recommendation": "retain current footprint for this task if both nominal targets and the sampled 50-mm edge-buffered center grid are reachable; this is a sampled workspace recommendation, not a structural table-design analysis",
            "edge_clearance_for_pick_center_m": {
                "x_min": float(0.35-x_bounds[0]), "x_max": float(x_bounds[1]-0.35),
                "y_min": float(0.0-y_bounds[0]), "y_max": float(y_bounds[1]-0.0),
            },
            "edge_clearance_for_place_center_m": {
                "x_min": float(0.35-x_bounds[0]), "x_max": float(x_bounds[1]-0.35),
                "y_min": float(0.18-y_bounds[0]), "y_max": float(y_bounds[1]-0.18),
            },
        },
        "runtime_wall_time_s": time.perf_counter()-started,
        "plots": ["workspace_task_xy_contact.png", "workspace_dexterous_xy_contact.png", "workspace_task_3d.png"],
        "sampling_limit": "Only sampled grid points are classified; unsampled positions and orientations are unknown.",
    }
    summary["table"]["recommendation"] = (
        "retain current footprint: both targets and all sampled edge-buffered table-center points are reachable at TCP z=0.295 m"
        if summary["task_workspace"]["nominal_pick_reachable_at_tcp_z_0_295"]
        and summary["task_workspace"]["nominal_place_reachable_at_tcp_z_0_295"]
        and table_good == len(center_rows)
        else "do not claim complete coverage: inspect reachable/infeasible samples before resizing"
    )

    _plot_task_xy(task_records, out, x_bounds, y_bounds, support_center_bounds, SAMPLE_Z_M[0])
    _plot_dex_xy(dex_records, dex_summary, positions, out, x_bounds, y_bounds,
                 support_center_bounds, SAMPLE_Z_M[0], len(YAW_DEGREES))
    _plot_task_3d(task_records, out, x_bounds, y_bounds, SAMPLE_Z_M[0])
    (out / "workspace_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps({
        "task_samples": summary["task_workspace"],
        "dexterous_contact_height": summary["dexterous_workspace"],
        "current_table_dimensions_xyz_m": summary["table"]["current_dimensions_xyz_m"],
        "recommendation": summary["table"]["recommendation"],
        "runtime_wall_time_s": summary["runtime_wall_time_s"],
        "samples_csv": str(csv_path), "summary": str(out / "workspace_summary.json"),
        "plots": summary["plots"],
    }, indent=2))


def _base_plot(ax, x_bounds, y_bounds, center_bounds):
    from matplotlib.patches import Rectangle
    ax.add_patch(Rectangle((x_bounds[0], y_bounds[0]), x_bounds[1]-x_bounds[0],
                           y_bounds[1]-y_bounds[0], fill=False, edgecolor="saddlebrown",
                           linewidth=2, label="Current table footprint"))
    ax.add_patch(Rectangle((center_bounds[0], center_bounds[2]), center_bounds[1]-center_bounds[0],
                           center_bounds[3]-center_bounds[2], fill=False, edgecolor="orange",
                           linestyle="--", linewidth=1.5, label="Cube-center region with 50 mm edge buffer"))
    ax.scatter([0.35], [0.0], marker="*", s=130, color="navy", label="Pick target")
    ax.scatter([0.35], [0.18], marker="X", s=85, color="purple", label="Place target")
    ax.set_xlabel("World x (m)")
    ax.set_ylabel("World y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=.25)


def _plot_task_xy(rows, out, xb, yb, center_bounds, z):
    import matplotlib.pyplot as plt
    # The regular center grid forms an occupancy projection; exact task targets
    # are shown with their own markers in _base_plot.
    rows = [r for r in rows if r["z_m"] == z and r["position_sample_kind"] == "table_center_grid"]
    fig, ax = plt.subplots(figsize=(8, 6), constrained_layout=True)
    _base_plot(ax, xb, yb, center_bounds)
    good = [r for r in rows if r["reachable"]]
    bad = [r for r in rows if not r["reachable"]]
    ax.scatter([r["x_m"] for r in good], [r["y_m"] for r in good], c="#229954", s=42, label=f"Reachable ({len(good)})")
    ax.scatter([r["x_m"] for r in bad], [r["y_m"] for r in bad], c="#c0392b", marker="x", s=46, label=f"Infeasible at sampled pose ({len(bad)})")
    ax.set_title(f"Task workspace at required orientation, TCP z={z:.3f} m (N={len(rows)})")
    ax.legend(loc="best", fontsize=8)
    fig.savefig(out / "workspace_task_xy_contact.png", dpi=180)
    plt.close(fig)


def _plot_dex_xy(rows, grouped, positions, out, xb, yb, center_bounds, z, required):
    import matplotlib.pyplot as plt
    grid = [p for p in positions if p[2] == "table_center_grid"]
    summaries = []
    for x, y, _ in grid:
        s = grouped[(x, y, z, "table_center_grid")]
        summaries.append((x, y, s["feasible_yaw_count"], s["all_yaws_reachable"]))
    fig, ax = plt.subplots(figsize=(8, 6), constrained_layout=True)
    _base_plot(ax, xb, yb, center_bounds)
    good = [s for s in summaries if s[3]]
    bad = [s for s in summaries if not s[3]]
    ax.scatter([s[0] for s in good], [s[1] for s in good], color="#229954", s=48,
               label=f"All {required} yaw samples feasible ({len(good)})")
    scatter = ax.scatter([s[0] for s in bad], [s[1] for s in bad], c=[s[2] for s in bad],
                         cmap="viridis", vmin=0, vmax=required, marker="x", s=55,
                         label=f"Not all yaw samples feasible ({len(bad)})")
    if bad:
        cb = fig.colorbar(scatter, ax=ax)
        cb.set_label(f"Feasible yaw samples out of {required}")
    ax.set_title(f"Dexterous workspace at TCP z={z:.3f} m, yaw −90°…+90° (N={len(summaries)})")
    ax.legend(loc="best", fontsize=8)
    fig.savefig(out / "workspace_dexterous_xy_contact.png", dpi=180)
    plt.close(fig)


def _plot_task_3d(rows, out, xb, yb, z_reference):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig = plt.figure(figsize=(9, 7), constrained_layout=True)
    ax = fig.add_subplot(111, projection="3d")
    good = [r for r in rows if r["reachable"]]
    bad = [r for r in rows if not r["reachable"]]
    ax.scatter([r["x_m"] for r in good], [r["y_m"] for r in good], [r["z_m"] for r in good],
               color="#229954", s=18, label=f"Reachable ({len(good)})")
    ax.scatter([r["x_m"] for r in bad], [r["y_m"] for r in bad], [r["z_m"] for r in bad],
               color="#c0392b", marker="x", s=22, label=f"Infeasible ({len(bad)})")
    z = z_reference
    verts = [[(xb[0], yb[0], z), (xb[1], yb[0], z), (xb[1], yb[1], z), (xb[0], yb[1], z)]]
    ax.add_collection3d(Poly3DCollection(verts, alpha=.12, facecolor="saddlebrown", edgecolor="saddlebrown"))
    ax.scatter([0.35], [0.0], [z], marker="*", s=100, color="navy", label="Pick xy")
    ax.scatter([0.35], [0.18], [z], marker="X", s=65, color="purple", label="Place xy")
    ax.set_xlabel("World x (m)")
    ax.set_ylabel("World y (m)")
    ax.set_zlabel("TCP z (m)")
    ax.set_title(f"Sampled task workspace (N={len(rows)}); table footprint projected at z={z:.3f} m")
    ax.legend(fontsize=8)
    fig.savefig(out / "workspace_task_3d.png", dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("collision", "workspace", "all"), default="all")
    parser.add_argument("--output", default=str(ROOT / "results/stage2"))
    args = parser.parse_args()
    if args.mode in ("collision", "all"):
        collision_smoke(args.output)
    if args.mode in ("workspace", "all"):
        workspace_analysis(args.output)


if __name__ == "__main__":
    main()
