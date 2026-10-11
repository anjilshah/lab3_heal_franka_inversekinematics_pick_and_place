#!/usr/bin/env python3
"""Seed-matched HEAL IK episode evaluation and reproducible summaries."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from ik_solver_smoke import make_episode_spec, run_episode  # noqa: E402
from robot_common import load_model  # noqa: E402
from utils.collision_audit import CollisionMonitor  # noqa: E402

METHODS = ("dls", "mink", "qp")
SEEDS = tuple(range(20261011, 20261036))
VELOCITY_LIMIT_RAD_S = 1.2
POSITION_TOLERANCE_M = 0.07
FINAL_Z_RANGE_M = (0.13, 0.22)


def _json_safe(value):
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, (np.floating, float)) and not np.isfinite(value):
        return None
    if isinstance(value, np.integer):
        return int(value)
    return value


def _one(method, seed, model, data):
    spec = make_episode_spec(seed)
    monitor = CollisionMonitor(model)
    result = run_episode(method, model, data, velocity_limit=VELOCITY_LIMIT_RAD_S,
                         collision_monitor=monitor, seed=seed, episode_spec=spec)
    result["episode_spec"] = spec
    result["comparison_task_definition"] = {
        "phases": ["approach", "descend", "close_gripper", "lift", "transfer",
                   "lower_place", "release", "retreat"],
        "waypoint_execution_residual_limit_m": 0.06,
        "waypoint_execution_residual_limit_rad": 0.25,
        "success_position_tolerance_m": POSITION_TOLERANCE_M,
        "success_final_cube_z_range_m": list(FINAL_Z_RANGE_M),
        "collision_rule": "unexpected actual contact dist<=0 is a failure; expected cube support and grasp contacts are excluded by the monitor rules",
    }
    result["metrics"] = {
        "ik_solve_time_total_s": result["ik_solve_time_total_s"],
        "episode_wall_time_s": result["episode_wall_time_s"],
        "ik_iterations_total": result["ik_iterations_total"],
        "backend_iterations": None,
        "max_position_residual_m": result["max_position_residual_m"],
        "max_orientation_residual_rad": result["max_orientation_residual_rad"],
        "max_joint_position_limit_violation_rad": result["max_joint_position_limit_violation_rad"],
        "velocity_limit_rad_s": result["velocity_limit_rad_s"],
        "velocity_limit_enforcement": result["velocity_limit_enforcement"],
        "max_velocity_limit_violation_rad_s": result["max_velocity_limit_violation_rad_s"],
    }
    audit = result.get("collision_audit")
    result["metrics"].update({
        "minimum_unexpected_clearance_m": (min(audit["min_unexpected_signed_distance_by_role_pair_m"].values())
                                           if audit and audit["min_unexpected_signed_distance_by_role_pair_m"] else None),
        "unexpected_contact_observations": (audit["unexpected_contact_observations"] if audit else None),
        "unexpected_actual_contact_observations": (audit["unexpected_actual_contact_observations_dist_le_0"] if audit else None),
        "unexpected_penetration_observations": (audit["unexpected_penetrating_contact_observations_lt_minus_1e_6_m"] if audit else None),
        "unexpected_distinct_contact_events": ((audit.get("distinct_contact_event_counts", {}).get("unexpected", 0)) if audit else None),
        "expected_distinct_contact_events": ((audit.get("distinct_contact_event_counts", {}).get("expected", 0)) if audit else None),
    })
    return result


def _smoke(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    model, data, *_ = load_model()
    records = [_one(m, SEEDS[0], model, data) for m in METHODS]
    specs = [r["episode_spec"] for r in records]
    assert specs[0] == specs[1] == specs[2], "matched seed did not yield identical task inputs"
    assert len({tuple(r["comparison_task_definition"]["phases"]) for r in records}) == 1
    assert all(r["episode_status"] in {"completed_success", "completed_failure", "aborted"} for r in records)
    assert all(r["collision_audit"] is not None for r in records)
    assert all(r["metrics"]["velocity_limit_enforcement"] == "not_enforced"
               and r["metrics"]["max_velocity_limit_violation_rad_s"] is None
               for r in records if r["method"] == "dls")
    target_file = output / "smoke_matched_seed.json"
    target_file.write_text(json.dumps(_json_safe({"seed": SEEDS[0], "records": records}), indent=2, allow_nan=False))
    print(json.dumps({"smoke_seed": SEEDS[0], "identical_episode_specs": True,
                      "methods": [{"method": r["method"], "episode_status": r["episode_status"],
                                   "success": r["success"], "failure_reason": r["failure_reason"],
                                   "start": r["initial_cube_xyz"], "yaw_rad": r["initial_cube_yaw_rad"],
                                   "place": r["place_target_xyz"], "phases_executed": r["executed_phase_count"],
                                   "unexpected_contacts": r["metrics"]["unexpected_actual_contact_observations"],
                                   "dls_velocity_metric": (r["metrics"]["max_velocity_limit_violation_rad_s"] if r["method"] == "dls" else "not_applicable")}
                                  for r in records], "log": str(target_file)}, indent=2))


def _ci_mean(values, seed=143):
    values = np.asarray(values, dtype=float)
    if len(values) == 0:
        return {"n": 0, "mean": None, "median": None, "std": None, "ci95_mean": None}
    rng = np.random.default_rng(seed)
    samples = rng.choice(values, size=(5000, len(values)), replace=True).mean(axis=1)
    return {"n": int(len(values)), "mean": float(values.mean()), "median": float(np.median(values)),
            "std": float(values.std(ddof=1)) if len(values) > 1 else 0.0,
            "ci95_mean": [float(np.quantile(samples, .025)), float(np.quantile(samples, .975))]}


def _wilson(successes, n):
    if n == 0:
        return None
    z = 1.959963984540054
    p = successes / n
    den = 1 + z*z/n
    center = (p + z*z/(2*n))/den
    half = z*np.sqrt(p*(1-p)/n + z*z/(4*n*n))/den
    return [float(max(0, center-half)), float(min(1, center+half))]


def _summary(records, output):
    by_method = {m: [r for r in records if r["method"] == m] for m in METHODS}
    summary = {"metric_definitions": {
        "ik_solve_time_total_s": "sum of wall-clock time around each iterative IK call in one episode; excludes controller simulation time",
        "episode_wall_time_s": "full episode wall-clock duration including IK, simulation, and passive collision measurements",
        "ik_iterations_total": "sum of method-specific outer IK updates/calls; comparable only as outer-call counts, not equal computational work",
        "backend_iterations": "unavailable (null); Mink/DAQP APIs used here do not expose backend iteration counts",
        "minimum_unexpected_clearance_m": "minimum signed mj_geomDistance over monitored pairs not excluded as expected contact for their phase; negative indicates overlap",
        "contact_observations": "per-physics-step contact records; repeated observations are not unique events",
        "distinct_contact_events": "transitions into dist<=0 per geom pair after a step without that pair in actual contact",
        "dls_velocity_limit": "not enforced; velocity-limit violation is unavailable (null), not zero",
    }, "seed_list": list(SEEDS), "episode_count_per_method": len(SEEDS), "velocity_limit_rad_s_mink_qp": VELOCITY_LIMIT_RAD_S,
        "methods": {}}
    for method, rows in by_method.items():
        n = len(rows)
        success_count = sum(bool(r["success"]) for r in rows)
        metrics = [r["metrics"] for r in rows]
        def vals(key):
            return [m[key] for m in metrics if m.get(key) is not None]
        failures = {}
        for r in rows:
            if not r["success"]:
                category = r["failure_reason"] or "unspecified"
                failures[category] = failures.get(category, 0) + 1
        summary["methods"][method] = {
            "episodes": n, "successes": success_count, "success_rate": success_count/n if n else None,
            "success_rate_wilson_95_ci": _wilson(success_count, n), "failures": failures,
            "aborted": sum(r["episode_status"] == "aborted" for r in rows),
            "ik_solve_time_total_s": _ci_mean(vals("ik_solve_time_total_s"), 144),
            "episode_wall_time_s": _ci_mean(vals("episode_wall_time_s"), 145),
            "ik_iterations_total_outer_updates": _ci_mean(vals("ik_iterations_total"), 146),
            "minimum_unexpected_clearance_m": _ci_mean(vals("minimum_unexpected_clearance_m"), 147),
            "max_joint_position_limit_violation_rad": _ci_mean(vals("max_joint_position_limit_violation_rad"), 148),
            "max_velocity_limit_violation_rad_s": _ci_mean(vals("max_velocity_limit_violation_rad_s"), 149),
            "velocity_limit_enforcement": "not_enforced" if method == "dls" else "hard_constraint",
            "unexpected_contact_episodes": sum((m.get("unexpected_actual_contact_observations") or 0) > 0 for m in metrics),
            "unexpected_penetration_episodes": sum((m.get("unexpected_penetration_observations") or 0) > 0 for m in metrics),
        }
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    _plots(by_method, out)
    print(json.dumps({"summary": str(out / "summary.json"), "plots": sorted(p.name for p in out.glob("*.png")),
                      "methods": {m: {"episodes": v["episodes"], "successes": v["successes"],
                                      "success_rate": v["success_rate"], "failures": v["failures"],
                                      "mean_ik_solve_time_s": v["ik_solve_time_total_s"]["mean"],
                                      "mean_episode_wall_time_s": v["episode_wall_time_s"]["mean"],
                                      "mean_iterations": v["ik_iterations_total_outer_updates"]["mean"],
                                      "mean_minimum_unexpected_clearance_m": v["minimum_unexpected_clearance_m"]["mean"]}
                                     for m, v in summary["methods"].items()}}, indent=2))


def _plots(by_method, out):
    import os
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/lab3_mpl_config")
    Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"dls": "#2874a6", "mink": "#d35400", "qp": "#239b56"}
    names = list(METHODS)
    # Success fraction with Wilson intervals.
    fig, ax = plt.subplots(figsize=(7, 4.5), constrained_layout=True)
    rates = [sum(r["success"] for r in by_method[m])/len(by_method[m]) for m in names]
    cis = [_wilson(sum(r["success"] for r in by_method[m]), len(by_method[m])) for m in names]
    errors = [[max(0.0, rates[i]-cis[i][0]) for i in range(3)],
              [max(0.0, cis[i][1]-rates[i]) for i in range(3)]]
    ax.bar(names, rates, color=[colors[m] for m in names], yerr=errors, capsize=4)
    ax.set_ylim(0, 1.05); ax.set_ylabel("Success rate (fraction)"); ax.set_title("Pick-and-place success (95% Wilson CI)")
    for i, m in enumerate(names): ax.text(i, rates[i]+.025, f"{sum(r['success'] for r in by_method[m])}/{len(by_method[m])}", ha="center")
    fig.savefig(out / "success_rate.png", dpi=180); plt.close(fig)

    for field, title, filename in [
        ("ik_solve_time_total_s", "Total IK solver time per episode", "ik_solve_time.png"),
        ("episode_wall_time_s", "Full episode wall time", "episode_wall_time.png"),
        ("ik_iterations_total", "Outer IK updates per episode (method-specific)", "ik_iterations.png"),
        ("minimum_unexpected_clearance_m", "Minimum unexpected-pair clearance", "minimum_clearance.png"),
        ("max_joint_position_limit_violation_rad", "Maximum joint-position limit excursion", "joint_limit_violations.png"),
        ("max_velocity_limit_violation_rad_s", "Maximum velocity-limit excess (DLS unavailable)", "velocity_limit_violations.png"),
    ]:
        fig, ax = plt.subplots(figsize=(7, 4.5), constrained_layout=True)
        data = [[r["metrics"].get(field) for r in by_method[m] if r["metrics"].get(field) is not None] for m in names]
        data = [d for d in data if d]
        labels = [m for m, d in zip(names, [[r["metrics"].get(field) for r in by_method[m] if r["metrics"].get(field) is not None] for m in names]) if d]
        if data:
            ax.boxplot(data, tick_labels=labels, showmeans=True)
        else:
            ax.text(.5, .5, "No measured values", ha="center", va="center", transform=ax.transAxes)
        unit = "(s)" if "time" in field else ("(rad)" if "rad" in field and "clearance" not in field else ("(m)" if "clearance" in field else "(updates)"))
        ax.set_ylabel(f"{field.replace('_', ' ')} {unit}"); ax.set_title(f"{title} (sample counts shown per method)")
        for i, label in enumerate(labels, 1): ax.text(i, 0.99, f"n={len(data[i-1])}", transform=ax.get_xaxis_transform(), ha="center", va="top", fontsize=8)
        fig.savefig(out / filename, dpi=180); plt.close(fig)

    failures = {}
    for m in names:
        for r in by_method[m]:
            if not r["success"]:
                failures.setdefault(r["failure_reason"] or "unspecified", {method: 0 for method in names})[m] += 1
    fig, ax = plt.subplots(figsize=(8, 4.8), constrained_layout=True)
    if failures:
        cats = list(failures)
        x = np.arange(len(cats)); width = .24
        for j, m in enumerate(names): ax.bar(x+(j-1)*width, [failures[c][m] for c in cats], width, color=colors[m], label=m)
        ax.set_xticks(x, cats, rotation=20, ha="right"); ax.legend(title="Method")
    else:
        ax.text(.5, .5, "No failures observed", ha="center", va="center", transform=ax.transAxes)
    ax.set_ylabel("Failed episodes (count)"); ax.set_title("Observed failure categories")
    fig.savefig(out / "failure_categories.png", dpi=180); plt.close(fig)


def _load_jsonl(path):
    with Path(path).open() as f:
        return [json.loads(line) for line in f if line.strip()]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("smoke", "batch", "summarize"), default="smoke")
    parser.add_argument("--output", default="results_stage3")
    parser.add_argument("--episodes", type=int, default=25)
    args = parser.parse_args()
    out = Path(args.output)
    if args.mode == "smoke":
        _smoke(out / "smoke")
    elif args.mode == "batch":
        if args.episodes < 25:
            parser.error("the required batch must contain at least 25 episodes per method")
        seeds = SEEDS[:args.episodes] if args.episodes <= len(SEEDS) else tuple(range(SEEDS[0], SEEDS[0]+args.episodes))
        out.mkdir(parents=True, exist_ok=True)
        model, data, *_ = load_model()
        # Preflight the exact fixed seed conditions before writing batch rows.
        preflight = [_one(m, seeds[0], model, data) for m in METHODS]
        if not all(p["episode_spec"] == preflight[0]["episode_spec"] for p in preflight):
            raise RuntimeError("seed preflight produced nonidentical initial/target conditions")
        (out / "preflight_seed.json").write_text(json.dumps(preflight, indent=2))
        manifest = {"seed_list": list(seeds), "methods": METHODS, "episodes_per_method": args.episodes,
                    "velocity_limit_rad_s_mink_qp": VELOCITY_LIMIT_RAD_S,
                    "randomization": "per-seed NumPy default_rng; cube x/y ±0.01 m around nominal pick, yaw ±0.10 rad; place x/y ±0.01 m around nominal target",
                    "episode_reset": "mj_resetData before every method/seed episode; fresh collision monitor per episode",
                    "condition_matching": "identical episode_spec for same seed across all methods",
                    "run_order": "seed-major then method order dls, mink, qp",
                    "success_criteria": "cube lifted >0.05 m; final x/y within 0.07 m of placement center; final z in (0.13,0.22) m; no unexpected actual contact",
                    "collision_avoidance_enforced": False, "tray_geometry_present": False}
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
        raw_path = out / "episodes.jsonl"
        records = []
        with raw_path.open("w") as f:
            for seed in seeds:
                spec = make_episode_spec(seed)
                for method in METHODS:
                    try:
                        record = _one(method, seed, model, data)
                        if record["episode_spec"] != spec:
                            raise RuntimeError("condition mismatch after run")
                    except Exception as exc:
                        record = {"method": method, "seed": int(seed), "episode_status": "aborted",
                                  "success": False, "failure_reason": f"harness_exception:{type(exc).__name__}:{exc}",
                                  "episode_spec": spec, "ik_solve_time_total_s": None,
                                  "episode_wall_time_s": None, "ik_iterations_total": None,
                                  "collision_audit": None, "metrics": {}}
                    record["batch_record_id"] = f"{method}-{seed}"
                    record = _json_safe(record)
                    f.write(json.dumps(record, allow_nan=False) + "\n")
                    f.flush()
                    records.append(record)
                    print(f"{record['batch_record_id']} {record['episode_status']} success={record['success']} reason={record['failure_reason']}", flush=True)
        _summary(records, out)
    else:
        raw_path = out / "episodes.jsonl"
        _summary(_load_jsonl(raw_path), out)


if __name__ == "__main__":
    main()
