#!/usr/bin/env python3
"""Create a 1080p video card from the saved Stage 3 raw episode records."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tempfile

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "lab3_mpl_config"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


METHODS = ("dls", "mink", "qp")
COLORS = {"dls": "#54d6a0", "mink": "#62b6ff", "qp": "#ffbf69"}


def load_records(path: Path):
    rows = []
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        if line.strip():
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSONL at {path}:{line_no}: {exc}") from exc
    return rows


def summarize(records, reference):
    """Derive means from raw episodes and verify stored summary values."""
    result = {}
    seeds_by_method = {
        method: sorted(r["seed"] for r in records if r.get("method") == method)
        for method in METHODS
    }
    if any(seeds != seeds_by_method["dls"] for seeds in seeds_by_method.values()):
        raise ValueError("raw records do not contain the same seed list for all methods")
    if seeds_by_method["dls"] != sorted(reference.get("seed_list", [])):
        raise ValueError("raw seed list does not match the saved summary")
    by_seed = {}
    for row in records:
        by_seed.setdefault(row["seed"], []).append(row)
    for seed, rows in by_seed.items():
        specs = [r.get("episode_spec") for r in rows]
        if len(rows) != len(METHODS) or any(spec != specs[0] for spec in specs[1:]):
            raise ValueError(f"episode conditions are not matched for seed {seed}")
    for method in METHODS:
        rows = [r for r in records if r.get("method") == method]
        ref = reference["methods"][method]
        if len(rows) != 25 or ref["episodes"] != 25:
            raise ValueError(f"expected 25 raw and summarized episodes for {method}; got {len(rows)}")
        successes = sum(bool(r["success"]) for r in rows)
        if successes != ref["successes"]:
            raise ValueError(f"raw/summary success count mismatch for {method}")
        ik = np.asarray([r["metrics"]["ik_solve_time_total_s"] for r in rows], dtype=float)
        wall = np.asarray([r["metrics"]["episode_wall_time_s"] for r in rows], dtype=float)
        if not np.isclose(ik.mean(), ref["ik_solve_time_total_s"]["mean"], rtol=1e-10):
            raise ValueError(f"raw/summary IK mean mismatch for {method}")
        if not np.isclose(wall.mean(), ref["episode_wall_time_s"]["mean"], rtol=1e-10):
            raise ValueError(f"raw/summary episode wall mean mismatch for {method}")
        ci = ref["ik_solve_time_total_s"]["ci95_mean"]
        result[method] = {
            "n": len(rows), "successes": successes,
            "ik_mean_ms": float(ik.mean() * 1000),
            "ik_ci_ms": [float(ci[0] * 1000), float(ci[1] * 1000)],
            "wall_mean_s": float(wall.mean()),
        }
    return result


def render(stats, output: Path):
    fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor="#101820")
    ax = fig.add_axes([0, 0, 1, 1], facecolor="#101820")
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    ax.text(.055, .91, "THREE IK METHODS · MATCHED HEAL PICK-AND-PLACE",
            color="#f4f7fb", fontsize=25, weight="bold", va="center")
    ax.text(.055, .855, "MuJoCo simulation  |  25 identical seeds per method  |  75 episodes total",
            color="#a9bac9", fontsize=16, va="center")

    method_labels = {"dls": "CUSTOM DLS", "mink": "MINK DIFFERENTIAL IK", "qp": "QP · DAQP BACKEND"}
    y_positions = [0.70, 0.52, 0.34]
    for method, y in zip(METHODS, y_positions):
        row = stats[method]
        color = COLORS[method]
        ax.text(.06, y + .035, method_labels[method], color=color,
                fontsize=18, weight="bold", va="center")
        ax.text(.06, y - .025, f"{row['successes']}/{row['n']} successful",
                color="#f4f7fb", fontsize=16, va="center")
        x0, x1 = .36, .78
        scale = (x1 - x0) / 125.0
        ax.plot([x0, x1], [y, y], color="#314451", lw=14, solid_capstyle="round")
        lo, hi = row["ik_ci_ms"]
        ax.plot([x0 + lo * scale, x0 + hi * scale], [y, y], color=color, lw=6,
                solid_capstyle="round")
        ax.scatter([x0 + row["ik_mean_ms"] * scale], [y], s=180,
                   color=color, edgecolors="#101820", linewidths=2, zorder=3)
        ax.text(.82, y + .015, f"{row['ik_mean_ms']:.1f} ms",
                color="#f4f7fb", fontsize=21, weight="bold", va="center")
        ax.text(.82, y - .035, f"episode: {row['wall_mean_s']:.2f} s",
                color="#a9bac9", fontsize=13, va="center")

    ax.text(.36, .775, "MEAN TOTAL IK SOLVE TIME PER EPISODE · ms (95% bootstrap CI)",
            color="#d9e4ed", fontsize=13, weight="bold")
    for tick in (0, 25, 50, 75, 100, 125):
        x = .36 + .42 * tick / 125
        ax.plot([x, x], [.30, .735], color="#273843", lw=1, zorder=0)
        ax.text(x, .285, str(tick), color="#9aabba", fontsize=11, ha="center", va="top")

    ax.plot([.055, .945], [.22, .22], color="#314451", lw=1.5)
    ax.text(.06, .17, "IK time measures solver calls only. Episode time includes simulation and passive collision measurements.",
            color="#d9e4ed", fontsize=13, va="center")
    ax.text(.06, .125, "Observed in this 25-seed sample; not a guarantee. Error bars show 95% bootstrap intervals for mean IK time.",
            color="#a9bac9", fontsize=12, va="center")
    ax.text(.06, .055, "Source: results_stage3/batch/episodes.jsonl + summary.json  ·  matched seeds 20261011–20261035",
            color="#8093a2", fontsize=11, va="center")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=100, facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", default="results_stage3/batch/episodes.jsonl")
    parser.add_argument("--summary", default="results_stage3/batch/summary.json")
    parser.add_argument("--output", default="results_video/ik_comparison_1080p.png")
    args = parser.parse_args()
    records_path, summary_path, output = Path(args.episodes), Path(args.summary), Path(args.output)
    reference = json.loads(summary_path.read_text())
    stats = summarize(load_records(records_path), reference)
    render(stats, output)
    print(json.dumps({"output": str(output), "resolution_px": [1920, 1080],
                      "source_records": str(records_path), "source_summary": str(summary_path),
                      "methods": stats}, indent=2))


if __name__ == "__main__":
    main()
