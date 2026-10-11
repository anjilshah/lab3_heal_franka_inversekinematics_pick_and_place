#!/usr/bin/env python3
"""Validate, summarize, and display the saved Lab 3 evaluation results.

Run from any working directory, for example:
    ./.venv/bin/python scripts/view_lab3_results.py
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "lab3_mpl_config"))
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
METHODS = ("dls", "mink", "qp")
PLOT_PATHS = (
    "results_video/ik_comparison_1080p.png",
    "results_stage2/workspace_task_xy_contact.png",
    "results_stage2/workspace_dexterous_xy_contact.png",
    "results_stage2/workspace_task_3d.png",
    "results_stage3/batch/ik_solve_time.png",
    "results_stage3/batch/episode_wall_time.png",
    "results_stage3/batch/minimum_clearance.png",
    "results_stage4/segment_clearance.png",
)


def load_jsonl(path: Path) -> list[dict]:
    records = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSON in {path}, line {line_number}: {exc}") from exc
            if not isinstance(record, dict):
                raise ValueError(f"Expected a JSON object in {path}, line {line_number}")
            records.append(record)
    return records


def _number(record: dict, key: str, *, allow_none: bool = False):
    value = record.get("metrics", {}).get(key)
    if value is None and allow_none:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"Record {record.get('batch_record_id', '<unknown>')} has invalid {key}: {value!r}")
    return float(value)


def _check_close(label: str, computed, stored, *, method: str):
    if computed is None or stored is None:
        if computed is not stored:
            raise ValueError(f"Summary mismatch for {method} {label}: computed {computed}, saved {stored}")
    elif not math.isclose(float(computed), float(stored), rel_tol=1e-8, abs_tol=1e-12):
        raise ValueError(f"Summary mismatch for {method} {label}: computed {computed}, saved {stored}")


def validate_and_summarize(summary: dict, records: list[dict]) -> dict:
    """Derive displayed metrics from raw episodes and check them against summary.json."""
    if not isinstance(summary, dict) or not isinstance(summary.get("methods"), dict):
        raise ValueError("summary.json must contain a 'methods' object")
    by_method = {method: [] for method in METHODS}
    for record in records:
        method = record.get("method")
        if method not in by_method:
            raise ValueError(f"Unexpected method in episode log: {method!r}")
        if not isinstance(record.get("success"), bool):
            raise ValueError(f"Episode record for {method} is missing a boolean 'success'")
        by_method[method].append(record)

    expected_seeds = summary.get("seed_list")
    computed = {}
    records_by_seed = {}
    for method_rows in by_method.values():
        for row in method_rows:
            records_by_seed.setdefault(row.get("seed"), []).append(row)
    for seed, seed_rows in records_by_seed.items():
        specs = [row.get("episode_spec") for row in seed_rows]
        if len(seed_rows) != len(METHODS) or any(spec != specs[0] for spec in specs[1:]):
            raise ValueError(f"Episode conditions are not matched across methods for seed {seed}")

    for method in METHODS:
        rows = by_method[method]
        saved = summary["methods"].get(method)
        if not rows:
            raise ValueError(f"No raw episode records found for {method}")
        if not isinstance(saved, dict):
            raise ValueError(f"summary.json is missing method '{method}'")
        successes = sum(row["success"] for row in rows)
        rate = successes / len(rows)
        ik = np.asarray([_number(row, "ik_solve_time_total_s") for row in rows])
        wall = np.asarray([_number(row, "episode_wall_time_s") for row in rows])
        clearance_values = [
            _number(row, "minimum_unexpected_clearance_m", allow_none=True)
            for row in rows
        ]
        clearance = np.asarray([value for value in clearance_values if value is not None])

        if expected_seeds is not None:
            seeds = [row.get("seed") for row in rows]
            if sorted(seeds) != sorted(expected_seeds):
                raise ValueError(f"Raw seed list for {method} does not match summary.json")
        for record_key, stored_key, value in (
            ("episodes", "episodes", len(rows)),
            ("successes", "successes", successes),
            ("success_rate", "success_rate", rate),
        ):
            _check_close(record_key, value, saved.get(stored_key), method=method)
        for name, arr in (("ik_solve_time_total_s", ik),
                          ("episode_wall_time_s", wall),
                          ("minimum_unexpected_clearance_m", clearance)):
            stored_stats = saved.get(name)
            if not isinstance(stored_stats, dict):
                raise ValueError(f"summary.json is missing {method}.{name}")
            _check_close(f"{name}.n", len(arr), stored_stats.get("n"), method=method)
            _check_close(f"{name}.mean", float(arr.mean()) if len(arr) else None,
                         stored_stats.get("mean"), method=method)
            median = float(np.median(arr)) if len(arr) else None
            _check_close(f"{name}.median", median, stored_stats.get("median"), method=method)

        computed[method] = {
            "episodes": len(rows), "successes": successes, "success_rate": rate,
            "ik_mean_s": float(ik.mean()), "ik_median_s": float(np.median(ik)),
            "episode_wall_mean_s": float(wall.mean()),
            "minimum_clearance_mean_m": float(clearance.mean()) if len(clearance) else None,
            "clearance_n": len(clearance),
        }

    counts = [computed[method]["episodes"] for method in METHODS]
    if len(set(counts)) != 1:
        raise ValueError(f"Methods do not have matched sample counts: {counts}")
    return computed


def make_table(stats: dict) -> str:
    headers = ("Method", "Success", "Rate", "IK mean (ms)", "IK median (ms)",
               "Episode mean (s)", "Mean min clearance (mm)")
    rows = []
    for method in METHODS:
        values = stats[method]
        clearance = values["minimum_clearance_mean_m"]
        rows.append((
            method.upper(),
            f"{values['successes']}/{values['episodes']}",
            f"{100 * values['success_rate']:.1f}%",
            f"{1000 * values['ik_mean_s']:.2f}",
            f"{1000 * values['ik_median_s']:.2f}",
            f"{values['episode_wall_mean_s']:.3f}",
            f"{1000 * clearance:.3f}" if clearance is not None else "n/a",
        ))
    widths = [max(len(headers[i]), *(len(row[i]) for row in rows)) for i in range(len(headers))]
    fmt = "  ".join(f"{{:<{width}}}" for width in widths)
    lines = [fmt.format(*headers), fmt.format(*("-" * width for width in widths))]
    lines.extend(fmt.format(*row) for row in rows)
    return "\n".join(lines)


def show_plots(root: Path) -> None:
    backend = plt.get_backend().lower()
    if backend in {"agg", "pdf", "ps", "svg", "template"}:
        print(f"Warning: Matplotlib backend '{plt.get_backend()}' is non-interactive; "
              "plot windows cannot be opened in this session. Use a GUI Matplotlib backend "
              "(such as TkAgg or QtAgg) on a desktop, or pass --no-plots.", file=sys.stderr)
        return
    for relative_path in PLOT_PATHS:
        path = root / relative_path
        if not path.is_file():
            print(f"Warning: plot not found; skipping: {relative_path}", file=sys.stderr)
            continue
        try:
            image = plt.imread(path)
        except (OSError, ValueError) as exc:
            print(f"Warning: could not read plot; skipping {relative_path}: {exc}", file=sys.stderr)
            continue
        fig, ax = plt.subplots(figsize=(12, 8), constrained_layout=True)
        ax.imshow(image)
        ax.set_title(relative_path, fontsize=14, pad=12)
        ax.axis("off")
        try:
            plt.show(block=True)
        finally:
            plt.close(fig)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate the saved Stage 3 batch, print a comparison table, and display Lab 3 plots.",
        epilog=("Example: python scripts/view_lab3_results.py --save-summary results_stage3/comparison.txt\n"
                "Use --no-plots to print/save only the table."),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--summary", type=Path, default=ROOT / "results_stage3/batch/summary.json",
                        help="saved Stage 3 summary JSON (default: results_stage3/batch/summary.json)")
    parser.add_argument("--episodes", type=Path, default=ROOT / "results_stage3/batch/episodes.jsonl",
                        help="saved Stage 3 raw episode JSONL (default: results_stage3/batch/episodes.jsonl)")
    parser.add_argument("--no-plots", action="store_true",
                        help="print the table without opening plot windows")
    parser.add_argument("--save-summary", type=Path, metavar="PATH",
                        help="also save the printed comparison table as a UTF-8 text file")
    args = parser.parse_args(argv)

    try:
        summary = json.loads(args.summary.read_text(encoding="utf-8"))
        records = load_jsonl(args.episodes)
        stats = validate_and_summarize(summary, records)
    except (OSError, json.JSONDecodeError, ValueError, KeyError, TypeError, AttributeError) as exc:
        parser.error(f"could not validate saved results: {exc}")

    table = make_table(stats)
    print("Stage 3 matched batch comparison")
    print("IK times are solver-call wall time; episode time includes simulation and collision measurements.")
    print("Mean minimum clearance is signed geometric distance over unexpected monitored pairs; negative means overlap.")
    print(table)
    if args.save_summary is not None:
        args.save_summary.parent.mkdir(parents=True, exist_ok=True)
        args.save_summary.write_text(table + "\n", encoding="utf-8")
        print(f"Saved table: {args.save_summary}")
    if not args.no_plots:
        show_plots(ROOT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
