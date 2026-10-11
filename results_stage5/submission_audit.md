# Stage 5 — Submission evidence audit

Audit date: 2026-10-11  
Scope: saved project evidence from Stages 1–4, plus targeted repeatability checks. No project controller, model, report, README, submission checklist, or existing result was changed. The Stage 3 75-episode batch was not rerun.

## Executive assessment

The repository has a traceable matched comparison of DLS, Mink, and QP over 25 shared seeds each; sampled workspace data and plots; passive collision-audit records; and a small Stage 4 opt-in controller that prechecks sampled segments and aborts when it detects an unsafe segment. A fresh run reproduced both Stage 4 cases, and the original DLS headless nominal run reproduced the saved final state exactly.

Submission readiness is incomplete. The required three-minute video is absent. The primary `REPORT.md`, `README.md`, and `SUBMISSION.md` still describe the earlier baseline and do not integrate the stronger Stage 1–4 evidence. Collision avoidance must be described narrowly: the Stage 4 controller checks sampled planned segments and monitors clearance at runtime, but it does not search for detours, constrain IK, certify continuous paths, or model a tray. Workspace reachability is endpoint IK evidence, not path feasibility.

## Evidence verified

### 1. Original demonstration and Stage 1 solver checks

- The legacy nominal record is in `results/episodes.json` (one episode). The DLS regression command produced a successful lift and placement, eight waypoint records, and final cube position `[0.3521443313, 0.1779457626, 0.1599447136] m` after `15.2000000000 s` of simulation time. That matches the saved legacy nominal values.
- `results_ik_smoke/smoke_and_episodes.json` contains a same-target IK smoke for each method and one pick-and-place episode for each of DLS, Mink, and QP-DAQP. All six records report success/convergence. In the smoke, position residuals were about `0.00338 m` and orientation residuals about `0.01906 rad` for Mink/QP and `0.00307 m` / `0.02138 rad` for DLS. The one-episode runs are useful integration checks, not a comparative performance estimate.
- The older scenario records exist: `results_randomized/episodes.json` (8 episodes), `results_demonstrations/episodes.json` (1), and `results_failures/episodes.json` (5). They are legacy single-controller scenarios and do not replace the Stage 3 three-method matched comparison.

### 2. Stage 3 matched comparison

Raw records are in `results_stage3/batch/episodes.jsonl`; the fixed seeds and task/reset protocol are in `results_stage3/batch/manifest.json`; preflight and smoke records are in `results_stage3/batch/preflight_seed.json` and `results_stage3/smoke/smoke_matched_seed.json`; the computed metrics and definitions are in `results_stage3/batch/summary.json`.

The JSONL has 75 records: exactly 25 per method, with seeds `20261011`–`20261035` for each. A record-by-record comparison confirms that initial cube position/yaw and pick/place targets match across methods for every seed. Each method executes the same eight phases. The manifest specifies cube x/y perturbations of ±`0.01 m`, yaw perturbation of ±`0.10 rad`, and placement x/y perturbations of ±`0.01 m`.

| Method | Success | Mean / median total IK-call time (s) | Mean / median full episode wall time (s) | Mean / median outer IK updates | Mean minimum unexpected clearance (m) |
|---|---:|---:|---:|---:|---:|
| DLS | 25/25 | 0.07035 / 0.06377 | 4.26768 / 4.27831 | 554.60 / 517 | 0.001834 / 0.001839 |
| Mink | 25/25 | 0.10223 / 0.09814 | 4.30942 / 4.32387 | 586.68 / 555 | 0.001774 / 0.001779 |
| QP | 25/25 | 0.10150 / 0.09371 | 4.31531 / 4.31735 | 592.44 / 561 | 0.001788 / 0.001780 |

All three have zero measured joint-position-limit violation, zero unexpected actual-contact observations, and zero unexpected penetration observations in these episodes. Mink/QP velocity violation measurements are numerical noise on the order of `10^-14 rad/s`, below meaningful precision against the configured `1.2 rad/s` limit; report as no measured violation (or round to zero with the method and limit stated). DLS velocity limit is not enforced and is unavailable (`null`), not zero. Backend iteration counts are unavailable (`null`). The summary has 95% intervals for means and Wilson intervals for success; with 25/25 successes, the Wilson interval is about `[0.867, 1.000]` for each method.

Comparability cautions:

- “IK iterations” are method-specific outer update counts, not backend iteration counts or equal units of computational work. The methods therefore should not be ranked by this count as if one update had equal cost.
- IK time sums wall-clock time spent inside the iterative IK calls. Episode wall time includes the entire run and collision-measurement work. These are separate metrics.
- There are 200 waypoint solver calls per method. Across them, `max_iterations` occurred 56 times for DLS and 55 times each for Mink and QP; these waypoints still executed under the episode’s comparatively loose waypoint thresholds (`0.06 m`, `0.25 rad`). The 25/25 task success result does not mean every waypoint solver call converged.
- The success definition allows final horizontal placement error up to `0.07 m` and final cube z in `[0.13, 0.22] m`. This is a permissive task-level placement criterion. State it beside the success rate; do not imply millimetre placement accuracy.
- The clearance values are minima over audited unexpected pairs for each episode; the batch has passive collision measurement only (`collision_avoidance_enforced=false` in the manifest). It does not measure collision-constrained trajectory planning.

Plots generated from the raw batch are in `results_stage3/batch/`: `success_rate.png`, `ik_solve_time.png`, `episode_wall_time.png`, `ik_iterations.png`, `minimum_clearance.png`, `joint_limit_violations.png`, `velocity_limit_violations.png`, and `failure_categories.png`. Visual inspection confirms labeled axes and method/sample counts. The success plot’s 95% Wilson intervals are valid but wide and identical because all methods have 25/25 successes. The time plots show overlapping/small differences and should not support a broad claim that one solver is “best.”

### 3. Workspace and table evidence

`results_stage2/workspace_samples.csv` contains 1,992 sampled records. `results_stage2/workspace_summary.json` documents the model joint limits, residual thresholds, deterministic grid, yaw samples, table values, and sampling limits. The method is endpoint DLS IK using the actual six MuJoCo joint ranges, with position threshold `0.025 m`, orientation threshold `0.05 rad`, maximum 180 iterations, and damping `0.08`. The grid samples x `[0.08, 0.64] m`, y `[-0.27, 0.37] m`, and TCP z `[0.295, 0.415, 0.475] m`; the edge-buffered table-center grid spans x `[0.13, 0.59] m`, y `[-0.22, 0.32] m`. The recorded seed is 7, but the sampler is deterministic and consumes no random samples.

- Task workspace: 248/249 sampled poses pass. At TCP z `0.295 m`, all 81 regular 9×9 edge-buffered table-center grid points pass, including the nominal pick and placement points.
- Dexterous workspace: 1,735/1,743 sampled poses pass across seven yaw samples `−90°` through `+90°`. At contact height, the summary classifies 82/83 sampled positions as feasible at all seven yaws.
- Plots: `results_stage2/workspace_task_xy_contact.png`, `results_stage2/workspace_dexterous_xy_contact.png`, and `results_stage2/workspace_task_3d.png`. The plot labels and axes were visually checked. The task 3D figure marks one infeasible sample; unsampled positions remain unknown.
- The dexterous XY plot shows the regular 81-position grid (80 feasible and one infeasible), while the summary’s 82/83 includes two exact task target samples in addition to the regular grid. This is explainable from the sampling records but should be stated in captions/report text so the numerator/denominator do not look contradictory.

The current table in `robot_descriptions/arm_gripper.xml` has center `[0.36, 0.05, 0.12] m`, full dimensions `[0.56, 0.64, 0.04] m` (x, y, z), and top surface `z=0.14 m`. `workspace_summary.json` recommends retaining the existing footprint because both targets and all sampled 50-mm edge-buffered center points are reachable at the task TCP height. The plot and data support only that sampled reachability statement. They do not establish continuous coverage, collision-free access, mechanical stability, load capacity, or a structural table design. No table dimensions were changed.

### 4. Passive collision audit versus Stage 4 active checks

Stage 2’s `results_stage2/nominal_collision_smoke.json` records 7,600 simulation steps, 494,000 distance queries, 24 active collision geoms, and 65 monitored pair sets. It observes 77,565 expected contact observations and zero unexpected actual-contact observations. The smallest unexpected measured clearance is `0.001517 m` (cube to follower gripper geometry). The deepest contact penetration, `−0.002010 m`, belongs to contacts classified as expected. The record explicitly says collision avoidance was not enforced. Repeated per-step contact observations are not distinct contact events.

Stage 4 evidence is in `results_stage4/collision_aware_smoke.json` and `results_stage4/segment_clearance.png`; implementation is in `scripts/collision_aware_pick_place.py`, `utils/collision_aware.py`, and `utils/collision_audit.py`.

- The nominal case completes all eight phases. Final cube position is `[0.331916, 0.181406, 0.159945] m` against target `[0.35, 0.18, 0.165] m`; the x error is about `18.1 mm`, within the broad task acceptance. It has 7,600 monitored steps, 494,000 distance queries, 77,847 expected contact observations across 38 distinct expected contact bouts, and no unexpected actual contacts or penetration observations. The deepest expected penetration is about `−2.468 mm`.
- The minimum planned-segment unexpected clearance is `0.7926 mm` during descent, just `0.0426 mm` above the configured `0.75 mm` threshold. This is a narrow simulation margin and is sensitive to model/mesh and sampling assumptions; it is not a hardware safety margin.
- The deliberately low pick target is rejected during descent with `unsafe_segment:descend:right_follower_tongue_collision<->worktable`. The rejected segment’s minimum signed clearance is `−31.598 mm`; its first threshold crossing occurs at `−0.0512 mm`. The approach completed, then the unsafe descent was refused. There were no unexpected actual contacts on the executed portion. The saved JSON and fresh rerun agree on this outcome and the relevant clearance values.
- The plot’s red descent bar is based on the six executed Cartesian refinement subsegments before the final unsafe check; it is not the rejected full segment’s `−31.598 mm` minimum. The figure should be captioned to say it shows executed/refined checks and omits the rejected candidate, or be regenerated in a future implementation to display rejected candidates distinctly. Do not quote the red bar as the minimum clearance of the rejected descent.
- The model has no tray geometry (`tray.present=false`). The target is a coordinate on the worktable. No tray collision avoidance can be claimed.
- Stage 4 is active sampled-segment rejection plus a per-step runtime clearance/contact abort guard, not a general collision-free planner. It samples 25 points along joint interpolation, uses an approximate rigid TCP-relative cube for some held-object kinematic checks, excludes cube/gripper envelope pairs from selected checks, and has no detour search or collision-constrained IK. Sampling can miss collisions between samples.

### 5. Existing report and submission artifacts

- `REPORT.md` and `README.md` remain focused on the earlier baseline. The report’s 95/125 workspace figure is traceable to the legacy `results_workspace/workspace.csv` (125 rows) and is not the Stage 2 workspace result. Its 8/8 randomized result is traceable to `results_randomized/episodes.json`. These are earlier experiments; the final report should clearly identify them as legacy or replace them with the Stage 2/3 evidence where appropriate.
- The current report does not present the matched three-method Stage 3 comparison or Stage 4 rejection result. It also has no integrated comparison plots/metrics table.
- `README.md` documents baseline commands but not how to run the three solver modes, the Stage 3 summarizer, workspace/collision audit, or Stage 4 collision-aware smoke.
- `SUBMISSION.md` lists legacy output locations and omits `results_ik_smoke/` and `results_stage2/`–`results_stage4/` evidence.
- No video file with extension `.mp4`, `.mkv`, `.webm`, `.mov`, or `.avi` exists in the repository. A three-minute video has not been evidenced and must remain marked missing until recorded and verified.
- No assignment PDF/handout or standalone rubric file was found in the repository search. Any final rubric mapping should use the assignment requirements provided by the user unless the official handout is supplied.

## Prioritized gaps by grading impact

1. **Critical — required video missing.** Record and verify a roughly 180-second video with about 60 seconds per method using the actual MuJoCo viewer and HEAL robot.
2. **High — report/submission documentation is stale.** Integrate the measured three-method comparison, workspace/table qualification, Stage 4 narrow collision-avoidance claim and limitation, failure analysis, units, and plots into a maximum-five-page report; update README and submission checklist only after approval.
3. **High — active collision evidence is limited.** Stage 4 has only one nominal success and one deliberately rejected target. Its 0.793-mm nominal clearance is close to the configured threshold, segment sampling is finite, and no tray is modeled. Present it as a verified limited check/rejection behavior, not general collision avoidance.
4. **Medium — explain task success tolerance and solver status counts.** The Stage 3 `0.07 m` final placement tolerance is broad, and many individual waypoint calls reached `max_iterations` but were accepted by the looser execution threshold. Report both facts explicitly.
5. **Medium — clarify plot/sample denominators.** State why the dexterous summary is 82/83 while the XY grid plot shows 80/81. Clarify that the Stage 4 red descent bar excludes the ultimately rejected candidate segment.
6. **Medium — table recommendation scope.** Keep the 0.56×0.64×0.04 m table recommendation as a sampled task reachability result only; there is no structural/load or continuous path analysis.
7. **Low — tidy reproducibility narrative.** Label the older 95/125 workspace and 8/8 randomized records as baseline-era results instead of mixing them with the newer Stage 2/3 evidence.

## Numerical and wording corrections for the final submission

- Use `248/249` task samples and `1,735/1,743` dexterous pose samples, and give the thresholds and sampled grid. Do not call unsampled cells reachable.
- For dexterous contact-height positions, either report `82/83` from the summary and explain the two exact target points, or report the plotted regular grid as `80/81`. Do not present these as the same denominator.
- Use the exact Stage 3 aggregate table above and distinguish solver-call seconds from full episode wall seconds. Backend iterations are unavailable. DLS velocity-limit violations are unavailable because no time-based cap is enforced.
- Describe the Stage 3 success rate as `25/25 per method` with its interval and the 70-mm horizontal success tolerance; do not imply tight placement accuracy or a solver winner.
- Say Stage 2 is a **passive audit**. Say Stage 4 **rejects sampled unsafe segments and monitors runtime clearance**. Do not say “collision-free trajectory,” “tray avoidance,” or “collision-constrained IK.”
- For Stage 4, state nominal minimum sampled clearance `0.7926 mm` and threshold `0.75 mm`; state the low-target candidate was rejected at descent. Distinguish the full rejected segment’s minimum (`−31.598 mm`) from the first threshold-crossing sample (`−0.0512 mm`) and from the plot’s executed refinement bar.
- Keep table coordinates and full extents explicit: center `[0.36, 0.05, 0.12] m`; dimensions `[0.56, 0.64, 0.04] m`; top `z=0.14 m`.

## Required screenshots and video shots

There are no verified video/screenshots in the repository. For the assignment video, capture the actual MuJoCo viewer and HEAL + Robotiq model with a visible method label and comparable cube/target conditions:

1. Opening title card or short slate: project, model, and three methods.
2. Mink segment (~60 s): initial scene, approach, gripper close/contact, cube lift, transfer, release at target, retreat/final state.
3. DLS segment (~60 s): same phases and framing, labeled DLS.
4. QP segment (~60 s): same phases and framing, labeled QP.
5. Brief ending view showing all methods were run (or a final result card). Verify final duration near 180 s and that the file plays with image and audio/video stream metadata; do not substitute plots or generated graphics for the viewer footage.

Useful report screenshots/figures: task workspace XY plot with pick/place and table boundary; dexterous workspace XY plot with yaw range; one clean nominal HEAL grasp/transfer/place viewer frame; Stage 3 success/time/clearance comparison plots or compact combined figure; Stage 4 segment-clearance plot only with the rejected-candidate explanation above.

## Proposed five-page report outline

1. **Introduction and goal** (about 0.4 page): task, HEAL and gripper model, comparison question, scope of simulation evidence.
2. **Methods** (about 1.2 pages): table/cube/target coordinates; matched seeds/reset; shared eight phases and torque controller; DLS, Mink, QP constraints and velocity limit; metric definitions; workspace sampling; passive collision audit versus Stage 4 sampled rejection/runtime guard.
3. **Results** (about 1.4 pages): matched 25-seed table with confidence intervals/sample counts; solver time versus wall time; limits/clearances; task and dexterous workspace figure; table footprint justification; nominal and obstructed Stage 4 outcomes.
4. **Failure analysis and limitations** (about 1 page): waypoint `max_iterations`, broad task acceptance, unsafe descent rejection, narrow clearance headroom, no tray, finite path samples, no hardware claim, and unavailable DLS velocity/backend-iteration measurements.
5. **Discussion and conclusion** (about 0.5 page): cautious comparison interpretation, what evidence supports, what remains to complete (especially video), and next steps.

Reserve remaining space for references/captions, and include only figures that remain legible when rendered to the five-page limit.

## Reproduction commands

Run from the project root. These are the saved artifact commands; `batch` reruns the full 75 episodes and was **not** run during this audit.

```bash
# Original DLS nominal regression
./.venv/bin/python scripts/pick_and_place.py --headless --output results

# Stage 1: one target smoke and one matched task episode per solver
./.venv/bin/python scripts/ik_solver_smoke.py --output results_ik_smoke

# Stage 2: regenerate workspace samples, plots, and passive nominal collision audit
./.venv/bin/python scripts/workspace_collision_audit.py --mode all --output results_stage2

# Stage 3: one-seed-per-method harness smoke
./.venv/bin/python scripts/batch_evaluate.py --mode smoke --output results_stage3/smoke

# Stage 3: full 25-seed-per-method batch (75 episodes total; not rerun for this audit)
./.venv/bin/python scripts/batch_evaluate.py --mode batch --episodes 25 --output results_stage3/batch

# Regenerate Stage 3 summary and plots from saved JSONL without rerunning episodes
./.venv/bin/python scripts/batch_evaluate.py --mode summarize --output results_stage3/batch

# Stage 4: nominal and deliberately low-pick challenge, 0.75-mm threshold, 25 path samples
./.venv/bin/python scripts/collision_aware_pick_place.py --output results_stage4 --clearance-margin-mm 0.75 --path-samples 25
```

Audit verification actually performed: inspected the saved Stage 1–4 JSON/JSONL/CSV and plots; checked matched seeds and per-seed task equality; checked Stage 3 raw counts against `summary.json`; visually inspected workspace, comparison, and Stage 4 clearance plots; ran the original DLS headless regression to `/tmp/lab3_stage5_baseline`; reran Stage 4 to `/tmp/lab3_stage5_repro` and compared nominal and obstructed status, final state, failure reason, and segment clearance. Both Stage 4 cases matched the saved records. No full batch, video recording, report edit, commit, or push was performed.
