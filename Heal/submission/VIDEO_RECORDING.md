# Three-minute Lab 3 video recording guide

The recording is still outstanding. This guide uses the live parallel viewer for the three robot trajectories and saved measured outputs for the results sections. The Stage 4 runner is headless, so show its real terminal summary and clearance plot rather than implying it produced viewer footage.

## Before recording

From the repository root, generate the comparison card from the saved Stage 3 JSONL and summary. The script checks the matched seed list, episode inputs, success counts, and raw-record means before drawing the card.

```sh
./.venv/bin/python Heal/scripts/create_video_results_card.py
```

Open OBS (`obs`) and set the canvas/output to 1920×1080. Capture the combined MuJoCo window and terminal. Use `--speed 0.2` for about 76 wall-clock seconds of simulated task motion, plus solver/render overhead. The three visible method headings remain above their own panels and the per-panel status updates during the task.

## Suggested 180-second edit

| Video time | Evidence to show |
|---|---|
| 0:00–0:05 | Title: HEAL + Robotiq pick-and-place; simulated study |
| 0:05–1:20 | Run the combined window: show DLS, Mink, and QP side by side through approach, grasp, lift, transfer, release, and retreat. |
| 1:20–1:45 | Show `results/video/ik_comparison_1080p.png`. State 25 matched seeds per method and that this finite sample is not a guarantee. IK time and full episode wall time are separate. |
| 1:45–2:15 | Run the Stage 4 command below while capturing the terminal output, then show `results/video/stage4/segment_clearance.png`. The run executes the nominal and deliberately unsafe low-pick cases in headless MuJoCo. Emphasize that the unsafe descent is rejected by sampled clearance checks. |
| 2:15–2:45 | Show `results/stage2/workspace_task_xy_contact.png` and `workspace_dexterous_xy_contact.png`; state that colored samples are tested endpoints, and unsampled regions are unknown. Mention the table is 0.56 × 0.64 × 0.04 m (full x/y/z extents). |
| 2:45–3:00 | Limitations: no tray geometry, no general/certified collision-free guarantee, simulation only. |

The live run takes about 76 seconds at `--speed 0.2`, leaving time for the measured-results and workspace/collision evidence. Keep the complete synchronized trajectory in the final video; trim only startup/closing delays. If a method fails, preserve the failure or re-run after diagnosing it rather than presenting it as a successful trajectory.

## Live method commands

Launch one combined viewer from a graphical desktop. The three panels run at the same pace and execute every simulation step; rendering combines live MuJoCo frames rather than replaying stored paths.

```sh
./.venv/bin/python Heal/scripts/parallel_ik_viewer.py --speed 0.2
```

Press **Start / Reset All** (or Space) to start/restart all three. Each panel begins from the same cube pose and target and uses the same eight phases. The online IK solvers and torque-tracking simulations run live. Do not use `--headless-check` for footage.

## Stage 4 evidence command

This command writes a fresh reproduction under `results/video/stage4/`, preserving the saved Stage 4 records:

```sh
./.venv/bin/python Heal/scripts/collision_aware_pick_place.py \
  --output Heal/results/video/stage4 --clearance-margin-mm 0.75 --path-samples 25
```

The nominal case measured a 0.793 mm minimum planned clearance against the 0.75 mm margin and 1.517 mm minimum unexpected-pair execution clearance. The low-pick case rejected an unsafe descent. This is finite sampled segment validation plus runtime abort checks, not general path planning; there is no tray geometry. The stage output is headless, so its terminal result and plot are evidence of tests, not a visual robot capture.

## Verification after recording

Save the actual recording as `submission/lab3_demo.mp4` (or another clearly named video file). Play it from beginning to end and verify it is about 180 seconds, shows all three live method labels and trajectories, includes the results card and workspace/collision evidence, and has readable text. Only then check off the video items in `SUBMISSION.md`.
