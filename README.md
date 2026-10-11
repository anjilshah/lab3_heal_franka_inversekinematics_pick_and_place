# HEAL pick-and-place in MuJoCo

This project runs a HEAL six-axis arm with a Robotiq 2F-85 gripper, cube, and table in MuJoCo. The original DLS controller in [`scripts/pick_and_place.py`](scripts/pick_and_place.py) remains the baseline. Solver comparison, workspace, collision, and result details are in [`REPORT.md`](REPORT.md); the current evidence checklist is [`SUBMISSION.md`](SUBMISSION.md).

## Setup

Use Python 3.10 or a compatible Python environment. From the project root:

```sh
python3 -m venv .venv
./.venv/bin/python -m pip install --upgrade pip
./.venv/bin/python -m pip install -r requirements.txt
```

The active model is [`robot_descriptions/arm_gripper.xml`](robot_descriptions/arm_gripper.xml). It includes the HEAL meshes and the Robotiq model/assets under `robot_descriptions/`. Run commands from the repository root. Do not remove or rename model assets referenced by the MJCF.

## Run the original DLS baseline

With no options, the original script opens the MuJoCo viewer and runs one nominal pick-and-place episode. Close the window after the sequence to exit. Add `--headless` to run without a viewer.

```sh
./.venv/bin/python scripts/pick_and_place.py
./.venv/bin/python scripts/pick_and_place.py --headless --output results
```

The baseline preserves the existing grasp, lift, transfer, and placement behavior and writes `episodes.json` under the chosen output directory.

## Run and compare the three IK methods

The compatible methods are custom DLS, Mink differential IK, and QP with the DAQP backend. The smoke command solves the same reachable target with each solver and runs one matched pick-and-place episode per method. It runs headlessly and writes per-method metrics to `results_ik_smoke/smoke_and_episodes.json`.

```sh
./.venv/bin/python scripts/ik_solver_smoke.py --output results_ik_smoke
```

To run the reproducible comparison (25 matched seeds per method; 75 episodes total), then regenerate summary/plots from its raw JSONL:

```sh
./.venv/bin/python scripts/batch_evaluate.py --mode batch --episodes 25 --output results_stage3/batch
./.venv/bin/python scripts/batch_evaluate.py --mode summarize --output results_stage3/batch
```

The batch is headless and takes longer than the smoke. Raw episodes are in `results_stage3/batch/episodes.jsonl`; plots and metrics are generated in the same directory. `--mode smoke` runs one matched seed per method:

```sh
./.venv/bin/python scripts/batch_evaluate.py --mode smoke --output results_stage3
```

IK smoke and batch scripts do not open a viewer. Use the separate viewer demo below for live trajectories with each method.

For interactive demonstrations, use the method-selectable runner. Each invocation resets to the same cube start, placement target, orientation, and eight task phases. The existing torque tracker simulates the gripper and cube; each selected IK solver computes waypoint targets online. The default playback speed is 0.6 simulation seconds per wall second (about 25 seconds for the 15.2-second simulated task, plus solver/render overhead). `--speed` changes wall-clock pacing without skipping simulation steps. The MuJoCo passive viewer uses its text overlay for a persistent method label; the terminal reports phase residuals/status.

```sh
./.venv/bin/python scripts/ik_viewer_demo.py --method mink --speed 0.6
./.venv/bin/python scripts/ik_viewer_demo.py --method dls --speed 0.6
./.venv/bin/python scripts/ik_viewer_demo.py --method qp --speed 0.6
```

After each run completes, close its viewer window, then launch the next command. Results are saved as `mink.json`, `dls.json`, or `qp.json` under `results_viewer_demo/`. To make a quick unpaced check without a GUI, add `--headless` and choose a separate output directory. `--playback-rate` remains an alias for `--speed`.

Generate a 1920×1080 comparison card from the saved Stage 3 JSONL and summary. It validates matched seeds and recomputes means from raw records before drawing:

```sh
./.venv/bin/python scripts/create_video_results_card.py
```

The output is `results_video/ik_comparison_1080p.png`. A timed OBS workflow, including live method commands and honest use of the headless Stage 4 output, is in [`VIDEO_RECORDING.md`](VIDEO_RECORDING.md).

To compare all three robots simultaneously in one window, use the combined runner. It creates three independent MuJoCo model/data instances and shows each live render in its own labeled panel. Press **Start / Reset All** (or Space) to restart the matched tasks together:

```sh
./.venv/bin/python scripts/parallel_ik_viewer.py --speed 0.6
```

The non-GUI synchronized task check is:

```sh
./.venv/bin/python scripts/parallel_ik_viewer.py --headless-check
```

## Workspace and collision evidence

Regenerate the sampled endpoint workspace analysis, labeled plots, and passive nominal collision audit with:

```sh
./.venv/bin/python scripts/workspace_collision_audit.py --mode all --output results_stage2
```

Use `--mode workspace` or `--mode collision` to run only one analysis. The workspace is sampled IK reachability; it does not certify paths. The collision audit observes contacts and distances without changing controller motion.

The opt-in Stage 4 run checks sampled path segments, rejects unsafe segments, and monitors runtime contacts/clearance. It runs headlessly and exercises both the nominal task and a deliberately unsafe low-pick case:

```sh
./.venv/bin/python scripts/collision_aware_pick_place.py --output results_stage4 --clearance-margin-mm 0.75 --path-samples 25
```

This is limited sampled-segment rejection, not a general or certified collision-free planner. The model has no tray geometry, and no tray avoidance is tested.

## Required three-minute demonstration video

An existing captured clip is [`submission/Lab3_IK_Comparison_DLS_Mink_QP.mp4.webm`](submission/Lab3_IK_Comparison_DLS_Mink_QP.mp4.webm). It is a 51.43-second, 1920×1080 VP8 WebM; sampled frames show the three labeled live panels and a final benchmark panel. It is shorter than the assignment's approximately three-minute requirement and has not been fully played through, so the required video remains incomplete. The single-window parallel runner shows all three online IK methods at once; use the results card and saved workspace/Stage 4 evidence for the remaining video time. Stage 4 itself is headless; show its actual terminal result and clearance plot, not a fabricated viewer trajectory.

Start OBS Studio from a graphical desktop:

```sh
obs
```

Select a desktop/window capture source, record the three labeled method segments in order, stop recording near 180 seconds, and verify that the saved video plays and contains all three trajectories. Follow [`VIDEO_RECORDING.md`](VIDEO_RECORDING.md) for suggested timings and evidence. Run one command at a time and close the viewer before starting the next:

```sh
./.venv/bin/python scripts/ik_viewer_demo.py --method mink --speed 0.6
./.venv/bin/python scripts/ik_viewer_demo.py --method dls --speed 0.6
./.venv/bin/python scripts/ik_viewer_demo.py --method qp --speed 0.6
```

The headless, matched all-method reproduction command is:

```sh
./.venv/bin/python scripts/ik_solver_smoke.py --output results_ik_smoke
```

This smoke runs all three methods headlessly and does not create video. The live commands above open the viewer and produce one online simulated trajectory for the selected method.
