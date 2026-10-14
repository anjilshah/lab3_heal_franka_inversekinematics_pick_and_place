# Franka Panda Pick and Place

This self-contained project adds a Franka Panda task under `franka/`. It uses the local Panda MJCF and mesh assets in `models/franka/`, with the Panda's seven revolute joints, seven position actuators, coupled two-finger hand, and tendon gripper actuator. No HEAL code or results are required at runtime.

## Model and task

`models/franka/panda.xml` is a bundled Panda MJCF with its associated collision and visual assets. The model defines `joint1` through `joint7`, `actuator1` through `actuator7`, two coupled finger slide joints, and tendon position actuator `actuator8` (255=open, 0=closed, as set by its force law). `ee_site` is a site placed at the finger pad center based on the included hand geometry. The scene defines a 0.76 × 0.96 m table (top at z=0.375 m), a 0.05 m cube (half extents 0.025 m), and start/target centers at (0.18, -0.16, 0.400) m and (0.18, 0.16, 0.400) m. Coordinates are world-frame meters.

All 67 bundled mesh assets and the model license match the official [MuJoCo Menagerie Panda model at revision `0059d4335f8156206f63a35662313385f7ad6d74`](https://github.com/google-deepmind/mujoco_menagerie/tree/0059d4335f8156206f63a35662313385f7ad6d74/franka_emika_panda) byte-for-byte. The upstream model is released under Apache-2.0. Its license and model documentation are preserved alongside [SHA-256 provenance](models/franka/ASSET_PROVENANCE.json) and [attribution/local-modification notices](models/franka/NOTICE.md). Verify the local files with `./.venv/bin/python franka/scripts/verify_assets.py`.

## Setup

The parent repository's `.venv` already has the packages below. From the repository root:

```sh
./.venv/bin/python franka/scripts/validate_model.py
```

For a separate environment, use Python 3.10 and install:

```sh
python3 -m venv franka/.venv
franka/.venv/bin/python -m pip install -r franka/requirements.txt
```

Run all commands from the repository root or use the paths shown below. Scripts resolve model and output paths relative to this directory.

## Experiments

```sh
# Model, names, dimensions and actuator validation
./.venv/bin/python franka/scripts/validate_model.py

# DLS, Mink/DAQP and QP/DAQP smoke plus matched one-seed episode checks
./.venv/bin/python franka/scripts/ik_smoke.py --output franka/results/smoke_check

# Nominal headless episode and sampled contact/clearance log
./.venv/bin/python franka/scripts/pick_place.py --method dls --output franka/results/nominal_dls_rerun

# Optional interactive nominal run
./.venv/bin/python franka/scripts/pick_place.py --method mink --viewer --output franka/results/viewer_rerun

# Endpoint workspace and nominal collision audit
./.venv/bin/python franka/scripts/workspace_collision_audit.py --grid 4 --output franka/results/workspace_collision_rerun

# Discrete joint-segment check and actual runtime contact monitoring
./.venv/bin/python franka/scripts/collision_aware.py --samples 25 --output franka/results/collision_aware_rerun

# Challenging boundary placement (no added obstacle in this model)
./.venv/bin/python franka/scripts/challenging_case.py --output franka/results/challenging_rerun

# Matched-seed experiment (25 seeds per solver) and plots/statistics
./.venv/bin/python franka/scripts/batch_compare.py --episodes 25 --output franka/results/batch_rerun

# Regenerate summary/plots from saved JSONL without rerunning episodes
./.venv/bin/python franka/scripts/batch_compare.py --mode summarize --output franka/results/batch_final
```

The batch script refuses to overwrite an existing `episodes.jsonl`; choose a new output directory for another raw run. Seeds determine the exact shared cube and place coordinates. Each solver uses the same 8 task phases and initial conditions for each seed. DLS uses a damped Jacobian update; Mink uses a differential frame task and DAQP limits; QP solves a bounded weighted least-squares update with DAQP. IK endpoints use a 2 mm / 0.02 rad residual threshold and a shared 1.2 rad/s joint velocity bound per solver update. All methods also pass a 25-sample joint-linear table/floor clearance gate that rejects negative signed distance. Shared local path refinement reserves 1.21408 mm for the measured adverse command-to-state clearance error plus 0.001 mm numerical slack. Joint limits and existing 2 mm / 0.02 rad IK pose tolerances remain enforced. Observed unexpected penetration fails the episode. The path check is discrete and cannot guarantee clearance between samples.

Workspace samples record endpoint IK success only. They do not establish complete reachable volume or path reachability. The collision audit samples selected robot/table/floor/cube geom pairs every ten physics steps, distinguishes hand/cube and cube/table contacts from unexpected contacts, and is not a certified planner. The experimental path check linearly interpolates joint coordinates and samples discrete states; narrow collisions between samples may be missed. No tray or obstacle geometry exists in this scene.

## Live three-method comparison window

Launch the synchronized Franka viewer from the repository root:

```sh
MUJOCO_GL=egl ./.venv/bin/python franka/scripts/parallel_viewer.py --speed 0.8
```

It opens one dark Tk window with independent DLS, Mink, and QP MuJoCo scenes. The panels use matched start poses, targets, camera settings, and physics steps. The status above each scene shows the active phase, last solver status, live end-effector position/orientation residuals, and—after completion—the cube placement result under the strict 10 mm XY tolerance. Playback speed is simulated seconds per wall second. Click **Start / Reset All** or press Space to start or reset all panels together. Close the window to exit; an interrupted episode is not reported as a placement success. The viewer does not overwrite saved episode or batch records.

The viewer uses the repository `.venv` with MuJoCo, NumPy, Pillow, and the standard-library Tkinter GUI. Its synchronized simulation engine can be checked without opening a window:

```sh
./.venv/bin/python franka/scripts/parallel_viewer.py --headless-check
```

Clicking **Start / Reset All** starts a new matched DLS/Mink/QP simulation run in the live panels. Each panel calls the existing Franka IK solver, advances its own MuJoCo model state, and renders that state. Exactly one visible table, **FINAL RUN RESULTS**, updates from newly completed episodes and reports episode count, task successes/rates, solver-measured IK time, monotonic wall time, and sampled clearance. Success uses the strict <10 mm XY tolerance, >45 mm lift, and final z in (0.39, 0.49) m. Each raw episode stores the per-step cube/EE trajectory, hand/cube contacts during grasp and carry, release observations, solver residuals, joint-limit events, path checks, and sampled runtime collision diagnostics. Clearance is a sampled signed geom distance, not a guarantee of collision-free motion. Every run is written to a unique `results/live_runs/run_<timestamp>/` directory with `manifest.json`, `episodes.jsonl`, and `summary.json`; interrupted runs are retained and marked interrupted. Pressing **Start / Reset All** during or after a run preserves that run and resets all three scenes for a new run. Historical Stage 3 data remains on disk under `results/batch_final/` and is not shown as a second table.

## Results and submission

Raw logs and plots live only in `franka/results/`; report and checklist live in `franka/submission/`. Completed evidence includes `results/smoke_final_check/`, `results/nominal_final_dls/`, `results/nominal_final_mink/`, `results/nominal_final_qp/`, `results/workspace_collision_reviewed/`, `results/collision_aware_gated_corrected/`, `results/challenging_final/`, and `results/batch_final/`. Measured results are summarized in `submission/REPORT.md`. Do not interpret an IK smoke pass as a successful pick-and-place episode.

The three original videos under `Heal/submission/` belong to HEAL, including `Lab3_IK_Comparison_DLS_Mink_QP.mp4` and `.webm`. A Franka video is generated separately from the real MuJoCo episode with:

```sh
./.venv/bin/python franka/scripts/render_episode_video.py --method dls --seed 20261013 --output franka/submission/franka_nominal_dls_repeat.mp4
```

No Franka MP4 is currently present in this checkout; the former `submission/franka_nominal_dls_verified.mp4` reference was not backed by a file. A new recording is still needed if submitting a Franka video. The video script captures the simulated episode at 25 frames per second and refuses to overwrite an existing output. It requires the `.venv` packages MuJoCo and Pillow plus the system `ffmpeg` executable.

`models/franka/mjx_cube+plate.xml` is an optional MJX example, not used by the validated pick-and-place workflow or listed as a Franka submission deliverable. It references the external plate at `models/external/Ecoforms_Plate_S20Avocado/model.xml`, a portable relative path. That dependency is absent, so this optional example is not included in the validated workflow. Its plate geometry has not been replaced or removed.

```sh
./.venv/bin/python franka/scripts/batch_compare.py --episodes 25 --output franka/results/batch_reproduction
```

## Latest small matched-seed verification

The corrected implementation passed DLS **3/3**, Mink **3/3**, and QP **3/3** on seeds 20261013–20261015. All nine completed retreat, passed strict placement, had zero recorded unexpected contacts and zero joint-limit violations. Raw records are in `results/retreat_allowance_matched_3/episodes.jsonl`; `full_geom_clearance_verification.json` checks all robot/table-floor pairs at every saved physics step. Minimum executed clearances across those three seeds were 8.701 mm (DLS), 1.210 mm (Mink), and 1.210 mm (QP). This is historical evidence from three seeds and is not a claim about the old 25-seed batch or all future runs.

```sh
./.venv/bin/python franka/scripts/validate_clearance.py --methods dls mink qp --seeds 20261013 20261014 20261015 --output franka/results/my_new_verification
```

The output directory must be new. The audit measures fresh FK of actual joint states at every physics step and saves raw episode and clearance traces.

## Submission checklist

- [x] Verify all 67 mesh hashes against pinned Menagerie revision and preserve Apache-2.0 notices.
- [ ] Run model validation, solver smoke, nominal task, workspace/collision audits, boundary case, and matched-seed batch.
- [ ] Review saved raw JSON/JSONL and all generated plots.
- [ ] Update `submission/REPORT.md` from the measured results and state failures and unsupported features.
- [ ] Verify all new files remain inside `franka/` and preserve the original HEAL project.
