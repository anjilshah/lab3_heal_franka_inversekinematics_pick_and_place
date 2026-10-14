# Franka Panda Pick and Place

This self-contained project adds a Franka Panda task under `franka/`. It uses the local Panda MJCF and mesh assets in `models/franka/`, with the Panda's seven revolute joints, seven position actuators, coupled two-finger hand, and tendon gripper actuator. No HEAL code or results are required at runtime.

## Model and task

`models/franka/panda.xml` is a bundled Panda MJCF with its associated collision and visual assets. The model defines `joint1` through `joint7`, `actuator1` through `actuator7`, two coupled finger slide joints, and tendon position actuator `actuator8` (255=open, 0=closed, as set by its force law). `ee_site` is a site placed at the finger pad center based on the included hand geometry. The scene defines a 0.76 × 0.96 m table (top at z=0.375 m), a 0.05 m cube (half extents 0.025 m), and start/target centers at (0.18, -0.16, 0.400) m and (0.18, 0.16, 0.400) m. Coordinates are world-frame meters.

The bundled model retains its source model license in `models/franka/LICENSE`. Its upstream provenance has not been resolved from the local checkout; confirm redistribution terms before publishing the assets.

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

The batch script refuses to overwrite an existing `episodes.jsonl`; choose a new output directory for another raw run. Seeds determine the exact shared cube and place coordinates. Each solver uses the same 8 task phases and initial conditions for each seed. DLS uses a damped Jacobian update; Mink uses a differential frame task and DAQP limits; QP solves a bounded weighted least-squares update with DAQP. The position and orientation residual tolerances are 20 mm and 0.12 rad. These are numerical endpoint criteria, not claims of high precision manipulation.

Workspace samples record endpoint IK success only. They do not establish complete reachable volume or path reachability. The collision audit samples selected robot/table/floor/cube geom pairs every ten physics steps, distinguishes hand/cube and cube/table contacts from unexpected contacts, and is not a certified planner. The experimental path check linearly interpolates joint coordinates and samples discrete states; narrow collisions between samples may be missed. No tray or obstacle geometry exists in this scene.

## Results and submission

Raw logs and plots live only in `franka/results/`; report and checklist live in `franka/submission/`. Completed evidence includes `results/smoke_final_check/`, `results/nominal_final_dls/`, `results/nominal_final_mink/`, `results/nominal_final_qp/`, `results/workspace_collision_reviewed/`, `results/collision_aware_gated_corrected/`, `results/challenging_final/`, and `results/batch_final/`. Measured results are summarized in `submission/REPORT.md`. Do not interpret an IK smoke pass as a successful pick-and-place episode.

```sh
./.venv/bin/python franka/scripts/batch_compare.py --episodes 25 --output franka/results/batch_reproduction
```

## Submission checklist

- [ ] Confirm model attribution and license terms before redistributing the bundled meshes.
- [ ] Run model validation, solver smoke, nominal task, workspace/collision audits, boundary case, and matched-seed batch.
- [ ] Review saved raw JSON/JSONL and all generated plots.
- [ ] Update `submission/REPORT.md` from the measured results and state failures and unsupported features.
- [ ] Verify all new files remain inside `franka/` and preserve the original HEAL project.
