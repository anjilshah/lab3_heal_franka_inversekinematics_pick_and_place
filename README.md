# Lab 3 — HEAL & Franka Inverse Kinematics Pick & Place

[GitHub repository](https://github.com/anjilshah/lab3_heal_franka_inversekinematics_pick_and_place)

Two independent MuJoCo projects compare DLS, Mink differential IK, and constrained QP for real pick-and-place simulation. Each dashboard drives separate simulated robots with the actual solvers. Solver convergence and object-placement success are reported separately.

See [final validation evidence and remaining submission limitations](VALIDATION.md).

```text
Lab3/
├── Heal/
│   ├── scripts/               # HEAL + Robotiq controllers and comparison viewer
│   ├── robot_descriptions/    # Original MJCFs and mesh assets
│   ├── utils/
│   ├── results/               # Original saved runs, stages, and plots
│   ├── submission/            # HEAL reports and original videos
│   ├── requirements.txt
│   └── README.md
├── franka/
│   ├── models/                # Panda models, verified meshes, license, provenance
│   ├── scripts/               # Franka controller, dashboard, and validation
│   ├── utils/
│   ├── results/               # Historical batches, diagnostics, and live runs
│   ├── submission/            # Franka report and checklist
│   ├── requirements.txt
│   └── README.md
├── requirements.txt
├── .gitignore
└── README.md
```

## Installation

Use Python 3.10 or a compatible version. Run commands below from this repository's root. The existing local `.venv` can be reused; it is excluded from Git.

```sh
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
```

The pinned simulation packages are MuJoCo 3.15.0, NumPy 2.2.6, Mink 1.3.0, qpsolvers/DAQP 4.13.0, and Matplotlib 3.10.8. The desktop dashboards also require Pillow, a working graphical display, and Tkinter (on Debian/Ubuntu, the system package is `python3-tk`). EGL is used for offscreen simulation panels. `ffmpeg` is optional for rendering a new video; it is not needed for either live dashboard.

## Launch the dashboards

HEAL + Robotiq:

```sh
MUJOCO_GL=egl ./.venv/bin/python Heal/scripts/parallel_ik_viewer.py --speed 0.6
```

Franka Panda:

```sh
MUJOCO_GL=egl ./.venv/bin/python franka/scripts/parallel_viewer.py --speed 0.8
```

Maximize the window and press **Start / Reset All** to record the real simulations together. Closing the window exits the application. Franka preserves its method colors and exactly one **FINAL RUN RESULTS** table, populated from current-run episode records. Each Franka start/reset creates a unique directory under `franka/results/live_runs/`; historical batches are not displayed as live results. HEAL's original dashboard retains its saved historical benchmark section.

## Results and success definitions

Franka task success requires final cube XY error **<0.010 m**, cube lift **>0.045 m** above its initial height, and final cube z strictly in **(0.39, 0.49) m**. Unsafe sampled paths and observed unexpected robot/environment penetration fail the episode. All methods use matching initial conditions and targets. Local collision refinement uses a shared execution allowance calibrated from measured command-to-state clearance loss; these discrete checks do not certify continuous-time or hardware safety.

The corrected implementation passed **3/3 episodes per method** on seeds 20261013–20261015. Raw records are in [`franka/results/retreat_allowance_matched_3/episodes.jsonl`](franka/results/retreat_allowance_matched_3/episodes.jsonl); complete per-step clearance verification is in [`full_geom_clearance_verification.json`](franka/results/retreat_allowance_matched_3/full_geom_clearance_verification.json). These are nine measured historical episodes, not a 25-seed benchmark or a guarantee for new runs. The older Franka 25-seed batch remains in `franka/results/batch_final/` and used the earlier controller. HEAL's separate 25-seed batch is in `Heal/results/stage3/batch/`; its original task definition and limitations are documented in its report.

Quick checks that preserve saved results:

```sh
./.venv/bin/python Heal/scripts/load_heal_robot.py --check
./.venv/bin/python Heal/scripts/view_lab3_results.py --no-plots
./.venv/bin/python franka/scripts/validate_model.py
./.venv/bin/python franka/scripts/verify_assets.py
MUJOCO_GL=egl ./.venv/bin/python franka/scripts/parallel_viewer.py --gui-self-test --speed 4
```

The GUI self-test executes a new real comparison, verifies the table against saved records, resets again, and closes. It creates separate live-run records. Use a fresh directory for any explicit experiment output.

## Reports, videos, and model sources

- [HEAL guide](Heal/README.md), [report](Heal/submission/REPORT.md), and [submission checklist](Heal/submission/SUBMISSION.md).
- [Franka guide](franka/README.md), [report](franka/submission/REPORT.md), and [checklist](franka/submission/CHECKLIST.md).
- Original HEAL videos: [`Lab3_IK_Comparison_DLS_Mink_QP.mp4`](Heal/submission/Lab3_IK_Comparison_DLS_Mink_QP.mp4), [`Lab3_IK_Comparison_DLS_Mink_QP.webm`](Heal/submission/Lab3_IK_Comparison_DLS_Mink_QP.webm), and [`Lab 3 IF Pick and place.webm`](Heal/submission/Lab%203%20IF%20Pick%20and%20place.webm). These show HEAL, not Franka. No Franka video is currently present; record its dashboard or use its documented video script to create one separately.
- All 67 bundled Franka meshes exactly match Google DeepMind's MuJoCo Menagerie Panda model at revision `0059d4335f8156206f63a35662313385f7ad6d74`. Its Apache-2.0 license, attribution, and per-file hashes are preserved under `franka/models/franka/`. See [model provenance](franka/models/franka/NOTICE.md).

The original Google Drive submission link is preserved in the [HEAL README](Heal/README.md). Optional legacy UR5/plate examples are retained but are not required by either validated pick-and-place workflow; their external dependencies are documented separately.
