# Final organization and validation — 2026-10-14

The repository contains two independent projects: `Heal/` and `franka/`.
The move inventory is in `Heal/submission/REORGANIZATION.json`.
Models, saved results, and all three original HEAL videos were preserved.

## Checks actually executed

| Check | Result |
| --- | --- |
| Python compilation of both projects' scripts and utilities | Passed |
| Imports of both model/controller utilities and dashboard modules | Passed |
| HEAL model loading with the existing `.venv` | Passed: nq=19, nv=18, nu=7 |
| HEAL synchronized headless DLS/Mink/QP episodes | Passed: all three completed eight phases and placed the cube |
| HEAL three-panel offscreen frames | Passed: three RGB frames, 400×520 |
| HEAL desktop startup, Start/Reset, and clean close | Passed |
| HEAL result saving after reset | Passed: existing output retained and new record written separately |
| HEAL historical results reader | Passed |
| Franka model validation, MuJoCo 3.15.0 | Passed: nq=16, nv=15, nu=8 |
| Franka local mesh/license integrity | Passed: 67/67 mesh hashes and license hash match the verified upstream revision |
| Franka real desktop GUI self-test at speed 4 | Passed: three rendered panels, Start/Reset, exactly one results table, table/record agreement, and clean close |
| JSON/JSONL result integrity | Passed: 158 files parsed, including 95,305 JSONL records |
| Original HEAL asset/result/video preservation | Passed: all 76 protected file hashes unchanged after moving |
| Original HEAL video inspection | Passed: all three opened with ffprobe; original hashes unchanged |
| Current README/checklist local links | Passed |
| Git whitespace checks | Passed |

The fresh Franka GUI comparison is saved in
`franka/results/live_runs/run_20261014T031532.524964Z_993520/`.
The subsequent reset is preserved separately in
`franka/results/live_runs/run_20261014T031537.573158Z_178954/`.

| Method | Completed | Placement success | Final XY error (mm) | Mean measured IK call time (ms) | Measured episode wall time (s) | Minimum sampled clearance (mm) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| DLS | 1 | 1/1 | 1.853 | 207.28 | 4.768 | 8.366 |
| Mink | 1 | 1/1 | 1.368 | 402.99 | 4.768 | 1.215 |
| QP | 1 | 1/1 | 1.452 | 327.04 | 4.768 | 1.215 |

These numbers are measured current verification episodes, not a new 25-seed
benchmark. The existing matched seeds 20261013–20261015 retain 3/3 successes per
method in `franka/results/retreat_allowance_matched_3/`. Historical unsuccessful
trials and the older 25-seed batch remain on disk unchanged.

Franka success still requires XY error <10 mm, lift >45 mm above initial cube z,
and final cube z strictly between 0.39 m and 0.49 m. Unsafe sampled paths and
unexpected robot/environment penetration fail the episode. Discrete clearance
checks are simulation diagnostics, not a continuous-time or hardware guarantee.

## Model provenance

All 67 bundled Panda meshes were compared byte for byte with the official
Google DeepMind MuJoCo Menagerie Panda model at revision
`0059d4335f8156206f63a35662313385f7ad6d74`: zero mismatches or missing files.
The exact upstream Apache-2.0 license, README, changelog, attribution, and
per-file SHA-256 manifest are preserved in `franka/models/franka/`.

## Remaining submission limitations

- No Franka MP4 currently exists in this workspace. Record the live dashboard
  or run the documented video script with a new output path.
- The original HEAL clips were preserved; they are shorter than the assignment's
  roughly three-minute recording requirement. Its existing PDF also has the
  documented font-substitution limitation. These deliverable gaps remain open.
- Optional legacy `mjx_cube+plate.xml` needs the external plate model under
  `franka/models/external/Ecoforms_Plate_S20Avocado/`. Its machine-specific
  absolute include was replaced with a portable relative include. This optional
  example is outside the validated Panda task and dashboard.
- Virtual environments, Python caches, Matplotlib caches, and local backups
  are ignored. Authentication credentials are outside this repository.

The existing GitHub repository was renamed to
`anjilshah/lab3_heal_franka_inversekinematics_pick_and_place` with its numeric
repository ID unchanged (`1412409792`), preserving its history and `main` branch.
Publication status is verified against GitHub after committing and pushing;
this document records validation evidence, not an assertion that a push occurred.
