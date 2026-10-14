# Franka submission checklist

- [x] Panda MJCF, hand geometry, joints and actuators validated.
- [x] DLS, Mink and QP solver smoke records reviewed.
- [x] Nominal headless episode and collision log reviewed.
- [x] Workspace samples and plots reviewed; endpoint sampling limitations stated.
- [x] Collision audit and sampled path check reviewed; no certification claim made.
- [x] Matched-seed batch raw JSONL, summary, and plots reviewed.
- [x] Boundary scenario outcome documented.
- [x] Report distinguishes saved nominal/batch results from the fresh smoke run and rendered video.
- [ ] Record a Franka video: no Franka MP4 is present in this checkout; original HEAL videos are preserved under `Heal/submission/`.
- [x] All 67 meshes and LICENSE match Menagerie revision `0059d4335f8156206f63a35662313385f7ad6d74`; provenance hashes and Apache-2.0 attribution retained.
- [x] Original HEAL assets/results/videos preserved while moving the project under `Heal/`; source imports/output paths updated.
- [x] Optional `models/franka/mjx_cube+plate.xml` dependency limitation documented; this example is not part of the validated submission workflow.

- [x] Corrected controller: nine real matched episodes, three per method on seeds 20261013–20261015; strict placement and executed-clearance checks passed.
