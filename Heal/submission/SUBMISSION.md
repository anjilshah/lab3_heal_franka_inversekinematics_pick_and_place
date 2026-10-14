# Lab 3 submission checklist

## Repository and written material

- [x] Organize both projects and rename the same repository to [lab3_heal_franka_inversekinematics_pick_and_place](https://github.com/anjilshah/lab3_heal_franka_inversekinematics_pick_and_place). See the root `VALIDATION.md` for executed checks; push status must be verified against remote `main`.
- [x] Final report source: [`submission/Lab3_Final_Report.docx`](Lab3_Final_Report.docx).
- [x] Final report PDF: [`submission/Lab3_Final_Report.pdf`](Lab3_Final_Report.pdf), four A4 pages.
- [ ] Confirm PDF typography requirement. The DOCX styles specify Times New Roman 12 pt body, 14 pt bold main headings, and 12 pt bold subheadings. This environment lacks Times New Roman, so LibreOffice substituted Liberation Serif in the rendered PDF; the PDF therefore does not strictly verify the required font.
- [x] `README.md` documents installation, model assets, baseline, solver comparisons, workspace/collision tools, and viewer commands.
- [x] `requirements.txt` lists the project dependencies.

## Code, model, and measured evidence

- [x] Preserved DLS baseline: `scripts/pick_and_place.py`.
- [x] DLS, Mink, and QP implementations and evaluation: `utils/ik_solvers.py`, `scripts/ik_solver_smoke.py`, and `scripts/batch_evaluate.py`.
- [x] Workspace sampling and passive contact audit: `scripts/workspace_collision_audit.py`, `utils/collision_audit.py`.
- [x] Opt-in sampled segment check: `scripts/collision_aware_pick_place.py`, `utils/collision_aware.py`.
- [x] HEAL + Robotiq model and assets: `robot_descriptions/arm_gripper.xml`, `robot_descriptions/heal_meshes/`, and `robot_descriptions/robotiq_2f85_v4/`.
- [x] Workspace samples and summary: `results/stage2/workspace_samples.csv`, `results/stage2/workspace_summary.json`; workspace figures: `workspace_task_xy_contact.png`, `workspace_dexterous_xy_contact.png`, `workspace_task_3d.png` under `results/stage2/`.
- [x] Matched Stage 3 logs and metrics: `results/stage3/batch/episodes.jsonl` (75 records, 25 per method), `manifest.json`, and `summary.json`; comparison plots are saved in the same directory.
- [x] Stage 4 nominal and rejected descent evidence: `results/stage4/collision_aware_smoke.json`, `results/stage4/segment_clearance.png`.
- [x] Stage 5 submission evidence audit: `results/stage5/submission_audit.md`.
- [x] Results card: `results/video/ik_comparison_1080p.png`.
- [x] Confirm `.venv/`, `__pycache__/`, Matplotlib caches, and `backups/` are excluded. No such paths are in the reviewed Git index.

## Demonstration video — incomplete

- [ ] Submit a roughly three-minute recording with about one minute per IK method, showing the actual HEAL + Robotiq simulation, labeled DLS/Mink/QP, and grasp, lift, transfer, release, and placement.
- [ ] Verify complete playback, method labels, task phases, and any final comparison table before marking the video complete.
- [x] A captured three-panel clip exists at [`Lab3_IK_Comparison_DLS_Mink_QP.webm`](Lab3_IK_Comparison_DLS_Mink_QP.webm). GStreamer identifies it as a 1920×1080 VP8 WebM of 51.43 seconds. Sampled frames show the three labeled live panels and a final benchmark panel, but the clip is substantially shorter than the assignment requirement and was not played from beginning to end. It is not renamed to `.mp4` because its actual container is WebM.
- [x] Live viewer runners are available: `scripts/ik_viewer_demo.py` and `scripts/parallel_ik_viewer.py`. Their existence does not itself satisfy the video requirement.
- [x] Recording storyboard: `VIDEO_RECORDING.md`.

## Reproduction commands

Run from the repository root. See `README.md` for setup and options.

```sh
# Baseline viewer run
./.venv/bin/python Heal/scripts/pick_and_place.py

# One-window live three-method viewer
./.venv/bin/python Heal/scripts/parallel_ik_viewer.py --speed 0.2

# Headless comparison smoke
./.venv/bin/python Heal/scripts/ik_solver_smoke.py --output Heal/results/ik_smoke

# Workspace and passive collision audit
./.venv/bin/python Heal/scripts/workspace_collision_audit.py --mode all --output Heal/results/stage2

# Stage 4 nominal and deliberately obstructed checks (headless)
./.venv/bin/python Heal/scripts/collision_aware_pick_place.py --output Heal/results/stage4 --clearance-margin-mm 0.75 --path-samples 25
```
