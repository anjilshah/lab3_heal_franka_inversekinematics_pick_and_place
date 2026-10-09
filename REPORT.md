# Lab 3 report — HEAL and Robotiq 2F-85

## Model integration

The project Python environment is `./.venv` (Python 3.10.12) with MuJoCo 3.15.0 and NumPy installed. No system-wide packages were installed. The standalone Robotiq `2f85.xml` was used as the gripper source. Its 8 mesh assets and relative asset paths compile both standalone and in the combined arm model. The combined model uses the source `2f85` defaults (including 0.001 mesh scaling), per-geom transforms, visual/collision layers, inertials, pad boxes, driver and follower joints, connect and joint equalities, split tendon, and 0–255 actuator with ±5 N actuator force range. The source base body is mounted under the HEAL wrist joint. The combined MJCF compiles as MuJoCo 3.15.0 (`nq=19`, `nv=18`, `nu=7`).

A static table and a 0.08 kg free cube were added. Since the arm model sets `inertiafromgeom=false`, the cube has explicit mass and box inertia. The pick controller uses damped least-squares position and orientation IK, gravity/bias-compensated joint PD control, smooth joint-target ramps, and the source gripper actuator. Episode success requires a measured cube lift of at least 5 cm and release within 7 cm of the requested x/y placement at table height.

## Measured tests

The workspace did not contain a formal assignment handout or rubric, so this audit maps the implementation to the requirements stated in the project request. `SUBMISSION.md` lists intended deliverables. Backups are preserved under `backups/` and excluded from runtime and submission inputs.

The reported metric runs below were executed headless. The default entry point now opens a passive MuJoCo viewer before starting the episode and keeps it open afterward. The actual GUI window was not launched during this audit; GUI control flow was checked with a passive-viewer test double.

- MJCF compilation: standalone gripper and combined scene both passed.
- Python syntax: `./.venv/bin/python -m py_compile scripts/*.py robot_descriptions/robotiq_2f85_v4/test.py` passed.
- Default GUI path: fake passive-viewer control-flow test verified viewer launch occurs before the first simulation step, simulation calls `sync`, completed runs wait for the viewer to close, and the episode success metric remains passing. A real graphical window was not opened.
- Headless regression after GUI integration: `--headless` passed with cube peak z 0.30975 m and final position `[0.35214, 0.17795, 0.15994]` m; JSON stdout remains machine-readable.
- Dependency audit: `./.venv/bin/python -m pip check` reported no broken requirements; `requirements.txt` records the installed MuJoCo and NumPy versions.
- Project-root entry points: the loader, arm hold viewer, gripper viewer, velocity demo, and standalone Robotiq viewer all passed their `--check` path without opening a GUI. A headless velocity-control step check passed.
- Asset/model compile: the active HEAL+Robotiq model, standalone Robotiq model/scene, and compatibility XML aliases compiled. Main mesh references resolve from project paths.
- Arm hold: three-second hold completed; all six arm joints remained effectively at 0 rad.
- Gripper actuation: open command 0 produced driver positions 0.0031/0.0031 rad; close command 255 produced 0.8238/0.8241 rad (symmetry error 0.000286 rad).
- Nominal pick/place: passed 1/1. Cube started at `[0.3500, 0.0000, 0.1650]` m, reached a peak z of 0.30975 m, and settled at `[0.35214, 0.17795, 0.15994]` m for target `[0.350, 0.180, 0.165]` m.
- Randomized pick/place: passed 8/8 using seed 21. Each cube start was perturbed by up to ±4 cm in x/y and each destination by up to ±5 cm; initial and final cube positions, pick targets, and placement targets are in `results_randomized/episodes.json`.
- Workspace sampling: 95/125 grid points met position error <25 mm and orientation error <0.05 rad over x=[0.15,0.55], y=[-0.25,0.30], z=[0.18,0.55] m. Full data is in `results_workspace/workspace.csv`.
- Demonstration: passed 1/1 and saved 382 timestamped arm/cube/gripper samples in `results_demonstrations/episodes.json`.
- Failure evaluation: 0/5 succeeded when the pick target was deliberately offset from the cube by deterministic pick-target offsets of (2,0), (4,0), (6,0), (0,6), and (8,8) cm. These cases failed to lift, as expected. Data is in `results_failures/episodes.json`.

## Audit findings and limitations

Several unused starter XML descriptions remain outside the submission model and do not compile in this workspace: `dual_ur5.xml` and `world.xml` use the unsupported `mujocoimport` format; `franka_scene.xml` and `load_dual_ur_5.xml` reference absent include files; `load_ur_5.xml` has a repeated `silver` material; and `ur5.xml` references absent mesh assets. They are excluded from the HEAL Lab 3 deliverables in `SUBMISSION.md`. The former machine-specific `arm_gripper_2.xml` and empty Robotiq compatibility files now include the maintained relative-path models.


The workspace result describes position and orientation IK reachability only; it does not include collision-free path planning. Randomization varies cube start and destination positions, but not cube properties, mass, or friction. Failure evaluation currently covers pick-target localization offsets, not actuator faults or sensor noise. The scripted controller has passed the listed nominal and randomized simulation scenarios, but these results are from MuJoCo simulation and do not establish real-world performance.
