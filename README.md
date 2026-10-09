# ITR-IK-MJ — Lab 3 HEAL pick and place

This project models a HEAL six-axis arm and Robotiq 2F-85 gripper in MuJoCo. The scene contains a static worktable and a free 4 cm cube. Gripper integration uses the standalone Robotiq MJCF's mesh scaling and transforms, link inertials and collision geometry, fingertip pads, linkage constraints, tendon, and force-limited actuator.

## Environment

The audited environment is `./.venv` (Python 3.10.12, MuJoCo 3.15.0, NumPy 2.2.6). Recreate it locally with:

```sh
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
```

Run commands from the project root with `./.venv/bin/python`. The main model paths are resolved relative to project files, not the shell's current directory.

## Simulation and evaluation

Running the pick-and-place script with no options opens the MuJoCo viewer immediately and performs one pick/place. The window remains open when the sequence ends; close it to exit. Add `--headless` to run the same episode without a GUI. Workspace, randomized, demonstration, and failure-evaluation modes remain headless batch runs.

```sh
./.venv/bin/python scripts/pick_and_place.py
./.venv/bin/python scripts/pick_and_place.py --headless
./.venv/bin/python scripts/pick_and_place.py --workspace --output results_workspace
./.venv/bin/python scripts/pick_and_place.py --randomized 8 --seed 21 --output results_randomized
./.venv/bin/python scripts/pick_and_place.py --demonstrations 1 --seed 11 --output results_demonstrations
./.venv/bin/python scripts/pick_and_place.py --failure-eval 5 --seed 5 --output results_failures
./.venv/bin/python scripts/test_arm_hold.py
./.venv/bin/python scripts/test_gripper.py
```

The headless pick/place command writes `results/episodes.json`. Randomized scenarios perturb cube start position by ±4 cm and placement target by ±5 cm. Workspace analysis writes Cartesian IK position/orientation residuals to CSV. Demonstration mode saves timestamped arm, cube, and gripper-command samples. Failure evaluation deliberately offsets the pick target from the cube. Pick/place success requires a lift of more than 5 cm and a released cube within 7 cm of its target at table height.

All project-root scripts with viewer entry points support `--check` to compile the relevant model without opening a GUI:

```sh
./.venv/bin/python scripts/load_any_xml.py --check
./.venv/bin/python scripts/load_heal_robot.py --check
./.venv/bin/python scripts/example_controlling_gripper.py --check
./.venv/bin/python scripts/example_controlling_velocity.py --check
./.venv/bin/python scripts/view_arm_hold.py --check
./.venv/bin/python robot_descriptions/robotiq_2f85_v4/test.py --check
```

`SUBMISSION.md` lists the intended submission files and excludes the preserved `backups/` archive and local environment. `REPORT.md` contains measured tests and limitations. Some unused starter descriptions for UR5/Franka remain incomplete and are not required or used by the Lab 3 HEAL model.
