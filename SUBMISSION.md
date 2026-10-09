# Lab 3 submission checklist

## Include

- `README.md`, `REPORT.md`, `requirements.txt`
- `scripts/`, `utils/`
- `robot_descriptions/arm_gripper.xml`, `robot_descriptions/robotiq_2f85_v4/`, and `robot_descriptions/heal_meshes/`
- `results/episodes.json`, `results_randomized/episodes.json`, `results_demonstrations/episodes.json`, `results_failures/episodes.json`, and `results_workspace/workspace.csv`

## Exclude

- `.venv/` and `__pycache__/`
- `backups/` (preservation archive; not used by runtime scripts)
- Unused starter robot descriptions that are not part of the HEAL + Robotiq Lab 3 model

## Reproduce

See the exact environment setup and headless test commands in `README.md`. The measured results and known limitations are in `REPORT.md`.
