#!/usr/bin/env python3
"""Open or compile the vendored standalone Robotiq 2F-85 MJCF."""
import argparse
from pathlib import Path
import mujoco
import mujoco.viewer

MODEL_PATH = Path(__file__).resolve().with_name('2f85.xml')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='compile without opening a viewer')
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    if args.check:
        print(f'Model compiled: {MODEL_PATH} (nq={model.nq}, nv={model.nv}, nu={model.nu})')
        return
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == '__main__':
    main()
