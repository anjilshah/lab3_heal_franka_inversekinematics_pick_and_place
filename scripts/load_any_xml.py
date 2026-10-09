#!/usr/bin/env python3
"""Compile or open a MuJoCo XML model; paths default to this Lab 3 model."""
import argparse
from pathlib import Path
import mujoco
import mujoco.viewer
from robot_common import MODEL_PATH


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('xml', nargs='?', type=Path, default=MODEL_PATH,
                        help='MJCF file (default: robot_descriptions/arm_gripper.xml)')
    parser.add_argument('--check', action='store_true', help='compile the model without opening a viewer')
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.xml.resolve()))
    if args.check:
        print(f'Model compiled: {args.xml.resolve()} (nq={model.nq}, nv={model.nv}, nu={model.nu})')
        return
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == '__main__':
    main()
