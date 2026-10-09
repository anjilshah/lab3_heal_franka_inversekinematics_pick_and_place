#!/usr/bin/env python3
"""Open the project HEAL + Robotiq model with gravity compensation."""
import argparse
import time
import mujoco
import mujoco.viewer
from robot_common import MODEL_PATH, arm_indices, load_model


def compensate_arm(model, data, dof_ids, actuator_ids):
    mujoco.mj_forward(model, data)
    for aid, dof in zip(actuator_ids, dof_ids):
        data.ctrl[aid] = data.qfrc_bias[dof]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='compile without opening a viewer')
    args = parser.parse_args()
    model, data, _, dof_ids, arm_actuators, *_ = load_model()
    if args.check:
        print(f'Model compiled: {MODEL_PATH} (nq={model.nq}, nv={model.nv}, nu={model.nu})')
        return
    print('Close the MuJoCo window or press Ctrl+C to stop.')
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                start = time.time()
                compensate_arm(model, data, dof_ids, arm_actuators)
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(max(0, model.opt.timestep-(time.time()-start)))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
