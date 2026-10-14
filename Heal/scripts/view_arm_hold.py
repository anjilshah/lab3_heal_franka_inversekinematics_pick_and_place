#!/usr/bin/env python3
"""Open the HEAL arm + gripper and hold the arm against gravity."""
import argparse
import time
import mujoco
import mujoco.viewer
from robot_common import MODEL_PATH, load_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='compile the model without opening a viewer')
    args = parser.parse_args()
    model, data, qpos_ids, dof_ids, actuator_ids, *_ = load_model()
    if args.check:
        print(f'Model compiled: {MODEL_PATH} (nq={model.nq}, nv={model.nv}, nu={model.nu})')
        return
    target = data.qpos[qpos_ids].copy()
    kp, kd = 500.0, 50.0
    print('Viewer opened. Close the MuJoCo window or press Ctrl+C to stop.')
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                start = time.time()
                mujoco.mj_forward(model, data)
                for i, (aid, dof, qpos) in enumerate(zip(actuator_ids, dof_ids, qpos_ids)):
                    data.ctrl[aid] = data.qfrc_bias[dof] + kp * (target[i] - data.qpos[qpos]) - kd * data.qvel[dof]
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(max(0, model.opt.timestep - (time.time() - start)))
    except KeyboardInterrupt:
        pass
    print('Viewer stopped.')


if __name__ == '__main__':
    main()
