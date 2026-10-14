#!/usr/bin/env python3
"""Demonstrate joint velocity control on the project HEAL arm."""
import argparse
import time
import sys
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
from robot_common import MODEL_PATH, arm_indices, load_model
from utils.mj_velocity_control.mj_velocity_ctrl import JointVelocityController


def build_controller(model, data):
    _, _, _, _, grip_id = arm_indices(model)
    controller = JointVelocityController(model, data, kd=2.0, gripper_ids={grip_id})
    targets = np.zeros(model.nu)
    targets[0] = 0.2
    targets[2] = -0.15
    controller.set_velocity_target(targets)
    return controller


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='run a short headless control smoke check')
    args = parser.parse_args()
    model, data, *_ = load_model()
    controller = build_controller(model, data)
    if args.check:
        for _ in range(int(0.2/model.opt.timestep)):
            mujoco.mj_forward(model, data)
            controller.control_callback(model, data)
            mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)):
            raise RuntimeError('Velocity controller produced non-finite joint positions')
        print(f'Headless velocity-control check passed: {MODEL_PATH} (t={data.time:.3f} s)')
        return
    print('Close the MuJoCo window or press Ctrl+C to stop.')
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                start = time.time()
                for _ in range(5):
                    mujoco.mj_forward(model, data)
                    controller.control_callback(model, data)
                    mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(max(0, model.opt.timestep*5-(time.time()-start)))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
