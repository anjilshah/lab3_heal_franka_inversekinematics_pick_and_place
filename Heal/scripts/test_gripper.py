#!/usr/bin/env python3
"""Headless Robotiq open/close travel and driver symmetry test."""
import mujoco
from robot_common import load_model

model, data, *_ = load_model()
left = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'left_driver_joint')
right = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'right_driver_joint')
act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper')
positions = []
for command in (0.0, 255.0):
    data.ctrl[act] = command
    for _ in range(int(1.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    left_q = data.qpos[model.jnt_qposadr[left]]
    right_q = data.qpos[model.jnt_qposadr[right]]
    positions.append((left_q, right_q))
    error = abs(left_q-right_q)
    print(f'command={command:.0f} left={left_q:.4f} right={right_q:.4f} symmetry_error={error:.6f}')
    assert error < 0.08, 'Driver equality did not maintain finger symmetry'
assert positions[1][0] > positions[0][0] + 0.3, 'Close command did not move the fingers from open to closed'
print('Gripper open/close travel test passed.')
