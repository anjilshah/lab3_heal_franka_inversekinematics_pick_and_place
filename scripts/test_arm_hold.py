#!/usr/bin/env python3
"""Numerically test three-second gravity-compensated hold of the HEAL arm."""
import numpy as np
import mujoco
from robot_common import load_model

model, data, qpos_ids, dof_ids, actuator_ids, *_ = load_model()
initial = data.qpos[qpos_ids].copy()
kp, kd = 500.0, 50.0
for _ in range(int(3.0 / model.opt.timestep)):
    mujoco.mj_forward(model, data)
    for i, (aid, dof, qpos) in enumerate(zip(actuator_ids, dof_ids, qpos_ids)):
        data.ctrl[aid] = data.qfrc_bias[dof] + kp * (initial[i] - data.qpos[qpos]) - kd * data.qvel[dof]
    mujoco.mj_step(model, data)
final = data.qpos[qpos_ids].copy()
error = np.max(np.abs(final-initial))
print('Arm hold test complete. Joint positions (rad):')
for name, value in zip(('joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'), final):
    print(f'  {name}: {value: .4f}')
print(f'Maximum hold error: {error:.6f} rad')
assert np.all(np.isfinite(final)) and error < 0.02, 'Arm did not hold its initial pose within 0.02 rad'
