"""Finite-path clearance validation and runtime stopping guard for HEAL tasks.

This is a deliberately bounded safety filter, not a general collision planner.
It validates the planned joint interpolation at sampled points and refuses a
segment that violates the configured clearance. Runtime measurements provide a
second stop condition if torque tracking departs from the validated path.
"""
from __future__ import annotations

import mujoco
import numpy as np

from utils.collision_audit import PHASES, CollisionMonitor, _name


class ClearancePlanner:
    def __init__(self, model, data, monitor: CollisionMonitor, arm_qpos_ids,
                 site_id, clearance_margin_m=0.001, samples=25):
        if clearance_margin_m <= 0:
            raise ValueError("clearance_margin_m must be positive")
        if samples < 2:
            raise ValueError("at least two path samples are required")
        self.model = model
        self.data = data
        self.monitor = monitor
        self.arm_qpos_ids = np.asarray(arm_qpos_ids, dtype=int)
        self.site_id = int(site_id)
        self.clearance_margin_m = float(clearance_margin_m)
        self.samples = int(samples)
        self.cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        cube_joint = int(model.body_jntadr[self.cube_body_id])
        self.cube_qpos_adr = int(model.jnt_qposadr[cube_joint])

    def capture_tcp_to_cube(self):
        """Capture the held object's rigid transform relative to the TCP."""
        mujoco.mj_forward(self.model, self.data)
        tcp_R = self.data.site_xmat[self.site_id].reshape(3, 3).copy()
        cube_R = self.data.xmat[self.cube_body_id].reshape(3, 3).copy()
        tcp_p = self.data.site_xpos[self.site_id].copy()
        cube_p = self.data.xpos[self.cube_body_id].copy()
        return {"rotation_tcp_to_cube": (tcp_R.T @ cube_R).tolist(),
                "translation_tcp_to_cube_m": (tcp_R.T @ (cube_p-tcp_p)).tolist()}

    def _place_attached_cube(self, transform):
        mujoco.mj_forward(self.model, self.data)
        tcp_R = self.data.site_xmat[self.site_id].reshape(3, 3)
        tcp_p = self.data.site_xpos[self.site_id]
        rel_R = np.asarray(transform["rotation_tcp_to_cube"], dtype=float)
        rel_p = np.asarray(transform["translation_tcp_to_cube_m"], dtype=float)
        cube_R = tcp_R @ rel_R
        cube_p = tcp_p + tcp_R @ rel_p
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, np.asarray(cube_R, dtype=float).reshape(9))
        self.data.qpos[self.cube_qpos_adr:self.cube_qpos_adr+3] = cube_p
        self.data.qpos[self.cube_qpos_adr+3:self.cube_qpos_adr+7] = quat

    def validate_segment(self, q_start, q_goal, phase_index, attached_transform=None):
        """Check a smooth joint interpolation without advancing simulation time."""
        phase = PHASES[phase_index]
        q_start = np.asarray(q_start, dtype=float)
        q_goal = np.asarray(q_goal, dtype=float)
        saved_qpos = self.data.qpos.copy()
        saved_qvel = self.data.qvel.copy()
        minimum = float("inf")
        minimum_pair = None
        violating_sample = None
        violating_pair = None
        violating_distance = None
        try:
            for sample_index, u in enumerate(np.linspace(0.0, 1.0, self.samples)):
                blend = u*u*(3.0 - 2.0*u)
                self.data.qpos[self.arm_qpos_ids] = q_start + (q_goal-q_start)*blend
                mujoco.mj_forward(self.model, self.data)
                if attached_transform is not None:
                    self._place_attached_cube(attached_transform)
                    mujoco.mj_forward(self.model, self.data)
                for pair in self.monitor.pairs:
                    if ((attached_transform is not None or phase in {"close_gripper", "release"})
                            and set(pair["roles"]) == {"cube", "gripper"}):
                        # The payload is intentionally nested in the fingers;
                        # this kinematic hold proxy is approximate and can
                        # show mesh overlap despite no simulated contact. The
                        # live MuJoCo contact guard still checks these pairs.
                        continue
                    expected, _ = self.monitor._expected(pair, phase, self.data)
                    if expected:
                        continue
                    g1, g2 = pair["geom_ids"]
                    distance = float(mujoco.mj_geomDistance(
                        self.model, self.data, g1, g2, self.monitor.distmax_m, None))
                    if distance < minimum:
                        minimum = distance
                        names = sorted((_name(self.model, g1), _name(self.model, g2)))
                        minimum_pair = "<->".join(names)
                    if distance < self.clearance_margin_m and violating_sample is None:
                        violating_sample = sample_index
                        names = sorted((_name(self.model, g1), _name(self.model, g2)))
                        violating_pair = "<->".join(names)
                        violating_distance = distance
        finally:
            self.data.qpos[:] = saved_qpos
            self.data.qvel[:] = saved_qvel
            mujoco.mj_forward(self.model, self.data)
        if minimum == float("inf"):
            minimum = None
        return {
            "phase": phase,
            "safe": violating_sample is None,
            "sample_count": self.samples,
            "clearance_margin_m": self.clearance_margin_m,
            "minimum_unexpected_clearance_m": minimum,
            "minimum_pair": minimum_pair,
            "first_violation_sample": violating_sample,
            "first_violation_pair": violating_pair,
            "first_violation_distance_m": violating_distance,
            "attached_cube_approximation_used": attached_transform is not None,
            "grasp_envelope_pairs_excluded": bool(attached_transform is not None
                                                   or phase in {"close_gripper", "release"}),
        }

    def runtime_safe(self):
        distance = self.monitor.last_unexpected_minimum_distance_m
        if distance is None:
            return True, None
        return distance >= self.clearance_margin_m, {
            "minimum_unexpected_clearance_m": distance,
            "pair": self.monitor.last_unexpected_minimum_pair,
            "clearance_margin_m": self.clearance_margin_m,
        }
