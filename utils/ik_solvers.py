"""Comparable task-space IK solvers for the HEAL MuJoCo model.

The original DLS implementation in scripts/pick_and_place.py is preserved.
This module exposes a compatible, instrumented DLS copy plus Mink and a direct
qpsolvers QP method. All methods solve only the six HEAL arm joints; other
model DOFs (gripper and free cube) are held fixed.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import time

import mujoco
import numpy as np

@dataclass
class IKResult:
    method: str
    q: list[float]
    status: str
    position_error_m: float
    orientation_error_rad: float
    ik_iterations: int | None
    ik_iteration_definition: str
    backend_iterations: int | None
    solve_time_s: float
    joint_position_limit_violation_rad: float
    velocity_limit_violation_rad_s: float | None
    velocity_limit_rad_s: float | None
    velocity_limit_enforcement: str
    solve_time_scope: str
    message: str = ""

    def to_dict(self):
        return asdict(self)


def _rotation_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    R = target @ current.T
    cosine = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    angle = float(np.arccos(cosine))
    vee = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if angle < 1e-6:
        return 0.5 * vee
    if np.pi - angle < 1e-5:
        vals, vecs = np.linalg.eig(R)
        axis = np.real(vecs[:, np.argmin(np.abs(vals - 1.0))])
        return angle * axis / max(np.linalg.norm(axis), 1e-12)
    return angle * vee / (2.0 * np.sin(angle))


def _joint_ranges(model, arm_joint_ids):
    return np.asarray([model.jnt_range[j] for j in arm_joint_ids], dtype=float)


def _residual(model, data, qpos_ids, site_id, target, target_rotation):
    mujoco.mj_forward(model, data)
    pos = float(np.linalg.norm(target - data.site_xpos[site_id]))
    rot = float(np.linalg.norm(_rotation_error(target_rotation, data.site_xmat[site_id].reshape(3, 3))))
    return pos, rot


def _result(method, q, status, pos, rot, iterations, elapsed, ranges,
            iteration_definition, velocity_limit=None, initial_q=None,
            velocity_violation=None, joint_violation=None, message=""):
    q = np.asarray(q, dtype=float)
    if joint_violation is None:
        joint_violation = float(max(0.0, np.max(ranges[:, 0] - q), np.max(q - ranges[:, 1])))
    if velocity_limit is None:
        velocity_violation = None
    enforcement = "not_enforced" if velocity_limit is None else "hard_constraint"
    return IKResult(method, q.tolist(), status, pos, rot, iterations,
                    iteration_definition, None, elapsed, joint_violation,
                    velocity_violation, velocity_limit, enforcement,
                    "wall_clock_for_one_complete_iterative_ik_call", message)


def solve_dls(model, data, qpos_ids, dof_ids, site_id, target, target_rotation=None,
              max_iterations=180, damping=0.08):
    """Preserved DLS update law, instrumented without changing the original script."""
    started = time.perf_counter()
    saved_qpos, saved_qvel = data.qpos.copy(), data.qvel.copy()
    if target_rotation is None:
        target_rotation = data.site_xmat[site_id].reshape(3, 3).copy()
    target_rotation = np.asarray(target_rotation).reshape(3, 3)
    joint_ids = [int(model.dof_jntid[d]) for d in dof_ids]
    ranges = _joint_ranges(model, joint_ids)
    initial = data.qpos[qpos_ids].copy()
    seed = initial.copy()
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    rot_weight = 0.25
    iterations = 0
    max_joint_violation = 0.0
    for iterations in range(1, max_iterations + 1):
        mujoco.mj_forward(model, data)
        pos_err = target - data.site_xpos[site_id]
        rot_err = _rotation_error(target_rotation, data.site_xmat[site_id].reshape(3, 3))
        if np.linalg.norm(pos_err) < 0.004 and np.linalg.norm(rot_err) < 0.025:
            break
        mujoco.mj_jacSite(model, data, jp, jr, site_id)
        J = np.vstack((jp[:, dof_ids], rot_weight * jr[:, dof_ids]))
        err = np.r_[pos_err, rot_weight * rot_err]
        dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), err)
        seed += np.clip(dq, -0.035, 0.035)
        seed = np.clip(seed, ranges[:, 0], ranges[:, 1])
        max_joint_violation = max(max_joint_violation,
                                  float(max(0.0, np.max(ranges[:, 0]-seed), np.max(seed-ranges[:, 1]))))
        data.qpos[qpos_ids] = seed
    pos, rot = _residual(model, data, qpos_ids, site_id, target, target_rotation)
    data.qpos[:], data.qvel[:] = saved_qpos, saved_qvel
    mujoco.mj_forward(model, data)
    elapsed = time.perf_counter() - started
    status = "converged" if pos < 0.004 and rot < 0.025 else "max_iterations"
    return _result("dls", seed, status, pos, rot, iterations, elapsed, ranges,
                   "DLS Jacobian update iterations",
                   initial_q=initial, joint_violation=max_joint_violation)


def _mink_components(model, data, dof_ids, target, target_rotation):
    import mink

    configuration = mink.Configuration(model, q=data.qpos.copy())
    task = mink.FrameTask("right_center", "site", position_cost=1.0,
                          orientation_cost=0.25, gain=0.8)
    rotation = mink.SO3.from_matrix(np.asarray(target_rotation).reshape(3, 3))
    task.set_target(mink.SE3.from_rotation_and_translation(rotation, np.asarray(target, dtype=float)))
    frozen = sorted(set(range(model.nv)) - set(map(int, dof_ids)))
    constraints = [mink.DofFreezingTask(model, dof_indices=frozen)] if frozen else []
    limits = [mink.ConfigurationLimit(model),
              mink.VelocityLimit(model, {f"joint_{i}": 1.2 for i in range(1, 7)})]
    return mink, configuration, task, constraints, limits


def solve_mink(model, data, qpos_ids, dof_ids, site_id, target, target_rotation=None,
               max_iterations=180, dt=0.02, velocity_limit=1.2):
    """Mink FrameTask differential IK with hard configuration and velocity limits."""
    import mink

    started = time.perf_counter()
    if target_rotation is None:
        target_rotation = data.site_xmat[site_id].reshape(3, 3).copy()
    initial = data.qpos[qpos_ids].copy()
    joint_ids = [int(model.dof_jntid[d]) for d in dof_ids]
    ranges = _joint_ranges(model, joint_ids)
    config, task = None, None
    iterations = 0
    message = ""
    max_velocity_violation = 0.0
    max_joint_violation = 0.0
    try:
        _, config, task, constraints, limits = _mink_components(model, data, dof_ids, target, target_rotation)
        # Use the requested explicit cap for this run (radians/second per joint).
        limits[1] = mink.VelocityLimit(model, {f"joint_{i}": velocity_limit for i in range(1, 7)})
        for iterations in range(1, max_iterations + 1):
            vel = mink.solve_ik(config, [task], dt, "daqp", damping=1e-4,
                                limits=limits, constraints=constraints)
            max_velocity_violation = max(
                max_velocity_violation,
                float(max(0.0, np.max(np.abs(vel[dof_ids]) - velocity_limit))),
            )
            config.integrate_inplace(vel, dt)
            arm_q = config.q[qpos_ids]
            max_joint_violation = max(
                max_joint_violation,
                float(max(0.0, np.max(ranges[:, 0]-arm_q), np.max(arm_q-ranges[:, 1]))),
            )
            err = task.compute_error(config)
            if np.linalg.norm(err[:3]) < 0.004 and np.linalg.norm(err[3:]) < 0.025:
                message = "Mink/DAQP: velocity solution returned; terminated on pose residual."
                break
            message = "Mink/DAQP: velocity solution returned; iteration limit not yet met."
        q = config.q[qpos_ids]
        # Evaluate the same MuJoCo model/data residual used for all methods.
        saved = data.qpos.copy()
        data.qpos[:] = config.q
        pos, rot = _residual(model, data, qpos_ids, site_id, target, target_rotation)
        data.qpos[:] = saved
        mujoco.mj_forward(model, data)
        status = "converged" if pos < 0.004 and rot < 0.025 else "max_iterations"
    except Exception as exc:
        q = initial
        pos, rot = float("inf"), float("inf")
        status, message = "solver_error", f"{type(exc).__name__}: {exc}"
    elapsed = time.perf_counter() - started
    return _result("mink", q, status, pos, rot, iterations, elapsed, ranges,
                   "outer differential-IK calls to mink.solve_ik",
                   velocity_limit=velocity_limit, initial_q=initial,
                   velocity_violation=max_velocity_violation,
                   joint_violation=max_joint_violation, message=message)


def solve_qp(model, data, qpos_ids, dof_ids, site_id, target, target_rotation=None,
             max_iterations=180, dt=0.02, velocity_limit=1.2, damping=1e-4):
    """Task-space least-squares QP with hard next-step joint and velocity bounds.

    Per step, solves min_v 1/2 ||W(Jv - gain*e/dt)||^2 + damping/2 ||v||^2,
    subject to joint-position bounds on q + v*dt and |v_i| <= 1.2 rad/s.
    The velocity cap is a documented simulation setting, not a measured HEAL
    hardware specification.
    """
    import qpsolvers

    started = time.perf_counter()
    saved_qpos, saved_qvel = data.qpos.copy(), data.qvel.copy()
    if target_rotation is None:
        target_rotation = data.site_xmat[site_id].reshape(3, 3).copy()
    target_rotation = np.asarray(target_rotation).reshape(3, 3)
    joint_ids = [int(model.dof_jntid[d]) for d in dof_ids]
    ranges = _joint_ranges(model, joint_ids)
    initial = data.qpos[qpos_ids].copy()
    q = initial.copy()
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    W = np.diag([1., 1., 1., .25, .25, .25])
    status, message, iterations = "max_iterations", "", 0
    max_velocity_violation = 0.0
    max_joint_violation = 0.0
    for iterations in range(1, max_iterations + 1):
        data.qpos[qpos_ids] = q
        mujoco.mj_forward(model, data)
        error = np.r_[target - data.site_xpos[site_id],
                      _rotation_error(target_rotation, data.site_xmat[site_id].reshape(3, 3))]
        if np.linalg.norm(error[:3]) < 0.004 and np.linalg.norm(error[3:]) < 0.025:
            status = "converged"
            break
        mujoco.mj_jacSite(model, data, jp, jr, site_id)
        J = np.vstack((jp[:, dof_ids], jr[:, dof_ids]))
        A = W @ J
        b = W @ (0.8 * error / dt)
        P = A.T @ A + damping * np.eye(len(dof_ids))
        c = -(A.T @ b)
        lb = np.maximum(-velocity_limit, (ranges[:, 0] - q) / dt)
        ub = np.minimum(velocity_limit, (ranges[:, 1] - q) / dt)
        if np.any(lb > ub + 1e-10):
            status, message = "infeasible", "Joint-position and velocity bounds have no feasible next step."
            break
        try:
            sol = qpsolvers.solve_problem(qpsolvers.Problem(P, c, lb=lb, ub=ub), solver="daqp")
            if not sol.found or sol.x is None:
                status, message = "infeasible", f"DAQP found={sol.found}; extras={sol.extras}"
                break
            message = f"DAQP found={sol.found}; solve_time_s={sol.solve_time}"
            max_velocity_violation = max(
                max_velocity_violation,
                float(max(0.0, np.max(np.abs(sol.x) - velocity_limit))),
            )
            q += np.asarray(sol.x) * dt
            max_joint_violation = max(
                max_joint_violation,
                float(max(0.0, np.max(ranges[:, 0]-q), np.max(q-ranges[:, 1]))),
            )
        except Exception as exc:
            status, message = "solver_error", f"{type(exc).__name__}: {exc}"
            break
    data.qpos[qpos_ids] = q
    pos, rot = _residual(model, data, qpos_ids, site_id, target, target_rotation)
    data.qpos[:], data.qvel[:] = saved_qpos, saved_qvel
    mujoco.mj_forward(model, data)
    if status == "max_iterations" and pos < 0.004 and rot < 0.025:
        status = "converged"
    elapsed = time.perf_counter() - started
    return _result("qp_daqp", q, status, pos, rot, iterations, elapsed, ranges,
                   "outer Cartesian-IK iterations / DAQP QP calls",
                   velocity_limit=velocity_limit, initial_q=initial,
                   velocity_violation=max_velocity_violation,
                   joint_violation=max_joint_violation, message=message)


SOLVERS = {"dls": solve_dls, "mink": solve_mink, "qp": solve_qp}
