"""Frank-specific MuJoCo model, IK solvers, and pick/place experiment."""
from __future__ import annotations

import json
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "scene.xml"
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))
ACTUATOR_NAMES = tuple(f"actuator{i}" for i in range(1, 8))
GRIPPER_ACTUATOR = "actuator8"
EE_SITE = "ee_site"
HOME = np.array([0.0, -0.5, 0.0, -2.2, 0.0, 1.7, 0.8])
PICK_XY = np.array([0.18, -0.16])
PLACE_XY = np.array([0.18, 0.16])
TABLE_TOP = 0.375
CUBE_HALF = 0.025
SITE_TO_PAD = 0.003  # ee_site is colocated with the fingertip pad centers (model geometry)
METHODS = ("dls", "mink", "qp")
FINAL_PLACEMENT_TOLERANCE_M = 0.010
# Solver endpoints must be materially tighter than the task placement tolerance.
IK_POSITION_TOLERANCE_M = 0.002
IK_ORIENTATION_TOLERANCE_RAD = 0.02
IK_TIMESTEP_MULTIPLIER = 8
IK_VELOCITY_LIMIT_RAD_S = 1.2
PATH_CLEARANCE_MARGIN_M = 0.0
COLLISION_REFINEMENT_CLEARANCE_EPSILON_M = 1e-6
# Maximum adverse command-to-state geom clearance loss measured at every step
# near the table in results/retreat_execution_calibration_20261013/summary.json.
# This common allowance is a calibration bound, not a safety certificate.
COLLISION_EXECUTION_ALLOWANCE_M = 0.0012140777853289486
COLLISION_REFINEMENT_POSE_SLACK = 1e-7


def load_model():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    jids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES]
    aids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ACTUATOR_NAMES]
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, EE_SITE)
    grip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, GRIPPER_ACTUATOR)
    if min(jids + aids + [site, grip]) < 0:
        raise ValueError("Panda joint/actuator/site validation failed")
    qpos = np.asarray([model.jnt_qposadr[j] for j in jids], dtype=int)
    dofs = np.asarray([model.jnt_dofadr[j] for j in jids], dtype=int)
    ranges = np.asarray([model.jnt_range[j] for j in jids], dtype=float)
    if model.njnt < 10 or len(qpos) != 7 or len(aids) != 7 or model.actuator_trntype[grip] != mujoco.mjtTrn.mjTRN_TENDON:
        raise ValueError("Expected a 7-DoF Panda, seven arm actuators and tendon gripper actuator")
    return model, data, qpos, dofs, np.asarray(aids), site, grip, ranges


def reset(model, data, indices, cube_xy=PICK_XY, yaw=0.0):
    qpos = indices[0]
    mujoco.mj_resetData(model, data)
    data.qpos[qpos] = HOME
    cube_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_free")
    start = int(model.jnt_qposadr[cube_joint])
    data.qpos[start:start+3] = [float(cube_xy[0]), float(cube_xy[1]), TABLE_TOP + CUBE_HALF]
    data.qpos[start+3:start+7] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
    data.ctrl[:] = 0
    data.ctrl[7] = 255
    mujoco.mj_forward(model, data)


def rot_error(target, current):
    rel = target @ current.T
    cosine = np.clip((np.trace(rel) - 1) / 2, -1, 1)
    angle = float(np.arccos(cosine))
    vee = np.array([rel[2, 1]-rel[1, 2], rel[0, 2]-rel[2, 0], rel[1, 0]-rel[0, 1]])
    if angle < 1e-7:
        return 0.5 * vee
    if np.pi-angle < 1e-5:
        vals, vecs = np.linalg.eig(rel)
        axis = np.real(vecs[:, np.argmin(np.abs(vals-1))])
        return angle * axis / max(np.linalg.norm(axis), 1e-12)
    return angle * vee / (2*np.sin(angle))


def _pose_error(model, data, site, xyz, rotation):
    mujoco.mj_forward(model, data)
    return xyz-data.site_xpos[site].copy(), rot_error(rotation, data.site_xmat[site].reshape(3, 3).copy())


def refine_collision_endpoint(model, data, indices, q_start, q_candidate,
                              xyz, rotation, collision_limit):
    """Refine an IK endpoint with a calibrated allowance along its joint path."""
    from scipy.optimize import minimize

    qpos, _, _, site, _, ranges = indices
    q_candidate = np.asarray(q_candidate, dtype=float)
    q_start = np.asarray(q_start, dtype=float)
    geom_pairs = collision_limit.geom_id_pairs
    clearance_budget = (COLLISION_EXECUTION_ALLOWANCE_M
                        + COLLISION_REFINEMENT_CLEARANCE_EPSILON_M)
    fractions = np.linspace(0.0, 1.0, 25)

    def evaluate(q):
        data.qpos[qpos] = q
        mujoco.mj_forward(model, data)
        position_error = np.asarray(xyz) - data.site_xpos[site].copy()
        orientation_error = rot_error(
            rotation, data.site_xmat[site].reshape(3, 3).copy())
        distances = np.asarray([
            mujoco.mj_geomDistance(model, data, int(a), int(b), 10.0, None)
            for a, b in geom_pairs
        ], dtype=float)
        return position_error, orientation_error, distances

    def path_distances(q, pair_indices=None):
        pairs = geom_pairs if pair_indices is None else [geom_pairs[i] for i in pair_indices]
        profile = []
        for u in fractions:
            data.qpos[qpos] = q_start * (1.0 - u) + q * u
            mujoco.mj_forward(model, data)
            profile.append([mujoco.mj_geomDistance(
                model, data, int(a), int(b), 10.0, None) for a, b in pairs])
        return np.asarray(profile, dtype=float)

    _, _, initial_distances = evaluate(q_candidate)
    initial_profile = path_distances(q_candidate)
    initial_clearance = float(np.min(initial_distances))
    initial_path_clearance = float(np.min(initial_profile))
    active_pairs = np.flatnonzero(np.min(initial_profile, axis=0) < clearance_budget)
    if not len(active_pairs):
        return q_candidate, {"attempted": False,
                             "minimum_clearance_before_m": initial_clearance,
                             "minimum_path_clearance_before_m": initial_path_clearance,
                             "execution_allowance_m": COLLISION_EXECUTION_ALLOWANCE_M}
    if np.min(initial_profile[0]) < 0.0:
        return q_candidate, {"attempted": False, "success": False,
                             "reason": "actual_start_configuration_penetrates",
                             "minimum_path_clearance_before_m": initial_path_clearance}
    # A safe actual start may have less clearance than the calibrated budget.
    # Require clearance to grow toward the budget rather than changing that start.
    start_allowance = np.minimum(initial_profile[0], clearance_budget)
    required_profile = ((1.0 - fractions[:, None]) * start_allowance
                        + fractions[:, None] * clearance_budget)

    def constraints(q):
        position_error, orientation_error, distances = evaluate(q)
        return np.r_[
            IK_POSITION_TOLERANCE_M - COLLISION_REFINEMENT_POSE_SLACK
            - np.linalg.norm(position_error),
            IK_ORIENTATION_TOLERANCE_RAD - COLLISION_REFINEMENT_POSE_SLACK
            - np.linalg.norm(orientation_error),
            distances - clearance_budget,
            (path_distances(q, active_pairs)
             - required_profile[:, active_pairs]).reshape(-1),
        ]

    result = minimize(
        lambda q: 0.5 * float(np.dot(q - q_candidate, q - q_candidate)),
        q_candidate,
        method="SLSQP",
        bounds=list(map(tuple, ranges)),
        constraints=[{"type": "ineq", "fun": constraints}],
        options={"maxiter": 250, "ftol": 1e-12},
    )
    refined = np.asarray(result.x, dtype=float)
    position_error, orientation_error, distances = evaluate(refined)
    minimum_clearance = float(np.min(distances))
    final_profile = path_distances(refined)
    final_path_clearance = float(np.min(final_profile))
    pose_ok = (np.linalg.norm(position_error) < IK_POSITION_TOLERANCE_M
               and np.linalg.norm(orientation_error) < IK_ORIENTATION_TOLERANCE_RAD)
    limits_ok = bool(np.all(refined >= ranges[:, 0]) and np.all(refined <= ranges[:, 1]))
    feasible = bool(result.success and pose_ok and limits_ok
                    and final_path_clearance >= PATH_CLEARANCE_MARGIN_M
                    and np.min(final_profile - required_profile) >= -1e-10)
    diagnostics = {
        "attempted": True,
        "success": feasible,
        "optimizer_message": str(result.message),
        "iterations": int(result.nit),
        "minimum_clearance_before_m": initial_clearance,
        "minimum_clearance_after_m": minimum_clearance,
        "minimum_path_clearance_before_m": initial_path_clearance,
        "minimum_path_clearance_after_m": final_path_clearance,
        "execution_allowance_m": COLLISION_EXECUTION_ALLOWANCE_M,
        "clearance_budget_m": clearance_budget,
        "position_residual_after_m": float(np.linalg.norm(position_error)),
        "orientation_residual_after_rad": float(np.linalg.norm(orientation_error)),
        "joint_displacement_norm_rad": float(np.linalg.norm(refined - q_candidate)),
    }
    return (refined if feasible else q_candidate), diagnostics


def solve(method, model, data, indices, xyz, rotation=None,
          velocity_limit=IK_VELOCITY_LIMIT_RAD_S, max_iterations=120):
    qpos, dofs, _, site, _, ranges = indices
    saved_qpos, saved_qvel = data.qpos.copy(), data.qvel.copy()
    if rotation is None:
        mujoco.mj_forward(model, data)
        rotation = data.site_xmat[site].reshape(3, 3).copy()
    rotation = np.asarray(rotation).reshape(3, 3)
    q = data.qpos[qpos].copy()
    started = time.perf_counter()
    status = "max_iterations"
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    backend_iterations = None
    # All methods use the same final path refinement. DLS retains its own IK
    # update; only Mink/QP include the velocity inequalities inside their QPs.
    from collision import mink_table_floor_limit
    collision_limit = mink_table_floor_limit(model)
    for it in range(1, max_iterations+1):
        data.qpos[qpos] = q
        pos_e, rot_e = _pose_error(model, data, site, np.asarray(xyz), rotation)
        if (np.linalg.norm(pos_e) < IK_POSITION_TOLERANCE_M
                and np.linalg.norm(rot_e) < IK_ORIENTATION_TOLERANCE_RAD):
            status = "converged"
            break
        mujoco.mj_jacSite(model, data, jp, jr, site)
        J = np.vstack([jp[:, dofs], 0.2*jr[:, dofs]])
        e = np.r_[pos_e, 0.2*rot_e]
        if method == "dls":
            dq = J.T @ np.linalg.solve(J@J.T + 0.05**2*np.eye(6), e)
        elif method == "mink":
            import mink
            conf = mink.Configuration(model, q=data.qpos.copy())
            task = mink.FrameTask(EE_SITE, "site", position_cost=1.0,
                                  orientation_cost=0.2, gain=1.0)
            target = mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(rotation), np.asarray(xyz))
            task.set_target(target)
            limits = [mink.ConfigurationLimit(model),
                      mink.VelocityLimit(model, {name: float(velocity_limit) for name in JOINT_NAMES}),
                      collision_limit]
            try:
                vel = mink.solve_ik(conf, [task],
                                    model.opt.timestep * IK_TIMESTEP_MULTIPLIER,
                                    "daqp", limits=limits,
                                    )
            except mink.exceptions.NoSolutionFound as exc:
                status = "solver_failure"
                break
            dq = vel[dofs] * model.opt.timestep * IK_TIMESTEP_MULTIPLIER
        elif method == "qp":
            from qpsolvers import solve_qp
            # Minimize task residual plus a small motion penalty; joint position and
            # velocity bounds are imposed on the incremental joint update.
            H = J.T@J + 1e-3*np.eye(7)
            f = -(J.T@e)
            dt = model.opt.timestep * IK_TIMESTEP_MULTIPLIER
            lower = np.maximum(ranges[:, 0]-q, -velocity_limit*dt)
            upper = np.minimum(ranges[:, 1]-q, velocity_limit*dt)
            import mink
            conf = mink.Configuration(model, q=data.qpos.copy())
            distance_constraint = collision_limit.compute_qp_inequalities(conf, dt)
            active = (np.isfinite(distance_constraint.h)
                      & (np.linalg.norm(distance_constraint.G[:, dofs], axis=1) > 0.0))
            collision_G = distance_constraint.G[active][:, dofs] / dt
            collision_h = distance_constraint.h[active]
            dq = solve_qp(H, f, G=collision_G, h=collision_h,
                          lb=lower, ub=upper, solver="daqp")
            if dq is None:
                status = "qp_infeasible"
                break
        else:
            raise ValueError(f"Unknown IK method: {method}")
        # Apply the same per-iteration velocity step bound for all solvers.
        max_step = velocity_limit * model.opt.timestep * IK_TIMESTEP_MULTIPLIER
        dq = np.clip(dq, -max_step, max_step)
        q = np.clip(q + dq, ranges[:, 0], ranges[:, 1])
    collision_refinement = None
    if collision_limit is not None:
        q, collision_refinement = refine_collision_endpoint(
            model, data, indices, saved_qpos[qpos], q, xyz, rotation,
            collision_limit)
    data.qpos[qpos] = q
    pe, re = _pose_error(model, data, site, np.asarray(xyz), rotation)
    pos_res, rot_res = float(np.linalg.norm(pe)), float(np.linalg.norm(re))
    data.qpos[:], data.qvel[:] = saved_qpos, saved_qvel
    mujoco.mj_forward(model, data)
    return {"method": method, "q": q.tolist(), "status": status,
            "iterations": it, "backend_iterations": backend_iterations,
            "solve_time_s": time.perf_counter()-started,
            "position_residual_m": pos_res, "orientation_residual_rad": rot_res,
            "joint_limit_violation_rad": float(max(0, np.max(ranges[:, 0]-q), np.max(q-ranges[:, 1]))),
            "velocity_limit_rad_s": float(velocity_limit),
            "max_joint_step_rad": float(velocity_limit * model.opt.timestep * IK_TIMESTEP_MULTIPLIER),
            "joint_position_limits_rad": ranges.tolist(),
            "collision_refinement": collision_refinement}


def _step(model, data, indices, qtarget, grip):
    _, _, arm_acts, _, grip_act, _ = indices
    data.ctrl[arm_acts] = qtarget
    data.ctrl[grip_act] = grip
    mujoco.mj_step(model, data)


def _hand_and_cube_geoms(model):
    hand = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    if hand < 0 or cube < 0:
        raise ValueError("Missing hand body or cube collision geom")
    hand_geoms = set()
    for geom in range(model.ngeom):
        body = int(model.geom_bodyid[geom])
        while body > 0 and body != hand:
            body = int(model.body_parentid[body])
        if body == hand:
            hand_geoms.add(geom)
    return hand_geoms, cube


def _has_hand_cube_contact(data, hand_geoms, cube_geom):
    for index in range(data.ncon):
        a, b = int(data.contact[index].geom1), int(data.contact[index].geom2)
        if (a == cube_geom and b in hand_geoms) or (b == cube_geom and a in hand_geoms):
            return True
    return False


class EpisodeTelemetry:
    """Measurements collected from actual MuJoCo states during one episode."""

    def __init__(self, model, data, indices, initial_cube_xyz, collision_audit=None):
        self.model = model
        self.data = data
        self.indices = indices
        self.hand_geoms, self.cube_geom = _hand_and_cube_geoms(model)
        self.cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        self.initial_cube_xyz = np.asarray(initial_cube_xyz, dtype=float).copy()
        self.collision_audit = collision_audit
        self.cube_trajectory = []
        self.phase_execution = []
        self.current_phase_record = None
        self.grasp_acquired = False
        self.grasp_acquired_sim_time_s = None
        self.carry_contact_steps = 0
        self.carry_steps = 0
        self.lift_contact_observed = False
        self.release_phase_reached = False
        self.cube_xyz_at_release_start = None
        self.cube_xyz_at_release_end = None
        self.release_hand_contact_steps = 0
        self.joint_limit_events = []
        self._active_limit_events = set()

    def begin_phase(self, phase, target_xyz, grip):
        if self.current_phase_record is not None:
            self.end_phase()
        cube = self.data.xpos[self.cube_body].copy()
        site = self.indices[3]
        record = {"phase": phase, "target_xyz": np.asarray(target_xyz).tolist(),
                  "grip_command": float(grip), "start_sim_time_s": float(self.data.time),
                  "cube_start_xyz": cube.tolist(),
                  "ee_start_xyz": self.data.site_xpos[site].copy().tolist(),
                  "hand_cube_contact_steps": 0, "steps": 0}
        self.current_phase_record = record
        self.phase_execution.append(record)
        if phase == "release":
            self.release_phase_reached = True
            self.cube_xyz_at_release_start = cube.tolist()

    def observe(self, phase, grip):
        cube = self.data.xpos[self.cube_body].copy()
        site = self.indices[3]
        ee = self.data.site_xpos[site].copy()
        contact = _has_hand_cube_contact(self.data, self.hand_geoms, self.cube_geom)
        if self.current_phase_record is None or self.current_phase_record["phase"] != phase:
            raise RuntimeError(f"Telemetry phase mismatch: {phase}")
        phase_record = self.current_phase_record
        phase_record["steps"] += 1
        phase_record["hand_cube_contact_steps"] += int(contact)
        if contact and grip < 128:
            self.grasp_acquired = True
            if self.grasp_acquired_sim_time_s is None:
                self.grasp_acquired_sim_time_s = float(self.data.time)
        if phase in ("lift", "transfer", "lower"):
            self.carry_steps += 1
            self.carry_contact_steps += int(contact)
            if (phase == "lift" and contact
                    and cube[2] > self.initial_cube_xyz[2] + 0.045):
                self.lift_contact_observed = True
        if phase == "release":
            self.release_hand_contact_steps += int(contact)
            self.cube_xyz_at_release_end = cube.tolist()

        self.cube_trajectory.append({
            "sim_time_s": float(self.data.time), "phase": phase,
            "cube_xyz": cube.tolist(), "ee_xyz": ee.tolist(),
            "hand_cube_contact": bool(contact), "grip_command": float(grip),
        })

        q = self.data.qpos[self.indices[0]]
        ranges = self.indices[-1]
        current_limits = set()
        for joint_index, (value, bounds) in enumerate(zip(q, ranges)):
            for side, bound in (("lower", bounds[0]), ("upper", bounds[1])):
                if abs(float(value) - float(bound)) <= 0.005:
                    key = (joint_index, side)
                    current_limits.add(key)
                    if key not in self._active_limit_events:
                        self.joint_limit_events.append({
                            "sim_time_s": float(self.data.time),
                            "joint": JOINT_NAMES[joint_index], "side": side,
                            "q_rad": float(value), "limit_rad": float(bound),
                            "threshold_rad": 0.005,
                        })
        self._active_limit_events = current_limits
        if self.collision_audit is not None:
            self.collision_audit.update(self.data, phase)

    def end_phase(self, target_xyz=None, target_rotation=None):
        record = self.current_phase_record
        if record is None:
            return
        cube = self.data.xpos[self.cube_body].copy()
        site_id = self.indices[3]
        ee = self.data.site_xpos[site_id].copy()
        record["end_sim_time_s"] = float(self.data.time)
        record["cube_end_xyz"] = cube.tolist()
        record["ee_end_xyz"] = ee.tolist()
        if target_xyz is not None:
            record["ee_position_tracking_error_m"] = float(
                np.linalg.norm(np.asarray(target_xyz, dtype=float) - ee))
        if target_rotation is not None:
            actual_rotation = self.data.site_xmat[site_id].reshape(3, 3).copy()
            record["ee_orientation_tracking_error_rad"] = float(
                np.linalg.norm(rot_error(target_rotation, actual_rotation)))
        if record["phase"] == "release":
            self.cube_xyz_at_release_end = cube.tolist()
        self.current_phase_record = None

    def summary(self, minimum_clearance_m=None):
        if self.current_phase_record is not None:
            self.end_phase()
        carry_ratio = (self.carry_contact_steps / self.carry_steps
                       if self.carry_steps else None)
        return {
            "cube_trajectory": self.cube_trajectory,
            "phase_execution": self.phase_execution,
            "grasp_diagnostics": {
                "hand_cube_contact_acquired_while_closing": self.grasp_acquired,
                "first_contact_sim_time_s": self.grasp_acquired_sim_time_s,
                "lift_contact_observed_above_45mm": self.lift_contact_observed,
                "carry_contact_steps": self.carry_contact_steps,
                "carry_steps": self.carry_steps,
                "carry_contact_fraction": carry_ratio,
            },
            "release_diagnostics": {
                "release_phase_reached": self.release_phase_reached,
                "cube_xyz_at_release_start": self.cube_xyz_at_release_start,
                "cube_xyz_at_release_end": self.cube_xyz_at_release_end,
                "hand_cube_contact_steps_while_opening": self.release_hand_contact_steps,
            },
            "joint_limit_events": self.joint_limit_events,
            "collision_diagnostics": (self.collision_audit.to_dict()
                                      if self.collision_audit is not None else None),
            "minimum_sampled_clearance_m": minimum_clearance_m,
        }


def make_path_guard(model, data, indices, samples=25,
                    clearance_margin_m=PATH_CLEARANCE_MARGIN_M):
    """Reject sampled joint paths with actual table/floor penetration."""
    from collision import sampled_segment_clearance

    qpos_ids = indices[0]

    def guard(q0, q1):
        report = sampled_segment_clearance(model, data, qpos_ids, q0, q1, samples=samples)
        return (report["minimum_sampled_clearance_m"] >= clearance_margin_m,
                {**report, "required_clearance_m": clearance_margin_m})

    return guard


def collision_audit_has_penetration(audit):
    """Expected cube support/grasp contact does not count as robot penetration."""
    minima = audit.get("minimum_signed_clearance_m_by_category", {})
    return any(minima.get(category, 0.0) < 0.0
               for category in ("robot-floor", "robot-table", "arm-cube"))


def run_episode(method, model, data, indices, seed=0, cube_xy=None, place_xy=None, max_phase_s=2.0, collision_audit=None, viewer=None, path_guard=None, frame_callback=None):
    wall_started = time.perf_counter()
    rng = np.random.default_rng(seed)
    cube_xy = np.asarray(PICK_XY if cube_xy is None else cube_xy, dtype=float)
    place_xy = np.asarray(PLACE_XY if place_xy is None else place_xy, dtype=float)
    yaw = float(rng.uniform(-0.12, 0.12))
    reset(model, data, indices, cube_xy, yaw)
    qpos, dofs, acts, site, grip_act, _ = indices
    initial = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")].copy()
    rotation = data.site_xmat[site].reshape(3, 3).copy()
    pad_z = TABLE_TOP + CUBE_HALF
    # The measured site is above the fingertips; move the site over the cube center.
    targets = [(cube_xy[0], cube_xy[1], pad_z + SITE_TO_PAD + 0.12, 255, "approach"),
               (cube_xy[0], cube_xy[1], pad_z + SITE_TO_PAD, 255, "descend"),
               (cube_xy[0], cube_xy[1], pad_z + SITE_TO_PAD, 0, "grasp"),
               (cube_xy[0], cube_xy[1], pad_z + SITE_TO_PAD + 0.16, 0, "lift"),
               (place_xy[0], place_xy[1], pad_z + SITE_TO_PAD + 0.16, 0, "transfer"),
               (place_xy[0], place_xy[1], pad_z + SITE_TO_PAD, 0, "lower"),
               (place_xy[0], place_xy[1], pad_z + SITE_TO_PAD, 255, "release"),
               (place_xy[0], place_xy[1], pad_z + SITE_TO_PAD + 0.12, 255, "retreat")]
    if collision_audit is None:
        from collision import CollisionAudit
        collision_audit = CollisionAudit(model, stride=25)
    telemetry = EpisodeTelemetry(model, data, indices, initial, collision_audit)
    if path_guard is None:
        path_guard = make_path_guard(model, data, indices)
    records=[]; path_checks=[];ik_time=0.; max_pos=0.; max_rot=0.; failure=None; peak=initial[2]; frame_index=0
    for x,y,z,grip,phase in targets:
        target=np.array([x,y,z], dtype=float)
        telemetry.begin_phase(phase, target, grip)
        result=solve(method, model, data, indices, target, rotation)
        records.append({**result, "phase": phase, "target_xyz": target.tolist()})
        ik_time += result["solve_time_s"]
        max_pos=max(max_pos,result["position_residual_m"]); max_rot=max(max_rot,result["orientation_residual_rad"])
        if result["status"] != "converged":
            failure=f"ik_{phase}_{result['status']}"
            telemetry.end_phase(target, rotation)
            break
        if path_guard is not None:
            safe,check=path_guard(data.qpos[qpos].copy(),np.asarray(result["q"]))
            path_checks.append({'phase':phase,**check,'accepted':bool(safe)})
            if not safe:
                failure=f"sampled_path_rejected_{phase}"
                telemetry.end_phase(target, rotation)
                break
        steps=max(1,int(max_phase_s/model.opt.timestep))
        start=data.qpos[qpos].copy(); end=np.asarray(result["q"])
        for k in range(steps):
            u=(k+1)/steps; blend=u*u*(3-2*u)
            _step(model,data,indices,start+(end-start)*blend,grip)
            telemetry.observe(phase, grip)
            if frame_callback is not None:
                frame_callback(model, data, phase, frame_index)
            frame_index += 1
            if viewer is not None:
                if not viewer.is_running():
                    failure="viewer_closed"
                    break
                viewer.sync()
                time.sleep(model.opt.timestep)
            peak=max(peak,float(data.xpos[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"cube"),2]))
        telemetry.end_phase(target, rotation)
        if failure == "viewer_closed":
            break
    body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"cube")
    final=data.xpos[body].copy()
    error=float(np.linalg.norm(final[:2]-place_xy))
    success=bool(failure is None and peak > initial[2]+0.045 and error < FINAL_PLACEMENT_TOLERANCE_M and 0.39 < final[2] < 0.49)
    if failure is None and not success: failure="placement_tolerance_or_lost_grasp"
    clearance = collision_audit.to_dict()
    if collision_audit_has_penetration(clearance):
        success = False
        if failure is None:
            failure = "unexpected_collision"
    unexpected_categories = ("robot-floor", "robot-table", "arm-cube")
    measured_clearances = [float(clearance["minimum_signed_clearance_m_by_category"][key])
                           for key in unexpected_categories
                           if key in clearance["minimum_signed_clearance_m_by_category"]
                           and np.isfinite(clearance["minimum_signed_clearance_m_by_category"][key])]
    path_clearances = [float(check["minimum_sampled_clearance_m"])
                       for check in path_checks
                       if np.isfinite(check["minimum_sampled_clearance_m"])]
    all_clearances = measured_clearances + path_clearances
    min_sampled_clearance = min(all_clearances) if all_clearances else None
    telemetry_data = telemetry.summary(min_sampled_clearance)
    return {"seed":int(seed),"method":method,"success":success,"failure_reason":failure,
            "initial_cube_xyz":initial.tolist(),"final_cube_xyz":final.tolist(),"place_target_xy":place_xy.tolist(),
            "target_error_m":error,"final_xy_error_m":error,"final_cube_z_m":float(final[2]),
            "lift_peak_z_m":peak,"sim_time_s":float(data.time),
            "episode_wall_time_s":time.perf_counter()-wall_started,
            "ik_solve_time_total_s":ik_time,"max_position_residual_m":max_pos,
            "max_orientation_residual_rad":max_rot,"solver_status":records[-1]["status"] if records else "none",
            "waypoints":records,"path_checks":path_checks,
            "task_success_checks":{"final_xy_error_lt_0_010_m":bool(error < FINAL_PLACEMENT_TOLERANCE_M),
                                   "lift_peak_gt_initial_z_plus_0_045_m":bool(peak > initial[2] + 0.045),
                                   "final_cube_z_in_open_0_39_0_49_m":bool(0.39 < final[2] < 0.49)},
            "placement_attempted":telemetry.release_phase_reached,
            **telemetry_data,
            "episode_specs":{"cube_xy":cube_xy.tolist(),"place_xy":place_xy.tolist(),"yaw_rad":yaw}}


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")
