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


def solve(method, model, data, indices, xyz, rotation=None, velocity_limit=1.2, max_iterations=120):
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
    for it in range(1, max_iterations+1):
        data.qpos[qpos] = q
        pos_e, rot_e = _pose_error(model, data, site, np.asarray(xyz), rotation)
        if np.linalg.norm(pos_e) < 0.020 and np.linalg.norm(rot_e) < 0.12:
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
            task = mink.FrameTask(EE_SITE, "site", position_cost=1.0, orientation_cost=0.2, gain=0.6)
            target = mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(rotation), np.asarray(xyz))
            task.set_target(target)
            limits = [mink.ConfigurationLimit(model),
                      mink.VelocityLimit(model, {name: float(velocity_limit) for name in JOINT_NAMES})]
            try:
                vel = mink.solve_ik(conf, [task], model.opt.timestep*8, "daqp", limits=limits,
                                    )
            except mink.exceptions.NoSolutionFound as exc:
                status = "solver_failure"
                break
            dq = vel[dofs] * model.opt.timestep*8
        elif method == "qp":
            from qpsolvers import solve_qp
            # Minimize task residual plus a small motion penalty; joint position and
            # velocity bounds are imposed on the incremental joint update.
            H = J.T@J + 1e-3*np.eye(7)
            f = -(J.T@e)
            dt = model.opt.timestep*8
            lower = np.maximum(ranges[:, 0]-q, -velocity_limit*dt)
            upper = np.minimum(ranges[:, 1]-q, velocity_limit*dt)
            dq = solve_qp(H, f, lb=lower, ub=upper, solver="daqp")
            if dq is None:
                status = "qp_infeasible"
                break
        else:
            raise ValueError(f"Unknown IK method: {method}")
        dq = np.clip(dq, -0.12, 0.12)
        q = np.clip(q + dq, ranges[:, 0], ranges[:, 1])
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
            "velocity_limit_rad_s": velocity_limit if method != "dls" else None}


def _step(model, data, indices, qtarget, grip):
    _, _, arm_acts, _, grip_act, _ = indices
    data.ctrl[arm_acts] = qtarget
    data.ctrl[grip_act] = grip
    mujoco.mj_step(model, data)


def run_episode(method, model, data, indices, seed=0, cube_xy=None, place_xy=None, max_phase_s=2.0, collision_audit=None, viewer=None, path_guard=None):
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
    records=[]; path_checks=[];ik_time=0.; max_pos=0.; max_rot=0.; failure=None; peak=initial[2]
    for x,y,z,grip,phase in targets:
        target=np.array([x,y,z], dtype=float)
        result=solve(method, model, data, indices, target, rotation)
        records.append({**result, "phase": phase, "target_xyz": target.tolist()})
        ik_time += result["solve_time_s"]
        max_pos=max(max_pos,result["position_residual_m"]); max_rot=max(max_rot,result["orientation_residual_rad"])
        if result["status"] != "converged":
            failure=f"ik_{phase}_{result['status']}"; break
        if path_guard is not None:
            safe,check=path_guard(data.qpos[qpos].copy(),np.asarray(result["q"]))
            path_checks.append({'phase':phase,**check,'accepted':bool(safe)})
            if not safe:
                failure=f"sampled_path_rejected_{phase}";break
        steps=max(1,int(max_phase_s/model.opt.timestep))
        start=data.qpos[qpos].copy(); end=np.asarray(result["q"])
        for k in range(steps):
            u=(k+1)/steps; blend=u*u*(3-2*u)
            _step(model,data,indices,start+(end-start)*blend,grip)
            if collision_audit is not None:
                collision_audit.update(data, phase)
            if viewer is not None:
                if not viewer.is_running():
                    failure="viewer_closed"
                    break
                viewer.sync()
                time.sleep(model.opt.timestep)
            peak=max(peak,float(data.xpos[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"cube"),2]))
        if failure == "viewer_closed":
            break
    body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"cube")
    final=data.xpos[body].copy()
    error=float(np.linalg.norm(final[:2]-place_xy))
    success=bool(failure is None and peak > initial[2]+0.045 and error < FINAL_PLACEMENT_TOLERANCE_M and 0.39 < final[2] < 0.49)
    if failure is None and not success: failure="placement_tolerance_or_lost_grasp"
    return {"seed":int(seed),"method":method,"success":success,"failure_reason":failure,
            "initial_cube_xyz":initial.tolist(),"final_cube_xyz":final.tolist(),"place_target_xy":place_xy.tolist(),
            "target_error_m":error,"lift_peak_z_m":peak,"sim_time_s":float(data.time),
            "ik_solve_time_total_s":ik_time,"max_position_residual_m":max_pos,
            "max_orientation_residual_rad":max_rot,"solver_status":records[-1]["status"] if records else "none",
            "waypoints":records,"path_checks":path_checks,"episode_specs":{"cube_xy":cube_xy.tolist(),"place_xy":place_xy.tolist(),"yaw_rad":yaw}}


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")
