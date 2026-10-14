#!/usr/bin/env python3
"""HEAL + Robotiq pick/place GUI demo and headless assignment evaluation tools."""
import argparse
import csv
import sys
import time
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer
from robot_common import ROOT, load_model

KP, KD = 180.0, 28.0
DAMPING = 0.08
IK_ITERATIONS = 180
PICK = np.array([0.35, 0.0, 0.165])
PLACE = np.array([0.35, 0.18, 0.165])

def rotation_error(target, current):
    """World-frame axis-angle error that rotates current orientation toward target."""
    R = target @ current.T
    cosine = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    angle = float(np.arccos(cosine))
    vee = np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])
    if angle < 1e-6:
        return 0.5 * vee
    if np.pi-angle < 1e-5:
        vals, vecs = np.linalg.eig(R)
        axis = np.real(vecs[:, np.argmin(np.abs(vals-1.0))])
        return angle * axis / max(np.linalg.norm(axis), 1e-12)
    return angle * vee / (2.0*np.sin(angle))

def solve_ik(model, data, qpos_ids, dof_ids, site_id, target, target_rotation=None):
    """Damped least-squares position and orientation IK, without changing live state."""
    saved_qpos, saved_qvel = data.qpos.copy(), data.qvel.copy()
    if target_rotation is None:
        target_rotation = data.site_xmat[site_id].reshape(3, 3).copy()
    seed = data.qpos[qpos_ids].copy()
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    rot_weight = 0.25
    for _ in range(IK_ITERATIONS):
        mujoco.mj_forward(model, data)
        pos_err = target - data.site_xpos[site_id]
        rot_err = rotation_error(target_rotation, data.site_xmat[site_id].reshape(3, 3))
        if np.linalg.norm(pos_err) < 0.004 and np.linalg.norm(rot_err) < 0.025:
            break
        mujoco.mj_jacSite(model, data, jp, jr, site_id)
        J = np.vstack((jp[:, dof_ids], rot_weight*jr[:, dof_ids]))
        err = np.r_[pos_err, rot_weight*rot_err]
        dq = J.T @ np.linalg.solve(J @ J.T + DAMPING**2*np.eye(6), err)
        seed += np.clip(dq, -0.035, 0.035)
        for i, qid in enumerate(qpos_ids):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, ("joint_1","joint_2","joint_3","joint_4","joint_5","joint_6")[i])
            seed[i] = np.clip(seed[i], *model.jnt_range[jid])
            data.qpos[qid] = seed[i]
    pos_err = target-data.site_xpos[site_id]
    rot_err = rotation_error(target_rotation,data.site_xmat[site_id].reshape(3,3))
    result=seed.copy()
    data.qpos[:],data.qvel[:]=saved_qpos,saved_qvel
    mujoco.mj_forward(model,data)
    return result,float(np.linalg.norm(pos_err)),float(np.linalg.norm(rot_err))

def ctrl_step(model,data,qpos_ids,dof_ids,actuator_ids,target,grip_id,grip_cmd):
    mujoco.mj_forward(model,data)
    for i,(aid,dof,qid) in enumerate(zip(actuator_ids,dof_ids,qpos_ids)):
        data.ctrl[aid] = np.clip(data.qfrc_bias[dof]+KP*(target[i]-data.qpos[qid])-KD*data.qvel[dof],-300,300)
    data.ctrl[grip_id]=grip_cmd
    mujoco.mj_step(model,data)

def run_episode(model,data,place_xy=None,seed=0,collect=False,cube_xy=None,pick_target_xy=None,viewer=None):
    from robot_common import arm_indices
    qpos,dof,acts,site,grip=arm_indices(model)
    cube_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'cube')
    place=np.array([*(place_xy if place_xy is not None else PLACE[:2]),PICK[2]])
    object_xy=np.array(cube_xy if cube_xy is not None else PICK[:2],dtype=float)
    grasp_xy=np.array(pick_target_xy if pick_target_xy is not None else object_xy,dtype=float)
    # Reset free cube and arm to model defaults each episode.
    mujoco.mj_resetData(model,data)
    cube_q=model.body_jntadr[cube_id]
    qadr=model.jnt_qposadr[cube_q]
    data.qpos[qadr:qadr+3]=[object_xy[0],object_xy[1],0.165]
    data.qpos[qadr+3:qadr+7]=[1,0,0,0]
    grip_height=PICK[2]+0.13  # Source-model pad center is 130 mm below the wrist site.
    phases=[(np.array([grasp_xy[0],grasp_xy[1],grip_height+0.12]),0,2.0),(np.array([grasp_xy[0],grasp_xy[1],grip_height]),0,2.5),
            (np.array([grasp_xy[0],grasp_xy[1],grip_height]),255,1.2),(np.array([grasp_xy[0],grasp_xy[1],grip_height+0.18]),255,2.5),
            (np.array([place[0],place[1],grip_height+0.18]),255,2.5),(np.array([place[0],place[1],grip_height]),255,2.0),
            (np.array([place[0],place[1],grip_height]),0,1.0),(np.array([place[0],place[1],grip_height+0.12]),0,1.5)]
    waypoints=[]; trajectory=[]; lifted=False; interrupted=False
    mujoco.mj_forward(model,data); start_cube=data.xpos[cube_id].copy()
    target_rotation=data.site_xmat[site].reshape(3,3).copy()
    lift_peak_z=float(start_cube[2])
    for xyz,cmd,duration in phases:
        if viewer is not None and not viewer.is_running():
            interrupted=True
            break
        qtarget,ikerr,ikrot=solve_ik(model,data,qpos,dof,site,xyz,target_rotation)
        waypoints.append({'target':xyz.tolist(),'ik_error_m':ikerr,'ik_error_rad':ikrot})
        if ikerr>0.06 or ikrot>0.25: continue
        n=int(duration/model.opt.timestep)
        q_start=data.qpos[qpos].copy()
        for step in range(n):
            if viewer is not None and not viewer.is_running():
                interrupted=True
                break
            step_start=time.perf_counter() if viewer is not None else None
            u=(step+1)/max(n,1)
            blend=u*u*(3.0-2.0*u)
            ramp_target=q_start+(qtarget-q_start)*blend
            ctrl_step(model,data,qpos,dof,acts,ramp_target,grip,cmd)
            if cmd==255 and xyz[2]>grip_height+0.05:
                lift_peak_z=max(lift_peak_z,float(data.xpos[cube_id,2]))
            if collect and step % 20 == 0:
                trajectory.append({'time_s':float(data.time),'arm_q':data.qpos[qpos].tolist(),'cube_xyz':data.xpos[cube_id].tolist(),'gripper_cmd':cmd})
            if viewer is not None:
                viewer.sync()
                remaining=model.opt.timestep-(time.perf_counter()-step_start)
                if remaining>0:
                    time.sleep(remaining)
        if interrupted:
            break
        pos=data.xpos[cube_id].copy()
    lifted=lift_peak_z>float(start_cube[2])+0.05
    final=data.xpos[cube_id].copy()
    # Placement succeeds when released cube settles near target on table.
    success=bool(not interrupted and lifted and abs(final[0]-place[0])<0.07 and abs(final[1]-place[1])<0.07 and 0.13<final[2]<0.22)
    result={'success':success,'lifted':lifted,'lift_peak_z':lift_peak_z,'start_cube':start_cube.tolist(),'final_cube':final.tolist(),'place_target':place.tolist(),'pick_target':grasp_xy.tolist(),'waypoints':waypoints,'sim_time_s':float(data.time),'trajectory':trajectory}
    if viewer is not None:
        result['interrupted']=interrupted
    return result

def workspace(model,data,grid=5):
    from robot_common import arm_indices
    qpos,dof,acts,site,grip=arm_indices(model)
    points=[]
    for x in np.linspace(0.15,0.55,grid):
        for y in np.linspace(-0.25,0.3,grid):
            for z in np.linspace(0.18,0.55,grid):
                _,err,rot=solve_ik(model,data,qpos,dof,site,np.array([x,y,z]))
                points.append((x,y,z,err,rot,err<0.025 and rot<0.05))
    return points

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--headless',action='store_true',help='run without opening a viewer'); ap.add_argument('--workspace',action='store_true'); ap.add_argument('--randomized',type=int,default=0); ap.add_argument('--demonstrations',type=int,default=0); ap.add_argument('--failure-eval',type=int,default=0); ap.add_argument('--seed',type=int,default=7); ap.add_argument('--output',default=str(ROOT / 'results/nominal')); args=ap.parse_args()
    model,data,*_=load_model(); out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
    if args.workspace:
        pts=workspace(model,data); good=sum(p[5] for p in pts); print(f'Workspace grid: {good}/{len(pts)} reachable at 25 mm and 0.05 rad tolerances')
        with (out/'workspace.csv').open('w',newline='') as f:
            w=csv.writer(f); w.writerow(['x','y','z','ik_error_m','ik_error_rad','reachable']); w.writerows(pts)
    rng=np.random.default_rng(args.seed); count=max(1,args.randomized,args.demonstrations,args.failure_eval)
    failure_offsets=[(0.02,0.0),(0.04,0.0),(0.06,0.0),(0.0,0.06),(0.08,0.08)]
    batch_mode=bool(args.workspace or args.randomized or args.demonstrations or args.failure_eval)
    # Evaluation modes stay non-interactive; a no-argument invocation opens the GUI.
    headless=args.headless or batch_mode
    if headless:
        print('Running in headless mode.' if args.headless else 'Running headless evaluation mode.', file=sys.stderr)
    else:
        print('Running in GUI mode. The viewer will stay open after the sequence finishes.', file=sys.stderr)
    if args.randomized or args.demonstrations or args.failure_eval:
        records=[]
        for i in range(count):
            xy=PLACE[:2].copy(); object_xy=PICK[:2].copy()
            if args.randomized:
                xy += rng.uniform([-0.05,-0.05],[0.05,0.05])
                object_xy += rng.uniform([-0.04,-0.04],[0.04,0.04])
            grasp_xy=object_xy.copy()
            if args.failure_eval:
                dx,dy=failure_offsets[i%len(failure_offsets)]; grasp_xy += [dx,dy]
            result=run_episode(model,data,xy,seed=args.seed+i,collect=bool(args.demonstrations),cube_xy=object_xy,pick_target_xy=grasp_xy)
            result['scenario']=i
            if args.failure_eval: result['pick_target_offset_m']=[float(grasp_xy[0]-PICK[0]),float(grasp_xy[1]-PICK[1])]
            records.append(result)
        print(f"Pick/place episodes: {sum(r['success'] for r in records)}/{len(records)} succeeded; lifted {sum(r['lifted'] for r in records)}/{len(records)}")
        for r in records: print(f"  #{r['scenario']}: success={r['success']} lifted={r['lifted']} peak_z={r['lift_peak_z']:.3f} cube={np.round(r['final_cube'],4)}")
        (out/'episodes.json').write_text(__import__('json').dumps(records,indent=2))
        return

    if headless:
        result=run_episode(model,data)
        (out/'episodes.json').write_text(__import__('json').dumps([result],indent=2))
        print(__import__('json').dumps(result,indent=2))
        return

    # Open first, then simulate and synchronize this same live model/data pair.
    with mujoco.viewer.launch_passive(model,data) as viewer:
        result=run_episode(model,data,viewer=viewer)
        (out/'episodes.json').write_text(__import__('json').dumps([result],indent=2))
        print(__import__('json').dumps(result,indent=2))
        if result['interrupted']:
            print('Sequence interrupted because the viewer was closed.')
        else:
            print('Sequence finished. Close the MuJoCo viewer window to exit.')
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.02)

if __name__=='__main__': main()
