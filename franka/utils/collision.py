"""Sampled MuJoCo collision/contact audit for the Franka table task."""
from __future__ import annotations
import mujoco
import numpy as np


def mink_table_floor_limit(model):
    """Build Mink's signed-distance velocity constraints for robot/table and floor."""
    import mink

    table = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "worktable")
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link0")
    if min(table, floor, base) < 0:
        raise ValueError("Missing Panda base, worktable, or floor geometry")

    robot_geoms = []
    above_floor_geoms = []
    for geom in range(model.ngeom):
        if geom in (table, floor):
            continue
        if not (model.geom_contype[geom] or model.geom_conaffinity[geom]):
            continue
        body = int(model.geom_bodyid[geom])
        ancestor = body
        while ancestor > 0 and ancestor != base:
            ancestor = int(model.body_parentid[ancestor])
        if ancestor != base:
            continue
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
        robot_geoms.append(geom)
        if body_name != "link0":
            above_floor_geoms.append(geom)

    pairs = [(robot_geoms, [table]), (above_floor_geoms, [floor])]
    return mink.CollisionAvoidanceLimit(
        model,
        geom_pairs=pairs,
        minimum_distance_from_collisions=0.0,
        collision_detection_distance=0.01,
    )


def sampled_segment_clearance(model,data,qpos_ids,q0,q1,samples=25):
    """Minimum robot/table or robot/floor clearance along joint-linear samples."""
    table=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'worktable')
    floor=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'floor')
    cube=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'cube_geom')
    saved_qpos=data.qpos.copy();saved_qvel=data.qvel.copy(); minimum=float('inf'); where=None
    try:
        for k,u in enumerate(np.linspace(0,1,max(2,int(samples)))):
            data.qpos[qpos_ids]=np.asarray(q0)*(1-u)+np.asarray(q1)*u
            mujoco.mj_forward(model,data)
            for geom in range(model.ngeom):
                if not (model.geom_contype[geom] or model.geom_conaffinity[geom]) or geom in (table,floor,cube): continue
                body=int(model.geom_bodyid[geom]); body_name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,body) or ''
                for obstacle in (table,floor):
                    if obstacle==floor and body_name=='link0': continue
                    clearance=float(mujoco.mj_geomDistance(model,data,geom,obstacle,10.0,None))
                    if clearance<minimum:
                        minimum=clearance;where={'sample_index':k,'geom':mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,geom),'obstacle':mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,obstacle)}
    finally:
        data.qpos[:]=saved_qpos;data.qvel[:]=saved_qvel;mujoco.mj_forward(model,data)
    return {'minimum_sampled_clearance_m':minimum,'closest_pair':where,'samples':max(2,int(samples))}


def name(model, geom_id):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or f"geom_{geom_id}"


class CollisionAudit:
    def __init__(self, model, stride=10):
        self.model=model; self.stride=max(1,int(stride)); self.step=0
        self.table=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"worktable")
        self.floor=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"floor")
        self.cube=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"cube_geom")
        self.hand=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"hand")
        if min(self.table,self.floor,self.cube,self.hand)<0: raise ValueError("Missing audit geometry")
        self.pairs=[]
        for g in range(model.ngeom):
            if not (model.geom_contype[g] or model.geom_conaffinity[g]): continue
            if g in (self.table,self.floor,self.cube): continue
            body=int(model.geom_bodyid[g])
            body_name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,body) or ""
            arm=body
            while arm and arm != self.hand:
                arm=int(model.body_parentid[arm])
            is_hand=arm==self.hand
            if body_name != "link0":
                self.pairs.append((g,self.floor,"robot-floor",False))
            self.pairs.extend([(g,self.table,"robot-table",False),
                               (g,self.cube,"gripper-cube" if is_hand else "arm-cube",is_hand)])
        self.pairs.append((self.cube,self.table,"cube-table",True))
        self.min_by_category={}; self.unexpected_contacts=set(); self.expected_contacts=set(); self.deepest_penetration_m=0.0
        self.samples=0

    def update(self,data,phase):
        self.step+=1
        if self.step%self.stride: return
        self.samples+=1
        actual={(int(data.contact[i].geom1),int(data.contact[i].geom2)) for i in range(data.ncon)}
        actual |= {(b,a) for a,b in list(actual)}
        for a,b,category,expected in self.pairs:
            d=float(mujoco.mj_geomDistance(self.model,data,a,b,10.0,None))
            self.min_by_category[category]=min(self.min_by_category.get(category,float('inf')),d)
            if (a,b) in actual and d<=0:
                key=f"{name(self.model,a)}<->{name(self.model,b)}"
                (self.expected_contacts if expected else self.unexpected_contacts).add(key)
                self.deepest_penetration_m=min(self.deepest_penetration_m,d)

    def to_dict(self):
        return {"audit_type":"sampled per-stride geom distance plus actual MuJoCo contacts",
                "sample_stride_simulation_steps":self.stride,"samples":self.samples,
                "minimum_signed_clearance_m_by_category":self.min_by_category,
                "expected_contact_pairs":sorted(self.expected_contacts),
                "unexpected_contact_pairs":sorted(self.unexpected_contacts),
                "unexpected_contact_count":len(self.unexpected_contacts),
                "deepest_observed_penetration_m":self.deepest_penetration_m,
                "expected_contacts":"cube-table support and hand/finger-cube grasp contacts",
                "limitation":"sampled geometry pairs only; not a certified collision-free path check"}
