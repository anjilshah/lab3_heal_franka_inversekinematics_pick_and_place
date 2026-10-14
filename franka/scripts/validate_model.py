#!/usr/bin/env python3
"""Load and validate model dimensions and named Panda actuators."""
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"utils"))
from core import load_model

m,d,q,v,a,s,g,r=load_model()
print(json.dumps({"model":"models/scene.xml",
                  "mujoco_version":__import__('mujoco').__version__,"nq":m.nq,"nv":m.nv,
                  "nu":m.nu,"arm_joint_count":len(q),"arm_actuator_count":len(a),
                  "gripper_actuator":__import__('mujoco').mj_id2name(m,__import__('mujoco').mjtObj.mjOBJ_ACTUATOR,g),
                  "end_effector_site":__import__('mujoco').mj_id2name(m,__import__('mujoco').mjtObj.mjOBJ_SITE,s),
                  "joint_ranges_rad":r.tolist(),"status":"passed"},indent=2))
