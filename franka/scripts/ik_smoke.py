#!/usr/bin/env python3
"""Three-method small-target smoke and matched one-seed task run."""
import argparse, json, sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/"utils"))
from core import METHODS, load_model, reset, solve, run_episode, save_json

ap=argparse.ArgumentParser(); ap.add_argument('--output',default=str(ROOT/'results/smoke')); args=ap.parse_args()
out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
if (out/'smoke.json').exists(): raise SystemExit(f"Refusing to overwrite {out/'smoke.json'}; choose a fresh output directory")
model,data,*rest=load_model(); indices=tuple(rest)
reset(model,data,indices); site=indices[3]; xyz=data.site_xpos[site].copy()+[0.015,0.0,-0.025]; rotation=data.site_xmat[site].reshape(3,3).copy()
records=[]
for method in METHODS:
    r=solve(method,model,data,indices,xyz,rotation); r['target_xyz']=xyz.tolist(); records.append(r)
episodes=[run_episode(method,model,data,indices,seed=20261013,max_phase_s=0.35) for method in METHODS]
save_json(out/'smoke.json',{'model':'Panda 7-DoF with coupled two-finger hand','ik_smoke':records,'matched_episode_smoke':episodes})
for r in records: print(f"IK {r['method']}: {r['status']} pos={r['position_residual_m']:.4f}m ori={r['orientation_residual_rad']:.4f}rad")
for r in episodes: print(f"episode {r['method']}: success={r['success']} failure={r['failure_reason']} cube={np.round(r['final_cube_xyz'],4)}")
