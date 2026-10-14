#!/usr/bin/env python3
"""Nominal Franka Panda pick-and-place, headless by default or interactive."""
import argparse, sys, time
from pathlib import Path
import mujoco, mujoco.viewer
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'utils'))
from core import METHODS, load_model, run_episode, save_json
from collision import CollisionAudit

ap=argparse.ArgumentParser(); ap.add_argument('--method',choices=METHODS,default='dls'); ap.add_argument('--seed',type=int,default=20261013); ap.add_argument('--viewer',action='store_true'); ap.add_argument('--output',default=str(ROOT/'results/nominal')); args=ap.parse_args()
out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
if (out/'episode.json').exists(): raise SystemExit(f"Refusing to overwrite {out/'episode.json'}; choose a fresh output directory")
model,data,*rest=load_model(); indices=tuple(rest); audit=CollisionAudit(model,stride=10)
if args.viewer:
    with mujoco.viewer.launch_passive(model,data) as viewer:
        result=run_episode(args.method,model,data,indices,args.seed,collision_audit=audit,viewer=viewer)
        result['collision_audit']=audit.to_dict(); save_json(out/'episode.json',result)
        while viewer.is_running(): viewer.sync(); time.sleep(0.02)
else:
    result=run_episode(args.method,model,data,indices,args.seed,collision_audit=audit)
    result['collision_audit']=audit.to_dict(); save_json(out/'episode.json',result)
print(f"method={args.method} success={result['success']} status={result['solver_status']} target_error={result['target_error_m']:.4f}m failure={result['failure_reason']}")
