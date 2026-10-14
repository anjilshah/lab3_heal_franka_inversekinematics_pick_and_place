#!/usr/bin/env python3
"""DLS controller variant with sampled joint-segment clearance gates."""
import argparse,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'utils'))
from core import load_model,run_episode,save_json
from collision import CollisionAudit,sampled_segment_clearance
p=argparse.ArgumentParser();p.add_argument('--samples',type=int,default=25);p.add_argument('--clearance-margin-mm',type=float,default=2.0);p.add_argument('--output',default=str(ROOT/'results/collision_aware'));a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
if (out/'audit.json').exists(): raise SystemExit(f"Refusing to overwrite {out/'audit.json'}; choose a fresh output directory")
m,d,*rest=load_model();idx=tuple(rest);qpos=idx[0]
def guard(q0,q1):
    info=sampled_segment_clearance(m,d,qpos,q0,q1,a.samples)
    return info['minimum_sampled_clearance_m']>=a.clearance_margin_mm/1000.0,info
audit=CollisionAudit(m,stride=10)
episode=run_episode('dls',m,d,idx,seed=20261013,collision_audit=audit,path_guard=guard)
rejected=[check for check in episode['path_checks'] if not check['accepted']]
save_json(out/'audit.json',{'controller':'DLS with per-waypoint sampled joint-linear clearance gate','sample_count_per_segment':a.samples,'clearance_margin_m':a.clearance_margin_mm/1000.0,'episode':episode,'rejected_segments':rejected,'runtime_collision_audit':audit.to_dict(),'limitation':'discrete linear joint samples can miss collisions between samples; not a certified planner'})
print('success:',episode['success'],'rejected segments:',len(rejected),'unexpected runtime contacts:',audit.to_dict()['unexpected_contact_pairs'])
