#!/usr/bin/env python3
"""Boundary placement near table extent; the scene adds no obstacle geometry."""
import argparse,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'utils'))
from core import METHODS,load_model,run_episode,save_json
m,d,*rest=load_model();idx=tuple(rest);records=[]
for method in METHODS:
    records.append(run_episode(method,m,d,idx,seed=20261091,cube_xy=[.30,-.28],place_xy=[.30,.28],max_phase_s=1.0))
p=argparse.ArgumentParser();p.add_argument('--output',default=str(ROOT/'results/challenging'));args=p.parse_args()
out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
if (out/'boundary_case.json').exists(): raise SystemExit(f"Refusing to overwrite {out/'boundary_case.json'}; choose a fresh output directory")
save_json(out/'boundary_case.json',{'scenario':'boundary placement near table extent; no added obstacles','methods':records})
for r in records: print(r['method'],r['success'],r['failure_reason'],r['final_cube_xyz'])
