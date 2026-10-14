#!/usr/bin/env python3
"""Matched-seed three-solver experiment and summary/plot regeneration."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import os
ROOT=Path(__file__).resolve().parents[1];mpldir=ROOT/'results'/'.mplconfig';mpldir.mkdir(parents=True,exist_ok=True);os.environ.setdefault('MPLCONFIGDIR',str(mpldir))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(ROOT/'utils'))
from core import METHODS,load_model,run_episode

def summarize(out):
    records=[json.loads(line) for line in (out/'episodes.jsonl').read_text().splitlines() if line]
    by_method={m:[r for r in records if r['method']==m] for m in METHODS}; summary={}
    for m,rs in by_method.items():
        summary[m]={'n':len(rs),'successes':sum(r['success'] for r in rs),'success_rate':float(np.mean([r['success'] for r in rs])) if rs else None}
        for k in ['ik_solve_time_total_s','target_error_m','sim_time_s','max_position_residual_m','max_orientation_residual_rad']:
            vals=[r[k] for r in rs]
            summary[m][k]={'mean':float(np.mean(vals)),'std':float(np.std(vals,ddof=1)) if len(vals)>1 else 0.0} if vals else None
    (out/'summary.json').write_text(json.dumps({'records':len(records),'methods':summary},indent=2)+'\n')
    for key,title in [('target_error_m','Final target error (m)'),('ik_solve_time_total_s','IK solve time per episode (s)'),('max_position_residual_m','Maximum IK position residual (m)'),('sim_time_s','Simulated episode time (s)')]:
        fig,ax=plt.subplots(figsize=(7,4.5),constrained_layout=True)
        methods=[m for m in METHODS if by_method[m]]; ax.boxplot([[r[key] for r in by_method[m]] for m in methods],tick_labels=methods,showmeans=True)
        ax.set_title(title);ax.set_ylabel(title);fig.savefig(out/(key+'.png'),dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(6,4),constrained_layout=True); rates=[summary[m]['success_rate'] or 0 for m in METHODS];bars=ax.bar(METHODS,rates);ax.set_ylim(0,1.1);ax.set_ylabel('Success rate');ax.set_title('Matched-seed task success')
    for bar,m in zip(bars,METHODS): ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+.02,f"{summary[m]['successes']}/{summary[m]['n']}",ha='center')
    fig.savefig(out/'success_rate.png',dpi=160);plt.close(fig)
    return summary

p=argparse.ArgumentParser();p.add_argument('--mode',choices=['batch','summarize'],default='batch');p.add_argument('--episodes',type=int,default=25);p.add_argument('--first-seed',type=int,default=20261013);p.add_argument('--output',default=str(ROOT/'results/batch'));a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
if a.mode=='batch':
    model,data,*rest=load_model();idx=tuple(rest);seeds=list(range(a.first_seed,a.first_seed+a.episodes))
    path=out/'episodes.jsonl'
    if path.exists(): raise SystemExit(f"Refusing to overwrite {path}; choose a fresh output directory")
    with path.open('w') as f:
        for seed in seeds:
            rng=np.random.default_rng(seed); cube=np.array([.18,-.16])+rng.uniform([-.025,-.025],[.025,.025]); place=np.array([.18,.16])+rng.uniform([-.025,-.025],[.025,.025])
            spec={'seed':seed,'cube_xy':cube.tolist(),'place_xy':place.tolist()}
            for method in METHODS:
                r=run_episode(method,model,data,idx,seed=seed,cube_xy=cube,place_xy=place); r['matched_spec']=spec
                f.write(json.dumps(r,allow_nan=False)+'\n');f.flush()
                print(f"seed={seed} method={method} success={r['success']} target_error={r['target_error_m']:.3f} failure={r['failure_reason']}",flush=True)
    (out/'manifest.json').write_text(json.dumps({'methods':METHODS,'seeds':seeds,'matched_conditions':True,'episodes_per_method':a.episodes},indent=2)+'\n')
print(json.dumps(summarize(out),indent=2))
