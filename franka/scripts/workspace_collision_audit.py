#!/usr/bin/env python3
"""Sample task-space endpoint reachability and one nominal collision audit."""
import argparse,csv,sys
from pathlib import Path
import numpy as np
import os
ROOT=Path(__file__).resolve().parents[1];mpldir=ROOT/'results'/'.mplconfig';mpldir.mkdir(parents=True,exist_ok=True);os.environ.setdefault('MPLCONFIGDIR',str(mpldir))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(ROOT/'utils'))
from core import METHODS,load_model,reset,solve,run_episode,save_json
from collision import CollisionAudit
ap=argparse.ArgumentParser(); ap.add_argument('--grid',type=int,default=4); ap.add_argument('--output',default=str(ROOT/'results/workspace_collision')); args=ap.parse_args()
out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
for artifact in ('workspace_samples.csv','workspace_summary.json','collision_audit.json'):
 if (out/artifact).exists(): raise SystemExit(f"Refusing to overwrite {out/artifact}; choose a fresh output directory")
m,d,*rest=load_model(); idx=tuple(rest); reset(m,d,idx); rotation=d.site_xmat[idx[3]].reshape(3,3).copy()
rows=[]
for x in np.linspace(.25,.65,args.grid):
 for y in np.linspace(-.35,.35,args.grid):
  for z in np.linspace(.40,.75,args.grid):
   for method in METHODS:
    result=solve(method,m,d,idx,np.array([x,y,z]),rotation)
    rows.append({'x_m':x,'y_m':y,'z_m':z,'method':method,'status':result['status'],'position_residual_m':result['position_residual_m'],'orientation_residual_rad':result['orientation_residual_rad'],'endpoint_reachable':result['position_residual_m']<.02 and result['orientation_residual_rad']<.15})
with (out/'workspace_samples.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
summary={'coordinate_frame':'world frame; MuJoCo meters; Panda base origin is model-defined at x=-0.3m','table_top_z_m':.375,'cube_half_extent_m':.025,'sampled_endpoint_reachability_only':True,'path_reachability_claimed':False,'grid_shape':[args.grid]*3,'position_tolerance_m':.02,'orientation_tolerance_rad':.15,'counts_by_method':{method:{'reachable':sum(r['endpoint_reachable'] for r in rows if r['method']==method),'sampled':sum(r['method']==method for r in rows)} for method in METHODS}}
save_json(out/'workspace_summary.json',summary)
fig=plt.figure(figsize=(15,5))
for i,(method,marker) in enumerate(zip(METHODS,['o','^','s']),1):
 ax=fig.add_subplot(1,3,i,projection='3d')
 pts=np.array([[r['x_m'],r['y_m'],r['z_m']] for r in rows if r['method']==method and r['endpoint_reachable']])
 if len(pts): ax.scatter(*pts.T,s=18,marker=marker,color='#2672b8')
 ax.set(xlabel='x (m)',ylabel='y (m)',zlabel='z (m)',title=f'{method}: {len(pts)}/{args.grid**3} sampled endpoints')
fig.suptitle('Sampled IK endpoint reachability (not path coverage)');fig.tight_layout();fig.savefig(out/'workspace_reachability.png',dpi=160);plt.close(fig)
audit=CollisionAudit(m,stride=10); episode=run_episode('dls',m,d,idx,seed=20261013,collision_audit=audit)
save_json(out/'collision_audit.json',{'episode':episode,'audit':audit.to_dict()})
print(summary); print('collision unexpected pairs:',audit.to_dict()['unexpected_contact_pairs'])
