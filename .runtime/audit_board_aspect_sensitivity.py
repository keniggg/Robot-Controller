#!/usr/bin/env python3
"""Image-only board aspect sensitivity; no runtime changes, no robot fit."""
import importlib.util,json
from pathlib import Path
import numpy as np
import cv2
P=Path(__file__).resolve().parent
s=importlib.util.spec_from_file_location('camaudit',P/'audit_board_camera_intrinsics.py');M=importlib.util.module_from_spec(s);s.loader.exec_module(M)
rows=json.loads((M.OUT/'detected_rays.json').read_text());train=[r for r in rows if r['role']=='training']
def adjusted(row,ratio):
 q=dict(row);o=np.array(row['object_points']);o[:,0]*=ratio;q['object_points']=o.tolist();return q
def cost(ratio,source):
 return sum(M.pose(adjusted(r,ratio),M.K0)[1]**2*len(r['object_points']) for r in source)
def fit(source):
 grid=np.linspace(.94,1.06,31);values=[cost(x,source) for x in grid];index=int(np.argmin(values))
 if index in (0,len(grid)-1):raise ValueError('aspect optimum lies at exploratory boundary')
 lo,hi=grid[index-1],grid[index+1];phi=(np.sqrt(5)-1)/2
 for _ in range(26):
  left=hi-phi*(hi-lo);right=lo+phi*(hi-lo)
  if cost(left,source)<cost(right,source):hi=right
  else:lo=left
 return float((lo+hi)/2)
ratio=fit(train);report=dict(diagnostic_only=True,runtime_changed=False,fit_uses_robot=False,fit_uses_depth=False,
 method='Fixed SDK rays and virtual K; scale board X only. Minimize image-only per-pose PnP residual over 22 training images. Validation roles never fit.',training_x_to_y_ratio=ratio,
 baseline_training_cost=cost(1.,train),candidate_training_cost=cost(ratio,train),frames=[],angle_pairs=[])
for r in rows:
 a,ar=M.pose(r,M.K0);b,br=M.pose(adjusted(r,ratio),M.K0)
 report['frames'].append(dict(capture=r['capture'],role=r['role'],dataset=r['dataset'],epoch=r['epoch'],T_base_tool=r['T_base_tool'],baseline_T_camera_board=a.tolist(),candidate_T_camera_board=b.tolist(),baseline_rms_px=ar,candidate_rms_px=br))
for i,a in enumerate(report['frames']):
 for b in report['frames'][:i]:
  if a['dataset']!=b['dataset'] or a['epoch']!=b['epoch']:continue
  A=np.linalg.inv(np.array(b['T_base_tool']))@np.array(a['T_base_tool']);fa=M.angle(A[:3,:3])
  if fa<2:continue
  q=dict(first=b['capture'],second=a['capture'],role=a['role'],fk_angle_deg=fa)
  for kind in ('baseline','candidate'):
   C=np.array(b[kind+'_T_camera_board'])@np.linalg.inv(np.array(a[kind+'_T_camera_board']));q[kind+'_camera_minus_fk_deg']=M.angle(C[:3,:3])-fa
  report['angle_pairs'].append(q)
report['fold_ratios']=[fit([r for i,r in enumerate(train) if i%4!=f]) for f in range(4)]
(M.OUT/'aspect_report.json').write_text(json.dumps(report,indent=2));(M.OUT/Path(__file__).name).write_text(Path(__file__).read_text())
print('ratio',ratio,'folds',report['fold_ratios'],'traincost',report['baseline_training_cost'],report['candidate_training_cost'])
for g in ('training','independent_validation','task_pose_validation'):
 part=[r for r in report['frames'] if r['role']==g];p=[r for r in report['angle_pairs'] if r['role']==g]
 print(g,'rms',*[np.median([r[k+'_rms_px'] for r in part]) for k in ('baseline','candidate')],'angle_abs_median_max',*[np.percentile([abs(r[k+'_camera_minus_fk_deg']) for r in p],[50,100]).tolist() for k in ('baseline','candidate')])
