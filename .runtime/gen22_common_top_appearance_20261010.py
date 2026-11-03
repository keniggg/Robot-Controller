from pathlib import Path
import json,hashlib
import numpy as np,cv2
from alicia_grasp_modes.tabletop import _project,_plane_axes
B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen22_analysis');P=B/'replay_inputs';m=json.loads((P/'manifest.json').read_text());idx={r['source_stamp_ns']:r for r in m['files']};lr=next(r for r in m['files'] if r['role']=='last_pre_motion');curr=1791638841807271480
status=json.loads((P/'last_pre_motion_status.json').read_text())['message'];plane=np.asarray(status['reference_support_plane_base']);basis=np.asarray(_plane_axes(plane[:3]));anchor=np.asarray(status['anchor_position_base']);origin=anchor@basis.T
frames=[]
for ns in (lr['source_stamp_ns'],curr):
 a=dict(np.load(idx[ns]['path']));depth=a['depth_raw'].astype(np.float32)*float(a['depth_scale']);T=a['T_base_camera'];points=_project(depth,a['K'])@T[:3,:3].T+T[:3,3];height=points@plane[:3]+plane[3]
 mask=a['recorded_refined_mask']!=0 if ns==lr['source_stamp_ns'] else cv2.imread(str(B/('crop_ab_production_'+str(ns)+'_mask.png')),0)!=0
 valid=mask&np.isfinite(depth)&(depth>=.03)&(depth<=2.)&(height>=.004)&(height<=.12)
 median=float(np.median(height[valid]));top=valid&(abs(height-median)<=.003)
 lab=cv2.cvtColor(a['color'],cv2.COLOR_BGR2LAB);xy=points[top]@basis.T-origin
 frames.append(dict(ns=ns,median_height_m=median,pixels=int(top.sum()),xy=xy,lab=lab[top]))
results=[]
for cell in (.005,.010):
 maps=[]
 for f in frames:
  keys=np.floor(f['xy']/cell).astype(int);groups={}
  for k,c in zip(map(tuple,keys),f['lab']):groups.setdefault(k,[]).append(c)
  maps.append({k:np.asarray(v) for k,v in groups.items() if len(v)>=10})
 common=sorted(set(maps[0])&set(maps[1]));rows=[]
 for key in common:
  x,y=maps[0][key],maps[1][key];xm=np.median(x,axis=0);ym=np.median(y,axis=0);rows.append(dict(cell_xy=list(map(int,key)),reference_count=len(x),current_count=len(y),reference_median_lab=xm.tolist(),current_median_lab=ym.tolist(),current_minus_reference_lab=(ym-xm).tolist()))
 deltas=np.asarray([r['current_minus_reference_lab'] for r in rows]);results.append(dict(cell_width_m=cell,minimum_points_per_view=10,common_cells=len(rows),median_cell_lab_change=np.median(deltas,axis=0).tolist(),p10_p90_cell_lab_change=np.percentile(deltas,[10,90],axis=0).tolist(),cells=rows))
report=dict(record=m['record'],reference_source_ns=lr['source_stamp_ns'],current_source_ns=curr,frames=[{k:v for k,v in f.items() if k not in ('xy','lab')} for f in frames],method='Diagnostic table-plane XY cells of current measured RGB-mask points within 3mm of each frame median foreground height, using each source exact recorded TF; no point transform correction or intensity normalization.',results=results,limitations=['Cells are common recorded base-coordinate regions, not proof of exact physical pixel/point correspondence. TF/calibration error and printed texture can mix cell contents.','The 3mm top band and 5/10mm cells are diagnostic regions only; no production thresholds are changed.','This does not isolate camera autoexposure from lighting, reflections, view angle or camera shadow.'])
out=B/'common_top_region_lab_diagnostic.json';out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),frames=report['frames'],summary=[{k:v for k,v in r.items() if k!='cells'} for r in results]),ensure_ascii=False))
