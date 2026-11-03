#!/usr/bin/env python3
import os, sys, json, dataclasses
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
sys.path.insert(0,'src/alicia_grasp_modes/src')
import numpy as np, cv2
import alicia_grasp_modes.tabletop as tt
ROOT='/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_resume_20261004T002930Z/aligned_capture_20261004T003847Z'
meta=json.load(open(ROOT+'/capture.json'))
cfg=tt.Config()
K=[meta['camera'][x] for x in ('fx','fy','cx','cy')]
v,u=np.indices((480,640)); r=np.hypot(u-320,v-240)
# This manually specified pixel ROI is diagnostic only; never a grasp target or config.
obj=(u>=305)&(u<=325)&(v>=259)&(v<=278)
obj_inner=(u>=308)&(u<=322)&(v>=261)&(v<=275)
local=(u>=284)&(u<=344)&(v>=239)&(v<=298)&~((u>=300)&(u<=330)&(v>=254)&(v<=283))
out={'source':ROOT,'motion_commands_sent':False,'parameters_source':'/unknown_perception absent; production dataclass defaults','config':dataclasses.asdict(cfg),'manual_diagnostic_roi':[305,259,325,278],'frames':[]}
def stats(a):
 a=np.asarray(a);a=a[np.isfinite(a)]
 return {'n':len(a),'percentiles_0_5_25_50_75_95_100':np.percentile(a,[0,5,25,50,75,95,100]).tolist()} if len(a) else {'n':0}
for i in (0,1,2):
 d=np.load(ROOT+'/depth_%d.npy'%i).astype(np.float32)*meta['camera']['depth_scale']; p=tt._project(d,K);valid=(d>=cfg.depth_min_m)&(d<=cfg.depth_max_m)
 T=meta['pairs'][i].get('T_base_depth_frame')
 row={'index':i,'exact_T_available':T is not None,'object_depth_valid_ratio':float(np.mean(valid[obj])),'object_inner_depth_m':stats(d[obj_inner&valid]),'rings':[]}
 for outer in (170,140,110):
  ring=(r>=70)&(r<=outer)&valid
  try:
   plane,ratio,residual,n=tt._fit_plane(p[ring],dataclasses.replace(cfg,support_min_ratio=0))
   dist=np.einsum('ijk,k->ij',p,plane[:3])+plane[3]
   supporting=ring&(np.abs(dist)<=cfg.support_threshold_m)
   above=(valid&(r<=155)&(dist>=cfg.foreground_min_m)&(dist<=cfg.foreground_max_m)).astype(np.uint8)
   above=cv2.morphologyEx(above,cv2.MORPH_OPEN,np.ones((3,3),np.uint8));above&=(valid&(dist>=cfg.foreground_min_m)).astype(np.uint8)
   comp=tt._depth_components(above,d,cfg.min_object_points,cfg.depth_edge_m)
   cs=[]
   for b in comp:
    xyz=p[b].astype(float);_,_,vh=np.linalg.svd(xyz-np.mean(xyz,axis=0),full_matrices=False);oriented=(xyz-np.mean(xyz,axis=0))@vh.T;extent=np.sort(np.percentile(oriented,98,axis=0)-np.percentile(oriented,2,axis=0))[::-1]; yy,xx=np.nonzero(b)
    cs.append(dict(n=int(b.sum()),bbox=[int(xx.min()),int(yy.min()),int(xx.max()-xx.min()+1),int(yy.max()-yy.min()+1)],extent_m=extent.tolist(),center_uv=[float(xx.mean()),float(yy.mean())],overlap_object_pixels=int((b&obj).sum()),rejected_extent=bool(extent[0]>cfg.max_surface_extent_m)))
   row['rings'].append(dict(outer_radius_px=outer,plane_camera=plane.tolist(),ratio=ratio,residual_mm=residual*1000,n=n,pass_ratio=ratio>=cfg.support_min_ratio,quadrants=tt._support_quadrants(supporting,u,v,[320,240]),object_height_mm=stats(1000*dist[obj&valid]),object_inner_height_mm=stats(1000*dist[obj_inner&valid]),local_table_residual_mm=stats(1000*dist[local&valid]),raw_object_foreground_count=int((obj&valid&(dist>=cfg.foreground_min_m)).sum()),opened_object_foreground_count=int((obj&above.astype(bool)).sum()),components=cs))
  except Exception as e:row['rings'].append({'outer_radius_px':outer,'error':str(e)})
 # Local independent plane fit diagnostic (retains original fit thresholds).
 try:
  plane,ratio,res,n=tt._fit_plane(p[local&valid],cfg);dist=np.einsum('ijk,k->ij',p,plane[:3])+plane[3]
  row['local_plane_diagnostic']=dict(plane_camera=plane.tolist(),ratio=ratio,residual_mm=res*1000,object_inner_height_mm=stats(1000*dist[obj_inner&valid]),object_height_mm=stats(1000*dist[obj&valid]))
 except Exception as e:row['local_plane_diagnostic']={'error':str(e)}
 if T is not None:
  try:
   result=tt.segment_with_support_fallback(d,K,np.array(T),cfg);candidate,reason=tt.TargetTracker(cfg).choose(result,d.shape)
   row['production_result']=dict(reason=reason,metrics=result.metrics,candidates=[dict(bbox=c.bbox,center_uv=c.center_uv.tolist(),extent_m=c.extent_m.tolist(),metrics=c.metrics) for c in result.candidates])
  except Exception as e:row['production_result']={'error':str(e)}
 out['frames'].append(row)
with open(ROOT+'/offline_segmentation_diagnostic.json','w') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
