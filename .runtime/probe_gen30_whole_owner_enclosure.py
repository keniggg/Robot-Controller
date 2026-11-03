"""Single-source geometry counterfactual; not a real multi-frame admission."""
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib
import json
import os
import sys
import time
W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
I=N/'gen29_attempt1_shadow_frames'; F=N/'gen30_current_fragment_ownership_r1'
O=N/'gen30_whole_owner_enclosure_r1'
assert M.is_mount() and not O.exists() and os.environ['ROS_MASTER_URI']=='http://127.0.0.1:1'
sys.path[:0]=[str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_flexible_grasp_supervisor/src')]
import cv2
import numpy as np
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample,fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_snapshot,inferred_box_enclosure_vertices
from alicia_grasp_modes.tabletop import _project
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
inputs={}
def read(p):inputs[str(p)]=sha(p);return json.loads(Path(p).read_text())
baseline=read(W/'.runtime/gen30_baseline_source/manifest.json')
assert len(baseline)==970 and all(sha(W/p)==s for p,s in baseline.items())
plan=read(N/'gen30_initial_view_box_replay_r1/result.json')
ownership=read(F/'result.json')
assert inputs[str(F/'result.json')]=='1ca314dc192d71755f6152f278078cf4112974126597523b15389de593e12f40'
identity=TargetTrackIdentity(**plan['source_identity'])
optical=np.eye(4);optical[:3,:3]=[[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]]
rows=[]
for previous in ownership['rows']:
 ns=previous['source_stamp_ns'];directory=I/f'source_{ns}'
 inputs[str(directory/'raw_arrays.npz')]=sha(directory/'raw_arrays.npz')
 a=dict(np.load(directory/'raw_arrays.npz',allow_pickle=False))
 masks=[]
 for index in (0,2):
  p=F/f'{ns}_candidate_{index}_refined.png';inputs[str(p)]=sha(p);masks.append(cv2.imread(str(p),0))
 owner,main=masks;boundary=(main>0)&~(owner>0)
 scale=float(a['depth_scale_m']);depth=a['depth_raw'].astype(np.float32)*scale
 points=_project(depth,a['intrinsics']);base=points@a['T_base_camera_link'][:3,:3].T+a['T_base_camera_link'][:3,3]
 valid=np.isfinite(depth)&(depth>=.03)&(depth<=2.)
 measured=base[boundary&valid].astype(float)
 yy,xx=np.nonzero(owner);bbox=(int(xx.min()),int(yy.min()),int(np.ptp(xx)+1),int(np.ptp(yy)+1))
 K=NS(fx=a['intrinsics'][0],fy=a['intrinsics'][1],cx=a['intrinsics'][2],cy=a['intrinsics'][3],width=depth.shape[1],height=depth.shape[0])
 sample=RgbdSample(a['color_bgr'],a['depth_raw'],owner,bbox,None,ns/1e9,'camera_link',np.zeros(6),ns,identity.epoch,identity)
 sk=dict(plan['snapshot_arguments']);sk['depth_scale']=scale
 start=time.perf_counter();snapshot=fuse_stable_samples([sample],**sk)
 row=dict(source_stamp_ns=ns,mode='one_actual_source_with_counterfactual_owner_mask',snapshot_ok=snapshot.ok,
  boundary_pixels=int(boundary.sum()),boundary_current_measured_points=len(measured),boundary_depth_unknown=int(np.count_nonzero(boundary&~valid)))
 if snapshot.ok:
  geometry=estimate_object_geometry(snapshot.depth_raw,snapshot.target_depth_raw,snapshot.object_mask,snapshot.bbox,K,scale,a['T_base_camera_link']@optical,snapshot.source_mode,**plan['geometry_arguments'])
  row.update(geometry_ok=geometry.ok,geometry_failure=geometry.failure_code)
  if geometry.ok:
   local=(measured-geometry.center_base)@geometry.axes_base
   excess=np.maximum(np.abs(local)-geometry.size_xyz_m/2.,0.)
   row.update(geometry_center=geometry.center_base,geometry_axes=geometry.axes_base,geometry_size=geometry.size_xyz_m,
      measured_boundary_max_obb_excess_by_axis_m=excess.max(axis=0),measured_boundary_obb_contains_all=bool(np.all(excess<=1e-12)),
      measured_boundary_points_base=measured,foreground_depth_points=len(geometry.object_points_base),
      boundary_pixels_remaining_after_fusion_erosion=int(np.count_nonzero(boundary&(snapshot.object_mask>0))))
   qualified=qualify_regular_box_snapshot(snapshot,K,scale,a['T_base_camera_link']@optical,geometry,identity,ns)
   row.update(box_ok=qualified.ok,box_code=qualified.code,box_audit=dict(qualified.audit))
   if qualified.ok:
    model=qualified.model;vertices=inferred_box_enclosure_vertices(model)
    vlocal=(vertices-model.center_base)@model.axes_base;plocal=(measured-model.center_base)@model.axes_base
    outside=np.maximum(np.maximum(vlocal.min(axis=0)-plocal,plocal-vlocal.max(axis=0)),0.)
    row.update(box=model.to_evidence_dict(),measured_boundary_inferred_collision_contains_all=bool(np.all(outside<=1e-12)),
      measured_boundary_inferred_collision_excess_by_axis_m=outside.max(axis=0),
      unknown_boundary_containment_proven=False,closing_axis_opening_margin_evaluated=False)
 row['wall_sec']=time.perf_counter()-start;rows.append(row)
 print(json.dumps(row,default=serial),flush=True)
assert all(sha(W/p)==s for p,s in baseline.items()) and all(sha(p)==s for p,s in inputs.items())
O.mkdir();report=dict(rows=rows,input_sha256=inputs,source_sha256=baseline,script_sha256=sha(__file__),source_unchanged=True,
 no_ros_calls=True,no_hardware_access=True,production_modified=False,execution_authority=False,new_restart_runs_only_gen29=True,
 limitations=['Single-source pure fuser counterfactual with dummy equal joints, not a valid three-frame runtime snapshot or motion/feedback admission.',
 'Native whole-owner masks are existing independent outputs; no old-plane replacement, mask threshold change or new mask synthesis.',
 'The sealed GEN29 initial-view replay arguments provide explicit production YAML/defaults; no claim of a complete contemporaneous remote parameter capture.',
 'Unknown-depth boundary and closing-axis opening margin are not proven by a point-in-OBB test.'])
(O/'result.json').write_text(json.dumps(report,indent=2,default=serial,allow_nan=False)+'\n')
print(O/'result.json',sha(O/'result.json'))
