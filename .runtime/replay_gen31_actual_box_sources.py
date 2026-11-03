"""Finite offline original initial/first-view evaluation, no ROS or control."""
from pathlib import Path
from types import SimpleNamespace
import sys, json, hashlib, time
import numpy as np
W=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision import regular_box_inference as box
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,evaluate_inferred_box_contact
C=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z')
I=C/'verification/gen30_attempt1_closed_evidence_r1'
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261016T021226.502516Z_0494c8bc84a34a74933e8eae122dd1f4')
O=C/'verification/gen31_actual_box_sources_r5';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
sourcepaths=[Path(box.__file__),Path(box.__file__).with_name('outer_rgb_rectangle.py'),W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',Path(__file__).resolve()]
inputs={}
def read(p):
 inputs[str(p)]=sha(p);return json.loads(Path(p).read_text())
initialpath=C/'verification/gen31_initial_source_geometry_r1/result.json';firstpath=R/'keyframes_audits/07a8ac807d6a7f0702ef09c8866949f4ce96f8379dc1421dc6ae203af6086135.json'
rows=[]
for stamp,ap,kind in [(1792116749240888833,initialpath,'original_initial_single_frame_new_geometry_not_historical_request'),(1792116802511774778,firstpath,'original_first_observation_exact_runtime_fused_geometry')]:
 a=read(ap);g=a['replay_geometry'];T=np.array(a['snapshot_transform']['T_base_optical']);cam=read(initialpath)['camera_parameters'];K=np.array([cam[k] for k in ('fx','fy','cx','cy')]);rp=I/f'{stamp}_raw.npz';inputs[str(rp)]=sha(rp);x=np.load(rp);geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode']);traces=[]
 def trace(frame,event,arg):
  if frame.f_code.co_filename==box.__file__ and frame.f_code.co_name=='_qualify_regular_box_top' and event=='return':traces.append(frame.f_locals.copy())
  return trace
 start=time.perf_counter();sys.settrace(trace)
 try:r=box.qualify_regular_box_top(x['depth_raw'],x['object_mask'],K,cam['depth_scale'],T,geometry,TargetTrackIdentity(4,'g1-t4'),stamp,color_bgr=x['color_bgr'])
 finally:sys.settrace(None)
 elapsed=time.perf_counter()-start
 row=dict(source_stamp_ns=stamp,kind=kind,ok=r.ok,code=r.code,audit=dict(r.audit),elapsed_with_python_trace_s=elapsed,geometry_point_count=len(geometry.object_points_base),model=r.model.to_evidence_dict() if r.model else None)
 if r.ok:
  finger=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
  opens=[]
  for axis in range(2):
   e=evaluate_inferred_box_contact(r.model,r.model.center_base,r.model.axes_base[:,axis],-r.model.support_normal_base,finger,opening_fit_clearance_each_side_m=.0005)
   opens.append(dict(axis=axis,ok=e.ok,code=e.code,targeted_precision=e.targeted_view))
  row['opening']=opens
  enclosed,enclosure=box.enclose_inferred_box_geometry(geometry,r.model);row['collision_enclosure']=enclosure
  assert enclosure['model_sha256']==r.model.model_sha256
 elif traces and 'excess' in traces[-1]:
  t=traces[-1];local=t['local'];sz=t['size'];uu=np.array([t['edge_uncertainty'],t['edge_uncertainty'],t['height_uncertainty']]);ex=np.abs(local)-sz/2-uu;conflicts=[]
  for axis in range(3):
   k=int(np.argmax(ex[:,axis]));conflicts.append(dict(axis=axis,maximum_excess_m=float(ex[k,axis]),count_over_1mm=int(np.count_nonzero(ex[:,axis]>.001)),max_point_base=np.asarray(geometry.object_points_base)[k].tolist(),max_point_local=local[k].tolist()))
  row['measured_conflict_axes']=conflicts
  np.savez_compressed(O/f'{stamp}_conflict_arrays.npz',points=np.asarray(geometry.object_points_base),local=local,excess=ex,axes=t['axes'],center=t['center'],size=sz)
 rows.append(row)
report=dict(mode='offline_actual_source_public_qualifier',production_source_sha256={str(p.relative_to(W)):sha(p) for p in sourcepaths},inputs_sha256=inputs,rows=rows,live_ros_contacted=False,motion_authorized=False,physical_grasp_success_claimed=False)
(O/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(dict(path=str(O/'result.json'),sha256=sha(O/'result.json'),rows=[dict(stamp=r['source_stamp_ns'],ok=r['ok'],code=r['code'],reason=r['audit'].get('reason'),opening=r.get('opening')) for r in rows])))
