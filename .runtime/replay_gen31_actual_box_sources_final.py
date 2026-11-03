"""Finite offline original initial/first-view evaluation, no ROS or control."""
from pathlib import Path
from types import SimpleNamespace
import sys, json, hashlib, time, math
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
O=C/'verification/gen31_actual_box_sources_r6';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
sourcepaths=[Path(box.__file__),Path(box.__file__).with_name('outer_rgb_rectangle.py'),W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',Path(__file__).resolve()]
inputs={}
def read(p):
 inputs[str(p)]=sha(p);return json.loads(Path(p).read_text())
initialpath=C/'verification/gen31_initial_source_geometry_r1/result.json';firstpath=R/'keyframes_audits/07a8ac807d6a7f0702ef09c8866949f4ce96f8379dc1421dc6ae203af6086135.json'
rows=[]
for stamp,ap,kind in [(1792116749240888833,initialpath,'original_initial_single_frame_new_geometry_not_historical_request'),(1792116802511774778,firstpath,'original_first_observation_exact_runtime_fused_geometry')]:
 a=read(ap);g=a['replay_geometry'];T=np.array(a['snapshot_transform']['T_base_optical']);cam=read(initialpath)['camera_parameters'];K=np.array([cam[k] for k in ('fx','fy','cx','cy')]);rp=I/f'{stamp}_raw.npz';inputs[str(rp)]=sha(rp);x=np.load(rp);geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode']);traces=[]
 start=time.perf_counter()
 r=box.qualify_regular_box_top(x['depth_raw'],x['object_mask'],K,cam['depth_scale'],T,geometry,TargetTrackIdentity(4,'g1-t4'),stamp,color_bgr=x['color_bgr'])
 elapsed=time.perf_counter()-start
 row=dict(source_stamp_ns=stamp,kind=kind,ok=r.ok,code=r.code,audit=dict(r.audit),elapsed_public_qualifier_no_trace_s=elapsed,geometry_point_count=len(geometry.object_points_base),model=r.model.to_evidence_dict() if r.model else None)
 if r.ok:
  finger=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
  opens=[]
  for axis in range(2):
   e=evaluate_inferred_box_contact(r.model,r.model.center_base,r.model.axes_base[:,axis],-r.model.support_normal_base,finger,opening_fit_clearance_each_side_m=.0005)
   opens.append(dict(axis=axis,ok=e.ok,code=e.code,targeted_precision=e.targeted_view))
  row['opening']=opens
  enclosed,enclosure=box.enclose_inferred_box_geometry(geometry,r.model);row['collision_enclosure']=enclosure
  assert enclosure['model_sha256']==r.model.model_sha256
 elif 'precision_view_candidate' in r.audit:
  candidate=box.validate_box_precision_candidate(r.audit['precision_view_candidate'])
  size=np.array(candidate['size_xyz_m']);narrow=int(np.argmin(size[:2]));other=1-narrow
  yaw=candidate['empirical_yaw_uncertainty_rad'];edge=candidate['empirical_edge_uncertainty_m']
  turn=min(yaw,math.atan2(size[other],size[narrow]));projected=float(size[narrow]*math.cos(turn)+size[other]*math.sin(turn))
  row['observation_candidate_budget_diagnostic']=dict(not_a_qualified_contact=True,motion_authorized=False,nominal_narrow_width_m=float(size[narrow]),whole_family_projected_width_upper_m=projected+2*edge,center_at_nominal_for_diagnostic_only=True,fit_clearance_each_side_m=.0005,required_open_upper_if_same_envelope_were_qualified_m=projected+2*edge+.001,physical_inner_gap_m=.05,physical_clearance_reserve_m=.002,all_hypotheses_count=len(candidate['outer_rgb_interval'].get('top_side_hypotheses',{}).get('members',[])),raw_unknown_interior_pixels=sum(len(g['raw_unknown_pixels_yx']) for g in candidate['interior_regions']),compatible_measured_interior_pixels=sum(len(g['measured_compatible_pixels_yx']) for g in candidate['interior_regions']))
 rows.append(row)
report=dict(mode='offline_actual_source_public_qualifier',production_source_sha256={str(p.relative_to(W)):sha(p) for p in sourcepaths},inputs_sha256=inputs,rows=rows,live_ros_contacted=False,motion_authorized=False,physical_grasp_success_claimed=False)
(O/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(dict(path=str(O/'result.json'),sha256=sha(O/'result.json'),rows=[dict(stamp=r['source_stamp_ns'],ok=r['ok'],code=r['code'],reason=r['audit'].get('reason'),opening=r.get('opening')) for r in rows])))
