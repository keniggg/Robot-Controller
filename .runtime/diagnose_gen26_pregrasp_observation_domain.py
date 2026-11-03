"""Pure bounded comparison of original contact pregrasp endpoints and view gates."""
import json,hashlib
from pathlib import Path
import numpy as np
from alicia_flexible_grasp.grasp.observation_working_volume import observation_obb_corners,validate_planned_observation_working_volume
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,evaluate_open_gripper_observation_envelope
from alicia_grasp_modes.observation_arrival_projection import capture_arrival_projection,validate_arrival_projection
from alicia_grasp_modes.reachable_observation_views import _optical_side_evidence
from dataclasses import asdict
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z')
ROOT=BASE/'gen26_contact_analysis'; REC=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261009T114637.653111Z_26d52c38592844ad8ff668163cae0f90')
meta=json.loads((ROOT/'source_inputs/manifest.json').read_text()); endpoints=json.loads((ROOT/'contact_endpoint_counterfactual.json').read_text())
audit=json.loads((REC/'keyframes_audits/55e2cba38b8d3d58fd4a4728eda31a143a96cc3b4735f871b55e03ba7147fabf.json').read_text())
proposal=audit['selected']['observation_view']['proposal'];mount=np.linalg.inv(proposal['T_base_tool0'])@proposal['T_base_camera_link']
camera=meta['frames'][-1]['camera'];K=[camera[k] for k in ('fx','fy','cx','cy')]
stamp=meta['snapshot_stamp_ns'];statuses=json.loads((BASE/'gen26_handoff_analysis/detector_status.json').read_text())
if isinstance(statuses,dict):raise ValueError(str(statuses.keys()))
statuses=[x['message'] for x in statuses if x['message'].get('state')=='ready' and x['message'].get('source_stamp_ns')==stamp]
assert len(statuses)==1,(stamp,len(statuses))
projection=capture_arrival_projection(statuses[0],audit['grasp_mode_selection'],stamp,camera)
g=np.load(ROOT/'fused_reconstruction_final'/('%d_enclosed_geometry.npz'%stamp))
center,axes,size,normal,offset=[g[k] for k in ('center_base','axes_base','size_xyz_m','support_normal_base','support_offset_m')]
corners=observation_obb_corners(center,axes,size)
gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
limits=dict(roi_margin_px=36.,minimum_depth_m=.07,maximum_depth_m=.5)
rows=[]
for r in endpoints['rows']:
 t=r['trials'][0];T=np.asarray(t['stages'][0]);cam=T@mount
 checks={}
 def check(name,fn):
  try:
   result=fn();checks[name]=dict(ok=bool(result.get('ok')),audit=result)
  except ValueError as exc:checks[name]=dict(ok=False,reason=str(exc),audit=getattr(exc,'audit',{}))
 check('working_volume',lambda:validate_planned_observation_working_volume(corners,cam,K,camera['width'],camera['height'],snapshot_stamp_ns=stamp,**limits))
 check('arrival_projection',lambda:validate_arrival_projection(projection,cam,K,camera['width'],camera['height'],**limits))
 check('open_gripper_envelope',lambda:asdict(evaluate_open_gripper_observation_envelope(gripper=gripper,T_base_tool0=T,opening_width_m=.04975,support_normal_base=normal,support_offset_m=float(offset),obb_center_base=center,R_base_obb=axes,obb_size_xyz_m=size)))
 check('optical_side',lambda:_optical_side_evidence(cam,normal,axes,size,.003))
 cam_points=(corners-cam[:3,3])@cam[:3,:3]
 uv=np.c_[K[2]-K[0]*cam_points[:,1]/cam_points[:,0],K[3]-K[1]*cam_points[:,2]/cam_points[:,0]]
 radial=float(np.linalg.norm(center-cam[:3,3]))
 rows.append(dict(source_index=r['source_index'],variant_index=r['variant_index'],T_base_tool0=T.tolist(),T_base_camera=cam.tolist(),radial_distance_m=radial,legacy_radial_18_22_diagnostic=.18<=radial<=.22,optical_depth_range_m=[float(cam_points[:,0].min()),float(cam_points[:,0].max())],uv_min=uv.min(0).tolist(),uv_max=uv.max(0).tolist(),checks=checks))
result=dict(scope='OFFLINE_PREGRASP_ENDPOINT_VIEW_DOMAIN_ONLY_NO_AUTHORITY',accepted_policy='rgbd_working_volume_v1; 18-22cm is legacy/soft preference, not current hard gate',source_stamp_ns=stamp,mount_derived_from_original_same_model_audited_proposal=True,projection_audit=projection['audit'],rows=rows,endpoint_geometric_all_pass=sum(all(x['ok'] for x in r['checks'].values()) for r in rows),limitations=['Original reconstructed factory endpoints, not exact cancelled runtime candidate IDs.','No IK, full arm, continuous trajectory, robust following, actual viewpoint gain, source lifetime or live transaction checked.','Original future pregrasp comes from registered contact coordinates; frozen target/prism use actual source coordinates as real observation planning would.','Existing supplemental observation still needs real missing-side/visual opportunity and budget. Contact collision alone is not evidence of visual deficit.'],inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),ROOT/'contact_endpoint_counterfactual.json',ROOT/'source_inputs/manifest.json']})
out=ROOT/'pregrasp_observation_domain.json';out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(path=str(out),all_geometric_pass=result['endpoint_geometric_all_pass'],rows=[dict(source=x['source_index'],variant=x['variant_index'],distance=x['radial_distance_m'],depth=x['optical_depth_range_m'],checks={k:v['ok'] for k,v in x['checks'].items()}) for x in rows]),indent=2))
