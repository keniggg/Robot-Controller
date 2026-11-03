#!/usr/bin/env python3
"""Pure sealed FK proposal side-gate comparison; no ROS operations."""
from pathlib import Path
from types import SimpleNamespace as NS
import importlib.util,json,math,hashlib
import numpy as np
from tf.transformations import quaternion_from_matrix
from alicia_grasp_modes.observe_return_visual import missing_side_view_gain
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
P=BASE/'grasp_records/20261009T050350.036425Z_ce203c7b9d4c4ba29e4303eb51b69d62/keyframes_audits/b8addcd0918bfbffd80a832646845834de2aa73078746db6eb867911e16c8031.json'
D=json.loads(P.read_text());G=D['replay_geometry'];F=D['observation_failure_diagnostics'];A=F['source']['captured_audit'];K=A['camera_parameters'];gaps=D['observe_return']['measured_missing_sides'];N=np.array(G['support_normal_base']);N/=np.linalg.norm(N);C=np.array(G['obb_center_base_m']);UNC=.003
SRC=Path('src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py');spec=importlib.util.spec_from_file_location('sealed_side_gate_original',SRC);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
geometry=NS(support_normal_base=N,axes_base=np.array(G['R_base_obb']),size_xyz_m=np.array(G['obb_size_xyz_m']))
height=m.RemoteGrasp6DNode._geometry_object_height_m(geometry)
def angle(a,b):return math.degrees(math.acos(float(np.clip(a@b,-1.,1.))))
rows=[]
for i,c in enumerate(F['joint_space_fk_search']['candidates']):
 tool=np.array(c['T_base_tool0']);cam=np.array(c['T_base_camera_link']);mount=np.linalg.inv(tool)@cam
 node=NS(_tool_from_camera_matrix=lambda:mount,_geometry_object_height_m=m.RemoteGrasp6DNode._geometry_object_height_m)
 pose=m.make_pose_stamped('base_link',tool[:3,3],quaternion_from_matrix(tool))
 evidence=m.RemoteGrasp6DNode._observation_side_evidence(node,geometry,pose,NS(depth_uncertainty_m=UNC))
 optical=cam[:3,0];ray=C-cam[:3,3];ray/=np.linalg.norm(ray)
 local=(C-cam[:3,3])@cam[:3,:3];uv=np.array([K['cx']-K['fx']*local[1]/local[0],K['cy']-K['fy']*local[2]/local[0]])
 gain=missing_side_view_gain(gaps,cam[:3,3],UNC);recorded=c['endpoint_checks']['missing_side_view_gain']
 assert gain['ok']==recorded['ok']
 for key in ['predicted_new_side_gain_m','projected_missing_side_m','initial_projected_missing_side_m']:assert abs(gain[key]-recorded[key])<1e-12,(i,key,gain[key],recorded[key])
 row=dict(index=i,q=c['q'],binding=c['binding'],T_base_camera_link=c['T_base_camera_link'],camera_origin_base=cam[:3,3].tolist(),optical_axis_base=optical.tolist(),current_target_ray_base=ray.tolist(),target_center_camera_xyz_m=local.tolist(),target_center_uv_px=uv.tolist(),recorded_target_center_uv_px=c['endpoint_checks']['measured_support_projection']['target_center_uv_px'],optical_to_target_ray_deg=angle(optical,ray),current_target_ray_incidence_deg=angle(ray,-N),current_target_ray_side_projection_m=height*math.sin(math.radians(angle(ray,-N))),original_side_evidence=evidence,missing_side_gain=gain,minimum_optical_incidence_deg=math.degrees(math.asin(UNC/height)),required_additional_optical_incidence_deg=max(0.,math.degrees(math.asin(UNC/height))-evidence['observation_incidence_angle_deg']),original_other_proposal_endpoint_gates={k:v['ok'] for k,v in c['endpoint_checks'].items() if isinstance(v,dict) and 'ok' in v},recorded_endpoint_rejection='OBSERVATION_TILT_SIDE_EVIDENCE_INSUFFICIENT')
 rows.append(row)
out=BASE/'ros_log/full_latest_ros_20261009T003407Z/gen14_handoff_analysis/fk_side_gate_comparison.json'
report=dict(diagnostic_only=True,execution_authority=False,no_ros_calls=True,production_modified=False,audit_path=str(P),audit_sha256=hashlib.sha256(P.read_bytes()).hexdigest(),support_normal_base=N.tolist(),current_target_center_base=C.tolist(),current_object_height_m=height,original_depth_uncertainty_m=UNC,source_binding=F['source']['captured_binding'],camera_parameters=K,rows=rows,required_generator_contract='Keep original side gate. Include h*sqrt(max(0,1-(camera_optical_axis dot -support_normal)^2)) >= existing depth uncertainty in FK optimization and re-evaluate the same original function after optimization. Keep original actual-ray missing-side gain, all FOV/support/collision/source gates and budgets.',interpretation=['The non-centred target ray and the optical axis are different; the original centred-view shortcut does not follow from a positive ray-based missing-side gain.','Both recorded proposals pass their helper endpoint gates but fail the downstream original optical side gate, so they never become strict motion candidates.','An additional optical inequality addresses this particular missing proposal constraint without lowering the gate. These two rejected proposals alone do not prove feasibility of a changed optimization or complete motion path.'],limitations=['No optimizer rerun or new feasible pose is claimed by this report.','Actual missing-side gain uses the frozen measured gap center/height; the additional current-target ray incidence is explicitly diagnostic, not a replacement for the original gate.'])
out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),rows=[{k:r[k] for k in ['index','target_center_uv_px','optical_to_target_ray_deg','current_target_ray_incidence_deg','current_target_ray_side_projection_m','original_side_evidence','minimum_optical_incidence_deg','required_additional_optical_incidence_deg']} for r in rows]),ensure_ascii=False,indent=2))
