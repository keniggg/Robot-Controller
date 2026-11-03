#!/usr/bin/env python3
"""Sealed pure geometric replay, never ROS operations or execution authority."""
import importlib.util,json,hashlib,math
from pathlib import Path
from collections import Counter
import numpy as np
from tf.transformations import quaternion_matrix,quaternion_from_matrix
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_obb_overlap,_box_corners,GripperGeometry,_signed_polygon_margin,ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M,ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');OUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/gen10_failure_analysis';P=ROOT/'grasp_records/20261009T031019.431802Z_cf2238ea63164fd2a86f610925c32457/keyframes_audits/88ea33aa28b5406fa05c8bec8d0b1d8c3e6173ddada0c34144853578f1dacdd2.json';D=json.loads(P.read_text());G=D['replay_geometry'];C=np.array(G['obb_center_base_m']);R=np.array(G['R_base_obb']);S=np.array(G['obb_size_xyz_m']);POINTS=np.array(G['object_points_base_m']);REG=np.array(D['multiview_surface']['latest_registration']['snapshot_to_reference_transform']);NORMAL=np.array(G['support_normal_base']);OFFSET=float(G['support_offset_m']);GRIPPER=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
SRC=Path('src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py');spec=importlib.util.spec_from_file_location('offline_gen10_contact',SRC);remote=importlib.util.module_from_spec(spec);spec.loader.exec_module(remote)
def matrix(s):
 T=quaternion_matrix(s['quaternion_xyzw']);T[:3,3]=s['position_m'];return T

def sat(center_a,rot_a,size_a,center_b,rot_b,size_b):
 axes=[('finger_'+str(i),rot_a[:,i]) for i in range(3)]+[('target_'+str(i),rot_b[:,i]) for i in range(3)]
 for i in range(3):
  for j in range(3):
   cross=np.cross(rot_a[:,i],rot_b[:,j]);n=np.linalg.norm(cross)
   if n>1e-9:axes.append(('cross_'+str(i)+'_'+str(j),cross/n))
 result=[]
 for name,axis in axes:
  da=float(axis@center_a);db=float(axis@center_b);ra=float(np.abs(rot_a.T@axis)@(size_a*.5));rb=float(np.abs(rot_b.T@axis)@(size_b*.5));gap=abs(db-da)-ra-rb
  result.append(dict(axis=name,direction_base=axis.tolist(),separation_gap_m=gap,translation_to_separate_m=-gap,projected_interval_overlap_m=max(0.,min(da+ra,db+rb)-max(da-ra,db-rb))))
 return dict(overlap=_obb_overlap(center_a,rot_a,size_a,center_b,rot_b,size_b),minimum_translation_to_separate=min(result,key=lambda x:x['translation_to_separate_m']),principal_axes=result[:6],all_axes=result)

def body_report(T,center,rotation,size,points,opening,core_sizes):
 out=[];jaw=T[:3,1];radius=float(np.abs(rotation.T@jaw)@(size*.5));objmid=float((center-T[:3,3])@jaw)
 for name,bc,bs in _stage_boxes(T,GRIPPER,opening,'y','z'):
  rel=(points-bc)@T[:3,:3];inside=np.all(np.abs(rel)<=bs*.5+1e-12,axis=1)
  corners=_box_corners(bc,T[:3,:3],bs);entry=dict(body=name,center_base_m=bc.tolist(),center_tool_m=((bc-T[:3,3])@T[:3,:3]).tolist(),dimensions_m=bs.tolist(),nominal_sat=sat(bc,T[:3,:3],bs,center,rotation,size),measured_point_count_inside_nominal_box=int(inside.sum()),measured_points_inside_box_sample=points[inside][:12].tolist(),minimum_current_support_clearance_m=float(np.min(corners@NORMAL+OFFSET)))
  if name in core_sizes:entry['eroded_core_sat']=sat(bc,T[:3,:3],np.array(core_sizes[name]),center,rotation,size)
  if name!='palm':
   side=1 if name=='left_finger' else -1;entry['nominal_opening_inner_face_tool_y_m']=float(((bc-T[:3,3])@T[:3,:3])[1]-side*bs[1]*.5);entry['target_outer_face_tool_y_m']=objmid+side*radius;entry['target_penetration_past_padded_inner_face_m']=side*(entry['target_outer_face_tool_y_m']-entry['nominal_opening_inner_face_tool_y_m'])
  if name!='palm':
   local=(points-T[:3,3])@T[:3,:3]
   patch_xz=local[:,[0,2]]-ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M[[0,2]]
   active=np.flatnonzero(inside)
   patch_selected=[i for i in active if _signed_polygon_margin(patch_xz[i],ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M)>=0.]
   entry['inside_aabb_and_physical_patch_xz_count']=len(patch_selected)
   entry['physical_patch_projection_is_not_full_mesh_collision_proof']=True
   entry['inside_aabb_local_tool_xyz_min_m']=local[inside].min(axis=0).tolist() if np.any(inside) else None
   entry['inside_aabb_local_tool_xyz_max_m']=local[inside].max(axis=0).tolist() if np.any(inside) else None
  out.append(entry)
 return dict(opening_width_m=opening,target_full_obb_projected_jaw_width_m=2*radius,target_center_tool_y_m=objmid,bodies=out)

def plan_at(Ts,center,rotation):
 p=Grasp6DPlan();p.header.frame_id='base_link';p.plan_id='offline_sealed_geometry_only'
 for T in Ts:
  q=quaternion_from_matrix(T);p.poses.append(remote.make_pose_stamped('base_link',T[:3,3],q).pose)
 o=p.object_geometry;o.valid=True;o.source_mode='instance_mask';o.target_track_id=D['observe_return']['target_track_id'];o.header=p.header
 for ax,v in zip('xyz',center):setattr(o.pose_base.position,ax,float(v))
 Rt=np.eye(4);Rt[:3,:3]=rotation
 for ax,v in zip('xyzw',quaternion_from_matrix(Rt)):setattr(o.pose_base.orientation,ax,float(v))
 for ax,v in zip('xyz',S):setattr(o.size_xyz_m,ax,float(v))
 for ax,v in zip('xyz',NORMAL):setattr(o.support_normal_base,ax,float(v))
 o.support_offset_m=OFFSET
 return p

rows=[]
for e in D['stable_evaluations']:
 if not e.get('moveit'):continue
 Ts=[matrix(s) for s in e['execution_sequence']['stages']];T=Ts[2];final=[REG@t for t in Ts];TF=final[2];cores={b['body']:b['core_size_xyz_m'] for b in e['contact_goal_necessary_check']['bodies']};r=remote.RemoteGrasp6DNode._contact_goal_inevitable_collision(plan_at(final,C,R),GRIPPER,.05,.001)
 assert r['rejected']==e['contact_goal_necessary_check']['rejected']
 for actual,recorded in zip(r['bodies'],e['contact_goal_necessary_check']['bodies']):
  assert actual['inevitable_overlap']==recorded['inevitable_overlap']
  assert np.allclose(actual['core_size_xyz_m'],recorded['core_size_xyz_m'],rtol=0,atol=1e-12)
 rc=REG[:3,:3]@C+REG[:3,3];rr=REG[:3,:3]@R;rp=POINTS@REG[:3,:3].T+REG[:3,3]
 raw=body_report(T,C,R,S,POINTS,.0495,cores);mixed=body_report(TF,C,R,S,POINTS,.0495,cores);consistent=body_report(TF,rc,rr,S,rp,.0495,cores)
 final_clear=[b['minimum_current_support_clearance_m'] for b in mixed['bodies']];recorded_clear=dict(e['final_contact_height_margin']['nominal']['support_clearances_m'])
 clearance_delta={b['body']:b['minimum_current_support_clearance_m']-recorded_clear[b['body']] for b in mixed['bodies']}
 maxopen=body_report(TF,C,R,S,POINTS,.05,cores)
 maxclear={b['body']:b['minimum_current_support_clearance_m']-recorded_clear[b['body']] for b in maxopen['bodies']}
 assert max(abs(x) for x in maxclear.values())<1e-9
 rawidx=e['lineage_binding']['source_raw_candidate_index'];rawvar=e['lineage_binding']['source_variant_index']
 rawrow=next(x for x in D['rows'] if x.get('source_index')==rawidx and x.get('variant_index')==rawvar and x.get('candidate_source')=='tabletop_geometry')
 declared=np.array(rawrow['jaw_axis_base']);dot=float(np.clip(declared@T[:3,1],-1,1))
 axis_audit=dict(source_raw_candidate_index=rawidx,source_variant_index=rawvar,declared_jaw_axis_base=declared.tolist(),actual_tool_y_base=T[:3,1].tolist(),angle_between_declared_and_actual_deg=math.degrees(math.acos(dot)),sampled_angle_deg=rawrow['sampled_angle_deg'],raw_projection_span_m=rawrow['projection_max_m']-rawrow['projection_min_m'],required_open_width_m=rawrow['required_open_width_m'],measured_band_width_excluding_two_2mm_clearances_m=rawrow['required_open_width_m']-.004,negative_contact_count=rawrow['negative_contact_count'],positive_contact_count=rawrow['positive_contact_count'])
 rows.append(dict(source_index=e['source_index'],variant_index=e['variant_index'],lineage=e['lineage_binding'],recorded_necessary_check=e['contact_goal_necessary_check'],original_function_replay_matched=True,final_pose_reconstruction='recorded execution_sequence transformed by exact accepted registration; zero extra support lift verified by all3 full-open box clearances matching recorded final contact height audit within1nm',opening49p5_vs_recorded50_clearance_difference_m=clearance_delta,fullopen50_vs_recorded50_clearance_difference_m=maxclear,candidate_axis_and_width=axis_audit,T_current_tool0=T.tolist(),T_final_tool0=TF.tolist(),unregistered_same_frame=raw,actual_registered_pose_current_geometry=mixed,diagnostic_registered_pose_registered_geometry=consistent,fully_open_50mm=maxopen,zero_gap_closed_diagnostic=body_report(TF,C,R,S,POINTS,0.,{})))
report=dict(scope='pure OBB/SAT reconstruction of16 sealed contact rejections, not executable evidence',record=str(P),source_sha256=hashlib.sha256(P.read_bytes()).hexdigest(),production_function_sha256=hashlib.sha256(SRC.read_bytes()).hexdigest(),opening_config_evidence='recorded necessary checks freeze49..50mm interval; midpoint49.5mm and maximum50mm both re-evaluated. This is commanded open width, not closed width.',model=dict(max_inner_gap_m=.05,finger_physical_aabb_size_m=[.0434,.0286,.06],finger_collision_padding_total_m=[.0012,.0012,.0012],finger_collision_box_size_m=[.0446,.0298,.0612],palm_size_m=[.1175,.155,.0774],source='production analytical GripperGeometry defaults; recorded core dimensions match independently for all16'),geometry={k:v for k,v in G.items() if k!='object_points_base_m'},registration=REG.tolist(),candidates=rows,limitations=['SAT translates analytical bounding boxes; overlap is not a mesh-level proof of real physical collision.','Measuring points inside boxes can establish observed overlap in the supplied frame but zero points does not certify empty space.','Registered-pose/registered-target is a diagnostic frame-consistent counterfactual only, not a change to production or authority.','No plan, service, hardware action or permission was issued.'])
p=OUT/'contact_obb_quantification_v2.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('OUTPUT',str(p));print('SHA256',hashlib.sha256(p.read_bytes()).hexdigest());print('ALL16 matched original necessary function/core sizes and final50mm support clearances; source axis consistency retained')
for key in ['unregistered_same_frame','actual_registered_pose_current_geometry','diagnostic_registered_pose_registered_geometry','fully_open_50mm']:
 views=[r[key] for r in rows];b=[x for v in views for x in v['bodies'] if x['body']!='palm'];print(key,'jaw_width_mm',min(v['target_full_obb_projected_jaw_width_m'] for v in views)*1000,max(v['target_full_obb_projected_jaw_width_m'] for v in views)*1000,'nominal_collision',sum(x['nominal_sat']['overlap'] for x in b),'minSAT_mm',min(x['nominal_sat']['minimum_translation_to_separate']['translation_to_separate_m'] for x in b)*1000,max(x['nominal_sat']['minimum_translation_to_separate']['translation_to_separate_m'] for x in b)*1000,'measuredpoints',min(x['measured_point_count_inside_nominal_box'] for x in b),max(x['measured_point_count_inside_nominal_box'] for x in b))
