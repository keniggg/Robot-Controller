"""Bounded original-factory endpoint counterfactual, never ROS/motion authority."""
import runpy, json, hashlib, math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_from_matrix
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_box_corners,_obb_overlap,evaluate_explicit_candidate
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.grasp.contact_height_placement import contact_height_window_proposals
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import solve_tool0_translation_for_support_clearance
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface,SurfaceView
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
m=runpy.run_path(str(Path(__file__).with_name('replay_gen26_contact_factory.py')))
node=m['node'];remote=m['remote'];root=m['ROOT'];g=m['geometry'];reg=m['REG'];sd=m['surface_data'];meta=m['meta'];rec=m['reconstruction'];convert=m['convert']
normal=np.array(g.support_normal_base);normal/=np.linalg.norm(normal);offset=float(g.support_offset_m)
surf=FusedTargetSurface(m['identity'],sd['reference_points'],sd['view_indices'],tuple(meta['surface_source_stamps_ns']),view_sources=tuple(tuple(x) for x in rec['online_fused_geometry_checks'][-1]['view_sources']))
ref=SurfaceView(m['identity'],meta['surface_source_stamps_ns'][0],m['root_points'],sd['support_normal'],float(sd['support_offset']),rec['views'][0]['edge_clearance_px'])
node._active_multiview_surface=lambda:(surf,ref)
center=reg[:3,:3]@g.center_base+reg[:3,3];axes=reg[:3,:3]@g.axes_base;points=g.object_points_base@reg[:3,:3].T+reg[:3,3]
registered=NS(**vars(g));registered.center_base=center;registered.axes_base=axes;registered.object_points_base=points
model=Path('.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf')
fk=SerialUrdfFk(model.read_text(),tuple('Joint%d'%i for i in range(1,7)))
now=1791546486.7
def latest(name):
 rows=[x for x in meta[name] if x['header']['stamp']['secs']+x['header']['stamp']['nsecs']*1e-9<=now]
 return rows[-1]
accepted=latest('accepted');sdk=latest('sdk_command');seed=np.array(accepted['position'][:6]);bias=np.array(sdk['position'])-seed;opening=accepted['position'][6]
def sat_gap(ca,ra,sa,cb,rb,sb):
 gaps=[]
 for a in [*ra.T,*rb.T]+[np.cross(a,b) for a in ra.T for b in rb.T]:
  if np.linalg.norm(a)<1e-10:continue
  a=a/np.linalg.norm(a);gaps.append(abs((ca-cb)@a)-abs(ra.T@a)@(sa/2)-abs(rb.T@a)@(sb/2))
 return float(max(gaps))
def report(T):
 bodies=[]
 for name,c,size in _stage_boxes(T,node.gripper_geometry,opening,'y','z'):
  inside=np.all(np.abs((points-c)@T[:3,:3])<=size/2+1e-12,axis=1)
  bodies.append(dict(body=name,overlap=bool(_obb_overlap(c,T[:3,:3],size,center,axes,g.size_xyz_m)),sat_gap_m=sat_gap(c,T[:3,:3],size,center,axes,g.size_xyz_m),measured_points_inside=int(inside.sum()),support_clearance_m=float(np.min(_box_corners(c,T[:3,:3],size)@normal+offset))))
 return dict(jaw_obb_width_m=float(abs(axes.T@T[:3,1])@g.size_xyz_m),target_center_jaw_m=float((center-T[:3,3])@T[:3,1]),bodies=bodies)
def pose(T):return remote.make_pose_stamped('base_link',T[:3,3],quaternion_from_matrix(T),stamp=m['rospy'].Time.from_sec(m['STAMP']*1e-9))
def ik(T,start):
 def residual(q):
  a=fk(q);return np.r_[a[:3,3]-T[:3,3],.2*Rotation.from_dcm(T[:3,:3].T@a[:3,:3]).as_rotvec()]
 fit=least_squares(residual,start,bounds=(fk.limits[:,0],fk.limits[:,1]),max_nfev=100,ftol=1e-11,xtol=1e-11,gtol=1e-11)
 return fit.x,float(np.linalg.norm(residual(fit.x))),fit.nfev
limit=node._contact_height_evidence_limit();rows=[]
for r in m['rows']:
 if r['placed_T'] is None:continue
 original=np.array(r['placed_T']);seq,profile=node._make_contact_sequence(pose(original),g,insertion_axis_base=original[:3,2],snapshot=m['snapshot'])
 stages=[reg@remote.pose_matrix(getattr(seq,s)) for s in ('pregrasp','approach','grasp','lift')]
 lift=0.
 for T in stages:
  safe=solve_tool0_translation_for_support_clearance(rotation=T[:3,:3],lateral_target=T[:3,3],support_point=-offset*normal,support_normal=normal,clearance_m=node.gripper_geometry.support_clearance_m,gripper=node.gripper_geometry,tool_jaw_axis='y',tool_finger_length_axis='z')
  lift=max(lift,float((safe-T[:3,3])@normal))
 for T in stages:T[:3,3]+=lift*normal
 T=stages[2];contact=reg[:3,:3]@r['contact_center_base']+reg[:3,3]
 _,evidence,bounds,axis=node._current_bilateral_surface_measurement(contact,T[:3,:3],surface_frame='reference')
 row=dict(source_index=r['source_index'],variant_index=r['variant_index'],support_lift_m=lift,original_height_placement=r['placement'],trials=[])
 if not evidence.ok or bounds is None:
  row['failure']='registered_bilateral';rows.append(row);continue
 hgeo=node._contact_height_geometry(T,registered,contact,bounds,axis,.002)
 height=evaluate_contact_height_margin(translation_evidence_limit_m=limit,**hgeo);row['registered_height']=height
 options=[(T,dict(normal_translation_m=0.))]+list(contact_height_window_proposals(translation_evidence_limit_m=limit,**hgeo))
 for goal,audit in options:
  delta=float(audit['normal_translation_m']);candidate_stages=[s.copy() for s in stages]
  for s in candidate_stages:s[:3,3]+=delta*normal
  gate=evaluate_explicit_candidate(gripper=node.gripper_geometry,candidate_center_base=contact,candidate_tool0_base=goal[:3,3],R_base_tool=goal[:3,:3],required_open_width_m=r['required_open_width_m'],target_points_base=surf.points_base,obb_center_base=center,R_base_obb=axes,obb_size_xyz_m=g.size_xyz_m,support_normal_base=normal,support_offset_m=offset,pregrasp_T_base_tool=candidate_stages[0],approach_T_base_tool=candidate_stages[1],grasp_T_base_tool=candidate_stages[2],lift_T_base_tool=candidate_stages[3],tool_jaw_axis='y',tool_finger_length_axis='z',motion_cost=0.,opening_fit_clearance_each_side_m=node.opening_fit_clearance_each_side_m,width_projection_trim_fraction=node.tabletop_geometry_config.width_projection_trim_fraction,contact_band_fraction=node._contact_band_fraction(),contact_height_bounds_m=bounds,bilateral_surface_evidence=evidence,contact_height_axis_base_override=axis,minimum_contact_patch_overlap_m=.002)
  q,res,nfev=ik(goal,seed);nom=fk(q);cmd=fk(q+bias);shift=cmd[:3,3]-nom[:3,3]
  nreport=report(nom);creport=report(cmd)
  row['trials'].append(dict(delta_m=delta,height_proposal=audit,full_geometry_gate=m['asdict'](gate),weighted_ik_residual=res,nfev=nfev,q=q,nominal_T=nom,command_T=cmd,stages=candidate_stages,nominal=nreport,predicted_command=creport,command_in_joint_limits=bool(np.all(q+bias>=fk.limits[:,0]) and np.all(q+bias<=fk.limits[:,1])),position_shift_m=shift,shift_normal_m=float(shift@normal),shift_jaw_m=float(shift@nom[:3,1]),shift_lateral_m=float(shift@nom[:3,0]),shift_insertion_m=float(shift@nom[:3,2]),endpoint_pair_clear=not any(b['overlap'] or b['support_clearance_m']<.003-1e-12 for report0 in (nreport,creport) for b in report0['bodies'])))
 rows.append(row)
 print(json.dumps(dict(source=r['source_index'],variant=r['variant_index'],height=height['ok'],trials=[dict(delta=t['delta_m'],gate=t['full_geometry_gate']['ok'],res=t['weighted_ik_residual'],clear=t['endpoint_pair_clear']) for t in row['trials']]),default=convert),flush=True)
out=dict(scope='OFFLINE_RECONSTRUCTED_FACTORY_ENDPOINT_COUNTERFACTUAL_NO_AUTHORITY',sample=dict(now_sec=now,accepted=accepted,sdk=sdk,bias=bias,opening_m=opening),registration=reg,model_sha256=fk.model_sha256,rows=rows,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(__file__).with_name('replay_gen26_contact_factory.py')]},limitations=['Reconstructed OBB size differs <=0.215 micrometre; point dedup differs. Candidate IDs/order are not claimed identical to canceled runtime request.','Independent bounded IK from recorded accepted q, not saved failed MoveIt trajectory or exact failed IK branch.','SDK-command FK q+bias is the original constant-current-bias prediction, not actual measured future pose.','Original registered four-stage geometry and original height margins evaluated, no full-arm/path/physical execution proof.'])
(root/'contact_endpoint_counterfactual.json').write_text(json.dumps(convert(out),indent=2)+'\n')
