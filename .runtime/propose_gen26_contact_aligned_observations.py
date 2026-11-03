"""Four original look-at proposals with full original endpoint checks; no ROS I/O."""
import runpy,json,hashlib,itertools,time
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np,yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_from_matrix
from alicia_grasp_modes.observe_return_visual import measured_bilateral_view_gaps,missing_side_view_gain
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk,observation_endpoint_support_bound
from alicia_flexible_grasp.robot.observation_view_guard import validate_observation_support_projection
import test_remote_grasp6d_streaming as fixtures
m=runpy.run_path(str(Path(__file__).with_name('diagnose_gen26_pregrasp_observation_domain.py')))
root=m['ROOT'];meta=m['meta'];g=m['g'];normal=m['normal'];offset=float(m['offset']);center=m['center'];axes=m['axes'];size=m['size'];mount=m['mount'];camera=m['camera'];K=m['K'];corners=m['corners'];projection=m['projection'];gripper=m['gripper'];limits=m['limits'];remote=fixtures.remote_node
config=yaml.safe_load(Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())['grasp_6d']['remote']; gg=config['gripper_geometry'];opening_clearance=remote.load_opening_fit_clearance_config(gg,gripper)
_,tcfg,_=remote.load_tabletop_geometry_config(config,gripper,opening_clearance)
geom=NS(center_base=center,axes_base=axes,size_xyz_m=size,support_normal_base=normal,support_offset_m=offset)
rec=json.loads((root/'fused_reconstruction_final/result.json').read_text());s=np.load(root/'fused_reconstruction_final'/('%d_surface.npz'%m['stamp']))
surface=FusedTargetSurface(TargetTrackIdentity(meta['target_epoch'],meta['target_track_id']),s['current_points'],s['view_indices'],tuple(meta['surface_source_stamps_ns']),view_sources=tuple(tuple(x) for x in rec['online_fused_geometry_checks'][-1]['view_sources']))
raw=np.load(root/'source_inputs'/('%d.npz'%m['stamp']));actualcam=raw['T_base_camera'];q0=raw['q'][:6]
factory=json.loads((root/'factory_replay.json').read_text())
gaps=measured_bilateral_view_gaps(dict(tabletop_geometry=factory['diagnostics']),geom,tcfg,surface,gripper,actualcam[:3,3])
fk=SerialUrdfFk(Path('.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text(),tuple('Joint%d'%i for i in range(1,7)))
actualtool=fk(q0)
depth=raw['depth_raw'];mask=raw['raw_mask'];ys,xs=np.nonzero((depth>0)&(mask==0));z=depth[ys,xs].astype(float)*camera['depth_scale']
p=np.column_stack(((xs-camera['cx'])*z/camera['fx'],(ys-camera['cy'])*z/camera['fy'],z));C=np.eye(4);C[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];Tc=actualcam@C;p=p@Tc[:3,:3].T+Tc[:3,3];support=p[np.abs(p@normal+offset)<=.002]
errors=np.array(m['audit']['selected']['observation_view']['proposal_search']['joint_error_bounds_rad'])
rows=[]
for orig in m['rows']:
 if not orig['checks']['optical_side']['ok']:continue
 T=np.array(orig['T_base_tool0']);ref=remote.make_pose_stamped('base_link',T[:3,3],quaternion_from_matrix(T))
 candidate,construction=remote.centered_observation_pose_at_camera_distance(ref,center,mount,.2,.18,.22)
 goal=remote.pose_matrix(candidate);cam=goal@mount;checks={}
 def record(name,fn):
  try:
   a=fn();checks[name]=dict(ok=bool(a.get('ok')),audit=a)
  except ValueError as exc:checks[name]=dict(ok=False,reason=str(exc),audit=getattr(exc,'audit',{}))
 record('gain',lambda:missing_side_view_gain(gaps,cam[:3,3],.003))
 record('working_volume',lambda:m['validate_planned_observation_working_volume'](corners,cam,K,camera['width'],camera['height'],snapshot_stamp_ns=m['stamp'],**limits))
 record('prism',lambda:m['validate_arrival_projection'](projection,cam,K,camera['width'],camera['height'],**limits))
 record('envelope',lambda:m['asdict'](m['evaluate_open_gripper_observation_envelope'](gripper=gripper,T_base_tool0=goal,opening_width_m=.04975,support_normal_base=normal,support_offset_m=offset,obb_center_base=center,R_base_obb=axes,obb_size_xyz_m=size)))
 record('optical_side',lambda:m['_optical_side_evidence'](cam,normal,axes,size,.003))
 record('measured_support',lambda:validate_observation_support_projection(cam,support,center,K,camera['width'],camera['height'],target_corners_base=corners,**limits))
 def residual(q):
  t=fk(q);return np.r_[t[:3,3]-goal[:3,3],.2*Rotation.from_dcm(goal[:3,:3].T@t[:3,:3]).as_rotvec()]
 start=time.perf_counter();fit=least_squares(residual,q0,bounds=(fk.limits[:,0],fk.limits[:,1]),max_nfev=100,ftol=1e-11,xtol=1e-11,gtol=1e-11);elapsed=time.perf_counter()-start;res=float(np.linalg.norm(residual(fit.x)))
 checks['ik']=dict(ok=res<1e-7,q=fit.x.tolist(),weighted_residual=res,nfev=fit.nfev,wall_sec=elapsed)
 if checks['ik']['ok']:
  record('following_support',lambda:observation_endpoint_support_bound(fk,fit.x,errors,normal,offset,gripper,.04975))
 checks['distinct_baseline']=dict(ok=float(np.linalg.norm(goal[:3,3]-actualtool[:3,3]))>=.015,minimum_m=.015,actual_m=float(np.linalg.norm(goal[:3,3]-actualtool[:3,3])))
 rows.append(dict(source_index=orig['source_index'],variant_index=orig['variant_index'],construction=construction,T_base_tool0=goal.tolist(),T_base_camera=cam.tolist(),distance_to_contact_pregrasp_m=float(np.linalg.norm(goal[:3,3]-T[:3,3])),current_distance_to_same_pregrasp_m=float(np.linalg.norm(actualtool[:3,3]-T[:3,3])),checks=checks,endpoint_all_pass=all(x['ok'] for x in checks.values())))
out=dict(scope='FOUR_CONTACT_ORIENTATION_CENTERED_REAL_OBSERVATION_ENDPOINT_PROPOSALS_NO_AUTHORITY',source_stamp_ns=m['stamp'],gaps=gaps,gap_count=len(gaps),source_support_point_count=len(support),directions=len(rows),accepted_endpoint_count=sum(r['endpoint_all_pass'] for r in rows),rows=rows,original_joint_error_bounds_rad=errors.tolist(),original_gain_uncertainty_m=.003,limitations=['Only four unchanged contact orientations with the existing centered0.2m look-at constructor, one bounded local IK each; not global IK or full MoveIt paths.','Current measured support, original measured missing-side gaps, rawfrozen prism/bitmap are used; no assumed bias reduction.','Same reconstructed source limitations as factory; no new execution/observation session is authorized.','Even endpoint passes still require original strict path/fullarm/trajectory visibility/current source/actual gain and session proof.'],input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),root/'pregrasp_observation_domain.json',root/'factory_replay.json']})
path=root/'contact_aligned_observation_proposals.json';path.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(dict(path=str(path),gaps=len(gaps),accepted=out['accepted_endpoint_count'],rows=[dict(source=r['source_index'],variant=r['variant_index'],near_contact_m=r['distance_to_contact_pregrasp_m'],current_near_m=r['current_distance_to_same_pregrasp_m'],checks={k:v['ok'] for k,v in r['checks'].items()}) for r in rows]),indent=2))
