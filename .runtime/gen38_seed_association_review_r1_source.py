"""Closed-record seed/final association diagnostic, never a model approval.

Continuous interpolation is a counterexample search, not an exhaustive proof
that no other cuboid/association exists. Original pixels are kept unchanged.
"""
import hashlib,json,math,os,sys,time
from pathlib import Path
import numpy as np
import cv2
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
O=D/'verification/gen38_seed_association_review_r1'
G=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261017T032113.476278Z_a84cc89fc53e4c2b82680749d78b9b9f')
assert Path('/media/zhuyupei/SanStand').is_mount() and 'sitecustomize' in sys.modules
assert os.environ['OFFLINE_BOUNDARY_LOG']==str(O/'boundary.jsonl')
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision import cuboid_rgb_interval as m
from alicia_flexible_grasp.vision.cuboid_silhouette import _ray_hits
cv2.setNumThreads(1)
read=lambda p:json.loads(Path(p).read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def plain(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
P=D/'verification/gen38_cuboid_silhouette_probe_r1'
fixed=read(P/'fixed_height_proposal_diagnostics.json')['cases'][0]['result'];f=fixed['audit']
latest=read(P/'new_interval_latest.json');v=latest['audit']
I=D/'verification/gen37_actual_attempt1_source_images_r1';stamp=1792207340733134031
paths=[I/(str(stamp)+suffix) for suffix in ('_color.png','_mask.png','_depth.npy')]
before={str(p):sha(p) for p in paths};module_before=sha(m.__file__)
color=cv2.imread(str(paths[0]));mask=cv2.imread(str(paths[1]),0)>0;depth=np.load(paths[2],allow_pickle=False)
T=np.array(f['T_base_optical']);K=np.array(f['intrinsics']);n=np.array(f['support_normal_base']);d=f['support_offset_m']
origin=np.array(f['measured_top_origin_base']);tn=np.array(f['measured_top_normal_base']);h=f['nominal_height_m']
prior=np.array(f['original_common_height_interval_m']);pixel=f.get('metric_pixel_size_m',.000462)
camera=read(D/'gen37_runtime/gen37_grasp_scene_r2/camera_configuration.json');scale=camera['depth_scale']
if 'metric_pixel_size_m' not in f:
 state=np.load(D/'verification/gen37_box_source_replay_r1/exact_current_top_fit.npz');uv=state['uv'][state['keep']]
 cosine=read(D/'verification/gen37_box_source_replay_r1/current_qualification_actual.json')['audit']['camera_top_cosine']
 pixel=float(np.median(depth[uv[:,1],uv[:,0]])*scale)/min(K[:2])/cosine
slab=f['top_plane_slab_m'];A=np.array(f['nominal_axes_base']);lo=np.array(f['nominal_lower']);hi=np.array(f['nominal_upper'])
S=np.array(v['seed_axes_base']);sl=np.array(v['seed_lower']);su=np.array(v['seed_upper']);sh=v['seed_nominal_height_m']
assert abs(sh-h)<1e-8
CF=A[:,:2]@((lo+hi)/2)+n*(h/2-d);CS=S[:,:2]@((sl+su)/2)+n*(h/2-d)
theta_seed=math.atan2(float(A[:,1]@S[:,0]),float(A[:,0]@S[:,0]))
edges=cv2.Canny(cv2.cvtColor(color,cv2.COLOR_BGR2GRAY),50.,100.);ey,ex=np.where(edges);ep=np.column_stack((ex,ey))
unknown=np.column_stack(np.where(mask & (~np.isfinite(depth)|(depth<=0.))))
physical=read(G/'keyframes_audits/0e5ebbd3c562cfab2617b338045f554ffc0920a23626fbc2a49cc474cc9d798d.json')['replay_geometry']
measured=np.array(physical['object_points_base_m'])
finalmodel=m._model(A,lo,hi,h,d,T,K)
finalgroups=m._groups(finalmodel,ep,T,K,A,n,d,h,f['empirical_height_uncertainty_m'])
finalsets=[{tuple(p) for p in row['line_pixels']} for row in finalgroups]
rows=[]
for fraction in np.linspace(0.,1.,51):
 angle=(1-fraction)*theta_seed;R=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
 B=np.column_stack((A[:,:2]@R,n));C=(1-fraction)*CS+fraction*CF
 size=(1-fraction)*(su-sl)+fraction*(hi-lo);mid=C@B[:,:2];lower,upper=mid-size/2,mid+size/2
 model=m._model(B,lower,upper,h,d,T,K);dist,corners=m._complete_rgb(model,ep)
 polygon=np.zeros(mask.shape,np.uint8);cv2.fillConvexPoly(polygon,np.rint(model['hull_pixels']).astype(np.int32),1)
 interior=cv2.erode(polygon,np.ones((3,3),np.uint8))>0;coverage=float(np.count_nonzero(interior&mask))/max(1,int(interior.sum()))
 row=dict(fraction_seed_to_final=float(fraction),full_rgb_max_px=max(dist),corner_max_px=max(corners),mask_coverage=coverage,
          nominal_complete_gate=bool(max(dist)<=math.sqrt(8) and max(corners)<=math.sqrt(8) and coverage>=.88))
 if row['nominal_complete_gate']:
  try:
   hu=max(abs(h-prior[0]),abs(h-prior[1]))
   for iteration in range(20):
    groups=m._groups(model,ep,T,K,B,n,d,h,hu);joint=m._reference_joint(groups,lower,upper)
    need,vertical=m._height_budget(B,lower,upper,h,d,origin,tn,slab,joint['edge_uncertainty'],joint['yaw_uncertainty'])
    next_hu=max(max(abs(h-prior[0]),abs(h-prior[1])),need)
    if next_hu<=hu+1e-13:hu=max(hu,next_hu)+1e-13;break
    hu=next_hu
   else:raise ValueError('height family did not close')
   E=np.asarray(joint['axis_uncertainty']);yaw=joint['yaw_uncertainty'];half=[]
   for j in (0,1):
    turn=min(yaw,math.atan2(size[1-j],size[j]));half.append((size[j]*math.cos(turn)+size[1-j]*math.sin(turn))/2+E[j])
   envelope_center=C+n*hu/2;envelope_size=np.r_[2*np.array(half),h+hu]
   compatible=np.ones(len(unknown),bool)
   for shift in [(0.,0.),(-.5,-.5),(-.5,.5),(.5,-.5),(.5,.5)]:
    compatible &= _ray_hits(unknown[:,::-1]+shift,K,T,envelope_center,B,envelope_size)['hit']
   ext=m._exterior(depth,scale,T,K,origin,tn,B,lower,upper,E,yaw,pixel)
   local=(measured-C)@B;measured_half=np.r_[half,h/2+hu]
   excess=float(max(0.,np.max(np.abs(local)-measured_half)))
   seed_only=[len({tuple(p) for p in g['line_pixels']}-finalsets[j]) for j,g in enumerate(groups)]
   turn=min(yaw,math.atan2(size[0],size[1]));required=size[1]*math.cos(turn)+size[0]*math.sin(turn)+2*E[1]+.001
   nominal_delta=abs(math.atan2(float(A[:,1]@B[:,0]),float(A[:,0]@B[:,0])))
   row.update(axis_error_m=E.tolist(),yaw_rad=yaw,height_uncertainty_m=hu,
       unexplained_unknown=int(np.count_nonzero(~compatible)),maximum_measured_excess_m=excess,
       original_geometry_1mm_gate=excess<=.001,all_unknown_family_compatible=bool(np.all(compatible)),
       height_contact_interval_valid=h>2*hu+.002,precision_family_bounds=bool(E.max()<=.01 and yaw<=math.radians(12)),
       source_responses_outside_final_only_groups=seed_only,nominal_axes_angle_from_A_rad=nominal_delta,
       nominal_center_from_A_m=((C-CF)@A).tolist(),size_xy_m=size.tolist(),center_base=C.tolist(),axes_base=B.tolist(),
       required_opening_m=required,required_plus_reserve_m=required+.002,
       numerical_shape_evidence_eligible=bool(np.all(compatible) and excess<=.001 and h>2*hu+.002 and E.max()<=.01 and yaw<=math.radians(12)))
  except ValueError as error:row['later_failure']=str(error)
 rows.append(row)
assert before=={str(p):sha(p) for p in paths}
assert module_before==sha(m.__file__)
boundary=O/'boundary.jsonl';attempts=boundary.read_text().splitlines() if boundary.exists() else [];assert not attempts
eligible=[row for row in rows if row.get('numerical_shape_evidence_eligible')]
out=dict(complete=True,diagnostic_only=True,qualified_model=None,motion_authorized=False,
    exhaustive_search=False,source_arrays_unchanged=True,production_modified=False,
    source_sha256=before,module_sha256=module_before,script_sha256=sha(__file__),
    common_nominal_height_m=h,original_height_interval_m=prior.tolist(),
    interpolation_semantics='same measured h; shared center, size and support-frame yaw interpolate between raw seed and fixed-A, no independent endpoint shifts',
    cases=rows,eligible_case_count=len(eligible),eligible_with_nonfinal_responses=sum(any(r['source_responses_outside_final_only_groups']) for r in eligible),
    external_boundary_attempts=attempts,
    limitations=['Numerical counterexample search, not exhaustive admissible-association proof.',
      'Completeness of raw RGB-D source is preserved, but individual seed response associations are not automatically physical measurements.',
      'No plan, contact, path, hardware or motion approval is issued.'])
(O/'result.json').write_text(json.dumps(out,indent=2,ensure_ascii=False,default=plain)+'\n')
print(json.dumps(dict(eligible=len(eligible),eligible_with_nonfinal_responses=out['eligible_with_nonfinal_responses'],first_eligible=eligible[0] if eligible else None),default=plain))
