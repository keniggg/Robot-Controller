#!/usr/bin/env python3
"""Diagnostic16 plane assignments using original group/joint/height formulas.

Frozen source raster pixels and the original nominal geometry are retained.
This calculation is not qualification, collision/path proof or motion authority.
"""
import copy,hashlib,itertools,json,math,os,sys,time,traceback
from pathlib import Path
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z';V=N/'verification'
IN=V/'gen48_g47_exact_box_visibility_probe_r1';OUT=V/'gen48_assignment_alternative_projection_r1'
BOUNDARY=[]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def put(p,v):
 raw=(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
 with Path(p).open('xb')as f:f.write(raw)
 assert Path(p).read_bytes()==raw
 return dict(path=str(p),sha256=sha(p))
def denied(kind,detail):BOUNDARY.append(dict(kind=kind,detail=str(detail)));raise RuntimeError('offline boundary '+kind)
def guard(event,args):
 if event.startswith('socket.')or event in ('subprocess.Popen','os.system','os.exec','os.posix_spawn','os.fork','os.forkpty'):denied(event,'forbidden')
 if event=='open'and isinstance(args[0],(str,bytes,os.PathLike)):
  p=os.fsdecode(args[0]);mode=args[1]if isinstance(args[1],str)else '';flags=args[2]if isinstance(args[2],int)else 0
  if p.startswith('/dev/')and p not in ('/dev/null','/dev/random','/dev/urandom'):denied('device',p)
  if p.startswith('/sys/class/thermal')or '/hwmon/'in p:denied('temperature',p)
  if p!='/dev/null'and(flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND)or any(c in mode for c in 'wax+')):
   try:Path(p).resolve().relative_to(OUT)
   except ValueError:denied('write_outside_output',p)
def main():
 assert E.parent.parent.is_mount()and not OUT.exists();OUT.mkdir();sys.dont_write_bytecode=True;sys.addaudithook(guard)
 os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 import numpy as np
 import cv2
 sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
 from alicia_flexible_grasp.vision import cuboid_rgb_interval as c
 release_path=V/'gen47_release_manifest.json';release=json.loads(release_path.read_text());assert release['source_count']==1052and all(sha(W/p)==h for p,h in release['source_sha256'].items())
 trace_path=IN/'assignment_stability_trace.json';trace=json.loads(trace_path.read_text());row=next(x for x in trace['events']if x['event']=='exception');p=row['infer_locals'];s=row['locals']
 A=np.asarray(p['nominal_A']);lo=np.asarray(p['nominal_lo']);hi=np.asarray(p['nominal_hi']);T=np.asarray(p['T']);K=np.asarray(p['K']);n=np.asarray(p['n']);h=float(p['top_height']);d=float(p['support_offset']);origin=np.asarray(p['origin']);tn=np.asarray(p['tn']);slab=float(p['slab']);base_hu=float(p['hu'])
 seed=p['seed_groups'];recorded=p['groups'];model=copy.deepcopy(p['model']);model['vertices_base']=np.asarray(model['vertices_base'])
 array_path=IN/'attempt_r6/inputs/1792313803719039440/color_bgr.npy';color=np.load(array_path,allow_pickle=False);assert c._array_hash(color)=='9a0a6cf3f3eb35b363f3684991ead0748d34eac87187559bbd24f9e1361c8943'
 ey,ex=np.where(cv2.Canny(cv2.cvtColor(color,cv2.COLOR_BGR2GRAY),50.,100.));edge_pixels=np.column_stack((ex,ey))
 final_only=c._groups(model,edge_pixels,T,K,A,n,d,h,base_hu)
 original_planes=tuple(r['plane']for r in recorded)
 for i in range(4):
  union=np.unique(np.concatenate((np.asarray(final_only[i]['line_pixels']),np.asarray(seed[i]['line_pixels']))),axis=0);union=union[np.lexsort((union[:,0],union[:,1]))]
  assert np.array_equal(union,np.asarray(recorded[i]['line_pixels']))
 sources=put(OUT/'original_raster_sources.json',dict(seed_groups=seed,final_only_groups=c._plain(final_only),original_union_groups=recorded,seed_plus_final_union_exact=True,nominal_A=A.tolist(),nominal_lower=lo.tolist(),nominal_upper=hi.tolist(),camera_base=T[:3,3].tolist(),T=T.tolist(),K=K.tolist(),support_normal=n.tolist(),support_offset=d,top_height=h,base_height_budget=base_hu,origin=origin.tolist(),top_normal=tn.tolist(),slab=slab))
 def make_groups(planes,hu):
  hypothetical=copy.deepcopy(model);retained=copy.deepcopy(seed)
  for i,plane in enumerate(planes):
   edge_index=recorded[i]['source_hull_edge_index'];hypothetical['edges'][edge_index]['kind']=plane
   retained[i]['plane']=plane;retained[i]['plane_height_m']=h if plane=='top'else 0.
  g=c._groups(hypothetical,edge_pixels,T,K,A,n,d,h,hu,retained_groups=retained)
  assert all(np.array_equal(np.asarray(g[i]['line_pixels']),np.asarray(recorded[i]['line_pixels']))for i in range(4))
  return g
 def states(planes):
  out=[]
  for lower,upper in ((3,1),(0,2)):
   pair=(planes[lower],planes[upper])
   out.append({('support','top'):'outside_negative',('top','top'):'inside',('top','support'):'outside_positive',('support','support'):'impossible_both_support'}[pair])
  return out
 camera_abs=T[:3,3]@A[:,:2]
 def theta0_witness(groups,physical_states):
  strips=[]
  for i,g in enumerate(groups):
   ac=1 if i%2==0 else 0;points=np.asarray(g['metric_points'])[:,ac];delta=float(g['strip_radius_m']);lower=float(points.max()-delta);upper=float(points.min()+delta)
   strips.append(dict(side=i,axis=ac,minimum_m=lower,maximum_m=upper,nonempty=lower<=upper))
  witness=dict(theta_rad=0.,strips=strips,all_four_strips_nonempty=all(x['nonempty']for x in strips),physical_states=physical_states,camera_tangent_absolute_m=camera_abs.tolist(),visibility_consistent_face_choice=False)
  if not witness['all_four_strips_nonempty']or'impossible_both_support'in physical_states:return witness
  lows=[];highs=[]
  for ac,(ls,us)in enumerate(((3,1),(0,2))):
   l0,l1=strips[ls]['minimum_m'],strips[ls]['maximum_m'];u0,u1=strips[us]['minimum_m'],strips[us]['maximum_m'];cam=float(camera_abs[ac]);state=physical_states[ac]
   if state=='inside':l1=min(l1,float(np.nextafter(cam,-np.inf)));u0=max(u0,float(np.nextafter(cam,np.inf)))
   elif state=='outside_negative':l0=max(l0,float(np.nextafter(cam,np.inf)))
   elif state=='outside_positive':u1=min(u1,float(np.nextafter(cam,-np.inf)))
   l0=max(l0,u0-.6);l1=min(l1,u1-.005)
   if l0>l1 or u0>u1:return witness
   l=float(np.clip(lo[ac],l0,l1));lower_u=max(u0,l+.005);upper_u=min(u1,l+.6)
   if lower_u>upper_u:return witness
   u=float(np.clip(hi[ac],lower_u,upper_u));lows.append(l);highs.append(u)
  # Verify every frozen raster point against its unchanged original strip.
  positions=[lows[1],highs[0],highs[1],lows[0]]
  for i,g in enumerate(groups):
   ac=1 if i%2==0 else 0
   if float(np.max(np.abs(np.asarray(g['metric_points'])[:,ac]-positions[i])))>float(g['strip_radius_m'])+1e-15:return witness
  witness.update(visibility_consistent_face_choice=True,lower_m=lows,upper_m=highs,extent_m=(np.asarray(highs)-lows).tolist(),face_normal_positions_m=positions)
  return witness
 projections=[]
 for i,g in enumerate(recorded):
  uv=np.asarray(g['line_pixels']);top=c._plane(uv,h,n,d,T,K)@A[:,:2];support=c._plane(uv,0.,n,d,T,K)@A[:,:2];delta=support-top;ac=1 if i%2==0 else 0
  projections.append(dict(side=i,normal_axis=ac,pixel_count=len(uv),support_minus_top_normal_min_m=float(delta[:,ac].min()),support_minus_top_normal_max_m=float(delta[:,ac].max()),maximum_absolute_normal_projection_difference_m=float(np.max(np.abs(delta[:,ac]))),maximum_tangent_projection_difference_m=float(np.max(np.linalg.norm(delta,axis=1))),support_minus_top_points_tangent_m=delta.tolist()))
 projref=put(OUT/'top_support_projection_differences.json',dict(sides=projections))
 all_results=[]
 for index,planes in enumerate(itertools.product(('top','support'),repeat=4)):
  started=time.monotonic();ps=states(planes);answer=dict(branch_index=index,planes=list(planes),original_assignment=planes==original_planes,physical_states=ps,physical_pattern_possible='impossible_both_support'not in ps,interpretation='Alternative input correspondence diagnostic, no qualification/authority',closure=[],original_reference_joint_failed=False)
  hu=base_hu;last=None;groups=None
  try:
   for step in range(16):
    groups=make_groups(planes,hu)
    try:joint=c._reference_joint(groups,lo,hi)
    except ValueError as error:
     answer.update(original_reference_joint_failed=True,reference_joint_error=str(error),not_a_proof_of_empty_family=True)
     try:answer['heuristic_joint']=c._plain(c._joint(groups));answer['heuristic_joint_returned']=True
     except ValueError as other:answer.update(heuristic_joint_returned=False,heuristic_joint_error=str(other),heuristic_failure_is_not_empty_family_proof=True)
     break
    new_hu,vertical=c._height_budget(A,lo,hi,h,d,origin,tn,slab,joint['edge_uncertainty'],joint['yaw_uncertainty'])
    answer['closure'].append(dict(step=step,height_budget_in_m=hu,required_height_budget_m=new_hu,axis_uncertainty_m=joint['axis_uncertainty'].tolist(),edge_uncertainty_m=joint['edge_uncertainty'],yaw_uncertainty_rad=joint['yaw_uncertainty']))
    last=joint
    if new_hu<=hu+1e-13:
     hu=max(hu,new_hu)+1e-13;answer['height_closure_completed']=True;break
    hu=new_hu
   else:answer.update(height_closure_completed=False,closure_unresolved=True)
   answer['theta0']=theta0_witness(groups,ps)
   answer['final_groups']=c._plain(groups)
   if last is not None:
    answer.update(reference_joint=c._plain(last),height_uncertainty_m=hu,axis_uncertainty_about_original_nominal_m=last['axis_uncertainty'].tolist(),yaw_uncertainty_about_original_nominal_rad=last['yaw_uncertainty'],common_yaw_interval_rad=last['common_yaw_interval_rad'],full_legacy_precision_budget=bool(last['edge_uncertainty']<=.01and last['yaw_uncertainty']<=math.radians(12.)and h>2.*hu+.002))
  except ValueError as error:answer.update(group_construction_failed=True,group_error=str(error),not_a_proof_of_empty_family=True)
  answer['wall_sec']=time.monotonic()-started;path=OUT/('branch_%02d.json'%index);ref=put(path,answer)
  summary={k:answer[k]for k in ['branch_index','planes','original_assignment','physical_states','physical_pattern_possible','original_reference_joint_failed','height_closure_completed','axis_uncertainty_about_original_nominal_m','yaw_uncertainty_about_original_nominal_rad','height_uncertainty_m','common_yaw_interval_rad','reference_joint_error','heuristic_joint_returned','group_error']if k in answer}
  summary.update(theta0_four_strips_nonempty=answer.get('theta0',{}).get('all_four_strips_nonempty'),theta0_visibility_feasible=answer.get('theta0',{}).get('visibility_consistent_face_choice'),artifact=ref);all_results.append(summary)
  if planes in (original_planes,tuple('support'if i==3 else original_planes[i]for i in range(4))):print(json.dumps(dict(stage='priority_branch',**summary),ensure_ascii=False),flush=True)
 def envelope(rows):
  if not rows:return None
  return dict(branch_count=len(rows),axis_uncertainty_about_original_nominal_m=np.max(np.asarray([r['axis_uncertainty_about_original_nominal_m']for r in rows]),axis=0).tolist(),yaw_uncertainty_about_original_nominal_rad=max(r['yaw_uncertainty_about_original_nominal_rad']for r in rows),height_uncertainty_m=max(r['height_uncertainty_m']for r in rows),all_included_branches_completed_height_closure=all(r.get('height_closure_completed')for r in rows))
 physical=[r for r in all_results if r['physical_pattern_possible']];budgeted=[r for r in physical if r.get('height_closure_completed')and not r.get('original_reference_joint_failed')];theta0=[r for r in budgeted if r['theta0_visibility_feasible']]
 assert len(all_results)==16and len(physical)==9
 assert not BOUNDARY and all(sha(W/p)==hsh for p,hsh in release['source_sha256'].items())
 result=dict(kind='gen48_all16_alternative_top_support_projection_diagnostic',calculation_completed=True,production_qualified=False,original_nominal_kept=True,original_raw_raster_responses_preserved=True,original_group_joint_height_functions_used=True,source_trace=dict(path=str(trace_path),sha256=sha(trace_path)),sealed_release=dict(path=str(release_path),sha256=sha(release_path)),source_count=1052,source_unchanged=True,raster_sources=sources,projection_differences=projref,side3_projection_summary={k:v for k,v in projections[3].items()if k!='support_minus_top_points_tangent_m'},branches=all_results,physical_pattern_possible_count=9,physical_pattern_budgeted_count=len(budgeted),theta0_visibility_feasible_count=len(theta0),conservative_envelope_all_budgeted_physical_patterns=envelope(budgeted),conservative_envelope_fixedtheta0_visibility_feasible=envelope(theta0),all_physical_patterns_budgeted=len(budgeted)==9,original_recorded_axis_uncertainty_m=s['axis'],original_recorded_yaw_uncertainty_rad=s['yaw'],limitations=['Original _reference_joint fixed0 and _joint heuristic failures are unresolved, not proofs of empty coupled families.','The theta0 witness checks all four fixed raster strips plus physical camera-side correspondence, without requiring the whole independent outer box to share one assignment.','Envelopes cover only their explicitly included branch families. No full qualification, exterior/unknown-ray, measured geometry, contact, opening or path check has been run.','Retained source raster_proposals keep original provenance; hypothetical plane labels change only diagnostic projection inputs.'],boundary_attempts=BOUNDARY,ROS_init_called=False,hardware_contacted=False,temperature_queried=False,motion_commands_sent=False,helper_sha256=sha(__file__))
 rr=put(OUT/'result.json',result);ss=put(OUT/'seal.json',dict(result=rr,helper_sha256=sha(__file__),source_trace=result['source_trace'],raster_sources=sources,projection_differences=projref,branch_artifacts=[r['artifact']for r in all_results]))
 print(json.dumps(dict(result=rr,seal=ss,physical_budgeted=len(budgeted),theta0_visibility_feasible=len(theta0),envelope=result['conservative_envelope_all_budgeted_physical_patterns'],side3=result['side3_projection_summary']),ensure_ascii=False),flush=True)
if __name__=='__main__':
 try:main()
 except BaseException as error:
  if OUT.is_dir()and not(OUT/'result.json').exists():put(OUT/'partial_failure.json',dict(error_type=type(error).__name__,error=str(error),traceback=traceback.format_exc(),boundary_attempts=BOUNDARY,not_production_qualification=True))
  raise
