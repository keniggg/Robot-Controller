#!/usr/bin/env python3
"""Independent static/numerical math review of frozen candidate helper AST only."""
import ast,copy,hashlib,itertools,json,math,os,sys,time,traceback
from pathlib import Path
W=Path(__file__).resolve().parents[1];E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
O=V/'gen48_visibility_family_independent_review_r1';C=W/'.runtime/gen48_visibility_candidate/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py'
NAMES=('_line_yaw_interval','_strip_projection_interval','_visibility_family','_same_family_evidence');BOUNDARY=[]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def put(p,x):
 raw=(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
 with p.open('xb')as f:f.write(raw)
 assert p.read_bytes()==raw
 return dict(path=str(p),sha256=sha(p))
def fingerprint(text):
 t=ast.parse(text);return {n.name:hashlib.sha256(ast.dump(n,include_attributes=False).encode()).hexdigest()for n in t.body if isinstance(n,ast.FunctionDef)and n.name in NAMES}
def denied(kind,detail):BOUNDARY.append(dict(kind=kind,detail=str(detail)));raise RuntimeError('offline boundary '+kind)
def guard(event,args):
 if event.startswith('socket.')or event in ('subprocess.Popen','os.system','os.exec','os.fork','os.forkpty','os.posix_spawn'):denied(event,'forbidden')
 if event=='open'and isinstance(args[0],(str,bytes,os.PathLike)):
  p=os.fsdecode(args[0]);mode=args[1]if isinstance(args[1],str)else '';flags=args[2]if isinstance(args[2],int)else 0
  if p.startswith('/dev/')and p not in ('/dev/null','/dev/urandom','/dev/random'):denied('device',p)
  if p.startswith('/sys/class/thermal')or '/hwmon/'in p:denied('temperature',p)
  if p!='/dev/null'and(flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND)or any(x in mode for x in 'wax+')):
   try:Path(p).resolve().relative_to(O)
   except ValueError:denied('write_outside_output',p)
def main():
 assert E.parent.parent.is_mount()and not O.exists();O.mkdir();sys.dont_write_bytecode=True;sys.addaudithook(guard);os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 import numpy as np
 sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
 from alicia_flexible_grasp.vision import cuboid_rgb_interval as original
 source=C.read_text();full_sha=sha(C);fps=fingerprint(source);assert len(fps)==4
 tree=ast.parse(source);nodes=[]
 for node in tree.body:
  if isinstance(node,ast.FunctionDef)and node.name in NAMES:nodes.append(node)
  elif isinstance(node,ast.Assign)and any(isinstance(t,ast.Name)and t.id=='_VISIBILITY_FAMILY_POLICY'for t in node.targets):nodes.append(node)
 ns=dict(original.__dict__);exec(compile(ast.Module(body=nodes,type_ignores=[]),str(C)+'::<frozen_math_helpers_only>','exec'),ns)
 frozen='\n\n'.join(ast.get_source_segment(source,n)for n in nodes)+'\n';(O/'reviewed_math_helpers.py.txt').write_text(frozen)
 trace_path=V/'gen48_g47_exact_box_visibility_probe_r1/assignment_stability_trace.json';t=json.loads(trace_path.read_text());event=next(x for x in t['events']if x['event']=='exception');p=event['infer_locals'];s=event['locals']
 A=np.asarray(p['nominal_A']);lo=np.asarray(p['nominal_lo']);hi=np.asarray(p['nominal_hi']);mid=(lo+hi)/2;size=hi-lo;T=np.asarray(p['T']);K=np.asarray(p['K']);h=float(p['top_height']);d=float(p['support_offset']);hu=float(p['hu']);camera=T[:3,3]@A[:,:2]-mid
 start=time.monotonic();family=ns['_visibility_family'](p['groups'],A,lo,hi,h,d,T,K,hu,np.asarray(s['axis']),s['yaw']);family_ref=put(O/'rebuilt_family.json',original._plain(family))
 # Pair periodicity review: every sampled exact |sin| solution must survive
 # the candidate enclosing interval. Include the closed +/-8deg endpoint case.
 beta=math.radians(8.);angles=np.r_[np.linspace(-math.pi/2,math.pi/2,49),math.pi/2-1e-12,-math.pi/2+1e-12];arcs=np.radians([.5,2.,20.,80.,82.,85.,89.,89.5]);thetas=np.linspace(-beta,beta,513);pair_trials=0;pair_points=0;pair_violations=[]
 for side in (0,1):
  for alpha in angles:
   direction=alpha+(math.pi/2 if side%2 else 0.);points=np.asarray([[0.,0.],[math.cos(direction),math.sin(direction)]])
   for arc in arcs:
    radius=.5*math.sin(arc);bounds=ns['_line_yaw_interval'](points,radius,side);valid=np.abs(np.sin(alpha-thetas))<=2.*radius
    good=thetas[valid];pair_trials+=1;pair_points+=len(good)
    bad=good[(good<bounds[0]-1e-12)|(good>bounds[1]+1e-12)]
    if len(bad):pair_violations.append(dict(side=side,alpha=alpha,arc=arc,bounds=bounds,first_bad_theta=float(bad[0])))
 endpoint_cases=[]
 for side in (0,1):
  alpha=math.pi/2;direction=alpha+(math.pi/2 if side%2 else 0.);points=np.asarray([[0.,0.],[math.cos(direction),math.sin(direction)]]);radius=.5*math.cos(beta);bounds=ns['_line_yaw_interval'](points,radius,side)
  endpoint_cases.append(dict(side=side,pair_direction_rad=direction,radius=radius,computed_bounds=bounds,negative_endpoint_retained=bounds[0]<=-beta+1e-15,positive_endpoint_retained=bounds[1]>=beta-1e-15))
 assert not pair_violations and all(x['negative_endpoint_retained']and x['positive_endpoint_retained']for x in endpoint_cases)
 pair_ref=put(O/'periodic_pair_sampling.json',dict(pair_trials=pair_trials,exact_feasible_theta_samples=pair_points,violations=pair_violations,closed_domain_endpoint_cases=endpoint_cases,proof_status='Finite checks only; interval inclusion argument recorded separately'))
 # For each of9 physical branches, exact fixed-theta offsets come from the
 # intersection of every original point band. No nominal0 fit is required.
 axis=np.asarray(family['axis_uncertainty']);yaw=family['yaw_uncertainty'];half=np.asarray([original._positive_rotation_support(size[i]/2,size[1-i]/2,yaw)+axis[i]for i in(0,1)])
 branches=[];violations=[];worst=-1e300;worst_witness=None;box_count=0;point_count=0
 for bidx,branch in enumerate(family['branches']):
  planes=branch['planes'];rows=[family['projection_groups'][i][plane]for i,plane in enumerate(planes)]
  probes=np.unique(np.r_[np.linspace(-beta,beta,513),0.,branch['raw_common_yaw_interval_rad']]);bs=dict(index=bidx,states=branch['states'],planes=planes,retained=branch['retained'],candidate_rejection=branch.get('rejection'),exact_theta_with_visibility_and_extent=0,sampled_boxes=0,points=0,maximum_signed_tangent_excess_m=None)
  best=-1e300
  for theta in probes:
   if not-beta-1e-12<=theta<=beta+1e-12:continue
   R=np.asarray([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]]);cam=camera@R;interval=[]
   for i,row in enumerate(rows):
    ac=1 if i%2==0 else 0;q=(np.asarray(row['metric_points'])-mid)@R[:,ac];delta=row['strip_radius_m'];low=float(q.max()-delta);high=float(q.min()+delta)
    positive=(planes[i]=='support')if i in(0,3)else(planes[i]=='top')
    if positive:low=max(low,float(np.nextafter(cam[ac],np.inf)))
    else:high=min(high,float(np.nextafter(cam[ac],-np.inf)))
    interval.append((low,high))
   if any(a>b for a,b in interval):continue
   had_box=False
   for choices in itertools.product((0,1),repeat=4):
    faces=[interval[i][choice]for i,choice in enumerate(choices)];lower=np.asarray([faces[3],faces[0]]);upper=np.asarray([faces[1],faces[2]]);dims=upper-lower
    if np.any(dims<.005)or np.any(dims>.6):continue
    had_box=True;xy=np.asarray(list(itertools.product((lower[0],upper[0]),(lower[1],upper[1]))))@R.T
    # Eight corner sets at both admitted top-height extremes, plus four
    # vertical-face centers, top/bottom centers and edge midpoints.
    xy=np.r_[xy,xy.mean(0)[None,:],(xy+np.roll(xy,1,axis=0))/2.]
    signed=np.abs(xy)-half;value=float(signed.max());best=max(best,value)
    if value>worst:
     worst=value;where=np.unravel_index(np.argmax(signed),signed.shape);worst_witness=dict(branch_index=bidx,theta_rad=float(theta),faces_about_rotated_nominal_center_m=faces,point_xy_about_original_center_m=xy[where[0]].tolist(),axis=int(where[1]),signed_excess_m=value)
    npoints=4*len(xy);box_count+=1;point_count+=npoints;bs['sampled_boxes']+=1;bs['points']+=npoints
    if not branch['retained']or theta<branch['enclosed_yaw_interval_rad'][0]-1e-12 or theta>branch['enclosed_yaw_interval_rad'][1]+1e-12 or value>1e-11:
     if len(violations)<16:violations.append(dict(branch_index=bidx,theta=float(theta),retained=branch['retained'],signed_excess_m=value))
   if had_box:bs['exact_theta_with_visibility_and_extent']+=1
  if best>-1e299:bs['maximum_signed_tangent_excess_m']=best
  branches.append(bs)
 assert not violations
 sample_ref=put(O/'actual_branch_corner_sampling.json',dict(theta_grid_count=513,branches=branches,box_count=box_count,corner_and_face_points_count=point_count,violations=violations,enclosure_half_extent_tangent_m=half.tolist(),axis_uncertainty_m=axis.tolist(),yaw_uncertainty_rad=yaw,maximum_signed_tangent_excess_m=worst,worst_witness=worst_witness,height_corner_sampling='Both original h-hu/h+hu tops with fixed verified support bottom; vertical envelope [support,h+hu] unchanged',no_full_qualification_claim=True))
 assert fingerprint(C.read_text())==fps,'Candidate math helpers changed during review'
 assert not BOUNDARY
 static=dict(periodic_pair_argument='Pair normal is unoriented and feasible yaw pieces repeat at pi. Wrapped pairs with a second piece touching the bounded domain must retain both; candidate now drops that restriction to the full domain with closed endpoint comparisons and roundoff allowance.',strip_interval_argument='max_i min_theta(p_i dot normal)-radius <= min_theta max_i(p_i dot normal-radius), and min_i max_theta(p_i dot normal)+radius >= max_theta min_i(p_i dot normal+radius); resulting interval encloses every feasible face offset.',camera_rejection_argument='Same projection interval is computed after subtracting the actual camera, so a whole-interval sign contradiction is a necessary infeasibility certificate. Tangencies retain adjacent interpretations.',axis_enclosure_argument='At each yaw, face offsets about the rotated original nominal center differ from nominal halfsizes by component errors. Rotating that error rectangle needs max(|e_i|cos+|e_other|sin), exactly positive_rotation_support; nominal box yaw is covered separately by unchanged downstream envelope.',floor_argument='Expanding each retained yaw interval to include +/-0.25deg can keep infeasible angles but cannot remove true feasible angles.',minimum_budget_argument='Final componentwise max with original minimum_axis/minimum_yaw prevents shrinking the original budgets.',height_closure_obligation='Candidate _visibility_family is pure for a supplied hu. Integration must recompute all assignments as hu increases and close original _height_budget with resulting maximum axis/yaw. This review does not approve infer/validator integration.',roundoff_evidence_comparator='Float fields use absolute1e-9 only; integer/count/string/bool topology is exact. Source validation must independently reconstruct evidence and maintain original policy/count/source bindings.')
 rr=put(O/'review.json',dict(kind='gen48_visibility_family_independent_math_review',reviewed=True,helper_math_no_remaining_blocker=True,production_integration_reviewed=False,source_path=str(C),source_sha256_at_capture=full_sha,helper_ast_sha256=fps,helper_ast_unchanged_during_review=True,original_dependency_source=str(Path(original.__file__)),original_dependency_sha256=sha(original.__file__),source_trace=dict(path=str(trace_path),sha256=sha(trace_path)),frozen_helper_source=dict(path=str(O/'reviewed_math_helpers.py.txt'),sha256=sha(O/'reviewed_math_helpers.py.txt')),family=family_ref,periodic=pair_ref,sampling=sample_ref,static_arguments=static,initial_closed_endpoint_issue_fixed_by_parent=True,new_helper_edits_by_reviewer=False,sampling_does_not_replace_argument=True,wall_sec=time.monotonic()-start,boundary_attempts=BOUNDARY,ROS_or_hardware_access=False,temperature_queried=False,production_or_model_or_calibration_changed=False,physical_grasp_verified=False))
 ss=put(O/'seal.json',dict(review=rr,helper_sha256=sha(__file__),family=family_ref,periodic=pair_ref,sampling=sample_ref))
 print(json.dumps(dict(review=rr,seal=ss,retained=family['retained_branch_count'],pair_trials=pair_trials,boxes=box_count,points=point_count,max_signed_excess_m=worst,wall_sec=time.monotonic()-start),ensure_ascii=False),flush=True)
if __name__=='__main__':
 try:main()
 except BaseException as error:
  if O.is_dir()and not(O/'review.json').exists():put(O/'partial_failure.json',dict(error_type=type(error).__name__,error=str(error),traceback=traceback.format_exc(),boundary_attempts=BOUNDARY,no_math_approval=True))
  raise
