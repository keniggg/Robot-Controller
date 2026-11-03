"""Isolated complete CURRENT measured-component proposal; never production admission."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib,json,os,sys,time
W=Path(__file__).resolve().parents[1];M=Path('/media/zhuyupei/SanStand')
N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
I=N/'gen29_attempt1_shadow_frames';O=N/'gen30_measured_component_prototype_r1'
assert M.is_mount() and not O.exists() and os.environ['ROS_MASTER_URI']=='http://127.0.0.1:1'
sys.path[:0]=[str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_flexible_grasp_supervisor/src'),str(W/'src/alicia_grasp_modes/tests')]
import cv2,numpy as np
from alicia_grasp_modes import tabletop as table,rgb_instance as rgb,current_fragment_owner as existing
from alicia_grasp_modes.rgb_observation_handoff import _appearance
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance,match_observation_appearance
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
from test_arrival_fragment_recovery_guards import fragment_scene,disconnected_scene,measured_candidate,K as TEST_K,PLANE as TEST_PLANE
from test_arrival_matching_fragments import scene as two_match_scene
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ah(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
inputs={}
def read(p):inputs[str(p)]=sha(p);return json.loads(Path(p).read_text())
def geometry_audit(c,anchor,last,cfg):
 return dict(position_base=c.position_base,extent_m=c.extent_m,quality=c.quality,
  points=c.metrics['valid_points'],anchor_distance_m=float(np.linalg.norm(c.position_base-anchor.position_base)),
  prior_step_m=float(np.linalg.norm(c.position_base-last.position_base)),
  extent_ratio=c.extent_m[:2]/np.maximum(anchor.extent_m[:2],.003),
  original_matching=bool(table._matching_candidates([c],anchor,last,cfg)))

def prototype(data,reference,ns):
 """No frozen mask enters generation. It is checked only after complete current RGB."""
 result,color,depth,K,T,cfg,anchor,last,spatial,valid=[data[k] for k in
  ('result','rgb','depth','intrinsics','transform','config','anchor','last','spatial_reference','rgb_valid')]
 audit=dict(measurement_entity='one complete current measured component instead of one split fragment',
  historical_pixels_generated=0,output_mask_union_used=False,production_admission=False)
 stage='scope';clock=time.perf_counter();cpu=time.process_time();refined=None;proposal=None
 def check():audit['check_calls']=audit.get('check_calls',0)+1
 try:
  matches=table._matching_candidates(result.candidates,anchor,last,cfg);audit['original_matching_count']=len(matches)
  if len(matches)!=1 or spatial is None:
   audit.update(outcome='unchanged',reason='original_zero_or_multiple_match_or_unbound_path_preserved');return audit,None,None
  if result.metrics.get('support_current_audit_available') is not True:raise ValueError('current_support_required')
  primary=matches[0][1];plane=np.asarray(result.plane_base,dtype=float)
  frame=rgb._FrameRefinement(depth,K,T,spatial,cfg);pc=T.T@plane
  heights=np.einsum('ijk,k->ij',frame.points,pc[:3])+pc[3]
  measured_valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)&np.all(np.isfinite(frame.points),axis=-1)&(heights>=cfg.foreground_min_m)&(heights<=cfg.foreground_max_m)
  occupied=np.zeros(depth.shape,bool)
  for candidate in result.candidates:
   native=candidate.mask>0
   if candidate.mask.shape!=depth.shape or not np.array_equal(candidate.metrics['support_plane_base'],plane):raise ValueError('native_source_mismatch')
   if np.any(native&occupied):raise ValueError('native_overlap')
   if np.any(native&~measured_valid):raise ValueError('native_contains_unmeasured_or_support')
   occupied|=native
  stage='current_component'
  groups=table._depth_components(occupied.astype(np.uint8),depth,cfg.min_object_points,cfg.depth_edge_m,connectivity=8)
  selected=[g for g in groups if np.any(g&(primary.mask>0))]
  if len(selected)!=1 or np.any((primary.mask>0)&~selected[0]):raise ValueError('primary_component_not_whole')
  measured=selected[0];members=[(i,c) for i,c in enumerate(result.candidates) if np.any(measured&(c.mask>0))]
  audit.update(measured_component_pixels=int(measured.sum()),component_count=len(groups),native_member_indices=[i for i,c in members])
  if len(members)<2:
   audit.update(outcome='unchanged',reason='unique_component_has_no_other_native_members');return audit,None,None
  cores=[];core_rows=[];margin=max(8,int(round(16*min(depth.shape[1]/640.,depth.shape[0]/480.))))
  for index,candidate in members:
   native=candidate.mask>0
   if np.any(native&~measured):raise ValueError('native_component_split')
   yy,xx=np.nonzero(native);sl=np.s_[max(0,int(yy.min())-margin):min(depth.shape[0],int(yy.max())+margin+1),max(0,int(xx.min())-margin):min(depth.shape[1],int(xx.max())+margin+1)]
   core,_,evidence=frame.confidence_labels(native[sl],frame.points[sl],depth[sl],heights[sl],pc[:3],cfg,bgr=color[sl])
   fullcore=np.zeros(depth.shape,bool);fullcore[sl]=core
   if fullcore.sum()<25:raise ValueError('native_trusted_core_unproven')
   cores.append(fullcore);core_rows.append(dict(candidate_index=index,pixels=int(native.sum()),trusted_core_pixels=int(fullcore.sum()),seed_evidence=evidence))
  audit['native_core_evidence']=core_rows;stage='single_pixel_bridge'
  existing._reject_single_pixel_bridge(measured,depth,cores,cfg.depth_edge_m,check)
  audit['single_pixel_articulation_guard_passed']=True
  # Same arithmetic as the existing arrival_fragment_group raw proposal, but
  # membership is the full current measured component; no footprint clipping.
  stage='whole_raw_geometry';xyz=frame.points[measured].astype(float);base=xyz@T[:3,:3].T+T[:3,3]
  pos=np.median(xyz,axis=0);pos_base=(T@np.r_[pos,1.])[:3];centered=xyz-xyz.mean(0);_,_,vh=np.linalg.svd(centered,full_matrices=False)
  oriented=centered@vh.T;extent=np.sort(np.percentile(oriented,98,axis=0)-np.percentile(oriented,2,axis=0))[::-1]
  if extent[0]>cfg.max_surface_extent_m:raise ValueError('whole_measured_extent_too_large')
  yy,xx=np.nonzero(measured);residual=float(primary.metrics['support_residual_m'])
  quality=.65*max(0.,1.-residual/(2*cfg.support_threshold_m))+.35*min(1.,len(xyz)/400.)
  metrics=dict(primary.metrics,valid_points=len(xyz),median_height_m=float(np.median(heights[measured])),observed_surface_extent_m=extent.tolist())
  proposal=table.Candidate(measured.astype(np.uint8)*255,np.array([xx.mean(),yy.mean()]),(int(xx.min()),int(yy.min()),int(np.ptp(xx)+1),int(np.ptp(yy)+1)),
   pos,pos_base,extent,float(pos[0]),quality,metrics,table._measured_support_footprint(base,plane))
  audit['whole_raw_geometry']=geometry_audit(proposal,anchor,last,cfg)
  if quality<data['minimum_quality'] or not table._matching_candidates([proposal],anchor,last,cfg):raise ValueError('whole_original_geometry_required')
  stage='original_rgb_refine';refined=frame.refine(color,proposal,rgb_valid=valid)
  # Preserve each independently measured native core, not only an aggregate
  # .85 ratio which could discard an entire small competitor.
  retained=[]
  for row,core in zip(core_rows,cores):
   fraction=float(np.count_nonzero((refined.mask>0)&core)/np.count_nonzero(core));row['retained_fraction']=fraction;retained.append(fraction)
  audit.update(final_rgb_mask_pixels=int(np.count_nonzero(refined.mask)),final_rgb_mask_sha256=ah((refined.mask>0).astype(np.uint8)),
   per_member_core_retained_fractions=retained,original_rgb_metrics=refined.metrics)
  stage='per_member_original_core'
  if any(f<.85 for f in retained):raise ValueError('member_core_not_owned')
  final=(refined.mask>0)&measured_valid;final_quality=.65*max(0.,1.-residual/(2*cfg.support_threshold_m))+.35*min(1.,int(final.sum())/400.)
  refined=replace(refined,quality=final_quality,metrics=dict(refined.metrics,median_height_m=float(np.median(heights[final]))))
  stage='final_geometry';audit['final_geometry']=geometry_audit(refined,anchor,last,cfg)
  if refined.quality<data['minimum_quality'] or not table._matching_candidates([refined],anchor,last,cfg):raise ValueError('final_original_geometry_required')
  stage='fixed_template'
  context=dict(reference=reference,reference_role='qualified_station_identity_template',source_ns=ns,same_station_mask_iou_minimum=.85,check_current=check)
  appearance=match_observation_appearance(color,refined.mask,reference.histogram,reference.contour,rgb_valid=valid,appearance_context=context)
  audit['fixed_template_appearance']=appearance
  if not appearance['appearance_match']:raise ValueError('fixed_original_rgb_template_required')
  audit.update(outcome='candidate_passed_gates_but_not_admitted',reason='independent_instance_uniqueness_not_proven',final_measured_pixels=int(final.sum()))
 except ValueError as exc:audit.update(outcome='rejected',reason=str(exc),rejection_stage=stage)
 finally:audit['prototype_time']=dict(wall_sec=time.perf_counter()-clock,cpu_sec=time.process_time()-cpu)
 return audit,proposal,refined

def reference_for(color,mask,stamp):
 valid=np.ones(mask.shape,bool);hist,contour=_appearance(color,mask)
 return FrozenObservationAppearance(color,mask,valid,stamp,hist,contour)

def synthetic_cases():
 rows=[]
 old=two_match_scene();old['depth'][210:250,313:317]=.3;old['rgb'][210:250,313:317]=(60,90,190)
 rows.append(('existing_independent_two_matching_objects',old,np.zeros(old['depth'].shape,bool),None,'existing_test_exact_two_match_scope'))
 old,big,small=disconnected_scene();rows.append(('existing_small_core_separate_objects',old,small>0,None,'existing_test_exact_zero_match_scope'))
 # A single known target and a same-height/same-colour smaller neighbor. Two
 # independent diagonal contacts keep the native masks 4-disconnected while
 # their measured 8-neighbour graph has no one-pixel articulation.
 for kind in ('two_pixel_parallel_bridges','small_neighbor_under_fifteen_percent','wide_diagonal_contact','missing_depth_bridge'):
  d=fragment_scene();shape=d['depth'].shape;yy,xx=np.indices(shape)
  target=(yy>=210)&(yy<250)&(xx>=285)&(xx<345)
  if kind=='wide_diagonal_contact':
   target&=(xx+yy<=570)
   neighbor=(yy>=225)&(yy<246)&(xx>=320)&(xx<350)&(xx+yy>=572)&(xx+yy<=578)
   bridge=np.zeros(shape,bool)
  else:
   size=8 if kind=='small_neighbor_under_fifteen_percent' else 16
   neighbor=(yy>=220)&(yy<220+size)&(xx>=346)&(xx<356)
   bridge=np.zeros(shape,bool);bridge[219,345]=True;bridge[220+size,345]=True
  raw=np.full(shape,.3,np.float32);raw[target|neighbor|bridge]=.28
  color=np.full(shape+(3,),(60,90,190),np.uint8);color[target|neighbor|bridge]=20
  template=color.copy();template[neighbor|bridge]=(60,90,190)
  anchor=measured_candidate(target.astype(np.uint8)*255,raw)
  if kind=='missing_depth_bridge':raw[bridge]=0.;bridge[:]=False
  first=(target|bridge).astype(np.uint8)*255;second=neighbor.astype(np.uint8)*255
  d.update(rgb=color,depth=raw,anchor=anchor,last=anchor,
   result=table.Segmentation([measured_candidate(first,raw),measured_candidate(second,raw)],TEST_PLANE.copy(),{'support_current_audit_available':True}),
   spatial_reference=rgb.freeze_observation_spatial_reference(anchor,TEST_PLANE))
  assert len(table._matching_candidates(d['result'].candidates,anchor,anchor,d['config']))==1
  assert cv2.connectedComponents((first|second)>0 if False else ((first|second)>0).astype(np.uint8),connectivity=4)[0]-1==2
  ref=reference_for(template,target.astype(np.uint8)*255,100)
  rows.append((kind,d,neighbor,ref,'adapted_existing_independent_fixture_single_match; native masks genuinely 4-disconnected'))
 return rows

meta=read(I/'mask_contract_review_r2/result.json');cfg=table.Config(**meta['effective_config'])
earlier=read(N/'gen30_current_fragment_owner_r3/result.json')
qns=meta['qualification_template_source_ns'];qdir=I/f'source_{qns}'
qmeta=read(qdir/'metadata.json');inputs[str(qdir/'raw_arrays.npz')]=sha(qdir/'raw_arrays.npz');q=dict(np.load(qdir/'raw_arrays.npz',allow_pickle=False))
qvalid=rectification_rgb_valid_mask(qmeta['camera_parameters'],q['depth_raw'].shape);hist,contour=_appearance(q['color_bgr'],q['object_mask'])
reference=FrozenObservationAppearance(q['color_bgr'],q['object_mask'],qvalid,qns,hist,contour)
sources={str(m.__file__):sha(m.__file__) for m in (table,rgb,existing,sys.modules['alicia_grasp_modes.arrival_fragment_group'],
 sys.modules['test_arrival_fragment_recovery_guards'],sys.modules['test_arrival_matching_fragments'])}
O.mkdir();cv2.setNumThreads(1);actual=[];synthetic=[]
for previous,old in zip(meta['rows'][1:],earlier['rows']):
 ns=previous['source_stamp_ns'];p=I/f'source_{ns}';status=read(p/'detector_status.json')['payload'];camera=read(p/'metadata.json')['camera_parameters']
 inputs[str(p/'raw_arrays.npz')]=sha(p/'raw_arrays.npz');a=dict(np.load(p/'raw_arrays.npz',allow_pickle=False))
 depth=a['depth_raw'].astype(np.float32)*float(a['depth_scale_m']);K=a['intrinsics'];T=a['T_base_camera_link'];color=a['color_bgr']
 anchor=NS(position_base=np.asarray(status['anchor_position_base']),extent_m=np.asarray(status['anchor_extent_m']),support_footprint_base=np.asarray(status['reference_support_footprint_base']),metrics={'median_height_m':status['reference_object_height_m']})
 last=NS(position_base=np.asarray(previous['prior_position_used']));locked=np.asarray(status['reference_support_plane_base'])
 start,cpu=time.perf_counter(),time.process_time();result=table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,locked,prefer_current_support=True)
 segment_time=dict(wall_sec=time.perf_counter()-start,cpu_sec=time.process_time()-cpu)
 d=dict(result=result,rgb=color,depth=depth,intrinsics=K,transform=T,config=cfg,anchor=anchor,last=last,
  spatial_reference=rgb.freeze_observation_spatial_reference(anchor,locked),rgb_valid=rectification_rgb_valid_mask(camera,depth.shape),minimum_quality=.5)
 audit,proposal,refined=prototype(d,reference,ns);audit.update(source_stamp_ns=ns,segmentation_time=segment_time,
  previously_measured_primary_refinement_time=old['primary_time'],node_total_wall_time_measured=False)
 if proposal is not None:cv2.imwrite(str(O/f'{ns}_whole_raw.png'),proposal.mask)
 if refined is not None:cv2.imwrite(str(O/f'{ns}_whole_refined.png'),refined.mask)
 actual.append(audit);print(json.dumps({k:v for k,v in audit.items() if k in ('source_stamp_ns','outcome','reason','rejection_stage','prototype_time','per_member_core_retained_fractions','fixed_template_appearance')},default=serial),flush=True)
for index,(name,d,neighbor,ref,origin) in enumerate(synthetic_cases()):
 d['rgb_valid']=np.ones(d['depth'].shape,bool)
 if ref is None:
  full=np.zeros(d['depth'].shape,np.uint8);full[210:250,285:345]=255;ref=reference_for(d['rgb'],full,100)
 audit,proposal,refined=prototype(d,ref,1000+index);audit.update(case=name,fixture_origin=origin,neighbor_pixels=int(neighbor.sum()),
  neighbor_fraction_of_reference=float(neighbor.sum()/np.count_nonzero(ref.mask)))
 if refined is not None:
  audit['known_neighbor_output_pixels']=int(np.count_nonzero((refined.mask>0)&neighbor));cv2.imwrite(str(O/f'{name}_refined.png'),refined.mask)
 else:audit['known_neighbor_output_pixels']=0
 audit['known_false_merge_passed_all_original_gates']=bool(audit['outcome']=='candidate_passed_gates_but_not_admitted' and audit['known_neighbor_output_pixels']>0)
 cv2.imwrite(str(O/f'{name}_rgb.png'),d['rgb']);synthetic.append(audit)
 print(json.dumps({k:v for k,v in audit.items() if k in ('case','outcome','reason','rejection_stage','known_neighbor_output_pixels','known_false_merge_passed_all_original_gates','per_member_core_retained_fractions','fixed_template_appearance')},default=serial),flush=True)
assert all(sha(p)==s for p,s in sources.items()) and all(sha(p)==s for p,s in inputs.items())
unsafe=any(row['known_false_merge_passed_all_original_gates'] for row in synthetic)
report=dict(actual_rows=actual,synthetic_rows=synthetic,input_sha256=inputs,source_sha256=sources,input_unchanged=True,source_unchanged=True,
 script_sha256=sha(__file__),classification='unsafe_false_merge_counterexample' if unsafe else 'not_admissible_without_independent_instance_uniqueness_proof',
 production_or_test_files_modified=False,production_admission=False,production_deployed=False,no_ros_network_calls=True,no_motion_commands=True,
 limitations=['Whole-component measurement changes proposal entity and the basis of the original 25mm seed-to-RGB displacement gate; threshold was not changed.',
 'All historical template pixels are confined to final fixed identity matching; no old mask/depth becomes current foreground.',
 'Synthetic same-colour neighbors are known distinct by fixture construction, which the prototype is not given as an oracle.',
 'Passing old geometry/template gates is not a proof of instance uniqueness, current planning precision, 0.8s runtime freshness or execution permission.'])
(O/'result.json').write_text(json.dumps(report,indent=2,default=serial,allow_nan=False)+'\n');print(O/'result.json',sha(O/'result.json'))
