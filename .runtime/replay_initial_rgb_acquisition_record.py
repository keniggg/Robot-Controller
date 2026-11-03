"""Reusable archived-source replay, writes only a project report; no live ROS I/O."""
from pathlib import Path
from collections import Counter
import json,hashlib,time,argparse,sys,importlib.util
from datetime import datetime,timezone
import numpy as np,cv2
from alicia_grasp_modes.tabletop import Config,segment,TargetTracker,_project
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes.acquisition import InitialAcquisition
from alicia_grasp_modes.measured_mask_seeds import confidence_labels
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy,propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_set import select_rgb_interest_set,RgbObservationSetTracker
parser=argparse.ArgumentParser(description="Offline raw-source initial acquisition and RGB-set replay. No ROS connection or motion.")
parser.add_argument('source',type=Path)
parser.add_argument('output',type=Path)
parser.add_argument('--deployment-record',type=Path,required=True)
args=parser.parse_args()
p=args.source.resolve()
WORK=Path(__file__).resolve().parents[1]
archive_mount=Path('/media/zhuyupei/SanStand')
archive_root=archive_mount/'实验记录/Robot-Controller'
if not archive_mount.is_mount() or archive_root not in args.output.resolve().parents:
 raise RuntimeError('SanStand report output required; no system disk fallback')
r=json.loads(p.read_text());cv2.setNumThreads(1);tracker=RgbObservationSetTracker();acq=None;results=[]
for i,row in enumerate(r['rows']):
 source=Path(row['source_file']);assert hashlib.sha256(source.read_bytes()).hexdigest()==row['source_file_sha256']
 with np.load(source,allow_pickle=False) as z:rgb=z['color_bgr'].copy();native=z['depth_raw'].copy();scale=float(z['depth_scale']);T=z['T_base_camera'].copy();intr=z['K'].copy()
 assert hashlib.sha256(rgb.tobytes()).hexdigest()==row['source_rgb_sha256'];assert hashlib.sha256(native.tobytes()).hexdigest()==row['source_depth_sha256']
 cfg=Config(**row['config']);policy=CenterSearchPolicy((.5,.5),radius_fraction=cfg.center_radius_fraction)
 if acq is None:acq=InitialAcquisition(cfg)
 depth=native.astype(np.float32)*scale;valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
 props=propose_rgb_observation_regions(rgb,policy=policy,depth_valid=valid);members=select_rgb_interest_set(props,policy)
 report=tracker.observe(props,policy=policy,source_id=row['source_id'],generation=row['selection']['generation'],stamp_ns=row['source_stamp_ns'],joints_rad=row['q_source'])
 target=props.candidates[0].mask
 comparisons=[]
 for j,m in enumerate(props.candidates[:4]):
  overlap=np.count_nonzero(target&m.mask)
  comparisons.append(dict(index=j,bbox=m.bbox_xywh,score=round(m.score,6),area=m.area_px,iou_to_top=round(overlap/max(1,np.count_nonzero(target|m.mask)),5),fraction_of_member_inside_top=round(overlap/m.area_px,5),fraction_of_top_inside_member=round(overlap/np.count_nonzero(target),5)))
 seg=segment(depth,intr,T,cfg);initial,reason=TargetTracker(cfg).choose(seg,depth.shape)
 initial_index=next((j for j,c in enumerate(seg.candidates) if c is initial),None)
 candidates=[];selected_refined=None;failure=None
 pts=_project(depth,intr);plane=T.T@seg.plane_base;heights=np.einsum('ijk,k->ij',pts,plane[:3])+plane[3]
 for j,c in enumerate(seg.candidates):
  x=dict(index=j,bbox=c.bbox,quality=c.quality,points=c.metrics['valid_points'],rgb_overlap_fraction=np.count_nonzero((c.mask>0)&target)/max(1,np.count_nonzero(c.mask)))
  try:
   _,_,ev=confidence_labels(c.mask>0,pts,depth,heights,plane[:3],cfg,bgr=rgb)
   x['seeds']={k:ev[k] for k in ('original_top_pixels','upper_surface_band_used','trusted_top_pixels','trusted_side_pixels','trusted_core_pixels')}
  except ValueError as exc:x['seed_error']=str(exc)
  try:
   ref=refine_candidate(rgb,depth,intr,T,c,cfg);x['refined_points']=ref.metrics['valid_points'];x['refined_bbox']=ref.bbox
   if j==initial_index:selected_refined=ref
  except ValueError as exc:
   x['refine_error']=str(exc)
   if j==initial_index:failure=str(exc)
  candidates.append(x)
 if initial is None or initial.quality<float(row['source_context']['perception'].get('minimum_geometry_quality',.5)):
  acq.reset();confirmed=False;detail=reason if initial is None else 'insufficient_geometry_quality'
 elif selected_refined is None:acq.reset();confirmed=False;detail='initial_instance_'+str(failure)
 else:confirmed,detail=acq.observe(initial,selected_refined,row['source_stamp_ns'])
 results.append(dict(frame=i,stamp=row['source_stamp_ns'],source_id=row['source_id'],epoch_ns=row['epoch_ns'],selection=row['selection'],q_source=row['q_source'],accepted_before=row['accepted_before'],accepted_after=row['accepted_after'],source_file=str(source),source_file_sha256=row['source_file_sha256'],source_rgb_sha256=row['source_rgb_sha256'],source_depth_sha256=row['source_depth_sha256'],recorded_detector_state=row.get('detector',{}).get('state'),recorded_detector_detail=row.get('detector',{}).get('detail'),recorder_single_region_track=row.get('rgb_track'),rgb_reason=props.reason,proposal_count=len(props.candidates),competing_member_count=len(members),members=[m.bbox_xywh for m in members],top_comparisons=comparisons,set_confirmed=report.confirmed,set_consistent_frames=report.consistent_frames,set_reason=report.reason,tracked_members=[m.bbox_xywh for m in report.regions],seg_candidates=candidates,selected_index=initial_index,initial_confirmed=confirmed,initial_detail=detail,initial_accumulated_frames=len(acq.samples)))
def sha(path):
 return hashlib.sha256(Path(path).read_bytes()).hexdigest()
modules=['src/alicia_grasp_modes/src/alicia_grasp_modes/'+name+'.py' for name in (
 'tabletop','measured_mask_seeds','rgb_instance','acquisition','rgb_observation_region','rgb_observation_set')]
modules.append('src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py')
implementation={name:sha(WORK/name) for name in modules}
deployment=json.loads(args.deployment_record.read_text())
seed='src/alicia_grasp_modes/src/alicia_grasp_modes/measured_mask_seeds.py'
if deployment.get('loaded') is not True or deployment['source_sha256'][seed] != implementation[seed]:
 raise RuntimeError('Current seed does not match the archived successful deployment')
# Existing adapter mocks transport, TF lookup and clocks only. It invokes the
# actual producer .process with original raw arrays and measured TF/brackets.
# Initial state is fresh solely in this offline object; it changes no live epoch.
harness=WORK/'.runtime/replay_thin_producer_current_20261004.py'
spec=importlib.util.spec_from_file_location('offline_producer_transport',harness)
transport=importlib.util.module_from_spec(spec);spec.loader.exec_module(transport)
if any(Config(**row['config']) != Config() for row in r['rows']):
 raise RuntimeError('The reused producer fixture uses default Config; archive differs')
producer=transport.replay(p,True)
summary=dict(frames=len(results),set_confirmed=[x['frame'] for x in results if x['set_confirmed']],
 initial_first_confirmed_frame=next((x['frame'] for x in results if x['initial_confirmed']),None),
 standalone_acquisition_observe_true_frames=[x['frame'] for x in results if x['initial_confirmed']],
 initial_reasons=dict(Counter(x['initial_detail'] for x in results)),
 producer_detected_frames=[x['index'] for x in producer['rows'] if x['detected']],
 producer_ready_frames=[x['index'] for x in producer['rows'] if x['state']=='ready'],
 producer_final_target_lost=producer['last_target_lost'],
 producer_published_interest_ready_frames=[x['index'] for x in producer['rows'] if x['interest_ready']],
 competing_member_counts=[x['competing_member_count'] for x in results],
 recorder_single_region_confirmed_frames=[x['frame'] for x in results if x['recorder_single_region_track'].get('confirmed')])
report=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),source=str(p),source_sha256=sha(p),
 script_sha256=sha(__file__),implementation_sha256=implementation,
 runtime=dict(python=sys.version,numpy=np.__version__,opencv=cv2.__version__,opencv_threads=cv2.getNumThreads()),
 loaded_seed_evidence=dict(deployment_record=str(args.deployment_record.resolve()),
   deployment_record_sha256=sha(args.deployment_record),loaded=deployment['loaded'],
   loaded_seed_sha256=deployment['source_sha256'][seed],replay_seed_matches_loaded=True,
   scope='archived successful deployment receipt; no live process introspection'),
 initial_acquisition_contract=dict(minimum_measured_points=120,required_frames=3,minimum_outline_iou=.85),
 rgb_set_contract=dict(min_mask_iou=.70,max_joint_delta_rad=.004,required_frames=3,max_gap_seconds=1.,
   ambiguity_margin=.12,max_members=4),
 effective_downstream_geometry=dict(min_object_points=80,support_distance_threshold_m=.0035,voxel_size_m=.0015,
   invoked_in_this_replay=False,provenance='previous independent runtime parameter priority audit'),
 archived_config=r['rows'][0]['config'],summary=summary,rows=results,full_producer_replay=producer,
 full_producer_transport_adapter=dict(path=str(harness),sha256=sha(harness)),
 ros_io=False,live_publications=False,robot_commands_sent=False,live_selection_or_lost_lock_modified=False,
 interpretation=['Recorder rgb_track uses old single-region diagnostic, not production RgbObservationSetTracker.',
 'Standalone acquisition observes all frames; full producer stops acquiring once locked and then tracks.',
 'Pure per-frame set tracking bypasses producer scheduling and initial-depth ambiguity resets; its confirmation is not a production ready publication.',
 'Transport adapter uses archived source age and mocked clocks; it is not a wall-time or live freshness qualification.',
 'Archive omits intervening live frames. Passing this finite new-initial-state replay does not establish live acquisition or grasp success.',
 'Target pixel position is diagnostic only; all candidates are generated and chosen by original generic production functions.',
 'RGB hypotheses are image-space interests, not proof that overlapping masks are the same physical object.'])
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
print(json.dumps(dict(output=str(args.output.resolve()),summary=summary,full_producer_rows=producer['rows']),ensure_ascii=False))
