"""Bounded three-source producer-helper check, without ROS or motion authority."""
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib,json,os,sys,time
W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand');N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
I=N/'gen29_attempt1_shadow_frames';O=N/'gen30_source_connected_owner_r3'
assert M.is_mount() and not O.exists() and os.environ['ROS_MASTER_URI']=='http://127.0.0.1:1'
sys.path[:0]=[str(W/'.runtime/realsense-2.56.5'),str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_flexible_grasp_supervisor/src')]
import cv2,numpy as np
from alicia_grasp_modes import tabletop as table,rgb_instance as rgb
from alicia_grasp_modes import source_connected_owner as owner
from alicia_grasp_modes.source_sampling_topology import SourceSamplingTopology
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance,match_observation_appearance
from alicia_grasp_modes.rgb_observation_handoff import _appearance
from alicia_grasp_modes.rgb_observation_source import require_fresh_source
from alicia_grasp_modes.station_compute_reacquisition import validate_contract
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample,fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_snapshot
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
inputs={}
def read(p):inputs[str(p)]=sha(p);return json.loads(Path(p).read_text())
meta=read(I/'mask_contract_review_r2/result.json');cfg=table.Config(**meta['effective_config'])
plan=read(N/'gen30_initial_view_box_replay_r1/result.json')
modules=[table,rgb,owner,sys.modules['alicia_grasp_modes.source_sampling_topology'],sys.modules['alicia_grasp_modes.measured_mask_seeds'],
 sys.modules['alicia_flexible_grasp.vision.rgbd_snapshot'],sys.modules['alicia_flexible_grasp.vision.object_geometry'],
 sys.modules['alicia_flexible_grasp.vision.regular_box_inference']]
sources={str(m.__file__):sha(m.__file__) for m in modules}
optical=np.eye(4);optical[:3,:3]=[[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]]
cv2.setNumThreads(1);O.mkdir();rows=[]
qns=meta['qualification_template_source_ns'];qdir=I/f'source_{qns}'
qmeta=read(qdir/'metadata.json');inputs[str(qdir/'raw_arrays.npz')]=sha(qdir/'raw_arrays.npz');q=dict(np.load(qdir/'raw_arrays.npz',allow_pickle=False))
qvalid=rectification_rgb_valid_mask(qmeta['camera_parameters'],q['depth_raw'].shape);hist,contour=_appearance(q['color_bgr'],q['object_mask'])
reference=FrozenObservationAppearance(q['color_bgr'],q['object_mask'],qvalid,qns,hist,contour)
wall,cpu=time.perf_counter(),time.process_time();topology=SourceSamplingTopology(qmeta['camera_parameters'])
topology_preparation_time=dict(wall_sec=time.perf_counter()-wall,cpu_sec=time.process_time()-cpu)

for prior in meta['rows'][1:]:
 ns=prior['source_stamp_ns'];p=I/f'source_{ns}'
 status=read(p/'detector_status.json')['payload'];camera=read(p/'metadata.json')['camera_parameters']
 inputs[str(p/'raw_arrays.npz')]=sha(p/'raw_arrays.npz');a=dict(np.load(p/'raw_arrays.npz',allow_pickle=False))
 depth=a['depth_raw'].astype(np.float32)*float(a['depth_scale_m']);K=a['intrinsics'];T=a['T_base_camera_link'];color=a['color_bgr']
 anchor=NS(position_base=np.asarray(status['anchor_position_base']),extent_m=np.asarray(status['anchor_extent_m']),support_footprint_base=np.asarray(status['reference_support_footprint_base']),metrics={'median_height_m':status['reference_object_height_m']})
 last=NS(position_base=np.asarray(prior['prior_position_used']));locked=np.asarray(status['reference_support_plane_base'])
 process_wall,process_cpu=time.perf_counter(),time.process_time()
 result=table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,locked,prefer_current_support=True)
 segmentation_time=dict(wall_sec=time.perf_counter()-process_wall,cpu_sec=time.process_time()-process_cpu)
 spatial=rgb.freeze_observation_spatial_reference(anchor,locked);valid=rectification_rgb_valid_mask(camera,depth.shape)
 primary=table._matching_candidates(result.candidates,anchor,last,cfg)[0][1]
 t,c=time.perf_counter(),time.process_time()
 refined=rgb.refine_candidate(color,depth,K,T,primary,cfg,rgb_valid=valid,observation_arrival_retry=True,observation_spatial_reference=spatial)
 primary_time=dict(wall_sec=time.perf_counter()-t,cpu_sec=time.process_time()-c)
 t,c=time.perf_counter(),time.process_time();checks=[0]
 original_entry_age=float(status['frame_timing']['processing_start_source_age_sec'])
 contract=status.get('station_compute_wait')
 contract_origin='same_source_ready_status'
 if contract is None:
  previous_ns=status['frame_timing']['arrival_frame_completion']['reference_source_stamp_ns']
  previous_status=read(I/f'source_{previous_ns}'/'detector_status.json')['payload']
  contract=previous_status['station_compute_wait']
  assert contract['source_ns']==previous_ns and previous_ns<ns
  contract_origin='unchanged_prior_registered_ready_status; current entry reference_source_stamp matches'
 binding=contract['binding']
 request=status['observation_measurement']['request']
 assert binding['selection']==request['selection'] and status['generation']==binding['selection']['generation']
 assert status['selection_stamp_ns']==binding['selection']['stamp_ns']
 assert binding['target_epoch']==request['target_epoch'] and binding['target_track_id']==request['target_track_id']
 assert request['source_stamps_ns'][-1]==reference.source_ns
 def check():
  checks[0]+=1
  virtual_now=ns+round((original_entry_age+time.perf_counter()-process_wall)*1e9)
  require_fresh_source(ns,virtual_now,.8)
  validate_contract(contract,binding=binding,now_ns=virtual_now,source_ns=ns if contract['source_ns']<ns else None)
 context=dict(reference=reference,reference_role='qualified_station_identity_template',source_ns=ns,same_station_mask_iou_minimum=.85,check_current=check)
 row=dict(source_stamp_ns=ns,primary_time=primary_time,segmentation_time=segmentation_time,original_entry_age_sec=original_entry_age,registration_origin=contract_origin,registration=contract)
 stages=[]
 def stage(name):
  stages.append(dict(stage=name,elapsed_wall_sec=time.perf_counter()-process_wall,elapsed_cpu_sec=time.process_time()-process_cpu))
 stage('primary_appearance')

 try:
  original=match_observation_appearance(color,refined.mask,reference.histogram,reference.contour,rgb_valid=valid,appearance_context=context)
  row['original_primary_appearance']=original
  assert original['code']=='SAME_STATION_COMPLETE_OUTLINE_MISMATCH'
  t,c=time.perf_counter(),time.process_time()
  recovered=owner.recover_source_connected_fragment_owner(result,color,depth,K,T,cfg,anchor,last,spatial,refined,.5,source_topology=topology,camera=camera,source_stamp_ns=ns,rgb_valid=valid,check_current=check,record_stage=stage)
  row['helper_time']=dict(wall_sec=time.perf_counter()-t,cpu_sec=time.process_time()-c)
  row['check_calls']=checks[0]
  if recovered is None:row['outcome']='unchanged'
  else:
   stage('final_template')
   appearance=match_observation_appearance(color,recovered.owner.mask,reference.histogram,reference.contour,rgb_valid=valid,appearance_context=context)
   row['final_appearance']=appearance
   assert appearance['appearance_match']
   check()
   request=status['observation_measurement']['request']
   identity={'epoch':request['target_epoch'],'track_id':request['target_track_id']}
   assert identity=={'epoch':binding['target_epoch'],'track_id':binding['target_track_id']}
   check()
   stage('perception_complete')
   row['complete_perception_compute_time']=dict(wall_sec=time.perf_counter()-process_wall,cpu_sec=time.process_time()-process_cpu)
   row['modeled_source_age_at_complete_sec']=original_entry_age+time.perf_counter()-process_wall
   row.update(outcome='recovered',audit=recovered.audit,boundary_contract_used=False)
   cv2.imwrite(str(O/f'{ns}_owner.png'),recovered.owner.mask)
   yy,xx=np.nonzero(recovered.owner.mask);bbox=(int(xx.min()),int(yy.min()),int(np.ptp(xx)+1),int(np.ptp(yy)+1))
   target_identity=TargetTrackIdentity(**identity)
   sample=RgbdSample(color,a['depth_raw'],recovered.owner.mask,bbox,None,ns/1e9,'camera_link',np.zeros(6),ns,identity['epoch'],target_identity)
   sk=dict(plan['snapshot_arguments']);sk['depth_scale']=float(a['depth_scale_m'])
   snapshot=fuse_stable_samples([sample],**sk)
   row.update(single_source_counterfactual_snapshot_ok=snapshot.ok)
   if snapshot.ok:
    camera_K=NS(fx=K[0],fy=K[1],cx=K[2],cy=K[3],width=depth.shape[1],height=depth.shape[0])
    geometry=estimate_object_geometry(snapshot.depth_raw,snapshot.target_depth_raw,snapshot.object_mask,snapshot.bbox,
     camera_K,float(a['depth_scale_m']),T@optical,snapshot.source_mode,**plan['geometry_arguments'])
    row.update(geometry_ok=geometry.ok,geometry_failure=geometry.failure_code)
    if geometry.ok:
     qualified=qualify_regular_box_snapshot(snapshot,camera_K,float(a['depth_scale_m']),T@optical,geometry,target_identity,ns)
     row.update(box_ok=qualified.ok,box_code=qualified.code)

 except ValueError as exc:
  stage('rejected')
  row.update(outcome='rejected',error=str(exc),helper_time=dict(wall_sec=time.perf_counter()-t,cpu_sec=time.process_time()-c),check_calls=checks[0])
 row['computation_stages']=stages
 (O/f'{ns}_row.json').write_text(json.dumps(row,indent=2,default=serial,allow_nan=False)+'\n')
 rows.append(row);print(json.dumps({k:v for k,v in row.items() if k not in ('audit','boundary','registration','computation_stages')},default=serial),flush=True)
assert all(sha(p)==s for p,s in sources.items()) and all(sha(p)==s for p,s in inputs.items())
report=dict(environment={k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','PYTHONNOUSERSITE')},sdk_offline_path_explicit=True,currently_running_unknown_sdk_available=False,topology_preparation_time=topology_preparation_time,rows=rows,input_sha256=inputs,source_sha256=sources,source_unchanged=True,input_unchanged=True,script_sha256=sha(__file__),
 no_ros_calls=True,no_hardware_access=True,production_helper_tested=True,production_deployed=False,new_restart_runs_only_gen29=True,execution_authority=False,
 limitations=['Actual original primary appearance mismatch and final template checks, frozen station/generation/source contracts validated offline; no live robot state checked. Finite recorded three-source helper check with previously documented prior-center proxies, not async producer/runtime proof.',
 'Offline virtual source-age checks include recorded ingress age plus measured segmentation, primary refinement, new helper and template computation; exclude ROS scheduling/decode/TF/pub and do not establish runtime freshness or execution authority.',
 'Single-source fusion counterfactual with dummy equal joints, not a real three-frame runtime admission.',
 'Source-topology path has no partial-outline tolerance and no extra boundary contract; original raw depth/geometry/box qualifiers were evaluated, no opening/collision/closure authority is granted.'])
(O/'result.json').write_text(json.dumps(report,indent=2,default=serial,allow_nan=False)+'\n');print(O/'result.json',sha(O/'result.json'))
