"""Bounded three-source producer-helper check, without ROS or motion authority."""
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib,json,os,sys,time
W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand');N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
I=N/'gen29_attempt1_shadow_frames';O=N/'gen30_current_fragment_owner_r3'
assert M.is_mount() and not O.exists() and os.environ['ROS_MASTER_URI']=='http://127.0.0.1:1'
sys.path[:0]=[str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_flexible_grasp_supervisor/src')]
import cv2,numpy as np
from alicia_grasp_modes import tabletop as table,rgb_instance as rgb
from alicia_grasp_modes import current_fragment_owner as owner
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample,fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_snapshot
from alicia_flexible_grasp.vision import current_instance_boundary as boundary_consumer
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
inputs={}
def read(p):inputs[str(p)]=sha(p);return json.loads(Path(p).read_text())
meta=read(I/'mask_contract_review_r2/result.json');cfg=table.Config(**meta['effective_config'])
plan=read(N/'gen30_initial_view_box_replay_r1/result.json')
modules=[table,rgb,owner,boundary_consumer,sys.modules['alicia_grasp_modes.measured_mask_seeds'],
 sys.modules['alicia_flexible_grasp.vision.rgbd_snapshot'],sys.modules['alicia_flexible_grasp.vision.object_geometry'],
 sys.modules['alicia_flexible_grasp.vision.regular_box_inference']]
sources={str(m.__file__):sha(m.__file__) for m in modules}
optical=np.eye(4);optical[:3,:3]=[[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]]
cv2.setNumThreads(1);O.mkdir();rows=[]
for prior in meta['rows'][1:]:
 ns=prior['source_stamp_ns'];p=I/f'source_{ns}'
 status=read(p/'detector_status.json')['payload'];camera=read(p/'metadata.json')['camera_parameters']
 inputs[str(p/'raw_arrays.npz')]=sha(p/'raw_arrays.npz');a=dict(np.load(p/'raw_arrays.npz',allow_pickle=False))
 depth=a['depth_raw'].astype(np.float32)*float(a['depth_scale_m']);K=a['intrinsics'];T=a['T_base_camera_link'];color=a['color_bgr']
 anchor=NS(position_base=np.asarray(status['anchor_position_base']),extent_m=np.asarray(status['anchor_extent_m']),support_footprint_base=np.asarray(status['reference_support_footprint_base']),metrics={'median_height_m':status['reference_object_height_m']})
 last=NS(position_base=np.asarray(prior['prior_position_used']));locked=np.asarray(status['reference_support_plane_base'])
 result=table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,locked,prefer_current_support=True)
 spatial=rgb.freeze_observation_spatial_reference(anchor,locked);valid=rectification_rgb_valid_mask(camera,depth.shape)
 primary=table._matching_candidates(result.candidates,anchor,last,cfg)[0][1]
 t,c=time.perf_counter(),time.process_time()
 refined=rgb.refine_candidate(color,depth,K,T,primary,cfg,rgb_valid=valid,observation_arrival_retry=True,observation_spatial_reference=spatial)
 primary_time=dict(wall_sec=time.perf_counter()-t,cpu_sec=time.process_time()-c)
 t,c=time.perf_counter(),time.process_time();checks=[0]
 def check():checks[0]+=1
 row=dict(source_stamp_ns=ns,primary_time=primary_time)
 try:
  recovered=owner.recover_current_fragment_owner(result,color,depth,K,T,cfg,anchor,last,spatial,refined,.5,rgb_valid=valid,check_current=check)
  row['helper_time']=dict(wall_sec=time.perf_counter()-t,cpu_sec=time.process_time()-c)
  row['check_calls']=checks[0]
  if recovered is None:row['outcome']='unchanged'
  else:
   request=status['observation_measurement']['request']
   identity={'epoch':request['target_epoch'],'track_id':request['target_track_id']}
   if 'station_compute_wait' in status:
    binding=status['station_compute_wait']['binding']
    assert identity=={'epoch':binding['target_epoch'],'track_id':binding['target_track_id']}
   boundary=owner.build_current_instance_boundary(recovered,source_stamp_ns=ns,target_identity=identity,rgb=color,depth_raw=a['depth_raw'],depth_scale_m=float(a['depth_scale_m']),intrinsics=K,transform=T,support_plane=result.plane_base,config=cfg)
   row.update(outcome='recovered',audit=recovered.audit,boundary=boundary)
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
     consumed=boundary_consumer.validate_current_instance_boundary(boundary,original=snapshot.original_sample,
      intrinsics=K,depth_scale_m=float(a['depth_scale_m']),T_base_camera=T@optical,support_plane_base=result.plane_base,
      geometry=geometry,inferred_box=qualified.model if qualified.ok else None)
     row['consumer_result']=dict(ok=consumed.ok,failure_code=consumed.failure_code,failure_reason=consumed.failure_reason,audit=dict(consumed.audit))
 except ValueError as exc:row.update(outcome='rejected',error=str(exc),helper_time=dict(wall_sec=time.perf_counter()-t,cpu_sec=time.process_time()-c),check_calls=checks[0])
 rows.append(row);print(json.dumps({k:v for k,v in row.items() if k not in ('audit','boundary')},default=serial),flush=True)
assert all(sha(p)==s for p,s in sources.items()) and all(sha(p)==s for p,s in inputs.items())
report=dict(rows=rows,input_sha256=inputs,source_sha256=sources,source_unchanged=True,input_unchanged=True,script_sha256=sha(__file__),
 no_ros_calls=True,no_hardware_access=True,production_helper_tested=True,production_deployed=False,new_restart_runs_only_gen29=True,execution_authority=False,
 limitations=['Finite recorded three-source helper check with previously documented prior-center proxies, not async producer/runtime proof.',
 'Deadline callback counts source checks but no execution freshness is granted.',
 'Single-source fusion counterfactual with dummy equal joints, not a real three-frame runtime admission.',
 'Boundary producer contract actually consumed against original raw data and unchanged geometry/box qualifiers; no opening, collision or closure permission granted.'])
(O/'result.json').write_text(json.dumps(report,indent=2,default=serial,allow_nan=False)+'\n');print(O/'result.json',sha(O/'result.json'))
