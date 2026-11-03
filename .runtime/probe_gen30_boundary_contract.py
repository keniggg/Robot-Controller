"""Three retained source contracts, plus the excluded fragments' RGB counterevidence.

The current helper rejects these sources. The earlier independent side owner
is used only to exercise source binding; this script grants no new admission.
"""
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib, json, os, sys, time
W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
I=N/'gen29_attempt1_shadow_frames';O=N/'gen30_boundary_contract_r1'
assert M.is_mount() and not O.exists() and os.environ['ROS_MASTER_URI']=='http://127.0.0.1:1'
sys.path[:0]=[str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_flexible_grasp_supervisor/src')]
import cv2, numpy as np
from alicia_grasp_modes import tabletop as table, rgb_instance as rgb, current_fragment_owner as owner
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_snapshot
from alicia_flexible_grasp.vision import current_instance_boundary as consumer
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
inputs={}
def read(p):inputs[str(p)]=sha(p);return json.loads(Path(p).read_text())
meta=read(I/'mask_contract_review_r2/result.json');cfg=table.Config(**meta['effective_config'])
plan=read(N/'gen30_initial_view_box_replay_r1/result.json')
prior_results=read(N/'gen30_current_fragment_owner_r2/result.json')
sources={str(m.__file__):sha(m.__file__) for m in (table,rgb,owner,consumer,
 sys.modules['alicia_flexible_grasp.vision.rgbd_snapshot'],sys.modules['alicia_flexible_grasp.vision.object_geometry'],
 sys.modules['alicia_flexible_grasp.vision.regular_box_inference'])}
optical=np.eye(4);optical[:3,:3]=[[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]]
cv2.setNumThreads(1);O.mkdir();rows=[]
for prior,old_result in zip(meta['rows'][1:],prior_results['rows']):
 ns=prior['source_stamp_ns'];assert ns==old_result['source_stamp_ns'];p=I/f'source_{ns}'
 status=read(p/'detector_status.json')['payload'];camera=read(p/'metadata.json')['camera_parameters']
 inputs[str(p/'raw_arrays.npz')]=sha(p/'raw_arrays.npz');a=dict(np.load(p/'raw_arrays.npz',allow_pickle=False))
 depth=a['depth_raw'].astype(np.float32)*float(a['depth_scale_m']);K=a['intrinsics'];T=a['T_base_camera_link'];color=a['color_bgr']
 anchor=NS(position_base=np.asarray(status['anchor_position_base']),extent_m=np.asarray(status['anchor_extent_m']),support_footprint_base=np.asarray(status['reference_support_footprint_base']),metrics={'median_height_m':status['reference_object_height_m']})
 last=NS(position_base=np.asarray(prior['prior_position_used']));locked=np.asarray(status['reference_support_plane_base'])
 result=table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,locked,prefer_current_support=True)
 spatial=rgb.freeze_observation_spatial_reference(anchor,locked);valid=rectification_rgb_valid_mask(camera,depth.shape)
 frame=rgb._FrameRefinement(depth,K,T,spatial,cfg)
 t,c=time.perf_counter(),time.process_time()
 independent=frame.refine(color,result.candidates[0],rgb_valid=valid)
 path=N/'gen30_current_fragment_owner_r2'/f'{ns}_owner.png';inputs[str(path)]=sha(path)
 assert np.array_equal(independent.mask,cv2.imread(str(path),0))
 plane_camera=T.T@result.plane_base
 heights=np.einsum('ijk,k->ij',frame.points,plane_camera[:3])+plane_camera[3]
 far=[]
 for index,candidate in enumerate(result.candidates):
  distance=float(np.linalg.norm(candidate.position_base-anchor.position_base))
  if distance<=cfg.association_distance_m:continue
  mask,evidence=owner._independent_identity_outline(frame,color,candidate,plane_camera,heights,valid)
  full=independent.mask>0;outside=(mask>0)&~full
  interior=cv2.erode((mask>0).astype(np.uint8),np.ones((3,3),np.uint8))>0
  band=cv2.dilate(full.astype(np.uint8),np.ones((3,3),np.uint8))>0
  measured=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
  yy,xx=np.nonzero(outside)
  try:
   owner._boundary_identity(mask,independent.mask);failure=None
  except ValueError as exc:failure=str(exc)
  far.append(dict(candidate_index=index,anchor_distance_m=distance,rgb_mask_pixels=int(np.count_nonzero(mask)),
   outside_owner_pixels=int(outside.sum()),interior_outside_owner_pixels=int(np.count_nonzero(interior&~full)),
   outside_one_pixel_band=int(np.count_nonzero((mask>0)&~band)),
   outside_measured_foreground=int(np.count_nonzero(outside&measured&(heights>=cfg.foreground_min_m))),
   outside_measured_low_or_support=int(np.count_nonzero(outside&measured&(heights<cfg.foreground_min_m))),
   outside_unknown_depth=int(np.count_nonzero(outside&~measured)),
   outside_bbox_xyxy=None if not len(xx) else [int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())],
   identity_failure=failure,rgb_evidence=evidence))
  cv2.imwrite(str(O/f'{ns}_far_{index}.png'),mask)
 boundary_mask=np.zeros(depth.shape,np.uint8)
 pixels=np.asarray(old_result['boundary']['boundary_pixels_yx'],dtype=np.int64)
 boundary_mask[pixels[:,0],pixels[:,1]]=1
 recovered=owner.CurrentFragmentOwner(independent,boundary_mask,{'not_admitted_by_current_helper':True})
 identity=old_result['boundary']['target_identity']
 boundary=owner.build_current_instance_boundary(recovered,source_stamp_ns=ns,target_identity=identity,rgb=color,
  depth_raw=a['depth_raw'],depth_scale_m=float(a['depth_scale_m']),intrinsics=K,transform=T,support_plane=result.plane_base,config=cfg)
 assert boundary==old_result['boundary']
 yy,xx=np.nonzero(independent.mask);bbox=(int(xx.min()),int(yy.min()),int(np.ptp(xx)+1),int(np.ptp(yy)+1))
 target_identity=TargetTrackIdentity(**identity)
 sample=RgbdSample(color,a['depth_raw'],independent.mask,bbox,None,ns/1e9,'camera_link',np.zeros(6),ns,identity['epoch'],target_identity)
 sk=dict(plan['snapshot_arguments']);sk['depth_scale']=float(a['depth_scale_m'])
 snapshot=fuse_stable_samples([sample],**sk);assert snapshot.ok
 camera_K=NS(fx=K[0],fy=K[1],cx=K[2],cy=K[3],width=depth.shape[1],height=depth.shape[0])
 geometry=estimate_object_geometry(snapshot.depth_raw,snapshot.target_depth_raw,snapshot.object_mask,snapshot.bbox,
  camera_K,float(a['depth_scale_m']),T@optical,snapshot.source_mode,**plan['geometry_arguments']);assert geometry.ok
 qualified=qualify_regular_box_snapshot(snapshot,camera_K,float(a['depth_scale_m']),T@optical,geometry,target_identity,ns)
 consumed=consumer.validate_current_instance_boundary(boundary,original=snapshot.original_sample,intrinsics=K,
  depth_scale_m=float(a['depth_scale_m']),T_base_camera=T@optical,support_plane_base=result.plane_base,geometry=geometry,
  inferred_box=qualified.model if qualified.ok else None)
 row=dict(source_stamp_ns=ns,far_fragment_counterevidence=far,box_ok=qualified.ok,box_code=qualified.code,
  consumer_result=dict(ok=consumed.ok,failure_code=consumed.failure_code,failure_reason=consumed.failure_reason,audit=dict(consumed.audit)),
  boundary_rebuilt_exactly=True,producer_contract=boundary,current_helper_admission=False,
  wall_sec=time.perf_counter()-t,cpu_sec=time.process_time()-c)
 rows.append(row);print(json.dumps({k:v for k,v in row.items() if k not in ('far_fragment_counterevidence','producer_contract')},default=serial),flush=True)
assert all(sha(p)==s for p,s in sources.items()) and all(sha(p)==s for p,s in inputs.items())
report=dict(rows=rows,input_sha256=inputs,source_sha256=sources,source_unchanged=True,input_unchanged=True,script_sha256=sha(__file__),
 no_ros_calls=True,no_hardware_access=True,new_restart_runs_only_gen29=True,production_deployed=False,execution_authority=False,
 limitations=['Current producer rejects all three sources; contract checks exercise an earlier counterfactual owner only.',
 'Original three-frame source/prior proxies remain as documented; one-source fusion uses dummy equal joints and is not runtime admission.',
 'No geometric budget, template threshold, original expiry or measured pixel was changed to obtain these results.'])
(O/'result.json').write_text(json.dumps(report,indent=2,default=serial,allow_nan=False)+'\n');print(O/'result.json',sha(O/'result.json'))
