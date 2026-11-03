"""Fresh unselected scene: sealed passive RGBD only, no ROS network or commands."""
from dataclasses import asdict,fields
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib,json,os,sys,time
W=Path(__file__).resolve().parents[1];M=Path('/media/zhuyupei/SanStand')
C=M/'实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z'
P=C/'current_passive_rgbd_r1';O=C/'verification/gen30_current_passive_initial_r1'
assert M.is_mount() and not O.exists() and os.environ['ROS_MASTER_URI']=='http://127.0.0.1:1'
sys.path[:0]=[str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_flexible_grasp_supervisor/src')]
import cv2,numpy as np,rosbag,rospy,tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes import tabletop as table,rgb_instance as rgb,acquisition
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample,fuse_stable_samples
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision import regular_box_inference as box
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def arrsha(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
inputs={}
def read(p):inputs[str(p)]=sha(p);return json.loads(Path(p).read_text())
capture=read(P/'capture.json');parameters=read(C/'current_passive_parameters_r2/parameters.json')
params=parameters['parameters'];cfg=table.Config(**{f.name:params['/unknown_perception'][f.name] for f in fields(table.Config) if f.name in params['/unknown_perception']})
remote=params['/grasp_6d/remote'];perception=params['/perception'];minimum_quality=params['/unknown_perception'].get('minimum_geometry_quality',.5)
bagpath=P/'arrival.bag';inputs[str(bagpath)]=sha(bagpath)
sources={str(m.__file__):sha(m.__file__) for m in (table,rgb,acquisition,box,
 sys.modules['alicia_flexible_grasp.vision.rgbd_snapshot'],sys.modules['alicia_flexible_grasp.vision.object_geometry'])}
topics=('/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw')
frames={};joints=[];cameras=[];buffer=tf2_py.BufferCore(rospy.Duration(60.))
with rosbag.Bag(str(bagpath),'r') as bag:
 for topic,msg,received in bag.read_messages():
  if topic in topics:frames.setdefault(msg.header.stamp.to_nsec(),{})[topic]=(msg,received.to_nsec())
  elif topic in ('/tf','/tf_static'):
   for transform in msg.transforms:
    if topic=='/tf_static':buffer.set_transform_static(transform,'offline_sealed_passive')
    else:buffer.set_transform(transform,'offline_sealed_passive')
  elif topic=='/alicia_d/accepted_joint_states':
   assert msg.header.frame_id=='sdk_measured'
   values=dict(zip(msg.name,msg.position));q=np.array([values['Joint%d'%i] for i in range(1,7)],dtype=float)
   joints.append(dict(source_ns=msg.header.stamp.to_nsec(),received_ns=received.to_nsec(),positions_rad=q))
  elif topic=='/arrival_diagnostic/camera_parameters':cameras.append(json.loads(msg.data))
stamps=sorted(ns for ns,pair in frames.items() if set(pair)==set(topics));assert len(stamps)==capture['pairs'] and len(cameras)==capture['camera_parameter_reads']
joints.sort(key=lambda row:row['source_ns']);cameras.sort(key=lambda row:row['stamp_ns'])
# Deterministic midpoint selection precedes image content/segmentation analysis.
middle=len(stamps)//2;selected=stamps[middle-1:middle+2];assert len(selected)==3
def tm(ns):return rospy.Time(ns//10**9,ns%10**9)
O.mkdir(parents=True);cv2.setNumThreads(1);bridge=CvBridge();rows=[];samples=[];calibrations=[]
identity=TargetTrackIdentity(1,'offline-passive-unselected-diagnostic')
confirmation=acquisition.InitialAcquisition(cfg)
for ns in selected:
 started,cpu=time.perf_counter(),time.process_time();pair=frames[ns]
 before=max((r for r in joints if r['source_ns']<=ns),key=lambda r:r['source_ns'])
 after=min((r for r in joints if r['source_ns']>=ns),key=lambda r:r['source_ns'])
 assert ns-before['source_ns']<200000000 and after['source_ns']-ns<200000000
 assert np.max(np.abs(before['positions_rad']-after['positions_rad']))<=remote['planning_max_joint_delta_rad']
 q=(before['positions_rad']+after['positions_rad'])/2.
 camera=max((r for r in cameras if r['stamp_ns']<=pair[topics[0]][1]),key=lambda r:r['stamp_ns'])['parameters']
 K=np.array([camera[key] for key in ('fx','fy','cx','cy')],dtype=float)
 color=bridge.imgmsg_to_cv2(pair[topics[0]][0],'bgr8');raw=bridge.imgmsg_to_cv2(pair[topics[1]][0],'passthrough')
 scale=1. if pair[topics[1]][0].encoding=='32FC1' else float(camera['depth_scale'])
 depth=raw.astype(np.float32)*scale;assert color.shape[:2]==depth.shape
 tr=buffer.lookup_transform_core('base_link',pair[topics[1]][0].header.frame_id,tm(ns)).transform
 T=quaternion_matrix([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]);T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
 calibrations.append((K,T,scale));valid=rectification_rgb_valid_mask(camera,depth.shape)
 row=dict(source_stamp_ns=ns,camera_configuration_stamp_ns=max(r['stamp_ns'] for r in cameras if r['stamp_ns']<=pair[topics[0]][1]),
  exact_source_headers={topic:dict(stamp_ns=pair[topic][0].header.stamp.to_nsec(),frame_id=pair[topic][0].header.frame_id,encoding=pair[topic][0].encoding) for topic in topics},
  accepted_joint_bracket=dict(before=before,after=after),source_image_shape=list(depth.shape),
  source_depth_valid_fraction=float(np.mean(np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m))),
  array_sha256={key:arrsha(value) for key,value in dict(color_bgr=color,depth_raw=raw,intrinsics=K,T_base_camera_link=T).items()})
 output=dict(color_bgr=color,depth_raw=raw,intrinsics=K,T_base_camera_link=T,depth_scale_m=scale,source_stamp_ns=ns)
 try:
  segmented=table.segment(depth,K,T,cfg)
  row['segmentation']=dict(metrics=segmented.metrics,plane_base=segmented.plane_base,candidates=[dict(index=i,bbox=c.bbox,center_uv=c.center_uv,position_base=c.position_base,extent_m=c.extent_m,quality=c.quality,mask_pixels=int(np.count_nonzero(c.mask)),metrics=c.metrics) for i,c in enumerate(segmented.candidates)])
  initial,reason=table.TargetTracker(cfg).choose(segmented,depth.shape)
  row['initial_center_choice']=dict(reason=reason,candidate_index=next((i for i,c in enumerate(segmented.candidates) if c is initial),None))
  if initial is not None and initial.quality>=minimum_quality:
   refined=(rgb.refine_candidate(color,depth,K,T,initial,cfg,rgb_valid=valid)
    if params['/unknown_perception'].get('rgb_instance_refinement',True) else initial)
   confirmed,reason=confirmation.observe(initial,refined,ns)
   measured=(refined.mask>0)&np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
   row['original_rgb_refinement']=dict(bbox=refined.bbox,quality=refined.quality,metrics=refined.metrics,
    mask_pixels=int(np.count_nonzero(refined.mask)),mask_valid_depth_pixels=int(measured.sum()),
    mask_valid_depth_fraction=float(measured.sum()/np.count_nonzero(refined.mask)),
    initial_three_source_confirmation=dict(confirmed=confirmed,reason=reason))
   output['object_mask']=refined.mask;cv2.imwrite(str(O/f'{ns}_mask.png'),refined.mask)
   samples.append(RgbdSample(color,raw,refined.mask,refined.bbox,None,ns/1e9,pair[topics[0]][0].header.frame_id,q,ns,identity.epoch,identity))
  else:confirmation.reset()
 except ValueError as exc:row['failure']=str(exc);confirmation.reset()
 np.savez_compressed(O/f'{ns}_arrays.npz',**output);cv2.imwrite(str(O/f'{ns}_rgb.png'),color)
 row.update(wall_sec=time.perf_counter()-started,cpu_sec=time.process_time()-cpu);rows.append(row)
 print(json.dumps({key:value for key,value in row.items() if key in ('source_stamp_ns','initial_center_choice','failure','wall_sec')},default=serial),flush=True)
downstream=dict(available_refined_sources=len(samples),required_diagnostic_sources=3,production_planning_snapshot_frames=remote['planning_snapshot_frames'])
if len(samples)==3:
 for K,T,scale in calibrations[1:]:
  np.testing.assert_array_equal(K,calibrations[0][0]);np.testing.assert_allclose(T,calibrations[0][1],rtol=0,atol=1e-12);assert scale==calibrations[0][2]
 K,T,scale=calibrations[-1];intrinsics=NS(fx=K[0],fy=K[1],cx=K[2],cy=K[3],width=depth.shape[1],height=depth.shape[0])
 optical=np.eye(4);optical[:3,:3]=[[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]];Topt=T@optical
 sk=dict(require_mask=True,min_mask_iou=remote['planning_mask_min_iou'],max_centroid_shift_px=remote['planning_mask_max_centroid_shift_px'],
  max_joint_delta_rad=remote['planning_max_joint_delta_rad'],erosion_px=remote['mask_erosion_px'],depth_scale=scale,
  depth_min_m=perception['depth_min_m'],depth_max_m=perception['depth_max_m'],mad_scale=remote['depth_mad_scale'],
  mad_absolute_floor_m=remote['depth_mad_absolute_floor_m'],internal_hole_max_area_px=0)
 snapshot=fuse_stable_samples(samples,**sk)
 downstream.update(snapshot_arguments=sk,snapshot_ok=snapshot.ok,snapshot_failure_code=snapshot.failure_code,snapshot_failure_reason=snapshot.failure_reason)
 if snapshot.ok:
  gg=remote['gripper_geometry']
  gk=dict(support_bbox_expand_ratio=remote.get('support_bbox_expand_ratio',.30),support_distance_threshold_m=remote.get('support_distance_threshold_m',remote.get('target_cloud_support_plane_inlier_distance_m',.004)),
   voxel_size_m=remote['target_cloud_voxel_size_m'],min_support_points=remote.get('geometry_min_support_points',200),min_object_points=remote.get('geometry_min_object_points',remote.get('target_cloud_min_points',120)),
   min_size_m=remote.get('geometry_min_size_m',.005),max_size_m=remote.get('geometry_max_size_m',.600),max_height_m=remote.get('geometry_max_height_m',.500),previous_axes_base=None,
   outlier_neighbors=remote['target_cloud_outlier_neighbors'],outlier_std_ratio=remote['target_cloud_outlier_std_ratio'],
   support_minimum_span_m=max(gg['finger_box_xyz_m'][0],gg['max_inner_gap_m']+2*gg['finger_box_xyz_m'][1])+2*gg.get('support_clearance_m',.003))
  geometry=estimate_object_geometry(snapshot.depth_raw,snapshot.target_depth_raw,snapshot.object_mask,snapshot.bbox,intrinsics,scale,Topt,snapshot.source_mode,**gk)
  downstream.update(geometry_arguments=gk,geometry_ok=geometry.ok,geometry_failure_code=geometry.failure_code,geometry_failure_reason=geometry.failure_reason)
  if geometry.ok:
   qualified=box.qualify_regular_box_snapshot(snapshot,intrinsics,scale,Topt,geometry,identity,selected[-1])
   downstream.update(geometry_center_base=geometry.center_base,geometry_axes_base=geometry.axes_base,geometry_size_m=geometry.size_xyz_m,
    object_cloud_points=len(geometry.object_points_base),box_ok=qualified.ok,box_code=qualified.code,box_audit=dict(qualified.audit),
    box_model=qualified.model.to_evidence_dict() if qualified.ok else None)
assert all(sha(p)==s for p,s in sources.items()) and all(sha(p)==s for p,s in inputs.items())
report=dict(rows=rows,downstream=downstream,source_sha256=sources,input_sha256=inputs,source_unchanged=True,input_unchanged=True,
 script_sha256=sha(__file__),selected_source_stamps_ns=selected,source_selection='deterministic three consecutive midpoint sources before looking at image/segmentation',
 effective_tabletop_config=asdict(cfg),parameter_capture_after_bag=True,camera_snapshots_in_bag=len(cameras),
 runtime_target_identity_used=False,runtime_target_selected=False,diagnostic_identity=asdict(identity),historical_anchor_used=False,
 no_ros_network_calls=True,no_motion_commands=True,no_parameter_writes=True,execution_authority=False,
 limitations=['Whole runtime parameter snapshot was captured after the passive bag; only camera was captured inside the bag.',
 'Three-source offline initial acquisition/geometry diagnosis is not the live five-source planning request or a fresh clear-workspace check.',
 'Current runtime generation is lost; offline image center selection cannot clear it, identify its target or authorize execution.',
 'Images include hands at the keyboard; this report contains no clearance or safety admission.'])
(O/'result.json').write_text(json.dumps(report,indent=2,default=serial,allow_nan=False)+'\n');print(json.dumps(downstream,default=serial));print(O/'result.json',sha(O/'result.json'))
