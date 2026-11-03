#!/usr/bin/env python3
"""Sealed-frame registration reconstruction; no ROS node, service or actions."""
from pathlib import Path
import json,hashlib,dataclasses
from types import SimpleNamespace as NS
import numpy as np
import rosbag,rospy,tf2_py,yaml
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample,fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision import multiview_surface as mv
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD=ROOT/'grasp_records/20261009T022026.944578Z_cca56e3000b84b5289c1adfa0e09a9bb'
OUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/gen7_registration_analysis'
AUDIT=RECORD/'keyframes_audits/e46041ada83fbbbde2bc82fc1dd30952e13ea3913309111df8234bf27c0e81be.json'
def convert(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 if dataclasses.is_dataclass(x):return {f.name:convert(getattr(x,f.name)) for f in dataclasses.fields(x)}
 if isinstance(x,dict):return {k:convert(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [convert(v) for v in x]
 return x

def main():
 audit=json.loads(AUDIT.read_text()); rg=audit['replay_geometry'];refns=audit['snapshot_stamp_ns']
 refstamps=[1791512429454794406,1791512429654921293,1791512429854358673]
 cameras=[];phases=[];static=[];dynamic=[];joints=[]
 with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
  for topic,msg,t in bag.read_messages(topics=['/tf','/tf_static','/grasp/near_field_phase','/grasp_record/camera_runtime_metadata','/alicia_d/accepted_joint_states']):
   if topic=='/tf':dynamic.extend(msg.transforms)
   elif topic=='/tf_static':static.extend(msg.transforms)
   elif topic.endswith('near_field_phase') and msg.active:phases.append(msg)
   elif topic.endswith('camera_runtime_metadata'):cameras.append((t.to_nsec(),json.loads(msg.data)['parameters']))
   elif topic.endswith('accepted_joint_states') and msg.header.frame_id=='sdk_measured':joints.append(msg)
 assert len(phases)==1
 moving_requested=phases[0].reference_source_stamp.to_nsec()
 frames={};object_stamps=[]
 with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
  for topic,msg,t in bag.read_messages(topics=['/perception/object','/perception/object_mask','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']):
   ns=msg.header.stamp.to_nsec()
   if topic=='/perception/object':object_stamps.append(ns)
   if ns in refstamps or abs(ns-moving_requested)<1000:
    frames.setdefault(ns,{})[topic]=msg
 moving_candidates=[ns for ns in frames if abs(ns-moving_requested)<1000];assert len(moving_candidates)==1,(moving_requested,moving_candidates)
 moving_ns=moving_candidates[0]
 buffer=tf2_py.BufferCore(rospy.Duration(600.))
 for t in static:buffer.set_transform_static(t,'sealed_registration_replay')
 for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buffer.set_transform(t,'sealed_registration_replay')
 def T(ns):
  t=buffer.lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)).transform
  m=quaternion_matrix([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]);m[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
  opt=np.eye(4);opt[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
  return m@opt
 identity=TargetTrackIdentity(4,'g1-t4');bridge=CvBridge()
 def sample(ns):
  f=frames[ns];obj=f['/perception/object'];dep=f['/supervisor/camera/depth/image_raw'];q=min(joints,key=lambda j:abs(j.header.stamp.to_nsec()-ns))
  return RgbdSample(bridge.imgmsg_to_cv2(f['/supervisor/camera/color/image_raw'],'bgr8'),bridge.imgmsg_to_cv2(dep,'passthrough'),bridge.imgmsg_to_cv2(f['/perception/object_mask'],'mono8'),(obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height),obj,ns/1e9,dep.header.frame_id,np.asarray(q.position[:6]),ns,4,identity)
 cfg=yaml.safe_load(Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())['grasp_6d']['remote']
 g=cfg.get('gripper_geometry',{}); fingers=g.get('finger_box_xyz_m',[.0434,.0286,.0600])
 geom=dict(support_bbox_expand_ratio=cfg.get('support_bbox_expand_ratio',.30),support_minimum_span_m=max(fingers[0],g.get('max_inner_gap_m',.05)+2*fingers[1])+2*g.get('support_clearance_m',.003),support_distance_threshold_m=cfg.get('support_distance_threshold_m',cfg.get('target_cloud_support_plane_inlier_distance_m',.004)),voxel_size_m=cfg.get('target_cloud_voxel_size_m',.0025),min_support_points=cfg.get('geometry_min_support_points',200),min_object_points=cfg.get('geometry_min_object_points',cfg.get('target_cloud_min_points',120)),min_size_m=cfg.get('geometry_min_size_m',.005),max_size_m=cfg.get('geometry_max_size_m',.6),max_height_m=cfg.get('geometry_max_height_m',.5),outlier_neighbors=cfg.get('target_cloud_outlier_neighbors',16),outlier_std_ratio=cfg.get('target_cloud_outlier_std_ratio',2.),previous_axes_base=None)
 def estimate(stamps):
  camera=min(cameras,key=lambda it:abs(it[0]-stamps[-1]))[1];scale=camera['depth_scale']
  fusion=dict(require_mask=True,min_mask_iou=cfg.get('planning_mask_min_iou',.85),max_centroid_shift_px=cfg.get('planning_mask_max_centroid_shift_px',5.),max_joint_delta_rad=cfg.get('planning_max_joint_delta_rad',.01),erosion_px=cfg.get('mask_erosion_px',2),depth_scale=scale,depth_min_m=cfg.get('depth_min_m',.1),depth_max_m=cfg.get('depth_max_m',2.),mad_scale=cfg.get('depth_mad_scale',3.5),mad_absolute_floor_m=cfg.get('depth_mad_absolute_floor_m',.002),internal_hole_max_area_px=cfg.get('mask_internal_hole_max_area_px',25))
  snap=fuse_stable_samples([sample(ns) for ns in stamps],**fusion);assert snap.ok,snap.failure_reason
  K=NS(**{k:camera[k] for k in ('fx','fy','cx','cy','width','height')});transform=T(stamps[-1])
  est=estimate_object_geometry(snap.depth_raw,snap.target_depth_raw,snap.object_mask,snap.bbox,K,scale,transform,snap.source_mode,**geom);assert est.ok,est.failure_reason
  x,y,w,h=snap.bbox;clearance=min(x,y,K.width-x-w,K.height-y-h)
  view=mv.SurfaceView(identity,stamps[-1],est.object_points_base,est.support_normal_base,est.support_offset_m,clearance)
  return est,view,snap,transform,fusion
 ref_est,ref_rebuilt,ref_snap,ref_T,ref_fusion=estimate(refstamps)
 reference=mv.SurfaceView(identity,refns,np.asarray(rg['object_points_base_m']),np.asarray(rg['support_normal_base']),rg['support_offset_m'],ref_rebuilt.edge_clearance_px)
 moving_est,moving,moving_snap,moving_T,moving_fusion=estimate([moving_ns])
 config=mv.RegistrationConfig(**cfg['multiview']);res=mv.register_surface_view(reference,moving,config)
 exact=np.array_equal(reference.points_base,ref_rebuilt.points_base)
 rebuilt_delta=None if reference.points_base.shape!=ref_rebuilt.points_base.shape else float(np.max(np.abs(reference.points_base-ref_rebuilt.points_base)))
 report=dict(record=str(RECORD),source_audit=str(AUDIT),source_audit_sha256=hashlib.sha256(AUDIT.read_bytes()).hexdigest(),actual_phase_requested_source_ns=moving_requested,exact_recorded_source_ns=moving_ns,source_stamp_roundtrip_difference_ns=moving_ns-moving_requested,reference_source_ns=refns,reference_fusion_sources_ns=refstamps,reference_points_count=len(reference.points_base),moving_points_count=len(moving.points_base),reference_reconstruction=dict(exact_array_match=exact,max_abs_difference_m=rebuilt_delta,rebuilt_point_count=len(ref_rebuilt.points_base),support_normal_difference=(ref_rebuilt.support_normal_base-reference.support_normal_base).tolist(),support_offset_difference_m=ref_rebuilt.support_offset_m-reference.support_offset_m),registration=convert(res),yaw_deg=float(mv._yaw_degrees(res.transform_base[:3,:3],mv._support_basis(reference.support_normal_base))),registration_config=convert(config),geometry_parameters=geom,reference_fusion_parameters=ref_fusion,moving_fusion_parameters=moving_fusion,reference_support_normal=reference.support_normal_base.tolist(),reference_support_offset_m=reference.support_offset_m,moving_support_normal=moving.support_normal_base.tolist(),moving_support_offset_m=moving.support_offset_m,reference_bbox=ref_snap.bbox,moving_bbox=moving_snap.bbox,reference_geometry_center=ref_est.center_base.tolist(),moving_geometry_center=moving_est.center_base.tolist(),reference_T_base_optical=ref_T.tolist(),moving_T_base_optical=moving_T.tolist(),no_ros_actions=True,execution_authorized=False,limitations=['Live failed RegistrationResult was not serialized; values are offline reconstruction from sealed source inputs.','Reference raw points come directly from committed request audit. Reconstruction verification below checks effective configuration and timestamp TF.'])
 OUT.mkdir(parents=True,exist_ok=True)
 np.savez_compressed(OUT/'registration_inputs.npz',reference_points=reference.points_base,moving_points=moving.points_base,reference_normal=reference.support_normal_base,moving_normal=moving.support_normal_base,reference_offset=reference.support_offset_m,moving_offset=moving.support_offset_m,reference_stamp_ns=reference.stamp_ns,moving_stamp_ns=moving.stamp_ns,reference_clearance=reference.edge_clearance_px,moving_clearance=moving.edge_clearance_px)
 (OUT/'reconstruction.json').write_text(json.dumps(convert(report),indent=2,ensure_ascii=False)+'\n')
 print(json.dumps(convert(report),ensure_ascii=False),flush=True)
if __name__=='__main__':main()
