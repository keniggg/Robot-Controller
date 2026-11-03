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
RECORD=ROOT/'grasp_records/20261009T023854.657870Z_44fae4d9c54c40c8a8ecf0969e9ca5d1'
OUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/gen8_contact_analysis'
AUDIT=RECORD/'keyframes_audits/26882236e207615a7af6e61a83247acb5ee974811f9c847947643f943813f8ec.json'
def convert(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 if dataclasses.is_dataclass(x):return {f.name:convert(getattr(x,f.name)) for f in dataclasses.fields(x)}
 if isinstance(x,dict):return {k:convert(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [convert(v) for v in x]
 return x

def main():
 audit=json.loads(AUDIT.read_text()); rg=audit['replay_geometry'];refns=audit['snapshot_stamp_ns']
 refstamps=audit["pipeline_funnel"]["snapshot_evidence"]["source_stamp_ns"]
 final=json.loads((RECORD/"keyframes_audits/6aa2e914a64e0bd50cf21f8452d46cb049bad61523c5f34d5e873165bcd6cc93.json").read_text()); latest_stamps=final["pipeline_funnel"]["snapshot_evidence"]["source_stamp_ns"]
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
   if ns in refstamps or ns in latest_stamps or abs(ns-moving_requested)<1000:
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
 identity=TargetTrackIdentity(6,'g1-t6');bridge=CvBridge()
 def sample(ns):
  f=frames[ns];obj=f['/perception/object'];dep=f['/supervisor/camera/depth/image_raw'];q=min(joints,key=lambda j:abs(j.header.stamp.to_nsec()-ns))
  return RgbdSample(bridge.imgmsg_to_cv2(f['/supervisor/camera/color/image_raw'],'bgr8'),bridge.imgmsg_to_cv2(dep,'passthrough'),bridge.imgmsg_to_cv2(f['/perception/object_mask'],'mono8'),(obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height),obj,ns/1e9,dep.header.frame_id,np.asarray(q.position[:6]),ns,6,identity)
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
 config=mv.RegistrationConfig(**cfg['multiview'])
 from alicia_grasp_modes.active_observation_policy import registered_information_gain
 from alicia_grasp_modes.observe_return_visual import measured_bilateral_view_gaps,bounded_bilateral_view_gaps
 from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_bilateral_surface_measurement
 surface0=mv.fused_surface_from_view(reference,geom['voxel_size_m'])
 res1,surface1=mv.append_registered_view(surface0,reference,moving,config,geom['voxel_size_m']);assert res1.ok,convert(res1)
 gain=registered_information_gain(surface0,surface1,res1,voxel_size_m=geom['voxel_size_m'],depth_uncertainty_m=.003)
 latest_est,latest,latest_snap,latest_T,latest_fusion=estimate(latest_stamps)
 res2,surface2=mv.append_registered_view(surface1,reference,latest,config,geom['voxel_size_m']);assert res2.ok,convert(res2)
 inverse=np.linalg.inv(res2.transform_base)
 current=dataclasses.replace(surface2,points_base=surface2.points_base@inverse[:3,:3].T+inverse[:3,3])
 ag=final['replay_geometry'];geometry=NS(center_base=np.asarray(ag['obb_center_base_m']),axes_base=np.asarray(ag['R_base_obb']),size_xyz_m=np.asarray(ag['obb_size_xyz_m']),support_normal_base=np.asarray(ag['support_normal_base']))
 finger=GripperGeometry(.05,.002,np.array([.0434,.0286,.0600]),np.array([.1175,.1550,.0774]),.003);tableconfig=NS(max_inner_gap_m=.05,opening_fit_clearance_each_side_m=.0005,min_contact_band_points=12)
 gaps=measured_bilateral_view_gaps(final['remote_diagnostics'],geometry,tableconfig,current,finger,latest_T[:3,3]);bounded=bounded_bilateral_view_gaps(final['remote_diagnostics'],geometry,tableconfig)
 # Evaluate exactly the same masks and common-height output for each current direction.
 measures=[]
 for gap in bounded:
  counts,lo,hi,masks=_bilateral_surface_measurement(current,geometry.center_base,np.asarray(gap['jaw_axis_base']),-geometry.support_normal_base,finger,12,geometry.support_normal_base)
  vals=[]
  if isinstance(masks,tuple):
   for mask in masks:
    h=(current.points_base[mask]-geometry.center_base)@geometry.support_normal_base
    vals.append(dict(count=len(h),quantile_05_95_m=np.quantile(h,[.05,.95]).tolist() if len(h) else None))
  measures.append(dict(angle_deg=gap['angle_deg'],counts=counts,common_insertion_bounds_m=None if lo is None else [lo,hi],support_heights=vals))
 expected=final['multiview_surface'];actual_counts={str(int(k)):int(v) for k,v in zip(*np.unique(surface2.view_indices,return_counts=True))}
 report=dict(diagnostic_only=True,no_ros_actions=True,record=str(RECORD),reference_stamp_ns=refns,arrival_stamp_ns=moving_ns,latest_stamps_ns=latest_stamps,reference_points_exact=np.array_equal(reference.points_base,ref_rebuilt.points_base),first_registration=convert(res1),first_registered_gain=convert(gain),latest_registration=convert(res2),latest_transform_exact=np.array_equal(np.asarray(expected['latest_registration']['snapshot_to_reference_transform']),res2.transform_base),latest_registration_max_difference=float(np.max(np.abs(np.asarray(expected['latest_registration']['snapshot_to_reference_transform'])-res2.transform_base))),fused_points_count=len(surface2.points_base),expected_fused_points_count=expected['point_count'],fused_view_point_counts=actual_counts,expected_view_point_counts=expected['view_point_counts'],fused_source_stamps_ns=surface2.view_stamps_ns,current_bounded_gaps=bounded,current_measured_gaps=gaps,current_bilateral_masks=measures,new_view_uncertainty_m=max(p['depth_uncertainty_m'] for p in final['remote_diagnostics']['tabletop_geometry']['adaptive_stage_profiles']),gain_novelty_uncertainty_m=.003,limitations=['Actual gain was not serialized by runtime; recomputed from exact sealed reference/arrival measurements and the unchanged gain function.','Gain concerns new occupied coverage only, never authorizes contact or predicts another view will succeed.'])
 OUT.mkdir(parents=True,exist_ok=True)
 np.savez_compressed(OUT/'fused_inputs.npz',initial_points=surface0.points_base,arrival_points=surface1.points_base,arrival_view_indices=surface1.view_indices,final_points=surface2.points_base,final_view_indices=surface2.view_indices,current_points=current.points_base,reference_normal=reference.support_normal_base,latest_normal=latest.support_normal_base,latest_transform=res2.transform_base)
 (OUT/'gain_gap_reconstruction.json').write_text(json.dumps(convert(report),indent=2,ensure_ascii=False)+'\n')
 print(json.dumps(convert(report),ensure_ascii=False),flush=True)
if __name__=='__main__':main()
