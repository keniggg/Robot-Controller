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
 record=ROOT/'grasp_records/20261010T065458.401325Z_1cbfec9472d144049738a77835f31d16'
 out=ROOT/'ros_log/full_latest_restart_20261010T002347Z/gen15_analysis/fused_reconstruction_windows_attempt'
 audits={}
 for p in (record/'keyframes_audits').glob('*.json'):
  d=json.loads(p.read_text())
  if d.get('request_id') in (2,17):audits[d['request_id']]=d
 final=audits[17];stamps=final['multiview_surface']['source_stamps_ns'];direct={d['snapshot_stamp_ns']:d for d in audits.values()}
 wanted=set(stamps)
 source_window={}
 with rosbag.Bag(str(record/'keyframes.bag')) as b:
  object_stamps=sorted({m.header.stamp.to_nsec() for _,m,_ in b.read_messages(topics=['/perception/object'])})
 for i,ns in enumerate(stamps):
  source_window[ns]=direct[ns]['pipeline_funnel']['snapshot_evidence']['source_stamp_ns'] if ns in direct else [x for x in object_stamps if x<=ns][- (5 if i==0 else 3):]
  wanted.update(source_window[ns])
 for d in audits.values():wanted.update(d['pipeline_funnel']['snapshot_evidence']['source_stamp_ns'])
 frames={};cameras=[];static=[];dynamic=[];joints=[]
 topics=['/tf','/tf_static','/grasp_record/camera_runtime_metadata','/alicia_d/accepted_joint_states','/perception/object','/perception/object_mask','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']
 with rosbag.Bag(str(record/'keyframes.bag')) as bag:
  for topic,msg,t in bag.read_messages(topics=topics):
   if topic=='/tf':dynamic.extend(msg.transforms)
   elif topic=='/tf_static':static.extend(msg.transforms)
   elif topic.endswith('camera_runtime_metadata'):cameras.append((t.to_nsec(),json.loads(msg.data)['parameters']))
   elif topic=='/alicia_d/accepted_joint_states' and msg.header.frame_id=='sdk_measured':joints.append(msg)
   elif hasattr(msg,'header') and msg.header.stamp.to_nsec() in wanted:
    frames.setdefault(msg.header.stamp.to_nsec(),{})[topic]=msg
 buffer=tf2_py.BufferCore(rospy.Duration(600.))
 for t in static:buffer.set_transform_static(t,'sealed_registration_replay')
 for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buffer.set_transform(t,'sealed_registration_replay')
 def T(ns):
  t=buffer.lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)).transform
  m=quaternion_matrix([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]);m[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
  opt=np.eye(4);opt[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
  return m@opt
 identity=TargetTrackIdentity(final['target_epoch'],final['observe_return']['target_track_id']);bridge=CvBridge()
 cfg=yaml.safe_load(Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())['grasp_6d']['remote']
 g=cfg.get('gripper_geometry',{}); fingers=g.get('finger_box_xyz_m',[.0434,.0286,.0600])
 geom=dict(support_bbox_expand_ratio=cfg.get('support_bbox_expand_ratio',.30),support_minimum_span_m=max(fingers[0],g.get('max_inner_gap_m',.05)+2*fingers[1])+2*g.get('support_clearance_m',.003),support_distance_threshold_m=cfg.get('support_distance_threshold_m',cfg.get('target_cloud_support_plane_inlier_distance_m',.004)),voxel_size_m=cfg.get('target_cloud_voxel_size_m',.0025),min_support_points=cfg.get('geometry_min_support_points',200),min_object_points=cfg.get('geometry_min_object_points',cfg.get('target_cloud_min_points',120)),min_size_m=cfg.get('geometry_min_size_m',.005),max_size_m=cfg.get('geometry_max_size_m',.6),max_height_m=cfg.get('geometry_max_height_m',.5),outlier_neighbors=cfg.get('target_cloud_outlier_neighbors',16),outlier_std_ratio=cfg.get('target_cloud_outlier_std_ratio',2.),previous_axes_base=None)
 def view(ns):
  actual_stamps=source_window[ns]
  samples=[]
  for sample_ns in actual_stamps:
   f=frames[sample_ns];obj=f['/perception/object'];dep=f['/supervisor/camera/depth/image_raw'];q=min(joints,key=lambda j:abs(j.header.stamp.to_nsec()-sample_ns))
   samples.append(RgbdSample(bridge.imgmsg_to_cv2(f['/supervisor/camera/color/image_raw'],'bgr8'),bridge.imgmsg_to_cv2(dep,'passthrough'),bridge.imgmsg_to_cv2(f['/perception/object_mask'],'mono8'),(obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height),obj,sample_ns/1e9,dep.header.frame_id,np.asarray(q.position[:6]),sample_ns,final['target_epoch'],identity))
  camera=min(cameras,key=lambda it:abs(it[0]-ns))[1];scale=camera['depth_scale']
  fusion=dict(require_mask=True,min_mask_iou=cfg.get('planning_mask_min_iou',.85),max_centroid_shift_px=cfg.get('planning_mask_max_centroid_shift_px',5.),max_joint_delta_rad=cfg.get('planning_max_joint_delta_rad',.01),erosion_px=cfg.get('mask_erosion_px',2),depth_scale=scale,depth_min_m=cfg.get('depth_min_m',.1),depth_max_m=cfg.get('depth_max_m',2.),mad_scale=cfg.get('depth_mad_scale',3.5),mad_absolute_floor_m=cfg.get('depth_mad_absolute_floor_m',.002),internal_hole_max_area_px=cfg.get('mask_internal_hole_max_area_px',25))
  snap=fuse_stable_samples(samples,**fusion);assert snap.ok,snap.failure_reason
  K=NS(**{k:camera[k] for k in ('fx','fy','cx','cy','width','height')})
  est=estimate_object_geometry(snap.depth_raw,snap.target_depth_raw,snap.object_mask,snap.bbox,K,scale,T(ns),snap.source_mode,**geom);assert est.ok,est.failure_reason
  x,y,w,h=snap.bbox;clearance=min(x,y,K.width-x-w,K.height-y-h)
  return mv.SurfaceView(identity,ns,est.object_points_base,est.support_normal_base,est.support_offset_m,clearance)
 reference=view(stamps[0]);surface=mv.fused_surface_from_view(reference,geom['voxel_size_m']);regrows=[]
 config=mv.RegistrationConfig(**cfg['multiview'])
 for ns in stamps[1:]:
  v=view(ns);result,surface=mv.append_registered_view(surface,reference,v,config,geom['voxel_size_m']);assert result.ok,convert(result)
  regrows.append(dict(stamp_ns=ns,input_points=len(v.points_base),registration=convert(result)))
 expected=final['multiview_surface'];counts={str(int(k)):int(v) for k,v in zip(*np.unique(surface.view_indices,return_counts=True))}
 report=dict(record=str(record),source_windows=source_window,sources=stamps,registration=regrows,expected_point_count=expected['point_count'],actual_point_count=len(surface.points_base),expected_view_point_counts=expected['view_point_counts'],actual_view_point_counts=counts,last_transform_max_error=float(np.max(abs(result.transform_base-np.array(expected['latest_registration']['snapshot_to_reference_transform'])))),reference_edge_clearance_is_placeholder=False,limitation='All raw SurfaceViews reconstructed from original RGB-D/mask/TF. Audit replay geometry after first view includes fused historical points and is not substituted as raw SurfaceView.')
 out.mkdir(exist_ok=True,parents=True);(out/'fused_reconstruction.json').write_text(json.dumps(convert(report),indent=2)+'\n')
 np.savez_compressed(out/'fused_reconstruction.npz',reference_points=surface.points_base,view_indices=surface.view_indices,last_transform=result.transform_base,current_points=surface.points_base@np.linalg.inv(result.transform_base)[:3,:3].T+np.linalg.inv(result.transform_base)[:3,3],support_normal=reference.support_normal_base,support_offset=reference.support_offset_m)
 print(json.dumps({k:v for k,v in convert(report).items() if k!='registration'}))
if __name__=='__main__':main()
