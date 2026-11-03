"""GEN7 finite sealed-bag surface reconstruction. No ROS nodes or actions."""
import dataclasses,hashlib,json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np,yaml,rosbag,rospy,tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample,fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision import multiview_surface as mv

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC=ROOT/'grasp_records/20261010T030141.809371bd9c281954c'
REC=ROOT/'grasp_records/20261010T030141.809371Z_7724ffb5910c4565bf751bd9c281954c'
NEW=ROOT/'ros_log/full_latest_restart_20261010T002347Z'
OUT=NEW/'gen7_handoff_analysis/contact_analysis/exact_surface'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def plain(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 if dataclasses.is_dataclass(v):return {f.name:plain(getattr(v,f.name)) for f in dataclasses.fields(v)}
 if isinstance(v,dict):return {str(k):plain(x) for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [plain(x) for x in v]
 return v
def tm(ns):return rospy.Time(ns//10**9,ns%10**9)
audits={}
for p in (REC/'keyframes_audits').glob('*.json'):
 d=json.loads(p.read_text())
 if d['request_id'] in (1,16):audits[d['request_id']]=(p,d)
P,A=audits[16];G=A['replay_geometry'];first=audits[1][1]
cfgpath=Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml')
cfg=yaml.safe_load(cfgpath.read_text())['grasp_6d']['remote']
identity=TargetTrackIdentity(A['target_epoch'],A['multiview_surface']['target_track_id'])
sources=A['multiview_surface']['source_stamps_ns']
groups=[[sources[1]],[1791601359716917514,1791601359979528903,1791601360247446060],
        [1791601361313328981,1791601361580522060,1791601361780489206]]
frames={s:{} for group in groups for s in group};camera=[];objects={};joints=[]
topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/perception/object']
buf=tf2_py.BufferCore(rospy.Duration(100.))
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
 for _,m,_ in bag.read_messages(topics=['/tf_static']):
  for t in m.transforms:buf.set_transform_static(t,'sealed_gen7')
 for topic,m,r in bag.read_messages(topics=topics+['/tf','/grasp_record/camera_runtime_metadata','/alicia_d/accepted_joint_states'],start_time=tm(sources[0]-10**9),end_time=tm(sources[-1]+10**9)):
  if topic=='/tf':
   for t in m.transforms:buf.set_transform(t,'sealed_gen7')
  elif topic=='/grasp_record/camera_runtime_metadata':camera.append((r.to_nsec(),json.loads(m.data)))
  elif topic=='/alicia_d/accepted_joint_states' and m.header.frame_id=='sdk_measured':joints.append((m.header.stamp.to_nsec(),np.array(m.position[:6])))
  elif topic=='/perception/object' and m.header.stamp.to_nsec() in sources:objects[m.header.stamp.to_nsec()]=m
  if topic in topics and m.header.stamp.to_nsec() in frames:frames[m.header.stamp.to_nsec()][topic]=(m,r.to_nsec())
assert all(set(f)==set(topics) for f in frames.values()),[(s,list(f)) for s,f in frames.items()]
cp=min(camera,key=lambda x:abs(x[0]-sources[1]));c=cp[1]['parameters']
K=NS(**{k:c[k] for k in ('fx','fy','cx','cy','width','height')});scale=c['depth_scale']
bridge=CvBridge();opt=np.eye(4);opt[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
fusion=dict(require_mask=True,min_mask_iou=cfg.get('planning_mask_min_iou',.85),max_centroid_shift_px=cfg.get('planning_mask_max_centroid_shift_px',5.),max_joint_delta_rad=cfg.get('planning_max_joint_delta_rad',.01),erosion_px=cfg.get('mask_erosion_px',2),depth_scale=scale,depth_min_m=cfg.get('depth_min_m',.1),depth_max_m=cfg.get('depth_max_m',2.),mad_scale=cfg.get('depth_mad_scale',3.5),mad_absolute_floor_m=cfg.get('depth_mad_absolute_floor_m',.002),internal_hole_max_area_px=cfg.get('mask_internal_hole_max_area_px',25))
geom=dict(support_bbox_expand_ratio=cfg.get('support_bbox_expand_ratio',.30),support_minimum_span_m=.05+2*.0286+2*.003,support_distance_threshold_m=cfg.get('support_distance_threshold_m',cfg.get('target_cloud_support_plane_inlier_distance_m',.004)),voxel_size_m=cfg.get('target_cloud_voxel_size_m',.0025),min_support_points=cfg.get('geometry_min_support_points',200),min_object_points=cfg.get('geometry_min_object_points',cfg.get('target_cloud_min_points',120)),min_size_m=cfg.get('geometry_min_size_m',.005),max_size_m=cfg.get('geometry_max_size_m',.6),max_height_m=cfg.get('geometry_max_height_m',.5),outlier_neighbors=cfg.get('target_cloud_outlier_neighbors',16),outlier_std_ratio=cfg.get('target_cloud_outlier_std_ratio',2.),previous_axes_base=None)
def edge(sn):
 o=objects[sn];return min(o.bbox_x,o.bbox_y,K.width-o.bbox_x-o.bbox_width,K.height-o.bbox_y-o.bbox_height)
fg=first['replay_geometry'];reference=mv.SurfaceView(identity,sources[0],np.array(fg['object_points_base_m']),np.array(fg['support_normal_base']),fg['support_offset_m'],edge(sources[0]))
surface=mv.fused_surface_from_view(reference,geom['voxel_size_m']);regcfg=mv.RegistrationConfig(**cfg['multiview'])
registrations=[];raws=[]
for group in groups:
 samples=[]
 for sn in group:
  f=frames[sn];o=f[topics[3]][0];box=tuple(getattr(o,k) for k in ('bbox_x','bbox_y','bbox_width','bbox_height'))
  samples.append(RgbdSample(bridge.imgmsg_to_cv2(f[topics[0]][0],'bgr8'),bridge.imgmsg_to_cv2(f[topics[1]][0],'passthrough'),bridge.imgmsg_to_cv2(f[topics[2]][0],'mono8'),box,o,sn/1e9,'camera_link',min(joints,key=lambda v:abs(v[0]-sn))[1],sn,identity.epoch,identity))
 snap=fuse_stable_samples(samples,**fusion);assert snap.ok,snap.failure_reason
 q=buf.lookup_transform_core('base_link','camera_link',tm(group[-1])).transform
 T=quaternion_matrix([q.rotation.x,q.rotation.y,q.rotation.z,q.rotation.w]);T[:3,3]=[q.translation.x,q.translation.y,q.translation.z]
 est=estimate_object_geometry(snap.depth_raw,snap.target_depth_raw,snap.object_mask,snap.bbox,K,scale,T@opt,snap.source_mode,**geom);assert est.ok,est.failure_reason
 raw=mv.SurfaceView(identity,group[-1],est.object_points_base,est.support_normal_base,est.support_offset_m,edge(group[-1]))
 reg,surface=mv.append_registered_view(surface,reference,raw,regcfg,geom['voxel_size_m']);assert reg.ok,reg
 registrations.append(plain(reg));raws.append(dict(source_ns=group[-1],points=len(est.object_points_base),size=est.size_xyz_m,center=est.center_base))
counts={str(int(k)):int(v) for k,v in zip(*np.unique(surface.view_indices,return_counts=True))}
Terr=float(np.max(abs(reg.transform_base-np.array(A['multiview_surface']['latest_registration']['snapshot_to_reference_transform']))))
inv=np.linalg.inv(reg.transform_base);current=dataclasses.replace(surface,points_base=surface.points_base@inv[:3,:3].T+inv[:3,3])
normal=reg.transform_base[:3,:3].T@reference.support_normal_base
checks=dict(expected_count=A['multiview_surface']['point_count'],actual_count=len(surface.points_base),expected_view_counts=A['multiview_surface']['view_point_counts'],actual_view_counts=counts,latest_transform_max_abs_error=Terr)
exact=checks['actual_count']==checks['expected_count'] and counts==checks['expected_view_counts'] and Terr<1e-12
report=dict(record=str(REC),audit_path=str(P),audit_sha256=sha(P),script_sha256=sha(__file__),config_sha256=sha(cfgpath),snapshot_source_groups=groups,registrations=registrations,reconstruction_checks=checks,exact_reconstruction=exact,raws=raws,limitations=['Reconstruction uses recorded RGBD/masks/TF and current unchanged estimation configuration; exactness is asserted only if point/view counts and registration transform agree with sealed audit.'])
OUT.mkdir(exist_ok=True,parents=True)
with (OUT/'reconstruction.json').open('x') as out:json.dump(plain(report),out,indent=2);out.write('\n')
np.savez_compressed(str(OUT/'fused_reconstruction.npz'),reference_points=surface.points_base,current_points=current.points_base,view_indices=surface.view_indices,registration_transform=reg.transform_base,current_reference_normal=normal,raw_points=est.object_points_base,raw_center=est.center_base,raw_axes=est.axes_base,raw_size=est.size_xyz_m)
print(json.dumps(plain(dict(exact_reconstruction=exact,reconstruction_checks=checks,raws=raws)),indent=2))
