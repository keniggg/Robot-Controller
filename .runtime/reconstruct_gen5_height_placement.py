"""Bounded read-only GEN5 reconstruction. No ROS node, publications or services."""
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
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,evaluate_bilateral_surface_evidence_and_bounds
from alicia_flexible_grasp.grasp.contact_height_placement import place_contact_height

REC=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261010T020909.515250Z_ec46b035f0d74e2688bb217beb3da64a')
NEW=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
OUT=NEW/'gen5_handoff_analysis/contact_height_placement/exact_snapshot_reconstruction'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def plain(v):
    if isinstance(v,np.ndarray):return v.tolist()
    if isinstance(v,np.generic):return v.item()
    if dataclasses.is_dataclass(v):return {f.name:plain(getattr(v,f.name)) for f in dataclasses.fields(v)}
    if isinstance(v,dict):return {str(k):plain(x) for k,x in v.items()}
    if isinstance(v,(tuple,list)):return [plain(x) for x in v]
    return v
audits={}
for p in (REC/'keyframes_audits').glob('*.json'):
    d=json.loads(p.read_text())
    if d['request_id'] in (29,41):audits[d['request_id']]=(p,d)
P,A=audits[41];G=A['replay_geometry'];first=audits[29][1]
cfgpath=Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml')
allcfg=yaml.safe_load(cfgpath.read_text());cfg=allcfg['grasp_6d']['remote'];limit=allcfg['grasp']['handeye_translation_max_error_m']
identity=TargetTrackIdentity(16,'g5-t16');sources=A['multiview_surface']['source_stamps_ns'];ns=sources[1]
def time(ns):return rospy.Time(ns//10**9,ns%10**9)
snapshot_sources=[1791598209819755554,1791598210083642482,1791598210282150745]
buf=tf2_py.BufferCore(rospy.Duration(100.));frames={s:{} for s in [ns]+snapshot_sources};camera=[];objects={};joints=[]
topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/perception/object']
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for _,m,_ in bag.read_messages(topics=['/tf_static']):
        for t in m.transforms:buf.set_transform_static(t,'sealed_gen5')
    for topic,m,r in bag.read_messages(topics=topics+['/tf','/grasp_record/camera_runtime_metadata','/alicia_d/accepted_joint_states'],start_time=time(sources[0]-10**9),end_time=time(sources[-1]+10**9)):
        if topic=='/tf':
            for t in m.transforms:buf.set_transform(t,'sealed_gen5')
        elif topic=='/grasp_record/camera_runtime_metadata':camera.append((r.to_nsec(),json.loads(m.data)))
        elif topic=='/alicia_d/accepted_joint_states' and m.header.frame_id=='sdk_measured':joints.append((m.header.stamp.to_nsec(),np.array(m.position[:6])))
        elif topic=='/perception/object' and m.header.stamp.to_nsec() in sources:objects[m.header.stamp.to_nsec()]=m
        if topic in topics and m.header.stamp.to_nsec() in frames:frames[m.header.stamp.to_nsec()][topic]=(m,r.to_nsec())
frame=frames[ns]
assert all(set(f)==set(topics) for f in frames.values()),[(s,list(f)) for s,f in frames.items()]
cp=min(camera,key=lambda x:abs(x[0]-ns));c=cp[1]['parameters'];K=NS(**{k:c[k] for k in ('fx','fy','cx','cy','width','height')});scale=c['depth_scale']
q=buf.lookup_transform_core('base_link','camera_link',time(ns)).transform;T=quaternion_matrix([q.rotation.x,q.rotation.y,q.rotation.z,q.rotation.w]);T[:3,3]=[q.translation.x,q.translation.y,q.translation.z]
opt=np.eye(4);opt[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
bridge=CvBridge();rgb=bridge.imgmsg_to_cv2(frame[topics[0]][0],'bgr8');depth=bridge.imgmsg_to_cv2(frame[topics[1]][0],'passthrough');mask=bridge.imgmsg_to_cv2(frame[topics[2]][0],'mono8');obj=frame[topics[3]][0];bbox=tuple(getattr(obj,k) for k in ('bbox_x','bbox_y','bbox_width','bbox_height'))
fusion=dict(require_mask=True,min_mask_iou=cfg.get('planning_mask_min_iou',.85),max_centroid_shift_px=cfg.get('planning_mask_max_centroid_shift_px',5.),max_joint_delta_rad=cfg.get('planning_max_joint_delta_rad',.01),erosion_px=cfg.get('mask_erosion_px',2),depth_scale=scale,depth_min_m=cfg.get('depth_min_m',.1),depth_max_m=cfg.get('depth_max_m',2.),mad_scale=cfg.get('depth_mad_scale',3.5),mad_absolute_floor_m=cfg.get('depth_mad_absolute_floor_m',.002),internal_hole_max_area_px=cfg.get('mask_internal_hole_max_area_px',25))
sample=RgbdSample(rgb,depth,mask,bbox,obj,ns/1e9,'camera_link',min(joints,key=lambda v:abs(v[0]-ns))[1],ns,16,identity)
snap=fuse_stable_samples([sample],**fusion);assert snap.ok,snap.failure_reason
geom=dict(support_bbox_expand_ratio=cfg.get('support_bbox_expand_ratio',.30),support_minimum_span_m=.05+2*.0286+2*.003,support_distance_threshold_m=cfg.get('support_distance_threshold_m',cfg.get('target_cloud_support_plane_inlier_distance_m',.004)),voxel_size_m=cfg.get('target_cloud_voxel_size_m',.0025),min_support_points=cfg.get('geometry_min_support_points',200),min_object_points=cfg.get('geometry_min_object_points',cfg.get('target_cloud_min_points',120)),min_size_m=cfg.get('geometry_min_size_m',.005),max_size_m=cfg.get('geometry_max_size_m',.6),max_height_m=cfg.get('geometry_max_height_m',.5),outlier_neighbors=cfg.get('target_cloud_outlier_neighbors',16),outlier_std_ratio=cfg.get('target_cloud_outlier_std_ratio',2.),previous_axes_base=None)
est=estimate_object_geometry(snap.depth_raw,snap.target_depth_raw,snap.object_mask,snap.bbox,K,scale,T@opt,snap.source_mode,**geom);assert est.ok,est.failure_reason
def edge(sn):
    o=objects[sn];return min(o.bbox_x,o.bbox_y,K.width-o.bbox_x-o.bbox_width,K.height-o.bbox_y-o.bbox_height)
fg=first['replay_geometry'];reference=mv.SurfaceView(identity,sources[0],np.array(fg['object_points_base_m']),np.array(fg['support_normal_base']),fg['support_offset_m'],edge(sources[0]))
middle=mv.SurfaceView(identity,ns,est.object_points_base,est.support_normal_base,est.support_offset_m,edge(ns))
samples=[]
for sn in snapshot_sources:
    f=frames[sn];o=f[topics[3]][0];box=tuple(getattr(o,k) for k in ('bbox_x','bbox_y','bbox_width','bbox_height'))
    samples.append(RgbdSample(bridge.imgmsg_to_cv2(f[topics[0]][0],'bgr8'),bridge.imgmsg_to_cv2(f[topics[1]][0],'passthrough'),bridge.imgmsg_to_cv2(f[topics[2]][0],'mono8'),box,o,sn/1e9,'camera_link',min(joints,key=lambda v:abs(v[0]-sn))[1],sn,16,identity))
lastsnap=fuse_stable_samples(samples,**fusion);assert lastsnap.ok,lastsnap.failure_reason
tf=buf.lookup_transform_core('base_link','camera_link',time(sources[-1])).transform;lastT=quaternion_matrix([tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w]);lastT[:3,3]=[tf.translation.x,tf.translation.y,tf.translation.z]
lastest=estimate_object_geometry(lastsnap.depth_raw,lastsnap.target_depth_raw,lastsnap.object_mask,lastsnap.bbox,K,scale,lastT@opt,lastsnap.source_mode,**geom);assert lastest.ok,lastest.failure_reason
last=mv.SurfaceView(identity,sources[-1],lastest.object_points_base,lastest.support_normal_base,lastest.support_offset_m,edge(sources[-1]))
surface=mv.fused_surface_from_view(reference,geom['voxel_size_m']);regcfg=mv.RegistrationConfig(**cfg['multiview']);registrations=[]
for v in (middle,last):
    reg,surface=mv.append_registered_view(surface,reference,v,regcfg,geom['voxel_size_m']);assert reg.ok,reg
    registrations.append(plain(reg))
oldT=np.array(A['multiview_surface']['latest_registration']['snapshot_to_reference_transform']);Terr=float(np.max(abs(reg.transform_base-oldT)))
counts={str(int(k)):int(v) for k,v in zip(*np.unique(surface.view_indices,return_counts=True))}
checks=dict(expected_count=A['multiview_surface']['point_count'],actual_count=len(surface.points_base),expected_view_counts=A['multiview_surface']['view_point_counts'],actual_view_counts=counts,latest_transform_max_abs_error=Terr,last_raw_points=len(lastest.object_points_base),last_raw_size_m=lastest.size_xyz_m.tolist(),published_enclosed_size_m=G['obb_size_xyz_m'])
inv=np.linalg.inv(reg.transform_base);current=dataclasses.replace(surface,points_base=surface.points_base@inv[:3,:3].T+inv[:3,3]);normal=reg.transform_base[:3,:3].T@reference.support_normal_base
gr=GripperGeometry(.05,.002,np.array([.0434,.0286,.060]),np.array([.1175,.155,.0774]),.003);rows=[]
for i,row in enumerate(A['rows'][24:]):
    transform=np.array(row['T_base_tool0']);e,bounds=evaluate_bilateral_surface_evidence_and_bounds(current,row['contact_center_base_m'],transform[:3,1],transform[:3,2],gr,support_normal_base=normal)
    if bounds is None:rows.append(dict(row_index=i+24,evidence=plain(e),bounds=None));continue
    result,placement=place_contact_height(translation_evidence_limit_m=limit,gripper=gr,actual_T_base_tool0=transform,planned_T_base_tool0=transform,contact_center_base=row['contact_center_base_m'],contact_height_axis_base=transform[:3,2],contact_height_bounds_m=bounds,minimum_contact_overlap_m=.002,support_normal_base=G['support_normal_base'],support_offset_m=G['support_offset_m'],opening_width_m=.05)
    before=placement['before'];intervals=before['allowed_normal_translation_intervals_m'];connected=[(a,b) for a,b in intervals if a is not None and b is not None and a<=0<=b]
    rows.append(dict(row_index=i+24,inferred_stable_track=i+1,source_index=row['source_index'],variant_index=row['variant_index'],yaw_deg=row['sampled_angle_deg'],tilt_deg=row['approach_tilt_deg'],bounds_m=bounds,evidence=plain(e),placement=placement,passes=result is not None,maximum_recentered_symmetric_margin_m=max(((b-a)/2 for a,b in connected),default=0),permitted_center_shift_for_full_evidence_m=[(a+limit,b-limit) for a,b in connected if b-a>=2*limit]))
report=dict(record=str(REC),record_manifest_sha256=sha(REC/'manifest.json'),audit_path=str(P),audit_sha256=sha(P),source_stamps_ns=sources,config_sha256=sha(cfgpath),script_sha256=sha(__file__),middle_source_receipts_ns={k:v[1] for k,v in frame.items()},middle_camera_metadata_receipt_ns=cp[0],middle_points=len(est.object_points_base),middle_fusion_parameters=fusion,middle_geometry_parameters=geom,registrations=registrations,reconstruction_checks=checks,rows=rows)
print(json.dumps(plain(dict(reconstruction_checks=checks,rows=[{k:v for k,v in r.items() if k not in ('placement','evidence')} for r in rows])),indent=2))
OUT.mkdir(exist_ok=True,parents=True)
assert not (OUT/'reconstruction.json').exists(),'Do not overwrite diagnostic evidence'
(OUT/'reconstruction.json').write_text(json.dumps(plain(report),indent=2)+'\n')
np.savez_compressed(str(OUT/'fused_reconstruction.npz'),reference_points=surface.points_base,current_points=current.points_base,view_indices=surface.view_indices,registration_transform=reg.transform_base,current_reference_normal=normal)
np.savez_compressed(str(OUT/'raw_geometry.npz'),points=lastest.object_points_base,center=lastest.center_base,axes=lastest.axes_base,size=lastest.size_xyz_m,depth_raw=lastsnap.depth_raw,object_mask=lastsnap.object_mask)
