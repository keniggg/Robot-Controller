"""Offline, closed-record geometry/registration replay; no ROS communication."""
from pathlib import Path
from types import SimpleNamespace
from dataclasses import asdict
import hashlib
import json
import numpy as np
import rosbag
import rospy
import tf2_ros
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.grasp6d_adapter import CameraIntrinsics
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.multiview_surface import SurfaceView, RegistrationConfig, register_surface_view

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC=BASE/'grasp_records/20261011T131409.553041Z_fbd9eb44a4bb4493bdc065e0622ed215'
OUT=BASE/'ros_log/full_latest_no_skin_20261011T000625Z/gen18_analysis/evidence_registration_replay'
REQ1=REC/'keyframes_audits/39d60d8f789c6c8334350c210e63a53d1fce0f73c7f3e412ef8524e3f9b8c867.json'
REQ42=REC/'keyframes_audits/3a88b0ffd52be7f55b2a5c234d4984f7a1e6695698cea7536b2c78338f57d08c.json'
a0=json.loads(REQ1.read_text()); audit=json.loads(REQ42.read_text())
config_data=json.loads((REC/'perception_execution_configuration.json').read_text())
config=RegistrationConfig(**config_data['parameters']['/grasp_6d/remote/multiview'])
identity=TargetTrackIdentity(audit['target_epoch'],'g1-t4')
def view(stamp, points, normal, offset):
    return SurfaceView(identity,stamp,np.asarray(points),np.asarray(normal),offset,10)
g0=a0['replay_geometry']; g=audit['replay_geometry']
reference=view(a0['snapshot_stamp_ns'],g0['object_points_base_m'],g0['support_normal_base'],g0['support_offset_m'])
raw_result=register_surface_view(reference,view(audit['snapshot_stamp_ns'],g['object_points_base_m'],g['support_normal_base'],g['support_offset_m']),config)
def result_summary(result):
    return dict(ok=result.ok,code=result.code,transform_base=result.transform_base.tolist(),
        rotation_deg=float(np.degrees(np.arccos(np.clip((np.trace(result.transform_base[:3,:3])-1)*.5,-1,1)))),
        inlier_count=result.inlier_count,overlap_fraction=result.overlap_fraction,rmse_m=result.rmse_m,
        support_normal_angle_deg=result.support_normal_angle_deg,
        support_plane_separation_m=result.support_plane_separation_m,
        maximum_point_displacement_m=result.maximum_point_displacement_m)
assert np.array_equal(raw_result.transform_base,audit['multiview_surface']['latest_registration']['snapshot_to_reference_transform'])
images={}; objects={}; camera=None
buffer=tf2_ros.Buffer(cache_time=rospy.Duration(10000),debug=False)
image_topics={'/supervisor/camera/color/image_raw':'color','/supervisor/camera/depth/image_raw':'depth','/perception/object_mask':'mask'}
topics=list(image_topics)+['/perception/object','/grasp_record/camera_runtime_metadata','/tf','/tf_static']
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for topic,m,t in bag.read_messages(topics=topics):
        if topic in ('/tf','/tf_static'):
            setter=buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform
            for v in m.transforms:setter(v,'closed_record')
            continue
        if topic=='/grasp_record/camera_runtime_metadata':
            if camera is None:camera=json.loads(m.data)['parameters']
            continue
        stamp=m.header.stamp.to_nsec()
        if not 1791724570000000000 <= stamp <= 1791724573259364604:continue
        if topic=='/perception/object':
            if m.detected:objects[stamp]=m
            continue
        dtype=np.uint16 if m.encoding=='16UC1' else np.uint8
        channels=3 if m.encoding=='bgr8' else 1
        assert m.encoding in ('16UC1','bgr8','mono8','8UC1'),m.encoding
        arr=np.frombuffer(m.data,dtype=dtype).reshape(m.height,m.step//np.dtype(dtype).itemsize)[:,:m.width*channels].copy()
        if channels==3:arr=arr.reshape(m.height,m.width,3)
        images.setdefault(stamp,{})[image_topics[topic]]=arr
intr=CameraIntrinsics(**{k:camera[k] for k in ('width','height','fx','fy','cx','cy','depth_scale')})
def transform(stamp):
    v=buffer.lookup_transform('base_link','camera_link',rospy.Time(stamp//10**9,stamp%10**9))
    q=v.transform.rotation; p=v.transform.translation
    T=quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[p.x,p.y,p.z]
    return T
optical_to_link=np.asarray([[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1.]])
available=[s for s in sorted(objects) if set(images.get(s,{}))==set(('color','depth','mask'))]
current=audit['pipeline_funnel']['snapshot_evidence']['source_stamp_ns']
future=[available[i-2:i+1] for i in range(2,len(available)) if available[i]>current[-1]]
windows=[current]+[future[i] for i in (0,2) if i<len(future)]
rows=[];OUT.mkdir(parents=True,exist_ok=True)
for stamps in windows:
    samples=[]
    for stamp in stamps:
        arrays=images[stamp];obj=objects[stamp];bbox=(obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height)
        samples.append(RgbdSample(arrays['color'],arrays['depth'],arrays['mask'],bbox,obj,stamp*1e-9,'camera_link',np.zeros(6),stamp,identity.epoch,identity))
    snap=fuse_stable_samples(samples,True,.85,5.,.004,2,intr.depth_scale,.03,2.,3.5,.002,25)
    row=dict(source_stamps_ns=stamps,snapshot_ok=snap.ok)
    if not snap.ok:
        row.update(failure_code=snap.failure_code,failure_reason=snap.failure_reason);rows.append(row);continue
    T=transform(stamps[-1]);row['camera_link_transform']=T.tolist()
    e=estimate_object_geometry(depth_raw=snap.depth_raw,target_depth_raw=snap.target_depth_raw,
        object_mask=snap.object_mask,bbox=snap.bbox,intrinsics=intr,depth_scale=intr.depth_scale,
        T_base_camera=T.dot(optical_to_link),source_mode='instance_mask',support_bbox_expand_ratio=.3,
        support_distance_threshold_m=.004,voxel_size_m=.0015,min_support_points=200,min_object_points=120,
        min_size_m=.005,max_size_m=.6,max_height_m=.5,previous_axes_base=np.asarray(g['R_base_obb']),
        outlier_neighbors=16,outlier_std_ratio=2.,support_minimum_span_m=.1132)
    row['geometry_ok']=e.ok
    if not e.ok:
        row.update(failure_code=e.failure_code,failure_reason=e.failure_reason);rows.append(row);continue
    points=e.object_points_base
    rr=register_surface_view(reference,view(stamps[-1],points,e.support_normal_base,e.support_offset_m),config)
    row.update(point_count=len(points),points_sha256=hashlib.sha256(points.tobytes()).hexdigest(),registration=result_summary(rr))
    if stamps==current:
        row['recorded_transform_max_abs_difference']=float(np.max(np.abs(T-np.asarray(audit['snapshot_transform']['T_base_camera_link']))))
        row['recorded_points_array_equal']=bool(np.array_equal(points,g['object_points_base_m']))
        row['recorded_points_max_abs_difference_m']=float(np.max(np.abs(points-np.asarray(g['object_points_base_m'])))) if len(points)==len(g['object_points_base_m']) else None
    np.savez_compressed(str(OUT/('%d.npz'%stamps[-1])),points=points,support_normal=e.support_normal_base,support_offset=np.asarray(e.support_offset_m),camera_link_transform=T)
    rows.append(row)
report=dict(schema='gen18_offline_registration_replay/v1',record_dir=str(REC),reference_audit_sha256=REQ1.stem,failed_audit_sha256=REQ42.stem,
    original_config=asdict(config),exact_recorded_cloud_replay=result_summary(raw_result),camera_parameters=camera,
    available_sources=available,windows=rows,
    limitations=['No ROS connection or motion; TF queries use an offline buffer populated solely from the closed bag.',
        'Window geometry uses the production image-fusion and geometry functions; exact current-window equality is reported before interpreting later counterfactual windows.',
        'Joint vectors are zero only for image fusion arithmetic; this is not station qualification, future source freshness, identity or motion authorization.',
        'Passing registration is not passing contact evidence, full collision path or physical grasp.'])
path=OUT/'result.json';assert not path.exists();path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),windows=rows),indent=2),flush=True)
