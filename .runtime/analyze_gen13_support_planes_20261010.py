"""Reproduce GEN13 stationary support disagreement from sealed RGB-D/TF only."""
from pathlib import Path
from dataclasses import asdict
from types import SimpleNamespace
import hashlib, json, math, re
import numpy as np
import cv2
import rosbag, rospy, tf2_ros
from tf.transformations import quaternion_matrix
from alicia_grasp_modes import tabletop

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC=ROOT/'grasp_records/20261010T050849.339487Z_fa4a10a8358a4fa2b80214f1f626b5c7'
OUT=ROOT/'ros_log/full_latest_restart_20261010T002347Z/gen13_handoff_analysis/support_plane_replay'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert json.loads((REC/'manifest.json').read_text())['recorder_closed']
ARRIVAL=1791609019473174095
DEADLINE=1791609022473174095
cv2.setNumThreads(1)
cfg=tabletop.Config()
buffer=tf2_ros.Buffer(cache_time=rospy.Duration(600))
statuses=[]; camera=None; tf_static=[]
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for topic,msg,received in bag.read_messages(topics=['/perception/unknown/detector_status', '/tf', '/tf_static', '/grasp_record/camera_runtime_metadata']):
        if topic in ('/tf','/tf_static'):
            for transform in msg.transforms:
                (buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(transform,'sealed_gen13')
                if topic=='/tf_static':
                    t=transform.transform
                    tf_static.append(dict(parent=transform.header.frame_id, child=transform.child_frame_id,
                        xyz=[t.translation.x,t.translation.y,t.translation.z], quaternion=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]))
        elif topic.endswith('detector_status'):
            d=json.loads(msg.data)
            if d.get('generation')==13 and d.get('source_stamp_ns'):
                statuses.append(dict(received_ns=received.to_nsec(),data=d))
        elif camera is None:
            camera=json.loads(msg.data)['parameters']

failure=[r for r in statuses if ARRIVAL<r['data']['source_stamp_ns']<DEADLINE and
    r['data'].get('observation_measurement',{}).get('reason','').startswith('support_plane_local_distance_inconsistent')]
initial=next(r for r in statuses if r['data'].get('detail')=='acquired')
firstview=next(r for r in statuses if r['data'].get('detail')=='observation_measurement_requalified')
eligible={r['data']['source_stamp_ns']:r for r in [initial,firstview]+failure}
images={}
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for topic,msg,received in bag.read_messages(topics=['/supervisor/camera/depth/image_raw','/experiment/observation_rgbd/depth']):
        ns=msg.header.stamp.to_nsec()
        if ns in eligible:
            dtype=np.dtype('<u2' if msg.encoding in ('16UC1','mono16') else '<f4')
            raw=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.step,dtype.itemsize)).copy()
            images[ns]=dict(raw=raw, encoding=msg.encoding, topic=topic,
                header_frame_id=msg.header.frame_id, raw_sha256=hashlib.sha256(msg.data).hexdigest())
available=[r for r in failure if r['data']['source_stamp_ns'] in images]
assert available
chosen={r['data']['source_stamp_ns']:r for r in [initial,firstview,available[0],available[len(available)//2],available[-1]]}
assert all(ns in images for ns in chosen)
OUT.mkdir(parents=True,exist_ok=True)
reference=np.asarray(initial['data']['reference_support_plane_base'])
anchor=np.asarray(initial['data']['anchor_position_base'])
footprint=np.asarray(initial['data']['reference_support_footprint_base'])
K=np.asarray([camera[k] for k in ('fx','fy','cx','cy')])

def metrics(plane):
    if plane[:3]@reference[:3]<0:plane=-plane
    delta=plane-reference; center=footprint.mean(axis=0)
    center_term=float(center@delta[:3]+delta[3]); edge=(footprint-center)@delta[:3]
    signed=footprint@delta[:3]+delta[3]
    anchor_shift=float(anchor@delta[:3]+delta[3])
    return dict(normal_delta_deg=float(np.rad2deg(np.arccos(np.clip(plane[:3]@reference[:3],-1,1)))),
        anchor_signed_delta_mm=anchor_shift*1000, footprint_center_signed_delta_mm=center_term*1000,
        footprint_edge_tilt_range_mm=[float(min(edge))*1000,float(max(edge))*1000],
        footprint_signed_delta_range_mm=[float(min(signed))*1000,float(max(signed))*1000],
        maximum_local_shift_mm=max(abs(anchor_shift),float(max(abs(signed))))*1000)

frames=[]
for ns,row in sorted(chosen.items()):
    status=row['data']; image=images[ns]
    raw=image['raw']; depth=raw.astype(np.float32)*(float(camera['depth_scale']) if image['encoding'] in ('16UC1','mono16') else 1.)
    stamp=rospy.Time(ns//10**9,ns%10**9)
    transform=buffer.lookup_transform('base_link','camera_link',stamp).transform
    T=quaternion_matrix([transform.rotation.x,transform.rotation.y,transform.rotation.z,transform.rotation.w])
    T[:3,3]=[transform.translation.x,transform.translation.y,transform.translation.z]
    center=tabletop.project_base_to_pixel(anchor,T,K)
    if row is initial:center=np.asarray([depth.shape[1]/2.,depth.shape[0]/2.])
    vv,uu=np.indices(depth.shape);scale=min(depth.shape[1]/640.,depth.shape[0]/480.)
    radius=np.hypot(uu-center[0],vv-center[1]);valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
    plane,ratio,residual,count,ring,points,outer=tabletop._fit_support_ring_sequence(depth,K,valid,radius,scale,cfg)
    current=tabletop._normalized_plane(np.linalg.solve(T.T,plane))
    support=abs(points@plane[:3]+plane[3])<=cfg.support_threshold_m
    covered=tabletop._support_quadrants(support,uu[ring],vv[ring],center)
    path=OUT/('source_%d.npz'%ns)
    with path.open('xb') as f:
        np.savez_compressed(f,depth_raw=raw,depth_scale=float(camera['depth_scale']),T_base_camera=T,intrinsics=K,reference_plane_base=reference,anchor_base=anchor,footprint_base=footprint)
    replay=None
    try:
        tabletop._audit_current_support(depth,K,T,cfg,SimpleNamespace(position_base=anchor,support_footprint_base=footprint),reference)
    except ValueError as exc: replay=str(exc)
    basering=points[support]@T[:3,:3].T+T[:3,3]
    frame=dict(source_stamp_ns=ns,received_ns=row['received_ns'],recorded_reason=status.get('observation_measurement',{}).get('reason',status['detail']),
        received_minus_source_sec=(row['received_ns']-ns)*1e-9, replayed_audit_rejection=replay,
        plane_camera=plane.tolist(),plane_base=current.tolist(),reference_delta=metrics(current),
        ratio=ratio,residual_mm=residual*1000,fit_sample_count=count,valid_ring_points=int(ring.sum()),
        quadrants=covered,ring_outer_radius_px=outer,ring_center_px=center.tolist(),
        support_ring_base_centroid=basering.mean(axis=0).tolist(),support_ring_base_aabb=[basering.min(axis=0).tolist(),basering.max(axis=0).tolist()],
        depth_source_topic=image['topic'],depth_bytes_sha256=image['raw_sha256'],T_base_camera=T.tolist(),
        exact_tf_lookup_stamp_ns=ns,source_file=str(path),source_file_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        published_reference_identical=bool(np.array_equal(reference,status['reference_support_plane_base'])))
    frames.append(frame)
    print(json.dumps({k:v for k,v in frame.items() if k in ['source_stamp_ns','recorded_reason','replayed_audit_rejection','reference_delta','ratio','residual_mm','quadrants']}))

timeline=[]
for row in statuses:
    d=row['data'];m=d.get('observation_measurement',{}); ns=d['source_stamp_ns']
    if ARRIVAL-100_000_000<=ns<=DEADLINE+100_000_000:
        timeline.append(dict(source_stamp_ns=ns,received_ns=row['received_ns'],reason=m.get('reason'),state=m.get('state'),
            exact_rgbd=d.get('exact_rgbd'), source_receipt_age_sec=(row['received_ns']-ns)*1e-9,source_stamps_ns=m.get('source_stamps_ns')))
report=dict(record=str(REC), arrival_stamp_ns=ARRIVAL,capture_deadline_ns=DEADLINE,
    config=asdict(cfg),intrinsics=K.tolist(),depth_scale=float(camera['depth_scale']),
    reference=dict(source_stamp_ns=initial['data']['source_stamp_ns'],plane_base=reference.tolist(),anchor_base=anchor.tolist(),footprint_base=footprint.tolist()),
    statuses=timeline,post_arrival_support_failure_count=len(failure),recorded_exact_failure_depth_frames=len(available),
    frames=frames,static_tf=tf_static,source_files=dict(tabletop=str(Path(tabletop.__file__)),tabletop_sha256=hashlib.sha256(Path(tabletop.__file__).read_bytes()).hexdigest()),
    restrictions=dict(production_changes=False,ros_master_access=False,device_commands=False,threshold_changes=False),
    limitations=['Published depth has prior camera processing. This is not unfiltered native depth.',
        'Source-exact TF validates the recorded coordinate chain, not physical hand-eye or robot calibration accuracy.',
        'A single depth camera observing a near-planar surface cannot uniquely separate depth bias, hand-eye/FK error and true surface shape.',
        'A fitted plane alone cannot authorize identity retention, registration correction or execution geometry.'])
path=OUT/'report.json'
with path.open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2);f.write('\n')
print(json.dumps(dict(report=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())))
