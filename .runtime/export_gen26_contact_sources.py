"""Export sealed contact inputs only; no ROS master, services or hardware."""
from pathlib import Path
import json, hashlib
import numpy as np
import rosbag, rospy, tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = EXT/'ros_log/full_latest_ros_20261009T003407Z'
RECORD = EXT/'grasp_records/20261009T114637.653111Z_26d52c38592844ad8ff668163cae0f90'
OUT = NEW/'gen26_contact_analysis/source_inputs'
SNAPSHOT = 1791546473600880622
current_sources = [1791546472940827369,1791546473312018156,SNAPSHOT]
direct = {1791546400947726249: [1791546400552541732, 1791546400749500751, 1791546400947726249]}
audits = {}
for p in (RECORD/'keyframes_audits').glob('*.json'):
    d=json.loads(p.read_text())
    if d.get('request_id') in (9,19):
        audits[d['request_id']]=d
        direct[d['snapshot_stamp_ns']]=d['pipeline_funnel']['snapshot_evidence']['source_stamp_ns']
direct[SNAPSHOT]=current_sources
sources=list(audits[19]['multiview_surface']['source_stamps_ns'])+[1791546472300270557,SNAPSHOT]
wanted=set(sources)
for x in direct.values(): wanted.update(x)
frames={};cameras=[];tf=[];static=[];geometry=[];sdk=[];accepted=[];errors=[]
topics=['/tf','/tf_static','/grasp_record/camera_runtime_metadata','/alicia_d/sdk_command',
 '/alicia_d/accepted_joint_states','/grasp_6d/object_geometry','/experiment/runtime_execution_error_snapshot',
 '/perception/object','/perception/object_mask','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']
with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
    for topic,m,t in bag.read_messages(topics=topics):
        ns=t.to_nsec()
        if topic=='/tf':tf.extend(m.transforms)
        elif topic=='/tf_static':static.extend(m.transforms)
        elif topic.endswith('camera_runtime_metadata'):cameras.append((ns,json.loads(m.data)['parameters']))
        elif topic.endswith('sdk_command'):sdk.append(m)
        elif topic.endswith('accepted_joint_states'):accepted.append(m)
        elif topic=='/grasp_6d/object_geometry':geometry.append((ns,m))
        elif topic.endswith('runtime_execution_error_snapshot'):errors.append((ns,json.loads(m.data)))
        elif m.header.stamp.to_nsec() in wanted:frames.setdefault(m.header.stamp.to_nsec(),{})[topic]=m
buffer=tf2_py.BufferCore(rospy.Duration(600.))
for m in static:buffer.set_transform_static(m,'sealed_contact_export')
for m in sorted(tf,key=lambda x:x.header.stamp.to_nsec()):buffer.set_transform(m,'sealed_contact_export')
bridge=CvBridge();OUT.mkdir(parents=True,exist_ok=True)
def stamp(m):return m.header.stamp.to_nsec()
def serialize(m):
    if hasattr(m,'__slots__'):return {k:serialize(getattr(m,k)) for k in m.__slots__}
    if isinstance(m,(list,tuple)):return [serialize(x) for x in m]
    if isinstance(m,bytes):return {'byte_count':len(m)}
    return m
rows=[]
for ns in sorted(wanted):
    f=frames.get(ns,{})
    row=dict(source_stamp_ns=ns,topics=sorted(f),complete=len(f)==4)
    if len(f)==4:
        tr=buffer.lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)).transform
        T=quaternion_matrix([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]);T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
        camera=min(cameras,key=lambda x:abs(x[0]-ns))[1]
        obj=f['/perception/object'];q=min(accepted,key=lambda m:abs(stamp(m)-ns))
        dest=OUT/(str(ns)+'.npz')
        np.savez_compressed(dest,rgb=bridge.imgmsg_to_cv2(f['/supervisor/camera/color/image_raw'],'bgr8'),
            depth_raw=bridge.imgmsg_to_cv2(f['/supervisor/camera/depth/image_raw'],'passthrough'),
            raw_mask=bridge.imgmsg_to_cv2(f['/perception/object_mask'],'mono8'),T_base_camera=T,
            q=np.array(q.position[:6]))
        row.update(npz=str(dest),npz_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),
            camera=camera,object=serialize(obj),accepted_stamp_ns=stamp(q))
    rows.append(row)
selected_geometry=[dict(received_ns=ns,message=serialize(m)) for ns,m in geometry if stamp(m) in wanted]
joint_window=lambda messages:[serialize(m) for m in messages if 1791546470000000000<=stamp(m)<=1791546525000000000]
report=dict(record=str(RECORD),request_id=28,snapshot_stamp_ns=SNAPSHOT,direct_snapshot_sources=direct,
    surface_source_stamps_ns=sources,frames=rows,geometry=selected_geometry,
    sdk_command=joint_window(sdk),accepted=joint_window(accepted),
    execution_error_snapshots=[dict(received_ns=ns,message=m) for ns,m in errors if 1791546470000000000<=ns<=1791546525000000000],
    target_epoch=audits[19]['target_epoch'],target_track_id=audits[19]['observe_return']['target_track_id'],
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    limitations=['No final request28 candidate audit or failed MoveIt trajectory was published before cancellation.',
      'Exported live ObjectGeometry can validate a reconstructed source; no execution authority.'])
(OUT/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(path=str(OUT/'manifest.json'),complete=sum(r['complete'] for r in rows),total=len(rows),
    geometry_stamps=[r['message']['header']['stamp'] for r in selected_geometry])))
