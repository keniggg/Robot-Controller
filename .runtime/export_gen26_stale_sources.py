"""Two exact sealed source checks. Missing pixels remain missing, never replaced."""
from pathlib import Path
import json,hashlib
import numpy as np
import rosbag,rospy,tf2_py,cv2
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z')
D=B/'gen26_handoff_analysis';A=json.loads((D/'arrival_and_terminal.json').read_text());R=Path(A['record'])
assert A['sealed_manifest']['state']=='complete' and A['sealed_manifest']['recorder_closed']
sources=[1791546527447860002,1791546527959462165]
def T(ns):return rospy.Time(ns//10**9,ns%10**9)
topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/perception/object']
frames={ns:{} for ns in sources};joints=[];camera=[]
buf=tf2_py.BufferCore(rospy.Duration(30.))
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
    for _,m,_ in bag.read_messages(topics=['/tf_static']):
        for t in m.transforms:buf.set_transform_static(t,'sealed_gen26')
    for topic,m,receipt in bag.read_messages(topics=topics+['/tf','/alicia_d/accepted_joint_states','/grasp_record/camera_runtime_metadata'],start_time=T(min(sources)-1000000000),end_time=T(max(sources)+3000000000)):
        if topic=='/tf':
            for t in m.transforms:buf.set_transform(t,'sealed_gen26')
        elif topic.endswith('accepted_joint_states'):
            if m.header.frame_id=='sdk_measured':joints.append(dict(source_stamp_ns=m.header.stamp.to_nsec(),positions=list(m.position),names=list(m.name),bag_received_ns=receipt.to_nsec()))
        elif topic.endswith('camera_runtime_metadata'):
            camera.append(dict(bag_received_ns=receipt.to_nsec(),message=json.loads(m.data)))
        elif m.header.stamp.to_nsec() in frames:frames[m.header.stamp.to_nsec()][topic]=(m,receipt.to_nsec())
def plain(m):
    if hasattr(m,'__slots__'):return {k:plain(getattr(m,k)) for k in m.__slots__}
    if isinstance(m,(list,tuple)):return [plain(x) for x in m]
    return m
def matrix(parent,child,ns):
    v=buf.lookup_transform_core(parent,child,T(ns)).transform;q=v.rotation
    m=quaternion_matrix([q.x,q.y,q.z,q.w]);m[:3,3]=[v.translation.x,v.translation.y,v.translation.z];return m
bridge=CvBridge();out=D/'stale_source_images';out.mkdir(exist_ok=True)
statuses=json.loads((D/'detector_status.json').read_text())
rows=[]
for ns,frame in frames.items():
    before=max((j for j in joints if j['source_stamp_ns']<=ns),key=lambda j:j['source_stamp_ns'])
    after=min((j for j in joints if j['source_stamp_ns']>=ns),key=lambda j:j['source_stamp_ns'])
    row=dict(source_stamp_ns=ns,role='last_good' if ns==sources[0] else 'first_loss',available_topics={k:v[1] for k,v in frame.items()},missing_topics=[x for x in topics if x not in frame],accepted_bracket=dict(before=before,after=after),detector_status=next(s for s in statuses if s['message'].get('source_stamp_ns')==ns),camera_metadata=min(camera,key=lambda c:abs(c['bag_received_ns']-ns)))
    arrays=dict(T_base_camera=matrix('base_link','camera_link',ns),tool_from_camera=matrix('tool0','camera_link',ns))
    for topic,key,encoding in zip(topics,('rgb','depth_raw','refined_mask','object'),('bgr8','passthrough','mono8',None)):
        if topic not in frame:continue
        if encoding is None:row['object_message']=plain(frame[topic][0]);continue
        arrays[key]=bridge.imgmsg_to_cv2(frame[topic][0],encoding)
        if key=='rgb':cv2.imwrite(str(out/(str(ns)+'_rgb.png')),arrays[key])
        if key=='refined_mask':row['mask_pixels']=int(np.count_nonzero(arrays[key]))
    row['exact_rgbd_available']='rgb' in arrays and 'depth_raw' in arrays
    if not row['exact_rgbd_available']:row['limitation']='Original failed source pixels absent from saved keyframes; no neighboring pixels substituted. Available TF/status/mask are not a full pixel replay.'
    path=out/(str(ns)+'.npz');np.savez_compressed(str(path),**arrays)
    row.update(npz=str(path),npz_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),array_sha256={k:hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k,v in arrays.items()})
    rows.append(row)
report=dict(record=str(R),record_manifest_sha256=A['manifest_sha256'],no_ros_calls=True,no_segmentation=True,sources=rows,limitations=['Only exact requested sources; original absent RGB/depth cannot be reconstructed from diagnostic metadata.','Camera parameter read is periodic, with original receipt preserved; no per-frame SDK-filter status is invented.'])
p=out/'manifest.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),sources=[{k:v for k,v in r.items() if k in ('source_stamp_ns','role','missing_topics','exact_rgbd_available','mask_pixels','npz','npz_sha256')} for r in rows]),ensure_ascii=False,indent=2))
