"""Exact sealed req19 RGB-D/mask/TF export, bounded timestamp read, no ROS IO."""
import hashlib,json
from pathlib import Path
import numpy as np
import rosbag,rospy,tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen25_handoff_analysis')
M=json.loads((D/'source_images/manifest.json').read_text());R=Path(M['record'])
sealed=json.loads((R/'manifest.json').read_text());assert sealed['state']=='complete' and sealed['recorder_closed']
ns=1791544659085737705
stamp=rospy.Time(ns//10**9,ns%10**9)
def time_ns(v):return rospy.Time(v//10**9,v%10**9)
topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask']
found={};buf=tf2_py.BufferCore(rospy.Duration(600.))
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
    for topic,msg,t in bag.read_messages(topics=topics,start_time=time_ns(ns-100000000),end_time=time_ns(ns+2000000000)):
        if msg.header.stamp.to_nsec()==ns:found[topic]=(msg,t.to_nsec())
    for topic,msg,t in bag.read_messages(topics=['/tf_static']):
        for tf in msg.transforms:buf.set_transform_static(tf,'sealed_gen25_req19')
    for topic,msg,t in bag.read_messages(topics=['/tf'],start_time=time_ns(ns-1000000000),end_time=time_ns(ns+2000000000)):
        for tf in msg.transforms:buf.set_transform(tf,'sealed_gen25_req19')
out=D/'request19_source';out.mkdir(exist_ok=True)
report=dict(source_stamp_ns=ns,record=str(R),sealed=True,found_topics={k:v[1] for k,v in found.items()},exact_original_source=True,no_segmentation=True,no_ros_calls=True)
if set(found)==set(topics):
    bridge=CvBridge();rgb=bridge.imgmsg_to_cv2(found[topics[0]][0],'bgr8');depth=bridge.imgmsg_to_cv2(found[topics[1]][0],'passthrough');mask=bridge.imgmsg_to_cv2(found[topics[2]][0],'mono8')
    def transform(parent,child):
        v=buf.lookup_transform_core(parent,child,stamp).transform;q=v.rotation;T=quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[v.translation.x,v.translation.y,v.translation.z];return T
    T=transform('base_link','camera_link');mount=transform('tool0','camera_link')
    arrays=dict(rgb=rgb,depth_raw=depth,raw_mask=mask,T_base_camera=T,tool_from_camera=mount)
    np.savez_compressed(str(out/'source.npz'),**arrays)
    audit_path=R/'keyframes_audits/e319974f04dabbffbf5bbaaa9556193b6c2978a352679318fb1f2dd41620ac24.json'
    audit=json.loads(audit_path.read_text())['selected']['observation_source_audit']
    hashes={k:hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k,v in arrays.items()}
    report.update(camera=M['camera'],source_audit=audit,array_sha256=hashes,raw_depth_hash_matches=hashes['depth_raw']==audit['raw_depth_sha256'],raw_mask_hash_matches=hashes['raw_mask']==audit['raw_mask_sha256'],npz=str(out/'source.npz'),npz_sha256=hashlib.sha256((out/'source.npz').read_bytes()).hexdigest(),limitations=['Exact source RGB-D and published mask, camera/tool transforms reconstructed from the sealed TF stream; no fabricated point cloud or future geometry. Floating point TF hash equality must be checked independently.'])
    assert report['raw_depth_hash_matches'] and report['raw_mask_hash_matches']
else:report['available']=False
(out/'manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('camera','source_audit')},ensure_ascii=False,indent=2))
