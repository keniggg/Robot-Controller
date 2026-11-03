"""Small, exact sealed RGB-D export; never starts a ROS node or synthesizes pixels."""
from pathlib import Path
import json,hashlib
import numpy as np
import rosbag,rospy,tf2_py,cv2
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen32_handoff_analysis')
A=json.loads((D/'arrival_details.json').read_text());R=Path(A['record']);V=A['views'][0]
assert A['sealed_manifest']['state']=='complete' and A['sealed_manifest']['recorder_closed']
anchor=A['first_loss']['message']['anchor_source_stamp_ns'];last=A['first_loss']['message']['first_loss']['last_good_source_stamp_ns']
T=lambda ns:rospy.Time(ns//10**9,ns%10**9)
rgbtopic='/supervisor/camera/color/image_raw';depthtopic='/supervisor/camera/depth/image_raw'
special={'/experiment/observation_rgbd/color':rgbtopic,'/experiment/observation_rgbd/depth':depthtopic}
masktopic='/perception/object_mask';objecttopic='/perception/object'
online={r['message'].get('source_stamp_ns'):r for r in V['detector_status_full_payloads'] if r['message'].get('observation_measurement',{}).get('reason')=='arrival_unique_original_identity_required'}
available={}
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
    origins={}
    for topic,m,receipt in bag.read_messages(topics=[rgbtopic,depthtopic]+list(special),start_time=T(V['capture_start_ns']),end_time=T(V['capture_deadline_ns']+1000000000)):
        ns=m.header.stamp.to_nsec()
        if ns in online:
            canonical=special.get(topic,topic)
            available.setdefault(ns,{})[canonical]=(m,receipt.to_nsec())
            origins.setdefault(ns,{})[canonical]=topic
    exact=sorted(ns for ns,f in available.items() if rgbtopic in f and depthtopic in f)
    assert exact,'No exact evaluated arrival RGB-D saved'
    chosen=sorted(set([exact[0],exact[len(exact)//2],exact[-1]]))
    frames={ns:{} for ns in [anchor,last]+chosen}
    for ns in chosen:frames[ns].update(available[ns])
    statuses=[];camera=[];joints=[];buf=tf2_py.BufferCore(rospy.Duration(100.))
    for _,m,_ in bag.read_messages(topics=['/tf_static']):
        for tf in m.transforms:buf.set_transform_static(tf,'sealed_gen32')
    for topic,m,receipt in bag.read_messages(topics=['/tf','/alicia_d/accepted_joint_states','/grasp_record/camera_runtime_metadata','/perception/unknown/detector_status'],start_time=T(anchor-1000000000),end_time=T(max(chosen)+1500000000)):
        if topic=='/tf':
            for tf in m.transforms:buf.set_transform(tf,'sealed_gen32')
        elif topic.endswith('accepted_joint_states'):
            if m.header.frame_id=='sdk_measured':joints.append(dict(source_stamp_ns=m.header.stamp.to_nsec(),positions=list(m.position),names=list(m.name),bag_received_ns=receipt.to_nsec()))
        elif topic.endswith('camera_runtime_metadata'):camera.append(dict(received_ns=receipt.to_nsec(),message=json.loads(m.data)))
        else:
            payload=json.loads(m.data)
            if payload.get('source_stamp_ns') in frames:statuses.append(dict(received_ns=receipt.to_nsec(),message=payload))
    # Each reference window is narrow; no unrelated image range is exported.
    for ns in frames:
        for topic,m,receipt in bag.read_messages(topics=[rgbtopic,depthtopic,masktopic,objecttopic],start_time=T(ns-10000000),end_time=T(ns+1000000000)):
            if m.header.stamp.to_nsec()==ns:frames[ns][topic]=(m,receipt.to_nsec())
def matrix(parent,child,ns):
    tf=buf.lookup_transform_core(parent,child,T(ns)).transform;q=tf.rotation
    result=quaternion_matrix([q.x,q.y,q.z,q.w]);result[:3,3]=[tf.translation.x,tf.translation.y,tf.translation.z];return result
out=D/'matching_source_images';out.mkdir(exist_ok=True);assert not (out/'manifest.json').exists()
bridge=CvBridge();rows=[]
for ns,f in frames.items():
    role='anchor' if ns==anchor else 'last_pre_motion' if ns==last else 'arrival'
    row=dict(source_stamp_ns=ns,role=role,online_evaluated=ns in online,detector_status=next((s for s in statuses if s['message'].get('state')=='ready'),None) if role!='arrival' else online[ns])
    # Exact source is mandatory even when reference arrival ordering differs.
    row['detector_status']=next((s for s in statuses if s['message'].get('source_stamp_ns')==ns and s['message'].get('state')=='ready'),None) if role!='arrival' else online[ns]
    row['camera_metadata']=min(camera,key=lambda c:abs(c['received_ns']-ns))
    row['accepted_bracket']={k:fn((j for j in joints if cond(j['source_stamp_ns'],ns)),key=lambda j:j['source_stamp_ns']) for k,fn,cond in [('before',max,lambda a,b:a<=b),('after',min,lambda a,b:a>=b)]}
    arrays=dict(T_base_camera=matrix('base_link','camera_link',ns),tool_from_camera=matrix('tool0','camera_link',ns))
    for topic,key,encoding in [(rgbtopic,'rgb','bgr8'),(depthtopic,'depth_raw','passthrough'),(masktopic,'refined_mask','mono8')]:
        if topic in f:arrays[key]=bridge.imgmsg_to_cv2(f[topic][0],encoding)
    assert 'rgb' in arrays and 'depth_raw' in arrays,(ns,'missing exact source')
    cv2.imwrite(str(out/(str(ns)+'_rgb.png')),arrays['rgb'])
    path=out/(str(ns)+'.npz');np.savez_compressed(str(path),**arrays)
    row.update(npz=str(path),npz_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),array_sha256={k:hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k,v in arrays.items()},available_topics={k:v[1] for k,v in f.items()},original_rgbd_topics=origins.get(ns,{rgbtopic:rgbtopic,depthtopic:depthtopic}))
    rows.append(row)
report=dict(record=str(R),record_manifest_sha256=A['manifest_sha256'],no_ros_calls=True,no_segmentation=True,all_saved_online_arrival_sources=exact,sources=rows,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),limitations=['Only exact saved sources selected at first/middle/last of saved evaluated arrivals; absent online image sources remain absent.','TF reconstructed using recorded exact timestamp and ROS transform math; per-source joint bracket and periodic camera metadata retained.','Premotion source bound by actual terminal last_good_source; not a separately dumped in-memory hold object.'])
p=out/'manifest.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(path=str(p),sources=[(r['source_stamp_ns'],r['role']) for r in rows],saved_exact=exact),ensure_ascii=False))
