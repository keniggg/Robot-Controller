"""Bounded exact closed-record input extraction; no ROS initialization or control."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
import sys
sys.dont_write_bytecode = True
from pathlib import Path
import hashlib
import json
import time
import resource
import importlib.util

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R = E/'grasp_records/20261022T060615.487250Z_8db865e4c1094eb1bbdc72457cb35e2e'
S = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/actual_attempt_r1'
D = S/'failed_seed_inputs_r1'
INITIAL = 1792649186302586078
FAILED = 1792649244237768888
TOPICS = ['/tf', '/tf_static', '/alicia_d/accepted_joint_states',
    '/grasp_record/camera_runtime_metadata', '/supervisor/camera/color/image_raw',
    '/supervisor/camera/depth/image_raw', '/perception/object_mask',
    '/experiment/observation_rgbd/color', '/experiment/observation_rgbd/depth',
    '/experiment/observation_rgbd/metadata']

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def val(x):
    if hasattr(x, 'to_nsec'): return dict(secs=x.secs, nsecs=x.nsecs, ns=x.to_nsec())
    if hasattr(x, '__slots__'): return {k: val(getattr(x, k)) for k in x.__slots__}
    if isinstance(x, dict): return {k: val(v) for k, v in x.items()}
    if isinstance(x, (tuple, list)): return [val(v) for v in x]
    return x
def identity(p):
    s = p.stat()
    return dict(device=s.st_dev, inode=s.st_ino, size=s.st_size, mtime_ns=s.st_mtime_ns)
def write(name, d):
    p=D/name
    with p.open('x') as f: json.dump(d, f, ensure_ascii=False, allow_nan=False, indent=2);f.write('\n')
    return dict(path=str(p), sha256=sha(p))
def guard(event,args):
    if event in ('socket.connect','socket.bind','socket.sendto','os.system'):
        raise RuntimeError('offline extraction rejects '+event)
sys.addaudithook(guard)

def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    m=json.loads((R/'manifest.json').read_bytes())
    assert m['state']=='complete' and m['recorder_closed']
    bagfile=R/'keyframes.bag';before=identity(bagfile)
    declared=next(row for row in m['files'] if row['path']=='keyframes.bag')
    assert before['size']==declared['size_bytes']
    D.mkdir(exist_ok=False)
    pins={str(p):sha(p) for p in (Path(__file__),R/'manifest.json',R/'recorder_outcome.json',R/'perception_execution_configuration.json')}
    sys.path[:0]=['/opt/ros/noetic/lib/python3/dist-packages',str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_flexible_grasp_supervisor/src')]
    import rosbag,genpy,tf2_py,cv2,numpy as np
    from cv_bridge import CvBridge
    from alicia_grasp_modes.rgb_observation_source import bracket_feedback,camera_geometry_snapshot
    from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
    cv2.setNumThreads(1)
    spec=importlib.util.spec_from_file_location('closed_record_transformations','/opt/ros/noetic/lib/python3/dist-packages/tf/transformations.py')
    transformations=importlib.util.module_from_spec(spec);spec.loader.exec_module(transformations)
    images={};masks={};metadata={};cams=[];static=[];dynamic=[];sdk=[];counts={}
    start=time.monotonic();usage=resource.getrusage(resource.RUSAGE_SELF)
    wanted=lambda ns:ns==INITIAL or FAILED-1_000_000_000<=ns<=FAILED+750_000_000
    with rosbag.Bag(str(bagfile),'r') as bag:
        info=bag.get_type_and_topic_info().topics
        write('message_contract.json',{t:dict(type=info[t].msg_type,count=info[t].message_count) for t in TOPICS if t in info})
        for topic,msg,stamp in bag.read_messages(topics=TOPICS):
            counts[topic]=counts.get(topic,0)+1;receipt=stamp.to_nsec()
            if topic in ('/tf','/tf_static'):
                (static if topic=='/tf_static' else dynamic).extend(msg.transforms);continue
            if topic=='/alicia_d/accepted_joint_states':
                assert msg.header.frame_id=='sdk_measured'
                joints=dict(zip(msg.name,msg.position));sdk.append((msg.header.stamp.to_nsec(),[joints['Joint%d'%i] for i in range(1,7)]));continue
            if topic=='/grasp_record/camera_runtime_metadata':
                d=json.loads(msg.data)
                if d.get('parameters'):cams.append(dict(bag_receipt_ns=receipt,data=d))
                continue
            if topic=='/experiment/observation_rgbd/metadata':
                d=json.loads(msg.data)
                if wanted(d['source_stamp_ns']):metadata[d['source_stamp_ns']]=dict(bag_receipt_ns=receipt,data=d)
                continue
            ns=msg.header.stamp.to_nsec()
            if not wanted(ns):continue
            if topic=='/perception/object_mask':masks[ns]=msg;continue
            key='color' if '/color/' in topic or topic.endswith('/color') else 'depth'
            old=images.setdefault(ns,{}).get(key)
            if old is not None:assert old[0].data==msg.data
            images[ns][key]=(msg,topic,receipt)
    assert cams and len(images)<=24
    geometry=camera_geometry_snapshot(cams[0]['data']['parameters'])
    assert all(camera_geometry_snapshot(c['data']['parameters'])==geometry for c in cams)
    buffer=tf2_py.BufferCore(genpy.Duration(600))
    for tr in static:buffer.set_transform_static(tr,'sealed_gen60_record')
    for tr in sorted(dynamic,key=lambda tr:tr.header.stamp.to_nsec()):buffer.set_transform(tr,'sealed_gen60_record')
    rows=[];bridge=CvBridge()
    for ns,pair in sorted(images.items()):
        assert set(pair)=={'color','depth'}
        c,d=pair['color'][0],pair['depth'][0]
        assert c.header.stamp==d.header.stamp and c.header.frame_id==d.header.frame_id=='camera_link'
        rgb=bridge.imgmsg_to_cv2(c,'bgr8');raw=bridge.imgmsg_to_cv2(d,'passthrough')
        camrow=max((r for r in cams if r['bag_receipt_ns']<=pair['color'][2]),key=lambda r:r['bag_receipt_ns']);cam=camrow['data']['parameters']
        tfmsg=buffer.lookup_transform_core('base_link','camera_link',genpy.Time(ns//10**9,ns%10**9));tr=tfmsg.transform
        T=transformations.quaternion_matrix([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]);T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
        q,lo,hi=bracket_feedback(sdk,ns,1792639534222874592)
        payload=dict(color=rgb,depth_raw=raw,depth_scale=float(cam['depth_scale']) if d.encoding!='32FC1' else 1.,K=np.array([cam[k] for k in ('fx','fy','cx','cy')]),T_base_camera=T,q=q,source_stamp_ns=np.int64(ns))
        valid=rectification_rgb_valid_mask(cam,rgb.shape[:2])
        if valid is not None:payload['rgb_valid']=valid
        if ns in masks:payload['recorded_refined_mask']=bridge.imgmsg_to_cv2(masks[ns],'mono8')
        p=D/(str(ns)+'.npz');png=D/(str(ns)+'_rgb.png');np.savez_compressed(p,**payload);assert cv2.imwrite(str(png),rgb)
        rows.append(dict(source_stamp_ns=ns,path=str(p),npz_sha256=sha(p),rgb_png=str(png),rgb_png_sha256=sha(png),exact_failed_source=ns==FAILED,initial_source=ns==INITIAL,exact_tf=val(tfmsg),feedback_before=lo,feedback_after=hi,camera_geometry=geometry,color_topic=pair['color'][1],depth_topic=pair['depth'][1],color_bag_receipt_ns=pair['color'][2],depth_bag_receipt_ns=pair['depth'][2],color_header=val(c.header),depth_header=val(d.header),color_wire_data_sha256=hashlib.sha256(c.data).hexdigest(),depth_wire_data_sha256=hashlib.sha256(d.data).hexdigest(),recorded_mask_available=ns in masks,observation_metadata=metadata.get(ns)))
    assert INITIAL in images and FAILED in images
    assert identity(bagfile)==before and all(sha(p)==h for p,h in pins.items())
    camera=write('camera_metadata.json',cams)
    config=write('unknown_perception_configuration.json',json.loads((R/'perception_execution_configuration.json').read_bytes())['parameters']['/unknown_perception'])
    u=resource.getrusage(resource.RUSAGE_SELF)
    result=dict(record=str(R),rows=rows,camera_metadata=camera,configuration=config,source_pins=pins,bag_identity=before,manifest_bag_sha256=declared['sha256'],full_bag_rehashed=False,exact_requested_sources=[INITIAL,FAILED],missing_requested_sources=[],nearest_substitution=False,message_counts=counts,ROS_initialized=False,motion_commands_sent=False,wall_seconds=time.monotonic()-start,user_cpu_seconds=u.ru_utime-usage.ru_utime,system_cpu_seconds=u.ru_stime-usage.ru_stime,max_rss_kib=u.ru_maxrss)
    print(json.dumps(write('manifest.json',result),ensure_ascii=False))

if __name__=='__main__':main()
