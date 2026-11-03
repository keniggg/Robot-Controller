"""Export only exact sealed gen29 sources for independent fusion/FK diagnostics.
No ROS master, mutation, temperature, segmentation or neighboring image substitution.
"""
from pathlib import Path
import json, hashlib
import numpy as np
import rosbag, rospy, tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=EXT/'ros_log/full_latest_ros_20261009T003407Z'
RECORD=EXT/'grasp_records/20261009T124424.224503Z_ee2856eb8dab4529a6d029babf69f7a5'
OUT=NEW/'gen29_contact_analysis/source_inputs'
SNAPSHOT=1791549928519117355
FK_SOURCE=1791549916246385812
manifest=json.loads((RECORD/'manifest.json').read_text())
assert manifest['state']=='complete' and manifest['recorder_closed'] is True
assert not (OUT/'manifest.json').exists(), 'Do not overwrite completed evidence'
audits={}
for p in (RECORD/'keyframes_audits').glob('*.json'):
    d=json.loads(p.read_text())
    if d.get('request_id') in (12,18,19,24):audits[d['request_id']]=(p,d)
assert set(audits)=={12,18,19,24}
sources=audits[24][1]['multiview_surface']['source_stamps_ns']
assert len(sources)==7 and sources[-1]==SNAPSHOT
initial=sources[0]
direct={initial:[1791549867109199047,1791549867308165550,initial]}
for req in (12,18,24):
    a=audits[req][1];direct[a['snapshot_stamp_ns']]=a['pipeline_funnel']['snapshot_evidence']['source_stamp_ns']
wanted=set(sources)|{FK_SOURCE}
for items in direct.values():wanted.update(items)
def T(ns):return rospy.Time(ns//10**9,ns%10**9)
def stamp(m):return m.header.stamp.to_nsec()
def serialize(m):
    if hasattr(m,'__slots__'):return {k:serialize(getattr(m,k)) for k in m.__slots__}
    if isinstance(m,(list,tuple)):return [serialize(x) for x in m]
    if isinstance(m,bytes):return {'byte_count':len(m)}
    return m
image_topics=['/perception/object','/perception/object_mask','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']
frames={};cameras=[];geometry=[];accepted=[]
buffer=tf2_py.BufferCore(rospy.Duration(120.))
with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
    for _,m,_ in bag.read_messages(topics=['/tf_static']):
        for tr in m.transforms:buffer.set_transform_static(tr,'sealed_gen29_export')
    for topic,m,t in bag.read_messages(topics=image_topics+['/tf','/grasp_record/camera_runtime_metadata','/alicia_d/accepted_joint_states','/grasp_6d/object_geometry'],start_time=T(min(wanted)-1000000000),end_time=T(max(wanted)+1500000000)):
        ns=t.to_nsec()
        if topic=='/tf':
            for tr in m.transforms:buffer.set_transform(tr,'sealed_gen29_export')
        elif topic.endswith('camera_runtime_metadata'):cameras.append(dict(received_ns=ns,message=json.loads(m.data)))
        elif topic.endswith('accepted_joint_states'):
            if m.header.frame_id=='sdk_measured':accepted.append(dict(received_ns=ns,message=m))
        elif topic=='/grasp_6d/object_geometry':
            if stamp(m) in wanted:geometry.append(dict(received_ns=ns,message=serialize(m)))
        elif stamp(m) in wanted:frames.setdefault(stamp(m),{})[topic]=(m,ns)
statuses=json.loads((NEW/'gen29_handoff_analysis/detector_status.json').read_text())
bridge=CvBridge();OUT.mkdir(parents=True,exist_ok=True)
def matrix(parent,child,ns):
    tr=buffer.lookup_transform_core(parent,child,T(ns)).transform
    a=quaternion_matrix([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]);a[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z];return a
rows=[]
for ns in sorted(wanted):
    f=frames.get(ns,{})
    row=dict(source_stamp_ns=ns,topics=sorted(f),topic_received_ns={k:v[1] for k,v in f.items()},complete=all(k in f for k in image_topics))
    if row['complete']:
        camera_row=min(cameras,key=lambda x:abs(x['received_ns']-ns));camera=camera_row['message']['parameters']
        before=max((r for r in accepted if stamp(r['message'])<=ns),key=lambda r:stamp(r['message']))
        after=min((r for r in accepted if stamp(r['message'])>=ns),key=lambda r:stamp(r['message']))
        q=min((before,after),key=lambda r:abs(stamp(r['message'])-ns))
        arrays=dict(rgb=bridge.imgmsg_to_cv2(f['/supervisor/camera/color/image_raw'][0],'bgr8'),depth_raw=bridge.imgmsg_to_cv2(f['/supervisor/camera/depth/image_raw'][0],'passthrough'),raw_mask=bridge.imgmsg_to_cv2(f['/perception/object_mask'][0],'mono8'),T_base_camera=matrix('base_link','camera_link',ns),tool_from_camera=matrix('tool0','camera_link',ns),q=np.array(q['message'].position[:6]))
        dest=OUT/(str(ns)+'.npz');np.savez_compressed(dest,**arrays)
        ready=[r for r in statuses if r['message'].get('source_stamp_ns')==ns and r['message'].get('state')=='ready']
        row.update(npz=str(dest),npz_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),array_sha256={k:hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k,v in arrays.items()},camera=camera,camera_metadata=camera_row,object=serialize(f['/perception/object'][0]),accepted_stamp_ns=stamp(q['message']),accepted_bracket={k:dict(received_ns=r['received_ns'],message=serialize(r['message'])) for k,r in [('before',before),('after',after)]},ready_status=ready)
    rows.append(row)
report=dict(record=str(RECORD),record_manifest_sha256=hashlib.sha256((RECORD/'manifest.json').read_bytes()).hexdigest(),request_id=24,snapshot_stamp_ns=SNAPSHOT,direct_snapshot_sources=direct,surface_source_stamps_ns=sources,additional_fk_source_stamp_ns=FK_SOURCE,frames=rows,geometry=geometry,accepted=[serialize(r['message']) for r in accepted],target_epoch=audits[24][1]['target_epoch'],target_track_id=audits[24][1]['observe_return']['target_track_id'],audit_sources={str(req):dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),snapshot_stamp_ns=a['snapshot_stamp_ns']) for req,(p,a) in audits.items()},script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),no_ros_calls=True,no_segmentation=True,limitations=['Exact recorded source stamps only; absent frames remain absent, no neighboring images substituted.','Camera parameter metadata is periodic: exact original read/receipt interval is preserved, not per-frame SDK filter authority.','q is the nearest of the real accepted source bracket, not an invented command or interpolated feedback.','TF uses original sealed transform interpolation; floating arithmetic hashes may differ from online cached transforms and must be checked before claims of bitexact geometry.','Initial reference uses the original three sample sources; reached phase sources use their original single frame; each direct contact snapshot uses its original recorded three sample list.','This export confers no live source or execution authority.'])
f=OUT/'manifest.json';f.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(f),sha256=hashlib.sha256(f.read_bytes()).hexdigest(),complete=sum(r['complete'] for r in rows),total=len(rows),missing=[r['source_stamp_ns'] for r in rows if not r['complete']],direct=direct,geometry_stamps=[r['message']['header']['stamp'] for r in geometry],fk=next(r for r in rows if r['source_stamp_ns']==FK_SOURCE)),ensure_ascii=False)[:6500])
