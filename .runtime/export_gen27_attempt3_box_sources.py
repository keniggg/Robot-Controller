"""Offline sealed RGBD export and qualification; no ROS/device commands."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import rosbag
import rospy
import tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_top
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate, deproject_depth
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity

E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R = E / 'grasp_records/20261015T075503.791411Z_24849e8304a44341adebca3867d29fdd'
OUT = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen27_attempt3_initial_inference_review'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert json.loads((R/'manifest.json').read_text())['recorder_closed']
targets = {1792050907106353521, 1792050907639824151, 1792050956912783622}
audits = {}
for path in (R/'keyframes_audits').glob('*.json'):
    value = json.loads(path.read_text())
    if value.get('snapshot_stamp_ns') in targets and value.get('replay_geometry',{}).get('available'):
        audits[value['snapshot_stamp_ns']] = (path, value)
def tm(ns): return rospy.Time(ns//10**9, ns%10**9)
topics = ['/supervisor/camera/color/image_raw', '/supervisor/camera/depth/image_raw', '/perception/object_mask']
frames = {ns:{} for ns in targets}
geometries = {ns:[] for ns in targets}
buf = tf2_py.BufferCore(rospy.Duration(600.))
cams = []
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
    for topic,msg,received in bag.read_messages(topics=topics+['/grasp_6d/object_geometry'], start_time=tm(min(targets)-100000000), end_time=tm(max(targets)+3000000000)):
        ns=msg.header.stamp.to_nsec()
        if ns in targets:
            if topic == '/grasp_6d/object_geometry': geometries[ns].append((msg,received.to_nsec()))
            else: frames[ns][topic]=(msg,received.to_nsec())
    for topic,msg,received in bag.read_messages(topics=['/tf_static','/grasp_record/camera_runtime_metadata']):
        if topic == '/tf_static':
            for tr in msg.transforms: buf.set_transform_static(tr,'sealed_attempt3')
        else:
            data=json.loads(msg.data)
            if data.get('parameters'): cams.append((received.to_nsec(),data['parameters']))
    for _,msg,_ in bag.read_messages(topics=['/tf'],start_time=tm(min(targets)-1000000000),end_time=tm(max(targets)+1000000000)):
        for tr in msg.transforms: buf.set_transform(tr,'sealed_attempt3')
OUT.mkdir(parents=True,exist_ok=True)
bridge=CvBridge()
rows=[]
for ns,pairs in sorted(frames.items()):
    row=dict(source_stamp_ns=ns,available_topics=sorted(pairs))
    if set(pairs) != set(topics): rows.append(row);continue
    color=bridge.imgmsg_to_cv2(pairs[topics[0]][0],'bgr8')
    depth=bridge.imgmsg_to_cv2(pairs[topics[1]][0],'passthrough')
    mask=bridge.imgmsg_to_cv2(pairs[topics[2]][0],'mono8')
    camera=max((c for c in cams if c[0]<=pairs[topics[0]][1]),default=cams[0],key=lambda c:c[0])[1]
    frame_id=pairs[topics[1]][0].header.frame_id
    tr=buf.lookup_transform_core('base_link',frame_id,tm(ns)).transform
    T=quaternion_matrix([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w])
    T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
    O=np.eye(4);O[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
    Topt=T if 'optical' in frame_id.lower() else T@O
    intrinsics=np.array([camera[k] for k in ('fx','fy','cx','cy')])
    arrays=dict(color=color,depth_raw=depth,object_mask=mask,T_base_camera=T,T_base_optical=Topt,
                intrinsics=intrinsics,depth_scale_m=np.array(camera['depth_scale']),source_stamp_ns=np.array(ns))
    p=OUT/('source_%d.npz'%ns);assert not p.exists();np.savez_compressed(str(p),**arrays)
    cv2.imwrite(str(OUT/('source_%d_color.png'%ns)),color)
    cv2.imwrite(str(OUT/('source_%d_mask.png'%ns)),mask)
    row.update(npz=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),frame_id=frame_id,
               T_base_camera_is_optical=('optical' in frame_id.lower()),T_base_optical_is_optical=True,
               array_sha256={k:hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k,v in arrays.items()},
               camera=camera,received_ns={k:v[1] for k,v in pairs.items()})
    if ns in audits:
        ap,a=audits[ns];g=a['replay_geometry']
        row.update(geometry_source='exact_source_replay_geometry_audit',audit=str(ap))
        identity=TargetTrackIdentity.from_stream(a['generation'],a['target_epoch'])
    else:
        msgs=[x for x in geometries[ns] if x[0].valid]
        row['exact_object_geometry_message_count']=len(msgs)
        if not msgs: rows.append(row);continue
        msg,rec=msgs[0];p0=msg.pose_base.position;q=msg.pose_base.orientation
        rot=quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
        points,_=deproject_depth(depth,mask>0,intrinsics,float(camera['depth_scale']))
        points=points@Topt[:3,:3].T+Topt[:3,3]
        g=dict(obb_center_base_m=[p0.x,p0.y,p0.z],R_base_obb=rot.tolist(),
               obb_size_xyz_m=[msg.size_xyz_m.x,msg.size_xyz_m.y,msg.size_xyz_m.z],
               support_normal_base=[msg.support_normal_base.x,msg.support_normal_base.y,msg.support_normal_base.z],
               support_offset_m=msg.support_offset_m,support_inlier_ratio=msg.support_inlier_ratio,
               object_points_base_m=points.tolist(),source_mode=msg.source_mode)
        row.update(geometry_source='same_stamp_ObjectGeometry_and_raw_deprojected_points_proxy',
                   limitation='Support offset/inlier ratio serialized float32; raw target points are not original cleaned cloud; this is an offline replay, not retained original inference audit.',
                   geometry_received_ns=rec,target_track_id=msg.target_track_id)
        identity=TargetTrackIdentity.from_stream(11,12)
    gp=OUT/('geometry_%d.json'%ns);assert not gp.exists();gp.write_text(json.dumps(g,ensure_ascii=False,indent=2)+'\n')
    row['geometry_json']=str(gp)
    geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g.get('support_inlier_ratio',1),g['object_points_base_m'],g.get('source_mode','instance_mask'))
    result=qualify_regular_box_top(depth,mask,intrinsics,float(camera['depth_scale']),Topt,geometry,identity,ns,color_bgr=color)
    row['qualification']=dict(ok=result.ok,code=result.code,audit=dict(result.audit))
    if result.ok: row['model']=result.model.to_evidence_dict()
    rows.append(row)
    print(json.dumps({k:v for k,v in row.items() if k in ('source_stamp_ns','geometry_source','qualification','npz','geometry_json')},ensure_ascii=False),flush=True)
report=dict(record=str(R),rows=rows,no_ros_calls=True,no_hardware_access=True,no_temperature_reads=True)
p=OUT/'source_manifest.json';assert not p.exists();p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(manifest=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())))
