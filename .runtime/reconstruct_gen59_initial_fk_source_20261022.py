"""Recover one sealed GEN58 pre-observation source, with exact hash checks."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
from pathlib import Path
import sys,json,hashlib,socket,time,itertools,io
W=Path(__file__).resolve().parents[1]
B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=B/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1'
R=B/'grasp_records/20261022T032906.414053Z_6a8a761e427d48ff9e4c14741837f7a3'
O=N/'observation_pose_fix/offline_selection_r1/source_attempt1'
assert Path('/media/zhuyupei/SanStand').is_mount()
O.mkdir(parents=True,exist_ok=True);(O/'genpy_tmp').mkdir(exist_ok=True);os.environ['TMPDIR']=str(O/'genpy_tmp')
sys.path[:0]=[str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages',str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_flexible_grasp_supervisor/src')]
blocked=[]
class OfflineSocket(socket.socket):
    def __new__(cls,*a,**kw):
        blocked.append('socket');raise RuntimeError('offline sockets prohibited')
socket.socket=OfflineSocket
import numpy as np
import cv2
cv2.setNumThreads(1)
import rosbag,rospy,tf
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.observation_arrival_projection import capture_arrival_projection
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
sha=lambda b:hashlib.sha256(b).hexdigest()
def arrsha(a): return sha(np.ascontiguousarray(a).tobytes())
def save(name,data): (O/name).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
def rigid(t,q):
    q=np.asarray(q,float);q/=np.linalg.norm(q);out=quaternion_matrix(q);out[:3,3]=t;return out
ap=R/'keyframes_audits/f50e175f19ee1a7b016b822f5275e1f8a6854db2c642095b2face9334c91d96e.json'
assert sha(ap.read_bytes())==ap.stem
a=json.loads(ap.read_text());ns=a['snapshot_stamp_ns'];assert ns==1792639754162577867
ca=a['selected']['observation_source_audit'];view=a['selected']['observation_view'];search=view['proposal_search']
bagpath=R/'keyframes.bag';stat=bagpath.stat();manifest=json.loads((R/'manifest.json').read_text())
assert manifest['state']=='complete' and manifest['recorder_closed'] is True
transformer=tf.Transformer(True,rospy.Duration(200));static=[];arrays={};image_rows={};status=None;status_wire=None;counts={}
topics=['/tf','/perception/unknown/detector_status','/supervisor/camera/depth/image_raw','/perception/object_mask']
with rosbag.Bag(str(bagpath),'r') as bag:
    available=bag.get_type_and_topic_info().topics
    assert all(t in available for t in topics)
    # Explicit finite time window and allowlist. No whole-record image scan.
    for topic,msg,t in bag.read_messages(topics=topics,start_time=rospy.Time.from_sec(ns*1e-9-2.),end_time=rospy.Time.from_sec(ns*1e-9+2.)):
        counts[topic]=counts.get(topic,0)+1
        if topic=='/tf':
            for tr in msg.transforms:transformer.setTransform(tr,'sealed_record')
        elif topic=='/perception/unknown/detector_status':
            data=json.loads(msg.data)
            if data.get('source_stamp_ns')==ns and data.get('state')=='ready':
                assert status is None or status==data;status=data
                out=io.BytesIO();msg.serialize(out);status_wire=dict(bag_stamp_ns=t.to_nsec(),wire_sha256=sha(out.getvalue()),raw_string_sha256=sha(msg.data.encode()))
        elif msg.header.stamp.to_nsec()==ns:
            kind='depth_raw' if 'depth' in topic else 'raw_mask';assert kind not in arrays
            assert msg.encoding in ('16UC1','mono8')
            dt=np.dtype('>u2' if msg.is_bigendian else '<u2') if msg.encoding=='16UC1' else np.dtype('u1')
            raw=np.frombuffer(msg.data,dtype=dt).reshape(msg.height,msg.step//dt.itemsize)[:,:msg.width].copy()
            arrays[kind]=raw;out=io.BytesIO();msg.serialize(out)
            image_rows[kind]=dict(topic=topic,source_stamp_ns=ns,bag_stamp_ns=t.to_nsec(),array_sha256=arrsha(raw),wire_sha256=sha(out.getvalue()),shape=list(raw.shape),dtype=str(raw.dtype))
    if '/tf_static' in available:
        for _,msg,_ in bag.read_messages(topics=['/tf_static']): static.extend(msg.transforms)
    scene_connections=[dict(topic=k,type=v.msg_type,count=v.message_count) for k,v in available.items() if v.msg_type in ('moveit_msgs/PlanningScene','moveit_msgs/PlanningSceneWorld','moveit_msgs/RobotState')]
assert bagpath.stat()==stat and not blocked
assert set(arrays)=={'depth_raw','raw_mask'} and status is not None
stamp=rospy.Time(ns//10**9,ns%10**9)
for tr in static:
    tr.header.stamp=stamp;transformer.setTransform(tr,'sealed_static')
mount=rigid(*transformer.lookupTransform('tool0','camera_link',stamp))
t,q=transformer.lookupTransform('base_link','camera_link',stamp);basecam=quaternion_matrix([float(v) for v in q]);basecam[:3,3]=t
optical_basis=np.eye(4);optical_basis[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];optical=basecam@optical_basis
k=ca['camera_parameters'];g=a['replay_geometry'];normal=np.asarray(g['support_normal_base']);offset=g['support_offset_m'];center=np.asarray(g['obb_center_base_m']);axes=np.asarray(g['R_base_obb']);size=np.asarray(g['obb_size_xyz_m'])
raw,mask=arrays['depth_raw'],arrays['raw_mask'];ys,xs=np.nonzero((raw>0)&(mask==0));z=raw[ys,xs].astype(float)*k['depth_scale']
points=np.column_stack(((xs-k['cx'])*z/k['fx'],(ys-k['cy'])*z/k['fy'],z))@optical[:3,:3].T+optical[:3,3];points=points[np.abs(points@normal+offset)<=ca['support_distance_tolerance_m']]
corners=center+(np.asarray(list(itertools.product((-.5,.5),repeat=3)))*size)@axes.T
base_tool=(basecam@optical_basis)@optical_basis.T@np.linalg.inv(mount)
checks={key:dict(expected=ca[key],actual=arrsha(value),exact=ca[key]==arrsha(value)) for key,value in [('raw_depth_sha256',raw),('raw_mask_sha256',mask),('tool_camera_sha256',mount),('base_optical_sha256',optical),('support_points_sha256',points),('conservative_target_corners_sha256',corners),('frozen_tool_reference_sha256',base_tool)]}
meta_path=N/'actual_attempt_r1_handoff/camera_metadata_r1/camera_runtime_metadata.json';meta=json.loads(meta_path.read_text())
camera_row=max((r for r in meta['rows'] if r['bag_stamp_ns']<=ns),key=lambda r:r['bag_stamp_ns'])
camera=camera_row['data']['parameters'];assert all(camera[key]==v for key,v in k.items())
arrival=capture_arrival_projection(status,a['grasp_mode_selection'],ns,camera)
arrival_equal=arrival['audit']==ca['arrival_projection']
modelpath=W/'.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf';model=modelpath.read_text();fk=SerialUrdfFk(model,['Joint%d'%i for i in range(1,7)]);assert fk.model_sha256==search['model_sha256']
save('status.json',dict(data=status,wire=status_wire))
np.savez_compressed(O/'source.npz',**arrays,points_base=points,corners_base=corners,center_base=center,tool_from_camera=mount,base_from_camera=basecam,base_from_tool=base_tool,prism_base=arrival['prism_base'],rgb_valid=arrival['rgb_valid'],invalid_integral=arrival['invalid_integral'])
source=dict(binding=a['selected']['observation_source_binding'],audit=ca,camera=k,geometry=g,view_limits=ca['view_limits'])
save('source.json',dict(source=source,arrival_audit=arrival['audit'],current_joint_positions=search['current_joint_positions'],gaps=a['observe_return']['measured_missing_sides'],joint_error_bounds_rad=search['joint_error_bounds_rad'],model_path=str(modelpath),model_sha256=fk.model_sha256,opening_width_m=.05,opening_source='mode_aware_remote_grasp6d.search_inputs min(max_inner_gap, gripper_physical_open_width); both original config .05, distinct from actual .0498 measured finger start',original_search_audit=search,source_audit_path=str(ap),source_audit_sha256=ap.stem))
report=dict(exact_all_source_hashes=all(v['exact'] for v in checks.values()),checks=checks,arrival_audit_exact=arrival_equal,support_point_count=len(points),expected_support_point_count=ca['support_point_count'],source_stamp_ns=ns,images=image_rows,metadata_path=str(meta_path),metadata_sha256=sha(meta_path.read_bytes()),metadata_bag_stamp_ns=camera_row['bag_stamp_ns'],metadata_image_time_association='none',source_audit_path=str(ap),source_audit_sha256=ap.stem,read_counts=counts,read_window_seconds=[-2,2],static_transform_count=len(static),planning_scene_connections=scene_connections,bag_stat_unchanged=bagpath.stat()==stat,bag_sha256_from_closed_manifest=next(r['sha256'] for r in manifest['files'] if r['path']=='keyframes.bag'),full_bag_rehashed=False,model_sha256=fk.model_sha256,source_npz_sha256=sha((O/'source.npz').read_bytes()),script_sha256=sha(Path(__file__).read_bytes()),ROS_initialized=rospy.core.is_initialized(),socket_attempts=blocked,candidate_executed=False,strict_path_checked=False)
save('reconstruction.json',report);(O/'helper_source.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps(report,indent=2))
