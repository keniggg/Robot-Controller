"""Read only original TF topic window, no ROS node/listener/service."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
from pathlib import Path
import sys,json,hashlib,socket,io
W=Path(__file__).resolve().parents[1]; B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=B/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1'; O=N/'observation_pose_fix/offline_selection_r1/source_attempt2';P=O.parent/'source_attempt1'
assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir(exist_ok=False);(O/'genpy_tmp').mkdir();os.environ['TMPDIR']=str(O/'genpy_tmp')
sys.path[:0]=[str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
blocked=[]
class OfflineSocket(socket.socket):
 def __new__(cls,*a,**kw):blocked.append('socket');raise RuntimeError('offline sockets prohibited')
socket.socket=OfflineSocket
import numpy as np
import rosbag,rospy,tf2_py
from tf.transformations import quaternion_matrix
sha=lambda x:hashlib.sha256(x).hexdigest()
def save(name,d):(O/name).write_text(json.dumps(d,indent=2,allow_nan=False)+'\n')
def matrix(tr):
 t,q=tr.transform.translation,tr.transform.rotation
 m=quaternion_matrix([float(q.x),float(q.y),float(q.z),float(q.w)]);m[:3,3]=[t.x,t.y,t.z];return m
ns=1792639754162577867;stamp=rospy.Time(ns//10**9,ns%10**9);buf=tf2_py.BufferCore(rospy.Duration(200))
bagpath=B/'grasp_records/20261022T032906.414053Z_6a8a761e427d48ff9e4c14741837f7a3/keyframes.bag';before=bagpath.stat();records=[]
with rosbag.Bag(str(bagpath),'r') as bag:
 for topic,msg,t in bag.read_messages(topics=['/tf'],start_time=rospy.Time.from_sec(ns*1e-9-2),end_time=rospy.Time.from_sec(ns*1e-9+2)):
  for tr in msg.transforms:
   buf.set_transform(tr,'sealed_record');v,q=tr.transform.translation,tr.transform.rotation
   out=io.BytesIO();tr.serialize(out)
   records.append(dict(topic=topic,bag_stamp_ns=t.to_nsec(),source_stamp_ns=tr.header.stamp.to_nsec(),parent=tr.header.frame_id,child=tr.child_frame_id,translation=[v.x,v.y,v.z],rotation=[q.x,q.y,q.z,q.w],wire_sha256=sha(out.getvalue())))
 for topic,msg,t in bag.read_messages(topics=['/tf_static']):
  for tr in msg.transforms:
   buf.set_transform_static(tr,'sealed_record');v,q=tr.transform.translation,tr.transform.rotation
   out=io.BytesIO();tr.serialize(out)
   records.append(dict(topic=topic,bag_stamp_ns=t.to_nsec(),source_stamp_ns=tr.header.stamp.to_nsec(),parent=tr.header.frame_id,child=tr.child_frame_id,translation=[v.x,v.y,v.z],rotation=[q.x,q.y,q.z,q.w],wire_sha256=sha(out.getvalue())))
assert bagpath.stat()==before and not blocked
save('tf_records.json',dict(rows=records,topic_allowlist=['/tf','/tf_static'],dynamic_window_seconds=[-2,2]))
meta=json.loads((P/'source.json').read_text());a=meta['source']['audit'];k=meta['source']['camera'];g=meta['source']['geometry']
with np.load(P/'source.npz') as z:arrays={key:z[key].copy() for key in z.files}
basecam=matrix(buf.lookup_transform_core('base_link','camera_link',stamp));mount=arrays['tool_from_camera'];C=np.eye(4);C[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];optical=basecam.dot(C)
raw,mask=arrays['depth_raw'],arrays['raw_mask'];ys,xs=np.nonzero((raw>0)&(mask==0));z=raw[ys,xs].astype(float)*k['depth_scale'];points=np.column_stack(((xs-k['cx'])*z/k['fx'],(ys-k['cy'])*z/k['fy'],z))@optical[:3,:3].T+optical[:3,3];points=points[np.abs(points@np.asarray(g['support_normal_base'])+g['support_offset_m'])<=.002]
ref=optical@C.T@np.linalg.inv(mount)
checks={key:dict(expected=a[key],actual=sha(v.astype('<f8').tobytes()),exact=a[key]==sha(v.astype('<f8').tobytes())) for key,v in [('base_optical_sha256',optical),('support_points_sha256',points),('frozen_tool_reference_sha256',ref)]}
arrays.update(base_from_camera=basecam,base_from_tool=ref,points_base=points);np.savez_compressed(O/'source.npz',**arrays);(O/'source.json').write_bytes((P/'source.json').read_bytes())
report=dict(checks=checks,source_parent_npz_sha256=sha((P/'source.npz').read_bytes()),source_npz_sha256=sha((O/'source.npz').read_bytes()),tf_records_sha256=sha((O/'tf_records.json').read_bytes()),support_point_count=len(points),tf2_base_from_camera=basecam.tolist(),max_tf1_tf2_abs_delta=float(np.max(np.abs(basecam-np.load(P/'source.npz')['base_from_camera']))),ROS_initialized=rospy.core.is_initialized(),socket_attempts=blocked,bag_stat_unchanged=bagpath.stat()==before,helper_sha256=sha(Path(__file__).read_bytes()))
save('reconstruction.json',report);(O/'helper_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(report,indent=2))
