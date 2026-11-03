"""Replay retained TF values under the original no-user-site numeric environment."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
from pathlib import Path
import sys,json,hashlib,socket
W=Path(__file__).resolve().parents[1];N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1');O=N/'observation_pose_fix/offline_selection_r1/source_attempt3';P=O.parent/'source_attempt2'
assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir(exist_ok=False)
sys.path[:0]=['/opt/ros/noetic/lib/python3/dist-packages'];blocked=[]
class NoSocket(socket.socket):
 def __new__(cls,*a,**k):blocked.append('socket');raise RuntimeError('offline socket denied')
socket.socket=NoSocket
import numpy as np
import rospy,tf2_py
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();ash=lambda v:hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest()
ns=1792639754162577867;buf=tf2_py.BufferCore(rospy.Duration(200));tfrows=json.loads((P/'tf_records.json').read_text())['rows']
for r in tfrows:
 tr=TransformStamped();tr.header.frame_id=r['parent'];tr.child_frame_id=r['child'];tr.header.stamp=rospy.Time(r['source_stamp_ns']//10**9,r['source_stamp_ns']%10**9);tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=r['translation'];tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=r['rotation']
 if r['topic']=='/tf_static':buf.set_transform_static(tr,'sealed_record')
 else:buf.set_transform(tr,'sealed_record')
t=buf.lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9));v,q=t.transform.translation,t.transform.rotation;camera=quaternion_matrix([float(q.x),float(q.y),float(q.z),float(q.w)]);camera[:3,3]=[v.x,v.y,v.z];C=np.eye(4);C[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];optical=camera.dot(C)
m=json.loads((P/'source.json').read_text());a=m['source']['audit'];g=m['source']['geometry'];k=m['source']['camera']
with np.load(P/'source.npz') as z:arrays={key:z[key].copy() for key in z.files}
raw,mask=arrays['depth_raw'],arrays['raw_mask'];ys,xs=np.nonzero((raw>0)&(mask==0));depth=raw[ys,xs].astype(float)*k['depth_scale'];points=np.column_stack(((xs-k['cx'])*depth/k['fx'],(ys-k['cy'])*depth/k['fy'],depth))@optical[:3,:3].T+optical[:3,3];points=points[np.abs(points@np.asarray(g['support_normal_base'])+g['support_offset_m'])<=.002];ref=optical@C.T@np.linalg.inv(arrays['tool_from_camera'])
checks={key:dict(expected=a[key],actual=ash(v),exact=a[key]==ash(v)) for key,v in [('base_optical_sha256',optical),('support_points_sha256',points),('frozen_tool_reference_sha256',ref)]}
arrays.update(base_from_camera=camera,base_from_tool=ref,points_base=points);np.savez_compressed(O/'source.npz',**arrays);(O/'source.json').write_bytes((P/'source.json').read_bytes())
r=dict(checks=checks,numpy_version=np.__version__,numpy_path=np.__file__,no_user_site=sys.flags.no_user_site,tf_records_sha256=sha(P/'tf_records.json'),source_parent_npz_sha256=sha(P/'source.npz'),source_npz_sha256=sha(O/'source.npz'),support_point_count=len(points),ROS_initialized=rospy.core.is_initialized(),socket_attempts=blocked,new_bag_reads=0,helper_sha256=sha(Path(__file__)))
(O/'reconstruction.json').write_text(json.dumps(r,indent=2)+'\n');(O/'helper_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(r,indent=2))
