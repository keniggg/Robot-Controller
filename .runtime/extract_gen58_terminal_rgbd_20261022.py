import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
from pathlib import Path
import sys,json,hashlib,socket,time,io
W=Path(__file__).resolve().parents[1]
B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R=B/'grasp_records/20261022T032906.414053Z_6a8a761e427d48ff9e4c14741837f7a3'
O=B/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/actual_attempt_r1_geometry/cuboid_exact_frame_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
sys.path[:0]=[str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
attempts=[]
class OfflineSocket(socket.socket):
 def __new__(cls,*a,**kw):
  attempts.append('socket');raise RuntimeError('offline socket forbidden')
socket.socket=OfflineSocket
import rosbag
import genpy
import numpy as np
stamp=1792639798898294925
manifest=json.loads((R/'manifest.json').read_text());assert manifest['state']=='complete' and manifest['recorder_closed'] is True
bagpath=R/'keyframes.bag';before=bagpath.stat()
sha=lambda b:hashlib.sha256(b).hexdigest()
selected={};metadata={};infos=[]
with rosbag.Bag(str(bagpath),'r') as bag:
 topicmeta=bag.get_type_and_topic_info().topics
 topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask']
 assert all(x in topicmeta for x in topics)
 for topic,msg,t in bag.read_messages(topics=topics,start_time=genpy.Time.from_sec(stamp*1e-9-1),end_time=genpy.Time.from_sec(stamp*1e-9+5)):
  if msg.header.stamp.to_nsec()!=stamp:continue
  assert topic not in selected
  kind='color' if 'color' in topic else 'depth' if 'depth' in topic else 'mask'
  assert msg.encoding in ('rgb8','bgr8','mono8','16UC1','mono16')
  dt=np.dtype('>u2' if msg.is_bigendian else '<u2') if msg.encoding in ('16UC1','mono16') else np.dtype('u1')
  ar=np.frombuffer(msg.data,dtype=dt).reshape(msg.height,msg.step//dt.itemsize)
  ar=ar[:,:msg.width*3].reshape(msg.height,msg.width,3) if kind=='color' else ar[:,:msg.width]
  if msg.encoding=='rgb8':ar=ar[:,:,::-1]
  ar=np.ascontiguousarray(ar);selected[topic]=ar
  out=io.BytesIO();msg.serialize(out)
  metadata[kind]=dict(topic=topic,source_stamp_ns=stamp,bag_stamp_ns=t.to_nsec(),frame_id=msg.header.frame_id,encoding=msg.encoding,step=msg.step,dtype=str(ar.dtype),shape=list(ar.shape),array_sha256=sha(ar.tobytes()),wire_sha256=sha(out.getvalue()))
 info_topics=[k for k,v in topicmeta.items() if v.msg_type=='sensor_msgs/CameraInfo']
 for topic,msg,t in (bag.read_messages(topics=info_topics) if info_topics else ()):
  if len(infos)>20:raise RuntimeError('camera info unexpectedly unbounded')
  infos.append(dict(topic=topic,stamp_ns=msg.header.stamp.to_nsec(),bag_stamp_ns=t.to_nsec(),height=msg.height,width=msg.width,K=list(msg.K),D=list(msg.D),P=list(msg.P),R=list(msg.R),distortion_model=msg.distortion_model,frame_id=msg.header.frame_id))
assert len(selected)==3 and not attempts
assert bagpath.stat()==before
O.mkdir(parents=True,exist_ok=False)
np.savez_compressed(O/'exact_rgbd_mask.npz',**{k: selected[v['topic']] for k,v in metadata.items()})
result=dict(source_stamp_ns=stamp,images=metadata,camera_info=infos,bag_identity=dict(path=str(bagpath),size=before.st_size,mtime_ns=before.st_mtime_ns,inode=before.st_ino,manifest_sha256=sha((R/'manifest.json').read_bytes()),manifest_bag_sha256=next(x['sha256'] for x in manifest['files'] if x['path']=='keyframes.bag'),full_bag_rehashed=False,unchanged_stat=True),archive_sha256=sha((O/'exact_rgbd_mask.npz').read_bytes()),script_sha256=sha(Path(__file__).read_bytes()),socket_attempts=attempts,ROS_initialized=False,production_modified=False)
(O/'extraction.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
