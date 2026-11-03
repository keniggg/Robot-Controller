import os,time,json,threading,pathlib
from datetime import datetime,timezone
import rospy,tf2_ros,numpy as np,cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,JointState
from std_msgs.msg import String
from tf.transformations import quaternion_matrix
root=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_resume_20261004T002930Z')
if not pathlib.Path('/media/zhuyupei/SanStand').is_mount(): raise RuntimeError('SanStand unavailable')
out=root/('aligned_capture_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir()
rospy.init_node('aligned_capture_readonly',anonymous=True,disable_signals=True)
buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf);bridge=CvBridge();lock=threading.Lock();cache={};pairs=[];states={}
def img(m,kind):
 with lock:
  stamp=m.header.stamp.to_nsec();row=cache.setdefault(stamp,{});row[kind]=m
  if len(row)==2 and len(pairs)<3 and (not pairs or stamp-pairs[-1][0]>500000000):pairs.append((stamp,dict(row)))
  while len(cache)>25:del cache[min(cache)]
def state(m,key):
 if hasattr(m,'data'):
  try:states[key]=json.loads(m.data)
  except:states[key]=m.data
 else:states[key]={'stamp_ns':m.header.stamp.to_nsec(),'name':list(m.name),'position':list(m.position)}
subs=[rospy.Subscriber('/supervisor/camera/'+topic+'/image_raw',Image,lambda m,k=k:img(m,k),queue_size=2,buff_size=4*1024*1024) for topic,k in [('color','rgb'),('depth','depth')]]
subs += [rospy.Subscriber(topic,typ,lambda m,k=k:state(m,k),queue_size=2) for topic,typ,k in [('/perception/unknown/detector_status',String,'detector'),('/grasp_mode/status',String,'mode'),('/alicia_d/accepted_joint_states',JointState,'accepted')]]
deadline=time.monotonic()+8
while time.monotonic()<deadline and len(pairs)<3:time.sleep(.1)
for sub in subs:sub.unregister()
meta={'motion_commands_sent':False,'camera':rospy.get_param('/camera'),'unknown_params':rospy.get_param('/unknown_perception',{}),'selection':rospy.get_param('/grasp_mode/selection'),'states':states,'pairs':[]}
for i,(stamp,pair) in enumerate(pairs):
 rgb=bridge.imgmsg_to_cv2(pair['rgb'],'bgr8');depth=bridge.imgmsg_to_cv2(pair['depth'],'passthrough')
 cv2.imwrite(str(out/f'color_{i}.png'),rgb);np.save(out/f'depth_{i}.npy',depth)
 item={'index':i,'stamp_ns':stamp,'rgb_frame':pair['rgb'].header.frame_id,'depth_frame':pair['depth'].header.frame_id,'depth_encoding':pair['depth'].encoding}
 try:
  tr=buf.lookup_transform('base_link',pair['depth'].header.frame_id,pair['depth'].header.stamp,rospy.Duration(2)).transform
  T=quaternion_matrix([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]);T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z];item['T_base_depth_frame']=T.tolist()
 except Exception as e:item['tf_error']=str(e)
 meta['pairs'].append(item)
(out/'capture.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'directory':str(out),'pairs':len(pairs),'states':states},ensure_ascii=False,indent=2))
