"""Twenty-second passive RGB-D continuity capture, without camera ownership."""
import json, os, time, threading
from pathlib import Path
from datetime import datetime, timezone
import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

m=Path('/media/zhuyupei/SanStand')
assert m.is_mount(), 'SanStand unavailable'
out=m/'实验记录/Robot-Controller/ros_log'/('rgbd_continuity_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
out.mkdir(); os.environ['ROS_LOG_DIR']=str(out/'ros')
rospy.init_node('observe_rgbd_passive',anonymous=True,disable_signals=True)
lock=threading.Lock(); streams={'rgb':[], 'depth':[]}; latest={}; states=[]
def frame(msg,key):
    with lock:
        streams[key].append((msg.header.stamp.to_nsec(),time.monotonic(),(rospy.Time.now()-msg.header.stamp).to_sec()))
        latest[key]=msg
def state(msg):
    try: value=json.loads(msg.data)
    except ValueError:return
    with lock: states.append(value)
subs=[rospy.Subscriber('/supervisor/camera/'+topic+'/image_raw',Image,lambda x,k=key:frame(x,k),queue_size=2,buff_size=4*1024*1024) for topic,key in [('color','rgb'),('depth','depth')]]
subs.append(rospy.Subscriber('/perception/unknown/detector_status',String,state,queue_size=3))
start=time.monotonic()
while time.monotonic()-start<20 and not rospy.is_shutdown():time.sleep(.05)
for sub in subs:sub.unregister()
with lock: data={k:list(v) for k,v in streams.items()}; images=dict(latest); statuses=list(states)
summary={}
for key,rows in data.items():
    summary[key]=dict(count=len(rows),max_source_gap_sec=max([(b[0]-a[0])/1e9 for a,b in zip(rows,rows[1:])],default=None),max_receipt_gap_sec=max([b[1]-a[1] for a,b in zip(rows,rows[1:])],default=None),max_age_sec=max([r[2] for r in rows],default=None))
common=set(r[0] for r in data['rgb'])&set(r[0] for r in data['depth'])
summary['exact_common_stamps']=len(common)
bridge=CvBridge()
if 'rgb' in images: cv2.imwrite(str(out/'current_rgb.png'),bridge.imgmsg_to_cv2(images['rgb'],'bgr8'))
if 'depth' in images: np.save(out/'current_depth.npy',bridge.imgmsg_to_cv2(images['depth'],'passthrough'))
report=dict(summary=summary,streams=data,detector_statuses=statuses,camera=rospy.get_param('/camera'),selection=rospy.get_param('/grasp_mode/selection'),motion_commands_sent=False,configuration_changed=False,image_stamps={k:v.header.stamp.to_nsec() for k,v in images.items()})
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(directory=str(out),summary=summary,last_status=statuses[-1] if statuses else None),ensure_ascii=False),flush=True)
