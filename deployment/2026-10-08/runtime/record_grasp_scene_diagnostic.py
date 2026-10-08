#!/usr/bin/env python3
"""Read-only exact RGB-D record at <=5 Hz, independent of target visibility."""
import os,json,time,signal,threading,shutil
from pathlib import Path
from collections import OrderedDict
p=Path(json.load(open('.runtime/current_grasp_diagnostic.json'))['directory']);assert os.path.ismount('/media/zhuyupei/SanStand');assert shutil.disk_usage(p).free>8*1024**3
os.environ['ROS_LOG_DIR']=str(p/'ros')
import rospy,rosbag
from sensor_msgs.msg import Image,JointState
from std_msgs.msg import String,Header,Bool
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Log
from diagnostic_msgs.msg import DiagnosticArray
from alicia_flexible_grasp_supervisor.msg import GraspState,ObjectPose
rospy.init_node('grasp_scene_diagnostic_readonly',anonymous=True,disable_signals=True)
interval_sec=float(os.environ.get('SCENE_RGBD_RECORD_INTERVAL_SEC','0.2'))
if not .2 <= interval_sec <= 2.:raise ValueError('invalid diagnostic image interval')
interval_ns=int(interval_sec*1e9)
lock=threading.RLock();quit=threading.Event();rgb=OrderedDict();depth=OrderedDict();bag=rosbag.Bag(str(p/'scene_diagnostic.bag'),'w',compression='lz4');state=dict(pairs=0,last_ns=0,active_seen=False,terminal_at=None,closed=False)
RGB='/supervisor/camera/color/image_raw';DEPTH='/supervisor/camera/depth/image_raw'
def receive(topic,m):
 with lock:
  if state['closed']:return
  if topic in (RGB,DEPTH):
   ns=m.header.stamp.to_nsec();cache=rgb if topic==RGB else depth;cache[ns]=m
   while len(cache)>30:cache.popitem(last=False)
   if ns in rgb and ns in depth and ns-state['last_ns']>=interval_ns:
    bag.write(RGB,rgb[ns],rgb[ns].header.stamp);bag.write(DEPTH,depth[ns],depth[ns].header.stamp);state['pairs']+=1;state['last_ns']=ns
  else:
   bag.write(topic,m,rospy.Time.now())
   if topic=='/grasp/state':
    if m.active:state['active_seen']=True
    elif state['active_seen'] and state['terminal_at'] is None:state['terminal_at']=time.monotonic()
subs=[]
for t,k in [(RGB,Image),(DEPTH,Image),('/tf',TFMessage),('/tf_static',TFMessage),('/alicia_d/accepted_joint_states',JointState),('/alicia_d/control_reference',JointState),('/alicia_d/control_reference_epoch',Header),('/alicia_d/actuation_status',String),('/grasp/state',GraspState),('/perception/unknown/detector_status',String),('/perception/object',ObjectPose),('/grasp_6d/status',String),('/rosout_agg',Log)]:
 subs.append(rospy.Subscriber(t,k,lambda m,t=t:receive(t,m),queue_size=50 if k!=Image else 3,buff_size=2**24))
for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:quit.set())
(p/'scene_recorder_ready.json').write_text(json.dumps(dict(pid=os.getpid(),ready=True,started=time.time(),read_only=True,sample_interval_ns=interval_ns))+'\n')
start=time.monotonic();reason='maximum_duration';last=0
try:
 while not rospy.is_shutdown() and not quit.is_set() and time.monotonic()-start<900:
  with lock:
   if state['terminal_at'] is not None and time.monotonic()-state['terminal_at']>=8:reason='task_terminal_plus_8sec';break
   if time.monotonic()-last>=2:
    (p/'scene_recorder_progress.json').write_text(json.dumps(dict(time=time.time(),pairs=state['pairs'],last_source_stamp_ns=state['last_ns'],active_seen=state['active_seen']))+'\n');last=time.monotonic()
  time.sleep(.1)
finally:
 for sub in subs:sub.unregister()
 with lock:state['closed']=True;bag.close()
 (p/'scene_recorder_result.json').write_text(json.dumps(dict(pairs=state['pairs'],closed=True,reason='signal_readonly_recorder' if quit.is_set() else reason,ended=time.time()))+'\n')
