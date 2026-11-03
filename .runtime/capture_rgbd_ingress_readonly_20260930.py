#!/usr/bin/env python3
import json,time,threading,statistics
from collections import Counter
from pathlib import Path
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Log
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z')
assert OUT.is_dir()
rospy.init_node('rgbd_ingress_capture_readonly',anonymous=True,disable_signals=True)
lock=threading.Lock()
rows=[]
feedback={'accepted':[],'sdk':[]}
logs=[]
def status(msg):
    at=time.monotonic()
    try: value=json.loads(msg.data)
    except ValueError: return
    with lock:
        if len(rows)<2000: rows.append({'receipt_monotonic_sec':at,'receipt_ros_ns':rospy.Time.now().to_nsec(),'status':value})
def joint(msg,key):
    at=time.monotonic()
    with lock:
        if len(feedback[key])<4000:feedback[key].append([msg.header.stamp.to_nsec(),at])
def log(msg):
    if msg.level<Log.WARN or not any(x in msg.name for x in ('camera','alicia_d_driver','unknown_tabletop')):return
    with lock:
        if len(logs)<500:logs.append({'stamp_ns':msg.header.stamp.to_nsec(),'name':msg.name,'text':msg.msg})
subs=[rospy.Subscriber('/perception/unknown/detector_status',String,status,queue_size=10),rospy.Subscriber('/alicia_d/accepted_joint_states',JointState,lambda m:joint(m,'accepted'),queue_size=50),rospy.Subscriber('/alicia_d/sdk_command',JointState,lambda m:joint(m,'sdk'),queue_size=50),rospy.Subscriber('/rosout_agg',Log,log,queue_size=100)]
start_ns=rospy.Time.now().to_nsec(); started=time.monotonic()
print('READONLY_INGRESS_CAPTURE_STARTED',start_ns,flush=True)
while not rospy.is_shutdown() and time.monotonic()-started<90.: time.sleep(.1)
for sub in subs:sub.unregister()
with lock:
    document={'start_ns':start_ns,'end_ns':rospy.Time.now().to_nsec(),'duration_sec':time.monotonic()-started,'motion_commands_sent':False,'rows':list(rows),'feedback':feedback,'warnings':list(logs)}
unique={}
for r in document['rows']:
    s=r['status'];t=s.get('frame_timing')
    if t:unique[t['source_stamp_ns']]=t
metrics={}
paths=[('color_arrival_age',('raw_arrivals','color','source_age_at_arrival_sec')),('depth_arrival_age',('raw_arrivals','depth','source_age_at_arrival_sec')),('raw_arrival_difference',('raw_arrival_difference_sec',)),('after_latest_raw_arrival',('after_latest_raw_arrival_sec',)),('input_lock_wait',('input_lock_wait_sec',)),('exact_pair_to_input_slot',('exact_pair_to_input_slot_sec',)),('ingress_to_processing',('ingress_to_processing_sec',))]
for key,path in paths:
    values=[]
    for timing in unique.values():
        value=timing
        for p in path:value=value.get(p) if isinstance(value,dict) else None
        if isinstance(value,(int,float)):values.append(value)
    values.sort()
    if values:metrics[key]={'n':len(values),'median_ms':statistics.median(values)*1000,'p95_ms':values[min(len(values)-1,int(.95*(len(values)-1)))]*1000,'maximum_ms':max(values)*1000}
summary={'unique_timed_source_frames':len(unique),'status_messages':len(rows),'metrics':metrics,'reasons':dict(Counter(r['status'].get('reason',r['status'].get('state','unknown')) for r in rows)),'warnings':len(logs)}
for key,data in feedback.items():
    summary[key]={'messages':len(data),'max_source_gap_ms':max([(b[0]-a[0])/1e6 for a,b in zip(data,data[1:])],default=None),'max_receipt_gap_ms':max([(b[1]-a[1])*1000 for a,b in zip(data,data[1:])],default=None)}
document['summary']=summary
p=OUT/'rgbd_ingress_live_readonly_20260930.json'
p.write_text(json.dumps(document,ensure_ascii=False,indent=2)+'\n')
print('READONLY_INGRESS_CAPTURE_COMPLETE',json.dumps(summary,ensure_ascii=False),flush=True)
