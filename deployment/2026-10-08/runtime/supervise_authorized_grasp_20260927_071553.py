#!/usr/bin/env python3
"""Read-only subscriber supervision. No services, parameters, or publications."""
import json
import os
from pathlib import Path
import threading
import time

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD=BASE/'grasp_records/20260927T071553.378661Z_f557b777f3ee4ad48b4e05890fdaac34'
OUT=BASE/'ros_log/unknown_grasp_20260923/approach_failure_20260927/supervision_071553'
if not Path('/media/zhuyupei/SanStand').is_mount():raise SystemExit('SanStand unavailable')
OUT.mkdir(parents=True,exist_ok=True)
os.environ['ROS_LOG_DIR']=str(OUT/'ros_node_logs')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool,UInt8,String
from alicia_flexible_grasp_supervisor.msg import GraspState,SafetyState

log=(OUT/'watch.jsonl').open('a',buffering=1)
lock=threading.RLock()
last={}
received={}
seen_active=False
terminal=False

def emit(kind,value):
    with lock:
        line=json.dumps({'unix':time.time(),'kind':kind,'value':value},ensure_ascii=False)
        print(line,flush=True);log.write(line+'\n')

def changed(kind,value):
    with lock:
        if last.get(kind)!=value:
            last[kind]=value;emit(kind,value)

def scalar(msg,kind):
    received[kind]=time.monotonic()
    changed(kind,msg.data)

def safety(msg):
    received['safety']=time.monotonic()
    changed('safety',{'ok':msg.ok,'emergency_stop':msg.emergency_stop,
                      'level':msg.level,'message':msg.message})

def grasp(msg):
    global seen_active,terminal
    received['grasp']=time.monotonic()
    if msg.active:seen_active=True
    changed('grasp',{'state':msg.state,'stage':msg.stage,'active':msg.active,
                     'success':msg.success,'message':msg.message})
    if seen_active and not msg.active and msg.state in ('FAILED','DONE','SUCCEEDED','SUCCESS','CANCELLED','CANCELED'):
        terminal=True

def accepted(msg):
    received['accepted']=time.monotonic()
    last['accepted_stamp']=msg.header.stamp.to_sec()

rospy.init_node('grasp_authorized_readonly_watch',anonymous=True,disable_signals=True)
subs=[rospy.Subscriber('/safety/status',SafetyState,safety,queue_size=10),
      rospy.Subscriber('/grasp/state',GraspState,grasp,queue_size=10),
      rospy.Subscriber('/alicia_d/accepted_joint_states',JointState,accepted,queue_size=10)]
for topic,cls,key in [('/alicia_d/protection_latched',Bool,'protection_latched'),
                     ('/alicia_d/feedback_ready',Bool,'feedback_ready'),
                     ('/alicia_d/run_status',UInt8,'run_status'),
                     ('/alicia_d/actuation_status',String,'actuation_status')]:
    subs.append(rospy.Subscriber(topic,cls,lambda m,k=key:scalar(m,k),queue_size=10))
started=time.monotonic();offset=0;last_progress=started
emit('started',{'record':str(RECORD),'maximum_seconds':480,'read_only':True})
while not rospy.is_shutdown() and time.monotonic()-started<480:
    now=time.monotonic()
    if now-started>3:
        age=now-received.get('accepted',started)
        source_age=rospy.Time.now().to_sec()-last.get('accepted_stamp',0)
        changed('accepted_age_health',{'receipt_fresh':age<.5,'source_fresh':0<=source_age<.5})
        if age>.5 or source_age>.5:
            if now-last_progress>2:emit('accepted_stale',{'receipt_age_sec':age,'source_age_sec':source_age});last_progress=now
    p=RECORD/'runner.log'
    if p.exists():
        with p.open(errors='replace') as f:
            f.seek(offset)
            for line in f:
                emit('runner',line.rstrip())
                if 'GRASP_RESULT ' in line:terminal=True
            offset=f.tell()
    if terminal:emit('terminal_observed',last.get('grasp'));break
    time.sleep(.2)
else:emit('monitor_timeout',{'elapsed_sec':time.monotonic()-started})
for sub in subs:sub.unregister()
rospy.signal_shutdown('read-only watch finished')
log.close()
