#!/usr/bin/env python3
"""Read-only exact production mask/RGB-D/board recorder; no publishers/services."""
import argparse,io,json,os,sys,threading,time,hashlib
from collections import OrderedDict
from datetime import datetime,timezone
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from bounded_async_bag_writer import BoundedAsyncBagWriter
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--board-optional',action='store_true',help='Explicit changed scene: no current board is claimed');args=parser.parse_args()
MOUNT=Path('/media/zhuyupei/SanStand');ARCHIVE=MOUNT/'实验记录/Robot-Controller/ros_log'
if not os.path.ismount(MOUNT):raise RuntimeError('SanStand absent')
out=ARCHIVE/('production_lock_readonly_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()));out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
import rospy,rosbag
from sensor_msgs.msg import Image,JointState
from std_msgs.msg import String,Header,Bool
from tf2_msgs.msg import TFMessage
from diagnostic_msgs.msg import DiagnosticArray
from alicia_flexible_grasp_supervisor.msg import ObjectPose,ObjectGeometry,Grasp6DPlan
rospy.init_node('production_lock_readonly',anonymous=True,disable_signals=True)
bag=rosbag.Bag(str(out/'evidence.bag'),'w');fault=[];lock=threading.RLock();started=rospy.Time.now().to_nsec();selected=[];fallback=[];cache={k:OrderedDict() for k in ('rgb','depth','quality','board','mask','status','object')};closed=False

def fail(reason):
 with lock:fault.append(str(reason))
def write(m,p,t):
 ns=m['stamp_ns'];bag.write(m['topic'],(m['datatype'],p,m['md5sum'],t),rospy.Time(ns//10**9,ns%10**9),raw=True)
writer=BoundedAsyncBagWriter(write,fail)
def save(topic,msg,receipt):
 b=io.BytesIO();msg.serialize(b);writer.submit(topic,b.getvalue(),receipt,msg._type,msg._md5sum,msg.__class__)
def put(key,stamp,value):
 cache[key][stamp]=value
 while len(cache[key])>40:cache[key].popitem(last=False)
def callback(topic,msg):
 global closed
 received=rospy.Time.now().to_nsec();pending=[]
 try:
  with lock:
   if closed:return
   kind=None;source=None;data=None
   if topic.endswith('/color/image_raw'):kind='rgb';source=msg.header.stamp.to_nsec()
   elif topic.endswith('/depth/image_raw'):kind='depth';source=msg.header.stamp.to_nsec()
   elif topic=='/charuco/quality':kind='quality';data=json.loads(msg.data);source=data.get('source_stamp_ns')
   elif topic=='/perception/unknown/object_mask':kind='mask';source=msg.header.stamp.to_nsec()
   elif topic=='/perception/unknown/detector_status':kind='status';data=json.loads(msg.data);source=data.get('source_stamp_ns')
   elif topic=='/perception/unknown/object':kind='object';source=msg.header.stamp.to_nsec()
   elif topic=='/tf':
    for t in msg.transforms:
     if t.header.frame_id=='camera_link':put('board',t.header.stamp.to_nsec(),(t,received))
   if kind is not None and type(source) is int:put(kind,source,(msg,received,data))
   if len(selected)<3:
    for ns,(status,status_received,s) in list(cache['status'].items()):
     if ns<=started+1_000_000_000 or any(x['source_stamp_ns']==ns for x in selected):continue
     if selected and ns-selected[-1]['source_stamp_ns']<200_000_000:continue
     required=('rgb','depth','mask','status','object') if args.board_optional else tuple(cache)
     if not all(ns in cache[k] for k in required):continue
     q=cache['quality'][ns][2] if ns in cache['quality'] else {}
     if s.get('target_locked') is not True or s.get('state')!='ready' or s.get('exact_rgbd') is not True:continue
     if not args.board_optional and (q.get('pose_ok') is not True or q.get('charuco_count',0)<35 or (q.get('edge_clearance_px') or 0)<20):continue
     if not all(0<=received-ns<=350_000_000 and 0<=cache[k][ns][1]-ns<=350_000_000 for k in required):continue
     if any(s.get(k)!=ns for k in ('color_stamp_ns','depth_stamp_ns','tf_lookup_stamp_ns')):continue
     m=cache['mask'][ns][0]
     if not any(m.data):continue
     row=dict(source_stamp_ns=ns,selected_receipt_ns=received,receipts_ns={k:cache[k][ns][1] for k in required},board_required=not args.board_optional,current_board_observed=ns in cache['board'],
       generation=s['generation'],selection_stamp_ns=s['selection_stamp_ns'],anchor_source_stamp_ns=s.get('anchor_source_stamp_ns'),
       camera_model_fingerprint=q.get('camera_model_fingerprint'),production_support_plane_base=s['geometry_metrics'].get('support_plane_base'))
     selected.append(row)
     for k in ('rgb','depth','mask','object','quality','status'):
      if ns not in cache[k]:continue
      pending.append(('/production_lock/selected_'+k,cache[k][ns][0],cache[k][ns][1]))
     if ns in cache['board']:pending.append(('/production_lock/selected_board',TFMessage(transforms=[cache['board'][ns][0]]),cache['board'][ns][1]))
     if len(selected)>=3:break
   common=set(cache['rgb'])&set(cache['depth'])
   if common and len(fallback)<5:
    ns=max(common)
    if not fallback or ns-fallback[-1]>=2_000_000_000:
     fallback.append(ns)
     for k in ('rgb','depth'):pending.append(('/production_lock/unfiltered_diagnostic_'+k,cache[k][ns][0],cache[k][ns][1]))
  if kind not in ('rgb','depth'):save(topic,msg,received)
  for t,m,r in pending:save(t,m,r)
 except BaseException as error:
  writer.latch_failure('readonly evidence callback: '+str(error))
subs=[]
for topic,kind in [('/supervisor/camera/color/image_raw',Image),('/supervisor/camera/depth/image_raw',Image),('/charuco/quality',String),('/tf',TFMessage),('/tf_static',TFMessage),('/perception/unknown/object_mask',Image),('/perception/unknown/object',ObjectPose),('/perception/unknown/detector_status',String),('/grasp_mode/selection',String),('/alicia_d/accepted_joint_states',JointState),('/alicia_d/control_reference',JointState),('/alicia_d/control_reference_epoch',Header),('/alicia_d/actuation_status',String),('/alicia_d/sdk_diagnostics',DiagnosticArray),('/grasp_6d/object_geometry',ObjectGeometry),('/grasp_6d/plan_enriched',Grasp6DPlan),('/grasp_6d/pipeline_metrics',String),('/grasp_6d/gate_audit',String),('/grasp_6d/status',String)]:
 subs.append(rospy.Subscriber(topic,kind,lambda msg,topic=topic:callback(topic,msg),queue_size=32,buff_size=2**24))
(out/'robot_description.urdf').write_text(rospy.get_param('/robot_description'))
(out/'camera_parameters.json').write_text(json.dumps(rospy.get_param('/camera'),indent=2)+'\n')
(out/'collector.py').write_bytes(Path(__file__).read_bytes())
try:
 deadline=time.monotonic()+10.
 while time.monotonic()<deadline and not rospy.is_shutdown():
  writer.require_healthy();time.sleep(.05)
finally:
 with lock:closed=True
 for sub in subs:sub.unregister()
 summary=writer.close();bag.close()
 report=dict(diagnostic_only=True,motion_authorized=False,started_ns=started,board_required=not args.board_optional,selected=selected,unfiltered_diagnostic_sources=fallback,writer=summary,faults=fault)
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(dict(session=str(out),selected_sources=[r['source_stamp_ns'] for r in selected],writer=summary),indent=2),flush=True)
if len(selected)<3:raise RuntimeError('three new exact production/board tuples were not acquired')
