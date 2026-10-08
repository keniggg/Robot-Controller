#!/usr/bin/env python3
"""Recorded bounded single-axis experiment; no changes to calibration/thresholds.

Requires a freshly prepared plan module. Keeps every image, raw diagnostic,
accepted feedback and SDK command on mounted SanStand. No disable command.
"""
import argparse, datetime, hashlib, importlib.util, io, json, math, signal, sys, threading, time
from pathlib import Path
import cv2, numpy as np, rospy, rosbag, tf2_ros, xmlrpc.client
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String, Header
from tf2_msgs.msg import TFMessage
from diagnostic_msgs.msg import DiagnosticArray
from alicia_flexible_grasp_supervisor.msg import GraspState
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'.runtime'))
from handeye_auto_motion import AutoMotion
from bounded_async_bag_writer import BoundedAsyncBagWriter
MOUNT=Path('/media/zhuyupei/SanStand')
OUTROOT=MOUNT/'实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause'
C=np.eye(4);C[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]

class Recorder:
 def __init__(self,out):
  self.out=out; self.lock=threading.RLock();self.active=True;self.frames={};self.counts={};self.states=[];self.fatal=None;self.armed=False;self.last_joint_ns=0;self.epoch_ns=None;self.allow_epoch_until=0;self.subs=[];self.bag=None;self.latest_task_active=False;self.archive_writer=None
  self.bridge=CvBridge();self.buffer=tf2_ros.Buffer(rospy.Duration(45));self.listener=tf2_ros.TransformListener(self.buffer)
  self.camera=rospy.get_param('/camera');self.xml=rospy.get_param('/robot_description')
  runtime={'driver_parameters':rospy.get_param('/alicia_d_driver_node'),'camera_parameters':self.camera,'nodes':{},'captured_ros_ns':rospy.Time.now().to_nsec()}
  for node in ['/alicia_d_driver_node','/supervisor_camera']:
   code,detail,uri=rospy.get_master().lookupNode(node)
   if code!=1:raise RuntimeError('runtime evidence node unavailable: '+node)
   code,detail,pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
   if code!=1:raise RuntimeError('runtime evidence PID unavailable: '+node)
   proc=Path('/proc')/str(pid)
   runtime['nodes'][node]={'pid':pid,'command':(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(),'executable_sha256':hashlib.sha256((proc/'exe').read_bytes()).hexdigest()}
  (out/'runtime_configuration.json').write_text(json.dumps(runtime,indent=2)+'\n')
  (out/'camera_parameters.json').write_text(json.dumps(self.camera,indent=2)+'\n');(out/'robot_description.urdf').write_text(self.xml)
  self.bag=rosbag.Bag(str(out/'samples.bag'),'w',compression='lz4')
  self.start_archive_writer()
  specs=[('/supervisor/camera/color/image_raw',Image),('/supervisor/camera/depth/image_raw',Image),
   ('/alicia_d/accepted_joint_states',JointState),('/alicia_d/sdk_command',JointState),('/alicia_d/sdk_diagnostics',DiagnosticArray),
   ('/alicia_d/actuation_status',String),('/alicia_d/control_reference',JointState),('/alicia_d/control_reference_epoch',Header),
   ('/tf',TFMessage),('/tf_static',TFMessage),('/alicia_d/motion_enabled',Bool),('/alicia_d/feedback_ready',Bool),
   ('/gui/joint_direct_mode',Bool),('/joint_commands',JointState),('/grasp/state',GraspState),('/perception/unknown/detector_status',String)]
  self.subs=[rospy.Subscriber(t,k,self.callback(t),queue_size=16,buff_size=2**24 if t.endswith('image_raw') else 65536) for t,k in specs]
  self.K=np.array([[self.camera['fx'],0,self.camera['cx']],[0,self.camera['fy'],self.camera['cy']],[0,0,1.]])
  (out/'camera_parameters.json').write_text(json.dumps(self.camera,indent=2)+'\n')
  (out/'robot_description.urdf').write_text(rospy.get_param('/robot_description'))
  self.mark('recording_started',out=str(out))
 def start_archive_writer(self):
  def write_raw(metadata,payload,pytype):
   ns=metadata['stamp_ns'];stamp=rospy.Time(*divmod(ns,10**9))
   self.bag.write(metadata['topic'],(metadata['datatype'],payload,metadata['md5sum'],pytype),stamp,raw=True)
  def archive_failure(reason):
   with self.lock:self.fatal='diagnostic archive: '+str(reason)
  # Disk compression/flush must not hold the feedback/image state lock. Keep
  # immutable serialized messages within a fixed memory budget; overflow is an
  # explicit recording failure and prevents another movement command.
  self.archive_writer=BoundedAsyncBagWriter(write_raw,archive_failure,maximum_bytes=32*1024*1024,maximum_items=512)
 def callback(self,topic):
  def cb(msg):
   try:
    with self.lock:self.receive(topic,msg)
   except Exception as exc:
    with self.lock:self.fatal='recording callback failed: '+str(exc)
  return cb
 def receive(self,topic,msg):
    if not self.active:return
    self.archive_message(topic,msg,rospy.Time.now());self.counts[topic]=self.counts.get(topic,0)+1
    if topic.endswith('image_raw'):
     ns=msg.header.stamp.to_nsec();k='color' if '/color/' in topic else 'depth';self.frames.setdefault(ns,{})[k]=msg
     for old in sorted(self.frames)[:-32]:del self.frames[old]
    if topic=='/grasp/state':
     self.latest_task_active=bool(msg.active)
     if msg.active:self.fatal='another grasp task activated'
    if topic=='/joint_commands' and self.armed and getattr(msg,'_connection_header',{}).get('callerid','')!=rospy.get_name():self.fatal='external joint command during diagnostic'
    if topic=='/alicia_d/control_reference_epoch':
     ns=msg.stamp.to_nsec()
     if self.armed and self.epoch_ns is not None and ns!=self.epoch_ns and time.monotonic()>self.allow_epoch_until:self.fatal='unexpected control reference epoch change'
     self.epoch_ns=ns
    if topic=='/alicia_d/accepted_joint_states':
     names=['Joint%d'%i for i in range(1,7)]+['right_finger'];values=dict(zip(msg.name,msg.position));ns=msg.header.stamp.to_nsec()
     if len(values)!=len(msg.name) or any(n not in values for n in names) or msg.header.frame_id!='sdk_measured' or ns<=self.last_joint_ns:return
     self.last_joint_ns=ns
     q=[float(values[n]) for n in names]
     if not np.isfinite(q).all() or not 0<=(rospy.Time.now().to_nsec()-ns)*1e-9<=.3:return
     self.states.append((ns,q));self.states=self.states[-300:]
 def archive_message(self,topic,msg,stamp):
  stream=io.BytesIO()
  try:msg.serialize(stream)
  except Exception as exc:
   self.archive_writer.latch_failure('diagnostic message serialization failed: '+str(exc));raise
  self.archive_writer.submit(topic,stream.getvalue(),stamp.to_nsec(),msg._type,msg._md5sum,type(msg))
 def guard(self):
  if self.archive_writer is not None:self.archive_writer.require_healthy()
  if self.fatal:raise RuntimeError(self.fatal)
  if self.latest_task_active:raise RuntimeError('another grasp task active')
  if not MOUNT.is_mount():raise RuntimeError('experiment drive disappeared')
  if rospy.is_shutdown():raise RuntimeError('ROS shutdown')
 def mark(self,event,**kw):
  row=dict(event=event,time_ns=rospy.Time.now().to_nsec(),**kw)
  with self.lock:
   with (self.out/'events.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
   self.archive_message('/support_diagnostic/event',String(data=json.dumps(row)),rospy.Time.now())
  print(json.dumps({k:v for k,v in row.items() if k!="audit"}),flush=True)
 def snapshot(self,label,after_ns=0):
  end=time.monotonic()+8
  while time.monotonic()<end:
   self.guard();now_ns=rospy.Time.now().to_nsec()
   with self.lock:
    records=[(ns,q) for ns,q in self.states if ns>after_ns and 0<=(now_ns-ns)*1e-9<=1.]
    recent=[q[:6] for ns,q in records]
    pairs=[(ns,p.copy()) for ns,p in self.frames.items() if len(p)==2 and ns>after_ns and .10<=(rospy.Time.now().to_nsec()-ns)*1e-9<=.5]
   stable=len(recent)>=7 and (records[-1][0]-records[0][0])>=600000000 and (now_ns-records[-1][0])<=300000000 and np.max(np.ptp(np.array(recent),axis=0))<=math.radians(.15)
   if stable and pairs:
    ns,pair=max(pairs)
    before=[r for r in records if r[0]<=ns];after=[r for r in records if r[0]>=ns]
    if not before or not after:time.sleep(.05);continue
    lo,hi=before[-1],after[0]
    if hi[0]-lo[0]>200000000:time.sleep(.05);continue
    try:tr=self.buffer.lookup_transform('base_link','camera_link',rospy.Time(*divmod(ns,10**9)),rospy.Duration(.15))
    except Exception:time.sleep(.05);continue
    t=tr.transform.translation;q=tr.transform.rotation;F=quaternion_matrix([q.x,q.y,q.z,q.w]);F[:3,3]=[t.x,t.y,t.z]
    rgb=self.bridge.imgmsg_to_cv2(pair['color'],'bgr8');depth=self.bridge.imgmsg_to_cv2(pair['depth'],'passthrough')
    path=self.out/(str(ns)+'_rgbd_tf.npz')
    np.savez_compressed(str(path),image_bgr=rgb,depth_raw=depth,depth_scale=self.camera['depth_scale'],K=self.K,T_base_ros_camera=F,T_base_optical=F@C,source_stamp_ns=np.int64(ns))
    cv2.imwrite(str(self.out/(str(ns)+'_rgb.png')),rgb)
    self.mark('stationary_snapshot',label=label,source_stamp_ns=ns,q_source_before=lo,q_source_after=hi,span_deg=np.degrees(np.ptp(recent,axis=0)).tolist(),file=str(path));return ns
   time.sleep(.05)
  raise RuntimeError('No stationary source-paired image/TF window')
 def close(self):
  with self.lock:
   self.active=False
  for s in self.subs:s.unregister()
  writer_summary=None
  try:
   if self.archive_writer is not None:
    writer_summary=self.archive_writer.close(timeout=30.)
   if self.bag is not None:self.bag.close()
   if writer_summary is not None and not writer_summary['archive_complete']:
    for index,(metadata,payload,_type,_cost) in enumerate(self.archive_writer.failed_payloads):
     (self.out/('failed_payload_%03d.bin'%index)).write_bytes(payload)
     (self.out/('failed_payload_%03d.json'%index)).write_text(json.dumps(metadata,indent=2)+'\n')
    raise RuntimeError('diagnostic archive incomplete: '+str(writer_summary['failure']))
  finally:
   if self.archive_writer is not None and writer_summary is None:writer_summary=self.archive_writer.summary()
   (self.out/'counts.json').write_text(json.dumps(dict(counts=self.counts,recording_fatal=self.fatal,archive_writer=writer_summary),indent=2)+'\n')

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--plan-module',required=True);parser.add_argument('--execute',action='store_true');args=parser.parse_args()
 if not args.execute:raise RuntimeError('Explicit --execute required; prepare/review plan first')
 if not MOUNT.is_mount():raise RuntimeError('SanStand unavailable; no fallback')
 rospy.init_node('support_single_axis_experiment',anonymous=True,disable_signals=True)
 gs=rospy.wait_for_message('/grasp/state',GraspState,timeout=3)
 if gs.active:raise RuntimeError('Task active')
 # Plan module is loaded only by explicit local path after its read-only review.
 spec=importlib.util.spec_from_file_location('support_axis_plan',args.plan_module);plan=importlib.util.module_from_spec(spec);spec.loader.exec_module(plan_module:=plan)
 out=OUTROOT/('single_axis_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir()
 recorder=None;owner=None
 def interrupted(signum,_frame):raise RuntimeError('diagnostic interrupted by signal %s'%signum)
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGALRM):signal.signal(sig,interrupted)
 signal.alarm(180)
 try:
  recorder=Recorder.__new__(Recorder);Recorder.__init__(recorder,out)
  plan=plan_module.prepare_executor_audit(out/'fresh_plan')
  source_q=np.asarray(plan.source_q,dtype=float)
  if source_q.shape!=(6,) or not np.isfinite(source_q).all():raise RuntimeError('invalid plan source q')
  if len(plan.steps)!=4 or any(i not in (1,4) or (d is not None and (not math.isfinite(d) or abs(d)>3.)) for i,d in plan.steps):raise RuntimeError('invalid bounded step list')
  evidence={'module':str(Path(args.plan_module).resolve()),'module_sha256':hashlib.sha256(Path(args.plan_module).read_bytes()).hexdigest(),'collector_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'source_q':source_q.tolist(),'steps':plan.steps,'return_semantics':'request measured starting joint angle, verify all six axes within 0.2 deg; not a frozen SDK word return'}
  (out/'execution_inputs.json').write_text(json.dumps(evidence,indent=2)+'\n')
  recorder.snapshot('baseline',plan.source_stamp_ns)
  recorder.guard()
  # Ownership change is not a torque command. The user finished manual adjustment.
  owner=rospy.Publisher('/gui/joint_direct_mode',Bool,queue_size=1,latch=True)
  rospy.set_param('/gui/joint_direct_mode',True);time.sleep(.3);owner.publish(Bool(data=True));time.sleep(.4)
  motion=AutoMotion(out);seed=np.asarray(motion.q[:6]).copy()
  if np.max(np.abs(seed-np.asarray(plan.source_q)))>math.radians(.2):raise RuntimeError('Posture changed since prepared plan')
  original_guard=motion.guard
  def guard():
   recorder.guard();original_guard()
  motion.guard=guard
  motion.command_guard=lambda target:plan.guard_segment(np.asarray(motion.q[:6]),np.asarray(target[:6]))
  original_apply=motion.apply
  def apply(decision):
   guard();action,target,detail=decision
   if action=='enable':recorder.allow_epoch_until=time.monotonic()+5
   if action in ('sync','probe','publish'):
    t=recorder.buffer.lookup_transform('tool0','camera_link',rospy.Time(0),rospy.Duration(.2)).transform
    X=quaternion_matrix([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]);X[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    if not np.allclose(X,plan.snapshot['T_tool_camera'],rtol=0,atol=1e-9):raise RuntimeError('handeye changed during experiment')
    camera=rospy.get_param('/camera')
    if camera['depth_scale']!=recorder.camera['depth_scale']:raise RuntimeError('depth scale changed')
    audit=motion.command_guard(target);recorder.mark('actual_submission_geometry',kind=action,target=target,audit=audit)
    guard()
   return original_apply(decision)
  motion.apply=apply
  recorder.armed=True
  for index,delta_deg in plan.steps:
   # None means measured return toward frozen baseline, never commanded zero.
   delta=float(np.degrees(seed[index]-motion.q[index])) if delta_deg is None else float(delta_deg)
   if abs(delta)<.2:
    recorder.mark('return_already_near_baseline',joint=index+1,error_deg=delta)
    recorder.snapshot('joint%d_return_no_command'%(index+1),rospy.Time.now().to_nsec())
    errors=np.degrees(np.asarray(motion.q[:6])-seed);recorder.mark('return_error',all_joint_errors_deg=errors.tolist())
    if np.max(np.abs(errors))>.2:raise RuntimeError('unchanged return has other axis drift; no next axis')
    continue
   if abs(delta)>4:raise RuntimeError('Measured return exceeds bounded single step')
   recorder.mark('step_begin',joint=index+1,delta_deg=delta,q=motion.q)
   motion.move(index,delta);settled_ns=rospy.Time.now().to_nsec();time.sleep(2);recorder.snapshot('joint%d_%s'%(index+1,'return_attempt' if delta_deg is None else 'probe'),settled_ns)
   if delta_deg is None:
    errors=np.degrees(np.asarray(motion.q[:6])-seed);recorder.mark('return_error',all_joint_errors_deg=errors.tolist())
    if np.max(np.abs(errors))>.2:raise RuntimeError('return did not reach starting measured posture within 0.2deg; no next axis')
  recorder.mark('sequence_complete',final_q=motion.q)
 except BaseException as exc:
  if recorder is not None:
   try:recorder.mark('sequence_failed',reason=str(exc))
   except Exception:pass
  raise
 finally:
  # Leave joint torque and physical held target untouched. Save partial data too.
  signal.alarm(0)
  if recorder is not None:recorder.close()
if __name__=='__main__':main()
