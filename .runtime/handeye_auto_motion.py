#!/usr/bin/env python3
"""One bounded adjustment through the hand-eye GUI's existing control protocol."""
import sys,json,time,threading,math,xmlrpc.client
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool,String
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity,GetStateValidityRequest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/alicia_flexible_grasp_supervisor'))
from gui.widgets.joint_control_widget import DirectControlRecovery
NAMES=['Joint%d'%i for i in range(1,7)]+['right_finger']
class AutoMotion:
 def __init__(self,session):
  self.session=Path(session);self.lock=threading.RLock();self.q=None;self.stamp=0;self.received=0;self.ready=False;self.protected=True;self.manual=False;self.status='';self.history=[];self.external=False;self.manual_seen=False;self.protected_seen=False;self.ready_seen=False
  self.recovery=DirectControlRecovery(timeout_sec=2.,enable_retry_sec=.5)
  self.log=(self.session/'automatic_motion.jsonl').open('a',buffering=1)
  self.pub=rospy.Publisher('/joint_commands',JointState,queue_size=1,latch=False)
  self.enable=rospy.Publisher('/demonstration',Bool,queue_size=1,latch=False)
  self.subs=[rospy.Subscriber('/alicia_d/accepted_joint_states',JointState,self.feedback,queue_size=5),rospy.Subscriber('/alicia_d/feedback_ready',Bool,self.ready_cb,queue_size=1),rospy.Subscriber('/alicia_d/protection_latched',Bool,lambda m:self.flag('protected',m.data),queue_size=1),rospy.Subscriber('/gui/joint_direct_mode',Bool,lambda m:self.flag('manual',m.data),queue_size=1),rospy.Subscriber('/alicia_d/actuation_status',String,self.status_cb,queue_size=1),rospy.Subscriber('/joint_commands',JointState,self.commands,queue_size=10)]
  end=time.monotonic()+6
  while time.monotonic()<end and (self.q is None or not self.status or not self.manual_seen or not self.protected_seen or not self.ready_seen or self.pub.get_num_connections()==0):time.sleep(.03)
  self.guard()
  code,detail,uri=rospy.get_master().lookupNode('/alicia_d_driver_node')
  if code!=1:raise RuntimeError('driver node unavailable')
  api=xmlrpc.client.ServerProxy(uri)
  connected=False;end=time.monotonic()+5
  while time.monotonic()<end:
   code,detail,rows=api.getBusInfo(rospy.get_name())
   connected=code==1 and any(len(row)>=6 and row[1] in (rospy.get_name(),rospy.get_node_uri()) and row[2]=='i' and row[4]=='/joint_commands' and row[5] for row in rows)
   if connected:break
   time.sleep(.05)
  if not connected:raise RuntimeError('driver command TCP connection not established')
  self.emit('driver_command_connection_confirmed',node=rospy.get_name())
 def emit(self,event,**kw):
  row={'event':event,'time':time.time(),**kw}
  with self.lock:self.log.write(json.dumps(row,ensure_ascii=False)+'\n')
  print(json.dumps(row,ensure_ascii=False),flush=True)
 def feedback(self,m):
  values=dict(zip(m.name,m.position))
  if any(n not in values for n in NAMES):return
  now=time.monotonic();self.q=[float(values[n]) for n in NAMES];self.received=now;self.stamp=m.header.stamp.to_nsec();self.recovery.note_joint_feedback(self.q,now)
  self.history.append((now,self.q.copy()));self.history=self.history[-160:]
 def flag(self,name,value):setattr(self,name,bool(value));setattr(self,name+'_seen',True)
 def ready_cb(self,m):self.ready_seen=True;self.ready=bool(m.data);self.recovery.update_feedback_ready(self.ready)
 def status_cb(self,m):
  if self.status!=m.data:self.emit('actuation_status',status=m.data)
  self.status=m.data;self.recovery.update_actuation_status(m.data)
 def commands(self,m):
  caller=getattr(m,'_connection_header',{}).get('callerid','')
  if caller!=rospy.get_name() and m.header.frame_id.startswith('gui_direct'):self.external=True;self.emit('external_gui_input',caller=caller)
 def guard(self):
  if rospy.is_shutdown():raise RuntimeError('ROS shutdown')
  if self.external:raise RuntimeError('Another GUI operator is issuing commands')
  if not self.manual:raise RuntimeError('GUI direct control ownership changed')
  if self.protected:raise RuntimeError('driver protection active')
  if not self.ready or self.q is None or time.monotonic()-self.received>.3:raise RuntimeError('accepted feedback unavailable/stale')
  if not np.isfinite(self.q).all():raise RuntimeError('non-finite feedback')
 def apply(self,decision):
  action,target,detail=decision
  if action in ('none','wait'):return action
  self.guard()
  if action=='enable':
   self.enable.publish(Bool(data=False));self.emit('positive_enable_request',topic='/demonstration',value=False);return action
  if action in ('clear','timeout'):raise RuntimeError('GUI handshake: '+detail)
  if action not in ('sync','probe','publish'):raise RuntimeError('unknown GUI action '+action)
  index=None if action=='sync' else self.recovery.last_action_edited_index
  msg=JointState();msg.header.stamp=rospy.Time.now();msg.header.frame_id='gui_direct_sync' if index is None else 'gui_direct';msg.name=NAMES;msg.position=list(target);msg.effort=[0.]*7
  if index is not None:msg.effort[index]=1.
  self.emit('command',kind=action,edited_index=index,q=msg.position,source_feedback_stamp_ns=self.stamp)
  self.pub.publish(msg);return action
 def move(self,index,delta_deg):
  self.guard();assert 0<=index<6 and .01<=abs(delta_deg)<=4.0
  start=np.array(self.q[:6]);target=list(self.q);target[index]+=math.radians(delta_deg)
  lo=rospy.get_param('/robot/joint_min_rad',[-3.14,-2.5,-2.5,-3.14,-2.5,-3.14]);hi=rospy.get_param('/robot/joint_max_rad',[3.14,2.5,2.5,3.14,2.5,3.14]);assert lo[index]<=target[index]<=hi[index]
  # Task verification only: reserve the possible measured endpoint before any
  # command. Keep the original 4.8 degree absolute local-response ceiling.
  extra_deg=.8
  if getattr(self,'task_pose_directional_reservation',False) and index in (1,2) and delta_deg<0:
   if abs(delta_deg)>3.0000001:raise RuntimeError('reverse shoulder/elbow observation step exceeds 3 degrees')
   extra_deg=1.2
  response_bound_deg=4.8 if getattr(self,'task_pose_directional_reservation',False) else min(4.8,abs(delta_deg)+extra_deg)
  reserved=list(target);reserved[index]=start[index]+math.copysign(math.radians(response_bound_deg),delta_deg)
  if hasattr(self,'command_guard'):
   self.command_guard(target);self.command_guard(reserved)
  rospy.wait_for_service('/check_state_validity',timeout=3);valid=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
  for alpha in np.linspace(0,1,9):
   request=GetStateValidityRequest();request.group_name='alicia';request.robot_state=RobotState();request.robot_state.joint_state=JointState(name=NAMES,position=(list(start+(np.array(reserved[:6])-start)*alpha)+[target[6]]));response=valid(request)
   if not response.valid:raise RuntimeError('Existing MoveIt collision constraints reject the local segment')
  self.emit('move_requested',joint=index+1,delta_deg=delta_deg,start_deg=np.degrees(start).tolist(),target_deg=np.degrees(target[:6]).tolist(),reserved_response_max_deg=response_bound_deg)
  action=self.apply(self.recovery.submit(target,time.monotonic(),edited_index=index,rebase_unedited=False))
  deadline=time.monotonic()+4
  waiting_for_feedback=False
  while action!='publish' and time.monotonic()<deadline:
   time.sleep(.04)
   # Positive enable deliberately resets the driver's ready flag. Wait without
   # publishing until fresh measured feedback is back; all command guards stay.
   if not self.ready or self.q is None or time.monotonic()-self.received>.3:
    if rospy.is_shutdown() or self.external or not self.manual or self.protected:self.guard()
    if not waiting_for_feedback:self.emit('handshake_waiting_for_fresh_feedback');waiting_for_feedback=True
    continue
   self.guard();action=self.apply(self.recovery.poll(time.monotonic()))
  if action!='publish':raise RuntimeError('full bounded target not admitted')
  deadline=time.monotonic()+18;last=0
  while time.monotonic()<deadline:
   time.sleep(.05);self.guard();now=time.monotonic();recent=[q for t,q in self.history if t>now-.8]
   err=abs(self.q[index]-target[index]);span=np.max(np.ptp(np.array(recent)[:,:6],axis=0)) if len(recent)>=6 else float('inf')
   if now-last>1:self.emit('motion_feedback',joint=index+1,error_deg=math.degrees(err),span_deg=math.degrees(span) if math.isfinite(span) else None,q_deg=np.degrees(self.q[:6]).tolist());last=now
   response=(self.q[index]-start[index])*(1 if delta_deg>0 else -1)
   # Calibration records measured poses; do not chase nominal servo deadband.
   settled_response=(response>=.003 and response<=math.radians(response_bound_deg))
   if len(recent)>=6 and span<math.radians(.15) and settled_response:
    self.emit('move_settled',joint=index+1,actual_delta_deg=math.degrees(self.q[index]-start[index]),residual_deg=math.degrees(err));return
  raise RuntimeError('Bounded move lacks a stationary directional response; no additional command sent')
if __name__=='__main__':
 config=json.loads((ROOT/'.runtime/current_handeye_autocalibration.json').read_text());rospy.init_node('automatic_handeye_gui_motion',anonymous=True,disable_signals=True)
 motion=AutoMotion(config['session'])
 try:motion.move(int(sys.argv[1])-1,float(sys.argv[2]))
 except Exception as exc:motion.emit('move_failed',reason=str(exc));raise
