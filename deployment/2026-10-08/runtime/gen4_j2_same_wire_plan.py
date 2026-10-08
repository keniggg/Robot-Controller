#!/usr/bin/env python3
"""Fresh J2-only +34/+68/+34/0 SDK-word audit. Never publishes commands.
prepare_executor_audit(directory) is the existing collector factory interface.
The strict guard's q1 argument is an ACTUAL intended complete SDK command,
not an old AutoMotion artificial reservation: this guard adds .8deg itself.
"""
import pathlib,sys,math,json,hashlib,os,time,signal
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import gen4_single_axis_plan as base
INDEX=1;COUNTS=[34,68,34,0];DRIFT_DEG=.5;EXTRA_DEG=.8;MAX_STEP_DEG=4.
OUT=base.OUT.parent/'j2_same_wire_plan'
class J2SameWireAudit:
 def __init__(self,snapshot,xml,validity):
  self.snapshot=snapshot;self.source_stamp_ns=int(snapshot['source_stamp_ns']);self.source_q=np.asarray(snapshot['q_rad'],float)[:6].copy();self.reference=np.asarray(snapshot['sdk_q_rad'],float).copy();self.audit=base.GeometryAudit(snapshot,xml,validity);self.xml=xml
  self.steps=[(INDEX,2.98828125),(INDEX,2.98828125),(INDEX,-2.98828125),(INDEX,-2.98828125)]
  self.absolute_sdk_targets=[]
  for count in COUNTS:
   q=self.reference.copy();q[INDEX]+=count*base.QUANTUM
   self.absolute_sdk_targets.append(q.tolist())
  self.edited_axis_index=INDEX;self.unedited_max_actual_drift_rad=math.radians(DRIFT_DEG)
  self.edited_axis_absolute_bounds_rad=[float(min(self.source_q[INDEX],self.reference[INDEX])-math.radians(EXTRA_DEG)),float(max(self.source_q[INDEX],self.reference[INDEX]+68*base.QUANTUM)+math.radians(EXTRA_DEG))]
  self.contract={'kind':'j2_positive68_counts_then_exact_original_sdk_words','source_stamp_ns':self.source_stamp_ns,'edited_index':INDEX,'absolute_sdk_targets':self.absolute_sdk_targets,'absolute_sdk_target_words':[base.words(q) for q in self.absolute_sdk_targets],'original_sdk_words':base.words(self.reference),'edited_absolute_bounds_rad':self.edited_axis_absolute_bounds_rad,'edited_absolute_bounds_deg':np.degrees(self.edited_axis_absolute_bounds_rad).tolist(),'initial_actual_q_rad':self.source_q.tolist(),'max_unedited_actual_drift_deg':DRIFT_DEG,'max_actual_to_command_step_deg':MAX_STEP_DEG,'response_extra_deg':EXTRA_DEG,'response_extra_is_not_compounded':True,'return_semantics':'Exact original SDK words, not assumed identical measured physical return.','guard_q0':'Fresh actual six/seven joints; caller binds source/epoch/authority at actual submission.','guard_q1':'Complete actual intended SDK command (6/7 values); internally reserves additional0.8deg. Do not pass synthetic4.8deg reservation as command.'}
 def _check_model(self):
  import rospy
  from tf.transformations import quaternion_matrix
  xml=rospy.get_param('/robot_description')
  if hashlib.sha256(xml.encode()).hexdigest()!=self.snapshot['urdf_sha256']:raise RuntimeError('URDF changed')
  cam=rospy.get_param('/camera')
  if not np.array_equal([cam[k] for k in ['fx','fy','cx','cy']],self.snapshot['K']) or cam['width']!=self.snapshot['width'] or cam['height']!=self.snapshot['height'] or cam['depth_scale']!=self.snapshot['depth_scale']:raise RuntimeError('camera K/dimensions/depth_scale changed')
  he=rospy.get_param('/handeye');X=quaternion_matrix(he['rotation_xyzw']);X[:3,3]=he['translation_xyz']
  if not np.allclose(X,self.snapshot['T_tool_camera'],rtol=0,atol=1e-9):raise RuntimeError('handeye X changed')
 def _validate(self,q0,q1):
  a=np.asarray(q0,float);b=np.asarray(q1,float)
  if a.shape not in [(6,),(7,)] or b.shape not in [(6,),(7,)] or not np.isfinite(a).all() or not np.isfinite(b).all():raise RuntimeError('invalid actual/SDK vector')
  if len(b)==7 and abs(b[6]-self.reference[6])>1e-12:raise RuntimeError('gripper SDK changed')
  if len(a)==7 and abs(a[6]-self.snapshot['q_rad'][6])>1e-4:raise RuntimeError('gripper actual changed')
  a=a[:6];b=b[:6];other=np.arange(6)!=INDEX
  if np.any(abs(a[other]-self.source_q[other])>self.unedited_max_actual_drift_rad+1e-12):raise RuntimeError('unedited actual axis drift exceeds0.5deg')
  aw=base.words(self.reference);bw=base.words(b)
  if any(bw[i]!=aw[i] for i in range(6) if i!=INDEX):raise RuntimeError('unedited SDK word changed')
  # Handshake targets, if any, must also remain within the frozen commanded
  # J2 range; geometry allowance is not an expanded command authorization.
  if not aw[INDEX]<=bw[INDEX]<=aw[INDEX]+68:raise RuntimeError('J2 SDK target outside frozen 0..+68 word range')
  if abs(b[INDEX]-a[INDEX])>math.radians(MAX_STEP_DEG)+1e-12:raise RuntimeError('actual-to-command J2 delta exceeds4deg')
  lo,hi=self.edited_axis_absolute_bounds_rad
  if not lo-1e-12<=a[INDEX]<=hi+1e-12 or not lo-1e-12<=b[INDEX]<=hi+1e-12:raise RuntimeError('J2 actual/command outside fixed absolute bounds')
  direction=0. if abs(b[INDEX]-a[INDEX])<1e-12 else math.copysign(1,b[INDEX]-a[INDEX]);reserved=b[INDEX]+direction*math.radians(EXTRA_DEG)
  if not lo-1e-12<=reserved<=hi+1e-12:raise RuntimeError('J2 .8deg response reserve outside fixed absolute bounds')
  return a,b,reserved
 def geometry(self,q):return self.audit.geometry(q)
 def _evaluate_segment(self,q0,q1,check_model):
  if check_model:self._check_model()
  a,b,reserved=self._validate(q0,q1);self.audit.cache.clear()
  nominal=self.audit.guard_segment(a,b);rows=[];low=min(a[INDEX],b[INDEX],reserved);high=max(a[INDEX],b[INDEX],reserved);n=max(2,int(math.ceil(math.degrees(high-low)/.15))+1)
  for bridge in [0,.5,1]:
   held=a*(1-bridge)+b*bridge
   for angle in np.linspace(low,high,n):
    q=held.copy();q[INDEX]=angle;r=self.audit.geometry(q);r['held_axis_bridge']=bridge;rows.append(r)
  reservation=self.audit.summarize(rows)
  result={'source_stamp_ns':self.source_stamp_ns,'q_actual_rad':a.tolist(),'sdk_target_rad':np.asarray(q1).tolist(),'sdk_target_words':base.words(b),'actual_step_deg':math.degrees(b[INDEX]-a[INDEX]),'reserved_response_angle_deg':math.degrees(reserved),'fixed_edited_absolute_bounds_deg':np.degrees(self.edited_axis_absolute_bounds_rad).tolist(),'unedited_actual_drift_deg':np.degrees(a-self.source_q).tolist(),'nominal':nominal,'reservation':reservation,'all_pass':nominal['all_geometry_pass'] and nominal['all_moveit_valid'] and reservation['all_geometry_pass'] and reservation['all_moveit_valid']}
  if not result['all_pass']:raise RuntimeError('J2 source-bound visibility/40mm plane/current MoveIt failed: '+json.dumps({k:v for k,v in reservation.items() if k!='rows'}))
  return result
 def guard_segment(self,q0,q1):return self._evaluate_segment(q0,q1,True)
 def audit_nominal_sequence(self):
  actual=self.source_q.copy();proof=[]
  for target in self.absolute_sdk_targets:
   proof.append(self._evaluate_segment(actual,target,True));actual[INDEX]=target[INDEX] # predicted endpoint only; held actual stays frozen
  return {'kind':'predicted_J2_same_wire_four_step_geometry_audit','source_stamp_ns':self.source_stamp_ns,'contract':self.contract,'proof_steps':proof,'all_pass':all(p['all_pass'] for p in proof),'robot_commands_published':0,'predicted_endpoints_are_not_measured':True,'fresh_factory_inputs_sha256':hashlib.sha256(json.dumps(self.snapshot,sort_keys=True,separators=(',',':')).encode()).hexdigest()}
def prepare_executor_audit(directory):
 import rospy
 from sensor_msgs.msg import JointState
 from moveit_msgs.msg import RobotState
 from moveit_msgs.srv import GetStateValidity,GetStateValidityRequest
 directory=pathlib.Path(directory);directory.mkdir(parents=True,exist_ok=True);snapshot,xml=base.capture(directory)
 if snapshot['task_active'] or snapshot['protection_latched'] or not snapshot['feedback_ready'] or not snapshot['actuation_status'].startswith('CONFIRMED:'):raise RuntimeError('fresh task/protection/feedback/actuation not eligible')
 rospy.wait_for_service('/check_state_validity',timeout=3);proxy=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
 def valid(q,g):
  req=GetStateValidityRequest(group_name='alicia');req.robot_state=RobotState();req.robot_state.joint_state=JointState(name=base.NAMES,position=list(q)+[g]);return proxy(req).valid
 obj=J2SameWireAudit(snapshot,xml,valid);proof=obj.audit_nominal_sequence();base.write(directory/'factory_audit_proof.json',proof);base.write(directory/'contract.json',obj.contract);(directory/'factory_source.py').write_bytes(pathlib.Path(__file__).read_bytes());return obj
def main():
 if not pathlib.Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('SanStand unavailable')
 OUT.mkdir(exist_ok=True);directory=OUT/time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());directory.mkdir();(directory/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(directory/'ros')
 import rospy
 rospy.init_node('gen4_j2_same_wire_readonly_audit',anonymous=True,disable_signals=True);signal.signal(signal.SIGALRM,lambda *_:(_ for _ in ()).throw(RuntimeError('readonly audit55s deadline')));signal.alarm(55)
 try:
  plan=prepare_executor_audit(directory);base.write(OUT/'latest.json',{'directory':str(directory),'source_stamp_ns':plan.source_stamp_ns});proof=json.loads((directory/'factory_audit_proof.json').read_text());print(json.dumps({'directory':str(directory),'source_stamp_ns':plan.source_stamp_ns,'contract':plan.contract,'steps':[{'step_deg':p['actual_step_deg'],'min_clearance_m':p['reservation']['minimum_clearance_m'],'min_target_margin_px':p['reservation']['minimum_target_margin_px'],'min_texture_margin_px':p['reservation']['minimum_texture_margin_px'],'all_pass':p['all_pass']} for p in proof['proof_steps']]},indent=2))
 except BaseException as e:base.write(directory/'failure.json',{'error':str(e),'robot_commands_published':0});raise
 finally:signal.alarm(0)
if __name__=='__main__':main()
