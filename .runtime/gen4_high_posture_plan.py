#!/usr/bin/env python3
"""Draft source-bound high-posture audit. Import/factory never publish commands.

All targets are frozen absolute SDK words. The whole alternating staircase,
independent probes, and reverse staircase are audited before an executor starts.
Runtime guards requery geometry/MoveIt from the actual endpoint at every step.
"""
import argparse, hashlib, json, math, os, pathlib, signal, time
import numpy as np
import gen4_single_axis_plan as base

STEP_COUNTS = 34
STEP_DEG = STEP_COUNTS * 360. / 4096
DRIFT_DEG = .5
EXTRA_DEG = .8
MAX_STEP_DEG = 4.
EDITABLE = (1, 4)
OUT = base.OUT.parent / 'high_posture_plan'

def vector(q):
 a = np.asarray(q, dtype=float)
 if a.shape not in ((6,), (7,)) or not np.isfinite(a).all():
  raise RuntimeError('invalid actual/SDK joint vector')
 return a

def build_sequence(reference, cycles=7, j2_probe_sign=1):
 reference = vector(reference)
 if reference.shape != (7,) or cycles not in (5, 7, 8) or j2_probe_sign not in (-1, 1):
  raise RuntimeError('only bounded 15/21/24deg domains and signed single J2 probe supported')
 stages = []; current = reference.copy(); outbound = []
 def append(axis, target, label, phase):
  nonlocal current
  target = vector(target).copy()
  before = base.words(current); after = base.words(target)
  edited = [i for i in range(6) if before[i] != after[i]]
  if edited != [axis] or abs(after[axis]-before[axis]) > STEP_COUNTS:
   raise RuntimeError('sequence step must edit exactly one joint by at most34counts')
  if target[6] != reference[6]: raise RuntimeError('sequence gripper changed')
  stages.append(dict(index=len(stages), axis=axis, label=label, phase=phase,
                     before_sdk=current.tolist(), target_sdk=target.tolist(),
                     before_words=before, target_words=after))
  current = target
 for cycle in range(cycles):
  for axis, sign in ((1, 1), (4, -1)):
   previous = current.copy(); target = current.copy(); target[axis] += sign*STEP_COUNTS*base.QUANTUM
   append(axis, target, 'domain_%02d_j%d'%(cycle+1,axis+1), 'outbound')
   outbound.append((axis, previous))
 high = current.copy()
 for axis, sign, label in ((4,-1,'high_j5_negative3'), (1,j2_probe_sign,'high_j2_%s3'%('positive' if j2_probe_sign>0 else 'negative'))):
  target = high.copy(); target[axis] += sign*STEP_COUNTS*base.QUANTUM
  append(axis, target, label, 'probe')
  append(axis, high, label+'_return', 'probe_return')
 for n,(axis, target) in enumerate(reversed(outbound)):
  append(axis, target, 'return_%02d_j%d'%(n+1,axis+1), 'return')
 if not np.array_equal(current, reference): raise RuntimeError('sequence must restore exact original SDK values')
 return stages

class HighPostureAudit:
 def __init__(self, snapshot, xml, validity, cycles=7, j2_probe_sign=1, audit_factory=None):
  self.snapshot = snapshot; self.xml = xml
  self.source_stamp_ns = int(snapshot['source_stamp_ns'])
  self.source_q = vector(snapshot['q_rad'])[:6].copy()
  self.reference = vector(snapshot['sdk_q_rad']).copy()
  self.sequence = build_sequence(self.reference, cycles, j2_probe_sign)
  self.audit = (audit_factory or base.GeometryAudit)(snapshot, xml, validity)
  targets = np.array([self.reference[:6]]+[s['target_sdk'][:6] for s in self.sequence]+[self.source_q.tolist()])
  self.absolute_lo = targets.min(0)-math.radians(EXTRA_DEG)
  self.absolute_hi = targets.max(0)+math.radians(EXTRA_DEG)
  self.cursor = 0; self.active = None; self.prepared = False
  self.contract = dict(kind='source_bound_alternating_J2_J5_high_posture',
   source_stamp_ns=self.source_stamp_ns, epoch_ns=int(snapshot['epoch_ns']),
   original_sdk_words=base.words(self.reference), original_sdk=self.reference.tolist(),
   cycles=cycles, j2_probe_sign=j2_probe_sign, sequence=self.sequence,
   max_sdk_word_step=STEP_COUNTS, max_sdk_step_deg=STEP_DEG,
   max_actual_to_command_step_deg=MAX_STEP_DEG, response_extra_deg=EXTRA_DEG,
   response_extra_is_not_compounded=True, globally_unedited_max_actual_drift_deg=DRIFT_DEG,
   per_step_unedited_max_actual_drift_deg=DRIFT_DEG,
   geometry_min_clearance_m=.04, target_and_texture_min_edge_px=20,
   return_semantics='Exact original SDK words; actual and visual return are measured, never assumed.',
   control_contract='Already CONFIRMED. No enable/sync/probe recovery permitted; source epoch immutable.',
   absolute_actual_bounds_rad=[self.absolute_lo.tolist(),self.absolute_hi.tolist()],
   nominal_full_path_is_not_future_actual_response_guarantee=True)

 def check_model(self):
  import rospy
  from tf.transformations import quaternion_matrix
  if hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest()!=self.snapshot['urdf_sha256']:
   raise RuntimeError('URDF changed')
  c=rospy.get_param('/camera')
  if not np.array_equal([c[k] for k in ('fx','fy','cx','cy')],self.snapshot['K']) or any(c[k]!=self.snapshot[k] for k in ('width','height','depth_scale')):
   raise RuntimeError('camera intrinsics/dimensions/depth scale changed')
  h=rospy.get_param('/handeye');X=quaternion_matrix(h['rotation_xyzw']);X[:3,3]=h['translation_xyz']
  if not np.allclose(X,self.snapshot['T_tool_camera'],rtol=0,atol=1e-9): raise RuntimeError('handeye changed')

 def check_actual(self, q):
  q=vector(q)
  if len(q)==7 and abs(q[6]-self.snapshot['q_rad'][6])>1e-4: raise RuntimeError('actual gripper changed')
  a=q[:6]
  unedited=[i for i in range(6) if i not in EDITABLE]
  if np.max(abs(a[unedited]-self.source_q[unedited]))>math.radians(DRIFT_DEG)+1e-12:
   raise RuntimeError('globally unedited actual axis drift exceeds0.5deg')
  if np.any(a<self.absolute_lo-1e-12) or np.any(a>self.absolute_hi+1e-12):
   raise RuntimeError('actual outside fixed whole-sequence bounds')
  return a

 def _evaluate(self, actual, target, axis, step_start, check_model):
  if check_model:self.check_model()
  a=self.check_actual(actual);b=vector(target);start=vector(step_start)[:6]
  if len(b)!=7 or b[6]!=self.reference[6]: raise RuntimeError('SDK gripper changed')
  held=[i for i in range(6) if i!=axis]
  if np.max(abs(a[held]-start[held]))>math.radians(DRIFT_DEG)+1e-12: raise RuntimeError('per-step unedited actual drift exceeds0.5deg')
  if abs(b[axis]-a[axis])>math.radians(MAX_STEP_DEG)+1e-12: raise RuntimeError('actual-to-command delta exceeds4deg')
  # Fixed one-step response bounds; repeated validation cannot accumulate .8deg.
  direction=math.copysign(1.,b[axis]-start[axis]) if b[axis]!=start[axis] else 0.
  reserved=b[axis]+direction*math.radians(EXTRA_DEG)
  low=min(start[axis],b[axis],reserved)
  high=max(start[axis],b[axis],reserved)
  if not low-1e-12<=a[axis]<=high+1e-12: raise RuntimeError('actual outside fixed step response bounds')
  if low<self.absolute_lo[axis]-1e-12 or high>self.absolute_hi[axis]+1e-12: raise RuntimeError('response reserve outside whole-sequence bounds')
  self.audit.cache.clear()
  nominal=self.audit.guard_segment(a,b[:6]);rows=[]
  # Held-axis actual->frozen-SDK bridge preserves the existing conservative audit.
  n=max(2,int(math.ceil(math.degrees(high-low)/.15))+1)
  for bridge in (0.,.5,1.):
   heldq=a*(1-bridge)+b[:6]*bridge
   for angle in np.linspace(low,high,n):
    q=heldq.copy();q[axis]=angle;row=self.audit.geometry(q);row['held_axis_bridge']=bridge;rows.append(row)
  reserve=self.audit.summarize(rows)
  result=dict(source_stamp_ns=self.source_stamp_ns,axis=axis,q_actual_rad=a.tolist(),
   target_sdk=b.tolist(),target_words=base.words(b),step_start_actual_rad=start.tolist(),
   fixed_step_response_bounds_deg=np.degrees([low,high]).tolist(),nominal=nominal,reservation=reserve,
   all_pass=nominal['all_geometry_pass'] and nominal['all_moveit_valid'] and reserve['all_geometry_pass'] and reserve['all_moveit_valid'])
  if not result['all_pass']:raise RuntimeError('40mm/table/FOV/current MoveIt path or response envelope failed: '+json.dumps({k:v for k,v in reserve.items() if k!='rows'}))
  return result

 def audit_nominal_sequence(self,check_model=True):
  self.prepared=False
  predicted=self.source_q.copy();proof=[]
  for s in self.sequence:
   proof.append(self._evaluate(predicted,s['target_sdk'],s['axis'],predicted.copy(),check_model))
   predicted[s['axis']]=s['target_sdk'][s['axis']]
  self.prepared=True
  return dict(kind='readonly_all_alternating_steps_probes_reverse_return',robot_commands_published=0,
   contract=self.contract,proof_steps=proof,all_pass=all(r['all_pass'] for r in proof),
   predicted_endpoints_are_not_measured=True,actual_submission_revalidation_required=True)

 def begin_step(self,index,actual,current_sdk):
  if not self.prepared or self.active is not None or index!=self.cursor: raise RuntimeError('step missing audit, skipped, repeated, or already active')
  s=self.sequence[index];current=vector(current_sdk)
  if len(current)!=7 or base.words(current)!=s['before_words'] or current[6]!=self.reference[6]: raise RuntimeError('current SDK differs from frozen previous step')
  a=self.check_actual(actual)
  self.active=dict(index=index,start=a.copy())
  return s

 def check_during_step(self,actual):
  a=self.check_actual(actual)
  if self.active is None:return a
  s=self.sequence[self.active['index']];start=self.active['start'];axis=s['axis'];held=[i for i in range(6) if i!=axis]
  if np.max(abs(a[held]-start[held]))>math.radians(DRIFT_DEG)+1e-12:raise RuntimeError('per-step unedited actual drift exceeds0.5deg')
  goal=s['target_sdk'][axis];direction=math.copysign(1.,goal-start[axis]) if goal!=start[axis] else 0.;reserved=goal+direction*math.radians(EXTRA_DEG)
  low=min(start[axis],goal,reserved);high=max(start[axis],goal,reserved)
  if not low-1e-12<=a[axis]<=high+1e-12:raise RuntimeError('actual outside fixed step response bounds')
  return a

 def guard_segment(self,actual,target):
  if self.active is None:raise RuntimeError('no active bound step')
  s=self.sequence[self.active['index']];b=vector(target)
  if len(b)!=7 or base.words(b)!=s['target_words'] or b[6]!=self.reference[6]:raise RuntimeError('submission differs from frozen current step SDK words')
  return self._evaluate(actual,b,s['axis'],self.active['start'],True)

 def complete_step(self,actual,current_sdk):
  if self.active is None:raise RuntimeError('no step to complete')
  self.check_during_step(actual);s=self.sequence[self.active['index']];q=vector(current_sdk)
  if len(q)!=7 or base.words(q)!=s['target_words'] or q[6]!=self.reference[6]:raise RuntimeError('measured SDK did not reach exact frozen words')
  self.active=None;self.cursor+=1

def bound_publish_target(plan, action, proposed, edited_index):
 if action not in ('none','wait','publish'):
  raise RuntimeError('already-CONFIRMED experiment forbids recovery/enable/sync/probe action: '+str(action))
 if action!='publish':return None
 if plan.active is None:raise RuntimeError('publish without active step')
 step=plan.sequence[plan.active['index']]
 q=vector(proposed)
 if edited_index!=step['axis'] or base.words(q)[edited_index]!=step['target_words'][edited_index]:
  raise RuntimeError('AutoMotion edited axis/target differs from frozen step')
 # AutoMotion forms held entries from feedback; preserve original/frozen SDK
 # words explicitly instead, including previously staged J2/J5 values.
 return np.array(step['target_sdk'],dtype=float)

def prepare_executor_audit(directory,cycles=7,j2_probe_sign=1):
 import rospy
 from sensor_msgs.msg import JointState
 from moveit_msgs.msg import RobotState
 from moveit_msgs.srv import GetStateValidity,GetStateValidityRequest
 directory=pathlib.Path(directory);directory.mkdir(parents=True,exist_ok=True)
 snapshot,xml=base.capture(directory)
 if snapshot['task_active'] or snapshot['protection_latched'] or not snapshot['feedback_ready'] or not snapshot['gui_direct_mode'] or not snapshot['actuation_status'].startswith('CONFIRMED:'):
  raise RuntimeError('fresh task/protection/feedback/manual/CONFIRMED evidence unavailable')
 rospy.wait_for_service('/check_state_validity',timeout=3);proxy=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
 def valid(q,g):
  request=GetStateValidityRequest(group_name='alicia');request.robot_state=RobotState();request.robot_state.joint_state=JointState(name=base.NAMES,position=list(q)+[g]);return proxy(request).valid
 obj=HighPostureAudit(snapshot,xml,valid,cycles,j2_probe_sign)
 proof=obj.audit_nominal_sequence();base.write(directory/'factory_audit_proof.json',proof);base.write(directory/'contract.json',obj.contract)
 (directory/'factory_source.py').write_bytes(pathlib.Path(__file__).read_bytes())
 return obj

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--cycles',type=int,choices=(5,7,8),default=7);parser.add_argument('--j2-probe-sign',type=int,choices=(-1,1),default=1);args=parser.parse_args()
 if not pathlib.Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('SanStand unavailable')
 directory=OUT/time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());directory.mkdir(parents=True);(directory/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(directory/'ros')
 import rospy
 rospy.init_node('gen4_high_posture_readonly_audit',anonymous=True,disable_signals=True)
 signal.signal(signal.SIGALRM,lambda *_:(_ for _ in ()).throw(RuntimeError('readonly audit deadline')));signal.alarm(240)
 try:
  obj=prepare_executor_audit(directory,args.cycles,args.j2_probe_sign);base.write(OUT/'latest.json',dict(directory=str(directory),source_stamp_ns=obj.source_stamp_ns));print(json.dumps(dict(directory=str(directory),source_stamp_ns=obj.source_stamp_ns,steps=len(obj.sequence),contract=obj.contract)))
 except BaseException as exc:base.write(directory/'failure.json',dict(error=str(exc),robot_commands_published=0));raise
 finally:signal.alarm(0)
if __name__=='__main__':main()
