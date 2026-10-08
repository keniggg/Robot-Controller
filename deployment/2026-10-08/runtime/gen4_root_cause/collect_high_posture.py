#!/usr/bin/env python3
"""DRAFT: bounded recorded high-posture experiment. Never run without root review.

Requires --execute and immutable reviewed factory SHA. No enable/recovery,
calibration change, disable command, or automatic return after a failed gate.
"""
import argparse,datetime,hashlib,importlib.util,json,math,signal,time
from pathlib import Path
import numpy as np,rospy
from std_msgs.msg import Bool
from alicia_flexible_grasp_supervisor.msg import GraspState
from collect_single_axis import Recorder,AutoMotion,MOUNT,OUTROOT

class BoundRecorder(Recorder):
 def __init__(self,out):
  self.latest_inputs={}
  self.wait_guard_callback=None
  super().__init__(out)
 def receive(self,topic,msg):
  super().receive(topic,msg)
  if self.active:self.latest_inputs[topic]=(msg,rospy.Time.now().to_nsec())
 def guard(self):
  # Shared Recorder.snapshot() calls self.guard() throughout its wait. Bind the
  # full experiment guard here so a snapshot wait never masks authority loss.
  super().guard()
  callback=getattr(self,'wait_guard_callback',None)
  if callback is not None:callback()
 def joint_and_sdk(self):
  with self.lock:
   if not self.states:raise RuntimeError('no validated sdk_measured feedback')
   ns,q=self.states[-1]
   if not 0<=(rospy.Time.now().to_nsec()-ns)*1e-9<=.3:raise RuntimeError('validated feedback stale')
   sdk=self.latest_inputs.get('/alicia_d/sdk_command');reference=self.latest_inputs.get('/alicia_d/control_reference')
   if sdk is None or reference is None:raise RuntimeError('SDK/reference evidence unavailable')
   m=sdk[0];ref=reference[0]
   if m.header.frame_id!='sdk_transmitted' or ref.header.frame_id!='sdk_manual':raise RuntimeError('SDK/reference provenance or manual ownership changed')
   if abs(m.header.stamp.to_nsec()-ref.header.stamp.to_nsec())>300000000:raise RuntimeError('SDK/reference source mismatch')
   names=dict(zip(m.name,m.position));refs=dict(zip(ref.name,ref.position))
   if len(names)!=len(m.name) or any('Joint%d'%i not in names for i in range(1,7)) or 'right_finger' not in refs:raise RuntimeError('malformed SDK/reference vector')
   if not 0<=(rospy.Time.now().to_nsec()-m.header.stamp.to_nsec())*1e-9<=.5:raise RuntimeError('SDK evidence stale')
   wire=np.array([names['Joint%d'%i] for i in range(1,7)]+[refs['right_finger']],float)
   if not np.isfinite(wire).all():raise RuntimeError('SDK/reference nonfinite')
   return np.asarray(q,float),wire,int(m.header.stamp.to_nsec())
 def source_guard(self,plan):
  # Direct base call avoids recursion when guard() invokes this method during
  # Recorder.snapshot(). The callback adds motion checks only at the outer edge.
  super().guard()
  with self.lock:
   if self.epoch_ns!=plan.snapshot['epoch_ns']:raise RuntimeError('frozen source epoch changed')
   ready=self.latest_inputs.get('/alicia_d/feedback_ready');status=self.latest_inputs.get('/alicia_d/actuation_status')
   if ready is None or not ready[0].data or status is None or not status[0].data.startswith('CONFIRMED:'):raise RuntimeError('feedback/CONFIRMED authority lost; no bootstrap permitted')
   pairs=[ns for ns,pair in self.frames.items() if len(pair)==2]
   if not pairs or not 0<=(rospy.Time.now().to_nsec()-max(pairs))*1e-9<=.5:raise RuntimeError('fresh paired camera source unavailable')
  q,sdk,stamp=self.joint_and_sdk();plan.check_during_step(q)
  if plan.active is not None:
   step=plan.sequence[plan.active['index']];words=plan_module_words(plan,sdk);axis=step['axis']
   if any(words[i]!=step['before_words'][i] for i in range(6) if i!=axis):raise RuntimeError('unedited SDK word drift during active step')
   lo=min(step['before_words'][axis],step['target_words'][axis]);hi=max(step['before_words'][axis],step['target_words'][axis])
   if not lo<=words[axis]<=hi:raise RuntimeError('profile SDK word outside frozen transition')
  return q,sdk,stamp

def plan_module_words(plan,q):
 # Factory owns quantization used by the driver; same constants as frozen audit.
 return [int(math.floor((v+math.pi)/(2*math.pi/4096)+.5)) for v in q[:6]]

def verify_constructed_source(plan,actual,current_sdk,motion_q):
 actual=np.asarray(actual,float);sdk=np.asarray(current_sdk,float);motion_q=np.asarray(motion_q,float)
 if any(q.shape!=(7,) or not np.isfinite(q).all() for q in (actual,sdk,motion_q)):
  raise RuntimeError('invalid post-construction source vectors')
 if np.max(abs(actual[:6]-plan.source_q))>math.radians(.2) or np.max(abs(motion_q[:6]-plan.source_q))>math.radians(.2):
  raise RuntimeError('source posture changed during AutoMotion construction')
 if plan_module_words(plan,sdk)!=plan_module_words(plan,plan.reference) or sdk[6]!=plan.reference[6]:
  raise RuntimeError('original SDK changed during AutoMotion construction')

def wait_profile_final(recorder,motion,plan,goal,after_ns,timeout=18.):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  motion.guard();q,sdk,stamp=recorder.source_guard(plan)
  if stamp>after_ns and plan_module_words(plan,sdk)==plan_module_words(plan,goal) and sdk[6]==goal[6]:
   recorder.mark('sdk_profile_final_words_observed',sdk_stamp_ns=stamp,final_words=plan_module_words(plan,sdk),q=q.tolist())
   return rospy.Time.now().to_nsec()
  time.sleep(.04)
 raise RuntimeError('SDK profile did not reach exact frozen final words; no next command')

def guarded_delay(seconds,guard):
 deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:guard();time.sleep(min(.05,max(0,deadline-time.monotonic())))

def audit_current_submission(recorder,motion,plan,wire,maximum_attempts=3):
 """Re-audit changed actual feedback; never authorize a stale proof.

 A quantum arriving while MoveIt replies previously aborted a quiet return.
 Each retry starts from new feedback, retains the same frozen target, and
 repeats the whole geometric audit. No angle/freshness tolerance is widened.
 """
 if maximum_attempts not in (1,2,3):raise RuntimeError('bounded submission audit requires1..3attempts')
 step=plan.sequence[plan.active['index']]
 for attempt in range(1,maximum_attempts+1):
  motion.guard();q,sdk,_=recorder.source_guard(plan)
  if plan_module_words(plan,sdk)!=step['before_words'] or sdk[6]!=plan.reference[6]:
   raise RuntimeError('SDK changed before actual publication')
  proof=plan.guard_segment(q,wire)
  motion.guard();fresh,fresh_sdk,_=recorder.source_guard(plan)
  if plan_module_words(plan,fresh_sdk)!=step['before_words'] or fresh_sdk[6]!=plan.reference[6]:
   raise RuntimeError('SDK changed during submission audit; no publication')
  if np.array_equal(fresh[:6],q[:6]):
   recorder.mark('actual_submission_geometry',step=step['index'],target=wire.tolist(),audit=proof,audit_attempt=attempt)
   return proof
  recorder.mark('submission_audit_discarded',step=step['index'],attempt=attempt,
                audited_q=q.tolist(),fresh_q=fresh.tolist(),reason='actual feedback changed; full reaudit required')
 raise RuntimeError('actual q kept changing during3submission audits; no publication')

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--plan-module',required=True);ap.add_argument('--expected-plan-sha256',required=True);ap.add_argument('--cycles',type=int,choices=(5,7,8),default=7);ap.add_argument('--j2-probe-sign',type=int,choices=(-1,1),default=1);ap.add_argument('--output-prefix',choices=('high_posture','orthogonal_probe'),default='high_posture');ap.add_argument('--execute',action='store_true');args=ap.parse_args()
 if not args.execute or not MOUNT.is_mount():raise RuntimeError('explicit execution and mounted evidence drive required')
 path=Path(args.plan_module).resolve();source=path.read_bytes()
 if hashlib.sha256(source).hexdigest()!=args.expected_plan_sha256:raise RuntimeError('factory differs from reviewed SHA')
 rospy.init_node('support_high_posture_experiment',anonymous=True,disable_signals=True)
 if rospy.wait_for_message('/grasp/state',GraspState,timeout=3).active:raise RuntimeError('grasp task active')
 spec=importlib.util.spec_from_file_location('support_high_posture_factory',str(path));module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 out=OUTROOT/(args.output_prefix+'_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir()
 recorder=None
 def interrupted(signum,_frame):raise RuntimeError('diagnostic interrupted signal %s'%signum)
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGALRM):signal.signal(sig,interrupted)
 signal.alarm(480)
 try:
  recorder=BoundRecorder.__new__(BoundRecorder);BoundRecorder.__init__(recorder,out)
  plan=module.prepare_executor_audit(out/'fresh_plan',args.cycles,args.j2_probe_sign)
  if hashlib.sha256(path.read_bytes()).hexdigest()!=args.expected_plan_sha256:raise RuntimeError('factory source changed during preparation')
  recorder.wait_guard_callback=lambda:recorder.source_guard(plan)
  recorder.source_guard(plan)
  (out/'execution_inputs.json').write_text(json.dumps(dict(factory_sha256=args.expected_plan_sha256,collector_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),contract=plan.contract,total_wall_deadline_sec=480,static_snapshot_min_delay_sec=2,commands_after_failure=0),indent=2)+'\n')
  recorder.snapshot('baseline',plan.source_stamp_ns)
  q,sdk,_=recorder.source_guard(plan)
  if np.max(abs(q[:6]-plan.source_q))>math.radians(.2):raise RuntimeError('source posture changed after factory audit')
  if module.base.words(sdk)!=module.base.words(plan.reference):raise RuntimeError('source SDK changed after factory audit')
  # Factory requires already-confirmed manual ownership. Do not change mode,
  # reset epoch, or issue enable/sync/probe requests to make this experiment run.
  motion=AutoMotion(out);original_guard=motion.guard
  # Constructing/subscribing may wait several seconds. Freeze neither a changed
  # posture nor changed command words as the new experiment baseline.
  q_after_construct,sdk_after_construct,_=recorder.source_guard(plan)
  verify_constructed_source(plan,q_after_construct,sdk_after_construct,motion.q)
  def guard():
   original_guard();recorder.source_guard(plan)
  motion.guard=guard
  recorder.wait_guard_callback=guard
  original_apply=motion.apply
  def apply(decision):
   guard();action,target,detail=decision
   wire=module.bound_publish_target(plan,action,target,motion.recovery.last_action_edited_index)
   if wire is None:return action
   audit_current_submission(recorder,motion,plan,wire)
   return original_apply((action,wire.tolist(),detail))
  motion.apply=apply;recorder.armed=True;guard()
  for step in plan.sequence:
   guard();q,sdk,_=recorder.source_guard(plan)
   plan.begin_step(step['index'],q,sdk);goal=np.array(step['target_sdk']);axis=step['axis']
   delta=math.degrees(goal[axis]-motion.q[axis])
   if not .01<=abs(delta)<=4.:raise RuntimeError('frozen target actual-relative step outside0.01..4deg')
   sent_after=rospy.Time.now().to_nsec();recorder.mark('step_begin',step=step['index'],label=step['label'],phase=step['phase'],axis=axis,delta_from_actual_deg=delta,target_sdk=goal.tolist(),q=q.tolist())
   motion.move(axis,delta)
   final_ns=wait_profile_final(recorder,motion,plan,goal,sent_after)
   # A stationary feedback plateau before a new profile finishes is insufficient.
   guarded_delay(2.,guard)
   recorder.snapshot(step['label'],final_ns+2000000000)
   q,sdk,_=recorder.source_guard(plan);plan.complete_step(q,sdk)
   recorder.mark('step_complete',step=step['index'],label=step['label'],q=q.tolist(),sdk=sdk.tolist(),measured_vs_original_deg=np.degrees(q[:6]-plan.source_q).tolist())
  late=rospy.Time.now().to_nsec();guarded_delay(2.,guard);recorder.snapshot('return_late',late+2000000000)
  q,sdk,_=recorder.source_guard(plan)
  if module.base.words(sdk)!=module.base.words(plan.reference):raise RuntimeError('whole sequence did not restore original SDK words')
  recorder.mark('sequence_complete',q=q.tolist(),actual_return_error_deg=np.degrees(q[:6]-plan.source_q).tolist(),exact_original_sdk_words=True)
 except BaseException as exc:
  if recorder is not None:
   try:recorder.mark('sequence_failed',reason=str(exc),automatic_return_attempted=False)
   except Exception:pass
  raise
 finally:
  signal.alarm(0)
  if recorder is not None:recorder.close()
if __name__=='__main__':main()
