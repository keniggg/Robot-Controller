#!/usr/bin/env python3
"""J5 negative six-degree same-SDK-word out/back diagnostic; no calibration edits."""
import argparse,datetime,hashlib,importlib.util,json,math,signal,time
from pathlib import Path
import numpy as np,rospy
from std_msgs.msg import Bool
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp_supervisor.msg import GraspState
from collect_single_axis import Recorder,AutoMotion,MOUNT,OUTROOT

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--plan-module',required=True);ap.add_argument('--execute',action='store_true');args=ap.parse_args()
 if not args.execute or not MOUNT.is_mount():raise RuntimeError('explicit execution and mounted evidence drive required')
 rospy.init_node('support_j5_absolute_experiment',anonymous=True,disable_signals=True)
 if rospy.wait_for_message('/grasp/state',GraspState,timeout=3).active:raise RuntimeError('task active')
 spec=importlib.util.spec_from_file_location('support_j5_plan',args.plan_module);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 out=OUTROOT/('j5_negative6_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir()
 recorder=None
 def interrupted(signum,_frame):raise RuntimeError('diagnostic interrupted by signal %s'%signum)
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGALRM):signal.signal(sig,interrupted)
 signal.alarm(180)
 try:
  recorder=Recorder.__new__(Recorder);Recorder.__init__(recorder,out)
  plan=module.prepare_executor_audit(out/'fresh_plan')
  seed=np.asarray(plan.source_q,dtype=float);reference=np.asarray(plan.snapshot['sdk_q_rad'],dtype=float)
  goals=np.asarray(plan.absolute_sdk_targets,dtype=float)
  if seed.shape!=(6,) or reference.shape!=(7,) or goals.shape!=(4,7) or not np.isfinite(np.r_[seed,reference,goals.ravel()]).all():raise RuntimeError('invalid absolute plan inputs')
  if not np.array_equal(goals[-1],reference):raise RuntimeError('final target must restore initial SDK values')
  held=[0,1,2,3,5,6]
  if not np.array_equal(goals[:,held],np.tile(reference[held],(4,1))):raise RuntimeError('non-J5 goal changed')
  evidence={'source_q':seed.tolist(),'initial_sdk_q':reference.tolist(),'absolute_sdk_targets':goals.tolist(),'return_semantics':'restore initial SDK words; report actual and visual return without assuming exact measured q return','plan_module':str(Path(args.plan_module).resolve()),'plan_module_sha256':hashlib.sha256(Path(args.plan_module).read_bytes()).hexdigest(),'collector_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
  (out/'execution_inputs.json').write_text(json.dumps(evidence,indent=2)+'\n')
  recorder.snapshot('baseline',plan.source_stamp_ns);recorder.guard()
  owner=rospy.Publisher('/gui/joint_direct_mode',Bool,queue_size=1,latch=True)
  rospy.set_param('/gui/joint_direct_mode',True);time.sleep(.3);owner.publish(Bool(data=True));time.sleep(.4)
  motion=AutoMotion(out)
  if np.max(abs(np.asarray(motion.q[:6])-seed))>math.radians(.2):raise RuntimeError('current posture changed since plan')
  original_guard=motion.guard
  def guard():recorder.guard();original_guard()
  motion.guard=guard
  # Actual submission checks below include response reservation. AutoMotion's
  # nominal move still checks its sampled collision segment, but its virtual
  # reserved target is deliberately not presented as an actual SDK command.
  original_apply=motion.apply
  def apply(decision):
   guard();action,target,detail=decision
   if action=='enable':recorder.allow_epoch_until=time.monotonic()+5
   if action in ('sync','probe','publish'):
    wire=reference.copy()
    edited=motion.recovery.last_action_edited_index
    if action!='sync':
     if edited!=4:raise RuntimeError('handshake attempted to edit another joint')
     wire[4]=float(target[4])
    t=recorder.buffer.lookup_transform('tool0','camera_link',rospy.Time(0),rospy.Duration(.2)).transform
    X=quaternion_matrix([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]);X[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    if not np.allclose(X,plan.snapshot['T_tool_camera'],rtol=0,atol=1e-9):raise RuntimeError('handeye changed')
    camera=rospy.get_param('/camera')
    if camera['depth_scale']!=recorder.camera['depth_scale']:raise RuntimeError('depth scale changed')
    audit=plan.guard_segment(np.asarray(motion.q[:6]),wire[:6])
    recorder.mark('actual_submission_geometry',kind=action,target=wire.tolist(),audit=audit)
    guard();return original_apply((action,wire.tolist(),detail))
   return original_apply(decision)
  motion.apply=apply;recorder.armed=True
  for i,goal in enumerate(goals):
   guard();delta=math.degrees(goal[4]-motion.q[4])
   if abs(delta)>4.:raise RuntimeError('next absolute SDK goal exceeds bounded actual relative step')
   recorder.mark('step_begin',step=i+1,joint=5,delta_from_actual_deg=delta,absolute_sdk_target=goal.tolist(),q=motion.q)
   if abs(delta)>=.01:motion.move(4,delta)
   settled=rospy.Time.now().to_nsec();time.sleep(2.)
   recorder.snapshot(['negative3','negative6','return_negative3','return_sdk_start'][i],settled)
   recorder.mark('measured_endpoint',step=i+1,q=motion.q,sdk_goal=goal.tolist(),measured_vs_goal_deg=np.degrees(np.asarray(motion.q[:6])-goal[:6]).tolist())
  # Longer endpoint control distinguishes decaying vibration from held drift.
  late=rospy.Time.now().to_nsec();time.sleep(3);recorder.snapshot('return_late',late)
  recorder.mark('sequence_complete',final_q=motion.q,actual_return_error_deg=np.degrees(np.asarray(motion.q[:6])-seed).tolist())
 except BaseException as exc:
  if recorder is not None:
   try:recorder.mark('sequence_failed',reason=str(exc))
   except Exception:pass
  raise
 finally:
  signal.alarm(0)
  if recorder is not None:recorder.close()
if __name__=='__main__':main()
