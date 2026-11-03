#!/usr/bin/env python3
import sys,time,json,math,pathlib,subprocess,numpy as np,rospy
from std_msgs.msg import String,Header
from easy_handeye_msgs.srv import TakeSample
from handeye_auto_motion import AutoMotion
from plan_task_pose_sweep import config,out,geometry,segment,seed
rospy.init_node('automatic_task_pose_validation',anonymous=True,disable_signals=True)
motion=AutoMotion(out);motion.task_pose_directional_reservation=True;latest={}
def quality(msg):
 try:
  latest.clear();latest.update(json.loads(msg.data));latest['received']=time.monotonic()
 except (ValueError,TypeError):pass
subscriber=rospy.Subscriber('/charuco/quality',String,quality,queue_size=1)
def await_board():
 deadline=time.monotonic()+5
 while time.monotonic()<deadline:
  motion.guard()
  if latest and time.monotonic()-latest.get('received',0)<1.0 and latest.get('pose_ok') and latest.get('charuco_count',0)>=35 and latest.get('edge_clearance_px',0)>=12:return
  time.sleep(.05)
 raise RuntimeError('current board observation unavailable or incomplete')
def guard(target):
 ok,details=segment(np.array(motion.q[:6]),np.array(target[:6]))
 if not ok:raise RuntimeError('local segment outside reserved board clearance/view: '+json.dumps(details))
motion.command_guard=guard
plan_path=pathlib.Path(sys.argv[2]) if len(sys.argv)>2 else out/'prospective_flexion_sweep.json';plan=json.loads(plan_path.read_text());sample=rospy.ServiceProxy(config['namespace']+'/take_sample',TakeSample)
start_id=int(sys.argv[1]) if len(sys.argv)>1 else 1
if start_id==1 and max(abs(np.array(motion.q[:6])-seed))>.02:raise RuntimeError('pose changed since capture002')
motion.emit('task_pose_validation_begin',source_plan=str(plan_path),frozen_handeye_sha256=config['frozen_yaml_sha256'],no_refit=True,start_pose_id=start_id)
try:
 for p in plan['poses']:
  if p['pose_id']<start_id:continue
  goal=np.array(p['q_rad']);motion.emit('validation_pose_begin',pose_id=p['pose_id'],role=p['role'],goal_deg=np.degrees(goal).tolist())
  steps=0
  while max(abs(goal-np.array(motion.q[:6])))>math.radians(2.5):
   if steps>=80:raise RuntimeError('bounded step budget exhausted')
   await_board();q=np.array(motion.q[:6]);remaining=np.degrees(goal-q);options=[]
   for j in range(6):
    if abs(remaining[j])<=2.5:continue
    for max_step in ([3.] if j in (1,2) and remaining[j]<0 else [4.,3.]):
     delta=float(np.clip(remaining[j],-max_step,max_step));target=q.copy();target[j]+=math.radians(delta);ok,info=segment(q,target)
     reserve=target.copy();reserve[j]=q[j]+math.copysign(math.radians(4.8),delta);reserved_ok,reserved_info=segment(q,reserve)
     if ok and reserved_ok:
      progress=remaining[j]**2-(remaining[j]-delta)**2
      options.append((progress+.03*min(info['edge_px'],80),j,delta,info))
   if not options:raise RuntimeError('No locally admissible next joint step toward prospective pose')
   _,j,delta,info=max(options,key=lambda x:x[0]);motion.emit('bounded_step_selected',pose_id=p['pose_id'],joint=j+1,delta_deg=delta,geometry=info)
   motion.move(j,delta);steps+=1
  await_board();accepted=False
  active=pathlib.Path(config.get('active_sample_session',str(out)))
  recorded_epoch=json.loads((active/'session.json').read_text()).get('initial_epoch')
  current_epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=3).stamp.to_nsec()
  if recorded_epoch is not None and current_epoch!=recorded_epoch:
   motion.emit('sampling_epoch_transition',pose_id=p['pose_id'],previous_epoch_ns=recorded_epoch,current_epoch_ns=current_epoch)
   subprocess.run(['/usr/bin/python3',str(pathlib.Path('.runtime/rotate_task_pose_sampler.py').resolve()),str(p['pose_id'])],check=True)
   updated=json.loads(pathlib.Path('.runtime/current_handeye_task_pose_validation.json').read_text());config.clear();config.update(updated)
   sample=rospy.ServiceProxy(config['namespace']+'/take_sample',TakeSample)
   motion.emit('pose_recorded_as_new_epoch_reference',pose_id=p['pose_id'],sample_session=config['active_sample_session']);continue
  for attempt in range(3):
   motion.guard();time.sleep(.5)
   try:
    response=sample();motion.emit('validation_sample_accepted',pose_id=p['pose_id'],role=p['role'],count=len(response.samples.hand_world_samples),attempt=attempt+1,actual_q_deg=np.degrees(motion.q[:6]).tolist(),sample_session=config.get('active_sample_session',str(out)));accepted=True;break
   except rospy.ServiceException as exc:
    motion.emit('validation_sample_rejected',pose_id=p['pose_id'],attempt=attempt+1,reason=str(exc))
    active=pathlib.Path(config.get('active_sample_session',str(out)));manifest=json.loads((active/'session.json').read_text());entry=manifest['captures'][-1] if manifest.get('captures') else {}
    if 'return' in p['role'] and entry.get('rejection','').startswith('与已有姿态差异不足'):
     motion.emit('validation_return_window_recorded',pose_id=p['pose_id'],role=p['role'],capture=entry['directory'],sample_session=str(active),admitted_as_novel_gui_sample=False);accepted=True;break
  if not accepted:raise RuntimeError('sampling window rejected three times')
 motion.emit('task_pose_validation_sequence_complete',frozen_handeye_sha256=config['frozen_yaml_sha256'],source_plan=str(plan_path),pose_ids=[item['pose_id'] for item in plan['poses'] if item['pose_id']>=start_id])
 terminal=dict(status='sequence_complete',at=time.time(),first_pose_id=start_id)
except Exception as exc:
 terminal=dict(status='interrupted',at=time.time(),reason=str(exc),pose_id=p['pose_id'] if 'p' in globals() else None);motion.emit('task_pose_validation_interrupted',**terminal);raise
finally:
 (out/('task_pose_sequence_terminal_'+str(time.time_ns())+'.json')).write_text(json.dumps(terminal,indent=2)+'\n')
